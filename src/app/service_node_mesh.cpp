// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/service_node_mesh.cpp
/// \brief Implements the service node mesh component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/service_node.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace mrs_uav_bluetooth::app {

namespace {

// Keep the on-air reliability header small. The normal bridge frame is
// reconstructed on receive so the bridge codec remains transport-independent.
constexpr size_t kMeshVendorPrefixBytes = 3;
constexpr size_t kBridgeFramePrefixBytes = 5;
constexpr size_t kReliableDataHeaderBytes = 10;
constexpr size_t kReliableAckHeaderBytes = 8;
constexpr uint8_t kReliableData = 0xd1;
constexpr uint8_t kReliableDataNoTurn = 0xd4;
constexpr uint8_t kReliableAck = 0xd2;
constexpr uint8_t kReliableReceiptQuery = 0xd3;
constexpr auto kReceiptQueryPeriod = std::chrono::seconds(2);
constexpr auto kMeshQueueRetryDelay = std::chrono::milliseconds(250);
// This is a lost-token failsafe, not a normal scheduling quantum. A segmented
// unicast can still be draining native SAR and queued access acknowledgements
// several seconds after its receiver handed the turn onward. Reclaiming inside
// that interval creates two token holders and makes the lowest address starve
// later ring members. Keep recovery below the 20 s default membership lease,
// but above the longest healthy transaction observed on the target radios.
constexpr auto kMeshTurnRecovery = std::chrono::seconds(12);
constexpr size_t kMaxPendingMeshSamples = 32;
constexpr size_t kMaxQueuedMeshAcks = 128;
constexpr uint8_t kMeshTtlConfigured = 1U;
constexpr uint8_t kMeshRelayConfigured = 2U;
constexpr uint8_t kMeshNetworkTransmitConfigured = 4U;
constexpr uint8_t kMeshRadioConfigured =
    kMeshTtlConfigured | kMeshRelayConfigured | kMeshNetworkTransmitConfigured;

/// \brief Allocate the next nonzero transfer sequence and persist rollover state.
/// \param data Reliable Mesh frame containing the sequence header.
/// \return Transfer sequence decoded from the reliability header.
uint16_t mesh_reliable_sequence(const std::vector<uint8_t>& data) {
    // Allocate the next nonzero transfer sequence and persist rollover state.
    return static_cast<uint16_t>(data[4]) |
        (static_cast<uint16_t>(data[5]) << 8U);
}

/// \brief Encode the transfer sequence into the reliability header.
/// \param data Reliable Mesh header buffer receiving the sequence number.
/// \param sequence nonzero transfer sequence identifying one reliable Mesh message.
void append_mesh_reliable_sequence(std::vector<uint8_t>& data, uint16_t sequence) {
    // Mesh reliability headers encode integers least-significant byte first.
    data.push_back(static_cast<uint8_t>(sequence));
    data.push_back(static_cast<uint8_t>(sequence >> 8U));
}

/// \brief Hash the admitted Mesh membership into the reliable-frame network identifier.
/// \param data Reliable Mesh frame containing the swarm identifier.
/// \return Logical group identifier decoded from the reliability header.
uint16_t mesh_reliable_swarm_id(const std::vector<uint8_t>& data) {
    // Hash the admitted Mesh membership into the reliable-frame network identifier.
    return static_cast<uint16_t>(data[6]) |
        (static_cast<uint16_t>(data[7]) << 8U);
}

/// \brief Encode the logical group identifier into the reliability header.
/// \param data Reliable Mesh header buffer receiving the swarm identifier.
/// \param swarm_id Logical Mesh group encoded into the reliability header.
void append_mesh_reliable_swarm_id(
    std::vector<uint8_t>& data, uint16_t swarm_id) {
    // Use the same little-endian layout as the sequence field beside it.
    data.push_back(static_cast<uint8_t>(swarm_id));
    data.push_back(static_cast<uint8_t>(swarm_id >> 8U));
}

/// \brief Frame one payload fragment with transfer identity, ordering, and checksum metadata.
/// \param payload Encoded bridge payload to wrap in the reliability header.
/// \param sequence nonzero transfer sequence identifying one reliable Mesh message.
/// \param swarm_id Logical Mesh group encoded into the reliability header.
/// \return Bridge payload prefixed with marker sequence and group identifiers.
std::vector<uint8_t> make_mesh_reliable_data(
    const std::vector<uint8_t>& payload,
    uint16_t sequence,
    uint16_t swarm_id) {
    // Frame one payload fragment with transfer identity, ordering, and checksum metadata.
    std::vector<uint8_t> wrapped(
        payload.begin(), payload.begin() + kMeshVendorPrefixBytes);
    wrapped.reserve(payload.size() + 2);
    wrapped.push_back(kReliableDataNoTurn);
    append_mesh_reliable_sequence(wrapped, sequence);
    append_mesh_reliable_swarm_id(wrapped, swarm_id);
    wrapped.push_back(payload[6]);
    wrapped.push_back(payload[7]);
    wrapped.insert(wrapped.end(),
                   payload.begin() + kMeshVendorPrefixBytes +
                       kBridgeFramePrefixBytes, payload.end());
    return wrapped;
}

/// \brief Return the ordered peer addresses eligible for the current radio turn.
/// \param local Local Mesh unicast address included in turn arbitration.
/// \param peers Currently present remote Mesh unicast addresses.
/// \return Sorted unique nonzero local and peer addresses participating in arbitration.
std::vector<uint16_t> mesh_turn_members(
    uint16_t local, std::vector<uint16_t> peers) {
    // Return the ordered peer addresses eligible for the current radio turn.
    peers.push_back(local);
    std::sort(peers.begin(), peers.end());
    peers.erase(std::unique(peers.begin(), peers.end()), peers.end());
    return peers;
}

/// \brief Recognize this package's reliable Mesh data and control frames.
/// \param data Mesh access payload to validate as a reliability frame.
/// \return True if the payload contains this package's complete reliability header; otherwise false.
bool is_mesh_reliable_frame(const std::vector<uint8_t>& data) {
    // Require the shared header and one of the supported reliability opcodes.
    return data.size() >= kReliableAckHeaderBytes &&
        (data[3] == kReliableData || data[3] == kReliableDataNoTurn ||
         data[3] == kReliableAck ||
         data[3] == kReliableReceiptQuery);
}

template<typename DurationT, typename CallbackT>
/// \brief Create a wall timer bound to the Mesh callback group.
/// \param node ROS node that owns the interfaces.
/// \param period timer interval used to schedule the callback.
/// \param callback Timer body invoked on each scheduled expiry.
/// \param group ROS callback group that serializes the timer or service.
/// \return New mesh wall timer.
rclcpp::TimerBase::SharedPtr create_mesh_wall_timer(
    rclcpp::Node& node,
    DurationT period,
    CallbackT&& callback,
    const rclcpp::CallbackGroup::SharedPtr& group) {
    // Bind the mesh retry timer to its mutually exclusive callback group.
    return rclcpp::create_wall_timer(
        period, std::forward<CallbackT>(callback), group,
        node.get_node_base_interface().get(),
        node.get_node_timers_interface().get());
}

/// Release an on-demand Mesh bearer before conventional LE starts. The package
/// grants the mrs account narrowly scoped StopUnit permission for this unit.
/// \param connection D-Bus connection used to stop the local Mesh daemon bearer.
void stop_mesh_bearer(bluez::DbusConnection& connection) {
    auto bus = sdbus::createProxy(connection.connection(),
        sdbus::ServiceName{"org.freedesktop.DBus"},
        sdbus::ObjectPath{"/org/freedesktop/DBus"});
    const auto running = [&]() {
        // Query D-Bus ownership directly so shutdown waits for bluetooth-meshd rather than stale cache state.
        bool present = false;
        bus->callMethod("NameHasOwner").onInterface("org.freedesktop.DBus")
            .withArguments(std::string{"org.bluez.mesh"}).storeResultsTo(present);
        return present;
    };
    const bool was_running = running();
    auto manager = sdbus::createProxy(connection.connection(),
        sdbus::ServiceName{"org.freedesktop.systemd1"},
        sdbus::ObjectPath{"/org/freedesktop/systemd1"});
    sdbus::ObjectPath job;
    if (was_running) {
        manager->callMethod("StopUnit").onInterface("org.freedesktop.systemd1.Manager")
            .withArguments(std::string{"bluetooth-mesh.service"}, std::string{"replace"})
            .storeResultsTo(job);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (running()) {
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error(
                "Mesh controller was not released; install the service package "
                "and stop any manually launched bluetooth-meshd before enabling LE");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // Losing the Mesh bus name precedes bluetoothd re-exporting Adapter1.
    // Wait for that second boundary before attempting LE registration.
    while (connection.find_adapter_path().empty()) {
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("BlueZ Adapter1 did not return after Mesh shutdown");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // A MGMT receiver can outlive both the daemon and controller power cycles.
    // Run the narrowly privileged cleanup even on cold LE startup, to recover
    // state left by an earlier daemon version or a crash. Its unit is ordered
    // after Mesh shutdown, including ExecStopPost. Await its job, not merely
    // its bus-name disappearance, before any discovery/advertising operation.
    const std::string cleanup_unit{"mrs-uav-bluetooth-le-radio.service"};
    manager->callMethod("StartUnit").onInterface("org.freedesktop.systemd1.Manager")
        .withArguments(cleanup_unit, std::string{"replace"}).storeResultsTo(job);
    const auto job_path = std::string(job);
    const auto job_id = static_cast<uint32_t>(std::stoul(
        job_path.substr(job_path.find_last_of('/') + 1)));
    const auto cleanup_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    for (;;) {
        try {
            sdbus::ObjectPath current_job;
            manager->callMethod("GetJob").onInterface("org.freedesktop.systemd1.Manager")
                .withArguments(job_id).storeResultsTo(current_job);
        } catch (const sdbus::Error& error) {
            if (error.getName() == "org.freedesktop.systemd1.NoSuchJob") break;
            throw;
        }
        if (std::chrono::steady_clock::now() >= cleanup_deadline)
            throw std::runtime_error("Timed out releasing the kernel Mesh receiver");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    sdbus::ObjectPath unit_path;
    try {
        manager->callMethod("GetUnit").onInterface("org.freedesktop.systemd1.Manager")
            .withArguments(cleanup_unit).storeResultsTo(unit_path);
    } catch (const sdbus::Error& error) {
        // systemd garbage-collects successful inactive oneshots immediately.
        // Failed units are retained (the unit uses default CollectMode), so an
        // unloaded unit after this completed job means successful cleanup.
        if (error.getName() == "org.freedesktop.systemd1.NoSuchUnit") return;
        throw;
    }
    auto unit = sdbus::createProxy(connection.connection(),
        sdbus::ServiceName{"org.freedesktop.systemd1"}, unit_path);
    const auto result = unit->getProperty("Result")
        .onInterface("org.freedesktop.systemd1.Service").get<std::string>();
    if (result != "success")
        throw std::runtime_error("Mesh receiver cleanup failed (" + result +
            "); inspect journalctl -u " + cleanup_unit);
}

/// \brief Normalize ASCII configuration text to lowercase for case-insensitive parsing.
/// \param value Text to normalize.
/// \return Lowercase copy of the input text.
std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                       // Normalize ASCII configuration text to lowercase for case-insensitive parsing.
                       return static_cast<char>(std::tolower(ch));
                   });
    return value;
}

/// \brief Hash the network, keys, model, and membership that define compatible Mesh state.
/// \param config Mesh settings serialized to detect changes that require reconfiguration.
/// \return Stable serialization of Mesh settings that require runtime rebuild.
std::string mesh_config_signature(const config::NodeConfig& config) {
    // Hash the network, keys, model, and membership that define compatible Mesh state.
    std::ostringstream stream;
    stream << config.enable_mesh << ':' << config.mesh_provisioner << ':'
           << config.mesh_auto_configure_relay << ':'
           << unsigned(config.mesh_default_ttl) << ':'
           << unsigned(config.mesh_relay_retransmit_count) << ':'
           << unsigned(config.mesh_relay_retransmit_interval_steps) << ':'
           << unsigned(config.mesh_network_retransmit_count) << ':'
           << unsigned(config.mesh_network_retransmit_interval_steps) << ':'
           << config.mesh_auto_attach << ':' << config.mesh_auto_join << ':'
           << config.mesh_auto_create_network << ':'
           << config.mesh_swarm_auto_provisioning << ':'
           << config.mesh_swarm_id << ':'
           << config.mesh_swarm_participating << ':'
           << config.mesh_swarm_state_path << ':'
           << config.mesh_provisioner_preference << ':'
           << config.mesh_swarm_startup_grace << ':'
           << config.mesh_swarm_heartbeat_period << ':'
           << config.mesh_swarm_provisioner_timeout << ':'
           << config.mesh_swarm_group_address << ':'
           << config.mesh_swarm_network_index << ':'
           << config.mesh_swarm_app_key_index << ':'
           << config.mesh_unicast_cursor_path << ':'
           << config.mesh_device_uuid << ':' << std::hex << config.mesh_token << ':'
           << config.mesh_token_path << ':' << config.mesh_company_id << ':'
           << config.mesh_product_id << ':' << config.mesh_version_id << ':'
           << config.mesh_crpl << ':' << config.mesh_vendor_model_id << ':'
           << config.mesh_next_unicast << ':' << config.mesh_agent_numeric_oob << ':'
           << config.mesh_agent_uri;
    for (const auto& item : config.mesh_agent_capabilities) stream << ":cap=" << item;
    for (const auto& item : config.peer_whitelist) stream << ":peer=" << item;
    for (const auto& item : config.mesh_agent_oob_info) stream << ":oob=" << item;
    for (const auto byte : config.mesh_agent_static_oob) stream << ':' << unsigned(byte);
    for (const auto byte : config.mesh_agent_private_key) stream << ':' << unsigned(byte);
    for (const auto byte : config.mesh_agent_public_key) stream << ':' << unsigned(byte);
    return stream.str();
}

template<typename ResponseT>
/// \brief Copy current attachment token path addresses and state into a service response.
/// \param status Current Mesh lifecycle and addressing state copied into the service response.
/// \param response service response to populate.
void populate_network_status(const mesh::Status& status, ResponseT& response) {
    // Populate network status.
    response.state = status.state;
    response.token = status.token;
    response.node_path = status.node_path;
    response.addresses = status.addresses;
}

}  // namespace

void ServiceNode::configure_mesh(const config::NodeConfig& config) {
    mesh_local_network_transmit_expected_.store(static_cast<uint8_t>(
        config.mesh_network_retransmit_count |
        (config.mesh_network_retransmit_interval_steps << 3)));
    mesh_local_radio_retry_after_ns_.store(0);
    mesh_local_radio_confirmed_.store(0);
    mesh_local_ttl_expected_.store(config.mesh_default_ttl);
    mesh_local_relay_expected_.store(static_cast<uint8_t>(
        config.mesh_relay_retransmit_count |
        (config.mesh_relay_retransmit_interval_steps << 3)));
    mesh_next_maintenance_ = {};
    clear_mesh_reliable_state();
    const auto signature = mesh_config_signature(config);
    const bool config_changed = signature != mesh_config_signature_;
    const bool topic_changed = config.node_topics_prefix != mesh_topic_prefix_;

    if (!config.enable_mesh) {
        if (mesh_timer_) mesh_timer_->cancel();
        mesh_timer_.reset();
        mesh_swarm_coordinator_.reset();
        mesh_app_.reset();
        mesh_dbus_.reset();
        stop_mesh_bearer(*server_dbus_);
        mesh_status_pub_.reset();
        mesh_message_pub_.reset();
        mesh_event_pub_.reset();
        mesh_bootstrap_attempted_ = false;
        mesh_local_radio_confirmed_.store(0);
        mesh_config_signature_ = signature;
        mesh_topic_prefix_ = config.node_topics_prefix;
        return;
    }

    if (topic_changed || !mesh_status_pub_) {
        mesh_status_pub_ = create_publisher<mrs_uav_bluetooth::msg::MeshStatus>(
            config.node_topics_prefix + "/mesh/status", 10);
        mesh_message_pub_ = create_publisher<mrs_uav_bluetooth::msg::MeshMessage>(
            config.node_topics_prefix + "/mesh/rx", 100);
        mesh_event_pub_ = create_publisher<mrs_uav_bluetooth::msg::MeshEvent>(
            config.node_topics_prefix + "/mesh/events", 50);
        mesh_topic_prefix_ = config.node_topics_prefix;
    }

    if (config_changed || !mesh_app_) {
        if (mesh_timer_) mesh_timer_->cancel();
        mesh_timer_.reset();
        mesh_swarm_coordinator_.reset();
        mesh_app_.reset();
        mesh_dbus_.reset();
        mesh_dbus_ = std::make_unique<bluez::DbusConnection>(get_logger(), "mesh");
        mesh_app_ = std::make_unique<mesh::MeshApplication>(
            *mesh_dbus_, config, hostname_, get_logger());
        mesh_last_peer_heard_ns_.store(0);
        if (config.mesh_swarm_auto_provisioning) {
            mesh_swarm_coordinator_ = std::make_unique<mesh::MeshSwarmCoordinator>(
                *mesh_app_, config, hostname_, get_logger());
        }
        mesh_app_->set_message_callback(
            [this](const mesh::ReceivedMessage& message) {
                if (!can_run_callbacks()) return;
                const auto local = mesh_app_->status().addresses;
                const bool local_source =
                    std::find(local.begin(), local.end(), message.source) != local.end();
                // D-Bus send success only accepts the request. Advance setup
                // when the local Config Server reports the expected value.
                // Device-key and local-address checks exclude topic messages
                // and another node's configuration replies.
                uint8_t confirmed = 0;
                if (local_source && message.device_key &&
                    message.data.size() >= 3 && message.data[0] == 0x80) {
                    const auto& data = message.data;
                    if (data[1] == 0x0e && data.size() == 3 &&
                        data[2] == mesh_local_ttl_expected_.load())
                        confirmed = kMeshTtlConfigured;
                    else if (data[1] == 0x28 && data.size() == 4 &&
                             data[2] == 0x01 &&
                             data[3] == mesh_local_relay_expected_.load())
                        confirmed = kMeshRelayConfigured;
                    else if (data[1] == 0x25 && data.size() == 3 &&
                             data[2] == mesh_local_network_transmit_expected_.load())
                        confirmed = kMeshNetworkTransmitConfigured;
                }
                if (confirmed != 0) {
                    const uint8_t previous =
                        mesh_local_radio_confirmed_.fetch_or(confirmed);
                    if ((previous & confirmed) == 0) {
                        mesh_local_radio_retry_after_ns_.store(0);
                        if ((previous | confirmed) == kMeshRadioConfigured)
                            RCLCPP_INFO(get_logger(),
                                "Local Mesh TTL, relay and network transmit confirmed");
                    }
                }
                if (message.source != 0 && !local_source) {
                    mesh_last_peer_heard_ns_.store(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count());
                }
                if (mesh_swarm_coordinator_ &&
                    mesh_swarm_coordinator_->handle_message(message)) {
                    return;
                }
                publish_mesh_message(message);
            });
        mesh_app_->set_event_callback(
            [this](const mesh::Event& event) {
                if (mesh_swarm_coordinator_) {
                    mesh_swarm_coordinator_->handle_event(event);
                }
                if (event.event == "join_failed") {
                    mesh_bootstrap_attempted_.store(false);
                }
                // Automatic Mesh lifecycle retries use the latest status.
                if (can_run_callbacks()) publish_mesh_event(event);
            });
        mesh_app_->set_token_callback([this](uint64_t) {
            // Force reliable-state reinitialization after the daemon assigns a new node token.
            mesh_reliable_reset_requested_.store(true);
            mesh_bootstrap_attempted_ = false;
            mesh_local_radio_confirmed_.store(0);
            mesh_local_radio_retry_after_ns_.store(0);
        });
        mesh_app_->export_objects();
        mesh_bootstrap_attempted_ = false;
        mesh_local_radio_confirmed_.store(0);
        mesh_config_signature_ = signature;
    }

    if (!mesh_timer_) {
        mesh_timer_ = create_mesh_wall_timer(
            *this, std::chrono::milliseconds(20),
            [this]() {
                std::unique_lock<std::mutex> lock(
                    config_apply_mutex_, std::try_to_lock);
                if (!lock.owns_lock() || !can_run_callbacks() || !mesh_app_) return;
                // One worker owns both setup and delivery. Separate periodic
                // try-lock timers can collide on every tick and indefinitely
                // starve setup while a one-shot value waits for its AppKey.
                // A busy configuration leaves the maintenance deadline due.
                const auto now = std::chrono::steady_clock::now();
                if (now >= mesh_next_maintenance_) {
                    mesh_next_maintenance_ = now + std::chrono::seconds(1);
                    maintain_mesh();
                }
                if (mesh_app_->status().attached) maintain_mesh_reliable();
            },
            timer_callback_group_);
    }

    // BlueZ Mesh owns the controller's advertising and scanning bearer. Config
    // validation rejects every ordinary LE role in this mode; retain this guard
    // for callers that construct NodeConfig without using the YAML loader.
    if (config.enable_server || config.enable_scan ||
        config.enable_time_service || config.enable_wifi_service ||
        !config.gatt_profile_uuids.empty() ||
        config.enable_serial_port_profile || config.auto_connect_enable) {
        RCLCPP_ERROR(get_logger(),
                     "Mesh requires exclusive controller ownership; ordinary "
                     "LE server, scan, GATT, SPP, and auto-connect roles are disabled");
    }
    // The worker begins setup after this configuration transaction releases
    // its lock, when D-Bus callbacks can observe the committed overlay.
}

void ServiceNode::maintain_mesh() {
    // Called by the Mesh worker with config_apply_mutex_ held.

    mesh_app_->refresh_status();
    const auto status = mesh_app_->status();
    if (mesh_swarm_coordinator_ && !status.daemon_available) {
        mesh_swarm_coordinator_->maintain(status);
    }
    if (!status.daemon_available) {
        mesh_bootstrap_attempted_.store(false);
        mesh_local_radio_confirmed_.store(0);
        mesh_local_radio_retry_after_ns_.store(0);
    } else {
        if (mesh_swarm_coordinator_) {
            try {
                mesh_swarm_coordinator_->maintain(status);
            } catch (const std::exception& error) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 10000,
                    "Automatic Mesh swarm coordination failed: %s", error.what());
            }
        } else if (!status.attached) {
            mesh_local_radio_confirmed_.store(0);
            mesh_local_radio_retry_after_ns_.store(0);
            const bool should_attach =
                status.token != 0 && active_config_.mesh_auto_attach;
            const bool should_create =
                status.token == 0 && active_config_.mesh_auto_create_network;
            const bool should_join =
                status.token == 0 && active_config_.mesh_auto_join;
            bool expected = false;
            if ((should_attach || should_create || should_join) &&
                mesh_bootstrap_attempted_.compare_exchange_strong(expected, true)) {
                try {
                    if (should_attach) {
                        mesh_app_->attach(status.token);
                    } else if (should_create) {
                        mesh_app_->create_network();
                    } else {
                        mesh_app_->join();
                    }
                } catch (const std::exception& error) {
                    mesh_bootstrap_attempted_.store(false);
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 10000,
                        "Automatic Mesh startup failed: %s", error.what());
                }
            }
        }

        const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const uint8_t confirmed = mesh_local_radio_confirmed_.load();
        if (status.attached && active_config_.mesh_auto_configure_relay &&
            confirmed != kMeshRadioConfigured &&
            steady_ns >= mesh_local_radio_retry_after_ns_.load()) {
            // Keep one configuration request outstanding. A matching status
            // releases the next step immediately. Only a missing or mismatched
            // status waits for the retry deadline.
            mesh_local_radio_retry_after_ns_.store(steady_ns + 5000000000LL);
            try {
                if (!(confirmed & kMeshTtlConfigured)) {
                    mesh_app_->configure_local_default_ttl(
                        active_config_.mesh_default_ttl);
                } else if (!(confirmed & kMeshRelayConfigured)) {
                    mesh_app_->configure_local_relay(
                        active_config_.mesh_relay_retransmit_count,
                        active_config_.mesh_relay_retransmit_interval_steps);
                } else {
                    mesh_app_->configure_local_network_transmit(
                        active_config_.mesh_network_retransmit_count,
                        active_config_.mesh_network_retransmit_interval_steps);
                }
            } catch (const std::exception& error) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 10000,
                    "Automatic local Mesh radio configuration failed: %s",
                    error.what());
            }
        }
    }
    publish_mesh_status();

}

void ServiceNode::remember_mesh_transfer(
    uint16_t sequence, uint16_t destination, const mesh::SendHandle& handle) {
    if (handle.transfer == 0) return;
    std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
    const auto sample = mesh_reliable_pending_.find(sequence);
    if (sample != mesh_reliable_pending_.end()) {
        const auto target = sample->second.awaiting.find(destination);
        if (target != sample->second.awaiting.end()) {
            target->second.transfer = handle;
            target->second.transfer_check_at =
                std::chrono::steady_clock::now() +
                std::chrono::milliseconds(100);
            sample->second.blocks_channel = true;
            return;
        }
        // A group transaction has peer receipts but no group-address target.
        // Keep its native handle until every peer acknowledges the sample or
        // the bounded application transaction expires.
        sample->second.primary_transfer = handle;
        return;
    }
    // A receipt can arrive before SendUnqueued returns. Release the returned
    // handle even when that callback has already removed its pending target.
    mesh_transfer_cancellations_.push_back(handle);
}

void ServiceNode::flush_mesh_transfer_cancellations() {
    std::deque<mesh::SendHandle> cancellations;
    {
        std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
        cancellations.swap(mesh_transfer_cancellations_);
    }
    if (!mesh_app_) return;
    while (!cancellations.empty()) {
        try {
            mesh_app_->cancel_send(cancellations.front());
            cancellations.pop_front();
        } catch (const std::exception& error) {
            // Keep ownership on a transient D-Bus failure. Retry the remaining
            // batch on a later tick rather than blocking once per transfer.
            // Reattachment invalidates old handles before new work starts.
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            mesh_transfer_cancellations_.insert(mesh_transfer_cancellations_.end(),
                cancellations.begin(), cancellations.end());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Mesh transfer cleanup failed: %s", error.what());
            return;
        }
    }
}

void ServiceNode::clear_mesh_reliable_state(bool preserve_latest) {
    // Cancel every daemon transfer and optionally retain the latest value per channel.
    {
        std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
        for (const auto& [sequence, pending] : mesh_reliable_pending_) {
            if (pending.primary_transfer)
                mesh_transfer_cancellations_.push_back(*pending.primary_transfer);
            for (const auto& [peer, target] : pending.awaiting)
                if (target.transfer)
                    mesh_transfer_cancellations_.push_back(*target.transfer);
        }
        mesh_reliable_pending_.clear();
        if (!preserve_latest) mesh_reliable_latest_.clear();
        mesh_reliable_seen_.clear();
        mesh_reliable_acks_.clear();
        mesh_reliable_peers_.clear();
        mesh_reliable_turns_.clear();
        mesh_reliable_next_probe_at_ = {};
        mesh_reliable_probe_cursor_ = 0;
    }
    flush_mesh_transfer_cancellations();
}

void ServiceNode::maintain_mesh_reliable() {
    if (mesh_reliable_reset_requested_.exchange(false))
        clear_mesh_reliable_state(true);
    struct Outbound {
        uint16_t destination;
        uint16_t app_key_index;
        uint8_t element_index;
        bool segmented;
        bool acknowledgement;
        std::vector<uint8_t> data;
        std::optional<mesh::SendHandle> transfer{};
    };
    std::vector<Outbound> outbound;
    struct TransferCheck {
        uint16_t sequence;
        uint16_t destination;
        mesh::SendHandle handle;
    };
    std::vector<TransferCheck> transfer_checks;
    std::vector<std::pair<config::SharedTopicConfig, std::vector<uint8_t>>> ready;
    std::set<uint16_t> scheduled_repairs;
    size_t expired = 0;
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
        for (auto it = mesh_reliable_seen_.begin(); it != mesh_reliable_seen_.end();) {
            if (now - it->second.last > std::chrono::seconds(90))
                it = mesh_reliable_seen_.erase(it);
            else
                ++it;
        }
        for (size_t count = 0; count < 4 && !mesh_reliable_acks_.empty(); ++count) {
            auto ack = std::move(mesh_reliable_acks_.front());
            mesh_reliable_acks_.pop_front();
            outbound.push_back({ack.destination, ack.app_key_index, 0, false,
                                true, std::move(ack.data)});
        }
        for (auto it = mesh_reliable_pending_.begin();
             it != mesh_reliable_pending_.end();) {
            auto& pending = it->second;
            if (pending.awaiting.empty()) {
                if (pending.primary_transfer)
                    mesh_transfer_cancellations_.push_back(*pending.primary_transfer);
                it = mesh_reliable_pending_.erase(it);
                continue;
            }
            if (now - pending.created > std::chrono::seconds(30)) {
                for (const auto& [peer, target] : pending.awaiting) {
                    if (target.transfer)
                        mesh_transfer_cancellations_.push_back(*target.transfer);
                    auto& state = mesh_reliable_peers_[peer];
                    state.consecutive_expiries =
                        std::min(state.consecutive_expiries + 1U, 6U);
                    state.retry_after = now + std::chrono::seconds(
                        std::min(5U * state.consecutive_expiries, 30U));
                }
                if (pending.primary_transfer)
                    mesh_transfer_cancellations_.push_back(*pending.primary_transfer);
                ++expired;
                it = mesh_reliable_pending_.erase(it);
                continue;
            }
            const auto latest = mesh_reliable_latest_.find(pending.channel_key);
            if (latest != mesh_reliable_latest_.end()) {
                // Topic bridges promise the newest state, not a historical
                // stream. If a receiver was offline, repair the outstanding
                // receipt identity with the newest encoded value. A receiver
                // that already published this sequence discards the repair;
                // a restarted receiver avoids publishing the obsolete sample.
                pending.data = make_mesh_reliable_data(
                    latest->second.second, it->first,
                    mesh_reliable_swarm_id(pending.data));
                mesh_reliable_latest_.erase(latest);
            }
            for (auto& [peer, target] : pending.awaiting) {
                if (outbound.size() >= 6) break;
                if (pending.native_completion_releases_channel && target.transfer &&
                    target.transfer_check_at <= now) {
                    transfer_checks.push_back({it->first, peer, *target.transfer});
                    target.transfer_check_at = now +
                        std::chrono::milliseconds(100);
                }
                if (target.receipt_query_at <= now && target.retry_at > now) {
                    // A lost application receipt does not require sending the
                    // complete segmented sample again. The receiver answers
                    // this unsegmented query only after its ROS bridge has
                    // decoded and published the original sample.
                    std::vector<uint8_t> query(pending.data.begin(),
                        pending.data.begin() + kReliableAckHeaderBytes);
                    query[3] = kReliableReceiptQuery;
                    outbound.push_back({peer, pending.app_key_index,
                        pending.element_index, false, false, std::move(query),
                        target.transfer});
                    target.receipt_query_at = now + kReceiptQueryPeriod;
                    continue;
                }
                if (target.retry_at > now || target.attempts >= 3 ||
                    scheduled_repairs.contains(peer) ||
                    mesh_reliable_peers_[peer].retry_after > now) continue;
                auto repair = pending.data;
                repair[3] = peer == pending.turn_successor
                    ? kReliableData : kReliableDataNoTurn;
                outbound.push_back({peer, pending.app_key_index,
                                    pending.element_index,
                                    pending.force_segmented, false,
                                    std::move(repair)});
                scheduled_repairs.insert(peer);
                ++target.attempts;
                target.receipt_query_at = now + kReceiptQueryPeriod;
                target.retry_at = now +
                    std::chrono::seconds(4 * (target.attempts + 1));
            }
            ++it;
        }
        for (auto it = mesh_reliable_latest_.begin(); it != mesh_reliable_latest_.end();) {
            const bool busy = std::any_of(
                mesh_reliable_pending_.begin(), mesh_reliable_pending_.end(),
                [&](const auto& pending) {
                    // Keep the channel blocked while any transfer on it is awaiting completion.
                    return pending.second.channel_key == it->first &&
                        pending.second.blocks_channel;
                });
            if (busy) {
                ++it;
            } else {
                ready.push_back(std::move(it->second));
                it = mesh_reliable_latest_.erase(it);
            }
        }
    }
    for (const auto& check : transfer_checks) {
        try {
            if (mesh_app_->send_pending(check.handle)) continue;
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            const auto sample = mesh_reliable_pending_.find(check.sequence);
            if (sample == mesh_reliable_pending_.end()) continue;
            const auto target = sample->second.awaiting.find(check.destination);
            if (target == sample->second.awaiting.end() ||
                !target->second.transfer ||
                target->second.transfer->attachment != check.handle.attachment ||
                target->second.transfer->transfer != check.handle.transfer) continue;
            target->second.transfer.reset();
            sample->second.blocks_channel = false;
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Mesh transfer status check failed: %s", error.what());
        }
    }
    if (expired != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                    "Mesh delivery expired for %zu sample(s) without all peer acknowledgements",
                    expired);
    }
    flush_mesh_transfer_cancellations();
    for (auto& item : outbound) {
        try {
            // Let native selective retransmission finish first. An early
            // access-layer query spends airtime while the sample is incomplete
            // and can advance the peer's replay watermark past queued segments.
            if (item.data[3] == kReliableReceiptQuery && item.transfer &&
                mesh_app_->send_pending(*item.transfer)) continue;
            const auto handle = mesh_app_->try_send(item.element_index,
                item.destination, item.app_key_index, item.segmented, item.data);
            if (handle && item.data[3] == kReliableData)
                remember_mesh_transfer(mesh_reliable_sequence(item.data),
                    item.destination, *handle);
            if (!handle) {
                // Native SAR still owns this destination. No duplicate was
                // queued, so this does not consume an application retry.
                std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                const auto retry_at = std::chrono::steady_clock::now() +
                    kMeshQueueRetryDelay;
                mesh_reliable_peers_[item.destination].transport_busy_until = retry_at;
                const auto sample = mesh_reliable_pending_.find(
                    mesh_reliable_sequence(item.data));
                if (sample != mesh_reliable_pending_.end()) {
                    const auto target = sample->second.awaiting.find(item.destination);
                    if (target != sample->second.awaiting.end()) {
                        if (target->second.attempts > 0) --target->second.attempts;
                        target->second.retry_at = retry_at;
                    }
                }
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Mesh %s to 0x%04x failed: %s",
                item.acknowledgement ? "acknowledgement" : "unicast repair",
                item.destination, error.what());
            if (item.acknowledgement) {
                std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                if (mesh_reliable_acks_.size() < kMaxQueuedMeshAcks)
                    mesh_reliable_acks_.push_front({item.destination,
                        item.app_key_index, std::move(item.data)});
            }
        }
    }
    for (auto& [bridge, payload] : ready) {
        try {
            send_mesh_bridge_payload_impl(bridge, payload, true);
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Could not resume Mesh channel %u: %s", bridge.channel_id, error.what());
        }
    }
}

void ServiceNode::publish_mesh_status() {
    // Copy the current daemon and node attachment snapshot into the ROS status message.
    if (!mesh_status_pub_ || !mesh_app_) return;
    const auto status = mesh_app_->status();
    mrs_uav_bluetooth::msg::MeshStatus message;
    message.header.stamp = now();
    message.enabled = active_config_.enable_mesh;
    message.daemon_available = status.daemon_available;
    message.attached = status.attached;
    message.state = status.state;
    message.uuid = status.uuid;
    message.node_path = status.node_path;
    message.token = status.token;
    message.automatic_provisioning = static_cast<bool>(mesh_swarm_coordinator_);
    if (mesh_swarm_coordinator_) {
        message.preferred_provisioner =
            mesh_swarm_coordinator_->preferred_provisioner();
        message.active_provisioner =
            mesh_swarm_coordinator_->active_provisioner();
        message.local_is_active_provisioner =
            mesh_swarm_coordinator_->local_is_active_provisioner();
        message.application_transport_ready =
            mesh_swarm_coordinator_->application_ready() &&
            (!active_config_.mesh_auto_configure_relay ||
             mesh_local_radio_confirmed_.load() == kMeshRadioConfigured);
        message.swarm_id = mesh_swarm_coordinator_->swarm_id();
        message.swarm_participating =
            mesh_swarm_coordinator_->swarm_participating();
        message.swarm_members = mesh_swarm_coordinator_->swarm_members();
    }
    message.addresses = status.addresses;
    message.friend_feature = status.friend_feature;
    message.low_power_feature = status.low_power_feature;
    message.proxy_feature = status.proxy_feature;
    message.relay_feature = status.relay_feature;
    message.beacon = status.beacon;
    message.iv_update = status.iv_update;
    message.iv_index = status.iv_index;
    const auto heard = mesh_last_peer_heard_ns_.load();
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    message.seconds_since_last_heard = heard == 0 ? UINT32_MAX :
        static_cast<uint32_t>(std::max<int64_t>(0, steady_ns - heard) / 1000000000);
    message.sequence_number = status.sequence_number;
    message.error = status.error;
    mesh_status_pub_->publish(std::move(message));
}

void ServiceNode::publish_mesh_message(const mesh::ReceivedMessage& received) {
    // Decode configured bridges internally, while always retaining the raw
    // MeshMessage publication below for user-written ROS-only controllers.
    if (transport_bridges_ && !received.device_key) {
        bool local_source = false;
        if (mesh_app_) {
            const auto status = mesh_app_->status();
            local_source = std::find(status.addresses.begin(), status.addresses.end(),
                                     received.source) != status.addresses.end();
        }
        // Logical swarm admission applies only to automatic Mesh overlays.
        // Manual controllers continue to receive the raw /mesh/rx topic and
        // retain their own bridge policy.
        // BlueZ does not provide the subnet index in MessageReceived. The
        // authenticated AppKey, coordination frame and live source mapping
        // provide the available admission checks for automatic overlays.
        const bool swarm_allowed = !mesh_swarm_coordinator_ ||
            mesh_swarm_coordinator_->accepts_swarm_payload(received.source);
        if (!local_source && is_mesh_reliable_frame(received.data)) {
            const auto sequence = mesh_reliable_sequence(received.data);
            const bool data_frame =
                (received.data[3] == kReliableData ||
                 received.data[3] == kReliableDataNoTurn) &&
                received.data.size() >= kReliableDataHeaderBytes;
            const bool frame_allowed = !mesh_swarm_coordinator_ ||
                mesh_swarm_coordinator_->accepts_swarm_payload(
                    received.source, mesh_reliable_swarm_id(received.data));
            // Receipts and queries have exactly the compact header. They
            // never pass through the topic codec as application data.
            const bool receipt_query = received.data[3] == kReliableReceiptQuery &&
                received.data.size() == kReliableAckHeaderBytes;
            if (received.data[3] == kReliableAck && frame_allowed &&
                received.data.size() == kReliableAckHeaderBytes) {
                std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                const auto it = mesh_reliable_pending_.find(sequence);
                if (it != mesh_reliable_pending_.end() &&
                    it->second.app_key_index == received.key_index &&
                    std::equal(received.data.begin(),
                        received.data.begin() + kMeshVendorPrefixBytes,
                        it->second.data.begin()) &&
                    mesh_reliable_swarm_id(received.data) ==
                        mesh_reliable_swarm_id(it->second.data)) {
                    const auto target = it->second.awaiting.find(received.source);
                    if (target != it->second.awaiting.end()) {
                        if (target->second.transfer)
                            mesh_transfer_cancellations_.push_back(*target->second.transfer);
                        it->second.awaiting.erase(target);
                    }
                    mesh_reliable_peers_[received.source] = {};
                }
            } else if ((data_frame || receipt_query) && frame_allowed) {
                const uint32_t vendor_opcode = received.data[0] |
                    (static_cast<uint32_t>(received.data[1]) << 8U) |
                    (static_cast<uint32_t>(received.data[2]) << 16U);
                const MeshReliableSeenKey key{received.source, received.key_index,
                    mesh_reliable_swarm_id(received.data), vendor_opcode, sequence};
                // Call only while holding mesh_reliable_mutex_. A duplicate
                // receives another ACK only after the first copy was accepted
                // and published by its configured ROS bridge.
                const auto queue_ack = [&]() {
                    // Build and queue one bounded acknowledgement for this received fragment.
                    if (mesh_reliable_acks_.size() >= kMaxQueuedMeshAcks) return;
                    std::vector<uint8_t> acknowledgement(
                        received.data.begin(),
                        received.data.begin() + kMeshVendorPrefixBytes);
                    acknowledgement.push_back(kReliableAck);
                    append_mesh_reliable_sequence(acknowledgement, sequence);
                    append_mesh_reliable_swarm_id(
                        acknowledgement, mesh_reliable_swarm_id(received.data));
                    const bool queued = std::any_of(mesh_reliable_acks_.begin(),
                        mesh_reliable_acks_.end(), [&](const auto& ack) {
                            // Avoid queuing a duplicate acknowledgement for the same source and transfer.
                            return ack.destination == received.source &&
                                ack.app_key_index == received.key_index &&
                                ack.data == acknowledgement;
                        });
                    if (!queued) mesh_reliable_acks_.push_back(
                        {received.source, received.key_index,
                         std::move(acknowledgement)});
                };
                bool should_decode = false;
                {
                    std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                    if (receipt_query) {
                        const auto it = mesh_reliable_seen_.find(key);
                        if (it != mesh_reliable_seen_.end() && it->second.delivered)
                            queue_ack();
                    } else {
                        const auto [it, inserted] = mesh_reliable_seen_.emplace(
                            key, MeshReliableSeen{std::chrono::steady_clock::now(), false});
                        should_decode = inserted;
                        if (!inserted && it->second.delivered) queue_ack();
                    }
                }
                if (should_decode) {
                    std::vector<uint8_t> decoded(
                        received.data.begin(),
                        received.data.begin() + kMeshVendorPrefixBytes);
                    decoded.insert(decoded.end(), {'M', 'B', 1,
                        received.data[8], received.data[9]});
                    decoded.insert(decoded.end(),
                        received.data.begin() + kReliableDataHeaderBytes,
                        received.data.end());
                    const bool delivered = transport_bridges_->handle_mesh(
                        received.source, received.key_index, decoded);
                    std::vector<uint16_t> turn_members;
                    uint16_t local_address = 0;
                    if (delivered && mesh_swarm_coordinator_ && mesh_app_) {
                        const auto mesh_status = mesh_app_->status();
                        if (!mesh_status.addresses.empty()) {
                            local_address = mesh_status.addresses.front();
                            turn_members = mesh_turn_members(local_address,
                                mesh_swarm_coordinator_->swarm_member_addresses());
                        }
                    }
                    std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                    const auto it = mesh_reliable_seen_.find(key);
                    if (it != mesh_reliable_seen_.end()) {
                        if (delivered) {
                            it->second.delivered = true;
                            it->second.last = std::chrono::steady_clock::now();
                            queue_ack();
                            const uint64_t channel_key =
                                (static_cast<uint64_t>(received.element_index) << 32U) |
                                (static_cast<uint64_t>(received.key_index) << 16U) |
                                (static_cast<uint16_t>(received.data[8]) |
                                 (static_cast<uint16_t>(received.data[9]) << 8U));
                            if (local_address != 0) {
                                auto& turn = mesh_reliable_turns_[channel_key];
                                turn.members = turn_members;
                                turn.initialized = true;
                                // The sender marks exactly one unicast copy as
                                // the handoff. All other recipients explicitly
                                // clear permission, so temporarily different
                                // heartbeat views cannot create two successors.
                                turn.permitted =
                                    received.data[3] == kReliableData;
                                turn.reclaim_at = std::chrono::steady_clock::now() +
                                    kMeshTurnRecovery;
                            }
                        } else {
                            mesh_reliable_seen_.erase(it);
                        }
                    }
                }
            }
        } else if (!local_source && swarm_allowed) {
            transport_bridges_->handle_mesh(
                received.source, received.key_index, received.data);
        }
    }

    if (!mesh_message_pub_) return;
    mrs_uav_bluetooth::msg::MeshMessage message;
    message.header.stamp = now();
    message.element_index = received.element_index;
    message.source = received.source;
    message.destination = received.destination;
    message.key_index = received.key_index;
    message.net_index = received.net_index;
    message.device_key = received.device_key;
    message.remote = received.remote;
    message.data = received.data;
    mesh_message_pub_->publish(std::move(message));
}

void ServiceNode::send_mesh_bridge_payload(
    const config::SharedTopicConfig& bridge,
    const std::vector<uint8_t>& payload) {
    // Route ordinary frames directly and reliable frames through per-channel arbitration.
    send_mesh_bridge_payload_impl(bridge, payload, false);
}

void ServiceNode::send_mesh_bridge_payload_impl(
    const config::SharedTopicConfig& bridge,
    const std::vector<uint8_t>& payload,
    bool resumed) {
    if (!mesh_app_) {
        throw std::runtime_error(
            "Mesh bridge cannot send because enable_mesh is false");
    }
    if (mesh_reliable_reset_requested_.exchange(false))
        clear_mesh_reliable_state(true);
    const uint64_t channel_key =
        (static_cast<uint64_t>(bridge.mesh_element_index) << 32U) |
        (static_cast<uint64_t>(bridge.mesh_app_key_index) << 16U) |
        bridge.channel_id;
    // A resumed value may race a newer source callback. Preserve the newer
    // value instead of putting an older sample back at the head of the queue.
    const auto store_latest_locked = [&]() {
        // Coalesce unsent samples by channel, preserving resumed traffic ahead of new data.
        if (resumed)
            mesh_reliable_latest_.try_emplace(channel_key, bridge, payload);
        else
            mesh_reliable_latest_[channel_key] = {bridge, payload};
    };
    const auto status = mesh_app_->status();
    if (!status.attached ||
        (active_config_.mesh_auto_configure_relay &&
         mesh_local_radio_confirmed_.load() != kMeshRadioConfigured) ||
        (mesh_swarm_coordinator_ &&
        (!mesh_swarm_coordinator_->swarm_participating() ||
         !mesh_swarm_coordinator_->application_ready()))) {
        // A one-shot ROS source must survive startup just like a periodic
        // source. Keep its latest encoded value until attachment, AppKey
        // binding, and swarm admission are ready. Overlay changes clear it.
        std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
        store_latest_locked();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
        if (resumed) {
            // A source callback queued a newer value after maintenance moved
            // this one out. Drop the stale resume without touching that value.
            if (mesh_reliable_latest_.contains(channel_key)) return;
        } else {
            mesh_reliable_latest_.erase(channel_key);
        }
    }
    if (bridge.mesh_swarm_reliable && mesh_swarm_coordinator_) {
        if (payload.size() < kMeshVendorPrefixBytes + kBridgeFramePrefixBytes ||
            payload[3] != 'M' || payload[4] != 'B' || payload[5] != 1) {
            throw std::runtime_error("Mesh bridge payload has no valid channel frame");
        }
        {
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            const bool busy = std::any_of(
                mesh_reliable_pending_.begin(), mesh_reliable_pending_.end(),
                [&](const auto& pending) {
                    // Keep a resumed channel blocked until its prior transfer reaches a terminal state.
                    return pending.second.channel_key == channel_key &&
                        pending.second.blocks_channel;
                });
            if (busy) {
                store_latest_locked();
                return;
            }
        }
        auto targets = mesh_swarm_coordinator_->swarm_member_addresses();
        const auto turn_peers = targets;
        const auto probe_candidates =
            mesh_swarm_coordinator_->swarm_probe_addresses();
        uint16_t sequence;
        uint16_t probe_destination = 0;
        bool consumes_mesh_turn = false;
        uint16_t turn_successor = 0;
        {
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            const auto now = std::chrono::steady_clock::now();
            if (!probe_candidates.empty() && now >= mesh_reliable_next_probe_at_) {
                // A missed heartbeat is not proof that a peer left the
                // network. Probe one configured, unheard address at a time so
                // many offline UAVs cannot flood the radio with repairs.
                for (size_t offset = 0; offset < probe_candidates.size(); ++offset) {
                    const auto index = (mesh_reliable_probe_cursor_ + offset) %
                        probe_candidates.size();
                    const auto peer = probe_candidates[index];
                    if (mesh_reliable_peers_[peer].retry_after > now) continue;
                    probe_destination = peer;
                    targets.push_back(peer);
                    mesh_reliable_probe_cursor_ = index + 1;
                    mesh_reliable_next_probe_at_ = now + std::chrono::seconds(10);
                    break;
                }
            }
            std::sort(targets.begin(), targets.end());
            targets.erase(std::unique(targets.begin(), targets.end()), targets.end());
            if (targets.empty()) {
                // Coordination discovers members independently. With no
                // recipient or due probe, retain one sample instead of
                // filling the daemon's multicast queue with undeliverable
                // data during startup, peer loss, or a swarm change.
                store_latest_locked();
                return;
            }
            do {
                sequence = static_cast<uint16_t>(++mesh_reliable_next_sequence_);
            } while (mesh_reliable_pending_.contains(sequence));
            // A token holder fans the latest value out once to every live
            // member. Per-destination unicast SAR supplies selective segment
            // repair; application receipts cover end-to-end publication.
            // Offline probes join this fan-out but never join the turn ring,
            // so an unheard address cannot stall N live participants.
            if (!turn_peers.empty() && !status.addresses.empty()) {
                const auto members = mesh_turn_members(
                    status.addresses.front(), turn_peers);
                auto& turn = mesh_reliable_turns_[channel_key];
                if (!turn.initialized) {
                    turn.members = members;
                    turn.initialized = true;
                    turn.permitted = members.front() == status.addresses.front();
                    turn.reclaim_at = now + kMeshTurnRecovery;
                } else if (turn.members != members) {
                    // A membership refresh must not manufacture a new token.
                    // Preserve the existing handoff and let only the lowest
                    // current member recover it after the silence deadline.
                    turn.members = members;
                }
                if (!turn.permitted && members.front() == status.addresses.front() &&
                    now >= turn.reclaim_at) {
                    // Only the lowest live address may recover a lost turn,
                    // so simultaneous restarts still have one initiator.
                    turn.permitted = true;
                }
                if (!turn.permitted) {
                    store_latest_locked();
                    return;
                }
                const auto local = std::find(members.begin(), members.end(),
                    status.addresses.front());
                const auto next = std::next(local) == members.end()
                    ? members.begin() : std::next(local);
                turn_successor = *next;
                consumes_mesh_turn = true;
            }
        }
        if (probe_destination != 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                "Probing unheard Mesh peer 0x%04x with reliable topic data",
                probe_destination);
        }
        auto wrapped = make_mesh_reliable_data(
            payload, sequence, mesh_swarm_coordinator_->swarm_id());
        if (!targets.empty()) {
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            if (mesh_reliable_pending_.size() >= kMaxPendingMeshSamples) {
                const auto oldest = std::min_element(
                    mesh_reliable_pending_.begin(), mesh_reliable_pending_.end(),
                    [](const auto& left, const auto& right) {
                        // Evict the oldest pending transfer first when the bounded queue is full.
                        return left.second.created < right.second.created;
                    });
                if (oldest->second.primary_transfer)
                    mesh_transfer_cancellations_.push_back(
                        *oldest->second.primary_transfer);
                for (const auto& [peer, target] : oldest->second.awaiting)
                    if (target.transfer)
                        mesh_transfer_cancellations_.push_back(*target.transfer);
                mesh_reliable_pending_.erase(oldest);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                    "Mesh reliable queue is full. Dropping the oldest "
                    "unacknowledged sample to keep current data moving");
            }
            MeshReliablePending pending;
            pending.channel_key = channel_key;
            pending.data = wrapped;
            for (const auto peer : targets) {
                pending.awaiting.emplace(peer, MeshReliableTarget{
                    0, std::nullopt, now + std::chrono::seconds(5),
                    now + kReceiptQueryPeriod, now});
            }
            pending.app_key_index = bridge.mesh_app_key_index;
            pending.element_index = bridge.mesh_element_index;
            pending.turn_successor = turn_successor;
            pending.force_segmented = bridge.mesh_force_segmented;
            pending.native_completion_releases_channel = targets.size() == 1;
            pending.created = now;
            mesh_reliable_pending_.emplace(sequence, std::move(pending));
        }
        if (consumes_mesh_turn) {
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            auto& turn = mesh_reliable_turns_[channel_key];
            turn.permitted = false;
            turn.reclaim_at = std::chrono::steady_clock::now() +
                kMeshTurnRecovery;
        }
        bool any_admitted = false;
        for (const auto destination : targets) {
            try {
                auto wire = wrapped;
                wire[3] = destination == turn_successor
                    ? kReliableData : kReliableDataNoTurn;
                const auto handle = mesh_app_->try_send(
                    bridge.mesh_element_index, destination,
                    bridge.mesh_app_key_index, bridge.mesh_force_segmented,
                    wire);
                if (handle) {
                    any_admitted = true;
                    remember_mesh_transfer(sequence, destination, *handle);
                    continue;
                }
                std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                const auto retry_at = std::chrono::steady_clock::now() +
                    kMeshQueueRetryDelay;
                mesh_reliable_peers_[destination].transport_busy_until = retry_at;
                const auto sample = mesh_reliable_pending_.find(sequence);
                if (sample != mesh_reliable_pending_.end()) {
                    const auto target = sample->second.awaiting.find(destination);
                    if (target != sample->second.awaiting.end())
                        target->second.retry_at = retry_at;
                }
            } catch (const std::exception& error) {
                {
                    std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
                    const auto sample = mesh_reliable_pending_.find(sequence);
                    if (sample != mesh_reliable_pending_.end()) {
                        const auto target =
                            sample->second.awaiting.find(destination);
                        if (target != sample->second.awaiting.end())
                            target->second.retry_at =
                                std::chrono::steady_clock::now() +
                                kMeshQueueRetryDelay;
                    }
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Mesh initial unicast to 0x%04x failed: %s",
                    destination, error.what());
            }
        }
        if (!any_admitted) {
            std::lock_guard<std::mutex> lock(mesh_reliable_mutex_);
            mesh_reliable_pending_.erase(sequence);
            store_latest_locked();
            if (consumes_mesh_turn)
                mesh_reliable_turns_[channel_key].permitted = true;
        }
        return;
    }
    mesh_app_->send(bridge.mesh_element_index, bridge.mesh_destination,
                    bridge.mesh_app_key_index, bridge.mesh_force_segmented,
                    payload);
}

void ServiceNode::publish_mesh_event(const mesh::Event& received) {
    // Preserve the Mesh event name, peer identity, reason, signal level, and data.
    if (!mesh_event_pub_) return;
    mrs_uav_bluetooth::msg::MeshEvent message;
    message.header.stamp = now();
    message.event = received.event;
    message.uuid = received.uuid;
    message.reason = received.reason;
    message.rssi = received.rssi;
    message.data = received.data;
    message.server = received.server;
    message.original = received.original;
    message.unicast = received.unicast;
    message.nppi = received.nppi;
    message.count = received.count;
    message.detail = received.detail;
    mesh_event_pub_->publish(std::move(message));
}

void ServiceNode::handle_mesh_network(
    const std::shared_ptr<mrs_uav_bluetooth::srv::MeshNetwork::Request> request,
    std::shared_ptr<mrs_uav_bluetooth::srv::MeshNetwork::Response> response) {
    // Restrict automatic mode, execute the requested Network1 action, and return fresh status.
    if (!mesh_app_) {
        response->success = false;
        response->message = "Mesh is disabled; set enable_mesh: true";
        return;
    }

    try {
        const auto action = lower_copy(request->action);
        if (mesh_swarm_coordinator_) {
            throw std::runtime_error(
                "Automatic Mesh setup owns the persistent identity; use mesh/swarm for logical membership");
        }
        if (action == "join") {
            mesh_app_->join(request->uuid);
        } else if (action == "cancel") {
            mesh_app_->cancel_join();
        } else if (action == "attach") {
            mesh_app_->attach(request->token);
        } else if (action == "leave") {
            mesh_app_->leave(request->token);
        } else if (action == "create" || action == "create_network") {
            mesh_app_->create_network(request->uuid);
        } else if (action == "import") {
            const auto& uuid = request->uuid.empty() ? mesh_app_->uuid() : request->uuid;
            mesh_app_->import_node(uuid, request->device_key, request->network_key,
                                   request->network_index, request->iv_update,
                                   request->key_refresh, request->iv_index,
                                   request->unicast);
        } else {
            throw std::invalid_argument("unknown Mesh network action: " + request->action);
        }
        mesh_app_->refresh_status();
        response->success = true;
        response->message = "ok";
        mesh_bootstrap_attempted_ = action == "join" || action == "create" ||
                                    action == "create_network";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
    }
    populate_network_status(mesh_app_->status(), *response);
    publish_mesh_status();
}

void ServiceNode::handle_mesh_send(
    const std::shared_ptr<mrs_uav_bluetooth::srv::MeshSend::Request> request,
    std::shared_ptr<mrs_uav_bluetooth::srv::MeshSend::Response> response) {
    // Select Node1 send, publish, device-key, or key-distribution behavior from the request mode.
    if (!mesh_app_) {
        response->success = false;
        response->message = "Mesh is disabled; set enable_mesh: true";
        return;
    }

    try {
        const auto mode = lower_copy(request->mode);
        if (mesh_swarm_coordinator_ && mode != "send" && mode != "publish") {
            throw std::runtime_error(
                "Automatic Mesh setup permits application data only; use manual Mesh mode "
                "for Device Key configuration or key distribution");
        }
        if (mode == "send") {
            mesh_app_->send(request->element_index, request->destination,
                            request->key_index, request->force_segmented,
                            request->data);
        } else if (mode == "dev_key_send") {
            mesh_app_->dev_key_send(request->element_index, request->destination,
                                    request->remote, request->network_index,
                                    request->force_segmented, request->data);
        } else if (mode == "publish") {
            mesh_app_->publish(request->element_index, request->model_id,
                               request->vendor_model
                                   ? std::optional<uint16_t>{request->vendor_id}
                                   : std::nullopt,
                               request->force_segmented, request->data);
        } else if (mode == "add_net_key") {
            mesh_app_->add_net_key(request->element_index, request->destination,
                                   request->key_index, request->network_index,
                                   request->update);
        } else if (mode == "add_app_key") {
            mesh_app_->add_app_key(request->element_index, request->destination,
                                   request->key_index, request->network_index,
                                   request->update);
        } else {
            throw std::invalid_argument("unknown Mesh send mode: " + request->mode);
        }
        response->success = true;
        response->message = "ok";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
    }
}

void ServiceNode::handle_mesh_management(
    const std::shared_ptr<mrs_uav_bluetooth::srv::MeshManagement::Request> request,
    std::shared_ptr<mrs_uav_bluetooth::srv::MeshManagement::Response> response) {
    // Translate the requested scan, provisioning, key, or remote-node action to Management1.
    if (!mesh_app_) {
        response->success = false;
        response->message = "Mesh is disabled; set enable_mesh: true";
        return;
    }

    try {
        const auto action = lower_copy(request->action);
        if (mesh_swarm_coordinator_ && action != "export_keys") {
            throw std::runtime_error(
                "Automatic Mesh setup owns keys and fixed addresses; use manual Mesh mode "
                "for PB-ADV provisioning or explicit key management");
        }
        mesh::VariantMap scan_options;
        if (request->seconds != 0)
            scan_options.emplace("Seconds", sdbus::Variant{request->seconds});
        if (request->server != 0)
            scan_options.emplace("Server", sdbus::Variant{request->server});
        if (request->subnet != 0)
            scan_options.emplace("Subnet", sdbus::Variant{request->subnet});
        if (!request->filter.empty())
            scan_options.emplace("Filter", sdbus::Variant{request->filter});
        if (!request->extended.empty())
            scan_options.emplace("Extended", sdbus::Variant{request->extended});

        mesh::VariantMap node_options;
        if (request->seconds != 0)
            node_options.emplace("Seconds", sdbus::Variant{request->seconds});
        if (request->server != 0)
            node_options.emplace("Server", sdbus::Variant{request->server});
        if (request->subnet != 0)
            node_options.emplace("Subnet", sdbus::Variant{request->subnet});

        if (action == "scan") {
            mesh_app_->unprovisioned_scan(scan_options);
        } else if (action == "scan_cancel") {
            mesh_app_->unprovisioned_scan_cancel();
        } else if (action == "add_node") {
            mesh_app_->add_node(request->uuid, node_options);
        } else if (action == "reprovision") {
            node_options.emplace("NPPI", sdbus::Variant{request->nppi});
            mesh_app_->reprovision(request->unicast, node_options);
        } else if (action == "create_subnet") {
            mesh_app_->create_subnet(request->network_index);
        } else if (action == "import_subnet") {
            mesh_app_->import_subnet(request->network_index, request->key);
        } else if (action == "update_subnet") {
            mesh_app_->update_subnet(request->network_index);
        } else if (action == "delete_subnet") {
            mesh_app_->delete_subnet(request->network_index);
        } else if (action == "set_key_phase") {
            mesh_app_->set_key_phase(request->network_index, request->phase);
        } else if (action == "create_app_key") {
            mesh_app_->create_app_key(request->network_index,
                                      request->application_index);
        } else if (action == "import_app_key") {
            mesh_app_->import_app_key(request->network_index,
                                      request->application_index, request->key);
        } else if (action == "update_app_key") {
            mesh_app_->update_app_key(request->application_index);
        } else if (action == "delete_app_key") {
            mesh_app_->delete_app_key(request->application_index);
        } else if (action == "import_remote_node") {
            mesh_app_->import_remote_node(request->unicast, request->count,
                                          request->key);
        } else if (action == "delete_remote_node") {
            mesh_app_->delete_remote_node(request->unicast, request->count);
        } else if (action == "export_keys") {
            response->result_yaml = mesh_app_->export_keys_yaml();
        } else {
            throw std::invalid_argument("unknown Mesh management action: " +
                                        request->action);
        }
        response->success = true;
        response->message = "ok";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
    }
}

void ServiceNode::handle_mesh_swarm(
    const std::shared_ptr<mrs_uav_bluetooth::srv::MeshSwarm::Request> request,
    std::shared_ptr<mrs_uav_bluetooth::srv::MeshSwarm::Response> response) {
    // Apply logical membership changes through the N-peer coordinator and return its state.
    if (!mesh_swarm_coordinator_) {
        response->success = false;
        response->message =
            "Automatic Mesh swarm mode is disabled by the active overlay";
        return;
    }

    try {
        const auto action = lower_copy(request->action);
        if (action == "join") {
            mesh_swarm_coordinator_->join_swarm(request->swarm_id);
        } else if (action == "leave") {
            mesh_swarm_coordinator_->leave_swarm();
        } else if (action != "status") {
            throw std::invalid_argument(
                "Mesh swarm action must be status, join, or leave");
        }
        response->success = true;
        response->message = "ok";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
    }
    response->swarm_id = mesh_swarm_coordinator_->swarm_id();
    response->participating =
        mesh_swarm_coordinator_->swarm_participating();
    response->members = mesh_swarm_coordinator_->swarm_members();
    publish_mesh_status();
}

}  // namespace mrs_uav_bluetooth::app
