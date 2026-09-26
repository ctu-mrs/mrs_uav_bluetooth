// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace mrs_uav_bluetooth::app {

namespace {

template<typename DurationT, typename CallbackT>
rclcpp::TimerBase::SharedPtr create_mesh_wall_timer(
    rclcpp::Node& node,
    DurationT period,
    CallbackT&& callback,
    const rclcpp::CallbackGroup::SharedPtr& group) {
    return rclcpp::create_wall_timer(
        period, std::forward<CallbackT>(callback), group,
        node.get_node_base_interface().get(),
        node.get_node_timers_interface().get());
}

/// Release an on-demand Mesh bearer before conventional LE starts. The package
/// grants the mrs account narrowly scoped StopUnit permission for this unit.
void stop_mesh_bearer(bluez::DbusConnection& connection) {
    auto bus = sdbus::createProxy(connection.connection(),
        sdbus::ServiceName{"org.freedesktop.DBus"},
        sdbus::ObjectPath{"/org/freedesktop/DBus"});
    const auto running = [&]() {
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

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

std::string mesh_config_signature(const config::NodeConfig& config) {
    std::ostringstream stream;
    stream << config.enable_mesh << ':' << config.mesh_provisioner << ':'
           << config.mesh_auto_configure_relay << ':'
           << unsigned(config.mesh_default_ttl) << ':'
           << unsigned(config.mesh_relay_retransmit_count) << ':'
           << unsigned(config.mesh_relay_retransmit_interval_steps) << ':'
           << config.mesh_auto_attach << ':' << config.mesh_auto_join << ':'
           << config.mesh_auto_create_network << ':'
           << config.mesh_swarm_auto_provisioning << ':'
           << config.mesh_fleet_config_path << ':' << config.mesh_fleet_id << ':'
           << mesh::mesh_bytes_to_hex(config.mesh_fleet_network_key) << ':'
           << mesh::mesh_bytes_to_hex(config.mesh_fleet_application_key) << ':'
           << config.mesh_fleet_iv_index << ':' << config.mesh_fleet_unicast << ':'
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
void populate_network_status(const mesh::Status& status, ResponseT& response) {
    response.state = status.state;
    response.token = status.token;
    response.node_path = status.node_path;
    response.addresses = status.addresses;
}

}  // namespace

void ServiceNode::configure_mesh(const config::NodeConfig& config) {
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
        mesh_local_relay_configured_ = false;
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
                if (message.source != 0 &&
                    std::find(local.begin(), local.end(), message.source) == local.end()) {
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
                // Automatic fleet lifecycle retries use the latest Mesh status.
                if (can_run_callbacks()) publish_mesh_event(event);
            });
        mesh_app_->set_token_callback([this](uint64_t) {
            mesh_bootstrap_attempted_ = false;
            mesh_local_relay_configured_ = false;
        });
        mesh_app_->export_objects();
        mesh_bootstrap_attempted_ = false;
        mesh_local_relay_configured_ = false;
        mesh_config_signature_ = signature;
    }

    if (!mesh_timer_) {
        mesh_timer_ = create_mesh_wall_timer(
            *this, std::chrono::seconds(1),
            [this]() {
                if (can_run_callbacks()) maintain_mesh();
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
    maintain_mesh();
}

void ServiceNode::maintain_mesh() {
    if (!mesh_app_) return;

    mesh_app_->refresh_status();
    const auto status = mesh_app_->status();
    if (mesh_swarm_coordinator_ && !status.daemon_available) {
        mesh_swarm_coordinator_->maintain(status);
    }
    if (!status.daemon_available) {
        mesh_bootstrap_attempted_.store(false);
        mesh_local_relay_configured_.store(false);
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
            mesh_local_relay_configured_.store(false);
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

        if (status.attached && active_config_.mesh_auto_configure_relay &&
            !mesh_local_relay_configured_.load()) {
            try {
                mesh_app_->configure_local_relay(
                    active_config_.mesh_default_ttl,
                    active_config_.mesh_relay_retransmit_count,
                    active_config_.mesh_relay_retransmit_interval_steps);
                mesh_local_relay_configured_.store(true);
                RCLCPP_INFO(
                    get_logger(), "Requested local Mesh relay with TTL %u",
                    static_cast<unsigned>(active_config_.mesh_default_ttl));
            } catch (const std::exception& error) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 10000,
                    "Automatic local Mesh relay configuration failed: %s",
                    error.what());
            }
        }
    }
    publish_mesh_status();
}

void ServiceNode::publish_mesh_status() {
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
            mesh_swarm_coordinator_->application_ready();
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
        if (!local_source && swarm_allowed) {
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
    if (!mesh_app_) {
        throw std::runtime_error(
            "Mesh bridge cannot send because enable_mesh is false");
    }
    const auto status = mesh_app_->status();
    if (!status.attached) {
        throw std::runtime_error(
            "Mesh bridge is waiting for bluetooth-meshd attachment");
    }
    if (mesh_swarm_coordinator_ &&
        !mesh_swarm_coordinator_->swarm_participating()) {
        return;  // Leaving a logical swarm never disables physical relaying.
    }
    mesh_app_->send(bridge.mesh_element_index, bridge.mesh_destination,
                    bridge.mesh_app_key_index, bridge.mesh_force_segmented,
                    payload);
}

void ServiceNode::publish_mesh_event(const mesh::Event& received) {
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
    if (!mesh_app_) {
        response->success = false;
        response->message = "Mesh is disabled; set enable_mesh: true";
        return;
    }

    try {
        const auto action = lower_copy(request->action);
        if (mesh_swarm_coordinator_) {
            throw std::runtime_error(
                "Fleet mode owns the persistent Mesh identity; use mesh/swarm for logical membership");
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
    if (!mesh_app_) {
        response->success = false;
        response->message = "Mesh is disabled; set enable_mesh: true";
        return;
    }

    try {
        const auto mode = lower_copy(request->mode);
        if (mesh_swarm_coordinator_ && mode != "send" && mode != "publish") {
            throw std::runtime_error(
                "Fleet mode permits application data only; use manual Mesh mode "
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
    if (!mesh_app_) {
        response->success = false;
        response->message = "Mesh is disabled; set enable_mesh: true";
        return;
    }

    try {
        const auto action = lower_copy(request->action);
        if (mesh_swarm_coordinator_ && action != "export_keys") {
            throw std::runtime_error(
                "Fleet mode owns keys and fixed addresses; use manual Mesh mode "
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
