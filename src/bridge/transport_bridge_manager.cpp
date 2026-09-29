// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bridge/transport_bridge_manager.cpp
/// \brief Implements the transport bridge manager component of the transport-independent ROS message bridge.

#include "mrs_uav_bluetooth/bridge/transport_bridge_manager.hpp"

#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace mrs_uav_bluetooth::bridge {

namespace {

constexpr uint8_t kFrameMagic0 = 'M';
constexpr uint8_t kFrameMagic1 = 'B';
constexpr uint8_t kFrameVersion = 1;
constexpr size_t kFrameHeaderSize = 5;

/// \brief Test whether a bridge sends local ROS messages over Bluetooth.
/// \param item Bridge configuration whose direction is tested.
/// \return True for `export` and `both` bridges; otherwise false.
bool exports(const config::SharedTopicConfig& item) {
    // Only export-capable entries need a ROS subscription and send timer.
    return item.mode == "export" || item.mode == "both";
}

/// \brief Test whether a bridge publishes received Bluetooth messages to ROS.
/// \param item Bridge configuration whose direction is tested.
/// \return True for `import` and `both` bridges; otherwise false.
bool imports(const config::SharedTopicConfig& item) {
    // Only import-capable entries accept decoded peer payloads.
    return item.mode == "import" || item.mode == "both";
}

/// \brief Build the stable lookup key for one transport channel.
/// \param transport `advertisement` or `mesh` transport name.
/// \param channel_id Application channel number within that transport.
/// \return Transport name and channel joined into one map key.
std::string entry_key(const std::string& transport, uint16_t channel_id) {
    // Including the transport lets Advertisement and Mesh reuse a channel number.
    return transport + ":" + std::to_string(channel_id);
}

/// \brief Prefix a bridge payload with its transport-independent channel header.
/// \param channel_id Application channel used to select the receiving bridge.
/// \param payload Encoded ROS fields to carry after the header.
/// \return Versioned frame containing the channel and payload bytes.
std::vector<uint8_t> frame_payload(uint16_t channel_id,
                                   const std::vector<uint8_t>& payload) {
    // The fixed magic and version reject unrelated service data before decoding.
    std::vector<uint8_t> framed;
    framed.reserve(kFrameHeaderSize + payload.size());
    framed.push_back(kFrameMagic0);
    framed.push_back(kFrameMagic1);
    framed.push_back(kFrameVersion);
    framed.push_back(static_cast<uint8_t>(channel_id & 0xffU));
    framed.push_back(static_cast<uint8_t>(channel_id >> 8));
    framed.insert(framed.end(), payload.begin(), payload.end());
    return framed;
}

/// \brief Validate and remove a bridge channel header from received bytes.
/// \param data Advertisement or Mesh bytes containing a bridge frame.
/// \param offset First possible header byte after any transport-specific prefix.
/// \return Channel and payload when the header is complete and supported.
std::optional<std::pair<uint16_t, std::vector<uint8_t>>> unframe_payload(
    const std::vector<uint8_t>& data,
    size_t offset) {
    // Reject short, unrelated, or future-version frames before indexing fields.
    if (data.size() < offset + kFrameHeaderSize ||
        data[offset] != kFrameMagic0 || data[offset + 1] != kFrameMagic1 ||
        data[offset + 2] != kFrameVersion) {
        return std::nullopt;
    }
    const uint16_t channel_id = static_cast<uint16_t>(
        data[offset + 3] | (static_cast<uint16_t>(data[offset + 4]) << 8));
    return std::pair<uint16_t, std::vector<uint8_t>>{
        channel_id,
        std::vector<uint8_t>(data.begin() + offset + kFrameHeaderSize, data.end())};
}

/// \brief Convert a peer identity into a safe ROS topic path component.
/// \param peer Resolved hostname, address, or Mesh fallback name.
/// \return Nonempty topic-safe peer token with a ROS-valid leading character.
std::string safe_peer_token(std::string peer) {
    // Prefix numeric identities with the ROS-compatible peer label.
    peer = util::sanitize_topic_suffix(peer);
    if (peer.empty()) peer = "unknown";
    if (std::isdigit(static_cast<unsigned char>(peer.front())) != 0) {
        peer = "peer_" + peer;
    }
    return peer;
}

}  // namespace

struct TransportBridgeManager::Impl {
    struct Entry {
        config::SharedTopicConfig config;
        uint64_t generation{0};
        uint64_t revision{0};
        std::shared_ptr<GenericMessageBridge> runtime;
        rclcpp::SubscriptionBase::SharedPtr subscription;
        rclcpp::TimerBase::SharedPtr timer;
        std::optional<std::vector<uint8_t>> pending_payload;
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
        // BlueZ can keep several random-address objects for one advertiser.
        // Remember the recent packet window to filter replay from alternating
        // cached scan snapshots.
        std::map<std::string, std::deque<std::pair<std::chrono::steady_clock::time_point,
                                                   std::vector<uint8_t>>>> recent_payloads;
    };

    /// \brief Store the ROS resources used to create bridge endpoints later.
    /// \param selected_node Node that owns subscriptions, timers, and publishers.
    /// \param selected_logger Logger used for bridge diagnostics.
    Impl(rclcpp::Node& selected_node, rclcpp::Logger selected_logger)
        : node(selected_node), logger(selected_logger) {
        // Senders remain unset until ServiceNode has constructed each transport.
    }

    // A transport callback can make a synchronous D-Bus call. Invoke it after
    // releasing mutex so the D-Bus Mesh receive callback can acquire the same
    // lock to publish an incoming bridge message.
    /// \brief Send one encoded bridge value through its configured transport.
    /// \param config Bridge routing and framing configuration.
    /// \param payload Encoded ROS value without transport framing.
    /// \param advertisement_sender Callback that updates advertisement data.
    /// \param mesh_sender Callback that submits a Mesh access message.
    /// \return True when the transport accepts ownership of the value.
    static bool dispatch(const config::SharedTopicConfig& config,
                         const std::vector<uint8_t>& payload,
                         const AdvertisementSender& advertisement_sender,
                         const MeshSender& mesh_sender) {
        // Advertisement optionally uses bare bytes; Mesh always carries its vendor prefix.
        if (config.transport == "advertisement") {
            if (!advertisement_sender) {
                throw std::runtime_error("advertisement bridge sender is unavailable");
            }
            auto framed = config.advertisement_bare
                ? payload
                : frame_payload(config.channel_id, payload);
            return advertisement_sender(framed);
        }

        if (!mesh_sender) {
            throw std::runtime_error("Mesh bridge sender is unavailable");
        }
        std::vector<uint8_t> access_payload{
            config.mesh_vendor_opcode,
            static_cast<uint8_t>(config.mesh_company_id & 0xffU),
            static_cast<uint8_t>(config.mesh_company_id >> 8)};
        const auto framed = frame_payload(config.channel_id, payload);
        access_payload.insert(access_payload.end(), framed.begin(), framed.end());
        return mesh_sender(config, access_payload);
    }

    /// \brief Encode a local ROS message and schedule its Bluetooth transfer.
    /// \param key Transport/channel map key identifying the bridge entry.
    /// \param generation Configuration generation captured by the subscription.
    /// \param message Type-erased ROS sample received on the export topic.
    void handle_local_message(
        const std::string& key, uint64_t generation,
        const std::shared_ptr<rclcpp::SerializedMessage>& message) {
        // Ignore callbacks retained by ROS after their overlay generation was replaced.
        config::SharedTopicConfig config;
        try {
            {
                std::lock_guard<std::recursive_mutex> lock(mutex);
                const auto found = entries.find(key);
                if (found == entries.end() ||
                    found->second.generation != generation) return;
                auto& entry = found->second;
                config = entry.config;
                auto payload = entry.runtime->encode_payload(
                    *message, config.member_specs, config.payload_format);
                entry.pending_payload = std::move(payload);
                ++entry.revision;
                if (config.rate_hz > 0.0) return;
            }
            flush(key, generation);
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                logger, *node.get_clock(), 5000,
                "Could not encode %s bridge channel %u: %s",
                config.transport.c_str(), config.channel_id, error.what());
        }
    }

    /// \brief Attempt the newest buffered value for one export bridge.
    /// \param key Transport/channel map key identifying the bridge entry.
    /// \param generation Configuration generation captured by the timer.
    void flush(const std::string& key, uint64_t generation) {
        // Serialize outbound work with configure/clear, but release the entry
        // mutex before D-Bus calls so receive callbacks can still publish.
        // Service callbacks try-lock their config mutex, avoiding lock inversion
        // when a configuration transaction is waiting for this dispatch.
        std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex);
        config::SharedTopicConfig config;
        AdvertisementSender advertisement;
        MeshSender mesh;
        std::vector<uint8_t> payload;
        uint64_t revision;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex);
            const auto found = entries.find(key);
            if (found == entries.end() || found->second.generation != generation ||
                !found->second.pending_payload) return;
            auto& entry = found->second;
            config = entry.config;
            advertisement = advertisement_sender;
            mesh = mesh_sender;
            revision = entry.revision;
            payload = std::move(*entry.pending_payload);
            entry.pending_payload.reset();
        }
        bool accepted = false;
        try {
            accepted = dispatch(config, payload, advertisement, mesh);
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                logger, *node.get_clock(), 5000,
                "Could not send %s bridge channel %u: %s",
                config.transport.c_str(), config.channel_id, error.what());
        }
        if (accepted) return;
        std::lock_guard<std::recursive_mutex> lock(mutex);
        const auto found = entries.find(key);
        // A newer value supersedes this attempt, even if that value has
        // already been sent. Generation and revision checks retain the newest overlay.
        if (found != entries.end() && found->second.generation == generation &&
            found->second.revision == revision && !found->second.pending_payload)
            found->second.pending_payload = std::move(payload);
    }

    /// \brief Decode peer bytes and publish them on the bridge's ROS import topic.
    /// \param entry Matching configured import bridge and its runtime codec.
    /// \param peer Stable peer identity used in the ROS topic path.
    /// \param payload Encoded ROS fields after transport framing is removed.
    /// \param deduplicate Whether repeated cached advertisement packets are suppressed.
    /// \return True after publication or intentional duplicate suppression.
    bool publish(Entry& entry,
                 const std::string& peer,
                 const std::vector<uint8_t>& payload,
                 bool deduplicate) {
        // Keep packet history for this overlay because BlueZ may alternate
        // cached random-address records long after one stopped changing.
        const auto token = safe_peer_token(peer);
        if (deduplicate) {
            auto& recent = entry.recent_payloads[token];
            if (std::any_of(recent.begin(), recent.end(), [&](const auto& seen) {
                    return seen.second == payload;
                })) {
                return true;
            }
        }

        try {
            auto& base_publisher = entry.publishers[token];
            if (!base_publisher) {
                const auto transport_segment = entry.config.transport == "mesh"
                    ? "mesh" : "le";
                auto topic = util::normalize_ros_topic(
                    node_topics_prefix + "/" + transport_segment + "/peers/" +
                    token + "/" + entry.config.import_topic_suffix);
                base_publisher = entry.runtime->create_publisher(node, topic, 10);
                RCLCPP_INFO(logger,
                            "Bridge channel %u publishes peer %s on %s",
                            entry.config.channel_id, token.c_str(), topic.c_str());
            }
            auto publisher =
                std::dynamic_pointer_cast<rclcpp::GenericPublisher>(base_publisher);
            if (!publisher) return false;
            auto serialized = entry.runtime->decode_payload(
                payload, entry.config.member_specs, entry.config.payload_format,
                entry.config.decode_assignments);
            publisher->publish(serialized);
            if (deduplicate) remember(entry, token, payload);
            return true;
        } catch (const std::exception& error) {
            RCLCPP_WARN(logger, "Could not decode %s bridge channel %u from %s: %s",
                        entry.config.transport.c_str(), entry.config.channel_id,
                        token.c_str(), error.what());
            return false;
        }
    }

    /// \brief Add one payload to an advertisement peer's bounded replay history.
    /// \param entry Configured advertisement import bridge that owns the history.
    /// \param token Topic-safe peer identity shared with the publication path.
    /// \param payload Decoded bridge bytes to suppress if BlueZ reports them again.
    static void remember(Entry& entry,
                         const std::string& token,
                         const std::vector<uint8_t>& payload) {
        auto& recent = entry.recent_payloads[token];
        if (std::any_of(recent.begin(), recent.end(), [&](const auto& seen) {
                return seen.second == payload;
            })) {
            return;
        }
        recent.emplace_back(std::chrono::steady_clock::now(), payload);
        // More than six minutes at the example's 10 Hz rate covers address
        // rotations without allowing retained process-lifetime state to grow.
        if (recent.size() > 4096) recent.pop_front();
    }

    rclcpp::Node& node;
    rclcpp::Logger logger;
    std::recursive_mutex mutex;
    std::mutex dispatch_mutex;
    uint64_t next_generation{0};
    std::string node_topics_prefix;
    std::vector<std::string> peer_whitelist;
    std::map<uint16_t, std::string> mesh_peer_by_address;
    AdvertisementSender advertisement_sender;
    MeshSender mesh_sender;
    std::map<std::string, Entry> entries;
};

TransportBridgeManager::TransportBridgeManager(
    rclcpp::Node& node, rclcpp::Logger logger)
    : impl_(std::make_unique<Impl>(node, logger)) {
    // Construct state first; ServiceNode installs transport senders separately.
}

TransportBridgeManager::~TransportBridgeManager() = default;

void TransportBridgeManager::set_advertisement_sender(AdvertisementSender sender) {
    // Install the guarded callback used by advertisement export bridges.
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->advertisement_sender = std::move(sender);
}

void TransportBridgeManager::set_mesh_sender(MeshSender sender) {
    // Install the guarded callback used by Mesh export bridges.
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->mesh_sender = std::move(sender);
}

void TransportBridgeManager::configure(
    const std::vector<config::SharedTopicConfig>& topics,
    const std::string& node_topics_prefix,
    const std::vector<std::string>& peer_whitelist) {
    // Replace the entire bridge generation while no outbound callback can run.
    std::lock_guard<std::mutex> dispatch_lock(impl_->dispatch_mutex);
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->entries.clear();
    impl_->node_topics_prefix = util::normalize_ros_topic(node_topics_prefix);
    impl_->peer_whitelist.clear();
    impl_->mesh_peer_by_address.clear();
    for (const auto& peer : peer_whitelist) {
        const auto normalized = util::lower_trim_copy(peer);
        impl_->peer_whitelist.push_back(normalized);
        // Automatic Mesh uses the numeric hostname suffix as its immutable
        // unicast address. Resolve it once so imported Mesh and LE topics use
        // the same human-readable peer namespace.
        const auto non_digit = normalized.find_last_not_of("0123456789");
        if (non_digit == std::string::npos ||
            non_digit + 1 == normalized.size()) continue;
        try {
            const auto number = std::stoul(normalized.substr(non_digit + 1));
            if (number > 0 && number <= 0x7fff)
                impl_->mesh_peer_by_address.emplace(
                    static_cast<uint16_t>(number), normalized);
        } catch (const std::exception&) {
            // Non-UAV names remain valid for LE overlays. A Mesh source
            // without a configured hostname uses its unicast address below.
        }
    }

    const auto advertisement_exports = std::count_if(
        topics.begin(), topics.end(), [](const auto& item) {
            // Advertisement has only one mutable application-data field.
            return item.transport == "advertisement" && exports(item);
        });
    if (advertisement_exports > 1) {
        throw std::runtime_error(
            "Only one advertisement shared-topic exporter can be active; "
            "a BLE advertisement has one dynamic user-data field");
    }

    for (const auto& configured : topics) {
        if (configured.transport == "gatt") continue;
        const auto key = entry_key(configured.transport, configured.channel_id);
        auto [iterator, inserted] = impl_->entries.try_emplace(key);
        if (!inserted) {
            throw std::runtime_error(
                "Duplicate " + configured.transport + " bridge channel_id " +
                std::to_string(configured.channel_id));
        }
        auto& entry = iterator->second;
        entry.config = configured;
        const auto generation = entry.generation = ++impl_->next_generation;
        entry.runtime = std::make_shared<GenericMessageBridge>(configured.message_type);

        if (!exports(configured)) continue;
        entry.subscription = entry.runtime->create_subscription(
            impl_->node, configured.export_topic,
            [this, key, generation](std::shared_ptr<rclcpp::SerializedMessage> message) {
                // Bind the entry generation so stale overlay callbacks are harmless.
                impl_->handle_local_message(key, generation, message);
            },
            10);
        // Unthrottled bridges dispatch immediately from the subscription.
        // Their timer only retries a value whose transport was unavailable.
        const double period = configured.rate_hz > 0.0
            ? 1.0 / configured.rate_hz : 0.02;
        entry.timer = impl_->node.create_wall_timer(
            std::chrono::duration<double>(period),
            [this, key, generation]() {
                // Retry the newest unsent value without reviving an old overlay.
                impl_->flush(key, generation);
            });
        RCLCPP_INFO(impl_->logger,
                    "Configured %s bridge channel %u (%s) for %s",
                    configured.transport.c_str(), configured.channel_id,
                    configured.mode.c_str(), configured.export_topic.c_str());
    }
}

void TransportBridgeManager::clear() {
    // Destroy timers and endpoints only after an in-flight sender finishes.
    std::lock_guard<std::mutex> dispatch_lock(impl_->dispatch_mutex);
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->entries.clear();
}

bool TransportBridgeManager::handle_advertisement(
    const std::string& hostname,
    const std::string& mac,
    const std::vector<uint8_t>& payload) {
    // Admit the peer before selecting either the configured bare or framed channel.
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    const auto normalized_peer = util::lower_trim_copy(hostname);
    const auto normalized_mac = util::lower_trim_copy(mac);
    if (!impl_->peer_whitelist.empty() &&
        std::find(impl_->peer_whitelist.begin(), impl_->peer_whitelist.end(),
                  normalized_peer) == impl_->peer_whitelist.end() &&
        std::find(impl_->peer_whitelist.begin(), impl_->peer_whitelist.end(),
                  normalized_mac) == impl_->peer_whitelist.end()) return false;
    const auto peer = hostname.empty() ? "mac_" + mac : hostname;
    const auto bare = impl_->entries.find(entry_key("advertisement", 0));
    if (bare != impl_->entries.end() && bare->second.config.advertisement_bare) {
        if (!imports(bare->second.config)) return false;
        return impl_->publish(bare->second, peer, payload, true);
    }
    const auto frame = unframe_payload(payload, 0);
    if (!frame) return false;
    const auto found = impl_->entries.find(entry_key("advertisement", frame->first));
    if (found == impl_->entries.end() || !imports(found->second.config)) return false;
    return impl_->publish(found->second, peer, frame->second, true);
}


void TransportBridgeManager::remember_advertisement(
    const std::string& hostname,
    const std::string& mac,
    const std::vector<uint8_t>& payload) {
    // Apply the same admission, peer naming, and frame selection used by live data.
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    const auto normalized_peer = util::lower_trim_copy(hostname);
    const auto normalized_mac = util::lower_trim_copy(mac);
    if (!impl_->peer_whitelist.empty() &&
        std::find(impl_->peer_whitelist.begin(), impl_->peer_whitelist.end(),
                  normalized_peer) == impl_->peer_whitelist.end() &&
        std::find(impl_->peer_whitelist.begin(), impl_->peer_whitelist.end(),
                  normalized_mac) == impl_->peer_whitelist.end()) return;

    const auto token = safe_peer_token(hostname.empty() ? "mac_" + mac : hostname);
    const auto bare = impl_->entries.find(entry_key("advertisement", 0));
    if (bare != impl_->entries.end() && bare->second.config.advertisement_bare) {
        if (imports(bare->second.config)) {
            Impl::remember(bare->second, token, payload);
        }
        return;
    }

    const auto frame = unframe_payload(payload, 0);
    if (!frame) return;
    const auto found = impl_->entries.find(entry_key("advertisement", frame->first));
    if (found == impl_->entries.end() || !imports(found->second.config)) return;
    Impl::remember(found->second, token, frame->second);
}
bool TransportBridgeManager::handle_mesh(
    uint16_t source,
    uint16_t key_index,
    const std::vector<uint8_t>& data) {
    // Accept both BlueZ callback forms: vendor opcode retained or already stripped.
    size_t frame_offset = 0;
    if (data.size() >= 3 && data[0] >= 0xc0) frame_offset = 3;
    const auto frame = unframe_payload(data, frame_offset);
    if (!frame) return false;

    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    const auto found = impl_->entries.find(entry_key("mesh", frame->first));
    if (found == impl_->entries.end() || !imports(found->second.config)) return false;
    const auto& configured = found->second.config;
    if (key_index != configured.mesh_app_key_index) return false;
    if (frame_offset == 3 &&
        (data[0] != configured.mesh_vendor_opcode ||
         data[1] != static_cast<uint8_t>(configured.mesh_company_id & 0xffU) ||
         data[2] != static_cast<uint8_t>(configured.mesh_company_id >> 8))) {
        return false;
    }
    const auto peer = impl_->mesh_peer_by_address.find(source);
    if (peer == impl_->mesh_peer_by_address.end() &&
        !impl_->peer_whitelist.empty()) return false;
    const auto peer_name = peer != impl_->mesh_peer_by_address.end()
        ? peer->second : "unicast_" + std::to_string(source);
    return impl_->publish(found->second, peer_name, frame->second, false);
}

}  // namespace mrs_uav_bluetooth::bridge
