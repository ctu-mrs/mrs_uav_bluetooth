// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bridge/transport_bridge_manager.hpp"

#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
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

bool exports(const config::SharedTopicConfig& item) {
    return item.mode == "export" || item.mode == "both";
}

bool imports(const config::SharedTopicConfig& item) {
    return item.mode == "import" || item.mode == "both";
}

std::string entry_key(const std::string& transport, uint16_t channel_id) {
    return transport + ":" + std::to_string(channel_id);
}

std::vector<uint8_t> frame_payload(uint16_t channel_id,
                                   const std::vector<uint8_t>& payload) {
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

std::optional<std::pair<uint16_t, std::vector<uint8_t>>> unframe_payload(
    const std::vector<uint8_t>& data,
    size_t offset) {
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

std::string safe_peer_token(std::string peer) {
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
        std::shared_ptr<GenericMessageBridge> runtime;
        rclcpp::SubscriptionBase::SharedPtr subscription;
        rclcpp::TimerBase::SharedPtr timer;
        std::optional<std::vector<uint8_t>> pending_payload;
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
        std::map<std::string, std::vector<uint8_t>> last_received_payload;
    };

    Impl(rclcpp::Node& selected_node, rclcpp::Logger selected_logger)
        : node(selected_node), logger(selected_logger) {}

    void dispatch(Entry& entry, const std::vector<uint8_t>& payload) {
        auto framed = frame_payload(entry.config.channel_id, payload);
        if (entry.config.transport == "advertisement") {
            if (!advertisement_sender) {
                throw std::runtime_error("advertisement bridge sender is unavailable");
            }
            advertisement_sender(framed);
            return;
        }

        if (!mesh_sender) {
            throw std::runtime_error("Mesh bridge sender is unavailable");
        }
        std::vector<uint8_t> access_payload{
            entry.config.mesh_vendor_opcode,
            static_cast<uint8_t>(entry.config.mesh_company_id & 0xffU),
            static_cast<uint8_t>(entry.config.mesh_company_id >> 8)};
        access_payload.insert(access_payload.end(), framed.begin(), framed.end());
        mesh_sender(entry.config, access_payload);
    }

    void handle_local_message(
        const std::string& key,
        const std::shared_ptr<rclcpp::SerializedMessage>& message) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        const auto found = entries.find(key);
        if (found == entries.end()) return;
        auto& entry = found->second;
        try {
            auto payload = entry.runtime->encode_payload(
                *message, entry.config.member_specs, entry.config.payload_format);
            if (entry.config.rate_hz > 0.0) {
                entry.pending_payload = std::move(payload);
            } else {
                dispatch(entry, payload);
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                logger, *node.get_clock(), 5000,
                "Could not encode %s bridge channel %u: %s",
                entry.config.transport.c_str(), entry.config.channel_id,
                error.what());
        }
    }

    void flush(const std::string& key) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        const auto found = entries.find(key);
        if (found == entries.end() || !found->second.pending_payload) return;
        auto& entry = found->second;
        auto payload = std::move(*entry.pending_payload);
        entry.pending_payload.reset();
        try {
            dispatch(entry, payload);
        } catch (const std::exception& error) {
            // A transport may legitimately be unavailable while BlueZ is
            // starting or a Mesh member is being provisioned. Keep that state
            // visible without emitting one warning at every bridge tick.
            RCLCPP_WARN_THROTTLE(
                logger, *node.get_clock(), 5000,
                "Could not send %s bridge channel %u: %s",
                entry.config.transport.c_str(), entry.config.channel_id,
                error.what());
        }
    }

    bool publish(Entry& entry,
                 const std::string& peer,
                 const std::vector<uint8_t>& payload,
                 bool deduplicate) {
        const auto token = safe_peer_token(peer);
        if (deduplicate) {
            const auto previous = entry.last_received_payload.find(token);
            if (previous != entry.last_received_payload.end() &&
                previous->second == payload) {
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
                payload, entry.config.member_specs, entry.config.payload_format);
            publisher->publish(serialized);
            if (deduplicate) entry.last_received_payload[token] = payload;
            return true;
        } catch (const std::exception& error) {
            RCLCPP_WARN(logger, "Could not decode %s bridge channel %u from %s: %s",
                        entry.config.transport.c_str(), entry.config.channel_id,
                        token.c_str(), error.what());
            return false;
        }
    }

    rclcpp::Node& node;
    rclcpp::Logger logger;
    std::recursive_mutex mutex;
    std::string node_topics_prefix;
    std::vector<std::string> peer_whitelist;
    std::map<uint16_t, std::string> mesh_peer_by_address;
    AdvertisementSender advertisement_sender;
    MeshSender mesh_sender;
    std::map<std::string, Entry> entries;
};

TransportBridgeManager::TransportBridgeManager(
    rclcpp::Node& node, rclcpp::Logger logger)
    : impl_(std::make_unique<Impl>(node, logger)) {}

TransportBridgeManager::~TransportBridgeManager() = default;

void TransportBridgeManager::set_advertisement_sender(AdvertisementSender sender) {
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->advertisement_sender = std::move(sender);
}

void TransportBridgeManager::set_mesh_sender(MeshSender sender) {
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->mesh_sender = std::move(sender);
}

void TransportBridgeManager::configure(
    const std::vector<config::SharedTopicConfig>& topics,
    const std::string& node_topics_prefix,
    const std::vector<std::string>& peer_whitelist) {
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
        entry.runtime = std::make_shared<GenericMessageBridge>(configured.message_type);

        if (!exports(configured)) continue;
        entry.subscription = entry.runtime->create_subscription(
            impl_->node, configured.export_topic,
            [this, key](std::shared_ptr<rclcpp::SerializedMessage> message) {
                impl_->handle_local_message(key, message);
            },
            10);
        if (configured.rate_hz > 0.0) {
            entry.timer = impl_->node.create_wall_timer(
                std::chrono::duration<double>(1.0 / configured.rate_hz),
                [this, key]() { impl_->flush(key); });
        }
        RCLCPP_INFO(impl_->logger,
                    "Configured %s bridge channel %u (%s) for %s",
                    configured.transport.c_str(), configured.channel_id,
                    configured.mode.c_str(), configured.export_topic.c_str());
    }
}

void TransportBridgeManager::clear() {
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    impl_->entries.clear();
}

bool TransportBridgeManager::handle_advertisement(
    const std::string& hostname,
    const std::string& mac,
    const std::vector<uint8_t>& payload) {
    const auto frame = unframe_payload(payload, 0);
    if (!frame) return false;
    std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
    const auto normalized_peer = util::lower_trim_copy(hostname);
    const auto normalized_mac = util::lower_trim_copy(mac);
    if (!impl_->peer_whitelist.empty() &&
        std::find(impl_->peer_whitelist.begin(), impl_->peer_whitelist.end(),
                  normalized_peer) == impl_->peer_whitelist.end() &&
        std::find(impl_->peer_whitelist.begin(), impl_->peer_whitelist.end(),
                  normalized_mac) == impl_->peer_whitelist.end()) {
        return false;
    }
    const auto found = impl_->entries.find(
        entry_key("advertisement", frame->first));
    if (found == impl_->entries.end() || !imports(found->second.config)) return false;
    const auto peer = hostname.empty() ? "mac_" + mac : hostname;
    return impl_->publish(found->second, peer, frame->second, true);
}

bool TransportBridgeManager::handle_mesh(
    uint16_t source,
    uint16_t key_index,
    const std::vector<uint8_t>& data) {
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
