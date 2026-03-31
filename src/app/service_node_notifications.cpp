// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <cstring>

namespace {

std::string lower_trim(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

std::string device_hostname_guess(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    if (mrs_uav_bluetooth::util::is_uav_hostname(device.name)) {
        return device.name;
    }
    if (mrs_uav_bluetooth::util::is_uav_hostname(device.alias)) {
        return device.alias;
    }
    return {};
}

bool device_can_host_peer_bridge(const mrs_uav_bluetooth::bluez::DeviceInfo& device,
                                 const mrs_uav_bluetooth::config::NodeConfig& config) {
    (void)config;
    return device.connected && device.services_resolved;
}

std::string bridge_characteristic_name_for_host(const std::string& host,
                                                const std::string& bridge_topic_path) {
    return "/" + mrs_uav_bluetooth::util::sanitize_topic_suffix(host) + bridge_topic_path;
}

std::string bridge_service_name_for_host(const std::string& host,
                                         const std::string& bridge_topic_path) {
    return "bridge:" + bridge_characteristic_name_for_host(host, bridge_topic_path);
}

std::string peer_topic_token(const std::string& peer_name, const std::string& mac) {
    auto token = mrs_uav_bluetooth::util::sanitize_topic_suffix(peer_name);
    if (token.empty() || token == "ble_device") {
        token = "peer_" + mrs_uav_bluetooth::util::sanitize_topic_suffix(mac);
    }
    if (!token.empty() && std::isdigit(static_cast<unsigned char>(token.front())) != 0) {
        token = "peer_" + token;
    }
    return token;
}

std::string trim_topic_segment(const std::string& value) {
    const auto start = value.find_first_not_of(" \t\r\n/");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n/");
    return value.substr(start, end - start + 1);
}

struct PublishRateSample {
    double monotonic_now;
    double hz;
};

PublishRateSample update_publish_rate(double last_publish_monotonic) {
    const auto now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (last_publish_monotonic > 0.0 && now > last_publish_monotonic) {
        return {now, 1.0 / (now - last_publish_monotonic)};
    }
    return {now, 0.0};
}

}  // namespace

namespace mrs_uav_bluetooth::app {

bool ServiceNode::has_ready_peer_time_bridge(const std::string& mac) const {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_) {
        return false;
    }

    const auto bridge_it = peers_->time_bridges().find(mac);
    return bridge_it != peers_->time_bridges().end() && bridge_it->second.status == "ready";
}

double ServiceNode::healthy_peer_time_bridge_last_activity_monotonic(
    const std::string& mac,
    double max_inactivity_s) const {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || max_inactivity_s <= 0.0) {
        return 0.0;
    }

    const auto bridge_it = peers_->time_bridges().find(mac);
    if (bridge_it == peers_->time_bridges().end()) {
        return 0.0;
    }

    const auto& bridge = bridge_it->second;
    const bool handshake_complete = bridge.status == "ready" &&
        bridge.time_notification_received &&
        bridge.time_writeback_received;
    if (!handshake_complete || bridge.last_activity_monotonic <= 0.0) {
        return 0.0;
    }

    const double inactivity_s = peers_->now_monotonic() - bridge.last_activity_monotonic;
    return inactivity_s <= max_inactivity_s ? bridge.last_activity_monotonic : 0.0;
}

bool ServiceNode::should_preserve_ready_bridge_during_expected_services_rediscovery(
    const bluez::DeviceInfo& device) const {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !device.connected || device.services_resolved) {
        return false;
    }

    const auto session_it = peers_->sessions().find(device.mac);
    const auto bridge_it = peers_->time_bridges().find(device.mac);
    if (session_it == peers_->sessions().end() || bridge_it == peers_->time_bridges().end()) {
        return false;
    }

    const auto& session = session_it->second;
    const auto& bridge = bridge_it->second;
    const bool handshake_complete = bridge.status == "ready" &&
        bridge.time_notification_received &&
        bridge.time_writeback_received;

    // BlueZ can pulse ServicesResolved=false during a fresh rediscovery even when the
    // already-handshaken time bridge is still healthy. Preserve that runtime until a
    // concrete teardown signal arrives, otherwise healthy bridges are torn down by churn.
    return handshake_complete &&
           session.desired &&
           !session.repair_requested &&
           !session.repair_in_progress;
}

bool ServiceNode::device_can_host_peer_bridge(const bluez::DeviceInfo& device) const {
    const bool pairing_ready = !active_config_.auto_pair || device.paired || device.bonded;
    return pairing_ready &&
           device.connected &&
           (device.services_resolved ||
            should_preserve_ready_bridge_during_expected_services_rediscovery(device));
}

void ServiceNode::clear_peer_runtime(const std::string& mac,
                                     const std::string& skip_characteristic_path) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !client_ || !import_bridges_) {
        return;
    }
    if (!peer_runtime_clear_in_progress_.insert(mac).second) {
        return;
    }

    std::string characteristic_path;

    if (auto bridge = peers_->time_bridges().find(mac); bridge != peers_->time_bridges().end()) {
        characteristic_path = bridge->second.characteristic_path;
        peers_->remove_time_bridge(mac);
    }

    state_lock.unlock();

    import_bridges_->clear_import_paths_for_mac(mac, *client_);

    if (!characteristic_path.empty() && characteristic_path != skip_characteristic_path) {
        client_->stop_notify(characteristic_path);
    }

    state_lock.lock();
    peer_runtime_clear_in_progress_.erase(mac);
}

void ServiceNode::refresh_import_bridges_for_device(const bluez::DeviceInfo& device) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (!import_bridges_ || !client_ || !peers_) {
        return;
    }

    const auto session_it = peers_->sessions().find(device.mac);
    const bool desired_peer = session_it != peers_->sessions().end() && session_it->second.desired;
    const auto peer_name = device_hostname_guess(device);
    const bool local_peer = !peer_name.empty() && lower_trim(peer_name) == lower_trim(hostname_);
    const bool functional_peer = desired_peer && !local_peer && device_can_host_peer_bridge(device);
    const auto now_mono = peers_ ? peers_->now_monotonic() : 0.0;
    const auto retry_period_s = std::max(1.0, active_config_.auto_connect_period);
    const auto missing_path_grace_s = std::max(8.0, retry_period_s * 4.0);

    std::set<std::string> desired_keys;
    if (functional_peer) {
        for (const auto& shared_topic : active_config_.shared_topics) {
            if (shared_topic.mode != "import" && shared_topic.mode != "both") {
                continue;
            }

            const auto registry_key = std::string("config-import::") + shared_topic.bridge_key + "::" + device.mac;
            desired_keys.insert(registry_key);

            bridge::TopicImportBridgeState state;
            state.mac = device.mac;
            state.requested_topic_name = shared_topic.import_topic_suffix;
            state.resolved_topic_name = peer_bridge_topic(device.mac, peer_name, shared_topic.import_topic_suffix);
            state.message_type = shared_topic.message_type;
            state.bridge_name = bridge_service_name_for_host(peer_name, shared_topic.bridge_topic_path);
            state.bridge_key = shared_topic.bridge_key;
            state.bridge_uuid = util::named_characteristic_uuid(
                bridge_characteristic_name_for_host(peer_name, shared_topic.bridge_topic_path));
            state.member_specs = shared_topic.member_specs;
            state.rate_hz = shared_topic.rate_hz;
            state.payload_format = shared_topic.payload_format;
            state.auto_managed = true;

            if (auto existing = bridge_registry_.imports().find(registry_key); existing != bridge_registry_.imports().end()) {
                state.path = existing->second.path;
                state.poll_timer = existing->second.poll_timer;
                state.pending_payload = existing->second.pending_payload;
                state.last_payload = existing->second.last_payload;
                state.last_publish_monotonic = existing->second.last_publish_monotonic;
                state.current_hz = existing->second.current_hz;
                if (existing->second.resolved_topic_name == state.resolved_topic_name &&
                    existing->second.message_type == state.message_type) {
                    state.publisher = existing->second.publisher;
                    state.runtime = existing->second.runtime;
                }
            }

            auto [it, inserted] = bridge_registry_.imports().insert_or_assign(registry_key, std::move(state));
            (void)inserted;
            import_bridges_->configure_import_bridge(registry_key, it->second, *client_);
        }
    }

    state_lock.unlock();

    if (functional_peer) {
        import_bridges_->refresh_import_paths_for_mac(device.mac, *client_);
    }

    prune_missing_import_bridges(device.mac, desired_keys, now_mono, missing_path_grace_s);

    if (desired_keys.empty()) {
        import_bridges_->clear_import_paths_for_mac(device.mac, *client_);
    }
}

void ServiceNode::prune_missing_import_bridges(const std::string& mac,
                                               const std::set<std::string>& desired_keys,
                                               double now_mono,
                                               double missing_path_grace_s) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !import_bridges_ || !client_) {
        return;
    }

    auto session_it = peers_->sessions().find(mac);
    if (session_it == peers_->sessions().end()) {
        return;
    }
    auto& session = session_it->second;
    std::vector<std::string> removed_paths;

    for (auto it = bridge_registry_.imports().begin(); it != bridge_registry_.imports().end();) {
        if (!it->second.auto_managed || it->second.mac != mac) {
            ++it;
            continue;
        }
        if (desired_keys.find(it->first) == desired_keys.end()) {
            session.import_bridge_missing_since.erase(it->first);
            if (!it->second.path.empty()) {
                removed_paths.push_back(it->second.path);
            }
            import_bridges_->destroy_import_bridge(it->second);
            it = bridge_registry_.imports().erase(it);
            continue;
        }
        if (!it->second.path.empty()) {
            session.import_bridge_missing_since.erase(it->first);
            ++it;
            continue;
        }

        auto& missing_since = session.import_bridge_missing_since[it->first];
        if (missing_since <= 0.0) {
            missing_since = now_mono;
            ++it;
            continue;
        }
        if ((now_mono - missing_since) < missing_path_grace_s) {
            ++it;
            continue;
        }

        session.import_bridge_missing_since.erase(it->first);
        if (!it->second.path.empty()) {
            removed_paths.push_back(it->second.path);
        }
        import_bridges_->destroy_import_bridge(it->second);
        it = bridge_registry_.imports().erase(it);
    }

    state_lock.unlock();

    for (const auto& path : removed_paths) {
        client_->stop_notify(path);
    }
}

std::string ServiceNode::peer_status_topic(const std::string& mac, const std::string& peer_name) const {
    return util::normalize_ros_topic(active_config_.node_topics_prefix + "/peers/" +
                                     peer_topic_token(peer_name, mac) + "/time_status");
}

std::string ServiceNode::peer_bridge_topic(const std::string& mac,
                                           const std::string& peer_name,
                                           const std::string& requested_topic_suffix) const {
    std::string topic = active_config_.node_topics_prefix + "/peers/" + peer_topic_token(peer_name, mac);
    const auto suffix = trim_topic_segment(requested_topic_suffix);
    if (!suffix.empty()) {
        topic += "/" + suffix;
    }
    return util::normalize_ros_topic(topic);
}

void ServiceNode::publish_peer_time_status(peer::PeerTimeBridge& bridge) const {
    auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<mrs_uav_bluetooth::msg::BlePeerTimeStatus>>(bridge.publisher);
    if (!publisher) {
        return;
    }

    mrs_uav_bluetooth::msg::BlePeerTimeStatus msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = util::sanitize_topic_suffix(hostname_);
    msg.mac = bridge.mac;
    msg.peer_name = bridge.peer_name;
    msg.peer_stamp.sec = static_cast<int32_t>(bridge.last_time_value_ns / 1000000000ULL);
    msg.peer_stamp.nanosec = static_cast<uint32_t>(bridge.last_time_value_ns % 1000000000ULL);
    msg.last_rtt_s = bridge.last_rtt_s;
    publisher->publish(msg);
}

void ServiceNode::on_notification(const std::vector<uint8_t>& data,
                                  const std::string& uuid,
                                  const std::string& characteristic_path) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    RCLCPP_DEBUG(get_logger(), "[node] on_notification: uuid=%s path=%s %zu bytes",
                 uuid.c_str(), characteristic_path.c_str(), data.size());
    if (!cache_ || !ros_) {
        return;
    }

    std::string mac;
    std::optional<std::string> clear_runtime_mac;
    if (const auto characteristic = cache_->characteristic(characteristic_path)) {
        if (const auto service = cache_->service(characteristic->service_path)) {
            if (const auto device = cache_->device(service->device_path)) {
                mac = device->mac;
            }
        }
    }

    ros_->status_publisher().publish_notification(mac, characteristic_path, uuid, data, hostname_);

    if (!mac.empty()) {
        const auto device = cache_->device_by_mac(mac);
        if (!device || !device_can_host_peer_bridge(*device)) {
            clear_runtime_mac = mac;
        }
    }

    if (clear_runtime_mac) {
        state_lock.unlock();
        clear_peer_runtime(*clear_runtime_mac);
        return;
    }

    if (import_bridges_) {
        import_bridges_->buffer_notification_payload(mac, characteristic_path, data);
    }

    if (!peers_) {
        return;
    }

    auto bridge_it = peers_->time_bridges().find(mac);
    if (bridge_it == peers_->time_bridges().end() ||
        bridge_it->second.characteristic_path != characteristic_path ||
        data.size() < sizeof(uint64_t)) {
        return;
    }

    uint64_t peer_time_ns = 0;
    std::memcpy(&peer_time_ns, data.data(), sizeof(peer_time_ns));
    auto& bridge = bridge_it->second;
    bridge.last_activity_monotonic = peers_->now_monotonic();
    bridge.last_time_value_ns = peer_time_ns;
    bridge.time_notification_received = true;
    const auto publish_rate = update_publish_rate(bridge.last_publish_monotonic);
    bridge.last_publish_monotonic = publish_rate.monotonic_now;
    bridge.current_hz = publish_rate.hz;
    if (bridge.time_writeback_received) {
        bridge.status = "ready";
        bridge.detail = "time notification";
        if (auto session_it = peers_->sessions().find(mac); session_it != peers_->sessions().end()) {
            session_it->second.time_bridge_healthy_this_connection = true;
        }
        publish_peer_time_status(bridge);
    } else {
        bridge.status = "subscribing";
        bridge.detail = "time notification received, awaiting writeback";
    }

    if (!bridge.writeback_descriptor_path.empty()) {
        client_->write_descriptor_async(bridge.writeback_descriptor_path, data);
    }
}

void ServiceNode::handle_time_writeback(const std::vector<uint8_t>& payload,
                                        const std::string& device_path,
                                        uint64_t received_time_ns) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || payload.size() < sizeof(uint64_t)) {
        return;
    }

    std::string mac;
    std::optional<std::string> clear_runtime_mac;
    if (!device_path.empty() && cache_) {
        if (const auto device = cache_->device(device_path)) {
            mac = device->mac;
        }
    }

    if (mac.empty()) {
        for (const auto& [candidate_mac, bridge] : peers_->time_bridges()) {
            if (!device_path.empty() && !bridge.characteristic_path.empty() &&
                bridge.characteristic_path.find(device_path + "/") == 0) {
                mac = candidate_mac;
                break;
            }
        }
    }

    if (mac.empty()) {
        return;
    }

    if (cache_) {
        const auto device = cache_->device_by_mac(mac);
        if (!device || !device_can_host_peer_bridge(*device)) {
            clear_runtime_mac = mac;
        }
    }

    if (clear_runtime_mac) {
        state_lock.unlock();
        clear_peer_runtime(*clear_runtime_mac);
        return;
    }

    auto bridge_it = peers_->time_bridges().find(mac);
    if (bridge_it == peers_->time_bridges().end()) {
        return;
    }

    uint64_t echoed_time_ns = 0;
    std::memcpy(&echoed_time_ns, payload.data(), sizeof(echoed_time_ns));
    if (echoed_time_ns == 0 || received_time_ns < echoed_time_ns) {
        return;
    }

    auto& bridge = bridge_it->second;
    bridge.last_activity_monotonic = peers_->now_monotonic();
    bridge.last_rtt_s = std::max(0.0, static_cast<double>(received_time_ns - echoed_time_ns) / 1e9);
    bridge.time_writeback_received = true;
    if (!bridge.time_notification_received) {
        bridge.status = "subscribing";
        bridge.detail = "peer time writeback received, awaiting notifications";
        return;
    }

    bridge.status = "ready";
    bridge.detail = "time writeback";
    if (auto session_it = peers_->sessions().find(mac); session_it != peers_->sessions().end()) {
        session_it->second.time_bridge_healthy_this_connection = true;
    }
    publish_peer_time_status(bridge);
}

}  // namespace mrs_uav_bluetooth::app