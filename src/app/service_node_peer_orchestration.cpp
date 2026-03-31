// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <cctype>
#include <future>
#include <optional>
#include <rclcpp/create_timer.hpp>
#include <thread>

namespace {

constexpr double kLocalReconfigureGraceMin = 5.0;
constexpr double kRemoteGattRepairGraceMin = 10.0;
constexpr int kRemoteGattRepairMinChecks = 3;

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

std::string normalize_mac_key(std::string value) {
    value = lower_trim(std::move(value));
    value.erase(std::remove_if(value.begin(), value.end(),
                               [](unsigned char ch) {
                                   return std::isspace(ch) != 0 || ch == '-';
                               }),
                value.end());
    std::replace(value.begin(), value.end(), '_', ':');
    return value;
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

bool device_has_local_security(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    return device.paired || device.bonded || device.trusted;
}

bool device_needs_forget(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    return device.connected || device.services_resolved || device_has_local_security(device);
}

bool device_is_secure_peer(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    return device.connected && device.trusted && (device.paired || device.bonded);
}

bool device_can_host_peer_bridge(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    return device_is_secure_peer(device) && device.services_resolved;
}

bool local_is_preferred_initiator(const std::string& local_hostname,
                                  const std::string& local_adapter_mac,
                                  const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
                                  const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    const auto local_name = lower_trim(local_hostname);
    const auto peer_name = lower_trim(session.peer_name.empty() ? device_hostname_guess(device)
                                                                : session.peer_name);
    if (!local_name.empty() && !peer_name.empty() && local_name != peer_name) {
        return local_name < peer_name;
    }

    const auto local_mac = normalize_mac_key(local_adapter_mac);
    const auto peer_mac = normalize_mac_key(device.mac);
    if (!local_mac.empty() && !peer_mac.empty() && local_mac != peer_mac) {
        return local_mac < peer_mac;
    }

    return true;
}

std::string passive_peer_wait_detail(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    if (!device.connected) {
        return "awaiting connection from preferred initiator";
    }
    if (!(device.paired || device.bonded)) {
        return "connected, waiting for peer-initiated pairing";
    }
    return "connected, waiting for service discovery";
}

template<typename DurationT, typename CallbackT>
rclcpp::TimerBase::SharedPtr create_grouped_wall_timer(
    rclcpp::Node& node,
    DurationT period,
    CallbackT&& callback,
    const rclcpp::CallbackGroup::SharedPtr& group) {
    return rclcpp::create_wall_timer(
        period,
        std::forward<CallbackT>(callback),
        group,
        node.get_node_base_interface().get(),
        node.get_node_timers_interface().get());
}

double remote_gatt_repair_grace_s(const mrs_uav_bluetooth::peer::PeerConnectionSession& session) {
    return std::max(kRemoteGattRepairGraceMin,
                    session.services_wait_grace_s > 0.0 ? session.services_wait_grace_s : kRemoteGattRepairGraceMin);
}

}  // namespace

namespace mrs_uav_bluetooth::app {

void ServiceNode::note_pair_attempt_result(const std::string& mac,
                                           bool success,
                                           const std::string& error_detail) {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_) {
        return;
    }

    auto session_it = peers_->sessions().find(mac);
    if (session_it == peers_->sessions().end()) {
        return;
    }

    auto& session = session_it->second;
    session.last_security_attempt_monotonic = peers_->now_monotonic();
    if (success) {
        session.pairing_failures = 0;
        peers_->clear_pairing_repair(session);
        return;
    }

    session.pairing_failures += 1;
    const auto normalized_error = lower_trim(error_detail);
    const bool authentication_failed = normalized_error.find("authentication failed") != std::string::npos ||
        normalized_error.find("authentication rejected") != std::string::npos ||
        normalized_error.find("authentication canceled") != std::string::npos;

    if (authentication_failed) {
        peers_->request_pairing_repair(session, "pair auth failed, resetting stale security");
        session.last_repair_monotonic = 0.0;
        return;
    }

    session.repair_in_progress = false;
    session.phase = "recovering";
    session.detail = error_detail.empty() ? "pair failed" : "pair failed: " + error_detail;
}

bool ServiceNode::should_allow_pairing_request(const std::string& event_type,
                                               const std::string& device_path) {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !cache_) {
        return false;
    }

    const auto device = cache_->device(device_path);
    if (!device) {
        return false;
    }

    const auto peer_name = device_hostname_guess(*device);
    auto& session = peers_->get_or_create_session(device->mac, peer_name);
    peers_->sync_device(*device, active_config_, peer_name);

    if (!session.desired) {
        log_warn_coalesced("pairing-rejected-policy:" + device->mac + ":" + event_type,
                           "[node] rejecting pairing request for " + device->mac +
                               " due to current config");
        return false;
    }

    if (session.stale_pairing_detected || session.repair_in_progress) {
        log_warn_coalesced("pairing-rejected-stale:" + device->mac + ":" + event_type,
                           "[node] rejecting pairing request for " + device->mac +
                               " because local bond is stale and will be repaired");
        return false;
    }

    if (device->blocked) {
        return false;
    }

    const auto now = peers_->now_monotonic();
    const double retry_period_s = std::max(0.5, active_config_.auto_connect_period);
    const double local_reconfigure_grace_s = std::max(kLocalReconfigureGraceMin, retry_period_s * 2.0);
    if (local_server_rebuild_monotonic_ > 0.0 &&
        (now - local_server_rebuild_monotonic_) < local_reconfigure_grace_s) {
        log_warn_coalesced("pairing-rejected-reconfigure:" + device->mac + ":" + event_type,
                           "[node] rejecting pairing request for " + device->mac +
                               " while local GATT/server rebuild is still settling");
        return false;
    }

    return true;
}

void ServiceNode::schedule_peer_reconcile(std::chrono::milliseconds delay) {
    if (!can_run_callbacks() || !peers_ || !client_) {
        return;
    }

    const auto arm_delay = std::max(delay, std::chrono::milliseconds(1));
    const auto requested_deadline = std::chrono::steady_clock::now() + arm_delay;

    if (peer_timer_ && peer_reconcile_deadline_ != std::chrono::steady_clock::time_point{} &&
        requested_deadline >= peer_reconcile_deadline_) {
        return;
    }

    if (peer_timer_) {
        peer_timer_->cancel();
        peer_timer_.reset();
    }

    peer_reconcile_deadline_ = requested_deadline;
    peer_timer_ = create_grouped_wall_timer(*this, arm_delay, [this]() {
        if (!can_run_callbacks()) {
            peer_timer_.reset();
            peer_reconcile_deadline_ = std::chrono::steady_clock::time_point{};
            return;
        }
        auto timer = peer_timer_;
        peer_timer_.reset();
        peer_reconcile_deadline_ = std::chrono::steady_clock::time_point{};
        if (timer) {
            timer->cancel();
        }
        reconcile_peers();
    }, peer_callback_group_);
}

bool ServiceNode::run_peer_task_once(const std::string& mac,
                                     const std::string& label,
                                     std::function<void()> task) {
    if (shutting_down_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(peer_task_mutex_);
    if (peer_task_.valid()) {
        if (peer_task_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            return false;
        }
        peer_task_ = std::shared_future<void>{};
        peer_task_mac_.clear();
        peer_task_label_.clear();
    }

    auto future = std::async(std::launch::async, [this, mac, label, task = std::move(task)]() mutable {
        try {
            task();
        } catch (const std::exception& exception) {
            if (!shutting_down_.load()) {
                RCLCPP_WARN(get_logger(), "Peer task %s for %s failed: %s",
                            label.c_str(), mac.c_str(), exception.what());
            }
        } catch (...) {
            if (!shutting_down_.load()) {
                RCLCPP_WARN(get_logger(), "Peer task %s for %s failed with unknown error",
                            label.c_str(), mac.c_str());
            }
        }

        if (!shutting_down_.load()) {
            schedule_peer_reconcile(std::chrono::milliseconds(1));
        }
    }).share();
    peer_task_ = std::move(future);
    peer_task_mac_ = mac;
    peer_task_label_ = label;
    return true;
}

void ServiceNode::wait_for_peer_tasks() {
    std::shared_future<void> task;
    {
        std::lock_guard<std::mutex> lock(peer_task_mutex_);
        if (peer_task_.valid()) {
            task = peer_task_;
        }
        peer_task_ = std::shared_future<void>{};
        peer_task_mac_.clear();
        peer_task_label_.clear();
    }

    if (task.valid()) {
        task.wait();
    }
}

bool ServiceNode::update_peer_time_bridge(const std::string& mac,
                                          const bluez::DeviceInfo& device,
                                          peer::PeerConnectionSession& session) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    const auto time_characteristic_uuid = util::named_characteristic_uuid("time/ns");
    const auto writeback_descriptor_uuid = util::named_descriptor_uuid("time/ns/writeback");
    const auto peer_name = session.peer_name.empty() ? device_hostname_guess(device) : session.peer_name;
    const auto characteristic_path = client_->find_characteristic(mac, time_characteristic_uuid);
    const auto cached_characteristic = (!characteristic_path.empty() && cache_)
        ? cache_->characteristic(characteristic_path)
        : std::optional<bluez::GattCharacteristicInfo>{};
    auto bridge_it = peers_->time_bridges().find(mac);
    std::string stop_notify_path;

    const auto clear_time_bridge = [this, &mac, &bridge_it, &stop_notify_path]() {
        if (bridge_it == peers_->time_bridges().end()) {
            return;
        }
        const auto old_characteristic_path = bridge_it->second.characteristic_path;
        peers_->remove_time_bridge(mac);
        bridge_it = peers_->time_bridges().end();
        if (!old_characteristic_path.empty()) {
            stop_notify_path = old_characteristic_path;
        }
    };

    if (!device_is_secure_peer(device)) {
        clear_time_bridge();
        state_lock.unlock();
        if (!stop_notify_path.empty()) {
            client_->stop_notify(stop_notify_path);
        }
        state_lock.lock();
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.phase = "securing";
        session.detail = device_has_local_security(device)
            ? "connected, repairing trust"
            : "connected, waiting for pairing";
        return false;
    }

    if (!characteristic_path.empty() &&
        bridge_it != peers_->time_bridges().end() &&
        bridge_it->second.characteristic_path == characteristic_path &&
        cached_characteristic && cached_characteristic->notifying) {
        auto& bridge = bridge_it->second;
        bridge.mac = mac;
        bridge.peer_name = peer_name;
        bridge.status = "ready";
        bridge.detail = "peer time bridge active";
        peers_->clear_pairing_repair(session);
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        session.phase = "ready";
        session.detail = "peer time bridge active";
        return true;
    }

    const auto services = client_->list_services(mac);
    const auto characteristics = client_->list_characteristics(mac);
    const auto descriptors = client_->list_descriptors(mac);
    RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): %zu services, %zu characteristics, %zu descriptors resolved",
                 mac.c_str(), services.size(), characteristics.size(), descriptors.size());

    if (characteristic_path.empty()) {
        clear_time_bridge();
        state_lock.unlock();
        if (!stop_notify_path.empty()) {
            client_->stop_notify(stop_notify_path);
        }
        log_info_coalesced("peer-time-bridge-missing:" + mac + ":" + std::to_string(characteristics.size()),
                           "[node] update_peer_time_bridge(" + mac + "): time characteristic uuid=" +
                               time_characteristic_uuid + " not found among " +
                               std::to_string(characteristics.size()) + " characteristics");
        state_lock.lock();

        const auto now_mono = peers_->now_monotonic();
        if (characteristics.empty()) {
            if (session.remote_gatt_missing_since_monotonic <= 0.0) {
                session.remote_gatt_missing_since_monotonic = now_mono;
            }
            session.remote_gatt_missing_checks += 1;
            if (session.bridge_wait_started_monotonic <= 0.0) {
                session.bridge_wait_started_monotonic = now_mono;
            }
            session.bridge_wait_reason = "gatt-cache";
            session.phase = "connected_unready";
            session.detail = "waiting for remote GATT cache";

            const auto waited_s = now_mono - session.remote_gatt_missing_since_monotonic;
            if (session.remote_gatt_missing_checks >= kRemoteGattRepairMinChecks &&
                waited_s >= remote_gatt_repair_grace_s(session) &&
                (session.last_repair_monotonic <= 0.0 ||
                 now_mono - session.last_repair_monotonic >= remote_gatt_repair_grace_s(session)) &&
                !session.stale_pairing_detected &&
                !session.repair_in_progress) {
                peers_->request_pairing_repair(session, "services resolved without remote GATT after grace");
                session.last_repair_monotonic = 0.0;
            }
            return false;
        }

        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = now_mono;
        }
        session.bridge_wait_reason = "gatt-layout";
        session.phase = "connected_unready";
        session.detail = "peer time characteristic not available";
        return false;
    }

    session.remote_gatt_missing_since_monotonic = 0.0;
    session.remote_gatt_missing_checks = 0;

    if (bridge_it != peers_->time_bridges().end() &&
        !bridge_it->second.characteristic_path.empty() &&
        bridge_it->second.characteristic_path != characteristic_path) {
        clear_time_bridge();
    }

    auto [created_bridge_it, inserted_bridge] = peers_->time_bridges().try_emplace(mac);
    (void)inserted_bridge;
    bridge_it = created_bridge_it;
    auto& bridge = bridge_it->second;

    const auto topic_name = peer_status_topic(mac, peer_name);
    if (!bridge.publisher || bridge.status_topic_name != topic_name) {
        if (bridge.publisher) {
            bridge.publisher.reset();
        }
        bridge.publisher = create_publisher<mrs_uav_bluetooth::msg::BlePeerTimeStatus>(topic_name, 10);
        bridge.status_topic_name = topic_name;
    }

    bridge.mac = mac;
    bridge.peer_name = peer_name;
    bridge.characteristic_path = characteristic_path;

    if (bridge.status == "subscribing") {
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "notify";
        session.phase = "connected_unready";
        session.detail = "awaiting peer time notifications";
        return false;
    }

    if (bridge.status == "notify_failed") {
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "notify";
        session.phase = "connected_unready";
        session.detail = "failed to enable peer time notifications";
        return false;
    }

    const auto wb_path = client_->find_descriptor(mac, writeback_descriptor_uuid, characteristic_path);
    if (wb_path.empty() && !descriptors.empty()) {
        RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): writeback descriptor uuid=%s not found (descriptors resolved=%zu)",
                     mac.c_str(), writeback_descriptor_uuid.c_str(), descriptors.size());
    } else if (wb_path.empty()) {
        RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): no descriptors resolved for this device, writeback disabled",
                     mac.c_str());
    }
    bridge.writeback_descriptor_path = wb_path;
    bridge.status = "subscribing";
    bridge.detail = "enabling peer time notifications";
    bridge.services_wait_grace_s = session.services_wait_grace_s;
    bridge.pairing_failures = session.pairing_failures;

    const auto notify_start_log = "[node] update_peer_time_bridge(" + mac + "): notifications requested on " +
        characteristic_path + " (writeback=" +
        (wb_path.empty() ? std::string{"none"} : wb_path) + ")";

    state_lock.unlock();
    if (!stop_notify_path.empty()) {
        client_->stop_notify(stop_notify_path);
    }

    if (client_->start_notify(characteristic_path)) {
        log_info_coalesced("peer-time-bridge-notify-start:" + mac + ":" + characteristic_path,
                           notify_start_log);
        state_lock.lock();
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = peers_->now_monotonic();
        }
        session.bridge_wait_reason = "notify";
        session.connect_repair_count = 0;
        session.phase = "connected_unready";
        session.detail = "awaiting peer time notifications";
        return false;
    }

    state_lock.lock();
    RCLCPP_WARN(get_logger(), "[node] update_peer_time_bridge(%s): failed to enable notifications on %s",
                mac.c_str(), characteristic_path.c_str());
    bridge.status = "notify_failed";
    bridge.detail = "failed to enable peer time notifications";
    if (session.bridge_wait_started_monotonic <= 0.0) {
        session.bridge_wait_started_monotonic = peers_->now_monotonic();
    }
    session.bridge_wait_reason = "notify";
    session.phase = "connected_unready";
    session.detail = "failed to enable peer time notifications";
    return false;
}

void ServiceNode::reconcile_peers() {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !client_) {
        return;
    }

    const auto now = peers_->now_monotonic();
    const auto retry_period_s = std::max(0.5, active_config_.auto_connect_period);
    const bool whitelist_enabled = !active_config_.auto_connect_whitelist.empty();
    const auto adapter = cache_ ? cache_->adapter(adapter_path_) : std::optional<bluez::AdapterInfo>{};
    const std::string local_adapter_mac = adapter ? adapter->address : std::string{};
    std::set<std::string> current_macs;
    for (const auto& device : client_->get_devices()) {
        current_macs.insert(device.mac);
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000,
                          "[reconcile] sessions=%zu devices=%zu auto_connect=%s whitelist=%zu",
                          peers_->sessions().size(), current_macs.size(),
                          active_config_.auto_connect_enable ? "on" : "off",
                          active_config_.auto_connect_whitelist.size());

    peers_->prune_sessions(current_macs, now, std::max(5.0, active_config_.peer_connection_timeout));

    for (auto& [mac, session] : peers_->sessions()) {
        const auto device = client_->get_device(mac);
        const auto device_label = mac + " (" + session.peer_name + ")";
        const bool is_connected = device && device->connected;
        const bool secure_bridge_peer = device && device_can_host_peer_bridge(*device);

        if (!session.desired || !secure_bridge_peer) {
            state_lock.unlock();
            clear_peer_runtime(mac);
            state_lock.lock();
        }

        if (secure_bridge_peer) {
            const auto device_copy = *device;
            state_lock.unlock();
            refresh_import_bridges_for_device(device_copy);
            state_lock.lock();
        }

        if (!session.desired) {
            const bool policy_violation = session.peer_candidate && device && device_needs_forget(*device);
            if (!policy_violation) {
                session.forget_pending = false;
                continue;
            }
            if (!session.forget_pending &&
                run_peer_task_once(mac, "policy cleanup", [this, mac, retry_period_s, connected = device->connected,
                                                            trusted = device->trusted]() {
                    if (connected) {
                        (void)client_->disconnect(mac, retry_period_s);
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                    if (trusted) {
                        (void)client_->untrust(mac);
                    }
                    (void)client_->remove(mac);
                })) {
                RCLCPP_INFO(get_logger(),
                            "[reconcile] %s: forgetting peer blocked by current config",
                            device_label.c_str());
                session.forget_pending = true;
                session.phase = "policy_blocked";
                session.detail = whitelist_enabled ? "peer not present in whitelist"
                                                  : "peer not allowed by current config";
                session.last_policy_action_monotonic = now;
            }
            continue;
        }

        session.forget_pending = false;

        const bool should_initiate = device ? local_is_preferred_initiator(hostname_,
                                                                           local_adapter_mac,
                                                                           session,
                                                                           *device)
                                            : true;

        if (device && device->blocked) {
            if (run_peer_task_once(mac, "unblock", [this, mac]() {
                    (void)client_->unblock(mac);
                })) {
                session.phase = "discovered";
                session.detail = "unblocking desired peer";
            }
            continue;
        }

        if (session.stale_pairing_detected) {
            if (session.repair_awaiting_cache_removal || session.repair_in_progress) {
                continue;
            }

            if (!session.repair_remove_issued &&
                !run_peer_task_once(mac, "reset stale pairing", [this, mac, retry_period_s, connected = is_connected]() {
                    if (connected) {
                        (void)client_->disconnect(mac, retry_period_s);
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                    (void)client_->remove(mac);
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    (void)client_->unblock(mac);
                })) {
                continue;
            }

            if (!session.repair_remove_issued) {
                RCLCPP_WARN(get_logger(), "[reconcile] %s: resetting stale pairing state (%s)",
                            device_label.c_str(),
                            session.repair_reason.empty() ? "stale security" : session.repair_reason.c_str());
                session.repair_remove_issued = true;
                session.repair_awaiting_cache_removal = true;
                session.repair_in_progress = true;
                state_lock.unlock();
                clear_peer_runtime(mac);
                state_lock.lock();
                session.phase = "recovering";
                session.detail = session.repair_reason.empty() ? "resetting stale security" : session.repair_reason;
                session.last_repair_monotonic = now;
                session.last_connect_attempt_monotonic = 0.0;
                session.connect_started_monotonic = 0.0;
                session.connected_since_monotonic = 0.0;
                session.last_security_attempt_monotonic = 0.0;
                session.services_wait_started_monotonic = 0.0;
                session.bridge_wait_started_monotonic = 0.0;
                session.bridge_wait_reason.clear();
                session.remote_gatt_missing_since_monotonic = 0.0;
                session.remote_gatt_missing_checks = 0;
                continue;
            }

            if (session.phase == "recovering") {
                session.phase = device ? "discovered" : "stale";
                session.detail = device ? "awaiting fresh connection after security reset"
                                        : "awaiting fresh discovery after security reset";
            }
        }

        if (session.repair_in_progress) {
            continue;
        }

        if (!is_connected) {
            if (!should_initiate) {
                session.phase = session.last_connect_attempt_monotonic > 0.0 ? "disconnected" : "discovered";
                session.detail = "awaiting connection from preferred initiator";
                continue;
            }
            if (peers_->should_attempt_connect(session, now, retry_period_s)) {
                if (run_peer_task_once(mac, "connect", [this, mac, retry_period_s]() {
                        (void)client_->connect(mac, retry_period_s);
                    })) {
                    RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting connect (phase=%s)",
                                device_label.c_str(), session.phase.c_str());
                    session.phase = "connecting";
                    session.detail = "auto-connect requested";
                    session.last_connect_attempt_monotonic = now;
                    session.connect_started_monotonic = now;
                }
            }
            continue;
        }

        if (device && !(device->paired || device->bonded)) {
            if (!should_initiate) {
                session.phase = "securing";
                session.detail = passive_peer_wait_detail(*device);
                continue;
            }
            if (peers_->should_attempt_pair(session, *device, now, retry_period_s)) {
                if (run_peer_task_once(mac, "pair", [this, mac, retry_period_s]() {
                        std::string pair_error;
                        const bool pair_ok = client_->pair(mac, retry_period_s, &pair_error);
                        note_pair_attempt_result(mac, pair_ok, pair_error);
                    })) {
                    RCLCPP_INFO(get_logger(),
                                "[reconcile] %s: attempting pair (phase=%s)",
                                device_label.c_str(),
                                session.phase.c_str());
                    session.phase = "securing";
                    session.detail = session.pairing_reset_pending
                        ? "re-pair requested"
                        : "auto-pair requested";
                    session.last_security_attempt_monotonic = now;
                }
            }
            continue;
        }

        if (device && peers_->should_attempt_trust(session, *device, now, retry_period_s)) {
            if (run_peer_task_once(mac, "trust", [this, mac]() {
                    (void)client_->trust(mac);
                })) {
                RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting trust (paired=%s bonded=%s trusted=%s)",
                            device_label.c_str(),
                            device->paired ? "Y" : "N",
                            device->bonded ? "Y" : "N",
                            device->trusted ? "Y" : "N");
                session.phase = "securing";
                session.detail = "trust requested";
                session.last_security_attempt_monotonic = now;
            }
            continue;
        }

        if (is_connected && device->trusted && (device->paired || device->bonded) && !device->services_resolved) {
            if ((session.last_service_retry_monotonic <= 0.0 ||
                 now - session.last_service_retry_monotonic >= retry_period_s) &&
                run_peer_task_once(mac, "wait services", [this, mac, retry_period_s]() {
                    (void)client_->wait_services_resolved(mac, std::max(8.0, retry_period_s * 4.0));
                })) {
                session.last_service_retry_monotonic = now;
            }
            session.phase = "connected_unready";
            session.detail = "connected, waiting for services";
            continue;
        }

        if (is_connected && device->trusted && (device->paired || device->bonded) && device->services_resolved) {
            const auto device_copy = *device;
            state_lock.unlock();
            (void)update_peer_time_bridge(mac, device_copy, session);
            state_lock.lock();
            continue;
        }

        RCLCPP_DEBUG(get_logger(), "[reconcile] %s: desired but pending (phase=%s conn=%s paired=%s trusted=%s svc_resolved=%s)",
                     device_label.c_str(), session.phase.c_str(),
                     is_connected ? "Y" : "N",
                     device ? (device->paired ? "Y" : "N") : "?",
                     device ? (device->trusted ? "Y" : "N") : "?",
                     device ? (device->services_resolved ? "Y" : "N") : "?");
    }
}

}  // namespace mrs_uav_bluetooth::app