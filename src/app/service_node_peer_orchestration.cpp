// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <cctype>
#include <future>
#include <optional>
#include <rclcpp/create_timer.hpp>

namespace {

constexpr double kLocalReconfigureGraceMin = 5.0;
constexpr double kRemoteGattSnapshotFallbackDelay = 2.0;
constexpr double kRemoteGattSnapshotRetryInterval = 2.0;
constexpr double kPeerPairTimeout = 20.0;
constexpr auto kPeerWaitReconcileDelay = std::chrono::milliseconds(1000);

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
        peers_->clear_device_reset(session);
        return;
    }

    session.pairing_failures += 1;
    const auto normalized_error = lower_trim(error_detail);
    const bool authentication_failed = normalized_error.find("authentication failed") != std::string::npos ||
        normalized_error.find("authentication rejected") != std::string::npos ||
        normalized_error.find("authentication canceled") != std::string::npos;

    const bool bridge_ready = [&]() {
        const auto bridge_it = peers_->time_bridges().find(mac);
        return bridge_it != peers_->time_bridges().end() && bridge_it->second.status == "ready";
    }();

    session.repair_in_progress = false;
    if (bridge_ready || session.phase == "ready") {
        session.detail = error_detail.empty() ? "pair failed" : "pair failed: " + error_detail;
        return;
    }
    session.phase = "connected_unready";
    session.detail = error_detail.empty()
        ? (authentication_failed ? "pair authentication failed" : "pair failed")
        : "pair failed: " + error_detail;
}

bool ServiceNode::should_allow_pairing_request(const std::string& event_type,
                                               const std::string& device_path) {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !cache_) {
        return false;
    }

    if (!active_config_.auto_pair) {
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

    if (session.repair_requested || session.repair_in_progress) {
        log_warn_coalesced("pairing-rejected-stale:" + device->mac + ":" + event_type,
                           "[node] rejecting pairing request for " + device->mac +
                               " because peer state reset is already in progress");
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
    const auto now_mono = peers_->now_monotonic();
    const auto time_characteristic_uuid = util::named_characteristic_uuid("time/ns");
    const auto writeback_descriptor_uuid = util::named_descriptor_uuid("time/ns/writeback");
    const auto peer_name = session.peer_name.empty() ? device_hostname_guess(device) : session.peer_name;
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

    if (!device.connected || !device.services_resolved) {
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
        session.last_service_retry_monotonic = 0.0;
        session.phase = "connected_unready";
        session.detail = device.connected ? "connected, waiting for services" : "awaiting connection";
        return false;
    }

    if (session.services_resolved_since_monotonic > 0.0 &&
        (now_mono - session.services_resolved_since_monotonic) < 1.5) {
        if (session.bridge_wait_started_monotonic <= 0.0) {
            session.bridge_wait_started_monotonic = now_mono;
        }
        session.bridge_wait_reason = "gatt-cache";
        session.phase = "connected_unready";
        session.detail = "waiting for remote GATT cache";
        schedule_peer_reconcile(kPeerWaitReconcileDelay);
        return false;
    }

    auto services = client_->list_services(mac);
    auto characteristics = client_->list_characteristics(mac);
    auto characteristic_path = client_->find_characteristic(mac, time_characteristic_uuid);
    auto cached_characteristic = (!characteristic_path.empty() && cache_)
        ? cache_->characteristic(characteristic_path)
        : std::optional<bluez::GattCharacteristicInfo>{};

    const bool should_attempt_snapshot_refresh =
        characteristic_path.empty() && characteristics.empty() &&
        session.services_resolved_since_monotonic > 0.0 &&
        (now_mono - session.services_resolved_since_monotonic) >= kRemoteGattSnapshotFallbackDelay &&
        (session.last_service_retry_monotonic <= 0.0 ||
         (now_mono - session.last_service_retry_monotonic) >= kRemoteGattSnapshotRetryInterval);

    if (should_attempt_snapshot_refresh) {
        session.last_service_retry_monotonic = now_mono;
        state_lock.unlock();
        const bool refreshed = client_->refresh_gatt_snapshot(mac);
        state_lock.lock();
        if (refreshed) {
            services = client_->list_services(mac);
            characteristics = client_->list_characteristics(mac);
            characteristic_path = client_->find_characteristic(mac, time_characteristic_uuid);
            cached_characteristic = (!characteristic_path.empty() && cache_)
                ? cache_->characteristic(characteristic_path)
                : std::optional<bluez::GattCharacteristicInfo>{};
        }
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
        peers_->clear_device_reset(session);
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        session.last_service_retry_monotonic = 0.0;
        session.phase = "ready";
        session.detail = "peer time bridge active";
        return true;
    }

    RCLCPP_DEBUG(get_logger(), "[node] update_peer_time_bridge(%s): %zu services, %zu characteristics, %zu descriptors resolved",
                 mac.c_str(), services.size(), characteristics.size(),
                 characteristic_path.empty() ? size_t{0} : client_->list_descriptors(mac, characteristic_path).size());

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
            session.detail = should_attempt_snapshot_refresh
                ? "waiting for remote GATT objects after fallback snapshot"
                : "waiting for remote GATT objects";
            schedule_peer_reconcile(kPeerWaitReconcileDelay);
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
        schedule_peer_reconcile(kPeerWaitReconcileDelay);
        return false;
    }

    session.remote_gatt_missing_since_monotonic = 0.0;
    session.remote_gatt_missing_checks = 0;
    session.last_service_retry_monotonic = 0.0;

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

    const auto descriptors = client_->list_descriptors(mac, characteristic_path);
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
    bool should_suspend_scan = false;
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
        const bool functional_bridge_peer = device && device_can_host_peer_bridge(*device, active_config_);

        if (!session.desired || !functional_bridge_peer) {
            state_lock.unlock();
            clear_peer_runtime(mac);
            state_lock.lock();
        }

        const bool recent_connect_flow = session.last_connect_attempt_monotonic > 0.0 &&
            (now - session.last_connect_attempt_monotonic) < std::max(4.0, retry_period_s * 2.0);
        const bool remote_flow_active = session.remote_initiated_until_monotonic > 0.0 &&
            now < session.remote_initiated_until_monotonic;
        const bool waiting_for_bridge = session.services_wait_started_monotonic > 0.0 ||
            session.bridge_wait_started_monotonic > 0.0;

        if (session.desired &&
            (is_connected || session.phase == "connecting" || session.phase == "connect_pending" ||
             session.phase == "connected_unready" || session.phase == "securing" || session.phase == "ready" ||
             recent_connect_flow || remote_flow_active || waiting_for_bridge)) {
            should_suspend_scan = true;
        }

        if (functional_bridge_peer) {
            const auto device_copy = *device;
            state_lock.unlock();
            refresh_import_bridges_for_device(device_copy);
            state_lock.lock();
        }

        if (!session.desired) {
            session.forget_pending = false;
            if (device && device->connected) {
                if (run_peer_task_once(mac, "disconnect undesired peer", [this, mac, retry_period_s]() {
                        (void)client_->disconnect(mac, retry_period_s);
                    })) {
                    session.phase = "policy_blocked";
                    session.detail = whitelist_enabled ? "disconnecting peer not present in whitelist"
                                                      : "disconnecting peer not allowed by current config";
                }
                continue;
            }
            session.phase = "policy_blocked";
            session.detail = whitelist_enabled ? "peer not present in whitelist"
                                              : "peer not allowed by current config";
            continue;
        }

        session.forget_pending = false;

        if (device && device->blocked) {
            if (run_peer_task_once(mac, "unblock", [this, mac]() {
                    (void)client_->unblock(mac);
                })) {
                session.phase = "discovered";
                session.detail = "unblocking desired peer";
            }
            continue;
        }

        if (session.repair_requested) {
            if (session.repair_awaiting_cache_removal || session.repair_in_progress) {
                continue;
            }

            if (!session.repair_remove_issued &&
                !run_peer_task_once(mac, "reset peer device", [this, mac, retry_period_s, connected = is_connected]() {
                    const auto should_continue_repair = [this, &mac]() {
                        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
                        if (shutting_down_.load() || !peers_) {
                            return false;
                        }
                        const auto session_it = peers_->sessions().find(mac);
                        return session_it != peers_->sessions().end() &&
                               session_it->second.repair_requested;
                    };

                    if (!should_continue_repair()) {
                        return;
                    }
                    if (connected) {
                        (void)client_->disconnect(mac, retry_period_s);
                    }
                    if (!should_continue_repair()) {
                        return;
                    }
                    (void)client_->remove(mac);
                    if (!should_continue_repair()) {
                        return;
                    }
                    (void)client_->unblock(mac);
                })) {
                continue;
            }

            if (!session.repair_remove_issued) {
                RCLCPP_WARN(get_logger(), "[reconcile] %s: resetting BlueZ device state (%s)",
                            device_label.c_str(),
                            session.repair_reason.empty() ? "device reset" : session.repair_reason.c_str());
                session.repair_remove_issued = true;
                session.repair_awaiting_cache_removal = true;
                session.repair_in_progress = true;
                state_lock.unlock();
                clear_peer_runtime(mac);
                state_lock.lock();
                session.phase = "recovering";
                session.detail = session.repair_reason.empty() ? "resetting peer device state" : session.repair_reason;
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
            const double local_reconfigure_grace_s = std::max(kLocalReconfigureGraceMin, retry_period_s * 2.0);
            if (local_server_rebuild_in_progress_.load() ||
                (local_server_rebuild_monotonic_ > 0.0 &&
                 (now - local_server_rebuild_monotonic_) < local_reconfigure_grace_s)) {
                session.phase = session.last_connect_attempt_monotonic > 0.0 ? "disconnected" : "discovered";
                session.detail = "waiting for local GATT rebuild";
                continue;
            }
            if (peers_->should_defer_connect_due_to_remote_activity(session, now)) {
                session.phase = session.last_connect_attempt_monotonic > 0.0 ? "disconnected" : "discovered";
                session.detail = "remote-initiated flow in progress";
                continue;
            }
            if (peers_->should_attempt_connect(session, now, retry_period_s)) {
                if (run_peer_task_once(mac, "connect", [this, mac, retry_period_s]() {
                        if (local_server_rebuild_in_progress_.load()) {
                            return;
                        }
                        (void)client_->connect(mac, retry_period_s, true);
                    })) {
                    RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting connect (phase=%s)",
                                device_label.c_str(), session.phase.c_str());
                    peers_->note_local_connect_attempt(session, now);
                    session.phase = "connecting";
                    session.detail = "auto-connect requested (explicit LE when available)";
                }
            }
            continue;
        }

        if (is_connected && device &&
            peers_->should_attempt_pair(session, *device, active_config_, now, retry_period_s)) {
            if (run_peer_task_once(mac, "pair", [this, mac]() {
                    std::string error_detail;
                    const bool success = client_->pair(mac, kPeerPairTimeout, &error_detail);
                    note_pair_attempt_result(mac, success, error_detail);
                })) {
                RCLCPP_INFO(get_logger(), "[reconcile] %s: attempting pair after connection", device_label.c_str());
                session.phase = "securing";
                session.detail = "pair requested";
                session.last_security_attempt_monotonic = now;
                continue;
            }
        }

        if (is_connected && device && !device->services_resolved) {
            schedule_peer_reconcile(kPeerWaitReconcileDelay);
            session.phase = "connected_unready";
            session.detail = "connected, waiting for services";
            continue;
        }

        if (is_connected && device && device->services_resolved) {
            const auto device_copy = *device;
            state_lock.unlock();
            (void)update_peer_time_bridge(mac, device_copy, session);
            state_lock.lock();

            if (session.phase == "ready" && device) {
                if (peers_->should_attempt_trust(session, *device, active_config_, now, retry_period_s)) {
                    if (run_peer_task_once(mac, "trust", [this, mac]() {
                            (void)client_->trust(mac);
                        })) {
                        RCLCPP_INFO(get_logger(),
                                    "[reconcile] %s: attempting late trust (paired=%s bonded=%s trusted=%s)",
                                    device_label.c_str(),
                                    device->paired ? "Y" : "N",
                                    device->bonded ? "Y" : "N",
                                    device->trusted ? "Y" : "N");
                        session.detail = "peer time bridge active, trust requested";
                        session.last_security_attempt_monotonic = now;
                        continue;
                    }
                }
            }
            continue;
        }

        RCLCPP_DEBUG(get_logger(), "[reconcile] %s: desired but pending (phase=%s conn=%s paired=%s trusted=%s svc_resolved=%s)",
                     device_label.c_str(), session.phase.c_str(),
                     is_connected ? "Y" : "N",
                     device ? (device->paired ? "Y" : "N") : "?",
                     device ? (device->trusted ? "Y" : "N") : "?",
                     device ? (device->services_resolved ? "Y" : "N") : "?");
    }

    const bool should_scan = active_config_.enable_scan && !should_suspend_scan;
    state_lock.unlock();
    if (should_scan) {
        if (!client_->is_scanning()) {
            client_->start_scan(active_config_.scan_mode, active_config_.enable_server);
        }
    } else {
        if (client_->is_scanning()) {
            client_->stop_scan();
        }
    }
}

}  // namespace mrs_uav_bluetooth::app