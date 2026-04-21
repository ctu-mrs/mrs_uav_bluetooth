// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/device_utils.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace {

constexpr double kPairingCancelAssociationTimeout = 20.0;
constexpr double kPeerInitNotifyFailureResetWindow = 45.0;

bool device_has_recorded_bond(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    return device.paired || device.bonded;
}

bool is_read_write_gatt_event(const std::string& event_type) {
    return event_type == "client_read" ||
           event_type == "client_write" ||
           event_type == "client_descriptor_read" ||
           event_type == "client_descriptor_write";
}

bool is_failed_gatt_event(const std::string& event_type) {
    constexpr auto suffix = "_failed";
    return event_type.size() >= std::char_traits<char>::length(suffix) &&
           event_type.compare(event_type.size() - std::char_traits<char>::length(suffix),
                              std::char_traits<char>::length(suffix),
                              suffix) == 0;
}

std::optional<std::string> device_path_for_cache_event(
    const mrs_uav_bluetooth::bluez::ObjectManagerCache& cache,
    mrs_uav_bluetooth::bluez::CacheEvent event,
    const std::string& object_path) {
    using mrs_uav_bluetooth::bluez::CacheEvent;

    if (event == CacheEvent::DeviceAdded || event == CacheEvent::DevicePropertyChanged) {
        return object_path;
    }
    if (event == CacheEvent::GattServiceAdded || event == CacheEvent::GattServiceRemoved) {
        if (const auto service = cache.service(object_path); service && !service->device_path.empty()) {
            return service->device_path;
        }
    }
    if (event == CacheEvent::GattCharacteristicAdded ||
        event == CacheEvent::GattCharacteristicChanged ||
        event == CacheEvent::GattCharacteristicValueChanged ||
        event == CacheEvent::GattCharacteristicRemoved) {
        if (const auto characteristic = cache.characteristic(object_path)) {
            if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
                return service->device_path;
            }
        }
    }
    if (event == CacheEvent::GattDescriptorAdded ||
        event == CacheEvent::GattDescriptorChanged ||
        event == CacheEvent::GattDescriptorValueChanged ||
        event == CacheEvent::GattDescriptorRemoved) {
        if (const auto descriptor = cache.descriptor(object_path)) {
            if (const auto characteristic = cache.characteristic(descriptor->characteristic_path)) {
                if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
                    return service->device_path;
                }
            }
        }
    }
    const auto service_pos = object_path.find("/service");
    if (service_pos != std::string::npos) {
        return object_path.substr(0, service_pos);
    }
    return std::nullopt;
}

std::optional<std::string> device_path_for_gatt_object(
    const mrs_uav_bluetooth::bluez::ObjectManagerCache& cache,
    const std::string& object_path) {
    if (const auto device = cache.device(object_path)) {
        return device->object_path;
    }
    if (const auto service = cache.service(object_path)) {
        return service->device_path.empty() ? std::nullopt : std::optional<std::string>(service->device_path);
    }
    if (const auto characteristic = cache.characteristic(object_path)) {
        if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
            return service->device_path;
        }
    }
    if (const auto descriptor = cache.descriptor(object_path)) {
        if (const auto characteristic = cache.characteristic(descriptor->characteristic_path)) {
            if (const auto service = cache.service(characteristic->service_path); service && !service->device_path.empty()) {
                return service->device_path;
            }
        }
    }
    const auto service_pos = object_path.find("/service");
    if (service_pos != std::string::npos) {
        return object_path.substr(0, service_pos);
    }
    return std::nullopt;
}

}  // namespace

namespace mrs_uav_bluetooth::app {

void ServiceNode::on_cache_event(bluez::CacheEvent event, const std::string& object_path) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !cache_) {
        return;
    }

    std::optional<bluez::DeviceInfo> refresh_device;
    std::optional<std::string> clear_runtime_mac;
    bool emit_disconnect_log = false;
    bool disconnect_expected = false;
    std::string disconnect_log_key;
    std::string disconnect_log_message;

    if (event == bluez::CacheEvent::GattCharacteristicValueChanged ||
        event == bluez::CacheEvent::GattDescriptorValueChanged) {
        return;
    }

    const char* event_name = [](bluez::CacheEvent e) -> const char* {
        switch (e) {
            case bluez::CacheEvent::DeviceAdded:                    return "DeviceAdded";
            case bluez::CacheEvent::DeviceRemoved:                  return "DeviceRemoved";
            case bluez::CacheEvent::DevicePropertyChanged:          return "DevicePropertyChanged";
            case bluez::CacheEvent::GattServiceAdded:               return "GattServiceAdded";
            case bluez::CacheEvent::GattServiceRemoved:             return "GattServiceRemoved";
            case bluez::CacheEvent::GattCharacteristicAdded:        return "GattCharacteristicAdded";
            case bluez::CacheEvent::GattCharacteristicChanged:      return "GattCharacteristicChanged";
            case bluez::CacheEvent::GattCharacteristicValueChanged: return "GattCharacteristicValueChanged";
            case bluez::CacheEvent::GattCharacteristicRemoved:      return "GattCharacteristicRemoved";
            case bluez::CacheEvent::GattDescriptorAdded:            return "GattDescriptorAdded";
            case bluez::CacheEvent::GattDescriptorChanged:          return "GattDescriptorChanged";
            case bluez::CacheEvent::GattDescriptorValueChanged:     return "GattDescriptorValueChanged";
            case bluez::CacheEvent::GattDescriptorRemoved:          return "GattDescriptorRemoved";
            case bluez::CacheEvent::AdapterChanged:                 return "AdapterChanged";
            default:                                                return "Unknown";
        }
    }(event);

    const auto maybe_capture_disconnect_transition = [&](const bluez::DeviceInfo& device) {
        auto session_it = peers_->sessions().find(device.mac);
        if (device.connected) {
            expected_disconnect_reasons_.erase(device.mac);
            return;
        }
        if (session_it == peers_->sessions().end() || session_it->second.connected_since_monotonic <= 0.0) {
            return;
        }

        const auto& session = session_it->second;
        const auto bridge_it = peers_->time_bridges().find(device.mac);
        const std::string bridge_status = bridge_it != peers_->time_bridges().end()
            ? bridge_it->second.status
            : "none";
        const double now_mono = peers_->now_monotonic();
        const double connected_for_s = session.connected_since_monotonic > 0.0
            ? std::max(0.0, now_mono - session.connected_since_monotonic)
            : 0.0;
        const double bridge_inactivity_s = bridge_it != peers_->time_bridges().end() &&
                bridge_it->second.last_activity_monotonic > 0.0
            ? std::max(0.0, now_mono - bridge_it->second.last_activity_monotonic)
            : -1.0;
        const std::string label = !session.peer_name.empty()
            ? session.peer_name
            : (!device.name.empty() ? device.name : device.mac);

        auto expected_it = expected_disconnect_reasons_.find(device.mac);
        disconnect_expected = expected_it != expected_disconnect_reasons_.end();
        const std::string expected_reason = disconnect_expected ? expected_it->second : std::string{};
        if (expected_it != expected_disconnect_reasons_.end()) {
            expected_disconnect_reasons_.erase(expected_it);
        }

        std::ostringstream stream;
        stream << "[disconnect] " << device.mac << " (" << label << "): "
               << (disconnect_expected ? "expected disconnect" : "unexpected disconnect")
               << " phase=" << session.phase
               << " bridge=" << bridge_status
               << " conn_for=" << std::fixed << std::setprecision(1) << connected_for_s << "s"
               << " local_time_notify=" << (session.local_time_notify_active_this_connection ? "Y" : "N")
               << " bridge_healthy=" << (session.time_bridge_healthy_this_connection ? "Y" : "N");
        if (bridge_inactivity_s >= 0.0) {
            stream << " bridge_idle=" << std::fixed << std::setprecision(1) << bridge_inactivity_s << "s";
        }
        if (!session.detail.empty()) {
            stream << " detail=" << session.detail;
        }
        if (disconnect_expected && !expected_reason.empty()) {
            stream << " reason=" << expected_reason;
        }

        disconnect_log_key = std::string{"disconnect:"} + (disconnect_expected ? "expected:" : "unexpected:") +
            device.mac + ":" + session.phase + ":" + bridge_status;
        disconnect_log_message = stream.str();
        emit_disconnect_log = true;
    };

    if (event == bluez::CacheEvent::DeviceAdded) {
        auto device = cache_->device(object_path);
        if (!device) {
            log_warn_coalesced("cache:DeviceAdded-missing:" + object_path,
                               "[node] on_cache_event: DeviceAdded path=" + object_path + " but device not in cache");
            return;
        }
        const auto peer_name = util::device_hostname_guess(*device);
        log_info_coalesced("cache:DeviceAdded:" + device->mac,
                           "[node] on_cache_event: DeviceAdded mac=" + device->mac +
                               " name='" + device->name + "' peer_name='" + peer_name +
                               " path=" + object_path);
        const bool preserve_ready_runtime =
            has_ready_peer_time_bridge(device->mac) ||
            should_preserve_ready_bridge_during_expected_services_rediscovery(*device);
        const bool preserve_active_bridge_runtime =
            should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(*device);
        maybe_capture_disconnect_transition(*device);
        peers_->sync_device(*device,
                            active_config_,
                            peer_name,
                            preserve_ready_runtime,
                            preserve_active_bridge_runtime);
        refresh_device = *device;
    } else if (event == bluez::CacheEvent::DeviceRemoved) {
        std::string removed_mac;
        auto session_it = std::find_if(peers_->sessions().begin(), peers_->sessions().end(),
            [&object_path, this](const auto& pair) {
                const auto& mac = pair.first;
                auto dev = cache_->device_by_mac(mac);
                return dev && dev->object_path == object_path;
            });
        if (session_it != peers_->sessions().end()) {
            removed_mac = session_it->first;
        } else {
            const auto dev_pos = object_path.rfind("/dev_");
            if (dev_pos != std::string::npos) {
                auto mac_part = object_path.substr(dev_pos + 5);
                std::replace(mac_part.begin(), mac_part.end(), '_', ':');
                removed_mac = mac_part;
            }
        }

        if (!removed_mac.empty()) {
            log_info_coalesced("cache:DeviceRemoved:" + removed_mac,
                               "[node] on_cache_event: DeviceRemoved mac=" + removed_mac +
                                   " path=" + object_path);
            if (peers_->sessions().count(removed_mac)) {
                peers_->note_missing_device(removed_mac, peers_->now_monotonic());
                clear_runtime_mac = removed_mac;
            }
        } else {
            RCLCPP_DEBUG(get_logger(), "[node] on_cache_event: DeviceRemoved path=%s (no matching session)",
                         object_path.c_str());
        }
    } else if (event == bluez::CacheEvent::GattServiceAdded || event == bluez::CacheEvent::GattServiceRemoved) {
        log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                           std::string{"[node] on_cache_event: "} + event_name + " path=" + object_path);
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                const bool preserve_ready_runtime =
                    has_ready_peer_time_bridge(device->mac) ||
                    should_preserve_ready_bridge_during_expected_services_rediscovery(*device);
                const bool preserve_active_bridge_runtime =
                    should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(*device);
                maybe_capture_disconnect_transition(*device);
                peers_->sync_device(*device,
                                    active_config_,
                                    util::device_hostname_guess(*device),
                                    preserve_ready_runtime,
                                    preserve_active_bridge_runtime);
                refresh_device = *device;
            }
        }
    } else if (event == bluez::CacheEvent::GattCharacteristicAdded || event == bluez::CacheEvent::GattCharacteristicRemoved) {
        if (const auto chrc = cache_->characteristic(object_path)) {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name +
                                   " uuid=" + chrc->uuid + " path=" + object_path);
        } else {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name + " path=" + object_path);
        }
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                const bool preserve_ready_runtime =
                    has_ready_peer_time_bridge(device->mac) ||
                    should_preserve_ready_bridge_during_expected_services_rediscovery(*device);
                const bool preserve_active_bridge_runtime =
                    should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(*device);
                maybe_capture_disconnect_transition(*device);
                peers_->sync_device(*device,
                                    active_config_,
                                    util::device_hostname_guess(*device),
                                    preserve_ready_runtime,
                                    preserve_active_bridge_runtime);
                refresh_device = *device;
            }
        }
    } else if (event == bluez::CacheEvent::GattDescriptorAdded || event == bluez::CacheEvent::GattDescriptorRemoved) {
        if (const auto desc = cache_->descriptor(object_path)) {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name +
                                   " uuid=" + desc->uuid + " chrc=" + desc->characteristic_path +
                                   " path=" + object_path);
        } else {
            log_info_coalesced(std::string{"cache:"} + event_name + ":" + object_path,
                               std::string{"[node] on_cache_event: "} + event_name + " path=" + object_path);
        }
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                const bool preserve_ready_runtime =
                    has_ready_peer_time_bridge(device->mac) ||
                    should_preserve_ready_bridge_during_expected_services_rediscovery(*device);
                const bool preserve_active_bridge_runtime =
                    should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(*device);
                maybe_capture_disconnect_transition(*device);
                peers_->sync_device(*device,
                                    active_config_,
                                    util::device_hostname_guess(*device),
                                    preserve_ready_runtime,
                                    preserve_active_bridge_runtime);
                refresh_device = *device;
            }
        }
    } else {
        RCLCPP_DEBUG(get_logger(), "[node] on_cache_event: %s path=%s", event_name, object_path.c_str());
        if (event == bluez::CacheEvent::AdapterChanged && object_path == adapter_path_) {
            if (const auto adapter = cache_->adapter(object_path)) {
                const bool should_be_discoverable = active_config_.enable_server;
                const bool should_be_pairable = active_config_.auto_pair;
                const bool drifted = !adapter->powered ||
                    !adapter->connectable ||
                    adapter->pairable != should_be_pairable ||
                    adapter->alias != hostname_ ||
                    adapter->discoverable != should_be_discoverable ||
                    (should_be_discoverable && adapter->discoverable_timeout != active_config_.discoverable_timeout);
                if (drifted) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "[node] adapter state drift detected, reapplying powered/pairable/discoverable/alias settings");
                    apply_adapter_state(active_config_);
                }
            }
        }
        if (const auto device_path = device_path_for_cache_event(*cache_, event, object_path)) {
            if (const auto device = cache_->device(*device_path)) {
                const bool preserve_ready_runtime =
                    has_ready_peer_time_bridge(device->mac) ||
                    should_preserve_ready_bridge_during_expected_services_rediscovery(*device);
                const bool preserve_active_bridge_runtime =
                    should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(*device);
                maybe_capture_disconnect_transition(*device);
                peers_->sync_device(*device,
                                    active_config_,
                                    util::device_hostname_guess(*device),
                                    preserve_ready_runtime,
                                    preserve_active_bridge_runtime);
                refresh_device = *device;
            }
        }
    }

    if (refresh_device && !active_config_.auto_pair) {
        const auto session_it = peers_->sessions().find(refresh_device->mac);
        if (session_it != peers_->sessions().end() &&
            session_it->second.desired &&
            device_has_recorded_bond(*refresh_device)) {
            peers_->request_device_reset(session_it->second,
                                         "pairing disabled by config, removing unexpected peer bond",
                                         true);
            schedule_peer_reconcile(std::chrono::milliseconds(1));
        }
    }

    state_lock.unlock();
    if (emit_disconnect_log) {
        if (disconnect_expected) {
            log_info_coalesced(disconnect_log_key, disconnect_log_message);
        } else {
            log_warn_coalesced(disconnect_log_key, disconnect_log_message);
        }
    }
    if (clear_runtime_mac) {
        clear_peer_runtime(*clear_runtime_mac);
    }
    if (refresh_device) {
        refresh_import_bridges_for_device(*refresh_device);
    }

    if (event == bluez::CacheEvent::DeviceAdded ||
        event == bluez::CacheEvent::DeviceRemoved ||
        event == bluez::CacheEvent::DevicePropertyChanged) {
        publish_scan_snapshot();
    }

    schedule_peer_reconcile();
}

void ServiceNode::on_gatt_event(const std::string& event_type,
                                const std::string& object_path,
                                const std::string& detail) {
    std::unique_lock<std::recursive_mutex> state_lock(state_mutex_);
    if (event_type == "client_notify_enabled") {
        log_info_coalesced("gatt:notify-enabled:" + object_path,
                           "[client] notify enabled path=" + object_path);
    } else if (event_type == "client_notify_disabled") {
        log_info_coalesced("gatt:notify-disabled:" + object_path,
                           "[client] notify disabled path=" + object_path);
    } else if (is_failed_gatt_event(event_type)) {
        log_warn_coalesced("gatt:" + event_type + ":" + object_path + ":" + detail,
                           "[client] " + event_type + " path=" + object_path + " detail=" + detail);
    } else if (is_read_write_gatt_event(event_type)) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000,
                              "[client] %s path=%s",
                              event_type.c_str(), object_path.c_str());
    } else {
        log_info_coalesced("gatt:" + event_type + ":" + object_path + ":" + detail,
                           "[client] " + event_type + " path=" + object_path + " detail=" + detail);
    }

    if (!cache_ || !client_) {
        return;
    }

    std::optional<std::string> clear_runtime_mac;
    std::optional<bluez::DeviceInfo> refresh_device;

    if (event_type == "client_notify_disabled" || event_type == "client_notify_failed") {
        std::string affected_mac;
        for (const auto& [mac, bridge] : peers_->time_bridges()) {
            if (bridge.characteristic_path == object_path) {
                affected_mac = mac;
                break;
            }
        }
        if (!affected_mac.empty() && peer_runtime_clear_in_progress_.count(affected_mac) == 0) {
            if (event_type == "client_notify_failed") {
                const auto session_it = peers_->sessions().find(affected_mac);
                if (session_it != peers_->sessions().end()) {
                    auto& session = session_it->second;
                    const auto failure_now = peers_->now_monotonic();
                    if (session.last_notify_failure_characteristic_path == object_path) {
                        session.notify_failure_count += 1;
                    } else {
                        session.notify_failure_count = 1;
                    }
                    session.last_notify_failure_monotonic = failure_now;
                    session.last_notify_failure_characteristic_path = object_path;
                    if (session.bridge_wait_started_monotonic <= 0.0) {
                        session.bridge_wait_started_monotonic = session.last_notify_failure_monotonic;
                    }
                    session.bridge_wait_reason = "notify";
                    session.phase = "connected_unready";
                    session.detail = "failed to enable peer time notifications";

                    if (!session.time_bridge_healthy_this_connection) {
                        if (session.time_bridge_init_notify_failure_monotonic > 0.0 &&
                            (failure_now - session.time_bridge_init_notify_failure_monotonic) <= kPeerInitNotifyFailureResetWindow) {
                            session.time_bridge_init_notify_failure_count += 1;
                        } else {
                            session.time_bridge_init_notify_failure_count = 1;
                        }
                        session.time_bridge_init_notify_failure_monotonic = failure_now;

                        const auto device = client_->get_device(affected_mac);
                        if (device &&
                            device_has_recorded_bond(*device) &&
                            session.time_bridge_init_notify_failure_count >= 3) {
                            peers_->request_device_reset(
                                session,
                                "bonded peer repeatedly rejected time notifications, resetting peer device state",
                                true);
                            RCLCPP_WARN(
                                get_logger(),
                                "[gatt] %s: repeated init-time peer time notify failures indicate stale local bond, arming BlueZ device reset",
                                affected_mac.c_str());
                            schedule_peer_reconcile(std::chrono::milliseconds(1));
                        }
                    }
                }
            }
            clear_runtime_mac = affected_mac;
        }
    }

    if (const auto device_path = device_path_for_gatt_object(*cache_, object_path)) {
        if (const auto device = cache_->device(*device_path)) {
            const bool preserve_ready_runtime =
                has_ready_peer_time_bridge(device->mac) ||
                should_preserve_ready_bridge_during_expected_services_rediscovery(*device);
            const bool preserve_active_bridge_runtime =
                should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(*device);
            peers_->sync_device(*device,
                                active_config_,
                                util::device_hostname_guess(*device),
                                preserve_ready_runtime,
                                preserve_active_bridge_runtime);
            if (!clear_runtime_mac) {
                refresh_device = *device;
            }
        }
    }

    state_lock.unlock();
    if (clear_runtime_mac) {
        clear_peer_runtime(*clear_runtime_mac, object_path);
    }
    if (refresh_device) {
        refresh_import_bridges_for_device(*refresh_device);
    }
    schedule_peer_reconcile();
}

void ServiceNode::on_pairing_event(const std::string& event_type, const std::string& device_path) {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    if (!peers_ || !cache_) {
        return;
    }

    if (!active_config_.auto_pair) {
        return;
    }

    RCLCPP_INFO(get_logger(), "[node] on_pairing_event: type=%s device=%s",
                event_type.c_str(), device_path.c_str());

    if (event_type == "cancel" && device_path.empty()) {
        peer::PeerConnectionSession* candidate_session = nullptr;
        double latest_activity_monotonic = 0.0;
        const double now = peers_->now_monotonic();

        for (auto& [mac, session] : peers_->sessions()) {
            (void)mac;
            if (!session.desired || session.repair_requested || session.repair_in_progress) {
                continue;
            }
            if (!session.pairing_in_progress && session.phase != "securing") {
                continue;
            }

            const double activity_monotonic = std::max(session.last_pairing_request_monotonic,
                                                       session.last_security_attempt_monotonic);
            if (activity_monotonic <= 0.0 ||
                (now - activity_monotonic) > kPairingCancelAssociationTimeout) {
                continue;
            }
            if (!candidate_session || activity_monotonic > latest_activity_monotonic) {
                candidate_session = &session;
                latest_activity_monotonic = activity_monotonic;
            }
        }

        if (candidate_session) {
            candidate_session->pairing_in_progress = false;
            candidate_session->pairing_failures += 1;
            peers_->request_device_reset(*candidate_session,
                                         "pairing cancelled by remote, resetting peer device state",
                                         true);
            schedule_peer_reconcile(std::chrono::milliseconds(1));
            return;
        }
    }

    peers_->note_pairing_event(device_path, event_type, *cache_);
    schedule_peer_reconcile();
}

}  // namespace mrs_uav_bluetooth::app