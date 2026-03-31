// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/service_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <algorithm>

namespace {

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

std::string device_hostname_guess(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    if (mrs_uav_bluetooth::util::is_uav_hostname(device.name)) {
        return device.name;
    }
    if (mrs_uav_bluetooth::util::is_uav_hostname(device.alias)) {
        return device.alias;
    }
    return {};
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

    if (event == bluez::CacheEvent::DeviceAdded) {
        auto device = cache_->device(object_path);
        if (!device) {
            log_warn_coalesced("cache:DeviceAdded-missing:" + object_path,
                               "[node] on_cache_event: DeviceAdded path=" + object_path + " but device not in cache");
            return;
        }
        const auto peer_name = device_hostname_guess(*device);
        log_info_coalesced("cache:DeviceAdded:" + device->mac,
                           "[node] on_cache_event: DeviceAdded mac=" + device->mac +
                               " name='" + device->name + "' peer_name='" + peer_name +
                               " path=" + object_path);
        peers_->sync_device(*device, active_config_, peer_name);
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
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
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
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
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
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
                refresh_device = *device;
            }
        }
    } else {
        RCLCPP_DEBUG(get_logger(), "[node] on_cache_event: %s path=%s", event_name, object_path.c_str());
        if (event == bluez::CacheEvent::AdapterChanged && object_path == adapter_path_) {
            if (const auto adapter = cache_->adapter(object_path)) {
                const bool should_be_discoverable = active_config_.enable_server;
                const bool drifted = !adapter->powered ||
                    !adapter->pairable ||
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
                peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
                refresh_device = *device;
            }
        }
    }

    state_lock.unlock();
    if (clear_runtime_mac) {
        clear_peer_runtime(*clear_runtime_mac);
    }
    if (refresh_device) {
        refresh_import_bridges_for_device(*refresh_device);
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
        if (!affected_mac.empty()) {
            clear_runtime_mac = affected_mac;
        }
    }

    if (const auto device_path = device_path_for_gatt_object(*cache_, object_path)) {
        if (const auto device = cache_->device(*device_path)) {
            peers_->sync_device(*device, active_config_, device_hostname_guess(*device));
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
    RCLCPP_INFO(get_logger(), "[node] on_pairing_event: type=%s device=%s",
                event_type.c_str(), device_path.c_str());
    if (!peers_ || !cache_) {
        return;
    }
    peers_->note_pairing_event(device_path, event_type, *cache_);
    schedule_peer_reconcile();
}

}  // namespace mrs_uav_bluetooth::app