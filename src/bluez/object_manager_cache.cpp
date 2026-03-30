// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <algorithm>
#include <chrono>

namespace mrs_uav_bluetooth::bluez {

namespace {

using ManagedObjectMap = std::map<sdbus::ObjectPath,
                                  std::map<std::string, std::map<std::string, sdbus::Variant>>>;

void dispatch_notifications(const std::vector<std::pair<CacheEvent, std::string>>& notifications,
                           const std::function<void(CacheEvent, const std::string&)>& notify) {
    for (const auto& [event, path] : notifications) {
        notify(event, path);
    }
}

bool is_child_path(const std::string& parent_path, const std::string& candidate_path) {
    return candidate_path.size() > parent_path.size() &&
           candidate_path.compare(0, parent_path.size(), parent_path) == 0 &&
           candidate_path[parent_path.size()] == '/';
}

}  // namespace

// ---- Variant helpers ----

template <typename T>
T ObjectManagerCache::get_or(const std::map<std::string, sdbus::Variant>& m,
                             const std::string& key, T fallback) {
    auto it = m.find(key);
    if (it == m.end()) return fallback;
    try {
        return it->second.get<T>();
    } catch (...) {
        return fallback;
    }
}

std::string ObjectManagerCache::get_string(
    const std::map<std::string, sdbus::Variant>& m, const std::string& key) {
    return get_or<std::string>(m, key, "");
}

std::vector<std::string> ObjectManagerCache::get_string_vector(
    const std::map<std::string, sdbus::Variant>& m, const std::string& key) {
    return get_or<std::vector<std::string>>(m, key, {});
}

// ---- Construction / destruction ----

ObjectManagerCache::ObjectManagerCache(DbusConnection& dbus, rclcpp::Logger logger)
    : dbus_(dbus), logger_(logger)
{
}

ObjectManagerCache::~ObjectManagerCache() = default;

void ObjectManagerCache::start() {
    om_proxy_ = sdbus::createProxy(dbus_.connection(),
                                   sdbus::ServiceName{std::string(kBluezServiceName)},
                                   sdbus::ObjectPath{"/"});

    // Subscribe to InterfacesAdded / InterfacesRemoved.
    om_proxy_->uponSignal("InterfacesAdded")
             .onInterface(std::string(kDbusObjectManagerIface))
             .call([this](const sdbus::ObjectPath& path, const InterfaceMap& ifaces) {
                 on_interfaces_added(path, ifaces);
             });

    om_proxy_->uponSignal("InterfacesRemoved")
             .onInterface(std::string(kDbusObjectManagerIface))
             .call([this](const sdbus::ObjectPath& path,
                          const std::vector<std::string>& ifaces) {
                 on_interfaces_removed(path, ifaces);
             });

    // Additionally subscribe to PropertiesChanged on the root match-all path.
    // sdbus-c++ signal matching will deliver these for any path under /org/bluez.
    // We use a separate proxy on the well-known name with match-all pattern.
    // Actually, we subscribe at the proxy level but the OM proxy only sees its
    // own path.  For PropertiesChanged on all paths we need a match rule:
    auto& conn = dbus_.connection();
    conn.addMatch(
        "type='signal',"
        "sender='org.bluez',"
        "interface='org.freedesktop.DBus.Properties',"
        "member='PropertiesChanged'",
        [this](sdbus::Message msg) {
            std::string interface;
            std::map<std::string, sdbus::Variant> changed;
            std::vector<std::string> invalidated;
            msg >> interface >> changed >> invalidated;
            on_properties_changed(sdbus::ObjectPath{msg.getPath()},
                                  interface, changed, invalidated);
        });

    load_initial_objects();
    RCLCPP_INFO(logger_, "ObjectManagerCache started: %zu adapters, %zu devices, "
                "%zu services, %zu characteristics, %zu descriptors",
                adapters_.size(), devices_.size(), gatt_services_.size(),
                gatt_characteristics_.size(), gatt_descriptors_.size());
}

int ObjectManagerCache::add_observer(CacheEventCallback cb) {
    std::lock_guard lock(mutex_);
    const int token = next_observer_token_++;
    observers_.emplace(token, std::move(cb));
    return token;
}

void ObjectManagerCache::remove_observer(int token) {
    std::lock_guard lock(mutex_);
    observers_.erase(token);
}

// ---- Queries ----

std::optional<AdapterInfo> ObjectManagerCache::adapter(const std::string& path) const {
    std::lock_guard lock(mutex_);
    auto it = adapters_.find(path);
    return it != adapters_.end() ? std::optional{it->second} : std::nullopt;
}

std::vector<AdapterInfo> ObjectManagerCache::adapters() const {
    std::lock_guard lock(mutex_);
    std::vector<AdapterInfo> result;
    result.reserve(adapters_.size());
    for (const auto& [_, v] : adapters_) result.push_back(v);
    return result;
}

std::optional<DeviceInfo> ObjectManagerCache::device(const std::string& path) const {
    std::lock_guard lock(mutex_);
    auto it = devices_.find(path);
    return it != devices_.end() ? std::optional{it->second} : std::nullopt;
}

std::optional<DeviceInfo> ObjectManagerCache::device_by_mac(const std::string& mac) const {
    std::lock_guard lock(mutex_);
    for (const auto& [_, dev] : devices_) {
        if (dev.mac == mac) return dev;
    }
    return std::nullopt;
}

std::vector<DeviceInfo> ObjectManagerCache::devices() const {
    std::lock_guard lock(mutex_);
    std::vector<DeviceInfo> result;
    result.reserve(devices_.size());
    for (const auto& [_, v] : devices_) result.push_back(v);
    return result;
}

std::vector<DeviceInfo> ObjectManagerCache::connected_devices() const {
    std::lock_guard lock(mutex_);
    std::vector<DeviceInfo> result;
    for (const auto& [_, v] : devices_) {
        if (v.connected) result.push_back(v);
    }
    return result;
}

std::optional<GattServiceInfo> ObjectManagerCache::service(const std::string& path) const {
    std::lock_guard lock(mutex_);
    auto it = gatt_services_.find(path);
    return it != gatt_services_.end() ? std::optional{it->second} : std::nullopt;
}

std::optional<GattCharacteristicInfo> ObjectManagerCache::characteristic(const std::string& path) const {
    std::lock_guard lock(mutex_);
    auto it = gatt_characteristics_.find(path);
    return it != gatt_characteristics_.end() ? std::optional{it->second} : std::nullopt;
}

std::optional<GattDescriptorInfo> ObjectManagerCache::descriptor(const std::string& path) const {
    std::lock_guard lock(mutex_);
    auto it = gatt_descriptors_.find(path);
    return it != gatt_descriptors_.end() ? std::optional{it->second} : std::nullopt;
}

std::vector<GattServiceInfo> ObjectManagerCache::services_for_device(
    const std::string& device_path) const {
    std::lock_guard lock(mutex_);
    std::vector<GattServiceInfo> result;
    for (const auto& [path, svc] : gatt_services_) {
        if (svc.device_path == device_path ||
            path.find(device_path + "/") == 0) {
            result.push_back(svc);
        }
    }
    return result;
}

std::vector<GattCharacteristicInfo> ObjectManagerCache::characteristics_for_service(
    const std::string& service_path) const {
    std::lock_guard lock(mutex_);
    std::vector<GattCharacteristicInfo> result;
    for (const auto& [path, chrc] : gatt_characteristics_) {
        if (chrc.service_path == service_path ||
            path.find(service_path + "/") == 0) {
            result.push_back(chrc);
        }
    }
    return result;
}

std::vector<GattDescriptorInfo> ObjectManagerCache::descriptors_for_characteristic(
    const std::string& chrc_path) const {
    std::lock_guard lock(mutex_);
    std::vector<GattDescriptorInfo> result;
    for (const auto& [path, desc] : gatt_descriptors_) {
        if (desc.characteristic_path == chrc_path ||
            path.find(chrc_path + "/") == 0) {
            result.push_back(desc);
        }
    }
    return result;
}

std::optional<GattCharacteristicInfo> ObjectManagerCache::find_characteristic_by_uuid(
    const std::string& device_path, const std::string& uuid) const {
    std::lock_guard lock(mutex_);
    std::string lower_uuid = uuid;
    std::transform(lower_uuid.begin(), lower_uuid.end(), lower_uuid.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    for (const auto& [path, chrc] : gatt_characteristics_) {
        if (path.find(device_path + "/") != 0) continue;
        std::string chrc_uuid = chrc.uuid;
        std::transform(chrc_uuid.begin(), chrc_uuid.end(), chrc_uuid.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (chrc_uuid == lower_uuid) return chrc;
    }
    return std::nullopt;
}

std::optional<GattDescriptorInfo> ObjectManagerCache::find_descriptor_by_uuid(
    const std::string& chrc_path, const std::string& uuid) const {
    std::lock_guard lock(mutex_);
    std::string lower_uuid = uuid;
    std::transform(lower_uuid.begin(), lower_uuid.end(), lower_uuid.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    for (const auto& [path, desc] : gatt_descriptors_) {
        if (desc.characteristic_path != chrc_path &&
            path.find(chrc_path + "/") != 0) continue;
        std::string desc_uuid = desc.uuid;
        std::transform(desc_uuid.begin(), desc_uuid.end(), desc_uuid.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (desc_uuid == lower_uuid) return desc;
    }
    return std::nullopt;
}

bool ObjectManagerCache::refresh_device_subtree(const std::string& device_path) {
    if (device_path.empty()) {
        return false;
    }

    ManagedObjectMap objects;
    try {
        auto connection = sdbus::createSystemBusConnection();
        auto proxy = sdbus::createProxy(*connection,
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{"/"});
        proxy->callMethod("GetManagedObjects")
            .onInterface(std::string(kDbusObjectManagerIface))
            .storeResultsTo(objects);
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "GetManagedObjects refresh for %s failed: %s",
                    device_path.c_str(), e.getMessage().c_str());
        return false;
    }

    PendingNotifications notifications;
    size_t refreshed_count = 0;
    {
        std::lock_guard lock(mutex_);
        for (const auto& [path, ifaces] : objects) {
            const auto path_string = static_cast<std::string>(path);
            if (path_string != device_path && !is_child_path(device_path, path_string)) {
                continue;
            }
            process_object(path_string, ifaces, &notifications);
            refreshed_count += 1;
        }
    }

    if (refreshed_count == 0) {
        return false;
    }

    RCLCPP_INFO(logger_, "[cache] refreshed %zu object(s) for %s from GetManagedObjects",
                refreshed_count, device_path.c_str());
    dispatch_notifications(notifications, [this](CacheEvent event, const std::string& object_path) {
        notify(event, object_path);
    });
    return true;
}

// ---- Signal handlers ----

void ObjectManagerCache::load_initial_objects() {
    std::map<sdbus::ObjectPath, InterfaceMap> objects;
    try {
        om_proxy_->callMethod("GetManagedObjects")
                 .onInterface(std::string(kDbusObjectManagerIface))
                 .storeResultsTo(objects);
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "GetManagedObjects failed: %s", e.what());
        return;
    }
    RCLCPP_INFO(logger_, "[cache] GetManagedObjects returned %zu objects", objects.size());
    std::lock_guard lock(mutex_);
    for (const auto& [path, ifaces] : objects) {
        RCLCPP_DEBUG(logger_, "[cache] initial object: %s  interfaces=%zu", std::string(path).c_str(), ifaces.size());
        process_object(std::string(path), ifaces, nullptr);
    }
}

void ObjectManagerCache::on_interfaces_added(const sdbus::ObjectPath& path,
                                             const InterfaceMap& ifaces) {
    RCLCPP_INFO(logger_, "[cache] InterfacesAdded path=%s  ifaces=%zu", std::string(path).c_str(), ifaces.size());
    for (const auto& [iface, _] : ifaces) {
        RCLCPP_DEBUG(logger_, "[cache]   iface: %s", iface.c_str());
    }
    PendingNotifications notifications;
    {
        std::lock_guard lock(mutex_);
        process_object(std::string(path), ifaces, &notifications);
    }
    dispatch_notifications(notifications, [this](CacheEvent event, const std::string& object_path) {
        notify(event, object_path);
    });
}

void ObjectManagerCache::on_interfaces_removed(const sdbus::ObjectPath& path,
                                               const std::vector<std::string>& ifaces) {
    RCLCPP_INFO(logger_, "[cache] InterfacesRemoved path=%s  ifaces=%zu", std::string(path).c_str(), ifaces.size());
    for (const auto& iface : ifaces) {
        RCLCPP_DEBUG(logger_, "[cache]   removed iface: %s", iface.c_str());
    }
    PendingNotifications notifications;
    {
        std::lock_guard lock(mutex_);
        remove_interfaces(std::string(path), ifaces, &notifications);
    }
    dispatch_notifications(notifications, [this](CacheEvent event, const std::string& object_path) {
        notify(event, object_path);
    });
}

void ObjectManagerCache::on_properties_changed(
    const sdbus::ObjectPath& path_obj,
    const std::string& interface,
    const std::map<std::string, sdbus::Variant>& changed,
    const std::vector<std::string>& /*invalidated*/) {
    std::string path = std::string(path_obj);
    PendingNotifications notifications;

    RCLCPP_DEBUG(logger_, "[cache] PropertiesChanged path=%s iface=%s changed_keys=%zu",
                path.c_str(), interface.c_str(), changed.size());

    {
        std::lock_guard lock(mutex_);

        if (interface == std::string(kDeviceIface)) {
            auto it = devices_.find(path);
            if (it != devices_.end()) {
                // Log key property changes at INFO level
                if (changed.count("Connected")) {
                    RCLCPP_INFO(logger_, "[cache] Device %s (%s) Connected=%s",
                                it->second.mac.c_str(), path.c_str(),
                                get_or<bool>(changed, "Connected", false) ? "true" : "false");
                }
                if (changed.count("Paired")) {
                    RCLCPP_INFO(logger_, "[cache] Device %s Paired=%s",
                                it->second.mac.c_str(),
                                get_or<bool>(changed, "Paired", false) ? "true" : "false");
                }
                if (changed.count("Trusted")) {
                    RCLCPP_INFO(logger_, "[cache] Device %s Trusted=%s",
                                it->second.mac.c_str(),
                                get_or<bool>(changed, "Trusted", false) ? "true" : "false");
                }
                if (changed.count("ServicesResolved")) {
                    RCLCPP_INFO(logger_, "[cache] Device %s ServicesResolved=%s",
                                it->second.mac.c_str(),
                                get_or<bool>(changed, "ServicesResolved", false) ? "true" : "false");
                }
                if (changed.count("RSSI")) {
                    RCLCPP_DEBUG(logger_, "[cache] Device %s RSSI=%d",
                                it->second.mac.c_str(),
                                get_or<int16_t>(changed, "RSSI", 0));
                }
                update_device_props(it->second, changed);
                notifications.emplace_back(CacheEvent::DevicePropertyChanged, path);
            }
        } else if (interface == std::string(kAdapterIface)) {
            auto it = adapters_.find(path);
            if (it != adapters_.end()) {
                auto& a = it->second;
                if (changed.count("Powered"))
                    a.powered = get_or<bool>(changed, "Powered", a.powered);
                if (changed.count("Connectable"))
                    a.connectable = get_or<bool>(changed, "Connectable", a.connectable);
                if (changed.count("Discoverable"))
                    a.discoverable = get_or<bool>(changed, "Discoverable", a.discoverable);
                if (changed.count("DiscoverableTimeout"))
                    a.discoverable_timeout = get_or<uint32_t>(changed, "DiscoverableTimeout", a.discoverable_timeout);
                if (changed.count("Pairable"))
                    a.pairable = get_or<bool>(changed, "Pairable", a.pairable);
                if (changed.count("Discovering"))
                    a.discovering = get_or<bool>(changed, "Discovering", a.discovering);
                if (changed.count("Alias"))
                    a.alias = get_string(changed, "Alias");
                notifications.emplace_back(CacheEvent::AdapterChanged, path);
            }
        } else if (interface == std::string(kGattCharacteristicIface)) {
            auto it = gatt_characteristics_.find(path);
            if (it != gatt_characteristics_.end()) {
                auto& c = it->second;
                bool changed_metadata = false;
                bool changed_value = false;
                if (changed.count("Notifying")) {
                    c.notifying = get_or<bool>(changed, "Notifying", c.notifying);
                    changed_metadata = true;
                }
                if (changed.count("MTU")) {
                    c.mtu = get_or<uint16_t>(changed, "MTU", c.mtu);
                    changed_metadata = true;
                }
                if (changed.count("Flags")) {
                    c.flags = get_string_vector(changed, "Flags");
                    changed_metadata = true;
                }
                if (changed.count("Value")) {
                    c.value = get_or<std::vector<uint8_t>>(changed, "Value", c.value);
                    changed_value = true;
                }
                if (changed_metadata) {
                    notifications.emplace_back(CacheEvent::GattCharacteristicChanged, path);
                }
                if (changed_value) {
                    notifications.emplace_back(CacheEvent::GattCharacteristicValueChanged, path);
                }
            }
        } else if (interface == std::string(kGattDescriptorIface)) {
            auto it = gatt_descriptors_.find(path);
            if (it != gatt_descriptors_.end()) {
                auto& d = it->second;
                bool changed_metadata = false;
                bool changed_value = false;
                if (changed.count("Flags")) {
                    d.flags = get_string_vector(changed, "Flags");
                    changed_metadata = true;
                }
                if (changed.count("Value")) {
                    d.value = get_or<std::vector<uint8_t>>(changed, "Value", d.value);
                    changed_value = true;
                }
                if (changed_metadata) {
                    notifications.emplace_back(CacheEvent::GattDescriptorChanged, path);
                }
                if (changed_value) {
                    notifications.emplace_back(CacheEvent::GattDescriptorValueChanged, path);
                }
            }
        }
    }

    dispatch_notifications(notifications, [this](CacheEvent event, const std::string& object_path) {
        notify(event, object_path);
    });
}

// ---- Internal processing ----

void ObjectManagerCache::process_object(const std::string& path,
                                        const InterfaceMap& ifaces,
                                        PendingNotifications* notifications) {
    auto adapter_it = ifaces.find(std::string(kAdapterIface));
    if (adapter_it != ifaces.end()) {
        adapters_[path] = parse_adapter(path, adapter_it->second);
        if (notifications) {
            notifications->emplace_back(CacheEvent::AdapterChanged, path);
        }
    }

    auto device_it = ifaces.find(std::string(kDeviceIface));
    if (device_it != ifaces.end()) {
        auto dev = parse_device(path, device_it->second);
        RCLCPP_INFO(logger_, "[cache] Device added: %s mac=%s name='%s' connected=%s paired=%s trusted=%s",
                    path.c_str(), dev.mac.c_str(), dev.name.c_str(),
                    dev.connected ? "true" : "false",
                    dev.paired ? "true" : "false",
                    dev.trusted ? "true" : "false");
        devices_[path] = std::move(dev);
        if (notifications) {
            notifications->emplace_back(CacheEvent::DeviceAdded, path);
        }
    }

    auto svc_it = ifaces.find(std::string(kGattServiceIface));
    if (svc_it != ifaces.end()) {
        auto svc = parse_gatt_service(path, svc_it->second);
        RCLCPP_INFO(logger_, "[cache] GATT service added: %s uuid=%s device=%s primary=%s",
                    path.c_str(), svc.uuid.c_str(), svc.device_path.c_str(),
                    svc.primary ? "true" : "false");
        gatt_services_[path] = std::move(svc);
        if (notifications) {
            notifications->emplace_back(CacheEvent::GattServiceAdded, path);
        }
    }

    auto chrc_it = ifaces.find(std::string(kGattCharacteristicIface));
    if (chrc_it != ifaces.end()) {
        auto chrc = parse_gatt_characteristic(path, chrc_it->second);
        RCLCPP_INFO(logger_, "[cache] GATT characteristic added: %s uuid=%s service=%s notifying=%s",
                    path.c_str(), chrc.uuid.c_str(), chrc.service_path.c_str(),
                    chrc.notifying ? "true" : "false");
        gatt_characteristics_[path] = std::move(chrc);
        if (notifications) {
            notifications->emplace_back(CacheEvent::GattCharacteristicAdded, path);
        }
    }

    auto desc_it = ifaces.find(std::string(kGattDescriptorIface));
    if (desc_it != ifaces.end()) {
        auto desc = parse_gatt_descriptor(path, desc_it->second);
        RCLCPP_INFO(logger_, "[cache] GATT descriptor added: %s uuid=%s chrc=%s",
                    path.c_str(), desc.uuid.c_str(), desc.characteristic_path.c_str());
        gatt_descriptors_[path] = std::move(desc);
        if (notifications) {
            notifications->emplace_back(CacheEvent::GattDescriptorAdded, path);
        }
    }
}

void ObjectManagerCache::remove_interfaces(const std::string& path,
                                           const std::vector<std::string>& ifaces,
                                           PendingNotifications* notifications) {
    auto record_removal = [notifications](CacheEvent event, const std::string& removed_path) {
        if (notifications) {
            notifications->emplace_back(event, removed_path);
        }
    };

    for (const auto& iface : ifaces) {
        if (iface == std::string(kDeviceIface)) {
            std::vector<std::string> descriptor_paths;
            std::vector<std::string> characteristic_paths;
            std::vector<std::string> service_paths;

            for (const auto& [descriptor_path, _] : gatt_descriptors_) {
                if (is_child_path(path, descriptor_path)) {
                    descriptor_paths.push_back(descriptor_path);
                }
            }
            for (const auto& [characteristic_path, _] : gatt_characteristics_) {
                if (is_child_path(path, characteristic_path)) {
                    characteristic_paths.push_back(characteristic_path);
                }
            }
            for (const auto& [service_path, _] : gatt_services_) {
                if (is_child_path(path, service_path)) {
                    service_paths.push_back(service_path);
                }
            }

            for (const auto& descriptor_path : descriptor_paths) {
                gatt_descriptors_.erase(descriptor_path);
                record_removal(CacheEvent::GattDescriptorRemoved, descriptor_path);
            }
            for (const auto& characteristic_path : characteristic_paths) {
                gatt_characteristics_.erase(characteristic_path);
                record_removal(CacheEvent::GattCharacteristicRemoved, characteristic_path);
            }
            for (const auto& service_path : service_paths) {
                gatt_services_.erase(service_path);
                record_removal(CacheEvent::GattServiceRemoved, service_path);
            }

            devices_.erase(path);
            record_removal(CacheEvent::DeviceRemoved, path);
        } else if (iface == std::string(kGattServiceIface)) {
            std::vector<std::string> descriptor_paths;
            std::vector<std::string> characteristic_paths;

            for (const auto& [descriptor_path, _] : gatt_descriptors_) {
                if (is_child_path(path, descriptor_path)) {
                    descriptor_paths.push_back(descriptor_path);
                }
            }
            for (const auto& [characteristic_path, _] : gatt_characteristics_) {
                if (is_child_path(path, characteristic_path)) {
                    characteristic_paths.push_back(characteristic_path);
                }
            }

            for (const auto& descriptor_path : descriptor_paths) {
                gatt_descriptors_.erase(descriptor_path);
                record_removal(CacheEvent::GattDescriptorRemoved, descriptor_path);
            }
            for (const auto& characteristic_path : characteristic_paths) {
                gatt_characteristics_.erase(characteristic_path);
                record_removal(CacheEvent::GattCharacteristicRemoved, characteristic_path);
            }

            gatt_services_.erase(path);
            record_removal(CacheEvent::GattServiceRemoved, path);
        } else if (iface == std::string(kGattCharacteristicIface)) {
            std::vector<std::string> descriptor_paths;

            for (const auto& [descriptor_path, _] : gatt_descriptors_) {
                if (is_child_path(path, descriptor_path)) {
                    descriptor_paths.push_back(descriptor_path);
                }
            }

            for (const auto& descriptor_path : descriptor_paths) {
                gatt_descriptors_.erase(descriptor_path);
                record_removal(CacheEvent::GattDescriptorRemoved, descriptor_path);
            }

            gatt_characteristics_.erase(path);
            record_removal(CacheEvent::GattCharacteristicRemoved, path);
        } else if (iface == std::string(kGattDescriptorIface)) {
            gatt_descriptors_.erase(path);
            record_removal(CacheEvent::GattDescriptorRemoved, path);
        } else if (iface == std::string(kAdapterIface)) {
            adapters_.erase(path);
        }
    }
}

// ---- Parsers ----

DeviceInfo ObjectManagerCache::parse_device(
    const std::string& path,
    const std::map<std::string, sdbus::Variant>& props) const {
    DeviceInfo dev;
    dev.object_path = path;
    update_device_props(dev, props);
    dev.last_seen = std::chrono::steady_clock::now();
    return dev;
}

void ObjectManagerCache::update_device_props(
    DeviceInfo& dev,
    const std::map<std::string, sdbus::Variant>& props) const {
    if (props.count("Address"))       dev.mac = get_string(props, "Address");
    if (props.count("AddressType"))   dev.address_type = get_string(props, "AddressType");
    if (props.count("Name"))          dev.name = get_string(props, "Name");
    if (props.count("Alias"))         dev.alias = get_string(props, "Alias");
    if (props.count("Icon"))          dev.icon = get_string(props, "Icon");
    if (props.count("Appearance"))    dev.appearance = get_or<int32_t>(props, "Appearance", dev.appearance);
    if (props.count("RSSI"))          dev.rssi = get_or<int16_t>(props, "RSSI", dev.rssi);
    if (props.count("TxPower"))       dev.tx_power = get_or<int16_t>(props, "TxPower", dev.tx_power);
    if (props.count("Connected"))     dev.connected = get_or<bool>(props, "Connected", dev.connected);
    if (props.count("Paired"))        dev.paired = get_or<bool>(props, "Paired", dev.paired);
    if (props.count("Bonded"))        dev.bonded = get_or<bool>(props, "Bonded", dev.bonded);
    if (props.count("Trusted"))       dev.trusted = get_or<bool>(props, "Trusted", dev.trusted);
    if (props.count("Blocked"))       dev.blocked = get_or<bool>(props, "Blocked", dev.blocked);
    if (props.count("ServicesResolved")) dev.services_resolved = get_or<bool>(props, "ServicesResolved", dev.services_resolved);
    if (props.count("UUIDs"))         dev.uuids = get_string_vector(props, "UUIDs");
    if (props.count("ManufacturerData")) {
        try {
            auto raw = props.at("ManufacturerData").get<std::map<uint16_t, sdbus::Variant>>();
            dev.manufacturer_data.clear();
            for (const auto& [key, variant] : raw) {
                try {
                    dev.manufacturer_data[key] = variant.get<std::vector<uint8_t>>();
                } catch (...) {}
            }
        } catch (...) {}
    }
    if (props.count("ServiceData")) {
        try {
            auto raw = props.at("ServiceData").get<std::map<std::string, sdbus::Variant>>();
            dev.service_data.clear();
            for (const auto& [key, variant] : raw) {
                try {
                    dev.service_data[key] = variant.get<std::vector<uint8_t>>();
                } catch (...) {}
            }
        } catch (...) {}
    }
    if (props.count("Adapter")) {
        try {
            dev.adapter = static_cast<std::string>(props.at("Adapter").get<sdbus::ObjectPath>());
        } catch (...) {}
    }
    dev.last_seen = std::chrono::steady_clock::now();
}

AdapterInfo ObjectManagerCache::parse_adapter(
    const std::string& path,
    const std::map<std::string, sdbus::Variant>& props) const {
    AdapterInfo a;
    a.object_path = path;
    a.address = get_string(props, "Address");
    a.address_type = get_string(props, "AddressType");
    a.name = get_string(props, "Name");
    a.alias = get_string(props, "Alias");
    a.powered = get_or<bool>(props, "Powered", false);
    a.connectable = get_or<bool>(props, "Connectable", false);
    a.discoverable = get_or<bool>(props, "Discoverable", false);
    a.discoverable_timeout = get_or<uint32_t>(props, "DiscoverableTimeout", 0);
    a.pairable = get_or<bool>(props, "Pairable", false);
    a.discovering = get_or<bool>(props, "Discovering", false);
    a.uuids = get_string_vector(props, "UUIDs");
    return a;
}

GattServiceInfo ObjectManagerCache::parse_gatt_service(
    const std::string& path,
    const std::map<std::string, sdbus::Variant>& props) const {
    GattServiceInfo s;
    s.object_path = path;
    s.uuid = get_string(props, "UUID");
    s.primary = get_or<bool>(props, "Primary", true);
    try {
        if (props.count("Device")) {
            s.device_path = static_cast<std::string>(props.at("Device").get<sdbus::ObjectPath>());
        }
    } catch (...) {}
    s.includes = get_string_vector(props, "Includes");
    return s;
}

GattCharacteristicInfo ObjectManagerCache::parse_gatt_characteristic(
    const std::string& path,
    const std::map<std::string, sdbus::Variant>& props) const {
    GattCharacteristicInfo c;
    c.object_path = path;
    c.uuid = get_string(props, "UUID");
    try {
        if (props.count("Service")) {
            c.service_path = static_cast<std::string>(props.at("Service").get<sdbus::ObjectPath>());
        }
    } catch (...) {}
    c.flags = get_string_vector(props, "Flags");
    c.notifying = get_or<bool>(props, "Notifying", false);
    c.mtu = get_or<uint16_t>(props, "MTU", 0);
    c.value = get_or<std::vector<uint8_t>>(props, "Value", {});
    return c;
}

GattDescriptorInfo ObjectManagerCache::parse_gatt_descriptor(
    const std::string& path,
    const std::map<std::string, sdbus::Variant>& props) const {
    GattDescriptorInfo d;
    d.object_path = path;
    d.uuid = get_string(props, "UUID");
    try {
        if (props.count("Characteristic")) {
            d.characteristic_path = static_cast<std::string>(props.at("Characteristic").get<sdbus::ObjectPath>());
        }
    } catch (...) {}
    d.flags = get_string_vector(props, "Flags");
    d.value = get_or<std::vector<uint8_t>>(props, "Value", {});
    return d;
}

void ObjectManagerCache::notify(CacheEvent event, const std::string& path) {
    std::vector<CacheEventCallback> callbacks;
    {
        std::lock_guard lock(mutex_);
        callbacks.reserve(observers_.size());
        for (const auto& [_, cb] : observers_) {
            callbacks.push_back(cb);
        }
    }

    for (auto& cb : callbacks) {
        try {
            cb(event, path);
        } catch (...) {}
    }
}

}  // namespace mrs_uav_bluetooth::bluez
