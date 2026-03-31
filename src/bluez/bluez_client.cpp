// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace mrs_uav_bluetooth::bluez {

namespace {

using ManagedObjectMap = std::map<sdbus::ObjectPath,
                                  std::map<std::string, std::map<std::string, sdbus::Variant>>>;

constexpr auto kConnectPollInterval = std::chrono::milliseconds(300);
constexpr auto kPairPollInterval = std::chrono::milliseconds(500);
constexpr auto kGattRefreshRetryBackoff = std::chrono::milliseconds(3000);

std::unique_ptr<sdbus::IConnection> create_blocking_system_bus() {
    return sdbus::createSystemBusConnection();
}

std::unique_ptr<sdbus::IProxy> create_bluez_proxy(sdbus::IConnection& connection,
                                                  const std::string& object_path) {
    return sdbus::createProxy(connection,
                              sdbus::ServiceName{std::string(kBluezServiceName)},
                              sdbus::ObjectPath{object_path});
}

template<typename T>
T get_variant_or(const std::map<std::string, sdbus::Variant>& values,
                 const std::string& key,
                 T fallback) {
    auto it = values.find(key);
    if (it == values.end()) {
        return fallback;
    }
    try {
        return it->second.get<T>();
    } catch (...) {
        return fallback;
    }
}

bool message_contains(const std::string& message,
                      std::initializer_list<const char*> needles) {
    for (const char* needle : needles) {
        if (message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool is_missing_object_error(const std::string& message) {
    if (message_contains(message, {"NoSuchObject", "UnknownObject"})) {
        return true;
    }
    return message.find("GetAll") != std::string::npos &&
           message.find("doesn't exist") != std::string::npos;
}

std::string device_root_path(const std::string& object_path) {
    const auto device_pos = object_path.find("/dev_");
    if (device_pos == std::string::npos) {
        return {};
    }
    const auto suffix_pos = object_path.find('/', device_pos + 1);
    if (suffix_pos == std::string::npos) {
        return object_path;
    }
    return object_path.substr(0, suffix_pos);
}

std::optional<std::map<std::string, sdbus::Variant>> read_device_properties(
    sdbus::IConnection& connection,
    const std::string& device_path) {
    auto proxy = create_bluez_proxy(connection, device_path);
    std::map<std::string, sdbus::Variant> properties;
    proxy->callMethod("GetAll")
        .onInterface(std::string(kDbusPropertiesIface))
        .withArguments(std::string{kDeviceIface})
        .storeResultsTo(properties);
    return properties;
}

bool device_has_resolved_characteristics(sdbus::IConnection& connection,
                                         const std::string& device_path) {
    auto proxy = create_bluez_proxy(connection, "/");
    ManagedObjectMap objects;
    proxy->callMethod("GetManagedObjects")
        .onInterface(std::string(kDbusObjectManagerIface))
        .storeResultsTo(objects);

    for (const auto& [path, interfaces] : objects) {
        const auto path_string = static_cast<std::string>(path);
        if (path_string.find(device_path + "/") != 0) {
            continue;
        }
        if (interfaces.count(std::string(kGattCharacteristicIface)) != 0) {
            return true;
        }
    }
    return false;
}

template<typename Predicate>
bool poll_until(double timeout_s,
                std::chrono::milliseconds interval,
                Predicate&& predicate) {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(std::max(0.0, timeout_s) * 1000.0));
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(interval);
    }
    return predicate();
}

}  // namespace

BluezClient::BluezClient(DbusConnection& dbus,
                         ObjectManagerCache& cache,
                         const std::string& adapter_path,
                         rclcpp::Logger logger)
    : dbus_(dbus),
      cache_(cache),
      adapter_path_(adapter_path),
      logger_(logger) {
    cache_observer_token_ = cache_.add_observer(
        [this](CacheEvent event, const std::string& object_path) {
            on_cache_event(event, object_path);
        });
}

BluezClient::~BluezClient() {
    if (cache_observer_token_ != 0) {
        cache_.remove_observer(cache_observer_token_);
        cache_observer_token_ = 0;
    }

    // Release per-characteristic notify match slots.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        notify_paths_.clear();
    }
}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

bool BluezClient::start_scan(const std::string& transport,
                             bool make_discoverable_while_scanning) {
    RCLCPP_INFO(logger_, "[client] start_scan transport=%s discoverable=%s",
                transport.c_str(), make_discoverable_while_scanning ? "true" : "false");
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        // Set discovery filter.
        std::map<std::string, sdbus::Variant> filter;
        filter["Transport"] = sdbus::Variant{transport};
        filter["Discoverable"] = sdbus::Variant{make_discoverable_while_scanning};
        proxy->callMethod("SetDiscoveryFilter")
            .onInterface(std::string(kAdapterIface))
            .withArguments(filter);
        proxy->callMethod("StartDiscovery")
            .onInterface(std::string(kAdapterIface));
        std::lock_guard<std::mutex> lock(mutex_);
        scan_running_ = true;
        return true;
    } catch (const sdbus::Error& e) {
        std::string msg = e.getMessage();
        if (msg.find("InProgress") != std::string::npos) {
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = true;
            return true;
        }
        RCLCPP_WARN(logger_, "start_scan failed: %s", msg.c_str());
        return false;
    }
}

bool BluezClient::stop_scan() {
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        proxy->callMethod("StopDiscovery")
            .onInterface(std::string(kAdapterIface));
        std::lock_guard<std::mutex> lock(mutex_);
        scan_running_ = false;
        return true;
    } catch (const sdbus::Error& e) {
        std::string msg = e.getMessage();
        if (msg.find("NotAuthorized") != std::string::npos ||
            msg.find("No discovery started") != std::string::npos) {
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = false;
            return true;
        }
        RCLCPP_WARN(logger_, "stop_scan failed: %s", msg.c_str());
        return false;
    }
}

bool BluezClient::is_scanning() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_running_;
}

// ---------------------------------------------------------------------------
// Device queries
// ---------------------------------------------------------------------------

std::vector<DeviceInfo> BluezClient::get_devices() const {
    return cache_.devices();
}

std::optional<DeviceInfo> BluezClient::get_device(const std::string& mac) const {
    return cache_.device_by_mac(mac);
}

std::vector<DeviceInfo> BluezClient::get_connected_devices() const {
    return cache_.connected_devices();
}

// ---------------------------------------------------------------------------
// Connection management
// ---------------------------------------------------------------------------

bool BluezClient::connect(const std::string& mac, double timeout_s) {
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return false;
    if (dev->connected) return true;
    const auto path = dev->object_path;
    RCLCPP_INFO(logger_, "[client] connect(%s) path=%s timeout=%.1fs",
                mac.c_str(), path.c_str(), timeout_s);

    auto connection = create_blocking_system_bus();
    try {
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("Connect")
            .onInterface(std::string(kDeviceIface));
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (!message_contains(message, {"Already Connected", "AlreadyConnected", "InProgress",
                                        "In Progress", "Operation already in progress",
                                        "No more profiles to connect to", "br-connection-already-connected"})) {
            RCLCPP_WARN(logger_, "connect(%s) failed: %s", mac.c_str(), message.c_str());
            return false;
        }
    }

    return poll_until(timeout_s, kConnectPollInterval, [&]() {
        try {
            const auto properties = read_device_properties(*connection, path);
            return properties && get_variant_or<bool>(*properties, "Connected", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    });
}

bool BluezClient::disconnect(const std::string& mac, double timeout_s) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return true;
    auto dev = cache_.device_by_mac(mac);
    if (!dev || !dev->connected) return true;
    RCLCPP_INFO(logger_, "[client] disconnect(%s) path=%s timeout=%.1fs",
                mac.c_str(), path.c_str(), timeout_s);

    auto connection = create_blocking_system_bus();
    try {
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("Disconnect")
            .onInterface(std::string(kDeviceIface));
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (!message_contains(message, {"NotConnected", "NoSuchObject", "UnknownObject"})) {
            RCLCPP_WARN(logger_, "disconnect(%s) failed: %s", mac.c_str(), message.c_str());
            return false;
        }
        return true;
    }

    return poll_until(timeout_s, kConnectPollInterval, [&]() {
        try {
            const auto properties = read_device_properties(*connection, path);
            return !properties || !get_variant_or<bool>(*properties, "Connected", false);
        } catch (const sdbus::Error& error) {
            return is_missing_object_error(error.getMessage());
        }
    });
}

// ---------------------------------------------------------------------------
// Pairing/trust
// ---------------------------------------------------------------------------

bool BluezClient::pair(const std::string& mac, double timeout_s, std::string* error_detail) {
    if (error_detail != nullptr) {
        error_detail->clear();
    }
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return false;
    if (dev->paired) return true;
    const auto path = dev->object_path;
    RCLCPP_INFO(logger_, "[client] pair(%s) path=%s timeout=%.1fs",
                mac.c_str(), path.c_str(), timeout_s);

    auto connection = create_blocking_system_bus();
    try {
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("Pair")
            .onInterface(std::string(kDeviceIface));
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (!message_contains(message, {"AlreadyExists", "Already Paired", "AlreadyPaired", "InProgress",
                                        "In Progress", "Operation already in progress"})) {
            if (error_detail != nullptr) {
                *error_detail = message;
            }
            RCLCPP_WARN(logger_, "pair(%s) failed: %s", mac.c_str(), message.c_str());
            return false;
        }
    }

    const bool paired = poll_until(timeout_s, kPairPollInterval, [&]() {
        try {
            const auto properties = read_device_properties(*connection, path);
            return properties && get_variant_or<bool>(*properties, "Paired", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    });
    if (!paired && error_detail != nullptr && error_detail->empty()) {
        *error_detail = "timed out waiting for Paired=true";
    }
    return paired;
}

bool BluezClient::trust(const std::string& mac) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    RCLCPP_INFO(logger_, "[client] trust(%s) path=%s", mac.c_str(), path.c_str());
    try {
        set_device_property(path, "Trusted", sdbus::Variant{true});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "trust(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::untrust(const std::string& mac) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "Trusted", sdbus::Variant{false});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "untrust(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::block(const std::string& mac) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "Blocked", sdbus::Variant{true});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "block(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::unblock(const std::string& mac) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "Blocked", sdbus::Variant{false});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "unblock(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::remove(const std::string& mac) {
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return false;
    RCLCPP_INFO(logger_, "[client] remove(%s) path=%s", mac.c_str(), dev->object_path.c_str());
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        proxy->callMethod("RemoveDevice")
            .onInterface(std::string(kAdapterIface))
            .withArguments(sdbus::ObjectPath{dev->object_path});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "remove(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::set_preferred_bearer(const std::string& mac, const std::string& bearer) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "PreferredBearer", sdbus::Variant{bearer});
        return true;
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (message_contains(message, {"UnknownProperty", "InvalidArguments", "NotSupported", "does not exist"})) {
            RCLCPP_DEBUG(logger_, "set_preferred_bearer(%s, %s) unavailable: %s",
                         mac.c_str(), bearer.c_str(), message.c_str());
            return false;
        }
        RCLCPP_WARN(logger_, "set_preferred_bearer(%s, %s) failed: %s",
                    mac.c_str(), bearer.c_str(), message.c_str());
        return false;
    }
}

// ---------------------------------------------------------------------------
// Services resolved
// ---------------------------------------------------------------------------

bool BluezClient::wait_services_resolved(const std::string& mac, double timeout_s) {
    auto dev = cache_.device_by_mac(mac);
    if (!dev) {
        return false;
    }

    const auto path = dev->object_path;
    auto connection = create_blocking_system_bus();
    return poll_until(timeout_s, kConnectPollInterval, [&]() {
        try {
            const auto properties = read_device_properties(*connection, path);
            if (!properties) {
                return false;
            }
            if (!get_variant_or<bool>(*properties, "Connected", false)) {
                return false;
            }
            return get_variant_or<bool>(*properties, "ServicesResolved", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    });
}

// ---------------------------------------------------------------------------
// GATT queries
// ---------------------------------------------------------------------------

std::vector<GattServiceInfo> BluezClient::list_services(const std::string& mac) const {
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return {};
    auto services = cache_.services_for_device(dev->object_path);
    if (services.empty() && dev->services_resolved) {
        if (refresh_device_gatt_cache(dev->object_path)) {
            services = cache_.services_for_device(dev->object_path);
        }
    }
    return services;
}

std::vector<GattCharacteristicInfo> BluezClient::list_characteristics(const std::string& mac) const {
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return {};
    std::vector<GattCharacteristicInfo> result;
    auto services = cache_.services_for_device(dev->object_path);
    if (services.empty() && dev->services_resolved) {
        if (refresh_device_gatt_cache(dev->object_path)) {
            services = cache_.services_for_device(dev->object_path);
        }
    }
    for (const auto& svc : services) {
        auto chars = cache_.characteristics_for_service(svc.object_path);
        result.insert(result.end(), chars.begin(), chars.end());
    }
    return result;
}

std::vector<GattDescriptorInfo> BluezClient::list_descriptors(
    const std::string& mac, const std::string& chrc_path) const {
    if (!chrc_path.empty()) {
        return cache_.descriptors_for_characteristic(chrc_path);
    }
    std::vector<GattDescriptorInfo> result;
    for (const auto& ch : list_characteristics(mac)) {
        auto descs = cache_.descriptors_for_characteristic(ch.object_path);
        result.insert(result.end(), descs.begin(), descs.end());
    }
    return result;
}

int BluezClient::add_notification_handler(NotificationCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    int token = next_ntf_token_++;
    notification_cbs_[token] = std::move(cb);
    return token;
}

void BluezClient::remove_notification_handler(int token) {
    std::lock_guard<std::mutex> lock(mutex_);
    notification_cbs_.erase(token);
}

int BluezClient::add_gatt_event_handler(GattEventCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    int token = next_gatt_token_++;
    gatt_event_cbs_[token] = std::move(cb);
    return token;
}

void BluezClient::remove_gatt_event_handler(int token) {
    std::lock_guard<std::mutex> lock(mutex_);
    gatt_event_cbs_.erase(token);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool BluezClient::refresh_device_gatt_cache(const std::string& device_path) const {
    if (device_path.empty()) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = gatt_refresh_backoff_until_.find(device_path);
        if (it != gatt_refresh_backoff_until_.end() && now < it->second) {
            return false;
        }
        gatt_refresh_backoff_until_[device_path] = now + kGattRefreshRetryBackoff;
    }

    auto& cache = const_cast<ObjectManagerCache&>(cache_);
    const bool refreshed = cache.refresh_device_subtree(device_path);

    if (refreshed) {
        std::lock_guard<std::mutex> lock(mutex_);
        gatt_refresh_backoff_until_.erase(device_path);
    }

    return refreshed;
}

void BluezClient::on_cache_event(CacheEvent event, const std::string& object_path) {
    if (const auto device_path = device_root_path(object_path); !device_path.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        gatt_refresh_backoff_until_.erase(device_path);
    }

    if (event == CacheEvent::AdapterChanged && object_path == adapter_path_) {
        if (auto adapter = cache_.adapter(adapter_path_)) {
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = adapter->discovering;
            RCLCPP_DEBUG(logger_, "[client] adapter changed: discovering=%s",
                         adapter->discovering ? "true" : "false");
        }
    }
    if (event == CacheEvent::GattCharacteristicValueChanged) {
        const auto characteristic = cache_.characteristic(object_path);
        if (characteristic) {
            RCLCPP_DEBUG(logger_, "[client] notification from %s uuid=%s %zu bytes",
                         object_path.c_str(), characteristic->uuid.c_str(),
                         characteristic->value.size());
            std::vector<NotificationCallback> cbs;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cbs.reserve(notification_cbs_.size());
                for (auto& [_, cb] : notification_cbs_) {
                    cbs.push_back(cb);
                }
            }
            for (auto& cb : cbs) {
                try {
                    cb(characteristic->value, characteristic->uuid, object_path);
                } catch (...) {
                }
            }
        }
    }
}

std::string BluezClient::device_path_for_mac(const std::string& mac) const {
    auto dev = cache_.device_by_mac(mac);
    return dev ? dev->object_path : std::string{};
}

void BluezClient::set_device_property(const std::string& device_path,
                                      const std::string& prop,
                                      const sdbus::Variant& value) {
    auto connection = create_blocking_system_bus();
    auto proxy = create_bluez_proxy(*connection, device_path);
    proxy->callMethod("Set")
        .onInterface(std::string(kDbusPropertiesIface))
        .withArguments(std::string{kDeviceIface}, prop, value);
}

void BluezClient::emit_gatt(const std::string& event,
                             const std::string& path,
                             const std::string& detail) {
    std::vector<GattEventCallback> cbs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cbs.reserve(gatt_event_cbs_.size());
        for (auto& [_, cb] : gatt_event_cbs_) {
            cbs.push_back(cb);
        }
    }
    for (auto& cb : cbs) {
        try {
            cb(event, path, detail);
        } catch (...) {}
    }
}

void BluezClient::ensure_notify_match(const std::string& chrc_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!notify_paths_.insert(chrc_path).second) {
        return;
    }
    RCLCPP_DEBUG(logger_, "[client] tracking notify path via cache observer for %s", chrc_path.c_str());
}

void BluezClient::remove_notify_match(const std::string& chrc_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    notify_paths_.erase(chrc_path);
}

}  // namespace mrs_uav_bluetooth::bluez
