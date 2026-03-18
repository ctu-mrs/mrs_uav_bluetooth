// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bluez {

/// Event types delivered to observers.
enum class CacheEvent {
    DeviceAdded,
    DeviceRemoved,
    DevicePropertyChanged,
    GattServiceAdded,
    GattServiceRemoved,
    GattCharacteristicAdded,
    GattCharacteristicChanged,
    GattCharacteristicValueChanged,
    GattCharacteristicRemoved,
    GattDescriptorAdded,
    GattDescriptorChanged,
    GattDescriptorValueChanged,
    GattDescriptorRemoved,
    AdapterChanged,
};

/// Callback signature for cache change notifications.
using CacheEventCallback = std::function<void(CacheEvent event,
                                              const std::string& object_path)>;

/// Maintains an in-memory cache of BlueZ managed objects by subscribing to
/// InterfacesAdded, InterfacesRemoved, and PropertiesChanged D-Bus signals.
/// After the initial GetManagedObjects call the cache is kept up-to-date
/// entirely through signal-driven updates.
class ObjectManagerCache {
public:
    ObjectManagerCache(DbusConnection& dbus, rclcpp::Logger logger);
    ~ObjectManagerCache();

    ObjectManagerCache(const ObjectManagerCache&) = delete;
    ObjectManagerCache& operator=(const ObjectManagerCache&) = delete;

    /// Perform the initial GetManagedObjects and subscribe to signals.
    void start();

    /// Register an observer. Called from the D-Bus event-loop thread.
    /// Returns a token that can later be removed.
    int add_observer(CacheEventCallback cb);
    void remove_observer(int token);

    // ---- Adapter queries ----
    std::optional<AdapterInfo> adapter(const std::string& path) const;
    std::vector<AdapterInfo> adapters() const;

    // ---- Device queries ----
    std::optional<DeviceInfo> device(const std::string& path) const;
    std::optional<DeviceInfo> device_by_mac(const std::string& mac) const;
    std::vector<DeviceInfo> devices() const;
    std::vector<DeviceInfo> connected_devices() const;

    // ---- GATT queries ----
    std::optional<GattServiceInfo> service(const std::string& path) const;
    std::optional<GattCharacteristicInfo> characteristic(const std::string& path) const;
    std::optional<GattDescriptorInfo> descriptor(const std::string& path) const;
    std::vector<GattServiceInfo> services_for_device(const std::string& device_path) const;
    std::vector<GattCharacteristicInfo> characteristics_for_service(const std::string& service_path) const;
    std::vector<GattDescriptorInfo> descriptors_for_characteristic(const std::string& chrc_path) const;

    std::optional<GattCharacteristicInfo> find_characteristic_by_uuid(
        const std::string& device_path, const std::string& uuid) const;
    std::optional<GattDescriptorInfo> find_descriptor_by_uuid(
        const std::string& chrc_path, const std::string& uuid) const;

private:
    using InterfaceMap = std::map<std::string, std::map<std::string, sdbus::Variant>>;
    using PendingNotifications = std::vector<std::pair<CacheEvent, std::string>>;

    void load_initial_objects();
    void on_interfaces_added(const sdbus::ObjectPath& path, const InterfaceMap& ifaces);
    void on_interfaces_removed(const sdbus::ObjectPath& path, const std::vector<std::string>& ifaces);
    void on_properties_changed(const sdbus::ObjectPath& path,
                               const std::string& interface,
                               const std::map<std::string, sdbus::Variant>& changed,
                               const std::vector<std::string>& invalidated);

    void process_object(const std::string& path,
                        const InterfaceMap& ifaces,
                        PendingNotifications* notifications);
    void remove_interfaces(const std::string& path,
                           const std::vector<std::string>& ifaces,
                           PendingNotifications* notifications);

    DeviceInfo parse_device(const std::string& path,
                            const std::map<std::string, sdbus::Variant>& props) const;
    void update_device_props(DeviceInfo& dev,
                             const std::map<std::string, sdbus::Variant>& props) const;
    AdapterInfo parse_adapter(const std::string& path,
                              const std::map<std::string, sdbus::Variant>& props) const;
    GattServiceInfo parse_gatt_service(const std::string& path,
                                       const std::map<std::string, sdbus::Variant>& props) const;
    GattCharacteristicInfo parse_gatt_characteristic(const std::string& path,
                                                      const std::map<std::string, sdbus::Variant>& props) const;
    GattDescriptorInfo parse_gatt_descriptor(const std::string& path,
                                              const std::map<std::string, sdbus::Variant>& props) const;

    void notify(CacheEvent event, const std::string& path);

    // Helpers to read typed values from Variant maps.
    template <typename T>
    static T get_or(const std::map<std::string, sdbus::Variant>& m,
                    const std::string& key, T fallback);
    static std::string get_string(const std::map<std::string, sdbus::Variant>& m,
                                  const std::string& key);
    static std::vector<std::string> get_string_vector(
        const std::map<std::string, sdbus::Variant>& m, const std::string& key);

    DbusConnection& dbus_;
    rclcpp::Logger logger_;
    std::unique_ptr<sdbus::IProxy> om_proxy_;

    mutable std::mutex mutex_;
    std::map<std::string, AdapterInfo> adapters_;
    std::map<std::string, DeviceInfo> devices_;
    std::map<std::string, GattServiceInfo> gatt_services_;
    std::map<std::string, GattCharacteristicInfo> gatt_characteristics_;
    std::map<std::string, GattDescriptorInfo> gatt_descriptors_;

    int next_observer_token_{1};
    std::map<int, CacheEventCallback> observers_;
};

}  // namespace mrs_uav_bluetooth::bluez
