// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bluez/object_manager_cache.hpp
/// \brief Declares the object manager cache component of the BlueZ system-D-Bus integration layer.

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
    /// \brief Create the typed, signal-driven view of BlueZ managed objects.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param logger ROS logger used for diagnostics.
    ObjectManagerCache(DbusConnection& dbus, rclcpp::Logger logger);
    /// \brief Remove daemon signal matches before releasing the event connection.
    ~ObjectManagerCache();

    /// \brief Disable copying of the object manager cache.
    ObjectManagerCache(const ObjectManagerCache&) = delete;
    /// \brief Disable copy assignment of the object manager cache.
    ObjectManagerCache& operator=(const ObjectManagerCache&) = delete;

    /// Perform the initial GetManagedObjects and subscribe to signals.
    void start();

    /// Register an observer. Called from the D-Bus event-loop thread.
    /// Returns a token that can later be removed.
    /// \param cb Observer invoked after a BlueZ cache object changes.
    /// \return Token used to remove this cache observer.
    int add_observer(CacheEventCallback cb);
    /// \brief Stop delivering cache changes to the observer identified by its token.
    /// \param token Registration token returned by add_observer.
    void remove_observer(int token);

    // ---- Adapter queries ----
    /// \brief Copy one cached adapter under lock.
    /// \param path BlueZ object path of the adapter to copy.
    /// \return Copy of the named cached adapter or std::nullopt when absent.
    std::optional<AdapterInfo> adapter(const std::string& path) const;
    /// \brief Snapshot all cached adapters under lock.
    /// \return Snapshot of every cached Bluetooth adapter.
    std::vector<AdapterInfo> adapters() const;

    // ---- Device queries ----
    /// \brief Copy one cached device under lock.
    /// \param path BlueZ object path of the device to copy.
    /// \return Copy of the named cached device or std::nullopt when absent.
    std::optional<DeviceInfo> device(const std::string& path) const;
    /// \brief Find one cached device by normalized address.
    /// \param mac peer Bluetooth MAC address.
    /// \return Cached device with the normalized address or std::nullopt when absent.
    std::optional<DeviceInfo> device_by_mac(const std::string& mac) const;
    /// \brief Snapshot all cached devices under lock.
    /// \return Snapshot of every cached Bluetooth device.
    std::vector<DeviceInfo> devices() const;
    /// \brief Snapshot cached devices whose Connected flag is true.
    /// \return Snapshots of cached devices whose Connected property is true.
    std::vector<DeviceInfo> connected_devices() const;

    // ---- GATT queries ----
    /// \brief Copy one cached GATT service under lock.
    /// \param path BlueZ object path of the GATT service to copy.
    /// \return Copy of the named cached service or std::nullopt when absent.
    std::optional<GattServiceInfo> service(const std::string& path) const;
    /// \brief Copy one cached GATT characteristic under lock.
    /// \param path BlueZ object path of the GATT characteristic to copy.
    /// \return Copy of the named cached characteristic or std::nullopt when absent.
    std::optional<GattCharacteristicInfo> characteristic(const std::string& path) const;
    /// \brief Copy one cached GATT descriptor under lock.
    /// \param path BlueZ object path of the GATT descriptor to copy.
    /// \return Copy of the named cached descriptor or std::nullopt when absent.
    std::optional<GattDescriptorInfo> descriptor(const std::string& path) const;
    /// \brief Collect services belonging to one device subtree.
    /// \param device_path BlueZ device path whose child services are returned.
    /// \return Cached services whose Device path matches the requested peer.
    std::vector<GattServiceInfo> services_for_device(const std::string& device_path) const;
    /// \brief Collect characteristics belonging to one service subtree.
    /// \param service_path path of the service.
    /// \return Cached characteristics owned by the requested service.
    std::vector<GattCharacteristicInfo> characteristics_for_service(const std::string& service_path) const;
    /// \brief Collect descriptors belonging to one characteristic subtree.
    /// \param chrc_path BlueZ characteristic path whose child descriptors are returned.
    /// \return Cached descriptors owned by the requested characteristic.
    std::vector<GattDescriptorInfo> descriptors_for_characteristic(const std::string& chrc_path) const;

    /// \brief Find a case-insensitive UUID within one device subtree.
    /// \param device_path BlueZ device path limiting the characteristic search.
    /// \param uuid Characteristic UUID to match below the device.
    /// \return Matching characteristic by UUID when available; otherwise std::nullopt.
    std::optional<GattCharacteristicInfo> find_characteristic_by_uuid(
        const std::string& device_path, const std::string& uuid) const;
    /// \brief Find a case-insensitive UUID within one characteristic subtree.
    /// \param chrc_path Optional BlueZ characteristic path limiting the descriptor search.
    /// \param uuid Descriptor UUID to match below the characteristic.
    /// \return Matching descriptor by UUID when available; otherwise std::nullopt.
    std::optional<GattDescriptorInfo> find_descriptor_by_uuid(
        const std::string& chrc_path, const std::string& uuid) const;

    /// Refresh a device subtree from a fresh GetManagedObjects snapshot when
    /// BlueZ has already resolved services but the signal-driven cache missed
    /// the remote GATT objects. Returns true when the snapshot contained any
    /// remote GATT objects for the device.
    /// \param device_path BlueZ device path whose GATT subtree is reloaded.
    /// \return True if BlueZ returned GATT descendants for the device; otherwise false.
    bool refresh_device_subtree(const std::string& device_path);

private:
    using InterfaceMap = std::map<std::string, std::map<std::string, sdbus::Variant>>;
    using PendingNotifications = std::vector<std::pair<CacheEvent, std::string>>;

    /// \brief Seed the cache from BlueZ's current ObjectManager snapshot.
    void load_initial_objects();
    /// \brief Clear stale objects and reload state when the BlueZ D-Bus owner changes.
    /// \param previous_owner D-Bus unique name of the daemon generation that disappeared.
    /// \param owner new D-Bus unique-name owner after daemon replacement.
    void on_bluez_owner_changed(const std::string& previous_owner,
                                const std::string& owner);
    /// \brief Replace cached BlueZ objects after daemon startup or owner replacement.
    void reload_managed_objects();
    /// \brief Merge interfaces announced by ObjectManager and notify observers afterward.
    /// \param path BlueZ object path that gained the reported interfaces.
    /// \param ifaces Complete interface-property maps newly added at the object path.
    void on_interfaces_added(const sdbus::ObjectPath& path, const InterfaceMap& ifaces);
    /// \brief Remove vanished interfaces and dependent cache entries, then notify observers.
    /// \param path BlueZ object path that lost the reported interfaces.
    /// \param ifaces Interface names removed from the object path.
    void on_interfaces_removed(const sdbus::ObjectPath& path, const std::vector<std::string>& ifaces);
    /// \brief Merge changed properties, erase invalidated ones, and emit precise cache events.
    /// \param path BlueZ object path whose properties changed.
    /// \param interface BlueZ interface name whose properties are read or changed.
    /// \param changed properties BlueZ supplied with new values.
    /// \param invalidated property names BlueZ removed from the object.
    void on_properties_changed(const sdbus::ObjectPath& path,
                               const std::string& interface,
                               const std::map<std::string, sdbus::Variant>& changed,
                               const std::vector<std::string>& invalidated);

    /// \brief Merge one ObjectManager interface snapshot and queue precise cache events.
    /// \param path BlueZ object path whose interface snapshot is being merged.
    /// \param ifaces Current interface-property snapshot to merge into the cache.
    /// \param notifications event batch queued for delivery after releasing the cache lock.
    void process_object(const std::string& path,
                        const InterfaceMap& ifaces,
                        PendingNotifications* notifications);
    /// \brief Erase vanished interfaces and their dependent cached GATT objects.
    /// \param path BlueZ object path from which the interfaces were removed.
    /// \param ifaces Interface names to erase from the cached object.
    /// \param notifications event batch queued for delivery after releasing the cache lock.
    void remove_interfaces(const std::string& path,
                           const std::vector<std::string>& ifaces,
                           PendingNotifications* notifications);

    /// \brief Decode Device1 properties into a normalized peer record.
    /// \param path BlueZ object path assigned to the parsed device.
    /// \param props BlueZ Device1 properties to decode into the cached device.
    /// \return Validated device.
    DeviceInfo parse_device(const std::string& path,
                            const std::map<std::string, sdbus::Variant>& props) const;
    /// \brief Merge changed Device1 properties without discarding fields BlueZ omitted.
    /// \param dev cached device record being updated in place.
    /// \param props BlueZ Device1 properties to decode into the cached device.
    void update_device_props(DeviceInfo& dev,
                             const std::map<std::string, sdbus::Variant>& props) const;
    /// \brief Merge advertising capacity and feature properties into the adapter record.
    /// \param adapter cached adapter record updated with manager properties.
    /// \param props BlueZ LEAdvertisingManager1 properties to merge into the cached adapter.
    void update_advertising_manager_props(AdapterInfo& adapter,
                                          const std::map<std::string, sdbus::Variant>& props) const;
    /// \brief Decode Adapter1 and advertising-manager properties into one adapter record.
    /// \param path BlueZ object path assigned to the parsed adapter.
    /// \param props BlueZ Adapter1 and advertising-manager properties to decode.
    /// \return Validated adapter.
    AdapterInfo parse_adapter(const std::string& path,
                              const std::map<std::string, sdbus::Variant>& props) const;
    /// \brief Decode a GattService1 property map and its parent device relationship.
    /// \param path BlueZ object path assigned to the parsed GATT service.
    /// \param props BlueZ GattService1 properties to decode.
    /// \return Validated GATT service.
    GattServiceInfo parse_gatt_service(const std::string& path,
                                       const std::map<std::string, sdbus::Variant>& props) const;
    /// \brief Decode a GattCharacteristic1 property map and its parent service relationship.
    /// \param path BlueZ object path assigned to the parsed GATT characteristic.
    /// \param props BlueZ GattCharacteristic1 properties to decode.
    /// \return Validated GATT characteristic.
    GattCharacteristicInfo parse_gatt_characteristic(const std::string& path,
                                                      const std::map<std::string, sdbus::Variant>& props) const;
    /// \brief Decode a GattDescriptor1 property map and its parent characteristic relationship.
    /// \param path BlueZ object path assigned to the parsed GATT descriptor.
    /// \param props BlueZ GattDescriptor1 properties to decode.
    /// \return Validated GATT descriptor.
    GattDescriptorInfo parse_gatt_descriptor(const std::string& path,
                                              const std::map<std::string, sdbus::Variant>& props) const;

    /// \brief Deliver one cache mutation to a snapshot of registered observers.
    /// \param event Cache change kind delivered to observers.
    /// \param path Affected BlueZ object path included in the cache event.
    void notify(CacheEvent event, const std::string& path);

    // Helpers to read typed values from Variant maps.
    template <typename T>
    /// \brief Decode a typed property while tolerating missing or mismatched variants.
    /// \param m D-Bus property map from which a typed value is read.
    /// \param key D-Bus property name to retrieve.
    /// \param fallback value returned when the requested property is absent or ill-typed.
    /// \return Decoded typed property or the caller-provided fallback.
    static T get_or(const std::map<std::string, sdbus::Variant>& m,
                    const std::string& key, T fallback);
    /// \brief Decode an optional string property safely.
    /// \param m D-Bus property map from which a typed value is read.
    /// \param key D-Bus property name to retrieve.
    /// \return Decoded string property or an empty string.
    static std::string get_string(const std::map<std::string, sdbus::Variant>& m,
                                  const std::string& key);
    /// \brief Decode an optional string-list property safely.
    /// \param m D-Bus property map from which a typed value is read.
    /// \param key D-Bus property name to retrieve.
    /// \return Decoded string-list property or an empty list.
    static std::vector<std::string> get_string_vector(
        const std::map<std::string, sdbus::Variant>& m, const std::string& key);

    DbusConnection& dbus_;
    rclcpp::Logger logger_;
    std::unique_ptr<sdbus::IProxy> om_proxy_;
    std::optional<sdbus::Slot> daemon_owner_match_;

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
