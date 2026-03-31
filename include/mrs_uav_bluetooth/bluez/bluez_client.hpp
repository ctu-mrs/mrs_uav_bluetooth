// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bluez {

/// Notification callback: (data, uuid, characteristic_path).
using NotificationCallback = std::function<void(const std::vector<uint8_t>&,
                                                const std::string&,
                                                const std::string&)>;

/// GATT event callback: (event_type, object_path, extra_info).
using GattEventCallback = std::function<void(const std::string& event,
                                             const std::string& path,
                                             const std::string& detail)>;

/// High-level BLE client wrapping BlueZ Device1, GattCharacteristic1, and
/// GattDescriptor1 D-Bus interfaces. Delegates device/GATT caching to
/// ObjectManagerCache and uses blocking control-path operations.
class BluezClient {
public:
    BluezClient(DbusConnection& dbus,
                ObjectManagerCache& cache,
                const std::string& adapter_path,
                rclcpp::Logger logger);
    ~BluezClient();

    // ---- Discovery ----
    bool start_scan(const std::string& transport = "le",
                    bool make_discoverable_while_scanning = false);
    bool stop_scan();
    bool is_scanning() const;

    // ---- Device queries (from cache) ----
    std::vector<DeviceInfo> get_devices() const;
    std::optional<DeviceInfo> get_device(const std::string& mac) const;
    std::vector<DeviceInfo> get_connected_devices() const;

    // ---- Connection management ----
    bool connect(const std::string& mac,
                 double timeout_s = 15.0,
                 bool prefer_le = false);
    bool disconnect(const std::string& mac, double timeout_s = 10.0);

    // ---- Pairing/trust ----
    bool pair(const std::string& mac,
              double timeout_s = 30.0,
              std::string* error_detail = nullptr);
    bool trust(const std::string& mac);
    bool untrust(const std::string& mac);
    bool block(const std::string& mac);
    bool unblock(const std::string& mac);
    bool remove(const std::string& mac);
    bool set_preferred_bearer(const std::string& mac, const std::string& bearer);

    // ---- Services resolved waiter ----
    bool wait_services_resolved(const std::string& mac, double timeout_s = 15.0);
    bool refresh_gatt_snapshot(const std::string& mac) const;

    // ---- GATT queries (from cache) ----
    std::vector<GattServiceInfo> list_services(const std::string& mac) const;
    std::vector<GattCharacteristicInfo> list_characteristics(const std::string& mac) const;
    std::vector<GattDescriptorInfo> list_descriptors(const std::string& mac,
                                                      const std::string& chrc_path = "") const;
    std::string find_characteristic(const std::string& mac, const std::string& uuid) const;
    std::string find_descriptor(const std::string& mac, const std::string& uuid,
                                const std::string& chrc_path = "") const;

    // ---- GATT read/write ----
    std::vector<uint8_t> read_characteristic(const std::string& chrc_path);
    bool write_characteristic(const std::string& chrc_path,
                              const std::vector<uint8_t>& data,
                              bool with_response = true);
    bool write_characteristic_async(const std::string& chrc_path,
                                    const std::vector<uint8_t>& data,
                                    bool with_response = true);
    std::vector<uint8_t> read_descriptor(const std::string& desc_path);
    bool write_descriptor(const std::string& desc_path,
                          const std::vector<uint8_t>& data);
    bool write_descriptor_async(const std::string& desc_path,
                                const std::vector<uint8_t>& data);

    // ---- Notifications ----
    bool start_notify(const std::string& chrc_path);
    bool stop_notify(const std::string& chrc_path);

    /// Register for notification value callbacks.  Returns a token for removal.
    int add_notification_handler(NotificationCallback cb);
    void remove_notification_handler(int token);

    /// Register for GATT event callbacks.
    int add_gatt_event_handler(GattEventCallback cb);
    void remove_gatt_event_handler(int token);

private:
    void on_cache_event(CacheEvent event, const std::string& object_path);
    std::string device_path_for_mac(const std::string& mac) const;
    bool refresh_device_gatt_cache(const std::string& device_path) const;
    void set_device_property(const std::string& device_path,
                             const std::string& prop,
                             const sdbus::Variant& value);
    void emit_gatt(const std::string& event, const std::string& path,
                   const std::string& detail = "");

    DbusConnection& dbus_;
    ObjectManagerCache& cache_;
    std::string adapter_path_;
    rclcpp::Logger logger_;

    mutable std::mutex mutex_;
    int cache_observer_token_{0};
    bool scan_running_{false};
    mutable std::map<std::string, std::chrono::steady_clock::time_point> gatt_refresh_backoff_until_;

    int next_ntf_token_{1};
    std::map<int, NotificationCallback> notification_cbs_;

    int next_gatt_token_{1};
    std::map<int, GattEventCallback> gatt_event_cbs_;

    /// Tracks characteristics for which notifications are enabled through the cache observer path.
    std::set<std::string> notify_paths_;
    void ensure_notify_match(const std::string& chrc_path);
    void remove_notify_match(const std::string& chrc_path);
};

}  // namespace mrs_uav_bluetooth::bluez
