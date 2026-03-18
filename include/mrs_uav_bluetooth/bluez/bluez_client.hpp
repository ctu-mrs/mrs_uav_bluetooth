// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <functional>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
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

using OperationResultCallback = std::function<void(bool, const std::string&)>;
using OperationTimeoutScheduler = std::function<rclcpp::TimerBase::SharedPtr(
    std::chrono::milliseconds,
    std::function<void()>)>;

/// High-level BLE client wrapping BlueZ Device1, GattCharacteristic1, and
/// GattDescriptor1 D-Bus interfaces.  Delegates device/GATT caching to
/// ObjectManagerCache and operates asynchronously where possible.
class BluezClient {
public:
    BluezClient(DbusConnection& dbus,
                ObjectManagerCache& cache,
                const std::string& adapter_path,
                rclcpp::Logger logger,
                OperationTimeoutScheduler timeout_scheduler);
    ~BluezClient();

    // ---- Discovery ----
    bool start_scan(const std::string& transport = "le");
    bool stop_scan();
    bool is_scanning() const;

    // ---- Device queries (from cache) ----
    std::vector<DeviceInfo> get_devices() const;
    std::optional<DeviceInfo> get_device(const std::string& mac) const;
    std::vector<DeviceInfo> get_connected_devices() const;

    // ---- Connection management ----
    bool connect(const std::string& mac, double timeout_s = 15.0);
    bool connect_async(const std::string& mac);
    void connect_async(const std::string& mac,
                       OperationResultCallback cb,
                       double timeout_s = 15.0);
    bool disconnect(const std::string& mac, double timeout_s = 10.0);
    void disconnect_async(const std::string& mac,
                          OperationResultCallback cb,
                          double timeout_s = 10.0);

    // ---- Pairing/trust ----
    bool pair(const std::string& mac, double timeout_s = 30.0);
    bool pair_async(const std::string& mac);
    void pair_async(const std::string& mac,
                    OperationResultCallback cb,
                    double timeout_s = 30.0);
    bool trust(const std::string& mac);
    bool untrust(const std::string& mac);
    bool remove(const std::string& mac);

    // ---- Services resolved waiter ----
    bool wait_services_resolved(const std::string& mac, double timeout_s = 15.0);
    void wait_services_resolved_async(const std::string& mac,
                                      OperationResultCallback cb,
                                      double timeout_s = 15.0);

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
    struct PendingOperation;

    void on_cache_event(CacheEvent event, const std::string& object_path);
    std::string device_path_for_mac(const std::string& mac) const;
    void set_device_property(const std::string& device_path,
                             const std::string& prop,
                             const sdbus::Variant& value);
    void emit_gatt(const std::string& event, const std::string& path,
                   const std::string& detail = "");
    std::shared_ptr<PendingOperation> start_pending_operation(std::function<bool()> predicate,
                                                              std::function<bool(CacheEvent, const std::string&)> event_filter,
                                                              OperationResultCallback cb,
                                                              std::chrono::milliseconds timeout,
                                                              std::string timeout_detail);
    void resolve_pending_operation(const std::shared_ptr<PendingOperation>& operation,
                                   bool success,
                                   const std::string& detail);
    void evaluate_pending_operations();
    void evaluate_pending_operations(CacheEvent event, const std::string& object_path);

    DbusConnection& dbus_;
    ObjectManagerCache& cache_;
    std::string adapter_path_;
    rclcpp::Logger logger_;
    OperationTimeoutScheduler timeout_scheduler_;

    mutable std::mutex mutex_;
    int cache_observer_token_{0};
    bool scan_running_{false};
    uint64_t next_pending_operation_id_{1};
    std::map<uint64_t, std::shared_ptr<PendingOperation>> pending_operations_;

    int next_ntf_token_{1};
    std::map<int, NotificationCallback> notification_cbs_;

    int next_gatt_token_{1};
    std::map<int, GattEventCallback> gatt_event_cbs_;
};

}  // namespace mrs_uav_bluetooth::bluez
