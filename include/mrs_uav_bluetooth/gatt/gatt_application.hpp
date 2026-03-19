// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::gatt {

using PropertyMap = std::map<std::string, sdbus::Variant>;
using InterfacePropsMap = std::map<std::string, PropertyMap>;
using ManagedObjects = std::map<sdbus::ObjectPath, InterfacePropsMap>;

class GattService;
class GattCharacteristic;
class GattDescriptor;

// ---------------------------------------------------------------------------
// GattDescriptor
// ---------------------------------------------------------------------------

/// Base class for an exported GATT descriptor.
class GattDescriptor {
public:
    GattDescriptor(bluez::DbusConnection& dbus,
                   const std::string& object_path,
                   const std::string& uuid,
                   const std::vector<std::string>& flags,
                   GattCharacteristic& parent);
    virtual ~GattDescriptor();

    void export_object();
    void unexport();

    const std::string& path() const { return path_; }
    const std::string& uuid() const { return uuid_; }

    PropertyMap get_properties() const;
    ManagedObjects get_managed_objects() const;

    void set_value(const std::vector<uint8_t>& val, bool emit = false);
    std::vector<uint8_t> value() const;

protected:
    virtual std::vector<uint8_t> on_read(const std::map<std::string, sdbus::Variant>& options);
    virtual void on_write(const std::vector<uint8_t>& data,
                          const std::map<std::string, sdbus::Variant>& options);

    friend class GattService;
    friend class GattCharacteristic;

    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string uuid_;
    std::vector<std::string> flags_;
    GattCharacteristic& parent_;
    uint16_t handle_{0};

    mutable std::mutex mutex_;
    std::vector<uint8_t> value_;
    std::unique_ptr<sdbus::IObject> exported_;
};

// ---------------------------------------------------------------------------
// GattCharacteristic
// ---------------------------------------------------------------------------

/// Callback for read/write/notify.
using ReadCallback = std::function<std::vector<uint8_t>()>;
using WriteCallback = std::function<void(const std::vector<uint8_t>&)>;
using NotifyCallback = std::function<void(bool enabled)>;

class GattCharacteristic {
public:
    GattCharacteristic(bluez::DbusConnection& dbus,
                       const std::string& object_path,
                       const std::string& uuid,
                       const std::vector<std::string>& flags,
                       GattService& parent);
    virtual ~GattCharacteristic();

    void export_object();
    void unexport();

    const std::string& path() const { return path_; }
    const std::string& uuid() const { return uuid_; }
    const std::vector<std::string>& flags() const { return flags_; }
    bool notifying() const { return notifying_; }
    void set_force_emit_value(bool enabled) { force_emit_value_ = enabled; }

    PropertyMap get_properties() const;
    ManagedObjects get_managed_objects() const;

    void add_descriptor(std::shared_ptr<GattDescriptor> desc);
    const std::vector<std::shared_ptr<GattDescriptor>>& descriptors() const { return descriptors_; }

    void set_value(const std::vector<uint8_t>& val, bool emit = false);
    std::vector<uint8_t> value() const;

    /// Push a notification to connected peers.
    void publish(const std::vector<uint8_t>& data);

    /// Install callbacks for read/write/notify events.
    void set_read_callback(ReadCallback cb) { read_cb_ = std::move(cb); }
    void set_write_callback(WriteCallback cb) { write_cb_ = std::move(cb); }
    void set_notify_callback(NotifyCallback cb) { notify_cb_ = std::move(cb); }

protected:
    virtual std::vector<uint8_t> on_read(const std::map<std::string, sdbus::Variant>& options);
    virtual void on_write(const std::vector<uint8_t>& data,
                          const std::map<std::string, sdbus::Variant>& options);
    virtual void on_start_notify();
    virtual void on_stop_notify();

    friend class GattService;

    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string uuid_;
    std::vector<std::string> flags_;
    GattService& parent_;
    bool notifying_{false};
    bool force_emit_value_{false};
    uint16_t handle_{0};
    std::optional<uint16_t> mtu_;

    mutable std::mutex mutex_;
    std::vector<uint8_t> value_;
    std::unique_ptr<sdbus::IObject> exported_;

    std::vector<std::shared_ptr<GattDescriptor>> descriptors_;

    ReadCallback read_cb_;
    WriteCallback write_cb_;
    NotifyCallback notify_cb_;
};

// ---------------------------------------------------------------------------
// GattService
// ---------------------------------------------------------------------------

class GattService {
public:
    GattService(bluez::DbusConnection& dbus,
                const std::string& object_path,
                const std::string& uuid,
                bool primary = true);
    virtual ~GattService();

    void export_object();
    void unexport();

    const std::string& path() const { return path_; }
    const std::string& uuid() const { return uuid_; }

    PropertyMap get_properties() const;
    ManagedObjects get_managed_objects() const;

    void add_characteristic(std::shared_ptr<GattCharacteristic> chrc);
    const std::vector<std::shared_ptr<GattCharacteristic>>& characteristics() const {
        return characteristics_;
    }

private:
    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string uuid_;
    bool primary_;
    uint16_t handle_{0};
    std::unique_ptr<sdbus::IObject> exported_;
    std::vector<std::shared_ptr<GattCharacteristic>> characteristics_;
};

// ---------------------------------------------------------------------------
// GattApplication
// ---------------------------------------------------------------------------

/// Exported ObjectManager for the local GATT application tree.
class GattApplication {
public:
    GattApplication(bluez::DbusConnection& dbus,
                    const std::string& app_path,
                    rclcpp::Logger logger);
    ~GattApplication();

    void add_service(std::shared_ptr<GattService> svc);

    /// Export the ObjectManager interface and register with BlueZ GattManager1.
    void register_application(const std::string& adapter_path);

    /// Unregister from BlueZ and unexport.
    void unregister_application(const std::string& adapter_path);

    const std::string& path() const { return path_; }
    const std::vector<std::shared_ptr<GattService>>& services() const { return services_; }
    ManagedObjects get_managed_objects() const;

private:
    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string adapter_path_;
    rclcpp::Logger logger_;
    std::unique_ptr<sdbus::IObject> exported_;
    std::vector<std::shared_ptr<GattService>> services_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::gatt
