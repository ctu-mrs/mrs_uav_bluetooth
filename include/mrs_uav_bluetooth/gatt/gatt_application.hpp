// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/gatt_application.hpp
/// \brief Declares the gatt application component of the Bluetooth Low Energy GATT layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
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
// GattProfile
// ---------------------------------------------------------------------------

/// Local org.bluez.GattProfile1 client profile. Registering it as part of a
/// GATT application asks BlueZ to reconnect devices exposing any listed UUID.
class GattProfile {
public:
    using ReleaseCallback = std::function<void()>;

    /// \brief Define a client profile object and its service UUID filter.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param object_path D-Bus path where the client profile is exported.
    /// \param uuids Remote service UUIDs BlueZ should connect automatically.
    GattProfile(bluez::DbusConnection& dbus,
                std::string object_path,
                std::vector<std::string> uuids);
    /// \brief Remove the client profile object from D-Bus.
    ~GattProfile();

    /// \brief Export the GattProfile1 UUID filter and release method.
    void export_object();
    /// \brief Stop exporting GATT profile.
    void unexport();

    /// \brief Return the D-Bus path of this client profile.
    /// \return Path exported as org.bluez.GattProfile1.
    const std::string& path() const {
        // Return the D-Bus path at which this client profile is exported.
        return path_;
    }
    /// \brief Return this object’s Bluetooth UUIDs.
    /// \return Configured Bluetooth UUID list.
    const std::vector<std::string>& uuids() const {
        // Return the service UUID filter advertised by this client profile.
        return uuids_;
    }
    /// \brief Install the observer invoked when BlueZ releases this profile.
    /// \param callback Handler invoked when BlueZ releases the exported profile.
    void set_release_callback(ReleaseCallback callback) {
        // Install the owner callback invoked when BlueZ releases this profile.
        release_callback_ = std::move(callback);
    }

private:
    bluez::DbusConnection& dbus_;
    std::string path_;
    std::vector<std::string> uuids_;
    ReleaseCallback release_callback_;
    std::unique_ptr<sdbus::IObject> exported_;
};

// ---------------------------------------------------------------------------
// GattDescriptor
// ---------------------------------------------------------------------------

/// Base class for an exported GATT descriptor.
class GattDescriptor {
public:
    /// \brief Define a descriptor and bind it to its parent characteristic.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param object_path D-Bus path where this descriptor is exported.
    /// \param uuid Stable UUID exported for this descriptor.
    /// \param flags BlueZ descriptor capabilities such as read or write.
    /// \param parent Characteristic that owns and exposes this descriptor.
    GattDescriptor(bluez::DbusConnection& dbus,
                   const std::string& object_path,
                   const std::string& uuid,
                   const std::vector<std::string>& flags,
                   GattCharacteristic& parent);
    /// \brief Remove the descriptor object from D-Bus.
    virtual ~GattDescriptor();

    /// \brief Export this descriptor's properties and read/write methods.
    void export_object();
    /// \brief Stop exporting GATT descriptor.
    void unexport();

    /// \brief Return the D-Bus path of this descriptor.
    /// \return Path exported as org.bluez.GattDescriptor1.
    const std::string& path() const {
        // Return the D-Bus path at which this descriptor is exported.
        return path_;
    }
    /// \brief Return the Bluetooth type UUID of this descriptor.
    /// \return UUID exported through the descriptor interface.
    const std::string& uuid() const {
        // Return the descriptor type UUID exposed to remote clients.
        return uuid_;
    }

    /// \brief Build this descriptor's GattDescriptor1 property dictionary.
    /// \return UUID, characteristic path, flags, value, and handle properties.
    PropertyMap get_properties() const;
    /// \brief Add this descriptor to an ObjectManager snapshot.
    /// \return One managed-object entry keyed by the descriptor path.
    ManagedObjects get_managed_objects() const;

    /// \brief Replace cached GATT bytes and optionally emit a Value change.
    /// \param val new cached byte value.
    /// \param emit whether to signal the D-Bus Value property after updating bytes.
    void set_value(const std::vector<uint8_t>& val, bool emit = false);
    /// \brief Copy the current bytes under the value lock.
    /// \return Locked copy of the bytes currently exported through Value.
    std::vector<uint8_t> value() const;

protected:
    /// \brief Serve a remote descriptor read from the cached value.
    /// \param options BlueZ read options such as peer device path and offset.
    /// \return Current descriptor bytes returned to the remote reader.
    virtual std::vector<uint8_t> on_read(const std::map<std::string, sdbus::Variant>& options);
    /// \brief Store bytes written remotely to this descriptor.
    /// \param data Bytes supplied by the remote GATT client.
    /// \param options BlueZ write options such as peer device path offset and write type.
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
    // Atomic shared snapshots keep an in-flight emission alive across unexport.
    std::shared_ptr<sdbus::IObject> exported_;
};

// ---------------------------------------------------------------------------
// GattCharacteristic
// ---------------------------------------------------------------------------

/// Callback for read/write/notify.
using ReadCallback = std::function<std::vector<uint8_t>()>;
using WriteCallback = std::function<void(const std::vector<uint8_t>&)>;
using NotifyCallback = std::function<void(bool enabled)>;

/// Exported GATT characteristic with read, write, and notification callbacks.
/// BlueZ invokes these callbacks from the shared D-Bus event-loop thread.
class GattCharacteristic {
public:
    /// \brief Define a value characteristic and bind it to its parent service.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param object_path D-Bus path where this characteristic is exported.
    /// \param uuid Stable UUID exported for this characteristic.
    /// \param flags BlueZ characteristic capabilities such as read write or notify.
    /// \param parent Service that owns and exposes this characteristic.
    GattCharacteristic(bluez::DbusConnection& dbus,
                       const std::string& object_path,
                       const std::string& uuid,
                       const std::vector<std::string>& flags,
                       GattService& parent);
    /// \brief Remove the characteristic and descriptor subtree from D-Bus.
    virtual ~GattCharacteristic();

    /// \brief Export this characteristic's value methods and notification controls.
    void export_object();
    /// \brief Stop exporting GATT characteristic.
    void unexport();

    /// \brief Return the D-Bus path of this characteristic.
    /// \return Path exported as org.bluez.GattCharacteristic1.
    const std::string& path() const {
        // Return the D-Bus path at which this characteristic is exported.
        return path_;
    }
    /// \brief Return the Bluetooth type UUID of this characteristic.
    /// \return UUID exported through the characteristic interface.
    const std::string& uuid() const {
        // Return the characteristic type UUID exposed to remote clients.
        return uuid_;
    }
    /// \brief Expose read, write, and notification capabilities.
    /// \return GATT capabilities exported for this characteristic.
    const std::vector<std::string>& flags() const {
        // Return the read, write, and notification capabilities exported to BlueZ.
        return flags_;
    }
    /// \brief Return whether notifications are enabled.
    /// \return Whether a remote client currently subscribes to notifications.
    bool notifying() const {
        // Report whether BlueZ currently has a notification subscriber.
        return notifying_;
    }
    /// \brief Choose whether repeated identical values still notify subscribed clients.
    /// \param enabled Whether identical characteristic values still emit a D-Bus change signal.
    void set_force_emit_value(bool enabled) {
        // Force Value signals even when the bytes repeat, as required for time samples.
        force_emit_value_ = enabled;
    }

    /// \brief Build this characteristic's GattCharacteristic1 properties.
    /// \return UUID, service path, flags, value, notification, MTU, and handle properties.
    PropertyMap get_properties() const;
    /// \brief Add this characteristic and its descriptors to ObjectManager state.
    /// \return Managed entries for the characteristic and descriptor subtree.
    ManagedObjects get_managed_objects() const;

    /// \brief Attach a descriptor and include it in ObjectManager snapshots.
    /// \param desc descriptor object transferred into its parent characteristic.
    void add_descriptor(std::shared_ptr<GattDescriptor> desc);
    /// \brief Access descriptors owned by this characteristic.
    /// \return GATT descriptors owned by this characteristic.
    const std::vector<std::shared_ptr<GattDescriptor>>& descriptors() const {
        // Return descriptors owned and exported beneath this characteristic.
        return descriptors_;
    }

    /// \brief Replace cached GATT bytes and optionally emit a Value change.
    /// \param val new cached byte value.
    /// \param emit whether to signal the D-Bus Value property after updating bytes.
    void set_value(const std::vector<uint8_t>& val, bool emit = false);
    /// \brief Copy the current bytes under the value lock.
    /// \return Locked copy of the bytes currently exported through Value.
    std::vector<uint8_t> value() const;

    /// Push a new characteristic value to subscribed GATT clients.
    /// \param data Raw characteristic bytes to cache and notify.
    void publish(const std::vector<uint8_t>& data);

    /// Install callbacks for read/write/notify events.
    /// \param cb Function supplying the characteristic value for a remote read.
    void set_read_callback(ReadCallback cb) {
        // Install the producer used to refresh bytes for each remote ReadValue.
        read_cb_ = std::move(cb);
    }
    /// \brief Install the application handler for remote characteristic writes.
    /// \param cb Function consuming bytes from a remote write.
    void set_write_callback(WriteCallback cb) {
        // Install the consumer invoked after each remote WriteValue.
        write_cb_ = std::move(cb);
    }
    /// \brief Install the handler for first-subscribe and last-unsubscribe transitions.
    /// \param cb Function notified when the first subscriber arrives or the last leaves.
    void set_notify_callback(NotifyCallback cb) {
        // Install the observer for the first subscribe and final unsubscribe transitions.
        notify_cb_ = std::move(cb);
    }

protected:
    /// \brief Refresh and return bytes for a remote characteristic read.
    /// \param options BlueZ read options such as peer device path and offset.
    /// \return Current characteristic bytes returned to the remote reader.
    virtual std::vector<uint8_t> on_read(const std::map<std::string, sdbus::Variant>& options);
    /// \brief Store a remote characteristic write and notify its application callback.
    /// \param data Bytes supplied by the remote GATT client.
    /// \param options BlueZ write options such as peer device path offset and write type.
    virtual void on_write(const std::vector<uint8_t>& data,
                          const std::map<std::string, sdbus::Variant>& options);
    /// \brief Enable notification state and call the application on the first subscriber.
    virtual void on_start_notify();
    /// \brief Disable notification state and call the application after the last subscriber.
    virtual void on_stop_notify();

    friend class GattService;

    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string uuid_;
    std::vector<std::string> flags_;
    GattService& parent_;
    std::atomic_bool notifying_{false};
    std::atomic_bool force_emit_value_{false};
    uint16_t handle_{0};
    std::optional<uint16_t> mtu_;

    mutable std::mutex mutex_;
    std::vector<uint8_t> value_;
    // Atomic shared snapshots keep an in-flight emission alive across unexport.
    std::shared_ptr<sdbus::IObject> exported_;

    std::vector<std::shared_ptr<GattDescriptor>> descriptors_;

    ReadCallback read_cb_;
    WriteCallback write_cb_;
    NotifyCallback notify_cb_;
};

// ---------------------------------------------------------------------------
// GattService
// ---------------------------------------------------------------------------

/// Container exported as org.bluez.GattService1 with child characteristics.
class GattService {
public:
    /// \brief Define a service that owns a characteristic subtree.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param object_path D-Bus path where this service is exported.
    /// \param uuid Stable UUID exported for this service.
    /// \param primary Whether BlueZ exposes this as a primary rather than secondary service.
    GattService(bluez::DbusConnection& dbus,
                const std::string& object_path,
                const std::string& uuid,
                bool primary = true);
    /// \brief Remove the complete service subtree from D-Bus.
    virtual ~GattService();

    /// \brief Export this service's UUID, role, includes, and handle properties.
    void export_object();
    /// \brief Stop exporting GATT service.
    void unexport();

    /// \brief Return the D-Bus path of this service.
    /// \return Path exported as org.bluez.GattService1.
    const std::string& path() const {
        // Return the D-Bus path at which this service is exported.
        return path_;
    }
    /// \brief Return the Bluetooth type UUID of this service.
    /// \return UUID exported through the service interface.
    const std::string& uuid() const {
        // Return the service type UUID exposed to remote clients.
        return uuid_;
    }

    /// \brief Build this service's GattService1 property dictionary.
    /// \return UUID, primary role, includes, and handle properties.
    PropertyMap get_properties() const;
    /// \brief Add this service and its children to ObjectManager state.
    /// \return Managed entries for the service, characteristics, and descriptors.
    ManagedObjects get_managed_objects() const;

    /// \brief Attach a characteristic and include it in ObjectManager snapshots.
    /// \param chrc characteristic object transferred into its parent service.
    void add_characteristic(std::shared_ptr<GattCharacteristic> chrc);
    /// \brief Access characteristics owned by this service.
    /// \return GATT characteristics owned by this service.
    const std::vector<std::shared_ptr<GattCharacteristic>>& characteristics() const {
        // Return characteristics owned and exported beneath this service.
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
    /// \brief Create the owner of the local GATT ObjectManager tree.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param app_path path of the app.
    /// \param logger ROS logger used for diagnostics.
    GattApplication(bluez::DbusConnection& dbus,
                    const std::string& app_path,
                    rclcpp::Logger logger);
    /// \brief Unregister and remove the local GATT application tree.
    ~GattApplication();

    /// \brief Attach a service and include its subtree in ObjectManager snapshots.
    /// \param svc service object transferred into the GATT application.
    void add_service(std::shared_ptr<GattService> svc);
    /// \brief Attach a client profile and include it in ObjectManager snapshots.
    /// \param profile client profile transferred into the GATT application lifecycle.
    void add_profile(std::shared_ptr<GattProfile> profile);

    /// Export the ObjectManager interface and register with BlueZ GattManager1.
    /// \param adapter_path BlueZ adapter whose GATT manager registers or removes this application.
    void register_application(const std::string& adapter_path);

    /// Unregister from BlueZ and unexport.
    /// \param adapter_path BlueZ adapter whose GATT manager registers or removes this application.
    void unregister_application(const std::string& adapter_path);

    /// \brief Return the ObjectManager root registered as the GATT application.
    /// \return Application root path passed to GattManager1.
    const std::string& path() const {
        // Return the ObjectManager root registered as the GATT application.
        return path_;
    }
    /// \brief Access services owned by this application.
    /// \return GATT services owned by this application.
    const std::vector<std::shared_ptr<GattService>>& services() const {
        // Return services owned by this registered application.
        return services_;
    }
    /// \brief Access client profiles registered with this application.
    /// \return Client profiles owned by this application.
    const std::vector<std::shared_ptr<GattProfile>>& profiles() const {
        // Return client profiles sharing this application registration lifecycle.
        return profiles_;
    }
    /// \brief Build the complete local GATT ObjectManager snapshot.
    /// \return Managed entries for every profile, service, characteristic, and descriptor.
    ManagedObjects get_managed_objects() const;

private:
    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string adapter_path_;
    rclcpp::Logger logger_;
    std::unique_ptr<sdbus::IObject> exported_;
    std::optional<sdbus::Slot> object_manager_slot_;
    std::vector<std::shared_ptr<GattService>> services_;
    std::vector<std::shared_ptr<GattProfile>> profiles_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::gatt
