// SPDX-License-Identifier: BSD-3-Clause
/// \file src/gatt/gatt_application.cpp
/// \brief Implements the gatt application component of the Bluetooth Low Energy GATT layer.

#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"

#include <future>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_bluetooth::gatt {

using namespace mrs_uav_bluetooth::bluez;

namespace {

/// \brief Log a failed property signal without letting a D-Bus callback unwind.
/// \param object_kind GATT object type included in a property-signal warning.
/// \param path D-Bus object whose property notification failed.
/// \param property D-Bus property whose failed change signal is being logged.
/// \param error error details to report.
void log_properties_changed_failure(const char* object_kind,
                                   const std::string& path,
                                   const std::string& property,
                                   const sdbus::Error& error) {
    // Log a failed property signal without letting a D-Bus callback unwind.
    static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("mrs_uav_bluetooth"), throttle_clock, 5000,
                         "[server] failed to emit PropertiesChanged for %s %s property %s: %s",
                         object_kind, path.c_str(), property.c_str(), error.what());
}

}  // namespace

// ===========================================================================
// GattProfile
// ===========================================================================

GattProfile::GattProfile(DbusConnection& dbus,
                         std::string object_path,
                         std::vector<std::string> uuids)
    : dbus_(dbus), path_(std::move(object_path)), uuids_(std::move(uuids)) {
        // Retain the profile path and UUID filter; export is deferred until application registration.
    }

GattProfile::~GattProfile() {
    // Remove the profile object from D-Bus before its connection is released.
    unexport();
}

void GattProfile::export_object() {
    // Export the client-profile UUID filter and BlueZ Release callback at this object path.
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});
    exported_->addVTable(
        sdbus::registerProperty("UUIDs")
            .withGetter([this]() -> std::vector<std::string> {
                // Tell BlueZ which remote services should be connected to this client profile.
                return uuids_;
            }),
        sdbus::registerMethod("Release")
            .implementedAs([this]() {
                // Notify the owner that BlueZ released this client profile registration.
                if (release_callback_) {
                    release_callback_();
                }
            })
    ).forInterface(std::string(kGattProfileIface));
}

void GattProfile::unexport() {
    // Stop exporting GATT profile.
    exported_.reset();
}

// ===========================================================================
// GattDescriptor
// ===========================================================================

GattDescriptor::GattDescriptor(DbusConnection& dbus,
                               const std::string& object_path,
                               const std::string& uuid,
                               const std::vector<std::string>& flags,
                               GattCharacteristic& parent)
    : dbus_(dbus), path_(object_path), uuid_(uuid), flags_(flags), parent_(parent) {
        // Bind this descriptor to its parent characteristic while leaving BlueZ handle allocation pending.
    }

GattDescriptor::~GattDescriptor() {
    // Remove the descriptor object from D-Bus before its parent is destroyed.
    unexport();
}

void GattDescriptor::export_object() {
    auto object = std::shared_ptr<sdbus::IObject>(
        sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_}));

    // Register BlueZ-expected properties + methods on the GattDescriptor1 interface.
    // sdbus-c++ provides org.freedesktop.DBus.Properties (GetAll, Get, Set,
    // PropertiesChanged) for registered properties. Register only
    // GattDescriptor1 here to keep the vtable valid for BlueZ.
    object->addVTable(
        sdbus::registerProperty("UUID")
            .withGetter([this]() -> std::string {
                // Expose this descriptor type to BlueZ and remote GATT clients.
                return uuid_;
            }),
        sdbus::registerProperty("Characteristic")
            .withGetter([this]() -> sdbus::ObjectPath {
                // Link this descriptor to the characteristic object that contains it.
                return sdbus::ObjectPath{parent_.path()};
            }),
        sdbus::registerProperty("Handle")
            .withGetter([this]() -> uint16_t {
                // Expose the descriptor handle, or zero so BlueZ allocates one.
                return handle_;
            }),
        sdbus::registerProperty("Flags")
            .withGetter([this]() -> std::vector<std::string> {
                // Advertise the operations permitted on this descriptor; default to read-only.
                return flags_.empty() ? std::vector<std::string>{"read"} : flags_;
            }),
        sdbus::registerProperty("Value")
            .withGetter([this]() -> std::vector<uint8_t> {
                // Return a locked copy of the descriptor bytes to avoid racing a local update.
                std::lock_guard<std::mutex> lock(mutex_);
                return value_;
            }),
        sdbus::registerMethod("ReadValue")
            .withInputParamNames("options")
            .withOutputParamNames("value")
            .implementedAs([this](const std::map<std::string, sdbus::Variant>& opts)
                -> std::vector<uint8_t> {
                // Serve a remote descriptor read from the descriptor value cache.
                return on_read(opts);
            }),
        sdbus::registerMethod("WriteValue")
            .withInputParamNames("value", "options")
            .implementedAs([this](const std::vector<uint8_t>& data,
                                  const std::map<std::string, sdbus::Variant>& opts) {
                // Apply bytes received from a remote descriptor write.
                on_write(data, opts);
            })
    ).forInterface(std::string(kGattDescriptorIface));

    std::atomic_store(&exported_, std::move(object));
}

void GattDescriptor::unexport() {
    // Drop the registration outside any value/service-state lock. In-flight
    // publishers retain a safe snapshot until their emission completes.
    std::atomic_store(&exported_, std::shared_ptr<sdbus::IObject>{});
}

void GattDescriptor::set_value(const std::vector<uint8_t>& val, bool emit) {
    // Replace the descriptor bytes under lock and optionally emit a Value change.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = val;
    }
    const auto object = std::atomic_load(&exported_);
    if (emit && object) {
        try {
            object->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattDescriptorIface)},
                                                  std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Value"}});
        } catch (const sdbus::Error& error) {
            log_properties_changed_failure("descriptor", path_, "Value", error);
        }
    }
}

std::vector<uint8_t> GattDescriptor::value() const {
    // Copy the descriptor bytes under the same lock used by local and remote writes.
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

std::vector<uint8_t> GattDescriptor::on_read(
    const std::map<std::string, sdbus::Variant>& /*options*/) {
    // Serialize descriptor reads with local value updates.
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

void GattDescriptor::on_write(const std::vector<uint8_t>& data,
                               const std::map<std::string, sdbus::Variant>& /*options*/) {
    // Store the remote write and emit the changed Value property.
    set_value(data, true);
}

// ===========================================================================
// GattCharacteristic
// ===========================================================================

GattCharacteristic::GattCharacteristic(DbusConnection& dbus,
                                       const std::string& object_path,
                                       const std::string& uuid,
                                       const std::vector<std::string>& flags,
                                       GattService& parent)
    : dbus_(dbus), path_(object_path), uuid_(uuid), flags_(flags), parent_(parent) {
        // Bind this characteristic to its parent service while leaving BlueZ handle allocation pending.
    }

GattCharacteristic::~GattCharacteristic() {
    // Remove the characteristic and its descriptors from D-Bus before its parent is destroyed.
    unexport();
}

void GattCharacteristic::export_object() {
    auto object = std::shared_ptr<sdbus::IObject>(
        sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_}));

    // Register BlueZ-expected properties + methods on the GattCharacteristic1
    // interface.  sdbus-c++ automatically provides org.freedesktop.DBus.Properties.
    object->addVTable(
        sdbus::registerProperty("UUID")
            .withGetter([this]() -> std::string {
                // Expose this characteristic type to BlueZ and remote GATT clients.
                return uuid_;
            }),
        sdbus::registerProperty("Service")
            .withGetter([this]() -> sdbus::ObjectPath {
                // Link this characteristic to the service object that contains it.
                return sdbus::ObjectPath{parent_.path()};
            }),
        sdbus::registerProperty("Descriptors")
            .withGetter([this]() -> std::vector<sdbus::ObjectPath> {
                // Enumerate the descriptor object paths BlueZ must include under this characteristic.
                std::vector<sdbus::ObjectPath> desc_paths;
                for (const auto& d : descriptors_) {
                    desc_paths.push_back(sdbus::ObjectPath{d->path()});
                }
                return desc_paths;
            }),
        sdbus::registerProperty("Handle")
            .withGetter([this]() -> uint16_t {
                // Expose the characteristic handle, or zero so BlueZ allocates one.
                return handle_;
            }),
        sdbus::registerProperty("Flags")
            .withGetter([this]() -> std::vector<std::string> {
                // Advertise the read, write, and notification operations this characteristic supports.
                return flags_;
            }),
        sdbus::registerProperty("Notifying")
            .withGetter([this]() -> bool {
                // Report whether a remote client currently requested notifications.
                return notifying_;
            }),
        sdbus::registerProperty("Value")
            .withGetter([this]() -> std::vector<uint8_t> {
                // Return a locked copy of the most recently published characteristic bytes.
                std::lock_guard<std::mutex> lock(mutex_);
                return value_;
            }),
        sdbus::registerMethod("ReadValue")
            .withInputParamNames("options")
            .withOutputParamNames("value")
            .implementedAs([this](const std::map<std::string, sdbus::Variant>& opts)
                -> std::vector<uint8_t> {
                // Obtain the characteristic value, refreshing it through the read handler when configured.
                return on_read(opts);
            }),
        sdbus::registerMethod("WriteValue")
            .withInputParamNames("value", "options")
            .implementedAs([this](const std::vector<uint8_t>& data,
                                  const std::map<std::string, sdbus::Variant>& opts) {
                // Store a remote write and forward its bytes to the application handler.
                on_write(data, opts);
            }),
        sdbus::registerMethod("StartNotify")
            .implementedAs([this]() {
                // Mark notifications active and inform the producer that a subscriber is present.
                on_start_notify();
            }),
        sdbus::registerMethod("StopNotify")
            .implementedAs([this]() {
                // Mark notifications inactive and inform the producer that the subscriber left.
                on_stop_notify();
            })
    ).forInterface(std::string(kGattCharacteristicIface));

    for (auto& desc : descriptors_) {
        desc->export_object();
    }

    std::atomic_store(&exported_, std::move(object));
}

void GattCharacteristic::unexport() {
    for (auto& desc : descriptors_) {
        desc->unexport();
    }
    // Drop the registration outside any value/service-state lock. In-flight
    // publishers retain a safe snapshot until their emission completes.
    std::atomic_store(&exported_, std::shared_ptr<sdbus::IObject>{});
}

void GattCharacteristic::add_descriptor(std::shared_ptr<GattDescriptor> desc) {
    // Transfer this descriptor into the characteristic subtree exported to BlueZ.
    descriptors_.push_back(std::move(desc));
}

void GattCharacteristic::set_value(const std::vector<uint8_t>& val, bool emit) {
    // Replace the characteristic bytes under lock and optionally notify subscribed clients.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = val;
    }
    const auto object = std::atomic_load(&exported_);
    if (emit && object) {
        static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
        RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("mrs_uav_bluetooth"), throttle_clock, 5000,
                              "[server] value changed path=%s bytes=%zu",
                              path_.c_str(), val.size());
        try {
            object->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattCharacteristicIface)},
                                                  std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Value"}});
        } catch (const sdbus::Error& error) {
            log_properties_changed_failure("characteristic", path_, "Value", error);
        }
    }
}

std::vector<uint8_t> GattCharacteristic::value() const {
    // Copy the characteristic bytes under the same lock used by publishers and remote writes.
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

void GattCharacteristic::publish(const std::vector<uint8_t>& data) {
    // Replace the cached value and signal subscribers when notification is active.
    set_value(data, notifying_ || force_emit_value_);
}

std::vector<uint8_t> GattCharacteristic::on_read(
    const std::map<std::string, sdbus::Variant>& /*options*/) {
    // Ask the application for fresh bytes when a dynamic read handler is installed.
    if (read_cb_) {
        auto val = read_cb_();
        set_value(val);
        return val;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

void GattCharacteristic::on_write(const std::vector<uint8_t>& data,
                                   const std::map<std::string, sdbus::Variant>& /*options*/) {
    // Cache the remote write before delivering it to the application callback.
    set_value(data, true);
    if (write_cb_) {
        write_cb_(data);
    }
}

void GattCharacteristic::on_start_notify() {
    // Ignore duplicate subscriptions so the producer sees one enable transition.
    if (notifying_) return;
    notifying_ = true;
    RCLCPP_INFO(rclcpp::get_logger("mrs_uav_bluetooth"),
                "[server] notify enabled path=%s", path_.c_str());
    if (const auto object = std::atomic_load(&exported_)) {
        try {
            object->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattCharacteristicIface)},
                                                  std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Notifying"}});
        } catch (const sdbus::Error& error) {
            log_properties_changed_failure("characteristic", path_, "Notifying", error);
        }
    }
    if (notify_cb_) notify_cb_(true);
}

void GattCharacteristic::on_stop_notify() {
    // Ignore duplicate unsubscriptions so the producer sees one disable transition.
    if (!notifying_) return;
    notifying_ = false;
    RCLCPP_INFO(rclcpp::get_logger("mrs_uav_bluetooth"),
                "[server] notify disabled path=%s", path_.c_str());
    if (const auto object = std::atomic_load(&exported_)) {
        try {
            object->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattCharacteristicIface)},
                                                  std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Notifying"}});
        } catch (const sdbus::Error& error) {
            log_properties_changed_failure("characteristic", path_, "Notifying", error);
        }
    }
    if (notify_cb_) notify_cb_(false);
}

// ===========================================================================
// GattService
// ===========================================================================

GattService::GattService(DbusConnection& dbus,
                         const std::string& object_path,
                         const std::string& uuid,
                         bool primary)
    : dbus_(dbus), path_(object_path), uuid_(uuid), primary_(primary) {
        // Retain the service identity and primary flag until the application exports its object tree.
    }

GattService::~GattService() {
    // Remove the service subtree from D-Bus before its connection is released.
    unexport();
}

void GattService::export_object() {
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});


    // Register BlueZ-expected properties on the GattService1 interface.
    // sdbus-c++ automatically provides org.freedesktop.DBus.Properties.
    exported_->addVTable(
        sdbus::registerProperty("UUID")
            .withGetter([this]() -> std::string {
                // Expose this service type to BlueZ and remote GATT clients.
                return uuid_;
            }),
        sdbus::registerProperty("Primary")
            .withGetter([this]() -> bool {
                // Tell BlueZ whether clients should discover this as a primary service.
                return primary_;
            }),
        sdbus::registerProperty("Characteristics")
            .withGetter([this]() -> std::vector<sdbus::ObjectPath> {
                // Enumerate the characteristic object paths contained by this service.
                std::vector<sdbus::ObjectPath> chrc_paths;
                for (const auto& c : characteristics_) {
                    chrc_paths.push_back(sdbus::ObjectPath{c->path()});
                }
                return chrc_paths;
            }),
        sdbus::registerProperty("Handle")
            .withGetter([this]() -> uint16_t {
                // Expose the service handle, or zero so BlueZ allocates one.
                return handle_;
            }),
        sdbus::registerProperty("Includes")
            .withGetter([]() -> std::vector<sdbus::ObjectPath> {
                // Return the empty list of local services included by this service.
                return {};
            })
    ).forInterface(std::string(kGattServiceIface));

    for (auto& chrc : characteristics_) {
        chrc->export_object();
    }

    //exported_->emitInterfacesAddedSignal();
}

void GattService::unexport() {
    // Unexport every characteristic first, then release its parent service
    // object to preserve the D-Bus hierarchy throughout removal.
    for (auto& chrc : characteristics_) {
        chrc->unexport();
    }
    /*if (exported_) {
        exported_->emitInterfacesRemovedSignal();
    }*/
    exported_.reset();
}

void GattService::add_characteristic(std::shared_ptr<GattCharacteristic> chrc) {
    // Transfer this characteristic into the service subtree exported to BlueZ.
    characteristics_.push_back(std::move(chrc));
}

/*GattService::ManagedObjects GattService::get_managed_objects() const {
    ManagedObjects result;
    // Service properties.
    std::map<std::string, sdbus::Variant> svc_props;
    svc_props["UUID"] = sdbus::Variant{uuid_};
    svc_props["Primary"] = sdbus::Variant{primary_};
    std::vector<sdbus::ObjectPath> chrc_paths;
    for (const auto& c : characteristics_) {
        chrc_paths.push_back(sdbus::ObjectPath{c->path()});
    }
    svc_props["Characteristics"] = sdbus::Variant{chrc_paths};
    result[sdbus::ObjectPath{path_}][std::string(kGattServiceIface)] = svc_props;

    for (const auto& chrc : characteristics_) {
        std::map<std::string, sdbus::Variant> chrc_props;
        chrc_props["Service"] = sdbus::Variant{sdbus::ObjectPath{path_}};
        chrc_props["UUID"] = sdbus::Variant{chrc->uuid()};
        chrc_props["Flags"] = sdbus::Variant{chrc->flags_};
        chrc_props["Notifying"] = sdbus::Variant{chrc->notifying()};
        {
            std::lock_guard<std::mutex> lock(chrc->mutex_);
            chrc_props["Value"] = sdbus::Variant{chrc->value_};
        }
        std::vector<sdbus::ObjectPath> desc_paths;
        for (const auto& d : chrc->descriptors()) {
            desc_paths.push_back(sdbus::ObjectPath{d->path()});
        }
        chrc_props["Descriptors"] = sdbus::Variant{desc_paths};
        result[sdbus::ObjectPath{chrc->path()}][std::string(kGattCharacteristicIface)] = chrc_props;

        for (const auto& desc : chrc->descriptors()) {
            std::map<std::string, sdbus::Variant> desc_props;
            desc_props["Characteristic"] = sdbus::Variant{sdbus::ObjectPath{chrc->path()}};
            desc_props["UUID"] = sdbus::Variant{desc->uuid()};
            desc_props["Flags"] = sdbus::Variant{desc->flags_};
            {
                std::lock_guard<std::mutex> lock(desc->mutex_);
                if (!desc->value_.empty()) {
                    desc_props["Value"] = sdbus::Variant{desc->value_};
                }
            }
            result[sdbus::ObjectPath{desc->path()}][std::string(kGattDescriptorIface)] = desc_props;
        }
    }
    return result;
}*/

// ===========================================================================
// GattApplication
// ===========================================================================

GattApplication::GattApplication(DbusConnection& dbus,
                                 const std::string& app_path,
                                 rclcpp::Logger logger)
    : dbus_(dbus), path_(app_path), logger_(logger) {
        // Retain the D-Bus connection and root path used for the later BlueZ registration.
    }

GattApplication::~GattApplication() {
    // Unregister the complete application while its adapter and D-Bus connection still exist.
    if (registered_ && !adapter_path_.empty()) {
        try {
            unregister_application(adapter_path_);
        } catch (...) {}
    }
    exported_.reset();
}

void GattApplication::add_service(std::shared_ptr<GattService> svc) {
    // Transfer this service into the application object tree registered with BlueZ.
    services_.push_back(std::move(svc));
}

void GattApplication::add_profile(std::shared_ptr<GattProfile> profile) {
    // Transfer this client profile into the application registration lifecycle.
    profiles_.push_back(std::move(profile));
}

void GattApplication::register_application(const std::string& adapter_path) {
    // Create the application-level D-Bus object.
    RCLCPP_INFO(logger_, "Creating GATT application object at %s", path_.c_str());
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});

    // Export services FIRST — each service recursively exports its
    // characteristics and descriptors.  They must exist on the bus BEFORE
    // addObjectManager() because BlueZ will call GetManagedObjects
    // immediately (or during RegisterApplication) and the automatic
    // sd-bus ObjectManager enumerates objects already registered on the
    // same connection under the ObjectManager path.
    for (auto& svc : services_) {
        RCLCPP_INFO(logger_, "Exporting GATT service at %s (uuid=%s, %zu characteristics)",
                    svc->path().c_str(), svc->uuid().c_str(), svc->characteristics().size());
        svc->export_object();
        for (auto& chrc : svc->characteristics()) {
            RCLCPP_INFO(logger_, "  Characteristic %s uuid=%s flags=[%s] %zu descriptors",
                        chrc->path().c_str(), chrc->uuid().c_str(),
                        [&]() {
                            // Convert the current value for the enclosing callback.
                            std::string f;
                            for (const auto& fl : chrc->flags()) {
                                if (!f.empty()) f += ",";
                                f += fl;
                            }
                            return f;
                        }().c_str(),
                        chrc->descriptors().size());
            for (auto& desc : chrc->descriptors()) {
                RCLCPP_INFO(logger_, "    Descriptor %s uuid=%s",
                            desc->path().c_str(), desc->uuid().c_str());
            }
        }
    }
    for (auto& profile : profiles_) {
        RCLCPP_INFO(logger_, "Exporting GATT client profile at %s (%zu UUIDs)",
                    profile->path().c_str(), profile->uuids().size());
        profile->export_object();
    }

    // Now enable the automatic ObjectManager on the application object.
    // sd-bus will respond to GetManagedObjects by enumerating all objects
    // below this path that carry at least one vtable — i.e. the services,
    // characteristics and descriptors we just exported.
    RCLCPP_INFO(logger_, "Adding ObjectManager at %s", path_.c_str());
    object_manager_slot_.emplace(exported_->addObjectManager(sdbus::return_slot));

    // Register with BlueZ GattManager1 using ASYNC call.
    // A synchronous call would deadlock: sdbus-c++ v2 blocks the connection
    // for the duration of a sync call, but BlueZ calls GetManagedObjects back
    // on the same connection before replying — deadlock.
    RCLCPP_INFO(logger_, "Calling RegisterApplication (async) on %s", adapter_path.c_str());
    auto proxy = sdbus::createProxy(dbus_.connection(),
                                    sdbus::ServiceName{std::string(kBluezServiceName)},
                                    sdbus::ObjectPath{adapter_path});
    std::map<std::string, sdbus::Variant> options;
    auto future = proxy->callMethodAsync("RegisterApplication")
        .onInterface(std::string(kGattManagerIface))
        .withArguments(sdbus::ObjectPath{path_}, options)
        .getResultAsFuture();

    // Wait for the async result with a generous timeout.
    const auto status = future.wait_for(std::chrono::seconds(30));
    if (status == std::future_status::timeout) {
        RCLCPP_ERROR(logger_, "RegisterApplication timed out after 30 s");
        throw sdbus::Error(sdbus::Error::Name{"org.freedesktop.DBus.Error.Timeout"},
                           "RegisterApplication timed out");
    }
    // .get() will re-throw any sdbus::Error from BlueZ.
    future.get();

    registered_ = true;
    adapter_path_ = adapter_path;
    RCLCPP_INFO(logger_, "GATT application registered successfully at %s", path_.c_str());
}

void GattApplication::unregister_application(const std::string& adapter_path) {
    // Unregister application.
    if (!registered_) return;
    RCLCPP_INFO(logger_, "Unregistering GATT application from %s", adapter_path.c_str());
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path});
        proxy->callMethod("UnregisterApplication")
            .onInterface(std::string(kGattManagerIface))
            .withArguments(sdbus::ObjectPath{path_});
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "Failed to unregister GATT app: %s", e.getMessage().c_str());
    }
    for (auto& svc : services_) {
        svc->unexport();
    }
    for (auto& profile : profiles_) {
        profile->unexport();
    }
    object_manager_slot_.reset();
    exported_.reset();
    registered_ = false;
    RCLCPP_INFO(logger_, "GATT application unregistered");
}

}  // namespace mrs_uav_bluetooth::gatt
