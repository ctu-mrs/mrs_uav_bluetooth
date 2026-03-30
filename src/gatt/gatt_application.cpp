// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"

#include <future>

#include <rclcpp/rclcpp.hpp>

namespace mrs_uav_bluetooth::gatt {

using namespace mrs_uav_bluetooth::bluez;

// ===========================================================================
// GattDescriptor
// ===========================================================================

GattDescriptor::GattDescriptor(DbusConnection& dbus,
                               const std::string& object_path,
                               const std::string& uuid,
                               const std::vector<std::string>& flags,
                               GattCharacteristic& parent)
    : dbus_(dbus), path_(object_path), uuid_(uuid), flags_(flags), parent_(parent) {}

GattDescriptor::~GattDescriptor() { unexport(); }

void GattDescriptor::export_object() {
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});

    // Register BlueZ-expected properties + methods on the GattDescriptor1 interface.
    // sdbus-c++ automatically provides org.freedesktop.DBus.Properties (GetAll,
    // Get, Set, PropertiesChanged) for registered properties — do NOT register
    // that interface manually, or BlueZ will reject the vtable.
    exported_->addVTable(
        sdbus::registerProperty("UUID")
            .withGetter([this]() -> std::string { return uuid_; }),
        sdbus::registerProperty("Characteristic")
            .withGetter([this]() -> sdbus::ObjectPath { return sdbus::ObjectPath{parent_.path()}; }),
        sdbus::registerProperty("Handle")
            .withGetter([this]() -> uint16_t { return handle_; }),
        sdbus::registerProperty("Flags")
            .withGetter([this]() -> std::vector<std::string> { 
                return flags_.empty() ? std::vector<std::string>{"read"} : flags_; 
            }),
        sdbus::registerProperty("Value")
            .withGetter([this]() -> std::vector<uint8_t> {
                std::lock_guard<std::mutex> lock(mutex_);
                return value_;
            }),
        sdbus::registerMethod("ReadValue")
            .withInputParamNames("options")
            .withOutputParamNames("value")
            .implementedAs([this](const std::map<std::string, sdbus::Variant>& opts)
                -> std::vector<uint8_t> {
                return on_read(opts);
            }),
        sdbus::registerMethod("WriteValue")
            .withInputParamNames("value", "options")
            .implementedAs([this](const std::vector<uint8_t>& data,
                                  const std::map<std::string, sdbus::Variant>& opts) {
                on_write(data, opts);
            })
    ).forInterface(std::string(kGattDescriptorIface));

    //exported_->emitInterfacesAddedSignal();
}

void GattDescriptor::unexport() {
    /*if (exported_) {
        exported_->emitInterfacesRemovedSignal();
    }*/
    exported_.reset();
}

void GattDescriptor::set_value(const std::vector<uint8_t>& val, bool emit) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = val;
    }
    if (emit && exported_) {
        exported_->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattDescriptorIface)},
                                              std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Value"}});
    }
}

std::vector<uint8_t> GattDescriptor::value() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

std::vector<uint8_t> GattDescriptor::on_read(
    const std::map<std::string, sdbus::Variant>& /*options*/) {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

void GattDescriptor::on_write(const std::vector<uint8_t>& data,
                               const std::map<std::string, sdbus::Variant>& /*options*/) {
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
    : dbus_(dbus), path_(object_path), uuid_(uuid), flags_(flags), parent_(parent) {}

GattCharacteristic::~GattCharacteristic() { unexport(); }

void GattCharacteristic::export_object() {
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});

    // Register BlueZ-expected properties + methods on the GattCharacteristic1
    // interface.  sdbus-c++ automatically provides org.freedesktop.DBus.Properties.
    exported_->addVTable(
        sdbus::registerProperty("UUID")
            .withGetter([this]() -> std::string { return uuid_; }),
        sdbus::registerProperty("Service")
            .withGetter([this]() -> sdbus::ObjectPath { return sdbus::ObjectPath{parent_.path()}; }),
        sdbus::registerProperty("Descriptors")
            .withGetter([this]() -> std::vector<sdbus::ObjectPath> { 
                std::vector<sdbus::ObjectPath> desc_paths;
                for (const auto& d : descriptors_) {
                    desc_paths.push_back(sdbus::ObjectPath{d->path()});
                }
                return desc_paths; 
            }),
        sdbus::registerProperty("Handle")
            .withGetter([this]() -> uint16_t { return handle_; }),
        sdbus::registerProperty("Flags")
            .withGetter([this]() -> std::vector<std::string> { return flags_; }),
        sdbus::registerProperty("Notifying")
            .withGetter([this]() -> bool { return notifying_; }),
        sdbus::registerProperty("Value")
            .withGetter([this]() -> std::vector<uint8_t> {
                std::lock_guard<std::mutex> lock(mutex_);
                return value_;
            }),
        sdbus::registerMethod("ReadValue")
            .withInputParamNames("options")
            .withOutputParamNames("value")
            .implementedAs([this](const std::map<std::string, sdbus::Variant>& opts)
                -> std::vector<uint8_t> {
                return on_read(opts);
            }),
        sdbus::registerMethod("WriteValue")
            .withInputParamNames("value", "options")
            .implementedAs([this](const std::vector<uint8_t>& data,
                                  const std::map<std::string, sdbus::Variant>& opts) {
                on_write(data, opts);
            }),
        sdbus::registerMethod("StartNotify")
            .implementedAs([this]() { on_start_notify(); }),
        sdbus::registerMethod("StopNotify")
            .implementedAs([this]() { on_stop_notify(); })
    ).forInterface(std::string(kGattCharacteristicIface));

    for (auto& desc : descriptors_) {
        desc->export_object();
    }

    //exported_->emitInterfacesAddedSignal();
}

void GattCharacteristic::unexport() {
    for (auto& desc : descriptors_) {
        desc->unexport();
    }
    /*if (exported_) {
        exported_->emitInterfacesRemovedSignal();
    }*/
    exported_.reset();
}

void GattCharacteristic::add_descriptor(std::shared_ptr<GattDescriptor> desc) {
    descriptors_.push_back(std::move(desc));
}

void GattCharacteristic::set_value(const std::vector<uint8_t>& val, bool emit) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = val;
    }
    if (emit && exported_) {
        static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
        RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("mrs_uav_bluetooth"), throttle_clock, 5000,
                              "[server] value changed path=%s bytes=%zu",
                              path_.c_str(), val.size());
        exported_->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattCharacteristicIface)},
                                              std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Value"}});
    }
}

std::vector<uint8_t> GattCharacteristic::value() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

void GattCharacteristic::publish(const std::vector<uint8_t>& data) {
    set_value(data, notifying_ || force_emit_value_);
}

std::vector<uint8_t> GattCharacteristic::on_read(
    const std::map<std::string, sdbus::Variant>& /*options*/) {
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
    set_value(data, true);
    if (write_cb_) {
        write_cb_(data);
    }
}

void GattCharacteristic::on_start_notify() {
    if (notifying_) return;
    notifying_ = true;
    RCLCPP_INFO(rclcpp::get_logger("mrs_uav_bluetooth"),
                "[server] notify enabled path=%s", path_.c_str());
    if (exported_) {
        exported_->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattCharacteristicIface)},
                                              std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Notifying"}});
    }
    if (notify_cb_) notify_cb_(true);
}

void GattCharacteristic::on_stop_notify() {
    if (!notifying_) return;
    notifying_ = false;
    RCLCPP_INFO(rclcpp::get_logger("mrs_uav_bluetooth"),
                "[server] notify disabled path=%s", path_.c_str());
    if (exported_) {
        exported_->emitPropertiesChangedSignal(sdbus::InterfaceName{std::string(kGattCharacteristicIface)},
                                              std::vector<sdbus::PropertyName>{sdbus::PropertyName{"Notifying"}});
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
    : dbus_(dbus), path_(object_path), uuid_(uuid), primary_(primary) {}

GattService::~GattService() { unexport(); }

void GattService::export_object() {
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});


    // Register BlueZ-expected properties on the GattService1 interface.
    // sdbus-c++ automatically provides org.freedesktop.DBus.Properties.
    exported_->addVTable(
        sdbus::registerProperty("UUID")
            .withGetter([this]() -> std::string { return uuid_; }),
        sdbus::registerProperty("Primary")
            .withGetter([this]() -> bool { return primary_; }),
        sdbus::registerProperty("Characteristics")
            .withGetter([this]() -> std::vector<sdbus::ObjectPath> { 
                std::vector<sdbus::ObjectPath> chrc_paths;
                for (const auto& c : characteristics_) {
                    chrc_paths.push_back(sdbus::ObjectPath{c->path()});
                }
                return chrc_paths; 
            }),
        sdbus::registerProperty("Handle")
            .withGetter([this]() -> uint16_t { return handle_; }),
        sdbus::registerProperty("Includes")
            .withGetter([]() -> std::vector<sdbus::ObjectPath> { return {}; })
    ).forInterface(std::string(kGattServiceIface));

    for (auto& chrc : characteristics_) {
        chrc->export_object();
    }

    //exported_->emitInterfacesAddedSignal();
}

void GattService::unexport() {
    for (auto& chrc : characteristics_) {
        chrc->unexport();
    }
    /*if (exported_) {
        exported_->emitInterfacesRemovedSignal();
    }*/
    exported_.reset();
}

void GattService::add_characteristic(std::shared_ptr<GattCharacteristic> chrc) {
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
    : dbus_(dbus), path_(app_path), logger_(logger) {}

GattApplication::~GattApplication() {
    if (registered_ && !adapter_path_.empty()) {
        try {
            unregister_application(adapter_path_);
        } catch (...) {}
    }
    exported_.reset();
}

void GattApplication::add_service(std::shared_ptr<GattService> svc) {
    services_.push_back(std::move(svc));
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
    object_manager_slot_.reset();
    exported_.reset();
    registered_ = false;
    RCLCPP_INFO(logger_, "GATT application unregistered");
}

}  // namespace mrs_uav_bluetooth::gatt
