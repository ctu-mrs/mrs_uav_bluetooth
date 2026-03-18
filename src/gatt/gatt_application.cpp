// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"

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

    // Properties interface – GetAll.
    exported_->addVTable(
        sdbus::registerMethod("GetAll")
            .onInterface(std::string(kDbusPropertiesIface))
            .withInputParamNames("interface")
            .withOutputParamNames("properties")
            .implementedAs([this](const std::string& iface)
                -> std::map<std::string, sdbus::Variant> {
                if (iface != std::string(kGattDescriptorIface)) {
                    throw sdbus::Error(sdbus::Error::Name{"org.freedesktop.DBus.Error.InvalidArgs"},
                                       "Invalid interface");
                }
                std::map<std::string, sdbus::Variant> props;
                props["Characteristic"] = sdbus::Variant{sdbus::ObjectPath{parent_.path()}};
                props["UUID"] = sdbus::Variant{uuid_};
                props["Flags"] = sdbus::Variant{flags_};
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!value_.empty()) {
                        props["Value"] = sdbus::Variant{value_};
                    }
                }
                return props;
            })
    ).forInterface(std::string(kDbusPropertiesIface));

    // GattDescriptor1 interface.
    exported_->addVTable(
        sdbus::registerMethod("ReadValue")
            .onInterface(std::string(kGattDescriptorIface))
            .withInputParamNames("options")
            .withOutputParamNames("value")
            .implementedAs([this](const std::map<std::string, sdbus::Variant>& opts)
                -> std::vector<uint8_t> {
                return on_read(opts);
            }),
        sdbus::registerMethod("WriteValue")
            .onInterface(std::string(kGattDescriptorIface))
            .withInputParamNames("value", "options")
            .implementedAs([this](const std::vector<uint8_t>& data,
                                  const std::map<std::string, sdbus::Variant>& opts) {
                on_write(data, opts);
            }),
        sdbus::registerSignal("PropertiesChanged")
            .onInterface(std::string(kDbusPropertiesIface))
            .withParameters<std::string,
                           std::map<std::string, sdbus::Variant>,
                           std::vector<std::string>>()
    ).forInterface(std::string(kGattDescriptorIface));
}

void GattDescriptor::unexport() {
    exported_.reset();
}

void GattDescriptor::set_value(const std::vector<uint8_t>& val, bool emit) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = val;
    }
    if (emit && exported_) {
        std::map<std::string, sdbus::Variant> changed;
        changed["Value"] = sdbus::Variant{val};
        exported_->emitSignal("PropertiesChanged")
            .onInterface(std::string(kDbusPropertiesIface))
            .withArguments(std::string{kGattDescriptorIface}, changed,
                           std::vector<std::string>{});
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

    // Properties interface.
    exported_->addVTable(
        sdbus::registerMethod("GetAll")
            .onInterface(std::string(kDbusPropertiesIface))
            .withInputParamNames("interface")
            .withOutputParamNames("properties")
            .implementedAs([this](const std::string& iface)
                -> std::map<std::string, sdbus::Variant> {
                if (iface != std::string(kGattCharacteristicIface)) {
                    throw sdbus::Error(sdbus::Error::Name{"org.freedesktop.DBus.Error.InvalidArgs"},
                                       "Invalid interface");
                }
                std::map<std::string, sdbus::Variant> props;
                props["Service"] = sdbus::Variant{sdbus::ObjectPath{parent_.path()}};
                props["UUID"] = sdbus::Variant{uuid_};
                props["Flags"] = sdbus::Variant{flags_};
                props["Notifying"] = sdbus::Variant{notifying_};
                // Descriptors.
                std::vector<sdbus::ObjectPath> desc_paths;
                for (const auto& d : descriptors_) {
                    desc_paths.push_back(sdbus::ObjectPath{d->path()});
                }
                props["Descriptors"] = sdbus::Variant{desc_paths};
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    props["Value"] = sdbus::Variant{value_};
                }
                return props;
            })
    ).forInterface(std::string(kDbusPropertiesIface));

    // GattCharacteristic1 interface.
    exported_->addVTable(
        sdbus::registerMethod("ReadValue")
            .onInterface(std::string(kGattCharacteristicIface))
            .withInputParamNames("options")
            .withOutputParamNames("value")
            .implementedAs([this](const std::map<std::string, sdbus::Variant>& opts)
                -> std::vector<uint8_t> {
                return on_read(opts);
            }),
        sdbus::registerMethod("WriteValue")
            .onInterface(std::string(kGattCharacteristicIface))
            .withInputParamNames("value", "options")
            .implementedAs([this](const std::vector<uint8_t>& data,
                                  const std::map<std::string, sdbus::Variant>& opts) {
                on_write(data, opts);
            }),
        sdbus::registerMethod("StartNotify")
            .onInterface(std::string(kGattCharacteristicIface))
            .implementedAs([this]() { on_start_notify(); }),
        sdbus::registerMethod("StopNotify")
            .onInterface(std::string(kGattCharacteristicIface))
            .implementedAs([this]() { on_stop_notify(); }),
        sdbus::registerSignal("PropertiesChanged")
            .onInterface(std::string(kDbusPropertiesIface))
            .withParameters<std::string,
                           std::map<std::string, sdbus::Variant>,
                           std::vector<std::string>>()
    ).forInterface(std::string(kGattCharacteristicIface));

    // Export child descriptors.
    for (auto& desc : descriptors_) {
        desc->export_object();
    }
}

void GattCharacteristic::unexport() {
    for (auto& desc : descriptors_) {
        desc->unexport();
    }
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
        std::map<std::string, sdbus::Variant> changed;
        changed["Value"] = sdbus::Variant{val};
        exported_->emitSignal("PropertiesChanged")
            .onInterface(std::string(kDbusPropertiesIface))
            .withArguments(std::string{kGattCharacteristicIface}, changed,
                           std::vector<std::string>{});
    }
}

std::vector<uint8_t> GattCharacteristic::value() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}

void GattCharacteristic::publish(const std::vector<uint8_t>& data) {
    set_value(data, notifying_);
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
    if (exported_) {
        std::map<std::string, sdbus::Variant> changed;
        changed["Notifying"] = sdbus::Variant{true};
        exported_->emitSignal("PropertiesChanged")
            .onInterface(std::string(kDbusPropertiesIface))
            .withArguments(std::string{kGattCharacteristicIface}, changed,
                           std::vector<std::string>{});
    }
    if (notify_cb_) notify_cb_(true);
}

void GattCharacteristic::on_stop_notify() {
    if (!notifying_) return;
    notifying_ = false;
    if (exported_) {
        std::map<std::string, sdbus::Variant> changed;
        changed["Notifying"] = sdbus::Variant{false};
        exported_->emitSignal("PropertiesChanged")
            .onInterface(std::string(kDbusPropertiesIface))
            .withArguments(std::string{kGattCharacteristicIface}, changed,
                           std::vector<std::string>{});
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

    exported_->addVTable(
        sdbus::registerMethod("GetAll")
            .onInterface(std::string(kDbusPropertiesIface))
            .withInputParamNames("interface")
            .withOutputParamNames("properties")
            .implementedAs([this](const std::string& iface)
                -> std::map<std::string, sdbus::Variant> {
                if (iface != std::string(kGattServiceIface)) {
                    throw sdbus::Error(sdbus::Error::Name{"org.freedesktop.DBus.Error.InvalidArgs"},
                                       "Invalid interface");
                }
                std::map<std::string, sdbus::Variant> props;
                props["UUID"] = sdbus::Variant{uuid_};
                props["Primary"] = sdbus::Variant{primary_};
                std::vector<sdbus::ObjectPath> chrc_paths;
                for (const auto& c : characteristics_) {
                    chrc_paths.push_back(sdbus::ObjectPath{c->path()});
                }
                props["Characteristics"] = sdbus::Variant{chrc_paths};
                return props;
            })
    ).forInterface(std::string(kDbusPropertiesIface));

    for (auto& chrc : characteristics_) {
        chrc->export_object();
    }
}

void GattService::unexport() {
    for (auto& chrc : characteristics_) {
        chrc->unexport();
    }
    exported_.reset();
}

void GattService::add_characteristic(std::shared_ptr<GattCharacteristic> chrc) {
    characteristics_.push_back(std::move(chrc));
}

GattService::ManagedObjects GattService::get_managed_objects() const {
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
}

// ===========================================================================
// GattApplication
// ===========================================================================

GattApplication::GattApplication(DbusConnection& dbus,
                                 const std::string& app_path,
                                 rclcpp::Logger logger)
    : dbus_(dbus), path_(app_path), logger_(logger) {}

GattApplication::~GattApplication() {
    if (registered_) {
        try {
            // Best effort unregister — adapter_path is not stored, so skip.
        } catch (...) {}
    }
    exported_.reset();
}

void GattApplication::add_service(std::shared_ptr<GattService> svc) {
    services_.push_back(std::move(svc));
}

void GattApplication::register_application(const std::string& adapter_path) {
    // Export services first.
    for (auto& svc : services_) {
        svc->export_object();
    }

    // Export the ObjectManager at the application path.
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});

    exported_->addVTable(
        sdbus::registerMethod("GetManagedObjects")
            .onInterface(std::string(kDbusObjectManagerIface))
            .withOutputParamNames("objects")
            .implementedAs([this]() {
                using MO = std::map<sdbus::ObjectPath,
                    std::map<std::string, std::map<std::string, sdbus::Variant>>>;
                MO result;
                for (const auto& svc : services_) {
                    auto mo = svc->get_managed_objects();
                    result.insert(mo.begin(), mo.end());
                }
                return result;
            })
    ).forInterface(std::string(kDbusObjectManagerIface));

    // Register with BlueZ GattManager1.
    auto proxy = sdbus::createProxy(dbus_.connection(),
                                    sdbus::ServiceName{std::string(kBluezServiceName)},
                                    sdbus::ObjectPath{adapter_path});
    std::map<std::string, sdbus::Variant> options;
    proxy->callMethod("RegisterApplication")
        .onInterface(std::string(kGattManagerIface))
        .withArguments(sdbus::ObjectPath{path_}, options);

    registered_ = true;
    RCLCPP_INFO(logger_, "GATT application registered at %s", path_.c_str());
}

void GattApplication::unregister_application(const std::string& adapter_path) {
    if (!registered_) return;
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
    exported_.reset();
    registered_ = false;
}

}  // namespace mrs_uav_bluetooth::gatt
