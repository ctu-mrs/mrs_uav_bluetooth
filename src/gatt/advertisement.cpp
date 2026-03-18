// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/advertisement.hpp"

namespace mrs_uav_bluetooth::gatt {

using namespace mrs_uav_bluetooth::bluez;

Advertisement::Advertisement(DbusConnection& dbus,
                             const std::string& object_path,
                             const std::string& ad_type)
    : dbus_(dbus), path_(object_path), ad_type_(ad_type) {}

Advertisement::~Advertisement() {
    unexport();
}

void Advertisement::export_object() {
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});

    exported_->addVTable(
        sdbus::registerMethod("GetAll")
            .withInputParamNames("interface")
            .withOutputParamNames("properties")
            .implementedAs([this](const std::string& iface)
                -> std::map<std::string, sdbus::Variant> {
                if (iface != std::string(kLeAdvertisementIface)) {
                    throw sdbus::Error(
                        sdbus::Error::Name{"org.freedesktop.DBus.Error.InvalidArgs"},
                        "Invalid interface");
                }
                return get_properties();
            })
    ).forInterface(std::string(kDbusPropertiesIface));

    exported_->addVTable(
        sdbus::registerMethod("Release")
            .implementedAs([]() {})
    ).forInterface(std::string(kLeAdvertisementIface));
}

void Advertisement::unexport() {
    exported_.reset();
}

void Advertisement::register_advertisement(const std::string& adapter_path) {
    if (!exported_) {
        export_object();
    }
    auto proxy = sdbus::createProxy(dbus_.connection(),
                                    sdbus::ServiceName{std::string(kBluezServiceName)},
                                    sdbus::ObjectPath{adapter_path});
    std::map<std::string, sdbus::Variant> options;
    proxy->callMethod("RegisterAdvertisement")
        .onInterface(std::string(kLeAdvManagerIface))
        .withArguments(sdbus::ObjectPath{path_}, options);
    registered_ = true;
}

void Advertisement::unregister_advertisement(const std::string& adapter_path) {
    if (!registered_) return;
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path});
        proxy->callMethod("UnregisterAdvertisement")
            .onInterface(std::string(kLeAdvManagerIface))
            .withArguments(sdbus::ObjectPath{path_});
    } catch (const sdbus::Error&) {
        // Best effort.
    }
    unexport();
    registered_ = false;
}

std::map<std::string, sdbus::Variant> Advertisement::get_properties() const {
    std::map<std::string, sdbus::Variant> props;
    props["Type"] = sdbus::Variant{ad_type_};
    if (!service_uuids_.empty()) {
        props["ServiceUUIDs"] = sdbus::Variant{service_uuids_};
    }
    if (!local_name_.empty()) {
        props["LocalName"] = sdbus::Variant{local_name_};
    }
    props["Discoverable"] = sdbus::Variant{discoverable_};
    if (discoverable_timeout_ > 0) {
        props["DiscoverableTimeout"] = sdbus::Variant{discoverable_timeout_};
    }
    if (!includes_.empty()) {
        props["Includes"] = sdbus::Variant{includes_};
    }
    if (has_tx_power_) {
        props["TxPower"] = sdbus::Variant{tx_power_};
    }
    return props;
}

}  // namespace mrs_uav_bluetooth::gatt
