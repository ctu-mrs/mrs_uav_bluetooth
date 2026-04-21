// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/advertisement.hpp"

#include <future>

namespace mrs_uav_bluetooth::gatt {

using namespace mrs_uav_bluetooth::bluez;

Advertisement::Advertisement(DbusConnection& dbus,
                             const std::string& object_path,
                             const std::string& ad_type)
    : dbus_(dbus), path_(object_path), ad_type_(ad_type) {}

namespace {

template<typename KeyT>
std::map<KeyT, sdbus::Variant> to_variant_map(const std::map<KeyT, std::vector<uint8_t>>& data) {
    std::map<KeyT, sdbus::Variant> out;
    for (const auto& [key, value] : data) {
        out.emplace(key, sdbus::Variant{value});
    }
    return out;
}

}  // namespace

Advertisement::~Advertisement() {
    unexport();
}

void Advertisement::export_object() {
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});

    // Register BlueZ-expected properties + methods on the LEAdvertisement1
    // interface.  sdbus-c++ automatically provides org.freedesktop.DBus.Properties.
    exported_->addVTable(
        sdbus::registerProperty("Type")
            .withGetter([this]() -> std::string { return ad_type_; }),
        sdbus::registerProperty("ServiceUUIDs")
            .withGetter([this]() -> std::vector<std::string> { return service_uuids_; }),
        sdbus::registerProperty("ServiceData")
            .withGetter([this]() -> std::map<std::string, sdbus::Variant> { return service_data_; }),
        sdbus::registerProperty("ManufacturerData")
            .withGetter([this]() -> std::map<uint16_t, sdbus::Variant> { return manufacturer_data_; }),
        sdbus::registerProperty("Data")
            .withGetter([this]() -> std::map<uint8_t, sdbus::Variant> { return data_; }),
        sdbus::registerProperty("SolicitUUIDs")
            .withGetter([this]() -> std::vector<std::string> { return solicit_uuids_; }),
        sdbus::registerProperty("LocalName")
            .withGetter([this]() -> std::string { return local_name_; }),
        sdbus::registerProperty("Discoverable")
            .withGetter([this]() -> bool { return discoverable_; }),
        sdbus::registerProperty("DiscoverableTimeout")
            .withGetter([this]() -> uint16_t { return discoverable_timeout_; }),
        sdbus::registerProperty("Includes")
            .withGetter([this]() -> std::vector<std::string> { return includes_; }),
        sdbus::registerProperty("ScanResponseServiceUUIDs")
            .withGetter([this]() -> std::vector<std::string> { return scan_response_service_uuids_; }),
        sdbus::registerProperty("ScanResponseManufacturerData")
            .withGetter([this]() -> std::map<uint16_t, sdbus::Variant> {
                return scan_response_manufacturer_data_;
            }),
        sdbus::registerProperty("ScanResponseSolicitUUIDs")
            .withGetter([this]() -> std::vector<std::string> { return scan_response_solicit_uuids_; }),
        sdbus::registerProperty("ScanResponseServiceData")
            .withGetter([this]() -> std::map<std::string, sdbus::Variant> {
                return scan_response_service_data_;
            }),
        sdbus::registerProperty("ScanResponseData")
            .withGetter([this]() -> std::map<uint8_t, sdbus::Variant> { return scan_response_data_; }),
        sdbus::registerProperty("Appearance")
            .withGetter([this]() -> uint16_t { return appearance_.value_or(0); }),
        sdbus::registerProperty("Duration")
            .withGetter([this]() -> uint16_t { return duration_.value_or(0); }),
        sdbus::registerProperty("Timeout")
            .withGetter([this]() -> uint16_t { return timeout_.value_or(0); }),
        sdbus::registerProperty("SecondaryChannel")
            .withGetter([this]() -> std::string { return secondary_channel_; }),
        sdbus::registerProperty("MinInterval")
            .withGetter([this]() -> uint32_t { return min_interval_.value_or(0); }),
        sdbus::registerProperty("MaxInterval")
            .withGetter([this]() -> uint32_t { return max_interval_.value_or(0); }),
        sdbus::registerProperty("TxPower")
            .withGetter([this]() -> int16_t { return tx_power_.value_or(0); }),
        sdbus::registerMethod("Release")
            .implementedAs([]() {})
    ).forInterface(std::string(kLeAdvertisementIface));
}

void Advertisement::unexport() {
    exported_.reset();
}

void Advertisement::register_advertisement(const std::string& adapter_path) {
    if (exported_) {
        unexport();
    }
    export_object();
    auto proxy = sdbus::createProxy(dbus_.connection(),
                                    sdbus::ServiceName{std::string(kBluezServiceName)},
                                    sdbus::ObjectPath{adapter_path});
    std::map<std::string, sdbus::Variant> options;
    auto future = proxy->callMethodAsync("RegisterAdvertisement")
        .onInterface(std::string(kLeAdvManagerIface))
        .withArguments(sdbus::ObjectPath{path_}, options)
        .getResultAsFuture();

    const auto status = future.wait_for(std::chrono::seconds(30));
    if (status == std::future_status::timeout) {
        throw sdbus::Error(sdbus::Error::Name{"org.freedesktop.DBus.Error.Timeout"},
                           "RegisterAdvertisement timed out");
    }

    future.get();
    registered_ = true;
}

void Advertisement::set_manufacturer_data(const std::map<uint16_t, std::vector<uint8_t>>& data) {
    manufacturer_data_ = to_variant_map(data);
}

void Advertisement::set_service_data(const std::map<std::string, std::vector<uint8_t>>& data) {
    service_data_ = to_variant_map(data);
}

void Advertisement::set_data(const std::map<uint8_t, std::vector<uint8_t>>& data) {
    data_ = to_variant_map(data);
}

void Advertisement::set_scan_response_manufacturer_data(
    const std::map<uint16_t, std::vector<uint8_t>>& data) {
    scan_response_manufacturer_data_ = to_variant_map(data);
}

void Advertisement::set_scan_response_service_data(
    const std::map<std::string, std::vector<uint8_t>>& data) {
    scan_response_service_data_ = to_variant_map(data);
}

void Advertisement::set_scan_response_data(const std::map<uint8_t, std::vector<uint8_t>>& data) {
    scan_response_data_ = to_variant_map(data);
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

}  // namespace mrs_uav_bluetooth::gatt
