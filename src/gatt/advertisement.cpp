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
    const auto interface_name = std::string(kLeAdvertisementIface);

    // BlueZ treats several optional LEAdvertisement1 properties as invalid
    // when they are present with empty or zero defaults, so omit them unless
    // the caller configured a concrete value.
    exported_->addVTable(
        sdbus::registerProperty("Type")
            .withGetter([this]() -> std::string { return ad_type_; }),
        sdbus::registerMethod("Release")
            .implementedAs([]() {})
    ).forInterface(interface_name);

    if (!service_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ServiceUUIDs")
                .withGetter([this]() -> std::vector<std::string> { return service_uuids_; })
        ).forInterface(interface_name);
    }
    if (!service_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ServiceData")
                .withGetter([this]() -> std::map<std::string, sdbus::Variant> { return service_data_; })
        ).forInterface(interface_name);
    }
    if (!manufacturer_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ManufacturerData")
                .withGetter([this]() -> std::map<uint16_t, sdbus::Variant> { return manufacturer_data_; })
        ).forInterface(interface_name);
    }
    if (!data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("Data")
                .withGetter([this]() -> std::map<uint8_t, sdbus::Variant> { return data_; })
        ).forInterface(interface_name);
    }
    if (!solicit_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("SolicitUUIDs")
                .withGetter([this]() -> std::vector<std::string> { return solicit_uuids_; })
        ).forInterface(interface_name);
    }
    if (!local_name_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("LocalName")
                .withGetter([this]() -> std::string { return local_name_; })
        ).forInterface(interface_name);
    }

    exported_->addVTable(
        sdbus::registerProperty("Discoverable")
            .withGetter([this]() -> bool { return discoverable_; })
    ).forInterface(interface_name);

    if (discoverable_timeout_ != 0) {
        exported_->addVTable(
            sdbus::registerProperty("DiscoverableTimeout")
                .withGetter([this]() -> uint16_t { return discoverable_timeout_; })
        ).forInterface(interface_name);
    }
    if (!includes_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("Includes")
                .withGetter([this]() -> std::vector<std::string> { return includes_; })
        ).forInterface(interface_name);
    }
    if (!scan_response_service_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseServiceUUIDs")
                .withGetter([this]() -> std::vector<std::string> { return scan_response_service_uuids_; })
        ).forInterface(interface_name);
    }
    if (!scan_response_manufacturer_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseManufacturerData")
                .withGetter([this]() -> std::map<uint16_t, sdbus::Variant> {
                    return scan_response_manufacturer_data_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_solicit_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseSolicitUUIDs")
                .withGetter([this]() -> std::vector<std::string> { return scan_response_solicit_uuids_; })
        ).forInterface(interface_name);
    }
    if (!scan_response_service_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseServiceData")
                .withGetter([this]() -> std::map<std::string, sdbus::Variant> {
                    return scan_response_service_data_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseData")
                .withGetter([this]() -> std::map<uint8_t, sdbus::Variant> { return scan_response_data_; })
        ).forInterface(interface_name);
    }
    if (appearance_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("Appearance")
                .withGetter([this]() -> uint16_t { return *appearance_; })
        ).forInterface(interface_name);
    }
    if (duration_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("Duration")
                .withGetter([this]() -> uint16_t { return *duration_; })
        ).forInterface(interface_name);
    }
    if (timeout_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("Timeout")
                .withGetter([this]() -> uint16_t { return *timeout_; })
        ).forInterface(interface_name);
    }
    if (!secondary_channel_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("SecondaryChannel")
                .withGetter([this]() -> std::string { return secondary_channel_; })
        ).forInterface(interface_name);
    }
    if (min_interval_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("MinInterval")
                .withGetter([this]() -> uint32_t { return *min_interval_; })
        ).forInterface(interface_name);
    }
    if (max_interval_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("MaxInterval")
                .withGetter([this]() -> uint32_t { return *max_interval_; })
        ).forInterface(interface_name);
    }
    if (tx_power_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("TxPower")
                .withGetter([this]() -> int16_t { return *tx_power_; })
        ).forInterface(interface_name);
    }
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
