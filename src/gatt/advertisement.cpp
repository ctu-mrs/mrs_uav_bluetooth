// SPDX-License-Identifier: BSD-3-Clause
/// \file src/gatt/advertisement.cpp
/// \brief Implements the advertisement component of the Bluetooth Low Energy GATT layer.

#include "mrs_uav_bluetooth/gatt/advertisement.hpp"

#include <future>

namespace mrs_uav_bluetooth::gatt {

using namespace mrs_uav_bluetooth::bluez;

Advertisement::Advertisement(DbusConnection& dbus,
                             const std::string& object_path,
                             const std::string& ad_type)
    : dbus_(dbus), path_(object_path), ad_type_(ad_type) {
        // Retain the D-Bus path and advertisement type; property export remains deferred.
    }

namespace {

/// \brief Log a failed property signal without letting a D-Bus callback unwind.
/// \param path D-Bus object whose property notification failed.
/// \param property D-Bus property whose failed change signal is being logged.
/// \param error error details to report.
void log_properties_changed_failure(const std::string& path,
                                   const std::string& property,
                                   const sdbus::Error& error) {
    // Log a failed property signal without letting a D-Bus callback unwind.
    static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("mrs_uav_bluetooth"), throttle_clock, 5000,
                         "[server] failed to emit PropertiesChanged for advertisement %s property %s: %s",
                         path.c_str(), property.c_str(), error.what());
}

template<typename KeyT>
/// \brief Wrap each byte vector in the D-Bus variant type required by BlueZ properties.
/// \param data Advertisement byte fields to wrap as D-Bus variants.
/// \return Converted variant map.
std::map<KeyT, sdbus::Variant> to_variant_map(const std::map<KeyT, std::vector<uint8_t>>& data) {
    // Wrap each byte vector in the D-Bus variant type required by BlueZ properties.
    std::map<KeyT, sdbus::Variant> out;
    for (const auto& [key, value] : data) {
        out.emplace(key, sdbus::Variant{value});
    }
    return out;
}

}  // namespace

Advertisement::~Advertisement() {
    // Remove the advertisement object from D-Bus before releasing its connection.
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
            .withGetter([this]() -> std::string {
                // Expose whether BlueZ should send a connectable peripheral advertisement or a broadcast-only packet.
                return ad_type_;
            }),
        sdbus::registerMethod("Release")
            .implementedAs([this]() {
                // BlueZ has already removed the registration. Mark the local
                // object released and let its owner destroy it after this
                // method dispatch returns.
                released_ = true;
                registration_ = Registration::Unregistered;
                RCLCPP_WARN(rclcpp::get_logger("mrs_uav_bluetooth"),
                            "BlueZ released advertisement %s", path_.c_str());
            })
    ).forInterface(interface_name);

    if (!service_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ServiceUUIDs")
                .withGetter([this]() -> std::vector<std::string> {
                    // Publish the service identifiers that scanning peers use to recognize this application.
                    return service_uuids_;
                })
        ).forInterface(interface_name);
    }
    if (!service_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ServiceData")
                .withGetter([this]() -> std::map<std::string, sdbus::Variant> {
                    // Publish application bytes indexed by the service identifier that owns them.
                    return service_data_;
                })
        ).forInterface(interface_name);
    }
    if (!manufacturer_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ManufacturerData")
                .withGetter([this]() -> std::map<uint16_t, sdbus::Variant> {
                    // Publish vendor-specific bytes indexed by the assigned company identifier.
                    return manufacturer_data_;
                })
        ).forInterface(interface_name);
    }
    if (!data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("Data")
                .withGetter([this]() -> std::map<uint8_t, sdbus::Variant> {
                    // Return a locked snapshot because the raw advertisement fields can change while BlueZ reads them.
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    return data_;
                })
        ).forInterface(interface_name);
    }
    if (!solicit_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("SolicitUUIDs")
                .withGetter([this]() -> std::vector<std::string> {
                    // List the services this device asks nearby peers to advertise.
                    return solicit_uuids_;
                })
        ).forInterface(interface_name);
    }
    if (!local_name_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("LocalName")
                .withGetter([this]() -> std::string {
                    // Place the configured device name in the advertising packet.
                    return local_name_;
                })
        ).forInterface(interface_name);
    }

    // LEAdvertisement1 forbids Discoverable on broadcast advertisements.
    if (ad_type_ != "broadcast") {
        exported_->addVTable(
            sdbus::registerProperty("Discoverable")
                .withGetter([this]() -> bool {
                    // Tell BlueZ whether scanning peers may discover this connectable advertisement.
                    return discoverable_;
                })
        ).forInterface(interface_name);
    }

    if (ad_type_ != "broadcast" && discoverable_timeout_ != 0) {
        exported_->addVTable(
            sdbus::registerProperty("DiscoverableTimeout")
                .withGetter([this]() -> uint16_t {
                    // Limit how long BlueZ keeps this advertisement discoverable.
                    return discoverable_timeout_;
                })
        ).forInterface(interface_name);
    }
    if (!includes_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("Includes")
                .withGetter([this]() -> std::vector<std::string> {
                    // Request controller-generated fields such as transmit power or the adapter name.
                    return includes_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_service_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseServiceUUIDs")
                .withGetter([this]() -> std::vector<std::string> {
                    // Move overflow service identifiers into the scan response packet.
                    return scan_response_service_uuids_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_manufacturer_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseManufacturerData")
                .withGetter([this]() -> std::map<uint16_t, sdbus::Variant> {
                    // Put this vendor-specific payload in the scan response.
                    return scan_response_manufacturer_data_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_solicit_uuids_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseSolicitUUIDs")
                .withGetter([this]() -> std::vector<std::string> {
                    // Put the requested peer services in the scan response.
                    return scan_response_solicit_uuids_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_service_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseServiceData")
                .withGetter([this]() -> std::map<std::string, sdbus::Variant> {
                    // Put the application service payload in the scan response.
                    return scan_response_service_data_;
                })
        ).forInterface(interface_name);
    }
    if (!scan_response_data_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("ScanResponseData")
                .withGetter([this]() -> std::map<uint8_t, sdbus::Variant> {
                    // Put these raw advertising fields in the scan response.
                    return scan_response_data_;
                })
        ).forInterface(interface_name);
    }
    if (appearance_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("Appearance")
                .withGetter([this]() -> uint16_t {
                    // Expose the standard device-category number included in the advertisement.
                    return *appearance_;
                })
        ).forInterface(interface_name);
    }
    if (duration_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("Duration")
                .withGetter([this]() -> uint16_t {
                    // Tell BlueZ how long this advertisement gets the radio before rotating to another one.
                    return *duration_;
                })
        ).forInterface(interface_name);
    }
    if (timeout_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("Timeout")
                .withGetter([this]() -> uint16_t {
                    // Tell BlueZ when to stop this advertisement automatically.
                    return *timeout_;
                })
        ).forInterface(interface_name);
    }
    if (!secondary_channel_.empty()) {
        exported_->addVTable(
            sdbus::registerProperty("SecondaryChannel")
                .withGetter([this]() -> std::string {
                    // Select the radio mode used for the extended-advertising secondary packet.
                    return secondary_channel_;
                })
        ).forInterface(interface_name);
    }
    if (min_interval_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("MinInterval")
                .withGetter([this]() -> uint32_t {
                    // Expose the shortest configured interval between advertising packets.
                    return *min_interval_;
                })
        ).forInterface(interface_name);
    }
    if (max_interval_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("MaxInterval")
                .withGetter([this]() -> uint32_t {
                    // Expose the longest configured interval between advertising packets.
                    return *max_interval_;
                })
        ).forInterface(interface_name);
    }
    if (tx_power_.has_value()) {
        exported_->addVTable(
            sdbus::registerProperty("TxPower")
                .withGetter([this]() -> int16_t {
                    // Request the configured transmit power for these advertising packets.
                    return *tx_power_;
                })
        ).forInterface(interface_name);
    }
}

void Advertisement::unexport() {
    // Stop exporting advertisement.
    exported_.reset();
}

void Advertisement::register_advertisement(const std::string& adapter_path) {
    // Register advertisement.
    if (exported_) {
        unexport();
    }
    export_object();
    auto proxy = sdbus::createProxy(dbus_.connection(),
                                    sdbus::ServiceName{std::string(kBluezServiceName)},
                                    sdbus::ObjectPath{adapter_path});
    std::map<std::string, sdbus::Variant> options;
    released_ = false;
    registration_ = Registration::Registering;
    auto future = proxy->callMethodAsync("RegisterAdvertisement")
        .onInterface(std::string(kLeAdvManagerIface))
        .withArguments(sdbus::ObjectPath{path_}, options)
        .getResultAsFuture();

    const auto status = future.wait_for(std::chrono::seconds(30));
    if (status == std::future_status::timeout) {
        // BlueZ lists the client before programming the controller. Cancel
        // that pending client now; otherwise later cleanup can skip it and an
        // old packet may remain on air after this process has exited.
        unregister_advertisement(adapter_path);
        throw sdbus::Error(sdbus::Error::Name{"org.freedesktop.DBus.Error.Timeout"},
                           "RegisterAdvertisement timed out");
    }

    try {
        future.get();
    } catch (...) {
        unregister_advertisement(adapter_path);
        throw;
    }
    auto expected = Registration::Registering;
    if (!registration_.compare_exchange_strong(expected, Registration::Registered)) {
        throw sdbus::Error(sdbus::Error::Name{"org.bluez.Error.Failed"},
                           "Advertisement released before registration completed");
    }
}

void Advertisement::set_manufacturer_data(const std::map<uint16_t, std::vector<uint8_t>>& data) {
    // Convert each company payload to a D-Bus byte-array variant for the next export.
    manufacturer_data_ = to_variant_map(data);
}

void Advertisement::set_service_data(const std::map<std::string, std::vector<uint8_t>>& data) {
    // Convert each service payload to a D-Bus byte-array variant for the next export.
    service_data_ = to_variant_map(data);
}

void Advertisement::set_data(const std::map<uint8_t, std::vector<uint8_t>>& data) {
    // Replace mutable raw advertisement fields under the lock used by BlueZ getters.
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_ = to_variant_map(data);
}

void Advertisement::set_scan_response_manufacturer_data(
    const std::map<uint16_t, std::vector<uint8_t>>& data) {
    // Convert vendor payloads for inclusion in the optional scan response.
    scan_response_manufacturer_data_ = to_variant_map(data);
}

void Advertisement::set_scan_response_service_data(
    const std::map<std::string, std::vector<uint8_t>>& data) {
    // Convert service payloads for inclusion in the optional scan response.
    scan_response_service_data_ = to_variant_map(data);
}

void Advertisement::set_scan_response_data(const std::map<uint8_t, std::vector<uint8_t>>& data) {
    // Convert raw field types for inclusion in the optional scan response.
    scan_response_data_ = to_variant_map(data);
}

void Advertisement::emit_property_changed(const std::string& property_name) {
    // Signal only while the LEAdvertisement1 object remains exported.
    if (!exported_) {
        return;
    }

    try {
        exported_->emitPropertiesChangedSignal(
            sdbus::InterfaceName{std::string(kLeAdvertisementIface)},
            std::vector<sdbus::PropertyName>{sdbus::PropertyName{property_name}});
    } catch (const sdbus::Error& error) {
        log_properties_changed_failure(path_, property_name, error);
    }
}

void Advertisement::unregister_advertisement(const std::string& adapter_path) {
    // Atomically leave the registered lifecycle, release the manager entry once,
    // and remove the exported advertisement object.
    const auto previous = registration_.exchange(Registration::Unregistered);
    if (previous == Registration::Unregistered) {
        unexport();
        return;
    }
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path});
        proxy->callMethod("UnregisterAdvertisement")
            .onInterface(std::string(kLeAdvManagerIface))
            .withArguments(sdbus::ObjectPath{path_});
    } catch (const sdbus::Error& error) {
        // A released or failed registration is already absent. Report other
        // failures because silently losing cleanup can leave stale data on air.
        if (error.getName() != "org.bluez.Error.DoesNotExist") {
            registration_ = previous;
            RCLCPP_WARN(rclcpp::get_logger("mrs_uav_bluetooth"),
                        "Could not unregister advertisement %s: %s",
                        path_.c_str(), error.what());
            throw;
        }
    }
    unexport();
    registration_ = Registration::Unregistered;
}

}  // namespace mrs_uav_bluetooth::gatt
