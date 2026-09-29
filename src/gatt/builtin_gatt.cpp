// SPDX-License-Identifier: BSD-3-Clause
/// \file src/gatt/builtin_gatt.cpp
/// \brief Implements the builtin gatt component of the Bluetooth Low Energy GATT layer.

#include "mrs_uav_bluetooth/gatt/builtin_gatt.hpp"

#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

namespace mrs_uav_bluetooth::gatt {

namespace {

/// \brief Memoize a deterministic built-in UUID so every caller receives the same value.
/// \param name Built-in object name used to derive and cache its UUID.
/// \param descriptor cached or exported GATT descriptor being converted.
/// \param service cached or exported GATT service that owns the child object.
/// \return Stable built-in UUID retained for process lifetime.
const std::string& cached_uuid(const char* name, bool descriptor, bool service) {
    // Memoize a deterministic built-in UUID so every caller receives the same value.
    static const std::string empty;
    if (name == nullptr) {
        return empty;
    }

    if (service) {
        static const std::string wifi_service = util::named_service_uuid("wifi");
        static const std::string time_service = util::named_service_uuid("time");
        if (std::string_view(name) == "wifi") {
            return wifi_service;
        }
        return time_service;
    }

    if (descriptor) {
        static const std::string time_value = util::named_descriptor_uuid("time/ns/value");
        static const std::string time_writeback = util::named_descriptor_uuid("time/ns/writeback");
        if (std::string_view(name) == "time/ns/value") {
            return time_value;
        }
        return time_writeback;
    }

    static const std::string wifi_ssid = util::named_characteristic_uuid("wifi/ssid");
    static const std::string wifi_password = util::named_characteristic_uuid("wifi/password");
    static const std::string wifi_status = util::named_characteristic_uuid("wifi/status");
    static const std::string time_ns = util::named_characteristic_uuid("time/ns");
    const std::string_view view(name);
    if (view == "wifi/ssid") {
        return wifi_ssid;
    }
    if (view == "wifi/password") {
        return wifi_password;
    }
    if (view == "wifi/status") {
        return wifi_status;
    }
    return time_ns;
}

}  // namespace

/// \brief Return the deterministic UUID for the built-in Wi-Fi service.
/// \return Deterministic UUID of the built-in Wi-Fi service.
const std::string& wifi_service_uuid() {
    // Return the deterministic UUID for the built-in Wi-Fi service.
    return cached_uuid("wifi", false, true);
}

/// \brief Return the deterministic UUID for the readable and writable network name.
/// \return Deterministic UUID of the Wi-Fi network-name characteristic.
const std::string& wifi_ssid_characteristic_uuid() {
    // Return the deterministic UUID for the readable and writable network name.
    return cached_uuid("wifi/ssid", false, false);
}

/// \brief Return the deterministic UUID for the writable network password.
/// \return Deterministic UUID of the Wi-Fi password characteristic.
const std::string& wifi_password_characteristic_uuid() {
    // Return the deterministic UUID for the writable network password.
    return cached_uuid("wifi/password", false, false);
}

/// \brief Return the deterministic UUID for Wi-Fi operation status.
/// \return Deterministic UUID of the Wi-Fi status characteristic.
const std::string& wifi_status_characteristic_uuid() {
    // Return the deterministic UUID for Wi-Fi operation status.
    return cached_uuid("wifi/status", false, false);
}

/// \brief Return the deterministic UUID for the built-in time service.
/// \return Deterministic UUID of the built-in time service.
const std::string& time_service_uuid() {
    // Return the deterministic UUID for the built-in time service.
    return cached_uuid("time", false, true);
}

/// \brief Return the deterministic UUID for the built-in nanosecond characteristic.
/// \return Deterministic UUID of the nanosecond time characteristic.
const std::string& time_characteristic_uuid() {
    // Return the deterministic UUID for the built-in nanosecond characteristic.
    return cached_uuid("time/ns", false, false);
}

/// \brief Return the deterministic UUID for the readable time value descriptor.
/// \return Deterministic UUID of the readable time descriptor.
const std::string& time_value_descriptor_uuid() {
    // Return the deterministic UUID for the readable time value descriptor.
    return cached_uuid("time/ns/value", true, false);
}

/// \brief Return the deterministic UUID for peer time writeback.
/// \return Deterministic UUID of the peer time-writeback descriptor.
const std::string& time_writeback_descriptor_uuid() {
    // Return the deterministic UUID for peer time writeback.
    return cached_uuid("time/ns/writeback", true, false);
}

/// \brief Test whether a UUID identifies the built-in Wi-Fi service.
/// \param uuid Service UUID to compare with the built-in identifier.
/// \return True if the UUID identifies the built-in Wi-Fi service; otherwise false.
bool is_wifi_service_uuid(std::string_view uuid) {
    // Compare against the single deterministic Wi-Fi service identifier.
    return uuid == wifi_service_uuid();
}

/// \brief Test whether a UUID identifies the built-in time service.
/// \param uuid Service UUID to compare with the built-in identifier.
/// \return True if the UUID identifies the built-in time service; otherwise false.
bool is_time_service_uuid(std::string_view uuid) {
    // Compare against the single deterministic time service identifier.
    return uuid == time_service_uuid();
}

/// \brief Map a known built-in service UUID to its operator-facing name.
/// \param service_uuid Service UUID mapped to a built-in display name.
/// \return Known built-in service name, or an empty string.
std::string builtin_service_name(std::string_view service_uuid) {
    // Map a known built-in service UUID to its operator-facing name.
    if (is_wifi_service_uuid(service_uuid)) {
        return "wifi";
    }
    if (is_time_service_uuid(service_uuid)) {
        return "time";
    }
    return std::string(service_uuid);
}

/// \brief Map a known characteristic UUID to its name within the selected built-in service.
/// \param service_uuid Owning service UUID used to disambiguate the characteristic.
/// \param characteristic_uuid UUID of the characteristic.
/// \return Known built-in characteristic name, or an empty string.
std::string builtin_characteristic_name(std::string_view service_uuid,
                                        std::string_view characteristic_uuid) {
    // Map a known characteristic UUID to its name within the selected built-in service.
    if (is_wifi_service_uuid(service_uuid)) {
        if (characteristic_uuid == wifi_ssid_characteristic_uuid()) {
            return "wifi/ssid";
        }
        if (characteristic_uuid == wifi_password_characteristic_uuid()) {
            return "wifi/password";
        }
        if (characteristic_uuid == wifi_status_characteristic_uuid()) {
            return "wifi/status";
        }
    }
    if (is_time_service_uuid(service_uuid) &&
        characteristic_uuid == time_characteristic_uuid()) {
        return "time/ns";
    }
    return std::string(characteristic_uuid);
}

/// \brief Map a known descriptor UUID to its name within the selected built-in service.
/// \param service_uuid Owning service UUID used to disambiguate the descriptor.
/// \param descriptor_uuid UUID of the descriptor.
/// \return Known built-in descriptor name, or an empty string.
std::string builtin_descriptor_name(std::string_view service_uuid,
                                    std::string_view descriptor_uuid) {
    // Map a known descriptor UUID to its name within the selected built-in service.
    if (is_time_service_uuid(service_uuid)) {
        if (descriptor_uuid == time_value_descriptor_uuid()) {
            return "time/ns/value";
        }
        if (descriptor_uuid == time_writeback_descriptor_uuid()) {
            return "time/ns/writeback";
        }
    }
    return std::string(descriptor_uuid);
}

}  // namespace mrs_uav_bluetooth::gatt
