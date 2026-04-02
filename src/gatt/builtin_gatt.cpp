// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/builtin_gatt.hpp"

#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

namespace mrs_uav_bluetooth::gatt {

namespace {

const std::string& cached_uuid(const char* name, bool descriptor, bool service) {
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

const std::string& wifi_service_uuid() {
    return cached_uuid("wifi", false, true);
}

const std::string& wifi_ssid_characteristic_uuid() {
    return cached_uuid("wifi/ssid", false, false);
}

const std::string& wifi_password_characteristic_uuid() {
    return cached_uuid("wifi/password", false, false);
}

const std::string& wifi_status_characteristic_uuid() {
    return cached_uuid("wifi/status", false, false);
}

const std::string& time_service_uuid() {
    return cached_uuid("time", false, true);
}

const std::string& time_characteristic_uuid() {
    return cached_uuid("time/ns", false, false);
}

const std::string& time_value_descriptor_uuid() {
    return cached_uuid("time/ns/value", true, false);
}

const std::string& time_writeback_descriptor_uuid() {
    return cached_uuid("time/ns/writeback", true, false);
}

bool is_wifi_service_uuid(std::string_view uuid) {
    return uuid == wifi_service_uuid();
}

bool is_time_service_uuid(std::string_view uuid) {
    return uuid == time_service_uuid();
}

std::string builtin_service_name(std::string_view service_uuid) {
    if (is_wifi_service_uuid(service_uuid)) {
        return "wifi";
    }
    if (is_time_service_uuid(service_uuid)) {
        return "time";
    }
    return std::string(service_uuid);
}

std::string builtin_characteristic_name(std::string_view service_uuid,
                                        std::string_view characteristic_uuid) {
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

std::string builtin_descriptor_name(std::string_view service_uuid,
                                    std::string_view descriptor_uuid) {
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