// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/remote_gatt_inspector.hpp"

#include "mrs_uav_bluetooth/gatt/builtin_gatt.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>
#include <sstream>
#include <string_view>

namespace mrs_uav_bluetooth::gatt {

namespace {

bool contains_flag(const std::vector<std::string>& flags, std::string_view expected) {
    return std::find(flags.begin(), flags.end(), expected) != flags.end();
}

std::string bytes_to_ascii(const std::vector<uint8_t>& value) {
    std::string result;
    result.reserve(value.size());
    for (const auto byte : value) {
        if (byte == 0) {
            continue;
        }
        if (byte < 32 || byte > 126) {
            return {};
        }
        result.push_back(static_cast<char>(byte));
    }
    return util::trim_ascii_copy(std::move(result));
}

bool looks_like_message_type(const std::string& value) {
    const auto first = value.find('/');
    if (first == std::string::npos || first == 0 || first + 1 >= value.size()) {
        return false;
    }
    const auto second = value.find('/', first + 1);
    if (second == std::string::npos || second + 1 >= value.size()) {
        return false;
    }
    if (value.find('/', second + 1) != std::string::npos) {
        return false;
    }
    return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
        return std::isalnum(ch) || ch == '_' || ch == '/';
    });
}

bool looks_like_payload_format(const std::string& value) {
    return value == "struct" || value == "ros2";
}

bool looks_like_rate_hz(const std::string& value) {
    if (value.empty()) {
        return false;
    }
    size_t parsed = 0;
    const double rate = std::stod(value, &parsed);
    return parsed == value.size() && std::isfinite(rate) && rate >= 0.0;
}

bool looks_like_member_type(const std::string& value) {
    static const std::set<std::string> kKnownTypes{
        "bool", "int8", "uint8", "int16", "uint16", "int32", "uint32",
        "int64", "uint64", "float32", "float64", "time_ns",
    };
    return kKnownTypes.find(value) != kKnownTypes.end();
}

bool looks_like_member_layout(const std::string& value) {
    if (value.empty()) {
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::Load(value);
    } catch (...) {
        return false;
    }

    if (!root || !root.IsSequence() || root.size() == 0) {
        return false;
    }

    for (const auto& entry : root) {
        if (!entry.IsMap() || !entry["path"] || !entry["type"]) {
            return false;
        }
        const auto path = entry["path"].as<std::string>("");
        const auto type = entry["type"].as<std::string>("");
        if (path.empty() || !looks_like_member_type(type)) {
            return false;
        }
    }

    return true;
}

enum class DescriptorKind {
    Unknown,
    MessageType,
    PayloadFormat,
    Members,
    RateHz,
};

DescriptorKind classify_descriptor_payload(const std::vector<uint8_t>& payload) {
    const auto text = bytes_to_ascii(payload);
    if (text.empty()) {
        return DescriptorKind::Unknown;
    }
    if (looks_like_payload_format(text)) {
        return DescriptorKind::PayloadFormat;
    }
    if (looks_like_rate_hz(text)) {
        return DescriptorKind::RateHz;
    }
    if (looks_like_member_layout(text)) {
        return DescriptorKind::Members;
    }
    if (looks_like_message_type(text)) {
        return DescriptorKind::MessageType;
    }
    return DescriptorKind::Unknown;
}

}  // namespace

bool RemoteBuiltinPaths::has_wifi() const {
    return !wifi_ssid_characteristic_path.empty() &&
           !wifi_password_characteristic_path.empty() &&
           !wifi_status_characteristic_path.empty();
}

bool RemoteBuiltinPaths::has_time() const {
    return !time_characteristic_path.empty();
}

RemoteGattInspector::RemoteGattInspector(bluez::BluezClient& client)
    : client_(client) {}

RemoteBuiltinPaths RemoteGattInspector::resolve_builtin_paths(const std::string& mac) {
    RemoteBuiltinPaths paths;
    paths.wifi_ssid_characteristic_path = client_.find_characteristic(mac, wifi_ssid_characteristic_uuid());
    paths.wifi_password_characteristic_path = client_.find_characteristic(mac, wifi_password_characteristic_uuid());
    paths.wifi_status_characteristic_path = client_.find_characteristic(mac, wifi_status_characteristic_uuid());
    paths.time_characteristic_path = client_.find_characteristic(mac, time_characteristic_uuid());
    if (!paths.time_characteristic_path.empty()) {
        paths.time_writeback_descriptor_path = client_.find_descriptor(
            mac, time_writeback_descriptor_uuid(), paths.time_characteristic_path);
    }
    return paths;
}

ExportedTopicCountResult RemoteGattInspector::count_exported_topics(const std::string& mac) {
    ExportedTopicCountResult result;
    const auto services = client_.list_services(mac);
    const auto characteristics = client_.list_characteristics(mac);

    std::map<std::string, std::string> service_uuids;
    for (const auto& service : services) {
        service_uuids.emplace(service.object_path, service.uuid);
    }

    result.inspected_services = services.size();
    for (const auto& characteristic : characteristics) {
        result.inspected_characteristics += 1;

        const auto service_it = service_uuids.find(characteristic.service_path);
        if (service_it == service_uuids.end()) {
            continue;
        }
        if (is_wifi_service_uuid(service_it->second) || is_time_service_uuid(service_it->second)) {
            continue;
        }

        const auto descriptors = client_.list_descriptors(mac, characteristic.object_path);
        if (characteristic_looks_like_export_bridge(characteristic, descriptors)) {
            result.discovered_topics += 1;
        }
    }

    return result;
}

bool RemoteGattInspector::characteristic_looks_like_export_bridge(
    const bluez::GattCharacteristicInfo& characteristic,
    const std::vector<bluez::GattDescriptorInfo>& descriptors) {
    if (!contains_flag(characteristic.flags, "read") ||
        !contains_flag(characteristic.flags, "notify")) {
        return false;
    }
    if (contains_flag(characteristic.flags, "write") ||
        contains_flag(characteristic.flags, "write-without-response")) {
        return false;
    }
    if (descriptors.size() != 4) {
        return false;
    }

    bool found_message_type = false;
    bool found_payload_format = false;
    bool found_members = false;
    bool found_rate = false;

    for (const auto& descriptor : descriptors) {
        if (!contains_flag(descriptor.flags, "read")) {
            return false;
        }

        const auto kind = classify_descriptor_payload(client_.read_descriptor(descriptor.object_path));
        switch (kind) {
        case DescriptorKind::MessageType:
            if (found_message_type) {
                return false;
            }
            found_message_type = true;
            break;
        case DescriptorKind::PayloadFormat:
            if (found_payload_format) {
                return false;
            }
            found_payload_format = true;
            break;
        case DescriptorKind::Members:
            if (found_members) {
                return false;
            }
            found_members = true;
            break;
        case DescriptorKind::RateHz:
            if (found_rate) {
                return false;
            }
            found_rate = true;
            break;
        case DescriptorKind::Unknown:
            return false;
        }
    }

    return found_message_type && found_payload_format && found_members && found_rate;
}

}  // namespace mrs_uav_bluetooth::gatt