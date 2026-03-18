// SPDX-License-Identifier: MIT
#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bluez {

/// Runtime representation of a discovered or cached BLE device.
struct DeviceInfo {
    std::string object_path;
    std::string adapter;
    std::string mac;
    std::string address_type;
    std::string name;
    std::string alias;
    std::string icon;
    int32_t appearance{0};
    int16_t rssi{0};
    int16_t tx_power{0};
    bool connected{false};
    bool paired{false};
    bool bonded{false};
    bool trusted{false};
    bool blocked{false};
    bool services_resolved{false};
    std::vector<std::string> uuids;
    std::map<uint16_t, std::vector<uint8_t>> manufacturer_data;
    std::map<std::string, std::vector<uint8_t>> service_data;
    std::chrono::steady_clock::time_point last_seen;
};

/// Runtime representation of a remote GATT service.
struct GattServiceInfo {
    std::string object_path;
    std::string uuid;
    bool primary{true};
    std::string device_path;
    std::vector<std::string> includes;
};

/// Runtime representation of a remote GATT characteristic.
struct GattCharacteristicInfo {
    std::string object_path;
    std::string service_path;
    std::string uuid;
    std::vector<std::string> flags;
    bool notifying{false};
    uint16_t mtu{0};
    std::vector<uint8_t> value;
};

/// Runtime representation of a remote GATT descriptor.
struct GattDescriptorInfo {
    std::string object_path;
    std::string characteristic_path;
    std::string uuid;
    std::vector<std::string> flags;
    std::vector<uint8_t> value;
};

/// Adapter information.
struct AdapterInfo {
    std::string object_path;
    std::string address;
    std::string address_type;
    std::string name;
    std::string alias;
    bool powered{false};
    bool discoverable{false};
    uint32_t discoverable_timeout{0};
    bool pairable{false};
    bool discovering{false};
    std::vector<std::string> uuids;
};

}  // namespace mrs_uav_bluetooth::bluez
