// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"

#include <cstddef>
#include <string>

namespace mrs_uav_bluetooth::gatt {

struct RemoteBuiltinPaths {
    std::string wifi_ssid_characteristic_path;
    std::string wifi_password_characteristic_path;
    std::string wifi_status_characteristic_path;
    std::string time_characteristic_path;
    std::string time_writeback_descriptor_path;

    bool has_wifi() const;
    bool has_time() const;
};

struct ExportedTopicCountResult {
    size_t discovered_topics{0};
    size_t inspected_services{0};
    size_t inspected_characteristics{0};
};

class RemoteGattInspector {
public:
    explicit RemoteGattInspector(bluez::BluezClient& client);

    RemoteBuiltinPaths resolve_builtin_paths(const std::string& mac);
    ExportedTopicCountResult count_exported_topics(const std::string& mac);

private:
    bool characteristic_looks_like_export_bridge(
        const bluez::GattCharacteristicInfo& characteristic,
        const std::vector<bluez::GattDescriptorInfo>& descriptors);

    bluez::BluezClient& client_;
};

}  // namespace mrs_uav_bluetooth::gatt