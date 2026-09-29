// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/remote_gatt_inspector.hpp
/// \brief Declares the remote gatt inspector component of the Bluetooth Low Energy GATT layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"

#include <cstddef>
#include <string>

namespace mrs_uav_bluetooth::gatt {

/// Object paths of optional built-in services found on one remote peer.
struct RemoteBuiltinPaths {
    std::string wifi_ssid_characteristic_path;
    std::string wifi_password_characteristic_path;
    std::string wifi_status_characteristic_path;
    std::string time_characteristic_path;
    std::string time_writeback_descriptor_path;

    /// \brief Report whether all required remote Wi-Fi characteristics were discovered.
    /// \return True when Wi-Fi is present; otherwise false.
    bool has_wifi() const;
    /// \brief Report whether the remote time characteristic was discovered.
    /// \return True when time is present; otherwise false.
    bool has_time() const;
};

/// Result and diagnostics from counting a peer's exported topic bridges.
struct ExportedTopicCountResult {
    size_t discovered_topics{0};
    size_t inspected_services{0};
    size_t inspected_characteristics{0};
};

/// Read-only helper that interprets cached remote GATT services by UUID.
class RemoteGattInspector {
public:
    /// \brief Bind built-in and topic-bridge discovery to the shared BlueZ client.
    /// \param client BlueZ client used for remote discovery and GATT operations.
    explicit RemoteGattInspector(bluez::BluezClient& client);

    /// \brief Locate and validate the peer's built-in Wi-Fi and time GATT objects.
    /// \param mac peer Bluetooth MAC address.
    /// \return Resolved paths for the peer built-in Wi-Fi and time objects.
    RemoteBuiltinPaths resolve_builtin_paths(const std::string& mac);
    /// \brief Count remote characteristics with complete bridge metadata.
    /// \param mac peer Bluetooth MAC address.
    /// \return Number of remote characteristics recognized as complete export bridges.
    ExportedTopicCountResult count_exported_topics(const std::string& mac);

private:
    /// \brief Validate flags and metadata for one remote export characteristic.
    /// \param characteristic Remote GATT characteristic checked for the complete bridge descriptor set.
    /// \param descriptors candidate descriptors searched for bridge metadata.
    /// \return True when characteristic looks like export bridge; otherwise false.
    bool characteristic_looks_like_export_bridge(
        const bluez::GattCharacteristicInfo& characteristic,
        const std::vector<bluez::GattDescriptorInfo>& descriptors);

    bluez::BluezClient& client_;
};

}  // namespace mrs_uav_bluetooth::gatt
