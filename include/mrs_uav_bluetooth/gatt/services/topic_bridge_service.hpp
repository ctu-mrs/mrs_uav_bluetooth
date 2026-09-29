// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/services/topic_bridge_service.hpp
/// \brief Declares the topic bridge service component of the Bluetooth Low Energy GATT layer.

#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <string>
#include <vector>

namespace mrs_uav_bluetooth::gatt::services {

/// Topic bridge BLE service exporting one ROS topic as a GATT characteristic
/// with metadata descriptors (topic, type, format, members, rate_hz, key).
class TopicBridgeService {
public:
    /// \brief Build one topic value characteristic and its metadata descriptors.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param base_path Application D-Bus path beneath which this service is exported.
    /// \param index Bridge service number used to keep this exported D-Bus subtree unique.
    /// \param topic_name Resolved ROS topic exposed in the bridge metadata.
    /// \param message_type Fully qualified ROS message type exposed in bridge metadata.
    /// \param bridge_name Stable bridge name used to derive service and characteristic UUIDs.
    /// \param bridge_key Stable bridge identity exposed in the metadata descriptors.
    /// \param member_specs Ordered ROS fields serialized into the characteristic value.
    /// \param rate_hz requested rate in hertz.
    /// \param payload_format Wire format advertised to peers for decoding this bridge value.
    TopicBridgeService(bluez::DbusConnection& dbus,
                       const std::string& base_path,
                       int index,
                       const std::string& topic_name,
                       const std::string& message_type,
                       const std::string& bridge_name,
                       const std::string& bridge_key,
                       const std::vector<config::BridgeMemberSpec>& member_specs,
                       double rate_hz,
                       const std::string& payload_format);

    /// \brief Access the complete exported topic-bridge object tree.
    /// \return GATT service object owned by this wrapper.
    std::shared_ptr<GattService> service() const {
        // Return the complete exported topic-bridge service object tree.
        return service_;
    }
    /// \brief Return the deterministic UUID of the bridge value characteristic.
    /// \return Deterministic UUID of the topic-bridge value characteristic.
    std::string characteristic_uuid() const;
    /// \brief Return the D-Bus path of the bridge value characteristic.
    /// \return D-Bus path of the topic-bridge value characteristic.
    std::string characteristic_path() const;

    /// \brief Return the value path used by transport bridge state.
    /// \return Stable internal path used to associate this service with a transport bridge.
    std::string transport_path() const;

    /// \brief Replace the bridge value and notify subscribed GATT clients.
    /// \param payload Encoded topic bridge frame exposed through the value characteristic.
    void publish(const std::vector<uint8_t>& payload);

private:
    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> characteristic_;

};

}  // namespace mrs_uav_bluetooth::gatt::services
