// SPDX-License-Identifier: BSD-3-Clause
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

    std::shared_ptr<GattService> service() const { return service_; }
    std::string characteristic_uuid() const;
    std::string characteristic_path() const;

    std::string transport_path() const;

    void publish(const std::vector<uint8_t>& payload);

private:
    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> characteristic_;
    std::vector<uint8_t> payload_;
};

}  // namespace mrs_uav_bluetooth::gatt::services
