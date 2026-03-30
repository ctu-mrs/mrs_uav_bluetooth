// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

namespace mrs_uav_bluetooth::bridge {

class ExportBridgeManager {
public:
    explicit ExportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger);

    void set_registry(BridgeRegistry* registry);
    void set_state_mutex(std::recursive_mutex* mutex);
    void configure_export_bridge(const std::string& bridge_key,
                                 TopicExportBridgeState& state);
    void rebuild_gatt_services(gatt::GattApplication& app,
                               bluez::DbusConnection& dbus,
                               const std::string& app_base_path);
    void publish_export_payload(const std::string& bridge_key,
                                const std::vector<uint8_t>& payload);
    void destroy_export_bridge(TopicExportBridgeState& state);
    void configure_export_rate_timer(const std::string& bridge_key,
                                     TopicExportBridgeState& state);

private:
    std::shared_ptr<GenericMessageBridge> runtime_for(TopicExportBridgeState& state);

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    BridgeRegistry* registry_{nullptr};
    std::recursive_mutex* state_mutex_{nullptr};
};

}  // namespace mrs_uav_bluetooth::bridge
