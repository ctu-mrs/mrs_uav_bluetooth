// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>

namespace mrs_uav_bluetooth::bridge {

class ImportBridgeManager {
public:
    explicit ImportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger);

    void set_registry(BridgeRegistry* registry);
    void destroy_import_bridge(TopicImportBridgeState& state);
    void configure_import_bridge(const std::string& bridge_key,
                                 TopicImportBridgeState& state,
                                 bluez::BluezClient& client);
    void configure_import_poll_timer(const std::string& bridge_key,
                                     TopicImportBridgeState& state,
                                     bluez::BluezClient& client);
    bool buffer_notification_payload(const std::string& mac,
                                     const std::string& characteristic_path,
                                     const std::vector<uint8_t>& payload);
    bool refresh_import_paths_for_mac(const std::string& mac,
                                      bluez::BluezClient& client);
    bool clear_import_paths_for_mac(const std::string& mac,
                                    bluez::BluezClient& client);

private:
    std::shared_ptr<GenericMessageBridge> runtime_for(TopicImportBridgeState& state);
    bool publish_payload(TopicImportBridgeState& state, const std::vector<uint8_t>& payload);

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    BridgeRegistry* registry_{nullptr};
};

}  // namespace mrs_uav_bluetooth::bridge
