// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/msg/ble_device_array.hpp"
#include "mrs_uav_bluetooth/msg/ble_notification.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <map>
#include <string>

namespace mrs_uav_bluetooth::ros {

class StatusPublisher {
public:
    explicit StatusPublisher(rclcpp::Node& node);

    void configure_topics(const std::string& node_topics_prefix);

    void publish_report(const std::string& report);
    void publish_devices(const std::map<std::string, bluez::DeviceInfo>& devices);
    void publish_notification(const std::string& mac,
                              const std::string& path,
                              const std::string& uuid,
                              const std::vector<uint8_t>& value,
                              const std::string& frame_id = "");

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher() const {
        return status_pub_;
    }

private:
    rclcpp::Node& node_;
    std::string node_topics_prefix_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr devices_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::BleNotification>::SharedPtr notifications_pub_;
};

}  // namespace mrs_uav_bluetooth::ros
