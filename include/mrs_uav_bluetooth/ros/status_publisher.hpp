// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/ros/status_publisher.hpp
/// \brief Declares the status publisher component of the ROS 2 interface layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/msg/ble_device_array.hpp"
#include "mrs_uav_bluetooth/msg/ble_notification.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::ros {

/// Converts cached BlueZ state and notifications into stable ROS messages.
class StatusPublisher {
public:
    /// \brief Create publishers for device, link, Mesh, and system status.
    /// \param node ROS node that owns the created interfaces.
    explicit StatusPublisher(rclcpp::Node& node);

    /// \brief Create publishers for aggregate status devices notifications reports and logs.
    /// \param node_topics_prefix normalized ROS namespace below which endpoints are created.
    void configure_topics(const std::string& node_topics_prefix);
    /// \brief Select the raw advertisement field copied into each published device message.
    /// \param data_type Raw advertisement field type reserved for application data.
    void set_advertisement_user_data_type(uint8_t data_type);

    /// \brief Publish a complete operator report without altering transport state.
    /// \param report stream receiving the formatted operator report.
    void publish_report(const std::string& report);
    /// \brief Publish one log line when log-topic output is enabled.
    /// \param report stream receiving the formatted operator report.
    void publish_log(const std::string& report);
    /// \brief Convert a cache snapshot into the public ROS device array.
    /// \param devices Cached BlueZ devices converted and published as a ROS array.
    void publish_devices(const std::map<std::string, bluez::DeviceInfo>& devices);
    /// \brief Publish a remote GATT value with peer path UUID and receive timestamp.
    /// \param mac peer Bluetooth MAC address.
    /// \param path BlueZ path of the characteristic that emitted the notification.
    /// \param uuid UUID of the characteristic that emitted the value.
    /// \param value Notification bytes received from the characteristic.
    /// \param frame_id identifier of the frame.
    void publish_notification(const std::string& mac,
                              const std::string& path,
                              const std::string& uuid,
                              const std::vector<uint8_t>& value,
                              const std::string& frame_id = "");

    /// \brief Access the publisher bundle that owns public status topics.
    /// \return Publisher used for the aggregate Bluetooth status topic.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher() const {
        // Return the publisher bundle that owns every public status topic.
        return status_pub_;
    }

private:
    rclcpp::Node& node_;
    std::string node_topics_prefix_;
    uint8_t advertisement_user_data_type_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr devices_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr advertisements_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::BleNotification>::SharedPtr notifications_pub_;
};

}  // namespace mrs_uav_bluetooth::ros
