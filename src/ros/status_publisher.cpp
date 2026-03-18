// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/ros/status_publisher.hpp"

#include "mrs_uav_bluetooth/msg/ble_device.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

namespace mrs_uav_bluetooth::ros {

StatusPublisher::StatusPublisher(rclcpp::Node& node)
    : node_(node) {
    configure_topics("/ble");
}

void StatusPublisher::configure_topics(const std::string& node_topics_prefix) {
    node_topics_prefix_ = util::normalize_ros_topic(node_topics_prefix);
    auto status_topic = util::normalize_ros_topic(node_topics_prefix_ + "/status");
    auto devices_topic = util::normalize_ros_topic(node_topics_prefix_ + "/devices");
    auto notifications_topic = util::normalize_ros_topic(node_topics_prefix_ + "/notifications");
    status_pub_ = node_.create_publisher<std_msgs::msg::String>(status_topic, 50);
    devices_pub_ = node_.create_publisher<mrs_uav_bluetooth::msg::BleDeviceArray>(devices_topic, 10);
    notifications_pub_ = node_.create_publisher<mrs_uav_bluetooth::msg::BleNotification>(notifications_topic, 50);
}

void StatusPublisher::publish_report(const std::string& report) {
    std_msgs::msg::String msg;
    msg.data = report;
    status_pub_->publish(msg);
}

void StatusPublisher::publish_devices(const std::map<std::string, bluez::DeviceInfo>& devices) {
    mrs_uav_bluetooth::msg::BleDeviceArray msg;
    msg.header.stamp = node_.get_clock()->now();
    for (const auto& [mac, device] : devices) {
        (void)mac;
        mrs_uav_bluetooth::msg::BleDevice item;
        item.mac = device.mac;
        item.path = device.object_path;
        item.adapter = device.adapter;
        item.address_type = device.address_type;
        item.name = device.name;
        item.alias = device.alias;
        item.icon = device.icon;
        item.appearance = device.appearance;
        item.rssi = device.rssi;
        item.tx_power = device.tx_power;
        item.pathloss = 0;
        item.connected = device.connected;
        item.paired = device.paired;
        item.bonded = device.bonded;
        item.trusted = device.trusted;
        item.blocked = device.blocked;
        item.services_resolved = device.services_resolved;
        item.uuids = device.uuids;
        item.last_seen = node_.get_clock()->now();
        msg.devices.push_back(item);
    }
    devices_pub_->publish(msg);
}

void StatusPublisher::publish_notification(const std::string& mac,
                                           const std::string& path,
                                           const std::string& uuid,
                                           const std::vector<uint8_t>& value,
                                           const std::string& frame_id) {
    mrs_uav_bluetooth::msg::BleNotification msg;
    msg.header.stamp = node_.get_clock()->now();
    msg.header.frame_id = util::sanitize_topic_suffix(frame_id);
    msg.mac = mac;
    msg.path = path;
    msg.uuid = uuid;
    msg.value = value;
    notifications_pub_->publish(msg);
}

}  // namespace mrs_uav_bluetooth::ros
