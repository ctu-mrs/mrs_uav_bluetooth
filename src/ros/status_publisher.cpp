// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/ros/status_publisher.hpp"

#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"
#include "mrs_uav_bluetooth/msg/ble_device.hpp"
#include "mrs_uav_bluetooth/util/device_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

namespace mrs_uav_bluetooth::ros {

namespace {

template<typename MapT>
std::vector<uint8_t> concatenate_byte_values(const MapT& values) {
    std::vector<uint8_t> bytes;
    for (const auto& [key, value] : values) {
        (void)key;
        bytes.insert(bytes.end(), value.begin(), value.end());
    }
    return bytes;
}

std::vector<uint8_t> advertisement_user_payload(const bluez::DeviceInfo& device,
                                                uint8_t data_type) {
    const auto it = device.advertising_data.find(data_type);
    if (it == device.advertising_data.end()) {
        return {};
    }
    return it->second;
}

mrs_uav_bluetooth::msg::BleDevice to_device_msg(rclcpp::Node& node,
                                                const bluez::DeviceInfo& device,
                                                uint8_t advertisement_user_data_type) {
    mrs_uav_bluetooth::msg::BleDevice item;
    item.mac = device.mac;
    item.path = device.object_path;
    item.adapter = device.adapter;
    item.address_type = device.address_type;
    item.name = device.name;
    item.alias = device.alias;
    item.hostname = util::device_hostname_guess(device);
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
    item.manufacturer_data = concatenate_byte_values(device.manufacturer_data);
    item.service_data = concatenate_byte_values(device.service_data);
    item.advertising_flags = device.advertising_flags;
    item.advertising_data = advertisement_user_payload(device, advertisement_user_data_type);
    item.last_seen = node.get_clock()->now();
    return item;
}

bool has_advertisement_user_data(const bluez::DeviceInfo& device,
                                 uint8_t advertisement_user_data_type) {
    return !advertisement_user_payload(device, advertisement_user_data_type).empty();
}

}  // namespace

StatusPublisher::StatusPublisher(rclcpp::Node& node)
    : node_(node),
      advertisement_user_data_type_(bluez::kDefaultAdvertisementExtraDataType) {
    configure_topics("/ble");
}

void StatusPublisher::set_advertisement_user_data_type(uint8_t data_type) {
    advertisement_user_data_type_ = data_type;
}

void StatusPublisher::configure_topics(const std::string& node_topics_prefix) {
    node_topics_prefix_ = util::normalize_ros_topic(node_topics_prefix);
    auto status_topic = util::normalize_ros_topic(node_topics_prefix_ + "/status");
    auto log_topic = util::normalize_ros_topic(node_topics_prefix_ + "/log");
    auto devices_topic = util::normalize_ros_topic(node_topics_prefix_ + "/devices");
    auto advertisements_topic = util::normalize_ros_topic(node_topics_prefix_ + "/advertisements");
    auto notifications_topic = util::normalize_ros_topic(node_topics_prefix_ + "/notifications");
    status_pub_ = node_.create_publisher<std_msgs::msg::String>(status_topic, 50);
    log_pub_ = node_.create_publisher<std_msgs::msg::String>(log_topic, 10);
    devices_pub_ = node_.create_publisher<mrs_uav_bluetooth::msg::BleDeviceArray>(devices_topic, 10);
    advertisements_pub_ = node_.create_publisher<mrs_uav_bluetooth::msg::BleDeviceArray>(advertisements_topic, 10);
    notifications_pub_ = node_.create_publisher<mrs_uav_bluetooth::msg::BleNotification>(notifications_topic, 50);
}

void StatusPublisher::publish_report(const std::string& report) {
    std_msgs::msg::String msg;
    msg.data = report;
    status_pub_->publish(msg);
}

void StatusPublisher::publish_log(const std::string& report) {
    std_msgs::msg::String msg;
    msg.data = report;
    log_pub_->publish(msg);
}

void StatusPublisher::publish_devices(const std::map<std::string, bluez::DeviceInfo>& devices) {
    mrs_uav_bluetooth::msg::BleDeviceArray msg;
    mrs_uav_bluetooth::msg::BleDeviceArray advertisement_msg;
    msg.header.stamp = node_.get_clock()->now();
    advertisement_msg.header = msg.header;
    for (const auto& [mac, device] : devices) {
        (void)mac;
        auto item = to_device_msg(node_, device, advertisement_user_data_type_);
        msg.devices.push_back(item);
        if (has_advertisement_user_data(device, advertisement_user_data_type_)) {
            advertisement_msg.devices.push_back(std::move(item));
        }
    }
    devices_pub_->publish(msg);
    advertisements_pub_->publish(advertisement_msg);
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
