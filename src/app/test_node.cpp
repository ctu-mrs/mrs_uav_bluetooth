// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/test_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>

namespace {

constexpr const char* kDefaultOdometryTopic = "/{hostname}/mavros/global_position/local";
constexpr const char* kDefaultAdvertisementTopic = "/{hostname}/ble/adv_local_extra";
constexpr const char* kDefaultAdvertisementObserveTopic = "/{hostname}/ble/advertisement";
constexpr const char* kDefaultAdvertisementObserveTopicCompat = "/{hostname}/ble/advertisements";
constexpr double kPi = 3.14159265358979323846;

}  // namespace

namespace mrs_uav_bluetooth::app {

TestNode::TestNode()
    : rclcpp::Node("mrs_uav_bluetooth_test"),
      random_engine_(std::random_device{}()) {
    configure_parameters();
    configure_publishers();
    configure_advertisement_watchers();

    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        [this]() { publish_once(); });

    if (mode_ == Mode::kAdvertisement) {
        advertisement_log_timer_ = create_wall_timer(
            std::chrono::duration<double>(advertisement_log_period_sec_),
            [this]() { log_observed_advertisements(); });
    }
}

void TestNode::configure_parameters() {
    declare_parameter<std::string>("mode", "odometry");
    declare_parameter<double>("rate_hz", 10.0);
    declare_parameter<std::string>("odometry_topic", "");
    declare_parameter<std::string>("advertisement_topic", "");
    declare_parameter<std::string>("advertisement_observe_topic", "");
    declare_parameter<std::string>("advertisement_observe_topic_compat", "");
    declare_parameter<double>("advertisement_log_period_sec", 2.0);
    declare_parameter<std::string>("frame_id", frame_id_);
    declare_parameter<std::string>("child_frame_id", child_frame_id_);

    const auto requested_mode = normalize_mode(get_parameter("mode").as_string());
    rate_hz_ = get_parameter("rate_hz").as_double();
    advertisement_log_period_sec_ = get_parameter("advertisement_log_period_sec").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();

    if (requested_mode == "odometry") {
        mode_ = Mode::kOdometry;
    } else if (requested_mode == "advertisement") {
        mode_ = Mode::kAdvertisement;
    } else {
        throw std::runtime_error("mode must be 'odometry' or 'advertisement'");
    }

    if (!std::isfinite(rate_hz_) || rate_hz_ <= 0.0) {
        throw std::runtime_error("rate_hz must be a finite value greater than 0");
    }

    const auto raw_hostname = util::system_hostname();
    hostname_ = util::sanitize_topic_suffix(raw_hostname.empty() ? "mrs-uav" : raw_hostname);

    auto configured_odometry_topic = get_parameter("odometry_topic").as_string();
    if (configured_odometry_topic.empty()) {
        configured_odometry_topic = kDefaultOdometryTopic;
    }
    odometry_topic_ = util::normalize_ros_topic(expand_hostname(configured_odometry_topic, hostname_));

    auto configured_advertisement_topic = get_parameter("advertisement_topic").as_string();
    if (configured_advertisement_topic.empty()) {
        configured_advertisement_topic = kDefaultAdvertisementTopic;
    }
    advertisement_topic_ = util::normalize_ros_topic(
        expand_hostname(configured_advertisement_topic, hostname_));

    auto configured_observe_topic = get_parameter("advertisement_observe_topic").as_string();
    if (configured_observe_topic.empty()) {
        configured_observe_topic = kDefaultAdvertisementObserveTopic;
    }
    advertisement_observe_topic_ = util::normalize_ros_topic(
        expand_hostname(configured_observe_topic, hostname_));

    auto configured_observe_topic_compat = get_parameter("advertisement_observe_topic_compat").as_string();
    if (configured_observe_topic_compat.empty()) {
        configured_observe_topic_compat = kDefaultAdvertisementObserveTopicCompat;
    }
    advertisement_observe_topic_compat_ = util::normalize_ros_topic(
        expand_hostname(configured_observe_topic_compat, hostname_));

    if (!std::isfinite(advertisement_log_period_sec_) || advertisement_log_period_sec_ <= 0.0) {
        throw std::runtime_error("advertisement_log_period_sec must be a finite value greater than 0");
    }
}

void TestNode::configure_publishers() {
    if (mode_ == Mode::kOdometry) {
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
        RCLCPP_INFO(get_logger(),
                    "Publishing dummy odometry at %.3f Hz on %s",
                    rate_hz_, odometry_topic_.c_str());
        return;
    }

    advertisement_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(advertisement_topic_, 10);
    RCLCPP_INFO(get_logger(),
                "Publishing 8-byte advertisement timestamps at %.3f Hz on %s",
                rate_hz_, advertisement_topic_.c_str());
}

void TestNode::configure_advertisement_watchers() {
    if (mode_ != Mode::kAdvertisement) {
        return;
    }

    const auto callback = [this](const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
        handle_advertisement_scan(message);
    };

    advertisement_scan_sub_ = create_subscription<mrs_uav_bluetooth::msg::BleDeviceArray>(
        advertisement_observe_topic_, rclcpp::QoS(10), callback);
    RCLCPP_INFO(get_logger(), "Monitoring advertisement device topic: %s",
                advertisement_observe_topic_.c_str());

    if (advertisement_observe_topic_compat_ != advertisement_observe_topic_) {
        advertisement_scan_sub_compat_ = create_subscription<mrs_uav_bluetooth::msg::BleDeviceArray>(
            advertisement_observe_topic_compat_, rclcpp::QoS(10), callback);
        RCLCPP_INFO(get_logger(), "Monitoring advertisement device topic (compat): %s",
                    advertisement_observe_topic_compat_.c_str());
    }
}

void TestNode::publish_once() {
    if (mode_ == Mode::kOdometry) {
        publish_odometry();
        return;
    }

    publish_advertisement_payload();
}

void TestNode::publish_odometry() {
    if (!odometry_pub_) {
        return;
    }

    nav_msgs::msg::Odometry message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.child_frame_id = child_frame_id_;

    message.pose.pose.position.x = sample_uniform(-50.0, 50.0);
    message.pose.pose.position.y = sample_uniform(-50.0, 50.0);
    message.pose.pose.position.z = sample_uniform(0.0, 20.0);

    const double yaw = sample_uniform(-kPi, kPi);
    message.pose.pose.orientation.z = std::sin(yaw / 2.0);
    message.pose.pose.orientation.w = std::cos(yaw / 2.0);

    message.twist.twist.linear.x = sample_uniform(-5.0, 5.0);
    message.twist.twist.linear.y = sample_uniform(-5.0, 5.0);
    message.twist.twist.linear.z = sample_uniform(-2.0, 2.0);

    message.twist.twist.angular.x = sample_uniform(-0.5, 0.5);
    message.twist.twist.angular.y = sample_uniform(-0.5, 0.5);
    message.twist.twist.angular.z = sample_uniform(-1.5, 1.5);

    odometry_pub_->publish(message);
}

void TestNode::publish_advertisement_payload() {
    if (!advertisement_pub_) {
        return;
    }

    const auto timestamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(sizeof(timestamp_ns));
    for (std::size_t index = 0; index < sizeof(timestamp_ns); ++index) {
        message.data[index] = static_cast<uint8_t>((timestamp_ns >> (8 * index)) & 0xFFu);
    }

    advertisement_pub_->publish(message);
}

void TestNode::handle_advertisement_scan(const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
    if (!message) {
        return;
    }

    std::vector<mrs_uav_bluetooth::msg::BleDevice> filtered_devices;
    filtered_devices.reserve(message->devices.size());
    for (const auto& device : message->devices) {
        if (has_non_empty_custom_data(device)) {
            filtered_devices.push_back(device);
        }
    }

    std::lock_guard<std::mutex> lock(observed_devices_mutex_);
    observed_devices_ = std::move(filtered_devices);
}

void TestNode::log_observed_advertisements() {
    std::vector<mrs_uav_bluetooth::msg::BleDevice> devices;
    {
        std::lock_guard<std::mutex> lock(observed_devices_mutex_);
        devices = observed_devices_;
    }

    if (devices.empty()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No devices with non-empty custom advertisement data observed on %s or %s",
                             advertisement_observe_topic_.c_str(),
                             advertisement_observe_topic_compat_.c_str());
        return;
    }

    std::ostringstream stream;
    stream << "Observed devices with custom advertisement data:";
    for (const auto& device : devices) {
        const auto matching_entries = matching_custom_data_entries(device);
        if (matching_entries.empty()) {
            continue;
        }

        stream << " [name='" << device.name << "' mac=" << device.mac;
        stream << " adv=";
        for (std::size_t index = 0; index < matching_entries.size(); ++index) {
            if (index != 0) {
                    stream << ",";
            }
            stream << matching_entries[index];
        }
        stream << "]";
    }
    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
}

bool TestNode::has_non_empty_custom_data(const mrs_uav_bluetooth::msg::BleDevice& device) const {
    return !matching_custom_data_entries(device).empty();
}

std::vector<std::string> TestNode::matching_custom_data_entries(
    const mrs_uav_bluetooth::msg::BleDevice& device) const {
    std::vector<std::string> matches;
    matches.reserve(device.advertising_data_hex.size());
    for (const auto& entry : device.advertising_data_hex) {
        if (entry.rfind("38:", 0) != 0) {
            continue;
        }
        if (entry.size() <= 3) {
            continue;
        }
        matches.push_back(entry);
    }
    return matches;
}

std::string TestNode::normalize_mode(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return value;
}

std::string TestNode::expand_hostname(std::string value, const std::string& hostname) {
    constexpr std::string_view token = "{hostname}";
    std::size_t position = 0;
    while ((position = value.find(token, position)) != std::string::npos) {
        value.replace(position, token.size(), hostname);
        position += hostname.size();
    }
    return value;
}

double TestNode::sample_uniform(double min, double max) {
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(random_engine_);
}

}  // namespace mrs_uav_bluetooth::app