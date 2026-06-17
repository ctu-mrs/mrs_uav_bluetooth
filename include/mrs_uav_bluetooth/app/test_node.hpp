// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/msg/ble_device_array.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::app {

class TestNode : public rclcpp::Node {
public:
    TestNode();

private:
    enum class Mode {
        kOdometry,
        kAdvertisement,
    };

    void configure_parameters();
    void configure_publishers();
    void configure_advertisement_watchers();
    void configure_advertisement_odometry_subscription();
    void publish_once();
    void publish_odometry();
    void publish_advertisement_payload();
    void handle_local_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
    std::optional<nav_msgs::msg::Odometry> fresh_local_odometry();
    void handle_advertisement_scan(const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message);
    void log_decoded_advertisement(const mrs_uav_bluetooth::msg::BleDevice& device,
                                   const std::vector<uint8_t>& data,
                                   uint64_t local_time_ns);
    void publish_decoded_peer_odometry(const mrs_uav_bluetooth::msg::BleDevice& device,
                                       uint64_t stamp_ns,
                                       const std::vector<float>& values);
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr peer_odometry_publisher(
        const mrs_uav_bluetooth::msg::BleDevice& device);

    static std::string normalize_mode(std::string value);
    static std::string expand_hostname(std::string value, const std::string& hostname);
    static std::optional<uint64_t> decode_microsecond_timestamp_ns(const std::vector<uint8_t>& data);
    static bool decode_fixed_odometry_payload(const std::vector<uint8_t>& data,
                                              uint64_t& stamp_ns,
                                              std::vector<float>& values);
    static void append_microsecond_timestamp(std::vector<uint8_t>& data, uint64_t timestamp_ns);
    static uint64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time& stamp);
    static builtin_interfaces::msg::Time nanoseconds_to_stamp(uint64_t stamp_ns);
    static std::string format_system_time(uint64_t timestamp_ns);
    static std::string device_topic_token(const mrs_uav_bluetooth::msg::BleDevice& device);

    double sample_uniform(double min, double max);

    Mode mode_{Mode::kOdometry};
    std::string hostname_;
    std::string odometry_topic_;
    std::string advertisement_topic_;
    std::string advertisement_observe_topic_;
    std::string peer_topic_prefix_;
    std::string frame_id_{"map"};
    std::string child_frame_id_{"base_link"};
    double rate_hz_{1.0};
    double odometry_timeout_sec_{2.5};

    std::mt19937 random_engine_;
    std::mutex observed_devices_mutex_;
    std::mutex local_odometry_mutex_;
    std::optional<nav_msgs::msg::Odometry> latest_local_odometry_;
    std::optional<std::chrono::steady_clock::time_point> latest_local_odometry_received_;
    std::map<std::string, std::vector<uint8_t>> last_logged_advertisements_;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> peer_odometry_publishers_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr advertisement_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr advertisement_odometry_sub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr advertisement_scan_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mrs_uav_bluetooth::app
