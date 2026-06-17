// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/msg/ble_device_array.hpp"

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
    void publish_once();
    void publish_odometry();
    void publish_advertisement_payload();
    void handle_advertisement_scan(const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message);
    void log_decoded_advertisement(const mrs_uav_bluetooth::msg::BleDevice& device,
                                   const std::vector<uint8_t>& data,
                                   uint64_t local_time_ns) const;

    static std::string normalize_mode(std::string value);
    static std::string expand_hostname(std::string value, const std::string& hostname);
    static std::optional<uint64_t> decode_little_endian_uint64(const std::vector<uint8_t>& data);
    static std::string format_system_time(uint64_t timestamp_ns);

    double sample_uniform(double min, double max);

    Mode mode_{Mode::kOdometry};
    std::string hostname_;
    std::string odometry_topic_;
    std::string advertisement_topic_;
    std::string advertisement_observe_topic_;
    std::string advertisement_observe_topic_compat_;
    std::string frame_id_{"map"};
    std::string child_frame_id_{"base_link"};
    double rate_hz_{10.0};
    double advertisement_log_period_sec_{2.0};

    std::mt19937 random_engine_;
    std::mutex observed_devices_mutex_;
    std::map<std::string, std::vector<uint8_t>> last_logged_advertisements_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr advertisement_pub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr advertisement_scan_sub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr advertisement_scan_sub_compat_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mrs_uav_bluetooth::app
