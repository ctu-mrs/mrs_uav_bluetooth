// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <random>
#include <string>

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
    void publish_once();
    void publish_odometry();
    void publish_advertisement_payload();

    static std::string normalize_mode(std::string value);
    static std::string expand_hostname(std::string value, const std::string& hostname);

    double sample_uniform(double min, double max);

    Mode mode_{Mode::kOdometry};
    std::string hostname_;
    std::string odometry_topic_;
    std::string advertisement_topic_;
    std::string frame_id_{"map"};
    std::string child_frame_id_{"base_link"};
    double rate_hz_{10.0};

    std::mt19937 random_engine_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr advertisement_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mrs_uav_bluetooth::app