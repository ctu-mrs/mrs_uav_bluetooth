// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/srv/set_active_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace mrs_uav_bluetooth::app {

class UserOverlayNode : public rclcpp::Node {
public:
    UserOverlayNode();
    ~UserOverlayNode() override;

private:
    void configure_parameters();
    void load_overlay_context();
    void activate_overlay();
    void revert_overlay();
    void publish_keepalive();
    mrs_uav_bluetooth::srv::SetActiveConfig::Response::SharedPtr call_config_service(const std::string& config_path);
    void handle_print(const std_msgs::msg::String::SharedPtr message);

    std::string config_path_;
    std::string keepalive_topic_;
    std::string default_config_path_;
    std::string hostname_;
    std::string print_source_;
    std::string print_topic_;
    std::string sentinel_topic_suffix_;
    double service_wait_timeout_sec_{10.0};
    double service_call_timeout_sec_{5.0};
    double deactivate_service_wait_timeout_sec_{2.0};
    double keepalive_publish_period_sec_{1.0};
    double min_keepalive_publish_period_sec_{0.2};
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr keepalive_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr print_sub_;
    rclcpp::TimerBase::SharedPtr keepalive_timer_;
    rclcpp::Client<mrs_uav_bluetooth::srv::SetActiveConfig>::SharedPtr set_active_config_client_;
};

}  // namespace mrs_uav_bluetooth::app
