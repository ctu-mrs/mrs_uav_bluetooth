// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/user_overlay_node.hpp"

#include "mrs_uav_bluetooth/config/config_loader.hpp"
#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>

using namespace std::chrono_literals;

namespace mrs_uav_bluetooth::app {

UserOverlayNode::UserOverlayNode()
    : rclcpp::Node("mrs_uav_bluetooth_user") {
    configure_parameters();
    load_overlay_context();

    keepalive_pub_ = create_publisher<std_msgs::msg::Empty>(keepalive_topic_, 10);
    print_sub_ = create_subscription<std_msgs::msg::String>(
        print_topic_, 200,
        [this](const std_msgs::msg::String::SharedPtr message) { handle_print(message); });
    keepalive_timer_ = create_wall_timer(
        std::chrono::duration<double>(keepalive_publish_period_sec_),
        [this]() { publish_keepalive(); });
    set_active_config_client_ = create_client<mrs_uav_bluetooth::srv::SetActiveConfig>("ble/set_active_config");

    activate_overlay();
}

UserOverlayNode::~UserOverlayNode() {
    revert_overlay();
}

void UserOverlayNode::configure_parameters() {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("print_source", "status");
    declare_parameter<double>("service_wait_timeout_sec", 10.0);
    declare_parameter<double>("service_call_timeout_sec", 5.0);
    declare_parameter<double>("deactivate_service_wait_timeout_sec", 2.0);
    declare_parameter<std::string>("sentinel_topic_suffix", "overlay_keepalive");
    declare_parameter<double>("sentinel_publish_period_sec", 1.0);
    declare_parameter<double>("min_sentinel_publish_period_sec", 0.2);

    config_path_ = get_parameter("config_path").as_string();
    print_source_ = get_parameter("print_source").as_string();
    service_wait_timeout_sec_ = std::max(0.0, get_parameter("service_wait_timeout_sec").as_double());
    service_call_timeout_sec_ = std::max(0.0, get_parameter("service_call_timeout_sec").as_double());
    deactivate_service_wait_timeout_sec_ = std::max(0.0, get_parameter("deactivate_service_wait_timeout_sec").as_double());
    sentinel_topic_suffix_ = get_parameter("sentinel_topic_suffix").as_string();
    keepalive_publish_period_sec_ = std::max(0.0, get_parameter("sentinel_publish_period_sec").as_double());
    min_keepalive_publish_period_sec_ = std::max(0.0, get_parameter("min_sentinel_publish_period_sec").as_double());

    if (config_path_.empty()) {
        throw std::runtime_error("config_path parameter is required");
    }
    if (!std::filesystem::is_regular_file(config_path_)) {
        throw std::runtime_error("Overlay config file not found: " + config_path_);
    }

    std::transform(print_source_.begin(), print_source_.end(), print_source_.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    if (print_source_ != "status" && print_source_ != "log") {
        throw std::runtime_error("print_source must be 'status' or 'log'");
    }

    sentinel_topic_suffix_ = util::sanitize_topic_suffix(sentinel_topic_suffix_);
    if (sentinel_topic_suffix_.empty()) {
        throw std::runtime_error("sentinel_topic_suffix must not be empty");
    }

    keepalive_publish_period_sec_ = std::max(min_keepalive_publish_period_sec_, keepalive_publish_period_sec_);
    if (keepalive_publish_period_sec_ <= 0.0) {
        throw std::runtime_error("sentinel_publish_period_sec must be > 0");
    }
}

void UserOverlayNode::load_overlay_context() {
    hostname_ = mrs_uav_bluetooth::util::sanitize_topic_suffix(
        mrs_uav_bluetooth::util::system_hostname().empty() ? "mrs-uav" : mrs_uav_bluetooth::util::system_hostname());
    const auto share_dir = ament_index_cpp::get_package_share_directory("mrs_uav_bluetooth");
    default_config_path_ = share_dir + "/config/default.yaml";
    keepalive_topic_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/ble/" + sentinel_topic_suffix_);
    print_topic_ = mrs_uav_bluetooth::util::normalize_ros_topic(
        "/" + hostname_ + "/ble/" + print_source_);
}

void UserOverlayNode::activate_overlay() {
    if (!set_active_config_client_->wait_for_service(std::chrono::duration<double>(service_wait_timeout_sec_))) {
        throw std::runtime_error("ble/set_active_config service is not available");
    }
    auto response = call_config_service(config_path_);
    if (!response || !response->success) {
        throw std::runtime_error(response ? response->message : "No response from ble/set_active_config");
    }
    RCLCPP_INFO(get_logger(), "Activated BLE overlay config: %s", response->active_config_path.c_str());
    RCLCPP_INFO(get_logger(), "Printing bluetooth service %s topic: %s", print_source_.c_str(), print_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing overlay keep-alive sentinel on: %s", keepalive_topic_.c_str());
}

void UserOverlayNode::revert_overlay() {
    if (!set_active_config_client_) {
        return;
    }
    // Guard against calling into ROS after context shutdown (Ctrl-C).
    if (!rclcpp::ok()) {
        return;
    }
    if (!set_active_config_client_->service_is_ready() &&
        !set_active_config_client_->wait_for_service(std::chrono::duration<double>(deactivate_service_wait_timeout_sec_))) {
        return;
    }
    if (!rclcpp::ok()) {
        return;
    }
    auto response = call_config_service("");
    if (response && response->success) {
        RCLCPP_INFO(get_logger(), "Reverted bluetooth service to default config");
    }
}

void UserOverlayNode::publish_keepalive() {
    std_msgs::msg::Empty msg;
    keepalive_pub_->publish(msg);
}

mrs_uav_bluetooth::srv::SetActiveConfig::Response::SharedPtr
UserOverlayNode::call_config_service(const std::string& config_path) {
    if (!rclcpp::ok()) {
        return nullptr;
    }
    auto request = std::make_shared<mrs_uav_bluetooth::srv::SetActiveConfig::Request>();
    request->config_path = config_path;
    request->hold_seconds = 0.0;
    auto future = set_active_config_client_->async_send_request(request);
    if (!rclcpp::ok()) {
        return nullptr;
    }
    const auto result = rclcpp::spin_until_future_complete(
        get_node_base_interface(), future, std::chrono::duration<double>(service_call_timeout_sec_));
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
    }
    return future.get();
}

void UserOverlayNode::handle_print(const std_msgs::msg::String::SharedPtr message) {
    RCLCPP_INFO(get_logger(), "%s", message->data.c_str());
}

}  // namespace mrs_uav_bluetooth::app
