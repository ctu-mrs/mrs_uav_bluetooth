// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/random_odometry_publisher_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <cmath>
#include <stdexcept>
#include <string_view>

namespace mrs_uav_bluetooth::app {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::string_view kHostnameToken = "{hostname}";

std::string expand_hostname(std::string value, const std::string& hostname) {
    std::size_t position = 0;
    while ((position = value.find(kHostnameToken, position)) != std::string::npos) {
        value.replace(position, kHostnameToken.size(), hostname);
        position += hostname.size();
    }
    return value;
}

void require_positive_finite(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(std::string(name) + " must be finite and greater than zero");
    }
}

}  // namespace

RandomOdometryPublisherNode::RandomOdometryPublisherNode()
    : rclcpp::Node("random_odometry_publisher") {
    configure_parameters();
    // Negative seed means non-deterministic demo data; any non-negative value
    // reproduces the same sequence for debugging and multi-run comparisons.
    random_engine_.seed(random_seed_ < 0
        ? std::random_device{}()
        : static_cast<std::mt19937::result_type>(random_seed_));
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, qos_depth_);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        [this]() { publish_odometry(); });
    RCLCPP_INFO(get_logger(), "Publishing random odometry at %.3f Hz on %s",
                rate_hz_, odometry_topic_.c_str());
}

void RandomOdometryPublisherNode::configure_parameters() {
    declare_parameter<std::string>(
        "odometry_topic", "/{hostname}/mavros/local_position/odom");
    declare_parameter<std::string>("frame_id", frame_id_);
    declare_parameter<std::string>("child_frame_id", child_frame_id_);
    declare_parameter<double>("rate_hz", rate_hz_);
    declare_parameter<double>("position_xy_limit", position_xy_limit_);
    declare_parameter<double>("position_z_min", position_z_min_);
    declare_parameter<double>("position_z_max", position_z_max_);
    declare_parameter<double>("roll_pitch_limit", roll_pitch_limit_);
    declare_parameter<double>("linear_xy_limit", linear_xy_limit_);
    declare_parameter<double>("linear_z_limit", linear_z_limit_);
    declare_parameter<double>("angular_xy_limit", angular_xy_limit_);
    declare_parameter<double>("angular_z_limit", angular_z_limit_);
    declare_parameter<int64_t>("random_seed", random_seed_);
    declare_parameter<int>("qos_depth", qos_depth_);

    // Resolve the token once so every publisher created by this process uses
    // the same hostname even if the surrounding environment later changes.
    const auto raw_hostname = util::system_hostname();
    const auto hostname = util::sanitize_topic_suffix(
        raw_hostname.empty() ? "mrs-uav" : raw_hostname);
    odometry_topic_ = util::normalize_ros_topic(expand_hostname(
        get_parameter("odometry_topic").as_string(), hostname));
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    rate_hz_ = get_parameter("rate_hz").as_double();
    position_xy_limit_ = get_parameter("position_xy_limit").as_double();
    position_z_min_ = get_parameter("position_z_min").as_double();
    position_z_max_ = get_parameter("position_z_max").as_double();
    roll_pitch_limit_ = get_parameter("roll_pitch_limit").as_double();
    linear_xy_limit_ = get_parameter("linear_xy_limit").as_double();
    linear_z_limit_ = get_parameter("linear_z_limit").as_double();
    angular_xy_limit_ = get_parameter("angular_xy_limit").as_double();
    angular_z_limit_ = get_parameter("angular_z_limit").as_double();
    random_seed_ = get_parameter("random_seed").as_int();
    qos_depth_ = get_parameter("qos_depth").as_int();

    require_positive_finite(rate_hz_, "rate_hz");
    require_positive_finite(position_xy_limit_, "position_xy_limit");
    require_positive_finite(roll_pitch_limit_, "roll_pitch_limit");
    require_positive_finite(linear_xy_limit_, "linear_xy_limit");
    require_positive_finite(linear_z_limit_, "linear_z_limit");
    require_positive_finite(angular_xy_limit_, "angular_xy_limit");
    require_positive_finite(angular_z_limit_, "angular_z_limit");
    if (!std::isfinite(position_z_min_) || !std::isfinite(position_z_max_) ||
        position_z_min_ > position_z_max_) {
        throw std::runtime_error(
            "position_z_min and position_z_max must be finite and ordered");
    }
    if (qos_depth_ <= 0) {
        throw std::runtime_error("qos_depth must be greater than zero");
    }
}

void RandomOdometryPublisherNode::publish_odometry() {
    nav_msgs::msg::Odometry message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.child_frame_id = child_frame_id_;
    message.pose.pose.position.x = sample(-position_xy_limit_, position_xy_limit_);
    message.pose.pose.position.y = sample(-position_xy_limit_, position_xy_limit_);
    message.pose.pose.position.z = sample(position_z_min_, position_z_max_);

    // Build a normalized quaternion from independently sampled intrinsic XYZ
    // Euler angles using the standard half-angle expansion.
    const double roll = sample(-roll_pitch_limit_, roll_pitch_limit_);
    const double pitch = sample(-roll_pitch_limit_, roll_pitch_limit_);
    const double yaw = sample(-kPi, kPi);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    message.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    message.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    message.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    message.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;

    // Independent components intentionally favor broad transport coverage over
    // physically continuous motion; this node is a data-path source, not a simulator.
    message.twist.twist.linear.x = sample(-linear_xy_limit_, linear_xy_limit_);
    message.twist.twist.linear.y = sample(-linear_xy_limit_, linear_xy_limit_);
    message.twist.twist.linear.z = sample(-linear_z_limit_, linear_z_limit_);
    message.twist.twist.angular.x = sample(-angular_xy_limit_, angular_xy_limit_);
    message.twist.twist.angular.y = sample(-angular_xy_limit_, angular_xy_limit_);
    message.twist.twist.angular.z = sample(-angular_z_limit_, angular_z_limit_);
    publisher_->publish(std::move(message));
}

double RandomOdometryPublisherNode::sample(double minimum, double maximum) {
    return std::uniform_real_distribution<double>(minimum, maximum)(random_engine_);
}

}  // namespace mrs_uav_bluetooth::app
