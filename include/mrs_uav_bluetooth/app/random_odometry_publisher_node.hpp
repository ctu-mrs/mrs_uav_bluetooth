// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <random>
#include <string>

namespace mrs_uav_bluetooth::app {

/// Optional synthetic odometry source for bridge testing.
///
/// The node publishes bounded, independent random pose and twist samples so a
/// user can exercise an overlay, advertisement exchange, or Mesh exchange when
/// no estimator is available. A fixed `random_seed` makes runs reproducible.
class RandomOdometryPublisherNode : public rclcpp::Node {
public:
    /// Declare/validate parameters and begin periodic publication.
    RandomOdometryPublisherNode();

private:
    /// Load parameters, expand `{hostname}`, and reject unsafe ranges.
    void configure_parameters();
    /// Construct and publish one complete odometry sample.
    void publish_odometry();
    /// Draw a uniformly distributed value from the configured PRNG.
    /// @param minimum Inclusive lower bound.
    /// @param maximum Inclusive upper bound as defined by the distribution.
    double sample(double minimum, double maximum);

    // Validated ROS interface and sample-distribution parameters.
    std::string odometry_topic_;
    std::string frame_id_{"map"};
    std::string child_frame_id_{"base_link"};
    double rate_hz_{10.0};
    double position_xy_limit_{50.0};
    double position_z_min_{0.0};
    double position_z_max_{20.0};
    double roll_pitch_limit_{0.35};
    double linear_xy_limit_{5.0};
    double linear_z_limit_{2.0};
    double angular_xy_limit_{0.5};
    double angular_z_limit_{1.5};
    int64_t random_seed_{-1};
    int qos_depth_{10};

    // Runtime objects are created only after all parameter checks pass.
    std::mt19937 random_engine_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mrs_uav_bluetooth::app
