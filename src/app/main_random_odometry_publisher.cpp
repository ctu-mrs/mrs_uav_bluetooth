// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/main_random_odometry_publisher.cpp
/// \brief Implements the main random odometry publisher component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/random_odometry_publisher_node.hpp"

#include <rclcpp/rclcpp.hpp>

/// \brief Run the configurable synthetic odometry publisher until ROS shutdown.
/// \param argc number of command-line arguments.
/// \param argv command-line argument vector.
/// \return Zero on success, or a nonzero process status on failure.
int main(int argc, char** argv) {
    // Publish deterministic-rate random odometry until ROS requests shutdown.
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::RandomOdometryPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
