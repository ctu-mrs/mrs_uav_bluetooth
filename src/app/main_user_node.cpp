// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/main_user_node.cpp
/// \brief Implements the main user node component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/user_overlay_node.hpp"

#include <rclcpp/rclcpp.hpp>

/// \brief Run the overlay lease client until ROS shutdown.
/// \param argc number of command-line arguments.
/// \param argv command-line argument vector.
/// \return Zero on success, or a nonzero process status on failure.
int main(int argc, char** argv) {
    // Keep the overlay lease alive for the lifetime of this single ROS node.
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::UserOverlayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
