// SPDX-License-Identifier: BSD-3-Clause
/// \file src/app/main_service_node.cpp
/// \brief Implements the main service node component of the ROS 2 application and operator-tool layer.

#include "mrs_uav_bluetooth/app/service_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

/// \brief Run the Bluetooth service node on its multithreaded ROS executor.
/// \param argc number of command-line arguments.
/// \param argv command-line argument vector.
/// \return Zero on success, or a nonzero process status on failure.
int main(int argc, char** argv) {
    // Keep lease and reliability timers responsive while worker threads wait on D-Bus.
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mrs_uav_bluetooth::app::ServiceNode>();
    // Six workers cover concurrent lease, peer, transport, and status callbacks
    // while limiting wakeups on small UAV CPUs.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 6);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}
