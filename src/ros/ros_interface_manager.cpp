// SPDX-License-Identifier: BSD-3-Clause
/// \file src/ros/ros_interface_manager.cpp
/// \brief Implements the ros interface manager component of the ROS 2 interface layer.

#include "mrs_uav_bluetooth/ros/ros_interface_manager.hpp"

namespace mrs_uav_bluetooth::ros {

RosInterfaceManager::RosInterfaceManager(rclcpp::Node& node) {
    // Construct the status publishers and service factory that share this ROS node.
    status_publisher_ = std::make_unique<StatusPublisher>(node);
    service_servers_ = std::make_unique<ServiceServers>(node);
}

}  // namespace mrs_uav_bluetooth::ros
