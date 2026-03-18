// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/ros/ros_interface_manager.hpp"

namespace mrs_uav_bluetooth::ros {

RosInterfaceManager::RosInterfaceManager(rclcpp::Node& node) {
    status_publisher_ = std::make_unique<StatusPublisher>(node);
    service_servers_ = std::make_unique<ServiceServers>(node);
}

}  // namespace mrs_uav_bluetooth::ros
