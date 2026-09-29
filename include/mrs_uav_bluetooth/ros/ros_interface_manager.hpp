// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/ros/ros_interface_manager.hpp
/// \brief Declares the ros interface manager component of the ROS 2 interface layer.

#pragma once

#include "mrs_uav_bluetooth/ros/service_servers.hpp"
#include "mrs_uav_bluetooth/ros/status_publisher.hpp"

#include <memory>

namespace mrs_uav_bluetooth::ros {

/// Aggregates ROS service servers and status publishers for the system node.
class RosInterfaceManager {
public:
    /// \brief Own the public status publishers and service factories for the node.
    /// \param node ROS node that owns the created interfaces.
    explicit RosInterfaceManager(rclcpp::Node& node);

    /// \brief Access the shared public status publisher bundle.
    /// \return Publisher used for the aggregate Bluetooth status topic.
    StatusPublisher& status_publisher() {
        // Return the status publisher bundle shared by the service node.
        return *status_publisher_;
    }
    /// \brief Expose the factory that owns the Bluetooth ROS service endpoints.
    /// \return Owner of this node's Bluetooth control services.
    ServiceServers& service_servers() {
        // Expose the factory that owns the Bluetooth ROS service endpoints.
        return *service_servers_;
    }

private:
    std::unique_ptr<StatusPublisher> status_publisher_;
    std::unique_ptr<ServiceServers> service_servers_;
};

}  // namespace mrs_uav_bluetooth::ros
