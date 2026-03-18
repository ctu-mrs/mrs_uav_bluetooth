// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/ros/service_servers.hpp"
#include "mrs_uav_bluetooth/ros/status_publisher.hpp"

#include <memory>

namespace mrs_uav_bluetooth::ros {

class RosInterfaceManager {
public:
    explicit RosInterfaceManager(rclcpp::Node& node);

    StatusPublisher& status_publisher() { return *status_publisher_; }
    ServiceServers& service_servers() { return *service_servers_; }

private:
    std::unique_ptr<StatusPublisher> status_publisher_;
    std::unique_ptr<ServiceServers> service_servers_;
};

}  // namespace mrs_uav_bluetooth::ros
