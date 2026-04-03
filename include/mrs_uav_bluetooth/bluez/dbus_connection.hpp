// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <sdbus-c++/sdbus-c++.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace mrs_uav_bluetooth::bluez {

/// Owns the primary system-bus connection and runs the sdbus-c++ event loop
/// on a background thread.  All BlueZ proxy and server objects should share
/// this connection.
class DbusConnection {
public:
    /// Open a system bus connection and start the async event loop.
    explicit DbusConnection(rclcpp::Logger logger,
                            std::string role = "main",
                            std::string requested_name = {});

    /// Gracefully leave the event loop and release the connection.
    ~DbusConnection();

    DbusConnection(const DbusConnection&) = delete;
    DbusConnection& operator=(const DbusConnection&) = delete;

    /// The shared system-bus connection.  Never null after construction.
    sdbus::IConnection& connection() { return *connection_; }

    /// Discover the first adapter object path (e.g. "/org/bluez/hci0").
    /// Returns empty string if no adapter is found.
    std::string find_adapter_path() const;

private:
    rclcpp::Logger logger_;
    std::string role_;
    std::string requested_name_;
    std::unique_ptr<sdbus::IConnection> connection_;
};

}  // namespace mrs_uav_bluetooth::bluez
