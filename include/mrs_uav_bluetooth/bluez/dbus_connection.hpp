// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bluez/dbus_connection.hpp
/// \brief Declares the dbus connection component of the BlueZ system-D-Bus integration layer.

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
    /// \param logger ROS logger used for diagnostics.
    /// \param role short connection role included in D-Bus lifecycle logs.
    /// \param requested_name optional well-known D-Bus name requested for this connection.
    explicit DbusConnection(rclcpp::Logger logger,
                            std::string role = "main",
                            std::string requested_name = {});

    /// Gracefully leave the event loop and release the connection.
    ~DbusConnection();

    /// \brief Disable copying of the D-Bus connection.
    DbusConnection(const DbusConnection&) = delete;
    /// \brief Disable copy assignment of the D-Bus connection.
    DbusConnection& operator=(const DbusConnection&) = delete;

    /// The shared system-bus connection.  Never null after construction.
    /// \return Live system-bus connection owned by this wrapper.
    sdbus::IConnection& connection() {
        // Expose the live system-bus connection and its background event loop.
        return *connection_;
    }

    /// Discover the first adapter object path (e.g. "/org/bluez/hci0").
    /// Returns empty string if no adapter is found.
    /// \return First Adapter1 object path, or an empty string when none exists.
    std::string find_adapter_path() const;

private:
    rclcpp::Logger logger_;
    std::string role_;
    std::string requested_name_;
    std::unique_ptr<sdbus::IConnection> connection_;
};

}  // namespace mrs_uav_bluetooth::bluez
