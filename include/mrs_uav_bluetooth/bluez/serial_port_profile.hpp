// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bluez/serial_port_profile.hpp
/// \brief Declares the serial port profile component of the BlueZ system-D-Bus integration layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace mrs_uav_bluetooth::bluez {

/// Selects whether the local BlueZ profile accepts or initiates RFCOMM links.
enum class ProfileRole {
    Client,
    Server,
};

/// Registration parameters passed to BlueZ ProfileManager1.
struct SerialPortProfileOptions {
    ProfileRole role{ProfileRole::Server};
    uint16_t channel{22};
    bool require_authentication{true};
    bool require_authorization{false};
    bool auto_connect{false};
    std::string name{"MRS UAV Serial Port"};
};

/// Implements org.bluez.Profile1 and registers the Bluetooth Serial Port
/// Profile with BlueZ's ProfileManager1. Ownership of a NewConnection file
/// descriptor is transferred to the installed connection handler before it is
/// invoked; the handler must close it on every success and failure path.
class SerialPortProfile {
public:
    using ConnectionHandler = std::function<void(
        const std::string& device_path,
        int socket_fd,
        const std::map<std::string, sdbus::Variant>& properties)>;
    using DisconnectionHandler = std::function<void(const std::string& device_path)>;
    using ReleaseHandler = std::function<void()>;

    /// \brief Define the RFCOMM Profile1 object and registration options.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param object_path D-Bus path where the serial profile handler is exported.
    /// \param logger ROS logger used for diagnostics.
    /// \param options Profile UUID channel and authorization behavior.
    SerialPortProfile(DbusConnection& dbus,
                      std::string object_path,
                      rclcpp::Logger logger,
                      SerialPortProfileOptions options = {});
    /// \brief Unregister the RFCOMM profile before releasing D-Bus.
    ~SerialPortProfile();

    /// \brief Disable copying of the serial port profile.
    SerialPortProfile(const SerialPortProfile&) = delete;
    /// \brief Disable copy assignment of the serial port profile.
    SerialPortProfile& operator=(const SerialPortProfile&) = delete;

    /// \brief Install the owner that accepts each authorized RFCOMM file descriptor.
    /// \param handler Handler receiving an accepted serial-profile file descriptor.
    void set_connection_handler(ConnectionHandler handler);
    /// \brief Install the observer notified after BlueZ closes a peer profile connection.
    /// \param handler Handler notified when a serial-profile device disconnects.
    void set_disconnection_handler(DisconnectionHandler handler);
    /// \brief Install the observer notified when BlueZ releases this profile object.
    /// \param handler Handler invoked when BlueZ unregisters the profile.
    void set_release_handler(ReleaseHandler handler);

    /// \brief Export Profile1 and ask BlueZ to accept incoming RFCOMM connections.
    void register_profile();
    /// \brief Remove the BlueZ profile registration while preserving idempotent shutdown.
    void unregister_profile();

    /// \brief Return whether the D-Bus object is registered.
    /// \return True when BlueZ has accepted the registration; otherwise false.
    bool registered() const {
        // Report whether ProfileManager1 accepted this registration.
        return registered_;
    }
    /// \brief Return the D-Bus path exporting this RFCOMM Profile1 object.
    /// \return Registered D-Bus object path.
    const std::string& path() const {
        // Return the D-Bus path exported for this Profile1 object.
        return path_;
    }
    /// \brief Return the registered profile options.
    /// \return RFCOMM UUID role channel and authorization options registered with BlueZ.
    const SerialPortProfileOptions& options() const {
        // Return the RFCOMM role, channel, and authorization options registered with BlueZ.
        return options_;
    }

private:
    /// \brief Export the Profile1 connection, disconnection, and release methods.
    void export_object();

    DbusConnection& dbus_;
    std::string path_;
    rclcpp::Logger logger_;
    SerialPortProfileOptions options_;
    ConnectionHandler connection_handler_;
    DisconnectionHandler disconnection_handler_;
    ReleaseHandler release_handler_;
    std::unique_ptr<sdbus::IObject> exported_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::bluez
