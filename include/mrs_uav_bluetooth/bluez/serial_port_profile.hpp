// SPDX-License-Identifier: BSD-3-Clause
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

enum class ProfileRole {
    Client,
    Server,
};

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

    SerialPortProfile(DbusConnection& dbus,
                      std::string object_path,
                      rclcpp::Logger logger,
                      SerialPortProfileOptions options = {});
    ~SerialPortProfile();

    SerialPortProfile(const SerialPortProfile&) = delete;
    SerialPortProfile& operator=(const SerialPortProfile&) = delete;

    void set_connection_handler(ConnectionHandler handler);
    void set_disconnection_handler(DisconnectionHandler handler);
    void set_release_handler(ReleaseHandler handler);

    void register_profile();
    void unregister_profile();

    bool registered() const { return registered_; }
    const std::string& path() const { return path_; }
    const SerialPortProfileOptions& options() const { return options_; }

private:
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
