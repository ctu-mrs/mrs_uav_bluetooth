// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bluez/serial_port_profile.cpp
/// \brief Implements the serial port profile component of the BlueZ system-D-Bus integration layer.

#include "mrs_uav_bluetooth/bluez/serial_port_profile.hpp"

#include <stdexcept>
#include <utility>

namespace mrs_uav_bluetooth::bluez {

SerialPortProfile::SerialPortProfile(DbusConnection& dbus,
                                     std::string object_path,
                                     rclcpp::Logger logger,
                                     SerialPortProfileOptions options)
    : dbus_(dbus),
      path_(std::move(object_path)),
      logger_(logger),
      options_(std::move(options)) {
          // Retain the RFCOMM profile identity and options until explicit BlueZ registration.
      }

SerialPortProfile::~SerialPortProfile() {
    // Unregister the RFCOMM profile while the D-Bus manager remains available.
    try {
        unregister_profile();
    } catch (...) {
    }
}

void SerialPortProfile::set_connection_handler(ConnectionHandler handler) {
    // Install the owner that accepts and takes ownership of new RFCOMM sockets.
    connection_handler_ = std::move(handler);
}

void SerialPortProfile::set_disconnection_handler(DisconnectionHandler handler) {
    // Install the owner notified when BlueZ requests a peer link shutdown.
    disconnection_handler_ = std::move(handler);
}

void SerialPortProfile::set_release_handler(ReleaseHandler handler) {
    // Install the owner notified when BlueZ drops the whole profile registration.
    release_handler_ = std::move(handler);
}

void SerialPortProfile::export_object() {
    // Export Profile1 connection, disconnection, and release methods at the configured path.
    exported_ = sdbus::createObject(dbus_.connection(), sdbus::ObjectPath{path_});
    exported_->addVTable(
        sdbus::registerMethod("Release")
            .implementedAs([this]() {
                // Mark the profile unregistered and let the owner rebuild it after BlueZ releases it.
                registered_ = false;
                RCLCPP_INFO(logger_, "BlueZ released serial profile %s", path_.c_str());
                if (release_handler_) {
                    release_handler_();
                }
            }),
        sdbus::registerMethod("NewConnection")
            .withInputParamNames("device", "fd", "fd_properties")
            .implementedAs(
                [this](const sdbus::ObjectPath& device,
                       sdbus::UnixFd fd,
                       const std::map<std::string, sdbus::Variant>& properties) {
                    // Transfer ownership of the accepted RFCOMM file descriptor to the local link manager.
                    const auto device_path = static_cast<std::string>(device);
                    if (!fd.isValid() || !connection_handler_) {
                        throw sdbus::Error(
                            sdbus::Error::Name{"org.bluez.Error.Rejected"},
                            "Serial connection has no local consumer");
                    }

                    const int owned_fd = fd.release();
                    try {
                        connection_handler_(device_path, owned_fd, properties);
                    } catch (const std::exception& error) {
                        RCLCPP_WARN(logger_, "Rejected serial connection from %s: %s",
                                    device_path.c_str(), error.what());
                        throw sdbus::Error(
                            sdbus::Error::Name{"org.bluez.Error.Rejected"},
                            error.what());
                    } catch (...) {
                        throw sdbus::Error(
                            sdbus::Error::Name{"org.bluez.Error.Rejected"},
                            "Serial connection setup failed");
                    }
                }),
        sdbus::registerMethod("RequestDisconnection")
            .withInputParamNames("device")
            .implementedAs([this](const sdbus::ObjectPath& device) {
                // Ask the local link manager to close the RFCOMM connection for this device.
                const auto device_path = static_cast<std::string>(device);
                if (disconnection_handler_) {
                    disconnection_handler_(device_path);
                }
            })
    ).forInterface(std::string(kProfileIface));
}

void SerialPortProfile::register_profile() {
    // Register profile.
    if (registered_) {
        return;
    }
    if (options_.channel == 0 || options_.channel > 30) {
        throw std::invalid_argument("RFCOMM channel must be in the range 1..30");
    }

    export_object();

    std::map<std::string, sdbus::Variant> profile_options;
    profile_options["Name"] = sdbus::Variant{options_.name};
    profile_options["Service"] = sdbus::Variant{std::string(kSerialPortProfileUuid)};
    profile_options["Role"] = sdbus::Variant{
        options_.role == ProfileRole::Server ? std::string{"server"} : std::string{"client"}};
    profile_options["Channel"] = sdbus::Variant{options_.channel};
    profile_options["RequireAuthentication"] =
        sdbus::Variant{options_.require_authentication};
    profile_options["RequireAuthorization"] =
        sdbus::Variant{options_.require_authorization};
    profile_options["AutoConnect"] = sdbus::Variant{options_.auto_connect};

    try {
        auto manager = sdbus::createProxy(
            dbus_.connection(),
            sdbus::ServiceName{std::string(kBluezServiceName)},
            sdbus::ObjectPath{"/org/bluez"});
        manager->callMethod("RegisterProfile")
            .onInterface(std::string(kProfileManagerIface))
            .withArguments(
                sdbus::ObjectPath{path_},
                std::string{kSerialPortProfileUuid},
                profile_options);
        registered_ = true;
        RCLCPP_INFO(logger_, "Registered %s serial profile at %s on RFCOMM channel %u",
                    options_.role == ProfileRole::Server ? "server" : "client",
                    path_.c_str(), options_.channel);
    } catch (...) {
        exported_.reset();
        throw;
    }
}

void SerialPortProfile::unregister_profile() {
    // Unregister profile.
    if (registered_) {
        try {
            auto manager = sdbus::createProxy(
                dbus_.connection(),
                sdbus::ServiceName{std::string(kBluezServiceName)},
                sdbus::ObjectPath{"/org/bluez"});
            manager->callMethod("UnregisterProfile")
                .onInterface(std::string(kProfileManagerIface))
                .withArguments(sdbus::ObjectPath{path_});
        } catch (const sdbus::Error& error) {
            const auto message = error.getMessage();
            if (message.find("DoesNotExist") == std::string::npos) {
                RCLCPP_WARN(logger_, "Failed to unregister serial profile: %s",
                            message.c_str());
            }
        }
    }
    registered_ = false;
    exported_.reset();
}

}  // namespace mrs_uav_bluetooth::bluez
