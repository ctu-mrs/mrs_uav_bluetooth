// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>

namespace mrs_uav_bluetooth::bluez {

DbusConnection::DbusConnection(rclcpp::Logger logger,
                               std::string role,
                               std::string requested_name)
    : logger_(logger),
      role_(std::move(role)),
      requested_name_(std::move(requested_name))
{
    RCLCPP_INFO(logger_, "Opening system D-Bus connection (%s)", role_.c_str());
    connection_ = sdbus::createSystemBusConnection();
    if (!requested_name_.empty()) {
        RCLCPP_INFO(logger_, "Requesting system D-Bus name %s (%s)",
                    requested_name_.c_str(), role_.c_str());
        try {
            connection_->requestName(sdbus::ServiceName{requested_name_});
        } catch (const sdbus::Error& error) {
            const auto message = error.getMessage();
            if (message.find("AccessDenied") != std::string::npos ||
                message.find("Permission denied") != std::string::npos) {
                RCLCPP_WARN(
                    logger_,
                    "Failed to request system D-Bus name %s (%s): %s. Install the mrs-uav-bluetooth-service package to add the required D-Bus policy file under /etc/dbus-1/system.d.",
                    requested_name_.c_str(), role_.c_str(), message.c_str());
            } else {
                RCLCPP_WARN(logger_, "Failed to request system D-Bus name %s (%s): %s",
                            requested_name_.c_str(), role_.c_str(), message.c_str());
            }
        }
    }
    // Start the event loop on a background thread.
    connection_->enterEventLoopAsync();
    RCLCPP_INFO(logger_, "D-Bus event loop started (%s)", role_.c_str());
}

DbusConnection::~DbusConnection() {
    if (connection_) {
        RCLCPP_INFO(logger_, "Leaving D-Bus event loop (%s)", role_.c_str());
        connection_->leaveEventLoop();
    }
}

std::string DbusConnection::find_adapter_path() const {
    try {
        auto proxy = sdbus::createProxy(*connection_,
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{"/"});

        // Call ObjectManager.GetManagedObjects on the root BlueZ path.
        sdbus::ObjectPath root{"/"};
        std::map<sdbus::ObjectPath,
                 std::map<std::string, std::map<std::string, sdbus::Variant>>> objects;

        proxy->callMethod("GetManagedObjects")
             .onInterface(std::string(kDbusObjectManagerIface))
             .storeResultsTo(objects);

        for (const auto& [path, ifaces] : objects) {
            if (ifaces.count(std::string(kAdapterIface))) {
                RCLCPP_INFO(logger_, "Found BlueZ adapter: %s",
                            std::string(path).c_str());
                return std::string(path);
            }
        }
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "Failed to enumerate BlueZ adapters: %s", e.what());
    }
    return {};
}

}  // namespace mrs_uav_bluetooth::bluez
