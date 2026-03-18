// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>

namespace mrs_uav_bluetooth::bluez {

DbusConnection::DbusConnection(rclcpp::Logger logger)
    : logger_(logger)
{
    RCLCPP_INFO(logger_, "Opening system D-Bus connection");
    connection_ = sdbus::createSystemBusConnection();
    // Start the event loop on a background thread.
    connection_->enterEventLoopAsync();
    RCLCPP_INFO(logger_, "D-Bus event loop started");
}

DbusConnection::~DbusConnection() {
    if (connection_) {
        RCLCPP_INFO(logger_, "Leaving D-Bus event loop");
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
