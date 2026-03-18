// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mrs_uav_bluetooth::bluez {

/// Controls the local BlueZ adapter: power, discoverable, discovery filter.
class AdapterController {
public:
    AdapterController(DbusConnection& dbus, const std::string& adapter_path,
                      rclcpp::Logger logger);

    /// Ensure the adapter is powered on.
    void power_on();

    /// Set the Discoverable property and timeout.
    void set_discoverable(bool discoverable, uint32_t timeout = 0);

    /// Set the Pairable property.
    void set_pairable(bool pairable);

    /// Start LE discovery with an optional transport filter (le, bredr, auto).
    void start_discovery(const std::string& transport = "le");

    /// Stop discovery.
    void stop_discovery();

    /// Remove a device by its object path.
    void remove_device(const std::string& device_path);

    const std::string& adapter_path() const { return adapter_path_; }

private:
    void set_adapter_property(const std::string& name, const sdbus::Variant& value);

    DbusConnection& dbus_;
    std::string adapter_path_;
    rclcpp::Logger logger_;
    std::unique_ptr<sdbus::IProxy> proxy_;
};

}  // namespace mrs_uav_bluetooth::bluez
