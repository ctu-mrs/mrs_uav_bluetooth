// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bluez/adapter_controller.hpp
/// \brief Declares the adapter controller component of the BlueZ system-D-Bus integration layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mrs_uav_bluetooth::bluez {

/// Controls the local BlueZ adapter: power, discoverable, discovery filter.
class AdapterController {
public:
    /// \brief Bind adapter property changes to one Adapter1 object path.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param adapter_path BlueZ Adapter1 object path used by this controller.
    /// \param logger ROS logger used for diagnostics.
    AdapterController(DbusConnection& dbus, const std::string& adapter_path,
                      rclcpp::Logger logger);

    /// Ensure the adapter is powered on.
    void power_on();

    /// Power down the adapter before a radio-mode handoff that needs a clean
    /// controller state. Bonds and persistent BlueZ device records remain.
    void power_off();

    /// Set the Discoverable property and timeout.
    /// \param discoverable whether scanning peers may discover the adapter or advertisement.
    /// \param timeout BlueZ discoverable timeout in seconds; zero leaves it unlimited.
    void set_discoverable(bool discoverable, uint32_t timeout = 0);

    /// Set the Connectable property.
    /// \param connectable whether incoming LE connections are accepted.
    void set_connectable(bool connectable);

    /// Set the Pairable property.
    /// \param pairable whether the adapter accepts pairing requests.
    void set_pairable(bool pairable);

    /// Set the PairableTimeout property.
    /// \param timeout BlueZ pairable timeout in seconds; zero leaves it unlimited.
    void set_pairable_timeout(uint32_t timeout);

    /// Set the adapter Alias property.
    /// \param alias human-readable adapter name exposed by BlueZ.
    void set_alias(const std::string& alias);

    /// Start LE discovery with an optional transport filter (le, bredr, auto).
    /// \param transport BlueZ discovery filter: low energy classic or automatic.
    void start_discovery(const std::string& transport = "le");

    /// Stop discovery.
    void stop_discovery();

    /// Remove a device by its object path.
    /// \param device_path BlueZ device path removed from the adapter.
    void remove_device(const std::string& device_path);

    /// \brief Return the selected BlueZ adapter path.
    /// \return Selected BlueZ Adapter1 path.
    const std::string& adapter_path() const {
        // Return the selected Adapter1 object path controlled by this wrapper.
        return adapter_path_;
    }

private:
    /// \brief Write one Adapter1 property and surface D-Bus failures with context.
    /// \param name BlueZ Adapter1 property to change.
    /// \param value D-Bus value assigned to the adapter property.
    void set_adapter_property(const std::string& name, const sdbus::Variant& value);

    DbusConnection& dbus_;
    std::string adapter_path_;
    rclcpp::Logger logger_;
    std::unique_ptr<sdbus::IProxy> proxy_;
};

}  // namespace mrs_uav_bluetooth::bluez
