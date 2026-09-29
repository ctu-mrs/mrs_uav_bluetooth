// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/app/central_client_runtime.hpp
/// \brief Declares the central client runtime component of the ROS 2 application and operator-tool layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/adapter_controller.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace mrs_uav_bluetooth::app {

/// Startup switches for the standalone BLE central-client tools.
struct CentralClientRuntimeOptions {
    std::string adapter_alias;
    std::string scan_mode{"auto"};
    bool scan_on_start{true};
};

/// Owns one client-side D-Bus connection, BlueZ cache, and adapter controller.
/// It contains no ROS publishers; command-line and TUI front ends share it.
class CentralClientRuntime {
public:
    /// \brief Open BlueZ, select an adapter, and start the shared object cache.
    /// \param logger ROS logger used for diagnostics.
    /// \param options Adapter selection scan and peer-management settings.
    CentralClientRuntime(rclcpp::Logger logger,
                         CentralClientRuntimeOptions options = {});

    /// \brief Return the selected BlueZ adapter path.
    /// \return Selected BlueZ Adapter1 path.
    const std::string& adapter_path() const;
    /// \brief Return whether Bluetooth discovery is active.
    /// \return True when scanning; otherwise false.
    bool is_scanning() const;
    /// \brief Return whether discovery should remain active.
    /// \return True when scan desired; otherwise false.
    bool scan_desired() const;
    /// \brief Record the desired discovery state and reconcile it with BlueZ.
    /// \param enabled Whether adapter discovery should run.
    /// \param transport Optional discovery transport replacing the current scan filter.
    void set_scan_enabled(bool enabled, const std::string& transport = {});
    /// \brief Reapply the current discovery request after adapter or configuration changes.
    /// \param transport Optional discovery transport replacing the current scan filter.
    void refresh_scan(const std::string& transport = {});

    /// \brief Return the shared D-Bus connection.
    /// \return D-Bus.
    bluez::DbusConnection& dbus();
    /// \brief Return the BlueZ object cache.
    /// \return Shared cache fed by BlueZ ObjectManager signals.
    bluez::ObjectManagerCache& cache();
    /// \brief Return the BlueZ client.
    /// \return BlueZ client bound to the selected adapter.
    bluez::BluezClient& client();

private:
    rclcpp::Logger logger_;
    CentralClientRuntimeOptions options_;
    std::string adapter_path_;
    std::unique_ptr<bluez::DbusConnection> dbus_;
    std::unique_ptr<bluez::ObjectManagerCache> cache_;
    std::unique_ptr<bluez::AdapterController> adapter_;
    std::unique_ptr<bluez::BluezClient> client_;
    bool scan_desired_{false};
};

}  // namespace mrs_uav_bluetooth::app
