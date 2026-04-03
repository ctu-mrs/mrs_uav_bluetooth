// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/adapter_controller.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace mrs_uav_bluetooth::app {

struct CentralClientRuntimeOptions {
    std::string adapter_alias;
    std::string scan_mode{"le"};
    bool scan_on_start{true};
};

class CentralClientRuntime {
public:
    CentralClientRuntime(rclcpp::Logger logger,
                         CentralClientRuntimeOptions options = {});

    const std::string& adapter_path() const;
    bool is_scanning() const;
    bool scan_desired() const;
    void set_scan_enabled(bool enabled, const std::string& transport = {});
    void refresh_scan(const std::string& transport = {});

    bluez::ObjectManagerCache& cache();
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