// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::gatt {

/// Exported LE advertisement on D-Bus.
class Advertisement {
public:
    Advertisement(bluez::DbusConnection& dbus,
                  const std::string& object_path,
                  const std::string& ad_type = "peripheral");
    ~Advertisement();

    void export_object();
    void unexport();

    const std::string& path() const { return path_; }

    // --- Builder methods ---
    void set_local_name(const std::string& name) { local_name_ = name; }
    void set_discoverable(bool d) { discoverable_ = d; }
    void set_discoverable_timeout(uint16_t t) { discoverable_timeout_ = t; }
    void set_includes(const std::vector<std::string>& inc) { includes_ = inc; }
    void set_tx_power(int16_t p) { tx_power_ = p; has_tx_power_ = true; }
    void add_service_uuid(const std::string& uuid) { service_uuids_.push_back(uuid); }
    void set_service_uuids(const std::vector<std::string>& uuids) { service_uuids_ = uuids; }

    /// Register this advertisement with BlueZ LEAdvertisingManager1.
    void register_advertisement(const std::string& adapter_path);

    /// Unregister from BlueZ.
    void unregister_advertisement(const std::string& adapter_path);

private:
    std::map<std::string, sdbus::Variant> get_properties() const;

    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string ad_type_;
    std::string local_name_;
    bool discoverable_{true};
    uint16_t discoverable_timeout_{0};
    std::vector<std::string> includes_;
    std::vector<std::string> service_uuids_;
    int16_t tx_power_{0};
    bool has_tx_power_{false};

    std::unique_ptr<sdbus::IObject> exported_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::gatt
