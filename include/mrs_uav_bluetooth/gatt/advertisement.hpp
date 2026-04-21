// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
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
    void set_solicit_uuids(const std::vector<std::string>& uuids) { solicit_uuids_ = uuids; }
    void set_manufacturer_data(const std::map<uint16_t, std::vector<uint8_t>>& data);
    void set_service_data(const std::map<std::string, std::vector<uint8_t>>& data);
    void set_data(const std::map<uint8_t, std::vector<uint8_t>>& data);
    void set_scan_response_service_uuids(const std::vector<std::string>& uuids) {
        scan_response_service_uuids_ = uuids;
    }
    void set_scan_response_manufacturer_data(const std::map<uint16_t, std::vector<uint8_t>>& data);
    void set_scan_response_solicit_uuids(const std::vector<std::string>& uuids) {
        scan_response_solicit_uuids_ = uuids;
    }
    void set_scan_response_service_data(const std::map<std::string, std::vector<uint8_t>>& data);
    void set_scan_response_data(const std::map<uint8_t, std::vector<uint8_t>>& data);
    void set_appearance(std::optional<uint16_t> appearance) { appearance_ = appearance; }
    void set_duration(std::optional<uint16_t> duration) { duration_ = duration; }
    void set_timeout(std::optional<uint16_t> timeout) { timeout_ = timeout; }
    void set_secondary_channel(const std::string& secondary_channel) { secondary_channel_ = secondary_channel; }
    void set_min_interval(std::optional<uint32_t> min_interval) { min_interval_ = min_interval; }
    void set_max_interval(std::optional<uint32_t> max_interval) { max_interval_ = max_interval; }
    void set_tx_power(std::optional<int16_t> power) { tx_power_ = power; }
    void add_service_uuid(const std::string& uuid) { service_uuids_.push_back(uuid); }
    void set_service_uuids(const std::vector<std::string>& uuids) { service_uuids_ = uuids; }

    /// Register this advertisement with BlueZ LEAdvertisingManager1.
    void register_advertisement(const std::string& adapter_path);

    /// Unregister from BlueZ.
    void unregister_advertisement(const std::string& adapter_path);

private:
    bluez::DbusConnection& dbus_;
    std::string path_;
    std::string ad_type_;
    std::string local_name_;
    bool discoverable_{true};
    uint16_t discoverable_timeout_{0};
    std::vector<std::string> includes_;
    std::vector<std::string> service_uuids_;
    std::vector<std::string> solicit_uuids_;
    std::map<uint16_t, sdbus::Variant> manufacturer_data_;
    std::map<std::string, sdbus::Variant> service_data_;
    std::map<uint8_t, sdbus::Variant> data_;
    std::vector<std::string> scan_response_service_uuids_;
    std::map<uint16_t, sdbus::Variant> scan_response_manufacturer_data_;
    std::vector<std::string> scan_response_solicit_uuids_;
    std::map<std::string, sdbus::Variant> scan_response_service_data_;
    std::map<uint8_t, sdbus::Variant> scan_response_data_;
    std::optional<uint16_t> appearance_;
    std::optional<uint16_t> duration_;
    std::optional<uint16_t> timeout_;
    std::string secondary_channel_;
    std::optional<uint32_t> min_interval_;
    std::optional<uint32_t> max_interval_;
    std::optional<int16_t> tx_power_;

    std::unique_ptr<sdbus::IObject> exported_;
    bool registered_{false};
};

}  // namespace mrs_uav_bluetooth::gatt
