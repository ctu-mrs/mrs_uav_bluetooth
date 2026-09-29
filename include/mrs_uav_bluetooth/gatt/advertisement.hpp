// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/advertisement.hpp
/// \brief Declares the advertisement component of the Bluetooth Low Energy GATT layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <sdbus-c++/sdbus-c++.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::gatt {

/// Exported LE advertisement on D-Bus.
class Advertisement {
public:
    /// \brief Define an LE advertisement object for later BlueZ registration.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param object_path D-Bus path where the advertisement object is exported.
    /// \param ad_type BlueZ advertisement role, peripheral or broadcast.
    Advertisement(bluez::DbusConnection& dbus,
                  const std::string& object_path,
                  const std::string& ad_type = "peripheral");
    /// \brief Unregister and remove the advertisement object from D-Bus.
    ~Advertisement();

    /// \brief Export LEAdvertisement1 properties and its release callback.
    void export_object();
    /// \brief Stop exporting advertisement.
    void unexport();

    /// \brief Return the D-Bus path registered with LEAdvertisingManager1.
    /// \return Registered D-Bus object path.
    const std::string& path() const {
        // Return the D-Bus path registered with LEAdvertisingManager1.
        return path_;
    }

    // --- Builder methods ---
    /// \brief Replace the human-readable name in the primary advertisement.
    /// \param name Human-readable name included in the advertisement.
    void set_local_name(const std::string& name) {
        // Select the local name included in the next advertising registration.
        local_name_ = name;
    }
    /// \brief Set and signal whether the advertisement invites connections.
    /// \param d Whether the advertisement declares the device discoverable.
    void set_discoverable(bool d) {
        // Select whether the next connectable advertisement is discoverable.
        discoverable_ = d;
    }
    /// \brief Set and signal how long discoverability remains active.
    /// \param t Discoverable lifetime in seconds exported to BlueZ.
    void set_discoverable_timeout(uint16_t t) {
        // Set the discoverable lifetime exported to BlueZ.
        discoverable_timeout_ = t;
    }
    /// \brief Replace optional controller-generated advertisement fields such as transmit power.
    /// \param inc Controller-generated advertisement field names to request.
    void set_includes(const std::vector<std::string>& inc) {
        // Select controller-generated fields requested from BlueZ.
        includes_ = inc;
    }
    /// \brief Replace service UUIDs requested from nearby advertisers.
    /// \param uuids Service UUIDs requested from nearby advertisers.
    void set_solicit_uuids(const std::vector<std::string>& uuids) {
        // Select services requested from nearby advertisers.
        solicit_uuids_ = uuids;
    }
    /// \brief Replace manufacturer-specific fields and signal the new snapshot.
    /// \param data Manufacturer identifiers and their advertised byte values.
    void set_manufacturer_data(const std::map<uint16_t, std::vector<uint8_t>>& data);
    /// \brief Replace UUID-keyed primary advertisement payloads and signal the new snapshot.
    /// \param data Service UUIDs and their advertised byte values.
    void set_service_data(const std::map<std::string, std::vector<uint8_t>>& data);
    /// \brief Replace raw primary advertisement fields and signal the new snapshot.
    /// \param data Raw advertisement field types and their byte values.
    void set_data(const std::map<uint8_t, std::vector<uint8_t>>& data);
    /// \brief Move the supplied service UUIDs into the scan response.
    /// \param uuids Service UUIDs placed in the scan response.
    void set_scan_response_service_uuids(const std::vector<std::string>& uuids) {
        // Put these service identifiers in the optional scan response.
        scan_response_service_uuids_ = uuids;
    }
    /// \brief Replace manufacturer-specific scan-response fields.
    /// \param data Manufacturer identifiers and bytes placed in the scan response.
    void set_scan_response_manufacturer_data(const std::map<uint16_t, std::vector<uint8_t>>& data);
    /// \brief Replace solicited service UUIDs in the scan response.
    /// \param uuids Solicited service UUIDs placed in the scan response.
    void set_scan_response_solicit_uuids(const std::vector<std::string>& uuids) {
        // Put these requested peer services in the optional scan response.
        scan_response_solicit_uuids_ = uuids;
    }
    /// \brief Replace UUID-keyed scan-response payloads.
    /// \param data Service UUIDs and bytes placed in the scan response.
    void set_scan_response_service_data(const std::map<std::string, std::vector<uint8_t>>& data);
    /// \brief Replace raw scan-response fields.
    /// \param data Raw field types and bytes placed in the scan response.
    void set_scan_response_data(const std::map<uint8_t, std::vector<uint8_t>>& data);
    /// \brief Set the optional Bluetooth appearance code advertised to peers.
    /// \param appearance standard Bluetooth appearance code included in advertising data.
    void set_appearance(std::optional<uint16_t> appearance) {
        // Set the optional standard device-category code.
        appearance_ = appearance;
    }
    /// \brief Set the optional advertising rotation duration.
    /// \param duration advertising rotation duration or requested timer interval.
    void set_duration(std::optional<uint16_t> duration) {
        // Set the radio rotation duration for this advertising instance.
        duration_ = duration;
    }
    /// \brief Set the optional lifetime after which BlueZ withdraws the advertisement.
    /// \param timeout Optional lifetime in seconds before BlueZ withdraws the advertisement.
    void set_timeout(std::optional<uint16_t> timeout) {
        // Set when BlueZ automatically stops this advertising instance.
        timeout_ = timeout;
    }
    /// \brief Select the controller's secondary advertising channel.
    /// \param secondary_channel extended-advertising secondary radio mode requested from BlueZ.
    void set_secondary_channel(const std::string& secondary_channel) {
        // Select the optional secondary radio mode for extended advertising.
        secondary_channel_ = secondary_channel;
    }
    /// \brief Set the minimum requested advertising interval.
    /// \param min_interval smallest requested advertising interval.
    void set_min_interval(std::optional<uint32_t> min_interval) {
        // Set the shortest requested interval between advertising packets.
        min_interval_ = min_interval;
    }
    /// \brief Set the maximum requested advertising interval.
    /// \param max_interval largest requested advertising interval.
    void set_max_interval(std::optional<uint32_t> max_interval) {
        // Set the longest requested interval between advertising packets.
        max_interval_ = max_interval;
    }
    /// \brief Set the requested transmit power in dBm.
    /// \param power Optional requested transmit power in dBm.
    void set_tx_power(std::optional<int16_t> power) {
        // Set the optional advertising transmit-power request.
        tx_power_ = power;
    }
    /// \brief Append one service UUID to the current primary advertisement list.
    /// \param uuid Service UUID to include in the advertisement.
    void add_service_uuid(const std::string& uuid) {
        // Preserve insertion order because packet fitting keeps an ordered prefix.
        service_uuids_.push_back(uuid);
    }
    /// \brief Replace all primary advertised service UUIDs in one change.
    /// \param uuids Service UUIDs placed in the primary advertisement.
    void set_service_uuids(const std::vector<std::string>& uuids) {
        // Select service identifiers included in the primary advertising packet.
        service_uuids_ = uuids;
    }

    /// Register this advertisement with BlueZ LEAdvertisingManager1.
    /// \param adapter_path BlueZ adapter whose advertising manager registers or removes this object.
    void register_advertisement(const std::string& adapter_path);

    /// Unregister from BlueZ.
    /// \param adapter_path BlueZ adapter whose advertising manager registers or removes this object.
    void unregister_advertisement(const std::string& adapter_path);

    /// \brief Report whether LEAdvertisingManager1 accepted this object.
    /// \return True only in the registered lifecycle state.
    bool is_registered() const {
        // Transitional and released states are deliberately not considered active.
        return registration_ == Registration::Registered;
    }
    /// \brief Return whether BlueZ released this object.
    /// \return True when BlueZ invoked Release; otherwise false.
    bool was_released() const {
        // Report whether BlueZ invoked Release on this local profile.
        return released_;
    }
    /// \brief Signal exactly one changed advertisement property with its current value.
    /// \param property_name D-Bus property name used in the Properties call.
    void emit_property_changed(const std::string& property_name);

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
    // Release runs on the D-Bus event thread, not the ROS config thread.
    enum class Registration { Unregistered, Registering, Registered };
    std::atomic<Registration> registration_{Registration::Unregistered};
    std::atomic<bool> released_{false};
    mutable std::mutex data_mutex_;
};

}  // namespace mrs_uav_bluetooth::gatt
