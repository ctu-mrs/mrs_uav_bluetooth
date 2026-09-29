// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/services/wifi_service.hpp
/// \brief Declares the wifi service component of the Bluetooth Low Energy GATT layer.

#pragma once

#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <functional>
#include <mutex>
#include <string>
#include <utility>

namespace mrs_uav_bluetooth::gatt::services {

/// Wi-Fi BLE service exposing current SSID and password configuration.
class WifiService {
public:
    using ReadCb = std::function<std::string()>;
    using WriteResult = std::pair<bool, std::string>;
    using WriteCb = std::function<WriteResult(const std::string&)>;

    /// \brief Build network-name, password, and operation-status characteristics.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param base_path Application D-Bus path beneath which this service is exported.
    /// \param index Service number used to keep this exported D-Bus subtree unique.
    /// \param wifi_name_cb callback that reads the active wireless network name.
    /// \param wifi_apply_cb callback that applies a peer-requested wireless network.
    /// \param wifi_password_write_cb callback that persists a newly written network password.
    /// \param wifi_password_read_cb callback that reads the currently configured network password.
    WifiService(bluez::DbusConnection& dbus,
                const std::string& base_path,
                int index,
                ReadCb wifi_name_cb,
                WriteCb wifi_apply_cb,
                WriteCb wifi_password_write_cb,
                ReadCb wifi_password_read_cb);

    /// \brief Access the complete exported Wi-Fi service object tree.
    /// \return GATT service object owned by this wrapper.
    std::shared_ptr<GattService> service() const {
        // Return the complete exported Wi-Fi service object tree.
        return service_;
    }
    /// \brief Return the deterministic UUID of the built-in Wi-Fi service.
    /// \return Wi-Fi service UUID registered in the local GATT tree.
    std::string uuid() const;

    /// \brief Replace the advertised SSID value and notify active subscribers.
    /// \param ssid Wi-Fi network name to persist and activate.
    void update(const std::string& ssid);
    /// \brief Replace the readable Wi-Fi status characteristic and notify subscribers.
    /// \param status Operator-facing Wi-Fi state exposed through the status characteristic.
    void update_status(const std::string& status);

private:
    /// \brief Copy the latest network-change result under lock.
    /// \return Human-readable result of the latest Wi-Fi read or change.
    std::string status_text() const;

    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> ssid_characteristic_;
    std::shared_ptr<GattCharacteristic> password_characteristic_;
    std::shared_ptr<GattCharacteristic> status_characteristic_;
    mutable std::mutex status_mutex_;
    std::string status_text_;
};

}  // namespace mrs_uav_bluetooth::gatt::services
