// SPDX-License-Identifier: BSD-3-Clause
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

    WifiService(bluez::DbusConnection& dbus,
                const std::string& base_path,
                int index,
                ReadCb wifi_name_cb,
                WriteCb wifi_apply_cb,
                WriteCb wifi_password_write_cb,
                ReadCb wifi_password_read_cb);

    std::shared_ptr<GattService> service() const { return service_; }
    std::string uuid() const;

    void update(const std::string& ssid);
    void update_status(const std::string& status);

private:
    std::string status_text() const;

    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> ssid_characteristic_;
    std::shared_ptr<GattCharacteristic> password_characteristic_;
    std::shared_ptr<GattCharacteristic> status_characteristic_;
    mutable std::mutex status_mutex_;
    std::string status_text_;
};

}  // namespace mrs_uav_bluetooth::gatt::services
