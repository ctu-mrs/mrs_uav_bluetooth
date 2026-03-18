// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <functional>
#include <string>

namespace mrs_uav_bluetooth::gatt::services {

/// Wi-Fi BLE service exposing current SSID and password configuration.
class WifiService {
public:
    using ReadCb = std::function<std::string()>;
    using WriteCb = std::function<void(const std::string&)>;

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

private:
    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> characteristic_;
    std::shared_ptr<GattDescriptor> ssid_descriptor_;
    std::shared_ptr<GattDescriptor> password_descriptor_;
};

}  // namespace mrs_uav_bluetooth::gatt::services
