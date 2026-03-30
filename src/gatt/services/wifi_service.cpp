// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/services/wifi_service.hpp"

namespace mrs_uav_bluetooth::gatt::services {

namespace {

std::string trim_trailing_ascii_whitespace(std::string value) {
    while (!value.empty() && (value.back() == ' ' || value.back() == '\n' || value.back() == '\r' || value.back() == '\t')) {
        value.pop_back();
    }
    return value;
}

}  // namespace

using namespace mrs_uav_bluetooth::bluez;
using namespace mrs_uav_bluetooth::util;

static const std::string kWifiServiceName = "wifi";
static const std::string kWifiSsidName = "wifi/ssid";
static const std::string kWifiPasswordName = "wifi/password";

WifiService::WifiService(DbusConnection& dbus,
                         const std::string& base_path,
                         int index,
                         ReadCb wifi_name_cb,
                         WriteCb wifi_apply_cb,
                         WriteCb wifi_password_write_cb,
                         ReadCb wifi_password_read_cb) {
    auto svc_uuid = named_service_uuid(kWifiServiceName);
    auto ssid_uuid = named_characteristic_uuid(kWifiSsidName);
    auto password_uuid = named_characteristic_uuid(kWifiPasswordName);

    std::string svc_path = base_path + "/service" + std::to_string(index);
    std::string ssid_path = svc_path + "/char0";
    std::string password_path = svc_path + "/char1";

    service_ = std::make_shared<GattService>(dbus, svc_path, svc_uuid, true);
    ssid_characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, ssid_path, ssid_uuid,
        std::vector<std::string>{"read", "write", "write-without-response"},
        *service_);
    password_characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, password_path, password_uuid,
        std::vector<std::string>{"read", "write", "write-without-response"},
        *service_);

    auto name_cb = wifi_name_cb;
    auto apply_cb = wifi_apply_cb;
    auto password_write_cb = wifi_password_write_cb;
    auto password_read_cb = wifi_password_read_cb;
    ssid_characteristic_->set_read_callback([name_cb]() -> std::vector<uint8_t> {
        auto s = name_cb();
        return {s.begin(), s.end()};
    });
    ssid_characteristic_->set_write_callback([apply_cb](const std::vector<uint8_t>& data) {
        std::string s(data.begin(), data.end());
        s = trim_trailing_ascii_whitespace(std::move(s));
        apply_cb(s);
    });
    password_characteristic_->set_read_callback([password_read_cb]() {
        const auto value = password_read_cb();
        return std::vector<uint8_t>(value.begin(), value.end());
    });
    password_characteristic_->set_write_callback([password_write_cb](const std::vector<uint8_t>& data) {
        std::string value(data.begin(), data.end());
        password_write_cb(value);
    });

    service_->add_characteristic(ssid_characteristic_);
    service_->add_characteristic(password_characteristic_);

    // Set initial value.
    auto initial = name_cb();
    ssid_characteristic_->set_value({initial.begin(), initial.end()});
    auto password_initial = password_read_cb();
    password_characteristic_->set_value({password_initial.begin(), password_initial.end()});
}

std::string WifiService::uuid() const {
    return named_service_uuid(kWifiServiceName);
}

void WifiService::update(const std::string& ssid) {
    std::vector<uint8_t> payload(ssid.begin(), ssid.end());
    ssid_characteristic_->set_value(payload, true);
}

}  // namespace mrs_uav_bluetooth::gatt::services
