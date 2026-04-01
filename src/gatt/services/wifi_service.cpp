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
static const std::string kWifiStatusName = "wifi/status";

WifiService::WifiService(DbusConnection& dbus,
                         const std::string& base_path,
                         int index,
                         ReadCb wifi_name_cb,
                         WriteCb wifi_apply_cb,
                         WriteCb wifi_password_write_cb,
                         ReadCb wifi_password_read_cb)
    : status_text_("ready") {
    auto svc_uuid = named_service_uuid(kWifiServiceName);
    auto ssid_uuid = named_characteristic_uuid(kWifiSsidName);
    auto password_uuid = named_characteristic_uuid(kWifiPasswordName);
    auto status_uuid = named_characteristic_uuid(kWifiStatusName);

    std::string svc_path = base_path + "/service" + std::to_string(index);
    std::string ssid_path = svc_path + "/char0";
    std::string password_path = svc_path + "/char1";
    std::string status_path = svc_path + "/char2";

    service_ = std::make_shared<GattService>(dbus, svc_path, svc_uuid, true);
    ssid_characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, ssid_path, ssid_uuid,
        std::vector<std::string>{"read", "write", "write-without-response"},
        *service_);
    password_characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, password_path, password_uuid,
        std::vector<std::string>{"read", "write", "write-without-response"},
        *service_);
    status_characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, status_path, status_uuid,
        std::vector<std::string>{"read", "notify"},
        *service_);

    auto name_cb = wifi_name_cb;
    auto apply_cb = wifi_apply_cb;
    auto password_write_cb = wifi_password_write_cb;
    auto password_read_cb = wifi_password_read_cb;
    ssid_characteristic_->set_read_callback([name_cb]() -> std::vector<uint8_t> {
        auto s = name_cb();
        return {s.begin(), s.end()};
    });
    password_characteristic_->set_read_callback([password_read_cb]() {
        const auto value = password_read_cb();
        return std::vector<uint8_t>(value.begin(), value.end());
    });
    status_characteristic_->set_read_callback([this]() {
        const auto value = status_text();
        return std::vector<uint8_t>(value.begin(), value.end());
    });
    ssid_characteristic_->set_write_callback([this, name_cb, password_read_cb, apply_cb](const std::vector<uint8_t>& data) {
        std::string requested(data.begin(), data.end());
        requested = trim_trailing_ascii_whitespace(std::move(requested));
        const auto result = apply_cb(requested);
        const auto actual_ssid = name_cb();
        const auto actual_password = password_read_cb();
        ssid_characteristic_->set_value({actual_ssid.begin(), actual_ssid.end()}, true);
        password_characteristic_->set_value({actual_password.begin(), actual_password.end()}, true);
        update_status((result.first ? "OK: " : "ERROR: ") + result.second);
    });
    password_characteristic_->set_write_callback([this, name_cb, password_read_cb, password_write_cb](const std::vector<uint8_t>& data) {
        std::string value(data.begin(), data.end());
        const auto result = password_write_cb(value);
        const auto actual_ssid = name_cb();
        const auto actual_password = password_read_cb();
        ssid_characteristic_->set_value({actual_ssid.begin(), actual_ssid.end()}, true);
        password_characteristic_->set_value({actual_password.begin(), actual_password.end()}, true);
        update_status((result.first ? "OK: " : "ERROR: ") + result.second);
    });

    service_->add_characteristic(ssid_characteristic_);
    service_->add_characteristic(password_characteristic_);
    service_->add_characteristic(status_characteristic_);

    // Set initial value.
    auto initial = name_cb();
    ssid_characteristic_->set_value({initial.begin(), initial.end()});
    auto password_initial = password_read_cb();
    password_characteristic_->set_value({password_initial.begin(), password_initial.end()});
    update_status(status_text_);
}

std::string WifiService::uuid() const {
    return named_service_uuid(kWifiServiceName);
}

void WifiService::update(const std::string& ssid) {
    std::vector<uint8_t> payload(ssid.begin(), ssid.end());
    ssid_characteristic_->set_value(payload, true);
}

void WifiService::update_status(const std::string& status) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_text_ = status;
    }
    std::vector<uint8_t> payload(status.begin(), status.end());
    status_characteristic_->publish(payload);
}

std::string WifiService::status_text() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_text_;
}

}  // namespace mrs_uav_bluetooth::gatt::services
