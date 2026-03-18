// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/gatt/services/wifi_service.hpp"

namespace mrs_uav_bluetooth::gatt::services {

namespace {

std::string trim_trailing_ascii_whitespace(std::string value) {
    while (!value.empty() && (value.back() == ' ' || value.back() == '\n' || value.back() == '\r' || value.back() == '\t')) {
        value.pop_back();
    }
    return value;
}

class CallbackDescriptor final : public GattDescriptor {
public:
    using ReadBytesCb = std::function<std::vector<uint8_t>()>;
    using WriteBytesCb = std::function<void(const std::vector<uint8_t>&)>;

    CallbackDescriptor(bluez::DbusConnection& dbus,
                       const std::string& object_path,
                       const std::string& uuid,
                       const std::vector<std::string>& flags,
                       GattCharacteristic& parent,
                       ReadBytesCb read_cb,
                       WriteBytesCb write_cb)
        : GattDescriptor(dbus, object_path, uuid, flags, parent),
          read_cb_(std::move(read_cb)),
          write_cb_(std::move(write_cb)) {}

protected:
    std::vector<uint8_t> on_read(const std::map<std::string, sdbus::Variant>&) override {
        if (read_cb_) {
            auto value = read_cb_();
            set_value(value);
            return value;
        }
        return value();
    }

    void on_write(const std::vector<uint8_t>& data,
                  const std::map<std::string, sdbus::Variant>&) override {
        set_value(data, true);
        if (write_cb_) {
            write_cb_(data);
        }
    }

private:
    ReadBytesCb read_cb_;
    WriteBytesCb write_cb_;
};

}  // namespace

using namespace mrs_uav_bluetooth::bluez;
using namespace mrs_uav_bluetooth::util;

static const std::string kWifiServiceName = "wifi";
static const std::string kWifiSsidName = "wifi/ssid";
static const std::string kWifiSsidDescName = "wifi/ssid/config";
static const std::string kWifiPasswordDescName = "wifi/password/config";

WifiService::WifiService(DbusConnection& dbus,
                         const std::string& base_path,
                         int index,
                         ReadCb wifi_name_cb,
                         WriteCb wifi_apply_cb,
                         WriteCb wifi_password_write_cb,
                         ReadCb wifi_password_read_cb) {
    auto svc_uuid = named_service_uuid(kWifiServiceName);
    auto chrc_uuid = named_characteristic_uuid(kWifiSsidName);
    auto desc_uuid = named_descriptor_uuid(kWifiSsidDescName);
    auto pwd_uuid = named_descriptor_uuid(kWifiPasswordDescName);

    std::string svc_path = base_path + "/service" + std::to_string(index);
    std::string chrc_path = svc_path + "/char0";
    std::string desc0_path = chrc_path + "/desc0";
    std::string desc1_path = chrc_path + "/desc1";

    service_ = std::make_shared<GattService>(dbus, svc_path, svc_uuid, true);
    characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, chrc_path, chrc_uuid,
        std::vector<std::string>{"read", "write", "write-without-response", "notify"},
        *service_);

    auto name_cb = wifi_name_cb;
    auto apply_cb = wifi_apply_cb;
    auto password_write_cb = wifi_password_write_cb;
    auto password_read_cb = wifi_password_read_cb;
    characteristic_->set_read_callback([name_cb]() -> std::vector<uint8_t> {
        auto s = name_cb();
        return {s.begin(), s.end()};
    });
    characteristic_->set_write_callback([apply_cb](const std::vector<uint8_t>& data) {
        std::string s(data.begin(), data.end());
        s = trim_trailing_ascii_whitespace(std::move(s));
        apply_cb(s);
    });

    ssid_descriptor_ = std::make_shared<CallbackDescriptor>(
        dbus, desc0_path, desc_uuid,
        std::vector<std::string>{"read", "write"}, *characteristic_,
        [name_cb]() {
            auto value = name_cb();
            return std::vector<uint8_t>(value.begin(), value.end());
        },
        [apply_cb](const std::vector<uint8_t>& data) {
            std::string value(data.begin(), data.end());
            apply_cb(trim_trailing_ascii_whitespace(std::move(value)));
        });

    password_descriptor_ = std::make_shared<CallbackDescriptor>(
        dbus, desc1_path, pwd_uuid,
        std::vector<std::string>{"read", "write"}, *characteristic_,
        [password_read_cb]() {
            auto value = password_read_cb();
            return std::vector<uint8_t>(value.begin(), value.end());
        },
        [password_write_cb](const std::vector<uint8_t>& data) {
            std::string value(data.begin(), data.end());
            password_write_cb(value);
        });

    characteristic_->add_descriptor(ssid_descriptor_);
    characteristic_->add_descriptor(password_descriptor_);
    service_->add_characteristic(characteristic_);

    // Set initial value.
    auto initial = name_cb();
    characteristic_->set_value({initial.begin(), initial.end()});
    ssid_descriptor_->set_value({initial.begin(), initial.end()});
    auto password_initial = password_read_cb();
    password_descriptor_->set_value({password_initial.begin(), password_initial.end()});
}

std::string WifiService::uuid() const {
    return named_service_uuid(kWifiServiceName);
}

void WifiService::update(const std::string& ssid) {
    std::vector<uint8_t> payload(ssid.begin(), ssid.end());
    ssid_descriptor_->set_value(payload, true);
    characteristic_->publish(payload);
}

}  // namespace mrs_uav_bluetooth::gatt::services
