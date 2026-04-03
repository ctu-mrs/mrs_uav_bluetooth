// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/services/time_service.hpp"

#include <chrono>
#include <cstring>

namespace mrs_uav_bluetooth::gatt::services {

namespace {

uint64_t now_time_ns() {
    auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now());
    return static_cast<uint64_t>(now.time_since_epoch().count());
}

std::string extract_device_path(const std::map<std::string, sdbus::Variant>& options) {
    auto it = options.find("device");
    if (it == options.end()) {
        return {};
    }
    try {
        return static_cast<std::string>(it->second.get<sdbus::ObjectPath>());
    } catch (...) {
    }
    try {
        return it->second.get<std::string>();
    } catch (...) {
    }
    return {};
}

class TimeWritebackDescriptor final : public GattDescriptor {
public:
    TimeWritebackDescriptor(bluez::DbusConnection& dbus,
                            const std::string& object_path,
                            const std::string& uuid,
                            const std::vector<std::string>& flags,
                            GattCharacteristic& parent,
                            std::function<void(const std::vector<uint8_t>&, const std::string&)> write_cb)
        : GattDescriptor(dbus, object_path, uuid, flags, parent), write_cb_(std::move(write_cb)) {}

protected:
    std::vector<uint8_t> on_read(const std::map<std::string, sdbus::Variant>&) override {
        return value();
    }

    void on_write(const std::vector<uint8_t>& data,
                  const std::map<std::string, sdbus::Variant>& options) override {
        set_value(data, true);
        if (write_cb_) {
            write_cb_(data, extract_device_path(options));
        }
    }

private:
    std::function<void(const std::vector<uint8_t>&, const std::string&)> write_cb_;
};

}  // namespace

using namespace mrs_uav_bluetooth::bluez;
using namespace mrs_uav_bluetooth::util;

static const std::string kTimeServiceName = "time";
static const std::string kTimeCharacteristicName = "time/ns";
static const std::string kTimeDescriptorName = "time/ns/value";
static const std::string kTimeWritebackDescriptorName = "time/ns/writeback";

TimeService::TimeService(DbusConnection& dbus,
                         const std::string& base_path,
                         int index,
                         TimeWritebackCb writeback_cb)
    : writeback_cb_(std::move(writeback_cb)), last_writeback_(8, 0) {
    auto svc_uuid = named_service_uuid(kTimeServiceName);
    auto chrc_uuid = named_characteristic_uuid(kTimeCharacteristicName);
    auto desc_uuid = named_descriptor_uuid(kTimeDescriptorName);
    auto writeback_uuid = named_descriptor_uuid(kTimeWritebackDescriptorName);

    std::string svc_path = base_path + "/service" + std::to_string(index);
    std::string chrc_path = svc_path + "/char0";
    std::string desc0_path = chrc_path + "/desc0";
    std::string desc1_path = chrc_path + "/desc1";

    service_ = std::make_shared<GattService>(dbus, svc_path, svc_uuid, true);
    characteristic_ = std::make_shared<GattCharacteristic>(
        dbus, chrc_path, chrc_uuid,
        std::vector<std::string>{"read", "notify"}, *service_);
    characteristic_->set_read_callback([this]() { return read_time(); });

    time_descriptor_ = std::make_shared<GattDescriptor>(
        dbus, desc0_path, desc_uuid,
        std::vector<std::string>{"read"}, *characteristic_);

    writeback_descriptor_ = std::make_shared<TimeWritebackDescriptor>(
        dbus, desc1_path, writeback_uuid,
        std::vector<std::string>{"read", "write"}, *characteristic_,
        [this](const std::vector<uint8_t>& payload, const std::string& device_path) {
            handle_writeback(payload, device_path);
        });

    writeback_descriptor_->set_value(last_writeback_);

    characteristic_->add_descriptor(time_descriptor_);
    characteristic_->add_descriptor(writeback_descriptor_);
    service_->add_characteristic(characteristic_);

    auto initial = read_time();
    characteristic_->set_value(initial);
    time_descriptor_->set_value(initial);
}

std::string TimeService::uuid() const {
    return named_service_uuid(kTimeServiceName);
}

std::vector<uint8_t> TimeService::read_time() {
    uint64_t ns = now_time_ns();
    std::vector<uint8_t> payload(sizeof(ns));
    std::memcpy(payload.data(), &ns, sizeof(ns));
    return payload;
}

void TimeService::handle_writeback(const std::vector<uint8_t>& payload,
                                   const std::string& device_path) {
    last_writeback_ = payload;
    if (writeback_descriptor_) {
        writeback_descriptor_->set_value(last_writeback_);
    }
    if (writeback_cb_) {
        writeback_cb_(last_writeback_, device_path, now_time_ns());
    }
}

void TimeService::update() {
    auto payload = read_time();
    time_descriptor_->set_value(payload, true);
    characteristic_->publish(payload);
}

}  // namespace mrs_uav_bluetooth::gatt::services
