// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::gatt::services {

/// Callback for time-writeback events: (payload, device_path, receive_time_ns).
using TimeWritebackCb = std::function<void(const std::vector<uint8_t>&, const std::string&, uint64_t)>;

/// Time BLE service publishing local nanosecond timestamps.
class TimeService {
public:
    TimeService(bluez::DbusConnection& dbus,
                const std::string& base_path,
                int index,
                TimeWritebackCb writeback_cb = nullptr);

    std::shared_ptr<GattService> service() const { return service_; }
    std::string uuid() const;

    void update();

private:
    void handle_writeback(const std::vector<uint8_t>& payload, const std::string& device_path);
    std::vector<uint8_t> read_time();

    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> characteristic_;
    std::shared_ptr<GattDescriptor> time_descriptor_;
    std::shared_ptr<GattDescriptor> writeback_descriptor_;
    TimeWritebackCb writeback_cb_;
    std::vector<uint8_t> last_writeback_;
};

}  // namespace mrs_uav_bluetooth::gatt::services
