// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/services/time_service.hpp
/// \brief Declares the time service component of the Bluetooth Low Energy GATT layer.

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
    /// \brief Build the local time value and peer writeback GATT objects.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param base_path Application D-Bus path beneath which this service is exported.
    /// \param index Service number used to keep this exported D-Bus subtree unique.
    /// \param writeback_cb consumer invoked for a peer time-writeback value.
    TimeService(bluez::DbusConnection& dbus,
                const std::string& base_path,
                int index,
                TimeWritebackCb writeback_cb = nullptr);

    /// \brief Access the complete exported time-service object tree.
    /// \return GATT service object owned by this wrapper.
    std::shared_ptr<GattService> service() const {
        // Return the complete exported time-service object tree.
        return service_;
    }
    /// \brief Return the deterministic UUID of the built-in time service.
    /// \return Time-service UUID registered in the local GATT tree.
    std::string uuid() const;

    /// \brief Refresh the readable time value and notify active subscribers.
    void update();

private:
    /// \brief Validate an eight-byte peer timestamp and deliver it with the writer's device path.
    /// \param payload Eight-byte peer timestamp written through GATT.
    /// \param device_path BlueZ device path identifying the peer that wrote the timestamp.
    void handle_writeback(const std::vector<uint8_t>& payload, const std::string& device_path);
    /// \brief Encode the current system clock as the built-in time characteristic value.
    /// \return Current system time encoded as eight little-endian nanosecond bytes.
    std::vector<uint8_t> read_time();

    std::shared_ptr<GattService> service_;
    std::shared_ptr<GattCharacteristic> characteristic_;
    std::shared_ptr<GattDescriptor> time_descriptor_;
    std::shared_ptr<GattDescriptor> writeback_descriptor_;
    TimeWritebackCb writeback_cb_;
    std::vector<uint8_t> last_writeback_;
};

}  // namespace mrs_uav_bluetooth::gatt::services
