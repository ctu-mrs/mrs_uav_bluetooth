// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/builtin_gatt.hpp
/// \brief Declares the builtin gatt component of the Bluetooth Low Energy GATT layer.

#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::gatt {

/// \brief Return the Wi-Fi service UUID.
/// \return Deterministic UUID of the built-in Wi-Fi service.
const std::string& wifi_service_uuid();
/// \brief Return the Wi-Fi ssid characteristic UUID.
/// \return Deterministic UUID of the Wi-Fi network-name characteristic.
const std::string& wifi_ssid_characteristic_uuid();
/// \brief Return the Wi-Fi password characteristic UUID.
/// \return Deterministic UUID of the Wi-Fi password characteristic.
const std::string& wifi_password_characteristic_uuid();
/// \brief Return the Wi-Fi status characteristic UUID.
/// \return Deterministic UUID of the Wi-Fi status characteristic.
const std::string& wifi_status_characteristic_uuid();

/// \brief Return the time service UUID.
/// \return Deterministic UUID of the built-in time service.
const std::string& time_service_uuid();
/// \brief Return the time characteristic UUID.
/// \return Deterministic UUID of the nanosecond time characteristic.
const std::string& time_characteristic_uuid();
/// \brief Return the time value descriptor UUID.
/// \return Deterministic UUID of the readable time descriptor.
const std::string& time_value_descriptor_uuid();
/// \brief Return the time writeback descriptor UUID.
/// \return Deterministic UUID of the peer time-writeback descriptor.
const std::string& time_writeback_descriptor_uuid();

/// \brief Test whether a UUID identifies the built-in Wi-Fi service.
/// \param uuid Service UUID to compare with the built-in identifier.
/// \return True only for the deterministic Wi-Fi service UUID.
bool is_wifi_service_uuid(std::string_view uuid);
/// \brief Test whether a UUID identifies the built-in time service.
/// \param uuid Service UUID to compare with the built-in identifier.
/// \return True only for the deterministic time service UUID.
bool is_time_service_uuid(std::string_view uuid);

/// \brief Map a known built-in service UUID to its operator-facing name.
/// \param service_uuid Service UUID mapped to a built-in display name.
/// \return Known built-in service name, or an empty string.
std::string builtin_service_name(std::string_view service_uuid);
/// \brief Map a known characteristic UUID to its name within the selected built-in service.
/// \param service_uuid Owning service UUID used to disambiguate the characteristic.
/// \param characteristic_uuid UUID of the characteristic.
/// \return Known built-in characteristic name, or an empty string.
std::string builtin_characteristic_name(std::string_view service_uuid,
                                        std::string_view characteristic_uuid);
/// \brief Map a known descriptor UUID to its name within the selected built-in service.
/// \param service_uuid Owning service UUID used to disambiguate the descriptor.
/// \param descriptor_uuid UUID of the descriptor.
/// \return Known built-in descriptor name, or an empty string.
std::string builtin_descriptor_name(std::string_view service_uuid,
                                    std::string_view descriptor_uuid);

}  // namespace mrs_uav_bluetooth::gatt
