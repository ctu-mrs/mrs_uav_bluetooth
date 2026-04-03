// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::gatt {

const std::string& wifi_service_uuid();
const std::string& wifi_ssid_characteristic_uuid();
const std::string& wifi_password_characteristic_uuid();
const std::string& wifi_status_characteristic_uuid();

const std::string& time_service_uuid();
const std::string& time_characteristic_uuid();
const std::string& time_value_descriptor_uuid();
const std::string& time_writeback_descriptor_uuid();

bool is_wifi_service_uuid(std::string_view uuid);
bool is_time_service_uuid(std::string_view uuid);

std::string builtin_service_name(std::string_view service_uuid);
std::string builtin_characteristic_name(std::string_view service_uuid,
                                        std::string_view characteristic_uuid);
std::string builtin_descriptor_name(std::string_view service_uuid,
                                    std::string_view descriptor_uuid);

}  // namespace mrs_uav_bluetooth::gatt