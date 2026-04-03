// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

std::string device_display_name(const bluez::DeviceInfo& device);

std::string device_hostname_guess(const bluez::DeviceInfo& device,
                                  std::string_view pattern = "^uav[0-9]{2}$");

}  // namespace mrs_uav_bluetooth::util