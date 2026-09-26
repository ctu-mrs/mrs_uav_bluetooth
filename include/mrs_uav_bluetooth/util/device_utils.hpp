// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace mrs_uav_bluetooth::util {

std::string device_display_name(const bluez::DeviceInfo& device);

/// Resolve a nearby device name as a UAV hostname. A name explicitly listed
/// in the shared peer policy wins even when the usual UAV pattern differs.
/// An empty result means callers should use the device's MAC as the fallback.
std::string device_hostname_guess(
    const bluez::DeviceInfo& device,
    std::string_view pattern = "^uav[0-9]{1,5}$",
    const std::vector<std::string>& known_peers = {});

}  // namespace mrs_uav_bluetooth::util
