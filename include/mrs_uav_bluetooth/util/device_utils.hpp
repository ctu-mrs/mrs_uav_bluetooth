// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/device_utils.hpp
/// \brief Declares the device utils component of the shared utility layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace mrs_uav_bluetooth::util {

/// \brief Choose the best available peer name, alias, or address for the UI.
/// \param device Cached BlueZ record supplying the preferred name alias or address.
/// \return Best available device name, alias, or address.
std::string device_display_name(const bluez::DeviceInfo& device);

/// Resolve a nearby device name as a UAV hostname. A name explicitly listed
/// in the shared peer policy wins even when the usual UAV pattern differs.
/// An empty result means callers should use the device's MAC as the fallback.
/// \param device Cached BlueZ record whose names and advertisement data may identify a UAV.
/// \param pattern Regular expression that valid UAV hostnames must match completely.
/// \param known_peers explicitly admitted hostnames used to validate a device name.
/// \return Validated peer hostname, or an empty string when none is credible.
std::string device_hostname_guess(
    const bluez::DeviceInfo& device,
    std::string_view pattern = "^uav[0-9]{1,5}$",
    const std::vector<std::string>& known_peers = {});

}  // namespace mrs_uav_bluetooth::util
