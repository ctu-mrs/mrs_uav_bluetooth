// SPDX-License-Identifier: BSD-3-Clause
/// \file src/util/device_utils.cpp
/// \brief Implements the device utils component of the shared utility layer.

#include "mrs_uav_bluetooth/util/device_utils.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <algorithm>

namespace mrs_uav_bluetooth::util {

/// \brief Choose the best available peer name, alias, or address for the UI.
/// \param device Cached BlueZ record supplying the preferred name alias or address.
/// \return Best available device name, alias, or address.
std::string device_display_name(const bluez::DeviceInfo& device) {
    // Choose the best available peer name, alias, or address for the UI.
    if (!device.alias.empty()) {
        return device.alias;
    }
    if (!device.name.empty()) {
        return device.name;
    }
    return "?";
}

/// \brief Choose a validated name or alias, including explicitly admitted peer names.
/// \param device Cached BlueZ record whose names and advertisement data may identify a UAV.
/// \param pattern Regular expression that valid UAV hostnames must match completely.
/// \param known_peers explicitly admitted hostnames used to validate a device name.
/// \return Validated peer hostname, or an empty string when none is credible.
std::string device_hostname_guess(const bluez::DeviceInfo& device,
                                  std::string_view pattern,
                                  const std::vector<std::string>& known_peers) {
    // Choose a validated name or alias, including explicitly admitted peer names.
    const auto resolves = [&](const std::string& candidate) {
        // Accept only safe non-address names that match policy or appear explicitly in the allow-list.
        if (candidate.empty() || candidate.find(':') != std::string::npos)
            return false;
        const auto normalized = lower_trim_copy(candidate);
        return is_uav_hostname(candidate, pattern) ||
            std::any_of(known_peers.begin(), known_peers.end(),
                        [&](const auto& peer) {
                            // Accept a candidate name when it matches either the UAV pattern or explicit peer list.
                            return lower_trim_copy(peer) == normalized;
                        });
    };
    if (resolves(device.name)) {
        return device.name;
    }
    if (resolves(device.alias)) {
        return device.alias;
    }
    return {};
}

}  // namespace mrs_uav_bluetooth::util
