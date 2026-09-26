// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/util/device_utils.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <algorithm>

namespace mrs_uav_bluetooth::util {

std::string device_display_name(const bluez::DeviceInfo& device) {
    if (!device.alias.empty()) {
        return device.alias;
    }
    if (!device.name.empty()) {
        return device.name;
    }
    return "?";
}

std::string device_hostname_guess(const bluez::DeviceInfo& device,
                                  std::string_view pattern,
                                  const std::vector<std::string>& known_peers) {
    const auto resolves = [&](const std::string& candidate) {
        if (candidate.empty() || candidate.find(':') != std::string::npos)
            return false;
        const auto normalized = lower_trim_copy(candidate);
        return is_uav_hostname(candidate, pattern) ||
            std::any_of(known_peers.begin(), known_peers.end(),
                        [&](const auto& peer) {
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
