// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/util/device_utils.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

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
                                  std::string_view pattern) {
    if (is_uav_hostname(device.name, pattern)) {
        return device.name;
    }
    if (is_uav_hostname(device.alias, pattern)) {
        return device.alias;
    }
    return {};
}

}  // namespace mrs_uav_bluetooth::util