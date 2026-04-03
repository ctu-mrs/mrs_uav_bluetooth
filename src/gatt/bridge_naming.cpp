// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/gatt/bridge_naming.hpp"

#include <string_view>

namespace mrs_uav_bluetooth::gatt {

std::string bridge_characteristic_name_for_service(const std::string& bridge_name) {
    constexpr std::string_view prefix{"bridge:"};
    if (bridge_name.rfind(prefix.data(), 0) == 0) {
        return bridge_name.substr(prefix.size());
    }
    return bridge_name + "/value";
}

}  // namespace mrs_uav_bluetooth::gatt