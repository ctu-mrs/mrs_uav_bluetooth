// SPDX-License-Identifier: BSD-3-Clause
/// \file src/gatt/bridge_naming.cpp
/// \brief Implements the bridge naming component of the Bluetooth Low Energy GATT layer.

#include "mrs_uav_bluetooth/gatt/bridge_naming.hpp"

#include <string_view>

namespace mrs_uav_bluetooth::gatt {

/// \brief Derive the characteristic name paired with a bridge service name.
/// \param bridge_name Stable bridge service name from which the child characteristic name is derived.
/// \return Deterministic characteristic name paired with the bridge service.
std::string bridge_characteristic_name_for_service(const std::string& bridge_name) {
    // Derive the characteristic name paired with a bridge service name.
    constexpr std::string_view prefix{"bridge:"};
    if (bridge_name.rfind(prefix.data(), 0) == 0) {
        return bridge_name.substr(prefix.size());
    }
    return bridge_name + "/value";
}

}  // namespace mrs_uav_bluetooth::gatt
