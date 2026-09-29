// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/gatt/bridge_naming.hpp
/// \brief Declares the bridge naming component of the Bluetooth Low Energy GATT layer.

#pragma once

#include <string>

namespace mrs_uav_bluetooth::gatt {

/// \brief Derive the characteristic name paired with a bridge service name.
/// \param bridge_name Stable bridge service name from which the child characteristic name is derived.
/// \return Deterministic characteristic name paired with the bridge service.
std::string bridge_characteristic_name_for_service(const std::string& bridge_name);

}  // namespace mrs_uav_bluetooth::gatt
