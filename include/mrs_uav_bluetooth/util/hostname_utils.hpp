// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/hostname_utils.hpp
/// \brief Declares the hostname utils component of the shared utility layer.

#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

/// Return the system hostname (gethostname).
/// \return Local hostname without surrounding whitespace.
std::string system_hostname();

/// True when the hostname matches the given regex pattern (default: ^uav[0-9]{1,5}$).
/// \param value Hostname to validate.
/// \param pattern Regular expression that valid UAV hostnames must match completely.
/// \return True when UAV hostname; otherwise false.
bool is_uav_hostname(std::string_view value,
                     std::string_view pattern = "^uav[0-9]{1,5}$");

}  // namespace mrs_uav_bluetooth::util
