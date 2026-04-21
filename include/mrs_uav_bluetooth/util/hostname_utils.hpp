// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

/// Return the system hostname (gethostname).
std::string system_hostname();

/// True when the hostname matches the given regex pattern (default: ^uav[0-9]{1,2}$).
bool is_uav_hostname(std::string_view value,
                     std::string_view pattern = "^uav[0-9]{1,2}$");

}  // namespace mrs_uav_bluetooth::util
