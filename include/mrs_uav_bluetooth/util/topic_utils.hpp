// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

/// Replace non-alphanumeric/underscore chars with '_', strip leading/trailing '_'.
/// Returns "ble_device" when the result would be empty.
std::string sanitize_topic_suffix(std::string_view value);

/// Normalize a ROS topic path: collapse multiple '/', ensure leading '/', strip trailing '/'.
std::string normalize_ros_topic(std::string_view value);

}  // namespace mrs_uav_bluetooth::util
