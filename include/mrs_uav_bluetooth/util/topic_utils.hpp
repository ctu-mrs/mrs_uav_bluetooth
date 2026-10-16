// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/topic_utils.hpp
/// \brief Declares the topic utils component of the shared utility layer.

#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

/// Replace non-alphanumeric/underscore chars with '_', strip leading/trailing '_'.
/// Returns "ble_device" when the result would be empty.
/// \param value Arbitrary text to convert into one ROS topic component.
/// \return ROS-safe topic component derived from arbitrary text.
std::string sanitize_topic_suffix(std::string_view value);

/// Normalize a ROS topic path: collapse multiple '/', ensure leading '/', strip trailing '/'.
/// \param value ROS topic path to make absolute and canonical.
/// \return Canonical absolute ROS topic without duplicate separators.
std::string normalize_ros_topic(std::string_view value);

/// Fixed namespace for this UAV's Bluetooth topics and services.
/// \param hostname UAV name used as the first ROS path component.
/// \return Absolute /<hostname>/bluetooth path.
std::string node_prefix(std::string_view hostname);

/// Resolve a configured ROS path; ~ denotes the package's fixed namespace.
/// \param value Topic path, with optional {hostname} and leading ~ or ~/.
/// \param hostname UAV name used for placeholders and the package namespace.
/// \return Absolute path with normalized separators; empty input resolves to /.
std::string resolve_topic_path(std::string_view value, std::string_view hostname);

}  // namespace mrs_uav_bluetooth::util
