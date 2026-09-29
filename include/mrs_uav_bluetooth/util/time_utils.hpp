// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/time_utils.hpp
/// \brief Declares the time utils component of the shared utility layer.

#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>

namespace mrs_uav_bluetooth::util {

/// Convert builtin_interfaces::msg::Time to a single uint64 nanosecond value.
/// \param t ROS timestamp whose seconds and nanoseconds are combined.
/// \return Unsigned nanoseconds since the ROS epoch.
inline uint64_t time_to_ns(const builtin_interfaces::msg::Time& t) {
    // Preserve the full timestamp while collapsing the two ROS fields.
    return static_cast<uint64_t>(t.sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(t.nanosec);
}

/// Convert a uint64 nanosecond value to builtin_interfaces::msg::Time.
/// \param ns nanosecond timestamp encoded into a time payload.
/// \return ROS timestamp split into seconds and remaining nanoseconds.
inline builtin_interfaces::msg::Time ns_to_time(uint64_t ns) {
    // Split at one second so nanosec remains inside its ROS-defined range.
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1'000'000'000ULL);
    t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
    return t;
}

}  // namespace mrs_uav_bluetooth::util
