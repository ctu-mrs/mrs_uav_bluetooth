// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>

namespace mrs_uav_bluetooth::util {

/// Convert builtin_interfaces::msg::Time to a single uint64 nanosecond value.
inline uint64_t time_to_ns(const builtin_interfaces::msg::Time& t) {
    return static_cast<uint64_t>(t.sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(t.nanosec);
}

/// Convert a uint64 nanosecond value to builtin_interfaces::msg::Time.
inline builtin_interfaces::msg::Time ns_to_time(uint64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1'000'000'000ULL);
    t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
    return t;
}

}  // namespace mrs_uav_bluetooth::util
