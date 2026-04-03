// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace mrs_uav_bluetooth::bridge {

/// A typed scalar value that can be packed/unpacked in a bridge payload.
using ScalarValue = std::variant<
    bool,
    int8_t, uint8_t,
    int16_t, uint16_t,
    int32_t, uint32_t,
    int64_t, uint64_t,
    float, double>;

/// Parsed segment of a dotted member path like "pose.pose.position.x".
struct PathSegment {
    std::string name;
    int index{-1};  // -1 means no array index
};

/// Parse a dotted member path into segments.
/// Throws std::invalid_argument if any segment is malformed.
std::vector<PathSegment> parse_member_path(const std::string& path);

/// Pack a sequence of scalar values according to the member specs into a
/// compact little-endian byte payload.
/// The caller is responsible for extracting values from the ROS message and
/// coercing them appropriately (using coerce_outgoing).
std::vector<uint8_t> encode_struct_payload(
    const std::vector<ScalarValue>& values,
    const std::vector<config::BridgeMemberSpec>& specs);

/// Unpack a compact little-endian byte payload back into scalar values.
/// Throws std::runtime_error if the payload is too short.
std::vector<ScalarValue> decode_struct_payload(
    const std::vector<uint8_t>& payload,
    const std::vector<config::BridgeMemberSpec>& specs);

/// Coerce a double into the appropriate ScalarValue for the given type.
/// time_ns: interprets the double as nanoseconds (uint64_t).
ScalarValue coerce_outgoing(double value, const std::string& value_type);

/// Coerce a decoded ScalarValue into a double.
/// time_ns: returns nanoseconds as double.
double coerce_incoming(const ScalarValue& value, const std::string& value_type);

/// For time_ns: split nanoseconds into (sec, nanosec).
std::pair<int32_t, uint32_t> ns_to_stamp(uint64_t ns);

/// For time_ns: combine (sec, nanosec) into nanoseconds.
uint64_t stamp_to_ns(int32_t sec, uint32_t nanosec);

/// Return the packed wire size in bytes for a single value_type string.
size_t wire_size(const std::string& value_type);

/// Compute total packed size for a list of member specs.
size_t total_wire_size(const std::vector<config::BridgeMemberSpec>& specs);

}  // namespace mrs_uav_bluetooth::bridge
