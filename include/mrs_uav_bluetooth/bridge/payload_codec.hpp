// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/bridge/math_expression.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
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
    bool has_slice{false};
    std::optional<int> slice_start;
    std::optional<int> slice_stop;
    int slice_step{1};
};

/// Parse a dotted member path into segments.
/// Supported array selectors:
/// - field[3]
/// - field[1:4]
/// - field[:4]
/// - field[2:]
/// - field[:] (for fixed-size arrays, or one open-ended dynamic leaf per bridge)
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
/// Integer types round and reject values outside their representable range.
ScalarValue coerce_outgoing(double value, const config::BridgeMemberSpec& spec);

/// Coerce an exact integer relation without passing through double.
ScalarValue coerce_outgoing_integer(ExactInteger value,
                                    const config::BridgeMemberSpec& spec);

/// Coerce a decoded ScalarValue into a double.
/// The expression engine handles any inverse conversion separately.
double coerce_incoming(const ScalarValue& value, const config::BridgeMemberSpec& spec);

/// Return an exact value for integer wire types, or none for floating types.
std::optional<ExactInteger> coerce_incoming_integer(const ScalarValue& value);

/// Return the packed wire size in bytes for a single value_type string.
size_t wire_size(const std::string& value_type);

/// Return the final scalar wire size of a declarative member.
size_t wire_size(const config::BridgeMemberSpec& spec);

/// Compute total packed size for a list of member specs.
size_t total_wire_size(const std::vector<config::BridgeMemberSpec>& specs);

}  // namespace mrs_uav_bluetooth::bridge
