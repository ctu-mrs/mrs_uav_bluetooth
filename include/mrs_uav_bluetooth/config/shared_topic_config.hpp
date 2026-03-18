// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <string>
#include <vector>

namespace mrs_uav_bluetooth::config {

/// Return the struct-pack format character for a bridge member value_type.
/// Returns '\0' if the type is unknown.
char struct_format_for(const std::string& value_type);

/// Compute the wire size in bytes for a single member type.
/// Returns 0 for unknown types.
size_t wire_size_for(const std::string& value_type);

/// Compute the total packed payload size for a list of member specs.
size_t payload_wire_size(const std::vector<BridgeMemberSpec>& specs);

/// Normalize a bridge member value_type string (e.g. "boolean" → "bool",
/// "double" → "float64").  Returns the raw string if no alias matches.
std::string normalize_value_type(const std::string& raw);

/// Validate that all member types in shared_topics are supported.
/// Throws std::runtime_error on the first invalid type.
void validate_shared_topics(const std::vector<SharedTopicConfig>& topics);

}  // namespace mrs_uav_bluetooth::config
