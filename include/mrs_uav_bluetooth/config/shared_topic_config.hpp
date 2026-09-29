// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/config/shared_topic_config.hpp
/// \brief Declares the shared topic config component of the YAML configuration layer.

#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <string>
#include <vector>

namespace mrs_uav_bluetooth::config {

/// Return the struct-pack format character for a bridge member value_type.
/// Returns '\0' if the type is unknown.
/// \param value_type Canonical scalar type whose struct-format code is requested.
/// \return Struct format code for the normalized scalar type.
char struct_format_for(const std::string& value_type);

/// Compute the wire size in bytes for a single member type.
/// Returns 0 for unknown types.
/// \param value_type Canonical scalar type whose encoded byte width is requested.
/// \return Fixed encoded width for the selected scalar alternative.
size_t wire_size_for(const std::string& value_type);

/// Compute the total packed payload size for a list of member specs.
/// \param specs ordered member specifications defining the payload layout.
/// \return Exact encoded size in bytes.
size_t payload_wire_size(const std::vector<BridgeMemberSpec>& specs);

/// Normalize a bridge member value_type string (e.g. "boolean" → "bool",
/// "double" → "float64").  Returns the raw string if no alias matches.
/// \param raw Configured scalar type name or alias to canonicalize.
/// \return Canonical scalar type name.
std::string normalize_value_type(const std::string& raw);

/// Validate that all member types in shared_topics are supported.
/// Throws std::runtime_error on the first invalid type.
/// \param topics All configured topic bridges checked for unique identities and valid codecs.
void validate_shared_topics(const std::vector<SharedTopicConfig>& topics);

}  // namespace mrs_uav_bluetooth::config
