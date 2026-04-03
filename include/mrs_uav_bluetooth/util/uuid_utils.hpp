// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

/// MD5-based UUID from a UTF-8 name (lowercase hex, 8-4-4-4-12).
std::string uuid_from_name(std::string_view name);

/// True when the string matches the standard UUID pattern (8-4-4-4-12 hex).
bool is_uuid(std::string_view value);

/// If already a UUID, return lowered; otherwise derive via uuid_from_name.
std::string resolve_uuid(std::string_view value);

/// Convenience wrapper kept for call-site readability.
std::string named_service_uuid(std::string_view name);

/// Convenience wrapper kept for call-site readability.
std::string named_characteristic_uuid(std::string_view name);

/// Convenience wrapper kept for call-site readability.
std::string named_descriptor_uuid(std::string_view name);

}  // namespace mrs_uav_bluetooth::util
