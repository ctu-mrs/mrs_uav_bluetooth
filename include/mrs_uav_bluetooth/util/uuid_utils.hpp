// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/uuid_utils.hpp
/// \brief Declares the uuid utils component of the shared utility layer.

#pragma once

#include <string>
#include <string_view>

namespace mrs_uav_bluetooth::util {

/// MD5-based UUID from a UTF-8 name (lowercase hex, 8-4-4-4-12).
/// \param name Stable application name used as the UUID seed.
/// \return RFC 4122 version-3 UUID derived from the package namespace and name.
std::string uuid_from_name(std::string_view name);

/// True when the string matches the standard UUID pattern (8-4-4-4-12 hex).
/// \param value Text to validate as a UUID.
/// \return True when UUID; otherwise false.
bool is_uuid(std::string_view value);

/// If already a UUID, return lowered; otherwise derive via uuid_from_name.
/// \param value Literal UUID or stable application name to resolve.
/// \return Canonical literal UUID or deterministic name-derived UUID.
std::string resolve_uuid(std::string_view value);

/// Derive a stable UUID in the service-name namespace.
/// \param name Service name used to derive a stable UUID.
/// \return Deterministic service UUID for the supplied name.
std::string named_service_uuid(std::string_view name);

/// Derive a stable UUID in the characteristic-name namespace.
/// \param name Characteristic name used to derive a stable UUID.
/// \return Deterministic characteristic UUID for the supplied name.
std::string named_characteristic_uuid(std::string_view name);

/// Derive a stable UUID in the descriptor-name namespace.
/// \param name Descriptor name used to derive a stable UUID.
/// \return Deterministic descriptor UUID for the supplied name.
std::string named_descriptor_uuid(std::string_view name);

}  // namespace mrs_uav_bluetooth::util
