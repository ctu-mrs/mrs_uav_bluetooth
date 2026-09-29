// SPDX-License-Identifier: BSD-3-Clause
/// \file src/config/shared_topic_config.cpp
/// \brief Implements the shared topic config component of the YAML configuration layer.

#include "mrs_uav_bluetooth/config/shared_topic_config.hpp"

#include <stdexcept>
#include <unordered_map>

namespace mrs_uav_bluetooth::config {

namespace {

const std::unordered_map<std::string, std::string> kTypeAliases = {
    {"boolean", "bool"},
    {"byte",    "int8"},
    {"octet",   "uint8"},
    {"char",    "uint8"},
    {"float",   "float32"},
    {"double",  "float64"},
};

struct TypeMeta {
    char fmt;
    size_t size;
};

const std::unordered_map<std::string, TypeMeta> kTypeMeta = {
    {"bool",    {'?', 1}},
    {"int8",    {'b', 1}},
    {"uint8",   {'B', 1}},
    {"int16",   {'h', 2}},
    {"uint16",  {'H', 2}},
    {"int32",   {'i', 4}},
    {"uint32",  {'I', 4}},
    {"int64",   {'q', 8}},
    {"uint64",  {'Q', 8}},
    {"float32", {'f', 4}},
    {"float64", {'d', 8}},
};

}  // namespace

/// \brief Map a normalized bridge scalar type to its struct-format code.
/// \param value_type Canonical scalar type whose struct-format code is requested.
/// \return Struct format code for the normalized scalar type.
char struct_format_for(const std::string& value_type) {
    // Map a normalized bridge scalar type to its struct-format code.
    auto it = kTypeMeta.find(value_type);
    return it != kTypeMeta.end() ? it->second.fmt : '\0';
}

/// \brief Return the fixed encoded byte width for the selected scalar alternative.
/// \param value_type Canonical scalar type whose encoded byte width is requested.
/// \return Fixed encoded width for the selected scalar alternative.
size_t wire_size_for(const std::string& value_type) {
    // Return the fixed encoded byte width for the selected scalar alternative.
    auto it = kTypeMeta.find(value_type);
    return it != kTypeMeta.end() ? it->second.size : 0;
}

/// \brief Calculate encoded member bytes plus any framing header required by the transport.
/// \param specs ordered member specifications defining the payload layout.
/// \return Total encoded bridge payload bytes including framing.
size_t payload_wire_size(const std::vector<BridgeMemberSpec>& specs) {
    // Calculate encoded member bytes plus any framing header required by the transport.
    size_t total = 0;
    for (const auto& s : specs) {
        total += wire_size_for(s.value_type);
    }
    return total;
}

/// \brief Map supported scalar aliases to the one canonical codec name.
/// \param raw Configured scalar type name or alias to canonicalize.
/// \return Canonical scalar type name.
std::string normalize_value_type(const std::string& raw) {
    // Resolve supported aliases to the canonical scalar name stored in bridge metadata.
    auto it = kTypeAliases.find(raw);
    if (it != kTypeAliases.end()) {
        return it->second;
    }
    return raw;
}

/// \brief Reject ambiguous bridge identities unsupported codecs and invalid field layouts.
/// \param topics All configured topic bridges checked for unique identities and valid codecs.
void validate_shared_topics(const std::vector<SharedTopicConfig>& topics) {
    // Reject unsupported wire types, ambiguous identities, and incompatible bridge settings.
    for (size_t i = 0; i < topics.size(); ++i) {
        for (const auto& m : topics[i].member_specs) {
            std::string vt = normalize_value_type(m.value_type);
            if (kTypeMeta.find(vt) == kTypeMeta.end()) {
                throw std::runtime_error(
                    "shared_topics[" + std::to_string(i) + "].members_encode: unsupported type '" +
                    m.value_type + "' for target '" + m.target + "'");
            }
        }
    }
}

}  // namespace mrs_uav_bluetooth::config
