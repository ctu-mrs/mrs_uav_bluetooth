// SPDX-License-Identifier: MIT
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
    {"time_ns", {'Q', 8}},
};

}  // namespace

char struct_format_for(const std::string& value_type) {
    auto it = kTypeMeta.find(value_type);
    return it != kTypeMeta.end() ? it->second.fmt : '\0';
}

size_t wire_size_for(const std::string& value_type) {
    auto it = kTypeMeta.find(value_type);
    return it != kTypeMeta.end() ? it->second.size : 0;
}

size_t payload_wire_size(const std::vector<BridgeMemberSpec>& specs) {
    size_t total = 0;
    for (const auto& s : specs) {
        total += wire_size_for(s.value_type);
    }
    return total;
}

std::string normalize_value_type(const std::string& raw) {
    auto it = kTypeAliases.find(raw);
    if (it != kTypeAliases.end()) {
        return it->second;
    }
    return raw;
}

void validate_shared_topics(const std::vector<SharedTopicConfig>& topics) {
    for (size_t i = 0; i < topics.size(); ++i) {
        for (const auto& m : topics[i].member_specs) {
            std::string vt = normalize_value_type(m.value_type);
            if (kTypeMeta.find(vt) == kTypeMeta.end()) {
                throw std::runtime_error(
                    "shared_topics[" + std::to_string(i) + "].members: unsupported type '" +
                    m.value_type + "' for path '" + m.path + "'");
            }
        }
    }
}

}  // namespace mrs_uav_bluetooth::config
