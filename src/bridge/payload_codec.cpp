// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bridge/payload_codec.hpp"

#include <cstring>
#include <stdexcept>
#include <unordered_map>

namespace mrs_uav_bluetooth::bridge {

namespace {

struct TypeInfo {
    size_t size;
};

const std::unordered_map<std::string, TypeInfo> kTypeInfo = {
    {"bool",    {1}},
    {"int8",    {1}},
    {"uint8",   {1}},
    {"int16",   {2}},
    {"uint16",  {2}},
    {"int32",   {4}},
    {"uint32",  {4}},
    {"int64",   {8}},
    {"uint64",  {8}},
    {"float32", {4}},
    {"float64", {8}},
    {"time_ns", {8}},
};

// Pack a single ScalarValue into the buffer at the given offset.
void pack_value(std::vector<uint8_t>& buf, const ScalarValue& val,
                const std::string& value_type) {
    auto it = kTypeInfo.find(value_type);
    if (it == kTypeInfo.end()) {
        throw std::runtime_error("Unknown value type: " + value_type);
    }
    size_t sz = it->second.size;
    size_t off = buf.size();
    buf.resize(off + sz);

    // Little-endian pack.
    std::visit([&](auto&& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, bool>) {
            uint8_t b = v ? 1 : 0;
            std::memcpy(buf.data() + off, &b, 1);
        } else if constexpr (std::is_same_v<T, float>) {
            std::memcpy(buf.data() + off, &v, 4);
        } else if constexpr (std::is_same_v<T, double>) {
            std::memcpy(buf.data() + off, &v, 8);
        } else {
            // Integer types — copy the raw bytes (assumes little-endian host,
            // which is the case on all target ARM64 and x86 boards).
            std::memcpy(buf.data() + off, &v, sizeof(v));
        }
    }, val);
}

// Unpack a single value from the buffer at the given offset.
ScalarValue unpack_value(const std::vector<uint8_t>& buf, size_t off,
                         const std::string& value_type) {
    auto it = kTypeInfo.find(value_type);
    if (it == kTypeInfo.end()) {
        throw std::runtime_error("Unknown value type: " + value_type);
    }
    size_t sz = it->second.size;
    if (off + sz > buf.size()) {
        throw std::runtime_error("Payload too short while decoding " + value_type);
    }

    if (value_type == "bool") {
        uint8_t b = 0;
        std::memcpy(&b, buf.data() + off, 1);
        return static_cast<bool>(b != 0);
    }
    if (value_type == "int8") {
        int8_t v;
        std::memcpy(&v, buf.data() + off, 1);
        return v;
    }
    if (value_type == "uint8") {
        uint8_t v;
        std::memcpy(&v, buf.data() + off, 1);
        return v;
    }
    if (value_type == "int16") {
        int16_t v;
        std::memcpy(&v, buf.data() + off, 2);
        return v;
    }
    if (value_type == "uint16") {
        uint16_t v;
        std::memcpy(&v, buf.data() + off, 2);
        return v;
    }
    if (value_type == "int32") {
        int32_t v;
        std::memcpy(&v, buf.data() + off, 4);
        return v;
    }
    if (value_type == "uint32") {
        uint32_t v;
        std::memcpy(&v, buf.data() + off, 4);
        return v;
    }
    if (value_type == "int64") {
        int64_t v;
        std::memcpy(&v, buf.data() + off, 8);
        return v;
    }
    if (value_type == "uint64" || value_type == "time_ns") {
        uint64_t v;
        std::memcpy(&v, buf.data() + off, 8);
        return v;
    }
    if (value_type == "float32") {
        float v;
        std::memcpy(&v, buf.data() + off, 4);
        return v;
    }
    if (value_type == "float64") {
        double v;
        std::memcpy(&v, buf.data() + off, 8);
        return v;
    }
    throw std::runtime_error("Unhandled value type: " + value_type);
}

std::string trim_copy(const std::string& raw) {
    const auto begin = raw.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = raw.find_last_not_of(" \t\r\n");
    return raw.substr(begin, end - begin + 1);
}

int parse_non_negative_int(const std::string& raw, const std::string& context) {
    const auto token = trim_copy(raw);
    if (token.empty()) {
        throw std::invalid_argument("Missing integer in member path segment: " + context);
    }
    size_t parsed = 0;
    const auto value = std::stoi(token, &parsed, 10);
    if (parsed != token.size() || value < 0) {
        throw std::invalid_argument("Invalid non-negative integer in member path segment: " + context);
    }
    return value;
}

}  // namespace

std::vector<PathSegment> parse_member_path(const std::string& path) {
    if (path.empty()) {
        throw std::invalid_argument("Member path must not be empty");
    }
    std::vector<PathSegment> segments;
    std::string::size_type start = 0;
    while (start < path.size()) {
        auto dot = path.find('.', start);
        std::string raw = (dot == std::string::npos)
                              ? path.substr(start)
                              : path.substr(start, dot - start);
        start = (dot == std::string::npos) ? path.size() : dot + 1;

        PathSegment seg;
        const auto bracket_open = raw.find('[');
        if (bracket_open == std::string::npos) {
            seg.name = raw;
        } else {
            const auto bracket_close = raw.find(']', bracket_open);
            if (bracket_close == std::string::npos || bracket_close + 1 != raw.size()) {
                throw std::invalid_argument("Invalid member path segment: " + raw);
            }
            seg.name = raw.substr(0, bracket_open);
            const auto selector = raw.substr(bracket_open + 1, bracket_close - bracket_open - 1);
            if (selector.find(':') == std::string::npos) {
                seg.index = parse_non_negative_int(selector, raw);
            } else {
                seg.has_slice = true;
                std::vector<std::string> parts;
                size_t selector_start = 0;
                while (selector_start <= selector.size()) {
                    const auto colon = selector.find(':', selector_start);
                    parts.push_back(selector.substr(
                        selector_start,
                        colon == std::string::npos ? std::string::npos : colon - selector_start));
                    if (colon == std::string::npos) {
                        break;
                    }
                    selector_start = colon + 1;
                }
                if (parts.size() > 3) {
                    throw std::invalid_argument("Invalid slice syntax in member path segment: " + raw);
                }
                if (!trim_copy(parts[0]).empty()) {
                    seg.slice_start = parse_non_negative_int(parts[0], raw);
                }
                if (parts.size() >= 2 && !trim_copy(parts[1]).empty()) {
                    seg.slice_stop = parse_non_negative_int(parts[1], raw);
                }
                if (parts.size() == 3 && !trim_copy(parts[2]).empty()) {
                    seg.slice_step = parse_non_negative_int(parts[2], raw);
                    if (seg.slice_step <= 0) {
                        throw std::invalid_argument("Slice step must be positive in member path segment: " + raw);
                    }
                }
            }
        }

        if (seg.name.empty()) {
            throw std::invalid_argument("Invalid member path segment: " + raw);
        }
        segments.push_back(std::move(seg));
    }
    return segments;
}

std::vector<uint8_t> encode_struct_payload(
    const std::vector<ScalarValue>& values,
    const std::vector<config::BridgeMemberSpec>& specs) {
    if (values.size() != specs.size()) {
        throw std::runtime_error("Value count does not match member spec count");
    }
    std::vector<uint8_t> buf;
    buf.reserve(total_wire_size(specs));
    for (size_t i = 0; i < specs.size(); ++i) {
        pack_value(buf, values[i], specs[i].value_type);
    }
    return buf;
}

std::vector<ScalarValue> decode_struct_payload(
    const std::vector<uint8_t>& payload,
    const std::vector<config::BridgeMemberSpec>& specs) {
    std::vector<ScalarValue> values;
    values.reserve(specs.size());
    size_t off = 0;
    for (const auto& spec : specs) {
        values.push_back(unpack_value(payload, off, spec.value_type));
        off += wire_size(spec.value_type);
    }
    return values;
}

ScalarValue coerce_outgoing(double value, const std::string& value_type) {
    if (value_type == "bool") return static_cast<bool>(value != 0.0);
    if (value_type == "int8") return static_cast<int8_t>(value);
    if (value_type == "uint8") return static_cast<uint8_t>(value);
    if (value_type == "int16") return static_cast<int16_t>(value);
    if (value_type == "uint16") return static_cast<uint16_t>(value);
    if (value_type == "int32") return static_cast<int32_t>(value);
    if (value_type == "uint32") return static_cast<uint32_t>(value);
    if (value_type == "int64") return static_cast<int64_t>(value);
    if (value_type == "uint64") return static_cast<uint64_t>(value);
    if (value_type == "time_ns") return static_cast<uint64_t>(value);
    if (value_type == "float32") return static_cast<float>(value);
    if (value_type == "float64") return value;
    throw std::runtime_error("Unknown value type for coercion: " + value_type);
}

double coerce_incoming(const ScalarValue& value, const std::string& /*value_type*/) {
    return std::visit([](auto&& v) -> double {
        return static_cast<double>(v);
    }, value);
}

std::pair<int32_t, uint32_t> ns_to_stamp(uint64_t ns) {
    return {static_cast<int32_t>(ns / 1'000'000'000ULL),
            static_cast<uint32_t>(ns % 1'000'000'000ULL)};
}

uint64_t stamp_to_ns(int32_t sec, uint32_t nanosec) {
    return static_cast<uint64_t>(sec) * 1'000'000'000ULL + nanosec;
}

size_t wire_size(const std::string& value_type) {
    auto it = kTypeInfo.find(value_type);
    return it != kTypeInfo.end() ? it->second.size : 0;
}

size_t total_wire_size(const std::vector<config::BridgeMemberSpec>& specs) {
    size_t total = 0;
    for (const auto& s : specs) {
        total += wire_size(s.value_type);
    }
    return total;
}

}  // namespace mrs_uav_bluetooth::bridge
