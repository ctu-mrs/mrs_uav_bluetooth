// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <algorithm>
#include <regex>

namespace mrs_uav_bluetooth::util {

std::string sanitize_topic_suffix(std::string_view value) {
    // Trim whitespace.
    auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return "ble_device";
    auto end = value.find_last_not_of(" \t\r\n");
    std::string trimmed(value.substr(start, end - start + 1));

    // Replace non-alphanumeric/underscore with '_'.
    std::string cleaned;
    cleaned.reserve(trimmed.size());
    for (char c : trimmed) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
            cleaned.push_back(c);
        } else {
            cleaned.push_back('_');
        }
    }

    // Strip leading/trailing underscores.
    auto first = cleaned.find_first_not_of('_');
    if (first == std::string::npos) return "ble_device";
    auto last = cleaned.find_last_not_of('_');
    auto result = cleaned.substr(first, last - first + 1);
    return result.empty() ? "ble_device" : result;
}

std::string normalize_ros_topic(std::string_view value) {
    // Trim whitespace.
    auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return "/";
    auto end = value.find_last_not_of(" \t\r\n");
    std::string candidate(value.substr(start, end - start + 1));

    if (candidate.empty()) return "/";

    // Collapse multiple slashes.
    std::string collapsed;
    collapsed.reserve(candidate.size());
    bool last_was_slash = false;
    for (char c : candidate) {
        if (c == '/') {
            if (!last_was_slash) {
                collapsed.push_back(c);
                last_was_slash = true;
            }
        } else {
            collapsed.push_back(c);
            last_was_slash = false;
        }
    }

    // Ensure leading '/'.
    if (collapsed.empty() || collapsed[0] != '/') {
        collapsed = "/" + collapsed;
    }

    // Strip trailing '/'.
    while (collapsed.size() > 1 && collapsed.back() == '/') {
        collapsed.pop_back();
    }

    return collapsed.empty() ? "/" : collapsed;
}

}  // namespace mrs_uav_bluetooth::util
