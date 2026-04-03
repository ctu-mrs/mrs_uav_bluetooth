// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <algorithm>
#include <cctype>

namespace mrs_uav_bluetooth::util {

std::string trim_ascii_copy(std::string value) {
    const auto is_space = [](unsigned char ch) {
        return ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t';
    };

    while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
        value.pop_back();
    }
    const auto start = std::find_if_not(value.begin(), value.end(), [&](unsigned char ch) {
        return is_space(ch);
    });
    value.erase(value.begin(), start);
    return value;
}

std::string lower_trim_copy(std::string value) {
    value = trim_ascii_copy(std::move(value));
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

}  // namespace mrs_uav_bluetooth::util