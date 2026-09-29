// SPDX-License-Identifier: BSD-3-Clause
/// \file src/util/string_utils.cpp
/// \brief Implements the string utils component of the shared utility layer.

#include "mrs_uav_bluetooth/util/string_utils.hpp"

#include <algorithm>
#include <cctype>

namespace mrs_uav_bluetooth::util {

/// \brief Return text without leading or trailing ASCII whitespace.
/// \param value Text to trim.
/// \return Copy with surrounding ASCII whitespace removed.
std::string trim_ascii_copy(std::string value) {
    // Trim ascii copy.
    const auto is_space = [](unsigned char ch) {
        // Classify bytes through unsigned char to avoid undefined ctype behavior.
        return ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t';
    };

    while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
        value.pop_back();
    }
    const auto start = std::find_if_not(value.begin(), value.end(), [&](unsigned char ch) {
        // Lowercase the trimmed text using unsigned bytes for locale-safe ctype calls.
        return is_space(ch);
    });
    value.erase(value.begin(), start);
    return value;
}

/// \brief Trim surrounding whitespace and lowercase a copy for policy comparisons.
/// \param value Text to trim.
/// \return Trimmed lowercase copy.
std::string lower_trim_copy(std::string value) {
    // Trim surrounding whitespace and lowercase a copy for policy comparisons.
    value = trim_ascii_copy(std::move(value));
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) {
                       // Preserve alphanumerics and map every other topic-suffix byte to an underscore.
                       return static_cast<char>(std::tolower(c));
                   });
    return value;
}

}  // namespace mrs_uav_bluetooth::util
