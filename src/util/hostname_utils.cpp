// SPDX-License-Identifier: BSD-3-Clause
/// \file src/util/hostname_utils.cpp
/// \brief Implements the hostname utils component of the shared utility layer.

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <algorithm>
#include <regex>
#include <unistd.h>

namespace mrs_uav_bluetooth::util {

/// \brief Trim trailing whitespace/newlines.
/// \return Local hostname without surrounding whitespace.
std::string system_hostname() {
    char buf[256]{};
    if (gethostname(buf, sizeof(buf) - 1) == 0) {
        // Trim trailing whitespace/newlines.
        std::string name(buf);
        auto end = name.find_last_not_of(" \t\r\n");
        if (end == std::string::npos) return {};
        return name.substr(0, end + 1);
    }
    return {};
}

/// \brief Validate a trimmed hostname against the configured UAV pattern.
/// \param value Hostname to validate.
/// \param pattern Regular expression that valid UAV hostnames must match completely.
/// \return True if the complete hostname matches the configured pattern; otherwise false.
bool is_uav_hostname(std::string_view value, std::string_view pattern) {
    // Trim surrounding whitespace before admission-policy matching.
    std::string trimmed;
    auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return false;
    auto end_pos = value.find_last_not_of(" \t\r\n");
    trimmed = std::string(value.substr(start, end_pos - start + 1));
    std::transform(trimmed.begin(), trimmed.end(), trimmed.begin(),
                   [](unsigned char c) {
                       // Normalize the candidate before regex matching and numeric-suffix extraction.
                       return std::tolower(c);
                   });
    try {
        std::regex re(std::string(pattern), std::regex::icase);
        return std::regex_match(trimmed, re);
    } catch (...) {
        return false;
    }
}

}  // namespace mrs_uav_bluetooth::util
