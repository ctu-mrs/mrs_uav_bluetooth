// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <algorithm>
#include <regex>
#include <unistd.h>

namespace mrs_uav_bluetooth::util {

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

bool is_uav_hostname(std::string_view value, std::string_view pattern) {
    std::string trimmed;
    auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return false;
    auto end_pos = value.find_last_not_of(" \t\r\n");
    trimmed = std::string(value.substr(start, end_pos - start + 1));
    std::transform(trimmed.begin(), trimmed.end(), trimmed.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    try {
        std::regex re(std::string(pattern), std::regex::icase);
        return std::regex_match(trimmed, re);
    } catch (...) {
        return false;
    }
}

}  // namespace mrs_uav_bluetooth::util
