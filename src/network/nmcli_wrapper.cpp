// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/network/nmcli_wrapper.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <sstream>

namespace mrs_uav_bluetooth::network {

namespace {

std::string quote_arg(const std::string& value) {
    std::string out = "\"";
    for (char c : value) {
        if (c == '\"' || c == '\\') {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    out.push_back('\"');
    return out;
}

}  // namespace

NmcliWrapper::NmcliWrapper(std::string executable)
    : executable_(std::move(executable)) {}

bool NmcliWrapper::available() const {
    auto path = std::filesystem::path(executable_);
    if (path.is_absolute()) {
        return std::filesystem::exists(path);
    }
    return !executable_.empty();
}

std::pair<bool, std::string> NmcliWrapper::run(const std::vector<std::string>& args,
                                               int /*timeout_s*/) const {
    if (!available()) {
        return {false, "nmcli not available"};
    }
    std::ostringstream cmd;
    cmd << executable_;
    for (const auto& arg : args) {
        cmd << ' ' << quote_arg(arg);
    }
    std::array<char, 512> buffer{};
    std::string output;
    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        return {false, "failed to spawn nmcli"};
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        output += buffer.data();
    }
    int rc = pclose(pipe);
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }
    if (rc == 0) {
        return {true, output};
    }
    if (output.empty()) {
        output = "nmcli exited with code " + std::to_string(rc);
    }
    return {false, output};
}

std::string NmcliWrapper::get_current_ssid() const {
    auto [ok, detail] = run({"--terse", "--escape", "no", "--fields",
                             "DEVICE,TYPE,STATE,CONNECTION", "device", "status"}, 10);
    if (!ok) {
        return {};
    }
    std::istringstream lines(detail);
    std::string line;
    while (std::getline(lines, line)) {
        std::vector<std::string> parts;
        std::stringstream ss(line);
        std::string part;
        while (std::getline(ss, part, ':')) {
            parts.push_back(part);
        }
        if (parts.size() != 4) {
            continue;
        }
        if (parts[1] != "wifi") {
            continue;
        }
        if (parts[2].rfind("connected", 0) != 0 && parts[2].rfind("connecting", 0) != 0) {
            continue;
        }
        if (!parts[3].empty() && parts[3] != "--") {
            return parts[3];
        }
    }
    return {};
}

std::vector<std::string> NmcliWrapper::list_visible_ssids() const {
    auto [ok, detail] = run({"--terse", "--escape", "no", "--fields", "SSID",
                             "device", "wifi", "list", "--rescan", "no"}, 15);
    if (!ok) {
        return {};
    }
    std::vector<std::string> result;
    std::istringstream lines(detail);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.empty()) {
            continue;
        }
        if (std::find(result.begin(), result.end(), line) == result.end()) {
            result.push_back(line);
        }
    }
    return result;
}

std::pair<bool, std::string> NmcliWrapper::connect(const std::string& ssid,
                                                   const std::string& password) const {
    std::vector<std::string> args = {"--wait", "20", "device", "wifi", "connect", ssid};
    if (!password.empty()) {
        args.push_back("password");
        args.push_back(password);
    }
    return run(args, 25);
}

}  // namespace mrs_uav_bluetooth::network
