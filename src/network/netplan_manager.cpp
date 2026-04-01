// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/network/netplan_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace mrs_uav_bluetooth::network {

namespace {

constexpr const char* kIwGetIdCommand = "iwgetid -r 2>/dev/null";
constexpr auto kConnectionVerifyTimeout = std::chrono::seconds(20);
constexpr auto kConnectionVerifyPollInterval = std::chrono::milliseconds(500);

bool contains_value(const std::vector<std::string>& values, const std::string& candidate) {
    return std::find(values.begin(), values.end(), candidate) != values.end();
}

std::string trim_ascii_whitespace(std::string value) {
    const auto is_space = [](unsigned char ch) {
        return ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t';
    };
    while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
        value.pop_back();
    }
    auto first = std::find_if_not(value.begin(), value.end(), [&](unsigned char ch) {
        return is_space(ch);
    });
    value.erase(value.begin(), first);
    return value;
}

std::string read_access_point_ssid(const YAML::Node& config) {
    const auto access_points = config["network"]["wifis"]["wlan0"]["access-points"];
    if (!access_points || !access_points.IsMap()) {
        return {};
    }
    for (auto ap = access_points.begin(); ap != access_points.end(); ++ap) {
        return ap->first.as<std::string>("");
    }
    return {};
}

std::string read_access_point_password(const YAML::Node& config) {
    const auto access_points = config["network"]["wifis"]["wlan0"]["access-points"];
    if (!access_points || !access_points.IsMap()) {
        return {};
    }
    for (auto ap = access_points.begin(); ap != access_points.end(); ++ap) {
        const auto ap_cfg = ap->second;
        if (!ap_cfg || !ap_cfg.IsMap() || !ap_cfg["password"]) {
            return {};
        }
        return ap_cfg["password"].as<std::string>("");
    }
    return {};
}

std::string read_command_output(const char* command) {
    std::array<char, 256> buffer{};
    std::string output;

    struct PipeCloser {
        void operator()(FILE* file) const {
            if (file != nullptr) {
                pclose(file);
            }
        }
    };

    std::unique_ptr<FILE, PipeCloser> pipe(popen(command, "r"));
    if (!pipe) {
        return {};
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        output.append(buffer.data());
    }
    return trim_ascii_whitespace(std::move(output));
}

std::string read_connected_ssid() {
    return read_command_output(kIwGetIdCommand);
}

struct FileSnapshot {
    bool existed{false};
    std::string contents;
};

FileSnapshot take_file_snapshot(const std::string& path) {
    FileSnapshot snapshot;
    std::ifstream input(path, std::ios::binary);
    if (!input.good()) {
        return snapshot;
    }

    snapshot.existed = true;
    std::ostringstream stream;
    stream << input.rdbuf();
    snapshot.contents = stream.str();
    return snapshot;
}

std::string restore_snapshot(const std::string& path, const FileSnapshot& snapshot) {
    namespace fs = std::filesystem;

    if (!snapshot.existed) {
        std::error_code error;
        fs::remove(path, error);
        if (error) {
            return error.message();
        }
        return {};
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output.good()) {
        return "failed to reopen config file for rollback";
    }
    output << snapshot.contents;
    if (!output.good()) {
        return "failed to restore previous config contents";
    }
    return {};
}

std::string run_netplan_apply() {
    const int rc = std::system("netplan apply");
    if (rc == 0) {
        return {};
    }
    return "netplan apply failed with code " + std::to_string(rc);
}

bool wait_for_connected_ssid(const std::string& expected_ssid) {
    const auto deadline = std::chrono::steady_clock::now() + kConnectionVerifyTimeout;
    while (true) {
        if (read_connected_ssid() == expected_ssid) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(kConnectionVerifyPollInterval);
    }
}

std::pair<bool, std::string> rollback_netplan_change(const std::string& path,
                                                     const FileSnapshot& snapshot,
                                                     std::string message) {
    const auto restore_error = restore_snapshot(path, snapshot);
    const auto rollback_apply_error = restore_error.empty() ? run_netplan_apply() : std::string{};

    if (!restore_error.empty()) {
        message += "; rollback write failed: " + restore_error;
    } else if (!rollback_apply_error.empty()) {
        message += "; rollback apply failed: " + rollback_apply_error;
    } else {
        message += "; previous netplan config restored";
    }

    return {false, std::move(message)};
}

std::string default_uav_static_address() {
    char hostname_buf[256]{};
    std::string hostname;
    if (gethostname(hostname_buf, sizeof(hostname_buf) - 1) == 0) {
        hostname = hostname_buf;
    }

    std::string lowered = hostname;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    std::string digits = "00";
    const auto uav_pos = lowered.find("uav");
    if (uav_pos != std::string::npos) {
        size_t idx = uav_pos + 3;
        std::string parsed;
        while (idx < lowered.size() && std::isdigit(static_cast<unsigned char>(lowered[idx]))) {
            parsed.push_back(lowered[idx]);
            ++idx;
        }
        if (!parsed.empty()) {
            if (parsed.size() == 1) {
                digits = "0" + parsed;
            } else {
                digits = parsed.substr(parsed.size() - 2);
            }
        }
    }

    return "192.168.69.1" + digits + "/24";
}

}  // namespace

NetplanManager::NetplanManager(std::string netplan_config_file,
                               std::vector<std::string> allowed_networks)
    : netplan_config_file_(std::move(netplan_config_file)),
      allowed_networks_(std::move(allowed_networks)) {}

bool NetplanManager::busy() {
    std::lock_guard<std::mutex> lock(mutex_);
    return busy_;
}

void NetplanManager::set_config_file(std::string value) {
    std::lock_guard<std::mutex> lock(mutex_);
    netplan_config_file_ = trim_ascii_whitespace(std::move(value));
    if (netplan_config_file_.empty()) {
        netplan_config_file_ = "/etc/netplan/01-netcfg.yaml";
    }
}

void NetplanManager::set_allowed_networks(std::vector<std::string> value) {
    std::lock_guard<std::mutex> lock(mutex_);
    allowed_networks_ = std::move(value);
}

std::string NetplanManager::get_current_ssid() {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        const auto ssid = read_access_point_ssid(YAML::LoadFile(netplan_config_file_));
        if (!ssid.empty()) {
            return ssid;
        }
    } catch (...) {
    }
    return read_command_output(kIwGetIdCommand);
}

std::string NetplanManager::get_configured_password() {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        return read_access_point_password(YAML::LoadFile(netplan_config_file_));
    } catch (...) {
    }
    return {};
}

std::vector<std::string> NetplanManager::list_known_ssids() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> result;
    for (const auto& ssid : allowed_networks_) {
        if (!ssid.empty() && !contains_value(result, ssid)) {
            result.push_back(ssid);
        }
    }
    return result;
}

std::pair<bool, std::string> NetplanManager::write_netplan(
    const std::string& ssid,
    const std::optional<std::string>& password) {
    busy_ = true;
    const auto snapshot = take_file_snapshot(netplan_config_file_);

    YAML::Node config(YAML::NodeType::Map);
    std::string previous_password;
    try {
        if (std::filesystem::exists(netplan_config_file_)) {
            config = YAML::LoadFile(netplan_config_file_);
            previous_password = read_access_point_password(config);
        }
    } catch (const std::exception& e) {
        busy_ = false;
        return {false, std::string("failed to parse netplan config: ") + e.what()};
    }

    auto network = config["network"];
    if (!network || !network.IsMap()) {
        network = config["network"] = YAML::Node(YAML::NodeType::Map);
    }
    auto wifis = network["wifis"];
    if (!wifis || !wifis.IsMap() || wifis.size() == 0) {
        wifis = network["wifis"] = YAML::Node(YAML::NodeType::Map);
        wifis["wlan0"] = YAML::Node(YAML::NodeType::Map);
    }
    auto iface_cfg = wifis["wlan0"];
    if (!iface_cfg || !iface_cfg.IsMap()) {
        iface_cfg = YAML::Node(YAML::NodeType::Map);
        iface_cfg["dhcp4"] = false;
        iface_cfg["dhcp6"] = false;
        iface_cfg["addresses"] = YAML::Node(YAML::NodeType::Sequence);
        iface_cfg["addresses"].push_back(default_uav_static_address());
    }
    YAML::Node aps(YAML::NodeType::Map);
    YAML::Node ap_cfg(YAML::NodeType::Map);

    if (password.has_value()) {
        const auto trimmed_password = trim_ascii_whitespace(*password);
        if (!trimmed_password.empty()) {
            ap_cfg["password"] = trimmed_password;
        } else if (!previous_password.empty()) {
            ap_cfg["password"] = previous_password;
        }
    } else if (!previous_password.empty()) {
        ap_cfg["password"] = previous_password;
    }
    aps[ssid] = ap_cfg;
    iface_cfg["access-points"] = aps;
    wifis["wlan0"] = iface_cfg;

    try {
        std::ofstream out(netplan_config_file_, std::ios::binary | std::ios::trunc);
        out << config;
        if (!out.good()) {
            throw std::runtime_error("failed to flush updated config to disk");
        }
    } catch (const std::exception& e) {
        busy_ = false;
        return {false, e.what()};
    }

    const auto apply_error = run_netplan_apply();
    if (!apply_error.empty()) {
        busy_ = false;
        return rollback_netplan_change(netplan_config_file_, snapshot, apply_error);
    }

    if (!wait_for_connected_ssid(ssid)) {
        const auto connected_ssid = read_connected_ssid();
        std::string message = "failed to connect to SSID '" + ssid + "' within " +
                              std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                                 kConnectionVerifyTimeout)
                                                 .count()) +
                              "s";
        if (!connected_ssid.empty()) {
            message += "; connected SSID remained '" + connected_ssid + "'";
        } else {
            message += "; no active Wi-Fi connection detected";
        }
        busy_ = false;
        return rollback_netplan_change(netplan_config_file_, snapshot, std::move(message));
    }

    busy_ = false;
    return {true, "connected to SSID '" + ssid + "'"};
}

std::pair<bool, std::string> NetplanManager::set_current_network(const std::string& ssid,
                                                                 std::optional<std::string> password) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_) {
        return {false, "netplan change already in progress"};
    }
    auto target = ssid;
    if (target.empty()) {
        return {false, "empty target SSID"};
    }
    if (!allowed_networks_.empty() && !contains_value(allowed_networks_, target)) {
        return {false, "SSID not present in allowed_wifi_networks"};
    }
    if (password.has_value() && trim_ascii_whitespace(*password).empty()) {
        password.reset();
    }
    return write_netplan(target, password);
}

std::pair<bool, std::string> NetplanManager::set_current_ssid(const std::string& target) {
    return set_current_network(target, std::nullopt);
}

}  // namespace mrs_uav_bluetooth::network
