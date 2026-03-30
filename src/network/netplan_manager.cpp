// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/network/netplan_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace mrs_uav_bluetooth::network {

namespace {

constexpr const char* kPreferredNetplanConfigFile = "/etc/netplan/01-netcfg.yaml";
constexpr const char* kNetplanConfigDirectory = "/etc/netplan";

std::string resolve_netplan_config_file() {
    namespace fs = std::filesystem;

    const fs::path preferred{kPreferredNetplanConfigFile};
    if (fs::exists(preferred)) {
        return preferred.string();
    }

    const fs::path config_dir{kNetplanConfigDirectory};
    if (fs::is_directory(config_dir)) {
        std::vector<fs::path> candidates;
        for (const auto& entry : fs::directory_iterator(config_dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const auto extension = entry.path().extension().string();
            if (extension == ".yaml" || extension == ".yml") {
                candidates.push_back(entry.path());
            }
        }
        std::sort(candidates.begin(), candidates.end());
        if (!candidates.empty()) {
            return candidates.front().string();
        }
    }

    return preferred.string();
}

bool contains_value(const std::vector<std::string>& values, const std::string& candidate) {
    return std::find(values.begin(), values.end(), candidate) != values.end();
}

}  // namespace

NetplanManager::NetplanManager(std::vector<std::string> allowed_networks)
    : netplan_config_file_(resolve_netplan_config_file()),
      allowed_networks_(std::move(allowed_networks)) {}

bool NetplanManager::busy() {
    std::lock_guard<std::mutex> lock(mutex_);
    return busy_;
}

void NetplanManager::set_allowed_networks(std::vector<std::string> value) {
    std::lock_guard<std::mutex> lock(mutex_);
    allowed_networks_ = std::move(value);
}

std::string NetplanManager::get_current_ssid() {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        YAML::Node config = YAML::LoadFile(netplan_config_file_);
        auto aps = config["network"]["wifis"];
        if (aps && aps.IsMap()) {
            for (auto it = aps.begin(); it != aps.end(); ++it) {
                auto iface_cfg = it->second;
                auto access_points = iface_cfg["access-points"];
                if (access_points && access_points.IsMap()) {
                    for (auto ap = access_points.begin(); ap != access_points.end(); ++ap) {
                        return ap->first.as<std::string>();
                    }
                }
            }
        }
    } catch (...) {
    }
    return {};
}

std::string NetplanManager::get_configured_password() {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        YAML::Node config = YAML::LoadFile(netplan_config_file_);
        auto aps = config["network"]["wifis"];
        if (aps && aps.IsMap()) {
            for (auto it = aps.begin(); it != aps.end(); ++it) {
                auto iface_cfg = it->second;
                auto access_points = iface_cfg["access-points"];
                if (!access_points || !access_points.IsMap()) {
                    continue;
                }
                for (auto ap = access_points.begin(); ap != access_points.end(); ++ap) {
                    auto ap_cfg = ap->second;
                    if (ap_cfg && ap_cfg.IsMap() && ap_cfg["password"]) {
                        return ap_cfg["password"].as<std::string>("");
                    }
                    return {};
                }
            }
        }
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

std::pair<bool, std::string> NetplanManager::write_netplan(const std::string& ssid,
                                                           const std::string& password) {
    busy_ = true;
    YAML::Node config;
    try {
        if (std::filesystem::exists(netplan_config_file_)) {
            config = YAML::LoadFile(netplan_config_file_);
        }
    } catch (...) {
        config = YAML::Node(YAML::NodeType::Map);
    }
    auto network = config["network"];
    if (!network || !network.IsMap()) {
        network = config["network"] = YAML::Node(YAML::NodeType::Map);
    }
    network["version"] = 2;
    auto wifis = network["wifis"];
    if (!wifis || !wifis.IsMap() || wifis.size() == 0) {
        wifis = network["wifis"] = YAML::Node(YAML::NodeType::Map);
        wifis["wlan0"] = YAML::Node(YAML::NodeType::Map);
    }
    auto first = wifis.begin();
    auto iface_cfg = first->second;
    iface_cfg["dhcp4"] = true;
    iface_cfg["optional"] = true;
    YAML::Node aps(YAML::NodeType::Map);
    YAML::Node ap_cfg(YAML::NodeType::Map);
    if (!password.empty()) {
        ap_cfg["password"] = password;
    }
    aps[ssid] = ap_cfg;
    iface_cfg["access-points"] = aps;
    first->second = iface_cfg;

    try {
        std::ofstream out(netplan_config_file_);
        out << config;
        out.close();
    } catch (const std::exception& e) {
        busy_ = false;
        return {false, e.what()};
    }

    int rc = std::system("netplan apply");
    busy_ = false;
    if (rc == 0) {
        return {true, "netplan:" + ssid};
    }
    return {false, "netplan apply failed with code " + std::to_string(rc)};
}

std::pair<bool, std::string> NetplanManager::set_current_network(const std::string& ssid,
                                                                 const std::string& password) {
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
    return write_netplan(target, password);
}

std::pair<bool, std::string> NetplanManager::set_current_ssid(const std::string& target) {
    return set_current_network(target, {});
}

}  // namespace mrs_uav_bluetooth::network
