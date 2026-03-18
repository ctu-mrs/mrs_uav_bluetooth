// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/network/netplan_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace mrs_uav_bluetooth::network {

NetplanManager::NetplanManager(std::string netplan_config_file,
                               std::string autoscripts_dir,
                               std::vector<std::string> allowed_networks)
    : netplan_config_file_(std::move(netplan_config_file)),
      autoscripts_dir_(std::move(autoscripts_dir)),
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
    auto ssid = nmcli_.get_current_ssid();
    if (!ssid.empty()) {
        return ssid;
    }

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
        if (!ssid.empty() && std::find(result.begin(), result.end(), ssid) == result.end()) {
            result.push_back(ssid);
        }
    }
    if (std::filesystem::is_directory(autoscripts_dir_)) {
        for (const auto& entry : std::filesystem::directory_iterator(autoscripts_dir_)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            auto path = entry.path();
            if (path.extension() != ".sh") {
                continue;
            }
            auto name = path.stem().string();
            if (!name.empty() && std::find(result.begin(), result.end(), name) == result.end()) {
                result.push_back(name);
            }
        }
    }
    for (const auto& ssid : nmcli_.list_visible_ssids()) {
        if (std::find(result.begin(), result.end(), ssid) == result.end()) {
            result.push_back(ssid);
        }
    }
    return result;
}

std::pair<std::optional<std::string>, std::string> NetplanManager::resolve_script(
    const std::string& target) const {
    auto trimmed = target;
    if (trimmed.empty()) {
        return {std::nullopt, "empty target SSID"};
    }
    auto direct = std::filesystem::path(autoscripts_dir_) / (trimmed + ".sh");
    if (std::filesystem::exists(direct)) {
        return {direct.string(), {}};
    }
    return {std::nullopt, "no netplan script for '" + trimmed + "'"};
}

std::pair<bool, std::string> NetplanManager::apply_script(const std::string& script_path) {
    busy_ = true;
    std::string cmd = "bash \"" + script_path + "\"";
    int rc = std::system(cmd.c_str());
    busy_ = false;
    if (rc == 0) {
        return {true, script_path};
    }
    return {false, "script failed with code " + std::to_string(rc)};
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
    auto [script_path, _] = resolve_script(target);
    if (script_path.has_value() && password.empty()) {
        return apply_script(*script_path);
    }
    if (nmcli_.available()) {
        auto [ok, detail] = nmcli_.connect(target, password);
        if (ok) {
            return {true, "nmcli:" + target};
        }
    }
    return write_netplan(target, password);
}

std::pair<bool, std::string> NetplanManager::set_current_ssid(const std::string& target) {
    return set_current_network(target, {});
}

}  // namespace mrs_uav_bluetooth::network
