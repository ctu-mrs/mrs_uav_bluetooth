// SPDX-License-Identifier: MIT
#pragma once

#include "mrs_uav_bluetooth/network/nmcli_wrapper.hpp"

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::network {

class NetplanManager {
public:
    NetplanManager(std::string netplan_config_file,
                   std::string autoscripts_dir,
                   std::vector<std::string> allowed_networks = {});

    bool busy();
    std::string get_current_ssid();
    std::string get_configured_password();
    std::vector<std::string> list_known_ssids();
    std::pair<bool, std::string> set_current_network(const std::string& ssid,
                                                     const std::string& password = "");
    std::pair<bool, std::string> set_current_ssid(const std::string& target);

    const std::vector<std::string>& allowed_networks() const { return allowed_networks_; }
    void set_allowed_networks(std::vector<std::string> value);

private:
    std::pair<std::optional<std::string>, std::string> resolve_script(const std::string& target) const;
    std::pair<bool, std::string> apply_script(const std::string& script_path);
    std::pair<bool, std::string> write_netplan(const std::string& ssid,
                                               const std::string& password);

    std::string netplan_config_file_;
    std::string autoscripts_dir_;
    std::vector<std::string> allowed_networks_;
    mutable std::mutex mutex_;
    NmcliWrapper nmcli_;
    bool busy_{false};
};

}  // namespace mrs_uav_bluetooth::network
