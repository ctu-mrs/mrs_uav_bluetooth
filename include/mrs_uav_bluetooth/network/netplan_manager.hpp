// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <mutex>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::network {

class NetplanManager {
public:
    explicit NetplanManager(std::vector<std::string> allowed_networks = {});

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
    std::pair<bool, std::string> write_netplan(const std::string& ssid,
                                               const std::string& password);

    std::string netplan_config_file_;
    std::vector<std::string> allowed_networks_;
    mutable std::mutex mutex_;
    bool busy_{false};
};

}  // namespace mrs_uav_bluetooth::network
