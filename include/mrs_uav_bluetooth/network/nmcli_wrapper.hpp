// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string>
#include <vector>

namespace mrs_uav_bluetooth::network {

class NmcliWrapper {
public:
    explicit NmcliWrapper(std::string executable = "nmcli");

    bool available() const;
    std::string get_current_ssid() const;
    std::vector<std::string> list_visible_ssids() const;
    std::pair<bool, std::string> connect(const std::string& ssid,
                                         const std::string& password = "") const;

private:
    std::pair<bool, std::string> run(const std::vector<std::string>& args,
                                     int timeout_s) const;

    std::string executable_;
};

}  // namespace mrs_uav_bluetooth::network
