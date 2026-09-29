// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/network/netplan_manager.hpp
/// \brief Declares the netplan manager component of the Wi-Fi and netplan integration layer.

#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::network {

/// Validates an allowed SSID and updates only the configured netplan file.
class NetplanManager {
public:
    /// \brief Bind peer-requested Wi-Fi changes to one Netplan file and allow-list.
    /// \param netplan_config_file Netplan YAML file managed for Wi-Fi changes.
    /// \param allowed_networks wireless network names that a peer may request.
    explicit NetplanManager(std::string netplan_config_file = "/etc/netplan/01-netcfg.yaml",
                            std::vector<std::string> allowed_networks = {});

    /// \brief Check whether a network change currently owns the manager.
    /// \return True when a Netplan change is in progress; otherwise false.
    bool busy();
    /// \brief Read the wireless network currently reported active.
    /// \return Currently active wireless network name, or an empty string.
    std::string get_current_ssid();
    /// \brief Read the password stored for the current network.
    /// \return Password stored for the currently configured network.
    std::string get_configured_password();
    /// \brief List allow-listed wireless network names.
    /// \return Allowed wireless network names known to Netplan.
    std::vector<std::string> list_known_ssids();
    /// \brief Validate save and activate a permitted Wi-Fi network with rollback on failure.
    /// \param ssid Wi-Fi network name to persist and activate.
    /// \param password wireless network password to persist or apply.
    /// \return Success flag and operator-facing diagnostic message.
    std::pair<bool, std::string> set_current_network(const std::string& ssid,
                                                     std::optional<std::string> password = std::nullopt);
    /// \brief Switch to a previously stored Wi-Fi network without changing its password.
    /// \param target Allowed Wi-Fi network name to activate using its stored password.
    /// \return Success flag and operator-facing diagnostic message.
    std::pair<bool, std::string> set_current_ssid(const std::string& target);

    /// \brief Return the managed Netplan file path.
    /// \return Managed Netplan file path.
    const std::string& config_file() const {
        // Return the exact Netplan file managed by Wi-Fi service requests.
        return netplan_config_file_;
    }
    /// \brief Select the Netplan YAML file this manager may update.
    /// \param value Netplan YAML file managed by this instance.
    void set_config_file(std::string value);
    /// \brief Return the allowed Wi-Fi network names.
    /// \return Permitted Wi-Fi network names.
    const std::vector<std::string>& allowed_networks() const {
        // Return network names permitted for peer-requested Wi-Fi changes.
        return allowed_networks_;
    }
    /// \brief Replace the exact allow-list enforced before any Wi-Fi change.
    /// \param value Complete replacement list of permitted Wi-Fi network names.
    void set_allowed_networks(std::vector<std::string> value);

private:
    /// \brief Rewrite the managed access-point entry while preserving unrelated Netplan data.
    /// \param ssid Wi-Fi network name to persist and activate.
    /// \param password wireless network password to persist or apply.
    /// \return Success flag and operator-facing diagnostic message.
    std::pair<bool, std::string> write_netplan(const std::string& ssid,
                                               const std::optional<std::string>& password);

    std::string netplan_config_file_;
    std::vector<std::string> allowed_networks_;
    mutable std::mutex mutex_;
    bool busy_{false};
};

}  // namespace mrs_uav_bluetooth::network
