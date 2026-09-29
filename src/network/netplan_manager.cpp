// SPDX-License-Identifier: BSD-3-Clause
/// \file src/network/netplan_manager.cpp
/// \brief Implements the netplan manager component of the Wi-Fi and netplan integration layer.

#include "mrs_uav_bluetooth/network/netplan_manager.hpp"

#include "mrs_uav_bluetooth/util/string_utils.hpp"

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

/// \brief Test membership in the configured allow-list without modifying it.
/// \param values Allowed strings searched for the candidate.
/// \param candidate Exact string sought in the allowed-value list.
/// \return True when the candidate is in the configured list; otherwise false.
bool contains_value(const std::vector<std::string>& values, const std::string& candidate) {
    // Test membership in the configured allow-list without modifying it.
    return std::find(values.begin(), values.end(), candidate) != values.end();
}

/// \brief Extract the configured access-point network name from a Netplan document.
/// \param config Parsed Netplan document containing the access-point definition.
/// \return Decoded Netplan access-point name.
std::string read_access_point_ssid(const YAML::Node& config) {
    // Extract and unquote the access-point name from one Netplan entry.
    const auto access_points = config["network"]["wifis"]["wlan0"]["access-points"];
    if (!access_points || !access_points.IsMap()) {
        return {};
    }
    for (auto ap = access_points.begin(); ap != access_points.end(); ++ap) {
        return ap->first.as<std::string>("");
    }
    return {};
}

/// \brief Extract the configured access-point password from a Netplan document.
/// \param config Parsed Netplan document containing the access-point credentials.
/// \return Decoded Netplan access-point password.
std::string read_access_point_password(const YAML::Node& config) {
    // Extract and unquote the password from one Netplan access-point entry.
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

/// \brief Capture a command's standard output and require a successful exit status.
/// \param command shell command executed while capturing its output and status.
/// \return Captured command standard output.
std::string read_command_output(const char* command) {
    // Capture command output and reject a nonzero exit status.
    std::array<char, 256> buffer{};
    std::string output;

    struct PipeCloser {
        /// \brief Close the captured command stream when leaving scope.
        /// \param file open command-output stream closed by the local RAII guard.
        void operator()(FILE* file) const {
            // Apply operator() while preserving class invariants.
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
    return util::trim_ascii_copy(std::move(output));
}

/// \brief Query NetworkManager for the currently active Wi-Fi network name.
/// \return Active network name parsed from NetworkManager output.
std::string read_connected_ssid() {
    // Parse the active network name from the NetworkManager command output.
    return read_command_output(kIwGetIdCommand);
}

struct FileSnapshot {
    bool existed{false};
    std::string contents;
};

/// \brief Read and retain a Netplan file so a failed apply can restore it byte-for-byte.
/// \param path Netplan file to preserve before modification.
/// \return Original file presence and bytes for rollback.
FileSnapshot take_file_snapshot(const std::string& path) {
    // Read and retain a Netplan file so a failed apply can restore it byte-for-byte.
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

/// \brief Rewrite or remove the Netplan file to match the captured pre-change state.
/// \param path Netplan file to restore.
/// \param snapshot saved file contents and existence state used for rollback.
/// \return Whether the original Netplan file state was restored.
std::string restore_snapshot(const std::string& path, const FileSnapshot& snapshot) {
    // Rewrite or remove the Netplan file to match the captured pre-change state.
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

/// \brief Apply Netplan through the privileged helper and capture its diagnostic.
/// \return Empty string on success otherwise the helper's captured diagnostic.
std::string run_netplan_apply() {
    // Preserve the helper’s exit status so callers can trigger rollback.
    const int rc = std::system("netplan apply");
    if (rc == 0) {
        return {};
    }
    return "netplan apply failed with code " + std::to_string(rc);
}

/// \brief Poll until the requested Wi-Fi network becomes active or times out.
/// \param expected_ssid network name that must become active before the wait succeeds.
/// \return True if NetworkManager reports the expected network before timeout; otherwise false.
bool wait_for_connected_ssid(const std::string& expected_ssid) {
    // Poll until NetworkManager reports the requested network or the deadline expires.
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

/// \brief Serialize YAML while quoting mapping keys that Netplan could misinterpret.
/// \param out destination buffer, stream, or size receiving the result.
/// \param node YAML value to emit recursively.
/// \param double_quote_map_keys whether YAML map keys require explicit double quoting.
void emit_yaml_node(YAML::Emitter& out,
                    const YAML::Node& node,
                    bool double_quote_map_keys = false) {
    // Recurse through maps and sequences while quoting Netplan-sensitive keys.
    if (node.IsMap()) {
        out << YAML::BeginMap;
        for (auto item = node.begin(); item != node.end(); ++item) {
            const auto key = item->first.as<std::string>("");
            out << YAML::Key;
            if (double_quote_map_keys) {
                out << YAML::DoubleQuoted << key;
            } else {
                emit_yaml_node(out, item->first);
            }

            out << YAML::Value;
            emit_yaml_node(out, item->second, key == "access-points");
        }
        out << YAML::EndMap;
        return;
    }

    if (node.IsSequence()) {
        out << YAML::BeginSeq;
        for (const auto& item : node) {
            emit_yaml_node(out, item);
        }
        out << YAML::EndSeq;
        return;
    }

    out << node;
}

/// \brief Restore the previous file and reapply Netplan after a connection attempt fails.
/// \param path Netplan file whose saved contents must be restored.
/// \param snapshot saved file contents and existence state used for rollback.
/// \param message Original failure explanation to extend with rollback status.
/// \return Whether file rollback and Netplan reapply both succeeded.
std::pair<bool, std::string> rollback_netplan_change(const std::string& path,
                                                     const FileSnapshot& snapshot,
                                                     std::string message) {
    // Restore the previous file and reapply Netplan after a connection attempt fails.
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

/// \brief Derive the default Wi-Fi address from the numeric suffix of the local hostname.
/// \return Deterministic Wi-Fi address derived from the local UAV number.
std::string default_uav_static_address() {
    // Derive the default Wi-Fi address from the numeric suffix of the local hostname.
    char hostname_buf[256]{};
    std::string hostname;
    if (gethostname(hostname_buf, sizeof(hostname_buf) - 1) == 0) {
        hostname = hostname_buf;
    }

    std::string lowered = hostname;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
        // Normalize the hostname before deriving its deterministic static address.
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
      allowed_networks_(std::move(allowed_networks)) {
    // Construction records policy only; it never changes the host network.
}

bool NetplanManager::busy() {
    // Read the transition flag under the same lock used by network changes.
    std::lock_guard<std::mutex> lock(mutex_);
    return busy_;
}

void NetplanManager::set_config_file(std::string value) {
    // Normalize an override and restore the system default for an empty path.
    std::lock_guard<std::mutex> lock(mutex_);
    netplan_config_file_ = util::trim_ascii_copy(std::move(value));
    if (netplan_config_file_.empty()) {
        netplan_config_file_ = "/etc/netplan/01-netcfg.yaml";
    }
}

void NetplanManager::set_allowed_networks(std::vector<std::string> value) {
    // Replace the admission list atomically with respect to service requests.
    std::lock_guard<std::mutex> lock(mutex_);
    allowed_networks_ = std::move(value);
}

std::string NetplanManager::get_current_ssid() {
    // Prefer the configured access point, then query the live Wi-Fi link.
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
    // Missing or invalid YAML deliberately appears as no saved password.
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        return read_access_point_password(YAML::LoadFile(netplan_config_file_));
    } catch (...) {
    }
    return {};
}

std::vector<std::string> NetplanManager::list_known_ssids() {
    // Preserve policy order while removing empty and duplicate names.
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
    // Snapshot the file so parse, apply, or connection failures can roll back.
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
        const auto trimmed_password = util::trim_ascii_copy(*password);
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
        YAML::Emitter emitter;
        emit_yaml_node(emitter, config);
        if (!emitter.good()) {
            throw std::runtime_error(emitter.GetLastError());
        }

        std::ofstream out(netplan_config_file_, std::ios::binary | std::ios::trunc);
        out << emitter.c_str();
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
    // Serialize requests and reject targets outside the configured allow-list.
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
    if (password.has_value() && util::trim_ascii_copy(*password).empty()) {
        password.reset();
    }
    return write_netplan(target, password);
}

std::pair<bool, std::string> NetplanManager::set_current_ssid(const std::string& target) {
    // Reuse the saved password when callers only select a network name.
    return set_current_network(target, std::nullopt);
}

}  // namespace mrs_uav_bluetooth::network
