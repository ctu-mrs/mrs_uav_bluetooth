// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/peer/peer_manager.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>

namespace {

constexpr double kConnectAttemptGraceMin = 8.0;
constexpr double kConnectAttemptGraceMultiplier = 2.0;
constexpr double kDesiredConnectGraceMin = 3.0;
constexpr double kServicesWaitGraceMin = 8.0;
constexpr double kPairCooldownMin = 5.0;

bool is_phase(const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
              std::initializer_list<const char*> values) {
    for (const char* value : values) {
        if (session.phase == value) {
            return true;
        }
    }
    return false;
}

std::string trim_copy(std::string value) {
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool looks_like_mac_address(std::string_view value) {
    if (value.size() != 17) {
        return false;
    }
    for (size_t index = 0; index < value.size(); ++index) {
        const unsigned char ch = static_cast<unsigned char>(value[index]);
        if (index % 3 == 2) {
            if (ch != ':') {
                return false;
            }
            continue;
        }
        if (std::isxdigit(ch) == 0) {
            return false;
        }
    }
    return true;
}

std::string normalize_mac(std::string value) {
    value = trim_copy(std::move(value));
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

std::string normalize_whitelist_token(std::string value) {
    value = trim_copy(std::move(value));
    if (looks_like_mac_address(value)) {
        return normalize_mac(std::move(value));
    }
    return lower_copy(std::move(value));
}

bool whitelist_contains(const std::vector<std::string>& whitelist,
                        const std::string& mac,
                        const std::string& peer_name) {
    const auto normalized_mac = normalize_mac(mac);
    const auto normalized_peer_name = lower_copy(trim_copy(peer_name));
    for (const auto& entry : whitelist) {
        const auto normalized_entry = normalize_whitelist_token(entry);
        if (normalized_entry.empty()) {
            continue;
        }
        if (normalized_entry == normalized_mac) {
            return true;
        }
        if (!normalized_peer_name.empty() && normalized_entry == normalized_peer_name) {
            return true;
        }
    }
    return false;
}

}  // namespace

namespace mrs_uav_bluetooth::peer {

PeerManager::PeerManager(rclcpp::Node& node, rclcpp::Logger logger)
    : node_(node), logger_(logger) {}

double PeerManager::now_monotonic() const {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

PeerConnectionSession& PeerManager::get_or_create_session(const std::string& mac,
                                                          const std::string& peer_name) {
    auto it = sessions_.find(mac);
    if (it == sessions_.end()) {
        PeerConnectionSession session;
        session.mac = mac;
        const auto now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        session.first_seen_monotonic = now;
        session.last_seen_monotonic = now;
        session.peer_name = peer_name;
        it = sessions_.emplace(mac, std::move(session)).first;
    } else {
        const auto now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        it->second.last_seen_monotonic = now;
        if (!peer_name.empty()) {
            it->second.peer_name = peer_name;
        }
    }
    return it->second;
}

bool PeerManager::matches_auto_connect_policy(const bluez::DeviceInfo& device,
                                              const config::NodeConfig& config,
                                              const std::string& peer_name) const {
    (void)device;
    const auto normalized_peer_name = lower_copy(trim_copy(peer_name));
    return !normalized_peer_name.empty() &&
           util::is_uav_hostname(normalized_peer_name, config.auto_connect_pattern);
}

void PeerManager::set_session_phase(PeerConnectionSession& session,
                                    const std::string& phase,
                                    const std::string& detail) const {
    session.phase = phase;
    session.detail = detail;
}

void PeerManager::sync_device(const bluez::DeviceInfo& device,
                              const config::NodeConfig& config,
                              const std::string& peer_name) {
    auto& session = get_or_create_session(device.mac, peer_name);
    const auto now = now_monotonic();
    const bool was_desired = session.desired;

    session.missing_since_monotonic = 0.0;

    const bool whitelist_enabled = !config.auto_connect_whitelist.empty();
    session.explicit_target = whitelist_contains(config.auto_connect_whitelist,
                                                 device.mac,
                                                 session.peer_name);
    session.peer_candidate = matches_auto_connect_policy(device, config, session.peer_name);

    // When whitelist is enabled, only explicit targets are desired among peer candidates.
    // When whitelist is empty, any matching peer candidate is desired (if auto_connect is on).
    session.desired = session.explicit_target ||
                      (config.auto_connect_enable && session.peer_candidate && !whitelist_enabled);
    if (session.desired) {
        if (!was_desired || session.desired_since_monotonic <= 0.0) {
            session.desired_since_monotonic = now;
        }
    } else {
        session.desired_since_monotonic = 0.0;
    }
    session.services_wait_grace_s = std::max(kServicesWaitGraceMin, config.peer_connection_timeout);

    if (!session.desired) {
        session.connected_since_monotonic = 0.0;
        session.services_wait_started_monotonic = 0.0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        if (session.peer_candidate) {
            set_session_phase(session,
                              "policy_blocked",
                              whitelist_enabled && !session.explicit_target
                                  ? "peer not present in whitelist"
                                  : "peer not allowed by current config");
        } else if (!session.peer_candidate) {
            set_session_phase(session, "idle",
                              session.peer_name.empty() ? "device does not match peer policy"
                                                        : "not a peer candidate");
        } else {
            set_session_phase(session, "idle", "auto-connect disabled");
        }
        return;
    }

    if (device.blocked) {
        set_session_phase(session, "blocked", "device is blocked by BlueZ");
        return;
    }

    if (!device.connected) {
        session.connected_since_monotonic = 0.0;
        session.services_wait_started_monotonic = 0.0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();

        if ((device.paired || device.bonded) && !device.trusted) {
            set_session_phase(session, "securing", "repairing trust before reconnect");
            return;
        }

        if (is_phase(session, {"connecting", "connect_pending"})) {
            set_session_phase(session, "connect_pending", "awaiting connection");
            return;
        }

        set_session_phase(session,
                          session.last_connect_attempt_monotonic > 0.0 ? "disconnected" : "discovered",
                          "awaiting connection");
        return;
    }

    session.connected_since_monotonic = session.connected_since_monotonic > 0.0
        ? session.connected_since_monotonic : now;

    if ((device.paired || device.bonded) && !device.trusted) {
        set_session_phase(session, "securing", "connected, repairing trust");
        return;
    }

    if (!device.paired && !device.bonded) {
        set_session_phase(session, "securing", "connected, waiting for pairing");
        return;
    }

    if (!device.services_resolved) {
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        if (session.services_wait_started_monotonic <= 0.0) {
            session.services_wait_started_monotonic = now;
        }

        if (now - session.services_wait_started_monotonic > session.services_wait_grace_s) {
            set_session_phase(session, "recovering", "services unresolved beyond grace");
            return;
        }

        set_session_phase(session, "connected_unready", "connected, waiting for services");
        return;
    }

    set_session_phase(session, "connected_unready", "connected, services resolved, awaiting peer time bridge");
    session.services_wait_started_monotonic = 0.0;
}

void PeerManager::note_missing_device(const std::string& mac, double now_mono) {
    auto it = sessions_.find(mac);
    if (it == sessions_.end()) {
        return;
    }
    if (it->second.missing_since_monotonic <= 0.0) {
        it->second.missing_since_monotonic = now_mono;
    }
    it->second.connected_since_monotonic = 0.0;
    it->second.services_wait_started_monotonic = 0.0;
    it->second.bridge_wait_started_monotonic = 0.0;
    it->second.bridge_wait_reason.clear();
    set_session_phase(it->second, "stale", "device missing from cache");
}

void PeerManager::note_pairing_event(const std::string& device_path,
                                     const std::string& event,
                                     const bluez::ObjectManagerCache& cache) {
    auto device = cache.device(device_path);
    if (!device) {
        return;
    }

    auto& session = get_or_create_session(device->mac);
    const auto now = now_monotonic();
    session.last_security_attempt_monotonic = now;

    if (event == "request_confirmation" || event == "request_authorization" ||
        event == "request_passkey" || event == "request_pin") {
        session.last_pairing_request_monotonic = now;
        if (device->paired || device->bonded) {
            session.stale_pairing_detected = true;
            set_session_phase(session, "recovering", "stale pairing detected");
            return;
        }
        set_session_phase(session, "securing", event);
        return;
    }

    if (event == "cancel") {
        session.pairing_failures += 1;
        set_session_phase(session, "recovering", "pairing cancelled");
        return;
    }

    if (event == "agent_release") {
        session.detail = "pairing agent released";
    }
}

bool PeerManager::should_attempt_trust(const PeerConnectionSession& session,
                                       const bluez::DeviceInfo& device,
                                       double now_mono,
                                       double retry_period_s) const {
    if (!session.desired) {
        return false;
    }
    if (!(device.paired || device.bonded) || device.trusted) {
        return false;
    }
    return session.last_security_attempt_monotonic <= 0.0 ||
           now_mono - session.last_security_attempt_monotonic >= std::max(kPairCooldownMin, retry_period_s);
}

bool PeerManager::should_attempt_connect(const PeerConnectionSession& session,
                                         double now_mono,
                                         double retry_period_s) const {
    if (!session.desired) {
        return false;
    }
    if (is_phase(session, {"ready", "connected_unready", "securing", "blocked", "policy_blocked"})) {
        return false;
    }

    if (session.desired_since_monotonic > 0.0 &&
        now_mono - session.desired_since_monotonic < kDesiredConnectGraceMin) {
        return false;
    }

    if (session.connect_started_monotonic > 0.0 &&
        is_phase(session, {"connecting", "connect_pending"}) &&
        now_mono - session.connect_started_monotonic <
            std::max(kConnectAttemptGraceMin, retry_period_s * kConnectAttemptGraceMultiplier)) {
        return false;
    }

    return session.last_connect_attempt_monotonic <= 0.0 ||
           now_mono - session.last_connect_attempt_monotonic >= retry_period_s;
}

bool PeerManager::should_attempt_pair(const PeerConnectionSession& session,
                                      const bluez::DeviceInfo& device,
                                      double now_mono,
                                      double retry_period_s) const {
    if (!session.desired) {
        return false;
    }
    if (!is_phase(session, {"securing", "connected_unready", "recovering"})) {
        return false;
    }
    if (session.connected_since_monotonic <= 0.0) {
        return false;
    }
    if (device.blocked) {
        return false;
    }
    if (device.paired || device.bonded) {
        return false;
    }
    const double cooldown = std::max(kPairCooldownMin,
                                     retry_period_s * std::max(1, session.pairing_failures + 1));
    return session.last_security_attempt_monotonic <= 0.0 ||
           now_mono - session.last_security_attempt_monotonic >= cooldown;
}

void PeerManager::prune_sessions(const std::set<std::string>& current_macs,
                                 double now_mono,
                                 double ttl_s) {
    for (auto it = sessions_.begin(); it != sessions_.end();) {
        if (current_macs.find(it->first) != current_macs.end() ||
            time_bridges_.find(it->first) != time_bridges_.end() ||
            now_mono - it->second.last_seen_monotonic <= ttl_s) {
            ++it;
            continue;
        }
        it = sessions_.erase(it);
    }
}

void PeerManager::remove_time_bridge(const std::string& mac) {
    auto it = time_bridges_.find(mac);
    if (it == time_bridges_.end()) {
        return;
    }
    it->second.publisher.reset();
    time_bridges_.erase(it);
}

}  // namespace mrs_uav_bluetooth::peer
