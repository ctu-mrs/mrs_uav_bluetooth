// SPDX-License-Identifier: BSD-3-Clause
/// \file src/peer/peer_manager.cpp
/// \brief Implements the peer manager component of the peer lifecycle layer.

#include "mrs_uav_bluetooth/peer/peer_manager.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <functional>

namespace {

constexpr double kConnectAttemptGraceMin = 15.0;
constexpr double kConnectAttemptGraceMultiplier = 5.0;
constexpr double kServicesWaitGraceMin = 15.0;
constexpr double kPairCooldownMin = 10.0;
constexpr double kPairAfterConnectGraceMin = 5.0;
constexpr double kTrustCooldownMin = 0.5;

/// \brief Test a peer session phase against a small accepted-state set.
/// \param session Peer session whose current phase is compared.
/// \param values Accepted peer phase names.
/// \return True if the session phase matches one of the accepted names; otherwise false.
bool is_phase(const mrs_uav_bluetooth::peer::PeerConnectionSession& session,
              std::initializer_list<const char*> values) {
    // Stop at the first exact phase match.
    for (const char* value : values) {
        if (session.phase == value) {
            return true;
        }
    }
    return false;
}

/// \brief Return text without surrounding ASCII whitespace.
/// \param value Text to normalize.
/// \return Input text without surrounding ASCII whitespace.
std::string trim_copy(std::string value) {
    // Locate both boundaries before returning the middle substring.
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

/// \brief Lowercase peer names before matching hostname-based admission rules.
/// \param value Text to normalize.
/// \return Lowercase copy of the input text.
std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        // Lowercase peer names before matching hostname-based admission rules.
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

/// \brief Validate the six-octet colon-separated Bluetooth address form.
/// \param value Peer identifier to test for Bluetooth-address syntax.
/// \return True if the full text contains six hexadecimal address octets; otherwise false.
bool looks_like_mac_address(std::string_view value) {
    // Require exactly six hexadecimal octets separated by five colons.
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

/// \brief Validate and uppercase a Bluetooth address for stable map keys.
/// \param value Bluetooth address to trim validate and uppercase.
/// \return Normalized mac.
std::string normalize_mac(std::string value) {
    value = trim_copy(std::move(value));
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        // Uppercase address hex digits while preserving colon separators.
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

/// \brief Canonicalize a configured hostname or Bluetooth address for policy matching.
/// \param value Hostname or Bluetooth address from the peer allow-list.
/// \return Normalized whitelist token.
std::string normalize_whitelist_token(std::string value) {
    // Preserve MAC separators while hostnames use lowercase policy matching.
    value = trim_copy(std::move(value));
    if (looks_like_mac_address(value)) {
        return normalize_mac(std::move(value));
    }
    return lower_copy(std::move(value));
}

/// \brief Match a normalized peer name or address against the configured admission list.
/// \param whitelist configured peer names or addresses allowed to connect.
/// \param mac peer Bluetooth MAC address.
/// \param peer_name Resolved peer hostname checked alongside its Bluetooth address.
/// \return True when whitelist contains; otherwise false.
bool whitelist_contains(const std::vector<std::string>& whitelist,
                        const std::string& mac,
                        const std::string& peer_name) {
    // Match a normalized peer name or address against the configured admission list.
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

/// \brief Detect cached pairing, bonding, or trust state that may need cleanup.
/// \param device Peer whose paired bonded or trusted flags are inspected.
/// \return True when device has local security; otherwise false.
bool device_has_local_security(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    // Detect cached pairing, bonding, or trust state that may need cleanup.
    return device.paired || device.bonded || device.trusted;
}

/// \brief Decide whether this peer must complete pairing before bridge setup.
/// \param config Security policy from which the required protection level is derived.
/// \return True if policy requires pairing before peer use; otherwise false.
bool pairing_required(const mrs_uav_bluetooth::config::NodeConfig& config) {
    // Decide whether this peer must complete pairing before bridge setup.
    return config.auto_pair;
}

/// \brief Derive whether the active security policy requires BlueZ trust.
/// \param config Security policy from which the required protection level is derived.
/// \return True if policy requires the peer's Trusted flag; otherwise false.
bool trust_required(const mrs_uav_bluetooth::config::NodeConfig& config) {
    // Trust required.
    return config.auto_trust;
}

/// \brief Require pairing only when the active configuration or cached peer state needs it.
/// \param device Peer whose pairing state is checked against policy.
/// \param config Security policy against which the peer flags are checked.
/// \return True when device has required pairing; otherwise false.
bool device_has_required_pairing(const mrs_uav_bluetooth::bluez::DeviceInfo& device,
                                 const mrs_uav_bluetooth::config::NodeConfig& config) {
    // Require pairing only when the active configuration or cached peer state needs it.
    return !pairing_required(config) || device.paired || device.bonded;
}

/// \brief Require Trusted only when the active policy enables automatic trust.
/// \param device Peer whose trusted state is checked against policy.
/// \param config Security policy against which the peer flags are checked.
/// \return True when device has required trust; otherwise false.
bool device_has_required_trust(const mrs_uav_bluetooth::bluez::DeviceInfo& device,
                               const mrs_uav_bluetooth::config::NodeConfig& config) {
    // Require Trusted only when the active policy enables automatic trust.
    return !trust_required(config) || device.trusted;
}

/// \brief Detect stale local security state that must be removed before retrying pairing.
/// \param device Peer checked for stale local security material requiring removal.
/// \return True when device needs forget; otherwise false.
bool device_needs_forget(const mrs_uav_bluetooth::bluez::DeviceInfo& device) {
    // Detect stale local security state that must be removed before retrying pairing.
    return device.connected || device.services_resolved || device_has_local_security(device);
}

/// \brief Block normal peer work until stale-bond removal is issued and observed.
/// \param session Peer session checked for an active repair or reset workflow.
/// \return True when repair blocks regular actions; otherwise false.
bool repair_blocks_regular_actions(const mrs_uav_bluetooth::peer::PeerConnectionSession& session) {
    // Block normal peer work until stale-bond removal is issued and observed.
    return session.repair_requested &&
           (!session.repair_remove_issued || session.repair_awaiting_cache_removal);
}

}  // namespace

namespace mrs_uav_bluetooth::peer {

PeerManager::PeerManager(rclcpp::Node& node, rclcpp::Logger logger)
    : node_(node), logger_(logger) {
        // Retain the ROS clock and logger used by peer retry and phase tracking.
    }

double PeerManager::now_monotonic() const {
    // Read steady-clock seconds for retry and lease calculations.
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

PeerConnectionSession& PeerManager::get_or_create_session(const std::string& mac,
                                                          const std::string& peer_name) {
    // Create first-seen state or refresh the last-seen time and discovered peer name.
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
    // Accept a peer only when its normalized name matches the configured UAV pattern.
    (void)device;
    const auto normalized_peer_name = lower_copy(trim_copy(peer_name));
    return !normalized_peer_name.empty() &&
           util::is_uav_hostname(normalized_peer_name, config.auto_connect_pattern);
}

void PeerManager::set_session_phase(PeerConnectionSession& session,
                                    const std::string& phase,
                                    const std::string& detail) const {
    // Update the externally reported phase and its diagnostic detail together.
    session.phase = phase;
    session.detail = detail;
}

void PeerManager::note_local_connect_attempt(PeerConnectionSession& session,
                                             double now_mono) const {
    // Mark the session as locally initiated and start its connection grace window.
    session.peer_initiated = false;
    session.last_connect_attempt_monotonic = now_mono;
    session.connect_started_monotonic = now_mono;
}

void PeerManager::request_device_reset(PeerConnectionSession& session,
                                       const std::string& reason,
                                       bool reset_pairing_state) {
    // Request device reset.
    if (session.repair_requested) {
        if (session.repair_reason.empty() && !reason.empty()) {
            session.repair_reason = reason;
        }
        if (reset_pairing_state) {
            session.pairing_reset_pending = true;
        }
        if (session.detail.empty()) {
            session.detail = session.repair_reason.empty() ? "resetting peer device state"
                                                           : session.repair_reason;
        }
        return;
    }

    session.repair_requested = true;
    session.pairing_reset_pending = reset_pairing_state;
    session.repair_in_progress = false;
    session.repair_remove_issued = false;
    session.repair_awaiting_cache_removal = false;
    session.repair_requested_monotonic = now_monotonic();
    session.repair_reason = reason;
    session.services_wait_started_monotonic = 0.0;
    session.bridge_wait_started_monotonic = 0.0;
    session.bridge_wait_reason.clear();
    session.last_notify_failure_monotonic = 0.0;
    session.notify_failure_count = 0;
    session.last_notify_failure_characteristic_path.clear();
    session.time_bridge_init_notify_failure_monotonic = 0.0;
    session.time_bridge_init_notify_failure_count = 0;
    session.service_regression_started_monotonic = 0.0;
    session.local_time_notify_active_this_connection = false;
    session.remote_gatt_missing_since_monotonic = 0.0;
    session.remote_gatt_missing_checks = 0;
    set_session_phase(session,
                      "recovering",
                      reason.empty() ? "resetting peer device state" : reason);
}

void PeerManager::clear_device_reset(PeerConnectionSession& session,
                                     bool clear_pairing_reset_pending) {
    // Release repair flags after removal completes or the repair is abandoned.
    session.repair_requested = false;
    session.repair_in_progress = false;
    session.repair_remove_issued = false;
    session.repair_awaiting_cache_removal = false;
    session.repair_requested_monotonic = 0.0;
    if (clear_pairing_reset_pending) {
        session.pairing_reset_pending = false;
    }
    session.forget_pending = false;
    session.repair_reason.clear();
    session.remote_gatt_missing_since_monotonic = 0.0;
    session.remote_gatt_missing_checks = 0;
    session.last_service_retry_monotonic = 0.0;
    session.services_resolved_since_monotonic = 0.0;
    session.last_notify_failure_monotonic = 0.0;
    session.notify_failure_count = 0;
    session.last_notify_failure_characteristic_path.clear();
    session.time_bridge_init_notify_failure_monotonic = 0.0;
    session.time_bridge_init_notify_failure_count = 0;
    session.service_regression_started_monotonic = 0.0;
}

void PeerManager::sync_device(const bluez::DeviceInfo& device,
                              const config::NodeConfig& config,
                              const std::string& peer_name,
                              bool preserve_ready_runtime,
                              bool preserve_active_bridge_runtime) {
    const bool has_connection_state =
        device.connected || device.services_resolved || device_has_local_security(device);
    if (!sessions_.contains(device.mac) && !has_connection_state &&
        (!config.auto_connect_enable || peer_name.empty())) {
        // Advertisement reception uses ObjectManagerCache directly and does
        // not need a GATT connection session. In broadcast mode, rotating
        // private addresses can otherwise create hundreds of permanently
        // irrelevant sessions. Still track an existing session so a mode
        // change can clear it, and always track connected or bonded devices.
        return;
    }
    auto& session = get_or_create_session(device.mac, peer_name);
    const auto now = now_monotonic();
    const bool was_desired = session.desired;

    session.missing_since_monotonic = 0.0;

    const bool whitelist_enabled = !config.peer_whitelist.empty();
    session.explicit_target = whitelist_contains(config.peer_whitelist,
                                                 device.mac,
                                                 session.peer_name);
    session.peer_candidate = matches_auto_connect_policy(device, config, session.peer_name);

    // The global flag gates all outbound automation. When enabled, a non-empty
    // whitelist selects explicit targets; otherwise the hostname policy does.
    session.desired = config.auto_connect_enable &&
                      (session.explicit_target ||
                       (session.peer_candidate && !whitelist_enabled));
    if (session.desired) {
        if (!was_desired || session.desired_since_monotonic <= 0.0) {
            session.desired_since_monotonic = now;
        }
        session.forget_pending = false;
    } else {
        session.desired_since_monotonic = 0.0;
        session.forget_pending = false;
    }
    session.services_wait_grace_s = std::max(kServicesWaitGraceMin, config.peer_connection_timeout);

    if (!device.paired && !device.bonded) {
        session.time_bridge_init_notify_failure_monotonic = 0.0;
        session.time_bridge_init_notify_failure_count = 0;
    }

    if (!device.connected && !device_has_local_security(device) && !device.services_resolved &&
        !session.repair_requested) {
        session.forget_pending = false;
        if (session.repair_in_progress) {
            session.repair_in_progress = false;
        }
    }

    if (!session.desired) {
        session.pairing_in_progress = false;
        session.time_bridge_healthy_this_connection = false;
        session.local_time_notify_active_this_connection = false;
        session.connected_since_monotonic = 0.0;
        session.services_wait_started_monotonic = 0.0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.last_notify_failure_monotonic = 0.0;
        session.notify_failure_count = 0;
        session.last_notify_failure_characteristic_path.clear();
        session.time_bridge_init_notify_failure_monotonic = 0.0;
        session.time_bridge_init_notify_failure_count = 0;
        session.last_service_retry_monotonic = 0.0;
        session.services_resolved_since_monotonic = 0.0;
        session.service_regression_started_monotonic = 0.0;
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        if (!device_needs_forget(device)) {
            session.forget_pending = false;
            clear_device_reset(session);
        }
        if (device.connected) {
            set_session_phase(session,
                              "passive",
                              "peer-initiated or manually managed connection");
        } else if (session.peer_candidate) {
            set_session_phase(session,
                              "idle",
                              config.auto_connect_enable
                                  ? "not selected for automatic connection"
                                  : "automatic connection disabled");
        } else if (!session.peer_candidate) {
            set_session_phase(session, "idle",
                              session.peer_name.empty() ? "device does not match peer policy"
                                                        : "not a peer candidate");
        }
        return;
    }

    if (device.blocked) {
        set_session_phase(session, "blocked", "device is blocked by BlueZ");
        return;
    }

    if (!device.connected) {
        // ``peer_initiated`` describes who owned the connection that just ended;
        // it must not permanently make an automatically selected UAV passive.
        // This is especially important after BlueZ removes and rediscovers a
        // device record: the PeerConnectionSession intentionally survives for a
        // short time, but the next connection has not been initiated by either
        // side yet. Non-selected/manual devices return above and therefore keep
        // their passive behaviour.
        session.peer_initiated = false;
        session.pairing_in_progress = false;
        session.time_bridge_healthy_this_connection = false;
        session.local_time_notify_active_this_connection = false;
        session.connected_since_monotonic = 0.0;
        session.services_wait_started_monotonic = 0.0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.last_notify_failure_monotonic = 0.0;
        session.notify_failure_count = 0;
        session.last_notify_failure_characteristic_path.clear();
        session.last_service_retry_monotonic = 0.0;
        session.services_resolved_since_monotonic = 0.0;
        session.service_regression_started_monotonic = 0.0;
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;

        if (is_phase(session, {"connecting", "connect_pending"})) {
            set_session_phase(session, "connect_pending", "awaiting connection");
            return;
        }

        set_session_phase(session,
                          session.last_connect_attempt_monotonic > 0.0 ? "disconnected" : "discovered",
                          "awaiting connection");
        return;
    }

    const bool first_connection_observed = session.connected_since_monotonic <= 0.0;
    const double connect_grace_s = std::max(kConnectAttemptGraceMin,
                                            config.auto_connect_period * kConnectAttemptGraceMultiplier);
    const bool local_connect_in_flight = session.connect_started_monotonic > 0.0 &&
        (now - session.connect_started_monotonic) < connect_grace_s;

    if (first_connection_observed && !local_connect_in_flight) {
        session.peer_initiated = true;
        clear_device_reset(session);
    }

    session.connected_since_monotonic = session.connected_since_monotonic > 0.0
        ? session.connected_since_monotonic : now;

    if (device_has_required_pairing(device, config)) {
        session.pairing_in_progress = false;
        session.pairing_wait_started_monotonic = 0.0;
    } else {
        session.last_service_retry_monotonic = 0.0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        if (session.pairing_in_progress) {
            if (session.detail.empty() || session.detail == "connected, waiting for pairing") {
                session.detail = "awaiting pairing completion";
            }
            session.phase = "securing";
        } else {
            set_session_phase(session, "connected_unready", "connected, waiting for pairing");
        }
        return;
    }

    if (!device.services_resolved) {
        session.last_service_retry_monotonic = 0.0;
        if (preserve_ready_runtime) {
            if (session.service_regression_started_monotonic <= 0.0) {
                session.service_regression_started_monotonic = now;
            }
            session.services_wait_started_monotonic = 0.0;
            session.bridge_wait_started_monotonic = 0.0;
            session.bridge_wait_reason.clear();
            session.last_notify_failure_monotonic = 0.0;
            session.notify_failure_count = 0;
            session.last_notify_failure_characteristic_path.clear();
            session.remote_gatt_missing_since_monotonic = 0.0;
            session.remote_gatt_missing_checks = 0;
            set_session_phase(session,
                              "ready",
                              "peer time bridge active during expected services rediscovery");
            return;
        }

        if (preserve_active_bridge_runtime) {
            if (session.service_regression_started_monotonic <= 0.0) {
                session.service_regression_started_monotonic = now;
            }
            session.services_wait_started_monotonic = 0.0;
            session.last_service_retry_monotonic = 0.0;
            session.remote_gatt_missing_since_monotonic = 0.0;
            session.remote_gatt_missing_checks = 0;
            session.phase = "connected_unready";
            if (session.detail.empty()) {
                session.detail = "peer time bridge active during expected services rediscovery";
            }
            return;
        }

        session.services_resolved_since_monotonic = 0.0;
        session.service_regression_started_monotonic = 0.0;
        session.bridge_wait_started_monotonic = 0.0;
        session.bridge_wait_reason.clear();
        session.last_notify_failure_monotonic = 0.0;
        session.notify_failure_count = 0;
        session.last_notify_failure_characteristic_path.clear();
        session.remote_gatt_missing_since_monotonic = 0.0;
        session.remote_gatt_missing_checks = 0;
        if (session.services_wait_started_monotonic <= 0.0) {
            session.services_wait_started_monotonic = now;
        }

        set_session_phase(session, "connected_unready", "connected, waiting for services");
        return;
    }

    if (session.services_resolved_since_monotonic <= 0.0) {
        session.services_resolved_since_monotonic = now;
    }
    session.service_regression_started_monotonic = 0.0;
    if (preserve_ready_runtime) {
        set_session_phase(session, "ready", "peer time bridge active");
        session.services_wait_started_monotonic = 0.0;
        return;
    }
    set_session_phase(session, "connected_unready", "connected, services resolved, awaiting peer time bridge");
    session.services_wait_started_monotonic = 0.0;
}

void PeerManager::note_missing_device(const std::string& mac, double now_mono) {
    // Start the absence timer once without extending it on later cache snapshots.
    auto it = sessions_.find(mac);
    if (it == sessions_.end()) {
        return;
    }
    if (it->second.missing_since_monotonic <= 0.0) {
        it->second.missing_since_monotonic = now_mono;
    }
    it->second.forget_pending = false;
    it->second.time_bridge_healthy_this_connection = false;
    it->second.local_time_notify_active_this_connection = false;
    it->second.connected_since_monotonic = 0.0;
    it->second.pairing_in_progress = false;
    it->second.services_wait_started_monotonic = 0.0;
    it->second.bridge_wait_started_monotonic = 0.0;
    it->second.bridge_wait_reason.clear();
    it->second.last_notify_failure_monotonic = 0.0;
    it->second.notify_failure_count = 0;
    it->second.last_notify_failure_characteristic_path.clear();
    it->second.last_service_retry_monotonic = 0.0;
    it->second.services_resolved_since_monotonic = 0.0;
    it->second.service_regression_started_monotonic = 0.0;
    it->second.remote_gatt_missing_since_monotonic = 0.0;
    it->second.remote_gatt_missing_checks = 0;
    if (it->second.repair_requested && it->second.repair_awaiting_cache_removal) {
        const auto repair_detail = it->second.repair_reason.empty()
            ? std::string{"awaiting fresh discovery after security reset"}
            : it->second.repair_reason;
        clear_device_reset(it->second);
        set_session_phase(it->second,
                          "stale",
                          repair_detail);
        return;
    }
    set_session_phase(it->second, "stale", "device missing from cache");
}

void PeerManager::note_pairing_event(const std::string& device_path,
                                     const std::string& event,
                                     const bluez::ObjectManagerCache& cache) {
    // Resolve the agent path to a session and advance its pairing flags and phase.
    auto device = cache.device(device_path);
    if (!device) {
        return;
    }

    auto& session = get_or_create_session(device->mac);
    const auto now = now_monotonic();
    session.last_security_attempt_monotonic = now;

    if (event == "request_authorization") {
        if (is_phase(session, {"ready"}) && session.detail.empty()) {
            session.detail = event;
        }
        return;
    }

    if (event == "request_confirmation" ||
        event == "request_passkey" || event == "request_pin") {
        session.pairing_in_progress = true;
        session.last_pairing_request_monotonic = now;
        if (device_has_local_security(*device)) {
            if (!is_phase(session, {"ready", "connected_unready"})) {
                set_session_phase(session, "securing", event);
            } else if (session.detail.empty()) {
                session.detail = event;
            }
            return;
        }
        if (is_phase(session, {"ready"})) {
            session.detail = event;
            return;
        }
        set_session_phase(session, "securing", event);
        return;
    }

    if (event == "cancel") {
        session.pairing_in_progress = false;
        session.pairing_failures += 1;
        set_session_phase(session, "recovering", "pairing cancelled");
        return;
    }

    if (event == "agent_release") {
        session.pairing_in_progress = false;
        session.detail = "pairing agent released";
    }
}

bool PeerManager::should_attempt_trust(const PeerConnectionSession& session,
                                       const bluez::DeviceInfo& device,
                                       const config::NodeConfig& config,
                                       double now_mono,
                                       double retry_period_s) const {
    // Trust only stable, connected, security-ready peers after the retry cooldown.
    if (!config.auto_trust) {
        return false;
    }
    if (!session.desired) {
        return false;
    }
    if (repair_blocks_regular_actions(session) || session.repair_in_progress) {
        return false;
    }
    if (!device.connected || device.blocked || device.trusted) {
        return false;
    }
    if (session.connected_since_monotonic <= 0.0) {
        return false;
    }
    if (session.pairing_in_progress) {
        return false;
    }
    if (pairing_required(config) && !device_has_required_pairing(device, config)) {
        return false;
    }
    if (is_phase(session, {"connecting", "connect_pending", "discovered", "disconnected",
                           "blocked", "policy_blocked", "recovering", "stale"})) {
        return false;
    }
    if ((now_mono - session.connected_since_monotonic) < kTrustCooldownMin) {
        return false;
    }
    return session.last_security_attempt_monotonic <= 0.0 ||
           now_mono - session.last_security_attempt_monotonic >= std::max(kTrustCooldownMin, retry_period_s * 0.25);
}

bool PeerManager::should_attempt_connect(const PeerConnectionSession& session,
                                         double now_mono,
                                         double retry_period_s) const {
    // Retry only desired visible peers that are outside active or blocked phases.
    if (!session.desired) {
        return false;
    }
    if (repair_blocks_regular_actions(session) || session.repair_in_progress) {
        return false;
    }
    if (session.missing_since_monotonic > 0.0) {
        return false;
    }
    if (is_phase(session, {"ready", "connected_unready", "securing", "blocked", "policy_blocked"})) {
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
                                      const config::NodeConfig& config,
                                      double now_mono,
                                      double retry_period_s) const {
    // Pair connected desired peers only after discovery grace and failure backoff.
    if (!config.auto_pair) {
        return false;
    }
    if (!session.desired) {
        return false;
    }
    if (repair_blocks_regular_actions(session) || session.repair_in_progress) {
        return false;
    }
    if (session.pairing_in_progress) {
        return false;
    }
    if (device.blocked) {
        return false;
    }
    if (!device.connected) {
        return false;
    }
    if (session.connected_since_monotonic <= 0.0) {
        return false;
    }
    if (device_has_required_pairing(device, config)) {
        return false;
    }

    const double pair_grace_s = std::max(kPairAfterConnectGraceMin, retry_period_s);
    const double services_wait_started = session.services_wait_started_monotonic > 0.0
        ? session.services_wait_started_monotonic
        : session.connected_since_monotonic;
    if (!session.pairing_fallback_due(now_mono) &&
        (services_wait_started <= 0.0 || (now_mono - services_wait_started) < pair_grace_s)) {
        return false;
    }

    const double cooldown = std::max(kPairCooldownMin,
                                     retry_period_s * std::max(1, session.pairing_failures + 1));
    return session.last_security_attempt_monotonic <= 0.0 ||
           now_mono - session.last_security_attempt_monotonic >= cooldown;
}

bool PeerManager::should_repair_authentication_disconnect(
    const PeerConnectionSession& session, const bluez::DeviceInfo& device,
    const config::NodeConfig& config, const std::string& reason) const {
    // Repair stored credentials only for an admitted automatically managed GATT peer.
    return reason == "org.bluez.Reason.Authentication" &&
        config.auto_pair && config.auto_connect_enable && !config.enable_mesh &&
        config.advertise_mode != "broadcast" && session.desired &&
        !device.blocked && (device.paired || device.bonded) &&
        !session.repair_requested && !session.repair_in_progress;
}

void PeerManager::prune_sessions(const std::set<std::string>& current_macs,
                                 double now_mono,
                                 double ttl_s) {
    // Remove only unseen, inactive sessions whose absence exceeds the configured TTL.
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
    // Release the peer’s status publisher before erasing its clock state.
    auto it = time_bridges_.find(mac);
    if (it == time_bridges_.end()) {
        return;
    }
    it->second.publisher.reset();
    time_bridges_.erase(it);
}

}  // namespace mrs_uav_bluetooth::peer
