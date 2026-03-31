// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"
#include "mrs_uav_bluetooth/peer/peer_session.hpp"
#include "mrs_uav_bluetooth/peer/peer_time_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <optional>
#include <set>
#include <string>

namespace mrs_uav_bluetooth::peer {

class PeerManager {
public:
    PeerManager(rclcpp::Node& node, rclcpp::Logger logger);

    PeerConnectionSession& get_or_create_session(const std::string& mac,
                                                 const std::string& peer_name = {});
    void sync_device(const bluez::DeviceInfo& device,
                     const config::NodeConfig& config,
                     const std::string& peer_name = {});
    void note_missing_device(const std::string& mac, double now_mono);
    void note_pairing_event(const std::string& device_path,
                            const std::string& event,
                            const bluez::ObjectManagerCache& cache);
    void request_device_reset(PeerConnectionSession& session,
                              const std::string& reason,
                              bool reset_pairing_state = false);
    void clear_device_reset(PeerConnectionSession& session,
                            bool clear_pairing_reset_pending = true);
    bool should_attempt_trust(const PeerConnectionSession& session,
                              const bluez::DeviceInfo& device,
                              const config::NodeConfig& config,
                              double now_mono,
                              double retry_period_s) const;
    bool should_attempt_connect(const PeerConnectionSession& session,
                                double now_mono,
                                double retry_period_s) const;
    bool should_attempt_pair(const PeerConnectionSession& session,
                             const bluez::DeviceInfo& device,
                             const config::NodeConfig& config,
                             double now_mono,
                             double retry_period_s) const;
    void note_local_connect_attempt(PeerConnectionSession& session,
                                    double now_mono) const;
    void prune_sessions(const std::set<std::string>& current_macs, double now_mono, double ttl_s);
    double now_monotonic() const;

    std::map<std::string, PeerConnectionSession>& sessions() { return sessions_; }
    const std::map<std::string, PeerConnectionSession>& sessions() const { return sessions_; }

    std::map<std::string, PeerTimeBridge>& time_bridges() { return time_bridges_; }
    const std::map<std::string, PeerTimeBridge>& time_bridges() const { return time_bridges_; }

    void remove_time_bridge(const std::string& mac);

private:
    bool matches_auto_connect_policy(const bluez::DeviceInfo& device,
                                     const config::NodeConfig& config,
                                     const std::string& peer_name) const;
    void set_session_phase(PeerConnectionSession& session,
                           const std::string& phase,
                           const std::string& detail) const;

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    std::map<std::string, PeerConnectionSession> sessions_;
    std::map<std::string, PeerTimeBridge> time_bridges_;
};

}  // namespace mrs_uav_bluetooth::peer
