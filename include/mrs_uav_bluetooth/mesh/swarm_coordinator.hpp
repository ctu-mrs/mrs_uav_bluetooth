// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/mesh/mesh_application.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::mesh {

/// Leader-independent fleet enrollment and ordered coordinator failover.
///
/// Every UAV imports the common fleet credentials at a stable unique address.
/// Availability heartbeats travel over encrypted Mesh, including relay hops.
/// The first reachable allowed peer is coordinator; loss of that peer never
/// resets identities or prevents another fleet member from starting alone.
/// Logical swarm changes are application admission, not security isolation.
class MeshSwarmCoordinator {
public:
    MeshSwarmCoordinator(MeshApplication& application,
                         config::NodeConfig config,
                         std::string hostname,
                         rclcpp::Logger logger);

    /// Consume a private coordination access message when recognized.
    /// Ordinary vendor messages return false for the generalized Mesh bridge.
    bool handle_message(const ReceivedMessage& message);
    /// Queue BlueZ scan/provisioning callbacks for the timer thread. Never
    /// make a provisioning D-Bus call inside a D-Bus callback.
    void handle_event(const Event& event);

    /// Advance import/attach, confirmed model setup, and ordered heartbeat leases.
    void maintain(const Status& status);

    std::string active_provisioner() const;
    std::string preferred_provisioner() const;
    bool local_is_active_provisioner() const;
    bool application_ready() const;

    /// Change only this UAV's logical application swarm, never its Mesh keys.
    /// The state is persisted before publication so reboot does not silently
    /// restore a previously left swarm.
    void join_swarm(uint16_t swarm_id);
    void leave_swarm();
    uint16_t swarm_id() const;
    bool swarm_participating() const;
    std::vector<std::string> swarm_members() const;

    /// Admission for bridged data: the source must have a fresh authenticated
    /// control heartbeat and advertise this UAV's current logical swarm.
    bool accepts_swarm_payload(uint16_t source) const;

private:
    using Clock = std::chrono::steady_clock;

    struct PeerPresence {
        uint16_t source{0};
        uint16_t swarm_id{0};
        bool participating{false};
        Clock::time_point expires{};
    };

    void update_selection(Clock::time_point now);
    void publish_coordination_heartbeat(Clock::time_point now);
    void accept_coordination_frame(const std::vector<uint8_t>& frame,
                                   uint16_t source,
                                   Clock::time_point now);
    void persist_swarm_state(uint16_t swarm_id, bool participating) const;
    void load_swarm_state();
    bool peer_allowed(uint32_t number) const;
    std::string member_name(uint32_t number) const;
    void maintain_local_model(const Status& status, Clock::time_point now);
    void maintain_auto_enrollment(const Status& status, Clock::time_point now);
    MeshApplication& application_;
    config::NodeConfig config_;
    std::string hostname_;
    rclcpp::Logger logger_;
    uint32_t local_number_{0};
    uint32_t preferred_number_{0};
    uint32_t whitelist_fingerprint_{0};
    std::map<uint32_t, std::string> member_by_number_;
    std::vector<uint32_t> member_priority_;

    mutable std::mutex mutex_;
    std::map<uint32_t, PeerPresence> peer_presence_;
    uint32_t active_number_{0};
    std::string active_provisioner_;
    std::atomic_bool local_is_active_{false};
    Clock::time_point active_since_{};
    uint16_t swarm_id_{1};
    bool swarm_participating_{true};
    Clock::time_point last_heartbeat_{};
    Clock::time_point bootstrap_started_at_{};
    Clock::time_point last_bootstrap_action_{};

    uint8_t local_model_stage_{0};
    Clock::time_point next_local_model_action_{};
    std::atomic_bool application_ready_{false};
    bool automatic_private_{false};
    std::deque<Event> pending_events_;
    std::string pending_provision_uuid_;
    uint16_t pending_provision_address_{0};
    uint8_t remote_model_stage_{0};
    Clock::time_point remote_model_action_at_{};
    Clock::time_point pending_provision_until_{};
    Clock::time_point last_scan_at_{};
    std::map<uint32_t, Clock::time_point> last_direct_attempt_;

};

}  // namespace mrs_uav_bluetooth::mesh
