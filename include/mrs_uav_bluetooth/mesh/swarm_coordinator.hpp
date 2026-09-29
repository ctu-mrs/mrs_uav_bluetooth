// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/mesh/swarm_coordinator.hpp
/// \brief Declares the swarm coordinator component of the Bluetooth Mesh D-Bus layer.

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

/// Leader-independent swarm enrollment and ordered coordinator failover.
///
/// Every UAV imports the common Mesh credentials at a stable unique address.
/// Availability heartbeats travel over encrypted Mesh, including relay hops.
/// The first reachable allowed peer is coordinator; loss of that peer never
/// resets identities or prevents another swarm member from starting alone.
/// Logical swarm changes are application admission, not security isolation.
class MeshSwarmCoordinator {
public:
    /// \brief Coordinate N-peer membership, provisioner election, and radio turns.
    /// \param application Mesh application controlled by this coordinator.
    /// \param config Node configuration defining Mesh membership identity and coordinator timing.
    /// \param hostname UAV hostname used to identify the node.
    /// \param logger ROS logger used for diagnostics.
    MeshSwarmCoordinator(MeshApplication& application,
                         config::NodeConfig config,
                         std::string hostname,
                         rclcpp::Logger logger);

    /// Consume a private coordination access message when recognized.
    /// Ordinary vendor messages return false for the generalized Mesh bridge.
    /// \param message Decoded Mesh control message to validate and apply.
    /// \return True if the packet is a valid private coordination frame and was consumed; otherwise false.
    bool handle_message(const ReceivedMessage& message);
    /// Queue BlueZ scan/provisioning callbacks for the timer thread. Never
    /// make a provisioning D-Bus call inside a D-Bus callback.
    /// \param event Mesh lifecycle event used to update coordinator state.
    void handle_event(const Event& event);

    /// Advance import/attach, confirmed model setup, and ordered heartbeat leases.
    /// \param status Latest Mesh attachment key and node inventory used for coordination.
    void maintain(const Status& status);

    /// \brief Read the provisioner elected from current live membership.
    /// \return Hostname of the provisioner elected from live membership.
    std::string active_provisioner() const;
    /// \brief Resolve the configured provisioner preference or ordered election.
    /// \return Configured preferred provisioner or ordered-election marker.
    std::string preferred_provisioner() const;
    /// \brief Check whether this node currently owns provisioning duties.
    /// \return True when this node currently owns provisioning duties; otherwise false.
    bool local_is_active_provisioner() const;
    /// \brief Check whether Mesh is attached and configured for data traffic.
    /// \return True when the Mesh application is attached and configured; otherwise false.
    bool application_ready() const;

    /// Change only this UAV's logical application swarm, never its Mesh keys.
    /// The state is persisted before publication so reboot does not silently
    /// restore a previously left swarm.
    /// \param swarm_id Nonzero logical Mesh group to join.
    void join_swarm(uint16_t swarm_id);
    /// \brief Persist a nonparticipating state and announce departure without changing Mesh keys.
    void leave_swarm();
    /// \brief Return the deterministic identifier for admitted membership.
    /// \return Deterministic identifier for the configured Mesh member set.
    uint16_t swarm_id() const;
    /// \brief Check whether the local node currently participates.
    /// \return True when this node participates in the configured Mesh group; otherwise false.
    bool swarm_participating() const;
    /// \brief Return canonical hostnames for admitted members.
    /// \return Canonical hostnames of admitted Mesh members.
    std::vector<std::string> swarm_members() const;
    /// Fresh peer unicast addresses for acknowledged automatic topic delivery.
    /// \return Unicast addresses of currently present Mesh members.
    std::vector<uint16_t> swarm_member_addresses() const;
    /// Configured peers with no fresh same-swarm presence. The sender probes
    /// these sparingly so heartbeat loss cannot permanently disable repairs.
    /// \return Member addresses that still require direct probing.
    std::vector<uint16_t> swarm_probe_addresses() const;

    /// Raw Mesh messages use the latest authenticated control heartbeat.
    /// \param source Mesh unicast address claimed by the sender.
    /// \return True when the requested value is accepted; otherwise false.
    bool accepts_swarm_payload(uint16_t source) const;
    /// Reliable bridge frames carry their own swarm ID, so a lost heartbeat
    /// cannot reject valid topic data from an admitted Mesh peer.
    /// \param source Mesh unicast address claimed by the sender.
    /// \param sender_swarm_id identifier of the sender swarm.
    /// \return True when the requested value is accepted; otherwise false.
    bool accepts_swarm_payload(uint16_t source, uint16_t sender_swarm_id);

private:
    using Clock = std::chrono::steady_clock;

    struct PeerPresence {
        uint16_t source{0};
        uint16_t swarm_id{0};
        bool participating{false};
        Clock::time_point expires{};
    };

    /// \brief Expire stale heartbeats and elect one provisioner deterministically from all live allowed members.
    /// \param now current monotonic time.
    void update_selection(Clock::time_point now);
    /// \brief Broadcast this node's group participation and provisioner preference.
    /// \param now current monotonic time.
    void publish_coordination_heartbeat(Clock::time_point now);
    /// \brief Validate one authenticated heartbeat and refresh that sender's presence lease.
    /// \param frame decoded coordinator frame whose membership or heartbeat is applied.
    /// \param source Mesh unicast address that sent the coordination frame.
    /// \param now current monotonic time.
    void accept_coordination_frame(const std::vector<uint8_t>& frame,
                                   uint16_t source,
                                   Clock::time_point now);
    /// \brief Atomically store logical group membership for restart recovery.
    /// \param swarm_id Logical Mesh group stored for the next process start.
    /// \param participating whether this node should advertise itself as an active Mesh member.
    void persist_swarm_state(uint16_t swarm_id, bool participating) const;
    /// \brief Restore the last logical group and participation flag if the state file is valid.
    void load_swarm_state();
    /// \brief Check one numeric peer against admission policy.
    /// \param number Numeric UAV suffix decoded from the Mesh UUID.
    /// \return True when peer allowed; otherwise false.
    bool peer_allowed(uint32_t number) const;
    /// \brief Format a numeric member address as a canonical UAV hostname.
    /// \param number Numeric UAV suffix decoded from the Mesh UUID.
    /// \return Canonical UAV hostname for the numeric member address.
    std::string member_name(uint32_t number) const;
    /// \brief Drive confirmed local AppKey binding and group subscription to readiness.
    /// \param status Latest Mesh attachment and local model configuration state.
    /// \param now current monotonic time.
    void maintain_local_model(const Status& status, Clock::time_point now);
    /// \brief Provision or repair one missing allowed member without blocking other peers.
    /// \param status Latest Mesh topology used to discover members needing enrollment.
    /// \param now current monotonic time.
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
