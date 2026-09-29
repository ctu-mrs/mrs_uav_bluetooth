// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/peer/peer_manager.hpp
/// \brief Declares the peer manager component of the peer lifecycle layer.

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

/// Maintains desired peer sessions and decides which BlueZ action is needed next.
/// Slow connect/pair operations are executed by ServiceNode outside its mutex.
class PeerManager {
public:
    /// \brief Create the owner of peer sessions, retry state, and clock bridges.
    /// \param node ROS node that owns the created interfaces.
    /// \param logger ROS logger used for diagnostics.
    PeerManager(rclcpp::Node& node, rclcpp::Logger logger);

    /// \brief Create or refresh the state record for one peer address.
    /// \param mac peer Bluetooth MAC address.
    /// \param peer_name Resolved peer hostname stored in the session when available.
    /// \return Existing session for the normalized address or a newly initialized one.
    PeerConnectionSession& get_or_create_session(const std::string& mac,
                                                 const std::string& peer_name = {});
    /// \brief Merge a BlueZ device snapshot into its desired peer session state.
    /// \param device Latest BlueZ snapshot used to synchronize the peer session.
    /// \param config Connection pairing trust and allow-list policy for this peer.
    /// \param peer_name Resolved peer hostname stored in the session when available.
    /// \param preserve_ready_runtime whether an already ready peer keeps its active runtime objects.
    /// \param preserve_active_bridge_runtime whether live bridge objects survive cache synchronization.
    void sync_device(const bluez::DeviceInfo& device,
                     const config::NodeConfig& config,
                     const std::string& peer_name = {},
                     bool preserve_ready_runtime = false,
                     bool preserve_active_bridge_runtime = false);
    /// \brief Mark when a known peer first disappears from the BlueZ cache.
    /// \param mac peer Bluetooth MAC address.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    void note_missing_device(const std::string& mac, double now_mono);
    /// \brief Apply one pairing-agent event to the session owning its device path.
    /// \param device_path BlueZ device path used to find and update the peer session.
    /// \param event Pairing-agent event used to advance the peer session.
    /// \param cache ObjectManager cache used to resolve the agent's device path to an address.
    void note_pairing_event(const std::string& device_path,
                            const std::string& event,
                            const bluez::ObjectManagerCache& cache);
    /// \brief Mark a peer for removal and controlled rediscovery after stale security state.
    /// \param session Peer session to mark for device removal and rediscovery.
    /// \param reason Diagnostic explaining why the peer must be removed and rediscovered.
    /// \param reset_pairing_state whether stale pairing progress is cleared with the runtime.
    void request_device_reset(PeerConnectionSession& session,
                              const std::string& reason,
                              bool reset_pairing_state = false);
    /// \brief Clear reset flags after the peer has been removed or recovered.
    /// \param session Peer session whose completed reset flags are cleared.
    /// \param clear_pairing_reset_pending whether the outstanding pairing-reset marker is cleared.
    void clear_device_reset(PeerConnectionSession& session,
                            bool clear_pairing_reset_pending = true);
    /// \brief Decide whether a connected secure peer is due for automatic trust.
    /// \param session Peer retry and in-flight state checked before starting trust.
    /// \param device Latest BlueZ trusted paired and connection flags for the peer.
    /// \param config Security policy controlling the peer action.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    /// \param retry_period_s minimum seconds between repeated peer operations.
    /// \return True when the caller should attempt trust; otherwise false.
    bool should_attempt_trust(const PeerConnectionSession& session,
                              const bluez::DeviceInfo& device,
                              const config::NodeConfig& config,
                              double now_mono,
                              double retry_period_s) const;
    /// \brief Decide whether a desired visible peer is due for a connection retry.
    /// \param session Peer retry and in-flight state checked before starting a connection.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    /// \param retry_period_s minimum seconds between repeated peer operations.
    /// \return True when the caller should attempt connect; otherwise false.
    bool should_attempt_connect(const PeerConnectionSession& session,
                                double now_mono,
                                double retry_period_s) const;
    /// \brief Decide whether a connected peer is due for an automatic pairing attempt.
    /// \param session Peer retry and in-flight state checked before starting pairing.
    /// \param device Latest BlueZ pairing and connection flags for the peer.
    /// \param config Security policy controlling the peer action.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    /// \param retry_period_s minimum seconds between repeated peer operations.
    /// \return True when the caller should attempt pair; otherwise false.
    bool should_attempt_pair(const PeerConnectionSession& session,
                             const bluez::DeviceInfo& device,
                             const config::NodeConfig& config,
                             double now_mono,
                             double retry_period_s) const;
    /// Recover only an admitted, automatically paired GATT peer whose stored
    /// bond failed authentication; RF loss and exclusive modes never qualify.
    /// \param session Peer repair history and phase after an authentication disconnect.
    /// \param device Latest BlueZ security flags after the authentication failure.
    /// \param config Security policy controlling the peer action.
    /// \param reason BlueZ disconnect detail checked for an authentication failure.
    /// \return True when the caller should repair authentication disconnect; otherwise false.
    bool should_repair_authentication_disconnect(const PeerConnectionSession& session,
                                                 const bluez::DeviceInfo& device,
                                                 const config::NodeConfig& config,
                                                 const std::string& reason) const;
    /// \brief Record that this process initiated the current connection attempt.
    /// \param session Peer session recording the local connection timestamp and ownership.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    void note_local_connect_attempt(PeerConnectionSession& session,
                                    double now_mono) const;
    /// \brief Remove idle peer sessions and their clock bridges after expiry.
    /// \param current_macs peer addresses observed in the latest BlueZ cache snapshot.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    /// \param ttl_s seconds an unseen peer session may remain cached.
    void prune_sessions(const std::set<std::string>& current_macs, double now_mono, double ttl_s);
    /// \brief Read steady-clock seconds for peer retry decisions.
    /// \return Current steady-clock time in seconds.
    double now_monotonic() const;

    /// \brief Access the peer session map used by reconciliation.
    /// \return Per-peer connection state map.
    std::map<std::string, PeerConnectionSession>& sessions() {
        // Expose mutable peer sessions to the serialized reconciliation loop.
        return sessions_;
    }
    /// \brief Access the peer session map used by reconciliation.
    /// \return Per-peer connection state map.
    const std::map<std::string, PeerConnectionSession>& sessions() const {
        // Expose peer sessions read-only for status publication.
        return sessions_;
    }

    /// \brief Access the per-peer clock bridge map.
    /// \return Per-peer clock bridge map.
    std::map<std::string, PeerTimeBridge>& time_bridges() {
        // Expose mutable per-peer clock bridges to reconciliation code.
        return time_bridges_;
    }
    /// \brief Access the per-peer clock bridge map.
    /// \return Per-peer clock bridge map.
    const std::map<std::string, PeerTimeBridge>& time_bridges() const {
        // Expose clock bridges read-only for status publication.
        return time_bridges_;
    }

    /// \brief Drop one peer's clock-offset state and publisher.
    /// \param mac peer Bluetooth MAC address.
    void remove_time_bridge(const std::string& mac);

private:
    /// \brief Check whether the peer name matches automatic connection policy.
    /// \param device Discovered peer evaluated against the configured connection policy.
    /// \param config Discovery and allow-list policy used to admit the peer.
    /// \param peer_name Resolved peer hostname matched against the allow-list.
    /// \return True when matches auto connect policy; otherwise false.
    bool matches_auto_connect_policy(const bluez::DeviceInfo& device,
                                     const config::NodeConfig& config,
                                     const std::string& peer_name) const;
    /// \brief Store a peer lifecycle phase only when it changes and retain its diagnostic.
    /// \param session Peer session receiving the new lifecycle phase and detail.
    /// \param phase New peer lifecycle phase.
    /// \param detail Operator-facing explanation stored with the peer phase.
    void set_session_phase(PeerConnectionSession& session,
                           const std::string& phase,
                           const std::string& detail) const;

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    std::map<std::string, PeerConnectionSession> sessions_;
    std::map<std::string, PeerTimeBridge> time_bridges_;
};

}  // namespace mrs_uav_bluetooth::peer
