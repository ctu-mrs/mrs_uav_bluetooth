// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/config/overlay_config_manager.hpp
/// \brief Declares the overlay config manager component of the YAML configuration layer.

#pragma once

#include "mrs_uav_bluetooth/config/config_loader.hpp"
#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <mutex>
#include <set>
#include <string>

namespace mrs_uav_bluetooth::config {

/// Callback invoked after the effective config has changed (overlay applied or
/// reverted).  The argument is the new effective NodeConfig.
using ConfigChangedCallback = std::function<void(const NodeConfig&)>;

/// Manages the overlay config lifecycle: activation, keepalive lease
/// monitoring, and automatic revert when the lease-sentinel topic disappears.
class OverlayConfigManager {
public:
    /// \brief Manage default configuration, overlay leases, and restart restoration.
    /// \param node ROS node that owns the created interfaces.
    /// \param logger ROS logger used for diagnostics.
    /// \param default_config_path path of the default config.
    /// \param hostname UAV hostname used to identify the node.
    OverlayConfigManager(rclcpp::Node& node, rclcpp::Logger logger,
                         const std::string& default_config_path,
                         const std::string& hostname);

    /// Register a callback to be invoked when the effective config changes.
    /// \param cb Observer invoked after a new overlay becomes active.
    void on_config_changed(ConfigChangedCallback cb);

    /// Load the initial (default) config and invoke callbacks.
    void load_initial();

    /// Activate an overlay file.  Returns success + message.
    /// \param overlay_path YAML overlay file to validate and activate.
    /// \param hold_seconds duration for which this member retains the shared Mesh radio turn.
    /// \return Success flag and operator-facing diagnostic message.
    std::pair<bool, std::string> activate_overlay(
        const std::string& overlay_path, double hold_seconds = 0.0);

    /// Revert to default config.  Returns success + message.
    /// \return Success flag and operator-facing diagnostic message.
    std::pair<bool, std::string> revert_to_default();

    /// Reload the active config (default + overlay if any).
    /// \return Success flag and operator-facing diagnostic message.
    std::pair<bool, std::string> reload();

    /// Called periodically (e.g. from a ROS timer) to check whether the overlay
    /// keepalive sentinel topic still has publishers.
    void check_lease();

    /// Current effective config (thread-safe copy).
    /// \return Locked copy of the effective configuration.
    NodeConfig current_config() const;

    /// Path of the active overlay file, or empty if none.
    /// \return Path of the active overlay, or an empty string for the default.
    std::string overlay_path() const;

    /// Path of the effective config source.
    /// \return Human-readable name of the configuration that owns the runtime.
    std::string active_source() const;

    /// Whether an overlay is active.
    /// \return True when an overlay currently owns the runtime; otherwise false.
    bool overlay_active() const;

    /// Fully resolved keepalive topic for the active effective config.
    /// \return Normalized ROS topic used for the overlay lease.
    std::string keepalive_topic() const;

    // --- Overlay connection baseline for lease-expire disconnects ---

    /// Capture the set of currently connected MACs as the baseline before
    /// overlay activation.
    /// \param connected_macs addresses whose active links must be reconciled.
    void capture_connection_baseline(const std::set<std::string>& connected_macs);

    /// Retrieve the set of MACs that should be disconnected when the overlay
    /// lease expires (current − baseline).
    /// \param current_connected whether Device1 currently reports this peer connected.
    /// \return Peer addresses whose keepalive leases have expired.
    std::set<std::string> lease_expired_macs(const std::set<std::string>& current_connected) const;

    /// Forget peers that were already connected when the active overlay began.
    void clear_connection_baseline();

private:
    /// \brief Merge the selected overlay over defaults and publish one validated configuration snapshot.
    /// \param overlay Overlay file path resolved and merged over the base configuration.
    /// \param source_label human-readable source name included in decoded bridge diagnostics.
    void apply_config(const std::string& overlay, const std::string& source_label);
    /// \brief Deliver the committed effective configuration to every registered observer.
    void notify();
    /// \brief Recover a persisted overlay only while its restart lease is valid.
    /// \return Persisted overlay path when its restart lease is still valid.
    std::string restored_overlay_path();
    /// \brief Atomically store the active overlay path for restart recovery.
    /// \param overlay_path Active overlay path written to persistent state.
    void persist_overlay_path(const std::string& overlay_path);
    /// \brief Remove persisted overlay state after returning to defaults.
    void clear_persisted_overlay();

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    std::string default_config_path_;
    std::string hostname_;

    std::string runtime_state_path_;
    mutable std::mutex mutex_;
    NodeConfig config_;
    std::string overlay_path_;
    std::string active_source_;
    std::string keepalive_topic_;
    int keepalive_miss_count_{0};
    /// Minimum initial hold requested through SetActiveConfig; zero disables it.
    std::chrono::steady_clock::time_point lease_hold_until_{};
    static constexpr int kRequiredMisses = 3;

    std::set<std::string> overlay_connected_baseline_;

    std::vector<ConfigChangedCallback> callbacks_;
};

}  // namespace mrs_uav_bluetooth::config
