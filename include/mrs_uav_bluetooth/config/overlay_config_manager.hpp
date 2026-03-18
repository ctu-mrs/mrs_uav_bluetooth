// SPDX-License-Identifier: MIT
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
    OverlayConfigManager(rclcpp::Node& node, rclcpp::Logger logger,
                         const std::string& default_config_path,
                         const std::string& hostname);

    /// Register a callback to be invoked when the effective config changes.
    void on_config_changed(ConfigChangedCallback cb);

    /// Load the initial (default) config and invoke callbacks.
    void load_initial();

    /// Activate an overlay file.  Returns success + message.
    std::pair<bool, std::string> activate_overlay(const std::string& overlay_path);

    /// Revert to default config.  Returns success + message.
    std::pair<bool, std::string> revert_to_default();

    /// Reload the active config (default + overlay if any).
    std::pair<bool, std::string> reload();

    /// Called periodically (e.g. from a ROS timer) to check whether the overlay
    /// keepalive sentinel topic still has publishers.
    void check_lease();

    /// Current effective config (thread-safe copy).
    NodeConfig current_config() const;

    /// Path of the active overlay file, or empty if none.
    std::string overlay_path() const;

    /// Path of the effective config source.
    std::string active_source() const;

    /// Whether an overlay is active.
    bool overlay_active() const;

    /// Fully resolved keepalive topic for the active effective config.
    std::string keepalive_topic() const;

    // --- Overlay connection baseline for lease-expire disconnects ---

    /// Capture the set of currently connected MACs as the baseline before
    /// overlay activation.
    void capture_connection_baseline(const std::set<std::string>& connected_macs);

    /// Retrieve the set of MACs that should be disconnected when the overlay
    /// lease expires (current − baseline).
    std::set<std::string> lease_expired_macs(const std::set<std::string>& current_connected) const;

    /// Clear the connection baseline.
    void clear_connection_baseline();

private:
    void apply_config(const std::string& overlay, const std::string& source_label);
    void notify();

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    std::string default_config_path_;
    std::string hostname_;

    mutable std::mutex mutex_;
    NodeConfig config_;
    std::string overlay_path_;
    std::string active_source_;
    std::string keepalive_topic_;
    int keepalive_miss_count_{0};
    static constexpr int kRequiredMisses = 3;

    std::set<std::string> overlay_connected_baseline_;

    std::vector<ConfigChangedCallback> callbacks_;
};

}  // namespace mrs_uav_bluetooth::config
