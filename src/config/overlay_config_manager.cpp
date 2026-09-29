// SPDX-License-Identifier: BSD-3-Clause
/// \file src/config/overlay_config_manager.cpp
/// \brief Implements the overlay config manager component of the YAML configuration layer.

#include "mrs_uav_bluetooth/config/overlay_config_manager.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace {
constexpr auto kRestartLeaseGrace = std::chrono::seconds(5);
}

namespace mrs_uav_bluetooth::config {

OverlayConfigManager::OverlayConfigManager(rclcpp::Node& node,
                                           rclcpp::Logger logger,
                                           const std::string& default_config_path,
                                           const std::string& hostname)
    : node_(node),
      logger_(logger),
      default_config_path_(default_config_path),
      hostname_(hostname) {
    // Select a restart-persistent lease file only when a private runtime directory exists.
    const char* runtime_dir = std::getenv("XDG_RUNTIME_DIR");
    std::error_code error;
    if (runtime_dir && *runtime_dir &&
        std::filesystem::is_directory(runtime_dir, error)) {
        runtime_state_path_ = (std::filesystem::path(runtime_dir) /
            "mrs_uav_bluetooth-active-overlay.txt").string();
    }
}

void OverlayConfigManager::on_config_changed(ConfigChangedCallback cb) {
    // Register a listener that will receive each successfully loaded overlay snapshot.
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(std::move(cb));
}

void OverlayConfigManager::load_initial() {
    // Restore a crash-persisted overlay when valid; otherwise start from defaults.
    const auto restored = restored_overlay_path();
    if (restored.empty()) {
        apply_config("", "initial load");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_path_ = restored;
        keepalive_miss_count_ = 0;
        lease_hold_until_ = std::chrono::steady_clock::now() +
            kRestartLeaseGrace;
    }
    try {
        apply_config(restored, "restored overlay");
    } catch (const std::exception& error) {
        RCLCPP_WARN(logger_, "Could not restore overlay %s: %s",
                    restored.c_str(), error.what());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            overlay_path_.clear();
            keepalive_miss_count_ = 0;
        }
        clear_persisted_overlay();
        apply_config("", "initial load");
    }
}

std::string OverlayConfigManager::restored_overlay_path() {
    // Read the persisted overlay only when its lease timestamp is still within restart grace.
    if (runtime_state_path_.empty()) return {};
    std::ifstream input(runtime_state_path_);
    std::string overlay;
    if (!std::getline(input, overlay) || overlay.empty()) return {};
    if (!std::filesystem::is_regular_file(overlay)) {
        RCLCPP_WARN(logger_, "Ignoring missing saved overlay: %s",
                    overlay.c_str());
        clear_persisted_overlay();
        return {};
    }
    return overlay;
}

void OverlayConfigManager::persist_overlay_path(
    const std::string& overlay_path) {
    // Write the crash-recovery path to a temporary file, then rename it atomically.
    if (runtime_state_path_.empty()) return;
    const auto temporary = runtime_state_path_ + ".tmp";
    {
        std::ofstream output(temporary, std::ios::trunc);
        output << overlay_path << '\n';
        if (!output) {
            RCLCPP_WARN(logger_, "Could not save active overlay path");
            return;
        }
    }
    std::error_code error;
    std::filesystem::permissions(
        temporary,
        std::filesystem::perms::owner_read |
            std::filesystem::perms::owner_write,
        std::filesystem::perm_options::replace, error);
    error.clear();
    std::filesystem::rename(temporary, runtime_state_path_, error);
    if (error) {
        std::filesystem::remove(temporary);
        RCLCPP_WARN(logger_, "Could not install active overlay state: %s",
                    error.message().c_str());
    }
}

void OverlayConfigManager::clear_persisted_overlay() {
    // Remove crash-recovery state after a deliberate return to defaults.
    if (runtime_state_path_.empty()) return;
    std::error_code error;
    std::filesystem::remove(runtime_state_path_, error);
    if (error) {
        RCLCPP_WARN(logger_, "Could not clear active overlay state: %s",
                    error.message().c_str());
    }
}

std::pair<bool, std::string> OverlayConfigManager::activate_overlay(
    const std::string& overlay_path, double hold_seconds) {
    // Validate the requested lease then load and commit the overlay as one serialized transition.
    if (!std::isfinite(hold_seconds) || hold_seconds < 0.0)
        return {false, "hold_seconds must be finite and nonnegative"};
    const auto previous_overlay = this->overlay_path();
    if (overlay_path.empty()) {
        return revert_to_default();
    }
    if (!std::filesystem::is_regular_file(overlay_path)) {
        return {false, "Overlay file not found: " + overlay_path};
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (overlay_path == overlay_path_) {
            keepalive_miss_count_ = 0;
            lease_hold_until_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(hold_seconds));
            persist_overlay_path(overlay_path);
            return {true, "Overlay already active: " + overlay_path};
        }
        overlay_path_ = overlay_path;
        keepalive_miss_count_ = 0;
        lease_hold_until_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(hold_seconds));
    }
    try {
        apply_config(overlay_path, "overlay activated");
        persist_overlay_path(overlay_path);
        return {true, "Activated overlay: " + overlay_path};
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_path_ = previous_overlay;
        return {false, std::string("Failed to activate overlay: ") + e.what()};
    }
}

std::pair<bool, std::string> OverlayConfigManager::revert_to_default() {
    try {
        // Commit lease metadata only after the radio transition succeeds.
        // apply_config restores the previous runtime on failure, so retaining
        // the overlay path allows a later lease check to retry the handoff.
        apply_config("", "reverted to default");
        clear_persisted_overlay();
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_path_.clear();
        keepalive_miss_count_ = 0;
        overlay_connected_baseline_.clear();
        return {true, "Reverted to default config"};
    } catch (const std::exception& e) {
        return {false, std::string("Failed to revert: ") + e.what()};
    }
}

std::pair<bool, std::string> OverlayConfigManager::reload() {
    // Reload overlay config manager.
    std::string overlay;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay = overlay_path_;
    }
    try {
        apply_config(overlay, "reloaded");
        return {true, "Config reloaded"};
    } catch (const std::exception& e) {
        return {false, std::string("Reload failed: ") + e.what()};
    }
}

void OverlayConfigManager::check_lease() {
    std::string keepalive_topic;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (overlay_path_.empty() ||
            std::chrono::steady_clock::now() < lease_hold_until_) {
            return;
        }
        keepalive_topic = keepalive_topic_;
    }

    if (keepalive_topic.empty()) {
        return;
    }

    // Check if the keepalive topic has publishers via the ROS graph.
    bool has_publishers = false;
    try {
        auto info = node_.get_publishers_info_by_topic(keepalive_topic);
        has_publishers = !info.empty();
    } catch (const std::exception&) {
        has_publishers = false;
    }

    if (has_publishers) {
        std::lock_guard<std::mutex> lock(mutex_);
        keepalive_miss_count_ = 0;
        return;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    ++keepalive_miss_count_;
    if (keepalive_miss_count_ < kRequiredMisses) {
        RCLCPP_DEBUG(logger_, "Overlay keepalive miss %d/%d for %s",
                     keepalive_miss_count_, kRequiredMisses,
                     overlay_path_.c_str());
        return;
    }

    RCLCPP_INFO(logger_, "Overlay keepalive missing (%d misses), reverting from %s",
                keepalive_miss_count_, overlay_path_.c_str());
    keepalive_miss_count_ = 0;

    // Radio callbacks must not execute under the metadata mutex. A failed
    // transition retains the lease so the next expiry check can retry it.
    lock.unlock();
    const auto [success, message] = revert_to_default();
    if (!success) {
        RCLCPP_ERROR(logger_, "Failed to revert after keepalive expiry: %s", message.c_str());
    }
}

NodeConfig OverlayConfigManager::current_config() const {
    // Copy the effective configuration under the same lock used by overlay transitions.
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

std::string OverlayConfigManager::overlay_path() const {
    // Copy the active overlay pathname under the configuration lock.
    std::lock_guard<std::mutex> lock(mutex_);
    return overlay_path_;
}

std::string OverlayConfigManager::active_source() const {
    // Identify whether default configuration or an overlay currently owns the runtime.
    std::lock_guard<std::mutex> lock(mutex_);
    return active_source_;
}

bool OverlayConfigManager::overlay_active() const {
    // Report whether a non-default overlay currently owns the runtime.
    std::lock_guard<std::mutex> lock(mutex_);
    return !overlay_path_.empty();
}

std::string OverlayConfigManager::keepalive_topic() const {
    // Build the normalized lease topic used by the selected overlay.
    std::lock_guard<std::mutex> lock(mutex_);
    return keepalive_topic_;
}

void OverlayConfigManager::capture_connection_baseline(const std::set<std::string>& connected_macs) {
    // Capture connection baseline.
    std::lock_guard<std::mutex> lock(mutex_);
    if (overlay_connected_baseline_.empty()) {
        overlay_connected_baseline_ = connected_macs;
    }
}

std::set<std::string> OverlayConfigManager::lease_expired_macs(
    const std::set<std::string>& current_connected) const {
    // Collect sessions whose peer activity has exceeded the configured lease.
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<std::string> result;
    for (const auto& mac : current_connected) {
        if (overlay_connected_baseline_.find(mac) == overlay_connected_baseline_.end()) {
            result.insert(mac);
        }
    }
    return result;
}

void OverlayConfigManager::clear_connection_baseline() {
    // Forget peers inherited from the overlay activation transaction.
    std::lock_guard<std::mutex> lock(mutex_);
    overlay_connected_baseline_.clear();
}

void OverlayConfigManager::apply_config(const std::string& overlay,
                                         const std::string& source_label) {
    NodeConfig cfg;
    if (overlay.empty()) {
        cfg = load_effective_config(default_config_path_, "", hostname_);
    } else {
        cfg = load_effective_config(default_config_path_, overlay, hostname_);
    }

    auto prefix = util::normalize_ros_topic(cfg.node_topics_prefix);
    auto suffix = util::sanitize_topic_suffix(cfg.overlay_keepalive_topic_suffix);
    auto keepalive_topic = prefix;
    if (keepalive_topic.empty()) {
        keepalive_topic = "/";
    }
    if (keepalive_topic.back() != '/') {
        keepalive_topic += '/';
    }
    keepalive_topic += "config/";
    keepalive_topic += suffix;
    keepalive_topic = util::normalize_ros_topic(keepalive_topic);

    NodeConfig previous;
    std::string previous_source;
    std::string previous_topic;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        previous = config_;
        previous_source = active_source_;
        previous_topic = keepalive_topic_;
        config_ = std::move(cfg);
        active_source_ = overlay.empty() ? default_config_path_ : overlay;
        keepalive_topic_ = std::move(keepalive_topic);
    }
    try {
        notify();
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            config_ = std::move(previous);
            active_source_ = previous_source;
            keepalive_topic_ = previous_topic;
        }
        if (!previous_source.empty()) {
            try { notify(); }
            catch (const std::exception& error) {
                RCLCPP_ERROR(logger_, "Overlay rollback also failed: %s", error.what());
            }
        }
        throw;  // Never claim that a failed radio transition was applied.
    }
    RCLCPP_INFO(logger_, "Config applied (%s): source=%s",
                source_label.c_str(), active_source_.c_str());
}

void OverlayConfigManager::notify() {
    // Notify overlay config manager.
    std::vector<ConfigChangedCallback> cbs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cbs = callbacks_;
    }
    NodeConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_;
    }
    for (auto& cb : cbs) cb(cfg);
}

}  // namespace mrs_uav_bluetooth::config
