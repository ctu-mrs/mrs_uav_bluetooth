// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/config/overlay_config_manager.hpp"

#include <filesystem>

namespace mrs_uav_bluetooth::config {

OverlayConfigManager::OverlayConfigManager(rclcpp::Node& node,
                                           rclcpp::Logger logger,
                                           const std::string& default_config_path,
                                           const std::string& hostname)
    : node_(node),
      logger_(logger),
      default_config_path_(default_config_path),
      hostname_(hostname) {}

void OverlayConfigManager::on_config_changed(ConfigChangedCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(std::move(cb));
}

void OverlayConfigManager::load_initial() {
    apply_config("", "initial load");
}

std::pair<bool, std::string> OverlayConfigManager::activate_overlay(const std::string& overlay_path) {
    if (overlay_path.empty()) {
        return revert_to_default();
    }
    if (!std::filesystem::is_regular_file(overlay_path)) {
        return {false, "Overlay file not found: " + overlay_path};
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (overlay_path == overlay_path_) {
            return {true, "Overlay already active: " + overlay_path};
        }
        overlay_path_ = overlay_path;
        keepalive_miss_count_ = 0;
    }
    try {
        apply_config(overlay_path, "overlay activated");
        return {true, "Activated overlay: " + overlay_path};
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_path_.clear();
        return {false, std::string("Failed to activate overlay: ") + e.what()};
    }
}

std::pair<bool, std::string> OverlayConfigManager::revert_to_default() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        overlay_path_.clear();
        keepalive_miss_count_ = 0;
        overlay_connected_baseline_.clear();
    }
    try {
        apply_config("", "reverted to default");
        return {true, "Reverted to default config"};
    } catch (const std::exception& e) {
        return {false, std::string("Failed to revert: ") + e.what()};
    }
}

std::pair<bool, std::string> OverlayConfigManager::reload() {
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
        if (overlay_path_.empty()) {
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
    overlay_path_.clear();
    keepalive_miss_count_ = 0;

    // Unlock before apply_config to avoid holding the lock during callbacks.
    lock.unlock();
    try {
        apply_config("", "keepalive expired");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to revert after keepalive expiry: %s", e.what());
    }
}

NodeConfig OverlayConfigManager::current_config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

std::string OverlayConfigManager::overlay_path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return overlay_path_;
}

std::string OverlayConfigManager::active_source() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_source_;
}

bool OverlayConfigManager::overlay_active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !overlay_path_.empty();
}

std::string OverlayConfigManager::keepalive_topic() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keepalive_topic_;
}

void OverlayConfigManager::capture_connection_baseline(const std::set<std::string>& connected_macs) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (overlay_connected_baseline_.empty()) {
        overlay_connected_baseline_ = connected_macs;
    }
}

std::set<std::string> OverlayConfigManager::lease_expired_macs(
    const std::set<std::string>& current_connected) const {
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
    keepalive_topic += suffix;
    keepalive_topic = util::normalize_ros_topic(keepalive_topic);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = std::move(cfg);
        active_source_ = overlay.empty() ? default_config_path_ : overlay;
        keepalive_topic_ = std::move(keepalive_topic);
    }

    RCLCPP_INFO(logger_, "Config applied (%s): source=%s",
                source_label.c_str(), active_source_.c_str());
    notify();
}

void OverlayConfigManager::notify() {
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
    for (auto& cb : cbs) {
        try {
            cb(cfg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Config-change callback threw: %s", e.what());
        }
    }
}

}  // namespace mrs_uav_bluetooth::config
