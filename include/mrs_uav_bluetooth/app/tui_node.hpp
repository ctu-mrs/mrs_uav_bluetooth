// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/app/central_client_runtime.hpp"
#include "mrs_uav_bluetooth/app/raw_terminal.hpp"
#include "mrs_uav_bluetooth/gatt/remote_gatt_inspector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

namespace mrs_uav_bluetooth::app {

class TuiNode : public rclcpp::Node {
public:
    TuiNode();
    ~TuiNode() override;

private:
    struct TimeSampleCacheEntry {
        bool available{false};
        bool pending{false};
        bool metrics_available{false};
        uint64_t remote_time_ns{0};
        double rtt_ms{0.0};
        double offset_ms{0.0};
        std::string error;
        std::shared_future<std::tuple<uint64_t, double, double, std::string>> future;
    };

    struct WifiCacheEntry {
        bool available{false};
        bool pending{false};
        bool config_known{false};
        std::string ssid;
        std::string status;
        bool password_configured{false};
        std::string error;
        std::shared_future<std::tuple<std::string, std::string, bool, std::string>> future;
    };

    struct TopicCountCacheEntry {
        size_t count{0};
        bool available{false};
        bool pending{false};
        std::chrono::steady_clock::time_point last_refresh{};
        std::shared_future<gatt::ExportedTopicCountResult> future;
    };

    struct BuiltinSubscriptionState {
        gatt::RemoteBuiltinPaths paths;
        bool wifi_seeded{false};
    };

    enum class NotificationKind {
        Time,
        WifiStatus,
    };

    struct PendingNotification {
        NotificationKind kind{NotificationKind::Time};
        std::string mac;
        uint64_t remote_time_ns{0};
        std::string wifi_status;
    };

    enum class PromptMode {
        None,
        WifiSsid,
        WifiPassword,
    };

    void configure_parameters();
    void ui_tick();
    void handle_input();
    void handle_key_event(const TerminalKeyEvent& event);
    void move_selection(int delta);
    void select_first_visible_device();
    void refresh_device_cache(bool force = false);
    void collect_topic_count_results();
    void collect_time_sample_results();
    void collect_wifi_results();
    void collect_action_result();
    void apply_pending_notifications();
    void request_topic_count_refresh(const bluez::DeviceInfo& device);
    void request_time_sample(const bluez::DeviceInfo& device, bool force = false);
    void request_wifi_refresh(const bluez::DeviceInfo& device, bool force = false);
    void trigger_connect_toggle();
    void trigger_scan_toggle();
    void trigger_wifi_write(PromptMode mode, std::string value);
    void ensure_builtin_subscriptions();
    void ensure_builtin_subscription(const bluez::DeviceInfo& device);
    void on_notification(const std::vector<uint8_t>& data,
                         const std::string& uuid,
                         const std::string& characteristic_path);
    bool effective_services_resolved(const bluez::DeviceInfo& device) const;
    std::string adapter_local_mac();
    const bluez::DeviceInfo* selected_device() const;
    std::vector<const bluez::DeviceInfo*> visible_devices() const;
    std::vector<const bluez::DeviceInfo*> visible_uav_devices() const;
    std::vector<const bluez::DeviceInfo*> visible_other_devices() const;
    void render_dashboard(std::vector<bluez::DeviceInfo> devices);

    std::unique_ptr<CentralClientRuntime> runtime_;
    std::unique_ptr<gatt::RemoteGattInspector> inspector_;
    RawTerminal terminal_;
    rclcpp::TimerBase::SharedPtr ui_timer_;
    std::vector<bluez::DeviceInfo> current_devices_;
    std::map<std::string, TopicCountCacheEntry> topic_counts_;
    std::map<std::string, TimeSampleCacheEntry> time_samples_;
    std::map<std::string, WifiCacheEntry> wifi_state_;
    std::map<std::string, BuiltinSubscriptionState> builtin_subscriptions_;
    std::map<std::string, bool> resolved_service_latch_;
    std::string adapter_alias_;
    std::string scan_mode_;
    std::string uav_name_pattern_;
    std::string selected_mac_;
    std::string status_message_;
    std::string last_frame_;
    std::string action_label_;
    int notification_token_{0};
    std::shared_future<std::string> action_future_;
    std::chrono::steady_clock::time_point last_device_refresh_{};
    std::chrono::steady_clock::time_point last_render_{};
    double refresh_period_sec_{1.0};
    double render_period_sec_{0.1};
    double topic_count_refresh_sec_{5.0};
    bool hide_non_uav_{false};
    PromptMode prompt_mode_{PromptMode::None};
    std::string prompt_buffer_;
    mutable std::mutex pending_notification_mutex_;
    std::vector<PendingNotification> pending_notifications_;
};

}  // namespace mrs_uav_bluetooth::app