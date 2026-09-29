// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/app/tui_node.hpp
/// \brief Declares the tui node component of the ROS 2 application and operator-tool layer.

#pragma once

#include "mrs_uav_bluetooth/app/central_client_runtime.hpp"
#include "mrs_uav_bluetooth/app/raw_terminal.hpp"
#include "mrs_uav_bluetooth/bluez/serial_port_profile.hpp"
#include "mrs_uav_bluetooth/gatt/remote_gatt_inspector.hpp"
#include "mrs_uav_bluetooth/serial/serial_link.hpp"

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

/// Interactive ROS 2 terminal client for discovery and peer control.
class TuiNode : public rclcpp::Node {
public:
    /// \brief Create the interactive Bluetooth client and its terminal dashboard.
    TuiNode();
    /// \brief Remove client handlers and close all serial terminals.
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
        std::string password;
        std::string status;
        std::string error;
        std::chrono::steady_clock::time_point last_attempt{};
        std::shared_future<std::tuple<std::string, std::string, std::string, std::string>> future;
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

    /// \brief Load dashboard timing, filtering, helper, and topic parameters.
    void configure_parameters();
    /// \brief Advance pending work and render one dashboard frame.
    void ui_tick();
    /// \brief Read all available terminal keys without blocking the ROS executor.
    void handle_input();
    /// \brief Dispatch one decoded key to navigation connection scan Wi-Fi or quit actions.
    /// \param event Decoded terminal key press to dispatch.
    void handle_key_event(const TerminalKeyEvent& event);
    /// \brief Move the selected peer while keeping the cursor inside the visible list.
    /// \param delta signed selection movement applied to the current UI row.
    void move_selection(int delta);
    /// \brief Restore a valid selection after filtering or device removal.
    void select_first_visible_device();
    /// \brief Refresh peer GATT metadata without issuing duplicate in-flight reads.
    /// \param force Whether to refresh even inside the normal cache throttle interval.
    void refresh_device_cache(bool force = false);
    /// \brief Harvest completed descriptor reads and update each peer's bridge count.
    void collect_topic_count_results();
    /// \brief Harvest completed time reads and calculate peer clock offsets.
    void collect_time_sample_results();
    /// \brief Harvest completed Wi-Fi characteristic reads into the peer display cache.
    void collect_wifi_results();
    /// \brief Finish one asynchronous operator action and retain its diagnostic.
    void collect_action_result();
    /// \brief Move notification updates from D-Bus threads into dashboard state.
    void apply_pending_notifications();
    /// \brief Start asynchronous descriptor discovery for one peer's exported bridges.
    /// \param device Connected peer whose exported bridge descriptors should be counted.
    void request_topic_count_refresh(const bluez::DeviceInfo& device);
    /// \brief Start a nonblocking read of one peer's clock characteristic.
    /// \param device Connected peer whose time characteristic should be sampled.
    /// \param force Whether to issue the read even when a recent request or value exists.
    void request_time_sample(const bluez::DeviceInfo& device, bool force = false);
    /// \brief Start nonblocking SSID, password, and status reads for one peer.
    /// \param device Connected peer whose Wi-Fi characteristics should be read.
    /// \param force Whether to issue the read even when a recent request or value exists.
    void request_wifi_refresh(const bluez::DeviceInfo& device, bool force = false);
    /// \brief Connect a disconnected selection or disconnect a connected one asynchronously.
    void trigger_connect_toggle();
    /// \brief Open the selected peer's RFCOMM terminal without blocking the dashboard.
    void trigger_serial_connect();
    /// \brief Launch the Bluetooth SSH helper for the selected peer.
    void trigger_ssh();
    /// \brief Start or stop discovery and update the requested scan state.
    void trigger_scan_toggle();
    /// \brief Apply an operator-entered SSID or password to the selected peer.
    /// \param mode Whether the input is an SSID or password.
    /// \param value Network name or password entered by the operator.
    void trigger_wifi_write(PromptMode mode, std::string value);
    /// \brief Resolve the installed helper used for Bluetooth SSH.
    /// \return Installed Bluetooth SSH launcher path or an empty path when unavailable.
    std::string ssh_wrapper_path() const;
    /// \brief Enable built-in notifications on every connected resolved peer.
    void ensure_builtin_subscriptions();
    /// \brief Subscribe once to a peer's built-in Wi-Fi and time characteristics.
    /// \param device Connected peer whose built-in characteristic notifications should be enabled.
    void ensure_builtin_subscription(const bluez::DeviceInfo& device);
    /// \brief Queue a built-in Wi-Fi or time notification for the UI thread.
    /// \param data Bytes received from the remote GATT characteristic.
    /// \param uuid UUID used to route the incoming characteristic value.
    /// \param characteristic_path BlueZ characteristic path used to identify the notification source.
    void on_notification(const std::vector<uint8_t>& data,
                         const std::string& uuid,
                         const std::string& characteristic_path);
    /// \brief Accept BlueZ resolution or an already populated remote GATT cache.
    /// \param device Peer whose BlueZ discovery state is combined with the local readiness cache.
    /// \return True when effective services resolved; otherwise false.
    bool effective_services_resolved(const bluez::DeviceInfo& device) const;
    /// \brief Read the normalized address of the selected adapter.
    /// \return Normalized address of the selected Bluetooth adapter.
    std::string adapter_local_mac();
    /// \brief Resolve the selected row by its stable peer address.
    /// \return Pointer to the selected cached device, or nullptr.
    const bluez::DeviceInfo* selected_device() const;
    /// \brief Combine visible UAV and non-UAV rows in display order.
    /// \return Visible UAV and non-UAV devices in display order.
    std::vector<const bluez::DeviceInfo*> visible_devices() const;
    /// \brief Filter visible rows recognized by the UAV hostname policy.
    /// \return Visible devices recognized by the UAV hostname policy.
    std::vector<const bluez::DeviceInfo*> visible_uav_devices() const;
    /// \brief Filter visible rows not recognized as UAV peers.
    /// \return Visible devices not recognized as UAV peers.
    std::vector<const bluez::DeviceInfo*> visible_other_devices() const;
    /// \brief Render current peers actions Mesh state and diagnostics into one terminal frame.
    /// \param devices Current peer snapshots sorted and rendered in the dashboard.
    void render_dashboard(std::vector<bluez::DeviceInfo> devices);

    std::unique_ptr<CentralClientRuntime> runtime_;
    std::unique_ptr<gatt::RemoteGattInspector> inspector_;
    std::unique_ptr<serial::SerialLinkManager> serial_links_;
    std::unique_ptr<bluez::SerialPortProfile> serial_profile_;
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
    std::string serial_device_directory_;
    std::string serial_fallback_directory_;
    std::string ssh_user_;
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
    uint16_t serial_port_channel_{22};
    bool enable_serial_port_profile_{true};
    bool hide_non_uav_{false};
    PromptMode prompt_mode_{PromptMode::None};
    std::string prompt_buffer_;
    mutable std::mutex pending_notification_mutex_;
    std::vector<PendingNotification> pending_notifications_;
};

}  // namespace mrs_uav_bluetooth::app
