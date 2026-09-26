// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/msg/ble_device_array.hpp"
#include "mrs_uav_bluetooth/msg/mesh_event.hpp"
#include "mrs_uav_bluetooth/msg/mesh_message.hpp"
#include "mrs_uav_bluetooth/msg/mesh_status.hpp"
#include "mrs_uav_bluetooth/srv/set_active_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace mrs_uav_bluetooth::app {

class UserOverlayNode : public rclcpp::Node {
public:
    UserOverlayNode();
    ~UserOverlayNode() override;

private:
    void configure_parameters();
    /// Load the merged default/overlay config and select diagnostics from the
    /// transports actually declared by the overlay.
    void load_overlay_context();
    /// Create subscriptions and a periodic reporter for advertisement/Mesh
    /// details. GATT keeps using the service's comprehensive text report.
    void configure_transport_reporting();
    void activate_overlay();
    void revert_overlay();
    void publish_keepalive();
    mrs_uav_bluetooth::srv::SetActiveConfig::Response::SharedPtr call_config_service(const std::string& config_path);
    void handle_print(const std_msgs::msg::String::SharedPtr message);
    /// Cache the latest nearby advertisement snapshot for the next report.
    void handle_advertisements(
        const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message);
    /// Cache current Mesh lifecycle and topology state.
    void handle_mesh_status(
        const mrs_uav_bluetooth::msg::MeshStatus::SharedPtr message);
    /// Retain the newest Mesh lifecycle event and a cumulative event count.
    void handle_mesh_event(
        const mrs_uav_bluetooth::msg::MeshEvent::SharedPtr message);
    /// Retain the newest raw Mesh packet and a cumulative receive count.
    void handle_mesh_message(
        const mrs_uav_bluetooth::msg::MeshMessage::SharedPtr message);
    /// Print one transport-aware report from cached non-blocking callbacks.
    void print_transport_status();

    std::string config_path_;
    std::string keepalive_topic_;
    std::string default_config_path_;
    std::string hostname_;
    std::string set_active_config_service_;
    std::string print_source_;
    std::string print_topic_;
    std::string sentinel_topic_suffix_;
    config::NodeConfig effective_config_;
    bool print_text_status_{true};
    bool report_advertisements_{false};
    bool report_mesh_{false};
    double service_wait_timeout_sec_{30.0};
    double service_call_timeout_sec_{45.0};
    double deactivate_service_wait_timeout_sec_{2.0};
    double keepalive_publish_period_sec_{1.0};
    double min_keepalive_publish_period_sec_{0.2};
    double transport_report_period_sec_{2.0};
    std::mutex print_mutex_;
    std::string last_print_payload_;

    // Structured status callbacks only copy messages and timestamps. Formatting
    // is centralized in the timer callback so high-rate Mesh RX cannot flood a
    // user's terminal.
    std::mutex transport_status_mutex_;
    std::optional<mrs_uav_bluetooth::msg::BleDeviceArray> latest_advertisements_;
    std::optional<mrs_uav_bluetooth::msg::MeshStatus> latest_mesh_status_;
    std::optional<mrs_uav_bluetooth::msg::MeshEvent> latest_mesh_event_;
    std::optional<mrs_uav_bluetooth::msg::MeshMessage> latest_mesh_message_;
    std::chrono::steady_clock::time_point advertisements_received_at_{};
    std::chrono::steady_clock::time_point mesh_status_received_at_{};
    std::chrono::steady_clock::time_point mesh_event_received_at_{};
    std::chrono::steady_clock::time_point mesh_message_received_at_{};
    uint64_t mesh_event_count_{0};
    uint64_t mesh_message_count_{0};

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr keepalive_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr print_sub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr
        advertisements_sub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::MeshStatus>::SharedPtr
        mesh_status_sub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::MeshEvent>::SharedPtr
        mesh_event_sub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::MeshMessage>::SharedPtr
        mesh_message_sub_;
    rclcpp::TimerBase::SharedPtr keepalive_timer_;
    rclcpp::TimerBase::SharedPtr transport_report_timer_;
    rclcpp::Client<mrs_uav_bluetooth::srv::SetActiveConfig>::SharedPtr set_active_config_client_;
};

}  // namespace mrs_uav_bluetooth::app
