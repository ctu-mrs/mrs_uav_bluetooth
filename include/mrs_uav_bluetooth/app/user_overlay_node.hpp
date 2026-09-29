// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/app/user_overlay_node.hpp
/// \brief Declares the user overlay node component of the ROS 2 application and operator-tool layer.

#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/msg/ble_device_array.hpp"
#include "mrs_uav_bluetooth/msg/ble_peer_time_status.hpp"
#include "mrs_uav_bluetooth/msg/mesh_event.hpp"
#include "mrs_uav_bluetooth/msg/mesh_message.hpp"
#include "mrs_uav_bluetooth/msg/mesh_status.hpp"
#include "mrs_uav_bluetooth/srv/set_active_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/empty.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace mrs_uav_bluetooth::app {

/// Experiment-side node that leases a temporary service-node configuration.
/// Destruction or loss of its keepalive publisher makes the service revert safely.
class UserOverlayNode : public rclcpp::Node {
public:
    /// \brief Load, activate, and keep alive one example overlay.
    UserOverlayNode();
    /// \brief Release the overlay lease and request restoration of the default configuration.
    ~UserOverlayNode() override;

private:
    /// \brief Load overlay path, service names, lease timing, and transport-report topics.
    void configure_parameters();
    /// Load the merged default/overlay config and select diagnostics from the
    /// transports actually declared by the overlay.
    void load_overlay_context();
    /// Observe local sources, imported topics, and transport status for every overlay.
    void configure_transport_reporting();
    /// \brief Acquire the overlay lease and ask the service node to activate its YAML file.
    void activate_overlay();
    /// \brief Release this node's overlay lease and request the default configuration.
    void revert_overlay();
    /// \brief Renew the overlay lease while this client remains alive.
    void publish_keepalive();
    /// \brief Wait briefly for configuration control and return its completed response.
    /// \param config_path path of the config.
    /// \return Completed configuration-service response or null when unavailable.
    mrs_uav_bluetooth::srv::SetActiveConfig::Response::SharedPtr call_config_service(const std::string& config_path);
    /// Keep recently seen advertisement peers across empty scan snapshots.
    /// \param message Latest advertisement scan snapshot.
    void handle_advertisements(
        const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message);
    /// Cache the complete BlueZ device snapshot for GATT and advertisement discovery.
    /// \param message Current BlueZ device records, including disconnected peers.
    void handle_devices(const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message);
    /// Cache current Mesh lifecycle and topology state.
    /// \param message Latest Mesh lifecycle and topology snapshot.
    void handle_mesh_status(
        const mrs_uav_bluetooth::msg::MeshStatus::SharedPtr message);
    /// Retain the newest Mesh lifecycle event and a cumulative event count.
    /// \param message Latest Mesh lifecycle event.
    void handle_mesh_event(
        const mrs_uav_bluetooth::msg::MeshEvent::SharedPtr message);
    /// Retain the newest raw Mesh packet and a cumulative receive count.
    /// \param message Latest raw Mesh message.
    void handle_mesh_message(
        const mrs_uav_bluetooth::msg::MeshMessage::SharedPtr message);
    /// Discover new peer publishers and attach rate observers to their import topics.
    void refresh_import_observers();
    /// Record one local ROS delivery for a configured source or imported topic.
    /// \param topic Exact ROS topic on which the sample arrived.
    void record_topic_sample(const std::string& topic);
    /// Print one transport-aware report from cached non-blocking callbacks.
    void print_transport_status();

    std::string config_path_;
    std::string keepalive_topic_;
    std::string default_config_path_;
    std::string hostname_;
    std::string set_active_config_service_;
    std::string sentinel_topic_suffix_;
    config::NodeConfig effective_config_;
    bool report_gatt_{false};
    bool report_advertisements_{false};
    bool report_mesh_{false};
    double service_wait_timeout_sec_{30.0};
    double service_call_timeout_sec_{45.0};
    double deactivate_service_wait_timeout_sec_{2.0};
    double keepalive_publish_period_sec_{1.0};
    double min_keepalive_publish_period_sec_{0.2};
    double transport_report_period_sec_{2.0};
    /// Rolling deliveries support comparable observed Hz and silence ages in every mode.
    struct TopicObservation {
        std::string type;
        rclcpp::SubscriptionBase::SharedPtr subscription;
        std::deque<std::chrono::steady_clock::time_point> samples;
        std::chrono::steady_clock::time_point started_at{};
        std::chrono::steady_clock::time_point last_at{};
        uint64_t total{0};
    };

    // Status callbacks cache data quickly; the periodic timer formats one coherent view.
    std::mutex transport_status_mutex_;
    std::map<std::string, TopicObservation> observed_topics_;
    std::map<std::string, double> peer_rtt_seconds_;
    std::optional<mrs_uav_bluetooth::msg::BleDeviceArray> latest_devices_;
    std::chrono::steady_clock::time_point devices_received_at_{};
    std::map<std::string, std::pair<
        mrs_uav_bluetooth::msg::BleDevice,
        std::chrono::steady_clock::time_point>> recent_advertisements_;
    std::optional<mrs_uav_bluetooth::msg::MeshStatus> latest_mesh_status_;
    std::optional<mrs_uav_bluetooth::msg::MeshEvent> latest_mesh_event_;
    std::map<std::string, std::pair<mrs_uav_bluetooth::msg::MeshEvent,
        std::chrono::steady_clock::time_point>> recent_mesh_scans_;
    std::optional<mrs_uav_bluetooth::msg::MeshMessage> latest_mesh_message_;
    std::chrono::steady_clock::time_point advertisements_received_at_{};
    std::chrono::steady_clock::time_point mesh_status_received_at_{};
    std::chrono::steady_clock::time_point mesh_event_received_at_{};
    std::chrono::steady_clock::time_point mesh_message_received_at_{};
    uint64_t mesh_event_count_{0};
    uint64_t mesh_message_count_{0};

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr keepalive_pub_;
    rclcpp::Subscription<mrs_uav_bluetooth::msg::BleDeviceArray>::SharedPtr devices_sub_;
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
