// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bridge/bridge_registry.hpp
/// \brief Declares the bridge registry component of the transport-independent ROS message bridge.

#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/gatt/services/topic_bridge_service.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bridge {

class GenericMessageBridge;

/// Runtime state for one locally exported ROS topic and its optional GATT service.
struct TopicExportBridgeState {
    std::string topic_name;
    std::string message_type;
    std::string bridge_name;
    std::string bridge_key;
    std::string bridge_uuid;
    std::vector<config::BridgeMemberSpec> member_specs;
    double rate_hz{0.0};
    std::string payload_format{"struct"};
    bool auto_managed{false};
    rclcpp::SubscriptionBase::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::vector<uint8_t> pending_payload;
    std::shared_ptr<GenericMessageBridge> runtime;
    std::shared_ptr<gatt::services::TopicBridgeService> service;
};

/// Runtime state for one peer topic imported into the local ROS graph.
struct TopicImportBridgeState {
    std::string mac;
    std::string requested_topic_name;
    std::string resolved_topic_name;
    std::string message_type;
    std::string bridge_name;
    std::string bridge_key;
    std::string bridge_uuid;
    std::vector<config::BridgeMemberSpec> member_specs;
    std::vector<config::BridgeAssignmentSpec> decode_assignments;
    double rate_hz{0.0};
    std::string payload_format{"struct"};
    std::string path;
    bool auto_managed{false};
    rclcpp::PublisherBase::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr poll_timer;
    std::vector<uint8_t> pending_payload;
    std::vector<uint8_t> last_payload;
    double last_publish_monotonic{0.0};
    double current_hz{0.0};
    std::shared_ptr<GenericMessageBridge> runtime;
};

/// Thread-safe registry shared by discovery, GATT, and transport managers.
/// Registry entries own bridge runtimes so callbacks cannot outlive their codec.
class BridgeRegistry {
public:
    /// \brief Return the configured outgoing bridges.
    /// \return Configured outgoing bridge map.
    std::map<std::string, TopicExportBridgeState>& exports() {
        // Expose mutable outgoing bridge state to configuration and publishers.
        return exports_;
    }
    /// \brief Return the configured outgoing bridges.
    /// \return Configured outgoing bridge map.
    const std::map<std::string, TopicExportBridgeState>& exports() const {
        // Expose outgoing bridge state read-only for status and lookup.
        return exports_;
    }

    /// \brief Return the configured incoming bridges.
    /// \return Configured incoming bridge map.
    std::map<std::string, TopicImportBridgeState>& imports() {
        // Expose mutable incoming bridge state to path and notification reconciliation.
        return imports_;
    }
    /// \brief Return the configured incoming bridges.
    /// \return Configured incoming bridge map.
    const std::map<std::string, TopicImportBridgeState>& imports() const {
        // Expose incoming bridge state read-only for status and lookup.
        return imports_;
    }

private:
    std::map<std::string, TopicExportBridgeState> exports_;
    std::map<std::string, TopicImportBridgeState> imports_;
};

}  // namespace mrs_uav_bluetooth::bridge
