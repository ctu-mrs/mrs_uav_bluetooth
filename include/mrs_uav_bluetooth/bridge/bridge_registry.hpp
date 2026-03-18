// SPDX-License-Identifier: BSD-3-Clause
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

struct TopicExportBridgeState {
    std::string topic_name;
    std::string message_type;
    std::string bridge_name;
    std::string bridge_key;
    std::string bridge_uuid;
    std::vector<config::BridgeMemberSpec> member_specs;
    double rate_hz{0.0};
    std::string transport_endpoint{"characteristic"};
    std::string payload_format{"struct"};
    bool auto_managed{false};
    rclcpp::SubscriptionBase::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::vector<uint8_t> pending_payload;
    std::shared_ptr<GenericMessageBridge> runtime;
    std::shared_ptr<gatt::services::TopicBridgeService> service;
};

struct TopicImportBridgeState {
    std::string mac;
    std::string requested_topic_name;
    std::string resolved_topic_name;
    std::string message_type;
    std::string bridge_name;
    std::string bridge_key;
    std::string bridge_uuid;
    std::vector<config::BridgeMemberSpec> member_specs;
    double rate_hz{0.0};
    std::string transport_endpoint{"characteristic"};
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

class BridgeRegistry {
public:
    std::map<std::string, TopicExportBridgeState>& exports() { return exports_; }
    const std::map<std::string, TopicExportBridgeState>& exports() const { return exports_; }

    std::map<std::string, TopicImportBridgeState>& imports() { return imports_; }
    const std::map<std::string, TopicImportBridgeState>& imports() const { return imports_; }

private:
    std::map<std::string, TopicExportBridgeState> exports_;
    std::map<std::string, TopicImportBridgeState> imports_;
};

}  // namespace mrs_uav_bluetooth::bridge
