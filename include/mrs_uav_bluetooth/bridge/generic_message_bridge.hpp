// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialized_message.hpp>

#include <memory>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bridge {

class GenericMessageBridge {
public:
    explicit GenericMessageBridge(std::string message_type);

    const std::string& message_type() const;

    std::shared_ptr<rclcpp::GenericSubscription> create_subscription(
        rclcpp::Node& node,
        const std::string& topic_name,
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
        size_t queue_depth = 10) const;

    std::shared_ptr<rclcpp::GenericPublisher> create_publisher(
        rclcpp::Node& node,
        const std::string& topic_name,
        size_t queue_depth = 10) const;

    std::vector<uint8_t> encode_payload(
        const rclcpp::SerializedMessage& serialized_message,
        const std::vector<config::BridgeMemberSpec>& member_specs,
        const std::string& payload_format) const;

    rclcpp::SerializedMessage decode_payload(
        const std::vector<uint8_t>& payload,
        const std::vector<config::BridgeMemberSpec>& member_specs,
        const std::string& payload_format) const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace mrs_uav_bluetooth::bridge