// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bridge/generic_message_bridge.hpp
/// \brief Declares the generic message bridge component of the transport-independent ROS message bridge.

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

/// Introspection-based ROS bridge that encodes or decodes configured fields.
/// The same codec is used by GATT, advertisements, and Mesh.
class GenericMessageBridge {
public:
    /// \brief Load runtime type support for a configured ROS message.
    /// \param message_type Fully qualified ROS message type whose runtime support is loaded.
    explicit GenericMessageBridge(std::string message_type);

    /// \brief Return the fully qualified runtime ROS message type.
    /// \return Fully qualified runtime ROS message type.
    const std::string& message_type() const;

    /// \brief Create a type-erased ROS subscription using the runtime-loaded message support.
    /// \param node ROS node that owns the created interfaces.
    /// \param topic_name Resolved ROS topic from which serialized messages are received.
    /// \param callback Handler invoked with each serialized ROS message from the topic.
    /// \param queue_depth ROS QoS history depth for the created endpoint.
    /// \return Type-erased ROS subscription for this runtime message type.
    std::shared_ptr<rclcpp::GenericSubscription> create_subscription(
        rclcpp::Node& node,
        const std::string& topic_name,
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
        size_t queue_depth = 10) const;

    /// \brief Create a type-erased ROS publisher using the runtime-loaded message support.
    /// \param node ROS node that owns the created interfaces.
    /// \param topic_name Resolved ROS topic on which decoded messages are published.
    /// \param queue_depth ROS QoS history depth for the created endpoint.
    /// \return Type-erased ROS publisher for this runtime message type.
    std::shared_ptr<rclcpp::GenericPublisher> create_publisher(
        rclcpp::Node& node,
        const std::string& topic_name,
        size_t queue_depth = 10) const;

    /// \brief Serialize a ROS message as full CDR bytes or selected packed scalar fields.
    /// \param serialized_message ROS serialized buffer to encode or decode.
    /// \param member_specs Ordered ROS fields selected and encoded into the transport frame.
    /// \param payload_format Wire format selecting full ROS serialization or the configured packed members.
    /// \return Transport payload encoded from the serialized ROS message.
    std::vector<uint8_t> encode_payload(
        const rclcpp::SerializedMessage& serialized_message,
        const std::vector<config::BridgeMemberSpec>& member_specs,
        const std::string& payload_format) const;

    /// \brief Reconstruct a ROS message from CDR bytes or selected packed scalar fields.
    /// \param payload Transport frame to decode into a ROS message.
    /// \param member_specs Ordered ROS fields reconstructed from the transport frame.
    /// \param payload_format Wire format selecting full ROS serialization or the configured packed members.
    /// \param decode_assignments post-decode expressions that reconstruct ROS fields.
    /// \return Serialized ROS message reconstructed from transport bytes.
    rclcpp::SerializedMessage decode_payload(
        const std::vector<uint8_t>& payload,
        const std::vector<config::BridgeMemberSpec>& member_specs,
        const std::string& payload_format,
        const std::vector<config::BridgeAssignmentSpec>& decode_assignments = {}) const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace mrs_uav_bluetooth::bridge
