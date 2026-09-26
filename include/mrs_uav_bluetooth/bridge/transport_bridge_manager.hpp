// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/config/config_models.hpp"
#include "mrs_uav_bluetooth/mesh/mesh_application.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bridge {

/// Runs declarative topic bridges over connectionless Bluetooth transports.
///
/// GATT bridges retain their descriptor-discovery managers. This class handles
/// only `advertisement` and `mesh` entries, but deliberately uses the same
/// GenericMessageBridge codec and SharedTopicConfig definition. Consequently a
/// user can change transport without writing a transport-specific ROS node.
class TransportBridgeManager {
public:
    /// Receives a complete framed payload for the advertisement Data field.
    using AdvertisementSender =
        std::function<void(const std::vector<uint8_t>& payload)>;

    /// Receives Mesh routing metadata and a complete access-message payload.
    using MeshSender = std::function<void(
        const config::SharedTopicConfig& bridge,
        const std::vector<uint8_t>& access_payload)>;

    TransportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger);
    ~TransportBridgeManager();

    TransportBridgeManager(const TransportBridgeManager&) = delete;
    TransportBridgeManager& operator=(const TransportBridgeManager&) = delete;

    /// Install transport callbacks owned by ServiceNode.
    void set_advertisement_sender(AdvertisementSender sender);
    void set_mesh_sender(MeshSender sender);

    /// Atomically replace all configured advertisement and Mesh bridges.
    /// GATT entries are ignored because they are managed by the existing GATT
    /// export/import managers. At most one advertisement exporter is accepted:
    /// a controller exposes one dynamic user-data field at a time.
    void configure(const std::vector<config::SharedTopicConfig>& topics,
                   const std::string& node_topics_prefix,
                   const std::vector<std::string>& peer_whitelist);

    /// Release subscriptions, timers, publishers, and buffered payloads.
    void clear();

    /// Decode a nearby device's dynamic advertisement user-data field.
    /// Use the resolved hostname for the peer topic. When BlueZ has not
    /// resolved one, use its MAC address as the stable fallback. Repeated
    /// BlueZ snapshots are deduplicated per peer and channel.
    bool handle_advertisement(const std::string& hostname,
                              const std::string& mac,
                              const std::vector<uint8_t>& payload);

    /// Decode a received Mesh application message.
    /// Both BlueZ variants (opcode retained or stripped) are accepted. Mesh
    /// access messages contain a unicast address, not a Bluetooth MAC, so
    /// an unresolved Mesh source can only have a unicast-based fallback.
    bool handle_mesh(uint16_t source,
                     uint16_t key_index,
                     const std::vector<uint8_t>& data);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mrs_uav_bluetooth::bridge
