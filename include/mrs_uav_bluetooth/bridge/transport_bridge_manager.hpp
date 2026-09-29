// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bridge/transport_bridge_manager.hpp
/// \brief Declares the transport bridge manager component of the transport-independent ROS message bridge.

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
    /// Return true after accepting ownership. False keeps the latest value
    /// buffered for a later attempt, including for one-shot sources.
    using AdvertisementSender =
        std::function<bool(const std::vector<uint8_t>& payload)>;

    /// Receives Mesh routing metadata and a complete access-message payload.
    /// Uses the same ownership result as AdvertisementSender.
    using MeshSender = std::function<bool(
        const config::SharedTopicConfig& bridge,
        const std::vector<uint8_t>& access_payload)>;

    /// \brief Create shared bridge state for GATT, Advertisement, and Mesh transports.
    /// \param node ROS node that owns the created interfaces.
    /// \param logger ROS logger used for diagnostics.
    TransportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger);
    /// \brief Cancel bridge timers and ROS endpoints before transport teardown.
    ~TransportBridgeManager();

    /// \brief Disable copying of the transport bridge manager.
    TransportBridgeManager(const TransportBridgeManager&) = delete;
    /// \brief Disable copy assignment of the transport bridge manager.
    TransportBridgeManager& operator=(const TransportBridgeManager&) = delete;

    /// Install transport callbacks owned by ServiceNode.
    /// \param sender transport callback invoked for an outgoing bridge payload.
    void set_advertisement_sender(AdvertisementSender sender);
    /// \brief Install the callback that submits encoded topic frames to Mesh.
    /// \param sender transport callback invoked for an outgoing bridge payload.
    void set_mesh_sender(MeshSender sender);

    /// Atomically replace all configured advertisement and Mesh bridges.
    /// GATT entries are ignored because they are managed by the existing GATT
    /// export/import managers. At most one advertisement exporter is accepted:
    /// a controller exposes one dynamic user-data field at a time.
    /// \param topics Topic bridge definitions used to create import and export runtime state.
    /// \param node_topics_prefix normalized ROS namespace below which endpoints are created.
    /// \param peer_whitelist admitted peer names or addresses used for bridge routing.
    void configure(const std::vector<config::SharedTopicConfig>& topics,
                   const std::string& node_topics_prefix,
                   const std::vector<std::string>& peer_whitelist);

    /// Release subscriptions, timers, publishers, and buffered payloads.
    void clear();

    /// Decode a nearby device's dynamic advertisement user-data field.
    /// Use the resolved hostname for the peer topic. When BlueZ has not
    /// resolved one, use its MAC address as the stable fallback. Repeated
    /// BlueZ snapshots are deduplicated per peer and channel.
    /// \param hostname UAV hostname used to identify the node.
    /// \param mac peer Bluetooth MAC address.
    /// \param payload Advertisement service-data frame received from the named peer.
    /// \return Whether an advertisement payload matched and updated an import bridge.
    bool handle_advertisement(const std::string& hostname,
                              const std::string& mac,
                              const std::vector<uint8_t>& payload);

    /// Decode a received Mesh application message.
    /// Both BlueZ variants (opcode retained or stripped) are accepted. Mesh
    /// access messages contain a unicast address, not a Bluetooth MAC, so
    /// an unresolved Mesh source can only have a unicast-based fallback.
    /// \param source Mesh unicast address that sent the bridge frame.
    /// \param key_index Application key index on which the bridge frame arrived.
    /// \param data Mesh access payload received from the source address.
    /// \return Whether a Mesh payload matched and updated an import bridge.
    bool handle_mesh(uint16_t source,
                     uint16_t key_index,
                     const std::vector<uint8_t>& data);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mrs_uav_bluetooth::bridge
