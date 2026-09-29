// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bridge/import_bridge_manager.hpp
/// \brief Declares the import bridge manager component of the transport-independent ROS message bridge.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <vector>
#include <string>

namespace mrs_uav_bluetooth::bridge {

/// Discovers remote bridge metadata and publishes decoded peer messages to ROS.
class ImportBridgeManager {
public:
    /// \brief Create the owner of incoming bridge publishers and poll timers.
    /// \param node ROS node that owns the created interfaces.
    /// \param logger ROS logger used for diagnostics.
    explicit ImportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger);

    /// \brief Point this manager at the shared configured bridge-state registry.
    /// \param registry shared owner of import and export bridge runtime state.
    void set_registry(BridgeRegistry* registry);
    /// \brief Share the recursive lock that protects overlay and bridge reconfiguration.
    /// \param mutex shared recursive mutex that serializes overlay and bridge state.
    void set_state_mutex(std::recursive_mutex* mutex);
    /// \brief Cancel polling and release the publisher and codec owned by one import bridge.
    /// \param state Import bridge whose subscription timer and decoder are released.
    void destroy_import_bridge(TopicImportBridgeState& state);
    /// \brief Resolve a peer characteristic and create the matching ROS publisher.
    /// \param bridge_key Stable key used to locate this import bridge after asynchronous work.
    /// \param state Import bridge receiving its resolved characteristic and runtime decoder.
    /// \param client BlueZ client used for remote discovery and GATT operations.
    void configure_import_bridge(const std::string& bridge_key,
                                 TopicImportBridgeState& state,
                                 bluez::BluezClient& client);
    /// \brief Poll readable characteristics that cannot send notifications.
    /// \param bridge_key Stable key used to locate this import bridge after asynchronous work.
    /// \param state Import bridge receiving a timer for characteristics without notifications.
    /// \param client BlueZ client used for remote discovery and GATT operations.
    void configure_import_poll_timer(const std::string& bridge_key,
                                     TopicImportBridgeState& state,
                                     bluez::BluezClient& client);
    /// \brief Buffer a matching notification for deferred ROS publication.
    /// \param mac peer Bluetooth MAC address.
    /// \param characteristic_path BlueZ characteristic path used to select the import bridge.
    /// \param payload GATT notification bytes to queue until the bridge is ready.
    /// \return Whether the notification matched and was buffered for an import bridge.
    bool buffer_notification_payload(const std::string& mac,
                                     const std::string& characteristic_path,
                                     const std::vector<uint8_t>& payload);
    /// \brief Re-resolve all configured import characteristics after a peer GATT tree changes.
    /// \param mac peer Bluetooth MAC address.
    /// \param client BlueZ client used for remote discovery and GATT operations.
    /// \return Whether any remote path or notification subscription changed.
    bool refresh_import_paths_for_mac(const std::string& mac,
                                      bluez::BluezClient& client);
    /// \brief Invalidate characteristic paths and queued values for a disconnected peer.
    /// \param mac peer Bluetooth MAC address.
    /// \param client BlueZ client used for remote discovery and GATT operations.
    /// \return Whether any paths or subscriptions were cleared.
    bool clear_import_paths_for_mac(const std::string& mac,
                                    bluez::BluezClient& client);

private:
    /// \brief Create or reuse the generic ROS codec for an import bridge.
    /// \param state Bridge state whose lazily created message codec is requested.
    /// \return Reusable type-erased ROS codec for the selected bridge.
    std::shared_ptr<GenericMessageBridge> runtime_for(TopicImportBridgeState& state);
    /// \brief Decode one complete transport frame and publish the resulting ROS message.
    /// \param state Import bridge supplying the decoder topic and publisher.
    /// \param payload Complete transport frame to decode and publish.
    /// \return Whether decoding and ROS publication succeeded.
    bool publish_payload(TopicImportBridgeState& state, const std::vector<uint8_t>& payload);

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    BridgeRegistry* registry_{nullptr};
    std::recursive_mutex* state_mutex_{nullptr};
};

}  // namespace mrs_uav_bluetooth::bridge
