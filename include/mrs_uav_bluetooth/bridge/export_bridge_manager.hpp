// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bridge/export_bridge_manager.hpp
/// \brief Declares the export bridge manager component of the transport-independent ROS message bridge.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

namespace mrs_uav_bluetooth::bridge {

/// Creates local ROS subscriptions and exposes their encoded values through GATT.
class ExportBridgeManager {
public:
    /// \brief Create the owner of outgoing ROS subscriptions and GATT publishers.
    /// \param node ROS node that owns the created interfaces.
    /// \param logger ROS logger used for diagnostics.
    explicit ExportBridgeManager(rclcpp::Node& node, rclcpp::Logger logger);

    /// \brief Point this manager at the shared configured bridge-state registry.
    /// \param registry shared owner of import and export bridge runtime state.
    void set_registry(BridgeRegistry* registry);
    /// \brief Share the recursive lock that protects overlay and bridge reconfiguration.
    /// \param mutex shared recursive mutex that serializes overlay and bridge state.
    void set_state_mutex(std::recursive_mutex* mutex);
    /// \brief Create the ROS subscription and encoder for one outgoing bridge.
    /// \param bridge_key Stable key used to locate this export bridge after asynchronous work.
    /// \param state Export bridge receiving its subscription encoder and transport publisher.
    void configure_export_bridge(const std::string& bridge_key,
                                 TopicExportBridgeState& state);
    /// \brief Recreate exported topic services so D-Bus metadata matches the active configuration.
    /// \param app GATT application that receives rebuilt export services.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param app_base_path path of the app base.
    void rebuild_gatt_services(gatt::GattApplication& app,
                               bluez::DbusConnection& dbus,
                               const std::string& app_base_path);
    /// \brief Route one encoded outgoing frame to Advertisement GATT or Mesh.
    /// \param bridge_key Stable key of the export bridge that produced the payload.
    /// \param payload Encoded bridge frame ready for the selected transport.
    void publish_export_payload(const std::string& bridge_key,
                                const std::vector<uint8_t>& payload);
    /// \brief Release one outgoing bridge's subscription timer codec and GATT service.
    /// \param state Export bridge whose subscription timer and codec are released.
    void destroy_export_bridge(TopicExportBridgeState& state);
    /// \brief Publish only the newest queued ROS sample at the configured maximum rate.
    /// \param bridge_key Stable key used to locate this export bridge after asynchronous work.
    /// \param state Export bridge receiving its configured rate-limit timer.
    void configure_export_rate_timer(const std::string& bridge_key,
                                     TopicExportBridgeState& state);

private:
    /// \brief Create or reuse the generic ROS codec for an export bridge.
    /// \param state Bridge state whose lazily created message codec is requested.
    /// \return Reusable type-erased ROS codec for the selected bridge.
    std::shared_ptr<GenericMessageBridge> runtime_for(TopicExportBridgeState& state);

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    BridgeRegistry* registry_{nullptr};
    std::recursive_mutex* state_mutex_{nullptr};
};

}  // namespace mrs_uav_bluetooth::bridge
