// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/app/service_node.hpp
/// \brief Declares the service node component of the ROS 2 application and operator-tool layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/adapter_controller.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_pairing_agent.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"
#include "mrs_uav_bluetooth/bluez/serial_port_profile.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"
#include "mrs_uav_bluetooth/msg/ble_peer_time_status.hpp"
#include "mrs_uav_bluetooth/bridge/export_bridge_manager.hpp"
#include "mrs_uav_bluetooth/bridge/import_bridge_manager.hpp"
#include "mrs_uav_bluetooth/bridge/transport_bridge_manager.hpp"
#include "mrs_uav_bluetooth/config/overlay_config_manager.hpp"
#include "mrs_uav_bluetooth/gatt/advertisement.hpp"
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"
#include "mrs_uav_bluetooth/gatt/services/time_service.hpp"
#include "mrs_uav_bluetooth/gatt/services/wifi_service.hpp"
#include "mrs_uav_bluetooth/network/netplan_manager.hpp"
#include "mrs_uav_bluetooth/mesh/mesh_application.hpp"
#include "mrs_uav_bluetooth/mesh/swarm_coordinator.hpp"
#include "mrs_uav_bluetooth/peer/peer_manager.hpp"
#include "mrs_uav_bluetooth/ros/ros_interface_manager.hpp"
#include "mrs_uav_bluetooth/serial/serial_link.hpp"
#include "mrs_uav_bluetooth/srv/configure_notification_bridge.hpp"
#include "mrs_uav_bluetooth/srv/connect_device.hpp"
#include "mrs_uav_bluetooth/srv/disconnect_device.hpp"
#include "mrs_uav_bluetooth/srv/find_gatt_path.hpp"
#include "mrs_uav_bluetooth/srv/get_device.hpp"
#include "mrs_uav_bluetooth/srv/list_devices.hpp"
#include "mrs_uav_bluetooth/srv/list_gatt_characteristics.hpp"
#include "mrs_uav_bluetooth/srv/list_gatt_descriptors.hpp"
#include "mrs_uav_bluetooth/srv/list_gatt_services.hpp"
#include "mrs_uav_bluetooth/srv/pair_device.hpp"
#include "mrs_uav_bluetooth/srv/read_gatt_value.hpp"
#include "mrs_uav_bluetooth/srv/remove_device.hpp"
#include "mrs_uav_bluetooth/srv/set_active_config.hpp"
#include "mrs_uav_bluetooth/srv/set_device_trust.hpp"
#include "mrs_uav_bluetooth/srv/set_notify.hpp"
#include "mrs_uav_bluetooth/srv/set_scan_enabled.hpp"
#include "mrs_uav_bluetooth/srv/write_gatt_value.hpp"
#include "mrs_uav_bluetooth/srv/mesh_network.hpp"
#include "mrs_uav_bluetooth/srv/mesh_send.hpp"
#include "mrs_uav_bluetooth/srv/mesh_management.hpp"
#include "mrs_uav_bluetooth/srv/mesh_swarm.hpp"
#include "mrs_uav_bluetooth/msg/mesh_message.hpp"
#include "mrs_uav_bluetooth/msg/mesh_status.hpp"
#include "mrs_uav_bluetooth/msg/mesh_event.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <atomic>
#include <mutex>
#include <set>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace mrs_uav_bluetooth::app {

/// Long-running ROS 2 node that owns every Bluetooth transport.
/// Configuration transitions are serialized while D-Bus callbacks and ROS timers
/// remain active on separate executor threads.
class ServiceNode : public rclcpp::Node {
public:
    /// \brief Create the Bluetooth service node and build its configured runtime.
    ServiceNode();
    /// \brief Stop callbacks and tear down every transport before D-Bus shutdown.
    ~ServiceNode() override;

private:
    struct RepeatedLogEntry {
        std::chrono::steady_clock::time_point last_emit_time{};
        size_t suppressed_count{0};
    };
    struct MeshReliableTarget {
        unsigned attempts{0};
        std::optional<mesh::SendHandle> transfer;
        std::chrono::steady_clock::time_point retry_at{};
        // Ask for a fresh receipt after native unicast repair has finished.
        std::chrono::steady_clock::time_point receipt_query_at{};
        std::chrono::steady_clock::time_point transfer_check_at{};
    };

    struct MeshReliablePending {
        uint64_t channel_key{0};
        std::vector<uint8_t> data;
        std::map<uint16_t, MeshReliableTarget> awaiting;
        std::optional<mesh::SendHandle> primary_transfer;
        uint16_t app_key_index{0};
        uint8_t element_index{0};
        uint16_t turn_successor{0};
        bool force_segmented{true};
        // A single unicast target has lower-transport acknowledgement. Once
        // that transaction completes, its application receipt remains a
        // delivery check but no longer stalls newer channel values.
        bool native_completion_releases_channel{false};
        bool blocks_channel{true};
        std::chrono::steady_clock::time_point created{};
    };

    struct MeshReliableSeen {
        std::chrono::steady_clock::time_point last{};
        bool delivered{false};
    };

    struct MeshReliableAck {
        uint16_t destination{0};
        uint16_t app_key_index{0};
        std::vector<uint8_t> data;
    };

    struct MeshReliablePeerState {
        std::chrono::steady_clock::time_point transport_busy_until{};
        unsigned consecutive_expiries{0};
        std::chrono::steady_clock::time_point retry_after{};
    };

    struct MeshReliableTurn {
        std::vector<uint16_t> members;
        bool initialized{false};
        bool permitted{false};
        std::chrono::steady_clock::time_point reclaim_at{};
    };

    /// \brief Cancel retained Mesh sends and reset acknowledgement and retry bookkeeping.
    /// \param preserve_latest whether to keep the newest queued value.
    void clear_mesh_reliable_state(bool preserve_latest = false);
    /// \brief Expire acknowledgements retry due peers and cancel completed Mesh transfers.
    void maintain_mesh_reliable();
    /// \brief Retain a BlueZ transfer handle until its acknowledgement or retry finishes.
    /// \param sequence nonzero transfer sequence identifying one reliable Mesh message.
    /// \param destination Mesh destination retained for retries and acknowledgement tracking.
    /// \param handle Prepared Mesh send retained until acknowledgement or retry.
    void remember_mesh_transfer(uint16_t sequence, uint16_t destination,
                                const mesh::SendHandle& handle);
    /// \brief Cancel completed or abandoned transfer handles outside the reliability lock.
    void flush_mesh_transfer_cancellations();

    /// \brief Load service paths, timing limits, and the default configuration path.
    void configure_parameters();
    /// \brief Construct D-Bus BlueZ bridge peer and ROS runtime objects in dependency order.
    void build_runtime();
    /// \brief Register all ROS control services in the mutually exclusive service group.
    void create_services();
    /// \brief Transition all transports atomically to a validated configuration snapshot.
    /// \param cfg Validated node configuration replacing the active runtime.
    void apply_config(const config::NodeConfig& cfg);
    /// \brief Reconcile adapter power discovery discoverability and advertising policy.
    /// \param cfg Adapter scanning discoverability and advertisement settings to reconcile.
    void apply_adapter_state(const config::NodeConfig& cfg);
    /// \brief Apply deferred adapter policy after the current callback releases shared state.
    void reconcile_adapter_state_if_requested();
    /// \brief Rebuild D-Bus-backed runtime after the BlueZ owner changes.
    void recover_bluez_daemon_if_requested();
    /// \brief Register update or remove the RFCOMM profile and its link handlers.
    /// \param cfg Serial-profile enablement and service identity settings.
    void configure_serial_profile(const config::NodeConfig& cfg);
    /// \brief Create attach reconfigure or tear down the Mesh D-Bus application.
    /// \param cfg Mesh enablement identity keys addresses and model settings.
    void configure_mesh(const config::NodeConfig& cfg);
    /// \brief Advance Mesh attachment configuration coordination and reliable-transfer timers.
    void maintain_mesh();
    /// \brief Publish the current Mesh attachment addresses keys and coordinator state.
    void publish_mesh_status();
    /// \brief Convert one decoded access packet into the public ROS Mesh message.
    /// \param message Decoded Mesh packet to publish on ROS.
    void publish_mesh_message(const mesh::ReceivedMessage& message);
    /// \brief Convert one Mesh lifecycle event into the public ROS event message.
    /// \param event Mesh lifecycle event converted to a ROS message.
    void publish_mesh_event(const mesh::Event& event);
    /// \brief Recreate advertisement and local GATT objects for the active configuration.
    void rebuild_server_objects();
    /// \brief Publish aggregate health and coalesced per-peer diagnostics.
    void publish_periodic_status();
    /// \brief Publish the latest cache as a ROS device array.
    void publish_scan_snapshot();
    /// \brief Render adapter transport peer bridge and Mesh state for operators.
    /// \param devices_map address-indexed device snapshot used to derive aggregate status.
    /// \return Multiline operator report for the supplied device snapshot.
    std::string build_detailed_status_report(const std::map<std::string, bluez::DeviceInfo>& devices_map) const;
    /// \brief Reconcile transport and peer state after a BlueZ cache mutation.
    /// \param event Kind of BlueZ cache mutation being reconciled.
    /// \param object_path BlueZ object path affected by the cache event.
    void on_cache_event(bluez::CacheEvent event, const std::string& object_path);
    /// \brief Record local GATT activity or failure against the owning peer.
    /// \param event_type GATT operation name used to classify and report the event.
    /// \param object_path BlueZ GATT object whose event updates peer diagnostics.
    /// \param detail BlueZ result text classified and surfaced in peer diagnostics.
    void on_gatt_event(const std::string& event_type,
                       const std::string& object_path,
                       const std::string& detail);
    /// \brief Route a remote GATT value to time synchronization or topic imports.
    /// \param data Bytes received from the remote GATT characteristic.
    /// \param uuid UUID used to route the incoming characteristic value.
    /// \param characteristic_path BlueZ characteristic path used to identify the notification source.
    void on_notification(const std::vector<uint8_t>& data,
                         const std::string& uuid,
                         const std::string& characteristic_path);
    /// \brief Decode a peer timestamp and update the corresponding clock-offset bridge.
    /// \param payload Eight-byte peer timestamp written through GATT.
    /// \param device_path BlueZ device path identifying the peer that wrote the timestamp.
    /// \param received_time_ns local timestamp captured when the peer sample arrived.
    void handle_time_writeback(const std::vector<uint8_t>& payload,
                               const std::string& device_path,
                               uint64_t received_time_ns);
    /// \brief Register the current advertisement or update its exported properties safely.
    void refresh_advertisement_registration();
    /// \brief Pause broadcast transmission long enough for a half-duplex controller to scan.
    ///
    /// The active BlueZ discovery request resumes after the advertisement is
    /// unregistered. A small hostname-phased receive window prevents equal-rate
    /// peers from remaining permanently synchronized as transmitters or receivers.
    void open_advertisement_scan_window();
    /// \brief Create or remove the raw advertisement payload subscription for the active mode.
    void refresh_advertisement_topic_subscription();
    /// \brief Apply a raw ROS byte array as the next application advertisement value.
    /// \param message ROS byte-array containing the next raw advertisement payload.
    void handle_advertisement_payload(const std_msgs::msg::UInt8MultiArray::SharedPtr message);
    /// Apply an advertisement payload from either the raw ROS topic or a
    /// declarative bridge. Configured bridge frames are rejected rather than
    /// truncated because truncation would silently corrupt their codec.
    /// \param payload Application bytes to place in advertisement service data.
    /// \param allow_truncate whether an oversized raw payload may be shortened.
    void set_advertisement_payload(std::vector<uint8_t> payload,
                                   bool allow_truncate);
    /// Forward one already-framed access message through the attached Mesh.
    /// \param bridge Topic bridge configuration supplying the Mesh destination key and reliability policy.
    /// \param payload Already encoded bridge frame to send through Mesh.
    void send_mesh_bridge_payload(const config::SharedTopicConfig& bridge,
                                  const std::vector<uint8_t>& payload);
    /// \brief Submit one reliable Mesh bridge frame or queue it behind the active channel transfer.
    /// \param bridge Topic bridge configuration supplying the Mesh destination key and reliability policy.
    /// \param payload Already encoded bridge frame to send through Mesh.
    /// \param resumed whether this payload resumes a channel after a previous transfer.
    void send_mesh_bridge_payload_impl(const config::SharedTopicConfig& bridge,
                                       const std::vector<uint8_t>& payload,
                                       bool resumed);
    /// \brief Track whether exactly one peer subscribes to the local time characteristic.
    /// \param enabled Whether a remote peer currently subscribes to the local time value.
    void note_local_time_notify_state(bool enabled);
    /// \brief Apply agent events to admission policy, peer state, and reconciliation.
    /// \param event_type Pairing-agent request name used for policy and session updates.
    /// \param device_path BlueZ device path associated with the agent event.
    void on_pairing_event(const std::string& event_type, const std::string& device_path);
    /// \brief Complete one peer’s pairing state with its success or failure detail.
    /// \param mac peer Bluetooth MAC address.
    /// \param success Whether the peer pairing attempt completed successfully.
    /// \param error_detail optional output receiving the concrete BlueZ failure text.
    void note_pair_attempt_result(const std::string& mac,
                                  bool success,
                                  const std::string& error_detail);
    /// \brief Decide whether the active policy admits one incoming pairing request.
    /// \param event_type Pairing-agent request name used for policy and session updates.
    /// \param device_path BlueZ device path associated with the agent event.
    /// \return True when the caller should allow pairing request; otherwise false.
    bool should_allow_pairing_request(const std::string& event_type,
                                      const std::string& device_path);
    /// \brief Advance every discovered peer through policy pairing trust connection and bridge setup.
    void reconcile_peers();
    /// \brief Coalesce peer work onto its callback group after the requested delay.
    /// \param delay Minimum coalescing delay before peer reconciliation runs.
    void schedule_peer_reconcile(std::chrono::milliseconds delay = std::chrono::milliseconds(0));
    /// \brief Test whether a peer has completed the bidirectional time handshake.
    /// \param mac peer Bluetooth MAC address.
    /// \return True when the peer’s retained time bridge is ready.
    bool has_ready_peer_time_bridge(const std::string& mac) const;
    /// \brief Read the latest valid time-bridge activity used for peer leases.
    /// \param mac peer Bluetooth MAC address.
    /// \param max_inactivity_s seconds without traffic before a healthy time bridge is considered stale.
    /// \return Newest valid time sample or GATT activity timestamp or zero when unavailable.
    double healthy_peer_time_bridge_last_activity_monotonic(const std::string& mac,
                                                            double max_inactivity_s) const;
    /// \brief Decide whether a healthy peer runtime may survive transient GATT rediscovery.
    /// \param device Peer whose transient GATT rediscovery state is being classified.
    /// \return True only inside the bounded expected rediscovery window.
    bool should_preserve_peer_bridge_runtime_during_expected_services_rediscovery(
        const bluez::DeviceInfo& device) const;
    /// \brief Decide whether a ready time bridge may survive transient GATT rediscovery.
    /// \param device Peer whose ready bridge may survive a planned GATT refresh.
    /// \return True when the ready bridge remains healthy during expected rediscovery.
    bool should_preserve_ready_bridge_during_expected_services_rediscovery(
        const bluez::DeviceInfo& device) const;
    /// \brief Require a resolved GATT tree with the bridge service before starting imports.
    /// \param device Peer checked for the connection and GATT state needed by a bridge.
    /// \return True when device can host peer bridge; otherwise false.
    bool device_can_host_peer_bridge(const bluez::DeviceInfo& device) const;
    /// \brief Check whether the remote GATT tree satisfies all configured imports.
    /// \param device Peer checked for the connection and discovered services needed by imports.
    /// \return True when device can host peer import bridges; otherwise false.
    bool device_can_host_peer_import_bridges(const bluez::DeviceInfo& device) const;
    /// \brief Cancel per-peer work and remove time import and status state for one address.
    /// \param mac peer Bluetooth MAC address.
    /// \param skip_characteristic_path path of the skip characteristic.
    void clear_peer_runtime(const std::string& mac,
                            const std::string& skip_characteristic_path = {});
    /// \brief Re-resolve configured imports against one peer's current GATT tree.
    /// \param device Peer whose discovered characteristics are reconciled with import bridges.
    void refresh_import_bridges_for_device(const bluez::DeviceInfo& device);
    /// \brief Remove import runtime for peers no longer present in the BlueZ cache.
    /// \param mac peer Bluetooth MAC address.
    /// \param desired_keys key indexes that should remain after Mesh reconciliation.
    /// \param now_mono current steady-clock time in seconds for retry and expiry decisions.
    /// \param missing_path_grace_s seconds a peer may lack a transient BlueZ path before cleanup.
    void prune_missing_import_bridges(const std::string& mac,
                                      const std::set<std::string>& desired_keys,
                                      double now_mono,
                                      double missing_path_grace_s);
    /// \brief Build the normalized status topic for one peer.
    /// \param mac peer Bluetooth MAC address.
    /// \param peer_name Resolved peer hostname preferred in the status topic suffix.
    /// \return Normalized ROS topic carrying status for this peer.
    std::string peer_status_topic(const std::string& mac, const std::string& peer_name) const;
    /// \brief Build the normalized imported bridge topic for one peer.
    /// \param mac peer Bluetooth MAC address.
    /// \param peer_name Resolved peer hostname preferred in the bridge topic prefix.
    /// \param requested_topic_suffix caller-selected suffix used to name the peer bridge topic.
    /// \return Normalized ROS topic carrying this peer bridge.
    std::string peer_bridge_topic(const std::string& mac,
                                  const std::string& peer_name,
                                  const std::string& requested_topic_suffix) const;
    /// \brief Create or refresh the peer clock bridge once its time characteristic is usable.
    /// \param mac peer Bluetooth MAC address.
    /// \param device Peer supplying the time characteristic and current connection state.
    /// \param session Peer session storing time-characteristic readiness and sampling state.
    /// \return True if the peer time characteristic is usable and its bridge is ready; otherwise false.
    bool update_peer_time_bridge(const std::string& mac,
                                 const bluez::DeviceInfo& device,
                                 peer::PeerConnectionSession& session);
    /// \brief Publish the newest clock offset latency and sample age for one peer.
    /// \param bridge Per-peer time bridge supplying the latest offset sample.
    void publish_peer_time_status(peer::PeerTimeBridge& bridge) const;
    /// \brief Claim one named peer task so duplicate callbacks cannot run it concurrently.
    /// \param mac peer Bluetooth MAC address.
    /// \param label Operation name used for per-peer exclusion and diagnostics.
    /// \param task asynchronous peer operation whose completion is tracked.
    /// \return True if this call claimed and ran the named task; otherwise false.
    bool run_peer_task_once(const std::string& mac,
                            const std::string& label,
                            std::function<void()> task);
    /// \brief Join every outstanding peer worker before shutdown or reconfiguration.
    void wait_for_peer_tasks();
    /// \brief Emit an informational message only when its coalescing window elapsed.
    /// \param key Stable coalescing key that owns an independent quiet window.
    /// \param message Operator-facing text emitted when the key is outside its quiet window.
    void log_info_coalesced(const std::string& key, const std::string& message);
    /// \brief Emit a warning only when its coalescing window elapsed.
    /// \param key Stable coalescing key that owns an independent quiet window.
    /// \param message Operator-facing text emitted when the key is outside its quiet window.
    void log_warn_coalesced(const std::string& key, const std::string& message);
    /// \brief Report whether ROS callbacks may still mutate runtime state.
    /// \return True while neither shutdown nor ROS context termination has begun.
    bool can_run_callbacks() const;

    /// \brief Copy a cached BlueZ device record into the public ROS message shape.
    /// \param device Cached BlueZ device converted to the public ROS representation.
    /// \return Converted device msg.
    mrs_uav_bluetooth::msg::BleDevice to_device_msg(const bluez::DeviceInfo& device) const;
    /// \brief Convert a cached service into its ROS response message.
    /// \param item Cached GATT service converted to a ROS message.
    /// \return Converted service msg.
    mrs_uav_bluetooth::msg::BleGattService to_service_msg(const bluez::GattServiceInfo& item) const;
    /// \brief Convert a cached characteristic into its ROS response message.
    /// \param item Cached GATT characteristic converted to a ROS message.
    /// \return Converted characteristic msg.
    mrs_uav_bluetooth::msg::BleGattCharacteristic to_characteristic_msg(const bluez::GattCharacteristicInfo& item) const;
    /// \brief Convert a cached descriptor into its ROS response message.
    /// \param item Cached GATT descriptor converted to a ROS message.
    /// \return Converted descriptor msg.
    mrs_uav_bluetooth::msg::BleGattDescriptor to_descriptor_msg(const bluez::GattDescriptorInfo& item) const;

    /// \brief Return a filtered snapshot of cached Bluetooth devices.
    /// \param request Connected-only filter from the list-devices request.
    /// \param response Success state diagnostic and matching cached devices returned to the caller.
    void handle_list_devices(const std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Request> request,
                             std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Response> response);
    /// \brief Resolve one Bluetooth address and return its cached device record.
    /// \param request Bluetooth address of the device to retrieve.
    /// \param response Success state diagnostic and resolved device returned to the caller.
    void handle_get_device(const std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Request> request,
                           std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Response> response);
    /// \brief Connect a peer and optionally wait for remote GATT discovery.
    /// \param request Peer address timeout and service-discovery wait policy.
    /// \param response Connection result resolved address and BlueZ device path returned to the caller.
    void handle_connect_device(const std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Request> request,
                               std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Response> response);
    /// \brief Disconnect a peer within the caller's timeout.
    /// \param request Peer address and disconnect timeout.
    /// \param response Disconnect result and diagnostic returned to the caller.
    void handle_disconnect_device(const std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Request> request,
                                  std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Response> response);
    /// \brief Pair a peer and optionally mark it trusted after success.
    /// \param request Peer address pairing timeout and post-pair trust choice.
    /// \param response Pairing result and diagnostic returned to the caller.
    void handle_pair_device(const std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Request> request,
                            std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Response> response);
    /// \brief Set or clear BlueZ's trusted flag for one peer.
    /// \param request Peer address and desired trusted state.
    /// \param response Trust update result and diagnostic returned to the caller.
    void handle_set_device_trust(const std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Request> request,
                                 std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Response> response);
    /// \brief Forget one peer and clear its local runtime state.
    /// \param request Bluetooth address of the peer to forget.
    /// \param response Removal result and diagnostic returned to the caller.
    void handle_remove_device(const std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Request> request,
                              std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Response> response);
    /// \brief Return all cached services below one peer.
    /// \param request Peer address whose discovered GATT services are requested.
    /// \param response Success state diagnostic and discovered services returned to the caller.
    void handle_list_gatt_services(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Request> request,
                                   std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Response> response);
    /// \brief Return all cached characteristics below one peer.
    /// \param request Peer address whose discovered characteristics are requested.
    /// \param response Success state diagnostic and discovered characteristics returned to the caller.
    void handle_list_gatt_characteristics(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Response> response);
    /// \brief Return descriptors below a peer or selected characteristic.
    /// \param request Peer address and optional characteristic path limiting the descriptor list.
    /// \param response Success state diagnostic and discovered descriptors returned to the caller.
    void handle_list_gatt_descriptors(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Request> request,
                                      std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Response> response);
    /// \brief Resolve a characteristic or descriptor UUID to its BlueZ path.
    /// \param request Peer UUID object kind and optional parent path to resolve.
    /// \param response Success state diagnostic and resolved BlueZ object path returned to the caller.
    void handle_find_gatt_path(const std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Request> request,
                               std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Response> response);
    /// \brief Read bytes from the requested remote characteristic or descriptor.
    /// \param request BlueZ object path and characteristic-or-descriptor selection.
    /// \param response Read result diagnostic and returned bytes.
    void handle_read_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Request> request,
                                std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Response> response);
    /// \brief Write bytes to the requested remote characteristic or descriptor.
    /// \param request BlueZ object path value object kind and response policy.
    /// \param response Write result and diagnostic returned to the caller.
    void handle_write_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Request> request,
                                 std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Response> response);
    /// \brief Enable or disable notifications on one remote characteristic.
    /// \param request Characteristic path and requested notification state.
    /// \param response Notification subscription result and diagnostic returned to the caller.
    void handle_set_notify(const std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Request> request,
                           std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Response> response);
    /// \brief Reconcile the caller's requested discovery state and transport.
    /// \param request Requested scan state and radio transport filter.
    /// \param response Scan update result diagnostic and effective scanning state.
    void handle_set_scan_enabled(const std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Request> request,
                                 std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Response> response);
    /// \brief Create update or remove a runtime bridge for one remote characteristic.
    /// \param request Bridge direction peer characteristic topic type fields rate and enable state.
    /// \param response Result diagnostic and fully resolved bridge identity returned to the caller.
    void handle_configure_notification_bridge(const std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Request> request,
                                              std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Response> response);
    /// \brief Reload and reapply the current base and overlay YAML files.
    /// \param request Empty trigger request to reload the current base and overlay files.
    /// \param response Reload result and diagnostic returned to the caller.
    void handle_reload_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// \brief Validate activate and persist a new overlay lease.
    /// \param request Overlay path and optional initial lease grace period.
    /// \param response Activation result diagnostic active path and lease state.
    void handle_set_active_config(const std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Request> request,
                                  std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Response> response);
    /// \brief Execute Mesh join attach leave create or import through D-Bus.
    /// \param request Mesh lifecycle action and credentials addresses or token required by that action.
    /// \param response Result diagnostic and resulting Mesh attachment state.
    void handle_mesh_network(const std::shared_ptr<mrs_uav_bluetooth::srv::MeshNetwork::Request> request,
                             std::shared_ptr<mrs_uav_bluetooth::srv::MeshNetwork::Response> response);
    /// \brief Execute a Mesh access send publication or configuration-client key message.
    /// \param request Mesh send mode addressing key model segmentation and payload fields.
    /// \param response Mesh transmission or configuration result and diagnostic.
    void handle_mesh_send(const std::shared_ptr<mrs_uav_bluetooth::srv::MeshSend::Request> request,
                          std::shared_ptr<mrs_uav_bluetooth::srv::MeshSend::Response> response);
    /// \brief Execute Mesh provisioning keyring or remote-node management through D-Bus.
    /// \param request Mesh provisioning key-management or remote-node action with its operands.
    /// \param response Management result diagnostic and exported key data when requested.
    void handle_mesh_management(const std::shared_ptr<mrs_uav_bluetooth::srv::MeshManagement::Request> request,
                                std::shared_ptr<mrs_uav_bluetooth::srv::MeshManagement::Response> response);
    /// Runtime logical-swarm control never changes Mesh provisioning credentials.
    /// \param request Logical Mesh group status join or leave action and group identifier.
    /// \param response Result diagnostic current group participation and member list.
    void handle_mesh_swarm(const std::shared_ptr<mrs_uav_bluetooth::srv::MeshSwarm::Request> request,
                           std::shared_ptr<mrs_uav_bluetooth::srv::MeshSwarm::Response> response);

    std::string hostname_;
    std::string adapter_path_;
    config::NodeConfig active_config_;

    std::unique_ptr<bluez::DbusConnection> dbus_;
    std::unique_ptr<bluez::DbusConnection> server_dbus_;
    std::unique_ptr<bluez::ObjectManagerCache> cache_;
    std::unique_ptr<bluez::AdapterController> adapter_;
    std::unique_ptr<bluez::BluezClient> client_;
    std::unique_ptr<bluez::BluezPairingAgent> pairing_agent_;
    std::unique_ptr<bluez::SerialPortProfile> serial_profile_;
    std::unique_ptr<serial::SerialSshServer> serial_ssh_server_;
    std::string serial_sshd_path_;
    // A dedicated connection lets BlueZ observe detach when an overlay ends.
    std::unique_ptr<bluez::DbusConnection> mesh_dbus_;
    std::unique_ptr<mesh::MeshApplication> mesh_app_;
    /// Steady-clock observation, independent of BlueZ's platform-specific clock.
    std::atomic<int64_t> mesh_last_peer_heard_ns_{0};
    std::unique_ptr<mesh::MeshSwarmCoordinator> mesh_swarm_coordinator_;
    std::atomic_bool mesh_bootstrap_attempted_{false};
    /// Bits are set only by matching local Config Server status messages.
    std::atomic<uint8_t> mesh_local_radio_confirmed_{0};
    std::atomic<int64_t> mesh_local_radio_retry_after_ns_{0};
    std::atomic<uint8_t> mesh_local_ttl_expected_{0};
    std::atomic<uint8_t> mesh_local_relay_expected_{0};
    std::atomic<uint8_t> mesh_local_network_transmit_expected_{0};
    std::string mesh_topic_prefix_;

    std::unique_ptr<config::OverlayConfigManager> overlay_config_;
    std::string mesh_config_signature_;

    std::unique_ptr<network::NetplanManager> netplan_;

    std::unique_ptr<gatt::GattApplication> gatt_app_;
    std::unique_ptr<gatt::Advertisement> advertisement_;
    std::unique_ptr<gatt::services::WifiService> wifi_service_;
    std::unique_ptr<gatt::services::TimeService> time_service_;

    bridge::BridgeRegistry bridge_registry_;
    std::unique_ptr<bridge::ExportBridgeManager> export_bridges_;
    std::unique_ptr<bridge::ImportBridgeManager> import_bridges_;
    std::unique_ptr<bridge::TransportBridgeManager> transport_bridges_;
    std::unique_ptr<peer::PeerManager> peers_;
    std::unique_ptr<ros::RosInterfaceManager> ros_;

    std::vector<rclcpp::ServiceBase::SharedPtr> services_;
    int cache_observer_token_{0};
    int gatt_event_token_{0};
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr peer_callback_group_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr lease_timer_;
    rclcpp::TimerBase::SharedPtr peer_timer_;
    rclcpp::TimerBase::SharedPtr time_service_timer_;
    rclcpp::TimerBase::SharedPtr wifi_service_timer_;
    rclcpp::TimerBase::SharedPtr mesh_timer_;
    std::chrono::steady_clock::time_point mesh_next_maintenance_{};
    rclcpp::Publisher<mrs_uav_bluetooth::msg::MeshStatus>::SharedPtr mesh_status_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::MeshMessage>::SharedPtr mesh_message_pub_;
    rclcpp::Publisher<mrs_uav_bluetooth::msg::MeshEvent>::SharedPtr mesh_event_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr advertisement_payload_sub_;
    std::chrono::steady_clock::time_point peer_reconcile_deadline_{};
    std::atomic_bool shutting_down_{false};
    // BlueZ emits Adapter1 PropertiesChanged while the service is applying a
    // group of adapter properties. Guard the group and defer reconciliation to
    // the lease timer so asynchronous property events can never recursively
    // write back to BlueZ from its D-Bus callback thread.
    std::atomic_bool adapter_state_apply_in_progress_{false};
    std::atomic_bool adapter_state_reconcile_requested_{false};
    std::atomic_bool bluez_daemon_recovery_requested_{false};
    // Some controllers reject policy writes while Mesh owns an advertising
    // operation. Keep the correction request pending, but do not retry it on
    // every one-second lease tick and flood BlueZ/the journal.
    std::chrono::steady_clock::time_point next_adapter_state_reconcile_{};
    // Overlay activation runs in a reentrant service callback while timers and
    // D-Bus events remain live. Serialize full config transitions and expose a
    // cheap flag that those callbacks can use to defer corrective work.
    std::mutex config_apply_mutex_;
    std::atomic_bool config_apply_in_progress_{false};
    mutable std::recursive_mutex state_mutex_;
    mutable std::mutex peer_task_mutex_;
    std::shared_future<void> peer_task_;
    std::string peer_task_mac_;
    std::string peer_task_label_;
    mutable std::mutex log_state_mutex_;
    std::string last_status_log_summary_;
    std::chrono::steady_clock::time_point last_status_log_time_{};
    std::optional<std::vector<uint8_t>> advertisement_extra_payload_;
    /// Monotonic slot number used to vary connectionless receive-window length.
    uint64_t advertisement_scan_sequence_{0};
    std::map<std::string, std::string> last_peer_status_log_;
    std::chrono::steady_clock::time_point last_peer_status_log_time_{};
    mutable std::mutex repeated_log_mutex_;
    std::map<std::string, RepeatedLogEntry> repeated_log_entries_;
    std::map<std::string, std::string> expected_disconnect_reasons_;
    double local_server_rebuild_monotonic_{0.0};
    std::atomic_bool local_server_rebuild_in_progress_{false};
    std::string local_gatt_layout_signature_;
    // Reliable samples are unicast to every live member. Compact receipts and
    // receipt queries avoid repeating a segmented frame; only missing members
    // receive a full repair.
    mutable std::mutex mesh_reliable_mutex_;
    std::map<uint16_t, MeshReliablePending> mesh_reliable_pending_;
    // One in-flight sample and one latest waiting value per channel keep a
    // congested radio from accumulating obsolete topic values.
    std::map<uint64_t, std::pair<config::SharedTopicConfig, std::vector<uint8_t>>>
        mesh_reliable_latest_;
    // Receipt identity includes the source, AppKey, swarm, vendor opcode and
    // sequence, so independent bridge protocols cannot acknowledge each other.
    using MeshReliableSeenKey =
        std::tuple<uint16_t, uint16_t, uint16_t, uint32_t, uint16_t>;
    std::map<MeshReliableSeenKey, MeshReliableSeen> mesh_reliable_seen_;
    std::deque<MeshReliableAck> mesh_reliable_acks_;
    // D-Bus receive callbacks only enqueue cleanup. The ROS executor performs
    // calls after releasing the reliability mutex, avoiding callback deadlocks.
    std::deque<mesh::SendHandle> mesh_transfer_cancellations_;
    std::atomic_bool mesh_reliable_reset_requested_{false};
    std::map<uint16_t, MeshReliablePeerState> mesh_reliable_peers_;
    // Active members alternate complete segmented values in address order.
    // This prevents simultaneous SAR transactions from repeatedly colliding;
    // the lowest live address deterministically recovers a lost turn.
    std::map<uint64_t, MeshReliableTurn> mesh_reliable_turns_;
    std::chrono::steady_clock::time_point mesh_reliable_next_probe_at_{};
    size_t mesh_reliable_probe_cursor_{0};
    uint32_t mesh_reliable_next_sequence_{
        static_cast<uint32_t>(std::chrono::steady_clock::now()
            .time_since_epoch().count())};

    std::set<std::string> peer_runtime_clear_in_progress_;
};

}  // namespace mrs_uav_bluetooth::app
