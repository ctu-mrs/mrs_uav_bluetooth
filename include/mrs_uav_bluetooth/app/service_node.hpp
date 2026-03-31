// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "mrs_uav_bluetooth/bluez/adapter_controller.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_pairing_agent.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"
#include "mrs_uav_bluetooth/bridge/bridge_registry.hpp"
#include "mrs_uav_bluetooth/msg/ble_peer_time_status.hpp"
#include "mrs_uav_bluetooth/bridge/export_bridge_manager.hpp"
#include "mrs_uav_bluetooth/bridge/import_bridge_manager.hpp"
#include "mrs_uav_bluetooth/config/overlay_config_manager.hpp"
#include "mrs_uav_bluetooth/gatt/advertisement.hpp"
#include "mrs_uav_bluetooth/gatt/gatt_application.hpp"
#include "mrs_uav_bluetooth/gatt/services/time_service.hpp"
#include "mrs_uav_bluetooth/gatt/services/wifi_service.hpp"
#include "mrs_uav_bluetooth/network/netplan_manager.hpp"
#include "mrs_uav_bluetooth/peer/peer_manager.hpp"
#include "mrs_uav_bluetooth/ros/ros_interface_manager.hpp"
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

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <future>
#include <atomic>
#include <mutex>
#include <set>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::app {

class ServiceNode : public rclcpp::Node {
public:
    ServiceNode();
    ~ServiceNode() override;

private:
    struct RepeatedLogEntry {
        std::chrono::steady_clock::time_point last_emit_time{};
        size_t suppressed_count{0};
    };

    void configure_parameters();
    void build_runtime();
    void create_services();
    void apply_config(const config::NodeConfig& cfg);
    void apply_adapter_state(const config::NodeConfig& cfg);
    void rebuild_server_objects();
    void publish_periodic_status();
    std::string build_detailed_status_report(const std::map<std::string, bluez::DeviceInfo>& devices_map) const;
    void on_cache_event(bluez::CacheEvent event, const std::string& object_path);
    void on_gatt_event(const std::string& event_type,
                       const std::string& object_path,
                       const std::string& detail);
    void on_notification(const std::vector<uint8_t>& data,
                         const std::string& uuid,
                         const std::string& characteristic_path);
    void handle_time_writeback(const std::vector<uint8_t>& payload,
                               const std::string& device_path,
                               uint64_t received_time_ns);
    void on_pairing_event(const std::string& event_type, const std::string& device_path);
    void note_pair_attempt_result(const std::string& mac,
                                  bool success,
                                  const std::string& error_detail);
    bool should_allow_pairing_request(const std::string& event_type,
                                      const std::string& device_path);
    void reconcile_peers();
    void schedule_peer_reconcile(std::chrono::milliseconds delay = std::chrono::milliseconds(0));
    void clear_peer_runtime(const std::string& mac,
                            const std::string& skip_characteristic_path = {});
    void refresh_import_bridges_for_device(const bluez::DeviceInfo& device);
    void prune_missing_import_bridges(const std::string& mac,
                                      const std::set<std::string>& desired_keys,
                                      double now_mono,
                                      double missing_path_grace_s);
    std::string peer_status_topic(const std::string& mac, const std::string& peer_name) const;
    std::string peer_bridge_topic(const std::string& mac,
                                  const std::string& peer_name,
                                  const std::string& requested_topic_suffix) const;
    bool update_peer_time_bridge(const std::string& mac,
                                 const bluez::DeviceInfo& device,
                                 peer::PeerConnectionSession& session);
    void publish_peer_time_status(peer::PeerTimeBridge& bridge) const;
    bool run_peer_task_once(const std::string& mac,
                            const std::string& label,
                            std::function<void()> task);
    void wait_for_peer_tasks();
    void log_info_coalesced(const std::string& key, const std::string& message);
    void log_warn_coalesced(const std::string& key, const std::string& message);
    bool can_run_callbacks() const;

    mrs_uav_bluetooth::msg::BleDevice to_device_msg(const bluez::DeviceInfo& device) const;
    mrs_uav_bluetooth::msg::BleGattService to_service_msg(const bluez::GattServiceInfo& item) const;
    mrs_uav_bluetooth::msg::BleGattCharacteristic to_characteristic_msg(const bluez::GattCharacteristicInfo& item) const;
    mrs_uav_bluetooth::msg::BleGattDescriptor to_descriptor_msg(const bluez::GattDescriptorInfo& item) const;

    void handle_list_devices(const std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Request> request,
                             std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Response> response);
    void handle_get_device(const std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Request> request,
                           std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Response> response);
    void handle_connect_device(const std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Request> request,
                               std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Response> response);
    void handle_disconnect_device(const std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Request> request,
                                  std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Response> response);
    void handle_pair_device(const std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Request> request,
                            std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Response> response);
    void handle_set_device_trust(const std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Request> request,
                                 std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Response> response);
    void handle_remove_device(const std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Request> request,
                              std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Response> response);
    void handle_list_gatt_services(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Request> request,
                                   std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Response> response);
    void handle_list_gatt_characteristics(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Request> request,
                                          std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Response> response);
    void handle_list_gatt_descriptors(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Request> request,
                                      std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Response> response);
    void handle_find_gatt_path(const std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Request> request,
                               std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Response> response);
    void handle_read_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Request> request,
                                std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Response> response);
    void handle_write_gatt_value(const std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Request> request,
                                 std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Response> response);
    void handle_set_notify(const std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Request> request,
                           std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Response> response);
    void handle_set_scan_enabled(const std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Request> request,
                                 std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Response> response);
    void handle_configure_notification_bridge(const std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Request> request,
                                              std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Response> response);
    void handle_reload_config(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_set_active_config(const std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Request> request,
                                  std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Response> response);

    std::string hostname_;
    std::string adapter_path_;
    config::NodeConfig active_config_;

    std::unique_ptr<bluez::DbusConnection> dbus_;
    std::unique_ptr<bluez::DbusConnection> server_dbus_;
    std::unique_ptr<bluez::ObjectManagerCache> cache_;
    std::unique_ptr<bluez::AdapterController> adapter_;
    std::unique_ptr<bluez::BluezClient> client_;
    std::unique_ptr<bluez::BluezPairingAgent> pairing_agent_;

    std::unique_ptr<config::OverlayConfigManager> overlay_config_;

    std::unique_ptr<network::NetplanManager> netplan_;

    std::unique_ptr<gatt::GattApplication> gatt_app_;
    std::unique_ptr<gatt::Advertisement> advertisement_;
    std::unique_ptr<gatt::services::WifiService> wifi_service_;
    std::unique_ptr<gatt::services::TimeService> time_service_;

    bridge::BridgeRegistry bridge_registry_;
    std::unique_ptr<bridge::ExportBridgeManager> export_bridges_;
    std::unique_ptr<bridge::ImportBridgeManager> import_bridges_;
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
    std::chrono::steady_clock::time_point peer_reconcile_deadline_{};
    std::atomic_bool shutting_down_{false};
    mutable std::recursive_mutex state_mutex_;
    mutable std::mutex peer_task_mutex_;
    std::shared_future<void> peer_task_;
    std::string peer_task_mac_;
    std::string peer_task_label_;
    mutable std::mutex log_state_mutex_;
    std::string last_status_log_summary_;
    std::chrono::steady_clock::time_point last_status_log_time_{};
    std::map<std::string, std::string> last_peer_status_log_;
    std::chrono::steady_clock::time_point last_peer_status_log_time_{};
    mutable std::mutex repeated_log_mutex_;
    std::map<std::string, RepeatedLogEntry> repeated_log_entries_;
    double local_server_rebuild_monotonic_{0.0};
    std::atomic_bool local_server_rebuild_in_progress_{false};
    std::string local_gatt_layout_signature_;
    std::set<std::string> peer_runtime_clear_in_progress_;
};

}  // namespace mrs_uav_bluetooth::app
