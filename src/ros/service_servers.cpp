// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/ros/service_servers.hpp"

namespace mrs_uav_bluetooth::ros {

ServiceServers::ServiceServers(rclcpp::Node& node)
    : node_(node) {}

std::vector<rclcpp::ServiceBase::SharedPtr> ServiceServers::register_all(
    rclcpp::Node& owner,
    const Handlers& handlers,
    const std::string& service_root,
    const rclcpp::CallbackGroup::SharedPtr& callback_group) {
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    services.reserve(18);

    const auto le_root = service_root + "/le";
    const auto config_root = service_root + "/config";

    // LE device discovery and lifecycle operations.
    services.push_back(create<mrs_uav_bluetooth::srv::ListDevices>(owner, le_root + "/devices/list", handlers.list_devices, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::GetDevice>(owner, le_root + "/devices/get", handlers.get_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ConnectDevice>(owner, le_root + "/devices/connect", handlers.connect_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::DisconnectDevice>(owner, le_root + "/devices/disconnect", handlers.disconnect_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::PairDevice>(owner, le_root + "/devices/pair", handlers.pair_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetDeviceTrust>(owner, le_root + "/devices/set_trust", handlers.set_device_trust, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::RemoveDevice>(owner, le_root + "/devices/remove", handlers.remove_device, callback_group));
    // Remote GATT introspection and value/notification operations.
    services.push_back(create<mrs_uav_bluetooth::srv::ListGattServices>(owner, le_root + "/gatt/services/list", handlers.list_gatt_services, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ListGattCharacteristics>(owner, le_root + "/gatt/characteristics/list", handlers.list_gatt_characteristics, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ListGattDescriptors>(owner, le_root + "/gatt/descriptors/list", handlers.list_gatt_descriptors, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::FindGattPath>(owner, le_root + "/gatt/path/find", handlers.find_gatt_path, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ReadGattValue>(owner, le_root + "/gatt/value/read", handlers.read_gatt_value, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::WriteGattValue>(owner, le_root + "/gatt/value/write", handlers.write_gatt_value, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetNotify>(owner, le_root + "/gatt/notifications/set", handlers.set_notify, callback_group));
    // Adapter scan control and dynamic ROS/GATT bridge configuration.
    services.push_back(create<mrs_uav_bluetooth::srv::SetScanEnabled>(owner, le_root + "/scan/set_enabled", handlers.set_scan_enabled, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ConfigureNotificationBridge>(owner, le_root + "/notification_bridge/configure", handlers.configure_notification_bridge, callback_group));
    // Base/overlay configuration lifecycle, intentionally separate from LE.
    services.push_back(create<std_srvs::srv::Trigger>(owner, config_root + "/reload", handlers.reload_config, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetActiveConfig>(owner, config_root + "/set_active", handlers.set_active_config, callback_group));

    return services;
}

}  // namespace mrs_uav_bluetooth::ros
