// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/ros/service_servers.hpp"

namespace mrs_uav_bluetooth::ros {

ServiceServers::ServiceServers(rclcpp::Node& node)
    : node_(node) {}

std::vector<rclcpp::ServiceBase::SharedPtr> ServiceServers::register_all(
    rclcpp::Node& owner,
    const Handlers& handlers,
    const rclcpp::CallbackGroup::SharedPtr& callback_group) {
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    services.reserve(18);

    services.push_back(create<mrs_uav_bluetooth::srv::ListDevices>(owner, "ble/list_devices", handlers.list_devices, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::GetDevice>(owner, "ble/get_device", handlers.get_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ConnectDevice>(owner, "ble/connect_device", handlers.connect_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::DisconnectDevice>(owner, "ble/disconnect_device", handlers.disconnect_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::PairDevice>(owner, "ble/pair_device", handlers.pair_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetDeviceTrust>(owner, "ble/set_device_trust", handlers.set_device_trust, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::RemoveDevice>(owner, "ble/remove_device", handlers.remove_device, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ListGattServices>(owner, "ble/list_gatt_services", handlers.list_gatt_services, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ListGattCharacteristics>(owner, "ble/list_gatt_characteristics", handlers.list_gatt_characteristics, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ListGattDescriptors>(owner, "ble/list_gatt_descriptors", handlers.list_gatt_descriptors, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::FindGattPath>(owner, "ble/find_gatt_path", handlers.find_gatt_path, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ReadGattValue>(owner, "ble/read_gatt_value", handlers.read_gatt_value, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::WriteGattValue>(owner, "ble/write_gatt_value", handlers.write_gatt_value, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetNotify>(owner, "ble/set_notify", handlers.set_notify, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetScanEnabled>(owner, "ble/set_scan_enabled", handlers.set_scan_enabled, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::ConfigureNotificationBridge>(owner, "ble/configure_notification_bridge", handlers.configure_notification_bridge, callback_group));
    services.push_back(create<std_srvs::srv::Trigger>(owner, "ble/reload_config", handlers.reload_config, callback_group));
    services.push_back(create<mrs_uav_bluetooth::srv::SetActiveConfig>(owner, "ble/set_active_config", handlers.set_active_config, callback_group));

    return services;
}

}  // namespace mrs_uav_bluetooth::ros
