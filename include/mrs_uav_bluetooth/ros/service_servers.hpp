// SPDX-License-Identifier: BSD-3-Clause
#pragma once

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

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::ros {

class ServiceServers {
public:
    struct Handlers {
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ListDevices::Response>)> list_devices;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::GetDevice::Response>)> get_device;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ConnectDevice::Response>)> connect_device;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::DisconnectDevice::Response>)> disconnect_device;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::PairDevice::Response>)> pair_device;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::SetDeviceTrust::Response>)> set_device_trust;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::RemoveDevice::Response>)> remove_device;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ListGattServices::Response>)> list_gatt_services;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ListGattCharacteristics::Response>)> list_gatt_characteristics;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ListGattDescriptors::Response>)> list_gatt_descriptors;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::FindGattPath::Response>)> find_gatt_path;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ReadGattValue::Response>)> read_gatt_value;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::WriteGattValue::Response>)> write_gatt_value;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::SetNotify::Response>)> set_notify;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::SetScanEnabled::Response>)> set_scan_enabled;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::ConfigureNotificationBridge::Response>)> configure_notification_bridge;
        std::function<void(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response>)> reload_config;
        std::function<void(const std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Request>,
                           std::shared_ptr<mrs_uav_bluetooth::srv::SetActiveConfig::Response>)> set_active_config;
    };

    explicit ServiceServers(rclcpp::Node& node);

    std::vector<rclcpp::ServiceBase::SharedPtr> register_all(rclcpp::Node& owner,
                                                             const Handlers& handlers);

    template<typename ServiceT, typename CallbackT>
    typename rclcpp::Service<ServiceT>::SharedPtr create(rclcpp::Node& owner,
                                                         const std::string& name,
                                                         CallbackT&& cb) {
        return owner.create_service<ServiceT>(name, std::forward<CallbackT>(cb));
    }

private:
    rclcpp::Node& node_;
};

}  // namespace mrs_uav_bluetooth::ros
