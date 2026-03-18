// SPDX-License-Identifier: MIT
#include "mrs_uav_bluetooth/bluez/adapter_controller.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <map>

namespace mrs_uav_bluetooth::bluez {

AdapterController::AdapterController(DbusConnection& dbus,
                                     const std::string& adapter_path,
                                     rclcpp::Logger logger)
    : dbus_(dbus), adapter_path_(adapter_path), logger_(logger)
{
    proxy_ = sdbus::createProxy(dbus_.connection(),
                                sdbus::ServiceName{std::string(kBluezServiceName)},
                                sdbus::ObjectPath{adapter_path_});
}

void AdapterController::power_on() {
    set_adapter_property("Powered", sdbus::Variant{true});
    RCLCPP_INFO(logger_, "Adapter %s powered on", adapter_path_.c_str());
}

void AdapterController::set_discoverable(bool discoverable, uint32_t timeout) {
    set_adapter_property("DiscoverableTimeout", sdbus::Variant{timeout});
    set_adapter_property("Discoverable", sdbus::Variant{discoverable});
    RCLCPP_INFO(logger_, "Adapter discoverable=%s timeout=%u",
                discoverable ? "true" : "false", timeout);
}

void AdapterController::set_pairable(bool pairable) {
    set_adapter_property("Pairable", sdbus::Variant{pairable});
}

void AdapterController::start_discovery(const std::string& transport) {
    try {
        std::map<std::string, sdbus::Variant> filter;
        filter["Transport"] = sdbus::Variant{transport};
        proxy_->callMethod("SetDiscoveryFilter")
              .onInterface(std::string(kAdapterIface))
              .withArguments(filter);
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "SetDiscoveryFilter failed: %s", e.what());
    }

    try {
        proxy_->callMethod("StartDiscovery")
              .onInterface(std::string(kAdapterIface));
        RCLCPP_INFO(logger_, "Discovery started (transport=%s)", transport.c_str());
    } catch (const sdbus::Error& e) {
        // "Already discovering" is not fatal.
        RCLCPP_WARN(logger_, "StartDiscovery: %s", e.what());
    }
}

void AdapterController::stop_discovery() {
    try {
        proxy_->callMethod("StopDiscovery")
              .onInterface(std::string(kAdapterIface));
        RCLCPP_INFO(logger_, "Discovery stopped");
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "StopDiscovery: %s", e.what());
    }
}

void AdapterController::remove_device(const std::string& device_path) {
    try {
        proxy_->callMethod("RemoveDevice")
              .onInterface(std::string(kAdapterIface))
              .withArguments(sdbus::ObjectPath{device_path});
        RCLCPP_INFO(logger_, "Removed device %s", device_path.c_str());
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "RemoveDevice(%s): %s", device_path.c_str(), e.what());
    }
}

void AdapterController::set_adapter_property(const std::string& name,
                                             const sdbus::Variant& value) {
    try {
        proxy_->callMethod("Set")
              .onInterface(std::string(kDbusPropertiesIface))
              .withArguments(std::string(kAdapterIface), name, value);
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "Set adapter property %s failed: %s",
                    name.c_str(), e.what());
    }
}

}  // namespace mrs_uav_bluetooth::bluez
