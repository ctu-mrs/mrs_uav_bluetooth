// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bluez/adapter_controller.cpp
/// \brief Implements the adapter controller component of the BlueZ system-D-Bus integration layer.

#include "mrs_uav_bluetooth/bluez/adapter_controller.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <chrono>
#include <map>
#include <thread>

namespace mrs_uav_bluetooth::bluez {

AdapterController::AdapterController(DbusConnection& dbus,
                                     const std::string& adapter_path,
                                     rclcpp::Logger logger)
    : dbus_(dbus), adapter_path_(adapter_path), logger_(logger)
{
    // Bind property operations to the selected adapter object path.
    proxy_ = sdbus::createProxy(dbus_.connection(),
                                sdbus::ServiceName{std::string(kBluezServiceName)},
                                sdbus::ObjectPath{adapter_path_});
}

void AdapterController::power_on() {
    // Change power for on.
    set_adapter_property("Powered", sdbus::Variant{true});
    RCLCPP_INFO(logger_, "Adapter %s powered on", adapter_path_.c_str());
}

void AdapterController::power_off() {
    // Change power for off.
    set_adapter_property("Powered", sdbus::Variant{false});
    RCLCPP_INFO(logger_, "Adapter %s powered off", adapter_path_.c_str());
}

void AdapterController::set_discoverable(bool discoverable, uint32_t timeout) {
    // Set whether nearby scanners can discover this adapter through Adapter1.
    set_adapter_property("DiscoverableTimeout", sdbus::Variant{timeout});
    set_adapter_property("Discoverable", sdbus::Variant{discoverable});
    RCLCPP_INFO(logger_, "Adapter discoverable=%s timeout=%u",
                discoverable ? "true" : "false", timeout);
}

void AdapterController::set_connectable(bool connectable) {
    // Set LEAdvertisingManager1.Connectable for incoming connection acceptance.
    set_adapter_property("Connectable", sdbus::Variant{connectable});
    RCLCPP_INFO(logger_, "Adapter connectable=%s", connectable ? "true" : "false");
}

void AdapterController::set_pairable(bool pairable) {
    // Set whether BlueZ accepts new pairing procedures on this adapter.
    set_adapter_property("Pairable", sdbus::Variant{pairable});
}

void AdapterController::set_pairable_timeout(uint32_t timeout) {
    // Set how long BlueZ keeps pairing enabled before disabling it automatically.
    set_adapter_property("PairableTimeout", sdbus::Variant{timeout});
    RCLCPP_INFO(logger_, "Adapter pairable timeout=%u", timeout);
}

void AdapterController::set_alias(const std::string& alias) {
    // Set the human-readable adapter name exposed to nearby peers.
    set_adapter_property("Alias", sdbus::Variant{alias});
    RCLCPP_INFO(logger_, "Adapter alias=%s", alias.c_str());
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
    // Tear down discovery.
    try {
        proxy_->callMethod("StopDiscovery")
              .onInterface(std::string(kAdapterIface));
        RCLCPP_INFO(logger_, "Discovery stopped");
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "StopDiscovery: %s", e.what());
    }
}

void AdapterController::remove_device(const std::string& device_path) {
    // Forward the exact Device1 path to Adapter1.RemoveDevice and report failures.
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
    // BlueZ returns Busy while a management command for this property is
    // pending, including commands issued internally when discovery stops.
    // Retry that transient state within a bounded deadline. A successful Set
    // confirms the requested value, while other errors reach the caller so
    // the configuration transaction cannot report a setting it did not apply.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    for (;;) {
        try {
            proxy_->callMethod("Set")
                  .onInterface(std::string(kDbusPropertiesIface))
                  .withTimeout(std::chrono::seconds(2))
                  .withArguments(std::string(kAdapterIface), name, value);
            return;
        } catch (const sdbus::Error& error) {
            if (error.getName() != "org.bluez.Error.Busy" ||
                std::chrono::steady_clock::now() >= deadline) {
                RCLCPP_WARN(logger_, "Set adapter property %s failed: %s",
                            name.c_str(), error.what());
                throw;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

}  // namespace mrs_uav_bluetooth::bluez
