// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <string_view>

namespace mrs_uav_bluetooth::bluez {

// ---- BlueZ well-known names ----
inline constexpr std::string_view kBluezServiceName  = "org.bluez";
inline constexpr std::string_view kBluezServicePath  = "/org/bluez";
inline constexpr std::string_view kLocalClientServiceName = "cz.cvut.mrs.uav.ble.client";
inline constexpr std::string_view kLocalServerServiceName = "cz.cvut.mrs.uav.ble.server";

// ---- BlueZ interface names ----
inline constexpr std::string_view kAdapterIface            = "org.bluez.Adapter1";
inline constexpr std::string_view kDeviceIface             = "org.bluez.Device1";
inline constexpr std::string_view kBatteryIface            = "org.bluez.Battery1";
inline constexpr std::string_view kGattManagerIface        = "org.bluez.GattManager1";
inline constexpr std::string_view kGattServiceIface        = "org.bluez.GattService1";
inline constexpr std::string_view kGattCharacteristicIface = "org.bluez.GattCharacteristic1";
inline constexpr std::string_view kGattDescriptorIface     = "org.bluez.GattDescriptor1";
inline constexpr std::string_view kLeAdvManagerIface       = "org.bluez.LEAdvertisingManager1";
inline constexpr std::string_view kLeAdvertisementIface    = "org.bluez.LEAdvertisement1";
inline constexpr std::string_view kAgentIface              = "org.bluez.Agent1";
inline constexpr std::string_view kAgentManagerIface       = "org.bluez.AgentManager1";

// ---- Standard D-Bus interface names ----
inline constexpr std::string_view kDbusObjectManagerIface  = "org.freedesktop.DBus.ObjectManager";
inline constexpr std::string_view kDbusPropertiesIface     = "org.freedesktop.DBus.Properties";

}  // namespace mrs_uav_bluetooth::bluez
