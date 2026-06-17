// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
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

// ---- Bluetooth GAP advertising data type identifiers ----
inline constexpr uint8_t kAdTypeFlags = 0x01;
inline constexpr uint8_t kAdTypeIncomplete16BitServiceUuids = 0x02;
inline constexpr uint8_t kAdTypeComplete16BitServiceUuids = 0x03;
inline constexpr uint8_t kAdTypeIncomplete32BitServiceUuids = 0x04;
inline constexpr uint8_t kAdTypeComplete32BitServiceUuids = 0x05;
inline constexpr uint8_t kAdTypeIncomplete128BitServiceUuids = 0x06;
inline constexpr uint8_t kAdTypeComplete128BitServiceUuids = 0x07;
inline constexpr uint8_t kAdTypeShortenedLocalName = 0x08;
inline constexpr uint8_t kAdTypeCompleteLocalName = 0x09;
inline constexpr uint8_t kAdTypeTxPowerLevel = 0x0A;
inline constexpr uint8_t kAdTypeServiceData16BitUuid = 0x16;
inline constexpr uint8_t kAdTypeServiceData32BitUuid = 0x20;
inline constexpr uint8_t kAdTypeServiceData128BitUuid = 0x21;
inline constexpr uint8_t kAdTypeTransportDiscoveryData = 0x26;
inline constexpr uint8_t kAdTypeManufacturerSpecificData = 0xFF;

inline constexpr uint8_t kDefaultAdvertisementExtraDataType = kAdTypeTransportDiscoveryData;

}  // namespace mrs_uav_bluetooth::bluez
