"""Shared BlueZ D-Bus constants and conversion helpers."""

import dbus

BLUEZ_SERVICE_NAME = "org.bluez"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
BATTERY_IFACE = "org.bluez.Battery1"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
GATT_DESC_IFACE = "org.bluez.GattDescriptor1"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
LE_ADVERTISEMENT_IFACE = "org.bluez.LEAdvertisement1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"


def dbus_byte_array(value):
    if value is None:
        return None
    if isinstance(value, dbus.Array):
        return value
    return dbus.Array([dbus.Byte(int(item)) for item in bytes(value)], signature="y")


def dbus_dict(values, *, signature="sv"):
    return dbus.Dictionary(values, signature=signature)


def dbus_object_path_array(paths):
    return dbus.Array([dbus.ObjectPath(str(path)) for path in paths], signature="o")


def dict_with_byte_arrays(source, *, key_wrapper=None, signature="sv"):
    if not source:
        return None
    payload = {}
    for key, value in source.items():
        wrapped_key = key_wrapper(key) if key_wrapper else key
        payload[wrapped_key] = dbus_byte_array(value)
    return dbus_dict(payload, signature=signature)


def bytes_from_variant(value):
    return bytes(bytearray(value))


def decode_byte_dict(value):
    return {str(key): bytes_from_variant(item) for key, item in value.items()}


def decode_manufacturer_dict(value):
    return {int(key): bytes_from_variant(item) for key, item in value.items()}