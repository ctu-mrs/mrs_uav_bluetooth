"""D-Bus LE Advertisement helpers."""

import dbus
import dbus.service

from .dbus_common import DBUS_PROP_IFACE, LE_ADVERTISEMENT_IFACE, dict_with_byte_arrays


class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/bluez/app/advertisement"

    def __init__(self, bus, index, ad_type="peripheral"):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = ad_type
        self.service_uuids = []
        self.manufacturer_data = {}
        self.solicit_uuids = []
        self.service_data = {}
        self.data = {}
        self.scan_response_service_uuids = []
        self.scan_response_manufacturer_data = {}
        self.scan_response_solicit_uuids = []
        self.scan_response_service_data = {}
        self.scan_response_data = {}
        self.local_name = None
        self.discoverable = None
        self.discoverable_timeout = None
        self.includes = []
        self.appearance = None
        self.duration = None
        self.timeout = None
        self.secondary_channel = None
        self.min_interval = None
        self.max_interval = None
        self.tx_power = None
        dbus.service.Object.__init__(self, bus, self.path)

    @property
    def include_tx_power(self):
        return "tx-power" in self.includes

    @include_tx_power.setter
    def include_tx_power(self, enabled):
        if enabled and "tx-power" not in self.includes:
            self.includes.append("tx-power")
        if not enabled:
            self.includes = [item for item in self.includes if item != "tx-power"]

    def add_service_uuid(self, uuid):
        self.service_uuids.append(str(uuid))

    def add_solicit_uuid(self, uuid):
        self.solicit_uuids.append(str(uuid))

    def add_service_data(self, uuid, value):
        self.service_data[str(uuid)] = bytes(value)

    def add_manufacturer_data(self, manufacturer_id, value):
        self.manufacturer_data[int(manufacturer_id)] = bytes(value)

    def add_data(self, ad_type, value):
        self.data[int(ad_type)] = bytes(value)

    def get_properties(self):
        props = {"Type": dbus.String(self.ad_type)}
        if self.service_uuids:
            props["ServiceUUIDs"] = dbus.Array(self.service_uuids, signature="s")
        if self.manufacturer_data:
            props["ManufacturerData"] = dict_with_byte_arrays(
                self.manufacturer_data,
                key_wrapper=dbus.UInt16,
                signature="qv",
            )
        if self.solicit_uuids:
            props["SolicitUUIDs"] = dbus.Array(self.solicit_uuids, signature="s")
        if self.service_data:
            props["ServiceData"] = dict_with_byte_arrays(self.service_data, signature="sv")
        if self.data:
            props["Data"] = dict_with_byte_arrays(self.data, key_wrapper=dbus.Byte, signature="yv")
        if self.scan_response_service_uuids:
            props["ScanResponseServiceUUIDs"] = dbus.Array(
                self.scan_response_service_uuids,
                signature="s",
            )
        if self.scan_response_manufacturer_data:
            props["ScanResponseManufacturerData"] = dict_with_byte_arrays(
                self.scan_response_manufacturer_data,
                key_wrapper=dbus.UInt16,
                signature="qv",
            )
        if self.scan_response_solicit_uuids:
            props["ScanResponseSolicitUUIDs"] = dbus.Array(
                self.scan_response_solicit_uuids,
                signature="s",
            )
        if self.scan_response_service_data:
            props["ScanResponseServiceData"] = dict_with_byte_arrays(
                self.scan_response_service_data,
                signature="sv",
            )
        if self.scan_response_data:
            props["ScanResponseData"] = dict_with_byte_arrays(
                self.scan_response_data,
                key_wrapper=dbus.Byte,
                signature="yv",
            )
        if self.local_name is not None:
            props["LocalName"] = dbus.String(self.local_name)
        if self.discoverable is not None:
            props["Discoverable"] = dbus.Boolean(self.discoverable)
        if self.discoverable_timeout is not None:
            props["DiscoverableTimeout"] = dbus.UInt16(int(self.discoverable_timeout))
        if self.includes:
            props["Includes"] = dbus.Array(self.includes, signature="s")
        if self.appearance is not None:
            props["Appearance"] = dbus.UInt16(int(self.appearance))
        if self.duration is not None:
            props["Duration"] = dbus.UInt16(int(self.duration))
        if self.timeout is not None:
            props["Timeout"] = dbus.UInt16(int(self.timeout))
        if self.secondary_channel is not None:
            props["SecondaryChannel"] = dbus.String(self.secondary_channel)
        if self.min_interval is not None:
            props["MinInterval"] = dbus.UInt32(int(self.min_interval))
        if self.max_interval is not None:
            props["MaxInterval"] = dbus.UInt32(int(self.max_interval))
        if self.tx_power is not None:
            props["TxPower"] = dbus.Int16(int(self.tx_power))
        return {LE_ADVERTISEMENT_IFACE: props}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def destroy(self):
        self.remove_from_connection()

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise dbus.exceptions.DBusException("org.freedesktop.DBus.Error.InvalidArgs")
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        pass
