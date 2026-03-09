"""Reusable server-side BLE GATT primitives."""

import dbus

from .dbus_common import GATT_CHRC_IFACE, dbus_byte_array
from .dbus_gatt import Characteristic, Descriptor


class WritableDescriptor(Descriptor):
    def __init__(self, bus, index, uuid, flags, characteristic, *, read_cb=None, write_cb=None):
        super().__init__(bus, index, uuid, flags, characteristic)
        self._read_cb = read_cb
        self._write_cb = write_cb

    def ReadValue(self, options):
        del options
        if self._read_cb is not None:
            value = self._read_cb()
            self.set_value(value)
        return dbus_byte_array(self.value or b"")

    def WriteValue(self, value, options):
        del options
        data = bytes(value)
        if self._write_cb is not None:
            self._write_cb(data)
        self.set_value(data, emit=True)


class ReadOnlyDescriptor(Descriptor):
    def __init__(self, bus, index, uuid, characteristic, *, read_cb=None, initial_value=b""):
        super().__init__(bus, index, uuid, ["read"], characteristic, value=initial_value)
        self._read_cb = read_cb

    def ReadValue(self, options):
        del options
        if self._read_cb is not None:
            self.set_value(self._read_cb())
        return dbus_byte_array(self.value or b"")


class NotifyingCharacteristic(Characteristic):
    def __init__(self, bus, index, uuid, flags, service, *, read_cb=None, write_cb=None, notify_cb=None, initial_value=b""):
        super().__init__(bus, index, uuid, flags, service, value=initial_value)
        self._read_cb = read_cb
        self._write_cb = write_cb
        self._notify_cb = notify_cb

    def ReadValue(self, options):
        del options
        if self._read_cb is not None:
            value = self._read_cb()
            self.set_value(value)
        return dbus_byte_array(self.value)

    def WriteValue(self, value, options):
        del options
        data = bytes(value)
        self.set_value(data, emit=True)
        if self._write_cb is not None:
            self._write_cb(data)

    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        self.PropertiesChanged(GATT_CHRC_IFACE, {"Notifying": dbus.Boolean(True)}, [])
        if self._notify_cb is not None:
            self._notify_cb(True)

    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
        self.PropertiesChanged(GATT_CHRC_IFACE, {"Notifying": dbus.Boolean(False)}, [])
        if self._notify_cb is not None:
            self._notify_cb(False)

    def publish(self, value: bytes):
        self.set_value(value, emit=self.notifying)
