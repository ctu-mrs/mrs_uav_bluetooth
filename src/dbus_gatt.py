"""D-Bus GATT application helpers."""

import dbus
import dbus.service

from .dbus_common import (
    BLUEZ_SERVICE_PATH,
    BLUEZ_SERVICE_NAME,
    DBUS_OM_IFACE,
    DBUS_PROP_IFACE,
    GATT_CHRC_IFACE,
    GATT_DESC_IFACE,
    GATT_MANAGER_IFACE,
    GATT_SERVICE_IFACE,
    InvalidArgsException,
    NotSupportedException,
    dbus_byte_array,
    dbus_object_path_array,
)

class Application(dbus.service.Object):
    PATH = BLUEZ_SERVICE_PATH + "/app"

    def __init__(self, bus, path=None):
        self.path = path or self.PATH
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    def destroy(self):
        for service in self.services:
            service.destroy()
        self.remove_from_connection()

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for characteristic in service.get_characteristics():
                response[characteristic.get_path()] = characteristic.get_properties()
                for descriptor in characteristic.get_descriptors():
                    response[descriptor.get_path()] = descriptor.get_properties()
        return response


class Service(dbus.service.Object):
    PATH_BASE = BLUEZ_SERVICE_PATH + "/app/service"

    def __init__(self, bus, index, uuid, primary=True, handle=0, includes=None, name="N/A"):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.handle = int(handle)
        self.includes = list(includes or [])
        self.characteristics = []
        self.name = name
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        props = {
            "UUID": self.uuid,
            "Primary": dbus.Boolean(self.primary),
            "Characteristics": dbus_object_path_array(self.get_characteristic_paths()),
            "Handle": dbus.UInt16(self.handle),
        }
        if self.includes:
            props["Includes"] = dbus_object_path_array(self.includes)
        return {GATT_SERVICE_IFACE: props}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        return [characteristic.get_path() for characteristic in self.characteristics]

    def get_characteristics(self):
        return self.characteristics

    def destroy(self):
        for characteristic in self.characteristics:
            characteristic.destroy()
        self.remove_from_connection()

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service, *, handle=0, value=None, mtu=None, name="N/A"):
        self.path = service.path + "/char" + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = list(flags)
        self.descriptors = []
        self.notifying = False
        self.handle = int(handle)
        self.mtu = None if mtu is None else int(mtu)
        self.value = bytes(value or b"")
        self.name = name
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        props = {
            "Service": self.service.get_path(),
            "UUID": self.uuid,
            "Flags": dbus.Array(self.flags, signature="s"),
            "Descriptors": dbus_object_path_array(self.get_descriptor_paths()),
            "Handle": dbus.UInt16(self.handle),
            "Value": dbus_byte_array(self.value),
            "Notifying": dbus.Boolean(self.notifying),
        }
        if self.mtu is not None:
            props["MTU"] = dbus.UInt16(self.mtu)
        return {GATT_CHRC_IFACE: props}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptor_paths(self):
        return [descriptor.get_path() for descriptor in self.descriptors]

    def get_descriptors(self):
        return self.descriptors

    def destroy(self):
        for descriptor in self.descriptors:
            descriptor.destroy()
        self.remove_from_connection()

    def set_value(self, value, *, emit=False):
        self.value = bytes(value)
        if emit:
            self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": dbus_byte_array(self.value)}, [])

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, options):
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        raise NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Descriptor(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, characteristic, *, handle=0, value=None, name="N/A"):
        self.path = characteristic.path + "/desc" + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = list(flags)
        self.chrc = characteristic
        self.handle = int(handle)
        self.value = None if value is None else bytes(value)
        self.name = name
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        props = {
            "Characteristic": self.chrc.get_path(),
            "UUID": self.uuid,
            "Flags": dbus.Array(self.flags, signature="s"),
            "Handle": dbus.UInt16(self.handle),
        }
        if self.value is not None:
            props["Value"] = dbus_byte_array(self.value)
        return {GATT_DESC_IFACE: props}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def destroy(self):
        self.remove_from_connection()

    def set_value(self, value, *, emit=False):
        self.value = bytes(value)
        if emit:
            self.PropertiesChanged(GATT_DESC_IFACE, {"Value": dbus_byte_array(self.value)}, [])

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_DESC_IFACE]

    @dbus.service.method(GATT_DESC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        raise NotSupportedException()

    @dbus.service.method(GATT_DESC_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, options):
        raise NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


def find_adapter(bus):
    om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE)
    objects = om.GetManagedObjects()
    for path, ifaces in objects.items():
        if GATT_MANAGER_IFACE in ifaces:
            return path
    return None
