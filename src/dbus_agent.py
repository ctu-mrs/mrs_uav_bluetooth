"""D-Bus Bluetooth pairing agent."""

import dbus
import dbus.service

from .dbus_common import BLUEZ_SERVICE_NAME, DBUS_PROP_IFACE

AGENT_INTERFACE = "org.bluez.Agent1"
AGENT_PATH = "/org/bluez/app/agent"


class Rejected(dbus.DBusException):
    _dbus_error_name = "org.bluez.Error.Rejected"


class PairingAgent(dbus.service.Object):
    def __init__(self, bus, auto_accept=False, auto_trust=True, on_event=None):
        self.bus = bus
        self._auto_accept = auto_accept
        self._auto_trust = auto_trust
        self._on_event = on_event
        dbus.service.Object.__init__(self, bus, AGENT_PATH)

    def _emit(self, event_type, device_path="", **kw):
        if self._on_event:
            try:
                self._on_event(event_type, device_path, **kw)
            except Exception:
                pass

    def _set_trusted(self, path):
        props = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, path), DBUS_PROP_IFACE)
        props.Set("org.bluez.Device1", "Trusted", True)

    @dbus.service.method(AGENT_INTERFACE, in_signature="", out_signature="")
    def Release(self):
        self._emit("agent_release")

    @dbus.service.method(AGENT_INTERFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        self._emit("authorize_service", device, uuid=uuid)
        if self._auto_accept:
            return
        raise Rejected("Connection rejected")

    @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        self._emit("request_pin", device)
        if self._auto_trust:
            self._set_trusted(device)
        return ""

    @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        self._emit("request_passkey", device)
        if self._auto_trust:
            self._set_trusted(device)
        return dbus.UInt32(0)

    @dbus.service.method(AGENT_INTERFACE, in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        self._emit("display_passkey", device, passkey=passkey, entered=entered)

    @dbus.service.method(AGENT_INTERFACE, in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        self._emit("display_pin", device, pincode=pincode)

    @dbus.service.method(AGENT_INTERFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        self._emit("request_confirmation", device, passkey=passkey)
        if self._auto_accept:
            if self._auto_trust:
                self._set_trusted(device)
            return
        raise Rejected("Passkey not confirmed")

    @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        self._emit("request_authorization", device)
        if self._auto_accept:
            return
        raise Rejected("Authorization rejected")

    @dbus.service.method(AGENT_INTERFACE, in_signature="", out_signature="")
    def Cancel(self):
        self._emit("cancel")
