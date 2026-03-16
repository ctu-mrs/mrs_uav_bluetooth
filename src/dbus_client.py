"""D-Bus BLE client helpers for discovery, device control, and GATT access."""

import threading
import time
from typing import Callable, Dict, List, Optional

import dbus

from .dbus_common import (
    ADAPTER_IFACE,
    BATTERY_IFACE,
    BLUEZ_SERVICE_NAME,
    DBUS_OM_IFACE,
    DBUS_PROP_IFACE,
    DEVICE_IFACE,
    GATT_CHRC_IFACE,
    GATT_DESC_IFACE,
    GATT_SERVICE_IFACE,
    bytes_from_variant,
    dbus_byte_array,
    dbus_dict,
    decode_byte_dict,
    decode_manufacturer_dict,
)


class DeviceInfo:
    __slots__ = (
        "mac",
        "path",
        "adapter",
        "address_type",
        "name",
        "alias",
        "icon",
        "appearance",
        "rssi",
        "tx_power",
        "pathloss",
        "connected",
        "paired",
        "bonded",
        "trusted",
        "blocked",
        "services_resolved",
        "uuids",
        "manufacturer_data",
        "service_data",
        "last_seen",
    )

    def __init__(self, mac: str, path: str = ""):
        self.mac = mac
        self.path = path
        self.adapter: Optional[str] = None
        self.address_type: Optional[str] = None
        self.name: Optional[str] = None
        self.alias: Optional[str] = None
        self.icon: Optional[str] = None
        self.appearance: Optional[int] = None
        self.rssi: Optional[int] = None
        self.tx_power: Optional[int] = None
        self.pathloss: Optional[int] = None
        self.connected = False
        self.paired = False
        self.bonded = False
        self.trusted = False
        self.blocked = False
        self.services_resolved = False
        self.uuids: List[str] = []
        self.manufacturer_data: Dict[int, bytes] = {}
        self.service_data: Dict[str, bytes] = {}
        self.last_seen = time.time()

    def copy(self):
        duplicate = DeviceInfo(self.mac, self.path)
        for attr in self.__slots__:
            if attr in {"mac", "path", "manufacturer_data", "service_data", "uuids"}:
                continue
            setattr(duplicate, attr, getattr(self, attr))
        duplicate.uuids = list(self.uuids)
        duplicate.manufacturer_data = dict(self.manufacturer_data)
        duplicate.service_data = dict(self.service_data)
        return duplicate


class BleClient:
    _DEVICE_REFRESH_TIMEOUT_S = 2.0
    _SCAN_STATE_GRACE_S = 6.0
    _SCAN_STATE_POLL_INTERVAL_S = 3.0

    def __init__(self, bus: dbus.SystemBus, adapter_path: str):
        self._bus = bus
        self._adapter_path = adapter_path
        self._adapter = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADAPTER_IFACE)
        self._adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), DBUS_PROP_IFACE)
        self._object_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE)
        self._devices: Dict[str, DeviceInfo] = {}
        self._lock = threading.RLock()
        self._scan_running = False
        self._last_scan_request_monotonic = 0.0
        self._last_scan_property_check_monotonic = 0.0
        self._discovering_false_since_monotonic = 0.0
        self._notification_cbs: Dict[int, Callable] = {}
        self._next_ntf_token = 1
        self._gatt_event_cbs: Dict[int, Callable] = {}
        self._next_gatt_token = 1
        self._notify_matches: Dict[str, object] = {}

        self._bus.add_signal_receiver(self._on_interfaces_added, dbus_interface=DBUS_OM_IFACE, signal_name="InterfacesAdded")
        self._bus.add_signal_receiver(self._on_interfaces_removed, dbus_interface=DBUS_OM_IFACE, signal_name="InterfacesRemoved")
        self._bus.add_signal_receiver(
            self._on_properties_changed,
            dbus_interface=DBUS_PROP_IFACE,
            signal_name="PropertiesChanged",
            path_keyword="path",
        )

        self._populate_known_devices()

    @property
    def scanning(self):
        return self.is_scanning()

    def is_scanning(self) -> bool:
        now = time.monotonic()
        should_refresh = (
            not self._scan_running
            or now - self._last_scan_property_check_monotonic >= self._SCAN_STATE_POLL_INTERVAL_S
        )
        if should_refresh:
            self._last_scan_property_check_monotonic = now
            try:
                discovering = bool(self._adapter_props.Get(ADAPTER_IFACE, "Discovering"))
                if discovering:
                    self._scan_running = True
                    self._last_scan_request_monotonic = now
                    self._discovering_false_since_monotonic = 0.0
                else:
                    if self._discovering_false_since_monotonic <= 0.0:
                        self._discovering_false_since_monotonic = now
                    elif now - self._discovering_false_since_monotonic >= self._SCAN_STATE_GRACE_S:
                        self._scan_running = False
            except dbus.DBusException:
                pass
        return self._scan_running

    def refresh_devices(self) -> Dict[str, DeviceInfo]:
        seen_paths = set()
        for path, ifaces in self._managed_objects().items():
            path_str = str(path)
            if not path_str.startswith(self._adapter_path + "/"):
                continue
            props = ifaces.get(DEVICE_IFACE)
            if not props:
                continue
            self._update_device(path_str, props)
            seen_paths.add(path_str)
        with self._lock:
            for device in self._devices.values():
                if device.path and device.path.startswith(self._adapter_path + "/") and device.path not in seen_paths:
                    device.connected = False
                    device.services_resolved = False
            return {mac: device.copy() for mac, device in self._devices.items()}

    def refresh_device(self, mac: str) -> Optional[DeviceInfo]:
        cached = self.get_device(mac)
        if not cached or not cached.path:
            return cached
        try:
            props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, cached.path), DBUS_PROP_IFACE).GetAll(
                DEVICE_IFACE,
                timeout=self._DEVICE_REFRESH_TIMEOUT_S,
            )
        except dbus.DBusException as exc:
            if self._is_transient_refresh_error(exc):
                return cached
            with self._lock:
                device = self._devices.get(mac.upper())
                if device is not None:
                    device.connected = False
                    device.services_resolved = False
                    return device.copy()
            return cached
        self._update_device(cached.path, props)
        return self.get_device(mac)

    def get_adapter_info(self):
        props = self._adapter_props.GetAll(ADAPTER_IFACE)
        return {str(key): props[key] for key in props.keys()}

    def set_adapter_property(self, prop: str, value) -> bool:
        try:
            self._adapter_props.Set(ADAPTER_IFACE, prop, value)
            return True
        except dbus.DBusException:
            return False

    def start_scan(self, transport: str = "le", *, uuids=None, rssi=None, pathloss=None, duplicate_data=None, discoverable=None, pattern=None, auto_connect=None) -> bool:
        filters = {"Transport": dbus.String(transport)}
        if uuids is not None:
            filters["UUIDs"] = dbus.Array([str(uuid) for uuid in uuids], signature="s")
        if rssi is not None:
            filters["RSSI"] = dbus.Int16(int(rssi))
        if pathloss is not None:
            filters["Pathloss"] = dbus.UInt16(int(pathloss))
        if duplicate_data is not None:
            filters["DuplicateData"] = dbus.Boolean(duplicate_data)
        if discoverable is not None:
            filters["Discoverable"] = dbus.Boolean(discoverable)
        if pattern is not None:
            filters["Pattern"] = dbus.String(pattern)
        if auto_connect is not None:
            filters["AutoConnect"] = dbus.Boolean(auto_connect)
        try:
            self._adapter.SetDiscoveryFilter(dbus_dict(filters))
            self._adapter.StartDiscovery()
            self._scan_running = True
            now = time.monotonic()
            self._last_scan_request_monotonic = now
            self._last_scan_property_check_monotonic = now
            self._discovering_false_since_monotonic = 0.0
            return True
        except dbus.DBusException as exc:
            if "InProgress" in str(exc):
                self._scan_running = True
                now = time.monotonic()
                self._last_scan_request_monotonic = now
                self._last_scan_property_check_monotonic = now
                self._discovering_false_since_monotonic = 0.0
                return True
            return False

    def stop_scan(self) -> bool:
        try:
            self._adapter.StopDiscovery()
            self._scan_running = False
            self._last_scan_request_monotonic = 0.0
            self._last_scan_property_check_monotonic = time.monotonic()
            self._discovering_false_since_monotonic = self._last_scan_property_check_monotonic
            return True
        except dbus.DBusException as exc:
            if "NotAuthorized" in str(exc) or "No discovery started" in str(exc):
                self._scan_running = False
                self._last_scan_request_monotonic = 0.0
                self._last_scan_property_check_monotonic = time.monotonic()
                self._discovering_false_since_monotonic = self._last_scan_property_check_monotonic
                return True
            return False

    def get_devices(self, refresh: bool = False) -> Dict[str, DeviceInfo]:
        if refresh:
            return self.refresh_devices()
        with self._lock:
            return {mac: device.copy() for mac, device in self._devices.items()}

    def get_device(self, mac: str, refresh: bool = False) -> Optional[DeviceInfo]:
        if refresh:
            return self.refresh_device(mac)
        with self._lock:
            device = self._devices.get(mac.upper())
            return device.copy() if device else None

    def get_connected_devices(self, refresh: bool = False) -> Dict[str, DeviceInfo]:
        devices = self.refresh_devices() if refresh else self.get_devices()
        return {mac: device for mac, device in devices.items() if device.connected}

    def wait_services_resolved(self, mac: str, timeout: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout
        next_refresh = 0.0
        while time.monotonic() < deadline:
            device = self.get_device(mac, refresh=False)
            if device and device.services_resolved:
                return True
            now = time.monotonic()
            if now >= next_refresh:
                device = self.refresh_device(mac)
                next_refresh = now + 1.0
                if device and device.services_resolved:
                    return True
                # BlueZ may never set ServicesResolved=True even though the
                # GATT hierarchy is populated.  Check for actual GATT objects.
                if device and device.path:
                    chars = self.list_characteristics(mac)
                    if chars:
                        return True
            time.sleep(0.3)
        return False

    def connect(self, mac: str, timeout: float = 15.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return False
        try:
            # Bound the DBus method call itself so this helper cannot block indefinitely.
            device.Connect(timeout=max(1.0, min(float(timeout), 6.0)))
        except dbus.DBusException as exc:
            message = str(exc)
            if (
                "Already Connected" not in message
                and "AlreadyConnected" not in message
                and "InProgress" not in message
            ):
                return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self.get_device(mac, refresh=True)
            if current and current.connected:
                return True
            time.sleep(0.3)
        return False

    def connect_async(self, mac: str, timeout: float = 6.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return False
        try:
            device.Connect(
                reply_handler=lambda *_: None,
                error_handler=lambda *_: None,
                timeout=max(1.0, float(timeout)),
            )
            return True
        except dbus.DBusException as exc:
            message = str(exc)
            if (
                "Already Connected" in message
                or "AlreadyConnected" in message
                or "InProgress" in message
            ):
                return True
            return False

    def disconnect(self, mac: str, timeout: float = 10.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return True
        try:
            device.Disconnect()
        except dbus.DBusException:
            pass
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self.get_device(mac, refresh=True)
            if current and not current.connected:
                return True
            time.sleep(0.3)
        return False

    def pair(self, mac: str, timeout: float = 30.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return False
        try:
            # Keep Pair() bounded so one problematic peer cannot block the caller indefinitely.
            device.Pair(timeout=max(2.0, min(float(timeout), 8.0)))
        except dbus.DBusException as exc:
            message = str(exc)
            if (
                "AlreadyExists" not in message
                and "Already Paired" not in message
                and "AlreadyPaired" not in message
                and "InProgress" not in message
            ):
                return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self.get_device(mac, refresh=True)
            if current and current.paired:
                return True
            time.sleep(0.5)
        return False

    def pair_async(self, mac: str, timeout: float = 10.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return False
        try:
            device.Pair(
                reply_handler=lambda *_: None,
                error_handler=lambda *_: None,
                timeout=max(2.0, min(float(timeout), 12.0)),
            )
            return True
        except dbus.DBusException as exc:
            message = str(exc)
            if (
                "AlreadyExists" in message
                or "Already Paired" in message
                or "AlreadyPaired" in message
                or "InProgress" in message
            ):
                return True
            return False

    def trust(self, mac: str) -> bool:
        return self._set_device_prop(mac, "Trusted", dbus.Boolean(True))

    def untrust(self, mac: str) -> bool:
        return self._set_device_prop(mac, "Trusted", dbus.Boolean(False))

    def remove(self, mac: str) -> bool:
        device = self.get_device(mac)
        if not device or not device.path:
            return False
        try:
            self._adapter.RemoveDevice(dbus.ObjectPath(device.path))
            with self._lock:
                self._devices.pop(mac.upper(), None)
            return True
        except dbus.DBusException:
            return False

    def list_services(self, mac: str) -> List[dict]:
        device = self.get_device(mac)
        if not device or not device.path:
            return []
        services = []
        for path, ifaces in self._managed_objects().items():
            if not str(path).startswith(device.path + "/"):
                continue
            props = ifaces.get(GATT_SERVICE_IFACE)
            if not props:
                continue
            services.append({
                "path": str(path),
                "uuid": str(props.get("UUID", "")),
                "primary": bool(props.get("Primary", True)),
                "device": str(props.get("Device", "")) if "Device" in props else "",
                "includes": [str(item) for item in props.get("Includes", [])],
            })
        return services

    def list_characteristics(self, mac: str) -> List[dict]:
        device = self.get_device(mac)
        if not device or not device.path:
            return []
        characteristics = []
        for path, ifaces in self._managed_objects().items():
            if not str(path).startswith(device.path + "/"):
                continue
            props = ifaces.get(GATT_CHRC_IFACE)
            if not props:
                continue
            characteristics.append({
                "path": str(path),
                "service": str(props.get("Service", "")),
                "uuid": str(props.get("UUID", "")),
                "flags": [str(flag) for flag in props.get("Flags", [])],
                "notifying": bool(props.get("Notifying", False)),
                "mtu": int(props["MTU"]) if "MTU" in props else 0,
            })
        return characteristics

    def list_descriptors(self, mac: str, chrc_path: Optional[str] = None) -> List[dict]:
        device = self.get_device(mac)
        if not device or not device.path:
            return []
        descriptors = []
        prefix = chrc_path + "/" if chrc_path else device.path + "/"
        for path, ifaces in self._managed_objects().items():
            if not str(path).startswith(prefix):
                continue
            props = ifaces.get(GATT_DESC_IFACE)
            if not props:
                continue
            descriptors.append({
                "path": str(path),
                "characteristic": str(props.get("Characteristic", "")),
                "uuid": str(props.get("UUID", "")),
                "flags": [str(flag) for flag in props.get("Flags", [])],
            })
        return descriptors

    def find_characteristic(self, mac: str, uuid: str) -> Optional[str]:
        uuid_lower = uuid.lower()
        for characteristic in self.list_characteristics(mac):
            if characteristic["uuid"].lower() == uuid_lower:
                return characteristic["path"]
        return None

    def find_characteristics(self, mac: str, uuid: str) -> List[str]:
        uuid_lower = uuid.lower()
        candidates = []
        for characteristic in self.list_characteristics(mac):
            if characteristic["uuid"].lower() != uuid_lower:
                continue
            path = characteristic["path"]
            if path in candidates:
                continue
            candidates.append(path)
        return candidates

    def find_descriptor(self, mac: str, uuid: str, chrc_path: Optional[str] = None) -> Optional[str]:
        uuid_lower = uuid.lower()
        for descriptor in self.list_descriptors(mac, chrc_path=chrc_path):
            if descriptor["uuid"].lower() == uuid_lower:
                return descriptor["path"]
        return None

    def read_characteristic(self, chrc_path: str) -> Optional[bytes]:
        try:
            characteristic = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE)
            value = bytes_from_variant(characteristic.ReadValue({}))
            self._emit_gatt("client_read", chrc_path=chrc_path, value=value)
            return value
        except dbus.DBusException as exc:
            self._emit_gatt("client_read_failed", chrc_path=chrc_path, error=str(exc))
            return None

    def write_characteristic(self, chrc_path: str, data: bytes, with_response=True) -> bool:
        try:
            characteristic = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE)
            options = {"type": dbus.String("request" if with_response else "command")}
            characteristic.WriteValue(dbus_byte_array(data), dbus_dict(options))
            self._emit_gatt("client_write", chrc_path=chrc_path, value=data, with_response=with_response)
            return True
        except dbus.DBusException as exc:
            self._emit_gatt("client_write_failed", chrc_path=chrc_path, error=str(exc), with_response=with_response)
            return False

    def write_characteristic_async(self, chrc_path: str, data: bytes, with_response=True, timeout: float = 6.0) -> bool:
        try:
            characteristic = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE)
            options = {"type": dbus.String("request" if with_response else "command")}

            def _reply_handler(*_):
                self._emit_gatt("client_write", chrc_path=chrc_path, value=data, with_response=with_response)

            def _error_handler(error):
                self._emit_gatt("client_write_failed", chrc_path=chrc_path, error=str(error), with_response=with_response)

            characteristic.WriteValue(
                dbus_byte_array(data),
                dbus_dict(options),
                reply_handler=_reply_handler,
                error_handler=_error_handler,
                timeout=timeout,
            )
            return True
        except dbus.DBusException as exc:
            self._emit_gatt("client_write_failed", chrc_path=chrc_path, error=str(exc), with_response=with_response)
            return False

    def read_descriptor(self, desc_path: str) -> Optional[bytes]:
        try:
            descriptor = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, desc_path), GATT_DESC_IFACE)
            value = bytes_from_variant(descriptor.ReadValue({}))
            self._emit_gatt("client_descriptor_read", desc_path=desc_path, value=value)
            return value
        except dbus.DBusException as exc:
            self._emit_gatt("client_descriptor_read_failed", desc_path=desc_path, error=str(exc))
            return None

    def write_descriptor(self, desc_path: str, data: bytes) -> bool:
        try:
            descriptor = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, desc_path), GATT_DESC_IFACE)
            descriptor.WriteValue(dbus_byte_array(data), dbus_dict({}))
            self._emit_gatt("client_descriptor_write", desc_path=desc_path, value=data)
            return True
        except dbus.DBusException as exc:
            self._emit_gatt("client_descriptor_write_failed", desc_path=desc_path, error=str(exc))
            return False

    def write_descriptor_async(self, desc_path: str, data: bytes, timeout: float = 6.0) -> bool:
        try:
            descriptor = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, desc_path), GATT_DESC_IFACE)

            def _reply_handler(*_):
                self._emit_gatt("client_descriptor_write", desc_path=desc_path, value=data)

            def _error_handler(error):
                self._emit_gatt("client_descriptor_write_failed", desc_path=desc_path, error=str(error))

            descriptor.WriteValue(
                dbus_byte_array(data),
                dbus_dict({}),
                reply_handler=_reply_handler,
                error_handler=_error_handler,
                timeout=timeout,
            )
            return True
        except dbus.DBusException as exc:
            self._emit_gatt("client_descriptor_write_failed", desc_path=desc_path, error=str(exc))
            return False

    def _ensure_notify_match(self, chrc_path: str):
        if chrc_path in self._notify_matches:
            return
        chrc_obj = self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path)
        props_iface = dbus.Interface(chrc_obj, DBUS_PROP_IFACE)
        self._notify_matches[chrc_path] = props_iface.connect_to_signal(
            "PropertiesChanged",
            lambda iface, changed, invalidated: self._on_chrc_notify(chrc_path, iface, changed, invalidated),
        )

    def start_notify(self, chrc_path: str) -> bool:
        try:
            chrc_obj = self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path)
            characteristic = dbus.Interface(chrc_obj, GATT_CHRC_IFACE)
            characteristic.StartNotify()
            self._ensure_notify_match(chrc_path)
            self._emit_gatt("client_notify_enabled", chrc_path=chrc_path)
            return True
        except dbus.DBusException as exc:
            message = str(exc)
            if "InProgress" in message or "Already notifying" in message or "AlreadyNotifying" in message:
                try:
                    self._ensure_notify_match(chrc_path)
                except Exception:
                    pass
                self._emit_gatt("client_notify_enabled", chrc_path=chrc_path, recovered_from=message)
                return True
            self._emit_gatt("client_notify_failed", chrc_path=chrc_path, error=str(exc))
            return False

    def stop_notify(self, chrc_path: str) -> bool:
        try:
            characteristic = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE)
            characteristic.StopNotify()
            match = self._notify_matches.pop(chrc_path, None)
            if match is not None:
                match.remove()
            self._emit_gatt("client_notify_disabled", chrc_path=chrc_path)
            return True
        except dbus.DBusException as exc:
            self._emit_gatt("client_notify_failed", chrc_path=chrc_path, error=str(exc))
            return False

    def add_notification_handler(self, cb: Callable) -> int:
        token = self._next_ntf_token
        self._next_ntf_token += 1
        self._notification_cbs[token] = cb
        return token

    def add_gatt_event_handler(self, cb: Callable) -> int:
        token = self._next_gatt_token
        self._next_gatt_token += 1
        self._gatt_event_cbs[token] = cb
        return token

    def _managed_objects(self):
        return self._object_manager.GetManagedObjects()

    def _populate_known_devices(self):
        for path, ifaces in self._managed_objects().items():
            props = ifaces.get(DEVICE_IFACE)
            if not props or not str(path).startswith(self._adapter_path + "/"):
                continue
            self._update_device(str(path), props)

    def _on_interfaces_added(self, path, interfaces):
        if DEVICE_IFACE in interfaces:
            self._update_device(str(path), interfaces[DEVICE_IFACE])

    def _on_interfaces_removed(self, path, interfaces):
        if DEVICE_IFACE not in interfaces:
            return
        with self._lock:
            stale = None
            for mac, device in self._devices.items():
                if device.path == str(path):
                    stale = mac
                    break
            if stale:
                self._devices.pop(stale, None)

    def _on_properties_changed(self, interface, changed, invalidated, path=""):
        del invalidated
        if interface == ADAPTER_IFACE and str(path) == self._adapter_path:
            if "Discovering" in changed:
                discovering = bool(changed.get("Discovering"))
                now = time.monotonic()
                if discovering:
                    self._scan_running = True
                    self._last_scan_request_monotonic = now
                    self._discovering_false_since_monotonic = 0.0
                else:
                    if self._discovering_false_since_monotonic <= 0.0:
                        self._discovering_false_since_monotonic = now
                    elif now - self._discovering_false_since_monotonic >= self._SCAN_STATE_GRACE_S:
                        self._scan_running = False
            return
        if interface == DEVICE_IFACE:
            self._update_device(str(path), changed)

    def _update_device(self, path: str, props: dict):
        mac = str(props.get("Address", "")).upper()
        if not mac:
            with self._lock:
                for device in self._devices.values():
                    if device.path == path:
                        mac = device.mac
                        break
        if not mac:
            return
        with self._lock:
            device = self._devices.get(mac)
            if device is None:
                device = DeviceInfo(mac, path)
                self._devices[mac] = device
            if not device.path:
                device.path = path
            device.last_seen = time.time()
            if "Adapter" in props:
                device.adapter = str(props["Adapter"])
            if "AddressType" in props:
                device.address_type = str(props["AddressType"])
            if "Name" in props:
                device.name = str(props["Name"])
            if "Alias" in props:
                device.alias = str(props["Alias"])
            if "Icon" in props:
                device.icon = str(props["Icon"])
            if "Appearance" in props:
                device.appearance = int(props["Appearance"])
            if "RSSI" in props:
                device.rssi = int(props["RSSI"])
            if "TxPower" in props:
                device.tx_power = int(props["TxPower"])
            if "Pathloss" in props:
                device.pathloss = int(props["Pathloss"])
            if "Connected" in props:
                device.connected = bool(props["Connected"])
            if "Paired" in props:
                device.paired = bool(props["Paired"])
            if "Bonded" in props:
                device.bonded = bool(props["Bonded"])
            if "Trusted" in props:
                device.trusted = bool(props["Trusted"])
            if "Blocked" in props:
                device.blocked = bool(props["Blocked"])
            if "ServicesResolved" in props:
                device.services_resolved = bool(props["ServicesResolved"])
            if "UUIDs" in props:
                device.uuids = [str(item) for item in props["UUIDs"]]
            if "ManufacturerData" in props:
                device.manufacturer_data = decode_manufacturer_dict(props["ManufacturerData"])
            if "ServiceData" in props:
                device.service_data = decode_byte_dict(props["ServiceData"])

    def _find_device_obj(self, mac: str):
        device = self.get_device(mac)
        if not device or not device.path:
            return None
        try:
            return dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device.path), DEVICE_IFACE)
        except dbus.DBusException:
            return None

    def _set_device_prop(self, mac: str, prop: str, value) -> bool:
        device = self.get_device(mac)
        if not device or not device.path:
            return False
        try:
            props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device.path), DBUS_PROP_IFACE)
            props.Set(DEVICE_IFACE, prop, value)
            return True
        except dbus.DBusException:
            return False

    def _is_transient_refresh_error(self, exc: dbus.DBusException) -> bool:
        message = str(exc)
        transient_markers = (
            "NoReply",
            "Timed out",
            "Timeout was reached",
            "Did not receive a reply",
            "org.freedesktop.DBus.Error.NoReply",
        )
        return any(marker in message for marker in transient_markers)

    def _on_chrc_notify(self, chrc_path, interface, changed, invalidated):
        del invalidated
        if interface != GATT_CHRC_IFACE:
            return
        value_variant = changed.get("Value")
        if value_variant is None:
            return
        data = bytes_from_variant(value_variant)
        uuid = self._resolve_uuid(chrc_path, GATT_CHRC_IFACE)
        for cb in list(self._notification_cbs.values()):
            try:
                cb(data, uuid, chrc_path)
            except Exception:
                continue

    def _resolve_uuid(self, path: str, interface: str) -> str:
        try:
            props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, path), DBUS_PROP_IFACE)
            return str(props.Get(interface, "UUID"))
        except Exception:
            return ""

    def _emit_gatt(self, event_type: str, **kw):
        for cb in list(self._gatt_event_cbs.values()):
            try:
                cb(event_type, kw)
            except Exception:
                continue
