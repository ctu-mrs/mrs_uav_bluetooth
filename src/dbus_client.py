"""D-Bus BLE client helpers for discovery, device control, and GATT access."""

import threading
import time
from typing import TYPE_CHECKING, Callable, Dict, List, Optional

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

if TYPE_CHECKING:
    from .bluetooth_dbus_runtime import SerializedDbusQueue


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
    def __init__(self, bus: dbus.SystemBus, adapter_path: str, dbus_queue: Optional["SerializedDbusQueue"] = None):
        self._bus = bus
        self._adapter_path = adapter_path
        self._adapter = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADAPTER_IFACE)
        self._adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), DBUS_PROP_IFACE)
        self._object_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE)
        self._dbus_queue = dbus_queue
        self._devices: Dict[str, DeviceInfo] = {}
        self._lock = threading.RLock()
        self._scan_running = False
        self._notification_cbs: Dict[int, Callable] = {}
        self._next_ntf_token = 1
        self._gatt_event_cbs: Dict[int, Callable] = {}
        self._next_gatt_token = 1
        self._device_event_cbs: Dict[int, Callable] = {}
        self._next_device_token = 1
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
        return self._scan_running

    def refresh_devices(self) -> Dict[str, DeviceInfo]:
        seen_paths = set()
        stale_events = []
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
                    changed_fields = []
                    if device.connected:
                        changed_fields.append("connected")
                    device.connected = False
                    if device.services_resolved:
                        changed_fields.append("services_resolved")
                    device.services_resolved = False
                    if changed_fields:
                        stale_events.append((device.mac, device.copy(), tuple(changed_fields)))
            snapshot = {mac: device.copy() for mac, device in self._devices.items()}
        for mac, device, changed_fields in stale_events:
            self._emit_device(mac, device, changed_fields)
        return snapshot

    def refresh_device(self, mac: str) -> Optional[DeviceInfo]:
        cached = self.get_device(mac)
        if not cached or not cached.path:
            return cached
        try:
            props = self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, cached.path), DBUS_PROP_IFACE).GetAll(DEVICE_IFACE),
                f"refresh device {mac}",
            )
        except dbus.DBusException:
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
        props = self._run_serialized(lambda: self._adapter_props.GetAll(ADAPTER_IFACE), "get adapter info", timeout=10.0)
        return {str(key): props[key] for key in props.keys()}

    def set_adapter_property(self, prop: str, value) -> bool:
        try:
            self._run_serialized(
                lambda: self._adapter_props.Set(ADAPTER_IFACE, prop, value),
                f"set adapter property {prop}",
                timeout=10.0,
            )
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
            self._run_serialized(
                lambda: (
                    self._adapter.SetDiscoveryFilter(dbus_dict(filters)),
                    self._adapter.StartDiscovery(),
                ),
                "start BLE discovery",
                timeout=10.0,
            )
            self._scan_running = True
            return True
        except dbus.DBusException as exc:
            if "InProgress" in str(exc):
                self._scan_running = True
                return True
            return False

    def stop_scan(self) -> bool:
        try:
            self._run_serialized(self._adapter.StopDiscovery, "stop BLE discovery", timeout=10.0)
            self._scan_running = False
            return True
        except dbus.DBusException as exc:
            if "NotAuthorized" in str(exc) or "No discovery started" in str(exc):
                self._scan_running = False
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
        deadline = time.time() + timeout
        while time.time() < deadline:
            device = self.get_device(mac, refresh=True)
            if device and device.services_resolved:
                return True
            time.sleep(0.3)
        return False

    def wait_for_descriptors(
        self,
        mac: str,
        *,
        descriptor_uuids: Optional[List[str]] = None,
        chrc_path: Optional[str] = None,
        timeout: float = 5.0,
        settle_delay_s: float = 0.2,
    ) -> bool:
        expected = {str(item).lower() for item in (descriptor_uuids or []) if str(item).strip()}
        deadline = time.time() + max(0.1, float(timeout))
        if settle_delay_s > 0:
            time.sleep(float(settle_delay_s))
        while time.time() < deadline:
            descriptors = self.list_descriptors(mac, chrc_path=chrc_path)
            if expected:
                available = {str(item.get("uuid", "")).lower() for item in descriptors}
                if expected.issubset(available):
                    return True
            elif descriptors:
                return True
            self.get_device(mac, refresh=True)
            time.sleep(0.25)
        descriptors = self.list_descriptors(mac, chrc_path=chrc_path)
        if expected:
            available = {str(item.get("uuid", "")).lower() for item in descriptors}
            return expected.issubset(available)
        return bool(descriptors)

    def connect(self, mac: str, timeout: float = 15.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return False
        try:
            # Bound the DBus method call itself so this helper cannot block indefinitely.
            call_timeout = max(1.0, min(float(timeout), 6.0))
            self._run_serialized(
                lambda: device.Connect(timeout=call_timeout),
                f"connect {mac}",
                timeout=call_timeout,
            )
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
            self._run_async_method(device.Connect, f"connect {mac}", timeout=max(1.0, float(timeout)))
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
            self._run_serialized(device.Disconnect, f"disconnect {mac}", timeout=max(1.0, float(timeout)))
        except dbus.DBusException:
            pass
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self.get_device(mac, refresh=True)
            if current and not current.connected:
                return True
            time.sleep(0.3)
        return False

    def disconnect_async(self, mac: str) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return True
        try:
            self._run_async_method(device.Disconnect, f"disconnect {mac}")
            return True
        except dbus.DBusException:
            return False

    def pair(self, mac: str, timeout: float = 30.0) -> bool:
        device = self._find_device_obj(mac)
        if device is None:
            return False
        try:
            # Keep Pair() bounded so one problematic peer cannot block the caller indefinitely.
            call_timeout = max(2.0, min(float(timeout), 8.0))
            self._run_serialized(
                lambda: device.Pair(timeout=call_timeout),
                f"pair {mac}",
                timeout=call_timeout,
            )
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
            self._run_async_method(
                device.Pair,
                f"pair {mac}",
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
            self._run_serialized(
                lambda: self._adapter.RemoveDevice(dbus.ObjectPath(device.path)),
                f"remove device {mac}",
                timeout=10.0,
            )
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
            value = self._run_serialized(
                lambda: bytes_from_variant(
                    dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE).ReadValue({})
                ),
                f"read characteristic {chrc_path}",
            )
            self._emit_gatt("client_read", chrc_path=chrc_path, value=value)
            return value
        except dbus.DBusException as exc:
            self._emit_gatt("client_read_failed", chrc_path=chrc_path, error=str(exc))
            return None

    def write_characteristic(self, chrc_path: str, data: bytes, with_response=True) -> bool:
        try:
            options = {"type": dbus.String("request" if with_response else "command")}
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE).WriteValue(
                    dbus_byte_array(data),
                    dbus_dict(options),
                ),
                f"write characteristic {chrc_path}",
            )
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

            self._run_async_method(
                characteristic.WriteValue,
                f"write characteristic {chrc_path}",
                dbus_byte_array(data),
                dbus_dict(options),
                timeout=timeout,
                on_reply=_reply_handler,
                on_error=_error_handler,
            )
            return True
        except dbus.DBusException as exc:
            self._emit_gatt("client_write_failed", chrc_path=chrc_path, error=str(exc), with_response=with_response)
            return False

    def read_descriptor(self, desc_path: str) -> Optional[bytes]:
        try:
            value = self._run_serialized(
                lambda: bytes_from_variant(
                    dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, desc_path), GATT_DESC_IFACE).ReadValue({})
                ),
                f"read descriptor {desc_path}",
            )
            self._emit_gatt("client_descriptor_read", desc_path=desc_path, value=value)
            return value
        except dbus.DBusException as exc:
            self._emit_gatt("client_descriptor_read_failed", desc_path=desc_path, error=str(exc))
            return None

    def write_descriptor(self, desc_path: str, data: bytes) -> bool:
        try:
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, desc_path), GATT_DESC_IFACE).WriteValue(
                    dbus_byte_array(data),
                    dbus_dict({}),
                ),
                f"write descriptor {desc_path}",
            )
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

            self._run_async_method(
                descriptor.WriteValue,
                f"write descriptor {desc_path}",
                dbus_byte_array(data),
                dbus_dict({}),
                timeout=timeout,
                on_reply=_reply_handler,
                on_error=_error_handler,
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
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE).StartNotify(),
                f"start notify {chrc_path}",
            )
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
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE).StopNotify(),
                f"stop notify {chrc_path}",
            )
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

    def add_device_handler(self, cb: Callable) -> int:
        token = self._next_device_token
        self._next_device_token += 1
        self._device_event_cbs[token] = cb
        return token

    def _managed_objects(self):
        return self._run_serialized(self._object_manager.GetManagedObjects, "read managed objects", timeout=10.0)

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
        removed = None
        with self._lock:
            for mac, device in self._devices.items():
                if device.path == str(path):
                    removed = device.copy()
                    break
            if removed is not None:
                self._devices.pop(removed.mac, None)
        if removed is not None:
            self._emit_device(removed.mac, removed, ("removed", "connected", "services_resolved"))

    def _on_properties_changed(self, interface, changed, invalidated, path=""):
        del invalidated
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
        observed_fields = (
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
        )
        with self._lock:
            device = self._devices.get(mac)
            is_new = device is None
            if device is None:
                device = DeviceInfo(mac, path)
                self._devices[mac] = device
            previous = {field: getattr(device, field) for field in observed_fields}
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
            changed_fields = [field for field in observed_fields if getattr(device, field) != previous[field]]
            snapshot = device.copy()
        if is_new:
            changed_fields.insert(0, "added")
        if changed_fields:
            self._emit_device(mac, snapshot, tuple(changed_fields))

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
            self._run_serialized(lambda: props.Set(DEVICE_IFACE, prop, value), f"set {prop} for {mac}", timeout=10.0)
            return True
        except dbus.DBusException:
            return False

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
            return str(
                self._run_serialized(
                    lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, path), DBUS_PROP_IFACE).Get(interface, "UUID"),
                    f"resolve UUID for {path}",
                    timeout=10.0,
                )
            )
        except Exception:
            return ""

    def _run_serialized(self, func: Callable[[], object], operation: str, *, timeout: float = 30.0):
        if self._dbus_queue is None:
            return func()
        try:
            reply = self._dbus_queue.submit_call(func, operation, timeout=timeout, wait=True)
        except TimeoutError:
            return func()
        if not reply:
            return None
        if len(reply) == 1:
            return reply[0]
        return reply

    def _run_async_method(self, method, operation: str, *args, timeout: float = 30.0, on_reply=None, on_error=None):
        if self._dbus_queue is None:
            method(
                *args,
                reply_handler=on_reply or (lambda *_: None),
                error_handler=on_error or (lambda *_: None),
                timeout=timeout,
            )
            return True
        return self._dbus_queue.submit_async_method(
            method,
            operation,
            *args,
            timeout=timeout,
            wait=False,
            on_reply=on_reply,
            on_error=on_error,
        )

    def _emit_gatt(self, event_type: str, **kw):
        for cb in list(self._gatt_event_cbs.values()):
            try:
                cb(event_type, kw)
            except Exception:
                continue

    def _emit_device(self, mac: str, device: DeviceInfo, changed_fields):
        for cb in list(self._device_event_cbs.values()):
            try:
                cb(mac, device.copy(), changed_fields)
            except Exception:
                continue
