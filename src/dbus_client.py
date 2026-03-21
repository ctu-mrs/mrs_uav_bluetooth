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
    def __init__(self, bus: dbus.SystemBus, adapter_path: str):
        self._bus = bus
        self._adapter_path = adapter_path
        self._adapter = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADAPTER_IFACE)
        self._adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), DBUS_PROP_IFACE)
        self._object_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE)
        self._devices: Dict[str, DeviceInfo] = {}
        self._gatt_services: Dict[str, Dict[str, dict]] = {}
        self._gatt_characteristics: Dict[str, Dict[str, dict]] = {}
        self._gatt_descriptors: Dict[str, Dict[str, dict]] = {}
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
        device_path = self._find_device_path(mac)
        if not device_path:
            return False
        try:
            # Bound the DBus method call itself so this helper cannot block indefinitely.
            call_timeout = max(1.0, min(float(timeout), 6.0))
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device_path), DEVICE_IFACE).Connect(
                    timeout=call_timeout
                ),
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
        device_path = self._find_device_path(mac)
        if not device_path:
            return False
        try:
            self._run_async_method(
                dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device_path), DEVICE_IFACE).Connect,
                f"connect {mac}",
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
        device_path = self._find_device_path(mac)
        if not device_path:
            return True
        try:
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device_path), DEVICE_IFACE).Disconnect(),
                f"disconnect {mac}",
                timeout=max(1.0, float(timeout)),
            )
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
        device_path = self._find_device_path(mac)
        if not device_path:
            return True
        try:
            self._run_async_method(
                dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device_path), DEVICE_IFACE).Disconnect,
                f"disconnect {mac}",
            )
            return True
        except dbus.DBusException:
            return False

    def pair(self, mac: str, timeout: float = 30.0) -> bool:
        device_path = self._find_device_path(mac)
        if not device_path:
            return False
        try:
            # Keep Pair() bounded so one problematic peer cannot block the caller indefinitely.
            call_timeout = max(2.0, min(float(timeout), 8.0))
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device_path), DEVICE_IFACE).Pair(
                    timeout=call_timeout
                ),
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
        device_path = self._find_device_path(mac)
        if not device_path:
            return False
        try:
            self._run_async_method(
                dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device_path), DEVICE_IFACE).Pair,
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
        self._ensure_gatt_cache(device)
        with self._lock:
            services = list(self._gatt_services.get(device.path, {}).values())
        return sorted((dict(item) for item in services), key=lambda item: item["path"])

    def list_characteristics(self, mac: str) -> List[dict]:
        device = self.get_device(mac)
        if not device or not device.path:
            return []
        self._ensure_gatt_cache(device)
        with self._lock:
            characteristics = list(self._gatt_characteristics.get(device.path, {}).values())
        return sorted((dict(item) for item in characteristics), key=lambda item: item["path"])

    def list_descriptors(self, mac: str, chrc_path: Optional[str] = None) -> List[dict]:
        device = self.get_device(mac)
        if not device or not device.path:
            return []
        self._ensure_gatt_cache(device)
        with self._lock:
            descriptors = list(self._gatt_descriptors.get(device.path, {}).values())
        if chrc_path:
            descriptors = [item for item in descriptors if item["characteristic"] == chrc_path]
        return sorted((dict(item) for item in descriptors), key=lambda item: item["path"])

    def find_service(self, mac: str, uuid: str) -> Optional[str]:
        uuid_lower = uuid.lower()
        services = self.list_services(mac)
        for service in services:
            if service["uuid"].lower() == uuid_lower:
                return service["path"]
        device = self.get_device(mac)
        if device and device.services_resolved:
            self._refresh_gatt_for_device(device.path)
            for service in self.list_services(mac):
                if service["uuid"].lower() == uuid_lower:
                    return service["path"]
        return None

    def find_characteristic(self, mac: str, uuid: str) -> Optional[str]:
        matches = self.find_characteristics(mac, uuid)
        return matches[0] if matches else None

    def find_characteristics(
        self,
        mac: str,
        uuid: str,
        *,
        service_path: Optional[str] = None,
        service_uuid: Optional[str] = None,
    ) -> List[str]:
        uuid_lower = uuid.lower()
        service_uuid_lower = service_uuid.lower() if service_uuid else ""

        def _collect_candidates() -> List[str]:
            candidates = []
            for characteristic in self.list_characteristics(mac):
                if characteristic["uuid"].lower() != uuid_lower:
                    continue
                if service_path and characteristic["service"] != service_path:
                    continue
                if service_uuid_lower:
                    owning_service = self._get_cached_service_by_path(mac, characteristic["service"])
                    if owning_service is None or owning_service["uuid"].lower() != service_uuid_lower:
                        continue
                path = characteristic["path"]
                if path in candidates:
                    continue
                candidates.append(path)
            return candidates

        candidates = _collect_candidates()
        if candidates:
            return candidates
        device = self.get_device(mac)
        if device and device.services_resolved:
            self._refresh_gatt_for_device(device.path)
            return _collect_candidates()
        return []

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
            options = {"type": dbus.String("request" if with_response else "command")}

            def _reply_handler(*_):
                self._emit_gatt("client_write", chrc_path=chrc_path, value=data, with_response=with_response)

            def _error_handler(error):
                self._emit_gatt("client_write_failed", chrc_path=chrc_path, error=str(error), with_response=with_response)

            self._run_async_method(
                dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, chrc_path), GATT_CHRC_IFACE).WriteValue,
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
            def _reply_handler(*_):
                self._emit_gatt("client_descriptor_write", desc_path=desc_path, value=data)

            def _error_handler(error):
                self._emit_gatt("client_descriptor_write_failed", desc_path=desc_path, error=str(error))

            self._run_async_method(
                dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, desc_path), GATT_DESC_IFACE).WriteValue,
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
                if GATT_SERVICE_IFACE in ifaces:
                    self._update_gatt_service(str(path), ifaces[GATT_SERVICE_IFACE])
                if GATT_CHRC_IFACE in ifaces:
                    self._update_gatt_characteristic(str(path), ifaces[GATT_CHRC_IFACE])
                if GATT_DESC_IFACE in ifaces:
                    self._update_gatt_descriptor(str(path), ifaces[GATT_DESC_IFACE])
                continue
            self._update_device(str(path), props)
            if GATT_SERVICE_IFACE in ifaces:
                self._update_gatt_service(str(path), ifaces[GATT_SERVICE_IFACE])
            if GATT_CHRC_IFACE in ifaces:
                self._update_gatt_characteristic(str(path), ifaces[GATT_CHRC_IFACE])
            if GATT_DESC_IFACE in ifaces:
                self._update_gatt_descriptor(str(path), ifaces[GATT_DESC_IFACE])

    def _on_interfaces_added(self, path, interfaces):
        if DEVICE_IFACE in interfaces:
            self._update_device(str(path), interfaces[DEVICE_IFACE])
        if GATT_SERVICE_IFACE in interfaces:
            self._update_gatt_service(str(path), interfaces[GATT_SERVICE_IFACE])
        if GATT_CHRC_IFACE in interfaces:
            self._update_gatt_characteristic(str(path), interfaces[GATT_CHRC_IFACE])
        if GATT_DESC_IFACE in interfaces:
            self._update_gatt_descriptor(str(path), interfaces[GATT_DESC_IFACE])

    def _on_interfaces_removed(self, path, interfaces):
        path_str = str(path)
        if DEVICE_IFACE in interfaces:
            removed = None
            with self._lock:
                for mac, device in self._devices.items():
                    if device.path == path_str:
                        removed = device.copy()
                        break
                if removed is not None:
                    self._devices.pop(removed.mac, None)
                    self._gatt_services.pop(path_str, None)
                    self._gatt_characteristics.pop(path_str, None)
                    self._gatt_descriptors.pop(path_str, None)
            if removed is not None:
                self._emit_device(removed.mac, removed, ("removed", "connected", "services_resolved"))
        if GATT_SERVICE_IFACE in interfaces:
            self._remove_gatt_service(path_str)
        if GATT_CHRC_IFACE in interfaces:
            self._remove_gatt_characteristic(path_str)
        if GATT_DESC_IFACE in interfaces:
            self._remove_gatt_descriptor(path_str)

    def _on_properties_changed(self, interface, changed, invalidated, path=""):
        del invalidated
        if interface == DEVICE_IFACE:
            self._update_device(str(path), changed)
        elif interface == GATT_SERVICE_IFACE:
            self._update_gatt_service(str(path), changed)
        elif interface == GATT_CHRC_IFACE:
            self._update_gatt_characteristic(str(path), changed)
        elif interface == GATT_DESC_IFACE:
            self._update_gatt_descriptor(str(path), changed)

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
        refresh_gatt_cache = False
        clear_gatt_cache = False
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
            if "services_resolved" in changed_fields:
                if device.services_resolved:
                    refresh_gatt_cache = True
                else:
                    clear_gatt_cache = True
            snapshot = device.copy()
            device_path = device.path
        if is_new:
            changed_fields.insert(0, "added")
        if clear_gatt_cache and device_path:
            with self._lock:
                self._gatt_services.pop(device_path, None)
                self._gatt_characteristics.pop(device_path, None)
                self._gatt_descriptors.pop(device_path, None)
        elif refresh_gatt_cache and device_path:
            self._refresh_gatt_for_device(device_path)
        if changed_fields:
            self._emit_device(mac, snapshot, tuple(changed_fields))

    def _find_device_path(self, mac: str) -> str:
        device = self.get_device(mac)
        if not device or not device.path:
            return ""
        return str(device.path)

    def _ensure_gatt_cache(self, device: DeviceInfo):
        if not device.path:
            return
        with self._lock:
            has_cache = bool(
                self._gatt_services.get(device.path)
                or self._gatt_characteristics.get(device.path)
                or self._gatt_descriptors.get(device.path)
            )
        if has_cache or not device.services_resolved:
            return
        self._refresh_gatt_for_device(device.path)

    def _refresh_gatt_for_device(self, device_path: str):
        if not device_path:
            return
        with self._lock:
            self._gatt_services.pop(device_path, None)
            self._gatt_characteristics.pop(device_path, None)
            self._gatt_descriptors.pop(device_path, None)
        for path, ifaces in self._managed_objects().items():
            path_str = str(path)
            if GATT_SERVICE_IFACE in ifaces:
                props = ifaces[GATT_SERVICE_IFACE]
                if str(props.get("Device", "")) == device_path or path_str.startswith(device_path + "/"):
                    self._update_gatt_service(path_str, props)
            if GATT_CHRC_IFACE in ifaces:
                props = ifaces[GATT_CHRC_IFACE]
                if self._infer_device_path_from_chrc_props(path_str, props) == device_path:
                    self._update_gatt_characteristic(path_str, props)
            if GATT_DESC_IFACE in ifaces:
                props = ifaces[GATT_DESC_IFACE]
                if self._infer_device_path_from_desc_props(path_str, props) == device_path:
                    self._update_gatt_descriptor(path_str, props)

    def _update_gatt_service(self, path: str, props: dict):
        device_path = self._infer_device_path_from_service_props(path, props)
        if not device_path:
            return
        with self._lock:
            bucket = self._gatt_services.setdefault(device_path, {})
            item = bucket.get(path, {"path": path, "uuid": "", "primary": True, "device": device_path, "includes": []})
            if "UUID" in props:
                item["uuid"] = str(props.get("UUID", ""))
            if "Primary" in props:
                item["primary"] = bool(props.get("Primary", True))
            if "Device" in props:
                item["device"] = str(props.get("Device", ""))
            if "Includes" in props:
                item["includes"] = [str(include) for include in props.get("Includes", [])]
            bucket[path] = item

    def _update_gatt_characteristic(self, path: str, props: dict):
        device_path = self._infer_device_path_from_chrc_props(path, props)
        if not device_path:
            return
        with self._lock:
            bucket = self._gatt_characteristics.setdefault(device_path, {})
            item = bucket.get(path, {"path": path, "service": "", "uuid": "", "flags": [], "notifying": False, "mtu": 0})
            if "Service" in props:
                item["service"] = str(props.get("Service", ""))
            if "UUID" in props:
                item["uuid"] = str(props.get("UUID", ""))
            if "Flags" in props:
                item["flags"] = [str(flag) for flag in props.get("Flags", [])]
            if "Notifying" in props:
                item["notifying"] = bool(props.get("Notifying", False))
            if "MTU" in props:
                item["mtu"] = int(props["MTU"])
            bucket[path] = item

    def _update_gatt_descriptor(self, path: str, props: dict):
        device_path = self._infer_device_path_from_desc_props(path, props)
        if not device_path:
            return
        with self._lock:
            bucket = self._gatt_descriptors.setdefault(device_path, {})
            item = bucket.get(path, {"path": path, "characteristic": "", "uuid": "", "flags": []})
            if "Characteristic" in props:
                item["characteristic"] = str(props.get("Characteristic", ""))
            if "UUID" in props:
                item["uuid"] = str(props.get("UUID", ""))
            if "Flags" in props:
                item["flags"] = [str(flag) for flag in props.get("Flags", [])]
            bucket[path] = item

    def _remove_gatt_service(self, path: str):
        with self._lock:
            for device_path, services in list(self._gatt_services.items()):
                if path not in services:
                    continue
                services.pop(path, None)
                for chrc_path, characteristic in list(self._gatt_characteristics.get(device_path, {}).items()):
                    if characteristic.get("service") != path:
                        continue
                    self._gatt_characteristics[device_path].pop(chrc_path, None)
                    for desc_path, descriptor in list(self._gatt_descriptors.get(device_path, {}).items()):
                        if descriptor.get("characteristic") == chrc_path:
                            self._gatt_descriptors[device_path].pop(desc_path, None)
                if not services:
                    self._gatt_services.pop(device_path, None)
                return

    def _remove_gatt_characteristic(self, path: str):
        with self._lock:
            for device_path, characteristics in list(self._gatt_characteristics.items()):
                if path not in characteristics:
                    continue
                characteristics.pop(path, None)
                for desc_path, descriptor in list(self._gatt_descriptors.get(device_path, {}).items()):
                    if descriptor.get("characteristic") == path:
                        self._gatt_descriptors[device_path].pop(desc_path, None)
                if not characteristics:
                    self._gatt_characteristics.pop(device_path, None)
                return

    def _remove_gatt_descriptor(self, path: str):
        with self._lock:
            for device_path, descriptors in list(self._gatt_descriptors.items()):
                if path in descriptors:
                    descriptors.pop(path, None)
                    if not descriptors:
                        self._gatt_descriptors.pop(device_path, None)
                    return

    def _infer_device_path_from_service_props(self, path: str, props: dict) -> str:
        device_path = str(props.get("Device", ""))
        if device_path:
            return device_path
        marker = "/service"
        if marker in path:
            prefix = path.split(marker, 1)[0]
            if "/dev_" in prefix:
                return prefix
        with self._lock:
            for device in self._devices.values():
                if path.startswith(device.path + "/"):
                    return device.path
        return ""

    def _infer_device_path_from_chrc_props(self, path: str, props: dict) -> str:
        service_path = str(props.get("Service", ""))
        if service_path:
            device_path = self._device_path_for_service(service_path)
            if device_path:
                return device_path
        marker = "/char"
        if marker in path:
            service_path = path.split(marker, 1)[0]
            return self._device_path_for_service(service_path)
        return ""

    def _infer_device_path_from_desc_props(self, path: str, props: dict) -> str:
        characteristic_path = str(props.get("Characteristic", ""))
        if characteristic_path:
            device_path = self._device_path_for_characteristic(characteristic_path)
            if device_path:
                return device_path
        marker = "/desc"
        if marker in path:
            characteristic_path = path.split(marker, 1)[0]
            return self._device_path_for_characteristic(characteristic_path)
        return ""

    def _device_path_for_service(self, service_path: str) -> str:
        with self._lock:
            for device_path, services in self._gatt_services.items():
                if service_path in services:
                    return device_path
        if "/dev_" in service_path and "/service" in service_path:
            return service_path.split("/service", 1)[0]
        return ""

    def _device_path_for_characteristic(self, characteristic_path: str) -> str:
        with self._lock:
            for device_path, characteristics in self._gatt_characteristics.items():
                if characteristic_path in characteristics:
                    return device_path
        if "/char" in characteristic_path:
            service_path = characteristic_path.split("/char", 1)[0]
            return self._device_path_for_service(service_path)
        return ""

    def _get_cached_service_by_path(self, mac: str, service_path: str) -> Optional[dict]:
        device = self.get_device(mac)
        if not device or not device.path:
            return None
        self._ensure_gatt_cache(device)
        with self._lock:
            service = self._gatt_services.get(device.path, {}).get(service_path)
            return dict(service) if service is not None else None

    def _set_device_prop(self, mac: str, prop: str, value) -> bool:
        device = self.get_device(mac)
        if not device or not device.path:
            return False
        try:
            self._run_serialized(
                lambda: dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, device.path), DBUS_PROP_IFACE).Set(
                    DEVICE_IFACE,
                    prop,
                    value,
                ),
                f"set {prop} for {mac}",
                timeout=10.0,
            )
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
        del operation, timeout
        return func()

    def _run_async_method(self, method, operation: str, *args, timeout: float = 30.0, on_reply=None, on_error=None):
        del operation
        method(
            *args,
            reply_handler=on_reply or (lambda *_: None),
            error_handler=on_error or (lambda *_: None),
            timeout=timeout,
        )
        return True

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
