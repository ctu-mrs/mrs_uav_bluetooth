"""Runtime BLE orchestration mixin for the Bluetooth node."""

import re
import struct
import time
from typing import Dict, Tuple

import dbus
from std_srvs.srv import Trigger
from std_msgs.msg import String

from mrs_uav_bluetooth.msg import BleDeviceArray, BleNotification
from mrs_uav_bluetooth.msg import BlePeerTimeStatus
from mrs_uav_bluetooth.srv import (
    ConfigureNotificationBridge,
    ConnectDevice,
    DisconnectDevice,
    FindGattPath,
    GetDevice,
    ListDevices,
    ListGattCharacteristics,
    ListGattDescriptors,
    ListGattServices,
    PairDevice,
    ReadGattValue,
    RemoveDevice,
    SetActiveConfig,
    SetDeviceTrust,
    SetNotify,
    SetScanEnabled,
    WriteGattValue,
)

from .bluetooth_bridge_state import PeerTimeBridgeState, TopicExportBridgeState, TopicImportBridgeState
from .dbus_client import DeviceInfo
from .netplan import NetplanConfiguration
from .gatt_services import (
    TIME_CHARACTERISTIC_UUID,
    TIME_WRITEBACK_DESCRIPTOR_UUID,
    topic_bridge_characteristic_uuid,
    topic_bridge_data_descriptor_uuid,
    topic_bridge_metadata_descriptor_uuid,
)
from .bridge_payload import bytes_to_serializable, member_specs_from_serializable
from .uuid_utils import is_uav_hostname


class BluetoothNodeRuntimeMixin:

    def _ensure_core_publishers(self):
        if self.devices_pub is not None:
            return
        self.devices_pub = self.create_publisher(BleDeviceArray, self._node_topic("devices"), 10)
        self.notifications_pub = self.create_publisher(BleNotification, self._node_topic("notifications"), 50)
        self.status_pub = self.create_publisher(String, self._node_topic("status"), 50)
        self.log_pub = self.create_publisher(String, self._node_topic("log"), 200)

    def _setup_ros_interfaces(self):
        keepalive_suffix = str(self.get_parameter("overlay_keepalive_topic_suffix").value or "overlay_keepalive").strip().strip("/")
        if keepalive_suffix:
            self._overlay_keepalive_topic = self._node_topic(keepalive_suffix)
        self.create_service(ListDevices, "ble/list_devices", self._handle_list_devices)
        self.create_service(GetDevice, "ble/get_device", self._handle_get_device)
        self.create_service(ConnectDevice, "ble/connect_device", self._handle_connect_device)
        self.create_service(DisconnectDevice, "ble/disconnect_device", self._handle_disconnect_device)
        self.create_service(PairDevice, "ble/pair_device", self._handle_pair_device)
        self.create_service(SetDeviceTrust, "ble/set_device_trust", self._handle_set_device_trust)
        self.create_service(RemoveDevice, "ble/remove_device", self._handle_remove_device)
        self.create_service(ListGattServices, "ble/list_gatt_services", self._handle_list_gatt_services)
        self.create_service(ListGattCharacteristics, "ble/list_gatt_characteristics", self._handle_list_gatt_characteristics)
        self.create_service(ListGattDescriptors, "ble/list_gatt_descriptors", self._handle_list_gatt_descriptors)
        self.create_service(FindGattPath, "ble/find_gatt_path", self._handle_find_gatt_path)
        self.create_service(ReadGattValue, "ble/read_gatt_value", self._handle_read_gatt_value)
        self.create_service(WriteGattValue, "ble/write_gatt_value", self._handle_write_gatt_value)
        self.create_service(SetNotify, "ble/set_notify", self._handle_set_notify)
        self.create_service(SetScanEnabled, "ble/set_scan_enabled", self._handle_set_scan_enabled)
        self.create_service(ConfigureNotificationBridge, "ble/configure_notification_bridge", self._handle_configure_notification_bridge)
        self.create_service(Trigger, "ble/reload_config", self._handle_reload_config)
        self.create_service(SetActiveConfig, "ble/set_active_config", self._handle_set_active_config)

    def _setup_bluetooth(self):
        self._dbus.setup()
        self._client.add_notification_handler(self._on_notification)
        self._client.add_gatt_event_handler(self._on_client_gatt_event)
        self._rebuild_netplan()
        self._dbus.ensure_pairing_agent(
            auto_accept=bool(self.get_parameter("auto_accept_pairing").value),
            auto_trust=bool(self.get_parameter("auto_trust").value),
            capability=self.get_parameter("pairing_agent").value,
        )
        self._apply_runtime_side_effects(initial=True)

    def _rebuild_netplan(self):
        self._netplan = NetplanConfiguration(
            self.get_parameter("netplan_config_file").value,
            self.get_parameter("netplan_scripts_dir").value,
            self._get_string_list("allowed_wifi_networks"),
        )

    def _apply_runtime_side_effects(self, *, initial: bool = False):
        self._setup_verbose_logger()
        self._rebuild_netplan()
        try:
            self._dbus.set_adapter_props(discoverable_timeout=int(self.get_parameter("discoverable_timeout").value))
        except Exception:
            pass
        self._sync_configured_exports()
        if bool(self.get_parameter("enable_server").value):
            self._rebuild_server()
        else:
            self._dbus.unregister_server_objects(self._topic_exports)
        desired_scan = bool(self.get_parameter("enable_scan").value)
        desired_transport = str(self.get_parameter("scan_mode").value)
        if desired_scan:
            if self._client is not None and self._client.scanning and self._scan_transport == desired_transport:
                pass
            else:
                self._start_scan(desired_transport)
        else:
            self._stop_scan()
        if not initial:
            self.get_logger().info(f"Applied config from {self._active_config_source}")
            self._log_verbose(f"Applied config from {self._active_config_source}")

    def _rebuild_server(self):
        with self._lock:
            self._log_verbose(
                f"Rebuilding GATT server: wifi={bool(self.get_parameter('enable_wifi_service').value)}, "
                f"time={bool(self.get_parameter('enable_time_service').value)}, "
                f"exports={len(self._topic_exports)}"
            )
            self._dbus.rebuild_server(
                self._topic_exports,
                enable_wifi_service=bool(self.get_parameter("enable_wifi_service").value),
                enable_time_service=bool(self.get_parameter("enable_time_service").value),
                advertise_mode=self.get_parameter("advertise_mode").value,
                discoverable_timeout=int(self.get_parameter("discoverable_timeout").value),
                wifi_name_cb=self._get_wifi_name,
                wifi_apply_cb=self._set_wifi_name,
                wifi_password_write_cb=self._set_wifi_password,
                wifi_password_read_cb=lambda: "",
                time_writeback_cb=self._handle_time_writeback,
            )

    def _sync_configured_exports(self):
        removed = False
        for key in [item for item, state in self._topic_exports.items() if state.auto_managed]:
            state = self._topic_exports.pop(key)
            self._destroy_export_bridge(state)
            self.destroy_subscription(state.subscription)
            removed = True
        for shared in self._shared_topic_configs.values():
            if shared.mode not in {"export", "both"}:
                continue
            key = f"config-export::{shared.bridge_key}"
            self._log_verbose(f"Creating export bridge: {shared.export_topic} -> {shared.bridge_key[:8]}...")
            subscription = self.create_subscription(
                shared.message_class,
                shared.export_topic,
                lambda msg, bridge_key=key: self._on_export_topic_message(bridge_key, msg),
                10,
            )
            self._topic_exports[key] = TopicExportBridgeState(
                topic_name=shared.export_topic,
                message_type=shared.message_type,
                message_class=shared.message_class,
                bridge_name=shared.bridge_name,
                bridge_key=shared.bridge_key,
                bridge_uuid=topic_bridge_data_descriptor_uuid(shared.bridge_name)
                if shared.transport_endpoint == "descriptor"
                else topic_bridge_characteristic_uuid(shared.bridge_name),
                member_specs=shared.member_specs,
                rate_hz=shared.rate_hz,
                transport_endpoint=shared.transport_endpoint,
                payload_format=shared.payload_format,
                subscription=subscription,
                auto_managed=True,
            )
            self._configure_export_bridge_timer(self._topic_exports[key], key)
        if removed and not self._topic_exports:
            self._log_verbose("Removed all auto-managed export bridges")

    def _start_scan(self, transport: str):
        ok = self._dbus.start_scan(transport)
        if ok:
            self._scan_transport = transport
        return ok

    def _stop_scan(self):
        return self._dbus.stop_scan()

    def _get_wifi_name(self) -> str:
        ssid = self._netplan.get_current_ssid()
        return ssid or "unknown"

    def _set_wifi_password(self, password: str):
        self._pending_wifi_password = password.rstrip("\r\n")
        self.get_logger().info("Received Wi-Fi password update over BLE")

    def _set_wifi_name(self, ssid: str):
        success, detail = self._netplan.set_current_network(ssid, password=self._pending_wifi_password)
        if success:
            self._pending_wifi_password = ""
            self.get_logger().info(f"Started Wi-Fi switch using {detail}")
        else:
            self.get_logger().error(f"Failed to change Wi-Fi to '{ssid}': {detail}")

    def _refresh_wifi_service(self):
        if self._wifi_service is not None:
            self._wifi_service.update(self._get_wifi_name())

    def _update_time_service(self):
        if self._time_service is not None:
            self._time_service.update()

    def _dbus_warning(self, key: str, message: str, interval_s: float = 5.0):
        now = time.monotonic()
        last = self._last_dbus_warning_at.get(key, 0.0)
        if now - last < interval_s:
            return
        self._last_dbus_warning_at[key] = now
        self.get_logger().warning(message)
        self._log_verbose(message)

    def _publish_scan_results(self):
        if self._client is None:
            return
        try:
            self._check_overlay_config_lease()
            snapshot = self._client.get_devices(refresh=True)
            self._sync_auto_import_bridges(snapshot)
            self._reconcile_import_bridges(snapshot)
            msg = BleDeviceArray()
            msg.header = self._header(frame_id=self._local_frame_id())
            msg.devices = [self._device_to_msg(device) for device in snapshot.values()]
            self.devices_pub.publish(msg)
            self._refresh_notification_mapping(snapshot)
            self._cleanup_peer_time_bridges(snapshot)
            if snapshot:
                lines = [f"Scan results: {len(snapshot)} device(s)"]
                for mac, dev in sorted(snapshot.items()):
                    lines.append(
                        f"  {mac}  name={dev.name or '?'}  alias={dev.alias or '?'}"
                        f"  rssi={dev.rssi}  connected={dev.connected}"
                        f"  paired={dev.paired}  trusted={dev.trusted}"
                    )
                self._log_verbose("\n".join(lines))
            else:
                self._log_verbose("Scan results: no devices found")
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("scan_results", f"Skipping BLE scan publish tick due to DBus error: {exc}")

    def _sync_auto_import_bridges(self, snapshot: Dict[str, DeviceInfo]):
        desired_keys = set()
        for shared in self._shared_topic_configs.values():
            if shared.mode not in {"import", "both"}:
                continue
            for mac, device in snapshot.items():
                if not device.connected:
                    continue
                if (device.alias or device.name or "").strip().lower() == self._local_name.strip().lower():
                    continue
                path, bridge_uuid, transport_endpoint = self._resolve_remote_characteristic(
                    mac, shared.bridge_name, shared.transport_endpoint
                )
                key = f"config-import::{shared.bridge_key}::{mac}"
                state = self._notification_bridges.get(key)
                device_label = f"{mac} ({device.alias or device.name or '?'})"
                if not path:
                    if state is not None:
                        self._log_verbose(f"Removing import bridge (no path): {device_label}")
                        self._remove_import_bridge(key, state)
                    continue
                desired_keys.add(key)
                resolved_topic_name = self._resolve_peer_topic_name(mac, shared.import_topic_suffix, device=device)
                if state is None:
                    if transport_endpoint == "characteristic":
                        started, path = self._start_notify_with_refresh(
                            mac,
                            path,
                            bridge_uuid,
                            label=f"import bridge {shared.bridge_name}",
                        )
                        if not started:
                            self._log_verbose(f"Failed to start notify for import bridge: {device_label}")
                            continue
                    self._log_verbose(f"Creating import bridge: {device_label} -> {resolved_topic_name}")
                    publisher = self.create_publisher(shared.message_class, resolved_topic_name, 10)
                    self._notification_bridges[key] = TopicImportBridgeState(
                        mac=mac,
                        requested_topic_name=shared.import_topic_suffix,
                        resolved_topic_name=resolved_topic_name,
                        message_type=shared.message_type,
                        message_class=shared.message_class,
                        bridge_name=shared.bridge_name,
                        bridge_key=shared.bridge_key,
                        bridge_uuid=bridge_uuid,
                        member_specs=shared.member_specs,
                        rate_hz=shared.rate_hz,
                        transport_endpoint=transport_endpoint,
                        payload_format=shared.payload_format,
                        path=path,
                        publisher=publisher,
                        auto_managed=True,
                    )
                    self._configure_import_bridge_timer(self._notification_bridges[key], key)
                    continue
                old_path = state.path
                old_transport = state.transport_endpoint
                if transport_endpoint == "characteristic" and (old_transport != "characteristic" or old_path != path):
                    started, path = self._start_notify_with_refresh(
                        mac,
                        path,
                        bridge_uuid,
                        label=f"import bridge {shared.bridge_name}",
                    )
                    if not started:
                        continue
                if state.resolved_topic_name != resolved_topic_name or state.message_type != shared.message_type:
                    replacement = self.create_publisher(shared.message_class, resolved_topic_name, 10)
                    self.destroy_publisher(state.publisher)
                    state.publisher = replacement
                state.requested_topic_name = shared.import_topic_suffix
                state.resolved_topic_name = resolved_topic_name
                state.message_type = shared.message_type
                state.message_class = shared.message_class
                state.bridge_name = shared.bridge_name
                state.bridge_key = shared.bridge_key
                state.bridge_uuid = bridge_uuid
                state.member_specs = shared.member_specs
                state.rate_hz = shared.rate_hz
                state.transport_endpoint = transport_endpoint
                state.payload_format = shared.payload_format
                state.path = path
                self._configure_import_bridge_timer(state, key)
                if old_transport == "characteristic" and (transport_endpoint != "characteristic" or old_path != path):
                    self._stop_notify_if_unused(old_path)
        for key in [item for item, state in self._notification_bridges.items() if state.auto_managed and item not in desired_keys]:
            self._remove_import_bridge(key, self._notification_bridges[key])

    def _remove_import_bridge(self, key: str, state: TopicImportBridgeState):
        self._notification_bridges.pop(key, None)
        self._destroy_import_bridge(state)
        self.destroy_publisher(state.publisher)
        if state.transport_endpoint == "characteristic":
            self._stop_notify_if_unused(state.path)

    def _refresh_notification_mapping(self, snapshot: Dict[str, DeviceInfo]):
        path_to_mac = {}
        for mac in snapshot:
            try:
                characteristics = self._client.list_characteristics(mac)
            except dbus.exceptions.DBusException as exc:
                self._dbus_warning("refresh_notification_mapping", f"Failed to refresh notification mapping for {mac}: {exc}")
                continue
            for characteristic in characteristics:
                path_to_mac[characteristic["path"]] = mac
        self._notification_path_to_mac = path_to_mac

    def _cleanup_peer_time_bridges(self, snapshot: Dict[str, DeviceInfo]):
        stale_macs = []
        now = time.monotonic()
        timeout = max(0.0, float(self.get_parameter("peer_connection_timeout").value))
        for mac, state in self._peer_time_bridges.items():
            device = snapshot.get(mac)
            if device is not None and device.connected:
                if timeout > 0 and now - state.last_activity_monotonic >= timeout:
                    self.get_logger().warning(
                        f"Disconnecting {mac}: peer time bridge inactive for {now - state.last_activity_monotonic:.1f}s"
                    )
                    self._client.disconnect(mac, timeout=5.0)
                    stale_macs.append(mac)
                continue
            stale_macs.append(mac)
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        for mac, device in snapshot.items():
            if not device.connected or not self._is_uav_peer_candidate(device, pattern):
                self._peer_inactive_since.pop(mac, None)
                continue
            if mac in self._peer_time_bridges:
                continue
            self._peer_inactive_since.setdefault(mac, now)
        for mac in stale_macs:
            state = self._peer_time_bridges.pop(mac, None)
            if state is not None:
                self.destroy_publisher(state.publisher)
                self._stop_notify_if_unused(state.characteristic_path)
            self._peer_security_attempts.pop(mac, None)
            self._peer_repair_attempts.pop(mac, None)
            self._peer_inactive_since.pop(mac, None)

    def _reconcile_import_bridges(self, snapshot: Dict[str, DeviceInfo]):
        for state in self._notification_bridges.values():
            device = snapshot.get(state.mac)
            if device is None or not device.connected:
                continue
            if state.bridge_uuid:
                current_path, notifying = self._find_remote_bridge_path(state.mac, state.bridge_uuid, state.transport_endpoint)
                if not current_path:
                    continue
                if current_path != state.path:
                    old_path = state.path
                    state.path = current_path
                    if state.transport_endpoint == "characteristic":
                        self._stop_notify_if_unused(old_path)
                if state.transport_endpoint != "characteristic" or notifying:
                    continue
            started, refreshed_path = self._start_notify_with_refresh(
                state.mac,
                state.path,
                state.bridge_uuid,
                label=f"import bridge {state.bridge_name}",
            )
            state.path = refreshed_path
            if not started:
                self.get_logger().warning(
                    f"Failed to re-enable notifications for topic bridge {state.resolved_topic_name} on {state.mac}"
                )

    def _find_remote_bridge_path(self, mac: str, bridge_uuid: str, transport_endpoint: str):
        if transport_endpoint == "descriptor":
            for descriptor in self._client.list_descriptors(mac):
                if descriptor["uuid"].lower() == bridge_uuid.lower():
                    return descriptor["path"], False
            return "", False
        for characteristic in self._client.list_characteristics(mac):
            if characteristic["uuid"].lower() != bridge_uuid.lower():
                continue
            return characteristic["path"], bool(characteristic.get("notifying", False))
        return "", False

    def _auto_connect_devices(self):
        if self._client is None or not bool(self.get_parameter("auto_connect_enable").value):
            return
        try:
            now = time.monotonic()
            retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
            whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
            whitelist_enabled = bool(whitelist_names or whitelist_macs)
            pattern = str(self.get_parameter("auto_connect_pattern").value)
            snapshot = self._client.get_devices(refresh=True)
            self._auto_connect_attempts = {mac: stamp for mac, stamp in self._auto_connect_attempts.items() if mac in snapshot}
            self._peer_security_attempts = {mac: stamp for mac, stamp in self._peer_security_attempts.items() if mac in snapshot}
            self._peer_repair_attempts = {mac: stamp for mac, stamp in self._peer_repair_attempts.items() if mac in snapshot}
            self._peer_inactive_since = {mac: stamp for mac, stamp in self._peer_inactive_since.items() if mac in snapshot}
            lines = [
                f"Auto-connect tick: {len(snapshot)} device(s), "
                f"whitelist={sorted(whitelist_names) or '(none)'}, pattern={pattern}, "
                f"enable={bool(self.get_parameter('auto_connect_enable').value)}"
            ]
            for mac, dev in sorted(snapshot.items()):
                lines.append(
                    f"  {mac}  name={dev.name or '?'}  alias={dev.alias or '?'}"
                    f"  connected={dev.connected}  candidate={self._is_uav_peer_candidate(dev, pattern)}"
                )
            self._log_verbose("\n".join(lines))
            for mac, device in snapshot.items():
                peer_candidate = self._is_uav_peer_candidate(device, pattern)
                explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
                should_connect = explicit_target or (peer_candidate and not whitelist_enabled)
                device_label = f"{mac} ({device.alias or device.name or '?'})"
                if whitelist_enabled and peer_candidate and not explicit_target:
                    self._log_verbose(f"Dropping non-whitelisted peer: {device_label}")
                    self._drop_non_whitelisted_peer(mac, device)
                    continue
                if device.connected:
                    self._auto_connect_attempts.pop(mac, None)
                    if should_connect:
                        self._maintain_peer_connection(mac, device, retry_period, explicit_target=explicit_target)
                    continue
                if not should_connect:
                    continue
                last_attempt = self._auto_connect_attempts.get(mac, 0.0)
                if now - last_attempt < retry_period:
                    continue
                self._auto_connect_attempts[mac] = now
                self._log_verbose(f"Auto-connect attempt: {device_label}")
                connected = self._client.connect(mac, timeout=10.0)
                if not connected:
                    refreshed = self._client.get_device(mac, refresh=True)
                    connected = bool(refreshed and refreshed.connected)
                if not connected:
                    self.get_logger().warning(f"Auto-connect failed: {device_label}")
                    self._log_verbose(f"Auto-connect failed: {device_label}")
                    continue
                self._client.wait_services_resolved(mac, timeout=10.0)
                current = self._client.get_device(mac, refresh=True) or device
                if peer_candidate and not self._maintain_peer_connection(mac, current, retry_period, explicit_target=explicit_target) and not explicit_target:
                    now_mono = time.monotonic()
                    missing_since = self._peer_inactive_since.setdefault(mac, now_mono)
                    grace_s = max(10.0, retry_period * 3.0)
                    waited_s = max(0.0, now_mono - missing_since)
                    if waited_s < grace_s:
                        self._log_verbose(
                            f"Keeping {device_label} connected while waiting for time characteristic "
                            f"({waited_s:.1f}s/{grace_s:.1f}s)"
                        )
                        continue
                    self.get_logger().info(f"Disconnecting {mac}: UAV peer candidate without BLE time characteristic")
                    self._log_verbose(f"Disconnecting {device_label}: no time characteristic found after grace period")
                    self._client.disconnect(mac, timeout=5.0)
                else:
                    self._auto_connect_attempts.pop(mac, None)
                    self.get_logger().info(f"Auto-connected BLE device {mac}")
                    self._log_verbose(f"Auto-connected: {device_label}")
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("auto_connect", f"Skipping auto-connect tick due to DBus error: {exc}")

    def _drop_non_whitelisted_peer(self, mac: str, device: DeviceInfo):
        if device.connected:
            self._client.disconnect(mac, timeout=5.0)
        if device.paired or device.bonded or device.trusted or device.path:
            if self._client.remove(mac):
                self.get_logger().info(f"Removed non-whitelisted peer {mac}")
        if mac in self._peer_time_bridges:
            state = self._peer_time_bridges.pop(mac)
            self.destroy_publisher(state.publisher)
            self._stop_notify_if_unused(state.characteristic_path)
        self._peer_security_attempts.pop(mac, None)
        self._peer_repair_attempts.pop(mac, None)
        self._peer_inactive_since.pop(mac, None)

    def _is_uav_peer_candidate(self, device: DeviceInfo, pattern: str) -> bool:
        local_name = self._local_name.strip().lower()
        for candidate in (device.alias or "", device.name or "", self._hostname_from_device(device)):
            normalized = candidate.strip().lower()
            if not normalized or normalized == local_name:
                continue
            if is_uav_hostname(normalized, pattern=pattern):
                return True
        return False

    def _start_notify_with_refresh(self, mac: str, path: str, characteristic_uuid: str, *, label: str) -> Tuple[bool, str]:
        tried_paths = []

        def _try_notify(candidate_path: str) -> bool:
            candidate = str(candidate_path or "")
            if not candidate or candidate in tried_paths:
                return False
            tried_paths.append(candidate)
            if self._client.start_notify(candidate):
                return True
            return False

        if _try_notify(path):
            return True, path

        self._log_verbose(f"Retrying notify setup for {label} on {mac}: path={path}")
        self._client.wait_services_resolved(mac, timeout=5.0)

        candidates = self._client.find_characteristics(mac, characteristic_uuid)
        if path and path in candidates:
            candidates = [path] + [item for item in candidates if item != path]

        for candidate in candidates:
            if _try_notify(candidate):
                if candidate != path:
                    self._log_verbose(f"Refreshed notify path for {label} on {mac}: {path} -> {candidate}")
                return True, candidate

        refreshed_path = candidates[0] if candidates else (path or "")
        return False, refreshed_path

    def _ensure_peer_time_bridge(self, mac: str, device: DeviceInfo) -> Tuple[bool, bool]:
        current = self._client.get_device(mac, refresh=True) or device
        device_label = f"{mac} ({self._hostname_from_device(device) or '?'})"
        if not current.services_resolved and not self._client.wait_services_resolved(mac, timeout=5.0):
            self._peer_inactive_since.setdefault(mac, time.monotonic())
            self._log_verbose(f"Peer {mac}: services not resolved yet, delaying time bridge setup")
            self._maybe_repair_peer_link(
                mac,
                device_label,
                reason="services unresolved for too long",
                min_wait_s=20.0,
            )
            return False, True

        time_candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        path = time_candidates[0] if time_candidates else ""
        if not path:
            self._peer_inactive_since.setdefault(mac, time.monotonic())
            self._log_verbose(f"Peer {device_label}: time characteristic {TIME_CHARACTERISTIC_UUID} not found")
            retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
            self._maybe_repair_peer_link(
                mac,
                device_label,
                reason=f"missing time characteristic {TIME_CHARACTERISTIC_UUID}",
                min_wait_s=max(8.0, retry_period * 3.0),
            )
            return False, False
        writeback_descriptor_path = self._client.find_descriptor(mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
        peer_name = self._hostname_from_device(device) or mac.lower().replace(":", "_")
        status_topic_name = self._resolve_peer_topic_name(mac, "/time_status", device=device)
        existing = self._peer_time_bridges.get(mac)
        if (
            existing is not None
            and existing.characteristic_path == path
            and existing.writeback_descriptor_path == writeback_descriptor_path
            and existing.status_topic_name == status_topic_name
        ):
            self._peer_inactive_since.pop(mac, None)
            if self._is_characteristic_notifying(mac, path):
                return True, True
            started, path = self._start_notify_with_refresh(
                mac,
                path,
                TIME_CHARACTERISTIC_UUID,
                label="peer time characteristic",
            )
            writeback_descriptor_path = self._client.find_descriptor(mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
            existing.characteristic_path = path
            existing.writeback_descriptor_path = writeback_descriptor_path
            if started:
                self._prime_peer_time_bridge(existing)
            return started, True
        if existing is not None:
            old_path = existing.characteristic_path
            self.destroy_publisher(existing.publisher)
            self._peer_time_bridges.pop(mac, None)
            self._stop_notify_if_unused(old_path)
        publisher = self.create_publisher(BlePeerTimeStatus, status_topic_name, 10)
        started, path = self._start_notify_with_refresh(
            mac,
            path,
            TIME_CHARACTERISTIC_UUID,
            label="peer time characteristic",
        )
        if not started:
            self.get_logger().warning(f"Failed to start time notify for peer {device_label}")
            self._log_verbose(f"Failed to start_notify on time characteristic {path} for {device_label}")
            self.destroy_publisher(publisher)
            self._peer_inactive_since.setdefault(mac, time.monotonic())
            return False, True
        writeback_descriptor_path = self._client.find_descriptor(mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
        self.get_logger().info(f"Established time bridge with peer {device_label} -> {status_topic_name}")
        self._log_verbose(
            f"Time bridge: {device_label} chrc={path} writeback={writeback_descriptor_path or 'none'}"
            f" topic={status_topic_name}"
        )
        self._peer_time_bridges[mac] = PeerTimeBridgeState(
            mac=mac,
            peer_name=peer_name,
            status_topic_name=status_topic_name,
            characteristic_path=path,
            writeback_descriptor_path=writeback_descriptor_path,
            publisher=publisher,
        )
        self._peer_repair_attempts.pop(mac, None)
        self._peer_inactive_since.pop(mac, None)
        self._prime_peer_time_bridge(self._peer_time_bridges[mac])
        return True, True

    def _maybe_repair_peer_link(self, mac: str, device_label: str, *, reason: str, min_wait_s: float) -> bool:
        now_mono = time.monotonic()
        missing_since = self._peer_inactive_since.setdefault(mac, now_mono)
        waited_s = max(0.0, now_mono - missing_since)
        if waited_s < max(0.0, float(min_wait_s)):
            return False
        retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
        repair_cooldown_s = max(15.0, retry_period * 4.0)
        last_attempt = self._peer_repair_attempts.get(mac, 0.0)
        if now_mono - last_attempt < repair_cooldown_s:
            return False
        self._peer_repair_attempts[mac] = now_mono
        self.get_logger().warning(f"Repairing peer {mac}: {reason}; clearing local bond/cache state")
        self._log_verbose(f"Repairing peer {device_label}: {reason}")
        bridge = self._peer_time_bridges.pop(mac, None)
        if bridge is not None:
            self.destroy_publisher(bridge.publisher)
            self._stop_notify_if_unused(bridge.characteristic_path)
        self._client.disconnect(mac, timeout=5.0)
        self._client.untrust(mac)
        self._client.remove(mac)
        self._peer_security_attempts.pop(mac, None)
        self._auto_connect_attempts.pop(mac, None)
        self._peer_inactive_since.pop(mac, None)
        return True

    def _prime_peer_time_bridge(self, state: PeerTimeBridgeState):
        payload = self._client.read_characteristic(state.characteristic_path)
        if payload is not None:
            self._process_peer_time_payload(state.mac, state.characteristic_path, payload)

    def _maintain_peer_connection(self, mac: str, device: DeviceInfo, retry_period: float, explicit_target: bool = False) -> bool:
        time_bridge_ready = False
        time_bridge_available = False
        self._ensure_peer_security(mac, device, retry_period)
        current = self._client.get_device(mac, refresh=True) or device
        if self._is_uav_peer_candidate(device, str(self.get_parameter("auto_connect_pattern").value)):
            time_bridge_ready, time_bridge_available = self._ensure_peer_time_bridge(mac, current)
        return time_bridge_ready or time_bridge_available or explicit_target

    def _ensure_peer_security(self, mac: str, device: DeviceInfo, retry_period: float):
        current = self._client.get_device(mac, refresh=True) or device
        security_ready = bool(current.trusted and (current.paired or current.bonded))
        if security_ready:
            self._peer_security_attempts.pop(mac, None)
            return
        now = time.monotonic()
        last_attempt = self._peer_security_attempts.get(mac, 0.0)
        if now - last_attempt < retry_period:
            return
        self._peer_security_attempts[mac] = now
        device_label = f"{mac} ({self._hostname_from_device(current) or '?'})"
        self._log_verbose(
            f"Security state for {device_label}: paired={current.paired} trusted={current.trusted} bonded={current.bonded}"
        )
        if not (current.paired or current.bonded):
            paired_ok = self._client.pair(mac, timeout=30.0)
            if not paired_ok and current.trusted:
                # Recover from stale one-sided bonds (peer removed pairing but local side kept trust/bond state).
                self._log_verbose(f"Pairing retry after clearing stale trust for {device_label}")
                self._client.untrust(mac)
                self._client.disconnect(mac, timeout=5.0)
                self._client.remove(mac)
                self._client.connect(mac, timeout=10.0)
                self._client.wait_services_resolved(mac, timeout=10.0)
                paired_ok = self._client.pair(mac, timeout=30.0)
            if paired_ok:
                self.get_logger().info(f"Paired BLE peer {mac}")
                self._log_verbose(f"Paired: {device_label}")
            else:
                self.get_logger().warning(f"Failed to pair BLE peer {mac}")
                self._log_verbose(f"Pairing failed: {device_label}")
                self._maybe_repair_peer_link(
                    mac,
                    device_label,
                    reason="pairing failed (possible one-sided stale bond)",
                    min_wait_s=0.0,
                )
            current = self._client.get_device(mac, refresh=True) or current
        if (current.paired or current.bonded) and not current.trusted:
            if self._client.trust(mac):
                self.get_logger().info(f"Trusted BLE peer {mac}")
                self._log_verbose(f"Trusted: {device_label}")
            else:
                self.get_logger().warning(f"Failed to trust BLE peer {mac}")
                self._log_verbose(f"Trust failed: {device_label}")
            current = self._client.get_device(mac, refresh=True) or current
        if current.trusted and (current.paired or current.bonded):
            self._peer_security_attempts.pop(mac, None)

    def _is_characteristic_notifying(self, mac: str, path: str) -> bool:
        for characteristic in self._client.list_characteristics(mac):
            if characteristic["path"] == path:
                return bool(characteristic.get("notifying", False))
        return False

    def _stop_notify_if_unused(self, path: str):
        if any(state.transport_endpoint == "characteristic" and state.path == path for state in self._notification_bridges.values()):
            return
        if any(state.characteristic_path == path for state in self._peer_time_bridges.values()):
            return
        try:
            self._client.stop_notify(path)
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("stop_notify_if_unused", f"Ignoring StopNotify failure for {path}: {exc}")

    def _on_notification(self, data: bytes, uuid: str, chrc_path: str):
        mac = self._notification_path_to_mac.get(chrc_path, "")
        self._log_verbose(f"Notification mac={mac} uuid={uuid} path={chrc_path} len={len(data)}")
        msg = BleNotification()
        msg.header = self._header(frame_id=self._peer_frame_id(mac) if mac else self._local_frame_id())
        msg.mac = mac
        msg.path = chrc_path
        msg.uuid = uuid
        msg.value = list(data)
        self.notifications_pub.publish(msg)
        if mac:
            self._process_peer_time_payload(mac, chrc_path, data)
        self._buffer_import_topic_bridge(mac, chrc_path, data)

    def _process_peer_time_payload(self, mac: str, chrc_path: str, data: bytes):
        state = self._peer_time_bridges.get(mac)
        if state is None or state.characteristic_path != chrc_path or len(data) < 8:
            return
        time_value_ns = struct.unpack("<Q", data[:8])[0]
        state.last_activity_monotonic = time.monotonic()
        state.last_time_value_ns = time_value_ns
        state.last_publish_monotonic, state.current_hz = self._update_publish_rate(state.last_publish_monotonic)
        self._publish_peer_time_status(state)
        if state.writeback_descriptor_path:
            self._update_peer_writeback_latency(state, data)

    def _update_peer_writeback_latency(self, state: PeerTimeBridgeState, payload: bytes):
        if not state.writeback_descriptor_path:
            return
        if self._client.write_descriptor(state.writeback_descriptor_path, payload):
            return
        refreshed_path = self._client.find_descriptor(state.mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=state.characteristic_path) or ""
        if not refreshed_path or refreshed_path == state.writeback_descriptor_path:
            self.get_logger().warning(
                f"Failed to write peer time writeback descriptor for {state.mac} at {state.writeback_descriptor_path}"
            )
            return
        self._log_verbose(
            f"Refreshing writeback descriptor path for {state.mac}: {state.writeback_descriptor_path} -> {refreshed_path}"
        )
        state.writeback_descriptor_path = refreshed_path
        if not self._client.write_descriptor(refreshed_path, payload):
            self.get_logger().warning(
                f"Failed to write refreshed peer time writeback descriptor for {state.mac} at {refreshed_path}"
            )

    def _buffer_import_topic_bridge(self, mac: str, chrc_path: str, data: bytes):
        for state in self._notification_bridges.values():
            if state.transport_endpoint != "characteristic" or state.path != chrc_path:
                continue
            if state.mac and mac and state.mac != mac:
                continue
            if state.rate_hz > 0:
                state.pending_payload = bytes(data)
                continue
            self._publish_import_payload(state, data)

    def _on_client_gatt_event(self, event_type: str, info: dict):
        self._log_verbose(f"GATT event {event_type}: {info}")
        if event_type.endswith("failed"):
            if event_type == "client_descriptor_write_failed":
                desc_path = str(info.get("desc_path", ""))
                if any(state.writeback_descriptor_path == desc_path for state in self._peer_time_bridges.values()):
                    key = (event_type, desc_path)
                    now = time.monotonic()
                    last = self._last_gatt_warning_at.get(key, 0.0)
                    if now - last < 30.0:
                        return
                    self._last_gatt_warning_at[key] = now
            self.get_logger().warning(f"BLE GATT event {event_type}: {info}")

    def _on_pairing_event(self, event_type, device_path, **kwargs):
        self.get_logger().info(f"Pairing event {event_type} for {device_path}: {kwargs}")
        self._log_verbose(f"Pairing event {event_type} device={device_path} {kwargs}")

    def _hostname_from_device(self, device: DeviceInfo) -> str:
        if device.alias:
            return device.alias
        if device.name:
            return device.name
        return ""

    def _mac_from_device_path(self, device_path: str) -> str:
        candidate = str(device_path or "").strip()
        if not candidate:
            return ""
        match = re.search(r"/dev_((?:[0-9A-Fa-f]{2}_){5}[0-9A-Fa-f]{2})(?:/|$)", candidate)
        if match:
            return match.group(1).replace("_", ":").upper()
        if self._client is None:
            return ""
        for mac, device in self._client.get_devices().items():
            if device.path == candidate:
                return mac
        return ""

    def _handle_time_writeback(self, payload: bytes, options: dict, received_time_ns: int):
        if len(payload) < 8:
            return
        device_path = str(options.get("device", ""))
        mac = self._mac_from_device_path(device_path)
        if not mac:
            self._log_verbose(f"Time writeback from unknown device path={device_path} len={len(payload)}")
            return
        state = self._peer_time_bridges.get(mac)
        if state is None:
            self._log_verbose(f"Time writeback for unmanaged peer mac={mac} path={device_path}")
            return
        echoed_time_ns = struct.unpack("<Q", payload[:8])[0]
        if echoed_time_ns <= 0:
            return
        state.last_activity_monotonic = time.monotonic()
        state.last_rtt_s = max(0.0, (received_time_ns - echoed_time_ns) / 1e9)
        self._log_verbose(
            f"Peer writeback received mac={mac} path={device_path} echoed_time_ns={echoed_time_ns} rtt_s={state.last_rtt_s}"
        )
        self._publish_peer_time_status(state)

    def _read_remote_bridge_metadata(self, mac: str, bridge_name: str, *, message_class: type):
        member_specs = ()
        payload_format = ""
        rate_hz = 0.0
        bridge_key = ""
        metadata_specs = {
            "format": topic_bridge_metadata_descriptor_uuid(bridge_name, "format"),
            "members": topic_bridge_metadata_descriptor_uuid(bridge_name, "members"),
            "rate_hz": topic_bridge_metadata_descriptor_uuid(bridge_name, "rate_hz"),
            "key": topic_bridge_metadata_descriptor_uuid(bridge_name, "key"),
        }
        raw_values = {}
        for key, descriptor_uuid in metadata_specs.items():
            path = self._client.find_descriptor(mac, descriptor_uuid)
            if not path:
                continue
            payload = self._client.read_descriptor(path)
            if payload is None:
                continue
            try:
                raw_values[key] = payload.decode("utf-8")
            except Exception:
                continue
        if raw_values.get("members"):
            member_specs = member_specs_from_serializable(
                bytes_to_serializable(raw_values["members"].encode("utf-8")),
                message_class=message_class,
            )
        if raw_values.get("format"):
            payload_format = raw_values["format"].strip().lower()
        if raw_values.get("rate_hz"):
            try:
                rate_hz = max(0.0, float(raw_values["rate_hz"]))
            except ValueError:
                rate_hz = 0.0
        if raw_values.get("key"):
            bridge_key = raw_values["key"].strip()
        return member_specs, payload_format, rate_hz, bridge_key

    def destroy_node(self):
        self._shutting_down = True
        try:
            self._shutdown_bluetooth()
        finally:
            super().destroy_node()

    def _shutdown_bluetooth(self):
        self._log_verbose("Shutting down Bluetooth node...")
        if self._client is not None:
            notify_paths = {
                state.path for state in self._notification_bridges.values() if state.transport_endpoint == "characteristic"
            }
            notify_paths.update(state.characteristic_path for state in self._peer_time_bridges.values())
            for path in notify_paths:
                try:
                    self._client.stop_notify(path)
                except Exception:
                    pass
            try:
                if self._client.scanning:
                    self._client.stop_scan()
            except Exception:
                pass
            try:
                devices = self._client.get_devices(refresh=True).values()
            except dbus.exceptions.DBusException as exc:
                self._dbus_warning("shutdown_get_devices", f"Skipping peer disconnects during shutdown due to DBus error: {exc}")
                devices = []
            for device in devices:
                if not device.connected:
                    continue
                try:
                    self._client.disconnect(device.mac, timeout=5.0)
                except Exception:
                    pass
        for state in self._topic_exports.values():
            self._destroy_export_bridge(state)
            self.destroy_subscription(state.subscription)
        for key, state in list(self._notification_bridges.items()):
            self._remove_import_bridge(key, state)
        for state in self._peer_time_bridges.values():
            self.destroy_publisher(state.publisher)
        self._notification_bridges.clear()
        self._peer_time_bridges.clear()
        self._topic_exports.clear()
        self._dbus.shutdown(self._topic_exports)
