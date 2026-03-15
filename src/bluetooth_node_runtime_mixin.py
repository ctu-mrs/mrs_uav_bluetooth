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
        self._enforce_peer_connection_policy(reason="config update")
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

    def _is_valid_log_name(self, name: str) -> bool:
        candidate = str(name or "").strip()
        if not candidate or candidate == "?":
            return False
        # Ignore placeholders that are just MAC-shaped aliases.
        if re.match(r"^(?:[0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}$", candidate):
            return False
        return True

    def _format_discovered_device_entry(self, mac: str, device: DeviceInfo, *, prefer_hostname: bool) -> str:
        if prefer_hostname:
            hostname = self._hostname_from_device(device)
            if self._is_valid_log_name(hostname):
                return f"{mac} ({hostname})"
            return mac
        label = ""
        for candidate in (device.name or "", device.alias or ""):
            if self._is_valid_log_name(candidate):
                label = candidate
                break
        if label:
            return f"{mac} ({label})"
        return mac

    def _log_discovered_devices_summary(self, snapshot: Dict[str, DeviceInfo], pattern: str):
        peers = []
        others = []
        for mac, device in sorted(snapshot.items()):
            if self._is_uav_peer_candidate(device, pattern):
                peers.append(self._format_discovered_device_entry(mac, device, prefer_hostname=True))
            else:
                others.append(self._format_discovered_device_entry(mac, device, prefer_hostname=False))
        peers_text = ", ".join(peers) if peers else "(none)"
        others_text = ", ".join(others) if others else "(none)"
        self._log_verbose(f"Discovered peers: {peers_text}")
        self._log_verbose(f"Discovered other: {others_text}")

    def _publish_scan_results(self):
        if self._client is None:
            return
        try:
            self._check_overlay_config_lease()
            snapshot = self._client.get_devices(refresh=True)
            snapshot = self._enforce_peer_connection_policy(snapshot=snapshot, reason="scan policy tick")
            self._sync_auto_import_bridges(snapshot)
            self._reconcile_import_bridges(snapshot)
            msg = BleDeviceArray()
            msg.header = self._header(frame_id=self._local_frame_id())
            msg.devices = [self._device_to_msg(device) for device in snapshot.values()]
            self.devices_pub.publish(msg)
            self._refresh_notification_mapping(snapshot)
            self._cleanup_peer_time_bridges(snapshot)
            self._log_verbose(f"Scan results: {len(snapshot)} device(s)")
            self._log_discovered_devices_summary(
                snapshot,
                pattern=str(self.get_parameter("auto_connect_pattern").value),
            )
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("scan_results", f"Skipping BLE scan publish tick due to DBus error: {exc}")

    def _enforce_peer_connection_policy(self, snapshot: Dict[str, DeviceInfo] = None, *, reason: str = "policy"):
        if self._client is None:
            return snapshot or {}

        if snapshot is None:
            snapshot = self._client.get_devices(refresh=True)

        auto_connect_enabled = bool(self.get_parameter("auto_connect_enable").value)
        whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        whitelist_enabled = bool(whitelist_names or whitelist_macs)
        changed = False

        for mac, device in snapshot.items():
            explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
            peer_candidate = self._is_uav_peer_candidate(device, pattern)
            if not peer_candidate:
                continue
            if (
                reason == "scan policy tick"
                and not whitelist_enabled
                and not auto_connect_enabled
            ):
                continue
            allowed = explicit_target or (auto_connect_enabled and not whitelist_enabled)
            if allowed:
                continue
            if not device.connected and not (device.paired or device.bonded or device.trusted):
                continue
            device_label = f"{mac} ({device.alias or device.name or '?'})"
            if device.connected:
                self.get_logger().info(f"Disconnecting {mac}: peer policy denies connection ({reason})")
                self._log_verbose(f"Disconnecting disallowed peer {device_label}: {reason}")
            else:
                self.get_logger().info(f"Marking disallowed peer {mac} as untrusted ({reason})")
                self._log_verbose(f"Updating stale disallowed peer {device_label} trust state: {reason}")
            self._clear_peer_local_state(mac, device=device, reason=f"policy denies connection ({reason})")
            changed = True

        if changed:
            return self._client.get_devices(refresh=True)
        return snapshot

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
                # Only enforce inactivity timeout for fully ready bridges. Waiting states
                # can be idle for a while during pairing/service resolution.
                if state.status == "ready" and timeout > 0 and now - state.last_activity_monotonic >= timeout:
                    self.get_logger().warning(
                        f"Disconnecting {mac}: peer time bridge inactive for {now - state.last_activity_monotonic:.1f}s"
                    )
                    self._client.disconnect(mac, timeout=5.0)
                    stale_macs.append(mac)
                continue
            stale_macs.append(mac)
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        for mac, device in snapshot.items():
            if not self._is_uav_peer_candidate(device, pattern):
                self._peer_inactive_since.pop(mac, None)
                self._peer_connected_since.pop(mac, None)
                self._peer_service_retry_at.pop(mac, None)
                continue
            if not device.connected:
                self._peer_connected_since.pop(mac, None)
                self._peer_inactive_since.setdefault(mac, now)
                continue
            self._peer_connected_since.setdefault(mac, now)
            if mac in self._peer_time_bridges:
                state = self._peer_time_bridges[mac]
                if state.status == "ready":
                    self._peer_inactive_since.pop(mac, None)
                    self._peer_service_retry_at.pop(mac, None)
                else:
                    self._peer_inactive_since.setdefault(mac, now)
                continue
            self._peer_inactive_since.setdefault(mac, now)
        for mac in stale_macs:
            state = self._peer_time_bridges.pop(mac, None)
            if state is not None:
                self.destroy_publisher(state.publisher)
                self._stop_notify_if_unused(state.characteristic_path)
                self._clear_peer_writeback_state(state.writeback_descriptor_path)
            self._peer_security_attempts.pop(mac, None)
            self._peer_repair_attempts.pop(mac, None)
            self._peer_inactive_since.pop(mac, None)
            self._peer_connected_since.pop(mac, None)
            self._peer_service_retry_at.pop(mac, None)

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
            self._auto_connect_attempts = self._prune_attempt_map(
                self._auto_connect_attempts,
                snapshot,
                now,
                ttl_s=max(20.0, retry_period * 10.0),
            )
            self._peer_security_attempts = self._prune_attempt_map(
                self._peer_security_attempts,
                snapshot,
                now,
                ttl_s=max(20.0, retry_period * 10.0),
            )
            self._peer_repair_attempts = self._prune_attempt_map(
                self._peer_repair_attempts,
                snapshot,
                now,
                ttl_s=max(40.0, retry_period * 20.0),
            )
            self._peer_inactive_since = self._prune_attempt_map(
                self._peer_inactive_since,
                snapshot,
                now,
                ttl_s=max(40.0, retry_period * 20.0),
            )
            self._peer_connected_since = self._prune_attempt_map(
                self._peer_connected_since,
                snapshot,
                now,
                ttl_s=max(40.0, retry_period * 20.0),
            )
            self._peer_service_retry_at = self._prune_attempt_map(
                self._peer_service_retry_at,
                snapshot,
                now,
                ttl_s=max(40.0, retry_period * 20.0),
            )
            self._log_verbose(
                f"Auto-connect tick: {len(snapshot)} device(s), "
                f"whitelist={sorted(whitelist_names) or '(none)'}, pattern={pattern}, "
                f"enable={bool(self.get_parameter('auto_connect_enable').value)}"
            )
            self._log_discovered_devices_summary(snapshot, pattern)
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
                    self._peer_connected_since.setdefault(mac, now)
                    self._auto_connect_attempts.pop(mac, None)
                    if should_connect:
                        healthy = self._maintain_peer_connection(mac, device, retry_period, explicit_target=explicit_target)
                        if not healthy and peer_candidate:
                            self._handle_unhealthy_peer_candidate(mac, device_label, retry_period)
                    continue
                if not should_connect:
                    continue
                last_attempt = self._auto_connect_attempts.get(mac, 0.0)
                if now - last_attempt < retry_period:
                    continue
                self._auto_connect_attempts[mac] = now
                self._log_verbose(f"Auto-connect attempt: {device_label}")
                connect_requested = self._client.connect_async(mac, timeout=min(3.0, retry_period))
                if not connect_requested:
                    # Fallback to a short, bounded synchronous probe only when async request cannot be scheduled.
                    connected = self._client.connect(mac, timeout=min(1.0, retry_period))
                else:
                    connected = False
                if not connected:
                    refreshed = self._client.get_device(mac, refresh=True)
                    connected = bool(refreshed and refreshed.connected)
                if not connected:
                    self._log_verbose(f"Auto-connect pending/failed: {device_label}")
                    continue
                current = self._client.get_device(mac, refresh=True) or device
                if peer_candidate and not self._maintain_peer_connection(mac, current, retry_period, explicit_target=explicit_target):
                    self._handle_unhealthy_peer_candidate(mac, device_label, retry_period)
                else:
                    self._auto_connect_attempts.pop(mac, None)
                    self.get_logger().info(f"Auto-connected BLE device {mac}")
                    self._log_verbose(f"Auto-connected: {device_label}")
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("auto_connect", f"Skipping auto-connect tick due to DBus error: {exc}")

    def _handle_unhealthy_peer_candidate(self, mac: str, device_label: str, retry_period: float) -> bool:
        now_mono = time.monotonic()
        missing_since = self._peer_inactive_since.setdefault(mac, now_mono)
        connected_since = self._peer_connected_since.get(mac, now_mono)
        if connected_since > missing_since:
            missing_since = connected_since
            self._peer_inactive_since[mac] = connected_since
        grace_s = max(90.0, retry_period * 4.0)
        waited_s = max(0.0, now_mono - missing_since)
        if waited_s < grace_s:
            self._log_verbose(
                f"Keeping {device_label} connected while waiting for time characteristic "
                f"({waited_s:.1f}s/{grace_s:.1f}s)"
            )
            self._set_peer_time_status(
                mac,
                "waiting-for-time-bridge",
                f"wait={waited_s:.1f}/{grace_s:.1f}s",
                wait_started=missing_since,
                wait_grace_s=grace_s,
            )
            return False
        self.get_logger().info(f"Disconnecting {mac}: UAV peer candidate without healthy BLE time bridge")
        self._log_verbose(f"Disconnecting {device_label}: no healthy time bridge after grace period")
        self._clear_peer_local_state(mac, reason="missing healthy BLE time bridge")
        return True

    def _drop_non_whitelisted_peer(self, mac: str, device: DeviceInfo):
        self._clear_peer_local_state(mac, device=device, reason="non-whitelisted peer")

    def _clear_peer_local_state(
        self,
        mac: str,
        device: DeviceInfo = None,
        *,
        reason: str = "",
        remove_pairing: bool = False,
        untrust: bool = True,
    ):
        current = device or self._client.get_device(mac, refresh=True)
        self._client.disconnect(mac, timeout=5.0)
        had_pairing_state = bool(current and (current.paired or current.bonded or current.trusted))
        if untrust and had_pairing_state and bool(current and current.trusted):
            if self._client.untrust(mac):
                detail = f" ({reason})" if reason else ""
                self.get_logger().info(f"Set peer {mac} trusted=False{detail}")
            else:
                detail = f" ({reason})" if reason else ""
                self.get_logger().warning(f"Failed to set peer {mac} trusted=False{detail}")
        if remove_pairing:
            removed = self._client.remove(mac)
            if removed:
                detail = f": {reason}" if reason else ""
                self.get_logger().info(f"Cleared local BLE bond/cache for {mac}{detail}")
            else:
                detail = f" ({reason})" if reason else ""
                self.get_logger().warning(f"Failed to clear local BLE bond/cache for {mac}{detail}")
        if mac in self._peer_time_bridges:
            state = self._peer_time_bridges.pop(mac)
            self.destroy_publisher(state.publisher)
            self._stop_notify_if_unused(state.characteristic_path)
            self._clear_peer_writeback_state(state.writeback_descriptor_path)
        self._peer_security_attempts.pop(mac, None)
        self._peer_repair_attempts.pop(mac, None)
        self._peer_inactive_since.pop(mac, None)
        self._peer_connected_since.pop(mac, None)
        self._peer_service_retry_at.pop(mac, None)
        self._auto_connect_attempts.pop(mac, None)

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
        self._client.wait_services_resolved(mac, timeout=1.5)

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

    def _prune_attempt_map(self, attempts: Dict[str, float], snapshot: Dict[str, DeviceInfo], now_mono: float, *, ttl_s: float):
        if not attempts:
            return {}
        ttl = max(0.0, float(ttl_s))
        current_macs = set(snapshot.keys())
        kept = {}
        for mac, stamp in attempts.items():
            if mac in current_macs or (ttl > 0.0 and now_mono - stamp <= ttl):
                kept[mac] = stamp
        return kept

    def _has_live_peer_time_bridge(self, mac: str, *, idle_timeout_s: float = 20.0) -> bool:
        state = self._peer_time_bridges.get(mac)
        if state is None or not state.characteristic_path:
            return False
        try:
            if self._is_characteristic_notifying(mac, state.characteristic_path):
                return True
        except dbus.exceptions.DBusException:
            # Fall back to recent payload activity if BlueZ transiently rejects characteristic listing.
            pass
        return (time.monotonic() - state.last_activity_monotonic) <= max(1.0, float(idle_timeout_s))

    def _ensure_peer_services_resolved(self, mac: str, device: DeviceInfo, *, device_label: str) -> bool:
        # BlueZ may keep ServicesResolved=False even after notifications are active.
        # If the peer time bridge is alive, GATT is already usable for our purposes.
        if self._has_live_peer_time_bridge(mac):
            self._set_peer_time_status(mac, "services-resolved", "services_resolved=active_time_bridge")
            return True
        if device.services_resolved:
            self._set_peer_time_status(mac, "services-resolved", "services_resolved=True")
            return True
        wait_started = self._peer_inactive_since.setdefault(mac, time.monotonic())
        grace_s = max(30.0, float(self.get_parameter("auto_connect_period").value) * 6.0)
        waited_s = max(0.0, time.monotonic() - wait_started)
        self._set_peer_time_status(
            mac,
            "waiting-services-resolved",
            f"wait={waited_s:.1f}/{grace_s:.1f}s",
            wait_started=wait_started,
            wait_grace_s=grace_s,
        )
        self._run_background_once(
            f"resolve-services::{mac}",
            self._client.wait_services_resolved,
            mac,
            8.0,
        )
        if self._client.wait_services_resolved(mac, timeout=1.0):
            self._set_peer_time_status(mac, "services-resolved", "services_resolved=True")
            return True
        # BlueZ can keep ServicesResolved=False while managed objects already expose
        # the characteristic tree; treat this as good enough for bridge setup.
        probe_path = self._resolve_peer_time_characteristic_path(mac, allow_wait=False)
        if probe_path:
            self._set_peer_time_status(
                mac,
                "services-resolved",
                f"services_resolved=False,time_path={probe_path}",
            )
            return True
        self._log_verbose(f"Peer {mac}: services not resolved yet, delaying time bridge setup")
        self._maybe_repair_peer_link(
            mac,
            device_label,
            reason="services unresolved for too long",
            min_wait_s=grace_s,
        )
        return False

    def _force_peer_service_rediscovery(self, mac: str, device_label: str) -> bool:
        current = self._client.get_device(mac, refresh=True)
        if current is None or not current.connected:
            return False
        self._log_verbose(f"Forcing service rediscovery for {device_label}")
        try:
            self._client.wait_services_resolved(mac, timeout=2.0)
            self._client.list_services(mac)
            self._client.list_characteristics(mac)
        except dbus.exceptions.DBusException as exc:
            self._log_verbose(f"Service rediscovery pre-pass failed for {device_label}: {exc}")
        if self._resolve_peer_time_characteristic_path(mac):
            return True

        self._log_verbose(f"Service rediscovery fallback reconnect for {device_label}")
        self._client.disconnect(mac, timeout=3.0)
        if not self._client.connect(mac, timeout=10.0):
            self._log_verbose(f"Service rediscovery reconnect failed for {device_label}")
            return False
        self._client.wait_services_resolved(mac, timeout=8.0)
        if self._resolve_peer_time_characteristic_path(mac):
            self._log_verbose(f"Service rediscovery recovered time characteristic for {device_label}")
            return True
        self._log_verbose(f"Service rediscovery could not resolve time characteristic for {device_label}")
        return False

    def _resolve_peer_time_characteristic_path(self, mac: str, *, allow_wait: bool = True) -> str:
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        if candidates:
            return candidates[0]
        # One more pass after refreshing device props and a short resolve wait.
        self._client.get_device(mac, refresh=True)
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        if candidates:
            return candidates[0]
        if not allow_wait:
            return ""
        self._client.wait_services_resolved(mac, timeout=1.0)
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        return candidates[0] if candidates else ""

    def _ensure_peer_time_bridge(self, mac: str, device: DeviceInfo) -> Tuple[bool, bool]:
        current = self._client.get_device(mac, refresh=True) or device
        device_label = f"{mac} ({self._hostname_from_device(device) or '?'})"
        existing = self._peer_time_bridges.get(mac)
        if existing is not None and self._has_live_peer_time_bridge(mac):
            self._peer_inactive_since.pop(mac, None)
            self._peer_service_retry_at.pop(mac, None)
            self._set_peer_time_status(mac, "ready", f"time_path={existing.characteristic_path}")
            return True, True
        if not self._ensure_peer_services_resolved(mac, current, device_label=device_label):
            return False, False

        path = self._resolve_peer_time_characteristic_path(mac)
        if not path:
            self._peer_inactive_since.setdefault(mac, time.monotonic())
            self._log_verbose(f"Peer {device_label}: time characteristic {TIME_CHARACTERISTIC_UUID} not found")
            retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
            wait_started = self._peer_inactive_since.get(mac, time.monotonic())
            grace_s = max(90.0, retry_period * 4.0)
            waited_s = max(0.0, time.monotonic() - wait_started)
            self._set_peer_time_status(
                mac,
                "waiting-time-characteristic",
                f"wait={waited_s:.1f}/{grace_s:.1f}s uuid={TIME_CHARACTERISTIC_UUID}",
                wait_started=wait_started,
                wait_grace_s=grace_s,
            )
            self._maybe_repair_peer_link(
                mac,
                device_label,
                reason=f"missing time characteristic {TIME_CHARACTERISTIC_UUID}",
                min_wait_s=grace_s,
            )
            return False, False
        writeback_descriptor_path = self._client.find_descriptor(mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
        peer_name = self._peer_name_token(mac, device=device)
        status_topic_name = self._resolve_peer_topic_name(mac, "/time_status", device=device)
        if re.search(r"/peers/[0-9]", status_topic_name):
            status_topic_name = self._node_topic(f"peers/{peer_name}/time_status")
        existing = self._peer_time_bridges.get(mac)
        if (
            existing is not None
            and existing.characteristic_path == path
            and existing.writeback_descriptor_path == writeback_descriptor_path
            and existing.status_topic_name == status_topic_name
        ):
            self._peer_inactive_since.pop(mac, None)
            self._set_peer_time_status(mac, "ready", f"time_path={path}")
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
                self._set_peer_time_status(mac, "ready", f"time_path={path}")
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
            self._set_peer_time_status(mac, "time-notify-failed", f"path={path}")
            return False, False
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
            status="ready",
            detail=f"time_path={path}",
        )
        self._peer_repair_attempts.pop(mac, None)
        self._peer_inactive_since.pop(mac, None)
        self._prime_peer_time_bridge(self._peer_time_bridges[mac])
        return True, True

    def _maybe_repair_peer_link(self, mac: str, device_label: str, *, reason: str, min_wait_s: float) -> bool:
        now_mono = time.monotonic()
        missing_since = self._peer_inactive_since.setdefault(mac, now_mono)
        connected_since = self._peer_connected_since.get(mac)
        if connected_since is not None and connected_since > missing_since:
            missing_since = connected_since
            self._peer_inactive_since[mac] = connected_since
        waited_s = max(0.0, now_mono - missing_since)
        if waited_s < max(0.0, float(min_wait_s)):
            return False
        retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
        repair_cooldown_s = max(60.0, retry_period * 10.0)
        recovery_retry_s = max(20.0, retry_period * 4.0)
        reason_lower = str(reason or "").lower()
        needs_service_recovery = "missing time characteristic" in reason_lower or "services unresolved" in reason_lower
        is_pairing_failure = "pairing failed" in reason_lower

        if is_pairing_failure:
            # Pairing failures are handled by the pairing path; avoid concurrent bond resets from service checks.
            return False

        if needs_service_recovery:
            last_service_retry = self._peer_service_retry_at.get(mac, 0.0)
            if now_mono - last_service_retry >= repair_cooldown_s:
                self._peer_service_retry_at[mac] = now_mono
                self.get_logger().warning(
                    f"Peer {mac}: forcing service rediscovery before bond reset ({reason})"
                )
                scheduled = self._run_background_once(
                    f"service-rediscovery::{mac}",
                    self._force_peer_service_rediscovery,
                    mac,
                    device_label,
                )
                self._peer_inactive_since[mac] = time.monotonic()
                if scheduled:
                    self._log_verbose(f"Scheduled background service rediscovery for {device_label}")
                    return False
                recovered = self._force_peer_service_rediscovery(mac, device_label)
                if recovered:
                    self._peer_service_retry_at.pop(mac, None)
                    return False
                # Never hard-reset bond due to service discovery alone; keep link alive and retry later.
                return False
            if now_mono - last_service_retry < recovery_retry_s:
                return False
        return False

    def _prime_peer_time_bridge(self, state: PeerTimeBridgeState):
        current = self._client.get_device(state.mac, refresh=True)
        if current is None or not current.connected or not current.services_resolved:
            return
        payload = self._client.read_characteristic(state.characteristic_path)
        if payload is not None:
            self._process_peer_time_payload(state.mac, state.characteristic_path, payload)

    def _maintain_peer_connection(self, mac: str, device: DeviceInfo, retry_period: float, explicit_target: bool = False) -> bool:
        time_bridge_ready = False
        self._ensure_peer_security(mac, device, retry_period)
        current = self._client.get_device(mac, refresh=True) or device
        security_ready = bool(current.trusted and (current.paired or current.bonded))
        if not security_ready:
            self._set_peer_time_status(mac, "pairing-pending", "waiting for paired+trusted")
            return False
        if self._is_uav_peer_candidate(device, str(self.get_parameter("auto_connect_pattern").value)):
            time_bridge_ready, _ = self._ensure_peer_time_bridge(mac, current)
        return time_bridge_ready

    def _peer_name_token(self, mac: str, *, device: DeviceInfo = None) -> str:
        name = ""
        if device is not None:
            name = self._hostname_from_device(device)
        if not name and self._client is not None:
            current = self._client.get_device(mac)
            if current is not None:
                name = self._hostname_from_device(current)
        token = re.sub(r"[^a-z0-9_]+", "_", str(name or "").strip().lower())
        token = token.strip("_")
        if not token:
            token = f"peer_{mac.lower().replace(':', '_')}"
        elif token[0].isdigit():
            token = f"peer_{token}"
        return token

    def _ensure_peer_security(self, mac: str, device: DeviceInfo, retry_period: float):
        pair_failures = getattr(self, "_peer_pair_failures", {})
        current = self._client.get_device(mac, refresh=True) or device
        security_ready = bool(current.trusted and (current.paired or current.bonded))
        if security_ready:
            self._peer_security_attempts.pop(mac, None)
            pair_failures.pop(mac, None)
            self._set_peer_time_status(mac, "paired", "paired+trusted")
            self._peer_pair_failures = pair_failures
            return
        now = time.monotonic()
        last_attempt = self._peer_security_attempts.get(mac, 0.0)
        if now - last_attempt < retry_period:
            return
        self._peer_security_attempts[mac] = now
        device_label = f"{mac} ({self._hostname_from_device(current) or '?'})"
        self._set_peer_time_status(mac, "pairing-requested", f"retry_period={retry_period:.1f}s")
        self._log_verbose(
            f"Security state for {device_label}: paired={current.paired} trusted={current.trusted} bonded={current.bonded}"
        )
        if not (current.paired or current.bonded):
            pair_timeout_s = max(4.0, min(12.0, retry_period * 2.0))
            pair_requested = self._client.pair_async(mac, timeout=pair_timeout_s)
            if pair_requested:
                self._set_peer_time_status(mac, "pairing-in-progress", f"timeout={pair_timeout_s:.1f}s")
                self._log_verbose(f"Pair requested for {device_label} (timeout={pair_timeout_s:.1f}s)")
                # Keep this loop short so one problematic peer cannot stall the whole auto-connect tick.
                probe_deadline = time.monotonic() + 1.0
                while time.monotonic() < probe_deadline:
                    current = self._client.get_device(mac, refresh=True) or current
                    if current.paired or current.bonded:
                        break
                    time.sleep(0.2)
                if current.paired or current.bonded:
                    self.get_logger().info(f"Paired BLE peer {mac}")
                    self._log_verbose(f"Paired: {device_label}")
                    self._set_peer_time_status(mac, "pairing-succeeded", "paired=True")
                    pair_failures.pop(mac, None)
                else:
                    self._peer_pair_failures = pair_failures
                    return
            else:
                self.get_logger().warning(f"Failed to pair BLE peer {mac}")
                self._log_verbose(f"Pairing failed: {device_label}")
                failures = pair_failures.get(mac, 0) + 1
                pair_failures[mac] = failures
                self._set_peer_time_status(mac, "pairing-failed", f"attempts={failures}")
                if failures >= 2 or bool(current.trusted):
                    self.get_logger().warning(
                        f"Repairing peer {mac}: pairing failed repeatedly (possible one-sided stale bond); resetting trust/connection state"
                    )
                    self._log_verbose(
                        f"Repairing peer {device_label}: pairing failed repeatedly (possible one-sided stale bond), preserving pairing"
                    )
                    self._clear_peer_local_state(
                        mac,
                        reason="pairing failed repeatedly (possible one-sided stale bond)",
                    )
                    self._peer_repair_attempts[mac] = time.monotonic()
                    self._set_peer_time_status(mac, "repairing-pairing", "resetting trust/connection state")
                else:
                    self._client.disconnect(mac, timeout=3.0)
                    self._log_verbose(f"Pairing failed once for {device_label}, retrying after reconnect")
                self._peer_pair_failures = pair_failures
                return
            current = self._client.get_device(mac, refresh=True) or current
        if (current.paired or current.bonded) and not current.trusted:
            if self._client.trust(mac):
                self.get_logger().info(f"Trusted BLE peer {mac}")
                self._log_verbose(f"Trusted: {device_label}")
                self._set_peer_time_status(mac, "trusted", "Trusted=True")
            else:
                self.get_logger().warning(f"Failed to trust BLE peer {mac}")
                self._log_verbose(f"Trust failed: {device_label}")
                self._set_peer_time_status(mac, "trust-failed", "Trusted=False")
            current = self._client.get_device(mac, refresh=True) or current
        if current.trusted and (current.paired or current.bonded):
            self._peer_security_attempts.pop(mac, None)
            pair_failures.pop(mac, None)
            self._set_peer_time_status(mac, "paired", "paired+trusted")
        self._peer_pair_failures = pair_failures

    def _set_peer_time_status(
        self,
        mac: str,
        status: str,
        detail: str = "",
        *,
        wait_started: float = None,
        wait_grace_s: float = None,
    ):
        state = self._peer_time_bridges.get(mac)
        if state is None:
            device = self._client.get_device(mac) if self._client is not None else None
            peer_name = self._peer_name_token(mac, device=device)
            topic = self._resolve_peer_topic_name(mac, "/time_status", device=device)
            if re.search(r"/peers/[0-9]", topic):
                topic = self._node_topic(f"peers/{peer_name}/time_status")
            publisher = self.create_publisher(BlePeerTimeStatus, topic, 10)
            state = PeerTimeBridgeState(
                mac=mac,
                peer_name=peer_name,
                status_topic_name=topic,
                characteristic_path="",
                writeback_descriptor_path="",
                publisher=publisher,
            )
            self._peer_time_bridges[mac] = state
        state.status = status
        state.detail = detail or ""
        if wait_started is not None:
            state.services_wait_started_monotonic = max(0.0, float(wait_started))
        if wait_grace_s is not None:
            state.services_wait_grace_s = max(0.0, float(wait_grace_s))
        if status.startswith("pairing"):
            state.pairing_requested_monotonic = time.monotonic()
            failures = getattr(self, "_peer_pair_failures", {}).get(mac)
            if failures is not None:
                state.pairing_failures = int(failures)

    def _is_characteristic_notifying(self, mac: str, path: str) -> bool:
        for characteristic in self._client.list_characteristics(mac):
            if characteristic["path"] == path:
                return bool(characteristic.get("notifying", False))
        return False

    def _stop_notify_if_unused(self, path: str):
        if not str(path or "").startswith("/"):
            return
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
        desc_path = state.writeback_descriptor_path
        payload_bytes = bytes(payload)
        with self._peer_writeback_lock:
            self._peer_writeback_pending[desc_path] = payload_bytes
        self._kick_peer_writeback(desc_path)

    def _kick_peer_writeback(self, desc_path: str):
        if not desc_path:
            return
        now = time.monotonic()
        with self._peer_writeback_lock:
            retry_at = getattr(self, "_peer_writeback_retry_at", {}).get(desc_path, 0.0)
            if now < retry_at:
                return
            payload = self._peer_writeback_pending.get(desc_path)
            if payload is None:
                return
            inflight = getattr(self, "_peer_writeback_inflight", set())
            if desc_path in inflight:
                return
            last_sent = getattr(self, "_peer_writeback_last_sent", {}).get(desc_path)
            if last_sent == payload:
                return
            inflight.add(desc_path)
            self._peer_writeback_inflight = inflight
            sent = getattr(self, "_peer_writeback_last_sent", {})
            sent[desc_path] = payload
            self._peer_writeback_last_sent = sent
        if self._client.write_descriptor_async(desc_path, payload):
            return
        with self._peer_writeback_lock:
            self._peer_writeback_inflight.discard(desc_path)
            self._peer_writeback_last_sent.pop(desc_path, None)
            retry_map = getattr(self, "_peer_writeback_retry_at", {})
            retry_map[desc_path] = time.monotonic() + 1.0
            self._peer_writeback_retry_at = retry_map

    def _clear_peer_writeback_state(self, desc_path: str):
        if not desc_path:
            return
        with self._peer_writeback_lock:
            self._peer_writeback_pending.pop(desc_path, None)
            self._peer_writeback_inflight.discard(desc_path)
            self._peer_writeback_last_sent.pop(desc_path, None)
            retry_map = getattr(self, "_peer_writeback_retry_at", {})
            retry_map.pop(desc_path, None)
            self._peer_writeback_retry_at = retry_map

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
        if event_type == "client_notify_disabled":
            chrc_path = str(info.get("chrc_path", ""))
            if chrc_path:
                for state in self._peer_time_bridges.values():
                    if state.characteristic_path == chrc_path:
                        self._clear_peer_writeback_state(state.writeback_descriptor_path)
        if event_type in {"client_descriptor_write", "client_descriptor_write_failed"}:
            desc_path = str(info.get("desc_path", ""))
            self._handle_peer_writeback_event(desc_path, event_type == "client_descriptor_write")
        if event_type.endswith("failed"):
            if event_type == "client_descriptor_write_failed":
                desc_path = str(info.get("desc_path", ""))
                if any(state.writeback_descriptor_path == desc_path for state in self._peer_time_bridges.values()):
                    for state in self._peer_time_bridges.values():
                        if state.writeback_descriptor_path != desc_path:
                            continue
                        self._run_background_once(
                            f"writeback-recover::{state.mac}",
                            self._recover_peer_writeback_path,
                            state.mac,
                            desc_path,
                        )
                    key = (event_type, desc_path)
                    now = time.monotonic()
                    last = self._last_gatt_warning_at.get(key, 0.0)
                    if now - last < 30.0:
                        return
                    self._last_gatt_warning_at[key] = now
            self.get_logger().warning(f"BLE GATT event {event_type}: {info}")

    def _handle_peer_writeback_event(self, desc_path: str, success: bool):
        if not desc_path:
            return
        with self._peer_writeback_lock:
            self._peer_writeback_inflight.discard(desc_path)
            if not success:
                self._peer_writeback_last_sent.pop(desc_path, None)
                retry_map = getattr(self, "_peer_writeback_retry_at", {})
                retry_map[desc_path] = time.monotonic() + 1.0
                self._peer_writeback_retry_at = retry_map
                return
            retry_map = getattr(self, "_peer_writeback_retry_at", {})
            retry_map.pop(desc_path, None)
            self._peer_writeback_retry_at = retry_map
            latest = self._peer_writeback_pending.get(desc_path)
            sent = self._peer_writeback_last_sent.get(desc_path)
            if latest is None:
                self._peer_writeback_last_sent.pop(desc_path, None)
                return
            if latest == sent:
                return
        self._kick_peer_writeback(desc_path)

    def _recover_peer_writeback_path(self, mac: str, failed_desc_path: str):
        state = self._peer_time_bridges.get(mac)
        if state is None:
            return
        refreshed = self._client.find_descriptor(mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=state.characteristic_path) or ""
        if not refreshed:
            return
        if refreshed != state.writeback_descriptor_path:
            self._log_verbose(
                f"Refreshing writeback descriptor path for {mac}: {state.writeback_descriptor_path} -> {refreshed}"
            )
            state.writeback_descriptor_path = refreshed
        with self._peer_writeback_lock:
            pending = self._peer_writeback_pending.pop(failed_desc_path, None)
            self._peer_writeback_inflight.discard(failed_desc_path)
            self._peer_writeback_last_sent.pop(failed_desc_path, None)
            retry_map = getattr(self, "_peer_writeback_retry_at", {})
            retry_map.pop(failed_desc_path, None)
            if pending is not None:
                self._peer_writeback_pending[refreshed] = pending
                self._peer_writeback_last_sent.pop(refreshed, None)
                retry_map.pop(refreshed, None)
            self._peer_writeback_retry_at = retry_map
        self._kick_peer_writeback(refreshed)

    def _on_pairing_event(self, event_type, device_path, **kwargs):
        self.get_logger().info(f"Pairing event {event_type} for {device_path}: {kwargs}")
        self._log_verbose(f"Pairing event {event_type} device={device_path} {kwargs}")
        if event_type not in {
            "request_confirmation",
            "request_pin",
            "request_passkey",
            "request_authorization",
            "authorize_service",
        }:
            return
        mac = self._mac_from_device_path(str(device_path or ""))
        if not mac:
            return
        now = time.monotonic()
        last = self._pairing_repair_attempts.get(mac, 0.0)
        if now - last < 20.0:
            return
        device = self._client.get_device(mac, refresh=True) if self._client is not None else None
        if device is None:
            return
        if not (device.paired or device.bonded or device.trusted):
            return
        self._pairing_repair_attempts[mac] = now
        self._repair_incoming_pairing_state(mac)

    def _repair_incoming_pairing_state(self, mac: str):
        if self._client is None:
            return
        self.get_logger().warning(
            f"Incoming pairing for {mac}: stale one-sided bond suspected, clearing local bond and re-pairing immediately"
        )
        # Keep this callback short; perform reset+re-pair in a background task.
        self._clear_peer_local_state(
            mac,
            reason="incoming pairing while already paired",
            remove_pairing=True,
            untrust=True,
        )
        scheduled = self._run_background_once(
            f"incoming-repair::{mac}",
            self._reconnect_and_repair_peer,
            mac,
        )
        if not scheduled:
            self._log_verbose(f"Incoming pairing repair already running for {mac}")

    def _reconnect_and_repair_peer(self, mac: str):
        device_label = mac
        device = self._client.get_device(mac, refresh=True)
        if device is not None:
            device_label = f"{mac} ({self._hostname_from_device(device) or '?'})"
        self._log_verbose(f"Incoming pairing recovery: reconnecting {device_label}")
        if not self._client.connect(mac, timeout=10.0):
            self.get_logger().warning(f"Incoming pairing recovery failed to reconnect {mac}")
            return
        self._client.wait_services_resolved(mac, timeout=6.0)
        paired = self._client.pair(mac, timeout=30.0)
        if not paired:
            self.get_logger().warning(f"Incoming pairing recovery failed to pair {mac}")
            return
        trusted = self._client.trust(mac)
        if trusted:
            self.get_logger().info(f"Incoming pairing recovery completed for {mac} (paired+trusted)")
        else:
            self.get_logger().warning(f"Incoming pairing recovery paired {mac} but trust step failed")

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
            key = f"time_writeback_unmanaged::{mac}"
            now = time.monotonic()
            last = self._last_dbus_warning_at.get(key, 0.0)
            if now - last >= 5.0:
                self._last_dbus_warning_at[key] = now
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
        self._shutdown_background_executor()
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
