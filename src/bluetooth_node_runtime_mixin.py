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

from .bluetooth_bridge_state import PeerRuntimeStatus, PeerTimeBridgeState, TopicExportBridgeState, TopicImportBridgeState
from .dbus_client import DeviceInfo
from .netplan import NetplanConfiguration
from .gatt_services import (
    TIME_CHARACTERISTIC_UUID,
    TIME_WRITE_CHARACTERISTIC_UUID,
    topic_bridge_characteristic_uuid,
)
from .uuid_utils import is_uav_hostname


class BluetoothNodeRuntimeMixin:
    DEVICE_QUEUE_DEPTH = 10
    NOTIFICATION_QUEUE_DEPTH = 50
    STATUS_QUEUE_DEPTH = 50
    LOG_QUEUE_DEPTH = 200
    BRIDGE_QUEUE_DEPTH = 10

    DBUS_WARNING_INTERVAL_S = 5.0
    GATT_WARNING_INTERVAL_S = 30.0
    CONNECT_RETRY_MIN_S = 1.0
    CONNECT_REQUEST_TIMEOUT_S = 6.0
    DISCONNECT_TIMEOUT_S = 5.0
    PAIR_REQUEST_TIMEOUT_S = 12.0
    PEER_SETUP_TIMEOUT_S = 30.0
    LIVE_BRIDGE_IDLE_TIMEOUT_S = 20.0
    PAIRING_REPAIR_COOLDOWN_S = 20.0
    WRITEBACK_RETRY_DELAY_S = 1.0
    TIME_WRITEBACK_WARNING_INTERVAL_S = 5.0
    LOG_PAYLOAD_PREVIEW_BYTES = 24

    def _ensure_core_publishers(self):
        if self.devices_pub is not None:
            return
        self.devices_pub = self.create_publisher(BleDeviceArray, self._node_topic("devices"), self.DEVICE_QUEUE_DEPTH)
        self.notifications_pub = self.create_publisher(
            BleNotification,
            self._node_topic("notifications"),
            self.NOTIFICATION_QUEUE_DEPTH,
        )
        self.status_pub = self.create_publisher(String, self._node_topic("status"), self.STATUS_QUEUE_DEPTH)
        self.log_pub = self.create_publisher(String, self._node_topic("log"), self.LOG_QUEUE_DEPTH)

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
        self._client.add_device_handler(self._on_client_device_event)
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
        self._update_scan_state(reason="runtime config")
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
                self.BRIDGE_QUEUE_DEPTH,
            )
            self._topic_exports[key] = TopicExportBridgeState(
                topic_name=shared.export_topic,
                message_type=shared.message_type,
                message_class=shared.message_class,
                bridge_name=shared.bridge_name,
                bridge_key=shared.bridge_key,
                bridge_uuid=topic_bridge_characteristic_uuid(shared.bridge_name),
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

    def _idle_scan_window_s(self) -> float:
        return max(0.5, float(self.get_parameter("scan_publish_period").value))

    def _idle_scan_interval_s(self) -> float:
        return max(self._idle_scan_window_s(), float(self.get_parameter("auto_connect_period").value))

    def _should_scan_for_peer_setup(self, snapshot: Dict[str, DeviceInfo] = None) -> bool:
        if self._client is None:
            return False
        if not bool(self.get_parameter("enable_scan").value):
            return False

        snapshot = snapshot or self._client.get_devices()
        whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
        whitelist_enabled = bool(whitelist_names or whitelist_macs)
        pattern = str(self.get_parameter("auto_connect_pattern").value)

        has_known_target = False
        has_disconnected_target = False
        has_connected_target = False

        for _, device in snapshot.items():
            peer_candidate = self._is_uav_peer_candidate(device, pattern)
            explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
            should_connect = explicit_target or (peer_candidate and not whitelist_enabled)
            if not should_connect:
                continue
            has_known_target = True
            if device.connected:
                has_connected_target = True
            else:
                has_disconnected_target = True

        if has_connected_target:
            return False
        if self._peer_repair_reasons:
            return True
        return has_disconnected_target or not has_known_target

    def _should_defer_scan_refresh(self, snapshot: Dict[str, DeviceInfo] = None) -> bool:
        if self._client is None:
            return False

        snapshot = snapshot or self._client.get_devices()
        whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
        whitelist_enabled = bool(whitelist_names or whitelist_macs)
        pattern = str(self.get_parameter("auto_connect_pattern").value)

        for mac, device in snapshot.items():
            peer_candidate = self._is_uav_peer_candidate(device, pattern)
            explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
            should_connect = explicit_target or (peer_candidate and not whitelist_enabled)
            if not should_connect or not device.connected:
                continue
            state = self._peer_status.get(mac)
            if state is None or state.status != "ready":
                return True
        return False

    def _update_scan_state(self, snapshot: Dict[str, DeviceInfo] = None, *, reason: str = ""):
        if self._client is None:
            return

        desired_transport = str(self.get_parameter("scan_mode").value)
        continuous_scan = self._should_scan_for_peer_setup(snapshot)

        if continuous_scan:
            self._idle_scan_started_at = 0.0
            self._idle_scan_next_allowed_at = 0.0
            if self._client.scanning and self._scan_transport == desired_transport:
                return
            if self._start_scan(desired_transport):
                if reason:
                    self._log_verbose(f"Scan enabled: {reason}")
            return

        if self._client.scanning:
            if self._stop_scan():
                self._idle_scan_started_at = 0.0
                self._idle_scan_next_allowed_at = 0.0
                if reason:
                    self._log_verbose(f"Scan paused: {reason}")
        return

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

    def _dbus_warning(self, key: str, message: str, interval_s: float = None):
        if interval_s is None:
            interval_s = self.DBUS_WARNING_INTERVAL_S
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
        details = [
            f"conn={'Y' if device.connected else 'N'}",
            f"pair={'Y' if device.paired else 'N'}",
            f"bond={'Y' if device.bonded else 'N'}",
            f"trust={'Y' if device.trusted else 'N'}",
            f"svc={'Y' if device.services_resolved else 'N'}",
        ]
        if device.rssi is not None:
            details.append(f"rssi={device.rssi}")
        if device.tx_power is not None:
            details.append(f"tx={device.tx_power}")
        if device.pathloss is not None:
            details.append(f"pathloss={device.pathloss}")
        detail_text = ", ".join(details)
        if prefer_hostname:
            hostname = self._hostname_from_device(device)
            if self._is_valid_log_name(hostname):
                return f"{mac} ({hostname}) [{detail_text}]"
            return f"{mac} [{detail_text}]"
        label = ""
        for candidate in (device.name or "", device.alias or ""):
            if self._is_valid_log_name(candidate):
                label = candidate
                break
        if label:
            return f"{mac} ({label}) [{detail_text}]"
        return f"{mac} [{detail_text}]"

    def _format_payload_preview(self, payload: bytes, *, limit: int = None) -> str:
        preview_bytes = bytes(payload or b"")
        if limit is None:
            limit = self.LOG_PAYLOAD_PREVIEW_BYTES
        clipped = preview_bytes[: max(0, int(limit))]
        hex_preview = clipped.hex()
        if len(preview_bytes) > len(clipped):
            hex_preview = f"{hex_preview}..."
        if not hex_preview:
            hex_preview = "-"
        return f"len={len(preview_bytes)} hex={hex_preview}"

    def _format_gatt_event_log(self, event_type: str, info: dict) -> str:
        info = dict(info or {})
        path = str(info.get("chrc_path") or info.get("desc_path") or "")
        pieces = [f"event={event_type}"]
        if path:
            pieces.append(f"path={path}")
        uuid = str(info.get("uuid") or "")
        if uuid:
            pieces.append(f"uuid={uuid}")
        if "with_response" in info:
            pieces.append(f"with_response={'Y' if info.get('with_response') else 'N'}")
        if "recovered_from" in info:
            pieces.append(f"recovered_from={info.get('recovered_from')}")
        value = info.get("value")
        if value is not None:
            pieces.append(self._format_payload_preview(value))
        error = str(info.get("error") or "")
        if error:
            pieces.append(f"error={error}")
        return "GATT " + " ".join(pieces)

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
            cached_snapshot = self._client.get_devices(refresh=False)
            defer_scan_refresh = self._should_defer_scan_refresh(cached_snapshot)
            snapshot = self._client.get_devices(refresh=not defer_scan_refresh)
            if not defer_scan_refresh:
                snapshot = self._enforce_peer_connection_policy(snapshot=snapshot, reason="scan policy tick")
            self._progress_peer_repairs(snapshot)
            self._sync_auto_import_bridges(snapshot)
            self._reconcile_import_bridges(snapshot)
            msg = BleDeviceArray()
            msg.header = self._header(frame_id=self._local_frame_id())
            msg.devices = [self._device_to_msg(device) for device in snapshot.values()]
            self.devices_pub.publish(msg)
            self._refresh_notification_mapping(snapshot)
            self._cleanup_peer_time_bridges(snapshot)
            if defer_scan_refresh:
                self._log_verbose("Scan refresh deferred: peer setup still in progress")
            else:
                self._log_verbose(f"Scan results: {len(snapshot)} device(s)")
            self._log_discovered_devices_summary(
                snapshot,
                pattern=str(self.get_parameter("auto_connect_pattern").value),
            )
            self._update_scan_state(snapshot, reason="scan publish")
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
                bridge_name = self._bridge_topic_name_for_peer(shared, mac, device=device)
                if not bridge_name:
                    continue
                path, bridge_uuid, transport_endpoint = self._resolve_remote_characteristic(
                    mac, bridge_name, shared.transport_endpoint
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
                    publisher = self.create_publisher(shared.message_class, resolved_topic_name, self.BRIDGE_QUEUE_DEPTH)
                    self._notification_bridges[key] = TopicImportBridgeState(
                        mac=mac,
                        requested_topic_name=shared.import_topic_suffix,
                        resolved_topic_name=resolved_topic_name,
                        message_type=shared.message_type,
                        message_class=shared.message_class,
                        bridge_name=bridge_name,
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
                    replacement = self.create_publisher(shared.message_class, resolved_topic_name, self.BRIDGE_QUEUE_DEPTH)
                    self.destroy_publisher(state.publisher)
                    state.publisher = replacement
                state.requested_topic_name = shared.import_topic_suffix
                state.resolved_topic_name = resolved_topic_name
                state.message_type = shared.message_type
                state.message_class = shared.message_class
                state.bridge_name = bridge_name
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

    def _ensure_peer_status(self, mac: str, device: DeviceInfo = None) -> PeerRuntimeStatus:
        current = device or (self._client.get_device(mac) if self._client is not None else None)
        peer_name = self._peer_name_token(mac, device=current)
        status = self._peer_status.get(mac)
        if status is None:
            status = PeerRuntimeStatus(mac=mac, peer_name=peer_name)
            self._peer_status[mac] = status
        else:
            status.peer_name = peer_name
        return status

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
                    self._client.disconnect(mac, timeout=self.DISCONNECT_TIMEOUT_S)
                    stale_macs.append(mac)
                continue
            stale_macs.append(mac)
        for mac in stale_macs:
            self._drop_peer_runtime_state(mac)

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
                if notifying:
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
        for characteristic in self._client.list_characteristics(mac):
            if characteristic["uuid"].lower() != bridge_uuid.lower():
                continue
            return characteristic["path"], bool(characteristic.get("notifying", False))
        return "", False

    def _auto_connect_devices(self):
        if self._client is None or not bool(self.get_parameter("auto_connect_enable").value):
            return
        try:
            retry_period = max(self.CONNECT_RETRY_MIN_S, float(self.get_parameter("auto_connect_period").value))
            whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
            whitelist_enabled = bool(whitelist_names or whitelist_macs)
            pattern = str(self.get_parameter("auto_connect_pattern").value)
            cached_snapshot = self._client.get_devices(refresh=False)
            snapshot = self._client.get_devices(refresh=not self._should_defer_scan_refresh(cached_snapshot))
            self._prune_peer_runtime_state(snapshot)
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
                    self._auto_connect_attempts.pop(mac, None)
                    if should_connect:
                        self._maintain_peer_connection(mac, device, retry_period)
                    else:
                        self._clear_peer_setup_state(mac)
                    continue
                if not should_connect:
                    self._clear_peer_setup_state(mac)
                    continue
                last_attempt = self._auto_connect_attempts.get(mac, 0.0)
                if time.monotonic() - last_attempt < retry_period:
                    continue
                self._auto_connect_attempts[mac] = time.monotonic()
                self._mark_peer_setup_pending(mac)
                self._log_verbose(f"Auto-connect attempt: {device_label}")
                if not self._client.connect_async(mac, timeout=self.CONNECT_REQUEST_TIMEOUT_S):
                    self._log_verbose(f"Auto-connect request rejected: {device_label}")
            self._update_scan_state(snapshot, reason="auto-connect tick")
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("auto_connect", f"Skipping auto-connect tick due to DBus error: {exc}")

    def _drop_non_whitelisted_peer(self, mac: str, device: DeviceInfo):
        self._clear_peer_local_state(mac, device=device, reason="non-whitelisted peer")

    def _mark_peer_setup_pending(self, mac: str) -> float:
        return self._peer_setup_started_at.setdefault(mac, time.monotonic())

    def _clear_peer_setup_state(self, mac: str):
        self._peer_setup_started_at.pop(mac, None)
        self._peer_pair_requested_at.pop(mac, None)

    def _complete_peer_pair_repair(self, mac: str):
        reason = self._peer_repair_reasons.get(mac, "")
        if reason:
            self._log_verbose(f"Completed peer pair repair for {mac}: {reason}")
        self._peer_repair_reasons.pop(mac, None)
        self._clear_peer_setup_state(mac)

    def _drop_peer_runtime_state(self, mac: str):
        for key, bridge in list(self._notification_bridges.items()):
            if bridge.mac == mac:
                self._remove_import_bridge(key, bridge)
        state = self._peer_time_bridges.pop(mac, None)
        if state is not None:
            self.destroy_publisher(state.publisher)
            self._stop_notify_if_unused(state.characteristic_path)
            self._clear_peer_writeback_state(state.writeback_characteristic_path)
        self._clear_peer_setup_state(mac)
        self._auto_connect_attempts.pop(mac, None)
        self._peer_status.pop(mac, None)

    def _request_peer_pair_repair(
        self,
        mac: str,
        reason: str,
        *,
        device: DeviceInfo = None,
        reset_local: bool = True,
    ):
        current = device or (self._client.get_device(mac, refresh=True) if self._client is not None else None)
        device_label = f"{mac} ({self._hostname_from_device(current) or '?'})" if current is not None else mac
        self._drop_peer_runtime_state(mac)
        if reset_local and current is not None:
            self._log_verbose(
                f"Resetting local pairing for {device_label}: "
                f"paired={current.paired} bonded={current.bonded} trusted={current.trusted}"
            )
            if current.trusted:
                self._client.untrust(mac)
            if current.paired or current.bonded or current.trusted:
                self._client.remove(mac)
        self._pairing_repair_attempts[mac] = time.monotonic()
        self._peer_repair_reasons[mac] = reason
        self._mark_peer_setup_pending(mac)
        self._set_peer_time_status(mac, "repairing", reason)
        self._log_verbose(f"Requested pair repair for {device_label}: {reason}")
        self._progress_peer_repair(mac, current)

    def _progress_peer_repair(self, mac: str, device: DeviceInfo = None) -> bool:
        reason = self._peer_repair_reasons.get(mac)
        if reason is None or self._client is None:
            return False
        current = device or self._client.get_device(mac, refresh=True)
        if current is None:
            self._log_verbose(f"Pair repair waiting for device object: {mac} ({reason})")
            return False
        device_label = f"{mac} ({self._hostname_from_device(current) or '?'})"
        if current.trusted and (current.paired or current.bonded):
            self._complete_peer_pair_repair(mac)
            self._set_peer_time_status(mac, "paired")
            self._log_verbose(f"Pair repair completed for {device_label}")
            return True
        self._mark_peer_setup_pending(mac)
        if not current.connected:
            self._log_verbose(f"Pair repair waiting for reconnect: {device_label} reason={reason}")
            if self._client.connect_async(mac, timeout=self.CONNECT_REQUEST_TIMEOUT_S):
                self._set_peer_time_status(mac, "repairing", reason)
                self._log_verbose(f"Reconnect requested for {device_label}")
            else:
                self._log_verbose(f"Reconnect request not submitted for {device_label}")
            return False
        if not (current.paired or current.bonded):
            last_attempt = self._peer_pair_requested_at.get(mac, 0.0)
            if time.monotonic() - last_attempt < self.CONNECT_RETRY_MIN_S:
                self._set_peer_time_status(mac, "repairing", reason)
                self._log_verbose(f"Pair repair pending pair completion for {device_label}")
                return False
            self._peer_pair_requested_at[mac] = time.monotonic()
            if self._client.pair_async(mac, timeout=self.PAIR_REQUEST_TIMEOUT_S):
                self._set_peer_time_status(mac, "repairing", reason)
                self._log_verbose(f"Pair requested for repair on {device_label}")
            else:
                self._set_peer_time_status(mac, "failed", reason)
                self._log_verbose(f"Pair request rejected during repair for {device_label}")
            return False
        if not current.trusted:
            self._log_verbose(f"Pair repair waiting for trust on {device_label}")
            if self._client.trust(mac):
                refreshed = self._client.get_device(mac, refresh=True) or current
                if refreshed.trusted:
                    self._complete_peer_pair_repair(mac)
                    self._set_peer_time_status(mac, "paired")
                    self._log_verbose(f"Pair repair trusted {device_label}")
                    return True
                self._log_verbose(f"Trust requested for {device_label}, awaiting property update")
            else:
                self._set_peer_time_status(mac, "failed", reason)
                self._log_verbose(f"Trust request failed during repair for {device_label}")
            return False
        return False

    def _progress_peer_repairs(self, snapshot: Dict[str, DeviceInfo]):
        if not self._peer_repair_reasons:
            return
        for mac in list(self._peer_repair_reasons.keys()):
            self._progress_peer_repair(mac, snapshot.get(mac))

    def _clear_peer_local_state(
        self,
        mac: str,
        device: DeviceInfo = None,
        *,
        reason: str = "",
        remove_pairing: bool = True,
        untrust: bool = True,
    ):
        current = device or self._client.get_device(mac, refresh=True)
        device_label = f"{mac} ({self._hostname_from_device(current) or '?'})" if current is not None else mac
        self._log_verbose(
            f"Clearing local peer state for {device_label}: reason={reason or 'n/a'} "
            f"remove_pairing={remove_pairing} untrust={untrust}"
        )
        self._client.disconnect_async(mac)
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
        self._drop_peer_runtime_state(mac)
        self._peer_repair_reasons.pop(mac, None)

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
        self._client.get_device(mac, refresh=True)

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

    def _prune_peer_runtime_state(self, snapshot: Dict[str, DeviceInfo]):
        current_macs = set(snapshot.keys())
        self._auto_connect_attempts = {
            mac: stamp for mac, stamp in self._auto_connect_attempts.items() if mac in current_macs
        }
        self._peer_setup_started_at = {
            mac: stamp for mac, stamp in self._peer_setup_started_at.items() if mac in current_macs
        }
        self._peer_pair_requested_at = {
            mac: stamp for mac, stamp in self._peer_pair_requested_at.items() if mac in current_macs
        }
        self._pairing_repair_attempts = {
            mac: stamp for mac, stamp in self._pairing_repair_attempts.items() if mac in current_macs
        }
        self._peer_repair_reasons = {
            mac: reason for mac, reason in self._peer_repair_reasons.items() if mac in current_macs
        }

    def _has_live_peer_time_bridge(self, mac: str, *, idle_timeout_s: float = LIVE_BRIDGE_IDLE_TIMEOUT_S) -> bool:
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
        if self._has_live_peer_time_bridge(mac):
            return True
        if self._resolve_peer_time_paths(mac)[0]:
            return True
        if device.services_resolved:
            return True
        wait_started = self._mark_peer_setup_pending(mac)
        self._set_peer_time_status(
            mac,
            "waiting-services",
            wait_started=wait_started,
            wait_grace_s=self.PEER_SETUP_TIMEOUT_S,
        )
        self._log_verbose(f"Peer {device_label}: services not resolved yet, delaying time bridge setup")
        return False

    def _resolve_peer_time_paths(self, mac: str) -> Tuple[str, str]:
        def _find_paths() -> Tuple[str, str]:
            time_path = ""
            writeback_path = ""
            for characteristic in self._client.list_characteristics(mac):
                uuid = str(characteristic.get("uuid") or "").lower()
                path = str(characteristic.get("path") or "")
                if not path:
                    continue
                if not time_path and uuid == TIME_CHARACTERISTIC_UUID.lower():
                    time_path = path
                elif not writeback_path and uuid == TIME_WRITE_CHARACTERISTIC_UUID.lower():
                    writeback_path = path
                if time_path and writeback_path:
                    break
            return time_path, writeback_path

        time_path, writeback_path = _find_paths()
        if time_path and writeback_path:
            return time_path, writeback_path
        self._client.get_device(mac, refresh=True)
        refreshed_time_path, refreshed_writeback_path = _find_paths()
        return refreshed_time_path, refreshed_writeback_path

    def _ensure_peer_time_bridge(self, mac: str, device: DeviceInfo) -> bool:
        current = self._client.get_device(mac, refresh=True) or device
        device_label = f"{mac} ({self._hostname_from_device(device) or '?'})"
        existing = self._peer_time_bridges.get(mac)
        if existing is not None and self._has_live_peer_time_bridge(mac):
            self._clear_peer_setup_state(mac)
            self._set_peer_time_status(mac, "ready")
            return True
        if not self._ensure_peer_services_resolved(mac, current, device_label=device_label):
            return False

        path, writeback_characteristic_path = self._resolve_peer_time_paths(mac)
        if not path:
            self._log_verbose(f"Peer {device_label}: time characteristic {TIME_CHARACTERISTIC_UUID} not found")
            wait_started = self._mark_peer_setup_pending(mac)
            self._set_peer_time_status(
                mac,
                "waiting-time",
                wait_started=wait_started,
                wait_grace_s=self.PEER_SETUP_TIMEOUT_S,
            )
            return False
        if not writeback_characteristic_path:
            wait_started = self._mark_peer_setup_pending(mac)
            self._set_peer_time_status(
                mac,
                "waiting-time",
                wait_started=wait_started,
                wait_grace_s=self.PEER_SETUP_TIMEOUT_S,
            )
            self._log_verbose(f"Peer {device_label}: writeback characteristic {TIME_WRITE_CHARACTERISTIC_UUID} not found")
            return False
        started, path = self._start_notify_with_refresh(
            mac,
            path,
            TIME_CHARACTERISTIC_UUID,
            label="peer time characteristic",
        )
        if not started:
            wait_started = self._mark_peer_setup_pending(mac)
            self._set_peer_time_status(
                mac,
                "waiting-time",
                wait_started=wait_started,
                wait_grace_s=self.PEER_SETUP_TIMEOUT_S,
            )
            self._log_verbose(f"Time notify still pending for {device_label}: path={path}")
            return False
        peer_name = self._peer_name_token(mac, device=device)
        status_topic_name = self._resolve_peer_topic_name(mac, "/time_status", device=device)
        if re.search(r"/peers/[0-9]", status_topic_name):
            status_topic_name = self._node_topic(f"peers/{peer_name}/time_status")
        new_bridge = existing is None
        if existing is not None:
            old_path = existing.characteristic_path
            if existing.status_topic_name != status_topic_name:
                self.destroy_publisher(existing.publisher)
                existing.publisher = self.create_publisher(BlePeerTimeStatus, status_topic_name, self.BRIDGE_QUEUE_DEPTH)
            existing.peer_name = peer_name
            existing.status_topic_name = status_topic_name
            existing.characteristic_path = path
            existing.writeback_characteristic_path = writeback_characteristic_path
            if old_path and old_path != path:
                self._stop_notify_if_unused(old_path)
            state = existing
        else:
            state = PeerTimeBridgeState(
                mac=mac,
                peer_name=peer_name,
                status_topic_name=status_topic_name,
                characteristic_path=path,
                writeback_characteristic_path=writeback_characteristic_path,
                publisher=self.create_publisher(BlePeerTimeStatus, status_topic_name, self.BRIDGE_QUEUE_DEPTH),
            )
            self._peer_time_bridges[mac] = state
        state.characteristic_path = path
        self._clear_peer_setup_state(mac)
        self._set_peer_time_status(mac, "ready")
        if new_bridge:
            self.get_logger().info(f"Established time bridge with peer {device_label} -> {status_topic_name}")
        self._log_verbose(
            f"Time bridge: {device_label} chrc={path} writeback={state.writeback_characteristic_path or 'none'}"
            f" topic={status_topic_name}"
        )
        self._prime_peer_time_bridge(state)
        return True

    def _prime_peer_time_bridge(self, state: PeerTimeBridgeState):
        current = self._client.get_device(state.mac, refresh=True)
        if current is None or not current.connected:
            return
        payload = self._client.read_characteristic(state.characteristic_path)
        if payload is not None:
            self._process_peer_time_payload(state.mac, state.characteristic_path, payload)

    def _maintain_peer_connection(self, mac: str, device: DeviceInfo, retry_period: float) -> bool:
        current = self._client.get_device(mac, refresh=True) or device
        if mac in self._peer_repair_reasons and not self._progress_peer_repair(mac, current):
            return False
        if not self._ensure_peer_security(mac, current, retry_period):
            return False
        if self._is_uav_peer_candidate(device, str(self.get_parameter("auto_connect_pattern").value)):
            return self._ensure_peer_time_bridge(mac, current)
        self._clear_peer_setup_state(mac)
        self._set_peer_time_status(mac, "paired")
        return True

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
        current = self._client.get_device(mac, refresh=True) or device
        security_ready = bool(current.trusted and (current.paired or current.bonded))
        if security_ready:
            self._peer_pair_requested_at.pop(mac, None)
            return True
        self._mark_peer_setup_pending(mac)
        device_label = f"{mac} ({self._hostname_from_device(current) or '?'})"
        if (current.paired or current.bonded) and not current.trusted:
            if self._client.trust(mac):
                self.get_logger().info(f"Trusted BLE peer {mac}")
                self._log_verbose(f"Trusted: {device_label}")
                current = self._client.get_device(mac, refresh=True) or current
            else:
                self.get_logger().warning(f"Failed to trust BLE peer {mac}")
                self._log_verbose(f"Trust failed: {device_label}")
                self._set_peer_time_status(mac, "failed", "trust")
                return False
        if current.trusted and (current.paired or current.bonded):
            self._peer_pair_requested_at.pop(mac, None)
            return True
        now = time.monotonic()
        last_attempt = self._peer_pair_requested_at.get(mac, 0.0)
        if now - last_attempt < retry_period:
            self._set_peer_time_status(mac, "pairing", f"timeout={retry_period:.1f}s")
            return False
        self._peer_pair_requested_at[mac] = now
        self._set_peer_time_status(mac, "pairing", f"timeout={self.PAIR_REQUEST_TIMEOUT_S:.1f}s")
        self._log_verbose(
            f"Security state for {device_label}: paired={current.paired} trusted={current.trusted} bonded={current.bonded}"
        )
        if self._client.pair_async(mac, timeout=self.PAIR_REQUEST_TIMEOUT_S):
            self._log_verbose(f"Pair requested for {device_label}")
            return False
        self.get_logger().warning(f"Repairing peer {mac}: pair request rejected, clearing stale local bond")
        self._request_peer_pair_repair(mac, "pair request rejected", device=current, reset_local=True)
        return False

    def _set_peer_time_status(
        self,
        mac: str,
        status: str,
        detail: str = "",
        *,
        wait_started: float = None,
        wait_grace_s: float = None,
    ):
        runtime_status = self._ensure_peer_status(mac)
        previous_status = runtime_status.status
        previous_detail = runtime_status.detail
        runtime_status.status = status
        runtime_status.detail = detail or ""
        runtime_status.wait_started_monotonic = max(0.0, float(wait_started)) if wait_started is not None else 0.0
        runtime_status.wait_timeout_s = max(0.0, float(wait_grace_s)) if wait_grace_s is not None else 0.0
        state = self._peer_time_bridges.get(mac)
        if state is not None:
            state.status = runtime_status.status
            state.detail = runtime_status.detail
            state.wait_started_monotonic = runtime_status.wait_started_monotonic
            state.wait_timeout_s = runtime_status.wait_timeout_s
        if previous_status != runtime_status.status or previous_detail != runtime_status.detail:
            self._log_verbose(
                f"Peer status {mac}: {previous_status or 'none'} -> {runtime_status.status}; detail={runtime_status.detail or '-'}"
            )

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
        details = [
            f"Notification path={chrc_path}",
            f"uuid={uuid or '-'}",
            f"mac={mac or '-'}",
            self._format_payload_preview(data),
        ]
        self._log_verbose(" ".join(details))
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
        if state.writeback_characteristic_path:
            self._update_peer_writeback_latency(state, data)

    def _update_peer_writeback_latency(self, state: PeerTimeBridgeState, payload: bytes):
        if not state.writeback_characteristic_path:
            return
        desc_path = state.writeback_characteristic_path
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
        if self._client.write_characteristic_async(desc_path, payload, with_response=False):
            return
        with self._peer_writeback_lock:
            self._peer_writeback_inflight.discard(desc_path)
            self._peer_writeback_last_sent.pop(desc_path, None)
            retry_map = getattr(self, "_peer_writeback_retry_at", {})
            retry_map[desc_path] = time.monotonic() + self.WRITEBACK_RETRY_DELAY_S
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
        self._log_verbose(self._format_gatt_event_log(event_type, info))
        if event_type == "client_notify_disabled":
            chrc_path = str(info.get("chrc_path", ""))
            if chrc_path:
                for state in self._peer_time_bridges.values():
                    if state.characteristic_path == chrc_path:
                        self._clear_peer_writeback_state(state.writeback_characteristic_path)
        if event_type in {"client_write", "client_write_failed"}:
            desc_path = str(info.get("chrc_path", ""))
            if any(state.writeback_characteristic_path == desc_path for state in self._peer_time_bridges.values()):
                self._handle_peer_writeback_event(desc_path, event_type == "client_write")
        if event_type.endswith("failed"):
            if event_type == "client_write_failed":
                desc_path = str(info.get("chrc_path", ""))
                if any(state.writeback_characteristic_path == desc_path for state in self._peer_time_bridges.values()):
                    for state in self._peer_time_bridges.values():
                        if state.writeback_characteristic_path != desc_path:
                            continue
                        self._recover_peer_writeback_path(state.mac, desc_path)
                    key = (event_type, desc_path)
                    now = time.monotonic()
                    last = self._last_gatt_warning_at.get(key, 0.0)
                    if now - last < self.GATT_WARNING_INTERVAL_S:
                        return
                    self._last_gatt_warning_at[key] = now
            self.get_logger().warning(f"BLE GATT event {event_type}: {info}")

    def _on_client_device_event(self, mac: str, device: DeviceInfo, changed_fields):
        relevant_fields = {"added", "removed", "connected", "paired", "bonded", "trusted", "services_resolved"}
        if not any(field in relevant_fields for field in changed_fields):
            return
        self._log_verbose(
            f"Device event {mac}: changed={list(changed_fields)} connected={device.connected} "
            f"paired={device.paired} bonded={device.bonded} trusted={device.trusted} "
            f"services_resolved={device.services_resolved}"
        )
        if mac in self._peer_repair_reasons:
            self._progress_peer_repair(mac, device)
            self._update_scan_state(reason=f"device event {mac} repair")
            return
        if not device.connected:
            self._log_verbose(f"Removing peer runtime state for disconnected device {mac}")
            if not device.paired:
                self._drop_peer_runtime_state(mac)
            else:
                self._clear_peer_local_state(mac, device=device, reason="Unpairing on disconnection event")
            self._update_scan_state(reason=f"device event {mac} disconnected and unpaired")
            return
        whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
        whitelist_enabled = bool(whitelist_names or whitelist_macs)
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        peer_candidate = self._is_uav_peer_candidate(device, pattern)
        explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
        should_connect = explicit_target or (peer_candidate and not whitelist_enabled)
        if should_connect:
            retry_period = max(self.CONNECT_RETRY_MIN_S, float(self.get_parameter("auto_connect_period").value))
            self._log_verbose(f"Device event triggers peer maintenance for {mac}")
            self._maintain_peer_connection(mac, device, retry_period)
        self._update_scan_state(reason=f"device event {mac}")

    def _handle_peer_writeback_event(self, desc_path: str, success: bool):
        if not desc_path:
            return
        with self._peer_writeback_lock:
            self._peer_writeback_inflight.discard(desc_path)
            if not success:
                self._peer_writeback_last_sent.pop(desc_path, None)
                retry_map = getattr(self, "_peer_writeback_retry_at", {})
                retry_map[desc_path] = time.monotonic() + self.WRITEBACK_RETRY_DELAY_S
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
        _, refreshed = self._resolve_peer_time_paths(mac)
        if not refreshed:
            return
        if refreshed != state.writeback_characteristic_path:
            self._log_verbose(
                f"Refreshing writeback characteristic path for {mac}: {state.writeback_characteristic_path} -> {refreshed}"
            )
            state.writeback_characteristic_path = refreshed
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
        if now - last < self.PAIRING_REPAIR_COOLDOWN_S:
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
        current = self._client.get_device(mac, refresh=True)
        self._request_peer_pair_repair(mac, "incoming pairing request", device=current, reset_local=True)

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
            if now - last >= self.TIME_WRITEBACK_WARNING_INTERVAL_S:
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
                    self._client.disconnect_async(device.mac)
                except Exception:
                    pass
        for state in self._topic_exports.values():
            self._destroy_export_bridge(state)
            self.destroy_subscription(state.subscription)
        for key, state in list(self._notification_bridges.items()):
            self._remove_import_bridge(key, state)
        for state in self._peer_time_bridges.values():
            self.destroy_publisher(state.publisher)
        self._peer_status.clear()
        self._notification_bridges.clear()
        self._peer_time_bridges.clear()
        self._topic_exports.clear()
        self._dbus.shutdown(self._topic_exports)
