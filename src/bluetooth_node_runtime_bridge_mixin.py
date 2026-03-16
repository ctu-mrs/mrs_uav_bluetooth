"""BLE import/export bridge and notification helpers for the Bluetooth node."""

import struct
import time

import dbus

from mrs_uav_bluetooth.msg import BleNotification

from .bluetooth_bridge_state import TopicExportBridgeState, TopicImportBridgeState
from .bridge_payload import bytes_to_serializable, member_specs_from_serializable
from .gatt_services import (
    TIME_WRITEBACK_DESCRIPTOR_UUID,
    topic_bridge_characteristic_uuid,
    topic_bridge_data_descriptor_uuid,
    topic_bridge_metadata_descriptor_uuid,
)


class BluetoothNodeRuntimeBridgeMixin:

    IMPORT_BRIDGE_MISSING_PATH_GRACE_MIN_S = 8.0
    IMPORT_BRIDGE_MISSING_PATH_GRACE_MULTIPLIER = 4.0
    PEER_WRITEBACK_RETRY_DELAY_S = 1.0
    PEER_WRITEBACK_WARNING_INTERVAL_S = 30.0
    DESCRIPTOR_POLL_FALLBACK_PERIOD_S = 1.0
    NOTIFY_REFRESH_WAIT_S = 1.5
    EXPORT_SUBSCRIPTION_QUEUE_SIZE = 10
    IMPORT_PUBLISH_QUEUE_SIZE = 10

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
                self.EXPORT_SUBSCRIPTION_QUEUE_SIZE,
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

    def _sync_auto_import_bridges(self, snapshot):
        desired_keys = set()
        now_mono = time.monotonic()
        retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
        missing_path_grace_s = max(
            self.IMPORT_BRIDGE_MISSING_PATH_GRACE_MIN_S,
            retry_period * self.IMPORT_BRIDGE_MISSING_PATH_GRACE_MULTIPLIER,
        )
        for shared in self._shared_topic_configs.values():
            if shared.mode not in {"import", "both"}:
                continue
            for mac, device in snapshot.items():
                if not self._is_peer_effectively_connected(mac, device=device):
                    continue
                if (device.alias or device.name or "").strip().lower() == self._local_name.strip().lower():
                    continue
                path, bridge_uuid, transport_endpoint = self._resolve_remote_characteristic(
                    mac, shared.bridge_name, shared.transport_endpoint
                )
                key = f"config-import::{shared.bridge_key}::{mac}"
                state = self._notification_bridges.get(key)
                session = self._get_peer_session(mac, device=device)
                device_label = f"{mac} ({device.alias or device.name or '?'})"
                if not path:
                    if state is None:
                        session.import_bridge_missing_since.pop(key, None)
                        continue
                    missing_since = session.import_bridge_missing_since.setdefault(key, now_mono)
                    if now_mono - missing_since < missing_path_grace_s:
                        desired_keys.add(key)
                        self._log_verbose(
                            f"Keeping import bridge while waiting for refreshed path: {device_label} "
                            f"({now_mono - missing_since:.1f}s/{missing_path_grace_s:.1f}s)"
                        )
                        continue
                    session.import_bridge_missing_since.pop(key, None)
                    self._log_verbose(f"Removing import bridge (no path): {device_label}")
                    self._remove_import_bridge(key, state)
                    continue
                session.import_bridge_missing_since.pop(key, None)
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
                        self._remember_notification_path(mac, path)
                    self._log_verbose(f"Creating import bridge: {device_label} -> {resolved_topic_name}")
                    publisher = self.create_publisher(
                        shared.message_class,
                        resolved_topic_name,
                        self.IMPORT_PUBLISH_QUEUE_SIZE,
                    )
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
                    self._remember_notification_path(mac, path)
                if state.resolved_topic_name != resolved_topic_name or state.message_type != shared.message_type:
                    replacement = self.create_publisher(
                        shared.message_class,
                        resolved_topic_name,
                        self.IMPORT_PUBLISH_QUEUE_SIZE,
                    )
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
            session = self._peer_sessions.get(self._notification_bridges[key].mac)
            if session is not None:
                session.import_bridge_missing_since.pop(key, None)
            self._remove_import_bridge(key, self._notification_bridges[key])

    def _remove_import_bridge(self, key: str, state: TopicImportBridgeState):
        session = self._peer_sessions.get(state.mac)
        if session is not None:
            session.import_bridge_missing_since.pop(key, None)
        self._notification_bridges.pop(key, None)
        self._destroy_import_bridge(state)
        self.destroy_publisher(state.publisher)
        if state.transport_endpoint == "characteristic":
            self._stop_notify_if_unused(state.path)

    def _refresh_notification_mapping(self, snapshot):
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

    def _remember_notification_path(self, mac: str, path: str):
        if not mac or not str(path or "").startswith("/"):
            return
        self._notification_path_to_mac[path] = mac

    def _reconcile_import_bridges(self, snapshot):
        for state in self._notification_bridges.values():
            device = snapshot.get(state.mac)
            if device is None or not self._is_peer_effectively_connected(state.mac, device=device):
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

    def _start_notify_with_refresh(self, mac: str, path: str, characteristic_uuid: str, *, label: str):
        tried_paths = []

        def _try_notify(candidate_path: str) -> bool:
            candidate = str(candidate_path or "")
            if not candidate or candidate in tried_paths:
                return False
            tried_paths.append(candidate)
            return self._client.start_notify(candidate)

        if _try_notify(path):
            return True, path

        self._log_verbose(f"Retrying notify setup for {label} on {mac}: path={path}")
        self._client.wait_services_resolved(mac, timeout=self.NOTIFY_REFRESH_WAIT_S)

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

    def _update_peer_writeback_latency(self, state, payload: bytes):
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
            retry_map[desc_path] = time.monotonic() + self.PEER_WRITEBACK_RETRY_DELAY_S
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
                    if now - last < self.PEER_WRITEBACK_WARNING_INTERVAL_S:
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
                retry_map[desc_path] = time.monotonic() + self.PEER_WRITEBACK_RETRY_DELAY_S
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