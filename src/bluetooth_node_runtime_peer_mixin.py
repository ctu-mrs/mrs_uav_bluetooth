"""Peer discovery, pairing, recovery, and time-bridge helpers for the Bluetooth node."""

import random
import re
import struct
import time
from typing import Tuple

import dbus

from mrs_uav_bluetooth.msg import BlePeerTimeStatus

from .bluetooth_bridge_state import PeerConnectionSessionState, PeerTimeBridgeState
from .dbus_client import DeviceInfo
from .gatt_services import TIME_CHARACTERISTIC_UUID, TIME_WRITEBACK_DESCRIPTOR_UUID
from .uuid_utils import is_uav_hostname


class BluetoothNodeRuntimePeerMixin:

    PEER_SCAN_PAUSE_PHASES = {
        "connected",
        "pairing-pending",
        "pairing-requested",
        "pairing-in-progress",
        "pairing-succeeded",
        "trusted",
        "services-resolved",
        "waiting-services-resolved",
        "waiting-for-time-bridge",
        "waiting-time-characteristic",
        "time-notify-failed",
        "repairing-connect",
        "repairing-services",
    }
    PEER_SCAN_PAUSE_MAX_S = 20.0
    PEER_TIME_BRIDGE_IDLE_TIMEOUT_S = 20.0
    PEER_DISCONNECT_GRACE_MIN_S = 8.0
    PEER_DISCONNECT_GRACE_MULTIPLIER = 4.0
    PEER_SESSION_PRUNE_TTL_MIN_S = 120.0
    PEER_SESSION_PRUNE_TTL_MULTIPLIER = 60.0
    PEER_TIME_BRIDGE_WAIT_GRACE_MIN_S = 20.0
    PEER_TIME_BRIDGE_WAIT_GRACE_MULTIPLIER = 4.0
    SERVICES_RESOLVED_WAIT_GRACE_MIN_S = 12.0
    SERVICES_RESOLVED_WAIT_GRACE_MULTIPLIER = 3.0
    TIME_CHARACTERISTIC_REDISCOVERY_MIN_S = 8.0
    TIME_CHARACTERISTIC_REDISCOVERY_MULTIPLIER = 2.0
    CONNECT_TIMEOUT_MIN_S = 6.0
    CONNECT_TIMEOUT_MAX_S = 15.0
    CONNECT_TIMEOUT_MULTIPLIER = 3.0
    CONNECT_STALL_GRACE_MIN_S = 12.0
    CONNECT_STALL_GRACE_MULTIPLIER = 4.0
    CONNECT_STALL_JITTER_MIN_S = 5.0
    CONNECT_STALL_JITTER_MULTIPLIER = 2.0
    CONNECT_REPAIR_COOLDOWN_MIN_S = 15.0
    CONNECT_REPAIR_COOLDOWN_MULTIPLIER = 5.0
    SERVICE_REPAIR_COOLDOWN_MIN_S = 60.0
    SERVICE_REPAIR_COOLDOWN_MULTIPLIER = 10.0
    SERVICE_RETRY_COOLDOWN_MIN_S = 20.0
    SERVICE_RETRY_COOLDOWN_MULTIPLIER = 4.0
    PEER_CLEAR_DISCONNECT_TIMEOUT_S = 5.0
    PEER_REDISCOVERY_DISCONNECT_TIMEOUT_S = 3.0
    PEER_REDISCOVERY_CONNECT_TIMEOUT_S = 10.0
    PEER_REDISCOVERY_WAIT_SERVICES_TIMEOUT_S = 3.0
    PEER_REDISCOVERY_POST_CONNECT_DELAY_S = 2.0
    PEER_REPAIR_CONNECT_TIMEOUT_S = 10.0
    PEER_REPAIR_WAIT_SERVICES_TIMEOUT_S = 6.0
    PEER_REPAIR_PAIR_TIMEOUT_S = 30.0
    PAIR_TIMEOUT_MIN_S = 4.0
    PAIR_TIMEOUT_MAX_S = 12.0
    PAIR_TIMEOUT_MULTIPLIER = 2.0
    PAIR_PROBE_WINDOW_S = 1.0
    PAIR_PROBE_SLEEP_S = 0.2
    PAIR_REPAIR_THRESHOLD = 2
    PAIRING_REPAIR_COOLDOWN_S = 20.0
    SERVICES_RESOLVED_SHORT_WAIT_S = 2.0
    PEER_TIME_STATUS_QUEUE_SIZE = 10
    PEER_UNMANAGED_WRITEBACK_WARNING_INTERVAL_S = 5.0

    def _start_peer_bridge_wait(self, session: PeerConnectionSessionState, *, reason: str = "") -> float:
        started = float(session.bridge_wait_started_monotonic or 0.0)
        if started <= 0.0:
            started = time.monotonic()
            session.bridge_wait_started_monotonic = started
        if reason:
            session.bridge_wait_reason = str(reason)
        return started

    def _clear_peer_bridge_wait(self, session: PeerConnectionSessionState):
        session.bridge_wait_started_monotonic = 0.0
        session.bridge_wait_reason = ""

    def _get_scan_pause_reason(self, snapshot=None) -> str:
        if self._client is None:
            return ""
        now = time.monotonic()
        devices = snapshot or {}
        for mac, session in self._peer_sessions.items():
            if not session.desired or not session.peer_candidate:
                continue
            device = devices.get(mac) if devices else self._client.get_device(mac)
            phase = str(session.phase or "")
            connected = bool(device and device.connected)
            transport_live = connected or self._has_live_peer_time_bridge(mac) or session.connected_since_monotonic > 0.0
            if not transport_live:
                continue
            if self._should_pause_scan_for_session(session, phase=phase, connected=connected, now_mono=now):
                state = phase or "connected"
                return f"{mac}:{state}"
        return ""

    def _should_pause_scan_for_session(
        self,
        session: PeerConnectionSessionState,
        *,
        phase: str,
        connected: bool,
        now_mono: float,
    ) -> bool:
        if phase not in self.PEER_SCAN_PAUSE_PHASES:
            return False
        if not connected and phase != "waiting-for-time-bridge":
            return False
        pause_started = session.bridge_wait_started_monotonic or session.connected_since_monotonic or now_mono
        return (now_mono - pause_started) <= self.PEER_SCAN_PAUSE_MAX_S

    def _pause_scan_for_peer_stabilization(self, session: PeerConnectionSessionState, device: DeviceInfo = None):
        if self._client is None or not bool(self.get_parameter("enable_scan").value):
            return
        if not session.peer_candidate or not session.desired:
            return
        if self._has_ready_peer_bridge_transport(session.mac):
            return
        connected = bool(device and device.connected)
        if not connected and session.connected_since_monotonic <= 0.0:
            return
        if not self._should_pause_scan_for_session(
            session,
            phase=str(session.phase or "connected"),
            connected=connected,
            now_mono=time.monotonic(),
        ):
            return
        if not self._client.scanning:
            return
        device_label = f"{session.mac} ({self._hostname_from_device(device) or session.peer_name or '?'})"
        self._log_verbose(f"Pausing BLE scan while stabilizing peer {device_label}")
        self._stop_scan()

    def _get_resolved_peer_bridge_detail(self, mac: str) -> str:
        for key, state in self._notification_bridges.items():
            if state.mac != mac or not state.path:
                continue
            if state.transport_endpoint == "characteristic":
                try:
                    if self._is_characteristic_notifying(mac, state.path):
                        return f"topic_bridge={key}"
                except dbus.exceptions.DBusException:
                    continue
                continue
            return f"topic_bridge={key}"
        return ""

    def _has_ready_peer_bridge_transport(self, mac: str) -> bool:
        if self._has_live_peer_time_bridge(mac):
            return True
        return bool(self._get_resolved_peer_bridge_detail(mac))

    def _enforce_peer_connection_policy(self, snapshot=None, *, reason: str = "policy"):
        if self._client is None:
            return snapshot or {}
        if snapshot is None:
            snapshot = self._client.get_devices(refresh=True)

        auto_connect_enabled = bool(self.get_parameter("auto_connect_enable").value)
        whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        whitelist_enabled = bool(whitelist_names or whitelist_macs)
        changed = False
        remove_pairing = "overlay" in str(reason or "").lower()

        for mac, device in snapshot.items():
            explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
            peer_candidate = self._is_uav_peer_candidate(device, pattern)
            if not peer_candidate:
                continue
            if reason == "scan policy tick" and not whitelist_enabled and not auto_connect_enabled:
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
            self._clear_peer_local_state(
                mac,
                device=device,
                reason=f"policy denies connection ({reason})",
                remove_pairing=remove_pairing,
                untrust=True,
            )
            changed = True

        if changed:
            return self._client.get_devices(refresh=True)
        return snapshot

    def _get_peer_session(self, mac: str, device: DeviceInfo = None) -> PeerConnectionSessionState:
        session = self._peer_sessions.get(mac)
        if session is None:
            session = PeerConnectionSessionState(mac=mac)
            self._peer_sessions[mac] = session
        session.last_seen_monotonic = time.monotonic()
        if device is not None:
            session.peer_name = self._hostname_from_device(device)
        return session

    def _prune_peer_sessions(self, snapshot, now_mono: float, *, ttl_s: float):
        ttl = max(0.0, float(ttl_s))
        current_macs = set(snapshot.keys())
        for mac, session in list(self._peer_sessions.items()):
            if mac in current_macs:
                continue
            if mac in self._peer_time_bridges:
                continue
            if any(state.mac == mac for state in self._notification_bridges.values()):
                continue
            if ttl > 0.0 and now_mono - session.last_seen_monotonic <= ttl:
                continue
            self._peer_sessions.pop(mac, None)

    def _cleanup_peer_time_bridges(self, snapshot):
        stale_macs = []
        now = time.monotonic()
        timeout = max(0.0, float(self.get_parameter("peer_connection_timeout").value))
        retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
        disconnect_grace_s = max(
            self.PEER_DISCONNECT_GRACE_MIN_S,
            retry_period * self.PEER_DISCONNECT_GRACE_MULTIPLIER,
        )
        for mac, state in self._peer_time_bridges.items():
            device = snapshot.get(mac)
            session = self._peer_sessions.get(mac)
            if device is not None and self._is_peer_effectively_connected(mac, device=device):
                if state.status == "ready" and timeout > 0 and now - state.last_activity_monotonic >= timeout:
                    self.get_logger().warning(
                        f"Disconnecting {mac}: peer time bridge inactive for {now - state.last_activity_monotonic:.1f}s"
                    )
                    self._client.disconnect(mac, timeout=self.PEER_CLEAR_DISCONNECT_TIMEOUT_S)
                    stale_macs.append(mac)
                if session is not None and state.status == "ready":
                    session.missing_since_monotonic = 0.0
                    session.last_service_retry_monotonic = 0.0
                    self._clear_peer_bridge_wait(session)
                continue
            if session is None:
                session = self._get_peer_session(mac)
            disconnected_since = session.missing_since_monotonic or now
            if session.missing_since_monotonic <= 0.0:
                session.missing_since_monotonic = now
            if now - disconnected_since < disconnect_grace_s:
                continue
            stale_macs.append(mac)
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        for mac, device in snapshot.items():
            session = self._get_peer_session(mac, device=device)
            if not self._is_uav_peer_candidate(device, pattern):
                session.missing_since_monotonic = 0.0
                session.connected_since_monotonic = 0.0
                session.last_service_retry_monotonic = 0.0
                self._clear_peer_bridge_wait(session)
                continue
            if not self._is_peer_effectively_connected(mac, device=device):
                session.connected_since_monotonic = 0.0
                if session.missing_since_monotonic <= 0.0:
                    session.missing_since_monotonic = now
                continue
            if session.connected_since_monotonic <= 0.0:
                session.connected_since_monotonic = now
            if mac in self._peer_time_bridges:
                state = self._peer_time_bridges[mac]
                if state.status == "ready":
                    session.missing_since_monotonic = 0.0
                    session.last_service_retry_monotonic = 0.0
                    self._clear_peer_bridge_wait(session)
                elif session.missing_since_monotonic <= 0.0:
                    session.missing_since_monotonic = now
                continue
            if session.missing_since_monotonic <= 0.0:
                session.missing_since_monotonic = now
        for mac in stale_macs:
            state = self._peer_time_bridges.pop(mac, None)
            if state is not None:
                self.destroy_publisher(state.publisher)
                self._stop_notify_if_unused(state.characteristic_path)
                self._clear_peer_writeback_state(state.writeback_descriptor_path)
            session = self._peer_sessions.get(mac)
            if session is not None:
                session.phase = "disconnected"
                session.detail = "stale peer bridge cleaned up"
                session.last_security_attempt_monotonic = 0.0
                session.missing_since_monotonic = 0.0
                session.connected_since_monotonic = 0.0
                session.last_service_retry_monotonic = 0.0
                self._clear_peer_bridge_wait(session)

    def _auto_connect_devices(self):
        if self._client is None:
            return
        try:
            now = time.monotonic()
            retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
            whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
            auto_connect_enabled = bool(self.get_parameter("auto_connect_enable").value)
            if not auto_connect_enabled and not whitelist_names and not whitelist_macs:
                return
            whitelist_enabled = bool(whitelist_names or whitelist_macs)
            pattern = str(self.get_parameter("auto_connect_pattern").value)
            snapshot = self._client.get_devices(refresh=True)
            self._prune_peer_sessions(
                snapshot,
                now,
                ttl_s=max(self.PEER_SESSION_PRUNE_TTL_MIN_S, retry_period * self.PEER_SESSION_PRUNE_TTL_MULTIPLIER),
            )
            self._log_verbose(
                f"Auto-connect tick: {len(snapshot)} device(s), "
                f"whitelist={sorted(whitelist_names) or '(none)'}, pattern={pattern}, "
                f"enable={auto_connect_enabled}"
            )
            self._log_discovered_devices_summary(snapshot, pattern)
            for mac, device in snapshot.items():
                session = self._get_peer_session(mac, device=device)
                session.peer_candidate = self._is_uav_peer_candidate(device, pattern)
                session.explicit_target = self._matches_auto_connect_whitelist(device, whitelist_names, whitelist_macs)
                session.desired = session.explicit_target or (auto_connect_enabled and session.peer_candidate and not whitelist_enabled)
                device_label = f"{mac} ({device.alias or device.name or '?'})"
                if whitelist_enabled and session.peer_candidate and not session.explicit_target:
                    session.phase = "policy-blocked"
                    session.detail = "peer not present in whitelist"
                    self._log_verbose(f"Dropping non-whitelisted peer: {device_label}")
                    self._drop_non_whitelisted_peer(mac, device)
                    continue
                if not session.desired:
                    session.phase = "idle"
                    session.detail = "not targeted by auto-connect policy"
                    session.connect_started_monotonic = 0.0
                    session.last_connect_attempt_monotonic = 0.0
                    session.connect_repair_count = 0
                    if device.connected:
                        self._clear_peer_local_state(mac, device=device, reason="auto-connect policy disabled")
                    continue
                self._advance_peer_session(session, device, retry_period)
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("auto_connect", f"Skipping auto-connect tick due to DBus error: {exc}")

    def _advance_peer_session(self, session: PeerConnectionSessionState, device: DeviceInfo, retry_period: float):
        now = time.monotonic()
        device_label = f"{session.mac} ({device.alias or device.name or '?'})"
        previous_phase = session.phase
        effectively_connected = self._is_peer_effectively_connected(session.mac, device=device)

        if session.desired and session.peer_candidate:
            self._pause_scan_for_peer_stabilization(session, device)

        if effectively_connected:
            if not device.connected:
                device = device.copy()
                device.connected = True
            if session.connected_since_monotonic <= 0.0:
                session.connected_since_monotonic = now
            if session.connect_started_monotonic <= 0.0:
                session.connect_started_monotonic = session.connected_since_monotonic
            session.phase = "connected"
            healthy = self._maintain_peer_connection(session, device, retry_period)
            if healthy:
                session.connect_started_monotonic = 0.0
                session.last_repair_monotonic = 0.0
                session.connect_repair_count = 0
                session.phase = "ready"
                session.detail = "peer session ready"
                if previous_phase != "ready":
                    self.get_logger().info(f"Auto-connected BLE device {session.mac}")
                    self._log_verbose(f"Auto-connected: {device_label}")
            elif session.peer_candidate:
                pending_s = "n/a"
                if session.connect_started_monotonic > 0.0:
                    pending_s = f"{max(0.0, now - session.connect_started_monotonic):.1f}s"
                self._log_verbose(
                    f"Connected but unhealthy peer: {device_label} "
                    f"(pending={pending_s}, paired={device.paired}, bonded={device.bonded}, trusted={device.trusted})"
                )
                self._handle_unhealthy_peer_candidate(session, device_label, retry_period)
            return

        session.connected_since_monotonic = 0.0
        self._clear_peer_bridge_wait(session)
        current = self._prepare_disconnected_peer_security(session, device, retry_period)
        if current is None:
            return
        if session.connect_started_monotonic <= 0.0:
            session.connect_started_monotonic = now
        if self._maybe_recover_stalled_connect(session, current, device_label, retry_period):
            return
        if now - session.last_connect_attempt_monotonic < retry_period:
            waited_s = max(0.0, now - session.connect_started_monotonic)
            self._set_peer_time_status(session.mac, "connect-pending", f"wait={waited_s:.1f}s")
            return

        waited_so_far = max(0.0, now - session.connect_started_monotonic)
        session.last_connect_attempt_monotonic = now
        timeout_s = max(self.CONNECT_TIMEOUT_MIN_S, min(self.CONNECT_TIMEOUT_MAX_S, retry_period * self.CONNECT_TIMEOUT_MULTIPLIER))
        self._log_verbose(
            f"Auto-connect attempt: {device_label} "
            f"(pending={waited_so_far:.1f}s, repairs={session.connect_repair_count}, "
            f"paired={current.paired}, bonded={current.bonded}, trusted={current.trusted})"
        )
        connect_requested = self._client.connect_async(session.mac, timeout=timeout_s)
        if not connect_requested:
            connect_requested = self._run_background_once(
                f"connect::{session.mac}",
                self._client.connect,
                session.mac,
                timeout_s,
            )
        refreshed = self._client.get_device(session.mac, refresh=True) or current
        if refreshed.connected:
            self._advance_peer_session(session, refreshed, retry_period)
            return
        waited_s = max(0.0, time.monotonic() - session.connect_started_monotonic)
        session.phase = "connect-pending" if connect_requested else "connect-request-failed"
        session.detail = f"wait={waited_s:.1f}s"
        self._set_peer_time_status(session.mac, session.phase, session.detail)
        self._log_verbose(
            f"Auto-connect pending: {device_label} "
            f"(requested={connect_requested}, wait={waited_s:.1f}s, phase={session.phase})"
        )

    def _handle_unhealthy_peer_candidate(self, session: PeerConnectionSessionState, device_label: str, retry_period: float) -> bool:
        now_mono = time.monotonic()
        missing_since = self._start_peer_bridge_wait(session, reason="waiting-for-time-bridge")
        connected_since = session.connected_since_monotonic or now_mono
        if connected_since > missing_since:
            missing_since = connected_since
            session.bridge_wait_started_monotonic = connected_since
        grace_s = max(self.PEER_TIME_BRIDGE_WAIT_GRACE_MIN_S, retry_period * self.PEER_TIME_BRIDGE_WAIT_GRACE_MULTIPLIER)
        waited_s = max(0.0, now_mono - missing_since)
        if waited_s < grace_s:
            self._log_verbose(
                f"Keeping {device_label} connected while waiting for time characteristic "
                f"({waited_s:.1f}s/{grace_s:.1f}s)"
            )
            self._set_peer_time_status(
                session.mac,
                "waiting-for-time-bridge",
                f"wait={waited_s:.1f}/{grace_s:.1f}s",
                wait_started=missing_since,
                wait_grace_s=grace_s,
            )
            return False
        self.get_logger().info(f"Disconnecting {session.mac}: UAV peer candidate without healthy BLE time bridge")
        self._log_verbose(f"Disconnecting {device_label}: no healthy time bridge after grace period")
        self._clear_peer_local_state(session.mac, reason="missing healthy BLE time bridge")
        return True

    def _prepare_disconnected_peer_security(self, session: PeerConnectionSessionState, device: DeviceInfo, retry_period: float):
        current = self._client.get_device(session.mac, refresh=True) or device
        if current.connected:
            session.last_security_attempt_monotonic = 0.0
            return current
        if not (current.paired or current.bonded):
            return current
        if current.trusted:
            session.last_security_attempt_monotonic = 0.0
            return current

        now_mono = time.monotonic()
        last_attempt = session.last_security_attempt_monotonic
        if now_mono - last_attempt < retry_period:
            return None
        session.last_security_attempt_monotonic = now_mono

        device_label = f"{session.mac} ({self._hostname_from_device(current) or '?'})"
        self._log_verbose(
            f"Pre-connect trust repair for {device_label}: paired={current.paired} "
            f"bonded={current.bonded} trusted={current.trusted}"
        )
        if not self._client.trust(session.mac):
            self.get_logger().warning(f"Failed to trust BLE peer {session.mac} before connect")
            self._log_verbose(f"Pre-connect trust failed: {device_label}")
            self._set_peer_time_status(session.mac, "trust-failed", "Trusted=False before connect")
            return None

        refreshed = self._client.get_device(session.mac, refresh=True) or current
        if refreshed.trusted:
            self.get_logger().info(f"Trusted BLE peer {session.mac} before connect")
            self._log_verbose(f"Pre-connect trusted: {device_label}")
            self._set_peer_time_status(session.mac, "trusted", "Trusted=True before connect")
            session.last_security_attempt_monotonic = 0.0
            return refreshed

        self._log_verbose(f"Pre-connect trust did not stick yet for {device_label}")
        self._set_peer_time_status(session.mac, "trust-failed", "Trusted=False after pre-connect trust")
        return None

    def _maybe_recover_stalled_connect(
        self,
        session: PeerConnectionSessionState,
        device: DeviceInfo,
        device_label: str,
        retry_period: float,
    ) -> bool:
        now_mono = time.monotonic()
        pending_since = session.connect_started_monotonic or now_mono
        waited_s = max(0.0, now_mono - pending_since)
        jitter_s = random.Random(session.mac).uniform(
            0.0,
            max(self.CONNECT_STALL_JITTER_MIN_S, retry_period * self.CONNECT_STALL_JITTER_MULTIPLIER),
        )
        grace_s = max(self.CONNECT_STALL_GRACE_MIN_S, retry_period * self.CONNECT_STALL_GRACE_MULTIPLIER) + jitter_s
        if waited_s < grace_s:
            return False
        last_repair = session.last_repair_monotonic
        repair_cooldown_s = max(self.CONNECT_REPAIR_COOLDOWN_MIN_S, retry_period * self.CONNECT_REPAIR_COOLDOWN_MULTIPLIER) + jitter_s
        if now_mono - last_repair < repair_cooldown_s:
            self._log_verbose(
                f"Stalled connect for {device_label}: cooldown active "
                f"(wait={waited_s:.1f}s, cooldown remaining={max(0.0, repair_cooldown_s - (now_mono - last_repair)):.1f}s)"
            )
            return False

        current = self._client.get_device(session.mac, refresh=True) or device
        repair_count = int(session.connect_repair_count)
        stale_security = bool(current.paired or current.bonded or current.trusted)
        remove_pairing = stale_security
        remove_device = repair_count >= 1
        self.get_logger().warning(
            f"Peer {session.mac} stayed discoverable but disconnected for {waited_s:.1f}s; repairing BLE link state"
        )
        self._log_verbose(
            f"Repairing stalled connect for {device_label}: wait={waited_s:.1f}s "
            f"connected={current.connected} paired={current.paired} bonded={current.bonded} "
            f"trusted={current.trusted} repair_count={repair_count + 1} "
            f"remove_pairing={remove_pairing} remove_device={remove_device}"
        )
        self._set_peer_time_status(
            session.mac,
            "repairing-connect",
            f"wait={waited_s:.1f}s repair={repair_count + 1} remove_pairing={remove_pairing}",
        )
        self._clear_peer_local_state(
            session.mac,
            device=current,
            reason="stalled BLE connect attempt",
            remove_pairing=remove_pairing or remove_device,
            untrust=stale_security,
        )
        session.last_repair_monotonic = now_mono
        session.connect_repair_count = repair_count + 1
        session.connect_started_monotonic = time.monotonic()
        if remove_device:
            self._client.remove(session.mac)
            self._log_verbose(
                f"Removed {device_label} from BlueZ cache after {repair_count + 1} failed repairs; waiting for re-discovery by scanner"
            )
            return True

        self._log_verbose(f"Stalled connect repair reset local state for {device_label}; waiting for next auto-connect tick")
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
        self._client.disconnect(mac, timeout=self.PEER_CLEAR_DISCONNECT_TIMEOUT_S)
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
        session = self._peer_sessions.get(mac)
        if session is not None:
            session.phase = "disconnected"
            session.detail = reason or "local peer state cleared"
            session.last_connect_attempt_monotonic = 0.0
            session.connect_started_monotonic = 0.0
            session.connected_since_monotonic = 0.0
            session.last_security_attempt_monotonic = 0.0
            session.last_repair_monotonic = 0.0
            session.last_service_retry_monotonic = 0.0
            session.missing_since_monotonic = 0.0
            session.connect_repair_count = 0
            session.services_wait_started_monotonic = 0.0
            session.services_wait_grace_s = 0.0
            self._clear_peer_bridge_wait(session)
            session.import_bridge_missing_since.clear()

    def _is_uav_peer_candidate(self, device: DeviceInfo, pattern: str) -> bool:
        local_name = self._local_name.strip().lower()
        for candidate in (device.alias or "", device.name or "", self._hostname_from_device(device)):
            normalized = candidate.strip().lower()
            if not normalized or normalized == local_name:
                continue
            if is_uav_hostname(normalized, pattern=pattern):
                return True
        return False

    def _has_live_peer_time_bridge(self, mac: str, *, idle_timeout_s: float = None) -> bool:
        if idle_timeout_s is None:
            idle_timeout_s = self.PEER_TIME_BRIDGE_IDLE_TIMEOUT_S
        state = self._peer_time_bridges.get(mac)
        if state is None or not state.characteristic_path:
            return False
        try:
            if self._is_characteristic_notifying(mac, state.characteristic_path):
                return True
        except dbus.exceptions.DBusException:
            pass
        return (time.monotonic() - state.last_activity_monotonic) <= max(1.0, float(idle_timeout_s))

    def _is_peer_effectively_connected(self, mac: str, device: DeviceInfo = None) -> bool:
        if device is not None and device.connected:
            return True
        return self._has_live_peer_time_bridge(mac)

    def _ensure_peer_services_resolved(self, session: PeerConnectionSessionState, device: DeviceInfo, *, device_label: str) -> bool:
        bridge_detail = self._get_resolved_peer_bridge_detail(session.mac)
        if bridge_detail:
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(session.mac, "services-resolved", bridge_detail)
            return True
        if device.services_resolved:
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(session.mac, "services-resolved", "services_resolved=True")
            return True
        probe_path = self._resolve_peer_time_characteristic_path(session.mac, allow_wait=False)
        if probe_path:
            self._log_verbose(
                f"Peer {session.mac}: ServicesResolved=False but time characteristic found at {probe_path}; proceeding"
            )
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(
                session.mac,
                "services-resolved",
                f"services_resolved=False,time_path={probe_path}",
            )
            return True
        self._client.wait_services_resolved(session.mac, timeout=self.SERVICES_RESOLVED_SHORT_WAIT_S)
        probe_path = self._resolve_peer_time_characteristic_path(session.mac, allow_wait=False)
        if probe_path:
            self._log_verbose(
                f"Peer {session.mac}: time characteristic appeared after short wait at {probe_path}; proceeding"
            )
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(
                session.mac,
                "services-resolved",
                f"services_resolved=False,time_path={probe_path}(after_wait)",
            )
            return True
        wait_started = self._start_peer_bridge_wait(session, reason="services unresolved")
        grace_s = max(
            self.SERVICES_RESOLVED_WAIT_GRACE_MIN_S,
            float(self.get_parameter("auto_connect_period").value) * self.SERVICES_RESOLVED_WAIT_GRACE_MULTIPLIER,
        )
        waited_s = max(0.0, time.monotonic() - wait_started)
        self._set_peer_time_status(
            session.mac,
            "waiting-services-resolved",
            f"wait={waited_s:.1f}/{grace_s:.1f}s",
            wait_started=wait_started,
            wait_grace_s=grace_s,
        )
        self._run_background_once(
            f"resolve-services::{session.mac}",
            self._background_resolve_services,
            session.mac,
            device_label,
        )
        self._log_verbose(f"Peer {session.mac}: services not resolved yet, delaying time bridge setup")
        self._maybe_repair_peer_link(
            session,
            device_label,
            reason="services unresolved for too long",
            min_wait_s=grace_s,
        )
        return False

    def _background_resolve_services(self, mac: str, device_label: str):
        self._client.wait_services_resolved(mac, timeout=self.PEER_REPAIR_WAIT_SERVICES_TIMEOUT_S)
        try:
            self._client.list_services(mac)
            self._client.list_characteristics(mac)
        except dbus.exceptions.DBusException as exc:
            self._log_verbose(f"Background service enumeration failed for {device_label}: {exc}")

    def _force_peer_service_rediscovery(self, mac: str, device_label: str) -> bool:
        current = self._client.get_device(mac, refresh=True)
        if current is None or not current.connected:
            return False
        self._log_verbose(f"Forcing service rediscovery for {device_label}")
        try:
            self._client.wait_services_resolved(mac, timeout=self.PEER_REDISCOVERY_DISCONNECT_TIMEOUT_S)
            self._client.list_services(mac)
            self._client.list_characteristics(mac)
        except dbus.exceptions.DBusException as exc:
            self._log_verbose(f"Service rediscovery pre-pass failed for {device_label}: {exc}")
        if self._resolve_peer_time_characteristic_path(mac):
            return True

        self._log_verbose(f"Service rediscovery fallback reconnect for {device_label}")
        self._client.disconnect(mac, timeout=self.PEER_REDISCOVERY_DISCONNECT_TIMEOUT_S)
        if not self._client.connect(mac, timeout=self.PEER_REDISCOVERY_CONNECT_TIMEOUT_S):
            self._log_verbose(f"Service rediscovery reconnect failed for {device_label}")
            return False
        self._client.wait_services_resolved(mac, timeout=self.PEER_REDISCOVERY_WAIT_SERVICES_TIMEOUT_S)
        try:
            self._client.list_services(mac)
            self._client.list_characteristics(mac)
        except dbus.exceptions.DBusException:
            pass
        if self._resolve_peer_time_characteristic_path(mac):
            self._log_verbose(f"Service rediscovery recovered time characteristic for {device_label}")
            return True
        time.sleep(self.PEER_REDISCOVERY_POST_CONNECT_DELAY_S)
        self._client.get_device(mac, refresh=True)
        try:
            self._client.list_services(mac)
            self._client.list_characteristics(mac)
        except dbus.exceptions.DBusException:
            pass
        if self._resolve_peer_time_characteristic_path(mac):
            self._log_verbose(f"Service rediscovery recovered time characteristic (delayed) for {device_label}")
            return True
        self._log_verbose(f"Service rediscovery could not resolve time characteristic for {device_label}")
        return False

    def _resolve_peer_time_characteristic_path(self, mac: str, *, allow_wait: bool = True) -> str:
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        if candidates:
            return candidates[0]
        self._client.get_device(mac, refresh=True)
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        if candidates:
            return candidates[0]
        try:
            self._client.list_services(mac)
            self._client.list_characteristics(mac)
        except dbus.exceptions.DBusException:
            pass
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        if candidates:
            return candidates[0]
        if not allow_wait:
            return ""
        self._client.wait_services_resolved(mac, timeout=self.PAIR_PROBE_WINDOW_S)
        candidates = self._client.find_characteristics(mac, TIME_CHARACTERISTIC_UUID)
        return candidates[0] if candidates else ""

    def _ensure_peer_time_bridge(self, session: PeerConnectionSessionState, device: DeviceInfo) -> Tuple[bool, bool]:
        current = self._client.get_device(session.mac, refresh=True) or device
        device_label = f"{session.mac} ({self._hostname_from_device(device) or '?'})"
        existing = self._peer_time_bridges.get(session.mac)
        bridge_detail = self._get_resolved_peer_bridge_detail(session.mac)
        if existing is not None and self._has_live_peer_time_bridge(session.mac):
            session.missing_since_monotonic = 0.0
            session.last_service_retry_monotonic = 0.0
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(session.mac, "ready", f"time_path={existing.characteristic_path}")
            return True, True
        if bridge_detail:
            session.missing_since_monotonic = 0.0
            session.last_service_retry_monotonic = 0.0
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(session.mac, "ready", bridge_detail)
            return True, False
        if not self._ensure_peer_services_resolved(session, current, device_label=device_label):
            return False, False

        path = self._resolve_peer_time_characteristic_path(session.mac)
        if not path:
            wait_started = self._start_peer_bridge_wait(
                session,
                reason=f"missing time characteristic {TIME_CHARACTERISTIC_UUID}",
            )
            waited_s = max(0.0, time.monotonic() - wait_started)
            self._log_verbose(f"Peer {device_label}: time characteristic {TIME_CHARACTERISTIC_UUID} not found")
            retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
            early_rediscovery_s = max(
                self.TIME_CHARACTERISTIC_REDISCOVERY_MIN_S,
                retry_period * self.TIME_CHARACTERISTIC_REDISCOVERY_MULTIPLIER,
            )
            last_svc_retry = session.last_service_retry_monotonic
            if waited_s >= early_rediscovery_s and (time.monotonic() - last_svc_retry >= early_rediscovery_s):
                session.last_service_retry_monotonic = time.monotonic()
                self._log_verbose(f"Proactive service rediscovery for {device_label}: time char missing for {waited_s:.1f}s")
                self._run_background_once(
                    f"service-rediscovery::{session.mac}",
                    self._force_peer_service_rediscovery,
                    session.mac,
                    device_label,
                )
            grace_s = max(self.PEER_TIME_BRIDGE_WAIT_GRACE_MIN_S, retry_period * self.PEER_TIME_BRIDGE_WAIT_GRACE_MULTIPLIER)
            waited_s = max(0.0, time.monotonic() - wait_started)
            self._set_peer_time_status(
                session.mac,
                "waiting-time-characteristic",
                f"wait={waited_s:.1f}/{grace_s:.1f}s uuid={TIME_CHARACTERISTIC_UUID}",
                wait_started=wait_started,
                wait_grace_s=grace_s,
            )
            self._maybe_repair_peer_link(
                session,
                device_label,
                reason=f"missing time characteristic {TIME_CHARACTERISTIC_UUID}",
                min_wait_s=grace_s,
            )
            return False, False
        writeback_descriptor_path = self._client.find_descriptor(session.mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
        peer_name = self._peer_name_token(session.mac, device=device)
        status_topic_name = self._resolve_peer_topic_name(session.mac, "/time_status", device=device)
        if re.search(r"/peers/[0-9]", status_topic_name):
            status_topic_name = self._node_topic(f"peers/{peer_name}/time_status")
        existing = self._peer_time_bridges.get(session.mac)
        if (
            existing is not None
            and existing.characteristic_path == path
            and existing.writeback_descriptor_path == writeback_descriptor_path
            and existing.status_topic_name == status_topic_name
        ):
            session.missing_since_monotonic = 0.0
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(session.mac, "ready", f"time_path={path}")
            if self._is_characteristic_notifying(session.mac, path):
                return True, True
            started, path = self._start_notify_with_refresh(
                session.mac,
                path,
                TIME_CHARACTERISTIC_UUID,
                label="peer time characteristic",
            )
            writeback_descriptor_path = self._client.find_descriptor(session.mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
            existing.characteristic_path = path
            existing.writeback_descriptor_path = writeback_descriptor_path
            if started:
                self._set_peer_time_status(session.mac, "ready", f"time_path={path}")
                self._prime_peer_time_bridge(existing)
            return started, True
        if existing is not None:
            old_path = existing.characteristic_path
            self.destroy_publisher(existing.publisher)
            self._peer_time_bridges.pop(session.mac, None)
            self._stop_notify_if_unused(old_path)
        publisher = self.create_publisher(BlePeerTimeStatus, status_topic_name, self.PEER_TIME_STATUS_QUEUE_SIZE)
        started, path = self._start_notify_with_refresh(
            session.mac,
            path,
            TIME_CHARACTERISTIC_UUID,
            label="peer time characteristic",
        )
        if not started:
            self.get_logger().warning(f"Failed to start time notify for peer {device_label}")
            self._log_verbose(f"Failed to start_notify on time characteristic {path} for {device_label}")
            self.destroy_publisher(publisher)
            if session.missing_since_monotonic <= 0.0:
                session.missing_since_monotonic = time.monotonic()
            self._set_peer_time_status(session.mac, "time-notify-failed", f"path={path}")
            return False, False
        self._remember_notification_path(session.mac, path)
        writeback_descriptor_path = self._client.find_descriptor(session.mac, TIME_WRITEBACK_DESCRIPTOR_UUID, chrc_path=path) or ""
        self.get_logger().info(f"Established time bridge with peer {device_label} -> {status_topic_name}")
        self._log_verbose(
            f"Time bridge: {device_label} chrc={path} writeback={writeback_descriptor_path or 'none'} topic={status_topic_name}"
        )
        self._peer_time_bridges[session.mac] = PeerTimeBridgeState(
            mac=session.mac,
            peer_name=peer_name,
            status_topic_name=status_topic_name,
            characteristic_path=path,
            writeback_descriptor_path=writeback_descriptor_path,
            publisher=publisher,
            status="ready",
            detail=f"time_path={path}",
        )
        session.missing_since_monotonic = 0.0
        self._clear_peer_bridge_wait(session)
        self._prime_peer_time_bridge(self._peer_time_bridges[session.mac])
        return True, True

    def _maybe_repair_peer_link(self, session: PeerConnectionSessionState, device_label: str, *, reason: str, min_wait_s: float) -> bool:
        now_mono = time.monotonic()
        missing_since = session.bridge_wait_started_monotonic or now_mono
        if session.bridge_wait_started_monotonic <= 0.0:
            session.bridge_wait_started_monotonic = now_mono
        connected_since = session.connected_since_monotonic or 0.0
        if connected_since is not None and connected_since > missing_since:
            missing_since = connected_since
            session.bridge_wait_started_monotonic = connected_since
        waited_s = max(0.0, now_mono - missing_since)
        if waited_s < max(0.0, float(min_wait_s)):
            return False
        retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
        repair_cooldown_s = max(self.SERVICE_REPAIR_COOLDOWN_MIN_S, retry_period * self.SERVICE_REPAIR_COOLDOWN_MULTIPLIER)
        recovery_retry_s = max(self.SERVICE_RETRY_COOLDOWN_MIN_S, retry_period * self.SERVICE_RETRY_COOLDOWN_MULTIPLIER)
        reason_lower = str(reason or "").lower()
        needs_service_recovery = "missing time characteristic" in reason_lower or "services unresolved" in reason_lower
        if "pairing failed" in reason_lower:
            return False
        if needs_service_recovery:
            last_service_retry = session.last_service_retry_monotonic
            if now_mono - last_service_retry >= repair_cooldown_s:
                session.last_service_retry_monotonic = now_mono
                self.get_logger().warning(f"Peer {session.mac}: forcing service rediscovery before bond reset ({reason})")
                scheduled = self._run_background_once(
                    f"service-rediscovery::{session.mac}",
                    self._force_peer_service_rediscovery,
                    session.mac,
                    device_label,
                )
                if scheduled:
                    self._log_verbose(f"Scheduled background service rediscovery for {device_label}")
                    return False
                recovered = self._force_peer_service_rediscovery(session.mac, device_label)
                if recovered:
                    session.last_service_retry_monotonic = 0.0
                return False
            if now_mono - last_service_retry < recovery_retry_s:
                return False
            if now_mono - session.last_repair_monotonic < repair_cooldown_s:
                return False
            self.get_logger().warning(
                f"Peer {session.mac}: bridge setup stuck for {waited_s:.1f}s; forcing clean reconnect ({reason})"
            )
            self._log_verbose(
                f"Repairing unresolved peer bridge for {device_label}: wait={waited_s:.1f}s reason={reason}"
            )
            self._set_peer_time_status(session.mac, "repairing-services", f"wait={waited_s:.1f}s")
            session.last_repair_monotonic = now_mono
            self._clear_peer_local_state(session.mac, reason=f"bridge setup stuck ({reason})")
            return True
        return False

    def _prime_peer_time_bridge(self, state: PeerTimeBridgeState):
        current = self._client.get_device(state.mac, refresh=True)
        if current is None or not current.connected:
            return
        payload = self._client.read_characteristic(state.characteristic_path)
        if payload is not None:
            self._process_peer_time_payload(state.mac, state.characteristic_path, payload)

    def _maintain_peer_connection(self, session: PeerConnectionSessionState, device: DeviceInfo, retry_period: float) -> bool:
        self._ensure_peer_security(session, device, retry_period)
        current = self._client.get_device(session.mac, refresh=True) or device
        security_ready = bool(current.trusted and (current.paired or current.bonded))
        if not security_ready:
            self._set_peer_time_status(session.mac, "pairing-pending", "waiting for paired+trusted")
            return False
        if not session.peer_candidate:
            self._clear_peer_bridge_wait(session)
            session.phase = "ready"
            session.detail = "connected and trusted"
            return True
        bridge_detail = self._get_resolved_peer_bridge_detail(session.mac)
        if bridge_detail:
            self._clear_peer_bridge_wait(session)
            self._set_peer_time_status(session.mac, "ready", bridge_detail)
            return True
        time_bridge_ready, _ = self._ensure_peer_time_bridge(session, current)
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

    def _ensure_peer_security(self, session: PeerConnectionSessionState, device: DeviceInfo, retry_period: float):
        current = self._client.get_device(session.mac, refresh=True) or device
        security_ready = bool(current.trusted and (current.paired or current.bonded))
        if security_ready:
            session.last_security_attempt_monotonic = 0.0
            session.pairing_failures = 0
            self._set_peer_time_status(session.mac, "paired", "paired+trusted")
            return
        now = time.monotonic()
        last_attempt = session.last_security_attempt_monotonic
        if now - last_attempt < retry_period:
            return
        session.last_security_attempt_monotonic = now
        device_label = f"{session.mac} ({self._hostname_from_device(current) or '?'})"
        self._set_peer_time_status(session.mac, "pairing-requested", f"retry_period={retry_period:.1f}s")
        self._log_verbose(
            f"Security state for {device_label}: paired={current.paired} trusted={current.trusted} bonded={current.bonded}"
        )
        if not (current.paired or current.bonded):
            pair_timeout_s = max(self.PAIR_TIMEOUT_MIN_S, min(self.PAIR_TIMEOUT_MAX_S, retry_period * self.PAIR_TIMEOUT_MULTIPLIER))
            pair_requested = self._client.pair_async(session.mac, timeout=pair_timeout_s)
            if pair_requested:
                self._set_peer_time_status(session.mac, "pairing-in-progress", f"timeout={pair_timeout_s:.1f}s")
                self._log_verbose(f"Pair requested for {device_label} (timeout={pair_timeout_s:.1f}s)")
                probe_deadline = time.monotonic() + self.PAIR_PROBE_WINDOW_S
                while time.monotonic() < probe_deadline:
                    current = self._client.get_device(session.mac, refresh=True) or current
                    if current.paired or current.bonded:
                        break
                    time.sleep(self.PAIR_PROBE_SLEEP_S)
                if current.paired or current.bonded:
                    self.get_logger().info(f"Paired BLE peer {session.mac}")
                    self._log_verbose(f"Paired: {device_label}")
                    self._set_peer_time_status(session.mac, "pairing-succeeded", "paired=True")
                    session.pairing_failures = 0
                else:
                    return
            else:
                self.get_logger().warning(f"Failed to pair BLE peer {session.mac}")
                self._log_verbose(f"Pairing failed: {device_label}")
                session.pairing_failures += 1
                failures = session.pairing_failures
                self._set_peer_time_status(session.mac, "pairing-failed", f"attempts={failures}")
                if failures >= self.PAIR_REPAIR_THRESHOLD or bool(current.trusted):
                    self.get_logger().warning(
                        f"Repairing peer {session.mac}: pairing failed repeatedly (possible one-sided stale bond); clearing local bond/cache"
                    )
                    self._log_verbose(
                        f"Repairing peer {device_label}: pairing failed repeatedly (possible one-sided stale bond), removing bond/cache"
                    )
                    self._clear_peer_local_state(
                        session.mac,
                        reason="pairing failed repeatedly (possible one-sided stale bond)",
                        remove_pairing=True,
                        untrust=True,
                    )
                    session.last_repair_monotonic = time.monotonic()
                    self._set_peer_time_status(session.mac, "repairing-pairing", "clearing bond/cache")
                else:
                    self._client.disconnect(session.mac, timeout=self.PEER_REDISCOVERY_DISCONNECT_TIMEOUT_S)
                    self._log_verbose(f"Pairing failed once for {device_label}, retrying after reconnect")
                return
            current = self._client.get_device(session.mac, refresh=True) or current
        if (current.paired or current.bonded) and not current.trusted:
            if self._client.trust(session.mac):
                self.get_logger().info(f"Trusted BLE peer {session.mac}")
                self._log_verbose(f"Trusted: {device_label}")
                self._set_peer_time_status(session.mac, "trusted", "Trusted=True")
            else:
                self.get_logger().warning(f"Failed to trust BLE peer {session.mac}")
                self._log_verbose(f"Trust failed: {device_label}")
                self._set_peer_time_status(session.mac, "trust-failed", "Trusted=False")
            current = self._client.get_device(session.mac, refresh=True) or current
        if current.trusted and (current.paired or current.bonded):
            session.last_security_attempt_monotonic = 0.0
            session.pairing_failures = 0
            self._set_peer_time_status(session.mac, "paired", "paired+trusted")

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
        session = self._peer_sessions.get(mac)
        if session is not None:
            session.phase = status
            session.detail = detail or ""
            if wait_started is not None:
                session.services_wait_started_monotonic = max(0.0, float(wait_started))
                session.services_wait_grace_s = max(0.0, float(wait_grace_s or 0.0))
            elif not status.startswith("waiting"):
                session.services_wait_started_monotonic = 0.0
                session.services_wait_grace_s = 0.0
            if status.startswith("pairing"):
                session.pairing_failures = max(session.pairing_failures, 0)
        if state is None:
            return
        state.status = status
        state.detail = detail or ""
        if wait_started is not None:
            state.services_wait_started_monotonic = max(0.0, float(wait_started))
        elif not status.startswith("waiting"):
            state.services_wait_started_monotonic = 0.0
        if wait_grace_s is not None:
            state.services_wait_grace_s = max(0.0, float(wait_grace_s))
        elif not status.startswith("waiting"):
            state.services_wait_grace_s = 0.0
        if status.startswith("pairing"):
            state.pairing_requested_monotonic = time.monotonic()
            if session is not None:
                state.pairing_failures = int(session.pairing_failures)

    def _is_characteristic_notifying(self, mac: str, path: str) -> bool:
        for characteristic in self._client.list_characteristics(mac):
            if characteristic["path"] == path:
                return bool(characteristic.get("notifying", False))
        return False

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
        if not self._client.connect(mac, timeout=self.PEER_REPAIR_CONNECT_TIMEOUT_S):
            self.get_logger().warning(f"Incoming pairing recovery failed to reconnect {mac}")
            return
        self._client.wait_services_resolved(mac, timeout=self.PEER_REPAIR_WAIT_SERVICES_TIMEOUT_S)
        paired = self._client.pair(mac, timeout=self.PEER_REPAIR_PAIR_TIMEOUT_S)
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
        if not mac and device_path:
            for state in self._peer_time_bridges.values():
                if state.characteristic_path and state.characteristic_path.startswith(device_path + "/"):
                    mac = state.mac
                    break
        if not mac:
            self._log_verbose(f"Time writeback from unknown device path={device_path} len={len(payload)}")
            return
        state = self._peer_time_bridges.get(mac)
        if state is None:
            key = f"time_writeback_unmanaged::{mac}"
            now = time.monotonic()
            last = self._last_dbus_warning_at.get(key, 0.0)
            if now - last >= self.PEER_UNMANAGED_WRITEBACK_WARNING_INTERVAL_S:
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