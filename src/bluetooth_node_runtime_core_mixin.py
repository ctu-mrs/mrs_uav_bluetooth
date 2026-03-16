"""Core runtime lifecycle, scan, and housekeeping helpers for the Bluetooth node."""

import re
import time
from typing import Dict

import dbus
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mrs_uav_bluetooth.msg import BleDeviceArray, BleNotification
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

from .dbus_client import DeviceInfo
from .netplan import NetplanConfiguration


class BluetoothNodeRuntimeCoreMixin:

    OVERLAY_KEEPALIVE_SUFFIX_DEFAULT = "overlay_keepalive"
    DBUS_WARNING_INTERVAL_S = 5.0
    SHUTDOWN_DISCONNECT_TIMEOUT_S = 5.0
    DEVICES_PUBLISH_QUEUE_SIZE = 10
    NOTIFICATIONS_PUBLISH_QUEUE_SIZE = 50
    STATUS_PUBLISH_QUEUE_SIZE = 50
    LOG_PUBLISH_QUEUE_SIZE = 200

    def _ensure_core_publishers(self):
        if self.devices_pub is not None:
            return
        self.devices_pub = self.create_publisher(
            BleDeviceArray,
            self._node_topic("devices"),
            self.DEVICES_PUBLISH_QUEUE_SIZE,
        )
        self.notifications_pub = self.create_publisher(
            BleNotification,
            self._node_topic("notifications"),
            self.NOTIFICATIONS_PUBLISH_QUEUE_SIZE,
        )
        self.status_pub = self.create_publisher(
            String,
            self._node_topic("status"),
            self.STATUS_PUBLISH_QUEUE_SIZE,
        )
        self.log_pub = self.create_publisher(
            String,
            self._node_topic("log"),
            self.LOG_PUBLISH_QUEUE_SIZE,
        )

    def _setup_ros_interfaces(self):
        keepalive_suffix = str(
            self.get_parameter("overlay_keepalive_topic_suffix").value or self.OVERLAY_KEEPALIVE_SUFFIX_DEFAULT
        ).strip().strip("/")
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
                self._ensure_scan_running(reason="config update")
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

    def _start_scan(self, transport: str):
        ok = self._dbus.start_scan(transport)
        if ok:
            self._scan_transport = transport
        return ok

    def _stop_scan(self):
        return self._dbus.stop_scan()

    def _ensure_scan_running(self, *, reason: str) -> bool:
        if self._client is None:
            return False
        desired_transport = str(self.get_parameter("scan_mode").value)
        scanning = bool(self._client.scanning)
        if scanning and self._scan_transport == desired_transport:
            return True
        if scanning and self._scan_transport != desired_transport:
            self._log_verbose(
                f"Restarting BLE scan for transport change: {self._scan_transport} -> {desired_transport} ({reason})"
            )
            self._stop_scan()
        else:
            self._log_verbose(f"BLE scan inactive, restarting ({reason})")
        ok = self._start_scan(desired_transport)
        if not ok:
            self._dbus_warning(
                "scan_restart",
                f"BLE discovery is not active and restart failed (transport={desired_transport}, reason={reason})",
            )
            return False
        self.get_logger().info(f"BLE scan active (transport={desired_transport}, reason={reason})")
        return True

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
            self._sync_scan_state_for_publish()
            self._check_overlay_config_lease()
            snapshot = self._client.get_devices(refresh=True)
            snapshot = self._enforce_peer_connection_policy(snapshot=snapshot, reason="scan policy tick")
            self._refresh_notification_mapping(snapshot)
            self._sync_auto_import_bridges(snapshot)
            self._reconcile_import_bridges(snapshot)
            msg = BleDeviceArray()
            msg.header = self._header(frame_id=self._local_frame_id())
            msg.devices = [self._device_to_msg(device) for device in snapshot.values()]
            self.devices_pub.publish(msg)
            self._cleanup_peer_time_bridges(snapshot)
            self._log_verbose(f"Scan results: {len(snapshot)} device(s)")
            self._log_discovered_devices_summary(
                snapshot,
                pattern=str(self.get_parameter("auto_connect_pattern").value),
            )
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("scan_results", f"Skipping BLE scan publish tick due to DBus error: {exc}")

    def _sync_scan_state_for_publish(self):
        if self._client is None or not bool(self.get_parameter("enable_scan").value):
            return
        self._ensure_scan_running(reason="scan publish tick")

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
                    self._client.disconnect(device.mac, timeout=self.SHUTDOWN_DISCONNECT_TIMEOUT_S)
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