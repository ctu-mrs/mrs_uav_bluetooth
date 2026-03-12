"""ROS 2 BLE node providing MRS UAV Bluetooth server and client control."""

import hashlib
import logging
import os
import re
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import dbus
import rclpy
import rclpy.exceptions
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from mrs_uav_bluetooth.msg import (
    BleDevice,
    BleDeviceArray,
    BleGattCharacteristic,
    BleGattDescriptor,
    BleGattService,
    BleNotification,
    BlePeerTimeStatus,
)
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

from .bridge_payload import (
    BridgeMemberSpec,
    bytes_to_serializable,
    decode_message_payload,
    encode_message_payload,
    member_specs_from_serializable,
    normalize_member_specs,
    payload_format_for_member_specs,
)
from .bluetooth_bridge_state import PeerTimeBridgeState, TopicExportBridgeState, TopicImportBridgeState
from .bluetooth_dbus_runtime import BluetoothDbusRuntime
from .dbus_client import BleClient, DeviceInfo
from .dbus_common import BLUEZ_SERVICE_NAME, DBUS_PROP_IFACE, GATT_CHRC_IFACE, GATT_DESC_IFACE
from .gatt_services import (
    TIME_CHARACTERISTIC_UUID,
    TIME_WRITEBACK_DESCRIPTOR_UUID,
    topic_bridge_characteristic_uuid,
    topic_bridge_data_descriptor_uuid,
    topic_bridge_metadata_descriptor_uuid,
)
from .netplan import NetplanConfiguration
from .uuid_utils import is_uav_hostname, resolve_uuid, sanitize_topic_suffix, system_hostname


@dataclass(frozen=True)
class SharedTopicConfig:
    name: str
    mode: str
    bridge_key: str
    bridge_name: str
    export_topic: str
    import_topic_suffix: str
    message_type: str
    message_class: type
    rate_hz: float
    transport_endpoint: str
    payload_format: str
    member_specs: Tuple[BridgeMemberSpec, ...]


class BluetoothNode(Node):
    _CONFIG_PARAM_NAMES = [
        "advertise_mode",
        "pairing_agent",
        "auto_accept_pairing",
        "auto_trust",
        "enable_scan",
        "scan_mode",
        "scan_publish_period",
        "time_update_period",
        "wifi_refresh_period",
        "auto_connect_period",
        "auto_connect_whitelist",
        "auto_connect_enable",
        "auto_connect_pattern",
        "peer_connection_timeout",
        "netplan_config_file",
        "netplan_scripts_dir",
        "allowed_wifi_networks",
        "enable_server",
        "enable_time_service",
        "enable_wifi_service",
        "discoverable_timeout",
        "status_report_period",
        "verbose_log_file",
    ]

    def __init__(self):
        super().__init__("mrs_uav_bluetooth")
        self._declare_parameters()
        self._lock = threading.RLock()
        self._local_name = system_hostname() or "mrs-uav"
        self._pending_wifi_password = ""
        self._auto_connect_attempts: Dict[str, float] = {}
        self._peer_security_attempts: Dict[str, float] = {}
        self._peer_inactive_since: Dict[str, float] = {}
        self._notification_path_to_mac: Dict[str, str] = {}
        self._last_gatt_warning_at: Dict[Tuple[str, str], float] = {}
        self._scan_transport = self.get_parameter("scan_mode").get_parameter_value().string_value
        self._topic_exports: Dict[str, TopicExportBridgeState] = {}
        self._notification_bridges: Dict[str, TopicImportBridgeState] = {}
        self._peer_time_bridges: Dict[str, PeerTimeBridgeState] = {}
        self._shared_topic_configs: Dict[str, SharedTopicConfig] = {}
        self._active_overlay_path = ""
        self._active_overlay_deadline = 0.0
        self._default_config_path = self._resolve_default_config_path()
        self._active_config_source = self._default_config_path
        self._node_topics_prefix = self._format_node_topics_prefix("/{hostname}/ble")
        self._shutting_down = False

        self.devices_pub = None
        self.notifications_pub = None
        self.log_pub = None
        self._verbose_logger = None  # type: Optional[logging.Logger]
        self._netplan = None
        self._scan_results_timer = None
        self._time_service_timer = None
        self._wifi_service_timer = None
        self._auto_connect_timer = None
        self._status_timer = None
        self._config_lease_timer = None

        self._apply_config_document(self._load_effective_config(), source_path=self._default_config_path, initial=True)
        self._ensure_core_publishers()
        self._setup_verbose_logger()
        self._dbus = BluetoothDbusRuntime(self._local_name, self.get_logger(), self._log_verbose, self._on_pairing_event)

        self._setup_ros_interfaces()
        self._setup_bluetooth()
        self._reconfigure_timers()

        self.get_logger().info(
            f"BluetoothNode started: adapter={self._adapter_path}, "
            f"server={'ON' if bool(self.get_parameter('enable_server').value) else 'OFF'}, "
            f"scan={'ON' if bool(self.get_parameter('enable_scan').value) else 'OFF'}, "
            f"hostname={self._local_name}, prefix={self._node_topics_prefix}"
        )

    @property
    def _adapter_path(self) -> str:
        return self._dbus.adapter_path

    @property
    def _app(self):
        return self._dbus.app

    @property
    def _advertisement(self):
        return self._dbus.advertisement

    @property
    def _bus(self):
        return self._dbus.bus

    @property
    def _client(self) -> Optional[BleClient]:
        return self._dbus.client

    @property
    def _time_service(self):
        return self._dbus.time_service

    @property
    def _wifi_service(self):
        return self._dbus.wifi_service

    def _declare_parameters(self):
        self.declare_parameter("default_config_path", "")
        self.declare_parameter("discoverable_timeout", 0)
        self.declare_parameter("advertise_mode", "peripheral")
        self.declare_parameter("pairing_agent", "NoInputNoOutput")
        self.declare_parameter("auto_accept_pairing", True)
        self.declare_parameter("auto_trust", True)
        self.declare_parameter("enable_scan", True)
        self.declare_parameter("scan_mode", "le")
        self.declare_parameter("scan_publish_period", 2.0)
        self.declare_parameter("time_update_period", 1.0)
        self.declare_parameter("wifi_refresh_period", 2.0)
        self.declare_parameter("auto_connect_period", 3.0)
        self.declare_parameter("auto_connect_whitelist", [])
        self.declare_parameter(
            "auto_connect_enable",
            False,
            ParameterDescriptor(description="Global switch for automatic peer connect/pair/trust handling."),
        )
        self.declare_parameter("auto_connect_pattern", r"^uav[0-9]{2}$")
        self.declare_parameter("peer_connection_timeout", 30.0)
        self.declare_parameter("netplan_config_file", "/etc/netplan/01-netcfg.yaml")
        self.declare_parameter("netplan_scripts_dir", "/etc/ctu-mrs/uav-bluetooth/netplan-scripts")
        self.declare_parameter("allowed_wifi_networks", [])
        self.declare_parameter("enable_server", True)
        self.declare_parameter("enable_time_service", True)
        self.declare_parameter("enable_wifi_service", True)
        self.declare_parameter("status_report_period", 10.0)
        self.declare_parameter("verbose_log_file", "")

    def _resolve_default_config_path(self) -> str:
        configured = str(self.get_parameter("default_config_path").value or "").strip()
        if configured:
            return configured
        return os.path.join(get_package_share_directory("mrs_uav_bluetooth"), "config", "default.yaml")

    def _ensure_core_publishers(self):
        if self.devices_pub is not None:
            return
        self.devices_pub = self.create_publisher(BleDeviceArray, self._node_topic("devices"), 10)
        self.notifications_pub = self.create_publisher(BleNotification, self._node_topic("notifications"), 50)
        self.log_pub = self.create_publisher(String, self._node_topic("log"), 200)

    def _node_topic(self, suffix: str) -> str:
        normalized = str(suffix or "").strip().lstrip("/")
        return self._node_topics_prefix if not normalized else f"{self._node_topics_prefix}/{normalized}"

    def _format_node_topics_prefix(self, value: str) -> str:
        candidate = str(value or "").strip() or "/{hostname}/ble"
        candidate = candidate.replace("{hostname}", sanitize_topic_suffix(self._local_name))
        if not candidate.startswith("/"):
            candidate = f"/{candidate}"
        return candidate.rstrip("/") or "/"

    def _format_shared_topic_name(self, value: str) -> str:
        return str(value or "").replace("{hostname}", sanitize_topic_suffix(self._local_name))

    def _normalize_ros_topic(self, value: str) -> str:
        candidate = re.sub(r"/+", "/", str(value or "").strip())
        if not candidate:
            return "/"
        if not candidate.startswith("/"):
            candidate = f"/{candidate}"
        candidate = candidate.rstrip("/")
        return candidate or "/"

    def _canonical_shared_topic(self, export_topic: str) -> str:
        normalized = self._normalize_ros_topic(export_topic)
        segments = [segment for segment in normalized.split("/") if segment]
        if not segments:
            return normalized
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        if is_uav_hostname(segments[0], pattern=pattern):
            if len(segments) == 1:
                return "/"
            return "/" + "/".join(segments[1:])
        return normalized

    def _load_yaml_mapping(self, path: str) -> dict:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise ValueError(f"Config {path} must contain a YAML mapping at the top level")
        return data

    def _deep_merge(self, base, override):
        if isinstance(base, dict) and isinstance(override, dict):
            merged = dict(base)
            for key, value in override.items():
                merged[key] = self._deep_merge(merged.get(key), value)
            return merged
        return override

    def _load_effective_config(self, overlay_path: str = "") -> dict:
        config = self._load_yaml_mapping(self._default_config_path)
        if overlay_path:
            config = self._deep_merge(config, self._load_yaml_mapping(overlay_path))
        return config

    def _make_parameter(self, name: str, value):
        return rclpy.parameter.Parameter(name, rclpy.parameter.Parameter.Type.from_parameter_value(value), value)

    def _apply_config_document(self, config: dict, *, source_path: str, initial: bool = False):
        params = []
        for name in self._CONFIG_PARAM_NAMES:
            if name in config:
                params.append(self._make_parameter(name, config[name]))
        if params:
            self.set_parameters(params)
        self._scan_transport = str(self.get_parameter("scan_mode").value)

        requested_prefix = self._format_node_topics_prefix(config.get("node_topics_prefix", self._node_topics_prefix))
        if initial:
            self._node_topics_prefix = requested_prefix
        elif requested_prefix != self._node_topics_prefix:
            self.get_logger().warning(
                f"Ignoring runtime node_topics_prefix change ({requested_prefix}); keeping {self._node_topics_prefix}"
            )

        self._shared_topic_configs = self._parse_shared_topics(config.get("shared_topics", []))
        self._active_config_source = source_path

    def _parse_shared_topics(self, raw_items) -> Dict[str, SharedTopicConfig]:
        configs = {}
        for index, raw in enumerate(raw_items or []):
            if not isinstance(raw, dict):
                raise ValueError(f"shared_topics[{index}] must be a mapping")
            mode = str(raw.get("mode", "both")).strip().lower() or "both"
            if mode not in {"export", "import", "both"}:
                raise ValueError(f"shared_topics[{index}].mode must be export, import, or both")
            export_topic = self._normalize_ros_topic(self._format_shared_topic_name(raw.get("export_topic", "")))
            if export_topic == "/":
                raise ValueError(f"shared_topics[{index}] requires export_topic")
            canonical_topic = self._canonical_shared_topic(export_topic)
            key_source = str(raw.get("key", canonical_topic)).strip()
            if not key_source:
                raise ValueError(f"shared_topics[{index}] produced an empty bridge key")
            bridge_key = hashlib.md5(key_source.encode("utf-8")).hexdigest()
            bridge_name = bridge_key
            message_type = str(raw.get("message_type", "")).strip()
            if not message_type:
                raise ValueError(f"shared_topics[{index}] requires message_type")
            message_class = get_message(message_type)
            member_specs = normalize_member_specs(raw.get("members", []), message_class=message_class)
            if not member_specs:
                raise ValueError(f"shared_topics[{index}] requires at least one compact member definition")
            payload_format = payload_format_for_member_specs(member_specs)
            import_topic_suffix = self._normalize_ros_topic(raw.get("import_topic_suffix", canonical_topic))
            transport_endpoint = self._normalize_transport_endpoint(str(raw.get("transport_endpoint", "characteristic")))
            rate_hz = max(0.0, float(raw.get("rate_hz", 0.0)))
            name = str(raw.get("name", canonical_topic)).strip() or canonical_topic
            if bridge_key in configs:
                raise ValueError(f"shared_topics[{index}] duplicates bridge key for {canonical_topic}")
            configs[bridge_key] = SharedTopicConfig(
                name=name,
                mode=mode,
                bridge_key=bridge_key,
                bridge_name=bridge_name,
                export_topic=export_topic,
                import_topic_suffix=import_topic_suffix.lstrip("/"),
                message_type=message_type,
                message_class=message_class,
                rate_hz=rate_hz,
                transport_endpoint=transport_endpoint,
                payload_format=payload_format,
                member_specs=member_specs,
            )
        return configs

    def _get_string_list(self, name: str):
        try:
            value = self.get_parameter(name).value
            return list(value) if value else []
        except rclpy.exceptions.ParameterUninitializedException:
            return []

    def _get_auto_connect_whitelist(self):
        names = set()
        macs = set()
        for item in self._get_string_list("auto_connect_whitelist"):
            candidate = str(item).strip()
            if not candidate:
                continue
            if re.fullmatch(r"(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}", candidate):
                macs.add(candidate.upper())
            else:
                names.add(candidate.lower())
        return names, macs

    def _matches_auto_connect_whitelist(self, device: DeviceInfo, target_names, target_macs) -> bool:
        if device.mac in target_macs:
            return True
        name_fields = (device.name or "", device.alias or "", self._hostname_from_device(device))
        return any(field.strip().lower() in target_names for field in name_fields if field.strip())

    def _resolve_peer_topic_name(self, mac: str, topic_name: str, device: Optional[DeviceInfo] = None) -> str:
        current_device = device or (self._client.get_device(mac, refresh=True) if self._client is not None else None)
        peer_name = self._hostname_from_device(current_device) if current_device is not None else ""
        peer_segment = sanitize_topic_suffix(peer_name or mac.lower().replace(":", "_"))
        scoped_prefix = self._node_topic(f"peers/{peer_segment}")
        normalized_topic = str(topic_name or "").strip() or "/"
        if not normalized_topic.startswith("/"):
            normalized_topic = f"/{normalized_topic}"
        if normalized_topic == scoped_prefix or normalized_topic.startswith(scoped_prefix + "/"):
            return normalized_topic
        return f"{scoped_prefix}{normalized_topic}"

    def _peer_display_name(self, mac: str) -> str:
        state = self._peer_time_bridges.get(mac)
        if state is not None and state.peer_name:
            return state.peer_name
        device = self._client.get_device(mac) if self._client is not None else None
        if device is not None:
            return self._hostname_from_device(device) or mac
        return mac

    def _update_publish_rate(self, last_publish_monotonic: float):
        now = time.monotonic()
        if last_publish_monotonic > 0 and now > last_publish_monotonic:
            return now, 1.0 / (now - last_publish_monotonic)
        return now, 0.0

    def _setup_verbose_logger(self):
        log_path = str(self.get_parameter("verbose_log_file").value or "").strip()
        logger = logging.getLogger("mrs_uav_bluetooth.verbose")
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        for handler in list(logger.handlers):
            handler.close()
            logger.removeHandler(handler)
        if not log_path:
            self._verbose_logger = None
            return
        try:
            os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
            file_handler = logging.FileHandler(log_path, mode="a", encoding="utf-8")
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
            logger.addHandler(file_handler)
            self._verbose_logger = logger
            self._verbose_logger.info("Verbose file logger started")
        except Exception as exc:
            self._verbose_logger = None
            self.get_logger().warning(f"Failed to open verbose log file {log_path}: {exc}")

    def _log_verbose(self, message: str):
        verbose_logger = self._verbose_logger
        if verbose_logger is not None:
            verbose_logger.debug(message)

        if self.log_pub is None or self._shutting_down or not self._ros_context_ok():
            return

        log_msg = String()
        log_msg.data = message
        try:
            self.log_pub.publish(log_msg)
        except Exception:
            if self._shutting_down or not self._ros_context_ok():
                return
            raise

    def _ros_context_ok(self) -> bool:
        try:
            return bool(self.context.ok())
        except Exception:
            return False

    def _reconfigure_timers(self):
        self._recreate_timer("_scan_results_timer", max(0.2, float(self.get_parameter("scan_publish_period").value)), self._publish_scan_results)
        self._recreate_timer("_time_service_timer", max(0.2, float(self.get_parameter("time_update_period").value)), self._update_time_service)
        self._recreate_timer("_wifi_service_timer", max(0.5, float(self.get_parameter("wifi_refresh_period").value)), self._refresh_wifi_service)
        self._recreate_timer("_auto_connect_timer", max(0.5, float(self.get_parameter("auto_connect_period").value)), self._auto_connect_devices)
        status_period = float(self.get_parameter("status_report_period").value)
        if status_period > 0:
            self._recreate_timer("_status_timer", max(1.0, status_period), self._publish_status_report)
        else:
            self._destroy_timer_attr("_status_timer")
        self._recreate_timer("_config_lease_timer", 1.0, self._check_overlay_config_lease)

    def _recreate_timer(self, attr_name: str, period: float, callback):
        self._destroy_timer_attr(attr_name)
        setattr(self, attr_name, self.create_timer(period, callback))

    def _destroy_timer_attr(self, attr_name: str):
        timer = getattr(self, attr_name, None)
        if timer is not None:
            self.destroy_timer(timer)
            setattr(self, attr_name, None)

    def _publish_status_report(self):
        try:
            lines = self._build_status_lines()
        except Exception as exc:
            self.get_logger().warning(f"Status report failed: {exc}")
            return
        report = "\n".join(lines)
        self.get_logger().info(f"[STATUS]\n{report}")
        self._log_verbose(f"[STATUS]\n{report}")

    def _build_status_lines(self):
        lines = []
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        lines.append(f"--- Bluetooth Node Status @ {ts} ---")
        lines.append(f"  adapter:    {self._adapter_path}")
        lines.append(f"  hostname:   {self._local_name}")
        lines.append(f"  prefix:     {self._node_topics_prefix}")
        lines.append(f"  config:     {self._active_config_source}")
        lines.append(f"  scanning:   {self._client.scanning if self._client else False}")
        lines.append(f"  server:     {'ACTIVE' if self._app is not None else 'OFF'}")
        lines.append(f"  advertise:  {'ACTIVE' if self._advertisement is not None else 'OFF'}")
        if self._wifi_service is not None:
            lines.append("  wifi-svc:   enabled")
        if self._time_service is not None:
            lines.append("  time-svc:   enabled")
        if self._client is not None:
            with self._lock:
                all_devices = self._client.get_devices()
            pattern = str(self.get_parameter("auto_connect_pattern").value)
            peers = {mac: dev for mac, dev in all_devices.items() if dev.connected and self._is_uav_peer_candidate(dev, pattern)}
            other_connected = {mac: dev for mac, dev in all_devices.items() if dev.connected and mac not in peers}
            lines.append(f"  discovered: {len(all_devices)} devices")
            lines.append(f"  connected:  {len(peers) + len(other_connected)} devices")
            for mac, dev in sorted(peers.items()):
                bridge_state = self._peer_time_bridges.get(mac)
                inactivity = max(0.0, time.monotonic() - bridge_state.last_activity_monotonic) if bridge_state is not None else -1.0
                lines.append(
                    f"    peer: {mac} {dev.alias or dev.name or '?'} RSSI={dev.rssi} paired={dev.paired and dev.trusted and dev.bonded} inactive_s={inactivity:.1f}"
                )
            if not peers:
                lines.append("    peers: (none)")
            if other_connected:
                lines.append("    other:")
                for mac, dev in sorted(other_connected.items()):
                    lines.append(f"      {mac} {dev.alias or dev.name or '?'} RSSI={dev.rssi}")
            else:
                lines.append("    other: (none)")
        if self._topic_exports:
            lines.append(f"  export bridges ({len(self._topic_exports)}):")
            for state in self._topic_exports.values():
                bridge_path = state.service.transport_path(state.transport_endpoint) if state.service is not None else "?"
                lines.append(f"    {state.topic_name} -> {state.bridge_key} [{bridge_path}]")
        if self._notification_bridges:
            lines.append(f"  import bridges ({len(self._notification_bridges)}):")
            for state in self._notification_bridges.values():
                lines.append(f"    {state.mac} {state.bridge_key} -> {state.resolved_topic_name}")
        peer_topic_lines = self._build_peer_topic_status_lines()
        if peer_topic_lines:
            lines.append("  peer topics:")
            lines.extend(peer_topic_lines)
        lines.append("---")
        return lines

    def _build_peer_topic_status_lines(self):
        per_peer = {}
        for mac, state in self._peer_time_bridges.items():
            per_peer.setdefault(mac, []).append(f"{state.status_topic_name} @ {state.current_hz:.2f} Hz")
        for state in self._notification_bridges.values():
            per_peer.setdefault(state.mac, []).append(f"{state.resolved_topic_name} @ {state.current_hz:.2f} Hz")
        lines = []
        for mac, topics in sorted(per_peer.items()):
            lines.append(f"    {self._peer_display_name(mac)} ({mac}): {', '.join(topics)}")
        return lines

    def _setup_ros_interfaces(self):
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
                pass  # Already scanning with same transport
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

    def _publish_scan_results(self):
        if self._client is None:
            return
        snapshot = self._client.get_devices(refresh=True)
        self._sync_auto_import_bridges(snapshot)
        self._reconcile_import_bridges(snapshot)
        msg = BleDeviceArray()
        msg.header = self._header()
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
                    if transport_endpoint == "characteristic" and not self._client.start_notify(path):
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
                    if not self._client.start_notify(path):
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
            for characteristic in self._client.list_characteristics(mac):
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
            if timeout > 0 and now - self._peer_inactive_since[mac] >= timeout:
                self.get_logger().warning(
                    f"Disconnecting {mac}: no active peer time bridge for {now - self._peer_inactive_since[mac]:.1f}s"
                )
                self._client.disconnect(mac, timeout=5.0)
                self._peer_inactive_since.pop(mac, None)
        for mac in stale_macs:
            state = self._peer_time_bridges.pop(mac, None)
            if state is not None:
                self.destroy_publisher(state.publisher)
                self._stop_notify_if_unused(state.characteristic_path)
            self._peer_security_attempts.pop(mac, None)
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
            if not self._client.start_notify(state.path):
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
        now = time.monotonic()
        retry_period = max(1.0, float(self.get_parameter("auto_connect_period").value))
        whitelist_names, whitelist_macs = self._get_auto_connect_whitelist()
        whitelist_enabled = bool(whitelist_names or whitelist_macs)
        pattern = str(self.get_parameter("auto_connect_pattern").value)
        snapshot = self._client.get_devices(refresh=True)
        self._auto_connect_attempts = {mac: stamp for mac, stamp in self._auto_connect_attempts.items() if mac in snapshot}
        self._peer_security_attempts = {mac: stamp for mac, stamp in self._peer_security_attempts.items() if mac in snapshot}
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
            if not self._client.connect(mac, timeout=10.0):
                self.get_logger().warning(f"Auto-connect failed: {device_label}")
                self._log_verbose(f"Auto-connect failed: {device_label}")
                continue
            self._client.wait_services_resolved(mac, timeout=10.0)
            current = self._client.get_device(mac, refresh=True) or device
            if peer_candidate and not self._maintain_peer_connection(mac, current, retry_period, explicit_target=explicit_target) and not explicit_target:
                self.get_logger().info(f"Disconnecting {mac}: UAV peer candidate without BLE time characteristic")
                self._log_verbose(f"Disconnecting {device_label}: no time characteristic found")
                self._client.disconnect(mac, timeout=5.0)
            else:
                self._auto_connect_attempts.pop(mac, None)
                self.get_logger().info(f"Auto-connected BLE device {mac}")
                self._log_verbose(f"Auto-connected: {device_label}")

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

    def _ensure_peer_time_bridge(self, mac: str, device: DeviceInfo) -> bool:
        path = self._client.find_characteristic(mac, TIME_CHARACTERISTIC_UUID)
        device_label = f"{mac} ({self._hostname_from_device(device) or '?'})"
        if not path:
            self._peer_inactive_since.setdefault(mac, time.monotonic())
            self._log_verbose(f"Peer {device_label}: time characteristic {TIME_CHARACTERISTIC_UUID} not found")
            return False
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
                return True
            started = bool(self._client.start_notify(path))
            if started:
                self._prime_peer_time_bridge(existing)
            return started
        if existing is not None:
            old_path = existing.characteristic_path
            self.destroy_publisher(existing.publisher)
            self._peer_time_bridges.pop(mac, None)
            self._stop_notify_if_unused(old_path)
        publisher = self.create_publisher(BlePeerTimeStatus, status_topic_name, 10)
        if not self._client.start_notify(path):
            self.get_logger().warning(f"Failed to start time notify for peer {device_label}")
            self._log_verbose(f"Failed to start_notify on time characteristic {path} for {device_label}")
            self.destroy_publisher(publisher)
            return False
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
        self._peer_inactive_since.pop(mac, None)
        self._prime_peer_time_bridge(self._peer_time_bridges[mac])
        return True

    def _prime_peer_time_bridge(self, state: PeerTimeBridgeState):
        payload = self._client.read_characteristic(state.characteristic_path)
        if payload is not None:
            self._process_peer_time_payload(state.mac, state.characteristic_path, payload)

    def _maintain_peer_connection(self, mac: str, device: DeviceInfo, retry_period: float, explicit_target: bool = False) -> bool:
        time_bridge_ready = self._ensure_peer_time_bridge(mac, device) if self._is_uav_peer_candidate(device, str(self.get_parameter("auto_connect_pattern").value)) else False
        self._ensure_peer_security(mac, device, retry_period)
        return time_bridge_ready or explicit_target

    def _ensure_peer_security(self, mac: str, device: DeviceInfo, retry_period: float):
        current = self._client.get_device(mac, refresh=True) or device
        if current.paired and current.trusted and current.bonded:
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
        if not current.paired or not current.bonded:
            if self._client.pair(mac, timeout=30.0):
                self.get_logger().info(f"Paired BLE peer {mac}")
                self._log_verbose(f"Paired: {device_label}")
            else:
                self.get_logger().warning(f"Failed to pair BLE peer {mac}")
                self._log_verbose(f"Pairing failed: {device_label}")
            current = self._client.get_device(mac, refresh=True) or current
        if current.paired and not current.trusted:
            if self._client.trust(mac):
                self.get_logger().info(f"Trusted BLE peer {mac}")
                self._log_verbose(f"Trusted: {device_label}")
            else:
                self.get_logger().warning(f"Failed to trust BLE peer {mac}")
                self._log_verbose(f"Trust failed: {device_label}")
            current = self._client.get_device(mac, refresh=True) or current
        if current.paired and current.trusted and current.bonded:
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
        self._client.stop_notify(path)

    def _header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "bluetooth"
        return header

    def _time_msg(self, stamp: float) -> TimeMsg:
        secs = int(stamp)
        nanosec = int((stamp - secs) * 1e9)
        return TimeMsg(sec=secs, nanosec=nanosec)

    def _device_to_msg(self, device: DeviceInfo) -> BleDevice:
        msg = BleDevice()
        msg.mac = device.mac
        msg.path = device.path or ""
        msg.adapter = device.adapter or ""
        msg.address_type = device.address_type or ""
        msg.name = device.name or ""
        msg.alias = device.alias or ""
        msg.hostname = self._hostname_from_device(device)
        msg.icon = device.icon or ""
        msg.appearance = int(device.appearance or 0)
        msg.rssi = int(device.rssi or 0)
        msg.tx_power = int(device.tx_power or 0)
        msg.pathloss = int(device.pathloss or 0)
        msg.connected = bool(device.connected)
        msg.paired = bool(device.paired)
        msg.bonded = bool(device.bonded)
        msg.trusted = bool(device.trusted)
        msg.blocked = bool(device.blocked)
        msg.services_resolved = bool(device.services_resolved)
        msg.uuids = list(device.uuids)
        msg.manufacturer_data_hex = [f"{key}:{value.hex()}" for key, value in sorted(device.manufacturer_data.items())]
        msg.service_data_hex = [f"{key}:{value.hex()}" for key, value in sorted(device.service_data.items())]
        msg.last_seen = self._time_msg(device.last_seen)
        return msg

    def _service_to_msg(self, item: dict) -> BleGattService:
        msg = BleGattService()
        msg.path = item.get("path", "")
        msg.uuid = item.get("uuid", "")
        msg.primary = bool(item.get("primary", False))
        msg.device_path = item.get("device", "")
        msg.includes = list(item.get("includes", []))
        return msg

    def _characteristic_to_msg(self, item: dict) -> BleGattCharacteristic:
        msg = BleGattCharacteristic()
        msg.path = item.get("path", "")
        msg.service_path = item.get("service", "")
        msg.uuid = item.get("uuid", "")
        msg.flags = list(item.get("flags", []))
        msg.notifying = bool(item.get("notifying", False))
        msg.mtu = int(item.get("mtu", 0))
        return msg

    def _descriptor_to_msg(self, item: dict) -> BleGattDescriptor:
        msg = BleGattDescriptor()
        msg.path = item.get("path", "")
        msg.characteristic_path = item.get("characteristic", "")
        msg.uuid = item.get("uuid", "")
        msg.flags = list(item.get("flags", []))
        return msg

    def _on_notification(self, data: bytes, uuid: str, chrc_path: str):
        mac = self._notification_path_to_mac.get(chrc_path, "")
        self._log_verbose(f"Notification mac={mac} uuid={uuid} path={chrc_path} len={len(data)}")
        msg = BleNotification()
        msg.header = self._header()
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

    def _publish_peer_time_status(self, state: PeerTimeBridgeState):
        message = BlePeerTimeStatus()
        message.header = self._header()
        message.mac = state.mac
        message.peer_name = state.peer_name
        ns = max(0, int(state.last_time_value_ns))
        message.peer_stamp = TimeMsg(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))
        message.last_rtt_s = max(0.0, state.last_rtt_s)
        state.publisher.publish(message)

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

    def _resolve_message_type(self, topic_name: str, explicit_message_type: str, prefer_publishers: bool) -> Tuple[str, type]:
        explicit = explicit_message_type.strip()
        if explicit:
            return explicit, get_message(explicit)
        if prefer_publishers:
            info_sets = [self.get_publishers_info_by_topic(topic_name), self.get_subscriptions_info_by_topic(topic_name)]
        else:
            info_sets = [self.get_subscriptions_info_by_topic(topic_name), self.get_publishers_info_by_topic(topic_name)]
        candidates = []
        for infos in info_sets:
            for info in infos:
                if info.topic_type not in candidates:
                    candidates.append(info.topic_type)
            if candidates:
                break
        if not candidates:
            raise ValueError(f"Unable to resolve ROS message type for topic {topic_name}")
        if len(candidates) > 1:
            raise ValueError(f"Multiple ROS message types found for topic {topic_name}: {candidates}")
        return candidates[0], get_message(candidates[0])

    def _normalize_transport_endpoint(self, endpoint: str) -> str:
        candidate = endpoint.strip().lower() or "characteristic"
        if candidate not in {"characteristic", "descriptor"}:
            raise ValueError("transport_endpoint must be 'characteristic' or 'descriptor'")
        return candidate

    def _resolve_remote_characteristic(self, mac: str, identifier: str, transport_endpoint: str) -> Tuple[str, str, str]:
        candidate = identifier.strip()
        if candidate.startswith("/"):
            props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, candidate), DBUS_PROP_IFACE)
            try:
                uuid = str(props.Get(GATT_CHRC_IFACE, "UUID"))
                return candidate, uuid, "characteristic"
            except Exception:
                try:
                    uuid = str(props.Get(GATT_DESC_IFACE, "UUID"))
                    return candidate, uuid, "descriptor"
                except Exception:
                    return candidate, "", transport_endpoint
        if transport_endpoint == "descriptor":
            descriptor_uuid = topic_bridge_data_descriptor_uuid(candidate)
            path = self._client.find_descriptor(mac, descriptor_uuid)
            if path:
                return path, descriptor_uuid, transport_endpoint
        else:
            characteristic_uuid = topic_bridge_characteristic_uuid(candidate)
            path = self._client.find_characteristic(mac, characteristic_uuid)
            if path:
                return path, characteristic_uuid, transport_endpoint
        resolved_uuid = resolve_uuid(candidate)
        if transport_endpoint == "descriptor":
            path = self._client.find_descriptor(mac, resolved_uuid)
        else:
            path = self._client.find_characteristic(mac, resolved_uuid)
        return path or "", resolved_uuid, transport_endpoint

    def _handle_list_devices(self, request, response):
        devices = self._client.get_connected_devices(refresh=True) if request.connected_only else self._client.get_devices(refresh=True)
        response.success = True
        response.message = "ok"
        response.devices = [self._device_to_msg(device) for device in devices.values()]
        return response

    def _handle_get_device(self, request, response):
        device = self._client.get_device(request.mac, refresh=True)
        if device is None:
            response.success = False
            response.message = f"device {request.mac} not found"
            return response
        response.success = True
        response.message = "ok"
        response.device = self._device_to_msg(device)
        return response

    def _handle_connect_device(self, request, response):
        success = self._client.connect(request.mac, timeout=max(1.0, request.timeout or 15.0))
        if success and request.wait_for_services:
            success = self._client.wait_services_resolved(request.mac, timeout=max(1.0, request.timeout or 15.0))
        device = self._client.get_device(request.mac, refresh=True)
        response.success = bool(success)
        response.message = "ok" if success else f"failed to connect {request.mac}"
        response.resolved_mac = device.mac if device else request.mac.upper()
        response.device_path = device.path if device else ""
        return response

    def _handle_disconnect_device(self, request, response):
        response.success = self._client.disconnect(request.mac, timeout=max(1.0, request.timeout or 10.0))
        response.message = "ok" if response.success else f"failed to disconnect {request.mac}"
        return response

    def _handle_pair_device(self, request, response):
        success = self._client.pair(request.mac, timeout=max(1.0, request.timeout or 30.0))
        if success and request.trust_after_pair:
            success = self._client.trust(request.mac)
        response.success = bool(success)
        response.message = "ok" if success else f"failed to pair {request.mac}"
        return response

    def _handle_set_device_trust(self, request, response):
        success = self._client.trust(request.mac) if request.trusted else self._client.untrust(request.mac)
        response.success = bool(success)
        response.message = "ok" if success else f"failed to set trust for {request.mac}"
        return response

    def _handle_remove_device(self, request, response):
        response.success = self._client.remove(request.mac)
        response.message = "ok" if response.success else f"failed to remove {request.mac}"
        return response

    def _handle_list_gatt_services(self, request, response):
        services = self._client.list_services(request.mac)
        response.success = True
        response.message = "ok"
        response.services = [self._service_to_msg(item) for item in services]
        return response

    def _handle_list_gatt_characteristics(self, request, response):
        items = self._client.list_characteristics(request.mac)
        response.success = True
        response.message = "ok"
        response.characteristics = [self._characteristic_to_msg(item) for item in items]
        return response

    def _handle_list_gatt_descriptors(self, request, response):
        items = self._client.list_descriptors(request.mac, chrc_path=request.characteristic_path or None)
        response.success = True
        response.message = "ok"
        response.descriptors = [self._descriptor_to_msg(item) for item in items]
        return response

    def _handle_find_gatt_path(self, request, response):
        resolved_name = resolve_uuid(request.uuid)
        if request.descriptor:
            path = self._client.find_descriptor(request.mac, resolved_name, chrc_path=request.characteristic_path or None)
        else:
            path = self._client.find_characteristic(request.mac, resolved_name)
        response.success = bool(path)
        response.message = "ok" if path else f"uuid {resolved_name} not found"
        response.path = path or ""
        return response

    def _handle_read_gatt_value(self, request, response):
        data = self._client.read_descriptor(request.path) if request.descriptor else self._client.read_characteristic(request.path)
        response.success = data is not None
        response.message = "ok" if data is not None else f"read failed for {request.path}"
        response.value = list(data or b"")
        return response

    def _handle_write_gatt_value(self, request, response):
        payload = bytes(request.value)
        if request.descriptor:
            success = self._client.write_descriptor(request.path, payload)
        else:
            success = self._client.write_characteristic(request.path, payload, with_response=request.with_response)
        response.success = bool(success)
        response.message = "ok" if success else f"write failed for {request.path}"
        return response

    def _handle_set_notify(self, request, response):
        success = self._client.start_notify(request.path) if request.enable else self._client.stop_notify(request.path)
        response.success = bool(success)
        response.message = "ok" if success else f"notify change failed for {request.path}"
        return response

    def _handle_set_scan_enabled(self, request, response):
        success = self._start_scan(request.transport or self._scan_transport) if request.enabled else self._stop_scan()
        response.success = bool(success)
        response.message = "ok" if success else "failed"
        response.scanning = bool(self._client.scanning)
        return response

    def _handle_configure_notification_bridge(self, request, response):
        direction = request.direction.strip().lower()
        try:
            if direction == "export":
                return self._configure_topic_export(request, response)
            if direction == "import":
                return self._configure_topic_import(request, response)
            response.success = False
            response.message = "direction must be 'export' or 'import'"
            return response
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response

    def _configure_topic_export(self, request, response):
        if not bool(self.get_parameter("enable_server").value):
            response.success = False
            response.message = "BLE server is disabled"
            return response
        topic_name = request.topic_name.strip()
        if not topic_name:
            response.success = False
            response.message = "topic_name is required"
            return response
        bridge_name = (request.characteristic or topic_name).strip()
        rate_hz = max(0.0, float(request.rate_hz))
        transport_endpoint = self._normalize_transport_endpoint(request.transport_endpoint)
        message_type, message_class = self._resolve_message_type(topic_name, request.message_type, prefer_publishers=True)
        member_specs = normalize_member_specs(request.member_paths, message_class=message_class)
        payload_format = payload_format_for_member_specs(member_specs)
        bridge_key = hashlib.md5(bridge_name.encode("utf-8")).hexdigest()
        bridge_uuid = topic_bridge_data_descriptor_uuid(bridge_name) if transport_endpoint == "descriptor" else topic_bridge_characteristic_uuid(bridge_name)
        existing_key = None
        for key, state in self._topic_exports.items():
            if state.topic_name == topic_name and state.bridge_name == bridge_name and not state.auto_managed:
                existing_key = key
                break

        if not request.enable:
            if existing_key is None:
                response.success = False
                response.message = "topic export bridge not found"
                return response
            state = self._topic_exports.pop(existing_key)
            self._destroy_export_bridge(state)
            self.destroy_subscription(state.subscription)
            self._rebuild_server()
            response.success = True
            response.message = "removed"
            response.resolved_uuid = state.bridge_uuid
            response.resolved_path = ""
            response.resolved_topic = state.topic_name
            response.resolved_message_type = state.message_type
            response.resolved_member_paths = list(state.member_paths)
            response.resolved_rate_hz = float(state.rate_hz)
            response.resolved_transport_endpoint = state.transport_endpoint
            return response

        if existing_key is None:
            key = f"manual-export::{topic_name}::{bridge_name}"
            subscription = self.create_subscription(
                message_class,
                topic_name,
                lambda msg, bridge_key=key: self._on_export_topic_message(bridge_key, msg),
                10,
            )
            self._topic_exports[key] = TopicExportBridgeState(
                topic_name=topic_name,
                message_type=message_type,
                message_class=message_class,
                bridge_name=bridge_name,
                bridge_key=bridge_key,
                bridge_uuid=bridge_uuid,
                member_specs=member_specs,
                rate_hz=rate_hz,
                transport_endpoint=transport_endpoint,
                payload_format=payload_format,
                subscription=subscription,
            )
            existing_key = key
            self._configure_export_bridge_timer(self._topic_exports[key], existing_key)
        else:
            state = self._topic_exports[existing_key]
            if state.message_type != message_type:
                self.destroy_subscription(state.subscription)
                state.subscription = self.create_subscription(
                    message_class,
                    topic_name,
                    lambda msg, bridge_key=existing_key: self._on_export_topic_message(bridge_key, msg),
                    10,
                )
                state.message_type = message_type
                state.message_class = message_class
            state.bridge_key = bridge_key
            state.bridge_uuid = bridge_uuid
            state.member_specs = member_specs
            state.rate_hz = rate_hz
            state.transport_endpoint = transport_endpoint
            state.payload_format = payload_format
            self._configure_export_bridge_timer(state, existing_key)
        self._rebuild_server()
        state = self._topic_exports[existing_key]
        response.success = True
        response.message = "ok"
        response.resolved_uuid = state.bridge_uuid
        response.resolved_path = state.service.transport_path(state.transport_endpoint) if state.service is not None else ""
        response.resolved_topic = state.topic_name
        response.resolved_message_type = state.message_type
        response.resolved_member_paths = list(state.member_paths)
        response.resolved_rate_hz = float(state.rate_hz)
        response.resolved_transport_endpoint = state.transport_endpoint
        return response

    def _on_export_topic_message(self, bridge_key: str, message):
        state = self._topic_exports.get(bridge_key)
        if state is None or state.service is None:
            return
        try:
            payload = encode_message_payload(message, state.member_specs, state.payload_format)
        except Exception as exc:
            self.get_logger().warning(f"Failed to serialize message for BLE topic bridge {state.topic_name}: {exc}")
            return
        if state.rate_hz > 0:
            state.pending_payload = payload
            return
        state.service.publish(payload)

    def _configure_export_bridge_timer(self, state: TopicExportBridgeState, bridge_key: str):
        if state.publish_timer is not None:
            self.destroy_timer(state.publish_timer)
            state.publish_timer = None
        if state.rate_hz <= 0:
            return
        state.publish_timer = self.create_timer(1.0 / state.rate_hz, lambda key=bridge_key: self._flush_export_bridge(key))

    def _flush_export_bridge(self, bridge_key: str):
        state = self._topic_exports.get(bridge_key)
        if state is None or state.service is None or not state.pending_payload:
            return
        payload = state.pending_payload
        state.pending_payload = b""
        state.service.publish(payload)

    def _destroy_export_bridge(self, state: TopicExportBridgeState):
        if state.publish_timer is not None:
            self.destroy_timer(state.publish_timer)
            state.publish_timer = None

    def _configure_topic_import(self, request, response):
        topic_name = request.topic_name.strip()
        mac = request.mac.strip().upper()
        if not topic_name:
            response.success = False
            response.message = "topic_name is required"
            return response
        if not mac:
            response.success = False
            response.message = "mac is required for import bridges"
            return response
        bridge_name = (request.characteristic or topic_name).strip()
        transport_endpoint = self._normalize_transport_endpoint(request.transport_endpoint)
        message_type, message_class = self._resolve_message_type(topic_name, request.message_type, prefer_publishers=False)
        remote_member_specs, remote_payload_format, remote_rate_hz, remote_bridge_key = self._read_remote_bridge_metadata(mac, bridge_name, message_class=message_class)
        member_specs = normalize_member_specs(request.member_paths, message_class=message_class) or remote_member_specs
        rate_hz = max(0.0, float(request.rate_hz or remote_rate_hz or 0.0))
        payload_format = remote_payload_format or payload_format_for_member_specs(member_specs)
        path, bridge_uuid, transport_endpoint = self._resolve_remote_characteristic(mac, bridge_name, transport_endpoint)
        bridge_key = remote_bridge_key or hashlib.md5(bridge_name.encode("utf-8")).hexdigest()

        if not request.enable:
            removed = None
            removed_key = None
            for key, state in self._notification_bridges.items():
                if state.mac == mac and state.requested_topic_name == topic_name and state.bridge_name == bridge_name and not state.auto_managed:
                    removed = state
                    removed_key = key
                    break
            if removed is None:
                response.success = False
                response.message = "notification bridge not found"
                return response
            self._remove_import_bridge(removed_key, removed)
            response.success = True
            response.message = "removed"
            response.resolved_uuid = removed.bridge_uuid
            response.resolved_path = removed.path
            response.resolved_topic = removed.resolved_topic_name
            response.resolved_message_type = removed.message_type
            response.resolved_member_paths = list(removed.member_paths)
            response.resolved_rate_hz = float(removed.rate_hz)
            response.resolved_transport_endpoint = removed.transport_endpoint
            return response

        if not path:
            response.success = False
            response.message = f"remote bridge {bridge_name} not found on {mac}"
            return response

        resolved_topic_name = self._resolve_peer_topic_name(mac, topic_name)
        key = f"manual-import::{mac}::{transport_endpoint}::{bridge_name}::{topic_name}"
        state = self._notification_bridges.get(key)
        if state is None:
            if transport_endpoint == "characteristic" and not self._client.start_notify(path):
                response.success = False
                response.message = f"failed to enable notifications for {path}"
                return response
            publisher = self.create_publisher(message_class, resolved_topic_name, 10)
            self._notification_bridges[key] = TopicImportBridgeState(
                mac=mac,
                requested_topic_name=topic_name,
                resolved_topic_name=resolved_topic_name,
                message_type=message_type,
                message_class=message_class,
                bridge_name=bridge_name,
                bridge_key=bridge_key,
                bridge_uuid=bridge_uuid,
                member_specs=member_specs,
                rate_hz=rate_hz,
                transport_endpoint=transport_endpoint,
                payload_format=payload_format,
                path=path,
                publisher=publisher,
            )
            state = self._notification_bridges[key]
            self._configure_import_bridge_timer(state, key)
        else:
            old_path = state.path
            old_transport_endpoint = state.transport_endpoint
            if transport_endpoint == "characteristic":
                needs_notify = old_transport_endpoint != "characteristic" or old_path != path
                if needs_notify and not self._client.start_notify(path):
                    response.success = False
                    response.message = f"failed to enable notifications for {path}"
                    return response
            if state.message_type != message_type:
                replacement_publisher = self.create_publisher(message_class, resolved_topic_name, 10)
                self.destroy_publisher(state.publisher)
                state.publisher = replacement_publisher
                state.message_type = message_type
                state.message_class = message_class
            elif state.resolved_topic_name != resolved_topic_name:
                replacement_publisher = self.create_publisher(message_class, resolved_topic_name, 10)
                self.destroy_publisher(state.publisher)
                state.publisher = replacement_publisher
            state.bridge_name = bridge_name
            state.bridge_key = bridge_key
            state.bridge_uuid = bridge_uuid
            state.requested_topic_name = topic_name
            state.resolved_topic_name = resolved_topic_name
            state.member_specs = member_specs
            state.rate_hz = rate_hz
            state.transport_endpoint = transport_endpoint
            state.payload_format = payload_format
            state.path = path
            self._configure_import_bridge_timer(state, key)
            if old_transport_endpoint == "characteristic" and (transport_endpoint != "characteristic" or old_path != path):
                self._stop_notify_if_unused(old_path)

        state = self._notification_bridges[key]
        response.success = True
        response.message = "ok"
        response.resolved_uuid = state.bridge_uuid
        response.resolved_path = state.path
        response.resolved_topic = state.resolved_topic_name
        response.resolved_message_type = state.message_type
        response.resolved_member_paths = list(state.member_paths)
        response.resolved_rate_hz = float(state.rate_hz)
        response.resolved_transport_endpoint = state.transport_endpoint
        return response

    def _configure_import_bridge_timer(self, state: TopicImportBridgeState, bridge_key: str):
        if state.poll_timer is not None:
            self.destroy_timer(state.poll_timer)
            state.poll_timer = None
        if state.transport_endpoint == "descriptor":
            period = 1.0 / state.rate_hz if state.rate_hz > 0 else 1.0
            state.poll_timer = self.create_timer(period, lambda key=bridge_key: self._poll_import_bridge(key))
            return
        if state.rate_hz <= 0:
            return
        state.poll_timer = self.create_timer(1.0 / state.rate_hz, lambda key=bridge_key: self._flush_import_bridge(key))

    def _destroy_import_bridge(self, state: TopicImportBridgeState):
        if state.poll_timer is not None:
            self.destroy_timer(state.poll_timer)
            state.poll_timer = None

    def _flush_import_bridge(self, bridge_key: str):
        state = self._notification_bridges.get(bridge_key)
        if state is None or not state.pending_payload:
            return
        payload = state.pending_payload
        state.pending_payload = b""
        self._publish_import_payload(state, payload)

    def _poll_import_bridge(self, bridge_key: str):
        state = self._notification_bridges.get(bridge_key)
        if state is None or state.transport_endpoint != "descriptor":
            return
        payload = self._client.read_descriptor(state.path)
        if payload is None or payload == state.last_payload:
            return
        self._publish_import_payload(state, payload)

    def _publish_import_payload(self, state: TopicImportBridgeState, payload: bytes):
        try:
            message = decode_message_payload(payload, state.message_class, state.member_specs, state.payload_format)
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode BLE payload for topic bridge {state.resolved_topic_name}: {exc}")
            return
        state.last_payload = bytes(payload)
        state.last_publish_monotonic, state.current_hz = self._update_publish_rate(state.last_publish_monotonic)
        state.publisher.publish(message)

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

    def _handle_reload_config(self, request, response):
        del request
        try:
            self._reload_active_config()
            response.success = True
            response.message = f"Reloaded config from {self._active_config_source}"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_set_active_config(self, request, response):
        path = str(request.config_path or "").strip()
        hold_seconds = max(0.5, float(request.hold_seconds or 3.0))
        try:
            if not path:
                if self._active_overlay_path:
                    self._active_overlay_path = ""
                    self._active_overlay_deadline = 0.0
                    self._reload_active_config()
                response.success = True
                response.message = "Reverted to default config"
                response.active_config_path = self._active_config_source
                response.overlay_active = False
                return response
            if not os.path.isfile(path):
                raise FileNotFoundError(path)
            new_deadline = time.monotonic() + hold_seconds
            if path == self._active_overlay_path:
                # Only extend the lease deadline — do not reload the full config
                self._active_overlay_deadline = new_deadline
                response.success = True
                response.message = f"Lease extended for overlay config {path}"
                response.active_config_path = self._active_config_source
                response.overlay_active = True
                return response
            self._active_overlay_path = path
            self._active_overlay_deadline = new_deadline
            self._reload_active_config()
            response.success = True
            response.message = f"Activated overlay config {path}"
            response.active_config_path = self._active_config_source
            response.overlay_active = True
            return response
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.active_config_path = self._active_config_source
            response.overlay_active = bool(self._active_overlay_path)
            return response

    def _check_overlay_config_lease(self):
        if not self._active_overlay_path:
            return
        if time.monotonic() <= self._active_overlay_deadline:
            return
        expired_path = self._active_overlay_path
        self._active_overlay_path = ""
        self._active_overlay_deadline = 0.0
        self._reload_active_config()
        self.get_logger().info(f"Overlay config lease expired, reverted to default after {expired_path}")

    def _reload_active_config(self):
        config = self._load_effective_config(self._active_overlay_path)
        source_path = self._active_overlay_path or self._default_config_path
        self._apply_config_document(config, source_path=source_path, initial=False)
        self._reconfigure_timers()
        self._apply_runtime_side_effects(initial=False)

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
            if self._client.scanning:
                self._client.stop_scan()
            for device in self._client.get_devices(refresh=True).values():
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


def main(args: Optional[Sequence[str]] = None):
    rclpy.init(args=args)
    node = BluetoothNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
