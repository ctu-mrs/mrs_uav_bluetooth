"""Configuration and runtime utility mixin for the Bluetooth node."""

import hashlib
import logging
import os
import re
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import dbus
import rclpy
import rclpy.exceptions
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from .bridge_payload import BridgeMemberSpec, normalize_member_specs, payload_format_for_member_specs
from .dbus_client import DeviceInfo
from .uuid_utils import is_uav_hostname, sanitize_topic_suffix


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


class BluetoothNodeConfigMixin:

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
        "log_topic_enable",
        "expire_connections_with_overlay",
        "verbose_log_file",
        "overlay_keepalive_topic_suffix",
    ]

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
        self.declare_parameter("log_topic_enable", False)
        self.declare_parameter("expire_connections_with_overlay", True)
        self.declare_parameter("verbose_log_file", "")
        self.declare_parameter("overlay_keepalive_topic_suffix", "overlay_keepalive")

    def _resolve_default_config_path(self) -> str:
        configured = str(self.get_parameter("default_config_path").value or "").strip()
        if configured:
            return configured
        return os.path.join(get_package_share_directory("mrs_uav_bluetooth"), "config", "default.yaml")

    def _local_frame_id(self) -> str:
        return sanitize_topic_suffix(self._local_name or "mrs-uav")

    def _peer_frame_id(self, mac: str) -> str:
        state = self._peer_time_bridges.get(mac)
        if state is not None and state.peer_name:
            return sanitize_topic_suffix(state.peer_name)
        if self._client is not None:
            device = self._client.get_device(mac)
            if device is not None:
                peer_name = self._hostname_from_device(device)
                if peer_name:
                    return sanitize_topic_suffix(peer_name)
        return sanitize_topic_suffix(mac.lower().replace(":", "_"))

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

    def _resolve_peer_topic_name(self, mac: str, topic_name: str, device: DeviceInfo = None) -> str:
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
        try:
            self.get_logger().info(f"[VERBOSE] {message}")
        except Exception:
            pass

        verbose_logger = self._verbose_logger
        if verbose_logger is not None:
            verbose_logger.debug(message)

        if not bool(self.get_parameter("log_topic_enable").value):
            return

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

    def _recreate_timer(self, attr_name: str, period: float, callback):
        self._destroy_timer_attr(attr_name)
        setattr(self, attr_name, self.create_timer(period, callback))

    def _destroy_timer_attr(self, attr_name: str):
        timer = getattr(self, attr_name, None)
        if timer is not None:
            self.destroy_timer(timer)
            setattr(self, attr_name, None)

    def _normalize_transport_endpoint(self, endpoint: str) -> str:
        candidate = endpoint.strip().lower() or "characteristic"
        if candidate not in {"characteristic", "descriptor"}:
            raise ValueError("transport_endpoint must be 'characteristic' or 'descriptor'")
        return candidate

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

    def _overlay_keepalive_active(self) -> bool:
        if not self._overlay_keepalive_topic:
            return False
        try:
            return bool(self.get_publishers_info_by_topic(self._overlay_keepalive_topic))
        except Exception as exc:
            self._dbus_warning("overlay_keepalive_graph", f"Failed to inspect overlay keepalive topic publishers: {exc}")
            return False

    def _handle_set_active_config(self, request, response):
        path = str(request.config_path or "").strip()
        try:
            if not path:
                if self._active_overlay_path:
                    self._expire_overlay_connections("explicit revert")
                    self._active_overlay_path = ""
                    self._reload_active_config()
                    self._enforce_peer_connection_policy(reason="overlay revert")
                response.success = True
                response.message = "Reverted to default config"
                response.active_config_path = self._active_config_source
                response.overlay_active = False
                return response
            if not os.path.isfile(path):
                raise FileNotFoundError(path)
            if path == self._active_overlay_path:
                response.success = True
                response.message = f"Overlay config already active: {path}"
                response.active_config_path = self._active_config_source
                response.overlay_active = True
                return response
            if self._active_overlay_path:
                # Switching overlays is a lease boundary for overlay-introduced peers.
                self._expire_overlay_connections("overlay config changed")
            self._capture_overlay_connection_baseline()
            self._active_overlay_path = path
            self._reload_active_config()
            self._enforce_peer_connection_policy(reason="overlay config changed")
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
        if self._overlay_keepalive_active():
            return
        expired_path = self._active_overlay_path
        self._expire_overlay_connections("keepalive sentinel missing")
        self._active_overlay_path = ""
        self._reload_active_config()
        self._enforce_peer_connection_policy(reason="overlay keepalive expired")
        self.get_logger().info(f"Overlay keepalive missing, reverted to default after {expired_path}")

    def _reload_active_config(self):
        config = self._load_effective_config(self._active_overlay_path)
        source_path = self._active_overlay_path or self._default_config_path
        self._apply_config_document(config, source_path=source_path, initial=False)
        self._reconfigure_timers()
        self._apply_runtime_side_effects(initial=False)

    def _capture_overlay_connection_baseline(self):
        if self._overlay_connected_baseline:
            return
        self._overlay_connected_baseline = self._get_connected_peer_macs()

    def _expire_overlay_connections(self, reason: str):
        if not bool(self.get_parameter("expire_connections_with_overlay").value):
            self._overlay_connected_baseline.clear()
            return
        if self._client is None:
            self._overlay_connected_baseline.clear()
            return
        current_connected = self._get_connected_peer_macs()
        if not current_connected:
            self._overlay_connected_baseline.clear()
            return
        to_disconnect = current_connected - self._overlay_connected_baseline
        for mac in sorted(to_disconnect):
            self.get_logger().info(f"Disconnecting {mac}: overlay lease expired ({reason})")
            self._client.disconnect(mac, timeout=5.0)
        self._overlay_connected_baseline.clear()

    def _get_connected_peer_macs(self):
        if self._client is None:
            return set()
        try:
            return {mac for mac, device in self._client.get_devices(refresh=True).items() if device.connected}
        except dbus.exceptions.DBusException as exc:
            self._dbus_warning("get_connected_peer_macs", f"Unable to list connected peers due to DBus error: {exc}")
            return set()
