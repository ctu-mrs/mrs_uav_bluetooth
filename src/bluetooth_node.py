"""ROS 2 BLE node providing MRS UAV Bluetooth server and client control."""

import logging
import threading
from typing import Dict, Optional, Sequence, Set, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .bluetooth_bridge_state import PeerRuntimeStatus, PeerTimeBridgeState, TopicExportBridgeState, TopicImportBridgeState
from .bluetooth_dbus_runtime import BluetoothDbusRuntime
from .bluetooth_node_config_mixin import BluetoothNodeConfigMixin, SharedTopicConfig
from .bluetooth_node_runtime_mixin import BluetoothNodeRuntimeMixin
from .bluetooth_node_service_mixin import BluetoothNodeServiceMixin
from .bluetooth_node_status_mixin import BluetoothNodeStatusMixin
from .dbus_client import BleClient
from .uuid_utils import system_hostname


class BluetoothNode(
    BluetoothNodeRuntimeMixin,
    BluetoothNodeServiceMixin,
    BluetoothNodeStatusMixin,
    BluetoothNodeConfigMixin,
    Node,
):
    def __init__(self):
        super().__init__("mrs_uav_bluetooth")
        self._declare_parameters()
        self._lock = threading.RLock()
        self._local_name = system_hostname() or "mrs-uav"
        self._pending_wifi_password = ""
        self._auto_connect_attempts: Dict[str, float] = {}
        self._peer_setup_started_at: Dict[str, float] = {}
        self._peer_pair_requested_at: Dict[str, float] = {}
        self._peer_repair_reasons: Dict[str, str] = {}
        self._notification_path_to_mac: Dict[str, str] = {}
        self._last_gatt_warning_at: Dict[Tuple[str, str], float] = {}
        self._last_dbus_warning_at: Dict[str, float] = {}
        self._scan_transport = self.get_parameter("scan_mode").get_parameter_value().string_value
        self._topic_exports: Dict[str, TopicExportBridgeState] = {}
        self._notification_bridges: Dict[str, TopicImportBridgeState] = {}
        self._peer_time_bridges: Dict[str, PeerTimeBridgeState] = {}
        self._peer_status: Dict[str, PeerRuntimeStatus] = {}
        self._shared_topic_configs: Dict[str, SharedTopicConfig] = {}
        self._active_overlay_path = ""
        self._overlay_connected_baseline: Set[str] = set()
        self._overlay_keepalive_topic = ""
        self._default_config_path = self._resolve_default_config_path()
        self._active_config_source = self._default_config_path
        self._node_topics_prefix = self._format_node_topics_prefix("/{hostname}/ble")
        self._shutting_down = False
        self._peer_writeback_lock = threading.RLock()
        self._peer_writeback_pending: Dict[str, bytes] = {}
        self._peer_writeback_inflight: Set[str] = set()
        self._peer_writeback_last_sent: Dict[str, bytes] = {}
        self._peer_writeback_retry_at: Dict[str, float] = {}
        self._pairing_repair_attempts: Dict[str, float] = {}

        self.devices_pub = None
        self.notifications_pub = None
        self.status_pub = None
        self.log_pub = None
        self._verbose_logger = None  # type: Optional[logging.Logger]
        self._netplan = None
        self._scan_results_timer = None
        self._time_service_timer = None
        self._wifi_service_timer = None
        self._auto_connect_timer = None
        self._status_timer = None

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


def main(args: Optional[Sequence[str]] = None):
    rclpy.init(args=args)
    node = BluetoothNode()
    executor = MultiThreadedExecutor(num_threads=16)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
