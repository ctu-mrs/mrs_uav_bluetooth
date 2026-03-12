#!/usr/bin/env python3
"""Experiment-side helper that applies a temporary bluetooth overlay config."""

import os
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from mrs_uav_bluetooth.srv import SetActiveConfig
from mrs_uav_bluetooth_pkg.uuid_utils import sanitize_topic_suffix, system_hostname


class BluetoothUserNode(Node):
    def __init__(self):
        super().__init__("mrs_uav_bluetooth_user")
        self.declare_parameter("config_path", "")
        self.declare_parameter("hold_seconds", 3.0)
        self.declare_parameter("min_hold_seconds", 0.5)
        self.declare_parameter("print_source", "status")
        self.declare_parameter("service_wait_timeout_sec", 10.0)
        self.declare_parameter("service_call_timeout_sec", 5.0)
        self.declare_parameter("deactivate_service_wait_timeout_sec", 2.0)
        self.declare_parameter("sentinel_topic_suffix", "overlay_keepalive")
        self.declare_parameter("sentinel_publish_period_sec", 1.0)
        self.declare_parameter("min_sentinel_publish_period_sec", 0.2)

        self._config_path = str(self.get_parameter("config_path").value or "").strip()
        self._min_hold_seconds = max(0.0, float(self.get_parameter("min_hold_seconds").value or 0.0))
        self._hold_seconds = max(self._min_hold_seconds, float(self.get_parameter("hold_seconds").value or 0.0))
        self._service_wait_timeout_sec = max(0.0, float(self.get_parameter("service_wait_timeout_sec").value or 0.0))
        self._service_call_timeout_sec = max(0.0, float(self.get_parameter("service_call_timeout_sec").value or 0.0))
        self._deactivate_service_wait_timeout_sec = max(
            0.0,
            float(self.get_parameter("deactivate_service_wait_timeout_sec").value or 0.0),
        )
        if not self._config_path:
            raise RuntimeError("config_path parameter is required")
        if not os.path.isfile(self._config_path):
            raise FileNotFoundError(self._config_path)

        hostname = sanitize_topic_suffix(system_hostname() or "mrs-uav")
        source = str(self.get_parameter("print_source").value or "status").strip().lower()
        if source not in {"status", "log"}:
            raise ValueError("print_source must be 'status' or 'log'")
        self._print_source = source
        self._print_topic = f"/{hostname}/ble/{self._print_source}"
        sentinel_suffix = str(self.get_parameter("sentinel_topic_suffix").value or "overlay_keepalive").strip().strip("/")
        if not sentinel_suffix:
            raise ValueError("sentinel_topic_suffix must not be empty")
        self._sentinel_topic = f"/{hostname}/ble/{sentinel_suffix}"
        min_sentinel_period = max(0.0, float(self.get_parameter("min_sentinel_publish_period_sec").value or 0.0))
        requested_sentinel_period = float(self.get_parameter("sentinel_publish_period_sec").value or 0.0)
        self._sentinel_publish_period_sec = max(min_sentinel_period, requested_sentinel_period)
        if self._sentinel_publish_period_sec <= 0.0:
            raise ValueError("sentinel_publish_period_sec must be > 0")
        self._client = self.create_client(SetActiveConfig, "ble/set_active_config")
        self.create_subscription(String, self._print_topic, self._handle_print, 200)
        self._sentinel_pub = self.create_publisher(Empty, self._sentinel_topic, 10)
        self._sentinel_timer = self.create_timer(self._sentinel_publish_period_sec, self._publish_sentinel)

    def activate(self):
        if not self._client.wait_for_service(timeout_sec=self._service_wait_timeout_sec):
            raise RuntimeError("ble/set_active_config service is not available")
        response = self._call_config_service(self._config_path, self._hold_seconds)
        if response is None or not response.success:
            raise RuntimeError(response.message if response is not None else "No response from ble/set_active_config")
        self.get_logger().info(f"Activated BLE overlay config: {response.active_config_path}")
        self.get_logger().info(f"Printing bluetooth service {self._print_source} topic: {self._print_topic}")
        self.get_logger().info(f"Publishing overlay keep-alive sentinel on: {self._sentinel_topic}")

    def deactivate(self):
        try:
            if not self.context.ok():
                return
            if not self._client.service_is_ready() and not self._client.wait_for_service(
                timeout_sec=self._deactivate_service_wait_timeout_sec
            ):
                return
            response = self._call_config_service("", 0.0)
            if response is not None and response.success:
                self.get_logger().info("Reverted bluetooth service to default config")
        except Exception:
            pass

    def _publish_sentinel(self):
        msg = Empty()
        self._sentinel_pub.publish(msg)

    def _call_config_service(self, config_path: str, hold_seconds: float):
        request = SetActiveConfig.Request()
        request.config_path = config_path
        request.hold_seconds = float(hold_seconds)
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_call_timeout_sec)
        if not future.done():
            return None
        return future.result()

    def _handle_print(self, message: String):
        self.get_logger().info(message.data)


def main(args: Optional[Sequence[str]] = None):
    rclpy.init(args=args)
    node = None
    try:
        node = BluetoothUserNode()
        node.activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.deactivate()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
