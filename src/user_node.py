#!/usr/bin/env python3
"""Experiment-side helper that applies a temporary bluetooth overlay config."""

import os
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mrs_uav_bluetooth.srv import SetActiveConfig
from mrs_uav_bluetooth_pkg.uuid_utils import sanitize_topic_suffix, system_hostname


class BluetoothUserNode(Node):
    def __init__(self):
        super().__init__("mrs_uav_bluetooth_user")
        self.declare_parameter("config_path", "")
        self.declare_parameter("hold_seconds", 3.0)
        self.declare_parameter("print_source", "status")

        self._config_path = str(self.get_parameter("config_path").value or "").strip()
        self._hold_seconds = max(0.5, float(self.get_parameter("hold_seconds").value or 3.0))
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
        self._client = self.create_client(SetActiveConfig, "ble/set_active_config")
        self._pending_future = None
        self.create_subscription(String, self._print_topic, self._handle_print, 200)
        self._lease_timer = self.create_timer(max(0.5, self._hold_seconds / 2.0), self._refresh_lease)

    def activate(self):
        if not self._client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("ble/set_active_config service is not available")
        response = self._call_config_service(self._config_path, self._hold_seconds)
        if response is None or not response.success:
            raise RuntimeError(response.message if response is not None else "No response from ble/set_active_config")
        self.get_logger().info(f"Activated BLE overlay config: {response.active_config_path}")
        self.get_logger().info(f"Printing bluetooth service {self._print_source} topic: {self._print_topic}")

    def deactivate(self):
        try:
            if not self.context.ok():
                return
            if not self._client.service_is_ready() and not self._client.wait_for_service(timeout_sec=2.0):
                return
            response = self._call_config_service("", 0.0)
            if response is not None and response.success:
                self.get_logger().info("Reverted bluetooth service to default config")
        except Exception:
            pass

    def _refresh_lease(self):
        if self._pending_future is not None and not self._pending_future.done():
            return
        request = SetActiveConfig.Request()
        request.config_path = self._config_path
        request.hold_seconds = float(self._hold_seconds)
        self._pending_future = self._client.call_async(request)
        self._pending_future.add_done_callback(self._handle_lease_response)

    def _handle_lease_response(self, future):
        self._pending_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to refresh bluetooth config lease: {exc}")
            return
        if not response.success:
            self.get_logger().warning(f"Bluetooth config lease rejected: {response.message}")

    def _call_config_service(self, config_path: str, hold_seconds: float):
        request = SetActiveConfig.Request()
        request.config_path = config_path
        request.hold_seconds = float(hold_seconds)
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
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
