"""Status reporting and message conversion mixin for the Bluetooth node."""

import time

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import String
from std_msgs.msg import Header

from mrs_uav_bluetooth.msg import (
    BleDevice,
    BleGattCharacteristic,
    BleGattDescriptor,
    BleGattService,
    BlePeerTimeStatus,
)

from .uuid_utils import sanitize_topic_suffix


class BluetoothNodeStatusMixin:

    def _publish_status_report(self):
        try:
            lines = self._build_status_lines()
        except Exception as exc:
            self.get_logger().warning(f"Status report failed: {exc}")
            return
        report = "\n".join(lines)
        self.get_logger().info(f"[STATUS]\n{report}")
        if self.status_pub is not None and not self._shutting_down and self._ros_context_ok():
            status_msg = String()
            status_msg.data = report
            try:
                self.status_pub.publish(status_msg)
            except Exception:
                if not self._shutting_down and self._ros_context_ok():
                    raise
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
        if self._app is not None:
            local_services = list(getattr(self._app, "services", []))
            local_chrc_count = sum(len(service.get_characteristics()) for service in local_services)
            local_desc_count = sum(
                len(characteristic.get_descriptors())
                for service in local_services
                for characteristic in service.get_characteristics()
            )
            lines.append(
                f"  local gatt: services={len(local_services)} characteristics={local_chrc_count} descriptors={local_desc_count}"
            )
            for service in local_services:
                lines.append(f"    service {service.uuid} [{service.get_path()}]")
                for characteristic in service.get_characteristics():
                    lines.append(f"      chrc {characteristic.uuid} [{characteristic.get_path()}]")
                    for descriptor in characteristic.get_descriptors():
                        lines.append(f"        desc {descriptor.uuid} [{descriptor.get_path()}]")
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

    def _header(self, frame_id=None) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = sanitize_topic_suffix(frame_id or self._local_frame_id())
        return header

    def _time_msg(self, stamp: float) -> TimeMsg:
        secs = int(stamp)
        nanosec = int((stamp - secs) * 1e9)
        return TimeMsg(sec=secs, nanosec=nanosec)

    def _device_to_msg(self, device) -> BleDevice:
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

    def _publish_peer_time_status(self, state):
        message = BlePeerTimeStatus()
        message.header = self._header(frame_id=self._local_frame_id())
        message.mac = state.mac
        message.peer_name = state.peer_name
        ns = max(0, int(state.last_time_value_ns))
        message.peer_stamp = TimeMsg(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))
        message.last_rtt_s = max(0.0, state.last_rtt_s)
        state.publisher.publish(message)
