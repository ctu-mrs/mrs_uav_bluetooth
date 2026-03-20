"""ROS service handlers and bridge control mixin for the Bluetooth node."""

import dbus
from rosidl_runtime_py.utilities import get_message

from .bluetooth_bridge_state import TopicExportBridgeState, TopicImportBridgeState
from .bridge_payload import decode_message_payload, encode_message_payload, normalize_member_specs, payload_format_for_member_specs
from .dbus_common import BLUEZ_SERVICE_NAME, DBUS_PROP_IFACE, GATT_CHRC_IFACE
from .gatt_services import topic_bridge_characteristic_uuid
from .uuid_utils import is_uuid, resolve_uuid


class BluetoothNodeServiceMixin:

    def _log_service_gatt_action(self, action: str, path: str, *, descriptor: bool = False, payload: bytes = None, success: bool = None, extra: str = ""):
        target = "descriptor" if descriptor else "characteristic"
        parts = [f"Service {action}", f"target={target}", f"path={path}"]
        if success is not None:
            parts.append(f"success={'Y' if success else 'N'}")
        if payload is not None:
            parts.append(self._format_payload_preview(payload))
        if extra:
            parts.append(extra)
        self._log_verbose(" ".join(parts))

    def _resolve_message_type(self, topic_name: str, explicit_message_type: str, prefer_publishers: bool):
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

    def _resolve_remote_characteristic(self, mac: str, identifier: str, transport_endpoint: str):
        if transport_endpoint != "characteristic":
            raise ValueError("Only characteristic transport is supported")
        candidate = identifier.strip()
        if candidate.startswith("/org/"):
            props = dbus.Interface(self._bus.get_object(BLUEZ_SERVICE_NAME, candidate), DBUS_PROP_IFACE)
            try:
                uuid = str(props.Get(GATT_CHRC_IFACE, "UUID"))
                return candidate, uuid, "characteristic"
            except Exception:
                return candidate, "", transport_endpoint
        resolved_uuid = resolve_uuid(candidate) if is_uuid(candidate) else topic_bridge_characteristic_uuid(candidate)
        path = self._client.find_characteristic(mac, resolved_uuid)
        return path or "", resolved_uuid, transport_endpoint

    def _reject_descriptor_request(self, response, message: str):
        response.success = False
        response.message = message
        return response

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
        del request
        response.success = False
        response.message = "Descriptor operations are deprecated"
        response.descriptors = []
        return response

    def _handle_find_gatt_path(self, request, response):
        resolved_name = resolve_uuid(request.uuid)
        if request.descriptor:
            return self._reject_descriptor_request(response, "Descriptor operations are deprecated")
        path = self._client.find_characteristic(request.mac, resolved_name)
        response.success = bool(path)
        response.message = "ok" if path else f"uuid {resolved_name} not found"
        response.path = path or ""
        return response

    def _handle_read_gatt_value(self, request, response):
        if request.descriptor:
            return self._reject_descriptor_request(response, "Descriptor read is deprecated")
        self._log_service_gatt_action("read-request", request.path, descriptor=request.descriptor)
        data = self._client.read_characteristic(request.path)
        response.success = data is not None
        response.message = "ok" if data is not None else f"read failed for {request.path}"
        response.value = list(data or b"")
        self._log_service_gatt_action(
            "read-result",
            request.path,
            descriptor=request.descriptor,
            payload=data,
            success=response.success,
        )
        return response

    def _handle_write_gatt_value(self, request, response):
        if request.descriptor:
            return self._reject_descriptor_request(response, "Descriptor write is deprecated")
        payload = bytes(request.value)
        self._log_service_gatt_action(
            "write-request",
            request.path,
            descriptor=request.descriptor,
            payload=payload,
            extra=f"with_response={'Y' if request.with_response else 'N'}" if not request.descriptor else "",
        )
        success = self._client.write_characteristic(request.path, payload, with_response=request.with_response)
        response.success = bool(success)
        response.message = "ok" if success else f"write failed for {request.path}"
        self._log_service_gatt_action(
            "write-result",
            request.path,
            descriptor=request.descriptor,
            payload=payload,
            success=response.success,
        )
        return response

    def _handle_set_notify(self, request, response):
        self._log_service_gatt_action(
            "notify-request",
            request.path,
            success=request.enable,
            extra=f"enable={'Y' if request.enable else 'N'}",
        )
        success = self._client.start_notify(request.path) if request.enable else self._client.stop_notify(request.path)
        response.success = bool(success)
        response.message = "ok" if success else f"notify change failed for {request.path}"
        self._log_service_gatt_action(
            "notify-result",
            request.path,
            success=response.success,
            extra=f"enable={'Y' if request.enable else 'N'}",
        )
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
        if not member_specs:
            raise ValueError("member_paths must define at least one field")
        payload_format = payload_format_for_member_specs(member_specs)
        bridge_key = bridge_name
        bridge_uuid = topic_bridge_characteristic_uuid(bridge_name)
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
        response.resolved_path = state.service.characteristic_path() if state.service is not None else ""
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
        member_specs = normalize_member_specs(request.member_paths, message_class=message_class)
        if not member_specs:
            raise ValueError("member_paths must define at least one field")
        rate_hz = max(0.0, float(request.rate_hz or 0.0))
        payload_format = payload_format_for_member_specs(member_specs)
        path, bridge_uuid, transport_endpoint = self._resolve_remote_characteristic(mac, bridge_name, transport_endpoint)
        bridge_key = bridge_name

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
            if transport_endpoint == "characteristic":
                started, path = self._start_notify_with_refresh(
                    mac,
                    path,
                    bridge_uuid,
                    label=f"manual import bridge {bridge_name}",
                )
                if not started:
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
                if needs_notify:
                    started, path = self._start_notify_with_refresh(
                        mac,
                        path,
                        bridge_uuid,
                        label=f"manual import bridge {bridge_name}",
                    )
                    if not started:
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

    def _publish_import_payload(self, state: TopicImportBridgeState, payload: bytes):
        try:
            message = decode_message_payload(payload, state.message_class, state.member_specs, state.payload_format)
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode BLE payload for topic bridge {state.resolved_topic_name}: {exc}")
            return
        header = getattr(message, "header", None)
        if header is not None and hasattr(header, "frame_id"):
            header.frame_id = self._peer_frame_id(state.mac)
        state.last_payload = bytes(payload)
        state.last_publish_monotonic, state.current_hz = self._update_publish_rate(state.last_publish_monotonic)
        state.publisher.publish(message)
