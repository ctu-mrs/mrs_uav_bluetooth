"""Server-side BLE GATT services for the ROS 2 Bluetooth node."""

import struct
import time
from typing import Callable, Optional, Sequence

from .bridge_payload import BridgeMemberSpec, member_specs_to_serializable, serializable_to_bytes
from .dbus_gatt import Service
from .gatt_server import NotifyingCharacteristic, ReadOnlyDescriptor, WritableDescriptor
from .uuid_utils import named_characteristic_uuid, named_descriptor_uuid, named_service_uuid

WIFI_SERVICE_NAME = "wifi"
WIFI_SSID_NAME = "wifi/ssid"
WIFI_SSID_DESCRIPTOR_NAME = "wifi/ssid/config"
WIFI_PASSWORD_DESCRIPTOR_NAME = "wifi/password/config"
TIME_SERVICE_NAME = "time"
TIME_CHARACTERISTIC_NAME = "time/ns"
TIME_DESCRIPTOR_NAME = "time/ns/value"
TIME_WRITEBACK_DESCRIPTOR_NAME = "time/ns/writeback"

WIFI_SERVICE_UUID = named_service_uuid(WIFI_SERVICE_NAME)
WIFI_CHARACTERISTIC_UUID = named_characteristic_uuid(WIFI_SSID_NAME)
WIFI_DESCRIPTOR_UUID = named_descriptor_uuid(WIFI_SSID_DESCRIPTOR_NAME)
WIFI_PASSWORD_DESCRIPTOR_UUID = named_descriptor_uuid(WIFI_PASSWORD_DESCRIPTOR_NAME)
TIME_SERVICE_UUID = named_service_uuid(TIME_SERVICE_NAME)
TIME_CHARACTERISTIC_UUID = named_characteristic_uuid(TIME_CHARACTERISTIC_NAME)
TIME_DESCRIPTOR_UUID = named_descriptor_uuid(TIME_DESCRIPTOR_NAME)
TIME_WRITEBACK_DESCRIPTOR_UUID = named_descriptor_uuid(TIME_WRITEBACK_DESCRIPTOR_NAME)


def topic_bridge_characteristic_uuid(bridge_name: str) -> str:
    return named_characteristic_uuid(bridge_name)


def topic_bridge_data_descriptor_uuid(bridge_name: str) -> str:
    return named_descriptor_uuid(f"{bridge_name}/data")


def topic_bridge_metadata_descriptor_uuid(bridge_name: str, key: str) -> str:
    return named_descriptor_uuid(f"{bridge_name}/{key}")


class WifiService(Service):
    def __init__(
        self,
        bus,
        index,
        wifi_name_cb: Callable[[], str],
        wifi_apply_cb: Callable[[str], None],
        wifi_password_write_cb: Callable[[str], None],
        wifi_password_read_cb: Callable[[], str],
    ):
        super().__init__(bus, index, WIFI_SERVICE_UUID, primary=True, name=WIFI_SERVICE_NAME)
        self._characteristic = NotifyingCharacteristic(
            bus,
            0,
            WIFI_CHARACTERISTIC_UUID,
            ["read", "write", "write-without-response", "notify"],
            self,
            read_cb=lambda: wifi_name_cb().encode("utf-8"),
            write_cb=lambda data: wifi_apply_cb(data.decode("utf-8").strip()),
            initial_value=wifi_name_cb().encode("utf-8"),
            name=WIFI_SSID_NAME,
        )
        self._descriptor = WritableDescriptor(
            bus,
            0,
            WIFI_DESCRIPTOR_UUID,
            ["read", "write"],
            self._characteristic,
            read_cb=lambda: wifi_name_cb().encode("utf-8"),
            write_cb=lambda data, options: wifi_apply_cb(data.decode("utf-8").strip()),
            name=WIFI_SSID_DESCRIPTOR_NAME,
        )
        self._password_descriptor = WritableDescriptor(
            bus,
            1,
            WIFI_PASSWORD_DESCRIPTOR_UUID,
            ["read", "write"],
            self._characteristic,
            read_cb=lambda: wifi_password_read_cb().encode("utf-8"),
            write_cb=lambda data, options: wifi_password_write_cb(data.decode("utf-8")),
            name=WIFI_PASSWORD_DESCRIPTOR_NAME,
        )
        self._characteristic.add_descriptor(self._descriptor)
        self._characteristic.add_descriptor(self._password_descriptor)
        self.add_characteristic(self._characteristic)

    def update(self, ssid: str):
        payload = ssid.encode("utf-8")
        self._descriptor.set_value(payload, emit=True)
        self._characteristic.publish(payload)


class TimeService(Service):
    def __init__(self, bus, index, *, writeback_cb: Optional[Callable[[bytes, dict, int], None]] = None):
        super().__init__(bus, index, TIME_SERVICE_UUID, primary=True, name=TIME_SERVICE_NAME)
        self._last_writeback = b"\x00" * 8
        self._writeback_cb = writeback_cb
        self._characteristic = NotifyingCharacteristic(
            bus,
            0,
            TIME_CHARACTERISTIC_UUID,
            ["read", "notify"],
            self,
            read_cb=self._read_time,
            initial_value=self._read_time(),
            name=TIME_CHARACTERISTIC_NAME,
        )
        self._descriptor = ReadOnlyDescriptor(
            bus,
            0,
            TIME_DESCRIPTOR_UUID,
            self._characteristic,
            read_cb=self._read_time,
            name=TIME_DESCRIPTOR_NAME,
        )
        self._writeback_descriptor = WritableDescriptor(
            bus,
            1,
            TIME_WRITEBACK_DESCRIPTOR_UUID,
            ["read", "write"],
            self._characteristic,
            read_cb=lambda: self._last_writeback,
            write_cb=self._write_writeback,
            name=TIME_WRITEBACK_DESCRIPTOR_NAME,
        )
        self._characteristic.add_descriptor(self._descriptor)
        self._characteristic.add_descriptor(self._writeback_descriptor)
        self.add_characteristic(self._characteristic)

    def _read_time(self):
        return struct.pack("<Q", time.time_ns())

    def _write_writeback(self, payload: bytes, options: dict):
        self._last_writeback = bytes(payload)
        if self._writeback_cb is not None:
            self._writeback_cb(self._last_writeback, options, time.time_ns())

    def update(self):
        payload = self._read_time()
        self._descriptor.set_value(payload, emit=True)
        self._characteristic.publish(payload)


class TopicBridgeService(Service):
    def __init__(self, bus, index, topic_name: str, message_type: str, bridge_name: str, bridge_key: str, *, member_specs: Sequence[BridgeMemberSpec], rate_hz: float, payload_format: str):
        service_uuid = named_service_uuid(f"bridge:{bridge_name}")
        super().__init__(bus, index, service_uuid, primary=True, name=f"bridge")
        self.topic_name = topic_name
        self.message_type = message_type
        self.bridge_name = bridge_name
        self.bridge_key = bridge_key
        self.member_specs = tuple(member_specs)
        self.rate_hz = float(rate_hz)
        self.payload_format = payload_format
        self._payload = b""
        characteristic_uuid = topic_bridge_characteristic_uuid(bridge_name)
        self._characteristic = NotifyingCharacteristic(
            bus,
            0,
            characteristic_uuid,
            ["read", "notify"],
            self,
            initial_value=b"",
            name=f"bridge/{bridge_name}",
        )
        self._data_descriptor = ReadOnlyDescriptor(
            bus,
            0,
            topic_bridge_data_descriptor_uuid(bridge_name),
            self._characteristic,
            read_cb=lambda: self._payload,
            initial_value=b"",
            name=f"bridge/{bridge_name}/data",
        )
        self._topic_descriptor = ReadOnlyDescriptor(
            bus,
            1,
            topic_bridge_metadata_descriptor_uuid(bridge_name, "topic"),
            self._characteristic,
            read_cb=lambda: self.topic_name.encode("utf-8"),
            initial_value=self.topic_name.encode("utf-8"),
            name=f"bridge/{bridge_name}/topic",
        )
        self._type_descriptor = ReadOnlyDescriptor(
            bus,
            2,
            topic_bridge_metadata_descriptor_uuid(bridge_name, "type"),
            self._characteristic,
            read_cb=lambda: self.message_type.encode("utf-8"),
            initial_value=self.message_type.encode("utf-8"),
            name=f"bridge/{bridge_name}/type",
        )
        self._format_descriptor = ReadOnlyDescriptor(
            bus,
            3,
            topic_bridge_metadata_descriptor_uuid(bridge_name, "format"),
            self._characteristic,
            read_cb=lambda: self.payload_format.encode("utf-8"),
            initial_value=self.payload_format.encode("utf-8"),
            name=f"bridge/{bridge_name}/format",
        )
        members_payload = serializable_to_bytes(member_specs_to_serializable(self.member_specs))
        self._member_descriptor = ReadOnlyDescriptor(
            bus,
            4,
            topic_bridge_metadata_descriptor_uuid(bridge_name, "members"),
            self._characteristic,
            read_cb=lambda: serializable_to_bytes(member_specs_to_serializable(self.member_specs)),
            initial_value=members_payload,
            name=f"bridge/{bridge_name}/members",
        )
        rate_payload = f"{self.rate_hz:.6f}".encode("utf-8")
        self._rate_descriptor = ReadOnlyDescriptor(
            bus,
            5,
            topic_bridge_metadata_descriptor_uuid(bridge_name, "rate_hz"),
            self._characteristic,
            read_cb=lambda: f"{self.rate_hz:.6f}".encode("utf-8"),
            initial_value=rate_payload,
            name=f"bridge/{bridge_name}/rate_hz",
        )
        key_payload = self.bridge_key.encode("utf-8")
        self._key_descriptor = ReadOnlyDescriptor(
            bus,
            6,
            topic_bridge_metadata_descriptor_uuid(bridge_name, "key"),
            self._characteristic,
            read_cb=lambda: self.bridge_key.encode("utf-8"),
            initial_value=key_payload,
            name=f"bridge/{bridge_name}/key",
        )
        self._characteristic.add_descriptor(self._data_descriptor)
        self._characteristic.add_descriptor(self._topic_descriptor)
        self._characteristic.add_descriptor(self._type_descriptor)
        self._characteristic.add_descriptor(self._format_descriptor)
        self._characteristic.add_descriptor(self._member_descriptor)
        self._characteristic.add_descriptor(self._rate_descriptor)
        self._characteristic.add_descriptor(self._key_descriptor)
        self.add_characteristic(self._characteristic)

    @property
    def characteristic_uuid(self) -> str:
        return self._characteristic.uuid

    def characteristic_path(self) -> str:
        return str(self._characteristic.get_path())

    def data_descriptor_path(self) -> str:
        return str(self._data_descriptor.get_path())

    def transport_path(self, endpoint: str) -> str:
        if endpoint == "descriptor":
            return self.data_descriptor_path()
        return self.characteristic_path()

    def publish(self, payload: bytes):
        self._payload = bytes(payload)
        self._data_descriptor.set_value(self._payload, emit=True)
        self._characteristic.publish(payload)