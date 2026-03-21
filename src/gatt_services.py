"""Server-side BLE GATT services for the ROS 2 Bluetooth node."""

import struct
import time
from typing import Callable, Optional, Sequence

from .bridge_payload import BridgeMemberSpec
from .dbus_gatt import Service
from .gatt_server import NotifyingCharacteristic
from .uuid_utils import named_characteristic_uuid, named_service_uuid

WIFI_SERVICE_NAME = "wifi"
WIFI_SSID_NAME = "wifi/ssid"
WIFI_PASSWORD_NAME = "wifi/password"
TIME_SERVICE_NAME = "time"
TIME_CHARACTERISTIC_NAME = "time/ns"
TIME_WRITE_CHARACTERISTIC_NAME = "time/ns/writeback"

WIFI_SERVICE_UUID = named_service_uuid(WIFI_SERVICE_NAME)
WIFI_CHARACTERISTIC_UUID = named_characteristic_uuid(WIFI_SSID_NAME)
WIFI_PASSWORD_CHARACTERISTIC_UUID = named_characteristic_uuid(WIFI_PASSWORD_NAME)
TIME_SERVICE_UUID = named_service_uuid(TIME_SERVICE_NAME)
TIME_CHARACTERISTIC_UUID = named_characteristic_uuid(TIME_CHARACTERISTIC_NAME)
TIME_WRITE_CHARACTERISTIC_UUID = named_characteristic_uuid(TIME_WRITE_CHARACTERISTIC_NAME)


def topic_bridge_characteristic_uuid(bridge_name: str) -> str:
    return named_characteristic_uuid(bridge_name)


def topic_bridge_service_uuid(bridge_name: str) -> str:
    return named_service_uuid(f"bridge:{bridge_name}")


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
            write_cb=lambda data, _options=None: wifi_apply_cb(data.decode("utf-8").strip()),
            initial_value=wifi_name_cb().encode("utf-8"),
            name=WIFI_SSID_NAME,
        )
        self._password_characteristic = NotifyingCharacteristic(
            bus,
            1,
            WIFI_PASSWORD_CHARACTERISTIC_UUID,
            ["read", "write", "write-without-response"],
            self,
            read_cb=lambda: wifi_password_read_cb().encode("utf-8"),
            write_cb=lambda data, _options=None: wifi_password_write_cb(data.decode("utf-8")),
            initial_value=wifi_password_read_cb().encode("utf-8"),
            name=WIFI_PASSWORD_NAME,
        )
        self.add_characteristic(self._characteristic)
        self.add_characteristic(self._password_characteristic)

    def update(self, ssid: str):
        payload = ssid.encode("utf-8")
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
        self._writeback_characteristic = NotifyingCharacteristic(
            bus,
            1,
            TIME_WRITE_CHARACTERISTIC_UUID,
            ["read", "write", "write-without-response"],
            self,
            read_cb=lambda: self._last_writeback,
            write_cb=self._write_writeback,
            initial_value=self._last_writeback,
            name=TIME_WRITE_CHARACTERISTIC_NAME,
        )
        self.add_characteristic(self._characteristic)
        self.add_characteristic(self._writeback_characteristic)

    def _read_time(self):
        return struct.pack("<Q", time.time_ns())

    def _write_writeback(self, payload: bytes, options: dict):
        self._last_writeback = bytes(payload)
        self._writeback_characteristic.set_value(self._last_writeback, emit=False)
        if self._writeback_cb is not None:
            self._writeback_cb(self._last_writeback, options, time.time_ns())

    def update(self):
        payload = self._read_time()
        self._characteristic.publish(payload)


class TopicBridgeService(Service):
    def __init__(self, bus, index, topic_name: str, message_type: str, bridge_name: str, bridge_key: str, *, member_specs: Sequence[BridgeMemberSpec], rate_hz: float, payload_format: str):
        service_uuid = topic_bridge_service_uuid(bridge_name)
        super().__init__(bus, index, service_uuid, primary=True, name=f"bridge:{bridge_name}")
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
            read_cb=lambda: self._payload,
            initial_value=b"",
            name=f"{bridge_name}",
        )
        self.add_characteristic(self._characteristic)

    @property
    def characteristic_uuid(self) -> str:
        return self._characteristic.uuid

    def characteristic_path(self) -> str:
        return str(self._characteristic.get_path())

    def transport_path(self, endpoint: str) -> str:
        return self.characteristic_path()

    def publish(self, payload: bytes):
        self._payload = bytes(payload)
        self._characteristic.publish(payload)