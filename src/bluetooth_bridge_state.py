"""State containers for BLE topic and peer bridges."""

import time
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING, Tuple


if TYPE_CHECKING:
    from .gatt_server import TopicBridgeService


@dataclass
class TopicExportBridgeState:
    topic_name: str
    message_type: str
    message_class: type
    bridge_name: str
    bridge_uuid: str
    member_paths: Tuple[str, ...]
    rate_hz: float
    transport_endpoint: str
    payload_format: str
    subscription: object
    service: Optional["TopicBridgeService"] = None
    publish_timer: object = None
    pending_payload: bytes = b""


@dataclass
class TopicImportBridgeState:
    mac: str
    requested_topic_name: str
    resolved_topic_name: str
    message_type: str
    message_class: type
    bridge_name: str
    bridge_uuid: str
    member_paths: Tuple[str, ...]
    rate_hz: float
    transport_endpoint: str
    payload_format: str
    path: str
    publisher: object
    poll_timer: object = None
    pending_payload: bytes = b""
    last_payload: bytes = b""
    last_publish_monotonic: float = 0.0
    current_hz: float = 0.0


@dataclass
class PeerTimeBridgeState:
    mac: str
    peer_name: str
    status_topic_name: str
    characteristic_path: str
    writeback_descriptor_path: str
    publisher: object
    last_activity_monotonic: float = field(default_factory=time.monotonic)
    last_publish_monotonic: float = 0.0
    current_hz: float = 0.0
    last_time_value_ns: int = 0
    last_writeback_latency_ns: int = 0