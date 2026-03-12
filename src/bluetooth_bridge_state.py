"""State containers for BLE topic and peer bridges."""

import time
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING, Tuple

from .bridge_payload import BridgeMemberSpec


if TYPE_CHECKING:
    from .gatt_server import TopicBridgeService


@dataclass
class TopicExportBridgeState:
    topic_name: str
    message_type: str
    message_class: type
    bridge_name: str
    bridge_key: str
    bridge_uuid: str
    member_specs: Tuple[BridgeMemberSpec, ...]
    rate_hz: float
    transport_endpoint: str
    payload_format: str
    subscription: object
    auto_managed: bool = False
    service: Optional["TopicBridgeService"] = None
    publish_timer: object = None
    pending_payload: bytes = b""

    @property
    def member_paths(self) -> Tuple[str, ...]:
        return tuple(spec.path for spec in self.member_specs)


@dataclass
class TopicImportBridgeState:
    mac: str
    requested_topic_name: str
    resolved_topic_name: str
    message_type: str
    message_class: type
    bridge_name: str
    bridge_key: str
    bridge_uuid: str
    member_specs: Tuple[BridgeMemberSpec, ...]
    rate_hz: float
    transport_endpoint: str
    payload_format: str
    path: str
    publisher: object
    auto_managed: bool = False
    poll_timer: object = None
    pending_payload: bytes = b""
    last_payload: bytes = b""
    last_publish_monotonic: float = 0.0
    current_hz: float = 0.0

    @property
    def member_paths(self) -> Tuple[str, ...]:
        return tuple(spec.path for spec in self.member_specs)


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
    last_rtt_s: float = 0.0