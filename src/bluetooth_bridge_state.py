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
    status: str = "connected"
    detail: str = ""
    services_wait_started_monotonic: float = 0.0
    services_wait_grace_s: float = 0.0
    pairing_requested_monotonic: float = 0.0
    pairing_failures: int = 0


@dataclass
class PeerConnectionSessionState:
    mac: str
    desired: bool = False
    explicit_target: bool = False
    peer_candidate: bool = False
    peer_name: str = ""
    phase: str = "idle"
    detail: str = ""
    first_seen_monotonic: float = field(default_factory=time.monotonic)
    last_seen_monotonic: float = field(default_factory=time.monotonic)
    last_connect_attempt_monotonic: float = 0.0
    connect_started_monotonic: float = 0.0
    connected_since_monotonic: float = 0.0
    last_security_attempt_monotonic: float = 0.0
    last_repair_monotonic: float = 0.0
    last_service_retry_monotonic: float = 0.0
    missing_since_monotonic: float = 0.0
    connect_repair_count: int = 0
    pairing_failures: int = 0
    import_bridge_missing_since: dict = field(default_factory=dict)