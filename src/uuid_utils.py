"""Helpers for hostname handling and stable md5-based UUID generation."""

import hashlib
import re
import socket

UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$"
)


def system_hostname() -> str:
    return socket.gethostname().strip()


def uuid_from_name(name: str) -> str:
    digest = hashlib.md5(name.encode("utf-8")).hexdigest()
    return "-".join([
        digest[0:8],
        digest[8:12],
        digest[12:16],
        digest[16:20],
        digest[20:32],
    ])


def is_uuid(value: str) -> bool:
    return bool(UUID_RE.fullmatch(value.strip()))


def resolve_uuid(value: str) -> str:
    candidate = value.strip()
    return candidate.lower() if is_uuid(candidate) else uuid_from_name(candidate)


def is_uav_hostname(value: str, pattern: str = r"^uav[0-9]{2}$") -> bool:
    return bool(re.fullmatch(pattern, value.strip().lower()))


def sanitize_topic_suffix(value: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_]+", "_", value.strip())
    cleaned = cleaned.strip("_")
    return cleaned or "ble_device"


def named_service_uuid(name: str) -> str:
    return uuid_from_name(f"svc:{name.strip()}")


def named_characteristic_uuid(name: str) -> str:
    return uuid_from_name(f"chr:{name.strip()}")


def named_descriptor_uuid(name: str) -> str:
    return uuid_from_name(f"dsc:{name.strip()}")
