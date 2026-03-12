"""Helpers for projecting ROS messages into compact BLE bridge payloads."""

import re
import struct
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

import yaml
from rclpy.serialization import deserialize_message, serialize_message


_PATH_SEGMENT_RE = re.compile(r"^(?P<name>[A-Za-z_][A-Za-z0-9_]*)(?:\[(?P<index>\d+)\])?$")
_STRUCT_FORMATS = {
    "bool": "?",
    "int8": "b",
    "uint8": "B",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
    "float32": "f",
    "float64": "d",
    "time_ns": "Q",
}
_ROS_TYPE_ALIASES = {
    "boolean": "bool",
    "byte": "int8",
    "octet": "uint8",
    "char": "uint8",
    "float": "float32",
    "double": "float64",
}


@dataclass(frozen=True)
class BridgeMemberSpec:
    path: str
    value_type: str


def normalize_member_paths(paths: Iterable[str]) -> Tuple[str, ...]:
    normalized = []
    for path in paths or ():
        candidate = str(path).strip()
        if not candidate or candidate in normalized:
            continue
        _parse_member_path(candidate)
        normalized.append(candidate)
    return tuple(normalized)


def normalize_member_specs(specs, *, message_class: type = None) -> Tuple[BridgeMemberSpec, ...]:
    normalized = []
    for item in specs or ():
        if isinstance(item, BridgeMemberSpec):
            spec = item
        elif isinstance(item, str):
            path = item.strip()
            if not path:
                continue
            value_type = infer_member_type(message_class, path) if message_class is not None else "float64"
            spec = BridgeMemberSpec(path=path, value_type=value_type)
        elif isinstance(item, dict):
            path = str(item.get("path", "")).strip()
            if not path:
                continue
            value_type = str(item.get("type", "")).strip().lower()
            if not value_type:
                if message_class is None:
                    raise ValueError(f"Missing type for member path {path}")
                value_type = infer_member_type(message_class, path)
            spec = BridgeMemberSpec(path=path, value_type=_normalize_value_type(value_type))
        else:
            raise ValueError(f"Unsupported member specification: {item!r}")
        _parse_member_path(spec.path)
        if spec not in normalized:
            normalized.append(spec)
    return tuple(normalized)


def member_specs_to_serializable(member_specs: Sequence[BridgeMemberSpec]):
    return [{"path": spec.path, "type": spec.value_type} for spec in member_specs]


def member_specs_from_serializable(value, *, message_class: type = None) -> Tuple[BridgeMemberSpec, ...]:
    return normalize_member_specs(value or (), message_class=message_class)


def payload_format_for_member_specs(member_specs: Sequence[BridgeMemberSpec]) -> str:
    return "struct" if member_specs else "ros2"


def payload_format_for_member_paths(member_paths: Sequence[str]) -> str:
    return payload_format_for_member_specs(normalize_member_specs(member_paths))


def serializable_to_bytes(value) -> bytes:
    return yaml.safe_dump(value, default_flow_style=True, sort_keys=False).strip().encode("utf-8")


def bytes_to_serializable(payload: bytes):
    return yaml.safe_load(payload.decode("utf-8"))


def encode_message_payload(message, member_specs: Sequence[BridgeMemberSpec], payload_format: str = "") -> bytes:
    fmt = payload_format or payload_format_for_member_specs(member_specs)
    if fmt == "ros2":
        return serialize_message(message)
    payload = bytearray()
    for spec in member_specs:
        value = _coerce_outgoing_value(_resolve_member_path(message, spec.path), spec.value_type)
        payload.extend(struct.pack("<" + _STRUCT_FORMATS[spec.value_type], value))
    return bytes(payload)


def decode_message_payload(payload: bytes, message_class: type, member_specs: Sequence[BridgeMemberSpec], payload_format: str = ""):
    fmt = payload_format or payload_format_for_member_specs(member_specs)
    if fmt == "ros2":
        message = deserialize_message(payload, message_class)
        if not member_specs:
            return message
        projected = message_class()
        for spec in member_specs:
            _assign_member_path(projected, spec.path, _to_builtin(_resolve_member_path(message, spec.path)))
        return projected
    message = message_class()
    offset = 0
    for spec in member_specs:
        struct_fmt = "<" + _STRUCT_FORMATS[spec.value_type]
        size = struct.calcsize(struct_fmt)
        if offset + size > len(payload):
            raise ValueError(f"Payload too short while decoding {spec.path} ({spec.value_type})")
        raw_value = struct.unpack_from(struct_fmt, payload, offset)[0]
        offset += size
        _assign_member_path(message, spec.path, _coerce_incoming_value(raw_value, spec.value_type))
    return message


def member_paths_to_bytes(member_paths: Sequence[str]) -> bytes:
    return serializable_to_bytes(list(member_paths))


def infer_member_type(message_class: type, path: str) -> str:
    if message_class is None:
        raise ValueError(f"Unable to infer member type for {path} without a ROS message class")
    instance = message_class()
    field_type = _resolve_member_field_type(instance, path)
    if field_type:
        return _map_ros_type(field_type)
    value = _resolve_member_path(instance, path)
    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        return "time_ns"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int64"
    if isinstance(value, float):
        return "float64"
    raise ValueError(f"Unsupported inferred member type for {path}: {type(value).__name__}")


def _resolve_member_path(root, path: str):
    current = root
    for name, index in _parse_member_path(path):
        current = getattr(current, name)
        if index is not None:
            current = current[index]
    return current


def _assign_member_path(root, path: str, value):
    segments = _parse_member_path(path)
    current = root
    for name, index in segments[:-1]:
        current = getattr(current, name)
        if index is not None:
            current = current[index]
    leaf_name, leaf_index = segments[-1]
    if leaf_index is None:
        existing = getattr(current, leaf_name)
        setattr(current, leaf_name, _merge_value(existing, value))
        return
    container = getattr(current, leaf_name)
    existing = container[leaf_index]
    container[leaf_index] = _merge_value(existing, value)


def _merge_value(existing, value):
    if hasattr(existing, "sec") and hasattr(existing, "nanosec") and isinstance(value, dict):
        existing.sec = int(value.get("sec", 0))
        existing.nanosec = int(value.get("nanosec", 0))
        return existing
    if hasattr(existing, "get_fields_and_field_types") and isinstance(value, dict):
        for field_name, field_value in value.items():
            current = getattr(existing, field_name)
            setattr(existing, field_name, _merge_value(current, field_value))
        return existing
    if isinstance(existing, (list, tuple)) and isinstance(value, list):
        return list(value)
    return value


def _to_builtin(value):
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [_to_builtin(item) for item in value]
    if hasattr(value, "get_fields_and_field_types"):
        return {field_name: _to_builtin(getattr(value, field_name)) for field_name in value.get_fields_and_field_types()}
    return value


def _parse_member_path(path: str):
    segments = []
    for raw_segment in path.split("."):
        match = _PATH_SEGMENT_RE.fullmatch(raw_segment.strip())
        if match is None:
            raise ValueError(f"Invalid member path segment: {raw_segment}")
        index = match.group("index")
        segments.append((match.group("name"), None if index is None else int(index)))
    if not segments:
        raise ValueError("Member path must not be empty")
    return segments


def _normalize_value_type(value_type: str) -> str:
    candidate = _ROS_TYPE_ALIASES.get(value_type, value_type)
    if candidate not in _STRUCT_FORMATS:
        raise ValueError(f"Unsupported bridge member type: {value_type}")
    return candidate


def _resolve_member_field_type(root, path: str):
    current = root
    field_type = None
    for name, index in _parse_member_path(path):
        if hasattr(current, "get_fields_and_field_types"):
            field_type = current.get_fields_and_field_types().get(name)
        current = getattr(current, name)
        if index is not None:
            current = current[index]
    return field_type


def _map_ros_type(field_type: str) -> str:
    candidate = str(field_type).strip()
    if candidate.startswith("builtin_interfaces/") and candidate.endswith("Time"):
        return "time_ns"
    if candidate.startswith("sequence<") or candidate.startswith("array<"):
        raise ValueError(f"Sequence bridge members are not supported for compact BLE payloads: {field_type}")
    return _normalize_value_type(_ROS_TYPE_ALIASES.get(candidate, candidate))


def _coerce_outgoing_value(value, value_type: str):
    if value_type == "time_ns":
        if hasattr(value, "sec") and hasattr(value, "nanosec"):
            return int(value.sec) * 1_000_000_000 + int(value.nanosec)
        return int(value)
    if value_type == "bool":
        return bool(value)
    if value_type.startswith("float"):
        return float(value)
    return int(value)


def _coerce_incoming_value(value, value_type: str):
    if value_type == "time_ns":
        total = max(0, int(value))
        return {
            "sec": int(total // 1_000_000_000),
            "nanosec": int(total % 1_000_000_000),
        }
    if value_type == "bool":
        return bool(value)
    if value_type.startswith("float"):
        return float(value)
    return int(value)