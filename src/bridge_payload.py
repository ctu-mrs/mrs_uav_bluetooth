"""Helpers for projecting ROS messages into compact BLE bridge payloads."""

import json
import re
from typing import Iterable, Sequence, Tuple

from rclpy.serialization import deserialize_message, serialize_message


_PATH_SEGMENT_RE = re.compile(r"^(?P<name>[A-Za-z_][A-Za-z0-9_]*)(?:\[(?P<index>\d+)\])?$")


def normalize_member_paths(paths: Iterable[str]) -> Tuple[str, ...]:
    normalized = []
    for path in paths or ():
        candidate = str(path).strip()
        if not candidate or candidate in normalized:
            continue
        _parse_member_path(candidate)
        normalized.append(candidate)
    return tuple(normalized)


def payload_format_for_member_paths(member_paths: Sequence[str]) -> str:
    return "json" if member_paths else "ros2"


def encode_message_payload(message, member_paths: Sequence[str], payload_format: str = "") -> bytes:
    fmt = payload_format or payload_format_for_member_paths(member_paths)
    if fmt == "ros2":
        return serialize_message(message)
    values = {path: _to_builtin(_resolve_member_path(message, path)) for path in member_paths}
    return json.dumps({"members": values}, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_message_payload(payload: bytes, message_class: type, member_paths: Sequence[str], payload_format: str = ""):
    fmt = payload_format or payload_format_for_member_paths(member_paths)
    if fmt == "ros2":
        message = deserialize_message(payload, message_class)
        if not member_paths:
            return message
        projected = message_class()
        for path in member_paths:
            _assign_member_path(projected, path, _to_builtin(_resolve_member_path(message, path)))
        return projected
    decoded = json.loads(payload.decode("utf-8"))
    values = decoded.get("members", decoded)
    message = message_class()
    paths_to_assign = tuple(member_paths)
    if not paths_to_assign and isinstance(values, dict):
        paths_to_assign = normalize_member_paths(values.keys())
    for path in paths_to_assign:
        if path not in values:
            continue
        _assign_member_path(message, path, values[path])
    return message


def member_paths_to_bytes(member_paths: Sequence[str]) -> bytes:
    return json.dumps(list(member_paths), separators=(",", ":")).encode("utf-8")


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