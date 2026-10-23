# SPDX-License-Identifier: GPL-3.0-only
"""Strict, bounded local parameter snapshot contract and atomic publication."""

import json
import math
import os
from pathlib import Path
import tempfile
import time

from .parameters import SLOTS, catalogue

MAXIMUM_BYTES = 256 * 1024
MAXIMUM_AGE_NS = 5_000_000_000
FUTURE_TOLERANCE_NS = 1_000_000_000
MAXIMUM_TEXT = 96


def runtime_directory():
    value = os.environ.get("XDG_RUNTIME_DIR")
    if not value or not Path(value).is_absolute():
        raise ValueError("XDG_RUNTIME_DIR must identify an absolute user runtime directory")
    return Path(value) / "lead-a1-emu"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _text(value):
    return (isinstance(value, str) and 0 < len(value) <= MAXIMUM_TEXT
            and not any(ord(character) < 32 or ord(character) == 127 for character in value))


def validate(snapshot, *, now_ns=None):
    expected = {"version", "session", "sequence", "captured_monotonic_ns", "source", "values"}
    if not isinstance(snapshot, dict) or set(snapshot) != expected:
        raise ValueError("Unexpected snapshot fields")
    if type(snapshot["version"]) is not int or snapshot["version"] != 1:
        raise ValueError("Unsupported snapshot version")
    if not _text(snapshot["session"]):
        raise ValueError("Invalid runtime session")
    if type(snapshot["sequence"]) is not int or snapshot["sequence"] < 0:
        raise ValueError("Invalid snapshot sequence")
    timestamp = snapshot["captured_monotonic_ns"]
    if type(timestamp) is not int or timestamp < 0:
        raise ValueError("Invalid monotonic capture timestamp")
    now = time.monotonic_ns() if now_ns is None else now_ns
    if timestamp > now + FUTURE_TOLERANCE_NS or now - timestamp > MAXIMUM_AGE_NS:
        raise ValueError("Snapshot expired or capture timestamp is in the future")
    if snapshot["source"] not in ("firmware", "synthetic"):
        raise ValueError("Unsupported value source")
    values = snapshot["values"]
    inventory = {entry["id"]: entry for entry in catalogue()}
    if not isinstance(values, dict) or values.keys() - inventory.keys():
        raise ValueError("Unknown parameter IDs")
    for name, slots in values.items():
        allowed = set(SLOTS) if inventory[name]["scope"] == "slot" else {"global"}
        if not isinstance(slots, dict) or slots.keys() - allowed:
            raise ValueError(f"Invalid scope for parameter {name}")
        for value in slots.values():
            if value is None or type(value) is bool:
                continue
            if type(value) in (int, float) and abs(value) <= 1e12 and math.isfinite(value):
                continue
            if _text(value):
                continue
            raise ValueError(f"Invalid parameter value for {name}")
    return snapshot


def read_snapshot(path, *, now_ns=None):
    with Path(path).open("rb") as stream:
        payload = stream.read(MAXIMUM_BYTES + 1)
    if len(payload) > MAXIMUM_BYTES:
        raise ValueError("Parameter snapshot exceeds size limit")
    snapshot = json.loads(payload, object_pairs_hook=_unique_object)
    return validate(snapshot, now_ns=now_ns)


def publish_snapshot(path, snapshot):
    validate(snapshot)
    payload = (json.dumps(snapshot, ensure_ascii=True, allow_nan=False) + "\n").encode()
    if len(payload) > MAXIMUM_BYTES:
        raise ValueError("Parameter snapshot exceeds size limit")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".parameters-",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def display_value(value):
    if value is None:
        return "—"
    if type(value) is bool:
        return "On" if value else "Off"
    return str(value)
