# SPDX-License-Identifier: GPL-3.0-only
"""Boundary checks for observed state; no instrument readback is claimed."""

import copy
import json
from pathlib import Path
import tempfile
import time
import unittest

from gui.parameters import catalogue
from gui.snapshot import (MAXIMUM_AGE_NS, MAXIMUM_BYTES, display_value,
                          publish_snapshot, read_snapshot, validate)


def fixture():
    return {"version": 1, "session": "owned-test-session", "sequence": 3,
            "captured_monotonic_ns": time.monotonic_ns(), "source": "synthetic",
            "values": {"filter.frequency": {"A": 64, "B": None},
                       "clock.tempo": {"global": 120}}}


class SnapshotTests(unittest.TestCase):
    def test_inventory_and_documented_cc_facts(self):
        inventory = {entry["id"]: entry for entry in catalogue()}
        self.assertEqual(inventory["filter.frequency"]["cc"], 74)
        self.assertEqual(inventory["osc.configuration"]["cc"], 33)
        self.assertEqual(inventory["slot.enabled"]["cc"], 115)
        self.assertEqual(inventory["slot.focus"]["cc"], 119)
        self.assertIsNone(inventory["slot.program"]["cc"])
        self.assertGreater(len(inventory), 60)

    def test_unknown_values_are_never_defaults(self):
        snapshot = validate(fixture())
        self.assertNotIn("C", snapshot["values"]["filter.frequency"])
        self.assertEqual(display_value(None), "—")
        self.assertEqual(display_value(False), "Off")
        self.assertEqual(display_value(0), "0")

    def test_reject_unsupported_schema_scope_and_values(self):
        for field, value in (("version", True), ("version", 2), ("sequence", -1),
                             ("sequence", False), ("captured_monotonic_ns", True),
                             ("source", "midi-sent"), ("session", ""),
                             ("values", {"unknown.parameter": {"A": 1}}),
                             ("values", {"filter.frequency": {"global": 1}}),
                             ("values", {"clock.tempo": {"A": 1}}),
                             ("values", {"filter.frequency": {"A": float("nan")}}),
                             ("values", {"filter.frequency": {"A": float("inf")}}),
                             ("values", {"filter.frequency": {"A": 10 ** 500}}),
                             ("values", {"filter.frequency": {"A": []}}),
                             ("values", {"filter.frequency": {"A": "x" * 97}}),
                             ("values", {"filter.frequency": {"A": "line\nbreak"}})):
            with self.subTest(field=field, value=str(value)[:100]):
                snapshot = fixture()
                snapshot[field] = value
                with self.assertRaises(ValueError):
                    validate(snapshot)
        snapshot = fixture()
        snapshot["unexpected"] = 1
        with self.assertRaises(ValueError):
            validate(snapshot)
        snapshot = fixture()
        del snapshot["source"]
        with self.assertRaises(ValueError):
            validate(snapshot)

    def test_expiry_and_future_capture_are_rejected(self):
        snapshot = fixture()
        now = snapshot["captured_monotonic_ns"]
        validate(snapshot, now_ns=now + MAXIMUM_AGE_NS)
        with self.assertRaisesRegex(ValueError, "expired"):
            validate(snapshot, now_ns=now + MAXIMUM_AGE_NS + 1)
        with self.assertRaisesRegex(ValueError, "future"):
            validate(snapshot, now_ns=now - 2_000_000_000)

    def test_atomic_publication_replaces_complete_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime/parameters.json"
            first = fixture()
            publish_snapshot(path, first)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(read_snapshot(path), first)
            second = copy.deepcopy(first)
            second["sequence"] += 1
            second["values"] = {}
            publish_snapshot(path, second)
            self.assertEqual(read_snapshot(path)["values"], {})
            self.assertEqual(list(path.parent.iterdir()), [path])
            invalid = copy.deepcopy(second)
            invalid["source"] = "invalid"
            with self.assertRaises(ValueError):
                publish_snapshot(path, invalid)
            self.assertEqual(read_snapshot(path), second)

    def test_bad_json_duplicates_and_oversized_input(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "parameters.json"
            for data in (b"{", b'{"version":1,"version":1}', b"x" * (MAXIMUM_BYTES + 1),
                         b"\xff", json.dumps([fixture()]).encode()):
                path.write_bytes(data)
                with self.subTest(data=data[:30]), self.assertRaises(ValueError):
                    read_snapshot(path)


if __name__ == "__main__":
    unittest.main()
