# SPDX-License-Identifier: GPL-3.0-only
"""Independent synthetic bundle/hash checks for local reference preparation."""

import copy
import hashlib
from pathlib import Path
import struct
import tempfile
import unittest

from tools.references import identity, read_bounded, verify_bundle


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.data = struct.pack(">8I", 32, 3, 35, 5, 0, 0, 0, 0) + b"abc12345"
        self.spec = {"bytes": 40, "sha256": hashlib.sha256(self.data).hexdigest(),
                     "header_bytes": 32, "header_reserved_words": [0, 0, 0, 0],
                     "parts": [{"offset": 32, "bytes": 3,
                                "sha256": hashlib.sha256(b"abc").hexdigest()},
                               {"offset": 35, "bytes": 5,
                                "sha256": hashlib.sha256(b"12345").hexdigest()}]}

    def test_complete_independent_geometry(self):
        verify_bundle(self.data, self.spec)

    def test_damaged_content_and_truncation_fail(self):
        for data in (self.data[:-1], self.data[:-1] + b"X", self.data + b"X"):
            with self.subTest(length=len(data)), self.assertRaises(ValueError):
                verify_bundle(data, self.spec)

    def test_geometry_checked_even_with_updated_outer_hash(self):
        for offset, value in ((3, 31), (11, 34), (11, 36), (15, 4), (16, 1)):
            data = bytearray(self.data)
            data[offset] = value
            spec = copy.deepcopy(self.spec)
            spec["sha256"] = hashlib.sha256(data).hexdigest()
            with self.subTest(offset=offset, value=value), self.assertRaises(ValueError):
                verify_bundle(data, spec)

    def test_component_hashes_are_required(self):
        spec = copy.deepcopy(self.spec)
        spec["parts"][1]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "Part 1"):
            verify_bundle(self.data, spec)

    def test_identity_and_bounded_reads(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input"
            path.write_bytes(self.data)
            self.assertEqual(read_bounded(path, 40), self.data)
            with self.assertRaises(ValueError):
                read_bounded(path, 39)
        with self.assertRaises(ValueError):
            identity(self.data, {"bytes": 40, "sha256": "0" * 64}, "owned input")


if __name__ == "__main__":
    unittest.main()
