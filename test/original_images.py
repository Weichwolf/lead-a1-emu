#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check native decoding against independent pinned section SHA-256 identities."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def main():
    reference = ROOT / "doc/local/reference/firmware.bin"
    if not reference.exists():
        print("SKIP: original reference absent")
        return 77
    specs = json.loads((ROOT / "reference/sections.json").read_text())
    if hashlib.sha256(reference.read_bytes()).hexdigest() != specs["bundle_sha256"]:
        raise ValueError("Unexpected original bundle hash")
    with tempfile.TemporaryDirectory(dir=ROOT / "doc/local") as directory:
        subprocess.run([sys.argv[1], "--extract", str(reference), directory], check=True)
        for part in specs["parts"]:
            for section in part["sections"]:
                name = f"{part['index']}-{section['tag']}.bin"
                data = (Path(directory) / name).read_bytes()
                if (len(data) != section["decoded_bytes"]
                        or hashlib.sha256(data).hexdigest() != section["decoded_sha256"]):
                    raise ValueError(f"Decoded section identity mismatch: {name}")
        sharc = json.loads((ROOT / "reference/sharc.json").read_text())["original_kernel"]
        if sharc["bundle_sha256"] != specs["bundle_sha256"]:
            raise ValueError("SHARC kernel reference bundle mismatch")
        for placement in sharc["placements"]:
            data = (Path(directory) / f"{placement['part']}-CODE.bin").read_bytes()
            offset = int(placement["code_offset"], 16)
            kernel = data[offset:offset + sharc["bytes"]]
            if (len(kernel) != sharc["bytes"]
                    or hashlib.sha256(kernel).hexdigest() != sharc["sha256"]):
                raise ValueError("Original SHARC kernel identity mismatch")
        if len(sys.argv) >= 4:
            for part in specs["parts"]:
                subprocess.run([sys.argv[2], sys.argv[3], directory, str(part["index"])],
                               check=True)
        if len(sys.argv) >= 6:
            subprocess.run([sys.argv[4], sys.argv[3], directory, sys.argv[5]], check=True)
        if len(sys.argv) >= 8:
            subprocess.run([sys.argv[6], sys.argv[3], directory, sys.argv[7]], check=True)
        if len(sys.argv) >= 10:
            subprocess.run([sys.argv[8], sys.argv[3], directory, sys.argv[9]], check=True)
        if len(sys.argv) >= 11:
            subprocess.run([sys.argv[10], sys.argv[3], directory], check=True)
        if len(sys.argv) >= 13:
            subprocess.run([sys.argv[11], sys.argv[3], directory, sys.argv[12]], check=True)
        if len(sys.argv) >= 14:
            subprocess.run([sys.argv[13], sys.argv[3], directory], check=True)
        if len(sys.argv) >= 15:
            subprocess.run([sys.argv[14], sys.argv[3], directory], check=True)
        if len(sys.argv) >= 16:
            subprocess.run([sys.argv[15], directory], check=True)
    print("All six native decoded sections match independent pinned hashes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
