#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check native decoding against independent pinned section SHA-256 identities."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def native_checks(arguments, directory, specs):
    if len(arguments) >= 4:
        for part in specs["parts"]:
            subprocess.run([arguments[2], arguments[3], directory, str(part["index"])],
                           check=True)
    if len(arguments) >= 6:
        subprocess.run([arguments[4], arguments[3], directory, arguments[5]], check=True)
    if len(arguments) >= 8:
        subprocess.run([arguments[6], arguments[3], directory, arguments[7]], check=True)
    if len(arguments) >= 10:
        subprocess.run([arguments[8], arguments[3], directory, arguments[9]], check=True)
    if len(arguments) >= 11:
        subprocess.run([arguments[10], arguments[3], directory], check=True)
    if len(arguments) >= 13:
        subprocess.run([arguments[11], arguments[3], directory, arguments[12]], check=True)
    if len(arguments) >= 14:
        subprocess.run([arguments[13], arguments[3], directory], check=True)
    if len(arguments) >= 15:
        subprocess.run([arguments[14], arguments[3], directory], check=True)
    if len(arguments) >= 16:
        subprocess.run([arguments[15], directory], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("emulator", help="native image decoder")
    parser.add_argument("native", nargs="*", help="optional native comparison executables/fixtures")
    parser.add_argument("--sharc-only", type=Path, help="run only this original DSP stream checker")
    profiles = parser.add_mutually_exclusive_group()
    profiles.add_argument("--sharc-deferred-f2", action="store_true",
                        help="also supply the explicit candidate deferred F2 response order")
    profiles.add_argument("--sharc-visa-instructions", action="store_true",
                          help="separate VISA instruction-order fork after the candidate prefix")
    args = parser.parse_args()
    if args.sharc_only and args.native:
        parser.error("--sharc-only cannot be combined with the aggregate native comparisons")
    if args.sharc_deferred_f2 and not args.sharc_only:
        parser.error("--sharc-deferred-f2 requires --sharc-only")
    if args.sharc_visa_instructions and not args.sharc_only:
        parser.error("--sharc-visa-instructions requires --sharc-only")
    reference = ROOT / "doc/local/reference/firmware.bin"
    if not reference.exists():
        print("SKIP: original reference absent")
        return 77
    specs = json.loads((ROOT / "reference/sections.json").read_text())
    if hashlib.sha256(reference.read_bytes()).hexdigest() != specs["bundle_sha256"]:
        raise ValueError("Unexpected original bundle hash")
    with tempfile.TemporaryDirectory(dir=ROOT / "doc/local") as directory:
        subprocess.run([args.emulator, "--extract", str(reference), directory], check=True)
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
        if args.sharc_only:
            extra = ["--deferred-f2"] if args.sharc_deferred_f2 else []
            if args.sharc_visa_instructions:
                extra = ["--visa-instructions"]
            subprocess.run([str(args.sharc_only), directory, *extra], check=True)
        else:
            native_checks([sys.argv[0], args.emulator, *args.native], directory, specs)
    print("All six native decoded sections match independent pinned hashes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
