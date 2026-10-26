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
SPI_CASES = {
    "spi-reset-kernel-0-0": ("--reset-kernel", "0", "0"),
    "spi-reset-kernel-0-1": ("--reset-kernel", "0", "1"),
    "spi-reset-kernel-1-0": ("--reset-kernel", "1", "0"),
    "spi-reset-kernel-1-1": ("--reset-kernel", "1", "1"),
    "spi-uploader-0": ("--uploader", "0"),
    "spi-uploader-1": ("--uploader", "1"),
    "spi-controls-0": ("--controllers", "0"),
    "spi-controls-1": ("--controllers", "1"),
    "spi-kernel-0-0": ("--kernel", "0", "0"),
    "spi-kernel-0-1": ("--kernel", "0", "1"),
    "spi-kernel-1-0": ("--kernel", "1", "0"),
    "spi-kernel-1-1": ("--kernel", "1", "1"),
}
NATIVE_CHECK_ARGUMENTS = {"arm": 4, "cache": 6, "exceptions": 8, "interrupts": 10,
                          "aintc": 11, "aborts": 13, "gpio": 14, "spi": 15, "boot": 16}
NATIVE_CHECK_ARGUMENTS.update({name: 15 for name in SPI_CASES})


def native_checks(arguments, directory, specs, selected=None):
    if selected is not None:
        if selected not in NATIVE_CHECK_ARGUMENTS:
            raise ValueError("Unknown original native check")
        if len(arguments) < NATIVE_CHECK_ARGUMENTS[selected]:
            raise ValueError("Missing original native executable/fixture arguments")

    def enabled(name):
        return (selected is None or selected == name) and len(arguments) >= NATIVE_CHECK_ARGUMENTS[name]

    if enabled("arm"):
        for part in specs["parts"]:
            subprocess.run([arguments[2], arguments[3], directory, str(part["index"])],
                           check=True)
    if enabled("cache"):
        subprocess.run([arguments[4], arguments[3], directory, arguments[5]], check=True)
    if enabled("exceptions"):
        subprocess.run([arguments[6], arguments[3], directory, arguments[7]], check=True)
    if enabled("interrupts"):
        subprocess.run([arguments[8], arguments[3], directory, arguments[9]], check=True)
    if enabled("aintc"):
        subprocess.run([arguments[10], arguments[3], directory], check=True)
    if enabled("aborts"):
        subprocess.run([arguments[11], arguments[3], directory, arguments[12]], check=True)
    if enabled("gpio"):
        subprocess.run([arguments[13], arguments[3], directory], check=True)
    if enabled("spi"):
        subprocess.run([arguments[14], arguments[3], directory], check=True)
    if selected in SPI_CASES:
        subprocess.run([arguments[14], arguments[3], directory, *SPI_CASES[selected]], check=True)
    if enabled("boot"):
        subprocess.run([arguments[15], directory], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("emulator", help="native image decoder")
    parser.add_argument("native", nargs="*", help="optional native comparison executables/fixtures")
    checks = parser.add_mutually_exclusive_group()
    checks.add_argument("--sharc-only", type=Path, help="run only this original DSP stream checker")
    checks.add_argument("--native-check", choices=tuple(NATIVE_CHECK_ARGUMENTS),
                        help="run one unchanged native comparison group after all identity checks")
    profiles = parser.add_mutually_exclusive_group()
    profiles.add_argument("--sharc-deferred-f2", action="store_true",
                        help="also supply the explicit candidate deferred F2 response order")
    profiles.add_argument("--sharc-visa-instructions", action="store_true",
                          help="separate VISA instruction-order fork after the candidate prefix")
    profiles.add_argument("--sharc-clocked-iop", action="store_true",
                          help="clocked application initialization with explicit IOP timing")
    args = parser.parse_args()
    if args.sharc_only and args.native:
        parser.error("--sharc-only cannot be combined with the aggregate native comparisons")
    if args.sharc_deferred_f2 and not args.sharc_only:
        parser.error("--sharc-deferred-f2 requires --sharc-only")
    if args.sharc_visa_instructions and not args.sharc_only:
        parser.error("--sharc-visa-instructions requires --sharc-only")
    if args.sharc_clocked_iop and not args.sharc_only:
        parser.error("--sharc-clocked-iop requires --sharc-only")
    if args.native_check and len(args.native) + 2 < NATIVE_CHECK_ARGUMENTS[args.native_check]:
        parser.error("--native-check requires its native executable/fixture arguments")
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
            if args.sharc_clocked_iop:
                extra = ["--clocked-iop"]
            subprocess.run([str(args.sharc_only), directory, *extra], check=True)
        else:
            native_checks([sys.argv[0], args.emulator, *args.native], directory, specs, args.native_check)
    print("All six native decoded sections match independent pinned hashes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
