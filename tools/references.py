#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Prepare hash-pinned original references without executing the updater."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parent.parent
DESTINATION = ROOT / "doc/local/reference/firmware.bin"


def manifest():
    return json.loads((ROOT / "reference/1.36.json").read_text())


def identity(data, spec, label):
    if len(data) != spec["bytes"] or hashlib.sha256(data).hexdigest() != spec["sha256"]:
        raise ValueError(f"{label}: size/SHA-256 mismatch")


def verify_bundle(data, spec):
    identity(data, spec, "Firmware bundle")
    words = struct.unpack(">8I", data[:32])
    if list(words[4:]) != spec["header_reserved_words"]:
        raise ValueError("Unexpected reserved header words")
    expected = spec["header_bytes"]
    for index, part in enumerate(spec["parts"]):
        offset, length = words[index * 2:index * 2 + 2]
        if (offset != expected or offset != part["offset"] or length != part["bytes"]
                or length == 0 or offset + length > len(data)):
            raise ValueError(f"Unexpected part {index} geometry")
        identity(data[offset:offset + length], part, f"Part {index}")
        expected += length
    if expected != len(data):
        raise ValueError("Incomplete bundle coverage")


def read_bounded(path, limit):
    with Path(path).open("rb") as stream:
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError(f"Input exceeds expected size: {path}")
    return data


def publish(data, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(data)
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def extract_archive(path, specs):
    identity(read_bounded(path, specs["archive"]["bytes"]), specs["archive"], "Archive")
    seven_zip = shutil.which("7z")
    if seven_zip is None:
        raise ValueError("7z is required for static PE resource extraction")
    updater_spec = specs["updater"]
    with zipfile.ZipFile(path) as archive:
        if archive.namelist() != [updater_spec["member"]]:
            raise ValueError("Unexpected ZIP members")
        with archive.open(updater_spec["member"]) as member:
            updater = member.read(updater_spec["bytes"] + 1)
    identity(updater, updater_spec, "Updater")
    with tempfile.TemporaryDirectory() as directory:
        executable = Path(directory) / "updater.exe"
        executable.write_bytes(updater)
        result = subprocess.run([seven_zip, "x", "-so", "-bd", str(executable),
                                 specs["firmware"]["resource"]], check=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    verify_bundle(result.stdout, specs["firmware"])
    return result.stdout


def fetch(spec):
    path = ROOT / "doc/local/downloads" / spec["file"]
    if not path.exists():
        request = urllib.request.Request(spec["url"], headers={
            "User-Agent": "lead-a1-emu-reference/1.0"})
        with urllib.request.urlopen(request, timeout=60) as response:
            data = response.read(spec["bytes"] + 1)
        identity(data, spec, "Downloaded archive")
        publish(data, path)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--verify", action="store_true")
    group.add_argument("--import-bundle", type=Path)
    group.add_argument("--archive", type=Path)
    group.add_argument("--fetch", action="store_true")
    parser.add_argument("--allow-missing", action="store_true",
                        help="CTest only: exit 77 if the local reference is absent")
    args = parser.parse_args()
    if args.allow_missing and not args.verify:
        parser.error("--allow-missing requires --verify")
    specs = manifest()
    try:
        if args.verify and args.allow_missing and not DESTINATION.exists():
            print("SKIP: local original reference absent")
            return 77
        if args.archive or args.fetch:
            path = args.archive or fetch(specs["archive"])
            data = extract_archive(path, specs)
        else:
            data = read_bounded(args.import_bundle or DESTINATION, specs["firmware"]["bytes"])
        verify_bundle(data, specs["firmware"])
        if not args.verify:
            publish(data, DESTINATION)
    except (OSError, ValueError, struct.error, subprocess.SubprocessError,
            zipfile.BadZipFile) as error:
        parser.exit(1, f"Reference preparation failed: {error}\n")
    print(f"Verified OS 1.36 bundle and both parts: {len(data)} bytes")
    print(DESTINATION)
    return 0


if __name__ == "__main__":
    sys.exit(main())
