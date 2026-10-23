#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Explicit download/verification of pinned native source archives."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import urllib.request
import zipfile

from references import publish, read_bounded

ROOT = Path(__file__).resolve().parent.parent


def verify_ghidra(root, spec):
    for relative, expected in spec["files"].items():
        path = root / relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Pinned Ghidra file mismatch: {path}")


def prepare_ghidra(allow_fetch):
    spec = json.loads((ROOT / "reference/ghidra.json").read_text())
    destination = ROOT / "doc/local/dependencies" / spec["directory"]
    candidates = [Path(os.environ["GHIDRA_ROOT"])] if os.environ.get("GHIDRA_ROOT") else [
        destination, Path.home() / ".local/opt" / spec["directory"]]
    for candidate in candidates:
        if candidate.is_dir():
            verify_ghidra(candidate, spec)
            print(f"Verified Ghidra native source/ARM language: {candidate}")
            return
    if not allow_fetch:
        raise ValueError("Ghidra absent; set GHIDRA_ROOT or use --fetch --ghidra")
    archive = ROOT / "doc/local/dependencies" / spec["file"]
    archive.parent.mkdir(parents=True, exist_ok=True)
    if not archive.exists():
        request = urllib.request.Request(spec["url"], headers={
            "User-Agent": "lead-a1-emu-dependency/1.0"})
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=archive.parent, delete=False) as stream:
                temporary = Path(stream.name)
                downloaded = 0
                digest = hashlib.sha256()
                with urllib.request.urlopen(request, timeout=60) as response:
                    for block in iter(lambda: response.read(1024 * 1024), b""):
                        downloaded += len(block)
                        if downloaded > spec["bytes"]:
                            raise ValueError("Ghidra download exceeds pinned size")
                        digest.update(block)
                        stream.write(block)
                if downloaded != spec["bytes"] or digest.hexdigest() != spec["sha256"]:
                    raise ValueError("Downloaded Ghidra size/hash mismatch")
            temporary.replace(archive)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    digest = hashlib.sha256()
    with archive.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    if archive.stat().st_size != spec["bytes"] or digest.hexdigest() != spec["sha256"]:
        raise ValueError("Ghidra distribution size/hash mismatch")
    with tempfile.TemporaryDirectory(dir=archive.parent) as stage:
        ready = Path(stage) / spec["directory"]
        with zipfile.ZipFile(archive) as bundle:
            for relative in spec["files"]:
                data = bundle.read(spec["directory"] + "/" + relative)
                if hashlib.sha256(data).hexdigest() != spec["files"][relative]:
                    raise ValueError(f"Ghidra distribution member mismatch: {relative}")
                publish(data, ready / relative)
        verify_ghidra(ready, spec)
        ready.replace(destination)
    verify_ghidra(destination, spec)
    print(f"Verified/extracted Ghidra native subset: {destination}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch", action="store_true", help="Explicitly allow network downloads")
    parser.add_argument("--ghidra", action="store_true", help="Also prepare pinned native Ghidra subset")
    args = parser.parse_args()
    specs = json.loads((ROOT / "doc/dependencies.json").read_text())
    try:
        for name, spec in specs.items():
            path = ROOT / "doc/local/dependencies" / spec["file"]
            if not path.exists() and args.fetch:
                request = urllib.request.Request(spec["url"], headers={
                    "User-Agent": "lead-a1-emu-dependency/1.0"})
                with urllib.request.urlopen(request, timeout=60) as response:
                    data = response.read(spec["bytes"] + 1)
                if (len(data) != spec["bytes"]
                        or hashlib.sha256(data).hexdigest() != spec["sha256"]):
                    raise ValueError(f"Downloaded {name} size/hash mismatch")
                publish(data, path)
            data = read_bounded(path, spec["bytes"])
            if (len(data) != spec["bytes"]
                    or hashlib.sha256(data).hexdigest() != spec["sha256"]):
                raise ValueError(f"{name} archive size/hash mismatch")
            print(f"Verified {name} {spec['version']}: {path}")
        if args.ghidra:
            prepare_ghidra(args.fetch)
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        parser.exit(1, f"Dependency preparation failed: {error}\n")


if __name__ == "__main__":
    sys.exit(main())
