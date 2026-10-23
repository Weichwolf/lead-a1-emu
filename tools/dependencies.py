#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Explicit download/verification of pinned native source archives."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

from references import publish, read_bounded

ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch", action="store_true", help="Explicitly allow network downloads")
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
    except (OSError, ValueError) as error:
        parser.exit(1, f"Dependency preparation failed: {error}\n")


if __name__ == "__main__":
    sys.exit(main())
