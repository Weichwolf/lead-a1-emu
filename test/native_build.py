#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify actual native compilation, sanitizer hooks and executable hardening."""

import argparse
import json
from pathlib import Path
import shlex
import subprocess


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--sanitizer", required=True)
    parser.add_argument("--nm", required=True)
    parser.add_argument("--readelf", required=True)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--native", action="append", required=True)
    args = parser.parse_args()
    commands = json.loads((args.build / "compile_commands.json").read_text())
    native_count = 0
    for entry in commands:
        flags = shlex.split(entry["command"])
        source = Path(entry["file"])
        root = Path(__file__).resolve().parent.parent
        owned = source.is_relative_to(root / "src") or source.is_relative_to(root / "test")
        native_count += source.name == "minilzo.c"
        label = entry["file"]
        for flag in ("-fstack-protector-strong", "-fstack-clash-protection", "-Wformat=2",
                     "-Werror=format-security", "-D_FORTIFY_SOURCE=3"):
            require(flag in flags, f"Missing {flag}: {label}")
        require("-fPIC" in flags or "-fPIE" in flags, f"Missing PIC/PIE: {label}")
        if owned:
            require("-Werror" in flags and "-std=c++23" in flags, f"Missing project gates: {label}")
        sanitizers = {item for flag in flags if flag.startswith("-fsanitize=")
                      for item in flag.split("=", 1)[1].split(",")}
        if args.sanitizer == "address-undefined":
            require({"address", "undefined"} <= sanitizers, f"Missing native sanitizers: {label}")
            require("-fno-sanitize-recover=all" in flags, f"Recovering sanitizer: {label}")
        elif args.sanitizer == "thread":
            require("thread" in sanitizers, f"Missing TSan: {label}")
        else:
            require(not sanitizers, f"Unexpected release instrumentation: {label}")
    require(native_count == 1, "Native miniLZO was not built from source exactly once")
    require(len(args.native) == 3, "Expected native miniLZO, zlib and SLEIGH archives")
    for archive in args.native:
        symbols = subprocess.check_output([args.nm, archive], text=True)
        if args.sanitizer == "address-undefined":
            require("__asan_" in symbols and "__ubsan_" in symbols,
                    f"Native dependency lacks ASan/UBSan object hooks: {archive}")
        elif args.sanitizer == "thread":
            require("__tsan_" in symbols, f"Native dependency lacks TSan object hooks: {archive}")
    elf = subprocess.check_output([args.readelf, "--file-header", "--program-headers",
                                   "--dynamic", args.binary], text=True)
    require("Type:                              DYN" in elf and "PIE" in elf, "Non-PIE executable")
    require("GNU_RELRO" in elf and "BIND_NOW" in elf, "Missing full RELRO")
    stack = [line for line in elf.splitlines() if "GNU_STACK" in line]
    require(len(stack) == 1 and " RW " in stack[0] and "RWE" not in stack[0], "Executable stack")
    print(f"Verified native instrumentation and hardening across {len(commands)} translation units")


if __name__ == "__main__":
    main()
