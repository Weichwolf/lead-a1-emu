# SPDX-License-Identifier: GPL-3.0-only
"""Finite, deterministic SHARC add/sub expectations from exact rational values."""

import itertools
from fractions import Fraction
from pathlib import Path
import subprocess
import sys
import tempfile


def power(exponent):
    return Fraction(2**exponent) if exponent >= 0 else Fraction(1, 2**-exponent)


def operand(raw, narrow):
    if narrow:
        raw &= ~255
    exponent = (raw >> 31) & 255
    significand = (raw & ((1 << 31) - 1)) | (1 << 31)
    magnitude = Fraction(significand) * power(exponent - 158) if exponent else Fraction(0)
    return -magnitude if raw & (1 << 39) else magnitude


def expected(left, right, narrow, truncate, subtract):
    value = operand(left, narrow) + (-1 if subtract else 1) * operand(right, narrow)
    padding = 8 if narrow else 0
    precision = 24 if narrow else 32
    if not value:
        # Only two effective negative zeros yield -zero under these modes.
        negative = operand(left, narrow) == 0 and bool(left & (1 << 39)) and (
            bool(right & (1 << 39)) != subtract
        )
        return (int(negative) << 39), 0
    negative = value < 0
    value = abs(value)
    exponent = value.numerator.bit_length() - value.denominator.bit_length()
    if value < power(exponent):
        exponent -= 1
    scaled = value / power(exponent - precision + 1)
    kept, remainder = divmod(scaled.numerator, scaled.denominator)
    if not truncate and (2 * remainder > scaled.denominator or (
        2 * remainder == scaled.denominator and kept % 2
    )):
        kept += 1
    if kept == 2**precision:
        kept //= 2
        exponent += 1
    signed_bit = int(negative) << 39
    if exponent > 127:
        magnitude = (((254 << (precision - 1)) | ((1 << (precision - 1)) - 1)) << padding
                     if truncate else 255 << 31)
        return signed_bit | magnitude, 2
    if exponent < -126:
        return signed_bit, 1
    raw = ((exponent + 127) << (precision - 1)) | (kept - (1 << (precision - 1)))
    return signed_bit | (raw << padding), 0


def operands():
    exponents = (0, 1, 2, 63, 126, 127, 128, 190, 253, 254)
    fractions = (0, 1, 127, 128, 129, 255, 256, (1 << 30) - 1,
                 1 << 30, (1 << 31) - 257, (1 << 31) - 2, (1 << 31) - 1)
    return tuple((sign << 39) | (exponent << 31) | fraction
                 for sign, exponent, fraction in itertools.product((0, 1), exponents, fractions))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: sharc_float_oracle.py NATIVE_TEST")
    values = operands()
    count = 0
    with tempfile.TemporaryDirectory(prefix="lead-a1-float-oracle-") as directory:
        target = Path(directory) / "cases.txt"
        with target.open("w", encoding="ascii") as output:
            for narrow, truncate, subtract in itertools.product((False, True), repeat=3):
                for left, right in itertools.product(values, repeat=2):
                    result, flags = expected(left, right, narrow, truncate, subtract)
                    output.write(f"{left} {right} {result} {flags} "
                                 f"{int(narrow)} {int(truncate)} {int(subtract)}\n")
                    count += 1
        subprocess.run([sys.argv[1], str(target)], check=True)
    print(f"{count} exact Fraction expectations: both precisions/roundings, add/sub, "
          "all signs, fixed exponent/fraction grid; no random inputs")


if __name__ == "__main__":
    main()
