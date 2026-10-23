# SPDX-License-Identifier: GPL-3.0-only
"""Manual-derived table inventory; no firmware addresses or guessed defaults."""

import json
from pathlib import Path

CATALOGUE = Path(__file__).resolve().parent.parent / "reference/parameters.json"
SLOTS = ("A", "B", "C", "D")


def catalogue():
    parameters = json.loads(CATALOGUE.read_text())["parameters"]
    names = [entry["id"] for entry in parameters]
    if len(names) != len(set(names)):
        raise ValueError("Duplicate parameter IDs")
    return parameters
