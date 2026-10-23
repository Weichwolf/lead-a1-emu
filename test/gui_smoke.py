#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Exercise actual GTK table updates, filtering and snapshot lifecycle under Xvfb."""

from pathlib import Path
import shutil
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import gi

gi.require_version("Gtk", "4.0")
from gi.repository import Gio, GLib, Gtk

from gui.snapshot import MAXIMUM_AGE_NS, publish_snapshot
from gui.table import ParameterWindow, demo_snapshot


def wait_for(predicate, seconds=3):
    deadline = time.monotonic() + seconds
    context = GLib.MainContext.default()
    while time.monotonic() < deadline:
        while context.pending():
            context.iteration(False)
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("GTK state did not reach the independent expected result")


def row(window, name):
    for index in range(window.store.get_n_items()):
        candidate = window.store.get_item(index)
        if name in candidate.search:
            return candidate
    raise AssertionError(f"Missing parameter row {name}")


def run():
    application = Gtk.Application(application_id="org.lead_a1_emu.Test",
                                  flags=Gio.ApplicationFlags.NON_UNIQUE)
    application.register(None)
    with tempfile.TemporaryDirectory() as directory:
        runtime = Path(directory) / "recreated/root/lead-a1-emu"
        window = ParameterWindow(application, runtime)
        try:
            window.window.present()
            wait_for(lambda: window.window.get_mapped())
            assert "Waiting" in window.status.get_text()
            assert row(window, "filter.frequency").cells[2:6] == ["—"] * 4
            path = runtime / "parameters.json"
            snapshot = demo_snapshot(12)
            publish_snapshot(path, snapshot)
            wait_for(lambda: row(window, "filter.frequency").cells[2] == "12")
            assert "SYNTHETIC" in window.status.get_text()
            assert row(window, "filter.frequency").cells[3] == "64"
            assert row(window, "clock.tempo").cells[6] == "120"
            window.search.set_text("filter")
            wait_for(lambda: window.selection.get_n_items() == 7)
            window.search.set_text("nonexistent-parameter")
            wait_for(lambda: window.selection.get_n_items() == 0)
            window.search.set_text("")
            wait_for(lambda: window.selection.get_n_items() == window.store.get_n_items())

            # Atomic replacement must update; missing values cannot remain cached.
            snapshot = demo_snapshot(13)
            snapshot["values"] = {"filter.frequency": {"A": 17}}
            publish_snapshot(path, snapshot)
            wait_for(lambda: row(window, "filter.frequency").cells[2] == "17")
            assert row(window, "filter.frequency").cells[3] == "—"
            regressed = demo_snapshot(12)
            publish_snapshot(path, regressed)
            wait_for(lambda: "regressed" in window.status.get_text())
            assert row(window, "filter.frequency").cells[2] == "—"

            snapshot = demo_snapshot(14)
            snapshot["session"] = "fresh-runtime"
            publish_snapshot(path, snapshot)
            wait_for(lambda: row(window, "filter.frequency").cells[2] == "14")
            snapshot["sequence"] = 15
            snapshot["captured_monotonic_ns"] = time.monotonic_ns() - MAXIMUM_AGE_NS + 300_000_000
            publish_snapshot(path, snapshot)
            wait_for(lambda: "expired" in window.status.get_text())
            assert row(window, "filter.frequency").cells[2] == "—"
            path.write_text("{invalid")
            wait_for(lambda: "State unavailable" in window.status.get_text())
            shutil.rmtree(Path(directory) / "recreated")
            wait_for(lambda: "Waiting" in window.status.get_text())
            publish_snapshot(path, demo_snapshot(0))
            wait_for(lambda: row(window, "filter.frequency").cells[2] == "0")
            path.unlink()
            wait_for(lambda: "Waiting" in window.status.get_text())
            assert row(window, "filter.frequency").cells[2:6] == ["—"] * 4
        finally:
            window.watch.close()
            window.window.destroy()
    print("GTK table: unknown state, values, filtering, atomic updates, expiry, "
          "regression rejection and directory recreation pass")


if __name__ == "__main__":
    run()
