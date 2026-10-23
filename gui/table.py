# SPDX-License-Identifier: GPL-3.0-only
"""Native searchable parameter table for four slots and global state."""

import argparse
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

import gi

gi.require_version("Gtk", "4.0")
from gi.repository import Gio, GLib, GObject, Gtk, Pango

from .parameters import SLOTS, catalogue
from .snapshot import display_value, publish_snapshot, runtime_directory
from .watch import SnapshotWatch


class ParameterRow(GObject.Object):
    def __init__(self, entry, values):
        super().__init__()
        self.search = f"{entry['section']} {entry['label']} {entry['id']}".casefold()
        self.cells = [entry["section"], entry["label"]]
        self.cells += [display_value(values.get(slot)) for slot in SLOTS]
        self.cells += [display_value(values.get("global")),
                       str(entry["cc"]) if entry["cc"] is not None else "—"]


class ParameterWindow:
    def __init__(self, application, directory):
        self.window = Gtk.ApplicationWindow(application=application, title="Lead A1 · Parameters")
        self.window.set_default_size(1180, 740)
        self.window.set_titlebar(Gtk.HeaderBar())
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        for side in ("top", "bottom", "start", "end"):
            getattr(box, f"set_margin_{side}")(16)
        title = Gtk.Label(label="NORD LEAD A1", xalign=0)
        title.add_css_class("title-1")
        box.append(title)
        self.status = Gtk.Label(xalign=0, wrap=True)
        self.status.set_selectable(True)
        self.status.set_tooltip_text(str(Path(directory) / "parameters.json"))
        box.append(self.status)
        self.search = Gtk.SearchEntry(placeholder_text="Search section or parameter…")
        self.search.set_tooltip_text("Filter parameter rows; values are read-only firmware observations")
        box.append(self.search)
        self.store = Gio.ListStore.new(ParameterRow)
        self.filter = Gtk.CustomFilter.new(self._matches)
        filtered = Gtk.FilterListModel.new(self.store, self.filter)
        self.selection = Gtk.NoSelection.new(filtered)
        self.table = Gtk.ColumnView.new(self.selection)
        self.table.set_show_row_separators(True)
        self.table.set_show_column_separators(True)
        columns = (("Section", 140), ("Parameter", 235), ("Slot A", 100), ("Slot B", 100),
                   ("Slot C", 100), ("Slot D", 100), ("Global", 140), ("MIDI CC", 80))
        for index, (label, width) in enumerate(columns):
            factory = Gtk.SignalListItemFactory()
            factory.connect("setup", self._setup_cell)
            factory.connect("bind", self._bind_cell, index)
            column = Gtk.ColumnViewColumn.new(label, factory)
            column.set_fixed_width(width)
            column.set_resizable(True)
            self.table.append_column(column)
        scroll = Gtk.ScrolledWindow(hexpand=True, vexpand=True)
        scroll.set_child(self.table)
        box.append(scroll)
        hint = Gtk.Label(label="— Unknown   ·   A–D: slot state   ·   Global: shared state",
                         xalign=0)
        hint.add_css_class("dim-label")
        box.append(hint)
        self.window.set_child(box)
        self.snapshot = None
        self._populate(None)
        self.search.connect("search-changed", self._search_changed)
        self.watch = SnapshotWatch(directory, self._changed)
        self.window.connect("close-request", self._close)

    @staticmethod
    def _setup_cell(_factory, item):
        label = Gtk.Label(xalign=0, ellipsize=Pango.EllipsizeMode.END, selectable=True)
        label.set_margin_start(10)
        label.set_margin_end(10)
        label.set_margin_top(7)
        label.set_margin_bottom(7)
        item.set_child(label)

    @staticmethod
    def _bind_cell(_factory, item, index):
        value = item.get_item().cells[index]
        item.get_child().set_text(value)
        item.get_child().set_tooltip_text(value)

    def _matches(self, row, _data=None):
        return self.search.get_text().strip().casefold() in row.search

    def _search_changed(self, _entry):
        self.filter.changed(Gtk.FilterChange.DIFFERENT)

    def _populate(self, snapshot):
        values = snapshot["values"] if snapshot else {}
        rows = [ParameterRow(entry, values.get(entry["id"], {})) for entry in catalogue()]
        self.store.splice(0, self.store.get_n_items(), rows)

    def _changed(self, snapshot, message):
        self.status.set_text(message)
        if snapshot != self.snapshot:
            self.snapshot = snapshot
            self._populate(snapshot)
        self.status.remove_css_class("warning")
        if snapshot is None or snapshot["source"] == "synthetic":
            self.status.add_css_class("warning")

    def _close(self, _window):
        self.watch.close()
        return False


def demo_snapshot(sequence):
    return {"version": 1, "session": "isolated-ui-demo", "sequence": sequence,
            "captured_monotonic_ns": time.monotonic_ns(), "source": "synthetic",
            "values": {"slot.program": {"A": "Preview A", "B": "Preview B"},
                       "slot.enabled": {"A": True, "B": True, "C": False, "D": False},
                       "filter.frequency": {"A": sequence % 128, "B": 64},
                       "osc.configuration": {"A": "Demo", "B": "Demo"},
                       "output.level": {slot: 100 for slot in SLOTS},
                       "clock.tempo": {"global": 120}}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--runtime-dir", type=Path, help="Explicit lead-a1-emu snapshot directory")
    group.add_argument("--demo", action="store_true", help="Isolated synthetic preview")
    args = parser.parse_args()
    if os.getuid() == 0:
        parser.exit(1, "Run the parameter UI as a regular user.\n")
    temporary = tempfile.TemporaryDirectory(prefix="lead-a1-ui-demo-") if args.demo else None
    sources, windows = [], []
    try:
        directory = Path(temporary.name) if temporary else args.runtime_dir or runtime_directory()
        if not directory.is_absolute():
            raise ValueError("Snapshot directory must be absolute")
        application = Gtk.Application(application_id="org.lead_a1_emu.Parameters",
                                      flags=Gio.ApplicationFlags.NON_UNIQUE)
        if args.demo:
            sequence = 0

            def update_demo():
                nonlocal sequence
                publish_snapshot(directory / "parameters.json", demo_snapshot(sequence))
                sequence += 1
                return GLib.SOURCE_CONTINUE

            update_demo()
            sources.append(GLib.timeout_add(1000, update_demo))

        def activate(app):
            if not windows:
                windows.append(ParameterWindow(app, directory))
            windows[0].window.present()

        def stop():
            application.quit()
            return GLib.SOURCE_CONTINUE

        application.connect("activate", activate)
        sources += [GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signum, stop)
                    for signum in (signal.SIGINT, signal.SIGTERM)]
        return application.run([])
    except (OSError, ValueError) as error:
        parser.exit(1, f"Parameter UI unavailable: {error}\n")
    finally:
        for window in windows:
            window.watch.close()
            window.window.destroy()
        for source in sources:
            GLib.source_remove(source)
        if temporary:
            temporary.cleanup()


if __name__ == "__main__":
    sys.exit(main())
