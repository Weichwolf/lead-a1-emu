# SPDX-License-Identifier: GPL-3.0-only
"""Monitor atomic snapshots and runtime directory recreation on the GTK loop.

Directory identity monitoring adapted from public Weichwolf/blofeld-emu;
see THIRD_PARTY.txt for provenance.
"""

from pathlib import Path

from gi.repository import Gio, GLib

from .snapshot import read_snapshot


class SnapshotWatch:
    def __init__(self, directory, changed):
        self.directory = Path(directory).absolute()
        self.path = self.directory / "parameters.json"
        self.changed = changed
        self.monitors = {}
        self.pending = 0
        self.closed = False
        self.last = None
        self.expiry = GLib.timeout_add(1000, self._tick)
        self.refresh()

    def _watch(self):
        nearest = self.directory
        while not nearest.is_dir() and nearest != nearest.parent:
            nearest = nearest.parent
        desired = {}
        for path in {nearest, nearest.parent}:
            info = path.stat()
            desired[path] = (info.st_dev, info.st_ino)
        for path, (identity, monitor) in list(self.monitors.items()):
            if desired.get(path) != identity:
                monitor.cancel()
                del self.monitors[path]
        for path, identity in desired.items():
            if path not in self.monitors:
                monitor = Gio.File.new_for_path(str(path)).monitor_directory(
                    Gio.FileMonitorFlags.WATCH_MOVES, None)
                monitor.set_rate_limit(40)
                monitor.connect("changed", self._event)
                self.monitors[path] = identity, monitor

    def _event(self, _monitor, file, other, _event):
        for entry in (file, other):
            if entry is None or entry.get_path() is None:
                continue
            path = Path(entry.get_path())
            if path == self.path or self.directory.is_relative_to(path):
                if not self.closed and not self.pending:
                    self.pending = GLib.timeout_add(40, self.refresh)
                return

    def _tick(self):
        if self.closed:
            return GLib.SOURCE_REMOVE
        self.refresh()
        return GLib.SOURCE_CONTINUE

    def refresh(self):
        if self.pending:
            GLib.source_remove(self.pending)
            self.pending = 0
        if self.closed:
            return GLib.SOURCE_REMOVE
        try:
            self._watch()
            snapshot = read_snapshot(self.path)
            current = (snapshot["session"], snapshot["sequence"], snapshot)
            if self.last is not None and current[0] == self.last[0]:
                if current[1] < self.last[1] or (current[1] == self.last[1]
                                               and snapshot != self.last[2]):
                    raise ValueError("Snapshot sequence regressed or changed without increment")
            self.last = current
            message = ("Firmware state" if snapshot["source"] == "firmware"
                       else "SYNTHETIC PREVIEW — not instrument readback")
        except FileNotFoundError:
            snapshot, message = None, "Waiting for firmware state — values are unknown"
        except (OSError, ValueError, RecursionError, GLib.Error) as error:
            snapshot, message = None, f"State unavailable — {error}"
        self.changed(snapshot, message)
        return GLib.SOURCE_REMOVE

    def close(self):
        self.closed = True
        if self.pending:
            GLib.source_remove(self.pending)
            self.pending = 0
        if self.expiry:
            GLib.source_remove(self.expiry)
            self.expiry = 0
        for _, monitor in self.monitors.values():
            monitor.cancel()
        self.monitors.clear()
