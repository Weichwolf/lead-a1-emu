# SPDX-License-Identifier: GPL-3.0-only
"""Preserve the complete original comparison matrix when selecting CTest groups."""

import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import original_images


class OriginalCheckSelectionTests(unittest.TestCase):
    arguments = ["wrapper", "decoder", "arm", "spec", "cache", "cache-owned", "exceptions",
                 "exception-owned", "interrupts", "interrupt-owned", "aintc", "aborts",
                 "abort-owned", "gpio", "spi", "boot"]
    expected = {
        "arm": [["arm", "spec", "images", "0"], ["arm", "spec", "images", "1"]],
        "cache": [["cache", "spec", "images", "cache-owned"]],
        "exceptions": [["exceptions", "spec", "images", "exception-owned"]],
        "interrupts": [["interrupts", "spec", "images", "interrupt-owned"]],
        "aintc": [["aintc", "spec", "images"]],
        "aborts": [["aborts", "spec", "images", "abort-owned"]],
        "gpio": [["gpio", "spec", "images"]],
        "spi": [["spi", "spec", "images"]],
        "spi-boot-entry-0-0": [["spi", "spec", "images", "--boot-entry", "0", "0"]],
        "spi-boot-entry-0-1": [["spi", "spec", "images", "--boot-entry", "0", "1"]],
        "spi-boot-entry-1-0": [["spi", "spec", "images", "--boot-entry", "1", "0"]],
        "spi-boot-entry-1-1": [["spi", "spec", "images", "--boot-entry", "1", "1"]],
        "spi-reset-kernel-0-0": [["spi", "spec", "images", "--reset-kernel", "0", "0"]],
        "spi-reset-kernel-0-1": [["spi", "spec", "images", "--reset-kernel", "0", "1"]],
        "spi-reset-kernel-1-0": [["spi", "spec", "images", "--reset-kernel", "1", "0"]],
        "spi-reset-kernel-1-1": [["spi", "spec", "images", "--reset-kernel", "1", "1"]],
        "boot": [["boot", "images"]],
        "spi-controls-0": [["spi", "spec", "images", "--controllers", "0"]],
        "spi-controls-1": [["spi", "spec", "images", "--controllers", "1"]],
        "spi-uploader-0": [["spi", "spec", "images", "--uploader", "0"]],
        "spi-uploader-1": [["spi", "spec", "images", "--uploader", "1"]],
        "spi-uploader-pulls-0": [["spi", "spec", "images", "--uploader-pulls", "0"]],
        "spi-uploader-pulls-1": [["spi", "spec", "images", "--uploader-pulls", "1"]],
        "spi-uploader-scheduled-0": [["spi", "spec", "images", "--uploader-scheduled", "0"]],
        "spi-uploader-scheduled-1": [["spi", "spec", "images", "--uploader-scheduled", "1"]],
        "spi-kernel-0-0": [["spi", "spec", "images", "--kernel", "0", "0"]],
        "spi-kernel-0-1": [["spi", "spec", "images", "--kernel", "0", "1"]],
        "spi-kernel-1-0": [["spi", "spec", "images", "--kernel", "1", "0"]],
        "spi-kernel-1-1": [["spi", "spec", "images", "--kernel", "1", "1"]],
    }
    specs = {"parts": [{"index": 0}, {"index": 1}]}

    def calls(self, selected):
        with patch.object(original_images.subprocess, "run") as run:
            original_images.native_checks(self.arguments, "images", self.specs, selected)
        for item in run.call_args_list:
            self.assertEqual(item.kwargs, {"check": True})
        return [item.args[0] for item in run.call_args_list]

    def test_every_selection_keeps_original_executables_fixtures_and_parts(self):
        for name, expected in self.expected.items():
            with self.subTest(name=name):
                self.assertEqual(self.calls(name), expected)

    def test_legacy_aggregate_keeps_all_groups_in_original_order(self):
        legacy = ("arm", "cache", "exceptions", "interrupts", "aintc", "aborts", "gpio", "spi", "boot")
        self.assertEqual(self.calls(None), [call for name in legacy for call in self.expected[name]])

    def test_unknown_or_incomplete_selection_fails_before_running_any_program(self):
        with patch.object(original_images.subprocess, "run") as run:
            for name in [*self.expected, "unknown"]:
                with self.subTest(name=name), self.assertRaises(ValueError):
                    original_images.native_checks(["wrapper", "decoder"], "images", self.specs, name)
            run.assert_not_called()

    def test_existing_sharc_cli_profiles_remain_accepted(self):
        for mode in ([], ["--sharc-deferred-f2"], ["--sharc-visa-instructions"],
                     ["--sharc-clocked-iop"]):
            with self.subTest(mode=mode):
                with patch.object(sys, "argv", ["wrapper", "decoder", "--sharc-only", "sharc", *mode]):
                    with patch.object(Path, "exists", return_value=False), contextlib.redirect_stdout(io.StringIO()):
                        self.assertEqual(original_images.main(), 77)

    def test_conflicting_or_incomplete_cli_modes_fail_before_reference_lookup(self):
        for extra in (["--native-check", "spi"],
                      ["--native-check", "spi", "--sharc-only", "sharc"],
                      ["--native-check", "unknown"],
                      ["--native-check", "spi", "--sharc-clocked-iop"]):
            with self.subTest(extra=extra), patch.object(sys, "argv", ["wrapper", "decoder", *extra]):
                with patch.object(Path, "exists") as exists, contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as error:
                        original_images.main()
                    self.assertEqual(error.exception.code, 2)
                    exists.assert_not_called()


if __name__ == "__main__":
    unittest.main()
