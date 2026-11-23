#!/usr/bin/env python3
"""Tests of tools/ci/firmware.py.

    python3 -I tools/ci/test_firmware.py
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("firmware", HERE / "firmware.py")
firmware = importlib.util.module_from_spec(_spec)
sys.modules["firmware"] = firmware
_spec.loader.exec_module(firmware)


class Offending(unittest.TestCase):
    def test_firmware_names(self):
        paths = [
            "fx2lafw-saleae-logic.fw",
            "tests/fixtures/top.rbf",
            "tools/emu/LogicPort.CCF",
            "a/b/design.bitstream",
            "vendor/Setup.exe",
            "src/drivers/x/image.hex",
        ]
        self.assertEqual(firmware.offending(paths), sorted(paths))

    def test_what_the_tree_holds(self):
        paths = [
            # The emulator's stand-ins and the fixtures next to them.
            "tools/emu/fx2lafw.firmware",
            "tests/fixtures/uart-hi.bits",
            "tests/fixtures/decoders/uart/hello.sr",
            # The extension is the last suffix only.
            "hil/gateware/testdata/top.fs.json",
            "docs/fw.md",
            "src/core/firmware.rs",
            # A directory of that name, and a name with no extension.
            "notes.fw/README",
            "fw",
            "Makefile",
        ]
        self.assertEqual(firmware.offending(paths), [])

    def test_this_checkout(self):
        self.assertEqual(firmware.offending(firmware.tracked()), [])


if __name__ == "__main__":
    unittest.main()
