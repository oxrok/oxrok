#!/usr/bin/env python3
"""Tests of tools/ci/release.py, on a made-up manifest and changelog, and
on the ones of this tree.

    python3 -I tools/ci/test_release.py
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("release", HERE / "release.py")
release = importlib.util.module_from_spec(_spec)
sys.modules["release"] = release
_spec.loader.exec_module(release)

CARGO = """\
[package]
name = "oxrok"
version.workspace = true

[workspace.package]
version = "1.2.0-alpha.1"
edition = "2024"

[workspace.dependencies]
oxrok = { path = ".", version = "1.2.0-alpha.1" }
"""

CHANGELOG = """\
# Changelog

## [Unreleased]

### Added

- Not out yet.

## [1.2.0-alpha.1] - 2026-01-02

A summary.

### Added

- One thing.

## [1.2.0] - 2025-12-01

- Sorts after its pre-release, and is another section.

## [1.1.0]

- No date.

## [1.0.0] - 2025-01-01

## [0.9.0] - 2024-01-01

- The oldest.
"""


class Release(unittest.TestCase):
    def test_version_is_the_workspace_one(self):
        self.assertEqual(release.workspace_version(CARGO), "1.2.0-alpha.1")
        with self.assertRaises(release.Refused):
            release.workspace_version('[package]\nname = "x"\n')

    def test_tag_that_agrees(self):
        version, notes = release.check("v1.2.0-alpha.1", CARGO, CHANGELOG)
        self.assertEqual(version, "1.2.0-alpha.1")
        self.assertEqual(notes, "A summary.\n\n### Added\n\n- One thing.\n")

    def test_tag_of_another_version(self):
        for tag in ("v1.2.0", "1.2.0-alpha.1", "v1.2.0-alpha.10", "V1.2.0-alpha.1"):
            with self.assertRaises(release.Refused, msg=tag):
                release.check(tag, CARGO, CHANGELOG)

    def test_section_is_exactly_the_version(self):
        # [1.2.0] is not taken for [1.2.0-alpha.1], nor the dots for any character.
        self.assertIn("another section", release.changelog_section(CHANGELOG, "1.2.0"))
        with self.assertRaises(release.Refused):
            release.changelog_section(CHANGELOG, "1x2.0")

    def test_last_section_runs_to_the_end(self):
        self.assertEqual(release.changelog_section(CHANGELOG, "0.9.0"), "- The oldest.\n")

    def test_section_that_is_missing_undated_or_empty(self):
        for version in ("2.0.0", "Unreleased", "1.1.0", "1.0.0"):
            with self.assertRaises(release.Refused, msg=version):
                release.changelog_section(CHANGELOG, version)

    def test_this_tree(self):
        """The Makefile's VERSION and this agree, and every released
        version in CHANGELOG.md has a section the release could use."""
        root = HERE.parents[1]
        cargo = (root / "Cargo.toml").read_text(encoding="utf-8")
        changelog = (root / "CHANGELOG.md").read_text(encoding="utf-8")
        version = release.workspace_version(cargo)
        self.assertRegex(version, r"^\d+\.\d+\.\d+(-[0-9A-Za-z.]+)?$")
        released = [
            line.split("]")[0][4:]
            for line in changelog.splitlines()
            if line.startswith("## [") and not line.startswith("## [Unreleased]")
        ]
        self.assertTrue(released)
        for v in released:
            self.assertTrue(release.changelog_section(changelog, v), v)


if __name__ == "__main__":
    unittest.main()
