#!/usr/bin/env python3
"""Fails when a tracked file is named like vendor firmware.

AGENTS.md, "Vendor firmware": no FX2 firmware, FPGA bitstream or vendor
installer is ever committed here, whole or in part. This is the part of
that rule a script can hold: no file in the index has one of the
extensions such files come with. The emulator's stand-in images are
`tools/emu/*.firmware`, which is not one of them.

It reads names only. Firmware under another name, or pasted into a source
file as bytes, passes: `git diff --cached --stat` after running an
extractor is still the check.

    firmware.py            the files of the index (git ls-files)
    firmware.py PATH...    these paths (no git)

The tests are tools/ci/test_firmware.py.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[2]

# Lower case, without the dot. Firmware and bitstream images, then the
# installers they are extracted from.
EXTENSIONS = {
    "fw",
    "rbf",
    "ccf",
    "bitstream",
    "bit",
    "ihx",
    "iic",
    "hex",
    "sof",
    "jed",
    "dfu",
    "exe",
    "msi",
    "cab",
}


def offending(paths: list[str]) -> list[str]:
    """The paths whose extension is one of EXTENSIONS, in any case."""
    return sorted(
        p for p in paths if PurePosixPath(p).suffix.lower().lstrip(".") in EXTENSIONS
    )


def tracked() -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "-z"], cwd=ROOT, check=True, capture_output=True
    ).stdout
    return [p for p in out.decode(errors="surrogateescape").split("\0") if p]


def main() -> int:
    paths = sys.argv[1:] or tracked()
    bad = offending(paths)
    for p in bad:
        print(f"{p}: named like vendor firmware")
    if bad:
        print(
            f"{len(bad)} file(s) that AGENTS.md (Vendor firmware) keeps out of the"
            " repository. Remove them from the commit; a stand-in image for the"
            " emulator is tools/emu/NAME.firmware.",
            file=sys.stderr,
        )
        return 1
    print(f"{len(paths)} file(s), none named like vendor firmware")
    return 0


if __name__ == "__main__":
    sys.exit(main())
