#!/usr/bin/env python3
"""What a release tag must agree with, before anything is built for it.

The archives are named after the version in Cargo.toml (the Makefile's
VERSION) and the release notes are the version's section of CHANGELOG.md,
so a tag that says another version, or a version with no dated section,
is stopped here and not after an hour of builds.

    release.py --tag v0.3.0 [--notes FILE]
    release.py --tag ''     (a test build of a branch: nothing to check)

The tag must be `v` and the workspace version, and CHANGELOG.md must have
a `## [VERSION] - YYYY-MM-DD` section with something in it. --notes
writes that section, without its heading, for the release's description.
With $GITHUB_OUTPUT set it writes `version` and `prerelease` there.

The tests are tools/ci/test_release.py. docs/RELEASING.md has the
procedure.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class Refused(Exception):
    """The tag, the version and the changelog do not agree."""


def workspace_version(cargo_toml: str) -> str:
    """The first `version = "..."` line, which is what the Makefile reads."""
    m = re.search(r'^version = "([^"]+)"', cargo_toml, re.M)
    if not m:
        raise Refused('Cargo.toml has no `version = "..."` line')
    return m.group(1)


def changelog_section(changelog: str, version: str) -> str:
    """The body of `## [version] - date`, up to the next `## ` heading."""
    lines = changelog.splitlines()
    heading = re.compile(r"^## \[" + re.escape(version) + r"\](.*)$")
    for start, line in enumerate(lines):
        m = heading.match(line)
        if not m:
            continue
        if not re.fullmatch(r" - \d{4}-\d{2}-\d{2}", m.group(1)):
            raise Refused(f"CHANGELOG.md: `{line}` is not `## [{version}] - YYYY-MM-DD`")
        body = []
        for following in lines[start + 1:]:
            if following.startswith("## "):
                break
            body.append(following)
        text = "\n".join(body).strip()
        if not text:
            raise Refused(f"CHANGELOG.md: the [{version}] section is empty")
        return text + "\n"
    raise Refused(
        f"CHANGELOG.md has no `## [{version}] - YYYY-MM-DD` section: "
        "the unreleased entries become it in the release commit"
    )


def check(tag: str, cargo_toml: str, changelog: str) -> tuple[str, str]:
    """The version and its release notes, for a tag that agrees with both."""
    version = workspace_version(cargo_toml)
    if tag != "v" + version:
        raise Refused(f"the tag is {tag}, and Cargo.toml says {version}: expected v{version}")
    return version, changelog_section(changelog, version)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tag", required=True, help="the pushed tag; empty for a test build")
    ap.add_argument("--notes", type=Path, help="write the changelog section here")
    args = ap.parse_args()

    cargo_toml = (ROOT / "Cargo.toml").read_text(encoding="utf-8")
    if not args.tag:
        print(f"no tag: a test build of {workspace_version(cargo_toml)}, nothing is published")
        return 0
    try:
        version, notes = check(
            args.tag, cargo_toml, (ROOT / "CHANGELOG.md").read_text(encoding="utf-8")
        )
    except Refused as e:
        print(f"release.py: {e}", file=sys.stderr)
        return 1
    # A semantic version with a hyphen is a pre-release.
    prerelease = "true" if "-" in version else "false"
    if args.notes:
        args.notes.write_text(notes, encoding="utf-8")
    output = os.environ.get("GITHUB_OUTPUT")
    if output:
        with open(output, "a", encoding="utf-8") as f:
            f.write(f"version={version}\nprerelease={prerelease}\n")
    print(f"{args.tag}: version {version}, prerelease {prerelease}, {len(notes.splitlines())} lines of notes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
