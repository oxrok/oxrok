#!/usr/bin/env python3
"""Folds an lcov file of the Rust tests into Markdown tables of line coverage.

    cargo llvm-cov report --lcov --output-path lcov.info
    python3 -I tools/ci/coverage_summary.py lcov.info >>"$GITHUB_STEP_SUMMARY"

One table by area (src/core, src/scpi, emu, ...), one by driver
(src/drivers/NAME) and one by native decoder (src/decoders/NAME.rs), each
with the lines the tests reach of those the compiler instrumented. The
nightly workflow writes it to its step summary.

What it measures is the unit and integration tests (cargo nextest), on
Linux. The emulator tier, which runs oxrok as a process against the
models, is not in it, so a driver whose cases are all there reads low.
It is a list of where to look, not a figure to raise: a line that runs is
not a line whose result is checked.
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[2]


def read_lcov(text: str, root: str) -> dict[str, dict[int, int]]:
    """Line hit counts by file, paths relative to root; files outside it are dropped."""
    files: dict[str, dict[int, int]] = defaultdict(dict)
    prefix = root.rstrip("/") + "/"
    lines: dict[int, int] | None = None
    for raw in text.splitlines():
        if raw.startswith("SF:"):
            path = raw[3:]
            if path.startswith(prefix):
                lines = files[path[len(prefix) :]]
            elif not path.startswith("/"):
                lines = files[path]
            else:
                lines = None
        elif raw.startswith("DA:") and lines is not None:
            fields = raw[3:].split(",")
            line, hits = int(fields[0]), int(fields[1])
            # A file compiled into several test binaries comes once per record.
            lines[line] = max(lines.get(line, 0), hits)
        elif raw == "end_of_record":
            lines = None
    return files


def area_of(path: str) -> tuple[str, str]:
    """The table a file belongs to and its row there."""
    parts = PurePosixPath(path).parts
    if len(parts) >= 3 and parts[:2] == ("src", "drivers"):
        return "drivers", PurePosixPath(parts[2]).stem
    if len(parts) == 3 and parts[:2] == ("src", "decoders") and parts[2] != "mod.rs":
        return "decoders", PurePosixPath(parts[2]).stem
    if parts[0] == "src" and len(parts) >= 3:
        return "areas", "src/" + parts[1]
    if parts[0] == "src":
        return "areas", path
    if parts[0] == "hil" and len(parts) >= 2:
        return "areas", "/".join(parts[:2])
    return "areas", parts[0]


def fold(files: dict[str, dict[int, int]]) -> dict[str, dict[str, list[int]]]:
    """[reached, instrumented] by table and row."""
    tables: dict[str, dict[str, list[int]]] = {"areas": {}, "drivers": {}, "decoders": {}}
    for path, lines in files.items():
        table, row = area_of(path)
        cell = tables[table].setdefault(row, [0, 0])
        cell[0] += sum(1 for hits in lines.values() if hits > 0)
        cell[1] += len(lines)
    for table in ("drivers", "decoders"):
        # With the files beside them (mod.rs), already in the area's row.
        cell = tables["areas"].setdefault("src/" + table, [0, 0])
        for i in (0, 1):
            cell[i] += sum(c[i] for c in tables[table].values())
    return tables


def percent(reached: int, of: int) -> str:
    return f"{100 * reached / of:.1f}%" if of else "-"


def table(title: str, name: str, rows: dict[str, list[int]]) -> list[str]:
    """A folded table, the least reached row first."""
    out = [f"<details><summary>{title} ({len(rows)})</summary>", ""]
    out += [f"| {name} | lines reached | of | |", "| --- | ---: | ---: | ---: |"]
    for row in sorted(rows, key=lambda r: (rows[r][0] / rows[r][1] if rows[r][1] else 0, r)):
        reached, of = rows[row]
        out.append(f"| {row} | {reached} | {of} | {percent(reached, of)} |")
    return out + ["", "</details>", ""]


def render(files: dict[str, dict[int, int]]) -> str:
    tables = fold(files)
    reached = sum(1 for lines in files.values() for hits in lines.values() if hits > 0)
    of = sum(len(lines) for lines in files.values())
    out = [
        "## Rust line coverage of the unit and integration tests",
        "",
        f"{reached} of {of} lines ({percent(reached, of)}) in {len(files)} files.",
        "The emulator tier is not measured: a driver tested there reads low here.",
        "",
        "| area | lines reached | of | |",
        "| --- | ---: | ---: | ---: |",
    ]
    for row in sorted(tables["areas"]):
        r, o = tables["areas"][row]
        out.append(f"| {row} | {r} | {o} | {percent(r, o)} |")
    out.append("")
    out += table("By driver, least reached first", "driver", tables["drivers"])
    out += table("By native decoder, least reached first", "decoder", tables["decoders"])
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("lcov", help="the lcov file cargo llvm-cov wrote")
    ap.add_argument("--root", default=str(ROOT), help="the checkout the paths are under")
    args = ap.parse_args()
    files = read_lcov(Path(args.lcov).read_text(encoding="utf-8", errors="replace"), args.root)
    if not files:
        print(f"{args.lcov}: no file under {args.root}", file=sys.stderr)
        return 1
    print(render(files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
