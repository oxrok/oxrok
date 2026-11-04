#!/usr/bin/env python3
# Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
"""Regenerate the "Counterpart in sigrok" comments of the Rust sources.

Each Rust file that corresponds to a sigrok source gets a comment naming
that source, the copyright holders from its header, and everyone else
who committed to it according to the git history, so that the credit
covers the people whose work defined the behaviour oxrok reproduces.

    tools/attribution.py [--ascii] [--check] (--only FILE... | --all) [SIGROK_SRC]

SIGROK_SRC holds the libsigrok and sigrok-cli checkouts (default:
$SIGROK_SRC, else ~/work/sigrok, else the sibling sigrok-intronix-driver
directory); it must contain libsigrok/. --only rewrites the header of
the named Rust files (repo-relative paths, as in MAP), --all rewrites
every file in MAP; one of them is required, so that regenerating one
model's header cannot rewrite the others. --check writes nothing: it
lists the files whose header would change and exits 1 if there is one.
A counterpart that names a file under SIGROK_SRC must exist there, or
the run stops instead of dropping its credits. --ascii transliterates
names to ASCII; by default they are kept as written in the git history.
"""
import argparse
import os
import re
import subprocess
import sys
import unicodedata

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEPENDENT = "a port of the sigrok counterpart, GPL-3.0-or-later like it"
SAME_AUTHOR = "a port by the same author, GPL-3.0-or-later like it"

# Rust file -> (counterparts, relation). A counterpart is a path under
# SIGROK_SRC, optionally with a note in parentheses, or free text when it
# is not a file in the checkouts.
MAP = {
    "src/core/key.rs": (["libsigrok/src/hwdriver.c (the key table)",
                                "libsigrok/include/libsigrok/libsigrok.h (the key definitions)"], INDEPENDENT),
    "src/core/option.rs": (["libsigrok/src/hwdriver.c", "libsigrok/src/std.c (the option list helpers)"], INDEPENDENT),
    "src/core/value.rs": (["sigrok-cli/parsers.c", "libsigrok/src/strutil.c"], INDEPENDENT),
    "src/core/util.rs": (["libsigrok/src/strutil.c"], INDEPENDENT),
    "src/core/session.rs": (["libsigrok/src/session.c"], INDEPENDENT),
    "src/core/packet.rs": (["libsigrok/src/session.c (the datafeed packets)"], INDEPENDENT),
    "src/core/device.rs": (["libsigrok/src/device.c", "libsigrok/src/hwdriver.c"], INDEPENDENT),
    "src/core/channel.rs": (["libsigrok/src/device.c"], INDEPENDENT),
    "src/core/softtrig.rs": (["libsigrok/src/soft-trigger.c"], INDEPENDENT),
    "src/core/firmware.rs": (["libsigrok/src/resource.c"], INDEPENDENT),
    "src/core/analog.rs": (["libsigrok/src/analog.c"], INDEPENDENT),
    "src/core/trigger.rs": (["libsigrok/src/trigger.c"], INDEPENDENT),
    "src/core/error.rs": (["libsigrok/src/error.c"], INDEPENDENT),
    "src/formats/output/mod.rs": (["libsigrok/src/output/output.c"], INDEPENDENT),
    "src/formats/output/bits.rs": (["libsigrok/src/output/bits.c"], INDEPENDENT),
    "src/formats/output/hex.rs": (["libsigrok/src/output/hex.c"], INDEPENDENT),
    "src/formats/output/ascii.rs": (["libsigrok/src/output/ascii.c"], INDEPENDENT),
    "src/formats/output/analog.rs": (["libsigrok/src/output/analog.c"], INDEPENDENT),
    "src/formats/output/csv.rs": (["libsigrok/src/output/csv.c"], INDEPENDENT),
    "src/formats/output/vcd.rs": (["libsigrok/src/output/vcd.c"], INDEPENDENT),
    "src/formats/output/binary.rs": (["libsigrok/src/output/binary.c"], INDEPENDENT),
    "src/formats/output/srzip.rs": (["libsigrok/src/output/srzip.c"], INDEPENDENT),
    "src/formats/sr.rs": (["libsigrok/src/session_file.c", "libsigrok/src/session_driver.c",
                                  "libsigrok/src/output/srzip.c"], INDEPENDENT),
    "src/formats/input/mod.rs": (["libsigrok/src/input/input.c"], INDEPENDENT),
    "src/formats/input/vcd.rs": (["libsigrok/src/input/vcd.c"], INDEPENDENT),
    "src/formats/input/binary.rs": (["libsigrok/src/input/binary.c"], INDEPENDENT),
    "src/usb/mod.rs": (["libsigrok/src/usb.c"], INDEPENDENT),
    "src/ftdi/mod.rs": (["libftdi1 (ftdi.c), by Intra2net AG and the libftdi developers"],
                               "written for oxrok from FTDI's application notes; "
                               "libftdi was the reference for which requests a driver needs"),
    "src/ezusb/mod.rs": (["libsigrok/src/ezusb.c"], INDEPENDENT),
    "src/drivers/demo/mod.rs": (["libsigrok/src/hardware/demo/api.c",
                                        "libsigrok/src/hardware/demo/protocol.c"], INDEPENDENT),
    "src/drivers/demo/patterns.rs": (["libsigrok/src/hardware/demo/protocol.c"],
                                            "a port of the sigrok counterpart, GPL-3.0-or-later like it; "
                                            "the logo bitmap was replaced by oxrok's own pattern"),
    "src/drivers/fx2lafw/mod.rs": (["libsigrok/src/hardware/fx2lafw/api.c",
                                           "libsigrok/src/hardware/fx2lafw/protocol.c",
                                           "sigrok-firmware-fx2lafw (the command set)"], INDEPENDENT),
    "src/drivers/logicport/mod.rs": (["libsigrok/src/hardware/intronix-logicport/api.c"], SAME_AUTHOR),
    "src/drivers/logicport/protocol.rs": (["libsigrok/src/hardware/intronix-logicport/protocol.c"], SAME_AUTHOR),
    "src/drivers/logicport/readout.rs": (["libsigrok/src/hardware/intronix-logicport/protocol.c (the readout)"], SAME_AUTHOR),
    "src/drivers/logicport/firmware.rs": (["libsigrok/src/hardware/intronix-logicport/protocol.c (the ccf loader)"], SAME_AUTHOR),
    "src/cli/mod.rs": (["sigrok-cli/main.c"], INDEPENDENT),
    "src/cli/args.rs": (["sigrok-cli/options.c"], INDEPENDENT),
    "src/cli/parse.rs": (["sigrok-cli/parsers.c"], INDEPENDENT),
    "src/cli/show.rs": (["sigrok-cli/show.c"], INDEPENDENT),
    "src/cli/run.rs": (["sigrok-cli/session.c"], INDEPENDENT),
    "src/cli/pipeline.rs": (["sigrok-cli/session.c"], INDEPENDENT),
    "src/cli/input.rs": (["sigrok-cli/input.c"], INDEPENDENT),
    "src/cli/logger.rs": (["libsigrok/src/log.c"], INDEPENDENT),
}

# Entries for the emulator's device models live one file per model in
# tools/attribution.d (NAME.py, run with MAP, INDEPENDENT and SAME_AUTHOR
# in scope), so that models written in parallel do not edit this file.
_FRAGMENTS = os.path.join(ROOT, "tools", "attribution.d")
for _f in sorted(os.listdir(_FRAGMENTS)) if os.path.isdir(_FRAGMENTS) else []:
    if _f.endswith(".py"):
        with open(os.path.join(_FRAGMENTS, _f), encoding="utf-8") as _src:
            exec(compile(_src.read(), _f, "exec"),
                 {"MAP": MAP, "INDEPENDENT": INDEPENDENT, "SAME_AUTHOR": SAME_AUTHOR})


def ascii(name):
    return unicodedata.normalize("NFKD", name).encode("ascii", "ignore").decode()


def holders(path):
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = re.match(r"\s*\*\s*(Copyright \(C\) .*?)[\s,]*$", line)
            if m:
                out.append(m.group(1))
            elif out and "*/" in line:
                break
    return out


def committers(repo, rel):
    log = subprocess.run(["git", "-C", repo, "log", "--format=%an", "--", rel],
                         capture_output=True, text=True, check=True).stdout.split("\n")
    counts = {}
    for a in log:
        if a:
            counts[a] = counts.get(a, 0) + 1
    return [a for a, _ in sorted(counts.items(), key=lambda x: (-x[1], x[0]))]


def describe(repo):
    r = subprocess.run(["git", "-C", repo, "describe", "--always", "--dirty"],
                       capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else "unknown"


def wrap(prefix, items, sep, width=78):
    """Comment lines starting with `prefix`, items joined by `sep`."""
    lines, cur = [], prefix
    for i, item in enumerate(items):
        piece = item + (sep.rstrip() if i + 1 < len(items) else "")
        if len(cur) + len(piece) + 1 > width:
            lines.append(cur.rstrip())
            cur = "//   " + piece
        else:
            cur += " " + piece
    lines.append(cur.rstrip())
    return lines


def default_src():
    env = os.environ.get("SIGROK_SRC")
    if env:
        return env
    work = os.path.expanduser("~/work/sigrok")
    if os.path.isdir(os.path.join(work, "libsigrok")):
        return work
    return os.path.join(os.path.dirname(ROOT), "sigrok-intronix-driver")


def main():
    ap = argparse.ArgumentParser(
        description="Regenerate the 'Counterpart in sigrok' headers.",
        epilog=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", nargs="?", default=None, metavar="SIGROK_SRC")
    ap.add_argument("--ascii", action="store_true")
    ap.add_argument("--check", action="store_true",
                    help="write nothing; list the headers that would change")
    which = ap.add_mutually_exclusive_group(required=True)
    which.add_argument("--only", nargs="+", metavar="FILE")
    which.add_argument("--all", action="store_true")
    a = ap.parse_args()
    src = a.src or default_src()
    if not os.path.isdir(os.path.join(src, "libsigrok")):
        sys.exit(f"attribution.py: {src} holds no libsigrok checkout")
    targets = MAP if a.all else {}
    for f in a.only or []:
        f = os.path.relpath(os.path.abspath(f), ROOT) if os.path.exists(f) else f
        if f not in MAP:
            sys.exit(f"attribution.py: {f} has no entry in MAP or tools/attribution.d")
        targets[f] = MAP[f]
    versions = {}
    changed = []
    for rust, (parts, relation) in targets.items():
        lines = ["// Counterpart in sigrok:"]
        all_holders, all_committers = [], []
        for part in parts:
            path = part.split(" (")[0]
            full = os.path.join(src, path)
            lines.append("//   " + part + ("," if part != parts[-1] else ","))
            if not os.path.isfile(full):
                top = path.split("/")[0]
                if "/" in path and os.path.isdir(os.path.join(src, top)):
                    sys.exit(f"attribution.py: {rust}: {full} does not exist")
                continue
            repo = os.path.join(src, path.split("/")[0])
            versions[path.split("/")[0]] = describe(repo)
            for h in holders(full):
                if h not in all_holders:
                    all_holders.append(h)
            for c in committers(repo, os.path.relpath(full, repo)):
                if c not in all_committers:
                    all_committers.append(c)
        names_in_holders = [re.sub(r".*\d{4}(?:-\d{4})?\s+([^<]+?)\s*<.*", r"\1", h) for h in all_holders]
        extra = [c for c in all_committers if c not in names_in_holders]
        if a.ascii:
            all_holders = [ascii(h) for h in all_holders]
            extra = [ascii(e) for e in extra]
        for h in all_holders:
            lines.append("//   " + h + ",")
        if extra:
            lines += wrap("//   with commits by", extra, ", ")
            lines[-1] += ","
        used = sorted({p.split("/")[0] for p in parts if p.split("/")[0] in versions})
        if used:
            lines[-1] = lines[-1].rstrip(",")
            lines.append("//   (" + ", ".join(f"{u} {versions[u]}" for u in used) + ").")
        else:
            lines[-1] = lines[-1].rstrip(",") + "."
        lines += wrap("// This file is", relation.split(" "), " ")
        lines[-1] += "."
        block = "\n".join(lines) + "\n\n"
        full_rust = os.path.join(ROOT, rust)
        orig = open(full_rust, encoding="utf-8").read()
        text = orig
        if text.startswith("// Counterpart in sigrok"):
            end = text.index("\n\n") + 2
            text = text[end:]
        if block + text != orig:
            changed.append(rust)
            if not a.check:
                open(full_rust, "w", encoding="utf-8").write(block + text)
    verb = "would change" if a.check else "updated"
    for c in changed:
        print(f"{verb}: {c}")
    print(f"{len(changed)} of {len(targets)} headers {verb}")
    if a.check and changed:
        sys.exit(1)


main()
