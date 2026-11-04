#!/usr/bin/env python3
"""Write THIRD-PARTY-NOTICES.txt: what a release binary links, and under what.

    tools/third-party/notices.py --target TRIPLE [--target TRIPLE...] \
        [--features LIST] --output FILE

cargo-about gives each crate in the release's dependency graph with its
licence text (about.toml, about.hbs). It does not see these, which this
script adds:

- the Apache NOTICE files of those crates (none today, but the Apache
  licence wants them kept when one appears);
- copyright lines for crates that ship no licence file, which
  cargo-about fills with the generic text and "<year> <copyright
  holders>". copyright-holders.txt says who they are, and a crate missing
  from it is an error rather than a placeholder in the archive;
- what no crate declares: rust-std.txt, musl-COPYRIGHT on Linux,
  python-stdlib.txt with the rustpython feature.

Any failure here is a non-zero exit, which stops `make dist`.
"""

import argparse
import fnmatch
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PLACEHOLDER = "<year> <copyright holders>"
RULE = "=" * 72


def about(targets, features, *extra):
    cmd = ["cargo", "about", "generate", "--locked", "--offline", "--fail"]
    cmd += ["-c", os.path.join(HERE, "about.toml")]
    if features:
        cmd += ["--features", features]
    for target in targets:
        cmd += ["--target", target]
    subprocess.run(cmd + list(extra), check=True)


def holders_table():
    table = []
    with open(os.path.join(HERE, "copyright-holders.txt"), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                pattern, _, text = line.partition(" :: ")
                table.append((pattern, text))
    return table


def holder_of(table, name):
    for pattern, text in table:
        if fnmatch.fnmatchcase(name, pattern):
            return text
    return None


def apache_notices(crates):
    """The NOTICE files of the crates: Apache-2.0 section 4(d)."""
    found = []
    for entry in crates:
        package = entry["package"]
        if package["source"] is None:  # the workspace: its NOTICE ships anyway
            continue
        root = os.path.dirname(package["manifest_path"])
        for path, _, files in sorted(os.walk(root)):
            for name in sorted(files):
                if re.fullmatch(r"NOTICE(\..*)?", name, re.IGNORECASE):
                    full = os.path.join(path, name)
                    rel = os.path.relpath(full, root).replace(os.sep, "/")
                    with open(full, encoding="utf-8", errors="replace") as f:
                        found.append((f'{package["name"]} {package["version"]}, {rel}', f.read()))
    return found


def read(name):
    with open(os.path.join(HERE, name), encoding="utf-8") as f:
        return f.read().rstrip("\n") + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", action="append", required=True)
    parser.add_argument("--features", default="")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    # cargo-about runs offline, so that what it finds does not depend on
    # the network; the crates of every platform have to be there first.
    subprocess.run(["cargo", "fetch", "--locked"], check=True)
    with tempfile.TemporaryDirectory() as tmp:
        data_path = os.path.join(tmp, "about.json")
        text_path = os.path.join(tmp, "about.txt")
        about(args.target, args.features, "--format", "json", "-o", data_path)
        about(args.target, args.features, os.path.join(HERE, "about.hbs"), "-o", text_path)
        with open(data_path, encoding="utf-8") as f:
            data = json.load(f)
        with open(text_path, encoding="utf-8") as f:
            crates_text = f.read()

    table = holders_table()
    placeholders = sorted(
        {
            used["crate"]["name"]
            for lic in data["licenses"]
            if PLACEHOLDER in lic["text"]
            for used in lic["used_by"]
        }
    )
    unknown = [name for name in placeholders if holder_of(table, name) is None]
    if unknown:
        sys.exit(
            "notices.py: these crates ship no licence file and are not in "
            "copyright-holders.txt: " + ", ".join(unknown)
        )
    if PLACEHOLDER not in crates_text and placeholders:
        sys.exit("notices.py: the template no longer shows cargo-about's placeholder")
    crates_text = crates_text.replace(
        "Copyright (c) " + PLACEHOLDER,
        'Copyright: the holders under "Copyright holders" below',
    )

    parts = [read("header.txt"), crates_text.rstrip("\n") + "\n"]

    notices = apache_notices(data["crates"])
    if notices:
        section = f"{RULE}\nApache NOTICE files\n{RULE}\n"
        for where, text in notices:
            section += f"\n{where}\n\n{text.rstrip()}\n"
        parts.append(section)

    section = (
        f"{RULE}\nCopyright holders\n{RULE}\n\n"
        "These crates ship no licence file, so the crate lists above show\n"
        "the generic text; the holders are from their upstream repositories\n"
        "and manifests.\n\n"
    )
    for name in placeholders:
        section += f"{name}: {holder_of(table, name)}\n"
    parts.append(section)

    parts.append(f"{RULE}\nThe Rust standard library\n{RULE}\n\n{read('rust-std.txt')}")
    if any("linux-musl" in target for target in args.target):
        parts.append(
            f"{RULE}\nmusl libc (Linux binaries, linked statically)\n{RULE}\n\n"
            f"{read('musl-COPYRIGHT')}"
        )
    if "rustpython" in args.features:
        parts.append(f"{RULE}\n{read('python-stdlib.txt')}")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n\n".join(parts))
    print(f"{args.output}: {len(data['crates'])} crates, {len(notices)} NOTICE files")


if __name__ == "__main__":
    main()
