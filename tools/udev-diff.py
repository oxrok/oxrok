#!/usr/bin/env python3
"""Compare oxrok's udev rules with libsigrok's: the ids one knows and the other does not.

    python3 tools/udev-diff.py contrib/60-oxrok.rules /path/to/libsigrok/contrib/60-libsigrok.rules

Prints the ids only libsigrok lists (with its comment) and the ids only oxrok
lists. USBTMC instruments are one class rule in oxrok, so libsigrok's lines for
them are reported under their own heading, not as missing.
"""

import re
import sys

ID = re.compile(r'ATTRS\{idVendor\}=="([0-9a-f]{4})", ATTRS\{idProduct\}=="([0-9a-f]{4})"')
# Vendors libsigrok lists only for USBTMC instruments (sigrok's rules name
# them so), which oxrok's class rule covers.
USBTMC_VENDORS = {"0957", "2a8d", "2184", "1ab1", "0aad", "f4ec", "f4ed", "0b21"}
USBTMC_IDS = {("05ff", "1023")}


def ids_of(path):
    out = {}
    comment = ""
    for line in open(path, encoding="utf-8"):
        s = line.strip()
        if s.startswith("# "):
            comment = s[2:]
            continue
        m = ID.search(s)
        if m:
            out.setdefault((m.group(1), m.group(2)), comment)
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    ours, theirs = ids_of(sys.argv[1]), ids_of(sys.argv[2])
    only_theirs = {k: v for k, v in theirs.items() if k not in ours}
    usbtmc = {k: v for k, v in only_theirs.items()
              if k[0] in USBTMC_VENDORS or k in USBTMC_IDS}
    missing = {k: v for k, v in only_theirs.items() if k not in usbtmc}
    print("only in libsigrok (%d):" % len(missing))
    for (v, p), c in sorted(missing.items()):
        print("  %s:%s  %s" % (v, p, c))
    print("only in libsigrok, USBTMC instruments oxrok covers by class (%d):" % len(usbtmc))
    for (v, p), c in sorted(usbtmc.items()):
        print("  %s:%s  %s" % (v, p, c))
    only_ours = {k: v for k, v in ours.items() if k not in theirs}
    print("only in oxrok (%d):" % len(only_ours))
    for (v, p), c in sorted(only_ours.items()):
        print("  %s:%s  %s" % (v, p, c))


if __name__ == "__main__":
    main()
