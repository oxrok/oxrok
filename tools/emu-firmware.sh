#!/bin/sh
# Firmware stand-ins for the emulated devices.
#
#   tools/emu-firmware.sh [DIR]
#
# Drivers upload firmware whose vendor files cannot be redistributed;
# under emulation both tools upload a stand-in of the right name and
# size instead, which the model treats as opaque (docs/ROADMAP.md,
# "What counts as emulator-verified"). Each tools/emu/*.firmware file
# lists the files a model's driver loads, one 'NAME SIZE' per line
# (SIZE in bytes, decimal or 0x hex; '#' starts a comment). Every file
# is written to DIR (default ~/work/sigrok/firmware-standin, or
# $EMU_FIRMWARE_DIR) as SHA-256(NAME:0), SHA-256(NAME:1), ... up to
# its size, so the content is the same on every machine, differs
# between files, and differs between any two 32 byte blocks of a file
# (a driver that sent one chunk twice, or out of order, is caught). A file already holding those bytes is left alone, and
# the same name listed twice with different sizes is an error.
# tools/diffcli.sh --emu runs this and exports SIGROK_FIRMWARE_DIR=DIR,
# which libsigrok and oxrok both search first (src/core/firmware.rs).

set -eu
DIR=${1:-${EMU_FIRMWARE_DIR:-$HOME/work/sigrok/firmware-standin}}
mkdir -p "$DIR"
exec python3 -I - "$DIR" "$(dirname "$0")"/emu/*.firmware <<'PY'
import hashlib, os, sys

out, lists = sys.argv[1], [f for f in sys.argv[2:] if os.path.isfile(f)]
want = {}
for path in lists:
    with open(path, encoding="utf-8") as f:
        for n, line in enumerate(f, 1):
            line = line.split("#", 1)[0].split()
            if not line:
                continue
            if len(line) != 2:
                sys.exit(f"{path}:{n}: expected 'NAME SIZE'")
            name = line[0]
            try:
                size = int(line[1], 0)
            except ValueError:
                size = -1
            if size < 0:
                sys.exit(f"{path}:{n}: {line[1]}: a size is a byte count, 0 or more")
            if "/" in name or name in (".", ".."):
                sys.exit(f"{path}:{n}: {name}: a file name, not a path")
            if name in want and want[name][0] != size:
                sys.exit(f"{path}:{n}: {name} is {size} bytes here, {want[name][0]} in {want[name][1]}")
            want[name] = (size, path)
written = 0
for name, (size, _) in sorted(want.items()):
    data = b"".join(hashlib.sha256(f"{name}:{i}".encode()).digest()
                    for i in range((size + 31) // 32))[:size]
    dest = os.path.join(out, name)
    try:
        with open(dest, "rb") as f:
            if f.read() == data:
                continue
    except FileNotFoundError:
        pass
    tmp = dest + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, dest)
    written += 1
print(f"firmware stand-ins: {len(want)} in {out}, {written} written")
PY
