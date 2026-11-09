#!/bin/sh
# Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
# Runs a command in a user and mount namespace of its own (unshare -Urm) with
# directories bound over others, so that a tool which opens fixed paths (sysfs)
# opens a model's files there, and nothing outside the command sees it:
#
#   tools/emu-ns.sh SRC:DST... -- COMMAND [ARGS...]
#
# Each SRC is bound over DST, in order, then COMMAND is executed in place of the
# script, with the same process id, so a signal or timeout reaches it. The
# command runs as the namespace's root, which is the caller's own user mapped:
# root's files outside stay as a user sees them (their modes decide, as they
# would for the caller). A scenario of tools/diffcli.sh sets EMU_WRAP to it
# (docs/USB.md, "Scenarios"). Fails with 125 when a bind fails; when unshare
# cannot make the namespaces, with its own status and message.
set -u
if [ "${EMU_NS_INSIDE:-}" != 1 ]; then
    EMU_NS_INSIDE=1 exec unshare -Urm sh "$0" "$@"
fi
unset EMU_NS_INSIDE
while [ $# -gt 0 ] && [ "$1" != -- ]; do
    case "$1" in
        *:/*) ;;
        *) echo "emu-ns.sh: '$1' is not SRC:DST" >&2; exit 125 ;;
    esac
    mount --bind "${1%%:*}" "${1#*:}" || exit 125
    shift
done
[ $# -gt 0 ] && shift
[ $# -gt 0 ] || { echo "emu-ns.sh: no command" >&2; exit 125; }
exec "$@"
