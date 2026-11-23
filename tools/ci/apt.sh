#!/bin/sh
# apt-get update and install, tried three times: a mirror that does not
# answer is the runner's trouble, not the change's.
#
#   sh tools/ci/apt.sh PACKAGE...
#
# apt-get retries each download itself (Acquire::Retries); the loop is
# for what that does not cover, a failed update or name lookup. An outage
# that outlasts it (a minute of waiting) still fails.
set -u
[ $# != 0 ] || { echo "usage: $0 PACKAGE..." >&2; exit 2; }
apt() { sudo apt-get -q -o Acquire::Retries=3 -o Acquire::http::Timeout=20 "$@"; }
for try in 1 2 3; do
    if apt update && apt install -y "$@"; then
        exit 0
    fi
    [ $try = 3 ] || { echo "apt.sh: try $try failed; again in $((try * 20)) s" >&2; sleep $((try * 20)); }
done
echo "apt.sh: could not install $*" >&2
exit 1
