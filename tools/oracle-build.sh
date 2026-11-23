#!/bin/sh
# Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
# Builds the sigrok oracle that tools/diffcli.sh --oracle compares oxrok
# with (docs/DEVICES.md, "Oracle").
#
#   tools/oracle-build.sh [DIR]
#
# DIR (default $ORACLE_DIR, else ~/work/sigrok) receives one checkout per
# project, each at the commit pinned below and built in its tree, the
# prefix DIR/install they install to, and DIR/env.sh, which puts that
# prefix on PATH, LD_LIBRARY_PATH and PKG_CONFIG_PATH:
#
#   sh tools/oracle-build.sh && . ~/work/sigrok/env.sh
#   sh tools/diffcli.sh --oracle --emu-model NAME
#
# libsigrok is the oxrok branch of https://github.com/oxrok/libsigrok:
# upstream master with the drivers upstream never merged and the fixes
# that make a driver reachable (its OXROK.md). A driver the oracle lacks
# goes to that branch first, and LIBSIGROK_REV below follows it.
#
# A checkout that has its pinned commit is not fetched again, so a
# machine can be given the sources by other means. ORACLE_GIT names the
# git that fetches (default git; "gh-app git" where a GitHub App does
# it), ORACLE_LIBSIGROK_URL another place to fetch libsigrok from.
# A checkout at another commit with local changes is left alone and the
# run fails, naming it. A project is configured again when its pin
# moved, and libsigrok's coverage counters are then deleted.
#
# libsigrok is built with --coverage (tools/emu-gcov.sh reads the
# counters in its tree). libsigrokdecode gets two edits to build with
# Python 3.9 and later: python3-embed among the pkg-config modules, and
# no PyEval_InitThreads() (nothing since 3.7, gone in 3.13).
#
# The build dependencies, in Fedora's names, are in docs/DEVICES.md.

set -eu

LIBSERIALPORT_URL=https://github.com/sigrokproject/libserialport.git
LIBSERIALPORT_REV=21b3dfe5f68c205be4086469335fd2fc2ce11ed2
LIBSIGROK_URL=${ORACLE_LIBSIGROK_URL:-https://github.com/oxrok/libsigrok.git}
LIBSIGROK_REV=49f29969285dc006b9ea249d21311811d1144a0f
LIBSIGROKDECODE_URL=https://github.com/sigrokproject/libsigrokdecode.git
# The libsigrokdecode-0.5.3 tag (the tag object is 1577c435).
LIBSIGROKDECODE_REV=97991a3919da6a07c4c87308ae66fb441bd512e3
SIGROK_CLI_URL=https://github.com/sigrokproject/sigrok-cli.git
SIGROK_CLI_REV=f44dd91347e7ac797cefc23162b9fcf0b7329f1f

case ${1:-} in
    -h|--help) sed -n '3,34s/^# \{0,1\}//p' "$0"; exit 0 ;;
    -*) echo "usage: $0 [DIR]" >&2; exit 2 ;;
esac
ROOT=${1:-${ORACLE_DIR:-$HOME/work/sigrok}}
mkdir -p "$ROOT"
ROOT=$(cd "$ROOT" && pwd -P)
PREFIX=$ROOT/install
GIT=${ORACLE_GIT:-git}
JOBS=${ORACLE_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}

export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$PREFIX/bin:$PATH"

die() { echo "$0: $*" >&2; exit 1; }

# checkout NAME URL REV: DIR/NAME at REV, fetched only when REV is not
# there. Says "moved" when the last build of NAME was of another commit.
checkout() {
    name=$1 url=$2 rev=$3 dir=$ROOT/$1
    [ -d "$dir/.git" ] || git init -q "$dir"
    if ! git -C "$dir" cat-file -e "$rev^{commit}" 2>/dev/null; then
        echo "== $name: fetching $url"
        # shellcheck disable=SC2086
        $GIT -C "$dir" fetch -q --tags "$url" '+refs/heads/*:refs/remotes/oracle/*' \
            || die "$name: cannot fetch $url"
        git -C "$dir" cat-file -e "$rev^{commit}" 2>/dev/null \
            || die "$name: $url has no commit $rev"
    fi
    if [ "$(git -C "$dir" rev-parse -q --verify HEAD 2>/dev/null || true)" != "$rev" ]; then
        [ -z "$(git -C "$dir" status --porcelain --untracked-files=no 2>/dev/null)" ] \
            || die "$name: $dir has local changes and is not at $rev; move them away first"
        git -C "$dir" checkout -q --detach "$rev"
    fi
    moved=0
    [ "$(cat "$ROOT/.oracle-$name" 2>/dev/null || true)" = "$rev" ] || moved=1
}

# build NAME REV [CONFIGURE-ARG...]: configure when the pin moved or
# nothing was configured, then make install.
build() {
    name=$1 rev=$2 dir=$ROOT/$1
    shift 2
    echo "== $name: building $(git -C "$dir" rev-parse --short=8 HEAD)"
    (
        cd "$dir"
        if [ $moved = 1 ] || [ ! -e Makefile ]; then
            ./autogen.sh >"$ROOT/.oracle-$name.log" 2>&1 \
                && ./configure --prefix="$PREFIX" "$@" >>"$ROOT/.oracle-$name.log" 2>&1
        fi && make -j"$JOBS" >>"$ROOT/.oracle-$name.log" 2>&1 \
           && make install >>"$ROOT/.oracle-$name.log" 2>&1
    ) || { tail -20 "$ROOT/.oracle-$name.log" >&2; die "$name: build failed, log in $ROOT/.oracle-$name.log"; }
    echo "$rev" >"$ROOT/.oracle-$name"
}

checkout libserialport "$LIBSERIALPORT_URL" "$LIBSERIALPORT_REV"
build libserialport "$LIBSERIALPORT_REV"

checkout libsigrok "$LIBSIGROK_URL" "$LIBSIGROK_REV"
[ $moved = 0 ] || find "$ROOT/libsigrok" -name '*.gcda' -exec rm -f {} +
CFLAGS="-O0 -g --coverage" LDFLAGS="--coverage" \
    build libsigrok "$LIBSIGROK_REV" --disable-bindings

checkout libsigrokdecode "$LIBSIGROKDECODE_URL" "$LIBSIGROKDECODE_REV"
sed -i -e 's/^\t\[python-3\.8-embed\], /\t[python3-embed], [python-3.8-embed], /' "$ROOT/libsigrokdecode/configure.ac"
sed -i -e '/^\tPyEval_InitThreads();$/d' "$ROOT/libsigrokdecode/srd.c"
grep -q '\[python3-embed\]' "$ROOT/libsigrokdecode/configure.ac" \
    || die "libsigrokdecode: configure.ac did not take the python3-embed edit"
build libsigrokdecode "$LIBSIGROKDECODE_REV"

checkout sigrok-cli "$SIGROK_CLI_URL" "$SIGROK_CLI_REV"
build sigrok-cli "$SIGROK_CLI_REV"

if [ ! -e "$ROOT/env.sh" ]; then
    cat >"$ROOT/env.sh" <<EOF
# The sigrok oracle of oxrok (tools/oracle-build.sh): . $ROOT/env.sh
export PATH="$PREFIX/bin:\$PATH"
export LD_LIBRARY_PATH="$PREFIX/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig\${PKG_CONFIG_PATH:+:\$PKG_CONFIG_PATH}"
EOF
fi

# The oracle that runs is the one pinned: libsigrok names its commit.
short=$(printf %.8s "$LIBSIGROK_REV")
version=$("$PREFIX/bin/sigrok-cli" --version) || die "sigrok-cli does not run"
echo "$version" | grep -q "libsigrok .*-git-$short" \
    || die "sigrok-cli loads another libsigrok than $short: $(echo "$version" | grep -m1 libsigrok)"
echo "$version" | grep -q 'libsigrokdecode' || die "sigrok-cli was built without libsigrokdecode"
echo "== oracle ready: libsigrok $short"
echo "   . $ROOT/env.sh"
