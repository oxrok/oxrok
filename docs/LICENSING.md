# Licensing

Not legal advice. The policy oxrok follows:

- The whole workspace is GPL-3.0-or-later. The drivers, the parsers and
  the file formats reproduce libsigrok's and sigrok-cli's behaviour
  closely enough to be ports of GPL code, the LogicPort driver stays
  under the GPL its author chose, and a binary that links them is GPL
  as a whole whatever the other crates said. One license keeps the
  provenance bookkeeping out of every manifest.
- The zeroplus-logic-cube driver ports libsigrok's `analyzer.c`,
  `analyzer.h` and `gl_usb.c`, which carry a BSD-style licence rather
  than the GPL. It is compatible with the GPL, and the binary as a whole
  stays GPL-3.0-or-later, but its binary clause requires reproducing the
  notice with the distribution: it is in `NOTICE` at the top of the
  repository, which the release archives ship next to `LICENSE`, and at
  the head of `src/drivers/zeroplus_logic_cube/protocol.rs`.
- Firmware loaded onto a device is a separate program; shipping it next
  to the binary is aggregation. Proprietary firmware is never shipped,
  the user installs it from the vendor. The bench firmware under `hil/`
  is part of this repository and GPL-3.0-or-later too.
- Python decoders from the sigrok distribution are loaded at run time
  from the user's install or a decoders directory. The release archives
  ship libsigrokdecode 0.5.3's `decoders` directory unchanged next to
  the binary, with libsigrokdecode's `COPYING` (GPL-3.0); like the
  binary they are GPL, and the archive is an aggregate of the two.
- The `rustpython` feature builds RustPython (MIT) and its standard
  library, which is CPython's (PSF License), into the binary; both are
  compatible with the GPL. It also brings `malachite-*`, LGPL-3.0-only:
  the LGPL-3.0 is the GPL-3.0 with extra permissions, so it links into a
  GPL binary whose source is published whole, and as `-only` it pins that
  binary to GPL-3.0 in practice.
- `serialport` is MPL-2.0, without the notice that rules out secondary
  licences, so section 3.3 lets it combine with the GPL. Its files stay
  MPL, the notices say so and the source tarball has them.
- "sigrok" is the name of another project. oxrok describes itself as
  sigrok compatible and never uses the name as its own.

## Crates whose manifest does not say, and bundled C

Four crates have no `license` field, only a `license-file`:

| Crate | Under | In the binary |
|---|---|---|
| `mt19937` 3.3.0 | BSD-3-Clause, Matsumoto and Nishimura's notice on the original C | yes, `rustpython` |
| `rustpython-doc` 0.6.0 | CPython's licence stack (`LICENSE`); the data is CPython's docstrings | yes, `rustpython` |
| `rustpython-pylib` 0.6.0 | CPython's licence stack (`Lib/PSF-LICENSE`), plus the per-file notices in `tools/third-party/python-stdlib.txt` | yes, `rustpython`: the `.py` files, frozen as bytecode |
| `syn-ext` 0.5.1 | BSD-2-Clause, Jeong YunWon | no: used by `rustpython-derive-impl`, a proc macro, at build time only |

`rustpython-pylib` also carries `Lib/ensurepip/_bundled/pip-*.whl`, a
wheel with pip and its own vendored libraries. Only `.py` files are
frozen, so it reaches the source tarball and not the binary.

No crate in the tree bundles anything that links into the binary under a
licence stricter than its manifest says, for the release features. What
is compiled from C or assembly, per release target:

| Tree | Licence | Linux musl | macOS | Windows MSVC |
|---|---|---|---|---|
| `libffi-sys` 4.2.2, `libffi/` | MIT-style, Anthony Green, Red Hat and others | not built | links the system's `libffi` as a shared library, not the bundled copy | compiled and linked statically |
| `psm` 0.1.32, `src/arch/*` assembly | the crate's MIT OR Apache-2.0 | linked | linked | linked |
| `stacker` 0.1.25, `src/arch/windows.c` | the crate's MIT OR Apache-2.0 | not built | not built | linked |

Everything else that looks like C is not compiled into the binary:
`rustpython-host_env`'s build script only probes for `altzone`,
`libbz2-rs-sys` and `libz-rs-sys` are Rust with C headers for their
users, and the `nix`, `libffi` and `ar_archive_writer` C files are tests
and examples. `xz-core`
is Rust as well, transpiled from liblzma with c2rust (liblzma is 0BSD,
the crate says MIT OR Apache-2.0). No crate ships a precompiled object
that links except `windows_x86_64_msvc`'s import library and `psm`'s
unused `wasm32.o`.

Two more things are linked statically and no crate declares them: the
Rust standard library in every binary, and on Linux musl libc (MIT, with
LLVM's libunwind from Rust's own musl target). On Windows the C runtime
is Microsoft's (`+crt-static`); it counts on GPL-3.0's System Libraries
exception.

Outside the release features, `libusb1-sys` 0.7.0 says MIT and bundles
libusb under LGPL-2.1-or-later: the `libusb` feature links it statically,
which the GPL binary allows. libffi's `msvcc.sh`, `make_sunver.pl` and
`testsuite/libffi.bhaible` are GPL-2.0 build and test tooling that is not
derived from or linked into libffi (its `LICENSE-BUILDTOOLS`); none of it
is used by the crate's build script, and it reaches only the source
tarball, as a separate program in the aggregate.

## What the release publishes

One archive per target (`make dist`), and one source tarball (`make
dist-src`), each with a `.sha256`, as assets of the same GitHub release:

| | Binary archive | `oxrok-<version>-vendored.tar.xz` |
|---|---|---|
| The program | `oxrok`, with the `rustpython` feature | the tagged tree from `git archive`, including `hil/`'s bench firmware sources; `GIT-COMMIT` names the commit |
| Our licences | `LICENSE`, `NOTICE` | `LICENSE`, `NOTICE` |
| Third parties | `THIRD-PARTY-NOTICES.txt`, for that target | `THIRD-PARTY-NOTICES.txt`, for all targets together, and `vendor/`, every crate of `Cargo.lock` for every platform with its own licence files |
| Decoders | `decoders/`, with libsigrokdecode's `COPYING` | the same under `third_party/libsigrokdecode-0.5.3/` |
| Other | `README.md`, `CHANGELOG.md` | `.cargo/config.toml`, pointing cargo at `vendor/` |

No firmware is in either: the binary includes none, it looks firmware
up at run time (`src/core/firmware.rs`), and the archives have only
what the table says. `tools/emu/*.firmware` are lists of sizes for the
emulator's stand-ins.

### The source offer

GPL-3.0 section 6(d) has the source available from the same place as the
binary, and directions to it next to the binary: the tarball is an asset
of the same release, and `THIRD-PARTY-NOTICES.txt` says so. It also
serves the MPL-2.0 and LGPL-3.0 crates. `git archive` has none of the
decoders, crates or notices, so `make dist-src` adds them; `make dist` in
the extracted tarball then rebuilds the release archive with no network
and no extra tools. The decoders are their own source, Python as
libsigrokdecode publishes it, and the tarball carries them so that it
stands alone.

The release workflow's `source-check` job extracts the tarball in an empty
directory and runs that with an empty `CARGO_HOME` and the network off
(`unshare -n`), before attaching it. The binary jobs wait for that job,
so a release cannot have binaries and no source.

`vendor/` also holds what no release binary links and each with its own
terms: the features' `libusb1-sys` and `pyo3`, the dev-dependencies, and
the GPL-2.0 libffi tooling and pip wheel from above, unmodified in their
crates.

### Checks

`deny.toml`: `cargo deny --workspace check licenses` in CI's `licenses`
job fails on a licence that is not on its allowlist, for any feature on
any platform. The LGPL and MPL crates are exceptions by name, each with
the reason. It reads manifests and licence files, so it cannot see bundled
C: when adding a `-sys` crate or a dependency with a vendored tree, read
it and add it to the tables above. Licences only, no advisories or bans.

`THIRD-PARTY-NOTICES.txt` is generated, not committed, by
`tools/third-party/notices.py`, from `make dist` and `make dist-src`, whose
failure stops the release. It runs cargo-about for the target and features
(`about.toml`, `about.hbs`) and adds what cargo-about does not do: Apache
`NOTICE` files, the copyright of crates that ship no licence file
(`copyright-holders.txt`; one that is missing there is an error), the Rust
standard library, musl on Linux and the notices of the frozen Python
modules. CI's `licenses` job generates it too, so a dependency that needs
a line there shows on the pull request. It needs cargo-about 0.9.3, and
`--features cli`: without it `cargo install` installs nothing and exits 0.

## Attribution of the sigrok counterparts

Every Rust source that corresponds to a sigrok file starts with a
`Counterpart in sigrok` comment naming that file, the copyright holders
from its header, and everyone else who committed to it according to
the libsigrok and sigrok-cli git histories, as credit to the people
whose work defined the behaviour oxrok reproduces. The comment also
says how the Rust file relates to it: a port of the counterpart (the
usual case), a port by the same author (the LogicPort driver), or a
file written from vendor documentation with the sigrok code as the
reference for scope (the FTDI layer).

`tools/attribution.py` generates these comments from the checkouts, so
they can be refreshed when the counterparts change. Names are kept as
the git history spells them; `--ascii` transliterates them. It
rewrites only what it is told to: `--only FILE...` for named files
(a new model's header) or `--all`, and `--check` lists the headers
that would change without writing. A counterpart missing from the
checkouts stops the run rather than dropping its credits (the
LogicPort's sources are only in the fork, so `--all` needs that
checkout).

## Copyright of the original code

A file that is original (not a port of a sigrok file) carries
`Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>`
instead of a `Counterpart in sigrok` comment: the `emu` framework, the
`tools`, the `tests`, the `examples` and the `hil` bench firmware. The
`emu` per-device models are original device-side reimplementations of
the devices the sigrok drivers talk to; they share only the devices'
protocol facts (constants, register maps, the clock table's values),
which are not expression, so they are original too.

`oxrok-emu` is a separate program: a standalone binary that links no
part of the GPL `oxrok` library, reached over USB/IP. It is therefore
aggregated with `oxrok`, not a combined work with it, and is
dual-licensed GPL-3.0-or-later OR Apache-2.0 (`emu/LICENSE`,
`emu/NOTICE`); the `SPDX-License-Identifier` on its files says so. The
one part that was a port of LGPL-2.1 code (the FTDI shell's baud-rate
divisor, from libftdi) was re-implemented from the FTDI AN_120 spec so
the crate can carry the Apache branch.

Its core, the contract, the bus, the shells and USB/IP, is the `vusb`
crate, MIT OR Apache-2.0, a separate project with its own repository,
which started as a copy of this crate's core by its author. It is a
dependency like any other, and `cargo deny` reads its licence. Nothing
of `vusb` is GPL only, and `emu/` keeps the models, the `.tx` log and
what knows sigrok's fixtures.
