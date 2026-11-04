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
  compatible with the GPL.
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
