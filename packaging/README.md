# Packaging

What this directory holds, and how the packages relate to the release
archives of the [Releases page](https://github.com/oxrok/oxrok/releases).

## Fedora: `fedora/oxrok.spec` and `.packit.yaml`

The spec builds `oxrok` with the `rustpython` feature from the vendored
source tarball, so the build needs no network and no crate from the
distribution: `make vendor-src` makes that tarball from a checkout
(`git archive` plus `cargo vendor`), and the release workflow publishes
the same layout with the notices and the bundled decoders added
(`make dist-src`). The package depends on `libsigrokdecode` for the
Python decoders, which `oxrok` finds under
`/usr/share/libsigrokdecode/decoders`, recommends the fx2lafw firmware,
and installs `contrib/60-oxrok.rules` with the `uaccess` policy file.
The published tarballs up to 0.2.0 predate those rules, so `Source0`
builds from the release after 0.2.0 on; until then, the tarball comes
from `make vendor-src`, as Packit makes it.

[Packit](https://packit.dev) reads `.packit.yaml` at the root and, once
the Packit GitHub App is installed on the repository and the
organisation is approved by the Packit team, builds in COPR for every
active Fedora with Rawhide, CentOS Stream 10 and EPEL 10 (on RHEL), on
x86_64 and aarch64: the chroots of the
[mangelajo/oxrok](https://copr.fedorainfracloud.org/coprs/mangelajo/oxrok/)
project. A pull request builds in Packit's own namespace, one COPR project
per pull request, and reports as a check; every commit on `main` builds
into mangelajo/oxrok, which users enable with
`dnf copr enable mangelajo/oxrok`. COPR takes those builds only from the
forge projects the project allows: its settings list
`github.com/oxrok/oxrok` under "Packit allowed forge projects".
The CentOS Stream 10 chroot has no EPEL, where `cargo-rpm-macros` comes
from, so `.packit.yaml` adds the EPEL 10 repository to it.

EPEL 10 has no `libsigrokdecode`, so on RHEL and CentOS the package
recommends it instead of requiring it, and runs without the Python
decoders until they are installed.

By hand, on a Fedora machine with `mock`:

```
make vendor-src
rpmbuild -bs --define "_sourcedir $PWD/dist" --define "_srcrpmdir $PWD/dist" packaging/fedora/oxrok.spec
mock -r fedora-rawhide-x86_64 rebuild dist/oxrok-*.src.rpm
```

The build is heavy: every RustPython crate compiles with LTO and debuginfo,
and one rustc of `rustpython-vm` or of `oxrok` itself passes 5 GB of memory.
`%constrain_build -m 6144` in the spec caps `-j` at one job per 6 GB of
memory (two on a 16 GB COPR builder). A per-process memory guard below that
kills the build; `--define 'rustflags_codegen_units 16'` gets a smaller
build through for checking the spec itself.
`rpmlint -r packaging/fedora/oxrok.rpmlintrc` on the results filters the known
noise (the sigrok names, the data dependency on `libsigrokdecode`).

Fedora proper is a different build: every crate from a `rust-*` package,
no vendoring, and the decoders on CPython (`-f python`), since the pinned
RustPython crates are not in Fedora. That spec would start from
`rust2rpm`; this one is for COPR.

The `License` tag is oxrok's licence and every vendored crate's, as
`%cargo_license_summary` lists them (the spec keeps that list as a comment
above the tag). Two crates carry no licence metadata, `mt19937` and
`rustpython-pylib`, and so are not in the tag; a Fedora review would want
them settled by hand, and would also refuse `CC0-1.0` for code
(`hexf-parse`, through RustPython), which COPR does not.

## Later

Debian and Ubuntu packages from the static binary (`cargo-deb` or
`nfpm` on the release archive), a Flatpak for `oxrok-ui`, Homebrew and
the AUR ([ROADMAP](../docs/ROADMAP.md)).
