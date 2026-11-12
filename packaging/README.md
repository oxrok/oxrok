# Packaging

What this directory holds, and how the packages relate to the release
archives of the [Releases page](https://github.com/oxrok/oxrok/releases).

## Fedora: `fedora/oxrok.spec` and `.packit.yaml`

The spec builds `oxrok` with the `python` feature: the decoders run on
the system's CPython through pyo3, and the package requires the
`libpython3.X.so` it links by soname. It builds from the vendored source
tarball, so the build needs no network and no crate from the
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

[Packit](https://packit.dev) reads `.packit.yaml` at the root and builds
in COPR:

- every commit on `main` into
  [mangelajo/oxrok](https://copr.fedorainfracloud.org/coprs/mangelajo/oxrok/),
  the repository users enable with `dnf copr enable mangelajo/oxrok`, for
  all its chroots: every active Fedora with Rawhide, CentOS Stream 10 and
  EPEL 10 (on RHEL), on x86_64 and aarch64;
- every pull request in Packit's own namespace (one COPR project per pull
  request), reporting as a check, for Rawhide and CentOS Stream 10 on
  x86_64 only: one distribution of each family, so a pull request waits
  for two builds, not twelve.

COPR takes the `main` builds only from the forge projects the project
allows (`github.com/oxrok/*` in its settings), and Packit has `admin` on
it to add the chroots of a new Fedora. The CentOS Stream 10 chroot has
no EPEL, where `cargo-rpm-macros` comes from, so `.packit.yaml` adds the
EPEL 10 repository to it.

EPEL 10 has no `libsigrokdecode`, so on RHEL and CentOS the package
recommends it instead of requiring it, and runs without the Python
decoders until they are installed.

By hand, on a Fedora machine with `mock`:

```
make vendor-src
rpmbuild -bs --define "_sourcedir $PWD/dist" --define "_srcrpmdir $PWD/dist" packaging/fedora/oxrok.spec
mock -r fedora-rawhide-x86_64 rebuild dist/oxrok-*.src.rpm
```

The rustc of `oxrok` itself, with LTO and debuginfo, takes gigabytes of
memory: `%constrain_build -m 6144` in the spec caps `-j` at one job per
6 GB (two on a 16 GB COPR builder), and `--define
'rustflags_codegen_units 16'` gets a lighter build through for checking
the spec itself.
`rpmlint -r packaging/fedora/oxrok.rpmlintrc` on the results filters the known
noise (the sigrok names, the data dependency on `libsigrokdecode`).

Fedora proper is a different build: every crate from a `rust-*` package,
no vendoring. That spec would start from
`rust2rpm`; this one is for COPR.

The `License` tag is oxrok's licence and every vendored crate's, as
`%cargo_license_summary` lists them (the spec keeps that list as a comment
above the tag); every crate of the `python` build states one.

## Later

Debian and Ubuntu packages from the static binary (`cargo-deb` or
`nfpm` on the release archive), a Flatpak for `oxrok-ui`, Homebrew and
the AUR ([ROADMAP](../docs/ROADMAP.md)).
