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

[Packit](https://packit.dev) reads `.packit.yaml` at the root and, once
the Packit GitHub App is installed on the repository and the
organisation is approved by the Packit team, builds every pull request
in COPR for the latest stable Fedora and Rawhide, reporting as a check.
A COPR project for `main` (the commented job) needs a COPR owner: the
`@oxrok` group or a user, created at copr.fedorainfracloud.org first.

By hand, on a Fedora machine with `mock`:

```
make vendor-src
rpmbuild -bs --define "_sourcedir $PWD/dist" --define "_srcrpmdir $PWD/dist" packaging/fedora/oxrok.spec
mock -r fedora-rawhide-x86_64 rebuild dist/oxrok-*.src.rpm
```

The build is heavy: every RustPython crate compiles with LTO and debuginfo,
about an hour and a half on 16 cores. `%limit_build -m 4096` in the spec caps
`-j` at one job per 4 GB of available memory, so a 16 GB COPR builder runs four
jobs; on a shared box run mock while nothing else compiles.

Fedora proper is a different build: every crate from a `rust-*` package,
no vendoring, and the decoders on CPython (`-f python`), since the pinned
RustPython crates are not in Fedora. That spec would start from
`rust2rpm`; this one is for COPR.

## Later

Debian and Ubuntu packages from the static binary (`cargo-deb` or
`nfpm` on the release archive), a Flatpak for `oxrok-ui`, Homebrew and
the AUR ([ROADMAP](../docs/ROADMAP.md)).
