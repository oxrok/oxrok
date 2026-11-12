# oxrok for Fedora, built from the vendored source tarball of a release
# (`make vendor-src`, or the `oxrok-VERSION-vendored.tar.xz` of the GitHub
# release): every crate is in vendor/, nothing comes from the network.
# The Python decoders run on the system's CPython (-f python, through
# pyo3). This is the COPR spec: Fedora proper takes the crates from the
# rust-* packages instead of vendor/.

%bcond check 1

Name:           oxrok
Version:        0.2.0
Release:        %autorelease
Summary:        A sigrok compatible logic analyzer, multimeter and oscilloscope tool

# oxrok is GPL-3.0-or-later. The binary links the vendored crates, so the tag
# is theirs too: the output of `%%cargo_license_summary -f python`, which
# the mock build prints, one term per line, each kept as the crate states it.
# (MIT OR Apache-2.0) AND Unicode-3.0
# 0BSD OR MIT OR Apache-2.0
# Apache-2.0 AND ISC
# Apache-2.0 OR ISC OR MIT
# Apache-2.0 OR MIT
# Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT
# BSD-2-Clause OR MIT OR Apache-2.0
# BSD-3-Clause
# GPL-3.0-or-later
# GPL-3.0-or-later OR Apache-2.0
# ISC
# MIT
# MIT OR Apache-2.0
# MIT OR Apache-2.0 OR LGPL-2.1-or-later
# MIT OR GPL-3.0-only
# MIT OR Zlib OR Apache-2.0
# MPL-2.0
# Unlicense OR MIT
# Zlib OR Apache-2.0 OR MIT
License:        GPL-3.0-or-later AND ((MIT OR Apache-2.0) AND Unicode-3.0) AND (0BSD OR MIT OR Apache-2.0) AND (Apache-2.0 AND ISC) AND (Apache-2.0 OR ISC OR MIT) AND (Apache-2.0 OR MIT) AND (Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT) AND (BSD-2-Clause OR MIT OR Apache-2.0) AND BSD-3-Clause AND (GPL-3.0-or-later OR Apache-2.0) AND ISC AND MIT AND (MIT OR Apache-2.0) AND (MIT OR Apache-2.0 OR LGPL-2.1-or-later) AND (MIT OR GPL-3.0-only) AND (MIT OR Zlib OR Apache-2.0) AND MPL-2.0 AND (Unlicense OR MIT) AND (Zlib OR Apache-2.0 OR MIT)
URL:            https://oxrok.dev
# The first release this spec builds from is the one after 0.2.0: the
# 0.2.0 tarball predates contrib/'s udev rules, which %%install takes.
# Packit builds from the tarball `make vendor-src` makes of the tree.
Source0:        https://github.com/oxrok/oxrok/releases/download/v%{version}/oxrok-%{version}-vendored.tar.xz

ExclusiveArch:  %{rust_arches}

BuildRequires:  cargo-rpm-macros >= 24
BuildRequires:  systemd-rpm-macros
# pyo3 builds against the system Python and links its libpython, which
# the package then requires by soname.
BuildRequires:  python3-devel
# The Python protocol decoders, found under %%{_datadir}/libsigrokdecode.
# EPEL 10 has no libsigrokdecode: there oxrok runs without decoders until
# the user installs them.
%if 0%{?rhel}
Recommends:     libsigrokdecode
%else
Requires:       libsigrokdecode
%endif
# Firmware the drivers upload, where libsigrok keeps it.
Recommends:     sigrok-firmware-fx2lafw
Suggests:       sigrok-firmware-nonfree

%description
oxrok is a sigrok compatible logic analyzer, multimeter and oscilloscope
stack in Rust: one binary that drives the devices of libsigrok's drivers
(USB, serial, USBTMC, TCP) and runs libsigrokdecode's protocol decoders
on the system's Python. The command line follows sigrok-cli. The udev rules
in %{_udevrulesdir} give the seated user access to the USB devices.

%prep
%autosetup -n oxrok-%{version}-vendored -p1
%cargo_prep -v vendor

%build
# pyo3 builds for this Python, not whichever python3 comes first on PATH.
export PYO3_PYTHON=%{python3}
# The rustc of oxrok itself with LTO and debuginfo takes gigabytes; let the
# parallelism follow the memory the builder has.
%constrain_build -m 6144
%cargo_build -f python
%cargo_vendor_manifest
%cargo_license_summary -f python
%{cargo_license -f python} > LICENSE.dependencies

%install
export PYO3_PYTHON=%{python3}
%cargo_install -f python
install -Dpm 0644 contrib/60-oxrok.rules %{buildroot}%{_udevrulesdir}/60-oxrok.rules
install -Dpm 0644 contrib/61-oxrok-uaccess.rules %{buildroot}%{_udevrulesdir}/61-oxrok-uaccess.rules

%check
# The test suite needs the emulator and the bundled decoders; the binary
# answering is the check here.
%{buildroot}%{_bindir}/oxrok --version
%{buildroot}%{_bindir}/oxrok --udev-rules | cmp - contrib/60-oxrok.rules

%files
%license LICENSE LICENSE.dependencies cargo-vendor.txt
%doc README.md CHANGELOG.md
%{_bindir}/oxrok
%{_udevrulesdir}/60-oxrok.rules
%{_udevrulesdir}/61-oxrok-uaccess.rules

%changelog
%autochangelog
