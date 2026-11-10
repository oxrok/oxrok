# oxrok for Fedora, built from the vendored source tarball of a release
# (`make vendor-src`, or the `oxrok-VERSION-vendored.tar.xz` of the GitHub
# release): every crate is in vendor/, nothing comes from the network.
# This is the COPR spec. Fedora proper takes crates from the rust-*
# packages and would run the Python decoders on CPython (-f python).

%bcond check 1

Name:           oxrok
Version:        0.2.0
Release:        %autorelease
Summary:        A sigrok compatible logic analyzer, multimeter and oscilloscope tool

# oxrok is GPL-3.0-or-later. The binary links the vendored crates, so the tag
# is theirs too: the output of `%%cargo_license_summary -f rustpython`, which
# the mock build prints, one term per line, each kept as the crate states it.
# Two crates carry no licence metadata (mt19937, rustpython-pylib) and are
# not in the tag; packaging/README.md says what a Fedora review would need.
# (MIT OR Apache-2.0) AND Unicode-3.0
# (MIT OR Apache-2.0) AND Unicode-DFS-2016
# 0BSD OR MIT OR Apache-2.0
# Apache-2.0
# Apache-2.0 OR BSL-1.0
# Apache-2.0 OR MIT
# Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT
# BSD-2-Clause OR Apache-2.0 OR MIT
# BSD-2-Clause OR MIT OR Apache-2.0
# BSD-2-Clause-Views
# BSD-3-Clause OR MIT OR Apache-2.0
# BSL-1.0
# CC0-1.0
# CC0-1.0 OR MIT-0 OR Apache-2.0
# GPL-3.0-or-later
# GPL-3.0-or-later OR Apache-2.0
# ISC
# LGPL-3.0-only
# MIT
# MIT OR Apache-2.0
# MIT OR Apache-2.0 OR LGPL-2.1-or-later
# MIT OR GPL-3.0-only
# MIT OR Zlib OR Apache-2.0
# MPL-2.0
# PSF-2.0
# Unicode-3.0
# Unlicense OR MIT
# Zlib
# Zlib OR Apache-2.0 OR MIT
# bzip2-1.0.6
License:        GPL-3.0-or-later AND ((MIT OR Apache-2.0) AND Unicode-3.0) AND ((MIT OR Apache-2.0) AND Unicode-DFS-2016) AND (0BSD OR MIT OR Apache-2.0) AND Apache-2.0 AND (Apache-2.0 OR BSL-1.0) AND (Apache-2.0 OR MIT) AND (Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT) AND (BSD-2-Clause OR Apache-2.0 OR MIT) AND (BSD-2-Clause OR MIT OR Apache-2.0) AND BSD-2-Clause-Views AND (BSD-3-Clause OR MIT OR Apache-2.0) AND BSL-1.0 AND CC0-1.0 AND (CC0-1.0 OR MIT-0 OR Apache-2.0) AND (GPL-3.0-or-later OR Apache-2.0) AND ISC AND LGPL-3.0-only AND MIT AND (MIT OR Apache-2.0) AND (MIT OR Apache-2.0 OR LGPL-2.1-or-later) AND (MIT OR GPL-3.0-only) AND (MIT OR Zlib OR Apache-2.0) AND MPL-2.0 AND PSF-2.0 AND Unicode-3.0 AND (Unlicense OR MIT) AND Zlib AND (Zlib OR Apache-2.0 OR MIT) AND bzip2-1.0.6
URL:            https://oxrok.dev
Source0:        https://github.com/oxrok/oxrok/releases/download/v%{version}/oxrok-%{version}-vendored.tar.xz

ExclusiveArch:  %{rust_arches}

BuildRequires:  cargo-rpm-macros >= 24
BuildRequires:  systemd-rpm-macros
# RustPython's ctypes module links the system libffi.
BuildRequires:  libffi-devel
# The Python protocol decoders, found under %%{_datadir}/libsigrokdecode.
Requires:       libsigrokdecode
# Firmware the drivers upload, where libsigrok keeps it.
Recommends:     sigrok-firmware-fx2lafw
Suggests:       sigrok-firmware-nonfree

%description
oxrok is a sigrok compatible logic analyzer, multimeter and oscilloscope
stack in Rust: one binary that drives the devices of libsigrok's drivers
(USB, serial, USBTMC, TCP) and runs libsigrokdecode's protocol decoders
on a built-in Python. The command line follows sigrok-cli. The udev rules
in %{_udevrulesdir} give the seated user access to the USB devices.

%prep
%autosetup -n oxrok-%{version}-vendored -p1
%cargo_prep -v vendor

%build
# One rustc of the RustPython crates or of oxrok itself with LTO and debuginfo
# passes 5 GB; let the parallelism follow the memory the builder has.
%constrain_build -m 6144
%cargo_build -f rustpython
%cargo_vendor_manifest
%cargo_license_summary -f rustpython
%{cargo_license -f rustpython} > LICENSE.dependencies

%install
%cargo_install -f rustpython
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
