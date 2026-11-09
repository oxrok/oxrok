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

# oxrok itself; the crates it bundles are listed in LICENSE.dependencies.
License:        GPL-3.0-or-later
URL:            https://oxrok.dev
Source0:        https://github.com/oxrok/oxrok/releases/download/v%{version}/oxrok-%{version}-vendored.tar.xz

ExclusiveArch:  %{rust_arches}

BuildRequires:  cargo-rpm-macros >= 24
BuildRequires:  systemd-rpm-macros
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
