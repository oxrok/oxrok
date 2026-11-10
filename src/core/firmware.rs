// Counterpart in sigrok:
//   libsigrok/src/resource.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.
#![allow(clippy::assert_is_empty)]

//! Finding firmware and bitstream files.
//!
//! The directories, first hit wins, follow libsigrok's
//! `resource.c` with oxrok's own names ahead of sigrok's at each step
//! (the differences are listed below):
//!
//! 1. `OXROK_FIRMWARE_DIR`, a colon (on Windows semicolon) separated
//!    list, then `OXROK_FIRMWARE_INSTALL_DIR`, where fetched firmware
//!    goes when it is set ([`install_dir`]);
//! 2. `SIGROK_FIRMWARE_DIR`, one directory, as libsigrok takes it;
//! 3. `SIGROK_FIRMWARE_PATH`, a separated list, empty items skipped;
//! 4. the user's data directory (`XDG_DATA_HOME`, else
//!    `~/.local/share`): `oxrok-firmware`, then `sigrok-firmware`;
//! 5. the system's data directories (`XDG_DATA_DIRS`, else
//!    `/usr/local/share`, `/opt/homebrew/share` and `/usr/share`), each
//!    with `oxrok-firmware` then `sigrok-firmware`.
//!
//! Unlike `resource.c`, there is no compile-time `FIRMWARE_DIR` (oxrok
//! has no install prefix), `/opt/homebrew/share` is in the default
//! system list, and an empty variable is skipped, where libsigrok keeps
//! an empty `SIGROK_FIRMWARE_DIR` as a lookup in the current directory.
//!
//! An existing sigrok installation keeps working, and a variable set
//! for sigrok-cli (the emulator harness points `SIGROK_FIRMWARE_DIR` at
//! its firmware stand-ins) reaches oxrok the same way.

use crate::core::error::{Error, Result};
use std::ffi::OsString;
use std::path::PathBuf;

/// The directories searched, in order.
pub fn search_dirs() -> Vec<PathBuf> {
    search_dirs_in(|var| std::env::var_os(var))
}

/// [`search_dirs`] with the environment given by `env`.
pub fn search_dirs_in(env: impl Fn(&str) -> Option<OsString>) -> Vec<PathBuf> {
    let set = |var: &str| env(var).filter(|v| !v.is_empty());
    let mut dirs = Vec::new();
    if let Some(v) = set("OXROK_FIRMWARE_DIR") {
        dirs.extend(std::env::split_paths(&v).filter(|p| !p.as_os_str().is_empty()));
    }
    if let Some(v) = set("OXROK_FIRMWARE_INSTALL_DIR") {
        dirs.push(PathBuf::from(v));
    }
    if let Some(v) = set("SIGROK_FIRMWARE_DIR") {
        dirs.push(PathBuf::from(v));
    }
    if let Some(v) = set("SIGROK_FIRMWARE_PATH") {
        dirs.extend(std::env::split_paths(&v).filter(|p| !p.as_os_str().is_empty()));
    }
    let user = user_data_dir(&set);
    let system: Vec<PathBuf> = match set("XDG_DATA_DIRS") {
        Some(v) => std::env::split_paths(&v).filter(|p| !p.as_os_str().is_empty()).collect(),
        None => ["/usr/local/share", "/opt/homebrew/share", "/usr/share"]
            .iter()
            .map(PathBuf::from)
            .collect(),
    };
    for base in user.into_iter().chain(system) {
        for sub in ["oxrok-firmware", "sigrok-firmware"] {
            let d = base.join(sub);
            if !dirs.contains(&d) {
                dirs.push(d);
            }
        }
    }
    dirs
}

/// The first directory holding `name`.
pub fn find_file(name: &str) -> Option<PathBuf> {
    search_dirs().into_iter().map(|d| d.join(name)).find(|p| p.is_file())
}

/// Reads `name` from the first directory that has it, or fails with
/// [`Error::Firmware`] naming the file and `hint`. A miss is noted for
/// [`take_missing`].
pub fn read(name: &str, hint: &str) -> Result<Vec<u8>> {
    let Some(path) = find_file(name) else {
        note_missing(name);
        return Err(Error::Firmware(format!("{name} ({hint})")));
    };
    log::debug!("Loading {} from {}.", name, path.display());
    Ok(std::fs::read(path)?)
}

/// Fetching firmware: the registry of where every file comes from, and
/// the downloaders and extractors (the `oxrok-firmware-fetch` crate).
pub use oxrok_firmware_fetch as fetch;

/// Files drivers needed and did not find, in order, each once.
static MISSING: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());

/// Notes that a driver needed firmware file `name` and did not find it.
/// [`read`] does this itself; a driver that looks files up with
/// [`find_file`] calls it when the file it needs is not there (not for
/// alternatives it merely probes).
pub fn note_missing(name: &str) {
    let mut missing = MISSING.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    if !missing.iter().any(|m| m == name) {
        missing.push(name.to_owned());
    }
}

/// The files noted missing since the last call, for a front end to offer
/// fetching them. Errors naming the file are often wrapped by the time
/// they reach the front end (an acquisition's failure is a string), so
/// misses are collected here instead.
pub fn take_missing() -> Vec<String> {
    std::mem::take(&mut *MISSING.lock().unwrap_or_else(std::sync::PoisonError::into_inner))
}

/// Where fetched firmware is installed: `OXROK_FIRMWARE_INSTALL_DIR` if
/// it is set, else the user's `oxrok-firmware` data directory. Both are
/// in [`search_dirs`], so drivers find what was installed.
/// `OXROK_FIRMWARE_DIR` is not used: an application bundle points it at
/// its own read-only firmware.
pub fn install_dir() -> Option<PathBuf> {
    install_dir_in(|var| std::env::var_os(var))
}

/// [`install_dir`] with the environment given by `env`.
pub fn install_dir_in(env: impl Fn(&str) -> Option<OsString>) -> Option<PathBuf> {
    let set = |var: &str| env(var).filter(|v| !v.is_empty());
    if let Some(dir) = set("OXROK_FIRMWARE_INSTALL_DIR") {
        return Some(PathBuf::from(dir));
    }
    Some(user_data_dir(&set)?.join("oxrok-firmware"))
}

/// `XDG_DATA_HOME`, else `~/.local/share`.
fn user_data_dir(set: &impl Fn(&str) -> Option<OsString>) -> Option<PathBuf> {
    set("XDG_DATA_HOME").map(PathBuf::from).or_else(|| {
        set("HOME").or_else(|| set("USERPROFILE")).map(|h| PathBuf::from(h).join(".local/share"))
    })
}

/// The package that provides firmware file `name`, if oxrok knows one.
pub fn source_of(name: &str) -> Option<&'static fetch::Package> {
    fetch::package_for(name)
}

/// Firmware drivers needed and did not find, grouped by where it comes
/// from: one entry per dialog a front end shows.
#[derive(Clone, Debug)]
pub struct Missing {
    /// The package providing the files; `None` for files oxrok knows no
    /// source for.
    pub package: Option<&'static fetch::Package>,
    /// The files drivers asked for, in order.
    pub files: Vec<String>,
}

impl Missing {
    /// Whether the files can be downloaded, as opposed to provided by
    /// the user with [`fetch::import`].
    pub fn downloadable(&self) -> bool {
        self.package.is_some_and(fetch::Package::downloadable)
    }
}

/// Groups `files` by the package that provides them, packages in the
/// order their first file comes.
pub fn group_missing(files: &[String]) -> Vec<Missing> {
    let mut out: Vec<Missing> = Vec::new();
    for file in files {
        let package = source_of(file);
        let same = |m: &&mut Missing| match (m.package, package) {
            (Some(a), Some(b)) => a.id == b.id,
            _ => false,
        };
        match out.iter_mut().find(|m| same(m)) {
            Some(m) => m.files.push(file.clone()),
            None => out.push(Missing { package, files: vec![file.clone()] }),
        }
    }
    out
}

/// [`take_missing`], grouped with [`group_missing`].
pub fn take_missing_grouped() -> Vec<Missing> {
    group_missing(&take_missing())
}

/// Which of a package's files the search finds.
#[derive(Clone, Debug, Default)]
pub struct PackageStatus {
    /// The files found, where they are.
    pub found: Vec<PathBuf>,
    /// The files not found.
    pub missing: Vec<&'static str>,
}

impl PackageStatus {
    /// Every file is there.
    pub fn complete(&self) -> bool {
        self.missing.is_empty()
    }
}

/// Which of `pkg`'s files are installed, anywhere [`search_dirs`] looks.
pub fn package_status(pkg: &fetch::Package) -> PackageStatus {
    let mut status = PackageStatus::default();
    for f in pkg.files {
        match find_file(f) {
            Some(p) => status.found.push(p),
            None => status.missing.push(f),
        }
    }
    status
}

/// How a driver's firmware gets onto the device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FirmwareKind {
    /// The driver uploads it on open.
    Upload,
    /// The device must already be running it.
    PreFlashed,
}

/// A firmware or bitstream dependency of a driver: the files looked up
/// in the firmware directories and where to get them. Drivers declare
/// theirs with [`crate::core::Driver::firmware`], which `--list-firmware`
/// and the site listing read.
#[derive(Clone, Debug)]
pub struct FirmwareDependency {
    /// How it gets onto the device.
    pub kind: FirmwareKind,
    /// The file names looked up in the firmware directories; none for
    /// [`FirmwareKind::PreFlashed`].
    pub files: Vec<String>,
    /// Where to get the files: a package, an extractor or a vendor
    /// installer.
    pub source: &'static str,
    /// An extra word, e.g. which file a setting chooses.
    pub note: Option<&'static str>,
}

impl FirmwareDependency {
    /// Files the driver uploads on open. Names several of its devices
    /// share are listed once, in the order first seen.
    pub fn upload(files: impl IntoIterator<Item: Into<String>>, source: &'static str) -> Self {
        let mut seen = std::collections::BTreeSet::new();
        let mut unique = Vec::new();
        for f in files.into_iter().map(Into::into) {
            if seen.insert(f.clone()) {
                unique.push(f);
            }
        }
        FirmwareDependency { kind: FirmwareKind::Upload, files: unique, source, note: None }
    }

    /// Firmware the device must already run; the driver uploads none.
    pub fn pre_flashed(source: &'static str) -> Self {
        FirmwareDependency { kind: FirmwareKind::PreFlashed, files: Vec::new(), source, note: None }
    }

    /// Adds a note.
    #[must_use]
    pub fn note(mut self, note: &'static str) -> Self {
        self.note = Some(note);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dirs(vars: &[(&str, &str)]) -> Vec<PathBuf> {
        search_dirs_in(|name| vars.iter().find(|(k, _)| *k == name).map(|(_, v)| v.into()))
    }

    /// A list in the platform's separator.
    fn list(items: &[&str]) -> String {
        std::env::join_paths(items).unwrap().into_string().unwrap()
    }

    #[test]
    fn follows_libsigrok_precedence() {
        let ox = list(&["/ox/a", "/ox/b"]);
        let path = list(&["/sr/p1", "", "/sr/p2"]);
        let d = dirs(&[
            ("HOME", "/home/u"),
            ("OXROK_FIRMWARE_DIR", &ox),
            ("SIGROK_FIRMWARE_DIR", "/sr/dir"),
            ("SIGROK_FIRMWARE_PATH", &path),
        ]);
        let want: Vec<PathBuf> = [
            "/ox/a",
            "/ox/b",
            "/sr/dir",
            "/sr/p1",
            "/sr/p2",
            "/home/u/.local/share/oxrok-firmware",
            "/home/u/.local/share/sigrok-firmware",
            "/usr/local/share/oxrok-firmware",
            "/usr/local/share/sigrok-firmware",
            "/opt/homebrew/share/oxrok-firmware",
            "/opt/homebrew/share/sigrok-firmware",
            "/usr/share/oxrok-firmware",
            "/usr/share/sigrok-firmware",
        ]
        .iter()
        .map(PathBuf::from)
        .collect();
        assert_eq!(d, want);
    }

    #[test]
    fn sigrok_firmware_dir_is_one_directory() {
        // libsigrok does not split it; neither does oxrok.
        let both = list(&["/a", "/b"]);
        let d = dirs(&[("SIGROK_FIRMWARE_DIR", &both)]);
        assert_eq!(d[0], PathBuf::from(&both));
    }

    #[test]
    fn honours_the_xdg_directories() {
        let sys = list(&["/sys1", "/sys2"]);
        let d = dirs(&[
            ("HOME", "/home/u"),
            ("XDG_DATA_HOME", "/data"),
            ("XDG_DATA_DIRS", &sys),
            ("SIGROK_FIRMWARE_DIR", ""),
        ]);
        let want: Vec<PathBuf> = [
            "/data/oxrok-firmware",
            "/data/sigrok-firmware",
            "/sys1/oxrok-firmware",
            "/sys1/sigrok-firmware",
            "/sys2/oxrok-firmware",
            "/sys2/sigrok-firmware",
        ]
        .iter()
        .map(PathBuf::from)
        .collect();
        assert_eq!(d, want, "empty variables are ignored");
    }

    #[test]
    fn installs_where_the_search_looks() {
        let install = |vars: &[(&str, &str)]| {
            install_dir_in(|name| vars.iter().find(|(k, _)| *k == name).map(|(_, v)| v.into()))
        };
        // A bundle's OXROK_FIRMWARE_DIR is read-only: not installed into.
        let home = [("HOME", "/home/u"), ("OXROK_FIRMWARE_DIR", "/app/firmware")];
        assert_eq!(install(&home), Some(PathBuf::from("/home/u/.local/share/oxrok-firmware")));
        let vars = [("HOME", "/home/u"), ("XDG_DATA_HOME", "/data")];
        let want = install(&vars).unwrap();
        assert!(dirs(&vars).contains(&want), "{want:?} is searched");
        let own = [("HOME", "/home/u"), ("OXROK_FIRMWARE_INSTALL_DIR", "/fw")];
        assert_eq!(install(&own), Some(PathBuf::from("/fw")));
        assert!(dirs(&own).contains(&PathBuf::from("/fw")), "the override is searched");
        assert_eq!(install(&[]), None);
    }

    #[test]
    fn missing_files_group_by_package() {
        let files: Vec<String> = [
            "kingst-la-01a2.fw",
            "nobody-knows.fw",
            "kingst-la2016a1-fpga.bitstream",
            "intronix-logicport-timing-sdr.rbf",
        ]
        .map(String::from)
        .to_vec();
        let groups = group_missing(&files);
        let ids: Vec<Option<&str>> = groups.iter().map(|g| g.package.map(|p| p.id)).collect();
        assert_eq!(ids, vec![Some("kingst-la"), None, Some("intronix-logicport")]);
        assert_eq!(groups[0].files, vec!["kingst-la-01a2.fw", "kingst-la2016a1-fpga.bitstream"]);
        assert!(groups[0].downloadable() && !groups[1].downloadable());
    }

    #[test]
    fn misses_are_noted_once_and_taken() {
        // Other tests may note misses too; look only for these names.
        note_missing("test-missing-a.fw");
        note_missing("test-missing-b.fw");
        note_missing("test-missing-a.fw");
        let taken: Vec<String> =
            take_missing().into_iter().filter(|m| m.starts_with("test-missing-")).collect();
        assert_eq!(taken, vec!["test-missing-a.fw", "test-missing-b.fw"]);
        assert!(!take_missing().iter().any(|m| m.starts_with("test-missing-")));
    }

    #[test]
    fn upload_lists_shared_names_once_in_order() {
        let d = FirmwareDependency::upload(["a.fw", "b.fw", "a.fw", "c.fw", "b.fw"], "a package");
        assert_eq!(d.files, vec!["a.fw", "b.fw", "c.fw"]);
        assert_eq!(d.kind, FirmwareKind::Upload);
        assert_eq!(d.source, "a package");
        assert_eq!(d.note, None);
    }

    #[test]
    fn pre_flashed_has_no_files() {
        let d = FirmwareDependency::pre_flashed("flashed to the board");
        assert_eq!(d.kind, FirmwareKind::PreFlashed);
        assert!(d.files.is_empty());
        assert_eq!(d.source, "flashed to the board");
    }

    #[test]
    fn note_replaces_the_absent_one() {
        let d = FirmwareDependency::upload(["a.fw"], "a package").note("chosen by a setting");
        assert_eq!(d.note, Some("chosen by a setting"));
    }
}
