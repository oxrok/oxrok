//! `oxrok firmware`, and the offer to fetch firmware a run missed.
//!
//! Vendor firmware is never downloaded without asking: a run that misses
//! a file asks on a terminal, and elsewhere prints the command that
//! fetches it. Files only the user can get come with instructions and
//! `oxrok firmware import`.

use crate::cli::args::{FirmwareAction, FirmwareArgs};
use oxrok::core::firmware::{self, fetch};
use oxrok::core::{Error, FirmwareKind, Registry, Result};
use std::cell::Cell;
use std::io::{BufRead, IsTerminal, Write};
use std::path::{Path, PathBuf};

/// `oxrok firmware …`.
pub fn run(registry: &Registry, args: &FirmwareArgs) -> Result<()> {
    match &args.action {
        FirmwareAction::List => {
            list(registry);
            Ok(())
        }
        FirmwareAction::Info { what } => {
            for p in resolve(registry, what)? {
                info(p);
            }
            Ok(())
        }
        FirmwareAction::Fetch { what, all } => {
            let dest = dest(args)?;
            let packages: Vec<&'static fetch::Package> = if *all {
                fetch::packages().iter().filter(|p| p.downloadable()).collect()
            } else {
                let mut v = Vec::new();
                for w in what {
                    for p in resolve(registry, w)? {
                        if !v.iter().any(|q: &&fetch::Package| q.id == p.id) {
                            v.push(p);
                        }
                    }
                }
                v
            };
            let mut failed = 0;
            for p in packages {
                if !p.downloadable() {
                    eprintln!("{} can't be downloaded.", p.title);
                    print_manual(p);
                    failed += 1;
                } else if let Err(e) = {
                    warn_unpacker(p);
                    fetch_one(p, &dest)
                } {
                    eprintln!("{}: {e}", p.id);
                    failed += 1;
                }
            }
            if failed > 0 {
                return Err(Error::Arg(format!("{failed} firmware package(s) not installed.")));
            }
            Ok(())
        }
        FirmwareAction::Import { what, paths } => {
            let dest = dest(args)?;
            let packages = resolve(registry, what)?;
            let [p] = packages.as_slice() else {
                return Err(Error::Arg(format!(
                    "{what} needs firmware from several packages; import them one at a time: {}",
                    packages.iter().map(|p| p.id).collect::<Vec<_>>().join(", ")
                )));
            };
            if fetch::unpacker().is_none()
                && let Some(f) = paths.iter().find(|f| fetch::file_needs_unpacker(f))
            {
                eprintln!(
                    "Opening {} needs unar, 7-Zip or bsdtar, and none is installed.",
                    f.display()
                );
            }
            let installed =
                fetch::import(p, paths, &dest, &Printer::default()).map_err(fetch_err)?;
            report(&installed);
            Ok(())
        }
    }
}

/// After a run: offers to fetch the firmware drivers missed.
pub fn offer_missing() {
    let interactive = std::io::stdin().is_terminal() && std::io::stderr().is_terminal();
    for missing in firmware::take_missing_grouped() {
        let files = missing.files.join(", ");
        let Some(p) = missing.package else {
            eprintln!("oxrok: no known source for firmware {files}.");
            continue;
        };
        if !interactive {
            let how = if p.downloadable() { "fetch" } else { "info" };
            eprintln!("oxrok: missing firmware {files}; see `oxrok firmware {how} {}`.", p.id);
            continue;
        }
        eprintln!("\nMissing firmware: {files} ({}).", p.title);
        if !p.downloadable() {
            print_manual(p);
            continue;
        }
        let Some(dest) = firmware::install_dir() else {
            eprintln!(
                "No home directory to install into; use `oxrok firmware fetch --dir DIR {}`.",
                p.id
            );
            continue;
        };
        warn_unpacker(p);
        if ask(&format!(
            "Download it ({} from {}) and install it into {}? [Y/n] ",
            human(p.download_size()),
            p.hosts().join(", "),
            dest.display()
        )) {
            match fetch_one(p, &dest) {
                Ok(()) => eprintln!("Run the command again to use it."),
                Err(e) => eprintln!("{e}"),
            }
        }
    }
}

/// Says so when `p`'s download needs an unpacker that is not installed.
fn warn_unpacker(p: &fetch::Package) {
    if p.needs_unpacker() && fetch::unpacker().is_none() {
        eprintln!("Unpacking {} needs unar, 7-Zip or bsdtar, and none is installed.", p.id);
    }
}

fn fetch_one(p: &fetch::Package, dest: &Path) -> Result<()> {
    let printer = Printer::default();
    let opts = fetch::Options { progress: &printer, ..fetch::Options::default() };
    let installed = fetch::fetch(p, dest, &opts).map_err(fetch_err)?;
    printer.end_line();
    report(&installed);
    Ok(())
}

fn report(installed: &fetch::Installed) {
    for w in &installed.warnings {
        eprintln!("warning: {w}");
    }
    let dir = installed.files.first().and_then(|f| f.parent());
    eprintln!(
        "Installed {} file{} into {}.",
        installed.files.len(),
        if installed.files.len() == 1 { "" } else { "s" },
        dir.map_or_else(String::new, |d| d.display().to_string())
    );
}

fn print_manual(p: &fetch::Package) {
    if let Some(m) = p.manual {
        eprintln!("{}", m.instructions());
    }
    eprintln!("Then run: oxrok firmware import {} PATH...", p.id);
}

/// `oxrok firmware info`.
fn info(p: &fetch::Package) {
    println!("{} ({})", p.title, p.id);
    println!("  files: {}", p.files.join(", "));
    if p.downloadable() {
        println!(
            "  download: {} from {}: `oxrok firmware fetch {}`",
            human(p.download_size()),
            p.hosts().join(", "),
            p.id
        );
    }
    if let Some(m) = p.manual {
        let lead = if p.downloadable() { "or provide it yourself" } else { "provide it yourself" };
        println!("  {lead}: `oxrok firmware import {} PATH...`", p.id);
        for line in m.instructions().lines() {
            println!("    {line}");
        }
    }
    if let Some(note) = p.note {
        println!("  note: {note}");
    }
}

/// `oxrok firmware list`: every package drivers need, and whether its
/// files are installed.
fn list(registry: &Registry) {
    let mut needed: Vec<&'static fetch::Package> = Vec::new();
    for d in registry.drivers() {
        for dep in d.firmware().into_iter().filter(|d| d.kind == FirmwareKind::Upload) {
            for f in &dep.files {
                if let Some(p) = firmware::source_of(f)
                    && !needed.iter().any(|q| q.id == p.id)
                {
                    needed.push(p);
                }
            }
        }
    }
    needed.sort_by_key(|p| p.id);
    println!("{:<24} {:<10} {:<20} DESCRIPTION", "PACKAGE", "INSTALLED", "SOURCE");
    for p in needed {
        let st = firmware::package_status(p);
        let status = if st.complete() {
            "yes".to_owned()
        } else {
            format!("{}/{}", st.found.len(), p.files.len())
        };
        let source = if p.downloadable() {
            format!("download {}", human(p.download_size()))
        } else {
            "you".to_owned()
        };
        println!("{:<24} {status:<10} {source:<20} {}", p.id, p.title);
    }
    if let Some(dir) = firmware::install_dir() {
        println!("\nFetched firmware goes to {}.", dir.display());
    }
}

/// The packages `what` names: a package id, a firmware file, or a
/// driver (every package its files come from).
fn resolve(registry: &Registry, what: &str) -> Result<Vec<&'static fetch::Package>> {
    if let Some(p) = fetch::package(what).or_else(|| firmware::source_of(what)) {
        return Ok(vec![p]);
    }
    if let Some(d) = registry.drivers().find(|d| d.id() == what) {
        let mut v: Vec<&'static fetch::Package> = Vec::new();
        for dep in d.firmware() {
            for f in &dep.files {
                if let Some(p) = firmware::source_of(f)
                    && !v.iter().any(|q| q.id == p.id)
                {
                    v.push(p);
                }
            }
        }
        if v.is_empty() {
            return Err(Error::Arg(format!("Driver {what} needs no firmware oxrok can fetch.")));
        }
        return Ok(v);
    }
    Err(Error::Arg(format!(
        "Unknown firmware '{what}': give a package id (see `oxrok firmware list`), a firmware file name or a driver id."
    )))
}

fn dest(args: &FirmwareArgs) -> Result<PathBuf> {
    args.dir
        .clone()
        .or_else(firmware::install_dir)
        .ok_or_else(|| Error::Arg("No home directory to install into; give --dir DIR.".into()))
}

fn fetch_err(e: fetch::Error) -> Error {
    match e {
        fetch::Error::Io(e) => Error::Io(e),
        other => Error::Arg(other.to_string()),
    }
}

/// Asks yes or no on the terminal; yes unless the answer starts with n.
fn ask(question: &str) -> bool {
    eprint!("{question}");
    let _ = std::io::stderr().flush();
    let mut line = String::new();
    if std::io::stdin().lock().read_line(&mut line).unwrap_or(0) == 0 {
        eprintln!();
        return false;
    }
    !line.trim().to_ascii_lowercase().starts_with('n')
}

fn human(bytes: u64) -> String {
    if bytes >= 1 << 20 {
        format!("{:.1} MB", bytes as f64 / f64::from(1u32 << 20))
    } else {
        format!("{} kB", bytes.div_ceil(1024))
    }
}

/// Prints steps, and on a terminal a download's progress on one line.
#[derive(Default)]
struct Printer {
    /// A progress line is open.
    open: Cell<bool>,
    /// The last percentage printed.
    shown: Cell<u64>,
}

impl Printer {
    fn end_line(&self) {
        if self.open.replace(false) {
            eprintln!();
        }
    }
}

impl fetch::Progress for Printer {
    fn message(&self, msg: &str) {
        self.end_line();
        if !msg.starts_with("Installed ") {
            eprintln!("{msg}");
        }
    }

    fn download(&self, _name: &str, done: u64, total: u64) {
        if !std::io::stderr().is_terminal() || total == 0 {
            return;
        }
        let pct = done * 100 / total;
        if pct != self.shown.get() || !self.open.get() {
            self.shown.set(pct);
            self.open.set(true);
            eprint!("\r  {pct:3}% of {}", human(total));
        }
    }
}
