// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! `oxrok-emu`: serves a device model over USB/IP and, on Linux with
//! root, attaches it to the machine through `vhci-hcd`, on a
//! [`vusb::session`].
//!
//! ```text
//! oxrok-emu list
//! sudo oxrok-emu serve usbrelay --attach --open-devices --log tx.log --control /tmp/emu.sock
//! vusb-ctl -s /tmp/emu.sock section name=run
//! sudo oxrok-emu cleanup
//! ```
//!
//! While serving, `vusb-ctl` drives the run through the control socket
//! (vusb's `docs/control.md`), and `quit` or a termination signal
//! detaches the device and exits. Standard output tells what happened
//! to the device, before the reply to any command taken in after it:
//! `READY port=N busid=B usb=BUS.ADDR usbfs=/dev/bus/usb/BBB/DDD
//! nodes=...` once the kernel has configured the device,
//! `RENUMERATED port=...` with the same fields each time it is attached
//! again after a renumeration or a `reset` that changed its descriptors,
//! `REPLUGGED port=...` when an unplugged device came back, and
//! `EVENT NAME` when an armed event fired. The exit status is 2 when any
//! violation was logged. With `--while-alive PID` the server also ends
//! once that process is gone, so a harness that is killed leaves no
//! server behind.
//!
//! The log (`--log`) is the transaction log of `tests/emu`, [`TxLog`]:
//! vusb's fixture format, under the rules of the model. In it an armed
//! event shows as `unplug`, or as the name of the mark; `tools/diffcli.sh`
//! arms `mark interrupt` for an interrupt of the tool under test, which
//! it delivers itself.
//!
//! Run under a model's `STANDIN` name (a link to this binary), `oxrok-emu` is
//! that model's stand-in for a program a driver runs, and takes that program's
//! arguments (`docs/USB.md`, "A program as the model").
//!
//! With `--tcp ADDRESS` the model is served on a socket too (`shells::tcp`): its
//! port is on a line `TCP port=N` on standard output, before anything else, and
//! without `--attach` a `READY` line with no device follows it.
//!
//! With `--sysfs DIR` (Linux) the model is a tree of files in `DIR` instead
//! (`shells::sysfs`), which the tools see in a mount namespace; nothing is
//! attached, and a `READY` line with no device says the tree is there.
//!
//! `cleanup` detaches the `vhci-hcd` ports a crashed run left attached:
//! those whose device announced an oxrok-emu `devid` (bus `0x6f78`), or
//! with `--all` every attached port. A running session's port carries
//! that `devid` too, so `cleanup` takes the session lock
//! (`/tmp/oxrok-emu.lock`, `--lock PATH`) and refuses while another
//! session holds it; a session cleaning up under its own lock says so
//! with `--locked`, and `--force` cleans up regardless.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use clap::{Parser, Subcommand};
use oxrok_emu::models;
use oxrok_emu::shells::tcp;
use oxrok_emu::txlog::{self, TxLog};
use serde_json::Value;
use vusb::attach::vhci;
use vusb::control::{Address, ListenOptions};
use vusb::observer::SharedObserver;
use vusb::session::{Builder, Stopper};
use vusb::{Error, Result, options};

#[derive(Parser)]
#[command(name = "oxrok-emu", about = "USB/IP device emulator for driver checks without hardware")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Lists the device models.
    List,
    /// Serves a model, and attaches it with --attach.
    Serve(Serve),
    /// Detaches the ports left attached by an oxrok-emu that died (root).
    Cleanup {
        /// Every attached port, whoever attached it.
        #[arg(long)]
        all: bool,
        /// The session lock that `tools/diffcli.sh` holds for a session.
        #[arg(long, default_value = DEFAULT_LOCK)]
        lock: PathBuf,
        /// The caller holds the lock already (a session cleaning up).
        #[arg(long)]
        locked: bool,
        /// Detach even while another session holds the lock.
        #[arg(long, conflicts_with = "locked")]
        force: bool,
    },
}

#[derive(Parser)]
struct Serve {
    /// The model (see `list`).
    model: String,
    /// A model option, KEY=VALUE.
    #[arg(short = 'o', long = "option", value_name = "KEY=VALUE")]
    options: Vec<String>,
    /// The address to listen on; port 0 picks a free one.
    #[arg(long, default_value = "127.0.0.1:3240")]
    listen: String,
    /// Attaches the device through vhci-hcd (Linux, root).
    #[arg(long)]
    attach: bool,
    /// After attaching, makes the device nodes world accessible.
    #[arg(long)]
    open_devices: bool,
    /// The transaction log file.
    #[arg(long)]
    log: Option<PathBuf>,
    /// Logs events (connections, held transfers, unlinks) too.
    #[arg(short, long)]
    verbose: bool,
    /// The control socket, for vusb-ctl: a path, or host:port on loopback.
    #[arg(long)]
    control: Option<String>,
    /// Serves the model's `build_tcp` on this address too (port 0 picks a free
    /// one), and prints `TCP port=N`.
    #[arg(long, value_name = "ADDRESS")]
    tcp: Option<String>,
    /// Serves the model's `build_sysfs` as a tree of files in this directory,
    /// emptied first and removed at the end, and attaches nothing (Linux).
    #[arg(long, value_name = "DIR", conflicts_with_all = ["attach", "tcp"])]
    sysfs: Option<PathBuf>,
    /// Ends the session once this process is gone (Linux).
    #[arg(long, value_name = "PID")]
    while_alive: Option<u32>,
}

/// The bus number the server announces; with the device number it
/// forms the `devid` in the controller's `status`, by which `cleanup`
/// recognises the ports oxrok-emu attached (`0x6f78` is "ox").
const BUSNUM: u32 = 0x6f78;

/// The session lock `tools/diffcli.sh` takes (`EMU_LOCK`).
const DEFAULT_LOCK: &str = "/tmp/oxrok-emu.lock";

fn main() -> ExitCode {
    // Run under a model's STANDIN name (a link to this binary), it is that
    // program, and takes that program's arguments.
    let mut args = std::env::args_os();
    if let Some(argv0) = args.next()
        && let Some(name) = Path::new(&argv0).file_name().and_then(|n| n.to_str())
        && let Some(standin) = models::standin(name)
    {
        let all: Vec<_> = std::iter::once(argv0).chain(args).collect();
        return ExitCode::from(standin(&all));
    }
    let cli = Cli::parse();
    let outcome = match cli.command {
        Command::List => {
            let width = models::NAMES.iter().map(|(n, _)| n.len()).max().unwrap_or(0);
            for (name, about) in models::NAMES {
                println!("{name:<width$}  {about}");
            }
            Ok(0)
        }
        Command::Cleanup { all, lock, locked, force } => {
            locked_cleanup(all, &lock, locked || force).map(|()| 0)
        }
        Command::Serve(args) => serve(&args),
    };
    match outcome {
        Ok(0) => ExitCode::SUCCESS,
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("oxrok-emu: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Runs [`cleanup`] under the session lock, unless `unlocked` says the
/// caller holds it or wants it ignored. A running session's port looks
/// like a stale one, so a cleanup that ignored a held lock would detach
/// it, and its server would exit 1 in the middle of the session's cases.
fn locked_cleanup(all: bool, lock: &Path, unlocked: bool) -> Result<()> {
    if unlocked {
        return cleanup(all);
    }
    // Opened for reading, as the harness opens it: root may not create
    // or open for writing another user's file in sticky /tmp
    // (fs.protected_regular), and flock needs neither.
    let file = match std::fs::File::open(lock) {
        Ok(f) => f,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return cleanup(all),
        Err(e) => return Err(Error::Other(format!("{}: {e}", lock.display()))),
    };
    match file.try_lock_shared() {
        Ok(()) => cleanup(all),
        Err(std::fs::TryLockError::WouldBlock) => Err(Error::Other(format!(
            "an emulator session holds {}; its port is not stale (--force detaches it anyway)",
            lock.display()
        ))),
        Err(std::fs::TryLockError::Error(e)) => {
            Err(Error::Other(format!("{}: {e}", lock.display())))
        }
    }
}

/// Detaches the stale ports and waits for the controller to free them.
fn cleanup(all: bool) -> Result<()> {
    for p in vhci::cleanup((!all).then_some(BUSNUM))? {
        println!("detached port {} (busid {}, devid {:#010x})", p.port, p.busid, p.devid);
    }
    Ok(())
}

/// The `port=... nodes=...` fields of a line of standard output, from a
/// lifecycle event.
fn attach_fields(event: &Value) -> String {
    let text = |key: &str| event[key].as_str().unwrap_or_default().to_owned();
    let nodes: Vec<&str> = event["nodes"]
        .as_array()
        .map(|n| n.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    format!(
        "port={} busid={} usb={} usbfs={} nodes={}",
        event["port"],
        text("busid"),
        text("usb"),
        text("usbfs"),
        nodes.join(",")
    )
}

/// Tells standard output what happened to the device: the session calls
/// it before it answers anything taken in later, so whoever gets the
/// reply to a `sync` finds the line there.
fn announce(event: &Value) {
    match event["event"].as_str().unwrap_or_default() {
        "ready" => println!("READY {}", attach_fields(event)),
        "renumerated" => println!("RENUMERATED {}", attach_fields(event)),
        "replugged" => println!("REPLUGGED {}", attach_fields(event)),
        "fired" => println!("EVENT {}", event["name"].as_str().unwrap_or_default()),
        _ => {}
    }
}

/// Ends the process from outside the session (INT, TERM or HUP, or the
/// process it serves gone), the device detached first; the status is that
/// of a run that ended (1 for a lost device, 2 after violations).
fn finish_process(stopper: &Stopper) -> ! {
    let report = stopper.finish();
    println!("violations: {}", report.violations);
    std::process::exit(if report.lost {
        1
    } else if report.violations == 0 {
        0
    } else {
        2
    });
}

#[cfg(target_os = "linux")]
type SysfsCarrier = oxrok_emu::shells::sysfs::Carrier;
/// No tree off Linux: `--sysfs` is refused there.
#[cfg(not(target_os = "linux"))]
#[derive(Clone)]
enum SysfsCarrier {}

/// The tree of `--sysfs`, when asked for.
#[cfg(target_os = "linux")]
fn sysfs_carrier(
    args: &Serve,
    log: &Arc<Mutex<TxLog<Box<dyn Write + Send>>>>,
) -> Result<Option<SysfsCarrier>> {
    let Some(dir) = &args.sysfs else { return Ok(None) };
    let lines: Arc<Mutex<dyn tcp::Log>> = log.clone();
    SysfsCarrier::new(dir, lines).map(Some)
}

#[cfg(not(target_os = "linux"))]
fn sysfs_carrier(
    args: &Serve,
    _log: &Arc<Mutex<TxLog<Box<dyn Write + Send>>>>,
) -> Result<Option<SysfsCarrier>> {
    match args.sysfs {
        Some(_) => Err(Error::Invalid("--sysfs is for Linux".into())),
        None => Ok(None),
    }
}

#[cfg(target_os = "linux")]
fn sysfs_factory(
    model: &str,
    log: &Arc<Mutex<TxLog<Box<dyn Write + Send>>>>,
    tree: &SysfsCarrier,
) -> vusb::session::ModelFactory {
    txlog::factory_with_sysfs(model, Arc::clone(log), tree.clone())
}

#[cfg(not(target_os = "linux"))]
fn sysfs_factory(
    _model: &str,
    _log: &Arc<Mutex<TxLog<Box<dyn Write + Send>>>>,
    tree: &SysfsCarrier,
) -> vusb::session::ModelFactory {
    match *tree {}
}

#[cfg(target_os = "linux")]
fn sysfs_commands(tree: Option<&SysfsCarrier>, builder: Builder) -> Builder {
    match tree {
        Some(tree) => tree.commands(builder),
        None => builder,
    }
}

#[cfg(not(target_os = "linux"))]
fn sysfs_commands(_tree: &Option<SysfsCarrier>, builder: Builder) -> Builder {
    builder
}

/// The tree goes with the session.
#[cfg(target_os = "linux")]
fn sysfs_stop(tree: Option<&SysfsCarrier>) {
    if let Some(tree) = tree {
        tree.stop();
    }
}

#[cfg(not(target_os = "linux"))]
fn sysfs_stop(_tree: Option<&SysfsCarrier>) {}

#[cfg(target_os = "linux")]
fn sysfs_violations(tree: Option<&SysfsCarrier>) -> usize {
    tree.map_or(0, SysfsCarrier::violations)
}

#[cfg(not(target_os = "linux"))]
fn sysfs_violations(_tree: Option<&SysfsCarrier>) -> usize {
    0
}

fn serve(args: &Serve) -> Result<u8> {
    let out: Box<dyn Write + Send> = match &args.log {
        Some(path) => Box::new(BufWriter::new(File::create(path)?)),
        None => Box::new(io::sink()),
    };
    let log = Arc::new(Mutex::new(TxLog::new(out, args.verbose)));
    let carrier = match &args.tcp {
        Some(address) => {
            let lines: Arc<Mutex<dyn tcp::Log>> = log.clone();
            Some(tcp::Carrier::bind(address, lines)?)
        }
        None => None,
    };
    let tree = sysfs_carrier(args, &log)?;
    let factory = match (&carrier, &tree) {
        (Some(carrier), _) => {
            txlog::factory_with_tcp(&args.model, Arc::clone(&log), carrier.clone())
        }
        (None, Some(tree)) => sysfs_factory(&args.model, &log, tree),
        (None, None) => txlog::factory(&args.model, Arc::clone(&log)),
    };
    let observer: SharedObserver = log;
    let mut builder = Builder::new(factory)
        .options(options::parse(&args.options)?)
        .listen(&args.listen)
        .busnum(BUSNUM)
        .attach(args.attach)
        .open_devices(args.open_devices)
        .observer(observer)
        .watch(announce);
    if let Some(carrier) = &carrier {
        builder = carrier.commands(builder);
    }
    builder = sysfs_commands(tree.as_ref(), builder);
    if let Some(control) = &args.control {
        builder = builder.control_with(Address::parse(control), ListenOptions::default());
    }
    let session = builder.start()?;
    if let Some(addr) = session.addr() {
        println!("listening on {addr} busid 1-1");
    }
    if let Some(carrier) = &carrier {
        println!("TCP port={}", carrier.port());
        // Nothing to attach: ready now.
        if !args.attach {
            println!("READY port= busid= usb= usbfs= nodes=");
        }
    }
    if args.sysfs.is_some() {
        println!("READY port= busid= usb= usbfs= nodes=");
    }
    let stopper = session.stopper();
    let ending = tree.clone();
    if let Some(pid) = args.while_alive {
        let stopper = stopper.clone();
        let ending = ending.clone();
        let alive = PathBuf::from(format!("/proc/{pid}"));
        thread::spawn(move || {
            while alive.exists() {
                thread::sleep(Duration::from_secs(1));
            }
            eprintln!("oxrok-emu: process {pid} is gone; ending the session");
            sysfs_stop(ending.as_ref());
            finish_process(&stopper);
        });
    }
    ctrlc::set_handler(move || {
        sysfs_stop(ending.as_ref());
        finish_process(&stopper)
    })
    .map_err(|e| Error::Other(format!("signal handler: {e}")))?;
    let report = session.run()?;
    sysfs_stop(tree.as_ref());
    let violations = report.violations
        + carrier.as_ref().map_or(0, tcp::Carrier::violations)
        + sysfs_violations(tree.as_ref());
    println!("violations: {violations}");
    Ok(if violations == 0 { 0 } else { 2 })
}
