//! `oxrok-emu`: serves a device model over USB/IP and, on Linux with
//! root, attaches it to the machine through `vhci-hcd`.
//!
//! ```text
//! oxrok-emu list
//! sudo oxrok-emu serve usbrelay --attach --open-devices --log tx.log
//! sudo oxrok-emu cleanup
//! ```
//!
//! While serving, commands on standard input drive the run, each
//! acknowledged with an `OK` line on standard output once done:
//! `section NAME` starts a section of the transaction log, `reset`
//! puts a fresh model behind the attached device (its initial state,
//! for a second run), `detach` (or end of input, or a termination
//! signal) detaches the device and exits. `READY port=N busid=B
//! usb=BUS.ADDR usbfs=/dev/bus/usb/BBB/DDD nodes=...` is printed once
//! the kernel has configured the device. The exit status is 2 when any
//! violation was logged.
//!
//! `cleanup` detaches the `vhci-hcd` ports a crashed run left attached:
//! those whose device announced an oxrok-emu `devid` (bus `0x6f78`), or
//! with `--all` every attached port. A running session's port carries
//! that `devid` too, so `cleanup` takes the session lock
//! (`/tmp/oxrok-emu.lock`, `--lock PATH`) and refuses while another
//! session holds it; a session cleaning up under its own lock says so
//! with `--locked`, and `--force` cleans up regardless.
//!
//! Each server announces a device number no attached oxrok-emu port
//! has, so its `devid` names its own port: on the way out it detaches
//! the port only while the port still holds that `devid`. A server
//! whose port someone else detached (a `cleanup`, say) thus leaves the
//! port alone even after another server has been given it, and exits
//! with status 1, as its device went away during the run.

use std::io::{self, BufRead};
use std::net::TcpListener;
use std::os::fd::AsRawFd;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::Duration;

use clap::{Parser, Subcommand};
use oxrok_emu::device::Device;
use oxrok_emu::txlog::TxLog;
use oxrok_emu::usbip::Server;
use oxrok_emu::{Result, models, usbip, vhci};

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
}

/// The busid the model is exported under; the client asks for it.
const BUSID: &str = "1-1";

/// The bus number the server announces; with the device number it
/// forms the `devid` in the controller's `status`, by which `cleanup`
/// recognises the ports oxrok-emu attached (`0x6f78` is "ox").
const BUSNUM: u32 = 0x6f78;

/// A device number, 1 to 0xffff, that no attached oxrok-emu port has;
/// derived from the process id, so that two servers started one after
/// the other differ even when the first one's port is already gone.
fn pick_devnum() -> u32 {
    let used: Vec<u32> = vhci::attached_ports(None)
        .map(|ps| ps.iter().filter(|p| p.devid >> 16 == BUSNUM).map(|p| p.devid & 0xffff).collect())
        .unwrap_or_default();
    let start = std::process::id() % 0xffff;
    (0..0xffff).map(|i| (start + i) % 0xffff + 1).find(|d| !used.contains(d)).unwrap_or(1)
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.command {
        Command::List => {
            let width = models::NAMES.iter().map(|(n, _)| n.len()).max().unwrap_or(0);
            for (name, about) in models::NAMES {
                println!("{name:<width$}  {about}");
            }
            ExitCode::SUCCESS
        }
        Command::Cleanup { all, lock, locked, force } => {
            match locked_cleanup(all, &lock, locked || force) {
                Ok(()) => ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("oxrok-emu: {e}");
                    ExitCode::FAILURE
                }
            }
        }
        Command::Serve(args) => match serve(&args) {
            Ok(violations) => {
                if violations == 0 {
                    ExitCode::SUCCESS
                } else {
                    ExitCode::from(2)
                }
            }
            Err(e) => {
                eprintln!("oxrok-emu: {e}");
                ExitCode::FAILURE
            }
        },
    }
}

/// The state a termination signal needs to clean up.
struct Attached {
    port: Mutex<Option<u8>>,
    /// The `devid` the port holds while it is this server's.
    devid: u32,
    /// Set when the port was found detached by someone else.
    lost: AtomicBool,
    log: Arc<Mutex<TxLog>>,
    stop: AtomicBool,
}

impl Attached {
    /// Detaches (once) and reports the violation count. A port that no
    /// longer holds this server's `devid` is someone else's by now and
    /// is left alone.
    fn finish(&self) -> usize {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(port) = self.port.lock().expect("port lock").take() {
            let ours =
                vhci::attached_ports(Some(self.devid)).map(|ps| ps.iter().any(|p| p.port == port));
            match ours {
                Ok(true) => match vhci::detach(port) {
                    Ok(()) => eprintln!("oxrok-emu: detached port {port}"),
                    Err(e) => eprintln!("oxrok-emu: {e}"),
                },
                Ok(false) => {
                    self.lost.store(true, Ordering::Relaxed);
                    eprintln!(
                        "oxrok-emu: port {port} no longer holds devid {:#010x}: the device was \
                         detached by someone else during the run; the port is left alone",
                        self.devid
                    );
                }
                Err(e) => eprintln!("oxrok-emu: port {port} left attached: {e}"),
            }
        }
        let n = self.log.lock().expect("log lock").violations();
        println!("violations: {n}");
        n
    }
}

/// The session lock `tools/diffcli.sh` takes (`EMU_LOCK`).
const DEFAULT_LOCK: &str = "/tmp/oxrok-emu.lock";

/// Runs [`cleanup`] under the session lock, unless `unlocked` says the
/// caller holds it or wants it ignored. A running session's port looks
/// like a stale one, so a cleanup that ignored a held lock would detach
/// it, and its server would exit 1 in the middle of the session's cases.
fn locked_cleanup(all: bool, lock: &std::path::Path, unlocked: bool) -> Result<()> {
    if unlocked {
        return cleanup(all);
    }
    // Opened for reading, as the harness opens it: root may not create
    // or open for writing another user's file in sticky /tmp
    // (fs.protected_regular), and flock needs neither.
    let file = match std::fs::File::open(lock) {
        Ok(f) => f,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return cleanup(all),
        Err(e) => return Err(format!("{}: {e}", lock.display()).into()),
    };
    match file.try_lock_shared() {
        Ok(()) => cleanup(all),
        Err(std::fs::TryLockError::WouldBlock) => Err(format!(
            "an emulator session holds {}; its port is not stale (--force detaches it anyway)",
            lock.display()
        )
        .into()),
        Err(std::fs::TryLockError::Error(e)) => Err(format!("{}: {e}", lock.display()).into()),
    }
}

/// Detaches the stale ports and waits for the controller to free them.
fn cleanup(all: bool) -> Result<()> {
    if !vhci::available() {
        return Ok(());
    }
    let stale: Vec<vhci::PortStatus> = vhci::attached_ports(None)?
        .into_iter()
        .filter(|p| all || p.devid >> 16 == BUSNUM)
        .collect();
    for p in &stale {
        vhci::detach(p.port)?;
        println!("detached port {} (busid {}, devid {:#010x})", p.port, p.busid, p.devid);
    }
    // The port is free once the controller has shut the connection
    // down, which a server that is stopped rather than dead delays.
    let start = std::time::Instant::now();
    loop {
        let busy: Vec<u8> = vhci::attached_ports(None)?
            .iter()
            .filter(|p| stale.iter().any(|s| s.port == p.port))
            .map(|p| p.port)
            .collect();
        if busy.is_empty() {
            return Ok(());
        }
        if start.elapsed() > Duration::from_secs(5) {
            return Err(format!(
                "ports {busy:?} still in use after detach; is an oxrok-emu process stopped?"
            )
            .into());
        }
        thread::sleep(Duration::from_millis(50));
    }
}

/// Waits for the device on `port` to be configured, calls
/// `configured`, opens its nodes, and returns the `port=... nodes=...`
/// fields of the `READY` line.
fn ready_fields(port: u8, open: bool, configured: impl FnOnce()) -> Result<String> {
    let busid = vhci::busid(port, Duration::from_secs(10))?;
    let dir = vhci::wait_configured(&busid, Duration::from_secs(10))?;
    configured();
    let found = vhci::open_device(&dir, open, Duration::from_secs(5))?;
    let names: Vec<String> = found.nodes.iter().map(|n| n.display().to_string()).collect();
    let usbfs = found.usbfs.as_ref().map(|p| p.display().to_string()).unwrap_or_default();
    Ok(format!(
        "port={port} busid={busid} usb={} usbfs={usbfs} nodes={}",
        found.bus_address.as_deref().unwrap_or(""),
        names.join(",")
    ))
}

/// What the main loop waits for: a command line, the end of input, or
/// a model that asks to renumerate.
enum Input {
    Line(String),
    End,
    Renumerate(String),
}

/// Parses `KEY=VALUE` options.
fn parse_options(words: &[String]) -> Result<Vec<(String, String)>> {
    Ok(words
        .iter()
        .map(|o| {
            o.split_once('=')
                .map(|(k, v)| (k.to_owned(), v.to_owned()))
                .ok_or_else(|| format!("option '{o}' is not KEY=VALUE"))
        })
        .collect::<std::result::Result<Vec<_>, _>>()?)
}

/// The attached device's side of a run: the socket handed to the
/// controller and what re-attaching needs.
struct Attachment<'a> {
    addr: std::net::SocketAddr,
    devid: u32,
    open: bool,
    state: &'a Attached,
    log: &'a Mutex<TxLog>,
    device: &'a Mutex<Device>,
    socket: Option<std::net::TcpStream>,
}

impl Attachment<'_> {
    /// Imports the device over a new connection and attaches it, on
    /// port `want` when that one is free; returns the `READY` fields.
    /// Once the kernel has configured it, the log goes on in section
    /// `resume` when given.
    fn attach(&mut self, want: Option<u8>, resume: Option<&str>) -> Result<String> {
        let speed = self.device.lock().expect("device lock").model_descriptors().speed;
        let (socket, _record) = usbip::import(self.addr, BUSID)?;
        let free = |p: u8| {
            vhci::ports().is_ok_and(|ps| {
                ps.iter().any(|s| s.port == p && s.hub == "hs" && s.status == vhci::PORT_FREE)
            })
        };
        let port = match want {
            Some(p) if free(p) => p,
            _ => vhci::free_port(speed)?,
        };
        vhci::attach(port, socket.as_raw_fd(), self.devid, speed)?;
        // The controller took its own reference to the socket; ours is
        // kept until the device is detached.
        self.socket = Some(socket);
        *self.state.port.lock().expect("port lock") = Some(port);
        // A device that never configures is detached again, not left on
        // the bus for the next run to trip over.
        let log = self.log;
        let configured = || {
            if let Some(name) = resume {
                log.lock().expect("log lock").section(name);
            }
        };
        match ready_fields(port, self.open, configured) {
            Ok(fields) => Ok(fields),
            Err(e) => {
                self.state.finish();
                Err(e)
            }
        }
    }

    /// A fresh model of `model` with `options` behind the device. A
    /// device that renumerated, or whose options changed its
    /// descriptors, is attached again (when `attach`) as the fresh
    /// model presents itself, in a `reset` section of the log, not in
    /// the section of the run before, whose transactions a case
    /// compares.
    fn reset(&mut self, model: &str, options: &[(String, String)], attach: bool) -> Result<()> {
        let fresh = models::build(model, options)?;
        let again = {
            let mut d = self.device.lock().expect("device lock");
            d.replace_model(fresh);
            d.needs_reenumeration()
        };
        self.log.lock().expect("log lock").event("model reset");
        if again && attach {
            self.log.lock().expect("log lock").section("reset");
            let fields = self.reattach("reset", None)?;
            println!("RENUMERATED {fields}");
        }
        Ok(())
    }

    /// The device leaves the bus and comes back with the model's
    /// current descriptors, on the same port when it is still free (a
    /// driver finds a renumerated device by its port): detach, drop the
    /// connection, wait for the port, import and attach again. The log
    /// gets `disconnect` and `connect VID:PID` lines, which the
    /// comparisons keep, then from the detach on the kernel's
    /// enumeration in a `renumeration` section, then section `resume`
    /// again (the server logged the two lines already when it passes
    /// `resume`).
    fn reattach(&mut self, why: &str, resume: Option<String>) -> Result<String> {
        let port = self.state.port.lock().expect("port lock").take();
        let ids = {
            let d = self.device.lock().expect("device lock");
            let m = d.model_descriptors();
            (m.vendor_id(), m.product_id())
        };
        let section = {
            let mut log = self.log.lock().expect("log lock");
            log.event(&format!("{why}: detaching port {port:?}"));
            let r = if let Some(r) = resume {
                r
            } else {
                log.transfer("disconnect");
                log.transfer(&format!("connect {:04x}:{:04x}", ids.0, ids.1));
                log.current_section().to_owned()
            };
            // From the detach on: the kernel's doing, not the tool's.
            log.section("renumeration");
            r
        };
        if let Some(p) = port {
            let ours =
                vhci::attached_ports(Some(self.devid)).map(|ps| ps.iter().any(|s| s.port == p))?;
            if !ours {
                self.state.lost.store(true, Ordering::Relaxed);
                return Err("the device was detached by someone else during the run".into());
            }
            vhci::detach(p)?;
        }
        // The server sees the connection close only once our end goes.
        drop(self.socket.take());
        let start = std::time::Instant::now();
        while let Some(p) = port {
            let busy = vhci::attached_ports(None)?.iter().any(|s| s.port == p);
            if !busy {
                break;
            }
            if start.elapsed() > Duration::from_secs(5) {
                return Err(format!("port {p} still in use 5 s after the detach").into());
            }
            thread::sleep(Duration::from_millis(20));
        }
        self.log.lock().expect("log lock").event(&format!("{why}: attaching again"));
        self.attach(port, Some(&section))
    }
}

/// A channel for the server's renumeration requests, passed on to the
/// main loop's inputs.
fn forward_renumerations(inputs: mpsc::Sender<Input>) -> mpsc::Sender<String> {
    let (send, recv) = mpsc::channel::<String>();
    thread::spawn(move || {
        while let Ok(resume) = recv.recv() {
            if inputs.send(Input::Renumerate(resume)).is_err() {
                break;
            }
        }
    });
    send
}

/// Passes the lines of standard input on to the main loop, then its end.
fn read_commands(inputs: mpsc::Sender<Input>) {
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            let Ok(line) = line else { break };
            if inputs.send(Input::Line(line)).is_err() {
                return;
            }
        }
        let _ = inputs.send(Input::End);
    });
}

/// The options of an `options` command, checked by building the model
/// with them; none restores `initial`, those of the command line.
fn new_options(
    model: &str,
    initial: &[(String, String)],
    rest: &str,
) -> Result<Vec<(String, String)>> {
    let words: Vec<String> = rest.split_whitespace().map(str::to_owned).collect();
    let options = if words.is_empty() { initial.to_vec() } else { parse_options(&words)? };
    models::build(model, &options)?;
    Ok(options)
}

/// Ends the process on INT, TERM or HUP, the device detached first;
/// the status is that of a run that ended (1 for a lost device, 2
/// after violations).
fn exit_on_signal(state: Arc<Attached>) -> Result<()> {
    ctrlc::set_handler(move || {
        let n = state.finish();
        let lost = state.lost.load(Ordering::Relaxed);
        std::process::exit(if lost {
            1
        } else if n == 0 {
            0
        } else {
            2
        });
    })
    .map_err(|e| format!("signal handler: {e}").into())
}

fn serve(args: &Serve) -> Result<usize> {
    let initial_options = parse_options(&args.options)?;
    let mut options = initial_options.clone();
    let model = models::build(&args.model, &options)?;
    let log = Arc::new(Mutex::new(TxLog::new(args.log.as_deref(), args.verbose)?));
    log.lock().expect("log lock").section("attach");
    let device = Arc::new(Mutex::new(Device::new(model)));
    let (tx, rx) = mpsc::channel();
    let renum_send = forward_renumerations(tx.clone());
    let server = Server::new(Arc::clone(&device), Arc::clone(&log), BUSID, BUSNUM, pick_devnum())
        .on_renumeration(renum_send);
    let devid = server.devid();
    let listener = TcpListener::bind(&args.listen)?;
    let addr = listener.local_addr()?;
    println!("listening on {addr} busid {BUSID}");

    let state = Arc::new(Attached {
        port: Mutex::new(None),
        devid,
        lost: AtomicBool::new(false),
        log: Arc::clone(&log),
        stop: AtomicBool::new(false),
    });
    let server_state = Arc::clone(&state);
    let server_thread = thread::spawn(move || server.run(&listener, &server_state.stop));
    exit_on_signal(Arc::clone(&state))?;

    let mut att = Attachment {
        addr,
        devid,
        open: args.open_devices,
        state: &state,
        log: &log,
        device: &device,
        socket: None,
    };
    if args.attach {
        if !vhci::available() {
            return Err("vhci-hcd is not loaded (sudo modprobe vhci-hcd)".into());
        }
        let fields = att.attach(None, None)?;
        println!("READY {fields}");
    }

    read_commands(tx);

    let result = loop {
        let Ok(input) = rx.recv() else { break Ok(()) };
        let line = match input {
            Input::End => break Ok(()),
            Input::Renumerate(resume) => {
                if args.attach {
                    match att.reattach("renumeration", Some(resume)) {
                        Ok(fields) => println!("RENUMERATED {fields}"),
                        Err(e) => break Err(e),
                    }
                }
                continue;
            }
            Input::Line(l) => l,
        };
        let line = line.trim();
        let (word, rest) = line.split_once(' ').unwrap_or((line, ""));
        match word {
            "" => {}
            "section" => {
                log.lock().expect("log lock").section(rest.trim());
                println!("OK {line}");
            }
            "options" => {
                // The options of the resets that follow; none restores
                // those of the command line.
                match new_options(&args.model, &initial_options, rest) {
                    Ok(o) => {
                        options = o;
                        println!("OK {line}");
                    }
                    Err(e) => println!("ERR {line}: {e}"),
                }
            }
            "reset" => {
                // A fresh model, the bus state kept: the next run starts
                // from the device's initial state. A device that
                // renumerated, or whose options changed its
                // descriptors, is attached again as the fresh model
                // presents itself.
                if let Err(e) = att.reset(&args.model, &options, args.attach) {
                    break Err(e);
                }
                println!("OK {line}");
            }
            "detach" | "quit" => {
                println!("OK {line}");
                break Ok(());
            }
            _ => println!("ERR unknown command: {line}"),
        }
    };
    let n = state.finish();
    drop(att.socket.take());
    match server_thread.join() {
        Ok(Ok(())) => {}
        Ok(Err(e)) => eprintln!("oxrok-emu: server: {e}"),
        Err(_) => eprintln!("oxrok-emu: server thread panicked"),
    }
    result?;
    if state.lost.load(Ordering::Relaxed) {
        return Err("the device was detached by someone else during the run".into());
    }
    Ok(n)
}
