// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A TCP carrier: a model that is a byte stream on a socket, for the instruments
//! and analyzers libsigrok reaches over `tcp-raw/HOST/PORT`, with what goes on the
//! session's log and control socket as on a USB shell's. `oxrok-emu serve MODEL
//! --tcp 127.0.0.1:0` binds the port once and prints it (`TCP port=N`); the port
//! outlives every case, so nothing can race for it.
//!
//! A model implements [`Stream`], as a serial one implements `cdc::Serial`, and
//! has a `pub fn build_tcp(options)` beside its `build`.
//! [`crate::shells::scpi::Config::tcp`] makes one of an instrument.
//!
//! - **Lifetimes.** A `reset` builds a fresh model and drops what is connected,
//!   so each tool of each case starts from power on.
//! - **The log.** `tcp_connect`, `tcp_close`, `tcp_out DATA -> LEN`, and
//!   `tcp_in LEN -> DATA` with the digest of the model's rules: the shape of a CDC
//!   model's `bulk_out` and `bulk_in`, whose cut means as little. Consecutive
//!   bytes of a direction are one line, whatever the kernel made of them, so a
//!   command with no answer shares a line with the next. `tcp_in` is what the
//!   model sent, in order, not what the host has read.
//! - **`sync`** waits for the connections of a tool that has ended to go, so that
//!   its `tcp_close` lands in its section.
//! - **An unplug** is `--unplug tcp-in=N` (after the model's Nth send) or
//!   `--unplug tcp-model` (when the model marks its point): what was sent gets
//!   out, the connection closes with a FIN, and `unplug` goes in the log. Until
//!   the next `replug` or `reset`, a new connection is accepted and closed at once
//!   (`tcp_refused`). `--interrupt` takes the same points. A model that closes on
//!   its own calls [`Port::hang_up`].
//! - **A stall** is a model that says nothing: the connection stays up and no
//!   bytes come (`mute-at=` in a SCPI model).
//! - **Time.** [`Stream::tick`] comes every 20 ms of wall time, while a host is
//!   connected: nothing else paces a socket. It stands where a CDC model's offers
//!   of a read do, at about their rate, but it is the clock, not the host.

use std::io::{ErrorKind, Read, Write};
use std::net::{Shutdown, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use sha2::{Digest as _, Sha256};
use vusb::arm::{Arm, Event};
use vusb::observer::fixture::{Rules, hex};
use vusb::session::{Builder, Ctx};

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::{Error, Result};

/// How long the carrier sleeps when nothing moved.
const IDLE: Duration = Duration::from_millis(1);
/// The period of [`Stream::tick`].
const TICK: Duration = Duration::from_millis(20);
/// How long `sync` waits for a connection whose tool has gone.
const DRAIN: Duration = Duration::from_secs(2);
/// The most read from a socket at a time.
const READ: usize = 1 << 16;

/// A device on a socket. Nothing is blocking here: answer in the call.
pub trait Stream: Send {
    /// What a host wrote. How the kernel cut it means nothing, so collect
    /// frames yourself, in a buffer with a bound.
    fn received(&mut self, data: &[u8], port: &mut Port);
    /// A host connected.
    fn connected(&mut self, port: &mut Port) {
        let _ = port;
    }
    /// The host closed, or the connection is gone.
    fn closed(&mut self) {}
    /// Every 20 ms of wall time, with a host connected.
    fn tick(&mut self, port: &mut Port) {
        let _ = port;
    }
}

/// What a [`Stream`] can do to the connection at hand.
#[derive(Debug, Default)]
pub struct Port {
    sent: Vec<Vec<u8>>,
    v: Violations,
    hang_up: bool,
    mark: bool,
}

impl Port {
    /// Sends bytes.
    pub fn send(&mut self, data: &[u8]) {
        self.sent.push(data.to_vec());
    }

    /// Reports what the host did wrong.
    pub fn violation(&mut self, message: impl Into<String>) {
        self.v.report(message);
    }

    /// Where a helper reports.
    pub fn violations(&mut self) -> &mut Violations {
        &mut self.v
    }

    /// Reaches the point an event armed with `tcp-model` waits for.
    pub fn mark(&mut self) {
        self.mark = true;
    }

    /// Closes this connection, after what was sent.
    pub fn hang_up(&mut self) {
        self.hang_up = true;
    }

    /// What was sent, for a model's unit test.
    pub fn sent(&self) -> &[Vec<u8>] {
        &self.sent
    }
}

/// Where the carrier's lines go.
pub trait Log: Send {
    /// A transaction line.
    fn line(&mut self, text: &str);
    /// A violation.
    fn violation(&mut self, message: &str);
    /// The model's rules for a line.
    fn rules(&self) -> Rules;
}

/// When an armed event fires.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum When {
    /// After the model's Nth send.
    Sent(u32),
    /// When the model marks its point.
    Model,
}

struct Conn {
    stream: TcpStream,
    /// What the socket has not taken yet.
    pending: Vec<u8>,
    /// Close once `pending` is out.
    closing: bool,
}

/// A direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Way {
    Out,
    In,
}

/// The bytes of a direction, until it changes. Hashed as they come, and kept only as
/// far as the digest rule shows them, as an analyzer's stream may not end soon. Under
/// no digest rule all of it is kept, so a model that streams gives one in `log_rules`.
struct Run {
    way: Way,
    kept: Vec<u8>,
    len: usize,
    hash: Sha256,
}

impl Run {
    fn new(way: Way) -> Run {
        Run { way, kept: Vec::new(), len: 0, hash: Sha256::new() }
    }

    fn add(&mut self, bytes: &[u8], above: Option<usize>) {
        self.len += bytes.len();
        self.hash.update(bytes);
        if above.is_some_and(|n| self.len > n) {
            self.kept = Vec::new();
        } else {
            self.kept.extend_from_slice(bytes);
        }
    }

    /// As `fixture::data` writes it.
    fn text(self, above: Option<usize>) -> String {
        if above.is_some_and(|n| self.len > n) {
            format!("@{}:{}", self.len, hex(&self.hash.finalize()))
        } else {
            hex(&self.kept)
        }
    }
}

struct Inner {
    listener: TcpListener,
    model: Option<Box<dyn Stream>>,
    conns: Vec<Conn>,
    /// Of `conns`, the one that last spoke, which a tick answers.
    current: usize,
    log: Arc<Mutex<dyn Log>>,
    run: Option<Run>,
    armed: Option<(Event, When)>,
    sends: u32,
    unplugged: bool,
    last_tick: Instant,
}

/// The carrier: a listening socket and a thread that serves it.
#[derive(Clone)]
pub struct Carrier {
    inner: Arc<Mutex<Inner>>,
    port: u16,
    violations: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
}

impl Carrier {
    /// Binds `address` and serves whatever [`Carrier::install`] installs.
    ///
    /// # Errors
    ///
    /// When it cannot bind.
    pub fn bind(address: &str, log: Arc<Mutex<dyn Log>>) -> Result<Carrier> {
        let listener = TcpListener::bind(address)?;
        listener.set_nonblocking(true)?;
        let port = listener.local_addr()?.port();
        let inner = Inner {
            listener,
            model: None,
            conns: Vec::new(),
            current: 0,
            log,
            run: None,
            armed: None,
            sends: 0,
            unplugged: false,
            last_tick: Instant::now(),
        };
        let carrier = Carrier {
            inner: Arc::new(Mutex::new(inner)),
            port,
            violations: Arc::new(AtomicUsize::new(0)),
            stop: Arc::new(AtomicBool::new(false)),
        };
        let serving = carrier.clone();
        thread::Builder::new().name("tcp carrier".to_owned()).spawn(move || serving.serve())?;
        Ok(carrier)
    }

    /// The port it listens on.
    pub fn port(&self) -> u16 {
        self.port
    }

    /// The violations it logged.
    pub fn violations(&self) -> usize {
        self.violations.load(Ordering::SeqCst)
    }

    /// Ends the thread.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().expect("carrier lock")
    }

    fn serve(&self) {
        while !self.stop.load(Ordering::SeqCst) {
            let moved = self.lock().poll(&self.violations);
            if !moved {
                thread::sleep(IDLE);
            }
        }
    }

    /// A fresh model: what was connected goes, in silence, and so does what
    /// was armed.
    pub fn install(&self, model: Box<dyn Stream>) {
        let mut inner = self.lock();
        inner.flush();
        inner.conns.clear();
        inner.model = Some(model);
        inner.armed = None;
        inner.sends = 0;
        inner.unplugged = false;
    }

    /// Waits for the connections to end, with a bound, and writes the log out.
    pub fn sync(&self) {
        let began = Instant::now();
        loop {
            let mut inner = self.lock();
            inner.poll(&self.violations);
            if inner.conns.is_empty() || began.elapsed() > DRAIN {
                inner.flush();
                return;
            }
            drop(inner);
            thread::sleep(IDLE);
        }
    }

    /// New connections are served again.
    pub fn replug(&self) {
        self.lock().unplugged = false;
    }

    /// Arms an event at `tcp-in=N` or `tcp-model`.
    ///
    /// # Errors
    ///
    /// On another point, or an N of 0.
    pub fn arm(&self, event: Event, when: &str) -> Result<()> {
        let when = if when == "tcp-model" {
            When::Model
        } else {
            let n = when.strip_prefix("tcp-in=").and_then(|n| n.parse().ok()).filter(|n| *n > 0);
            When::Sent(n.ok_or_else(|| {
                Error::Invalid(format!("{when}: a TCP point is tcp-in=N or tcp-model"))
            })?)
        };
        let mut inner = self.lock();
        inner.armed = Some((event, when));
        inner.sends = 0;
        Ok(())
    }

    /// The session's `sync`, `arm` and `replug` for a session with a carrier:
    /// the USB device's as ever, unless the point says `tcp-`.
    #[must_use]
    pub fn commands(&self, builder: Builder) -> Builder {
        let (sync, arm, replug) = (self.clone(), self.clone(), self.clone());
        builder
            .command("sync", "as ever, and the tool's connections have gone", move |_, _| {
                sync.sync();
                Ok(json!({}))
            })
            .command(
                "arm",
                "event=unplug or event=mark name=NAME, when=in=EP:N, out=EP:N, model, tcp-in=N \
                 or tcp-model",
                move |args, ctx| arm_command(&arm, args, ctx),
            )
            .command("replug", "an unplugged device comes back", move |_, ctx: &mut Ctx| {
                replug.replug();
                ctx.replug().map(|()| json!({}))
            })
    }
}

fn arm_command(carrier: &Carrier, args: &Value, ctx: &mut Ctx) -> Result<Value> {
    let text = |key| args.get(key).and_then(Value::as_str);
    let (Some(kind), Some(when)) = (text("event"), text("when")) else {
        return Err(Error::Invalid("arm takes event=, name= for a mark, and when=".into()));
    };
    if when.starts_with("tcp-") {
        let event = match (kind, text("name")) {
            ("mark", Some(name)) => Event::Mark(name.to_owned()),
            ("unplug", _) => Event::Unplug,
            _ => return Err(Error::Invalid(format!("no event {kind}"))),
        };
        carrier.arm(event, when)?;
    } else {
        let spec = match (kind, text("name")) {
            ("mark", Some(name)) => format!("mark {name} {when}"),
            _ => format!("{kind} {when}"),
        };
        ctx.bus().arm(Arm::parse(&spec)?);
    }
    Ok(json!({}))
}

impl Inner {
    fn line(&self, text: &str) {
        self.log.lock().expect("log lock").line(text);
    }

    /// Writes the run of bytes out.
    fn flush(&mut self) {
        let Some(run) = self.run.take() else { return };
        let above = self.above();
        let (way, len) = (run.way, run.len);
        let text = run.text(above);
        match way {
            Way::Out => self.line(&format!("tcp_out {text} -> {len}")),
            Way::In => self.line(&format!("tcp_in {len} -> {text}")),
        }
    }

    fn above(&self) -> Option<usize> {
        self.log.lock().expect("log lock").rules().digest_above
    }

    fn logged(&mut self, way: Way, bytes: &[u8]) {
        if self.run.as_ref().is_none_or(|r| r.way != way) {
            self.flush();
            self.run = Some(Run::new(way));
        }
        let above = self.above();
        if let Some(run) = &mut self.run {
            run.add(bytes, above);
        }
    }

    /// Runs a call of the model and does what it asked, on connection `at`.
    fn with_model(
        &mut self,
        at: usize,
        violations: &AtomicUsize,
        call: impl FnOnce(&mut dyn Stream, &mut Port),
    ) {
        let Some(mut model) = self.model.take() else { return };
        let mut port = Port::default();
        call(model.as_mut(), &mut port);
        self.model = Some(model);
        for message in port.v.drain() {
            self.flush();
            violations.fetch_add(1, Ordering::SeqCst);
            self.log.lock().expect("log lock").violation(&message);
        }
        for bytes in &port.sent {
            self.logged(Way::In, bytes);
            if let Some(c) = self.conns.get_mut(at) {
                c.pending.extend_from_slice(bytes);
            }
            self.sends += 1;
            if matches!(self.armed, Some((_, When::Sent(n))) if n == self.sends) {
                self.fire();
            }
        }
        if port.mark && matches!(self.armed, Some((_, When::Model))) {
            self.fire();
        }
        if port.hang_up
            && let Some(c) = self.conns.get_mut(at)
        {
            c.closing = true;
        }
    }

    /// The armed event happens: its word in the log, its line on standard
    /// output, as a USB device's, and for an unplug the end of the connections.
    fn fire(&mut self) {
        let Some((event, _)) = self.armed.take() else { return };
        self.flush();
        self.line(event.word());
        println!("EVENT {}", event.word());
        if event == Event::Unplug {
            self.unplugged = true;
            for c in &mut self.conns {
                c.closing = true;
            }
        }
    }

    /// Reads what connection `at` has. Whether anything moved, and whether the
    /// host is gone.
    fn read(&mut self, at: usize, violations: &AtomicUsize) -> (bool, bool) {
        let mut buf = vec![0u8; READ];
        let mut moved = false;
        loop {
            let got = match self.conns.get_mut(at) {
                // Bytes after an unplug go nowhere.
                Some(c) if !c.closing => c.stream.read(&mut buf),
                _ => return (moved, false),
            };
            match got {
                Ok(0) => return (true, true),
                Ok(n) => {
                    moved = true;
                    self.current = at;
                    self.logged(Way::Out, &buf[..n]);
                    let bytes = &buf[..n];
                    self.with_model(at, violations, |m, p| m.received(bytes, p));
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return (moved, false),
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(_) => return (true, true),
            }
        }
    }

    /// What was sent goes out, as far as the socket takes it.
    fn write(&mut self, at: usize) -> bool {
        let Some(c) = self.conns.get_mut(at) else { return false };
        let mut moved = false;
        while !c.pending.is_empty() {
            match c.stream.write(&c.pending) {
                Ok(0) => break,
                Ok(n) => {
                    c.pending.drain(..n);
                    moved = true;
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                // The host has gone; a read says so.
                Err(_) => {
                    c.pending.clear();
                    break;
                }
            }
        }
        moved
    }

    /// Reads every connection; those whose host has gone leave the log's way.
    fn read_all(&mut self, violations: &AtomicUsize) -> bool {
        let mut moved = false;
        let mut at = 0;
        while at < self.conns.len() {
            let (got, gone) = self.read(at, violations);
            moved |= got;
            if gone {
                self.flush();
                self.line("tcp_close");
                self.conns.remove(at);
                if let Some(model) = &mut self.model {
                    model.closed();
                }
            } else {
                at += 1;
            }
        }
        moved
    }

    /// One round: reads, writes, closes, accepts, ticks. Whether anything moved.
    fn poll(&mut self, violations: &AtomicUsize) -> bool {
        let mut moved = self.read_all(violations);
        for at in 0..self.conns.len() {
            moved |= self.write(at);
        }
        // What is to close and has its bytes out. The device's end, so no
        // tcp_close: the unplug or the model's hang up said it.
        self.conns.retain(|c| {
            let done = c.closing && c.pending.is_empty();
            if done {
                let _ = c.stream.shutdown(Shutdown::Both);
            }
            !done
        });
        moved |= self.accept(violations);
        if !self.conns.is_empty() && self.last_tick.elapsed() >= TICK {
            self.last_tick = Instant::now();
            let at = self.current.min(self.conns.len() - 1);
            self.with_model(at, violations, |m, p| m.tick(p));
        }
        moved
    }

    fn accept(&mut self, violations: &AtomicUsize) -> bool {
        let mut moved = false;
        while let Ok((stream, _)) = self.listener.accept() {
            moved = true;
            // A tool that closes and connects again: its close came first, so
            // the log says so whichever this round saw first.
            self.read_all(violations);
            self.flush();
            if self.unplugged {
                self.line("tcp_refused");
                continue;
            }
            if stream.set_nonblocking(true).is_err() {
                continue;
            }
            let _ = stream.set_nodelay(true);
            self.line("tcp_connect");
            self.conns.push(Conn { stream, pending: Vec::new(), closing: false });
            self.current = self.conns.len() - 1;
            let at = self.current;
            self.with_model(at, violations, |m, p| m.connected(p));
        }
        moved
    }
}

/// The USB side of a model with none: `oxrok-emu serve` needs a device to build,
/// and `EMU_TCP=only` never attaches it. It answers nothing.
#[derive(Debug)]
pub struct NoUsb {
    name: &'static str,
    descriptors: Descriptors,
}

impl NoUsb {
    /// One for the model `name`.
    pub fn new(name: &'static str) -> NoUsb {
        let device = crate::descriptor::DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: 0x1209,
            product: 0x0001,
            device: 0x0100,
            strings: [0, 0, 0],
            configurations: 1,
        };
        let config = crate::descriptor::configuration(1, 0, 0x80, 100, &[]);
        NoUsb {
            name,
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![config],
                strings: Vec::new(),
            },
        }
    }
}

impl DeviceModel for NoUsb {
    fn name(&self) -> &'static str {
        self.name
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, _setup: Setup, _data: &[u8], _v: &mut Violations) -> Reply {
        Reply::Stall
    }

    fn out(&mut self, _ep: u8, _data: &[u8], _v: &mut Violations) -> Reply {
        Reply::Stall
    }

    fn in_transfer(&mut self, _ep: u8, _max: usize, _v: &mut Violations) -> Reply {
        Reply::Stall
    }

    fn reset(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Lines(Vec<String>);

    impl Log for Lines {
        fn line(&mut self, text: &str) {
            self.0.push(text.to_owned());
        }
        fn violation(&mut self, message: &str) {
            self.0.push(format!("! {message}"));
        }
        fn rules(&self) -> Rules {
            Rules::digest_above(16)
        }
    }

    /// `h` answers `H`, `b` a long burst, `B` one no socket holds, `x` hangs up, `v` is flagged, `m` marks,
    /// `l` answers `L` on the next tick.
    #[derive(Default)]
    struct Toy {
        later: bool,
    }

    impl Stream for Toy {
        fn received(&mut self, data: &[u8], port: &mut Port) {
            for b in data {
                match b {
                    b'h' => port.send(b"H"),
                    b'b' => port.send(&vec![7; 1 << 20]),
                    b'B' => port.send(&vec![7; 1 << 25]),
                    b'x' => port.hang_up(),
                    b'v' => port.violation("v it is"),
                    b'm' => port.mark(),
                    b'l' => self.later = true,
                    _ => {}
                }
            }
        }

        fn tick(&mut self, port: &mut Port) {
            if std::mem::take(&mut self.later) {
                port.send(b"L");
            }
        }
    }

    fn carrier() -> (Carrier, Arc<Mutex<Lines>>) {
        let lines = Arc::new(Mutex::new(Lines::default()));
        let log: Arc<Mutex<dyn Log>> = lines.clone();
        let c = Carrier::bind("127.0.0.1:0", log).unwrap();
        c.install(Box::new(Toy::default()));
        (c, lines)
    }

    fn connect(c: &Carrier) -> TcpStream {
        let s = TcpStream::connect(("127.0.0.1", c.port())).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(30))).unwrap();
        s
    }

    fn read_n(s: &mut TcpStream, n: usize) -> Vec<u8> {
        let mut out = vec![0; n];
        s.read_exact(&mut out).unwrap();
        out
    }

    fn logged(lines: &Arc<Mutex<Lines>>) -> Vec<String> {
        lines.lock().unwrap().0.clone()
    }

    #[test]
    fn a_connection_is_logged() {
        let (c, lines) = carrier();
        let mut s = connect(&c);
        s.write_all(b"ah").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H");
        drop(s);
        c.sync();
        c.stop();
        assert_eq!(
            logged(&lines),
            ["tcp_connect", "tcp_out 6168 -> 2", "tcp_in 1 -> 48", "tcp_close"]
        );
    }

    #[test]
    fn a_direction_is_one_line_however_it_came() {
        let (c, lines) = carrier();
        {
            let mut inner = c.lock();
            inner.logged(Way::Out, b"a");
            inner.logged(Way::Out, b"h");
            inner.logged(Way::In, b"H");
            inner.logged(Way::In, b"I");
            inner.logged(Way::Out, b"b");
            inner.flush();
        }
        c.stop();
        assert_eq!(logged(&lines), ["tcp_out 6168 -> 2", "tcp_in 2 -> 4849", "tcp_out 62 -> 1"]);
    }

    #[test]
    fn a_long_run_is_hashed_as_it_comes_and_keeps_nothing() {
        let mut run = Run::new(Way::In);
        let whole = vec![7u8; 100];
        for piece in whole.chunks(9) {
            run.add(piece, Some(16));
        }
        assert_eq!(run.kept, Vec::<u8>::new());
        assert_eq!(run.text(Some(16)), vusb::observer::fixture::data(&whole, Some(16)));
        let mut run = Run::new(Way::In);
        run.add(&whole[..16], Some(16));
        assert_eq!(run.text(Some(16)), vusb::observer::fixture::data(&whole[..16], Some(16)));
    }

    #[test]
    fn a_tick_comes_with_a_host_connected() {
        let (c, _lines) = carrier();
        let mut s = connect(&c);
        s.write_all(b"l").unwrap();
        assert_eq!(read_n(&mut s, 1), b"L");
        c.stop();
    }

    #[test]
    fn a_fresh_model_forgets_what_was_armed() {
        let (c, lines) = carrier();
        c.arm(Event::Unplug, "tcp-in=1").unwrap();
        c.install(Box::new(Toy::default()));
        let mut s = connect(&c);
        s.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H");
        s.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H", "still up");
        drop(s);
        c.sync();
        c.stop();
        assert!(!logged(&lines).contains(&"unplug".to_owned()));
    }

    #[test]
    fn a_fresh_model_is_plugged_in() {
        let (c, lines) = carrier();
        c.arm(Event::Unplug, "tcp-in=1").unwrap();
        let mut s = connect(&c);
        s.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H");
        let mut rest = Vec::new();
        s.read_to_end(&mut rest).unwrap();
        c.install(Box::new(Toy::default()));
        let mut s = connect(&c);
        s.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H", "served, not refused");
        drop(s);
        c.sync();
        c.stop();
        assert!(!logged(&lines).contains(&"tcp_refused".to_owned()));
    }

    #[test]
    fn sync_waits_for_a_host_on_its_way_out() {
        let (c, lines) = carrier();
        let s = connect(&c);
        // Accepted, and logged.
        let began = Instant::now();
        while logged(&lines).is_empty() && began.elapsed() < Duration::from_secs(30) {
            thread::sleep(IDLE);
        }
        let closer = thread::spawn(move || {
            thread::sleep(Duration::from_millis(100));
            drop(s);
        });
        c.sync();
        c.stop();
        closer.join().unwrap();
        assert_eq!(logged(&lines), ["tcp_connect", "tcp_close"]);
    }

    #[test]
    fn a_close_of_the_device_sends_all_it_had_first() {
        let (c, _lines) = carrier();
        c.arm(Event::Unplug, "tcp-in=1").unwrap();
        let mut s = connect(&c);
        s.write_all(b"B").unwrap();
        // More than a socket holds, with the host reading only now.
        assert_eq!(read_n(&mut s, 1 << 25).len(), 1 << 25);
        let mut rest = Vec::new();
        s.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, Vec::<u8>::new());
        c.stop();
    }

    #[test]
    fn a_close_comes_before_the_connect_after_it() {
        let (c, lines) = carrier();
        for _ in 0..20 {
            drop(connect(&c));
            // No wait: the two come as they may.
        }
        c.sync();
        c.stop();
        let log = logged(&lines);
        assert_eq!(log.len(), 40, "{log:?}");
        for pair in log.chunks(2) {
            assert_eq!(pair, ["tcp_connect", "tcp_close"], "{log:?}");
        }
    }

    #[test]
    fn a_long_answer_waits_for_a_reader_and_does_not_block_the_carrier() {
        let (c, lines) = carrier();
        let mut s = connect(&c);
        s.write_all(b"b").unwrap();
        // The socket is full by now, with the host reading nothing.
        let mut other = connect(&c);
        other.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut other, 1), b"H", "the other connection is served");
        assert_eq!(read_n(&mut s, 1 << 20).len(), 1 << 20);
        drop((s, other));
        c.sync();
        c.stop();
        assert!(logged(&lines).iter().any(|l| l.starts_with("tcp_in 1048576 -> @1048576:")));
    }

    #[test]
    fn an_unplug_closes_after_what_was_sent_and_a_replug_undoes_it() {
        let (c, lines) = carrier();
        c.arm(Event::Unplug, "tcp-in=2").unwrap();
        let mut s = connect(&c);
        s.write_all(b"hh").unwrap();
        assert_eq!(read_n(&mut s, 2), b"HH");
        let mut rest = Vec::new();
        s.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, Vec::<u8>::new(), "a FIN");
        // While unplugged: accepted, and closed at once.
        let mut refused = connect(&c);
        refused.read_to_end(&mut rest).unwrap();
        c.sync();
        c.replug();
        let mut back = connect(&c);
        back.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut back, 1), b"H");
        drop((s, refused, back));
        c.sync();
        c.stop();
        let log = logged(&lines);
        assert_eq!(log[..4], ["tcp_connect", "tcp_out 6868 -> 2", "tcp_in 2 -> 4848", "unplug"]);
        assert_eq!(log[4], "tcp_refused");
        assert_eq!(log[5], "tcp_connect");
    }

    #[test]
    fn a_mark_fires_what_waits_for_the_model() {
        let (c, lines) = carrier();
        c.arm(Event::Mark("interrupt".into()), "tcp-model").unwrap();
        let mut s = connect(&c);
        s.write_all(b"mh").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H", "a mark does nothing to the device");
        drop(s);
        c.sync();
        c.stop();
        assert!(logged(&lines).contains(&"interrupt".to_owned()));
        assert!(c.arm(Event::Unplug, "in=2:1").is_err());
        assert!(c.arm(Event::Unplug, "tcp-in=0").is_err());
    }

    #[test]
    fn a_hang_up_and_a_violation_are_logged_and_counted() {
        let (c, lines) = carrier();
        let mut s = connect(&c);
        s.write_all(b"vhx").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H", "what was sent gets out first");
        let mut rest = Vec::new();
        s.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, Vec::<u8>::new());
        c.sync();
        c.stop();
        assert_eq!(c.violations(), 1);
        assert!(logged(&lines).contains(&"! v it is".to_owned()));
    }

    #[test]
    fn a_fresh_model_drops_the_old_connection_in_silence() {
        let (c, lines) = carrier();
        let mut s = connect(&c);
        s.write_all(b"h").unwrap();
        assert_eq!(read_n(&mut s, 1), b"H");
        // Not sync: it would wait for this connection to go.
        c.lock().flush();
        let before = logged(&lines);
        c.install(Box::new(Toy::default()));
        let mut rest = Vec::new();
        s.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, Vec::<u8>::new());
        c.stop();
        assert_eq!(logged(&lines), before, "no tcp_close");
    }
}
