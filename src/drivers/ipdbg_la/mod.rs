// Counterpart in sigrok:
//   libsigrok/src/hardware/ipdbg-la/api.c,
//   libsigrok/src/hardware/ipdbg-la/protocol.c,
//   Copyright (C) 2016 Eva Kissling <eva.kissling@bluewin.ch>,
//   with commits by Uwe Hermann, Daniel Anselmi, danselmi
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `ipdbg-la`: the IPDBG logic analyzer core in an FPGA, reached over TCP through an
//! IPDBG server such as `OpenOCD`'s (`conn=tcp-raw/HOST/PORT`).
//!
//! A scan connects, resets the core twice, reads its id and its widths and closes, and
//! opening the device connects again. A capture sends the trigger and the delay and
//! arms; the core then takes as long as its trigger and its design's sample signal
//! take, and sends all of its memory, of which the limit's first samples are kept:
//! those ahead of the delay, a trigger, and the rest. See [`protocol`].
//!
//! What differs from C:
//!
//! - **A capture has a header.** `dev_acquisition_start` (`api.c:240-258`) sends none,
//!   and sigrok-cli drops all that comes before one (`session.c:285-287`), so a capture
//!   of stock C prints nothing and exits 0. The core's maintainer's branch
//!   (`danselmi/libsigrok`, `ipdbg-la-rlc`) has it right after the delay, and it goes
//!   there here.
//! - **A close waits for the server.** That branch's `ipdbg_la_tcp_close` shuts the
//!   sending side and reads until the peer closes, up to 2 s, as "the IPDBG JTAG host
//!   in `OpenOCD` serves one connection per core", and stock's next connection is
//!   rejected. This does the same; nothing shows on the wire.
//! - **Widths it can't use are no device:** a short or missing answer to the sizes,
//!   where C reads an uninitialised `buf[8]` (`protocol.c:479-496`), and the widths
//!   [`protocol::parse_widths`] refuses.
//! - **A gone device ends things.** C's `data_available` says 0 for ever after a close,
//!   and `dev_acquisition_stop` spins with no bound (`api.c:268-275`). Here a close
//!   inside a capture fails it, a silent dump fails it at
//!   [`protocol::SILENCE_LIMIT`], and a stop waits [`STOP_SILENCE`] for a dump's rest.
//!   A write after a close kills C with `SIGPIPE`.
//! - **A wait for the trigger warns** at 30 s. It has no bound, as in C.
//! - **A limit past the memory warns.** C keeps its limit and says nothing
//!   (`api.c:212`).
//! - **The device can be opened again.** `dev_close` drops its `conn` (`api.c:172`) so
//!   that `dev_open` fails after it.
//! - **`conn=` with one field is no device,** where C crashes (`api.c:51`), and the
//!   port must be a number, where `getaddrinfo` also takes a service's name.
//! - A second capture on one connection starts by dropping what has arrived, as C
//!   would count it as data. After a stop gave up on a dump it connects again.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::serial::Port;
use crate::serial::tcpraw::TcpRaw;
use protocol::{Silence, Verdict};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
];
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];

/// A connect's bound per address, where C has the system's.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
/// How long a close waits for the peer to close too: the maintainer's branch's.
const CLOSE_WAIT: Duration = Duration::from_secs(2);
/// A wait of `tcp_receive_blocking`'s reads.
const RECEIVE_TRY: Duration = Duration::from_millis(5);
/// How long a capture's read waits between looks at the stop flag.
const STOP_CHECK: Duration = Duration::from_millis(20);
/// How long a stop waits on a dump's rest with nothing coming, where C spins for ever.
pub const STOP_SILENCE: Duration = Duration::from_secs(5);
/// What one read of a dump takes.
const READ_SIZE: usize = 16 * 1024;

/// A connection, as the driver needs one: a socket, or a test's fake.
pub trait Wire: Send {
    /// Reads what there is, up to `timeout`: 0 for nothing, an error for a close.
    ///
    /// # Errors
    ///
    /// A connection that has gone.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize>;
    /// Writes all of `data`.
    ///
    /// # Errors
    ///
    /// A connection that has gone, or no room for long.
    fn write(&mut self, data: &[u8]) -> Result<()>;
    /// Drops what has arrived.
    ///
    /// # Errors
    ///
    /// A connection that has gone.
    fn purge(&mut self) -> Result<()>;
    /// Closes.
    fn close(self: Box<Self>);
}

impl Wire for TcpRaw {
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        Port::read(self, buf, timeout)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        Port::write(self, data)
    }

    fn purge(&mut self) -> Result<()> {
        Port::purge(self)
    }

    fn close(self: Box<Self>) {
        (*self).close_waiting(CLOSE_WAIT);
    }
}

/// `ipdbg_la_tcp_open`, with its messages.
///
/// # Errors
///
/// A name that doesn't resolve, or no address that takes the connection.
fn connect(host: &str, port: Option<&str>) -> Result<Box<dyn Wire>> {
    let shown = port.unwrap_or("(no port)");
    let lookup = |why: &dyn std::fmt::Display| {
        let message = format!("Address lookup failed: {host}:{shown}: {why}");
        log::error!("{message}");
        Error::device(message)
    };
    let number = match port.map(str::parse::<u16>) {
        Some(Ok(n)) => n,
        // A missing port is 0 to `getaddrinfo`, which then connects to nothing.
        None => 0,
        Some(Err(_)) => return Err(lookup(&"the port is no number")),
    };
    let addrs: Vec<SocketAddr> = match (host, number).to_socket_addrs() {
        Ok(addrs) => addrs.collect(),
        Err(e) => return Err(lookup(&os_text(&e))),
    };
    let mut last = "no address".to_owned();
    for addr in addrs {
        match TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT) {
            Ok(stream) => {
                let name = format!("tcp-raw/{host}/{shown}");
                return Ok(Box::new(TcpRaw::from_stream(name, stream)?));
            }
            Err(e) => last = os_text(&e),
        }
    }
    let message = format!("Failed to connect to {host}:{shown}: {last}");
    log::error!("{message}");
    Err(Error::device(message))
}

/// An error's words, without Rust's ` (os error N)`.
fn os_text(e: &std::io::Error) -> String {
    let text = e.to_string();
    text.split(" (os error").next().unwrap_or_default().to_owned()
}

/// `tcp_receive_blocking`: fills `buf` within [`protocol::RECEIVE_WAIT`] and says how
/// much came. A close ends it, where C tries on. The time is taken ahead of each read,
/// so a stalled host still finds an answer that lies waiting.
fn receive_blocking(wire: &mut dyn Wire, buf: &mut [u8]) -> usize {
    let began = Instant::now();
    let mut received = 0;
    while received < buf.len() {
        let looked = Instant::now();
        match wire.read(&mut buf[received..], RECEIVE_TRY) {
            Ok(0) => {
                if looked.saturating_duration_since(began) >= protocol::RECEIVE_WAIT {
                    break;
                }
            }
            Ok(n) => received += n,
            Err(Error::Disconnected) => break,
            Err(e) => {
                log::error!("Receive error: {e}");
                break;
            }
        }
    }
    received
}

/// `tcp_send` with its warning: C goes on, and what comes next says the rest.
fn send_or_warn(wire: &mut dyn Wire, data: &[u8], what: &str) {
    if let Err(e) = wire.write(data) {
        log::error!("Send error: {e}");
        log::warn!("{what}");
    }
}

/// What a scan found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Widths {
    data: u32,
    addr: u32,
}

/// `scan` after the connect: `None` is no device.
fn probe(wire: &mut dyn Wire) -> Option<Widths> {
    send_or_warn(wire, &[protocol::CMD_RESET, protocol::CMD_RESET], "Couldn't send reset");
    send_or_warn(wire, &[protocol::CMD_GET_LA_ID], "Couldn't send ID request");
    let mut id = [0u8; 4];
    if receive_blocking(wire, &mut id) != id.len() {
        log::error!("Couldn't read device ID");
        return None;
    }
    if &id != protocol::ID {
        log::error!("Invalid device ID: expected 'IDBG', got '{}'.", id.escape_ascii());
        return None;
    }
    send_or_warn(wire, &[protocol::CMD_GET_BUS_WIDTHS], "Can't send read command");
    let mut sizes = [0u8; 8];
    if receive_blocking(wire, &mut sizes) != sizes.len() {
        log::warn!("Can't get address and data width from device");
        return None;
    }
    match protocol::parse_widths(&sizes) {
        Ok((data, addr)) => {
            log::debug!("addr_width = {addr}, data_width = {data}");
            Some(Widths { data, addr })
        }
        Err(message) => {
            log::error!("{message}");
            None
        }
    }
}

/// The `ipdbg-la` driver.
pub struct IpdbgLaDriver;

impl Driver for IpdbgLaDriver {
    fn id(&self) -> &'static str {
        "ipdbg-la"
    }

    fn long_name(&self) -> &'static str {
        "IPDBG LA"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let Some((host, port)) = protocol::split_conn(&conn) else {
            return Ok(Vec::new());
        };
        let (host, port) = (host.to_owned(), port.map(str::to_owned));
        let opener: Opener = Arc::new(move || connect(&host, port.as_deref()));
        Ok(scan_with(&opener).map_or_else(Vec::new, |d| vec![Box::new(d) as Box<dyn Device>]))
    }
}

/// How the device connects, each time.
type Opener = Arc<dyn Fn() -> Result<Box<dyn Wire>> + Send + Sync>;

/// A scan through `opener`: a failed connect or probe is no device, as C's `NULL`.
fn scan_with(opener: &Opener) -> Option<IpdbgLaDevice> {
    let mut wire = opener().ok()?;
    let widths = probe(wire.as_mut());
    wire.close();
    Some(IpdbgLaDevice::new(widths?, Arc::clone(opener)))
}

/// The state a capture's thread leaves behind.
#[derive(Default)]
struct Link {
    wire: Option<Box<dyn Wire>>,
    /// A stop gave up on a dump, so what still comes isn't for the next capture.
    doubtful: bool,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One core.
pub struct IpdbgLaDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    opener: Opener,
    data_bytes: usize,
    addr_bytes: usize,
    limit_samples_max: u64,
    limit_samples: u64,
    capture_ratio: u64,
    trigger: Option<Trigger>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

impl IpdbgLaDevice {
    fn new(widths: Widths, opener: Opener) -> IpdbgLaDevice {
        let max = 1u64 << widths.addr;
        IpdbgLaDevice {
            info: DeviceInfo {
                driver: "ipdbg-la",
                vendor: "ipdbg.org".to_owned(),
                model: "IPDBG LA".to_owned(),
                version: "v1.0".to_owned(),
                serial: String::new(),
                conn: None,
            },
            channels: (0..widths.data as usize)
                .map(|i| Channel::new(i, ChannelKind::Logic, format!("CH{i}")))
                .collect(),
            opener,
            data_bytes: widths.data.div_ceil(8) as usize,
            addr_bytes: widths.addr.div_ceil(8) as usize,
            limit_samples_max: max,
            limit_samples: max,
            capture_ratio: 50,
            trigger: None,
            link: Arc::new(Mutex::new(Link::default())),
            worker: None,
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("ipdbg-la lock")
    }

    /// Asks a running capture to end and waits for its thread.
    fn end_worker(&mut self) {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
    }
}

impl Drop for IpdbgLaDevice {
    fn drop(&mut self) {
        self.end_worker();
    }
}

impl Device for IpdbgLaDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::CaptureRatio => Ok(Value::U64(self.capture_ratio)),
            Key::LimitSamples => Ok(Value::U64(self.limit_samples)),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::CaptureRatio => {
                let ratio = value.as_u64().ok_or_else(bad)?;
                // hwdriver.c's check of the key, ahead of the driver.
                if ratio > 100 {
                    return Err(bad());
                }
                self.capture_ratio = ratio;
                Ok(())
            }
            Key::LimitSamples => {
                let limit = value.as_u64().ok_or_else(bad)?;
                if limit == 0 {
                    return Err(bad());
                }
                if limit <= self.limit_samples_max {
                    self.limit_samples = limit;
                } else {
                    log::warn!(
                        "{limit} samples are more than the core's memory of {}: the limit stays {}.",
                        self.limit_samples_max,
                        self.limit_samples
                    );
                }
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::TriggerMatch => Ok(ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec())),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        self.trigger = trigger;
        Ok(())
    }

    fn open(&mut self) -> Result<()> {
        if self.lock().wire.is_some() {
            return Ok(());
        }
        let wire = (self.opener)()?;
        *self.lock() = Link { wire: Some(wire), doubtful: false };
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.end_worker();
        let wire = self.lock().wire.take();
        if let Some(wire) = wire {
            wire.close();
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // The thread gives the wire back ahead of End, so a client that has seen
        // End finds it here. What is left of the thread is that End: not joined, as
        // a full queue would hold it.
        if self.worker.is_some() {
            if self.lock().wire.is_none() {
                return Err(Error::Busy);
            }
            self.worker = None;
        }
        let (wire, doubtful) = {
            let mut link = self.lock();
            (link.wire.take(), std::mem::take(&mut link.doubtful))
        };
        let Some(mut wire) = wire else {
            return Err(Error::device("device is not open"));
        };
        if doubtful {
            // A new connection can't hold an old answer, and the core may be in the
            // dump that was given up on, where only a reset is heard.
            wire.close();
            wire = (self.opener)()?;
            send_or_warn(wire.as_mut(), &[protocol::CMD_RESET], "Couldn't send reset");
        } else {
            match wire.purge() {
                Err(Error::Disconnected) | Ok(()) => {}
                Err(e) => log::debug!("A purge failed: {e}"),
            }
        }
        let enabled: Vec<bool> = self.channels.iter().map(|c| c.enabled).collect();
        let fields = protocol::convert_trigger(self.trigger.as_ref(), self.data_bytes, |c| {
            enabled.get(c).copied().unwrap_or(false)
        });
        let delay = protocol::delay_value(self.limit_samples, self.capture_ratio);
        let unit = self.data_bytes as u64;
        let capture = Capture {
            unit_size: self.data_bytes,
            delay_bytes: delay.saturating_mul(unit),
            keep_bytes: self.limit_samples.saturating_mul(unit),
            total_bytes: self.limit_samples_max.saturating_mul(unit),
            stop_silence: STOP_SILENCE,
        };
        if let Err(e) = wire.write(&protocol::configuration(&fields, delay, self.addr_bytes)) {
            log::error!("Send error: {e}");
            self.lock().wire = Some(wire);
            return Err(e);
        }
        let link = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("ipdbg-la acquisition".into())
            .spawn(move || {
                let (outcome, doubtful) = capture.run(wire.as_mut(), &sink, &stop2);
                *link.lock().expect("ipdbg-la lock") = Link { wire: Some(wire), doubtful };
                sink.end_while(outcome, || !stop2.load(Ordering::SeqCst));
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.end_worker();
        Ok(())
    }
}

/// `ipdbg_la_receive_data` and `dev_acquisition_stop`, on a thread.
struct Capture {
    unit_size: usize,
    delay_bytes: u64,
    keep_bytes: u64,
    total_bytes: u64,
    stop_silence: Duration,
}

impl Capture {
    /// What the capture came to, and whether what may still come is doubtful.
    fn run(&self, wire: &mut dyn Wire, sink: &PacketSink, stop: &AtomicBool) -> (Result<()>, bool) {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            send_or_warn(wire, &[protocol::CMD_RESET], "Couldn't send reset");
            return (Ok(()), false);
        }
        if let Err(e) = wire.write(&[protocol::CMD_START]) {
            log::error!("Send error: {e}");
            return (Err(e), false);
        }
        let mut kept: Vec<u8> = Vec::new();
        let mut received = 0u64;
        let mut silence = Silence::new(Instant::now(), self.total_bytes);
        let mut buf = vec![0u8; READ_SIZE];
        while received < self.total_bytes {
            if stop.load(Ordering::SeqCst) {
                return (Ok(()), self.stop_in(wire, received));
            }
            silence.look(Instant::now());
            match wire.read(&mut buf, STOP_CHECK) {
                Ok(0) => match silence.empty() {
                    Verdict::Wait => {}
                    Verdict::Warn(text) => log::warn!("{text}"),
                    Verdict::Fail(text) => {
                        log::error!("{text}");
                        return (Err(Error::device(text)), true);
                    }
                },
                Ok(n) => {
                    if let Some(text) = silence.got(Instant::now(), n as u64) {
                        log::warn!("{text}");
                    }
                    let room = self.keep_bytes.saturating_sub(received);
                    let take = room.min(n as u64) as usize;
                    kept.extend_from_slice(&buf[..take]);
                    received = received.saturating_add(n as u64);
                }
                Err(Error::Disconnected) => {
                    let text = format!(
                        "The server closed the connection, at {received} of {} bytes.",
                        self.total_bytes
                    );
                    log::error!("{text}");
                    return (Err(Error::device(text)), false);
                }
                Err(e) => {
                    log::error!("Receive error: {e}");
                    return (Err(e), false);
                }
            }
        }
        self.send_samples(kept, sink, stop);
        (Ok(()), false)
    }

    /// The samples ahead of the delay, a trigger, and those after. The order is C's:
    /// no logic packet when no sample comes ahead of the trigger.
    fn send_samples(&self, mut kept: Vec<u8>, sink: &PacketSink, stop: &AtomicBool) {
        let go_on = || !stop.load(Ordering::SeqCst);
        let after = kept.split_off(kept.len().min(self.delay_bytes as usize));
        let before = kept;
        let _ = (before.is_empty()
            || sink.send_while(Packet::logic(self.unit_size, before), go_on))
            && sink.send_while(Packet::Trigger, go_on)
            && sink.send_while(Packet::logic(self.unit_size, after), go_on);
    }

    /// `dev_acquisition_stop`: a dump begun is read to its end, if it comes, and then
    /// the core is reset. True if it didn't all come.
    fn stop_in(&self, wire: &mut dyn Wire, mut received: u64) -> bool {
        let mut gave_up = false;
        if received > 0 {
            let mut buf = vec![0u8; READ_SIZE];
            let mut last = Instant::now();
            while received < self.total_bytes {
                let looked = Instant::now();
                match wire.read(&mut buf, STOP_CHECK) {
                    Ok(0) => {
                        if looked.saturating_duration_since(last) >= self.stop_silence {
                            log::warn!(
                                "A stop gave up on the dump, at {received} of {} bytes.",
                                self.total_bytes
                            );
                            gave_up = true;
                            break;
                        }
                    }
                    Ok(n) => {
                        received = received.saturating_add(n as u64);
                        last = Instant::now();
                    }
                    Err(_) => break,
                }
            }
        }
        send_or_warn(wire, &[protocol::CMD_RESET], "Couldn't send reset");
        gave_up
    }
}

#[cfg(test)]
mod tests;
