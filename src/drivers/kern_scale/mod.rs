// Counterpart in sigrok:
//   libsigrok/src/hardware/kern-scale/api.c,
//   libsigrok/src/hardware/kern-scale/protocol.c,
//   libsigrok/src/hardware/kern-scale/protocol.h,
//   Copyright (C) 2015 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Gerhard Sittig, Daniel Elstner,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! KERN EW and EG-N scales on their RS-232 port (`kern-ew-6200-2nm`).
//!
//! `conn=` names the port and `serialcomm=` overrides 1200/8n2, as the scale's
//! menu can. A scan sends `O1` CR LF, which asks for continuous output, and looks
//! for a packet for 3 s or 128 bytes. A capture sends `O1` again and reads what
//! comes, until a limit or a stop. The scale answers `A00` or `E01`, which nothing
//! reads: the packet search steps over it. See [`protocol`] for the packets.
//!
//! Where this differs from C, each with its cause:
//!
//! - A stream of 14-byte packets gives each packet. C steps 15 over a packet of
//!   14 (`protocol.c:89-92`), which eats the next one's sign, so it reads every
//!   second packet, and needs a 15th byte to read the last. A real unit sends 14
//!   by default, by the captures in sigrok-dumps. 15-byte streams are the same.
//! - A port that hangs up ends the capture, and so does a read that fails. C reads
//!   only when `revents` is exactly `G_IO_IN` (`protocol.c:121`), and logs a failed
//!   read and goes on (`protocol.c:81-84`), for ever at a sample limit.
//! - At a sample limit with no time limit, [`SILENCE_BOUND`] with no usable packet
//!   ends the capture with an error: bytes that make no packet don't count. C waits
//!   for ever. The 10 s is a choice, and it can be wrong: by the manual (8.6) `O1`
//!   lasts until power off, so a scale that was power cycled and is set to print on a
//!   key or on a stable value goes quiet, and this gives up where C keeps waiting. A
//!   scale that answered `E01` is said to have. With a time limit, or none, the
//!   capture waits as in C.
//! - A write of `O1` that finds no room in 5 s fails: no device at a scan, with a
//!   warning, and a refused start at a capture. C waits for ever (`api.c:84`,
//!   `api.c:142`).
//! - A number behind spaces reads as that number, and a digit field that is no
//!   number is no reading: see [`protocol`]. It is stepped over a byte at a time
//!   like any other bytes, with a warning the first time, and so a lone sign ahead
//!   of a good packet costs that sign. C reads 0 for both, with exit 0.
//! - A second capture on an open port drops what came since the first and starts
//!   with an empty buffer. C reads old packets as readings, and joins the tail of
//!   its last read to the new bytes.
//!
//! Kept from C, as nothing says otherwise:
//!
//! - A sample limit is looked at after a whole read, so a read holding two packets
//!   sends both. A client that falls behind by a packet may so get one too many.
//! - An `E` packet is infinity whatever its digits and sign say, and a packet with
//!   no unit it knows still counts.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::Head;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The driver's name in sigrok.
pub const DRIVER: &str = "kern-ew-6200-2nm";
const SERIALCOMM: &str = "1200/8n2";
const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Scale];
const WRITE_ONLY: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, WRITE_ONLY),
    OptionDesc::new(Key::LimitMsec, WRITE_ONLY),
];

/// Continuous output, stable and unstable values.
const O1: &[u8] = b"O1\r\n";
/// The scale's answer when it refuses a command (the manual's 8.6).
const E01: &[u8] = b"E01\r\n";
/// How long a scan looks for a packet (`api.c:91`).
const DETECT_TIMEOUT: Duration = Duration::from_millis(3000);
/// A scan's buffer (`api.c:52`).
const DETECT_BYTES: usize = 128;
/// A capture's buffer (`SCALE_BUFSIZE`).
const BUF_SIZE: usize = 256;
/// How long a read of a capture waits, `serial_source_add`'s 50 ms.
const READ_TIMEOUT: Duration = Duration::from_millis(50);
/// How long a thread that was told to stop waits to put its `End` in a queue no
/// one reads. A close joins it. An output may have rows pending on the `End`, so
/// this outlasts a reader's short stall.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long no usable packet may come at a sample limit alone. A choice. All the real
/// evidence is two captures in sigrok-dumps, at 1200/8n2 with whatever output mode the
/// unit had: a stable packet about every 500 ms, an unstable or error one about every
/// 128 ms, and a longest silence of 2018 ms at a tare, so this is 5 times that.
const SILENCE_BOUND: Duration = Duration::from_secs(10);

/// The driver.
pub struct KernScaleDriver;

impl Driver for KernScaleDriver {
    fn id(&self) -> &'static str {
        DRIVER
    }

    fn long_name(&self) -> &'static str {
        "KERN EW 6200-2NM"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = SERIALCOMM.to_owned();
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        // A serialcomm that does not parse or a port that does not open: no
        // device, as `serial_open`'s failure there.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| crate::serial::open(&conn, params).map(|port| (params, port)));
        let (params, mut port) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{DRIVER}: {e}");
                return Ok(Vec::new());
            }
        };
        if !probe(port.as_mut(), &params, &conn, DETECT_TIMEOUT) {
            return Ok(Vec::new());
        }
        // `serial_close`; `dev_open` opens the port again.
        drop(port);
        Ok(vec![Box::new(KernScaleDevice::new(conn, params, None))])
    }
}

/// Asks for continuous output, with what went wrong in words of its own.
fn send_o1(port: &mut dyn Port) -> Result<()> {
    log::trace!("Set O1 mode (continuous values, stable and unstable ones).");
    port.write(O1).map_err(|e| Error::device(format!("could not send O1: {e}")))
}

/// What a scan does on the open port.
fn probe(port: &mut dyn Port, params: &Params, conn: &str, timeout: Duration) -> bool {
    log::info!("Probing serial port {conn}.");
    if let Err(e) = send_o1(port) {
        log::warn!("{conn}: {e}.");
        return false;
    }
    if !detect(port, params, timeout) {
        return false;
    }
    log::info!("Found device on port {conn}.");
    true
}

/// `serial_stream_detect`: reads byte by byte until 15 bytes start a packet,
/// [`DETECT_BYTES`] came, or `timeout` passed. A read that fails is no byte, and
/// the time still runs.
fn detect(port: &mut dyn Port, params: &Params, timeout: Duration) -> bool {
    let byte_delay = crate::serial::serial_timeout(params, 1);
    let start = Instant::now();
    let mut buf = Vec::with_capacity(DETECT_BYTES);
    let mut check = 0;
    while buf.len() < DETECT_BYTES {
        let mut byte = [0u8];
        match port.read(&mut byte, byte_delay) {
            Ok(0) => {}
            Ok(_) => buf.push(byte[0]),
            Err(e) => {
                log::debug!("Scan read: {e}.");
                std::thread::sleep(byte_delay);
            }
        }
        if buf.len() - check >= protocol::PACKET_SIZE {
            if protocol::valid(&buf[check..]).is_some() {
                return true;
            }
            check += 1;
        }
        if start.elapsed() >= timeout {
            break;
        }
    }
    log::info!("Didn't find a valid packet (read {} bytes).", buf.len());
    false
}

/// When a capture that only a sample limit can end has heard enough of nothing.
/// Times come in as arguments, so that its tests take no time.
#[derive(Debug)]
struct Silence {
    bound: Duration,
    since: Instant,
}

impl Silence {
    fn new(bound: Duration, now: Instant) -> Silence {
        Silence { bound, since: now }
    }

    /// A packet came, and went out to the client: `now` is taken after that, so
    /// that a slow client is no silence.
    fn heard(&mut self, now: Instant) {
        self.since = now;
    }

    fn over(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.since) >= self.bound
    }
}

/// The software limits, which a set changes under a running capture, as in C.
#[derive(Debug, Default)]
struct Limits {
    samples: AtomicU64,
    msec: AtomicU64,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One scale.
pub struct KernScaleDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limits: Arc<Limits>,
    /// [`SILENCE_BOUND`], which a test shortens.
    silence_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port, unless a capture holds it. It comes back before `End`.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    /// A capture ran on this open, so what is queued is old.
    captured: bool,
    worker: Option<Worker>,
}

impl KernScaleDevice {
    fn new(conn: String, params: Params, port: Option<Box<dyn Port>>) -> Self {
        KernScaleDevice {
            info: DeviceInfo {
                driver: DRIVER,
                vendor: "KERN".to_owned(),
                model: "EW 6200-2NM".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "Mass".to_owned())],
            conn,
            params,
            limits: Arc::new(Limits::default()),
            silence_bound: SILENCE_BOUND,
            pending: port,
            port: Arc::new(Mutex::new(None)),
            captured: false,
            worker: None,
        }
    }

    /// A device on an already open port. No scan is run.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        Self::new(name, params, Some(port))
    }

    /// Tells a worker to stop and waits for it.
    fn join(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for KernScaleDevice {
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
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    /// C has no `config_get`.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        match key {
            Key::LimitSamples => self.limits.samples.store(number()?, Ordering::SeqCst),
            Key::LimitMsec => self.limits.msec.store(number()?, Ordering::SeqCst),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            // Flushed as `serial_open` flushes a port it opens.
            Some(mut p) => {
                p.purge()?;
                p
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        *slot = Some(port);
        drop(slot);
        self.captured = false;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.join();
        self.port.lock().expect("port lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let port = self.port.lock().expect("port lock").take();
        let Some(mut port) = port else {
            // No port and a worker that lives: it holds the port. The worker
            // gives it back before its End, so a client that saw an End is never
            // told this.
            if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
                return Err(Error::Busy);
            }
            return Err(Error::device("device is not open"));
        };
        // A last capture's thread may wait to send its End to a queue nobody
        // reads: tell it, and it gives up within a bound.
        self.join();
        if self.captured
            && let Err(e) = port.purge()
        {
            log::debug!("Flush: {e}.");
        }
        if let Err(e) = send_o1(port.as_mut()) {
            *self.port.lock().expect("port lock") = Some(port);
            return Err(e);
        }
        self.captured = true;
        let capture = Capture {
            enabled: self.channels.first().is_some_and(|c| c.enabled),
            limits: Arc::clone(&self.limits),
            silence_bound: self.silence_bound,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(format!("{DRIVER} acquisition"))
            .spawn(move || {
                let outcome = capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                send_end(&sink, &stop2, outcome);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// `std_serial_dev_acquisition_stop` has nothing to send: the flag ends the
    /// capture. A close joins.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for KernScaleDevice {
    fn drop(&mut self) {
        self.join();
    }
}

/// `End`, waiting for room for as long as nobody asked to stop, and after a stop
/// only for [`END_AFTER_STOP`]: a close joins this thread.
fn send_end(sink: &PacketSink, stop: &AtomicBool, outcome: Result<()>) {
    let stopped_at = std::cell::Cell::new(None);
    sink.end_while(outcome, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

/// One capture, on its own thread.
struct Capture {
    enabled: bool,
    limits: Arc<Limits>,
    silence_bound: Duration,
}

impl Capture {
    /// Runs until a limit, a stop, a port that fails or a consumer that left.
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let send = |p: Packet| sink.send_while(p, || !stop.load(Ordering::SeqCst));
        if !send(Packet::header()) {
            return Ok(());
        }
        // After the send, as a slow client is no silence and no measuring time.
        let started = Instant::now();
        let mut silence = Silence::new(self.silence_bound, started);
        let mut samples = 0u64;
        let mut refused = false;
        let mut warned = false;
        let mut buf: Vec<u8> = Vec::with_capacity(BUF_SIZE);
        let mut chunk = [0u8; BUF_SIZE];
        while !stop.load(Ordering::SeqCst) {
            let room = BUF_SIZE - buf.len();
            let n = port.read(&mut chunk[..room], READ_TIMEOUT).map_err(|e| {
                log::error!("Serial port read error: {e}.");
                e
            })?;
            buf.extend_from_slice(&chunk[..n]);
            if !refused && buf.windows(E01.len()).any(|w| w == E01) {
                refused = true;
                log::warn!("The scale answered E01 to O1.");
            }
            // handle_new_data: as many packets as there are.
            let before = samples;
            let mut at = 0;
            loop {
                match protocol::head(&buf[at..]) {
                    Head::Wait => break,
                    Head::Skip => at += 1,
                    Head::Packet(len) => {
                        let packet = &buf[at..at + len];
                        let Some(reading) = protocol::parse(packet) else {
                            if warned {
                                log::debug!("No reading in {}.", packet.escape_ascii());
                            } else {
                                warned = true;
                                log::warn!(
                                    "Dropped a packet whose digits are no number: {}.",
                                    packet.escape_ascii()
                                );
                            }
                            at += 1;
                            continue;
                        };
                        // A channel that is off gets nothing, and it still counts.
                        if self.enabled && !send(analog(reading)) {
                            return Ok(());
                        }
                        samples += 1;
                        at += len;
                    }
                }
            }
            buf.drain(..at);
            if samples != before {
                silence.heard(Instant::now());
            }
            // The limits are looked at once all of a read is out.
            let limit = self.limits.samples.load(Ordering::SeqCst);
            if limit > 0 && samples >= limit {
                return Ok(());
            }
            let msec = self.limits.msec.load(Ordering::SeqCst);
            if msec > 0 && started.elapsed() > Duration::from_millis(msec) {
                return Ok(());
            }
            if limit > 0 && msec == 0 && silence.over(Instant::now()) {
                let seconds = self.silence_bound.as_secs();
                let about = if refused {
                    "It answered E01 to O1, which tells it to send without being asked."
                } else {
                    "It was told to send without being asked, so it is off, its cable is out, \
                     its line differs, or a power cycle took it back to another output mode."
                };
                log::error!("No usable packet from the scale for {seconds} s. {about}");
                let and = if refused { ", and it answered E01" } else { "" };
                return Err(Error::device(format!(
                    "no usable packet from the scale for {seconds} s{and}"
                )));
            }
        }
        Ok(())
    }
}

/// `handle_packet`'s analog packet.
fn analog(r: protocol::Reading) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![r.value],
        mq: Mq::Mass,
        flags: r.flags,
        unit: r.unit,
        digits: r.digits,
    })
}

#[cfg(test)]
mod tests;
