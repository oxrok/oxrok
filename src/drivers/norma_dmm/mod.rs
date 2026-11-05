// Counterpart in sigrok:
//   libsigrok/src/hardware/norma-dmm/api.c,
//   libsigrok/src/hardware/norma-dmm/protocol.c,
//   Copyright (C) 2013 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Norma DM9x0 bench multimeters, sold by Siemens as the B102x, on a serial port
//! at 4800 baud. C registers two drivers from one directory, `norma-dmm` and
//! `siemens-b102x`, which differ only in the names they give a meter, and so does
//! this.
//!
//! A scan opens the port, waits 150 ms and asks `IDN?` up to seven times, each
//! read with 2 s. A line that does not match on the fourth try waits 5 s more, for
//! a capacitor in the interface; and the LF after a CR is an empty read that takes
//! a try, so under CR LF that wait is not reached. It closes the port; an open
//! opens it again.
//!
//! A capture has one `STATUS?` out at a time. The first goes 100 ms after the
//! start and each next one in the pass that read a line, or 2 s after the last if
//! no line came, for as long as the capture runs. A line ends at an LF. `--get`
//! has nothing to answer: C has no `config_get`.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - Hex digits have their values, and a pair of digits is a byte: see
//!   [`protocol::hex_digit`].
//! - Part of a line is dropped when its request expires, and a line longer than
//!   20 digits is an error, so what came of two replies is no reading. C keeps
//!   the part, adds the next reply to it and cuts it at 20.
//! - A line that fills the buffer is one bad line, where C stops reading for good.
//! - A reply a capture left on its way is no sample of the next one on an open
//!   device: C's request state stays, and so does the wire, but it takes the line.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use protocol::{BUFSIZE, Brand, ID_BUF, IDN, Reading, SERIALCOMM, STATUS};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
const SET: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET),
    OptionDesc::new(Key::LimitMsec, SET),
];

/// The scan's tries of `IDN?`.
const ID_TRIES: usize = 7;
/// The try after which a line that does not match waits, from 0.
const CHARGE_TRY: usize = 3;
/// `NMADMM_TIMEOUT_MS`: a scan's read, and a request's life.
const REQUEST_TIMEOUT: Duration = Duration::from_millis(2000);
/// The session source's timeout in `dev_acquisition_start`.
const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// `serial_readline`'s pause after a read that gave nothing.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(2);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The scan's two waits, which a test does not want to sit through.
#[derive(Clone, Copy, Debug)]
struct ScanWaits {
    /// "Wait a little to allow serial port to settle."
    settle: Duration,
    /// "Waiting 5s to allow interface to settle."
    charge: Duration,
    /// Each read's.
    read: Duration,
}

const SCAN_WAITS: ScanWaits = ScanWaits {
    settle: Duration::from_millis(150),
    charge: Duration::from_secs(5),
    read: REQUEST_TIMEOUT,
};

/// The `norma-dmm` driver.
pub struct NormaDmmDriver;
/// The `siemens-b102x` driver.
pub struct SiemensB102xDriver;

fn scan(
    brand: Brand,
    driver: &'static str,
    options: &[(Key, Value)],
) -> Result<Vec<Box<dyn Device>>> {
    let mut conn = None;
    let mut serialcomm = SERIALCOMM.to_owned();
    for (key, value) in options {
        match key {
            Key::Conn => conn = value.as_str().map(str::to_owned),
            Key::Serialcomm => {
                value.as_str().ok_or_else(|| Error::arg("serialcomm"))?.clone_into(&mut serialcomm);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
    }
    let Some(conn) = conn else {
        return Ok(Vec::new());
    };
    let opened = Params::parse(&serialcomm)
        .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
    let (mut port, params) = match opened {
        Ok(opened) => opened,
        Err(e) => {
            log::error!("{e}");
            return Ok(Vec::new());
        }
    };
    let Some((meter_type, version)) = probe(port.as_mut(), params, SCAN_WAITS) else {
        return Ok(Vec::new());
    };
    log::trace!("{} {} DMM {version} detected!", brand.name(), brand.model(meter_type));
    let device = NormaDevice::new(driver, brand, meter_type, version, conn, params);
    Ok(vec![Box::new(device)])
}

/// `serial_readline`: up to a CR or an LF, which is left out, or up to what fits
/// or to the end of the time. A failed read counts as nothing having come.
fn read_line(port: &mut dyn Port, timeout: Duration) -> Vec<u8> {
    let deadline = Instant::now() + timeout;
    let mut line = Vec::new();
    while line.len() < ID_BUF - 1 {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        let mut byte = [0];
        match read_blocking(port, &mut byte, left) {
            Ok(1) => {
                if byte[0] == b'\r' || byte[0] == b'\n' {
                    break;
                }
                line.push(byte[0]);
            }
            Ok(_) => {}
            Err(_) => std::thread::sleep(EMPTY_READ_PAUSE.min(left)),
        }
    }
    line
}

/// The scan past the open: the type and version, if something answers as a meter.
fn probe(port: &mut dyn Port, params: Params, waits: ScanWaits) -> Option<(u32, String)> {
    let timeout = request_timeout(&params, IDN);
    std::thread::sleep(waits.settle);
    for attempt in 0..ID_TRIES {
        if port.write_timeout(IDN, timeout).is_err() {
            log::error!("Unable to send identification request.");
            return None;
        }
        let line = read_line(port, waits.read);
        if line.is_empty() {
            continue;
        }
        log::debug!("Received {}: '{}'.", line.len(), String::from_utf8_lossy(&line));
        if let Some(found) = protocol::parse_id(&line) {
            return Some(found);
        }
        if attempt == CHARGE_TRY {
            log::info!("Waiting 5s to allow interface to settle.");
            std::thread::sleep(waits.charge);
        }
    }
    None
}

fn request_timeout(params: &Params, request: &[u8]) -> Duration {
    serial_timeout(params, u32::try_from(request.len()).unwrap_or(u32::MAX))
}

/// The port, and what stays of the request state from one capture to the next,
/// as it does in C's `dev_context`.
struct Link {
    port: Option<Box<dyn Port>>,
    params: Params,
    /// A capture ended with a request unanswered, sent then.
    open_request: Option<Instant>,
}

/// One meter.
pub struct NormaDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    limit_samples: u64,
    limit_msec: u64,
    meter_type: u32,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    request_timeout: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl NormaDevice {
    fn new(
        driver: &'static str,
        brand: Brand,
        meter_type: u32,
        version: String,
        conn: String,
        params: Params,
    ) -> Self {
        NormaDevice {
            info: DeviceInfo {
                driver,
                vendor: brand.name().to_owned(),
                model: brand.model(meter_type).to_owned(),
                version,
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            conn,
            limit_samples: 0,
            limit_msec: 0,
            meter_type,
            pending: None,
            link: Arc::new(Mutex::new(Link { port: None, params, open_request: None })),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            request_timeout: REQUEST_TIMEOUT,
        }
    }

    /// A device on a port that stands in for the hardware, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>, meter_type: u32) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device =
            Self::new("norma-dmm", Brand::Norma, meter_type, String::new(), conn, params);
        device.pending = Some(port);
        device
    }

    /// Sets the old worker's stop and joins it: an End stuck on a queue no one
    /// reads gives up within its bound.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for NormaDevice {
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

    fn options(&self, _group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(DEV_OPTIONS.to_vec())
    }

    /// C has no `config_get`.
    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        // `core::config_set` refuses a 0 and says so; this is for a library
        // caller that comes straight here.
        let positive = || {
            value
                .as_u64()
                .filter(|n| *n > 0)
                .ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())))
        };
        match key {
            Key::LimitSamples => self.limit_samples = positive()?,
            Key::LimitMsec => self.limit_msec = positive()?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(port) => port,
            None => crate::serial::open(&self.conn, link.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        self.link.lock().expect("link lock").port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        if self.link.lock().expect("link lock").port.is_none() {
            return Err(Error::device("device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            meter_type: self.meter_type,
            enabled: self.channels[0].enabled,
            request_timeout: self.request_timeout,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2);
            });
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.capturing.store(false, Ordering::SeqCst);
                Err(Error::Io(e))
            }
        }
    }

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes
    /// to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and
/// after a stop only for [`END_AFTER_STOP`]: a close joins this thread.
fn send_end(sink: &PacketSink, stop: &AtomicBool) {
    let stopped_at = std::cell::Cell::new(None);
    sink.send_while(Packet::End, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

/// What has come of a line. A line that fills the buffer has its tail thrown away
/// up to its end, and is one bad line: what is kept is far over the 20 digits.
#[derive(Default)]
struct LineBuffer {
    bytes: Vec<u8>,
}

impl LineBuffer {
    fn clear(&mut self) {
        self.bytes.clear();
    }

    /// One byte: the whole line, when it was the last.
    fn push(&mut self, byte: u8) -> Option<Vec<u8>> {
        if self.bytes.len() < BUFSIZE - 1 {
            self.bytes.push(byte);
        }
        (byte == b'\n').then(|| std::mem::take(&mut self.bytes))
    }
}

/// One capture, on its own thread: `norma_dmm_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    meter_type: u32,
    /// `-C`, which C ignores: its packets go out whatever it says.
    enabled: bool,
    request_timeout: Duration,
}

/// `last_req_pending` and `req_sent_at`.
struct Requests {
    sent_at: Option<Instant>,
    /// The next line answers a request of the capture before.
    drop_next: bool,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
            return;
        }
        let open = self.link.lock().expect("link lock").open_request.take();
        let mut requests = Requests { sent_at: open, drop_next: open.is_some() };
        self.passes(sink, stop, &mut requests);
        self.link.lock().expect("link lock").open_request = requests.sent_at;
    }

    /// Reads up to the end of a line, as a pass of C's callback: `None` when it
    /// has not come to a pass yet, `Some(None)` when it has with no line. C's
    /// read loop stops with the line, or with what is there.
    fn pass(&self, buffer: &mut LineBuffer, due: Instant) -> Option<Option<Vec<u8>>> {
        let mut link = self.link.lock().expect("link lock");
        let port = link.port.as_mut()?.as_mut();
        let mut byte = [0];
        let wait = due.saturating_duration_since(Instant::now()).min(STOP_CHECK);
        match read_blocking(port, &mut byte, wait) {
            Ok(1) => {}
            Ok(_) => return (Instant::now() >= due).then_some(None),
            Err(e) => {
                // A port that has gone: revents is no G_IO_IN, so C reads nothing
                // and its next request is what fails.
                log::debug!("Read failed: {e}.");
                drop(link);
                std::thread::sleep(STOP_CHECK);
                return (Instant::now() >= due).then_some(None);
            }
        }
        if let Some(line) = buffer.push(byte[0]) {
            return Some(Some(line));
        }
        // The most one line can be, so a flood gives way to the checks.
        for _ in 0..BUFSIZE {
            match read_blocking(port, &mut byte, Duration::ZERO) {
                Ok(1) => {
                    if let Some(line) = buffer.push(byte[0]) {
                        return Some(Some(line));
                    }
                }
                _ => break,
            }
        }
        Some(None)
    }

    /// What a finished line says: a sample to send, if it holds one.
    fn take_line(&self, line: &[u8], requests: &mut Requests) -> Option<Reading> {
        requests.sent_at = None;
        if std::mem::take(&mut requests.drop_next) {
            log::debug!("Dropping a reply from the capture before.");
            return None;
        }
        protocol::process_line(line, self.meter_type)
    }

    fn send_request(&self, requests: &mut Requests) -> bool {
        let mut link = self.link.lock().expect("link lock");
        let timeout = request_timeout(&link.params, STATUS);
        let Some(port) = link.port.as_mut() else {
            return false;
        };
        log::trace!("Sending request: 'STATUS?'.");
        requests.sent_at = Some(Instant::now());
        if port.write_timeout(STATUS, timeout).is_err() {
            requests.sent_at = None;
            log::error!("Unable to send request.");
            return false;
        }
        requests.sent_at = Some(Instant::now());
        true
    }

    fn passes(&self, sink: &PacketSink, stop: &AtomicBool, requests: &mut Requests) {
        let go_on = || !stop.load(Ordering::SeqCst);
        let started = Instant::now();
        let mut buffer = LineBuffer::default();
        let mut samples = 0u64;
        let mut due = started + POLL_INTERVAL;
        loop {
            if stop.load(Ordering::SeqCst) {
                return;
            }
            let Some(line) = self.pass(&mut buffer, due) else {
                continue;
            };
            due = Instant::now() + POLL_INTERVAL;
            if let Some(line) = line
                && let Some(reading) = self.take_line(&line, requests)
            {
                if self.enabled && !sink.send_while(packet(&reading), go_on) {
                    return;
                }
                samples += 1;
            }
            let time_up =
                self.limit_msec > 0 && started.elapsed() > Duration::from_millis(self.limit_msec);
            if time_up || (self.limit_samples > 0 && samples >= self.limit_samples) {
                return;
            }
            if requests.sent_at.is_some_and(|at| at.elapsed() > self.request_timeout) {
                log::trace!("Request timeout!");
                requests.sent_at = None;
                requests.drop_next = false;
                // Its tail may yet come, and ends its own line; the next reply
                // must not be read as the tail of this.
                buffer.clear();
            }
            if requests.sent_at.is_none() && !self.send_request(requests) {
                return;
            }
        }
    }
}

fn packet(reading: &Reading) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![reading.value],
        mq: reading.mq,
        flags: reading.flags,
        unit: reading.unit,
        digits: 2,
    })
}

impl Driver for NormaDmmDriver {
    fn id(&self) -> &'static str {
        "norma-dmm"
    }

    fn long_name(&self) -> &'static str {
        "Norma DM9x0 DMMs"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(Brand::Norma, self.id(), options)
    }
}

impl Driver for SiemensB102xDriver {
    fn id(&self) -> &'static str {
        "siemens-b102x"
    }

    fn long_name(&self) -> &'static str {
        "Siemens B102x DMMs"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        scan(Brand::Siemens, self.id(), options)
    }
}
