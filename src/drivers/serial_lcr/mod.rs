// Counterpart in sigrok:
//   libsigrok/src/hardware/serial-lcr/api.c,
//   libsigrok/src/hardware/serial-lcr/protocol.c,
//   libsigrok/src/hardware/serial-lcr/protocol.h,
//   Copyright (C) 2014 Janne Huttunen <jahuttun@gmail.com>,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Valentin Ochs, Vitaliy Vorobyov,
//   Wolfram Sang
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! LCR meters on a serial cable.
//!
//! Every supported meter is its own driver id (`deree-de5000`,
//! `voltcraft-4080`, ...), as in sigrok, since nothing on the line says which
//! meter is talking. A scan needs `conn=`; `serialcomm=` overrides the cable's
//! parameters. It listens for up to three seconds for a packet of the meter's
//! chip ([`crate::lcr`]), then for up to a second and a half for another, to
//! learn the output frequency and the circuit model. Neither can be set: they
//! are what the meter's dial says.
//!
//! A capture ends at `limit_frames`, `limit_time` or a stop. Each packet with a
//! value is a frame, with the frequency and the model as `Meta` when they
//! changed, on every capture from nothing. A packet with no value (a blank
//! display, calibration, a range switching) counts for nothing.
//!
//! Silence differs by chip. An ES51919 takes no commands and streams all the
//! time, with calibration, sorting and hold as flags inside its packets, so no
//! bytes at all means it is off or its cable is out: when only a frame limit
//! could end the capture, [`SILENCE_BOUND`] of no bytes ends it with an error.
//! Bytes count, not frames, as calibration streams with none. C waits for ever.
//! A Voltcraft 4080 that does not answer may be in setup mode, which ends when
//! its user acts, so it is warned about once and waited for, as in C. A time
//! limit ends a silent capture of either as it does in C.
//!
//! `equiv_circuit_model` has no value until a packet told it, so `options()`
//! has its `get` off until then: a library client sees that flip at a scan's
//! second listen or at a capture's first `Meta`, where C's list is static and
//! its get says `SR_ERR_NA`. sigrok-cli and oxrok then say the same.
//!
//! A Voltcraft 4080 is asked for each packet with `N`, once the line has been
//! silent for a read and at least 500 ms after the last time, which a device
//! remembers across captures as C's `req_next_at` does. The meter says
//! nothing of which request it answers, but its packets end in CR LF, so a late
//! answer cannot shift anything: the bytes before a frame are stepped over. The
//! receive buffer lives for one capture, where C keeps it between two: the head
//! of an answer that the stop cut would make a false packet with the tail of
//! another.
//!
//! Not copied from C: a port that hangs up ends the capture here. `receive`
//! there takes it for a timeout (protocol.c:193-196), for ever at a frame limit.

#![forbid(unsafe_code)]

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::lcr::{Chip, Settings};
use crate::serial::{Params, Port};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::LcrMeter];
static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::OutputFrequency, Access { get: true, set: false, list: true }),
    OptionDesc::new(Key::EquivCircuitModel, Access { get: true, set: false, list: true }),
];

/// How long a scan listens for a packet (`scan_lcr_port`).
const DETECT_TIMEOUT: Duration = Duration::from_millis(3000);
/// How long it then listens for the settings (`read_lcr_port`).
const SETTINGS_TIMEOUT: Duration = Duration::from_millis(1500);
/// The receive buffers of a scan (api.c's `buf[128]`) and a capture
/// (`LCR_BUFSIZE`).
const BUF_SIZE: usize = 128;
/// How long a read of a capture waits, `serial_source_add`'s 50 ms.
const READ_TIMEOUT: Duration = Duration::from_millis(50);
/// How long a request may wait for room. C waits for ever
/// (`serial_write_blocking` with 0), in a callback: here a stop must not, and
/// the next request is due soon.
const REQUEST_WRITE_TIMEOUT: Duration = Duration::from_millis(500);
/// How long a thread that was told to stop waits to put its `End` in a queue
/// no one reads. A close joins it. An output may have rows pending on the `End`
/// (`formats/output/csv.rs`), so this outlasts a reader's short stall.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long no byte may come before [`Silence`] has a say.
const SILENCE_BOUND: Duration = Duration::from_secs(10);

/// A meter of the table.
#[derive(Debug)]
pub struct Model {
    /// The driver id.
    pub id: &'static str,
    /// The vendor.
    pub vendor: &'static str,
    /// The model.
    pub model: &'static str,
    /// Vendor and model.
    pub long_name: &'static str,
    /// The packet format.
    pub chip: Chip,
}

/// api.c's `lcr_es51919_drivers` and `lcr_vc4080_drivers`.
pub static MODELS: [Model; 6] = [
    Model {
        id: "deree-de5000",
        vendor: "DER EE",
        model: "DE-5000",
        long_name: "DER EE DE-5000",
        chip: Chip::Es51919,
    },
    Model {
        id: "mastech-ms5308",
        vendor: "MASTECH",
        model: "MS5308",
        long_name: "MASTECH MS5308",
        chip: Chip::Es51919,
    },
    Model {
        id: "peaktech-2170",
        vendor: "PeakTech",
        model: "2170",
        long_name: "PeakTech 2170",
        chip: Chip::Es51919,
    },
    Model {
        id: "uni-t-ut612",
        vendor: "UNI-T",
        model: "UT612",
        long_name: "UNI-T UT612",
        chip: Chip::Es51919,
    },
    Model {
        id: "peaktech-2165",
        vendor: "PeakTech",
        model: "2165",
        long_name: "PeakTech 2165",
        chip: Chip::Vc4080,
    },
    Model {
        id: "voltcraft-4080",
        vendor: "Voltcraft",
        model: "4080",
        long_name: "Voltcraft 4080",
        chip: Chip::Vc4080,
    },
];

/// The driver of one meter.
pub struct SerialLcrDriver(pub &'static Model);

/// Registers a driver for every meter.
pub fn register_all(registry: &mut crate::core::Registry) {
    for model in &MODELS {
        registry.register(Box::new(SerialLcrDriver(model)));
    }
}

impl Driver for SerialLcrDriver {
    fn id(&self) -> &'static str {
        self.0.id
    }

    fn long_name(&self) -> &'static str {
        self.0.long_name
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = self.0.chip.serialcomm().to_owned();
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
                log::error!("{}: {e}", self.0.id);
                return Ok(Vec::new());
            }
        };
        let Probed::Found(settings) = probe(port.as_mut(), self.0, &params, &conn) else {
            return Ok(Vec::new());
        };
        // `serial_close`; `dev_open` opens the port again.
        drop(port);
        Ok(vec![Box::new(LcrDevice::new(self.0, conn, params, settings, None))])
    }
}

/// What a scan found.
#[derive(Debug, PartialEq, Eq)]
enum Probed {
    /// No packet came.
    Nothing,
    /// A meter, and its settings unless the second listen heard nothing.
    Found(Option<Settings>),
}

/// What a scan does on the open port: `scan_lcr_port` and `read_lcr_port`.
fn probe(port: &mut dyn Port, model: &Model, params: &Params, conn: &str) -> Probed {
    let chip = model.chip;
    log::info!("Probing serial port {conn}.");
    request(port, chip);
    let (found, read) = detect(port, chip, params, DETECT_TIMEOUT);
    if found.is_none() {
        return Probed::Nothing;
    }
    if dropped_much(read, chip.packet_size()) {
        log::warn!("Had to drop unexpected amounts of data.");
    }
    log::info!("Found {} {} device on port {conn}.", model.vendor, model.model);
    if let Err(e) = port.purge() {
        log::debug!("Flush: {e}.");
    }
    request(port, chip);
    log::info!("Retrieving current acquisition parameters.");
    let (packet, _) = detect(port, chip, params, SETTINGS_TIMEOUT);
    Probed::Found(packet.map(|p| chip.settings(&p)))
}

/// Whether a search that read `read` bytes for a packet of `size` dropped more
/// than two packets' worth, which is no error: a cable may add bytes.
fn dropped_much(read: usize, size: usize) -> bool {
    read - size > 2 * size
}

/// A meter's request, with nobody looking at the result: `vc4080_packet_request`
/// ignores `serial_write_blocking`'s, so a write that fails or finds no room is
/// a request that gets no answer.
fn request(port: &mut dyn Port, chip: Chip) {
    if let Some(req) = chip.request() {
        match port.write_timeout(req, REQUEST_WRITE_TIMEOUT) {
            Ok(n) if n == req.len() => {}
            Ok(_) => log::debug!("Packet request found no room."),
            Err(e) => log::debug!("Packet request not sent: {e}."),
        }
    }
}

/// `serial_stream_detect`: reads byte by byte until the last bytes make a packet
/// of `chip`, its [`BUF_SIZE`] bytes are full, or `timeout` passed. Gives the
/// packet, if any, and how many bytes it read. A read that fails is no data, and
/// the time still runs.
fn detect(
    port: &mut dyn Port,
    chip: Chip,
    params: &Params,
    timeout: Duration,
) -> (Option<Vec<u8>>, usize) {
    let size = chip.packet_size();
    let wait = crate::serial::serial_timeout(params, 1);
    let start = Instant::now();
    let mut buf = Vec::with_capacity(BUF_SIZE);
    let mut byte = [0u8; 1];
    while buf.len() < BUF_SIZE {
        let got = match port.read(&mut byte, wait) {
            Ok(n) => n,
            Err(e) => {
                log::debug!("Read: {e}.");
                std::thread::sleep(wait);
                0
            }
        };
        if got > 0 {
            buf.push(byte[0]);
            if buf.len() >= size && chip.valid(&buf[buf.len() - size..]) {
                let read = buf.len();
                return (Some(buf.split_off(read - size)), read);
            }
        }
        if start.elapsed() >= timeout {
            break;
        }
    }
    log::info!("Didn't find a valid packet (read {} bytes).", buf.len());
    (None, buf.len())
}

/// What a device knows of the meter's settings; a capture updates it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Values {
    output_freq: u64,
    circuit_model: Option<&'static str>,
}

impl Values {
    fn of(settings: Option<Settings>) -> Values {
        settings.map_or_else(Values::default, |s| Values {
            output_freq: s.output_freq,
            circuit_model: Some(s.circuit_model),
        })
    }
}

/// What a capture does about bytes that do not come.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Silent {
    /// Nothing.
    Listen,
    /// Says the meter may be in setup mode, once.
    Warn,
    /// Ends the capture with an error.
    End,
}

/// How long the line has been silent, and what that means for a chip. Times come
/// in as arguments, so that its tests take no time.
#[derive(Debug)]
struct Silence {
    bound: Duration,
    since: Instant,
    warned: bool,
}

impl Silence {
    fn new(bound: Duration, now: Instant) -> Silence {
        Silence { bound, since: now, warned: false }
    }

    /// Bytes came, whatever they were.
    fn heard(&mut self, now: Instant) {
        self.since = now;
    }

    /// `only_frames`: a frame limit is all that could end the capture.
    fn verdict(&mut self, chip: Chip, only_frames: bool, now: Instant) -> Silent {
        if now.saturating_duration_since(self.since) < self.bound {
            return Silent::Listen;
        }
        match chip {
            Chip::Es51919 if only_frames => Silent::End,
            Chip::Es51919 => Silent::Listen,
            Chip::Vc4080 if self.warned => Silent::Listen,
            Chip::Vc4080 => {
                self.warned = true;
                Silent::Warn
            }
        }
    }
}

/// The software limits, which a set changes under a running capture, as in C.
#[derive(Debug, Default)]
struct Limits {
    frames: AtomicU64,
    msec: AtomicU64,
}

/// One meter.
pub struct LcrDevice {
    model: &'static Model,
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limits: Arc<Limits>,
    values: Arc<Mutex<Values>>,
    /// `req_next_at`: when the next request may go, on the device as a capture
    /// may follow another straight away.
    next_request: Arc<Mutex<Option<Instant>>>,
    /// [`SILENCE_BOUND`], which a test shortens.
    silence_bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port, unless a capture holds it. It comes back before `End`.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl LcrDevice {
    fn new(
        model: &'static Model,
        conn: String,
        params: Params,
        settings: Option<Settings>,
        port: Option<Box<dyn Port>>,
    ) -> Self {
        let channels = (0..model.chip.channel_count())
            .map(|i| Channel::new(i, ChannelKind::Analog, format!("P{}", i + 1)))
            .collect();
        LcrDevice {
            model,
            info: DeviceInfo {
                driver: model.id,
                vendor: model.vendor.to_owned(),
                model: model.model.to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels,
            conn,
            params,
            limits: Arc::new(Limits::default()),
            values: Arc::new(Mutex::new(Values::of(settings))),
            next_request: Arc::new(Mutex::new(None)),
            silence_bound: SILENCE_BOUND,
            pending: port,
            port: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A device of `model` on an already open port. No detection is run.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        let params = Params::parse(model.chip.serialcomm()).expect("table parameters parse");
        Self::new(model, name, params, None, Some(port))
    }

    /// Tells a worker to stop and waits for it.
    fn join(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for LcrDevice {
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
            // A model nothing has told yet cannot be read, which C says with
            // SR_ERR_NA from its get, and sigrok-cli as "not applicable".
            None if self.values.lock().expect("values lock").circuit_model.is_none() => {
                let mut options = DEV_OPTIONS.to_vec();
                for o in &mut options {
                    if o.key == Key::EquivCircuitModel {
                        o.access.get = false;
                    }
                }
                Ok(options)
            }
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let values = *self.values.lock().expect("values lock");
        Ok(match key {
            Key::LimitFrames => Value::U64(self.limits.frames.load(Ordering::SeqCst)),
            Key::LimitMsec => Value::U64(self.limits.msec.load(Ordering::SeqCst)),
            Key::OutputFrequency => Value::F64(values.output_freq as f64),
            Key::EquivCircuitModel => match values.circuit_model {
                Some(model) => Value::Str(model.to_owned()),
                None => return Err(Error::not_supported("no circuit model known yet")),
            },
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let number =
            || value.as_u64().ok_or_else(|| Error::Arg(format!("invalid value for {}", key.id())));
        match key {
            Key::LimitFrames => self.limits.frames.store(number()?, Ordering::SeqCst),
            Key::LimitMsec => self.limits.msec.store(number()?, Ordering::SeqCst),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::OutputFrequency => Ok(ValueList::F64(self.model.chip.frequencies().to_vec())),
            Key::EquivCircuitModel => Ok(ValueList::Str(
                self.model.chip.circuit_models().iter().map(|s| (*s).to_owned()).collect(),
            )),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
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
            // gives it back before its End, so a client that saw an End is
            // never told this.
            if self.worker.as_ref().is_some_and(|w| !w.handle.is_finished()) {
                return Err(Error::Busy);
            }
            return Err(Error::device("device is not open"));
        };
        // A last capture's thread may wait to send its End to a queue nobody
        // reads: tell it, and it gives up within a bound.
        self.join();
        // As `dev_acquisition_start`: this capture says it all anew.
        *self.values.lock().expect("values lock") = Values::default();
        let capture = Capture {
            chip: self.model.chip,
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limits: Arc::clone(&self.limits),
            values: Arc::clone(&self.values),
            next_request: Arc::clone(&self.next_request),
            silence_bound: self.silence_bound,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name(format!("{} acquisition", self.model.id))
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

impl Drop for LcrDevice {
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
    chip: Chip,
    enabled: Vec<bool>,
    limits: Arc<Limits>,
    values: Arc<Mutex<Values>>,
    next_request: Arc<Mutex<Option<Instant>>>,
    silence_bound: Duration,
}

impl Capture {
    /// Runs until a limit, a stop, a port that fails or a consumer that left.
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let send = |p: Packet| sink.send_while(p, || !stop.load(Ordering::SeqCst));
        if !send(Packet::header()) {
            return Ok(());
        }
        let started = Instant::now();
        let mut frames = 0u64;
        let mut known = Values::default();
        let mut buf: Vec<u8> = Vec::with_capacity(BUF_SIZE);
        let mut chunk = [0u8; BUF_SIZE];
        let mut silence = Silence::new(self.silence_bound, started);
        while !stop.load(Ordering::SeqCst) {
            let room = BUF_SIZE - buf.len();
            match port.read(&mut chunk[..room], READ_TIMEOUT) {
                Err(e) => {
                    log::error!("Acquisition failed: {e}.");
                    return Err(e);
                }
                Ok(0) => {
                    // handle_timeout: at the first silence, and then once the
                    // time has passed.
                    let now = Instant::now();
                    let due =
                        self.next_request.lock().expect("request lock").is_none_or(|at| now >= at);
                    if self.chip.request().is_some() && due {
                        request(port, self.chip);
                        *self.next_request.lock().expect("request lock") =
                            Some(now + self.chip.request_interval());
                    }
                }
                Ok(n) => {
                    silence.heard(Instant::now());
                    buf.extend_from_slice(&chunk[..n]);
                    // handle_new_data: as many packets as there are, stepping
                    // over the bytes that start none.
                    let size = self.chip.packet_size();
                    let mut at = 0;
                    while buf.len() - at >= size {
                        let pkt = &buf[at..at + size];
                        if !self.chip.valid(pkt) {
                            at += 1;
                            continue;
                        }
                        match self.packet(pkt, &mut known, &send) {
                            None => return Ok(()),
                            Some(true) => frames += 1,
                            Some(false) => {}
                        }
                        at += size;
                    }
                    buf.drain(..at);
                }
            }
            // The limits are looked at once all of a read is out, so a read
            // holding two packets sends both.
            let limit = self.limits.frames.load(Ordering::SeqCst);
            if limit > 0 && frames >= limit {
                return Ok(());
            }
            let msec = self.limits.msec.load(Ordering::SeqCst);
            if msec > 0 && started.elapsed() > Duration::from_millis(msec) {
                return Ok(());
            }
            let seconds = self.silence_bound.as_secs();
            match silence.verdict(self.chip, limit > 0 && msec == 0, Instant::now()) {
                Silent::Listen => {}
                Silent::Warn => log::warn!(
                    "No answer from the meter for {seconds} s. It may be in setup mode, which \
                     ends when you act on it."
                ),
                Silent::End => {
                    log::error!(
                        "No bytes from the meter for {seconds} s. It sends without being asked, \
                         so it is off or its cable is out."
                    );
                    return Err(Error::device(format!("no bytes from the meter for {seconds} s")));
                }
            }
        }
        Ok(())
    }

    /// `handle_packet`: whether a frame went out, `None` when the sink would
    /// not take it. A channel that is off still has its display parsed, whose
    /// complaints are the same then.
    fn packet(
        &self,
        pkt: &[u8],
        known: &mut Values,
        send: &impl Fn(Packet) -> bool,
    ) -> Option<bool> {
        log::debug!("LCR packet: {pkt:02x?}");
        let settings = self.chip.settings(pkt);
        let mut frame = false;
        for channel in 0..self.chip.channel_count() {
            let reading = self.chip.reading(pkt, channel);
            let Some(r) = reading else { continue };
            if !self.enabled.get(channel).copied().unwrap_or(false) {
                continue;
            }
            if !frame {
                self.frame_start(settings, known, send)?;
                frame = true;
            }
            let data = AnalogData {
                channels: vec![channel],
                samples: vec![r.value],
                mq: r.mq,
                flags: r.flags,
                unit: r.unit,
                digits: r.digits,
            };
            send(Packet::Analog(data)).then_some(())?;
        }
        if frame {
            send(Packet::FrameEnd).then_some(())?;
        }
        Some(frame)
    }

    /// `send_frame_start`: what changed, then the frame.
    fn frame_start(
        &self,
        settings: Settings,
        known: &mut Values,
        send: &impl Fn(Packet) -> bool,
    ) -> Option<()> {
        if settings.output_freq != known.output_freq {
            known.output_freq = settings.output_freq;
            *self.values.lock().expect("values lock") = *known;
            let freq = settings.output_freq as f64;
            send(Packet::meta(Key::OutputFrequency, freq)).then_some(())?;
        }
        if known.circuit_model != Some(settings.circuit_model) {
            known.circuit_model = Some(settings.circuit_model);
            *self.values.lock().expect("values lock") = *known;
            let model = settings.circuit_model.to_owned();
            send(Packet::meta(Key::EquivCircuitModel, model)).then_some(())?;
        }
        send(Packet::FrameBegin).then_some(())
    }
}

#[cfg(test)]
mod tests;
