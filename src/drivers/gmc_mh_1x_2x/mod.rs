// Counterpart in sigrok:
//   libsigrok/src/hardware/gmc-mh-1x-2x/api.c,
//   libsigrok/src/hardware/gmc-mh-1x-2x/protocol.c,
//   Copyright (C) 2013, 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang,
//   Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Gossen Metrawatt `METRAHit` multimeters behind their infrared adapters, two
//! drivers on a serial port.
//!
//! `gmc-mh-1x-2x-rs232` listens to a 12S to 29S in send mode: 9600/6n1, then
//! 8228/6n1 unless `serialcomm` was given. It takes nothing from the host, so a
//! capture only reads, for as long as it takes: a meter's rate may be ten
//! minutes, so nothing bounds the wait.
//!
//! `gmc-mh-2x-bd232` asks a 22S to 29S: 9600/8n1. A scan writes the status
//! request three times a second apart to wake a meter that is off, flushes, asks
//! again, and listens for 2 s for answers. A capture has one request out at a
//! time: a measurement, the status at each tenth (a count kept across captures),
//! the next in the pass that read the answer, again after 1 s with none. When a
//! limit ends the capture in a pass that read an answer, one more request goes
//! out, as in C, where the check of the state after a stop does not look at the
//! stop. `power_off=on` writes the command and waits 2 s for it to leave.
//!
//! Both close the port after the scan and open it again. C's scans return with it
//! open, so from libserialport 0.1.2 its `dev_open` is refused and sigrok-cli runs
//! only `--scan`.
//!
//! Where these go their own way, see `docs/DEVICES.md` and [`protocol`]:
//!
//! - The sign bit negates the value, and leaves the exponent alone, as C did until
//!   `f4bd8a17` (2016).
//! - A broken fuse reads as no number, where C adds in a 13.
//! - Every meter that answers a scan is found. C loses all but the first: it makes
//!   a new `sdi` with no `priv` for the next (`api.c:294-296`), and
//!   `chk_msg14` then refuses each answer (`protocol.c:915`).
//! - An answer whose bytes are out of step finds the next header. C drops 14 bytes
//!   at a time, so one lost byte costs every answer from there on.
//! - Requests are counted, see [`Requests`], and an answer to an earlier one goes
//!   unused. A capture does not take the answers the last one left owed.
//! - A capture that only a sample limit can end stops after [`SILENCE_BOUND`] with
//!   no usable answer, where C goes on asking for ever. So a steady latency above
//!   that fails such a capture, by choice: a meter answers in milliseconds, and
//!   nothing shows one this slow. Under a time limit it goes on. In send mode only
//!   a warning, as a meter's rate may be minutes.
//! - `close` leaves the model. C forgets it (`api.c:320`), so after a reopen a 2x
//!   in send mode decodes by the 12S's tables.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{self, Params, Port};
use protocol::{Decoder, MASK_6BITS, Model, REPLY_BYTES, Reply, Sample, SendStream, WIRE_BYTES};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// `SERIALCOMM_2X_RS232`.
const SERIALCOMM_2X_RS232: &str = "9600/6n1/dtr=1/rts=1/flow=0";
/// `SERIALCOMM_1X_RS232`: 8192 Bd is the meter's, this is closer with a divider.
const SERIALCOMM_1X_RS232: &str = "8228/6n1/dtr=1/rts=1/flow=0";
/// `SERIALCOMM_2X`.
const SERIALCOMM_2X: &str = "9600/8n1/dtr=1/rts=1/flow=0";

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 2] = [DeviceClass::Multimeter, DeviceClass::Thermometer];

static SEND_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];
static BIDI_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::PowerOff, Access::GET_SET),
];

/// A scan in send mode, per rate.
const SEND_SCAN_TIME: Duration = Duration::from_secs(1);
/// The bytes a scan in send mode looks through for a start.
const SEND_SCAN_BYTES: usize = 100;
/// How often a read looks again in a scan, as `read_byte`'s `g_usleep`.
const SCAN_READ: Duration = Duration::from_millis(2);
/// The session source's timeout in both `dev_acquisition_start`s.
const POLL_INTERVAL: Duration = Duration::from_millis(40);
/// How long a request waits for its answer before the next goes.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(1);
/// How long a capture that only a sample limit can end goes with no usable answer,
/// where C asks for ever. Past the waits of four requests, at 1, 2, 4 and 8 s.
pub const SILENCE_BOUND: Duration = Duration::from_secs(10);
/// When to say that nothing has come, once per silence.
const SILENCE_WARNING: Duration = Duration::from_secs(30);
/// How long a request can come to wait.
const MAX_WAIT: Duration = Duration::from_secs(8);
/// The most answers owed: past this they are taken for lost.
const MAX_OWED: u32 = 8;
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// The most one pass reads, far above what 9600 baud brings in a pass.
const PASS_BYTES: usize = 4096;
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// What the status request's wake up writes wait for, each.
const WAKE_WAIT: Duration = Duration::from_secs(1);
/// How long a scan listens for meters after it asks.
const ANSWER_WAIT: Duration = Duration::from_secs(2);
/// What setting `power_off` waits, for the command to leave before the interface
/// goes.
const POWER_OFF_WAIT: Duration = Duration::from_secs(2);

/// The `gmc-mh-1x-2x-rs232` driver: send mode.
pub struct GmcRs232Driver;
/// The `gmc-mh-2x-bd232` driver: bidirectional.
pub struct GmcBd232Driver;

/// A scan's `conn` and `serialcomm`.
fn scan_options(options: &[(Key, Value)]) -> Result<(Option<String>, Option<String>)> {
    let (mut conn, mut serialcomm) = (None, None);
    for (key, value) in options {
        match key {
            Key::Conn => conn = value.as_str().map(str::to_owned),
            Key::Serialcomm => {
                serialcomm =
                    Some(value.as_str().ok_or_else(|| Error::arg("serialcomm"))?.to_owned());
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
    }
    Ok((conn, serialcomm))
}

/// Opens a port for a scan: an error is no device, with what it says logged.
fn scan_open(conn: &str, serialcomm: &str) -> Option<(Box<dyn Port>, Params)> {
    let opened =
        Params::parse(serialcomm).and_then(|params| Ok((serial::open(conn, params)?, params)));
    match opened {
        Ok(opened) => Some(opened),
        Err(e) => {
            log::error!("{e}");
            None
        }
    }
}

impl Driver for GmcRs232Driver {
    fn id(&self) -> &'static str {
        "gmc-mh-1x-2x-rs232"
    }

    fn long_name(&self) -> &'static str {
        "Gossen Metrawatt Metrahit 1x/2x, RS232 interface"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let (conn, serialcomm) = scan_options(options)?;
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let given = serialcomm.is_some();
        let text = serialcomm.as_deref().unwrap_or(SERIALCOMM_2X_RS232);
        let Some((port, params)) = scan_open(&conn, text) else {
            return Ok(Vec::new());
        };
        let found = probe_send(port, params, given, SEND_SCAN_TIME);
        Ok(found
            .into_iter()
            .map(|(model, params)| -> Box<dyn Device> {
                Box::new(GmcDevice::send(model, conn.clone(), params))
            })
            .collect())
    }
}

/// `scan_1x_2x_rs232` after the open. The port goes when this returns.
fn probe_send(
    mut port: Box<dyn Port>,
    mut params: Params,
    given: bool,
    time: Duration,
) -> Option<(Model, Params)> {
    let mut model = scan_model_send(&mut *port, time);
    if model == Model::None && !given {
        params = Params::parse(SERIALCOMM_1X_RS232).expect("the table's line parses");
        if port.set_params(params).is_ok() {
            model = scan_model_send(&mut *port, time);
        }
    }
    (model != Model::None).then_some((model, params))
}

/// `read_byte`: a byte, or none by `deadline`.
fn read_byte(port: &mut dyn Port, deadline: Instant) -> Option<u8> {
    let mut byte = [0];
    loop {
        match port.read(&mut byte, SCAN_READ) {
            Ok(1) => return Some(byte[0]),
            Ok(_) => {}
            // C's read fails and it goes on until its time is up.
            Err(_) => std::thread::sleep(SCAN_READ),
        }
        if Instant::now() > deadline {
            return None;
        }
    }
}

/// `scan_model_sm`: a start and four data bytes, within a time and 100 bytes.
fn scan_model_send(port: &mut dyn Port, time: Duration) -> Model {
    let deadline = Instant::now() + time;
    if port.purge().is_err() {
        return Model::None;
    }
    for _ in 0..SEND_SCAN_BYTES {
        let Some(byte) = read_byte(port, deadline) else {
            return Model::None;
        };
        if Instant::now() > deadline {
            return Model::None;
        }
        if protocol::is_start(byte) {
            let model = protocol::decode_model_sm(protocol::contents(byte));
            if model == Model::None {
                return model;
            }
            for _ in 0..4 {
                if !read_byte(port, deadline).is_some_and(protocol::is_rest) {
                    return Model::None;
                }
            }
            return model;
        }
    }
    Model::None
}

impl Driver for GmcBd232Driver {
    fn id(&self) -> &'static str {
        "gmc-mh-2x-bd232"
    }

    fn long_name(&self) -> &'static str {
        "Gossen Metrawatt Metrahit 2x, BD232/SI232-II interface"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let (conn, serialcomm) = scan_options(options)?;
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let text = serialcomm.as_deref().unwrap_or(SERIALCOMM_2X);
        let Some((port, params)) = scan_open(&conn, text) else {
            return Ok(Vec::new());
        };
        let meters = probe_bidi(port, params, WAKE_WAIT, ANSWER_WAIT).unwrap_or_else(|e| {
            log::debug!("Scan failed: {e}.");
            Vec::new()
        });
        Ok(meters
            .into_iter()
            .map(|decoder| -> Box<dyn Device> {
                Box::new(GmcDevice::bidi(decoder, conn.clone(), params))
            })
            .collect())
    }
}

/// `scan_2x_bd232` after the open: every meter that answers. The port goes when
/// this returns.
fn probe_bidi(
    mut port: Box<dyn Port>,
    params: Params,
    wake: Duration,
    answers: Duration,
) -> Result<Vec<Decoder>> {
    let request = protocol::status_request(0);
    let timeout = serial::serial_timeout(&params, WIRE_BYTES as u32);
    log::info!("Write some data and wait 3s to turn on powered off device...");
    for _ in 0..3 {
        port.write_timeout(&request, timeout)?;
        std::thread::sleep(wake);
    }
    port.purge()?;
    if port.write_timeout(&request, timeout)? < WIRE_BYTES {
        return Err(Error::device("the request did not go out"));
    }
    let deadline = Instant::now() + answers;
    let mut meters = Vec::new();
    while Instant::now() < deadline {
        let mut buf = [0u8; REPLY_BYTES];
        let mut len = 0;
        for _ in 0..REPLY_BYTES {
            if let Some(byte) = read_byte(&mut *port, deadline) {
                buf[len] = byte & MASK_6BITS;
                len += 1;
            }
        }
        if len != REPLY_BYTES {
            continue;
        }
        let mut decoder = Decoder::new(Model::None);
        decoder.addr = buf[0];
        decoder.reply(&buf);
        if decoder.model != Model::None {
            meters.push(decoder);
        }
    }
    Ok(meters)
}

/// `devc`, while the device exists.
struct Link {
    port: Option<Box<dyn Port>>,
    params: Params,
    decoder: Decoder,
    /// `cmd_seq`, kept across captures as in C.
    cmd_seq: u32,
    /// Answers the last capture left owed, and how long it had come to wait.
    owed: u32,
    wait: Duration,
    /// What had come of an answer when it ended.
    partial: Vec<u8>,
}

impl Link {
    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `serial_write_blocking` with its timeout: true if all of it went.
    fn send(&mut self, wire: &[u8; WIRE_BYTES]) -> Result<bool> {
        let timeout = serial::serial_timeout(&self.params, WIRE_BYTES as u32);
        Ok(self.port()?.write_timeout(wire, timeout)? == WIRE_BYTES)
    }
}

/// One meter.
pub struct GmcDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    bidi: bool,
    conn: String,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    /// A capture is running: a setting would mix with its requests.
    capturing: Arc<AtomicBool>,
    silence_bound: Duration,
    silence_warning: Duration,
    request_timeout: Duration,
    power_off_wait: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl GmcDevice {
    fn new(
        driver: &'static str,
        version: String,
        decoder: Decoder,
        conn: String,
        params: Params,
    ) -> Self {
        let link = Link {
            port: None,
            params,
            decoder,
            cmd_seq: 0,
            owed: 0,
            wait: REQUEST_TIMEOUT,
            partial: Vec::new(),
        };
        GmcDevice {
            info: DeviceInfo {
                driver,
                vendor: "Gossen Metrawatt".to_owned(),
                model: link.decoder.model.name().to_owned(),
                version,
                serial: String::new(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            bidi: driver == "gmc-mh-2x-bd232",
            conn,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            silence_bound: SILENCE_BOUND,
            silence_warning: SILENCE_WARNING,
            request_timeout: REQUEST_TIMEOUT,
            power_off_wait: POWER_OFF_WAIT,
        }
    }

    fn send(model: Model, conn: String, params: Params) -> Self {
        Self::new("gmc-mh-1x-2x-rs232", String::new(), Decoder::new(model), conn, params)
    }

    fn bidi(decoder: Decoder, conn: String, params: Params) -> Self {
        let (major, minor) = decoder.firmware;
        Self::new("gmc-mh-2x-bd232", format!("Firmware {major}.{minor}"), decoder, conn, params)
    }

    /// A send mode meter on a port that stands in for the hardware, without a
    /// scan. [`Device::open`] takes it.
    pub fn send_from_port(port: Box<dyn Port>, model: Model) -> Self {
        let params = Params::parse(SERIALCOMM_2X_RS232).expect("the default parses");
        let mut device = Self::send(model, port.name().to_owned(), params);
        device.pending = Some(port);
        device
    }

    /// A bidirectional meter at `addr` on a port that stands in for the hardware,
    /// without a scan. [`Device::open`] takes it.
    pub fn bidi_from_port(port: Box<dyn Port>, model: Model, addr: u8) -> Self {
        let params = Params::parse(SERIALCOMM_2X).expect("the default parses");
        let mut decoder = Decoder::new(model);
        decoder.addr = addr;
        let mut device = Self::bidi(decoder, port.name().to_owned(), params);
        device.pending = Some(port);
        device
    }

    fn set_power_off(&mut self, off: bool) -> Result<()> {
        if !off {
            return Err(Error::arg("power_off takes only on: nothing switches a meter on"));
        }
        log::info!("Powering device off.");
        let mut link = self.link.lock().expect("link lock");
        let wire = protocol::power_off_request(link.decoder.addr);
        link.send(&wire)?;
        drop(link);
        std::thread::sleep(self.power_off_wait);
        Ok(())
    }
}

impl Device for GmcDevice {
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
        Ok(if self.bidi { BIDI_OPTIONS.to_vec() } else { SEND_OPTIONS.to_vec() })
    }

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            // Nothing says a meter is off: this is C's constant.
            Key::PowerOff if self.bidi => Value::Bool(false),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::not_supported(format!("{} while capturing", key.id())));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::PowerOff if self.bidi => self.set_power_off(value.as_bool().ok_or_else(bad)?)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Ok(());
        }
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => serial::open(&self.conn, link.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link.lock().expect("link lock");
        link.port = None;
        // An open flushes what is on its way.
        link.owed = 0;
        link.partial.clear();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        // A capture past its End may still be handing it over to a full queue.
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        if self.link.lock().expect("link lock").port.is_none() {
            return Err(Error::device("device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled: self.channels[0].enabled,
            bidi: self.bidi,
            silence_bound: self.silence_bound,
            silence_warning: self.silence_warning,
            request_timeout: self.request_timeout,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
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

/// `Failed` if it failed and `End`, waiting for room in the queue for as long as
/// nobody asked to stop, and after a stop only for [`END_AFTER_STOP`]: a close joins
/// this thread.
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

fn packet(sample: &Sample) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![0],
        samples: vec![sample.value],
        // C's mq and unit of 0 have no name; nothing prints them. A function no
        // table has is measured in nothing this could say, as for other ports.
        mq: sample.mq.unwrap_or(Mq::Gain),
        flags: sample.flags,
        unit: sample.unit.unwrap_or(Unit::Unitless),
        digits: sample.digits,
    })
}

/// What a request loop does next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Turn {
    /// None is out, or none should be yet.
    Wait,
    /// None is owed: the next goes.
    Ask,
    /// The one out has waited long enough: another goes.
    Expired,
}

/// `response_pending` and `req_sent_at`, with a count, since no answer says which
/// request it answers or when it was measured.
///
/// C sends at each answer and at each second, so at a latency over 1 s the answers
/// owed grow; a meter that queues had 47 owed after a minute, and a sample answering
/// a request 25 s old. Here an answer with others owed behind it is late and goes
/// unused. A request goes when none is owed. An expiry sends one anyway, as answers
/// may be lost, and the wait doubles up to [`MAX_WAIT`] while nothing is heard, so
/// at a steady latency it grows past it and no more pile up. An answer heard
/// restarts the wait. When that runs out too, what is still owed was lost, which
/// makes a lost answer cost 3 s where C's costs 1 s. Never more than [`MAX_OWED`].
#[derive(Debug)]
struct Requests {
    owed: u32,
    /// Answers still to come to requests of the capture before.
    to_drop: u32,
    /// When the last request went.
    asked_at: Instant,
    /// What the wait counts from: that, or the last answer since.
    since: Instant,
    first_wait: Duration,
    wait: Duration,
    /// An answer has come since the last request.
    heard: bool,
    /// Answers taken for lost that may only be slow: no use within a capture, where
    /// a capture's own first answer would be held up, but the next must not read
    /// them. Two answers in the first wait say the line is in step.
    doubtful: u32,
    /// Answers in a row in the first wait. One may be an older one's that happened
    /// to come just then, so it takes two to set the wait back.
    prompt: u32,
}

impl Requests {
    /// As a capture finds them: what the last one left owed is dropped as it comes,
    /// for twice what that one had got to waiting, which a capture ended early need
    /// not have grown past the latency. With none left it starts over.
    fn new(left: u32, first_wait: Duration, wait: Duration, now: Instant) -> Requests {
        let wait = if left > 0 { (wait * 2).min(MAX_WAIT) } else { first_wait };
        Requests {
            owed: left,
            to_drop: left,
            asked_at: now,
            since: now,
            first_wait,
            wait: wait.max(first_wait),
            heard: false,
            doubtful: 0,
            prompt: 0,
        }
    }

    /// An answer came, in step: whether what it holds may be used.
    fn answer(&mut self, now: Instant) -> bool {
        let dropped = self.to_drop > 0;
        self.to_drop = self.to_drop.saturating_sub(1);
        let newest = self.owed == 1;
        self.owed = self.owed.saturating_sub(1);
        self.heard = true;
        self.since = now;
        if self.owed == 0 && now.saturating_duration_since(self.asked_at) <= self.first_wait {
            self.prompt += 1;
            if self.prompt >= 2 {
                self.wait = self.first_wait;
                self.doubtful = 0;
            }
        } else {
            self.prompt = 0;
        }
        newest && !dropped
    }

    fn turn(&self, now: Instant) -> Turn {
        if self.owed == 0 {
            return Turn::Ask;
        }
        if now.saturating_duration_since(self.since) > self.wait {
            return Turn::Expired;
        }
        Turn::Wait
    }

    /// The wait ran out, so it is longer next time. An answer heard since the last
    /// request, or a count at its bound, says what is owed was lost. If it was only
    /// slow, one answer stays out for good, which ages each sample by one latency.
    fn expire(&mut self) {
        if self.to_drop > 0 || self.heard || self.owed >= MAX_OWED {
            if self.to_drop == 0 {
                self.doubtful = (self.doubtful + self.owed).min(MAX_OWED);
            }
            self.owed = 0;
        }
        self.wait = (self.wait * 2).min(self.first_wait.max(MAX_WAIT));
        self.to_drop = 0;
    }

    /// What the next capture is to drop.
    fn left(&self) -> u32 {
        (self.owed + self.doubtful).min(MAX_OWED)
    }

    fn asked(&mut self, now: Instant) {
        self.owed = self.owed.saturating_add(1);
        self.asked_at = now;
        self.since = now;
        self.heard = false;
    }
}

fn no_data_warning(after: Duration) -> String {
    format!("No data for {} s; the meter's rate may be slower.", after.as_secs())
}

fn no_reading_warning(after: Duration) -> String {
    format!("No new reading for {} s; still asking.", after.as_secs())
}

/// Whether to say now that nothing has come for `after`: once per silence.
struct SilenceNote {
    after: Duration,
    told: bool,
}

impl SilenceNote {
    fn new(after: Duration) -> SilenceNote {
        SilenceNote { after, told: false }
    }

    fn data(&mut self) {
        self.told = false;
    }

    fn due(&mut self, since: Instant, now: Instant) -> bool {
        if self.told || now.saturating_duration_since(since) < self.after {
            return false;
        }
        self.told = true;
        true
    }
}

/// One capture, on its own thread: `gmc_mh_1x_2x_receive_data` or
/// `gmc_mh_2x_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    /// `-C`, which C ignores.
    enabled: bool,
    bidi: bool,
    silence_bound: Duration,
    silence_warning: Duration,
    request_timeout: Duration,
}

/// What one capture has counted.
struct Count {
    started: Instant,
    samples: u64,
}

/// What a pass found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pass {
    /// Nothing, and none is due.
    NotYet,
    /// Nothing, and C's timeout has run.
    Empty,
    /// Bytes.
    Bytes,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
            return Ok(());
        }
        // The lock stays for the whole capture: `set` and `open` look at
        // `capturing` first, and `get` takes nothing from the link.
        let mut link = self.link.lock().expect("link lock");
        if self.bidi {
            self.run_bidi(&mut link, sink, stop)
        } else {
            self.run_send(&mut link, sink, stop)
        }
    }

    fn limit_reached(&self, count: &Count) -> bool {
        (self.limit_msec > 0 && count.started.elapsed() >= Duration::from_millis(self.limit_msec))
            || (self.limit_samples > 0 && count.samples >= self.limit_samples)
    }

    /// One sample out: false when a stop came while the queue was full.
    fn send_sample(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        count: &mut Count,
        sample: &Sample,
    ) -> bool {
        count.samples += 1;
        !self.enabled || sink.send_while(packet(sample), || !stop.load(Ordering::SeqCst))
    }

    /// Reads what there is, as C's callback, with `each` for every byte, which says
    /// to stop reading. A port that fails is an error: C's `revents == G_IO_IN` is
    /// false on a hung up port, so it reads nothing and never notices in send mode.
    fn pass(port: &mut dyn Port, due: Instant, mut each: impl FnMut(u8) -> bool) -> Result<Pass> {
        let left = due.saturating_duration_since(Instant::now()).min(STOP_CHECK);
        let mut buf = [0u8; 64];
        let mut n = port.read(&mut buf, left)?;
        if n == 0 {
            return Ok(if Instant::now() >= due { Pass::Empty } else { Pass::NotYet });
        }
        let mut total = 0;
        while n > 0 {
            for byte in &buf[..n] {
                if !each(*byte) {
                    return Ok(Pass::Bytes);
                }
            }
            total += n;
            if total >= PASS_BYTES {
                break;
            }
            n = serial::read_blocking(port, &mut buf, Duration::ZERO)?;
        }
        Ok(Pass::Bytes)
    }

    fn run_send(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let mut stream = SendStream::new(link.decoder.model);
        let mut count = Count { started: Instant::now(), samples: 0 };
        let mut due = count.started + POLL_INTERVAL;
        let mut note = SilenceNote::new(self.silence_warning);
        let mut last_data = count.started;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            let went = Self::pass(&mut **link.port()?, due, |byte| {
                last_data = Instant::now();
                note.data();
                let Some(sample) = stream.byte(byte) else {
                    return true;
                };
                self.send_sample(sink, stop, &mut count, &sample)
            })?;
            if went == Pass::NotYet {
                continue;
            }
            due = Instant::now() + POLL_INTERVAL;
            if self.limit_reached(&count) {
                return Ok(());
            }
            // A meter's rate may be minutes, so this goes on, and says so.
            if note.due(last_data, Instant::now()) {
                log::warn!("{}", no_data_warning(self.silence_warning));
            }
        }
    }

    /// Whether a meter that stops answering would keep this capture for ever:
    /// nothing but samples can end it.
    fn bounded(&self) -> bool {
        self.limit_samples > 0 && self.limit_msec == 0
    }

    fn run_bidi(&self, link: &mut Link, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let mut count = Count { started: Instant::now(), samples: 0 };
        let (left, wait) = (link.owed, link.wait);
        let mut requests = Requests::new(left, self.request_timeout, wait, count.started);
        let mut buf = std::mem::take(&mut link.partial);
        let ended = self.bidi_loop(link, sink, stop, &mut count, &mut requests, &mut buf);
        link.owed = requests.left();
        link.wait = requests.wait;
        link.partial = buf;
        ended
    }

    /// Sends the next request: a failed last one, after a limit, says nothing.
    fn ask(link: &mut Link, requests: &mut Requests, first: bool, done: bool) -> Result<()> {
        if !first {
            link.cmd_seq = link.cmd_seq.wrapping_add(1);
        }
        let addr = link.decoder.addr;
        let wire = if !first && link.cmd_seq.is_multiple_of(10) {
            protocol::status_request(addr)
        } else {
            protocol::measure_request(addr)
        };
        let sent = link.send(&wire);
        requests.asked(Instant::now());
        if done {
            return Ok(());
        }
        if !sent? {
            return Err(Error::device(if first {
                "the first request did not go out"
            } else {
                "a request did not go out"
            }));
        }
        Ok(())
    }

    fn bidi_loop(
        &self,
        link: &mut Link,
        sink: &PacketSink,
        stop: &AtomicBool,
        count: &mut Count,
        requests: &mut Requests,
        buf: &mut Vec<u8>,
    ) -> Result<()> {
        // C's start sends one with no regard for what is owed: with none owed so
        // does this, and with some the first goes when they are in.
        let mut first = true;
        if requests.owed == 0 {
            Self::ask(link, requests, true, false)?;
            first = false;
        }
        let mut last_usable = Instant::now();
        let mut last_sample = last_usable;
        let mut note = SilenceNote::new(self.silence_warning);
        let mut due = Instant::now() + POLL_INTERVAL;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(());
            }
            let Some(port) = link.port.as_mut() else {
                return Err(Error::device("device is not open"));
            };
            let decoder = &mut link.decoder;
            let went = Self::pass(&mut **port, due, |byte| {
                buf.push(byte & MASK_6BITS);
                if buf.len() < REPLY_BYTES {
                    return true;
                }
                let block: [u8; REPLY_BYTES] =
                    buf.as_slice().try_into().expect("as long as this checks");
                let reply = decoder.reply(&block);
                let kept =
                    if reply == Reply::Bad { protocol::resync(&block, decoder.addr) } else { None };
                buf.clear();
                if let Some(at) = kept {
                    // Out of step: whatever this was, it answered nothing.
                    buf.extend_from_slice(&block[at..]);
                    return true;
                }
                let current = requests.answer(Instant::now());
                let Reply::Good(sample) = reply else {
                    return true;
                };
                let go_on = match sample {
                    Some(s) if current => {
                        let sent = self.send_sample(sink, stop, count, &s);
                        last_sample = Instant::now();
                        note.data();
                        sent
                    }
                    Some(_) => {
                        log::debug!("Dropping an answer to an earlier request.");
                        true
                    }
                    None => true,
                };
                // After the send: a slow client is no silent meter.
                last_usable = Instant::now();
                go_on
            })?;
            if went == Pass::NotYet {
                continue;
            }
            let now = Instant::now();
            due = now + POLL_INTERVAL;
            let done = self.limit_reached(count);
            if self.bounded()
                && !done
                && now.saturating_duration_since(last_usable) >= self.silence_bound
            {
                return Err(Error::device(format!(
                    "no usable answer from the meter for {} s",
                    self.silence_bound.as_secs()
                )));
            }
            if note.due(last_sample, now) {
                log::warn!("{}", no_reading_warning(self.silence_warning));
            }
            // Read before the timer, as `fd_source_check` does: an answer that is
            // here wins, or one stall would send an extra request.
            let turn = if went == Pass::Bytes && requests.owed > 0 {
                Turn::Wait
            } else {
                requests.turn(now)
            };
            if turn == Turn::Expired {
                log::trace!("Request timeout.");
                requests.expire();
            }
            if turn != Turn::Wait {
                Self::ask(link, requests, first, done)?;
                first = false;
            }
            if done {
                return Ok(());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit that
    /// does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(30);

    type Received = Receiver<(usize, Packet)>;

    /// What a meter does with a request.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Answer {
        /// Answers, all of it.
        Whole,
        /// Answers when the next request has come, so late.
        Late,
        /// Answers all but its last bytes.
        Short(usize),
        /// Nothing.
        Silent,
    }

    #[derive(Default)]
    struct State {
        /// What has arrived, for a read to take.
        input: VecDeque<u8>,
        /// What is on the way: one byte comes per `pace` reads.
        coming: VecDeque<u8>,
        pace: u32,
        reads_since: u32,
        /// Requests seen, as command numbers.
        commands: Vec<u8>,
        /// Whole writes.
        writes: Vec<Vec<u8>>,
        /// What each request gets, from the first; `Whole` after that.
        script: VecDeque<Answer>,
        /// An answer held by `Answer::Late`.
        held: Vec<u8>,
        /// The next measurement's first digit counts from this.
        next_value: u32,
        purges: u32,
        fail_reads: bool,
        fail_writes_after: Option<usize>,
        /// An endless stream, for a stop to be seen against.
        endless: Vec<u8>,
        reads: u64,
        /// A read that finds nothing while bytes are coming waits as any other, so
        /// that they take real time. Otherwise they come by the count of reads.
        real_time: bool,
        /// Each answer arrives this long after its request, and when.
        latency: Duration,
        timed: VecDeque<(Instant, Vec<u8>)>,
        /// Writes go only this far.
        short_writes: bool,
        /// The next read with bytes coming waits this long, and after this many
        /// bytes a read finds nothing, once: a stall in the middle of an answer.
        stall: Option<(Duration, usize)>,
        stalled_bytes: usize,
    }

    #[derive(Clone, Default)]
    struct Shared(Arc<Mutex<State>>);

    impl Shared {
        fn lock(&self) -> std::sync::MutexGuard<'_, State> {
            self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
        }

        fn commands(&self) -> Vec<u8> {
            self.lock().commands.clone()
        }

        /// Until `n` requests have come, within a bound.
        fn wait_for_requests(&self, n: usize) {
            let deadline = Instant::now() + CAPTURE_BOUND;
            while self.lock().commands.len() < n {
                assert!(Instant::now() < deadline, "not {n} requests in time");
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }

    /// The meter's answer to a measurement: digits lowest first that read as
    /// `value` / 100000, in V DC.
    fn measure_answer(value: u32) -> Vec<u8> {
        let mut b = vec![1u8, 0x27, 0x3f, 8, 0, 1, 0x11];
        let mut v = value;
        for _ in 0..6 {
            b.push((v % 10) as u8);
            v /= 10;
        }
        b.push(0);
        b[13] = protocol::checksum(&b);
        b
    }

    fn status_answer() -> Vec<u8> {
        let mut b = vec![1u8, 0x27, 0x3f, 3, 2, 1, 3, 1, 1, 0, 0, 30, 1, 0];
        b[13] = protocol::checksum(&b);
        b
    }

    /// A request's command from the wire.
    fn command_of(wire: &[u8]) -> u8 {
        let two = |b: u8| match b {
            0x00 => 0u8,
            0x0f => 1,
            0xf0 => 2,
            _ => 3,
        };
        two(wire[9]) | (two(wire[10]) << 2) | (two(wire[11]) << 4)
    }

    struct Fake {
        state: Shared,
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "/dev/ttyFAKE"
        }

        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }

        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }

        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            {
                let mut s = self.state.lock();
                if s.fail_reads {
                    return Err(Error::device("the port went"));
                }
                if let Some((wait, bytes)) = s.stall
                    && !s.coming.is_empty()
                {
                    if s.stalled_bytes == 0 {
                        std::thread::sleep(wait);
                    }
                    if s.stalled_bytes == bytes {
                        s.stall = None;
                        return Ok(0);
                    }
                    s.stalled_bytes += 1;
                }
                while s.timed.front().is_some_and(|(at, _)| *at <= Instant::now()) {
                    let (_, bytes) = s.timed.pop_front().expect("just seen");
                    s.input.extend(bytes);
                }
                s.reads += 1;
                s.reads_since += 1;
                if s.reads_since >= s.pace.max(1) {
                    s.reads_since = 0;
                    if let Some(b) = s.coming.pop_front() {
                        s.input.push_back(b);
                    }
                }
                if s.input.is_empty() && !s.endless.is_empty() {
                    let chunk = s.endless.clone();
                    s.input.extend(chunk);
                }
                if !s.input.is_empty() {
                    let n = buf.len().min(s.input.len());
                    for (out, b) in buf.iter_mut().zip(s.input.drain(..n)) {
                        *out = b;
                    }
                    return Ok(n);
                }
                if !s.coming.is_empty() && !s.real_time {
                    return Ok(0);
                }
            }
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            Ok(0)
        }

        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut s = self.state.lock();
            if s.fail_writes_after.is_some_and(|n| s.writes.len() >= n) {
                return Err(Error::device("the port went"));
            }
            s.writes.push(data.to_vec());
            if data.len() != WIRE_BYTES {
                return Ok(());
            }
            let command = command_of(data);
            s.commands.push(command);
            let held = std::mem::take(&mut s.held);
            s.coming.extend(held);
            let answer = match command {
                3 => status_answer(),
                8 => {
                    s.next_value += 1;
                    measure_answer(s.next_value * 1000)
                }
                _ => return Ok(()),
            };
            if !s.latency.is_zero() {
                let at = Instant::now() + s.latency;
                s.timed.push_back((at, answer));
                return Ok(());
            }
            match s.script.pop_front().unwrap_or(Answer::Whole) {
                Answer::Whole => s.coming.extend(answer),
                Answer::Late => s.held = answer,
                Answer::Short(k) => s.coming.extend(&answer[..REPLY_BYTES - k]),
                Answer::Silent => {}
            }
            Ok(())
        }

        fn write_timeout(&mut self, data: &[u8], _: Duration) -> Result<usize> {
            if self.state.lock().short_writes {
                return Ok(data.len() / 2);
            }
            self.write(data)?;
            Ok(data.len())
        }

        fn purge(&mut self) -> Result<()> {
            let mut s = self.state.lock();
            s.input.clear();
            s.purges += 1;
            Ok(())
        }
    }

    fn fake() -> (Box<Fake>, Shared) {
        let state = Shared::default();
        (Box::new(Fake { state: state.clone() }), state)
    }

    fn bidi_device() -> (GmcDevice, Shared) {
        let (port, state) = fake();
        let mut d = GmcDevice::bidi_from_port(port, Model::M26S, 1);
        d.request_timeout = Duration::from_millis(40);
        d.open().unwrap();
        (d, state)
    }

    fn send_device(model: Model) -> (GmcDevice, Shared) {
        let (port, state) = fake();
        let mut d = GmcDevice::send_from_port(port, model);
        d.open().unwrap();
        (d, state)
    }

    fn start(d: &mut GmcDevice) -> Received {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    /// The values a capture sends up to its End, which must come in time.
    fn readings(rx: &Received) -> Vec<f32> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left).expect("no End in time").1 {
                Packet::Analog(a) => out.push(a.samples[0]),
                Packet::End => return out,
                _ => {}
            }
        }
    }

    /// As [`readings`], and what `Packet::Failed` said.
    fn ending(rx: &Received) -> (Vec<f32>, Option<String>) {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let (mut out, mut failed) = (Vec::new(), None);
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left).expect("no End in time").1 {
                Packet::Analog(a) => out.push(a.samples[0]),
                Packet::Failed(why) => failed = Some(why),
                Packet::End => return (out, failed),
                _ => {}
            }
        }
    }

    fn limit(d: &mut GmcDevice, samples: u64) {
        d.set(&Key::LimitSamples, &Value::U64(samples), None).unwrap();
    }

    /// A value as [`measure_answer`] gives the nth measurement.
    fn nth(n: u32) -> f32 {
        (f64::from((n * 1000) as f32) * 10f64.powf(-5.0)) as f32
    }

    #[test]
    fn the_constants_are_cs() {
        assert_eq!(SEND_SCAN_TIME, Duration::from_secs(1));
        assert_eq!(SEND_SCAN_BYTES, 100);
        assert_eq!(POLL_INTERVAL, Duration::from_millis(40));
        assert_eq!(REQUEST_TIMEOUT, Duration::from_secs(1));
        assert_eq!(WAKE_WAIT, Duration::from_secs(1));
        assert_eq!(ANSWER_WAIT, Duration::from_secs(2));
        assert_eq!(POWER_OFF_WAIT, Duration::from_secs(2));
        assert_eq!(SILENCE_BOUND, Duration::from_secs(10));
        assert_eq!(SILENCE_WARNING, Duration::from_secs(30));
        assert_eq!(MAX_WAIT, Duration::from_secs(8));
        assert_eq!(MAX_OWED, 8);
    }

    #[test]
    fn requests_go_one_at_a_time_a_status_each_tenth_and_one_after_the_limit() {
        let (mut d, state) = bidi_device();
        limit(&mut d, 12);
        let rx = start(&mut d);
        let values = readings(&rx);
        assert_eq!(values.len(), 12);
        assert_eq!(values[0], nth(1));
        assert_eq!(values[11], nth(12));
        let want = [8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 3, 8, 8, 8];
        assert_eq!(state.commands(), want);
        d.close().unwrap();
    }

    #[test]
    fn the_count_of_requests_goes_on_across_captures() {
        let (mut d, state) = bidi_device();
        limit(&mut d, 5);
        readings(&start(&mut d));
        readings(&start(&mut d));
        readings(&start(&mut d));
        let commands = state.commands();
        // The first of each capture is a measurement, and counts nothing.
        let at: Vec<usize> =
            commands.iter().enumerate().filter(|(_, c)| **c == 3).map(|(i, _)| i).collect();
        assert_eq!(at.len(), 1, "{commands:?}");
        assert_eq!(at[0], 11, "{commands:?}");
        d.close().unwrap();
    }

    #[test]
    fn a_late_answer_goes_unused_and_shifts_nothing() {
        let (mut d, state) = bidi_device();
        d.request_timeout = Duration::from_millis(200);
        state.lock().pace = 2;
        state.lock().script = VecDeque::from([Answer::Whole, Answer::Late]);
        limit(&mut d, 3);
        let values = readings(&start(&mut d));
        // The late one comes when the next request has gone, so it isn't that
        // request's reading, and each that counts is the one asked for last.
        assert_eq!(values, [nth(1), nth(3), nth(4)]);
        d.close().unwrap();
    }

    #[test]
    fn bytes_lost_cost_that_answer_and_the_next_finds_its_header() {
        let (mut d, state) = bidi_device();
        state.lock().script = VecDeque::from([Answer::Whole, Answer::Short(3)]);
        limit(&mut d, 3);
        let values = readings(&start(&mut d));
        assert_eq!(values[0], nth(1));
        // What came of the second is gone with its request, or with a failed check
        // after it, and nothing false comes of it: only the requests' own values.
        for pair in values.windows(2) {
            assert!(pair[0] < pair[1], "{values:?}");
        }
        assert!(
            values.iter().all(|v| (1..=9).any(|n| v.to_bits() == nth(n).to_bits())),
            "{values:?}"
        );
        d.close().unwrap();
    }

    #[test]
    fn a_fragment_with_no_expiry_finds_the_next_header_by_its_check() {
        // The tail of a lost answer ahead of a good one, with no time to expire.
        let (mut d, state) = bidi_device();
        d.request_timeout = Duration::from_secs(20);
        let tail = measure_answer(7000)[8..].to_vec();
        state.lock().coming.extend(tail);
        limit(&mut d, 2);
        let values = readings(&start(&mut d));
        assert_eq!(values, [nth(1), nth(2)], "the first answer's head was kept");
        d.close().unwrap();
    }

    #[test]
    fn the_answer_left_by_a_capture_is_not_the_next_ones_first() {
        for pace in [1, 3] {
            let (mut d, state) = bidi_device();
            d.request_timeout = Duration::from_secs(20);
            state.lock().pace = pace;
            limit(&mut d, 2);
            assert_eq!(readings(&start(&mut d)), [nth(1), nth(2)]);
            // The third request went at the limit; its answer is on its way.
            assert_eq!(state.commands().len(), 3);
            let values = readings(&start(&mut d));
            assert_eq!(values, [nth(4), nth(5)], "pace {pace}");
            d.close().unwrap();
        }
    }

    #[test]
    fn no_answer_of_a_capture_reaches_the_next_at_a_latency_over_the_timeout() {
        let (mut d, state) = bidi_device();
        d.request_timeout = Duration::from_millis(30);
        d.link.lock().unwrap().wait = d.request_timeout;
        state.lock().latency = Duration::from_millis(300);
        limit(&mut d, 2);
        readings(&start(&mut d));
        let last_of_first = state.commands().len() as u32;
        assert!(d.link.lock().unwrap().owed >= 1, "or this says nothing");
        if last_of_first > 3 {
            assert!(d.link.lock().unwrap().wait > d.request_timeout, "expiries lengthen it");
        }
        let values = readings(&start(&mut d));
        assert_eq!(values.len(), 2);
        for v in values {
            let earlier = (1..=last_of_first).any(|n| v.to_bits() == nth(n).to_bits());
            assert!(!earlier, "{v} answers a request of the first capture");
        }
        d.close().unwrap();
    }

    #[test]
    fn a_capture_after_the_requests_time_finds_the_old_answer_gone() {
        let (mut d, state) = bidi_device();
        limit(&mut d, 1);
        readings(&start(&mut d));
        // Long enough for the request to have expired, and it has all arrived.
        state.lock().pace = 1;
        std::thread::sleep(Duration::from_millis(60));
        assert_eq!(readings(&start(&mut d)), [nth(3)]);
        d.close().unwrap();
    }

    #[test]
    fn a_stop_during_an_answer_leaves_the_next_capture_in_step() {
        let (mut d, state) = bidi_device();
        d.request_timeout = Duration::from_secs(20);
        // The second answer stops three bytes short, where the stop lands.
        state.lock().script = VecDeque::from([Answer::Whole, Answer::Short(3)]);
        let rx = start(&mut d);
        state.wait_for_requests(2);
        let deadline = Instant::now() + CAPTURE_BOUND;
        while !(state.lock().coming.is_empty() && state.lock().input.is_empty()) {
            assert!(Instant::now() < deadline, "the bytes were not read");
            std::thread::sleep(Duration::from_millis(1));
        }
        // A read's bytes are in the buffer by the next read: wait for that.
        let reads = state.lock().reads;
        while state.lock().reads < reads + 2 {
            assert!(Instant::now() < deadline, "no more reads");
            std::thread::sleep(Duration::from_millis(1));
        }
        d.stop().unwrap();
        assert_eq!(readings(&rx), [nth(1)]);
        let tail = measure_answer(2000)[11..].to_vec();
        state.lock().coming.extend(tail);
        limit(&mut d, 2);
        // The answer it stopped in is finished, not waited out for 20 s.
        let began = Instant::now();
        let values = readings(&start(&mut d));
        assert!(began.elapsed() < Duration::from_secs(10), "{:?}", began.elapsed());
        assert_eq!(values, [nth(3), nth(4)]);
        d.close().unwrap();
    }

    #[test]
    fn a_setting_right_after_a_capture_goes_out_whole() {
        let (mut d, state) = bidi_device();
        d.power_off_wait = Duration::from_millis(1);
        limit(&mut d, 1);
        readings(&start(&mut d));
        d.set(&Key::PowerOff, &Value::Bool(true), None).unwrap();
        let last = state.lock().writes.last().unwrap().clone();
        assert_eq!(last, protocol::power_off_request(1));
        d.close().unwrap();
    }

    #[test]
    fn power_off_takes_only_on_and_only_in_bidirectional_mode() {
        let (mut d, state) = bidi_device();
        let before = state.lock().writes.len();
        let refused = d.set(&Key::PowerOff, &Value::Bool(false), None).unwrap_err();
        assert!(refused.to_string().contains("power_off takes only on"), "{refused}");
        assert_eq!(state.lock().writes.len(), before, "nothing went out");
        assert_eq!(d.get(&Key::PowerOff, None).unwrap().as_bool(), Some(false));
        let (mut s, _) = send_device(Model::M26SM);
        assert!(s.set(&Key::PowerOff, &Value::Bool(true), None).is_err());
        assert!(s.get(&Key::PowerOff, None).is_err());
        assert_eq!(s.options(None).unwrap().len(), 3);
        assert_eq!(d.options(None).unwrap().len(), 4);
    }

    #[test]
    fn a_setting_during_a_capture_is_refused_and_nothing_mixes_with_requests() {
        let (mut d, state) = bidi_device();
        state.lock().script = VecDeque::from([Answer::Silent]);
        d.request_timeout = Duration::from_secs(20);
        let rx = start(&mut d);
        state.wait_for_requests(1);
        assert!(d.set(&Key::PowerOff, &Value::Bool(true), None).is_err());
        assert!(d.set(&Key::LimitSamples, &Value::U64(1), None).is_err());
        assert!(matches!(d.start(PacketSink::new(sync_channel(1).0, 0)), Err(Error::Busy)));
        assert_eq!(state.commands(), [8]);
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.close().unwrap();
    }

    #[test]
    fn a_silent_meter_fails_a_capture_only_a_sample_limit_can_end() {
        let (mut d, state) = bidi_device();
        d.request_timeout = Duration::from_millis(10);
        d.silence_bound = Duration::from_secs(2);
        state.lock().script = VecDeque::from(vec![Answer::Silent; 100_000]);
        limit(&mut d, 1);
        let began = Instant::now();
        let (values, failed) = ending(&start(&mut d));
        assert_eq!(values, Vec::<f32>::new());
        assert_eq!(
            failed.as_deref(),
            Some("device error: no usable answer from the meter for 2 s")
        );
        assert!(began.elapsed() >= Duration::from_secs(2), "not before its time");
        assert!(state.commands().len() >= 2, "it asked again on the way");
        d.close().unwrap();
    }

    #[test]
    fn garbage_is_no_answer_and_fails_the_capture_all_the_same() {
        let (mut d, state) = bidi_device();
        d.silence_bound = Duration::from_millis(500);
        state.lock().endless = vec![9; 14];
        state.lock().script = VecDeque::from(vec![Answer::Silent; 100_000]);
        limit(&mut d, 1);
        let (values, failed) = ending(&start(&mut d));
        assert_eq!(values, Vec::<f32>::new());
        assert!(failed.is_some_and(|f| f.contains("no usable answer")));
        d.close().unwrap();
        // In step, with its checksum wrong: an answer in form, and of no use.
        let (mut d, state) = bidi_device();
        d.silence_bound = Duration::from_millis(500);
        let mut wrong = measure_answer(1000);
        wrong[13] ^= 1;
        state.lock().endless = wrong;
        state.lock().script = VecDeque::from(vec![Answer::Silent; 100_000]);
        limit(&mut d, 1);
        let (values, failed) = ending(&start(&mut d));
        assert_eq!(values, Vec::<f32>::new());
        assert!(failed.is_some_and(|f| f.contains("no usable answer")));
        d.close().unwrap();
    }

    #[test]
    fn answers_keep_the_silence_bound_away() {
        let (mut d, state) = bidi_device();
        d.silence_bound = Duration::from_millis(1500);
        d.request_timeout = Duration::from_secs(20);
        // A byte per two reads, of which one finds nothing and waits a millisecond.
        state.lock().pace = 2;
        state.lock().real_time = true;
        limit(&mut d, 150);
        let began = Instant::now();
        let (values, failed) = ending(&start(&mut d));
        assert_eq!((values.len(), failed), (150, None));
        assert!(began.elapsed() > d.silence_bound, "or this says nothing");
        d.close().unwrap();
    }

    #[test]
    fn a_time_limit_outlasts_the_silence_bound() {
        let (mut d, state) = bidi_device();
        d.silence_bound = Duration::from_millis(20);
        state.lock().script = VecDeque::from(vec![Answer::Silent; 100_000]);
        d.set(&Key::LimitMsec, &Value::U64(200), None).unwrap();
        let began = Instant::now();
        readings(&start(&mut d));
        assert!(began.elapsed() >= Duration::from_millis(200));
        d.close().unwrap();
    }

    #[test]
    fn with_no_limit_only_a_stop_ends_a_silent_capture() {
        let (mut d, state) = bidi_device();
        d.silence_bound = Duration::from_millis(1);
        state.lock().script = VecDeque::from(vec![Answer::Silent; 100_000]);
        let rx = start(&mut d);
        state.wait_for_requests(3);
        assert!(matches!(rx.try_recv().map(|p| p.1), Ok(Packet::Header { .. })));
        assert!(rx.try_recv().is_err(), "no End");
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.close().unwrap();
    }

    #[test]
    fn a_stall_in_the_middle_of_an_answer_sends_no_extra_request() {
        let (mut d, state) = bidi_device();
        d.request_timeout = Duration::from_millis(30);
        state.lock().stall = Some((Duration::from_millis(100), 5));
        limit(&mut d, 1);
        assert_eq!(readings(&start(&mut d)), [nth(1)]);
        assert_eq!(state.commands(), [8, 8], "the first, and the one after the limit");
        d.close().unwrap();
    }

    #[test]
    fn a_close_forgets_what_was_owed() {
        let (mut d, state) = bidi_device();
        state.lock().pace = 3;
        limit(&mut d, 1);
        readings(&start(&mut d));
        assert_eq!(d.link.lock().unwrap().owed, 1, "the request after the limit");
        d.close().unwrap();
        assert_eq!(d.link.lock().unwrap().owed, 0);
    }

    #[test]
    fn a_stop_bites_while_answers_keep_coming() {
        let (mut d, state) = bidi_device();
        let rx = start(&mut d);
        state.wait_for_requests(5);
        d.stop().unwrap();
        readings(&rx);
        let seen = state.commands().len();
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(state.commands().len(), seen, "and no request after it");
        d.close().unwrap();
    }

    #[test]
    fn a_failed_write_fails_a_capture() {
        let (mut d, state) = bidi_device();
        state.lock().fail_writes_after = Some(2);
        let (_, failed) = ending(&start(&mut d));
        assert_eq!(failed.as_deref(), Some("device error: the port went"));
        assert_eq!(state.lock().writes.len(), 2);
        d.close().unwrap();
    }

    #[test]
    fn a_failed_last_request_fails_nothing() {
        let (mut d, state) = bidi_device();
        state.lock().fail_writes_after = Some(1);
        limit(&mut d, 1);
        let (values, failed) = ending(&start(&mut d));
        assert_eq!((values, failed), (vec![nth(1)], None));
        d.close().unwrap();
    }

    #[test]
    fn a_failed_first_request_ends_with_a_header_and_a_failure() {
        let (mut d, state) = bidi_device();
        state.lock().fail_writes_after = Some(0);
        let (values, failed) = ending(&start(&mut d));
        assert_eq!(values, Vec::<f32>::new());
        assert_eq!(failed.as_deref(), Some("device error: the port went"));
        d.close().unwrap();
    }

    #[test]
    fn a_short_write_fails_a_capture_and_says_which() {
        let (mut d, state) = bidi_device();
        state.lock().short_writes = true;
        let (_, failed) = ending(&start(&mut d));
        assert_eq!(failed.as_deref(), Some("device error: the first request did not go out"));
        d.close().unwrap();
        let (mut d, state) = bidi_device();
        let rx = start(&mut d);
        state.wait_for_requests(3);
        state.lock().short_writes = true;
        let (_, failed) = ending(&rx);
        assert_eq!(failed.as_deref(), Some("device error: a request did not go out"));
        d.close().unwrap();
    }

    #[test]
    fn a_read_that_fails_fails_a_capture_in_either_mode() {
        let (mut d, state) = bidi_device();
        state.lock().fail_reads = true;
        let (_, failed) = ending(&start(&mut d));
        assert_eq!(failed.as_deref(), Some("device error: the port went"));
        d.close().unwrap();
        let (mut d, state) = send_device(Model::M26SM);
        state.lock().fail_reads = true;
        let (_, failed) = ending(&start(&mut d));
        assert_eq!(failed.as_deref(), Some("device error: the port went"));
        d.close().unwrap();
    }

    #[test]
    fn a_limit_a_stop_and_a_time_up_fail_nothing() {
        let (mut d, _) = bidi_device();
        limit(&mut d, 1);
        assert_eq!(ending(&start(&mut d)).1, None);
        d.close().unwrap();
        let (mut d, state) = bidi_device();
        let rx = start(&mut d);
        state.wait_for_requests(2);
        d.stop().unwrap();
        assert_eq!(ending(&rx).1, None);
        d.close().unwrap();
        let (mut d, _) = send_device(Model::M26SM);
        d.set(&Key::LimitMsec, &Value::U64(20), None).unwrap();
        assert_eq!(ending(&start(&mut d)).1, None);
        d.close().unwrap();
    }

    #[test]
    fn nothing_is_written_after_a_close_and_a_restart_lets_go_of_the_old_sink() {
        let (mut d, state) = bidi_device();
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        state.wait_for_requests(3);
        d.stop().unwrap();
        readings(&rx);
        limit(&mut d, 1);
        let second = start(&mut d);
        assert!(
            matches!(rx.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)),
            "the old thread is gone with its sink"
        );
        readings(&second);
        d.close().unwrap();
        let seen = state.lock().writes.len();
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(state.lock().writes.len(), seen);
    }

    /// A simulated meter: each answer `latency` after its request, or `service`
    /// apart when it queues them. Returns the most owed, the requests sent, the
    /// samples taken and the oldest request a sample answered, over `length`.
    fn simulated(latency: Duration, queues: bool, length: Duration) -> (u32, u32, u32, Duration) {
        let base = Instant::now();
        let mut now = base;
        let mut r = Requests::new(0, REQUEST_TIMEOUT, REQUEST_TIMEOUT, now);
        // When each answer comes, and when its request went.
        let mut coming: VecDeque<(Instant, Instant)> = VecDeque::new();
        let mut free_at = now;
        let (mut most, mut sent, mut samples) = (0, 0, 0);
        let mut oldest = Duration::ZERO;
        while now < base + length {
            while coming.front().is_some_and(|(at, _)| *at <= now) {
                let (_, asked) = coming.pop_front().expect("just seen");
                if r.answer(now) {
                    samples += 1;
                    oldest = oldest.max(now - asked);
                }
            }
            let turn = r.turn(now);
            if turn == Turn::Expired {
                r.expire();
            }
            if turn != Turn::Wait {
                let at = if queues {
                    free_at = free_at.max(now) + latency;
                    free_at
                } else {
                    now + latency
                };
                coming.push_back((at, now));
                r.asked(now);
                sent += 1;
            }
            most = most.max(r.owed);
            now += Duration::from_millis(10);
        }
        (most, sent, samples, oldest)
    }

    #[test]
    fn a_simulated_minute_at_a_steady_latency_owes_little() {
        const MINUTE: Duration = Duration::from_secs(60);
        for queues in [false, true] {
            let (most, sent, samples, oldest) =
                simulated(Duration::from_millis(2500), queues, MINUTE);
            assert!(most <= 2, "queues {queues}: {most} owed");
            assert!(sent <= 30, "queues {queues}: {sent} sent");
            assert!(samples >= 15, "queues {queues}: {samples} samples");
            assert!(oldest <= Duration::from_secs(5), "queues {queues}: {oldest:?} old");
        }
        // A meter in time: one request at a time, none expires.
        let (most, sent, samples, oldest) = simulated(Duration::from_millis(15), false, MINUTE);
        assert_eq!(most, 1);
        assert_eq!(sent, samples + u32::from(sent > samples));
        assert!(oldest <= Duration::from_millis(30));
    }

    #[test]
    fn past_every_wait_what_is_owed_stays_bounded() {
        let (most, _, _, _) = simulated(Duration::from_secs(1000), false, Duration::from_secs(600));
        assert!(most <= MAX_OWED, "{most} owed");
        let (most, sent, samples, _) =
            simulated(Duration::from_secs(20), true, Duration::from_secs(600));
        assert!(most <= MAX_OWED, "{most} owed");
        assert!(samples > 0 || sent > 0);
    }

    #[test]
    fn a_lost_answer_costs_three_seconds_and_sets_the_wait_back() {
        let base = Instant::now();
        let at = |ms: u64| base + Duration::from_millis(ms);
        let mut r = Requests::new(0, REQUEST_TIMEOUT, REQUEST_TIMEOUT, at(0));
        assert_eq!(r.turn(at(0)), Turn::Ask);
        r.asked(at(0));
        assert_eq!(r.turn(at(1000)), Turn::Wait, "not at its second");
        assert_eq!(r.turn(at(1001)), Turn::Expired);
        r.expire();
        assert_eq!(r.owed, 1, "nothing heard: it may yet come");
        assert_eq!(r.wait, Duration::from_secs(2));
        r.asked(at(1001));
        assert!(!r.answer(at(1010)), "with another owed, it is an earlier one's");
        assert_eq!(r.turn(at(3010)), Turn::Wait);
        assert_eq!(r.turn(at(3011)), Turn::Expired);
        r.expire();
        assert_eq!(r.owed, 0, "an answer heard: what is owed was lost");
        r.asked(at(3011));
        assert!(r.answer(at(3020)));
        assert_ne!(r.wait, REQUEST_TIMEOUT, "one in time may be an older one's");
        r.asked(at(3020));
        assert!(r.answer(at(3030)));
        assert_eq!(r.wait, REQUEST_TIMEOUT, "two in time, so back to a second");
    }

    #[test]
    fn what_a_capture_left_owed_is_dropped_and_then_given_up_on() {
        let base = Instant::now();
        let at = |ms: u64| base + Duration::from_millis(ms);
        let mut r = Requests::new(2, REQUEST_TIMEOUT, Duration::from_secs(2), at(0));
        assert_eq!(r.turn(at(0)), Turn::Wait, "no request while they are owed");
        assert!(!r.answer(at(100)));
        assert!(!r.answer(at(200)), "the last of them too");
        assert_eq!(r.turn(at(200)), Turn::Ask);
        let mut r = Requests::new(2, REQUEST_TIMEOUT, Duration::from_secs(2), at(0));
        assert_eq!(r.turn(at(4000)), Turn::Wait, "twice what that capture had come to wait");
        assert_eq!(r.turn(at(4001)), Turn::Expired);
        r.expire();
        assert_eq!((r.owed, r.to_drop), (0, 0));
        let r = Requests::new(0, REQUEST_TIMEOUT, Duration::from_secs(8), at(0));
        assert_eq!(r.wait, REQUEST_TIMEOUT, "with none left it starts over");
    }

    #[test]
    fn answers_taken_for_lost_are_still_dropped_by_the_next_capture() {
        let base = Instant::now();
        let at = |ms: u64| base + Duration::from_millis(ms);
        let mut r = Requests::new(0, REQUEST_TIMEOUT, REQUEST_TIMEOUT, at(0));
        r.asked(at(0));
        assert_eq!(r.turn(at(1001)), Turn::Expired);
        r.expire();
        r.asked(at(1001));
        assert!(!r.answer(at(1500)));
        assert_eq!(r.turn(at(3501)), Turn::Expired);
        r.expire();
        assert_eq!((r.owed, r.doubtful), (0, 1));
        assert_eq!(r.left(), 1, "taken for lost, and it may yet come");
        r.asked(at(3501));
        assert_eq!(r.left(), 2);
        assert!(r.answer(at(3510)));
        r.asked(at(3510));
        assert!(r.answer(at(3520)));
        assert_eq!(r.left(), 0, "two in time: the line is in step");
    }

    #[test]
    fn a_stray_answer_with_none_owed_goes_unused() {
        let now = Instant::now();
        let mut r = Requests::new(0, REQUEST_TIMEOUT, REQUEST_TIMEOUT, now);
        assert!(!r.answer(now));
        assert_eq!(r.owed, 0);
    }

    #[test]
    fn the_warnings_say_what_happened() {
        assert_eq!(
            no_data_warning(SILENCE_WARNING),
            "No data for 30 s; the meter's rate may be slower."
        );
        assert_eq!(no_reading_warning(SILENCE_WARNING), "No new reading for 30 s; still asking.");
    }

    #[test]
    fn a_silence_is_told_once_until_data_comes() {
        let base = Instant::now();
        let mut note = SilenceNote::new(Duration::from_secs(30));
        assert!(!note.due(base, base + Duration::from_secs(29)));
        assert!(note.due(base, base + Duration::from_secs(30)));
        assert!(!note.due(base, base + Duration::from_secs(31)), "once");
        note.data();
        assert!(note.due(base, base + Duration::from_secs(32)), "again after data");
    }

    /// A 13 byte block of `value` in V DC.
    fn block(value: u32) -> Vec<u8> {
        let mut b = vec![0x01, 0x31, 0x30, 0x30, 0x31];
        let mut v = value;
        for _ in 0..6 {
            b.push(0x30 | (v % 10) as u8);
            v /= 10;
        }
        b.extend([0x30, 0x33]);
        b
    }

    #[test]
    fn send_mode_reads_blocks_writes_nothing_and_checks_its_limit_when_bytes_run_out() {
        let (mut d, state) = send_device(Model::M26SM);
        {
            let mut s = state.lock();
            // All there at once: the limit waits for the end of the pass, as C's.
            for n in 1..=4 {
                let b = block(n * 1000);
                s.input.extend(b);
            }
        }
        limit(&mut d, 2);
        assert_eq!(readings(&start(&mut d)), [nth(1), nth(2), nth(3), nth(4)]);
        assert_eq!(state.lock().writes.len(), 0);
        d.close().unwrap();
    }

    #[test]
    fn send_mode_takes_blocks_as_slowly_as_they_come() {
        let (mut d, state) = send_device(Model::M26SM);
        {
            let mut s = state.lock();
            s.pace = 3;
            let b = block(1000);
            s.coming.extend(b);
            let b = block(2000);
            s.coming.extend(b);
        }
        limit(&mut d, 2);
        assert_eq!(readings(&start(&mut d)), [nth(1), nth(2)]);
        d.close().unwrap();
    }

    #[test]
    fn nothing_bounds_a_silent_meter_in_send_mode() {
        let (mut d, _state) = send_device(Model::M26SM);
        d.silence_bound = Duration::from_millis(1);
        limit(&mut d, 1);
        let rx = start(&mut d);
        std::thread::sleep(Duration::from_millis(50));
        assert!(matches!(rx.try_recv().map(|p| p.1), Ok(Packet::Header { .. })));
        assert!(rx.try_recv().is_err(), "still waiting: a meter's rate may be minutes");
        d.stop().unwrap();
        assert_eq!(readings(&rx), Vec::<f32>::new());
        d.close().unwrap();
    }

    #[test]
    fn send_mode_stops_against_an_endless_stream_and_on_a_time_limit() {
        let (mut d, state) = send_device(Model::M26SM);
        state.lock().endless = block(1000);
        let rx = start(&mut d);
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut got = 0;
        while got < 10 {
            let left = deadline.saturating_duration_since(Instant::now());
            if matches!(rx.recv_timeout(left).expect("samples").1, Packet::Analog(_)) {
                got += 1;
            }
        }
        d.stop().unwrap();
        // It reads on while the queue has room; the End must come all the same.
        readings(&rx);
        d.close().unwrap();
        let (mut d, _) = send_device(Model::M26SM);
        d.set(&Key::LimitMsec, &Value::U64(100), None).unwrap();
        let began = Instant::now();
        readings(&start(&mut d));
        assert!(began.elapsed() >= Duration::from_millis(100));
        d.close().unwrap();
    }

    #[test]
    fn a_disabled_channel_sends_nothing_and_still_counts() {
        let (mut d, _) = bidi_device();
        d.enable_channel(0, false).unwrap();
        limit(&mut d, 2);
        assert_eq!(readings(&start(&mut d)), Vec::<f32>::new());
        d.close().unwrap();
    }

    #[test]
    fn a_close_with_a_full_queue_returns_and_leaves_no_thread() {
        let (mut d, state) = send_device(Model::M26SM);
        state.lock().endless = block(1000);
        let (tx, rx) = sync_channel(1);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // Unread from here on: the header sits there and the first sample blocks.
        std::thread::sleep(Duration::from_millis(20));
        let (done_tx, done_rx) = sync_channel(1);
        std::thread::spawn(move || {
            d.close().unwrap();
            let _ = done_tx.send(());
        });
        done_rx.recv_timeout(CAPTURE_BOUND).expect("close came back");
        // It joined: the only sender is gone, with a free queue or not.
        while rx.try_recv().is_ok() {}
        assert!(matches!(rx.try_recv(), Err(std::sync::mpsc::TryRecvError::Disconnected)));
    }

    #[test]
    fn a_start_after_an_end_stuck_on_a_full_queue_does_not_hang() {
        let (mut d, _) = bidi_device();
        limit(&mut d, 1);
        // Room for the header and the sample: the End cannot go.
        let (tx, rx) = sync_channel(2);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let (done_tx, done_rx) = sync_channel(1);
        std::thread::spawn(move || {
            let deadline = Instant::now() + CAPTURE_BOUND;
            while d.capturing.load(Ordering::SeqCst) && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(1));
            }
            let started = d.start(PacketSink::new(sync_channel(16).0, 0));
            let _ = done_tx.send(started.is_ok());
            let _ = d.close();
        });
        assert!(done_rx.recv_timeout(CAPTURE_BOUND).expect("start came back"));
        drop(rx);
    }

    #[test]
    fn close_leaves_the_model_and_a_reopen_decodes_as_before() {
        let (mut d, state) = send_device(Model::M26SM);
        state.lock().input.extend(block(1000));
        limit(&mut d, 1);
        assert_eq!(readings(&start(&mut d)), [nth(1)]);
        d.close().unwrap();
        assert_eq!(d.link.lock().unwrap().decoder.model, Model::M26SM);
    }

    #[test]
    fn a_scan_in_send_mode_wants_a_start_and_four_data_bytes() {
        let found = |bytes: &[u8]| {
            let (mut port, state) = fake();
            state.lock().coming.extend(bytes);
            state.lock().pace = 1;
            scan_model_send(&mut *port, Duration::from_millis(60))
        };
        assert_eq!(found(&[0x01, 0x31, 0x30, 0x30, 0x31]), Model::M26SM);
        assert_eq!(found(&[0x04, 0x31, 0x30, 0x30, 0x31]), Model::M12S);
        assert_eq!(found(&[0xc1, 0xf1, 0xf0, 0xf0, 0xf1]), Model::M26SM, "bits 7 and 6");
        assert_eq!(found(&[0x01, 0x31]), Model::None, "short of data");
        assert_eq!(found(&[0x00, 0x31, 0x30, 0x30, 0x31]), Model::None, "no such code");
        assert_eq!(found(&[0x01, 0x31, 0x30, 0x11, 0x31]), Model::None, "a start in the data");
        let mut late = vec![0x30; 99];
        late.extend([0x01, 0x31, 0x30, 0x30, 0x31]);
        assert_eq!(found(&late), Model::M26SM, "the hundredth byte");
        late.insert(0, 0x30);
        assert_eq!(found(&late), Model::None, "the hundred and first");
        assert_eq!(found(&[]), Model::None);
    }

    #[test]
    fn a_scan_in_send_mode_flushes_first_and_tries_the_second_rate_unless_given() {
        struct Rates {
            params: Arc<Mutex<Vec<u32>>>,
        }
        impl Port for Rates {
            fn name(&self) -> &'static str {
                "/dev/ttyFAKE"
            }
            fn set_params(&mut self, p: Params) -> Result<()> {
                self.params.lock().unwrap().push(p.baud);
                Ok(())
            }
            fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
                Ok(())
            }
            fn read(&mut self, _: &mut [u8], timeout: Duration) -> Result<usize> {
                std::thread::sleep(timeout);
                Ok(0)
            }
            fn write(&mut self, _: &[u8]) -> Result<()> {
                Ok(())
            }
            fn purge(&mut self) -> Result<()> {
                Ok(())
            }
        }
        let params = Arc::new(Mutex::new(Vec::new()));
        let port = Rates { params: Arc::clone(&params) };
        let line = Params::parse(SERIALCOMM_2X_RS232).unwrap();
        assert!(probe_send(Box::new(port), line, false, Duration::from_millis(5)).is_none());
        assert_eq!(*params.lock().unwrap(), [8228]);
        params.lock().unwrap().clear();
        let port = Rates { params: Arc::clone(&params) };
        assert!(probe_send(Box::new(port), line, true, Duration::from_millis(5)).is_none());
        assert_eq!(*params.lock().unwrap(), Vec::<u32>::new(), "given: that one only");
    }

    #[test]
    fn a_bidirectional_scan_writes_three_times_flushes_and_asks_and_finds_every_meter() {
        let (port, state) = fake();
        let params = Params::parse(SERIALCOMM_2X).unwrap();
        let mut second = status_answer();
        second[0] = 2;
        second[13] = protocol::checksum(&second);
        state.lock().coming.extend(second);
        state.lock().script = VecDeque::from(vec![Answer::Silent; 3]);
        let meters =
            probe_bidi(port, params, Duration::from_millis(1), Duration::from_millis(80)).unwrap();
        assert_eq!(state.lock().writes.len(), 4);
        assert_eq!(state.lock().writes[0], protocol::status_request(0));
        assert_eq!(state.lock().purges, 1);
        let addrs: Vec<u8> = meters.iter().map(|m| m.addr).collect();
        assert_eq!(meters.len(), 2, "the second meter's answer and the last request's");
        assert!(addrs.contains(&1) && addrs.contains(&2), "{addrs:?}");
        assert_eq!(meters[0].firmware, (1, 2));
        assert_eq!(meters[0].model, Model::M26S);
    }

    #[test]
    fn a_bidirectional_scan_with_no_answer_is_no_device_and_with_a_failed_write_too() {
        let (port, state) = fake();
        state.lock().script = VecDeque::from(vec![Answer::Silent; 8]);
        let params = Params::parse(SERIALCOMM_2X).unwrap();
        let wait = Duration::from_millis(1);
        assert!(probe_bidi(port, params, wait, Duration::from_millis(20)).unwrap().is_empty());
        let (port, state) = fake();
        state.lock().fail_writes_after = Some(2);
        assert!(probe_bidi(port, params, wait, wait).is_err());
    }
}
