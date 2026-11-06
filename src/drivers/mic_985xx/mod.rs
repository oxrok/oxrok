// Counterpart in sigrok:
//   libsigrok/src/hardware/mic-985xx/api.c,
//   libsigrok/src/hardware/mic-985xx/protocol.c,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang,
//   Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The MIC 98581 temperature logger and MIC 98583 temperature and humidity
//! logger, a PL2303 serial port on USB.
//!
//! Each is its own driver id, since nothing asks the logger what it is. A scan
//! needs `conn=`, opens the port and closes it, and has found a logger if the
//! port opened: it says no word to the device, so any port will do. `serialcomm=`
//! overrides 38400/8n2.
//!
//! A capture ends at `limit_samples`, `limit_time` or a stop. It listens, and
//! at the first 100 ms of silence it sends the logger's real-time command, once;
//! a logger already sending faster than that is not told. The limits are looked
//! at once all of a read is out, so a read holding two readings sends both.
//!
//! The protocol is in [`protocol`].
//!
//! Not as in C, each with what it costs:
//! - The third digit of a field counts: `v 235` is 23.5. C divides it away and
//!   says 23.0 (protocol.c:83, :87). That the digit is real is an inference from
//!   the field's scale and the driver's one decimal: there is no recording. If a
//!   logger's third digit is always 0, nothing changes.
//! - The command is sent per capture. C's flag is a function's `static`
//!   (protocol.c:189), so in one process only the first capture of any MIC logger
//!   sends it, and a second logger, or one replugged, stays silent.
//! - A port that hangs up ends the capture. `receive_data` takes it for a timeout
//!   (protocol.c:202-211), for ever at a sample limit.
//! - When only a sample limit could end a capture, [`NO_READING_BOUND`] without
//!   a reading ends it with an error, whether or not bytes came. C waits for ever.
//!   Nobody has said how often a logger sends, so the bound is wide.
//! - A capture's receive buffer lives as long as the capture, where C keeps it
//!   between two, and the bytes queued between two captures on an open port are
//!   dropped when the second starts: they are old readings.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
pub use protocol::{MODELS, Model};
use protocol::{REALTIME, Realtime, Starved, Watch};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES_TEMP: [DeviceClass; 1] = [DeviceClass::Thermometer];
const CLASSES_TEMP_HUM: [DeviceClass; 2] = [DeviceClass::Thermometer, DeviceClass::Hygrometer];
const SET_ONLY: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET_ONLY),
    OptionDesc::new(Key::LimitMsec, SET_ONLY),
];

/// `SERIAL_BUFSIZE`.
const BUF_SIZE: usize = 256;
/// The digits a reading says it has, `sr_analog_init`'s 1.
const DIGITS: i32 = 1;
/// How long a thread that was told to stop waits to put its `End` in a queue no
/// one reads. A close joins it. An output may have rows pending on the `End`, so
/// this outlasts a reader's short stall.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long a capture that only a sample limit could end goes without a reading.
pub const NO_READING_BOUND: Duration = Duration::from_secs(30);

/// The driver of one logger.
pub struct MicDriver(pub &'static Model);

/// Registers a driver for each logger.
pub fn register_all(registry: &mut crate::core::Registry) {
    for model in &MODELS {
        registry.register(Box::new(MicDriver(model)));
    }
}

fn classes(model: &Model) -> &'static [DeviceClass] {
    if model.has_humidity { &CLASSES_TEMP_HUM } else { &CLASSES_TEMP }
}

impl Driver for MicDriver {
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
        classes(self.0)
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = self.0.serialcomm.to_owned();
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
        let (params, port) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{}: {e}", self.0.id);
                return Ok(Vec::new());
            }
        };
        log::info!("Found device on port {conn}.");
        // `serial_close`; `dev_open` opens the port again.
        drop(port);
        Ok(vec![Box::new(MicDevice::new(self.0, conn, params, None))])
    }
}

/// The software limits, which a set changes under a running capture, as in C.
#[derive(Debug, Default)]
struct Limits {
    samples: AtomicU64,
    msec: AtomicU64,
}

/// One logger.
pub struct MicDevice {
    model: &'static Model,
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limits: Arc<Limits>,
    /// [`NO_READING_BOUND`], which a test shortens.
    bound: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The port, unless a capture holds it. It comes back before `End`.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    /// A capture ran since the port was opened, so bytes may wait in it.
    captured: bool,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl MicDevice {
    fn new(
        model: &'static Model,
        conn: String,
        params: Params,
        port: Option<Box<dyn Port>>,
    ) -> Self {
        let mut channels = vec![Channel::new(0, ChannelKind::Analog, "Temperature")];
        if model.has_humidity {
            channels.push(Channel::new(1, ChannelKind::Analog, "Humidity"));
        }
        MicDevice {
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
            bound: NO_READING_BOUND,
            pending: port,
            port: Arc::new(Mutex::new(None)),
            captured: false,
            worker: None,
        }
    }

    /// A device of `model` on an already open port.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let name = port.name().to_owned();
        let params = Params::parse(model.serialcomm).expect("table parameters parse");
        Self::new(model, name, params, Some(port))
    }

    /// Tells a worker to stop and waits for it.
    fn join(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

impl Device for MicDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        classes(self.model)
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

    /// `config_get` is NULL in C: nothing to read.
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
        if self.captured
            && let Err(e) = port.purge()
        {
            log::debug!("Flush: {e}.");
        }
        self.captured = true;
        let capture = Capture {
            model: self.model,
            params: self.params,
            limits: Arc::clone(&self.limits),
            bound: self.bound,
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

impl Drop for MicDevice {
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
    model: &'static Model,
    params: Params,
    limits: Arc<Limits>,
    bound: Duration,
}

impl Capture {
    /// Runs until a limit, a stop, a port that fails, a bound or a consumer that
    /// left.
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let send = |p: Packet| sink.send_while(p, || !stop.load(Ordering::SeqCst));
        if !send(Packet::header()) {
            return Ok(());
        }
        let started = Instant::now();
        let mut realtime = Realtime::new(started);
        let mut watch = Watch::new(self.bound, started);
        let mut samples = 0u64;
        let mut buf: Vec<u8> = Vec::with_capacity(BUF_SIZE);
        let mut chunk = [0u8; BUF_SIZE];
        while !stop.load(Ordering::SeqCst) {
            // Bytes that are ready win over the timer, as `fd_source_check`
            // looks at `revents` first.
            let room = BUF_SIZE - buf.len();
            match port.read(&mut chunk[..room], realtime.wait(Instant::now())) {
                Err(e) => {
                    log::error!("{}", read_error_line(&e));
                    return Err(e);
                }
                Ok(0) => {
                    if realtime.empty(Instant::now()) {
                        self.send_command(port);
                    } else {
                        // A read that gave nothing at once must not spin.
                        std::thread::sleep(Duration::from_millis(1));
                    }
                }
                Ok(n) => {
                    watch.heard();
                    buf.extend_from_slice(&chunk[..n]);
                    // handle_new_data: as many frames as there are, stepping
                    // over the bytes that start none.
                    let size = self.model.packet_size();
                    let mut at = 0;
                    while buf.len() - at >= size {
                        let window = &buf[at..at + size];
                        if !self.model.valid(window) {
                            at += 1;
                            continue;
                        }
                        if !self.reading(window, &send) {
                            return Ok(());
                        }
                        samples += 1;
                        // After the send: a slow client's time is not the
                        // logger's.
                        watch.reading(Instant::now());
                        at += size;
                    }
                    buf.drain(..at);
                    // After the sends, as `fd_source_dispatch` sets its time
                    // after the callback: a slow client is no silence.
                    realtime.heard(Instant::now());
                }
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
            if limit > 0 && msec == 0 {
                self.check_starved(&watch)?;
            }
        }
        Ok(())
    }

    /// `mic_send`: its result ends nothing, as a port that is gone says so at the
    /// next read, and a write that found no room is no error there.
    fn send_command(&self, port: &mut dyn Port) {
        let timeout = crate::serial::serial_timeout(&self.params, REALTIME.len() as u32);
        match port.write_timeout(REALTIME.as_bytes(), timeout) {
            Ok(n) if n == REALTIME.len() => {}
            Ok(n) => log::debug!("The command found room for {n} bytes."),
            Err(e) => log::error!("{}", command_error_line(&e)),
        }
    }

    /// `handle_packet`: false when the sink would not take it.
    fn reading(&self, frame: &[u8], send: &impl Fn(Packet) -> bool) -> bool {
        let (temperature, humidity) = self.model.parse(frame);
        let packet = |channel, value, mq, unit| {
            Packet::Analog(AnalogData {
                channels: vec![channel],
                samples: vec![value],
                mq,
                flags: MqFlags::empty(),
                unit,
                digits: DIGITS,
            })
        };
        if !send(packet(0, temperature, Mq::Temperature, Unit::Celsius)) {
            return false;
        }
        humidity.is_none_or(|h| send(packet(1, h, Mq::RelativeHumidity, Unit::Percentage)))
    }

    /// The bound, when only a sample limit could end the capture.
    fn check_starved(&self, watch: &Watch) -> Result<()> {
        let Some(starved) = watch.starved(Instant::now()) else {
            return Ok(());
        };
        let (line, why) = starved_words(starved, self.model, self.bound.as_secs());
        log::error!("{line}");
        Err(Error::device(why))
    }
}

/// What a failed read logs, as C words it.
fn read_error_line(e: &Error) -> String {
    format!("Serial port read error: {e}.")
}

/// What a failed command logs, as C words it, its CR and all.
fn command_error_line(e: &Error) -> String {
    format!("Error sending '{REALTIME}' command: {e}.")
}

/// What a capture that starved logs, and what it fails with.
fn starved_words(starved: Starved, model: &Model, seconds: u64) -> (String, String) {
    match starved {
        Starved::NoBytes => (
            format!(
                "No bytes from the logger for {seconds} s: it is off, its cable is out, or it \
                 did not take the command."
            ),
            format!("no bytes from the logger for {seconds} s"),
        ),
        Starved::NoReading => (
            format!(
                "No usable data from the logger for {seconds} s: bytes came, but none made a \
                 reading of a {}.",
                model.long_name
            ),
            format!("no usable data from the logger for {seconds} s"),
        ),
    }
}

#[cfg(test)]
mod tests;
