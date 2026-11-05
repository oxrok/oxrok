// Counterpart in sigrok:
//   libsigrok/src/hardware/zketech-ebd-usb/api.c,
//   libsigrok/src/hardware/zketech-ebd-usb/protocol.c,
//   Copyright (C) 2018 Sven Bursch-Osewold <sb_git@bursch.com>,
//   Copyright (C) 2019 King Kévin <kingkevin@cuvoodoo.info>,
//   with commits by Uwe Hermann, Frank Stettner, Florian Ragwitz, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The ZKETECH EBD-USB+, a 35 W electronic load behind a USB serial adapter,
//! which streams its measurements once told to.
//!
//! `conn=` names the port and `serialcomm=` overrides 9600/8e1. A scan starts the
//! stream, reads one frame of any kind and stops it, and does not take what the
//! frame says.
//!
//! **The settings reach the load in a capture only,** as in C: `current_limit`
//! and `uvc_threshold` read back what was set, starting at 0, whatever the device
//! holds. So `--set` alone sends nothing, for which the close warns. In a capture
//! each measurement compares what the device says with what was set: with a limit
//! the load is started, again and again if it stays off, and with none **a load
//! that is on is turned off**, so a capture with nothing set turns off one that
//! was left on. A stop turns the load off only if the last frame said it was on,
//! as in C, so a capture that ends right after starting it leaves it on; and one
//! that ends because the device went quiet sends nothing. Both warn. What the stop
//! command `06` itself does to a load is not known.
//!
//! Where this differs from C, each with its cause in [`Device::set`] or
//! [`run_capture`]:
//!
//! - `send_cmd` compares a last write's 1 with the whole command's length
//!   (`protocol.c:58`), so every command that goes out counts as failed. It shows
//!   only in a setter's return, as all others drop it. Here it counts as sent.
//!   The same return hides `ebd_loadstart`'s last lines (`protocol.c:115-123`).
//! - A NaN passes both of `config_set`'s comparisons (`api.c:176,181`). Here it
//!   is refused, as the setpoints of a capture would hold what x86 makes of it.
//! - After an unplug C's read neither ends nor fails. Here it ends the capture.
//! - An error in a read has C try for ever (`protocol.c:299-301`, `return SR_ERR`
//!   keeps the source). Here it ends the capture.
//!
//! C's read turns are kept, and so its silence of some 2 s for one frame. Frames
//! that fail their checks are dropped, as in C, so a device that keeps sending
//! only those holds a capture with a sample limit until it is stopped.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use protocol::{Measurement, Message, Parsed};
use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::ElectronicLoad];
const SERIALCOMM: &str = "9600/8e1";
const CHANNELS: [&str; 4] = ["V.VBUS", "I.VBUS", "V.D+", "V.D-"];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::UnderVoltageConditionThreshold, Access::GET_SET_LIST),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
];
/// `config_set`'s and `config_list`'s range and step of the current limit.
const LIMIT: [f64; 3] = [0.0, 4.0, 0.001];
/// `config_set`'s and `config_list`'s range and step of the threshold.
const THRESHOLD: [f64; 3] = [0.0, 21.0, 0.01];
/// `send_cmd`'s wait between bytes, or the device loses some.
const BYTE_GAP: Duration = Duration::from_millis(10);
/// The pause after a read that gave nothing, with the lock free: a mutex is not
/// fair, and a reader that takes it back at once would starve a setter.
const EMPTY_READ_PAUSE: Duration = Duration::from_millis(1);
/// How long a stopped capture tries to get its End to a queue no one may read.
const END_AFTER_STOP: Duration = Duration::from_secs(1);

/// The `zketech-ebd-usb` driver.
pub struct ZketechEbdUsbDriver;

impl Driver for ZketechEbdUsbDriver {
    fn id(&self) -> &'static str {
        "zketech-ebd-usb"
    }

    fn long_name(&self) -> &'static str {
        "ZKETECH EBD-USB"
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
        // Every failure of the port here is no device, as `scan` returns NULL.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        if !probe(port.as_mut(), serial_timeout(&params, 1)) {
            return Ok(Vec::new());
        }
        Ok(vec![Box::new(ZketechEbdUsbDevice::new(conn, params))])
    }
}

/// The middle of `scan`: any frame of more than two bytes will do, and the stop
/// goes out whatever came. The commands' failures are C's to drop as well.
fn probe(port: &mut dyn Port, byte_timeout: Duration) -> bool {
    let _ = send_command(port, &protocol::init());
    let message =
        protocol::read_message(&mut |buf| read_blocking(port, buf, byte_timeout), &|| false);
    let found = match message {
        Ok(Message::Frame(..)) => true,
        Ok(Message::Nothing) => {
            log::warn!("No message received!");
            false
        }
        Ok(Message::Bad | Message::Stopped) | Err(_) => {
            log::warn!("Could not receive message!");
            false
        }
    };
    let _ = send_command(port, &protocol::stop());
    found
}

/// `send_cmd`: a byte at a time, each followed by a pause, stopping at the first
/// that fails.
fn send_command(port: &mut dyn Port, command: &[u8]) -> Result<()> {
    if log::log_enabled!(log::Level::Debug) {
        let text = command.iter().fold(String::new(), |mut s, b| {
            let _ = write!(s, "{b:02X}");
            s
        });
        log::debug!("Sending: {text} [{} bytes]", command.len());
    }
    for byte in command {
        port.write(std::slice::from_ref(byte)).inspect_err(|e| {
            log::error!("Error sending command: {e}.");
        })?;
        std::thread::sleep(BYTE_GAP);
    }
    Ok(())
}

/// What the capture and the frontend share: `dev_context`, and the port.
#[allow(clippy::struct_excessive_bools)]
struct Shared {
    port: Option<Box<dyn Port>>,
    current_limit: f32,
    uvc_threshold: f32,
    /// `running`: a measurement has come and the reads did not run out since.
    running: bool,
    /// `load_activated`: what the last measurement said.
    load_activated: bool,
    /// A start went out since the last measurement, so a load may be on
    /// that `load_activated` does not know of.
    start_unconfirmed: bool,
    /// Between the first thing a capture does and the last thing before End.
    capturing: bool,
    /// A setting no measurement has been compared with.
    unsent: bool,
}

impl Shared {
    fn port(&mut self) -> Result<&mut dyn Port> {
        match self.port.as_mut() {
            Some(port) => Ok(port.as_mut()),
            None => Err(Error::device("device is not open")),
        }
    }

    fn send(&mut self, command: &[u8]) -> Result<()> {
        send_command(self.port()?, command)
    }

    /// `ebd_loadstart`.
    fn load_start(&mut self) -> Result<()> {
        log::info!("Activating load");
        self.start_unconfirmed = true;
        self.send(&protocol::load_start(self.current_limit, self.uvc_threshold))?;
        log::debug!("current limit: {:.3}", self.current_limit);
        log::debug!("under-voltage threshold: {:.2}", self.uvc_threshold);
        Ok(())
    }

    /// `ebd_loadtoggle`.
    fn load_toggle(&mut self) -> Result<()> {
        log::info!("Toggling load");
        self.send(&protocol::toggle())
    }

    /// `send_cfg`.
    fn send_config(&mut self) -> Result<()> {
        self.send(&protocol::config(self.current_limit, self.uvc_threshold))
    }

    /// What `ebd_set_current_limit` and `ebd_set_uvc_threshold` do once the
    /// value is stored: they share these lines in C.
    fn apply(&mut self) -> Result<()> {
        if !self.running {
            log::debug!("Setting a value later.");
            self.unsent = true;
            return Ok(());
        }
        let no_current = protocol::current_is_0(self.current_limit);
        match (self.load_activated, no_current) {
            (true, true) => self.load_toggle(),
            (true, false) => self.send_config(),
            (false, true) => Ok(()),
            (false, false) => self.load_start(),
        }
    }

    /// The load's part of `ebd_receive_data`, which drops what the commands
    /// return. The values compare exactly, as in C: a limit off the grid is sent
    /// again for each frame.
    #[allow(clippy::float_cmp)]
    fn update_load(&mut self, m: &Measurement) {
        self.running = true;
        self.unsent = false;
        self.start_unconfirmed = false;
        self.load_activated = m.load_on;
        let (limit, threshold) = m.reported.unwrap_or((self.current_limit, self.uvc_threshold));
        let no_current = protocol::current_is_0(self.current_limit);
        // The errors have been logged, and a port that failed shows in the reads.
        let _ = if self.load_activated && no_current {
            self.load_toggle()
        } else if !self.load_activated && !no_current {
            self.load_start()
        } else if self.load_activated
            && (limit != self.current_limit || threshold != self.uvc_threshold)
        {
            log::debug!(
                "Adjusting limit from {limit:.3} A {threshold:.3} V to {:.3} A {:.3} V",
                self.current_limit,
                self.uvc_threshold
            );
            self.send_config()
        } else {
            Ok(())
        };
    }

    /// `dev_acquisition_stop`'s commands.
    fn stop_commands(&mut self) {
        if self.load_activated {
            let _ = self.load_toggle();
        } else if self.start_unconfirmed {
            log::warn!(
                "The capture ended before the device reported the load on, so it was not turned \
                 off: it may still be on."
            );
        }
        let _ = self.send(&protocol::stop());
    }
}

/// The one load.
pub struct ZketechEbdUsbDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_samples: u64,
    /// What one read of a byte waits: `serial_timeout(serial, 1)`, but for tests.
    byte_timeout: Duration,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    shared: Arc<Mutex<Shared>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

/// A value in `range`, in the single precision C keeps it in. A NaN is refused,
/// where it passes C's two comparisons. A -0.0 stays, as `--get` shows it in C.
fn in_range(key: &Key, value: &Value, range: &[f64; 3]) -> Result<f32> {
    let bad = || Error::Arg(format!("{} out of range", key.id()));
    let v = value.as_f64().ok_or_else(bad)? as f32;
    if (range[0]..=range[1]).contains(&f64::from(v)) { Ok(v) } else { Err(bad()) }
}

impl ZketechEbdUsbDevice {
    fn new(conn: String, params: Params) -> Self {
        ZketechEbdUsbDevice {
            info: DeviceInfo {
                driver: "zketech-ebd-usb",
                vendor: "ZKETECH".to_owned(),
                model: "EBD-USB".to_owned(),
                version: String::new(),
                serial: String::new(),
                // `conn` is not in `devopts`, so sigrok-cli names none.
                conn: None,
            },
            channels: CHANNELS
                .iter()
                .enumerate()
                .map(|(i, name)| Channel::new(i, ChannelKind::Analog, *name))
                .collect(),
            conn,
            params,
            limit_samples: 0,
            byte_timeout: serial_timeout(&params, 1),
            pending: None,
            shared: Arc::new(Mutex::new(Shared {
                port: None,
                current_limit: 0.0,
                uvc_threshold: 0.0,
                running: false,
                load_activated: false,
                start_unconfirmed: false,
                capturing: false,
                unsent: false,
            })),
            worker: None,
        }
    }

    /// A device on a port that stands in for the hardware, with no scan.
    /// [`Device::open`] takes it.
    pub fn from_port(port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let mut device = Self::new(port.name().to_owned(), params);
        device.pending = Some(port);
        device
    }

    fn shared(&self) -> MutexGuard<'_, Shared> {
        self.shared.lock().expect("state lock")
    }
}

impl Device for ZketechEbdUsbDevice {
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
        no_group(group)?;
        Ok(DEV_OPTIONS.to_vec())
    }

    /// What was set: the device is not asked, as in C.
    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        let s = self.shared();
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::CurrentLimit => Value::F64(f64::from(s.current_limit)),
            Key::UnderVoltageConditionThreshold => Value::F64(f64::from(s.uvc_threshold)),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// Stored, and sent at once only after a measurement has come. The commands
    /// then go out on this thread, between two of a capture's reads.
    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        match key {
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().ok_or_else(|| Error::arg("limit_samples"))?;
                Ok(())
            }
            Key::CurrentLimit => {
                let v = in_range(key, value, &LIMIT)?;
                let mut s = self.shared();
                s.current_limit = v;
                s.apply()
            }
            Key::UnderVoltageConditionThreshold => {
                let v = in_range(key, value, &THRESHOLD)?;
                let mut s = self.shared();
                s.uvc_threshold = v;
                s.apply()
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        let range = match key {
            Key::CurrentLimit => LIMIT,
            Key::UnderVoltageConditionThreshold => THRESHOLD,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        Ok(ValueList::RangeStepF64 { min: range[0], max: range[1], step: range[2] })
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let pending = self.pending.take();
        let params = self.params;
        let conn = self.conn.clone();
        let mut s = self.shared();
        if s.port.is_some() {
            return Ok(());
        }
        s.port = Some(match pending {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&conn, params)?,
        });
        Ok(())
    }

    /// `dev_close`: the port and nothing on the wire, so a setting no capture
    /// took is lost, which C does not say.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut s = self.shared();
        if std::mem::take(&mut s.unsent) {
            log::warn!(
                "current_limit and uvc_threshold reach the device in a capture only: nothing was \
                 sent to it."
            );
        }
        s.port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        {
            let mut s = self.shared();
            if s.capturing {
                return Err(Error::Busy);
            }
            if s.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            s.capturing = true;
        }
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let shared = Arc::clone(&self.shared);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capture = Capture {
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limit_samples: self.limit_samples,
            byte_timeout: self.byte_timeout,
        };
        let handle = std::thread::Builder::new()
            .name("zketech-ebd-usb acquisition".to_owned())
            .spawn(move || {
                run_capture(&capture, &shared, &sink, &stop2);
                shared.lock().expect("state lock").capturing = false;
                send_end(&sink, &stop2);
            })
            .map_err(|e| {
                self.shared().capturing = false;
                Error::Io(e)
            })?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. The thread
    /// sends what a stop sends.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// What a capture needs of the device.
struct Capture {
    enabled: Vec<bool>,
    limit_samples: u64,
    byte_timeout: Duration,
}

/// The End, which a client that goes on reading gets however long it takes, and
/// after a stop only if there is room within [`END_AFTER_STOP`].
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

/// One message, with the lock held for each read alone, for a setter or a getter.
fn read_message(capture: &Capture, shared: &Mutex<Shared>, stop: &AtomicBool) -> Result<Message> {
    let mut read = |buf: &mut [u8]| {
        let got = {
            let mut s = shared.lock().expect("state lock");
            read_blocking(s.port()?, buf, capture.byte_timeout)?
        };
        if got == 0 {
            std::thread::sleep(EMPTY_READ_PAUSE);
        }
        Ok(got)
    };
    protocol::read_message(&mut read, &|| stop.load(Ordering::SeqCst))
}

/// Sends a frame of what the channels that are on measured.
fn send_measurement(
    m: &Measurement,
    capture: &Capture,
    sink: &PacketSink,
    stop: &AtomicBool,
) -> bool {
    let go_on = || !stop.load(Ordering::SeqCst);
    let values = [
        (m.voltage, Mq::Voltage, Unit::Volt, 3),
        (m.current, Mq::Current, Unit::Ampere, 4),
        (m.voltage_dp, Mq::Voltage, Unit::Volt, 3),
        (m.voltage_dm, Mq::Voltage, Unit::Volt, 3),
    ];
    sink.send_while(Packet::FrameBegin, go_on)
        && values.into_iter().enumerate().filter(|(i, _)| capture.enabled[*i]).all(
            |(i, (value, mq, unit, digits))| {
                sink.send_while(
                    Packet::Analog(AnalogData {
                        channels: vec![i],
                        samples: vec![value],
                        mq,
                        flags: MqFlags::DC,
                        unit,
                        digits,
                    }),
                    go_on,
                )
            },
        )
        && sink.send_while(Packet::FrameEnd, go_on)
}

/// `dev_acquisition_start` and `ebd_receive_data`, up to what a stop sends. A
/// capture that a read or the device's silence ended sends nothing more: C has
/// no stop on those ways out either.
fn run_capture(capture: &Capture, shared: &Mutex<Shared>, sink: &PacketSink, stop: &AtomicBool) {
    if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
        return;
    }
    let _ = shared.lock().expect("state lock").send(&protocol::init());
    let mut samples = 0u64;
    loop {
        let (frame, len) = match read_message(capture, shared, stop) {
            Ok(Message::Frame(frame, len)) => (frame, len),
            Ok(Message::Stopped) => break,
            Ok(Message::Bad) => {
                log::error!("Can't receive messages");
                continue;
            }
            Ok(Message::Nothing) => {
                log::error!("No messages received");
                let mut s = shared.lock().expect("state lock");
                s.running = false;
                warn_left_on(&s);
                return;
            }
            Err(e) => {
                log::error!("Can't receive messages: {e}.");
                warn_left_on(&shared.lock().expect("state lock"));
                return;
            }
        };
        let m = match protocol::parse(&frame, len) {
            Parsed::Measurement(m) => m,
            Parsed::NotMeasurement => {
                log::info!("Not measurement message received");
                continue;
            }
            Parsed::BadChecksum => {
                log::warn!("Invalid checksum");
                continue;
            }
        };
        log::debug!("VBUS current {:.4} A", m.current);
        log::debug!("VBUS voltage {:.3} V", m.voltage);
        log::debug!("D+ voltage {:.3} V", m.voltage_dp);
        log::debug!("D- voltage {:.3} V", m.voltage_dm);
        if let Some((limit, threshold)) = m.reported {
            log::debug!("Current limit {limit:.3} A");
            log::debug!("UVC threshold {threshold:.3} V");
        }
        shared.lock().expect("state lock").update_load(&m);
        if !send_measurement(&m, capture, sink, stop) {
            break;
        }
        samples += 1;
        if capture.limit_samples > 0 && samples >= capture.limit_samples {
            break;
        }
    }
    shared.lock().expect("state lock").stop_commands();
}

/// What a capture with no stop command leaves behind.
fn warn_left_on(s: &Shared) {
    if s.load_activated || s.start_unconfirmed {
        log::warn!("The device stopped answering, with the load on: it may still be on.");
    }
}

#[cfg(test)]
mod tests;
