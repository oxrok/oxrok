// Counterpart in sigrok:
//   libsigrok/src/hardware/atorch/api.c,
//   libsigrok/src/hardware/atorch/protocol.c,
//   Copyright (C) 2023 Mathieu Pilato <pilato.mathieu@free.fr>,
//   with commits by Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Atorch meters and loads (`DC Meter`: the DL24 family; `USB Meter`: the J7-C,
//! UD18 and others), which report about once a second, unasked. `conn=` names a
//! tty. That is an RFCOMM tty for a unit on Bluetooth SPP (a capture of a UD18;
//! `serial_bt.c:85,89` names DL24M and UC96 `_SPP` units, on channel 2, which an
//! `rfcomm bind` must then be told). libsigrok's `bt/...` names are refused by
//! `serial::open`, as `src/serial` has no Bluetooth transport, so a BLE unit (`*_BLE`,
//! `ac6328` and the like) has no tty and no path here.
//!
//! A scan waits up to 10 s for a report and goes by the type in it, and nothing
//! is ever written to a unit. A capture gives a frame per report, with a sample
//! on each channel that is enabled.
//!
//! Where this differs from C:
//!
//! - C looks for a message only after a read that brought bytes
//!   (`protocol.c:151-157`, as `len <= 0` returns first), so what a read holds
//!   beyond its first message waits for the next byte. A second report of a read
//!   comes only with the bytes after it, or never, when none come. Here a read's
//!   messages are all taken at once. So `--time 1500` with two reports per read
//!   gives 2 frames in C and 3 here. A reply and a report in one read find no
//!   device in C only if nothing follows them within the probe's time, as a real
//!   unit's next report would.
//! - The stop in `parse_report_msg` (`protocol.c:192-193`) leaves the callback's
//!   loop running (`protocol.c:239`). With 20 reports in one write on a pty and
//!   `--frames 1`, C goes on with frames and a repeated `SR_DF_END` after the first.
//!   sigrok-cli hides that, and a library client would get it. Here the limit is
//!   the end.
//! - A scan's read can leave a report in `buf`, which only a probe's start clears
//!   (`protocol.c:202-203`), so that it is a capture's first frame, from before
//!   the capture. Here each capture starts empty. On a second capture of an open
//!   device what the port has gathered in the meantime goes as well, a choice with
//!   no C run behind it, as sigrok-cli makes one capture: it would be old
//!   readings, given as new.
//! - C reads a limit only when a report has come (`protocol.c:192`), and it never
//!   calls `serial_timeout`, so a unit gone quiet holds a capture: runs passed
//!   100 s on the emulator and 120 s on a pty. Here a time limit is kept as C keeps
//!   it while reports come, the first past the limit being the last, and with none
//!   it ends [`Pace::grace`] after the limit with a warning. A frame limit alone
//!   ends after [`Pace::silence`] with an error. With neither limit the user ends
//!   it, as in C.
//! - After an unplug a read is EOF in C, which it takes for no data, so it never
//!   ends. Here a read that fails ends the capture, and a probe at once. A real
//!   read error in C lands in a `size_t` (`protocol.c:137`, 151), passes `len <= 0`
//!   and moves `wr_idx` back: with EIO injected on a pty it logged `Read error (5)`,
//!   sent a stale frame again and died of a SIGSEGV.
//! - `Unrecognized device type` prints `buf[3]` where the report's fourth byte is
//!   meant (`protocol.c:216`), so with anything ahead of the report in its read it
//!   names another byte. Here it names the report's.
//! - `parse_report_msg` sends every channel whatever `-C` said (`protocol.c:184`).
//!   Here only those enabled.
//!
//! The sum at a message's end goes unchecked, as in C, and so a false magic in
//! the stream takes a report's length with it: some units fail the sum, says
//! `esphome-atorch-dl24`'s README, and the DL24M log in its issue 59 has last bytes
//! that look random. Temperature is unsigned, as in C, so a reading below 0 would
//! show about 65535 °C; `NiceLabs/atorch-console` reads the USB meter's as signed.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{self, Params, Port};
use protocol::{Framer, Message, Profile, Report};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 3] =
    [DeviceClass::EnergyMeter, DeviceClass::PowerMeter, DeviceClass::ElectronicLoad];
/// `scan`'s default.
const SERIALCOMM: &str = "9600/8n1";
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
];
/// How long a stopped capture tries to get its End to a queue no one may read.
const END_AFTER_STOP: Duration = Duration::from_secs(1);

/// The times of a scan and a capture; tests take shorter ones.
#[derive(Clone, Copy, Debug)]
pub struct Pace {
    /// `ATORCH_PROBE_TIMEOUT_MS`.
    pub probe: Duration,
    /// How long a read waits, so how often a limit and a stop are looked at. The
    /// probe's 100 ms pause and the session source's 100 ms are C's; this
    /// answers a stop sooner and is not seen on the wire.
    pub wait: Duration,
    /// How long past a time limit a capture waits for the report that would end
    /// it in C: 3 s, three of a unit's reports. A capture of a UD18
    /// (`NiceLabs/atorch-console`, `docs/UD18-SPP.pcap`) has them 0.987 to 1.039 s
    /// apart. A choice, in C for ever.
    pub grace: Duration,
    /// How long a capture only a frame limit can end waits for a report: 30 s, as
    /// `rdtech-um` waits on a Bluetooth link. A choice, in C for ever.
    pub silence: Duration,
}

impl Default for Pace {
    fn default() -> Pace {
        Pace {
            probe: Duration::from_secs(10),
            wait: Duration::from_millis(50),
            grace: Duration::from_secs(3),
            silence: Duration::from_secs(30),
        }
    }
}

/// The `atorch` driver.
pub struct AtorchDriver;

impl Driver for AtorchDriver {
    fn id(&self) -> &'static str {
        "atorch"
    }

    fn long_name(&self) -> &'static str {
        "atorch meters and loads"
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
        let Some(conn) = conn.filter(|c| !c.is_empty()) else {
            return Ok(Vec::new());
        };
        // Every failure of the port here is no device, as `atorch_scan` returns
        // NULL for each.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((serial::open(&conn, params)?, params)));
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let profile = match probe(port.as_mut(), &Pace::default()) {
            Ok(profile) => profile,
            Err(end) => {
                if let ProbeEnd::UnknownType(device_type) = end {
                    log::error!("{}", unknown_type_message(device_type));
                }
                log::error!("Failed to find a supported Atorch device.");
                return Ok(Vec::new());
            }
        };
        // The scan's close: `dev_open` opens it again.
        drop(port);
        Ok(vec![Box::new(AtorchDevice::new(profile, conn, params))])
    }
}

/// Why a probe found no unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ProbeEnd {
    /// The first report has this type, which nobody knows.
    UnknownType(u8),
    /// No report came, or the port failed.
    NoReport,
}

/// What `atorch_probe` logs of an unknown type, the report's own byte.
fn unknown_type_message(device_type: u8) -> String {
    format!("Unrecognized device type ({device_type:#06x}).")
}

/// `atorch_probe`: the first report decides.
fn probe(port: &mut dyn Port, pace: &Pace) -> std::result::Result<&'static Profile, ProbeEnd> {
    let mut framer = Framer::default();
    let started = Instant::now();
    while started.elapsed() <= pace.probe {
        let got = match port.read(framer.room(), pace.wait) {
            Ok(got) => got,
            Err(e) => {
                log::debug!("{e}");
                return Err(ProbeEnd::NoReport);
            }
        };
        framer.filled(got);
        while let Some(message) = framer.next_message() {
            let Message::Report(report) = message else { continue };
            let device_type = report.device_type();
            return protocol::find_profile(device_type).ok_or(ProbeEnd::UnknownType(device_type));
        }
    }
    Err(ProbeEnd::NoReport)
}

/// One unit.
pub struct AtorchDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    profile: &'static Profile,
    conn: String,
    params: Params,
    limits: Arc<Limits>,
    pace: Pace,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    /// The open port, when no capture holds it: a capture gives it back ahead
    /// of its End, so a client can start again as soon as it sees that.
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    /// A capture has run on this open.
    captured: bool,
    worker: Option<Worker>,
}

/// `sr_sw_limits`, which a capture reads each time as C does, so a set during a
/// capture counts. 0 is no limit.
#[derive(Debug, Default)]
struct Limits {
    frames: AtomicU64,
    msec: AtomicU64,
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

impl AtorchDevice {
    fn new(profile: &'static Profile, conn: String, params: Params) -> Self {
        let channels = profile
            .channels
            .iter()
            .enumerate()
            .map(|(i, c)| Channel::new(i, ChannelKind::Analog, c.name.to_owned()))
            .collect();
        AtorchDevice {
            info: DeviceInfo {
                driver: "atorch",
                vendor: "Atorch".to_owned(),
                model: profile.name.to_owned(),
                version: String::new(),
                serial: String::new(),
                // `conn` is not in `devopts`, so sigrok-cli names none.
                conn: None,
            },
            channels,
            profile,
            conn,
            params,
            limits: Arc::default(),
            pace: Pace::default(),
            pending: None,
            port: Arc::new(Mutex::new(None)),
            captured: false,
            worker: None,
        }
    }

    /// A unit of `profile`'s kind on a port that stands in for the hardware, with
    /// no scan. [`Device::open`] takes it.
    pub fn from_port(profile: &'static Profile, port: Box<dyn Port>, pace: Pace) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(profile, conn, params);
        device.pending = Some(port);
        device.pace = pace;
        device
    }
}

impl Device for AtorchDevice {
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

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        match key {
            Key::LimitFrames => Ok(Value::U64(self.limits.frames.load(Ordering::SeqCst))),
            Key::LimitMsec => Ok(Value::U64(self.limits.msec.load(Ordering::SeqCst))),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitFrames => {
                self.limits.frames.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
            }
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitMsec => {
                let msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
                self.limits.msec.store(msec, Ordering::SeqCst);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut slot = self.port.lock().expect("port lock");
        if slot.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => serial::open(&self.conn, self.params)?,
        };
        *slot = Some(port);
        self.captured = false;
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        *self.port.lock().expect("port lock") = None;
        Ok(())
    }

    /// Busy while a capture holds the port. One that gave it back has only its
    /// End to send: told to stop, it gives that up within [`END_AFTER_STOP`] if the
    /// old client's queue stays full, so this does not wait for ever.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        let Some(mut port) = self.port.lock().expect("port lock").take() else {
            return Err(match &self.worker {
                Some(_) => Error::Busy,
                None => Error::device("device is not open"),
            });
        };
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        if std::mem::replace(&mut self.captured, true) {
            // What came since the last capture is old by now.
            if let Err(e) = port.purge() {
                log::debug!("{e}");
            }
        }
        let capture = Capture {
            profile: self.profile,
            enabled: self.channels.iter().map(|c| c.enabled).collect(),
            limits: Arc::clone(&self.limits),
            pace: self.pace,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let slot = Arc::clone(&self.port);
        let handle = std::thread::Builder::new()
            .name("atorch acquisition".to_owned())
            .spawn(move || {
                let mut port = port;
                capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("port lock") = Some(port);
                send_end(&sink, &stop2);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. Nothing
    /// goes to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// The End, which a client that goes on reading gets however long it takes, and
/// after a stop only if there is room within [`END_AFTER_STOP`].
fn send_end(sink: &PacketSink, stop: &AtomicBool) {
    let stopped_at = Cell::new(None);
    sink.send_while(Packet::End, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < END_AFTER_STOP
    });
}

/// Why a capture ends with no report to end it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Overdue {
    /// The time limit is past by [`Pace::grace`].
    Time,
    /// A frame limit is all that could end it, and [`Pace::silence`] has gone.
    Silence,
}

/// What the limits say with no report coming. A time limit reads from the capture's
/// start and silence from the last report.
fn overdue(
    pace: &Pace,
    frames: u64,
    msec: u64,
    running: Duration,
    since_report: Duration,
) -> Option<Overdue> {
    if msec > 0 {
        let end = Duration::from_millis(msec).saturating_add(pace.grace);
        return (running > end).then_some(Overdue::Time);
    }
    (frames > 0 && since_report >= pace.silence).then_some(Overdue::Silence)
}

/// One capture, on its own thread: `dev_acquisition_start` and
/// `atorch_receive_data_callback`.
struct Capture {
    profile: &'static Profile,
    enabled: Vec<bool>,
    limits: Arc<Limits>,
    pace: Pace,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) {
        let started = Instant::now();
        if !sink.send_while(Packet::header(), || !stop.load(Ordering::SeqCst)) {
            return;
        }
        let mut framer = Framer::default();
        let mut sent = 0u64;
        let mut last_report = started;
        loop {
            if stop.load(Ordering::SeqCst) {
                return;
            }
            let got = match port.read(framer.room(), self.pace.wait) {
                Ok(got) => got,
                Err(e) => {
                    log::error!("Reading the port failed: {e}.");
                    return;
                }
            };
            framer.filled(got);
            while let Some(message) = framer.next_message() {
                let Message::Report(report) = message else { continue };
                if stop.load(Ordering::SeqCst) || !self.send_frame(&report, sink, stop) {
                    return;
                }
                sent += 1;
                last_report = Instant::now();
                if self.limit_reached(sent, started) {
                    return;
                }
            }
            let frame_limit = self.limits.frames.load(Ordering::SeqCst);
            let msec = self.limits.msec.load(Ordering::SeqCst);
            match overdue(&self.pace, frame_limit, msec, started.elapsed(), last_report.elapsed()) {
                Some(Overdue::Time) => {
                    log::warn!("No report came after the time limit.");
                    return;
                }
                Some(Overdue::Silence) => {
                    log::error!("No report from the device for {} s.", self.pace.silence.as_secs());
                    return;
                }
                None => {}
            }
        }
    }

    /// `sr_sw_limits_check`, after a report.
    fn limit_reached(&self, frames: u64, started: Instant) -> bool {
        let frame_limit = self.limits.frames.load(Ordering::SeqCst);
        let msec = self.limits.msec.load(Ordering::SeqCst);
        (frame_limit > 0 && frames >= frame_limit)
            || (msec > 0 && started.elapsed() > Duration::from_millis(msec))
    }

    /// `parse_report_msg`'s frame: the channels that are on, in order.
    fn send_frame(&self, report: &Report, sink: &PacketSink, stop: &AtomicBool) -> bool {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::FrameBegin, go_on) {
            return false;
        }
        let on = self.profile.channels.iter().enumerate().filter(|(i, _)| self.enabled[*i]);
        for (i, channel) in on {
            let sent = sink.send_while(
                Packet::Analog(AnalogData {
                    channels: vec![i],
                    samples: vec![channel.value(report)],
                    mq: channel.mq,
                    flags: channel.flags,
                    unit: channel.unit,
                    digits: channel.digits,
                }),
                go_on,
            );
            if !sent {
                return false;
            }
        }
        sink.send_while(Packet::FrameEnd, go_on)
    }
}

#[cfg(test)]
mod tests;
