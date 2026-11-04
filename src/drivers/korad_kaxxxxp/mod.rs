// Counterpart in sigrok:
//   libsigrok/src/hardware/korad-kaxxxxp/api.c,
//   Copyright (C) 2015 Hannu Vuolasaho <vuokkosetae@gmail.com>,
//   Copyright (C) 2018-2019 Frank Stettner <frank-stettner@gmx.net>,
//   with commits by Gerhard Sittig, Uwe Hermann, Lars-Peter Clausen,
//   Axel Hinrichs, Christian Fruth, David Sastre Medina, Dewelde Anthony,
//   Elen Eisendle, Holger Müller, JD Steffen, Peter van der Perk,
//   Peter Åstrand, Thomas Andres, Tim Van Helleputte, Walter Goossens,
//   Wolfram Sang, motte
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Korad `KAxxxxP` power supplies and their badges (Tenma, Velleman, RND,
//! Stamos), one channel each, on a USB virtual COM port or RS232.
//!
//! `conn=` names the port, `serialcomm=` overrides 9600/8n1, and
//! `force_detect=` names an ID to use when the device's matches no model.
//! A scan asks `*IDN?` and reads every value, and sets nothing. Nor does
//! an open, a capture, `--get`, `--show` or a close: the only commands
//! that change the supply are those `set` sends, one for each setting.
//!
//! A capture asks `IOUT1?`, `VOUT1?` and `STATUS?` in turn, each 80 ms
//! after the last, and a sample counts at each `VOUT1?`. A setting during a
//! capture goes out between two requests, as the C's `rw_mutex` has it.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Link, Model, Setting, Target};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 3] = [Key::Conn, Key::Serialcomm, Key::ForceDetect];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
const SERIALCOMM: &str = "9600/8n1";
static DEV_OPTIONS: [OptionDesc; 12] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::Regulation, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, Access::GET_SET),
    OptionDesc::new(Key::OverVoltageProtectionEnabled, Access::GET_SET),
];

/// `KAXXXXP_POLL_INTERVAL_MS`: the session's source runs this long after
/// the last ran, from when it started, so it is the time from one request
/// to the next unless a reply takes longer.
const POLL_INTERVAL: Duration = Duration::from_millis(80);
/// How often a wait between two requests looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);

/// Channel 0 and 1, `sr_channel_new`'s order in `scan`.
const CHANNEL_V: usize = 0;
const CHANNEL_I: usize = 1;

/// The `korad-kaxxxxp` driver.
pub struct KoradDriver;

impl Driver for KoradDriver {
    fn id(&self) -> &'static str {
        "korad-kaxxxxp"
    }

    fn long_name(&self) -> &'static str {
        "Korad KAxxxxP"
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
        let mut force_detect = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                Key::ForceDetect => {
                    force_detect = value.as_str().filter(|s| !s.is_empty()).map(str::to_owned);
                }
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        // Every failure of the port here is no device, as `scan` returns
        // NULL for each.
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        match probe(port, params, force_detect.as_deref()) {
            Ok(Some(found)) => Ok(vec![Box::new(KoradDevice::new(found, conn, params))]),
            Ok(None) => Ok(Vec::new()),
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// What a scan found.
struct Found {
    link: Link,
    serial: String,
}

/// `scan` after the open: the ID, the model, then all the values. The port
/// goes when this returns, as the scan's `serial_close`; `dev_open` opens
/// it again.
fn probe(port: Box<dyn Port>, params: Params, force_detect: Option<&str>) -> Result<Option<Found>> {
    // Which model does not matter to the asking; the ID's timeouts are
    // the line's.
    let mut link = Link::new(Some(port), params, &protocol::MODELS[0]);
    let reply = link.ask_id()?;
    let text = protocol::c_string(&reply);
    log::debug!("Received: {}, {}", reply.len(), String::from_utf8_lossy(text));
    let (id, serial) = protocol::split_serial(text);
    let id_shown = String::from_utf8_lossy(id);
    let mut model = protocol::lookup(id);
    if let (None, Some(forced)) = (model, force_detect) {
        log::warn!("Could not find model ID '{id_shown}', trying '{forced}'.");
        model = protocol::lookup(forced.as_bytes());
        if model.is_some() {
            log::info!("Found replacement, using it instead.");
        }
    }
    let Some(model) = model else {
        log::error!("Unsupported model ID '{id_shown}', aborting.");
        return Ok(None);
    };
    log::debug!("Found: {} {}.", model.vendor, model.name);
    link.model = model;
    link.get_all_values()?;
    link.port = None;
    let serial = serial.map(|s| String::from_utf8_lossy(s).into_owned()).unwrap_or_default();
    Ok(Some(Found { link, serial }))
}

/// One power supply.
pub struct KoradDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests and replays.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl KoradDevice {
    fn new(found: Found, conn: String, params: Params) -> Self {
        let model = found.link.model;
        KoradDevice {
            info: DeviceInfo {
                driver: "korad-kaxxxxp",
                vendor: model.vendor.to_owned(),
                model: model.name.to_owned(),
                version: String::new(),
                serial: found.serial,
                conn: Some(conn.clone()),
            },
            channels: vec![
                Channel::new(CHANNEL_V, ChannelKind::Analog, "V".to_owned()),
                Channel::new(CHANNEL_I, ChannelKind::Analog, "I".to_owned()),
            ],
            conn,
            params,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            link: Arc::new(Mutex::new(found.link)),
            worker: None,
        }
    }

    /// A device of `model` on a port that stands in for the hardware,
    /// without a scan's reading. [`Device::open`] takes it.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let link = Link::new(None, params, model);
        let found = Found { link, serial: String::new() };
        let mut device = Self::new(found, conn, params);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    /// A value `api.c`'s `config_get` asks the device for. It ignores how
    /// that went and answers what it has.
    fn read(&self, target: Target, value: fn(&Link) -> Value) -> Value {
        let mut link = self.link();
        if let Err(e) = link.get_value(target) {
            log::debug!("Reading {target:?}: {e}.");
        }
        value(&link)
    }

    fn apply(&self, setting: Setting) -> Result<()> {
        self.link().set_value(setting)
    }
}

/// A value within `range`. A NaN passes `api.c`'s two comparisons and goes
/// out as `VSET1:  nan` to a live output; here it is refused.
fn in_range(key: &Key, value: &Value, range: &protocol::Range) -> Result<f64> {
    match value.as_f64() {
        Some(v) if (range[0]..=range[1]).contains(&v) => Ok(v),
        _ => Err(Error::Arg(format!("{} out of range", key.id()))),
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for KoradDevice {
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
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            Key::Conn => Value::Str(self.conn.clone()),
            Key::Voltage => self.read(Target::Voltage, |l| Value::F64(f64::from(l.voltage))),
            Key::VoltageTarget => {
                self.read(Target::VoltageTarget, |l| Value::F64(f64::from(l.voltage_target)))
            }
            Key::Current => self.read(Target::Current, |l| Value::F64(f64::from(l.current))),
            Key::CurrentLimit => {
                self.read(Target::CurrentLimit, |l| Value::F64(f64::from(l.current_limit)))
            }
            Key::Enabled => self.read(Target::Status, |l| Value::Bool(l.output_enabled())),
            Key::Regulation => {
                self.read(Target::Status, |l| Value::from(if l.cc_mode[0] { "CC" } else { "CV" }))
            }
            Key::OverCurrentProtectionEnabled => {
                self.read(Target::Status, |l| Value::Bool(l.ocp_enabled()))
            }
            Key::OverVoltageProtectionEnabled => {
                self.read(Target::Status, |l| Value::Bool(l.ovp_enabled()))
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let model = self.link().model;
        match key {
            // sr_config_set() refuses a 0 ahead of the driver (hwdriver.c,
            // check_key()).
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::VoltageTarget => {
                self.apply(Setting::VoltageTarget(in_range(key, value, &model.voltage)?))?;
            }
            Key::CurrentLimit => {
                self.apply(Setting::CurrentLimit(in_range(key, value, &model.current)?))?;
            }
            Key::Enabled => self.apply(Setting::Output(value.as_bool().ok_or_else(bad)?))?,
            Key::OverCurrentProtectionEnabled => {
                self.apply(Setting::Ocp(value.as_bool().ok_or_else(bad)?))?;
            }
            Key::OverVoltageProtectionEnabled => {
                self.apply(Setting::Ovp(value.as_bool().ok_or_else(bad)?))?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        let model = self.link().model;
        let range = match key {
            Key::VoltageTarget => model.voltage,
            Key::CurrentLimit => model.current,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        };
        Ok(ValueList::RangeStepF64 { min: range[0], max: range[1], step: range[2] })
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(mut port) => {
                port.purge()?;
                port
            }
            None => crate::serial::open(&self.conn, self.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        self.link().port = None;
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if let Some(worker) = &self.worker {
            if !worker.handle.is_finished() {
                return Err(Error::Busy);
            }
            let worker = self.worker.take().expect("just seen");
            let _ = worker.handle.join();
        }
        {
            let mut link = self.link();
            if link.port.is_none() {
                return Err(Error::device("device is not open"));
            }
            link.forget_pace();
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("korad-kaxxxxp acquisition".to_owned())
            .spawn(move || {
                capture.run(&sink, &stop2);
                sink.send(Packet::End);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    /// Asks the thread to end, and does not wait: it may be sending. It
    /// sends nothing to the device for this, as `std_serial_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// One capture, on its own thread: `korad_kaxxxxp_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        sink.send(Packet::header());
        let started = Instant::now();
        let limit = Duration::from_millis(self.limit_msec);
        let mut samples = 0u64;
        let mut due = started + POLL_INTERVAL;
        loop {
            if !wait_until(due, stop) {
                return;
            }
            due = Instant::now() + POLL_INTERVAL;
            match self.step(sink) {
                Step::Sample => samples += 1,
                Step::Other => {}
                Step::End => return,
            }
            // Checked after each step, as `receive_data` does.
            if self.limit_samples > 0 && samples >= self.limit_samples {
                return;
            }
            if self.limit_msec > 0 && started.elapsed() > limit {
                return;
            }
        }
    }

    /// One request and what it gives. C goes on after a port that failed
    /// and repeats the last values at full speed until a limit, for ever
    /// without one (`protocol.c:452`, its result is dropped): here it ends
    /// the capture.
    fn step(&self, sink: &PacketSink) -> Step {
        let mut link = self.link.lock().expect("link lock");
        let target = link.acquisition_target;
        if let Err(e) = link.get_value(target) {
            log::error!("Acquisition failed: {e}.");
            return Step::End;
        }
        let mut step = Step::Other;
        let sent = match target {
            Target::Current => {
                sink.send(analog(CHANNEL_I, link.current, Mq::Current, Unit::Ampere, 3))
            }
            Target::Voltage => {
                step = Step::Sample;
                sink.send(analog(CHANNEL_V, link.voltage, Mq::Voltage, Unit::Volt, 2))
            }
            _ => send_changes(&mut link, sink),
        };
        link.next_measurement();
        if sent { step } else { Step::End }
    }
}

enum Step {
    Sample,
    Other,
    End,
}

/// Waits for `due`: false when a stop came first.
fn wait_until(due: Instant, stop: &AtomicBool) -> bool {
    loop {
        if stop.load(Ordering::SeqCst) {
            return false;
        }
        let left = due.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return true;
        }
        std::thread::sleep(left.min(STOP_CHECK));
    }
}

fn analog(channel: usize, value: f32, mq: Mq, unit: Unit, digits: i32) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value],
        mq,
        flags: MqFlags::DC,
        unit,
        digits,
    })
}

/// The status's changes as metadata, each at most once. Channel 2's
/// regulation goes under the same key as channel 1's, as in C.
fn send_changes(link: &mut Link, sink: &PacketSink) -> bool {
    let regulation = |cc: bool| Packet::meta(Key::Regulation, if cc { "CC" } else { "CV" });
    let mut packets = Vec::new();
    if std::mem::take(&mut link.cc_mode_1_changed) {
        packets.push(regulation(link.cc_mode[0]));
    }
    if std::mem::take(&mut link.cc_mode_2_changed) {
        packets.push(regulation(link.cc_mode[1]));
    }
    if std::mem::take(&mut link.output_enabled_changed) {
        packets.push(Packet::meta(Key::Enabled, link.output_enabled()));
    }
    if std::mem::take(&mut link.ocp_enabled_changed) {
        packets.push(Packet::meta(Key::OverCurrentProtectionEnabled, link.ocp_enabled()));
    }
    if std::mem::take(&mut link.ovp_enabled_changed) {
        packets.push(Packet::meta(Key::OverVoltageProtectionEnabled, link.ovp_enabled()));
    }
    packets.into_iter().all(|p| sink.send(p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// What the fake saw.
    #[derive(Default)]
    struct Seen {
        writes: Vec<(Instant, Vec<u8>)>,
        purges: usize,
    }

    /// A supply that answers as the protocol says, the sixth byte
    /// included, and notes each write and when.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<u8>,
        status: u8,
        fail_writes_after: Option<usize>,
    }

    impl Fake {
        fn new() -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                status: 0x51,
                fail_writes_after: None,
            };
            (fake, seen)
        }
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
            if self.input.is_empty() {
                std::thread::sleep(timeout);
                return Ok(0);
            }
            let n = buf.len().min(self.input.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut seen = self.seen.lock().unwrap();
            if self.fail_writes_after.is_some_and(|n| seen.writes.len() >= n) {
                return Err(Error::device("Broken pipe"));
            }
            seen.writes.push((Instant::now(), data.to_vec()));
            let answer: &[u8] = match data {
                b"*IDN?" => b"KORADKA3005PV2.0",
                b"IOUT1?" => b"0.500",
                b"ISET1?" => b"1.000K",
                b"VOUT1?" | b"VSET1?" => b"05.00",
                b"STATUS?" => std::slice::from_ref(&self.status),
                _ => b"",
            };
            self.input.extend(answer);
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.seen.lock().unwrap().purges += 1;
            self.input.clear();
            Ok(())
        }
    }

    fn model(id: &[u8]) -> &'static Model {
        protocol::lookup(id).unwrap()
    }

    fn device(fake: Fake, id: &[u8]) -> KoradDevice {
        let mut d = KoradDevice::from_port(model(id), Box::new(fake));
        d.open().unwrap();
        d
    }

    fn writes(seen: &Arc<Mutex<Seen>>) -> Vec<String> {
        let seen = seen.lock().unwrap();
        seen.writes.iter().map(|(_, w)| String::from_utf8_lossy(w).into_owned()).collect()
    }

    /// Runs a capture to its end and gives its packets.
    fn capture(d: &mut KoradDevice) -> Vec<Packet> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        let mut out = Vec::new();
        while let Ok((_, p)) = rx.recv() {
            let end = matches!(p, Packet::End);
            out.push(p);
            if end {
                break;
            }
        }
        out
    }

    #[test]
    fn a_scan_asks_and_sets_nothing() {
        let (fake, seen) = Fake::new();
        let found = probe(Box::new(fake), Params::parse(SERIALCOMM).unwrap(), None).unwrap();
        let found = found.expect("a model");
        assert_eq!(found.link.model.name, "KA3005P");
        assert_eq!(writes(&seen), ["*IDN?", "IOUT1?", "ISET1?", "VOUT1?", "VSET1?", "STATUS?"]);
        assert!(found.link.port.is_none(), "closed, as the C's scan");
        // The sixth byte was dropped, or VSET1? and all after it slip.
        assert_eq!(found.link.voltage, 5.0);
        assert_eq!(found.link.voltage_target, 5.0);
        assert_eq!(found.link.current_limit, 1.0);
        assert!(found.link.output_enabled());
    }

    #[test]
    fn requests_wait_80_ms_from_the_last() {
        let (fake, seen) = Fake::new();
        probe(Box::new(fake), Params::parse(SERIALCOMM).unwrap(), None).unwrap().unwrap();
        let seen = seen.lock().unwrap();
        // *IDN? sets no pace: IOUT1? may follow it at once.
        for pair in seen.writes[1..].windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!(gap >= Duration::from_millis(80), "{gap:?}");
        }
    }

    #[test]
    fn the_slow_model_waits_530_ms_after_a_status_and_a_setting() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"VELLEMANPS3005DV1.3");
        d.get(&Key::Enabled, None).unwrap();
        d.get(&Key::Voltage, None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        d.get(&Key::Voltage, None).unwrap();
        d.get(&Key::Voltage, None).unwrap();
        let seen = seen.lock().unwrap();
        let gap = |i: usize| seen.writes[i + 1].0 - seen.writes[i].0;
        assert!(gap(0) >= Duration::from_millis(530), "status: {:?}", gap(0));
        assert!(gap(2) >= Duration::from_millis(530), "setting: {:?}", gap(2));
        assert!(gap(3) >= Duration::from_millis(80), "{:?}", gap(3));
        assert!(gap(3) < Duration::from_millis(530), "a reading is not long: {:?}", gap(3));
    }

    #[test]
    fn open_and_close_send_nothing() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        d.close().unwrap();
        assert_eq!(writes(&seen), Vec::<String>::new());
        assert_eq!(seen.lock().unwrap().purges, 1, "flushed once, as serial_open");
    }

    #[test]
    fn samples_count_at_the_voltage() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let packets = capture(&mut d);
        d.close().unwrap();
        assert_eq!(
            writes(&seen),
            ["IOUT1?", "VOUT1?", "STATUS?", "IOUT1?", "VOUT1?"],
            "and nothing at the end"
        );
        let analog: Vec<(usize, f32)> = packets
            .iter()
            .filter_map(|p| match p {
                Packet::Analog(a) => Some((a.channels[0], a.samples[0])),
                _ => None,
            })
            .collect();
        assert_eq!(analog, [(1, 0.5), (0, 5.0), (1, 0.5), (0, 5.0)]);
        assert!(matches!(packets[0], Packet::Header { .. }));
    }

    #[test]
    fn a_time_limit_ends_the_capture() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        d.set(&Key::LimitMsec, &Value::U64(200), None).unwrap();
        let begun = Instant::now();
        capture(&mut d);
        d.close().unwrap();
        assert!(begun.elapsed() < Duration::from_secs(2), "{:?}", begun.elapsed());
        let n = writes(&seen).len();
        assert!((1..=4).contains(&n), "{n} requests");
    }

    #[test]
    fn a_stop_ends_the_capture_while_the_device_still_answers() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        let (tx, rx) = std::sync::mpsc::sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        // The first reading came, so data is on offer when it stops.
        while !matches!(rx.recv().unwrap().1, Packet::Analog(_)) {}
        d.stop().unwrap();
        let before = writes(&seen).len();
        let begun = Instant::now();
        while !matches!(rx.recv().unwrap().1, Packet::End) {}
        assert!(begun.elapsed() < Duration::from_millis(500), "{:?}", begun.elapsed());
        d.close().unwrap();
        assert!(writes(&seen).len() <= before + 1, "at most the request under way");
    }

    #[test]
    fn a_setting_during_a_capture_goes_out_between_requests() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        let (tx, rx) = std::sync::mpsc::sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        while !matches!(rx.recv().unwrap().1, Packet::Analog(_)) {}
        d.set(&Key::VoltageTarget, &Value::F64(3.3), None).unwrap();
        d.stop().unwrap();
        while !matches!(rx.recv().unwrap().1, Packet::End) {}
        d.close().unwrap();
        assert!(writes(&seen).contains(&"VSET1:03.30".to_owned()), "{:?}", writes(&seen));
    }

    #[test]
    fn a_failed_port_ends_the_capture_where_c_repeats_old_values() {
        let (mut fake, seen) = Fake::new();
        fake.fail_writes_after = Some(2);
        let mut d = device(fake, b"KORADKA3005PV2.0");
        d.set(&Key::LimitSamples, &Value::U64(5), None).unwrap();
        let packets = capture(&mut d);
        let analog = packets.iter().filter(|p| matches!(p, Packet::Analog(_))).count();
        assert_eq!(analog, 2, "no reading from a device that left");
        assert_eq!(writes(&seen).len(), 2);
    }

    #[test]
    fn a_changed_status_is_metadata_once() {
        let (fake, _seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let metas: Vec<(Key, Value)> = capture(&mut d)
            .into_iter()
            .filter_map(|p| match p {
                Packet::Meta(items) => Some(items),
                _ => None,
            })
            .flatten()
            .collect();
        // From all clear: CH1 stays CV (bit 0 set is not CC), CH2 reads CC,
        // the output is on. Then nothing changes.
        assert_eq!(
            metas,
            [(Key::Regulation, Value::from("CC")), (Key::Enabled, Value::Bool(true))]
        );
    }

    #[test]
    fn each_setting_is_its_command_and_ranges_bind() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"TENMA 72-2535 V2.1");
        let set = |d: &mut KoradDevice, k: Key, v: Value| d.set(&k, &v, None);
        set(&mut d, Key::VoltageTarget, Value::F64(31.0)).unwrap();
        set(&mut d, Key::CurrentLimit, Value::F64(3.1)).unwrap();
        set(&mut d, Key::Enabled, Value::Bool(true)).unwrap();
        set(&mut d, Key::OverCurrentProtectionEnabled, Value::Bool(false)).unwrap();
        set(&mut d, Key::OverVoltageProtectionEnabled, Value::Bool(true)).unwrap();
        for (k, v) in [
            (Key::VoltageTarget, 31.01),
            (Key::VoltageTarget, -0.01),
            (Key::VoltageTarget, f64::NAN),
            (Key::CurrentLimit, 3.11),
            (Key::CurrentLimit, f64::NAN),
        ] {
            assert!(set(&mut d, k, Value::F64(v)).is_err(), "{v}");
        }
        assert_eq!(writes(&seen), ["VSET1:31.00", "ISET1:3.100", "OUT1", "OCP0", "OVP1"]);
    }

    #[test]
    fn a_labps_ignores_bit_7_with_the_output_off() {
        let (mut fake, _seen) = Fake::new();
        fake.status = 0x80;
        let d = device(fake, b"VELLEMANLABPS3005DV2.0");
        assert_eq!(d.get(&Key::OverVoltageProtectionEnabled, None).unwrap(), Value::Bool(false));
        let (mut fake, _seen) = Fake::new();
        fake.status = 0xc0;
        let d = device(fake, b"VELLEMANLABPS3005DV2.0");
        assert_eq!(d.get(&Key::OverVoltageProtectionEnabled, None).unwrap(), Value::Bool(true));
        let (mut fake, _seen) = Fake::new();
        fake.status = 0x80;
        let d = device(fake, b"VELLEMANPS3005DV2.0");
        assert_eq!(d.get(&Key::OverVoltageProtectionEnabled, None).unwrap(), Value::Bool(true));
    }

    #[test]
    fn a_closed_port_answers_what_is_known_as_the_c_does() {
        let (fake, seen) = Fake::new();
        let mut d = device(fake, b"KORADKA3005PV2.0");
        d.close().unwrap();
        assert_eq!(d.get(&Key::Voltage, None).unwrap(), Value::F64(0.0));
        assert_eq!(writes(&seen), Vec::<String>::new());
    }
}
