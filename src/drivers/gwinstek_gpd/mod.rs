// Counterpart in sigrok:
//   libsigrok/src/hardware/gwinstek-gpd/api.c,
//   libsigrok/src/hardware/gwinstek-gpd/protocol.c,
//   Copyright (C) 2018 Bastian Schmitz <bastian.schmitz@udo.edu>,
//   with commits by Richard Allen, Timo Kokkonen, Uwe Hermann,
//   Andreas Sandberg, Gerhard Sittig, Wolfram Sang, Michał Janiszewski
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! GW Instek GPD-2303S and GPD-3303S power supplies, two channels each, on a
//! serial port (a USB converter on the real ones).
//!
//! `conn=` names the port and `serialcomm=` a line; without one a scan tries
//! 9600, 57600 and 115200, as the panel can set any of them. A scan asks
//! `*IDN?` up to three times, since junk in the device's buffer spoils the
//! first, then `STATUS?` and each channel's values, and sets nothing. The port
//! closes after the scan and an open opens it again. C leaves it open
//! (`api.c:255`, no `serial_close` before `std_scan_complete`), and its
//! `dev_open` then finds it locked: stock sigrok-cli runs only `--scan` here.
//!
//! `--get` and `--show` answer what the scan read and what was set since, and
//! put nothing on the wire. A setting is one line with no answer.
//!
//! A capture asks `IOUTn?` and `VOUTn?` of each channel every 100 ms after the
//! last answer, and a sample counts at each round. An answer that is no number,
//! or a round that does not come, ends the capture with the C's message, where
//! C sends no more queries and never ends under a sample limit. A round not
//! begun after [`REPLY_BOUND`] is such a one (`req_sent_at` is there in C, but
//! nothing reads it).
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CAPTURE_BUFFER, CHANNELS, Link, Model, SCAN_BUFFER};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::PowerSupply];
/// `gpd_serialcomms`, in the order they are tried.
const SERIALCOMMS: [&str; 3] = ["9600/8n1", "57600/8n1", "115200/8n1"];
/// `IDN_RETRIES`.
const IDN_RETRIES: usize = 3;
const CHANNEL_MODE: &str = "Independent";

static DEV_OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::ChannelConfig, Access::GET_SET_LIST),
    OptionDesc::new(Key::Enabled, Access::GET_SET),
];
static GROUP_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
];

/// The session source's timeout in `dev_acquisition_start`.
const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// How long a round's first byte may take. C waits for ever. No device says
/// what is right; a reply is 250 ms at most once begun, and a supply busy on
/// its panel should not end a capture.
pub const REPLY_BOUND: Duration = Duration::from_secs(2);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);

/// The `gwinstek-gpd` driver.
pub struct GpdDriver;

impl Driver for GpdDriver {
    fn id(&self) -> &'static str {
        "gwinstek-gpd"
    }

    fn long_name(&self) -> &'static str {
        "GW Instek GPD"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = None;
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
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let rates = match &serialcomm {
            Some(one) => vec![one.as_str()],
            None => SERIALCOMMS.to_vec(),
        };
        for rate in rates {
            log::info!("Probing serial port {conn} @ {rate}");
            // A port that does not open is the next rate, as `scan`'s continue.
            let opened = Params::parse(rate)
                .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
            let (port, params) = match opened {
                Ok(opened) => opened,
                Err(e) => {
                    log::error!("{e}");
                    continue;
                }
            };
            match probe(port, params) {
                Ok(Some(link)) => return Ok(vec![Box::new(GpdDevice::new(link, conn))]),
                Ok(None) => {}
                Err(e) => log::debug!("Scan failed: {e}."),
            }
        }
        Ok(Vec::new())
    }
}

/// `scan` after the open, for one rate. The port goes when this returns.
fn probe(port: Box<dyn Port>, params: Params) -> Result<Option<Link>> {
    // Which model does not matter to the asking.
    let mut link = Link::new(Some(port), params, &protocol::MODELS[0]);
    let mut reply = None;
    for _ in 0..IDN_RETRIES {
        let answer = link.ask("*IDN?\n", SCAN_BUFFER)?;
        if protocol::is_identification(&answer) {
            reply = Some(answer);
            break;
        }
    }
    let Some(reply) = reply else {
        log::error!("Device did not reply to identification request.");
        return Ok(None);
    };
    let Some(model) = protocol::lookup(&reply) else {
        log::error!("Unsupported model '{}'.", String::from_utf8_lossy(&reply));
        return Ok(None);
    };
    log::info!("Detected model '{}'.", model.name);
    link.model = model;
    if !link.read_everything()? {
        return Ok(None);
    }
    link.port = None;
    Ok(Some(link))
}

/// One power supply.
pub struct GpdDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    conn: String,
    limit_samples: u64,
    limit_msec: u64,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl GpdDevice {
    fn new(link: Link, conn: String) -> Self {
        let names = ["CH1", "CH2"];
        GpdDevice {
            info: DeviceInfo {
                driver: "gwinstek-gpd",
                vendor: "GW Instek".to_owned(),
                model: link.model.name.to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: None,
            },
            channels: names
                .iter()
                .enumerate()
                .map(|(i, n)| Channel::new(i, ChannelKind::Analog, (*n).to_owned()))
                .collect(),
            groups: names.iter().enumerate().map(|(i, n)| ChannelGroup::new(*n, vec![i])).collect(),
            conn,
            limit_samples: 0,
            limit_msec: 0,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
        }
    }

    /// A device of `model` on a port that stands in for the hardware, without
    /// a scan's reading. [`Device::open`] takes it.
    pub fn from_port(model: &'static Model, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMMS[0]).expect("the default parses");
        let conn = port.name().to_owned();
        let mut device = Self::new(Link::new(None, params, model), conn);
        device.pending = Some(port);
        device
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    /// The channel a group names. The core hands it as the device spells it.
    fn channel_of(&self, group: &str) -> Result<usize> {
        self.groups
            .iter()
            .position(|g| g.name == group)
            .ok_or_else(|| Error::ChannelGroup(group.to_owned()))
    }

    fn get_global(&self, key: &Key) -> Result<Value> {
        Ok(match key {
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            Key::ChannelConfig => Value::from(CHANNEL_MODE),
            Key::Enabled => Value::Bool(self.link().output_enabled),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set_channel(&mut self, key: &Key, value: &Value, channel: usize) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        let model = link.model;
        match key {
            Key::VoltageTarget => {
                let volts = in_range(key, value, &model.voltage)?;
                link.send(&protocol::voltage_command(channel, volts))?;
                link.channels[channel].voltage_target = volts as f32;
            }
            Key::CurrentLimit => {
                let amps = in_range(key, value, &model.current)?;
                link.send(&protocol::current_command(channel, amps))?;
                link.channels[channel].current_limit = amps as f32;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }
}

/// A value within `range`, each side in single precision: sigrok-cli's `strtof`
/// comes in there and `api.c` compares it to a double, so it refuses 3.2 A on the
/// model that lists 3.2 (`api.c:361`; 3.2f is above 3.2). A NaN passes those two
/// comparisons and goes out as `VSET1:  nan` to a live output, and a -0.0 as
/// `-0.000`, which nothing says a supply parses; here the one is refused and the
/// other is 0.
fn in_range(key: &Key, value: &Value, range: &protocol::Range) -> Result<f64> {
    let refused = || Error::Arg(format!("{} out of range", key.id()));
    let v = value.as_f64().ok_or_else(refused)?;
    let single = v as f32;
    if single >= range[0] as f32 && single <= range[1] as f32 {
        Ok(v + 0.0)
    } else {
        Err(refused())
    }
}

impl Device for GpdDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
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
        Ok(match group {
            None => DEV_OPTIONS.to_vec(),
            Some(g) => {
                self.channel_of(g)?;
                GROUP_OPTIONS.to_vec()
            }
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let Some(group) = group else { return self.get_global(key) };
        let channel = self.channel_of(group)?;
        let state = self.link().channels[channel];
        Ok(Value::F64(f64::from(match key {
            Key::Voltage => state.voltage,
            Key::VoltageTarget => state.voltage_target,
            Key::Current => state.current,
            Key::CurrentLimit => state.current_limit,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })))
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(group) = group {
            let channel = self.channel_of(group)?;
            return self.set_channel(key, value, channel);
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
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(bad)?;
                let mut link = self.link();
                link.send(if on { "OUT1\n" } else { "OUT0\n" })?;
                link.output_enabled = on;
            }
            // Listed as settable, and `config_set` has no case for it.
            Key::ChannelConfig => return Err(Error::not_supported("channel_config")),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let model = self.link().model;
        let Some(group) = group else {
            return match key {
                Key::ChannelConfig => Ok(ValueList::Str(vec![CHANNEL_MODE.to_owned()])),
                other => Err(Error::UnknownKey(other.id().to_owned())),
            };
        };
        self.channel_of(group)?;
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
            None => crate::serial::open(&self.conn, link.params)?,
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
        if self.link().port.is_none() {
            return Err(Error::device("device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            limit_samples: self.limit_samples,
            limit_msec: self.limit_msec,
            enabled: [self.channels[0].enabled, self.channels[1].enabled],
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("gwinstek-gpd acquisition".to_owned())
            .spawn(move || {
                capture.run(&sink, &stop2);
                sink.send(Packet::End);
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

/// One capture, on its own thread: `gpd_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limit_samples: u64,
    limit_msec: u64,
    /// `-C`, which C ignores: its packets go out for every channel.
    enabled: [bool; CHANNELS],
}

/// How a wait ended.
enum Wait {
    /// What was waited for came.
    Go(Option<u8>),
    /// A stop or the time limit.
    End,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) {
        sink.send(Packet::header());
        let started = Instant::now();
        let mut samples = 0u64;
        if let Err(e) = self.rounds(sink, stop, started, &mut samples) {
            log::error!("Acquisition failed: {e}.");
        }
    }

    fn time_up(&self, started: Instant) -> bool {
        self.limit_msec > 0 && started.elapsed() >= Duration::from_millis(self.limit_msec)
    }

    fn rounds(
        &self,
        sink: &PacketSink,
        stop: &AtomicBool,
        started: Instant,
        samples: &mut u64,
    ) -> Result<()> {
        loop {
            match self.idle(stop, started)? {
                Wait::Go(_) => {}
                Wait::End => return Ok(()),
            }
            {
                let mut link = self.link.lock().expect("link lock");
                for n in 1..=CHANNELS {
                    link.send(&format!("IOUT{n}?\nVOUT{n}?\n"))?;
                }
            }
            // Checked after each pass of the callback, the query's too.
            if self.time_up(started) {
                return Ok(());
            }
            let first = match self.first_byte(stop, started)? {
                Wait::Go(first) => first,
                Wait::End => return Ok(()),
            };
            if !self.round(sink, first)? {
                return Ok(());
            }
            *samples += 1;
            if self.limit_samples > 0 && *samples >= self.limit_samples {
                return Ok(());
            }
            if self.time_up(started) {
                return Ok(());
            }
        }
    }

    /// The time between two rounds. What comes then nobody asked for: the
    /// line end's second half on the firmware with CR LF, which says nothing
    /// (C logs it as an error and waits out a reply's time, each round), or a
    /// line, which is logged as C does.
    fn idle(&self, stop: &AtomicBool, started: Instant) -> Result<Wait> {
        let mut due = Instant::now() + POLL_INTERVAL;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(Wait::End);
            }
            let left = due.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(Wait::Go(None));
            }
            let mut link = self.link.lock().expect("link lock");
            match link.read_byte(left.min(STOP_CHECK))? {
                None | Some(b'\r' | b'\n') => {}
                Some(byte) => {
                    log::error!("No reply pending.");
                    let text = link.read_line(CAPTURE_BUFFER, Some(byte))?.unwrap_or_default();
                    log::error!("Unexpected data '{}'.", String::from_utf8_lossy(&text));
                    drop(link);
                    if self.time_up(started) {
                        return Ok(Wait::End);
                    }
                    due = Instant::now() + POLL_INTERVAL;
                }
            }
        }
    }

    /// Up to [`REPLY_BOUND`] for a round to begin, where C waits for ever. Only
    /// its timer's passes look at the limits there: every 100 ms.
    fn first_byte(&self, stop: &AtomicBool, started: Instant) -> Result<Wait> {
        let asked = Instant::now();
        let mut next_check = asked + POLL_INTERVAL;
        loop {
            if stop.load(Ordering::SeqCst) {
                return Ok(Wait::End);
            }
            let byte = self.link.lock().expect("link lock").read_byte(STOP_CHECK)?;
            if byte.is_some() {
                return Ok(Wait::Go(byte));
            }
            if Instant::now() >= next_check {
                if self.time_up(started) {
                    return Ok(Wait::End);
                }
                next_check += POLL_INTERVAL;
            }
            if asked.elapsed() >= REPLY_BOUND {
                return Err(Error::device("no reply to the readings"));
            }
        }
    }

    /// Reads a round and sends it: false when an answer is no number, which
    /// ends the capture, as this round's lines no longer say which is which.
    fn round(&self, sink: &PacketSink, mut first: Option<u8>) -> Result<bool> {
        let mut link = self.link.lock().expect("link lock");
        for ch in 0..CHANNELS {
            let Some(amps) = reading(&mut link, first.take(), "IOUT1?")? else {
                return Ok(false);
            };
            link.channels[ch].current = amps;
            if self.enabled[ch]
                && !sink.send(analog(ch, amps, Mq::Current, Unit::Ampere, MqFlags::empty()))
            {
                return Ok(false);
            }
            let Some(volts) = reading(&mut link, None, "VOUT1?")? else {
                return Ok(false);
            };
            link.channels[ch].voltage = volts;
            if self.enabled[ch]
                && !sink.send(analog(ch, volts, Mq::Voltage, Unit::Volt, MqFlags::DC))
            {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

/// A line that must be a number. Both channels' errors name channel 1, as C's.
fn reading(link: &mut Link, first: Option<u8>, name: &str) -> Result<Option<f32>> {
    let reply = link.read_line(CAPTURE_BUFFER, first)?.unwrap_or_default();
    let value = protocol::scan_f32(&reply);
    if value.is_none() {
        log::error!("Invalid reply to {name}: '{}'.", String::from_utf8_lossy(&reply));
    }
    Ok(value)
}

fn analog(channel: usize, value: f32, mq: Mq, unit: Unit, flags: MqFlags) -> Packet {
    Packet::Analog(AnalogData {
        channels: vec![channel],
        samples: vec![value],
        mq,
        flags,
        unit,
        digits: 3,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    /// How long a test waits for a capture before it fails: a stop or a limit
    /// that does not bite must fail, not hang the run.
    const CAPTURE_BOUND: Duration = Duration::from_secs(10);

    type Received = Receiver<(usize, Packet)>;

    type Seen = Arc<Mutex<Vec<(Instant, Vec<u8>)>>>;

    /// A supply that answers as the protocol says, noting each write and when.
    struct Fake {
        seen: Seen,
        input: VecDeque<u8>,
        eol: &'static [u8],
        fail_writes_after: Option<usize>,
        /// Queries after which no more are answered.
        answers: Option<usize>,
        /// Answers instead of the usual, for each query in turn.
        replies: VecDeque<Vec<u8>>,
        /// A line after each `VOUT2?`.
        chatter: Option<&'static [u8]>,
    }

    impl Fake {
        fn new(eol: &'static [u8]) -> (Fake, Seen) {
            let seen = Arc::new(Mutex::new(Vec::new()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                eol,
                fail_writes_after: None,
                answers: None,
                replies: VecDeque::new(),
                chatter: None,
            };
            (fake, seen)
        }

        fn answer(query: &[u8]) -> Vec<u8> {
            match query {
                b"*IDN?" => b"GW INSTEK,GPD-3303S,SN:1,V2.10".to_vec(),
                b"STATUS?" => b"11011110".to_vec(),
                b"ISET1?" => b"1.000A".to_vec(),
                b"VSET1?" => b"12.000V".to_vec(),
                b"ISET2?" => b"2.500A".to_vec(),
                b"VSET2?" => b"05.000V".to_vec(),
                b"IOUT1?" => b"0.100A".to_vec(),
                b"VOUT1?" => b"11.900V".to_vec(),
                b"IOUT2?" => b"0.200A".to_vec(),
                b"VOUT2?" => b"04.900V".to_vec(),
                _ => Vec::new(),
            }
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
            let n = self.input.len().min(buf.len());
            for b in &mut buf[..n] {
                *b = self.input.pop_front().expect("counted");
            }
            Ok(n)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut seen = self.seen.lock().unwrap();
            if self.fail_writes_after.is_some_and(|n| seen.len() >= n) {
                return Err(Error::device("Broken pipe"));
            }
            seen.push((Instant::now(), data.to_vec()));
            for query in data.split(|c| *c == b'\n').filter(|q| q.ends_with(b"?")) {
                if self.answers == Some(0) {
                    continue;
                }
                if let Some(n) = &mut self.answers {
                    *n -= 1;
                }
                let reply = self.replies.pop_front().unwrap_or_else(|| Fake::answer(query));
                self.input.extend(reply);
                self.input.extend(self.eol);
                if let (b"VOUT2?", Some(chatter)) = (query, self.chatter) {
                    self.input.extend(chatter);
                    self.input.extend(self.eol);
                }
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            self.input.clear();
            Ok(())
        }
    }

    /// One byte per read and a millisecond for each, as a slow device has it.
    struct Slow(Fake);
    impl Port for Slow {
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
            std::thread::sleep(Duration::from_millis(1));
            self.0.read(&mut buf[..1], timeout)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            self.0.write(data)
        }
        fn purge(&mut self) -> Result<()> {
            self.0.purge()
        }
    }

    fn device(fake: Fake) -> GpdDevice {
        let mut d = GpdDevice::from_port(&protocol::MODELS[1], Box::new(fake));
        d.open().unwrap();
        d
    }

    fn params() -> Params {
        Params::parse(SERIALCOMMS[0]).unwrap()
    }

    fn start(d: &mut GpdDevice) -> Received {
        let (tx, rx) = sync_channel(1024);
        d.start(PacketSink::new(tx, 0)).unwrap();
        rx
    }

    /// The (channel, value) of what a capture sends up to its End, which must
    /// come within [`CAPTURE_BOUND`]: a stop or a limit that does not bite fails.
    fn readings(rx: &Received) -> Vec<(usize, f32)> {
        let deadline = Instant::now() + CAPTURE_BOUND;
        let mut out = Vec::new();
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left).expect("no End in time").1 {
                Packet::Analog(a) => out.push((a.channels[0], a.samples[0])),
                Packet::End => return out,
                _ => {}
            }
        }
    }

    const NOTHING: [(usize, f32); 0] = [];
    const ROUND: [(usize, f32); 4] = [(0, 0.1), (0, 11.9), (1, 0.2), (1, 4.9)];

    #[test]
    fn a_scan_asks_and_sets_nothing_and_the_port_goes() {
        let (fake, seen) = Fake::new(b"\r\n");
        let link = probe(Box::new(fake), params()).unwrap().expect("a model");
        let writes: Vec<String> = seen
            .lock()
            .unwrap()
            .iter()
            .map(|(_, w)| String::from_utf8_lossy(w).trim_end().to_owned())
            .collect();
        assert_eq!(
            writes,
            [
                "*IDN?", "STATUS?", "ISET1?", "VSET1?", "IOUT1?", "VOUT1?", "ISET2?", "VSET2?",
                "IOUT2?", "VOUT2?",
            ]
        );
        assert_eq!(link.model.name, "GPD-3303S");
        assert!(link.port.is_none(), "closed, unlike C's scan");
        assert!(link.output_enabled);
        assert_eq!(link.channels[1].voltage, 4.9);
        assert_eq!(link.channels[0].current_limit, 1.0);
    }

    #[test]
    fn a_silent_answer_in_a_scan_is_no_device_not_the_last_answer() {
        // C parses what the buffer held: an ISET1? unanswered reads the status.
        let (mut fake, _) = Fake::new(b"\n");
        fake.answers = Some(2);
        assert!(probe(Box::new(fake), params()).unwrap().is_none());
    }

    #[test]
    fn the_id_gets_three_tries() {
        let (mut fake, seen) = Fake::new(b"\n");
        fake.replies = VecDeque::from([b"Invalid Character.".to_vec(), b"x".to_vec()]);
        assert!(probe(Box::new(fake), params()).unwrap().is_some());
        assert_eq!(seen.lock().unwrap().iter().filter(|(_, w)| w == b"*IDN?\n").count(), 3);
        let (mut fake, seen) = Fake::new(b"\n");
        fake.answers = Some(0);
        assert!(probe(Box::new(fake), params()).unwrap().is_none());
        assert_eq!(seen.lock().unwrap().len(), 3, "and no more");
    }

    #[test]
    fn a_line_stops_at_the_buffer_and_at_its_time() {
        let (mut fake, _) = Fake::new(b"\n");
        fake.input.extend(vec![b'1'; 51]);
        fake.input.extend(b"\nrest\n");
        let mut link = Link::new(Some(Box::new(fake)), params(), &protocol::MODELS[0]);
        assert_eq!(link.read_line(CAPTURE_BUFFER, None).unwrap(), None, "51 of 50");
        // C reads the same 51 bytes and no more: the line end is what comes next.
        assert_eq!(link.read_line(CAPTURE_BUFFER, None).unwrap().unwrap(), b"rest");

        let (mut fake, _) = Fake::new(b"\n");
        fake.input.extend(vec![b'1'; 50]);
        fake.input.extend(b"\n");
        let mut link = Link::new(Some(Box::new(fake)), params(), &protocol::MODELS[0]);
        assert_eq!(link.read_line(CAPTURE_BUFFER, None).unwrap().unwrap().len(), 50);

        // Line ends without an end: a device can't hold the read past its time.
        let (mut fake, _) = Fake::new(b"\n");
        fake.input.extend(vec![b'\n'; 1 << 20]);
        fake.input.extend(b"late\n");
        let mut link = Link::new(Some(Box::new(Slow(fake))), params(), &protocol::MODELS[0]);
        let began = Instant::now();
        assert_eq!(link.read_line(CAPTURE_BUFFER, None).unwrap(), None);
        assert!(began.elapsed() < Duration::from_secs(2));
    }

    #[test]
    fn a_capture_counts_a_sample_per_round_and_stops_at_the_limit() {
        let (fake, seen) = Fake::new(b"\r\n");
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
        let rx = start(&mut d);
        let got = readings(&rx);
        assert_eq!(got, [ROUND, ROUND].concat());
        let seen = seen.lock().unwrap();
        // Two writes per round, and no third round's.
        assert_eq!(seen.len(), 4, "{seen:?}");
        assert_eq!(seen[0].1, b"IOUT1?\nVOUT1?\n");
        assert_eq!(seen[1].1, b"IOUT2?\nVOUT2?\n");
    }

    #[test]
    fn rounds_are_100_ms_apart() {
        let (fake, seen) = Fake::new(b"\r\n");
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        readings(&rx);
        let seen = seen.lock().unwrap();
        for pair in [(0, 2), (2, 4)] {
            let gap = seen[pair.1].0 - seen[pair.0].0;
            assert!(gap >= POLL_INTERVAL, "{gap:?}");
            assert!(gap < POLL_INTERVAL * 3, "{gap:?}");
        }
    }

    #[test]
    fn the_second_half_of_a_line_end_costs_nothing_and_says_nothing() {
        // C logs two errors and waits 250 ms per round on this.
        for eol in [&b"\r"[..], b"\n", b"\r\n"] {
            let (fake, _) = Fake::new(eol);
            let mut d = device(fake);
            d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
            let began = Instant::now();
            let rx = start(&mut d);
            assert_eq!(readings(&rx), [ROUND, ROUND].concat());
            assert!(began.elapsed() < POLL_INTERVAL * 4, "{:?}", began.elapsed());
        }
    }

    #[test]
    fn an_answer_that_is_no_number_ends_the_capture() {
        let (mut fake, seen) = Fake::new(b"\r\n");
        fake.replies = VecDeque::from([b"0.100A".to_vec(), b"x".to_vec()]);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(5), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [(0, 0.1)]);
        assert_eq!(seen.lock().unwrap().len(), 2, "no round after it");
    }

    #[test]
    fn a_missing_answer_ends_the_capture_with_no_made_up_value() {
        // Three answers of a round, and the CR LF's half makes C parse old bytes.
        let (mut fake, _) = Fake::new(b"\r\n");
        fake.answers = Some(3);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(5), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), ROUND[..3]);
    }

    #[test]
    fn a_silent_device_ends_a_sample_limited_capture_within_the_bound() {
        let (mut fake, seen) = Fake::new(b"\r\n");
        fake.answers = Some(0);
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
        assert!(began.elapsed() >= REPLY_BOUND);
        assert_eq!(seen.lock().unwrap().len(), 2, "asked once");
    }

    #[test]
    fn a_time_limit_ends_a_wait_for_a_silent_device_ahead_of_the_bound() {
        let (mut fake, _) = Fake::new(b"\r\n");
        fake.answers = Some(0);
        let mut d = device(fake);
        d.set(&Key::LimitMsec, &Value::U64(200), None).unwrap();
        let began = Instant::now();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
        assert!(began.elapsed() < REPLY_BOUND);
    }

    #[test]
    fn a_stop_ends_a_wait_for_a_silent_device() {
        let (mut fake, _) = Fake::new(b"\r\n");
        fake.answers = Some(0);
        let mut d = device(fake);
        let rx = start(&mut d);
        std::thread::sleep(POLL_INTERVAL * 2);
        d.stop().unwrap();
        let began = Instant::now();
        assert_eq!(readings(&rx), NOTHING);
        assert!(began.elapsed() < REPLY_BOUND / 2);
    }

    #[test]
    fn a_stop_ends_a_capture_that_runs_and_asks_no_more() {
        let (fake, seen) = Fake::new(b"\r\n");
        let mut d = device(fake);
        let rx = start(&mut d);
        // Up to its first round, then into the 100 ms before the next.
        let deadline = Instant::now() + CAPTURE_BOUND;
        while seen.lock().unwrap().len() < 2 {
            assert!(Instant::now() < deadline, "no round");
            std::thread::sleep(Duration::from_millis(1));
        }
        std::thread::sleep(POLL_INTERVAL / 3);
        d.stop().unwrap();
        assert_eq!(readings(&rx), ROUND);
        std::thread::sleep(POLL_INTERVAL * 2);
        assert_eq!(seen.lock().unwrap().len(), 2, "a query after the stop");
    }

    #[test]
    fn a_failed_write_ends_the_capture_and_fails_a_setting() {
        // C drops both results: its capture waits for a reply for ever, its
        // setting says all is well and keeps the value.
        let (mut fake, _) = Fake::new(b"\r\n");
        fake.fail_writes_after = Some(0);
        let mut d = device(fake);
        assert!(d.set(&Key::Enabled, &Value::Bool(true), None).is_err());
        assert_eq!(d.get(&Key::Enabled, None).unwrap(), Value::Bool(false));
        assert!(d.set(&Key::VoltageTarget, &Value::F64(1.0), Some("CH1")).is_err());
        assert_eq!(d.get(&Key::VoltageTarget, Some("CH1")).unwrap(), Value::F64(0.0));
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), NOTHING);
    }

    #[test]
    fn data_no_one_asked_for_is_read_and_shifts_nothing() {
        let (mut fake, _) = Fake::new(b"\r\n");
        fake.chatter = Some(b"12.5");
        let mut d = device(fake);
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), [ROUND, ROUND, ROUND].concat());
    }

    #[test]
    fn a_disabled_channel_sends_nothing_but_is_still_asked() {
        let (fake, seen) = Fake::new(b"\r\n");
        let mut d = device(fake);
        d.enable_channel(0, false).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
        let rx = start(&mut d);
        assert_eq!(readings(&rx), ROUND[2..]);
        assert_eq!(seen.lock().unwrap().len(), 2);
    }

    #[test]
    fn settings_go_out_and_read_back_as_a_c_float() {
        let (fake, seen) = Fake::new(b"\n");
        let mut d = device(fake);
        d.set(&Key::VoltageTarget, &Value::F64(3.3), Some("CH1")).unwrap();
        d.set(&Key::CurrentLimit, &Value::F64(0.1), Some("CH2")).unwrap();
        d.set(&Key::Enabled, &Value::Bool(true), None).unwrap();
        d.set(&Key::Enabled, &Value::Bool(false), None).unwrap();
        let writes: Vec<Vec<u8>> = seen.lock().unwrap().iter().map(|(_, w)| w.clone()).collect();
        assert_eq!(
            writes,
            [&b"VSET1:3.300\n"[..], b"ISET2:0.100\n", b"OUT1\n", b"OUT0\n"].map(<[u8]>::to_vec)
        );
        assert_eq!(d.get(&Key::VoltageTarget, Some("CH1")).unwrap(), Value::F64(f64::from(3.3f32)));
        assert_eq!(d.get(&Key::Enabled, None).unwrap(), Value::Bool(false));
    }

    #[test]
    fn the_range_is_checked_in_single_precision_and_refuses_a_nan() {
        let (fake, seen) = Fake::new(b"\n");
        let mut d = device(fake);
        // The GPD-3303S: 32 V, 3.2 A, its last entries as listed.
        for (key, ok) in [
            (Key::VoltageTarget, 32.0),
            (Key::VoltageTarget, 0.0),
            (Key::VoltageTarget, -0.0),
            (Key::CurrentLimit, 3.2),
            // Above in double precision, the maximum in single.
            (Key::VoltageTarget, 32.000_000_1),
        ] {
            d.set(&key, &Value::F64(ok), Some("CH2")).unwrap();
        }
        let sent = seen.lock().unwrap().len();
        for (key, bad) in [
            (Key::VoltageTarget, 32.01),
            (Key::VoltageTarget, -0.5),
            (Key::VoltageTarget, f64::NAN),
            (Key::VoltageTarget, f64::INFINITY),
            (Key::CurrentLimit, 3.21),
            (Key::CurrentLimit, f64::NAN),
        ] {
            assert!(d.set(&key, &Value::F64(bad), Some("CH2")).is_err(), "{bad}");
        }
        assert_eq!(seen.lock().unwrap().len(), sent, "nothing sent for those");
        let seen = seen.lock().unwrap();
        assert_eq!(seen[2].1, b"VSET2:0.000\n", "-0.0 goes out as 0");
    }

    #[test]
    fn channel_config_reads_and_lists_and_cannot_be_set() {
        let (fake, seen) = Fake::new(b"\n");
        let mut d = device(fake);
        assert_eq!(d.get(&Key::ChannelConfig, None).unwrap(), Value::from(CHANNEL_MODE));
        assert!(d.set(&Key::ChannelConfig, &Value::from(CHANNEL_MODE), None).is_err());
        assert!(seen.lock().unwrap().is_empty());
    }

    #[test]
    fn the_options_are_the_cs_and_groups_have_theirs() {
        let (fake, _) = Fake::new(b"\n");
        let d = device(fake);
        let keys = |g| -> Vec<Key> { d.options(g).unwrap().into_iter().map(|o| o.key).collect() };
        assert_eq!(keys(None).len(), 5);
        assert_eq!(keys(Some("CH1")).len(), 4);
        assert!(d.options(Some("CH3")).is_err());
        assert_eq!(d.get(&Key::Voltage, None).ok(), None);
        assert_eq!(d.get(&Key::Enabled, Some("CH1")).ok(), None);
        assert!(matches!(
            d.list(&Key::CurrentLimit, Some("CH1")).unwrap(),
            ValueList::RangeStepF64 { min, max, .. } if min.abs() < 1e-9 && (max - 3.2).abs() < 1e-9
        ));
    }
}
