// Counterpart in sigrok:
//   libsigrok/src/hardware/raspberrypi-pico/api.c,
//   libsigrok/src/hardware/raspberrypi-pico/protocol.c,
//   libsigrok/src/hardware/raspberrypi-pico/protocol.h,
//   Copyright (C) 2022 Shawn Walker <ac0bi00@gmail.com>,
//   with commits by Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! A Raspberry Pi Pico running the sigrok-pico firmware: up to 32
//! digital channels and 8 analog channels streamed over its USB serial
//! port, the `raspberrypi-pico` driver.
//!
//! The protocol is text commands with `*` acknowledgements, and a
//! sample stream of 7 bit fields with run length bytes. Triggers run on
//! the host: the device streams until told to stop.
//!
//! `conn=` names the serial port and is required; `serialcomm=`
//! overrides `115200/8n1/dtr=1/rts=0`; `force_detect=` appends a user
//! string to the identify command. `OXROK_SERIAL_RECORD=FILE` records
//! the session for replay.

#![forbid(unsafe_code)]

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, PreTriggerBuffer, Result,
    SoftTrigger, Trigger, TriggerFeatures, TriggerMatch, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The line parameters the firmware expects.
pub const SERIALCOMM: &str = "115200/8n1/dtr=1/rts=0";
const SCAN_OPTIONS: [Key; 3] = [Key::Conn, Key::Serialcomm, Key::ForceDetect];
const CLASSES: [DeviceClass; 2] = [DeviceClass::Oscilloscope, DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
];
const TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];
/// The sample rates sigrok lists; the firmware takes any value and
/// rounds as it can.
pub static SAMPLERATES: &[u64] = &[
    5_000,
    6_000,
    8_000,
    10_000,
    20_000,
    30_000,
    40_000,
    50_000,
    60_000,
    80_000,
    100_000,
    125_000,
    150_000,
    160_000,
    200_000,
    250_000,
    300_000,
    400_000,
    500_000,
    600_000,
    800_000,
    1_000_000,
    1_200_000,
    1_500_000,
    2_000_000,
    2_400_000,
    3_000_000,
    4_000_000,
    5_000_000,
    6_000_000,
    8_000_000,
    10_000_000,
    15_000_000,
    20_000_000,
    30_000_000,
    40_000_000,
    60_000_000,
    80_000_000,
    100_000_000,
    120_000_000,
    150_000_000,
    200_000_000,
    240_000_000,
];
const MAX_ANALOG: usize = 8;
const MAX_DIGITAL: usize = 32;
const ANALOG_DIGITS: i32 = 4;
/// Bytes read from the port per call.
const READ_SIZE: usize = 32_000;

/// The driver.
pub struct RaspberryPiPicoDriver;

impl Driver for RaspberryPiPicoDriver {
    fn id(&self) -> &'static str {
        "raspberrypi-pico"
    }

    fn long_name(&self) -> &'static str {
        "RaspberryPI PICO"
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
                Key::ForceDetect => force_detect = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let params = Params::parse(&serialcomm)?;
        log::info!("Opening {conn}.");
        let mut port = crate::serial::open(&conn, params)?;
        let Some(dev) = probe(port.as_mut(), force_detect.as_deref())? else {
            return Ok(Vec::new());
        };
        Ok(vec![Box::new(PicoDevice::new(dev, conn, port))])
    }
}

/// How long the output may take to leave before an acquisition starts.
const TX_DRAIN_TIMEOUT: Duration = Duration::from_secs(1);

/// What the identify command reported.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    /// Analog channels.
    pub analog: usize,
    /// Bytes per analog sample on the wire, 1 to 4.
    pub analog_size: usize,
    /// Digital channels.
    pub digital: usize,
}

/// Parses `SRPICO,AxxyDzz,VV`: analog count, analog sample size,
/// digital count, protocol version 02.
pub fn parse_identity(text: &str) -> Option<Identity> {
    let b = text.as_bytes();
    if b.len() < 16 || !text.starts_with("SRPICO,A") || b[11] != b'D' || &b[15..] != b"02" {
        return None;
    }
    let analog: usize = text[8..10].parse().ok()?;
    let analog_size = usize::from(b[10].checked_sub(b'0')?);
    let digital: usize = text[12..14].parse().ok()?;
    if (analog == 0 && digital == 0)
        || analog > MAX_ANALOG
        || digital > MAX_DIGITAL
        || !(1..=4).contains(&analog_size)
    {
        return None;
    }
    Some(Identity { analog, analog_size, digital })
}

/// Reads a response of unknown length: waits up to a second for the
/// first byte, then 10 ms for the rest.
fn read_response(port: &mut dyn Port, max: usize) -> Result<Vec<u8>> {
    let mut buf = vec![0u8; max];
    let mut n = 0;
    let start = Instant::now();
    while n == 0 && start.elapsed() < Duration::from_secs(1) {
        n = port.read(&mut buf, Duration::from_millis(50))?;
    }
    if n == 0 {
        return Ok(Vec::new());
    }
    while n < max {
        let more = port.read(&mut buf[n..], Duration::from_millis(10))?;
        if more == 0 {
            break;
        }
        n += more;
    }
    buf.truncate(n);
    Ok(buf)
}

/// Sends `cmd` and reads the answer.
fn command(port: &mut dyn Port, cmd: &str, max: usize) -> Result<Vec<u8>> {
    port.write(cmd.as_bytes())?;
    read_response(port, max)
}

/// Sends `cmd` and expects a `*` back.
fn command_ack(port: &mut dyn Port, cmd: &str) -> Result<()> {
    let mut junk = [0u8; 2];
    while port.read(&mut junk, Duration::from_millis(10))? > 0 {
        log::debug!("Dropping stale bytes before {}", cmd.trim_end());
    }
    port.write(cmd.as_bytes())?;
    let mut buf = [0u8; 1];
    let n = port.read(&mut buf, Duration::from_secs(1))?;
    if n == 1 && buf[0] == b'*' {
        return Ok(());
    }
    Err(Error::device(format!("Command {} was not acknowledged.", cmd.trim_end())))
}

/// Drops whatever the device is still sending.
fn drain(port: &mut dyn Port) -> Result<()> {
    let mut buf = [0u8; 256];
    while port.read(&mut buf, Duration::from_millis(100))? > 0 {}
    Ok(())
}

/// Resets the device and asks it to identify itself.
fn identify(port: &mut dyn Port, force_detect: Option<&str>) -> Result<Vec<u8>> {
    port.write(b"*")?;
    std::thread::sleep(Duration::from_millis(10));
    drain(port)?;
    let cmd = match force_detect {
        Some(s) if s.len() <= 60 => format!("i{s}\n"),
        _ => "i\n".to_owned(),
    };
    command(port, &cmd, 17)
}

/// Probes an open port for the firmware; `None` when nothing answers.
pub fn probe(port: &mut dyn Port, force_detect: Option<&str>) -> Result<Option<Identity>> {
    let mut answer = identify(port, force_detect)?;
    if answer.len() < 16 {
        log::info!("First identify got {} bytes, retrying.", answer.len());
        std::thread::sleep(Duration::from_millis(100));
        answer = identify(port, force_detect)?;
    }
    let text = String::from_utf8_lossy(&answer).trim_end().to_owned();
    let id = parse_identity(&text);
    if id.is_none() {
        log::info!("Bad identify response '{text}'.");
    }
    Ok(id)
}

/// Reads the scale and offset of every analog channel: `SCALExOFFSET`
/// in microvolts per count and microvolts.
fn read_analog_config(port: &mut dyn Port, analog: usize) -> Result<Vec<(f32, f32)>> {
    let mut out = Vec::with_capacity(analog);
    for i in 0..analog {
        let answer = command(port, &format!("a{i}\n"), 20)?;
        if answer.is_empty() {
            return Err(Error::device("No response from device for analog channel query"));
        }
        let text = String::from_utf8_lossy(&answer);
        let parsed = text.trim().split_once('x').and_then(|(s, o)| {
            Some((
                s.trim().parse::<i64>().ok()? as f32 / 1e6,
                o.trim().parse::<i64>().ok()? as f32 / 1e6,
            ))
        });
        if parsed.is_none() {
            log::warn!("Unparseable analog scale response '{}' for A{i}.", text.trim());
        }
        out.push(parsed.unwrap_or((0.0257, 0.0)));
    }
    Ok(out)
}

/// Settings kept between acquisitions.
#[derive(Clone, Debug)]
struct Settings {
    samplerate: u64,
    limit_samples: u64,
    capture_ratio: u64,
}

/// One Pico.
pub struct PicoDevice {
    info: DeviceInfo,
    identity: Identity,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    trigger: Option<Trigger>,
    analog_config: Vec<(f32, f32)>,
    conn: String,
    pending: Option<Box<dyn Port>>,
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl PicoDevice {
    /// A device on an open, identified port.
    pub fn new(identity: Identity, conn: String, port: Box<dyn Port>) -> PicoDevice {
        // Analog channels first, then digital, as libsigrok creates
        // them; show sorts by index for display, outputs keep this
        // order for their columns.
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        for i in 0..identity.analog {
            channels.push(Channel::new(i, ChannelKind::Analog, format!("A{i}")));
            groups.push(ChannelGroup::new(format!("A{i}"), vec![i]));
        }
        for i in 0..identity.digital {
            // Digital channel names start at D2 to match the board.
            channels.push(Channel::new(i, ChannelKind::Logic, format!("D{}", i + 2)));
        }
        PicoDevice {
            info: DeviceInfo {
                driver: "raspberrypi-pico",
                vendor: "Raspberry Pi".to_owned(),
                model: "PICO".to_owned(),
                version: "00".to_owned(),
                serial: "N/A".to_owned(),
                // sigrok shows no connection for serial devices.
                conn: None,
            },
            identity,
            channels,
            groups,
            settings: Settings { samplerate: 5000, limit_samples: 1000, capture_ratio: 10 },
            trigger: None,
            analog_config: Vec::new(),
            conn,
            pending: Some(port),
            port: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }
}

impl Device for PicoDevice {
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
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) if self.groups.iter().any(|cg| cg.name == g) => Ok(Vec::new()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        Ok(match key {
            Key::Samplerate => Value::U64(self.settings.samplerate),
            Key::CaptureRatio => Value::U64(self.settings.capture_ratio),
            Key::LimitSamples => Value::U64(self.settings.limit_samples),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => self.settings.samplerate = value.as_u64().ok_or_else(bad)?,
            Key::LimitSamples => self.settings.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::CaptureRatio => self.settings.capture_ratio = value.as_u64().ok_or_else(bad)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Ok(match key {
            Key::Samplerate => ValueList::U64(SAMPLERATES.to_vec()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::LimitSamples => ValueList::MinMaxU64(1, 1_000_000_000),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn trigger_features(&self) -> TriggerFeatures {
        TriggerFeatures { max_stages: 1, ..SoftTrigger::FEATURES }
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            SoftTrigger::check(t)?;
            if t.stages.len() > 1 {
                return Err(Error::not_supported("the Pico takes one trigger stage"));
            }
            // Conditions carry channel indices; analog and digital
            // channels share them here, and the trigger is digital.
            for stage in &t.stages {
                for c in &stage.conditions {
                    if c.channel >= self.identity.digital {
                        return Err(Error::not_supported("triggers apply to digital channels"));
                    }
                }
            }
        }
        self.trigger = trigger;
        Ok(())
    }

    fn open(&mut self) -> Result<()> {
        if self.port.lock().expect("port lock").is_some() {
            return Ok(());
        }
        let mut port = match self.pending.take() {
            Some(p) => p,
            None => crate::serial::open(&self.conn, Params::parse(SERIALCOMM)?)?,
        };
        self.analog_config = read_analog_config(port.as_mut(), self.identity.analog)?;
        *self.port.lock().expect("port lock") = Some(port);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        self.port.lock().expect("port lock").take();
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.worker.is_some() {
            return Err(Error::Busy);
        }
        let port = self.port.lock().expect("port lock").take();
        let Some(port) = port else {
            return Err(Error::device("device is not open"));
        };
        let acq = Acquisition {
            identity: self.identity.clone(),
            channels: self.channels.clone(),
            settings: self.settings.clone(),
            trigger: self.trigger.clone(),
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("raspberrypi-pico acquisition".to_owned())
            .spawn(move || {
                let mut port = port;
                let outcome = acq.run(port.as_mut(), &sink, &stop2);
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}.");
                }
                sink.end(outcome);
                *slot.lock().expect("port lock") = Some(port);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = self.worker.take() {
            w.stop.store(true, Ordering::SeqCst);
            let _ = w.handle.join();
        }
        Ok(())
    }
}

/// One acquisition on its own thread.
struct Acquisition {
    identity: Identity,
    channels: Vec<Channel>,
    settings: Settings,
    trigger: Option<Trigger>,
}

/// How the stream ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StreamEnd {
    /// Still streaming.
    Active,
    /// `$` seen: the byte count follows.
    Stopped,
    /// Something else: the capture is lost.
    Aborted,
}

/// Decoded samples of one chunk: logic slices packed as the session
/// wants them, analog values per analog channel index.
#[derive(Default)]
struct Slices {
    logic: Vec<u8>,
    analog: Vec<Vec<f32>>,
    count: usize,
}

/// The stream decoder: masks, sizes and the last slice for run lengths.
struct Decoder {
    num_a: usize,
    num_d: usize,
    a_size: usize,
    a_mask: u32,
    d_mask: u32,
    bytes_per_slice: usize,
    dig_sample_bytes: usize,
    a_scale: Vec<(f32, f32)>,
    d_last: [u8; 4],
    a_last: Vec<f32>,
    /// Data bytes consumed, which the device reports at the end.
    byte_count: u64,
    end: StreamEnd,
    /// Bytes after the `$` marker, up to the `+`.
    tail: Vec<u8>,
}

impl Decoder {
    /// Whether the compact four channel format is in use: no analog
    /// channels and only D2 to D5.
    fn d4_mode(&self) -> bool {
        self.a_mask == 0 && self.d_mask & 0xffff_fff0 == 0
    }

    fn push_last(&self, out: &mut Slices, times: usize) {
        for _ in 0..times {
            out.logic.extend_from_slice(&self.d_last[..self.dig_sample_bytes]);
            for (i, a) in out.analog.iter_mut().enumerate() {
                if self.a_mask >> i & 1 != 0 {
                    a.push(self.a_last[i]);
                }
            }
        }
        out.count += times;
    }

    fn set_last(&mut self, word: u32) {
        self.d_last = word.to_le_bytes();
    }

    /// Decodes as much of `buf` as forms whole slices. Returns the
    /// decoded samples and the bytes consumed.
    fn decode(&mut self, buf: &[u8], out: &mut Slices) -> usize {
        out.analog.resize(self.num_a, Vec::new());
        if self.d4_mode() { self.decode_d4(buf, out) } else { self.decode_slices(buf, out) }
    }

    fn decode_d4(&mut self, buf: &[u8], out: &mut Slices) -> usize {
        let mut pos = 0;
        let mut rle = 0usize;
        while pos < buf.len() {
            let b = buf[pos];
            if (48..=127).contains(&b) {
                rle += usize::from(b - 47) * 8;
                self.byte_count += 1;
            } else if b >= 0x80 {
                rle += usize::from((b & 0x70) >> 4);
                if rle > 0 {
                    self.push_last(out, rle);
                    rle = 0;
                }
                self.set_last(u32::from(b & 0x0f));
                self.push_last(out, 1);
                self.byte_count += 1;
            } else {
                self.end = if b == b'$' { StreamEnd::Stopped } else { StreamEnd::Aborted };
                if self.end == StreamEnd::Aborted {
                    log::error!("Data stream aborts with byte {b:#04x} at {pos}.");
                }
                break;
            }
            pos += 1;
        }
        if rle > 0 {
            self.push_last(out, rle);
        }
        pos
    }

    fn decode_slices(&mut self, buf: &[u8], out: &mut Slices) -> usize {
        // Data bytes are 0x30 and up; anything lower ends the stream.
        let mut data_len = buf.len();
        for (i, b) in buf.iter().enumerate() {
            if *b < 0x30 {
                data_len = i;
                self.end = if *b == b'$' { StreamEnd::Stopped } else { StreamEnd::Aborted };
                if self.end == StreamEnd::Aborted {
                    log::error!("Data stream aborts with byte {b:#04x} at {i}.");
                }
                break;
            }
        }
        let mut pos = 0;
        while pos < data_len {
            let b = buf[pos];
            if b < 0x80 {
                let rle = if b <= 79 { usize::from(b) - 47 } else { (usize::from(b) - 78) * 32 };
                if (1..=1568).contains(&rle) {
                    self.push_last(out, rle);
                } else {
                    log::error!("Bad run length {rle} in byte {b:#04x}.");
                }
                pos += 1;
                self.byte_count += 1;
                continue;
            }
            if pos + self.bytes_per_slice > data_len {
                break;
            }
            let mut word = 0u32;
            let mut p = pos;
            for i in (0..self.num_d).step_by(7) {
                if self.d_mask >> i & 0x7f != 0 {
                    word |= u32::from(buf[p] & 0x7f) << i;
                    p += 1;
                }
            }
            self.set_last(word);
            for i in 0..self.num_a {
                if self.a_mask >> i & 1 != 0 {
                    let mut raw = 0u32;
                    for k in 0..self.a_size {
                        raw |= u32::from(buf[p + k].wrapping_sub(0x80)) << (7 * k);
                    }
                    p += self.a_size;
                    let (scale, offset) = self.a_scale[i];
                    self.a_last[i] = raw as f32 * scale + offset;
                }
            }
            self.byte_count += (p - pos) as u64;
            pos = p;
            self.push_last(out, 1);
        }
        pos
    }
}

impl Acquisition {
    #[allow(clippy::too_many_lines)]
    fn run(&self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let id = &self.identity;
        // As dev_acquisition_start's serial_drain, bounded; then what the
        // device still had to say, which is this driver's own.
        port.drain(TX_DRAIN_TIMEOUT)?;
        drain(port)?;
        port.write(b"*")?;
        port.drain(TX_DRAIN_TIMEOUT)?;
        drain(port)?;
        let mut a_mask = 0u32;
        let mut d_mask = 0u32;
        let (mut a_enabled, mut d_enabled) = (0usize, 0usize);
        for ch in &self.channels {
            let on = u8::from(ch.enabled);
            match ch.kind {
                ChannelKind::Analog => {
                    if ch.enabled {
                        a_mask |= 1 << ch.index;
                        a_enabled += 1;
                    }
                    command_ack(port, &format!("A{on}{}\n", ch.index))?;
                }
                ChannelKind::Logic => {
                    if ch.enabled {
                        d_mask |= 1 << ch.index;
                        d_enabled += 1;
                    }
                    command_ack(port, &format!("D{on}{}\n", ch.index))?;
                }
            }
        }
        if d_mask != 0 && (d_mask + 1) & d_mask != 0 {
            return Err(Error::Arg(format!(
                "Digital channel mask {d_mask:#x} not continuous: enable the digital channels from D2 up without gaps."
            )));
        }
        if a_enabled == 0 && d_enabled == 0 {
            return Err(Error::arg("No channels enabled"));
        }
        let mut bytes_per_slice = a_enabled * id.analog_size;
        for i in (0..id.digital).step_by(7) {
            if d_mask >> i & 0x7f != 0 {
                bytes_per_slice += 1;
            }
        }
        let mut rate = self.settings.samplerate;
        if a_enabled == 3 && rate > 160_000 {
            log::warn!("3 channel ADC sample rate above 160 kHz");
        }
        if a_enabled == 2 && rate > 250_000 {
            log::warn!("2 channel ADC sample rate above 250 kHz");
        }
        if a_enabled == 1 && rate > 500_000 {
            log::warn!("1 channel ADC sample rate above 500 kHz");
        }
        if rate < 5000 {
            log::warn!("Sample rate override to min of 5 kHz");
            rate = 5000;
        }
        if rate > 120_000_000 {
            log::warn!("Sample rate above 120 MHz");
        }
        if a_enabled > 0 && d_enabled > 0 && 24_000_000 % rate != 0 {
            // Both clocks must divide 24 MHz or the streams drift apart.
            let div = 24_000_000 / rate + 1;
            rate = 24_000_000 / div;
            if rate < 5000 {
                rate = 50_000;
            }
            log::warn!("Forcing common integer divisor sample rate of {rate} (div {div})");
        }
        if a_enabled > 0 && 48_000_000 % (rate * a_enabled as u64) != 0 {
            log::warn!(
                "Non integer ADC divisor of the 48 MHz clock for sample rate {rate} may cause sample to sample variability."
            );
        }
        if d_enabled > 0 && 120_000_000 % rate != 0 {
            log::warn!(
                "Non integer PIO divisor of the 120 MHz clock for sample rate {rate} may cause sample to sample variability."
            );
        }
        let limit = self.settings.limit_samples;
        command_ack(port, &format!("L{limit}\n"))?;
        let a_scale = read_analog_config(port, id.analog)?;
        let answer = command(port, &format!("R{rate}\n"), 30)?;
        if answer.first() != Some(&b'*') {
            return Err(Error::device(format!(
                "Sample rate to device failed: '{}'",
                String::from_utf8_lossy(&answer).trim()
            )));
        }
        if answer.len() > 1 {
            log::debug!("Sample rate response: {}", String::from_utf8_lossy(&answer[1..]).trim());
        }
        // Whatever the device still had to say.
        let mut junk = vec![0u8; READ_SIZE];
        let n = port.read(&mut junk, Duration::from_millis(50))?;
        if n > 0 {
            log::info!(
                "Pre-arm drain had {n} bytes: {}",
                String::from_utf8_lossy(&junk[..n]).trim()
            );
        }
        let dig_sample_bytes = id.digital.div_ceil(8);
        let pretrig = (self.settings.capture_ratio * limit / 100) as usize;
        let mut soft = None;
        if let Some(trigger) = &self.trigger {
            for stage in &trigger.stages {
                for c in &stage.conditions {
                    if c.channel >= id.digital || d_mask >> c.channel & 1 == 0 {
                        continue;
                    }
                    let code = match c.matches {
                        TriggerMatch::Zero => 0,
                        TriggerMatch::One => 1,
                        TriggerMatch::Rising => 2,
                        TriggerMatch::Falling => 3,
                        TriggerMatch::Edge => 4,
                        _ => continue,
                    };
                    command_ack(port, &format!("t{code}{:02}\n", c.channel + 2))?;
                }
            }
            command_ack(port, &format!("p{pretrig}\n"))?;
            soft = SoftTrigger::new(trigger, dig_sample_bytes.max(1));
            log::info!("Entering sw triggered mode");
            port.write(b"C\n")?;
        } else {
            log::info!("Entering fixed sample mode");
            port.write(b"F\n")?;
        }
        sink.send(Packet::header());
        if soft.is_none() {
            sink.send(Packet::Trigger);
        }
        let mut dec = Decoder {
            num_a: id.analog,
            num_d: id.digital,
            a_size: id.analog_size,
            a_mask,
            d_mask,
            bytes_per_slice,
            dig_sample_bytes,
            a_scale,
            d_last: [0; 4],
            a_last: vec![0.0; id.analog],
            byte_count: 0,
            end: StreamEnd::Active,
            tail: Vec::new(),
        };
        let mut pre_logic = PreTriggerBuffer::new(dig_sample_bytes.max(1), pretrig);
        let mut pre_analog: Vec<VecDeque<f32>> = vec![VecDeque::new(); id.analog];
        let mut sent = 0u64;
        let mut stop_sent = false;
        let mut buf: Vec<u8> = Vec::with_capacity(READ_SIZE);
        let mut chunk = vec![0u8; READ_SIZE];
        let mut idle = 0u32;
        let mut gave_up = Ok(());
        loop {
            if stop.load(Ordering::SeqCst) {
                break;
            }
            let n = port.read(&mut chunk, Duration::from_millis(10))?;
            if n == 0 {
                idle += 1;
                if idle > 500 {
                    log::warn!("Device stopped sending data.");
                    // With the limit met it is only the closing count missing.
                    if !(limit > 0 && sent >= limit) {
                        gave_up = Err(Error::device(format!(
                            "the device stopped sending data after {sent} samples"
                        )));
                    }
                    break;
                }
                continue;
            }
            idle = 0;
            buf.extend_from_slice(&chunk[..n]);
            if dec.end == StreamEnd::Active {
                let mut out = Slices::default();
                let used = dec.decode(&buf, &mut out);
                buf.drain(..used);
                if out.count > 0 {
                    let (a, b) = self.deliver(
                        sink,
                        &mut dec,
                        &mut soft,
                        &mut pre_logic,
                        &mut pre_analog,
                        pretrig,
                        &out,
                        sent,
                    );
                    sent += a;
                    if !b {
                        break;
                    }
                }
                if dec.end == StreamEnd::Aborted {
                    return Err(Error::device("the sample stream was aborted by the device"));
                }
                if limit > 0 && sent >= limit && dec.end == StreamEnd::Active && !stop_sent {
                    log::debug!("Ending: sent {sent} of {limit} samples, {} bytes", dec.byte_count);
                    port.write(b"+")?;
                    stop_sent = true;
                }
            }
            if dec.end == StreamEnd::Stopped {
                // `$`, the decimal byte count, `+`.
                let Some(start) = buf.iter().position(|b| *b == b'$') else {
                    buf.clear();
                    continue;
                };
                dec.tail.extend_from_slice(&buf[start + 1..]);
                buf.clear();
                if let Some(plus) = dec.tail.iter().position(|b| *b == b'+') {
                    let text = String::from_utf8_lossy(&dec.tail[..plus]);
                    let count: u64 = text.trim().parse().unwrap_or(0);
                    if count != dec.byte_count {
                        log::error!(
                            "received {count} and counted {} bytes don't match, data may be lost",
                            dec.byte_count
                        );
                    }
                    break;
                }
            }
        }
        if dec.end != StreamEnd::Stopped {
            port.write(b"+")?;
        }
        drain(port)?;
        log::debug!("Sent {sent} samples.");
        gave_up
    }

    /// Sends decoded samples to the session, through the software
    /// trigger when one is armed. Returns the samples counted and
    /// whether to go on.
    #[allow(clippy::too_many_arguments)]
    fn deliver(
        &self,
        sink: &PacketSink,
        dec: &mut Decoder,
        soft: &mut Option<SoftTrigger>,
        pre_logic: &mut PreTriggerBuffer,
        pre_analog: &mut [VecDeque<f32>],
        pretrig: usize,
        out: &Slices,
        sent: u64,
    ) -> (u64, bool) {
        let unit = dec.dig_sample_bytes.max(1);
        let limit = self.settings.limit_samples;
        let mut from = 0usize;
        let mut total = 0u64;
        if let Some(st) = soft.as_mut() {
            match st.feed(&out.logic) {
                None => {
                    pre_logic.push(&out.logic);
                    for (i, a) in out.analog.iter().enumerate() {
                        if dec.a_mask >> i & 1 != 0 {
                            let ring = &mut pre_analog[i];
                            ring.extend(a.iter().copied());
                            while ring.len() > pretrig {
                                ring.pop_front();
                            }
                        }
                    }
                    return (0, true);
                }
                Some(at) => {
                    // The pre-trigger window: buffered samples plus this
                    // chunk's head, cut to the window.
                    let mut before = pre_logic.take();
                    before.extend_from_slice(&out.logic[..at * unit]);
                    let keep = pretrig * unit;
                    if before.len() > keep {
                        before.drain(..before.len() - keep);
                    }
                    let n_before = before.len() / unit;
                    let mut analog_before: Vec<Vec<f32>> = vec![Vec::new(); dec.num_a];
                    for (i, a) in out.analog.iter().enumerate() {
                        if dec.a_mask >> i & 1 != 0 {
                            let mut v: Vec<f32> = pre_analog[i].iter().copied().collect();
                            v.extend_from_slice(&a[..at]);
                            if v.len() > n_before {
                                v.drain(..v.len() - n_before);
                            }
                            analog_before[i] = v;
                        }
                    }
                    if n_before > 0 {
                        Self::emit(sink, dec, before, &analog_before, sent, limit);
                        total += n_before as u64;
                    }
                    if !sink.send(Packet::Trigger) {
                        return (total, false);
                    }
                    from = at;
                    *soft = None;
                }
            }
        }
        let logic = out.logic[from * unit..].to_vec();
        let analog: Vec<Vec<f32>> = out
            .analog
            .iter()
            .map(|a| if a.len() > from { a[from..].to_vec() } else { Vec::new() })
            .collect();
        let n = out.count - from;
        let ok = Self::emit(sink, dec, logic, &analog, sent + total, limit);
        total += n as u64;
        (total, ok)
    }

    /// Sends logic and analog packets for the same sample range, cut at
    /// the sample limit.
    fn emit(
        sink: &PacketSink,
        dec: &Decoder,
        mut logic: Vec<u8>,
        analog: &[Vec<f32>],
        sent: u64,
        limit: u64,
    ) -> bool {
        let unit = dec.dig_sample_bytes.max(1);
        let mut count = logic.len() / unit;
        if limit > 0 {
            let room = (limit - sent.min(limit)) as usize;
            count = count.min(room);
            logic.truncate(count * unit);
        }
        if count == 0 {
            return true;
        }
        if dec.num_d > 0 && !sink.send(Packet::logic(unit, logic)) {
            return false;
        }
        for (i, a) in analog.iter().enumerate() {
            if dec.a_mask >> i & 1 == 0 {
                continue;
            }
            let samples = a[..count.min(a.len())].to_vec();
            let data = AnalogData {
                channels: vec![i],
                samples,
                mq: Mq::Voltage,
                flags: MqFlags::empty(),
                unit: Unit::Volt,
                digits: ANALOG_DIGITS,
            };
            if !sink.send(Packet::Analog(data)) {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_parses() {
        let id = parse_identity("SRPICO,A031D21,02").unwrap();
        assert_eq!(id, Identity { analog: 3, analog_size: 1, digital: 21 });
        assert!(parse_identity("SRPICO,A031D21,01").is_none());
        assert!(parse_identity("SRPICO,A001D00,02").is_none());
        assert!(parse_identity("SRPICO,A095D21,02").is_none());
        assert!(parse_identity("SRPIC").is_none());
    }

    fn decoder(num_a: usize, num_d: usize, a_mask: u32, d_mask: u32) -> Decoder {
        let mut bps = num_a.min(a_mask.count_ones() as usize);
        for i in (0..num_d).step_by(7) {
            if d_mask >> i & 0x7f != 0 {
                bps += 1;
            }
        }
        Decoder {
            num_a,
            num_d,
            a_size: 1,
            a_mask,
            d_mask,
            bytes_per_slice: bps,
            dig_sample_bytes: num_d.div_ceil(8),
            a_scale: vec![(0.0257, 0.0); num_a],
            d_last: [0; 4],
            a_last: vec![0.0; num_a],
            byte_count: 0,
            end: StreamEnd::Active,
            tail: Vec::new(),
        }
    }

    #[test]
    fn d4_stream_from_the_device() {
        // What a Pico sent for D2 only with L8: eight zero samples, a run
        // of eight more, a sample byte carrying seven repeats and a zero,
        // then the stop marker and the byte count.
        let stream = b"\x80\x80\x80\x80\x80\x80\x80\x800\xf0$10+\n";
        let mut d = decoder(3, 21, 0, 0x1);
        assert!(d.d4_mode());
        let mut out = Slices::default();
        let used = d.decode(stream, &mut out);
        assert_eq!(used, 10);
        assert_eq!(d.end, StreamEnd::Stopped);
        assert_eq!(d.byte_count, 10);
        assert_eq!(out.count, 8 + 8 + 7 + 1);
        assert_eq!(out.logic.len(), out.count * 3);
        assert!(out.logic.iter().all(|b| *b == 0));
        // A set bit on D3 with two repeats.
        let mut d = decoder(3, 21, 0, 0x3);
        let mut out = Slices::default();
        d.decode(b"\xa2", &mut out);
        assert_eq!(out.count, 3);
        assert_eq!(&out.logic[..3], &[0, 0, 0]);
        assert_eq!(&out.logic[6..9], &[2, 0, 0]);
    }

    #[test]
    fn slice_stream_with_analog_and_runs() {
        // One analog channel and D2..D8 (seven digital): a slice is one
        // digital byte then one analog byte.
        let mut d = decoder(3, 21, 0x1, 0x7f);
        assert!(!d.d4_mode());
        assert_eq!(d.bytes_per_slice, 2);
        let mut out = Slices::default();
        // Slice: digital 0x05, analog 0x80 + 100; then a run of 3
        // (byte 50); then a partial slice that must wait.
        let used = d.decode(&[0x85, 0x80 + 100, 50, 0x81], &mut out);
        assert_eq!(used, 3);
        assert_eq!(out.count, 4);
        assert_eq!(out.logic, vec![5, 0, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0]);
        assert_eq!(out.analog[0].len(), 4);
        assert!((out.analog[0][0] - 2.57).abs() < 1e-4);
        assert_eq!(out.analog[1].len(), 0);
        // The stop marker ends decoding.
        let mut out = Slices::default();
        let used = d.decode(&[0x81, 0x80, b'$', b'5', b'+'], &mut out);
        assert_eq!(used, 2);
        assert_eq!(d.end, StreamEnd::Stopped);
        assert_eq!(out.count, 1);
    }
}
