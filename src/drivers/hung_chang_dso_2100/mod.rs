// Counterpart in sigrok:
//   libsigrok/src/hardware/hung-chang-dso-2100/api.c,
//   libsigrok/src/hardware/hung-chang-dso-2100/protocol.c,
//   Copyright (C) 2015 Daniel Glöckner <daniel-gl@gmx.net>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Gerhard Sittig, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Hung-Chang DSO-2100, a two channel oscilloscope on a PC's parallel
//! port.
//!
//! The port is Linux's `ppdev` (`crate::parport`), driven with the calls
//! libieee1284 makes, so a USB cable the kernel's `uss720` serves works as
//! a built-in port does. `conn=` names the port as `/proc/sys/dev/parport`
//! does (`parport0`) and is required. The protocol is in [`protocol`].
//!
//! A capture: the settings go down the microcontroller's chain, the scope
//! is armed, then polled every 8 ms (`0x99`) until it says it triggered.
//! A frame is the readout of each step up to the buffer size's: 1000
//! samples from the first, 500 (or 2 x 499, 2 x 500 or 2 x 120 from both
//! ADCs in turn) from each after it, each followed by a trip round the
//! chain and a new extraction. One channel at a time, as the scope has.
//! A frame limit of 0 is one frame, as in C.
//!
//! # Waiting
//!
//! The microcontroller answers its mailbox at once (C waits 0.25 ms for
//! the trigger poll's answer and 100 ms in `move_to`); a scope waiting for
//! its trigger answers "armed", which is not silence. So 600 trigger polls
//! in a row whose mailbox reads 0, 8 ms apart and so at least 4.8 s, end
//! the capture ("No answer from the DSO-2100 in 600 trigger polls."),
//! whether or not a frame came; C reads on for ever. The bound counts polls
//! rather than time, so the transactions of a silent scope are the same
//! from one run to the next.
//! A port that went away (a USB cable pulled) is seen at the first silent
//! read: the kernel's dead port reads 0 for ever, and C waits out each of
//! `move_to`'s 40 steps of 100 ms, or polls for ever.
//!
//! # Against C
//!
//! - C hands each block of samples on with a count of 4294967295
//!   (`push_samples` runs `num` down to `(size_t)-1` before
//!   `analog.num_samples = num`, since its commit 99f8fa25 of 2016), so no
//!   output module gets a sample: `analog` fails its allocation and prints
//!   nothing, at exit 0, and `csv` aborts. Here the count is the samples'.
//! - A readout loop that fails to get back to the scope's states (a
//!   `move_to` that times out, an extraction not answered with `0x03`),
//!   a fourth bad signature ("Failed to read frame without transfer
//!   errors"), a re-arm that fails, and a state the trigger poll does not
//!   know end the capture with an error. C ends the frame with what it has
//!   and goes on, or (the last) drops its poll and leaves the session
//!   without an end.
//! - A port write that fails is an error; libieee1284 ignores it.

#![deny(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::parport::{Parport, PortInfo};
use protocol::{MAX_RETRIES, MoveError, Port, READOUT, STEPS, Settings, state};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// The driver id.
pub const DRIVER_ID: &str = "hung-chang-dso-2100";

static CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
static SCAN_OPTIONS: [Key; 1] = [Key::Conn];
static DEV_OPTIONS: [OptionDesc; 6] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
    OptionDesc::new(Key::Buffersize, Access::GET_SET_LIST),
];
static GROUP_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
    OptionDesc::new(Key::ProbeFactor, Access::GET_SET),
];

/// `samplerates[]`.
pub const SAMPLERATES: [u64; 28] = [
    100_000_000,
    50_000_000,
    25_000_000,
    20_000_000,
    10_000_000,
    5_000_000,
    2_500_000,
    2_000_000,
    1_000_000,
    500_000,
    250_000,
    200_000,
    100_000,
    50_000,
    25_000,
    20_000,
    10_000,
    5_000,
    2_500,
    2_000,
    1_000,
    500,
    250,
    200,
    100,
    50,
    25,
    20,
];

/// `buffersizes[]`, one per readout step it reads up to.
pub const BUFFERSIZES: [u64; 15] =
    [1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5998, 6998, 7998, 8998, 9998, 10238];

/// `vdivs[]`.
pub const VDIVS: [(u64, u64); 9] = [
    (10, 1000),
    (20, 1000),
    (50, 1000),
    (100, 1000),
    (200, 1000),
    (500, 1000),
    (1, 1),
    (2, 1),
    (5, 1),
];
/// `vdivs_map[]`: bits 4 and 5 the /10 relays, 0 and 1 the resistor array.
const VDIV_CODES: [u8; 9] = [0x01, 0x02, 0x03, 0x21, 0x22, 0x23, 0x31, 0x32, 0x33];
/// `trigger_sources[]`, also the channel and group names.
pub const TRIGGER_SOURCES: [&str; 3] = ["A", "B", "EXT"];
const TRIGGER_SOURCE_CODES: [u8; 3] = [0x00, 0x80, 0x40];
/// `trigger_slopes[]`.
pub const TRIGGER_SLOPES: [&str; 2] = ["f", "r"];
/// `coupling[]`.
pub const COUPLINGS: [&str; 3] = ["DC", "AC", "GND"];
const COUPLING_CODES: [u8; 3] = [0x00, 0x08, 0x04];

/// The sample buffer's interleaved readouts need both halves.
const BUFFER: usize = READOUT;

/// How long a capture's waits are: tests shorten them.
#[derive(Clone, Copy, Debug)]
struct Times {
    /// The trigger poll's period, the session source's 8 ms.
    poll: Duration,
    /// Trigger polls in a row whose mailbox reads 0 that end a capture.
    silent_polls: u32,
}

const TIMES: Times = Times { poll: Duration::from_millis(8), silent_polls: 600 };

/// The driver.
pub struct HungChangDso2100Driver;

impl Driver for HungChangDso2100Driver {
    fn id(&self) -> &'static str {
        DRIVER_ID
    }

    fn long_name(&self) -> &'static str {
        "Hung-Chang DSO-2100"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else { return Ok(Vec::new()) };
        let ports = crate::parport::find_ports();
        let mut found: Vec<Box<dyn Device>> = Vec::new();
        let mut any = false;
        for port in ports.iter().filter(|p| p.name == conn) {
            any = true;
            if scan_port(port) {
                found.push(Box::new(Dso2100::new(port.clone())));
            }
        }
        if !any {
            log::error!("Parallel port {conn} not found. Valid names are:");
            for p in &ports {
                log::error!("\t{}", p.name);
            }
        }
        Ok(found)
    }
}

/// `scan_port`: whether a scope answers on the port.
fn scan_port(info: &PortInfo) -> bool {
    let Ok((port, caps)) = Parport::open(info) else {
        log::error!("Can't open parallel port {}", info.name);
        return false;
    };
    if !(caps.raw && caps.byte) {
        log::error!("Parallel port {} does not provide low-level bidirection access", info.name);
        return false;
    }
    if port.claim().is_err() {
        log::error!("Parallel port {} already in use", info.name);
        return false;
    }
    let mut port = port;
    let found = protocol::check_id(&mut port).unwrap_or(false);
    port.release();
    found
}

/// The settings `-c` sets, beyond the protocol's.
#[derive(Clone, Debug)]
struct Config {
    proto: Settings,
    /// `last_step`: the buffer size's readout step.
    last_step: usize,
    probe: [u64; 2],
    frame_limit: u64,
    /// The channel last enabled, which packets name (`enabled_channel`).
    enabled_channel: Option<usize>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            proto: Settings::default(),
            last_step: 0,
            probe: [10, 10],
            frame_limit: 0,
            enabled_channel: None,
        }
    }
}

/// The port, open and claimed.
struct Link {
    port: Box<dyn Port>,
}

impl Link {
    /// `dev_close`'s: reset, released, and closed as it drops.
    fn close(mut self) {
        let _ = protocol::reset_port(self.port.as_mut());
        self.port.release();
    }
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// One scope.
pub struct Dso2100 {
    info: DeviceInfo,
    port_info: PortInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    config: Config,
    times: Times,
    link: Arc<Mutex<Option<Link>>>,
    worker: Option<Worker>,
}

impl Dso2100 {
    /// A scope found on `port`.
    pub fn new(port: PortInfo) -> Dso2100 {
        let channels = (0..2)
            .map(|i| {
                let mut c = Channel::new(i, ChannelKind::Analog, TRIGGER_SOURCES[i]);
                c.enabled = false;
                c
            })
            .collect();
        Dso2100 {
            info: DeviceInfo {
                driver: DRIVER_ID,
                vendor: "Hung-Chang".to_owned(),
                model: "DSO-2100".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: Some(port.name.clone()),
            },
            port_info: port,
            channels,
            groups: vec![ChannelGroup::new("A", vec![0]), ChannelGroup::new("B", vec![1])],
            config: Config::default(),
            times: TIMES,
            link: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    /// A scope on a port of the caller's, open: for tests.
    #[cfg(test)]
    fn on_port(mut port: Box<dyn Port>, times: Times) -> Dso2100 {
        // As the open leaves it.
        port.data_dir(true).expect("data_dir");
        let mut d = Dso2100::new(PortInfo { name: "parport0".into(), device: "/dev/null".into() });
        d.times = times;
        *d.link.lock().expect("link lock") = Some(Link { port });
        d
    }

    fn group(&self, group: Option<&str>) -> Result<Option<usize>> {
        match group {
            None => Ok(None),
            Some(g) => self
                .groups
                .iter()
                .position(|cg| cg.name == g)
                .map(Some)
                .ok_or_else(|| Error::ChannelGroup(g.to_owned())),
        }
    }

    /// `config_commit`, then `dev_acquisition_start`'s arming.
    fn arm(&mut self, link: &mut Link) -> Result<()> {
        let port = link.port.as_mut();
        let s = &self.config.proto;
        let mut target = protocol::read_mbox(port, protocol::STATE_WAIT)?;
        match target {
            // The chain is on the way back to idle.
            state::EXTRACTED | state::SCROLL | state::ARMED => {}
            state::IDLE => protocol::write_mbox(port, 4)?,
            other => {
                if other == 0 {
                    target = state::IDLE;
                }
                protocol::move_to(port, state::IDLE, s).map_err(|e| e.into_error(state::IDLE))?;
                protocol::write_mbox(port, 4)?;
            }
        }
        protocol::move_to(port, state::IDLE, s).map_err(|e| e.into_error(state::IDLE))?;
        protocol::move_to(port, target, s).map_err(|e| e.into_error(target))?;
        // dev_acquisition_start.
        let mut armed = self.config.proto.clone();
        armed.step = 0;
        armed.adc2 = false;
        protocol::move_to(link.port.as_mut(), state::ARMED, &armed)
            .map_err(|e| e.into_error(state::ARMED))
    }
}

fn not_applicable() -> Error {
    Error::not_supported("not applicable")
}

impl Device for Dso2100 {
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

    /// `config_channel_set`: one channel at a time.
    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        if pos >= self.channels.len() {
            return Err(Error::arg("no such channel"));
        }
        let bit = 1u8 << pos;
        if enabled {
            let v = self.config.proto.channel_bits() | bit;
            if v & (v - 1) != 0 {
                return Err(Error::arg("The DSO-2100 takes one channel at a time."));
            }
            self.config.proto.channel = v.trailing_zeros() as u8 + 1;
            self.config.enabled_channel = Some(pos);
        } else if self.config.proto.channel_bits() & bit != 0 {
            self.config.proto.channel = 0;
        }
        self.channels[pos].enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(match self.group(group)? {
            None => DEV_OPTIONS.to_vec(),
            Some(_) => GROUP_OPTIONS.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let c = &self.config;
        let s = &c.proto;
        Ok(match (self.group(group)?, key) {
            (None, Key::Conn) => Value::Str(self.port_info.name.clone()),
            (None, Key::LimitFrames) => Value::U64(c.frame_limit),
            (None, Key::Samplerate) => Value::U64(SAMPLERATES[usize::from(s.rate)]),
            (None, Key::TriggerSource) => {
                let i = TRIGGER_SOURCE_CODES
                    .iter()
                    .position(|&code| code == s.cctl[0] & 0xc0)
                    .ok_or_else(|| Error::device("trigger source bits out of the table"))?;
                Value::Str(TRIGGER_SOURCES[i].to_owned())
            }
            (None, Key::TriggerSlope) => Value::Str(TRIGGER_SLOPES[usize::from(s.edge)].to_owned()),
            (None, Key::Buffersize) => Value::U64(BUFFERSIZES[c.last_step]),
            (Some(ch), Key::Vdiv) => {
                let i = VDIV_CODES
                    .iter()
                    .position(|&code| code == s.cctl[ch] & 0x33)
                    .ok_or_else(|| Error::device("vdiv bits out of the table"))?;
                Value::Rational(VDIVS[i].0, VDIVS[i].1)
            }
            (Some(ch), Key::Coupling) => {
                let i = COUPLING_CODES
                    .iter()
                    .position(|&code| code == s.cctl[ch] & 0x0c)
                    .ok_or_else(|| Error::device("coupling bits out of the table"))?;
                Value::Str(COUPLINGS[i].to_owned())
            }
            (Some(ch), Key::ProbeFactor) => Value::U64(c.probe[ch]),
            (None, Key::Vdiv | Key::Coupling | Key::ProbeFactor) => {
                return Err(Error::ChannelGroup(String::new()));
            }
            (Some(_), _) => return Err(not_applicable()),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let str_index = |list: &[&str]| {
            let v = value.as_str().ok_or_else(bad)?;
            list.iter().position(|s| *s == v).ok_or_else(bad)
        };
        let group = self.group(group)?;
        let c = &mut self.config;
        match (group, key) {
            (None, Key::LimitFrames) => c.frame_limit = value.as_u64().ok_or_else(bad)?,
            (None, Key::Samplerate) => {
                let v = value.as_u64().ok_or_else(bad)?;
                c.proto.rate = SAMPLERATES.iter().position(|&r| r == v).ok_or_else(bad)? as u8;
            }
            (None, Key::TriggerSource) => {
                let i = str_index(&TRIGGER_SOURCES)?;
                c.proto.cctl[0] = (c.proto.cctl[0] & 0x3f) | TRIGGER_SOURCE_CODES[i];
            }
            (None, Key::TriggerSlope) => c.proto.edge = str_index(&TRIGGER_SLOPES)? as u8,
            (None, Key::Buffersize) => {
                let v = value.as_u64().ok_or_else(bad)?;
                c.last_step = BUFFERSIZES.iter().position(|&b| b == v).ok_or_else(bad)?;
            }
            (Some(ch), Key::Vdiv) => {
                let Value::Rational(p, q) = value else { return Err(bad()) };
                // The parser reduces its fractions: 10 mV comes as 1/100.
                let i = VDIVS.iter().position(|(a, b)| a * q == p * b).ok_or_else(bad)?;
                c.proto.cctl[ch] = (c.proto.cctl[ch] & 0xcc) | VDIV_CODES[i];
            }
            (Some(ch), Key::Coupling) => {
                let i = str_index(&COUPLINGS)?;
                c.proto.cctl[ch] = (c.proto.cctl[ch] & 0xf3) | COUPLING_CODES[i];
            }
            (Some(ch), Key::ProbeFactor) => {
                let v = value.as_u64().ok_or_else(bad)?;
                if v == 0 {
                    return Err(bad());
                }
                c.probe[ch] = v;
            }
            (None, Key::Vdiv | Key::Coupling | Key::ProbeFactor) => {
                return Err(Error::ChannelGroup(String::new()));
            }
            (Some(_), _) => return Err(not_applicable()),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let strs = |list: &[&str]| ValueList::Str(list.iter().map(|s| (*s).to_owned()).collect());
        Ok(match (self.group(group)?, key) {
            (None, Key::Samplerate) => ValueList::U64(SAMPLERATES.to_vec()),
            (None, Key::TriggerSource) => strs(&TRIGGER_SOURCES),
            (None, Key::TriggerSlope) => strs(&TRIGGER_SLOPES),
            (None, Key::Buffersize) => ValueList::U64(BUFFERSIZES.to_vec()),
            (Some(_), Key::Vdiv) => ValueList::Rational(VDIVS.to_vec()),
            (Some(_), Key::Coupling) => strs(&COUPLINGS),
            (None, Key::Vdiv | Key::Coupling) => return Err(Error::ChannelGroup(String::new())),
            (Some(_), _) => return Err(not_applicable()),
            (None, other) => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    /// `dev_open`: the port claimed, the data lines read, the scope idle.
    fn open(&mut self) -> Result<()> {
        if self.link.lock().expect("link lock").is_some() || self.worker.is_some() {
            return Ok(());
        }
        let (parport, _) = Parport::open(&self.port_info)?;
        parport.claim()?;
        let mut port: Box<dyn Port> = Box::new(parport);
        let opened = port.data_dir(true).and_then(|()| {
            protocol::move_to(port.as_mut(), state::IDLE, &self.config.proto)
                .map_err(|e| e.into_error(state::IDLE))
        });
        if let Err(e) = opened {
            Link { port }.close();
            return Err(e);
        }
        *self.link.lock().expect("link lock") = Some(Link { port });
        Ok(())
    }

    /// `dev_close`: the port reset, released and closed.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(w) = self.worker.take() {
            let _ = w.handle.join();
        }
        if let Some(link) = self.link.lock().expect("link lock").take() {
            link.close();
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        match self.worker.take() {
            Some(w) if w.handle.is_finished() => {
                let _ = w.handle.join();
            }
            Some(w) => {
                self.worker = Some(w);
                return Err(Error::Busy);
            }
            None => {}
        }
        let Some(mut link) = self.link.lock().expect("link lock").take() else {
            return Err(Error::device("device is not open"));
        };
        if let Err(e) = self.arm(&mut link) {
            *self.link.lock().expect("link lock") = Some(link);
            return Err(e);
        }
        let cfg = self.config.clone();
        let factor = match cfg.proto.channel {
            0 => 0.0,
            ch => {
                let i = usize::from(ch - 1);
                protocol::factor(cfg.probe[i], cfg.proto.cctl[i])
            }
        };
        let times = self.times;
        let slot = Arc::clone(&self.link);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("hung-chang-dso-2100 acquisition".into())
            .spawn(move || {
                let go_on = || !stop2.load(Ordering::SeqCst);
                let mut capture = Capture::new(cfg, factor, times, &sink, &stop2);
                let outcome = capture.run(link.port.as_mut());
                // dev_acquisition_stop: back to idle, whatever happened.
                let idle = protocol::move_to(link.port.as_mut(), state::IDLE, &capture.settings)
                    .map_err(|e| e.into_error(state::IDLE));
                let outcome = match (outcome, idle) {
                    (Err(e), _) => Err(e),
                    // After a stop or a failure the way back is best effort, as
                    // in C; after a frame limit too.
                    (Ok(()), Err(e)) => {
                        log::warn!("{e}");
                        Ok(())
                    }
                    (Ok(()), Ok(())) => Ok(()),
                };
                *slot.lock().expect("link lock") = Some(link);
                sink.end_while(outcome, go_on);
            })
            .map_err(Error::Io)?;
        self.worker = Some(Worker { stop, handle });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        if let Some(w) = &self.worker {
            w.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Settings {
    /// `devc->channel` as C keeps it, a bit per channel.
    fn channel_bits(&self) -> u8 {
        match self.channel {
            0 => 0,
            c => 1 << (c - 1),
        }
    }
}

/// One capture's state, `hung_chang_dso_2100_poll`'s.
struct Capture<'a> {
    cfg: Config,
    settings: Settings,
    factor: f32,
    times: Times,
    sink: &'a PacketSink,
    stop: &'a AtomicBool,
    retries: u8,
    frame: u64,
}

/// Why a capture's loop ended early.
enum Halt {
    /// Nobody takes packets any more, or a stop came: no error.
    Gone,
    Failed(Error),
}

impl From<Error> for Halt {
    fn from(e: Error) -> Halt {
        Halt::Failed(e)
    }
}

impl<'a> Capture<'a> {
    fn new(
        cfg: Config,
        factor: f32,
        times: Times,
        sink: &'a PacketSink,
        stop: &'a AtomicBool,
    ) -> Capture<'a> {
        let mut settings = cfg.proto.clone();
        settings.step = 0;
        settings.adc2 = false;
        Capture { cfg, settings, factor, times, sink, stop, retries: MAX_RETRIES, frame: 0 }
    }

    fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    fn send(&self, packet: Packet) -> std::result::Result<(), Halt> {
        if self.sink.send_while(packet, || !self.stopped()) { Ok(()) } else { Err(Halt::Gone) }
    }

    fn run(&mut self, port: &mut dyn Port) -> Result<()> {
        match self.poll(port) {
            Ok(()) | Err(Halt::Gone) => Ok(()),
            Err(Halt::Failed(e)) => Err(e),
        }
    }

    /// The trigger poll, frame after frame, until the limit or a stop.
    fn poll(&mut self, port: &mut dyn Port) -> std::result::Result<(), Halt> {
        self.send(Packet::header())?;
        let mut state_known = true;
        let mut silent: u32 = 0;
        loop {
            if self.stopped() {
                return Ok(());
            }
            if state_known {
                protocol::write_mbox(port, 0x99)?;
            }
            let state = protocol::read_mbox(port, protocol::POLL_WAIT)?;
            state_known = state != 0;
            if !state_known {
                if port.gone() {
                    return Err(Halt::Failed(Error::Disconnected));
                }
                silent += 1;
                if silent >= self.times.silent_polls {
                    return Err(Halt::Failed(Error::device(format!(
                        "No answer from the DSO-2100 in {silent} trigger polls."
                    ))));
                }
                std::thread::sleep(self.times.poll);
                continue;
            }
            silent = 0;
            if state == state::ARMED {
                std::thread::sleep(self.times.poll);
                continue;
            }
            if state != state::EXTRACTED {
                log::error!("Unexpected state 0x{state:X} while checking for trigger");
                return Err(Halt::Failed(Error::device(format!(
                    "Unexpected state 0x{state:X} while checking for trigger"
                ))));
            }
            self.send(Packet::FrameBegin)?;
            if self.settings.channel != 0 {
                self.read_frame(port)?;
            }
            self.send(Packet::FrameEnd)?;
            self.frame += 1;
            if self.frame >= self.cfg.frame_limit {
                return Ok(());
            }
            protocol::move_to(port, state::ARMED, &self.settings)
                .map_err(|e| Halt::Failed(e.into_error(state::ARMED)))?;
            state_known = true;
        }
    }

    /// The readout loop of a frame.
    fn read_frame(&mut self, port: &mut dyn Port) -> std::result::Result<(), Halt> {
        let mut buf = [0u8; BUFFER];
        while self.read_subframe(port, &mut buf)? {
            protocol::move_to(port, state::IDLE, &self.settings)
                .map_err(|e| Halt::Failed(e.into_error(state::IDLE)))?;
            protocol::write_mbox(port, state::EXTRACTED)?;
            std::thread::sleep(protocol::EXTRACT_WAIT);
            let s = protocol::read_mbox(port, protocol::STATE_WAIT)?;
            if s != state::EXTRACTED {
                if s == 0 && port.gone() {
                    return Err(Halt::Failed(Error::Disconnected));
                }
                return Err(Halt::Failed(Error::device(format!(
                    "The DSO-2100 answered 0x{s:02X} to the extraction of a readout, not 0x03."
                ))));
            }
        }
        Ok(())
    }

    /// `read_subframe`: one readout. Whether another follows.
    fn read_subframe(
        &mut self,
        port: &mut dyn Port,
        buf: &mut [u8; READOUT],
    ) -> std::result::Result<bool, Halt> {
        let step = STEPS[self.settings.step];
        let good = protocol::read_readout(port, &self.settings, buf)?;
        if !good {
            self.retries -= 1;
            if self.retries > 0 {
                log::debug!("Missing signature at end of buffer, {} tries remaining", self.retries);
                return Ok(true);
            }
            log::error!("Failed to read frame without transfer errors");
            if port.gone() {
                return Err(Halt::Failed(Error::Disconnected));
            }
            return Err(Halt::Failed(Error::device(
                "Failed to read frame without transfer errors",
            )));
        }
        if step.interleave && !self.settings.adc2 {
            self.settings.adc2 = true;
            self.retries = MAX_RETRIES;
            return Ok(true);
        }
        let mut num = usize::from(step.num);
        if step.interleave {
            num *= 2;
        }
        let mut at = 0;
        if self.settings.step == 0 {
            self.push(&buf[..6])?;
            self.send(Packet::Trigger)?;
            at = 6;
        }
        self.push(&buf[at..num])?;
        self.settings.step += 1;
        if self.settings.step > self.cfg.last_step {
            self.settings.step = 0;
        }
        self.settings.adc2 = false;
        self.retries = MAX_RETRIES;
        Ok(self.settings.step > 0)
    }

    /// `push_samples`.
    fn push(&self, buf: &[u8]) -> std::result::Result<(), Halt> {
        let channel = self.cfg.enabled_channel.unwrap_or(0);
        self.send(Packet::Analog(AnalogData {
            channels: vec![channel],
            samples: protocol::to_volts(buf, self.factor),
            mq: Mq::Voltage,
            flags: MqFlags::empty(),
            unit: Unit::Volt,
            digits: protocol::digits(self.factor),
        }))
    }
}

impl From<MoveError> for Halt {
    fn from(e: MoveError) -> Halt {
        Halt::Failed(e.into_error(0))
    }
}
