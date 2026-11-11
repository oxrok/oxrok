// Counterpart in sigrok:
//   libsigrok/src/hardware/lecroy-xstream/api.c,
//   libsigrok/src/hardware/lecroy-xstream/protocol.c,
//   libsigrok/src/hardware/lecroy-xstream/protocol.h,
//   Copyright (C) 2017 Sven Schnelle <svens@stackframe.org>,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   with commits by Uwe Hermann, Soeren Apel, Gerhard Sittig,
//   Sylvain Pelissier, Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `LeCroy` X-Stream oscilloscopes over SCPI: the `lecroy-xstream` driver.
//!
//! One default configuration for every scope that says `LECROY`: four channels, ten
//! divisions by eight. Open reads the scope's settings. A capture asks each enabled
//! channel for `Cn:WAVEFORM?`; a block with no samples means no acquisition yet, so it
//! asks again with `ARM;WAIT;*OPC` ahead, and between frames it sends that and asks.
//!
//! [`protocol`] says what the answers mean, and where this reads them otherwise than
//! libsigrok does. The X-Stream manual is WM-RCM-E Rev C (May 2004); pages are its
//! printed ones. More differences, of the open and of a capture:
//!
//! - `protocol.c:439` frees the answer to `TRIG_SELECT?` and goes on using the source
//!   in it. That free came with 36165cf0 (2021, made with no scope at hand); before it
//!   the answer was never freed, which is how the driver worked for its authors. The
//!   answer is a 1024 byte `GString`, too large for the tcache, so the free puts it in
//!   the unsorted bin and writes its first 32 bytes (two list pointers and the large
//!   chunk's two size links). A source within those, as `EDGE,SR,C1,HT,OFF` has at byte
//!   8, is lost, and libsigrok opens no such scope. Here the answer is read whole.
//! - The manual's trigger sources `EX`, `EX10`, `PA` and `ETM10` (page 230) open here,
//!   and a get answers them. C's list (`EXT`, no `EX`) is kept for what can be set.
//! - C retries a failed or a bad block for ever (`receive_data` returns `TRUE`, and
//!   `SR_ERR` is true to a source as well) and on TCP its `recv` blocks. Here a capture
//!   ends with an error, and [`protocol::Silence`] bounds what can be bounded.
//! - A block whose `WAVE_SOURCE` (page 274) names another channel than the one asked
//!   for ends a capture: it is an answer out of step.
//! - A stop with the scope armed (`ARM;WAIT`) leaves an answer to come. Answers come
//!   in order, and `WAIT` holds the commands after it, so the next use of the
//!   connection sends `*OPC?` and reads until it says `1`, dropping what came first.
//!   A capture waits for that until its stop; a setting or an open for five seconds.
//!
//! `TRIG POS 1.0e-3 S`, which `horiz_triggerpos` sends, is C's and in neither manual.
//! The 2004 manual has `TRIG_DELAY` (page 224), with 0 at the centre on X-Stream as
//! here, but positive values in percent and negative in seconds where C's are seconds
//! either way, so they are not the same setting, and C's command is kept.
//!
//! `conn=` takes a SCPI resource; without it every USBTMC device is asked `*IDN?`.
//! USBTMC is the real path. The 2004 manual's network interface is VICP (page 38), and
//! no documented interface is plain SCPI on a socket, so `tcp-raw` here is unverified
//! with a real scope.
//!
//! What a setting does when a scope doesn't confirm it: the setting is an error, the
//! state keeps the old value, and a warning says the scope may have taken it.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, MqFlags, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::scpi::Scpi;
use protocol::{Framer, Framing, Silence, Verdict};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
static DEV_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET),
    OptionDesc::new(Key::Timebase, Access::GET_SET_LIST),
    OptionDesc::new(Key::NumHdiv, Access::GET),
    OptionDesc::new(Key::HorizTriggerpos, Access::GET_SET),
    OptionDesc::new(Key::TriggerSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerSlope, Access::GET_SET_LIST),
];
static ANALOG_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::NumVdiv, Access::GET),
    OptionDesc::new(Key::Vdiv, Access::GET_SET_LIST),
    OptionDesc::new(Key::Coupling, Access::GET_SET_LIST),
];

const CHANNELS: usize = 4;
const NUM_HDIV: i32 = 10;
const NUM_VDIV: i32 = 8;
const COUPLINGS: [&str; 5] = ["A1M", "D50", "D1M", "GND", "OVL"];
const SLOPES: [&str; 2] = ["POS", "NEG"];
const SOURCES: [&str; 6] = ["C1", "C2", "C3", "C4", "LINE", "EXT"];
/// Sources the manual has beyond C's list, which an open takes.
const MANUAL_SOURCES: [&str; 4] = ["EX", "EX10", "PA", "ETM10"];
#[rustfmt::skip]
const TIMEBASES: [(u64, u64); 42] = [
    (20, 1_000_000_000_000), (50, 1_000_000_000_000), (100, 1_000_000_000_000),
    (200, 1_000_000_000_000), (500, 1_000_000_000_000),
    (1, 1_000_000_000), (2, 1_000_000_000), (5, 1_000_000_000), (10, 1_000_000_000),
    (20, 1_000_000_000), (50, 1_000_000_000), (100, 1_000_000_000), (200, 1_000_000_000),
    (500, 1_000_000_000),
    (1, 1_000_000), (2, 1_000_000), (5, 1_000_000), (10, 1_000_000), (20, 1_000_000),
    (50, 1_000_000), (100, 1_000_000), (200, 1_000_000), (500, 1_000_000),
    (1, 1000), (2, 1000), (5, 1000), (10, 1000), (20, 1000), (50, 1000), (100, 1000),
    (200, 1000), (500, 1000),
    (1, 1), (2, 1), (5, 1), (10, 1), (20, 1), (50, 1), (100, 1), (200, 1), (500, 1), (1000, 1),
];
#[rustfmt::skip]
const VDIVS: [(u64, u64); 15] = [
    (1, 1000), (2, 1000), (5, 1000), (10, 1000), (20, 1000), (50, 1000), (100, 1000),
    (200, 1000), (500, 1000),
    (1, 1), (2, 1), (5, 1), (10, 1), (20, 1), (50, 1),
];

/// `encoding->digits` of 6 binary digits, as `output/analog.c` makes them decimal.
const DIGITS: i32 = 2;
/// A read of a capture: how often it looks at its stop while a scope is quiet.
const LOOK: Duration = Duration::from_millis(100);
/// A read outside a capture's wait, as `sr_scpi_get_string` has it.
const READ_TIMEOUT: Duration = Duration::from_secs(1);
/// How long a line end may take to follow a block that ended on a read's edge.
const LINE_END: Duration = Duration::from_millis(5);
/// What a read of a block takes at a time.
const READ_CHUNK: usize = 64 * 1024;
/// How long a setting or an open waits for what a stop left armed.
const ARMED_SETTLE: Duration = Duration::from_secs(5);

/// The driver.
pub struct LecroyXstreamDriver;

impl Driver for LecroyXstreamDriver {
    fn id(&self) -> &'static str {
        "lecroy-xstream"
    }

    fn long_name(&self) -> &'static str {
        "LeCroy X-Stream"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn usb_claims(&self) -> Vec<crate::core::UsbClaim> {
        vec![crate::scpi::usbtmc::CLAIM]
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let found = crate::scpi::scan(conn.as_deref(), None, |scpi| Ok(probe(scpi)))?;
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// `probe_device`: the name, the two modes, and which channels are on.
fn probe(mut scpi: Scpi) -> Option<LecroyDevice> {
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::info!("Couldn't get IDN response: {e}");
            let _ = scpi.close();
            return None;
        }
    };
    if id.manufacturer != "LECROY" {
        log::info!("{} is no LeCroy.", id.manufacturer);
        let _ = scpi.close();
        return None;
    }
    log::debug!("Unknown LeCroy device, using default config.");
    // What fails here shows at the next question, as C doesn't look either.
    let _ = scpi.send("COMM_HEADER OFF");
    let _ = scpi.send("COMM_FORMAT DEF9,WORD,BIN");
    let mut enabled = [false; CHANNELS];
    for (i, on) in enabled.iter_mut().enumerate() {
        match scpi.get_bool(&format!("C{}:TRACE?", i + 1)) {
            Ok(value) => *on = value,
            Err(e) => {
                log::info!("No answer about channel {}: {e}", i + 1);
                let _ = scpi.close();
                return None;
            }
        }
    }
    let _ = scpi.close();
    Some(LecroyDevice::new(&id, scpi, enabled))
}

/// What the scope has, as the last open or setting left it.
#[derive(Clone, Debug, PartialEq)]
struct Settings {
    trace: [bool; CHANNELS],
    vdiv: [usize; CHANNELS],
    offset: [f32; CHANNELS],
    coupling: [usize; CHANNELS],
    timebase: usize,
    source: String,
    slope: usize,
    triggerpos: f32,
}

/// One scope.
pub struct LecroyDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    settings: Settings,
    limit_frames: u64,
    /// What the running or last capture worked out, as `state->sample_rate`.
    sample_rate: Arc<AtomicU64>,
    /// The instrument while the device is open and no capture holds it.
    scpi: Arc<Mutex<Option<Scpi>>>,
    /// The instrument while the device is closed.
    closed: Option<Scpi>,
    worker: Option<Worker>,
    /// How a capture's bounds on silence are made: [`Silence::new`] but in a test.
    silence: fn(Instant, f64) -> Silence,
    /// A stop came with the scope armed, and its answer is still to come.
    left_armed: Arc<AtomicBool>,
    /// What [`LecroyDevice::warn`] said, for the tests.
    #[cfg(test)]
    said: Mutex<Vec<String>>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

fn index_of(list: &[&str], answer: &str) -> Option<usize> {
    list.iter().position(|item| *item == answer)
}

/// `lecroy_xstream_state_get`, with C's order of questions.
fn read_settings(scpi: &mut Scpi) -> Result<Settings> {
    log::info!("Fetching scope state");
    let bad = |what: &str, answer: &str| Error::device(format!("{what}: '{answer}'"));
    let mut settings = Settings {
        trace: [false; CHANNELS],
        vdiv: [0; CHANNELS],
        offset: [0.0; CHANNELS],
        coupling: [0; CHANNELS],
        timebase: 0,
        source: String::new(),
        slope: 0,
        triggerpos: 0.0,
    };
    for i in 0..CHANNELS {
        let c = i + 1;
        settings.trace[i] = scpi.get_bool(&format!("C{c}:TRACE?"))?;
        let answer = scpi.get_string(&format!("C{c}:VDIV?"))?;
        let Some(vdiv) = protocol::table_index(&VDIVS, &answer) else {
            log::error!("Could not determine array index for vertical div scale.");
            return Err(bad("A vertical scale off the table", &answer));
        };
        settings.vdiv[i] = vdiv;
        settings.offset[i] = scpi.get_f32(&format!("C{c}:OFFSET?"))?;
        let answer = scpi.get_string(&format!("C{c}:COUPLING?"))?;
        settings.coupling[i] =
            index_of(&COUPLINGS, &answer).ok_or_else(|| bad("An unknown coupling", &answer))?;
    }
    let answer = scpi.get_string("TIME_DIV?")?;
    let Some(timebase) = protocol::table_index(&TIMEBASES, &answer) else {
        log::error!("Could not determine array index for timbase scale.");
        return Err(bad("A timebase off the table", &answer));
    };
    settings.timebase = timebase;
    let answer = scpi.get_string("TRIG_SELECT?")?;
    let source = protocol::trigger_source(&answer)
        .ok_or_else(|| bad("A trigger with no source", &answer))?;
    if index_of(&SOURCES, source).is_none() && index_of(&MANUAL_SOURCES, source).is_none() {
        return Err(bad("An unknown trigger source", source));
    }
    source.clone_into(&mut settings.source);
    let answer = scpi.get_string(&format!("{source}:TRIG_SLOPE?"))?;
    settings.slope =
        index_of(&SLOPES, &answer).ok_or_else(|| bad("An unknown trigger slope", &answer))?;
    settings.triggerpos = scpi.get_f32("TRIG_DELAY?")?;
    log::info!("Fetching finished.");
    Ok(settings)
}

impl LecroyDevice {
    fn new(id: &crate::scpi::HwInfo, scpi: Scpi, enabled: [bool; CHANNELS]) -> LecroyDevice {
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        for (i, on) in enabled.iter().enumerate() {
            let name = format!("CH{}", i + 1);
            let mut channel = Channel::new(i, ChannelKind::Analog, name.clone());
            channel.enabled = *on;
            channels.push(channel);
            groups.push(ChannelGroup::new(name, vec![i]));
        }
        LecroyDevice {
            info: DeviceInfo {
                driver: "lecroy-xstream",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels,
            groups,
            settings: Settings {
                trace: enabled,
                vdiv: [0; CHANNELS],
                offset: [0.0; CHANNELS],
                coupling: [0; CHANNELS],
                timebase: 0,
                source: SOURCES[0].to_owned(),
                slope: 0,
                triggerpos: 0.0,
            },
            limit_frames: 0,
            sample_rate: Arc::new(AtomicU64::new(0)),
            scpi: Arc::new(Mutex::new(None)),
            closed: Some(scpi),
            worker: None,
            silence: Silence::new,
            left_armed: Arc::new(AtomicBool::new(false)),
            #[cfg(test)]
            said: Mutex::new(Vec::new()),
        }
    }

    /// A warning, which a test can read back.
    #[cfg_attr(not(test), expect(clippy::unused_self, reason = "the tests read it back"))]
    fn warn(&self, text: &str) {
        log::warn!("{text}");
        #[cfg(test)]
        self.said.lock().expect("said lock").push(text.to_owned());
    }

    /// What a stop left armed, read away within a bound, before anything else is asked.
    fn settle_armed(&self, scpi: &mut Scpi) -> Result<()> {
        if !self.left_armed.load(Ordering::SeqCst) {
            return Ok(());
        }
        let until = Instant::now() + ARMED_SETTLE;
        if !drain_to_opc(scpi, || Instant::now() < until)? {
            return Err(Error::device(
                "The scope is still in the acquisition a stop left waiting for a trigger.",
            ));
        }
        self.left_armed.store(false, Ordering::SeqCst);
        Ok(())
    }

    /// Runs `f` on the open instrument. A capture holds it, so a setting is refused
    /// during one rather than kept unsent.
    fn with_scpi<T>(&self, f: impl FnOnce(&mut Scpi) -> Result<T>) -> Result<T> {
        let mut guard = self.scpi.lock().expect("scpi lock");
        let scpi = guard.as_mut().ok_or(Error::Busy)?;
        self.settle_armed(scpi)?;
        f(scpi)
    }

    /// A setting, then as many `*OPC?` as C asks; if one fails the scope may have
    /// taken it, and a get would still answer the old value.
    fn setting(&self, command: &str, opcs: usize) -> Result<()> {
        let done = self.with_scpi(|scpi| {
            scpi.send(command)?;
            for _ in 0..opcs {
                scpi.get_opc()?;
            }
            Ok(())
        });
        if let Err(e) = &done {
            self.warn(&format!(
                "'{command}' was not confirmed ({e}): the scope may have taken it."
            ));
        }
        done
    }

    fn analog_group(&self, group: Option<&str>) -> Result<usize> {
        let name = group.ok_or_else(|| Error::ChannelGroup(String::new()))?;
        self.groups
            .iter()
            .position(|g| g.name == name)
            .ok_or_else(|| Error::ChannelGroup(name.to_owned()))
    }
}

impl Device for LecroyDevice {
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

    /// `config_channel_set`: on an open scope it's a setting at once, as C has it.
    /// A closed one has nothing to say to, and a capture's start puts it right.
    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let was = self.channels.get(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?.enabled;
        if was == enabled {
            return Ok(());
        }
        if self.scpi.lock().expect("scpi lock").is_some() {
            let on = if enabled { "ON" } else { "OFF" };
            self.setting(&format!("C{}:TRACE {on}", pos + 1), 1)?;
            self.settings.trace[pos] = enabled;
        }
        self.channels[pos].enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        if group.is_none() {
            return Ok(DEV_OPTIONS.to_vec());
        }
        self.analog_group(group)?;
        Ok(ANALOG_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let st = &self.settings;
        Ok(match key {
            Key::NumHdiv => Value::I32(NUM_HDIV),
            Key::Timebase => {
                let (p, q) = TIMEBASES[st.timebase];
                Value::Rational(p, q)
            }
            Key::NumVdiv => {
                self.analog_group(group)?;
                Value::I32(NUM_VDIV)
            }
            Key::Vdiv => {
                let (p, q) = VDIVS[st.vdiv[self.analog_group(group)?]];
                Value::Rational(p, q)
            }
            Key::TriggerSource => Value::Str(st.source.clone()),
            Key::TriggerSlope => Value::Str(SLOPES[st.slope].to_owned()),
            Key::HorizTriggerpos => Value::F64(f64::from(st.triggerpos)),
            Key::Coupling => {
                Value::Str(COUPLINGS[st.coupling[self.analog_group(group)?]].to_owned())
            }
            Key::Samplerate => Value::U64(self.sample_rate.load(Ordering::SeqCst)),
            // limit_frames is listed as a get, and config_get has no case for it.
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        let string = |list: &[&str]| -> Result<usize> {
            value.as_str().and_then(|s| index_of(list, s)).ok_or_else(bad)
        };
        let fraction = |table: &[(u64, u64)]| -> Result<usize> {
            let Value::Rational(p, q) = value else { return Err(bad()) };
            table
                .iter()
                .position(|&(tp, tq)| {
                    u128::from(*p) * u128::from(tq) == u128::from(tp) * u128::from(*q)
                })
                .ok_or_else(bad)
        };
        match key {
            Key::LimitFrames => {
                // config_set's last lines ask *OPC? for this one too, on a live scope.
                self.limit_frames = value.as_u64().ok_or_else(bad)?;
                if self.scpi.lock().expect("scpi lock").is_some() {
                    self.with_scpi(Scpi::get_opc)?;
                }
            }
            Key::TriggerSource => {
                let i = string(&SOURCES)?;
                self.setting(&format!("TRIG_SELECT EDGE,SR,{}", SOURCES[i]), 1)?;
                SOURCES[i].clone_into(&mut self.settings.source);
            }
            Key::Vdiv => {
                let i = fraction(&VDIVS)?;
                let c = self.analog_group(group)?;
                let (p, q) = VDIVS[i];
                let volts = f64::from(p as f32 / q as f32);
                self.setting(&format!("C{}:VDIV {}", c + 1, protocol::c_exp(volts, true)), 2)?;
                self.settings.vdiv[c] = i;
            }
            Key::Timebase => {
                let i = fraction(&TIMEBASES)?;
                let (p, q) = TIMEBASES[i];
                let seconds = f64::from(p as f32 / q as f32);
                self.setting(&format!("TIME_DIV {}", protocol::c_exp(seconds, true)), 1)?;
                self.settings.timebase = i;
            }
            Key::HorizTriggerpos => {
                let position = value.as_f64().ok_or_else(bad)?;
                // A NaN passes C's two comparisons and goes out as `TRIG POS nan S`.
                if !(0.0..=1.0).contains(&position) {
                    return Err(Error::Arg(format!(
                        "Invalid horiz. trigger position: {position}."
                    )));
                }
                let (p, q) = TIMEBASES[self.settings.timebase];
                let seconds = -(position - 0.5) * (p as f64 / q as f64) * f64::from(NUM_HDIV);
                self.setting(&format!("TRIG POS {} S", protocol::c_exp(seconds, false)), 1)?;
                self.settings.triggerpos = position as f32;
            }
            Key::TriggerSlope => {
                let i = string(&SLOPES)?;
                let source = &self.settings.source;
                self.setting(&format!("{source}:TRIG_SLOPE {}", SLOPES[i]), 1)?;
                self.settings.slope = i;
            }
            Key::Coupling => {
                let i = string(&COUPLINGS)?;
                let c = self.analog_group(group)?;
                self.setting(&format!("C{}:COUPLING {}", c + 1, COUPLINGS[i]), 2)?;
                self.settings.coupling[c] = i;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        let strings =
            |items: &[&str]| ValueList::Str(items.iter().map(|s| (*s).to_owned()).collect());
        Ok(match key {
            Key::Coupling => strings(&COUPLINGS),
            Key::TriggerSource => strings(&SOURCES),
            Key::TriggerSlope => strings(&SLOPES),
            Key::Timebase => ValueList::Rational(TIMEBASES.to_vec()),
            Key::Vdiv => ValueList::Rational(VDIVS.to_vec()),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn open(&mut self) -> Result<()> {
        if self.scpi.lock().expect("scpi lock").is_some() {
            return Ok(());
        }
        let Some(mut scpi) = self.closed.take() else {
            return Err(Error::device("The device has no instrument."));
        };
        if let Err(e) = scpi.reopen() {
            self.closed = Some(scpi);
            return Err(e);
        }
        match self.settle_armed(&mut scpi).and_then(|()| read_settings(&mut scpi)) {
            Ok(settings) => {
                self.settings = settings;
                *self.scpi.lock().expect("scpi lock") = Some(scpi);
                Ok(())
            }
            Err(e) => {
                let _ = scpi.close();
                self.closed = Some(scpi);
                Err(e)
            }
        }
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(mut scpi) = self.scpi.lock().expect("scpi lock").take() {
            // A new connection can't receive an old answer; a USB device keeps it.
            if scpi.resource().starts_with("tcp") {
                self.left_armed.store(false, Ordering::SeqCst);
            }
            let _ = scpi.close();
            self.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // A capture whose End is stuck on a queue nobody reads must not hold this up.
        self.stop()?;
        self.sample_rate.store(0, Ordering::SeqCst);
        let enabled: Vec<usize> =
            self.channels.iter().filter(|c| c.enabled).map(|c| c.index).collect();
        if enabled.is_empty() {
            return Err(Error::device("No channels enabled."));
        }
        // setup_channels: what changed since the open, and its writes before the header.
        let wanted: Vec<bool> = self.channels.iter().map(|c| c.enabled).collect();
        let trace = self.settings.trace;
        // A stop's armed acquisition is the capture's to wait out, as long as it takes.
        let left = self.left_armed.swap(false, Ordering::SeqCst);
        let set = self.with_scpi(|scpi| {
            for (i, on) in wanted.iter().enumerate() {
                if *on != trace[i] {
                    let word = if *on { "ON" } else { "OFF" };
                    scpi.send(&format!("C{}:TRACE {word}", i + 1))?;
                }
            }
            Ok(())
        });
        if let Err(e) = set {
            log::error!("Failed to setup channel configuration!");
            return Err(e);
        }
        for (i, on) in wanted.iter().enumerate() {
            self.settings.trace[i] = *on;
        }
        let Some(scpi) = self.scpi.lock().expect("scpi lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let (p, q) = TIMEBASES[self.settings.timebase];
        let capture = Capture {
            enabled,
            limit_frames: self.limit_frames,
            time_div: p as f64 / q as f64,
            sample_rate: Arc::clone(&self.sample_rate),
            silence: self.silence,
            left_armed: Arc::clone(&self.left_armed),
            line_end: std::cell::Cell::new(false),
        };
        if left {
            self.left_armed.store(true, Ordering::SeqCst);
        }
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = Arc::clone(&stop);
        let slot = Arc::clone(&self.scpi);
        let handle = std::thread::Builder::new()
            .name("lecroy-xstream acquisition".to_owned())
            .spawn(move || {
                let mut scpi = scpi;
                // A wait for a trigger outlasts a USBTMC read: the next read waits on
                // the same request rather than make another, which libsigrok does
                // and no firmware is known to take.
                scpi.wait_out_requests(true);
                let outcome = capture.run(&mut scpi, &sink, &stopped);
                scpi.wait_out_requests(false);
                if capture.line_end.get() {
                    scpi.forget_question();
                }
                if let Err(e) = &outcome {
                    log::error!("Acquisition failed: {e}");
                }
                sink.end_while(outcome, || !stopped.load(Ordering::SeqCst));
                scpi.read_timeout = READ_TIMEOUT;
                *slot.lock().expect("scpi lock") = Some(scpi);
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

#[cfg(test)]
impl LecroyDevice {
    fn set_limit_for_test(&mut self, frames: u64) {
        self.limit_frames = frames;
    }
}

impl Drop for LecroyDevice {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// One capture on its own thread.
struct Capture {
    enabled: Vec<usize>,
    limit_frames: u64,
    time_div: f64,
    sample_rate: Arc<AtomicU64>,
    /// [`Silence::new`], or a test's bounds.
    silence: fn(Instant, f64) -> Silence,
    /// The device's: a stop came with the scope armed.
    left_armed: Arc<AtomicBool>,
    /// The last block's line end may still come.
    line_end: std::cell::Cell<bool>,
}

fn request(channel: usize) -> String {
    format!("C{}:WAVEFORM?", channel + 1)
}

/// How much the receive of one block came to: the block, and whether its line end may
/// still come.
enum Read {
    Block(Vec<u8>, bool),
    Stopped,
}

impl Capture {
    /// `dev_acquisition_start`'s end and `receive_data`.
    fn run(&self, scpi: &mut Scpi, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if self.left_armed.load(Ordering::SeqCst) {
            if !drain_to_opc(scpi, go_on)? {
                return Ok(());
            }
            self.left_armed.store(false, Ordering::SeqCst);
        }
        // Close joins this thread, so a full queue no one reads must not hold it.
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let mut silence = (self.silence)(Instant::now(), self.time_div);
        let mut position = 0;
        let mut frames = 0u64;
        scpi.send(&request(self.enabled[position]))?;
        silence.asked(Instant::now(), false);
        loop {
            let block = match read_block(scpi, &mut silence, stop) {
                Ok(Read::Block(block, line_end)) => {
                    self.line_end.set(line_end);
                    block
                }
                // A question is out, and its answer can still come.
                Ok(Read::Stopped) => {
                    scpi.forget_question();
                    if silence.armed() {
                        self.left_armed.store(true, Ordering::SeqCst);
                    }
                    return Ok(());
                }
                Err(e) => {
                    scpi.forget_question();
                    return Err(e);
                }
            };
            let wave = protocol::parse_waveform(&block)?;
            drop(block);
            let channel = self.enabled[position];
            if let Some(source) = wave.source.filter(|s| *s <= 3)
                && usize::from(source) != channel
            {
                return Err(Error::device(format!(
                    "The scope answered for C{} where C{} was asked.",
                    source + 1,
                    channel + 1
                )));
            }
            if wave.samples.is_empty() {
                // No data available, we have to acquire data first.
                self.sample_rate.store(0, Ordering::SeqCst);
                match silence.no_samples(Instant::now()) {
                    Verdict::Wait => {}
                    Verdict::Say(text) => log::warn!("{text}"),
                    Verdict::Fail(text) => return Err(Error::device(text)),
                }
                scpi.send(&format!("ARM;WAIT;*OPC;{}", request(channel)))?;
                silence.asked(Instant::now(), true);
                continue;
            }
            silence.sample();
            if self.sample_rate.load(Ordering::SeqCst) == 0 {
                self.update_sample_rate(scpi, wave.samples.len())?;
            }
            if position == 0 && !sink.send_while(Packet::FrameBegin, go_on) {
                return Ok(());
            }
            let packet = Packet::Analog(AnalogData {
                channels: vec![channel],
                samples: wave.samples,
                mq: wave.mq,
                flags: MqFlags::empty(),
                unit: wave.unit,
                digits: DIGITS,
            });
            if !sink.send_while(packet, go_on) {
                return Ok(());
            }
            if position + 1 < self.enabled.len() {
                position += 1;
                scpi.send(&request(self.enabled[position]))?;
                silence.asked(Instant::now(), false);
                continue;
            }
            if !sink.send_while(Packet::FrameEnd, go_on) {
                return Ok(());
            }
            frames += 1;
            if self.limit_frames != 0 && frames == self.limit_frames {
                return Ok(());
            }
            // Wait for trigger, then begin fetching data.
            position = 0;
            scpi.send("ARM;WAIT;*OPC")?;
            scpi.send(&request(self.enabled[position]))?;
            silence.asked(Instant::now(), true);
        }
    }

    /// `lecroy_xstream_update_sample_rate`.
    fn update_sample_rate(&self, scpi: &mut Scpi, samples: usize) -> Result<()> {
        let mut answer = scpi.get_string("TIME_DIV?")?;
        // The last block's line end, come late: it was read as an answer of its own.
        if self.line_end.replace(false) && answer.is_empty() {
            answer = String::from_utf8_lossy(&scpi.get_data(None)?).into_owned();
        }
        let answer = answer.trim();
        let time_div: f64 =
            answer.parse().map_err(|_| Error::device(format!("SCPI: not a number: '{answer}'")))?;
        let rate = samples as f64 / (time_div * f64::from(NUM_HDIV));
        if rate.is_finite() && rate > 0.0 {
            self.sample_rate.store(rate as u64, Ordering::SeqCst);
        } else {
            // C's cast is undefined here; the next frame asks again, as for a rate of 0.
            log::warn!("The scope's timebase of {time_div} s gives no sample rate.");
        }
        Ok(())
    }
}

/// One block, however it comes, looking at the stop all the while. On USBTMC a read
/// that found nothing reads again, under the request already made (see the capture).
fn read_block(scpi: &mut Scpi, silence: &mut Silence, stop: &AtomicBool) -> Result<Read> {
    let result = read_block_inner(scpi, silence, stop);
    scpi.read_timeout = READ_TIMEOUT;
    result
}

fn read_block_inner(scpi: &mut Scpi, silence: &mut Silence, stop: &AtomicBool) -> Result<Read> {
    scpi.read_timeout = LOOK;
    let mut framer = Framer::default();
    let mut chunk = vec![0u8; READ_CHUNK];
    let mut started = false;
    loop {
        if stop.load(Ordering::SeqCst) {
            return Ok(Read::Stopped);
        }
        // Ahead of the read that may find nothing: a stall after it isn't silence.
        let now = Instant::now();
        let read = if started { Ok(()) } else { scpi.read_begin() };
        let got = match read {
            Ok(()) => {
                started = true;
                scpi.read_data(&mut chunk)
            }
            // USBTMC asks with its read, and says "too little data" of no answer and
            // fails a header that came wrong: C retries both. A gone device is
            // `Disconnected`, an I/O error or a closed socket, and ends it.
            Err(Error::Device(why)) if !framer.begun() => {
                log::debug!("No answer to the read: {why}");
                Ok(0)
            }
            Err(e) => Err(e),
        };
        let n = match got {
            Ok(n) => n,
            // On USBTMC only, whose reads ask: nothing, so ask again. Within a block
            // the rest is still to come of this asking.
            Err(Error::Timeout) => {
                started = framer.begun();
                0
            }
            Err(e) => {
                return Err(match framer.progress() {
                    Some((has, of)) => {
                        log::error!("Read {has} of {of} bytes of a block: {e}");
                        Error::device(format!("The scope's block came short, {has} of {of} bytes."))
                    }
                    None => e,
                });
            }
        };
        if n == 0 {
            match silence.nothing(now) {
                Verdict::Wait => {}
                Verdict::Say(text) => log::warn!("{text}"),
                Verdict::Fail(text) => return Err(Error::device(text)),
            }
            continue;
        }
        let framing = framer.feed(&chunk[..n])?;
        if (framer.begun() || framing != Framing::More)
            && let Some(text) = silence.heard(Instant::now())
        {
            log::warn!("{text}");
        }
        if let Framing::Whole(block) = framing {
            // The line end may lie just past a read's edge, and a text answer would
            // start with it. A look takes it if it comes at once; if not, it is owed.
            let owed =
                framer.line_end_owed() && matches!(scpi.discard(LINE_END), Ok(Some(0)) | Err(_));
            scpi.answered();
            return Ok(Read::Block(block, owed));
        }
    }
}

/// Sends `*OPC?` and reads until an answer says `1`, dropping what came before it:
/// blocks by their length and text by its lines. `Ok(false)` when `go_on` says no first.
fn drain_to_opc(scpi: &mut Scpi, go_on: impl Fn() -> bool) -> Result<bool> {
    log::info!("Reading away the answer of an acquisition a stop left armed.");
    scpi.send("*OPC?")?;
    let saved = scpi.read_timeout;
    scpi.read_timeout = LOOK;
    let result = drain(scpi, &go_on);
    scpi.read_timeout = saved;
    if result.as_ref().is_ok_and(|done| *done) {
        scpi.answered();
    } else {
        scpi.forget_question();
    }
    result
}

fn drain(scpi: &mut Scpi, go_on: &impl Fn() -> bool) -> Result<bool> {
    let mut chunk = vec![0u8; READ_CHUNK];
    let mut pending: Vec<u8> = Vec::new();
    // What is left of a block being dropped.
    let mut skip = 0usize;
    let mut started = false;
    while go_on() {
        if !started {
            match scpi.read_begin() {
                Ok(()) => started = true,
                Err(Error::Timeout | Error::Device(_)) => continue,
                Err(e) => return Err(e),
            }
        }
        let n = match scpi.read_data(&mut chunk) {
            Ok(n) => n,
            Err(Error::Timeout | Error::Device(_)) => 0,
            Err(e) => return Err(e),
        };
        if n == 0 || scpi.read_complete() {
            started = false;
        }
        let mut bytes = &chunk[..n];
        let dropped = skip.min(bytes.len());
        skip -= dropped;
        bytes = &bytes[dropped..];
        pending.extend_from_slice(bytes);
        loop {
            let lead = pending.iter().take_while(|b| matches!(b, b'\n' | b'\r')).count();
            pending.drain(..lead);
            if pending.is_empty() {
                break;
            }
            if let Some((head, length)) = protocol::block_head(&pending) {
                let have = pending.len().min(head + length);
                skip = head + length - have;
                pending.drain(..have);
                continue;
            }
            if protocol::may_be_block(&pending) {
                break;
            }
            let Some(end) = pending.iter().position(|b| *b == b'\n') else { break };
            let line: Vec<u8> = pending.drain(..=end).collect();
            if String::from_utf8_lossy(&line).trim() == "1" {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

#[cfg(test)]
mod tests;
