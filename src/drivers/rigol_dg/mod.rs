// Counterpart in sigrok:
//   libsigrok/src/hardware/rigol-dg/api.c,
//   Copyright (C) 2020 Timo Kokkonen <tjko@iki.fi>,
//   with commits by Ralf, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Rigol DG800, DG900 and DG1000Z function generators, and the generator inside an
//! MSO5000, on any SCPI carrier: USBTMC and TCP are what a real one has.
//!
//! A scan asks `*IDN?`, then on a DG `*OPC?` and `SYST:KLOC:STATE OFF`, which frees
//! its front panel, and closes. An open sends nothing, and a close the same two as the
//! scan. Each `--get` and each `-c` asks the instrument first, with an `*OPC?` before
//! every query: a value is never held here. A setting has no answer, and the
//! instrument sets a value out of range to the nearest limit and says nothing (the
//! DG1000Z guide, p.2-97, 2-155, 2-182, 2-185): such a value is refused here before it
//! goes out. Frequency, phase and a pulse's duty cycle go by the table; amplitude,
//! offset and a duty cycle by what `X? MIN` and `X? MAX` say just before, as they move
//! with the load and the frequency. Where those don't answer, and on an MSO5000, whose
//! way of asking isn't known, the value goes with a warning. `pattern=` is
//! `SOURn:FUNC x`, which keeps what is valid for the new waveform.
//!
//! A capture has nothing of the generator in it. It runs the frequency counter, turning
//! it on if `COUN:STAT?` doesn't say `RUN` and off again at the end, and asks
//! `COUN:MEAS?` 100 ms after each answer. A frame is a reading, for `limit_samples`:
//! `FREQ1`, `PERIOD1`, `DUTY1` and `WIDTH1`, as C sends them, whatever `-C` says. A
//! DG800's or DG900's counter wants a second after each of those two commands, by C's
//! commit. A set during a capture goes between two rounds. An MSO5000 has no counter
//! and no limit keys: a capture there sends a header, and an end when it is stopped.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A reading C can't tell from a good one: what isn't a number is no reading, where
//!   `g_ascii_strtod` (`protocol.c:289`) makes 0 of `abc` and 1.5 of `1.5xyz`. The same
//!   goes for a channel's values (`protocol.c:89`, `185-203`), and a value the
//!   instrument didn't give is an error from `get`, where C answers what it had last,
//!   0 at first. `DEF`, the guide's word, stays 0.
//! - A capture goes on past a counter that doesn't answer ([`protocol::Patience`]): 30 s
//!   for the first reading, and 10 minutes later on, with a warning, and a line when
//!   it's back; 10 s and a minute for answers that make no reading. C ends at the first
//!   timeout with exit 0 (`protocol.c:274-277`), and never checks a limit after a short
//!   answer (`protocol.c:287-288`), so `--time` and `--samples` never end on a stream of
//!   them. A capture that fails is exit 1. With a time limit and nothing captured,
//!   exit 0 and a warning.
//! - An MSO5000's `SINC` reads as `Sinc`. C takes the first name that fits it, so it
//!   reads as `Sine`, with Sine's range (`protocol.c:30-36`).
//! - An empty `SOURn:APPL?` is an error. C goes on with no waveform and dies on it
//!   (`protocol.c:168-169`, `api.c:606-607`).
//! - A NaN or an infinity is refused, and nothing goes out: C sends `SOUR1:VOLT nan`.
//! - A setting stays a double, as in the driver. sigrok-cli's `strtof` narrows it
//!   first, so 123456789 Hz goes out from sigrok-cli as 123456792.000000.
//! - `pattern=` is `SOURn:FUNC x` on a DG, where C's `SOURn:APPL:x` with no values
//!   takes 1 kHz, 5 Vpp, 0 V and 0 degrees on what may be a live output.
//! - A value out of range is refused, and amplitude, offset and duty cycle ask the
//!   instrument's limits first: C sends it, and the instrument clamps it.
//! - A frame's values are `f32`, as every analog packet here: a counter's ten digits
//!   come to seven or so. C sends doubles.
//! - A counter that is off answers zeros, which are a reading here as in C: a counter
//!   that didn't go on looks like no signal.
//!
//! Kept as in C, with a word on it:
//!
//! - Channel 2 stays as it is when the counter goes on, on a DG812 too. C means to turn
//!   it off there, as its commit says the two can't run together, but `need_quirk &=
//!   num_channels > 1` makes `2 & 1` of that (`api.c:902-904`), so the path has never
//!   run. It would turn off an output and never turn it on again. A warning says it.
//! - A failed `COUN:STAT?` refuses the start, and still sends the stop's
//!   `COUN:STAT OFF` (`session.c:831-835`), to a counter whose state is then unknown.
//! - A counter in `STOP` or `SINGLE` is not `RUN`, so it runs through the capture and
//!   ends off.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::{self, CmdArg, Resource, Scpi};
use protocol::{Bounds, ChannelState, Cmd, DeviceSpec, Limits, Link, Next, Patience, Round};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SignalGenerator];
/// `dg1000z_devopts`.
static DG_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];
/// `dg1000z_devopts_cg`, and `mso5000_devopts_cg`, which is the same.
static GROUP_OPTIONS: [OptionDesc; 7] = [
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
    OptionDesc::new(Key::OutputFrequency, Access::GET_SET_LIST),
    OptionDesc::new(Key::Amplitude, Access::GET_SET),
    OptionDesc::new(Key::Offset, Access::GET_SET),
    OptionDesc::new(Key::Phase, Access::GET_SET_LIST),
    OptionDesc::new(Key::DutyCycle, Access::GET_SET),
];

/// The session source's timeout in `dev_acquisition_start`.
const POLL: Duration = Duration::from_millis(100);
/// `RIGOL_DG_COUNTER_BUG_DELAY`.
const COUNTER_DELAY: Duration = Duration::from_secs(1);
/// How often a wait looks for a stop.
const STOP_CHECK: Duration = Duration::from_millis(5);
/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `rigol-dg` driver.
pub struct RigolDgDriver;

/// `probe_device`: a generator of the table, or `None`.
fn probe(mut scpi: Scpi) -> Option<RigolDgDevice> {
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::debug!("*IDN?: {e}.");
            let _ = scpi.close();
            return None;
        }
    };
    let Some(spec) = protocol::find_model(&id) else {
        log::debug!("Device {} {} is not supported by this driver.", id.manufacturer, id.model);
        let _ = scpi.close();
        return None;
    };
    let mut link = Link::new(scpi, spec);
    // In case only a scan was done.
    link.setup_local();
    if let Err(e) = link.scpi.close() {
        log::debug!("Closing after the scan: {e}.");
    }
    Some(RigolDgDevice::new(&id, spec, link))
}

impl Driver for RigolDgDriver {
    fn id(&self) -> &'static str {
        "rigol-dg"
    }

    fn long_name(&self) -> &'static str {
        "Rigol DG Series"
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
        // What fails here is no device, as `sr_scpi_scan_resource` has it.
        if let Some(Err(e)) = conn.as_deref().map(Resource::parse) {
            log::info!("{e}");
            return Ok(Vec::new());
        }
        let found = match scpi::scan(conn.as_deref(), None, |s| Ok(probe(s))) {
            Ok(found) => found,
            Err(e) => {
                log::info!("{e}");
                Vec::new()
            }
        };
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// A limit's value, which a running capture reads each round, as `sr_sw_limits`.
#[derive(Debug, Default)]
struct SharedLimits {
    samples: AtomicU64,
    msec: AtomicU64,
}

impl SharedLimits {
    fn now(&self) -> Limits {
        Limits {
            samples: self.samples.load(Ordering::SeqCst),
            msec: self.msec.load(Ordering::SeqCst),
        }
    }
}

/// One generator.
pub struct RigolDgDevice {
    info: DeviceInfo,
    spec: &'static DeviceSpec,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    limits: Arc<SharedLimits>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    /// A capture is running and the next one must wait.
    capturing: Arc<AtomicBool>,
    poll: Duration,
    counter_delay: Duration,
    bounds: Bounds,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl RigolDgDevice {
    fn new(id: &scpi::HwInfo, spec: &'static DeviceSpec, link: Link) -> Self {
        let mut channels = Vec::new();
        let mut groups = Vec::new();
        for i in 0..spec.channels {
            channels.push(Channel::new(i, ChannelKind::Analog, format!("CH{}", i + 1)));
            groups.push(ChannelGroup::new(format!("{}", i + 1), vec![i]));
        }
        if spec.has_counter {
            for (i, name) in ["FREQ1", "PERIOD1", "DUTY1", "WIDTH1"].iter().enumerate() {
                channels.push(Channel::new(
                    spec.channels + i,
                    ChannelKind::Analog,
                    (*name).to_owned(),
                ));
            }
        }
        RigolDgDevice {
            info: DeviceInfo {
                driver: "rigol-dg",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            spec,
            channels,
            groups,
            limits: Arc::new(SharedLimits::default()),
            link: Arc::new(Mutex::new(link)),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            poll: POLL,
            counter_delay: COUNTER_DELAY,
            bounds: Bounds::default(),
        }
    }

    /// A device that a scan of `scpi` found, with the scan's wire. For tests, and for
    /// clients with a transport of their own.
    ///
    /// # Errors
    ///
    /// When it is not one of the table's, or does not answer.
    pub fn from_scpi(scpi: Scpi) -> Result<Self> {
        probe(scpi).ok_or_else(|| Error::device("Not a generator of this driver's table."))
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn check_group(&self, group: &str) -> Result<()> {
        if self.groups.iter().any(|g| g.name == group) {
            Ok(())
        } else {
            Err(Error::ChannelGroup(group.to_owned()))
        }
    }

    fn open_link(&self) -> Result<std::sync::MutexGuard<'_, Link>> {
        let link = self.link();
        if link.is_open { Ok(link) } else { Err(Error::device("device is not open")) }
    }

    fn get_channel(&self, key: &Key, group: &str) -> Result<Value> {
        if !GROUP_OPTIONS.iter().any(|o| &o.key == key) {
            return Err(Error::UnknownKey(key.id().to_owned()));
        }
        let mut link = self.open_link()?;
        if *key == Key::Enabled {
            link.opc();
            let answer = link.ask(Cmd::GetEnabled, group)?;
            return scpi::parse_bool(&answer)
                .map(Value::Bool)
                .ok_or_else(|| Error::device(format!("SCPI: not a boolean: '{answer}'")));
        }
        let state = link.channel_state(group)?;
        match key {
            Key::PatternMode => Ok(Value::from(state.waveform.user_name)),
            Key::OutputFrequency => said(key, state.frequency),
            Key::Amplitude => said(key, state.amplitude),
            Key::Offset => said(key, state.offset),
            Key::Phase => said(key, state.phase),
            _ => {
                let command = match state.waveform.user_name {
                    "Square" => Cmd::GetDutySquare,
                    "Pulse" => Cmd::GetDutyPulse,
                    other => return Err(not_on(key, other)),
                };
                link.opc();
                let answer = link.ask(command, group)?;
                protocol::parse_number(&answer)
                    .map(Value::F64)
                    .ok_or_else(|| Error::device(format!("SCPI: not a number: '{answer}'")))
            }
        }
    }

    fn set_channel(&self, key: &Key, value: &Value, group: &str) -> Result<()> {
        let setting = Setting::of(key, value)?;
        let mut link = self.open_link()?;
        let state = link.channel_state(group)?;
        link.opc();
        setting.send(key, &mut link, &state, group, self.spec)
    }
}

/// A value the instrument gave, or an error.
fn said(key: &Key, value: Option<f64>) -> Result<Value> {
    value.map(Value::F64).ok_or_else(|| {
        Error::device(format!("The instrument gave no usable value for {}.", key.id()))
    })
}

fn not_on(key: &Key, waveform: &str) -> Error {
    Error::not_supported(format!("{} on a waveform of {waveform}", key.id()))
}

/// What a `-c` asks, checked with nothing on the wire.
enum Setting<'v> {
    Enabled(bool),
    Pattern(&'v str),
    Number(f64),
}

impl<'v> Setting<'v> {
    fn of(key: &Key, value: &'v Value) -> Result<Setting<'v>> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Enabled => value.as_bool().map(Setting::Enabled).ok_or_else(bad),
            Key::PatternMode => value.as_str().map(Setting::Pattern).ok_or_else(bad),
            Key::OutputFrequency | Key::Amplitude | Key::Offset | Key::Phase | Key::DutyCycle => {
                value.as_f64().filter(|v| v.is_finite()).map(Setting::Number).ok_or_else(bad)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// The part of `config_set` behind the state's read and its `*OPC?`.
    fn send(
        self,
        key: &Key,
        link: &mut Link,
        state: &ChannelState,
        group: &str,
        spec: &DeviceSpec,
    ) -> Result<()> {
        let waveform = state.waveform;
        let number = match self {
            Setting::Enabled(on) => {
                let command = if on { Cmd::SetEnable } else { Cmd::SetDisable };
                return link.tell(command, group, None);
            }
            Setting::Pattern(name) => {
                let Some(new) = spec.waveform_named(name) else {
                    return Err(Error::not_supported(format!("no pattern '{name}'")));
                };
                return link.tell(Cmd::SetSource, group, Some(CmdArg::Str(new.scpi_name)));
            }
            Setting::Number(number) => number,
        };
        // What the waveform takes, a table's range if there is one, the query that says
        // the instrument's limits where they move with the load and the frequency, and
        // the unit.
        let (opt, command, table, asked, unit): (u8, Cmd, Option<[f64; 2]>, Option<Cmd>, &str) =
            match key {
                Key::OutputFrequency => (
                    protocol::WFO_FREQUENCY,
                    Cmd::SetFrequency,
                    Some([waveform.freq_min, waveform.freq_max]),
                    None,
                    "Hz",
                ),
                Key::Amplitude => (
                    protocol::WFO_AMPLITUDE,
                    Cmd::SetAmplitude,
                    None,
                    Some(Cmd::GetAmplitude),
                    "Vpp",
                ),
                Key::Offset => {
                    (protocol::WFO_OFFSET, Cmd::SetOffset, None, Some(Cmd::GetOffset), "V")
                }
                Key::Phase => (
                    protocol::WFO_PHASE,
                    Cmd::SetPhase,
                    Some([spec.phase[0], spec.phase[1]]),
                    None,
                    "degrees",
                ),
                _ => match waveform.user_name {
                    "Square" => (
                        protocol::WFO_DUTY_CYCLE,
                        Cmd::SetDutySquare,
                        None,
                        Some(Cmd::GetDutySquare),
                        "%",
                    ),
                    "Pulse" => (
                        protocol::WFO_DUTY_CYCLE,
                        Cmd::SetDutyPulse,
                        Some(protocol::PULSE_DUTY),
                        Some(Cmd::GetDutyPulse),
                        "%",
                    ),
                    other => return Err(not_on(key, other)),
                },
            };
        if waveform.opts & opt == 0 {
            return Err(not_on(key, waveform.user_name));
        }
        let refuse = |range: [f64; 2]| -> Result<()> {
            match protocol::out_of_range(key.id(), number, range, unit) {
                Some(why) => Err(Error::Arg(why)),
                None => Ok(()),
            }
        };
        if let Some(range) = table {
            refuse(range)?;
        }
        // An MSO5000's way of asking for its limits isn't known: its value goes with the
        // word, as the instrument's clamp is all that stands there.
        if let Some(query) = asked {
            match spec.has_counter.then(|| link.limits(query, group)).flatten() {
                Some(range) => refuse(range)?,
                None => log::warn!("{}", protocol::limits_unknown(key.id())),
            }
        }
        link.tell(command, group, Some(CmdArg::Float(number)))
    }
}

impl Device for RigolDgDevice {
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
            None if self.spec.has_counter => DG_OPTIONS.to_vec(),
            None => Vec::new(),
            Some(g) => {
                self.check_group(g)?;
                GROUP_OPTIONS.to_vec()
            }
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let Some(group) = group else {
            let limits = self.limits.now();
            return Ok(match key {
                Key::LimitSamples => Value::U64(limits.samples),
                Key::LimitMsec => Value::U64(limits.msec),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            });
        };
        self.check_group(group)?;
        self.get_channel(key, group)
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(group) = group {
            self.check_group(group)?;
            return self.set_channel(key, value, group);
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        // `core::config_set` refuses a 0 and says so; this is for a library caller
        // that comes straight here.
        let number = value.as_u64().filter(|n| *n > 0).ok_or_else(bad);
        match key {
            Key::LimitSamples => self.limits.samples.store(number?, Ordering::SeqCst),
            Key::LimitMsec => self.limits.msec.store(number?, Ordering::SeqCst),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let Some(group) = group else {
            return Err(Error::UnknownKey(key.id().to_owned()));
        };
        self.check_group(group)?;
        match key {
            Key::PatternMode => Ok(ValueList::Str(
                self.spec.waveforms.iter().map(|w| w.user_name.to_owned()).collect(),
            )),
            Key::OutputFrequency => {
                let state = self.open_link()?.channel_state(group)?;
                let w = state.waveform;
                Ok(ValueList::RangeStepF64 { min: w.freq_min, max: w.freq_max, step: w.freq_step })
            }
            Key::Phase => {
                let [min, max, step] = self.spec.phase;
                Ok(ValueList::RangeStepF64 { min, max, step })
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `sr_scpi_open`: on the wire what the carrier needs to connect.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link();
        if link.is_open {
            return Ok(());
        }
        link.scpi.reopen()?;
        link.is_open = true;
        Ok(())
    }

    /// `dev_close`: back to local, and the connection goes.
    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(worker) = self.worker.take() {
            let _ = worker.handle.join();
        }
        let mut link = self.link();
        if !link.is_open {
            return Ok(());
        }
        link.setup_local();
        link.is_open = false;
        link.scpi.close()
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        if let Some(worker) = self.worker.take() {
            // Its End may be stuck on a queue nobody reads.
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        if !self.link().is_open {
            return Err(Error::device("device is not open"));
        }
        let capture = Capture {
            link: Arc::clone(&self.link),
            spec: self.spec,
            limits: Arc::clone(&self.limits),
            poll: self.poll,
            counter_delay: self.counter_delay,
            bounds: self.bounds,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned =
            std::thread::Builder::new().name("rigol-dg acquisition".to_owned()).spawn(move || {
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

    /// Asks the thread to end, and does not wait: it may be sending, and it has the
    /// counter to turn off, as `dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and
/// after a stop only for [`END_AFTER_STOP`]: a close joins this thread.
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

/// One capture, on its own thread: `dev_acquisition_start`, `rigol_dg_receive_data`
/// and `dev_acquisition_stop`.
struct Capture {
    link: Arc<Mutex<Link>>,
    spec: &'static DeviceSpec,
    limits: Arc<SharedLimits>,
    poll: Duration,
    counter_delay: Duration,
    bounds: Bounds,
}

impl Capture {
    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let ended = self.start_counter().and_then(|()| {
            let go_on = || !stop.load(Ordering::SeqCst);
            if !sink.send_while(Packet::header(), go_on) {
                return Ok(());
            }
            self.rounds(sink, stop)
        });
        self.stop_counter();
        ended
    }

    /// The part of `dev_acquisition_start` ahead of the header.
    fn start_counter(&self) -> Result<()> {
        let mut link = self.link();
        if !link.has(Cmd::CounterGetEnabled) {
            return Ok(());
        }
        let answer = link
            .scpi
            .cmd_resp(self.spec.cmdset, None, &Cmd::CounterGetEnabled, &[])
            .map_err(|e| Error::device(format!("No answer about the counter's state: {e}")))?;
        let running = answer.get(..3).is_some_and(|s| s.eq_ignore_ascii_case("RUN"));
        link.counter_was_running = running;
        if running {
            return Ok(());
        }
        if self.spec.counter_conflicts_with_channel_2() {
            log::warn!(
                "A {} can't run channel 2 and the counter together, by libsigrok's notes: \
                 channel 2 is left as it is.",
                self.spec.model
            );
        }
        link.opc();
        let turned_on = link.scpi.cmd(self.spec.cmdset, None, &Cmd::CounterSetEnable, &[]);
        drop(link);
        if self.spec.counter_needs_delay() {
            std::thread::sleep(self.counter_delay);
        }
        turned_on
    }

    /// The part of `dev_acquisition_stop` on the wire, which also runs after a refused
    /// start, as the session's does.
    fn stop_counter(&self) {
        let mut link = self.link();
        if !link.has(Cmd::CounterSetDisable) || link.counter_was_running {
            return;
        }
        link.opc();
        if let Err(e) = link.scpi.cmd(self.spec.cmdset, None, &Cmd::CounterSetDisable, &[]) {
            log::debug!("Counter off: {e}.");
        }
        drop(link);
        if self.spec.counter_needs_delay() {
            std::thread::sleep(self.counter_delay);
        }
    }

    /// Waits for the session source's timeout. False for a stop.
    fn pause(&self, stop: &AtomicBool) -> bool {
        let began = Instant::now();
        loop {
            if stop.load(Ordering::SeqCst) {
                return false;
            }
            let left = self.poll.saturating_sub(began.elapsed());
            if left.is_zero() {
                return true;
            }
            std::thread::sleep(left.min(STOP_CHECK));
        }
    }

    /// `rigol_dg_receive_data`, again and again: with the lock free between rounds, for
    /// a client's get or set.
    fn rounds(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let mut patience = Patience::new(Instant::now(), self.bounds);
        loop {
            let waited = Instant::now();
            if !self.pause(stop) {
                return Ok(());
            }
            patience.excuse(waited.elapsed().saturating_sub(self.poll));
            if !self.spec.has_counter {
                continue;
            }
            let limits = self.limits.now();
            let asked = Instant::now();
            let keep_at_it =
                || !stop.load(Ordering::SeqCst) && !patience.out_of_time(Instant::now(), limits);
            let round = self.link().measure(&keep_at_it);
            let (next, say) = match round {
                Round::Reading(measure) => {
                    let go_on = || !stop.load(Ordering::SeqCst);
                    if !frame(self.spec.channels, &measure)
                        .into_iter()
                        .all(|p| sink.send_while(p, go_on))
                    {
                        return Ok(());
                    }
                    // After the queue took it: a slow client is no silent counter.
                    let (next, back) = patience.reading(Instant::now(), limits);
                    (next, back.map(str::to_owned))
                }
                Round::Junk(text) => patience.junk(&text, asked, Instant::now(), limits),
                Round::Stopped if stop.load(Ordering::SeqCst) => return Ok(()),
                Round::Silent | Round::Stopped => patience.silent(asked, Instant::now(), limits),
                Round::Failed(e) => return Err(e),
            };
            if let Some(say) = say {
                log::warn!("{say}");
            }
            match next {
                Next::GoOn => {}
                Next::Done => return Ok(()),
                Next::Fail(why) => return Err(Error::device(why)),
            }
        }
    }
}

/// One reading's frame, on the channels behind the generator's own: `rigol_dg_receive_data`
/// sends all four, whatever is enabled.
fn frame(first: usize, measure: &protocol::Measure) -> Vec<Packet> {
    let analog = |at: usize, mq, unit, digits| {
        Packet::Analog(AnalogData {
            channels: vec![first + at],
            samples: vec![measure[at] as f32],
            mq,
            flags: MqFlags::empty(),
            unit,
            digits,
        })
    };
    vec![
        Packet::FrameBegin,
        analog(0, Mq::Frequency, Unit::Hertz, 10),
        analog(1, Mq::Time, Unit::Second, 10),
        analog(2, Mq::DutyCycle, Unit::Percentage, 3),
        analog(3, Mq::PulseWidth, Unit::Second, 10),
        Packet::FrameEnd,
    ]
}

#[cfg(test)]
mod tests;
