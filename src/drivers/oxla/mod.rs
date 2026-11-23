// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `oxla`: the oxla logic analyzer gateware (a Sipeed Tang Nano 20K) on
//! its serial port. 8, 14 or 16 channels, a two term trigger, run length
//! encoding where the bitstream has it. There is no libsigrok counterpart.
//!
//! `conn=` names the port and is required; `serialcomm=` overrides
//! 3000000/8n1. The `oxla-tn20k` driver ([`tn20k`]) finds the board on USB
//! instead, and loads the gateware into it when it does not run it
//! ([`OxlaDevice::to_load`]); from the open on its device is this one. A scan gets in step with the device, which resets it, and
//! asks for its ID; everything else comes from that: the channels, the
//! memory, whether there is RLE and a rate of two samples per clock (see
//! [`protocol`]). Where the ID says the host can change the capture clock,
//! the scan sets each of its settings once and asks what clock that is, so
//! the rates are those of the clocks the device reports; it is left at the
//! last. The scan keeps the port for the open, so one
//! `OXROK_SERIAL_RECORD` file holds a session.
//!
//! `samplerate` takes any rate from the lowest to the highest and sets the
//! nearest one a clock gives, the clock times two or divided by a whole
//! number: of the clocks that give it, the one the last rate had.
//!
//! A capture resets the device, asks for its ID, changes its clock when it
//! is not the rate's, writes its registers and arms it, waits
//! for it to end, reads the part of the memory the samples wanted are in,
//! 2048 words a request, and sends them oldest first with the trigger
//! marked. The trigger is looked for from the first sample on: one that
//! comes early has fewer samples ahead of it than `captureratio` asks for.
//! With no trigger the capture is the first `limit_samples` samples.
//!
//! A stop while the device waits for its trigger sends its STOP, and what
//! it had is sent on: the last `limit_samples` samples, with no marker. A
//! stop during the read sends what was read. A capture whose memory
//! overflowed (RLE on a signal that changes with nearly every sample, above
//! half the clock rate) fails, with none of its samples sent: the device
//! then does not vouch for the trigger's place.
//!
//! `pattern=Internal` makes the samples a counter that goes up by one per
//! sample from 0, inside the device, which the trigger sees too. Disabled
//! channels are low in what is sent, as the device has them on its inputs;
//! its counter does not go through its channel mask, so the driver clears
//! them.

#![forbid(unsafe_code)]

pub mod gowin;
pub(crate) mod protocol;
#[cfg(test)]
mod tests;
pub mod tn20k;

use crate::core::{
    Access, Channel, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc, Packet,
    PacketSink, Result, Trigger, TriggerFeatures, TriggerMatch, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{ClockSetting, Clocks, Cut, Description, Ended, ID_LEN, Id, Plan, Rate, Window};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// The time the device gets to act on the bytes that put it in step, and
/// what it was still sending to arrive, before the input is dropped.
const SETTLE: Duration = Duration::from_millis(50);
/// Silence this long inside an answer is a failure: a chunk of memory
/// takes a tenth of a second.
const ANSWER_TIMEOUT: Duration = Duration::from_secs(2);
/// How long one read waits.
const READ_SLICE: Duration = Duration::from_millis(50);
/// Between two looks at the status of a capture.
const POLL: Duration = Duration::from_millis(10);
/// The looks at the status a STOP gets to show.
const STOP_POLLS: u32 = 200;
/// What the device gets after a change of clock before the first look at
/// its status, and the looks it gets to be ready: the SDRAM starts over,
/// which takes some milliseconds.
const RETUNE_SETTLE: Duration = Duration::from_millis(20);
const READY_POLLS: u32 = 200;
/// The sample limit until one is set.
const DEFAULT_SAMPLES: u64 = 10_000;
/// The size of a logic packet.
const PACKET_BYTES: usize = 1 << 16;
const PATTERN_NONE: &str = "None";
const PATTERN_INTERNAL: &str = "Internal";

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static TRIGGER_MATCHES: [TriggerMatch; 5] = [
    TriggerMatch::Zero,
    TriggerMatch::One,
    TriggerMatch::Rising,
    TriggerMatch::Falling,
    TriggerMatch::Edge,
];
/// One stage is the device's term A, two are A then B or A or B, and one
/// stage can have a count ([`protocol::trigger_setup`]).
const FEATURES: TriggerFeatures = TriggerFeatures {
    count: true,
    consecutive: false,
    duration: false,
    pulse: false,
    range: false,
    or: true,
    and: false,
    max_stages: 2,
    holdoff: false,
};

/// Fills `buf` from the port and says how much came: less than all of it
/// when the device went silent for `silence`.
fn read_answer(port: &mut dyn Port, buf: &mut [u8], silence: Duration) -> Result<usize> {
    let mut got = 0;
    let mut last = Instant::now();
    while got < buf.len() {
        let n = port.read(&mut buf[got..], READ_SLICE)?;
        if n > 0 {
            got += n;
            last = Instant::now();
        } else if last.elapsed() >= silence {
            break;
        }
    }
    Ok(got)
}

/// Gets in step with the device, which resets it, and drops what it was
/// still sending.
fn resync(port: &mut dyn Port) -> Result<()> {
    port.write(&protocol::RESYNC)?;
    std::thread::sleep(SETTLE);
    port.purge()
}

/// Reads a register.
fn rreg(port: &mut dyn Port, addr: u8, silence: Duration) -> Result<u32> {
    port.write(&protocol::rreg(addr))?;
    let mut v = [0; 4];
    match read_answer(port, &mut v, silence)? {
        4 => Ok(u32::from_le_bytes(v)),
        got => Err(Error::device(format!(
            "no answer to the read of register {addr:#04x}: {got} of 4 bytes"
        ))),
    }
}

/// How long the device gets: [`ANSWER_TIMEOUT`] and [`POLL`], or less for
/// a test.
#[derive(Clone, Copy, Debug)]
struct Pace {
    answer: Duration,
    poll: Duration,
}

const PACE: Pace = Pace { answer: ANSWER_TIMEOUT, poll: POLL };

/// The answer to the ID command, and how much of it came.
fn ask_id(port: &mut dyn Port, pace: Pace) -> Result<(std::result::Result<Id, String>, usize)> {
    port.write(&[protocol::OP_ID])?;
    let mut d = [0; ID_LEN];
    let got = read_answer(port, &mut d, pace.answer)?;
    Ok((Id::parse(&d[..got]), got))
}

/// The ID of a device known to be one.
fn read_id(port: &mut dyn Port, pace: Pace) -> Result<Id> {
    ask_id(port, pace)?.0.map_err(Error::device)
}

/// Sets the capture clock to `select` and gives the ID once the device is
/// ready again. What it had captured is lost, and the other registers are
/// to be written again.
fn retune(port: &mut dyn Port, select: u32, pace: Pace) -> Result<Id> {
    port.write(&protocol::wreg(protocol::REG_CLOCK, select))?;
    std::thread::sleep(RETUNE_SETTLE);
    for _ in 0..READY_POLLS {
        if rreg(port, protocol::REG_STATUS, pace.answer)? & protocol::STATUS_READY != 0 {
            return read_id(port, pace);
        }
        std::thread::sleep(pace.poll);
    }
    Err(Error::device(format!("the device did not get ready at clock setting {select}")))
}

/// What a [`probe`] found: the device and the clocks it has.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    /// The ID, as the device first gave it.
    pub id: Id,
    /// Its clocks, each as the device reported it.
    pub clocks: Clocks,
    /// What it says of itself, where it does.
    pub description: Option<Description>,
}

/// The description of a device whose ID says it has one, or none when its
/// format is 0: a bitstream that has the bit and not the registers.
fn read_description(port: &mut dyn Port, pace: Pace) -> Result<Option<Description>> {
    let format = rreg(port, protocol::REG_DESC, pace.answer)?;
    if format == 0 {
        return Ok(None);
    }
    // A later format adds to these and leaves them as they are.
    log::debug!("Description format {format}.");
    let mut regs = [0; 5];
    for (value, addr) in regs.iter_mut().zip(protocol::DESCRIPTION_REGS) {
        *value = rreg(port, addr, pace.answer)?;
    }
    let count = (regs[1] & 0xff).min(protocol::MAX_CLOCKS);
    let mut clocks_hz = Vec::new();
    for select in 0..count {
        clocks_hz.push(rreg(port, protocol::REG_CLOCK_HZ + select as u8, pace.answer)?);
    }
    Ok(Some(Description::new(regs, clocks_hz)))
}

/// Asks what is on `port`: an oxla device and its clocks, or none when it
/// does not answer as one. The device is reset, and one whose clock the
/// host can change is left at the last of its settings.
///
/// # Errors
///
/// A port that fails, or a device that does not come back from a change of
/// clock.
pub fn probe(port: &mut dyn Port) -> Result<Option<Found>> {
    probe_within(port, PACE)
}

/// [`probe`], at `pace`.
fn probe_within(port: &mut dyn Port, pace: Pace) -> Result<Option<Found>> {
    probe_logging(port, pace, log::Level::Error, None)
}

/// [`probe`] of a board that may well run something else: what answers
/// with other bytes is no error to report.
fn probe_board(port: &mut dyn Port) -> Result<Option<Found>> {
    probe_logging(port, PACE, log::Level::Debug, None)
}

/// [`probe`], at `pace`, with an answer that is no ID logged at `level`.
/// With a variant to `expect`, an ID of another is no device either, and
/// is not asked anything more: it may be an answer of what ran before.
fn probe_logging(
    port: &mut dyn Port,
    pace: Pace,
    level: log::Level,
    expect: Option<&str>,
) -> Result<Option<Found>> {
    resync(port)?;
    let id = match ask_id(port, pace)? {
        (Ok(id), _) => id,
        (Err(m), 0) => {
            log::debug!("No answer to the ID command ({m}).");
            return Ok(None);
        }
        (Err(m), _) => {
            log::log!(level, "{m}.");
            return Ok(None);
        }
    };
    if expect.is_some_and(|variant| id.variant() != variant) {
        log::log!(level, "An ID of a {}.", id.variant());
        return Ok(None);
    }
    // A device that describes itself says what its clocks are, and is
    // left at the one it has.
    if id.has_description()
        && let Some(description) = read_description(port, pace)?
        && !description.clock_settings().is_empty()
    {
        let clocks = Clocks::described(&id, &description);
        return Ok(Some(Found { id, clocks, description: Some(description) }));
    }
    let mut settings = Vec::new();
    if id.has_clocks() {
        for select in 0..protocol::CLOCK_SETTINGS {
            let at = retune(port, select, pace)?;
            if !at.same_board(&id) {
                return Err(Error::device("the device is another after a change of clock"));
            }
            log::debug!("Clock setting {select} is {} Hz.", at.clock_hz);
            settings.push(ClockSetting { select: Some(select), hz: at.clock_hz });
        }
    }
    Ok(Some(Found { id, clocks: Clocks::new(&id, settings), description: None }))
}

/// What a device is listed as: its variant, and the release its bitstream
/// says it is of.
fn version_of(id: &Id, description: Option<&Description>) -> String {
    match description.filter(|d| d.build != [0; 3]) {
        Some(d) => format!("{} {}", id.variant(), d.build()),
        None => id.variant(),
    }
}

/// The `oxla` driver.
pub struct OxlaDriver;

impl Driver for OxlaDriver {
    fn id(&self) -> &'static str {
        "oxla"
    }

    fn long_name(&self) -> &'static str {
        "oxla logic analyzer gateware"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = protocol::SERIALCOMM.to_owned();
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
        // The port must be named: probing one resets what is on it.
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let params = Params::parse(&serialcomm)?;
        log::info!("Opening {conn}.");
        let mut port = crate::serial::open(&conn, params)?;
        let Some(found) = probe(port.as_mut())? else {
            return Ok(Vec::new());
        };
        let id = &found.id;
        let clocks: Vec<String> =
            found.clocks.settings().iter().map(|c| c.hz.to_string()).collect();
        log::info!(
            "Found {}: {} channels, clocks of {} Hz, {} words of memory.",
            id.variant(),
            id.channels,
            clocks.join(", "),
            id.mem_words
        );
        Ok(vec![Box::new(OxlaDevice::with_params(found, conn, params, Some(port)))])
    }
}

/// What gets the gateware onto a board that does not run it, which an open
/// does first: the load of a bitstream, and the port the board then
/// answers on.
pub type Loader = Box<dyn FnMut() -> Result<Box<dyn Port>> + Send>;

/// What a board gets after a load before it is asked what it is, and
/// between two times it is asked: its PLLs lock and its memory starts, in
/// some milliseconds.
pub const LOAD_SETTLE: Duration = Duration::from_millis(300);
/// The times a board is asked what it is after a load.
const LOAD_PROBES: u32 = 4;

/// One analyzer.
pub struct OxlaDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    id: Id,
    /// Where every rate comes from: the clocks the device reported.
    clocks: Clocks,
    /// What the device says of itself, where it does.
    description: Option<Description>,
    conn: String,
    params: Params,
    /// The port of the scan, which the open takes.
    pending: Option<Box<dyn Port>>,
    /// The load the board needs before it is this device.
    load: Option<Loader>,
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    rate: Rate,
    limit_samples: u64,
    capture_ratio: u64,
    rle: bool,
    test_pattern: bool,
    trigger: Option<Trigger>,
    worker: Option<Worker>,
    pace: Pace,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl OxlaDevice {
    /// The device a [`probe`] `found` on `port`: the port of a scan, or one
    /// that stands in for the hardware. [`Device::open`] takes it.
    pub fn new(found: Found, conn: String, port: Box<dyn Port>) -> Self {
        let params = Params::parse(protocol::SERIALCOMM).expect("the default parses");
        Self::with_params(found, conn, params, Some(port))
    }

    fn with_params(
        found: Found,
        conn: String,
        params: Params,
        pending: Option<Box<dyn Port>>,
    ) -> Self {
        let Found { id, clocks, description } = found;
        let rate = clocks.fastest();
        OxlaDevice {
            info: DeviceInfo {
                driver: "oxla",
                vendor: "oxrok".to_owned(),
                model: "oxla".to_owned(),
                version: version_of(&id, description.as_ref()),
                serial: String::new(),
                conn: Some(conn.clone()),
            },
            channels: (0..usize::from(id.channels)).map(Channel::logic).collect(),
            id,
            clocks,
            description,
            conn,
            params,
            pending,
            load: None,
            port: Arc::new(Mutex::new(None)),
            rate,
            limit_samples: DEFAULT_SAMPLES,
            capture_ratio: 0,
            rle: false,
            test_pattern: false,
            trigger: None,
            worker: None,
            pace: PACE,
        }
    }

    /// The device a board at `conn` becomes once `load` has put on it the
    /// bitstream that answers as `id`, with clocks of `clocks_hz` for the
    /// values of `CLOCK` from 0. Nothing is loaded until it is opened, and
    /// from then on it is what the board says.
    pub fn to_load(id: Id, clocks_hz: &[u32], conn: String, load: Loader) -> Self {
        let settings = clocks_hz
            .iter()
            .zip(0..)
            .map(|(hz, select)| ClockSetting { select: Some(select), hz: *hz })
            .collect();
        let found = Found { id, clocks: Clocks::new(&id, settings), description: None };
        let params = Params::parse(protocol::SERIALCOMM).expect("the default parses");
        let mut device = Self::with_params(found, conn, params, None);
        device.load = Some(load);
        device
    }

    /// Names the driver and the board the device is listed with.
    #[must_use]
    pub fn listed_as(mut self, driver: &'static str, model: &str, serial: &str) -> Self {
        self.info.driver = driver;
        model.clone_into(&mut self.info.model);
        serial.clone_into(&mut self.info.serial);
        self
    }

    /// Whether an open loads a bitstream first.
    pub fn needs_load(&self) -> bool {
        self.load.is_some()
    }

    /// What the device says of itself beyond its ID, where it does.
    pub fn description(&self) -> Option<&Description> {
        self.description.as_ref()
    }

    /// What the device said it is.
    pub fn id(&self) -> &Id {
        &self.id
    }

    fn lock_port(&self) -> std::sync::MutexGuard<'_, Option<Box<dyn Port>>> {
        self.port.lock().expect("oxla lock")
    }

    fn channel_mask(&self) -> u32 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | (1 << c.index))
    }

    /// The most samples a capture can be asked for.
    fn max_samples(&self) -> u64 {
        if self.rle { u64::from(u32::MAX) } else { self.id.raw_capacity() }
    }

    /// The capture the settings and `trigger` make.
    fn plan(&self, trigger: Option<&Trigger>) -> Result<Plan> {
        let settings = protocol::Settings {
            rate: self.rate,
            limit_samples: self.limit_samples,
            capture_ratio: self.capture_ratio,
            rle: self.rle,
            test_pattern: self.test_pattern,
            channel_mask: self.channel_mask(),
        };
        protocol::plan(&self.id, &settings, trigger).map_err(Error::Arg)
    }
}

/// Resets the device of `id`, sets the clock of `plan` when it has another,
/// writes the registers and arms it.
fn arm(port: &mut dyn Port, id: &Id, plan: &Plan, pace: Pace) -> Result<()> {
    resync(port)?;
    // What the device says now, not what it was left at: another program
    // may have had it since.
    let now = read_id(port, pace)?;
    if !now.same_board(id) {
        return Err(Error::device(format!(
            "the device is now a {}, where a {} was found: scan again",
            now.variant(),
            id.variant()
        )));
    }
    let clock = plan.clock;
    if now.clock_hz != clock.hz {
        let Some(select) = clock.select else {
            return Err(Error::device(format!(
                "the device's clock is {} Hz, where it was {}",
                now.clock_hz, clock.hz
            )));
        };
        log::debug!("Clock setting {select}, for {} Hz.", clock.hz);
        let at = retune(port, select, pace)?.clock_hz;
        if at != clock.hz {
            return Err(Error::device(format!(
                "clock setting {select} is {at} Hz, where it was {}: scan again",
                clock.hz
            )));
        }
    }
    for (addr, value) in &plan.registers {
        log::debug!("Register {addr:#04x} = {value:#x}.");
        port.write(&protocol::wreg(*addr, *value))?;
    }
    port.write(&[protocol::OP_ARM])
}

impl Device for OxlaDevice {
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
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        c.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let c = self.channels.get_mut(pos).ok_or_else(|| Error::arg("no such channel"))?;
        name.clone_into(&mut c.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let mut options = vec![
            OptionDesc::new(Key::Conn, Access::GET),
            OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
            OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
            OptionDesc::new(Key::TriggerMatch, Access::LIST),
            OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
            OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
        ];
        if self.id.has_rle() {
            options.push(OptionDesc::new(Key::Rle, Access::GET_SET));
        }
        Ok(options)
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Conn => Value::Str(self.conn.clone()),
            Key::Samplerate => Value::U64(self.rate.hz()),
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            Key::PatternMode => Value::Str(
                if self.test_pattern { PATTERN_INTERNAL } else { PATTERN_NONE }.to_owned(),
            ),
            Key::Rle if self.id.has_rle() => Value::Bool(self.rle),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::Samplerate => {
                let hz = value.as_u64().ok_or_else(bad)?;
                let Some(rate) = self.clocks.nearest(hz, self.rate) else {
                    return Err(Error::Arg(format!(
                        "the samplerate is {} to {} Hz",
                        self.clocks.slowest().hz().max(1),
                        self.clocks.fastest().hz()
                    )));
                };
                let real = rate.hz();
                if real != hz {
                    log::info!("Can't match samplerate {hz}, using {real}.");
                }
                self.rate = rate;
            }
            Key::LimitSamples => {
                let n = value.as_u64().ok_or_else(bad)?;
                // What the memory holds depends on the mode, so that is
                // looked at when the capture starts.
                if n == 0 || n > u64::from(u32::MAX) {
                    return Err(bad());
                }
                self.limit_samples = n;
            }
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                if r > 100 {
                    return Err(bad());
                }
                self.capture_ratio = r;
            }
            Key::PatternMode => {
                self.test_pattern = match value.as_str().ok_or_else(bad)? {
                    PATTERN_NONE => false,
                    PATTERN_INTERNAL => true,
                    _ => return Err(bad()),
                };
            }
            Key::Rle if self.id.has_rle() => self.rle = value.as_bool().ok_or_else(bad)?,
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            // Any rate is taken, and the nearest a clock gives is set.
            Key::Samplerate => ValueList::RangeStepU64 {
                min: self.clocks.slowest().hz().max(1),
                max: self.clocks.fastest().hz(),
                step: 1,
            },
            Key::LimitSamples => ValueList::MinMaxU64(1, self.max_samples()),
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::PatternMode => {
                ValueList::Str([PATTERN_NONE, PATTERN_INTERNAL].map(str::to_owned).to_vec())
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn trigger_features(&self) -> TriggerFeatures {
        FEATURES
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        if let Some(t) = &trigger {
            t.check(&FEATURES)?;
            protocol::trigger_setup(Some(t), self.id.channels, self.channel_mask())
                .map_err(Error::Arg)?;
        }
        self.trigger = trigger;
        Ok(())
    }

    fn check_capture(&self, trigger: Option<&Trigger>) -> Result<()> {
        self.plan(trigger).map(|_| ())
    }

    fn open(&mut self) -> Result<()> {
        if self.lock_port().is_some() {
            return Ok(());
        }
        let port = if let Some(port) = self.pending.take() {
            port
        } else if let Some(load) = self.load.as_mut() {
            log::info!("Loading the {} bitstream.", self.id.variant());
            let mut port = load()?;
            // What the board's bridge still held of the design that ran
            // before can come in place of the first answers.
            // An answer of the variant that ran before is one of those too.
            // The variant is what was asked for; the rest is the board's
            // to say.
            let variant = self.id.variant();
            let mut found = None;
            let mut failed = None;
            for _ in 0..LOAD_PROBES {
                // An answer cut short is asked for again, and not by a
                // second load at the next open; a port that fails is not.
                match probe_logging(port.as_mut(), self.pace, log::Level::Debug, Some(&variant)) {
                    Ok(answer) => (found, failed) = (answer, None),
                    Err(e @ (Error::Device(_) | Error::Timeout)) => failed = Some(e),
                    Err(e) => return Err(e),
                }
                if found.is_some() {
                    break;
                }
                std::thread::sleep(self.pace.answer.min(LOAD_SETTLE));
            }
            let Some(found) = found else {
                if let Some(e) = failed {
                    return Err(e);
                }
                return Err(Error::device(format!(
                    "{} does not answer as a {} after the load of its bitstream",
                    self.conn,
                    self.id.variant()
                )));
            };
            self.load = None;
            self.id = found.id;
            self.rate = found.clocks.fastest();
            self.clocks = found.clocks;
            self.info.version = version_of(&found.id, found.description.as_ref());
            self.description = found.description;
            port
        } else {
            let mut port = crate::serial::open(&self.conn, self.params)?;
            let found = probe_within(port.as_mut(), self.pace)?;
            let Some(found) = found.filter(|f| f.id.same_board(&self.id)) else {
                return Err(Error::device(format!(
                    "{} is no longer the {} that was found",
                    self.conn,
                    self.id.variant()
                )));
            };
            // The rate as it was, where its clock still is.
            if !found.clocks.settings().contains(&self.rate.clock) {
                self.rate = found.clocks.fastest();
            }
            self.clocks = found.clocks;
            self.description = found.description;
            port
        };
        *self.lock_port() = Some(port);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        drop(self.lock_port().take());
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // The thread gives the port back ahead of End, so a client that has
        // seen End finds it here, where `is_finished` may still say no.
        if self.worker.is_some() {
            if self.lock_port().is_none() {
                return Err(Error::Busy);
            }
            self.worker = None;
        }
        let plan = self.plan(self.trigger.as_ref())?;
        let Some(mut port) = self.lock_port().take() else {
            return Err(Error::device("device is not open"));
        };
        if let Err(e) = arm(port.as_mut(), &self.id, &plan, self.pace) {
            *self.lock_port() = Some(port);
            return Err(e);
        }
        let capture = Capture { id: self.id, plan, pace: self.pace };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("oxla acquisition".into())
            .spawn(move || {
                let sink = Patient { sink: &sink, stop: &stop2, stopped_at: Cell::new(None) };
                let outcome = capture.run(port.as_mut(), &sink, &stop2);
                if let Err(e) = &outcome {
                    log::error!("The capture failed: {e}.");
                }
                *slot.lock().expect("oxla lock") = Some(port);
                sink.end(outcome);
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

/// What a queue nobody reads gets after a stop before a packet is given
/// up: a close joins the thread that waits.
const SEND_AFTER_STOP: Duration = Duration::from_millis(500);

/// A sink that waits for room in the queue for as long as nobody asked to
/// stop, and after a stop for [`SEND_AFTER_STOP`] in all: what a stopped
/// capture had still goes to a reader that reads, and a thread a close
/// joins does not wait for ever on one that does not.
struct Patient<'a> {
    sink: &'a PacketSink,
    stop: &'a AtomicBool,
    stopped_at: Cell<Option<Instant>>,
}

impl Patient<'_> {
    fn go_on(&self) -> bool {
        if !self.stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = self.stopped_at.get().unwrap_or_else(Instant::now);
        self.stopped_at.set(Some(at));
        at.elapsed() < SEND_AFTER_STOP
    }

    fn send(&self, packet: Packet) -> bool {
        self.sink.send_while(packet, || self.go_on())
    }

    fn end(&self, outcome: Result<()>) -> bool {
        self.sink.end_while(outcome, || self.go_on())
    }
}

/// One capture on its own thread, from the armed device to the last
/// packet.
struct Capture {
    id: Id,
    plan: Plan,
    pace: Pace,
}

impl Capture {
    fn run(&self, port: &mut dyn Port, sink: &Patient, stop: &AtomicBool) -> Result<()> {
        sink.send(Packet::header());
        let (status, stopped) = self.wait(port, stop)?;
        let mut read = |addr| rreg(port, addr, self.pace.answer);
        let ended = Ended {
            status,
            trig: read(protocol::REG_TRIG)?,
            stop: read(protocol::REG_STOP)?,
            count: read(protocol::REG_COUNT)?,
        };
        log::debug!(
            "Ended: status {:#x}, trigger {:#x}, stop {:#x}, {} words.",
            ended.status,
            ended.trig,
            ended.stop,
            ended.count
        );
        if ended.overflowed() {
            let advice = if self.plan.rle {
                "run length encoding takes up to a word per sample, which the memory \
                 has time for from half the clock rate down"
            } else {
                "lower the sample rate"
            };
            return Err(Error::device(format!(
                "the sample memory overflowed, so the capture is not whole: {advice}"
            )));
        }
        let window = protocol::window(&self.id, &self.plan, &ended);
        // A stop that ended the wait wants what the device has. One that
        // comes during the read ends the read.
        let stop = (!stopped).then_some(stop);
        let (words, whole) = self.read_memory(port, &window, stop)?;
        self.send(sink, &ended, &window, &words, whole);
        Ok(())
    }

    /// Waits for the capture to end and gives its status, and whether a
    /// stop ended it: the device's STOP went out.
    fn wait(&self, port: &mut dyn Port, stop: &AtomicBool) -> Result<(u32, bool)> {
        let mut stopped = false;
        let mut polls = 0;
        loop {
            let status = rreg(port, protocol::REG_STATUS, self.pace.answer)?;
            if status & protocol::STATUS_DONE != 0 {
                return Ok((status, stopped));
            }
            if stopped {
                polls += 1;
                if polls > STOP_POLLS {
                    return Err(Error::device("the capture did not end at the device's STOP"));
                }
            } else if stop.load(Ordering::SeqCst) {
                log::debug!("Stopping the capture.");
                port.write(&[protocol::OP_STOP])?;
                stopped = true;
            }
            std::thread::sleep(self.pace.poll);
        }
    }

    /// Reads the words of `window`, a chunk a request, and says whether
    /// that is all of them: `stop` ends the read between two chunks.
    fn read_memory(
        &self,
        port: &mut dyn Port,
        window: &Window,
        stop: Option<&AtomicBool>,
    ) -> Result<(Vec<u32>, bool)> {
        let mut words = Vec::with_capacity(window.words as usize);
        let mut buf = vec![0u8; protocol::RMEM_CHUNK as usize * 4];
        for (addr, n) in protocol::rmem_chunks(window.start, window.words, self.id.mem_words) {
            if stop.is_some_and(|s| s.load(Ordering::SeqCst)) {
                return Ok((words, false));
            }
            port.write(&protocol::rmem(addr, n))?;
            let chunk = &mut buf[..n as usize * 4];
            let got = read_answer(port, chunk, self.pace.answer)?;
            if got < chunk.len() {
                return Err(Error::device(format!(
                    "the memory read went silent after {got} of the {} bytes of the {n} words \
                     at {addr}, {} of {} words read",
                    chunk.len(),
                    words.len(),
                    window.words
                )));
            }
            words.extend(chunk.as_chunks::<4>().0.iter().map(|b| u32::from_le_bytes(*b)));
        }
        Ok((words, true))
    }

    /// Sends the samples of `words`, the part of them that was wanted,
    /// with the trigger marked when the user set one.
    fn send(&self, sink: &Patient, ended: &Ended, window: &Window, words: &[u32], whole: bool) {
        let w = self.id.channels;
        let (pre, post) = (self.plan.pre, self.plan.post);
        let mut out = Emitter::new(sink, self.id.unit_size(), self.plan.channel_mask as u16);
        if self.plan.rle {
            let words = if whole { protocol::rle_runs(words, ended.last_bits()) } else { words };
            // The trigger sample starts its word.
            let trigger = window
                .trigger
                .filter(|(word, _)| (*word as usize) < words.len())
                .map(|(word, _)| protocol::rle_len(&words[..word as usize], w));
            out.cut(protocol::cut(protocol::rle_len(words, w), trigger, pre, post), &self.plan);
            for word in words {
                let (value, count) = protocol::rle_run(*word, w);
                out.push(value, count);
            }
        } else {
            // Short of the end every word read is whole.
            let last_bits = if whole { ended.last_bits() } else { 32 };
            let samples = protocol::unpack_raw(words, window.start, w, last_bits);
            let trigger = window
                .trigger
                .map(|(word, sub)| protocol::raw_trigger_index(window.start, word, sub, w));
            out.cut(protocol::cut(samples.len() as u64, trigger, pre, post), &self.plan);
            for value in samples {
                out.push(value, 1);
            }
        }
        out.flush();
    }
}

/// Makes logic packets of runs of samples: those of a [`Cut`], the trigger
/// packet ahead of its sample.
struct Emitter<'a> {
    sink: &'a Patient<'a>,
    unit: usize,
    /// The enabled channels: the others are sent low.
    mask: u16,
    skip: u64,
    left: u64,
    /// Samples to send before the trigger packet.
    marker: Option<u64>,
    buf: Vec<u8>,
    /// Someone still listens.
    heard: bool,
}

impl<'a> Emitter<'a> {
    fn new(sink: &'a Patient<'a>, unit: usize, mask: u16) -> Self {
        Emitter {
            sink,
            unit,
            mask,
            skip: 0,
            left: 0,
            marker: None,
            buf: Vec::with_capacity(PACKET_BYTES),
            heard: true,
        }
    }

    fn cut(&mut self, cut: Cut, plan: &Plan) {
        self.skip = cut.skip;
        self.left = cut.take;
        self.marker = cut.trigger.filter(|_| plan.marked);
    }

    fn push(&mut self, value: u16, mut count: u64) {
        let dropped = count.min(self.skip);
        self.skip -= dropped;
        count = (count - dropped).min(self.left);
        self.left -= count;
        let bytes = (value & self.mask).to_le_bytes();
        while count > 0 && self.heard {
            if self.marker == Some(0) {
                self.flush();
                self.heard &= self.sink.send(Packet::Trigger);
                self.marker = None;
            }
            let room = ((PACKET_BYTES - self.buf.len()) / self.unit) as u64;
            let n = count.min(room).min(self.marker.unwrap_or(u64::MAX));
            for _ in 0..n {
                self.buf.extend_from_slice(&bytes[..self.unit]);
            }
            if let Some(m) = &mut self.marker {
                *m -= n;
            }
            count -= n;
            if self.buf.len() >= PACKET_BYTES {
                self.flush();
            }
        }
    }

    fn flush(&mut self) {
        if !self.buf.is_empty() {
            let data = std::mem::replace(&mut self.buf, Vec::with_capacity(PACKET_BYTES));
            self.heard &= self.sink.send(Packet::logic(self.unit, data));
        }
    }
}
