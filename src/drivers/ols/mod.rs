// Counterpart in sigrok:
//   libsigrok/src/hardware/openbench-logic-sniffer/api.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, v1ne, Lars-Peter Clausen, Gerhard Sittig,
//   Wolfram Sang, Martin Ling, Soeren Apel, Daniel Elstner,
//   Gwenhael Goavec-Merou, Ken Taylor, Tilman Sauerbeck
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `ols`: the Open Bench Logic Sniffer and other SUMP analyzers on a serial
//! port, up to 32 channels.
//!
//! `conn=` names the port, `serialcomm=` overrides 115200/8n1, `probe_names=`
//! names the channels. A scan sends five resets and the ID command, waits
//! 20 ms for its reply, asks for the metadata and waits 20 ms for that, and
//! closes the port; opening the device opens it again. A capture writes its
//! commands, each one drained, then waits for the first byte for as long as
//! it takes, reads until the sample limit or 100 ms with nothing more, sends
//! the samples as one block, oldest first (see [`protocol`]), and resets.
//!
//! The C driver has no error path for a device that stops reading: its drain
//! never ends. Here a drain gives up after [`DRAIN_TIMEOUT`]. Also unlike the
//! C driver: a board with no channel count works, where its sample size of 0
//! keeps sigrok-cli's output in a loop; a capture with no sample ends with no
//! logic packet, where an empty one crashes it; a channel count beyond 32 is
//! 32, where it runs off C's sample; and a board below the default of 200 kHz
//! starts at the highest rate the clock gives at or below its maximum. Stopping a capture drops what came, as C does.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::util::parse_probe_names;
use crate::core::{
    Access, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Trigger, TriggerMatch, Value, ValueList,
};
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use protocol::{
    CMD_ID, CMD_METADATA, CMD_RESET, Decoder, FLAG_CLOCK_EXTERNAL, FLAG_DEMUX,
    FLAG_EXTERNAL_TEST_MODE, FLAG_FILTER, FLAG_INTERNAL_TEST_MODE, FLAG_INVERT_EXT_CLOCK, FLAG_RLE,
    FLAG_SWAP_CHANNELS, MIN_NUM_SAMPLES, Metadata, SAMPLERATE_MAX, SAMPLERATE_MIN,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SERIALCOMM: &str = "115200/8n1";
/// `RESPONSE_DELAY_US`.
const RESPONSE_DELAY: Duration = Duration::from_millis(20);
/// The session source's timeout: silence this long ends a capture that has
/// begun.
const END_OF_DATA: Duration = Duration::from_millis(100);
/// How long a read waits between looks at the stop flag.
const STOP_CHECK: Duration = Duration::from_millis(20);
/// `serial_drain` has no bound; a command takes well below this at any rate.
pub const DRAIN_TIMEOUT: Duration = Duration::from_secs(1);
const NUM_CHANNELS: usize = 32;
const PATTERN_NONE: &str = "None";
const PATTERN_EXTERNAL: &str = "External";
const PATTERN_INTERNAL: &str = "Internal";
const EDGE_RISING: &str = "rising";
const EDGE_FALLING: &str = "falling";

const SCAN_OPTIONS: [Key; 3] = [Key::Conn, Key::Serialcomm, Key::ProbeNames];
static CLASSES: [DeviceClass; 1] = [DeviceClass::LogicAnalyzer];
static DEV_OPTIONS: [OptionDesc; 10] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET_LIST),
    OptionDesc::new(Key::Samplerate, Access::GET_SET_LIST),
    OptionDesc::new(Key::TriggerMatch, Access::LIST),
    OptionDesc::new(Key::CaptureRatio, Access::GET_SET),
    OptionDesc::new(Key::ExternalClock, Access::GET_SET),
    OptionDesc::new(Key::ClockEdge, Access::GET_SET_LIST),
    OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
    OptionDesc::new(Key::Swap, Access { get: false, set: true, list: false }),
    OptionDesc::new(Key::Rle, Access::GET_SET),
];
static TRIGGER_MATCHES: [TriggerMatch; 2] = [TriggerMatch::Zero, TriggerMatch::One];

/// `send_shortcommand` and `send_longcommand`: the write and the drain.
fn send(port: &mut dyn Port, command: &[u8]) -> Result<()> {
    match command {
        [cmd] => log::debug!("Sending cmd 0x{cmd:02x}."),
        [cmd, a, b, c, d] => {
            log::debug!("Sending cmd 0x{cmd:02x} data 0x{a:02x}{b:02x}{c:02x}{d:02x}.");
        }
        _ => {}
    }
    port.write(command)?;
    port.drain(DRAIN_TIMEOUT)
}

/// `ols_send_reset`.
fn send_reset(port: &mut dyn Port) -> Result<()> {
    for _ in 0..5 {
        send(port, &[CMD_RESET])?;
    }
    Ok(())
}

/// What a scan found.
struct Found {
    vendor: String,
    metadata: Option<Metadata>,
}

/// `serial_has_receive_data` after the delay: a byte, if one is there. A
/// port that fails has none; the next step sees to that.
fn first_byte(port: &mut dyn Port) -> Option<u8> {
    std::thread::sleep(RESPONSE_DELAY);
    let mut b = [0];
    match read_blocking(port, &mut b, Duration::ZERO) {
        Ok(1) => Some(b[0]),
        _ => None,
    }
}

/// `scan` after the open: `None` is no device.
fn probe(port: &mut dyn Port, params: Params, conn: &str) -> Option<Found> {
    log::info!("Probing {conn}.");
    if send_reset(port).is_err() {
        log::error!("Could not use port {conn}. Quitting.");
        return None;
    }
    let _ = send(port, &[CMD_ID]);
    let Some(first) = first_byte(port) else {
        log::debug!("Didn't get any ID reply.");
        return None;
    };
    let mut id = [first, 0, 0, 0];
    let got = match read_blocking(port, &mut id[1..], serial_timeout(&params, 4)) {
        Ok(n) => n + 1,
        Err(e) => {
            log::error!("Getting ID reply failed ({e}).");
            return None;
        }
    };
    if &id != b"1SLO" && &id != b"1ALS" {
        let shown: Vec<String> = id[..got].iter().map(|b| format!("{b:02x}")).collect();
        log::error!("Invalid ID reply (got {}).", shown.join(" "));
        return None;
    }
    log::debug!("Successful detection, got '{}'.", String::from_utf8_lossy(&id));
    let _ = send(port, &[CMD_METADATA]);
    let Some(first) = first_byte(port) else {
        log::info!("Device does not support metadata.");
        return Some(Found { vendor: "Sump".to_owned(), metadata: None });
    };
    let mut first = Some(first);
    // A byte's time for each byte: C allows a 32 bit value `serial_timeout`
    // of 4 bytes in all, so a board that stalls inside one gets a little more
    // here. Nothing on the wire shows it.
    let timeout = serial_timeout(&params, 1);
    let mut metadata = protocol::read_metadata(|| {
        first.take().or_else(|| {
            let mut b = [0];
            matches!(read_blocking(port, &mut b, timeout), Ok(1)).then_some(b[0])
        })
    });
    protocol::apply_quirks(&mut metadata);
    Some(Found { vendor: String::new(), metadata: Some(metadata) })
}

/// The `ols` driver.
pub struct OlsDriver;

impl Driver for OlsDriver {
    fn id(&self) -> &'static str {
        "ols"
    }

    fn long_name(&self) -> &'static str {
        "Openbench Logic Sniffer & SUMP compatibles"
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
        let mut probe_names = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => {
                    value
                        .as_str()
                        .ok_or_else(|| Error::arg("serialcomm"))?
                        .clone_into(&mut serialcomm);
                }
                Key::ProbeNames => probe_names = value.as_str().map(str::to_owned),
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
        let (mut port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let Some(found) = probe(port.as_mut(), params, &conn) else {
            return Ok(Vec::new());
        };
        Ok(vec![Box::new(OlsDevice::new(found, conn, params, probe_names.as_deref()))])
    }
}

/// One analyzer.
pub struct OlsDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    params: Params,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    unit_size: usize,
    max_samples: u32,
    max_samplerate: u32,
    samplerate: u64,
    divider: u32,
    flags: u16,
    limit_samples: u64,
    capture_ratio: u64,
    trigger: Option<Trigger>,
    port: Arc<Mutex<Option<Box<dyn Port>>>>,
    worker: Option<Worker>,
    /// The silence that ends a capture: [`END_OF_DATA`], or more for a test
    /// that must not race it.
    end_of_data: Duration,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl OlsDevice {
    fn new(found: Found, conn: String, params: Params, probe_names: Option<&str>) -> Self {
        let defaults: Vec<String> = (0..NUM_CHANNELS).map(|i| i.to_string()).collect();
        let defaults: Vec<&str> = defaults.iter().map(String::as_str).collect();
        let mut names = match probe_names {
            Some(spec) => parse_probe_names(spec, &defaults),
            None => defaults.iter().map(|s| (*s).to_owned()).collect(),
        };
        let metadata = found.metadata.as_ref();
        let (model, version) = metadata.map_or_else(
            || ("Logic Analyzer".to_owned(), "v1.0".to_owned()),
            |m| (m.name.clone(), m.version.clone()),
        );
        let counted = metadata.map_or(0, |m| m.channels);
        if counted != 0 {
            names.truncate(counted);
        }
        // `protocol.c:304`'s `unitsize` comes only from a channel count in
        // the metadata; 0 otherwise, which no output survives.
        let width = if counted == 0 { names.len() } else { counted };
        let mut dev = OlsDevice {
            info: DeviceInfo {
                driver: "ols",
                vendor: found.vendor,
                model,
                version,
                serial: String::new(),
                conn: Some(conn.clone()),
            },
            channels: names
                .into_iter()
                .enumerate()
                .map(|(i, name)| Channel::new(i, ChannelKind::Logic, name))
                .collect(),
            conn,
            params,
            pending: None,
            unit_size: width.div_ceil(8).max(1),
            max_samples: metadata.map_or(0, |m| m.max_samplebytes),
            max_samplerate: metadata.map_or(0, |m| m.max_samplerate),
            samplerate: 0,
            divider: 0,
            flags: 0,
            limit_samples: 0,
            capture_ratio: 0,
            trigger: None,
            port: Arc::new(Mutex::new(None)),
            worker: None,
            end_of_data: END_OF_DATA,
        };
        dev.use_rate(protocol::starting_rate(dev.max_samplerate));
        dev
    }

    /// A device of `metadata` on a port that stands in for the hardware,
    /// without a scan's reading. [`Device::open`] takes it.
    pub fn from_port(metadata: Option<Metadata>, port: Box<dyn Port>) -> Self {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let found = Found { vendor: String::new(), metadata };
        let mut dev = Self::new(found, conn, params, None);
        dev.pending = Some(port);
        dev
    }

    /// `ols_set_samplerate()`.
    fn set_samplerate(&mut self, rate: u64) -> Result<()> {
        let Some(r) = protocol::samplerate(rate, self.max_samplerate) else {
            return Err(Error::arg("invalid samplerate"));
        };
        self.use_rate(r);
        if r.samplerate != rate {
            log::info!("Can't match samplerate {rate}, using {}.", r.samplerate);
        }
        Ok(())
    }

    fn use_rate(&mut self, r: protocol::Rate) {
        if r.demux {
            log::info!("Enabling demux mode.");
            self.flags |= FLAG_DEMUX;
            self.flags &= !FLAG_FILTER;
        } else {
            log::info!("Disabling demux mode.");
            self.flags &= !FLAG_DEMUX;
            self.flags |= FLAG_FILTER;
        }
        self.divider = r.divider;
        self.samplerate = r.samplerate;
    }

    /// `ols_channel_mask()`.
    fn channel_mask(&self) -> u32 {
        self.channels.iter().filter(|c| c.enabled).fold(0, |m, c| m | (1 << c.index))
    }

    fn flag(&mut self, flag: u16, on: bool) {
        if on {
            self.flags |= flag;
        } else {
            self.flags &= !flag;
        }
    }

    fn lock_port(&self) -> std::sync::MutexGuard<'_, Option<Box<dyn Port>>> {
        self.port.lock().expect("ols lock")
    }

    /// `ols_prepare_acquisition()` and the run command.
    fn arm(&mut self, port: &mut dyn Port) -> Result<protocol::Plan> {
        let channel_mask = self.channel_mask();
        let stages =
            protocol::convert_trigger(self.trigger.as_ref(), channel_mask).map_err(|m| {
                log::error!("{m}");
                log::error!("Failed to configure channels.");
                Error::Arg(m)
            })?;
        let settings = protocol::Settings {
            flags: self.flags,
            channel_mask,
            max_samples: self.max_samples,
            limit_samples: self.limit_samples,
            capture_ratio: self.capture_ratio,
            divider: self.divider,
        };
        let plan = protocol::plan(&settings, &stages).map_err(|m| {
            log::error!("{m}");
            Error::Arg(m)
        })?;
        log::debug!("Setting samplerate to {}Hz (divider {})", self.samplerate, self.divider);
        self.flags = plan.flags;
        for command in &plan.commands {
            send(port, command)?;
        }
        Ok(plan)
    }
}

impl Device for OlsDevice {
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
        match group {
            None => Ok(DEV_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Conn => Value::Str(self.conn.clone()),
            Key::Samplerate => Value::U64(self.samplerate),
            Key::CaptureRatio => Value::U64(self.capture_ratio),
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::PatternMode => Value::Str(
                if self.flags & FLAG_EXTERNAL_TEST_MODE != 0 {
                    PATTERN_EXTERNAL
                } else if self.flags & FLAG_INTERNAL_TEST_MODE != 0 {
                    PATTERN_INTERNAL
                } else {
                    PATTERN_NONE
                }
                .to_owned(),
            ),
            Key::Rle => Value::Bool(self.flags & FLAG_RLE != 0),
            Key::ExternalClock => Value::Bool(self.flags & FLAG_CLOCK_EXTERNAL != 0),
            Key::ClockEdge => Value::Str(
                if self.flags & FLAG_INVERT_EXT_CLOCK != 0 { EDGE_FALLING } else { EDGE_RISING }
                    .to_owned(),
            ),
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
                let rate = value.as_u64().ok_or_else(bad)?;
                if !(SAMPLERATE_MIN..=SAMPLERATE_MAX).contains(&rate) {
                    return Err(Error::arg("invalid samplerate"));
                }
                self.set_samplerate(rate)?;
            }
            Key::LimitSamples => {
                let n = value.as_u64().ok_or_else(bad)?;
                if n < MIN_NUM_SAMPLES {
                    return Err(Error::arg("fewer samples than the device takes"));
                }
                self.limit_samples = n;
            }
            Key::CaptureRatio => {
                let r = value.as_u64().ok_or_else(bad)?;
                // hwdriver.c's check of the key, before the driver.
                if r > 100 {
                    return Err(bad());
                }
                self.capture_ratio = r;
            }
            Key::ExternalClock => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::info!(
                    "{}",
                    if on { "Enabling external clock." } else { "Disabled external clock." }
                );
                self.flag(FLAG_CLOCK_EXTERNAL, on);
            }
            Key::ClockEdge => {
                // Whatever is not "falling" is rising, as `config_set` has it.
                let falling = value.as_str().ok_or_else(bad)? == EDGE_FALLING;
                log::info!(
                    "Triggering on {} edge of external clock.",
                    if falling { EDGE_FALLING } else { EDGE_RISING }
                );
                self.flag(FLAG_INVERT_EXT_CLOCK, falling);
            }
            Key::PatternMode => {
                let flag = match value.as_str().ok_or_else(bad)? {
                    PATTERN_NONE => {
                        log::info!("Disabling test modes.");
                        0
                    }
                    PATTERN_INTERNAL => {
                        log::info!("Enabling internal test mode.");
                        FLAG_INTERNAL_TEST_MODE
                    }
                    PATTERN_EXTERNAL => {
                        log::info!("Enabling external test mode.");
                        FLAG_EXTERNAL_TEST_MODE
                    }
                    _ => return Err(bad()),
                };
                self.flags &= !(FLAG_INTERNAL_TEST_MODE | FLAG_EXTERNAL_TEST_MODE);
                self.flags |= flag;
            }
            Key::Swap => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::info!(
                    "{}",
                    if on { "Enabling channel swapping." } else { "Disabling channel swapping." }
                );
                self.flag(FLAG_SWAP_CHANNELS, on);
            }
            Key::Rle => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::info!("{}", if on { "Enabling RLE." } else { "Disabling RLE." });
                self.flag(FLAG_RLE, on);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        Ok(match key {
            Key::Samplerate => ValueList::RangeStepU64 {
                min: SAMPLERATE_MIN,
                max: match self.max_samplerate {
                    0 => SAMPLERATE_MAX,
                    max => u64::from(max),
                },
                step: 1,
            },
            Key::TriggerMatch => ValueList::TriggerMatches(TRIGGER_MATCHES.to_vec()),
            Key::ClockEdge => {
                ValueList::Str([EDGE_RISING, EDGE_FALLING].map(str::to_owned).to_vec())
            }
            Key::PatternMode => ValueList::Str(
                [PATTERN_NONE, PATTERN_EXTERNAL, PATTERN_INTERNAL].map(str::to_owned).to_vec(),
            ),
            Key::LimitSamples => {
                if self.max_samples == 0 {
                    return Err(Error::not_supported("a sample limit without a memory size"));
                }
                let groups = protocol::group_mask(self.channel_mask()).count_ones();
                let max = self.max_samples.checked_div(groups).map_or(MIN_NUM_SAMPLES, u64::from);
                ValueList::MinMaxU64(MIN_NUM_SAMPLES, max)
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set_trigger(&mut self, trigger: Option<Trigger>) -> Result<()> {
        self.trigger = trigger;
        Ok(())
    }

    fn open(&mut self) -> Result<()> {
        if self.lock_port().is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(port) => port,
            None => crate::serial::open(&self.conn, self.params)?,
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
        // seen End finds it here, where `is_finished` may still say no. What
        // is left of the thread is that End: not joined, as a full queue
        // would hold it.
        if self.worker.is_some() {
            if self.lock_port().is_none() {
                return Err(Error::Busy);
            }
            self.worker = None;
        }
        let Some(mut port) = self.lock_port().take() else {
            return Err(Error::device("device is not open"));
        };
        let plan = match self.arm(port.as_mut()) {
            Ok(plan) => plan,
            Err(e) => {
                // libsigrok's session stops a device whose start failed:
                // the resets of `abort_acquisition`.
                let _ = send_reset(port.as_mut());
                *self.lock_port() = Some(port);
                return Err(e);
            }
        };
        let capture = Capture {
            decoder: Decoder::plain(plan.flags, self.limit_samples, plan.trigger_word),
            end_of_data: self.end_of_data,
            trigger_sample: plan.trigger_sample,
            whole: plan.whole,
            unit_size: self.unit_size,
        };
        let slot = Arc::clone(&self.port);
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("ols acquisition".into())
            .spawn(move || {
                let outcome = capture.run(port.as_mut(), &sink, &stop2);
                *slot.lock().expect("ols lock") = Some(port);
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

/// `ols_receive_data` on its own thread.
struct Capture {
    decoder: Decoder,
    /// [`OlsDevice::end_of_data`].
    end_of_data: Duration,
    /// [`protocol::Plan::trigger_sample`].
    trigger_sample: Option<u64>,
    /// [`protocol::Plan::whole`].
    whole: Option<u64>,
    unit_size: usize,
}

impl Capture {
    fn run(mut self, port: &mut dyn Port, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        sink.send(Packet::header());
        let mut outcome = Ok(());
        if let Some(read) = self.read(port, stop) {
            outcome = read;
            self.send(sink);
            let _ = port.purge();
        }
        let _ = send_reset(port);
        outcome
    }

    /// Whether silence ends the download as it should: the board sends a set
    /// number of samples and then nothing.
    fn silence_is_the_end(&self) -> Result<()> {
        match self.whole {
            Some(whole) if (self.decoder.len() as u64) < whole => Err(Error::device(format!(
                "the board went silent after {} of {whole} samples",
                self.decoder.len()
            ))),
            _ => Ok(()),
        }
    }

    /// Reads to the limit, or to the silence, or to a port that fails, and
    /// says how that went. `None` when stopped: a stop sends nothing of what
    /// came.
    fn read(&mut self, port: &mut dyn Port, stop: &AtomicBool) -> Option<Result<()>> {
        let mut buf = [0u8; 4096];
        let mut last = None;
        loop {
            if stop.load(Ordering::SeqCst) {
                return None;
            }
            if self.decoder.len() as u64 >= self.decoder.limit() {
                return Some(Ok(()));
            }
            match port.read(&mut buf, STOP_CHECK) {
                Ok(0) => {
                    if last.is_some_and(|at: Instant| at.elapsed() >= self.end_of_data) {
                        return Some(self.silence_is_the_end());
                    }
                }
                Ok(n) => {
                    self.decoder.feed(&buf[..n]);
                    last = Some(Instant::now());
                }
                // A hangup: C's source fires with no G_IO_IN and ends so.
                Err(e) => {
                    log::debug!("The port went: {e}.");
                    return Some(Err(e));
                }
            }
        }
    }

    fn send(self, sink: &PacketSink) {
        let unit = self.unit_size;
        let (samples, by_word) = self.decoder.finish();
        // Beyond what came is no marker, where C's length wraps.
        let by_sample = self
            .trigger_sample
            .and_then(|at| usize::try_from(at).ok())
            .filter(|&at| at <= samples.len());
        let trigger = by_word.or(by_sample);
        let bytes = |s: &[u32]| -> Vec<u8> {
            s.iter().flat_map(|v| v.to_le_bytes().into_iter().take(unit)).collect()
        };
        let at = trigger.unwrap_or(0);
        if at > 0 {
            sink.send(Packet::logic(unit, bytes(&samples[..at])));
        }
        if trigger.is_some() {
            sink.send(Packet::Trigger);
        }
        if at < samples.len() {
            sink.send(Packet::logic(unit, bytes(&samples[at..])));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::Params;
    use std::collections::VecDeque;
    use std::sync::mpsc::{Receiver, sync_channel};

    #[derive(Default)]
    struct Seen {
        /// Each write, with when it came.
        writes: Vec<(Instant, Vec<u8>)>,
        drains: usize,
        purges: usize,
        reads_from: Vec<Instant>,
    }

    /// What a write gets back: when, and the bytes.
    type Answer = fn(&[u8]) -> Option<(Duration, Vec<u8>)>;

    /// A board on a fake port: `answer` gives what a write gets, a byte at
    /// `(delay, bytes)`; a byte is there from its time on.
    struct Fake {
        seen: Arc<Mutex<Seen>>,
        input: VecDeque<(Instant, u8)>,
        answer: Answer,
        fail_writes_after: Option<usize>,
        fail_drains: bool,
        /// Reads fail from this much data read on: a hangup.
        hangup_after: Option<usize>,
        read: usize,
    }

    impl Fake {
        fn new(answer: Answer) -> (Fake, Arc<Mutex<Seen>>) {
            let seen = Arc::new(Mutex::new(Seen::default()));
            let fake = Fake {
                seen: Arc::clone(&seen),
                input: VecDeque::new(),
                answer,
                fail_writes_after: None,
                fail_drains: false,
                hangup_after: None,
                read: 0,
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
            self.seen.lock().unwrap().reads_from.push(Instant::now());
            if self.hangup_after.is_some_and(|n| self.read >= n) {
                return Err(Error::device("Broken pipe"));
            }
            let deadline = Instant::now() + timeout;
            loop {
                let now = Instant::now();
                let there = self.input.iter().take_while(|(at, _)| *at <= now).count();
                let room = self.hangup_after.map_or(buf.len(), |n| (n - self.read).min(buf.len()));
                let n = there.min(room);
                if n > 0 {
                    for b in &mut buf[..n] {
                        *b = self.input.pop_front().expect("counted").1;
                    }
                    self.read += n;
                    return Ok(n);
                }
                if now >= deadline {
                    return Ok(0);
                }
                let next = self.input.front().map_or(deadline, |(at, _)| *at);
                std::thread::sleep(next.min(deadline).saturating_duration_since(now));
            }
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let now = Instant::now();
            let mut seen = self.seen.lock().unwrap();
            if self.fail_writes_after.is_some_and(|n| seen.writes.len() >= n) {
                return Err(Error::device("Broken pipe"));
            }
            seen.writes.push((now, data.to_vec()));
            if let Some((delay, bytes)) = (self.answer)(data) {
                self.input.extend(bytes.into_iter().map(|b| (now + delay, b)));
            }
            Ok(())
        }
        fn drain(&mut self, _: Duration) -> Result<()> {
            self.seen.lock().unwrap().drains += 1;
            if self.fail_drains { Err(Error::Timeout) } else { Ok(()) }
        }
        fn purge(&mut self) -> Result<()> {
            self.seen.lock().unwrap().purges += 1;
            Ok(())
        }
    }

    fn params() -> Params {
        Params::parse(SERIALCOMM).unwrap()
    }

    /// What the plain board of the model answers.
    fn board(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
        match data {
            [CMD_ID] => Some((Duration::ZERO, b"1SLO".to_vec())),
            [CMD_METADATA] => Some((
                Duration::ZERO,
                b"\x01OLS\0\x21\0\0\x60\0\x23\x0b\xeb\xc2\0\x40\x20\0".to_vec(),
            )),
            // Newest first, four bytes each: 7 down to 0.
            [protocol::CMD_RUN] => Some((
                Duration::from_millis(30),
                (0..8u32).rev().flat_map(u32::to_le_bytes).collect(),
            )),
            _ => None,
        }
    }

    fn device(fake: Fake) -> OlsDevice {
        let meta = Metadata {
            name: "OLS".into(),
            channels: 32,
            max_samplebytes: 24576,
            max_samplerate: 200_000_000,
            ..Metadata::default()
        };
        let mut dev = OlsDevice::from_port(Some(meta), Box::new(fake));
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(8), None).unwrap();
        dev
    }

    fn sink() -> (PacketSink, Receiver<(usize, Packet)>) {
        let (tx, rx) = sync_channel(64);
        (PacketSink::new(tx, 0), rx)
    }

    /// The packets up to `End`, with a bound of their own, so that a mutant
    /// fails rather than hangs.
    fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
        let mut got = Vec::new();
        loop {
            let (_, p) = rx.recv_timeout(Duration::from_secs(10)).expect("an End in time");
            let end = matches!(p, Packet::End);
            got.push(p);
            if end {
                return got;
            }
        }
    }

    fn logic(packets: &[Packet]) -> Vec<u8> {
        packets
            .iter()
            .filter_map(|p| if let Packet::Logic(l) = p { Some(l.data.clone()) } else { None })
            .flatten()
            .collect()
    }

    fn written(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
        seen.lock().unwrap().writes.iter().map(|(_, w)| w.clone()).collect()
    }

    #[test]
    fn the_scan_waits_for_each_reply_and_keeps_the_order() {
        let (mut fake, seen) = Fake::new(board);
        let found = probe(&mut fake, params(), "x").unwrap();
        assert_eq!(found.metadata.unwrap().name, "OLS");
        let writes = seen.lock().unwrap();
        let commands: Vec<&[u8]> = writes.writes.iter().map(|(_, w)| w.as_slice()).collect();
        assert_eq!(commands, [&[0][..], &[0], &[0], &[0], &[0], &[CMD_ID], &[CMD_METADATA]]);
        assert_eq!(writes.drains, 7, "each command drained");
        // RESPONSE_DELAY after each of the two questions.
        let id_at = writes.writes[5].0;
        let meta_at = writes.writes[6].0;
        assert!(meta_at - id_at >= RESPONSE_DELAY);
    }

    #[test]
    fn a_reply_that_comes_after_the_delay_is_no_reply() {
        fn late(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            (data == [CMD_ID]).then(|| (Duration::from_secs(2), b"1SLO".to_vec()))
        }
        let (mut fake, _) = Fake::new(late);
        assert!(probe(&mut fake, params(), "x").is_none());
    }

    #[test]
    fn no_metadata_is_a_sump_board_and_a_short_id_is_none() {
        fn sump(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            (data == [CMD_ID]).then(|| (Duration::ZERO, b"1ALS".to_vec()))
        }
        fn short(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            (data == [CMD_ID]).then(|| (Duration::ZERO, b"1S".to_vec()))
        }
        let (mut fake, _) = Fake::new(sump);
        let found = probe(&mut fake, params(), "x").unwrap();
        assert_eq!(found.vendor, "Sump");
        assert!(found.metadata.is_none());
        let (mut fake, _) = Fake::new(short);
        assert!(probe(&mut fake, params(), "x").is_none());
    }

    #[test]
    fn a_port_that_will_not_write_or_drain_is_no_device() {
        let (mut fake, seen) = Fake::new(board);
        fake.fail_writes_after = Some(2);
        assert!(probe(&mut fake, params(), "x").is_none());
        assert_eq!(written(&seen).len(), 2);
        let (mut fake, seen) = Fake::new(board);
        fake.fail_drains = true;
        assert!(probe(&mut fake, params(), "x").is_none());
        assert_eq!(written(&seen).len(), 1, "gives up at the first drain");
    }

    #[test]
    fn a_capture_waits_then_ends_at_the_limit_at_once() {
        let (fake, seen) = Fake::new(board);
        let mut dev = device(fake);
        // Far longer than any stall here, so it can only end at the limit.
        dev.end_of_data = Duration::from_secs(3600);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        // Oldest first.
        let expected: Vec<u8> = (0..8u32).flat_map(u32::to_le_bytes).collect();
        assert_eq!(logic(&packets), expected);
        let writes = written(&seen);
        assert_eq!(&writes[writes.len() - 5..], [[0], [0], [0], [0], [0]]);
        assert_eq!(writes[writes.len() - 6], [protocol::CMD_RUN]);
        assert_eq!(seen.lock().unwrap().purges, 1);
    }

    #[test]
    fn a_short_capture_ends_after_the_silence() {
        fn cut(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            (data == [protocol::CMD_RUN]).then(|| (Duration::ZERO, vec![9, 0, 0, 0, 8, 0, 0, 0]))
        }
        let (fake, _) = Fake::new(cut);
        let mut dev = device(fake);
        let (s, rx) = sink();
        let at = Instant::now();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        assert!(at.elapsed() >= END_OF_DATA, "after the first byte, 100 ms of nothing");
        assert_eq!(logic(&packets), [8, 0, 0, 0, 9, 0, 0, 0]);
        assert!(packets.iter().any(|p| matches!(p, Packet::Failed(_))), "cut short: {packets:?}");
    }

    /// A capture where a word comes, `gap` of nothing, then another: the bytes
    /// it kept, and how long from the first word it took.
    fn capture_with_a_gap(gap: Duration) -> (Vec<u8>, Duration) {
        fn none(_: &[u8]) -> Option<(Duration, Vec<u8>)> {
            None
        }
        let (mut fake, _) = Fake::new(none);
        let at = Instant::now();
        fake.input.extend([1u8, 0, 0, 0].map(|b| (at, b)));
        fake.input.extend([2u8, 0, 0, 0].map(|b| (at + gap, b)));
        let mut dev = device(fake);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        let took = at.elapsed();
        dev.stop().unwrap();
        (logic(&packets), took)
    }

    #[test]
    fn silence_ends_a_capture_at_a_hundred_milliseconds() {
        // Literal, not from END_OF_DATA: a constant of 2000 ms passed the rest.
        let (kept, took) = capture_with_a_gap(Duration::from_millis(80));
        assert_eq!(kept, [2, 0, 0, 0, 1, 0, 0, 0], "80 ms is inside the capture");
        assert!(took >= Duration::from_millis(180), "then 100 ms more, took {took:?}");
        let (kept, took) = capture_with_a_gap(Duration::from_millis(200));
        assert_eq!(kept, [1, 0, 0, 0], "the silence ended it before the rest");
        assert!(took >= Duration::from_millis(100), "too soon: {took:?}");
        assert!(took < Duration::from_millis(200), "too late: {took:?}");
    }

    #[test]
    fn before_the_first_byte_silence_is_no_end() {
        fn never(_: &[u8]) -> Option<(Duration, Vec<u8>)> {
            None
        }
        let (fake, seen) = Fake::new(never);
        let mut dev = device(fake);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        assert!(matches!(rx.recv_timeout(Duration::from_secs(5)), Ok((_, Packet::Header { .. }))));
        assert!(rx.recv_timeout(END_OF_DATA * 3).is_err(), "still waiting");
        dev.stop().unwrap();
        let packets = until_end(&rx);
        assert_eq!(logic(&packets), [0u8; 0]);
        assert_eq!(written(&seen)[written(&seen).len() - 5..], [[0], [0], [0], [0], [0]]);
    }

    #[test]
    fn a_second_capture_follows_the_first_without_a_stop() {
        let (fake, seen) = Fake::new(board);
        let mut dev = device(fake);
        for _ in 0..2 {
            let (s, rx) = sink();
            // At once after End, with no retry.
            dev.start(s).unwrap();
            assert_eq!(logic(&until_end(&rx)).len(), 32);
        }
        dev.stop().unwrap();
        let runs = written(&seen).iter().filter(|w| w.as_slice() == [protocol::CMD_RUN]).count();
        assert_eq!(runs, 2);
    }

    #[test]
    fn a_capture_under_way_is_busy() {
        fn none(_: &[u8]) -> Option<(Duration, Vec<u8>)> {
            None
        }
        let (fake, _) = Fake::new(none);
        let mut dev = device(fake);
        let (s, _rx) = sink();
        dev.start(s).unwrap();
        let (s, _rx2) = sink();
        assert!(matches!(dev.start(s), Err(Error::Busy)));
        dev.stop().unwrap();
    }

    #[test]
    fn a_stop_drops_what_came() {
        fn slow(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            (data == [protocol::CMD_RUN]).then(|| (Duration::ZERO, vec![1, 0, 0, 0]))
        }
        let (fake, seen) = Fake::new(slow);
        let mut dev = device(fake);
        // No silence ends it, however late the stop comes.
        dev.end_of_data = Duration::from_secs(3600);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        // The second read began, so the first took what came.
        let deadline = Instant::now() + Duration::from_secs(60);
        while seen.lock().unwrap().reads_from.len() < 2 {
            assert!(Instant::now() < deadline, "no second read");
            std::thread::sleep(Duration::from_millis(1));
        }
        dev.stop().unwrap();
        assert_eq!(logic(&until_end(&rx)), [0u8; 0]);
    }

    #[test]
    fn a_hangup_ends_with_what_came_and_none_is_no_packet() {
        let (mut fake, _) = Fake::new(board);
        fake.hangup_after = Some(8);
        let mut dev = device(fake);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        // The newest two of eight words, in time order.
        assert_eq!(logic(&packets), [6, 0, 0, 0, 7, 0, 0, 0]);
        let (mut fake, _) = Fake::new(board);
        fake.hangup_after = Some(0);
        let mut dev = device(fake);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        assert!(!packets.iter().any(|p| matches!(p, Packet::Logic(_))), "no empty packet");
    }

    #[test]
    fn a_refused_start_sends_the_stop_s_resets() {
        let (fake, seen) = Fake::new(board);
        let mut dev = device(fake);
        let mut t = Trigger::new();
        for stage in 0..5 {
            t.add(stage, 0, TriggerMatch::One);
        }
        dev.set_trigger(Some(t)).unwrap();
        let (s, _rx) = sink();
        assert!(dev.start(s).is_err());
        assert_eq!(written(&seen), [[0], [0], [0], [0], [0]]);
    }

    #[test]
    fn the_marker_sits_at_the_board_s_word() {
        let (fake, _) = Fake::new(board);
        let mut dev = device(fake);
        dev.set(&Key::CaptureRatio, &Value::U64(50), None).unwrap();
        let mut t = Trigger::new();
        t.add(0, 4, TriggerMatch::One);
        dev.set_trigger(Some(t)).unwrap();
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        // (2 - 1) * 4 - 1 samples ahead of it, as C has it.
        let pos = packets.iter().position(|p| matches!(p, Packet::Trigger)).unwrap();
        let Packet::Logic(first) = &packets[pos - 1] else { panic!("samples before it") };
        assert_eq!(first.data.len(), 3 * 4);
    }

    #[test]
    fn a_limit_that_cuts_the_read_keeps_the_marker_on_its_word() {
        let (fake, _) = Fake::new(board);
        let mut dev = device(fake);
        dev.set(&Key::CaptureRatio, &Value::U64(50), None).unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(5), None).unwrap();
        let mut t = Trigger::new();
        t.add(0, 4, TriggerMatch::One);
        dev.set_trigger(Some(t)).unwrap();
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        // Words 3 to 7 of the eight stay. The marker is on word 3, the oldest
        // kept, not 3 samples further on as `protocol.c:618` has it.
        assert!(matches!(packets[1], Packet::Trigger));
        let expected: Vec<u8> = (3..8u32).flat_map(u32::to_le_bytes).collect();
        assert_eq!(logic(&packets), expected);
    }

    #[test]
    fn under_rle_the_marker_counts_samples_and_stops_at_what_came() {
        fn runs(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            // A count of 9, value 1: ten samples, one group.
            (data == [protocol::CMD_RUN]).then(|| (Duration::ZERO, vec![0x89, 1]))
        }
        let capture = |limit: u64| {
            let (fake, _) = Fake::new(runs);
            let mut dev = device(fake);
            for i in 8..32 {
                dev.enable_channel(i, false).unwrap();
            }
            dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
            dev.set(&Key::CaptureRatio, &Value::U64(50), None).unwrap();
            dev.set(&Key::LimitSamples, &Value::U64(limit), None).unwrap();
            let mut t = Trigger::new();
            t.add(0, 4, TriggerMatch::One);
            dev.set_trigger(Some(t)).unwrap();
            let (s, rx) = sink();
            dev.start(s).unwrap();
            let packets = until_end(&rx);
            dev.stop().unwrap();
            packets
        };
        // Limit 8: units 2, delay 1, marker (2 - 1) * 4 - 1 = 3 from the oldest.
        let packets = capture(8);
        let Packet::Logic(first) = &packets[1] else { panic!("samples first") };
        assert_eq!(first.data.len(), 3 * 4);
        assert!(matches!(packets[2], Packet::Trigger));
        // Limit 32: units 8, delay 4, marker at 15, past the 10 that came.
        let packets = capture(32);
        assert!(!packets.iter().any(|p| matches!(p, Packet::Trigger)));
        assert_eq!(logic(&packets).len(), 10 * 4);
    }

    #[test]
    fn a_narrow_board_crops_its_samples() {
        fn one(data: &[u8]) -> Option<(Duration, Vec<u8>)> {
            (data == [protocol::CMD_RUN]).then(|| (Duration::ZERO, (0..4).rev().collect()))
        }
        let (fake, _) = Fake::new(one);
        let meta = Metadata { channels: 4, max_samplebytes: 64, ..Metadata::default() };
        let mut dev = OlsDevice::from_port(Some(meta), Box::new(fake));
        dev.open().unwrap();
        dev.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
        assert_eq!(dev.unit_size, 1);
        assert_eq!(dev.channels().len(), 4);
        let (s, rx) = sink();
        dev.start(s).unwrap();
        let packets = until_end(&rx);
        dev.stop().unwrap();
        assert_eq!(logic(&packets), [0, 1, 2, 3]);
    }

    #[test]
    fn a_board_with_no_channel_count_has_a_sample_size() {
        let (fake, _) = Fake::new(board);
        let dev = OlsDevice::from_port(None, Box::new(fake));
        assert_eq!(dev.unit_size, 4);
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(200_000));
    }

    #[test]
    fn a_board_slower_than_the_default_starts_no_faster_than_its_maximum() {
        let (fake, _) = Fake::new(board);
        let meta = Metadata { max_samplerate: 150_000, ..Metadata::default() };
        let dev = OlsDevice::from_port(Some(meta), Box::new(fake));
        assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(149_925));
        assert_ne!(dev.flags & FLAG_FILTER, 0);
    }

    #[test]
    fn options_as_the_driver_keeps_them() {
        let (fake, _) = Fake::new(board);
        let mut dev = device(fake);
        let on = Value::Bool(true);
        dev.set(&Key::ExternalClock, &on, None).unwrap();
        dev.set(&Key::ClockEdge, &Value::Str("up".into()), None).unwrap();
        assert_eq!(dev.get(&Key::ClockEdge, None).unwrap(), Value::Str("rising".into()));
        dev.set(&Key::ClockEdge, &Value::Str("falling".into()), None).unwrap();
        assert_eq!(dev.get(&Key::ClockEdge, None).unwrap(), Value::Str("falling".into()));
        assert_eq!(dev.flags & 0xc0, 0xc0);
        dev.set(&Key::PatternMode, &Value::Str("Internal".into()), None).unwrap();
        dev.set(&Key::PatternMode, &Value::Str("None".into()), None).unwrap();
        assert_eq!(dev.flags & (FLAG_INTERNAL_TEST_MODE | FLAG_EXTERNAL_TEST_MODE), 0);
        assert!(dev.set(&Key::PatternMode, &Value::Str("x".into()), None).is_err());
        assert!(dev.set(&Key::LimitSamples, &Value::U64(3), None).is_err());
        assert!(dev.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
        assert!(dev.set(&Key::Samplerate, &Value::U64(9), None).is_err());
        assert!(dev.set(&Key::Samplerate, &Value::U64(201_000_000), None).is_err());
        dev.set(&Key::Samplerate, &Value::U64(200_000_000), None).unwrap();
        assert_ne!(dev.flags & FLAG_DEMUX, 0);
        assert_eq!(dev.flags & FLAG_FILTER, 0);
        assert!(matches!(dev.list(&Key::LimitSamples, None), Ok(ValueList::MinMaxU64(4, 6144))));
    }
}
