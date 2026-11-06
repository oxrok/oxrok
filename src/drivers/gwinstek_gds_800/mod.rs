// Counterpart in sigrok:
//   libsigrok/src/hardware/gwinstek-gds-800/api.c,
//   libsigrok/src/hardware/gwinstek-gds-800/protocol.c,
//   libsigrok/src/hardware/gwinstek-gds-800/protocol.h,
//   Copyright (C) 2015 Martin Lederhilger <martin.lederhilger@gmx.at>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig,
//   Martin Ling, Valentin Ochs, Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! GW Instek GDS-806, 810, 820 and 840 oscilloscopes over SCPI on their serial port.
//!
//! The programming manual has the protocol. A frame is `:TRIG:MOD 3`, `:STOP`,
//! `:RUN`, then for each enabled channel `:ACQ<X>:MEM?` (firmware 2.03 and above)
//! and `:CHAN<X>:SCAL?`. A memory read's answer is
//!
//! ```text
//! #  N  SIZE (N digits)  RATE (4)  CHANNEL (1)  RESERVED (3)  POINTS (SIZE - 8)
//! ```
//!
//! with `N` 4 to 6, `SIZE` of the manual's table (1008 to 250008), `RATE` a float
//! high byte first (libsigrok's comment says so against the manual's words, and the
//! manual's example agrees with the comment), and each point an `i16`, high byte first.
//! A volt is `:CHAN<X>:SCAL?` times 10/256 per count. There is no check on any of it:
//! a damaged point can't be told from a good one, see the test of single bit flips.
//!
//! The scope speaks RS-232 with no handshake lines; `serialcomm=` is what its panel is
//! set to, and with none the port keeps its line.
//!
//! # Against C
//!
//! - Points are binary, so a block is read a byte at a time: `scpi_serial_read_data`
//!   takes a CR off a read ending in LF CR, and the driver's `read_data` adds an LF
//!   after any short read ending in LF. C gets each such byte doubled, then reads the
//!   rest of the block as its scale, and 0 V, or ends with "Too many bytes read". The
//!   lines were right when written: libsigrok's 9ea4018f (2017) stopped stripping
//!   line ends below the driver. Pull request 121 on libsigrok, from a GDS-2000 owner
//!   and unmerged, reports the 0 V on a real scope; any trace at 10 counts has the byte.
//! - C waits for ever for a block. Here 30 s of silence warns and 10 minutes ends the
//!   capture, since a single trigger may wait and the slowest sweep is 100 s, at 10 s
//!   per division. 4096 bytes with no `#` end it at once. See [`Silence`].
//! - `SIZE` must be digits and fit: over 250008 C writes past its buffers.
//! - `:CHAN<X>:SCAL?`'s number must be one, with nothing behind it but `V` or `mV`, and
//!   above 0, where `strtod` makes 0 V of what it can't read, and exit 0.
//! - An empty answer to it is read past: the manual has a line feed behind each
//!   answer, and if a block has one C takes that for its scale, and 0 V.

#![forbid(unsafe_code)]

#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::{HwInfo, Scpi};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Oscilloscope];
static DEV_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::Samplerate, Access::GET),
];

const ANALOG_CHANNELS: usize = 2;
const VERTICAL_DIVISIONS: f32 = 10.0;
/// The rate, the channel and the reserved bytes of a block.
const FIELDS: usize = 8;
/// `MAX_RCV_BUFFER_SIZE` and the largest size in the manual's table: 125000 points.
const LARGEST_POINTS: usize = 250_000;
/// The manual's sizes, one per record length.
const SIZES: [usize; 8] = [1008, 2508, 5008, 10008, 25008, 50008, 100_008, 250_008];
/// How long one read of the line waits, which is also how long a stop waits.
const READ_WAIT: Duration = Duration::from_millis(50);
/// `sr_scpi_get_string`'s wait.
const ANSWER_WAIT: Duration = Duration::from_secs(1);
/// Silence before a word.
const SILENCE_WARN: Duration = Duration::from_secs(30);
/// Silence that ends a capture. The slowest sweep is 100 s, at 10 s per division, and
/// a single trigger may hold the answer until it comes.
const SILENCE_LIMIT: Duration = Duration::from_secs(600);
/// Bytes with no block start that end a capture, without waiting out [`SILENCE_LIMIT`].
const JUNK_LIMIT: usize = 4096;
/// What a stop left on its way: the quiet that says it is over, and the longest wait
/// for that, within which 250 kB don't pass at a slow baud rate.
const LEFTOVER_QUIET: Duration = Duration::from_secs(1);
const LEFTOVER_LIMIT: Duration = Duration::from_secs(30);

/// The driver.
pub struct GwinstekGds800Driver;

/// Whether `*IDN?` says it is a GDS-800, as `probe_device`.
fn is_gds_800(id: &HwInfo) -> bool {
    id.manufacturer == "GW" && id.model.starts_with("GDS-8")
}

impl Driver for GwinstekGds800Driver {
    fn id(&self) -> &'static str {
        "gwinstek-gds-800"
    }

    fn long_name(&self) -> &'static str {
        "GW Instek GDS-800 series"
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
                Key::Serialcomm => serialcomm = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let found = crate::scpi::scan(conn.as_deref(), serialcomm.as_deref(), |mut scpi| {
            let id = match scpi.hw_id() {
                Ok(id) => id,
                Err(e) => {
                    log::info!("Couldn't get IDN response: {e}");
                    let _ = scpi.close();
                    return Ok(None);
                }
            };
            if !is_gds_800(&id) {
                log::info!("{} {} is not a GDS-800.", id.manufacturer, id.model);
                let _ = scpi.close();
                return Ok(None);
            }
            let _ = scpi.close();
            Ok(Some(GdsDevice::new(&id, scpi)))
        })?;
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// How long the capture's waits are: tests shorten them.
#[derive(Clone, Copy, Debug)]
struct Times {
    read: Duration,
    answer: Duration,
    warn: Duration,
    limit: Duration,
    quiet: Duration,
    leftover: Duration,
}

const TIMES: Times = Times {
    read: READ_WAIT,
    answer: ANSWER_WAIT,
    warn: SILENCE_WARN,
    limit: SILENCE_LIMIT,
    quiet: LEFTOVER_QUIET,
    leftover: LEFTOVER_LIMIT,
};

/// What a capture and the device share.
#[derive(Debug, Default)]
struct Shared {
    /// `frame_limit`: a capture reads it at each frame's end, as C does.
    frames: AtomicU64,
    /// The last block's rate, as a float's bits.
    rate: AtomicU32,
    /// A block was asked for and not read to its end, so more of it may come.
    in_block: AtomicBool,
}

/// One scope.
pub struct GdsDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    shared: Arc<Shared>,
    times: Times,
    /// The instrument while the device is closed.
    closed: Option<Scpi>,
    /// The instrument while it is open and no capture holds it.
    scpi: Arc<Mutex<Option<Scpi>>>,
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl GdsDevice {
    /// A device on an instrument, which [`Device::open`] opens again.
    pub fn new(id: &HwInfo, scpi: Scpi) -> GdsDevice {
        let channels = (0..ANALOG_CHANNELS)
            .map(|i| Channel::new(i, ChannelKind::Analog, format!("CH{}", i + 1)))
            .collect();
        let shared = Shared { frames: AtomicU64::new(1), ..Shared::default() };
        GdsDevice {
            info: DeviceInfo {
                driver: "gwinstek-gds-800",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels,
            // C's one group has an empty name, and no options of its own.
            groups: vec![ChannelGroup::new("", vec![0, 1])],
            shared: Arc::new(shared),
            times: TIMES,
            closed: Some(scpi),
            scpi: Arc::new(Mutex::new(None)),
            worker: None,
        }
    }

    fn known_group(&self, group: Option<&str>) -> Result<()> {
        match group {
            Some(g) if !self.groups.iter().any(|cg| cg.name == g) => {
                Err(Error::ChannelGroup(g.to_owned()))
            }
            _ => Ok(()),
        }
    }
}

impl Device for GdsDevice {
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
        self.known_group(group)?;
        Ok(if group.is_none() { DEV_OPTIONS.to_vec() } else { Vec::new() })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        self.known_group(group)?;
        match key {
            Key::LimitFrames => Ok(Value::U64(self.shared.frames.load(Ordering::SeqCst))),
            // The last block's; 0 before one, as in C.
            Key::Samplerate => {
                let rate = f32::from_bits(self.shared.rate.load(Ordering::SeqCst));
                Ok(Value::U64(rate as u64))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        self.known_group(group)?;
        match key {
            Key::LimitFrames => {
                let bad = || Error::Arg(format!("invalid value for {}", key.id()));
                let frames = value.as_u64().ok_or_else(bad)?;
                self.shared.frames.store(frames, Ordering::SeqCst);
                Ok(())
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        self.known_group(group)?;
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn open(&mut self) -> Result<()> {
        let mut slot = self.scpi.lock().expect("scpi lock");
        if slot.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let Some(mut scpi) = self.closed.take() else {
            return Err(Error::device("the instrument is gone"));
        };
        if let Err(e) = scpi.reopen() {
            log::error!("Failed to open SCPI device: {e}.");
            self.closed = Some(scpi);
            return Err(e);
        }
        *slot = Some(scpi);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.stop()?;
        if let Some(mut scpi) = self.scpi.lock().expect("scpi lock").take() {
            let closed = scpi.close();
            self.closed = Some(scpi);
            return closed;
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        // The instrument back in its slot says a capture is over, whatever its thread
        // is doing with an End nobody reads: tell it to go, and join it.
        if self.worker.is_some() && self.scpi.lock().expect("scpi lock").is_none() {
            return Err(Error::Busy);
        }
        self.stop()?;
        let Some(mut scpi) = self.scpi.lock().expect("scpi lock").take() else {
            return Err(Error::device("device is not open"));
        };
        let enabled = [self.channels[0].enabled, self.channels[1].enabled];
        if !enabled.iter().any(|on| *on) {
            *self.scpi.lock().expect("scpi lock") = Some(scpi);
            return Err(Error::arg("No channels enabled."));
        }
        if firmware_lacks_memory_read(&self.info.version) {
            log::warn!(
                "Firmware {} is older than 2.03, and the manual has :ACQ<X>:MEM? from then on.",
                self.info.version
            );
        }
        let stop = Arc::new(AtomicBool::new(false));
        let shared = Arc::clone(&self.shared);
        let slot = Arc::clone(&self.scpi);
        let times = self.times;
        let stop2 = Arc::clone(&stop);
        let handle = std::thread::Builder::new()
            .name("gwinstek-gds-800 acquisition".to_owned())
            .spawn(move || {
                let outcome = Capture::new(&mut scpi, &sink, &stop2, &shared, enabled, times).run();
                // Back ahead of the End, so a client that saw the End can start again.
                *slot.lock().expect("scpi lock") = Some(scpi);
                sink.end_while(outcome, || !stop2.load(Ordering::SeqCst));
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

/// `*IDN?`'s version says before 2.03. One in another form says nothing, and passes.
fn firmware_lacks_memory_read(version: &str) -> bool {
    let digits = version.trim_start_matches(['V', 'v', '.']);
    let Some((major, minor)) = digits.split_once('.') else { return false };
    match (major.parse::<u32>(), minor.parse::<u32>()) {
        (Ok(major), Ok(minor)) => (major, minor) < (2, 3),
        _ => false,
    }
}

/// `:CHAN<X>:SCAL?`'s answer, in volts per division, as C computes it: a float, and
/// `mV` a product in double precision. `None` for what is no scale: C's `strtod` takes
/// whatever comes and makes 0 of what it can't read.
fn parse_scale(text: &str) -> Option<f32> {
    let text = text.trim();
    let end = number_end(text)?;
    let value = text[..end].parse::<f64>().ok()? as f32;
    let scale = match text[end..].trim() {
        "" | "V" | "v" => value,
        "mV" => (f64::from(value) * 1e-3) as f32,
        _ => return None,
    };
    (scale.is_finite() && scale > 0.0).then_some(scale)
}

/// Where a plain decimal number with an exponent ends in `text`, if it starts with
/// one. No hex, `inf` or `nan`, all of which `strtod` takes.
fn number_end(text: &str) -> Option<usize> {
    let bytes = text.as_bytes();
    let digits = |from: usize| bytes[from..].iter().take_while(|b| b.is_ascii_digit()).count();
    let mut at = usize::from(matches!(bytes.first(), Some(b'+' | b'-')));
    let whole = digits(at);
    at += whole;
    let mut fraction = 0;
    if bytes.get(at) == Some(&b'.') {
        fraction = digits(at + 1);
        at += 1 + fraction;
    }
    if whole + fraction == 0 {
        return None;
    }
    if matches!(bytes.get(at), Some(b'e' | b'E')) {
        let sign = usize::from(matches!(bytes.get(at + 1), Some(b'+' | b'-')));
        let exponent = digits(at + 1 + sign);
        if exponent > 0 {
            at += 1 + sign + exponent;
        }
    }
    Some(at)
}

/// Volts per count, and libsigrok's digits for it.
fn volts_per_count(scale: f32) -> (f32, i32) {
    let vbit = (f64::from(scale * VERTICAL_DIVISIONS) / 256.0) as f32;
    let log = vbit.log10();
    (vbit, -(log as i32) + i32::from(log < 0.0))
}

/// The points of a block, in volts. A last odd byte is no point.
fn to_volts(points: &[u8], vbit: f32) -> Vec<f32> {
    let (points, _) = points.as_chunks::<2>();
    points.iter().map(|p| f32::from(i16::from_be_bytes(*p)) * vbit).collect()
}

/// The size field: digits, where `sr_atoi` takes a sign and spaces too, and within what
/// a block holds. C writes a block over 250008 past its buffers, and one under 8 asks
/// for a read of a negative size.
fn parse_size(field: &[u8]) -> std::result::Result<usize, String> {
    let bad = || format!("Could not parse data size '{}'", String::from_utf8_lossy(field));
    if !field.iter().all(u8::is_ascii_digit) {
        return Err(bad());
    }
    let size: usize =
        std::str::from_utf8(field).ok().and_then(|s| s.parse().ok()).ok_or_else(bad)?;
    if !(FIELDS..=FIELDS + LARGEST_POINTS).contains(&size) {
        return Err(format!(
            "Data size {size} is out of range: {FIELDS} to {}.",
            FIELDS + LARGEST_POINTS
        ));
    }
    Ok(size)
}

/// What a wait for the scope does next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Verdict {
    Wait,
    /// Say so, once a silence.
    Warn,
    GiveUp,
}

/// How long the scope has said nothing that counts. Bytes ahead of a block's `#` don't
/// count, so a trickle of them can't hold a capture past the limit, and a lot of them
/// end it sooner.
#[derive(Clone, Copy, Debug)]
struct Silence {
    since: Instant,
    warned: bool,
    junk: usize,
    warn_after: Duration,
    limit: Duration,
}

impl Silence {
    fn new(now: Instant, warn_after: Duration, limit: Duration) -> Silence {
        Silence { since: now, warned: false, junk: 0, warn_after, limit }
    }

    /// Something that counts came. True if a warning stands, to say it is over.
    fn heard(&mut self, now: Instant) -> bool {
        self.since = now;
        self.junk = 0;
        std::mem::take(&mut self.warned)
    }

    /// A read found nothing, and `asked` is when it began: a stall ahead of the read
    /// would have left the bytes there to find, so that much was silence.
    fn after_empty_read(&mut self, asked: Instant) -> Verdict {
        let silent = asked.saturating_duration_since(self.since);
        if silent >= self.limit {
            Verdict::GiveUp
        } else if silent >= self.warn_after && !self.warned {
            self.warned = true;
            Verdict::Warn
        } else {
            Verdict::Wait
        }
    }

    /// Bytes that make nothing, and whether they are enough to end it. They count for
    /// nothing against the silence, whose limit stands behind them.
    fn after_junk(&mut self, asked: Instant) -> Verdict {
        self.junk += 1;
        if self.junk >= JUNK_LIMIT || asked.saturating_duration_since(self.since) >= self.limit {
            Verdict::GiveUp
        } else {
            Verdict::Wait
        }
    }

    fn warning(&self) -> String {
        format!(
            "No answer from the scope for {} s, still waiting: a single trigger may not have come.",
            self.warn_after.as_secs_f32()
        )
    }

    fn give_up(&self) -> String {
        if self.junk >= JUNK_LIMIT {
            format!("No usable answer: {} bytes and no block.", self.junk)
        } else {
            format!("No answer from the scope in {} s.", self.limit.as_secs_f32())
        }
    }
}

/// The capture thread's own.
struct Capture<'a> {
    scpi: &'a mut Scpi,
    sink: &'a PacketSink,
    stop: &'a AtomicBool,
    shared: &'a Shared,
    enabled: [bool; ANALOG_CHANNELS],
    times: Times,
    silence: Silence,
    /// `df_started`: the header and a frame have gone out.
    started: bool,
    size_warned: bool,
}

/// Why a capture leaves a step early with no error: a stop, or nobody listening.
struct Stopped;

/// A capture's step: `Err(Err)` is a failure.
type Step<T> = std::result::Result<T, std::result::Result<Stopped, Error>>;

impl<'a> Capture<'a> {
    fn new(
        scpi: &'a mut Scpi,
        sink: &'a PacketSink,
        stop: &'a AtomicBool,
        shared: &'a Shared,
        enabled: [bool; ANALOG_CHANNELS],
        times: Times,
    ) -> Capture<'a> {
        let silence = Silence::new(Instant::now(), times.warn, times.limit);
        let size_warned = false;
        Capture { scpi, sink, stop, shared, enabled, times, silence, started: false, size_warned }
    }

    fn run(&mut self) -> Result<()> {
        self.leftover();
        let outcome = match self.frames() {
            Ok(()) | Err(Ok(Stopped)) => Ok(()),
            Err(Err(e)) => Err(e),
        };
        // `dev_acquisition_stop`'s frame end. The End is the caller's.
        if self.started {
            self.send(Packet::FrameEnd);
        }
        outcome
    }

    fn going_on(&self) -> bool {
        !self.stop.load(Ordering::SeqCst)
    }

    /// A packet, unless a stop gets in first. False when it can't go.
    fn send(&self, packet: Packet) -> bool {
        self.sink.send_while(packet, || self.going_on())
    }

    fn deliver(&mut self, packet: Packet) -> Step<()> {
        if !self.send(packet) {
            return Err(Ok(Stopped));
        }
        // A client that kept us waiting is no silence of the scope's.
        self.silence.heard(Instant::now());
        Ok(())
    }

    /// A stop or failure left a block on its way: it ends before anything is asked.
    fn leftover(&mut self) {
        if !self.shared.in_block.swap(false, Ordering::SeqCst) {
            return;
        }
        let began = Instant::now();
        while self.going_on() {
            match self.scpi.discard(self.times.quiet) {
                Ok(Some(0) | None) | Err(_) => return,
                Ok(Some(_)) => {
                    if began.elapsed() >= self.times.leftover {
                        log::warn!("The scope is still sending the block a stop left behind.");
                        return;
                    }
                }
            }
        }
    }

    fn frames(&mut self) -> Step<()> {
        let mut frame = 0u64;
        loop {
            self.arm()?;
            for channel in 0..ANALOG_CHANNELS {
                if self.enabled[channel] {
                    self.channel(channel)?;
                }
                if !self.going_on() {
                    return Err(Ok(Stopped));
                }
            }
            // 0 wraps and never comes, as in C: a capture until stopped.
            let limit = self.shared.frames.load(Ordering::SeqCst);
            if frame == limit.wrapping_sub(1) {
                return Ok(());
            }
            if self.started {
                self.deliver(Packet::FrameEnd)?;
                self.deliver(Packet::FrameBegin)?;
            }
            frame = frame.wrapping_add(1);
        }
    }

    /// Sends a command of the capture's, with C's words if it fails.
    fn command(&mut self, command: &str, words: &str) -> Step<()> {
        self.scpi.read_timeout = self.times.read;
        match self.scpi.send(command) {
            Ok(()) => {
                // After the send, or a slow port counts as a silent scope.
                self.silence.heard(Instant::now());
                Ok(())
            }
            Err(e) => {
                log::error!("{words}");
                Err(Err(e))
            }
        }
    }

    fn arm(&mut self) -> Step<()> {
        self.command(":TRIG:MOD 3", "Failed to set trigger mode to SINGLE.")?;
        self.command(":STOP", "Failed to put the trigger system into STOP state.")?;
        self.command(":RUN", "Failed to put the trigger system into RUN state.")
    }

    /// One channel's block and its scale, and the packet.
    fn channel(&mut self, channel: usize) -> Step<()> {
        self.shared.in_block.store(true, Ordering::SeqCst);
        self.command(&format!(":ACQ{}:MEM?", channel + 1), "Failed to acquire memory.")?;
        if let Err(e) = self.scpi.read_begin() {
            log::error!("Could not begin reading SCPI response.");
            return Err(Err(e));
        }
        let points = self.block(channel)?;
        self.shared.in_block.store(false, Ordering::SeqCst);
        let scale = self.scale(channel)?;
        let (vbit, digits) = volts_per_count(scale);
        log::trace!("Received {} samples from channel {}.", points.len() / 2, channel + 1);
        self.deliver(Packet::Analog(AnalogData {
            channels: vec![channel],
            samples: to_volts(&points, vbit),
            mq: Mq::Voltage,
            flags: MqFlags::empty(),
            unit: Unit::Volt,
            digits,
        }))
    }

    /// The answer to a memory read, up to the points, with the header on the way.
    fn block(&mut self, channel: usize) -> Step<Vec<u8>> {
        self.find_start()?;
        let digit = self.read_byte()?;
        if !matches!(digit, b'4' | b'5' | b'6') {
            return Err(Err(Self::refuse(format!(
                "Data size digits is not 4, 5 or 6 but '{}'.",
                char::from(digit)
            ))));
        }
        let field = self.read_bytes(usize::from(digit - b'0'))?;
        let size = parse_size(&field).map_err(|words| Err(Self::refuse(words)))?;
        if !SIZES.contains(&size) && !self.size_warned {
            self.size_warned = true;
            log::warn!("A block of {size} bytes is no record length in the manual.");
        }
        let rate = self.read_bytes(4)?;
        let rate = u32::from_be_bytes([rate[0], rate[1], rate[2], rate[3]]);
        self.shared.rate.store(rate, Ordering::SeqCst);
        if !self.started {
            self.deliver(Packet::header())?;
            self.deliver(Packet::FrameBegin)?;
            self.started = true;
        }
        let indicator = self.read_byte()?;
        if usize::from(indicator) != channel + 1 {
            log::warn!(
                "The block read for channel {} says it is of channel {indicator}.",
                channel + 1
            );
        }
        self.read_bytes(3)?;
        self.read_bytes(size - FIELDS)
    }

    fn refuse(words: String) -> Error {
        log::error!("{words}");
        Error::device(words)
    }

    /// Up to a block's `#`: C passes over all else.
    fn find_start(&mut self) -> Step<()> {
        loop {
            let asked = Instant::now();
            let Some(byte) = self.next_byte(asked)? else { continue };
            if byte == b'#' {
                self.note_heard();
                return Ok(());
            }
            if self.silence.after_junk(asked) == Verdict::GiveUp {
                return Err(Err(Self::refuse(self.silence.give_up())));
            }
        }
    }

    fn note_heard(&mut self) {
        if self.silence.heard(Instant::now()) {
            log::warn!("The scope is answering again.");
        }
    }

    fn read_byte(&mut self) -> Step<u8> {
        loop {
            let asked = Instant::now();
            if let Some(byte) = self.next_byte(asked)? {
                self.note_heard();
                return Ok(byte);
            }
        }
    }

    fn read_bytes(&mut self, n: usize) -> Step<Vec<u8>> {
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            out.push(self.read_byte()?);
        }
        Ok(out)
    }

    /// One byte off the line, or none yet. One at a time, as these are binary and the
    /// serial transport looks at how a read ends: `n >= 2` and LF CR takes a byte off.
    /// A byte at a time also leaves unread what a stop leaves, and looks at the stop
    /// between bytes.
    fn next_byte(&mut self, asked: Instant) -> Step<Option<u8>> {
        if !self.going_on() {
            return Err(Ok(Stopped));
        }
        let mut byte = [0u8];
        match self.scpi.read_data(&mut byte) {
            Ok(1) => Ok(Some(byte[0])),
            Ok(_) | Err(Error::Timeout) => match self.silence.after_empty_read(asked) {
                Verdict::Wait => Ok(None),
                Verdict::Warn => {
                    log::warn!("{}", self.silence.warning());
                    Ok(None)
                }
                Verdict::GiveUp => Err(Err(Self::refuse(self.silence.give_up()))),
            },
            Err(e) => {
                log::error!("Read data error.");
                Err(Err(e))
            }
        }
    }

    /// `:CHAN<X>:SCAL?`. A read that stopped on the way to a block has left the line
    /// marked, and the send drains it for a read's short wait, so this asks with that
    /// wait and reads with the answer's.
    fn scale(&mut self, channel: usize) -> Step<f32> {
        const WORDS: &str = "Failed to get volts per division.";
        let query = format!(":CHAN{}:SCAL?", channel + 1);
        self.scpi.read_timeout = self.times.read;
        if let Err(e) = self.scpi.send(&query) {
            log::error!("{WORDS}");
            return Err(Err(e));
        }
        self.scpi.read_timeout = self.times.answer;
        // Once more past an empty line: what ended the block, as the manual has it.
        let mut text = String::new();
        for _ in 0..2 {
            let answer = self.scpi.get_data(None);
            self.silence.heard(Instant::now());
            match answer {
                Ok(data) => String::from_utf8_lossy(&data).trim().clone_into(&mut text),
                Err(Error::Timeout) => {
                    log::error!("{WORDS}");
                    return Err(Err(Error::device(format!("No answer to {query}."))));
                }
                Err(e) => {
                    log::error!("{WORDS}");
                    return Err(Err(e));
                }
            }
            if !text.is_empty() {
                break;
            }
        }
        parse_scale(&text).ok_or_else(|| {
            log::error!("{WORDS}");
            Err(Error::device(format!("{query} said '{text}', which is no scale.")))
        })
    }
}
