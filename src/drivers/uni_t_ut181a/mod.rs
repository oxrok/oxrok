// Counterpart in sigrok:
//   libsigrok/src/hardware/uni-t-ut181a/api.c,
//   libsigrok/src/hardware/uni-t-ut181a/protocol.c,
//   Copyright (C) 2019-2020 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The UNI-T UT181A multimeter, on a serial cable at 9600 baud.
//!
//! No id can be asked of the meter, so a scan needs `conn=` and takes what
//! answers as one. It runs monitor mode until a measurement says what the meter
//! is set to, then asks how many recordings there are and each one's name, and
//! closes the port; an open opens it again.
//!
//! `--get` answers from the head of the last measurement (there is no other way
//! to ask) and what was set since; `data_source`'s list asks the recordings
//! again. A capture has three sources: live, where monitor mode streams
//! measurements; `Save`, the saved ones; and a recording, in chunks. In the last
//! two a request goes out when the last one's answer has been read.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A range or a mode set waits for its reply code, and `ER` or nothing is an
//!   error. C has no wait at the range (`protocol.c:826`) and crashes at the mode
//!   (`api.c:412`). `range=auto` sends a range of 0: C sends a mode of 0
//!   (`protocol.c:684`).
//! - The waits are 500 ms where C's are 100 or 200, and 2 s for the first
//!   reading, as C meant: `api.c:140-142` returns at the first 100 ms.
//! - The next chunk of a recording asks for that recording: C asks for the first
//!   (`protocol.c:2682`, whose field nothing sets).
//! - An `ER` during a download ends it, an empty chunk ends it, and so does
//!   [`DOWNLOAD_SILENCE`] with nothing read: C waits for ever, or asks for ever.
//!   Such a capture exits 0 with its message on stderr, like every failed capture
//!   for now. An `ER` to a count or an info ends that wait at once; C waits it out.
//! - A time limit is checked on every pass of the loop, not only at a packet
//!   (`protocol.c:2643`), so a silent meter ends. A live capture with only a
//!   sample or frame limit ends after [`LIVE_SILENCE`] with nothing read.
//! - `consume_str`'s write past its buffer is not copied (`protocol.c:1720`).
//!
//! What it keeps of C's quirks, as it shows in the output: only the first value
//! of a frame counts as a sample, and the limit drops the rest of that frame; a
//! frame that fails halfway stays open and the next one closes it; and in a live
//! capture one step of a search for the next frame is made per read that brought
//! bytes, so what comes behind garbage waits for the next bytes.
//!
//! A reply code says nothing of what it answers, so after a wait that expired or
//! a capture that left something on its way nothing goes out until that answer
//! has come, or a reply wait has passed, and the line is quiet ([`Link::settle`]).
//! A download sends no request ahead of an answer, so a late answer cannot shift
//! the ones behind it inside a capture. A capture takes only its own kind of
//! packet: live measurements, or saved ones, or chunks, where C feeds any of them
//! and a stray one can end its capture. A record info must carry the name the
//! list had, or the start is refused: its unit gives every sample's scale.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    Mq, OptionDesc, Packet, PacketSink, Result, Unit, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{Head, Item, Measurement, RangeChoice, RecInfo, Receiver};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 2] = [DeviceClass::Multimeter, DeviceClass::Thermometer];

static OPTIONS: [OptionDesc; 9] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitFrames, Access::GET_SET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::DataSource, Access::GET_SET_LIST),
    OptionDesc::new(Key::Datalog, Access::GET_SET),
    OptionDesc::new(Key::MeasuredQuantity, Access::GET_SET_LIST),
    OptionDesc::new(Key::Range, Access::GET_SET_LIST),
];

/// `SEND_TO_MS`.
const SEND_TIMEOUT: Duration = Duration::from_millis(100);
/// How long a reply may take. Over the C's 100 or 200 ms, as a busy machine can
/// hold a read for that long; a record info takes 56 ms on the line alone.
const REPLY_WAIT: Duration = Duration::from_millis(500);
/// `ut181a_query_initial_state`'s time for a first reading.
const FIRST_READING: Duration = Duration::from_secs(2);
/// How long a wait for bytes takes at most in a capture's pass.
const PASS_WAIT: Duration = Duration::from_millis(10);
/// The pause after an empty read, with the lock free: a mutex is not fair.
const EMPTY_PAUSE: Duration = Duration::from_millis(1);
/// How long the line must be silent before a request follows an answer that may
/// still be on its way: much more than a pause inside a frame.
const QUIET: Duration = Duration::from_millis(100);
/// The most a line that never falls silent can hold the next request, past the
/// wait for what is owed.
const SETTLE_MAX: Duration = Duration::from_secs(3);
/// How long a download goes with nothing read. The longest chunk takes 2.4 s on
/// the line, and its first byte may not come at once.
pub const DOWNLOAD_SILENCE: Duration = Duration::from_secs(5);
/// How long a live capture that only a sample or frame limit can end goes with
/// nothing read. The meter's rate is not known, and a range change may pause it.
pub const LIVE_SILENCE: Duration = Duration::from_secs(10);
/// What the meter's refusal reads as.
const REFUSED: &str = "the meter refused the request";
/// How long a stopped capture tries to hand over a packet on a full queue.
const AFTER_STOP: Duration = Duration::from_millis(500);

/// The names ahead of the recordings in the list of data sources.
const SOURCE_LIVE: usize = 0;
const SOURCE_SAVE: usize = 1;
const SOURCE_RECORDINGS: usize = 2;

/// The `uni-t-ut181a` driver.
pub struct UniTUt181aDriver;

impl Driver for UniTUt181aDriver {
    fn id(&self) -> &'static str {
        "uni-t-ut181a"
    }

    fn long_name(&self) -> &'static str {
        "UNI-T UT181A"
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
        // No default: nothing tells a meter from what else a port may hold.
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let opened = Params::parse(&serialcomm)
            .and_then(|params| Ok((crate::serial::open(&conn, params)?, params)));
        let (port, params) = match opened {
            Ok(opened) => opened,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let mut link = Link::new(Some(port), params);
        match link.probe() {
            Ok(()) => {
                link.port = None;
                Ok(vec![Box::new(UniTUt181aDevice::new(link, conn))])
            }
            Err(e) => {
                log::debug!("Scan failed: {e}.");
                Ok(Vec::new())
            }
        }
    }
}

/// What a wait is for, `ut181a_configure_waitfor`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Want {
    code: bool,
    /// The code of a command whose reply data, or for a record info its sample
    /// count, is wanted.
    data: Option<u8>,
    measurement: bool,
    /// What a download's request gets: a saved measurement, a chunk or a code.
    answer: bool,
}

/// What a wait saw, the rest of `wait_state`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Got {
    code: Option<bool>,
    data: Option<u32>,
    measurement: bool,
    info: Option<RecInfo>,
    /// An `ER` where data was wanted.
    refused: bool,
    answered: bool,
}

impl Got {
    fn meets(&self, want: Want) -> bool {
        (want.code && self.code.is_some())
            || (want.data.is_some() && self.data.is_some())
            || (want.measurement && self.measurement)
            || (want.answer && self.answered)
            || self.refused
    }
}

/// The port and what the device has said: most of `dev_context`.
struct Link {
    port: Option<Box<dyn Port>>,
    params: Params,
    rx: Receiver,
    head: Head,
    /// `record_names`: an entry stays until an answer replaces it.
    names: Vec<String>,
    record_count: usize,
    /// `rec_info.rec_idx`: whose name a record info is.
    info_index: usize,
    /// Something may still be on its way.
    unsettled: bool,
    /// What the last request is still owed, when that is known.
    owed: Option<Want>,
    /// The waits, for tests.
    reply_wait: Duration,
    first_reading: Duration,
    settle_max: Duration,
}

impl Link {
    fn new(port: Option<Box<dyn Port>>, params: Params) -> Link {
        Link {
            port,
            params,
            rx: Receiver::new(),
            head: Head::default(),
            names: vec![String::new(); protocol::MAX_RECORDINGS],
            record_count: 0,
            info_index: 0,
            unsettled: false,
            owed: None,
            reply_wait: REPLY_WAIT,
            first_reading: FIRST_READING,
            settle_max: SETTLE_MAX,
        }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `ut181a_send_frame`: time running out on a full queue is no error here,
    /// as `serial_write_blocking` returns what went.
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.port()?.write_timeout(frame, SEND_TIMEOUT)?;
        Ok(())
    }

    /// One frame for a wait, in the way `process_packet` takes it outside a
    /// capture. Says whether it was good.
    fn absorb(
        head: &mut Head,
        names: &mut [String],
        index: usize,
        frame: &[u8],
        want: Want,
        got: &mut Got,
    ) -> bool {
        match protocol::parse_frame(frame, head) {
            protocol::Packet::ReplyCode(ok) => {
                if want.code {
                    got.code = Some(ok);
                } else if !ok && want.data.is_some() {
                    got.refused = true;
                }
                got.answered = true;
                true
            }
            protocol::Packet::Measurement(m) => {
                got.measurement |= m.began;
                m.whole
            }
            protocol::Packet::Saved(m) => {
                got.measurement |= m.began;
                got.answered = true;
                m.whole
            }
            protocol::Packet::RecInfo(name, info) => {
                if let (Some(name), Some(slot)) = (name, names.get_mut(index)) {
                    *slot = String::from_utf8_lossy(&name).into_owned();
                }
                let Some(info) = info else {
                    return false;
                };
                if want.data == Some(protocol::CMD_GET_REC_INFO) {
                    got.data = Some(info.samples);
                }
                got.info = Some(info);
                true
            }
            protocol::Packet::ReplyData(code, value) => {
                if want.data == Some(code) {
                    got.data = Some(u32::from(value));
                }
                true
            }
            protocol::Packet::Chunk(_) => {
                got.answered = true;
                true
            }
            protocol::Packet::Unknown(_) => true,
            protocol::Packet::Bad => false,
        }
    }

    /// `ut181a_waitfor_response`. A wait that expires leaves its answer free to
    /// come later, so the next request settles first.
    fn wait(&mut self, want: Want, timeout: Duration) -> Result<Got> {
        let deadline = Instant::now() + timeout;
        let mut got = Got::default();
        let mut buf = [0u8; protocol::RECV_BUFF];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let room = self.rx.room().min(buf.len());
            // At least a millisecond: 0 is no wait, and on Windows no documented one.
            let n = self.port()?.read(&mut buf[..room], left.clamp(EMPTY_PAUSE, PASS_WAIT))?;
            self.rx.push(&buf[..n]);
            let Link { rx, head, names, info_index, .. } = self;
            let index = *info_index;
            rx.frames(false, |frame| Self::absorb(head, names, index, frame, want, &mut got));
            if got.meets(want) {
                return Ok(got);
            }
            if left.is_zero() {
                self.unsettled = true;
                self.owed = Some(want);
                return Err(Error::Timeout);
            }
            if n == 0 {
                std::thread::sleep(EMPTY_PAUSE.min(left));
            }
        }
    }

    /// A wait for data: an `ER` is the end of it, with no more to wait for.
    fn wait_data(&mut self, code: u8) -> Result<Got> {
        let patience = self.reply_wait;
        let got = self.wait(Want { data: Some(code), ..Want::default() }, patience)?;
        if got.refused {
            return Err(Error::device(REFUSED));
        }
        Ok(got)
    }

    /// Before a request, when something may still be on its way: gives an answer
    /// that is owed a reply wait to come, for it would answer this request else,
    /// then reads and drops until the line has been silent for [`QUIET`], then
    /// drops what is left. A purge alone drops only what has arrived.
    fn settle(&mut self) -> Result<()> {
        if !self.unsettled {
            return Ok(());
        }
        if let Some(owed) = self.owed.take() {
            let patience = self.reply_wait;
            let _ = self.wait(owed, patience);
            self.owed = None;
        }
        let began = Instant::now();
        let mut last = began;
        let mut buf = [0u8; 512];
        while last.elapsed() < QUIET && began.elapsed() < self.settle_max {
            if self.port()?.read(&mut buf, PASS_WAIT)? > 0 {
                last = Instant::now();
            } else {
                std::thread::sleep(EMPTY_PAUSE);
            }
        }
        self.unsettled = false;
        self.rx.clear();
        Ok(())
    }

    /// `ut181a_query_initial_state`, then `ut181a_update_recordings`. C frees its
    /// device on either failing, so a meter that does not answer is no device.
    fn probe(&mut self) -> Result<()> {
        self.head.mode = 0;
        self.send(&protocol::monitor(true))?;
        let patience = self.first_reading;
        self.wait(Want { measurement: true, ..Want::default() }, patience)?;
        let _ = self.send(&protocol::monitor(false));
        let patience = self.reply_wait;
        let _ = self.wait(Want { code: true, ..Want::default() }, patience);
        self.update_recordings()
    }

    /// `ut181a_update_recordings`.
    fn update_recordings(&mut self) -> Result<()> {
        self.send(&protocol::get_recs_count())?;
        let got = self.wait_data(protocol::CMD_GET_RECS_COUNT)?;
        let count = (got.data.unwrap_or(0) as usize).min(protocol::MAX_RECORDINGS);
        for index in 0..count {
            self.info_index = index;
            self.send(&protocol::get_rec_info(index))?;
            self.wait_data(protocol::CMD_GET_REC_INFO)?;
        }
        self.record_count = count;
        Ok(())
    }

    /// Reads away what has come, so it answers no request. No purge: C has no flush
    /// at a setting, and on a HID cable a flush is a report on the wire.
    fn drop_arrived(&mut self) -> Result<()> {
        let began = Instant::now();
        let mut buf = [0u8; 512];
        while began.elapsed() < self.settle_max && self.port()?.read(&mut buf, EMPTY_PAUSE)? > 0 {}
        self.rx.clear();
        Ok(())
    }

    /// A setting and its reply code: the meter answers each one, so `ER` or
    /// nothing is a failure.
    fn command(&mut self, frame: &[u8]) -> Result<()> {
        self.settle()?;
        self.drop_arrived()?;
        self.send(frame)?;
        let patience = self.reply_wait;
        let got = self.wait(Want { code: true, ..Want::default() }, patience)?;
        if got.code == Some(true) { Ok(()) } else { Err(Error::device(REFUSED)) }
    }

    /// An entry of the list of data sources. A recording the list has lost keeps
    /// its name, as `record_names` does, and a start refuses it.
    fn source_name(&self, source: usize) -> &str {
        match source {
            SOURCE_LIVE => "Live",
            SOURCE_SAVE => "Save",
            _ => self.names.get(source - SOURCE_RECORDINGS).map_or("", String::as_str),
        }
    }

    fn source_names(&self) -> Vec<String> {
        let mut names = vec!["Live".to_owned(), "Save".to_owned()];
        names.extend_from_slice(&self.names[..self.record_count]);
        names
    }
}

/// `strtol` in base 10 as `sr_atoi` takes it: leading and trailing space, a sign,
/// and an `int`. An empty text is 0.
fn c_atoi(text: &str) -> Option<i32> {
    let trimmed = text.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let (negative, digits) = match trimmed.as_bytes().first() {
        Some(b'-') => (true, &trimmed[1..]),
        Some(b'+') => (false, &trimmed[1..]),
        _ => (false, trimmed),
    };
    let end = digits.bytes().position(|c| !c.is_ascii_digit()).unwrap_or(digits.len());
    let (number, rest) = digits.split_at(end);
    if !rest.trim_end_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']).is_empty() {
        return None;
    }
    if number.is_empty() {
        return if rest.is_empty() && trimmed.is_empty() { Some(0) } else { None };
    }
    let value: i64 = number.parse().ok()?;
    i32::try_from(if negative { -value } else { value }).ok()
}

/// One meter.
pub struct UniTUt181aDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    conn: String,
    limit_samples: u64,
    limit_frames: u64,
    limit_msec: u64,
    /// Which entry of the list of data sources.
    source: usize,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Arc<Mutex<Link>>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    silence: Silence,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// The bounds on nothing read.
#[derive(Clone, Copy, Debug)]
struct Silence {
    live: Duration,
    download: Duration,
}

impl UniTUt181aDevice {
    fn new(link: Link, conn: String) -> Self {
        UniTUt181aDevice {
            info: DeviceInfo {
                driver: "uni-t-ut181a",
                vendor: "UNI-T".to_owned(),
                model: "UT181A".to_owned(),
                version: String::new(),
                serial: String::new(),
                conn: Some(conn.clone()),
            },
            channels: protocol::CHANNELS
                .iter()
                .enumerate()
                .map(|(i, name)| Channel::new(i, ChannelKind::Analog, (*name).to_owned()))
                .collect(),
            conn,
            limit_samples: 0,
            limit_frames: 0,
            limit_msec: 0,
            source: SOURCE_LIVE,
            pending: None,
            link: Arc::new(Mutex::new(link)),
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            silence: Silence { live: LIVE_SILENCE, download: DOWNLOAD_SILENCE },
        }
    }

    /// A device on a port that stands in for the hardware: the scan's probe runs
    /// on it. [`Device::open`] takes the port back.
    ///
    /// # Errors
    ///
    /// What a scan would take for no device.
    pub fn from_port(port: Box<dyn Port>) -> Result<Self> {
        let params = Params::parse(protocol::SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        let mut link = Link::new(Some(port), params);
        link.probe()?;
        let port = link.port.take();
        let mut device = Self::new(link, conn);
        device.pending = port;
        Ok(device)
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn busy(&self) -> bool {
        self.capturing.load(Ordering::SeqCst)
    }

    fn set_data_source(&mut self, name: &str) -> Result<()> {
        let refused = || Error::Arg(format!("invalid value for {}", Key::DataSource.id()));
        let link = self.link();
        let names = link.source_names();
        let count = link.record_count;
        drop(link);
        // A name first. The meter keeps offering a name used before, so the same
        // one can come twice, and a number picks the later one.
        if let Some(found) = names.iter().position(|n| n == name) {
            self.source = found;
            return Ok(());
        }
        let number = c_atoi(name).ok_or_else(refused)?;
        let number = usize::try_from(number).ok().filter(|n| *n > 0).ok_or_else(refused)?;
        if number > count {
            return Err(refused());
        }
        self.source = SOURCE_RECORDINGS + number - 1;
        Ok(())
    }

    fn set_measured_quantity(&mut self, value: &Value) -> Result<()> {
        let Value::Mq(mq, flags) = value else {
            return Err(Error::arg("measured_quantity"));
        };
        let Some(mode) = protocol::mode_of_mq(*mq, *flags) else {
            return Err(Error::not_supported("measured_quantity"));
        };
        if self.busy() {
            return Err(Error::Busy);
        }
        let mut link = self.link();
        link.command(&protocol::set_mode(mode))?;
        link.head.mode = mode;
        // The range byte was the last mode's: it says nothing here until a reading
        // does, and auto is all that a change of mode is known to leave.
        link.head.range = 0;
        link.head.auto_range = true;
        Ok(())
    }

    fn set_range(&mut self, value: &Value) -> Result<()> {
        let text = value.as_str().ok_or_else(|| Error::arg("range"))?;
        if self.busy() {
            return Err(Error::Busy);
        }
        let mut link = self.link();
        let refused = || Error::Arg(format!("invalid value for {}", Key::Range.id()));
        match protocol::range_choice(link.head.mode, text) {
            RangeChoice::Send(range) => {
                link.command(&protocol::set_range(range))?;
                link.head.range = range;
                link.head.auto_range = range == 0;
                Ok(())
            }
            RangeChoice::BadArg => Err(refused()),
            RangeChoice::NotApplicable => Err(Error::not_supported("range")),
            RangeChoice::UnknownMode => Err(Error::device("a mode with no range table")),
        }
    }

    /// `dev_acquisition_start` as far as its first request: a refusal is the
    /// start's own, and the rest goes on a thread.
    fn arm(&mut self) -> Result<Capture> {
        let mut link = self.link();
        link.settle()?;
        link.port()?.purge()?;
        let mut capture = Capture::new(
            Arc::clone(&self.link),
            Limits {
                samples: self.limit_samples,
                frames: self.limit_frames,
                msec: self.limit_msec,
            },
            self.silence,
        );
        match self.source {
            SOURCE_LIVE => {
                capture.flow = Flow::Live;
                link.send(&protocol::monitor(true))?;
            }
            SOURCE_SAVE => {
                link.send(&protocol::get_saved_count())?;
                let got = link.wait_data(protocol::CMD_GET_SAVED_COUNT)?;
                capture.save_count = got.data.unwrap_or(0) as usize;
                capture.flow = Flow::Saved;
                link.send(&protocol::get_saved(0))?;
            }
            source => {
                let index = source - SOURCE_RECORDINGS;
                if index >= link.record_count {
                    return Err(Error::device("no such recording"));
                }
                link.info_index = index;
                let name = link.names[index].clone();
                link.send(&protocol::get_rec_info(index))?;
                let got = link.wait_data(protocol::CMD_GET_REC_INFO)?;
                let info = got.info.unwrap_or_else(|| RecInfo {
                    name: name.clone().into_bytes(),
                    ..RecInfo::default()
                });
                // A late answer of another recording would label these samples, and
                // nothing in the samples can say so.
                if String::from_utf8_lossy(&info.name) != name.as_str() {
                    link.names[index] = name;
                    link.unsettled = true;
                    return Err(Error::device("the meter's info is of another recording"));
                }
                capture.samples_total = info.samples as usize;
                capture.unit = info.unit;
                capture.interval = Some(info.interval);
                capture.recording = index;
                capture.flow = Flow::Recording;
                link.send(&protocol::get_rec_samples(index, 0))?;
            }
        }
        capture.awaiting = capture.flow != Flow::Live;
        // The head of a frame kept here would take its tail, if that is still on its
        // way, for a whole one.
        link.rx.clear();
        Ok(capture)
    }
}

impl Device for UniTUt181aDevice {
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
        match group {
            None => Ok(OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let link = self.link();
        Ok(match key {
            Key::Conn => Value::from(self.conn.as_str()),
            Key::LimitFrames => Value::U64(self.limit_frames),
            Key::LimitSamples => Value::U64(self.limit_samples),
            Key::LimitMsec => Value::U64(self.limit_msec),
            Key::DataSource => Value::from(link.source_name(self.source)),
            Key::Datalog => Value::Bool(link.head.recording),
            Key::MeasuredQuantity => {
                let (mq, flags) = protocol::mq_of_mode(link.head.mode)
                    .ok_or_else(|| Error::not_supported("measured_quantity"))?;
                Value::Mq(mq, flags)
            }
            Key::Range => {
                let range = link.head.range_text().ok_or_else(|| Error::not_supported("range"))?;
                Value::from(range)
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // `core::config_set` refuses a 0 and says so; this is for a library
            // caller that comes straight here.
            Key::LimitFrames => {
                self.limit_frames = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitSamples => {
                self.limit_samples = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::LimitMsec => {
                self.limit_msec = value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?;
            }
            Key::DataSource => return self.set_data_source(value.as_str().ok_or_else(bad)?),
            // The meter has commands to start and stop a recording, and a boolean
            // cannot give their name, interval and duration.
            Key::Datalog => {
                let on = value.as_bool().ok_or_else(bad)?;
                log::error!("DIAG: record start/stop {}, currently ENOIMPL", u8::from(on));
                return Err(Error::not_supported("datalog"));
            }
            Key::MeasuredQuantity => return self.set_measured_quantity(value),
            Key::Range => return self.set_range(value),
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        if let Some(g) = group {
            return Err(Error::ChannelGroup(g.to_owned()));
        }
        match key {
            Key::DataSource => {
                let mut link = self.link();
                if !self.busy() {
                    link.settle()?;
                    link.update_recordings()?;
                }
                Ok(ValueList::Str(link.source_names()))
            }
            Key::MeasuredQuantity => Ok(ValueList::Mq(protocol::mq_list())),
            Key::Range => Ok(ValueList::Str(protocol::range_list())),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// One packet per value, of the channels a frame has: csv makes up the rest.
    fn note_output(&self, module: &str) {
        if module == "csv" {
            log::warn!(
                "Resulting CSV output data may be incomplete or incorrect: each value gets a \
                 row of its own, with 0 for the channels it is not of. Use -O analog."
            );
        }
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
        link.rx.clear();
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
        if self.busy() {
            return Err(Error::Busy);
        }
        // An End may be stuck on a full queue: ask it to give up, then join.
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
        let capture = self.arm()?;
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name("uni-t-ut181a acquisition".to_owned())
            .spawn(move || {
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

    /// Asks the thread to end, and does not wait: it may be sending. The thread
    /// turns monitor mode off, as `ut181a_handle_events` does.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// A packet, waiting for room for as long as nobody asked to stop, and after a
/// stop only for [`AFTER_STOP`]: a close joins this thread.
fn send_while_wanted(sink: &PacketSink, stop: &AtomicBool, packet: Packet) -> bool {
    let stopped_at = std::cell::Cell::new(None);
    sink.send_while(packet, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < AFTER_STOP
    })
}

fn send_end(sink: &PacketSink, stop: &AtomicBool, outcome: Result<()>) {
    let stopped_at = std::cell::Cell::new(None);
    sink.end_while(outcome, || {
        if !stop.load(Ordering::SeqCst) {
            return true;
        }
        let at = stopped_at.get().unwrap_or_else(Instant::now);
        stopped_at.set(Some(at));
        at.elapsed() < AFTER_STOP
    });
}

/// `sr_sw_limits`; 0 is none.
#[derive(Clone, Copy, Debug)]
struct Limits {
    samples: u64,
    frames: u64,
    msec: u64,
}

/// What comes in a capture and what ends it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Flow {
    /// Monitor mode's stream.
    Live,
    /// Saved measurements, a request after each answer.
    Saved,
    /// A recording's chunks, a request after each answer.
    Recording,
}

/// A running capture: the rest of `dev_context`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limits: Limits,
    silence: Silence,
    flow: Flow,
    /// `save_info`.
    save_index: usize,
    save_count: usize,
    /// `rec_data` and which recording it is.
    samples_done: usize,
    samples_total: usize,
    recording: usize,
    /// `last_data.unit_text`.
    unit: Vec<u8>,
    interval: Option<u16>,
    /// A request has gone with no answer yet.
    awaiting: bool,
    /// `frame_started`.
    frame_open: bool,
    samples_read: u64,
    frames_read: u64,
    began: Instant,
    last_byte: Instant,
    /// `SR_ST_STOPPING`: a limit, a stop, the end of a download, or a failure.
    stopping: bool,
    /// Why the capture gave up.
    failure: Option<Error>,
}

/// `Packet`s of the protocol that a pass has read, taken with the link free.
type Read = Vec<protocol::Packet>;

impl Capture {
    fn new(link: Arc<Mutex<Link>>, limits: Limits, silence: Silence) -> Capture {
        let now = Instant::now();
        Capture {
            link,
            limits,
            silence,
            flow: Flow::Live,
            save_index: 0,
            save_count: 0,
            samples_done: 0,
            samples_total: 0,
            recording: 0,
            unit: Vec::new(),
            interval: None,
            awaiting: false,
            frame_open: false,
            samples_read: 0,
            frames_read: 0,
            began: now,
            last_byte: now,
            stopping: false,
            failure: None,
        }
    }

    /// The capture ends, failed.
    fn give_up(&mut self, why: Error) {
        self.failure.get_or_insert(why);
        self.stopping = true;
    }

    fn run(mut self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        if !send_while_wanted(sink, stop, Packet::header()) {
            self.stopping = true;
        }
        // C's meta goes out ahead of its header, and sigrok-cli drops it.
        if let Some(seconds) = self.interval {
            let items = vec![(Key::SampleInterval, Value::U64(u64::from(seconds)))];
            send_while_wanted(sink, stop, Packet::Meta(items));
        }
        self.began = Instant::now();
        self.last_byte = self.began;
        let ended_by_error = loop {
            if stop.load(Ordering::SeqCst) {
                self.stopping = true;
            }
            if self.stopping {
                break false;
            }
            let (read, n) = match self.pass() {
                Ok(read) => read,
                Err(e) => {
                    log::error!("{e}");
                    self.failure.get_or_insert(e);
                    break true;
                }
            };
            if n > 0 {
                self.last_byte = Instant::now();
            }
            for packet in read {
                self.handle(packet, sink, stop);
            }
            self.check_time();
            self.check_silence();
            if n == 0 {
                std::thread::sleep(EMPTY_PAUSE);
            }
        };
        self.finish(ended_by_error, sink, stop);
        self.failure.map_or(Ok(()), Err)
    }

    /// One read, and the frames it made whole, with the link's lock held for that
    /// alone. In a live capture what comes behind garbage waits for the next
    /// bytes, as `ut181a_receive_data` runs only when there are some: the wire
    /// stays as C's. A download has no more bytes coming until it asks.
    fn pass(&self) -> Result<(Read, usize)> {
        let mut link = self.link.lock().expect("link lock");
        let mut buf = [0u8; protocol::RECV_BUFF];
        let room = link.rx.room().min(buf.len());
        let n = link.port()?.read(&mut buf[..room], PASS_WAIT)?;
        let mut read = Vec::new();
        if n == 0 && self.flow == Flow::Live {
            return Ok((read, 0));
        }
        link.rx.push(&buf[..n]);
        let Link { rx, head, .. } = &mut *link;
        let live = self.flow == Flow::Live;
        rx.frames(live, |frame| {
            let packet = protocol::parse_frame(frame, head);
            let good = packet.good();
            read.push(packet);
            good
        });
        Ok((read, n))
    }

    fn send(&mut self, packet: Packet, sink: &PacketSink, stop: &AtomicBool) {
        if !send_while_wanted(sink, stop, packet) {
            self.stopping = true;
        }
    }

    fn send_item(&mut self, item: &Item, sink: &PacketSink, stop: &AtomicBool) {
        let packet = Packet::Analog(AnalogData {
            channels: vec![item.channel],
            samples: vec![item.value],
            // What a unit text does not say is 0 and 0 in C, and prints no unit.
            mq: item.mq.unwrap_or(Mq::Voltage),
            flags: item.flags,
            unit: item.unit.unwrap_or(Unit::Unitless),
            digits: item.digits,
        });
        self.send(packet, sink, stop);
    }

    fn limit_reached(&self) -> bool {
        (self.limits.samples > 0 && self.samples_read >= self.limits.samples)
            || (self.limits.frames > 0 && self.frames_read >= self.limits.frames)
    }

    fn check_time(&mut self) {
        if self.limits.msec > 0 && self.began.elapsed().as_millis() >= u128::from(self.limits.msec)
        {
            self.stopping = true;
        }
    }

    fn check_silence(&mut self) {
        let quiet = self.last_byte.elapsed();
        if self.flow != Flow::Live {
            if quiet >= self.silence.download {
                log::error!("No answer from the meter for {} s.", quiet.as_secs());
                self.give_up(Error::device(format!(
                    "no answer from the meter for {} s",
                    quiet.as_secs()
                )));
            }
        } else if self.limits.msec == 0
            && (self.limits.samples > 0 || self.limits.frames > 0)
            && quiet >= self.silence.live
        {
            log::warn!("No reading from the meter for {} s.", quiet.as_secs());
            self.give_up(Error::device(format!(
                "no reading from the meter for {} s",
                quiet.as_secs()
            )));
        }
    }

    fn feed_value(&mut self, item: &Item, sink: &PacketSink, stop: &AtomicBool) {
        if self.stopping {
            return;
        }
        self.send_item(item, sink, stop);
        if item.counts {
            self.samples_read += 1;
            if self.limit_reached() {
                self.stopping = true;
            }
        }
    }

    fn end_frame(&mut self, sink: &PacketSink, stop: &AtomicBool) {
        if self.frame_open {
            self.send(Packet::FrameEnd, sink, stop);
            self.frame_open = false;
            self.frames_read += 1;
            if self.limit_reached() {
                self.stopping = true;
            }
        }
    }

    fn begin_frame(&mut self, sink: &PacketSink, stop: &AtomicBool) {
        if !self.frame_open {
            self.send(Packet::FrameBegin, sink, stop);
            self.frame_open = true;
        }
    }

    /// A measurement's way to the feed. Past a stop what is left of a buffer still
    /// begins and ends a frame, with no values, as `ut181a_feedbuff_start_frame`
    /// does not look at the state.
    fn feed_measurement(&mut self, m: &Measurement, sink: &PacketSink, stop: &AtomicBool) {
        if !m.began {
            return;
        }
        self.begin_frame(sink, stop);
        for item in &m.items {
            self.feed_value(item, sink, stop);
        }
        if let Some(comp) = &m.comp {
            comp.log();
        }
        if m.whole {
            self.end_frame(sink, stop);
        }
    }

    /// The feed and the next request, `process_packet` in a capture.
    fn handle(&mut self, packet: protocol::Packet, sink: &PacketSink, stop: &AtomicBool) {
        match packet {
            protocol::Packet::Measurement(m) if self.flow == Flow::Live => {
                self.feed_measurement(&m, sink, stop);
            }
            protocol::Packet::Saved(m) if self.flow == Flow::Saved => {
                self.awaiting = false;
                self.save_index += 1;
                self.feed_measurement(&m, sink, stop);
                if m.whole {
                    self.next_saved();
                }
            }
            protocol::Packet::Chunk(Some(chunk)) if self.flow == Flow::Recording => {
                self.awaiting = false;
                if self.stopping {
                    return;
                }
                self.feed_chunk(&chunk, sink, stop);
                if chunk.whole {
                    self.next_chunk(chunk.count);
                }
            }
            protocol::Packet::ReplyCode(false) if self.flow != Flow::Live && !self.stopping => {
                self.awaiting = false;
                log::error!("The meter refused the request.");
                self.give_up(Error::device("the meter refused the request"));
            }
            // Stale, or stray: no sample and no next request comes of them.
            _ => {}
        }
    }

    fn request(&mut self, frame: &[u8]) {
        self.check_time();
        if self.stopping {
            return;
        }
        let mut link = self.link.lock().expect("link lock");
        if let Err(e) = link.send(frame) {
            drop(link);
            self.give_up(e);
            return;
        }
        self.awaiting = true;
    }

    fn next_saved(&mut self) {
        if self.stopping {
            return;
        }
        if self.save_index >= self.save_count {
            self.stopping = true;
            return;
        }
        self.request(&protocol::get_saved(self.save_index));
    }

    fn feed_chunk(&mut self, chunk: &protocol::Chunk, sink: &PacketSink, stop: &AtomicBool) {
        self.samples_done = self.samples_done.wrapping_add(usize::from(chunk.count));
        let unit = std::mem::take(&mut self.unit);
        for &(value, prec) in &chunk.samples {
            if self.stopping {
                continue;
            }
            self.begin_frame(sink, stop);
            let item = protocol::item(0, value, prec, &unit, crate::core::MqFlags::empty(), true);
            self.feed_value(&item, sink, stop);
            self.end_frame(sink, stop);
        }
        self.unit = unit;
    }

    fn next_chunk(&mut self, count: u8) {
        if self.stopping {
            return;
        }
        if self.samples_done >= self.samples_total {
            self.stopping = true;
            return;
        }
        if count == 0 {
            log::warn!(
                "The meter has {} samples of the {} it said.",
                self.samples_done,
                self.samples_total
            );
            self.give_up(Error::device(format!(
                "the meter has {} samples of the {} it said",
                self.samples_done, self.samples_total
            )));
            return;
        }
        self.request(&protocol::get_rec_samples(self.recording, self.samples_done));
    }

    /// The end of `ut181a_handle_events`: monitor mode off, and one look at what
    /// has come, which feeds nothing; or for a download nothing at all.
    fn finish(&mut self, ended_by_error: bool, sink: &PacketSink, stop: &AtomicBool) {
        let mut link = self.link.lock().expect("link lock");
        link.unsettled = true;
        let live = self.flow == Flow::Live && !ended_by_error;
        if live {
            let _ = link.send(&protocol::monitor(false));
            link.owed = Some(Want { code: true, ..Want::default() });
        } else if self.awaiting {
            link.owed = Some(Want { answer: true, ..Want::default() });
        }
        drop(link);
        if live {
            self.stopping = true;
            if let Ok((read, _)) = self.pass() {
                for packet in read {
                    match packet {
                        protocol::Packet::Measurement(m) => self.feed_measurement(&m, sink, stop),
                        // The monitor off's code has come: nothing is owed.
                        protocol::Packet::ReplyCode(_) => {
                            self.link.lock().expect("link lock").owed = None;
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
