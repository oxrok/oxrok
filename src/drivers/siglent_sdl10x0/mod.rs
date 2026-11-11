// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sdl10x0/api.c,
//   libsigrok/src/hardware/siglent-sdl10x0/protocol.c,
//   Copyright (C) 2024 Timo Boettcher <timo@timoboettcher.name>,
//   with commits by Bert Vermeulen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Siglent's SDL1000X electronic loads: the SDL1020X, SDL1030X and their -E versions,
//! on USBTMC and on a TCP socket. C has VXI and the rest, and oxrok hasn't.
//!
//! A scan asks `*IDN?` and closes. An open and a close send nothing beyond what the
//! transport does. `--get` asks the load each time, and a setting is a line with no
//! answer: a value the load refuses goes in its error queue, which C doesn't read and
//! this doesn't either, as that would be a request C doesn't send. So a setpoint the
//! load would refuse is refused here before anything goes out, and a `-c` with one
//! sends none of its lines: see [`setpoint`].
//!
//! A capture asks `MEAS:VOLT?` and `MEAS:CURR?` in turn, one question out at a time,
//! and a frame is the two. An answer is gathered up to its line end, which the guide's
//! and a real load's have. When a limit is met the next question goes out anyway, and
//! nobody reads its answer, as in C. So the next thing sent on this connection, by a
//! client or a second capture, resyncs the line first ([`Link::resync`]): on TCP by
//! connecting again, as a new connection can't receive an old answer; on USBTMC by a
//! device clear; on a serial port by waiting until it has been quiet for twice the
//! slowest answer yet. So does anything that isn't one answer to the question: junk,
//! more than a line, or the last answer again (a copy). The round then starts again
//! from the voltage, so V and I can't trade places.
//!
//! What can still go wrong: an answer that is a number but the wrong one (a digit's
//! damage, which no check sees), a copy of an answer that comes after the next answer
//! instead of ahead of it, a copy on a load whose two readings came out alike (after
//! which copies aren't looked for), and on a serial port an answer later than the
//! quiet. On USBTMC a load that wants its bulk OUT halt cleared after a clear fails
//! the next write.
//!
//! Where this leaves `api.c` and `protocol.c`:
//!
//! - **A capture on USBTMC works.** C's sends one question and is over 100 ms later
//!   with exit 0: `usb_source_add` has no fd that a read could wake, as no transfer is
//!   in flight, so its callback sees a timeout with `revents` 0 (`protocol.c:169-173`).
//! - **Late answers.** On TCP the same callback ends a capture, with exit 0, at any
//!   answer later than 100 ms. Here there is no re-ask, so a late answer can't pass for
//!   the next one's: the question stays out and the read goes on. On USBTMC the one
//!   request is waited out ([`Scpi::wait_out_requests`]), as a second one would get the
//!   first's answer, whose tag is no header for it. With no sample yet,
//!   10 s of silence fails the capture, or with a time limit warns and exits 0. After
//!   a sample it warns at 10 s and fails at 10 minutes, and says when the load is back.
//!   Neither the manual nor C has a pace to go by, so these are choices. What it costs:
//!   a load that lost a question waits out those 10 minutes.
//! - **What isn't a number isn't a reading.** `sr_atof_ascii` gives 0 for an empty
//!   answer and takes `nan`, `inf` and hexadecimal. C ends its capture, with exit 0, on
//!   what it refuses. Here the line is resynced and the round asked again, three times
//!   since the last frame, and then the capture fails.
//! - **An answer in pieces, or twice,** shifts C's readings for good: it takes a short
//!   read for a whole answer. Here a piece waits for its line end, and a copy resyncs.
//! - **A setpoint the load would refuse**, or one with more than three places, is
//!   refused; C sends it, and a `-c` that turns the input on after it does so at the old
//!   setting.
//! - **A time limit** that comes with no sample yet looks on for 2 s, where C's check
//!   ends a capture after a lone voltage, with no reading if a round takes longer.
//! - **`ocp_threshold` reads a number.** `api.c:265` reads a boolean, so `--get` fails
//!   on what the manual shows the load answers (58), and on a `1` returns a float it
//!   never set (`api.c:268`).
//! - **`regulation`** takes the words its list gives, which C refuses, and refuses
//!   `Unknown`, which C sends as `:FUNC Unknown`; see [`protocol::regulation`].
//! - **A NaN or an infinity** is refused, where C sends `:VOLT:LEV:IMM nan`.
//! - **A get or a set during a capture** is its own exchange: the capture holds the
//!   line for one question at a time, so a get reads its own answer. In C it reads what
//!   the capture asked for. While a question is out on a silent line, a get says Busy.
//! - **A write that fails** ends a capture: C drops `sr_scpi_send`'s result, and on TCP
//!   dies of SIGPIPE at it. A closed connection is an error, where C reads 0 bytes.
//! - A setting stays a double, as in the driver. sigrok-cli's `strtof` narrows it
//!   first, which shows at three places from about 16777 up, past every range here.
//!
//! What the driver calls "active" is what the manual calls enabled (58, 59), and what
//! it calls enabled is a constant: kept, with its wire.
//!
//! What the format can't catch: an answer has no check, so damage to a digit is
//! another number (`protocol::tests::what_damage_gives_a_wrong_value` counts it).

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Unit, Value,
    ValueList,
};
use crate::scpi::{self, HwInfo, Scpi};
use protocol::{ASK_CURRENT, ASK_VOLTAGE, Bounds, Query, Step, Watch};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::ElectronicLoad];
const GROUP: &str = "1";

static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
];
static GROUP_OPTIONS: [OptionDesc; 16] = [
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::Regulation, Access::GET_SET_LIST),
    OptionDesc::new(Key::Voltage, Access::GET),
    OptionDesc::new(Key::VoltageTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Current, Access::GET),
    OptionDesc::new(Key::CurrentLimit, Access::GET_SET_LIST),
    OptionDesc::new(Key::Power, Access::GET),
    OptionDesc::new(Key::PowerTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::Resistance, Access::GET),
    OptionDesc::new(Key::ResistanceTarget, Access::GET_SET_LIST),
    OptionDesc::new(Key::OverPowerProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverPowerProtectionActive, Access::GET),
    OptionDesc::new(Key::OverPowerProtectionThreshold, Access::GET_SET),
    OptionDesc::new(Key::OverCurrentProtectionEnabled, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionActive, Access::GET),
    OptionDesc::new(Key::OverCurrentProtectionThreshold, Access::GET_SET),
];

/// How long a stopped capture waits to deliver its End to a queue that is full.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How long a get waits for a capture's unanswered question before it says Busy.
const GET_WAITS: Duration = Duration::from_secs(2);
/// How often that wait looks, and a capture's wait for a client.
const LOOK: Duration = Duration::from_millis(2);
/// How long a capture holds back for clients at most, between two questions.
const CLIENTS_WAIT: Duration = Duration::from_secs(5);

/// The `siglent-sdl10x0` driver.
pub struct SiglentSdl10x0Driver;

/// `probe_device`: a load of the tables, closed, or `None`.
fn probe(mut scpi: Scpi) -> Option<Sdl10x0Device> {
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::info!("Couldn't get IDN response: {e}.");
            return None;
        }
    };
    if !protocol::MANUFACTURERS.contains(&id.manufacturer.as_str())
        || !protocol::MODELS.contains(&id.model.as_str())
    {
        log::info!("{} {} is no load of this driver's.", id.manufacturer, id.model);
        return None;
    }
    // The probe's connection ends with it, and a failed close loses no device.
    let _ = scpi.close();
    Some(Sdl10x0Device::new(&id, scpi))
}

impl Driver for SiglentSdl10x0Driver {
    fn id(&self) -> &'static str {
        "siglent-sdl10x0"
    }

    fn long_name(&self) -> &'static str {
        "SIGLENT SDL10x0"
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
        // What fails here is no device, as `sr_scpi_scan_resource` has it.
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

/// What a capture and a client share. A capture's lock is one exchange long.
struct Link {
    scpi: Option<Scpi>,
    limit_samples: u64,
    limit_msec: u64,
    /// A capture's question is out, unanswered after a read's time.
    question_out: bool,
    /// What's on the line can't be trusted to answer what's asked next: a question
    /// was left, or junk came. [`Link::resync`] before the next.
    owed: bool,
    capturing: bool,
    /// What came of an answer whose line end hasn't.
    rest: Vec<u8>,
    /// A capture's last answer as it came: the same text to the next question is a
    /// copy, and every answer after it would be the one before's.
    last: Option<String>,
    /// How a copy of an answer is looked for.
    copies: Copies,
    /// The slowest answer a capture had, which a quiet line must outlast.
    slowest: Duration,
}

/// How a capture looks for a copy of an answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Copies {
    /// The same text twice in a row resyncs.
    Look,
    /// A copy made the last resync since the last frame: the same again is no copy
    /// but a load whose two readings are alike.
    Resynced,
    /// The readings came out alike over a resync: no more looking.
    Off,
}

/// The longest answer taken: a reading is ten characters or so.
const LONGEST: usize = 64;
/// The longest an answer can count for in [`Link::slowest`].
const SLOWEST: Duration = Duration::from_secs(10);
/// How many times a resync waits for quiet before it gives up and warns.
const QUIET_TURNS: u32 = 10;

impl Link {
    fn scpi(&mut self) -> Result<&mut Scpi> {
        self.scpi.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// The connection, resynced first if [`Link::owed`].
    fn clean(&mut self) -> Result<&mut Scpi> {
        if self.owed {
            self.resync()?;
        }
        self.scpi()
    }

    /// Gets rid of whatever is on its way: on TCP by connecting again, as a new
    /// connection can't receive an old answer; on USBTMC by a device clear; elsewhere
    /// by waiting until the line has been quiet for longer than the slowest answer yet,
    /// which an answer later still gets past.
    fn resync(&mut self) -> Result<()> {
        self.owed = false;
        self.rest.clear();
        self.last = None;
        let slowest = self.slowest;
        let scpi = self.scpi()?;
        let quiet = scpi.read_timeout.max(slowest * 2);
        if scpi.resource().starts_with("tcp") {
            let _ = scpi.close();
            return scpi.reopen();
        }
        if scpi.clear()? {
            return Ok(());
        }
        for _ in 0..QUIET_TURNS {
            match scpi.discard(quiet)? {
                Some(0) | None => return Ok(()),
                Some(_) => {}
            }
        }
        log::warn!("The load still sends what nobody asked for.");
        Ok(())
    }

    /// A whole answer, out of [`Link::rest`]: `Ok(None)` while its line end is to come,
    /// and `Err` with what came for more than one line or one too long, which nothing
    /// asked. The guide's answers end in LF, and so do those of the Python driver
    /// written against a real load.
    fn answer(&mut self) -> std::result::Result<Option<String>, String> {
        let Some(end) = self.rest.iter().position(|b| *b == b'\n') else {
            if self.rest.len() > LONGEST {
                return Err(String::from_utf8_lossy(&std::mem::take(&mut self.rest)).into_owned());
            }
            return Ok(None);
        };
        let all = std::mem::take(&mut self.rest);
        if end + 1 < all.len() {
            return Err(String::from_utf8_lossy(&all).into_owned());
        }
        Ok(Some(answer_text(&all)))
    }

    /// A client's question and its answer, gathered up to the line end within a
    /// read's time of silence. Anything else leaves the line to resync.
    fn exchange(&mut self, command: &str) -> Result<String> {
        self.clean()?.send(command)?;
        loop {
            let polled = self.scpi()?.poll_answer();
            match polled {
                Ok(Some(data)) => {
                    self.rest.extend_from_slice(&data);
                    match self.answer() {
                        Ok(Some(text)) => return Ok(text),
                        Ok(None) => {}
                        Err(junk) => {
                            self.owed = true;
                            return Err(Error::device(format!(
                                "'{command}': no usable answer: '{}'",
                                junk.escape_debug()
                            )));
                        }
                    }
                }
                Ok(None) => {
                    self.owed = true;
                    return Err(Error::Timeout);
                }
                Err(e) => {
                    self.owed = true;
                    return Err(e);
                }
            }
        }
    }
}

/// The lock, with a capture told to wait: a `Mutex` isn't fair, and it takes this lock
/// again right after it lets go.
#[derive(Clone)]
struct Shared {
    link: Arc<Mutex<Link>>,
    clients: Arc<AtomicUsize>,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    fn client(&self) -> MutexGuard<'_, Link> {
        self.clients.fetch_add(1, Ordering::SeqCst);
        let guard = self.lock();
        self.clients.fetch_sub(1, Ordering::SeqCst);
        guard
    }

    /// A client's turn at the line with no capture's question on it.
    fn asker(&self) -> Result<MutexGuard<'_, Link>> {
        let began = Instant::now();
        loop {
            let guard = self.client();
            if !guard.question_out {
                return Ok(guard);
            }
            drop(guard);
            if began.elapsed() >= GET_WAITS {
                return Err(Error::Busy);
            }
            std::thread::sleep(LOOK);
        }
    }
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

/// A set, checked.
enum Setting {
    Line(String),
    LimitSamples(u64),
    LimitMsec(u64),
}

/// One load.
pub struct Sdl10x0Device {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    max_power: f64,
    shared: Shared,
    /// The connection, while closed.
    closed: Option<Scpi>,
    worker: Option<Worker>,
    bounds: Bounds,
}

impl Sdl10x0Device {
    fn new(id: &HwInfo, scpi: Scpi) -> Self {
        Sdl10x0Device {
            info: DeviceInfo {
                driver: "siglent-sdl10x0",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "1".to_owned())],
            groups: vec![ChannelGroup { name: GROUP.to_owned(), channels: vec![0] }],
            max_power: protocol::max_power(&id.model),
            shared: Shared {
                link: Arc::new(Mutex::new(Link {
                    scpi: None,
                    limit_samples: 0,
                    limit_msec: 0,
                    question_out: false,
                    owed: false,
                    capturing: false,
                    rest: Vec::new(),
                    last: None,
                    copies: Copies::Look,
                    slowest: Duration::ZERO,
                })),
                clients: Arc::new(AtomicUsize::new(0)),
            },
            closed: Some(scpi),
            worker: None,
            bounds: Bounds::default(),
        }
    }

    /// A device that a scan of `scpi` found; [`Device::open`] opens it again. For
    /// tests and for clients with a transport of their own.
    ///
    /// # Errors
    ///
    /// When it is no load of this driver's, or does not answer.
    pub fn from_scpi(scpi: Scpi) -> Result<Self> {
        probe(scpi).ok_or_else(|| Error::device("Not a load of this driver's."))
    }

    fn check(&self, key: &Key, group: Option<&str>) -> Result<()> {
        if self.options(group)?.iter().any(|o| &o.key == key) {
            Ok(())
        } else {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
    }

    /// A line with no answer. What a client reads next may tell how it ended.
    fn send(&self, line: &str) -> Result<()> {
        self.shared.client().clean()?.send(line).inspect_err(|e| {
            log::error!("Sending '{line}' failed, so the load's state isn't known: {e}.");
        })
    }

    fn ask(&self, query: Query) -> Result<Value> {
        let text_of = |command: &str| -> Result<String> { self.shared.asker()?.exchange(command) };
        Ok(match query {
            Query::Always(value) => Value::Bool(value),
            Query::Text(command) => Value::Str(text_of(command)?),
            Query::Bool(command) => {
                let text = text_of(command)?;
                Value::Bool(scpi::parse_bool(&text).ok_or_else(|| {
                    Error::device(format!("'{command}' says what's no boolean: '{text}'"))
                })?)
            }
            Query::Number(command) => {
                let text = text_of(command)?;
                let value = protocol::reading(&text).ok_or_else(|| {
                    Error::device(format!("'{command}' says what's no number: '{text}'"))
                })?;
                Value::F64(f64::from(value))
            }
        })
    }

    /// What a set comes to, refused before anything goes out.
    fn setting(&self, key: &Key, value: &Value) -> Result<Setting> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        Ok(match key {
            Key::LimitSamples => {
                Setting::LimitSamples(value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?)
            }
            Key::LimitMsec => {
                Setting::LimitMsec(value.as_u64().filter(|n| *n > 0).ok_or_else(bad)?)
            }
            Key::Enabled => {
                let on = value.as_bool().ok_or_else(bad)?;
                Setting::Line(if on { ":INPUT ON" } else { ":INPUT OFF" }.to_owned())
            }
            Key::Regulation => {
                let word = value.as_str().and_then(protocol::regulation).ok_or_else(bad)?;
                Setting::Line(format!(":FUNC {word}"))
            }
            other => {
                let (command, min, max) = protocol::setter(other, self.max_power)
                    .ok_or_else(|| Error::UnknownKey(other.id().to_owned()))?;
                let number = value.as_f64().ok_or_else(bad)?;
                Setting::Line(setpoint(other, command, number, min, max)?)
            }
        })
    }

    /// What the worker holds on to, ended.
    fn join(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

/// A setpoint's line, or why it can't go: the load refuses a number outside its range
/// in silence and keeps what it had, so a `-c` that switched the input on after it
/// would do so at a setting nobody knows. The ranges are the guide's: 30 A and 150 V
/// at the top of `IRANGe` and `VRANGe` (PDF 20, 7), the model's wattage, and C's list
/// for the resistance, which the guide gives none of. A number with more than three
/// places would go out as another (`0.0004` as `0.000`), so it's refused too.
fn setpoint(key: &Key, command: &str, value: f64, min: f64, max: f64) -> Result<String> {
    let id = key.id();
    if !value.is_finite() {
        return Err(Error::Arg(format!("{id} takes a number, not {value}")));
    }
    if value < min || value > max {
        return Err(Error::Arg(format!(
            "{id} {value} is outside the load's {min} to {max}, and the load would keep its \
             old setting with no word"
        )));
    }
    let thousandths = (value * 1000.0).round();
    if (thousandths / 1000.0 - value).abs() > 1e-9 * value.abs().max(1.0) {
        return Err(Error::Arg(format!(
            "{id} {value} has more than three places, and would go out as {:.3}",
            thousandths / 1000.0 + 0.0
        )));
    }
    // `-0.0` goes out as 0.
    Ok(format!("{command} {:.3}", thousandths / 1000.0 + 0.0))
}

impl Device for Sdl10x0Device {
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
            Some(g) if g.eq_ignore_ascii_case(GROUP) => Ok(GROUP_OPTIONS.to_vec()),
            Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        }
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        self.check(key, group)?;
        match key {
            Key::LimitSamples => Ok(Value::U64(self.shared.client().limit_samples)),
            Key::LimitMsec => Ok(Value::U64(self.shared.client().limit_msec)),
            other => {
                let query = protocol::query(other)
                    .ok_or_else(|| Error::UnknownKey(other.id().to_owned()))?;
                self.ask(query)
            }
        }
    }

    fn check_set(&self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        self.check(key, group)?;
        self.setting(key, value).map(|_| ())
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        self.check(key, group)?;
        match self.setting(key, value)? {
            Setting::Line(line) => self.send(&line),
            Setting::LimitSamples(limit) => {
                self.shared.client().limit_samples = limit;
                Ok(())
            }
            Setting::LimitMsec(limit) => {
                self.shared.client().limit_msec = limit;
                Ok(())
            }
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let unknown = || Error::UnknownKey(key.id().to_owned());
        if group.is_none() {
            return Err(unknown());
        }
        self.options(group)?;
        Ok(match key {
            Key::Regulation => ValueList::Str(
                ["CURRENT", "VOLTAGE", "POWER", "RESISTANCE"].map(str::to_owned).to_vec(),
            ),
            Key::VoltageTarget
            | Key::CurrentLimit
            | Key::PowerTarget
            | Key::ResistanceTarget
            | Key::OverPowerProtectionThreshold
            | Key::OverCurrentProtectionThreshold => {
                let (_, min, max) = protocol::setter(key, self.max_power).ok_or_else(unknown)?;
                let step = if *key == Key::ResistanceTarget { 0.01 } else { 0.001 };
                ValueList::RangeStepF64 { min, max, step }
            }
            _ => return Err(unknown()),
        })
    }

    /// `sr_scpi_open`: the transport's own opening, and no message.
    fn open(&mut self) -> Result<()> {
        let mut link = self.shared.lock();
        if link.scpi.is_some() {
            return Ok(());
        }
        let Some(mut scpi) = self.closed.take() else {
            return Err(Error::device("the device has no connection"));
        };
        if let Err(e) = scpi.reopen() {
            self.closed = Some(scpi);
            return Err(e);
        }
        // One request per question on USBTMC, waited out: see `ask`.
        scpi.wait_out_requests(true);
        link.scpi = Some(scpi);
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.join();
        let mut link = self.shared.lock();
        link.capturing = false;
        link.question_out = false;
        link.rest.clear();
        link.last = None;
        if let Some(mut scpi) = link.scpi.take() {
            let _ = scpi.close();
            // A new connection can't hear an old answer; a USBTMC device keeps it.
            if scpi.resource().starts_with("tcp") {
                link.owed = false;
            }
            self.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        {
            let mut link = self.shared.lock();
            if link.capturing {
                return Err(Error::Busy);
            }
            if link.scpi.is_none() {
                return Err(Error::device("device is not open"));
            }
            link.capturing = true;
        }
        // Over, by what it said ahead of its End, and maybe still sending that.
        self.join();
        let capture = Capture { shared: self.shared.clone(), bounds: self.bounds };
        let shared = self.shared.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let spawned = std::thread::Builder::new()
            .name("siglent-sdl10x0 acquisition".to_owned())
            .spawn(move || {
                let outcome = capture.run(&sink, &stop2);
                // Ahead of the End, so that a client that sees it can start again.
                shared.lock().capturing = false;
                send_end(&sink, &stop2, outcome);
            });
        match spawned {
            Ok(handle) => {
                self.worker = Some(Worker { stop, handle });
                Ok(())
            }
            Err(e) => {
                self.shared.lock().capturing = false;
                Err(Error::Io(e))
            }
        }
    }

    /// Asks the thread to end, and does not wait: it may be sending.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

impl Drop for Sdl10x0Device {
    /// A close, but in a panic, where a thread that doesn't end must not hold it.
    fn drop(&mut self) {
        if std::thread::panicking() {
            let _ = self.stop();
        } else {
            let _ = self.close();
        }
    }
}

/// How asking went.
enum Asked {
    /// The answer, as a number.
    Reading(f32),
    /// It made nothing: start the round over.
    Again,
    /// The capture is over, and well: `Ok` from [`Capture::run`].
    Over,
}

/// One capture, on its own thread: `siglent_sdl10x0_receive_data`.
struct Capture {
    shared: Shared,
    bounds: Bounds,
}

impl Capture {
    fn run(&self, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let started = Instant::now();
        let mut watch = Watch::new(self.bounds);
        {
            let mut link = self.shared.lock();
            link.last = None;
            link.copies = Copies::Look;
        }
        loop {
            let volts = match self.ask(ASK_VOLTAGE, &mut watch, started, stop)? {
                Asked::Reading(v) => v,
                Asked::Again => continue,
                Asked::Over => return Ok(()),
            };
            if self.limit_met(&watch, started) {
                self.leave_question(ASK_CURRENT, stop);
                return Ok(());
            }
            let amps = match self.ask(ASK_CURRENT, &mut watch, started, stop)? {
                Asked::Reading(v) => v,
                Asked::Again => continue,
                Asked::Over => return Ok(()),
            };
            let mut link = self.shared.lock();
            if link.copies == Copies::Resynced {
                link.copies = Copies::Look;
            }
            drop(link);
            for packet in frame(volts, amps) {
                if !sink.send_while(packet, go_on) {
                    return Ok(());
                }
            }
            watch.sample();
            if self.limit_met(&watch, started) {
                self.leave_question(ASK_VOLTAGE, stop);
                return Ok(());
            }
        }
    }

    /// `sr_sw_limits_check`, but that a time limit waits for a first frame: a lone
    /// voltage makes no reading. [`Watch::empty_read`] bounds that wait.
    fn limit_met(&self, watch: &Watch, started: Instant) -> bool {
        let (samples, msec) = {
            let link = self.shared.lock();
            (link.limit_samples, link.limit_msec)
        };
        (samples > 0 && watch.samples() >= samples)
            || (msec > 0 && watch.samples() > 0 && started.elapsed() >= Duration::from_millis(msec))
    }

    /// A limit met: C sends what comes next ahead of its look at the limit, and nobody
    /// reads the answer. Its failure ends nothing, as the capture is over.
    fn leave_question(&self, question: &str, stop: &AtomicBool) {
        if stop.load(Ordering::SeqCst) {
            return;
        }
        let mut link = self.shared.lock();
        if let Ok(scpi) = link.scpi()
            && let Err(e) = scpi.send(question)
        {
            log::debug!("Sending '{question}' at the limit: {e}.");
        }
        link.owed = true;
    }

    /// Lets clients in, ahead of a question.
    fn hold_for_clients(&self, stop: &AtomicBool) {
        let began = Instant::now();
        while self.shared.clients.load(Ordering::SeqCst) > 0
            && !stop.load(Ordering::SeqCst)
            && began.elapsed() < CLIENTS_WAIT
        {
            std::thread::sleep(LOOK);
        }
    }

    /// One question and its answer, with no question left marked out whatever comes.
    fn ask(
        &self,
        question: &str,
        watch: &mut Watch,
        started: Instant,
        stop: &AtomicBool,
    ) -> Result<Asked> {
        let asked = self.asking(question, watch, started, stop);
        self.shared.lock().question_out = false;
        asked
    }

    /// One question, and the wait for its answer, which is the only thing it can get:
    /// nothing else goes out meanwhile that wants one. It's never asked twice: on a line
    /// with no tags a second asking would make every later reading the answer to the one
    /// before, and on USBTMC the transport reads for the one request it made. An answer
    /// is gathered up to its line end. One that isn't a number, comes with more than a
    /// line, or is the last answer over again (a copy, which would shift all after it)
    /// resyncs the line, and the round starts again from the voltage.
    fn asking(
        &self,
        question: &str,
        watch: &mut Watch,
        started: Instant,
        stop: &AtomicBool,
    ) -> Result<Asked> {
        self.hold_for_clients(stop);
        let mut link = self.shared.lock();
        link.clean()?.send(question).inspect_err(|e| {
            log::error!("Asking '{question}' failed: {e}.");
        })?;
        let asked_at = Instant::now();
        loop {
            if stop.load(Ordering::SeqCst) {
                link.owed = true;
                return Ok(Asked::Over);
            }
            let limit_msec = link.limit_msec;
            // The time ahead of the read: a stall in it must not look like silence.
            let before = Instant::now();
            let past = (limit_msec > 0)
                .then(|| {
                    before.duration_since(started).checked_sub(Duration::from_millis(limit_msec))
                })
                .flatten();
            let read = link.scpi()?.poll_answer();
            let took = before.elapsed();
            let data = match read {
                Ok(data) => data,
                Err(e) if stop.load(Ordering::SeqCst) => {
                    log::debug!("Read error after a stop: {e}.");
                    link.owed = true;
                    return Ok(Asked::Over);
                }
                Err(e) => {
                    log::error!("Reading the answer to '{question}' failed: {e}.");
                    link.owed = true;
                    return Err(e);
                }
            };
            let Some(data) = data else {
                if !link.rest.is_empty() {
                    // Silence after a piece: an answer cut short, no answer to wait for.
                    let cut = String::from_utf8_lossy(&std::mem::take(&mut link.rest)).into_owned();
                    let why = format!("'{}' with no line end", cut.escape_debug());
                    return Self::junk(link, watch, &why, limit_msec > 0);
                }
                link.question_out = true;
                drop(link);
                match watch.empty_read(took, past, limit_msec > 0) {
                    Step::Wait => {}
                    Step::Warn(text) => log::warn!("{text}"),
                    Step::DoneWarning(text) => {
                        log::warn!("{text}");
                        self.over_with_question_out();
                        return Ok(Asked::Over);
                    }
                    Step::Fail(text) => {
                        log::error!("{text}");
                        self.over_with_question_out();
                        return Err(Error::device(text));
                    }
                }
                self.hold_for_clients(stop);
                link = self.shared.lock();
                continue;
            };
            link.rest.extend_from_slice(&data);
            let text = match link.answer() {
                // Its line end is still to come: the same question, the same wait.
                Ok(None) => continue,
                Ok(Some(text)) => text,
                Err(junk) => junk,
            };
            link.question_out = false;
            if let Some(line) = watch.heard() {
                log::warn!("{line}");
            }
            let reading = protocol::reading(&text);
            let mut copy = link.copies != Copies::Off
                && link.last.as_deref() == Some(text.as_str())
                && reading.is_some_and(|v| v != 0.0);
            if copy && link.copies == Copies::Resynced {
                // The same over a clean line: what the load reads, alike.
                link.copies = Copies::Off;
                copy = false;
            }
            if copy {
                link.copies = Copies::Resynced;
            }
            if let (Some(value), false) = (reading, copy) {
                link.slowest = link.slowest.max(asked_at.elapsed().min(SLOWEST));
                link.last = Some(text);
                return Ok(Asked::Reading(value));
            }
            let why = if copy {
                format!("the same answer twice: '{}'", text.escape_debug())
            } else {
                format!("'{}'", text.escape_debug())
            };
            return Self::junk(link, watch, &why, limit_msec > 0);
        }
    }

    /// What came is no answer: the line is resynced, and the round starts over or,
    /// after too many, the capture ends.
    fn junk(
        mut link: MutexGuard<'_, Link>,
        watch: &mut Watch,
        why: &str,
        time_limit: bool,
    ) -> Result<Asked> {
        link.question_out = false;
        link.resync().inspect_err(|e| {
            log::error!("Resyncing the line failed: {e}.");
        })?;
        drop(link);
        match watch.unusable(why, time_limit) {
            Step::Warn(text) => {
                log::warn!("{text}");
                Ok(Asked::Again)
            }
            Step::DoneWarning(text) => {
                log::warn!("{text}");
                Ok(Asked::Over)
            }
            Step::Fail(text) => {
                log::error!("{text}");
                Err(Error::device(text))
            }
            Step::Wait => Ok(Asked::Again),
        }
    }

    fn over_with_question_out(&self) {
        self.shared.lock().owed = true;
    }
}

/// An answer without its line end, as `sr_scpi_get_string` has it.
fn answer_text(data: &[u8]) -> String {
    let data = data.strip_suffix(b"\n").unwrap_or(data);
    let data = data.strip_suffix(b"\r").unwrap_or(data);
    String::from_utf8_lossy(data).into_owned()
}

/// `siglent_sdl10x0_send_value` twice, in a frame.
fn frame(volts: f32, amps: f32) -> [Packet; 4] {
    let analog = |value, mq, unit| {
        Packet::Analog(AnalogData {
            channels: vec![0],
            samples: vec![value],
            mq,
            flags: MqFlags::DC,
            unit,
            digits: 7,
        })
    };
    [
        Packet::FrameBegin,
        analog(volts, Mq::Voltage, Unit::Volt),
        analog(amps, Mq::Current, Unit::Ampere),
        Packet::FrameEnd,
    ]
}

/// The End, for which a client that goes on reading is waited for without a bound, and
/// after a stop only for [`END_AFTER_STOP`].
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

#[cfg(test)]
mod tests;
