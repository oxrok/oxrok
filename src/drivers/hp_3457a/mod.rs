// Counterpart in sigrok:
//   libsigrok/src/hardware/hp-3457a/api.c,
//   libsigrok/src/hardware/hp-3457a/protocol.c,
//   Copyright (C) 2016 Alexandru Gagniuc <mr.nuke.me@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The HP 3457A multimeter. A real one has HP-IB, which oxrok does not: it goes through
//! [`scpi`], so through whatever stands between the two as `conn=`, `tcp-raw/HOST/PORT`
//! being the one tested, on the emulator's model. It speaks no SCPI (no `*IDN?`), and
//! commands with no question mark answer, such as `?`, `TRIG SGL` and `RMATH HIRES`.
//! Nothing is probed without a `conn=`, as `END ALWAYS` is no command for anything else.
//!
//! A scan sends `END ALWAYS`, asks `ID?`, `REV?` and `OPT?` and closes. An open sends
//! `PRESET`, `INBUF ON` and `TRIG HOLD`, and asks `NPLC?`. A close sends `SADV HOLD` and
//! `TRIG AUTO`. The quantity can't be read back, so without `measured_quantity` samples
//! have no unit, as in C. [`protocol`] has a measurement's steps.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A silence: on TCP C blocks for good. Here a question waits for its answer and is
//!   not asked again: at 100 PLC a reading takes seconds, and a second trigger earns an
//!   error and then an answer too many, taken for the next one. With a sample limit, a
//!   failure at 60 s with no sample yet, and after one a warning at 60 s and a failure
//!   at 10 minutes. With no limit a warning at 60 s, and a line when it answers again.
//! - Numbers: an empty answer, hexadecimal, `inf`, `nan` and an underflow are no number,
//!   where C reads the first as 0 and passes the others on. An answer that is no number
//!   retriggers as in C, and 16 in a row with no sample fail, where C goes on for ever.
//! - An answer is a line. On `tcp-raw` a short read ends a response, where half a number
//!   is a number. `ID?` shows whether the line ends its answers: if so, an answer waits
//!   for its line end as long as a silence may last, and a fragment is no answer. On one
//!   shown to have none, only [`protocol::Bounds::tail`] tells where an answer ends, and
//!   a piece later than that can't be told from an answer.
//! - Bytes with nothing asked mean something is out of step, which `CHAN?` shows only on
//!   a scan. Ahead of each question it looks at the line: what it finds is dropped with
//!   a warning, and the measurement starts again. So one extra answer costs at most one
//!   wrong sample, if it lands where an answer is awaited, and that says so just after.
//!   USBTMC has no such look. It is no check of what a number means: a range taken for
//!   a reading passes, and this catches the answer left over from it.
//! - `TERM` goes at the first capture after each open, where C takes the front for
//!   selected. What `PRESET` does to the terminals is a guess, and a `-C rear` before it
//!   could read the rear and label it `Front`.
//! - Rear terminals get no `SLIST` and `SADV AUTO`. `api.c:385` compares the card's id
//!   with an enum, so C sends them `SLIST ,4294967295`.
//! - A setting of the same quantity with other flags goes out, where `protocol.c:117`
//!   drops it and says OK. `NPLC NAN` does not.
//! - `NPLC?` that doesn't answer leaves the time unknown: a setting fails and says so,
//!   `--get` fails, and a capture goes on with no high resolution digits and a warning.
//!   C keeps the last value, or 0.
//! - Both ends of a capture work: a stop ends it (C's stop frees a list its callback
//!   goes on using, a segfault here), and a refused start is a refusal (C frees that
//!   list twice, an abort). `REV?` that is no number is version `0.0`, as C means it:
//!   its `g_array_free(NULL)` makes a critical, which ends sigrok-cli.
//! - A reopen forgets the quantity, which the `PRESET` may have changed; C remembers it
//!   and may send nothing. No CLI run gets there.
//! - A set during a capture is refused, where C's would land between question and answer
//!   and its limit would take effect. A question given up on, by a stopped capture, an
//!   open or a setting, is waited for ahead of the next, for [`protocol::Bounds::owed`].
//!   An answer later than that meets the look above.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo,
    Driver, Error, Key, Mq, MqFlags, OptionDesc, Packet, PacketSink, Result, Trigger, Unit, Value,
    ValueList,
};
use crate::scpi::{self, Resource, Scpi, ScpiTransport};
use protocol::{Bounds, Card, REAR_TERMINALS, Silence, Silent, Steps, Then};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
const SET: Access = Access { get: false, set: true, list: false };
const GET_SET: Access = Access { get: true, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 4] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::LimitSamples, SET),
    OptionDesc::new(Key::MeasuredQuantity, SET),
    OptionDesc::new(Key::AdcPowerlineCycles, GET_SET),
];

/// How long a stopped capture tries to hand over its End on a full queue.
const END_AFTER_STOP: Duration = Duration::from_millis(500);

/// The `hp-3457a` driver.
pub struct Hp3457aDriver;

/// How long a look at the line takes, ahead of a question.
const LOOK: Duration = Duration::from_millis(1);
/// What the meter says for itself.
const ID: &str = "HP3457A";

/// Without its line end.
fn strip_line_end(mut data: Vec<u8>) -> String {
    if data.last() == Some(&b'\n') {
        data.pop();
    }
    if data.last() == Some(&b'\r') {
        data.pop();
    }
    String::from_utf8_lossy(&data).into_owned()
}

/// What has come of one answer.
struct Answer {
    line_ends: bool,
    tail: Duration,
    data: Vec<u8>,
}

impl Answer {
    fn new(line_ends: bool, bounds: &Bounds) -> Answer {
        Answer { line_ends, tail: bounds.tail, data: Vec::new() }
    }

    /// The answer, or `Error::Timeout` with what has come kept for the next call.
    ///
    /// On a line that ends its answers, what has no line end isn't one yet, whatever
    /// the wait. More than one line is what no single question gets, so it is returned
    /// whole, which is no number. Past [`protocol::LONGEST_ANSWER`] with no line end it
    /// goes the same way. On a line that ends none, what follows within `tail` joins it.
    fn read(&mut self, scpi: &mut Scpi) -> Result<String> {
        let more = scpi.get_data(None)?;
        self.data.extend_from_slice(&more);
        if self.line_ends {
            if !self.data.contains(&b'\n') && self.data.len() <= protocol::LONGEST_ANSWER {
                return Err(Error::Timeout);
            }
        } else {
            let wait = std::mem::replace(&mut scpi.read_timeout, self.tail);
            while !self.data.contains(&b'\n') {
                match scpi.get_data(None) {
                    Ok(more) => self.data.extend_from_slice(&more),
                    Err(Error::Timeout) => break,
                    Err(e) => {
                        scpi.read_timeout = wait;
                        return Err(e);
                    }
                }
            }
            scpi.read_timeout = wait;
        }
        scpi.answered();
        Ok(strip_line_end(std::mem::take(&mut self.data)))
    }
}

/// `command` and its answer, waiting up to `wait`.
fn ask(
    scpi: &mut Scpi,
    command: &str,
    line_ends: bool,
    wait: Duration,
    bounds: &Bounds,
) -> Result<String> {
    scpi.send(command)?;
    let mut answer = Answer::new(line_ends, bounds);
    let began = Instant::now();
    loop {
        let looked = Instant::now();
        match answer.read(scpi) {
            Err(Error::Timeout) if looked.duration_since(began) < wait => {}
            other => return other,
        }
    }
}

/// `ID?` and its answer, and whether the answer ended its line, which says how to read
/// all the others. Part of the id with no end waits for the rest up to `bounds.query`.
/// The whole id or something else with no end has a read's timeout for the end to come.
fn ask_id(scpi: &mut Scpi, bounds: &Bounds) -> Result<(String, bool)> {
    scpi.send("ID?")?;
    let began = Instant::now();
    let mut data = Vec::new();
    loop {
        let looked = Instant::now();
        match scpi.get_data(None) {
            Ok(more) => {
                data.extend_from_slice(&more);
                if data.contains(&b'\n') {
                    scpi.answered();
                    return Ok((strip_line_end(data), true));
                }
            }
            Err(Error::Timeout) => {
                let text = strip_line_end(data.clone());
                let part = text.len() < ID.len() && ID.starts_with(text.as_str());
                if !data.is_empty() && !part {
                    scpi.answered();
                    return Ok((text, false));
                }
                if looked.duration_since(began) >= bounds.query {
                    return Err(Error::Timeout);
                }
            }
            Err(e) => return Err(e),
        }
    }
}

/// `probe_device`.
fn probe(mut scpi: Scpi, bounds: &Bounds) -> Option<Hp3457aDevice> {
    if let Err(e) = scpi.send("END ALWAYS") {
        log::debug!("END ALWAYS: {e}.");
        return None;
    }
    let (id, line_ends) = match ask_id(&mut scpi, bounds) {
        Ok(id) => id,
        Err(Error::Timeout) => {
            log::warn!(
                "No answer to ID? in {} s, so no HP 3457A is taken to be there.",
                bounds.query.as_secs()
            );
            return None;
        }
        Err(e) => {
            log::debug!("ID?: {e}.");
            return None;
        }
    };
    if id != ID {
        log::debug!("'{id}' is no HP 3457A.");
        return None;
    }
    let mut query = |command| ask(&mut scpi, command, line_ends, bounds.query, bounds);
    // Report a version of 0.0 if it can't be read.
    let rev = query("REV?");
    let silent = matches!(rev, Err(Error::Timeout));
    let version = match rev {
        Ok(text) => protocol::revision(&text),
        Err(e) => {
            log::debug!("REV?: {e}.");
            "0.0".to_owned()
        }
    };
    // Its late answer would be taken for this one's.
    let card = if silent {
        log::warn!("No answer to REV?, so no card is looked for.");
        None
    } else {
        match query("OPT?") {
            Ok(text) => protocol::find_card(&text),
            Err(e) => {
                log::warn!("No answer to OPT?, so no card is taken to be there: {e}.");
                None
            }
        }
    };
    if let Some(card) = card {
        log::info!("Found {}.", card.name);
    }
    let resource = scpi.resource().to_owned();
    let _ = scpi.close();
    let mut device = Hp3457aDevice::new(version, resource, card, Some(scpi), *bounds);
    device.line_ends = line_ends;
    Some(device)
}

impl Driver for Hp3457aDriver {
    fn id(&self) -> &'static str {
        "hp-3457a"
    }

    fn long_name(&self) -> &'static str {
        "HP 3457A"
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
        // Commands that are no SCPI go to nothing it wasn't told to go to.
        let Some(conn) = conn else { return Ok(Vec::new()) };
        let bounds = Bounds::default();
        let found = match scpi::scan(Some(&conn), None, |s| Ok(probe(s, &bounds))) {
            Ok(found) => found,
            Err(e) => {
                log::info!("{e}");
                Vec::new()
            }
        };
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// Which terminals are selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Terminals {
    Front,
    Rear,
}

/// The quantity selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Quantity {
    mq: Mq,
    flags: MqFlags,
    unit: Unit,
}

struct Worker {
    stop: Arc<AtomicBool>,
    /// Ends by giving the connection back, and whether a question is unanswered.
    handle: JoinHandle<(Scpi, bool)>,
}

/// One meter.
pub struct Hp3457aDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    card: Option<&'static Card>,
    resource: String,
    /// The connection while it is closed.
    closed: Option<Scpi>,
    /// The open connection, while no capture has it.
    scpi: Option<Scpi>,
    worker: Option<Worker>,
    capturing: Arc<AtomicBool>,
    limit_samples: u64,
    quantity: Option<Quantity>,
    /// What the meter last said of its integration time; `None` when it didn't.
    plc: Option<f32>,
    /// Which were selected since the open; `None` until a capture does.
    terminals: Option<Terminals>,
    /// A question given up on has no answer yet.
    owed: bool,
    /// `ID?` ended its line, so all answers do.
    line_ends: bool,
    bounds: Bounds,
}

impl Hp3457aDevice {
    fn new(
        version: String,
        resource: String,
        card: Option<&'static Card>,
        closed: Option<Scpi>,
        bounds: Bounds,
    ) -> Self {
        let mut channels = vec![Channel::new(0, ChannelKind::Analog, "Front")];
        let mut groups = vec![ChannelGroup::new("Front", vec![0])];
        if let Some(card) = card {
            let mut members = Vec::new();
            for i in 0..card.channels as usize {
                let name = if card.id == REAR_TERMINALS {
                    card.group.to_owned()
                } else {
                    format!("{}{i}", card.group)
                };
                let mut channel = Channel::new(i + 1, ChannelKind::Analog, name);
                channel.enabled = false;
                channels.push(channel);
                members.push(i + 1);
            }
            groups.push(ChannelGroup::new(card.group, members));
        }
        Hp3457aDevice {
            info: DeviceInfo {
                driver: "hp-3457a",
                vendor: "Hewlett-Packard".to_owned(),
                model: "3457A".to_owned(),
                version,
                serial: String::new(),
                conn: None,
            },
            channels,
            groups,
            card,
            resource,
            closed,
            scpi: None,
            worker: None,
            capturing: Arc::new(AtomicBool::new(false)),
            limit_samples: 0,
            quantity: None,
            plc: None,
            terminals: None,
            owed: false,
            line_ends: true,
            bounds,
        }
    }

    /// A device on a transport that stands in for the connection, without a scan.
    /// [`Device::open`] takes it.
    pub fn from_transport(transport: Box<dyn ScpiTransport>, opt: &str) -> Self {
        let scpi = Scpi::new(transport);
        let resource = scpi.resource().to_owned();
        let card = protocol::find_card(opt);
        Self::new("0.0".to_owned(), resource, card, Some(scpi), Bounds::default())
    }

    /// Sets the old worker's stop and joins it, which gets the connection back: an End
    /// stuck on a queue no one reads gives up within its bound.
    fn end_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            match worker.handle.join() {
                Ok((scpi, owed)) => {
                    self.scpi = Some(scpi);
                    self.owed = owed;
                }
                Err(_) => log::error!("The capture thread died with the connection."),
            }
        }
    }

    /// Whether the channels asked for can run together: front or rear, and at least one.
    fn check_channels(&self) -> Result<()> {
        let enabled = || self.channels.iter().filter(|c| c.enabled);
        let front = enabled().any(|c| c.index == 0);
        let rear = enabled().any(|c| c.index != 0);
        if front && rear {
            return Err(Error::arg("Can not use front and rear channels at the same time!"));
        }
        if !front && !rear {
            return Err(Error::arg("Need at least one active channel!"));
        }
        Ok(())
    }

    /// Waits out an answer given up on, for `bounds.owed` at most: it would be read as the
    /// answer to what comes next. One later than that meets a capture's look at the line.
    fn settle_owed(&mut self) {
        if !std::mem::take(&mut self.owed) {
            return;
        }
        let Some(scpi) = self.scpi.as_mut() else { return };
        let mut answer = Answer::new(self.line_ends, &self.bounds);
        let began = Instant::now();
        loop {
            let looked = Instant::now();
            match answer.read(scpi) {
                Ok(text) => {
                    log::debug!("The answer given up on: '{text}'.");
                    return;
                }
                Err(Error::Timeout) if looked.duration_since(began) < self.bounds.owed => {}
                Err(e) => {
                    log::warn!("The answer given up on did not come: {e}.");
                    return;
                }
            }
        }
    }

    /// The connection for a setting, with nothing on its way.
    fn idle(&mut self) -> Result<&mut Scpi> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        self.settle_owed();
        self.scpi.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `hp_3457a_set_mq`, comparing the flags too.
    fn set_quantity(&mut self, mq: Mq, flags: MqFlags) -> Result<()> {
        if self.quantity.is_some_and(|q| q.mq == mq && q.flags == flags) {
            return Ok(());
        }
        let (command, unit) = protocol::mq_command(mq, flags)
            .ok_or_else(|| Error::not_supported("the meter has no such quantity"))?;
        self.idle()?.send(&command)?;
        self.quantity = Some(Quantity { mq, flags, unit });
        Ok(())
    }

    /// `hp_3457a_set_nplc`. A time the meter doesn't then state may or may not have been
    /// taken, so it is no longer known, which fails where C says OK.
    fn set_plc(&mut self, plc: f32) -> Result<()> {
        let command = protocol::plc_command(plc)
            .ok_or_else(|| Error::arg(format!("no integration time of {plc} PLC")))?;
        let (bounds, line_ends) = (self.bounds, self.line_ends);
        let scpi = self.idle()?;
        scpi.send(&command)?;
        let answer = ask(scpi, "NPLC?", line_ends, bounds.setting, &bounds);
        self.owed = matches!(answer, Err(Error::Timeout));
        let number = answer.as_ref().ok().and_then(|text| protocol::parse_number(text));
        self.plc = number.map(|v| v as f32);
        match (number, answer) {
            (Some(_), _) => Ok(()),
            (None, Err(e)) => Err(e),
            (None, Ok(text)) => Err(Error::device(format!(
                "NPLC? says '{text}'; the setting may or may not have been taken"
            ))),
        }
    }
}

impl Device for Hp3457aDevice {
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

    fn options(&self, _group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        match key {
            Key::AdcPowerlineCycles => self
                .plc
                .map(|p| Value::F64(f64::from(p)))
                .ok_or_else(|| Error::device("the meter did not say its integration time")),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            // A capture took its limit at its start.
            Key::LimitSamples if self.capturing.load(Ordering::SeqCst) => {
                return Err(Error::Busy);
            }
            Key::LimitSamples => self.limit_samples = value.as_u64().ok_or_else(bad)?,
            Key::MeasuredQuantity => match value {
                Value::Mq(mq, flags) => return self.set_quantity(*mq, *flags),
                _ => return Err(bad()),
            },
            // A float in C as well, whatever the key's type says.
            Key::AdcPowerlineCycles => {
                let plc = value.as_f64().ok_or_else(bad)?;
                return self.set_plc(plc as f32);
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    fn check_capture(&self, _trigger: Option<&Trigger>) -> Result<()> {
        self.check_channels()
    }

    /// `dev_open`: back to a known state. C drops every result here; a line that
    /// can't take a command is no device to go on with.
    fn open(&mut self) -> Result<()> {
        if self.scpi.is_some() || self.worker.is_some() {
            return Ok(());
        }
        let mut scpi = match self.closed.take() {
            Some(scpi) => scpi,
            None => scpi::open(&Resource::parse(&self.resource)?, None)?,
        };
        scpi.reopen()?;
        let bounds = self.bounds;
        let plc = (|| {
            scpi.send("PRESET")?;
            scpi.send("INBUF ON")?;
            scpi.send("TRIG HOLD")?;
            ask(&mut scpi, "NPLC?", self.line_ends, bounds.query, &bounds)
        })();
        self.owed = matches!(plc, Err(Error::Timeout));
        let plc = match plc {
            Ok(text) => protocol::parse_number(&text).map(|v| v as f32),
            Err(Error::Timeout) => None,
            Err(e) => {
                let _ = scpi.close();
                self.closed = Some(scpi);
                return Err(e);
            }
        };
        if plc.is_none() {
            log::warn!("The meter did not say its integration time: no high resolution digits.");
        }
        self.plc = plc;
        // What a preset does to these isn't known, so nothing is taken for done.
        self.quantity = None;
        self.terminals = None;
        self.scpi = Some(scpi);
        Ok(())
    }

    /// `dev_close`.
    fn close(&mut self) -> Result<()> {
        self.end_worker();
        self.owed = false;
        if let Some(mut scpi) = self.scpi.take() {
            for command in ["SADV HOLD", "TRIG AUTO"] {
                if let Err(e) = scpi.send(command) {
                    log::debug!("{command}: {e}.");
                }
            }
            let _ = scpi.close();
            self.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.end_worker();
        self.check_channels().inspect_err(|e| log::error!("{}", refusal_text(e)))?;
        self.settle_owed();
        let front = self.channels[0].enabled;
        let wanted = if front { Terminals::Front } else { Terminals::Rear };
        let (places, list) = active_channels(&self.channels);
        let card = self.card.filter(|c| !front && c.id != REAR_TERMINALS);
        let Some(mut scpi) = self.scpi.take() else {
            return Err(Error::device("device is not open"));
        };
        let armed = (|| {
            if self.terminals != Some(wanted) {
                scpi.send(if front { "TERM FRONT" } else { "TERM REAR" })?;
            }
            self.terminals = Some(wanted);
            // A scan list only for a card, with C's leading comma.
            if card.is_some() {
                let text: String =
                    list.iter().flat_map(|i| [",".to_owned(), i.to_string()]).collect();
                scpi.send(&format!("SLIST {text}"))?;
                scpi.send("SADV AUTO")?;
            }
            Ok::<(), Error>(())
        })();
        if let Err(e) = armed {
            self.scpi = Some(scpi);
            return Err(e);
        }
        let capture = Capture {
            steps: Steps::new(protocol::high_resolution(self.plc), list, self.limit_samples),
            places,
            quantity: self.quantity,
            silence_limited: self.limit_samples > 0,
            line_ends: self.line_ends,
            bounds: self.bounds,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let capturing = Arc::clone(&self.capturing);
        capturing.store(true, Ordering::SeqCst);
        let spawned = std::thread::Builder::new()
            .name(format!("{} acquisition", self.info.driver))
            .spawn(move || {
                let mut owed = false;
                let outcome = capture.run(&mut scpi, &sink, &stop2, &mut owed);
                capturing.store(false, Ordering::SeqCst);
                send_end(&sink, &stop2, outcome);
                drop(sink);
                (scpi, owed)
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

    /// Asks the thread to end, and does not wait: it may be sending. Nothing goes to
    /// the meter for this; its close sends what there is.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// The words of a refused start, without `Error`'s words around them.
fn refusal_text(e: &Error) -> String {
    match e {
        Error::Arg(text) => text.clone(),
        other => other.to_string(),
    }
}

/// `create_channel_index_list`: the enabled channels' positions, and for each what
/// `CHAN?` says of it: its number on the card. The rear terminals come to 0 here, where
/// C has -1, and their only channel never gets asked.
fn active_channels(channels: &[Channel]) -> (Vec<usize>, Vec<i32>) {
    let places: Vec<usize> = channels.iter().filter(|c| c.enabled).map(|c| c.index).collect();
    let list = places.iter().map(|i| *i as i32 - 1).collect();
    (places, list)
}

/// `End`, waiting for room in the queue for as long as nobody asked to stop, and after
/// a stop only for [`END_AFTER_STOP`]: a close joins this thread.
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

/// With nothing asked nothing should come: a look at the line, ahead of a question. What
/// it finds goes, with a word. Where a transport has no such look, 0.
fn look_for_strays(scpi: &mut Scpi) -> Result<usize> {
    let bytes = scpi.discard(LOOK)?.unwrap_or(0);
    if bytes > 0 {
        log::warn!(
            "The meter sent {bytes} bytes with nothing asked: the answers are out of step, so \
             this measurement starts again."
        );
    }
    Ok(bytes)
}

/// One capture, on its own thread.
struct Capture {
    steps: Steps,
    places: Vec<usize>,
    quantity: Option<Quantity>,
    silence_limited: bool,
    line_ends: bool,
    bounds: Bounds,
}

impl Capture {
    fn packet(&self, sample: protocol::Sample) -> Packet {
        // C sends mq 0 and unit 0 until one is set, which prints no unit. A quantity
        // must be named here, and what a preset selects is DC volts.
        let (mq, flags, unit) = self
            .quantity
            .map_or((Mq::Voltage, MqFlags::empty(), Unit::Unitless), |q| (q.mq, q.flags, q.unit));
        Packet::Analog(AnalogData {
            channels: vec![self.places.get(sample.place).copied().unwrap_or(0)],
            samples: vec![sample.value],
            mq,
            flags,
            unit,
            digits: sample.digits,
        })
    }

    fn run(
        mut self,
        scpi: &mut Scpi,
        sink: &PacketSink,
        stop: &AtomicBool,
        owed: &mut bool,
    ) -> Result<()> {
        let go_on = || !stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        look_for_strays(scpi)?;
        scpi.send("TRIG SGL")?;
        *owed = true;
        let wait = scpi.read_timeout;
        let mut silence = Silence::new(self.bounds, self.silence_limited, Instant::now());
        let mut answer = Answer::new(self.line_ends, &self.bounds);
        loop {
            if !go_on() {
                return Ok(());
            }
            // Stamped ahead of the read: what an empty read says holds from here.
            let looked = Instant::now();
            let text = match answer.read(scpi) {
                Ok(text) => text,
                Err(Error::Timeout) => {
                    match silence.empty_read(looked, wait) {
                        Silent::Wait => {}
                        Silent::Warn(message) => log::warn!("{message}"),
                        Silent::GiveUp(message) => {
                            log::error!("{message}");
                            return Err(Error::device(message));
                        }
                    }
                    continue;
                }
                Err(e) => return Err(e),
            };
            *owed = false;
            let (sample, then) = self.steps.answer(&text);
            if let Some(message) = silence.heard(Instant::now(), sample.is_some()) {
                log::warn!("{message}");
            }
            if let Some(sample) = sample
                && !sink.send_while(self.packet(sample), go_on)
            {
                return Ok(());
            }
            // Only with a question next: what comes after a limit is nobody's.
            let then = match then {
                Then::Ask(_) if !go_on() => return Ok(()),
                Then::Ask(command) => match look_for_strays(scpi)? {
                    0 => Then::Ask(command),
                    bytes => self.steps.stray(bytes),
                },
                other => other,
            };
            match then {
                Then::Done => return Ok(()),
                Then::Ask(command) => {
                    scpi.send(command)?;
                    *owed = true;
                    silence.asked(Instant::now());
                }
                Then::Fail(lines) => {
                    for line in &lines {
                        log::error!("{line}");
                    }
                    let message = lines.first().cloned().unwrap_or_default();
                    return Err(Error::device(message));
                }
            }
        }
    }
}
