// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-dmm/api.c,
//   libsigrok/src/hardware/scpi-dmm/protocol.c,
//   Copyright (C) 2018 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Timo Kokkonen, Petteri Aimonen, Adrian Godwin,
//   Electro707, Frank Stettner, Mike Walters, Mike Williams, Sahaj Sarup,
//   Sylvain Pelissier
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! SCPI bench multimeters: Agilent's 34405A, 34410A and 34460A, GW Instek's GDM8251A,
//! GDM8255A, GDM9060 and GDM9061, HP's 34401A, Keysight's 34465A, OWON's XDM1041 and
//! XDM2041, and Siglent's SDM3055, by the tables of [`protocol`], on anything [`scpi`]
//! has. No GPIB, VXI or VISA here.
//!
//! A scan asks `*IDN?`, an OWON `*OPC?` once to see if it has it, sends the local
//! command where the model has one, and closes. With no `conn=` only USBTMC devices are
//! probed: a meter on a USB serial port, as a GDM9061 in its USB-CDC mode (2184:0058, in
//! C's `scpi_serial_usb_ids`), needs `conn=/dev/ttyACM0`. An open sends nothing. Each query and
//! setting has `*OPC?` first, unless the scan found none.
//!
//! A capture runs C's rounds, 10 ms apart: `*OPC?`, the function, `*OPC?`, a value, one
//! sample a round, and the limits looked at only after a round. So `--time 1` still has
//! a reading. A close sends `*OPC?` and the local command, after `--show`, `--get` and
//! `--set` too, where C's `dev_close` returns at `sdi->status <= SR_ST_INACTIVE`
//! (`api.c:453`) as `sr_dev_close` has just set that (`device.c:779`), and leaves the
//! meter's panel locked, against its comment and commit 118da66f.
//!
//! Where it goes its own way, see `docs/DEVICES.md`:
//!
//! - A silent query in a capture: C ends the capture at the first read's timeout, 1 s
//!   to 2.5 s, with exit 0, though a slow reading is still to come; over TCP its read
//!   never times out and it stays for ever. Here it keeps waiting ([`protocol::Silence`]):
//!   it reads on for the answer to the one question out, on USBTMC too, where it reads
//!   again for the request already made (`Scpi::wait_out_requests`): a second request
//!   while one is out gets a real meter's old answer under the wrong tag, "USBTMC invalid
//!   bulk in header" in the 34405A's and 34465A's logs. A sample limit alone fails at 30 s with no reading
//!   yet, and at 10 minutes after one, with a warning at 30 s. A time limit ends it at
//!   that limit, with a warning and exit 0. With neither it goes on until it is stopped.
//! - An answer that makes no reading ends a capture, as in C, and is a failure, but
//!   under a time limit with nothing captured a warning and exit 0. An empty answer is
//!   no reading where C makes 0 of it.
//! - A function answer the table doesn't know, in a round, is taken for a line out of
//!   step: the round starts again once the line is quiet, [`RESYNCS`] times in a row at
//!   most, and then the capture fails, where C fails it at the first.
//! - A range is a number or `AUTO`, `MIN`, `MAX` or `DEF`, in any case; anything else is
//!   refused before the wire, where C sends it whole, a `;` and what follows included.
//! - `*OPC?` silent goes on after 30 s, where C takes 100 tries, 100 s or more; at a
//!   stop or close after one read's timeout, so that a dead meter ends. A protocol error
//!   in it counts as a try, as in C.
//! - An answer over 4 KiB is a failed read, as C's has no bound.
//! - Readings are `f32` in [`crate::core::AnalogData`], where C sends doubles at 6
//!   digits and more on the Agilent reader. C's output modules take a float of them too.
//! - An instrument's refusal of a setting isn't seen, as in C: nothing reads the error
//!   queue, a `--set` exits 0 and the next `--get` says what it really is. A failed write
//!   is an error, and may or may not have taken effect.

#![forbid(unsafe_code)]

pub mod protocol;
#[cfg(test)]
mod tests;

use crate::core::{
    Access, AnalogData, Channel, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error, Key,
    OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::scpi::{self, CmdArg, Scpi};
use protocol::{Cmd, Limits, Model, MqOpt, Reader, Silence, Verdict};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::Multimeter];
static OPTIONS: [OptionDesc; 5] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::MeasuredQuantity, Access::GET_SET_LIST),
];
static OPTIONS_RANGE: [OptionDesc; 6] = [
    OptionDesc::new(Key::Continuous, Access::NONE),
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::LimitSamples, Access::GET_SET),
    OptionDesc::new(Key::LimitMsec, Access::GET_SET),
    OptionDesc::new(Key::MeasuredQuantity, Access::GET_SET_LIST),
    OptionDesc::new(Key::Range, Access::GET_SET_LIST),
];

/// `sr_scpi_source_add`'s 10 ms: how long a round waits after the last.
const ROUND: Duration = Duration::from_millis(10);
/// How often `*OPC?` is asked or fails before going on: `SCPI_READ_RETRIES`.
const OPC_TRIES: u32 = 100;
/// `SCPI_READ_RETRY_TIMEOUT_US`.
const OPC_PAUSE: Duration = Duration::from_millis(10);
/// How long `*OPC?` may be silent before what comes next goes ahead: a choice, against
/// C's 100 tries of a read's timeout each.
const OPC_SILENCE: Duration = Duration::from_secs(30);
/// How much of an answer is read: more is no answer of a meter's.
const ANSWER_MAX: usize = 4096;
/// How long a worker that was told to stop tries to hand over its `End`.
const END_AFTER_STOP: Duration = Duration::from_millis(500);
/// How many rounds in a row may start again on a function answer the table doesn't know.
pub const RESYNCS: u32 = 3;

/// The `scpi-dmm` driver.
pub struct ScpiDmmDriver;

impl Driver for ScpiDmmDriver {
    fn id(&self) -> &'static str {
        "scpi-dmm"
    }

    fn long_name(&self) -> &'static str {
        "SCPI DMM"
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
        // What fails here is no device, as `sr_scpi_scan_resource` has it.
        let found = match scpi::scan(conn.as_deref(), serialcomm.as_deref(), |s| Ok(probe(s))) {
            Ok(found) => found,
            Err(e) => {
                log::info!("{e}");
                Vec::new()
            }
        };
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// `probe_device`, and the close `sr_scpi_scan_resource` adds.
fn probe(mut scpi: Scpi) -> Option<ScpiDmmDevice> {
    let found = identify(&mut scpi).map(|(id, model, no_opc)| {
        if let Some(timeout) = model.read_timeout {
            scpi.read_timeout = timeout;
        }
        let link = Link::new(model, no_opc);
        if let Some(local) = model.command(Cmd::SetupLocal) {
            link.delay(&mut scpi, &|| true, OPC_SILENCE);
            if let Err(e) = scpi.send(local) {
                log::debug!("{local}: {e}");
            }
        }
        (id, model, link)
    });
    let _ = scpi.close();
    found.map(|(id, model, link)| ScpiDmmDevice::new(&id, model, link, scpi))
}

/// Whose it is: `*IDN?` and, on a model that needs it, whether it has `*OPC?`.
fn identify(scpi: &mut Scpi) -> Option<(scpi::HwInfo, &'static Model, bool)> {
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::debug!("*IDN?: {e}");
            log::info!("Could not get IDN response.");
            return None;
        }
    };
    let model = protocol::find_model(scpi::vendor_alias(&id.manufacturer), &id.model)?;
    // On USBTMC a request left out would take the next question's answer: cleared.
    let no_opc = model.check_opc && asked(scpi, |s| s.get_data(Some("*OPC?"))).is_err();
    Some((id, model, no_opc))
}

/// What ended a query in a capture.
#[derive(Debug)]
enum Halt {
    /// Told to stop.
    Stopped,
    /// The time limit, with nothing heard: a word and no failure.
    Ended(String),
    /// A read or write that ends the capture.
    Failed(Error),
    /// A failure by a silence bound or by what an answer says, which a time limit with
    /// nothing captured turns to a word: the device is there.
    NoReading(Error),
    /// A function answer the table doesn't know: the round starts again, as the line
    /// may be out of step.
    OutOfStep(Error),
}

/// What ends a capture's waits: a stop and the rule on silence.
struct Watch<'a> {
    stop: &'a AtomicBool,
    limits: &'a SharedLimits,
    started: Instant,
    silence: Silence,
}

impl Watch<'_> {
    fn go_on(&self) -> bool {
        !self.stop.load(Ordering::SeqCst)
    }

    fn limits(&self) -> Limits {
        let msec = self.limits.msec.load(Ordering::SeqCst);
        Limits {
            only_samples: self.limits.samples.load(Ordering::SeqCst) > 0 && msec == 0,
            time_up: self.time_up(msec),
        }
    }

    fn time_up(&self, msec: u64) -> bool {
        msec > 0 && self.started.elapsed() >= Duration::from_millis(msec)
    }

    /// A read that began at `at` found nothing. Ok to wait on.
    fn silent(&mut self, at: Instant) -> std::result::Result<(), Halt> {
        match self.silence.silent(at, self.limits()) {
            Verdict::Wait => Ok(()),
            Verdict::Warn(text) => {
                log::warn!("{text}");
                Ok(())
            }
            Verdict::End(text) => Err(Halt::Ended(text)),
            Verdict::Fail(text) => Err(Halt::NoReading(Error::device(text))),
        }
    }
}

/// The limits, which a client may set on a running capture.
#[derive(Default)]
struct SharedLimits {
    samples: AtomicU64,
    msec: AtomicU64,
}

/// A reading, with what it says of itself.
#[derive(Clone, Debug)]
pub struct Reading {
    /// In the unit.
    pub value: f64,
    /// The table's line.
    pub item: &'static MqOpt,
    /// Volts, ohms and so on.
    pub unit: crate::core::Unit,
    /// Decimal digits.
    pub digits: i32,
}

/// What a device keeps of the conversation, and the meter while it is open.
struct Link {
    model: &'static Model,
    scpi: Option<Scpi>,
    /// `no_opc_command`: the scan found it has no `*OPC?`.
    no_opc: bool,
    /// What the precision query said at the start of a capture.
    precision: Option<String>,
}

impl Link {
    fn new(model: &'static Model, no_opc: bool) -> Link {
        Link { model, scpi: None, no_opc, precision: None }
    }

    /// The open meter.
    fn meter(&mut self) -> Result<&mut Scpi> {
        self.scpi.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `scpi_dmm_cmd_delay`, whose result C drops: `*OPC?` up to [`OPC_TRIES`] times.
    /// `silence` is how long it may go unanswered, and a protocol error costs a try:
    /// a real 34405A goes through bad headers and pipe errors here after `INIT`
    /// (e90551c3), and comes back.
    fn delay(&self, scpi: &mut Scpi, go_on: &dyn Fn() -> bool, silence: Duration) {
        if self.no_opc {
            return;
        }
        let mut tries = 0;
        let mut heard = Instant::now();
        let mut asked = false;
        let mut keep = Vec::new();
        while go_on() && tries < OPC_TRIES {
            if !asked {
                keep.clear();
                if let Err(e) = scpi.send("*OPC?") {
                    if is_gone(&e) {
                        return;
                    }
                    log::debug!("*OPC?: {e}");
                    tries += 1;
                    std::thread::sleep(OPC_PAUSE);
                    continue;
                }
            }
            let began = Instant::now();
            match read_answer(scpi, &mut keep, go_on) {
                Ok(true) => {
                    let text = take_answer(scpi, &mut keep);
                    if scpi::parse_bool(&text) == Some(true) {
                        return;
                    }
                    tries += 1;
                    heard = Instant::now();
                    asked = false;
                    std::thread::sleep(OPC_PAUSE);
                }
                Ok(false) => {
                    give_up(scpi);
                    return;
                }
                Err(Error::Timeout) => {
                    if began.saturating_duration_since(heard) >= silence {
                        log::debug!("No answer to *OPC? in {:.1} s.", silence.as_secs_f32());
                        give_up(scpi);
                        return;
                    }
                    // One out at a time: on USBTMC the open's `wait_out_requests` reads
                    // again for the request already made.
                    asked = true;
                }
                Err(e) if is_gone(&e) => return,
                Err(e) => {
                    log::debug!("*OPC?: {e}");
                    tries += 1;
                    asked = false;
                    std::thread::sleep(OPC_PAUSE);
                }
            }
        }
        // Told to stop with a question out.
        if asked {
            give_up(scpi);
        }
    }

    /// [`Link::delay`] for what a client asks, with no stop to look at.
    fn plain_delay(&mut self) -> Result<()> {
        let mut scpi = self.scpi.take().ok_or_else(|| Error::device("device is not open"))?;
        self.delay(&mut scpi, &|| true, OPC_SILENCE);
        self.scpi = Some(scpi);
        Ok(())
    }

    /// `scpi_dmm_get_mq`: what the meter is set to, and what it said of it.
    fn get_mq(&mut self) -> Result<(String, &'static MqOpt)> {
        self.plain_delay()?;
        let query = self.model.command(Cmd::QueryFunc).ok_or_else(no_command)?;
        let answer = asked(self.meter()?, |s| s.get_string(query))?;
        self.function_item(answer)
            .map_err(|text| Error::not_supported(format!("Unknown measurement quantity: {text}")))
    }

    /// The table's line of a function query's answer, or what it said instead.
    fn function_item(
        &self,
        answer: String,
    ) -> std::result::Result<(String, &'static MqOpt), String> {
        if answer.is_empty() {
            return Err(answer);
        }
        let have = protocol::function_text(&answer);
        let Some(item) = self.model.by_text(have) else {
            log::warn!("Unknown measurement quantity: {have}");
            return Err(have.to_owned());
        };
        Ok((answer, item))
    }

    /// `scpi_dmm_set_mq`.
    fn set_mq(&mut self, item: &'static MqOpt) -> Result<()> {
        self.plain_delay()?;
        let table = self.model.commands;
        let delay = self.model.conf_delay;
        self.meter()?.cmd(table, None, &Cmd::SetupFunc, &[CmdArg::Str(item.setup)])?;
        if item.conf_delay {
            std::thread::sleep(delay);
        }
        Ok(())
    }

    /// `scpi_dmm_get_range_text`: `None` where C has no text.
    fn get_range(&mut self) -> Option<String> {
        let (_, item) = self.get_mq().ok()?;
        if item.no_range {
            return None;
        }
        let table = self.model.commands;
        self.plain_delay().ok()?;
        let auto = {
            let scpi = self.meter().ok()?;
            scpi.cmd(table, None, &Cmd::QueryRangeAuto, &[CmdArg::Str(item.setup)]).ok()?;
            answer_text(&asked(scpi, |s| s.get_data(None)).ok()?)
        };
        if protocol::strict_bool(&auto)? {
            return Some("auto".to_owned());
        }
        self.plain_delay().ok()?;
        let scpi = self.meter().ok()?;
        scpi.cmd(table, None, &Cmd::QueryRange, &[CmdArg::Str(item.setup)]).ok()?;
        protocol::range_text(&answer_text(&asked(scpi, |s| s.get_data(None)).ok()?))
    }

    /// `scpi_dmm_set_range_from_text`.
    fn set_range(&mut self, range: &str) -> Result<()> {
        if !protocol::range_word_ok(range) {
            return Err(Error::arg(format!(
                "a range is a number or AUTO, MIN, MAX or DEF, not '{}'",
                range.escape_debug()
            )));
        }
        let (_, item) = self.get_mq()?;
        if item.no_range {
            return Err(Error::not_supported(format!("{} has no range", item.setup)));
        }
        let word = if range.eq_ignore_ascii_case("auto") { "AUTO" } else { range };
        self.plain_delay()?;
        let table = self.model.commands;
        let delay = self.model.conf_delay;
        self.meter()?.cmd(
            table,
            None,
            &Cmd::SetupRange,
            &[CmdArg::Str(item.setup), CmdArg::Str(word)],
        )?;
        if item.conf_delay {
            std::thread::sleep(delay);
        }
        Ok(())
    }

    /// `dev_acquisition_start`, up to the header. Errors are C's: the function query's,
    /// and the start command's.
    fn start_acquisition(&mut self) -> Result<()> {
        let (_, item) = self.get_mq()?;
        if let Some(query) = self.model.command(Cmd::QueryPrec) {
            self.plain_delay()?;
            match asked(self.meter()?, |s| s.get_string(query)) {
                Ok(text) => {
                    let text = text.trim_matches(protocol::c_space).to_owned();
                    log::debug!("Precision: '{text}'");
                    self.precision = Some(text);
                }
                Err(e) => log::info!("Precision query ('{query}') failed: {e}"),
            }
        }
        if let Some(start) = self.model.command(Cmd::StartAcq) {
            self.plain_delay()?;
            self.meter()?.send(start)?;
        }
        if item.meas_delay && !self.model.meas_delay.is_zero() {
            std::thread::sleep(self.model.meas_delay);
        }
        Ok(())
    }

    /// `dev_acquisition_stop`. Nothing it meets ends anything, and `*OPC?` gets one
    /// read's timeout: a meter that doesn't say must not hold a stop for long.
    fn stop_acquisition(&mut self) {
        self.precision = None;
        let model = self.model;
        if let Some(stop) = model.command(Cmd::StopAcq) {
            let Some(mut scpi) = self.scpi.take() else { return };
            let silence = scpi.read_timeout;
            self.delay(&mut scpi, &|| true, silence);
            if let Err(e) = scpi.send(stop) {
                log::debug!("{stop}: {e}");
            }
            self.scpi = Some(scpi);
        }
    }

    /// `dev_close` of a device still active: `*OPC?` and the local command.
    fn say_local(&mut self) {
        let model = self.model;
        if let Some(local) = model.command(Cmd::SetupLocal)
            && let Some(mut scpi) = self.scpi.take()
        {
            let silence = scpi.read_timeout;
            self.delay(&mut scpi, &|| true, silence);
            if let Err(e) = scpi.send(local) {
                log::debug!("{local}: {e}");
            }
            self.scpi = Some(scpi);
        }
    }

    /// A query in a capture, waiting as [`Watch`] says, reading on for its one answer.
    fn ask(scpi: &mut Scpi, query: &str, watch: &mut Watch<'_>) -> Result2<String> {
        scpi.send(query).map_err(Halt::Failed)?;
        let mut keep = Vec::new();
        loop {
            let began = Instant::now();
            let stop = watch.stop;
            match read_answer(scpi, &mut keep, &|| !stop.load(Ordering::SeqCst)) {
                Ok(true) => {
                    return Ok(take_answer(scpi, &mut keep));
                }
                Ok(false) => {
                    give_up(scpi);
                    return Err(Halt::Stopped);
                }
                Err(Error::Timeout) => {
                    if let Err(halt) = watch.silent(began) {
                        give_up(scpi);
                        return Err(halt);
                    }
                    if !watch.go_on() {
                        give_up(scpi);
                        return Err(Halt::Stopped);
                    }
                }
                Err(e) => return Err(Halt::Failed(e)),
            }
        }
    }

    /// `*OPC?` in a round. A time limit that has come stops it after a read's timeout,
    /// and says so: a limit never cuts a round that gets its answers.
    fn round_delay(&self, scpi: &mut Scpi, watch: &mut Watch<'_>) -> Result2<()> {
        let began = Instant::now();
        let patience = scpi.read_timeout;
        let gave_up = std::cell::Cell::new(false);
        {
            let (stop, limits, started) = (watch.stop, watch.limits, watch.started);
            let go_on = || {
                if stop.load(Ordering::SeqCst) {
                    return false;
                }
                let msec = limits.msec.load(Ordering::SeqCst);
                let late = msec > 0
                    && started.elapsed() >= Duration::from_millis(msec)
                    && began.elapsed() >= patience;
                if late {
                    gave_up.set(true);
                }
                !late
            };
            self.delay(scpi, &go_on, OPC_SILENCE);
        }
        if !watch.go_on() {
            return Err(Halt::Stopped);
        }
        if gave_up.get() {
            watch.silent(began)?;
        }
        Ok(())
    }

    /// A round of `scpi_dmm_receive_data`, on the model's reader.
    fn measure(&mut self, watch: &mut Watch<'_>) -> Result2<Reading> {
        let model = self.model;
        let mut scpi =
            self.scpi.take().ok_or_else(|| Halt::Failed(Error::device("device is not open")))?;
        let reading = self.read_one(&mut scpi, watch, model);
        self.scpi = Some(scpi);
        reading
    }

    fn read_one(
        &self,
        scpi: &mut Scpi,
        watch: &mut Watch<'_>,
        model: &'static Model,
    ) -> Result2<Reading> {
        let question = |cmd| model.command(cmd).ok_or_else(|| Halt::Failed(no_command()));
        self.round_delay(scpi, watch)?;
        self.round_delay(scpi, watch)?;
        let answer = Self::ask(scpi, question(Cmd::QueryFunc)?, watch)?;
        let (mode, item) = self.function_item(answer).map_err(|text| {
            // What follows on the line may be one answer behind: the next send drains it.
            scpi.forget_question();
            Halt::OutOfStep(Error::device(format!("Unknown measurement quantity: '{text}'")))
        })?;
        let (digits, code) = match model.reader {
            Reader::Agilent => {
                let exponent = protocol::precision_exponent(&mode, item).ok_or_else(|| {
                    Halt::NoReading(Error::device(format!("No usable precision in '{mode}'")))
                })?;
                (protocol::digits_of_exponent(exponent), 0)
            }
            Reader::Gwinstek => {
                self.round_delay(scpi, watch)?;
                (protocol::gw_digits(model, self.precision.as_deref()), protocol::mode_code(&mode))
            }
        };
        self.round_delay(scpi, watch)?;
        let text = Self::ask(scpi, question(Cmd::QueryValue)?, watch)?;
        let (limit, at_limit) = protocol::reading_limit(model);
        let value = protocol::parse_reading(&text, limit, at_limit).map_err(|e| {
            // What made no reading may stand for the real one still to come: the stop's
            // first send waits for the line to go quiet.
            scpi.forget_question();
            Halt::NoReading(Error::device(e.to_string()))
        })?;
        let value = match model.reader {
            Reader::Agilent => value,
            Reader::Gwinstek => protocol::gw_overload(code, model.name, value),
        };
        Ok(Reading { value, item, unit: protocol::unit_of(item.mq, code), digits })
    }
}

type Result2<T> = std::result::Result<T, Halt>;

fn no_command() -> Error {
    Error::not_supported("the meter has no such command")
}

/// A question given up on: on USBTMC a device clear, as a real meter would answer the
/// next request with the old one's answer under the old tag; elsewhere the next send
/// drains what may still come.
fn give_up(scpi: &mut Scpi) {
    if scpi.tags_answers() {
        if let Err(e) = scpi.clear() {
            log::debug!("USBTMC clear: {e}");
        }
    } else {
        scpi.forget_question();
    }
}

/// What `ask` gets, with the question given up on if its answer never came.
fn asked<T>(scpi: &mut Scpi, ask: impl FnOnce(&mut Scpi) -> Result<T>) -> Result<T> {
    let answer = ask(scpi);
    if matches!(answer, Err(Error::Timeout)) && scpi.tags_answers() {
        give_up(scpi);
    }
    answer
}

/// A connection or a device that is no more.
fn is_gone(e: &Error) -> bool {
    matches!(e, Error::Disconnected | Error::Io(_))
}

/// The answer to the one question out, read whole: its first line. What came after it
/// in the same read is no answer to anything asked, and the next send waits for the
/// line to go quiet; a socket's read can hold two answers where a meter sent one twice.
fn take_answer(scpi: &mut Scpi, keep: &mut Vec<u8>) -> String {
    scpi.answered();
    if let Some(end) = keep.iter().position(|&b| b == b'\n')
        && end + 1 < keep.len()
    {
        log::debug!(
            "{} bytes after an answer: taken for the line out of step.",
            keep.len() - end - 1
        );
        keep.truncate(end + 1);
        scpi.forget_question();
    }
    answer_text(keep)
}

/// An answer's text, with its line end off: `sr_scpi_get_string`.
fn answer_text(data: &[u8]) -> String {
    let mut data = data;
    if let [rest @ .., b'\n'] = data {
        data = rest;
    }
    if let [rest @ .., b'\r'] = data {
        data = rest;
    }
    String::from_utf8_lossy(data).into_owned()
}

/// One read of an answer into `keep`, in pieces so that a stop gets through and a
/// stream with no end does not. True when it is whole, false when `go_on` said no.
fn read_answer(scpi: &mut Scpi, keep: &mut Vec<u8>, go_on: &dyn Fn() -> bool) -> Result<bool> {
    let mut chunk = [0u8; 512];
    scpi.read_begin()?;
    let mut deadline = Instant::now() + scpi.read_timeout;
    while !scpi.read_complete() {
        if !go_on() {
            return Ok(false);
        }
        let n = scpi.read_data(&mut chunk)?;
        if n == 0 {
            // A transport can end an answer with a read of nothing: tcp-raw ends one
            // with no line end at a whole read's silence.
            if scpi.read_complete() && !keep.is_empty() {
                break;
            }
            // A signal, as a stall's, cuts a read short: only the time says silence.
            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }
            continue;
        }
        deadline = Instant::now() + scpi.read_timeout;
        keep.extend_from_slice(&chunk[..n]);
        if keep.len() > ANSWER_MAX {
            return Err(Error::device(format!(
                "An answer of more than {ANSWER_MAX} bytes: no meter's."
            )));
        }
    }
    Ok(true)
}

/// One meter.
pub struct ScpiDmmDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    model: &'static Model,
    limits: Arc<SharedLimits>,
    enabled: Arc<AtomicBool>,
    /// The instrument between a close and an open.
    closed: Option<Scpi>,
    link: Arc<Mutex<Link>>,
    capturing: Arc<AtomicBool>,
    /// [`protocol::NO_READING`] and [`protocol::GIVE_UP`].
    silence: (Duration, Duration),
    worker: Option<Worker>,
}

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl ScpiDmmDevice {
    fn new(id: &scpi::HwInfo, model: &'static Model, link: Link, scpi: Scpi) -> Self {
        ScpiDmmDevice {
            info: DeviceInfo {
                driver: "scpi-dmm",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: Some(scpi.resource().to_owned()),
            },
            channels: vec![Channel::new(0, ChannelKind::Analog, "P1".to_owned())],
            model,
            limits: Arc::new(SharedLimits::default()),
            enabled: Arc::new(AtomicBool::new(true)),
            closed: Some(scpi),
            link: Arc::new(Mutex::new(link)),
            capturing: Arc::new(AtomicBool::new(false)),
            silence: (protocol::NO_READING, protocol::GIVE_UP),
            worker: None,
        }
    }

    /// A device a scan of `scpi` found, for tests and clients with a transport of their
    /// own. [`Device::open`] takes it.
    ///
    /// # Errors
    ///
    /// When it is none of the table's, or doesn't answer.
    pub fn from_scpi(scpi: Scpi) -> Result<Self> {
        probe(scpi).ok_or_else(|| Error::device("Not a meter of this driver's table."))
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    /// Stops a worker and waits for it. A close and a restart need it gone.
    fn join_worker(&mut self) {
        if let Some(worker) = self.worker.take() {
            worker.stop.store(true, Ordering::SeqCst);
            let _ = worker.handle.join();
        }
    }
}

fn no_group(group: Option<&str>) -> Result<()> {
    match group {
        Some(g) => Err(Error::ChannelGroup(g.to_owned())),
        None => Ok(()),
    }
}

impl Device for ScpiDmmDevice {
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
        self.enabled.store(enabled, Ordering::SeqCst);
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        no_group(group)?;
        Ok(if self.model.ranged { OPTIONS_RANGE.to_vec() } else { OPTIONS.to_vec() })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        no_group(group)?;
        Ok(match key {
            Key::Conn => {
                let conn = self.info.conn.clone();
                Value::Str(conn.ok_or_else(|| Error::not_supported("no connection id"))?)
            }
            Key::LimitSamples => Value::U64(self.limits.samples.load(Ordering::SeqCst)),
            Key::LimitMsec => Value::U64(self.limits.msec.load(Ordering::SeqCst)),
            Key::MeasuredQuantity => {
                let (_, item) = self.link().get_mq()?;
                Value::Mq(item.mq, item.flags())
            }
            Key::Range if self.model.ranged => match self.link().get_range() {
                Some(text) => Value::Str(text),
                None => return Err(Error::not_supported("No range to read.")),
            },
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        no_group(group)?;
        let bad = || Error::Arg(format!("invalid value for {}", key.id()));
        match key {
            Key::LimitSamples => {
                self.limits.samples.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
            }
            Key::LimitMsec => {
                self.limits.msec.store(value.as_u64().ok_or_else(bad)?, Ordering::SeqCst);
            }
            Key::MeasuredQuantity => {
                let Value::Mq(mq, flags) = value else { return Err(bad()) };
                let item = self
                    .model
                    .by_number(*mq, *flags)
                    .ok_or_else(|| Error::not_supported("the meter has no such function"))?;
                self.link().set_mq(item)?;
            }
            Key::Range if self.model.ranged => {
                self.link().set_range(value.as_str().ok_or_else(bad)?)?;
            }
            other => return Err(Error::UnknownKey(other.id().to_owned())),
        }
        Ok(())
    }

    /// C's models all leave `get_range_text_list` out, so a range has no list.
    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        no_group(group)?;
        match key {
            Key::MeasuredQuantity => {
                Ok(ValueList::Mq(self.model.mqopts.iter().map(|o| (o.mq, o.flags())).collect()))
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `sr_scpi_open`: what a connection does at its open, and no more.
    fn open(&mut self) -> Result<()> {
        let shared = Arc::clone(&self.link);
        let mut link = shared.lock().expect("link lock");
        if link.scpi.is_some() {
            return Ok(());
        }
        let Some(mut scpi) = self.closed.take() else {
            return Err(Error::device("Failed to open SCPI device: it has gone."));
        };
        if let Err(e) = scpi.reopen() {
            self.closed = Some(scpi);
            return Err(Error::device(format!("Failed to open SCPI device: {e}.")));
        }
        // One request out at a time on USBTMC, read again after a timeout.
        scpi.wait_out_requests(true);
        link.scpi = Some(scpi);
        Ok(())
    }

    /// `*OPC?` and the local command where the model has one, then the connection: see
    /// the module for where this leaves C.
    fn close(&mut self) -> Result<()> {
        self.join_worker();
        self.link().say_local();
        let taken = self.link().scpi.take();
        if let Some(mut scpi) = taken {
            let _ = scpi.close();
            self.closed = Some(scpi);
        }
        Ok(())
    }

    fn start(&mut self, sink: PacketSink) -> Result<()> {
        if self.capturing.load(Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        // One that has said End, or can't: it has given all back.
        self.join_worker();
        {
            let mut link = self.link();
            if link.scpi.is_none() {
                return Err(Error::device("device is not open"));
            }
            if let Err(e) = link.start_acquisition() {
                // A session stops a device whose start failed (session.c:831).
                link.stop_acquisition();
                return Err(e);
            }
        }
        self.capturing.store(true, Ordering::SeqCst);
        let stop = Arc::new(AtomicBool::new(false));
        let capture = Capture {
            link: Arc::clone(&self.link),
            limits: Arc::clone(&self.limits),
            enabled: Arc::clone(&self.enabled),
            stop: Arc::clone(&stop),
            started: Instant::now(),
            silence: self.silence,
        };
        let capturing = Arc::clone(&self.capturing);
        let stop2 = Arc::clone(&stop);
        let spawned =
            std::thread::Builder::new().name("scpi-dmm acquisition".to_owned()).spawn(move || {
                let outcome = capture.run(&sink);
                capture.link.lock().expect("link lock").stop_acquisition();
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

    /// Asks the thread to end, and does not wait: it may be sending.
    fn stop(&mut self) -> Result<()> {
        if let Some(worker) = &self.worker {
            worker.stop.store(true, Ordering::SeqCst);
        }
        Ok(())
    }
}

/// `End`, waiting for room while nobody has said stop, and for a moment after.
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

/// One capture: the rounds of `scpi_dmm_receive_data`.
struct Capture {
    link: Arc<Mutex<Link>>,
    limits: Arc<SharedLimits>,
    enabled: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    started: Instant,
    silence: (Duration, Duration),
}

impl Capture {
    fn run(&self, sink: &PacketSink) -> Result<()> {
        let go_on = || !self.stop.load(Ordering::SeqCst);
        if !sink.send_while(Packet::header(), go_on) {
            return Ok(());
        }
        let mut silence = Silence::new(self.started);
        (silence.no_reading, silence.give_up) = self.silence;
        let mut watch =
            Watch { stop: &self.stop, limits: &self.limits, started: self.started, silence };
        let mut samples = 0u64;
        let mut resyncs = 0;
        loop {
            // The source's timeout, in which a client's get or set finds the lock free.
            let wait = Instant::now();
            while go_on() && wait.elapsed() < ROUND {
                std::thread::sleep(Duration::from_millis(2));
            }
            if !go_on() {
                return Ok(());
            }
            if self.enabled.load(Ordering::SeqCst) {
                let round = self.link.lock().expect("link lock").measure(&mut watch);
                let reading = match round {
                    Ok(reading) => reading,
                    Err(Halt::Stopped) => return Ok(()),
                    Err(Halt::Ended(text)) => {
                        log::warn!("{text}");
                        return Ok(());
                    }
                    Err(Halt::OutOfStep(e)) if resyncs < RESYNCS => {
                        resyncs += 1;
                        log::warn!("{e}: the round starts again.");
                        continue;
                    }
                    Err(Halt::NoReading(e) | Halt::OutOfStep(e))
                        if samples == 0 && self.limits.msec.load(Ordering::SeqCst) > 0 =>
                    {
                        log::warn!("No reading in this capture: {e}");
                        return Ok(());
                    }
                    Err(Halt::NoReading(e) | Halt::OutOfStep(e) | Halt::Failed(e)) => {
                        log::error!("Acquisition failed: {e}");
                        return Err(e);
                    }
                };
                let packet = Packet::Analog(AnalogData {
                    channels: vec![0],
                    // C's output modules take a float of a double too.
                    samples: vec![reading.value as f32],
                    mq: reading.item.mq,
                    flags: reading.item.flags(),
                    unit: reading.unit,
                    digits: reading.digits,
                });
                if !sink.send_while(packet, go_on) {
                    return Ok(());
                }
                samples += 1;
                resyncs = 0;
                // After the send: a slow reader is no silent meter.
                if let Some(text) = watch.silence.reading(Instant::now()) {
                    log::warn!("{text}");
                }
            }
            let limit = self.limits.samples.load(Ordering::SeqCst);
            if limit > 0 && samples >= limit {
                return Ok(());
            }
            let msec = self.limits.msec.load(Ordering::SeqCst);
            if msec > 0 && self.started.elapsed() >= Duration::from_millis(msec) {
                return Ok(());
            }
        }
    }
}
