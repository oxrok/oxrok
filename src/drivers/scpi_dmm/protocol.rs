// Counterpart in sigrok:
//   libsigrok/src/hardware/scpi-dmm/api.c,
//   libsigrok/src/hardware/scpi-dmm/protocol.c,
//   libsigrok/src/hardware/scpi-dmm/protocol.h,
//   Copyright (C) 2018 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Timo Kokkonen, Petteri Aimonen, Adrian Godwin,
//   Electro707, Frank Stettner, Mike Walters, Mike Williams, Sahaj Sarup,
//   Sylvain Pelissier
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The tables of `scpi-dmm`, and what its answers mean, with no instrument: which
//! text is which function, how a function query gives a reading's digits, what is a
//! reading, and when a silent meter has been silent too long.
//!
//! The conversation, for what the tables hold, each query with its `*OPC?` first
//! ([`Model::commands`] leaves out what a dialect lacks):
//!
//! - a start: function query, precision query, start of acquisition, a delay for some
//!   functions on some models;
//! - each round: `*OPC?`, function query, value query (a GW Instek's reader has
//!   one more `*OPC?`), one sample per round, the function re-read every time;
//! - an end: stop of acquisition; at the end of a command line's capture, the local
//!   command.
//!
//! What a function query says, real ones from logs in libsigrok's history (e90551c3):
//! `"FREQ +1.00000E+00,+1.00000E-05"`, its last field giving a reading's digits by
//! its exponent alone, `"RES +1.00000E+08,+1.00000E+3"` with a short exponent, and
//! `"DIOD"` with no precision. A reading: `+9.90000000E+37 ` with a space on the end,
//! on overload, and `+9.00555531E+01`. It holds no check of any kind, so a digit
//! wrong on the line is a wrong number: [`reading_damage`] counts what the form does
//! catch. A function code of GW's is matched by its start: `013` reads as `01`.

use crate::core::{Mq, MqFlags, Unit};
use crate::drivers::motech_lps_30x::protocol::{atod_ascii, atoi};
use crate::scpi;
use std::time::{Duration, Instant};

/// What a table says, one per `enum scpi_dmm_cmdcode`. `SetupRemote` is in C's tables
/// and nothing sends it (`api.c:52`, and the code has no use of it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmd {
    /// `DMM_CMD_SETUP_REMOTE`.
    SetupRemote,
    /// `DMM_CMD_SETUP_LOCAL`.
    SetupLocal,
    /// `DMM_CMD_SETUP_FUNC`.
    SetupFunc,
    /// `DMM_CMD_QUERY_FUNC`.
    QueryFunc,
    /// `DMM_CMD_START_ACQ`.
    StartAcq,
    /// `DMM_CMD_STOP_ACQ`.
    StopAcq,
    /// `DMM_CMD_QUERY_VALUE`.
    QueryValue,
    /// `DMM_CMD_QUERY_PREC`.
    QueryPrec,
    /// `DMM_CMD_QUERY_RANGE_AUTO`.
    QueryRangeAuto,
    /// `DMM_CMD_QUERY_RANGE`.
    QueryRange,
    /// `DMM_CMD_SETUP_RANGE`.
    SetupRange,
}

/// A table of commands.
pub type Commands = &'static [(Cmd, &'static str)];

const AGILENT: Commands = &[
    (Cmd::SetupRemote, "\n"),
    (Cmd::SetupLocal, "SYST:LOC"),
    (Cmd::SetupFunc, "CONF:%s"),
    (Cmd::QueryFunc, "CONF?"),
    (Cmd::StartAcq, "INIT"),
    (Cmd::StopAcq, "ABORT"),
    (Cmd::QueryValue, "FETCH?"),
    (Cmd::QueryPrec, "CONF?"),
    (Cmd::QueryRangeAuto, "%s:RANGE:AUTO?"),
    (Cmd::QueryRange, "%s:RANGE?"),
    (Cmd::SetupRange, "CONF:%s %s"),
];

const HP: Commands = &[
    (Cmd::SetupRemote, "\n"),
    (Cmd::SetupFunc, "CONF:%s"),
    (Cmd::QueryFunc, "CONF?"),
    (Cmd::StartAcq, "INIT"),
    (Cmd::StopAcq, "ABORT"),
    (Cmd::QueryValue, "READ?"),
    (Cmd::QueryPrec, "CONF?"),
];

const GWINSTEK: Commands = &[
    (Cmd::SetupRemote, "SYST:REM"),
    (Cmd::SetupLocal, "SYST:LOC"),
    (Cmd::SetupFunc, "CONF:%s"),
    (Cmd::QueryFunc, "CONF:STAT:FUNC?"),
    (Cmd::StartAcq, "*CLS;SYST:REM"),
    (Cmd::QueryValue, "VAL1?"),
    (Cmd::QueryPrec, "SENS:DET:RATE?"),
];

const GWINSTEK_906X: Commands = &[
    (Cmd::SetupRemote, "SYST:REM"),
    (Cmd::SetupLocal, "SYST:LOC"),
    (Cmd::SetupFunc, "CONF:%s"),
    (Cmd::QueryFunc, "CONF?"),
    (Cmd::StartAcq, "INIT"),
    (Cmd::StopAcq, "ABORT"),
    (Cmd::QueryValue, "VAL1?"),
    (Cmd::QueryPrec, "SENS:DET:RATE?"),
];

const OWON: Commands = &[
    (Cmd::SetupRemote, "SYST:REM"),
    (Cmd::SetupLocal, "SYST:LOC"),
    (Cmd::SetupFunc, "CONF:%s"),
    (Cmd::QueryFunc, "FUNC?"),
    (Cmd::QueryValue, "MEAS1?"),
];

/// One line of a model's table of functions: `struct mqopt_item`.
#[allow(clippy::struct_excessive_bools)] // C's flags, by their names.
#[derive(Clone, Copy, Debug)]
pub struct MqOpt {
    /// What it measures.
    pub mq: Mq,
    /// DC, AC, four wire or nothing.
    pub flag: MqFlags,
    /// And the diode flag, which a `const` can't add.
    pub diode: bool,
    /// What goes in the commands.
    pub setup: &'static str,
    /// How a function query's answer starts.
    pub query: &'static str,
    /// The exponent of the precision, where a function query gives none.
    pub default_precision: Option<i32>,
    /// `FLAG_NO_RANGE`.
    pub no_range: bool,
    /// `FLAG_CONF_DELAY`.
    pub conf_delay: bool,
    /// `FLAG_MEAS_DELAY`.
    pub meas_delay: bool,
}

impl MqOpt {
    /// The flags, as a `measured_quantity` value has them.
    pub fn flags(&self) -> MqFlags {
        if self.diode { self.flag | MqFlags::DIODE } else { self.flag }
    }
}

const fn plain(mq: Mq, flag: MqFlags, setup: &'static str, query: &'static str) -> MqOpt {
    MqOpt {
        mq,
        flag,
        diode: false,
        setup,
        query,
        default_precision: None,
        no_range: false,
        conf_delay: false,
        meas_delay: false,
    }
}

const NO: MqFlags = MqFlags::empty();

/// The continuity line, `query` as the table spells it.
const fn continuity(query: &'static str) -> MqOpt {
    let mut item = plain(Mq::Continuity, NO, "CONT", query);
    item.default_precision = Some(-1);
    item
}

/// The diode line, `query` as the table spells it.
const fn diode(query: &'static str) -> MqOpt {
    let mut item = plain(Mq::Voltage, MqFlags::DC, "DIOD", query);
    item.diode = true;
    item.default_precision = Some(-4);
    item
}

const fn no_range(mut item: MqOpt) -> MqOpt {
    item.no_range = true;
    item
}

const fn delays(mut item: MqOpt, conf: bool) -> MqOpt {
    item.conf_delay = conf;
    item.meas_delay = true;
    item
}

const AGILENT_34405A: &[MqOpt] = &[
    plain(Mq::Voltage, MqFlags::DC, "VOLT:DC", "VOLT "),
    delays(plain(Mq::Voltage, MqFlags::AC, "VOLT:AC", "VOLT:AC "), true),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "CURR "),
    delays(plain(Mq::Current, MqFlags::AC, "CURR:AC", "CURR:AC "), true),
    delays(plain(Mq::Resistance, NO, "RES", "RES "), false),
    delays(plain(Mq::Resistance, MqFlags::FOUR_WIRE, "FRES", "FRES "), false),
    no_range(continuity("CONT")),
    plain(Mq::Capacitance, NO, "CAP", "CAP "),
    no_range(diode("DIOD")),
    no_range(delays(plain(Mq::Temperature, NO, "TEMP", "TEMP "), false)),
    no_range(delays(plain(Mq::Frequency, NO, "FREQ", "FREQ "), false)),
];

const AGILENT_34401A: &[MqOpt] = &[
    plain(Mq::Voltage, MqFlags::DC, "VOLT:DC", "VOLT "),
    plain(Mq::Voltage, MqFlags::AC, "VOLT:AC", "VOLT:AC "),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "CURR "),
    plain(Mq::Current, MqFlags::AC, "CURR:AC", "CURR:AC "),
    plain(Mq::Resistance, NO, "RES", "RES "),
    plain(Mq::Resistance, MqFlags::FOUR_WIRE, "FRES", "FRES "),
    continuity("CONT"),
    diode("DIOD"),
    plain(Mq::Frequency, NO, "FREQ", "FREQ "),
    plain(Mq::Time, NO, "PER", "PER "),
];

const GWINSTEK_GDM8200A: &[MqOpt] = &[
    plain(Mq::Voltage, MqFlags::DC, "VOLT:DC", "01"),
    plain(Mq::Voltage, MqFlags::AC, "VOLT:AC", "02"),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "03"),
    plain(Mq::Current, MqFlags::AC, "CURR:AC", "04"),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "05"),
    plain(Mq::Current, MqFlags::AC, "CURR:AC", "06"),
    plain(Mq::Resistance, NO, "RES", "07"),
    plain(Mq::Resistance, MqFlags::FOUR_WIRE, "FRES", "16"),
    continuity("13"),
    diode("17"),
    plain(Mq::Temperature, NO, "TEMP", "09"),
    plain(Mq::Temperature, NO, "TEMP", "15"),
    plain(Mq::Frequency, NO, "FREQ", "08"),
    plain(Mq::Time, NO, "PER", "14"),
];

const GWINSTEK_GDM906X: &[MqOpt] = &[
    plain(Mq::Voltage, MqFlags::DC, "VOLT:DC", "VOLT "),
    plain(Mq::Voltage, MqFlags::AC, "VOLT:AC", "VOLT:AC"),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "CURR "),
    plain(Mq::Current, MqFlags::AC, "CURR:AC", "CURR:AC"),
    plain(Mq::Resistance, NO, "RES", "RES"),
    plain(Mq::Resistance, MqFlags::FOUR_WIRE, "FRES", "FRES"),
    continuity("CONT"),
    diode("DIOD"),
    plain(Mq::Temperature, NO, "TEMP", "TEMP"),
    plain(Mq::Frequency, NO, "FREQ", "FREQ"),
    plain(Mq::Time, NO, "PER", "PER"),
    plain(Mq::Capacitance, NO, "CAP", "CAP"),
];

const OWON_XDM2041: &[MqOpt] = &[
    plain(Mq::Voltage, MqFlags::AC, "VOLT:AC", "VOLT AC"),
    plain(Mq::Voltage, MqFlags::DC, "VOLT:DC", "VOLT"),
    plain(Mq::Current, MqFlags::AC, "CURR:AC", "CURR AC"),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "CURR"),
    plain(Mq::Resistance, NO, "RES", "RES"),
    plain(Mq::Resistance, MqFlags::FOUR_WIRE, "FRES", "FRES"),
    continuity("CONT"),
    diode("DIOD"),
    plain(Mq::Temperature, NO, "TEMP", "TEMP"),
    plain(Mq::Frequency, NO, "FREQ", "FREQ"),
    plain(Mq::Capacitance, NO, "CAP", "CAP"),
];

const SIGLENT_SDM3055: &[MqOpt] = &[
    plain(Mq::Voltage, MqFlags::DC, "VOLT:DC", "VOLT "),
    plain(Mq::Voltage, MqFlags::AC, "VOLT:AC", "VOLT:AC "),
    plain(Mq::Current, MqFlags::DC, "CURR:DC", "CURR "),
    plain(Mq::Current, MqFlags::AC, "CURR:AC", "CURR:AC "),
    plain(Mq::Resistance, NO, "RES", "RES "),
    plain(Mq::Resistance, MqFlags::FOUR_WIRE, "FRES", "FRES "),
    continuity("CONT"),
    diode("DIOD"),
    plain(Mq::Frequency, NO, "FREQ", "FREQ "),
    plain(Mq::Time, NO, "PER", "PER "),
    plain(Mq::Capacitance, NO, "CAP", "CAP"),
];

/// Which of C's two readers a model takes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reader {
    /// `scpi_dmm_get_meas_agilent`.
    Agilent,
    /// `scpi_dmm_get_meas_gwinstek`.
    Gwinstek,
}

/// A line of the driver's table: `struct scpi_dmm_model`.
#[derive(Debug)]
pub struct Model {
    /// The maker, by `sr_vendor_alias`'s short name.
    pub vendor: &'static str,
    /// As `*IDN?` has it.
    pub name: &'static str,
    /// Without a precision to go by, a GW's or an OWON's digits.
    pub digits: i32,
    /// What it speaks.
    pub commands: Commands,
    /// What it measures.
    pub mqopts: &'static [MqOpt],
    /// How it is read.
    pub reader: Reader,
    /// It has a range key: `devopts_generic_range`.
    pub ranged: bool,
    /// What a read may take, if not the default.
    pub read_timeout: Option<Duration>,
    /// After a command that changes a function, where the table's flag says.
    pub conf_delay: Duration,
    /// After the start of an acquisition, where the table's flag says.
    pub meas_delay: Duration,
    /// Over this is infinity, if not 9e37.
    pub infinity_limit: Option<f64>,
    /// Ask `*OPC?` once in a scan to see if it has one.
    pub check_opc: bool,
}

const fn model(
    vendor: &'static str,
    name: &'static str,
    digits: i32,
    commands: Commands,
    mqopts: &'static [MqOpt],
    reader: Reader,
) -> Model {
    Model {
        vendor,
        name,
        digits,
        commands,
        mqopts,
        reader,
        ranged: false,
        read_timeout: None,
        conf_delay: Duration::ZERO,
        meas_delay: Duration::ZERO,
        infinity_limit: None,
        check_opc: false,
    }
}

/// `conf_delay` and `meas_delay` on a model with a range key.
const fn ranged(mut m: Model, conf_ms: u64, meas_ms: u64) -> Model {
    m.ranged = true;
    m.conf_delay = Duration::from_millis(conf_ms);
    m.meas_delay = Duration::from_millis(meas_ms);
    m
}

const fn timeout(mut m: Model, ms: u64) -> Model {
    m.read_timeout = Some(Duration::from_millis(ms));
    m
}

const fn owon(mut m: Model) -> Model {
    m.infinity_limit = Some(1e9);
    m.check_opc = true;
    m
}

use Reader::{Agilent, Gwinstek};

/// `models[]`.
pub static MODELS: [Model; 12] = [
    ranged(model("Agilent", "34405A", 5, AGILENT, AGILENT_34405A, Agilent), 200, 2500),
    model("Agilent", "34410A", 6, HP, AGILENT_34405A, Agilent),
    ranged(model("Agilent", "34460A", 6, AGILENT, AGILENT_34405A, Agilent), 0, 10),
    timeout(model("GW", "GDM8251A", 6, GWINSTEK, GWINSTEK_GDM8200A, Gwinstek), 2500),
    timeout(model("GW", "GDM8255A", 6, GWINSTEK, GWINSTEK_GDM8200A, Gwinstek), 2500),
    model("GWInstek", "GDM9060", 6, GWINSTEK_906X, GWINSTEK_GDM906X, Agilent),
    model("GWInstek", "GDM9061", 6, GWINSTEK_906X, GWINSTEK_GDM906X, Agilent),
    // 34401A: typically 1020 ms for AC readings, where the default is 1000 ms.
    timeout(model("HP", "34401A", 6, HP, AGILENT_34401A, Agilent), 1500),
    ranged(model("Keysight", "34465A", 6, AGILENT, AGILENT_34405A, Agilent), 0, 10),
    owon(model("OWON", "XDM1041", 5, OWON, OWON_XDM2041, Gwinstek)),
    owon(model("OWON", "XDM2041", 5, OWON, OWON_XDM2041, Gwinstek)),
    model("Siglent", "SDM3055", 5, HP, SIGLENT_SDM3055, Agilent),
];

/// `is_compatible`, by the maker's short name.
pub fn find_model(vendor: &str, name: &str) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.vendor == vendor && m.name == name)
}

impl Model {
    /// A command's text, if it has that command and it says anything.
    pub fn command(&self, command: Cmd) -> Option<&'static str> {
        scpi::cmd_get(self.commands, &command).filter(|text| !text.is_empty())
    }

    /// `scpi_dmm_lookup_mq_number`.
    pub fn by_number(&self, mq: Mq, flags: MqFlags) -> Option<&'static MqOpt> {
        self.mqopts.iter().find(|o| o.mq == mq && o.flags() == flags)
    }

    /// `scpi_dmm_lookup_mq_text`: by how the text starts, in the table's order.
    pub fn by_text(&self, text: &str) -> Option<&'static MqOpt> {
        self.mqopts.iter().find(|o| text.starts_with(o.query))
    }
}

/// Past `*have == '"'`, what a function query's answer is looked up by.
pub fn function_text(answer: &str) -> &str {
    answer.strip_prefix('"').unwrap_or(answer)
}

/// The exponent of the precision a function query's answer ends in, or the table's
/// fallback, or 0 with neither. `None` for one that is no number: a data error.
/// `scpi_dmm_get_meas_agilent`, as it comes, cut at C's 19 bytes.
pub fn precision_exponent(answer: &str, item: &MqOpt) -> Option<i32> {
    const PREC_TEXT: usize = 19;
    let unquoted = scpi::unquote_string(answer);
    let fields: Vec<&str> = unquoted.split(',').collect();
    let text: Vec<u8> = if fields.len() >= 2 {
        let last = fields[fields.len() - 1].as_bytes();
        last[..last.len().min(PREC_TEXT)].to_vec()
    } else if let Some(exponent) = item.default_precision {
        format!("1e{exponent}").into_bytes()
    } else {
        return Some(0);
    };
    exponent_of(&text)
}

/// Strip space, sign and a number with no exponent: then the end, or an exponent.
fn exponent_of(text: &[u8]) -> Option<i32> {
    let mut at = text.iter().position(|c| !c_space(char::from(*c))).unwrap_or(text.len());
    if matches!(text.get(at), Some(b'+' | b'-')) {
        at += 1;
    }
    while text.get(at).is_some_and(u8::is_ascii_digit) {
        at += 1;
    }
    if text.get(at) == Some(&b'.') {
        at += 1;
    }
    while text.get(at).is_some_and(u8::is_ascii_digit) {
        at += 1;
    }
    match text.get(at) {
        None => Some(0),
        Some(b'e' | b'E') => atoi(&text[at + 1..]),
        Some(_) => None,
    }
}

/// `digits = -prec_exp` through an `int8_t`: `analog->encoding->digits`.
pub fn digits_of_exponent(exponent: i32) -> i32 {
    i32::from(i64::from(exponent).wrapping_neg() as i8)
}

/// A reading that is no reading.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NoReading(pub String);

impl std::fmt::Display for NoReading {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "No usable reading: '{}'", self.0.escape_debug())
    }
}

/// What a reading's text says, as `sr_atod_ascii` and the form check after it have
/// it, with what is out of range for a double refused, as C's `errno`: `+inf` and
/// `-inf` over `limit`, or from `inf` itself. `at_limit` is the GW reader's `>=`, where
/// the Agilent's is `>`.
///
/// An empty answer and one with a NUL are refused, a stated difference:
/// `sr_atod_ascii` gives an empty text 0 and reads a C string up to its NUL, so a meter
/// that answers nothing reads 0 V with exit 0. `nan`, and the hexadecimal strtod takes,
/// fall to C's form check, as here.
pub fn parse_reading(text: &str, limit: f64, at_limit: bool) -> Result<f64, NoReading> {
    let refuse = || NoReading(text.to_owned());
    let body = text.trim_matches(c_space);
    if body.is_empty() || body.contains('\0') {
        return Err(refuse());
    }
    let value = atod_ascii(body.as_bytes()).ok_or_else(refuse)?;
    let over = if at_limit { value >= limit } else { value > limit };
    let under = if at_limit { value <= -limit } else { value < -limit };
    if over {
        Ok(f64::INFINITY)
    } else if under {
        Ok(f64::NEG_INFINITY)
    } else {
        // The form: the same walk as a precision's, and what a double took above may
        // still be no int in its exponent.
        exponent_of(body.as_bytes()).ok_or_else(refuse)?;
        Ok(value)
    }
}

/// The limit a model's reader goes by.
pub fn reading_limit(model: &Model) -> (f64, bool) {
    match model.reader {
        Reader::Agilent => (9e37, false),
        Reader::Gwinstek => (model.infinity_limit.unwrap_or(9e37), true),
    }
}

/// `atoi` on a whole answer, for a GW's function code: 0 of what is no number.
pub fn mode_code(answer: &str) -> i64 {
    let text = answer.trim_start_matches(c_space);
    let (negative, digits) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    let end = digits.find(|c: char| !c.is_ascii_digit()).unwrap_or(digits.len());
    let value = digits[..end].parse::<i64>().unwrap_or(0);
    if negative { -value } else { value }
}

/// "Make sure we report INFINITY when meter displays 0L", by a GW's function code.
#[allow(clippy::float_cmp)] // C's `== 1.0e0`: the very text a diode's OL reads.
pub fn gw_overload(code: i64, model: &str, value: f64) -> f64 {
    let over = match code {
        7 | 16 => value >= if model == "GDM8255A" { 1.99999e8 } else { 1.2e8 },
        13 => value >= 1.2e3,
        17 => value == 1.0,
        _ => false,
    };
    if over { f64::INFINITY } else { value }
}

/// A GW's digits by the rate it named at the start, else the table's.
pub fn gw_digits(model: &Model, precision: Option<&str>) -> i32 {
    match precision {
        Some(p) if p.starts_with("Slow") => 6,
        Some(p) if p.starts_with("Mid") => 5,
        Some(p) if p.starts_with("Fast") => 4,
        Some(p) => {
            if !p.is_empty() {
                log::info!("Unknown precision: '{p}'");
            }
            model.digits
        }
        None => model.digits,
    }
}

/// A unit by what is measured, and by a GW's code for a temperature.
pub fn unit_of(mq: Mq, code: i64) -> Unit {
    match mq {
        Mq::Voltage => Unit::Volt,
        Mq::Current => Unit::Ampere,
        Mq::Resistance | Mq::Continuity => Unit::Ohm,
        Mq::Capacitance => Unit::Farad,
        Mq::Temperature if code == 15 => Unit::Fahrenheit,
        Mq::Temperature => Unit::Celsius,
        Mq::Frequency => Unit::Hertz,
        Mq::Time => Unit::Second,
        // The tables hold no other: C's SR_ERR_NA is out of reach.
        other => other.default_unit(),
    }
}

/// A range's answer as `--get range` has it: to the comma, a number whose form
/// `sr_count_digits` takes, as `%lf`. `None` where C returns no text.
pub fn range_text(answer: &str) -> Option<String> {
    let text = answer.split(',').next().unwrap_or(answer);
    if !counts_digits(text.as_bytes()) {
        return None;
    }
    atod_ascii(text.as_bytes()).map(|range| format!("{range:.6}"))
}

/// `sr_count_digits` says it can: a number with some digit, and no more than that.
fn counts_digits(text: &[u8]) -> bool {
    let mut at = text.iter().position(|c| !c_space(char::from(*c))).unwrap_or(text.len());
    if matches!(text.get(at), Some(b'+' | b'-')) {
        at += 1;
    }
    let start = at;
    while text.get(at).is_some_and(u8::is_ascii_digit) {
        at += 1;
    }
    let mut digits = at - start;
    if text.get(at) == Some(&b'.') {
        at += 1;
        let start = at;
        while text.get(at).is_some_and(u8::is_ascii_digit) {
            at += 1;
        }
        digits += at - start;
    }
    if digits == 0 {
        return false;
    }
    match text.get(at) {
        None => true,
        Some(b'e' | b'E') => {
            let rest = &text[at + 1..];
            let body = rest.iter().position(|c| !c_space(char::from(*c))).unwrap_or(rest.len());
            let rest = &rest[body..];
            let sign = usize::from(matches!(rest.first(), Some(b'+' | b'-')));
            let end = rest[sign..].iter().take_while(|c| c.is_ascii_digit()).count();
            // `errno` on what a long can't hold, and nothing may be left over.
            sign + end == rest.len()
                && (end == 0 || std::str::from_utf8(rest).is_ok_and(|e| e.parse::<i64>().is_ok()))
        }
        Some(_) => false,
    }
}

/// C's white space, `g_ascii_isspace` and `isspace` in the C locale: Rust's ASCII white
/// space and the vertical tab.
pub fn c_space(c: char) -> bool {
    c.is_ascii_whitespace() || c == '\x0b'
}

/// What a range may be: a number, as `sr_atod_ascii` reads one, or `AUTO`, `MIN`, `MAX`
/// or `DEF` in any case. It goes out inside `CONF:%s %s`, so a `;` or a line end in it
/// would be another command: no number holds one.
pub fn range_word_ok(range: &str) -> bool {
    let word = ["AUTO", "MIN", "MAX", "DEF"].iter().any(|w| range.eq_ignore_ascii_case(w));
    let number = !range.is_empty()
        && !range.contains(c_space)
        && atod_ascii(range.as_bytes()).is_some_and(f64::is_finite);
    word || number
}

/// `sr_scpi_get_bool`'s rule on a whole answer.
pub fn strict_bool(answer: &str) -> Option<bool> {
    if answer.trim() == answer { scpi::parse_bool(answer) } else { None }
}

/// How a silent meter goes on, by what can end the capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// A sample limit, and no time limit.
    pub only_samples: bool,
    /// A time limit, and its time has come.
    pub time_up: bool,
}

/// What comes of a read that found nothing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Ask or read on.
    Wait,
    /// Ask or read on, and say this.
    Warn(String),
    /// The limit has ended it, and this is said; exit 0.
    End(String),
    /// A failed capture, for this.
    Fail(String),
}

/// With no reading in the capture, how long before a capture only a sample limit can
/// end fails, and how long before a warning otherwise. A choice, above a real meter's
/// slowest readings: the 34401A's specifications have 100 power line cycles at 2 s a
/// reading on 50 Hz, and from memory, unverified, its slow AC filter takes some 7 s. A
/// trigger delay set at the panel, or a large `SAMP:COUNT`, can be longer: `*OPC?`
/// after `INIT` then outlasts 30 s, and a capture with only a sample limit fails where
/// the meter is only slow. C's read gives up at 1 s to 2.5 s.
pub const NO_READING: Duration = Duration::from_secs(30);
/// With readings before, how long a meter may be silent before such a capture fails.
pub const GIVE_UP: Duration = Duration::from_secs(600);

/// The rule on a silent meter, on times given to it.
#[derive(Clone, Debug)]
pub struct Silence {
    since: Instant,
    got_one: bool,
    warned: bool,
    /// [`NO_READING`].
    pub no_reading: Duration,
    /// [`GIVE_UP`].
    pub give_up: Duration,
}

impl Silence {
    /// A capture starting at `now`.
    pub fn new(now: Instant) -> Silence {
        Silence {
            since: now,
            got_one: false,
            warned: false,
            no_reading: NO_READING,
            give_up: GIVE_UP,
        }
    }

    /// A reading handed over at `now`. What to say if a warning came before.
    pub fn reading(&mut self, now: Instant) -> Option<String> {
        let back = self.warned.then(|| {
            format!(
                "Readings are back, after {:.1} s.",
                now.duration_since(self.since).as_secs_f32()
            )
        });
        self.since = now;
        self.got_one = true;
        self.warned = false;
        back
    }

    /// A read that began at `at` found nothing. The time goes by when it began, as a
    /// stall after the read is no silence of the meter's. A warning comes once to a
    /// silence.
    pub fn silent(&mut self, at: Instant, limits: Limits) -> Verdict {
        let gone = at.saturating_duration_since(self.since);
        let seconds = gone.as_secs_f32();
        if limits.time_up {
            return Verdict::End(if self.got_one {
                format!("No reading for {seconds:.1} s when the time limit came.")
            } else {
                "No reading in this capture, and the time limit has come.".to_owned()
            });
        }
        if limits.only_samples {
            if !self.got_one && gone >= self.no_reading {
                return Verdict::Fail(format!("No reading in {seconds:.1} s."));
            }
            if self.got_one && gone >= self.give_up {
                return Verdict::Fail(format!("No reading for {seconds:.1} s."));
            }
        }
        if gone >= self.no_reading && !self.warned {
            self.warned = true;
            return Verdict::Warn(format!("No reading for {seconds:.1} s, still waiting."));
        }
        Verdict::Wait
    }
}

/// What single-bit flips and lost bytes do to a reading on the line, for a report:
/// `(how many damaged forms, refused, read as another number)`.
#[cfg(test)]
#[allow(clippy::float_cmp)]
pub fn reading_damage(text: &str, limit: f64) -> (usize, usize, usize) {
    let good = parse_reading(text, limit, false).expect("a good reading");
    let mut forms: Vec<Vec<u8>> = Vec::new();
    for at in 0..text.len() {
        for bit in 0..8 {
            let mut form = text.as_bytes().to_vec();
            form[at] ^= 1 << bit;
            forms.push(form);
        }
        let mut form = text.as_bytes().to_vec();
        form.remove(at);
        forms.push(form);
    }
    let (mut refused, mut wrong) = (0, 0);
    for form in &forms {
        match parse_reading(&String::from_utf8_lossy(form), limit, false) {
            Err(_) => refused += 1,
            Ok(value) if value == good => {}
            Ok(_) => wrong += 1,
        }
    }
    (forms.len(), refused, wrong)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(name: &str) -> &'static Model {
        MODELS.iter().find(|m| m.name == name).unwrap()
    }

    #[test]
    fn the_table_goes_by_the_short_maker() {
        assert_eq!(find_model("Keysight", "34465A").unwrap().name, "34465A");
        assert_eq!(
            find_model("HP", "34401A").unwrap().read_timeout,
            Some(Duration::from_millis(1500))
        );
        assert!(find_model("Agilent", "34465A").is_none());
        assert!(find_model("agilent", "34405A").is_none());
        assert!(find_model("Agilent", "34405a").is_none());
        for (name, ranged, conf, meas) in [
            ("34405A", true, 200, 2500),
            ("34460A", true, 0, 10),
            ("34465A", true, 0, 10),
            ("34410A", false, 0, 0),
        ] {
            let m = at(name);
            assert_eq!(m.ranged, ranged, "{name}");
            assert_eq!(m.conf_delay, Duration::from_millis(conf), "{name}");
            assert_eq!(m.meas_delay, Duration::from_millis(meas), "{name}");
        }
        assert_eq!(at("XDM1041").infinity_limit, Some(1e9));
        assert!(at("XDM2041").check_opc);
        assert!(!at("GDM8251A").check_opc);
    }

    #[test]
    fn a_function_goes_by_how_its_text_starts() {
        let m = at("34405A");
        assert_eq!(m.by_text("VOLT +1E+00,+1E-05").unwrap().setup, "VOLT:DC");
        assert_eq!(m.by_text("VOLT:AC +1E+00").unwrap().setup, "VOLT:AC");
        assert_eq!(m.by_text("RES +1").unwrap().setup, "RES");
        assert_eq!(m.by_text("FRES +1").unwrap().setup, "FRES");
        assert!(m.by_text("VOLT").is_none(), "with no space, on this table");
        assert!(m.by_text("VOLT:DC").is_none());
        assert!(m.by_text("").is_none());
        let owon = at("XDM2041");
        assert_eq!(owon.by_text("VOLT AC").unwrap().setup, "VOLT:AC", "ahead of VOLT");
        assert_eq!(owon.by_text("VOLT").unwrap().setup, "VOLT:DC");
        let gw = at("GDM8251A");
        assert_eq!(gw.by_text("05").unwrap().mq, Mq::Current);
        assert_eq!(gw.by_text("013").unwrap().setup, "VOLT:DC", "by its start");
        assert!(gw.by_text("1").is_none());
        assert_eq!(function_text("\"VOLT\""), "VOLT\"");
    }

    #[test]
    fn a_number_finds_the_first_of_its_flags() {
        let gw = at("GDM8251A");
        assert_eq!(gw.by_number(Mq::Current, MqFlags::DC).unwrap().query, "03");
        assert_eq!(gw.by_number(Mq::Temperature, NO).unwrap().query, "09");
        let m = at("34465A");
        assert_eq!(m.by_number(Mq::Voltage, MqFlags::DC | MqFlags::DIODE).unwrap().setup, "DIOD");
        assert_eq!(m.by_number(Mq::Voltage, MqFlags::DC).unwrap().setup, "VOLT:DC");
        assert!(m.by_number(Mq::Voltage, NO).is_none());
        assert!(m.by_number(Mq::Time, NO).is_none());
    }

    /// The table's flags, line by line, against C's on the 34405A's.
    #[test]
    fn the_34405a_flags_are_cs() {
        let flags: Vec<(&str, bool, bool, bool)> = AGILENT_34405A
            .iter()
            .map(|o| (o.setup, o.no_range, o.conf_delay, o.meas_delay))
            .collect();
        assert_eq!(
            flags,
            [
                ("VOLT:DC", false, false, false),
                ("VOLT:AC", false, true, true),
                ("CURR:DC", false, false, false),
                ("CURR:AC", false, true, true),
                ("RES", false, false, true),
                ("FRES", false, false, true),
                ("CONT", true, false, false),
                ("CAP", false, false, false),
                ("DIOD", true, false, false),
                ("TEMP", true, false, true),
                ("FREQ", true, false, true),
            ]
        );
    }

    #[test]
    fn digits_come_from_the_last_field() {
        let freq = at("34405A").by_text("FREQ ").unwrap();
        let cont = at("34405A").by_text("CONT").unwrap();
        let diod = at("34405A").by_text("DIOD").unwrap();
        for (answer, item, want) in [
            // The real ones.
            ("\"FREQ +1.00000E+00,+1.00000E-05\"", freq, Some(-5)),
            ("\"RES +1.00000E+08,+1.00000E+3\"", freq, Some(3)),
            ("\"FREQ +3.00000000E+00,+3.00000000E-06\"", freq, Some(-6)),
            ("\"TEMP THER,5000,+1.00000E+00,+1.00000E-01\"", freq, Some(-1)),
            ("\"DIOD\"", diod, Some(-4)),
            ("\"CONT\"", cont, Some(-1)),
            // None to go by.
            ("\"FREQ\"", freq, Some(0)),
            // What the walk takes and refuses.
            ("\"FREQ 1,1.5\"", freq, Some(0)),
            ("\"FREQ 1,1E\"", freq, Some(0)),
            ("\"FREQ 1, +1E-03\"", freq, Some(-3)),
            ("\"FREQ 1,+1E-03 \"", freq, Some(-3)),
            ("\"FREQ 1,x\"", freq, None),
            ("\"FREQ 1,1Ex\"", freq, None),
            ("\"FREQ 1,1E-99999999999\"", freq, None),
            // No closing quote: unquoted it isn't, and a field has the rest.
            ("\"FREQ 1,1E-03", freq, Some(-3)),
            // No opening one: the closing quote is in the field.
            ("FREQ 1,1E-03\"", freq, None),
            // Cut at 19 bytes, so ahead of its exponent.
            ("\"FREQ 1,1.00000000000000000000E-06\"", freq, Some(0)),
        ] {
            assert_eq!(precision_exponent(answer, item), want, "{answer}");
        }
    }

    #[test]
    fn digits_go_through_a_byte() {
        assert_eq!(digits_of_exponent(-5), 5);
        assert_eq!(digits_of_exponent(3), -3);
        assert_eq!(digits_of_exponent(-200), -56);
        assert_eq!(digits_of_exponent(i32::MIN), 0, "no panic");
    }

    #[test]
    fn readings_as_the_logs_and_the_form_have_them() {
        let read = |t| parse_reading(t, 9e37, false);
        assert_eq!(read("+9.00555531E+01"), Ok(90.055_553_1));
        assert_eq!(read("+9.90000000E+37 "), Ok(f64::INFINITY), "the 34405A's");
        assert_eq!(read("-9.90000000E+37"), Ok(f64::NEG_INFINITY));
        assert_eq!(read("+9.00000000E+37"), Ok(9e37), "at it, by C's >");
        assert_eq!(read("  +1.5E+00  "), Ok(1.5));
        assert_eq!(read("\x0b+1.5E+00\x0b"), Ok(1.5), "isspace has the vertical tab");
        assert_eq!(read("1.5"), Ok(1.5));
        assert_eq!(read(".5"), Ok(0.5));
        assert_eq!(read("5."), Ok(5.0));
        assert_eq!(read("5e-3"), Ok(5e-3));
        assert_eq!(read("inf"), Ok(f64::INFINITY));
        assert_eq!(read("-infinity"), Ok(f64::NEG_INFINITY));
        for t in [
            "",
            "   ",
            "x",
            "1.5x",
            "1e",
            "nan",
            "0x10",
            "1e999",
            "1e-999",
            "1e-310",
            "0e99999999999",
            "1 2",
            "1,2",
            "1\0",
        ] {
            let refused = read(t);
            assert_eq!(refused, Err(NoReading(t.to_owned())), "{t:?}");
        }
        let gw = |t| parse_reading(t, 1e9, true);
        assert_eq!(gw("1E+9"), Ok(f64::INFINITY), "at it, by C's >=");
        assert_eq!(gw("-1E+9"), Ok(f64::NEG_INFINITY));
        assert_eq!(gw("9.9E+8"), Ok(9.9e8));
        assert_eq!(reading_limit(at("XDM2041")), (1e9, true));
        assert_eq!(reading_limit(at("GDM8251A")), (9e37, true));
        assert_eq!(reading_limit(at("34401A")), (9e37, false));
        assert_eq!(NoReading("a\nb".to_owned()).to_string(), "No usable reading: 'a\\nb'");
    }

    /// What the form catches on the longest real reading. It holds no check: what is
    /// left is what the format cannot catch, and this pins the count for the docs.
    #[test]
    fn what_a_reading_cannot_catch() {
        let (forms, refused, wrong) = reading_damage("+9.00555531E+01", 9e37);
        assert_eq!(forms, 15 * 9);
        assert_eq!((refused, wrong), (82, 48), "{refused} refused, {wrong} wrong of {forms}");
    }

    #[test]
    fn a_gw_says_overload_by_its_code() {
        for (code, model, value, inf) in [
            (7, "GDM8251A", 1.2e8, true),
            (7, "GDM8251A", 1.199e8, false),
            (16, "GDM8251A", 1.2e8, true),
            (7, "GDM8255A", 1.2e8, false),
            (7, "GDM8255A", 1.99999e8, true),
            (16, "GDM8255A", 1.99999e8, true),
            (13, "GDM8251A", 1.2e3, true),
            (13, "GDM8251A", 1.19e3, false),
            (17, "GDM8251A", 1.0, true),
            (17, "GDM8251A", 0.999, false),
            (17, "GDM8251A", 1.001, false),
            (1, "GDM8251A", 1e12, false),
        ] {
            assert_eq!(
                gw_overload(code, model, value).is_infinite(),
                inf,
                "{code} {model} {value}"
            );
        }
        assert_eq!(mode_code("01"), 1);
        assert_eq!(mode_code("15"), 15);
        assert_eq!(mode_code("\"01\""), 0);
        assert_eq!(mode_code(" +7x"), 7);
        assert_eq!(mode_code("-7"), -7);
        assert_eq!(mode_code(""), 0);
        assert_eq!(mode_code("99999999999999999999999"), 0, "no panic");
    }

    #[test]
    fn a_gw_says_digits_by_its_rate() {
        let m = at("GDM8251A");
        assert_eq!(gw_digits(m, Some("Slow")), 6);
        assert_eq!(gw_digits(m, Some("Mid")), 5);
        assert_eq!(gw_digits(m, Some("Medium")), 6, "C's table has Mid");
        assert_eq!(gw_digits(m, Some("Fast")), 4);
        assert_eq!(gw_digits(m, Some("x")), 6);
        assert_eq!(gw_digits(m, Some("")), 6);
        assert_eq!(gw_digits(m, None), 6);
        assert_eq!(gw_digits(at("XDM2041"), None), 5);
    }

    #[test]
    fn units_are_what_cs_are() {
        assert_eq!(unit_of(Mq::Temperature, 15), Unit::Fahrenheit);
        assert_eq!(unit_of(Mq::Temperature, 9), Unit::Celsius);
        assert_eq!(unit_of(Mq::Temperature, 0), Unit::Celsius);
        assert_eq!(unit_of(Mq::Continuity, 0), Unit::Ohm);
        assert_eq!(unit_of(Mq::Time, 0), Unit::Second);
    }

    #[test]
    fn a_range_says_as_percent_lf() {
        assert_eq!(range_text("+1.00000E+01").as_deref(), Some("10.000000"));
        assert_eq!(range_text("+1.00000E+01,+1.00000E-04").as_deref(), Some("10.000000"));
        assert_eq!(range_text("10").as_deref(), Some("10.000000"));
        assert_eq!(range_text("1e-3").as_deref(), Some("0.001000"));
        for bad in ["", "x", "1x", "inf", "nan", ",1", "1e", "0e99999999999999999999"] {
            assert_eq!(range_text(bad), None, "{bad}");
        }
    }

    #[test]
    fn a_range_is_a_number_or_a_word() {
        for ok in ["10", "1e-3", "+1.0E+01", ".5", "auto", "AUTO", "Min", "max", "def"] {
            assert!(range_word_ok(ok), "{ok}");
        }
        for bad in
            ["", "10;*RST", "10\n*RST", "10\r", "volts", "inf", "nan", "1 0", "AUTO;*RST", " 10"]
        {
            assert!(!range_word_ok(bad), "{bad:?}");
        }
    }

    #[test]
    fn a_bool_is_read_whole() {
        assert_eq!(strict_bool("1"), Some(true));
        assert_eq!(strict_bool("0"), Some(false));
        assert_eq!(strict_bool(" 1"), None);
        assert_eq!(strict_bool("x"), None);
    }

    fn only_samples() -> Limits {
        Limits { only_samples: true, time_up: false }
    }

    fn free() -> Limits {
        Limits { only_samples: false, time_up: false }
    }

    #[test]
    fn with_no_reading_a_sample_limit_fails_at_thirty_seconds() {
        let t0 = Instant::now();
        let mut s = Silence::new(t0);
        assert_eq!(s.silent(t0 + Duration::from_millis(29_900), only_samples()), Verdict::Wait);
        assert_eq!(
            s.silent(t0 + Duration::from_secs(30), only_samples()),
            Verdict::Fail("No reading in 30.0 s.".to_owned())
        );
    }

    #[test]
    fn after_a_reading_it_warns_once_and_fails_at_ten_minutes() {
        let t0 = Instant::now();
        let mut s = Silence::new(t0);
        assert_eq!(s.reading(t0), None);
        assert_eq!(s.silent(t0 + Duration::from_millis(29_900), only_samples()), Verdict::Wait);
        assert_eq!(
            s.silent(t0 + Duration::from_secs(30), only_samples()),
            Verdict::Warn("No reading for 30.0 s, still waiting.".to_owned())
        );
        assert_eq!(s.silent(t0 + Duration::from_secs(40), only_samples()), Verdict::Wait);
        assert_eq!(s.silent(t0 + Duration::from_secs(599), only_samples()), Verdict::Wait);
        assert_eq!(
            s.silent(t0 + Duration::from_secs(600), only_samples()),
            Verdict::Fail("No reading for 600.0 s.".to_owned())
        );
    }

    #[test]
    fn a_reading_resets_it_and_says_it_is_back() {
        let t0 = Instant::now();
        let mut s = Silence::new(t0);
        s.reading(t0);
        let t1 = t0 + Duration::from_secs(590);
        assert!(matches!(s.silent(t1, only_samples()), Verdict::Warn(_)));
        assert_eq!(s.reading(t1).as_deref(), Some("Readings are back, after 590.0 s."));
        assert_eq!(s.reading(t1), None, "said once");
        // Ten bounds' worth, in pieces, with a reading between each.
        let mut at = t1;
        for _ in 0..10 {
            at += Duration::from_secs(599);
            assert!(matches!(s.silent(at, only_samples()), Verdict::Warn(_)));
            s.reading(at);
        }
        // And a warning for a silence each.
        assert!(matches!(s.silent(at + Duration::from_secs(30), only_samples()), Verdict::Warn(_)));
    }

    #[test]
    fn a_time_limit_ends_it_with_a_word_at_any_time() {
        let t0 = Instant::now();
        let up = Limits { only_samples: false, time_up: true };
        let mut s = Silence::new(t0);
        assert_eq!(
            s.silent(t0, up),
            Verdict::End("No reading in this capture, and the time limit has come.".to_owned())
        );
        s.reading(t0);
        assert_eq!(
            s.silent(t0 + Duration::from_secs(2), up),
            Verdict::End("No reading for 2.0 s when the time limit came.".to_owned())
        );
    }

    #[test]
    fn under_neither_limit_it_warns_and_goes_on_past_any_bound() {
        let t0 = Instant::now();
        let mut s = Silence::new(t0);
        assert_eq!(s.silent(t0 + Duration::from_secs(29), free()), Verdict::Wait);
        assert!(matches!(s.silent(t0 + Duration::from_secs(30), free()), Verdict::Warn(_)));
        assert_eq!(s.silent(t0 + Duration::from_secs(100_000), free()), Verdict::Wait);
        // A time limit not yet up is the same.
        s.reading(t0);
        assert!(matches!(s.silent(t0 + Duration::from_secs(100_000), free()), Verdict::Warn(_)));
    }

    #[test]
    fn the_bound_goes_by_when_the_read_began() {
        let t0 = Instant::now();
        let mut s = Silence::new(t0);
        // A read that began before the capture's clock, as a saturating sum.
        let earlier = t0.checked_sub(Duration::from_secs(1)).unwrap_or(t0);
        assert_eq!(s.silent(earlier, only_samples()), Verdict::Wait);
    }
}
