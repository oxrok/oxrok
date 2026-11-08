// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! SCPI bench multimeters, on USBTMC and on TCP, as `scpi-dmm` speaks to them: five
//! dialects under twelve names.
//!
//! The device, and how sure this is:
//!
//! - Real, from session logs in libsigrok's commit messages (e90551c3, 18426d1c):
//!   a 34405A's `"FREQ +1.00000E+00,+1.00000E-05"` and `"RES +1.00000E+02,+1.00000E-03"`,
//!   its `+9.90000000E+37 ` with a space on the end for an overload; a 34465A's
//!   `"FREQ +3.00000000E+00,+3.00000000E-06"` and `+9.00555531E+01`; `*OPC?` answering
//!   `1`, and going unanswered for a second or more right after `INIT` (`opc-pace=`).
//! - From commit messages too: that a range query on temperature, frequency, diode
//!   and continuity fails and loses the USB connection on a 34405A and a 34465A
//!   (5bf642db), so it is flagged here; that `SYST:LOC` is what ends remote mode.
//! - From the C driver's comment only: `"TEMP THER,5000,+1.00000E+00,+1.00000E-01"`,
//!   `"DIOD"`, and that a range answer may have a comma and a precision after it.
//! - From the C driver's tables only, a reading of what it sends and what it expects
//!   back, with no device or manual behind it: all of GW Instek's (the codes of
//!   `CONF:STAT:FUNC?`, `VAL1?`, `SENS:DET:RATE?`), OWON's (`FUNC?`, `MEAS1?`, 1e9 for
//!   an overload), and that an OWON has no `*OPC?` and says nothing to one
//!   (commit ddeaa49d). The model and the port share that reading.
//!
//! Guesses, from memory and from no source, which no test should lean on: the USB
//! id, one for all twelve so that `model=` costs no re-attach (0957:0618 is what
//! memory gives for a 34405A); serial numbers and firmware; the ladder of ranges, and
//! `-222` and `-224` for what it refuses; the values read, as a stand-in.
//!
//! Emulation artifacts: a reading is ready at once unless `reading-pace=` says otherwise, and
//! changes by one step per reading. `CONF:` always takes effect. `INIT` is only a flag.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: as in the driver's table, `34405A` by default;
//! - `func=SETUP`: what it is set to at power on, as the setup command spells it;
//!   `VOLT:DC` by default;
//! - `manual=1`: on a fixed range at power on, not on auto;
//! - `readings=A|B|...`: what the next readings say, once each;
//! - `functions=A|B|...`: what the next function queries say, once each;
//! - `say=MAKER,MODEL,...`: what `*IDN?` says. In these three, `_` is a space, which an
//!   option's value can't hold;
//! - `rate=TEXT`: what `SENS:DET:RATE?` says, `Slow` by default;
//! - `range=TEXT`, `auto=TEXT`: what `:RANGE?` and `:RANGE:AUTO?` say;
//! - `reading-pace=T`: each reading waits T ticks;
//! - `opc-pace=T`: the first `*OPC?` after `INIT` waits T ticks;
//! - `opc=1`, `opc=0`: it has `*OPC?`, or says nothing to one; by its dialect otherwise.
//!
//! The pedantic checks: only what the dialect has; a function that isn't its own; a
//! range on a function that has none; a reading ahead of `INIT` where it needs one. On
//! USBTMC the shell's `one-request=1` is on by default: a second request while one is
//! unanswered is flagged. `one-request=0` turns it off, for C, which asks again after a
//! timeout.

use crate::model::{DeviceModel, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "scpi-dmm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "SCPI multimeters on USBTMC and TCP; model=, func=, manual=, readings=, \
                         functions=, say=, rate=, range=, auto=, reading-pace=, opc-pace=, opc=, and the SCPI \
                         shell's mute-at=, silent-at=, late-at=, opc-late=, mark-at=, idn=, eol=, \
                         stall=, in-stall=, empty-at=, bad-tag-at=, message=";

/// A way of speaking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dialect {
    /// `cmdset_agilent`.
    Agilent,
    /// `cmdset_hp`.
    Hp,
    /// `cmdset_gwinstek`.
    Gw8200,
    /// `cmdset_gwinstek_906x`.
    Gw906x,
    /// `cmdset_owon`.
    Owon,
}

/// A function: what sets it, and how the function query names it.
struct Function {
    setup: &'static str,
    /// The text of the function query's answer, a dialect's own.
    name: &'static str,
    /// It has ranges to set and ask for. Only `Dialect::Agilent` takes a range at all.
    ranged: bool,
    /// The largest range, and a stand-in for what it reads.
    top: f64,
    value: f64,
}

const fn f(setup: &'static str, name: &'static str, ranged: bool, top: f64, value: f64) -> Function {
    Function { setup, name, ranged, top, value }
}

const AGILENT: &[Function] = &[
    f("VOLT:DC", "VOLT", true, 1000.0, 0.109_45),
    f("VOLT:AC", "VOLT:AC", true, 750.0, 1.5),
    f("CURR:DC", "CURR", true, 10.0, 0.012),
    f("CURR:AC", "CURR:AC", true, 10.0, 0.25),
    f("RES", "RES", true, 1e8, 1000.5),
    f("FRES", "FRES", true, 1e8, 99.5),
    f("CONT", "CONT", false, 0.0, 12.5),
    f("CAP", "CAP", true, 1e-5, 1e-6),
    f("DIOD", "DIOD", false, 0.0, 0.6),
    f("TEMP", "TEMP", false, 0.0, 23.5),
    f("FREQ", "FREQ", false, 0.0, 90.055_553_1),
];
const HP_34401A: &[Function] = &[
    f("VOLT:DC", "VOLT", false, 0.0, 0.109_45),
    f("VOLT:AC", "VOLT:AC", false, 0.0, 1.5),
    f("CURR:DC", "CURR", false, 0.0, 0.012),
    f("CURR:AC", "CURR:AC", false, 0.0, 0.25),
    f("RES", "RES", false, 0.0, 1000.5),
    f("FRES", "FRES", false, 0.0, 99.5),
    f("CONT", "CONT", false, 0.0, 12.5),
    f("DIOD", "DIOD", false, 0.0, 0.6),
    f("FREQ", "FREQ", false, 0.0, 90.0),
    f("PER", "PER", false, 0.0, 0.011),
];
const SIGLENT: &[Function] = &[
    f("VOLT:DC", "VOLT", false, 0.0, 0.109_45),
    f("VOLT:AC", "VOLT:AC", false, 0.0, 1.5),
    f("CURR:DC", "CURR", false, 0.0, 0.012),
    f("CURR:AC", "CURR:AC", false, 0.0, 0.25),
    f("RES", "RES", false, 0.0, 1000.5),
    f("FRES", "FRES", false, 0.0, 99.5),
    f("CONT", "CONT", false, 0.0, 12.5),
    f("DIOD", "DIOD", false, 0.0, 0.6),
    f("FREQ", "FREQ", false, 0.0, 90.0),
    f("PER", "PER", false, 0.0, 0.011),
    f("CAP", "CAP", false, 0.0, 1e-6),
];
const GW_8200: &[Function] = &[
    f("VOLT:DC", "01", false, 0.0, 0.109_45),
    f("VOLT:AC", "02", false, 0.0, 1.5),
    f("CURR:DC", "03", false, 0.0, 0.012),
    f("CURR:AC", "04", false, 0.0, 0.25),
    f("RES", "07", false, 0.0, 1000.5),
    f("FRES", "16", false, 0.0, 99.5),
    f("CONT", "13", false, 0.0, 12.5),
    f("DIOD", "17", false, 0.0, 0.6),
    f("TEMP", "09", false, 0.0, 23.5),
    f("FREQ", "08", false, 0.0, 90.0),
    f("PER", "14", false, 0.0, 0.011),
];
const GW_906X: &[Function] = &[
    f("VOLT:DC", "VOLT", false, 0.0, 0.109_45),
    f("VOLT:AC", "VOLT:AC", false, 0.0, 1.5),
    f("CURR:DC", "CURR", false, 0.0, 0.012),
    f("CURR:AC", "CURR:AC", false, 0.0, 0.25),
    f("RES", "RES", false, 0.0, 1000.5),
    f("FRES", "FRES", false, 0.0, 99.5),
    f("CONT", "CONT", false, 0.0, 12.5),
    f("DIOD", "DIOD", false, 0.0, 0.6),
    f("TEMP", "TEMP", false, 0.0, 23.5),
    f("FREQ", "FREQ", false, 0.0, 90.0),
    f("PER", "PER", false, 0.0, 0.011),
    f("CAP", "CAP", false, 0.0, 1e-6),
];
const OWON: &[Function] = &[
    f("VOLT:AC", "VOLT AC", false, 0.0, 1.5),
    f("VOLT:DC", "VOLT", false, 0.0, 0.109_45),
    f("CURR:AC", "CURR AC", false, 0.0, 0.25),
    f("CURR:DC", "CURR", false, 0.0, 0.012),
    f("RES", "RES", false, 0.0, 1000.5),
    f("FRES", "FRES", false, 0.0, 99.5),
    f("CONT", "CONT", false, 0.0, 12.5),
    f("DIOD", "DIOD", false, 0.0, 0.6),
    f("TEMP", "TEMP", false, 0.0, 23.5),
    f("FREQ", "FREQ", false, 0.0, 90.0),
    f("CAP", "CAP", false, 0.0, 1e-6),
];

/// A meter of the driver's table.
struct Kind {
    /// `*IDN?`'s first field; the driver goes by its alias.
    vendor: &'static str,
    name: &'static str,
    dialect: Dialect,
    functions: &'static [Function],
    /// The decimals of what a function query says of range and resolution.
    decimals: usize,
    /// A space after a reading, as a 34405A's log has it.
    space: bool,
}

#[rustfmt::skip]
const KINDS: &[Kind] = &[
    Kind { vendor: "Agilent Technologies", name: "34405A", dialect: Dialect::Agilent, functions: AGILENT, decimals: 5, space: true },
    Kind { vendor: "Agilent Technologies", name: "34410A", dialect: Dialect::Hp, functions: AGILENT, decimals: 6, space: false },
    Kind { vendor: "Agilent Technologies", name: "34460A", dialect: Dialect::Agilent, functions: AGILENT, decimals: 8, space: false },
    Kind { vendor: "GW", name: "GDM8251A", dialect: Dialect::Gw8200, functions: GW_8200, decimals: 0, space: false },
    Kind { vendor: "GW", name: "GDM8255A", dialect: Dialect::Gw8200, functions: GW_8200, decimals: 0, space: false },
    Kind { vendor: "GWInstek", name: "GDM9060", dialect: Dialect::Gw906x, functions: GW_906X, decimals: 6, space: false },
    Kind { vendor: "GWInstek", name: "GDM9061", dialect: Dialect::Gw906x, functions: GW_906X, decimals: 6, space: false },
    Kind { vendor: "HEWLETT-PACKARD", name: "34401A", dialect: Dialect::Hp, functions: HP_34401A, decimals: 6, space: false },
    Kind { vendor: "Keysight Technologies", name: "34465A", dialect: Dialect::Agilent, functions: AGILENT, decimals: 8, space: false },
    Kind { vendor: "OWON", name: "XDM1041", dialect: Dialect::Owon, functions: OWON, decimals: 0, space: false },
    Kind { vendor: "OWON", name: "XDM2041", dialect: Dialect::Owon, functions: OWON, decimals: 0, space: false },
    Kind { vendor: "Siglent Technologies", name: "SDM3055", dialect: Dialect::Hp, functions: SIGLENT, decimals: 5, space: false },
];

/// `+1.00000E+01`, as the logs have it.
fn sci(value: f64, decimals: usize) -> String {
    let text = format!("{value:+.decimals$E}");
    let (mantissa, exponent) = text.split_once('E').expect("the E of {:E}");
    let exponent: i32 = exponent.parse().expect("an exponent");
    format!("{mantissa}E{exponent:+03}")
}

/// A meter.
pub(super) struct Meter {
    kind: &'static Kind,
    say: Option<String>,
    /// Which of its functions it is on.
    function: usize,
    /// The fixed range of the function it is on, or `None` for auto.
    fixed: Option<f64>,
    initiated: bool,
    /// How many readings it gave.
    taken: u32,
    readings: VecDeque<String>,
    functions: VecDeque<String>,
    rate: String,
    range_text: Option<String>,
    auto_text: Option<String>,
    pace: u32,
    opc_pace: Option<u32>,
    has_opc: bool,
}

impl Meter {
    fn function(&self) -> &'static Function {
        &self.kind.functions[self.function]
    }

    fn dialect_is(&self, dialects: &[Dialect]) -> bool {
        dialects.contains(&self.kind.dialect)
    }

    /// What `top` of a ladder of decades comes to for `wanted`: the least that holds
    /// it, as a request "will use an upper limit which is at least the specified
    /// value" (`protocol.c`).
    fn ladder(top: f64, wanted: f64) -> Option<f64> {
        let mut range = top;
        if wanted > top {
            return None;
        }
        while range / 10.0 >= wanted && range / 10.0 >= top / 1e8 {
            range /= 10.0;
        }
        Some(range)
    }

    /// `CONF:FUNC` and `CONF:FUNC RANGE`.
    fn configure(&mut self, m: &Message<'_>, v: &mut Violations) -> Answer {
        let setup = m.name().strip_prefix("CONF:").unwrap_or_default();
        let Some(at) = self.kind.functions.iter().position(|x| x.setup == setup) else {
            return Answer::Unknown;
        };
        let function = &self.kind.functions[at];
        let fixed = if m.args.is_empty() || m.args.eq_ignore_ascii_case("AUTO") {
            None
        } else if !function.ranged {
            v.report(format!("{}: {setup} has no ranges", m.text));
            return Answer::Done;
        } else {
            match m.args.parse::<f64>() {
                Ok(wanted) if wanted.is_finite() => match Meter::ladder(function.top, wanted) {
                    Some(range) => Some(range),
                    None => return Answer::Error(-222, "Data out of range".to_owned()),
                },
                _ => return Answer::Error(-224, "Illegal parameter value".to_owned()),
            }
        };
        self.function = at;
        self.fixed = fixed;
        self.initiated = false;
        Answer::Done
    }

    /// What a function query says.
    fn function_answer(&mut self) -> String {
        if let Some(text) = self.functions.pop_front() {
            return text;
        }
        let function = self.function();
        match self.kind.dialect {
            Dialect::Gw8200 => function.name.to_owned(),
            Dialect::Owon => format!("\"{}\"", function.name),
            _ if matches!(function.setup, "CONT" | "DIOD") => format!("\"{}\"", function.name),
            _ if function.setup == "TEMP" => {
                "\"TEMP THER,5000,+1.00000E+00,+1.00000E-01\"".to_owned()
            }
            _ => {
                let range = self.fixed.unwrap_or(if function.top > 0.0 { function.top } else { 1.0 });
                let (range, resolution) =
                    (sci(range, self.kind.decimals), sci(range * 1e-5, self.kind.decimals));
                format!("\"{} {range},{resolution}\"", function.name)
            }
        }
    }

    fn reading(&mut self, m: &Message<'_>, v: &mut Violations) -> Answer {
        if self.dialect_is(&[Dialect::Agilent]) && !self.initiated {
            v.report(format!("{}: no INIT ahead of it", m.text));
        }
        let text = self.readings.pop_front().unwrap_or_else(|| {
            let value = self.function().value + 0.001 * f64::from(self.taken % 1000);
            let space = if self.kind.space { " " } else { "" };
            format!("{}{space}", sci(value, 8))
        });
        self.taken += 1;
        if self.pace > 0 { Answer::After(self.pace, vec![text]) } else { Answer::Text(text) }
    }

    /// `FUNC:RANGE?` and `FUNC:RANGE:AUTO?`.
    fn range(&mut self, m: &Message<'_>, v: &mut Violations) -> Answer {
        let name = m.name();
        let (setup, auto) = match name.strip_suffix(":RANGE:AUTO") {
            Some(setup) => (setup, true),
            None => (name.strip_suffix(":RANGE").unwrap_or_default(), false),
        };
        let Some(function) = self.kind.functions.iter().find(|x| x.setup == setup) else {
            return Answer::Unknown;
        };
        if !function.ranged {
            // A real one fails and loses its USB connection (5bf642db).
            v.report(format!("{}: {setup} has no ranges", m.text));
            return Answer::Silent;
        }
        let on_it = self.kind.functions[self.function].setup == setup;
        if auto {
            let text = self.auto_text.clone();
            return Answer::Text(text.unwrap_or_else(|| {
                if on_it && self.fixed.is_some() { "0" } else { "1" }.to_owned()
            }));
        }
        let text = self.range_text.clone();
        Answer::Text(text.unwrap_or_else(|| {
            let range = if on_it { self.fixed.unwrap_or(function.top) } else { function.top };
            sci(range, self.kind.decimals)
        }))
    }
}

impl Instrument for Meter {
    fn idn(&self) -> Option<String> {
        let plain = || format!("{},{},EMU0001,1.0-1.0-1.0", self.kind.vendor, self.kind.name);
        Some(self.say.clone().unwrap_or_else(plain))
    }

    fn settings(&self) -> Settings {
        Settings::default()
    }

    fn command(&mut self, m: &Message<'_>, _s: &mut Settings, v: &mut Violations) -> Answer {
        use Dialect::{Agilent, Gw906x, Gw8200, Hp, Owon};
        let header = m.header.as_str();
        match header {
            "*OPC?" if !self.has_opc => Answer::Silent,
            "*OPC?" => match self.opc_pace.take() {
                Some(ticks) if self.initiated => Answer::After(ticks, vec!["1".to_owned()]),
                keep => {
                    self.opc_pace = keep;
                    Answer::Unknown
                }
            },
            "SYST:LOC" if self.dialect_is(&[Agilent, Gw8200, Gw906x, Owon]) => Answer::Done,
            // As the IEEE 488.2 common command has it, which a range of `10;*RST` sends.
            "*RST" => {
                self.function = self.kind.functions.iter().position(|x| x.setup == "VOLT:DC").unwrap_or(0);
                self.fixed = None;
                self.initiated = false;
                Answer::Done
            }
            "SYST:REM" if self.dialect_is(&[Gw8200, Gw906x, Owon]) => Answer::Done,
            "INIT" if self.dialect_is(&[Agilent, Hp, Gw906x]) => {
                self.initiated = true;
                Answer::Done
            }
            "ABORT" if self.dialect_is(&[Agilent, Hp, Gw906x]) => {
                self.initiated = false;
                Answer::Done
            }
            "CONF?" if self.dialect_is(&[Agilent, Hp, Gw906x]) => {
                Answer::Text(self.function_answer())
            }
            "CONF:STAT:FUNC?" if self.dialect_is(&[Gw8200]) => Answer::Text(self.function_answer()),
            "FUNC?" if self.dialect_is(&[Owon]) => Answer::Text(self.function_answer()),
            "FETCH?" if self.dialect_is(&[Agilent]) => self.reading(m, v),
            "READ?" if self.dialect_is(&[Hp]) => self.reading(m, v),
            "VAL1?" if self.dialect_is(&[Gw8200, Gw906x]) => self.reading(m, v),
            "MEAS1?" if self.dialect_is(&[Owon]) => self.reading(m, v),
            "SENS:DET:RATE?" if self.dialect_is(&[Gw8200, Gw906x]) => {
                Answer::Text(self.rate.clone())
            }
            _ if header.starts_with("CONF:") && !m.is_query() => self.configure(m, v),
            _ if self.dialect_is(&[Agilent])
                && m.is_query()
                && (header.ends_with(":RANGE?") || header.ends_with(":RANGE:AUTO?")) =>
            {
                self.range(m, v)
            }
            _ => Answer::Unknown,
        }
    }
}

/// The meter and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Meter, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut kind = &KINDS[0];
    let mut func = None;
    let mut manual = false;
    let mut opc = None;
    let (mut readings, mut functions) = (VecDeque::new(), VecDeque::new());
    let mut rate = "Slow".to_owned();
    let (mut range_text, mut auto_text) = (None, None);
    let (mut pace, mut opc_pace) = (0, None);
    let mut say = None;
    let list = |v: &str| v.split('|').map(|t| t.replace('_', " ")).collect::<VecDeque<_>>();
    config.option("one-request", "1")?;
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        let number = |range| parse_number(k, v, range);
        match k.as_str() {
            "model" => {
                kind = KINDS
                    .iter()
                    .find(|m| m.name == v)
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model here")))?;
            }
            "func" => func = Some(v.clone()),
            "manual" => manual = number(0..=1)? == 1,
            "readings" => readings = list(v),
            "functions" => functions = list(v),
            "say" => say = Some(v.replace('_', " ")),
            "rate" => rate.clone_from(v),
            "range" => range_text = Some(v.clone()),
            "auto" => auto_text = Some(v.clone()),
            "reading-pace" => pace = number(0..=10_000)?,
            "opc-pace" => opc_pace = Some(number(1..=10_000)?),
            "opc" => opc = Some(number(0..=1)? == 1),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let function = match &func {
        None => kind.functions.iter().position(|x| x.setup == "VOLT:DC").unwrap_or(0),
        Some(name) => kind
            .functions
            .iter()
            .position(|x| x.setup == name)
            .ok_or_else(|| Error::Invalid(format!("func={name}: not on a {}", kind.name)))?,
    };
    let fixed = manual.then(|| {
        let top = kind.functions[function].top;
        if top > 0.0 { top / 10.0 } else { 1.0 }
    });
    let meter = Meter {
        kind,
        say,
        function,
        fixed,
        initiated: false,
        taken: 0,
        readings,
        functions,
        rate,
        range_text,
        auto_text,
        pace,
        opc_pace,
        has_opc: opc.unwrap_or(kind.dialect != Dialect::Owon),
    };
    Ok((meter, config))
}

/// A meter on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (meter, config) = parts(options)?;
    let id = Identity::new(0x0957, 0x0618, "Agilent Technologies", "34405A");
    Ok(config.usbtmc(NAME, &id, meter))
}

/// The same meter on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (meter, config) = parts(options)?;
    Ok(config.tcp(meter))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::tcp::Port;

    fn on_socket(options: &[(&str, &str)]) -> Box<dyn Stream> {
        let pairs: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        build_tcp(&pairs).unwrap()
    }

    /// A line, and what it makes and the violations.
    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (String, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        let bytes = port.sent().concat();
        (String::from_utf8(bytes).unwrap(), port.violations().drain())
    }

    fn says(s: &mut Box<dyn Stream>, line: &str) -> String {
        let (text, flags) = ask(s, line);
        assert_eq!(flags, Vec::<String>::new(), "{line}");
        text
    }

    #[test]
    fn numbers_read_as_the_logs_have_them() {
        assert_eq!(sci(1.0, 5), "+1.00000E+00");
        assert_eq!(sci(1e-5, 5), "+1.00000E-05");
        assert_eq!(sci(90.055_553_1, 8), "+9.00555531E+01");
        assert_eq!(sci(-1e8, 5), "-1.00000E+08");
    }

    #[test]
    fn a_34405a_says_what_its_log_says() {
        let mut s = on_socket(&[("func", "FREQ")]);
        assert_eq!(says(&mut s, "*IDN?\n"), "Agilent Technologies,34405A,EMU0001,1.0-1.0-1.0\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"FREQ +1.00000E+00,+1.00000E-05\"\n");
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "FETCH?\n"), "+9.00555531E+01 \n");
        says(&mut s, "ABORT\n");
    }

    #[test]
    fn functions_change_and_say_so() {
        let mut s = on_socket(&[]);
        assert_eq!(says(&mut s, "CONF?\n"), "\"VOLT +1.00000E+03,+1.00000E-02\"\n");
        says(&mut s, "CONF:FRES\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"FRES +1.00000E+08,+1.00000E+03\"\n");
        says(&mut s, "CONF:DIOD\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"DIOD\"\n");
        says(&mut s, "CONF:CONT\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"CONT\"\n");
        says(&mut s, "CONF:TEMP\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"TEMP THER,5000,+1.00000E+00,+1.00000E-01\"\n");
        says(&mut s, "CONF:VOLT:AC\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"VOLT:AC +7.50000E+02,+7.50000E-03\"\n");
    }

    #[test]
    fn a_range_is_the_least_that_holds_it_and_a_refusal_changes_nothing() {
        let mut s = on_socket(&[]);
        assert_eq!(says(&mut s, "VOLT:DC:RANGE:AUTO?\n"), "1\n");
        says(&mut s, "CONF:VOLT:DC 5\n");
        assert_eq!(says(&mut s, "VOLT:DC:RANGE:AUTO?\n"), "0\n");
        assert_eq!(says(&mut s, "VOLT:DC:RANGE?\n"), "+1.00000E+01\n");
        says(&mut s, "CONF:VOLT:DC 2000\n");
        assert_eq!(says(&mut s, "VOLT:DC:RANGE?\n"), "+1.00000E+01\n", "out of range");
        says(&mut s, "CONF:VOLT:DC volts\n");
        assert_eq!(says(&mut s, "VOLT:DC:RANGE?\n"), "+1.00000E+01\n", "no number");
        assert_eq!(says(&mut s, "SYST:ERR?\n"), "-222,\"Data out of range\"\n");
        assert_eq!(says(&mut s, "SYST:ERR?\n"), "-224,\"Illegal parameter value\"\n");
        says(&mut s, "CONF:VOLT:DC AUTO\n");
        assert_eq!(says(&mut s, "VOLT:DC:RANGE:AUTO?\n"), "1\n");
    }

    #[test]
    fn what_a_range_is_flagged_on() {
        let mut s = on_socket(&[("func", "TEMP")]);
        for line in ["TEMP:RANGE?\n", "TEMP:RANGE:AUTO?\n", "CONF:TEMP 10\n"] {
            let (text, flags) = ask(&mut s, line);
            assert_eq!(text, "", "{line}");
            assert_eq!(flags.len(), 1, "{line}");
        }
        // An HP's dialect has no range at all.
        let mut s = on_socket(&[("model", "34401A")]);
        assert_eq!(ask(&mut s, "VOLT:DC:RANGE?\n").1.len(), 1);
        assert_eq!(ask(&mut s, "CONF:VOLT:DC 10\n").1.len(), 1);
    }

    #[test]
    fn each_dialect_has_its_own_and_flags_the_rest() {
        let mut s = on_socket(&[("model", "34405A")]);
        assert_eq!(ask(&mut s, "READ?\n").1.len(), 1);
        assert_eq!(ask(&mut s, "FETCH?\n").1.len(), 1, "no INIT");
        assert_eq!(ask(&mut s, "VAL1?\n").1.len(), 1);
        assert_eq!(ask(&mut s, "CONF:PER\n").1.len(), 1, "not on this one");

        let mut s = on_socket(&[("model", "34401A")]);
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "READ?\n"), "+1.09450000E-01\n");
        assert_eq!(ask(&mut s, "SYST:LOC\n").1.len(), 1);
        says(&mut s, "CONF:PER\n");
        says(&mut s, "ABORT\n");

        let mut s = on_socket(&[("model", "GDM8251A")]);
        says(&mut s, "*CLS;SYST:REM\n");
        says(&mut s, "SYST:LOC\n");
        assert_eq!(says(&mut s, "CONF:STAT:FUNC?\n"), "01\n");
        says(&mut s, "CONF:FRES\n");
        assert_eq!(says(&mut s, "CONF:STAT:FUNC?\n"), "16\n");
        assert_eq!(says(&mut s, "VAL1?\n"), "+9.95000000E+01\n");
        assert_eq!(says(&mut s, "SENS:DET:RATE?\n"), "Slow\n");
        assert_eq!(ask(&mut s, "INIT\n").1.len(), 1);
        assert_eq!(ask(&mut s, "ABORT\n").1.len(), 1);
        assert_eq!(ask(&mut s, "CONF?\n").1.len(), 1);

        let mut s = on_socket(&[("model", "GDM9060")]);
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "VAL1?\n"), "+1.09450000E-01\n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"VOLT +1.000000E+00,+1.000000E-05\"\n");

        let mut s = on_socket(&[("model", "XDM2041")]);
        assert_eq!(says(&mut s, "FUNC?\n"), "\"VOLT\"\n");
        says(&mut s, "CONF:VOLT:AC\n");
        assert_eq!(says(&mut s, "FUNC?\n"), "\"VOLT AC\"\n");
        assert_eq!(says(&mut s, "MEAS1?\n"), "+1.50000000E+00\n");
        assert_eq!(ask(&mut s, "INIT\n").1.len(), 1);
    }

    #[test]
    fn an_owon_says_nothing_to_opc_unless_told() {
        let mut s = on_socket(&[("model", "XDM2041")]);
        assert_eq!(says(&mut s, "*OPC?\n"), "");
        let mut s = on_socket(&[("model", "XDM2041"), ("opc", "1")]);
        assert_eq!(says(&mut s, "*OPC?\n"), "1\n");
        let mut s = on_socket(&[("opc", "0")]);
        assert_eq!(says(&mut s, "*OPC?\n"), "");
        assert_ne!(says(&mut s, "*IDN?\n"), "");
    }

    #[test]
    fn the_lists_play_once() {
        let mut s = on_socket(&[("readings", "1E0|x"), ("functions", "\"RES\"")]);
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "FETCH?\n"), "1E0\n");
        assert_eq!(says(&mut s, "FETCH?\n"), "x\n");
        assert_eq!(says(&mut s, "FETCH?\n"), "+1.11450000E-01 \n", "the third reading");
        assert_eq!(says(&mut s, "CONF?\n"), "\"RES\"\n");
        assert_ne!(says(&mut s, "CONF?\n"), "\"RES\"\n");
        let options = [("readings", "_1_"), ("functions", "\"VOLT_1\""), ("say", "A_B,C,1,2")];
        let mut s = on_socket(&options);
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "FETCH?\n"), " 1 \n");
        assert_eq!(says(&mut s, "CONF?\n"), "\"VOLT 1\"\n");
        assert_eq!(says(&mut s, "*IDN?\n"), "A B,C,1,2\n");
    }

    #[test]
    fn a_reading_keeps_its_pace_and_so_does_the_first_opc() {
        let mut s = on_socket(&[("reading-pace", "2")]);
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "FETCH?\n"), "", "it takes its time");
        let mut port = Port::default();
        s.tick(&mut port);
        assert_eq!(port.sent().concat(), Vec::<u8>::new());
        s.tick(&mut port);
        assert!(port.sent().concat().starts_with(b"+1.0945"));

        let mut s = on_socket(&[("opc-pace", "1")]);
        assert_eq!(says(&mut s, "*OPC?\n"), "1\n", "before INIT");
        says(&mut s, "INIT\n");
        assert_eq!(says(&mut s, "*OPC?\n"), "");
        let mut port = Port::default();
        s.tick(&mut port);
        assert_eq!(port.sent().concat(), b"1\n");
        assert_eq!(says(&mut s, "*OPC?\n"), "1\n", "the first alone");
    }

    #[test]
    fn it_takes_its_options_on_usb_too() {
        for kind in KINDS {
            let options = [("model".to_owned(), kind.name.to_owned())];
            let m = build(&options).unwrap();
            assert_eq!(m.descriptors().vendor_id(), 0x0957, "one id, so no re-attach");
        }
        assert!(build(&[("model".to_owned(), "9".to_owned())]).is_err());
        assert!(build(&[("func".to_owned(), "NO".to_owned())]).is_err());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
