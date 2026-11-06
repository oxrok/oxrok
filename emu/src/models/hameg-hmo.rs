// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Hameg and Rohde & Schwarz HMO, RTC, RTB, RTM, RTA and RTH oscilloscopes, on
//! USBTMC and on TCP, with analog channels and logic pods.
//!
//! The device, and how sure this is:
//!
//! - Everything here comes from what libsigrok's driver sends and what its
//!   comments say of answers. No recording of any of these scopes was found (none in
//!   `sigrok-dumps` or the oracle's tree), and no answer was compared with a real
//!   one. `tests/emu/hameg-hmo/` shows the C driver and the port agree about this
//!   model, not that either agrees with a scope.
//! - The commit that added these models calls the RTB2000, RTM3000 and RTA4000
//!   "untested" (aac3063, b720f16), so the C's second dialect, `:LOG%d:STAT` for a
//!   channel and for a pod, `:DIG%d:TECH`, is itself a reading of a manual. A real
//!   RTB's answer to `:LOG0:STAT?` is unknown. This model takes what C sends: one
//!   value per `LOGn:STAT`, so `LOG1:STAT` is D1's and pod 1's at once, as the C's
//!   table has it.
//! - USB id 0aad:0118 is one of the three that `contrib/60-libsigrok.rules` gives "HMO
//!   series ... VCP/USBTMC mode". Which of them is USBTMC is a guess, and using it for
//!   the other series too is a choice, as a change of id costs 5.5 s a case.
//! - The strings, serial number, firmware and bus speed are stand-ins.
//!
//! Emulation artifacts:
//!
//! - A block ends in a line feed by default, as IEEE 488.2's `NL^END` has it. C reads
//!   past it or not by how a read falls. `block-lf=0` leaves it off.
//! - Numbers answer as they were set, in the driver's form. A real scope reformats.
//! - A triangle per channel, a counter per pod, each a little on per read, with no
//!   trigger, noise or timebase in it. A real scope gives what it has on screen, and
//!   no driver path here starts or stops it.
//! - `:ACQ:SRAT?` is `points / (timebase * divisions)`, halved with more than two
//!   analog channels on, only so that a stale rate shows in a `--get`.
//! - A pattern with no closing quote takes no effect and queues error -151. C sends one
//!   at 20 characters: what a real scope does with that is not known.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: any name in C's `scope_models`; `HMO3054` by default;
//! - `points=N`: samples per channel and pod (1000); `points2=N`: pod 2's alone;
//! - `set=HEADER=VALUE`, as often as needed: what a header holds at power on, right
//!   or wrong, for the answers a driver must take or refuse;
//! - `block-lf=0|1`; `digits=N`, a block length's digits (as few as fit);
//! - counted in data reads, from 1: `cut-at=N`, half of what the block announces;
//!   `zero-at=N`, `#10`; `junk-at=N`, a line of text; `indefinite-at=N`, `#0`;
//!   `odd-at=N`, a block one byte longer than a whole number of samples;
//!   `bad-len-at=N`, a block whose length is no number.
//!
//! The pedantic checks: a setting out of the model's tables; a timebase or vertical
//! scale not in the table of this model; a read of a channel or pod that is off; a read
//! before `:FORM` says the format it needs; `CH3` read with pod 1 on, `CH4` with pod 2;
//! a set of what is read only. What the model has no header for is the engine's
//! violation, so `:LOG1:DATA?` on an HMO and `:POD1:DATA?` on an RTB are flagged.

use crate::model::{DeviceModel, Speed, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings, block};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hameg-hmo";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Hameg and R&S HMO, RTB, RTM, RTA, RTH scopes on USBTMC and TCP; model=, \
                         points=, points2=, set=HEADER=VALUE, block-lf=, digits=, cut-at=, \
                         zero-at=, junk-at=, indefinite-at=, odd-at=, bad-len-at=, remote=, and \
                         the SCPI shell's options";

const HAMEG: &str = "HAMEG";
const ROHDE: &str = "Rohde&Schwarz";

const COUPLING: &[&str] = &["AC", "ACL", "DC", "DCL", "GND"];
const COUPLING_RTB: &[&str] = &["ACL", "DCL", "GND"];
const COUPLING_RTM: &[&str] = &["ACL", "DC", "DCL", "GND"];
const COUPLING_RTH: &[&str] = &["ACL", "DCL"];
const THRESHOLD: &[&str] = &["TTL", "ECL", "CMOS", "USER1", "USER2"];
const THRESHOLD_MAN: &[&str] = &["TTL", "ECL", "CMOS", "MAN"];
const BUS: &[&str] = &["LINE", "EXT", "PATT", "BUS1", "BUS2"];
const SBUS: &[&str] = &["LINE", "EXT", "PATT", "SBUS1", "SBUS2"];
const NO_BUS: &[&str] = &[];

/// A series, as far as it shows on the wire.
struct Kind {
    names: &'static [&'static str],
    analog: u8,
    digital: u8,
    /// `:LOGn:DATA?` and `:DIGn:TECH`, not `:PODn:DATA?` and `:PODn:THR`.
    log_dialect: bool,
    coupling: &'static [&'static str],
    threshold: &'static [&'static str],
    /// The threshold's number is the pod's, from 1, not its first channel's.
    threshold_for_pod: bool,
    sources: &'static [&'static str],
    /// No 1 ns.
    compact: bool,
    /// Up to 50 V a division, not 10 V.
    vdiv_50: bool,
}

#[rustfmt::skip]
const KINDS: &[Kind] = &[
    Kind { names: &["HMO722", "HMO1022", "HMO1522", "HMO2022"], analog: 2, digital: 8, log_dialect: false, coupling: COUPLING, threshold: THRESHOLD, threshold_for_pod: true, sources: BUS, compact: true, vdiv_50: false },
    Kind { names: &["RTC1002", "HMO1002", "HMO1202"], analog: 2, digital: 8, log_dialect: false, coupling: COUPLING, threshold: THRESHOLD, threshold_for_pod: true, sources: BUS, compact: false, vdiv_50: false },
    Kind { names: &["HMO3032", "HMO3042", "HMO3052", "HMO3522"], analog: 2, digital: 16, log_dialect: false, coupling: COUPLING, threshold: THRESHOLD, threshold_for_pod: true, sources: BUS, compact: false, vdiv_50: false },
    Kind { names: &["HMO724", "HMO1024", "HMO1524", "HMO2024"], analog: 4, digital: 8, log_dialect: false, coupling: COUPLING, threshold: THRESHOLD, threshold_for_pod: true, sources: BUS, compact: true, vdiv_50: false },
    Kind { names: &["HMO2524", "HMO3034", "HMO3044", "HMO3054", "HMO3524"], analog: 4, digital: 16, log_dialect: false, coupling: COUPLING, threshold: THRESHOLD, threshold_for_pod: true, sources: BUS, compact: false, vdiv_50: false },
    Kind { names: &["RTB2002"], analog: 2, digital: 16, log_dialect: true, coupling: COUPLING_RTB, threshold: THRESHOLD_MAN, threshold_for_pod: false, sources: SBUS, compact: false, vdiv_50: true },
    Kind { names: &["RTB2004"], analog: 4, digital: 16, log_dialect: true, coupling: COUPLING_RTB, threshold: THRESHOLD_MAN, threshold_for_pod: false, sources: SBUS, compact: false, vdiv_50: true },
    Kind { names: &["RTM3002"], analog: 2, digital: 16, log_dialect: true, coupling: COUPLING_RTM, threshold: THRESHOLD_MAN, threshold_for_pod: false, sources: SBUS, compact: false, vdiv_50: false },
    Kind { names: &["RTM3004"], analog: 4, digital: 16, log_dialect: true, coupling: COUPLING_RTM, threshold: THRESHOLD_MAN, threshold_for_pod: false, sources: SBUS, compact: false, vdiv_50: false },
    Kind { names: &["RTA4004"], analog: 4, digital: 16, log_dialect: true, coupling: COUPLING_RTM, threshold: THRESHOLD_MAN, threshold_for_pod: false, sources: SBUS, compact: false, vdiv_50: false },
    Kind { names: &["RTH1002"], analog: 2, digital: 8, log_dialect: true, coupling: COUPLING_RTH, threshold: THRESHOLD, threshold_for_pod: true, sources: NO_BUS, compact: false, vdiv_50: true },
    Kind { names: &["RTH1004"], analog: 4, digital: 8, log_dialect: true, coupling: COUPLING_RTH, threshold: THRESHOLD, threshold_for_pod: true, sources: NO_BUS, compact: false, vdiv_50: true },
];

/// The mantissas of a 1-2-5 table.
const STEPS: [f64; 3] = [1.0, 2.0, 5.0];

/// Whether `value` is a step of a 1-2-5 table from `low` up to `high` inclusive.
fn on_table(value: f64, low: f64, high: f64) -> bool {
    let mut decade = low;
    while decade <= high * 1.001 {
        for step in STEPS {
            let entry = step * decade;
            if entry <= high * 1.001 && (entry - value).abs() <= entry * 1e-4 {
                return true;
            }
        }
        decade *= 10.0;
    }
    false
}

/// `value` as `%.3E` has it, which is a scope's way: `1.000E-03`.
fn sci(value: f64) -> String {
    let text = format!("{value:.3E}");
    let (mantissa, exponent) = text.split_once('E').unwrap_or((&text, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    format!("{mantissa}E{}{:02}", if exponent < 0 { '-' } else { '+' }, exponent.abs())
}

/// What goes wrong at a data read, by count.
#[derive(Clone, Copy, Default)]
struct DataFaults {
    cut_at: Option<u32>,
    zero_at: Option<u32>,
    junk_at: Option<u32>,
    indefinite_at: Option<u32>,
    odd_at: Option<u32>,
    bad_len_at: Option<u32>,
}

/// A scope.
pub(super) struct Scope {
    kind: &'static Kind,
    name: String,
    vendor: &'static str,
    points: usize,
    points2: usize,
    power_on: Vec<(String, String)>,
    block_lf: bool,
    digits: Option<usize>,
    faults: DataFaults,
    /// Data reads so far.
    reads: u32,
}

impl Scope {
    fn pods(&self) -> u8 {
        self.kind.digital / 8
    }

    /// The number a threshold's command has for a pod, from 1.
    fn threshold_number(&self, pod: u8) -> u32 {
        if self.kind.threshold_for_pod { u32::from(pod) } else { u32::from(pod - 1) * 8 }
    }

    fn is_source(&self, name: &str) -> bool {
        let numbered = |prefix: &str, first: u8, count: u8| {
            name.strip_prefix(prefix)
                .and_then(|n| n.parse::<u8>().ok())
                .is_some_and(|n| n >= first && n < first + count)
        };
        numbered("CH", 1, self.kind.analog)
            || numbered("D", 0, self.kind.digital)
            || self.kind.sources.contains(&name)
    }

    /// A block as the faults at this read have it.
    fn framed(&mut self, data: &[u8]) -> Answer {
        self.reads += 1;
        let at = Some(self.reads);
        let end: &[u8] = if self.block_lf { b"\n" } else { b"" };
        let f = self.faults;
        if f.junk_at == at {
            return Answer::Text("NOT A BLOCK".to_owned());
        }
        if f.zero_at == at {
            return Answer::Bytes(block(1, b"", end));
        }
        if f.indefinite_at == at {
            let mut out = b"#0".to_vec();
            out.extend_from_slice(data);
            out.extend_from_slice(b"\n");
            return Answer::Bytes(out);
        }
        if f.bad_len_at == at {
            let mut out = b"#4x000".to_vec();
            out.extend_from_slice(data);
            return Answer::Bytes(out);
        }
        let digits = self.digits.unwrap_or_else(|| data.len().to_string().len());
        if f.cut_at == at {
            let mut out = block(digits, data, b"");
            out.truncate(out.len() - data.len() / 2);
            return Answer::Bytes(out);
        }
        if f.odd_at == at {
            let mut long = data.to_vec();
            long.push(0x55);
            return Answer::Bytes(block(digits, &long, end));
        }
        Answer::Bytes(block(digits, data, end))
    }

    fn format_is(s: &Settings, want: &str) -> bool {
        s.get("FORM").is_some_and(|f| f.eq_ignore_ascii_case(want))
    }

    /// `:CHANn:DATA?`: a triangle over the screen, as floats.
    fn analog_data(&mut self, m: &Message<'_>, n: u8, s: &Settings, v: &mut Violations) -> Answer {
        if s.get(&format!("CHAN{n}:STAT")) != Some("1") {
            v.report(format!("{}: the channel is off", m.text));
        }
        if !Scope::format_is(s, "REAL,32") {
            v.report(format!("{}: :FORM is not REAL,32", m.text));
        }
        if n >= 3 && self.pod_on(n - 2, s) {
            v.report(format!("{}: pod {} is on, and shares its inputs", m.text, n - 2));
        }
        let big = s.get("FORM:BORD") == Some("MSBF");
        let scale = s.number(&format!("CHAN{n}:SCAL")) as f32;
        let phase = self.reads as usize * 7 + usize::from(n) * 37;
        let mut data = Vec::with_capacity(self.points * 4);
        for i in 0..self.points {
            let t = (i * 3 + phase) % 400;
            let up = if t < 200 { t } else { 400 - t };
            let value = (up as f32 - 100.0) / 100.0 * 3.0 * scale;
            data.extend_from_slice(&if big { value.to_be_bytes() } else { value.to_le_bytes() });
        }
        self.framed(&data)
    }

    fn pod_on(&self, pod: u8, s: &Settings) -> bool {
        let header =
            if self.kind.log_dialect { format!("LOG{pod}:STAT") } else { format!("POD{pod}:STAT") };
        s.get(&header) == Some("1")
    }

    /// `:PODn:DATA?` or `:LOGn:DATA?`: a counter, one byte a sample.
    fn pod_data(&mut self, m: &Message<'_>, pod: u8, s: &Settings, v: &mut Violations) -> Answer {
        if !self.pod_on(pod, s) {
            v.report(format!("{}: the pod is off", m.text));
        }
        if !Scope::format_is(s, "UINT,8") {
            v.report(format!("{}: :FORM is not UINT,8", m.text));
        }
        let count = if pod == 2 { self.points2 } else { self.points };
        let from = self.reads as usize * 5;
        let data: Vec<u8> =
            (0..count).map(|i| ((from + i) * usize::from(pod * 2 - 1) & 0xff) as u8).collect();
        self.framed(&data)
    }

    /// A number between two limits, in the driver's form.
    fn check_number(m: &Message<'_>, low: f64, high: f64, v: &mut Violations) {
        if !m.args.parse::<f64>().is_ok_and(|x| (low..=high).contains(&x)) {
            v.report(format!("{}: not a number from {low} to {high}", m.text));
        }
    }

    fn check_listed(m: &Message<'_>, list: &[&str], v: &mut Violations) {
        if !list.contains(&m.args) {
            v.report(format!("{}: not one of {list:?}", m.text));
        }
    }

    /// A setting's value, on a channel or a pod: what a table says. The engine does the
    /// rest, as its settings hold the header or don't.
    fn check_setting(&self, m: &Message<'_>, v: &mut Violations) {
        let header = m.header.as_str();
        match header {
            "TIM:SCAL" => {
                let low = if self.kind.compact { 2e-9 } else { 1e-9 };
                let ok = m.args.parse().is_ok_and(|x| {
                    on_table(x, 1e-9, 50.0) && (x >= low * 0.999) && x <= 50.0 * 1.001
                });
                if !ok {
                    v.report(format!("{}: no such timebase on {}", m.text, self.name));
                }
            }
            "TIM:DIV" => v.report(format!("{}: it is read only", m.text)),
            "TRIG:A:SOUR" => {
                if !self.is_source(m.args) {
                    v.report(format!("{}: no such source on {}", m.text, self.name));
                }
            }
            "TRIG:A:EDGE:SLOP" => Scope::check_listed(m, &["POS", "NEG", "EITH"], v),
            "TRIG:A:TYPE" => Scope::check_listed(m, &["EDGE", "LOGIC"], v),
            "ACQ:HRES" | "ACQ:PEAK" => Scope::check_listed(m, &["AUTO", "OFF"], v),
            "FORM" => Scope::check_listed(m, &["REAL,32", "UINT,8"], v),
            "FORM:BORD" => Scope::check_listed(m, &["LSBF", "MSBF"], v),
            "TIM:POS" => Scope::check_number(m, -1e4, 1e4, v),
            _ if header.ends_with(":STAT") => Scope::check_listed(m, &["0", "1"], v),
            _ if header.ends_with(":COUP") => Scope::check_listed(m, self.kind.coupling, v),
            _ if header.ends_with(":SCAL") => {
                let high = if self.kind.vdiv_50 { 50.0 } else { 10.0 };
                if !m.args.parse().is_ok_and(|x| on_table(x, 1e-3, high)) {
                    v.report(format!("{}: no such scale on {}", m.text, self.name));
                }
            }
            _ if header.ends_with(":THR") && !self.kind.log_dialect => {
                Scope::check_listed(m, self.kind.threshold, v);
            }
            _ if header.ends_with(":TECH") => Scope::check_listed(m, self.kind.threshold, v),
            _ if header.ends_with(":THR") || header.contains(":THR:UDL") => {
                Scope::check_number(m, -2.0, 8.0, v);
            }
            _ => {}
        }
    }

    /// `:ACQ:SRAT?`.
    fn sample_rate(&self, s: &Settings) -> String {
        let on = (1..=self.kind.analog)
            .filter(|c| s.get(&format!("CHAN{c}:STAT")) == Some("1"))
            .count();
        let sweep = s.number("TIM:SCAL") * s.number("TIM:DIV");
        let rate = self.points as f64 / sweep / if on > 2 { 2.0 } else { 1.0 };
        sci(rate)
    }
}

/// A numbered header's number: `CHAN2:DATA?` of `CHAN`, `:DATA?` is 2.
fn numbered(header: &str, prefix: &str, suffix: &str) -> Option<u8> {
    header.strip_prefix(prefix)?.strip_suffix(suffix)?.parse().ok()
}

impl Instrument for Scope {
    fn idn(&self) -> Option<String> {
        Some(format!("{},{},EMU0001,05.886", self.vendor, self.name))
    }

    fn settings(&self) -> Settings {
        let mut s = Settings::default()
            .with("TIM:SCAL", "1.000E-03")
            .with("TIM:DIV", "12")
            .with("TIM:POS", "0.000E+00")
            .with("ACQ:HRES", "OFF")
            .with("ACQ:PEAK", "OFF")
            .with("TRIG:A:SOUR", "CH1")
            .with("TRIG:A:TYPE", "EDGE")
            .with("TRIG:A:EDGE:SLOP", "POS")
            .with("TRIG:A:PATT:FUNC", "AND")
            .with("TRIG:A:PATT:COND", "\"TRUE\"")
            .with("TRIG:A:PATT:MODE", "OFF")
            .with("TRIG:A:PATT:SOUR", "\"XXXX\"")
            .with("FORM", "ASC,0")
            .with("FORM:BORD", "MSBF");
        for c in 1..=self.kind.analog {
            s.set(&format!("CHAN{c}:STAT"), if c <= 2 { "1" } else { "0" });
            s.set(&format!("CHAN{c}:SCAL"), "1.000E+00");
            s.set(&format!("CHAN{c}:POS"), "0.000E+00");
            s.set(&format!("CHAN{c}:COUP"), "DCL");
            s.set(&format!("PROB{c}:SET:ATT:UNIT"), "V");
        }
        for d in 0..self.kind.digital {
            s.set(&format!("LOG{d}:STAT"), "0");
        }
        for pod in 1..=self.pods() {
            let n = self.threshold_number(pod);
            if self.kind.log_dialect {
                s.set(&format!("LOG{pod}:STAT"), "0");
                s.set(&format!("DIG{n}:TECH"), "TTL");
                s.set(&format!("DIG{n}:THR"), "1.400E+00");
            } else {
                s.set(&format!("POD{pod}:STAT"), "0");
                s.set(&format!("POD{n}:THR"), "TTL");
                s.set(&format!("POD{n}:THR:UDL1"), "1.400E+00");
                s.set(&format!("POD{n}:THR:UDL2"), "1.400E+00");
            }
        }
        for (header, value) in &self.power_on {
            s.set(header, value.clone());
        }
        s
    }

    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let header = m.header.as_str();
        if let Some(n) = numbered(header, "CHAN", ":DATA?").filter(|n| *n <= self.kind.analog) {
            return self.analog_data(m, n, s, v);
        }
        let pod_prefix = if self.kind.log_dialect { "LOG" } else { "POD" };
        if let Some(pod) = numbered(header, pod_prefix, ":DATA?").filter(|p| *p <= self.pods()) {
            return self.pod_data(m, pod, s, v);
        }
        if header == "ACQ:SRAT?" {
            return Answer::Text(self.sample_rate(s));
        }
        if m.is_query() {
            return Answer::Unknown;
        }
        if header == "TRIG:A:PATT:SOUR"
            && (m.args.len() < 2 || !m.args.starts_with('"') || !m.args.ends_with('"'))
        {
            return Answer::Error(-151, "Invalid string data".to_owned());
        }
        self.check_setting(m, v);
        Answer::Unknown
    }
}

/// The scope and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Scope, scpi::Config, bool)> {
    let mut config = scpi::Config::default();
    let mut name = "HMO3054".to_owned();
    let mut points = 1000;
    let mut points2 = None;
    let mut power_on = Vec::new();
    let mut block_lf = true;
    let mut digits = None;
    let mut faults = DataFaults::default();
    let mut remote = false;
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        let number = |range| parse_number(k, v, range);
        match k.as_str() {
            "model" => name.clone_from(v),
            "points" => points = number(0..=1 << 24)? as usize,
            "points2" => points2 = Some(number(0..=1 << 24)? as usize),
            "set" => {
                let (header, value) = v
                    .split_once('=')
                    .ok_or_else(|| Error::Invalid(format!("set={v}: as in TIM:DIV=10")))?;
                power_on.push((header.to_owned(), value.to_owned()));
            }
            "block-lf" => block_lf = number(0..=1)? == 1,
            "digits" => digits = Some(number(1..=9)? as usize),
            "cut-at" => faults.cut_at = Some(number(1..=1 << 24)?),
            "zero-at" => faults.zero_at = Some(number(1..=1 << 24)?),
            "junk-at" => faults.junk_at = Some(number(1..=1 << 24)?),
            "indefinite-at" => faults.indefinite_at = Some(number(1..=1 << 24)?),
            "odd-at" => faults.odd_at = Some(number(1..=1 << 24)?),
            "bad-len-at" => faults.bad_len_at = Some(number(1..=1 << 24)?),
            "remote" => remote = number(0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let kind = KINDS
        .iter()
        .find(|m| m.names.contains(&name.as_str()))
        .ok_or_else(|| Error::Invalid(format!("model={name}: no such model here")))?;
    let vendor = if name.starts_with("HMO") { HAMEG } else { ROHDE };
    let scope = Scope {
        kind,
        name,
        vendor,
        points,
        points2: points2.unwrap_or(points),
        power_on,
        block_lf,
        digits,
        faults,
        reads: 0,
    };
    Ok((scope, config, remote))
}

/// A scope on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (scope, config, remote) = parts(options)?;
    let mut id = Identity::new(0x0aad, 0x0118, ROHDE, "Oscilloscope");
    id.speed = Speed::High;
    id.remote = remote;
    Ok(config.usbtmc(NAME, &id, scope))
}

/// The same scope on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (scope, config, _) = parts(options)?;
    Ok(config.tcp(scope))
}

/// A block is long: its digest in the log.
pub fn log_rules(_options: &[(String, String)]) -> Rules {
    Rules::digest_above(HASH_ABOVE)
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

    /// A line, and the bytes and the violations it makes.
    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (Vec<u8>, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        let bytes = port.sent().concat();
        (bytes, port.violations().drain())
    }

    /// The line is taken and nothing is flagged.
    fn clean(s: &mut Box<dyn Stream>, line: &str) -> Vec<u8> {
        let (bytes, flags) = ask(s, &format!("{line}\n"));
        assert_eq!(flags, Vec::<String>::new(), "{line}");
        bytes
    }

    fn flagged(s: &mut Box<dyn Stream>, line: &str) {
        let (_, flags) = ask(s, &format!("{line}\n"));
        assert_eq!(flags.len(), 1, "{line}: {flags:?}");
    }

    const ANALOG: &str = ":FORM:BORD LSBF;:FORM REAL,32;:CHAN1:DATA?";

    #[test]
    fn it_says_who_it_is() {
        let mut s = on_socket(&[]);
        assert_eq!(clean(&mut s, "*IDN?"), b"HAMEG,HMO3054,EMU0001,05.886\n");
        let mut s = on_socket(&[("model", "RTB2004")]);
        assert_eq!(clean(&mut s, "*IDN?"), b"Rohde&Schwarz,RTB2004,EMU0001,05.886\n");
        assert!(build_tcp(&[("model".to_owned(), "HMO9".to_owned())]).is_err());
        assert!(build_tcp(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }

    #[test]
    fn every_name_in_the_table_builds_on_usb_too() {
        for kind in KINDS {
            for name in kind.names {
                let options = [("model".to_owned(), (*name).to_owned())];
                let m = build(&options).unwrap();
                assert_eq!(m.descriptors().vendor_id(), 0x0aad);
            }
        }
    }

    #[test]
    fn a_block_is_floats_in_the_order_asked_for() {
        let mut s = on_socket(&[("points", "4")]);
        let little = clean(&mut s, ANALOG);
        assert_eq!(&little[..4], b"#216");
        assert_eq!(little.len(), 4 + 16 + 1);
        assert_eq!(little.last(), Some(&b'\n'));
        let value = f32::from_le_bytes(little[4..8].try_into().unwrap());
        assert!(value.abs() <= 3.0, "{value}");
        let big = clean(&mut s, ":FORM:BORD MSBF;:FORM REAL,32;:CHAN2:DATA?");
        let value = f32::from_be_bytes(big[4..8].try_into().unwrap());
        assert!(value.abs() <= 3.0, "{value}");
    }

    #[test]
    fn the_scale_shows_in_the_data() {
        let mut s = on_socket(&[("points", "8")]);
        clean(&mut s, ":CHAN1:SCAL 1.000000E-03");
        let block = clean(&mut s, ANALOG);
        let value = f32::from_le_bytes(block[4..8].try_into().unwrap());
        assert!(value.abs() <= 3e-3, "{value}");
    }

    #[test]
    fn a_read_needs_its_format_and_its_channel_on() {
        let mut s = on_socket(&[("points", "4")]);
        flagged(&mut s, ":CHAN1:DATA?");
        flagged(&mut s, ":FORM REAL,32;:CHAN3:DATA?");
        flagged(&mut s, ":FORM UINT,8;:POD1:DATA?");
        clean(&mut s, ":POD1:STAT 1");
        assert_eq!(clean(&mut s, ":FORM UINT,8;:POD1:DATA?").len(), 3 + 4 + 1);
        flagged(&mut s, ":FORM REAL,32;:POD1:DATA?");
    }

    #[test]
    fn channels_three_and_four_share_inputs_with_the_pods() {
        let mut s = on_socket(&[("points", "4")]);
        clean(&mut s, ":CHAN3:STAT 1");
        clean(&mut s, ":POD1:STAT 1");
        flagged(&mut s, ":FORM REAL,32;:CHAN3:DATA?");
        clean(&mut s, ":CHAN4:STAT 1");
        clean(&mut s, ":FORM REAL,32;:CHAN4:DATA?");
        clean(&mut s, ":POD2:STAT 1");
        flagged(&mut s, ":FORM REAL,32;:CHAN4:DATA?");
    }

    #[test]
    fn the_dialects_keep_to_their_own_commands() {
        let mut s = on_socket(&[("model", "RTB2004"), ("points", "4")]);
        flagged(&mut s, ":POD1:STAT?");
        flagged(&mut s, ":POD1:THR?");
        assert_eq!(clean(&mut s, ":DIG0:TECH?"), b"TTL\n");
        assert_eq!(clean(&mut s, ":DIG8:TECH?"), b"TTL\n");
        flagged(&mut s, ":DIG1:TECH?");
        clean(&mut s, ":LOG1:STAT 1");
        assert_eq!(clean(&mut s, ":LOG1:STAT?"), b"1\n");
        assert_eq!(clean(&mut s, ":FORM UINT,8;:LOG1:DATA?").len(), 3 + 4 + 1);
        let mut s = on_socket(&[("model", "RTH1002")]);
        assert_eq!(clean(&mut s, ":DIG1:TECH?"), b"TTL\n");
        flagged(&mut s, ":DIG0:TECH?");
        flagged(&mut s, ":LOG8:STAT?");
        let mut s = on_socket(&[]);
        flagged(&mut s, ":LOG1:DATA?");
        flagged(&mut s, ":DIG0:TECH?");
        assert_eq!(clean(&mut s, ":POD2:THR:UDL2?"), b"1.400E+00\n");
    }

    #[test]
    fn what_a_model_lacks_is_flagged() {
        let mut s = on_socket(&[("model", "HMO1002")]);
        flagged(&mut s, ":CHAN3:STAT?");
        flagged(&mut s, ":LOG8:STAT?");
        flagged(&mut s, ":POD2:STAT?");
        flagged(&mut s, ":NO:SUCH?");
    }

    #[test]
    fn settings_keep_to_the_tables_of_the_model() {
        let mut s = on_socket(&[("model", "HMO1024")]);
        flagged(&mut s, ":TIM:SCAL 1.000000E-09");
        clean(&mut s, ":TIM:SCAL 2.000000E-09");
        clean(&mut s, ":TIM:SCAL 5.000000E+01");
        flagged(&mut s, ":TIM:SCAL 3.000000E-03");
        flagged(&mut s, ":TIM:SCAL 1.000000E+02");
        flagged(&mut s, ":TIM:DIV 10");
        flagged(&mut s, ":ACQ:SRAT 10");
        flagged(&mut s, ":CHAN1:SCAL 2.000000E+01");
        clean(&mut s, ":CHAN1:SCAL 1.000000E+01");
        flagged(&mut s, ":TRIG:A:SOUR D8");
        flagged(&mut s, ":TRIG:A:SOUR SBUS1");
        clean(&mut s, ":TRIG:A:SOUR D7");
        flagged(&mut s, ":TRIG:A:SOUR CH5");
        flagged(&mut s, ":TRIG:A:EDGE:SLOP UP");
        flagged(&mut s, ":TRIG:A:TYPE VIDEO");
        flagged(&mut s, ":ACQ:HRES ON");
        flagged(&mut s, ":ACQ:PEAK 1");
        flagged(&mut s, ":FORM ASC,0");
        flagged(&mut s, ":FORM:BORD LE");
        flagged(&mut s, ":TIM:POS soon");
        flagged(&mut s, ":CHAN1:STAT 2");
        flagged(&mut s, ":CHAN1:COUP XX");
        flagged(&mut s, ":POD1:THR MAN");
        flagged(&mut s, ":POD1:THR:UDL2 9.000000E+00");
        clean(&mut s, ":POD1:THR USER2");
        let mut s = on_socket(&[("model", "RTB2002")]);
        clean(&mut s, ":CHAN1:SCAL 5.000000E+01");
        flagged(&mut s, ":CHAN1:COUP DC");
        flagged(&mut s, ":DIG0:TECH USER2");
        clean(&mut s, ":DIG0:TECH MAN");
        flagged(&mut s, ":DIG0:THR nine");
        clean(&mut s, ":DIG0:THR 1.000000E+00");
    }

    #[test]
    fn a_pattern_must_close_its_quote() {
        let mut s = on_socket(&[]);
        clean(&mut s, ":TRIG:A:PATT:SOUR \"10X\"");
        assert_eq!(clean(&mut s, ":TRIG:A:PATT:SOUR?"), b"\"10X\"\n");
        clean(&mut s, ":TRIG:A:PATT:SOUR \"10X1");
        assert_eq!(clean(&mut s, ":TRIG:A:PATT:SOUR?"), b"\"10X\"\n", "taken as nothing");
        assert!(clean(&mut s, "SYST:ERR?").starts_with(b"-151,"));
    }

    #[test]
    fn the_rate_follows_the_settings() {
        let mut s = on_socket(&[("points", "1200")]);
        assert_eq!(clean(&mut s, ":ACQ:SRAT?"), b"1.000E+05\n");
        clean(&mut s, ":TIM:SCAL 1.000000E-06");
        assert_eq!(clean(&mut s, ":ACQ:SRAT?"), b"1.000E+08\n");
        clean(&mut s, ":CHAN3:STAT 1");
        assert_eq!(clean(&mut s, ":ACQ:SRAT?"), b"5.000E+07\n");
    }

    #[test]
    fn a_power_on_value_is_what_the_option_says() {
        let mut s = on_socket(&[("set", "CHAN1:COUP=BOGUS")]);
        assert_eq!(clean(&mut s, ":CHAN1:COUP?"), b"BOGUS\n");
    }

    #[test]
    fn the_faults_come_at_their_read() {
        let on = ":CHAN1:STAT 1;:FORM REAL,32;:CHAN1:DATA?";
        let mut s = on_socket(&[("points", "4"), ("cut-at", "1"), ("zero-at", "2")]);
        assert_eq!(clean(&mut s, on), {
            let mut b = b"#216".to_vec();
            b.extend_from_slice(&clean(&mut on_socket(&[("points", "4")]), on)[4..12]);
            b
        });
        assert_eq!(clean(&mut s, on), b"#10\n");
        assert_eq!(clean(&mut s, on).len(), 4 + 16 + 1);
        let mut s = on_socket(&[("points", "4"), ("junk-at", "1"), ("indefinite-at", "2")]);
        assert_eq!(clean(&mut s, on), b"NOT A BLOCK\n");
        assert_eq!(&clean(&mut s, on)[..2], b"#0");
        let mut s = on_socket(&[("points", "4"), ("odd-at", "1"), ("bad-len-at", "2")]);
        assert_eq!(&clean(&mut s, on)[..4], b"#217");
        assert_eq!(&clean(&mut s, on)[..6], b"#4x000");
        let mut s = on_socket(&[("points", "4"), ("block-lf", "0"), ("digits", "9")]);
        let block = clean(&mut s, on);
        assert_eq!(&block[..11], b"#9000000016");
        assert_eq!(block.len(), 11 + 16);
    }

    #[test]
    fn the_second_pod_can_differ() {
        let mut s = on_socket(&[("points", "4"), ("points2", "6")]);
        clean(&mut s, ":POD1:STAT 1;:POD2:STAT 1;:FORM UINT,8");
        assert_eq!(clean(&mut s, ":POD1:DATA?").len(), 3 + 4 + 1);
        assert_eq!(clean(&mut s, ":POD2:DATA?").len(), 3 + 6 + 1);
    }

    #[test]
    fn numbers_read_as_a_scope_writes_them() {
        assert_eq!(sci(1e-3), "1.000E-03");
        assert_eq!(sci(1.25e9), "1.250E+09");
        assert_eq!(sci(0.0), "0.000E+00");
    }
}
