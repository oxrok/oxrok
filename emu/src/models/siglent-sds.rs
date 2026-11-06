// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Siglent SDS oscilloscopes, on USBTMC and on TCP (`docs/USB.md`, "Writing a SCPI
//! model").
//!
//! The device, and how sure this is. No recording of a real one was found:
//! sigrok-dumps has none here, and no firmware source exists outside Siglent.
//!
//! - From libsigrok's `siglent-sds`: the commands and the three protocols, the block's
//!   prefix of 15 bytes on `C1:WF? ALL` and the descriptor's two fields, the `:INR?`
//!   values, what `SANU?` looks like on each protocol. Those are what C asks and how it
//!   reads the answer, not what a scope was seen to say.
//! - From libsigrok pull request 247 (mike42, Matti Eiden, not merged; "works" reported
//!   on an SDS1104X-E and an SDS1204X-E): `DAT2,#9` and nine digits ahead of `WF? DAT2`,
//!   and two line feeds after a waveform. So 16 bytes ahead of digital data here, where
//!   stock C skips 15: this model follows the report, for a query C has never been
//!   shown to complete.
//! - From the `LeCroy` descriptor C's two offsets belong to: `WAVEDESC` at the start,
//!   346 bytes in all, `COMM_ORDER` low first. C looks at none of it but the two fields.
//! - From memory of Siglent's programming guide, unchecked as it couldn't be fetched
//!   here: `CHDR` taking the header and the unit off answers, `C1:VDIV?` as `1.00E+00`,
//!   `TRSE?` as `EDGE,SR,C1,HT,OFF`, one bit per digital sample, low bit first.
//! - USB ids: `contrib/60-libsigrok.rules`, f4ec:ee38 for the SDS1104X-E and f4ec:ee3a
//!   for the SDS1052DL+. Which of the two the other models have is a guess.
//!
//! Guesses no test should lean on: the bus speeds, firmware strings and serial numbers;
//! that `C1:WF? ALL`'s `#9` length counts the descriptor and the samples, not the line
//! feeds; `FPAR?`, whose only witness is C reading a frame count at byte 40 of it;
//! that `:TIM:OFFS` and `:TRIG:EDGE:LEV`, which read like Rigol's, do nothing here.
//!
//! Emulation artifacts:
//!
//! - Numbers don't round to what a scope can do. `:INR?` says 8193 at once after `ARM`,
//!   unless `inr=` plays another story.
//! - A triangle per channel, in signed codes, with no timebase or trigger in it. A
//!   channel's depth is the same whatever the timebase.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: any name in C's table (default `SDS1104X-E`);
//! - `depth=N`: samples a waveform says it has, and `SANU?` with it (default 700). 0 is
//!   a scope with nothing to say: the descriptor and two line feeds, the one block
//!   stock C can read to its end;
//! - `sanu=TEXT`: what `SANU? C1` answers instead;
//! - `inr=A,B,...`: what `:INR?` plays in turn, once, then as with no option;
//! - `trse=TEXT`: what `TRSE?` answers (`EDGE,SR,C1,HT,OFF`);
//! - `off=1,2`: analog channels off at power on; `la=1`: the logic analyzer on, with
//!   D0 to D7 on;
//! - `frames=N`: the frame count `FPAR?` says (default 2);
//! - `cut=N`: `WF?` ends N bytes before it should, line feeds first. A real
//!   scope's stalled transfer could do this; `cut=363` is what stock C needs to read
//!   a waveform's samples and end;
//! - `prefix=TEXT`: what `WF? ALL` says ahead of `#9`;
//! - `array=N`: a waveform's `WAVE_ARRAY_1` lies, and says N.
//!
//! The pedantic checks: only what the model has, by channel; a query of a `C3` on two
//! channels; a `WF?` of a channel that is off. `D0:WF? DAT2` with the analyzer off is
//! answered: stock C never turns it on, and no source says what a scope does then.

use crate::model::{DeviceModel, Speed, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "siglent-sds";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Siglent SDS scopes on USBTMC and TCP; model=, depth=, sanu=, inr=, trse=, \
                         off=, la=, frames=, cut=, prefix=, array=, and the SCPI shell's mute-at=, \
                         silent-at=, late-at=, opc-late=, mark-at=, idn=, eol=, stall=, in-stall=, \
                         empty-at=, bad-tag-at=, message=";

/// What C's three protocols come to on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Protocol {
    Spo,
    NonSpo,
    Eseries,
}

/// A model of C's table.
struct Kind {
    name: &'static str,
    protocol: Protocol,
    analog: u8,
    digital: bool,
    usb: (u16, u16),
}

const fn kind(name: &'static str, protocol: Protocol, analog: u8, digital: bool) -> Kind {
    let usb = match protocol {
        Protocol::Eseries => (0xf4ec, 0xee38),
        _ => (0xf4ec, 0xee3a),
    };
    Kind { name, protocol, analog, digital, usb }
}

#[rustfmt::skip]
const KINDS: &[Kind] = &[
    kind("SDS1104X-E", Protocol::Eseries, 4, true),
    kind("SDS1204X-E", Protocol::Eseries, 4, true),
    kind("SDS1202X-E", Protocol::Eseries, 2, false),
    kind("SDS1152CML", Protocol::NonSpo, 2, false),
    kind("SDS1102CML", Protocol::NonSpo, 2, false),
    kind("SDS1072CML", Protocol::NonSpo, 2, false),
    kind("SDS1202CNL", Protocol::NonSpo, 2, false),
    kind("SDS1102CNL", Protocol::NonSpo, 2, false),
    kind("SDS1072CNL", Protocol::NonSpo, 2, false),
    kind("SDS1202DL", Protocol::NonSpo, 2, false),
    kind("SDS1102DL", Protocol::NonSpo, 2, false),
    kind("SDS1022DL", Protocol::NonSpo, 2, false),
    kind("SDS1052DL", Protocol::NonSpo, 2, false),
    kind("SDS1052DL+", Protocol::NonSpo, 2, false),
    kind("SDS1102X", Protocol::Spo, 2, false),
    kind("SDS1102X+", Protocol::Spo, 2, false),
    kind("SDS1202X", Protocol::Spo, 2, false),
    kind("SDS1202X+", Protocol::Spo, 2, false),
    kind("SDS2072X", Protocol::Spo, 2, false),
    kind("SDS2074X", Protocol::Spo, 4, false),
    kind("SDS2102X", Protocol::Spo, 2, false),
    kind("SDS2104X", Protocol::Spo, 4, false),
    kind("SDS2202X", Protocol::Spo, 2, false),
    kind("SDS2204X", Protocol::Spo, 4, false),
    kind("SDS2302X", Protocol::Spo, 2, false),
    kind("SDS2304X", Protocol::Spo, 4, false),
];

/// The length of the descriptor, in the `LeCroy` layout.
const DESCRIPTOR: usize = 346;
/// A waveform goes no longer than the driver's buffer, for what a test can ask.
const LONGEST: usize = 1 << 24;

/// A scope.
pub(super) struct Scope {
    kind: &'static Kind,
    /// `CHDR`: answers carry a header and a unit until it is off.
    headers: bool,
    depth: usize,
    sanu: Option<String>,
    inr: std::collections::VecDeque<String>,
    /// What `:INR?` says next with no list: `ARM` sets it, reading clears it.
    pending: u32,
    trse: String,
    off: Vec<u8>,
    la: bool,
    digital_on: [bool; 16],
    frames: u32,
    cut: usize,
    prefix: String,
    array: Option<u32>,
}

/// As a scope prints a number: `5.00E-04`.
fn sci(value: f64) -> String {
    let text = format!("{value:.2E}");
    let (mantissa, exponent) = text.split_once('E').unwrap_or((&text, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let sign = if exponent < 0 { '-' } else { '+' };
    format!("{mantissa}E{sign}{:02}", exponent.abs())
}

/// `2MS` or `500UV`, for what C sends: the number and what its unit multiplies by.
fn with_unit(args: &str, base: &str) -> Option<f64> {
    let body = args.strip_suffix(base)?;
    let (digits, factor) = if let Some(d) = body.strip_suffix('M') {
        (d, 1e-3)
    } else if let Some(d) = body.strip_suffix('U') {
        (d, 1e-6)
    } else if let Some(d) = body.strip_suffix('N') {
        (d, 1e-9)
    } else {
        (body, 1.0)
    };
    digits.parse::<f64>().ok().map(|n| n * factor)
}

/// The channel of a header such as `C2:VDIV`.
fn channel(prefix: char, header: &str) -> Option<(u8, &str)> {
    let rest = header.strip_prefix(prefix)?;
    let (number, name) = rest.split_once(':')?;
    Some((number.parse().ok()?, name))
}

impl Scope {
    /// Answers with a value, wearing its header and unit while `CHDR` is on.
    fn value(&self, m: &Message<'_>, value: &str, unit: &str) -> Answer {
        if self.headers {
            Answer::Text(format!("{} {value}{unit}", m.name()))
        } else {
            Answer::Text(value.to_owned())
        }
    }

    /// `SANU?`, by protocol: a number on an E, points with a suffix on the others.
    fn sample_points(&self) -> String {
        if let Some(text) = &self.sanu {
            return text.clone();
        }
        if self.kind.protocol == Protocol::Eseries {
            return sci(self.depth as f64);
        }
        let depth = self.depth as f64;
        if self.depth >= 1_000_000 {
            format!("{:.2}Mpts", depth / 1e6)
        } else if self.depth >= 1000 {
            format!("{:.2}Kpts", depth / 1e3)
        } else {
            format!("{}pts", self.depth)
        }
    }

    /// The samples of an analog channel: a signed triangle, its own for each channel,
    /// through both signs and all byte classes in a long enough wave.
    fn wave(channel: u8, n: usize) -> Vec<u8> {
        (0..n)
            .map(|i| {
                let t = (i * 3 + usize::from(channel) * 41) % 400;
                let up = if t < 200 { t } else { 400 - t };
                (up as i32 - 100) as i8 as u8
            })
            .collect()
    }

    /// `Cn:WF? ALL`.
    fn waveform(&self, channel: u8) -> Vec<u8> {
        let samples = Scope::wave(channel, self.depth);
        let mut descriptor = vec![0u8; DESCRIPTOR];
        descriptor[..8].copy_from_slice(b"WAVEDESC");
        descriptor[16..23].copy_from_slice(b"WAVEACE");
        descriptor[34] = 1; // COMM_ORDER: low first.
        descriptor[36..40].copy_from_slice(&(DESCRIPTOR as u32).to_le_bytes());
        let array = self.array.unwrap_or(self.depth as u32);
        descriptor[60..64].copy_from_slice(&array.to_le_bytes());
        let mut out = format!("{}#9{:09}", self.prefix, DESCRIPTOR + self.depth).into_bytes();
        out.extend_from_slice(&descriptor);
        out.extend_from_slice(&samples);
        out.extend_from_slice(b"\n\n");
        self.cut_short(out)
    }

    /// `Dn:WF? DAT2`: a bit per sample, the low bit first, a square of its own for
    /// each channel.
    fn digital(&self, channel: u8) -> Vec<u8> {
        let period = usize::from(channel) + 3;
        let bytes: Vec<u8> = (0..self.depth.div_ceil(8))
            .map(|byte| {
                (0..8).fold(0u8, |acc, bit| {
                    let sample = byte * 8 + bit;
                    acc | (u8::from((sample / period).is_multiple_of(2)) << bit)
                })
            })
            .collect();
        let mut out = format!("DAT2,#9{:09}", bytes.len()).into_bytes();
        out.extend_from_slice(&bytes);
        out.extend_from_slice(b"\n\n");
        self.cut_short(out)
    }

    fn cut_short(&self, mut out: Vec<u8>) -> Vec<u8> {
        if self.cut > 0 {
            out.truncate(out.len().saturating_sub(self.cut).max(1));
        }
        out
    }

    /// What is said of analog channel `c`, and false when it has no such channel.
    fn analog(&self, c: u8, m: &Message<'_>, v: &mut Violations) -> bool {
        if (1..=self.kind.analog).contains(&c) {
            true
        } else {
            v.report(format!("{}: no such channel on this model", m.text));
            false
        }
    }

    fn analog_command(
        &mut self,
        c: u8,
        name: &str,
        m: &Message<'_>,
        s: &mut Settings,
        v: &mut Violations,
    ) -> Answer {
        if !self.analog(c, m, v) {
            return Answer::Silent;
        }
        let key = format!("C{c}:{name}");
        match name {
            "WF?" => {
                if m.args != "ALL" {
                    v.report(format!("{}: asked for what the model doesn't have", m.text));
                    return Answer::Silent;
                }
                if s.get(&format!("C{c}:TRA")) == Some("OFF") {
                    v.report(format!("{}: the channel is off", m.text));
                    return Answer::Silent;
                }
                Answer::Bytes(self.waveform(c))
            }
            "VDIV" => match with_unit(m.args, "V") {
                Some(volts) => {
                    s.set(&key, sci(volts));
                    Answer::Done
                }
                None => Answer::Error(-224, "Illegal parameter value".to_owned()),
            },
            "ATTN" | "CPL" | "OFST" | "TRA" | "TRSL" | "TRLV" => {
                s.set(&key, m.args);
                Answer::Done
            }
            "VDIV?" | "OFST?" | "TRLV?" => {
                let value = s.get(&key.replace('?', "")).unwrap_or("0.00E+00").to_owned();
                self.value(m, &value, "V")
            }
            "ATTN?" | "CPL?" | "TRA?" | "TRSL?" => {
                let value = s.get(&key.replace('?', "")).unwrap_or("").to_owned();
                self.value(m, &value, "")
            }
            _ => Answer::Unknown,
        }
    }

    fn digital_command(&mut self, n: u8, name: &str, m: &Message<'_>, v: &mut Violations) -> Answer {
        if !self.kind.digital || n > 15 {
            v.report(format!("{}: no such digital channel on this model", m.text));
            return Answer::Silent;
        }
        match name {
            "TRA" => {
                self.digital_on[usize::from(n)] = m.args == "ON";
                Answer::Done
            }
            "TRA?" => {
                let on = if self.digital_on[usize::from(n)] { "ON" } else { "OFF" };
                self.value(m, on, "")
            }
            "WF?" => {
                if m.args != "DAT2" {
                    v.report(format!("{}: asked for what the model doesn't have", m.text));
                    return Answer::Silent;
                }
                Answer::Bytes(self.digital(n))
            }
            _ => Answer::Unknown,
        }
    }

    /// `FPAR?`: only its frame count at byte 40 is known.
    fn frame_parameters(&self) -> Vec<u8> {
        let mut out = vec![0u8; 200];
        out[40..44].copy_from_slice(&self.frames.to_le_bytes());
        out
    }
}

impl Instrument for Scope {
    fn idn(&self) -> Option<String> {
        Some(format!("Siglent Technologies,{},SDSEMU0001,7.6.1.15", self.kind.name))
    }

    fn settings(&self) -> Settings {
        let mut s = Settings::default()
            .with("TDIV", "5.00E-04")
            .with("WFSU", "SP,0,NP,0,FP,0")
            .with("ACQW", "SAMPLING");
        for c in 1..=self.kind.analog {
            let on = if self.off.contains(&c) { "OFF" } else { "ON" };
            s.set(&format!("C{c}:TRA"), on);
            s.set(&format!("C{c}:ATTN"), "1");
            s.set(&format!("C{c}:VDIV"), "1.00E+00");
            s.set(&format!("C{c}:OFST"), "0.00E+00");
            s.set(&format!("C{c}:CPL"), "D1M");
            s.set(&format!("C{c}:TRSL"), "POS");
            s.set(&format!("C{c}:TRLV"), "0.00E+00");
        }
        s
    }

    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let header = m.header.as_str();
        if let Some((c, name)) = channel('C', header)
            && header != "CHDR"
        {
            let name = name.to_owned();
            return self.analog_command(c, &name, m, s, v);
        }
        if let Some((n, name)) = channel('D', header)
            && !header.starts_with("DI:")
        {
            let name = name.to_owned();
            return self.digital_command(n, &name, m, v);
        }
        match header {
            "CHDR" => {
                self.headers = m.args != "OFF";
                Answer::Done
            }
            "TDIV?" => {
                let value = s.get("TDIV").unwrap_or("").to_owned();
                self.value(m, &value, "S")
            }
            "TDIV" => match with_unit(m.args, "S") {
                Some(seconds) => {
                    s.set("TDIV", sci(seconds));
                    Answer::Done
                }
                None => Answer::Error(-224, "Illegal parameter value".to_owned()),
            },
            "DI:SW?" => {
                if !self.kind.digital {
                    v.report(format!("{}: no analyzer on this model", m.text));
                    return Answer::Silent;
                }
                self.value(m, if self.la { "ON" } else { "OFF" }, "")
            }
            "DI:SW" | "DGST" => {
                if !self.kind.digital {
                    v.report(format!("{}: no analyzer on this model", m.text));
                    return Answer::Silent;
                }
                self.la = m.args == "ON";
                Answer::Done
            }
            "SANU?" => {
                let text = self.sample_points();
                if m.args == "C1" || (m.args == "D0" && self.kind.digital) {
                    self.value(m, &text, "")
                } else {
                    v.report(format!("{}: no such source on this model", m.text));
                    Answer::Silent
                }
            }
            "TRSE?" => self.value(m, &self.trse.clone(), ""),
            "TRSE" => {
                self.trse = format!("{},HT,OFF", m.args.trim_end_matches(",OFF"));
                Answer::Done
            }
            "ARM" => {
                self.pending = 8193;
                Answer::Done
            }
            "INR?" => {
                let value = match self.inr.pop_front() {
                    Some(text) => text,
                    None => std::mem::take(&mut self.pending).to_string(),
                };
                self.value(m, &value, "")
            }
            "FPAR?" => Answer::Bytes(self.frame_parameters()),
            // As C sends them: nothing here says what a real one does with them.
            "FRAM" | "TIM:OFFS" | "TRIG:EDGE:LEV" => Answer::Done,
            _ => {
                // A source's slope and level, of `EX`, `EX5` or `LINE`.
                if let Some((source, name)) = header.split_once(':')
                    && matches!(source, "EX" | "EX5" | "LINE" | "CH1" | "CH2" | "D0")
                {
                    return match name {
                        "TRSL?" => self.value(m, "POS", ""),
                        "TRSL" => Answer::Done,
                        _ => Answer::Unknown,
                    };
                }
                Answer::Unknown
            }
        }
    }
}

/// The scope and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Scope, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut scope = Scope {
        kind: &KINDS[0],
        headers: true,
        depth: 700,
        sanu: None,
        inr: std::collections::VecDeque::new(),
        pending: 8193,
        trse: "EDGE,SR,C1,HT,OFF".to_owned(),
        off: Vec::new(),
        la: false,
        digital_on: [false; 16],
        frames: 2,
        cut: 0,
        prefix: "ALL,".to_owned(),
        array: None,
    };
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        let number = |range| parse_number(k, v, range);
        match k.as_str() {
            "model" => {
                scope.kind = KINDS
                    .iter()
                    .find(|m| m.name == v)
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model here")))?;
            }
            "depth" => scope.depth = number(0..=LONGEST as u32)? as usize,
            "sanu" => scope.sanu = Some(v.clone()),
            "inr" => scope.inr = v.split(',').map(str::to_owned).collect(),
            "trse" => scope.trse.clone_from(v),
            "off" => {
                scope.off = v
                    .split(',')
                    .map(|c| parse_number(k, c, 1..=4).map(|c: u32| c as u8))
                    .collect::<Result<_>>()?;
            }
            "la" => {
                scope.la = number(0..=1)? == 1;
                for on in &mut scope.digital_on[..8] {
                    *on = scope.la;
                }
            }
            "frames" => scope.frames = number(0..=1000)?,
            "cut" => scope.cut = number(0..=LONGEST as u32)? as usize,
            "prefix" => scope.prefix.clone_from(v),
            "array" => scope.array = Some(number(0..=u32::MAX)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok((scope, config))
}

/// A scope on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (scope, config) = parts(options)?;
    let (vendor, product) = scope.kind.usb;
    let mut id = Identity::new(vendor, product, "Siglent Technologies", scope.kind.name);
    id.speed = if scope.kind.protocol == Protocol::NonSpo { Speed::Full } else { Speed::High };
    Ok(config.usbtmc(NAME, &id, scope))
}

/// The same scope on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (scope, config) = parts(options)?;
    Ok(config.tcp(scope))
}

/// A waveform is long: its digest in the log.
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

    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (Vec<u8>, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        let bytes = port.sent().concat();
        (bytes, port.violations().drain())
    }

    #[test]
    fn headers_stay_on_until_chdr_is_off() {
        let mut s = on_socket(&[]);
        assert_eq!(ask(&mut s, "C1:VDIV?\n").0, b"C1:VDIV 1.00E+00V\n".to_vec());
        ask(&mut s, "CHDR OFF\n");
        assert_eq!(ask(&mut s, "C1:VDIV?\n").0, b"1.00E+00\n");
        assert_eq!(ask(&mut s, ":TDIV?\n").0, b"5.00E-04\n");
    }

    #[test]
    fn a_setting_comes_back_as_a_scope_prints_it() {
        let mut s = on_socket(&[]);
        ask(&mut s, "CHDR OFF\n");
        ask(&mut s, "TDIV 2MS\n");
        assert_eq!(ask(&mut s, ":TDIV?\n").0, b"2.00E-03\n");
        ask(&mut s, "C2:VDIV 500UV\n");
        assert_eq!(ask(&mut s, "C2:VDIV?\n").0, b"5.00E-04\n");
        ask(&mut s, "TRSE EDGE,SR,C2,OFF\n");
        assert_eq!(ask(&mut s, "TRSE?\n").0, b"EDGE,SR,C2,HT,OFF\n");
    }

    #[test]
    fn a_waveform_has_its_prefix_descriptor_and_line_feeds() {
        let mut s = on_socket(&[("depth", "400")]);
        let (block, flags) = ask(&mut s, "C1:WF? ALL\n");
        assert_eq!(flags, Vec::<String>::new());
        assert_eq!(&block[..15], b"ALL,#9000000746");
        assert_eq!(&block[15..23], b"WAVEDESC");
        let d = &block[15..];
        assert_eq!(u32::from_le_bytes(d[36..40].try_into().unwrap()), 346);
        assert_eq!(u32::from_le_bytes(d[60..64].try_into().unwrap()), 400);
        assert_eq!(block.len(), 15 + 346 + 400 + 2);
        assert_eq!(&block[block.len() - 2..], b"\n\n");
    }

    #[test]
    fn nothing_to_say_is_exactly_what_c_reads_first() {
        let mut s = on_socket(&[("depth", "0")]);
        assert_eq!(ask(&mut s, "C1:WF? ALL\n").0.len(), 363);
    }

    #[test]
    fn a_cut_ends_early_with_no_line_feeds() {
        let mut s = on_socket(&[("depth", "400"), ("cut", "363")]);
        let block = ask(&mut s, "C1:WF? ALL\n").0;
        assert_eq!(block.len(), 400, "what stock C reads whole");
    }

    #[test]
    fn digital_data_is_a_bit_per_sample_with_the_longer_prefix() {
        let mut s = on_socket(&[("depth", "16"), ("la", "1")]);
        let block = ask(&mut s, "D0:WF? DAT2\n").0;
        assert_eq!(&block[..16], b"DAT2,#9000000002");
        assert_eq!(block.len(), 16 + 2 + 2);
        // A period of 3: three high, three low.
        assert_eq!(block[16], 0b1100_0111);
    }

    #[test]
    fn what_the_scope_lacks_is_flagged() {
        let mut s = on_socket(&[("model", "SDS1202X-E")]);
        assert_eq!(ask(&mut s, "C3:VDIV?\n").1.len(), 1);
        assert_eq!(ask(&mut s, "DI:SW?\n").1.len(), 1, "no analyzer");
        assert_eq!(ask(&mut s, "D0:TRA ON\n").1.len(), 1);
        assert_eq!(ask(&mut s, "NOPE\n").1.len(), 1);
        let mut s = on_socket(&[("off", "2")]);
        assert_eq!(ask(&mut s, "C2:WF? ALL\n").1.len(), 1, "off");
        assert_eq!(ask(&mut s, "C1:WF? DAT2\n").1.len(), 1, "not asked of an analog one");
        let mut s = on_socket(&[]);
        assert_eq!(ask(&mut s, "D0:WF? ALL\n").1.len(), 1, "not asked of a digital one");
        assert_eq!(ask(&mut s, "SANU? C9\n").1.len(), 1);
        let (_, flags) = ask(&mut s, "EX:NOPE\n");
        assert_eq!(flags.len(), 1);
    }

    #[test]
    fn inr_clears_at_a_read_and_arm_sets_it() {
        let mut s = on_socket(&[]);
        ask(&mut s, "CHDR OFF\n");
        ask(&mut s, "ARM\n");
        assert_eq!(ask(&mut s, ":INR?\n").0, b"8193\n");
        assert_eq!(ask(&mut s, ":INR?\n").0, b"0\n");
        let mut s = on_socket(&[("inr", "8192,0,1")]);
        ask(&mut s, "CHDR OFF\n");
        assert_eq!(ask(&mut s, ":INR?\n").0, b"8192\n");
        assert_eq!(ask(&mut s, ":INR?\n").0, b"0\n");
        assert_eq!(ask(&mut s, ":INR?\n").0, b"1\n");
    }

    #[test]
    fn sample_points_by_protocol() {
        let mut s = on_socket(&[("depth", "14000000")]);
        ask(&mut s, "CHDR OFF\n");
        assert_eq!(ask(&mut s, "SANU? C1\n").0, b"1.40E+07\n");
        let mut s = on_socket(&[("depth", "7000"), ("model", "SDS1202X")]);
        ask(&mut s, "CHDR OFF\n");
        assert_eq!(ask(&mut s, "SANU? C1\n").0, b"7.00Kpts\n");
        let mut s = on_socket(&[("depth", "14000000"), ("model", "SDS1202X")]);
        ask(&mut s, "CHDR OFF\n");
        assert_eq!(ask(&mut s, "SANU? C1\n").0, b"14.00Mpts\n");
        let mut s = on_socket(&[("sanu", "odd")]);
        ask(&mut s, "CHDR OFF\n");
        assert_eq!(ask(&mut s, "SANU? C1\n").0, b"odd\n");
    }

    #[test]
    fn fpar_says_its_frames_at_byte_40() {
        let mut s = on_socket(&[("frames", "7")]);
        let block = ask(&mut s, "FPAR?\n").0;
        assert_eq!(block.len(), 200);
        assert_eq!(&block[40..44], &[7, 0, 0, 0]);
    }

    #[test]
    fn the_wave_goes_through_both_signs() {
        let wave = Scope::wave(1, 700);
        assert!(wave.iter().any(|b| (*b as i8) < 0));
        assert!(wave.iter().any(|b| (*b as i8) > 0));
        assert_ne!(Scope::wave(1, 50), Scope::wave(2, 50));
    }

    #[test]
    fn it_answers_its_own_options_on_usb_too() {
        for k in KINDS {
            let options = [("model".to_owned(), k.name.to_owned())];
            let m = build(&options).unwrap();
            assert_eq!(m.descriptors().vendor_id(), k.usb.0);
            assert_eq!(m.descriptors().product_id(), k.usb.1);
        }
        assert!(build(&[("model".to_owned(), "SDS9".to_owned())]).is_err());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
