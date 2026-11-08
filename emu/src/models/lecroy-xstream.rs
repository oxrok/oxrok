// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! `LeCroy` X-Stream oscilloscopes, on USBTMC and on TCP.
//!
//! The device, and how sure this is:
//!
//! - The commands are what libsigrok's `lecroy-xstream` sends. The forms of the
//!   answers are from the maker's "Remote Control Manual" for its LT and `WaveRunner`
//!   series (LTXXX-RCM-E Rev B, January 2002), the one found: `*IDN LECROY,...` with
//!   its header, `ON`, `A1M`, a `WAVEFORM?` of `#9` and a line end after the block,
//!   and a `WAIT` that waits for ever. That manual is of an older series than
//!   X-Stream, and nothing here was compared with a real scope.
//! - The descriptor is that manual's template, `LECROY_2_3`, 346 bytes, little endian
//!   as the driver reads it. Its bytes are written per field, from the manual's
//!   offsets, not from the driver's struct.
//! - The manual has `ALL,` ahead of a block; what libsigrok reads since its 2019
//!   commit 24931412 is the block at once. So the block comes at once, and
//!   `prefix=ALL,` shows the other.
//! - `TRIG POS 1.0e-3 S` is in no manual found (it has `TRIG_DELAY`), and a real scope
//!   may refuse it. It is what the driver sends, so it is taken: a model that flagged
//!   it would leave the path with no case.
//!
//! Guesses, which no test should lean on: the USB ids (05ff as the maker's, and a product id
//! from no source), the bus speed, the model name and the serial number.
//!
//! Emulation artifacts:
//!
//! - `*IDN?` keeps its header after `COMM_HEADER OFF`, which only a second probe in a
//!   session sees.
//! - Settings answer as `%.3E`, the form of the driver's comment; a real scope has
//!   its own number of digits.
//! - A triangle per channel and per acquisition, with no trigger in it.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `points=N`: the samples of a waveform (default 200);
//! - `empty=N`: the first N `WAVEFORM?` have no samples, as a scope with no
//!   acquisition yet (default 0);
//! - `sweep=T`: what follows a `WAIT` waits T ticks, an acquisition's time;
//! - `vertunit=TEXT`, `offset=VOLTS`: of the descriptor;
//! - `template=TEXT`, `comm-order=N`, `comm-type=N`: of the descriptor, and the
//!   order and the size of what is sent go by the last two;
//! - `usertext=N`, `trigtime=N`, `ristime=N`: the bytes of those blocks;
//! - `prefix=TEXT`: ahead of each block;
//! - `no-eol=1`: no line end after a block;
//! - `claim=N`: the descriptor's count of samples, with the block as it was;
//! - `cut=N`: the Nth `WAVEFORM?` says its full length and sends half;
//! - `head=TEXT`: the Nth of `head-at=N` answers this instead of its block (`#0`,
//!   `#9000000000`, `junk`), and its answer is bytes as they are;
//! - `desc-len=N`: what the descriptor says of its own length;
//! - `tiny=N`: the block holds only N bytes of the descriptor;
//! - `trace=BITS`: which channels are on, as `1100` (default all);
//! - `eol-late=T`: a block's line end comes T ticks after it, as a socket's peer can
//!   send it;
//! - `wrong-source=N`: the Nth `WAVEFORM?` says another channel in `WAVE_SOURCE`;
//! - `trace-later=BITS`: the channels on from the fifth `TRACE?` on, the scan having
//!   asked four, as a scope someone changed between a scan and an open;
//! - `vdiv=`, `tdiv=`, `coupling=`, `trse=`, `slope=`, `trdl=`, `opos=`: what those
//!   queries answer at power on; `opos=` is `OFFSET?`.
//!
//! The pedantic checks: only what the model has, and a `WAVEFORM?` before the driver
//! set `COMM_HEADER OFF` and `COMM_FORMAT DEF9,WORD,BIN`, or of a channel that is off.

use crate::model::{DeviceModel, Speed, Violations};
use crate::options::parse_number;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings, block};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "lecroy-xstream";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "LeCroy X-Stream scopes on USBTMC and TCP; points=, empty=, sweep=, \
                         vertunit=, offset=, template=, comm-order=, comm-type=, usertext=, \
                         trigtime=, ristime=, prefix=, no-eol=, claim=, cut=, head=, head-at=, \
                         desc-len=, tiny=, eol-late=, wrong-source=, trace=, trace-later=, vdiv=, tdiv=, coupling=, trse=, slope=, \
                         trdl=, opos=, and the SCPI shell's mute-at=, silent-at=, late-at=, \
                         opc-late=, mark-at=, idn=, eol=, stall=, in-stall=, empty-at=, \
                         bad-tag-at=, message=";

/// The length of the manual's descriptor.
const DESCRIPTOR: usize = 346;
/// The most of one block a host's options can ask for.
const MOST_BYTES: u32 = 1 << 24;
/// What a division is in a word: 65536 counts on eight divisions.
const COUNTS_PER_DIV: f64 = 8192.0;

/// A number as the driver's comment has it: `2.000E-03`.
fn sci(value: f64) -> String {
    let text = format!("{value:.3E}");
    let (mantissa, exponent) = text.split_once('E').unwrap_or((&text, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let sign = if exponent < 0 { '-' } else { '+' };
    format!("{mantissa}E{sign}{:02}", exponent.abs())
}

/// What a descriptor and a block hold, from the options.
#[derive(Clone)]
struct Shape {
    points: usize,
    vertunit: String,
    offset: f32,
    template: String,
    comm_order: u16,
    comm_type: u16,
    usertext: usize,
    trigtime: usize,
    ristime: usize,
    prefix: String,
    eol: bool,
    claim: Option<u32>,
    cut: Option<u32>,
    head: Option<String>,
    head_at: u32,
    desc_len: Option<u32>,
    tiny: Option<usize>,
    eol_late: u32,
    wrong_source: Option<u32>,
}

/// What stands in at power on, from the options.
#[derive(Clone)]
struct Power {
    trace: Vec<bool>,
    trace_later: Option<Vec<bool>>,
    vdiv: String,
    tdiv: String,
    coupling: String,
    trse: String,
    slope: String,
    trdl: String,
    opos: String,
}

/// A scope.
pub(super) struct Scope {
    shape: Shape,
    power: Power,
    empty: u32,
    sweep: u32,
    /// `ARM` and `WAIT` came, and nothing has waited for that yet.
    waiting: bool,
    acquisition: u32,
    reads: u32,
    trace_queries: u32,
}

impl Scope {
    /// One channel's samples of one acquisition: a triangle, within a division or so
    /// either way, and no two channels or acquisitions alike.
    fn wave(&self, channel: u8) -> Vec<i16> {
        (0..self.shape.points)
            .map(|i| {
                let phase =
                    (i * 37 + usize::from(channel) * 811 + self.acquisition as usize * 97) % 8000;
                let up = if phase < 4000 { phase } else { 8000 - phase };
                (up as i32 * 3 - 6000) as i16
            })
            .collect()
    }

    /// The bytes of a field in the order the descriptor says.
    fn put(&self, out: &mut [u8], at: usize, little: &[u8]) {
        let mut bytes = little.to_vec();
        if self.shape.comm_order == 0 {
            bytes.reverse();
        }
        out[at..at + bytes.len()].copy_from_slice(&bytes);
    }

    /// The whole of a `WAVEFORM?`, as the manual's template lays it out.
    fn waveform(&self, channel: u8, gain: f32, empty: bool) -> Vec<u8> {
        let shape = &self.shape;
        let points = if empty { 0 } else { shape.points };
        let size = if shape.comm_type == 0 { 1 } else { 2 };
        let mut d = vec![0u8; DESCRIPTOR];
        d[..8].copy_from_slice(b"WAVEDESC");
        let template = shape.template.as_bytes();
        d[16..16 + template.len().min(16)].copy_from_slice(&template[..template.len().min(16)]);
        self.put(&mut d, 32, &shape.comm_type.to_le_bytes());
        self.put(&mut d, 34, &shape.comm_order.to_le_bytes());
        let own = shape.desc_len.unwrap_or(DESCRIPTOR as u32);
        self.put(&mut d, 36, &own.to_le_bytes());
        self.put(&mut d, 40, &(shape.usertext as u32).to_le_bytes());
        self.put(&mut d, 48, &(shape.trigtime as u32).to_le_bytes());
        self.put(&mut d, 52, &(shape.ristime as u32).to_le_bytes());
        self.put(&mut d, 60, &((points * size) as u32).to_le_bytes());
        d[76..84].copy_from_slice(b"EMU-XSTR");
        let label = format!("C{channel}");
        d[96..96 + label.len()].copy_from_slice(label.as_bytes());
        let count = shape.claim.unwrap_or(points as u32);
        self.put(&mut d, 116, &count.to_le_bytes());
        self.put(&mut d, 120, &(points as u32).to_le_bytes());
        self.put(&mut d, 156, &gain.to_le_bytes());
        self.put(&mut d, 160, &shape.offset.to_le_bytes());
        self.put(&mut d, 172, &16u16.to_le_bytes());
        self.put(&mut d, 176, &1e-9f32.to_le_bytes());
        // WAVE_SOURCE, the 2004 template's last field: channel 1 is 0.
        let mut source = u16::from(channel) - 1;
        if shape.wrong_source == Some(self.reads) {
            source = (source + 1) % 4;
        }
        self.put(&mut d, 344, &source.to_le_bytes());
        let unit = shape.vertunit.as_bytes();
        d[196..196 + unit.len().min(47)].copy_from_slice(&unit[..unit.len().min(47)]);
        d[244] = b'S';
        if let Some(tiny) = shape.tiny {
            d.truncate(tiny);
            return d;
        }
        let mut out = d;
        out.resize(out.len() + shape.usertext, b'T');
        // What is in these two blocks would read as big samples.
        out.resize(out.len() + shape.trigtime, 0x7e);
        out.resize(out.len() + shape.ristime, 0x7d);
        if !empty {
            for sample in self.wave(channel) {
                if shape.comm_type == 0 {
                    out.push((sample >> 8) as u8);
                } else if shape.comm_order == 0 {
                    out.extend_from_slice(&sample.to_be_bytes());
                } else {
                    out.extend_from_slice(&sample.to_le_bytes());
                }
            }
        }
        out
    }

    /// `Cn:WAVEFORM?`.
    fn answer_waveform(
        &mut self,
        channel: u8,
        m: &Message<'_>,
        s: &Settings,
        v: &mut Violations,
    ) -> Answer {
        if s.get("COMM_HEADER") != Some("OFF") || s.get("COMM_FORMAT") != Some("DEF9,WORD,BIN") {
            v.report(format!("{}: before COMM_HEADER OFF and COMM_FORMAT DEF9,WORD,BIN", m.text));
        }
        if s.get(&format!("C{channel}:TRACE")) != Some("ON") {
            v.report(format!("{}: the channel is off", m.text));
        }
        self.reads += 1;
        let empty = self.empty > 0;
        self.empty = self.empty.saturating_sub(1);
        let late = if std::mem::take(&mut self.waiting) { self.sweep } else { 0 };
        let vdiv = s.number(&format!("C{channel}:VDIV"));
        let gain = (vdiv / COUNTS_PER_DIV) as f32;
        let shape = &self.shape;
        let bytes = if let Some(head) = shape.head.as_ref().filter(|_| self.reads == shape.head_at)
        {
            head.clone().into_bytes()
        } else {
            let mut wave = self.waveform(channel, gain, empty);
            let whole = wave.len();
            let eol: &[u8] = if shape.eol { b"\n" } else { b"" };
            let mut out = shape.prefix.clone().into_bytes();
            let cut = shape.cut == Some(self.reads);
            if cut {
                wave.truncate(whole / 2);
            }
            let framed = block(9, &wave, eol);
            if cut {
                // Says what it should have sent, and leaves it at that.
                out.extend_from_slice(format!("#9{whole:09}").as_bytes());
                out.extend_from_slice(&wave);
            } else {
                out.extend_from_slice(&framed);
            }
            out
        };
        if shape.eol_late > 0 && bytes.last() == Some(&b'\n') && shape.head.is_none() {
            let mut bytes = bytes;
            bytes.pop();
            return Answer::BytesApart(vec![bytes, b"\n".to_vec()]);
        }
        if late > 0 { Answer::BytesAfter(late, bytes) } else { Answer::Bytes(bytes) }
    }
}

/// The channel of a `Cn:...` header.
fn channel_of(header: &str, tail: &str) -> Option<u8> {
    let n: u8 = header.strip_prefix('C')?.strip_suffix(tail)?.parse().ok()?;
    (1..=4).contains(&n).then_some(n)
}

impl Instrument for Scope {
    fn idn(&self) -> Option<String> {
        Some("*IDN LECROY,WR8104,LCRY0001N00001,8.6.0".to_owned())
    }

    fn settings(&self) -> Settings {
        let p = &self.power;
        let mut s = Settings::default()
            .with("COMM_HEADER", "SHORT")
            .with("COMM_FORMAT", "OFF,WORD,BIN")
            .with("TIME_DIV", p.tdiv.clone())
            .with("TRIG_SELECT", p.trse.clone())
            .with("TRIG_DELAY", p.trdl.clone())
            .with("TRIG", "POS 0");
        for c in 1..=4 {
            let on = p.trace.get(c - 1).copied().unwrap_or(true);
            s.set(&format!("C{c}:TRACE"), if on { "ON" } else { "OFF" });
            s.set(&format!("C{c}:VDIV"), p.vdiv.clone());
            s.set(&format!("C{c}:OFFSET"), p.opos.clone());
            s.set(&format!("C{c}:COUPLING"), p.coupling.clone());
        }
        for source in ["C1", "C2", "C3", "C4", "LINE", "EXT", "EX", "EX10", "PA", "ETM10"] {
            s.set(&format!("{source}:TRIG_SLOPE"), p.slope.clone());
        }
        s
    }

    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let header = m.header.as_str();
        if channel_of(header, ":TRACE?").is_some() {
            self.trace_queries += 1;
            if self.trace_queries == 5
                && let Some(later) = &self.power.trace_later
            {
                for (i, on) in later.iter().enumerate() {
                    s.set(&format!("C{}:TRACE", i + 1), if *on { "ON" } else { "OFF" });
                }
            }
            return Answer::Unknown;
        }
        if let Some(channel) = channel_of(header, ":WAVEFORM?") {
            return self.answer_waveform(channel, m, s, v);
        }
        if header == "ARM" {
            self.acquisition += 1;
            return Answer::Done;
        }
        if header == "WAIT" {
            self.waiting = true;
            return Answer::Done;
        }
        // A scope says numbers its own way, whatever way they came.
        if !m.is_query() && (header == "TIME_DIV" || channel_of(header, ":VDIV").is_some()) {
            let Ok(value) = m.args.parse::<f64>() else {
                return Answer::Error(-224, format!("{}: no number", m.text));
            };
            s.set(header, sci(value));
            return Answer::Done;
        }
        Answer::Unknown
    }
}

/// The scope and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Scope, scpi::Config)> {
    let mut config = scpi::Config::default();
    // Nothing says what the firmware does with a request while one waits.
    config.option("one-request", "1")?;
    let mut shape = Shape {
        points: 200,
        vertunit: "V".to_owned(),
        offset: 0.0,
        template: "LECROY_2_3".to_owned(),
        comm_order: 1,
        comm_type: 1,
        usertext: 0,
        trigtime: 0,
        ristime: 0,
        prefix: String::new(),
        eol: true,
        claim: None,
        cut: None,
        head: None,
        head_at: 1,
        desc_len: None,
        tiny: None,
        eol_late: 0,
        wrong_source: None,
    };
    let mut power = Power {
        trace: vec![true; 4],
        trace_later: None,
        vdiv: "1.000E+00".to_owned(),
        tdiv: "1.000E-03".to_owned(),
        coupling: "D1M".to_owned(),
        trse: "EDGE,SR,C1,HT,OFF".to_owned(),
        slope: "POS".to_owned(),
        trdl: "0".to_owned(),
        opos: "0".to_owned(),
    };
    let (mut empty, mut sweep) = (0, 0);
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        let bytes = |range| parse_number(k, v, range);
        match k.as_str() {
            "points" => shape.points = bytes(0..=MOST_BYTES / 2)? as usize,
            "empty" => empty = bytes(0..=1000)?,
            "sweep" => sweep = bytes(0..=100_000)?,
            "vertunit" => shape.vertunit.clone_from(v),
            "offset" => {
                shape.offset =
                    v.parse().map_err(|_| Error::Invalid(format!("offset={v}: a number")))?;
            }
            "template" => shape.template.clone_from(v),
            "comm-order" => shape.comm_order = bytes(0..=255)? as u16,
            "comm-type" => shape.comm_type = bytes(0..=255)? as u16,
            "usertext" => shape.usertext = bytes(0..=MOST_BYTES)? as usize,
            "trigtime" => shape.trigtime = bytes(0..=MOST_BYTES)? as usize,
            "ristime" => shape.ristime = bytes(0..=MOST_BYTES)? as usize,
            "prefix" => shape.prefix.clone_from(v),
            "no-eol" => shape.eol = bytes(0..=1)? == 0,
            "claim" => shape.claim = Some(bytes(0..=u32::MAX)?),
            "cut" => shape.cut = Some(bytes(1..=MOST_BYTES)?),
            "head" => shape.head = Some(v.clone()),
            "head-at" => shape.head_at = bytes(1..=MOST_BYTES)?,
            "desc-len" => shape.desc_len = Some(bytes(0..=u32::MAX)?),
            "eol-late" => {
                // The line end goes as a second piece, the shell's line gap after.
                shape.eol_late = bytes(1..=1 << 16)?;
                config.option("line-gap", v)?;
            }
            "wrong-source" => shape.wrong_source = Some(bytes(1..=MOST_BYTES)?),
            "tiny" => shape.tiny = Some(bytes(0..=DESCRIPTOR as u32)? as usize),
            "trace" => power.trace = v.chars().map(|c| c == '1').collect(),
            "trace-later" => power.trace_later = Some(v.chars().map(|c| c == '1').collect()),
            "vdiv" => power.vdiv.clone_from(v),
            "tdiv" => power.tdiv.clone_from(v),
            "coupling" => power.coupling.clone_from(v),
            "trse" => power.trse.clone_from(v),
            "slope" => power.slope.clone_from(v),
            "trdl" => power.trdl.clone_from(v),
            "opos" => power.opos.clone_from(v),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let scope = Scope { shape, power, empty, sweep, waiting: false, acquisition: 0, reads: 0, trace_queries: 0 };
    Ok((scope, config))
}

/// A scope on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (scope, config) = parts(options)?;
    let mut id = Identity::new(0x05ff, 0x1023, "LeCroy", "WaveRunner 8104");
    id.speed = Speed::High;
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

    /// A line, and the bytes and the violations it makes.
    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (Vec<u8>, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        let bytes = port.sent().concat();
        (bytes, port.violations().drain())
    }

    fn ready(s: &mut Box<dyn Stream>) {
        ask(s, "COMM_HEADER OFF\nCOMM_FORMAT DEF9,WORD,BIN\n");
    }

    fn u32_at(block: &[u8], at: usize) -> u32 {
        u32::from_le_bytes(block[11 + at..11 + at + 4].try_into().unwrap())
    }

    fn f32_at(block: &[u8], at: usize) -> f32 {
        f32::from_le_bytes(block[11 + at..11 + at + 4].try_into().unwrap())
    }

    #[test]
    fn the_manual_lays_out_the_descriptor() {
        let mut s = on_socket(&[("offset", "0.25")]);
        ready(&mut s);
        let (block, flags) = ask(&mut s, "C2:WAVEFORM?\n");
        assert_eq!(flags, Vec::<String>::new());
        assert_eq!(&block[..11], format!("#9{:09}", DESCRIPTOR + 400).as_bytes());
        assert_eq!(block.len(), 11 + DESCRIPTOR + 400 + 1);
        assert_eq!(block.last(), Some(&b'\n'), "the manual's line end");
        assert_eq!(&block[11..19], b"WAVEDESC");
        assert_eq!(&block[11 + 16..11 + 26], b"LECROY_2_3");
        assert_eq!(u32_at(&block, 36), DESCRIPTOR as u32);
        assert_eq!(u32_at(&block, 60), 400);
        assert_eq!(u32_at(&block, 116), 200);
        assert_eq!(block[11 + 34], 1, "little endian");
        assert_eq!(f32_at(&block, 156), (1.0 / COUNTS_PER_DIV) as f32);
        assert_eq!(f32_at(&block, 160), 0.25);
        assert_eq!(block[11 + 196], b'V');
        assert_eq!(block[11 + 197], 0);
        assert_eq!(block[11 + 344], 1, "WAVE_SOURCE of C2");
    }

    #[test]
    fn big_endian_goes_by_the_field() {
        let mut s = on_socket(&[("comm-order", "0")]);
        ready(&mut s);
        let block = ask(&mut s, "C1:WAVEFORM?\n").0;
        assert_eq!(block[11 + 35], 0);
        assert_eq!(block[11 + 39], 0x5a, "346 ends in 0x5a, last");
    }

    #[test]
    fn an_empty_scope_says_so_and_an_acquisition_gives_samples() {
        let mut s = on_socket(&[("empty", "1")]);
        ready(&mut s);
        assert_eq!(u32_at(&ask(&mut s, "C1:WAVEFORM?\n").0, 116), 0);
        let (block, flags) = ask(&mut s, "ARM;WAIT;*OPC;C1:WAVEFORM?\n");
        assert_eq!(u32_at(&block, 116), 200);
        assert_eq!(flags, Vec::<String>::new());
    }

    #[test]
    fn each_acquisition_differs() {
        let mut s = on_socket(&[]);
        ready(&mut s);
        let one = ask(&mut s, "C1:WAVEFORM?\n").0;
        assert_eq!(ask(&mut s, "C1:WAVEFORM?\n").0, one, "no ARM, the same trace");
        ask(&mut s, "ARM;WAIT;*OPC\n");
        assert_ne!(ask(&mut s, "C1:WAVEFORM?\n").0, one);
        assert_ne!(ask(&mut s, "C2:WAVEFORM?\n").0, one);
    }

    #[test]
    fn a_sweep_holds_what_follows_a_wait() {
        let mut s = on_socket(&[("sweep", "2")]);
        ready(&mut s);
        assert_eq!(ask(&mut s, "ARM;WAIT;*OPC;C1:WAVEFORM?\n").0, Vec::<u8>::new());
        let mut port = Port::default();
        s.tick(&mut port);
        assert_eq!(port.sent().len(), 0);
        s.tick(&mut port);
        assert_eq!(port.sent().len(), 1);
    }

    #[test]
    fn the_checks_fire() {
        let mut s = on_socket(&[("trace", "1000")]);
        assert_eq!(ask(&mut s, "C1:WAVEFORM?\n").1.len(), 1, "before the format");
        ready(&mut s);
        assert_eq!(ask(&mut s, "C1:WAVEFORM?\n").1.len(), 0);
        assert_eq!(ask(&mut s, "C2:WAVEFORM?\n").1.len(), 1, "a channel that is off");
        assert_eq!(ask(&mut s, "C5:TRACE?\n").1.len(), 1, "four channels");
        assert_eq!(ask(&mut s, "NO_SUCH\n").1.len(), 1);
    }

    #[test]
    fn settings_answer_in_the_form_of_the_comment() {
        let mut s = on_socket(&[]);
        assert_eq!(ask(&mut s, "*IDN?\n").0, b"*IDN LECROY,WR8104,LCRY0001N00001,8.6.0\n");
        assert_eq!(ask(&mut s, "C1:VDIV?\n").0, b"1.000E+00\n");
        ask(&mut s, "C1:VDIV 5.000000E-03\n");
        assert_eq!(ask(&mut s, "C1:VDIV?\n").0, b"5.000E-03\n");
        ask(&mut s, "TIME_DIV 2.000000E-11\n");
        assert_eq!(ask(&mut s, "TIME_DIV?\n").0, b"2.000E-11\n");
        assert_eq!(ask(&mut s, "TRIG_SELECT?\n").0, b"EDGE,SR,C1,HT,OFF\n");
        ask(&mut s, "TRIG_SELECT EDGE,SR,C2\n");
        assert_eq!(ask(&mut s, "C2:TRIG_SLOPE?\n").0, b"POS\n");
        assert_eq!(ask(&mut s, "TRIG POS 1.000000e-03 S\n").1, Vec::<String>::new());
    }

    #[test]
    fn a_line_end_can_come_late_and_a_source_be_wrong() {
        let mut s = on_socket(&[("eol-late", "2"), ("wrong-source", "1")]);
        ready(&mut s);
        let block = ask(&mut s, "C1:WAVEFORM?\n").0;
        assert_ne!(block.last(), Some(&b'\n'));
        assert_eq!(block[11 + 344], 1, "C2's");
        let mut port = Port::default();
        s.tick(&mut port);
        assert_eq!(port.sent().len(), 0);
        s.tick(&mut port);
        assert_eq!(port.sent().concat(), b"\n");
        assert_eq!(ask(&mut s, "C1:WAVEFORM?\n").0[11 + 344], 0);
    }

    #[test]
    fn the_traces_change_after_the_scan_has_asked() {
        let mut s = on_socket(&[("trace-later", "0100")]);
        for c in 1..=4 {
            assert_eq!(ask(&mut s, &format!("C{c}:TRACE?\n")).0, b"ON\n");
        }
        assert_eq!(ask(&mut s, "C1:TRACE?\n").0, b"OFF\n");
        assert_eq!(ask(&mut s, "C2:TRACE?\n").0, b"ON\n");
    }

    #[test]
    fn the_faults_of_the_blocks() {
        let mut s = on_socket(&[("cut", "1"), ("points", "10")]);
        ready(&mut s);
        let whole = DESCRIPTOR + 20;
        let block = ask(&mut s, "C1:WAVEFORM?\n").0;
        assert_eq!(&block[..11], format!("#9{whole:09}").as_bytes());
        assert_eq!(block.len(), 11 + whole / 2, "half of it");
        assert_eq!(ask(&mut s, "C1:WAVEFORM?\n").0.len(), 11 + whole + 1);
        let mut s = on_socket(&[("head", "#0"), ("head-at", "2")]);
        ready(&mut s);
        assert!(ask(&mut s, "C1:WAVEFORM?\n").0.len() > 100);
        assert_eq!(ask(&mut s, "C1:WAVEFORM?\n").0, b"#0");
        let mut s = on_socket(&[("prefix", "ALL,"), ("no-eol", "1"), ("tiny", "20")]);
        ready(&mut s);
        let block = ask(&mut s, "C1:WAVEFORM?\n").0;
        assert_eq!(&block[..6], b"ALL,#9");
        assert_eq!(block.len(), 4 + 11 + 20);
    }

    #[test]
    fn it_answers_its_options_on_usb_too() {
        assert_eq!(build(&[]).unwrap().descriptors().vendor_id(), 0x05ff);
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
