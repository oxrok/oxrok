// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A GW Instek GDS-806/810/820/840 scope on its RS-232 port, here on CDC-ACM.
//!
//! The device, and how sure this is:
//!
//! - GW Instek's "GDS-806/810/820/840 Programming Manual", which the sigrok wiki links:
//!   the `*IDN?` form (`GW,GDS-820,P920130,V.2.01`), `:TRIG:MOD 3` for single,
//!   `:ACQ<X>:MEM?` from firmware 2.03 only, its block `#41008` up to `#6250008` with
//!   four bytes of rate, one of channel, three reserved, and 16 bit points high byte
//!   first. No real unit, recording or firmware was looked at; none was found.
//! - From libsigrok's driver, against the manual: the rate's bytes. The manual says
//!   little endian and its own example, 0x4C 0xBE 0xBC 0x20 for 100 MSa/s, comes high
//!   byte first, which is what the driver's comment reports from a scope.
//! - **No line end after a block,** against the manual's general note (a line feed
//!   behind each response). A driver that worked in 2015 could not have met one:
//!   the serial layer of its day gave `Too many bytes read` on a block read in pieces
//!   with a line feed behind it, and the `:CHANn:SCAL?` after a block would read that
//!   line feed as its answer, and 0 volts. An inference: `end=lf` is the other reading.
//! - `:CHANn:SCAL?`: the manual says `<NR3>`. The driver takes `mV` behind the number,
//!   so a real one has said that, and `V` beside it is a guess. CH1 says `1.00V` here,
//!   CH2 `500mV`.
//!
//! Emulation artifacts:
//!
//! - The answer comes at once. A real scope in single mode may hold its answer until
//!   a trigger and a sweep: the manual doesn't say. `late-at=` shows that.
//! - The trace is a triangle free of the byte 0x0A, as `src/scpi/scpi_serial.c` makes
//!   stock C see such a byte at a read's end twice, and where reads end goes with the
//!   clock. `wave=full` has every low byte, which any real signal has: libsigrok's
//!   unmerged pull request 121 reports what C does then, from a real GDS-2000.
//! - The record length comes from `points=`, not from `:ACQ:LENG`, which no driver sends.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: `GDS-820` by default, and `firmware=TEXT`, `V.2.03` by default;
//!   before 2.03 `:ACQ<X>:MEM?` is an unknown command, with no answer;
//! - `points=N`: the record length (500 by default);
//! - `rate=F`: the rate field as a float (1e8);
//! - `wave=full`: every low byte in turn, and `last=HEX`: the last point's low byte;
//! - `scale1=TEXT`, `scale2=TEXT`: what `:CHANn:SCAL?` says;
//! - `end=lf`: a line feed behind each block;
//! - `indicator=N`: the channel byte says this;
//! - `head=TEXT`: in place of `#41008`, for a wrong or foreign start;
//! - `cut=N`: only N bytes behind the head;
//! - `at=N`: `head=`, `cut=`, `last=` and `end=` for the Nth memory read alone.
//!
//! The pedantic checks: a memory read of a channel that isn't there, one before any
//! `:RUN`, and a `:TRIG:MOD` that isn't 0 to 3.

use crate::model::{DeviceModel, Violations};
use crate::options::parse_number;
use crate::shells::cdc::Identity;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings};
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "gwinstek-gds-800";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A GW Instek GDS-800 scope on CDC-ACM; model=, firmware=, points=, rate=, \
                         wave=, last=, scale1=, scale2=, end=, indicator=, head=, cut=, at=, and the \
                         SCPI shell's mute-at=, silent-at=, late-at=, mark-at=, idn=, eol=, stall=, \
                         chunk=, in-limit=, in-stall=, out-hold=, uart=";

/// The eight bytes ahead of the points.
const FIELDS: usize = 8;
/// The largest record, in points.
const MOST_POINTS: u32 = 1 << 20;

/// A scope.
struct Scope {
    model: String,
    firmware: String,
    points: usize,
    rate: f32,
    full_wave: bool,
    last: Option<u8>,
    scales: [String; 2],
    end_lf: bool,
    indicator: Option<u8>,
    head: Option<String>,
    cut: Option<usize>,
    at: Option<u32>,
    reads: u32,
    run: bool,
}

impl Scope {
    /// Firmware's `V.a.bb`, as a number; none in another form, as new as can be.
    fn has_memory_read(&self) -> bool {
        let digits = self.firmware.trim_start_matches(['V', 'v', '.']);
        let Some((major, minor)) = digits.split_once('.') else { return true };
        let (Ok(major), Ok(minor)) = (major.parse::<u32>(), minor.parse::<u32>()) else {
            return true;
        };
        (major, minor) >= (2, 3)
    }

    /// A point, in the counts `vbit` scales: eight bits' worth in sixteen.
    fn point(&self, channel: u8, i: usize) -> i16 {
        if self.full_wave {
            return ((i * 5 + usize::from(channel) * 37) % 256) as i16 - 128;
        }
        let t = ((i * 3 + usize::from(channel) * 37) % 200) as i16 - 100;
        // A low byte of 0x0A: see the module's words on the trace.
        if t == 10 { 11 } else { t }
    }

    fn memory(&mut self, m: &Message<'_>, v: &mut Violations) -> Answer {
        let channel = m.header.strip_prefix("ACQ").and_then(|h| h.strip_suffix(":MEM?"));
        let channel = channel.and_then(|c| c.parse().ok()).filter(|c| matches!(c, 1 | 2));
        let Some(channel) = channel else {
            v.report(format!("{}: the memory of no channel this model has", m.text));
            return Answer::Silent;
        };
        if !self.run {
            v.report(format!("{}: nothing was acquired, no :RUN came", m.text));
        }
        if !self.has_memory_read() {
            return Answer::Error(-113, "Undefined header".to_owned());
        }
        self.reads += 1;
        let fault = self.at.is_none_or(|at| at == self.reads);
        Answer::Bytes(self.block(channel, fault))
    }

    fn block(&self, channel: u8, fault: bool) -> Vec<u8> {
        let mut body = Vec::with_capacity(FIELDS + self.points * 2);
        body.extend_from_slice(&self.rate.to_bits().to_be_bytes());
        body.push(self.indicator.unwrap_or(channel));
        body.extend_from_slice(&[0, 0, 0]);
        for i in 0..self.points {
            body.extend_from_slice(&self.point(channel, i).to_be_bytes());
        }
        if fault {
            if let (Some(last), true) = (self.last, self.points > 0) {
                *body.last_mut().expect("a point") = last;
            }
            if let Some(n) = self.cut {
                body.truncate(n);
            }
        }
        let size = FIELDS + self.points * 2;
        let digits = size.to_string().len();
        let mut out = match self.head.as_deref().filter(|_| fault) {
            Some(head) => head.as_bytes().to_vec(),
            None => format!("#{digits}{size}").into_bytes(),
        };
        out.extend_from_slice(&body);
        if self.end_lf && fault {
            out.push(b'\n');
        }
        out
    }
}

impl Instrument for Scope {
    fn idn(&self) -> Option<String> {
        Some(format!("GW,{},P920130,{}", self.model, self.firmware))
    }

    fn settings(&self) -> Settings {
        Settings::default()
            .with("TRIG:MOD", "1")
            .with("CHAN1:SCAL", self.scales[0].clone())
            .with("CHAN2:SCAL", self.scales[1].clone())
    }

    fn command(&mut self, m: &Message<'_>, _s: &mut Settings, v: &mut Violations) -> Answer {
        match m.header.as_str() {
            "RUN" => {
                self.run = true;
                Answer::Done
            }
            "STOP" => Answer::Done,
            "TRIG:MOD" if !matches!(m.args, "0" | "1" | "2" | "3") => {
                v.report(format!("{}: a trigger mode is 0 to 3", m.text));
                Answer::Done
            }
            h if h.starts_with("ACQ") && m.is_query() => self.memory(m, v),
            _ => Answer::Unknown,
        }
    }
}

fn parts(options: &[(String, String)]) -> Result<(Scope, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut scope = Scope {
        model: "GDS-820".to_owned(),
        firmware: "V.2.03".to_owned(),
        points: 500,
        rate: 1e8,
        full_wave: false,
        last: None,
        scales: ["1.00V".to_owned(), "500mV".to_owned()],
        end_lf: false,
        indicator: None,
        head: None,
        cut: None,
        at: None,
        reads: 0,
        run: false,
    };
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => v.clone_into(&mut scope.model),
            "firmware" => v.clone_into(&mut scope.firmware),
            "points" => scope.points = parse_number(k, v, 0..=MOST_POINTS)? as usize,
            "rate" => {
                scope.rate = v.parse().map_err(|_| Error::Invalid(format!("rate={v}: a float")))?;
            }
            "wave" if v == "full" => scope.full_wave = true,
            "last" => {
                scope.last = Some(
                    u8::from_str_radix(v, 16)
                        .map_err(|_| Error::Invalid(format!("last={v}: a byte in hex")))?,
                );
            }
            "scale1" => v.clone_into(&mut scope.scales[0]),
            "scale2" => v.clone_into(&mut scope.scales[1]),
            "end" if v == "lf" => scope.end_lf = true,
            "indicator" => scope.indicator = Some(parse_number(k, v, 0..=255)? as u8),
            "head" => scope.head = Some(v.clone()),
            "cut" => scope.cut = Some(parse_number(k, v, 0..=MOST_POINTS)? as usize),
            "at" => scope.at = Some(parse_number(k, v, 1..=1 << 24)?),
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}={v}"))),
        }
    }
    Ok((scope, config))
}

/// A scope on a tty, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (scope, config) = parts(options)?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "GDS-800 on a serial port");
    Ok(config.cdc(NAME, &id, scope))
}

/// A block is long: its digest in the log.
pub fn log_rules(_options: &[(String, String)]) -> Rules {
    Rules::digest_above(HASH_ABOVE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::tcp::{Port, Stream};

    /// The same [`scpi::Engine`] on a socket, which needs no kernel to ask.
    fn scope(options: &[(&str, &str)]) -> Box<dyn Stream> {
        let pairs: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let (scope, config) = parts(&pairs).unwrap();
        config.tcp(scope)
    }

    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (Vec<u8>, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        let bytes = port.sent().concat();
        (bytes, port.violations().drain())
    }

    fn armed(options: &[(&str, &str)]) -> Box<dyn Stream> {
        let mut s = scope(options);
        for line in [":TRIG:MOD 3\n", ":STOP\n", ":RUN\n"] {
            assert_eq!(ask(&mut s, line), (Vec::new(), Vec::new()));
        }
        s
    }

    #[test]
    fn it_builds_and_refuses_what_it_lacks() {
        assert_eq!(build(&[]).unwrap().name(), NAME);
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
        assert!(build(&[("wave".to_owned(), "other".to_owned())]).is_err());
    }

    #[test]
    fn it_says_who_it_is_as_the_manual_has_it() {
        let mut s = scope(&[]);
        assert_eq!(ask(&mut s, "*IDN?\n").0, b"GW,GDS-820,P920130,V.2.03\n");
        let mut s = scope(&[("model", "GDS-840C"), ("firmware", "V.2.10")]);
        assert_eq!(ask(&mut s, "*IDN?\n").0, b"GW,GDS-840C,P920130,V.2.10\n");
    }

    #[test]
    fn a_block_is_as_the_manual_lays_it_out() {
        let mut s = armed(&[]);
        let (block, flags) = ask(&mut s, ":ACQ2:MEM?\n");
        assert_eq!(flags, Vec::<String>::new());
        assert_eq!(&block[..6], b"#41008");
        // The manual's example of 100 MSa/s.
        assert_eq!(&block[6..10], &[0x4c, 0xbe, 0xbc, 0x20]);
        assert_eq!(&block[10..14], &[2, 0, 0, 0]);
        assert_eq!(block.len(), 6 + 1008, "no line end");
        let (block, _) = ask(&mut s, ":ACQ1:MEM?\n");
        assert_eq!(block[10], 1);
    }

    #[test]
    fn every_record_length_has_the_manuals_head() {
        for (points, head) in [
            ("500", "#41008"),
            ("1250", "#42508"),
            ("2500", "#45008"),
            ("5000", "#510008"),
            ("12500", "#525008"),
            ("25000", "#550008"),
            ("50000", "#6100008"),
            ("125000", "#6250008"),
        ] {
            let mut s = armed(&[("points", points)]);
            let (block, _) = ask(&mut s, ":ACQ1:MEM?\n");
            assert!(block.starts_with(head.as_bytes()), "{points}");
            assert_eq!(block.len(), head.len() + 8 + points.parse::<usize>().unwrap() * 2);
        }
    }

    #[test]
    fn the_plain_trace_has_no_line_feed_and_the_full_one_does() {
        let lf = |block: &[u8]| block[14..].split(|b| *b == b'\n').count() - 1;
        let mut s = armed(&[("points", "125000")]);
        assert_eq!(lf(&ask(&mut s, ":ACQ1:MEM?\n").0), 0);
        assert_eq!(lf(&ask(&mut s, ":ACQ2:MEM?\n").0), 0);
        let mut s = armed(&[("wave", "full")]);
        assert!(lf(&ask(&mut s, ":ACQ1:MEM?\n").0) > 0);
    }

    #[test]
    fn the_scale_answers_with_a_line_end() {
        let mut s = scope(&[("scale2", "2.00V")]);
        assert_eq!(ask(&mut s, ":CHAN1:SCAL?\n").0, b"1.00V\n");
        assert_eq!(ask(&mut s, ":CHAN2:SCAL?\n").0, b"2.00V\n");
    }

    #[test]
    fn faults_land_where_they_are_asked_for() {
        let mut s = armed(&[("last", "0a"), ("at", "2")]);
        assert_ne!(ask(&mut s, ":ACQ1:MEM?\n").0.last(), Some(&b'\n'));
        assert_eq!(ask(&mut s, ":ACQ2:MEM?\n").0.last(), Some(&b'\n'));
        assert_ne!(ask(&mut s, ":ACQ1:MEM?\n").0.last(), Some(&b'\n'));

        let mut s = armed(&[("cut", "20")]);
        assert_eq!(ask(&mut s, ":ACQ1:MEM?\n").0.len(), 26);
        let mut s = armed(&[("end", "lf")]);
        assert_eq!(ask(&mut s, ":ACQ1:MEM?\n").0.len(), 6 + 1008 + 1);
        let mut s = armed(&[("head", "x#47")]);
        assert!(ask(&mut s, ":ACQ1:MEM?\n").0.starts_with(b"x#47\x4c"));
        let mut s = armed(&[("indicator", "9")]);
        assert_eq!(ask(&mut s, ":ACQ1:MEM?\n").0[10], 9);
        let mut s = armed(&[("rate", "1e3")]);
        assert_eq!(&ask(&mut s, ":ACQ1:MEM?\n").0[6..10], &1e3f32.to_bits().to_be_bytes());
    }

    #[test]
    fn firmware_before_2_03_has_no_memory_read() {
        let mut s = armed(&[("firmware", "V.2.01")]);
        assert_eq!(ask(&mut s, ":ACQ1:MEM?\n"), (Vec::new(), Vec::new()));
        for firmware in ["V.2.03", "V.2.10", "V.3.00", "whatever"] {
            let mut s = armed(&[("firmware", firmware)]);
            assert!(!ask(&mut s, ":ACQ1:MEM?\n").0.is_empty(), "{firmware}");
        }
    }

    #[test]
    fn each_check_fires() {
        let mut s = scope(&[]);
        assert_eq!(ask(&mut s, ":ACQ1:MEM?\n").1.len(), 1, "no :RUN yet");
        let mut s = armed(&[]);
        assert_eq!(ask(&mut s, ":ACQ3:MEM?\n").1.len(), 1, "no such channel");
        assert_eq!(ask(&mut s, ":ACQ:MEM?\n").1.len(), 1, "no channel");
        assert_eq!(ask(&mut s, ":TRIG:MOD 4\n").1.len(), 1, "no such mode");
        assert_eq!(ask(&mut s, ":NO:SUCH\n").1.len(), 1, "the engine's");
    }
}
