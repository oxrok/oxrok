// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Rigol DG800, DG900 and DG1000Z function generators, and the generator inside an
//! MSO5000, on USBTMC and on TCP (`docs/USB.md`, "Writing a SCPI model").
//!
//! The device, and how sure this is:
//!
//! - The DG1000Z's commands and answers: its Programming Guide (DG1000Z, 2-xx).
//!   `SOURn:APPL?` is a quoted string of five parts with `DEF` for what a waveform
//!   lacks; `COUN:MEAS?` is five numbers in ten digits, all 0 when the counter is
//!   off, and goes on answering while the counter runs, whatever its gate time
//!   (1.31 ms to over 10 s); `COUN:STAT?` is `RUN`, `STOP`, `SINGLE` or `OFF`;
//!   `OUTPn:STATE?` is `ON` or `OFF`; `*OPC?` is `1`.
//! - An `APPL:x` with no values takes the guide's defaults, 1 kHz, 5 Vpp, 0 V and
//!   0 degrees: the guide's "Default" column, read for a value left out, and no
//!   run on a unit.
//! - The DG800 and DG900: as the DG1000Z, by the C driver's one table of commands.
//!   Their manual was not to be had.
//! - The MSO5000's generator: only what libsigrok's table sends. Numbers as the
//!   DG's, and `0` or `1` for an output: a guess.
//!
//! Guesses, from memory and from no source, which no test should lean on:
//!
//! - The ids 1ab1:0642, 0643 and 0644 for the series, and 1ab1:0515 from
//!   `rigol-ds`'s table for the MSO5000. All on one id per series, and the series'
//!   name for its strings, so that a `model=` of the same series needs no re-attach;
//!   a real one has its model there.
//! - The firmware strings, serial numbers and bus speeds. Whether a real one takes
//!   remote control (`remote=1`).
//!
//! What the model refuses: a number out of the waveform's range or a NaN gets `-222`
//! in the queue and changes nothing, which is a guess at what a unit does, and no
//! tool reads the queue. A set of a key on a waveform that lacks it goes unflagged:
//! `-221` likewise.
//!
//! Emulation artifacts:
//!
//! - The counter's readings are made up, a new one per query so that a shift shows:
//!   frequency 1000 + N, period 1/f, a duty that steps from 40 to 59 %, and the
//!   widths from those.
//! - No time passes. The 1 s that C waits around the counter on a DG800 or DG900
//!   can't be checked here (no wall time in a model): the driver's tests hold it.
//! - What a DG800 does with channel 2 on and the counter on isn't modelled: C's
//!   commit says they can't be on together, and nothing says what happens.
//!
//! Options, besides the shell's ([`scpi::Config::option`]):
//!
//! - `model=NAME`: any of C's table, `DG1062Z` by default;
//! - `counter=TEXT`: what `COUN:STAT?` says at first, `OFF` by default;
//! - `wave=NAME`, `wave2=NAME`: the first waveform of a channel, by its SCPI name;
//! - `output2=1`: channel 2's output on, at first;
//! - `say=QUERY=TEXT`: what a query answers, always, with `~` for a space, as options
//!   are cut at spaces; `eol` still goes after it;
//! - `play=A;B;C`: what `COUN:MEAS?` answers, each once, and then its readings;
//! - `remote=1`: says it takes remote control (`RL1`).
//!
//! The pedantic checks: only the commands the model has; a channel it lacks.

use crate::model::{DeviceModel, Violations};
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "rigol-dg";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Rigol DG800, DG900, DG1000Z and MSO5000 generators on USBTMC and TCP; \
                         model=, counter=, wave=, wave2=, output2=, say=, play=, remote=, and the \
                         SCPI shell's mute-at=, silent-at=, late-at=, opc-late=, mark-at=, idn=, \
                         eol=, framing=, stall=, in-stall=, empty-at=, bad-tag-at=, message=";

/// What a waveform takes: a frequency, an amplitude, an offset, a phase.
#[derive(Clone, Copy, Debug)]
struct Wave {
    name: &'static str,
    freq_min: f64,
    freq_max: f64,
    amplitude: bool,
    frequency: bool,
    phase: bool,
}

const fn wave(name: &'static str, freq_min: f64, freq_max: f64) -> Wave {
    Wave { name, freq_min, freq_max, amplitude: true, frequency: true, phase: true }
}

const fn noise(name: &'static str) -> Wave {
    Wave { name, freq_min: 0.0, freq_max: 0.0, amplitude: true, frequency: false, phase: false }
}

const DC: Wave =
    Wave { name: "DC", freq_min: 0.0, freq_max: 0.0, amplitude: false, frequency: false, phase: false };

/// The seven waveforms of a DG, with their tops, as `api.c` has them.
const fn dg(sine: f64, square: f64, ramp: f64, pulse: f64, arb: f64) -> [Wave; 7] {
    const LOW: f64 = 1e-6;
    [
        wave("SIN", LOW, sine),
        wave("SQU", LOW, square),
        wave("RAMP", LOW, ramp),
        wave("PULSE", LOW, pulse),
        wave("USER", LOW, arb),
        noise("NOISE"),
        DC,
    ]
}

const DG810: [Wave; 7] = dg(10e6, 5e6, 0.2e6, 5e6, 5e6);
const DG820: [Wave; 7] = dg(25e6, 10e6, 0.5e6, 10e6, 10e6);
const DG830: [Wave; 7] = dg(35e6, 10e6, 1e6, 10e6, 10e6);
const DG952: [Wave; 7] = dg(50e6, 15e6, 1.5e6, 15e6, 15e6);
const DG972: [Wave; 7] = dg(70e6, 20e6, 1.5e6, 20e6, 20e6);
const DG992: [Wave; 7] = dg(100e6, 25e6, 2e6, 25e6, 25e6);
const DG1022Z: [Wave; 7] = dg(25e6, 25e6, 0.5e6, 15e6, 10e6);
const DG1032Z: [Wave; 7] = dg(30e6, 25e6, 0.5e6, 15e6, 10e6);
const DG1062Z: [Wave; 7] = dg(60e6, 25e6, 1e6, 25e6, 20e6);
const MSO: [Wave; 14] = [
    wave("SIN", 0.1, 25e6),
    wave("SQU", 0.1, 15e6),
    wave("RAMP", 0.1, 100e3),
    wave("PULS", 0.1, 1e6),
    wave("USER", 0.1, 10e6),
    noise("NOIS"),
    DC,
    wave("SINC", 0.1, 1e6),
    wave("EXPR", 0.1, 1e6),
    wave("EXPF", 0.1, 1e6),
    wave("ECG", 0.1, 1e6),
    wave("GAUS", 0.1, 1e6),
    wave("LOR", 0.1, 1e6),
    wave("HAV", 0.1, 1e6),
];

/// A model, as far as it shows on the wire.
struct Kind {
    name: &'static str,
    series: &'static str,
    usb: u16,
    channels: u8,
    waves: &'static [Wave],
    is_mso: bool,
}

const fn kind(
    name: &'static str,
    series: &'static str,
    usb: u16,
    channels: u8,
    waves: &'static [Wave],
) -> Kind {
    Kind { name, series, usb, channels, waves, is_mso: false }
}

const fn mso(name: &'static str) -> Kind {
    Kind { name, series: "MSO5000", usb: 0x0515, channels: 2, waves: &MSO, is_mso: true }
}

const KINDS: &[Kind] = &[
    kind("DG1062Z", "DG1000Z", 0x0642, 2, &DG1062Z),
    kind("DG1032Z", "DG1000Z", 0x0642, 2, &DG1032Z),
    kind("DG1022Z", "DG1000Z", 0x0642, 2, &DG1022Z),
    kind("DG811", "DG800", 0x0643, 1, &DG810),
    kind("DG812", "DG800", 0x0643, 2, &DG810),
    kind("DG821", "DG800", 0x0643, 1, &DG820),
    kind("DG822", "DG800", 0x0643, 2, &DG820),
    kind("DG831", "DG800", 0x0643, 1, &DG830),
    kind("DG832", "DG800", 0x0643, 2, &DG830),
    kind("DG952", "DG900", 0x0644, 2, &DG952),
    kind("DG972", "DG900", 0x0644, 2, &DG972),
    kind("DG992", "DG900", 0x0644, 2, &DG992),
    mso("MSO5354"),
    mso("MSO5204"),
    mso("MSO5104"),
    mso("MSO5102"),
    mso("MSO5074"),
    mso("MSO5072"),
];

/// One channel's state.
#[derive(Clone, Debug)]
struct Channel {
    wave: usize,
    frequency: f64,
    amplitude: f64,
    offset: f64,
    phase: f64,
    duty_square: f64,
    duty_pulse: f64,
    output: bool,
}

impl Channel {
    fn new(wave: usize) -> Channel {
        Channel {
            wave,
            frequency: 1e3,
            amplitude: 5.0,
            offset: 0.0,
            phase: 0.0,
            duty_square: 50.0,
            duty_pulse: 50.0,
            output: false,
        }
    }
}

/// `1.000000E+03`, with `places` digits after the point.
fn sci(value: f64, places: usize) -> String {
    let text = format!("{value:.places$E}");
    let (mantissa, exponent) = text.split_once('E').expect("LowerExp has an E");
    let exponent: i32 = exponent.parse().expect("LowerExp's exponent is a number");
    format!("{mantissa}E{}{:02}", if exponent < 0 { '-' } else { '+' }, exponent.abs())
}

/// Splits `SOUR1:VOLT?` into its channel and what follows.
fn channel_of<'m>(header: &'m str, prefix: &str) -> Option<(usize, &'m str)> {
    let rest = header.strip_prefix(prefix)?;
    let digits = rest.find(|c: char| !c.is_ascii_digit()).unwrap_or(rest.len());
    let number: usize = rest[..digits].parse().ok()?;
    Some((number, &rest[digits..]))
}

/// A generator.
pub(super) struct Generator {
    kind: &'static Kind,
    channels: Vec<Channel>,
    counter: String,
    /// `say=`: a query's answer, whatever the state.
    say: Vec<(String, String)>,
    /// `play=`: the next `COUN:MEAS?`.
    play: VecDeque<String>,
    readings: u32,
}

impl Generator {
    fn measure(&mut self) -> String {
        if let Some(text) = self.play.pop_front() {
            return text;
        }
        if self.counter == "OFF" {
            return vec![sci(0.0, 9); 5].join(",");
        }
        self.readings += 1;
        let frequency = 1000.0 + f64::from(self.readings);
        let period = 1.0 / frequency;
        let duty = 40.0 + f64::from(self.readings % 20);
        let width = period * duty / 100.0;
        [frequency, period, duty, width, period - width].map(|v| sci(v, 9)).join(",")
    }

    fn apply(&self, channel: &Channel) -> String {
        let wave = &self.kind.waves[channel.wave];
        let field = |has: bool, value: f64| if has { sci(value, 6) } else { "DEF".to_owned() };
        format!(
            "\"{},{},{},{},{}\"",
            wave.name,
            field(wave.frequency, channel.frequency),
            field(wave.amplitude, channel.amplitude),
            sci(channel.offset, 6),
            field(wave.phase, channel.phase),
        )
    }

    fn output_answer(&self, on: bool) -> &'static str {
        match (self.kind.is_mso, on) {
            (true, true) => "1",
            (true, false) => "0",
            (false, true) => "ON",
            (false, false) => "OFF",
        }
    }

    /// What a channel's number comes to. `Done` or the refusal.
    fn set_number(
        &mut self,
        channel: usize,
        m: &Message<'_>,
        set: impl Fn(&mut Channel, f64, &Wave) -> bool,
    ) -> Answer {
        let Ok(value) = m.args.parse::<f64>() else {
            return Answer::Error(-222, "Data out of range".to_owned());
        };
        let state = &mut self.channels[channel];
        let wave = self.kind.waves[state.wave];
        if value.is_finite() && set(state, value, &wave) {
            Answer::Done
        } else {
            Answer::Error(-222, "Data out of range".to_owned())
        }
    }

    fn source(&mut self, channel: usize, rest: &str, m: &Message<'_>) -> Answer {
        let state = &self.channels[channel];
        match rest {
            ":APPL?" if !self.kind.is_mso => Answer::Text(self.apply(state)),
            ":FUNC?" if self.kind.is_mso => {
                Answer::Text(self.kind.waves[state.wave].name.to_owned())
            }
            ":FUNC" if self.kind.is_mso => self.set_wave(channel, m.args),
            ":FREQ?" => Answer::Text(sci(state.frequency, 6)),
            ":VOLT?" => Answer::Text(sci(state.amplitude, 6)),
            ":VOLT:OFFS?" => Answer::Text(sci(state.offset, 6)),
            ":PHAS?" => Answer::Text(sci(state.phase, 6)),
            ":FUNC:SQU:DCYC?" if !self.kind.is_mso => Answer::Text(sci(state.duty_square, 6)),
            ":FUNC:PULS:DCYC?" if !self.kind.is_mso => Answer::Text(sci(state.duty_pulse, 6)),
            ":PULS:DCYC?" if self.kind.is_mso => Answer::Text(sci(state.duty_pulse, 6)),
            ":FREQ" => self.set_number(channel, m, |c, v, w| {
                let good = v >= w.freq_min && v <= w.freq_max;
                if good {
                    c.frequency = v;
                }
                good
            }),
            ":VOLT" => self.set_number(channel, m, |c, v, _| {
                c.amplitude = v;
                true
            }),
            ":VOLT:OFFS" => self.set_number(channel, m, |c, v, _| {
                c.offset = v;
                true
            }),
            ":PHAS" => self.set_number(channel, m, |c, v, _| {
                let good = (0.0..=360.0).contains(&v);
                if good {
                    c.phase = v;
                }
                good
            }),
            ":FUNC:SQU:DCYC" if !self.kind.is_mso => self.set_number(channel, m, |c, v, _| {
                let good = (0.0..=100.0).contains(&v);
                if good {
                    c.duty_square = v;
                }
                good
            }),
            ":FUNC:PULS:DCYC" | ":PULS:DCYC" => self.set_number(channel, m, |c, v, _| {
                let good = (0.0..=100.0).contains(&v);
                if good {
                    c.duty_pulse = v;
                }
                good
            }),
            _ => match rest.strip_prefix(":APPL:") {
                Some(name) if !self.kind.is_mso => self.set_wave(channel, name),
                _ => Answer::Unknown,
            },
        }
    }

    /// `APPL:x` or `FUNC x`, with the guide's defaults for what it leaves out.
    fn set_wave(&mut self, channel: usize, name: &str) -> Answer {
        let Some(wave) = self.kind.waves.iter().position(|w| w.name.eq_ignore_ascii_case(name))
        else {
            return Answer::Unknown;
        };
        let state = &mut self.channels[channel];
        let output = state.output;
        *state = Channel::new(wave);
        state.output = output;
        Answer::Done
    }

    fn output(&mut self, channel: usize, rest: &str, m: &Message<'_>) -> Answer {
        match rest {
            ":STATE?" => Answer::Text(self.output_answer(self.channels[channel].output).to_owned()),
            ":STATE" => match m.args {
                "ON" => {
                    self.channels[channel].output = true;
                    Answer::Done
                }
                "OFF" => {
                    self.channels[channel].output = false;
                    Answer::Done
                }
                _ => Answer::Unknown,
            },
            _ => Answer::Unknown,
        }
    }
}

impl Instrument for Generator {
    fn idn(&self) -> Option<String> {
        Some(format!("Rigol Technologies,{},{}0000000001,00.03.07", self.kind.name, self.kind.series))
    }

    fn command(&mut self, m: &Message<'_>, _: &mut Settings, v: &mut Violations) -> Answer {
        if let Some((_, text)) = self.say.iter().find(|(query, _)| query == &m.header) {
            return Answer::Text(text.clone());
        }
        let is_dg = !self.kind.is_mso;
        match (m.header.as_str(), m.args) {
            ("SYST:KLOC:STATE", "OFF") if is_dg => {
                return Answer::Done;
            }
            ("COUN:STAT?", _) if is_dg => return Answer::Text(self.counter.clone()),
            ("COUN:STAT", "ON") if is_dg => {
                "RUN".clone_into(&mut self.counter);
                return Answer::Done;
            }
            ("COUN:STAT", "OFF") if is_dg => {
                "OFF".clone_into(&mut self.counter);
                return Answer::Done;
            }
            ("COUN:MEAS?", _) if is_dg => return Answer::Text(self.measure()),
            _ => {}
        }
        let header = m.header.clone();
        let (found, is_source) = if let Some(found) = channel_of(&header, "SOUR") {
            (found, true)
        } else if let Some(found) = channel_of(&header, "OUTP").filter(|_| is_dg) {
            (found, false)
        } else {
            return Answer::Unknown;
        };
        let (number, rest) = found;
        if number == 0 || number > usize::from(self.kind.channels) {
            v.report(format!("{}: a {} has {} channels", m.text, self.kind.name, self.kind.channels));
            return Answer::Silent;
        }
        let channel = number - 1;
        match (is_source, rest.strip_prefix(":OUTPUT")) {
            (true, Some(state)) if !is_dg => self.output(channel, state, m),
            (true, _) => self.source(channel, rest, m),
            (false, _) => self.output(channel, rest, m),
        }
    }
}

/// The generator and the shell's options, whichever carrier they go on.
pub(super) fn parts(options: &[(String, String)]) -> Result<(Generator, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut kind = &KINDS[0];
    let (mut counter, mut wave1, mut wave2) = ("OFF".to_owned(), None, None);
    let mut output2 = false;
    let (mut say, mut play) = (Vec::new(), VecDeque::new());
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                kind = KINDS
                    .iter()
                    .find(|m| m.name == v)
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model here")))?;
            }
            "counter" => counter.clone_from(v),
            "wave" => wave1 = Some(v.clone()),
            "wave2" => wave2 = Some(v.clone()),
            "output2" => output2 = v == "1",
            "say" => {
                let (query, text) = v
                    .split_once('=')
                    .ok_or_else(|| Error::Invalid(format!("say={v}: as in COUN:MEAS?=1,2")))?;
                say.push((query.to_ascii_uppercase(), text.replace('~', " ")));
            }
            "play" => play.extend(v.split(';').map(str::to_owned)),
            "remote" => {}
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let position = |name: Option<String>| -> Result<usize> {
        let Some(name) = name else { return Ok(0) };
        kind.waves
            .iter()
            .position(|w| w.name == name)
            .ok_or_else(|| Error::Invalid(format!("wave={name}: a {} has no such", kind.name)))
    };
    let mut channels = vec![Channel::new(position(wave1)?)];
    if kind.channels > 1 {
        channels.push(Channel::new(position(wave2)?));
        channels[1].output = output2;
    }
    let generator =
        Generator { kind, channels, counter, say, play, readings: 0 };
    Ok((generator, config))
}

/// A generator on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (generator, config) = parts(options)?;
    let mut id =
        Identity::new(0x1ab1, generator.kind.usb, "Rigol Technologies", generator.kind.series);
    id.remote = options.iter().any(|(k, v)| k == "remote" && v == "1");
    Ok(config.usbtmc(NAME, &id, generator))
}

/// The same generator on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (generator, config) = parts(options)?;
    Ok(config.tcp(generator))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Violations;

    fn generator(options: &[(&str, &str)]) -> Generator {
        let options: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        parts(&options).unwrap().0
    }

    /// One command straight to the instrument: its answer and what it flagged.
    fn ask(g: &mut Generator, line: &str) -> (Answer, usize) {
        let mut v = Violations::default();
        let (header, args) = line.split_once(' ').unwrap_or((line, ""));
        let m = Message { text: line, header: header.to_ascii_uppercase(), args };
        let answer = g.command(&m, &mut Settings::default(), &mut v);
        (answer, v.drain().len())
    }

    fn text(g: &mut Generator, line: &str) -> String {
        match ask(g, line).0 {
            Answer::Text(t) => t,
            other => panic!("{line}: {other:?}"),
        }
    }

    #[test]
    fn a_number_as_the_guide_writes_it() {
        assert_eq!(sci(1000.0, 6), "1.000000E+03");
        assert_eq!(sci(0.0, 9), "0.000000000E+00");
        assert_eq!(sci(0.000_5, 9), "5.000000000E-04");
    }

    #[test]
    fn apply_says_what_a_waveform_has() {
        let mut g = generator(&[]);
        assert_eq!(
            text(&mut g, "SOUR1:APPL?"),
            "\"SIN,1.000000E+03,5.000000E+00,0.000000E+00,0.000000E+00\""
        );
        assert_eq!(ask(&mut g, "SOUR1:APPL:NOISE").0, Answer::Done);
        assert_eq!(text(&mut g, "SOUR1:APPL?"), "\"NOISE,DEF,5.000000E+00,0.000000E+00,DEF\"");
        assert_eq!(ask(&mut g, "SOUR1:APPL:DC").0, Answer::Done);
        assert_eq!(text(&mut g, "SOUR1:APPL?"), "\"DC,DEF,DEF,0.000000E+00,DEF\"");
        assert_eq!(ask(&mut g, "SOUR1:APPL:SINC").0, Answer::Unknown, "a DG has no SINC");
    }

    #[test]
    fn a_new_waveform_takes_the_defaults_and_keeps_the_output() {
        let mut g = generator(&[]);
        ask(&mut g, "OUTP1:STATE ON");
        ask(&mut g, "SOUR1:FREQ 2000.000000");
        ask(&mut g, "SOUR1:APPL:SQU");
        assert_eq!(text(&mut g, "SOUR1:FREQ?"), "1.000000E+03");
        assert_eq!(text(&mut g, "OUTP1:STATE?"), "ON");
    }

    #[test]
    fn what_a_channel_refuses_changes_nothing() {
        let mut g = generator(&[]);
        for line in ["SOUR1:FREQ 1e12", "SOUR1:FREQ nan", "SOUR1:FREQ abc", "SOUR1:PHAS 400"] {
            assert!(matches!(ask(&mut g, line).0, Answer::Error(-222, _)), "{line}");
        }
        assert_eq!(text(&mut g, "SOUR1:FREQ?"), "1.000000E+03");
        assert_eq!(text(&mut g, "SOUR1:PHAS?"), "0.000000E+00");
    }

    #[test]
    fn the_counter_measures_only_when_on() {
        let mut g = generator(&[]);
        assert_eq!(text(&mut g, "COUN:STAT?"), "OFF");
        assert_eq!(text(&mut g, "COUN:MEAS?"), ["0.000000000E+00"; 5].join(","));
        ask(&mut g, "COUN:STAT ON");
        assert_eq!(text(&mut g, "COUN:STAT?"), "RUN");
        let first = text(&mut g, "COUN:MEAS?");
        assert!(first.starts_with("1.001000000E+03,"), "{first}");
        assert!(text(&mut g, "COUN:MEAS?").starts_with("1.002000000E+03,"));
        ask(&mut g, "COUN:STAT OFF");
        assert_eq!(text(&mut g, "COUN:STAT?"), "OFF");
    }

    #[test]
    fn play_goes_once_and_say_always() {
        let mut g = generator(&[("play", "a;b"), ("counter", "RUN")]);
        assert_eq!(text(&mut g, "COUN:MEAS?"), "a");
        assert_eq!(text(&mut g, "COUN:MEAS?"), "b");
        assert!(text(&mut g, "COUN:MEAS?").starts_with("1.001"));
        let mut g = generator(&[("say", "SOUR1:APPL?=x,y")]);
        assert_eq!(text(&mut g, "SOUR1:APPL?"), "x,y");
        assert_eq!(text(&mut g, "SOUR1:APPL?"), "x,y");
    }

    #[test]
    fn each_flag_fires() {
        let mut g = generator(&[("model", "DG811")]);
        assert_eq!(ask(&mut g, "SOUR2:APPL?"), (Answer::Silent, 1), "one channel");
        assert_eq!(ask(&mut g, "OUTP2:STATE?"), (Answer::Silent, 1));
        assert_eq!(ask(&mut g, "SOUR0:APPL?"), (Answer::Silent, 1));
        assert_eq!(ask(&mut g, "SOUR1:NOPE").0, Answer::Unknown);
        assert_eq!(ask(&mut g, "OUTP1:STATE MAYBE").0, Answer::Unknown);
        assert_eq!(ask(&mut g, "NOPE").0, Answer::Unknown);
    }

    #[test]
    fn an_mso_has_its_own_table() {
        let mut g = generator(&[("model", "MSO5074")]);
        assert_eq!(text(&mut g, "SOUR1:FUNC?"), "SIN");
        assert_eq!(ask(&mut g, "SOUR1:FUNC SINC").0, Answer::Done);
        assert_eq!(text(&mut g, "SOUR1:FUNC?"), "SINC");
        assert_eq!(text(&mut g, "SOUR1:FREQ?"), "1.000000E+03");
        assert_eq!(text(&mut g, "SOUR2:OUTPUT:STATE?"), "0");
        assert_eq!(ask(&mut g, "SOUR2:OUTPUT:STATE ON").0, Answer::Done);
        assert_eq!(text(&mut g, "SOUR2:OUTPUT:STATE?"), "1");
        // A DG's commands aren't an MSO's, and the other way round.
        assert_eq!(ask(&mut g, "SOUR1:APPL?").0, Answer::Unknown);
        assert_eq!(ask(&mut g, "COUN:STAT?").0, Answer::Unknown);
        assert_eq!(ask(&mut g, "SYST:KLOC:STATE OFF").0, Answer::Unknown);
        assert_eq!(ask(&mut g, "OUTP1:STATE?").0, Answer::Unknown);
        let mut g = generator(&[]);
        assert_eq!(ask(&mut g, "SOUR1:FUNC?").0, Answer::Unknown);
        assert_eq!(ask(&mut g, "SOUR1:PULS:DCYC?").0, Answer::Unknown);
    }

    #[test]
    fn it_builds_on_both_carriers() {
        for kind in KINDS {
            let options = [("model".to_owned(), kind.name.to_owned())];
            let m = build(&options).unwrap();
            assert_eq!(m.descriptors().vendor_id(), 0x1ab1);
            assert_eq!(m.descriptors().product_id(), kind.usb);
            assert!(build_tcp(&options).is_ok());
        }
        assert!(build(&[("model".to_owned(), "DG9".to_owned())]).is_err());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
        assert!(build(&[("wave".to_owned(), "SINC".to_owned())]).is_err());
        assert!(build(&[("say".to_owned(), "nothing".to_owned())]).is_err());
    }
}
