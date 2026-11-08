// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Siglent SDL1000X series electronic load (the SDL1020X, SDL1030X and their -E
//! versions) on USBTMC and on TCP.
//!
//! The device, and how sure this is. The manual is the maker's SDL1000X Programming
//! Guide (`siglentna.com/download/8968`, sha256 `bd3ec689...`), by PDF page:
//!
//! - USB and LAN are its carriers, the LAN with VXI-11, sockets and telnet (7). A
//!   serial port it has none of, so this model is on no CDC-ACM.
//! - `*IDN?`: `Siglent Technologies,SDL1020X,0123456789,1.01.01.15` (15; the page has a
//!   full stop after it, taken for the sentence's).
//! - `MEASure:VOLTage?`: `7.924678`, `MEASure:CURRent?`: `3.986634`, `MEASure:POWer?`:
//!   `31.867329`, `MEASure:RESistance?`: `5.842789`, so six places (17). The manual
//!   gives no time for a query, so a pace is only `pace=`.
//! - `:INPut?`: `1` or `0` (18). `:FUNCtion?`: the long word, `RESISTANCE` (19), and
//!   `LED` is the fifth mode (19), which C's list lacks.
//! - `:CURRent:LEVel?`: `2.674`, so three places (20).
//! - `:CURRent:PROTection:LEVel?` and `:POWer:PROTection:LEVel?`: `8.000`, a number,
//!   where `api.c:265` reads it as a boolean (58, 59).
//! - `:POWer:PROTection:STATe?`: `1`, "whether the protection switch is enabled" (59).
//!   The manual's page for the current's state prints the wrong command (58). The
//!   driver calls this "active" and answers "enabled" with a constant.
//! - LF ends an answer: a Python driver written against a real unit
//!   (JxR-TestMeasure/siglent_sdl1000x) sets it, on VISA. The manual doesn't say.
//!
//! What's a guess, which no test should lean on:
//!
//! - f4ec:1621 is from memory and unverified: Siglent's vendor id, and a product id
//!   no source found has (sigrok's `60-libsigrok.rules` lists f4ec:ee38 and ee3a only,
//!   so a real load would get no udev rule from it), and nothing here checks. The vendor id puts it on both of `scpi_usbtmc_libusb.c`'s tables, whose
//!   comments say "all Siglent SDS": no remote control, and a USB reset at each open.
//!   The high speed, the serial number and the firmware of the other models.
//! - That a load refuses an out of range number with an error in its queue and keeps
//!   the old value, which the manual doesn't describe; no driver reads the queue.
//! - That it answers `CURR:PROT:STAT?`, which only the wrong page stands for.
//!
//! Emulation artifacts: measurements are a slope per query that starts over at a
//! reset, and the current is the setting while the input is on in CC mode, or nearly
//! 0. No voltage source, no regulation.
//!
//! Options, besides the shell's ([`scpi::Config::option`], `pace=` among them):
//!
//! - `model=NAME`: `SDL1020X-E` (the default), `SDL1020X`, `SDL1030X-E`, `SDL1030X`;
//! - `name=TEXT`: what `*IDN?` says the model is, with the ids staying;
//! - `firmware=TEXT`, `serial=TEXT`;
//! - `volt=A,B,...` and `curr=A,B,...`: what `MEAS:VOLT?` and `MEAS:CURR?` answer first,
//!   each once, the measurements after. An empty entry is an empty line;
//! - `func=TEXT`, `input=TEXT`, `ocp-state=TEXT`, `opp-state=TEXT`, `ocp-level=TEXT`,
//!   `opp-level=TEXT`, `volt-level=TEXT`: an answer at power on, as is.
//!
//! The pedantic checks: anything but the spellings `api.c` and `protocol.c` send is
//! the shell's unknown command. C's `:FUNC Unknown` and its out of range numbers are
//! no violations: the load can refuse them, and does.

use std::collections::VecDeque;

use crate::model::{DeviceModel, Violations};
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings};
use crate::shells::tcp::Stream;
use crate::shells::usbtmc::Identity;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "siglent-sdl10x0";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Siglent SDL1000X electronic loads on USBTMC and TCP; model=, name=, \
                         firmware=, serial=, volt=, curr=, func=, input=, ocp-state=, opp-state=, ocp-level=, \
                         opp-level=, volt-level=, and the SCPI shell's mute-at=, silent-at=, \
                         late-at=, pace=, split-at=, double-at=, junk-at=, mark-at=, idn=, eol=, \
                         stall=, in-stall=, empty-at=, \
                         bad-tag-at=, message=";

const VENDOR: &str = "Siglent Technologies";
const MODELS: [(&str, f64); 4] =
    [("SDL1020X-E", 200.0), ("SDL1020X", 200.0), ("SDL1030X-E", 300.0), ("SDL1030X", 300.0)];

/// What is asked by one spelling and set by another.
const READ_ONLY: [&str; 8] = [
    "VOLTAGE:LEVEL",
    "CURRENT:LEVEL",
    "POWER:LEVEL",
    "RESISTANCE:LEVEL",
    "POWER:PROTECTION:STATE",
    "POWER:PROTECTION:LEVEL",
    "CURRENT:PROTECTION:STATE",
    "CURRENT:PROTECTION:LEVEL",
];

/// A load.
struct Load {
    name: String,
    firmware: String,
    serial: String,
    max_power: f64,
    volts: VecDeque<String>,
    amps: VecDeque<String>,
    /// How many of each have been measured.
    measured: (u32, u32),
    func: String,
    input: String,
    ocp_state: String,
    opp_state: String,
    ocp_level: String,
    opp_level: String,
    volt_level: String,
}

impl Load {
    fn volts_now(&self) -> f64 {
        12.0 + f64::from(self.measured.0) * 0.001_234
    }

    fn amps_now(&self, s: &Settings) -> f64 {
        let n = f64::from(self.measured.1) * 0.000_321;
        if s.get("INPUT") == Some("1") && s.get("FUNC") == Some("CURRENT") {
            s.number("CURRENT:LEVEL") + n
        } else {
            0.000_012 + n
        }
    }

    /// A number within range is kept with three places, as the manual's replies have
    /// it. Another goes in the error queue, and the old value stays: a guess.
    fn level(m: &Message<'_>, s: &mut Settings, header: &str, range: (f64, f64)) -> Answer {
        match m.args.parse::<f64>() {
            Ok(v) if v.is_finite() && v >= range.0 && v <= range.1 => {
                s.set(header, format!("{v:.3}"));
                Answer::Done
            }
            Ok(v) if v.is_finite() => Answer::Error(-222, "Data out of range".to_owned()),
            _ => Answer::Error(-104, "Data type error".to_owned()),
        }
    }
}

impl Instrument for Load {
    fn idn(&self) -> Option<String> {
        Some(format!("{VENDOR},{},{},{}", self.name, self.serial, self.firmware))
    }

    fn settings(&self) -> Settings {
        Settings::default()
            .with("INPUT", self.input.as_str())
            .with("FUNC", self.func.as_str())
            .with("VOLTAGE:LEVEL", self.volt_level.as_str())
            .with("CURRENT:LEVEL", "1.000")
            .with("POWER:LEVEL", "10.000")
            .with("RESISTANCE:LEVEL", "100.000")
            .with("POWER:PROTECTION:STATE", self.opp_state.as_str())
            .with("POWER:PROTECTION:LEVEL", self.opp_level.as_str())
            .with("CURRENT:PROTECTION:STATE", self.ocp_state.as_str())
            .with("CURRENT:PROTECTION:LEVEL", self.ocp_level.as_str())
    }

    fn command(&mut self, m: &Message<'_>, s: &mut Settings, v: &mut Violations) -> Answer {
        let max_power = self.max_power;
        match m.header.as_str() {
            // A capture's spelling and a get's.
            "MEAS:VOLT?" | "MEAS:VOLTAGE?" => {
                let text = self.volts.pop_front();
                let now = self.volts_now();
                self.measured.0 += 1;
                Answer::Text(text.unwrap_or_else(|| format!("{now:.6}")))
            }
            "MEAS:CURR?" | "MEAS:CURRENT?" => {
                let text = self.amps.pop_front();
                let now = self.amps_now(s);
                self.measured.1 += 1;
                Answer::Text(text.unwrap_or_else(|| format!("{now:.6}")))
            }
            "MEAS:POWER?" => {
                Answer::Text(format!("{:.6}", self.volts_now() * self.amps_now(s)))
            }
            "MEAS:RESISTANCE?" => {
                Answer::Text(format!("{:.6}", self.volts_now() / self.amps_now(s)))
            }
            "INPUT" => match m.args {
                "ON" => {
                    s.set("INPUT", "1");
                    Answer::Done
                }
                "OFF" => {
                    s.set("INPUT", "0");
                    Answer::Done
                }
                _ => Answer::Error(-104, "Data type error".to_owned()),
            },
            "FUNC" => match m.args {
                "CURRENT" | "VOLTAGE" | "POWER" | "RESISTANCE" | "LED" => {
                    s.set("FUNC", m.args);
                    Answer::Done
                }
                // `:FUNC Unknown`, which `string_to_mode` lets through.
                _ => Answer::Error(-224, "Illegal parameter value".to_owned()),
            },
            "VOLT:LEV:IMM" => Load::level(m, s, "VOLTAGE:LEVEL", (0.0, 150.0)),
            "CURR:LEV:IMM" => Load::level(m, s, "CURRENT:LEVEL", (0.0, 30.0)),
            "POW:LEV:IMM" => Load::level(m, s, "POWER:LEVEL", (0.0, max_power)),
            "RES:LEV:IMM" => Load::level(m, s, "RESISTANCE:LEVEL", (0.03, 10000.0)),
            "POW:PROT:LEV" => Load::level(m, s, "POWER:PROTECTION:LEVEL", (0.0, max_power)),
            "CURR:PROT:LEV" => Load::level(m, s, "CURRENT:PROTECTION:LEVEL", (0.0, 30.0)),
            // A get's spelling is only asked: what the Engine has would take a set.
            h if !m.is_query() && READ_ONLY.contains(&h) => {
                v.report(format!("{}: a set by a get's spelling", m.text));
                Answer::Silent
            }
            _ => Answer::Unknown,
        }
    }
}

fn parts(options: &[(String, String)]) -> Result<(Load, scpi::Config)> {
    let mut config = scpi::Config::default();
    let mut load = Load {
        name: MODELS[0].0.to_owned(),
        firmware: "1.01.01.25".to_owned(),
        serial: "SDL2EMU0001".to_owned(),
        max_power: MODELS[0].1,
        volts: VecDeque::new(),
        amps: VecDeque::new(),
        measured: (0, 0),
        func: "CURRENT".to_owned(),
        input: "0".to_owned(),
        ocp_state: "0".to_owned(),
        opp_state: "1".to_owned(),
        ocp_level: "8.000".to_owned(),
        opp_level: "200.000".to_owned(),
        volt_level: "12.000".to_owned(),
    };
    let list = |v: &str| v.split(',').map(str::to_owned).collect::<VecDeque<_>>();
    for (k, v) in options {
        if config.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {
                let (name, power) = MODELS
                    .iter()
                    .find(|(name, _)| name == v)
                    .ok_or_else(|| Error::Invalid(format!("model={v}: no such model")))?;
                (*name).clone_into(&mut load.name);
                load.max_power = *power;
            }
            "name" => v.clone_into(&mut load.name),
            "firmware" => v.clone_into(&mut load.firmware),
            "serial" => v.clone_into(&mut load.serial),
            "volt" => load.volts = list(v),
            "curr" => load.amps = list(v),
            "func" => v.clone_into(&mut load.func),
            "input" => v.clone_into(&mut load.input),
            "ocp-state" => v.clone_into(&mut load.ocp_state),
            "opp-state" => v.clone_into(&mut load.opp_state),
            "ocp-level" => v.clone_into(&mut load.ocp_level),
            "opp-level" => v.clone_into(&mut load.opp_level),
            "volt-level" => v.clone_into(&mut load.volt_level),
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
        }
    }
    Ok((load, config))
}

/// A load on USBTMC, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (load, config) = parts(options)?;
    let id = Identity::new(0xf4ec, 0x1621, VENDOR, &load.name);
    Ok(config.usbtmc(NAME, &id, load))
}

/// The same load on a socket.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    let (load, config) = parts(options)?;
    Ok(config.tcp(load))
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

    /// A line, and the text and the violations it makes.
    fn ask(s: &mut Box<dyn Stream>, line: &str) -> (String, Vec<String>) {
        let mut port = Port::default();
        s.received(line.as_bytes(), &mut port);
        (String::from_utf8(port.sent().concat()).unwrap(), port.violations().drain())
    }

    fn text(s: &mut Box<dyn Stream>, line: &str) -> String {
        let (text, flags) = ask(s, line);
        assert_eq!(flags, Vec::<String>::new(), "{line}");
        text
    }

    #[test]
    fn what_the_manual_shows() {
        let mut s = on_socket(&[]);
        assert_eq!(text(&mut s, "*IDN?\n"), "Siglent Technologies,SDL1020X-E,SDL2EMU0001,1.01.01.25\n");
        assert_eq!(text(&mut s, ":INPUT?\n"), "0\n");
        assert_eq!(text(&mut s, ":FUNC?\n"), "CURRENT\n");
        assert_eq!(text(&mut s, ":CURRENT:LEVel?\n"), "1.000\n");
        assert_eq!(text(&mut s, ":CURRent:PROTection:LEVel?\n"), "8.000\n");
        assert_eq!(text(&mut s, ":POWer:PROTection:LEVel?\n"), "200.000\n");
    }

    #[test]
    fn it_measures_with_six_places_and_each_spelling() {
        let mut s = on_socket(&[]);
        assert_eq!(text(&mut s, "MEAS:VOLT?\n"), "12.000000\n");
        assert_eq!(text(&mut s, "MEAS:VOLTage?\n"), "12.001234\n");
        assert_eq!(text(&mut s, "MEAS:CURR?\n"), "0.000012\n");
        assert_eq!(text(&mut s, "MEAS:CURRent?\n"), "0.000333\n");
        assert_eq!(text(&mut s, "MEAS:POWer?\n"), "0.007850\n");
        assert_eq!(text(&mut s, "MEAS:RESistance?\n").lines().count(), 1);
    }

    #[test]
    fn the_input_on_draws_what_is_set() {
        let mut s = on_socket(&[]);
        assert_eq!(text(&mut s, ":CURR:LEV:IMM 2.500\n"), "");
        assert_eq!(text(&mut s, ":CURRENT:LEVel?\n"), "2.500\n");
        assert_eq!(text(&mut s, ":INPUT ON\n"), "");
        assert_eq!(text(&mut s, ":INPUT?\n"), "1\n");
        assert_eq!(text(&mut s, "MEAS:CURR?\n"), "2.500000\n");
        text(&mut s, ":FUNC VOLTAGE\n");
        assert_eq!(text(&mut s, "MEAS:CURR?\n"), "0.000333\n");
        text(&mut s, ":INPUT OFF\n");
        assert_eq!(text(&mut s, ":INPUT?\n"), "0\n");
    }

    #[test]
    fn a_refused_number_keeps_the_old_one_and_goes_in_the_queue() {
        let mut s = on_socket(&[]);
        assert_eq!(text(&mut s, ":VOLT:LEV:IMM 200.000\n"), "");
        assert_eq!(text(&mut s, ":VOLTage:LEVel?\n"), "12.000\n");
        assert_eq!(text(&mut s, "SYST:ERR?\n"), "-222,\"Data out of range\"\n");
        assert_eq!(text(&mut s, ":VOLT:LEV:IMM nan\n"), "");
        assert_eq!(text(&mut s, "SYST:ERR?\n"), "-104,\"Data type error\"\n");
        assert_eq!(text(&mut s, ":FUNC Unknown\n"), "");
        assert_eq!(text(&mut s, ":FUNC?\n"), "CURRENT\n");
        assert_eq!(text(&mut s, "SYST:ERR?\n"), "-224,\"Illegal parameter value\"\n");
    }

    #[test]
    fn the_big_load_takes_more_power() {
        let mut s = on_socket(&[]);
        text(&mut s, ":POW:LEV:IMM 250.000\n");
        assert_eq!(text(&mut s, ":POWer:LEVel?\n"), "10.000\n");
        let mut s = on_socket(&[("model", "SDL1030X")]);
        text(&mut s, ":POW:LEV:IMM 250.000\n");
        assert_eq!(text(&mut s, ":POWer:LEVel?\n"), "250.000\n");
        text(&mut s, ":POW:PROT:LEV 250.000\n");
        assert_eq!(text(&mut s, ":POWer:PROTection:LEVel?\n"), "250.000\n");
        text(&mut s, ":RES:LEV:IMM 5.000\n");
        assert_eq!(text(&mut s, ":RESistance:LEVel?\n"), "5.000\n");
        text(&mut s, ":CURR:PROT:LEV 5.000\n");
        assert_eq!(text(&mut s, ":CURRent:PROTection:LEVel?\n"), "5.000\n");
    }

    #[test]
    fn a_list_plays_once() {
        let mut s = on_socket(&[("volt", ",abc"), ("curr", "1 A")]);
        assert_eq!(text(&mut s, "MEAS:VOLT?\n"), "\n");
        assert_eq!(text(&mut s, "MEAS:VOLT?\n"), "abc\n");
        assert_eq!(text(&mut s, "MEAS:VOLT?\n"), "12.002468\n");
        assert_eq!(text(&mut s, "MEAS:CURR?\n"), "1 A\n");
        assert_eq!(text(&mut s, "MEAS:CURR?\n"), "0.000333\n");
    }

    #[test]
    fn what_no_driver_sends_is_flagged() {
        let mut s = on_socket(&[]);
        assert_eq!(ask(&mut s, "MEAS:EXT?\n").1.len(), 1);
        assert_eq!(ask(&mut s, ":CURR:LEV 1\n").1.len(), 1);
        assert_eq!(ask(&mut s, ":VOLTage:LEVel 1\n").1.len(), 1, "a set by a get's spelling");
    }

    #[test]
    fn the_options_are_checked() {
        let on = |k: &str, v: &str| vec![(k.to_owned(), v.to_owned())];
        assert!(build(&on("model", "SDL1020X")).is_ok());
        assert!(build(&on("model", "SDL9")).is_err());
        let mut s = on_socket(&[("name", "SDL2000X"), ("firmware", "9"), ("serial", "S")]);
        assert_eq!(text(&mut s, "*IDN?\n"), "Siglent Technologies,SDL2000X,S,9\n");
        assert!(build(&on("no-such", "1")).is_err());
        assert!(build(&on("pace", "x")).is_err());
        let m = build(&[]).unwrap();
        assert_eq!(m.descriptors().vendor_id(), 0xf4ec);
    }
}
