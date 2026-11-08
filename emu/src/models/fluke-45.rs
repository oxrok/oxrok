// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Fluke 45 dual display multimeter on its RS-232 port, on the CDC-ACM shell.
//!
//! The device, and how sure this is. The users manual (PN 855981, rev. 4, chapter 5,
//! pages as printed):
//!
//! - 5-2, Table 5-1: 9600 baud, no parity, 8 bits, 1 stop bit, and echo On at the
//!   factory: "each command sent to the meter ... is echoed".
//! - 5-4: `^C` is a device clear, "causing `=>` followed by a carriage return and line
//!   feed to be output"; 5-20 has "`<CR><LF>` and a new prompt". After each command
//!   the meter sends one of three prompts: `=>` done, `?>` a Command Error (not
//!   understood), `!>` an Execution Error (understood, not done).
//! - 5-11: an answer ends in CR LF. "Wait for the meter to return a prompt before
//!   sending another command", or the second string is a command error and is
//!   discarded. Readings are `+1.2345E+0`, and `+1E+9` and `-1E+9` are positive and
//!   negative overload.
//! - 5-22: `FUNC1?` answers a mnemonic, and `FUNC2?` with the second display off is
//!   an Execution Error. AACDC, VACDC and CONT are on the primary display only, and
//!   with AACDC or VACDC there is no second display.
//! - 5-23: `MOD?` answers a sum of 1 MN, 2 MX, 4 HOLD, 8 dB, 16 dB Power, 32 REL, 64
//!   COMP. In dB Power the primary display is in watts.
//! - 5-24: `AUTO?` answers 1 or 0. 5-26: `VAL1?` answers what the display shows; on a
//!   blank display it waits for the next triggered measurement. Slow is 2.5 readings
//!   a second. FORMAT 2 adds a unit (`+1.2345E+0 VDC`), which this model doesn't do.
//!
//! No firmware was found, nor a recording.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id: a real meter has a D-sub and
//!   whatever cable the host has.
//! - An answer and its prompt go a few offers apart ([`Answer::Apart`]), where a real
//!   one sends them about 5 ms apart at 9600 baud. libsigrok takes a read that ends in
//!   LF for a whole answer, so sent together C reads `VDC\r\n=>` and has no function,
//!   and C's own `protocol.h` says "Always USB-serial", where an adapter's latency
//!   timer (16 ms on an FTDI) usually puts the two in one read: the apart model is
//!   the case C needs, not the likely one. `prompts=together` is the other.
//! - Numbers don't move: a reading comes from the list, and no measurement takes time
//!   unless `latency=` or `trigger=` says.
//! - Commands no driver path sends are flagged and answered `?>`, though the meter has
//!   them.
//!
//! Options, besides those of [`scpi::Config`] (`early=discard` is the manual's word on a
//! command before the prompt, `echo=` the meter's echo):
//!
//! - `func=NAME`, `func2=NAME` or `none`, the two displays, refused where the manual
//!   says no meter can be so;
//! - `auto=0|1`, `mod=N`;
//! - `values=A,B,...`, `values2=A,B,...`, the readings in turn, coming round;
//! - `say=N:TEXT`, the Nth query (from 1) answers TEXT and a prompt, or TEXT alone
//!   when that is `!>`, `?>` or `=>`; once each, and any number of them;
//! - `prompts=apart|together|none`, how the prompt goes; `idn=TEXT`;
//! - `latency=T`, every answer T offers late; `trigger=T`, a reading T offers late, as
//!   on an external trigger; `blocked=T`, at power on a reading is on its way, due in T
//!   offers, and what comes before it is discarded, as a meter a host left in `VAL1?`.

use crate::Error;
use crate::Result;
use crate::model::{DeviceModel, Violations};
use crate::options::parse_number;
use crate::shells::cdc::Identity;
use crate::shells::scpi::{self, Answer, Instrument, Message, Settings};

/// The model name.
pub const NAME: &str = "fluke-45";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Fluke 45 multimeter on CDC-ACM; func=, func2=, auto=, mod=, values=, \
                         values2=, say=, prompts=, idn=, latency=, trigger=, blocked=, and the \
                         SCPI shell's mute-at=, silent-at=, late-at=, mark-at=, line-gap=, \
                         early=, echo=, and stall=, chunk=, in-limit=, in-stall=, out-hold=, \
                         uart=";

/// As the manual gives it, with a serial number and versions of the form.
const IDN: &str = "FLUKE, 45, 7654321, 1.6 D1.6";
/// The manual's two examples, then two more of other ranges.
const VALUES: &str = "+1.2345E+0,+6.7890E+3,-2.5000E-3,+3.0000E+2";
const VALUES2: &str = "+6.0000E+1,+5.9990E+1";
/// What `FUNC1?` can answer: Table 5-11's mnemonics.
const FUNCTIONS: [&str; 10] =
    ["AAC", "AACDC", "ADC", "CONT", "DIODE", "FREQ", "OHMS", "VAC", "VACDC", "VDC"];
/// What the primary display alone has (5-22).
const PRIMARY_ONLY: [&str; 3] = ["AACDC", "CONT", "VACDC"];
/// The most `say=` a model holds.
const MOST_SAID: usize = 64;
/// What a meter left in `VAL1?` answers when it comes.
const BLOCKED_READING: &str = "+9.9999E+0";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Prompts {
    Apart,
    Together,
    None,
}

/// A meter.
struct Meter {
    idn: String,
    func: String,
    func2: Option<String>,
    auto: bool,
    modifiers: u32,
    values: Vec<String>,
    values2: Vec<String>,
    next: usize,
    next2: usize,
    said: Vec<(u32, String)>,
    prompts: Prompts,
    latency: u32,
    trigger: u32,
    queries: u32,
}

impl Meter {
    /// An answer and what follows it, `late` offers on.
    fn answer(&self, text: String, late: u32) -> Answer {
        // Apart too: it may follow an earlier answer's prompt.
        let lines = if matches!(text.as_str(), "!>" | "?>" | "=>") {
            vec![text]
        } else {
            match self.prompts {
                Prompts::Apart => vec![text, "=>".to_owned()],
                Prompts::Together => vec![format!("{text}\r\n=>")],
                Prompts::None => vec![text],
            }
        };
        Answer::After(late, lines)
    }

    fn take(list: &[String], next: &mut usize) -> String {
        let value = list[*next % list.len()].clone();
        *next += 1;
        value
    }
}

impl Instrument for Meter {
    /// `^C`, a device clear (5-4).
    fn control(&mut self, byte: u8) -> Option<Answer> {
        (byte == 0x03).then(|| match self.prompts {
            Prompts::None => Answer::Silent,
            _ => Answer::Apart(vec!["=>".to_owned()]),
        })
    }

    fn command(&mut self, m: &Message<'_>, _s: &mut Settings, v: &mut Violations) -> Answer {
        self.queries += 1;
        let n = self.queries;
        let mut late = self.latency;
        // A reading is taken whatever is said for it, so that the next is the next.
        let text = match m.header.as_str() {
            "*IDN?" => self.idn.clone(),
            "FUNC1?" => self.func.clone(),
            "FUNC2?" => self.func2.clone().unwrap_or_else(|| "!>".to_owned()),
            "AUTO?" => u32::from(self.auto).to_string(),
            "MOD?" => self.modifiers.to_string(),
            "VAL1?" => {
                late += self.trigger;
                Meter::take(&self.values, &mut self.next)
            }
            "VAL2?" if self.func2.is_some() => {
                late += self.trigger;
                Meter::take(&self.values2, &mut self.next2)
            }
            "VAL2?" => "!>".to_owned(),
            _ => {
                v.report(format!("{}: no driver path sends this", m.text));
                "?>".to_owned()
            }
        };
        if let Some(at) = self.said.iter().position(|(at, _)| *at == n) {
            let (_, said) = self.said.remove(at);
            return self.answer(said, late);
        }
        self.answer(text, late)
    }
}

fn function(key: &str, value: &str) -> Result<String> {
    if FUNCTIONS.contains(&value) {
        Ok(value.to_owned())
    } else {
        Err(Error::Invalid(format!("{key}={value}: one of {}", FUNCTIONS.join(" "))))
    }
}

fn values(key: &str, value: &str) -> Result<Vec<String>> {
    let list: Vec<String> = value.split(',').map(str::to_owned).collect();
    if list.iter().any(String::is_empty) {
        return Err(Error::Invalid(format!("{key}={value}: an empty reading")));
    }
    Ok(list)
}

fn parts(options: &[(String, String)]) -> Result<(Meter, scpi::Config)> {
    let mut meter = Meter {
        idn: IDN.to_owned(),
        func: "VDC".to_owned(),
        func2: None,
        auto: true,
        modifiers: 0,
        values: values("values", VALUES)?,
        values2: values("values2", VALUES2)?,
        next: 0,
        next2: 0,
        said: Vec::new(),
        prompts: Prompts::Apart,
        latency: 0,
        trigger: 0,
        queries: 0,
    };
    let mut config = scpi::Config::default().with_eol(b"\r\n");
    let mut blocked = None;
    let ticks = |k: &str, v: &str| parse_number(k, v, 0..=1 << 20);
    for (k, v) in options {
        match k.as_str() {
            "idn" => meter.idn.clone_from(v),
            "func" => meter.func = function(k, v)?,
            "func2" if v == "none" => meter.func2 = None,
            "func2" => meter.func2 = Some(function(k, v)?),
            "auto" => meter.auto = parse_number(k, v, 0..=1)? == 1,
            "mod" => meter.modifiers = parse_number(k, v, 0..=127)?,
            "values" => meter.values = values(k, v)?,
            "values2" => meter.values2 = values(k, v)?,
            "latency" => meter.latency = ticks(k, v)?,
            "trigger" => meter.trigger = ticks(k, v)?,
            "blocked" => blocked = Some(ticks(k, v)?),
            "say" => {
                let (at, text) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("say={v}: as in 3:!>")))?;
                if meter.said.len() >= MOST_SAID {
                    return Err(Error::Invalid(format!("more than {MOST_SAID} say=")));
                }
                meter.said.push((parse_number(k, at, 1..=1 << 24)?, text.to_owned()));
            }
            "prompts" => {
                meter.prompts = match v.as_str() {
                    "apart" => Prompts::Apart,
                    "together" => Prompts::Together,
                    "none" => Prompts::None,
                    _ => {
                        return Err(Error::Invalid(format!(
                            "prompts={v}: apart, together or none"
                        )));
                    }
                };
            }
            _ => {
                if !config.option(k, v)? {
                    return Err(Error::Invalid(format!("{NAME} has no option {k}")));
                }
            }
        }
    }
    if let Some(func2) = &meter.func2 {
        if PRIMARY_ONLY.contains(&func2.as_str()) {
            return Err(Error::Invalid(format!("func2={func2}: the primary display's alone")));
        }
        if matches!(meter.func.as_str(), "AACDC" | "VACDC") {
            return Err(Error::Invalid(format!(
                "func={} has no second display, so no func2={func2}",
                meter.func
            )));
        }
    }
    if let Some(ticks) = blocked {
        config = config.with_pending(&[BLOCKED_READING, "=>"], ticks).with_early_discard();
    }
    Ok((meter, config))
}

/// A meter on a tty, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (meter, config) = parts(options)?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Fluke 45 on a serial port");
    Ok(config.cdc(NAME, &id, meter))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::EP_DATA;
    use crate::shells::cdc::host::setup;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    /// What the host sees: the model through its `DeviceModel`, one offer at a time.
    struct Rig(Box<dyn DeviceModel>, Violations);

    impl Rig {
        fn new(list: &[(&str, &str)]) -> Rig {
            let mut rig = Rig(build(&opts(list)).unwrap(), Violations::default());
            let _ = rig.0.control(setup(0x21, 0x22, 3, 0), &[], &mut rig.1);
            rig
        }

        fn write(&mut self, text: &str) {
            let _ = self.0.out(EP_DATA, text.as_bytes(), &mut self.1);
        }

        /// The transfers of the next `n` offers.
        fn offers(&mut self, n: usize) -> Vec<String> {
            (0..n)
                .filter_map(|_| match self.0.in_transfer(EP_DATA, 128, &mut self.1) {
                    Reply::Data(d) => Some(String::from_utf8(d).unwrap()),
                    _ => None,
                })
                .collect()
        }

        /// How many offers until something comes, up to `most`.
        fn quiet_for(&mut self, most: usize) -> usize {
            (0..most).take_while(|_| self.offers(1).is_empty()).count()
        }
    }

    #[test]
    fn an_answer_and_its_prompt_come_apart_and_the_next_waits_behind() {
        let mut r = Rig::new(&[]);
        r.write("FUNC1?\n");
        r.write("AUTO?\n");
        assert_eq!(r.offers(40), ["VDC\r\n", "=>\r\n", "1\r\n", "=>\r\n"]);
    }

    #[test]
    fn the_second_display_off_is_an_execution_error() {
        let mut r = Rig::new(&[]);
        r.write("FUNC2?\n");
        assert_eq!(r.offers(10), ["!>\r\n"]);
        r.write("VAL2?\n");
        assert_eq!(r.offers(10), ["!>\r\n"]);
        let mut r = Rig::new(&[("func2", "FREQ")]);
        r.write("FUNC2?\n");
        r.write("VAL2?\n");
        assert_eq!(r.offers(40), ["FREQ\r\n", "=>\r\n", "+6.0000E+1\r\n", "=>\r\n"]);
    }

    #[test]
    fn a_refusal_behind_a_prompt_is_a_line_of_its_own() {
        let mut r = Rig::new(&[]);
        r.write("AUTO?\n");
        r.write("FUNC2?\n");
        assert_eq!(r.offers(40), ["1\r\n", "=>\r\n", "!>\r\n"]);
    }

    #[test]
    fn readings_come_round_and_the_manuals_idn_has_spaces() {
        let mut r = Rig::new(&[("values", "+1E+0,+2E+0")]);
        for expected in ["+1E+0\r\n", "+2E+0\r\n", "+1E+0\r\n"] {
            r.write("VAL1?\n");
            assert_eq!(r.offers(10), [expected, "=>\r\n"]);
        }
        r.write("*IDN?\n");
        assert_eq!(r.offers(10)[0], format!("{IDN}\r\n"));
        assert_eq!(r.1.drain().len(), 0);
    }

    #[test]
    fn together_and_none() {
        let mut r = Rig::new(&[("prompts", "together")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["1\r\n=>\r\n"]);
        let mut r = Rig::new(&[("prompts", "none")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["1\r\n"]);
        r.write("\x03");
        assert_eq!(r.offers(10).len(), 0);
    }

    #[test]
    fn say_stands_in_once_and_a_reading_is_still_taken() {
        let mut r = Rig::new(&[("say", "1:!>"), ("say", "2:rubbish")]);
        r.write("VAL1?\n");
        assert_eq!(r.offers(10), ["!>\r\n"]);
        r.write("VAL1?\n");
        assert_eq!(r.offers(10), ["rubbish\r\n", "=>\r\n"]);
        r.write("VAL1?\n");
        assert_eq!(r.offers(10), ["-2.5000E-3\r\n", "=>\r\n"]);
    }

    #[test]
    fn what_no_driver_sends_is_flagged_and_refused() {
        let mut r = Rig::new(&[]);
        r.write("VDC\n");
        assert_eq!(r.offers(10), ["?>\r\n"]);
        assert_eq!(r.1.drain().len(), 1);
    }

    #[test]
    fn early_discard_has_the_manuals_word_on_a_command_ahead_of_the_prompt() {
        let mut r = Rig::new(&[("early", "discard")]);
        r.write("FUNC1?\n");
        r.write("AUTO?\n");
        assert_eq!(r.offers(40), ["VDC\r\n", "=>\r\n"]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(40), ["1\r\n", "=>\r\n"]);
    }

    #[test]
    fn a_muted_query_says_nothing_not_even_a_prompt() {
        let mut r = Rig::new(&[("mute-at", "2")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["1\r\n", "=>\r\n"]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10).len(), 0);
    }

    #[test]
    fn a_device_clear_drops_what_is_on_its_way_and_says_its_prompt() {
        let mut r = Rig::new(&[("trigger", "50")]);
        r.write("VAL1?\n");
        assert_eq!(r.offers(10).len(), 0);
        r.write("FUN");
        r.write("\x03");
        assert_eq!(r.offers(60), ["=>\r\n"]);
        r.write("FUNC1?\n");
        assert_eq!(r.offers(10), ["VDC\r\n", "=>\r\n"]);
    }

    #[test]
    fn latency_and_trigger_hold_answers_back_and_what_follows_waits() {
        let mut r = Rig::new(&[("latency", "20")]);
        r.write("AUTO?\n");
        assert!(r.quiet_for(100) >= 19);
        let mut r = Rig::new(&[("trigger", "30")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["1\r\n", "=>\r\n"]);
        r.write("VAL1?\n");
        r.write("MOD?\n");
        assert!(r.quiet_for(100) >= 29);
        assert_eq!(r.offers(20), ["=>\r\n", "0\r\n", "=>\r\n"]);
    }

    #[test]
    fn a_meter_left_in_val1_discards_until_it_answers_or_is_cleared() {
        let mut r = Rig::new(&[("blocked", "30")]);
        r.write("*IDN?\n");
        assert!(r.quiet_for(100) >= 29);
        assert_eq!(r.offers(20), ["=>\r\n"]);
        let mut r = Rig::new(&[("blocked", "1000")]);
        r.write("\x03");
        assert_eq!(r.offers(10), ["=>\r\n"]);
        r.write("*IDN?\n");
        assert_eq!(r.offers(10)[0], format!("{IDN}\r\n"));
    }

    #[test]
    fn echo_as_it_came_as_a_line_or_bare() {
        let mut r = Rig::new(&[("echo", "raw")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["AUTO?\n1\r\n", "=>\r\n"]);
        let mut r = Rig::new(&[("echo", "line")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["AUTO?\r\n1\r\n", "=>\r\n"]);
        let mut r = Rig::new(&[("echo", "bare")]);
        r.write("AUTO?\n");
        assert_eq!(r.offers(10), ["AUTO?1\r\n", "=>\r\n"]);
    }

    #[test]
    fn bad_options() {
        for bad in [
            &[("no-such", "1")][..],
            &[("func", "VOLTS")],
            &[("func2", "VOLTS")],
            &[("func2", "CONT")],
            &[("func2", "AACDC")],
            &[("func", "VACDC"), ("func2", "FREQ")],
            &[("func", "AACDC"), ("func2", "VDC")],
            &[("auto", "2")],
            &[("mod", "128")],
            &[("values", "+1E+0,")],
            &[("say", "1")],
            &[("say", "x:y")],
            &[("prompts", "maybe")],
            &[("early", "keep")],
            &[("echo", "loud")],
            &[("line-gap", "0")],
        ] {
            assert!(build(&opts(bad)).is_err(), "{bad:?}");
        }
        assert!(build(&opts(&[("func", "CONT"), ("func2", "FREQ")])).is_ok());
        assert_eq!(build(&[]).unwrap().name(), NAME);
    }
}
