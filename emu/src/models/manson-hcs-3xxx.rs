// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Manson HCS-3xxx switching power supply on the CDC-ACM shell: one output,
//! carriage return terminated ASCII, and every answer ends in `OK\r`.
//!
//! The device, and how sure this is:
//!
//! - The USB side: a Silicon Labs CP2102 (`10c4:ea60`, vendor class interface)
//!   in front of an `ATmega64A` (sigrok wiki's "Manson HCS-3202" and its Info
//!   page, an `lsusb` dump), at 9600/8n1 (the `hcscom` Python project, and an
//!   `EEVblog` user's script for an HCS-3404). So the model sits behind a UART
//!   at that line, and a misset port gets nothing either way.
//! - The commands: the sigrok wiki's "Manson HCS-3xxx series" and that script.
//!   `GMOD` gives a model code then `OK`, `GETD` gives `VVVVCCCCM` then `OK`
//!   (hundredths of a volt, hundredths of an ampere, 0 CV or 1 CC), `SOUTn`,
//!   `VOLTnnn` and `CURRnnn` give `OK`. The wiki warns that the vendor's
//!   manuals are incomplete and in places wrong.
//! - `SOUT0` is on and `SOUT1` off. The wiki's line reads both ways ("Disable
//!   output", "on=0"); the script and libsigrok send 1 for off.
//! - `GMAX`: `VVVCCC`, in the units of `VOLT` and `CURR`. No document at hand
//!   gives it; this is how the libsigrok driver reads it.
//! - The model codes: `3202` and the like, `HCS-3202` on later firmware
//!   (the wiki; libsigrok commit 6d8205a, "a recently bought device").
//! - The ratings: the wiki's table. Which `CURR` has hundredths (`n.nn`) rather
//!   than tenths (`nn.n`) is "model-dependent" there, and is here as the
//!   libsigrok driver has it. `GMAX` saying exactly the rating is a guess:
//!   `vmax=` and `imax=` say otherwise.
//! - The output reads 0 V and 0 A while off: only a comment in the libsigrok
//!   driver says so, and nothing else. `getd-raw=` plays any other reading.
//!
//! Emulation artifacts:
//!
//! - It is a CDC-ACM function and not a CP2102, since the shell is what there
//!   is. The tools see `/dev/ttyACMn` where a real one is `/dev/ttyUSBn`, and
//!   none of `cp210x`'s vendor requests is on the wire. The ids are not the
//!   real ones either, as `cp210x` binds those.
//! - It answers at once, in the round of the request. No document gives a real
//!   one's latency, and a model has no clock.
//! - It takes a command wherever the writes cut it, up to its terminator.
//! - `getd-raw=` and the other `-raw` options are stand-ins for a device gone
//!   wrong; nothing shows that a real one sends such replies.
//!
//! Not modelled: `SESS`, `ENDS` (the wiki), `GETS` (the script) and whatever
//! else the real device has, which no path of the libsigrok driver sends, so
//! they are flagged; what a real one answers to a command it does not know, so
//! such a command gets no answer.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=3100` etc., the table's rating; `3202` by default;
//! - `gmod=TEXT`, the model code, for a table's rating under another name;
//! - `vmax=NNN`, `imax=NNN`, what `GMAX` says and the setters take, in the
//!   units of `VOLT` and `CURR`;
//! - `vset=NNN`, `iset=NNN`, `output=0|1`, the state at power on;
//! - `load=OHMS` on the output, or none; it decides CV or CC;
//! - `gmod-raw=HEX`, `gmax-raw=HEX`, whole replies, terminators included;
//! - `getd-raw=HEX[,HEX...]`, `ack-raw=HEX[,HEX...]`, whole replies played in
//!   turn; an empty one is no reply;
//! - `answers=N`, commands answered per power on, then silence and the event
//!   point;
//! - `gap=N`, the command after N answers gets none and marks the event point,
//!   and the next go on as before: a capture has data on offer after an event;
//! - `setters=0`, which flags any setting, for the cases that show a tool
//!   changes nothing it was not asked to.
//!
//! The pedantic checks: only the commands above, in the format `%03.0f` gives,
//! within the limits and the supply's 1 V floor; nothing left of a command when
//! a tool opens. Three C habits are taken, as nothing shows how a real one
//! refuses them (the driver's range check is two comparisons): `CURR-00`, as 0,
//! and `VOLTnan` and `CURRnan`, and `VOLTinf` and `CURRinf` after a `GMAX` that
//! says `inf`, as nothing. All are answered `OK`, which is a guess.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "manson-hcs-3xxx";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Manson HCS-3xxx power supply on CDC-ACM; model=, gmod=, vmax=, imax=, \
                         vset=, iset=, output=, load=, gmod-raw=, gmax-raw=, getd-raw=, ack-raw=, \
                         answers=, gap=, setters=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// The line the bridge's UART runs at.
const UART: &str = "9600/8n1";
/// What is kept of a command before its terminator. The longest the commands
/// need is 7, and the C habits are as long.
const LONGEST: usize = 15;
/// The largest reply an option may set: past what any driver reads.
const REPLY_MAX: usize = 4096;
/// The lowest `VOLT`, the table's 1 V, in tenths.
const VOLT_FLOOR: u32 = 10;

/// One line of the wiki's table.
#[derive(Clone, Copy, Debug)]
struct Rating {
    code: &'static str,
    /// Tenths of a volt.
    vmax: u32,
    /// In `CURR`'s unit.
    imax: u32,
    /// `CURR` in hundredths of an ampere, not tenths.
    centi_amps: bool,
}

const fn rating(code: &'static str, vmax: u32, imax: u32, centi_amps: bool) -> Rating {
    Rating { code, vmax, imax, centi_amps }
}

const RATINGS: [Rating; 16] = [
    rating("3100", 180, 100, false),
    rating("3102", 360, 500, true),
    rating("3104", 600, 250, true),
    rating("3150", 180, 150, false),
    rating("3200", 180, 200, false),
    rating("3202", 360, 100, false),
    rating("3204", 600, 500, true),
    rating("3300", 160, 300, false),
    rating("3302", 320, 150, false),
    rating("3304", 600, 80, false),
    rating("3400", 160, 400, false),
    rating("3402", 320, 200, false),
    rating("3404", 600, 100, false),
    rating("3600", 160, 600, false),
    rating("3602", 300, 300, false),
    rating("3604", 600, 150, false),
];

/// A supply from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn one_reply(key: &str, value: &str) -> Result<Vec<u8>> {
    Ok(parse_hex_list(key, value, 0, REPLY_MAX)?.remove(0))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    // The rating first, whatever the order: the others stand on it.
    for (k, v) in options {
        if k == "model" {
            let rating = RATINGS.iter().find(|r| r.code == v);
            let rating = rating.ok_or_else(|| Error::Invalid(format!("model={v}: not in the table")))?;
            config.rating = *rating;
            rating.code.clone_into(&mut config.gmod);
        }
    }
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => {}
            "gmod" => v.clone_into(&mut config.gmod),
            "vmax" => config.rating.vmax = parse_number(k, v, 0..=999)?,
            "imax" => config.rating.imax = parse_number(k, v, 0..=999)?,
            "vset" => config.power_on.vset = parse_number(k, v, 0..=999)?,
            "iset" => config.power_on.iset = parse_number(k, v, 0..=999)?,
            "output" => config.power_on.output = parse_number(k, v, 0..=1)? == 1,
            "load" => config.load = Some(parse_number(k, v, 1..=1_000_000)?),
            "gmod-raw" => config.gmod_raw = Some(one_reply(k, v)?),
            "gmax-raw" => config.gmax_raw = Some(one_reply(k, v)?),
            "getd-raw" => {
                config.getd_raw = Some(Packets::new(parse_hex_list(k, v, 0, REPLY_MAX)?));
            }
            "ack-raw" => {
                config.ack_raw = Some(Packets::new(parse_hex_list(k, v, 0, REPLY_MAX)?));
            }
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "gap" => config.gap = Some(parse_number(k, v, 0..=100_000)?),
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // A CP2102's strings (the wiki's `lsusb`), not its ids; see the head.
    let id = Identity::new(0x1209, 0x0001, "Silicon Labs", "CP2102 USB to UART Bridge Controller");
    Ok(Cdc::new(NAME, &id, faults, Supply::new(config)).behind_uart(Line::parse(UART)?))
}

#[derive(Clone, Copy, Debug)]
struct Panel {
    /// Tenths of a volt.
    vset: u32,
    /// In `CURR`'s unit.
    iset: u32,
    output: bool,
}

#[derive(Clone, Debug)]
struct Config {
    rating: Rating,
    gmod: String,
    power_on: Panel,
    load: Option<u32>,
    gmod_raw: Option<Vec<u8>>,
    gmax_raw: Option<Vec<u8>>,
    getd_raw: Option<Packets>,
    ack_raw: Option<Packets>,
    answers: Option<u32>,
    gap: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        let rating = rating("3202", 360, 100, false);
        Config {
            rating,
            gmod: rating.code.to_owned(),
            power_on: Panel { vset: 120, iset: 50, output: false },
            load: None,
            gmod_raw: None,
            gmax_raw: None,
            getd_raw: None,
            ack_raw: None,
            answers: None,
            gap: None,
            setters: true,
        }
    }
}

struct Supply {
    config: Config,
    now: Panel,
    getd_raw: Option<Packets>,
    ack_raw: Option<Packets>,
    /// What has come of a command so far.
    pending: Vec<u8>,
    answered: u32,
    gap_taken: bool,
}

impl Supply {
    fn new(config: Config) -> Supply {
        Supply {
            now: config.power_on,
            getd_raw: config.getd_raw.clone(),
            ack_raw: config.ack_raw.clone(),
            config,
            pending: Vec::new(),
            answered: 0,
            gap_taken: false,
        }
    }

    /// Volts and amperes in hundredths and whether it is in constant current,
    /// from the settings and the load.
    fn operating_point(&self) -> (u64, u64, bool) {
        if !self.now.output {
            return (0, 0, false);
        }
        let volts = u64::from(self.now.vset) * 10;
        let unit = if self.config.rating.centi_amps { 1 } else { 10 };
        let limit = u64::from(self.now.iset) * unit;
        let Some(load) = self.config.load.map(u64::from) else { return (volts, 0, false) };
        let wanted = volts / load;
        if wanted <= limit { (volts, wanted, false) } else { ((limit * load).min(volts), limit, true) }
    }

    fn getd(&mut self) -> Vec<u8> {
        if let Some(list) = &mut self.getd_raw {
            return list.take();
        }
        let (volts, amps, cc) = self.operating_point();
        format!("{volts:04}{amps:04}{}\rOK\r", u8::from(cc)).into_bytes()
    }

    fn gmax(&self) -> Vec<u8> {
        if let Some(raw) = &self.config.gmax_raw {
            return raw.clone();
        }
        let rating = &self.config.rating;
        format!("{:03}{:03}\rOK\r", rating.vmax, rating.imax).into_bytes()
    }

    fn gmod(&self) -> Vec<u8> {
        if let Some(raw) = &self.config.gmod_raw {
            return raw.clone();
        }
        format!("{}\rOK\r", self.config.gmod).into_bytes()
    }

    fn ack(&mut self) -> Vec<u8> {
        match &mut self.ack_raw {
            Some(list) => list.take(),
            None => b"OK\r".to_vec(),
        }
    }

    /// A setting's effect, or what is wrong with it.
    fn set(&mut self, command: &str) -> std::result::Result<(), String> {
        let (name, value) = command.split_at(4);
        if name == "SOUT" {
            match value {
                "0" => self.now.output = true,
                "1" => self.now.output = false,
                _ => return Err("not 0 or 1".to_owned()),
            }
            return Ok(());
        }
        // The C habits, see the head.
        if value == "nan" || value == "inf" {
            return Ok(());
        }
        if name == "CURR" && value == "-00" {
            self.now.iset = 0;
            return Ok(());
        }
        let number = three_digits(value).ok_or("not three digits")?;
        if name == "VOLT" {
            if !(VOLT_FLOOR..=self.config.rating.vmax).contains(&number) {
                return Err("outside what the supply takes".to_owned());
            }
            self.now.vset = number;
        } else {
            if number > self.config.rating.imax {
                return Err("above what the supply takes".to_owned());
            }
            self.now.iset = number;
        }
        Ok(())
    }

    fn command(&mut self, command: &[u8], port: &mut Port<'_>) {
        let text = String::from_utf8_lossy(command).into_owned();
        let setting = ["SOUT", "VOLT", "CURR"].iter().any(|n| text.starts_with(n));
        if !setting && !matches!(command, b"GMOD" | b"GETD" | b"GMAX") {
            port.violation(format!("'{}': no such command", show(command)));
            return;
        }
        if setting {
            if !self.config.setters {
                port.violation(format!("'{text}': a setting no one asked for"));
            }
            if let Err(why) = self.set(&text) {
                port.violation(format!("'{text}': {why}"));
                return;
            }
        }
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        if self.config.gap == Some(self.answered) && !self.gap_taken {
            self.gap_taken = true;
            port.mark();
            return;
        }
        self.answered += 1;
        let reply = match command {
            b"GMOD" => self.gmod(),
            b"GETD" => self.getd(),
            b"GMAX" => self.gmax(),
            _ => self.ack(),
        };
        if !reply.is_empty() {
            port.send(&reply);
        }
    }
}

fn three_digits(text: &str) -> Option<u32> {
    (text.len() == 3 && text.bytes().all(|c| c.is_ascii_digit()))
        .then(|| text.parse().expect("digits"))
}

fn show(bytes: &[u8]) -> String {
    bytes.escape_ascii().to_string()
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\r' {
                let command = std::mem::take(&mut self.pending);
                self.command(&command, port);
            } else if self.pending.len() < LONGEST {
                self.pending.push(byte);
            } else {
                // The rest of it, up to its terminator, is one more of these.
                port.violation(format!("'{}': too long", show(&self.pending)));
                self.pending.clear();
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("'{}': a command left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn reset(&mut self) {
        *self = Supply::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn supply(options: &[(&str, &str)]) -> Host<Supply> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse(UART).unwrap(), true, true)
    }

    fn violations(h: &mut Host<Supply>) -> usize {
        h.v.drain().len()
    }

    fn ask(h: &mut Host<Supply>, command: &[u8]) -> Vec<u8> {
        h.write(command);
        h.read()
    }

    #[test]
    fn the_queries_answer_and_end_in_ok() {
        let mut h = supply(&[]);
        assert_eq!(ask(&mut h, b"GMOD\r"), b"3202\rOK\r");
        assert_eq!(ask(&mut h, b"GETD\r"), b"000000000\rOK\r", "the output is off");
        assert_eq!(ask(&mut h, b"GMAX\r"), b"360100\rOK\r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn each_rating_says_its_limits() {
        for (code, gmax) in [("3104", &b"600250\rOK\r"[..]), ("3304", b"600080\rOK\r")] {
            let mut h = supply(&[("model", code)]);
            assert_eq!(ask(&mut h, b"GMOD\r"), format!("{code}\rOK\r").as_bytes());
            assert_eq!(ask(&mut h, b"GMAX\r"), gmax);
        }
    }

    #[test]
    fn another_name_for_a_rating() {
        let mut h = supply(&[("gmod", "HCS-3104"), ("model", "3104")]);
        assert_eq!(ask(&mut h, b"GMOD\r"), b"HCS-3104\rOK\r");
        assert_eq!(ask(&mut h, b"GMAX\r"), b"600250\rOK\r");
    }

    #[test]
    fn settings_take_effect_and_answer_ok() {
        let mut h = supply(&[("load", "10")]);
        assert_eq!(ask(&mut h, b"VOLT050\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"CURR025\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"GETD\r"), b"000000000\rOK\r");
        assert_eq!(ask(&mut h, b"SOUT0\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"GETD\r"), b"050000500\rOK\r", "5 V on 10 ohms");
        assert_eq!(ask(&mut h, b"SOUT1\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"GETD\r"), b"000000000\rOK\r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_load_too_heavy_is_constant_current() {
        let mut h = supply(&[("load", "1"), ("output", "1"), ("vset", "120"), ("iset", "20")]);
        assert_eq!(ask(&mut h, b"GETD\r"), b"020002001\rOK\r");
    }

    #[test]
    fn hundredths_of_an_ampere_on_the_models_that_count_so() {
        let o = [("model", "3102"), ("load", "1"), ("output", "1"), ("vset", "120"), ("iset", "200")];
        let mut h = supply(&o);
        assert_eq!(ask(&mut h, b"GETD\r"), b"020002001\rOK\r", "CURR200 is 2.00 A here");
    }

    #[test]
    fn a_command_may_come_in_pieces_or_together() {
        let mut h = supply(&[]);
        h.write(b"GM");
        h.write(b"OD");
        assert_eq!(h.read(), b"");
        h.write(b"\rGMAX\rGETD\r");
        assert_eq!(h.read(), b"3202\rOK\r360100\rOK\r000000000\rOK\r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_no_c_path_sends_is_flagged_and_unanswered() {
        for bad in [
            &b"SESS\r"[..],
            b"ENDS\r",
            b"GETS\r",
            b"GOVP\r",
            b"GMOD\n",
            b"gmod\r",
            b"\r",
            b"GMOD \r",
        ] {
            let mut h = supply(&[]);
            h.write(bad);
            h.write(b"\r");
            assert!(violations(&mut h) >= 1, "{}", show(bad));
            assert_eq!(h.read(), b"", "{}", show(bad));
        }
    }

    #[test]
    fn a_value_in_another_format_is_flagged_and_changes_nothing() {
        for bad in [
            &b"VOLT50\r"[..],
            b"VOLT0500\r",
            b"VOLT05x\r",
            b"VOLT009\r",
            b"VOLT361\r",
            b"VOLT-00\r",
            b"CURR101\r",
            b"CURR5\r",
            b"SOUT2\r",
            b"SOUT\r",
        ] {
            let mut h = supply(&[("output", "1"), ("load", "10")]);
            let before = ask(&mut h, b"GETD\r");
            assert_eq!(ask(&mut h, bad), b"", "{}", show(bad));
            assert_eq!(violations(&mut h), 1, "{}", show(bad));
            assert_eq!(ask(&mut h, b"GETD\r"), before);
        }
    }

    #[test]
    fn the_limits_are_options() {
        let mut h = supply(&[("vmax", "362"), ("imax", "102")]);
        assert_eq!(ask(&mut h, b"GMAX\r"), b"362102\rOK\r");
        h.write(b"VOLT362\rCURR102\r");
        assert_eq!(violations(&mut h), 0);
        h.write(b"VOLT363\rCURR103\r");
        assert_eq!(violations(&mut h), 2);
    }

    #[test]
    fn the_c_habits_are_taken() {
        let mut h = supply(&[("output", "1"), ("load", "10")]);
        assert_eq!(ask(&mut h, b"VOLTnan\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"CURRnan\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"VOLTinf\r"), b"OK\r");
        assert_eq!(ask(&mut h, b"CURRinf\r"), b"OK\r");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(ask(&mut h, b"GETD\r"), b"120001200\rOK\r", "nothing changed");
        assert_eq!(ask(&mut h, b"CURR-00\r"), b"OK\r");
        assert_eq!(violations(&mut h), 0);
        assert_eq!(ask(&mut h, b"GETD\r"), b"000000001\rOK\r", "no current allowed");
    }

    #[test]
    fn setters_0_flags_each_setting() {
        for setting in [&b"SOUT0\r"[..], b"SOUT1\r", b"VOLT050\r", b"CURR010\r"] {
            let mut h = supply(&[("setters", "0")]);
            h.write(setting);
            assert_eq!(violations(&mut h), 1, "{}", show(setting));
        }
        let mut h = supply(&[("setters", "0")]);
        h.write(b"GMOD\rGETD\rGMAX\r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_is_kept_of_a_command_stays_short() {
        let mut h = supply(&[]);
        h.write(&[b'x'; 4096]);
        assert!(h.cdc.model.pending.len() <= LONGEST);
        assert!(violations(&mut h) > 0);
    }

    #[test]
    fn a_command_left_unfinished_shows_at_the_next_open() {
        let mut h = supply(&[]);
        h.write(b"GET");
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = supply(&[("answers", "1")]);
        assert_eq!(ask(&mut h, b"GMOD\r"), b"3202\rOK\r");
        assert!(!h.cdc.mark());
        assert_eq!(ask(&mut h, b"GETD\r"), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_gap_misses_one_answer_and_marks_the_point() {
        let mut h = supply(&[("gap", "1")]);
        assert_eq!(ask(&mut h, b"GMOD\r"), b"3202\rOK\r");
        assert!(!h.cdc.mark());
        assert_eq!(ask(&mut h, b"GETD\r"), b"");
        assert!(h.cdc.mark());
        assert_eq!(ask(&mut h, b"GETD\r"), b"000000000\rOK\r", "and on it goes");
        assert!(!h.cdc.mark());
    }

    #[test]
    fn raw_replies_play_in_turn() {
        let o = [
            ("gmod-raw", "78"),
            ("gmax-raw", "79"),
            ("getd-raw", "61,,62"),
            ("ack-raw", "63,64"),
        ];
        let mut h = supply(&o);
        assert_eq!(ask(&mut h, b"GMOD\r"), b"x");
        assert_eq!(ask(&mut h, b"GMAX\r"), b"y");
        assert_eq!(ask(&mut h, b"GETD\r"), b"a");
        assert_eq!(ask(&mut h, b"GETD\r"), b"", "an empty one is no reply");
        assert_eq!(ask(&mut h, b"GETD\r"), b"b");
        assert_eq!(ask(&mut h, b"GETD\r"), b"a");
        assert_eq!(ask(&mut h, b"SOUT0\r"), b"c");
        assert_eq!(ask(&mut h, b"SOUT1\r"), b"d");
    }

    #[test]
    fn it_sits_behind_a_uart_at_its_line() {
        let mut h = supply(&[]);
        assert_eq!(ask(&mut h, b"GMOD\r"), b"3202\rOK\r");
        h.set_line(Line::parse("19200/8n1").unwrap());
        assert_eq!(ask(&mut h, b"GMOD\r"), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = supply(&[("getd-raw", "61,62"), ("answers", "3")]);
        ask(&mut h, b"VOLT050\r");
        assert_eq!(ask(&mut h, b"GETD\r"), b"a");
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.now.vset, 120);
        assert_eq!(ask(&mut h, b"GETD\r"), b"a");
        assert_eq!(h.cdc.model.answered, 1);
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("model", "3999"),
            ("vmax", "1000"),
            ("iset", "x"),
            ("output", "2"),
            ("load", "0"),
            ("gmod-raw", "4"),
            ("getd-raw", "zz"),
            ("answers", "x"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("model", "3604"), ("gmod", "x"), ("getd-raw", "00"), ("chunk", "1")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
