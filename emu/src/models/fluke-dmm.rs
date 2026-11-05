// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Fluke handheld meter (87, 89, 187, 189, 287, 289) or `ScopeMeter` (199B)
//! behind its infrared cable, on the CDC-ACM shell. Commands end in CR, and
//! every answer starts with a one digit acknowledge and CR; 0 is followed by
//! the data and CR, anything else by nothing.
//!
//! The device, and how sure this is:
//!
//! - The cable: Fluke's IR189USB and its kin are USB to serial adapters, so
//!   a real one is a `ttyUSBn` and the meter sits behind a UART. This model
//!   gives it a CDC-ACM cable, which shows the same line settings to the
//!   tools. The ids and strings are not a real cable's.
//! - The 87, 89, 187, 189: "Fluke 189/187/89-IV/87-IV Remote Interface
//!   Specification" (a copy at photonicsguy.ca): 9600/8n1, `ID` answers
//!   `FLUKE 89,V0.39,123456789`, `QM` answers `QM,` and the primary display
//!   as text. The copy's own examples (`+47.66 KOhms`, `-121.43 VDC`, `Out of
//!   Range mVDC`) disagree with its list of units (`V DC`, `Ohms`, `mS`) and
//!   the text the driver looks for (`V DC`, `Out of range`, `ms`). No
//!   recording was found. The default reading follows the list; the
//!   scenario plays the others through `qm=`.
//! - The 287, 289: 115200/8n1 (a script for the IR cable on `EEVblog`,
//!   "Going further with the Fluke 289"); `ID` as `FLUKE 289,V1.00,95680259`
//!   (`wasle/Fluke289`); `QM` as `1.0327E0,ADC,NORMAL,NONE`
//!   (`mikedanylov/meaplot-Fluke289`). Fluke's "289/287 Remote Interface
//!   Specification" was seen only as a search engine's summary of a copy:
//!   acknowledges 0, 1, 2 and 5, and overload as
//!   `+9.99999999E+37,VDC,OL,NONE`.
//! - The 199B: "Fluke 190 Family Remote Control & Programming Reference":
//!   1200/8n1 at power on, `ID` answers `MODEL;VERSION;DATE;LANGUAGES`, `QM`
//!   answers sets of `NO,VALID,SOURCE,UNIT,TYPE,PRES,RESOL`, and `QM 11` a
//!   value such as `+1.234E+00`. Reading 11 is the meter's absolute reading
//!   and 19 its relative one. The reference has no example of a whole line;
//!   the identity string's model number as `FLUKE 199B` is what the driver
//!   looks for.
//!
//! Emulation artifacts:
//!
//! - It answers at once. A real meter takes its time (the driver allows 150
//!   ms for the acknowledge and 850 ms for the rest), and at 1200 Bd a line
//!   takes tens of milliseconds to cross.
//! - `id=`, `qm=` and `data=` play whole answers, acknowledge and CR
//!   included, as given. A real meter's come from its display; these stand in
//!   for a display that changes, a mode switch and a line gone wrong.
//!
//! Not modelled: `DS`, `RI`, `SF` and the rest of what a real meter takes,
//! which no path of the driver sends, so they are flagged; the `ScopeMeter`'s
//! `PC` rate change and its scope mode other than as a `qm=` answer; a meter
//! that answers a wrong line with noise rather than silence.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=87|89|187|189|199B|287|289` (default 287), the identity, the line
//!   and the answers;
//! - `uart=LINE|native`, the line instead of the model's;
//! - `id=HEX[,HEX...]`, answers to `ID` played in turn, or `id=none` for no
//!   answer and the event point;
//! - `qm=HEX[,HEX...]`, answers to `QM` played in turn;
//! - `data=HEX[,HEX...]`, answers to `QM N` played in turn;
//! - `answers=N`, queries other than `ID` answered per power on, then silence
//!   and the event point.
//!
//! The pedantic checks: only `ID`, `QM` and, on a `ScopeMeter`, `QM 11` and
//! `QM 19`, each with its CR; no `ID` after a `QM` (the scan comes first and
//! no path goes back); no `QM` before an `ID`; `QM N` only of a reading the
//! last `QM` answer listed; nothing left of a command when a tool opens.
//!
//! What the scenario's cases leave unreached of the C driver: the write of
//! the first `QM` failing (`api.c:220-221`) and of a `QM N` (`protocol.c:73`),
//! as an unplug straight before a write is a race (`--unplug out=` fires at
//! a completed transfer, and the next may or may not be accepted); the two
//! null checks of `fluke_receive_data` (`protocol.c:98`, `101`), which no
//! session reaches; and `parse_measurement_state`'s `NULL` (`fluke-28x.c:137`),
//! from a line with no state, which the scenario runs under oxrok alone as the
//! read a line later crashes sigrok-cli in about half of the runs.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "fluke-dmm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Fluke 87/89/187/189/199B/287/289 behind an IR cable on CDC-ACM; \
                         model=, uart=, id=HEX|none, qm=HEX, data=HEX, answers=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// The longest answer an option may play: past the driver's 255 byte line.
const LONGEST_ANSWER: usize = 1024;
/// The longest command the driver sends is `QM 19` and its CR.
const LONGEST_COMMAND: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Family {
    /// 87, 89, 187, 189: the display as text.
    Display,
    /// 199B: what is shown, then its value.
    Scope,
    /// 287, 289: value, unit, state, attribute.
    Fields,
}

/// A meter's identity, its line and its family, from `model=`.
fn profile(model: &str) -> Result<(&'static str, &'static str, Family)> {
    Ok(match model {
        "87" => ("0\rFLUKE 87,V1.00,12345678\r", "9600/8n1", Family::Display),
        "89" => ("0\rFLUKE 89,V0.39,123456789\r", "9600/8n1", Family::Display),
        "187" => ("0\rFLUKE 187,V1.10,12345678\r", "9600/8n1", Family::Display),
        "189" => ("0\rFLUKE 189,V2.00,12345678\r", "9600/8n1", Family::Display),
        "199B" => ("0\rFLUKE 199B;V11.40;2008-06-11;ENGLISH\r", "1200/8n1", Family::Scope),
        "287" => ("0\rFLUKE 287,V1.16,95680258\r", "115200/8n1", Family::Fields),
        "289" => ("0\rFLUKE 289,V1.00,95680259\r", "115200/8n1", Family::Fields),
        _ => return Err(Error(format!("model={model}: 87, 89, 187, 189, 199B, 287 or 289"))),
    })
}

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn answers_list(key: &str, value: &str) -> Result<Packets> {
    Ok(Packets::new(parse_hex_list(key, value, 1, LONGEST_ANSWER)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut model = "287";
    let mut uart = None;
    let mut id = None;
    let mut qm = None;
    let mut data = None;
    let mut answers = None;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "model" => model = v,
            "uart" => uart = Some(v.as_str()),
            "id" if v == "none" => id = Some(None),
            "id" => id = Some(Some(answers_list(k, v)?)),
            "qm" => qm = Some(answers_list(k, v)?),
            "data" => data = Some(answers_list(k, v)?),
            "answers" => answers = Some(parse_number(k, v, 0..=100_000)?),
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let (identity, line, family) = profile(model)?;
    let config = Config {
        family,
        id: id.unwrap_or_else(|| Some(Packets::new(vec![identity.as_bytes().to_vec()]))),
        qm: qm.unwrap_or_else(|| Packets::new(vec![default_qm(family).to_vec()])),
        data: data.unwrap_or_else(|| Packets::new(vec![b"0\r+1.234E+00\r".to_vec()])),
        answers,
    };
    let cable = Identity::new(0x1209, 0x0001, "oxrok-emu", "Fluke IR cable");
    let cdc = Cdc::new(NAME, &cable, faults, Meter::new(config));
    Ok(match uart.unwrap_or(line) {
        "native" => cdc,
        line => cdc.behind_uart(Line::parse(line)?),
    })
}

fn default_qm(family: Family) -> &'static [u8] {
    match family {
        Family::Display => b"0\rQM,+12.345 V DC\r",
        Family::Scope => b"0\r11,1,3,1,1,0,1.0E-03\r",
        Family::Fields => b"0\r1.0327E0,ADC,NORMAL,NONE\r",
    }
}

#[derive(Clone, Debug)]
struct Config {
    family: Family,
    id: Option<Packets>,
    qm: Packets,
    data: Packets,
    answers: Option<u32>,
}

struct Meter {
    config: Config,
    id: Option<Packets>,
    qm: Packets,
    data: Packets,
    /// What has come of a command so far.
    pending: Vec<u8>,
    id_asked: bool,
    qm_asked: bool,
    /// The last `QM` answer, for the readings it lists.
    listed: Vec<u8>,
    answered: u32,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            id: config.id.clone(),
            qm: config.qm.clone(),
            data: config.data.clone(),
            config,
            pending: Vec::new(),
            id_asked: false,
            qm_asked: false,
            listed: Vec::new(),
            answered: 0,
        }
    }

    /// Whether a query gets its answer.
    fn answers_left(&mut self, port: &mut Port<'_>) -> bool {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark_event_point();
            return false;
        }
        self.answered += 1;
        true
    }

    fn identify(&mut self, port: &mut Port<'_>) {
        if self.qm_asked {
            port.violation("ID after a QM");
        }
        self.id_asked = true;
        match &mut self.id {
            Some(list) => port.send(&list.take()),
            None => port.mark_event_point(),
        }
    }

    fn query(&mut self, port: &mut Port<'_>) {
        if !self.id_asked {
            port.violation("QM before an ID");
        }
        self.qm_asked = true;
        if self.answers_left(port) {
            let answer = self.qm.take();
            port.send(&answer);
            self.listed = answer;
        }
    }

    fn reading(&mut self, number: &[u8], port: &mut Port<'_>) {
        if self.config.family != Family::Scope {
            port.violation("QM N on a meter that has no readings to number");
        }
        if !lists(&self.listed, number) {
            port.violation(format!(
                "QM {}: not in the last QM's answer",
                number.escape_ascii()
            ));
        }
        if self.answers_left(port) {
            let answer = self.data.take();
            port.send(&answer);
        }
    }

    fn command(&mut self, command: &[u8], port: &mut Port<'_>) {
        match command {
            b"ID" => self.identify(port),
            b"QM" => self.query(port),
            b"QM 11" => self.reading(b"11", port),
            b"QM 19" => self.reading(b"19", port),
            other => port.violation(format!("'{}': no such command", other.escape_ascii())),
        }
    }
}

/// Whether a `QM` answer has a set of seven starting with this reading.
fn lists(answer: &[u8], number: &[u8]) -> bool {
    let data = answer.strip_prefix(b"0\r").unwrap_or(answer);
    let data = data.strip_suffix(b"\r").unwrap_or(data);
    data.split(|&b| b == b',').step_by(7).any(|n| n == number)
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte == b'\r' {
                let command = std::mem::take(&mut self.pending);
                self.command(&command, port);
            } else if self.pending.len() < LONGEST_COMMAND {
                self.pending.push(byte);
            } else {
                port.violation(format!(
                    "'{}': a command too long",
                    self.pending.escape_ascii()
                ));
                self.pending.clear();
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!(
                "'{}': a command left unfinished",
                self.pending.escape_ascii()
            ));
            self.pending.clear();
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn at(line: &str, options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse(line).unwrap(), true, true)
    }

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        at("115200/8n1", options)
    }

    fn violations(h: &mut Host<Meter>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn each_model_has_its_identity_and_line() {
        for (model, line, id) in [
            ("87", "9600/8n1", &b"0\rFLUKE 87,V1.00,12345678\r"[..]),
            ("89", "9600/8n1", b"0\rFLUKE 89,V0.39,123456789\r"),
            ("187", "9600/8n1", b"0\rFLUKE 187,V1.10,12345678\r"),
            ("189", "9600/8n1", b"0\rFLUKE 189,V2.00,12345678\r"),
            ("199B", "1200/8n1", b"0\rFLUKE 199B;V11.40;2008-06-11;ENGLISH\r"),
            ("287", "115200/8n1", b"0\rFLUKE 287,V1.16,95680258\r"),
            ("289", "115200/8n1", b"0\rFLUKE 289,V1.00,95680259\r"),
        ] {
            let mut h = at(line, &[("model", model)]);
            h.write(b"ID\r");
            assert_eq!(h.read(), id, "{model}");
            assert_eq!(violations(&mut h), 0);
        }
    }

    #[test]
    fn nothing_crosses_a_wrong_line() {
        let mut h = at("9600/8n1", &[("model", "287")]);
        h.write(b"ID\r");
        assert_eq!(h.read(), b"");
        assert_eq!(violations(&mut h), 0);
        let mut h = at("9600/8n1", &[("model", "287"), ("uart", "native")]);
        h.write(b"ID\r");
        assert_eq!(h.read(), b"0\rFLUKE 287,V1.16,95680258\r");
        let mut h = at("9600/8n1", &[("model", "287"), ("uart", "9600/8n1")]);
        h.write(b"ID\r");
        assert_eq!(h.read(), b"0\rFLUKE 287,V1.16,95680258\r");
    }

    #[test]
    fn each_family_answers_qm_its_own_way() {
        let mut h = at("9600/8n1", &[("model", "187")]);
        h.write(b"ID\r");
        h.read();
        h.write(b"QM\r");
        assert_eq!(h.read(), b"0\rQM,+12.345 V DC\r");
        let mut h = meter(&[]);
        h.write(b"ID\r");
        h.read();
        h.write(b"QM\r");
        assert_eq!(h.read(), b"0\r1.0327E0,ADC,NORMAL,NONE\r");
        let mut h = at("1200/8n1", &[("model", "199B")]);
        h.write(b"ID\r");
        h.read();
        h.write(b"QM\r");
        assert_eq!(h.read(), b"0\r11,1,3,1,1,0,1.0E-03\r");
        h.write(b"QM 11\r");
        assert_eq!(h.read(), b"0\r+1.234E+00\r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn answers_play_in_turn() {
        let mut h = meter(&[("id", "310d,300d"), ("qm", "300d610d,300d620d")]);
        h.write(b"ID\r");
        assert_eq!(h.read(), b"1\r");
        h.write(b"ID\r");
        assert_eq!(h.read(), b"0\r");
        for answer in [&b"0\ra\r"[..], b"0\rb\r", b"0\ra\r"] {
            h.write(b"QM\r");
            assert_eq!(h.read(), answer);
        }
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_command_may_come_in_pieces() {
        let mut h = meter(&[]);
        h.write(b"I");
        h.write(b"D");
        assert_eq!(h.read(), b"");
        h.write(b"\r");
        assert_eq!(h.read(), b"0\rFLUKE 287,V1.16,95680258\r");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn silence_marks_the_event_point() {
        let mut h = meter(&[("id", "none")]);
        h.write(b"ID\r");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.take_event_point());
        let mut h = meter(&[("answers", "1")]);
        h.write(b"ID\r");
        h.read();
        assert!(!h.cdc.take_event_point(), "ID does not count");
        h.write(b"QM\r");
        assert_eq!(h.read(), b"0\r1.0327E0,ADC,NORMAL,NONE\r");
        assert!(!h.cdc.take_event_point());
        h.write(b"QM\r");
        assert_eq!(h.read(), b"");
        assert!(h.cdc.take_event_point());
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn answers_counts_numbered_queries() {
        let mut h = at("1200/8n1", &[("model", "199B"), ("answers", "1")]);
        h.write(b"ID\r");
        h.read();
        h.write(b"QM\r");
        h.read();
        h.write(b"QM 11\r");
        assert_eq!(h.read(), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn an_unknown_command_is_flagged() {
        let mut h = meter(&[]);
        h.write(b"RI\r");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn id_after_a_qm_is_flagged() {
        let mut h = meter(&[]);
        h.write(b"ID\r");
        h.write(b"QM\r");
        assert_eq!(violations(&mut h), 0);
        h.write(b"ID\r");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn qm_before_an_id_is_flagged() {
        let mut h = meter(&[]);
        h.write(b"QM\r");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_numbered_query_on_a_handheld_is_flagged() {
        let mut h = meter(&[("qm", "300d31312c310d")]);
        h.write(b"ID\r");
        h.write(b"QM\r");
        assert_eq!(violations(&mut h), 0);
        h.write(b"QM 11\r");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_reading_not_listed_is_flagged() {
        let mut h = at("1200/8n1", &[("model", "199B")]);
        h.write(b"ID\r");
        h.write(b"QM 11\r");
        assert_eq!(violations(&mut h), 1, "no QM yet");
        h.write(b"QM\r");
        h.write(b"QM 11\r");
        assert_eq!(violations(&mut h), 0);
        h.write(b"QM 19\r");
        assert_eq!(violations(&mut h), 1, "only 11 is listed");
    }

    #[test]
    fn the_second_of_two_sets_counts_but_not_their_other_fields() {
        assert!(lists(b"0\r11,1,3,1,1,0,1\r", b"11"));
        assert!(lists(b"0\r11,1,3,1,1,0,1,19,1,3,1,1,1,1\r", b"19"));
        assert!(!lists(b"0\r11,19,3,1,1,0,1\r", b"19"));
    }

    #[test]
    fn a_command_too_long_is_flagged_and_bounded() {
        let mut h = meter(&[]);
        h.write(b"QMQMQMQ");
        assert_eq!(violations(&mut h), 1);
        assert!(h.cdc.model.pending.len() <= LONGEST_COMMAND);
    }

    #[test]
    fn a_command_left_unfinished_is_flagged_at_the_next_open() {
        let mut h = meter(&[]);
        h.write(b"QM");
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = meter(&[("id", "300d,310d")]);
        h.write(b"ID\r");
        assert_eq!(h.read(), b"0\r");
        h.cdc.model.reset();
        h.write(b"ID\r");
        assert_eq!(h.read(), b"0\r");
    }

    #[test]
    fn bad_options_are_refused() {
        for bad in [
            ("model", "88"),
            ("uart", "fast"),
            ("id", "xx"),
            ("qm", ""),
            ("answers", "x"),
            ("nosuch", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(function(&o).is_err(), "{bad:?}");
        }
    }
}
