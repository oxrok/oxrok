// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An `ICStation` relay board (ICSE012A, ICSE013A or ICSE014A) on the CDC-ACM
//! shell: a one-byte protocol with no framing, at 9600/8n1.
//!
//! The device, and how sure this is. No factory firmware source or dump was
//! found, and joaofl's `usb-relay04-fw` README says the factory image is read
//! protected; that replacement firmware's README and programs written against
//! real units are the sources, read as code where they have any:
//!
//! - The USB side: sigrok's wiki and `icsectl`'s udev rule have a PL2303,
//!   `067b:2303`, which `pl2303` would bind. See the artifacts below.
//! - The line: 9600/8n1, the driver's default, and what those programs open.
//! - After power on, `50` asks for the board's id, answered with one byte (`ab`
//!   ICSE012A with 4 relays, `ad` ICSE013A with 2, `ac` ICSE014A with 8), and
//!   `51` enters command mode, with no answer. `50` can come again before that
//!   (`mrkaleBashScripts`' `icse_init.sh` sends four). g33k's sketch says that
//!   until a `50` the board answers to nothing, so a `51` before one is ignored.
//! - In command mode every byte is the state of all relays, bit 0 for the first.
//!   Nothing leaves this mode but a power cycle, so a later `50` is a relay byte
//!   too (the driver's comment, the wiki, `usb-relay04-fw`'s README, the code of
//!   `brightlight`, the `delendanet` post): the board switches, stays silent, and
//!   a host that is identifying it times out. A second `51` is one as well.
//! - A bit at 0 means on, as the driver has it and as every source on the
//!   ICSE012A and ICSE014A says. The sources on the ICSE013A (`brightlight`,
//!   `icserelay`, the `delendanet` post) have a 1 for on, which no recording here
//!   settles. The polarity only decides what a byte does to the board and nothing
//!   on the wire, so the model keeps the driver's on all three.
//! - Bits above the relays do nothing: `delendanet` and `brightlight` say so for
//!   the ICSE013A, and Tasmota's driver sends them to all boards.
//! - Relays at power on: `icsectl` and an `EventGhost` plugin say all are on, the
//!   driver assumes all off, and no source measures it. The model starts off, and
//!   no case can see it.
//!
//! Emulation artifacts:
//!
//! - The USB side is a CDC-ACM function with the shell's descriptors and test
//!   ids, not a PL2303, as the tools see a tty either way.
//! - It answers at once. `late=` delays an answer by a count of the shell's
//!   offers, not by a clock.
//! - `started=1` stands for a board that an earlier run left in command mode:
//!   the harness replaces the model before each tool, so no case can leave it
//!   there from one tool to the next.
//! - `answer=` and `silent=1` are a board gone wrong, or a device of another kind
//!   behind the port.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `board=ICSE012A|ICSE013A|ICSE014A`, by default the first;
//! - `answer=HEX`, the bytes answered to `50` in place of the id, 1 to 8 of them;
//! - `silent=1`, no answer to `50`, with the rest as usual;
//! - `late=N`, an answer held for N offers;
//! - `started=1`, command mode from power on;
//! - `uart=LINE`, by default 9600/8n1.
//!
//! The pedantic checks, for what no path of the C driver sends: outside command
//! mode, a byte that is neither `50` nor `51`, or a `51` before any `50`. In
//! command mode nothing is flagged: every byte is a state a real one takes.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, Port, Serial};
use crate::{Error, Result};
use std::fmt::Write as _;

/// The model name.
pub const NAME: &str = "icstation-usbrelay";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An ICStation ICSE012A, ICSE013A or ICSE014A relay board on CDC-ACM; \
                         board=, answer=, silent=, late=, started=, uart=, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// Asks for the id.
const IDENTIFY: u8 = 0x50;
/// Enters command mode.
const START: u8 = 0x51;
/// The longest `answer=`.
const LONGEST_ANSWER: usize = 8;

/// A board by its name: its id and its relays.
fn board(name: &str) -> Option<(u8, u32)> {
    match name {
        "ICSE012A" => Some((0xab, 4)),
        "ICSE013A" => Some((0xad, 2)),
        "ICSE014A" => Some((0xac, 8)),
        _ => None,
    }
}

/// A board from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn hex_bytes(key: &str, value: &str) -> Result<Vec<u8>> {
    let bad = || Error::Invalid(format!("{key}={value}: 1 to {LONGEST_ANSWER} bytes in hex"));
    if value.is_empty() || !value.len().is_multiple_of(2) || value.len() > LONGEST_ANSWER * 2 {
        return Err(bad());
    }
    (0..value.len())
        .step_by(2)
        .map(|i| {
            value.get(i..i + 2).and_then(|b| u8::from_str_radix(b, 16).ok()).ok_or_else(bad)
        })
        .collect()
}

fn function(options: &[(String, String)]) -> Result<Cdc<Board>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut config = Config { id: vec![0xab], count: 4, late: 0, started: false };
    let mut answer = None;
    let mut silent = false;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "board" => {
                let (id, count) = board(v)
                    .ok_or_else(|| Error::Invalid(format!("{NAME} has no board {v}")))?;
                config.id = vec![id];
                config.count = count;
            }
            "answer" => answer = Some(hex_bytes(k, v)?),
            "silent" => silent = parse_number(k, v, 0..=1)? == 1,
            "late" => config.late = parse_number(k, v, 1..=10_000)?,
            "started" => config.started = parse_number(k, v, 0..=1)? == 1,
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    if let Some(a) = answer {
        config.id = a;
    }
    if silent {
        config.id.clear();
    }
    // Test ids: see the module's note on the USB side.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "ICStation relay board stand-in");
    Ok(Cdc::new(NAME, &id, faults, Board::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
struct Config {
    /// What `50` gets.
    id: Vec<u8>,
    count: u32,
    late: u32,
    started: bool,
}

impl Config {
    fn mask(&self) -> u8 {
        u8::try_from((1u32 << self.count) - 1).expect("at most 8 relays")
    }
}

struct Board {
    config: Config,
    /// In command mode.
    started: bool,
    /// A `50` has come.
    identified: bool,
    /// The relays, 1 for on.
    relays: u8,
    /// An answer waits for this many offers. Each `50` restarts it and they
    /// don't add up, as a late answer is one answer however often it is asked.
    pending: Option<u32>,
}

impl Board {
    fn new(config: Config) -> Board {
        Board {
            started: config.started,
            identified: false,
            relays: 0,
            pending: None,
            config,
        }
    }

    fn switch(&mut self, byte: u8) {
        self.relays = !byte & self.config.mask();
    }

    fn byte(&mut self, byte: u8, port: &mut Port<'_>) {
        if self.started {
            self.switch(byte);
            return;
        }
        match byte {
            IDENTIFY => {
                self.identified = true;
                if self.config.late > 0 {
                    self.pending = Some(self.config.late);
                } else {
                    port.send(&self.config.id);
                }
            }
            START => {
                if self.identified {
                    self.started = true;
                } else {
                    port.violation(format!("{}: command mode before an id", show(byte)));
                }
            }
            _ => port.violation(format!("{}: no such request", show(byte))),
        }
    }
}

fn show(byte: u8) -> String {
    let mut s = String::new();
    let _ = write!(s, "{byte:02x}");
    s
}

impl Serial for Board {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for b in data {
            self.byte(*b, port);
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some(n) = self.pending {
            if n > 1 {
                self.pending = Some(n - 1);
            } else {
                self.pending = None;
                port.send(&self.config.id);
            }
        }
    }

    fn reset(&mut self) {
        *self = Board::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    fn opened(options: &[(&str, &str)]) -> Host<Board> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Board>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn each_board_says_its_id() {
        for (name, id, mask) in
            [("ICSE012A", 0xab, 0x0f), ("ICSE013A", 0xad, 0x03), ("ICSE014A", 0xac, 0xff)]
        {
            let mut h = opened(&[("board", name)]);
            h.write(&[IDENTIFY]);
            assert_eq!(h.read(), [id], "{name}");
            assert_eq!(h.cdc.model.config.mask(), mask, "{name}");
            assert_eq!(violations(&mut h), 0, "{name}");
        }
        assert!(function(&[("board".to_owned(), "ICSE099".to_owned())]).is_err());
    }

    #[test]
    fn an_id_can_be_asked_again_before_the_start() {
        let mut h = opened(&[]);
        h.write(&[IDENTIFY]);
        h.write(&[IDENTIFY]);
        assert_eq!(h.read(), [0xab, 0xab]);
        assert!(!h.cdc.model.started);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn relay_bytes_are_active_low_and_no_answer() {
        let mut h = opened(&[("board", "ICSE014A")]);
        h.write(&[IDENTIFY, START]);
        assert_eq!(h.read(), [0xac]);
        assert_eq!(h.cdc.model.relays, 0, "the start switches nothing");
        h.write(&[0xfe]);
        assert_eq!(h.cdc.model.relays, 0x01);
        h.write(&[0x00]);
        assert_eq!(h.cdc.model.relays, 0xff);
        h.write(&[0xff]);
        assert_eq!(h.cdc.model.relays, 0x00);
        assert_eq!(h.read(), [0u8; 0]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn nothing_leaves_command_mode_and_an_id_request_switches() {
        let mut h = opened(&[("started", "1")]);
        h.write(&[IDENTIFY]);
        assert_eq!(h.read(), [0u8; 0]);
        assert_eq!(h.cdc.model.relays, 0x0f, "50's low bits are all 0, so all four go on");
        h.write(&[START]);
        assert_eq!(h.read(), [0u8; 0]);
        assert_eq!(h.cdc.model.relays, 0x0e, "51's low bit keeps R1 off");
        assert_eq!(violations(&mut h), 0, "the C driver sends these, and it cannot refuse");
    }

    #[test]
    fn bits_above_the_relays_do_nothing() {
        let mut h = opened(&[("started", "1")]);
        h.write(&[0xf5]);
        assert_eq!(h.cdc.model.relays, 0x0a);
        let mut h = opened(&[("started", "1"), ("board", "ICSE013A")]);
        h.write(&[0xfe]);
        assert_eq!(h.cdc.model.relays, 0x01);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn what_no_c_path_sends_before_the_start_is_flagged() {
        let mut h = opened(&[]);
        h.write(&[0x0f]);
        assert_eq!(violations(&mut h), 1);
        assert!(!h.cdc.model.started);
        h.write(&[START]);
        assert_eq!(violations(&mut h), 1, "a start before any id");
        assert!(!h.cdc.model.started, "and it is ignored");
    }

    #[test]
    fn answers_can_be_wrong_or_missing() {
        let mut h = opened(&[("answer", "12ab")]);
        h.write(&[IDENTIFY]);
        assert_eq!(h.read(), [0x12, 0xab]);
        let mut h = opened(&[("silent", "1")]);
        h.write(&[IDENTIFY]);
        assert_eq!(h.read(), [0u8; 0]);
        assert_eq!(violations(&mut h), 0);
        for bad in ["", "1", "zz", "001122334455667788"] {
            assert!(hex_bytes("answer", bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn a_late_answer_waits_for_its_offers() {
        let mut h = opened(&[("late", "3")]);
        h.write(&[IDENTIFY]);
        assert!(matches!(h.offer(), Reply::Nak));
        assert!(matches!(h.offer(), Reply::Nak));
        assert_eq!(h.read(), [0xab]);
        assert!(h.cdc.model.pending.is_none());
    }

    #[test]
    fn the_uart_holds_a_wrong_line() {
        let o = [("uart".to_owned(), "1200/8n1".to_owned())];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.write(&[IDENTIFY]);
        assert_eq!(h.read(), [0u8; 0]);
    }

    #[test]
    fn a_reset_is_power_on() {
        let mut h = opened(&[]);
        h.write(&[IDENTIFY, START, 0x00]);
        assert_eq!(h.cdc.model.relays, 0x0f);
        h.cdc.model.reset();
        assert_eq!(h.cdc.model.relays, 0);
        assert!(!h.cdc.model.started);
    }
}
