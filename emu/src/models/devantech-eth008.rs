// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Devantech relay board's binary command set, on the CDC-ACM shell: requests
//! of a fixed length by command byte, answers of a fixed length with no framing.
//!
//! The device, and how sure this is:
//!
//! - The commands and their lengths: the vendor's documents for the ETH008-B,
//!   ETH484-B and ETH8020-B, "TCP access and commands", and its Python module
//!   (`devantech_eth`, `module_socket.py`, `eth8020.py`). A set answers 0 for
//!   success and 1 for failure. A get of the outputs is low byte first, a get of
//!   the inputs and of an analog value high byte first. The supply is a byte in
//!   tenths of a volt, the serial number the 6 bytes of the MAC.
//! - Inputs: the ETH484-B's document gives 2 bytes whose first is always 0, the
//!   ETH8020-B's gives 4 whose first three are. Its Python module asks for 2 of
//!   the ETH8020, but `read` is a `recv(2048)` that takes what has come, so that's
//!   a lower bound, and it then indexes the fourth: it fits the document's 4.
//! - The board table (ids, how many outputs, inputs and analog inputs, and the
//!   widths): the ETH documents for 19, 20 and 21, the Python module for 18 to
//!   21. The other ids are what libsigrok's driver says of them. So a WIFI board
//!   or the ETH1610 here is that driver's reading, never checked.
//! - Relays 5 to 8 of an ETH484: its document's "All on = 255 (xxxx1111)". What
//!   the board reads back for them isn't said; they read 0 unless `do=` says.
//! - That a command it doesn't have gets no answer: libsigrok's driver says so of
//!   its firmware, and it is all the model does for one.
//!
//! Emulation artifacts:
//!
//! - **The transport.** These boards are reached over TCP (port 17494 by
//!   default), with a command in one TCP frame. No board with this command set
//!   on a serial port is known: the USB-RLY boards' set is another one. libsigrok
//!   takes either transport, so this is the same bytes through a CDC-ACM
//!   function with a test id, and it has neither a line setting nor a connection
//!   of its own: state outlives a close, as it does a TCP connection's end.
//! - It answers at once. A real board's time is its own, and the shell can't
//!   show a late one; the port's tests do that with a paced fake.
//! - No pulse, no password and no ASCII text set, as no path of the driver sends
//!   them: a request for one is flagged and has no answer.
//! - The inputs and analog readings stay what the options say.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `board=ID`, the module id: 19 (an ETH008) by default. An id that no table
//!   has answers its module info and serial number, and flags the rest;
//! - `hw=`, `fw=`, `mac=` (12 hex digits), `volts=` (tenths), the identity;
//! - `do=`, `di=`, the images at power on, as numbers (`0x0f`);
//! - `ai=`, analog values from the first input on, separated by commas;
//! - `module=`, `serial=`, `outputs=`, `inputs=`, `analog=`, `supply=`, `set=`,
//!   lists of answers separated by commas, played in turn and then again. An
//!   answer is hex digits, `~` for none, or `-` for the board's own;
//! - `answers=N`, commands answered per power on, then silence and the event
//!   point;
//! - `setters=0`, which flags any set.
//!
//! The pedantic checks: only the commands the driver has for this board, a relay
//! that exists, no pulse, an image with no bit beyond its relays, an analog input
//! that exists, nothing left of a command when a tool opens.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, LineEvent, Port, Serial};
use crate::{Error, Result};
use std::fmt::Write;

/// The model name.
pub const NAME: &str = "devantech-eth008";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Devantech relay board's commands on CDC-ACM; board=, hw=, fw=, mac=, \
                         volts=, do=, di=, ai=, module=, serial=, outputs=, inputs=, analog=, \
                         supply=, set= (answers in turn), answers=, setters=, and the shell's \
                         stall=, chunk=, in-limit=, in-stall=, out-hold=";

const GET_MODULE_INFO: u8 = 0x10;
const DIGITAL_ACTIVE: u8 = 0x20;
const DIGITAL_INACTIVE: u8 = 0x21;
const DIGITAL_SET_OUTPUTS: u8 = 0x23;
const DIGITAL_GET_OUTPUTS: u8 = 0x24;
const DIGITAL_GET_INPUTS: u8 = 0x25;
const ANALOG_GET_INPUT: u8 = 0x32;
const GET_SERIAL_NUMBER: u8 = 0x77;
const GET_SUPPLY_VOLTS: u8 = 0x78;

/// What a board has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Board {
    id: u8,
    outputs: u32,
    inputs: u32,
    analog: u32,
    output_bytes: usize,
    input_bytes: usize,
    /// Relays the images have room for, but that the board doesn't have.
    missing: u32,
}

const fn board(
    id: u8,
    (outputs, inputs, analog): (u32, u32, u32),
    (output_bytes, input_bytes): (usize, usize),
    missing: u32,
) -> Board {
    Board { id, outputs, inputs, analog, output_bytes, input_bytes, missing }
}

const BOARDS: [Board; 9] = [
    board(18, (2, 0, 0), (1, 0), 0),
    board(19, (8, 0, 0), (1, 0), 0),
    board(20, (16, 8, 4), (2, 2), 0x00f0),
    board(21, (20, 8, 8), (3, 4), 0),
    board(22, (16, 8, 4), (2, 2), 0x00f0),
    board(24, (20, 8, 8), (3, 4), 0),
    board(26, (2, 0, 0), (1, 0), 0),
    board(28, (8, 0, 0), (1, 0), 0),
    board(52, (10, 16, 16), (2, 2), 0),
];

impl Board {
    /// The outputs as bits.
    fn mask(&self) -> u32 {
        // 20 at most, so the shift holds; a board with no table has 0.
        ((1u32 << self.outputs) - 1) & !self.missing
    }
}

/// An answer in a list.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Answer {
    Own,
    Bytes(Vec<u8>),
}

/// Answers played in turn and then again; none means the board's own.
#[derive(Clone, Debug, Default)]
struct Turns {
    answers: Vec<Answer>,
    next: usize,
}

impl Turns {
    fn parse(key: &str, value: &str) -> Result<Turns> {
        let answers = value
            .split(',')
            .map(|item| match item {
                "-" => Ok(Answer::Own),
                "~" => Ok(Answer::Bytes(Vec::new())),
                hex => parse_hex(key, hex).map(Answer::Bytes),
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(Turns { answers, next: 0 })
    }

    fn take(&mut self, own: Vec<u8>) -> Vec<u8> {
        let Some(answer) = self.answers.get(self.next) else { return own };
        self.next = (self.next + 1) % self.answers.len();
        match answer {
            Answer::Own => own,
            Answer::Bytes(bytes) => bytes.clone(),
        }
    }
}

fn parse_hex(key: &str, text: &str) -> Result<Vec<u8>> {
    let bad = || Error::Invalid(format!("{key}={text}: hex digits in pairs"));
    if text.is_empty() || !text.len().is_multiple_of(2) {
        return Err(bad());
    }
    (0..text.len())
        .step_by(2)
        .map(|i| {
            text.get(i..i + 2)
                .and_then(|pair| u8::from_str_radix(pair, 16).ok())
                .ok_or_else(bad)
        })
        .collect()
}

#[derive(Clone, Debug, Default)]
struct Lists {
    module: Turns,
    serial: Turns,
    outputs: Turns,
    inputs: Turns,
    analog: Turns,
    supply: Turns,
    set: Turns,
}

#[derive(Clone, Debug)]
struct Config {
    id: u8,
    hardware: u8,
    firmware: u8,
    mac: [u8; 6],
    tenths: u8,
    outputs: u32,
    inputs: u32,
    analog: Vec<u16>,
    lists: Lists,
    answers: Option<u32>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            id: 19,
            hardware: 2,
            firmware: 3,
            mac: [0x00, 0x04, 0xa3, 0x12, 0x34, 0x56],
            tenths: 125,
            outputs: 0,
            inputs: 0,
            analog: Vec::new(),
            lists: Lists::default(),
            answers: None,
            setters: true,
        }
    }
}

/// A relay board from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn byte(key: &str, value: &str) -> Result<u8> {
    u8::try_from(parse_number(key, value, 0..=255)?)
        .map_err(|_| Error::Invalid(format!("{key}={value}: a byte")))
}

/// A number, in hex with `0x`, up to `max`.
fn image(key: &str, value: &str, max: u32) -> Result<u32> {
    match value.strip_prefix("0x") {
        Some(hex) => u32::from_str_radix(hex, 16).ok().filter(|v| *v <= max),
        None => value.parse().ok().filter(|v| *v <= max),
    }
    .ok_or_else(|| Error::Invalid(format!("{key}={value}: 0 to {max:#x}")))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Relays>> {
    let mut faults = Faults::default();
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "board" => config.id = byte(k, v)?,
            "hw" => config.hardware = byte(k, v)?,
            "fw" => config.firmware = byte(k, v)?,
            "volts" => config.tenths = byte(k, v)?,
            "mac" => {
                config.mac = parse_hex(k, v)?
                    .try_into()
                    .map_err(|_| Error::Invalid(format!("mac={v}: 12 hex digits")))?;
            }
            "do" => config.outputs = image(k, v, 0x00ff_ffff)?,
            "di" => config.inputs = image(k, v, u32::MAX)?,
            "ai" => {
                config.analog = v
                    .split(',')
                    .map(|one| {
                        u16::try_from(image(k, one, 0xffff)?)
                            .map_err(|_| Error::Invalid(format!("ai={v}: 16 bits")))
                    })
                    .collect::<Result<Vec<_>>>()?;
            }
            "module" => config.lists.module = Turns::parse(k, v)?,
            "serial" => config.lists.serial = Turns::parse(k, v)?,
            "outputs" => config.lists.outputs = Turns::parse(k, v)?,
            "inputs" => config.lists.inputs = Turns::parse(k, v)?,
            "analog" => config.lists.analog = Turns::parse(k, v)?,
            "supply" => config.lists.supply = Turns::parse(k, v)?,
            "set" => config.lists.set = Turns::parse(k, v)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Devantech relay board");
    Ok(Cdc::new(NAME, &id, faults, Relays::new(config)))
}

struct Relays {
    config: Config,
    board: Option<Board>,
    outputs: u32,
    lists: Lists,
    /// What has come of a command so far.
    pending: Vec<u8>,
    answered: u32,
}

impl Relays {
    fn new(config: Config) -> Relays {
        Relays {
            board: BOARDS.iter().find(|b| b.id == config.id).copied(),
            outputs: config.outputs,
            lists: config.lists.clone(),
            pending: Vec::new(),
            answered: 0,
            config,
        }
    }

    /// How long a command is, which is all the framing there is.
    fn length(&self, command: u8) -> usize {
        match command {
            DIGITAL_ACTIVE | DIGITAL_INACTIVE => 3,
            DIGITAL_SET_OUTPUTS => 1 + self.board.map_or(0, |b| b.output_bytes),
            ANALOG_GET_INPUT => 2,
            _ => 1,
        }
    }

    fn own_inputs(&self, board: &Board) -> Vec<u8> {
        let image = self.config.inputs.to_be_bytes();
        image[image.len() - board.input_bytes..].to_vec()
    }

    fn relay(&mut self, request: &[u8], on: bool, port: &mut Port<'_>) -> Vec<u8> {
        let (number, pulse) = (u32::from(request[1]), request[2]);
        if pulse != 0 {
            port.violation(format!("relay {number}: a pulse of {pulse}"));
        }
        let bit = number.checked_sub(1).filter(|bit| *bit < u32::BITS).map(|bit| 1u32 << bit);
        let mask = self.board.map_or(0, |b| b.mask());
        let Some(bit) = bit.filter(|bit| mask & bit != 0) else {
            port.violation(format!("relay {number}: this board has none"));
            return vec![1];
        };
        if on {
            self.outputs |= bit;
        } else {
            self.outputs &= !bit;
        }
        vec![0]
    }

    fn set_image(&mut self, request: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let image =
            request[1..].iter().rev().fold(0u32, |image, byte| image << 8 | u32::from(*byte));
        let excess = image & !self.board.map_or(0, |b| b.mask());
        if excess != 0 {
            port.violation(format!("set of all: bits {excess:#x} are no relays"));
        }
        self.outputs = image;
        vec![0]
    }

    /// The answer, or none when the board has no such command.
    fn answer(&mut self, request: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let command = request[0];
        if matches!(command, DIGITAL_ACTIVE | DIGITAL_INACTIVE | DIGITAL_SET_OUTPUTS)
            && !self.config.setters
        {
            port.violation(format!("{}: a set no one asked for", show(request)));
        }
        match (command, self.board) {
            (GET_MODULE_INFO, _) => {
                let own = vec![self.config.id, self.config.hardware, self.config.firmware];
                self.lists.module.take(own)
            }
            (GET_SERIAL_NUMBER, _) => {
                let own = self.config.mac.to_vec();
                self.lists.serial.take(own)
            }
            (GET_SUPPLY_VOLTS, Some(_)) => {
                let own = vec![self.config.tenths];
                self.lists.supply.take(own)
            }
            (DIGITAL_ACTIVE, Some(_)) => {
                let own = self.relay(request, true, port);
                self.lists.set.take(own)
            }
            (DIGITAL_INACTIVE, Some(_)) => {
                let own = self.relay(request, false, port);
                self.lists.set.take(own)
            }
            (DIGITAL_SET_OUTPUTS, Some(_)) => {
                let own = self.set_image(request, port);
                self.lists.set.take(own)
            }
            (DIGITAL_GET_OUTPUTS, Some(board)) => {
                let own = self.outputs.to_le_bytes()[..board.output_bytes].to_vec();
                self.lists.outputs.take(own)
            }
            (DIGITAL_GET_INPUTS, Some(board)) if board.inputs > 0 => {
                let own = self.own_inputs(&board);
                self.lists.inputs.take(own)
            }
            (ANALOG_GET_INPUT, Some(board)) if board.analog > 0 => {
                let channel = u32::from(request[1]);
                if channel == 0 || channel > board.analog {
                    port.violation(format!("analog input {channel}: this board has none"));
                }
                let value = channel
                    .checked_sub(1)
                    .and_then(|index| self.config.analog.get(index as usize))
                    .copied()
                    .unwrap_or(0);
                self.lists.analog.take(value.to_be_bytes().to_vec())
            }
            _ => {
                port.violation(format!("{}: no such command for this board", show(request)));
                Vec::new()
            }
        }
    }

    fn command(&mut self, request: &[u8], port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        let answer = self.answer(request, port);
        port.send(&answer);
    }
}

fn show(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut text, b| {
        let _ = write!(text, "{b:02x}");
        text
    })
}

impl Serial for Relays {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            self.pending.push(byte);
            if self.pending.len() >= self.length(self.pending[0]) {
                let request = std::mem::take(&mut self.pending);
                self.command(&request, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("{}: a command left unfinished", show(&self.pending)));
            self.pending.clear();
        }
    }

    fn reset(&mut self) {
        *self = Relays::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;
    use crate::shells::cdc::Line;

    fn relays(options: &[(&str, &str)]) -> Host<Relays> {
        let owned: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let line = Line::parse("9600/8n1").expect("a line");
        Host::open(function(&owned).expect("a board"), line, true, true)
    }

    fn violations(h: &mut Host<Relays>) -> usize {
        h.v.drain().len()
    }

    fn ask(h: &mut Host<Relays>, request: &[u8]) -> Vec<u8> {
        h.write(request);
        h.read()
    }

    #[test]
    fn identity_and_supply() {
        let mut h = relays(&[("board", "20"), ("hw", "4"), ("fw", "9"), ("mac", "0a0b0c0d0e0f")]);
        assert_eq!(ask(&mut h, &[0x10]), [20, 4, 9]);
        assert_eq!(ask(&mut h, &[0x77]), [0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f]);
        assert_eq!(ask(&mut h, &[0x78]), [125]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn every_board_of_the_table_has_its_widths() {
        for (id, outputs, inputs) in [
            ("18", 1, None),
            ("19", 1, None),
            ("20", 2, Some(2)),
            ("21", 3, Some(4)),
            ("22", 2, Some(2)),
            ("24", 3, Some(4)),
            ("26", 1, None),
            ("28", 1, None),
            ("52", 2, Some(2)),
        ] {
            let mut h = relays(&[("board", id)]);
            assert_eq!(ask(&mut h, &[0x24]).len(), outputs, "board {id}");
            if let Some(width) = inputs {
                assert_eq!(ask(&mut h, &[0x25]).len(), width, "board {id}");
                assert_eq!(violations(&mut h), 0);
            } else {
                assert_eq!(ask(&mut h, &[0x25]), b"", "board {id}");
                assert_eq!(violations(&mut h), 1);
            }
        }
    }

    #[test]
    fn outputs_are_low_byte_first_and_inputs_high_byte_first() {
        let mut h = relays(&[("board", "21"), ("do", "0x030201"), ("di", "0x000000a5")]);
        assert_eq!(ask(&mut h, &[0x24]), [1, 2, 3]);
        assert_eq!(ask(&mut h, &[0x25]), [0, 0, 0, 0xa5]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_relay_goes_on_and_off_by_its_number() {
        let mut h = relays(&[]);
        assert_eq!(ask(&mut h, &[0x20, 3, 0]), [0]);
        assert_eq!(ask(&mut h, &[0x24]), [0x04]);
        assert_eq!(ask(&mut h, &[0x20, 8, 0]), [0]);
        assert_eq!(ask(&mut h, &[0x21, 3, 0]), [0]);
        assert_eq!(ask(&mut h, &[0x24]), [0x80]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_image_is_low_byte_first_on_every_width() {
        let mut h = relays(&[("board", "20")]);
        assert_eq!(ask(&mut h, &[0x23, 0x0f, 0xff]), [0]);
        assert_eq!(ask(&mut h, &[0x24]), [0x0f, 0xff]);
        let mut h = relays(&[("board", "21")]);
        assert_eq!(ask(&mut h, &[0x23, 0xff, 0xff, 0x0f]), [0]);
        assert_eq!(ask(&mut h, &[0x24]), [0xff, 0xff, 0x0f]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_relay_that_is_not_there_fails_and_is_flagged() {
        for (board, number) in
            [("19", 0u8), ("19", 9), ("18", 3), ("20", 5), ("20", 8), ("20", 17), ("19", 255)]
        {
            let mut h = relays(&[("board", board)]);
            assert_eq!(ask(&mut h, &[0x20, number, 0]), [1], "{board} {number}");
            assert_eq!(violations(&mut h), 1, "{board} {number}");
            assert_eq!(ask(&mut h, &[0x24])[0], 0);
        }
    }

    #[test]
    fn a_pulse_is_flagged() {
        let mut h = relays(&[]);
        assert_eq!(ask(&mut h, &[0x20, 1, 50]), [0]);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(ask(&mut h, &[0x21, 1, 1]), [0]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn an_image_beyond_the_relays_is_flagged() {
        let mut h = relays(&[("board", "20")]);
        assert_eq!(ask(&mut h, &[0x23, 0xff, 0xff]), [0]);
        assert_eq!(violations(&mut h), 1);
        let mut h = relays(&[("board", "18")]);
        assert_eq!(ask(&mut h, &[0x23, 0x03]), [0]);
        assert_eq!(violations(&mut h), 0);
        assert_eq!(ask(&mut h, &[0x23, 0x04]), [0]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn analog_values_by_input_and_a_missing_input_flagged() {
        let mut h = relays(&[("board", "20"), ("ai", "0x0102,1023")]);
        assert_eq!(ask(&mut h, &[0x32, 1]), [0x01, 0x02]);
        assert_eq!(ask(&mut h, &[0x32, 2]), [0x03, 0xff]);
        assert_eq!(ask(&mut h, &[0x32, 4]), [0, 0]);
        assert_eq!(violations(&mut h), 0);
        for channel in [0u8, 5] {
            ask(&mut h, &[0x32, channel]);
            assert_eq!(violations(&mut h), 1, "{channel}");
        }
        let mut h = relays(&[]);
        assert_eq!(ask(&mut h, &[0x32, 1]), b"");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn what_no_path_sends_gets_no_answer_and_is_flagged() {
        for command in [0x26u8, 0x27, 0x33, 0x34, 0x3a, 0x79, 0x7a, 0x7b, 0x00, 0xff] {
            let mut h = relays(&[("board", "21")]);
            assert_eq!(ask(&mut h, &[command]), b"", "{command:#x}");
            assert_eq!(violations(&mut h), 1, "{command:#x}");
        }
    }

    #[test]
    fn a_board_with_no_table_has_only_what_every_board_has() {
        let mut h = relays(&[("board", "99")]);
        assert_eq!(ask(&mut h, &[0x10]), [99, 2, 3]);
        assert_eq!(ask(&mut h, &[0x77]).len(), 6);
        assert_eq!(violations(&mut h), 0);
        assert_eq!(ask(&mut h, &[0x24]), b"");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_command_cut_in_two_transfers_is_one_command() {
        let mut h = relays(&[]);
        assert_eq!(ask(&mut h, &[0x20]), b"");
        assert_eq!(ask(&mut h, &[1]), b"");
        assert_eq!(ask(&mut h, &[0]), [0]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_command_left_unfinished_at_an_open_is_flagged() {
        let mut h = relays(&[]);
        h.write(&[0x20, 1]);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(ask(&mut h, &[0x10]), [19, 2, 3]);
    }

    #[test]
    fn state_outlives_an_open_and_not_a_reset() {
        let mut h = relays(&[]);
        ask(&mut h, &[0x20, 1, 0]);
        h.set_lines(true, true);
        assert_eq!(ask(&mut h, &[0x24]), [1]);
        h.cdc.model.reset();
        assert_eq!(ask(&mut h, &[0x24]), [0]);
    }

    #[test]
    fn lists_play_in_turn_and_then_again() {
        let mut h = relays(&[("set", "01,~,-,0002")]);
        for want in [&[1u8][..], b"", &[0], &[0, 2], &[1]] {
            assert_eq!(ask(&mut h, &[0x20, 1, 0]), want);
        }
        let mut h = relays(&[("outputs", "-,~")]);
        assert_eq!(ask(&mut h, &[0x24]), [0]);
        assert_eq!(ask(&mut h, &[0x24]), b"");
        assert_eq!(ask(&mut h, &[0x24]), [0]);
    }

    #[test]
    fn a_list_replaces_each_answer() {
        for (key, request) in [
            ("module", &[0x10u8][..]),
            ("serial", &[0x77]),
            ("outputs", &[0x24]),
            ("inputs", &[0x25]),
            ("analog", &[0x32, 1]),
            ("supply", &[0x78]),
            ("set", &[0x21, 1, 0]),
        ] {
            let mut h = relays(&[("board", "20"), (key, "abcd")]);
            assert_eq!(ask(&mut h, request), [0xab, 0xcd], "{key}");
            assert_eq!(violations(&mut h), 0, "{key}");
        }
    }

    #[test]
    fn answers_stop_at_their_count_and_mark() {
        let mut h = relays(&[("answers", "1")]);
        assert_eq!(ask(&mut h, &[0x10]), [19, 2, 3]);
        assert!(!h.cdc.mark());
        assert_eq!(ask(&mut h, &[0x10]), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn setters_0_flags_each_set() {
        for request in [&[0x20u8, 1, 0][..], &[0x21, 1, 0], &[0x23, 0]] {
            let mut h = relays(&[("setters", "0")]);
            h.write(request);
            assert_eq!(violations(&mut h), 1, "{}", show(request));
        }
        let mut h = relays(&[("setters", "0")]);
        h.write(&[0x10, 0x24, 0x77, 0x78]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn bad_options_are_refused() {
        for (key, value) in [
            ("board", "256"),
            ("mac", "0011"),
            ("mac", "00112233445"),
            ("mac", "zz1122334455"),
            ("ai", "65536"),
            ("set", "abc"),
            ("do", "0x1000000"),
            ("nonesuch", "1"),
        ] {
            let owned = vec![(key.to_owned(), value.to_owned())];
            assert!(function(&owned).is_err(), "{key}={value}");
        }
    }
}
