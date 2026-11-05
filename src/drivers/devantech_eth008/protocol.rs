// Counterpart in sigrok:
//   libsigrok/src/hardware/devantech-eth008/api.c,
//   libsigrok/src/hardware/devantech-eth008/protocol.c,
//   libsigrok/src/hardware/devantech-eth008/protocol.h,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The boards' binary command set: a command byte and a fixed number of
//! arguments, an answer of a fixed length, and nothing that frames either or
//! says which request an answer belongs to.
//!
//! | Command | Sent | Answer |
//! |---|---|---|
//! | `0x10` module info | | id, hardware, firmware |
//! | `0x20`, `0x21` relay on, off | number, pulse (0, none) | 0 for success |
//! | `0x23` set all relays | image, low byte first | 0 for success |
//! | `0x24` get relays | | image, low byte first |
//! | `0x25` get inputs | | image, high byte first |
//! | `0x32` get analog | number | 16 bits, high byte first, no unit |
//! | `0x77` serial number | | the 6 bytes of the MAC |
//! | `0x78` supply | | a byte, in 0.1 V |
//!
//! A board doesn't answer a command it hasn't got, which is why the inputs are
//! only asked of a board that has them.
//!
//! With no framing a late answer would pass for the answer to the next request.
//! C has that (`recv_response` fails, and nothing clears what comes next). Here
//! each request starts with an empty input queue, so a byte too many goes, and
//! after a request went unanswered the next one first waits for the line to go
//! quiet, with [`Settle`], as a purge drops only what has come. What comes later
//! than that is still taken for the next answer: nothing in the bytes can tell.

use crate::core::{Error, Result};
use crate::serial::{Port, read_blocking};
use std::time::{Duration, Instant};

const CMD_GET_MODULE_INFO: u8 = 0x10;
const CMD_DIGITAL_ACTIVE: u8 = 0x20;
const CMD_DIGITAL_INACTIVE: u8 = 0x21;
const CMD_DIGITAL_SET_OUTPUTS: u8 = 0x23;
const CMD_DIGITAL_GET_OUTPUTS: u8 = 0x24;
const CMD_DIGITAL_GET_INPUTS: u8 = 0x25;
const CMD_ANALOG_GET_INPUT: u8 = 0x32;
const CMD_GET_SERIAL_NUMBER: u8 = 0x77;
const CMD_GET_SUPPLY_VOLTS: u8 = 0x78;

/// One line of `models[]`. `min_serno_fw` is 0 on all of them, so there's no
/// firmware that isn't asked for its serial number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Model {
    /// The module id.
    pub code: u8,
    /// Its name.
    pub name: &'static str,
    /// Digital outputs, the relays among them.
    pub outputs: usize,
    /// Digital inputs.
    pub inputs: usize,
    /// Analog inputs.
    pub analog: usize,
    /// Bytes of the outputs' image.
    pub output_bytes: usize,
    /// Bytes of the inputs' image.
    pub input_bytes: usize,
    /// Outputs the image has room for and the board doesn't have.
    pub missing: u32,
}

const fn model(
    code: u8,
    name: &'static str,
    (outputs, inputs, analog): (usize, usize, usize),
    (output_bytes, input_bytes): (usize, usize),
    missing: u32,
) -> Model {
    Model { code, name, outputs, inputs, analog, output_bytes, input_bytes, missing }
}

/// `models[]`, sorted by id.
pub const MODELS: [Model; 9] = [
    model(18, "ETH002", (2, 0, 0), (1, 0), 0),
    model(19, "ETH008", (8, 0, 0), (1, 0), 0),
    model(20, "ETH484", (16, 8, 4), (2, 2), 0x00f0),
    model(21, "ETH8020", (20, 8, 8), (3, 4), 0),
    model(22, "WIFI484", (16, 8, 4), (2, 2), 0x00f0),
    model(24, "WIFI8020", (20, 8, 8), (3, 4), 0),
    model(26, "WIFI002", (2, 0, 0), (1, 0), 0),
    model(28, "WIFI008", (8, 0, 0), (1, 0), 0),
    model(52, "ETH1610", (10, 16, 16), (2, 2), 0),
];

impl Model {
    /// `find_model`.
    pub fn find(code: u8) -> Option<&'static Model> {
        MODELS.iter().find(|m| m.code == code)
    }

    /// `mask_do`: the outputs that exist, as bits.
    pub fn output_mask(&self) -> u32 {
        low_bits(self.outputs) & !self.missing
    }
}

/// The lowest `count` bits; a count is 32 at most in any table.
fn low_bits(count: usize) -> u32 {
    u32::try_from((1u64 << count) - 1).unwrap_or(u32::MAX)
}

/// `SR_ERR_DATA`: what C's every wrong answer comes to.
fn invalid_data() -> Error {
    Error::Io(std::io::Error::new(std::io::ErrorKind::InvalidData, "data is invalid"))
}

/// The times of a request. C's `READ_TIMEOUT_MS` is 20, and over TCP it has no
/// effect, since its read is a blocking `recv` (`tcp.c:362`): a board at the
/// other end of a slow link works, and a silent one hangs. So [`Timing::reply`]
/// is long enough for a slow link and for a loaded host, and does end.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// How long a whole answer may take.
    pub reply: Duration,
    /// How long the line must stay silent after a request went unanswered.
    pub quiet: Duration,
    /// The most a line that never falls silent can hold the next request.
    pub settle_max: Duration,
}

/// What a device uses.
pub const TIMING: Timing = Timing {
    reply: Duration::from_secs(1),
    quiet: Duration::from_millis(300),
    settle_max: Duration::from_secs(1),
};

/// How often [`Settle`]'s wait looks at the line.
const SETTLE_POLL: Duration = Duration::from_millis(5);

/// Waits out what is left of an unanswered request: over when nothing has come
/// for `quiet`, or at `settle_max`.
#[derive(Debug)]
pub struct Settle {
    timing: Timing,
    last: Instant,
    began: Instant,
}

impl Settle {
    /// A wait that begins now.
    pub fn new(timing: Timing) -> Settle {
        let now = Instant::now();
        Settle { timing, last: now, began: now }
    }

    /// Something came.
    pub fn heard(&mut self) {
        self.last = Instant::now();
    }

    /// There's no more waiting.
    pub fn over(&self) -> bool {
        self.last.elapsed() >= self.timing.quiet || self.began.elapsed() >= self.timing.settle_max
    }
}

/// The port and how it stands.
pub struct Link {
    /// The port, while open.
    pub port: Option<Box<dyn Port>>,
    timing: Timing,
    /// A request went unanswered, and its answer may still be on the way.
    out_of_step: bool,
}

impl Link {
    /// A link with the given times.
    pub fn new(port: Option<Box<dyn Port>>, timing: Timing) -> Link {
        Link { port, timing, out_of_step: false }
    }

    /// A port, freshly opened: nothing can be on its way on it that C or this
    /// would mistake. An open flushes, and TCP has a new connection.
    pub fn attach(&mut self, port: Box<dyn Port>) {
        self.port = Some(port);
        self.out_of_step = false;
    }

    /// Drops what's left of an unanswered request.
    fn settle(&mut self, port: &mut dyn Port) -> Result<()> {
        let mut settle = Settle::new(self.timing);
        let mut buf = [0u8; 16];
        while !settle.over() {
            if port.read(&mut buf, SETTLE_POLL)? > 0 {
                settle.heard();
            }
        }
        Ok(())
    }

    /// `send_then_recv`, from a clean line. Any failure to get the whole answer
    /// leaves the link out of step.
    fn exchange(&mut self, request: &[u8], reply: &mut [u8]) -> Result<()> {
        let mut port = self.port.take().ok_or_else(|| Error::arg("The device is not open."))?;
        let result = self.exchange_on(port.as_mut(), request, reply);
        self.port = Some(port);
        result
    }

    fn exchange_on(&mut self, port: &mut dyn Port, request: &[u8], reply: &mut [u8]) -> Result<()> {
        if self.out_of_step {
            self.settle(port)?;
        }
        port.purge()?;
        log::trace!("TX --> {}.", hex(request));
        port.write(request)?;
        self.out_of_step = true;
        let got = read_blocking(port, reply, self.timing.reply)?;
        log::trace!("<-- RX {}.", hex(&reply[..got]));
        if got != reply.len() {
            return Err(invalid_data());
        }
        self.out_of_step = false;
        Ok(())
    }

    /// `devantech_eth008_get_model`: the id, hardware and firmware versions.
    ///
    /// # Errors
    ///
    /// The port's, or no whole answer.
    pub fn get_model(&mut self) -> Result<(u8, u8, u8)> {
        let mut reply = [0u8; 3];
        self.exchange(&[CMD_GET_MODULE_INFO], &mut reply)?;
        Ok((reply[0], reply[1], reply[2]))
    }

    /// `devantech_eth008_get_serno`: the MAC, as lower case hex digits.
    ///
    /// # Errors
    ///
    /// The port's, or no whole answer.
    pub fn get_serial(&mut self) -> Result<String> {
        let mut reply = [0u8; 6];
        self.exchange(&[CMD_GET_SERIAL_NUMBER], &mut reply)?;
        Ok(hex(&reply).replace(' ', ""))
    }

    /// `devantech_eth008_cache_state`: the images of what `model` has, as the
    /// outputs and the inputs. The two are always read together, and either
    /// failing fails the lot.
    ///
    /// # Errors
    ///
    /// The port's, or no whole answer.
    pub fn read_state(&mut self, model: &Model) -> Result<(u32, u32)> {
        let mut outputs = 0;
        if model.outputs > 0 {
            let mut reply = [0u8; 3];
            let reply = &mut reply[..model.output_bytes];
            self.exchange(&[CMD_DIGITAL_GET_OUTPUTS], reply)?;
            outputs = reply.iter().rev().fold(0, |image, b| image << 8 | u32::from(*b));
            outputs &= model.output_mask();
        }
        let mut inputs = 0;
        if model.inputs > 0 {
            let mut reply = [0u8; 4];
            let reply = &mut reply[..model.input_bytes];
            self.exchange(&[CMD_DIGITAL_GET_INPUTS], reply)?;
            inputs = reply.iter().fold(0, |image, b| image << 8 | u32::from(*b));
            inputs &= low_bits(model.inputs);
        }
        Ok((outputs, inputs))
    }

    /// `devantech_eth008_setup_do`, for one relay by its number. An answer that
    /// isn't 0 is a failure, though the documents only name 1.
    ///
    /// # Errors
    ///
    /// The port's, no answer, or a failure.
    pub fn set_output(&mut self, number: u8, on: bool) -> Result<()> {
        let command = if on { CMD_DIGITAL_ACTIVE } else { CMD_DIGITAL_INACTIVE };
        self.set(&[command, number, 0])
    }

    /// `devantech_eth008_setup_do`, for all of them.
    ///
    /// # Errors
    ///
    /// The port's, no answer, or a failure.
    pub fn set_all_outputs(&mut self, model: &Model, on: bool) -> Result<()> {
        let image = if on { model.output_mask() } else { 0 }.to_le_bytes();
        let mut request = [0u8; 4];
        request[0] = CMD_DIGITAL_SET_OUTPUTS;
        request[1..=model.output_bytes].copy_from_slice(&image[..model.output_bytes]);
        self.set(&request[..=model.output_bytes])
    }

    fn set(&mut self, request: &[u8]) -> Result<()> {
        let mut reply = [0u8; 1];
        self.exchange(request, &mut reply)?;
        if reply[0] != 0 {
            return Err(invalid_data());
        }
        Ok(())
    }

    /// `devantech_eth008_query_ai`: a value with no unit, as models differ.
    ///
    /// # Errors
    ///
    /// The port's, or no whole answer.
    pub fn get_analog(&mut self, number: u8) -> Result<u16> {
        let mut reply = [0u8; 2];
        self.exchange(&[CMD_ANALOG_GET_INPUT, number], &mut reply)?;
        Ok(u16::from_be_bytes(reply))
    }

    /// `devantech_eth008_query_supply`, in millivolts.
    ///
    /// # Errors
    ///
    /// The port's, or no whole answer.
    pub fn get_supply_millivolts(&mut self) -> Result<u16> {
        let mut reply = [0u8; 1];
        self.exchange(&[CMD_GET_SUPPLY_VOLTS], &mut reply)?;
        Ok(u16::from(reply[0]) * 100)
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}
