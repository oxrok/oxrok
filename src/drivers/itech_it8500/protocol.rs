// Counterpart in sigrok:
//   libsigrok/src/hardware/itech-it8500/api.c,
//   libsigrok/src/hardware/itech-it8500/protocol.c,
//   libsigrok/src/hardware/itech-it8500/protocol.h,
//   Copyright (C) 2020 Timo Kokkonen <tjko@iki.fi>,
//   with commits by Gerhard Sittig, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The IT8500's wire protocol, the same as a BK Precision 8500's.
//!
//! Every packet is 26 bytes both ways: `aa`, an address, a command, 22 bytes
//! of data, and the sum of the 25 before it. A command answers with a packet
//! of its own number, or a status packet, command `12`, whose first data byte
//! is `80` when all went well. A load takes an address up to `fe`, and `ff` is
//! the broadcast a scan asks on, whose answer names the address to use.
//!
//! Numbers are little endian: mV, 0.1 mA, mW and mOhm.

use crate::core::Error;
use crate::serial::{Params, Port, read_blocking, serial_timeout};
use std::time::Duration;

/// A packet's length, both ways.
pub const PACKET_LEN: usize = 26;
/// The data's length within a packet.
pub const DATA_LEN: usize = 22;
const PREAMBLE: u8 = 0xaa;
/// The address that reaches a load whichever it has.
pub const BROADCAST: u8 = 0xff;
/// How long an answer may take, `itech_it8500_send_cmd`'s 100.
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(100);

/// The commands the driver sends, by number; the manual's names.
#[allow(missing_docs)]
pub mod command {
    pub const GET_LOAD_LIMITS: u8 = 0x01;
    pub const RESPONSE: u8 = 0x12;
    pub const SET_REMOTE_MODE: u8 = 0x20;
    pub const LOAD_ON_OFF: u8 = 0x21;
    pub const SET_MAX_VOLTAGE: u8 = 0x22;
    pub const GET_MAX_VOLTAGE: u8 = 0x23;
    pub const SET_MAX_CURRENT: u8 = 0x24;
    pub const GET_MAX_CURRENT: u8 = 0x25;
    pub const SET_MODE: u8 = 0x28;
    pub const SET_CC_CURRENT: u8 = 0x2a;
    pub const GET_CC_CURRENT: u8 = 0x2b;
    pub const SET_CV_VOLTAGE: u8 = 0x2c;
    pub const GET_CV_VOLTAGE: u8 = 0x2d;
    pub const SET_CW_POWER: u8 = 0x2e;
    pub const GET_CW_POWER: u8 = 0x2f;
    pub const SET_CR_RESISTANCE: u8 = 0x30;
    pub const GET_CR_RESISTANCE: u8 = 0x31;
    pub const GET_STATE: u8 = 0x5f;
    pub const GET_MODEL_INFO: u8 = 0x6a;
    pub const GET_BARCODE_INFO: u8 = 0x6b;
}

const STATUS_OK: u8 = 0x80;
/// The operation state's output bit.
const OS_OUT: u8 = 1 << 3;
/// The demand state's over voltage bit.
pub const DS_OV: u16 = 1 << 1;
/// The demand state's over current bit.
pub const DS_OC: u16 = 1 << 2;
/// The demand state's over temperature bit.
pub const DS_OT: u16 = 1 << 4;
const DS_CC_MODE: u16 = 1 << 6;
const DS_CV_MODE: u16 = 1 << 7;
const DS_CW_MODE: u16 = 1 << 8;
const DS_CR_MODE: u16 = 1 << 9;

/// The sample rates a load can be polled at, of which a slow line takes the
/// first few.
pub const SAMPLERATES: [u64; 10] = [1, 2, 5, 10, 15, 20, 30, 40, 50, 60];
/// The rate a capture polls at until told otherwise.
pub const DEFAULT_SAMPLERATE: u64 = 10;

/// An operating mode, with its number in `SET_MODE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Constant current.
    Cc = 0,
    /// Constant voltage.
    Cv = 1,
    /// Constant power.
    Cw = 2,
    /// Constant resistance.
    Cr = 3,
}

impl Mode {
    /// All the modes, in their order.
    pub const ALL: [Mode; 4] = [Mode::Cc, Mode::Cv, Mode::Cw, Mode::Cr];

    /// What the option calls it.
    pub fn name(self) -> &'static str {
        match self {
            Mode::Cc => "CC",
            Mode::Cv => "CV",
            Mode::Cw => "CW",
            Mode::Cr => "CR",
        }
    }

    /// `itech_it8500_string_to_mode`: the name may have more behind it.
    pub fn from_text(text: &str) -> Option<Mode> {
        Mode::ALL.into_iter().find(|m| text.starts_with(m.name()))
    }

    fn from_demand(demand: u16) -> Mode {
        if demand & DS_CC_MODE != 0 {
            Mode::Cc
        } else if demand & DS_CV_MODE != 0 {
            Mode::Cv
        } else if demand & DS_CW_MODE != 0 {
            Mode::Cw
        } else if demand & DS_CR_MODE != 0 {
            Mode::Cr
        } else {
            Mode::Cc
        }
    }
}

/// The fastest rate of [`SAMPLERATES`] a line can be polled at.
pub fn max_samplerate(baud: u32) -> u64 {
    let safe = (u64::from(baud) * 15 / 9600).min(60);
    let safe = if safe < 15 { 10 } else { safe };
    SAMPLERATES.into_iter().take_while(|r| *r <= safe).last().unwrap_or(SAMPLERATES[0])
}

/// The sum that ends a packet.
pub fn checksum(packet: &[u8; PACKET_LEN]) -> u8 {
    packet[..PACKET_LEN - 1].iter().fold(0, |sum, b| sum.wrapping_add(*b))
}

/// A whole packet from what it says.
pub fn packet(address: u8, command: u8, data: &[u8]) -> [u8; PACKET_LEN] {
    let mut p = [0; PACKET_LEN];
    p[0] = PREAMBLE;
    p[1] = address;
    p[2] = command;
    p[3..3 + data.len()].copy_from_slice(data);
    p[PACKET_LEN - 1] = checksum(&p);
    p
}

/// A number to send, from a value and its unit, as `ivalue = value * unit`.
/// A value that fits no `int` has no defined conversion in C, and what x86
/// makes of it is not a setting: None.
pub fn scaled(value: f64, unit: f64) -> Option<i32> {
    let scaled = value * unit;
    (scaled > f64::from(i32::MIN) - 1.0 && scaled < f64::from(i32::MAX) + 1.0)
        .then_some(scaled as i32)
}

/// Why a command failed.
#[derive(Debug)]
pub enum Failed {
    /// The port itself failed: nothing more will go through it.
    Port(Error),
    /// The answer was late, or wrong, or the write found no room. The next
    /// command may do better.
    Answer,
}

impl From<Failed> for Error {
    fn from(failed: Failed) -> Error {
        match failed {
            Failed::Port(e) => e,
            Failed::Answer => Error::device("generic/unspecified error"),
        }
    }
}

/// An answer's parts that count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reply {
    /// The address it came from.
    pub address: u8,
    /// Its data.
    pub data: [u8; DATA_LEN],
}

/// Checks an answer to `command`: `itech_it8500_send_cmd`. A status packet of
/// success answers a command that returns data, too, by the C: `wants_data`
/// says when that would make readings out of nothing.
///
/// # Errors
///
/// [`Failed::Answer`] on a packet that won't do, said at debug level.
pub fn parse_reply(
    command: u8,
    wants_data: bool,
    p: &[u8; PACKET_LEN],
) -> std::result::Result<Reply, Failed> {
    if p[0] != PREAMBLE {
        log::debug!("Invalid packet received (first byte: {:02x})", p[0]);
        return Err(Failed::Answer);
    }
    if p[PACKET_LEN - 1] != checksum(p) {
        log::debug!("Invalid packet received: checksum mismatch");
        return Err(Failed::Answer);
    }
    let answered = p[2];
    log::trace!("Response packet received: cmd={answered:02x}");
    if answered == command::RESPONSE {
        if p[3] != STATUS_OK {
            log::debug!("Command ({command:02x}) failed: status={:02x}", p[3]);
            return Err(Failed::Answer);
        }
        if wants_data {
            log::debug!("Command ({command:02x}) returned no data");
            return Err(Failed::Answer);
        }
    } else if answered != command {
        log::debug!("Invalid response received: {answered:02x} (expected: {command:02x})");
        return Err(Failed::Answer);
    }
    let mut data = [0; DATA_LEN];
    data.copy_from_slice(&p[3..PACKET_LEN - 1]);
    Ok(Reply { address: p[1], data })
}

/// A command over a port and its answer.
///
/// The input queue goes first: no framing but the count, so a byte left over
/// from a late or a long answer would shift every later one.
///
/// # Errors
///
/// [`Failed::Port`] on a port that fails, [`Failed::Answer`] on the rest.
pub fn exchange(
    port: &mut dyn Port,
    params: &Params,
    address: u8,
    command: u8,
    data: &[u8],
) -> std::result::Result<Reply, Failed> {
    let wants_data = data.is_empty();
    let out = packet(address, command, data);
    port.purge().map_err(Failed::Port)?;
    log::trace!("Sending command: {command:02x}");
    let line = serial_timeout(params, PACKET_LEN as u32);
    let wrote = port.write_timeout(&out, line).map_err(Failed::Port)?;
    if wrote < PACKET_LEN {
        log::debug!("Error sending command 0x{command:02x}: {wrote}");
        return Err(Failed::Answer);
    }
    let mut buf = [0; PACKET_LEN];
    let got = read_blocking(port, &mut buf, REPLY_TIMEOUT).map_err(Failed::Port)?;
    if got < PACKET_LEN {
        log::debug!("Timeout waiting response to command: {got}");
        return Err(Failed::Answer);
    }
    parse_reply(command, wants_data, &buf)
}

/// What a `GET_MODEL_INFO` says.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ModelInfo {
    /// The model, up to five characters.
    pub model: String,
    /// The firmware's two bytes, as `%x.%02x`.
    pub version: String,
    /// The serial number, up to ten characters.
    pub serial: String,
}

/// C string in a field: up to the first NUL.
fn field(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

/// The model name, firmware and serial number of an answer.
pub fn parse_model_info(data: &[u8; DATA_LEN]) -> ModelInfo {
    ModelInfo {
        model: field(&data[..5]),
        version: format!("{:x}.{:02x}", data[6], data[5]),
        serial: field(&data[7..17]),
    }
}

/// The text of a `GET_BARCODE_INFO`'s answer.
pub fn parse_barcode(data: &[u8; DATA_LEN]) -> String {
    field(data)
}

/// What a `GET_LOAD_LIMITS` says, in volts, amperes, watts and ohms.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Limits {
    /// The highest current.
    pub max_current: f64,
    /// The highest voltage.
    pub max_voltage: f64,
    /// The lowest voltage.
    pub min_voltage: f64,
    /// The highest power.
    pub max_power: f64,
    /// The highest resistance.
    pub max_resistance: f64,
    /// The lowest resistance.
    pub min_resistance: f64,
}

fn word(data: &[u8; DATA_LEN], at: usize) -> u32 {
    u32::from_le_bytes([data[at], data[at + 1], data[at + 2], data[at + 3]])
}

/// The limits of an answer.
pub fn parse_limits(data: &[u8; DATA_LEN]) -> Limits {
    Limits {
        max_current: f64::from(word(data, 0)) / 10000.0,
        max_voltage: f64::from(word(data, 4)) / 1000.0,
        min_voltage: f64::from(word(data, 8)) / 1000.0,
        max_power: f64::from(word(data, 12)) / 1000.0,
        max_resistance: f64::from(word(data, 16)) / 1000.0,
        min_resistance: f64::from(u16::from_le_bytes([data[20], data[21]])) / 1000.0,
    }
}

/// A signed number from a read's answer.
pub fn parse_int(data: &[u8; DATA_LEN]) -> i32 {
    word(data, 0) as i32
}

/// What a `GET_STATE` says.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    /// Volts.
    pub voltage: f64,
    /// Amperes.
    pub current: f64,
    /// Watts.
    pub power: f64,
    /// The operation state.
    pub operation: u8,
    /// The demand state.
    pub demand: u16,
}

impl Status {
    /// The status of an answer.
    pub fn parse(data: &[u8; DATA_LEN]) -> Status {
        let status = Status {
            voltage: f64::from(word(data, 0)) / 1000.0,
            current: f64::from(word(data, 4)) / 10000.0,
            power: f64::from(word(data, 8)) / 1000.0,
            operation: data[12],
            demand: u16::from_le_bytes([data[13], data[14]]),
        };
        log::debug!(
            "Load status: V={:.4}, I={:.4}, P={:.3}, State={}, Mode={} (op=0x{:02x}, \
             demand=0x{:04x})",
            status.voltage,
            status.current,
            status.power,
            if status.load_on() { "ON" } else { "OFF" },
            status.mode().name(),
            status.operation,
            status.demand,
        );
        status
    }

    /// Whether the load is on.
    pub fn load_on(&self) -> bool {
        self.operation & OS_OUT != 0
    }

    /// The operating mode. A state with no mode's bit reads as CC.
    pub fn mode(&self) -> Mode {
        Mode::from_demand(self.demand)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn answer(command: u8, data: &[u8]) -> [u8; PACKET_LEN] {
        packet(0, command, data)
    }

    #[test]
    fn a_packet_has_its_sum() {
        // The manual's example: remote mode on.
        let p = packet(0, 0x20, &[1]);
        assert_eq!(p[..4], [0xaa, 0, 0x20, 1]);
        assert_eq!(p[25], 0xcb);
        assert_eq!(packet(0xff, 0x6a, &[])[25], 0x13);
    }

    #[test]
    fn a_reply_needs_its_preamble_and_sum() {
        let good = answer(0x2b, &[1, 2]);
        assert_eq!(parse_reply(0x2b, true, &good).unwrap().data[..2], [1, 2]);
        let mut bad = good;
        bad[0] = 0x55;
        bad[25] = checksum(&bad);
        assert!(matches!(parse_reply(0x2b, true, &bad), Err(Failed::Answer)));
        let mut bad = good;
        bad[25] ^= 1;
        assert!(matches!(parse_reply(0x2b, true, &bad), Err(Failed::Answer)));
    }

    #[test]
    fn a_reply_must_be_of_its_command_or_a_status() {
        let other = answer(0x2d, &[]);
        assert!(matches!(parse_reply(0x2b, true, &other), Err(Failed::Answer)));
        let ok = answer(command::RESPONSE, &[0x80]);
        assert!(parse_reply(0x21, false, &ok).is_ok());
        let refused = answer(command::RESPONSE, &[0xa0]);
        assert!(matches!(parse_reply(0x21, false, &refused), Err(Failed::Answer)));
    }

    #[test]
    fn a_status_of_success_is_no_data() {
        let ok = answer(command::RESPONSE, &[0x80]);
        assert!(matches!(parse_reply(0x5f, true, &ok), Err(Failed::Answer)));
    }

    #[test]
    fn a_setting_may_be_echoed() {
        assert!(parse_reply(0x21, false, &answer(0x21, &[])).is_ok());
    }

    #[test]
    fn a_reply_says_its_address() {
        let p = packet(9, 0x6a, &[]);
        assert_eq!(parse_reply(0x6a, true, &p).unwrap().address, 9);
    }

    #[test]
    fn modes_take_more_behind_them() {
        assert_eq!(Mode::from_text("CC"), Some(Mode::Cc));
        assert_eq!(Mode::from_text("CVX"), Some(Mode::Cv));
        assert_eq!(Mode::from_text("CW"), Some(Mode::Cw));
        assert_eq!(Mode::from_text("CR"), Some(Mode::Cr));
        assert_eq!(Mode::from_text("cc"), None);
        assert_eq!(Mode::from_text("C"), None);
    }

    #[test]
    fn the_rate_of_a_line() {
        for (baud, rate) in [
            (2400, 10),
            (4800, 10),
            (9599, 10),
            (9600, 15),
            (14400, 20),
            (19200, 30),
            (28800, 40),
            (38400, 60),
            (115_200, 60),
        ] {
            assert_eq!(max_samplerate(baud), rate, "{baud}");
        }
    }

    #[test]
    fn a_status_reads_its_mode_first_flag_first() {
        let status = |demand: u16| Status { demand, ..Status::default() }.mode();
        assert_eq!(status(0), Mode::Cc);
        assert_eq!(status(1 << 6 | 1 << 7), Mode::Cc);
        assert_eq!(status(1 << 7 | 1 << 8), Mode::Cv);
        assert_eq!(status(1 << 8 | 1 << 9), Mode::Cw);
        assert_eq!(status(1 << 9), Mode::Cr);
    }

    #[test]
    fn a_status_from_its_bytes() {
        let mut data = [0; DATA_LEN];
        data[..4].copy_from_slice(&12345u32.to_le_bytes());
        data[4..8].copy_from_slice(&12345u32.to_le_bytes());
        data[8..12].copy_from_slice(&15240u32.to_le_bytes());
        data[12] = 0x08;
        data[13..15].copy_from_slice(&0x0086u16.to_le_bytes());
        let s = Status::parse(&data);
        assert_eq!(s.voltage, 12.345);
        assert_eq!(s.current, 1.2345);
        assert_eq!(s.power, 15.24);
        assert!(s.load_on());
        assert_eq!(s.demand & DS_OV, DS_OV);
        assert_eq!(s.demand & DS_OC, DS_OC);
        assert_eq!(s.demand & DS_OT, 0);
        assert_eq!(s.mode(), Mode::Cv);
    }

    #[test]
    fn the_names_stop_at_their_field_or_a_nul() {
        let mut data = [0; DATA_LEN];
        data[..5].copy_from_slice(b"85112");
        data[5] = 0x5a;
        data[6] = 0x02;
        data[7..17].copy_from_slice(b"ABCDEFGHIJ");
        data[17] = b'!';
        let info = parse_model_info(&data);
        assert_eq!(info.model, "85112");
        assert_eq!(info.version, "2.5a");
        assert_eq!(info.serial, "ABCDEFGHIJ");
        data[2] = 0;
        assert_eq!(parse_model_info(&data).model, "85");
        assert_eq!(parse_barcode(&[b'x'; DATA_LEN]).len(), DATA_LEN);
    }

    #[test]
    fn the_limits_are_in_their_units() {
        let mut data = [0; DATA_LEN];
        for (i, n) in [300_000u32, 120_000, 100, 300_000, 7_500_000].into_iter().enumerate() {
            data[i * 4..i * 4 + 4].copy_from_slice(&n.to_le_bytes());
        }
        data[20..22].copy_from_slice(&50u16.to_le_bytes());
        let l = parse_limits(&data);
        assert_eq!(l.max_current, 30.0);
        assert_eq!(l.max_voltage, 120.0);
        assert_eq!(l.min_voltage, 0.1);
        assert_eq!(l.max_power, 300.0);
        assert_eq!(l.max_resistance, 7500.0);
        assert_eq!(l.min_resistance, 0.05);
    }

    #[test]
    fn a_read_is_signed() {
        let mut data = [0; DATA_LEN];
        data[..4].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(parse_int(&data), -1);
    }

    #[test]
    fn scaled_truncates_as_c_and_refuses_what_c_leaves_undefined() {
        assert_eq!(scaled(5.0, 1000.0), Some(5000));
        assert_eq!(scaled(f64::from(3.3f32), 1000.0), Some(3299));
        assert_eq!(scaled(-1.0, 1000.0), Some(-1000));
        assert_eq!(scaled(-0.0, 1000.0), Some(0));
        assert_eq!(scaled(2_147_483.647, 1000.0), Some(2_147_483_647));
        assert_eq!(scaled(1e10, 1000.0), None);
        assert_eq!(scaled(-1e10, 1000.0), None);
        assert_eq!(scaled(f64::NAN, 1000.0), None);
        assert_eq!(scaled(f64::INFINITY, 1000.0), None);
    }
}
