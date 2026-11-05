// Counterpart in sigrok:
//   libsigrok/src/hardware/maynuo-m97/protocol.c,
//   libsigrok/src/hardware/maynuo-m97/protocol.h,
//   Copyright (C) 2015 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Uwe Hermann, Daniel Elstner, François Revol,
//   Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Maynuo M97 and M98's registers and coils, over Modbus RTU.
//!
//! Coils hold the panel's lock (`PC1`), the input's state and what tripped, and
//! a read of one gives a byte of which only the lowest bit is that coil: the
//! user's manual's own example reads `ISTATE` as off from `0x48`.
//!
//! Registers hold settings and readings as big endian floats in two registers,
//! high word first (`0x41 0x20 0x00 0x2A` is 10 V in the manual). `CMD` takes a
//! mode or the input's on and off, and `SETMODE` reads the mode. In which byte
//! of `SETMODE` is open: this has the low one as C does, where `ATZ9712`'s
//! `main.cpp`, written against an Atten ATZ9712, has the high one. If that is
//! what a load does, its regulation reads "UNKNOWN" and a change of the input
//! writes `CMD` 0, as in C (`docs/DEVICES.md`).

use crate::core::{Error, Result};
use crate::modbus::Modbus;
use std::sync::atomic::{AtomicBool, Ordering};

/// `PC1`: on, the front panel takes no keys.
pub const PC1: u16 = 0x0500;
/// The input's state.
pub const ISTATE: u16 = 0x0510;
/// Over current.
pub const IOVER: u16 = 0x0520;
/// Over voltage.
pub const UOVER: u16 = 0x0521;
/// Over temperature.
pub const HEAT: u16 = 0x0523;
/// Unregulated.
pub const UNREG: u16 = 0x0525;

/// The command register.
pub const CMD: u16 = 0x0a00;
/// The current setting, a float.
pub const IFIX: u16 = 0x0a01;
/// The voltage setting, a float.
pub const UFIX: u16 = 0x0a03;
/// The current's limit, a float.
pub const IMAX: u16 = 0x0a34;
/// The voltage's limit, a float.
pub const UMAX: u16 = 0x0a36;
/// The voltage read, a float.
pub const U: u16 = 0x0b00;
/// The current read, a float.
pub const I: u16 = 0x0b02;
/// The mode.
pub const SETMODE: u16 = 0x0b04;
/// The model, and `EDITION` after it.
pub const MODEL: u16 = 0x0b06;

/// `CMD`: the input on.
pub const INPUT_ON: u16 = 42;
/// `CMD`: the input off.
pub const INPUT_OFF: u16 = 43;

/// A line of `supported_models[]`. C's comment says its ids are guessed, and
/// the rest of its table is commented out, so that a load of those is refused.
#[derive(Debug, PartialEq, Eq)]
pub struct Model {
    /// The `MODEL` register.
    pub id: u16,
    /// Its name.
    pub name: &'static str,
    /// Amperes, for the list.
    pub max_current: u32,
    /// Volts, for the list.
    pub max_voltage: u32,
}

/// The models a scan finds.
pub static MODELS: [Model; 2] = [
    Model { id: 28, name: "M9710", max_current: 30, max_voltage: 150 },
    Model { id: 101, name: "M9812", max_current: 30, max_voltage: 150 },
];

/// The line of `id`.
pub fn lookup(id: u16) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.id == id)
}

/// How a version shows.
pub fn version_text(version: u16) -> String {
    format!("v{}.{}", version / 10, version % 10)
}

/// `maynuo_m97_mode_to_str`, of the low byte of `SETMODE`.
pub fn mode_name(setmode: u16) -> &'static str {
    match setmode & 0xff {
        1 => "CC",
        2 => "CV",
        3 => "CP",
        4 => "CR",
        20 => "CC Soft Start",
        25 => "Dynamic",
        26 => "Short Circuit",
        27 => "List Mode",
        30 => "CC Loading and Unloading",
        31 => "CV Loading and Unloading",
        32 => "CP Loading and Unloading",
        33 => "CR Loading and Unloading",
        34 => "CC + CV",
        36 => "CR + CV",
        38 => "Battery Test",
        39 => "CV Soft Start",
        _ => "UNKNOWN",
    }
}

/// A float from two registers, high word first.
pub fn float_from(registers: [u16; 2]) -> f32 {
    f32::from_bits(u32::from(registers[0]) << 16 | u32::from(registers[1]))
}

/// The two registers of a float, high word first.
pub fn float_to(value: f32) -> [u16; 2] {
    let bits = value.to_bits();
    [(bits >> 16) as u16, (bits & 0xffff) as u16]
}

/// The two readings of a poll.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// Volts.
    pub voltage: f32,
    /// Amperes.
    pub current: f32,
}

/// A load's model and, while it is open, its bus.
pub struct Link {
    /// The bus, while open.
    pub modbus: Option<Modbus>,
    /// Its model.
    pub model: &'static Model,
}

impl Link {
    /// A link of `model`.
    pub fn new(modbus: Option<Modbus>, model: &'static Model) -> Link {
        Link { modbus, model }
    }

    fn bus(&mut self) -> Result<&mut Modbus> {
        self.modbus.as_mut().ok_or_else(|| Error::device("the device is not open"))
    }

    /// A coil's state, the lowest bit of what comes: `maynuo_m97_get_bit`.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn get_bit(&mut self, address: u16) -> Result<bool> {
        let mut coil = [0];
        self.bus()?.read_coils(address, 1, &mut coil)?;
        Ok(coil[0] & 1 == 1)
    }

    /// `maynuo_m97_set_bit`.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn set_bit(&mut self, address: u16, value: bool) -> Result<()> {
        self.bus()?.write_coil(address, value)
    }

    /// `maynuo_m97_get_float`.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn get_float(&mut self, address: u16) -> Result<f32> {
        let mut registers = [0; 2];
        self.bus()?.read_holding_registers(address, &mut registers)?;
        Ok(float_from(registers))
    }

    /// `maynuo_m97_set_float`.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn set_float(&mut self, address: u16, value: f32) -> Result<()> {
        self.bus()?.write_multiple_registers(address, &float_to(value))
    }

    /// `SETMODE` as it reads: `maynuo_m97_get_mode` before its mask.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn get_mode(&mut self) -> Result<u16> {
        let mut registers = [0];
        self.bus()?.read_holding_registers(SETMODE, &mut registers)?;
        Ok(registers[0])
    }

    fn command(&mut self, command: u16) -> Result<()> {
        self.bus()?.write_multiple_registers(CMD, &[command])
    }

    /// The input on or off, then the mode it was in again:
    /// `maynuo_m97_set_input`. An error ends it where it came.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn set_input(&mut self, enable: bool) -> Result<()> {
        let mode = self.get_mode()? & 0xff;
        self.command(if enable { INPUT_ON } else { INPUT_OFF })?;
        self.command(mode)
    }

    /// `U` and `I`, in one read: what `maynuo_m97_receive_data` reads the reply
    /// of. Unless a stop has come, which then saves the request.
    ///
    /// # Errors
    ///
    /// What the bus gives.
    pub fn poll(&mut self, stop: &AtomicBool) -> Result<Reading> {
        let mut registers = [0; 4];
        if stop.load(Ordering::SeqCst) {
            return Err(Error::device("stopped"));
        }
        self.bus()?.read_holding_registers(U, &mut registers)?;
        Ok(Reading {
            voltage: float_from([registers[0], registers[1]]),
            current: float_from([registers[2], registers[3]]),
        })
    }

    /// Whether the last request could not go out.
    pub fn request_unsent(&self) -> bool {
        self.modbus.as_ref().is_some_and(Modbus::request_unsent)
    }
}

/// The model and the version: `maynuo_m97_get_model_version`.
///
/// # Errors
///
/// What the bus gives.
pub fn read_identity(bus: &mut Modbus) -> Result<(u16, u16)> {
    let mut registers = [0; 2];
    bus.read_holding_registers(MODEL, &mut registers)?;
    Ok((registers[0], registers[1]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_models_of_the_table() {
        assert_eq!(lookup(28).unwrap().name, "M9710");
        assert_eq!(lookup(101).unwrap().name, "M9812");
        assert_eq!(lookup(53), None, "commented out in C");
        assert_eq!(lookup(101).unwrap().max_voltage, 150);
        assert_eq!(lookup(28).unwrap().max_current, 30);
    }

    #[test]
    fn a_version_has_its_last_digit_apart() {
        assert_eq!(version_text(123), "v12.3");
        assert_eq!(version_text(5), "v0.5");
        assert_eq!(version_text(65535), "v6553.5");
    }

    #[test]
    fn every_mode_has_its_name() {
        for (mode, name) in [
            (1, "CC"),
            (2, "CV"),
            (3, "CP"),
            (4, "CR"),
            (20, "CC Soft Start"),
            (25, "Dynamic"),
            (26, "Short Circuit"),
            (27, "List Mode"),
            (30, "CC Loading and Unloading"),
            (31, "CV Loading and Unloading"),
            (32, "CP Loading and Unloading"),
            (33, "CR Loading and Unloading"),
            (34, "CC + CV"),
            (36, "CR + CV"),
            (38, "Battery Test"),
            (39, "CV Soft Start"),
            (41, "UNKNOWN"),
            (42, "UNKNOWN"),
            (0, "UNKNOWN"),
        ] {
            assert_eq!(mode_name(mode), name, "{mode}");
        }
        assert_eq!(mode_name(0x0201), "CC", "the low byte");
        assert_eq!(mode_name(0x0100), "UNKNOWN", "the high byte counts for nothing");
    }

    #[test]
    fn a_float_is_big_endian_high_word_first() {
        // The manual's example: "the read-back voltage value ... is 10V".
        assert_eq!(float_from([0x4120, 0x002a]), f32::from_bits(0x4120_002a));
        assert_eq!(float_from([0x4120, 0x0000]), 10.0);
        assert_eq!(float_to(10.0), [0x4120, 0x0000]);
        assert_eq!(float_to(3.3), [0x4053, 0x3333]);
        assert_eq!(float_from(float_to(1.2345)), 1.2345);
    }
}
