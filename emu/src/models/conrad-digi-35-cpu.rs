// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Conrad DIGI 35 CPU on the CDC-ACM shell: a laboratory power supply of 1992
//! whose RS-232 port only listens. Nothing ever comes back.
//!
//! The device, and how sure this is. The sources are sigrok's wiki page (its
//! Hardware and Protocol sections, read as text) and the driver. Conrad's manual
//! (`512982-an-02-de`) could not be fetched, as its host no longer resolves, and no
//! firmware source was found for its 68HC05C4. No recording was examined.
//!
//! - The line: 9600/8n1 at power on. The wiki also lists 4800, 2400 and 300 baud,
//!   chosen by a command. The port receives only and uses no handshake lines.
//! - `Vxxx` CR sets the voltage in 100 mV and `Cxxx` CR the current in 10 mA.
//! - Special functions go through `V`: `900` is over-current protection (reaching
//!   the limit disables the output), and `901` constant current, the power on
//!   default (reaching the limit lowers the voltage).
//! - At power on the voltage and the current are 0.
//!
//! That the device swallows a command coming under 50 ms after another rests on the
//! driver's comment alone. This model can't check it, as it has no clock; the port's
//! unit tests do, on a fake port.
//!
//! The other commands are real by the wiki, and no path of the driver sends them:
//! `L`, `E`, `U`, `D` (keyboard lock and unlock, a step up and down), `V4xx` to
//! `V7xx` (timers and an unknown one), `V800` to `V803` (the baud rate) and `V902`
//! (keyboard lock). They aren't modelled. [`Supply`] flags them, because a port that
//! sent one would change the state of a unit, and it ignores them.
//!
//! A habit of C's, accepted and not flagged: `V-21` CR and `C-21` CR. On x86-64 a
//! NaN passes C's range check and its conversion gives `INT_MIN`, whose first three
//! characters fill the field. The device can't refuse anything, having no way to
//! answer, and what it does with this is not known: here it changes nothing.
//!
//! Emulation artifacts:
//!
//! - The USB side is a CDC-ACM function with the shell's descriptors and test ids.
//!   The real device has a DB-9 socket, so a tool reaches it through some adapter.
//! - A command takes effect when its CR comes. No time passes in the model.
//! - `uart=` stands for a unit that an earlier program left at another rate.
//!
//! Options, besides the shell's ([`Faults`]): `uart=LINE`, by default 9600/8n1.
//!
//! The pedantic checks, for what no path of the C driver sends: a frame that isn't
//! one of the above, a value past the ranges, and a frame longer than any command.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "conrad-digi-35-cpu";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Conrad DIGI 35 CPU power supply on CDC-ACM, which takes commands and \
                         never answers; uart=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// What ends a command.
const CR: u8 = b'\r';
/// The longest command, without its CR.
const LONGEST_COMMAND: usize = 4;
/// The top of the voltage, in 100 mV.
const MAX_VOLTAGE: u16 = 350;
/// The top of the current, in 10 mA.
const MAX_CURRENT: u16 = 255;

/// A supply from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // Test ids: see the module's note on the USB side.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Conrad DIGI 35 CPU stand-in");
    Ok(Cdc::new(NAME, &id, faults, Supply::default()).behind_uart(uart))
}

/// One command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    Voltage(u16),
    Current(u16),
    /// `V900` for true, `V901` for false.
    Protection(bool),
    /// See the module's note on `V-21` and `C-21`.
    NanHabit,
}

/// What a frame without its CR means, or why it is flagged.
fn command(frame: &[u8]) -> std::result::Result<Command, String> {
    if frame == b"V-21" || frame == b"C-21" {
        return Ok(Command::NanHabit);
    }
    let shown = String::from_utf8_lossy(frame);
    let (&letter, digits) = frame.split_first().ok_or_else(|| "an empty command".to_owned())?;
    if digits.len() != 3 || !digits.iter().all(u8::is_ascii_digit) {
        return Err(format!("{shown:?}: no path of the driver sends this form"));
    }
    let value = digits.iter().fold(0u16, |n, d| n * 10 + u16::from(d - b'0'));
    match (letter, value) {
        (b'V', 0..=MAX_VOLTAGE) => Ok(Command::Voltage(value)),
        (b'C', 0..=MAX_CURRENT) => Ok(Command::Current(value)),
        (b'V', 900) => Ok(Command::Protection(true)),
        (b'V', 901) => Ok(Command::Protection(false)),
        _ => Err(format!("{shown:?}: no path of the driver sends this value")),
    }
}

/// The supply. The fields are what its front panel would show; no host can read
/// them, so only the unit tests look.
#[derive(Default)]
struct Supply {
    /// In 100 mV.
    voltage: u16,
    /// In 10 mA.
    current: u16,
    protection: bool,
    /// The command so far.
    frame: Vec<u8>,
    /// The frame grew past any command, and was flagged: skip to its CR.
    overlong: bool,
}

impl Supply {
    fn byte(&mut self, byte: u8, port: &mut Port<'_>) {
        if byte == CR {
            if !self.overlong {
                match command(&self.frame) {
                    Ok(c) => self.apply(c),
                    Err(e) => port.violation(e),
                }
            }
            self.frame.clear();
            self.overlong = false;
        } else if self.frame.len() < LONGEST_COMMAND {
            self.frame.push(byte);
        } else if !self.overlong {
            self.overlong = true;
            port.violation(format!(
                "{:?}: longer than any command",
                String::from_utf8_lossy(&self.frame)
            ));
        }
    }

    fn apply(&mut self, command: Command) {
        match command {
            Command::Voltage(v) => self.voltage = v,
            Command::Current(c) => self.current = c,
            Command::Protection(on) => self.protection = on,
            Command::NanHabit => {}
        }
    }
}

impl Serial for Supply {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for b in data {
            self.byte(*b, port);
        }
    }

    fn reset(&mut self) {
        *self = Supply::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn opened(options: &[(&str, &str)]) -> Host<Supply> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn violations(h: &mut Host<Supply>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn the_settings_take_effect_at_the_cr_and_nothing_answers() {
        let mut h = opened(&[]);
        h.write(b"V123");
        assert_eq!(h.cdc.model.voltage, 0, "not before its CR");
        h.write(b"\r");
        assert_eq!(h.cdc.model.voltage, 123);
        h.write(b"C045\r");
        assert_eq!(h.cdc.model.current, 45);
        assert_eq!(h.cdc.model.voltage, 123, "the other stays");
        assert_eq!(h.read(), [0u8; 0]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_ends_of_the_ranges_pass_and_one_past_them_is_flagged() {
        let mut h = opened(&[]);
        h.write(b"V000\rC000\rV350\rC255\r");
        assert_eq!(violations(&mut h), 0);
        h.write(b"V351\r");
        assert_eq!(violations(&mut h), 1, "V351");
        h.write(b"C256\r");
        assert_eq!(violations(&mut h), 1, "C256");
        assert_eq!((h.cdc.model.voltage, h.cdc.model.current), (350, 255), "ignored");
    }

    #[test]
    fn protection_is_900_and_901_and_starts_off() {
        let mut h = opened(&[]);
        assert!(!h.cdc.model.protection);
        h.write(b"V900\r");
        assert!(h.cdc.model.protection);
        h.write(b"V901\r");
        assert!(!h.cdc.model.protection);
        assert_eq!(h.cdc.model.voltage, 0, "not a voltage");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_nan_habit_changes_nothing_and_is_not_flagged() {
        let mut h = opened(&[]);
        h.write(b"V123\r");
        h.write(b"C045\r");
        h.write(b"V-21\r");
        h.write(b"C-21\r");
        assert_eq!((h.cdc.model.voltage, h.cdc.model.current), (123, 45));
        assert_eq!(violations(&mut h), 0);
        h.write(b"V-22\r");
        assert_eq!(violations(&mut h), 1, "only that one");
        h.write(b"X-21\r");
        assert_eq!(violations(&mut h), 1, "and only on V and C");
    }

    #[test]
    fn what_no_c_path_sends_is_flagged_and_ignored() {
        let mut h = opened(&[]);
        for frame in [
            &b"\r"[..],
            b"L\r",
            b"E\r",
            b"U\r",
            b"D\r",
            b"V400\r",
            b"V599\r",
            b"V600\r",
            b"V700\r",
            b"V800\r",
            b"V803\r",
            b"V902\r",
            b"V12\r",
            b"Vabc\r",
            b"X123\r",
            b"C900\r",
        ] {
            h.write(frame);
            assert_eq!(violations(&mut h), 1, "{:?}", String::from_utf8_lossy(frame));
        }
        let m = &h.cdc.model;
        assert_eq!((m.voltage, m.current, m.protection), (0, 0, false));
    }

    #[test]
    fn an_overlong_frame_is_flagged_once_and_the_next_one_counts() {
        let mut h = opened(&[]);
        h.write(b"V1234567890");
        assert_eq!(violations(&mut h), 1);
        h.write(b"V123\r");
        assert_eq!(violations(&mut h), 0, "still in the old frame, up to its CR");
        assert_eq!(h.cdc.model.voltage, 0);
        h.write(b"V124\r");
        assert_eq!(h.cdc.model.voltage, 124);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_frame_may_come_in_pieces() {
        let mut h = opened(&[]);
        for b in b"C123\r" {
            h.write(&[*b]);
        }
        assert_eq!(h.cdc.model.current, 123);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_uart_holds_a_wrong_line() {
        let o = [("uart".to_owned(), "4800/8n1".to_owned())];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.write(b"V123\r");
        assert_eq!(h.cdc.model.voltage, 0, "the supply hears nothing");
        assert_eq!(violations(&mut h), 0);
        let mut h = Host::open(function(&o).unwrap(), Line::parse("4800/8n1").unwrap(), true, true);
        h.write(b"V123\r");
        assert_eq!(h.cdc.model.voltage, 123);
    }

    #[test]
    fn a_reset_is_power_on() {
        let mut h = opened(&[]);
        h.write(b"V123\rC045\rV900\rV1");
        h.cdc.model.reset();
        let m = &h.cdc.model;
        assert_eq!((m.voltage, m.current, m.protection), (0, 0, false));
        h.write(b"V124\r");
        assert_eq!(h.cdc.model.voltage, 124, "the piece before the reset is gone");
        assert_eq!(violations(&mut h), 0);
        assert!(function(&[("no".to_owned(), "1".to_owned())]).is_err());
    }
}
