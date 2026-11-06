// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The `rigol-ds` model's scope on CDC-ACM, the template for a SCPI instrument
//! reached as a serial port (`gwinstek-gds-800`, `fluke-45`), and what runs
//! `src/scpi/serial.rs`, its close and its reopen. A separate model as a scenario is
//! on one tty harness or the other. The USB side is the shell's, with a test id: a
//! real scope's serial port is no USB device of its own, and only the byte stream is
//! the scope's. A DS2072 by default, as a serial read ends at a line end, and a
//! DS1102E has none.

use crate::Result;
use crate::model::DeviceModel;
use crate::models::rigol_ds;
use crate::shells::cdc::Identity;

/// The model name.
pub const NAME: &str = "rigol-ds-serial";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The rigol-ds scope on CDC-ACM; its options, and the shell's stall=, \
                         chunk=, in-limit=, in-stall=, out-hold=, uart=";

/// A scope on a tty, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut with_default = vec![("model".to_owned(), "DS2072".to_owned())];
    with_default.extend_from_slice(options);
    let (scope, config) = rigol_ds::parts(&with_default)?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Rigol scope on a serial port");
    Ok(config.cdc(NAME, &id, scope))
}

/// A frame is long: its digest in the log.
pub fn log_rules(options: &[(String, String)]) -> crate::txlog::Rules {
    rigol_ds::log_rules(options)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_is_a_tty_and_takes_the_scopes_options() {
        let m = build(&[]).unwrap();
        assert_eq!(m.name(), NAME);
        assert!(build(&[("model".to_owned(), "DS1054Z".to_owned())]).is_ok());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
