// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The `hameg-hmo` model's scope on CDC-ACM: what `src/scpi/serial.rs` does with this
//! driver's blocks and its silences. A separate model, as a scenario is on one tty
//! harness or the other. The real carrier is an HO720's FTDI chip or an HMO's virtual
//! COM port (`scpi_serial.c`'s table, 115200/8n1). The USB side here is the shell's,
//! with a test id: only the byte stream is the scope's. A two channel scope by default,
//! as a scope with fewer queries makes the counts in a scenario shorter.

use crate::Result;
use crate::model::DeviceModel;
use crate::models::hameg_hmo;
use crate::shells::cdc::Identity;

/// The model name.
pub const NAME: &str = "hameg-hmo-serial";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The hameg-hmo scope on CDC-ACM; its options, and the shell's stall=, \
                         chunk=, in-limit=, in-stall=, out-hold=, uart=";

/// A scope on a tty, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut with_default = vec![("model".to_owned(), "HMO1002".to_owned())];
    with_default.extend_from_slice(options);
    let (scope, config, _) = hameg_hmo::parts(&with_default)?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Hameg scope on a serial port");
    Ok(config.cdc(NAME, &id, scope))
}

/// A block is long: its digest in the log.
pub fn log_rules(options: &[(String, String)]) -> crate::txlog::Rules {
    hameg_hmo::log_rules(options)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_is_a_tty_and_takes_the_scopes_options() {
        let m = build(&[]).unwrap();
        assert_eq!(m.name(), NAME);
        assert!(build(&[("model".to_owned(), "RTB2002".to_owned())]).is_ok());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
