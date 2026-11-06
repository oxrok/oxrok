// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The `scpi-pps` model's supply on CDC-ACM, as a real one on its RS-232 port is
//! reached. The USB side is the shell's, with a test id: only the byte stream is the
//! supply's. Text ends in a line feed, which is what ends a serial read.

use crate::Result;
use crate::model::DeviceModel;
use crate::models::scpi_pps;
use crate::shells::cdc::Identity;

/// The model name.
pub const NAME: &str = "scpi-pps-serial";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The scpi-pps supply on CDC-ACM; its options, and the shell's stall=, \
                         chunk=, in-limit=, in-stall=, out-hold=, uart=";

/// A supply on a tty, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let (supply, config) = scpi_pps::parts(options)?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "SCPI supply on a serial port");
    Ok(config.cdc(NAME, &id, supply))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_is_a_tty_and_takes_the_supplys_options() {
        let m = build(&[]).unwrap();
        assert_eq!(m.name(), NAME);
        assert!(build(&[("model".to_owned(), "6632B".to_owned())]).is_ok());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
