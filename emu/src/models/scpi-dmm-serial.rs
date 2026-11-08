// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The `scpi-dmm` model's meter on CDC-ACM, as an OWON or a GW Instek is reached: a
//! separate model, as a scenario is on one tty harness or the other. The USB side is
//! the shell's, with a test id; only the byte stream is the meter's. An XDM2041 by
//! default, which is the one with no `*OPC?`.

use crate::Result;
use crate::model::DeviceModel;
use crate::models::scpi_dmm;
use crate::shells::cdc::Identity;

/// The model name.
pub const NAME: &str = "scpi-dmm-serial";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The scpi-dmm meter on CDC-ACM; its options, and the shell's stall=, \
                         chunk=, in-limit=, in-stall=, out-hold=, uart=";

/// A meter on a tty, from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut with_default = vec![("model".to_owned(), "XDM2041".to_owned())];
    with_default.extend_from_slice(options);
    let (meter, config) = scpi_dmm::parts(&with_default)?;
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "SCPI meter on a serial port");
    Ok(config.cdc(NAME, &id, meter))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_is_a_tty_and_takes_the_meters_options() {
        let m = build(&[]).unwrap();
        assert_eq!(m.name(), NAME);
        assert!(build(&[("model".to_owned(), "GDM8255A".to_owned())]).is_ok());
        assert!(build(&[("no-such".to_owned(), "1".to_owned())]).is_err());
    }
}
