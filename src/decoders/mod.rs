//! Native protocol decoders. Each one reproduces the annotation
//! classes, rows and texts of the libsigrokdecode decoder of the same
//! id, so `-A` selections and existing scripts keep working, and is
//! written from the protocol itself.

#![forbid(unsafe_code)]

pub mod can;
pub mod i2c;
pub mod jtag;
pub mod mdio;
pub mod spi;
pub mod swd;
pub mod uart;

use crate::decode::DecoderDef;

/// Every decoder in this crate.
pub static ALL: [&DecoderDef; 7] =
    [&can::DEF, &i2c::DEF, &jtag::DEF, &mdio::DEF, &spi::DEF, &swd::DEF, &uart::DEF];
