// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Transport shells: the parts every device of a kind shares, which a
//! model builds on and fills in with what its own protocol means. HID,
//! FTDI, EZ-USB and CDC-ACM are vusb's (`vusb::shells`), re-exported
//! under the names the models use.
//!
//! - [`hidcable`]: what the serial over HID cable models (`bu86x`,
//!   `ch9325`, `victor`) share: the Pico emulator's descriptors, the
//!   meter packets they play, a poll counted stream.
//!
//! USBTMC comes next (`docs/ROADMAP.md`, "Emulation plan").

pub mod hid {
    //! vusb's HID shell.
    pub use vusb::shells::hid::*;
}
