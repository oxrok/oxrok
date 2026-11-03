//! Transport shells: the parts every device of a kind shares, which a
//! model builds on and fills in with what its own protocol means.
//!
//! - [`hid`]: the HID class descriptor, the report descriptor fetch and
//!   the class requests (`GET_REPORT`, `SET_REPORT`, idle, protocol).
//! - [`ftdi`]: the FTDI chips' descriptors, vendor requests (reset,
//!   purge, baud rate, line, flow and modem control, latency timer, bit
//!   modes, pins, EEPROM) and the two status bytes of every IN packet.
//! - [`hidcable`]: what the serial over HID cable models (`bu86x`,
//!   `ch9325`, `victor`) share: the Pico emulator's descriptors, the
//!   meter packets they play, a poll counted stream.
//!
//! The EZ-USB shell (firmware upload to RAM, renumeration) comes next;
//! CDC-ACM and USBTMC after it (`docs/ROADMAP.md`, "Emulation plan").

pub mod ftdi;
pub mod hid;
pub mod hidcable;
