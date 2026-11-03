//! Transport shells: the parts every device of a kind shares, which a
//! model builds on and fills in with what its own protocol means.
//!
//! - [`hid`]: the HID class descriptor, the report descriptor fetch and
//!   the class requests (`GET_REPORT`, `SET_REPORT`, idle, protocol).
//! - [`hidcable`]: what the serial over HID cable models (`bu86x`,
//!   `ch9325`, `victor`) share: the Pico emulator's descriptors, the
//!   meter packets they play, a poll counted stream.
//!
//! The FTDI shell (vendor requests, latency timer, the two modem
//! status bytes per packet) and the EZ-USB shell (firmware upload to
//! RAM, renumeration) come next; CDC-ACM and USBTMC after them
//! (`docs/ROADMAP.md`, "Emulation plan").

pub mod hid;
pub mod hidcable;
