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
//! - [`ezusb`]: the Cypress FX2 without firmware, its boot loader
//!   (request `0xa0`, the 8051 reset in `CPUCS`) and the checks on an
//!   upload; the model renumerates as the firmware's device.
//! - [`cdc`]: a CDC-ACM function: its descriptors and class requests, with
//!   a model that is a byte stream ([`cdc::Serial`]), reached through
//!   `/dev/ttyACMn`.
//!
//! USBTMC comes next (`docs/ROADMAP.md`, "Emulation plan").

pub mod cdc;
pub mod ezusb;
pub mod ftdi;
pub mod hid;
pub mod hidcable;
