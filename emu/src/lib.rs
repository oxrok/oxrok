// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! Device emulation over USB/IP.
//!
//! A device is a [`model::DeviceModel`]: descriptors, control
//! transfers, OUT and IN transfers, reset. Plain Rust with no I/O, so
//! the same model runs in a unit test, on this server and, later, on a
//! microcontroller. [`device::Device`] wraps a model with the standard
//! requests every USB device answers (descriptors, configuration,
//! status, halts) and the checks on what the host sends. [`shells`]
//! carry what every device of a kind shares (HID so far), and
//! [`models`] holds the devices, one file each, found by `build.rs`.
//! [`usbip`] is
//! the USB/IP server the Linux `vhci-hcd` module connects to, and
//! [`vhci`] attaches and detaches through its sysfs files, so no
//! `usbip` tool is needed. Everything the host does goes to a
//! [`txlog::TxLog`]: one line per completed transfer, in the dialect
//! of oxrok's USB fixtures, and a `! violation:` line whenever the
//! host leaves the protocol as the model defines it.
//!
//! Only Linux can attach the device; the models and the protocol code
//! build and test everywhere.

#![forbid(unsafe_code)]

pub mod descriptor;
pub mod device;
pub mod model;
pub mod models;
pub mod options;
pub mod shells;
pub mod txlog;
pub mod usbip;
pub mod vhci;

use std::fmt;

/// An error with a message.
#[derive(Debug)]
pub struct Error(pub String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Error {
        Error(e.to_string())
    }
}

impl From<String> for Error {
    fn from(s: String) -> Error {
        Error(s)
    }
}

impl From<&str> for Error {
    fn from(s: &str) -> Error {
        Error(s.to_owned())
    }
}

/// Result with [`Error`].
pub type Result<T> = std::result::Result<T, Error>;
