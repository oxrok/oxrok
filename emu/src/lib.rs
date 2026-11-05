// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The devices oxrok's drivers are checked against, on [`vusb`].
//!
//! The contract, the bus, the class shells, USB/IP and the control
//! socket are the `vusb` crate's; its paths are re-exported here, so a
//! model says `crate::model::DeviceModel` or `crate::shells::cdc`
//! whichever crate holds them. What is oxrok's own:
//!
//! - [`models`]: the devices, one file each, found by `build.rs`.
//! - [`txlog`]: the transaction log, an observer of the bus: one line
//!   per completed transfer in the dialect of oxrok's USB fixtures, and
//!   a `! violation:` line whenever the host leaves the protocol as the
//!   model defines it. `tests/emu` holds this format.
//! - [`shells::hidcable`] and the checks on a firmware upload in
//!   [`shells::ezusb`], which know sigrok's fixtures.
//!
//! Only Linux can attach the device; the models build and test
//! everywhere.

#![forbid(unsafe_code)]

pub use vusb::{Error, Result, arm, descriptor, device, model};

pub mod models;
pub mod options;
pub mod shells;
pub mod txlog;
