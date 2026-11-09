// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The device models, by name.
//!
//! Each file `NAME.rs` (or directory `NAME/mod.rs`) here is one model;
//! `build.rs` finds them and generates the module declarations, the
//! [`NAMES`] table and the [`build`] dispatch, so adding a model touches
//! no shared file. A model module exports:
//!
//! - `pub const NAME: &str`, the name `oxrok-emu serve NAME` takes,
//!   equal to the file name (a `-` in it becomes `_` in the module
//!   identifier);
//! - `pub const ABOUT: &str`, one line for `oxrok-emu list`: what the
//!   device is and the options it takes;
//! - `pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>>`,
//!   which parses the `-o KEY=VALUE` options and rejects unknown ones;
//! - optionally `pub fn build_tcp(options)`, the same device on a socket
//!   (`shells::tcp`), for `oxrok-emu serve --tcp`;
//! - optionally `pub fn build_sysfs(options)`, the same device as a tree of
//!   files (`shells::sysfs`), for `oxrok-emu serve --sysfs`;
//! - optionally `pub const STANDIN: &str` and
//!   `pub fn standin(args: &[OsString]) -> u8`, for a device a driver
//!   reaches through a program it runs: `oxrok-emu` run under the name
//!   `STANDIN` (a link to it) is that program ([`standin`]).

use crate::model::DeviceModel;
use crate::{Error, Result};

include!(concat!(env!("OUT_DIR"), "/models.rs"));

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_model_is_named_after_its_file() {
        assert_eq!(NAMES.len(), FILES.len());
        for ((name, about), file) in NAMES.iter().zip(FILES) {
            assert_eq!(name, file, "a model's NAME is its file name");
            assert!(!about.is_empty(), "{name} has no ABOUT");
        }
    }

    #[test]
    fn every_model_builds_with_no_options() {
        for (name, _) in NAMES {
            let m = build(name, &[]).unwrap_or_else(|e| panic!("{name}: {e}"));
            assert_eq!(m.name(), *name, "DeviceModel::name of {name}");
        }
        assert!(build("no-such-model", &[]).is_err());
        assert!(build(NAMES[0].0, &[("no-such-option".into(), "1".into())]).is_err());
    }
}
