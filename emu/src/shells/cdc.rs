// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! vusb's CDC-ACM shell, and a stand-in host for the models' unit tests.
//!
//! What a model on it means for the harness (`tools/diffcli.sh`): the
//! tools reach it through `/dev/ttyACMn`, the tty goes back to 9600/8n1
//! before each run, which is also what a fresh model assumes, and a
//! `.tx` joins the bulk transfers a direction makes in a row.

pub use vusb::shells::cdc::*;

/// A tool with the tty open, as seen from the function, for unit tests.
#[cfg(test)]
pub mod host {
    use super::*;
    use crate::model::{DeviceModel, Reply, Setup, Violations};

    /// `SET_LINE_CODING`.
    const SET_LINE_CODING: u8 = 0x20;
    /// `SET_CONTROL_LINE_STATE`.
    const SET_CONTROL_LINE_STATE: u8 = 0x22;

/// A function with the tty open.
    pub struct Host<S> {
        /// The function.
        pub cdc: Cdc<S>,
        /// What it reported.
        pub v: Violations,
    }

    impl<S: Serial> Host<S> {
        /// Sets the line and the lines, as a tool's open with these
        /// settings does: `dtr` and `rts` as given.
        pub fn open(cdc: Cdc<S>, line: Line, dtr: bool, rts: bool) -> Host<S> {
            let mut h = Host { cdc, v: Violations::default() };
            h.set_lines(true, true);
            h.set_line(line);
            h.set_lines(dtr, rts);
            h
        }

        /// `SET_LINE_CODING`.
        pub fn set_line(&mut self, line: Line) -> Reply {
            let s = setup(0x21, SET_LINE_CODING, 0, 7);
            self.cdc.control(s, &line.to_bytes(), &mut self.v)
        }

        /// `SET_CONTROL_LINE_STATE`.
        pub fn set_lines(&mut self, dtr: bool, rts: bool) -> Reply {
            let value = u16::from(dtr) | u16::from(rts) << 1;
            self.cdc.control(setup(0x21, SET_CONTROL_LINE_STATE, value, 0), &[], &mut self.v)
        }

        /// A `write()`.
        pub fn write(&mut self, data: &[u8]) -> Reply {
            self.cdc.out(EP_DATA, data, &mut self.v)
        }

        /// One read of up to 128 bytes, the kernel's on a full speed
        /// function.
        pub fn offer(&mut self) -> Reply {
            self.cdc.in_transfer(EP_DATA, 128, &mut self.v)
        }

        /// All there is to read now.
        pub fn read(&mut self) -> Vec<u8> {
            let mut out = Vec::new();
            while let Reply::Data(d) = self.offer() {
                out.extend(d);
            }
            out
        }
    }

    /// A class setup packet to the control interface.
    pub fn setup(request_type: u8, request: u8, value: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index: 0, length }
    }
}
