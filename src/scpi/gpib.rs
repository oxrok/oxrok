// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi_libgpib.c,
//   Copyright (C) 2014 Martin Ling <martin-sigrok@earth.li>,
//   with commits by Frank Stettner, Bert Vermeulen, Alexandru Gagniuc,
//   Daniel Elstner, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! SCPI over GPIB: `gpib/MINOR/PAD[/SAD]`, a device on the kernel's board
//! `/dev/gpibMINOR` ([`crate::gpib`]).
//!
//! libsigrok's `libgpib/NAME` looks `NAME` up in linux-gpib's `/etc/gpib.conf`,
//! which oxrok does not read, so the address is given instead. The transport is
//! `scpi_libgpib.c`'s: a message goes out whole with EOI on its last byte, a read
//! takes what comes up to EOI, which ends the response (`read_complete` is
//! `ibsta & END`), a close sends the device to local (`ibloc`) and drops the
//! descriptor (`ibonl(ud, 0)`), and [`ScpiTransport::serial_poll`] is
//! `sr_scpi_gpib_spoll`. A read that times out with nothing has taken its whole
//! timeout, so a caller's wait asks the bus once per timeout.

#![forbid(unsafe_code)]

use super::ScpiTransport;
use crate::core::{Error, Result};
use crate::gpib::{Address, Board, SPOLL_TIMEOUT};
use std::time::Duration;

/// A GPIB device as a SCPI transport.
pub struct Gpib {
    resource: String,
    address: Address,
    board: Option<Board>,
    complete: bool,
}

impl Gpib {
    /// Opens the device at `address`.
    pub fn open(address: Address) -> Result<Gpib> {
        let board = Board::open(address)?;
        Ok(Gpib {
            resource: format!("gpib/{address}"),
            address,
            board: Some(board),
            complete: false,
        })
    }

    fn board(&self) -> Result<&Board> {
        self.board.as_ref().ok_or_else(|| Error::device("GPIB device is closed"))
    }
}

impl ScpiTransport for Gpib {
    fn resource(&self) -> &str {
        &self.resource
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.board()?.write(data)?;
        log::trace!(
            "Successfully sent SCPI command: '{}'.",
            String::from_utf8_lossy(data).trim_end()
        );
        Ok(())
    }

    fn read_begin(&mut self) -> Result<()> {
        self.complete = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let (n, end) = self.board()?.read(buf, timeout)?;
        self.complete = end;
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.complete
    }

    /// `scpi_gpib_close`: local, so the front panel isn't locked out, then the
    /// descriptor. A failed local is no reason to keep the descriptor.
    fn close(&mut self) -> Result<()> {
        let Some(board) = self.board.take() else { return Ok(()) };
        let local = board.go_to_local();
        let closed = board.close();
        local.and(closed)
    }

    fn reopen(&mut self) -> Result<()> {
        if self.board.is_none() {
            self.board = Some(Board::open(self.address)?);
        }
        self.complete = false;
        Ok(())
    }

    fn serial_poll(&mut self) -> Result<u8> {
        let byte = self.board()?.serial_poll(SPOLL_TIMEOUT)?;
        log::trace!("Successful serial poll: {byte:#x}");
        Ok(byte)
    }
}
