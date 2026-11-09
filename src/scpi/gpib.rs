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

/// What [`Gpib`] needs of a device on a board: [`Board`], or a test's stand-in.
pub trait Port: Send {
    /// `ibwrt`: a message, EOI with its last byte.
    fn write(&self, data: &[u8]) -> Result<()>;
    /// `ibrd`: up to `buf.len()` bytes within `timeout`, and whether EOI came.
    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<(usize, bool)>;
    /// `ibrsp`: the serial poll byte within `timeout`.
    fn serial_poll(&self, timeout: Duration) -> Result<u8>;
    /// `ibloc`: the device goes to local.
    fn go_to_local(&self) -> Result<()>;
    /// `ibonl(ud, 0)`.
    fn close(self: Box<Self>) -> Result<()>;
}

impl Port for Board {
    fn write(&self, data: &[u8]) -> Result<()> {
        Board::write(self, data)
    }

    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<(usize, bool)> {
        Board::read(self, buf, timeout)
    }

    fn serial_poll(&self, timeout: Duration) -> Result<u8> {
        Board::serial_poll(self, timeout)
    }

    fn go_to_local(&self) -> Result<()> {
        Board::go_to_local(self)
    }

    fn close(self: Box<Self>) -> Result<()> {
        Board::close(*self)
    }
}

/// How a closed device opens again.
type Opener = fn(Address) -> Result<Box<dyn Port>>;

fn open_board(address: Address) -> Result<Box<dyn Port>> {
    Ok(Box::new(Board::open(address)?))
}

/// A GPIB device as a SCPI transport.
pub struct Gpib {
    resource: String,
    address: Address,
    port: Option<Box<dyn Port>>,
    opener: Opener,
    complete: bool,
}

impl Gpib {
    /// Opens the device at `address`.
    pub fn open(address: Address) -> Result<Gpib> {
        Gpib::with_opener(address, open_board)
    }

    /// Opens the device at `address` with `opener`.
    fn with_opener(address: Address, opener: Opener) -> Result<Gpib> {
        Ok(Gpib {
            resource: format!("gpib/{address}"),
            address,
            port: Some(opener(address)?),
            opener,
            complete: false,
        })
    }

    fn port(&self) -> Result<&dyn Port> {
        self.port.as_deref().ok_or_else(|| Error::device("GPIB device is closed"))
    }
}

impl ScpiTransport for Gpib {
    fn resource(&self) -> &str {
        &self.resource
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.port()?.write(data)?;
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

    /// A read that fills `buf` ends nothing: only EOI does (`ibsta & END`).
    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let (n, end) = self.port()?.read(buf, timeout)?;
        self.complete = end;
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.complete
    }

    /// `scpi_gpib_close`: local, so the front panel isn't locked out, then the
    /// descriptor. A failed local is no reason to keep the descriptor.
    fn close(&mut self) -> Result<()> {
        let Some(port) = self.port.take() else { return Ok(()) };
        let local = port.go_to_local();
        let closed = port.close();
        local.and(closed)
    }

    fn reopen(&mut self) -> Result<()> {
        if self.port.is_none() {
            self.port = Some((self.opener)(self.address)?);
        }
        self.complete = false;
        Ok(())
    }

    fn serial_poll(&mut self) -> Result<u8> {
        let byte = self.port()?.serial_poll(SPOLL_TIMEOUT)?;
        log::trace!("Successful serial poll: {byte:#x}");
        Ok(byte)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scpi::Scpi;
    use std::sync::Mutex;

    /// Reads given out in turn: bytes and whether EOI ended them.
    static READS: Mutex<Vec<(Vec<u8>, bool)>> = Mutex::new(Vec::new());

    struct Pieces;

    impl Port for Pieces {
        fn write(&self, _: &[u8]) -> Result<()> {
            Ok(())
        }

        fn read(&self, buf: &mut [u8], _: Duration) -> Result<(usize, bool)> {
            let mut reads = READS.lock().unwrap();
            if reads.is_empty() {
                return Err(Error::Timeout);
            }
            let (bytes, end) = reads.remove(0);
            buf[..bytes.len()].copy_from_slice(&bytes);
            Ok((bytes.len(), end))
        }

        fn serial_poll(&self, _: Duration) -> Result<u8> {
            Ok(0x41)
        }

        fn go_to_local(&self) -> Result<()> {
            Ok(())
        }

        fn close(self: Box<Self>) -> Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_read_with_no_eoi_ends_nothing_and_eoi_ends_the_answer() {
        let address = Address { minor: 0, pad: 23, sad: None };
        let gpib = Gpib::with_opener(address, |_| Ok(Box::new(Pieces))).unwrap();
        assert_eq!(gpib.resource(), "gpib/0/23");
        let mut scpi = Scpi::new(Box::new(gpib));
        // An answer in two reads, as one that filled the buffer would come: the
        // first has no EOI and no error.
        *READS.lock().unwrap() =
            vec![(b"+1.23".to_vec(), false), (b"456E+0\r\n".to_vec(), true), (b"x".to_vec(), true)];
        assert_eq!(scpi.get_string("B").unwrap(), "+1.23456E+0");
        assert_eq!(scpi.get_data(None).unwrap(), b"x", "EOI ended the one before");
        assert_eq!(scpi.serial_poll().unwrap(), 0x41);
        scpi.close().unwrap();
        assert!(scpi.serial_poll().unwrap_err().to_string().contains("closed"));
        scpi.reopen().unwrap();
        assert_eq!(scpi.serial_poll().unwrap(), 0x41);
    }
}
