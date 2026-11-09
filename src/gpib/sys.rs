// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi_libgpib.c,
//   Copyright (C) 2014 Martin Ling <martin-sigrok@earth.li>,
//   with commits by Frank Stettner, Bert Vermeulen, Alexandru Gagniuc,
//   Daniel Elstner, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The ioctls of `/dev/gpibN`, each with the struct `gpib_ioctl.h` gives it.

use super::{Address, IO_TIMEOUT, errno_error, receive_setup, send_setup, usec};
use crate::core::{Error, Result};
use rustix::ioctl::{Opcode, Updater, ioctl, opcode};
use std::fs::{File, OpenOptions};
use std::time::{Duration, Instant};

/// `GPIB_CODE`.
const GPIB_CODE: u8 = 160;

/// `struct gpib_read_write_ioctl`.
#[repr(C)]
#[derive(Default)]
struct ReadWrite {
    buffer_ptr: u64,
    requested_transfer_count: u32,
    completed_transfer_count: u32,
    end: i32,
    handle: i32,
}

/// `struct gpib_open_dev_ioctl`.
#[repr(C)]
struct OpenDev {
    handle: u32,
    pad: u32,
    sad: i32,
    is_board: u32,
}

/// `struct gpib_close_dev_ioctl`.
#[repr(C)]
struct CloseDev {
    handle: u32,
}

/// `struct gpib_serial_poll_ioctl`.
#[repr(C)]
struct SerialPoll {
    pad: u32,
    sad: i32,
    status_byte: u8,
    padding: [u8; 3],
}

/// `struct gpib_eos_ioctl`.
#[repr(C)]
struct Eos {
    eos: i32,
    eos_flags: i32,
}

/// `struct gpib_board_info_ioctl`, its bit fields as one word.
#[repr(C)]
#[derive(Default)]
struct BoardInfo {
    pad: u32,
    sad: i32,
    parallel_poll_configuration: i32,
    autopolling: i32,
    is_system_controller: i32,
    t1_delay: u32,
    bits: u32,
}

const IBRD: Opcode = opcode::read_write::<ReadWrite>(GPIB_CODE, 100);
const IBWRT: Opcode = opcode::read_write::<ReadWrite>(GPIB_CODE, 101);
const IBCMD: Opcode = opcode::read_write::<ReadWrite>(GPIB_CODE, 102);
const IBOPENDEV: Opcode = opcode::read_write::<OpenDev>(GPIB_CODE, 3);
const IBCLOSEDEV: Opcode = opcode::write::<CloseDev>(GPIB_CODE, 4);
const IBTMO: Opcode = opcode::write::<u32>(GPIB_CODE, 17);
const IBEOS: Opcode = opcode::write::<Eos>(GPIB_CODE, 19);
const IBRSP: Opcode = opcode::read_write::<SerialPoll>(GPIB_CODE, 18);
const IBMUTEX: Opcode = opcode::write::<i32>(GPIB_CODE, 26);
const IBBOARD_INFO: Opcode = opcode::read::<BoardInfo>(GPIB_CODE, 29);

/// One ioctl, its errno on failure.
///
/// # Safety
///
/// `OP` must be the opcode `gpib_ioctl.h` gives `T`, and any buffer `T` points at
/// must be valid for the lengths it names for the whole call.
unsafe fn call<const OP: Opcode, T>(file: &File, value: &mut T) -> std::result::Result<(), i32> {
    // SAFETY: the caller pairs the opcode with its `repr(C)` struct, which lives
    // for the call, and vouches for any buffer it points at.
    unsafe { ioctl(file, Updater::<OP, T>::new(value)) }.map_err(rustix::io::Errno::raw_os_error)
}

/// The board's lock, held by this thread: the kernel takes I/O only from the thread
/// (`current->pid`) that holds it.
struct Locked<'a>(&'a File);

impl<'a> Locked<'a> {
    fn take(file: &'a File, address: Address) -> Result<Locked<'a>> {
        // SAFETY: IBMUTEX takes an int.
        unsafe { call::<IBMUTEX, _>(file, &mut 1i32) }
            .map_err(|e| errno_error(e, "lock", address))?;
        Ok(Locked(file))
    }
}

impl Drop for Locked<'_> {
    fn drop(&mut self) {
        // SAFETY: IBMUTEX takes an int.
        if let Err(e) = unsafe { call::<IBMUTEX, _>(self.0, &mut 0i32) } {
            log::warn!("GPIB: the board's lock did not let go: errno {e}");
        }
    }
}

/// A device on a GPIB board, open: what `ibfind` gives libsigrok.
pub struct Board {
    file: File,
    address: Address,
    handle: i32,
    own_pad: u32,
    own_sad: Option<u32>,
}

impl Board {
    /// Opens `/dev/gpibMINOR` and a descriptor for the device (`IBOPENDEV`), after
    /// reading the board's own address (`IBBOARD_INFO`).
    pub fn open(address: Address) -> Result<Board> {
        let node = format!("/dev/gpib{}", address.minor);
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&node)
            .map_err(|e| Error::Io(std::io::Error::new(e.kind(), format!("{node}: {e}"))))?;
        let mut info = BoardInfo::default();
        // SAFETY: IBBOARD_INFO fills a gpib_board_info_ioctl.
        unsafe { call::<IBBOARD_INFO, _>(&file, &mut info) }
            .map_err(|e| errno_error(e, "board info", address))?;
        let sad = address.sad.map_or(-1, |s| s as i32);
        let mut dev = OpenDev { handle: 0, pad: address.pad, sad, is_board: 0 };
        // SAFETY: IBOPENDEV takes and returns a gpib_open_dev_ioctl.
        unsafe { call::<IBOPENDEV, _>(&file, &mut dev) }
            .map_err(|e| errno_error(e, "open", address))?;
        Ok(Board {
            file,
            address,
            handle: dev.handle as i32,
            own_pad: info.pad,
            own_sad: u32::try_from(info.sad).ok(),
        })
    }

    /// The device's address.
    pub fn address(&self) -> Address {
        self.address
    }

    fn timeout(&self, timeout: Duration) -> Result<()> {
        // SAFETY: IBTMO takes an unsigned int.
        unsafe { call::<IBTMO, _>(&self.file, &mut usec(timeout)) }
            .map_err(|e| errno_error(e, "timeout", self.address))
    }

    /// `IBRD`, `IBWRT` or `IBCMD` on the buffer at `buffer_ptr`, `len` bytes long:
    /// its count done and its end flag.
    ///
    /// # Safety
    ///
    /// `OP` is `IBRD`, `IBWRT` or `IBCMD`, and `buffer_ptr` points at `len` bytes
    /// valid for the call, writable for `IBRD`.
    unsafe fn transfer<const OP: Opcode>(
        &self,
        buffer_ptr: u64,
        len: usize,
        end: bool,
        what: &str,
    ) -> (usize, bool, Result<()>) {
        let mut rw = ReadWrite {
            buffer_ptr,
            requested_transfer_count: u32::try_from(len).unwrap_or(u32::MAX),
            end: i32::from(end),
            handle: self.handle,
            ..ReadWrite::default()
        };
        // SAFETY: the opcode takes a gpib_read_write_ioctl, and the caller vouches
        // for the buffer.
        let r = unsafe { call::<OP, _>(&self.file, &mut rw) };
        let done = (rw.completed_transfer_count as usize).min(len);
        (done, rw.end != 0, r.map_err(|e| errno_error(e, what, self.address)))
    }

    /// `IBWRT` or `IBCMD` of `data`, which the kernel only reads.
    fn send<const OP: Opcode>(&self, data: &[u8], end: bool, what: &str) -> Result<()> {
        // SAFETY: IBWRT and IBCMD copy from the buffer and never write it, and the
        // slice is valid for its length for the call.
        let (done, _, r) =
            unsafe { self.transfer::<OP>(data.as_ptr() as u64, data.len(), end, what) };
        r?;
        if done < data.len() {
            return Err(Error::device(format!(
                "GPIB: {what}: {done} of {} bytes went out",
                data.len()
            )));
        }
        Ok(())
    }

    fn command(&self, bytes: &[u8]) -> Result<()> {
        self.send::<IBCMD>(bytes, false, "command")
    }

    /// `iblcleos`: no end of string mode, so a read ends at EOI alone, whatever an
    /// earlier program left set. A binary answer can hold any byte.
    fn clear_eos(&self) -> Result<()> {
        let mut eos = Eos { eos: 0, eos_flags: 0 };
        // SAFETY: IBEOS takes a gpib_eos_ioctl.
        unsafe { call::<IBEOS, _>(&self.file, &mut eos) }
            .map_err(|e| errno_error(e, "end of string mode", self.address))
    }

    /// `ibwrt`: `data` to the device, EOI with its last byte.
    pub fn write(&self, data: &[u8]) -> Result<()> {
        let _lock = Locked::take(&self.file, self.address)?;
        self.timeout(IO_TIMEOUT)?;
        self.command(&send_setup(self.own_pad, self.own_sad, self.address))?;
        self.send::<IBWRT>(data, true, "write")
    }

    /// `ibrd`: up to `buf.len()` bytes from the device, waiting at most `timeout`.
    /// How many came and whether the last had EOI; a timeout with some bytes is
    /// those bytes, with no end. A board that gives up sooner than `timeout` (an
    /// emulated one answers at once) is waited out, so that a timeout takes its time
    /// and a caller's loop asks once per timeout, as on a real bus.
    pub fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<(usize, bool)> {
        let began = Instant::now();
        let read = {
            let _lock = Locked::take(&self.file, self.address)?;
            self.timeout(timeout)?;
            self.clear_eos()?;
            self.command(&receive_setup(self.own_pad, self.own_sad, self.address))?;
            // SAFETY: the slice is writable for its length for the call.
            unsafe { self.transfer::<IBRD>(buf.as_mut_ptr() as u64, buf.len(), false, "read") }
        };
        match read {
            (n, end, Ok(())) => Ok((n, end)),
            (n, _, Err(Error::Timeout)) => {
                std::thread::sleep(timeout.saturating_sub(began.elapsed()));
                if n > 0 { Ok((n, false)) } else { Err(Error::Timeout) }
            }
            (_, _, Err(e)) => Err(e),
        }
    }

    /// `ibrsp`: the device's serial poll byte, waiting at most `timeout`.
    pub fn serial_poll(&self, timeout: Duration) -> Result<u8> {
        let _lock = Locked::take(&self.file, self.address)?;
        self.timeout(timeout)?;
        let sad = self.address.sad.map_or(-1, |s| s as i32);
        let mut poll = SerialPoll { pad: self.address.pad, sad, status_byte: 0, padding: [0; 3] };
        // SAFETY: IBRSP takes and returns a gpib_serial_poll_ioctl.
        unsafe { call::<IBRSP, _>(&self.file, &mut poll) }
            .map_err(|e| errno_error(e, "serial poll", self.address))?;
        Ok(poll.status_byte)
    }

    /// `ibloc` on a device descriptor: the write's addressing and `GTL`.
    pub fn go_to_local(&self) -> Result<()> {
        let _lock = Locked::take(&self.file, self.address)?;
        self.timeout(IO_TIMEOUT)?;
        let mut c = send_setup(self.own_pad, self.own_sad, self.address);
        c.push(super::cmd::GTL);
        self.command(&c)
    }

    /// `ibonl(ud, 0)`: the descriptor goes, then the node.
    pub fn close(self) -> Result<()> {
        let mut dev = CloseDev { handle: self.handle as u32 };
        // SAFETY: IBCLOSEDEV takes a gpib_close_dev_ioctl.
        unsafe { call::<IBCLOSEDEV, _>(&self.file, &mut dev) }
            .map_err(|e| errno_error(e, "close", self.address))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_structs_are_the_headers_sizes() {
        // gpib_ioctl.h on x86-64 and aarch64: the opcodes carry these sizes.
        assert_eq!(std::mem::size_of::<ReadWrite>(), 24);
        assert_eq!(std::mem::size_of::<OpenDev>(), 16);
        assert_eq!(std::mem::size_of::<CloseDev>(), 4);
        assert_eq!(std::mem::size_of::<SerialPoll>(), 12);
        assert_eq!(std::mem::size_of::<BoardInfo>(), 28);
        // _IOWR(160, 100, 24 bytes) and _IOW(160, 17, unsigned int).
        assert_eq!(IBRD, 0xc018_a064);
        assert_eq!(IBTMO, 0x4004_a011);
        assert_eq!(IBBOARD_INFO, 0x801c_a01d);
    }

    #[test]
    fn a_board_that_is_not_there_says_which() {
        let e = Board::open(Address { minor: 15, pad: 1, sad: None }).err().unwrap();
        assert!(e.to_string().contains("/dev/gpib15"), "{e}");
    }
}
