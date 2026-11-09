// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi_libgpib.c,
//   Copyright (C) 2014 Martin Ling <martin-sigrok@earth.li>,
//   with commits by Frank Stettner, Bert Vermeulen, Alexandru Gagniuc,
//   Daniel Elstner, Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! A GPIB device through the Linux kernel's `gpib` subsystem: `/dev/gpibN` and its
//! ioctls (`include/uapi/linux/gpib_ioctl.h`), with no linux-gpib userspace.
//!
//! libsigrok reaches GPIB through linux-gpib's `libgpib` (`scpi_libgpib.c`): `ibfind`
//! of a name in `/etc/gpib.conf`, `ibwrt`, `ibrd`, `ibrsp`, `ibloc` and `ibonl`.
//! This does what those calls do on the wire, from libgpib's source (`ibWrt.c`,
//! `ibRd.c`, `ibCmd.c`, `ibLoc.c`, `ibutil.c` of linux-gpib-user): each operation
//! takes the board's lock (`IBMUTEX`), sets the board's timeout (`IBTMO`), addresses
//! the device with command bytes (`IBCMD`) and then moves the data. A write is
//! addressed `MTA(board) UNL MLA(device)` and sends EOI with its last byte; a read
//! `UNL MLA(board) MTA(device)` and ends at EOI; going to local is the write's
//! addressing and `GTL`. The serial poll is the kernel's own (`IBRSP`).
//!
//! The board must be online: `gpib_config` (or `oxrok-emu gpib-config` on a tester)
//! sets its type and brings it up, which needs `CAP_SYS_ADMIN`; nothing here does.
//! An unconfigured board answers `EINVAL` or `ENODEV`, which [`Board`] names.
//!
//! This is the one module besides `pydecode` with `unsafe`: the ioctls, each paired
//! with the struct `gpib_ioctl.h` gives it, in `sys`.

use crate::core::Error;
#[cfg(not(target_os = "linux"))]
use crate::core::Result;
use std::time::Duration;

/// The default timeout of a libgpib descriptor (`init_descriptor_settings`: 3 s,
/// and T3s is `gpib.conf`'s usual one), for writes and reads.
pub const IO_TIMEOUT: Duration = Duration::from_secs(3);
/// libgpib's default serial poll timeout (`spoll_usec_timeout`, 1 s).
pub const SPOLL_TIMEOUT: Duration = Duration::from_secs(1);

/// Bus commands (`gpib_cmd.h`).
pub mod cmd {
    /// Go to local.
    pub const GTL: u8 = 0x01;
    /// Unlisten.
    pub const UNL: u8 = 0x3f;
    /// My listen address of `a`.
    pub const fn mla(a: u32) -> u8 {
        0x20 | restrict(a)
    }
    /// My talk address of `a`.
    pub const fn mta(a: u32) -> u8 {
        0x40 | restrict(a)
    }
    /// My secondary address of `a`.
    pub const fn msa(a: u32) -> u8 {
        0x60 | (a & 0x1f) as u8
    }
    /// `gpib_address_restrict`.
    const fn restrict(a: u32) -> u8 {
        let a = (a & 0x1f) as u8;
        if a == 0x1f { 0 } else { a }
    }
}

/// Where a device is: the board's minor and its primary and secondary addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Address {
    /// `/dev/gpibMINOR`.
    pub minor: u32,
    /// Primary address, 0 to 30.
    pub pad: u32,
    /// Secondary address, 0 to 30, or none.
    pub sad: Option<u32>,
}

impl Address {
    /// `MINOR/PAD[/SAD]`, as `gpib/` resources name it.
    pub fn parse(text: &str) -> Option<Address> {
        let mut parts = text.split('/');
        let minor = parts.next()?.parse().ok().filter(|m| *m < 16)?;
        let pad = parts.next()?.parse().ok().filter(|p| *p <= 30)?;
        let sad = match parts.next() {
            Some(s) => Some(s.parse().ok().filter(|s| *s <= 30)?),
            None => None,
        };
        parts.next().is_none().then_some(Address { minor, pad, sad })
    }
}

impl std::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.minor, self.pad)?;
        if let Some(sad) = self.sad {
            write!(f, "/{sad}")?;
        }
        Ok(())
    }
}

/// The command bytes that address `device` to listen with the board talking: libgpib's
/// `create_send_setup`.
#[allow(clippy::similar_names)]
pub fn send_setup(own_pad: u32, own_sad: Option<u32>, device: Address) -> Vec<u8> {
    let mut c = vec![cmd::mta(own_pad)];
    c.extend(own_sad.map(cmd::msa));
    c.push(cmd::UNL);
    c.push(cmd::mla(device.pad));
    c.extend(device.sad.map(cmd::msa));
    c
}

/// The command bytes that address `device` to talk with the board listening: libgpib's
/// `InternalReceiveSetup`.
#[allow(clippy::similar_names)]
pub fn receive_setup(own_pad: u32, own_sad: Option<u32>, device: Address) -> Vec<u8> {
    let mut c = vec![cmd::UNL, cmd::mla(own_pad)];
    c.extend(own_sad.map(cmd::msa));
    c.push(cmd::mta(device.pad));
    c.extend(device.sad.map(cmd::msa));
    c
}

/// A timeout as the board takes it, in microseconds; 0 would be none at all.
fn usec(timeout: Duration) -> u32 {
    u32::try_from(timeout.as_micros()).unwrap_or(u32::MAX).max(1)
}

#[cfg(target_os = "linux")]
mod sys;

#[cfg(target_os = "linux")]
pub use sys::Board;

/// A GPIB device on a board of the kernel's `gpib` subsystem, which only Linux has.
#[cfg(not(target_os = "linux"))]
pub struct Board;

#[cfg(not(target_os = "linux"))]
#[allow(clippy::unused_self, clippy::needless_pass_by_value)]
impl Board {
    /// No board here.
    pub fn open(address: Address) -> Result<Board> {
        let _ = (address, usec(IO_TIMEOUT));
        Err(Self::none())
    }

    fn none() -> Error {
        Error::not_supported("GPIB needs Linux's gpib subsystem")
    }

    /// As on Linux; never reached, as no board opens.
    pub fn write(&self, data: &[u8]) -> Result<()> {
        let _ = data;
        Err(Self::none())
    }

    /// As on Linux; never reached.
    pub fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<(usize, bool)> {
        let _ = (buf, timeout);
        Err(Self::none())
    }

    /// As on Linux; never reached.
    pub fn serial_poll(&self, timeout: Duration) -> Result<u8> {
        let _ = timeout;
        Err(Self::none())
    }

    /// As on Linux; never reached.
    pub fn go_to_local(&self) -> Result<()> {
        Err(Self::none())
    }

    /// As on Linux; never reached.
    pub fn close(self) -> Result<()> {
        Err(Self::none())
    }
}

/// What an errno from the board means, for operation `what` on `address`.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn errno_error(errno: i32, what: &str, address: Address) -> Error {
    const EIO: i32 = 5;
    const ENODEV: i32 = 19;
    const EINVAL: i32 = 22;
    const ETIMEDOUT: i32 = 110;
    const ECOMM: i32 = 70;
    const ENOTCONN: i32 = 107;
    let minor = address.minor;
    match errno {
        ETIMEDOUT => Error::Timeout,
        ENODEV => Error::device(format!(
            "GPIB board {minor}: no adapter (unplugged, or no board type configured: gpib_config)"
        )),
        EINVAL => Error::device(format!(
            "GPIB board {minor}: {what} refused; is the board online (gpib_config)?"
        )),
        ECOMM => Error::device(format!("GPIB: nothing listens at address {}", address.pad)),
        ENOTCONN => Error::device(format!("GPIB board {minor}: no device on the bus")),
        EIO if what == "serial poll" => {
            Error::device(format!("GPIB: the serial poll of address {} got no answer", address.pad))
        }
        _ => Error::Io(std::io::Error::other(format!(
            "GPIB board {minor}: {what}: {}",
            std::io::Error::from_raw_os_error(errno)
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addresses_parse_and_print() {
        let a = Address::parse("0/23").unwrap();
        assert_eq!(a, Address { minor: 0, pad: 23, sad: None });
        assert_eq!(a.to_string(), "0/23");
        assert_eq!(Address::parse("15/30/2").unwrap().to_string(), "15/30/2");
        for bad in ["", "0", "16/1", "0/31", "0/1/31", "0/1/2/3", "x/1", "0/-1"] {
            assert_eq!(Address::parse(bad), None, "{bad}");
        }
    }

    #[test]
    fn addressing_is_libgpibs() {
        let dev = Address { minor: 0, pad: 23, sad: None };
        assert_eq!(send_setup(0, None, dev), [0x40, 0x3f, 0x37]);
        assert_eq!(receive_setup(0, None, dev), [0x3f, 0x20, 0x57]);
        let sec = Address { sad: Some(2), ..dev };
        assert_eq!(send_setup(1, Some(3), sec), [0x41, 0x63, 0x3f, 0x37, 0x62]);
        assert_eq!(receive_setup(1, Some(3), sec), [0x3f, 0x21, 0x63, 0x57, 0x62]);
        // gpib_address_restrict: 31 is 0.
        assert_eq!(cmd::mla(31), 0x20);
    }

    #[test]
    fn errors_say_what_happened() {
        let a = Address { minor: 2, pad: 23, sad: None };
        let says = |errno, what| errno_error(errno, what, a).to_string();
        assert!(matches!(errno_error(110, "read", a), Error::Timeout));
        assert!(says(19, "read").contains("GPIB board 2: no adapter"));
        assert!(says(22, "write").contains("write refused; is the board online"));
        assert!(says(70, "write").contains("nothing listens at address 23"));
        assert!(says(107, "command").contains("no device on the bus"));
        assert!(says(5, "serial poll").contains("serial poll of address 23 got no answer"));
        assert!(says(5, "read").contains("GPIB board 2: read:"));
        assert_eq!(usec(Duration::ZERO), 1);
        assert_eq!(usec(Duration::from_secs(3)), 3_000_000);
    }
}
