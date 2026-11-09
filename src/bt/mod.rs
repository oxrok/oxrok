// Counterpart in sigrok:
//   libsigrok/src/bt/bt_bluez.c,
//   Copyright (C) 2018-2019 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann, Andreas Sandberg
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Bluetooth LE, as libsigrok's `bt/bt_bluez.c` does it on Linux: an L2CAP
//! socket on the ATT fixed channel (CID 4) to a public address, with the link
//! left to the kernel. No bluetoothd and no D-Bus: the kernel scans for the
//! address, connects and keeps the link while a socket holds it. The ATT
//! client is the few PDUs a notifying characteristic needs: a Write Request to
//! a CCCD, Write Requests to a value, Handle Value Notifications and
//! Indications (each confirmed), and what libsigrok notes and passes over
//! (Write Responses, Error Responses, an MTU request it doesn't answer).
//!
//! Where it goes its own way:
//!
//! - A connection has a bound: `connect` gives up after the time it is given.
//!   C's blocking `connect()` waits for the kernel's 40 s, and when that ends
//!   with the socket still connecting it polls for writability until its own
//!   20 s deadline, but never leaves that loop on success, so a connection
//!   that completes late is reported as a timeout (`bt_bluez.c:729-748`).
//! - A peer that has gone is an error on the next read or write, where C's
//!   `sr_bt_check_notify` returns -2 and its callers poll on.
//!
//! [`Bearer`] is the socket's two calls, so a driver's tests take a fake.

use std::io;
use std::time::Duration;

/// The Bluetooth LE address the user wrote, as libsigrok reads it
/// (`sr_bt_mac_text_to_bytes`): six bytes in hex, each one or two digits,
/// separated by `:` or `-`, or two digits each and nothing between them.
/// The error is C's message, which names the part where it stopped, and is
/// logged here as C logs it.
///
/// # Errors
///
/// Fewer or more than six bytes, or something else in between.
pub fn parse_mac(text: &str) -> std::result::Result<[u8; 6], String> {
    let b = text.as_bytes();
    let mut out = [0u8; 6];
    let mut got = 0;
    let mut at = 0;
    let mut part = 0;
    while got < 6 && at < b.len() {
        part = at;
        // Up to two hex digits, as `strtol` of a two byte buffer would read.
        let mut v = 0u8;
        let mut digits = 0;
        while digits < 2 && at < b.len() && b[at].is_ascii_hexdigit() {
            v = v * 16 + (b[at] as char).to_digit(16).expect("a hex digit") as u8;
            at += 1;
            digits += 1;
        }
        if digits == 0 {
            break;
        }
        if at < b.len() && b[at] != b':' && b[at] != b'-' && digits < 2 {
            break;
        }
        out[got] = v;
        got += 1;
        if at < b.len() && (b[at] == b':' || b[at] == b'-') {
            at += 1;
        }
    }
    let error = if got < 6 {
        format!("Failed to parse MAC, too few bytes in '{}'", &text[part..])
    } else if !text[at..].trim_start().is_empty() {
        format!("Failed to parse MAC, excess data in '{}'", &text[part..])
    } else {
        return Ok(out);
    };
    // Logged here, as C logs it in its bt layer.
    log::error!("{error}");
    Err(error)
}

/// An address as `BlueZ`'s `ba2str` writes it.
pub fn mac_text(addr: [u8; 6]) -> String {
    let parts: Vec<String> = addr.iter().map(|b| format!("{b:02X}")).collect();
    parts.join(":")
}

/// The ATT bearer: one PDU per call either way.
pub trait Bearer: Send {
    /// Sends one PDU.
    ///
    /// # Errors
    ///
    /// The link has gone, or the socket refused it.
    fn send(&mut self, pdu: &[u8]) -> io::Result<()>;

    /// The next PDU, waiting up to `wait` for one; `None` when none came.
    ///
    /// # Errors
    ///
    /// The link has gone.
    fn recv(&mut self, wait: Duration) -> io::Result<Option<Vec<u8>>>;
}

/// ATT opcodes (`bt_bluez.c:110-133`).
pub mod att {
    /// Error Response.
    pub const ERROR_RESP: u8 = 0x01;
    /// Exchange MTU Request.
    pub const EXCHANGE_MTU_REQ: u8 = 0x02;
    /// Exchange MTU Response.
    pub const EXCHANGE_MTU_RESP: u8 = 0x03;
    /// Write Request.
    pub const WRITE_REQ: u8 = 0x12;
    /// Write Response.
    pub const WRITE_RESP: u8 = 0x13;
    /// Handle Value Notification.
    pub const NOTIFICATION: u8 = 0x1b;
    /// Handle Value Indication.
    pub const INDICATION: u8 = 0x1d;
    /// Handle Value Confirmation.
    pub const CONFIRMATION: u8 = 0x1e;
}

/// What came of one PDU, as `sr_bt_check_notify` sorts them.
#[derive(Debug, PartialEq, Eq)]
pub enum Incoming {
    /// A notification or an indication of the value read: its bytes.
    Data(Vec<u8>),
    /// A notification or an indication of another handle (C's -4).
    OtherHandle(u16),
    /// Noted and passed over: a write response, an error response, an MTU
    /// request.
    Noted,
    /// An opcode C does not handle (C's -3).
    Unhandled(u8),
}

/// A characteristic pair of a BLE UART: the value notified, the value written,
/// and the CCCD that turns the notifications on (`sr_bt_config_notify`).
#[derive(Clone, Copy, Debug)]
pub struct Notify {
    /// The handle notified.
    pub read_handle: u16,
    /// The handle written.
    pub write_handle: u16,
    /// The CCCD's handle.
    pub cccd_handle: u16,
    /// What is written to the CCCD.
    pub cccd_value: u16,
}

impl Notify {
    /// `sr_bt_start_notify`: writes the CCCD, and does not wait for the answer.
    ///
    /// # Errors
    ///
    /// The bearer's.
    pub fn start(&self, bearer: &mut dyn Bearer) -> io::Result<()> {
        log::debug!("BLE start notify");
        bearer.send(&write_req(self.cccd_handle, &self.cccd_value.to_le_bytes()))
    }

    /// `sr_bt_write` to a BLE UART: a Write Request of `data` to the written
    /// handle, not waited for.
    ///
    /// # Errors
    ///
    /// The bearer's.
    pub fn write(&self, bearer: &mut dyn Bearer, data: &[u8]) -> io::Result<()> {
        bearer.send(&write_req(self.write_handle, data))
    }

    /// One PDU sorted as `sr_bt_check_notify` sorts it, an indication
    /// confirmed; `None` when none came within `wait`.
    ///
    /// # Errors
    ///
    /// The bearer's: the link has gone.
    pub fn check(&self, bearer: &mut dyn Bearer, wait: Duration) -> io::Result<Option<Incoming>> {
        let Some(pdu) = bearer.recv(wait)? else { return Ok(None) };
        log::trace!("check notify, read success, length {}", pdu.len());
        let Some((&op, rest)) = pdu.split_first() else {
            return Ok(Some(Incoming::Unhandled(0)));
        };
        let handle = (rest.len() >= 2).then(|| u16::from_le_bytes([rest[0], rest[1]]));
        let payload = if rest.len() > 2 { rest[2..].to_vec() } else { Vec::new() };
        Ok(Some(match op {
            att::EXCHANGE_MTU_REQ => {
                match handle {
                    None => log::debug!("MTU exchange request, invalid (size)"),
                    Some(mtu) => {
                        log::debug!("MTU exchange request, peripheral value {mtu}");
                        log::warn!("Unhandled BLE MTU exchange request.");
                    }
                }
                Incoming::Noted
            }
            att::ERROR_RESP => {
                log::debug!("error response, not handled here");
                Incoming::Noted
            }
            att::WRITE_RESP => {
                log::debug!("write response, note taken");
                Incoming::Noted
            }
            att::INDICATION | att::NOTIFICATION => {
                if op == att::INDICATION {
                    log::debug!("handle indication, data len {}", payload.len());
                    bearer.send(&[att::CONFIRMATION])?;
                } else {
                    log::debug!("handle notification, data len {}", payload.len());
                }
                match handle {
                    Some(h) if h == self.read_handle => Incoming::Data(payload),
                    h => Incoming::OtherHandle(h.unwrap_or(0)),
                }
            }
            other => {
                log::debug!("unhandled type 0x{other:02x}, len {}", rest.len());
                Incoming::Unhandled(other)
            }
        }))
    }
}

/// A Write Request PDU.
pub fn write_req(handle: u16, data: &[u8]) -> Vec<u8> {
    let mut pdu = vec![att::WRITE_REQ];
    pdu.extend_from_slice(&handle.to_le_bytes());
    pdu.extend_from_slice(data);
    pdu
}

/// `sr_bt_connect_ble`: an L2CAP socket on the ATT channel to `addr`, a
/// public address, through whichever controller the kernel routes it by.
///
/// # Errors
///
/// No connection within `timeout`, or no Bluetooth here.
pub fn connect(addr: [u8; 6], timeout: Duration) -> io::Result<Box<dyn Bearer>> {
    log::debug!("BLE connect, remote addr {}", mac_text(addr));
    #[cfg(target_os = "linux")]
    {
        linux::L2cap::connect(addr, timeout).map(|b| Box::new(b) as Box<dyn Bearer>)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = timeout;
        Err(io::Error::new(io::ErrorKind::Unsupported, "Bluetooth LE is supported on Linux only"))
    }
}

#[cfg(target_os = "linux")]
mod linux {
    //! The socket, through socket2: one `unsafe` block, for the address.

    use std::io::{self, Read as _};
    use std::os::fd::AsRawFd;
    use std::time::Duration;

    use socket2::{Domain, Protocol, SockAddr, Socket, Type};

    use super::Bearer;

    const AF_BLUETOOTH: i32 = 31;
    const SOCK_SEQPACKET: i32 = 5;
    const BTPROTO_L2CAP: i32 = 0;
    /// The ATT fixed channel.
    const CID_ATT: u16 = 4;
    const BDADDR_LE_PUBLIC: u8 = 1;
    /// The most one PDU can be on the socket, as C reads.
    const READ: usize = 1024;

    /// A `sockaddr_l2`: family, PSM, address (little endian), CID, address type.
    fn sockaddr(addr: [u8; 6]) -> io::Result<SockAddr> {
        let mut raw = [0u8; 14];
        raw[..2].copy_from_slice(&(AF_BLUETOOTH as u16).to_ne_bytes());
        // l2_psm: 0, as C has it.
        let mut wire = addr;
        wire.reverse();
        raw[4..10].copy_from_slice(&wire);
        raw[10..12].copy_from_slice(&CID_ATT.to_le_bytes());
        raw[12] = BDADDR_LE_PUBLIC;
        // SAFETY: the storage is zeroed and as large as any address; the 14
        // bytes written are a whole `struct sockaddr_l2` (family, psm, bdaddr,
        // cid, bdaddr_type, as <bluetooth/l2cap.h> lays it out, with no
        // padding), the family says AF_BLUETOOTH, and the length is set to them.
        let ((), sa) = unsafe {
            SockAddr::try_init(|storage, len| {
                std::ptr::copy_nonoverlapping(raw.as_ptr(), storage.cast::<u8>(), raw.len());
                *len = raw.len() as _;
                Ok(())
            })
        }?;
        Ok(sa)
    }

    /// The socket.
    pub struct L2cap {
        socket: Socket,
    }

    impl L2cap {
        /// Binds to any controller and connects, with a bound.
        pub fn connect(addr: [u8; 6], timeout: Duration) -> io::Result<L2cap> {
            let socket = Socket::new(
                Domain::from(AF_BLUETOOTH),
                Type::from(SOCK_SEQPACKET),
                Some(Protocol::from(BTPROTO_L2CAP)),
            )?;
            socket.bind(&sockaddr([0; 6])?)?;
            socket.connect_timeout(&sockaddr(addr)?, timeout)?;
            socket.set_nonblocking(false)?;
            log::trace!("connected, fd {}", socket.as_raw_fd());
            Ok(L2cap { socket })
        }
    }

    impl Bearer for L2cap {
        fn send(&mut self, pdu: &[u8]) -> io::Result<()> {
            let n = self.socket.send(pdu)?;
            if n != pdu.len() {
                return Err(io::Error::new(io::ErrorKind::WriteZero, "a short write"));
            }
            Ok(())
        }

        fn recv(&mut self, wait: Duration) -> io::Result<Option<Vec<u8>>> {
            // A read timeout of zero means none at all to the socket.
            self.socket.set_read_timeout(Some(wait.max(Duration::from_millis(1))))?;
            let mut buf = [0u8; READ];
            match (&self.socket).read(&mut buf) {
                // A SEQPACKET socket's 0 is the peer gone.
                Ok(0) => Err(io::Error::new(io::ErrorKind::ConnectionReset, "the link has gone")),
                Ok(n) => Ok(Some(buf[..n].to_vec())),
                Err(e)
                    if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) =>
                {
                    Ok(None)
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => Ok(None),
                Err(e) => Err(e),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addresses_read_as_libsigrok_reads_them() {
        let a = [0xd0, 0x5f, 0xb8, 0x4d, 0x00, 0x01];
        for text in [
            "D0:5F:B8:4D:00:01",
            "D0-5F-B8-4D-00-01",
            "D05FB84D0001",
            "d0-5f-b8-4d-0-1",
            "D05FB84D0001  ",
        ] {
            assert_eq!(parse_mac(text), Ok(a), "{text}");
        }
        assert_eq!(
            parse_mac("D0-5F-B8-4D-00"),
            Err("Failed to parse MAC, too few bytes in '00'".to_owned())
        );
        assert_eq!(
            parse_mac("D0-5F-B8-4D-00-01-02"),
            Err("Failed to parse MAC, excess data in '01-02'".to_owned())
        );
        assert!(parse_mac("").is_err());
        assert!(parse_mac("D0-5G").is_err());
        assert_eq!(mac_text(a), "D0:5F:B8:4D:00:01");
    }

    struct Fake {
        incoming: Vec<Vec<u8>>,
        sent: Vec<Vec<u8>>,
    }

    impl Bearer for Fake {
        fn send(&mut self, pdu: &[u8]) -> io::Result<()> {
            self.sent.push(pdu.to_vec());
            Ok(())
        }
        fn recv(&mut self, _wait: Duration) -> io::Result<Option<Vec<u8>>> {
            Ok((!self.incoming.is_empty()).then(|| self.incoming.remove(0)))
        }
    }

    #[test]
    fn pdus_sort_as_check_notify_sorts_them() {
        let n = Notify { read_handle: 0x15, write_handle: 0x12, cccd_handle: 0x16, cccd_value: 1 };
        let mut f = Fake {
            incoming: vec![
                vec![0x1b, 0x15, 0x00, 1, 2],
                vec![0x1d, 0x15, 0x00, 3],
                vec![0x1b, 0x99, 0x00, 4],
                vec![0x13],
                vec![0x02, 0x17, 0x00],
                vec![0x55],
            ],
            sent: Vec::new(),
        };
        n.start(&mut f).unwrap();
        n.write(&mut f, &[0, 1]).unwrap();
        let w = Duration::ZERO;
        assert_eq!(n.check(&mut f, w).unwrap(), Some(Incoming::Data(vec![1, 2])));
        assert_eq!(n.check(&mut f, w).unwrap(), Some(Incoming::Data(vec![3])));
        assert_eq!(n.check(&mut f, w).unwrap(), Some(Incoming::OtherHandle(0x99)));
        assert_eq!(n.check(&mut f, w).unwrap(), Some(Incoming::Noted));
        assert_eq!(n.check(&mut f, w).unwrap(), Some(Incoming::Noted));
        assert_eq!(n.check(&mut f, w).unwrap(), Some(Incoming::Unhandled(0x55)));
        assert_eq!(n.check(&mut f, w).unwrap(), None);
        assert_eq!(
            f.sent,
            [vec![0x12, 0x16, 0x00, 1, 0], vec![0x12, 0x12, 0x00, 0, 1], vec![0x1e]]
        );
    }
}
