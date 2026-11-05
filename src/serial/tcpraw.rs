// Counterpart in sigrok:
//   libsigrok/src/serial_tcpraw.c,
//   libsigrok/src/tcp.c,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Marc Schink, Charlie Birks
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Raw bytes over TCP as a serial port: `tcp-raw/HOST/PORT`, the
//! counterpart of libsigrok's `serial_tcpraw.c`. Line settings mean nothing
//! here and are taken without complaint, as C's `std_dummy_set_params`.
//!
//! What differs from C:
//!
//! - A read keeps to its timeout. C's `sr_tcp_read_bytes` is a plain `recv`
//!   (`tcp.c:362`), so a blocking read's timeout never runs while the peer
//!   is connected and silent: a run on a loopback board that doesn't answer
//!   waits for as long as it is left.
//! - A peer that has closed is [`Error::Disconnected`] at once. C's `recv`
//!   gives 0, and a read then waits out its timeout and comes up short.
//! - The port is a number, where C's `getaddrinfo` takes a service name too.
//! - A connect gives up after [`CONNECT_TIMEOUT`], where C's has the
//!   system's.
//! - [`Port::purge`] drops what has arrived. C has no flush for this
//!   transport, and `serial_open` ignores that.

use super::{Params, Port};
use crate::core::{Error, Result};
use std::io::{ErrorKind, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

/// `SER_TCPRAW_CONN_PREFIX` and its separator.
const PREFIX: &str = "tcp-raw/";
/// How long a connect may take.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// Whether a port's name means this transport: `ser_name_is_tcpraw`.
pub fn is_tcpraw_name(name: &str) -> bool {
    name.starts_with(PREFIX)
}

/// The host and port of a name, `ser_tcpraw_parse_conn_spec`. Fields after
/// them are ignored with a warning.
fn parse_name(name: &str) -> Option<(&str, u16)> {
    let mut fields = name.strip_prefix(PREFIX)?.split('/');
    let host = fields.next().filter(|h| !h.is_empty())?;
    let port = fields.next().filter(|p| !p.is_empty())?;
    if fields.next().is_some() {
        log::warn!("Ignoring excess parameters in {name}.");
    }
    Some((host, port.parse().ok()?))
}

/// A TCP connection carrying a serial protocol.
#[derive(Debug)]
pub struct TcpRaw {
    name: String,
    stream: TcpStream,
}

impl TcpRaw {
    /// Connects to what `name` says. A commands-and-answers protocol sends
    /// little and waits, so Nagle is off.
    ///
    /// # Errors
    ///
    /// A name that isn't `tcp-raw/HOST/PORT`, or a failed connect.
    pub fn open(name: &str) -> Result<TcpRaw> {
        let (host, port) =
            parse_name(name).ok_or_else(|| Error::Arg(format!("Invalid TCP port '{name}'.")))?;
        let failed = |why: &dyn std::fmt::Display| {
            let why = why.to_string();
            let why = why.split(" (os error").next().unwrap_or_default();
            Error::Io(std::io::Error::other(format!("Failed to connect to {host}:{port}: {why}.")))
        };
        let addrs = (host, port).to_socket_addrs().map_err(|e| failed(&e))?;
        let mut last = None;
        for addr in addrs {
            match TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT) {
                Ok(stream) => {
                    stream.set_nodelay(true)?;
                    return Ok(TcpRaw { name: name.to_owned(), stream });
                }
                Err(e) => last = Some(e),
            }
        }
        Err(match last {
            Some(e) => failed(&e),
            None => failed(&"no address"),
        })
    }
}

impl Port for TcpRaw {
    fn name(&self) -> &str {
        &self.name
    }

    fn set_params(&mut self, _params: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _rts: Option<bool>, _dtr: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        // A zero duration is refused here, and would mean no timeout anyway.
        self.stream.set_read_timeout(Some(timeout.max(Duration::from_millis(1))))?;
        match self.stream.read(buf) {
            Ok(0) => Err(Error::Disconnected),
            Ok(n) => Ok(n),
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => Ok(0),
            Err(e) if e.kind() == ErrorKind::Interrupted => Ok(0),
            Err(e) => Err(Error::Io(e)),
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.stream.write_all(data).map_err(Error::Io)
    }

    fn purge(&mut self) -> Result<()> {
        self.stream.set_nonblocking(true)?;
        let mut buf = [0u8; 256];
        let result = loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break Err(Error::Disconnected),
                Ok(_) => {}
                Err(e) if e.kind() == ErrorKind::WouldBlock => break Ok(()),
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => break Err(Error::Io(e)),
            }
        };
        self.stream.set_nonblocking(false)?;
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::read_blocking;
    use std::net::TcpListener;
    use std::sync::mpsc;
    use std::time::Instant;

    /// A peer that runs `talk` on its first connection, and the name to
    /// reach it by. The thread ends with its `talk`.
    fn peer(
        talk: impl FnOnce(TcpStream) + Send + 'static,
    ) -> (String, std::thread::JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        let name =
            format!("tcp-raw/127.0.0.1/{}", listener.local_addr().expect("an address").port());
        let handle = std::thread::spawn(move || {
            let (stream, _) = listener.accept().expect("a connection");
            talk(stream);
        });
        (name, handle)
    }

    #[test]
    fn a_name_is_this_transport_only_with_its_separator() {
        assert!(is_tcpraw_name("tcp-raw/host/1"));
        assert!(is_tcpraw_name("tcp-raw/"));
        assert!(!is_tcpraw_name("tcp-raw"));
        assert!(!is_tcpraw_name("tcp-rawer/host/1"));
        assert!(!is_tcpraw_name("/dev/ttyACM0"));
        assert!(!is_tcpraw_name(""));
    }

    #[test]
    fn a_name_needs_a_host_and_a_port_and_may_have_more() {
        assert_eq!(parse_name("tcp-raw/host/17494"), Some(("host", 17494)));
        assert_eq!(parse_name("tcp-raw/host/17494/more/still"), Some(("host", 17494)));
        for bad in
            ["tcp-raw/", "tcp-raw/host", "tcp-raw/host/", "tcp-raw//1", "tcp-raw/h/x", "x/h/1"]
        {
            assert_eq!(parse_name(bad), None, "{bad}");
        }
    }

    #[test]
    fn a_bad_name_or_a_refused_connect_is_an_error() {
        assert!(TcpRaw::open("tcp-raw/127.0.0.1").is_err());
        // A port just closed: its listener is gone.
        let listener = TcpListener::bind("127.0.0.1:0").expect("a listener");
        let port = listener.local_addr().expect("an address").port();
        drop(listener);
        assert!(TcpRaw::open(&format!("tcp-raw/127.0.0.1/{port}")).is_err());
    }

    #[test]
    fn bytes_go_both_ways_and_settings_are_taken() {
        let (name, peer) = peer(|mut s| {
            let mut got = [0u8; 2];
            s.read_exact(&mut got).expect("the request");
            assert_eq!(got, [0x10, 0x20]);
            s.write_all(&[1, 2, 3]).expect("the answer");
            // Until the test has read it.
            let _ = s.read(&mut got);
        });
        let mut port = TcpRaw::open(&name).expect("a connection");
        assert_eq!(port.name(), name);
        port.set_params(Params::parse("115200/7e2").expect("params")).expect("taken");
        port.set_handshake(Some(true), Some(false)).expect("taken");
        port.write(&[0x10, 0x20]).expect("sent");
        let mut buf = [0u8; 3];
        assert_eq!(read_blocking(&mut port, &mut buf, Duration::from_secs(20)).expect("read"), 3);
        assert_eq!(buf, [1, 2, 3]);
        drop(port);
        peer.join().expect("the peer");
    }

    #[test]
    fn a_silent_peer_times_out_with_what_came() {
        let (done, wait) = mpsc::channel::<()>();
        let (name, peer) = peer(move |mut s| {
            s.write_all(&[7]).expect("a byte");
            let _ = wait.recv();
        });
        let mut port = TcpRaw::open(&name).expect("a connection");
        let mut buf = [0u8; 2];
        let began = Instant::now();
        assert_eq!(
            read_blocking(&mut port, &mut buf, Duration::from_millis(100)).expect("read"),
            1
        );
        assert!(began.elapsed() >= Duration::from_millis(100));
        assert_eq!(buf[0], 7);
        done.send(()).expect("the peer waits");
        peer.join().expect("the peer");
    }

    #[test]
    fn a_read_with_no_time_left_does_not_wait_for_ever() {
        let (done, wait) = mpsc::channel::<()>();
        let (name, peer) = peer(move |_s| {
            let _ = wait.recv();
        });
        let mut port = TcpRaw::open(&name).expect("a connection");
        let mut buf = [0u8; 1];
        assert_eq!(port.read(&mut buf, Duration::ZERO).expect("a read"), 0);
        assert_eq!(port.read(&mut [], Duration::from_secs(60)).expect("no room"), 0);
        done.send(()).expect("the peer waits");
        peer.join().expect("the peer");
    }

    #[test]
    fn a_peer_gone_is_a_disconnect() {
        let (name, peer) = peer(drop);
        let mut port = TcpRaw::open(&name).expect("a connection");
        peer.join().expect("the peer");
        let mut buf = [0u8; 1];
        assert!(matches!(port.read(&mut buf, Duration::from_secs(20)), Err(Error::Disconnected)));
    }

    #[test]
    fn a_purge_drops_what_came_and_leaves_what_comes_later() {
        let (go, wait) = mpsc::channel::<()>();
        let (arrived, seen) = mpsc::channel::<()>();
        let (name, peer) = peer(move |mut s| {
            s.write_all(&[1, 2, 3]).expect("first");
            arrived.send(()).expect("the test waits");
            wait.recv().expect("the purge");
            s.write_all(&[9]).expect("second");
            let mut buf = [0u8; 1];
            let _ = s.read(&mut buf);
        });
        let mut port = TcpRaw::open(&name).expect("a connection");
        seen.recv().expect("the peer wrote");
        // It left the peer; wait for it here, with a bound, by peeking.
        let began = Instant::now();
        let mut look = [0u8; 3];
        while port.stream.peek(&mut look).expect("a peek") < 3 {
            assert!(began.elapsed() < Duration::from_secs(20), "the bytes never came");
        }
        port.purge().expect("a purge");
        // Blocking again: a read keeps to its timeout, and doesn't just return.
        let mut buf = [0u8; 1];
        // A signal may end a read early, so one of a few has to take its time.
        let waited = (0..20).any(|_| {
            let began = Instant::now();
            assert_eq!(port.read(&mut buf, Duration::from_millis(100)).expect("a read"), 0);
            began.elapsed() >= Duration::from_millis(100)
        });
        assert!(waited, "no read waited: still nonblocking");
        go.send(()).expect("the peer waits");
        assert_eq!(read_blocking(&mut port, &mut buf, Duration::from_secs(20)).expect("read"), 1);
        assert_eq!(buf, [9]);
        drop(port);
        peer.join().expect("the peer");
    }

    #[test]
    fn a_purge_on_a_gone_peer_says_so() {
        let (name, peer) = peer(drop);
        let mut port = TcpRaw::open(&name).expect("a connection");
        peer.join().expect("the peer");
        let began = Instant::now();
        loop {
            match port.purge() {
                Err(Error::Disconnected) => break,
                Ok(()) => assert!(began.elapsed() < Duration::from_secs(20), "no end seen"),
                Err(e) => panic!("{e}"),
            }
        }
    }
}
