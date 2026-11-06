// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi_tcp.c,
//   Copyright (C) 2013 Martin Ling <martin-sigrok@earth.li>,
//   with commits by Bert Vermeulen, Uwe Hermann, Gerhard Sittig,
//   Aurelien Jacobs, Daniel Elstner
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! SCPI over TCP: raw, where a short read ends a response, and Rigol's
//! variant where every chunk carries a four byte little endian length.
//!
//! Raw has no framing, so two things follow, the first being C's rule too. An answer
//! the stack cuts in two is two answers: the first piece is the response, and the
//! rest is taken for the next. And an answer of exactly the size of the read looks
//! unfinished: here it ends after a read's timeout of silence and comes back whole,
//! where C's blocking read hangs.

use crate::core::{Error, Result};
use crate::scpi::ScpiTransport;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

/// A TCP connection carrying SCPI.
pub struct Tcp {
    stream: Option<TcpStream>,
    host: String,
    port: u16,
    resource: String,
    rigol: bool,
    length: Option<usize>,
    length_buf: Vec<u8>,
    response_length: usize,
    response_read: usize,
}

impl Tcp {
    /// Connects to `host:port`; `rigol` selects the length prefixed
    /// protocol.
    pub fn connect(host: &str, port: u16, rigol: bool) -> Result<Tcp> {
        let stream = open(host, port)?;
        let prefix = if rigol { "tcp-rigol" } else { "tcp-raw" };
        Ok(Tcp {
            stream: Some(stream),
            host: host.to_owned(),
            port,
            resource: format!("{prefix}/{host}/{port}"),
            rigol,
            length: None,
            length_buf: Vec::new(),
            response_length: 0,
            response_read: 0,
        })
    }

    fn stream(&mut self) -> Result<&mut TcpStream> {
        self.stream.as_mut().ok_or_else(|| Error::device("the TCP connection is closed"))
    }

    fn read_some(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let stream = self.stream()?;
        let began = Instant::now();
        let mut last_look = false;
        loop {
            let left = timeout.saturating_sub(began.elapsed());
            stream.set_read_timeout(Some(left.max(Duration::from_millis(1)))).map_err(Error::Io)?;
            match stream.read(buf) {
                // Not silence, which a caller would wait out at full speed.
                Ok(0) if !buf.is_empty() => {
                    return Err(Error::device("the instrument closed the TCP connection"));
                }
                Ok(n) => return Ok(n),
                // A socket's timeout makes a stop and continue end the call with EINTR.
                // It is neither an error nor silence, which would end a raw answer. With
                // the time up it still looks once more, as what came during the stop is
                // there to read: that one's EINTR is silence, or signals could hold it.
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {
                    if last_look {
                        return Ok(0);
                    }
                    last_look = began.elapsed() >= timeout;
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    return Ok(0);
                }
                Err(e) => return Err(Error::Io(e)),
            }
        }
    }
}

fn open(host: &str, port: u16) -> Result<TcpStream> {
    let addr = (host, port)
        .to_socket_addrs()
        .map_err(Error::Io)?
        .next()
        .ok_or_else(|| Error::device(format!("cannot resolve {host}")))?;
    let stream = TcpStream::connect_timeout(&addr, Duration::from_secs(5)).map_err(Error::Io)?;
    stream.set_nodelay(true).map_err(Error::Io)?;
    Ok(stream)
}

impl ScpiTransport for Tcp {
    fn resource(&self) -> &str {
        &self.resource
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.stream()?.write_all(data).map_err(Error::Io)
    }

    fn read_begin(&mut self) -> Result<()> {
        self.length = None;
        self.length_buf.clear();
        self.response_length = 0;
        self.response_read = 0;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        if !self.rigol {
            let n = self.read_some(buf, timeout)?;
            if n == 0 {
                // Silence ends what a full read left open; with nothing yet it is
                // only silence.
                if self.response_read > 0 {
                    self.response_length = self.response_read;
                }
                return Ok(0);
            }
            // A short read ends a response, a full one may continue.
            self.length = Some(0);
            self.response_length = if n < buf.len() { n } else { buf.len() + 1 };
            self.response_read = n;
            return Ok(n);
        }
        if self.length.is_none() {
            let mut tmp = [0u8; 4];
            let want = 4 - self.length_buf.len();
            let n = self.read_some(&mut tmp[..want], timeout)?;
            self.length_buf.extend_from_slice(&tmp[..n]);
            if self.length_buf.len() < 4 {
                return Ok(0);
            }
            let len = u32::from_le_bytes([
                self.length_buf[0],
                self.length_buf[1],
                self.length_buf[2],
                self.length_buf[3],
            ]) as usize;
            self.length = Some(len);
            self.response_length = len;
        }
        if self.response_read >= self.response_length {
            return Err(Error::device("tcp-rigol: more data than announced"));
        }
        let room = buf.len().min(self.response_length - self.response_read);
        let n = self.read_some(&mut buf[..room], timeout)?;
        self.response_read += n;
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.length.is_some() && self.response_read >= self.response_length
    }

    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        let mut chunk = [0u8; 4096];
        self.read_some(&mut chunk, timeout).map(Some)
    }

    fn close(&mut self) -> Result<()> {
        self.stream = None;
        Ok(())
    }

    /// Nothing when it is open: a second connection would be a second client.
    fn reopen(&mut self) -> Result<()> {
        if self.stream.is_none() {
            self.stream = Some(open(&self.host, self.port)?);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    const BOUND: Duration = Duration::from_secs(30);

    fn pair(rigol: bool) -> (Tcp, TcpListener, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let tcp = Tcp::connect("127.0.0.1", port, rigol).unwrap();
        let (peer, _) = listener.accept().unwrap();
        peer.set_read_timeout(Some(BOUND)).unwrap();
        (tcp, listener, peer)
    }

    /// The next connection, or a failure at [`BOUND`], as a blocking accept would
    /// hang a mutant for good.
    fn accept(listener: &TcpListener) -> TcpStream {
        listener.set_nonblocking(true).unwrap();
        let began = std::time::Instant::now();
        loop {
            match listener.accept() {
                Ok((s, _)) => {
                    s.set_nonblocking(false).unwrap();
                    s.set_read_timeout(Some(BOUND)).unwrap();
                    return s;
                }
                Err(_) if began.elapsed() < BOUND => {
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => panic!("no connection: {e}"),
            }
        }
    }

    #[test]
    fn a_close_by_the_instrument_is_no_silence() {
        let (mut tcp, _listener, peer) = pair(false);
        drop(peer);
        let mut buf = [0u8; 8];
        tcp.read_begin().unwrap();
        let e = tcp.read_data(&mut buf, BOUND).unwrap_err().to_string();
        assert!(e.contains("the instrument closed the TCP connection"), "{e}");
    }

    #[test]
    fn it_closes_and_opens_again_on_the_same_port() {
        let (mut tcp, listener, mut peer) = pair(false);
        tcp.close().unwrap();
        let mut rest = Vec::new();
        peer.read_to_end(&mut rest).expect("the close, within the bound");
        assert_eq!(rest, Vec::<u8>::new(), "the peer saw the close");
        let e = tcp.send(b"x").unwrap_err().to_string();
        assert!(e.contains("the TCP connection is closed"), "{e}");
        tcp.close().unwrap();
        tcp.reopen().unwrap();
        let mut again = accept(&listener);
        tcp.send(b"y").unwrap();
        let mut got = [0u8; 1];
        again.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"y");
    }

    #[test]
    fn reopening_what_is_open_makes_no_second_connection() {
        let (mut tcp, listener, mut peer) = pair(false);
        tcp.reopen().unwrap();
        listener.set_nonblocking(true).unwrap();
        assert!(listener.accept().is_err(), "no second client");
        tcp.send(b"y").unwrap();
        let mut got = [0u8; 1];
        peer.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"y", "and the first still works");
    }

    #[test]
    fn a_discard_says_how_much_it_dropped_and_reads_go_on() {
        let (mut tcp, _listener, mut peer) = pair(false);
        peer.write_all(b"stale").unwrap();
        assert_eq!(tcp.discard(BOUND).unwrap(), Some(5));
        assert_eq!(tcp.discard(Duration::from_millis(5)).unwrap(), Some(0), "then silence");
        peer.write_all(b"new").unwrap();
        let mut buf = [0u8; 8];
        tcp.read_begin().unwrap();
        assert_eq!(tcp.read_data(&mut buf, BOUND).unwrap(), 3);
        assert_eq!(&buf[..3], b"new");
    }

    #[test]
    fn silence_is_no_answer_and_ends_a_full_read() {
        let (mut tcp, _listener, mut peer) = pair(false);
        let mut buf = [0u8; 4];
        tcp.read_begin().unwrap();
        assert_eq!(tcp.read_data(&mut buf, Duration::from_millis(1)).unwrap(), 0);
        assert!(!tcp.read_complete(), "nothing yet is no answer");
        peer.write_all(b"abcd").unwrap();
        assert_eq!(tcp.read_data(&mut buf, BOUND).unwrap(), 4);
        assert!(!tcp.read_complete(), "a full read may go on");
        assert_eq!(tcp.read_data(&mut buf, Duration::from_millis(1)).unwrap(), 0);
        assert!(tcp.read_complete(), "silence ends it");
    }

    #[test]
    fn rigol_framing_says_when_it_is_complete() {
        let (mut tcp, _listener, mut peer) = pair(true);
        peer.write_all(&[3, 0, 0, 0]).unwrap();
        peer.write_all(b"abc").unwrap();
        tcp.read_begin().unwrap();
        let mut out = Vec::new();
        let mut buf = [0u8; 8];
        while !tcp.read_complete() {
            let n = tcp.read_data(&mut buf, BOUND).unwrap();
            assert!(n > 0, "a read of BOUND came back empty");
            out.extend_from_slice(&buf[..n]);
        }
        assert_eq!(out, b"abc");
    }
}
