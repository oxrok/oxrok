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

use crate::core::{Error, Result};
use crate::scpi::ScpiTransport;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

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
        stream.set_read_timeout(Some(timeout.max(Duration::from_millis(1)))).map_err(Error::Io)?;
        match stream.read(buf) {
            // Not silence, which a caller would wait out at full speed.
            Ok(0) if !buf.is_empty() => {
                Err(Error::device("the instrument closed the TCP connection"))
            }
            Ok(n) => Ok(n),
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(Error::Io(e)),
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
    /// What has come and nobody asked for goes.
    fn purge(&mut self) -> Result<()> {
        let stream = self.stream()?;
        stream.set_nonblocking(true).map_err(Error::Io)?;
        let mut chunk = [0u8; 4096];
        // Not an error here: the next read says if the line is gone.
        while matches!(stream.read(&mut chunk), Ok(n) if n > 0) {}
        stream.set_nonblocking(false).map_err(Error::Io)
    }

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

    fn close(&mut self) -> Result<()> {
        self.stream = None;
        Ok(())
    }

    fn reopen(&mut self) -> Result<()> {
        self.stream = Some(open(&self.host, self.port)?);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    fn pair(rigol: bool) -> (Tcp, TcpListener, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let tcp = Tcp::connect("127.0.0.1", port, rigol).unwrap();
        let (peer, _) = listener.accept().unwrap();
        (tcp, listener, peer)
    }

    #[test]
    fn a_close_by_the_instrument_is_no_silence() {
        let (mut tcp, _listener, peer) = pair(false);
        drop(peer);
        let mut buf = [0u8; 8];
        tcp.read_begin().unwrap();
        let e = tcp.read_data(&mut buf, Duration::from_secs(30)).unwrap_err().to_string();
        assert!(e.contains("the instrument closed the TCP connection"), "{e}");
    }

    #[test]
    fn it_closes_and_opens_again_on_the_same_port() {
        let (mut tcp, listener, mut peer) = pair(false);
        tcp.close().unwrap();
        let mut rest = Vec::new();
        peer.read_to_end(&mut rest).unwrap();
        assert!(rest.is_empty(), "the peer saw the close");
        let e = tcp.send(b"x").unwrap_err().to_string();
        assert!(e.contains("the TCP connection is closed"), "{e}");
        tcp.reopen().unwrap();
        let (mut again, _) = listener.accept().unwrap();
        tcp.send(b"y").unwrap();
        let mut got = [0u8; 1];
        again.read_exact(&mut got).unwrap();
        assert_eq!(&got, b"y");
    }

    #[test]
    fn a_purge_drops_what_has_come_and_reads_block_again() {
        let (mut tcp, _listener, mut peer) = pair(false);
        peer.write_all(b"stale").unwrap();
        // Over loopback, by the time a byte of a later write has come.
        peer.flush().unwrap();
        let mut waited = 0;
        loop {
            let mut buf = [0u8; 1];
            tcp.stream().unwrap().set_nonblocking(true).unwrap();
            let seen = tcp.stream().unwrap().peek(&mut buf).is_ok();
            tcp.stream().unwrap().set_nonblocking(false).unwrap();
            if seen || waited > 5000 {
                break;
            }
            waited += 1;
            std::thread::sleep(Duration::from_millis(1));
        }
        tcp.purge().unwrap();
        let mut buf = [0u8; 8];
        tcp.read_begin().unwrap();
        assert_eq!(tcp.read_data(&mut buf, Duration::from_millis(20)).unwrap(), 0, "a timeout");
        peer.write_all(b"new").unwrap();
        assert_eq!(tcp.read_data(&mut buf, Duration::from_secs(30)).unwrap(), 3);
        assert_eq!(&buf[..3], b"new");
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
            let n = tcp.read_data(&mut buf, Duration::from_secs(30)).unwrap();
            out.extend_from_slice(&buf[..n]);
        }
        assert_eq!(out, b"abc");
    }
}
