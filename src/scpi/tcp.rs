// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi_tcp.c,
//   Copyright (C) 2013 Martin Ling <martin-sigrok@earth.li>,
//   with commits by Bert Vermeulen, Uwe Hermann, Gerhard Sittig,
//   Aurelien Jacobs, Daniel Elstner
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! SCPI over TCP: raw, and Rigol's variant where every chunk carries a four byte little
//! endian length.
//!
//! Raw has no framing but the answer's own. A text answer ends at a line feed that
//! ends a read, or after a read's whole timeout of silence with something read; an
//! IEEE 488.2 block (`#`, a digit, that many digits of length) ends at its length,
//! and a line end that follows it is taken off the next answer. A short read ends
//! nothing. C's rule is that a short read ends an answer (`scpi_tcp.c`), so an answer
//! the stack or the instrument cuts in two is two answers there, and every one after
//! it stands one behind for good. An instrument that ends no text with a line feed
//! costs a read's timeout per answer here.

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
    /// Raw: what this answer is, as far as its first bytes say.
    shape: Shape,
    /// Raw: its first bytes, until they say.
    head: Vec<u8>,
    /// Raw: the answer has ended.
    ended: bool,
    /// Raw: a block just ended, and its line end may come at the next answer's start.
    after_block: bool,
    /// Raw: what a read brought past the end of an answer, for the next one.
    pending: Vec<u8>,
}

/// What a raw answer is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shape {
    /// Not known yet: it starts as a block may.
    Unsure,
    /// Text, which a line feed ends.
    Text,
    /// A block of this many bytes in all, header included.
    Block(usize),
}

/// What the first bytes of a raw answer make it.
fn shape(head: &[u8]) -> Shape {
    let Some(&first) = head.first() else { return Shape::Unsure };
    if first != b'#' {
        return Shape::Text;
    }
    let Some(&digit) = head.get(1) else { return Shape::Unsure };
    if !(b'1'..=b'9').contains(&digit) {
        return Shape::Text;
    }
    let digits = usize::from(digit - b'0');
    let Some(length) = head.get(2..2 + digits) else { return Shape::Unsure };
    match std::str::from_utf8(length).ok().and_then(|s| s.parse::<usize>().ok()) {
        Some(n) => Shape::Block(2 + digits + n),
        None => Shape::Text,
    }
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
            shape: Shape::Unsure,
            head: Vec::new(),
            ended: false,
            after_block: false,
            pending: Vec::new(),
        })
    }

    /// A raw read: see the module's documentation.
    fn read_raw(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut n = if self.pending.is_empty() {
            self.read_some(buf, timeout)?
        } else {
            let n = self.pending.len().min(buf.len());
            buf[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            n
        };
        if n == 0 {
            // Silence ends what has begun; with nothing yet it is only silence.
            self.ended = self.response_read > 0;
            return Ok(0);
        }
        // A driver that reads on past an answer's end has framing of its own, as one
        // that reads a block's line end itself: it gets what comes.
        if self.ended {
            self.after_block = false;
            return Ok(n);
        }
        if std::mem::take(&mut self.after_block) && self.response_read == 0 {
            let skip = buf[..n].iter().take(2).take_while(|b| matches!(b, b'\r' | b'\n')).count();
            buf.copy_within(skip..n, 0);
            n -= skip;
            if n == 0 {
                return Ok(0);
            }
        }
        if self.shape == Shape::Unsure {
            let want = 11usize.saturating_sub(self.head.len()).min(n);
            self.head.extend_from_slice(&buf[..want]);
            self.shape = shape(&self.head);
        }
        // Where this answer ends in what came, if it does.
        let end = match self.shape {
            Shape::Block(total) => {
                (self.response_read + n >= total).then(|| total - self.response_read)
            }
            Shape::Text => buf[..n].iter().position(|b| *b == b'\n').map(|at| at + 1),
            Shape::Unsure => None,
        };
        if let Some(end) = end {
            // The next answer's start stays for it, ahead of what is still to come.
            let mut rest = buf[end..n].to_vec();
            rest.append(&mut self.pending);
            self.pending = rest;
            n = end;
            self.ended = true;
            self.after_block = matches!(self.shape, Shape::Block(_));
        }
        self.response_read += n;
        Ok(n)
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
        self.shape = Shape::Unsure;
        self.head.clear();
        self.ended = false;
        Ok(())
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        if !self.rigol {
            return self.read_raw(buf, timeout);
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
        if !self.rigol {
            return self.ended;
        }
        self.length.is_some() && self.response_read >= self.response_length
    }

    fn purge(&mut self) -> Result<()> {
        self.pending.clear();
        Ok(())
    }

    fn discard(&mut self, timeout: Duration) -> Result<Option<usize>> {
        let held = std::mem::take(&mut self.pending).len();
        if held > 0 {
            return Ok(Some(held));
        }
        let mut chunk = [0u8; 4096];
        self.read_some(&mut chunk, timeout).map(Some)
    }

    fn close(&mut self) -> Result<()> {
        self.stream = None;
        self.pending.clear();
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
    fn silence_is_no_answer_and_ends_one_begun() {
        let (mut tcp, _listener, mut peer) = pair(false);
        let mut buf = [0u8; 4];
        tcp.read_begin().unwrap();
        assert_eq!(tcp.read_data(&mut buf, Duration::from_millis(1)).unwrap(), 0);
        assert!(!tcp.read_complete(), "nothing yet is no answer");
        peer.write_all(b"ab").unwrap();
        assert_eq!(tcp.read_data(&mut buf, BOUND).unwrap(), 2);
        assert!(!tcp.read_complete(), "a short read ends nothing");
        assert_eq!(tcp.read_data(&mut buf, Duration::from_millis(1)).unwrap(), 0);
        assert!(tcp.read_complete(), "silence ends it");
    }

    /// Reads one raw answer as `Scpi::get_data` does: until complete, a short wait each.
    fn answer(tcp: &mut Tcp, wait: Duration) -> Vec<u8> {
        tcp.read_begin().unwrap();
        let mut out = Vec::new();
        let mut buf = [0u8; 64];
        let began = std::time::Instant::now();
        while !tcp.read_complete() {
            assert!(began.elapsed() < BOUND);
            let n = tcp.read_data(&mut buf, wait).unwrap();
            out.extend_from_slice(&buf[..n]);
        }
        out
    }

    /// An answer cut at each of its offsets, with a pause, comes back whole, and the
    /// next one after it is its own.
    #[test]
    fn a_text_answer_split_anywhere_is_one_answer() {
        let text = b"12.3450\n";
        for at in 1..text.len() {
            let (mut tcp, _listener, mut peer) = pair(false);
            peer.write_all(&text[..at]).unwrap();
            let writer = std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(30));
                peer.write_all(&text[at..]).unwrap();
                peer.write_all(b"1.5000\n").unwrap();
                peer
            });
            assert_eq!(answer(&mut tcp, Duration::from_millis(500)), text, "cut at {at}");
            let _peer = writer.join().unwrap();
            assert_eq!(answer(&mut tcp, Duration::from_millis(500)), b"1.5000\n", "cut at {at}");
        }
    }

    #[test]
    fn two_answers_in_one_read_are_both_given_whole() {
        let (mut tcp, _listener, mut peer) = pair(false);
        peer.write_all(b"1\n").unwrap();
        assert_eq!(answer(&mut tcp, BOUND), b"1\n");
        peer.write_all(b"\n").unwrap();
        assert_eq!(answer(&mut tcp, BOUND), b"\n", "an empty answer is an answer");
    }

    /// A block ends at its length, whatever line feeds it holds, and its line end is
    /// taken off the next answer even when it comes on its own.
    #[test]
    fn a_block_ends_at_its_length() {
        let (mut tcp, _listener, mut peer) = pair(false);
        peer.write_all(b"#15ab").unwrap();
        let writer = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(30));
            peer.write_all(b"\nde").unwrap();
            std::thread::sleep(Duration::from_millis(30));
            peer.write_all(b"\n").unwrap();
            std::thread::sleep(Duration::from_millis(30));
            peer.write_all(b"OK\n").unwrap();
            peer
        });
        assert_eq!(answer(&mut tcp, Duration::from_millis(500)), b"#15ab\nde");
        let _peer = writer.join().unwrap();
        assert_eq!(answer(&mut tcp, Duration::from_millis(500)), b"OK\n");
    }

    /// A driver that reads a block's line end itself, as `rigol-ds` does, gets it.
    #[test]
    fn a_read_past_a_block_gets_what_comes() {
        let (mut tcp, _listener, mut peer) = pair(false);
        peer.write_all(b"#13abc\nOK\n").unwrap();
        assert_eq!(answer(&mut tcp, BOUND), b"#13abc");
        let mut one = [0u8; 1];
        assert_eq!(tcp.read_data(&mut one, BOUND).unwrap(), 1);
        assert_eq!(&one, b"\n");
        assert_eq!(answer(&mut tcp, BOUND), b"OK\n");
    }

    #[test]
    fn what_a_first_bytes_make_of_an_answer() {
        assert_eq!(shape(b""), Shape::Unsure);
        assert_eq!(shape(b"#"), Shape::Unsure);
        assert_eq!(shape(b"#3"), Shape::Unsure);
        assert_eq!(shape(b"#310"), Shape::Unsure);
        assert_eq!(shape(b"#3100"), Shape::Block(105));
        assert_eq!(shape(b"#0abc"), Shape::Text);
        assert_eq!(shape(b"#ABC"), Shape::Text);
        assert_eq!(shape(b"#2x1"), Shape::Text);
        assert_eq!(shape(b"12.5"), Shape::Text);
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
