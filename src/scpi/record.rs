//! Recording a SCPI session and replaying it. The fixture lists the
//! messages sent and the responses collected, one per line:
//!
//! ```text
//! # oxrok scpi fixture 1
//! resource usbtmc/1.5
//! send "*IDN?\n"
//! response "Rigol Technologies,DS1102E,DS1EB104702974,00.04.02\n"
//! send ":WAV:DATA? CHAN1\n"
//! response hex 2339303030303630302a...
//! partial -
//! ```
//!
//! Text is quoted with `\n`, `\r`, `\"`, `\\` and `\xHH` escapes; a
//! response that is mostly binary is `hex`. `partial` is a response
//! the transport never completed (a timeout), `-` an empty one.

use crate::core::{Error, Result};
use crate::scpi::ScpiTransport;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const MAGIC: &str = "# oxrok scpi fixture 1";

/// One recorded exchange.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// A message sent to the instrument.
    Send(Vec<u8>),
    /// A response, complete or not.
    Response {
        /// The bytes read.
        data: Vec<u8>,
        /// Whether the transport reported the response complete.
        complete: bool,
    },
}

/// A recorded session.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fixture {
    /// The resource string.
    pub resource: String,
    /// The exchanges, in order.
    pub steps: Vec<Step>,
}

fn quote(data: &[u8]) -> String {
    let mut s = String::from("\"");
    for &b in data {
        match b {
            b'\n' => s.push_str("\\n"),
            b'\r' => s.push_str("\\r"),
            b'"' => s.push_str("\\\""),
            b'\\' => s.push_str("\\\\"),
            0x20..=0x7e => s.push(b as char),
            _ => {
                let _ = write!(s, "\\x{b:02x}");
            }
        }
    }
    s.push('"');
    s
}

fn unquote(text: &str) -> std::result::Result<Vec<u8>, String> {
    let inner = text
        .strip_prefix('"')
        .and_then(|t| t.strip_suffix('"'))
        .ok_or("expected a quoted string")?;
    let bytes = inner.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'\\' {
            let e = *bytes.get(i + 1).ok_or("unterminated escape")?;
            i += 2;
            match e {
                b'n' => out.push(b'\n'),
                b'r' => out.push(b'\r'),
                b'"' => out.push(b'"'),
                b'\\' => out.push(b'\\'),
                b'x' => {
                    let h = inner.get(i..i + 2).ok_or("short \\x escape")?;
                    out.push(u8::from_str_radix(h, 16).map_err(|_| "bad \\x escape")?);
                    i += 2;
                }
                _ => return Err("unknown escape".into()),
            }
        } else {
            out.push(bytes[i]);
            i += 1;
        }
    }
    Ok(out)
}

fn hex(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        let _ = write!(s, "{b:02x}");
    }
    s
}

fn unhex(text: &str) -> std::result::Result<Vec<u8>, String> {
    if !text.len().is_multiple_of(2) {
        return Err("odd hex length".into());
    }
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).map_err(|_| "bad hex".to_owned()))
        .collect()
}

fn data_text(data: &[u8]) -> String {
    if data.is_empty() {
        return "-".to_owned();
    }
    let printable = data.iter().filter(|b| matches!(b, 0x20..=0x7e | b'\n' | b'\r')).count();
    if data.len() <= 256 && printable * 4 >= data.len() * 3 {
        quote(data)
    } else {
        format!("hex {}", hex(data))
    }
}

fn parse_data(text: &str) -> std::result::Result<Vec<u8>, String> {
    let text = text.trim();
    if text == "-" {
        return Ok(Vec::new());
    }
    if let Some(h) = text.strip_prefix("hex ") {
        return unhex(h.trim());
    }
    unquote(text)
}

impl Fixture {
    /// The fixture as text.
    pub fn to_text(&self) -> String {
        let mut s = format!("{MAGIC}\nresource {}\n", self.resource);
        for step in &self.steps {
            match step {
                Step::Send(d) => {
                    let _ = writeln!(s, "send {}", data_text(d));
                }
                Step::Response { data, complete } => {
                    let kind = if *complete { "response" } else { "partial" };
                    let _ = writeln!(s, "{kind} {}", data_text(data));
                }
            }
        }
        s
    }

    /// Parses fixture text.
    pub fn parse(text: &str) -> Result<Fixture> {
        let mut lines = text.lines().enumerate();
        let Some((_, first)) = lines.next() else {
            return Err(Error::arg("empty fixture"));
        };
        if first.trim() != MAGIC {
            return Err(Error::arg("not an oxrok scpi fixture"));
        }
        let mut f = Fixture::default();
        for (n, line) in lines {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let bad = |what: &str| Error::Arg(format!("fixture line {}: {what}", n + 1));
            let (kind, rest) = line.split_once(' ').unwrap_or((line, ""));
            match kind {
                "resource" => rest.trim().clone_into(&mut f.resource),
                "send" => f.steps.push(Step::Send(parse_data(rest).map_err(|e| bad(&e))?)),
                "response" | "partial" => f.steps.push(Step::Response {
                    data: parse_data(rest).map_err(|e| bad(&e))?,
                    complete: kind == "response",
                }),
                other => return Err(bad(&format!("unknown line {other}"))),
            }
        }
        Ok(f)
    }
}

/// A transport that records what passes through it.
pub struct Recorder {
    inner: Box<dyn ScpiTransport>,
    fixture: Arc<Mutex<Fixture>>,
    pending: Option<Vec<u8>>,
    file: Option<String>,
}

impl Recorder {
    /// Wraps `inner`; the fixture is written to `file` when dropped.
    pub fn to_file(inner: Box<dyn ScpiTransport>, file: String) -> Recorder {
        let mut r = Recorder::new(inner);
        r.file = Some(file);
        r
    }

    /// Wraps `inner` and keeps the fixture in memory.
    pub fn new(inner: Box<dyn ScpiTransport>) -> Recorder {
        let fixture = Fixture { resource: inner.resource().to_owned(), steps: Vec::new() };
        Recorder { inner, fixture: Arc::new(Mutex::new(fixture)), pending: None, file: None }
    }

    /// A copy of the recording so far; a response still being read is
    /// not in it yet.
    pub fn fixture(&self) -> Fixture {
        self.fixture.lock().expect("fixture lock").clone()
    }

    fn finish_response(&mut self, complete: bool) {
        if let Some(data) = self.pending.take() {
            self.fixture
                .lock()
                .expect("fixture lock")
                .steps
                .push(Step::Response { data, complete });
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        self.finish_response(false);
        if let Some(path) = &self.file {
            let text = self.fixture.lock().expect("fixture lock").to_text();
            match std::fs::write(path, text) {
                Ok(()) => log::info!("SCPI session recorded to {path}"),
                Err(e) => log::error!("cannot write the SCPI recording {path}: {e}"),
            }
        }
    }
}

impl ScpiTransport for Recorder {
    fn resource(&self) -> &str {
        self.inner.resource()
    }

    /// No step in a fixture: nothing goes over the wire.
    fn purge(&mut self) -> Result<()> {
        self.inner.purge()
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.finish_response(false);
        self.fixture.lock().expect("fixture lock").steps.push(Step::Send(data.to_vec()));
        self.inner.send(data)
    }

    fn read_begin(&mut self) -> Result<()> {
        self.finish_response(false);
        self.pending = Some(Vec::new());
        self.inner.read_begin()
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let r = self.inner.read_data(buf, timeout);
        if let Ok(n) = r {
            self.pending.get_or_insert_with(Vec::new).extend_from_slice(&buf[..n]);
            if self.inner.read_complete() {
                self.finish_response(true);
            }
        }
        r
    }

    fn read_complete(&self) -> bool {
        self.inner.read_complete()
    }
}

/// A transport that answers from a fixture.
pub struct Replay {
    fixture: Fixture,
    pos: usize,
    /// The response being read: its data and how far.
    current: Option<(Vec<u8>, bool, usize)>,
}

impl Replay {
    /// Replays `fixture`.
    pub fn new(fixture: Fixture) -> Replay {
        Replay { fixture, pos: 0, current: None }
    }

    /// Replays fixture text.
    pub fn from_text(text: &str) -> Result<Replay> {
        Ok(Replay::new(Fixture::parse(text)?))
    }

    /// Steps not yet replayed.
    pub fn remaining(&self) -> usize {
        self.fixture.steps.len() - self.pos
    }
}

impl ScpiTransport for Replay {
    fn resource(&self) -> &str {
        &self.fixture.resource
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        self.current = None;
        match self.fixture.steps.get(self.pos) {
            Some(Step::Send(d)) if d == data => {
                self.pos += 1;
                Ok(())
            }
            Some(other) => Err(Error::Device(format!(
                "replay: step {} expected {other:?}, got send {}",
                self.pos + 1,
                quote(data)
            ))),
            None => Err(Error::Device(format!(
                "replay: past the end of the recording at send {}",
                quote(data)
            ))),
        }
    }

    fn read_begin(&mut self) -> Result<()> {
        match self.fixture.steps.get(self.pos) {
            Some(Step::Response { data, complete }) => {
                self.current = Some((data.clone(), *complete, 0));
                self.pos += 1;
                Ok(())
            }
            Some(other) => Err(Error::Device(format!(
                "replay: step {} expected a response, got {other:?}",
                self.pos + 1
            ))),
            None => Err(Error::device("replay: past the end of the recording at a read")),
        }
    }

    fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
        let Some((data, complete, at)) = &mut self.current else {
            return Err(Error::device("replay: read without read_begin"));
        };
        if *at >= data.len() {
            return if *complete { Ok(0) } else { Err(Error::Timeout) };
        }
        let n = buf.len().min(data.len() - *at);
        buf[..n].copy_from_slice(&data[*at..*at + n]);
        *at += n;
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        match &self.current {
            Some((data, complete, at)) => *complete && *at >= data.len(),
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixture_round_trip() {
        let f = Fixture {
            resource: "usbtmc/1.5".into(),
            steps: vec![
                Step::Send(b"*IDN?\n".to_vec()),
                Step::Response { data: b"Rigol \"X\",1\r\n".to_vec(), complete: true },
                Step::Send(b":WAV:DATA?\n".to_vec()),
                Step::Response { data: vec![0x23, 0x00, 0xff, 0x80, 0x01], complete: false },
                Step::Response { data: Vec::new(), complete: true },
            ],
        };
        let text = f.to_text();
        assert!(text.contains("response \"Rigol \\\"X\\\",1\\r\\n\""));
        assert!(text.contains("partial hex 2300ff8001"));
        assert!(text.contains("response -"));
        assert_eq!(Fixture::parse(&text).unwrap(), f);
        assert!(Fixture::parse("# other\n").is_err());
    }

    #[test]
    fn recorder_then_replay() {
        let script = Fixture {
            resource: "r".into(),
            steps: vec![
                Step::Send(b"A?\n".to_vec()),
                Step::Response { data: b"1\n".to_vec(), complete: true },
            ],
        };
        let mut rec = Recorder::new(Box::new(Replay::new(script.clone())));
        rec.send(b"A?\n").unwrap();
        rec.read_begin().unwrap();
        let mut buf = [0u8; 8];
        assert_eq!(rec.read_data(&mut buf, Duration::from_millis(1)).unwrap(), 2);
        assert!(rec.read_complete());
        assert_eq!(rec.fixture(), script);
        let mut rp = Replay::new(script);
        assert!(rp.send(b"B?\n").is_err());
    }
}
