//! Recording a serial session to a fixture and replaying it. The
//! format is described in the crate documentation.

use crate::core::{Error, Result};
use crate::serial::{Params, Port};
use std::fmt::Write as _;
use std::time::Duration;

/// One recorded call.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// A write of these bytes.
    Write(Vec<u8>),
    /// A read of up to the first many bytes that returned the second.
    Read(usize, Vec<u8>),
    /// A parameter change.
    Params(Params),
    /// A handshake line change.
    Handshake(Option<bool>, Option<bool>),
    /// Pending input discarded.
    Purge,
}

fn hex(data: &[u8]) -> String {
    if data.is_empty() {
        return "-".to_owned();
    }
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        let _ = write!(s, "{b:02x}");
    }
    s
}

fn unhex(text: &str) -> Option<Vec<u8>> {
    if text == "-" {
        return Some(Vec::new());
    }
    if !text.len().is_multiple_of(2) {
        return None;
    }
    (0..text.len()).step_by(2).map(|i| u8::from_str_radix(&text[i..i + 2], 16).ok()).collect()
}

#[allow(clippy::option_option)]
fn level(text: &str) -> Option<Option<bool>> {
    match text {
        "-" => Some(None),
        "0" => Some(Some(false)),
        "1" => Some(Some(true)),
        _ => None,
    }
}

fn level_text(l: Option<bool>) -> &'static str {
    match l {
        None => "-",
        Some(false) => "0",
        Some(true) => "1",
    }
}

/// A recorded session: the port, its parameters and the steps.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Fixture {
    /// The port name.
    pub name: String,
    /// The parameters it was opened with.
    pub params: Params,
    /// The calls, in order.
    pub steps: Vec<Step>,
}

impl Fixture {
    /// Parses the text format.
    pub fn parse(text: &str) -> Result<Fixture> {
        let mut lines = text.lines().map(str::trim).filter(|l| !l.is_empty());
        let bad = |l: &str| Error::Arg(format!("Invalid serial fixture line '{l}'."));
        if lines.next() != Some("# oxrok serial fixture 1") {
            return Err(Error::arg("Not a serial fixture."));
        }
        let mut name = String::new();
        let mut params = None;
        let mut steps = Vec::new();
        for line in lines {
            if line.starts_with('#') {
                continue;
            }
            let words: Vec<&str> = line.split_whitespace().collect();
            match words.as_slice() {
                ["port", n, p] => {
                    (*n).clone_into(&mut name);
                    params = Some(Params::parse(p)?);
                }
                ["write", data] => steps.push(Step::Write(unhex(data).ok_or_else(|| bad(line))?)),
                ["read", len, "->", data] => steps.push(Step::Read(
                    len.parse().map_err(|_| bad(line))?,
                    unhex(data).ok_or_else(|| bad(line))?,
                )),
                ["params", p] => steps.push(Step::Params(Params::parse(p)?)),
                ["handshake", r, d] => steps.push(Step::Handshake(
                    level(r).ok_or_else(|| bad(line))?,
                    level(d).ok_or_else(|| bad(line))?,
                )),
                ["purge"] => steps.push(Step::Purge),
                _ => return Err(bad(line)),
            }
        }
        let params = params.ok_or_else(|| Error::arg("Serial fixture without a port line."))?;
        Ok(Fixture { name, params, steps })
    }

    /// The text format.
    pub fn to_text(&self) -> String {
        let mut s =
            format!("# oxrok serial fixture 1\nport {} {}\n", self.name, self.params.to_text());
        for step in &self.steps {
            match step {
                Step::Write(d) => {
                    let _ = writeln!(s, "write {}", hex(d));
                }
                Step::Read(n, d) => {
                    let _ = writeln!(s, "read {n} -> {}", hex(d));
                }
                Step::Params(p) => {
                    let _ = writeln!(s, "params {}", p.to_text());
                }
                Step::Handshake(r, d) => {
                    let _ = writeln!(s, "handshake {} {}", level_text(*r), level_text(*d));
                }
                Step::Purge => s.push_str("purge\n"),
            }
        }
        s
    }
}

/// Wraps a port and records every call; the fixture is written when
/// the recorder is dropped.
pub struct Recorder {
    inner: Box<dyn Port>,
    fixture: Fixture,
    file: Option<String>,
}

impl Recorder {
    /// Records `inner`, opened with `params`, into `file` on drop.
    pub fn to_file(inner: Box<dyn Port>, params: Params, file: String) -> Recorder {
        let fixture = Fixture { name: inner.name().to_owned(), params, steps: Vec::new() };
        Recorder { inner, fixture, file: Some(file) }
    }

    /// Records `inner` in memory; [`Recorder::fixture`] reads it back.
    pub fn in_memory(inner: Box<dyn Port>, params: Params) -> Recorder {
        let fixture = Fixture { name: inner.name().to_owned(), params, steps: Vec::new() };
        Recorder { inner, fixture, file: None }
    }

    /// What was recorded so far.
    pub fn fixture(&self) -> &Fixture {
        &self.fixture
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Some(file) = &self.file
            && let Err(e) = std::fs::write(file, self.fixture.to_text())
        {
            log::error!("Writing the serial fixture {file}: {e}.");
        }
    }
}

impl Port for Recorder {
    fn name(&self) -> &str {
        self.inner.name()
    }

    fn set_params(&mut self, params: Params) -> Result<()> {
        self.fixture.steps.push(Step::Params(params));
        self.inner.set_params(params)
    }

    fn set_handshake(&mut self, rts: Option<bool>, dtr: Option<bool>) -> Result<()> {
        self.fixture.steps.push(Step::Handshake(rts, dtr));
        self.inner.set_handshake(rts, dtr)
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let n = self.inner.read(buf, timeout)?;
        self.fixture.steps.push(Step::Read(buf.len(), buf[..n].to_vec()));
        Ok(n)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.fixture.steps.push(Step::Write(data.to_vec()));
        self.inner.write(data)
    }

    fn purge(&mut self) -> Result<()> {
        self.fixture.steps.push(Step::Purge);
        self.inner.purge()
    }

    fn drain(&mut self, timeout: Duration) -> Result<()> {
        self.inner.drain(timeout)
    }
}

/// Plays a fixture back. Reads return the recorded data in order; a
/// read asking for less than a recorded chunk gets the rest on the
/// next call. Writes must match the recorded ones in order; recorded
/// empty reads in between are skipped, so a driver that polls at a
/// different pace still matches. Parameter and handshake steps are
/// checked loosely: they may be repeated or skipped.
pub struct Replay {
    fixture: Fixture,
    pos: usize,
    /// Bytes of the current read step not yet returned.
    pending: Vec<u8>,
}

impl Replay {
    /// A port playing `fixture`.
    pub fn new(fixture: Fixture) -> Replay {
        Replay { fixture, pos: 0, pending: Vec::new() }
    }

    /// A port playing the fixture in `text`.
    pub fn from_text(text: &str) -> Result<Replay> {
        Ok(Replay::new(Fixture::parse(text)?))
    }

    /// The parameters the recording was opened with.
    pub fn params(&self) -> Params {
        self.fixture.params
    }

    /// True when every step was consumed.
    pub fn finished(&self) -> bool {
        self.pos >= self.fixture.steps.len() && self.pending.is_empty()
    }

    fn skip_passive(&mut self) {
        while let Some(Step::Params(_) | Step::Handshake(..) | Step::Purge) =
            self.fixture.steps.get(self.pos)
        {
            self.pos += 1;
        }
    }
}

impl Port for Replay {
    fn name(&self) -> &str {
        &self.fixture.name
    }

    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }

    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
        if self.pending.is_empty() {
            self.skip_passive();
            match self.fixture.steps.get(self.pos) {
                Some(Step::Read(_, data)) => {
                    self.pending = data.clone();
                    self.pos += 1;
                }
                Some(Step::Write(_)) => {
                    return Err(Error::device("replay: a read where a write was recorded"));
                }
                _ => return Err(Error::Eof),
            }
        }
        let n = buf.len().min(self.pending.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        loop {
            self.skip_passive();
            match self.fixture.steps.get(self.pos) {
                Some(Step::Write(recorded)) if recorded == data => {
                    self.pos += 1;
                    return Ok(());
                }
                Some(Step::Write(recorded)) => {
                    return Err(Error::device(format!(
                        "replay: wrote {} where {} was recorded",
                        hex(data),
                        hex(recorded)
                    )));
                }
                Some(Step::Read(_, d)) if d.is_empty() && self.pending.is_empty() => self.pos += 1,
                _ => return Err(Error::device("replay: a write where a read was recorded")),
            }
        }
    }

    fn purge(&mut self) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unnecessary_literal_bound)]
mod tests {
    use super::*;

    const TEXT: &str = "# oxrok serial fixture 1
port /dev/ttyUSB0 2400/8n1/rts=0/dtr=1
handshake 0 1
write 44
read 8 -> -
read 8 -> 2b30
read 8 -> 3132
";

    #[test]
    fn round_trips_and_replays() {
        let f = Fixture::parse(TEXT).unwrap();
        assert_eq!(f.to_text(), TEXT);
        assert_eq!(f.steps.len(), 5);
        let mut r: Box<dyn Port> = Box::new(Replay::new(f));
        r.set_handshake(Some(false), Some(true)).unwrap();
        r.write(b"D").unwrap();
        let mut buf = [0u8; 8];
        assert_eq!(r.read(&mut buf, Duration::ZERO).unwrap(), 0);
        assert_eq!(r.read(&mut buf[..1], Duration::ZERO).unwrap(), 1);
        assert_eq!(buf[0], b'+');
        let mut rest = [0u8; 3];
        r.read_exact(&mut rest, Duration::ZERO).unwrap();
        assert_eq!(&rest, b"012");
        assert!(matches!(r.read(&mut buf, Duration::ZERO), Err(Error::Eof)));
    }

    #[test]
    fn writes_skip_empty_reads_and_must_match() {
        let mut r = Replay::from_text(
            "# oxrok serial fixture 1\nport p 9600/8n1\nread 4 -> -\nread 4 -> -\nwrite 01\nread 4 -> 02\n",
        )
        .unwrap();
        r.write(&[1]).unwrap();
        let mut b = [0u8; 4];
        assert_eq!(r.read(&mut b, Duration::ZERO).unwrap(), 1);
        assert!(r.finished());
        let mut r =
            Replay::from_text("# oxrok serial fixture 1\nport p 9600/8n1\nwrite 01\n").unwrap();
        assert!(r.write(&[2]).is_err());
    }

    #[test]
    fn recorder_captures_calls() {
        struct Fake;
        impl Port for Fake {
            fn name(&self) -> &str {
                "fake"
            }
            fn set_params(&mut self, _: Params) -> Result<()> {
                Ok(())
            }
            fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
                Ok(())
            }
            fn read(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
                buf[0] = 0xaa;
                Ok(1)
            }
            fn write(&mut self, _: &[u8]) -> Result<()> {
                Ok(())
            }
            fn purge(&mut self) -> Result<()> {
                Ok(())
            }
        }
        let params = Params::parse("9600/8n1").unwrap();
        let mut rec = Recorder::in_memory(Box::new(Fake), params);
        rec.write(&[1, 2]).unwrap();
        let mut b = [0u8; 4];
        rec.read(&mut b, Duration::ZERO).unwrap();
        assert_eq!(
            rec.fixture().to_text(),
            "# oxrok serial fixture 1\nport fake 9600/8n1\nwrite 0102\nread 4 -> aa\n"
        );
    }
}
