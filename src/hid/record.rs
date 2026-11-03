//! Recording a HID session to a fixture and replaying it, as
//! `usb::record` does for USB transports and `serial::record` for
//! serial ports.
//!
//! A fixture is a text file, one call per line:
//!
//! ```text
//! # oxrok hid fixture 1
//! device vid=16c0 pid=05df manufacturer="www.dcttech.com" product="USBRelay2" path="/dev/hidraw3"
//! get_feature 01 9 -> 0141424344450000
//! set_feature 00ff01000000000000 -> ok
//! write 0001 -> ok
//! read 64 -> f30d0a
//! read 64 -> -
//! ```
//!
//! `get_feature ID LEN` is a feature report read with the report id
//! `ID` in a buffer of `LEN` bytes; `read LEN` an input report read
//! with a buffer of `LEN` bytes, `-` when none arrived in time. Data is
//! hex pairs, `xx{N}` a run of a byte. Errors are `err KIND ["message"]`
//! as in USB fixtures.
//!
//! Reads that time out depend on the pace of the driver, so on replay
//! they float: a timed out read recorded where the replay makes another
//! call is skipped, and a read where the recording has no read next (or
//! nothing more) answers a timeout. So a fixture may leave timed out
//! reads out altogether, as the emulator's recordings do. Reads that
//! returned data, writes and feature reports must come as recorded.

use crate::core::{Error, Result};
use crate::hid::HidDevice;
use crate::usb::record::{Outcome, hex, quote, unhex, unquote};
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// The first line of a HID fixture, and of each session in a file of
/// several.
pub const MAGIC: &str = "# oxrok hid fixture 1";

/// The device a fixture was recorded from, as listing reported it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DeviceLine {
    /// USB vendor id.
    pub vendor_id: u16,
    /// USB product id.
    pub product_id: u16,
    /// Manufacturer string.
    pub manufacturer: Option<String>,
    /// Product string.
    pub product: Option<String>,
    /// Serial number.
    pub serial: Option<String>,
    /// The device node (`/dev/hidrawN` on Linux), which a `conn=` may
    /// name; on replay only compared as text.
    pub path: Option<String>,
}

/// One recorded call.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Call {
    /// `read_report` with a buffer of this size.
    Read(usize),
    /// `write_report` of these bytes.
    Write(Vec<u8>),
    /// `get_feature` of this report id with a buffer of this size.
    GetFeature(u8, usize),
    /// `set_feature` of these bytes.
    SetFeature(Vec<u8>),
}

/// One recorded call and its result: `Ok`, or `Data` for reads (empty
/// for a read that timed out), or an error.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Step {
    /// The call.
    pub call: Call,
    /// Its result.
    pub outcome: Outcome,
}

/// A recorded session.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fixture {
    /// The device.
    pub device: DeviceLine,
    /// The calls, in order.
    pub steps: Vec<Step>,
}

impl Fixture {
    /// Formats the fixture as text.
    pub fn to_text(&self) -> String {
        let mut s = String::from(MAGIC);
        s.push('\n');
        let d = &self.device;
        let _ = write!(s, "device vid={:04x} pid={:04x}", d.vendor_id, d.product_id);
        for (key, v) in [
            ("manufacturer", &d.manufacturer),
            ("product", &d.product),
            ("serial", &d.serial),
            ("path", &d.path),
        ] {
            if let Some(v) = v {
                let _ = write!(s, " {key}={}", quote(v));
            }
        }
        s.push('\n');
        for step in &self.steps {
            match &step.call {
                Call::Read(n) => {
                    let _ = write!(s, "read {n}");
                }
                Call::Write(d) => {
                    let _ = write!(s, "write {}", hex(d));
                }
                Call::GetFeature(id, n) => {
                    let _ = write!(s, "get_feature {id:02x} {n}");
                }
                Call::SetFeature(d) => {
                    let _ = write!(s, "set_feature {}", hex(d));
                }
            }
            s.push_str(" -> ");
            match &step.outcome {
                Outcome::Data(d) => s.push_str(&hex(d)),
                Outcome::Err(kind, msg) => {
                    let _ = write!(s, "err {kind}");
                    if !msg.is_empty() {
                        let _ = write!(s, " {}", quote(msg));
                    }
                }
                Outcome::Ok | Outcome::Count(_) => s.push_str("ok"),
            }
            s.push('\n');
        }
        s
    }

    /// Parses fixture text.
    pub fn parse(text: &str) -> Result<Fixture> {
        let mut lines = text.lines().enumerate();
        if lines.next().map(|(_, l)| l.trim()) != Some(MAGIC) {
            return Err(Error::arg("not an oxrok hid fixture"));
        }
        let mut fixture = Fixture::default();
        let mut have_device = false;
        for (n, line) in lines {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let bad = |what: &str| Error::Arg(format!("hid fixture line {}: {what}", n + 1));
            if let Some(rest) = line.strip_prefix("device ") {
                fixture.device = parse_device(rest).map_err(|e| bad(&e))?;
                have_device = true;
                continue;
            }
            let (call, outcome) = line.split_once(" -> ").ok_or_else(|| bad("missing ->"))?;
            let call = parse_call(call).map_err(|e| bad(&e))?;
            let outcome = parse_outcome(outcome, &call).map_err(|e| bad(&e))?;
            fixture.steps.push(Step { call, outcome });
        }
        if !have_device {
            return Err(Error::arg("hid fixture has no device line"));
        }
        Ok(fixture)
    }
}

fn parse_device(s: &str) -> std::result::Result<DeviceLine, String> {
    let mut d = DeviceLine::default();
    let mut rest = s.trim();
    while !rest.is_empty() {
        let (key, after) = rest.split_once('=').ok_or("expected key=value")?;
        let (value, after) = if after.starts_with('"') {
            unquote(after)?
        } else {
            let end = after.find(' ').unwrap_or(after.len());
            (after[..end].to_owned(), &after[end..])
        };
        rest = after.trim_start();
        let hex16 = |v: &str| u16::from_str_radix(v, 16).map_err(|_| format!("bad {key}"));
        match key {
            "vid" => d.vendor_id = hex16(&value)?,
            "pid" => d.product_id = hex16(&value)?,
            "manufacturer" => d.manufacturer = Some(value),
            "product" => d.product = Some(value),
            "serial" => d.serial = Some(value),
            "path" => d.path = Some(value),
            other => return Err(format!("unknown device field {other}")),
        }
    }
    Ok(d)
}

fn parse_call(s: &str) -> std::result::Result<Call, String> {
    let words: Vec<&str> = s.split_whitespace().collect();
    let num = |w: &str| w.parse::<usize>().map_err(|_| format!("bad number {w}"));
    Ok(match words.as_slice() {
        ["read", n] => Call::Read(num(n)?),
        ["write", d] => Call::Write(unhex(d)?),
        ["get_feature", id, n] => {
            Call::GetFeature(u8::from_str_radix(id, 16).map_err(|_| "bad report id")?, num(n)?)
        }
        ["set_feature", d] => Call::SetFeature(unhex(d)?),
        _ => return Err(format!("unknown call {s}")),
    })
}

fn parse_outcome(s: &str, call: &Call) -> std::result::Result<Outcome, String> {
    let s = s.trim();
    if let Some(rest) = s.strip_prefix("err ") {
        let (kind, msg) = rest.split_once(' ').unwrap_or((rest, ""));
        let msg = if msg.is_empty() { String::new() } else { unquote(msg.trim())?.0 };
        return Ok(Outcome::Err(kind.to_owned(), msg));
    }
    Ok(match call {
        Call::Read(_) | Call::GetFeature(..) => Outcome::Data(unhex(s)?),
        Call::Write(_) | Call::SetFeature(_) if s == "ok" => Outcome::Ok,
        _ => return Err(format!("bad result {s}")),
    })
}

/// Wraps a HID device and logs every call; with a file, the fixture is
/// written when the recorder is dropped.
pub struct Recorder {
    inner: Box<dyn HidDevice>,
    fixture: Arc<Mutex<Fixture>>,
    file: Option<String>,
}

impl Recorder {
    /// Records `inner`, described by `device`, in memory.
    pub fn new(inner: Box<dyn HidDevice>, device: DeviceLine) -> Recorder {
        Recorder {
            inner,
            fixture: Arc::new(Mutex::new(Fixture { device, steps: Vec::new() })),
            file: None,
        }
    }

    /// Records `inner` and writes the fixture to `file` when dropped.
    pub fn to_file(inner: Box<dyn HidDevice>, device: DeviceLine, file: String) -> Recorder {
        let mut r = Recorder::new(inner, device);
        r.file = Some(file);
        r
    }

    /// A copy of the recording so far.
    pub fn fixture(&self) -> Fixture {
        self.fixture.lock().expect("recording lock").clone()
    }

    fn log(&self, call: Call, outcome: Outcome) {
        self.fixture.lock().expect("recording lock").steps.push(Step { call, outcome });
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Some(file) = &self.file {
            match crate::core::sessions::write_session(file, &self.fixture().to_text()) {
                Ok(()) => log::info!("HID session recorded to {file}"),
                Err(e) => log::error!("cannot write the HID recording {file}: {e}"),
            }
        }
    }
}

fn data_outcome(r: &Result<usize>, buf: &[u8]) -> Outcome {
    match r {
        Ok(n) => Outcome::Data(buf[..*n].to_vec()),
        Err(e) => Outcome::from_error(e),
    }
}

fn unit_outcome(r: &Result<()>) -> Outcome {
    match r {
        Ok(()) => Outcome::Ok,
        Err(e) => Outcome::from_error(e),
    }
}

impl HidDevice for Recorder {
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let r = self.inner.read_report(buf, timeout);
        self.log(Call::Read(buf.len()), data_outcome(&r, buf));
        r
    }

    fn write_report(&mut self, data: &[u8]) -> Result<()> {
        let r = self.inner.write_report(data);
        self.log(Call::Write(data.to_vec()), unit_outcome(&r));
        r
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize> {
        let call = Call::GetFeature(buf.first().copied().unwrap_or(0), buf.len());
        let r = self.inner.get_feature(buf);
        self.log(call, data_outcome(&r, buf));
        r
    }

    fn set_feature(&mut self, data: &[u8]) -> Result<()> {
        let r = self.inner.set_feature(data);
        self.log(Call::SetFeature(data.to_vec()), unit_outcome(&r));
        r
    }
}

/// A HID device that answers from a [`Fixture`] and fails on the first
/// call that differs from it (reads that timed out float, see the
/// module documentation).
pub struct Replay {
    fixture: Fixture,
    pos: usize,
}

impl Replay {
    /// Replays `fixture` from the start.
    pub fn new(fixture: Fixture) -> Replay {
        Replay { fixture, pos: 0 }
    }

    /// Replays fixture text.
    pub fn from_text(text: &str) -> Result<Replay> {
        Ok(Replay::new(Fixture::parse(text)?))
    }

    /// The device line of the recording.
    pub fn device(&self) -> &DeviceLine {
        &self.fixture.device
    }

    /// Fails when steps other than timed out reads remain.
    pub fn finish(&self) -> Result<()> {
        match self.fixture.steps[self.pos..].iter().find(|s| !is_timeout(s)) {
            Some(s) => Err(Error::Device(format!("replay: {:?} was not replayed", s.call))),
            None => Ok(()),
        }
    }

    fn step(&mut self, call: &Call) -> Result<Outcome> {
        let is_read = matches!(call, Call::Read(_));
        loop {
            let Some(step) = self.fixture.steps.get(self.pos) else {
                if is_read {
                    return Ok(Outcome::Data(Vec::new()));
                }
                return Err(Error::Device(format!(
                    "replay: past the end of the recording at {call:?}"
                )));
            };
            if step.call == *call {
                self.pos += 1;
                return Ok(step.outcome.clone());
            }
            if is_timeout(step) {
                self.pos += 1;
                continue;
            }
            if is_read && !matches!(step.call, Call::Read(_)) {
                return Ok(Outcome::Data(Vec::new()));
            }
            return Err(Error::Device(format!(
                "replay: step {} expected {:?}, got {call:?}",
                self.pos + 1,
                step.call
            )));
        }
    }
}

/// A [`Replay`] shared with whoever checks it was played to its end.
#[derive(Clone)]
pub struct SharedReplay(pub Arc<Mutex<Replay>>);

impl SharedReplay {
    /// Shares `replay`.
    pub fn new(replay: Replay) -> SharedReplay {
        SharedReplay(Arc::new(Mutex::new(replay)))
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Replay> {
        self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// As [`Replay::finish`].
    pub fn finish(&self) -> Result<()> {
        self.lock().finish()
    }
}

impl HidDevice for SharedReplay {
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        self.lock().read_report(buf, timeout)
    }

    fn write_report(&mut self, data: &[u8]) -> Result<()> {
        self.lock().write_report(data)
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.lock().get_feature(buf)
    }

    fn set_feature(&mut self, data: &[u8]) -> Result<()> {
        self.lock().set_feature(data)
    }
}

fn is_timeout(step: &Step) -> bool {
    matches!(step.call, Call::Read(_)) && step.outcome == Outcome::Data(Vec::new())
}

fn answer(outcome: Outcome, buf: &mut [u8]) -> Result<usize> {
    match outcome {
        Outcome::Data(d) => {
            let n = d.len().min(buf.len());
            buf[..n].copy_from_slice(&d[..n]);
            Ok(n)
        }
        Outcome::Err(..) => Err(outcome.to_error()),
        Outcome::Ok | Outcome::Count(_) => Err(Error::device("replay: a read recorded no data")),
    }
}

fn done(outcome: &Outcome) -> Result<()> {
    match outcome {
        Outcome::Err(..) => Err(outcome.to_error()),
        _ => Ok(()),
    }
}

impl HidDevice for Replay {
    fn read_report(&mut self, buf: &mut [u8], _: Duration) -> Result<usize> {
        let outcome = self.step(&Call::Read(buf.len()))?;
        answer(outcome, buf)
    }

    fn write_report(&mut self, data: &[u8]) -> Result<()> {
        done(&self.step(&Call::Write(data.to_vec()))?)
    }

    fn get_feature(&mut self, buf: &mut [u8]) -> Result<usize> {
        let call = Call::GetFeature(buf.first().copied().unwrap_or(0), buf.len());
        let outcome = self.step(&call)?;
        answer(outcome, buf)
    }

    fn set_feature(&mut self, data: &[u8]) -> Result<()> {
        done(&self.step(&Call::SetFeature(data.to_vec()))?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hid::SharedFake;

    const TEXT: &str = "# oxrok hid fixture 1
device vid=16c0 pid=05df manufacturer=\"www.dcttech.com\" product=\"USBRelay2\" path=\"/dev/hidraw3\"
get_feature 01 9 -> 0141424344450000
set_feature 00ff01 -> ok
read 64 -> -
read 64 -> -
read 64 -> f30d0a
write 0001 -> err device \"stalled\"
";

    #[test]
    fn round_trips() {
        let f = Fixture::parse(TEXT).unwrap();
        assert_eq!(f.to_text(), TEXT);
        assert_eq!(f.device.product.as_deref(), Some("USBRelay2"));
        assert_eq!(f.device.path.as_deref(), Some("/dev/hidraw3"));
        assert_eq!(f.steps.len(), 6);
    }

    #[test]
    fn records_what_the_device_answers() {
        let fake = SharedFake::default();
        fake.0.lock().unwrap().feature_answers.push_back(vec![1, 2, 3]);
        fake.push_input(&[]);
        fake.push_input(&[0xf3, 0x0d]);
        let mut r = Recorder::new(Box::new(fake.clone()), DeviceLine::default());
        let mut buf = [0u8; 9];
        buf[0] = 1;
        assert_eq!(r.get_feature(&mut buf).unwrap(), 3);
        r.set_feature(&[0, 0xff, 1]).unwrap();
        let mut report = [0u8; 64];
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 0);
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 2);
        let text = r.fixture().to_text();
        assert!(text.contains("get_feature 01 9 -> 010203\n"), "{text}");
        assert!(text.contains("set_feature 00ff01 -> ok\nread 64 -> -\nread 64 -> f30d\n"));
        assert_eq!(fake.features(), [vec![0, 0xff, 1]]);
    }

    #[test]
    fn timed_out_reads_float() {
        let mut buf = [0u8; 9];
        buf[0] = 1;
        let mut report = [0u8; 64];
        // As recorded.
        let mut r = Replay::from_text(TEXT).unwrap();
        assert_eq!(r.get_feature(&mut buf).unwrap(), 8);
        assert_eq!(&buf[..3], [1, 0x41, 0x42]);
        r.set_feature(&[0, 0xff, 1]).unwrap();
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 0);
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 0);
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 3);
        assert!(r.write_report(&[0, 1]).unwrap_err().to_string().contains("stalled"));
        r.finish().unwrap();
        // Without the timed out reads: a read before the feature
        // report and after the data answers a timeout.
        let text: String =
            TEXT.lines().filter(|l| *l != "read 64 -> -").flat_map(|l| [l, "\n"]).collect();
        let mut r = Replay::from_text(&text).unwrap();
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 0);
        r.get_feature(&mut buf).unwrap();
        r.set_feature(&[0, 0xff, 1]).unwrap();
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 3);
        assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 0);
        assert!(r.write_report(&[0, 1]).is_err());
        for _ in 0..5 {
            assert_eq!(r.read_report(&mut report, Duration::ZERO).unwrap(), 0);
        }
        r.finish().unwrap();
        // Recorded timeouts the replay does not read are skipped.
        let mut r = Replay::from_text(TEXT).unwrap();
        r.get_feature(&mut buf).unwrap();
        r.set_feature(&[0, 0xff, 1]).unwrap();
        assert!(r.write_report(&[0, 1]).unwrap_err().to_string().contains("expected"));
    }

    #[test]
    fn data_reads_and_commands_do_not_float() {
        let mut buf = [1u8; 9];
        let mut report = [0u8; 64];
        let mut r = Replay::from_text(TEXT).unwrap();
        r.get_feature(&mut buf).unwrap();
        assert!(r.finish().is_err());
        r.set_feature(&[0, 0xff, 1]).unwrap();
        // A data read is next: another call fails, as does a read of
        // another size.
        assert!(r.get_feature(&mut buf).is_err());
        let mut r = Replay::from_text(TEXT).unwrap();
        r.get_feature(&mut buf).unwrap();
        r.set_feature(&[0, 0xff, 1]).unwrap();
        assert!(r.read_report(&mut report[..32], Duration::ZERO).is_err());
        // A different feature report fails.
        let mut r = Replay::from_text(TEXT).unwrap();
        r.get_feature(&mut buf).unwrap();
        assert!(r.set_feature(&[0, 0xfd, 1]).is_err());
    }
}
