//! Recording a USB session to a fixture and replaying it.
//!
//! A fixture is a text file, one call per line, hand editable:
//!
//! ```text
//! # oxrok usb fixture 1
//! device vid=0403 pid=dc48 version=0400 bus=2 addr=7 product="LogicPort Logic Analyzer"
//! claim 0 -> ok
//! ctl_out 40 00 0000 0001 - -> ok
//! ctl_in 40 0c 0000 0001 1 -> 18
//! bulk_out 02 04 -> 1
//! bulk_in 81 64 -> 3160
//! bulk_in 81 64 -> err timeout
//! int_out 01 b9 -> 1
//! int_in 81 64 -> 00
//! ```
//!
//! Control requests are `bmRequestType bRequest wValue wIndex` in hex,
//! then the data (`-` when empty) or the length. Data is hex pairs; a
//! run of the same byte may be written `00{4096}`, and data sent to the
//! device that is longer than [`HASH_ABOVE`] bytes is recorded as
//! `@LEN:SHA256` and compared by digest on replay. Results are `ok`, a
//! number, hex data, or `err <kind> ["message"]` with kind `timeout`,
//! `busy`, `device`, `io` or `unsupported`.

use crate::core::Error;
use crate::usb::{BulkStream, ControlRequest, Result, Transport, UsbDeviceInfo};
use sha2::{Digest as _, Sha256};
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Data sent to the device above this size is recorded as a digest.
pub const HASH_ABOVE: usize = 1024;

/// Data sent to the device: the bytes, or their length and SHA-256.
#[derive(Clone, Debug, Eq)]
pub enum Payload {
    /// The bytes themselves.
    Bytes(Vec<u8>),
    /// Length and SHA-256 of the bytes.
    Digest(usize, [u8; 32]),
}

impl Payload {
    /// The payload of `data`, hashed when longer than [`HASH_ABOVE`].
    pub fn of(data: &[u8]) -> Self {
        if data.len() > HASH_ABOVE {
            Payload::Digest(data.len(), Sha256::digest(data).into())
        } else {
            Payload::Bytes(data.to_vec())
        }
    }

    fn digest(&self) -> (usize, [u8; 32]) {
        match self {
            Payload::Bytes(b) => (b.len(), Sha256::digest(b).into()),
            Payload::Digest(n, d) => (*n, *d),
        }
    }
}

impl PartialEq for Payload {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Payload::Bytes(a), Payload::Bytes(b)) => a == b,
            _ => self.digest() == other.digest(),
        }
    }
}

impl From<Vec<u8>> for Payload {
    fn from(v: Vec<u8>) -> Self {
        Payload::Bytes(v)
    }
}

/// A call to the transport, without its result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Call {
    /// `set_configuration`.
    SetConfiguration(u8),
    /// `claim_interface`.
    Claim(u8),
    /// `release_interface`.
    Release(u8),
    /// `reset`.
    Reset,
    /// `max_packet_size`.
    MaxPacketSize(u8),
    /// `interfaces`; the outcome data is the descriptors encoded by
    /// [`encode_interfaces`].
    Interfaces,
    /// `control_out`.
    ControlOut(ControlRequest, Payload),
    /// `control_in` with the requested length.
    ControlIn(ControlRequest, usize),
    /// `bulk_out`.
    BulkOut(u8, Payload),
    /// `bulk_in` with the requested length.
    BulkIn(u8, usize),
    /// `interrupt_out`.
    IntOut(u8, Payload),
    /// `interrupt_in` with the requested length.
    IntIn(u8, usize),
}

/// What a call returned.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// `Ok(())`.
    Ok,
    /// A byte count.
    Count(usize),
    /// Data read.
    Data(Vec<u8>),
    /// An error, by kind name and message.
    Err(String, String),
}

impl Outcome {
    fn from_error(e: &Error) -> Self {
        let (kind, msg) = match e {
            Error::Timeout => ("timeout", String::new()),
            Error::Busy => ("busy", String::new()),
            Error::Io(e) => ("io", e.to_string()),
            Error::NotSupported(m) => ("unsupported", m.clone()),
            other => ("device", other.to_string()),
        };
        Outcome::Err(kind.to_owned(), msg)
    }

    fn to_error(&self) -> Error {
        match self {
            Outcome::Err(kind, msg) => match kind.as_str() {
                "timeout" => Error::Timeout,
                "busy" => Error::Busy,
                "io" => Error::Io(std::io::Error::other(msg.clone())),
                "unsupported" => Error::NotSupported(msg.clone()),
                _ => Error::Device(msg.clone()),
            },
            _ => Error::device("not an error"),
        }
    }
}

/// One recorded call and its result.
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
    /// The device as enumeration described it.
    pub device: UsbDeviceInfo,
    /// The calls, in order.
    pub steps: Vec<Step>,
}

const MAGIC: &str = "# oxrok usb fixture 1";

impl Fixture {
    /// Formats the fixture as text.
    pub fn to_text(&self) -> String {
        let mut s = String::new();
        s.push_str(MAGIC);
        s.push('\n');
        let d = &self.device;
        let _ = write!(
            s,
            "device vid={:04x} pid={:04x} version={:04x} bus={} addr={}",
            d.vendor_id, d.product_id, d.device_version, d.bus, d.address
        );
        for (key, v) in
            [("manufacturer", &d.manufacturer), ("product", &d.product), ("serial", &d.serial)]
        {
            if let Some(v) = v {
                let _ = write!(s, " {key}={}", quote(v));
            }
        }
        if !d.interfaces.is_empty() {
            let list: Vec<String> = d
                .interfaces
                .iter()
                .map(|i| {
                    format!("{}:{:02x}:{:02x}:{:02x}", i.number, i.class, i.subclass, i.protocol)
                })
                .collect();
            let _ = write!(s, " interfaces={}", list.join(","));
        }
        s.push('\n');
        for step in &self.steps {
            match &step.call {
                Call::SetConfiguration(c) => {
                    let _ = write!(s, "config {c}");
                }
                Call::Claim(i) => {
                    let _ = write!(s, "claim {i}");
                }
                Call::Release(i) => {
                    let _ = write!(s, "release {i}");
                }
                Call::Reset => s.push_str("reset"),
                Call::MaxPacketSize(ep) => {
                    let _ = write!(s, "mps {ep:02x}");
                }
                Call::Interfaces => s.push_str("interfaces"),
                Call::ControlOut(r, data) => {
                    let _ = write!(
                        s,
                        "ctl_out {:02x} {:02x} {:04x} {:04x} {}",
                        r.request_type,
                        r.request,
                        r.value,
                        r.index,
                        payload(data)
                    );
                }
                Call::ControlIn(r, len) => {
                    let _ = write!(
                        s,
                        "ctl_in {:02x} {:02x} {:04x} {:04x} {len}",
                        r.request_type, r.request, r.value, r.index
                    );
                }
                Call::BulkOut(ep, data) => {
                    let _ = write!(s, "bulk_out {ep:02x} {}", payload(data));
                }
                Call::BulkIn(ep, len) => {
                    let _ = write!(s, "bulk_in {ep:02x} {len}");
                }
                Call::IntOut(ep, data) => {
                    let _ = write!(s, "int_out {ep:02x} {}", payload(data));
                }
                Call::IntIn(ep, len) => {
                    let _ = write!(s, "int_in {ep:02x} {len}");
                }
            }
            s.push_str(" -> ");
            match &step.outcome {
                Outcome::Ok => s.push_str("ok"),
                Outcome::Count(n) => {
                    let _ = write!(s, "{n}");
                }
                Outcome::Data(d) => s.push_str(&hex(d)),
                Outcome::Err(kind, msg) => {
                    let _ = write!(s, "err {kind}");
                    if !msg.is_empty() {
                        let _ = write!(s, " {}", quote(msg));
                    }
                }
            }
            s.push('\n');
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
            return Err(Error::arg("not an oxrok usb fixture"));
        }
        let mut fixture = Fixture::default();
        let mut have_device = false;
        for (n, line) in lines {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let bad = |what: &str| Error::Arg(format!("fixture line {}: {what}", n + 1));
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
            return Err(Error::arg("fixture has no device line"));
        }
        Ok(fixture)
    }
}

fn quote(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' | '\\' => {
                out.push('\\');
                out.push(c);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// Reads a quoted string at the start of `s`; returns it and the rest.
fn unquote(s: &str) -> std::result::Result<(String, &str), String> {
    let rest = s.strip_prefix('"').ok_or("expected a quoted string")?;
    let mut out = String::new();
    let mut chars = rest.char_indices();
    while let Some((i, c)) = chars.next() {
        match c {
            '\\' => {
                let (_, e) = chars.next().ok_or("unterminated escape")?;
                out.push(e);
            }
            '"' => return Ok((out, &rest[i + 1..])),
            c => out.push(c),
        }
    }
    Err("unterminated string".into())
}

fn parse_device(s: &str) -> std::result::Result<UsbDeviceInfo, String> {
    let mut d = UsbDeviceInfo::default();
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
            "version" => d.device_version = hex16(&value)?,
            "bus" => d.bus = value.parse().map_err(|_| "bad bus")?,
            "addr" => d.address = value.parse().map_err(|_| "bad addr")?,
            "manufacturer" => d.manufacturer = Some(value),
            "product" => d.product = Some(value),
            "serial" => d.serial = Some(value),
            "interfaces" => {
                for item in value.split(',') {
                    let f: Vec<&str> = item.split(':').collect();
                    if f.len() != 4 {
                        return Err("bad interfaces field".into());
                    }
                    d.interfaces.push(crate::usb::InterfaceClass {
                        number: f[0].parse().map_err(|_| "bad interface number")?,
                        class: hex16(f[1])? as u8,
                        subclass: hex16(f[2])? as u8,
                        protocol: hex16(f[3])? as u8,
                    });
                }
            }
            other => return Err(format!("unknown device field {other}")),
        }
    }
    Ok(d)
}

/// Encodes interface descriptors for a fixture: per interface its
/// number, class, subclass, protocol and endpoint count, then per
/// endpoint the address, transfer type and packet size (little endian).
pub fn encode_interfaces(list: &[crate::usb::InterfaceDesc]) -> Vec<u8> {
    let mut out = Vec::new();
    for i in list {
        out.extend([i.class.number, i.class.class, i.class.subclass, i.class.protocol]);
        out.push(i.endpoints.len() as u8);
        for e in &i.endpoints {
            out.push(e.address);
            out.push(e.transfer_type);
            out.extend(e.max_packet_size.to_le_bytes());
        }
    }
    out
}

/// Decodes what [`encode_interfaces`] produced.
pub fn decode_interfaces(data: &[u8]) -> Result<Vec<crate::usb::InterfaceDesc>> {
    let bad = || Error::device("replay: malformed interface descriptors");
    let mut out = Vec::new();
    let mut pos = 0;
    while pos < data.len() {
        let head = data.get(pos..pos + 5).ok_or_else(bad)?;
        pos += 5;
        let mut desc = crate::usb::InterfaceDesc {
            class: crate::usb::InterfaceClass {
                number: head[0],
                class: head[1],
                subclass: head[2],
                protocol: head[3],
            },
            endpoints: Vec::new(),
        };
        for _ in 0..head[4] {
            let e = data.get(pos..pos + 4).ok_or_else(bad)?;
            pos += 4;
            desc.endpoints.push(crate::usb::EndpointDesc {
                address: e[0],
                transfer_type: e[1],
                max_packet_size: u16::from_le_bytes([e[2], e[3]]),
            });
        }
        out.push(desc);
    }
    Ok(out)
}

fn payload(p: &Payload) -> String {
    match p {
        Payload::Bytes(b) => hex(b),
        Payload::Digest(n, d) => {
            let mut s = format!("@{n}:");
            for b in d {
                let _ = write!(s, "{b:02x}");
            }
            s
        }
    }
}

fn parse_payload(s: &str) -> std::result::Result<Payload, String> {
    if let Some(rest) = s.strip_prefix('@') {
        let (n, d) = rest.split_once(':').ok_or("expected @LEN:SHA256")?;
        let n: usize = n.parse().map_err(|_| "bad digest length")?;
        let d = unhex(d)?;
        let d: [u8; 32] = d.try_into().map_err(|_| "digest must be 32 bytes")?;
        return Ok(Payload::Digest(n, d));
    }
    Ok(Payload::Bytes(unhex(s)?))
}

/// Hex pairs with `xx{N}` runs.
pub fn hex(data: &[u8]) -> String {
    if data.is_empty() {
        return "-".into();
    }
    let mut s = String::new();
    let mut i = 0;
    while i < data.len() {
        let b = data[i];
        let mut run = 1;
        while i + run < data.len() && data[i + run] == b {
            run += 1;
        }
        if run >= 4 {
            let _ = write!(s, "{b:02x}{{{run}}}");
            i += run;
        } else {
            let _ = write!(s, "{b:02x}");
            i += 1;
        }
    }
    s
}

/// The inverse of [`hex`].
pub fn unhex(s: &str) -> std::result::Result<Vec<u8>, String> {
    let mut out = Vec::new();
    if s == "-" {
        return Ok(out);
    }
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if i + 2 > bytes.len() {
            return Err("odd hex length".into());
        }
        let b = u8::from_str_radix(&s[i..i + 2], 16).map_err(|_| "bad hex")?;
        i += 2;
        if i < bytes.len() && bytes[i] == b'{' {
            let end = s[i..].find('}').map(|e| i + e).ok_or("unterminated run length")?;
            let n: usize = s[i + 1..end].parse().map_err(|_| "bad run length")?;
            out.extend(std::iter::repeat_n(b, n));
            i = end + 1;
        } else {
            out.push(b);
        }
    }
    Ok(out)
}

fn parse_call(s: &str) -> std::result::Result<Call, String> {
    let words: Vec<&str> = s.split_whitespace().collect();
    let kind = *words.first().ok_or("empty call")?;
    let field = |i: usize| words.get(i).copied().ok_or_else(|| format!("{kind}: missing field"));
    let h8 = |i: usize| {
        let w = field(i)?;
        u8::from_str_radix(w, 16).map_err(|_| format!("bad byte {w}"))
    };
    let h16 = |i: usize| {
        let w = field(i)?;
        u16::from_str_radix(w, 16).map_err(|_| format!("bad word {w}"))
    };
    let num = |i: usize| {
        let w = field(i)?;
        w.parse::<usize>().map_err(|_| format!("bad number {w}"))
    };
    let request = || {
        Ok::<_, String>(ControlRequest {
            // The direction bit is implied by the call.
            request_type: h8(1)? & 0x7f,
            request: h8(2)?,
            value: h16(3)?,
            index: h16(4)?,
        })
    };
    Ok(match kind {
        "config" => Call::SetConfiguration(num(1)? as u8),
        "claim" => Call::Claim(num(1)? as u8),
        "release" => Call::Release(num(1)? as u8),
        "reset" => Call::Reset,
        "mps" => Call::MaxPacketSize(h8(1)?),
        "interfaces" => Call::Interfaces,
        "ctl_out" => Call::ControlOut(request()?, parse_payload(field(5)?)?),
        "ctl_in" => Call::ControlIn(request()?, num(5)?),
        "bulk_out" => Call::BulkOut(h8(1)?, parse_payload(field(2)?)?),
        "bulk_in" => Call::BulkIn(h8(1)?, num(2)?),
        "int_out" => Call::IntOut(h8(1)?, parse_payload(field(2)?)?),
        "int_in" => Call::IntIn(h8(1)?, num(2)?),
        other => return Err(format!("unknown call {other}")),
    })
}

fn parse_outcome(s: &str, call: &Call) -> std::result::Result<Outcome, String> {
    let s = s.trim();
    if s == "ok" {
        return Ok(Outcome::Ok);
    }
    if let Some(rest) = s.strip_prefix("err ") {
        let (kind, msg) = rest.split_once(' ').unwrap_or((rest, ""));
        let msg = if msg.is_empty() { String::new() } else { unquote(msg.trim())?.0 };
        return Ok(Outcome::Err(kind.to_owned(), msg));
    }
    Ok(match call {
        Call::ControlIn(..) | Call::BulkIn(..) | Call::IntIn(..) | Call::Interfaces => {
            Outcome::Data(unhex(s)?)
        }
        _ => Outcome::Count(s.parse().map_err(|_| format!("bad count {s}"))?),
    })
}

/// A transport that logs every call to a [`Fixture`].
pub struct Recorder {
    inner: Box<dyn Transport>,
    fixture: Arc<Mutex<Fixture>>,
    file: Option<String>,
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Some(path) = &self.file {
            let text = self.fixture().to_text();
            match std::fs::write(path, text) {
                Ok(()) => log::info!("USB session recorded to {path}"),
                Err(e) => log::error!("cannot write the USB recording {path}: {e}"),
            }
        }
    }
}

/// A view of a [`Recorder`]'s fixture that outlives the recorder being
/// moved into a driver.
#[derive(Clone)]
pub struct RecordingHandle(Arc<Mutex<Fixture>>);

impl RecordingHandle {
    /// A copy of the recording so far.
    pub fn fixture(&self) -> Fixture {
        self.0.lock().expect("recording lock").clone()
    }
}

impl Recorder {
    /// Wraps `inner`.
    pub fn new(inner: Box<dyn Transport>) -> Self {
        let device = inner.info().clone();
        Recorder {
            inner,
            fixture: Arc::new(Mutex::new(Fixture { device, steps: Vec::new() })),
            file: None,
        }
    }

    /// Wraps `inner` and writes the fixture to `path` when dropped.
    pub fn to_file(inner: Box<dyn Transport>, path: impl Into<String>) -> Self {
        let mut r = Self::new(inner);
        r.file = Some(path.into());
        r
    }

    /// Wraps `inner` and returns a handle to read the recording later.
    pub fn with_handle(inner: Box<dyn Transport>) -> (Self, RecordingHandle) {
        let r = Self::new(inner);
        let h = RecordingHandle(Arc::clone(&r.fixture));
        (r, h)
    }

    /// A copy of the recording so far.
    pub fn fixture(&self) -> Fixture {
        self.fixture.lock().expect("recording lock").clone()
    }

    /// Stops recording and returns the fixture. The transport is
    /// dropped.
    pub fn finish(self) -> Fixture {
        self.fixture()
    }

    fn log<T>(&mut self, call: Call, r: Result<T>, ok: impl FnOnce(&T) -> Outcome) -> Result<T> {
        let outcome = match &r {
            Ok(v) => ok(v),
            Err(e) => Outcome::from_error(e),
        };
        self.fixture.lock().expect("recording lock").steps.push(Step { call, outcome });
        r
    }
}

impl Transport for Recorder {
    fn info(&self) -> &UsbDeviceInfo {
        self.inner.info()
    }

    fn set_configuration(&mut self, configuration: u8) -> Result<()> {
        let r = self.inner.set_configuration(configuration);
        self.log(Call::SetConfiguration(configuration), r, |()| Outcome::Ok)
    }

    fn claim_interface(&mut self, interface: u8) -> Result<()> {
        let r = self.inner.claim_interface(interface);
        self.log(Call::Claim(interface), r, |()| Outcome::Ok)
    }

    fn release_interface(&mut self, interface: u8) -> Result<()> {
        let r = self.inner.release_interface(interface);
        self.log(Call::Release(interface), r, |()| Outcome::Ok)
    }

    fn reset(&mut self) -> Result<()> {
        let r = self.inner.reset();
        self.log(Call::Reset, r, |()| Outcome::Ok)
    }

    fn max_packet_size(&mut self, endpoint: u8) -> Result<usize> {
        let r = self.inner.max_packet_size(endpoint);
        self.log(Call::MaxPacketSize(endpoint), r, |n| Outcome::Count(*n))
    }

    fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
        let r = self.inner.interfaces();
        self.log(Call::Interfaces, r, |d| Outcome::Data(encode_interfaces(d)))
    }

    fn control_out(&mut self, req: ControlRequest, data: &[u8], timeout: Duration) -> Result<()> {
        let r = self.inner.control_out(req, data, timeout);
        self.log(Call::ControlOut(req, Payload::of(data)), r, |()| Outcome::Ok)
    }

    fn control_in(
        &mut self,
        req: ControlRequest,
        len: usize,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let r = self.inner.control_in(req, len, timeout);
        self.log(Call::ControlIn(req, len), r, |d| Outcome::Data(d.clone()))
    }

    fn bulk_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize> {
        let r = self.inner.bulk_out(endpoint, data, timeout);
        self.log(Call::BulkOut(endpoint, Payload::of(data)), r, |n| Outcome::Count(*n))
    }

    fn bulk_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>> {
        let r = self.inner.bulk_in(endpoint, len, timeout);
        self.log(Call::BulkIn(endpoint, len), r, |d| Outcome::Data(d.clone()))
    }

    fn interrupt_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<usize> {
        let r = self.inner.interrupt_out(endpoint, data, timeout);
        self.log(Call::IntOut(endpoint, Payload::of(data)), r, |n| Outcome::Count(*n))
    }

    fn interrupt_in(&mut self, endpoint: u8, len: usize, timeout: Duration) -> Result<Vec<u8>> {
        let r = self.inner.interrupt_in(endpoint, len, timeout);
        self.log(Call::IntIn(endpoint, len), r, |d| Outcome::Data(d.clone()))
    }

    /// Each completed transfer is recorded as a `bulk_in` of the
    /// transfer size, so a stream replays through the same steps.
    fn bulk_in_stream(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        depth: usize,
    ) -> Result<Box<dyn BulkStream>> {
        let inner = self.inner.bulk_in_stream(endpoint, transfer_size, depth)?;
        Ok(Box::new(RecordedStream {
            inner,
            fixture: Arc::clone(&self.fixture),
            endpoint,
            transfer_size,
        }))
    }
}

fn is_read(call: &Call) -> bool {
    matches!(call, Call::ControlIn(..) | Call::BulkIn(..) | Call::IntIn(..))
}

/// A command a device answers through reads; claims and resets are not.
fn is_command(call: &Call) -> bool {
    matches!(call, Call::ControlOut(..) | Call::BulkOut(..) | Call::IntOut(..))
}

struct RecordedStream {
    inner: Box<dyn BulkStream>,
    fixture: Arc<Mutex<Fixture>>,
    endpoint: u8,
    transfer_size: usize,
}

impl BulkStream for RecordedStream {
    fn next(&mut self, timeout: Duration) -> Result<Vec<u8>> {
        let r = self.inner.next(timeout);
        let outcome = match &r {
            Ok(d) => Outcome::Data(d.clone()),
            Err(e) => Outcome::from_error(e),
        };
        self.fixture
            .lock()
            .expect("recording lock")
            .steps
            .push(Step { call: Call::BulkIn(self.endpoint, self.transfer_size), outcome });
        r
    }
}

/// A transport that answers from a [`Fixture`] and fails on any call
/// that differs from the recording, except for repeated polls (see
/// [`Replay::finish`] and the rules in the source).
pub struct Replay {
    shared: Arc<Mutex<ReplayState>>,
    device: UsbDeviceInfo,
}

impl Replay {
    /// Replays `fixture` from the start.
    pub fn new(fixture: Fixture) -> Self {
        let device = fixture.device.clone();
        Replay {
            shared: Arc::new(Mutex::new(ReplayState {
                fixture,
                pos: 0,
                last_cmd: None,
                repeat: None,
            })),
            device,
        }
    }

    /// Replays fixture text.
    pub fn from_text(text: &str) -> Result<Self> {
        Ok(Self::new(Fixture::parse(text)?))
    }

    fn state(&self) -> std::sync::MutexGuard<'_, ReplayState> {
        self.shared.lock().expect("replay lock")
    }

    /// The number of recorded steps not yet replayed.
    pub fn remaining(&self) -> usize {
        self.state().remaining()
    }

    /// Fails when steps remain, for the end of a test. Leftover
    /// repeats of the last read or of the last command group do not
    /// count.
    pub fn finish(&self) -> Result<()> {
        self.state().finish()
    }
}

/// The cursor over the recording, shared with streams.
struct ReplayState {
    fixture: Fixture,
    pos: usize,
    /// The last answered command (a non-read step).
    last_cmd: Option<usize>,
    /// While a command is repeated: the next recorded response.
    repeat: Option<usize>,
}

impl ReplayState {
    /// The number of recorded steps not yet replayed.
    pub fn remaining(&self) -> usize {
        self.fixture.steps.len() - self.pos
    }

    /// Fails when steps remain, for the end of a test. Leftover
    /// repeats of the last read or of the last command group do not
    /// count.
    pub fn finish(&self) -> Result<()> {
        let steps = &self.fixture.steps;
        let last = self.pos.checked_sub(1).map(|i| &steps[i].call);
        let cmd = self.last_cmd.map(|i| &steps[i].call);
        let mut i = self.pos;
        while i < steps.len() {
            let c = &steps[i].call;
            if is_read(c) && Some(c) == last {
                i += 1;
            } else if Some(c) == cmd {
                i += 1;
                while i < steps.len() && is_read(&steps[i].call) {
                    i += 1;
                }
            } else {
                return Err(Error::Device(format!(
                    "replay: {} recorded steps were not replayed, next is {:?}",
                    steps.len() - i,
                    c
                )));
            }
        }
        Ok(())
    }

    /// Answers `call` from the recording.
    ///
    /// Polling loops run on time, so a replay may poll more or fewer
    /// times than the recording did. A poll is either a read repeated
    /// on its own (a pin or status request) or a command followed by
    /// its reads (a FIFO protocol). When the call does not match the
    /// next step: a repeat of the last read is answered again; a repeat
    /// of the last command re-enters its recorded responses; and when
    /// the recording repeats the last read or command group while the
    /// replay moved on, those steps are skipped.
    fn step(&mut self, call: &Call) -> Result<Outcome> {
        if let Some(r) = self.repeat {
            if is_read(call) && r < self.pos && self.fixture.steps[r].call == *call {
                self.repeat = Some(r + 1);
                return Ok(self.fixture.steps[r].outcome.clone());
            }
            self.repeat = None;
        }
        loop {
            let steps = &self.fixture.steps;
            let last = self.pos.checked_sub(1).map(|i| &steps[i]);
            if let Some(step) = steps.get(self.pos)
                && step.call == *call
            {
                if is_command(call) {
                    self.last_cmd = Some(self.pos);
                }
                self.pos += 1;
                return Ok(step.outcome.clone());
            }
            if let Some(l) = last
                && is_read(call)
                && l.call == *call
            {
                return Ok(l.outcome.clone());
            }
            if let Some(lc) = self.last_cmd
                && steps[lc].call == *call
            {
                self.repeat = Some(lc + 1);
                return Ok(steps[lc].outcome.clone());
            }
            let Some(step) = steps.get(self.pos) else {
                return Err(Error::Device(format!(
                    "replay: past the end of the recording at {call:?}"
                )));
            };
            if let Some(l) = last
                && is_read(&step.call)
                && step.call == l.call
            {
                self.pos += 1;
                continue;
            }
            if let Some(lc) = self.last_cmd
                && step.call == steps[lc].call
            {
                self.pos += 1;
                while self.pos < steps.len() && is_read(&steps[self.pos].call) {
                    self.pos += 1;
                }
                continue;
            }
            return Err(Error::Device(format!(
                "replay: step {} expected {:?}, got {call:?}",
                self.pos + 1,
                step.call
            )));
        }
    }

    fn unit(&mut self, call: &Call) -> Result<()> {
        match self.step(call)? {
            Outcome::Ok => Ok(()),
            e @ Outcome::Err(..) => Err(e.to_error()),
            other => Err(Error::Device(format!("replay: {call:?} recorded {other:?}"))),
        }
    }

    fn count(&mut self, call: &Call) -> Result<usize> {
        match self.step(call)? {
            Outcome::Count(n) => Ok(n),
            e @ Outcome::Err(..) => Err(e.to_error()),
            other => Err(Error::Device(format!("replay: {call:?} recorded {other:?}"))),
        }
    }

    fn data(&mut self, call: &Call) -> Result<Vec<u8>> {
        match self.step(call)? {
            Outcome::Data(d) => Ok(d),
            e @ Outcome::Err(..) => Err(e.to_error()),
            other => Err(Error::Device(format!("replay: {call:?} recorded {other:?}"))),
        }
    }
}

impl Transport for Replay {
    fn info(&self) -> &UsbDeviceInfo {
        &self.device
    }

    fn set_configuration(&mut self, configuration: u8) -> Result<()> {
        self.state().unit(&Call::SetConfiguration(configuration))
    }

    fn claim_interface(&mut self, interface: u8) -> Result<()> {
        self.state().unit(&Call::Claim(interface))
    }

    fn release_interface(&mut self, interface: u8) -> Result<()> {
        self.state().unit(&Call::Release(interface))
    }

    fn reset(&mut self) -> Result<()> {
        self.state().unit(&Call::Reset)
    }

    fn max_packet_size(&mut self, endpoint: u8) -> Result<usize> {
        self.state().count(&Call::MaxPacketSize(endpoint))
    }

    fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
        decode_interfaces(&self.state().data(&Call::Interfaces)?)
    }

    fn control_out(&mut self, req: ControlRequest, data: &[u8], _timeout: Duration) -> Result<()> {
        self.state().unit(&Call::ControlOut(req, data.to_vec().into()))
    }

    fn control_in(
        &mut self,
        req: ControlRequest,
        len: usize,
        _timeout: Duration,
    ) -> Result<Vec<u8>> {
        self.state().data(&Call::ControlIn(req, len))
    }

    fn bulk_out(&mut self, endpoint: u8, data: &[u8], _timeout: Duration) -> Result<usize> {
        self.state().count(&Call::BulkOut(endpoint, data.to_vec().into()))
    }

    fn bulk_in(&mut self, endpoint: u8, len: usize, _timeout: Duration) -> Result<Vec<u8>> {
        self.state().data(&Call::BulkIn(endpoint, len))
    }

    fn interrupt_out(&mut self, endpoint: u8, data: &[u8], _timeout: Duration) -> Result<usize> {
        self.state().count(&Call::IntOut(endpoint, data.to_vec().into()))
    }

    fn interrupt_in(&mut self, endpoint: u8, len: usize, _timeout: Duration) -> Result<Vec<u8>> {
        self.state().data(&Call::IntIn(endpoint, len))
    }

    /// The stream shares the recording cursor: its reads are the
    /// `bulk_in` steps of the transfer size that follow.
    fn bulk_in_stream(
        &mut self,
        endpoint: u8,
        transfer_size: usize,
        _depth: usize,
    ) -> Result<Box<dyn BulkStream>> {
        Ok(Box::new(ReplayStream { replay: Arc::clone(&self.shared), endpoint, transfer_size }))
    }
}

struct ReplayStream {
    replay: Arc<Mutex<ReplayState>>,
    endpoint: u8,
    transfer_size: usize,
}

impl BulkStream for ReplayStream {
    fn next(&mut self, _timeout: Duration) -> Result<Vec<u8>> {
        self.replay
            .lock()
            .expect("replay lock")
            .data(&Call::BulkIn(self.endpoint, self.transfer_size))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEXT: &str = "# oxrok usb fixture 1\n\
        device vid=0403 pid=dc48 version=0400 bus=2 addr=7 manufacturer=\"Intronix\" product=\"LogicPort \\\"LA\\\"\" serial=\"80806096\"\n\
        claim 0 -> ok\n\
        mps 81 -> 64\n\
        ctl_out 40 00 0000 0001 - -> ok\n\
        ctl_in 40 0c 0000 0001 1 -> 18\n\
        bulk_out 02 00{5}aa -> 6\n\
        bulk_in 81 64 -> 3160\n\
        bulk_in 81 64 -> err timeout\n\
        reset -> err device \"device disconnected\"\n";

    #[test]
    fn hex_runs() {
        assert_eq!(hex(&[]), "-");
        assert_eq!(hex(&[0, 0, 0, 0, 0xaa]), "00{4}aa");
        assert_eq!(hex(&[0, 0, 0, 0xaa]), "000000aa");
        assert_eq!(unhex("00{4}aa").unwrap(), [0, 0, 0, 0, 0xaa]);
        assert_eq!(unhex("00{59}316000{62}").unwrap().len(), 59 + 2 + 62);
        assert!(unhex("00{4").is_err());
        assert_eq!(unhex("-").unwrap(), Vec::<u8>::new());
        assert!(unhex("0").is_err());
        assert!(unhex("zz").is_err());
    }

    #[test]
    fn parse_and_format_round_trip() {
        let f = Fixture::parse(TEXT).unwrap();
        assert_eq!(f.device.product.as_deref(), Some("LogicPort \"LA\""));
        assert_eq!(f.device.conn(), "2.7");
        assert_eq!(f.steps.len(), 8);
        assert_eq!(f.steps[4].call, Call::BulkOut(2, vec![0, 0, 0, 0, 0, 0xaa].into()));
        assert_eq!(f.to_text(), TEXT);
        assert_eq!(Fixture::parse(&f.to_text()).unwrap(), f);
    }

    #[test]
    fn replay_follows_the_recording() {
        let mut r = Replay::from_text(TEXT).unwrap();
        let t = Duration::from_millis(10);
        r.claim_interface(0).unwrap();
        assert_eq!(r.max_packet_size(0x81).unwrap(), 64);
        r.control_out(ControlRequest::vendor(0, 0, 1), &[], t).unwrap();
        assert_eq!(r.control_in(ControlRequest::vendor(0x0c, 0, 1), 1, t).unwrap(), [0x18]);
        assert_eq!(r.bulk_out(2, &[0, 0, 0, 0, 0, 0xaa], t).unwrap(), 6);
        assert_eq!(r.bulk_in(0x81, 64, t).unwrap(), [0x31, 0x60]);
        assert!(matches!(r.bulk_in(0x81, 64, t), Err(Error::Timeout)));
        assert_eq!(r.remaining(), 1);
        assert!(r.finish().is_err());
        let e = r.reset().unwrap_err().to_string();
        assert_eq!(e, "device error: device disconnected");
        r.finish().unwrap();
        assert!(r.reset().unwrap_err().to_string().contains("past the end"));
    }

    #[test]
    fn interrupt_transfers_round_trip_and_replay() {
        let text = "# oxrok usb fixture 1\n\
            device vid=04d8 pid=0033 version=0000 bus=0 addr=0\n\
            claim 0 -> ok\n\
            int_out 01 b9ad -> 2\n\
            int_in 81 64 -> 0102\n\
            int_in 81 64 -> err timeout\n\
            release 0 -> ok\n";
        let f = Fixture::parse(text).unwrap();
        assert_eq!(f.steps[1].call, Call::IntOut(1, vec![0xb9, 0xad].into()));
        assert_eq!(f.steps[2].call, Call::IntIn(0x81, 64));
        assert_eq!(f.to_text(), text);
        let t = Duration::from_millis(10);
        let mut r = Replay::from_text(text).unwrap();
        r.claim_interface(0).unwrap();
        assert_eq!(r.interrupt_out(1, &[0xb9, 0xad], t).unwrap(), 2);
        assert_eq!(r.interrupt_in(0x81, 64, t).unwrap(), [1, 2]);
        assert!(matches!(r.interrupt_in(0x81, 64, t), Err(Error::Timeout)));
        r.release_interface(0).unwrap();
        r.finish().unwrap();
        // A recording of the replay writes the same fixture.
        let mut rec = Recorder::new(Box::new(Replay::from_text(text).unwrap()));
        rec.claim_interface(0).unwrap();
        rec.interrupt_out(1, &[0xb9, 0xad], t).unwrap();
        rec.interrupt_in(0x81, 64, t).unwrap();
        let _ = rec.interrupt_in(0x81, 64, t);
        rec.release_interface(0).unwrap();
        assert_eq!(rec.finish().steps, f.steps);
    }

    #[test]
    fn replay_tolerates_polling_loops() {
        let text = "# oxrok usb fixture 1\n\
            device vid=0403 pid=dc48\n\
            ctl_in 40 0c 0000 0000 1 -> 00\n\
            ctl_in 40 0c 0000 0000 1 -> 00\n\
            ctl_in 40 0c 0000 0000 1 -> 10\n\
            ctl_in 40 0c 0000 0000 1 -> 10\n\
            reset -> ok\n";
        let t = Duration::ZERO;
        let pins = ControlRequest::vendor(0x0c, 0, 0);
        // Fewer polls than recorded: the leftover identical polls are skipped.
        let mut r = Replay::from_text(text).unwrap();
        assert_eq!(r.control_in(pins, 1, t).unwrap(), [0]);
        assert_eq!(r.control_in(pins, 1, t).unwrap(), [0]);
        assert_eq!(r.control_in(pins, 1, t).unwrap(), [0x10]);
        r.reset().unwrap();
        r.finish().unwrap();
        // More polls than recorded: the last answer repeats.
        let mut r = Replay::from_text(text).unwrap();
        for _ in 0..4 {
            r.control_in(pins, 1, t).unwrap();
        }
        assert_eq!(r.control_in(pins, 1, t).unwrap(), [0x10]);
        assert_eq!(r.control_in(pins, 1, t).unwrap(), [0x10]);
        r.reset().unwrap();
        r.finish().unwrap();
        // Past the end, a repeated poll still answers; a new call fails.
        assert!(r.reset().is_err());
        // A command with its responses, polled more and fewer times.
        let text = "# oxrok usb fixture 1\n\
            device vid=0403 pid=dc48\n\
            bulk_out 02 c2050000 -> 4\n\
            bulk_in 81 64 -> 01\n\
            bulk_out 02 c2050000 -> 4\n\
            bulk_in 81 64 -> 02\n\
            bulk_out 02 c2050000 -> 4\n\
            bulk_in 81 64 -> 02\n\
            bulk_out 02 c3000000 -> 4\n\
            bulk_in 81 64 -> 03\n";
        let poll = |r: &mut Replay| {
            r.bulk_out(2, &[0xc2, 5, 0, 0], t).unwrap();
            r.bulk_in(0x81, 64, t).unwrap()
        };
        let mut r = Replay::from_text(text).unwrap();
        assert_eq!(poll(&mut r), [1]);
        assert_eq!(poll(&mut r), [2]);
        assert_eq!(poll(&mut r), [2]);
        assert_eq!(poll(&mut r), [2]);
        assert_eq!(poll(&mut r), [2]);
        r.bulk_out(2, &[0xc3, 0, 0, 0], t).unwrap();
        assert_eq!(r.bulk_in(0x81, 64, t).unwrap(), [3]);
        r.finish().unwrap();
        let mut r = Replay::from_text(text).unwrap();
        assert_eq!(poll(&mut r), [1]);
        r.bulk_out(2, &[0xc3, 0, 0, 0], t).unwrap();
        assert_eq!(r.bulk_in(0x81, 64, t).unwrap(), [3]);
        r.finish().unwrap();
    }

    #[test]
    fn replay_rejects_a_different_call() {
        let mut r = Replay::from_text(TEXT).unwrap();
        let e = r.claim_interface(1).unwrap_err().to_string();
        assert!(e.contains("step 1 expected Claim(0), got Claim(1)"), "{e}");
    }

    #[test]
    fn recorder_logs_what_replay_answers() {
        let inner = Replay::from_text(TEXT).unwrap();
        let mut rec = Recorder::new(Box::new(inner));
        let t = Duration::from_millis(10);
        rec.claim_interface(0).unwrap();
        assert_eq!(rec.max_packet_size(0x81).unwrap(), 64);
        rec.control_out(ControlRequest::vendor(0, 0, 1), &[], t).unwrap();
        let f = rec.finish();
        assert_eq!(f.steps.len(), 3);
        assert_eq!(f.steps[1].outcome, Outcome::Count(64));
        assert!(f.to_text().starts_with(MAGIC));
    }

    #[test]
    fn large_payloads_are_hashed() {
        let big = vec![0x5a; HASH_ABOVE + 1];
        let p = Payload::of(&big);
        assert!(matches!(p, Payload::Digest(n, _) if n == HASH_ABOVE + 1));
        assert_eq!(p, Payload::Bytes(big.clone()));
        assert_ne!(p, Payload::Bytes(vec![0x5a; HASH_ABOVE]));
        let text = format!(
            "# oxrok usb fixture 1\ndevice vid=0403 pid=dc48 version=0000 bus=0 addr=0\nbulk_out 02 {} -> {}\n",
            payload(&p),
            big.len()
        );
        let f = Fixture::parse(&text).unwrap();
        assert_eq!(f.to_text(), text);
        let mut r = Replay::new(f);
        assert_eq!(r.bulk_out(2, &big, Duration::ZERO).unwrap(), big.len());
    }

    #[test]
    fn recorder_writes_a_file_on_drop() {
        let path = std::env::temp_dir().join(format!("oxrok-rec-{}.txt", std::process::id()));
        {
            let inner = Replay::from_text(TEXT).unwrap();
            let mut rec = Recorder::to_file(Box::new(inner), path.to_str().unwrap());
            rec.claim_interface(0).unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(text.contains("claim 0 -> ok\n"), "{text}");
    }

    #[test]
    fn parse_errors_name_the_line() {
        assert!(Fixture::parse("").is_err());
        assert!(Fixture::parse("# other\n").is_err());
        let e = Fixture::parse("# oxrok usb fixture 1\ndevice vid=0403 pid=1\nclaim x -> ok\n")
            .unwrap_err()
            .to_string();
        assert!(e.contains("fixture line 3"), "{e}");
        assert!(Fixture::parse("# oxrok usb fixture 1\nclaim 0 -> ok\n").is_err());
    }
}
