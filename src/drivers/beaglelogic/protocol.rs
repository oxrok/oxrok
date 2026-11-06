// Counterpart in sigrok:
//   libsigrok/src/hardware/beaglelogic/beaglelogic_tcp.c,
//   libsigrok/src/hardware/beaglelogic/protocol.c,
//   libsigrok/src/hardware/beaglelogic/protocol.h,
//   Copyright (C) 2017 Kumar Abhishek <abhishek@theembeddedkitchen.net>,
//   Copyright (C) 2013 Martin Ling <martin-sigrok@earth.li>,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   Copyright (C) 2014-2017 Kumar Abhishek <abhishek@theembeddedkitchen.net>,
//   with commits by Gerhard Sittig, Uwe Hermann, Michał Janiszewski,
//   Aurelien Jacobs, Bert Vermeulen, Daniel Elstner, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire of a `BeagleLogic`'s TCP server, and the rules of a capture over it.
//!
//! One line per command, cut at `\n`. `samplerate`, `sampleunit`, `bufunitsize` and
//! `memalloc` ask, and with a value `samplerate`, `sampleunit` and `triggerflags` set:
//! `OK` or `ERR`, with `\r\n`. `version` names it. `get` starts a stream of samples on
//! this same connection, and `close` ends it; neither has an answer. A one-shot stream
//! ends after `memalloc` bytes with no end mark and no close, so the client counts.
//!
//! This is read from the `BeagleLogic` repository (`tcp-server-go/server.go`,
//! `kernel/beaglelogic.c`), with no device: see `docs/DEVICES.md`. Nothing tags an
//! answer, so a late one would pass for the next one's: [`Link`] connects again rather
//! than take that.

use crate::core::{Error, LogicData, Packet, Result, SoftTriggerLogic};
use crate::serial::Port;
use std::time::Duration;

/// `TCP_BUFFER_SIZE`: the most a capture reads at a time.
pub const TCP_BUFFER_SIZE: usize = 128 * 1024;
/// `coreclockfreq / 2` in the kernel module: the rate at a divisor of 1.
pub const TOP_RATE: u32 = 100_000_000;
/// `devc->read_timeout`: an answer later than this is an error, in C's words.
pub const LATE: Duration = Duration::from_secs(1);
/// How long an answer is waited for, where C's `recv` has no end.
pub const NO_ANSWER: Duration = Duration::from_secs(3);
/// How long a line with no end is given after its last byte.
pub const LINE_TAIL: Duration = Duration::from_millis(100);
/// What a read waits at a time, so that a stop gets in.
pub const SLICE: Duration = Duration::from_millis(50);
/// `beaglelogic_tcp_drain`'s `select`.
pub const DRAIN_QUIET: Duration = Duration::from_millis(25);
/// How long a drain goes on against a device that doesn't stop.
pub const DRAIN_MOST: Duration = Duration::from_secs(2);
/// The longest answer there is: a version.
const ANSWER_MOST: usize = 256;
/// What a pre-trigger buffer may take, whatever a limit and a ratio say.
const PRE_TRIGGER_MOST: u64 = 1 << 28;

/// The host and port of `tcp-raw/HOST/PORT`, or C's message for what isn't one:
/// `g_strsplit`, then the fields' checks in `scan`. A first field of `tcp...` in any
/// case is enough, and fields behind the port mean nothing.
pub fn parse_conn(conn: &str) -> std::result::Result<(&str, &str), &'static str> {
    let mut fields = conn.split('/');
    let first = fields.next().unwrap_or_default();
    let (Some(host), Some(port)) = (fields.next(), fields.next()) else {
        return Err("Invalid Parameters.");
    };
    if first.get(..3).is_none_or(|t| !t.eq_ignore_ascii_case("tcp")) {
        return Err("Only TCP (tcp-raw) protocol is currently supported.");
    }
    Ok((host, port))
}

/// `beaglelogic_tcp_detect`.
pub fn is_beaglelogic(version: &str) -> bool {
    version.get(..11).is_some_and(|t| t.eq_ignore_ascii_case("BeagleLogic"))
}

/// What a set's answer must start with, in any case.
pub fn is_ok(answer: &str) -> bool {
    answer.get(..2).is_some_and(|t| t.eq_ignore_ascii_case("ok"))
}

/// The number in an answer. The server prints a kernel's `%d`, so digits and nothing
/// else, up to an `int`. C's `sr_atoi` also takes a sign, leading and trailing white
/// space, and an empty answer as 0.
pub fn parse_number(answer: &str) -> Option<u32> {
    if answer.is_empty() || !answer.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    answer.parse().ok().filter(|n| i32::try_from(*n).is_ok())
}

/// The sample unit in an answer: the server only ever says 0 or 1.
pub fn parse_unit(answer: &str) -> Option<Unit> {
    match answer {
        "0" => Some(Unit::Sixteen),
        "1" => Some(Unit::Eight),
        _ => None,
    }
}

/// The module's `sampleunit`: 1 is 8 bits, the rest 16.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    /// `BL_SAMPLEUNIT_16_BITS`.
    Sixteen,
    /// `BL_SAMPLEUNIT_8_BITS`.
    Eight,
}

impl Unit {
    /// `SAMPLEUNIT_TO_BYTES`.
    pub fn bytes(self) -> usize {
        match self {
            Unit::Sixteen => 2,
            Unit::Eight => 1,
        }
    }

    /// What the wire says for it.
    pub fn number(self) -> u32 {
        match self {
            Unit::Sixteen => 0,
            Unit::Eight => 1,
        }
    }
}

/// What the module keeps of a rate it took: a whole divisor of [`TOP_RATE`]
/// (`beaglelogic_set_samplerate`, `kernel/beaglelogic.c`). `None` for a rate it
/// refuses.
pub fn module_rate(asked: u32) -> Option<u32> {
    (1..=TOP_RATE).contains(&asked).then(|| TOP_RATE / (TOP_RATE / asked))
}

/// The warning for a limit past the buffer, as `config_set` says it.
pub fn truncated_message(buffersize: u32, unit: Unit) -> String {
    let msamples = u64::from(buffersize) / (unit.bytes() as u64 * 1_000_000);
    format!(
        "Please use 'echo <size in bytes> > /sys/devices/virtual/misc/beaglelogic/memalloc' to \
         increase the buffer size, this capture is now truncated to {msamples} Msamples"
    )
}

/// Whether a limit is past the buffer. C multiplies in 64 bits and wraps, so a limit
/// near the top of them passed the check.
pub fn past_buffer(limit: u64, unit: Unit, buffersize: u32) -> bool {
    u128::from(limit) * unit.bytes() as u128 > u128::from(buffersize)
}

/// How long a buffer takes to fill, which is how long the stream may say nothing: the
/// module hands out only whole buffers. `BUFUNIT_TIMEOUT_MS`'s reckoning, with the
/// sample's width, which C leaves out.
pub fn buffer_time(bufunitsize: u32, samplerate: u32, unit: Unit) -> Duration {
    let per_second = u64::from(samplerate) * unit.bytes() as u64;
    if per_second == 0 {
        return Duration::ZERO;
    }
    Duration::from_secs_f64(f64::from(bufunitsize) / per_second as f64)
}

/// The samples to keep ahead of a trigger: `capture_ratio * limit / 100` with no wrap,
/// and no more than the stream can hold or a buffer should. A continuous capture has
/// no limit to take a share of, and C's `UINT64_MAX` gives a number that means nothing.
pub fn pre_trigger_samples(ratio: u64, limit: Option<u64>, unit: Unit, buffersize: u32) -> usize {
    let Some(limit) = limit else { return 0 };
    let wanted = u128::from(ratio) * u128::from(limit) / 100;
    let bytes = unit.bytes() as u64;
    let most = u64::from(buffersize).min(PRE_TRIGGER_MOST) / bytes;
    usize::try_from(wanted.min(u128::from(most))).unwrap_or(0)
}

/// The bounds on a silent stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bounds {
    /// Until the first sample: a limited capture fails here, and any other warns.
    pub first: Duration,
    /// After a sample, a warning.
    pub warn: Duration,
    /// After a sample, a limited capture fails.
    pub fail: Duration,
}

impl Bounds {
    /// For a device that fills a buffer in `buffer`: twice that, and a margin. C's
    /// `recv` has no bound at all. A slow rate gives days, and C's `BUFUNIT_TIMEOUT_MS`
    /// counts on the same pace.
    pub fn for_buffer(buffer: Duration) -> Bounds {
        let paced = buffer.saturating_mul(2).saturating_add(Duration::from_secs(30));
        Bounds { first: paced, warn: paced, fail: paced.max(Duration::from_secs(600)) }
    }
}

/// What a silence comes to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Nothing yet.
    Wait,
    /// Say this, once, and go on.
    Warn(String),
    /// The capture has failed, for this reason.
    Fail(String),
}

/// How long the stream has said nothing, counted from what the reads waited: what a
/// stall or a slow consumer took is not the device's. Each read counts at most
/// `cap`, so stops of the process can't add up to a bound, and it only takes a longer
/// silence to get there on a busy machine.
#[derive(Clone, Debug)]
pub struct Quiet {
    bounds: Bounds,
    /// A limit makes an end to wait for.
    limited: bool,
    cap: Duration,
    silent: Duration,
    any: bool,
    warned: bool,
}

impl Quiet {
    /// A capture's start.
    pub fn new(bounds: Bounds, limited: bool, cap: Duration) -> Quiet {
        Quiet { bounds, limited, cap, silent: Duration::ZERO, any: false, warned: false }
    }

    /// Bytes came. What to say if a warning has gone out.
    pub fn heard(&mut self) -> Option<&'static str> {
        self.any = true;
        self.silent = Duration::ZERO;
        std::mem::take(&mut self.warned).then_some("The device is sending again.")
    }

    /// A read waited `waited` and found nothing.
    pub fn empty(&mut self, waited: Duration) -> Verdict {
        self.silent += waited.min(self.cap);
        let (warn_at, fail_at) = if self.any {
            (self.bounds.warn, self.bounds.fail)
        } else {
            (self.bounds.first, self.bounds.first)
        };
        if self.limited && self.silent >= fail_at {
            return Verdict::Fail(self.message());
        }
        if self.silent >= warn_at && !self.warned {
            self.warned = true;
            return Verdict::Warn(self.message());
        }
        Verdict::Wait
    }

    fn message(&self) -> String {
        let seconds = self.silent.as_secs();
        if self.any {
            format!("The device has sent no data for {seconds} s")
        } else {
            format!("The device has sent no data in {seconds} s, since it was asked to start")
        }
    }
}

/// Why a capture has what it wanted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Done {
    /// Not yet.
    No,
    /// The limit.
    Limit,
    /// A one-shot stream is `memalloc` bytes, and nothing marks its end.
    Buffer,
}

/// What a capture does with the bytes: `beaglelogic_tcp_receive_data`, without a read.
///
/// Where it differs from C, whose lines are `protocol.c:150-188`:
///
/// - Whole samples only. A read that ends inside a 16-bit sample keeps its byte for
///   the next, where C sends it, an output drops it and all that follows is a byte out.
/// - The limit counts samples sent. C also counts what it threw away ahead of the
///   trigger, so a late trigger comes to fewer samples or none, with exit 0, and it
///   leaves the pre-trigger samples out of the sum, so there are too many.
/// - The trigger's sample width is the stream's. C's is from all channels whether
///   enabled or not (`logic_channel_unitsize`), so 8 of 14 channels step two bytes at
///   a time, see half the samples, and put the trigger at half its place.
#[derive(Debug)]
pub struct Acquire {
    unit: usize,
    limit: Option<u64>,
    /// Only for a one-shot stream.
    buffersize: Option<u64>,
    stl: Option<SoftTriggerLogic>,
    carry: Vec<u8>,
    sent: u64,
    offset: u64,
}

impl Acquire {
    /// `limit` and `buffersize` as a one-shot capture has them; `stl` for a trigger.
    pub fn new(
        unit: Unit,
        limit: Option<u64>,
        buffersize: Option<u64>,
        stl: Option<SoftTriggerLogic>,
    ) -> Acquire {
        Acquire {
            unit: unit.bytes(),
            limit,
            buffersize,
            stl,
            carry: Vec::new(),
            sent: 0,
            offset: 0,
        }
    }

    /// Whether a trigger is still waited for.
    pub fn waiting(&self) -> bool {
        self.stl.is_some()
    }

    /// Bytes the stream gave so far.
    pub fn offset(&self) -> u64 {
        self.offset
    }

    fn logic(&mut self, data: &[u8], out: &mut Vec<Packet>) {
        let left = self.limit.map_or(usize::MAX, |l| {
            let samples = l.saturating_sub(self.sent);
            usize::try_from(samples.saturating_mul(self.unit as u64)).unwrap_or(usize::MAX)
        });
        let data = &data[..data.len().min(left)];
        if !data.is_empty() {
            self.sent += (data.len() / self.unit) as u64;
            out.push(Packet::Logic(LogicData { unit_size: self.unit, data: data.to_vec() }));
        }
    }

    /// The packets `bytes` make, and whether that's the end.
    pub fn take(&mut self, bytes: &[u8]) -> (Vec<Packet>, Done) {
        self.offset += bytes.len() as u64;
        let mut whole = std::mem::take(&mut self.carry);
        whole.extend_from_slice(bytes);
        let usable = whole.len() - whole.len() % self.unit;
        self.carry = whole[usable..].to_vec();
        whole.truncate(usable);

        let mut out = Vec::new();
        let mut from = 0;
        if let Some(stl) = &mut self.stl {
            match stl.check(&whole) {
                Some((at, pre)) => {
                    self.stl = None;
                    from = at * self.unit;
                    for packet in &pre {
                        self.logic(packet, &mut out);
                    }
                    out.push(Packet::Trigger);
                }
                None => from = whole.len(),
            }
        }
        self.logic(&whole[from..], &mut out);

        let done = if self.limit.is_some_and(|l| self.sent >= l) {
            Done::Limit
        } else if self.buffersize.is_some_and(|b| self.offset >= b) {
            Done::Buffer
        } else {
            Done::No
        };
        (out, done)
    }
}

/// What opens a connection.
pub type Dial = Box<dyn FnMut() -> Result<Box<dyn Port>> + Send>;

/// The times of an exchange.
#[derive(Clone, Copy, Debug)]
pub struct Times {
    /// [`LATE`].
    pub late: Duration,
    /// [`NO_ANSWER`].
    pub no_answer: Duration,
    /// [`LINE_TAIL`].
    pub line_tail: Duration,
    /// [`SLICE`].
    pub slice: Duration,
    /// [`DRAIN_QUIET`].
    pub drain_quiet: Duration,
    /// [`DRAIN_MOST`].
    pub drain_most: Duration,
}

impl Default for Times {
    fn default() -> Self {
        Times {
            late: LATE,
            no_answer: NO_ANSWER,
            line_tail: LINE_TAIL,
            slice: SLICE,
            drain_quiet: DRAIN_QUIET,
            drain_most: DRAIN_MOST,
        }
    }
}

/// The connection to a server, and whether it can be trusted.
pub struct Link {
    dial: Dial,
    port: Option<Box<dyn Port>>,
    /// Something may still come on this connection that no command will answer: an
    /// answer past its wait, or what the stream sends after a `close` when the module
    /// is slow to fill its buffer. The next command goes on a new connection, which
    /// nothing old can reach, and the server serves one at a time.
    stale: bool,
    pub(crate) times: Times,
}

impl std::fmt::Debug for Link {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Link").field("stale", &self.stale).finish_non_exhaustive()
    }
}

impl Link {
    /// A link that isn't connected.
    pub fn new(dial: Dial) -> Link {
        Link { dial, port: None, stale: false, times: Times::default() }
    }

    /// Connects: `beaglelogic_tcp_open`.
    pub fn connect(&mut self) -> Result<()> {
        self.close();
        self.port = Some((self.dial)()?);
        self.stale = false;
        Ok(())
    }

    /// `beaglelogic_close`.
    pub fn close(&mut self) {
        self.port = None;
    }

    /// Whether a connection is up, as far as anything has shown.
    pub fn is_connected(&self) -> bool {
        self.port.is_some()
    }

    /// What it was made with, for the next open.
    pub fn into_dial(self) -> Dial {
        self.dial
    }

    /// Whether the next command goes on a new connection.
    pub fn is_stale(&self) -> bool {
        self.stale
    }

    /// See [`Link::stale`].
    pub fn mark_stale(&mut self) {
        self.stale = true;
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("The connection is closed"))
    }

    /// Sends a command, on a new connection if this one can't be trusted.
    /// `beaglelogic_tcp_send_cmd`.
    pub fn send(&mut self, command: &str) -> Result<()> {
        if self.stale {
            log::debug!("Connecting again: something may still come on the old connection.");
            self.connect()?;
        }
        log::trace!("Sending command: {command:?}.");
        let line = format!("{command}\n");
        match self.port()?.write(line.as_bytes()) {
            Err(Error::Disconnected) => {
                self.close();
                Err(closed_error())
            }
            other => other,
        }
    }

    /// Reads one answer: `beaglelogic_tcp_get_string`, whose single `recv` takes a
    /// piece for the whole, and has no end while the connection stays up.
    ///
    /// Times are taken ahead of a read, so that an answer that lies ready is an answer
    /// on time whatever a stall did after it was looked for, and no answer has to have
    /// been looked for after the wait is up.
    fn answer(&mut self, command: &str) -> Result<String> {
        let times = self.times;
        let began = std::time::Instant::now();
        let mut line = Vec::new();
        let mut heard = None;
        let mut since_byte = Duration::ZERO;
        let mut buf = [0u8; ANSWER_MOST];
        loop {
            let looked = std::time::Instant::now();
            match self.port()?.read(&mut buf, times.slice) {
                Ok(0) => since_byte += looked.elapsed().min(times.slice.saturating_mul(2)),
                Ok(n) => {
                    line.extend_from_slice(&buf[..n]);
                    heard.get_or_insert(looked.duration_since(began));
                    since_byte = Duration::ZERO;
                }
                Err(Error::Disconnected) => {
                    self.close();
                    return Err(closed_error());
                }
                Err(e) => return Err(e),
            }
            if let Some(end) = line.iter().position(|b| *b == b'\n') {
                if end + 1 < line.len() {
                    log::debug!("{} bytes came behind an answer.", line.len() - end - 1);
                    self.stale = true;
                }
                line.truncate(end + 1);
                break;
            }
            if line.len() >= ANSWER_MOST {
                self.stale = true;
                return Err(Error::device(format!("No usable answer to '{command}'")));
            }
            if !line.is_empty() && since_byte >= times.line_tail {
                break;
            }
            if line.is_empty() && looked.duration_since(began) >= times.no_answer {
                self.stale = true;
                return Err(Error::device(no_answer_message(command)));
            }
        }
        if heard.is_some_and(|at| at > times.late) {
            log::error!("Timed out waiting for response.");
            return Err(Error::Timeout);
        }
        Ok(strip_line(&line))
    }

    /// A command and its answer.
    pub fn ask(&mut self, command: &str) -> Result<String> {
        self.send(command)?;
        let answer = self.answer(command)?;
        log::trace!("Got response: {answer:?}.");
        Ok(answer)
    }

    /// A number's question, with what isn't one refused.
    pub fn ask_number(&mut self, command: &str) -> Result<u32> {
        let answer = self.ask(command)?;
        parse_number(&answer).ok_or_else(|| Error::device(no_number_message(command, &answer)))
    }

    /// A setting: `OK`, or an error saying what became of it.
    pub fn set(&mut self, command: &str, value: u32) -> Result<()> {
        let line = format!("{command} {value}");
        let answer = match self.ask(&line) {
            Ok(answer) => answer,
            // It may have got there, with its answer lost or late.
            Err(e) => {
                if self.is_connected() {
                    log::warn!("'{line}' got no answer in time, and may have taken effect.");
                }
                return Err(e);
            }
        };
        if is_ok(&answer) { Ok(()) } else { Err(Error::device(refused_message(&line))) }
    }

    /// `beaglelogic_tcp_drain`: what has come goes, until the line has been quiet for
    /// a moment. C's has no end at a closed connection, as `select` keeps saying
    /// there's something to read and `recv` keeps saying 0. This one also has an end
    /// against a stream that doesn't stop. Whether the connection is gone.
    pub fn drain(&mut self) -> Result<bool> {
        let times = self.times;
        let began = std::time::Instant::now();
        let mut quiet = Duration::ZERO;
        let mut buf = [0u8; 1024];
        let mut drained = 0usize;
        while quiet < times.drain_quiet && began.elapsed() < times.drain_most {
            let looked = std::time::Instant::now();
            let wait = times.drain_quiet.saturating_sub(quiet).min(times.slice);
            match self.port()?.read(&mut buf, wait) {
                Ok(0) => quiet += looked.elapsed().min(wait.saturating_mul(2)),
                Ok(n) => drained += n,
                Err(Error::Disconnected) => {
                    log::trace!("Drained {drained} bytes of data.");
                    self.close();
                    return Ok(true);
                }
                Err(e) => return Err(e),
            }
        }
        log::trace!("Drained {drained} bytes of data.");
        Ok(false)
    }

    /// A read of the stream.
    pub fn read_stream(&mut self, buf: &mut [u8], wait: Duration) -> Result<usize> {
        match self.port()?.read(buf, wait) {
            Err(Error::Disconnected) => {
                self.close();
                Err(closed_error())
            }
            other => other,
        }
    }
}

/// The line end off an answer, as C takes it: a `\n`, then a `\r`.
fn strip_line(line: &[u8]) -> String {
    let line = line.strip_suffix(b"\n").unwrap_or(line);
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    String::from_utf8_lossy(line).into_owned()
}

/// What the connection's end says.
pub fn closed_error() -> Error {
    Error::device("The device closed the connection")
}

fn no_answer_message(command: &str) -> String {
    format!("No answer to '{command}'")
}

fn no_number_message(command: &str, answer: &str) -> String {
    format!("The answer to '{command}' is not a number: {answer:?}")
}

/// What a refused setting says.
pub fn refused_message(line: &str) -> String {
    format!("The device refused '{line}'")
}

/// What a start says when the rate leaves a long wait.
pub fn slow_message(buffer: Duration) -> String {
    format!(
        "At this rate and buffer size the device sends nothing for {} s at a time.",
        buffer.as_secs()
    )
}

/// What a one-shot stream that ended with its trigger unfired says.
pub fn unfired_message(bytes: u64) -> String {
    format!("The trigger did not fire in the {bytes} bytes of the device's buffer.")
}

/// What a start says of a module with no buffers, which won't send.
pub const NO_BUFFERS: &str = "The device has no buffer (memalloc is 0) and will send nothing: write its size to \
     /sys/devices/virtual/misc/beaglelogic/memalloc.";

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::TriggerMatch;

    #[test]
    fn a_conn_takes_any_tcp_word_and_three_fields() {
        assert_eq!(parse_conn("tcp-raw/h/5555"), Ok(("h", "5555")));
        assert_eq!(parse_conn("TCPX/h/5555/more"), Ok(("h", "5555")));
        assert_eq!(parse_conn("tcp/h/"), Ok(("h", "")));
        assert_eq!(parse_conn("tcp-raw"), Err("Invalid Parameters."));
        assert_eq!(parse_conn("tcp-raw/h"), Err("Invalid Parameters."));
        assert_eq!(parse_conn(""), Err("Invalid Parameters."));
        let not = Err("Only TCP (tcp-raw) protocol is currently supported.");
        assert_eq!(parse_conn("serial/h/1"), not);
        assert_eq!(parse_conn("tc/h/1"), not);
        assert_eq!(parse_conn("é/h/1"), not);
    }

    #[test]
    fn a_version_and_an_ok_are_prefixes_in_any_case() {
        assert!(is_beaglelogic("BeagleLogic 1.0"));
        assert!(is_beaglelogic("BeagleLogic Standalone 1.0"));
        assert!(is_beaglelogic("BEAGLELOGICx"));
        assert!(!is_beaglelogic("BeagleLogi"));
        assert!(!is_beaglelogic(""));
        assert!(is_ok("OK"));
        assert!(is_ok("ok then"));
        assert!(!is_ok("ERR"));
        assert!(!is_ok("O"));
        assert!(!is_ok(""));
    }

    #[test]
    fn a_number_is_digits_up_to_an_int() {
        assert_eq!(parse_number("100000000"), Some(100_000_000));
        assert_eq!(parse_number("0"), Some(0));
        assert_eq!(parse_number("2147483647"), Some(i32::MAX as u32));
        for bad in ["", "abc", "-5", "+5", " 5", "5 ", "0x10", "2147483648", "1e3", "5\r"] {
            assert_eq!(parse_number(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn a_unit_is_0_or_1() {
        assert_eq!(parse_unit("0"), Some(Unit::Sixteen));
        assert_eq!(parse_unit("1"), Some(Unit::Eight));
        for bad in ["", "2", "8", "abc"] {
            assert_eq!(parse_unit(bad), None, "{bad:?}");
        }
    }

    /// What a damaged answer comes to, with nothing but its form to check: TCP's sum
    /// covers the wire, and the format has none. A flip that keeps the digits is a wrong
    /// value no check could see. This counts them.
    #[test]
    fn what_the_format_can_catch_of_a_flipped_bit_or_a_lost_byte() {
        let answer = b"100000000\r\n";
        let good = 100_000_000;
        let (mut wrong, mut refused, mut same) = (0, 0, 0);
        let mut judge = |bytes: &[u8]| match parse_number(&strip_line(bytes)) {
            Some(n) if n == good => same += 1,
            Some(_) => wrong += 1,
            None => refused += 1,
        };
        for at in 0..answer.len() {
            for bit in 0..8 {
                let mut hit = answer.to_vec();
                hit[at] ^= 1 << bit;
                judge(&hit);
            }
        }
        for at in 0..answer.len() {
            let mut hit = answer.to_vec();
            hit.remove(at);
            judge(&hit);
        }
        // 88 flips and 11 lost bytes. Each of the 9 digits has 4 flips to another digit,
        // and each lost digit is another number, so 45 give a wrong value that nothing
        // can see. A lost line end changes nothing. The rest aren't digits.
        assert_eq!(wrong + refused + same, 99);
        assert_eq!(wrong, 45);
        assert_eq!(same, 2);
        assert_eq!(refused, 52);
    }

    #[test]
    fn a_rate_goes_to_a_whole_divisor() {
        assert_eq!(module_rate(100_000_000), Some(100_000_000));
        assert_eq!(module_rate(60_000_000), Some(100_000_000));
        assert_eq!(module_rate(50_000_000), Some(50_000_000));
        assert_eq!(module_rate(30_000_000), Some(33_333_333));
        assert_eq!(module_rate(1_000_000), Some(1_000_000));
        assert_eq!(module_rate(1), Some(1));
        assert_eq!(module_rate(0), None);
        assert_eq!(module_rate(100_000_001), None);
    }

    #[test]
    fn the_truncation_warning_and_its_test() {
        assert_eq!(
            truncated_message(33_554_432, Unit::Sixteen),
            "Please use 'echo <size in bytes> > /sys/devices/virtual/misc/beaglelogic/memalloc' \
             to increase the buffer size, this capture is now truncated to 16 Msamples"
        );
        assert!(truncated_message(33_554_432, Unit::Eight).ends_with(" 33 Msamples"));
        assert!(past_buffer(5, Unit::Sixteen, 9));
        assert!(!past_buffer(5, Unit::Sixteen, 10));
        assert!(!past_buffer(10, Unit::Eight, 10));
        assert!(past_buffer(1 << 63, Unit::Sixteen, 10), "no wrap");
    }

    #[test]
    fn a_buffer_takes_its_time() {
        // 4 MiB at 100 MHz, 8 bits.
        let t = buffer_time(4 * 1024 * 1024, 100_000_000, Unit::Eight);
        assert_eq!(t.as_millis(), 41);
        // At 10 Hz and 16 bits: 2.4 days.
        let t = buffer_time(4 * 1024 * 1024, 10, Unit::Sixteen);
        assert_eq!(t.as_secs(), 209_715);
        assert_eq!(buffer_time(4096, 0, Unit::Eight), Duration::ZERO);
    }

    #[test]
    fn pre_trigger_samples_are_a_share_with_no_wrap_and_a_cap() {
        assert_eq!(pre_trigger_samples(50, Some(8), Unit::Sixteen, 65536), 4);
        assert_eq!(pre_trigger_samples(0, Some(8), Unit::Sixteen, 65536), 0);
        assert_eq!(pre_trigger_samples(100, Some(8), Unit::Eight, 65536), 8);
        assert_eq!(pre_trigger_samples(33, Some(150), Unit::Eight, 65536), 49);
        assert_eq!(pre_trigger_samples(50, None, Unit::Sixteen, 65536), 0);
        // What the stream can hold.
        assert_eq!(pre_trigger_samples(100, Some(u64::MAX), Unit::Sixteen, 65536), 32768);
        // A memory a device's answer can't size.
        assert_eq!(pre_trigger_samples(100, Some(u64::MAX), Unit::Eight, i32::MAX as u32), 1 << 28);
    }

    fn secs(n: u64) -> Duration {
        Duration::from_secs(n)
    }

    #[test]
    fn bounds_follow_the_pace() {
        let b = Bounds::for_buffer(Duration::ZERO);
        assert_eq!(b, Bounds { first: secs(30), warn: secs(30), fail: secs(600) });
        let b = Bounds::for_buffer(secs(1000));
        assert_eq!(b, Bounds { first: secs(2030), warn: secs(2030), fail: secs(2030) });
    }

    const B: Bounds = Bounds {
        first: Duration::from_secs(30),
        warn: Duration::from_secs(40),
        fail: Duration::from_secs(100),
    };

    #[test]
    fn a_limited_capture_with_no_sample_fails_at_the_first_bound() {
        let mut q = Quiet::new(B, true, secs(1000));
        assert_eq!(q.empty(secs(29)), Verdict::Wait);
        assert_eq!(
            q.empty(secs(1)),
            Verdict::Fail(
                "The device has sent no data in 30 s, since it was asked to start".into()
            )
        );
    }

    #[test]
    fn a_continuous_capture_warns_once_and_goes_on_for_ever() {
        let mut q = Quiet::new(B, false, secs(1000));
        assert_eq!(q.empty(secs(29)), Verdict::Wait);
        assert!(matches!(q.empty(secs(1)), Verdict::Warn(_)));
        for _ in 0..10 {
            assert_eq!(q.empty(secs(1000)), Verdict::Wait);
        }
        assert_eq!(q.heard(), Some("The device is sending again."));
        assert_eq!(q.heard(), None);
    }

    #[test]
    fn after_a_sample_a_limited_capture_warns_and_then_fails_late() {
        let mut q = Quiet::new(B, true, secs(1000));
        assert_eq!(q.heard(), None);
        assert_eq!(q.empty(secs(39)), Verdict::Wait);
        assert_eq!(q.empty(secs(1)), Verdict::Warn("The device has sent no data for 40 s".into()));
        assert_eq!(q.empty(secs(59)), Verdict::Wait);
        assert_eq!(q.empty(secs(1)), Verdict::Fail("The device has sent no data for 100 s".into()));
    }

    #[test]
    fn bytes_put_the_clock_back_and_a_warning_may_come_again() {
        let mut q = Quiet::new(B, true, secs(1000));
        q.heard();
        for _ in 0..50 {
            assert_eq!(q.empty(secs(39)), Verdict::Wait);
            q.heard();
        }
        assert!(matches!(q.empty(secs(40)), Verdict::Warn(_)));
        assert!(q.heard().is_some());
        assert!(matches!(q.empty(secs(40)), Verdict::Warn(_)), "and again");
    }

    #[test]
    fn a_stall_counts_for_no_more_than_the_cap() {
        let mut q = Quiet::new(B, true, secs(1));
        // Stops of the process, each seen as one long read.
        for _ in 0..29 {
            assert_eq!(q.empty(secs(1000)), Verdict::Wait);
        }
        assert!(matches!(q.empty(secs(1000)), Verdict::Fail(_)));
    }

    fn logic(p: &Packet) -> &[u8] {
        match p {
            Packet::Logic(l) => &l.data,
            other => panic!("{other:?}"),
        }
    }

    fn all(packets: &[Packet]) -> Vec<u8> {
        packets
            .iter()
            .filter(|p| matches!(p, Packet::Logic(_)))
            .flat_map(|p| logic(p).to_vec())
            .collect()
    }

    #[test]
    fn a_limit_cuts_a_read() {
        let mut a = Acquire::new(Unit::Sixteen, Some(2), Some(100), None);
        let (p, done) = a.take(&[1, 0, 2, 0, 3, 0]);
        assert_eq!(all(&p), [1, 0, 2, 0]);
        assert_eq!(done, Done::Limit);
    }

    #[test]
    fn a_read_that_ends_in_a_sample_keeps_the_byte_for_the_next() {
        let mut a = Acquire::new(Unit::Sixteen, Some(3), Some(100), None);
        let (p, done) = a.take(&[1, 0, 2]);
        assert_eq!(all(&p), [1, 0]);
        assert_eq!(done, Done::No);
        let (p, done) = a.take(&[0]);
        assert_eq!(all(&p), [2, 0]);
        assert_eq!(done, Done::No);
        let (p, done) = a.take(&[3, 0]);
        assert_eq!(all(&p), [3, 0]);
        assert_eq!(done, Done::Limit);
        match Acquire::new(Unit::Sixteen, None, None, None).take(&[1, 0]).0.first() {
            Some(Packet::Logic(l)) => assert_eq!(l.unit_size, 2),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_one_shot_stream_ends_with_the_buffer_and_a_continuous_one_does_not() {
        let mut a = Acquire::new(Unit::Eight, Some(100), Some(4), None);
        assert_eq!(a.take(&[0, 1, 2]).1, Done::No);
        assert_eq!(a.take(&[3]).1, Done::Buffer);
        let mut a = Acquire::new(Unit::Eight, None, None, None);
        assert_eq!(a.take(&[0; 1000]).1, Done::No);
        // A buffer it never heard of, as C: the first read ends it.
        let mut a = Acquire::new(Unit::Eight, Some(100), Some(0), None);
        assert_eq!(a.take(&[0]).1, Done::Buffer);
        // The limit first when both come.
        let mut a = Acquire::new(Unit::Eight, Some(4), Some(4), None);
        assert_eq!(a.take(&[0; 4]).1, Done::Limit);
    }

    fn rising(ch: usize, unit: usize, pre: usize) -> SoftTriggerLogic {
        SoftTriggerLogic::new(vec![vec![(ch, TriggerMatch::Rising)]], unit, pre)
    }

    #[test]
    fn what_comes_ahead_of_the_trigger_does_not_count_for_the_limit() {
        let mut a = Acquire::new(Unit::Eight, Some(2), Some(100), Some(rising(0, 1, 0)));
        assert!(a.waiting());
        let (p, done) = a.take(&[0; 50]);
        assert_eq!(p.len(), 0);
        assert_eq!(done, Done::No);
        let (p, done) = a.take(&[0, 1, 1, 1]);
        assert!(matches!(p[0], Packet::Trigger));
        assert_eq!(all(&p), [1, 1]);
        assert_eq!(done, Done::Limit);
        assert!(!a.waiting());
    }

    #[test]
    fn a_trigger_is_where_it_is_at_the_stream_s_width() {
        // At 8 bits, where C steps by the 14 channels' two bytes.
        let mut a = Acquire::new(Unit::Eight, Some(2), Some(100), Some(rising(0, 1, 0)));
        let (p, _) = a.take(&[0, 0, 0, 1, 7, 8]);
        assert_eq!(all(&p), [1, 7]);
    }

    #[test]
    fn pre_trigger_samples_count_for_the_limit() {
        let mut a = Acquire::new(Unit::Eight, Some(4), Some(100), Some(rising(0, 1, 2)));
        let (p, done) = a.take(&[0, 2, 4, 1, 3, 5, 7]);
        assert_eq!(all(&p), [2, 4, 1, 3]);
        let at = p.iter().position(|x| matches!(x, Packet::Trigger));
        assert_eq!(at, Some(1), "after the pre-trigger samples, ahead of the point");
        assert_eq!(done, Done::Limit);
    }

    #[test]
    fn a_trigger_across_a_cut_sample_still_sees_it_whole() {
        let mut a = Acquire::new(Unit::Sixteen, Some(1), Some(100), Some(rising(9, 2, 0)));
        assert_eq!(a.take(&[0, 0, 0]).0.len(), 0);
        let (p, done) = a.take(&[2, 5, 6]);
        assert_eq!(all(&p), [0, 2]);
        assert_eq!(done, Done::Limit);
    }

    #[test]
    fn a_stream_that_ends_with_no_trigger_has_nothing_to_show() {
        let mut a = Acquire::new(Unit::Eight, Some(2), Some(4), Some(rising(7, 1, 0)));
        let (p, done) = a.take(&[0; 4]);
        assert_eq!(p.len(), 0);
        assert_eq!(done, Done::Buffer);
        assert!(a.waiting());
        assert_eq!(a.offset(), 4);
        assert_eq!(
            unfired_message(a.offset()),
            "The trigger did not fire in the 4 bytes of the device's buffer."
        );
    }

    #[test]
    fn messages_say_what_happened() {
        assert_eq!(no_answer_message("memalloc"), "No answer to 'memalloc'");
        assert_eq!(
            no_number_message("samplerate", "abc"),
            "The answer to 'samplerate' is not a number: \"abc\""
        );
        assert_eq!(refused_message("sampleunit 1"), "The device refused 'sampleunit 1'");
        assert_eq!(
            closed_error().to_string(),
            Error::device("The device closed the connection").to_string()
        );
        assert_eq!(
            slow_message(secs(4194)),
            "At this rate and buffer size the device sends nothing for 4194 s at a time."
        );
        assert!(NO_BUFFERS.starts_with("The device has no buffer (memalloc is 0)"));
    }

    #[test]
    fn a_line_end_goes_as_c_takes_it() {
        assert_eq!(strip_line(b"12\r\n"), "12");
        assert_eq!(strip_line(b"12\n"), "12");
        assert_eq!(strip_line(b"12\r"), "12");
        assert_eq!(strip_line(b"12"), "12");
        assert_eq!(strip_line(b"12\n\r"), "12\n");
        assert_eq!(strip_line(b"\r\n"), "");
    }
}
