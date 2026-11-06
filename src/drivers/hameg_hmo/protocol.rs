// Counterpart in sigrok:
//   libsigrok/src/hardware/hameg-hmo/protocol.c,
//   libsigrok/src/hardware/hameg-hmo/protocol.h,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   Copyright (C) 2018 Guido Trentalancia <guido@trentalancia.com>,
//   with commits by Uwe Hermann, Gerhard Sittig, Stefan Brüns, Soeren Apel,
//   Bert Vermeulen, Martin Ling, Mathias Katzer, Daniel Elstner,
//   Johannes Römer, Valentin Ochs, xandinator
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire of `hameg-hmo`: the state an open reads, the blocks a capture reads, and
//! how long it waits for them.
//!
//! No command starts or stops anything. A capture asks for what the scope holds, a
//! channel at a time, and `:FORM` comes in the same line as each request:
//! `:FORM:BORD LSBF;:FORM REAL,32;:CHAN1:DATA?` for an analog channel, 32 bit floats in
//! volts or amperes, and `:FORM UINT,8;:POD1:DATA?` for 8 logic channels a byte.
//!
//! What a block's read differs in from `sr_scpi_get_block`, each a choice:
//!
//! - Text isn't asked to be a block for ever: see [`Wait`].
//! - A stop is looked at between reads, and a block that crawls ends: see [`too_slow`].
//! - A line end ahead of a block is passed over, as what a read of the block before
//!   it left. C takes it for a block that isn't one.
//! - A length of 0 is an empty block. C returns no block with no error
//!   (`scpi.c`), and `hmo_receive_data` reads through the null pointer.
//! - A length must be all digits. C's `sr_atol` takes `-12`, which then sizes a string.

use super::models::{MAN, MANUFACTURERS, Model, PATTERN_ROOM, TRIGGER_SLOPES, USER1, USER2};
use crate::core::{Error, Result};
use crate::scpi::{Scpi, unquote_string};
use std::time::{Duration, Instant};

/// Bytes asked for per read while collecting a block.
const READ_CHUNK: usize = 32 * 1024;

/// `value` as `g_ascii_formatd(.., "%E", value)`: `1.000000E-03`.
pub fn format_e(value: f64) -> String {
    if value.is_nan() {
        return "NAN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "INF" } else { "-INF" }.to_owned();
    }
    let text = format!("{value:.6E}");
    let (mantissa, exponent) = text.split_once('E').expect("{:E} has an exponent");
    let exponent: i32 = exponent.parse().expect("{:E} has a number for an exponent");
    format!("{mantissa}E{}{:02}", if exponent < 0 { '-' } else { '+' }, exponent.abs())
}

/// `(float)p / q` as a table's entry goes to the scope, in `%E`.
pub fn table_value(entry: (u64, u64)) -> String {
    format_e(f64::from(entry.0 as f32 / entry.1 as f32))
}

/// `sr_parse_rational`: sign, digits with an optional point, an optional exponent,
/// and nothing after. `None` where C would fail or would overflow its 64 bits. The
/// exponent's size is limited here, where C's loops wrap.
pub fn parse_rational(text: &str) -> Option<(i64, i64)> {
    let bytes = text.as_bytes();
    let mut at = 0;
    while bytes.get(at).is_some_and(u8::is_ascii_whitespace) {
        at += 1;
    }
    let negative = bytes.get(at) == Some(&b'-');
    if matches!(bytes.get(at), Some(b'-' | b'+')) {
        at += 1;
    }
    let digits = |from: usize| {
        let mut end = from;
        while bytes.get(end).is_some_and(u8::is_ascii_digit) {
            end += 1;
        }
        end
    };
    let int_end = digits(at);
    let integral = &text[at..int_end];
    at = int_end;
    let mut fractional = "";
    if bytes.get(at) == Some(&b'.') {
        let end = digits(at + 1);
        fractional = &text[at + 1..end];
        at = end;
    }
    if integral.is_empty() && fractional.is_empty() {
        return None;
    }
    let mut exponent: i32 = 0;
    if matches!(bytes.get(at), Some(b'E' | b'e')) {
        at += 1;
        let exp_negative = bytes.get(at) == Some(&b'-');
        if matches!(bytes.get(at), Some(b'-' | b'+')) {
            at += 1;
        }
        let end = digits(at);
        exponent = text[at..end].parse().ok()?;
        if exp_negative {
            exponent = -exponent;
        }
        at = end;
    }
    if at != bytes.len() || exponent.abs() > 40 {
        return None;
    }
    let mut p: i64 = 0;
    for b in integral.bytes().chain(fractional.bytes()) {
        p = p.checked_mul(10)?.checked_add(i64::from(b - b'0'))?;
    }
    exponent -= i32::try_from(fractional.len()).ok()?;
    let mut q: i64 = 1;
    while exponent > 0 {
        p = p.checked_mul(10)?;
        exponent -= 1;
    }
    while exponent < 0 {
        q = q.checked_mul(10)?;
        exponent += 1;
    }
    Some((if negative { -p } else { p }, q))
}

/// `array_float_get`: which entry of a table an answer says, by value.
pub fn table_index(answer: &str, table: &[(u64, u64)]) -> Option<usize> {
    let (p, q) = parse_rational(answer)?;
    table
        .iter()
        .position(|(ap, aq)| i128::from(p) * i128::from(*aq) == i128::from(*ap) * i128::from(q))
}

/// True when two rationals are the same value, whatever their form: the core's
/// parser reduces its fractions, and C's tables don't.
pub fn same_ratio(a: (u64, u64), b: (u64, u64)) -> bool {
    u128::from(a.0) * u128::from(b.1) == u128::from(b.0) * u128::from(a.1)
}

/// What a scope has said about an analog channel.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AnalogState {
    /// Index in the model's `coupling`.
    pub coupling: usize,
    /// Index in the model's `vdivs`.
    pub vdiv: usize,
    /// `:CHANn:POS?`, only logged, as in C.
    pub vertical_offset: f32,
    /// It is on.
    pub on: bool,
    /// `A` or `V`.
    pub probe_unit: char,
}

/// What a scope has said about a pod.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PodState {
    /// It is on.
    pub on: bool,
    /// Index in the model's `thresholds`.
    pub threshold: usize,
    /// The level of a user's threshold, when one is in use.
    pub user_threshold: f32,
}

/// `struct scope_state`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct State {
    /// Per analog channel.
    pub analog: Vec<AnalogState>,
    /// Per logic channel.
    pub digital: Vec<bool>,
    /// Per pod.
    pub pods: Vec<PodState>,
    /// Index in the model's `timebases`.
    pub timebase: usize,
    /// `:TIM:DIV?`, which C puts in its model table.
    pub xdivs: i64,
    /// The trigger's place in a frame, from 0 to 1.
    pub horiz_triggerpos: f32,
    /// Index in the model's `trigger_sources`.
    pub trigger_source: usize,
    /// Index in [`TRIGGER_SLOPES`].
    pub trigger_slope: usize,
    /// As the scope gave it, or as it was set.
    pub trigger_pattern: String,
    /// `:ACQ:HRES?` is not `OFF`.
    pub high_resolution: bool,
    /// `:ACQ:PEAK?` is not `OFF`.
    pub peak_detection: bool,
    /// `:ACQ:SRAT?`.
    pub sample_rate: u64,
}

impl State {
    /// What it starts as, before a read of the scope: `g_malloc0`.
    pub fn new(model: &Model) -> State {
        State {
            analog: vec![AnalogState { probe_unit: 'V', ..AnalogState::default() }; model.analog],
            digital: vec![false; model.digital],
            pods: vec![PodState::default(); model.pods()],
            ..State::default()
        }
    }

    /// The time on screen, from which [`Bounds`] takes a pace.
    pub fn sweep(&self, model: &Model) -> Duration {
        let (p, q) = model.timebases[self.timebase];
        let seconds = p as f64 / q as f64 * self.xdivs.max(0) as f64;
        Duration::try_from_secs_f64(seconds).unwrap_or(Duration::ZERO)
    }
}

/// `scope_state_get_array_option`: `Err` for a silent scope and for an answer no entry
/// has, and C has one error for both at its one retry.
fn array_option(scpi: &mut Scpi, command: &str, table: &[&str]) -> Result<usize> {
    let answer = scpi.get_string(command)?;
    table
        .iter()
        .position(|e| *e == answer)
        .ok_or_else(|| Error::device(format!("{command} answered '{answer}', which isn't known")))
}

/// `analog_channel_state_get`.
fn read_analog(scpi: &mut Scpi, model: &Model, state: &mut State) -> Result<()> {
    for i in 0..model.analog {
        let n = i + 1;
        let on = scpi.get_bool(&format!(":CHAN{n}:STAT?"))?;
        state.analog[i].on = on;
        let scale = scpi.get_string(&format!(":CHAN{n}:SCAL?"))?;
        let Some(vdiv) = table_index(&scale, model.vdivs) else {
            log::error!("Could not determine array index for vertical div scale.");
            return Err(Error::device(format!("no such vertical scale here: '{scale}'")));
        };
        state.analog[i].vdiv = vdiv;
        state.analog[i].vertical_offset = scpi.get_f32(&format!(":CHAN{n}:POS?"))?;
        state.analog[i].coupling = array_option(scpi, &format!(":CHAN{n}:COUP?"), model.coupling)?;
        let unit = scpi.get_string(&format!(":PROB{n}:SET:ATT:UNIT?"))?;
        state.analog[i].probe_unit = if unit.starts_with('A') { 'A' } else { 'V' };
    }
    Ok(())
}

/// `digital_channel_state_get`.
fn read_digital(scpi: &mut Scpi, model: &Model, state: &mut State) -> Result<()> {
    for i in 0..model.digital {
        state.digital[i] = scpi.get_bool(&format!(":LOG{i}:STAT?"))?;
    }
    // "USER" for USER1 is what SCPI's short form makes of it, so a second reading asks
    // again and takes that too. So an answer nobody knows is asked for twice.
    let short: Vec<&str> =
        model.thresholds.iter().map(|t| if *t == USER1 { "USER" } else { *t }).collect();
    for i in 0..model.pods() {
        state.pods[i].on = scpi.get_bool(&model.dialect.get_pod_state(i + 1))?;
        let number = model.threshold_number(i);
        let command = model.dialect.get_threshold(number);
        let threshold = match array_option(scpi, &command, model.thresholds) {
            Ok(t) => t,
            Err(_) => array_option(scpi, &command, &short)?,
        };
        state.pods[i].threshold = threshold;
        let user = match model.thresholds[threshold] {
            USER1 => Some(1),
            USER2 => Some(2),
            MAN => Some(0),
            _ => None,
        };
        if let Some(user) = user {
            let command = model.dialect.get_user_threshold(number, user);
            state.pods[i].user_threshold = scpi.get_f32(&command)?;
        }
    }
    Ok(())
}

/// `hmo_update_sample_rate`. A float, and truncated, as in C.
pub fn read_sample_rate(scpi: &mut Scpi) -> Result<u64> {
    Ok(scpi.get_f32(":ACQ:SRAT?")? as u64)
}

/// `hmo_scope_state_get`: all of the scope's settings, in C's order.
///
/// C looks the timebase up in the long table on every model, and then uses the index
/// in the model's own. On a compact scope (no 1 ns) that says 2 ms for 1 ms and reads
/// past the table at 50 s (`protocol.c:1197`, left by a058de0, which split the table).
/// Here it is the model's table, so a compact scope answering 1 ns fails.
pub fn read_state(scpi: &mut Scpi, model: &Model, state: &mut State) -> Result<()> {
    log::info!("Fetching scope state");
    read_analog(scpi, model, state)?;
    read_digital(scpi, model, state)?;
    let timebase = scpi.get_string(":TIM:SCAL?")?;
    let Some(index) = table_index(&timebase, model.timebases) else {
        log::error!("Could not determine array index for time base.");
        return Err(Error::device(format!("no such time base here: '{timebase}'")));
    };
    state.timebase = index;
    state.xdivs = scpi.get_int(":TIM:DIV?")?;
    let position = scpi.get_f32(":TIM:POS?")?;
    let (p, q) = model.timebases[index];
    state.horiz_triggerpos = trigger_position(position, p as f64 / q as f64, state.xdivs);
    state.trigger_source = array_option(scpi, ":TRIG:A:SOUR?", model.trigger_sources)?;
    state.trigger_slope = array_option(scpi, ":TRIG:A:EDGE:SLOP?", TRIGGER_SLOPES)?;
    let pattern = scpi.get_string(":TRIG:A:PATT:SOUR?")?;
    state.trigger_pattern = keep_pattern(&unquote_string(&pattern));
    state.high_resolution = scpi.get_string(":ACQ:HRES?")? != "OFF";
    state.peak_detection = scpi.get_string(":ACQ:PEAK?")? != "OFF";
    state.sample_rate = read_sample_rate(scpi)?;
    log::info!("Fetching finished.");
    Ok(())
}

/// What the state keeps of a pattern: `strncpy`'s bytes, at a character's end.
pub fn keep_pattern(pattern: &str) -> String {
    let mut end = pattern.len().min(PATTERN_ROOM);
    while !pattern.is_char_boundary(end) {
        end -= 1;
    }
    pattern[..end].to_owned()
}

/// `:TIM:POS?` as a place in the frame, in C's mix of widths: the float state gets each
/// step. No divisions make an infinity or a NaN, as in C, and no panic.
pub fn trigger_position(position: f32, timebase: f64, xdivs: i64) -> f32 {
    let mut place = (f64::from(position) / (timebase * xdivs as f64)) as f32;
    place = (f64::from(place) - 0.5) as f32;
    -place
}

/// `:TIM:POS`'s seconds for a place in the frame.
pub fn trigger_seconds(place: f64, timebase: f64, xdivs: i64) -> f64 {
    -(place - 0.5) * timebase * xdivs as f64
}

/// `hmo_check_channels`: an analog channel and a pod that share an input.
/// `analog` and `pods` say which are wanted.
pub fn channels_conflict(analog: &[bool], pods: &[bool]) -> bool {
    let on = |list: &[bool], i: usize| list.get(i).copied().unwrap_or(false);
    (on(pods, 0) && on(analog, 2)) || (on(pods, 1) && on(analog, 3))
}

/// The end of a frame says the capture is over.
///
/// C has `++num_frames >= frame_limit || num_samples >= samples_limit`, in which
/// the second is true with no sample limit, so `--frames 3` gives one frame. That
/// was `== frame_limit` until d779dca added the sample limit, so it is taken for a
/// slip: the sample limit counts here only when there is one. As in C, a frame is the
/// end with no frame limit, and `num_samples` is that of the last channel, not a sum.
pub fn capture_over(frames: u64, frame_limit: u64, num_samples: u64, samples_limit: u64) -> bool {
    frames >= frame_limit || (samples_limit > 0 && num_samples >= samples_limit)
}

/// The start of a block, as far as it has come.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Head {
    /// Too short to say.
    More,
    /// `#n` and `n` digits take this much, for this many bytes of data.
    Block {
        /// Bytes ahead of the data.
        head: usize,
        /// Bytes of data.
        len: usize,
    },
    /// `#0`.
    Indefinite,
    /// It isn't a block.
    Not,
}

/// What `sr_scpi_get_block` makes of the start of an answer.
pub fn parse_head(buf: &[u8]) -> Head {
    match buf {
        [] | [b'#'] => Head::More,
        [b'#', b'0', ..] => Head::Indefinite,
        [b'#', d @ b'1'..=b'9', rest @ ..] => {
            let digits = usize::from(*d - b'0');
            let Some(text) = rest.get(..digits) else {
                return if rest.iter().all(u8::is_ascii_digit) { Head::More } else { Head::Not };
            };
            if !text.iter().all(u8::is_ascii_digit) {
                return Head::Not;
            }
            let len = text.iter().fold(0usize, |n, d| n * 10 + usize::from(*d - b'0'));
            Head::Block { head: 2 + digits, len }
        }
        _ => Head::Not,
    }
}

/// A block's read took long enough to look at, and is below what any of these
/// carriers does. A serial line at 115200 baud gives 11 kB/s and a block's read
/// starts in that line's first second, so 1 kB/s of a whole read says a stuck device
/// and not a long record.
pub fn too_slow(elapsed: Duration, got: usize, limit: Duration) -> bool {
    const FLOOR: f64 = 1000.0;
    elapsed >= limit && (got as f64) < elapsed.as_secs_f64() * FLOOR
}

/// What came of a look for a block.
#[derive(Debug, PartialEq, Eq)]
pub enum BlockRead {
    /// All it said it had. Empty for `#10`.
    Whole(Vec<u8>),
    /// Less than it said, and no more came.
    Short {
        /// What came.
        data: Vec<u8>,
        /// What it said.
        announced: usize,
    },
    /// Nothing in a read's wait.
    Silent,
    /// Bytes, which make no block.
    Junk,
    /// `go_on` said no.
    Stopped,
}

/// Reads one answer as a block, with no question asked: `sr_scpi_get_block(scpi, NULL)`.
///
/// `read_complete` isn't trusted for an end: on `tcp-raw` any short read says it, and
/// a block arrives in many. A USBTMC message that ended short comes out as an error on
/// the next read, with `read_complete` true. A read past that on USBTMC goes no further
/// than this function, as the error is of the message's end.
pub fn read_block(
    scpi: &mut Scpi,
    limit: Duration,
    go_on: &impl Fn() -> bool,
) -> Result<BlockRead> {
    match scpi.read_begin() {
        Ok(()) => {}
        Err(Error::Timeout) => return Ok(BlockRead::Silent),
        Err(e @ (Error::Disconnected | Error::Io(_))) => return Err(e),
        Err(e) => {
            log::debug!("Reading a block: {e}");
            return Ok(BlockRead::Junk);
        }
    }
    let began = Instant::now();
    let mut buf: Vec<u8> = Vec::new();
    let mut chunk = vec![0u8; READ_CHUNK];
    // Where the block's bytes start, past any line end before it.
    let mut skip = 0;
    let mut block: Option<(usize, usize)> = None;
    loop {
        if !go_on() {
            return Ok(BlockRead::Stopped);
        }
        let read = scpi.read_data(&mut chunk);
        let n = match read {
            Ok(n) => n,
            Err(Error::Timeout) => 0,
            Err(e @ (Error::Disconnected | Error::Io(_))) => return Err(e),
            Err(e) if scpi.read_complete() => {
                log::debug!("Reading a block: {e}");
                0
            }
            Err(e) => return Err(e),
        };
        if n == 0 {
            return Ok(match block {
                Some((head, len)) => {
                    buf.drain(..skip + head);
                    BlockRead::Short { data: buf, announced: len }
                }
                None if buf.is_empty() => BlockRead::Silent,
                None => BlockRead::Junk,
            });
        }
        buf.extend_from_slice(&chunk[..n]);
        if block.is_none() {
            while matches!(buf.get(skip), Some(b'\r' | b'\n')) {
                skip += 1;
            }
            match parse_head(&buf[skip..]) {
                Head::More => {}
                Head::Block { head, len } => block = Some((head, len)),
                Head::Indefinite => {
                    log::error!("unsupported INDEFINITE LENGTH ARBITRARY BLOCK RESPONSE");
                    return Ok(BlockRead::Junk);
                }
                Head::Not => return Ok(BlockRead::Junk),
            }
        }
        if let Some((head, len)) = block {
            let have = buf.len() - skip - head;
            if have >= len {
                buf.drain(..skip + head);
                buf.truncate(len);
                return Ok(BlockRead::Whole(buf));
            }
            if too_slow(began.elapsed(), have, limit) {
                log::warn!("The block comes too slowly: {have} of {len} bytes.");
                buf.drain(..skip + head);
                return Ok(BlockRead::Short { data: buf, announced: len });
            }
        } else if began.elapsed() >= limit {
            return Ok(BlockRead::Junk);
        }
    }
}

/// How long a capture waits for a block.
///
/// C waits for ever. The HMO compact's SCPI manual gives no time for `:CHANn:DATA?` to
/// start its answer, so the bounds come from the only pace known: it is taken not to need
/// more than two sweeps of the screen, an inference, and at 50 s a division that is 20
/// minutes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bounds {
    /// With no block yet in this capture, silence fails here.
    pub first: Duration,
    /// After a block: a word, once a request.
    pub warn: Duration,
    /// After a block: silence fails here.
    pub limit: Duration,
    /// Bytes that make no block, and nothing after that makes one, fail here.
    pub junk: Duration,
}

impl Bounds {
    /// The bounds at a sweep of this length.
    pub fn for_sweep(sweep: Duration) -> Bounds {
        let pace = sweep.saturating_mul(2);
        let first = pace.max(Duration::from_secs(30));
        Bounds {
            first,
            warn: first,
            limit: pace.max(Duration::from_secs(600)),
            junk: Duration::from_secs(5),
        }
    }
}

/// What to do at a look that found no block.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Go on.
    Wait,
    /// Say this, once, and go on.
    Warn(String),
    /// End the capture with this.
    Fail(String),
}

/// The wait for one block. Its times come in as arguments, so its tests take none.
///
/// A look's time is taken ahead of the read that found nothing, so that a stall between
/// the two can't pass for silence with an answer waiting. Time this thread didn't run
/// counts for nothing, up to [`Wait::MOST_CREDIT`], so a process stopped again and
/// again still comes to its end.
#[derive(Clone, Debug)]
pub struct Wait {
    bounds: Bounds,
    since: Instant,
    looked: Instant,
    junk_since: Option<Instant>,
    credit: Duration,
    heard: bool,
    warned: bool,
}

impl Wait {
    /// The longest a turn of the wait's loop takes when it runs at all: a read's wait,
    /// a USBTMC request's write, and room for a loaded box.
    const STALL: Duration = Duration::from_secs(3);
    /// The most time that counts for nothing, for one request.
    const MOST_CREDIT: Duration = Duration::from_secs(60);

    /// No block yet, in a capture starting at `now`.
    pub fn new(bounds: Bounds, now: Instant) -> Wait {
        Wait {
            bounds,
            since: now,
            looked: now,
            junk_since: None,
            credit: Duration::ZERO,
            heard: false,
            warned: false,
        }
    }

    /// A request has gone, at `now`, taken after its send and after anything a client
    /// kept waiting.
    pub fn asked(&mut self, now: Instant) {
        self.since = now;
        self.looked = now;
        self.junk_since = None;
        self.credit = Duration::ZERO;
        self.warned = false;
    }

    /// A block came. True if the wait had said something, so it should say it's over.
    pub fn block(&mut self) -> bool {
        self.heard = true;
        std::mem::take(&mut self.warned)
    }

    /// A look that began at `now` found nothing, or with `junk` bytes that make no block.
    pub fn look(&mut self, now: Instant, junk: bool) -> Verdict {
        let gap = now.saturating_duration_since(self.looked);
        self.looked = now;
        let left = Self::MOST_CREDIT.saturating_sub(self.credit);
        if gap > Self::STALL && !left.is_zero() {
            let credit = gap.min(left);
            self.since += credit;
            self.junk_since = self.junk_since.map(|j| j + credit);
            self.credit += credit;
            return Verdict::Wait;
        }
        if junk && self.junk_since.is_none() {
            self.junk_since = Some(now);
        }
        if let Some(j) = self.junk_since
            && now.saturating_duration_since(j) >= self.bounds.junk
        {
            return Verdict::Fail(format!(
                "No usable answer from the scope in {} s: what it sent is no block.",
                self.bounds.junk.as_secs()
            ));
        }
        let silent = now.saturating_duration_since(self.since);
        if !self.heard {
            if silent >= self.bounds.first {
                return Verdict::Fail(format!(
                    "The scope has sent no block in {} s.",
                    self.bounds.first.as_secs()
                ));
            }
            return Verdict::Wait;
        }
        if silent >= self.bounds.limit {
            return Verdict::Fail(format!(
                "The scope has sent no block for {} s.",
                self.bounds.limit.as_secs()
            ));
        }
        if silent >= self.bounds.warn && !self.warned {
            self.warned = true;
            return Verdict::Warn(format!(
                "The scope has sent no block for {} s; waiting up to {} s.",
                self.bounds.warn.as_secs(),
                self.bounds.limit.as_secs()
            ));
        }
        Verdict::Wait
    }
}

/// `hmo_queue_logic_data`: pods gathered for one logic packet with a byte of each per
/// sample. The bytes of a pod that didn't answer stay 0.
///
/// C sizes its store from the first pod to come and writes each later one to its own
/// length, past the store when it is longer (the check on that went in d779dca, and a
/// comment still says it's there). Here a longer one is cut, with a word.
#[derive(Debug, Default)]
pub struct LogicStore {
    store: Option<Vec<u8>>,
}

impl LogicStore {
    /// Folds in a pod's bytes.
    pub fn queue(&mut self, group: usize, pod_count: usize, data: &[u8]) {
        let store = self.store.get_or_insert_with(|| vec![0; data.len() * pod_count]);
        let room = store.len() / pod_count.max(1);
        if group >= pod_count {
            return;
        }
        if data.len() > room {
            log::warn!(
                "Pod {} sent {} samples where the first sent {room}; cut to those.",
                group + 1,
                data.len()
            );
        }
        for (i, byte) in data.iter().take(room).enumerate() {
            store[i * pod_count + group] = *byte;
        }
    }

    /// What goes in the frame's logic packet, cut to a limit: `None` if nothing queued.
    pub fn take(&mut self, pod_count: usize, samples_limit: u64) -> Option<Vec<u8>> {
        let mut store = self.store.take()?;
        if samples_limit > 0 {
            let most = usize::try_from(samples_limit).unwrap_or(usize::MAX);
            store.truncate(most.saturating_mul(pod_count));
        }
        Some(store)
    }
}

/// Whether a manufacturer's name is one of this driver's: exact, as `std_str_idx_s`.
pub fn known_manufacturer(name: &str) -> bool {
    MANUFACTURERS.contains(&name)
}

/// A block's bytes as samples: C hands on the host's floats, having asked for the
/// host's order. Here it always asks for `LSBF`. What's over a whole sample goes.
pub fn floats(data: &[u8]) -> Vec<f32> {
    data.as_chunks::<4>().0.iter().map(|b| f32::from_le_bytes(*b)).collect()
}
