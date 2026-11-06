// Counterpart in sigrok:
//   libsigrok/src/hardware/cem-dt-885x/protocol.c,
//   libsigrok/src/hardware/cem-dt-885x/protocol.h,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Aurelien Jacobs, Daniel Elstner,
//   Frank Stettner, Martin Ling
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire of a CEM DT-885x: a stream no one asks for, and commands no one answers.
//!
//! A meter sends packets all the time: `a5`, a token and up to three data bytes.
//! [`Parser`] takes them one byte at a time, as `process_byte`; what it finds goes
//! to the [`Event`] list a capture sends, and in a wait for a token nowhere.
//!
//! A command is one byte. It is never acknowledged and is often lost, and each one
//! heard steps the state, so a setting is: send it, wait for the next announcement of
//! the state, send it again if that is not the state wanted, and when it is, have the
//! next announcement agree ([`Link::toggle`]).
//!
//! A stored log starts with `bb` and a length, and then has records: `aa` or `cc`,
//! seven bytes of date, time and interval, and after `ac` the levels, in BCD like the
//! live ones. `dd` ends it. `ff`, at any place, says the meter is in hold.

use crate::core::{Error, MqFlags, Result};
use crate::serial::{Params, Port};
use std::time::{Duration, Instant};

/// `SERIALCOMM`.
pub const SERIALCOMM: &str = "9600/8n1";

/// The token of a measurement's level.
pub const TOKEN_MEASUREMENT: u8 = 0x0d;
/// That level was on the display's readout.
pub const TOKEN_MEAS_WAS_READOUT: u8 = 0x0b;
/// That level was on the bar graph.
pub const TOKEN_MEAS_WAS_BARGRAPH: u8 = 0x0c;
/// The time weighting.
pub const TOKEN_WEIGHT_TIME_FAST: u8 = 0x02;
/// The time weighting.
pub const TOKEN_WEIGHT_TIME_SLOW: u8 = 0x03;
/// The frequency weighting.
pub const TOKEN_WEIGHT_FREQ_A: u8 = 0x1b;
/// The frequency weighting.
pub const TOKEN_WEIGHT_FREQ_C: u8 = 0x1c;
/// The hold mode.
pub const TOKEN_HOLD_MAX: u8 = 0x04;
/// The hold mode.
pub const TOKEN_HOLD_MIN: u8 = 0x05;
/// The hold mode.
pub const TOKEN_HOLD_NONE: u8 = 0x0e;
/// Recording to memory.
pub const TOKEN_RECORDING_ON: u8 = 0x0a;
/// Not recording to memory.
pub const TOKEN_RECORDING_OFF: u8 = 0x1a;
/// The range.
pub const TOKEN_MEAS_RANGE_30_80: u8 = 0x30;
/// The range.
pub const TOKEN_MEAS_RANGE_30_130: u8 = 0x40;
/// The range.
pub const TOKEN_MEAS_RANGE_50_100: u8 = 0x4b;
/// The range.
pub const TOKEN_MEAS_RANGE_80_130: u8 = 0x4c;

/// The ranges, in the order `--show` lists them, with their tokens.
pub const RANGES: [(u64, u64, u8); 4] = [
    (30, 130, TOKEN_MEAS_RANGE_30_130),
    (30, 80, TOKEN_MEAS_RANGE_30_80),
    (50, 100, TOKEN_MEAS_RANGE_50_100),
    (80, 130, TOKEN_MEAS_RANGE_80_130),
];

/// A command.
pub const CMD_TOGGLE_RECORDING: u8 = 0x55;
/// A command.
pub const CMD_TOGGLE_WEIGHT_FREQ: u8 = 0x99;
/// A command.
pub const CMD_TOGGLE_WEIGHT_TIME: u8 = 0x77;
/// A command.
pub const CMD_TOGGLE_HOLD_MAX_MIN: u8 = 0x11;
/// A command.
pub const CMD_TOGGLE_MEAS_RANGE: u8 = 0x88;
/// A command.
pub const CMD_TOGGLE_POWER_OFF: u8 = 0x33;
/// A command.
pub const CMD_TRANSFER_MEMORY: u8 = 0xac;

const SYNC: u8 = 0xa5;
const HOLD_NOTICE: u8 = 0xff;
const LOG_START: u8 = 0xbb;
const RECORD_DBA: u8 = 0xaa;
const RECORD_DBC: u8 = 0xcc;
const RECORD_DATA: u8 = 0xac;
const RECORD_END: u8 = 0xdd;

/// Levels of a log per packet.
pub const SAMPLES_PER_PACKET: usize = 50;
const BUF_SIZE: usize = SAMPLES_PER_PACKET * 2;
/// A record's head: its token and seven bytes.
const RECORD_HEAD: usize = 8;

/// In hold the last level goes out again this often, as it does on the display.
pub const HOLD_REPEAT_INTERVAL: Duration = Duration::from_millis(50);

/// How long the next announcement of a state said in most blocks may take, about
/// two blocks on a real one where C has 10 to 110 ms, and a few on a slow host.
pub const FREQUENT_WAIT: Duration = Duration::from_millis(600);
/// The same for a state said in the long block, at 2 Hz by the C's comment.
pub const SLOW_WAIT: Duration = Duration::from_millis(1200);
/// A 9600 baud packet of five bytes. What completes this soon after a command
/// began before the meter could have heard it.
pub const PACKET_TIME: Duration = Duration::from_millis(6);
/// How long a meter's next token takes at most, in the C's comment.
pub const POWER_OFF_WAIT: Duration = Duration::from_millis(25);

/// How long a wait goes with no byte: the meter never stops talking, in 23 ms at
/// most, so that is a meter gone. C errors out at once on no byte, see [`Link`].
pub const WAIT_SILENCE: Duration = Duration::from_secs(1);
/// How long a wait with no time of its own goes with bytes but not the token,
/// where C waits for ever. The slowest token comes at 2 Hz.
pub const WAIT_BOUND: Duration = Duration::from_secs(2);
/// How long a toggle goes on, where C does for ever: a hundred commands at one per
/// block. It also bounds a power off.
pub const TOGGLE_BOUND: Duration = Duration::from_secs(5);
/// The same for a state said in the long block: some thirty commands at one per
/// long block.
pub const SLOW_TOGGLE_BOUND: Duration = Duration::from_secs(15);
/// How long one read in a wait takes, at most.
const WAIT_READ: Duration = Duration::from_millis(20);

/// The token's data bytes: `None`, a token no one knows, whose data ends where a
/// new command seems to start.
pub fn token_payload_len(token: u8) -> Option<usize> {
    match token {
        0x02 | 0x03 | 0x04 | 0x05 | 0x07 | 0x08 | 0x09 | 0x0a | 0x0c | 0x0e | 0x0f | 0x11
        | 0x19 | 0x1a | 0x1f | 0x30 | 0x40 | 0x4b | 0x4c => Some(0),
        0x0b | 0x1b | 0x1c => Some(1),
        0x0d => Some(2),
        0x06 => Some(3),
        _ => None,
    }
}

/// `process_mset`'s and `send_data`'s level: four BCD digits, or what they make
/// when they are not, in C's order of a float sum and a last step in double.
pub fn decode_level(high: u8, low: u8) -> f32 {
    let mut level = f32::from(u16::from(high >> 4) * 100);
    level += f32::from(u16::from(high & 0x0f) * 10);
    level += f32::from(low >> 4);
    (f64::from(level) + f64::from(low & 0x0f) / 10.0) as f32
}

/// `flags` with those of `gone` off. The core's type only adds.
fn without(flags: MqFlags, gone: MqFlags) -> MqFlags {
    flags.iter().filter(|f| !gone.contains(*f)).fold(MqFlags::empty(), |all, f| all | f)
}

/// What the stream brought for a capture.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// Levels, one for a live reading, a packet's for a log.
    Levels(Vec<f32>, MqFlags),
    /// A record's interval, in ms.
    Interval(u64),
    /// A log's end: a capture of it stops here, as a client could not tell it
    /// from the live levels that follow.
    End,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Init,
    GetToken,
    GetData,
    LogHeader,
    RecordMeta,
    RecordData,
}

/// `process_byte`, and the part of `dev_context` it keeps.
#[derive(Debug)]
pub struct Parser {
    /// `cur_mqflags`.
    pub flags: MqFlags,
    /// `recording`, as last seen in a capture.
    pub recording: Option<bool>,
    /// `cur_meas_range`, a token.
    pub range: Option<u8>,
    /// A level has come in this capture. A display token ahead of any says nothing:
    /// C sends what `last_spl` holds, 0 or the last capture's.
    have_level: bool,
    state: State,
    token: u8,
    buf: [u8; BUF_SIZE],
    buf_len: usize,
    last_spl: f32,
    hold_last_sent: Option<Instant>,
}

impl Default for Parser {
    fn default() -> Self {
        Parser {
            flags: MqFlags::empty(),
            recording: None,
            range: None,
            have_level: false,
            state: State::Init,
            token: 0,
            buf: [0; BUF_SIZE],
            buf_len: 0,
            last_spl: 0.0,
            hold_last_sent: None,
        }
    }
}

impl Parser {
    /// `dev_acquisition_start`'s reset, and no level yet. A hold's time may stay: a
    /// repeat it brings early has no level to say.
    pub fn start(&mut self) {
        self.state = State::Init;
        self.buf_len = 0;
        self.have_level = false;
    }

    /// In a log's header, the two bytes of its length: a capture asks for the log
    /// until it is.
    pub fn in_log_header(&self) -> bool {
        self.state == State::LogHeader
    }

    fn set(&mut self, on: MqFlags, off: MqFlags) {
        self.flags = without(self.flags, off) | on;
    }

    fn mset(&mut self, events: &mut Vec<Event>) {
        match self.token {
            TOKEN_WEIGHT_TIME_FAST => {
                self.set(MqFlags::SPL_TIME_WEIGHT_F, MqFlags::SPL_TIME_WEIGHT_S);
            }
            TOKEN_WEIGHT_TIME_SLOW => {
                self.set(MqFlags::SPL_TIME_WEIGHT_S, MqFlags::SPL_TIME_WEIGHT_F);
            }
            TOKEN_WEIGHT_FREQ_A => {
                self.set(MqFlags::SPL_FREQ_WEIGHT_A, MqFlags::SPL_FREQ_WEIGHT_C);
            }
            TOKEN_WEIGHT_FREQ_C => {
                self.set(MqFlags::SPL_FREQ_WEIGHT_C, MqFlags::SPL_FREQ_WEIGHT_A);
            }
            TOKEN_HOLD_MAX => self.set(MqFlags::HOLD | MqFlags::MAX, MqFlags::MIN),
            TOKEN_HOLD_MIN => self.set(MqFlags::HOLD | MqFlags::MIN, MqFlags::MAX),
            TOKEN_HOLD_NONE => {
                self.set(MqFlags::empty(), MqFlags::MAX | MqFlags::MIN | MqFlags::HOLD);
            }
            TOKEN_MEASUREMENT => {
                self.last_spl = decode_level(self.buf[0], self.buf[1]);
                self.have_level = true;
            }
            TOKEN_MEAS_WAS_READOUT | TOKEN_MEAS_WAS_BARGRAPH => {
                // In max or min hold the bar graph goes on, and the display does
                // not show it.
                let held = self.flags.contains(MqFlags::MAX) || self.flags.contains(MqFlags::MIN);
                if self.have_level && !(held && self.token == TOKEN_MEAS_WAS_BARGRAPH) {
                    events.push(Event::Levels(vec![self.last_spl], self.flags));
                }
            }
            TOKEN_RECORDING_ON => self.recording = Some(true),
            TOKEN_RECORDING_OFF => self.recording = Some(false),
            TOKEN_MEAS_RANGE_30_80
            | TOKEN_MEAS_RANGE_30_130
            | TOKEN_MEAS_RANGE_50_100
            | TOKEN_MEAS_RANGE_80_130 => self.range = Some(self.token),
            _ => {}
        }
    }

    /// What `send_data` sends, with an empty packet left out.
    fn send_data(&self, count: usize, events: &mut Vec<Event>) {
        if count > 0 {
            let levels = self.buf[..count * 2]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| decode_level(pair[0], pair[1]))
                .collect();
            events.push(Event::Levels(levels, self.flags));
        }
    }

    /// One byte. With `handle_packets` false, a wait for a token: nothing goes to
    /// the settings or a capture from a packet, as in C, where a get leaves the
    /// flags alone and asks again the next time.
    pub fn byte(&mut self, c: u8, handle_packets: bool, now: Instant, events: &mut Vec<Event>) {
        if c == HOLD_NOTICE {
            self.flags |= MqFlags::HOLD;
            match self.hold_last_sent {
                None => {
                    self.hold_last_sent = Some(now);
                    // The meter starts from scratch when it leaves hold.
                    self.state = State::Init;
                }
                Some(last) if now.saturating_duration_since(last) > HOLD_REPEAT_INTERVAL => {
                    self.token = TOKEN_MEAS_WAS_READOUT;
                    if handle_packets {
                        self.mset(events);
                    }
                    self.hold_last_sent = Some(now);
                }
                Some(_) => {}
            }
            return;
        }
        self.flags = without(self.flags, MqFlags::HOLD);
        self.hold_last_sent = None;
        match self.state {
            State::Init => {
                if c == SYNC {
                    self.token = 0;
                    self.state = State::GetToken;
                } else if c == LOG_START {
                    self.buf_len = 0;
                    self.state = State::LogHeader;
                    log::debug!("got command 0xbb");
                }
            }
            State::GetToken => {
                self.token = c;
                self.buf_len = 0;
                if token_payload_len(c) == Some(0) {
                    if handle_packets {
                        self.mset(events);
                    }
                    self.state = State::Init;
                } else {
                    self.state = State::GetData;
                }
            }
            State::GetData => self.get_data(c, handle_packets, events),
            State::LogHeader => {
                log::debug!("log header: 0x{c:02x}");
                self.buf[self.buf_len] = c;
                self.buf_len += 1;
                if self.buf_len == 2 {
                    let length = i32::from(self.buf[0]) * 256 + i32::from(self.buf[1]) - 100;
                    log::debug!("Device says it has {length} bytes stored.");
                    self.buf_len = 0;
                    self.state = State::RecordMeta;
                }
            }
            State::RecordMeta => self.record_meta(c, events),
            State::RecordData => {
                log::debug!("log data: 0x{c:02x}");
                if matches!(c, RECORD_DBA | RECORD_DBC | RECORD_DATA | RECORD_END) {
                    // The firmware's last byte before the end is half a level.
                    self.send_data(self.buf_len / 2, events);
                    self.buf_len = 0;
                    self.state = State::RecordMeta;
                    self.record_meta(c, events);
                } else {
                    self.buf[self.buf_len] = c;
                    self.buf_len += 1;
                    if self.buf_len == BUF_SIZE {
                        self.send_data(self.buf_len / 2, events);
                        self.buf_len = 0;
                    }
                }
            }
        }
    }

    fn get_data(&mut self, c: u8, handle_packets: bool, events: &mut Vec<Event>) {
        let Some(len) = token_payload_len(self.token) else {
            log::debug!("Unknown 0xa5 token 0x{:02x}", self.token);
            if c == SYNC || c == LOG_START {
                // It looks like a new command, and goes like the rest: the packet it
                // starts is lost.
                if handle_packets {
                    self.mset(events);
                }
                self.state = State::Init;
            } else if self.buf_len < BUF_SIZE {
                self.buf[self.buf_len] = c;
                self.buf_len += 1;
            } else {
                // C has stored this one past the end of the buffer by now.
                self.state = State::Init;
            }
            return;
        };
        self.buf[self.buf_len] = c;
        self.buf_len += 1;
        if self.buf_len == len {
            if handle_packets {
                self.mset(events);
            }
            self.state = State::Init;
        }
    }

    fn record_meta(&mut self, c: u8, events: &mut Vec<Event>) {
        log::debug!("log meta: 0x{c:02x}");
        if c == RECORD_END {
            self.state = State::Init;
            events.push(Event::End);
        } else if c == RECORD_DATA {
            self.buf_len = 0;
            self.state = State::RecordData;
        } else {
            // A head that names no record leaves the buffer as it is. C goes on
            // storing what follows, past its end.
            if self.buf_len < RECORD_HEAD {
                self.buf[self.buf_len] = c;
                self.buf_len += 1;
            }
            if self.buf_len < RECORD_HEAD {
                return;
            }
            self.flags = match self.buf[0] {
                RECORD_DBA => MqFlags::SPL_FREQ_WEIGHT_A,
                RECORD_DBC => MqFlags::SPL_FREQ_WEIGHT_C,
                other => {
                    log::debug!("Unknown record token 0x{other:02x}");
                    return;
                }
            };
            // The byte as it is. The wiki's table says BCD, and the C and the
            // `dt8852` package both read it as binary.
            events.push(Event::Interval(u64::from(self.buf[RECORD_HEAD - 1]) * 1000));
            self.buf_len = 0;
        }
    }
}

/// What ended a wait that did not fail on the port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Waited {
    /// A token of the list came.
    Found(u8),
    /// Its time ran out with bytes coming, or none yet.
    TimeUp,
    /// No byte for [`Link::silence`].
    Silent,
}

/// What a get or a set of one state needs to know about it.
#[derive(Clone, Copy, Debug)]
pub struct Setting {
    /// What an error calls it.
    pub what: &'static str,
    /// The command that steps it.
    pub cmd: u8,
    /// How long its next announcement may take.
    pub announce: Duration,
    /// How long a toggle of it goes on.
    pub bound: Duration,
}

/// The frequency weighting, said in each block.
pub const FREQ: Setting = Setting {
    what: "the frequency weighting",
    cmd: CMD_TOGGLE_WEIGHT_FREQ,
    announce: FREQUENT_WAIT,
    bound: TOGGLE_BOUND,
};
/// The time weighting, said in the long block.
pub const TIME: Setting = Setting {
    what: "the time weighting",
    cmd: CMD_TOGGLE_WEIGHT_TIME,
    announce: SLOW_WAIT,
    bound: SLOW_TOGGLE_BOUND,
};
/// The hold mode, said in the short blocks.
pub const HOLD: Setting = Setting {
    what: "the hold mode",
    cmd: CMD_TOGGLE_HOLD_MAX_MIN,
    announce: FREQUENT_WAIT,
    bound: TOGGLE_BOUND,
};
/// The range, said in the short blocks.
pub const RANGE: Setting = Setting {
    what: "the range",
    cmd: CMD_TOGGLE_MEAS_RANGE,
    announce: FREQUENT_WAIT,
    bound: TOGGLE_BOUND,
};
/// Recording, said in the long block.
pub const RECORDING: Setting = Setting {
    what: "the recording state",
    cmd: CMD_TOGGLE_RECORDING,
    announce: SLOW_WAIT,
    bound: SLOW_TOGGLE_BOUND,
};

/// A time as a message says it.
pub(super) fn spoken(time: Duration) -> String {
    if time.subsec_millis() == 0 {
        format!("{} s", time.as_secs())
    } else {
        format!("{} ms", time.as_millis())
    }
}

/// The port and the state on it.
///
/// `wait_for_token` in C reads without waiting, and a read with no byte is an
/// error, "Device might have gone away" (`protocol.c:425-427`). At 9600 baud there
/// is nearly never a byte there yet, so nearly every get or set of the meter's state
/// fails. The read was a blocking one until libsigrok `d7b269da` (2014-09-21) opened
/// the port without `SERIAL_RDWR`'s wait. This waits for the byte, up to
/// [`WAIT_SILENCE`].
pub struct Link {
    /// The open port.
    pub port: Option<Box<dyn Port>>,
    /// The line.
    pub params: Params,
    /// The parser.
    pub parser: Parser,
    /// [`WAIT_SILENCE`], or a test's.
    pub silence: Duration,
    /// [`WAIT_BOUND`], or a test's.
    pub wait_bound: Duration,
    /// A test's in place of [`Setting::bound`] of power off and every toggle.
    pub toggle_bound: Option<Duration>,
    /// A test's in place of [`Setting::announce`].
    pub announce: Option<Duration>,
}

impl Link {
    /// A link with no port yet.
    pub fn new(params: Params) -> Link {
        Link {
            port: None,
            params,
            parser: Parser::default(),
            silence: WAIT_SILENCE,
            wait_bound: WAIT_BOUND,
            toggle_bound: None,
            announce: None,
        }
    }

    fn port(&mut self) -> Result<&mut Box<dyn Port>> {
        self.port.as_mut().ok_or_else(|| Error::device("device is not open"))
    }

    /// `wait_for_token`, with what is said above, and an end in every state of the
    /// parser: a stray `bb` never comes back to where C looks at its time. A whole
    /// packet has to come, of this wait: in C a first byte that is no `a5` also looks
    /// at the last wait's token. A token that completes before `ignore_until` goes
    /// by.
    fn wait(
        &mut self,
        tokens: &[u8],
        limit: Duration,
        ignore_until: Option<Instant>,
    ) -> Result<Waited> {
        let silence = self.silence;
        let started = Instant::now();
        let mut last_byte = started;
        self.parser.state = State::Init;
        self.parser.token = 0;
        let mut ignored = Vec::new();
        loop {
            if started.elapsed() > limit {
                return Ok(Waited::TimeUp);
            }
            let mut byte = [0];
            if self.port()?.read(&mut byte, WAIT_READ)? == 0 {
                if last_byte.elapsed() >= silence {
                    return Ok(Waited::Silent);
                }
                continue;
            }
            last_byte = Instant::now();
            ignored.clear();
            self.parser.byte(byte[0], false, last_byte, &mut ignored);
            if self.parser.state != State::Init || !tokens.contains(&self.parser.token) {
                continue;
            }
            if ignore_until.is_some_and(|until| last_byte < until) {
                // A later byte in Init must not take it again.
                self.parser.token = 0;
                continue;
            }
            log::trace!("wait_for_token: got token 0x{:02x}", self.parser.token);
            return Ok(Waited::Found(self.parser.token));
        }
    }

    /// The token saying what `setting` is now, for a get or ahead of a first command.
    fn current(&mut self, setting: &Setting, tokens: &[u8], limit: Duration) -> Result<u8> {
        match self.wait(tokens, limit, None)? {
            Waited::Found(token) => Ok(token),
            Waited::TimeUp => Err(Error::device(format!(
                "the meter did not say {} in {}",
                setting.what,
                spoken(limit)
            ))),
            Waited::Silent => {
                Err(Error::device(format!("no byte from the meter in {}", spoken(self.silence))))
            }
        }
    }

    /// The next announcement after a command, which may have changed the state.
    fn announcement(
        &mut self,
        setting: &Setting,
        tokens: &[u8],
        ignore_until: Option<Instant>,
    ) -> Result<u8> {
        let limit = self.announce.unwrap_or(setting.announce);
        match self.wait(tokens, limit, ignore_until)? {
            Waited::Found(token) => Ok(token),
            Waited::TimeUp => Err(Error::device(format!(
                "the meter did not say {} in {}; its state is unknown",
                setting.what,
                spoken(limit)
            ))),
            Waited::Silent => Err(Error::device(format!(
                "no byte from the meter in {}; its state is unknown",
                spoken(self.silence)
            ))),
        }
    }

    /// `cem_dt_885x_toggle`, but it can't be sent as C sends it. A meter takes a
    /// command when it takes it, and then a second sent while the first is on its way
    /// undoes it: `dt8852.py:184-196` warns of the same "overshoot". So one command
    /// per announcement, sent as one has just come and so well ahead of the next. What
    /// completes within a packet's time of a command began before it and says nothing.
    /// Once a token says `wanted`, nothing more goes out, and the next one has to agree:
    /// one that doesn't was a command taken late. C takes the first token, and says
    /// success with the meter in the other state.
    fn toggle(&mut self, setting: &Setting, wanted: u8, tokens: &[u8]) -> Result<()> {
        let bound = self.toggle_bound.unwrap_or(setting.bound);
        let started = Instant::now();
        loop {
            if started.elapsed() > bound {
                return Err(Error::device(format!(
                    "the meter did not take the command in {}; its state is unknown",
                    spoken(bound)
                )));
            }
            self.port()?.write(&[setting.cmd])?;
            let ignore_until = Instant::now() + PACKET_TIME;
            if self.announcement(setting, tokens, Some(ignore_until))? == wanted
                && self.announcement(setting, tokens, None)? == wanted
            {
                return Ok(());
            }
        }
    }

    /// `recording_get`. Once a capture has seen it, the state that says, where C
    /// reads whichever token came last (`protocol.c:487`).
    pub fn recording_get(&mut self) -> Result<bool> {
        if let Some(on) = self.parser.recording {
            return Ok(on);
        }
        let tokens = [TOKEN_RECORDING_ON, TOKEN_RECORDING_OFF];
        Ok(self.current(&RECORDING, &tokens, RECORDING.announce)? == TOKEN_RECORDING_ON)
    }

    /// `recording_set`. Remembers what it did, as C does not: a later set of what
    /// the last capture saw would do nothing.
    pub fn recording_set(&mut self, on: bool) -> Result<()> {
        let wanted = if on { TOKEN_RECORDING_ON } else { TOKEN_RECORDING_OFF };
        let tokens = [TOKEN_RECORDING_ON, TOKEN_RECORDING_OFF];
        match self.parser.recording {
            None => {
                if self.current(&RECORDING, &tokens, self.wait_bound)? == wanted {
                    return Ok(());
                }
            }
            Some(now) if now == on => return Ok(()),
            Some(_) => {}
        }
        self.toggle(&RECORDING, wanted, &tokens)?;
        if self.parser.recording.is_some() {
            self.parser.recording = Some(on);
        }
        Ok(())
    }

    /// `weight_freq_get`: whether it is C.
    pub fn weight_freq_get(&mut self) -> Result<bool> {
        self.weight_get(
            &FREQ,
            MqFlags::SPL_FREQ_WEIGHT_A,
            MqFlags::SPL_FREQ_WEIGHT_C,
            [TOKEN_WEIGHT_FREQ_A, TOKEN_WEIGHT_FREQ_C],
        )
    }

    /// `weight_time_get`: whether it is slow.
    pub fn weight_time_get(&mut self) -> Result<bool> {
        self.weight_get(
            &TIME,
            MqFlags::SPL_TIME_WEIGHT_F,
            MqFlags::SPL_TIME_WEIGHT_S,
            [TOKEN_WEIGHT_TIME_FAST, TOKEN_WEIGHT_TIME_SLOW],
        )
    }

    fn weight_get(
        &mut self,
        setting: &Setting,
        first: MqFlags,
        second: MqFlags,
        tokens: [u8; 2],
    ) -> Result<bool> {
        if self.parser.flags.contains(first) {
            return Ok(false);
        }
        if self.parser.flags.contains(second) {
            return Ok(true);
        }
        Ok(self.current(setting, &tokens, self.wait_bound)? == tokens[1])
    }

    /// `weight_freq_set`.
    pub fn weight_freq_set(&mut self, c_weight: bool) -> Result<()> {
        self.weight_set(
            &FREQ,
            c_weight,
            [MqFlags::SPL_FREQ_WEIGHT_A, MqFlags::SPL_FREQ_WEIGHT_C],
            [TOKEN_WEIGHT_FREQ_A, TOKEN_WEIGHT_FREQ_C],
        )
    }

    /// `weight_time_set`.
    pub fn weight_time_set(&mut self, slow: bool) -> Result<()> {
        self.weight_set(
            &TIME,
            slow,
            [MqFlags::SPL_TIME_WEIGHT_F, MqFlags::SPL_TIME_WEIGHT_S],
            [TOKEN_WEIGHT_TIME_FAST, TOKEN_WEIGHT_TIME_SLOW],
        )
    }

    fn weight_set(
        &mut self,
        setting: &Setting,
        second: bool,
        flags: [MqFlags; 2],
        tokens: [u8; 2],
    ) -> Result<()> {
        let (wanted, other) = if second { (flags[1], flags[0]) } else { (flags[0], flags[1]) };
        let token = tokens[usize::from(second)];
        if self.parser.flags.contains(wanted) {
            return Ok(());
        }
        let known = self.parser.flags.contains(other);
        if !known && self.current(setting, &tokens, self.wait_bound)? == token {
            return Ok(());
        }
        self.toggle(setting, token, &tokens)?;
        // C leaves the old flag, so a later set of it would do nothing. Only one
        // known is kept known: holdmode_set's test of no flags at all needs the
        // rest as C has them, or it sends other toggles.
        if known {
            self.parser.set(wanted, other);
        }
        Ok(())
    }

    /// `holdmode_get`: max and min, as flags.
    pub fn holdmode_get(&mut self) -> Result<MqFlags> {
        if self.parser.flags.is_empty() {
            let tokens = [TOKEN_HOLD_MAX, TOKEN_HOLD_MIN, TOKEN_HOLD_NONE];
            let token = self.current(&HOLD, &tokens, self.wait_bound)?;
            if token == TOKEN_HOLD_MAX {
                self.parser.flags = MqFlags::MAX;
            } else if token == TOKEN_HOLD_MIN {
                self.parser.flags = MqFlags::MIN;
            }
        }
        Ok(self.held())
    }

    fn held(&self) -> MqFlags {
        let flags = self.parser.flags;
        let max = if flags.contains(MqFlags::MAX) { MqFlags::MAX } else { MqFlags::empty() };
        let min = if flags.contains(MqFlags::MIN) { MqFlags::MIN } else { MqFlags::empty() };
        max | min
    }

    /// `holdmode_set`: `MAX`, `MIN`, or no flag for neither.
    pub fn holdmode_set(&mut self, mode: MqFlags) -> Result<()> {
        let tokens = [TOKEN_HOLD_MAX, TOKEN_HOLD_MIN, TOKEN_HOLD_NONE];
        let wanted = if mode == MqFlags::MAX {
            TOKEN_HOLD_MAX
        } else if mode == MqFlags::MIN {
            TOKEN_HOLD_MIN
        } else {
            TOKEN_HOLD_NONE
        };
        let known = !self.parser.flags.is_empty();
        if known {
            if self.held() == mode {
                return Ok(());
            }
        } else if self.current(&HOLD, &tokens, self.wait_bound)? == wanted {
            return Ok(());
        }
        self.toggle(&HOLD, wanted, &tokens)?;
        if known {
            self.parser.set(mode, MqFlags::MAX | MqFlags::MIN);
        }
        Ok(())
    }

    /// `meas_range_get`, as low and high.
    pub fn meas_range_get(&mut self) -> Result<(u64, u64)> {
        let token = match self.parser.range {
            Some(token) => token,
            None => {
                let token = self.current(&RANGE, &RANGES.map(|r| r.2), self.wait_bound)?;
                self.parser.range = Some(token);
                token
            }
        };
        RANGES
            .iter()
            .find(|r| r.2 == token)
            .map(|r| (r.0, r.1))
            .ok_or_else(|| Error::device("no such range"))
    }

    /// `meas_range_set`. Remembers the range it set, where C leaves the old one for
    /// a get after it (`protocol.c:806`). C resends every 11 ms and looks only for the
    /// range it wants, and on a meter that takes them that goes round the four ranges
    /// past it; this is [`Link::toggle`], with one command per announcement.
    pub fn meas_range_set(&mut self, token: u8) -> Result<()> {
        log::debug!("want 0x{token:02x}");
        let tokens = RANGES.map(|r| r.2);
        if self.parser.range.is_none() {
            let now = self.current(&RANGE, &tokens, RANGE.announce)?;
            self.parser.range = Some(now);
        }
        if self.parser.range == Some(token) {
            return Ok(());
        }
        self.toggle(&RANGE, token, &tokens)?;
        self.parser.range = Some(token);
        Ok(())
    }

    /// `power_off`, up to the port's turn: flush, tell it, wait as long as the next
    /// token takes and look for one. Where C goes on for ever, a meter that talks
    /// through [`TOGGLE_BOUND`] fails.
    pub fn power_off(&mut self) -> Result<()> {
        let bound = self.toggle_bound.unwrap_or(TOGGLE_BOUND);
        let started = Instant::now();
        loop {
            let port = self.port()?;
            port.purge()?;
            port.write(&[CMD_TOGGLE_POWER_OFF])?;
            std::thread::sleep(POWER_OFF_WAIT);
            let mut byte = [0];
            if !matches!(port.read(&mut byte, Duration::ZERO), Ok(1)) {
                // No longer responding. Good.
                return Ok(());
            }
            if started.elapsed() > bound {
                return Err(Error::device(format!(
                    "the meter did not turn off in {}",
                    spoken(bound)
                )));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(p: &mut Parser, bytes: &[u8]) -> Vec<Event> {
        let mut events = Vec::new();
        let now = Instant::now();
        for b in bytes {
            p.byte(*b, true, now, &mut events);
        }
        events
    }

    fn flags(list: &[MqFlags]) -> MqFlags {
        list.iter().fold(MqFlags::empty(), |all, f| all | *f)
    }

    #[test]
    fn levels_are_bcd_and_what_c_makes_of_the_rest() {
        assert!((decode_level(0x06, 0x55) - 65.5).abs() < 1e-5);
        assert!((decode_level(0x12, 0x98) - 129.8).abs() < 1e-5);
        assert_eq!(decode_level(0, 0), 0.0);
        // Not BCD: each nibble still counts in its place.
        assert_eq!(decode_level(0xff, 0xff), 1500.0 + 150.0 + 15.0 + 1.5);
        // The last step is in double: 0.1 narrowed from a double sum.
        assert_eq!(decode_level(0x00, 0x01), 0.1_f32);
        assert_eq!(decode_level(0x06, 0x61).to_bits(), 66.1_f32.to_bits());
    }

    #[test]
    fn every_token_of_the_wiki_has_its_length() {
        for (token, len) in [(0x06, 3), (0x0d, 2), (0x0b, 1), (0x1b, 1), (0x1c, 1), (0x02, 0)] {
            assert_eq!(token_payload_len(token), Some(len));
        }
        for token in [0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x0a, 0x0c, 0x0e, 0x0f, 0x11, 0x19, 0x1a]
        {
            assert_eq!(token_payload_len(token), Some(0), "{token:02x}");
        }
        for token in [0x1f, 0x30, 0x40, 0x4b, 0x4c] {
            assert_eq!(token_payload_len(token), Some(0), "{token:02x}");
        }
        for token in [0x00, 0x01, 0x10, 0x12, 0xa5, 0xe1] {
            assert_eq!(token_payload_len(token), None, "{token:02x}");
        }
    }

    #[test]
    fn a_level_goes_out_on_the_display_token_with_the_flags_so_far() {
        let mut p = Parser::default();
        let events = feed(&mut p, &[0xa5, 0x1b, 0, 0xa5, 0x03, 0xa5, 0x0d, 0x06, 0x55, 0xa5, 0x0c]);
        assert_eq!(
            events,
            [Event::Levels(
                vec![65.5],
                flags(&[MqFlags::SPL_FREQ_WEIGHT_A, MqFlags::SPL_TIME_WEIGHT_S])
            )]
        );
        assert_eq!(p.flags, flags(&[MqFlags::SPL_FREQ_WEIGHT_A, MqFlags::SPL_TIME_WEIGHT_S]));
    }

    #[test]
    fn each_token_sets_its_flags_and_clears_the_other() {
        let mut p = Parser::default();
        feed(&mut p, &[0xa5, 0x02, 0xa5, 0x1b, 0]);
        feed(&mut p, &[0xa5, 0x03, 0xa5, 0x1c, 0]);
        assert_eq!(p.flags, flags(&[MqFlags::SPL_TIME_WEIGHT_S, MqFlags::SPL_FREQ_WEIGHT_C]));
        feed(&mut p, &[0xa5, 0x02, 0xa5, 0x1b, 0]);
        assert_eq!(p.flags, flags(&[MqFlags::SPL_TIME_WEIGHT_F, MqFlags::SPL_FREQ_WEIGHT_A]));
        let mut p = Parser::default();
        feed(&mut p, &[0xa5, 0x04]);
        assert_eq!(p.flags, flags(&[MqFlags::HOLD, MqFlags::MAX]));
        feed(&mut p, &[0xa5, 0x05]);
        // A byte that is no ff takes HOLD off, then the token puts it back.
        assert_eq!(p.flags, flags(&[MqFlags::HOLD, MqFlags::MIN]));
        feed(&mut p, &[0xa5, 0x0e]);
        assert_eq!(p.flags, MqFlags::empty());
    }

    #[test]
    fn the_settings_are_kept() {
        let mut p = Parser::default();
        feed(&mut p, &[0xa5, 0x0a, 0xa5, 0x4c]);
        assert_eq!(p.recording, Some(true));
        assert_eq!(p.range, Some(TOKEN_MEAS_RANGE_80_130));
        feed(&mut p, &[0xa5, 0x1a, 0xa5, 0x30, 0xa5, 0x07, 0xa5, 0x08, 0xa5, 0x09, 0xa5, 0x0f]);
        assert_eq!(p.recording, Some(false));
        assert_eq!(p.range, Some(TOKEN_MEAS_RANGE_30_80));
        assert_eq!(p.flags, MqFlags::empty(), "battery, store and threshold say nothing");
    }

    #[test]
    fn max_or_min_hold_drops_the_bar_graph_but_not_the_display() {
        for token in [TOKEN_HOLD_MAX, TOKEN_HOLD_MIN] {
            let mut p = Parser::default();
            let events = feed(
                &mut p,
                &[
                    0xa5, token, 0xa5, 0x0d, 0x07, 0x02, 0xa5, 0x0c, 0xa5, 0x0d, 0x07, 0x03, 0xa5,
                    0x0b, 0,
                ],
            );
            assert_eq!(events.len(), 1);
            let Event::Levels(levels, _) = &events[0] else { panic!("{events:?}") };
            assert_eq!(levels, &[70.3]);
        }
    }

    #[test]
    fn an_unknown_token_takes_the_next_commands_start_with_it() {
        let mut p = Parser::default();
        let events = feed(&mut p, &[0xa5, 0xe1, 1, 2, 0xa5, 0x0d, 0x07, 0x02, 0xa5, 0x0c]);
        // The a5 ended the unknown one, so 0d, 07 and 02 meant nothing, and the
        // display token that follows has no level to say.
        assert_eq!(events, Vec::<Event>::new());
        // The next level does.
        let events = feed(&mut p, &[0xa5, 0x0d, 0x06, 0x55, 0xa5, 0x0c]);
        assert_eq!(events, [Event::Levels(vec![65.5], MqFlags::empty())]);
    }

    #[test]
    fn an_unknown_token_past_the_buffer_ends_without_writing_past_it() {
        let mut p = Parser::default();
        let mut bytes = vec![0xa5, 0xe1];
        bytes.extend([7; BUF_SIZE + 1]);
        feed(&mut p, &bytes);
        assert_eq!(p.state, State::Init);
        assert_eq!(p.buf_len, BUF_SIZE);
    }

    #[test]
    fn a_token_that_is_the_sync_byte_is_unknown() {
        let mut p = Parser::default();
        assert_eq!(feed(&mut p, &[0xa5, 0xa5, 0xa5]), Vec::<Event>::new());
        assert_eq!(p.state, State::Init);
    }

    #[test]
    fn hold_cuts_a_packet_and_repeats_the_level_only_after_its_interval() {
        let mut p = Parser::default();
        let t = Instant::now();
        let mut events = Vec::new();
        for b in [0xa5, 0x0d, 0x06, 0x55, 0xa5, 0x0d, 0x07] {
            p.byte(b, true, t, &mut events);
        }
        p.byte(0xff, true, t, &mut events);
        assert_eq!(p.state, State::Init, "starts from scratch");
        assert!(p.flags.contains(MqFlags::HOLD));
        assert_eq!(events, Vec::<Event>::new());
        // Not at 50 ms: over it.
        p.byte(0xff, true, t + Duration::from_millis(50), &mut events);
        assert_eq!(events, Vec::<Event>::new());
        p.byte(0xff, true, t + Duration::from_millis(51), &mut events);
        assert_eq!(events, [Event::Levels(vec![65.5], MqFlags::HOLD)]);
        // Counted from that repeat.
        p.byte(0xff, true, t + Duration::from_millis(100), &mut events);
        assert_eq!(events.len(), 1);
        p.byte(0xff, true, t + Duration::from_millis(102), &mut events);
        assert_eq!(events.len(), 2);
        // Any other byte ends the hold, and its time.
        p.byte(0x00, true, t + Duration::from_millis(103), &mut events);
        assert!(!p.flags.contains(MqFlags::HOLD));
        p.byte(0xff, true, t + Duration::from_millis(500), &mut events);
        assert_eq!(events.len(), 2, "a first notice again");
    }

    #[test]
    fn in_a_wait_hold_says_nothing_but_the_flag() {
        let mut p = Parser::default();
        let t = Instant::now();
        let mut events = Vec::new();
        p.byte(0xff, false, t, &mut events);
        p.byte(0xff, false, t + Duration::from_millis(60), &mut events);
        assert_eq!(events, Vec::<Event>::new());
        assert_eq!(p.token, TOKEN_MEAS_WAS_READOUT);
        assert!(p.flags.contains(MqFlags::HOLD));
    }

    #[test]
    fn a_wait_leaves_the_settings_as_c_does() {
        let mut p = Parser::default();
        let mut events = Vec::new();
        for b in [0xa5, 0x1c, 0, 0xa5, 0x4c, 0xa5, 0x0a, 0xa5, 0x0d, 1, 2, 0xa5, 0x0b, 0] {
            p.byte(b, false, Instant::now(), &mut events);
        }
        assert_eq!(events, Vec::<Event>::new());
        assert_eq!(p.flags, MqFlags::empty());
        assert_eq!(p.range, None);
        assert_eq!(p.recording, None);
    }

    #[test]
    fn a_log_gives_intervals_levels_and_its_end() {
        let mut p = Parser::default();
        let log = [
            0xbb, 0, 112, 0xaa, 0x26, 0x11, 5, 0x12, 0x34, 0x56, 0x10, 0xac, 0x04, 0x00, 0x04,
            0x01, 0xcc, 0x26, 0x11, 5, 0x12, 0x34, 0x56, 0x02, 0xac, 0x04, 0x02, 0x45, 0xdd,
        ];
        let events = feed(&mut p, &log);
        assert_eq!(
            events,
            [
                Event::Interval(16_000),
                Event::Levels(vec![40.0, 40.1], MqFlags::SPL_FREQ_WEIGHT_A),
                Event::Interval(2000),
                // The half level at the end is dropped.
                Event::Levels(vec![40.2], MqFlags::SPL_FREQ_WEIGHT_C),
                Event::End,
            ]
        );
        assert_eq!(p.state, State::Init);
    }

    #[test]
    fn a_display_token_ahead_of_any_level_says_nothing_and_a_start_forgets_it() {
        let mut p = Parser::default();
        assert_eq!(feed(&mut p, &[0xa5, 0x0c, 0xa5, 0x0b, 0]), Vec::<Event>::new());
        assert_eq!(feed(&mut p, &[0xa5, 0x0d, 0x06, 0x55, 0xa5, 0x0c]).len(), 1);
        p.start();
        assert_eq!(feed(&mut p, &[0xa5, 0x0c]), Vec::<Event>::new());
        // Nor does a hold's repeat.
        let t = Instant::now();
        let mut events = Vec::new();
        p.byte(0xff, true, t, &mut events);
        p.byte(0xff, true, t + Duration::from_millis(60), &mut events);
        assert_eq!(events, Vec::<Event>::new());
    }

    #[test]
    fn a_second_record_head_with_no_levels_is_read_whole() {
        let mut p = Parser::default();
        let head = [0xaa, 0x26, 0x11, 5, 0x12, 0x34, 0x56, 0x07];
        let mut log = vec![0xbb, 0, 100];
        log.extend(head);
        log.extend(head);
        assert_eq!(feed(&mut p, &log), [Event::Interval(7000), Event::Interval(7000)]);
    }

    #[test]
    fn times_are_said_in_the_unit_they_are_set_in() {
        assert_eq!(spoken(Duration::from_secs(5)), "5 s");
        assert_eq!(spoken(Duration::from_millis(600)), "600 ms");
    }

    #[test]
    fn a_log_says_its_levels_50_at_a_time() {
        let mut p = Parser::default();
        let mut log = vec![0xbb, 0, 100, 0xac];
        log.extend([0x05, 0x00].repeat(120));
        log.push(0xdd);
        let events = feed(&mut p, &log);
        let lengths: Vec<usize> = events
            .iter()
            .filter_map(|e| if let Event::Levels(l, _) = e { Some(l.len()) } else { None })
            .collect();
        assert_eq!(lengths, [50, 50, 20]);
        assert_eq!(events.last(), Some(&Event::End));
    }

    #[test]
    fn a_full_packet_ending_a_record_sends_no_empty_one() {
        let mut p = Parser::default();
        let mut log = vec![0xbb, 0, 100, 0xac];
        log.extend([0x05, 0x00].repeat(50));
        log.push(0xdd);
        let events = feed(&mut p, &log);
        assert_eq!(events.len(), 2);
    }

    #[test]
    fn a_record_head_with_no_record_token_writes_nothing_past_the_buffer() {
        let mut p = Parser::default();
        let mut log = vec![0xbb, 0, 100];
        log.extend([0x12; 3 * BUF_SIZE]);
        let events = feed(&mut p, &log);
        assert_eq!(events, Vec::<Event>::new());
        assert_eq!(p.buf_len, RECORD_HEAD);
        // Levels may still follow, and the end.
        let events = feed(&mut p, &[0xac, 0x05, 0x00, 0xdd]);
        assert_eq!(events.len(), 2);
    }

    #[test]
    fn an_empty_log_with_the_wikis_stray_token_ends() {
        let mut p = Parser::default();
        assert_eq!(feed(&mut p, &[0xbb, 0, 0x64, 0xaa, 0xdd]), [Event::End]);
    }

    #[test]
    fn the_log_header_is_seen_and_the_start_resets() {
        let mut p = Parser::default();
        feed(&mut p, &[0xbb]);
        assert!(p.in_log_header());
        p.start();
        assert!(!p.in_log_header());
        assert_eq!(p.state, State::Init);
    }

    #[test]
    fn the_times_are_cs() {
        assert_eq!(PACKET_TIME, Duration::from_millis(6));
        assert_eq!(POWER_OFF_WAIT, Duration::from_millis(25));
        assert_eq!(HOLD_REPEAT_INTERVAL, Duration::from_millis(50));
        assert_eq!(WAIT_SILENCE, Duration::from_secs(1));
        assert_eq!(WAIT_BOUND, Duration::from_secs(2));
        assert_eq!(TOGGLE_BOUND, Duration::from_secs(5));
        assert_eq!(SLOW_TOGGLE_BOUND, Duration::from_secs(15));
        for (setting, cmd, announce, bound) in [
            (FREQ, 0x99, 600, 5),
            (HOLD, 0x11, 600, 5),
            (RANGE, 0x88, 600, 5),
            (TIME, 0x77, 1200, 15),
            (RECORDING, 0x55, 1200, 15),
        ] {
            assert_eq!(setting.cmd, cmd, "{}", setting.what);
            assert_eq!(setting.announce, Duration::from_millis(announce), "{}", setting.what);
            assert_eq!(setting.bound, Duration::from_secs(bound), "{}", setting.what);
        }
    }
}
