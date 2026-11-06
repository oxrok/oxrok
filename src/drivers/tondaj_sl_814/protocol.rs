// Counterpart in sigrok:
//   libsigrok/src/hardware/tondaj-sl-814/protocol.c,
//   libsigrok/src/hardware/tondaj-sl-814/protocol.h,
//   Copyright (C) 2012 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Martin Ling, Daniel Elstner, Aurelien Jacobs,
//   Bert Vermeulen, Gerhard Sittig, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire protocol of a Tondaj SL-814 and what a capture does with it, with no
//! port and no clock of its own.
//!
//! Every command is three bytes ending in CR. Init is `10 04 0D`, answered `05
//! 0D`; a measurement is `30 00 0D`, answered `AA BB 01 0D`, the third byte being
//! the second of the request plus one. The sigrok wiki's page gives both, with
//! examples of a reading: AA bit 7 is C weighting and bit 3 slow, bits 5 and 4 a
//! level nothing uses, bit 6 unknown, and bits 2 to 0 with BB eleven bits of tenths
//! of a dB. It says "BCD", where its examples and C are plain binary.
//!
//! Nothing checks a reading: there is no checksum, and parity is not looked at, so
//! a corrupted AA or BB cannot be told. The form of a reply can't always tell a
//! shifted one either: a reading of `01 0D` after two stray bytes passes as what
//! those bytes say.
//!
//! ZZ is the wiki's sequence number, "0-255". C always sends 0, and so does this,
//! except on asking again after a silence: see [`Exchange`]. The wiki's script sends
//! 1 and shows `02` come back; other values rest on its words alone.

use crate::core::MqFlags;
use std::time::{Duration, Instant};

/// C's `SERIALCOMM`.
pub const SERIALCOMM: &str = "9600/8e1";
/// The init command.
pub const INIT: [u8; 3] = [0x10, 0x04, 0x0d];
/// The request for a measurement, with C's sequence number.
pub const REQUEST: [u8; 3] = [0x30, 0x00, 0x0d];
/// What init is answered with.
pub const INIT_REPLY: [u8; 2] = [0x05, 0x0d];
/// How long a reply to a measurement is.
pub const REPLY_LEN: usize = 4;

/// The session source's timeout in `dev_acquisition_start`: what a command waits
/// after the last reply, or from the start.
pub const PACE: Duration = Duration::from_millis(500);
/// How long the line is quiet before a command, after bytes nothing asked for: a
/// USB cable may hold a reply's tail for tens of ms.
pub const QUIET: Duration = Duration::from_millis(100);
/// How long a command waits before it goes again, and how long a capture on a
/// sample limit alone goes with nothing usable before its first sample. C has no
/// bound.
pub const NO_ANSWER: Duration = Duration::from_secs(5);
/// How long the wait for an answer gets, doubling while nothing is heard.
pub const LONGEST_WAIT: Duration = Duration::from_secs(30);
/// How long a capture on a sample limit alone goes with nothing usable, once it has
/// had a sample: only this could end it.
pub const GIVE_UP: Duration = Duration::from_secs(600);

/// One reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// In dB.
    pub value: f32,
    /// The weightings.
    pub flags: MqFlags,
}

/// `parse_packet`: the reading in a reply's first two bytes.
pub fn parse_reading(a: u8, b: u8) -> Reading {
    let weight =
        if a & 0x80 == 0 { MqFlags::SPL_FREQ_WEIGHT_A } else { MqFlags::SPL_FREQ_WEIGHT_C };
    let time = if a & 0x08 == 0 { MqFlags::SPL_TIME_WEIGHT_F } else { MqFlags::SPL_TIME_WEIGHT_S };
    let tenths = (u16::from(a & 0x07) << 8) | u16::from(b);
    Reading { value: f32::from(tenths) / 10.0, flags: weight | time }
}

/// A command to send.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// [`INIT`].
    Init,
    /// [`REQUEST`].
    Request,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Waiting to send this, reading away what comes.
    Idle(Command),
    /// This is out, and what comes is its reply.
    Await(Command),
}

/// What bytes that came amount to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Heard {
    /// Part of a reply.
    Part,
    /// Nothing asked for them.
    Dropped,
    /// The right reply to init.
    InitOk,
    /// Not the reply to init, so the next command is init again.
    BadInit([u8; INIT_REPLY.len()]),
    /// A reading's two bytes.
    Reading(u8, u8),
    /// The reply to a request asked again since, by its sequence number. The
    /// command out is still out.
    Late(u8),
    /// Not the reply to a request, so the next command is a request.
    BadReply([u8; REPLY_LEN]),
}

/// What is to be done, there being nothing to read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// Go on reading.
    Wait,
    /// Send this: its bytes from [`Exchange::bytes`], and then tell [`Exchange::sent`].
    Send(Command),
    /// Say once that nothing usable has come, and go on.
    Warn,
    /// A time limit has met a wait with nothing to show: say so and end well.
    TimeUp,
    /// The capture failed, after this long.
    GiveUp(GiveUp, Duration),
}

/// Why a capture gave up, each said in its own words.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GiveUp {
    /// Not a byte came for init.
    NoInitReply,
    /// Not a byte came for a request.
    NoReply,
    /// Bytes came, and none of them was a reading or a right init reply.
    NoUsableAnswer,
}

impl GiveUp {
    /// The words of a capture that gave up after `after`.
    pub fn message(self, after: Duration) -> String {
        let secs = after.as_secs_f32();
        match self {
            GiveUp::NoInitReply => format!("no answer to init for {secs} s"),
            GiveUp::NoReply => format!("no answer to a request for {secs} s"),
            GiveUp::NoUsableAnswer => format!("no usable answer for {secs} s"),
        }
    }
}

/// The times of an [`Exchange`], which a test may want shorter.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// [`PACE`].
    pub pace: Duration,
    /// [`QUIET`].
    pub quiet: Duration,
    /// [`NO_ANSWER`].
    pub no_answer: Duration,
    /// [`LONGEST_WAIT`].
    pub longest_wait: Duration,
    /// [`GIVE_UP`].
    pub give_up: Duration,
}

impl Default for Timing {
    fn default() -> Timing {
        Timing {
            pace: PACE,
            quiet: QUIET,
            no_answer: NO_ANSWER,
            longest_wait: LONGEST_WAIT,
            give_up: GIVE_UP,
        }
    }
}

/// What can end a capture, as far as a silence goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// Only a sample limit could end it: a meter that stops must be given up on.
    pub samples_only: bool,
    /// The time limit in ms, 0 being none.
    pub msec: u64,
}

/// C's state machine with one command out at a time, as a value over `Instant`.
///
/// What comes with nothing out is read away, and a command goes only after an empty
/// read past its time, so a stray byte costs one reading, where in C it has every
/// later reply cut in the same wrong place.
///
/// C never asks again, and waits for ever. This asks again after [`NO_ANSWER`] of
/// silence, one out at a time, and the wait doubles to [`LONGEST_WAIT`] while nothing
/// is heard. A reply within the first wait sets it back, and a slower one leaves it,
/// so a meter slower than that settles with no more asking. A request asked again
/// has a new sequence number, so that the late reply to the last is known for what
/// it is and passed over: with the same number it would answer the next request, and
/// every later reading would be one reply behind, looking fresh. Numbers run on
/// through a capture, skipping C's 0, so that none is met twice for 255 of these.
///
/// How a silence ends depends on what else could: see [`Exchange::bound`].
#[derive(Debug)]
pub struct Exchange {
    phase: Phase,
    timing: Timing,
    limits: Limits,
    started: Instant,
    reply: [u8; REPLY_LEN],
    len: usize,
    /// When the next command is due.
    due: Instant,
    /// The last reading handed on, or right init reply.
    usable: Instant,
    /// The last command, once its write had returned.
    sent: Instant,
    /// The command that first asked for what is out, once its write had returned.
    asked: Instant,
    /// A byte has come since `usable`.
    heard: bool,
    /// What a command out waits before it goes again.
    wait: Duration,
    /// The sequence number of the request out or last out.
    sequence: u8,
    /// The last number used to ask again, 0 being none.
    last_again: u8,
    /// The numbers have come round, so all of them have been used.
    wrapped: bool,
    /// A sample has been handed on.
    sampled: bool,
    warned: bool,
}

impl Exchange {
    /// As a capture starts: init is the first command, after a pace.
    pub fn new(timing: Timing, limits: Limits, now: Instant) -> Exchange {
        Exchange {
            phase: Phase::Idle(Command::Init),
            timing,
            limits,
            started: now,
            reply: [0; REPLY_LEN],
            len: 0,
            due: now + timing.pace,
            usable: now,
            sent: now,
            asked: now,
            heard: false,
            wait: timing.no_answer,
            sequence: 0,
            last_again: 0,
            wrapped: false,
            sampled: false,
            warned: false,
        }
    }

    /// How many bytes to read at most now, `room` being what the caller holds.
    pub fn want(&self, room: usize) -> usize {
        match self.phase {
            Phase::Idle(_) => room,
            Phase::Await(Command::Init) => INIT_REPLY.len() - self.len,
            Phase::Await(Command::Request) => REPLY_LEN - self.len,
        }
    }

    /// How long the next read may wait: up to `most`, less when a command is due
    /// sooner, and nothing once it is due, as a read goes ahead of the timer.
    pub fn read_wait(&self, most: Duration, now: Instant) -> Duration {
        match self.phase {
            Phase::Idle(_) => self.due.saturating_duration_since(now).min(most),
            Phase::Await(_) => most,
        }
    }

    /// The command that is out and not answered.
    pub fn owed(&self) -> Option<Command> {
        match self.phase {
            Phase::Idle(_) => None,
            Phase::Await(command) => Some(command),
        }
    }

    /// Bytes came, no more than [`Exchange::want`] says.
    pub fn take(&mut self, data: &[u8], now: Instant) -> Heard {
        self.heard = true;
        let command = match self.phase {
            Phase::Idle(_) => {
                self.due = self.due.max(now + self.timing.quiet);
                return Heard::Dropped;
            }
            Phase::Await(command) => command,
        };
        let end = self.len + data.len();
        self.reply[self.len..end].copy_from_slice(data);
        self.len = end;
        match command {
            Command::Init if self.len == INIT_REPLY.len() => {
                let reply = [self.reply[0], self.reply[1]];
                if reply == INIT_REPLY {
                    self.answered(Command::Request, now);
                    self.settle_wait(now);
                    self.usable = now;
                    self.heard = false;
                    Heard::InitOk
                } else {
                    self.answered(Command::Init, now);
                    Heard::BadInit(reply)
                }
            }
            Command::Request if self.len == REPLY_LEN => {
                let reply = self.reply;
                if reply[3] == 0x0d && reply[2] == self.sequence.wrapping_add(1) {
                    self.answered(Command::Request, now);
                    self.settle_wait(now);
                    Heard::Reading(reply[0], reply[1])
                } else if reply[3] == 0x0d && self.was_asked(reply[2].wrapping_sub(1)) {
                    self.len = 0;
                    Heard::Late(reply[2].wrapping_sub(1))
                } else {
                    self.answered(Command::Request, now);
                    Heard::BadReply(reply)
                }
            }
            _ => Heard::Part,
        }
    }

    /// Whether a request of this capture may have had this number, other than the
    /// one out. C's 0 only counts once something has been asked again.
    fn was_asked(&self, sequence: u8) -> bool {
        self.last_again != 0 && (self.wrapped || sequence <= self.last_again)
    }

    fn answered(&mut self, next: Command, now: Instant) {
        self.phase = Phase::Idle(next);
        self.len = 0;
        self.due = now + self.timing.pace;
    }

    /// A reply that came within the first wait sets the wait back, and one slower
    /// than that leaves it, so that the next is not asked twice.
    fn settle_wait(&mut self, now: Instant) {
        if now.saturating_duration_since(self.sent) <= self.timing.no_answer {
            self.wait = self.timing.no_answer;
        }
    }

    /// A reading has been handed on, which may have taken a slow client some time:
    /// none of that counts against the meter.
    pub fn delivered(&mut self, now: Instant) {
        self.usable = now;
        self.heard = false;
        self.sampled = true;
        self.due = now + self.timing.pace;
    }

    /// How long nothing usable may go on before the capture fails, if ever. On a
    /// sample limit alone nothing else could end it: before a first sample [`NO_ANSWER`],
    /// so that a meter that never spoke is said at once, and after one [`GIVE_UP`]. A
    /// time limit ends by itself, and `--continuous` runs until it is stopped.
    fn bound(&self) -> Option<Duration> {
        if !self.limits.samples_only {
            None
        } else if self.sampled {
            Some(self.timing.give_up)
        } else {
            Some(self.timing.no_answer)
        }
    }

    /// A read came back empty.
    pub fn empty_read(&mut self, now: Instant) -> Next {
        let command = match self.phase {
            Phase::Idle(command) => {
                return if now >= self.due { Next::Send(command) } else { Next::Wait };
            }
            Phase::Await(command) => command,
        };
        if self.time_is_up(now) && self.overdue(now) {
            return Next::TimeUp;
        }
        let silence = now.saturating_duration_since(self.usable.max(self.asked));
        if let Some(bound) = self.bound()
            && silence >= bound
        {
            return Next::GiveUp(
                match command {
                    _ if self.heard => GiveUp::NoUsableAnswer,
                    Command::Init => GiveUp::NoInitReply,
                    Command::Request => GiveUp::NoReply,
                },
                bound,
            );
        }
        if silence >= self.timing.no_answer && !self.warned {
            self.warned = true;
            return Next::Warn;
        }
        if now.saturating_duration_since(self.sent) >= self.wait {
            return Next::Send(command);
        }
        Next::Wait
    }

    /// Bytes came that were no reading. This runs from the last thing that counted,
    /// so that no flood and no request can put it off.
    pub fn after_junk(&mut self, now: Instant) -> Next {
        let silence = now.saturating_duration_since(self.usable);
        if let Some(bound) = self.bound()
            && silence >= bound
        {
            return Next::GiveUp(GiveUp::NoUsableAnswer, bound);
        }
        if silence >= self.timing.no_answer && !self.warned {
            self.warned = true;
            return Next::Warn;
        }
        Next::Wait
    }

    /// Whether the capture has run its time limit.
    pub fn time_is_up(&self, now: Instant) -> bool {
        self.limits.msec > 0
            && now.saturating_duration_since(self.started) > Duration::from_millis(self.limits.msec)
    }

    /// Whether a command has been out for a whole pace, when C's next pass finds
    /// nothing. A time limit may end a capture here; any sooner would lose a reply
    /// that C takes.
    fn overdue(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.sent) >= self.timing.pace
    }

    /// Whether bytes have come since the last usable thing.
    pub fn heard(&self) -> bool {
        self.heard
    }

    /// The bytes of the command to send now. A request out goes again with a new
    /// sequence number; any other has C's.
    pub fn bytes(&mut self, command: Command) -> [u8; 3] {
        match (command, self.phase) {
            (Command::Init, _) => INIT,
            (Command::Request, Phase::Await(_)) => {
                let (next, wrapped) = match self.last_again.checked_add(1) {
                    Some(next) => (next, self.wrapped),
                    None => (1, true),
                };
                self.last_again = next;
                self.wrapped = wrapped;
                self.sequence = next;
                [REQUEST[0], next, REQUEST[2]]
            }
            (Command::Request, Phase::Idle(_)) => {
                self.sequence = REQUEST[1];
                REQUEST
            }
        }
    }

    /// The command's write has returned. When it was one going again, the next wait
    /// is longer, and part of a reply is dropped: it would read as the head of the next.
    pub fn sent(&mut self, command: Command, now: Instant) {
        if matches!(self.phase, Phase::Await(_)) {
            self.wait = (self.wait * 2).min(self.timing.longest_wait.max(self.timing.no_answer));
        } else {
            self.asked = now;
        }
        self.phase = Phase::Await(command);
        self.len = 0;
        self.sent = now;
    }
}
