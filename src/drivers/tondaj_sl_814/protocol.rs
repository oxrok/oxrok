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

use crate::core::MqFlags;
use std::time::{Duration, Instant};

/// C's `SERIALCOMM`.
pub const SERIALCOMM: &str = "9600/8e1";
/// The init command.
pub const INIT: [u8; 3] = [0x10, 0x04, 0x0d];
/// The request for a measurement.
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
/// How long a capture goes without a reading or a right init reply. C has no bound.
pub const NO_ANSWER: Duration = Duration::from_secs(5);

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

impl Command {
    /// Its bytes.
    pub const fn bytes(self) -> &'static [u8; 3] {
        match self {
            Command::Init => &INIT,
            Command::Request => &REQUEST,
        }
    }
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
    /// Not the reply to a request, so the next command is a request.
    BadReply([u8; REPLY_LEN]),
}

/// What an empty read means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// Go on reading.
    Wait,
    /// Send this, and tell [`Exchange::sent`].
    Send(Command),
    /// No more waiting.
    GiveUp(GiveUp),
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
}

impl Default for Timing {
    fn default() -> Timing {
        Timing { pace: PACE, quiet: QUIET, no_answer: NO_ANSWER }
    }
}

/// C's state machine with one command out at a time, as a value over `Instant`.
///
/// Nothing is asked twice for one silence, so there is no backlog: a slower meter
/// has one reply owed at any time, however slow, and a late reply has no later
/// request to be taken for. What comes with nothing out is read away, and a command
/// goes only after an empty read past its time, so a stray byte costs one reading,
/// where in C it has every later reply cut in the same wrong place.
#[derive(Debug)]
pub struct Exchange {
    phase: Phase,
    timing: Timing,
    reply: [u8; REPLY_LEN],
    len: usize,
    /// When the next command is due.
    due: Instant,
    /// The last reading handed on, or right init reply.
    usable: Instant,
    /// The last command, once its write had returned.
    sent: Instant,
    /// A byte has come since `usable`.
    heard: bool,
}

impl Exchange {
    /// As a capture starts: init is the first command, after a pace.
    pub fn new(timing: Timing, now: Instant) -> Exchange {
        Exchange {
            phase: Phase::Idle(Command::Init),
            timing,
            reply: [0; REPLY_LEN],
            len: 0,
            due: now + timing.pace,
            usable: now,
            sent: now,
            heard: false,
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
                self.answered(Command::Request, now);
                if reply[2] == 0x01 && reply[3] == 0x0d {
                    Heard::Reading(reply[0], reply[1])
                } else {
                    Heard::BadReply(reply)
                }
            }
            _ => Heard::Part,
        }
    }

    fn answered(&mut self, next: Command, now: Instant) {
        self.phase = Phase::Idle(next);
        self.len = 0;
        self.due = now + self.timing.pace;
    }

    /// A reading has been handed on, which may have taken a slow client some time:
    /// none of that counts against the meter.
    pub fn delivered(&mut self, now: Instant) {
        self.usable = now;
        self.heard = false;
        self.due = now + self.timing.pace;
    }

    /// A read came back empty.
    pub fn empty_read(&self, now: Instant) -> Next {
        match self.phase {
            Phase::Idle(command) => {
                if now >= self.due {
                    Next::Send(command)
                } else {
                    Next::Wait
                }
            }
            Phase::Await(command) => {
                let since = self.usable.max(self.sent);
                if now.saturating_duration_since(since) < self.timing.no_answer {
                    Next::Wait
                } else if self.heard {
                    Next::GiveUp(GiveUp::NoUsableAnswer)
                } else if command == Command::Init {
                    Next::GiveUp(GiveUp::NoInitReply)
                } else {
                    Next::GiveUp(GiveUp::NoReply)
                }
            }
        }
    }

    /// Whether bytes that were no reading have gone on for too long. This runs from
    /// the last thing that counted, so that no flood and no request can put it off.
    pub fn junk_too_long(&self, now: Instant) -> bool {
        now.saturating_duration_since(self.usable) >= self.timing.no_answer
    }

    /// Whether a command has been out for a whole pace, when C's next pass finds
    /// nothing. A time limit may end a capture here; any sooner would lose a reply
    /// that C takes.
    pub fn overdue(&self, now: Instant) -> bool {
        self.owed().is_some() && now.saturating_duration_since(self.sent) >= self.timing.pace
    }

    /// Whether bytes have come since the last usable thing.
    pub fn heard(&self) -> bool {
        self.heard
    }

    /// The command's write has returned.
    pub fn sent(&mut self, command: Command, now: Instant) {
        self.phase = Phase::Await(command);
        self.len = 0;
        self.sent = now;
    }
}
