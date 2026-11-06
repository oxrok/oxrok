// Counterpart in sigrok:
//   libsigrok/src/hardware/mic-985xx/api.c,
//   libsigrok/src/hardware/mic-985xx/protocol.c,
//   libsigrok/src/hardware/mic-985xx/protocol.h,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Lars-Peter Clausen, Bert Vermeulen, Martin Ling,
//   Soeren Apel, Daniel Elstner, Gerhard Sittig, Wolfram Sang,
//   Aurelien Jacobs
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The frames of a MIC logger, and the rules of a capture over time, all pure.
//!
//! Told to, with `S 1 M 2 32 3` and a CR, a logger sends readings: `v ttt\r` on
//! a 98581, `v ttt hhh\r` on a 98583, at 38400/8n2. Nothing marks a frame but
//! its form, so a search steps a byte at a time past what does not fit, and a
//! window has to be as long as a frame before it is looked at.
//!
//! A field is three digits, in tenths. C divides it by 10 in integers
//! (protocol.c:83 and :87), which throws away the third digit and prints it as a
//! `.0`, though the digits it reports are 1. Here it is a division in floats.

use std::time::{Duration, Instant};

/// The command that starts the readings, `mic_cmd_set_realtime_mode`.
pub(super) const REALTIME: &str = "S 1 M 2 32 3\r";
/// `serial_source_add`'s 100 ms: how long a silence must be before it counts.
pub(super) const SILENCE: Duration = Duration::from_millis(100);

/// A logger of the table.
#[derive(Debug)]
pub struct Model {
    /// The driver id.
    pub id: &'static str,
    /// The vendor.
    pub vendor: &'static str,
    /// The model.
    pub model: &'static str,
    /// Vendor and model.
    pub long_name: &'static str,
    /// The line, `mic_devs[].conn`.
    pub serialcomm: &'static str,
    /// Whether frames carry a humidity.
    pub has_humidity: bool,
}

/// `mic_devs`.
pub static MODELS: [Model; 2] = [
    Model {
        id: "mic-98581",
        vendor: "MIC",
        model: "98581",
        long_name: "MIC 98581",
        serialcomm: "38400/8n2",
        has_humidity: false,
    },
    Model {
        id: "mic-98583",
        vendor: "MIC",
        model: "98583",
        long_name: "MIC 98583",
        serialcomm: "38400/8n2",
        has_humidity: true,
    },
];

impl Model {
    /// How long a frame is.
    pub(super) fn packet_size(&self) -> usize {
        if self.has_humidity { 10 } else { 6 }
    }

    /// `packet_valid_temp` and `packet_valid_temp_hum`, on a window of
    /// [`Model::packet_size`] bytes.
    pub(super) fn valid(&self, buf: &[u8]) -> bool {
        let digits = |range: std::ops::Range<usize>| buf[range].iter().all(u8::is_ascii_digit);
        if self.has_humidity {
            buf[0] == b'v'
                && buf[1] == b' '
                && buf[5] == b' '
                && buf[9] == b'\r'
                && digits(2..5)
                && digits(6..9)
        } else {
            buf[0] == b'v' && buf[1] == b' ' && buf[5] == b'\r' && digits(2..5)
        }
    }

    /// `packet_parse`, on a frame [`Model::valid`] passed: the temperature, and
    /// the humidity if this logger has one.
    pub(super) fn parse(&self, frame: &[u8]) -> (f32, Option<f32>) {
        (field(&frame[2..5]), self.has_humidity.then(|| field(&frame[6..9])))
    }
}

/// Three digits in tenths.
fn field(digits: &[u8]) -> f32 {
    let tenths = digits.iter().fold(0u16, |n, d| n * 10 + u16::from(d - b'0'));
    f32::from(tenths) / 10.0
}

/// When a capture sends its one command: at the first silence, of
/// [`SILENCE`] since the last bytes or since the start. Bytes that keep coming
/// put it off, as they restart C's timer (`fd_source_dispatch`), so a logger
/// already sending fast is not told again.
///
/// There is one command per capture, whatever comes of it: C's flag says the
/// same (protocol.c:207-210), though it is one flag for the process.
#[derive(Debug)]
pub(super) struct Realtime {
    due: Instant,
    asked: bool,
}

impl Realtime {
    pub(super) fn new(now: Instant) -> Realtime {
        Realtime { due: now + SILENCE, asked: false }
    }

    /// How long the next read may wait, at least 1 ms: 0 would be a poll that
    /// spins.
    pub(super) fn wait(&self, now: Instant) -> Duration {
        self.due.saturating_duration_since(now).max(Duration::from_millis(1))
    }

    /// Bytes came.
    pub(super) fn heard(&mut self, now: Instant) {
        self.due = now + SILENCE;
    }

    /// A read came back empty. Whether to send the command now. An empty read
    /// before its time (a signal) is no silence.
    pub(super) fn empty(&mut self, now: Instant) -> bool {
        if now < self.due {
            return false;
        }
        self.due = now + SILENCE;
        !std::mem::replace(&mut self.asked, true)
    }
}

/// What went wrong with a capture that a sample limit alone could end.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Starved {
    /// Not one byte since the start or the last reading.
    NoBytes,
    /// Bytes, but none made a reading.
    NoReading,
}

/// How long a capture has gone with no reading. It is readings that count, not
/// bytes: bytes that make no frame would hold a sample limit open for ever.
#[derive(Debug)]
pub(super) struct Watch {
    bound: Duration,
    since: Instant,
    bytes: bool,
}

impl Watch {
    pub(super) fn new(bound: Duration, now: Instant) -> Watch {
        Watch { bound, since: now, bytes: false }
    }

    /// Bytes came.
    pub(super) fn heard(&mut self) {
        self.bytes = true;
    }

    /// A reading came and went out: `now` must be after the send, as a slow
    /// client's time is not the logger's.
    pub(super) fn reading(&mut self, now: Instant) {
        self.since = now;
        self.bytes = false;
    }

    /// Whether the bound has passed.
    pub(super) fn starved(&self, now: Instant) -> Option<Starved> {
        if now.saturating_duration_since(self.since) < self.bound {
            None
        } else if self.bytes {
            Some(Starved::NoReading)
        } else {
            Some(Starved::NoBytes)
        }
    }
}
