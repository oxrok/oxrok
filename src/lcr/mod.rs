// Counterpart in sigrok:
//   libsigrok/src/hardware/serial-lcr/protocol.h,
//   Copyright (C) 2014 Janne Huttunen <jahuttun@gmail.com>,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The packets of LCR meters on a serial cable, as pure parsers: the
//! counterpart of libsigrok's `src/lcr`, as [`crate::dmm`] is of `src/dmm`.
//! The driver around them is `drivers::serial_lcr`.
//!
//! Two chips, and nothing in a packet says which: each meter is its own
//! driver id. A [`Chip`] tells a packet's size and frame, what a meter is
//! asked with, what a packet says of the meter's settings, and what it reads
//! on a channel.

#![forbid(unsafe_code)]

pub mod es51919;
pub mod vc4080;

use crate::core::{Mq, MqFlags, Unit};
use std::time::Duration;

/// A packet's biggest size, for buffers.
pub const MAX_PACKET_SIZE: usize = vc4080::PACKET_SIZE;

/// What one channel of a packet reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// The value, scaled to the unit.
    pub value: f32,
    /// What was measured.
    pub mq: Mq,
    /// How.
    pub flags: MqFlags,
    /// The unit.
    pub unit: Unit,
    /// Significant decimals.
    pub digits: i32,
}

/// What a packet says of the meter's settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// The output frequency in Hz; 0 when it is DC, or not known.
    pub output_freq: u64,
    /// The equivalent circuit model.
    pub circuit_model: &'static str,
}

/// `10^exponent` as C's `powf(10, exponent)` gets it: `powi` differs in the
/// last digit of some results.
pub(crate) fn pow10(exponent: i32) -> f32 {
    10f32.powf(exponent as f32)
}

/// A packet format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    /// Cyrustek ES51919: streams 17 byte binary packets.
    Es51919,
    /// Voltcraft 4080: answers `N` with 39 characters.
    Vc4080,
}

impl Chip {
    /// A packet's size.
    pub fn packet_size(self) -> usize {
        match self {
            Chip::Es51919 => es51919::PACKET_SIZE,
            Chip::Vc4080 => vc4080::PACKET_SIZE,
        }
    }

    /// The cable's default `serialcomm`.
    pub fn serialcomm(self) -> &'static str {
        match self {
            Chip::Es51919 => "9600/8n1/rts=1/dtr=1",
            Chip::Vc4080 => "1200/8n1",
        }
    }

    /// The channels, both chips having a primary and a secondary display.
    pub fn channel_count(self) -> usize {
        2
    }

    /// What a meter is asked for a packet with, if it must be.
    pub fn request(self) -> Option<&'static [u8]> {
        match self {
            Chip::Es51919 => None,
            Chip::Vc4080 => Some(vc4080::REQUEST),
        }
    }

    /// How long after a request the next may go (`req_timeout_ms`).
    pub fn request_interval(self) -> Duration {
        match self {
            Chip::Es51919 => Duration::ZERO,
            Chip::Vc4080 => vc4080::REQUEST_INTERVAL,
        }
    }

    /// Whether `pkt`, of [`Chip::packet_size`] bytes, has the frame of a
    /// packet: all the check there is.
    pub fn valid(self, pkt: &[u8]) -> bool {
        match self {
            Chip::Es51919 => es51919::valid(pkt),
            Chip::Vc4080 => vc4080::valid(pkt),
        }
    }

    /// The meter's settings from a valid packet.
    pub fn settings(self, pkt: &[u8]) -> Settings {
        match self {
            Chip::Es51919 => es51919::settings(pkt),
            Chip::Vc4080 => vc4080::settings(pkt),
        }
    }

    /// What a valid packet reads on `channel`, if anything.
    pub fn reading(self, pkt: &[u8], channel: usize) -> Option<Reading> {
        match self {
            Chip::Es51919 => es51919::reading(pkt, channel == 1),
            Chip::Vc4080 => vc4080::reading(pkt, channel),
        }
    }

    /// `config_list` of the output frequency.
    pub fn frequencies(self) -> &'static [f64] {
        match self {
            Chip::Es51919 => &es51919::FREQUENCIES,
            Chip::Vc4080 => &vc4080::FREQUENCIES,
        }
    }

    /// `config_list` of the equivalent circuit model.
    pub fn circuit_models(self) -> &'static [&'static str] {
        match self {
            Chip::Es51919 => &es51919::CIRCUIT_MODELS,
            Chip::Vc4080 => &vc4080::CIRCUIT_MODELS,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn powers_of_ten_as_powf() {
        assert_eq!(pow10(0), 1.0);
        assert_eq!(pow10(3), 1000.0);
        assert!((pow10(-3) - 0.001).abs() < 1e-9);
        assert!((pow10(-12) - 1e-12).abs() < 1e-18);
    }

    #[test]
    fn the_table_of_each_chip() {
        assert_eq!(Chip::Es51919.packet_size(), 17);
        assert_eq!(Chip::Vc4080.packet_size(), 39);
        assert_eq!(MAX_PACKET_SIZE, 39);
        assert_eq!(Chip::Es51919.serialcomm(), "9600/8n1/rts=1/dtr=1");
        assert_eq!(Chip::Vc4080.serialcomm(), "1200/8n1");
        assert_eq!(Chip::Es51919.request(), None);
        assert_eq!(Chip::Vc4080.request(), Some(&b"N"[..]));
        assert_eq!(Chip::Vc4080.request_interval(), Duration::from_millis(500));
        assert_eq!(Chip::Es51919.channel_count(), 2);
        assert_eq!(Chip::Vc4080.channel_count(), 2);
    }
}
