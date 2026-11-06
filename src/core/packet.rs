// Counterpart in sigrok:
//   libsigrok/src/session.c (the datafeed packets).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The data feed: what a device sends while it acquires.

use crate::core::analog::{Mq, MqFlags, Unit};
use crate::core::key::Key;
use crate::core::value::Value;
use std::time::SystemTime;

/// A block of logic samples.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogicData {
    /// Bytes per sample. Bit `i` of a sample is logic channel index `i`,
    /// with byte 0 holding channels 0 to 7.
    pub unit_size: usize,
    /// `unit_size` bytes per sample, samples back to back.
    pub data: Vec<u8>,
}

impl LogicData {
    /// The number of samples in this block.
    pub fn len(&self) -> usize {
        self.data.len().checked_div(self.unit_size).unwrap_or(0)
    }

    /// True when the block holds no samples.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The level of logic channel `index` in sample `sample`.
    pub fn bit(&self, sample: usize, index: usize) -> bool {
        let byte = self.data[sample * self.unit_size + index / 8];
        byte & (1 << (index % 8)) != 0
    }
}

/// A block of analog samples of one or more channels.
///
/// With several channels the samples are interleaved: sample 0 of every
/// channel, then sample 1, and so on. Values are in the unit, already
/// scaled.
#[derive(Clone, Debug, PartialEq)]
pub struct AnalogData {
    /// Channel indices, as the device numbers them.
    pub channels: Vec<usize>,
    /// Samples, `channels.len()` per sample.
    pub samples: Vec<f32>,
    /// What was measured.
    pub mq: Mq,
    /// How it was measured, e.g. DC.
    pub flags: MqFlags,
    /// The unit of the samples.
    pub unit: Unit,
    /// Significant decimal digits after the point; negative means digits
    /// before the point are not significant either.
    pub digits: i32,
}

impl AnalogData {
    /// The number of samples per channel.
    pub fn len(&self) -> usize {
        if self.channels.is_empty() { 0 } else { self.samples.len() / self.channels.len() }
    }

    /// True when the block holds no samples.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// One item of the data feed.
///
/// The variants mirror the sigrok datafeed packets, so a session file
/// writer or a front end written against libsigrok maps them one to one.
#[derive(Clone, Debug, PartialEq)]
pub enum Packet {
    /// The first packet of an acquisition.
    Header {
        /// When the acquisition started.
        start: SystemTime,
    },
    /// Settings that changed, e.g. the sample rate a device chose.
    Meta(Vec<(Key, Value)>),
    /// The trigger fired. The next logic sample is the trigger point.
    Trigger,
    /// Logic samples.
    Logic(LogicData),
    /// Analog samples.
    Analog(AnalogData),
    /// The start of a frame (one oscilloscope sweep, one buffer).
    FrameBegin,
    /// The end of a frame.
    FrameEnd,
    /// The acquisition failed, and why, in the driver's words: it ended for
    /// a reason other than a limit, a stop or the natural end of the data.
    /// What came before it is good, and [`Packet::End`] still follows. A
    /// driver sends it through [`PacketSink::end`](crate::core::PacketSink::end).
    Failed(String),
    /// The last packet of an acquisition.
    End,
}

impl Packet {
    /// A logic packet over `data` with `unit_size` bytes per sample.
    pub fn logic(unit_size: usize, data: Vec<u8>) -> Self {
        Packet::Logic(LogicData { unit_size, data })
    }

    /// A header packet starting now.
    pub fn header() -> Self {
        Packet::Header { start: SystemTime::now() }
    }

    /// A meta packet with one setting.
    pub fn meta(key: Key, value: impl Into<Value>) -> Self {
        Packet::Meta(vec![(key, value.into())])
    }
}
