// Counterpart in sigrok:
//   libsigrok/src/device.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Channels and channel groups of a device.

/// Whether a channel carries logic (one bit per sample) or analog
/// (one float per sample) data.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ChannelKind {
    /// A digital input.
    Logic,
    /// An analog input.
    Analog,
}

/// One input of a device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Channel {
    /// The channel's index as the device numbers it. For logic channels
    /// this is the bit position inside a logic sample unit.
    pub index: usize,
    /// Logic or analog.
    pub kind: ChannelKind,
    /// The name, e.g. `D0`, `CLK1` or `A0`. Front ends may rename it.
    pub name: String,
    /// Whether the channel takes part in the next acquisition.
    pub enabled: bool,
}

impl Channel {
    /// A new, enabled channel.
    pub fn new(index: usize, kind: ChannelKind, name: impl Into<String>) -> Self {
        Channel { index, kind, name: name.into(), enabled: true }
    }

    /// A logic channel named `D<index>`.
    pub fn logic(index: usize) -> Self {
        Channel::new(index, ChannelKind::Logic, format!("D{index}"))
    }
}

/// A named set of channels that share settings, e.g. all logic channels,
/// or one oscilloscope input with its own vertical scale.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelGroup {
    /// The group's name, as used by `-g` on the command line.
    pub name: String,
    /// Positions into the device's channel list (not channel indices).
    pub channels: Vec<usize>,
}

impl ChannelGroup {
    /// A new group over the given channel list positions.
    pub fn new(name: impl Into<String>, channels: Vec<usize>) -> Self {
        ChannelGroup { name: name.into(), channels }
    }
}

/// Bytes per logic sample for `count` logic channels.
pub fn unit_size(count: usize) -> usize {
    count.div_ceil(8)
}
