// Counterpart in sigrok:
//   libsigrok/src/hardware/kingst-la2016/protocol.c (send_chunk(), stream_data(): the sample layouts),
//   Copyright (C) 2022 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   Copyright (C) 2020 Florian Schmidt <schmidt_florian@gmx.de>,
//   Copyright (C) 2013 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>,
//   with commits by Kevin Grant, Helge Kruse, Dave Platt, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The sample side of the Kingst model: the signal on the probes, the
//! FPGA's hardware trigger, the run length encoded sample memory of a
//! capture and the uncompressed stream of streaming mode, in the
//! layouts `protocol.c` decodes (`send_chunk()`, `stream_data()`).
//!
//! Pure functions of the configuration the host wrote, so that both
//! tools and every run see the same samples.

/// The longest run one packet holds: its repeat count is a byte.
pub const MAX_REP: usize = 255;

/// The most samples the model keeps of one capture: a capture of more
/// (the driver passes 10 G for "no limit") keeps this many, an
/// emulation artifact; the device keeps what its SDRAM holds.
pub const MAX_SAMPLES: u64 = 1 << 22;

/// How far past the pre-trigger samples the model looks for the
/// trigger; a trigger that does not match by then never does, and the
/// capture waits for it for ever, as the device would.
pub const TRIGGER_SEARCH: u64 = 1 << 22;

/// The sample at time `t` (in samples since the capture started): in
/// each period of 1000 samples the low byte counts up every four
/// samples for 400 samples (0 to 99, so D0 toggles every four), then
/// holds 0x5a for 600 (runs longer than one packet holds); the high
/// byte counts the periods. A 32 channel device sees the complement
/// of those 16 bits on channels 16 to 31.
pub fn signal(t: u64, wide: bool) -> u32 {
    let (period, r) = (t / 1000, t % 1000);
    let low = if r < 400 { (r / 4) as u32 } else { 0x5a };
    let v = low | ((period as u32 & 0xff) << 8);
    if wide { v | ((!v & 0xffff) << 16) } else { v }
}

/// The trigger configuration of `REG_TRIGGER` (`set_trigger_config()`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Trigger {
    /// The enabled channels (`cfg.channels`).
    pub channels: u32,
    /// The channels the trigger looks at (`cfg.enabled`).
    pub enabled: u32,
    /// Those of them matched on a level; the one other is an edge.
    pub level: u32,
    /// High levels and falling edges.
    pub high_or_falling: u32,
}

impl Trigger {
    /// The bytes as the driver writes them: four `u32` little endian.
    pub fn parse(b: &[u8]) -> Option<Trigger> {
        if b.len() != 16 {
            return None;
        }
        let w = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Some(Trigger { channels: w(0), enabled: w(4), level: w(8), high_or_falling: w(12) })
    }

    /// Whether the trigger looks at anything.
    pub fn involved(&self) -> bool {
        self.enabled != 0
    }

    /// The edge channel's mask, 0 for none.
    pub fn edge(&self) -> u32 {
        self.enabled & !self.level
    }

    /// Whether the condition holds at sample `t`.
    pub fn matches(&self, t: u64, wide: bool) -> bool {
        let v = signal(t, wide);
        if v & self.level != self.high_or_falling & self.level {
            return false;
        }
        let edge = self.edge();
        if edge == 0 {
            return true;
        }
        if t == 0 {
            return false;
        }
        let (before, now) = (signal(t - 1, wide) & edge, v & edge);
        if self.high_or_falling & edge == 0 { before == 0 && now != 0 } else { before != 0 && now == 0 }
    }
}

/// How a device lays out its sample memory: for 16 channels five
/// packets (`u16` pins, `u8` repeat count) and one sequence byte per
/// 16 byte unit; for 32 channels six packets (`u32` pins, `u8` count)
/// and two sequence bytes (inverted, then plain) per 32 byte unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    /// 32 channels.
    pub wide: bool,
}

impl Layout {
    /// Bytes of a sample value.
    pub fn unit(self) -> usize {
        if self.wide { 4 } else { 2 }
    }

    /// Bytes of a transfer unit (`devc->transfer_size`).
    pub fn xfer(self) -> usize {
        if self.wide { 32 } else { 16 }
    }

    /// Packets per transfer unit (`devc->packets_per_chunk`).
    pub fn packets(self) -> usize {
        if self.wide { 6 } else { 5 }
    }
}

/// A capture in the sample memory.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Capture {
    /// The transfer units, as stored from the capture's start address.
    pub memory: Vec<u8>,
    /// Packets stored, the padding included (`n_rep_packets`).
    pub packets: u32,
    /// Packets before the trigger point (`n_rep_packets_before_trigger`).
    pub before_trigger: u32,
    /// Samples stored.
    pub samples: u64,
}

/// The first sample at which `trigger` fires, looking from `from` on;
/// `None` if not within [`TRIGGER_SEARCH`] samples.
pub fn find_trigger(trigger: &Trigger, from: u64, wide: bool) -> Option<u64> {
    (from..from + TRIGGER_SEARCH).find(|&t| trigger.matches(t, wide))
}

/// The capture the FPGA stores for `limit` samples of which `pre` come
/// before the trigger point: the window of the signal from `start` on,
/// run length encoded, a run split where it reaches [`MAX_REP`] and
/// where the trigger point falls, and the last unit filled up with
/// packets of the last value and a count of 0. Whether the real FPGA
/// pads that way is not known (the driver warns when the packet count
/// is no multiple of a unit and reads only whole units).
pub fn capture(start: u64, limit: u64, pre: Option<u64>, layout: Layout) -> Capture {
    let limit = limit.min(MAX_SAMPLES);
    let mut packets: Vec<(u32, u8)> = Vec::new();
    let mut before_trigger = 0;
    let mut run: Option<(u32, usize)> = None;
    for i in 0..limit {
        let v = signal(start + i, layout.wide);
        if pre == Some(i) {
            if let Some((value, n)) = run.take() {
                packets.push((value, n as u8));
            }
            before_trigger = packets.len() as u32;
        }
        run = match run {
            Some((value, n)) if value == v && n < MAX_REP => Some((value, n + 1)),
            Some((value, n)) => {
                packets.push((value, n as u8));
                Some((v, 1))
            }
            None => Some((v, 1)),
        };
    }
    if let Some((value, n)) = run {
        packets.push((value, n as u8));
    }
    if pre.is_some_and(|p| p >= limit) {
        before_trigger = packets.len() as u32;
    }
    let last = packets.last().map_or(0, |p| p.0);
    while !packets.len().is_multiple_of(layout.packets()) {
        packets.push((last, 0));
    }
    let mut memory = Vec::with_capacity(packets.len() / layout.packets() * layout.xfer());
    for (seq, unit) in packets.chunks(layout.packets()).enumerate() {
        for (value, rep) in unit {
            memory.extend_from_slice(&value.to_le_bytes()[..layout.unit()]);
            memory.push(*rep);
        }
        let seq = seq as u8;
        if layout.wide {
            memory.push(!seq);
        }
        memory.push(seq);
    }
    Capture { memory, packets: packets.len() as u32, before_trigger, samples: limit }
}

/// The stream of streaming mode: for each enabled channel in turn one
/// `u16` of 16 consecutive samples, the earliest in bit 0; then the
/// next 16 samples. Starts at sample 0 when the capture starts.
#[derive(Clone, Debug)]
pub struct Stream {
    channels: Vec<u32>,
    wide: bool,
    t: u64,
    next: usize,
}

impl Stream {
    /// A stream of the channels in `mask`.
    pub fn new(mask: u32, wide: bool) -> Stream {
        Stream { channels: (0..32).filter(|c| mask & (1 << c) != 0).collect(), wide, t: 0, next: 0 }
    }

    /// The next `len` bytes (an even number).
    pub fn take(&mut self, len: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(len);
        while out.len() + 2 <= len && !self.channels.is_empty() {
            let c = self.channels[self.next];
            let mut word = 0u16;
            for bit in 0..16 {
                if signal(self.t + bit, self.wide) & (1 << c) != 0 {
                    word |= 1 << bit;
                }
            }
            out.extend_from_slice(&word.to_le_bytes());
            self.next += 1;
            if self.next == self.channels.len() {
                self.next = 0;
                self.t += 16;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decodes memory as `send_chunk()` does: the samples, and the
    /// index of the first after the trigger mark.
    fn decode(c: &Capture, layout: Layout) -> (Vec<u32>, u64) {
        let mut out = Vec::new();
        let mut until = c.before_trigger;
        let mut mark = if until == 0 { Some(0) } else { None };
        for unit in c.memory.chunks(layout.xfer()) {
            for p in unit[..layout.packets() * (layout.unit() + 1)].chunks(layout.unit() + 1) {
                let mut v = [0u8; 4];
                v[..layout.unit()].copy_from_slice(&p[..layout.unit()]);
                for _ in 0..p[layout.unit()] {
                    out.push(u32::from_le_bytes(v));
                }
                if mark.is_none() {
                    until -= 1;
                    if until == 0 {
                        mark = Some(out.len() as u64);
                    }
                }
            }
        }
        (out, mark.unwrap_or(u64::MAX))
    }

    #[test]
    fn the_signal_has_short_and_long_runs() {
        assert_eq!((signal(0, false), signal(3, false), signal(4, false)), (0, 0, 1));
        assert_eq!(signal(400, false), 0x5a);
        assert_eq!(signal(1000, false), 0x100);
        assert_eq!(signal(1, true), 0xffff_0000);
    }

    #[test]
    fn a_capture_decodes_to_the_signal() {
        for layout in [Layout { wide: false }, Layout { wide: true }] {
            let c = capture(0, 2500, None, layout);
            assert_eq!(c.packets as usize % layout.packets(), 0);
            assert_eq!(c.memory.len(), c.packets as usize / layout.packets() * layout.xfer());
            let (samples, _) = decode(&c, layout);
            let want: Vec<u32> = (0..2500).map(|t| signal(t, layout.wide)).collect();
            assert_eq!(samples, want);
            // 600 samples of 0x5a: 255, 255, 90.
            let run: &[u8] = if layout.wide { &[0x5a, 0, 0xa5, 0xff, 255] } else { &[0x5a, 0, 255] };
            assert!(c.memory.windows(run.len()).any(|p| p == run));
        }
        assert_eq!(capture(0, 1, None, Layout { wide: false }).memory.len(), 16);
    }

    #[test]
    fn a_triggered_capture_splits_the_run_at_the_trigger() {
        let layout = Layout { wide: false };
        // Rising D0: at 4, 12, ...; with 10 samples before it, at 12.
        let t = Trigger { channels: 0xffff, enabled: 1, level: 0, high_or_falling: 0 };
        let at = find_trigger(&t, 10, false).unwrap();
        assert_eq!(at, 12);
        let c = capture(at - 10, 30, Some(10), layout);
        let (samples, mark) = decode(&c, layout);
        assert_eq!(mark, 10);
        assert_eq!(samples[10], signal(12, false));
        assert_eq!(samples.len(), 30);
        // Levels and a falling edge.
        let t = Trigger { channels: 0xffff, enabled: 0x103, level: 0x102, high_or_falling: 0x003 };
        let at = find_trigger(&t, 0, false).unwrap();
        assert_eq!(signal(at, false) & 0x103, 0x002);
        assert_eq!(signal(at - 1, false) & 1, 1);
    }

    #[test]
    fn the_stream_rotates_the_channels() {
        let mut s = Stream::new(0b101, false);
        let b = s.take(8);
        // D0 over samples 0..16: 0000 1111 0000 1111 (LSB first).
        assert_eq!(u16::from_le_bytes([b[0], b[1]]), 0xf0f0);
        // D2 over samples 0..16: toggles every 16.
        assert_eq!(u16::from_le_bytes([b[2], b[3]]), 0x0000);
        assert_eq!(u16::from_le_bytes([b[6], b[7]]), 0xffff, "D2 over 16..32");
    }
}
