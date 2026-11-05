// Counterpart in sigrok:
//   libsigrok/src/hardware/openbench-logic-sniffer/protocol.c,
//   libsigrok/src/hardware/openbench-logic-sniffer/protocol.h,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by v1ne, Uwe Hermann, Gerhard Sittig, Wolfram Sang,
//   Martin Ling, Daniel Elstner, Lars-Peter Clausen, Ben Gardiner,
//   Gwenhael Goavec-Merou, Marek Vasut, Pavel Fedin, Soeren Apel,
//   Uffe Jakobsen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The protocol of `ols`, `protocol.c`: SUMP commands over a serial port,
//! the capture they set up, and the sample rate divider.
//!
//! A short command is one byte, a long one the byte and a 32 bit argument,
//! little endian. A device answers `CMD_ID` with `1SLO` or `1ALS`, and may
//! answer `CMD_METADATA` with the block of [`sump::read_metadata`]. A
//! capture is set up by the trigger stages (mask, value, config of each it
//! uses), the divider of the 100 MHz clock, the read and delay counts in
//! units of 4 words minus 1 (`CMD_CAPTURE_SIZE`, 16 bits each, or
//! `CMD_CAPTURE_READCOUNT` and `CMD_CAPTURE_DELAYCOUNT` on a board with more
//! than 256 KiB), the flags, and the run command. The words come back
//! newest first, see [`sump::Decoder`].
//!
//! Commands, metadata, channel groups, trigger stages and sample decoding are
//! `p-ols`' ([`crate::drivers::pipistrello_ols::protocol`], the same SUMP).
//! This driver's own are what `protocol.c` does differently: no edge
//! registers, no disabled stages, the start bit on the last stage used, the
//! 16 bit counts, a demux word taken for one sample, a sample rate that may
//! pass a maximum the clock does not divide to.
//!
//! Where this differs from the C driver:
//! - a read count comes from the limit where the board reports no memory, up
//!   to the 0xffff that C always sends there (`protocol.c:597-599` takes the
//!   0, and 656 sends it minus 1); a limit beyond that gets a warning;
//! - the delay count is at least one unit (`protocol.c:617` gives 0 for
//!   `captureratio=100`, and 650-657 send it minus 1: all of the capture
//!   after the trigger, against the marker);
//! - the marker follows the board's words, not samples taken for words
//!   (`protocol.c:618`), see [`Plan::trigger_word`].

use crate::drivers::pipistrello_ols::protocol as sump;
pub use sump::{
    CLOCK_RATE, CMD_ID, CMD_METADATA, CMD_RESET, CMD_SET_DIVIDER, CMD_SET_FLAGS,
    CMD_SET_TRIGGER_CONFIG, CMD_SET_TRIGGER_MASK, CMD_SET_TRIGGER_VALUE, Decoder,
    FLAG_CLOCK_EXTERNAL, FLAG_DEMUX, FLAG_EXTERNAL_TEST_MODE, FLAG_FILTER, FLAG_INTERNAL_TEST_MODE,
    FLAG_RLE, FLAG_SWAP_CHANNELS, MIN_NUM_SAMPLES, Metadata, Stages, TRIGGER_START,
    convert_trigger, group_mask, long_command, read_metadata, short_command,
};

/// `DEFAULT_SAMPLERATE`.
pub const DEFAULT_SAMPLERATE: u64 = 200_000;
/// What `config_set()` takes: 10 Hz.
pub const SAMPLERATE_MIN: u64 = 10;
/// What `config_set()` takes, and lists without metadata: 200 MHz.
pub const SAMPLERATE_MAX: u64 = 200_000_000;
/// `CMD_ARM_BASIC_TRIGGER`.
pub const CMD_RUN: u8 = sump::CMD_RUN;
/// `CMD_CAPTURE_SIZE`.
pub const CMD_CAPTURE_SIZE: u8 = 0x81;
/// `CMD_CAPTURE_DELAYCOUNT`.
pub const CMD_CAPTURE_DELAYCOUNT: u8 = sump::CMD_CAPTURE_DELAYCOUNT;
/// `CMD_CAPTURE_READCOUNT`.
pub const CMD_CAPTURE_READCOUNT: u8 = sump::CMD_CAPTURE_READCOUNT;
/// `CAPTURE_FLAG_INVERT_EXT_CLOCK`.
pub const FLAG_INVERT_EXT_CLOCK: u16 = 1 << 7;
/// A board with more memory than this takes the long counts.
const SMALL_MEMORY: u32 = 256 * 1024;
/// The most `CMD_CAPTURE_SIZE`'s 16 bits say, in units of 4 words. A board
/// with no memory key gets this form too, so a limit beyond it is cut here,
/// or the count minus 1 would wrap to a small one.
const MAX_SHORT_COUNT: u64 = 0x1_0000;

/// A sample rate as `ols_set_samplerate()` sets it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate {
    /// The rate the divider gives.
    pub samplerate: u64,
    /// The divider of the clock, minus 1.
    pub divider: u32,
    /// Above 100 MHz.
    pub demux: bool,
}

/// `ols_set_samplerate()`: `None` beyond the board's maximum, when it gave
/// one, or for 0. Unlike `p-ols`' this keeps C's rounding: at a maximum the
/// clock does not divide to, the rate given is above it, as nothing shows a
/// board minds.
pub fn samplerate(rate: u64, max_samplerate: u32) -> Option<Rate> {
    if rate == 0 || (max_samplerate != 0 && rate > u64::from(max_samplerate)) {
        return None;
    }
    let demux = rate > CLOCK_RATE;
    let clock = if demux { CLOCK_RATE * 2 } else { CLOCK_RATE };
    let divider = u32::try_from(clock / rate - 1).ok()?;
    let samplerate = clock / (u64::from(divider) + 1);
    Some(Rate { samplerate, divider, demux })
}

/// `ols_metadata_quirks()`: what a Shrimp doesn't say.
pub fn apply_quirks(m: &mut Metadata) {
    if m.name == "Shrimp1.0" {
        if m.channels == 0 {
            m.channels = 4;
        }
        if m.max_samplebytes == 0 {
            m.max_samplebytes = 256 * 1024;
        }
        if m.max_samplerate == 0 {
            m.max_samplerate = 20_000_000;
        }
    }
}

/// The settings a capture is planned from.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// `capture_flags`, as the options and the sample rate left them.
    pub flags: u16,
    /// The enabled channels.
    pub channel_mask: u32,
    /// The board's memory, 0 when not reported.
    pub max_samples: u32,
    /// The sample limit.
    pub limit_samples: u64,
    /// The capture ratio, in percent.
    pub capture_ratio: u64,
    /// The divider.
    pub divider: u32,
}

/// A capture as `ols_prepare_acquisition()` sets it up.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// The commands to send, in order, the run command last.
    pub commands: Vec<Vec<u8>>,
    /// `capture_flags` afterwards: the disabled groups in bits 2-5.
    pub flags: u16,
    /// The word, counted from the newest one sent, at the marker.
    /// `protocol.c:618` has it `(readcount - delaycount) * 4 - num_stages`
    /// samples from the oldest, which is this word without RLE and at a limit
    /// of whole units.
    pub trigger_word: Option<u64>,
}

/// The commands of `stage`, of `count` stages used: its level in the config,
/// and the start bit on the last.
fn stage_commands(s: &Stages, stage: usize, count: usize) -> [Vec<u8>; 3] {
    let off = 4 * u8::try_from(stage).unwrap_or(0);
    let level = u32::try_from(stage).unwrap_or(0) << 16;
    let start = if stage + 1 == count { u32::from(TRIGGER_START) << 24 } else { 0 };
    [
        long_command(CMD_SET_TRIGGER_MASK + off, s.mask[stage]),
        long_command(CMD_SET_TRIGGER_VALUE + off, s.value[stage]),
        long_command(CMD_SET_TRIGGER_CONFIG + off, level | start),
    ]
}

/// `ols_prepare_acquisition()` and the run command, without the C driver's
/// wraps (see the module documentation).
///
/// # Errors
///
/// The message to log when the capture cannot be set up.
pub fn plan(s: &Settings, stages: &Stages) -> Result<Plan, String> {
    let groups = group_mask(s.channel_mask);
    if groups == 0 {
        return Err("No channels enabled.".into());
    }
    if s.limit_samples == 0 {
        return Err("A sample limit is needed.".into());
    }
    let samplecount = match s.max_samples / groups.count_ones() {
        0 => s.limit_samples,
        most => u64::from(most).min(s.limit_samples),
    };
    let mut units = samplecount.div_ceil(4).max(1);
    if s.max_samples <= SMALL_MEMORY {
        units = units.min(MAX_SHORT_COUNT);
    }
    let readcount = u32::try_from(units).unwrap_or(u32::MAX);
    let returned = u64::from(readcount) * 4;
    // With RLE a word can stand for many samples, so nothing is known.
    if s.flags & FLAG_RLE == 0 && s.limit_samples > returned {
        log::warn!("The board returns {returned} samples of the {} asked for.", s.limit_samples);
    }
    let mut commands = Vec::new();
    let delaycount;
    let trigger_word;
    if stages.count > 0 {
        log::debug!("Send reset command before trigger configure");
        commands.extend(std::iter::repeat_n(short_command(CMD_RESET), 5));
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let d = (f64::from(readcount) * (1.0 - s.capture_ratio as f64 / 100.0)) as u32;
        delaycount = d.max(1);
        trigger_word = Some(u64::from(delaycount) * 4 + stages.count as u64 - 1);
        for i in 0..stages.count {
            log::debug!("Setting OLS stage {i} trigger.");
            commands.extend(stage_commands(stages, i, stages.count));
        }
    } else {
        log::debug!("Forcing trigger at stage 0.");
        commands.extend(stage_commands(stages, 0, 1));
        delaycount = readcount;
        trigger_word = None;
    }
    commands.push(long_command(CMD_SET_DIVIDER, s.divider & 0x00ff_ffff));
    let (read, delay) = (readcount - 1, delaycount - 1);
    log::debug!(
        "Setting sample limit {}, trigger point at {}",
        read.wrapping_mul(4),
        delay.wrapping_mul(4)
    );
    if s.max_samples > SMALL_MEMORY {
        commands.push(long_command(CMD_CAPTURE_READCOUNT, read));
        commands.push(long_command(CMD_CAPTURE_DELAYCOUNT, delay));
    } else {
        commands.push(long_command(CMD_CAPTURE_SIZE, (read & 0xffff) | (delay & 0xffff) << 16));
    }
    let mut flags = s.flags & !0x3c;
    flags |= !(u16::from(groups) << 2) & 0x3c;
    commands.push(long_command(CMD_SET_FLAGS, u32::from(flags)));
    commands.push(short_command(CMD_RUN));
    Ok(Plan { commands, flags, trigger_word })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Trigger, TriggerMatch};

    fn settings() -> Settings {
        Settings {
            flags: FLAG_FILTER,
            channel_mask: u32::MAX,
            max_samples: 24576,
            limit_samples: 16,
            capture_ratio: 0,
            divider: 499,
        }
    }

    fn hexes(p: &Plan) -> Vec<String> {
        use std::fmt::Write as _;
        p.commands
            .iter()
            .map(|c| {
                c.iter().fold(String::new(), |mut s, b| {
                    let _ = write!(s, "{b:02x}");
                    s
                })
            })
            .collect()
    }

    fn stages(spec: &[(usize, usize, TriggerMatch)], mask: u32) -> Stages {
        let mut t = Trigger::new();
        for &(stage, ch, m) in spec {
            t.add(stage, ch, m);
        }
        convert_trigger(Some(&t), mask).unwrap()
    }

    #[test]
    fn a_capture_without_a_trigger_as_the_model_saw_it() {
        let p = plan(&settings(), &Stages::default()).unwrap();
        // The model's "samples 16" golden.
        assert_eq!(
            hexes(&p).concat(),
            "c000000000c100000000c20000000880f30100008103000300820200000001"
        );
        assert_eq!(p.trigger_word, None);
        assert_eq!(p.flags, 2);
    }

    #[test]
    fn two_stages_have_their_levels_and_the_start_on_the_last() {
        let s = Settings { capture_ratio: 50, ..settings() };
        let st = stages(&[(0, 4, TriggerMatch::One), (1, 4, TriggerMatch::Zero)], u32::MAX);
        let p = plan(&s, &st).unwrap();
        // The model's "two stages" golden.
        assert_eq!(
            hexes(&p)[5..].concat(),
            "c010000000c110000000c200000000c410000000c500000000c60000010880f30100008103000100820200000001"
        );
        assert_eq!(hexes(&p)[..5].concat(), "0000000000");
        assert_eq!(p.trigger_word, Some(9));
    }

    #[test]
    fn the_counts_take_two_forms() {
        let s = Settings { max_samples: 262_144, ..settings() };
        let p = plan(&s, &Stages::default()).unwrap();
        assert!(hexes(&p).contains(&"8103000300".to_owned()));
        let s = Settings { max_samples: 262_148, ..settings() };
        let p = plan(&s, &Stages::default()).unwrap();
        let h = hexes(&p);
        let (read, delay) = (
            h.iter().position(|c| c == "8403000000").unwrap(),
            h.iter().position(|c| c == "8303000000").unwrap(),
        );
        assert_eq!(delay, read + 1, "the read count first");
    }

    #[test]
    fn the_memory_bounds_the_read_count_by_groups() {
        let s = Settings { max_samples: 64, limit_samples: 1000, ..settings() };
        assert!(hexes(&plan(&s, &Stages::default()).unwrap()).contains(&"8103000300".to_owned()));
        let s = Settings { channel_mask: 0xff, ..s };
        assert!(hexes(&plan(&s, &Stages::default()).unwrap()).contains(&"810f000f00".to_owned()));
    }

    #[test]
    fn no_memory_reported_counts_from_the_limit() {
        let s = Settings { max_samples: 0, ..settings() };
        assert!(hexes(&plan(&s, &Stages::default()).unwrap()).contains(&"8103000300".to_owned()));
    }

    #[test]
    fn no_memory_reported_stops_at_what_the_short_counts_hold() {
        let counts = |limit: u64, ratio: u64, st: &Stages| {
            let s = Settings {
                max_samples: 0,
                limit_samples: limit,
                capture_ratio: ratio,
                ..settings()
            };
            let p = plan(&s, st).unwrap();
            hexes(&p).into_iter().find(|c| c.starts_with("81")).unwrap()
        };
        let none = Stages::default();
        assert_eq!(counts(262_140, 0, &none), "81feff feff".replace(' ', ""));
        // Each of these wrapped: 262148 samples to a read count of 0.
        for limit in [262_144, 262_145, 262_148, 300_000, 1 << 20, u64::MAX] {
            assert_eq!(counts(limit, 0, &none), "81ffffffff", "{limit}");
        }
        // The delay comes from the count that goes out, so it is never above.
        let st = stages(&[(0, 4, TriggerMatch::One)], u32::MAX);
        assert_eq!(counts(300_000, 50, &st), "81ffffff7f");
        // A reported memory at the edge of the short form holds as many.
        let s = Settings { max_samples: 262_144, limit_samples: 262_144, ..settings() };
        let s = Settings { channel_mask: 0xff, ..s };
        let p = plan(&s, &none).unwrap();
        assert!(hexes(&p).contains(&"81ffffffff".to_owned()));
        // The long form has no such bound.
        let s = Settings { max_samples: 1 << 24, limit_samples: 1 << 24, ..settings() };
        let s = Settings { channel_mask: 0xff, ..s };
        let p = plan(&s, &none).unwrap();
        assert!(hexes(&p).contains(&"84ffff3f00".to_owned()));
    }

    #[test]
    fn ratio_corners() {
        let st = stages(&[(0, 4, TriggerMatch::One)], u32::MAX);
        let s = Settings { capture_ratio: 100, ..settings() };
        let p = plan(&s, &st).unwrap();
        assert!(hexes(&p).contains(&"8103000000".to_owned()), "one unit, not 0xffff");
        assert_eq!(p.trigger_word, Some(4));
        let s = Settings { capture_ratio: 0, ..settings() };
        let p = plan(&s, &st).unwrap();
        assert!(hexes(&p).contains(&"8103000300".to_owned()));
        assert_eq!(p.trigger_word, Some(16), "no such word of 16: no marker");
        // A limit of 5 reads 2 units of 4.
        let s = Settings { capture_ratio: 50, limit_samples: 5, ..settings() };
        let p = plan(&s, &st).unwrap();
        assert!(hexes(&p).contains(&"8101000000".to_owned()));
        assert_eq!(p.trigger_word, Some(4));
    }

    #[test]
    fn what_cannot_be_set_up_is_refused() {
        let s = Settings { channel_mask: 0, ..settings() };
        assert!(plan(&s, &Stages::default()).is_err());
        let s = Settings { limit_samples: 0, ..settings() };
        assert!(plan(&s, &Stages::default()).is_err());
        // A limit beyond u32 stays in range.
        let s = Settings { limit_samples: u64::MAX, max_samples: 0, ..settings() };
        assert!(plan(&s, &Stages::default()).is_ok());
    }

    #[test]
    fn groups_go_in_the_flags_keeping_the_rest() {
        let s = Settings { channel_mask: 0x00ff_00ff, flags: FLAG_RLE | 0x3c, ..settings() };
        let p = plan(&s, &Stages::default()).unwrap();
        assert_eq!(p.flags, FLAG_RLE | 0x28);
    }

    #[test]
    fn samplerates_as_the_c_divides() {
        assert_eq!(samplerate(200_000, 0).unwrap().divider, 499);
        let r = samplerate(3_000_000, 0).unwrap();
        assert_eq!((r.samplerate, r.divider, r.demux), (3_030_303, 32, false));
        let r = samplerate(150_000_000, 0).unwrap();
        assert_eq!((r.samplerate, r.divider, r.demux), (200_000_000, 0, true));
        assert_eq!(samplerate(10, 0).unwrap().divider, 9_999_999);
        assert!(samplerate(200_000_000, 100_000_000).is_none());
        // The clock does not divide to 15 MHz: 16.67, above the maximum.
        assert_eq!(samplerate(15_000_000, 15_000_000).unwrap().samplerate, 16_666_666);
        assert!(samplerate(0, 0).is_none());
    }

    #[test]
    fn the_shrimp_quirk_fills_only_what_is_not_said() {
        let mut m = Metadata { name: "Shrimp1.0".into(), channels: 8, ..Metadata::default() };
        apply_quirks(&mut m);
        assert_eq!((m.channels, m.max_samplebytes, m.max_samplerate), (8, 262_144, 20_000_000));
        let mut m = Metadata { name: "Other".into(), ..Metadata::default() };
        apply_quirks(&mut m);
        assert_eq!((m.channels, m.max_samplebytes), (0, 0));
    }
}
