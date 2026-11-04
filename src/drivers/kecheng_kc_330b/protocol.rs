// Counterpart in sigrok:
//   libsigrok/src/hardware/kecheng-kc-330b/protocol.c,
//   libsigrok/src/hardware/kecheng-kc-330b/protocol.h,
//   libsigrok/src/hardware/kecheng-kc-330b/api.c (the intervals, the identify),
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Aurelien Jacobs,
//   Daniel Elstner, Martin Ling, Soeren Apel, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The KC-330B's wire protocol: commands on bulk OUT 2, answers on bulk
//! IN 0x81, each answer starting with the command with its top bit set.
//!
//! | command | length | answer |
//! |---|---|---|
//! | `01` configure | 7 | `81` |
//! | `02` identify | 1 | `82`, length, ASCII model |
//! | `03` set date and time | 7 | `83` |
//! | `04` status | 1 | `84` activated, `a4` not |
//! | `05` log info | 1 | `85`, the six configuration bytes, count (BE) |
//! | `07` log data | 4 | `87`, then 2 BE bytes per sample |
//! | `08` live level | 1 | `88`, 2 BE bytes: dB times 10 |
//!
//! `01` carries the interval index, the alarm's low and high thresholds,
//! the time weighting (0 F, 1 S), the frequency weighting (0 A, 1 C) and
//! the data source (0 live, 1 memory, as the C sends it; the sigrok wiki
//! says the reverse, and nothing shows which is right). `03` carries the
//! year in the century, month, day, hours, minutes and seconds. `07` asks
//! at an offset in blocks of 63 samples for a count, 63 until the last
//! chunk. The C's comment says a configure clears the log. `06`, the log's
//! start time, is the wiki's and no C uses it.
//!
//! Sources: the C driver and the wiki's protocol table. No firmware source
//! or recording was found.

use std::time::Duration;

/// The USB vendor id.
pub const VENDOR: u16 = 0x1041;
/// The USB product id.
pub const PRODUCT: u16 = 0x8101;
/// Bulk OUT.
pub const EP_OUT: u8 = 0x02;
/// Bulk IN.
pub const EP_IN: u8 = 0x81;
/// The one interface, in configuration 1.
pub const INTERFACE: u8 = 0;
/// The configuration the open selects.
pub const CONFIGURATION: u8 = 1;

/// Every command's write (`api.c:75`, `protocol.c:210`).
pub const OUT_TIMEOUT: Duration = Duration::from_millis(5);
/// A short answer's read: identify, status, log info, date.
pub const ANSWER_TIMEOUT: Duration = Duration::from_millis(10);
/// The configure ack's read, "about 32ms" in coming (`protocol.c:216`).
pub const CONFIGURE_TIMEOUT: Duration = Duration::from_millis(40);
/// A level's or a chunk's read (`api.c:412`).
pub const DATA_TIMEOUT: Duration = Duration::from_millis(15);

/// The identify's read.
pub const IDENTIFY_READ: usize = 32;
/// The longest model the identify may name (`api.c:92`).
pub const MODEL_MAX: u8 = 30;
/// Samples in a block of the log, the most one request asks for.
pub const BLOCK: usize = 63;
/// The interval a meter has until set, the table's first (`protocol.h:34`,
/// whose comment says 500 ms: the table says 125).
pub const DEFAULT_INTERVAL: usize = 0;
/// The alarm's low threshold, which no option sets (`protocol.h:35`).
pub const ALARM_LOW: u8 = 40;
/// The alarm's high threshold (`protocol.h:36`).
pub const ALARM_HIGH: u8 = 120;

/// The configure command.
pub const CONFIGURE: u8 = 0x01;
/// The identify command.
pub const IDENTIFY: u8 = 0x02;
/// The set date and time command.
pub const SET_DATE_TIME: u8 = 0x03;
/// The status command.
pub const STATUS: u8 = 0x04;
/// The log info command.
pub const LOG_INFO: u8 = 0x05;
/// The log data command.
pub const LOG_DATA: u8 = 0x07;
/// The live level command.
pub const LIVE: u8 = 0x08;

/// The intervals, as `{seconds numerator, denominator}` (`api.c:41`) in ms.
pub const INTERVALS_MS: [u64; 7] = [125, 500, 1000, 2000, 5000, 10000, 60000];

/// Where a capture's samples come from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// Live readings.
    Live,
    /// The meter's log.
    Memory,
}

/// What a configure sends. The protocol has no way to read it back
/// (`api.c:132`), so a meter gets these however it was set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// Index into [`INTERVALS_MS`].
    pub interval: usize,
    /// Slow time weighting, otherwise fast.
    pub slow: bool,
    /// C frequency weighting, otherwise A.
    pub c_weight: bool,
    /// The data source.
    pub source: Source,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { interval: DEFAULT_INTERVAL, slow: false, c_weight: false, source: Source::Live }
    }
}

impl Settings {
    /// The configure command for these settings.
    pub fn command(&self) -> [u8; 7] {
        [
            CONFIGURE,
            self.interval as u8,
            ALARM_LOW,
            ALARM_HIGH,
            u8::from(self.slow),
            u8::from(self.c_weight),
            u8::from(self.source == Source::Memory),
        ]
    }
}

/// A date and time as the meter takes it: 2000 to 2099.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clock {
    /// Four digits.
    pub year: u32,
    /// 1 to 12.
    pub month: u32,
    /// 1 to 31.
    pub day: u32,
    /// 0 to 23.
    pub hour: u32,
    /// 0 to 59.
    pub minute: u32,
    /// 0 to 59.
    pub second: u32,
}

impl Clock {
    /// The set date and time command. The C writes `year - 2000` to a byte
    /// (`protocol.c:245`); [`Clock`] holds what fits.
    pub fn command(&self) -> [u8; 7] {
        [
            SET_DATE_TIME,
            (self.year.saturating_sub(2000) & 0xff) as u8,
            self.month as u8,
            self.day as u8,
            self.hour as u8,
            self.minute as u8,
            self.second as u8,
        ]
    }
}

/// The model the identify's answer names; `None` for an answer that is
/// not one. Unlike the C, whose `buf[1]` of 30 writes the terminator
/// past `buf[32]` (`api.c:97`) and whose copy takes bytes the answer never
/// held (`api.c:98`), it takes only what came.
pub fn parse_identify(answer: &[u8]) -> Option<String> {
    if answer.len() < 2
        || answer.len() > IDENTIFY_READ
        || answer[0] != IDENTIFY | 0x80
        || answer[1] > MODEL_MAX
    {
        return None;
    }
    let name = &answer[2..answer.len().min(2 + usize::from(answer[1]))];
    let name = &name[..name.iter().position(|b| *b == 0).unwrap_or(name.len())];
    Some(String::from_utf8_lossy(name).into_owned())
}

/// Whether the status answer says activated; `None` for anything else.
pub fn parse_status(answer: &[u8]) -> Option<bool> {
    match answer {
        [0x84] => Some(true),
        [0xa4] => Some(false),
        _ => None,
    }
}

/// The log info's answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogInfo {
    /// Index into [`INTERVALS_MS`], at most 6.
    pub interval: usize,
    /// Logged with slow time weighting.
    pub slow: bool,
    /// Logged with C frequency weighting.
    pub c_weight: bool,
    /// Samples in the log.
    pub stored: u16,
}

/// The log info in an answer; `None` for one that is not valid
/// (`protocol.c:324`, `protocol.c:328`).
pub fn parse_log_info(answer: &[u8]) -> Option<LogInfo> {
    let &[echo, interval, _, _, time, freq, _, hi, lo] = answer else {
        return None;
    };
    if echo != LOG_INFO | 0x80 || usize::from(interval) >= INTERVALS_MS.len() {
        return None;
    }
    Some(LogInfo {
        interval: usize::from(interval),
        slow: time != 0,
        c_weight: freq != 0,
        stored: u16::from_be_bytes([hi, lo]),
    })
}

/// A level from its two bytes: dB times 10. C divides in double and
/// stores a float (`protocol.c:156`), and so does this.
pub fn level(hi: u8, lo: u8) -> f32 {
    (f64::from(u16::from_be_bytes([hi, lo])) / 10.0) as f32
}

/// The level in a live answer; `None` for one that is not valid
/// (`protocol.c:153`).
pub fn parse_live(answer: &[u8]) -> Option<f32> {
    match answer {
        &[0x88, hi, lo] => Some(level(hi, lo)),
        _ => None,
    }
}

/// The levels in a log answer; `None` for one that is not valid. The C
/// checks only for an odd length (`protocol.c:168`); the wiki has every
/// answer echo its command.
pub fn parse_log_data(answer: &[u8]) -> Option<Vec<f32>> {
    let (&echo, levels) = answer.split_first()?;
    if echo != LOG_DATA | 0x80 || levels.len() % 2 != 0 {
        return None;
    }
    Some(levels.as_chunks::<2>().0.iter().map(|b| level(b[0], b[1])).collect())
}

/// The request for the chunk after `done` of `total` samples; `done` is a
/// whole number of blocks. The C's own offset is the same (`protocol.c:80`).
pub fn log_request(done: usize, total: usize) -> [u8; 4] {
    let block = (done / BLOCK).to_be_bytes();
    let n = block.len();
    [LOG_DATA, block[n - 2], block[n - 1], total.saturating_sub(done).min(BLOCK) as u8]
}

/// The bytes a log request's answer holds.
pub fn log_answer_len(request: &[u8; 4]) -> usize {
    1 + usize::from(request[3]) * 2
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configure_defaults_are_the_cs() {
        assert_eq!(Settings::default().command(), [1, 0, 40, 120, 0, 0, 0]);
        let s = Settings { interval: 6, slow: true, c_weight: true, source: Source::Memory };
        assert_eq!(s.command(), [1, 6, 40, 120, 1, 1, 1]);
    }

    #[test]
    fn identify_takes_what_came() {
        assert_eq!(parse_identify(b"\x82\x07KC-330B").as_deref(), Some("KC-330B"));
        assert_eq!(parse_identify(b"\x82\x00").as_deref(), Some(""));
        assert_eq!(parse_identify(b"\x82\x1eKC-330B").as_deref(), Some("KC-330B"), "past the data");
        assert_eq!(parse_identify(b"\x82\x02KC-330B").as_deref(), Some("KC"), "the length rules");
        assert_eq!(parse_identify(b"\x82\x07KC\x00330B").as_deref(), Some("KC"), "a C string");
        let thirty = [&[0x82, 30][..], &[b'x'; 30]].concat();
        assert_eq!(parse_identify(&thirty).map(|s| s.len()), Some(30));
        assert_eq!(parse_identify(&[0x82, 31]), None, "over the length");
        assert_eq!(parse_identify(&[0x82]), None, "too short");
        assert_eq!(parse_identify(&[0xc2, 1, b'x']), None, "wrong echo");
        assert_eq!(parse_identify(&[0x82, 1, b'x'].repeat(11)), None, "longer than the read");
    }

    #[test]
    fn status_takes_the_two_answers() {
        assert_eq!(parse_status(&[0x84]), Some(true));
        assert_eq!(parse_status(&[0xa4]), Some(false));
        for bad in [&[0xc4][..], &[], &[0x84, 0]] {
            assert_eq!(parse_status(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn log_info_fields() {
        let i = parse_log_info(&[0x85, 2, 40, 120, 1, 0, 0, 0x7d, 0]).unwrap();
        assert_eq!(i, LogInfo { interval: 2, slow: true, c_weight: false, stored: 32000 });
        assert_eq!(parse_log_info(&[0x85, 7, 40, 120, 1, 0, 0, 0, 1]), None, "interval");
        assert_eq!(parse_log_info(&[0xc5, 2, 40, 120, 1, 0, 0, 0, 1]), None, "echo");
        assert_eq!(parse_log_info(&[0x85, 2, 40, 120, 1, 0, 0, 0]), None, "short");
    }

    #[test]
    fn levels_are_tenths() {
        assert_eq!(parse_live(&[0x88, 0x01, 0x2c]), Some(30.0));
        assert_eq!(parse_live(&[0x88, 0x05, 0x14]), Some(130.0));
        assert_eq!(parse_live(&[0x88, 0xff, 0xff]), Some(6553.5));
        assert_eq!(level(0x01, 0x51), (33.7f64) as f32);
        assert_eq!(parse_live(&[0xc8, 0x01, 0x2c]), None);
        assert_eq!(parse_live(&[0x88, 0x01]), None);
        assert_eq!(parse_live(&[]), None);
        assert_eq!(parse_log_data(&[0x87, 0x01, 0x2c, 0x01, 0x39]), Some(vec![30.0, 31.3]));
        assert_eq!(parse_log_data(&[0x87]), Some(vec![]));
        assert_eq!(parse_log_data(&[0x87, 0x01]), None, "half a sample");
        assert_eq!(parse_log_data(&[0xc7, 0x01, 0x2c]), None, "echo");
        assert_eq!(parse_log_data(&[]), None);
    }

    #[test]
    fn requests_count_blocks() {
        assert_eq!(log_request(0, 5), [7, 0, 0, 5]);
        assert_eq!(log_request(0, 63), [7, 0, 0, 63]);
        assert_eq!(log_request(0, 64), [7, 0, 0, 63]);
        assert_eq!(log_request(63, 64), [7, 0, 1, 1]);
        assert_eq!(log_request(31500, 32000), [7, 1, 244, 63]);
        assert_eq!(log_request(31941, 32000), [7, 1, 251, 59]);
        assert_eq!(log_answer_len(&[7, 0, 0, 63]), 127);
        assert_eq!(log_answer_len(&[7, 0, 0, 1]), 3);
    }

    #[test]
    fn clock_is_the_cs_bytes() {
        let c = Clock { year: 2026, month: 11, day: 4, hour: 12, minute: 34, second: 56 };
        assert_eq!(c.command(), [3, 26, 11, 4, 12, 34, 56]);
    }
}
