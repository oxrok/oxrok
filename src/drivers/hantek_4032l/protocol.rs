// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-4032l/protocol.c,
//   libsigrok/src/hardware/hantek-4032l/protocol.h,
//   libsigrok/src/hardware/hantek-4032l/api.c (the rate and clock tables),
//   Copyright (C) 2016 Andreas Zschunke <andreas.zschunke@gmx.net>,
//   Copyright (C) 2017 Andrej Valek <andy@skyrain.eu>,
//   Copyright (C) 2017 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Hantek 4032L's wire protocol: the command packet, the status
//! packet, the sample stream's words, the rate and clock codes and the
//! threshold conversion, all from `protocol.h`, `protocol.c` and `api.c`.
//!
//! - Vendor request `0xb3` (host to device, value and index 0, ten
//!   bytes): resets the analyzer and arms it, before every capture.
//! - Bulk OUT 2 takes the command packet, 84 bytes as gcc packs
//!   `struct h4032l_cmd_pkt` under `pack(2)`: magic `0x017f`, the rate
//!   code, the trigger enables, PWM A and B, a reserved word, the sample
//!   count, the pre-trigger count, two triggers of 32 bytes and the
//!   command. Every command carries the whole packet.
//! - Bulk IN 6 gives the status (20 bytes: magic, values, status, usbxi
//!   data, FPGA version; status 2 means the capture is complete) and,
//!   after a get, the stream: the start word, the samples, one u32 each
//!   (bit `n` is channel `n`), the end word.

use crate::core::TriggerMatch;

/// USB vendor id.
pub const VID: u16 = 0x04b5;
/// USB product id.
pub const PID: u16 = 0x4032;
/// The vendor request that resets and arms the analyzer (`CMD_RESET`).
pub const REQ_RESET: u8 = 0xb3;
/// Its ten bytes.
pub const RESET_DATA: [u8; 10] = [0x0f, 0x03, 0x03, 0x03, 0, 0, 0, 0, 0, 0];

/// The command endpoint.
pub const EP_OUT: u8 = 0x02;
/// The data endpoint.
pub const EP_IN: u8 = 0x86;
/// The size of the status and first stream reads (`devc->buf`).
pub const POLL_SIZE: usize = 512;
/// The size of a data transfer (`H4032L_DATA_BUFFER_SIZE`).
pub const DATA_TRANSFER: usize = 2048;
/// The most data transfers at once (`H4032L_DATA_TRANSFER_MAX_NUM`).
pub const DATA_TRANSFER_MAX: usize = 32;

/// The smallest sample count (`H4043L_NUM_SAMPLES_MIN`).
pub const SAMPLES_MIN: u64 = 2 * 1024;
/// The largest sample count (`H4032L_NUM_SAMPLES_MAX`).
pub const SAMPLES_MAX: u64 = 64 * 1024 * 1024;
/// The sample count until one is set (`scan()`).
pub const SAMPLES_DEFAULT: u32 = 16 * 1024;

/// The command packet's length.
pub const CMD_LEN: usize = 84;
const CMD_MAGIC: u16 = 0x017f;
/// Configure.
pub const CMD_CONFIGURE: u16 = 0x2b1a;
/// Ask the status.
pub const CMD_STATUS: u16 = 0x4b3a;
/// Get the samples.
pub const CMD_GET: u16 = 0x6b5a;

/// The status packet's magic.
pub const STATUS_MAGIC: u32 = 0x2b1a_037f;
/// The word before the samples.
pub const START_MAGIC: u32 = 0x2b1a_027f;
/// The word after them.
pub const END_MAGIC: u32 = 0x4d3c_037f;
/// The status of a complete capture.
pub const STATUS_COMPLETE: u32 = 2;

/// The rates `--show` lists (`samplerates[]`).
pub const SAMPLERATES: [u64; 33] = [
    1_000,
    2_000,
    4_000,
    8_000,
    16_000,
    31_250,
    62_500,
    125_000,
    250_000,
    500_000,
    625_000,
    781_250,
    1_000_000,
    1_250_000,
    1_562_500,
    2_000_000,
    2_500_000,
    3_125_000,
    4_000_000,
    5_000_000,
    6_250_000,
    10_000_000,
    12_500_000,
    20_000_000,
    25_000_000,
    40_000_000,
    50_000_000,
    80_000_000,
    100_000_000,
    160_000_000,
    200_000_000,
    320_000_000,
    400_000_000,
];

/// The rate of each code (`samplerates_hw[]`), 0 for the codes with none.
pub const RATE_CODES: [u64; 36] = [
    100_000_000,
    50_000_000,
    25_000_000,
    12_500_000,
    6_250_000,
    3_125_000,
    1_562_500,
    781_250,
    80_000_000,
    40_000_000,
    20_000_000,
    10_000_000,
    5_000_000,
    2_500_000,
    1_250_000,
    625_000,
    4_000_000,
    2_000_000,
    1_000_000,
    500_000,
    250_000,
    125_000,
    62_500,
    31_250,
    16_000,
    8_000,
    4_000,
    2_000,
    1_000,
    0,
    0,
    0,
    200_000_000,
    160_000_000,
    400_000_000,
    320_000_000,
];

/// The code of a rate, as `config_set()` looks for it.
pub fn rate_code(rate: u64) -> Option<usize> {
    if rate == 0 {
        return None;
    }
    RATE_CODES.iter().position(|r| *r == rate)
}

/// The names of the external clock's sources (`ext_clock_sources[]`).
pub const CLOCK_SOURCES: [&str; 2] = ["ACLK", "BCLK"];
/// The names of its edges (`signal_edges[]`).
pub const CLOCK_EDGES: [&str; 3] = ["rising", "falling", "both"];

/// The rate code of an external clock (`ext_clock_edges[][]`): `source`
/// indexes [`CLOCK_SOURCES`], `edge` [`CLOCK_EDGES`].
pub fn external_clock_code(source: usize, edge: usize) -> u8 {
    const CODES: [[u8; 3]; 2] = [[0x24, 0x28, 0x26], [0x25, 0x29, 0x27]];
    CODES[source][edge]
}

/// The lowest threshold listed (`H4032L_THR_VOLTAGE_MIN`).
const THRESHOLD_MIN: f64 = -6.0;
/// The highest threshold listed (`H4032L_THR_VOLTAGE_MAX`).
const THRESHOLD_MAX: f64 = 6.0;
/// The step of the list (`H4032L_THR_VOLTAGE_STEP`).
const THRESHOLD_STEP: f64 = 0.1;
/// The index of 2.5 V in the list (`H4032L_THR_VOLTAGE_DEFAULT`).
const THRESHOLD_DEFAULT: usize = 85;

/// `std_gvar_min_max_step_thresholds()`: the list accumulates the step,
/// its error and all (the default reads 2.4999999999999964), and only
/// the value next to zero is forced to 0.
pub fn thresholds() -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut d = THRESHOLD_MIN;
    while d <= THRESHOLD_MAX + THRESHOLD_STEP / 2.0 {
        let v = if d > -THRESHOLD_STEP / 2.0 && d < THRESHOLD_STEP / 2.0 { 0.0 } else { d };
        out.push((v, v));
        d += THRESHOLD_STEP;
    }
    out
}

/// The threshold until one is set (`scan()`).
pub fn default_threshold() -> f64 {
    thresholds()[THRESHOLD_DEFAULT].0
}

/// `h4032l_voltage2pwm()`. The reference clamps at 10 V, which gives
/// 4096 where the C comment says 4095: no firmware or recording says
/// which the board wants, so this stays what the C driver sends.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn voltage_to_pwm(voltage: f64) -> u16 {
    let vref = (1.8 - voltage).clamp(-5.0, 10.0);
    ((vref + 5.0) * (4096.0 / 15.0)) as u16
}

/// What `config_set()` makes of a sample limit: rounded up to a multiple
/// of 512 and checked; `Err` holds the rounded count for the message.
/// The C driver masks with the 32 bit constant `0xfffffe00` (api.c:415),
/// which drops the upper half of a count of 4 Gi samples and more; here
/// such a count stays what it is and is refused.
pub fn round_samples(limit: u64) -> std::result::Result<u32, u64> {
    let rounded = limit.saturating_add(511) & !511;
    if !(SAMPLES_MIN..=SAMPLES_MAX).contains(&rounded) {
        return Err(rounded);
    }
    Ok(u32::try_from(rounded).expect("at most 64M"))
}

/// The edge types of the trigger's flags
/// (`enum h4032l_trigger_edge_type`).
const EDGE_RISE: u32 = 0;
const EDGE_FALL: u32 = 1;
const EDGE_TOGGLE: u32 = 2;
const EDGE_DISABLED: u32 = 3;
/// `data_range_enabled` in the flags.
const DATA_RANGE_ENABLED: u32 = 1 << 12;

/// The part of `struct h4032l_trigger` the driver sets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HwTrigger {
    edge_signal: u32,
    edge_type: u32,
    /// `data_range_mask`.
    mask: u32,
    /// `data_range_max`.
    value: u32,
}

impl HwTrigger {
    /// `dev_acquisition_start()`'s packing of one trigger stage's
    /// `(channel, match)` conditions: levels into the data range, one edge
    /// at most. `Err` holds the message the C driver logs.
    pub fn from_conditions(
        conditions: impl IntoIterator<Item = (usize, TriggerMatch)>,
    ) -> std::result::Result<HwTrigger, &'static str> {
        let mut t = HwTrigger { edge_signal: 0, edge_type: EDGE_DISABLED, mask: 0, value: 0 };
        for (channel, m) in conditions {
            let index = u32::try_from(channel).unwrap_or(u32::MAX);
            let bit = 1u32.checked_shl(index).ok_or("Unknown trigger value.")?;
            let edge = match m {
                TriggerMatch::Zero => {
                    t.mask |= bit;
                    continue;
                }
                TriggerMatch::One => {
                    t.mask |= bit;
                    t.value |= bit;
                    continue;
                }
                TriggerMatch::Rising => EDGE_RISE,
                TriggerMatch::Falling => EDGE_FALL,
                TriggerMatch::Edge => EDGE_TOGGLE,
                TriggerMatch::Over | TriggerMatch::Under => return Err("Unknown trigger value."),
            };
            if t.edge_type != EDGE_DISABLED {
                return Err("Only one trigger signal with fall/rising/edge allowed.");
            }
            t.edge_type = edge;
            t.edge_signal = index;
        }
        Ok(t)
    }
}

/// `struct h4032l_cmd_pkt`, what it holds between commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CmdPacket {
    /// The rate or external clock code.
    pub rate: u8,
    /// PWM A and B.
    pub pwm: [u16; 2],
    /// The sample count.
    pub samples: u32,
    /// The pre-trigger count.
    pub pre: u32,
    /// Trigger 1, when enabled.
    pub trigger: Option<HwTrigger>,
}

impl Default for CmdPacket {
    /// As `scan()` initialises it.
    fn default() -> Self {
        CmdPacket { rate: 0, pwm: [0, 0], samples: SAMPLES_DEFAULT, pre: 0, trigger: None }
    }
}

impl CmdPacket {
    /// The bytes on the wire, with command `cmd`. Fields the driver never
    /// sets are 0.
    pub fn bytes(&self, cmd: u16) -> [u8; CMD_LEN] {
        let mut b = [0u8; CMD_LEN];
        b[0..2].copy_from_slice(&CMD_MAGIC.to_le_bytes());
        b[2] = self.rate;
        b[3] = u8::from(self.trigger.is_some());
        b[4..6].copy_from_slice(&self.pwm[0].to_le_bytes());
        b[6..8].copy_from_slice(&self.pwm[1].to_le_bytes());
        b[10..14].copy_from_slice(&self.samples.to_le_bytes());
        b[14..18].copy_from_slice(&self.pre.to_le_bytes());
        if let Some(t) = self.trigger {
            let flags = t.edge_signal | t.edge_type << 5 | DATA_RANGE_ENABLED;
            b[18..22].copy_from_slice(&flags.to_le_bytes());
            b[26..30].copy_from_slice(&t.value.to_le_bytes());
            b[38..42].copy_from_slice(&t.mask.to_le_bytes());
        }
        b[82..84].copy_from_slice(&cmd.to_le_bytes());
        b
    }
}

/// The little endian word at index `word` of `b`.
pub fn word_at(b: &[u8], word: usize) -> u32 {
    u32::from_le_bytes([b[word * 4], b[word * 4 + 1], b[word * 4 + 2], b[word * 4 + 3]])
}

/// How many data transfers `h4032l_start_data_transfers()` submits for
/// `remaining` samples.
pub fn data_transfers(remaining: u32, fpga_version: u32) -> usize {
    let wanted = remaining as usize * 4 / DATA_TRANSFER;
    wanted.min(if fpga_version == 0 { 1 } else { DATA_TRANSFER_MAX }).max(1)
}

/// `h4032l_get_fpga_version()`'s answer, from the read buffer.
pub fn status_fpga_version(buf: &[u8; POLL_SIZE]) -> Option<u32> {
    (word_at(buf, 0) == STATUS_MAGIC).then(|| word_at(buf, 4))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_command_packet_is_laid_out_as_gcc_packs_it() {
        let p = CmdPacket {
            rate: 0x24,
            pwm: [1174, 0x0102],
            samples: 2048,
            pre: 102,
            trigger: Some(
                HwTrigger::from_conditions([
                    (0, TriggerMatch::One),
                    (1, TriggerMatch::Zero),
                    (31, TriggerMatch::Falling),
                ])
                .unwrap(),
            ),
        };
        let b = p.bytes(CMD_CONFIGURE);
        assert_eq!(b.len(), 84);
        assert_eq!(&b[0..4], &[0x7f, 0x01, 0x24, 0x01]);
        assert_eq!(&b[4..10], &[0x96, 0x04, 0x02, 0x01, 0, 0]);
        assert_eq!(&b[10..18], &[0, 8, 0, 0, 102, 0, 0, 0]);
        // Edge signal 31, type fall, data range on.
        assert_eq!(&b[18..22], &[0x3f, 0x10, 0, 0]);
        // data_range_max at 26, data_range_mask at 38.
        assert_eq!(&b[26..30], &[1, 0, 0, 0]);
        assert_eq!(&b[38..42], &[3, 0, 0, 0]);
        assert!(b[50..82].iter().all(|x| *x == 0));
        assert_eq!(&b[82..], &[0x1a, 0x2b]);
    }

    #[test]
    fn a_packet_without_trigger_leaves_it_zero() {
        let b = CmdPacket::default().bytes(CMD_STATUS);
        assert_eq!(&b[..4], &[0x7f, 0x01, 0, 0]);
        assert_eq!(&b[10..14], &[0, 0x40, 0, 0]);
        assert!(b[18..82].iter().all(|x| *x == 0));
        assert_eq!(&b[82..], &[0x3a, 0x4b]);
    }

    #[test]
    fn a_second_edge_is_refused_whichever_it_is() {
        for (a, b) in [
            (TriggerMatch::Rising, TriggerMatch::Rising),
            (TriggerMatch::Rising, TriggerMatch::Falling),
            (TriggerMatch::Falling, TriggerMatch::Edge),
        ] {
            assert_eq!(
                HwTrigger::from_conditions([(0, a), (1, b)]),
                Err("Only one trigger signal with fall/rising/edge allowed.")
            );
        }
        assert_eq!(
            HwTrigger::from_conditions([(0, TriggerMatch::Over)]),
            Err("Unknown trigger value.")
        );
    }

    #[test]
    fn rate_codes_follow_the_table() {
        assert_eq!(rate_code(100_000_000), Some(0));
        assert_eq!(rate_code(1_000), Some(28));
        assert_eq!(rate_code(320_000_000), Some(35));
        assert_eq!(rate_code(3_000_000), None);
        assert_eq!(rate_code(0), None);
        for r in SAMPLERATES {
            assert!(rate_code(r).is_some(), "{r}");
        }
    }

    #[test]
    fn external_clock_codes_follow_the_enum() {
        assert_eq!(external_clock_code(0, 0), 0x24);
        assert_eq!(external_clock_code(1, 0), 0x25);
        assert_eq!(external_clock_code(0, 2), 0x26);
        assert_eq!(external_clock_code(1, 2), 0x27);
        assert_eq!(external_clock_code(0, 1), 0x28);
        assert_eq!(external_clock_code(1, 1), 0x29);
    }

    #[test]
    fn thresholds_accumulate_as_the_c_loop_does() {
        let t = thresholds();
        assert_eq!(t.len(), 121);
        assert_eq!(t[0], (-6.0, -6.0));
        assert_eq!(t[60], (0.0, 0.0));
        assert_eq!(default_threshold(), 2.499_999_999_999_996_4);
    }

    #[test]
    fn pwm_as_the_c_cast_truncates() {
        // The default threshold's value in the goldens: 0x0496.
        assert_eq!(voltage_to_pwm(default_threshold()), 1174);
        assert_eq!(voltage_to_pwm(9.0), 0);
        assert_eq!(voltage_to_pwm(-9.0), 4096);
        assert_eq!(voltage_to_pwm(-6.0), 3495);
    }

    #[test]
    fn sample_limits_round_up_and_keep_their_upper_bits() {
        assert_eq!(round_samples(2048), Ok(2048));
        assert_eq!(round_samples(3000), Ok(3072));
        assert_eq!(round_samples(64 << 20), Ok(64 << 20));
        assert_eq!(round_samples(1000), Err(1024));
        assert_eq!(round_samples((64 << 20) + 1), Err((64 << 20) + 512));
        // api.c:415 would take this one as 2048.
        assert_eq!(round_samples((1 << 32) + 2048), Err((1 << 32) + 2048));
        assert!(round_samples(u64::MAX).is_err());
    }

    #[test]
    fn data_transfers_as_the_c_driver_counts_them() {
        assert_eq!(data_transfers(1921, 0x4303), 3);
        assert_eq!(data_transfers(1 << 20, 0x4303), 32);
        assert_eq!(data_transfers(1 << 20, 0), 1);
        assert_eq!(data_transfers(100, 0x4303), 1);
    }
}
