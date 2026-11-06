// Counterpart in sigrok:
//   libsigrok/src/hardware/link-mso19/api.c,
//   libsigrok/src/hardware/link-mso19/protocol.c,
//   libsigrok/src/hardware/link-mso19/protocol.h,
//   Copyright (C) 2011 Daniel Ribeiro <drwyrm@gmail.com>,
//   Copyright (C) 2012 Renato Caldas <rmsc@fe.up.pt>,
//   Copyright (C) 2013 Lior Elazary <lelazary@yahoo.com>,
//   Copyright (C) 2022 Paul Kasemir <paul.kasemir@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Bert Vermeulen,
//   Martin Ling, Soeren Apel, Daniel Elstner, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The MSO-19's wire: registers in framed words, a status byte, a dump.
//!
//! A frame is `@LDS~`, big endian words, `~`. A word holds a register in bits 8
//! to 11 and a value around them: bits 0 to 5 in place, bits 6 and 7 in bits 12
//! and 13, bit 6 the inverse of value bit 5, bit 14 the inverse of value bit 7. So
//! no word holds `~`, XON or XOFF.
//!
//! Register 15 selects a bank in its two low bits and holds the slow mode. Bank 0
//! has the status and dump requests, the trigger, the clock and the DAC; bank 2
//! the SPI and I2C trigger, which has no setting here and is written with its
//! defaults on every start.
//!
//! A status request answers one byte. A dump request answers 1024 samples of
//! three bytes: the analog value in the 6 low bits of the first and the 4 low
//! bits of the second, the logic byte in bits 4 and 5 of the second and the 6 low
//! bits of the third. The high bits are not read.
//!
//! The calibration comes from the USB serial number; no request reads it.
//!
//! The arithmetic is C's on x86-64, as it decides what goes on the wire: a double
//! that does not fit an `int` is `INT_MIN` ([`c_int`]) and an `int` wraps.

use crate::core::{Error, Result};
use crate::serial::{self, Port};
use std::time::Duration;

/// How the port is opened.
pub const SERIALCOMM: &str = "460800/8n1/flow=2";
/// `idVendor`.
pub const USB_VENDOR: u16 = 0x3195;
/// `idProduct`.
pub const USB_PRODUCT: u16 = 0xf190;
/// Samples in a dump, and all a capture can be.
pub const NUM_SAMPLES: usize = 1024;
/// Bytes in a dump.
pub const DUMP_BYTES: usize = NUM_SAMPLES * 3;

const HEAD: &[u8] = b"@LDS~";
const FOOT: u8 = b'~';

const REG_BUFFER: u8 = 1;
const REG_STATUS: u8 = 2;
const REG_TRIG_THRESH: u8 = 3;
const REG_TRIG: u8 = 4;
const REG_TRIG_LA_VAL: u8 = 5;
const REG_TRIG_LA_MASK: u8 = 6;
const REG_TRIG_POS_LSB: u8 = 7;
const REG_TRIG_POS_MSB: u8 = 8;
const REG_CLKRATE1: u8 = 9;
const REG_CLKRATE2: u8 = 10;
const REG_TRIG_WIDTH: u8 = 11;
const REG_DAC_MSB: u8 = 12;
const REG_DAC_LSB: u8 = 13;
const REG_CTL1: u8 = 14;
const REG_CTL2: u8 = 15;

const REG_PT_MASK: u8 = 4;
const REG_PT_SPIMODE: u8 = 8;

const CTL1_RESET_FSM: u8 = 1 << 0;
const CTL1_ARM: u8 = 1 << 1;
const CTL1_ADC_ENABLE: u8 = 1 << 4;
const CTL1_LOAD_DAC: u8 = 1 << 5;
const CTL1_RESET_ADC: u8 = 1 << 6;
const CTL1_DC_COUPLING: u8 = 1 << 7;

const TRIG_EDGE: u8 = 1 << 2;
const TRIG_SRC_LA: u8 = 7 << 5;
const TRIG_THRESH_MSB_MASK: u16 = 3;
/// A trigger at 0 V.
const TRIG_THRESH_START: i32 = 0x200;

const TRIG_POS_VALUE_MASK: u16 = 0x7fff;
const TRIG_POS_NEGATIVE: u16 = 1 << 15;

const DAC_DSO_VALUE_MASK: i32 = 0xfff;
const DAC_SELECT_LA: u16 = 1 << 15;

const STATUS_MASK: u8 = 0x7;
const STATUS_DATA_READY: u8 = 6;

const VBIT_CALIBRATION_DENOMINATOR: f64 = 10_000.0 * 1000.0;
const OFFSET_VBIT_CALIBRATION_NUMERATOR: f64 = 3.0;

/// `samplerate`: the rate, the divider, the slow mode bit of register 15. mso19fcgi
/// has other dividers for the same rates, a step off these (0x0362 for 2 MHz), where
/// its own comments' counts are these: C's are kept.
pub const RATES: [(u64, u16, u8); 20] = [
    (200_000_000, 0x0205, 0),
    (100_000_000, 0x0105, 0),
    (50_000_000, 0x0005, 0),
    (20_000_000, 0x0303, 0),
    (10_000_000, 0x0308, 0),
    (5_000_000, 0x0312, 0),
    (2_000_000, 0x0330, 0),
    (1_000_000, 0x0362, 0),
    (500_000, 0x03c6, 0),
    (200_000, 0x07f2, 0),
    (100_000, 0x0fe6, 0),
    (50_000, 0x1fce, 0),
    (20_000, 0x4f86, 0),
    (10_000, 0x9f0e, 0),
    (5_000, 0x03c7, 0x20),
    (2_000, 0x07f3, 0x20),
    (1_000, 0x0fe7, 0x20),
    (500, 0x1fcf, 0x20),
    (200, 0x4f87, 0x20),
    (100, 0x9f0f, 0x20),
];

/// `logic_threshold`, the names.
pub const LOGIC_THRESHOLDS: [&str; 6] =
    ["1.2V Logic", "1.5V Logic", "1.8V Logic", "2.5V Logic", "3.0V Logic", "3.3V/5.0V Logic"];
/// The DAC's word for each, which C has from a wireshark capture.
const LOGIC_THRESHOLD_VALUES: [u16; 6] = [0x600, 0x770, 0x8ff, 0xc70, 0xeff, 0xfff];

/// `coupling`.
pub const COUPLINGS: [&str; 2] = ["DC", "AC"];
/// `triggersource`.
pub const TRIGGER_SOURCES: [&str; 2] = ["DSO", "LA"];
/// `triggerslope` on the analog group.
pub const DSO_SLOPES: [&str; 2] = ["Rising", "Falling"];
/// `triggerslope` on the logic group.
pub const LA_SLOPES: [&str; 2] = ["F->T", "T->F"];

/// A double converted to `int`, as C's does on x86-64: out of range, or no
/// number, is `INT_MIN`.
pub fn c_int(value: f64) -> i32 {
    // Every i32 is exact in a double. What truncates into the range is in it.
    if value > -2_147_483_649.0 && value < 2_147_483_648.0 { value as i32 } else { i32::MIN }
}

/// `mso_trans`: register `a` holds `v`.
pub fn word(a: u8, v: u8) -> [u8; 2] {
    let high = (a & 0xf) | ((v >> 6) << 4) | if v & 0x80 == 0 { 0x40 } else { 0 };
    let low = (v & 0x3f) | if v & 0x20 == 0 { 0x40 } else { 0 };
    [high, low]
}

/// `mso_send_control_message`'s bytes.
pub fn frame(words: &[[u8; 2]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEAD.len() + words.len() * 2 + 1);
    out.extend_from_slice(HEAD);
    for w in words {
        out.extend_from_slice(w);
    }
    out.push(FOOT);
    out
}

/// `mso_send_control_message`. C loops on a write of 10 ms for as long as the
/// device takes nothing; [`Port::write`] gives up after its 5 s.
pub fn send(port: &mut dyn Port, words: &[[u8; 2]]) -> Result<()> {
    port.write(&frame(words))
}

/// `mso_configure_rate`'s table: the rate's words and its slow mode bit. C takes a
/// `uint32_t`, so a rate 2^32 over one of these is that one; here it is not in the
/// table.
pub fn rate_words(rate: u64) -> Option<([[u8; 2]; 2], u8)> {
    let (_, value, slow) = RATES.iter().find(|(r, ..)| *r == rate)?;
    let [lsb, msb] = value.to_le_bytes();
    Some(([word(REG_CLKRATE1, msb), word(REG_CLKRATE2, lsb)], *slow))
}

/// What a dump request is.
pub fn read_buffer_request() -> [[u8; 2]; 1] {
    [word(REG_BUFFER, 0)]
}

/// What a status request is.
pub fn read_status_request() -> [[u8; 2]; 1] {
    [word(REG_STATUS, 0)]
}

/// `mso_validate_status`, without its message.
pub fn status_valid(value: u8) -> bool {
    let action = value & STATUS_MASK;
    action != 0 && action <= STATUS_DATA_READY && value & 0xc0 == 0
}

/// The action says the dump can be read.
pub fn status_ready(value: u8) -> bool {
    value & STATUS_MASK == STATUS_DATA_READY
}

/// `mso_read_status` with a place for the reply: the request, then one byte in
/// `timeout`. C's 10 ms fails on a loaded host where nothing is wrong, so the
/// caller has a longer one, which costs only a device that does not answer.
pub fn read_status(port: &mut dyn Port, timeout: Duration) -> Result<u8> {
    log::debug!("Requesting status.");
    send(port, &read_status_request())?;
    let mut byte = [0u8];
    if serial::read_blocking(port, &mut byte, timeout)? != 1 {
        log::error!("Reading status failed");
        return Err(Error::device("Reading status failed"));
    }
    if !status_valid(byte[0]) {
        log::error!("Invalid status byte {:02x}", byte[0]);
        return Err(Error::device(format!("Invalid status byte {:02x}", byte[0])));
    }
    log::debug!("Status is: {:#x}.", byte[0]);
    Ok(byte[0])
}

/// What the USB serial number holds. No request reads any of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// Volts per bit of the analog value.
    pub vbit: f64,
    /// The DAC's word at 0 V.
    pub dac_offset: u16,
    /// Volts per bit of the DAC.
    pub offset_vbit: f64,
    /// The hardware revision.
    pub hwrev: u8,
}

/// `sscanf`'s `%Nu`, from `at`: white space, a sign within the width, digits
/// within it, the negative wrapping.
fn scan_unsigned(text: &[u8], at: &mut usize, width: usize) -> Option<u32> {
    while text.get(*at).is_some_and(u8::is_ascii_whitespace) {
        *at += 1;
    }
    let mut left = width;
    let negative = match text.get(*at) {
        Some(b'-') => true,
        Some(b'+') => false,
        Some(_) | None => {
            return scan_digits(text, at, left);
        }
    };
    *at += 1;
    left -= 1;
    let value = scan_digits(text, at, left)?;
    Some(if negative { value.wrapping_neg() } else { value })
}

fn scan_digits(text: &[u8], at: &mut usize, width: usize) -> Option<u32> {
    let mut value = 0u32;
    let mut digits = 0;
    while digits < width {
        let Some(digit) = text.get(*at).filter(|c| c.is_ascii_digit()) else {
            break;
        };
        value = value.wrapping_mul(10).wrapping_add(u32::from(digit - b'0'));
        *at += 1;
        digits += 1;
    }
    (digits > 0).then_some(value)
}

/// `mso_parse_serial`: 5, 3, 3, 1, 1 and 6 digits, the first a 4. What follows
/// is not read: real units have 11 digits more, which mso19fcgi uses for 200 MSa/s
/// and C says nothing about. A field of 0 takes C's default.
pub fn parse_serial(serial: &str) -> Option<Calibration> {
    let text = serial.as_bytes();
    if text.first() != Some(&b'4') {
        return None;
    }
    let mut at = 0;
    let u1 = scan_unsigned(text, &mut at, 5)?;
    let u2 = scan_unsigned(text, &mut at, 3)?;
    let u3 = scan_unsigned(text, &mut at, 3)?;
    let _model = scan_unsigned(text, &mut at, 1)?;
    let u5 = scan_unsigned(text, &mut at, 1)?;
    let _unit = scan_unsigned(text, &mut at, 6)?;
    let u2 = if u2 == 0 { 343 } else { u2 };
    let u3 = if u3 == 0 { 500 } else { u3 };
    Some(Calibration {
        vbit: f64::from(u1) / VBIT_CALIBRATION_DENOMINATOR,
        dac_offset: u2.to_le_bytes()[0..2].try_into().map(u16::from_le_bytes).expect("two bytes"),
        offset_vbit: OFFSET_VBIT_CALIBRATION_NUMERATOR / f64::from(u3),
        hwrev: u5.to_le_bytes()[0],
    })
}

/// Which trigger a capture uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerSource {
    /// The analog input.
    Dso,
    /// The logic inputs.
    La,
}

/// `dev_context`: the settings, and what follows from them.
#[derive(Clone, Debug)]
pub struct Settings {
    /// From the serial number.
    pub calibration: Calibration,
    /// `samplerate`.
    pub cur_rate: u64,
    /// `coupling`, an index in [`COUPLINGS`].
    pub coupling: usize,
    /// `probe_factor`.
    pub probe_factor: u16,
    /// `triggersource`.
    pub trigger_source: TriggerSource,
    /// `triggerslope` on the analog group, an index in [`DSO_SLOPES`].
    pub dso_slope: usize,
    /// `triggerslope` on the logic group, an index in [`LA_SLOPES`].
    pub la_slope: usize,
    /// `logic_threshold`, an index in [`LOGIC_THRESHOLDS`].
    pub logic_threshold: usize,
    /// `horiz_triggerpos`.
    pub horiz_triggerpos: f64,
    /// `triggerlevel`.
    pub trigger_level: f64,
    /// `offset`.
    pub offset: f64,
    /// The trigger level the range holds.
    trigger_adjusted: f64,
    /// The offset the DAC holds.
    offset_adjusted: f64,
    /// The DAC's word for it.
    offset_value: u16,
    /// The trigger position's registers.
    trig_pos: u16,
    /// Register 14's lasting bits.
    ctlbase1: u8,
    /// Register 15's lasting bits.
    ctlbase2: u8,
    /// The value a logic trigger matches, from the session's trigger.
    pub la_trigger: u8,
    /// The channels it does not look at, as bits.
    pub la_trigger_mask: u8,
}

impl Settings {
    /// What `scan_handle_port` starts with.
    pub fn new(calibration: Calibration) -> Settings {
        let mut settings = Settings {
            calibration,
            cur_rate: 10_000,
            coupling: 0,
            probe_factor: 10,
            trigger_source: TriggerSource::Dso,
            dso_slope: 0,
            la_slope: 0,
            logic_threshold: LOGIC_THRESHOLDS.len() - 1,
            horiz_triggerpos: 0.0,
            trigger_level: 0.0,
            offset: 0.0,
            trigger_adjusted: 0.0,
            offset_adjusted: 0.0,
            offset_value: 0,
            trig_pos: 0,
            ctlbase1: CTL1_ADC_ENABLE,
            ctlbase2: 0,
            la_trigger: 0,
            la_trigger_mask: 0xff,
        };
        settings.set_trigger_pos(0.5);
        settings.set_offset(0.0);
        settings
    }

    /// `mso_set_trigger_level`. A value compared with itself, or a limit it was set
    /// to, is exact.
    #[allow(clippy::float_cmp)]
    pub fn set_trigger_level(&mut self, level: f64) {
        let range = 2.0 * f64::from(self.probe_factor);
        let max_level = range - self.offset_adjusted;
        let min_level = -range - self.offset_adjusted;
        self.trigger_level = level;
        self.trigger_adjusted = if level < min_level {
            min_level
        } else if level > max_level {
            max_level
        } else {
            level
        };
        if level != self.trigger_adjusted {
            log::info!("Adjusted dso trigger level to {:.6}", self.trigger_adjusted);
        }
    }

    /// `mso_set_trigger_pos`.
    pub fn set_trigger_pos(&mut self, position: f64) {
        self.horiz_triggerpos = position;
        let samples = i32::try_from(NUM_SAMPLES).expect("1024");
        let mut pos = c_int(position * f64::from(samples) + 0.5);
        let mut sign = 0;
        if pos < 0 {
            pos = pos.wrapping_neg();
            sign = TRIG_POS_NEGATIVE;
            if pos < samples + 10 {
                // The vendor's program does not allow these, but C's comment
                // has them sometimes working.
                log::warn!("Trigger holdoff > -1.011 may be unstable {position:.6}");
            }
        } else if pos >= samples {
            pos = samples - 1;
        } else if pos < 10 {
            // C's comment: the vendor's program never goes under 10, and it
            // sometimes fails to trigger there.
            log::warn!("Trigger position < .01 may be unstable {position:.6}");
        }
        self.trig_pos =
            pos.to_le_bytes()[0..2].try_into().map(u16::from_le_bytes).expect("two bytes")
                & TRIG_POS_VALUE_MASK;
        if self.trig_pos != 0 {
            self.trig_pos |= sign;
        }
    }

    /// `mso_set_offset_value`, which moves the range the trigger level is
    /// limited to.
    #[allow(clippy::float_cmp)]
    pub fn set_offset(&mut self, offset: f64) {
        self.offset = offset;
        let probe = f64::from(self.probe_factor);
        let scaled = offset / probe;
        // MIN(2.0, MAX(-2.0, scaled)): no number goes through both.
        let low = if -2.0 > scaled { -2.0 } else { scaled };
        let limited = if 2.0 < low { 2.0 } else { low };
        let dac_offset = self.calibration.dac_offset;
        let value = c_int(f64::from(dac_offset) - limited / self.calibration.offset_vbit);
        let value = value.clamp(0, DAC_DSO_VALUE_MASK);
        self.offset_value = u16::try_from(value).expect("under 0x1000");
        // An int times an int, then a double.
        let product = (i32::from(dac_offset) - value).wrapping_mul(i32::from(self.probe_factor));
        self.offset_adjusted = f64::from(product) * self.calibration.offset_vbit;
        if limited != scaled {
            log::info!("Adjusted dso offset to {:.6}", self.offset_adjusted);
        }
        self.set_trigger_level(self.trigger_level);
    }

    /// `probe_factor`, which scales the offset and the trigger level's range.
    pub fn set_probe_factor(&mut self, factor: u16) {
        self.probe_factor = factor;
        self.set_offset(self.offset);
    }

    /// `mso_calc_trigger_threshold`, in doubles. C since 932a3452 holds the sum
    /// in an `int` and divides it by the probe factor as an `int`, so 15 V on a
    /// 10x probe is 1 V and 9 V is 0 V; the commit before it, and mso19fcgi,
    /// divide the voltage. On whole volts of the probe's own scale they agree.
    pub fn trigger_threshold(&self) -> u16 {
        let volts = self.trigger_adjusted + self.offset_adjusted;
        let raw = c_int(volts / f64::from(self.probe_factor) / self.calibration.vbit);
        TRIG_THRESH_START.wrapping_sub(raw).to_le_bytes()[0..2]
            .try_into()
            .map(u16::from_le_bytes)
            .expect("two bytes")
    }

    /// `mso_calc_trigger_register`. Only the DSO's edge and the LA's: the pulse,
    /// SPI and I2C sources have no setting.
    pub fn trigger_register(&self, threshold: u16) -> u8 {
        let thresh_msb = ((threshold >> 8) & TRIG_THRESH_MSB_MASK).to_le_bytes()[0];
        let (source, edge) = match self.trigger_source {
            TriggerSource::Dso => (0, if self.dso_slope == 1 { TRIG_EDGE } else { 0 }),
            // F->T sets it, and T->F does not.
            TriggerSource::La => (TRIG_SRC_LA, if self.la_slope == 0 { TRIG_EDGE } else { 0 }),
        };
        source | edge | thresh_msb
    }

    /// `mso_bank_select`.
    fn bank_select(&self, bank: u8) -> [u8; 2] {
        word(REG_CTL2, self.ctlbase2 | (bank & 3))
    }

    /// The words of `mso_reset_adc`.
    pub fn reset_adc_words(&self) -> [[u8; 2]; 3] {
        [self.bank_select(0), word(REG_CTL1, CTL1_RESET_ADC), word(REG_CTL1, 0)]
    }

    /// The words of `mso_reset_fsm`.
    pub fn reset_fsm_words(&self) -> [[u8; 2]; 1] {
        [word(REG_CTL1, self.ctlbase1 | CTL1_RESET_FSM)]
    }

    /// The words of `mso_arm`.
    pub fn arm_words(&self) -> [[u8; 2]; 3] {
        [
            word(REG_CTL1, self.ctlbase1 | CTL1_RESET_FSM),
            word(REG_CTL1, self.ctlbase1 | CTL1_ARM),
            word(REG_CTL1, self.ctlbase1),
        ]
    }

    /// The words of `mso_dac_out`.
    fn dac_words(&self, value: u16) -> [[u8; 2]; 3] {
        let [lsb, msb] = value.to_le_bytes();
        [
            word(REG_DAC_MSB, msb),
            word(REG_DAC_LSB, lsb),
            word(REG_CTL1, self.ctlbase1 | CTL1_LOAD_DAC),
        ]
    }

    /// The DAC's word for the offset.
    pub fn dac_offset_words(&self) -> [[u8; 2]; 3] {
        self.dac_words(self.offset_value)
    }

    /// The DAC's word for the logic threshold.
    pub fn dac_threshold_words(&self) -> [[u8; 2]; 3] {
        self.dac_words(DAC_SELECT_LA | LOGIC_THRESHOLD_VALUES[self.logic_threshold])
    }

    /// The words of `mso_configure_trigger`. The width register is C's field that
    /// nothing sets, 0 at every rate. The protocol trigger's are its defaults,
    /// since no setting reaches them.
    pub fn trigger_words(&self) -> [[u8; 2]; 18] {
        let threshold = self.trigger_threshold();
        let [thresh_lsb, _] = threshold.to_le_bytes();
        let [position_low, position_high] = self.trig_pos.to_le_bytes();
        [
            word(REG_TRIG_THRESH, thresh_lsb),
            word(REG_TRIG, self.trigger_register(threshold)),
            word(REG_TRIG_LA_VAL, self.la_trigger),
            word(REG_TRIG_LA_MASK, self.la_trigger_mask),
            word(REG_TRIG_POS_LSB, position_low),
            word(REG_TRIG_POS_MSB, position_high),
            word(REG_TRIG_WIDTH, 0),
            self.bank_select(2),
            word(0, 0),
            word(1, 0),
            word(2, 0),
            word(3, 0),
            word(REG_PT_MASK, 0xff),
            word(REG_PT_MASK + 1, 0xff),
            word(REG_PT_MASK + 2, 0xff),
            word(REG_PT_MASK + 3, 0xff),
            word(REG_PT_SPIMODE, 0),
            self.bank_select(0),
        ]
    }

    /// Register 15's lasting bit for the rate that went out, for the next bank
    /// select.
    pub fn set_slow_mode(&mut self, slow: u8) {
        self.ctlbase2 = slow;
    }

    /// `dev_acquisition_start`'s coupling bit.
    pub fn apply_coupling(&mut self) {
        if self.coupling == 0 {
            self.ctlbase1 |= CTL1_DC_COUPLING;
        } else {
            self.ctlbase1 &= !CTL1_DC_COUPLING;
        }
    }

    /// `mso_receive_data`'s conversion: logic bytes and volts.
    pub fn decode(&self, dump: &[u8]) -> (Vec<u8>, Vec<f32>) {
        let mut logic = Vec::with_capacity(NUM_SAMPLES);
        let mut analog = Vec::with_capacity(NUM_SAMPLES);
        let probe = f64::from(self.probe_factor);
        for sample in dump.as_chunks::<3>().0 {
            let raw = u16::from(sample[0] & 0x3f) | (u16::from(sample[1] & 0x0f) << 6);
            let volts =
                (512.0 - f64::from(raw)) * self.calibration.vbit * probe - self.offset_adjusted;
            analog.push(volts as f32);
            logic.push(((sample[1] & 0x30) >> 4) | ((sample[2] & 0x3f) << 2));
        }
        (logic, analog)
    }

    /// Samples ahead of the trigger, and whether the trigger comes between: not
    /// with a holdoff, which has them all ahead and says nothing.
    pub fn trigger_split(&self) -> (usize, bool) {
        if self.trig_pos & TRIG_POS_NEGATIVE != 0 {
            (NUM_SAMPLES, false)
        } else {
            (usize::from(self.trig_pos & TRIG_POS_VALUE_MASK).min(NUM_SAMPLES), true)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_calibration() -> Calibration {
        parse_serial("4287434350012000123").unwrap()
    }

    fn settings() -> Settings {
        Settings::new(default_calibration())
    }

    #[test]
    fn a_word_is_as_the_c_macro_has_it() {
        // g_htons(v & 0x3f | (v & 0xc0) << 6 | (a & 0xf) << 8 | (~v & 0x20) << 1
        // | (~v & 0x80) << 7), worked by hand.
        assert_eq!(word(2, 0), [0x42, 0x40]);
        assert_eq!(word(14, 0x11), [0x4e, 0x51]);
        assert_eq!(word(14, 0x20), [0x4e, 0x20]);
        assert_eq!(word(9, 0x9f), [0x29, 0x5f]);
        assert_eq!(word(5, 0xff), [0x35, 0x3f]);
        assert_eq!(word(15, 0x40), [0x5f, 0x40]);
    }

    #[test]
    fn no_word_holds_a_frame_end_or_flow_control() {
        for a in 0..16 {
            for v in 0..=255 {
                for byte in word(a, v) {
                    assert!(!matches!(byte, 0x7e | 0x11 | 0x13), "{a} {v}");
                }
            }
        }
    }

    #[test]
    fn a_frame_is_head_words_foot() {
        assert_eq!(frame(&[word(2, 0)]), b"@LDS~\x42\x40~");
    }

    #[test]
    fn status_validity() {
        for bad in [0x00, 0x07, 0x40, 0x80, 0x21 | 0x40, 0xff, 0x20] {
            assert!(!status_valid(bad), "{bad:#x}");
        }
        for good in [0x01, 0x06, 0x21, 0x36, 0x3e] {
            assert!(status_valid(good), "{good:#x}");
        }
        assert!(status_ready(0x36));
        assert!(!status_ready(0x35));
    }

    #[test]
    fn a_serial_number_is_its_fields() {
        let c = parse_serial("428743435001200012342874343500").unwrap();
        assert!((c.vbit - 0.004_287_4).abs() < 1e-12);
        assert_eq!(c.dac_offset, 343);
        assert!((c.offset_vbit - 0.006).abs() < 1e-12);
        assert_eq!(c.hwrev, 2);
    }

    #[test]
    fn a_serial_number_field_of_0_takes_a_default() {
        let c = parse_serial("4000000000012000123").unwrap();
        assert_eq!(c.dac_offset, 343);
        assert!((c.offset_vbit - 0.006).abs() < 1e-12);
        assert!((c.vbit - 0.004).abs() < 1e-12);
    }

    #[test]
    fn a_serial_number_goes_as_sscanf_reads_it() {
        for bad in ["", "x", "3287434350012000123", "4287434350012", "4xxxx343500120001"] {
            assert_eq!(parse_serial(bad), None, "{bad}");
        }
        // One digit is enough for the last field.
        assert!(parse_serial("42874343500120").is_some());
        // A sign takes a place of the width, and a negative wraps.
        let c = parse_serial("41234-5060012000123").unwrap();
        assert_eq!(c.dac_offset, 0xffce);
        assert!((c.offset_vbit - 3.0 / 600.0).abs() < 1e-12);
        // White space is skipped and does not count.
        let c = parse_serial("42874 343 500 1 2 000123").unwrap();
        assert_eq!(c.dac_offset, 343);
        assert_eq!(c.hwrev, 2);
        // A sign with no room for a digit.
        assert_eq!(parse_serial("4287434350012-"), None);
        assert_eq!(parse_serial("428743435001-0"), None);
    }

    #[test]
    fn c_int_is_x86_on_what_does_not_fit() {
        assert_eq!(c_int(1.9), 1);
        assert_eq!(c_int(-1.9), -1);
        assert_eq!(c_int(2_147_483_647.5), i32::MAX);
        assert_eq!(c_int(-2_147_483_648.5), i32::MIN);
        assert_eq!(c_int(2_147_483_648.0), i32::MIN);
        assert_eq!(c_int(-2_147_483_649.0), i32::MIN);
        assert_eq!(c_int(f64::NAN), i32::MIN);
        assert_eq!(c_int(f64::INFINITY), i32::MIN);
    }

    #[test]
    fn the_position_splits_the_samples() {
        let mut s = settings();
        assert_eq!(s.trigger_split(), (512, true));
        for (position, want) in [
            (0.0, (0, true)),
            (0.25, (256, true)),
            (1.0, (1023, true)),
            (-2.5, (1024, false)),
            // Rounds to 0, which has no sign.
            (-0.0001, (0, true)),
        ] {
            s.set_trigger_pos(position);
            assert_eq!(s.trigger_split(), want, "{position}");
        }
        s.set_trigger_pos(-10.0);
        assert_eq!(s.trig_pos, 0x8000 | 0x27ff);
        s.set_trigger_pos(f64::NAN);
        assert_eq!(s.trig_pos, 0);
    }

    #[test]
    fn the_offset_sets_the_dac_and_limits_the_level() {
        let mut s = settings();
        assert_eq!(s.offset_value, 343);
        s.set_offset(3.0);
        // 0.3 V at 6 mV per bit.
        assert_eq!(s.offset_value, 343 - 50);
        assert!((s.offset_adjusted - 3.0).abs() < 1e-9);
        s.set_offset(-1000.0);
        assert_eq!(s.offset_value, 343 + 333);
        s.set_offset(1000.0);
        assert_eq!(s.offset_value, 9);
        s.set_offset(0.0);
        s.set_trigger_level(1000.0);
        assert!((s.trigger_adjusted - 20.0).abs() < 1e-9);
        s.set_trigger_level(-1000.0);
        assert!((s.trigger_adjusted + 20.0).abs() < 1e-9);
        s.set_trigger_level(5.0);
        assert!((s.trigger_adjusted - 5.0).abs() < 1e-9);
        // The level's limits follow an offset.
        s.set_trigger_level(19.0);
        s.set_offset(3.0);
        assert!((s.trigger_adjusted - 17.0).abs() < 1e-9);
        assert!((s.trigger_level - 19.0).abs() < 1e-12);
    }

    #[test]
    fn the_offset_at_its_limits_with_no_number() {
        let mut s = settings();
        s.set_offset(f64::NAN);
        assert_eq!(s.offset_value, 0);
        s.set_offset(f64::INFINITY);
        assert_eq!(s.offset_value, 9);
    }

    #[test]
    fn the_threshold_divides_the_voltage() {
        let mut s = settings();
        assert_eq!(s.trigger_threshold(), 0x200);
        // Under a whole volt of the probe's scale, which C makes 0 V.
        s.set_trigger_level(9.0);
        assert_eq!(s.trigger_threshold(), 0x200 - 209);
        s.set_trigger_level(15.0);
        assert_eq!(s.trigger_threshold(), 0x200 - 349);
        s.set_trigger_level(-15.0);
        assert_eq!(s.trigger_threshold(), 0x200 + 349);
        // On a whole volt it is C's.
        s.set_trigger_level(10.0);
        assert_eq!(s.trigger_threshold(), 0x200 - 233);
    }

    #[test]
    fn the_trigger_register() {
        let mut s = settings();
        assert_eq!(s.trigger_register(0x200), 0x02);
        assert_eq!(s.trigger_register(0x1ff), 0x01);
        s.dso_slope = 1;
        assert_eq!(s.trigger_register(0x200), 0x06);
        s.trigger_source = TriggerSource::La;
        assert_eq!(s.trigger_register(0x200), 0xe6);
        s.la_slope = 1;
        assert_eq!(s.trigger_register(0x200), 0xe2);
    }

    #[test]
    fn the_rate_table_has_the_slow_mode_for_a_bank_select() {
        let mut s = settings();
        assert_eq!(rate_words(10_000), Some(([word(9, 0x9f), word(10, 0x0e)], 0)));
        assert_eq!(s.bank_select(2), word(15, 2));
        assert_eq!(rate_words(100), Some(([word(9, 0x9f), word(10, 0x0f)], 0x20)));
        s.set_slow_mode(0x20);
        assert_eq!(s.bank_select(0), word(15, 0x20));
        assert_eq!(rate_words(3_000_000), None);
        assert_eq!(rate_words((1 << 32) + 100), None);
    }

    #[test]
    fn a_dump_decodes_as_both_programs_read_it() {
        let s = settings();
        // Analog 0b10_1100_1101 with both high bits set; logic 0b1011_0110.
        let (logic, analog) = s.decode(&[0xcd, 0xeb, 0xed]);
        assert_eq!(logic, [0b1011_0110]);
        let raw = 0b10_1100_1101;
        let want = (512.0 - f64::from(raw)) * 0.004_287_4 * 10.0;
        assert!((f64::from(analog[0]) - want).abs() < 1e-5);
    }

    #[test]
    fn a_sample_is_less_the_offset_the_dac_holds() {
        let mut s = settings();
        s.set_offset(3.0);
        let (_, analog) = s.decode(&[0x00, 0x00, 0x00]);
        let want = 512.0 * 0.004_287_4 * 10.0 - 3.0;
        assert!((f64::from(analog[0]) - want).abs() < 1e-4);
    }

    #[test]
    fn the_coupling_bit_goes_with_the_setting() {
        let mut s = settings();
        s.apply_coupling();
        assert_eq!(s.reset_fsm_words(), [word(14, 0x91)]);
        s.coupling = 1;
        s.apply_coupling();
        assert_eq!(s.reset_fsm_words(), [word(14, 0x11)]);
    }

    #[test]
    fn the_words_of_a_start_hold_the_settings() {
        let mut s = settings();
        s.apply_coupling();
        assert_eq!(s.dac_offset_words(), [word(12, 0x01), word(13, 0x57), word(14, 0xb0)],);
        assert_eq!(s.dac_threshold_words(), [word(12, 0x8f), word(13, 0xff), word(14, 0xb0)],);
        s.logic_threshold = 0;
        assert_eq!(s.dac_threshold_words()[0], word(12, 0x86));
        assert_eq!(s.arm_words(), [word(14, 0x91), word(14, 0x92), word(14, 0x90)]);
        s.la_trigger = 0x81;
        s.la_trigger_mask = 0x7e;
        let t = s.trigger_words();
        assert_eq!(t[2], word(5, 0x81));
        assert_eq!(t[3], word(6, 0x7e));
        assert_eq!(t[4], word(7, 0));
        assert_eq!(t[5], word(8, 2));
        assert_eq!(t[7], word(15, 2));
        assert_eq!(t[17], word(15, 0));
    }
}
