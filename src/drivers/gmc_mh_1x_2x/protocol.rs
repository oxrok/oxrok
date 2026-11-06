// Counterpart in sigrok:
//   libsigrok/src/hardware/gmc-mh-1x-2x/protocol.c,
//   libsigrok/src/hardware/gmc-mh-1x-2x/protocol.h,
//   Copyright (C) 2013, 2014 Matthias Heidbrink <m-sigrok@heidbrink.biz>,
//   with commits by Uwe Hermann, Martin Ling, Bert Vermeulen, Daniel Elstner,
//   Aurelien Jacobs, Gerhard Sittig, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `METRAHit`'s bytes.
//!
//! In send mode the meter sends blocks of six bit bytes. Bits 5 and 4 of a byte
//! say what it is: 00 starts a block of settings, 01 and 10 a block of readings,
//! 11 is the rest of a block; the low four bits hold its contents.
//!
//! - 5 bytes (settings: device code, function, two special characters, range and
//!   sign), which end when the next start comes, then 6 (range and sign, five
//!   digits lowest first);
//! - 10 bytes, a settings block with five digits, on a 15S to 18S;
//! - 13 bytes, a settings block with six digits, the function's high bits and the
//!   send interval, on a 2x.
//!
//! In bidirectional mode each request is 14 six bit bytes (address byte, `2b 3f`,
//! command, nine parameters, checksum), each going out as three bytes, and the
//! answer 14 bytes (address, `27 3f`, command, nine bytes, checksum), of which
//! bits 7 and 6 mean nothing.
//!
//! Gossen-Metrawatt's "Interface Protocol for `METRAHit` 22-29S/M Multimeters"
//! (2000-10-02), read from a saved text extract: page 1's figure and the Ohm symbols
//! are unread. Against it and C:
//!
//! - D3 of a range byte is the sign of the value, 0 being plus ("a=1 sign -", and
//!   command 8's "signum of the value, 0 = +"). That is how C had it until
//!   `f4bd8a17` (2016-08-24, "properly set encoding digits"): `scale` was a factor
//!   from 1.0 and `scale *= -1.0` negated the value. That commit made it an exponent
//!   from 0 and the line became `scale = -scale`, so a negative value shows as
//!   positive, and wherever the exponent is not 0 yet it changes sign: a positive
//!   capacitance reads 100 times high, a continuity voltage and a power too, and a
//!   current by command 8 reads 300 uA as 3e8 A. Here the bit negates the value.
//!   Pre-2016 C is the witness that does not share this reading: the model's encoder
//!   does. The 12S to 16S have the same regression on their bit 2, which C's author
//!   moved there in `1d4a2839` ("Fixed sign and AC/DC ... Metrahit <= 16").
//! - With the exponent left alone V, A, mA, ohm, Hz and F come out at their ranges'
//!   full scale in the document's tables. Not everything does, and those stay as C
//!   has them, unverified: power ignores its range, where the table has 1 mW to
//!   10 kW; `TRMS V on mains` at range 0 reads 0.3 V where it says 300 V (`5791bdf6`
//!   removed a mains scale on purpose, and C keeps a TODO there).
//! - 1101 in a digit is a broken fuse. C logs it and adds it in as 13.
//! - 1010 is OL in the document's digit table. It has a 2x in `SI-232 StorE` send
//!   10 bytes under the 18S's code, where C adds the digit in as 10 (`protocol.c:
//!   756-758`). So here it is OL for that code. For 15S and 16S, which it doesn't
//!   cover, as C.
//! - A digit is D3-D0, so one above 0x0f is refused. It catches a window a lost byte
//!   made, whose sum still holds when that byte equals the address. That assumes
//!   meters send those bits clear, which no recording confirms; C masks them.

use crate::core::{Mq, MqFlags, Unit};

/// A request on the wire.
pub const WIRE_BYTES: usize = 42;
/// An answer, and a request before it goes on the wire.
pub const REPLY_BYTES: usize = 14;

/// Bits 5 and 4 of a send mode byte.
const MSGID_MASK: u8 = 0x30;
const MSGID_INF: u8 = 0x00;
const MSGID_D10: u8 = 0x10;
const MSGID_DTA: u8 = 0x20;
/// A send mode byte's contents.
const MSGC_MASK: u8 = 0x0f;
/// The six bits of a byte that mean anything.
pub const MASK_6BITS: u8 = 0x3f;
/// The send mode buffer in C. A block ends at 13, so this is never reached by a
/// stream, and it bounds this one's as well.
const SEND_BUFFER: usize = 266;

/// `enum model`: the order is C's, as it compares them as ranges.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Model {
    /// None yet, or a code no table has.
    None,
    /// 12S.
    M12S,
    /// 13S and 14A.
    M13S14A,
    /// 14S.
    M14S,
    /// 15S.
    M15S,
    /// 16S.
    M16S,
    /// 16I and 16L.
    M16I,
    /// 16T, 16U and KMM2002.
    M16T,
    /// 18S.
    M18S,
    /// 22S or 22M in send mode.
    M22SM,
    /// 22S.
    M22S,
    /// 22M.
    M22M,
    /// 23S.
    M23S,
    /// 24S.
    M24S,
    /// 25S.
    M25S,
    /// 26S or 26M in send mode.
    M26SM,
    /// 26S.
    M26S,
    /// 26M.
    M26M,
    /// 28S.
    M28S,
    /// 29S.
    M29S,
}

impl Model {
    /// `gmc_model_str`.
    pub fn name(self) -> &'static str {
        match self {
            Model::None => "-uninitialized model variable-",
            Model::M12S => "METRAHit 12S",
            Model::M13S14A => "METRAHit 13S/14A",
            Model::M14S => "METRAHit 14S",
            Model::M15S => "METRAHit 15S",
            Model::M16S => "METRAHit 16S",
            Model::M16I => "METRAHit 16I/16L",
            Model::M16T => "METRAHit 16T/16U/KMM2002",
            Model::M18S => "METRAHit 18S",
            Model::M22SM => "METRAHit 22S/M",
            Model::M22S => "METRAHit 22S",
            Model::M22M => "METRAHit 22M",
            Model::M23S => "METRAHit 23S",
            Model::M24S => "METRAHit 24S",
            Model::M25S => "METRAHit 25S",
            Model::M26SM => "METRAHit 26S/M",
            Model::M26S => "METRAHit 26S",
            Model::M26M => "METRAHit 26M",
            Model::M28S => "METRAHit 28S",
            Model::M29S => "METRAHit 29S",
        }
    }
}

/// `gmc_decode_model_sm`, of the four bits it is always given.
pub fn decode_model_sm(mcode: u8) -> Model {
    match mcode {
        0x04 => Model::M12S,
        0x08 => Model::M13S14A,
        0x09 => Model::M14S,
        0x0a => Model::M15S,
        0x0b => Model::M16S,
        0x06 => Model::M16I,
        0x07 => Model::M16T,
        0x0d => Model::M18S,
        0x02 => Model::M22SM,
        0x03 => Model::M23S,
        0x0f => Model::M24S,
        0x05 => Model::M25S,
        0x01 => Model::M26SM,
        0x0c => Model::M28S,
        0x0e => Model::M29S,
        _ => {
            log::error!("Unknown model code {mcode}!");
            Model::None
        }
    }
}

/// `gmc_decode_model_bd`: D4-D0 the type, D5 "with memory".
pub fn decode_model_bd(mcode: u8) -> Model {
    let memory = mcode & 0x20 != 0;
    match mcode & 0x1f {
        2 if memory => Model::M22M,
        2 => Model::M22S,
        3 => Model::M23S,
        4 => Model::M24S,
        5 => Model::M25S,
        1 if memory => Model::M26M,
        1 => Model::M26S,
        12 => Model::M28S,
        14 => Model::M29S,
        _ => {
            log::error!("Unknown model code {mcode}!");
            Model::None
        }
    }
}

/// `calc_chksum_14`: the complement of the sum of the first 13 bytes.
pub fn checksum(block: &[u8]) -> u8 {
    let sum = block[..REPLY_BYTES - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    64u8.wrapping_sub(sum) & MASK_6BITS
}

/// `create_cmd_14`: a request, three bytes on the wire for each one.
pub fn create_cmd(addr: u8, func: u8, params: &[u8; 9]) -> [u8; WIRE_BYTES] {
    let mut block = [0u8; REPLY_BYTES];
    block[0] = addr.wrapping_shl(2) & MASK_6BITS | 0x03;
    block[1] = 0x2b;
    block[2] = 0x3f;
    block[3] = func;
    for (out, p) in block[4..13].iter_mut().zip(params) {
        *out = p & MASK_6BITS;
    }
    block[13] = checksum(&block);
    let two = |b: u8, low: u8, high: u8| {
        (if b & low != 0 { 0x0f } else { 0 }) | (if b & high != 0 { 0xf0 } else { 0 })
    };
    let mut wire = [0u8; WIRE_BYTES];
    for (out, b) in wire.as_chunks_mut::<3>().0.iter_mut().zip(block) {
        out[0] = two(b, 0x01, 0x02);
        out[1] = two(b, 0x04, 0x08);
        out[2] = two(b, 0x10, 0x20);
    }
    wire
}

/// The request for one measurement (command 8).
pub fn measure_request(addr: u8) -> [u8; WIRE_BYTES] {
    create_cmd(addr, 8, &[0; 9])
}

/// The request for version and status (command 3).
pub fn status_request(addr: u8) -> [u8; WIRE_BYTES] {
    create_cmd(addr, 3, &[0; 9])
}

/// The command that switches the meter off (command 6, 5 and 5).
pub fn power_off_request(addr: u8) -> [u8; WIRE_BYTES] {
    create_cmd(addr, 6, &[5, 5, 0, 0, 0, 0, 0, 0, 0])
}

/// What goes out for a reading: all of `send_value` that can vary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// The value.
    pub value: f32,
    /// What was measured. `None` for a function no table has, or none at all.
    pub mq: Option<Mq>,
    /// Its unit, likewise.
    pub unit: Option<Unit>,
    /// Its flags.
    pub flags: MqFlags,
    /// `-(scale + 3 * scale1000)`.
    pub digits: i32,
}

/// `devc`'s reading: what the tables make of the bytes.
#[derive(Clone, Debug)]
pub struct Decoder {
    /// `devc->model`.
    pub model: Model,
    /// The meter's address, from its answer to the scan.
    pub addr: u8,
    /// Firmware version, from a status.
    pub firmware: (u8, u8),
    autorange: bool,
    mq: Option<Mq>,
    unit: Option<Unit>,
    flags: MqFlags,
    value: f32,
    scale: i32,
    scale1000: i32,
    /// The range byte's sign bit. See the module's documentation.
    negative: bool,
}

/// `pow(10.0, n)` as an f64, the way C's arithmetic takes it.
fn pow10(n: i32) -> f64 {
    10f64.powf(f64::from(n))
}

impl Decoder {
    /// A decoder that knows nothing yet.
    pub fn new(model: Model) -> Decoder {
        Decoder {
            model,
            addr: 0,
            firmware: (0, 0),
            autorange: false,
            mq: None,
            unit: None,
            flags: MqFlags::empty(),
            value: 0.0,
            scale: 0,
            scale1000: 0,
            negative: false,
        }
    }

    fn set_flag(&mut self, flag: MqFlags, set: bool) {
        let others =
            self.flags.iter().filter(|f| *f != flag).fold(MqFlags::empty(), |all, f| all | f);
        self.flags = if set { others | flag } else { others };
    }

    fn quantity(&mut self, mq: Mq, unit: Unit) {
        self.mq = Some(mq);
        self.unit = Some(unit);
    }

    fn clean_function(&mut self) {
        self.mq = None;
        self.unit = None;
        self.flags = MqFlags::empty();
    }

    fn clean_range(&mut self) {
        self.value = 0.0;
        self.scale = 0;
        self.negative = false;
    }

    fn clean_all(&mut self) {
        self.clean_function();
        self.scale1000 = 0;
        self.clean_range();
    }

    /// `decode_ctmv_16`.
    fn function_16(&mut self, code: u8) {
        self.clean_function();
        match code {
            0x01..=0x04 => {
                if code == 0x01 {
                    self.scale1000 = -1;
                }
                self.quantity(Mq::Voltage, Unit::Volt);
                if code <= 0x03 {
                    self.flags |= MqFlags::DC;
                }
                if code >= 0x03 {
                    self.flags |= MqFlags::AC;
                    if self.model >= Model::M16S {
                        self.flags |= MqFlags::RMS;
                    }
                }
            }
            0x05 | 0x06 => {
                self.quantity(Mq::Frequency, Unit::Hertz);
                if code == 0x06 {
                    self.scale1000 = 1;
                }
            }
            0x07 => self.quantity(Mq::DutyCycle, Unit::Percentage),
            0x08 => {
                self.quantity(Mq::Voltage, Unit::Volt);
                self.flags |= MqFlags::DIODE | MqFlags::DC;
            }
            0x09..=0x0b => {
                self.quantity(Mq::Resistance, Unit::Ohm);
                self.scale1000 = i32::from(code) - 0x09;
            }
            0x0c | 0x0d => {
                self.quantity(Mq::Capacitance, Unit::Farad);
                self.scale1000 = if code == 0x0c { -3 } else { -2 };
            }
            0x0e | 0x0f => {
                if code == 0x0e {
                    self.scale1000 = -1;
                }
                self.quantity(Mq::Current, Unit::Ampere);
                if self.model == Model::M16S {
                    self.flags |= MqFlags::RMS;
                }
            }
            _ => {}
        }
    }

    /// `decode_rs_16`. `len` is the bytes of the block so far: a 10 byte one
    /// means degrees where a shorter one means an empty range.
    fn range_16(&mut self, rs: u8, len: usize) {
        self.negative = rs & 0x04 != 0;
        if self.mq == Some(Mq::Current) {
            self.flags |= if rs & 0x08 != 0 { MqFlags::AC } else { MqFlags::DC };
        }
        match rs & 0x03 {
            0 => match self.mq {
                Some(Mq::Voltage) => self.scale -= 1,
                Some(Mq::Current) => self.scale -= 5,
                Some(Mq::Resistance) => {
                    if len >= 10 {
                        self.quantity(Mq::Temperature, Unit::Celsius);
                        self.scale -= 2;
                    } else if self.scale1000 == 2 {
                        self.scale -= 1;
                    }
                }
                _ => {}
            },
            1 => self.scale -= 4,
            2 => self.scale -= 3,
            _ => self.scale -= 2,
        }
    }

    /// `decode_spc_16`.
    fn special_16(&mut self, spc: u8) {
        self.set_flag(MqFlags::MIN, spc & 0x80 != 0);
        self.set_flag(MqFlags::AUTORANGE, spc & 0x40 == 0);
        self.set_flag(MqFlags::HOLD, spc & 0x20 != 0);
        self.set_flag(MqFlags::MAX, spc & 0x10 != 0);
    }

    /// `decode_ctmv_18`.
    fn function_18(&mut self, code: u8) {
        self.clean_function();
        match code {
            0x01..=0x03 => {
                self.quantity(Mq::Voltage, Unit::Volt);
                if code <= 0x02 {
                    self.flags |= MqFlags::AC | MqFlags::RMS;
                }
                if code >= 0x02 {
                    self.flags |= MqFlags::DC;
                }
            }
            0x04 => self.quantity(Mq::Resistance, Unit::Ohm),
            0x05 => {
                self.quantity(Mq::Voltage, Unit::Volt);
                self.flags |= MqFlags::DIODE | MqFlags::DC;
            }
            0x06 => self.quantity(Mq::Temperature, Unit::Celsius),
            0x07 => self.quantity(Mq::Capacitance, Unit::Farad),
            0x08..=0x0b => {
                self.quantity(Mq::Current, Unit::Ampere);
                self.flags |= MqFlags::DC;
                if code >= 0x0a {
                    self.flags |= MqFlags::AC | MqFlags::RMS;
                }
                if code == 0x08 || code == 0x0a {
                    self.scale1000 = -1;
                }
            }
            0x0c => self.quantity(Mq::Frequency, Unit::Hertz),
            0x0d => {
                self.quantity(Mq::Voltage, Unit::DecibelVolt);
                self.flags |= MqFlags::AC;
            }
            0x0e => {
                self.quantity(Mq::Voltage, Unit::Volt);
                self.flags |= MqFlags::AC | MqFlags::DC | MqFlags::RMS;
            }
            0x0f => {
                self.quantity(Mq::Time, Unit::Second);
                self.flags |= MqFlags::DURATION;
            }
            _ => {}
        }
    }

    /// `decode_rs_18`, its sign bit on the value.
    fn range_18(&mut self, rs: u8) {
        self.negative = rs & 0x08 != 0;
        let range = i32::from(rs & 0x07);
        match self.mq {
            Some(Mq::Voltage) if self.unit == Some(Unit::DecibelVolt) => self.scale -= 2,
            Some(Mq::Voltage) => self.scale += range - 5,
            Some(Mq::Current) => {
                self.scale += if self.scale1000 == -1 { range - 5 } else { range - 4 };
            }
            Some(Mq::Resistance | Mq::Frequency | Mq::Temperature) => self.scale += range - 2,
            Some(Mq::Capacitance) => self.scale += range - 13,
            _ => {}
        }
    }

    /// `decode_spc_18`.
    fn special_18(&mut self, spc: u8) {
        if self.mq == Some(Mq::Time) {
            log::trace!("Clock running: {}", spc >> 4);
            return;
        }
        self.set_flag(MqFlags::AUTORANGE, spc & 0x80 == 0);
        self.set_flag(MqFlags::MIN, spc & 0x40 != 0);
        self.set_flag(MqFlags::MAX, spc & 0x20 != 0);
        self.set_flag(MqFlags::HOLD, spc & 0x10 != 0);
    }

    /// `decode_ctmv_2x`.
    fn function_2x(&mut self, code: u8) {
        if code > 0x20 {
            log::error!("decode_ctmv_2x({code:#x}): invalid param(s)!");
            return;
        }
        self.clean_function();
        let rms_model = self.model >= Model::M24S;
        match code {
            0x00 | 0x13 | 0x14 | 0x1e => log::error!("decode_ctmv_2x({code}, ...): Unknown ctmv!"),
            0x01..=0x03 => {
                self.quantity(Mq::Voltage, Unit::Volt);
                if code <= 0x02 {
                    self.flags |= MqFlags::DC;
                }
                if code >= 0x02 {
                    self.flags |= MqFlags::AC;
                    if rms_model {
                        self.flags |= MqFlags::RMS;
                    }
                }
            }
            0x04..=0x07 => {
                if code <= 0x05 {
                    self.scale1000 = -1;
                }
                self.quantity(Mq::Current, Unit::Ampere);
                self.flags |= MqFlags::DC;
                if code == 0x05 || code == 0x07 {
                    self.flags |= MqFlags::AC;
                    if rms_model {
                        self.flags |= MqFlags::RMS;
                    }
                }
            }
            0x08 => self.quantity(Mq::Resistance, Unit::Ohm),
            0x09 => {
                self.quantity(Mq::Capacitance, Unit::Farad);
                self.scale -= 1;
            }
            0x0a => {
                self.quantity(Mq::Voltage, Unit::DecibelVolt);
                self.flags |= MqFlags::AC;
                if rms_model {
                    self.flags |= MqFlags::RMS;
                }
            }
            0x0b | 0x0c => {
                self.quantity(Mq::Frequency, Unit::Hertz);
                self.flags |= MqFlags::AC;
                if code <= 0x0b {
                    self.flags |= MqFlags::DC;
                }
            }
            0x0d | 0x0e => {
                self.scale -= if code == 0x0d { 2 } else { 1 };
                self.scale1000 = -1;
                self.quantity(Mq::Power, Unit::Watt);
                self.flags |= MqFlags::AC | MqFlags::DC | MqFlags::RMS;
            }
            0x0f => {
                self.quantity(Mq::Voltage, Unit::Volt);
                self.flags |= MqFlags::DIODE | MqFlags::DC;
            }
            0x10 => {
                self.quantity(Mq::Continuity, Unit::Volt);
                self.scale -= 5;
            }
            0x11 => {
                self.quantity(Mq::Continuity, Unit::Ohm);
                self.scale1000 = -1;
            }
            0x12 => self.quantity(Mq::Temperature, Unit::Celsius),
            0x15 => self.quantity(Mq::Gain, Unit::Percentage),
            0x16 => self.quantity(Mq::Power, Unit::Watt),
            0x17 | 0x1d => {
                self.quantity(Mq::Voltage, Unit::Volt);
                self.flags |= MqFlags::AC | MqFlags::DC | MqFlags::RMS;
            }
            0x18 | 0x20 => self.quantity(Mq::Voltage, Unit::Unitless),
            0x19 | 0x1a => {
                self.quantity(Mq::Voltage, Unit::Unitless);
                self.flags |= MqFlags::AC;
                if code <= 0x19 {
                    self.flags |= MqFlags::DC;
                }
            }
            0x1b => {
                self.quantity(Mq::Current, Unit::Ampere);
                self.flags |= MqFlags::AC | MqFlags::DC | MqFlags::RMS;
                self.scale1000 = -1;
            }
            0x1c => {
                self.quantity(Mq::Voltage, Unit::Unitless);
                self.flags |= MqFlags::AC;
            }
            _ => {
                self.quantity(Mq::Voltage, Unit::Volt);
                self.flags |= MqFlags::DC;
            }
        }
    }

    /// `decode_rs_2x`, table TR. The sign is the value's, where C negates `scale`.
    fn range_2x(&mut self, rs: u8) {
        self.negative = rs & 0x08 != 0;
        let range = i32::from(rs & 0x07);
        match self.mq {
            Some(Mq::Voltage) => {
                self.scale += if self.unit == Some(Unit::DecibelVolt) { -3 } else { range - 6 };
            }
            Some(Mq::Current) => {
                // The mA and A ranges differ by 10^4.
                let range = if self.scale1000 == -1 { range } else { range + 1 };
                self.scale += range - 6;
            }
            Some(Mq::Resistance | Mq::Frequency) => self.scale += range - 3,
            Some(Mq::Temperature) => {
                if range == 4 {
                    self.unit = Some(Unit::Fahrenheit);
                }
                self.scale -= 2;
            }
            Some(Mq::Capacitance) => {
                let range = if range == 7 { 6 } else { range };
                self.scale += range - 13;
            }
            _ => {}
        }
    }

    /// `decode_rs_2x_TR2`, table TR 2, which differs in a current's ranges.
    fn range_2x_tr2(&mut self, rs: u8) {
        if self.mq != Some(Mq::Current) {
            self.range_2x(rs);
            return;
        }
        self.negative = rs & 0x08 != 0;
        let range = rs & 0x07;
        if self.scale1000 == -1 {
            self.scale += match range {
                0 | 1 => -6,
                2 | 3 => -5,
                4 | 5 => -4,
                _ => -3,
            };
        } else {
            self.scale += match range {
                0 | 1 => -5,
                2 => -4,
                _ => 0,
            };
        }
    }

    /// `decode_spc_2x`.
    fn special_2x(&mut self, spc: u8) {
        self.set_flag(MqFlags::HOLD, spc & 0x10 != 0);
        self.set_flag(MqFlags::AUTORANGE, spc & 0x80 == 0);
    }

    fn overload(&mut self) {
        self.value = f32::NAN;
        self.scale = 0;
    }

    fn add_digit(&mut self, position: i32, digit: u8) {
        self.value = (f64::from(self.value) + pow10(position) * f64::from(digit)) as f32;
    }

    /// The end of each block in C: `value *= pow(10, scale) * pow(1000, scale1000)`,
    /// and `send_value`.
    fn finish(&mut self) -> Sample {
        let factor = pow10(self.scale) * 1000f64.powf(f64::from(self.scale1000));
        self.value = (f64::from(self.value) * factor) as f32;
        if self.negative && self.value != 0.0 && !self.value.is_nan() {
            self.value = -self.value;
        }
        Sample {
            value: self.value,
            mq: self.mq,
            unit: self.unit,
            flags: self.flags,
            digits: -(self.scale + 3 * self.scale1000),
        }
    }

    /// `process_msg_dta_6`.
    pub fn block_6(&mut self, buf: &[u8]) -> Sample {
        self.clean_range();
        let rs = buf[0] & MSGC_MASK;
        if self.model <= Model::M16T {
            self.range_16(rs, buf.len());
        } else if self.model < Model::M22SM {
            self.range_18(rs);
        } else {
            self.range_2x(rs);
            // Five digits here, six in what the table assumes.
            self.scale += 1;
        }
        for (position, b) in (0..).zip(&buf[1..6]) {
            let digit = b & MSGC_MASK;
            if digit >= 10 {
                self.overload();
                break;
            }
            self.add_digit(position, digit);
        }
        self.finish()
    }

    fn warn_mismatch(&self, code: u8) {
        let model = decode_model_sm(code);
        if model != self.model {
            log::warn!(
                "Model mismatch in data: Detected {}, now {}",
                self.model.name(),
                model.name()
            );
        }
    }

    /// `process_msg_inf_5`. `len` is how long the block was when it ended.
    pub fn block_5(&mut self, buf: &[u8], len: usize) {
        self.clean_all();
        self.warn_mismatch(buf[0] & MSGC_MASK);
        let spc = (buf[2] & MSGC_MASK) | ((buf[3] & MSGC_MASK) << 4);
        let (code, rs) = (buf[1] & MSGC_MASK, buf[4] & MSGC_MASK);
        if self.model <= Model::M16T {
            self.function_16(code);
            self.special_16(spc);
            self.range_16(rs, len);
        } else if self.model <= Model::M18S {
            self.function_18(code);
            self.special_18(spc);
            self.range_18(rs);
        } else {
            self.function_2x(code);
            self.special_2x(spc);
            self.range_2x(rs);
        }
    }

    /// `process_msg_inf_10`.
    pub fn block_10(&mut self, buf: &[u8]) -> Sample {
        self.block_5(buf, buf.len());
        for (position, b) in (0..).zip(&buf[5..10]) {
            let digit = match b & MSGC_MASK {
                11 => 0,
                10 if self.model == Model::M18S => {
                    self.overload();
                    break;
                }
                d if d >= 12 => {
                    self.overload();
                    break;
                }
                d => d,
            };
            self.add_digit(position, digit);
        }
        self.finish()
    }

    /// `process_msg_inf_13`.
    pub fn block_13(&mut self, buf: &[u8]) -> Sample {
        self.clean_all();
        self.warn_mismatch(buf[0] & MSGC_MASK);
        self.function_2x((buf[1] & MSGC_MASK) | ((buf[11] & MSGC_MASK) << 4));
        self.special_2x((buf[2] & MSGC_MASK) | ((buf[3] & MSGC_MASK) << 4));
        self.range_2x(buf[4] & MSGC_MASK);
        for (position, b) in (0..).zip(&buf[5..11]) {
            let digit = b & MSGC_MASK;
            if digit == 10 {
                self.overload();
                break;
            }
            self.add_digit(position, digit);
        }
        log::trace!("Send interval code: {}", buf[12] & MSGC_MASK);
        self.finish()
    }

    /// `chk_msg14`: logs what it finds, all of it. Takes the six bit bytes.
    fn check_reply(&self, buf: &[u8; REPLY_BYTES]) -> bool {
        let mut good = true;
        let request = buf[1] == 0x2b;
        let addr = if request { buf[0] >> 2 } else { buf[0] & 0x0f };
        if self.addr != addr && !(request && addr == 0) {
            log::error!("process_msg_14(): Address mismatch, msg for other device!");
            good = false;
        }
        if buf[1] == 0 {
            log::error!(
                "{}",
                match buf[2] {
                    1 => "Device: Illegal error code!",
                    2 => "Device: Incorrect checksum in cmd!",
                    3 => "Device: Incorrect block length in cmd!",
                    4 => "Device: Incorrect byte 2 or 3 in cmd!",
                    5 => "Device: Parameter out of range!",
                    _ => "Device: Unknown error code!",
                }
            );
            good = false;
        } else if !request && (buf[1] != 0x27 || buf[2] != 0x3f) {
            log::error!("process_msg_14(): byte 1/2 unexpected!");
            good = false;
        }
        if checksum(buf) != buf[13] {
            log::error!("process_msg_14(): Invalid checksum!");
            good = false;
        }
        if buf[3] == 8 && buf[7..13].iter().any(|digit| *digit > MSGC_MASK) {
            log::error!("A digit above 0x0f: not an answer.");
            good = false;
        }
        if !good {
            log::trace!("msg14: {buf:02x?}");
        }
        good
    }

    /// `process_msg14`: the answer's sample, if it is one.
    pub fn reply(&mut self, buf: &[u8; REPLY_BYTES]) -> Reply {
        if !self.check_reply(buf) {
            return Reply::Bad;
        }
        self.clean_all();
        let addr = buf[0] & MASK_6BITS;
        if addr != self.addr {
            log::info!("Device address mismatch {addr}/{}!", self.addr);
        }
        match buf[3] {
            3 => {
                self.firmware = (buf[5], buf[4]);
                log::trace!("Firmware version {}.{}", buf[5], buf[4]);
                self.function_2x(buf[7]);
                self.range_2x_tr2(buf[8] & 0x0f);
                self.autorange = buf[8] & 0x20 == 0;
                self.model = decode_model_bd(buf[12]);
                log::trace!("Model={}", self.model.name());
                Reply::Good(None)
            }
            8 => {
                self.function_2x(buf[5]);
                if buf[6] & 0x10 == 0 {
                    // Old data.
                    return Reply::Good(None);
                }
                self.range_2x_tr2(buf[6] & 0x0f);
                self.set_flag(MqFlags::AUTORANGE, self.autorange);
                for (position, b) in (0..).zip(&buf[7..13]) {
                    match b & MSGC_MASK {
                        10 => {
                            self.overload();
                            break;
                        }
                        13 => {
                            log::error!("FUSE!");
                            self.overload();
                            break;
                        }
                        14 => {
                            log::info!("Function recognition mode, OPEN!");
                            self.overload();
                            break;
                        }
                        digit => self.add_digit(position, digit),
                    }
                }
                Reply::Good(Some(self.finish()))
            }
            other => {
                log::trace!("Cmd {other} unimplemented!");
                Reply::Good(None)
            }
        }
    }
}

/// What came of an answer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reply {
    /// It failed the checks.
    Bad,
    /// It is an answer, with a sample if it holds one.
    Good(Option<Sample>),
}

/// Where the next answer starts in a block that failed its checks, whose bytes
/// are out of step when its header is not where it belongs. C drops all 14, so
/// after a lost byte every block from there on straddles two answers. `None`
/// if it was in step, or as good as: then the whole block goes.
pub fn resync(buf: &[u8; REPLY_BYTES], addr: u8) -> Option<usize> {
    // An answer's bytes add up to 0 whatever they say, so a tail and a head can
    // too: an error block is also from this meter.
    let error_block = buf[0] == addr && buf[1] == 0 && checksum(buf) == buf[13];
    if error_block || buf[1] == 0x2b || buf[1] == 0x27 {
        return None;
    }
    (1..REPLY_BYTES).find(|i| {
        let at = |k: usize| buf.get(i + k).copied();
        at(1).is_none_or(|b| b == 0x27) && at(2).is_none_or(|b| b == 0x3f)
    })
}

/// `gmc_mh_1x_2x_receive_data`'s byte loop: blocks out of a send mode stream.
#[derive(Clone, Debug)]
pub struct SendStream {
    /// The tables.
    pub decoder: Decoder,
    settings_ok: bool,
    buf: Vec<u8>,
}

impl SendStream {
    /// As at a start.
    pub fn new(model: Model) -> SendStream {
        SendStream {
            decoder: Decoder::new(model),
            settings_ok: false,
            buf: Vec::with_capacity(SEND_BUFFER),
        }
    }

    /// One byte, and the sample it completes.
    pub fn byte(&mut self, byte: u8) -> Option<Sample> {
        self.buf.push(byte);
        if !self.settings_ok {
            if self.buf[0] & MSGID_MASK != MSGID_INF {
                self.buf.clear();
                return None;
            }
            self.settings_ok = true;
        }
        let len = self.buf.len();
        match self.buf[0] & MSGID_MASK {
            MSGID_INF => {
                if len == 13 {
                    let sample = self.decoder.block_13(&self.buf);
                    self.buf.clear();
                    return Some(sample);
                }
                if len == 10 && self.decoder.model <= Model::M18S {
                    let sample = self.decoder.block_10(&self.buf);
                    self.buf.clear();
                    return Some(sample);
                }
                if len >= 5 && byte & MSGID_MASK != MSG_REST {
                    // The byte just read starts the next block.
                    self.decoder.block_5(&self.buf, len);
                    self.buf.clear();
                    self.buf.push(byte);
                }
                None
            }
            MSGID_DTA | MSGID_D10 => {
                if len == 6 {
                    let sample = self.decoder.block_6(&self.buf);
                    self.buf.clear();
                    return Some(sample);
                }
                None
            }
            _ => {
                log::error!("Comm error, unexpected data byte!");
                self.buf.clear();
                None
            }
        }
    }
}

/// Bits 5 and 4 of the bytes after a block's first.
pub const MSG_REST: u8 = 0x30;

/// What a scan in send mode makes of a byte: the bits of a start.
pub fn is_start(byte: u8) -> bool {
    byte & MSGID_MASK == MSGID_INF
}

/// Whether a byte is the rest of a block.
pub fn is_rest(byte: u8) -> bool {
    byte & MSGID_MASK == MSG_REST
}

/// A byte's contents.
pub fn contents(byte: u8) -> u8 {
    byte & MSGC_MASK
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 13 byte block: FUNCTION RANGE SPECIAL and digits lowest first.
    fn block_13(code: u8, function: u8, rs: u8, spc: u8, digits: [u8; 6]) -> Vec<u8> {
        let mut b =
            vec![code, 0x30 | (function & 15), 0x30 | (spc & 15), 0x30 | (spc >> 4), 0x30 | rs];
        b.extend(digits.iter().map(|d| 0x30 | d));
        b.push(0x30 | (function >> 4));
        b.push(0x33);
        b
    }

    fn decoder(model: Model) -> Decoder {
        Decoder::new(model)
    }

    /// An answer's 14 bytes to command 8 from address 1.
    fn answer(function: u8, ra: u8, digits: [u8; 6]) -> [u8; REPLY_BYTES] {
        let mut b = [1, 0x27, 0x3f, 8, 0, function, ra, 0, 0, 0, 0, 0, 0, 0];
        b[7..13].copy_from_slice(&digits);
        b[13] = checksum(&b);
        b
    }

    fn sample(d: &mut Decoder, reply: &[u8; REPLY_BYTES]) -> Sample {
        match d.reply(reply) {
            Reply::Good(Some(s)) => s,
            other => panic!("{other:?}"),
        }
    }

    const DIGITS: [u8; 6] = [6, 5, 4, 3, 2, 1];

    #[test]
    fn a_request_goes_out_as_the_document_has_it() {
        // Address 1 is 07H; each two bits are a nibble of 0 or f.
        let wire = measure_request(1);
        assert_eq!(wire[..3], [0xff, 0x0f, 0x00], "07");
        assert_eq!(wire[3..6], [0xff, 0xf0, 0xf0], "2b");
        assert_eq!(wire[6..9], [0xff, 0xff, 0xff], "3f");
        assert_eq!(wire[9..12], [0x00, 0xf0, 0x00], "08");
        // 07 + 2b + 3f + 08 = 79; 64 - 0x79 in six bits is 7.
        assert_eq!(wire[39..42], [0xff, 0x0f, 0x00], "the checksum");
        assert_eq!(power_off_request(0)[12..18], [0x0f, 0x0f, 0x00, 0x0f, 0x0f, 0x00], "5 and 5");
        assert_eq!(status_request(0)[..3], [0xff, 0x00, 0x00], "03 is all meters");
    }

    #[test]
    fn an_address_keeps_six_bits() {
        assert_eq!(create_cmd(63, 3, &[0; 9])[..3], [0xff, 0xff, 0xff], "3f");
    }

    #[test]
    fn a_full_scale_in_each_table_range() {
        // The document's ranges at their full scale, 300000 in the digits.
        let full = [0, 0, 0, 0, 0, 3];
        let cases: [(u8, u8, f32); 8] = [
            (0x01, 0, 0.3),    // 300 mV
            (0x01, 4, 3000.0), // "1 kV" is the next decade's scale
            (0x08, 0, 300.0),  // 300 ohm
            (0x08, 5, 3.0e7),  // 30 Mohm
            (0x0b, 0, 300.0),  // 300 Hz
            (0x09, 0, 3.0e-9), // 3 nF
            (0x09, 5, 3.0e-4), // 300 uF
            (0x09, 7, 3.0e-3), // 3000 uF, as range 6
        ];
        for (function, range, want) in cases {
            let mut d = decoder(Model::M26SM);
            let s = d.block_13(&block_13(1, function, range, 0, full));
            let off = ((s.value - want) / want).abs();
            assert!(off < 1e-5, "function {function:#x} range {range}: {} not {want}", s.value);
        }
    }

    #[test]
    fn a_current_by_table_tr2_reads_its_range_at_either_sign() {
        let full = [0, 0, 0, 0, 0, 3];
        let mut d = decoder(Model::M26S);
        d.addr = 1;
        for (function, range, want) in [
            (0x04, 1, 300e-6), // 300 uA
            (0x04, 3, 3e-3),
            (0x04, 7, 300e-3),
            (0x06, 1, 3.0),
            (0x06, 2, 30.0),
        ] {
            let s = sample(&mut d, &answer(function, 0x10 | range, full));
            assert!(((f64::from(s.value) - want) / want).abs() < 1e-5, "{}", s.value);
            let s = sample(&mut d, &answer(function, 0x18 | range, full));
            assert!(((f64::from(s.value) + want) / want).abs() < 1e-5, "{}", s.value);
        }
    }

    #[test]
    fn the_sign_bit_is_the_values() {
        let mut d = decoder(Model::M26SM);
        let s = d.block_13(&block_13(1, 1, 1 | 8, 0, DIGITS));
        assert_eq!(s.value, -1.23456);
        assert_eq!(s.digits, 5, "and the exponent as without it");
        let s = d.block_13(&block_13(1, 1, 1, 0, DIGITS));
        assert_eq!(s.value, 1.23456);
        let s = d.block_13(&block_13(1, 1, 1 | 8, 0, [0; 6]));
        assert!(s.value == 0.0 && s.value.is_sign_positive(), "no minus zero");
        let mut d = decoder(Model::M18S);
        let s = d.block_10(&[0x0d, 0x33, 0x30, 0x30, 0x38 | 3, 0x35, 0x34, 0x33, 0x32, 0x31]);
        assert_eq!(s.value, -123.45);
    }

    #[test]
    fn a_12_to_16s_sign_is_bit_2() {
        let mut d = decoder(Model::M16S);
        let s = d.block_10(&[0x0b, 0x32, 0x30, 0x30, 0x34 | 2, 0x35, 0x34, 0x33, 0x32, 0x31]);
        assert_eq!(s.value, -12.345);
        let s = d.block_10(&[0x0b, 0x32, 0x30, 0x30, 2, 0x35, 0x34, 0x33, 0x32, 0x31]);
        assert_eq!(s.value, 12.345);
    }

    /// Value and digits of each function, range by range, at 123456 or 12345.
    #[test]
    fn the_exponents_of_every_table() {
        fn close(got: f32, want: f64, what: &str) {
            assert!(
                (f64::from(got) - want).abs() <= want.abs() * 1e-6,
                "{what}: {got}, not {want}"
            );
        }
        let digits = [6, 5, 4, 3, 2, 1];
        // 2x, 13 bytes: function, range, value, digits.
        for (f, range, want, dg) in [
            (0x01, 3, 123.456, 3),
            (0x04, 2, 12.3456e-3, 7),
            (0x06, 2, 123.456, 3),
            (0x08, 2, 12345.6, 1),
            (0x09, 3, 1.23456e-6, 11),
            (0x0a, 5, 123.456, 3),
            (0x0b, 1, 1234.56, 2),
            (0x12, 3, 1234.56, 2),
            (0x0d, 4, 1.23456, 5),
            (0x0e, 4, 12.3456, 4),
            (0x10, 4, 1.23456, 5),
            (0x11, 4, 123.456, 3),
            (0x1b, 4, 1.23456, 5),
        ] {
            let mut d = decoder(Model::M26SM);
            let s = d.block_13(&block_13(1, f, range, 0, digits));
            close(s.value, want, &format!("2x function {f:#x} range {range}"));
            assert_eq!(s.digits, dg, "2x function {f:#x} range {range}");
        }
        let mut d = decoder(Model::M26SM);
        let s = d.block_13(&block_13(1, 0x12, 4, 0, digits));
        assert_eq!(s.unit, Some(Unit::Fahrenheit), "range 4 of a temperature");
        // 18S, 10 bytes.
        let block =
            |f: u8, r: u8| [0x0d, 0x30 | f, 0x30, 0x30, 0x30 | r, 0x35, 0x34, 0x33, 0x32, 0x31];
        for (f, range, want, dg) in [
            (0x03, 3, 123.45, 2),
            (0x04, 3, 123_450.0, -1),
            (0x07, 3, 12345e-10, 10),
            (0x08, 3, 12345e-5, 5),
            (0x09, 3, 1234.5, 1),
            (0x0c, 1, 1234.5, 1),
            (0x0d, 5, 123.45, 2),
            (0x06, 2, 12345.0, 0),
        ] {
            let mut d = decoder(Model::M18S);
            let s = d.block_10(&block(f, range));
            close(s.value, want, &format!("18S function {f:#x} range {range}"));
            assert_eq!(s.digits, dg, "18S function {f:#x} range {range}");
        }
        // 16S, 10 bytes, and what a shorter block makes of the empty range.
        let block =
            |f: u8, r: u8| [0x0b, 0x30 | f, 0x30, 0x30, 0x30 | r, 0x35, 0x34, 0x33, 0x32, 0x31];
        for (f, range, want, dg) in [
            (0x02, 0, 1234.5, 1),
            (0x02, 1, 1.2345, 4),
            (0x02, 2, 12.345, 3),
            (0x02, 3, 123.45, 2),
            (0x01, 3, 123.45e-3, 5),
            (0x06, 3, 123.45e3, -1),
            (0x0a, 3, 123.45e3, -1),
            (0x0b, 3, 123.45e6, -4),
            (0x0c, 3, 123.45e-9, 11),
            (0x0d, 3, 123.45e-6, 8),
            (0x0e, 0, 12345e-8, 8),
            (0x0f, 0, 12345e-5, 5),
            (0x09, 0, 123.45, 2),
        ] {
            let mut d = decoder(Model::M16S);
            let s = d.block_10(&block(f, range));
            close(s.value, want, &format!("16S function {f:#x} range {range}"));
            assert_eq!(s.digits, dg, "16S function {f:#x} range {range}");
        }
        let mut d = decoder(Model::M16S);
        let s = d.block_10(&block(0x09, 0));
        assert_eq!(s.unit, Some(Unit::Celsius), "10 bytes: degrees");
        d.block_5(&[0x0b, 0x39, 0x30, 0x30, 0x30], 6);
        let s = d.block_6(&[0x10, 0x35, 0x34, 0x33, 0x32, 0x31]);
        assert_eq!(s.mq, Some(Mq::Resistance), "shorter: ohms");
        close(s.value, 12345.0, "16S 6 bytes, an empty range");
    }

    #[test]
    fn overload_of_an_18_on_ten_bytes_is_1010_as_the_document_has_it() {
        let block = [0x0d, 0x33, 0x30, 0x30, 0x33, 0x3a, 0x34, 0x33, 0x32, 0x31];
        let mut d = decoder(Model::M18S);
        assert!(d.block_10(&block).value.is_nan());
        let mut block = block;
        block[0] = 0x0b;
        let mut d = decoder(Model::M16S);
        assert!(!d.block_10(&block).value.is_nan(), "a 16S's, as C");
    }

    #[test]
    fn a_digit_above_its_four_bits_is_no_answer() {
        let mut d = decoder(Model::M26S);
        d.addr = 1;
        let mut b = answer(1, 0x11, DIGITS);
        b[12] = 0x21;
        b[13] = checksum(&b);
        assert_eq!(d.reply(&b), Reply::Bad);
    }

    #[test]
    fn overload_fuse_and_open_read_nan_and_old_data_nothing() {
        let mut d = decoder(Model::M26S);
        d.addr = 1;
        for digit in [10, 13, 14] {
            let s = sample(&mut d, &answer(1, 0x11, [digit, 5, 4, 3, 2, 1]));
            assert!(s.value.is_nan(), "digit {digit}");
            assert_eq!(s.digits, 0);
        }
        assert_eq!(d.reply(&answer(1, 0x01, DIGITS)), Reply::Good(None));
        // A reserved digit counts, as in C.
        assert_eq!(sample(&mut d, &answer(1, 0x11, [11, 0, 0, 0, 0, 0])).value, 11e-5);
        // Overload on a negative is still no number with a sign.
        assert!(sample(&mut d, &answer(1, 0x19, [10, 0, 0, 0, 0, 0])).value.is_sign_positive());
    }

    #[test]
    fn a_status_sets_the_model_the_firmware_and_the_range_mode() {
        let mut d = decoder(Model::None);
        d.addr = 1;
        let mut b = [1, 0x27, 0x3f, 3, 2, 1, 3, 1, 0x21, 0, 0, 30, 0x22, 0];
        b[13] = checksum(&b);
        assert_eq!(d.reply(&b), Reply::Good(None));
        assert_eq!(d.model, Model::M22M);
        assert_eq!(d.firmware, (1, 2));
        let s = sample(&mut d, &answer(2, 0x11, DIGITS));
        assert!(!s.flags.contains(MqFlags::AUTORANGE), "MAN");
        assert!(!s.flags.contains(MqFlags::RMS), "a 22 has none");
        b[8] = 0x01;
        b[12] = 0x04;
        b[13] = checksum(&b);
        d.reply(&b);
        let s = sample(&mut d, &answer(2, 0x11, DIGITS));
        assert!(s.flags.contains(MqFlags::AUTORANGE));
        assert!(s.flags.contains(MqFlags::RMS), "a 24 has it");
    }

    #[test]
    fn every_model_code_and_its_name() {
        for (code, name) in [
            (0x04, "METRAHit 12S"),
            (0x08, "METRAHit 13S/14A"),
            (0x09, "METRAHit 14S"),
            (0x0a, "METRAHit 15S"),
            (0x0b, "METRAHit 16S"),
            (0x06, "METRAHit 16I/16L"),
            (0x07, "METRAHit 16T/16U/KMM2002"),
            (0x0d, "METRAHit 18S"),
            (0x02, "METRAHit 22S/M"),
            (0x03, "METRAHit 23S"),
            (0x0f, "METRAHit 24S"),
            (0x05, "METRAHit 25S"),
            (0x01, "METRAHit 26S/M"),
            (0x0c, "METRAHit 28S"),
            (0x0e, "METRAHit 29S"),
        ] {
            assert_eq!(decode_model_sm(code).name(), name);
        }
        assert_eq!(decode_model_sm(0), Model::None);
        for (code, name) in [
            (2, "METRAHit 22S"),
            (0x22, "METRAHit 22M"),
            (3, "METRAHit 23S"),
            (4, "METRAHit 24S"),
            (5, "METRAHit 25S"),
            (1, "METRAHit 26S"),
            (0x21, "METRAHit 26M"),
            (12, "METRAHit 28S"),
            (14, "METRAHit 29S"),
        ] {
            assert_eq!(decode_model_bd(code).name(), name);
        }
        assert_eq!(decode_model_bd(0), Model::None);
        assert_eq!(decode_model_bd(6), Model::None);
    }

    #[test]
    fn the_checks_refuse_and_a_request_from_all_passes() {
        let mut d = decoder(Model::M26S);
        d.addr = 1;
        let good = answer(1, 0x11, DIGITS);
        let mut other = good;
        other[0] = 2;
        other[13] = checksum(&other);
        assert_eq!(d.reply(&other), Reply::Bad, "another address");
        let mut wrong = good;
        wrong[13] ^= 1;
        assert_eq!(d.reply(&wrong), Reply::Bad, "the checksum");
        let mut header = good;
        header[2] = 0x3e;
        header[13] = checksum(&header);
        assert_eq!(d.reply(&header), Reply::Bad, "byte 2");
        let mut error = [1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        error[13] = checksum(&error);
        assert_eq!(d.reply(&error), Reply::Bad, "an error block");
        let mut request = good;
        request[0] = 3; // Address 0 in a request's form.
        request[1] = 0x2b;
        request[13] = checksum(&request);
        assert!(matches!(d.reply(&request), Reply::Good(Some(_))), "as C: a request reads as one");
    }

    #[test]
    fn a_block_out_of_step_finds_the_next_header() {
        let good = answer(1, 0x11, DIGITS);
        // The tail of one answer, then the head of the next.
        let mut torn = [0; REPLY_BYTES];
        torn[..3].copy_from_slice(&good[11..]);
        torn[3..].copy_from_slice(&good[..11]);
        assert_eq!(resync(&torn, 1), Some(3));
        assert_eq!(resync(&good, 1), None, "in step");
        let mut error = [1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        error[13] = checksum(&error);
        assert_eq!(resync(&error, 1), None, "an error block is in step");
        // A 0 in the second byte is a digit if the checksum is off.
        let mut digits = error;
        digits[13] ^= 1;
        assert_eq!(resync(&digits, 1), Some(13));
        // Nothing like a header: only the last byte may start one.
        assert_eq!(resync(&[9; REPLY_BYTES], 1), Some(13));
        // A header cut at the end still counts.
        let mut end = [9; REPLY_BYTES];
        end[13] = 0x27;
        assert_eq!(resync(&end, 1), Some(12));
    }

    #[test]
    fn a_stream_waits_for_a_start_and_frames_each_size() {
        let mut s = SendStream::new(Model::M26SM);
        assert_eq!(s.byte(0x30), None, "data ahead of the first start");
        assert_eq!(s.buf.len(), 0);
        let block = block_13(1, 1, 1, 0, DIGITS);
        let got: Vec<Sample> = block.iter().filter_map(|b| s.byte(*b)).collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].value, 1.23456);
        assert_eq!(got[0].mq, Some(Mq::Voltage));
        // In step now, a stray data byte is an error and costs itself.
        assert_eq!(s.byte(0x30), None);
        assert_eq!(s.buf.len(), 0);
        // 5 bytes end at the next start, which stays.
        for b in [0x01, 0x31, 0x30, 0x30, 0x31] {
            assert_eq!(s.byte(b), None);
        }
        assert_eq!(s.buf.len(), 5);
        assert_eq!(s.byte(0x11), None);
        assert_eq!(s.buf, [0x11]);
        let got: Vec<Sample> =
            [0x35, 0x34, 0x33, 0x32, 0x31].iter().filter_map(|b| s.byte(*b)).collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].value, 1.2345);
        assert_eq!(got[0].digits, 4, "a table of six, with five digits");
    }

    #[test]
    fn ten_bytes_end_a_block_only_up_to_an_18s() {
        let ten = [0x0b, 0x32, 0x30, 0x30, 0x32, 0x35, 0x34, 0x33, 0x32, 0x31];
        let mut s = SendStream::new(Model::M16S);
        assert_eq!(ten.iter().filter_map(|b| s.byte(*b)).count(), 1);
        let mut s = SendStream::new(Model::M22SM);
        assert_eq!(ten.iter().filter_map(|b| s.byte(*b)).count(), 0);
        assert_eq!(s.buf.len(), 10);
    }

    #[test]
    fn a_function_past_the_table_keeps_none() {
        let mut d = decoder(Model::M26SM);
        let s = d.block_13(&block_13(1, 33, 1, 0, DIGITS));
        assert_eq!((s.mq, s.unit), (None, None));
    }

    #[test]
    fn the_scale_is_c_s_float_and_double_arithmetic() {
        // Each step goes through a float: 123456 is exact, the product is not.
        let mut d = decoder(Model::M26SM);
        let s = d.block_13(&block_13(1, 4, 1, 0, DIGITS));
        let want = (f64::from(123_456.0f32) * (10f64.powf(-5.0) * 1000f64.powf(-1.0))) as f32;
        assert_eq!(s.value, want);
        assert_eq!(s.digits, 8);
    }
}
