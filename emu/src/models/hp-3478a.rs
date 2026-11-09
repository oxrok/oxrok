// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An HP 3478A multimeter behind a National Instruments GPIB-USB-HS adapter
//! (`3923:709b`), so that the kernel's `gpib` subsystem and its `ni_usb_gpib` driver
//! carry it exactly as they carry a real one: `/dev/gpibN`, its ioctls, the
//! adapter's bulk protocol and the GPIB bus behind it.
//!
//! The device, and how sure this is:
//!
//! - **The adapter, from the kernel's driver alone** (`drivers/gpib/ni_usb/`,
//!   `ni_usb_gpib.c` and `.h`, torvalds/linux master of 2026-11): the ids, the
//!   endpoints of the HS (bulk OUT 2, bulk IN 4, interrupt IN 1), the vendor
//!   requests (`0x41` serial number, `0x40` ready, `0x21` wait and its `0x200`
//!   status and `0x300` monitor forms, `0x20` stop), and each bulk request with the
//!   answer its parser takes: register writes (`0x09`, 16 bytes back), register
//!   reads (`0x08`), command bytes (`0x0c`, at most 16), data writes (`0x0d`),
//!   reads (`0x0a`, data in blocks of `0x36` and 15 bytes, a `0x38` status, the
//!   two register writes' status), take control (`0x01`), standby (`0x06`) and
//!   interface clear (`0x0f`), each ending in the termination `04 00 00 00`. The
//!   error codes are the driver's: 3 for a board not addressed, 5 for no device on
//!   the bus, 8 for no listener, 10 for a timeout. **Inferred, not seen:** the
//!   string descriptors, the alternate endpoints (6 OUT and 8 IN, which the driver
//!   never uses), the bytes of the ready answer (one of the sets the driver takes,
//!   an HS's by its comments), what the id byte of a status from a control request
//!   is, what a register read's answer ends with, and that the interrupt endpoint
//!   completes only when SRQ rises while the host monitors it.
//! - **The meter, from HP's quick reference card** (Martin Hepperle's 2018
//!   typesetting of the 3478A card) **and the operating manual's programming
//!   chapter** (paragraphs 3-105 to 3-113 as manualslib shows them): the codes
//!   `F1`..`F7`, `R-2`..`R7` and `RA`, `N3`..`N5`, `T1`..`T5`, `Z0`/`Z1`, `D1`..`D3`,
//!   `M`, `K`, `B` and `S`; the five binary status bytes (function, range and
//!   digits; status; SRQ mask; error; the DAC value); the serial poll byte (bit 0
//!   data ready, cleared when the reading is output; bits 2 to 5 and 7 cleared by
//!   `K` and a device clear; bit 6 the service request, cleared by a serial poll);
//!   a reading as 13 characters, `+1.23456E+0` and CR LF, with EOI, the fifth and
//!   sixth digits zero below 5½ digits, and an overload, plus or minus, as
//!   `+9.99999E+9`. **Inferred:** that the meter takes a message at a line feed or
//!   EOI, what it does with a code it has no use for (a syntax error, bit 2), the
//!   factory address 23, and the values themselves (a fixed walk per reading).
//!
//! What the bus does is IEEE 488.1's: `MLA` and `MTA` address, `UNL` and `UNT`
//! unaddress, a talker addressed to listen stops talking and the other way round,
//! `SPE` and `SPD` frame a serial poll, `GTL`, `LLO`, `SDC`, `DCL` and `GET`. The
//! meter has no secondary address.
//!
//! Time is the host's count, never the clock: a reading completes on the meter's
//! `every`th serial poll since its last one was output (2 by default, so that a poll
//! with nothing ready comes in every case). A read with nothing to say answers the
//! adapter's timeout error at once, where a real one waits for the timeout the
//! request names: the host sees the same error, sooner.
//!
//! Options: `pad=N` the meter's address (23), `board-pad=N` the adapter's own (0, as
//! `gpib_config` sets with no `pad`), `absent=1` nothing on the bus at all,
//! `every=N` polls per reading (0: none ever), `init=CODES` codes the meter takes
//! at power on (`F3R2N4`), `dac=N` the DAC's setting in the status's fifth byte (33), `rear=1` the rear terminals, `hz=50` the line, `cal=1`
//! the CAL RAM switch, `error=HEX` the error register, `power-on-srq=1` the rear
//! switch that requests service at power on, and the faults, each numbered from 1:
//! `mute-poll=N` (the Nth serial poll is not answered), `mute-read=N` (the Nth
//! reading read is not answered), `mute-status=N` (the Nth binary status is not
//! answered), `cut-read=N` (the Nth reading stops after 5 bytes with no EOI, and the meter
//! says nothing more),
//! `junk-at=N` (the Nth reading is no number), `overload-at=N` (the Nth reading is
//! an overload), `short-status=N` (the binary status is N bytes, 1 to 4) and
//! `mark-at=N` (the model's mark after the Nth reading, for `--unplug model`).
//!
//! The pedantic checks: every bulk request is one the driver sends, with its
//! lengths, its padding and its termination; a request comes only once the answer
//! to the one before has been read; a register write names a device of the
//! adapter's; a command byte is one a host may send to this bus, the meter's
//! secondary address none; data goes out only with the adapter addressed to talk,
//! a read only with it addressed to listen, and neither with ATN on; and the meter
//! gets only codes that it has and that libsigrok's driver sends (`F`, `R`, `N`,
//! `T`, `D`, `B` and the ones a user may need: `Z`, `M`, `K`, `S`), within their
//! ranges.

use std::collections::VecDeque;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    BULK, Descriptors, DeviceModel, INTERRUPT, Reply, Setup, Speed, VENDOR, Violations,
};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hp-3478a";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "HP 3478A multimeter behind an NI GPIB-USB-HS (3923:709b) for the \
                         kernel's ni_usb_gpib; pad=N board-pad=N absent=1 every=N init=CODES \
                         dac=N rear=1 hz=50 cal=1 error=HEX power-on-srq=1, faults mute-poll=N \
                         mute-read=N mute-status=N cut-read=N junk-at=N overload-at=N \
                         short-status=N mark-at=N";

/// National Instruments.
pub const VENDOR_ID: u16 = 0x3923;
/// The GPIB-USB-HS.
pub const PRODUCT_ID: u16 = 0x709b;
/// Bulk OUT, the requests.
pub const EP_OUT: u8 = 2;
/// Bulk IN, the answers.
pub const EP_IN: u8 = 4;
/// Interrupt IN, status changes.
pub const EP_INT: u8 = 1;
/// The meter's factory address.
pub const DEFAULT_PAD: u8 = 23;

// The adapter's bulk request ids (ni_usb_gpib.h, ni_usb_bulk_ids).
const IBCAC: u8 = 0x01;
const TERM: u8 = 0x04;
const IBGTS: u8 = 0x06;
const IBRPP: u8 = 0x07;
const REG_READ: u8 = 0x08;
const REG_WRITE: u8 = 0x09;
const READ: u8 = 0x0a;
const COMMAND: u8 = 0x0c;
const WRITE: u8 = 0x0d;
const IBSIC: u8 = 0x0f;
const READ_DATA_START: u8 = 0x34;
// The end of string mode's high byte in a read request (gpib.h: REOS, BIN).
const REOS: u8 = 0x04;
const BIN: u8 = 0x10;
const READ_DATA_END: u8 = 0x35;
const IBRD_DATA: u8 = 0x36;
const IBRD_STATUS: u8 = 0x38;

// Error codes (ni_usb_error_codes).
const ADDRESSING_ERROR: u8 = 3;
const NO_BUS_ERROR: u8 = 5;
const NO_LISTENER_ERROR: u8 = 8;
const TIMEOUT_ERROR: u8 = 10;

// Vendor requests (ni_usb_control_requests).
const STOP_REQUEST: u8 = 0x20;
const WAIT_REQUEST: u8 = 0x21;
const POLL_READY_REQUEST: u8 = 0x40;
const SERIAL_NUMBER_REQUEST: u8 = 0x41;

// ibsta bits (uapi gpib.h).
const LACS: u16 = 1 << 2;
const TACS: u16 = 1 << 3;
const ATN: u16 = 1 << 4;
const CIC: u16 = 1 << 5;
const SRQI: u16 = 1 << 12;
const END: u16 = 1 << 13;

// The TNT4882's registers the driver names (nec7210 offsets doubled).
const SUBDEV_TNT4882: u8 = 1;
const AUXMR: u8 = 2 * 5;
const ADR: u8 = 2 * 6;
const BSR: u8 = 0x1f;
const AUX_SREN: u8 = 0x1f;
const AUX_CREN: u8 = 0x17;
const BCSR_REN: u8 = 0x01;
const BCSR_SRQ: u8 = 0x04;
const BCSR_ATN: u8 = 0x80;

// Bus commands (gpib_cmd.h).
const GTL: u8 = 0x01;
const SDC: u8 = 0x04;
const GET: u8 = 0x08;
const LLO: u8 = 0x11;
const DCL: u8 = 0x14;
const SPE: u8 = 0x18;
const SPD: u8 = 0x19;
const UNL: u8 = 0x3f;
const UNT: u8 = 0x5f;

// The serial poll byte.
const SPOLL_READY: u8 = 0x01;
const SPOLL_SYNTAX: u8 = 0x04;
const SPOLL_RQS: u8 = 0x40;
const SPOLL_POWER_ON: u8 = 0x80;

/// The overload reading.
pub const OVERLOAD: &str = "+9.99999E+9";

/// A reading's sign and six digits, the walk the meter's values take: never the same
/// twice in a row, and every third one negative where the function has a sign.
pub fn reading_digits(n: u32) -> (bool, u32) {
    let digits = (12_345 + 23_456 * n) % 300_000;
    (n % 3 == 2, digits)
}

/// The functions, `F1` to `F7`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Function {
    /// DC volts.
    Vdc = 1,
    /// AC volts.
    Vac = 2,
    /// Two-wire ohms.
    Ohm2 = 3,
    /// Four-wire ohms.
    Ohm4 = 4,
    /// DC amps.
    Adc = 5,
    /// AC amps.
    Aac = 6,
    /// Extended ohms.
    OhmExt = 7,
}

impl Function {
    fn from_code(code: u8) -> Option<Function> {
        Some(match code {
            1 => Function::Vdc,
            2 => Function::Vac,
            3 => Function::Ohm2,
            4 => Function::Ohm4,
            5 => Function::Adc,
            6 => Function::Aac,
            7 => Function::OhmExt,
            _ => return None,
        })
    }

    /// The `R` codes the function has, lowest first (the card's range table).
    pub fn ranges(self) -> std::ops::RangeInclusive<i8> {
        match self {
            Function::Vdc => -2..=2,
            Function::Vac => -1..=2,
            Function::Ohm2 | Function::Ohm4 => 1..=7,
            Function::Adc | Function::Aac => -1..=0,
            Function::OhmExt => 7..=7,
        }
    }

    /// Whether a reading has a sign.
    fn signed(self) -> bool {
        matches!(self, Function::Vdc | Function::Adc)
    }

    /// The range a function starts on when it is selected under autorange.
    fn home(self) -> i8 {
        match self {
            Function::Vdc | Function::Vac | Function::Adc | Function::Aac => 0,
            Function::Ohm2 | Function::Ohm4 => 3,
            Function::OhmExt => 7,
        }
    }
}

/// The meter's state.
#[derive(Clone, Debug, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Meter {
    /// The function.
    pub function: Function,
    /// The range's `R` code.
    pub range: i8,
    /// Autorange on.
    pub autorange: bool,
    /// Digits: 3, 4 or 5 (the half digit not counted).
    pub digits: u8,
    /// The trigger, 1 to 5.
    pub trigger: u8,
    /// Autozero on.
    pub autozero: bool,
    /// The display mode, 1 to 3.
    pub display: u8,
    /// The SRQ mask.
    pub mask: u8,
    /// The serial poll register, bit 6 aside.
    pub spoll: u8,
    /// Service requested: bit 6 and the SRQ line.
    pub rqs: bool,
    /// The error register.
    pub error: u8,
    /// Front terminals.
    pub front: bool,
    /// A 50 Hz line.
    pub hz50: bool,
    /// The CAL RAM switch.
    pub cal: bool,
    /// The DAC's setting, 0 to 63.
    pub dac: u8,
}

impl Default for Meter {
    /// As the meter comes up: DC volts on autorange at 3 V, 5½ digits, the internal
    /// trigger, autozero on, the front terminals and a 60 Hz line.
    fn default() -> Meter {
        Meter {
            function: Function::Vdc,
            range: 0,
            autorange: true,
            digits: 5,
            trigger: 1,
            autozero: true,
            display: 1,
            mask: 0,
            spoll: 0,
            rqs: false,
            error: 0,
            front: true,
            hz50: false,
            cal: false,
            dac: 33,
        }
    }
}

impl Meter {
    /// The range's code in the function's first byte: 1 for its lowest range.
    fn range_code(&self) -> u8 {
        let lowest = match self.function {
            Function::Vdc => -2,
            Function::Vac | Function::Adc | Function::Aac => -1,
            Function::Ohm2 | Function::Ohm4 | Function::OhmExt => 1,
        };
        // An extended ohms range sits where 30 MOhm does.
        if self.function == Function::OhmExt {
            return 7;
        }
        (self.range - lowest + 1) as u8
    }

    /// The five bytes of `B`.
    pub fn binary_status(&self) -> [u8; 5] {
        let digits = match self.digits {
            5 => 1,
            4 => 2,
            _ => 3,
        };
        let first = (self.function as u8) << 5 | self.range_code() << 2 | digits;
        let second = u8::from(self.trigger == 2) << 6
            | u8::from(self.cal) << 5
            | u8::from(self.front) << 4
            | u8::from(self.hz50) << 3
            | u8::from(self.autozero) << 2
            | u8::from(self.autorange) << 1
            | u8::from(self.trigger == 1);
        [first, second, self.mask, self.error, self.dac]
    }

    /// The serial poll byte.
    pub fn spoll_byte(&self) -> u8 {
        self.spoll | if self.rqs { SPOLL_RQS } else { 0 }
    }

    /// Sets bits of the serial poll register, and requests service for those the mask
    /// has (bit 7 by the rear switch, which the mask shows too).
    fn raise(&mut self, bits: u8) {
        self.spoll |= bits;
        if bits & self.mask != 0 {
            self.rqs = true;
        }
    }

    /// `K` and a device clear: bits 2 to 5 and 7 go, data ready stays.
    fn clear_status(&mut self) {
        self.spoll &= SPOLL_READY;
        if self.spoll & self.mask == 0 {
            self.rqs = false;
        }
    }

    /// The text of reading number `n` (from 0).
    pub fn reading(&self, n: u32) -> String {
        let (negative, mut digits) = reading_digits(n);
        let negative = negative && self.function.signed();
        // Below 5½ digits the last one or two are zeros.
        match self.digits {
            4 => digits -= digits % 10,
            3 => digits -= digits % 100,
            _ => {}
        }
        // R codes run in decades from 3: R0 is 3 units, R1 30, R-1 300 milli.
        let (shift, exp) = match self.range.rem_euclid(3) {
            0 => (1, self.range),
            1 => (2, self.range - 1),
            _ => (3, self.range - 2),
        };
        let text = format!("{digits:06}");
        let (int, frac) = text.split_at(shift);
        format!("{}{int}.{frac}E{exp:+}", if negative { '-' } else { '+' })
    }

    /// Takes one code; `Err` with what is wrong with it.
    fn code(&mut self, code: char, arg: &str) -> std::result::Result<(), String> {
        let digit = |lo: u8, hi: u8| -> std::result::Result<u8, String> {
            arg.parse::<u8>()
                .ok()
                .filter(|d| (lo..=hi).contains(d) && arg.len() == 1)
                .ok_or_else(|| format!("{code}{arg}: {code} takes {lo} to {hi}"))
        };
        match code {
            'F' => {
                let f = Function::from_code(digit(1, 7)?).expect("checked");
                if f != self.function {
                    self.function = f;
                    if self.autorange || !f.ranges().contains(&self.range) {
                        self.range = f.home();
                    }
                }
            }
            'R' if arg == "A" => self.autorange = true,
            'R' => {
                let r: i8 = arg
                    .parse()
                    .ok()
                    .filter(|r: &i8| (-2..=7).contains(r) && !arg.starts_with('+'))
                    .ok_or_else(|| format!("R{arg}: R takes A or -2 to 7"))?;
                if !self.function.ranges().contains(&r) {
                    return Err(format!("R{arg}: no such range on F{}", self.function as u8));
                }
                self.range = r;
                self.autorange = false;
            }
            'N' => self.digits = digit(3, 5)?,
            'T' => self.trigger = digit(1, 5)?,
            'Z' => self.autozero = digit(0, 1)? == 1,
            'D' => self.display = digit(1, 3)?,
            'M' => {
                let bits = u8::from_str_radix(arg, 8)
                    .ok()
                    .filter(|_| arg.len() == 2)
                    .ok_or_else(|| format!("M{arg}: M takes two octal digits"))?;
                self.mask = bits & 0x3f | self.mask & SPOLL_POWER_ON;
                if self.spoll & self.mask == 0 {
                    self.rqs = false;
                }
            }
            _ => return Err(format!("{code}{arg}: no such code")),
        }
        Ok(())
    }
}

/// The faults, each a count from 1 at which it fires.
#[derive(Clone, Debug, Default)]
struct Faults {
    mute_poll: Option<u32>,
    mute_read: Option<u32>,
    mute_status: Option<u32>,
    cut_read: Option<u32>,
    junk_at: Option<u32>,
    overload_at: Option<u32>,
    short_status: Option<usize>,
    mark_at: Option<u32>,
}

/// The emulated adapter, its bus and the meter on it.
#[allow(clippy::struct_excessive_bools)]
pub struct Hp3478a {
    descriptors: Descriptors,
    /// The meter's address.
    pad: u8,
    /// The adapter's own address.
    board_pad: u8,
    /// Nothing on the bus.
    absent: bool,
    /// Polls per reading.
    every: u32,
    faults: Faults,
    meter: Meter,

    // The adapter and the bus.
    atn: bool,
    ren: bool,
    board_listens: bool,
    board_talks: bool,
    meter_listens: bool,
    meter_talks: bool,
    serial_poll: bool,
    /// The ibsta bits the host monitors on the interrupt endpoint.
    monitored: u16,
    /// SRQ rose while monitored: the interrupt endpoint has a status to send.
    interrupt: bool,
    /// The answer to the last bulk request, until read.
    answer: Option<Vec<u8>>,

    // The meter's GPIB side.
    remote: bool,
    lockout: bool,
    input: Vec<u8>,
    /// What a `B` or `S` left to say, the last byte with EOI, ahead of a reading.
    output: Option<Vec<u8>>,
    /// What is left of the bytes being sent.
    sending: VecDeque<u8>,
    /// Whether the last of `sending` goes with EOI.
    sending_eoi: bool,
    /// A reading is complete and not yet output.
    ready: bool,
    polls_since: u32,
    /// Readings output, statuses output, polls answered.
    readings: u32,
    statuses: u32,
    polls: u32,
    marked: bool,
    /// A reading was cut: the meter says nothing more.
    dead: bool,
}

/// A model from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(Hp3478a::from_options(options)?))
}

fn number(key: &str, value: &str, range: std::ops::RangeInclusive<u32>) -> Result<u32> {
    crate::options::parse_number(key, value, range)
}

fn flag(key: &str, value: &str) -> Result<bool> {
    Ok(number(key, value, 0..=1)? == 1)
}

impl Default for Hp3478a {
    fn default() -> Self {
        Self::new()
    }
}

impl Hp3478a {
    /// A meter at address 23, as it comes up.
    pub fn new() -> Hp3478a {
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 5, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(0x80 | EP_INT, INTERRUPT, 64, 4));
        body.extend_from_slice(&descriptor::endpoint(EP_OUT, BULK, 512, 0));
        body.extend_from_slice(&descriptor::endpoint(0x80 | EP_IN, BULK, 512, 0));
        body.extend_from_slice(&descriptor::endpoint(6, BULK, 512, 0));
        body.extend_from_slice(&descriptor::endpoint(0x88, BULK, 512, 0));
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x0101,
            strings: [1, 2, 3],
            configurations: 1,
        };
        Hp3478a {
            descriptors: Descriptors {
                speed: Speed::High,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 0, 0x80, 500, &body)],
                strings: vec![
                    "National Instruments".to_owned(),
                    "GPIB-USB-HS".to_owned(),
                    "01A2B3C4".to_owned(),
                ],
            },
            pad: DEFAULT_PAD,
            board_pad: 0,
            absent: false,
            every: 2,
            faults: Faults::default(),
            meter: Meter::default(),
            atn: false,
            // gpib_config asserts REN and sends IFC after the attach.
            ren: true,
            board_listens: false,
            board_talks: false,
            meter_listens: false,
            meter_talks: false,
            serial_poll: false,
            monitored: 0,
            interrupt: false,
            answer: None,
            remote: false,
            lockout: false,
            input: Vec::new(),
            output: None,
            sending: VecDeque::new(),
            sending_eoi: false,
            ready: false,
            polls_since: 0,
            readings: 0,
            statuses: 0,
            polls: 0,
            marked: false,
            dead: false,
        }
    }

    /// The model the options describe.
    ///
    /// # Errors
    ///
    /// On an unknown option or a value out of range.
    pub fn from_options(options: &[(String, String)]) -> Result<Hp3478a> {
        let mut m = Hp3478a::new();
        let mut init = None;
        for (k, v) in options {
            let at = |v: &str| number(k, v, 1..=1_000_000).map(Some);
            match k.as_str() {
                "pad" => m.pad = number(k, v, 0..=30)? as u8,
                "board-pad" => m.board_pad = number(k, v, 0..=30)? as u8,
                "absent" => m.absent = flag(k, v)?,
                "every" => m.every = number(k, v, 0..=1000)?,
                "init" => init = Some(v.clone()),
                "rear" => m.meter.front = !flag(k, v)?,
                "hz" => {
                    m.meter.hz50 = match v.as_str() {
                        "50" => true,
                        "60" => false,
                        _ => return Err(Error::Invalid(format!("hz={v}: 50 or 60"))),
                    }
                }
                "cal" => m.meter.cal = flag(k, v)?,
                "dac" => m.meter.dac = number(k, v, 0..=63)? as u8,
                "error" => {
                    m.meter.error = u8::from_str_radix(v, 16)
                        .ok()
                        .filter(|e| *e < 0x40)
                        .ok_or_else(|| Error::Invalid(format!("error={v}: hex, 00 to 3f")))?;
                }
                "power-on-srq" => {
                    if flag(k, v)? {
                        m.meter.mask |= SPOLL_POWER_ON;
                        m.meter.raise(SPOLL_POWER_ON);
                    }
                }
                "mute-poll" => m.faults.mute_poll = at(v)?,
                "mute-read" => m.faults.mute_read = at(v)?,
                "mute-status" => m.faults.mute_status = at(v)?,
                "cut-read" => m.faults.cut_read = at(v)?,
                "junk-at" => m.faults.junk_at = at(v)?,
                "overload-at" => m.faults.overload_at = at(v)?,
                "short-status" => m.faults.short_status = Some(number(k, v, 1..=4)? as usize),
                "mark-at" => m.faults.mark_at = at(v)?,
                other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
            }
        }
        if let Some(codes) = init {
            let mut v = Violations::default();
            m.meter_message(codes.as_bytes(), &mut v);
            if let Some(e) = v.drain().first() {
                return Err(Error::Invalid(format!("init={codes}: {e}")));
            }
        }
        if m.board_pad == m.pad {
            return Err(Error::Invalid(format!("pad={} is the adapter's own address", m.pad)));
        }
        Ok(m)
    }

    /// The meter, for a test.
    pub fn meter(&self) -> &Meter {
        &self.meter
    }

    /// The meter is in remote.
    pub fn remote(&self) -> bool {
        self.remote
    }

    fn ibsta(&self) -> u16 {
        let mut s = CIC;
        if self.atn {
            s |= ATN;
        }
        if self.board_listens {
            s |= LACS;
        }
        if self.board_talks {
            s |= TACS;
        }
        if self.meter.rqs {
            s |= SRQI;
        }
        s
    }

    /// A status block: id, ibsta, error, the count left (as its negative), two zeros.
    fn status(&self, id: u8, error: u8, left: usize, end: bool) -> [u8; 8] {
        let ibsta = self.ibsta() | if end { END } else { 0 };
        let count = (left as u16).wrapping_neg().to_le_bytes();
        let s = ibsta.to_be_bytes();
        [id, s[0], s[1], error, count[0], count[1], 0, 0]
    }

    fn termination(out: &mut Vec<u8>) {
        out.extend_from_slice(&[TERM, 0, 0, 0]);
    }

    /// Checks that a request ends, from `at`, with zeros to a 4 byte boundary and the
    /// termination, and nothing after it.
    fn check_end(data: &[u8], at: usize, what: &str, v: &mut Violations) {
        let padded = at.div_ceil(4) * 4;
        if data.len() < padded || data[at..padded].iter().any(|b| *b != 0) {
            v.report(format!("{what}: not padded with zeros to a 4 byte boundary"));
            return;
        }
        if data[padded..] != [TERM, 0, 0, 0] {
            v.report(format!(
                "{what}: ends in {}, not the termination 04 00 00 00",
                crate::txlog::hex(&data[padded..])
            ));
        }
    }

    /// A register write request, its writes from `at`.
    fn register_writes(&mut self, data: &[u8], v: &mut Violations) -> Vec<u8> {
        let n = usize::from(data.get(1).copied().unwrap_or(0));
        if data.get(2) != Some(&0) || data.len() < 3 + 3 * n {
            v.report(format!("register write of {n}: {}", crate::txlog::hex(data)));
        } else {
            for w in data[3..3 + 3 * n].chunks(3) {
                self.register_write(w[0], w[1], w[2], v);
            }
            Self::check_end(data, 3 + 3 * n, "register write", v);
        }
        let mut out = self.status(REG_WRITE, 0, 0, false).to_vec();
        out.extend_from_slice(&[n as u8, 0, 0, 0]);
        Self::termination(&mut out);
        out
    }

    fn register_write(&mut self, device: u8, address: u8, value: u8, v: &mut Violations) {
        if !(1..=3).contains(&device) {
            v.report(format!("register write to device {device}; the adapter has 1 to 3"));
            return;
        }
        if device == SUBDEV_TNT4882 {
            match (address, value) {
                (AUXMR, AUX_SREN) => self.ren = true,
                (AUXMR, AUX_CREN) => {
                    self.ren = false;
                    self.remote = false;
                    self.lockout = false;
                }
                (ADR, a) if a & 0x80 == 0 => self.board_pad = a & 0x1f,
                _ => {}
            }
        }
    }

    /// A register read request: BSR is the bus, the serial number's registers have
    /// a number, the rest read 0.
    fn register_reads(&mut self, data: &[u8], v: &mut Violations) -> Vec<u8> {
        let n = usize::from(data.get(1).copied().unwrap_or(0));
        let mut results = Vec::new();
        if n == 0 || data.len() < 2 + 2 * n {
            v.report(format!("register read of {n}: {}", crate::txlog::hex(data)));
        } else {
            for r in data[2..2 + 2 * n].chunks(2) {
                results.push(match (r[0], r[1]) {
                    (SUBDEV_TNT4882, BSR) => {
                        (u8::from(self.ren) * BCSR_REN)
                            | (u8::from(self.meter.rqs) * BCSR_SRQ)
                            | (u8::from(self.atn) * BCSR_ATN)
                    }
                    (3, a @ 8..=11) => [0xc4, 0xb3, 0xa2, 0x01][usize::from(a - 8)],
                    (d, a) => {
                        v.report(format!("register read of device {d} address {a:#04x}"));
                        0
                    }
                });
            }
            Self::check_end(data, 2 + 2 * n, "register read", v);
        }
        let mut out = Vec::new();
        for chunk in results.chunks(3) {
            out.push(READ_DATA_START);
            out.extend_from_slice(chunk);
        }
        while !out.len().is_multiple_of(4) {
            out.push(0);
        }
        out.extend_from_slice(&[READ_DATA_END, results.len() as u8, 0, 0]);
        Self::termination(&mut out);
        out
    }

    /// Command bytes, with ATN on.
    fn command(&mut self, data: &[u8], v: &mut Violations) -> Vec<u8> {
        let n = usize::from(!data.get(1).copied().unwrap_or(0)) + 1;
        if data.get(2) != Some(&0) || data.len() < 4 + n || n > 16 {
            v.report(format!("command of {n} bytes: {}", crate::txlog::hex(data)));
            return self.short_answer(COMMAND, 0, 0);
        }
        Self::check_end(data, 4 + n, "command", v);
        if !self.atn {
            v.report("command bytes with ATN off: the host takes control first");
        }
        if self.absent {
            return self.short_answer(COMMAND, NO_BUS_ERROR, n);
        }
        for &byte in &data[4..4 + n] {
            self.bus_command(byte & 0x7f, v);
        }
        self.short_answer(COMMAND, 0, 0)
    }

    fn bus_command(&mut self, c: u8, v: &mut Violations) {
        match c {
            UNL => {
                self.meter_listens = false;
                self.board_listens = false;
            }
            UNT => {
                self.meter_talks = false;
                self.board_talks = false;
            }
            0x20..=0x3e => {
                let a = c & 0x1f;
                if a == self.pad {
                    self.meter_listens = true;
                    self.meter_talks = false;
                    if self.ren {
                        self.remote = true;
                    }
                }
                if a == self.board_pad {
                    self.board_listens = true;
                    self.board_talks = false;
                }
            }
            0x40..=0x5e => {
                let a = c & 0x1f;
                self.meter_talks = a == self.pad;
                self.board_talks = a == self.board_pad;
                if self.meter_talks {
                    self.meter_listens = false;
                }
                if self.board_talks {
                    self.board_listens = false;
                }
            }
            SPE => self.serial_poll = true,
            SPD => self.serial_poll = false,
            GTL if self.meter_listens => {
                self.remote = false;
                self.input.clear();
            }
            LLO => self.lockout = true,
            DCL => self.device_clear(),
            SDC if self.meter_listens => self.device_clear(),
            GET if self.meter_listens => self.trigger(),
            // Addressed commands for others.
            GTL | SDC | GET => {}
            0x60..=0x7f => v.report(format!("secondary address {c:#04x}: the 3478A has none")),
            other => v.report(format!("command byte {other:#04x}: no driver sends it")),
        }
    }

    fn device_clear(&mut self) {
        self.input.clear();
        self.output = None;
        self.sending.clear();
        self.meter.clear_status();
    }

    /// A trigger: under the single trigger a reading completes.
    fn trigger(&mut self) {
        if self.meter.trigger == 3 && !self.ready {
            self.ready = true;
            self.meter.raise(SPOLL_READY);
        }
    }

    /// A data write: bytes for the listeners.
    fn write(&mut self, data: &[u8], v: &mut Violations) -> Vec<u8> {
        let n = usize::from(u16::from_le_bytes([
            !data.get(1).copied().unwrap_or(0),
            !data.get(2).copied().unwrap_or(0),
        ])) + 1;
        if data.len() < 8 + n || data[4..6] != [0, 0] || data[7] != 0 || !matches!(data[6], 0 | 8) {
            v.report(format!("write of {n} bytes: {}", crate::txlog::hex(data)));
            return self.short_answer(WRITE, 0, 0);
        }
        Self::check_end(data, 8 + n, "write", v);
        let eoi = data[6] == 8;
        if self.atn {
            v.report("a data write with ATN on: the host goes to standby first");
        }
        if !self.board_talks {
            return self.short_answer(WRITE, ADDRESSING_ERROR, n);
        }
        if self.absent || !self.meter_listens {
            return self.short_answer(WRITE, NO_LISTENER_ERROR, n);
        }
        let bytes = &data[8..8 + n];
        for (i, &b) in bytes.iter().enumerate() {
            self.input.push(b);
            if b == b'\n' || (eoi && i + 1 == n) {
                let message = std::mem::take(&mut self.input);
                self.meter_message(&message, v);
            }
        }
        self.short_answer(WRITE, 0, 0)
    }

    /// A status alone, and the termination: the 12 byte answer.
    fn short_answer(&self, id: u8, error: u8, left: usize) -> Vec<u8> {
        let mut out = self.status(id, error, left, false).to_vec();
        Self::termination(&mut out);
        out
    }

    /// A message the meter takes: its codes, in order.
    fn meter_message(&mut self, message: &[u8], v: &mut Violations) {
        let text = String::from_utf8_lossy(message).into_owned();
        let mut chars = text.chars().peekable();
        while let Some(c) = chars.next() {
            let c = c.to_ascii_uppercase();
            match c {
                ' ' | ',' | '\r' | '\n' => {}
                'B' => {
                    self.statuses += 1;
                    let mut status = self.meter.binary_status().to_vec();
                    if let Some(n) = self.faults.short_status {
                        status.truncate(n);
                    }
                    self.output = Some(status);
                    // B also clears the error register.
                    self.meter.error = 0;
                }
                'S' => {
                    let switch = if self.meter.front { b"1\r\n" } else { b"0\r\n" };
                    self.output = Some(switch.to_vec());
                }
                'K' => self.meter.clear_status(),
                'D' if matches!(chars.peek(), Some('2' | '3')) => {
                    // The rest of the message is the text shown.
                    let mode = chars.next().expect("peeked");
                    self.meter.display = mode as u8 - b'0';
                    for t in chars.by_ref() {
                        if t == '\r' || t == '\n' {
                            break;
                        }
                    }
                }
                'F' | 'R' | 'N' | 'T' | 'Z' | 'D' | 'M' => {
                    let mut arg = String::new();
                    if c == 'R' && chars.peek() == Some(&'-') {
                        arg.push(chars.next().expect("peeked"));
                    }
                    let want = if c == 'M' { 2 } else { 1 };
                    while arg.trim_start_matches('-').len() < want {
                        match chars.peek() {
                            Some(d) if d.is_ascii_alphanumeric() => {
                                arg.push(d.to_ascii_uppercase());
                                chars.next();
                            }
                            _ => break,
                        }
                        if c == 'R' && arg == "A" {
                            break;
                        }
                    }
                    let before = self.meter.trigger;
                    match self.meter.code(c, &arg) {
                        Ok(()) => {
                            if c == 'T' && self.meter.trigger == 3 {
                                self.ready = false;
                                self.trigger();
                            }
                            if c == 'T' && before != self.meter.trigger && self.meter.trigger == 4 {
                                // Hold: no reading until the next trigger.
                                self.polls_since = 0;
                            }
                        }
                        Err(e) => {
                            v.report(format!("the meter refuses {e}"));
                            self.meter.raise(SPOLL_SYNTAX);
                        }
                    }
                }
                other => {
                    // A code's digits go with it.
                    while chars.peek().is_some_and(|d| d.is_ascii_digit() || *d == '-') {
                        chars.next();
                    }
                    v.report(format!("code {other:?}: the 3478A has none the driver sends"));
                    self.meter.raise(SPOLL_SYNTAX);
                }
            }
        }
    }

    /// What the meter puts on the bus when addressed to talk: `None` when it says
    /// nothing.
    fn talk(&mut self) -> Option<(Vec<u8>, bool)> {
        if self.dead {
            return None;
        }
        if self.serial_poll {
            self.polls += 1;
            if self.faults.mute_poll == Some(self.polls) {
                return None;
            }
            if !self.ready && self.meter.trigger == 1 && self.every > 0 {
                self.polls_since += 1;
                if self.polls_since >= self.every {
                    self.ready = true;
                    self.meter.raise(SPOLL_READY);
                }
            }
            let byte = self.meter.spoll_byte();
            // A serial poll ends the service request.
            self.meter.rqs = false;
            return Some((vec![byte], false));
        }
        match self.output.take() {
            // The status is lost: nothing more is said for it.
            Some(_) if self.faults.mute_status == Some(self.statuses) => None,
            Some(bytes) => Some((bytes, true)),
            None => self.next_reading(),
        }
    }

    fn next_reading(&mut self) -> Option<(Vec<u8>, bool)> {
        if !self.ready && self.meter.trigger != 1 {
            return None;
        }
        let n = self.readings;
        self.readings += 1;
        let count = self.readings;
        if self.faults.mute_read == Some(count) {
            return None;
        }
        self.ready = false;
        self.polls_since = 0;
        self.meter.spoll &= !SPOLL_READY;
        if self.faults.mark_at == Some(count) {
            self.marked = true;
        }
        let text = if self.faults.overload_at == Some(count) {
            OVERLOAD.to_owned()
        } else if self.faults.junk_at == Some(count) {
            "+1.2X456E+0".to_owned()
        } else {
            self.meter.reading(n)
        };
        let mut bytes = format!("{text}\r\n").into_bytes();
        if self.faults.cut_read == Some(count) {
            bytes.truncate(5);
            self.dead = true;
            return Some((bytes, false));
        }
        Some((bytes, true))
    }

    /// A read of up to `n` bytes.
    fn read(&mut self, data: &[u8], v: &mut Violations) -> Vec<u8> {
        let n = usize::from(u16::from_le_bytes([
            !data.get(4).copied().unwrap_or(0),
            !data.get(5).copied().unwrap_or(0),
        ])) + 1;
        // The two register writes behind it: AUX_HLDI and AUX_CLEAR_END.
        let behind = [REG_WRITE, 2, 0, 1, AUXMR, 0x51, 1, AUXMR, 0x55];
        if data.len() < 17 || data[6..8] != [0, 0] || data[8..17] != behind {
            v.report(format!("read of {n} bytes: {}", crate::txlog::hex(data)));
        } else {
            Self::check_end(data, 17, "read", v);
        }
        // An end of string mode the board was left in (REOS, BIN): the adapter ends
        // the read at that byte too. libgpib clears it before each read (iblcleos).
        let mode = data.get(1).copied().unwrap_or(0);
        let eos = (mode & REOS != 0).then(|| {
            let mask = if mode & BIN != 0 { 0xff } else { 0x7f };
            (data.get(2).copied().unwrap_or(0) & mask, mask)
        });
        if mode != 0 || data.get(2) != Some(&0) {
            v.report("a read with an end of string mode: the driver's reads end on EOI alone");
        }
        if self.atn {
            v.report("a read with ATN on: the host goes to standby first");
        }
        let (bytes, error, end) = if !self.board_listens {
            (Vec::new(), ADDRESSING_ERROR, false)
        } else if self.absent || !self.meter_talks {
            (Vec::new(), TIMEOUT_ERROR, false)
        } else {
            self.read_bytes(n, eos)
        };
        let mut out = Vec::new();
        for block in bytes.chunks(15) {
            out.push(IBRD_DATA);
            out.extend_from_slice(block);
            out.resize(out.len() + 15 - block.len(), 0);
        }
        out.extend_from_slice(&self.status(IBRD_STATUS, error, n - bytes.len(), end));
        let last = if bytes.is_empty() { 0 } else { (bytes.len() - 1) % 15 + 1 };
        out.extend_from_slice(&[0, last as u8, 0, 0]);
        out.extend_from_slice(&self.status(REG_WRITE, 0, 0, false));
        out.extend_from_slice(&[2, 0, 0, 0]);
        Self::termination(&mut out);
        out
    }

    /// Up to `n` of what the meter sends, the error code and whether EOI came.
    fn read_bytes(&mut self, n: usize, eos: Option<(u8, u8)>) -> (Vec<u8>, u8, bool) {
        if self.sending.is_empty() {
            match self.talk() {
                Some((bytes, eoi)) => {
                    self.sending = bytes.into();
                    self.sending_eoi = eoi;
                }
                None => return (Vec::new(), TIMEOUT_ERROR, false),
            }
        }
        let mut take = n.min(self.sending.len());
        let mut at_eos = false;
        if let Some((char, mask)) = eos
            && let Some(i) = self.sending.iter().take(take).position(|b| b & mask == char)
        {
            take = i + 1;
            at_eos = true;
        }
        let bytes: Vec<u8> = self.sending.drain(..take).collect();
        let end = at_eos || (self.sending.is_empty() && self.sending_eoi);
        (bytes, 0, end)
    }

    fn bulk(&mut self, data: &[u8], v: &mut Violations) -> Vec<u8> {
        match data.first().copied() {
            Some(REG_WRITE) => self.register_writes(data, v),
            Some(REG_READ) => self.register_reads(data, v),
            Some(COMMAND) => self.command(data, v),
            Some(WRITE) => self.write(data, v),
            Some(READ) => self.read(data, v),
            Some(id @ (IBCAC | IBGTS | IBSIC)) => {
                if data.len() != 8 || data[1..4].iter().any(|b| *b > u8::from(id == IBCAC)) {
                    v.report(format!("request {id:#04x}: {}", crate::txlog::hex(data)));
                }
                Self::check_end(data, 4, "request", v);
                match id {
                    IBCAC => self.atn = true,
                    IBGTS => self.atn = false,
                    _ => {
                        // IFC: everyone unaddressed, and the adapter in charge.
                        self.meter_listens = false;
                        self.meter_talks = false;
                        self.board_listens = false;
                        self.board_talks = false;
                        self.serial_poll = false;
                        self.atn = true;
                    }
                }
                self.short_answer(id, 0, 0)
            }
            Some(IBRPP) => {
                v.report("a parallel poll: no driver asks one");
                let mut out = self.status(IBRPP, 0, 0, false).to_vec();
                out.extend_from_slice(&[0, 0, 0, 0]);
                Self::termination(&mut out);
                out
            }
            other => {
                v.report(format!("bulk request {other:02x?}: not one the adapter has"));
                self.short_answer(0, 0, 0)
            }
        }
    }

    /// The 8 byte status a vendor request answers.
    fn control_status(&self, request: u8) -> Vec<u8> {
        self.status(request, 0, 0, false).to_vec()
    }
}

impl DeviceModel for Hp3478a {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() != VENDOR || !setup.is_in() {
            v.report(format!(
                "request {:#04x} {:#04x}: the adapter's requests are vendor IN ones",
                setup.request_type, setup.request
            ));
            return Reply::Stall;
        }
        let answer = match setup.request {
            SERIAL_NUMBER_REQUEST => vec![SERIAL_NUMBER_REQUEST, 0xc4, 0xb3, 0xa2, 0x01],
            POLL_READY_REQUEST => {
                let mut a = vec![POLL_READY_REQUEST, 1, 0, 1, 0x30, 1, 0x02, 0x03, 0, 0x03, 0x96];
                a.resize(16, 0);
                a
            }
            WAIT_REQUEST => match setup.value {
                0x200 => self.control_status(WAIT_REQUEST),
                0x300 => {
                    let had = self.monitored & SRQI != 0;
                    self.monitored = setup.index;
                    if !had && self.monitored & SRQI != 0 && self.meter.rqs {
                        self.interrupt = true;
                    }
                    self.control_status(WAIT_REQUEST)
                }
                other => {
                    v.report(format!("wait request with value {other:#06x}"));
                    return Reply::Stall;
                }
            },
            STOP_REQUEST => {
                self.sending.clear();
                self.control_status(STOP_REQUEST)
            }
            other => {
                v.report(format!("vendor request {other:#04x}: not one the GPIB-USB-HS has"));
                return Reply::Stall;
            }
        };
        Reply::Data(answer[..answer.len().min(usize::from(setup.length))].to_vec())
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_OUT {
            v.report(format!("OUT transfer to endpoint {ep}: requests go to {EP_OUT}"));
            return Reply::Stall;
        }
        if self.answer.is_some() {
            v.report("a request while the answer to the one before is unread");
        }
        let srq = self.meter.rqs;
        let answer = self.bulk(data, v);
        if !srq && self.meter.rqs && self.monitored & SRQI != 0 {
            self.interrupt = true;
        }
        self.answer = Some(answer);
        Reply::ack()
    }

    fn in_transfer(&mut self, ep: u8, _max: usize, v: &mut Violations) -> Reply {
        match ep {
            EP_IN => self.answer.take().map_or(Reply::Nak, Reply::Data),
            EP_INT if self.interrupt => {
                self.interrupt = false;
                Reply::Data(self.control_status(0))
            }
            EP_INT => Reply::Nak,
            other => {
                v.report(format!("IN transfer from endpoint {other}"));
                Reply::Stall
            }
        }
    }

    fn reset(&mut self) {
        self.answer = None;
    }

    fn mark(&mut self) -> bool {
        std::mem::take(&mut self.marked)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MLA_23: u8 = 0x20 | DEFAULT_PAD;
    const MTA_23: u8 = 0x40 | DEFAULT_PAD;

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    /// A bulk request and its answer, as the kernel exchanges them.
    fn ask(m: &mut Hp3478a, data: &[u8], v: &mut Violations) -> Vec<u8> {
        assert_eq!(m.out(EP_OUT, data, v), Reply::ack());
        match m.in_transfer(EP_IN, 512, v) {
            Reply::Data(d) => d,
            other => panic!("no answer: {other:?}"),
        }
    }

    fn padded(mut out: Vec<u8>) -> Vec<u8> {
        while !out.len().is_multiple_of(4) {
            out.push(0);
        }
        out.extend_from_slice(&[TERM, 0, 0, 0]);
        out
    }

    // The requests as ni_usb_gpib.c builds them.
    fn command(bytes: &[u8]) -> Vec<u8> {
        let mut out = vec![COMMAND, !(bytes.len() as u8 - 1), 0, 0xfc];
        out.extend_from_slice(bytes);
        padded(out)
    }

    fn write(bytes: &[u8], eoi: bool) -> Vec<u8> {
        let c = (!(bytes.len() as u16 - 1)).to_le_bytes();
        let mut out = vec![WRITE, c[0], c[1], 0xfc, 0, 0, if eoi { 8 } else { 0 }, 0];
        out.extend_from_slice(bytes);
        padded(out)
    }

    fn read(n: u16) -> Vec<u8> {
        let c = (!(n - 1)).to_le_bytes();
        let out = vec![
            READ, 0, 0, 0xfc, c[0], c[1], 0, 0, REG_WRITE, 2, 0, 1, AUXMR, 0x51, 1, AUXMR, 0x55,
        ];
        padded(out)
    }

    fn simple(id: u8, arg: u8) -> Vec<u8> {
        vec![id, arg, 0, 0, TERM, 0, 0, 0]
    }

    /// `parse_board_ibrd_readback`: the data, the status, END, and the length it
    /// checks against what was read.
    fn parse_read(raw: &[u8], requested: usize) -> (Vec<u8>, u8, u16, bool) {
        let mut i = 0;
        let mut data = Vec::new();
        let mut blocks = 0;
        while raw[i] == IBRD_DATA {
            data.extend_from_slice(&raw[i + 1..i + 16]);
            i += 16;
            blocks += 1;
        }
        assert_eq!(raw[i], IBRD_STATUS);
        let ibsta = u16::from_be_bytes([raw[i + 1], raw[i + 2]]);
        let error = raw[i + 3];
        let left = u16::from_le_bytes([raw[i + 4], raw[i + 5]]).wrapping_neg();
        i += 9;
        let actual = if blocks > 0 { (blocks - 1) * 15 + usize::from(raw[i]) } else { 0 };
        i += 1;
        assert_eq!(raw[i..i + 2], [0, 0]);
        i += 2;
        assert_eq!(raw[i], REG_WRITE);
        i += 8;
        assert_eq!(raw[i..i + 4], [2, 0, 0, 0]);
        i += 4;
        assert_eq!(raw[i..i + 4], [TERM, 0, 0, 0]);
        i += 4;
        assert_eq!(i, raw.len(), "the length the driver checks");
        assert_eq!(actual, requested - usize::from(left), "actual_length");
        data.truncate(actual);
        (data, error, ibsta, ibsta & END != 0)
    }

    fn error_of(answer: &[u8]) -> u8 {
        assert_eq!(answer.len(), 12, "a status and the termination");
        answer[3]
    }

    /// Takes control, addresses as libgpib does and writes `text` with EOI.
    fn send(m: &mut Hp3478a, text: &str, v: &mut Violations) -> u8 {
        ask(m, &simple(IBCAC, 1), v);
        assert_eq!(error_of(&ask(m, &command(&[0x40, UNL, MLA_23]), v)), 0);
        ask(m, &simple(IBGTS, 0), v);
        error_of(&ask(m, &write(text.as_bytes(), true), v))
    }

    /// Addresses the meter to talk and reads up to `n` bytes.
    fn receive(m: &mut Hp3478a, n: u16, v: &mut Violations) -> (Vec<u8>, u8, bool) {
        ask(m, &simple(IBCAC, 1), v);
        ask(m, &command(&[UNL, 0x20, MTA_23]), v);
        ask(m, &simple(IBGTS, 0), v);
        let (data, error, _, end) = parse_read(&ask(m, &read(n), v), usize::from(n));
        (data, error, end)
    }

    /// The kernel's serial poll: setup, the byte, cleanup.
    fn spoll(m: &mut Hp3478a, v: &mut Violations) -> Option<u8> {
        ask(m, &simple(IBCAC, 1), v);
        ask(m, &command(&[UNL, 0x20, SPE]), v);
        ask(m, &command(&[MTA_23]), v);
        ask(m, &simple(IBGTS, 0), v);
        let (data, error, _, _) = parse_read(&ask(m, &read(1), v), 1);
        ask(m, &simple(IBCAC, 0), v);
        ask(m, &command(&[SPD, UNT]), v);
        (error == 0).then(|| data[0])
    }

    #[test]
    fn descriptors_are_the_hs() {
        let m = Hp3478a::new();
        let d = m.descriptors();
        assert_eq!((d.vendor_id(), d.product_id()), (0x3923, 0x709b));
        let eps: Vec<u8> =
            Descriptors::endpoints(&d.configs[0]).iter().map(|e| e.address).collect();
        assert_eq!(eps, [0x81, 0x02, 0x84, 0x06, 0x88]);
    }

    #[test]
    fn the_attach_requests_are_answered_as_the_driver_parses_them() {
        let mut m = Hp3478a::new();
        let mut v = Violations::default();
        let setup =
            |request, value, index| Setup { request_type: 0xc0, request, value, index, length: 16 };
        let Reply::Data(serial) = m.control(setup(0x41, 0, 0), &[], &mut v) else { panic!() };
        assert_eq!(serial.len(), 5);
        assert_eq!(serial[0], 0x41);
        let Reply::Data(ready) = m.control(setup(0x40, 0, 0), &[], &mut v) else { panic!() };
        // ni_usb_hs_wait_for_ready's checks, and ready by [6].
        assert_eq!(&ready[..11], &[0x40, 1, 0, 1, 0x30, 1, 2, 3, 0, 3, 0x96]);
        let wait = Setup { length: 8, ..setup(0x21, 0x300, 0x10fc) };
        let Reply::Data(status) = m.control(wait, &[], &mut v) else { panic!() };
        assert_eq!(status.len(), 8);
        // ni_usb_init's 26 writes come back as 26 done, in 16 bytes.
        let mut init = vec![REG_WRITE, 26, 0];
        for _ in 0..26 {
            init.extend_from_slice(&[1, AUXMR, 0x02]);
        }
        let answer = ask(&mut m, &padded(init), &mut v);
        assert_eq!(answer.len(), 16);
        assert_eq!((answer[0], answer[3], answer[8]), (REG_WRITE, 0, 26));
        assert!(v.is_empty(), "{:?}", v.drain());
        assert_eq!(m.in_transfer(EP_INT, 17, &mut v), Reply::Nak, "nothing to say");
    }

    #[test]
    fn a_reading_is_ready_on_every_second_poll_and_reads_as_13_characters() {
        let mut m = Hp3478a::new();
        let mut v = Violations::default();
        assert_eq!(send(&mut m, "T1\n", &mut v), 0);
        assert_eq!(spoll(&mut m, &mut v), Some(0));
        assert_eq!(spoll(&mut m, &mut v), Some(SPOLL_READY));
        let (data, error, end) = receive(&mut m, 1024, &mut v);
        assert_eq!((data.as_slice(), error, end), (b"+0.12345E+0\r\n".as_slice(), 0, true));
        assert_eq!(spoll(&mut m, &mut v), Some(0), "output clears data ready");
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn binary_status_is_the_cards_five_bytes() {
        let mut m = Hp3478a::new();
        let mut v = Violations::default();
        assert_eq!(send(&mut m, "B\n", &mut v), 0);
        let (data, _, end) = receive(&mut m, 1024, &mut v);
        // DCV, range 3 (3 V), 5½; internal trigger, autorange, autozero, front.
        assert_eq!((data, end), (vec![0b0010_1101, 0b0001_0111, 0, 0, 33], true));
        assert_eq!(send(&mut m, "F3R5N3Z0\n", &mut v), 0);
        assert_eq!(send(&mut m, "B\n", &mut v), 0);
        let (data, _, _) = receive(&mut m, 1024, &mut v);
        assert_eq!(data[..2], [0b0111_0111, 0b0001_0001]);
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn readings_follow_range_and_digits() {
        let mut meter = Meter::default();
        assert_eq!(meter.reading(0), "+0.12345E+0");
        assert_eq!(meter.reading(2), "-0.59257E+0");
        for (codes, n, text) in [
            ("R-2", 1, "+03.5801E-3"),
            ("R-1", 1, "+035.801E-3"),
            ("R1", 1, "+03.5801E+0"),
            ("R2", 1, "+035.801E+0"),
            ("N4", 1, "+035.800E+0"),
            ("N3", 1, "+035.800E+0"),
        ] {
            meter.code(codes.as_bytes()[0] as char, &codes[1..]).unwrap();
            assert_eq!(meter.reading(n), text, "{codes}");
        }
        let mut ohms = Meter { function: Function::Ohm2, range: 6, ..Meter::default() };
        assert_eq!(ohms.reading(2), "+0.59257E+6", "ohms have no sign");
        ohms.range = 4;
        assert_eq!(ohms.reading(0), "+01.2345E+3");
    }

    #[test]
    fn nothing_to_say_is_a_timeout_and_a_cut_reading_has_no_end() {
        let mut cut = Hp3478a::from_options(&opts(&[("cut-read", "1")])).unwrap();
        let mut v = Violations::default();
        let (data, error, end) = receive(&mut cut, 1024, &mut v);
        assert_eq!((data.as_slice(), error, end), (b"+0.12".as_slice(), 0, false));
        assert_eq!(receive(&mut cut, 1024, &mut v).1, TIMEOUT_ERROR, "silent from then on");
        assert_eq!(spoll(&mut cut, &mut v), None);
        let mut m = Hp3478a::from_options(&opts(&[("mute-read", "2")])).unwrap();
        receive(&mut m, 1024, &mut v);
        let (data, error, end) = receive(&mut m, 1024, &mut v);
        assert_eq!((data.len(), error, end), (0, TIMEOUT_ERROR, false), "muted");
        let (data, _, end) = receive(&mut m, 4, &mut v);
        assert_eq!((data.as_slice(), end), (b"-0.5".as_slice(), false), "a short read");
        let (data, _, end) = receive(&mut m, 1024, &mut v);
        assert_eq!((data.as_slice(), end), (b"9257E+0\r\n".as_slice(), true), "the rest");
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn a_board_left_in_reos_ends_a_read_at_the_eos_byte() {
        let mut m = Hp3478a::from_options(&opts(&[("dac", "10")])).unwrap();
        let mut v = Violations::default();
        assert_eq!(send(&mut m, "B\n", &mut v), 0);
        ask(&mut m, &simple(IBCAC, 1), &mut v);
        ask(&mut m, &command(&[UNL, 0x20, MTA_23]), &mut v);
        ask(&mut m, &simple(IBGTS, 0), &mut v);
        let mut request = read(1024);
        request[1] = REOS;
        request[2] = b'\n';
        let (data, _, ibsta, end) = parse_read(&ask(&mut m, &request, &mut v), 1024);
        assert_eq!((data, end), (vec![0x2d, 0x17, 0, 0, 10], true), "ends at the DAC's 10");
        assert_eq!(ibsta & END, END);
        assert_eq!(v.drain().len(), 1, "flagged: libgpib clears it");
        // With the byte not last, the rest waits for the next read.
        let mut m = Hp3478a::from_options(&opts(&[("error", "0a")])).unwrap();
        send(&mut m, "B\n", &mut v);
        ask(&mut m, &simple(IBCAC, 1), &mut v);
        ask(&mut m, &command(&[UNL, 0x20, MTA_23]), &mut v);
        ask(&mut m, &simple(IBGTS, 0), &mut v);
        let (data, _, _, end) = parse_read(&ask(&mut m, &request, &mut v), 1024);
        assert_eq!((data.len(), end), (4, true), "cut at the error byte");
        assert_eq!(receive(&mut m, 1024, &mut v).0, [33]);
    }

    #[test]
    fn a_meter_at_another_address_answers_nothing() {
        let mut m = Hp3478a::from_options(&opts(&[("pad", "9")])).unwrap();
        let mut v = Violations::default();
        assert_eq!(send(&mut m, "B\n", &mut v), NO_LISTENER_ERROR);
        let (_, error, _) = receive(&mut m, 1024, &mut v);
        assert_eq!(error, TIMEOUT_ERROR);
        assert_eq!(spoll(&mut m, &mut v), None);
        let mut gone = Hp3478a::from_options(&opts(&[("absent", "1")])).unwrap();
        ask(&mut gone, &simple(IBCAC, 1), &mut v);
        let answer = ask(&mut gone, &command(&[0x40, UNL, MLA_23]), &mut v);
        assert_eq!(error_of(&answer), NO_BUS_ERROR);
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn service_requests_show_in_the_poll_the_status_and_the_interrupt() {
        let mut m = Hp3478a::from_options(&opts(&[("power-on-srq", "1")])).unwrap();
        let mut v = Violations::default();
        let wait =
            Setup { request_type: 0xc0, request: 0x21, value: 0x300, index: 0x10fc, length: 8 };
        let Reply::Data(status) = m.control(wait, &[], &mut v) else { panic!() };
        assert_eq!(u16::from_be_bytes([status[1], status[2]]) & SRQI, SRQI);
        let Reply::Data(_) = m.in_transfer(EP_INT, 17, &mut v) else { panic!("SRQ was up") };
        assert_eq!(spoll(&mut m, &mut v), Some(SPOLL_POWER_ON | SPOLL_RQS));
        assert_eq!(spoll(&mut m, &mut v), Some(SPOLL_POWER_ON | SPOLL_READY), "RQS goes");
        // A reading with the mask's bit 0 requests service again.
        assert_eq!(send(&mut m, "M01K\n", &mut v), 0);
        let (_, _, _) = receive(&mut m, 1024, &mut v);
        spoll(&mut m, &mut v);
        assert_eq!(spoll(&mut m, &mut v), Some(SPOLL_READY | SPOLL_RQS));
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn faults_fire_once_at_their_count() {
        let mut m = Hp3478a::from_options(&opts(&[
            ("mute-poll", "2"),
            ("junk-at", "1"),
            ("overload-at", "2"),
            ("short-status", "2"),
            ("mute-status", "2"),
            ("mark-at", "2"),
        ]))
        .unwrap();
        let mut v = Violations::default();
        assert_eq!(spoll(&mut m, &mut v), Some(0));
        assert_eq!(spoll(&mut m, &mut v), None, "the second poll is mute");
        assert_eq!(receive(&mut m, 1024, &mut v).0, b"+1.2X456E+0\r\n");
        assert!(!m.mark());
        assert_eq!(receive(&mut m, 1024, &mut v).0, b"+9.99999E+9\r\n");
        assert!(m.mark() && !m.mark(), "marked once");
        send(&mut m, "B\n", &mut v);
        assert_eq!(receive(&mut m, 1024, &mut v).0.len(), 2, "short");
        send(&mut m, "B\n", &mut v);
        assert_eq!(receive(&mut m, 1024, &mut v).1, TIMEOUT_ERROR, "the second is mute");
        assert!(v.is_empty(), "{:?}", v.drain());
    }

    #[test]
    fn init_and_the_status_options_show_in_the_binary_status() {
        let m = Hp3478a::from_options(&opts(&[
            ("init", "F7N4T4"),
            ("rear", "1"),
            ("hz", "50"),
            ("cal", "1"),
            ("error", "21"),
        ]))
        .unwrap();
        assert_eq!(m.meter().binary_status(), [0b1111_1110, 0b0010_1110, 0, 0x21, 33]);
        for bad in [("init", "F8"), ("pad", "0"), ("error", "40"), ("hz", "55"), ("x", "1")] {
            assert!(Hp3478a::from_options(&opts(&[bad])).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn flags_what_no_driver_sends() {
        let mut m = Hp3478a::new();
        let mut v = Violations::default();
        // Codes the meter refuses, or has and nobody sends.
        for text in ["F9\n", "R7\n", "N6\n", "M9\n", "H1\n", "Q\n"] {
            send(&mut m, text, &mut v);
            assert_eq!(v.drain().len(), 1, "{text}");
        }
        assert_eq!(spoll(&mut m, &mut v).map(|b| b & SPOLL_SYNTAX), Some(SPOLL_SYNTAX));
        // The bus and the adapter.
        let checks: Vec<(Vec<u8>, &str)> = vec![
            (command(&[0x61]), "secondary"),
            (command(&[0x05]), "command byte"),
            (vec![0x22, 0, 0, 0, TERM, 0, 0, 0], "bulk request"),
            (vec![REG_WRITE, 1, 0, 9, 0, 0, 0, 0, TERM, 0, 0, 0], "device 9"),
            (vec![REG_WRITE, 1, 0, 1, 0, 0, 0, 0, 5, 0, 0, 0], "termination"),
            (simple(IBRPP, 0xf0), "parallel poll"),
        ];
        for (request, says) in checks {
            ask(&mut m, &request, &mut v);
            let found = v.drain();
            assert!(found.iter().any(|e| e.contains(says)), "{says}: {found:?}");
        }
        // Data with ATN on, a read with no standby, a request over an unread answer.
        ask(&mut m, &simple(IBCAC, 1), &mut v);
        ask(&mut m, &write(b"T1\n", true), &mut v);
        assert!(v.drain().iter().any(|e| e.contains("ATN on")));
        ask(&mut m, &read(1), &mut v);
        assert!(v.drain().iter().any(|e| e.contains("ATN on")));
        assert_eq!(m.out(EP_OUT, &simple(IBGTS, 0), &mut v), Reply::ack());
        assert_eq!(m.out(EP_OUT, &simple(IBGTS, 0), &mut v), Reply::ack());
        assert!(v.drain().iter().any(|e| e.contains("unread")));
        let vendor = Setup { request_type: 0xc0, request: 0x48, value: 0, index: 0, length: 16 };
        assert_eq!(m.control(vendor, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "an HS+ request");
    }

    #[test]
    fn gtl_and_a_cleared_ren_put_the_meter_back_in_local() {
        let mut m = Hp3478a::new();
        let mut v = Violations::default();
        send(&mut m, "T1\n", &mut v);
        assert!(m.remote());
        ask(&mut m, &simple(IBCAC, 1), &mut v);
        ask(&mut m, &command(&[0x40, UNL, MLA_23, GTL]), &mut v);
        assert!(!m.remote());
        send(&mut m, "T1\n", &mut v);
        ask(&mut m, &padded(vec![REG_WRITE, 1, 0, 1, AUXMR, AUX_CREN]), &mut v);
        assert!(!m.remote());
        assert!(v.is_empty(), "{:?}", v.drain());
    }
}
