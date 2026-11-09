// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Hung-Chang DSO-2100 oscilloscope behind a USS720 USB to parallel
//! port cable (a Belkin F5U002): the kernel's `uss720` makes the cable a
//! parallel port, `ppdev` makes it `/dev/parportN`, and libieee1284 (or
//! oxrok's own ppdev code) drives the scope through it, unchanged. So the
//! boundary is the kernel's, as on a real cable: what this model sees is
//! the USS720's register requests, and the scope's behaviour is decoded
//! from the parallel port lines they set.
//!
//! The cable, from `drivers/usb/misc/uss720.c` (Linux 6.12) and an
//! `lsusb -v` of an F5U002 posted to linux-kernel on 2007-09-05:
//!
//! - `050d:0002`, full speed, `bcdUSB` 1.00, `bcdDevice` 2.02, strings
//!   "Belk USB Printing Support" and "IEEE-1284 Controller", one
//!   interface with three alternate settings (printer, unidirectional;
//!   printer, bidirectional; vendor specific with bulk 1 OUT, bulk 2 IN
//!   and interrupt 3 IN). `uss720` wants exactly three and selects the
//!   third. (That posting's own cable was a revision `uss720` cannot
//!   drive, which shares the ids; the descriptors are the chip's.)
//! - `GET_1284_REGISTER`, `c0 03`, `wValue` the register in its high
//!   byte, 7 bytes back: whatever register is asked for, the answer is
//!   the block `uss720` indexes with `regindex[]`: 0 the status (DSR),
//!   1 the control (DCR), 2 the ECR, 3 the USS720 control register
//!   (register 7), 4 the data lines (register 0), 5 the EPP or ECP
//!   data, 6 register 8. `uss720` keeps the block as its soft copy, so
//!   its next control write keeps the DCR's high nibble from here.
//! - `SET_1284_REGISTER`, `40 04`, `wValue` the register in the high
//!   byte and the value in the low one, no data: register 0 the data
//!   latch, 2 the DCR, 6 the ECR, 7 the USS720 control register.
//!   `uss720`'s probe writes 7, 6 and 2 and reads 0; each `PPCLAIM`
//!   writes 2 and 6 and reads (unwaited), and `ppdev` then sets the IRQ
//!   enable bit (4) of the DCR; `PPDATADIR` sets or clears its bit 5.
//! - The DCR in the PC's sense: bits 0, 1 and 3 drive nStrobe, nAutoFd
//!   and nSelectIn inverted, bit 2 drives nInit as it is. Reading the
//!   data register while bit 5 is set gives the lines as the scope
//!   drives them; with it clear, the latch.
//!
//! The scope, from libsigrok's `hung-chang-dso-2100` (`protocol.c`, whose
//! comment documents the firmware's states, and the sequences of its
//! `write_mbox`, `read_mbox`, `check_id` and readout), in line levels
//! (`H` high):
//!
//! - nSelectIn `H`, nInit `L`, nAutoFd `H`, nStrobe `L`: the scope
//!   drives its id, `0x55`.
//! - nSelectIn `H`, nInit `H`, nAutoFd `L`: a falling nStrobe latches the
//!   data lines into the microcontroller's mailbox.
//! - nSelectIn `H`, nInit `L`, nAutoFd `L`, nStrobe `L`: the scope drives
//!   the mailbox: the state the microcontroller posted, `0` while it has
//!   not posted one ("busy").
//! - nSelectIn `L`, nInit `L`: the FPGA's readout. nAutoFd picks the ADC
//!   (`H` the second), nStrobe `L` drives the current sample and its
//!   rise steps to the next: 1000 samples (the two 500 sample subsets of
//!   state `0x03`), then the signature `01 fe 80`.
//! - Any other combination drives nothing, and the lines read `0xff`
//!   (pulled up). That and the line decoding above are this model's
//!   reading of the C driver; no schematic or firmware was found.
//!
//! The microcontroller's states and what moves them are those of
//! `protocol.c`'s comment: `0x01` idle (2 arms, 3 extracts, 4 enters
//! scroll mode), `0x21` armed (`0x99` asks whether it triggered, `0xfe`
//! forces a trigger, `0xff` aborts to `0x03`), `0x03` extracted (4 to
//! the configuration chain), `0x14` scroll mode (`0xff` to the chain),
//! and the chain `0x0f 0x0f 0x05 0x08 0x09 0x06 0x07 0x0a 0x0b 0x0c 0x0d
//! 0x0e 0x10 0x11 0x12 0x13`, each taking one value, back to `0x01`. The
//! state after a write is posted at once; how long a real one takes is
//! unknown, and `busy=N` makes each post N reads late instead.
//!
//! The samples: what a real DSO-2100 extracts from its 5000 sample
//! buffer for each step size is not documented beyond the comment (the
//! second subset starts at `5 * X * step` of the first). The model
//! takes that formula, modulo the buffer, and fills the buffer from a
//! made-up input: on A a 100 kHz triangle from -0.4 V to 0.6 V, on B a
//! 1 MHz square of +-0.3 V, at the configured rate, scaled by the
//! channel's volts/div (32 counts a division) and coupling (AC removes
//! the mean, GND gives `0x80`), clipped to a byte. Interleaved steps
//! read A on both ADCs at half a sample apart. Trigger source, slope and
//! level are checked, not used: the waveform starts where it starts.
//!
//! Options:
//!
//! - `scope=absent`: no scope on the cable; nothing drives the lines.
//! - `trigger=N`: the scope triggers at the Nth `0x99` after arming
//!   (default 3); `trigger=never` stays armed.
//! - `busy=N`: each post of a state comes N mailbox reads late (default
//!   0). C reads twice before its first timeout check, so `busy=1` keeps
//!   the transactions stable.
//! - `silent-after=N`: after the Nth mailbox write the microcontroller
//!   posts nothing more: the scope stops answering.
//! - `bad-sig=N`: the next N readouts end with a wrong signature
//!   (`01 fe 00`).
//! - `odd-state=N`: the Nth mailbox write is answered with the state
//!   `0x42`, which no firmware state has, then the scope goes on.
//! - `mark=armed` marks the first `0x99` that finds no trigger, and
//!   `mark=sample` the 500th sample of a readout, for an event armed with
//!   `model`.
//!
//! The pedantic checks: any request but the two above, a length or
//! index other than theirs, a register the C path does not use (EPP and
//! ECP data and address), a bulk or interrupt transfer, a mailbox write
//! strobed with the port reversed, a readout past the signature, and a
//! value outside what the firmware's comment and the C tables allow in
//! a state: a channel not 1 or 2, a rate index not 1 to 28, a step of
//! 0, a vdiv, coupling or trigger source code outside the tables, an edge
//! not 1 or 2, a command no state takes. `0xff` and `4` are not flagged
//! in a command state: C sends them blindly to recover after a read that
//! timed out ("Alternating between 0xff and 4 helps in all states").

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{BULK, Descriptors, DeviceModel, INTERRUPT, Reply, Setup, Speed, Violations};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hung-chang-dso-2100";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Hung-Chang DSO-2100 oscilloscope behind a USS720 USB to parallel cable \
                         (050d:0002, the kernel's uss720 and ppdev make it /dev/parportN); \
                         scope=absent, trigger=N|never, busy=N, silent-after=N, bad-sig=N, \
                         odd-state=N, mark=armed|sample";

/// The cable's vendor id (Belkin).
pub const VENDOR_ID: u16 = 0x050d;
/// The cable's product id (F5U002).
pub const PRODUCT_ID: u16 = 0x0002;

/// `GET_1284_REGISTER`.
pub const GET_REGISTER: u8 = 3;
/// `SET_1284_REGISTER`.
pub const SET_REGISTER: u8 = 4;

/// The DCR bits, in the PC's sense.
pub const DCR_STROBE: u8 = 0x01;
/// nAutoFd, inverted.
pub const DCR_AUTOFD: u8 = 0x02;
/// nInit, as it is.
pub const DCR_INIT: u8 = 0x04;
/// nSelectIn, inverted.
pub const DCR_SELECTIN: u8 = 0x08;
/// The data lines driven by the peripheral.
pub const DCR_REVERSE: u8 = 0x20;

/// The scope's id byte.
pub const ID: u8 = 0x55;
/// The readout's signature after the 1000 samples.
pub const SIGNATURE: [u8; 3] = [0x01, 0xfe, 0x80];
/// Samples a readout holds before the signature.
pub const READOUT: usize = 1000;
/// The capture buffer of an ADC.
pub const BUFFER: usize = 5000;
/// A state no firmware state has, for `odd-state=`.
pub const ODD_STATE: u8 = 0x42;

/// The sample rates of the C table, by index.
pub const SAMPLERATES: [u64; 28] = [
    100_000_000,
    50_000_000,
    25_000_000,
    20_000_000,
    10_000_000,
    5_000_000,
    2_500_000,
    2_000_000,
    1_000_000,
    500_000,
    250_000,
    200_000,
    100_000,
    50_000,
    25_000,
    20_000,
    10_000,
    5_000,
    2_500,
    2_000,
    1_000,
    500,
    250,
    200,
    100,
    50,
    25,
    20,
];

/// The vdiv codes of a channel byte (bits 0, 1, 4, 5), as the C table.
const VDIV_CODES: [u8; 9] = [0x01, 0x02, 0x03, 0x21, 0x22, 0x23, 0x31, 0x32, 0x33];
/// The coupling codes (bits 2, 3): DC, AC, GND.
const COUPLING_CODES: [u8; 3] = [0x00, 0x08, 0x04];
/// The trigger source codes of the first channel byte (bits 6, 7).
const SOURCE_CODES: [u8; 3] = [0x00, 0x80, 0x40];

/// The firmware's states, as the mailbox shows them; the two `0x0f`
/// states of the chain are told apart by [`Uc::first_channel`].
mod state {
    pub const IDLE: u8 = 0x01;
    pub const EXTRACTED: u8 = 0x03;
    pub const CHANNEL: u8 = 0x0f;
    pub const RATE: u8 = 0x05;
    pub const STEP2: u8 = 0x08;
    pub const STEP1: u8 = 0x09;
    pub const CCTL0: u8 = 0x06;
    pub const CCTL1: u8 = 0x07;
    pub const SHIFT: u8 = 0x0a;
    pub const EDGE: u8 = 0x0b;
    pub const POS0: u8 = 0x0c;
    pub const POS1: u8 = 0x0d;
    pub const TLEVEL: u8 = 0x0e;
    pub const OFFSET0: u8 = 0x10;
    pub const GAIN0: u8 = 0x11;
    pub const OFFSET1: u8 = 0x12;
    pub const GAIN1: u8 = 0x13;
    pub const SCROLL: u8 = 0x14;
    pub const ARMED: u8 = 0x21;
}

/// When the trigger comes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    /// At the Nth `0x99` after arming.
    After(u32),
    /// Never.
    Never,
}

/// What `mark=` marks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mark {
    /// The first `0x99` that finds no trigger.
    Armed,
    /// The 500th sample of a readout.
    Sample,
}

/// The model's options.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Options {
    /// A scope on the cable.
    pub present: bool,
    /// When the trigger comes.
    pub trigger: Trigger,
    /// Mailbox reads each post comes late by.
    pub busy: u32,
    /// The mailbox write after which nothing is posted.
    pub silent_after: Option<u32>,
    /// Readouts that end with a wrong signature.
    pub bad_sig: u32,
    /// The mailbox write answered with [`ODD_STATE`].
    pub odd_state: Option<u32>,
    /// What to mark.
    pub mark: Option<Mark>,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            present: true,
            trigger: Trigger::After(3),
            busy: 0,
            silent_after: None,
            bad_sig: 0,
            odd_state: None,
            mark: None,
        }
    }
}

fn number(key: &str, value: &str, max: u32) -> Result<u32> {
    value
        .parse::<u32>()
        .ok()
        .filter(|n| *n <= max)
        .ok_or_else(|| Error::Invalid(format!("{NAME}: {key}={value}: a number up to {max}")))
}

/// The options from `-o KEY=VALUE` pairs.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn parse_options(options: &[(String, String)]) -> Result<Options> {
    let mut o = Options::default();
    for (k, v) in options {
        match k.as_str() {
            "scope" => match v.as_str() {
                "absent" => o.present = false,
                "present" => o.present = true,
                _ => return Err(Error::Invalid(format!("{NAME}: scope={v}: absent or present"))),
            },
            "trigger" => {
                o.trigger = if v == "never" {
                    Trigger::Never
                } else {
                    Trigger::After(number(k, v, 100_000)?.max(1))
                };
            }
            "busy" => o.busy = number(k, v, 1000)?,
            "silent-after" => o.silent_after = Some(number(k, v, 1_000_000)?),
            "bad-sig" => o.bad_sig = number(k, v, 1000)?,
            "odd-state" => o.odd_state = Some(number(k, v, 1_000_000)?.max(1)),
            "mark" => {
                o.mark = Some(match v.as_str() {
                    "armed" => Mark::Armed,
                    "sample" => Mark::Sample,
                    _ => return Err(Error::Invalid(format!("{NAME}: mark={v}: armed or sample"))),
                });
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(o)
}

/// A model from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(Dso2100::new(parse_options(options)?)))
}

/// The cable's descriptors.
pub fn descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 1, [7, 1, 1], 0));
    body.extend_from_slice(&descriptor::endpoint(0x01, BULK, 64, 0));
    body.extend_from_slice(&descriptor::interface(0, 1, 2, [7, 1, 2], 0));
    body.extend_from_slice(&descriptor::endpoint(0x01, BULK, 64, 0));
    body.extend_from_slice(&descriptor::endpoint(0x82, BULK, 64, 0));
    body.extend_from_slice(&descriptor::interface(0, 2, 3, [0xff, 0, 0xff], 0));
    body.extend_from_slice(&descriptor::endpoint(0x01, BULK, 64, 0));
    body.extend_from_slice(&descriptor::endpoint(0x82, BULK, 64, 0));
    body.extend_from_slice(&descriptor::endpoint(0x83, INTERRUPT, 4, 1));
    let device = DeviceDescriptor {
        usb: 0x0100,
        class: [0, 0, 0],
        max_packet0: 8,
        vendor: VENDOR_ID,
        product: PRODUCT_ID,
        device: 0x0202,
        strings: [1, 2, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::Full,
        device: device.bytes(),
        configs: vec![descriptor::configuration(1, 0, 0x80, 100, &body)],
        strings: vec!["Belk USB Printing Support".to_owned(), "IEEE-1284 Controller".to_owned()],
    }
}

/// The readout steps of `protocol.c`, `readout_steps[]`: whether the
/// step interleaves, by its first step size (the C sends `step1 + 1`).
fn interleaves(step1: u8) -> bool {
    matches!(step1, 212 | 157 | 247 | 232 | 230)
}

/// The line levels the DCR sets: four lines, so four bools.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Lines {
    strobe: bool,
    autofd: bool,
    init: bool,
    selectin: bool,
}

impl Lines {
    fn of(dcr: u8) -> Lines {
        Lines {
            strobe: dcr & DCR_STROBE == 0,
            autofd: dcr & DCR_AUTOFD == 0,
            init: dcr & DCR_INIT != 0,
            selectin: dcr & DCR_SELECTIN == 0,
        }
    }

    fn mailbox_write(self) -> bool {
        self.selectin && self.init && !self.autofd
    }

    fn mailbox_read(self) -> bool {
        self.selectin && !self.init && !self.autofd && !self.strobe
    }

    fn id(self) -> bool {
        self.selectin && !self.init && self.autofd && !self.strobe
    }

    fn readout(self) -> bool {
        !self.selectin && !self.init
    }
}

/// The scope's configuration, as the chain set it.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Config {
    /// The ADC a readout reads, 1 or 2.
    channel: u8,
    /// Index into [`SAMPLERATES`].
    rate: usize,
    step1: u8,
    step2: u8,
    shift: u8,
    cctl: [u8; 2],
}

impl Default for Config {
    fn default() -> Config {
        Config { channel: 1, rate: 0, step1: 1, step2: 1, shift: 100, cctl: [0x31, 0x31] }
    }
}

/// The volts a division of a channel byte selects, as the C's
/// `res_array[]` over `relays[]`.
fn volts_per_div(cctl: u8) -> f64 {
    const RES: [f64; 4] = [0.5, 1.0, 2.0, 5.0];
    const RELAYS: [f64; 4] = [100.0, 10.0, 10.0, 1.0];
    RES[usize::from(cctl & 3)] / RELAYS[usize::from((cctl >> 4) & 3)]
}

/// The made-up input of channel `ch` (0 A, 1 B) at time `t` seconds,
/// in volts, with its mean.
fn input(ch: usize, t: f64) -> (f64, f64) {
    if ch == 0 {
        let phase = (t * 100_000.0).fract();
        let tri = if phase < 0.5 { phase * 2.0 } else { 2.0 - phase * 2.0 };
        (tri - 0.4, 0.1)
    } else {
        let phase = (t * 1_000_000.0).fract();
        (if phase < 0.5 { 0.3 } else { -0.3 }, 0.0)
    }
}

/// A sample byte of channel `ch` at time `t` under channel byte `cctl`.
fn sample(ch: usize, t: f64, cctl: u8) -> u8 {
    let (v, mean) = input(ch, t);
    let v = match cctl & 0x0c {
        0x04 => return 0x80,
        0x08 => v - mean,
        _ => v,
    };
    let counts = (v * 32.0 / volts_per_div(cctl)).round();
    (128.0 + counts).clamp(0.0, 255.0) as u8
}

/// The 1000 samples a readout of `cfg` gives from ADC `adc` (1 or 2).
fn readout(cfg: &Config, adc: u8) -> Vec<u8> {
    let rate = SAMPLERATES[cfg.rate] as f64;
    let interleaved = interleaves(cfg.step1);
    let step1 = usize::from(cfg.step1.max(1));
    let step2 = usize::from(cfg.step2.max(1));
    let start2 = 5 * usize::from(cfg.shift) * step1;
    let at = |i: usize| -> u8 {
        let index = if i < READOUT / 2 { i * step1 } else { start2 + (i - READOUT / 2) * step2 };
        let index = index % BUFFER;
        if interleaved {
            let t = (2 * index + usize::from(adc == 2)) as f64 / (2.0 * rate);
            sample(0, t, cfg.cctl[0])
        } else {
            let ch = usize::from(adc == 2);
            sample(ch, index as f64 / rate, cfg.cctl[ch])
        }
    };
    (0..READOUT).map(at).collect()
}

/// The microcontroller behind the mailbox.
#[derive(Clone, Debug)]
struct Uc {
    state: u8,
    /// In the chain, at the first of the two `0x0f` states.
    first_channel: bool,
    /// The first channel value, for the second to match.
    channel_value: u8,
    cfg: Config,
    /// The posted state, or `None` while nothing is posted.
    posted: Option<u8>,
    /// Mailbox reads before the post shows.
    late: u32,
    /// `0x99`s left before the trigger.
    checks_left: u32,
    /// A forced trigger is pending.
    forced: bool,
    /// Mailbox writes so far.
    writes: u32,
}

impl Uc {
    fn new(o: &Options) -> Uc {
        Uc {
            state: state::IDLE,
            first_channel: false,
            channel_value: 0,
            cfg: Config::default(),
            posted: Some(state::IDLE),
            late: 0,
            checks_left: 0,
            forced: false,
            writes: 0,
            // The trigger count is set at each arming.
        }
        .armed_with(o)
    }

    fn armed_with(mut self, o: &Options) -> Uc {
        self.checks_left = match o.trigger {
            Trigger::After(n) => n,
            Trigger::Never => u32::MAX,
        };
        self
    }
}

/// The emulated cable and scope.
pub struct Dso2100 {
    descriptors: Descriptors,
    options: Options,
    /// The data latch, register 0.
    data: u8,
    dcr: u8,
    ecr: u8,
    ucr: u8,
    uc: Uc,
    /// The readout: its samples, the ADC and the position.
    samples: Vec<u8>,
    adc: u8,
    pos: usize,
    readout_ready: bool,
    bad_sig_left: u32,
    marked: bool,
}

impl Dso2100 {
    /// A cable with a scope as `options` say.
    pub fn new(options: Options) -> Dso2100 {
        let uc = Uc::new(&options);
        Dso2100 {
            descriptors: descriptors(),
            data: 0,
            dcr: 0,
            ecr: 0,
            ucr: 0,
            samples: Vec::new(),
            adc: 1,
            pos: 0,
            readout_ready: false,
            bad_sig_left: options.bad_sig,
            marked: false,
            uc,
            options,
        }
    }

    fn lines(&self) -> Lines {
        Lines::of(self.dcr)
    }

    /// What the data lines read as.
    fn data_lines(&mut self) -> u8 {
        if self.dcr & DCR_REVERSE == 0 {
            return self.data;
        }
        if !self.options.present {
            return 0xff;
        }
        let l = self.lines();
        if l.id() {
            ID
        } else if l.mailbox_read() {
            self.mailbox()
        } else if l.readout() && !l.strobe {
            self.current_sample()
        } else {
            0xff
        }
    }

    /// One read of the mailbox.
    fn mailbox(&mut self) -> u8 {
        if self.uc.late > 0 {
            self.uc.late -= 1;
            return 0;
        }
        self.uc.posted.unwrap_or(0)
    }

    fn current_sample(&mut self) -> u8 {
        if self.samples.is_empty() {
            // The ADC is the one nAutoFd picked before the first strobe.
            self.samples = readout(&self.uc.cfg, self.adc);
        }
        match self.pos {
            p if p < READOUT => self.samples[p],
            p if p < READOUT + 3 => {
                let mut s = SIGNATURE[p - READOUT];
                if p == READOUT + 2 && self.bad_sig_left > 0 {
                    s = 0x00;
                }
                s
            }
            _ => 0xff,
        }
    }

    /// The DCR changes: edges on nStrobe act.
    fn set_dcr(&mut self, dcr: u8, v: &mut Violations) {
        let before = self.lines();
        self.dcr = dcr;
        let after = self.lines();
        if !self.options.present {
            return;
        }
        if before.strobe && !after.strobe && after.mailbox_write() {
            if self.dcr & DCR_REVERSE != 0 {
                v.report(format!(
                    "mailbox strobe with the port reversed (DCR {dcr:#04x}): the scope latches \
                     lines nobody drives"
                ));
            }
            let value = self.data;
            self.mailbox_write(value, v);
        }
        if after.readout() && after.autofd != before.autofd && before.readout() {
            // The ADC is picked before the strobes start; C sets it with
            // nStrobe high, so a change mid-readout is not one it makes.
            if self.pos > 0 && self.pos < READOUT {
                v.report(format!("readout ADC changed at sample {}", self.pos));
            }
        }
        if after.readout() {
            self.adc = if after.autofd { 2 } else { 1 };
        }
        if before.readout() && after.readout() && !before.strobe && after.strobe {
            self.step_readout(v);
        }
    }

    fn step_readout(&mut self, v: &mut Violations) {
        if !self.readout_ready {
            v.report("readout strobe before the scope extracted a capture (state 0x03)");
            return;
        }
        if self.samples.is_empty() {
            self.samples = readout(&self.uc.cfg, self.adc);
        }
        self.pos += 1;
        if self.pos == READOUT + 3 && self.bad_sig_left > 0 {
            self.bad_sig_left -= 1;
        }
        if self.pos > READOUT + 3 {
            v.report(format!("readout strobe {} past the signature", self.pos));
        }
        if self.pos == READOUT / 2 && self.options.mark == Some(Mark::Sample) {
            self.marked = true;
        }
    }

    /// The microcontroller takes `value` from the mailbox.
    fn mailbox_write(&mut self, value: u8, v: &mut Violations) {
        let uc = &mut self.uc;
        uc.writes += 1;
        if self.options.silent_after.is_some_and(|n| uc.writes > n) {
            uc.posted = None;
            return;
        }
        let next = self.transition(value, v);
        let uc = &mut self.uc;
        uc.state = next;
        uc.posted = Some(if self.options.odd_state == Some(uc.writes) { ODD_STATE } else { next });
        uc.late = self.options.busy;
    }

    fn command(&self, value: u8, takes: &[u8], v: &mut Violations) {
        if !takes.contains(&value) && value != 0xff && value != 0x04 {
            v.report(format!(
                "mailbox {value:#04x} in state {:#04x}, which takes {}",
                self.uc.state,
                takes.iter().map(|t| format!("{t:#04x}")).collect::<Vec<_>>().join(", ")
            ));
        }
    }

    fn extract(&mut self) -> u8 {
        self.readout_ready = true;
        self.pos = 0;
        self.samples.clear();
        state::EXTRACTED
    }

    fn check_cctl(which: usize, value: u8, v: &mut Violations) {
        if !VDIV_CODES.contains(&(value & 0x33)) {
            v.report(format!("channel {which} byte {value:#04x}: vdiv code not in the table"));
        }
        if !COUPLING_CODES.contains(&(value & 0x0c)) {
            v.report(format!("channel {which} byte {value:#04x}: coupling code not in the table"));
        }
        let top = value & 0xc0;
        if which == 0 && !SOURCE_CODES.contains(&top) {
            v.report(format!("channel 0 byte {value:#04x}: trigger source not in the table"));
        }
        if which == 1 && top != 0 {
            v.report(format!(
                "channel 1 byte {value:#04x}: a TV sync trigger, which the C never selects"
            ));
        }
    }

    /// The next state from `value` in the current one: a state machine
    /// of the firmware's comment, one arm a state.
    #[allow(clippy::too_many_lines)]
    fn transition(&mut self, value: u8, v: &mut Violations) -> u8 {
        let s = self.uc.state;
        match s {
            state::IDLE => match value {
                2 => {
                    self.uc = self.uc.clone().armed_with(&self.options);
                    self.uc.forced = false;
                    self.readout_ready = false;
                    state::ARMED
                }
                3 => self.extract(),
                4 => state::SCROLL,
                _ => {
                    self.command(value, &[2, 3, 4], v);
                    s
                }
            },
            state::ARMED => match value {
                0x99 => {
                    if self.uc.forced || self.uc.checks_left <= 1 {
                        self.uc.forced = false;
                        self.extract()
                    } else {
                        self.uc.checks_left -= 1;
                        if self.options.mark == Some(Mark::Armed) {
                            self.marked = true;
                        }
                        s
                    }
                }
                0xfe => {
                    self.uc.forced = true;
                    s
                }
                0xff => self.extract(),
                _ => {
                    self.command(value, &[0x99, 0xfe, 0xff], v);
                    s
                }
            },
            state::EXTRACTED => {
                if value == 4 {
                    self.readout_ready = false;
                    self.uc.first_channel = true;
                    state::CHANNEL
                } else {
                    self.command(value, &[4], v);
                    s
                }
            }
            state::SCROLL => {
                if value == 0xff {
                    self.uc.first_channel = true;
                    state::CHANNEL
                } else {
                    self.command(value, &[0xff], v);
                    s
                }
            }
            state::CHANNEL => {
                if value != 1 && value != 2 {
                    v.report(format!("channel {value:#04x}: the firmware takes 1 or 2"));
                }
                if self.uc.first_channel {
                    self.uc.first_channel = false;
                    self.uc.channel_value = value;
                    state::CHANNEL
                } else {
                    // Values that do not match are discarded; the chain
                    // goes on either way.
                    if value == self.uc.channel_value && (value == 1 || value == 2) {
                        self.uc.cfg.channel = value;
                    }
                    state::RATE
                }
            }
            state::RATE => {
                if (1..=28).contains(&value) {
                    self.uc.cfg.rate = usize::from(value - 1);
                } else {
                    v.report(format!("rate {value:#04x}: the firmware takes 1 to 28"));
                }
                state::STEP2
            }
            state::STEP2 => {
                self.step(value, v, |c, s| c.step2 = s);
                state::STEP1
            }
            state::STEP1 => {
                self.step(value, v, |c, s| c.step1 = s);
                state::CCTL0
            }
            state::CCTL0 => {
                Self::check_cctl(0, value, v);
                self.uc.cfg.cctl[0] = value;
                state::CCTL1
            }
            state::CCTL1 => {
                Self::check_cctl(1, value, v);
                self.uc.cfg.cctl[1] = value;
                state::SHIFT
            }
            state::SHIFT => {
                self.step(value, v, |c, s| c.shift = s);
                state::EDGE
            }
            state::EDGE => {
                if value != 1 && value != 2 {
                    v.report(format!("edge {value:#04x}: the firmware takes 1 or 2"));
                }
                state::POS0
            }
            state::POS0 => state::POS1,
            state::POS1 => state::TLEVEL,
            state::TLEVEL => state::OFFSET0,
            state::OFFSET0 => state::GAIN0,
            state::GAIN0 => state::OFFSET1,
            state::OFFSET1 => state::GAIN1,
            state::GAIN1 => state::IDLE,
            _ => s,
        }
    }

    /// A step size or shift, sent plus one.
    fn step(&mut self, value: u8, v: &mut Violations, set: impl FnOnce(&mut Config, u8)) {
        if value == 0 {
            v.report(format!("step value 0 in state {:#04x}: the firmware takes N + 1", self.uc.state));
            return;
        }
        set(&mut self.uc.cfg, value - 1);
    }

    /// The register block `GET_1284_REGISTER` answers.
    fn block(&mut self) -> Vec<u8> {
        let data = self.data_lines();
        vec![0x00, self.dcr, self.ecr, self.ucr, data, 0x00, 0x00]
    }
}

impl DeviceModel for Dso2100 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        let reg = (setup.value >> 8) as u8;
        match (setup.request_type, setup.request) {
            (0xc0, GET_REGISTER) => {
                if setup.length != 7 || setup.index != 0 || setup.value & 0xff != 0 {
                    v.report(format!(
                        "GET_1284_REGISTER value {:#06x} index {:#06x} length {}: uss720 asks \
                         with the register in the high byte, index 0, 7 bytes",
                        setup.value, setup.index, setup.length
                    ));
                }
                if matches!(reg, 3..=5) || reg > 8 {
                    v.report(format!("GET_1284_REGISTER of register {reg}, which no C path reads"));
                }
                Reply::Data(self.block())
            }
            (0x40, SET_REGISTER) => {
                if setup.length != 0 || setup.index != 0 {
                    v.report(format!(
                        "SET_1284_REGISTER index {:#06x} length {}: uss720 sends index 0, no data",
                        setup.index, setup.length
                    ));
                }
                let val = setup.value as u8;
                match reg {
                    0 => self.data = val,
                    2 => self.set_dcr(val, v),
                    6 => self.ecr = val,
                    7 => self.ucr = val,
                    _ => {
                        v.report(format!(
                            "SET_1284_REGISTER of register {reg} ({val:#04x}), which no C path writes"
                        ));
                    }
                }
                Reply::ack()
            }
            _ => {
                v.report(format!(
                    "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one uss720 sends",
                    setup.request_type, setup.request, setup.value, setup.index
                ));
                Reply::Stall
            }
        }
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "OUT transfer of {} bytes to endpoint {ep}: the C path uses the registers only",
            data.len()
        ));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, _max: usize, v: &mut Violations) -> Reply {
        v.report(format!("IN transfer on endpoint {ep}: the C path uses the registers only"));
        Reply::Stall
    }

    fn reset(&mut self) {
        // A bus reset leaves the scope alone; the cable's registers stay
        // as the kernel set them, and uss720 restores them at each claim.
    }

    fn mark(&mut self) -> bool {
        std::mem::take(&mut self.marked)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// libieee1284's control bits, as `protocol.c` writes them.
    const NSTROBE: u8 = 0x01;
    const NAUTOFD: u8 = 0x02;
    const NINIT: u8 = 0x04;
    const NSELECTIN: u8 = 0x08;

    /// The host side as libieee1284 over ppdev over uss720 drives it.
    struct Host {
        dev: Dso2100,
        v: Violations,
        dcr: u8,
    }

    impl Host {
        fn new(o: Options) -> Host {
            let mut h = Host { dev: Dso2100::new(o), v: Violations::default(), dcr: 0x0c };
            h.set(6, 0x30);
            h.set(2, 0x0c);
            h
        }

        fn set(&mut self, reg: u8, val: u8) {
            let s = Setup {
                request_type: 0x40,
                request: 4,
                value: u16::from(reg) << 8 | u16::from(val),
                index: 0,
                length: 0,
            };
            assert_eq!(self.dev.control(s, &[], &mut self.v), Reply::ack());
        }

        fn get(&mut self, reg: u8) -> Vec<u8> {
            let s =
                Setup { request_type: 0xc0, request: 3, value: u16::from(reg) << 8, index: 0, length: 7 };
            match self.dev.control(s, &[], &mut self.v) {
                Reply::Data(d) => {
                    self.dcr = d[1];
                    d
                }
                other => panic!("{other:?}"),
            }
        }

        /// `ieee1284_write_control`: libieee1284 inverts, uss720 keeps
        /// the high nibble.
        fn control(&mut self, ct: u8) {
            let d = ((ct & 0x0f) ^ 0x0b) | (self.dcr & 0xf0);
            self.dcr = d;
            self.set(2, d);
        }

        fn dir(&mut self, reverse: bool) {
            let d = if reverse { self.dcr | 0x20 } else { self.dcr & !0x20 };
            self.dcr = d;
            self.set(2, d);
        }

        fn read(&mut self) -> u8 {
            self.get(0)[4]
        }

        fn write_mbox(&mut self, val: u8) {
            self.control(NSTROBE | NINIT | NSELECTIN);
            self.dir(false);
            self.set(0, val);
            self.control(NINIT | NSELECTIN);
            self.control(NSTROBE | NINIT | NSELECTIN);
            self.dir(true);
            self.control(NSTROBE | NAUTOFD | NINIT | NSELECTIN);
        }

        /// `read_mbox` with a count of reads for a timeout.
        fn read_mbox(&mut self, tries: u32) -> u8 {
            self.control(NSTROBE | NSELECTIN);
            self.control(NSELECTIN);
            let mut val = 0;
            for _ in 0..tries {
                if self.read() != 0 {
                    val = self.read();
                    break;
                }
            }
            self.control(NSTROBE | NSELECTIN);
            self.control(NSTROBE | NAUTOFD | NINIT | NSELECTIN);
            val
        }

        fn check_id(&mut self) -> bool {
            self.dir(true);
            self.control(NSTROBE | NAUTOFD | NSELECTIN);
            self.control(NAUTOFD | NSELECTIN);
            let ok = self.read() == 0x55;
            self.control(NSTROBE | NAUTOFD | NSELECTIN);
            self.dir(false);
            ok
        }

        /// One `move_to` step: the reply to `c`.
        fn send(&mut self, c: u8) -> u8 {
            self.write_mbox(c);
            self.read_mbox(4)
        }

        /// The chain with C's defaults, from state 0x03 or 0x14's next.
        fn chain(&mut self, channel: u8, rate: u8, step1: u8, shift: u8) -> u8 {
            let mut last = 0;
            for c in [
                channel, channel, rate, 2, step1, 0x31, 0x31, shift, 1, 0x80, 0x80, 0x80, 0x80, 0x80,
                0x80, 0x80,
            ] {
                last = self.send(c);
            }
            last
        }

        fn readout(&mut self, ctrl: u8) -> (Vec<u8>, Vec<u8>) {
            self.control(ctrl);
            let mut samples = Vec::new();
            for _ in 0..READOUT {
                self.control(ctrl & !NSTROBE);
                samples.push(self.read());
                self.control(ctrl);
            }
            let mut sig = Vec::new();
            for _ in 0..3 {
                self.control(ctrl & !NSTROBE);
                sig.push(self.read());
                self.control(ctrl);
            }
            (samples, sig)
        }
    }

    #[test]
    fn descriptors_have_three_alternate_settings() {
        let d = descriptors();
        let intfs = Descriptors::interfaces(&d.configs[0]);
        assert_eq!(intfs.len(), 3);
        assert_eq!(intfs[2], [0, 2, 0xff, 0, 0xff]);
        assert_eq!(Descriptors::endpoints(&d.configs[0]).len(), 6);
        assert_eq!(d.vendor_id(), 0x050d);
        assert_eq!(d.product_id(), 0x0002);
    }

    #[test]
    fn id_and_idle_state() {
        let mut h = Host::new(Options::default());
        assert!(h.check_id());
        h.dir(true);
        assert_eq!(h.read_mbox(2), 0x01);
        assert!(h.v.is_empty(), "{:?}", h.v.drain());
        let mut absent = Host::new(Options { present: false, ..Options::default() });
        assert!(!absent.check_id());
        absent.dir(true);
        assert_eq!(absent.read_mbox(2), 0xff, "nothing drives the lines");
    }

    #[test]
    fn forward_reads_the_latch() {
        let mut h = Host::new(Options::default());
        h.set(0, 0x5a);
        assert_eq!(h.read(), 0x5a);
    }

    #[test]
    fn a_capture_as_c_drives_it() {
        let mut h = Host::new(Options::default());
        assert!(h.check_id());
        h.dir(true);
        // config_commit from idle: 4 to scroll mode, 0xff to the chain.
        assert_eq!(h.read_mbox(2), 0x01);
        assert_eq!(h.send(4), 0x14);
        assert_eq!(h.send(0xff), 0x0f);
        assert_eq!(h.chain(1, 1, 2, 101), 0x01);
        // Arm, and poll: two misses, then the trigger.
        assert_eq!(h.send(2), 0x21);
        assert_eq!(h.send(0x99), 0x21);
        assert_eq!(h.send(0x99), 0x21);
        assert_eq!(h.send(0x99), 0x03);
        let (samples, sig) = h.readout(NSTROBE);
        assert_eq!(sig, SIGNATURE);
        assert_eq!(samples, readout(&Config::default(), 1));
        assert!(samples.iter().any(|&s| s != samples[0]), "a waveform");
        // Back to idle through the chain, and the stop.
        assert_eq!(h.send(4), 0x0f);
        assert_eq!(h.chain(1, 1, 2, 101), 0x01);
        assert!(h.v.is_empty(), "{:?}", h.v.drain());
    }

    #[test]
    fn channel_b_and_rate_reach_the_samples() {
        let mut h = Host::new(Options::default());
        h.dir(true);
        assert_eq!(h.send(4), 0x14);
        assert_eq!(h.send(0xff), 0x0f);
        assert_eq!(h.chain(2, 5, 2, 101), 0x01);
        assert_eq!(h.send(3), 0x03);
        let (samples, sig) = h.readout(NSTROBE | NAUTOFD);
        assert_eq!(sig, SIGNATURE);
        let cfg = Config { channel: 2, rate: 4, ..Config::default() };
        assert_eq!(samples, readout(&cfg, 2));
        assert!(samples.contains(&(0x80 + 10)) && samples.contains(&(0x80 - 10)), "+-0.3 V at 1 V/div");
        assert!(h.v.is_empty(), "{:?}", h.v.drain());
    }

    #[test]
    fn vdiv_and_coupling_scale_the_samples() {
        assert!((volts_per_div(0x01) - 0.01).abs() < 1e-12);
        assert!((volts_per_div(0x33) - 5.0).abs() < 1e-12);
        assert_eq!(sample(1, 0.0, 0x31), 0x80 + 10);
        assert_eq!(sample(1, 0.0, 0x35), 0x80, "GND");
        assert_eq!(sample(1, 0.0, 0x33), 0x80 + 2);
        assert_eq!(sample(1, 0.0, 0x01), 0xff, "clipped");
        assert_eq!(sample(0, 0.0, 0x31), 0x80 - 13);
        assert_eq!(sample(0, 0.0, 0x39), 0x80 - 16, "AC takes the mean off");
    }

    #[test]
    fn trigger_never_and_forced_and_abort() {
        let mut h = Host::new(Options { trigger: Trigger::Never, ..Options::default() });
        h.dir(true);
        assert_eq!(h.send(2), 0x21);
        for _ in 0..10 {
            assert_eq!(h.send(0x99), 0x21);
        }
        assert_eq!(h.send(0xfe), 0x21);
        assert_eq!(h.send(0x99), 0x03, "forced");
        assert_eq!(h.send(4), 0x0f);
        assert_eq!(h.chain(1, 1, 2, 101), 0x01);
        assert_eq!(h.send(2), 0x21);
        assert_eq!(h.send(0xff), 0x03, "aborted");
        assert!(h.v.is_empty(), "{:?}", h.v.drain());
    }

    #[test]
    fn busy_and_silence() {
        let mut h = Host::new(Options { busy: 1, ..Options::default() });
        h.dir(true);
        h.write_mbox(4);
        h.control(NSTROBE | NSELECTIN);
        h.control(NSELECTIN);
        assert_eq!(h.read(), 0);
        assert_eq!(h.read(), 0x14);
        assert_eq!(h.read(), 0x14, "a read does not take the post");
        let mut s = Host::new(Options { silent_after: Some(1), ..Options::default() });
        s.dir(true);
        assert_eq!(s.send(4), 0x14);
        assert_eq!(s.send(0xff), 0, "silent");
        assert_eq!(s.read_mbox(10), 0);
    }

    #[test]
    fn odd_state_and_bad_signature() {
        let mut h = Host::new(Options { odd_state: Some(2), bad_sig: 1, ..Options::default() });
        h.dir(true);
        assert_eq!(h.send(2), 0x21);
        assert_eq!(h.send(0x99), ODD_STATE);
        assert_eq!(h.send(0xff), 0x03);
        let (_, sig) = h.readout(NSTROBE);
        assert_eq!(sig, [0x01, 0xfe, 0x00]);
        assert_eq!(h.send(4), 0x0f);
        assert_eq!(h.chain(1, 1, 2, 101), 0x01);
        assert_eq!(h.send(3), 0x03);
        let (_, sig) = h.readout(NSTROBE);
        assert_eq!(sig, SIGNATURE, "one bad readout only");
        assert!(h.v.is_empty(), "{:?}", h.v.drain());
    }

    #[test]
    fn interleaved_steps_read_a_on_both_adcs() {
        let cfg = Config { step1: 212, shift: 41, rate: 4, ..Config::default() };
        let a = readout(&cfg, 1);
        let b = readout(&cfg, 2);
        assert_ne!(a, b);
        assert_eq!(a.len(), READOUT);
    }

    #[test]
    fn marks() {
        let mut h = Host::new(Options { mark: Some(Mark::Armed), ..Options::default() });
        h.dir(true);
        h.send(2);
        assert!(!h.dev.mark());
        h.send(0x99);
        assert!(h.dev.mark());
        assert!(!h.dev.mark(), "once");
        let mut s = Host::new(Options { mark: Some(Mark::Sample), ..Options::default() });
        s.dir(true);
        s.send(3);
        s.control(NSTROBE);
        for _ in 0..READOUT / 2 {
            s.control(0);
            s.control(NSTROBE);
        }
        assert!(s.dev.mark());
    }

    #[test]
    fn violations_fire() {
        let mut h = Host::new(Options::default());
        h.dir(true);
        // A command no state takes.
        h.send(9);
        assert_eq!(h.v.drain().len(), 1);
        // Bad chain values.
        assert_eq!(h.send(4), 0x14);
        assert_eq!(h.send(0xff), 0x0f);
        for (c, n) in [(3, 1), (3, 1), (0, 1), (0, 1), (2, 0), (0xff, 2), (0xc0, 2), (2, 0), (7, 1)] {
            h.send(c);
            assert_eq!(h.v.drain().len(), n, "value {c:#04x}");
        }
        // A mailbox strobe with the port reversed.
        h.control(NSTROBE | NINIT | NSELECTIN);
        h.control(NINIT | NSELECTIN);
        assert_eq!(h.v.drain().len(), 1);
        // Readout before extraction, and past the signature.
        let mut r = Host::new(Options::default());
        r.dir(true);
        r.control(NSTROBE);
        r.control(0);
        r.control(NSTROBE);
        assert_eq!(r.v.drain().len(), 1);
        r.send(3);
        r.readout(NSTROBE);
        assert!(r.v.is_empty());
        r.control(0);
        r.control(NSTROBE);
        assert_eq!(r.v.drain().len(), 1);
        // The ADC changed mid-readout.
        r.send(4);
        r.chain(1, 1, 2, 101);
        r.send(3);
        r.control(NSTROBE);
        r.control(0);
        r.control(NSTROBE);
        r.control(NSTROBE | NAUTOFD);
        assert_eq!(r.v.drain().len(), 1);
        // Registers and requests.
        r.set(4, 1);
        r.get(4);
        r.get(9);
        assert_eq!(r.v.drain().len(), 3);
        let odd = Setup { request_type: 0xc0, request: 3, value: 0, index: 0, length: 8 };
        r.dev.control(odd, &[], &mut r.v);
        let set = Setup { request_type: 0x40, request: 4, value: 0, index: 1, length: 0 };
        r.dev.control(set, &[], &mut r.v);
        let other = Setup { request_type: 0x40, request: 5, value: 0, index: 0, length: 0 };
        assert_eq!(r.dev.control(other, &[], &mut r.v), Reply::Stall);
        assert_eq!(r.dev.out(1, &[0], &mut r.v), Reply::Stall);
        assert_eq!(r.dev.in_transfer(2, 64, &mut r.v), Reply::Stall);
        assert_eq!(r.v.drain().len(), 5);
    }

    #[test]
    fn options_parse() {
        let o = |k: &str, v: &str| parse_options(&[(k.into(), v.into())]);
        assert_eq!(o("trigger", "never").unwrap().trigger, Trigger::Never);
        assert_eq!(o("trigger", "5").unwrap().trigger, Trigger::After(5));
        assert!(!o("scope", "absent").unwrap().present);
        assert!(o("scope", "maybe").is_err());
        assert!(o("busy", "x").is_err());
        assert!(o("mark", "x").is_err());
        assert!(o("nope", "1").is_err());
        assert!(build(&[]).is_ok());
    }
}
