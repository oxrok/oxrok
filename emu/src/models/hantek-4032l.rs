// Counterpart in sigrok:
//   libsigrok/src/hardware/hantek-4032l/api.c (the rates, the trigger packing),
//   libsigrok/src/hardware/hantek-4032l/protocol.c (the commands, the status and the stream),
//   libsigrok/src/hardware/hantek-4032l/protocol.h,
//   Copyright (C) 2016 Andreas Zschunke <andreas.zschunke@gmx.net>,
//   Copyright (C) 2017 Andrej Valek <andy@skyrain.eu>,
//   Copyright (C) 2017 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Hantek 4032L logic analyzer (32 channels in two groups of 16, A
//! and B) as libsigrok's `hantek-4032l` driver sees it: a vendor USB
//! device with no firmware or bitstream to upload, one interface, a
//! command endpoint and a data endpoint.
//!
//! Where the facts come from. Everything protocol is from the C driver
//! (`api.c`, `protocol.c`, `protocol.h`) and its history in libsigrok's
//! git (commits 74c4c174, 7a7afc00, 2958315d, 80d34975); no Hantek
//! firmware, FPGA source or recording of the device is available here,
//! so what the driver does not say is the model's choice, named as such.
//!
//! - Ids: `04b5:4032` (`protocol.h`, `H4032L_USB_VENDOR`/`_PRODUCT`);
//!   the driver matches on nothing else and reads no string, so the
//!   strings, `bcdDevice` (0x0000) and the device class (vendor, 0xff)
//!   are the model's. The driver's 0xb3 request with the 10 bytes
//!   `0f 03 03 03 ...` is Hantek's FX2 "begin command" (`hantek-dso`
//!   sends it with the same first four bytes), and vendor id `04b5` is
//!   the one Hantek's FX2 firmwares take, so the board is taken to be an
//!   FX2 running firmware from its EEPROM at high speed: unverified.
//! - Endpoints (`protocol.c`): bulk OUT 2 takes the command packet,
//!   bulk IN 6 (`0x86`) gives the status and the samples; 512 byte
//!   packets (high speed). Interface 0 (`api.c`, `USB_INTERFACE`).
//! - The vendor request `0xb3` (`CMD_RESET`, "also arms the logic
//!   analyzer"): host to device, `wValue` and `wIndex` 0, the ten bytes
//!   `0f 03 03 03 00 00 00 00 00 00`, sent before every capture.
//! - The command packet (`struct h4032l_cmd_pkt`, `#pragma pack(2)`, 84
//!   bytes, checked against gcc's layout): magic `0x017f` (u16), the
//!   sample rate code (u8, an index into `samplerates_hw[]` or an
//!   external clock edge 0x24 to 0x29), the trigger flags (u8: bit 0
//!   trigger 1, bit 1 trigger 2, bit 2 AND), PWM A and B (u16 each, the
//!   threshold DACs, 12 bits), a reserved u16, the sample count per
//!   channel (u32, "2k-64M, must be multiple of 512"), the pre-trigger
//!   count (u32, "must be < `sample_size`"), two triggers of 32 bytes
//!   (flags u32: bits 0-4 edge signal, 5-6 edge type, 8-9 data range
//!   type, 10-11 time range type, 12 data range on, 13 time range on,
//!   16-17 data selection, 18 combined; then data range min, max, time
//!   range min, max, data range mask, combine mask, combine data), and
//!   the command (u16): `0x2b1a` configure, `0x4b3a` status, `0x6b5a`
//!   get. Every command carries the whole packet.
//! - The status packet (`struct h4032l_status_packet`, 20 bytes): magic
//!   `0x2b1a037f`, `values`, `status` (2 is "capture complete", the only
//!   value the driver reads), `usbxi_data`, `fpga_version`. Version
//!   `0x4303` is the one commit 80d34975 was verified on, the model's
//!   default; version 0 is the driver's other variant (one data transfer
//!   at a time, no external clock).
//! - The sample stream after a get: `0x2b1a027f` (start), the samples,
//!   one u32 each, bit `n` channel `n` (`api.c`: channel `n` is `A{n/2}`
//!   for even `n`, `B{n/2}` for odd), then `0x4d3c037f` (end). The driver
//!   reads it with one transfer of 512 bytes and then transfers of 2048
//!   (up to 32 at once, one at a time for FPGA version 0); the end word
//!   follows the last sample in the same transfer (protocol.c:194).
//!
//! How the model behaves (its choices, not the driver's facts):
//!
//! - After a status command, every IN transfer gets the status packet
//!   until the next command. In a capture the driver reads two answers
//!   per status command and looks at the first (protocol.c:244-270); at
//!   the open it reads until it sees the magic, up to ten times
//!   (protocol.c:541-555). `values` and `usbxi_data` are 0. `status` is
//!   0 before a capture, 1 while the capture runs or waits for its
//!   trigger, 2 when it is complete. Commits 74c4c174 ("drop packets
//!   until the start magic", "fix same data receiving") suggest the
//!   real FIFO can hold leftovers; `stale=N` plays that.
//! - A capture is complete after `fill` status commands that answer 1
//!   (default 1), once its trigger has fired. The signal is a counter:
//!   sample `t` of the device's time is `t` (mod 2^32), so channel `n`
//!   is a square wave of period `2^(n+1)` samples; it depends on nothing
//!   the host sets (rate, external clock, thresholds). Without a trigger
//!   the capture is samples 0 to `sample_size - 1` and the trigger point
//!   is `pre_trigger_size`. With trigger 1, the trigger point is the
//!   first `t >= pre_trigger_size` (and >= 1 for an edge) where every
//!   masked channel equals its value (data range type 0 read as "equal
//!   to the max under the mask", the driver's level matches) and the
//!   edge channel has its edge from `t - 1` to `t`; the capture starts
//!   `pre_trigger_size` samples before it. A condition the counter can
//!   never meet (a channel low and rising at once) never fires and the
//!   capture stays at status 1.
//! - Everything is paced by the host's commands and transfers, nothing
//!   by time. The stream is cut into transfers of the size the host
//!   asks for; it is ready as soon as the get command is taken.
//!
//! The pedantic checks: the reset request's type, value, index and ten
//! bytes; vendor requests other than `0xb3`; every command packet's
//! length, magic and command; for a configure: preceded by a reset, a
//! known rate code (not the zero entries 29 to 31; external clock codes
//! not on FPGA version 0), trigger 2 and the AND flag never set, the
//! reserved word 0, PWM values within 12 bits, a sample count a multiple
//! of 512 from 2048 to 64M, a pre-trigger count below it, trigger 1's
//! reserved bits, data range type 0, data range min, time ranges,
//! selection and combine fields 0 (the driver never sets them), an edge
//! type other than "disabled" only with a channel below 32, and trigger
//! 2 all zero; a get before a status answer of 2; IN transfers other
//! than 512 bytes for the status and the first stream transfer, or
//! other than 512 or 2048 for the rest; transfers on other endpoints.
//!
//! What the driver sends against its own comments, which the model
//! takes because a board could not refuse it (the configure is a bulk
//! OUT with no answer): a capture ratio of 100 (libsigrok refuses more,
//! `hwdriver.c` `check_key()`; api.c:409-410 takes it and api.c:508
//! then makes `pre_trigger_size` equal to the sample count, where
//! protocol.h:150 says it must be below; the trigger point is then
//! sample `sample_size`, past the capture, so sigrok-cli shows no
//! trigger marker), and a threshold of -8.2 V or below (api.c:430
//! takes any value; `h4032l_voltage2pwm()` clamps the reference at
//! 10 V, which gives a PWM value of 4096 where the comment at
//! protocol.c:345-346 says 4095; a C habit of an inclusive clamp, the
//! uint16 field carries it). Cases "set captureratio=100", "trigger
//! captureratio=100" and "threshold A minus 9" run both; a pre-trigger
//! count above the sample count is still flagged.
//!
//! A latent libsigrok bug: `h4032l_data_transfer_callback()` checks the
//! end word at `buf[num_samples]` (protocol.c:194), `num_samples` being
//! the samples the last data transfer takes. When that transfer is full
//! of samples (2048 bytes, the end word left for the next packet), the
//! read lands 4 bytes past the buffer. The model sends the start word
//! and 127 samples in the first 512 bytes, so the last transfer always
//! holds 385 samples and the end word (1544 bytes). A board that sent
//! the start word in a short packet of its own (commit 74c4c174 says
//! packetization varies) would leave a sample count, a multiple of 512,
//! that fills the last transfer exactly, and the driver would read heap
//! memory every time. No fault or case reaches it: its output would
//! depend on what lies past the buffer.
//!
//! Faults, all off by default: `stall=WHAT[@N],...` STALLs the Nth (or
//! every) `reset` request, `out` bulk OUT command or `in` bulk IN
//! transfer, counted since the model was built; a stalled request has
//! no effect and a stalled IN transfer takes no data. A one-shot STALL
//! is an emulation artifact: a real halt lasts until cleared, and this
//! driver never clears one. `stale=N` puts N packets of 64 zero bytes
//! before the answer of every command (leftovers in the FIFO). `end=HEX`
//! ends the stream with another word than the end magic, as a stream
//! cut or garbled inside the device would. `fill=N` (0 and up) and
//! `fpga=VERSION` (decimal or 0x hex) are the board's options.
//! `hold=N` sends N data transfers after the stream's first read and
//! holds those behind them open with nothing, as a board whose stream
//! stops. With it, `partial=BYTES` (whole packets) is what the oldest
//! held transfer has received when it is cancelled, once per capture, the
//! driver's timed out transfer with data; and `release=1` lets the stream
//! go on after that first cancel, a slow board rather than a stopped one.
//!
//! Emulation artifacts, besides the one-shot STALL: the stream is ready
//! the moment the get is taken and every bulk IN is answered at once,
//! so the driver's 500 ms timeouts never run out except where the model
//! has nothing to send (a status command it stalled); the stale packet
//! length; the counter signal; no strings in the descriptors.
//!
//! What the cases (`tools/emu/hantek-4032l.sh`) do not reach, and why
//! (`tests/emu/hantek-4032l/gcov.txt`): in api.c, a port path that
//! cannot be read (226), an open or claim failing other than for a busy
//! interface (294-295, 305-311), `dev_close()` without a handle (331),
//! the threshold without a channel group, a `conn` get without a
//! connection and unknown keys (352, 354, 375, 381-382, 426, 428,
//! 445-446: sigrok-cli and `check_key()` refuse first), an unknown match
//! (581-583: sigrok-cli passes the listed matches alone; a second stage,
//! `-t A0=01`, reaches 529-530 in the case "two trigger stages"); in
//! protocol.c, a status or start read coming back after an abort
//! (225-226: only `dev_acquisition_stop()` aborts, which sigrok-cli never
//! calls; the runner's stop reaches a data transfer, 171-172, in "lib stop
//! while held", and at the header it races the model's answer, which comes
//! at once, so no case is stable), the callback in the idle
//! state (235-237, 327: an idle transfer is freed, not resubmitted), fewer than 512
//! samples left after the first transfer (376: at least 1921 are),
//! submissions that fail at the start of the data or of the capture
//! (298-299, 394-399, 441-444: an unplug right after the stream's start,
//! tried with `--unplug in=6:6`, still saw the data transfers submitted
//! and fail on completion, and which way it goes depends on timing; the
//! polling loop's failed submit, 321-324, is reached), the
//! device list failing, a second device and an open after a firmware
//! upload or one refused (473-475, 489, 493, 502-508).

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{BULK, Descriptors, DeviceModel, Reply, Setup, Speed, VENDOR, Violations};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hantek-4032l";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Hantek 4032L logic analyzer (04b5:4032, 32 channels); fpga=VERSION \
     (default 0x4303, 0 the driver's old variant), fill=N; faults: stall=reset|out|in[@N], \
     stale=N, end=HEX, hold=N, partial=BYTES, release=1";

/// The USB vendor id.
pub const VID: u16 = 0x04b5;
/// The USB product id.
pub const PID: u16 = 0x4032;
/// The FPGA version the model reports by default (commit 80d34975).
pub const FPGA_VERSION: u32 = 0x4303;

/// The vendor request that resets and arms the analyzer.
pub const REQ_RESET: u8 = 0xb3;
/// Its ten bytes.
pub const RESET_DATA: [u8; 10] = [0x0f, 0x03, 0x03, 0x03, 0, 0, 0, 0, 0, 0];

/// The command endpoint (OUT).
pub const EP_CMD: u8 = 2;
/// The data endpoint (IN).
pub const EP_DATA: u8 = 6;
/// Bulk packet size, high speed.
pub const PACKET: usize = 512;
/// The length of a stale packet (`stale=N`), the model's choice.
pub const STALE: usize = 64;
/// The driver's data transfer size (`H4032L_DATA_BUFFER_SIZE`).
pub const DATA_TRANSFER: usize = 2048;

/// The command packet's length.
pub const CMD_LEN: usize = 84;
/// The command packet's magic.
pub const CMD_MAGIC: u16 = 0x017f;
/// Configure.
pub const CMD_CONFIGURE: u16 = 0x2b1a;
/// Status.
pub const CMD_STATUS: u16 = 0x4b3a;
/// Get the samples.
pub const CMD_GET: u16 = 0x6b5a;

/// The status packet's magic.
pub const STATUS_MAGIC: u32 = 0x2b1a_037f;
/// The word before the samples.
pub const START_MAGIC: u32 = 0x2b1a_027f;
/// The word after them.
pub const END_MAGIC: u32 = 0x4d3c_037f;

/// `samplerates_hw[]` (api.c:131-168): the rate of each code, 0 for the
/// three codes that have none.
pub const RATES: [u64; 36] = [
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

/// The external clock codes (`enum h4032l_clock_edge_type_channel`).
pub const EXT_CLOCK: std::ops::RangeInclusive<u8> = 0x24..=0x29;

const EDGE_RISE: u32 = 0;
const EDGE_FALL: u32 = 1;
const EDGE_TOGGLE: u32 = 2;
const EDGE_DISABLED: u32 = 3;

/// Which requests a `stall=` fault names.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum What {
    /// The vendor reset request.
    Reset,
    /// A bulk OUT command.
    Out,
    /// A bulk IN transfer.
    In,
}

/// Ways the board can misbehave; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Requests that STALL: every one (`None`) or the Nth.
    pub stall: Vec<(What, Option<u32>)>,
    /// Stale packets before the answer of each command.
    pub stale: u32,
    /// The word that ends the stream instead of [`END_MAGIC`].
    pub end: Option<u32>,
    /// Data transfers sent before those behind them are held.
    pub hold: Option<u32>,
    /// What the oldest held transfer holds when cancelled, once per
    /// capture, in bytes.
    pub partial: u32,
    /// The stream goes on after the first cancel of a held transfer.
    pub release: bool,
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        match k {
            "stall" => {
                self.stall.clear();
                for item in v.split(',').filter(|i| !i.is_empty()) {
                    let (name, n) = match item.split_once('@') {
                        Some((name, n)) => (
                            name,
                            Some(n.parse::<u32>().ok().filter(|n| *n > 0).ok_or_else(|| {
                                Error(format!("stall={item}: N from 1"))
                            })?),
                        ),
                        None => (item, None),
                    };
                    let what = match name {
                        "reset" => What::Reset,
                        "out" => What::Out,
                        "in" => What::In,
                        _ => return Err(Error(format!("stall={name}: reset, out or in"))),
                    };
                    self.stall.push((what, n));
                }
            }
            "stale" => {
                self.stale = v.parse().map_err(|_| Error(format!("stale={v}: a count")))?;
            }
            "end" => self.end = Some(parse_u32(k, v)?),
            "hold" => {
                self.hold = Some(v.parse().map_err(|_| Error(format!("hold={v}: a count")))?);
            }
            "partial" => {
                self.partial = v.parse().map_err(|_| Error(format!("partial={v}: bytes")))?;
                if !(self.partial as usize).is_multiple_of(PACKET) {
                    return Err(Error(format!("partial={v}: whole packets of {PACKET}")));
                }
            }
            "release" => self.release = v == "1",
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// A number in decimal or with `0x` in hex.
fn parse_u32(k: &str, v: &str) -> Result<u32> {
    let n = match v.strip_prefix("0x") {
        Some(hex) => u32::from_str_radix(hex, 16),
        None => v.parse(),
    };
    n.map_err(|_| Error(format!("{k}={v}: a 32 bit number, decimal or 0x hex")))
}

/// A board from the `-o` options: `fpga=VERSION`, `fill=N` and the
/// faults (see the module documentation).
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut fpga = FPGA_VERSION;
    let mut fill = 1;
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "fpga" => fpga = parse_u32(k, v)?,
            "fill" => fill = v.parse().map_err(|_| Error(format!("fill={v}: a count")))?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    if faults.hold.is_none() && (faults.partial > 0 || faults.release) {
        return Err(Error("partial= and release= need hold=".to_owned()));
    }
    let mut model = H4032l::new(fpga);
    model.fill = fill;
    model.faults = faults;
    Ok(Box::new(model))
}

/// The configuration a configure command carries.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Config {
    /// The rate code.
    pub rate: u8,
    /// Trigger 1 on.
    pub trigger: bool,
    /// PWM A and B.
    pub pwm: [u16; 2],
    /// Samples per channel.
    pub samples: u32,
    /// Samples before the trigger.
    pub pre: u32,
    /// Trigger 1's level mask and values.
    pub mask: u32,
    /// Trigger 1's level values.
    pub value: u32,
    /// Trigger 1's edge type (3 disabled).
    pub edge_type: u32,
    /// Trigger 1's edge channel.
    pub edge_signal: u32,
}

fn u16_at(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

/// The smallest `t >= lo` with `t & mask == value` (`value` within
/// `mask`, both within 32 bits, `t` counted on 64).
pub fn next_match(lo: u64, mask: u32, value: u32) -> u64 {
    let (mask, value) = (u64::from(mask), u64::from(value));
    if lo & mask == value {
        return lo;
    }
    // The first bit, from the bottom, where the answer can go above lo:
    // lo has 0 there, the answer 1, the bits above as lo's, those below
    // the least the mask allows.
    for b in 0..40 {
        let bit = 1u64 << b;
        let above = !((bit << 1) - 1);
        if lo & bit != 0 || (mask & bit != 0 && value & bit == 0) {
            continue;
        }
        if lo & above & mask != value & above {
            continue;
        }
        return (lo & above) | bit | (value & (bit - 1));
    }
    unreachable!("a 32 bit mask always has an answer below 2^40")
}

impl Config {
    /// The first sample of the capture, in the device's time; `None`
    /// when its trigger can never fire.
    pub fn start(&self) -> Option<u64> {
        let pre = u64::from(self.pre);
        if !self.trigger {
            return Some(0);
        }
        let (mut mask, mut value) = (self.mask, self.value & self.mask);
        let mut lo = pre;
        if self.edge_type != EDGE_DISABLED {
            let e = self.edge_signal;
            let low = (1u32 << e) - 1;
            let (m, val) = match self.edge_type {
                // From 0 to 1 at bit e: bits 0 to e of t are 1 followed by 0s.
                EDGE_RISE => (low | (1 << e), 1 << e),
                // From 1 to 0: bits 0 to e of t all 0.
                EDGE_FALL => (low | (1 << e), 0),
                // Either: bits below e all 0.
                EDGE_TOGGLE => (low, 0),
                _ => unreachable!("two bits"),
            };
            if (value ^ val) & mask & m != 0 {
                return None;
            }
            mask |= m;
            value |= val;
            lo = lo.max(1);
        }
        Some(next_match(lo, mask, value) - pre)
    }
}

/// Where the device is.
#[derive(Clone, Debug, PartialEq, Eq)]
enum State {
    /// Nothing asked yet, or the stream was read to its end.
    Idle,
    /// Status answers after a status command: the status value.
    Status(u32),
    /// The stream: the next byte, the stream's length and its first
    /// sample in the device's time.
    Stream { at: usize, len: usize, first: u64 },
}

/// The emulated board.
pub struct H4032l {
    descriptors: Descriptors,
    fpga: u32,
    /// Status commands that answer 1 before a capture is complete.
    pub fill: u32,
    faults: Faults,
    state: State,
    /// A reset request came since the last configure.
    armed: bool,
    /// The capture: its configuration and the polls left.
    capture: Option<(Config, u32)>,
    /// A status of 2 was answered since the configure.
    ready: bool,
    /// Stale packets still to send before the current answer.
    stale: u32,
    /// The first IN transfer of the stream is yet to come.
    first_stream_in: bool,
    /// Data transfers sent since the stream's first read.
    data_transfers: u32,
    /// What the oldest held transfer still holds, in bytes.
    partial: u32,
    /// `release=1` came into force in this capture.
    released: bool,
    seen: Vec<(What, u32)>,
}

impl H4032l {
    /// A board reporting FPGA version `fpga`.
    pub fn new(fpga: u32) -> H4032l {
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(EP_CMD, BULK, 512, 0));
        body.extend_from_slice(&descriptor::endpoint(0x80 | EP_DATA, BULK, 512, 0));
        let config = descriptor::configuration(1, 0, 0x80, 500, &body);
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0xff, 0xff, 0xff],
            max_packet0: 64,
            vendor: VID,
            product: PID,
            device: 0x0000,
            strings: [0, 0, 0],
            configurations: 1,
        };
        H4032l {
            descriptors: Descriptors {
                speed: Speed::High,
                device: device.bytes(),
                configs: vec![config],
                strings: Vec::new(),
            },
            fpga,
            fill: 1,
            faults: Faults::default(),
            state: State::Idle,
            armed: false,
            capture: None,
            ready: false,
            stale: 0,
            first_stream_in: false,
            data_transfers: 0,
            partial: 0,
            released: false,
            seen: Vec::new(),
        }
    }

    /// Counts a request of kind `what`; whether a fault STALLs it.
    fn stalls(&mut self, what: What) -> bool {
        let n = if let Some((_, n)) = self.seen.iter_mut().find(|(w, _)| *w == what) {
            *n += 1;
            *n
        } else {
            self.seen.push((what, 1));
            1
        };
        self.faults.stall.iter().any(|(w, at)| *w == what && at.is_none_or(|a| a == n))
    }

    /// The checks of a configure's fields; the configuration.
    fn configuration(&self, p: &[u8], v: &mut Violations) -> Config {
        let rate = p[2];
        if EXT_CLOCK.contains(&rate) {
            if self.fpga == 0 {
                v.report(format!(
                    "external clock code {rate:#04x} to FPGA version 0, which has no external clock"
                ));
            }
        } else if RATES.get(usize::from(rate)).is_none_or(|r| *r == 0) {
            v.report(format!("sample rate code {rate}: not one of samplerates_hw[] with a rate"));
        }
        let flags = p[3];
        if flags & !1 != 0 {
            v.report(format!(
                "trigger flags {flags:#04x}: the driver sets trigger 1 (bit 0) alone"
            ));
        }
        // PWM values are not checked: the driver sends 4096 for -8.2 V
        // and below (see the module doc), and a uint16 field of a
        // command with no answer is nothing a board could refuse.
        if u16_at(p, 8) != 0 {
            v.report(format!("reserved word {:#06x}, not 0", u16_at(p, 8)));
        }
        let samples = u32_at(p, 10);
        if !samples.is_multiple_of(512) || !(2048..=64 << 20).contains(&samples) {
            v.report(format!(
                "sample count {samples}: a multiple of 512 from 2048 to 64M (protocol.h)"
            ));
        }
        let pre = u32_at(p, 14);
        if pre > samples {
            v.report(format!(
                "pre-trigger count {pre}: above the sample count {samples} (protocol.h)"
            ));
        }
        let t = &p[18..50];
        let f = u32_at(t, 0);
        let mut cfg = Config {
            rate,
            trigger: flags & 1 != 0,
            pwm: [u16_at(p, 4), u16_at(p, 6)],
            samples,
            pre,
            mask: u32_at(t, 20),
            value: u32_at(t, 8),
            edge_type: (f >> 5) & 3,
            edge_signal: f & 0x1f,
        };
        if cfg.trigger {
            // Bits the driver never sets: 7, 10-11 (time range type),
            // 13 (time range on), 14-15, 16-17 (data selection), 18
            // (combined), and above; 8-9, the data range type, are 0.
            if f & !0x107f != 0 {
                v.report(format!(
                    "trigger 1 flags {f:#010x}: only the edge, its type and the data range \
                     enable (bits 0-6, 12) are the driver's"
                ));
            }
            if f & (1 << 12) == 0 {
                v.report("trigger 1 without its data range on, which the driver always sets");
            }
            for (name, at) in [
                ("data range min", 4),
                ("time range min", 12),
                ("time range max", 16),
                ("combine mask", 24),
                ("combine data", 28),
            ] {
                if u32_at(t, at) != 0 {
                    v.report(format!("trigger 1 {name} {:#x}, not 0", u32_at(t, at)));
                }
            }
            if cfg.value & !cfg.mask != 0 {
                v.report(format!(
                    "trigger 1 data range max {:#010x} outside its mask {:#010x}",
                    cfg.value, cfg.mask
                ));
            }
            // The C driver would fail this on a second capture in one
            // process, a level trigger after an edge trigger: api.c:533
            // resets the edge type, not the signal. sigrok-cli runs one
            // capture per process, so no case reaches it.
            if cfg.edge_type == EDGE_DISABLED && cfg.edge_signal != 0 {
                v.report(format!(
                    "trigger 1 edge signal {} with the edge disabled",
                    cfg.edge_signal
                ));
            }
        } else {
            cfg.mask = 0;
            cfg.value = 0;
            cfg.edge_type = EDGE_DISABLED;
            cfg.edge_signal = 0;
        }
        if p[50..82].iter().any(|b| *b != 0) {
            v.report("trigger 2 not all zero; the driver never sets it");
        }
        cfg
    }

    fn command(&mut self, p: &[u8], v: &mut Violations) {
        if p.len() != CMD_LEN {
            v.report(format!("command packet of {} bytes; it has {CMD_LEN}", p.len()));
            return;
        }
        if u16_at(p, 0) != CMD_MAGIC {
            v.report(format!("command packet magic {:#06x}, not {CMD_MAGIC:#06x}", u16_at(p, 0)));
            return;
        }
        self.stale = self.faults.stale;
        match u16_at(p, 82) {
            CMD_CONFIGURE => {
                if !self.armed {
                    v.report("configure without the reset request (0xb3) that arms the analyzer");
                }
                self.armed = false;
                let cfg = self.configuration(p, v);
                self.capture = Some((cfg, self.fill));
                self.ready = false;
                self.state = State::Idle;
            }
            CMD_STATUS => {
                let status = match &mut self.capture {
                    None => 0,
                    Some((cfg, left)) => {
                        if *left > 0 {
                            *left -= 1;
                            1
                        } else if cfg.start().is_some() {
                            2
                        } else {
                            1
                        }
                    }
                };
                self.state = State::Status(status);
            }
            CMD_GET => {
                let Some((cfg, _)) = self.capture.filter(|_| self.ready) else {
                    v.report("get before a status answer of 2");
                    self.state = State::Idle;
                    return;
                };
                let first = cfg.start().unwrap_or(0);
                let len = 4 * (2 + cfg.samples as usize);
                self.state = State::Stream { at: 0, len, first };
                self.first_stream_in = true;
                self.data_transfers = 0;
                self.partial = self.faults.partial;
                self.released = false;
                self.capture = None;
                self.ready = false;
            }
            other => {
                v.report(format!("command {other:#06x}: not configure, status or get"));
                self.state = State::Idle;
            }
        }
    }

    /// The word at index `w` of a stream of `len` bytes.
    fn stream_word(&self, w: usize, len: usize, first: u64) -> u32 {
        if w == 0 {
            START_MAGIC
        } else if w == len / 4 - 1 {
            self.faults.end.unwrap_or(END_MAGIC)
        } else {
            // The counter, mod 2^32.
            (first + (w as u64 - 1)) as u32
        }
    }

    /// `n` bytes of the stream from `at`.
    fn stream_bytes(&self, at: usize, n: usize, len: usize, first: u64) -> Vec<u8> {
        let mut b = Vec::with_capacity(n);
        let mut i = at;
        while i < at + n {
            let word = self.stream_word(i / 4, len, first).to_le_bytes();
            let off = i % 4;
            let take = (4 - off).min(at + n - i);
            b.extend_from_slice(&word[off..off + take]);
            i += take;
        }
        b
    }

    /// The stream's data transfers are held (`hold=N`).
    fn holding(&self) -> bool {
        matches!(self.state, State::Stream { .. })
            && !self.first_stream_in
            && !self.released
            && self.stale == 0
            && self.faults.hold.is_some_and(|n| self.data_transfers >= n)
    }

    /// The status packet.
    fn status_packet(&self, status: u32) -> Vec<u8> {
        let mut b = Vec::with_capacity(20);
        for word in [STATUS_MAGIC, 0, status, 0, self.fpga] {
            b.extend_from_slice(&word.to_le_bytes());
        }
        b
    }
}

impl DeviceModel for H4032l {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() != VENDOR || setup.request != REQ_RESET {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} index {:#06x}: the board answers 0xb3 alone",
                setup.request_type, setup.request, setup.value, setup.index
            ));
            return Reply::Stall;
        }
        if setup.request_type != 0x40 || setup.value != 0 || setup.index != 0 {
            v.report(format!(
                "reset request type {:#04x} value {:#06x} index {:#06x}: the driver sends 0x40, 0, 0",
                setup.request_type, setup.value, setup.index
            ));
        }
        if data != RESET_DATA {
            v.report(format!("reset request data {data:02x?}, not {RESET_DATA:02x?}"));
        }
        if self.stalls(What::Reset) {
            return Reply::Stall;
        }
        // The reset ends whatever ran and flushes the FIFO.
        self.armed = true;
        self.capture = None;
        self.ready = false;
        self.stale = 0;
        self.state = State::Idle;
        Reply::ack()
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_CMD {
            v.report(format!("OUT transfer to endpoint {ep}; commands go to {EP_CMD}"));
            return Reply::Stall;
        }
        if self.stalls(What::Out) {
            return Reply::Stall;
        }
        self.command(data, v);
        Reply::ack()
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP_DATA {
            v.report(format!("IN transfer from endpoint {ep}; the board answers on {EP_DATA}"));
            return Reply::Stall;
        }
        if self.state == State::Idle && self.stale == 0 {
            // Nothing in the FIFO: the host waits.
            return Reply::Nak;
        }
        if self.holding() {
            return Reply::Nak;
        }
        let first = matches!(self.state, State::Status(_)) || self.first_stream_in;
        if first && max != PACKET {
            v.report(format!(
                "IN transfer of {max} bytes for a status or the stream's start; the driver \
                 reads {PACKET}"
            ));
        } else if !first && max != PACKET && max != DATA_TRANSFER {
            v.report(format!(
                "IN transfer of {max} bytes; the driver reads {PACKET} or {DATA_TRANSFER}"
            ));
        }
        if self.stalls(What::In) {
            return Reply::Stall;
        }
        if self.stale > 0 {
            self.stale -= 1;
            return Reply::Data(vec![0; STALE.min(max)]);
        }
        match self.state {
            State::Idle => Reply::Nak,
            State::Status(status) => {
                if status == 2 {
                    self.ready = true;
                }
                Reply::Data(self.status_packet(status))
            }
            State::Stream { at, len, first } => {
                if !std::mem::take(&mut self.first_stream_in) {
                    self.data_transfers += 1;
                }
                // Whole packets up to what the host asks for.
                let n = (max - max % PACKET).max(PACKET.min(max)).min(len - at);
                let b = self.stream_bytes(at, n, len, first);
                self.state =
                    if at + n == len { State::Idle } else { State::Stream { at: at + n, len, first } };
                Reply::Data(b)
            }
        }
    }

    /// `partial=` and `release=1`: the emulator offers this to the oldest
    /// held transfer alone.
    fn in_unlinked(&mut self, ep: u8, max: usize) -> Option<Vec<u8>> {
        if ep != EP_DATA || !self.holding() {
            return None;
        }
        self.released = self.faults.release;
        let State::Stream { at, len, first } = self.state else { return None };
        let n = (self.partial as usize).min(max).min(len - at);
        self.partial = 0;
        if n == 0 {
            return None;
        }
        let b = self.stream_bytes(at, n, len, first);
        self.state = if at + n == len { State::Idle } else { State::Stream { at: at + n, len, first } };
        Some(b)
    }

    fn reset(&mut self) {
        self.data_transfers = 0;
        self.partial = 0;
        self.released = false;
        self.state = State::Idle;
        self.armed = false;
        self.capture = None;
        self.ready = false;
        self.stale = 0;
        self.first_stream_in = false;
    }

    /// Data transfers of 2048 bytes.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The command packet as the driver builds it at the open.
    fn packet(cmd: u16) -> Vec<u8> {
        let mut p = vec![0u8; CMD_LEN];
        p[0..2].copy_from_slice(&CMD_MAGIC.to_le_bytes());
        p[10..14].copy_from_slice(&16384u32.to_le_bytes());
        p[82..84].copy_from_slice(&cmd.to_le_bytes());
        p
    }

    /// A configure of `samples` with the default threshold (PWM 1174)
    /// and `pre` samples before the trigger.
    fn configure(samples: u32, pre: u32) -> Vec<u8> {
        let mut p = packet(CMD_CONFIGURE);
        p[4..6].copy_from_slice(&1174u16.to_le_bytes());
        p[6..8].copy_from_slice(&1174u16.to_le_bytes());
        p[10..14].copy_from_slice(&samples.to_le_bytes());
        p[14..18].copy_from_slice(&pre.to_le_bytes());
        p
    }

    fn reset_setup() -> Setup {
        Setup { request_type: 0x40, request: REQ_RESET, value: 0, index: 0, length: 10 }
    }

    fn data(r: Reply) -> Vec<u8> {
        match r {
            Reply::Data(d) => d,
            other => panic!("{other:?}"),
        }
    }

    fn words(b: &[u8]) -> Vec<u32> {
        b.chunks(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
    }

    /// Runs a capture as the driver does: reset, configure, status
    /// polls (two reads each) until 2, get; the stream's words.
    fn capture(dev: &mut H4032l, p: &[u8], v: &mut Violations) -> Vec<u32> {
        assert_eq!(dev.control(reset_setup(), &RESET_DATA, v), Reply::ack());
        assert_eq!(dev.out(EP_CMD, p, v), Reply::ack());
        let mut st = p.to_vec();
        st[82..84].copy_from_slice(&CMD_STATUS.to_le_bytes());
        loop {
            assert_eq!(dev.out(EP_CMD, &st, v), Reply::ack());
            // Read until the magic, then once more (protocol.c:248-270).
            let s = loop {
                let s = words(&data(dev.in_transfer(EP_DATA, 512, v)));
                if s[0] == STATUS_MAGIC {
                    break s;
                }
            };
            data(dev.in_transfer(EP_DATA, 512, v));
            if s[2] == 2 {
                break;
            }
        }
        let mut get = p.to_vec();
        get[82..84].copy_from_slice(&CMD_GET.to_le_bytes());
        assert_eq!(dev.out(EP_CMD, &get, v), Reply::ack());
        // Drop packets until the start magic (protocol.c:277-281).
        let mut stream = loop {
            let d = data(dev.in_transfer(EP_DATA, 512, v));
            if words(&d)[0] == START_MAGIC {
                break d;
            }
        };
        loop {
            match dev.in_transfer(EP_DATA, 2048, v) {
                Reply::Data(d) => stream.extend(d),
                Reply::Nak => break,
                Reply::Stall => panic!("stall"),
            }
        }
        words(&stream)
    }

    #[test]
    fn the_command_packet_is_84_bytes_as_gcc_packs_it() {
        // The driver's struct as gcc lays it out (protocol.h under its
        // pack(2)): sizeof 84, sample_size at 10, pre_trigger_size at
        // 14, trigger at 18, cmd at 82; with enable_trigger1 = 1 and
        // trigger[0]'s edge_signal 31, edge_type 3, both range types 3,
        // both range enables, data_sel 3 and combined_enabled 1 its
        // bytes 3 and 18 to 20 read 01 and 7f 3f 07.
        let mut p = [0u8; CMD_LEN];
        p[3] = 1;
        p[18..21].copy_from_slice(&[0x7f, 0x3f, 0x07]);
        let dev = H4032l::new(FPGA_VERSION);
        let mut v = Violations::default();
        let c = dev.configuration(&p, &mut v);
        assert!(c.trigger);
        assert_eq!((c.edge_signal, c.edge_type), (31, 3));
        let f = u32_at(&p, 18);
        assert_eq!(((f >> 8) & 3, (f >> 10) & 3, (f >> 12) & 3, (f >> 16) & 3), (3, 3, 3, 3));
        assert_eq!(f >> 18, 1);
    }

    #[test]
    fn answers_the_open_with_the_fpga_version() {
        let mut dev = H4032l::new(FPGA_VERSION);
        let mut v = Violations::default();
        assert_eq!(dev.in_transfer(EP_DATA, 512, &mut v), Reply::Nak);
        assert_eq!(dev.out(EP_CMD, &packet(CMD_STATUS), &mut v), Reply::ack());
        let s = words(&data(dev.in_transfer(EP_DATA, 512, &mut v)));
        assert_eq!(s, vec![STATUS_MAGIC, 0, 0, 0, FPGA_VERSION]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn captures_a_counter_without_trigger() {
        let mut dev = H4032l::new(FPGA_VERSION);
        let mut v = Violations::default();
        let w = capture(&mut dev, &configure(2048, 102), &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(w.len(), 2050);
        assert_eq!(w[0], START_MAGIC);
        assert_eq!(w[1..2049], (0..2048).collect::<Vec<u32>>());
        assert_eq!(w[2049], END_MAGIC);
        // The second capture waits for its fill again.
        dev.fill = 3;
        let w = capture(&mut dev, &configure(4096, 0), &mut v);
        assert_eq!(w.len(), 4098);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_stream_is_cut_as_the_driver_reads_it() {
        let mut dev = H4032l::new(FPGA_VERSION);
        let mut v = Violations::default();
        dev.control(reset_setup(), &RESET_DATA, &mut v);
        dev.out(EP_CMD, &configure(2048, 0), &mut v);
        let mut st = configure(2048, 0);
        st[82..84].copy_from_slice(&CMD_STATUS.to_le_bytes());
        for _ in 0..2 {
            dev.out(EP_CMD, &st, &mut v);
            dev.in_transfer(EP_DATA, 512, &mut v);
            dev.in_transfer(EP_DATA, 512, &mut v);
        }
        let mut get = st.clone();
        get[82..84].copy_from_slice(&CMD_GET.to_le_bytes());
        dev.out(EP_CMD, &get, &mut v);
        let lens: Vec<usize> = std::iter::from_fn(|| match dev.in_transfer(EP_DATA, 2048, &mut v) {
            Reply::Data(d) => Some(d.len()),
            _ => None,
        })
        .collect();
        // 8 + 4 * 2048 bytes.
        assert_eq!(lens, vec![2048, 2048, 2048, 2048, 8]);
        assert_eq!(v.drain().len(), 1, "the stream's first read is 512 bytes");
    }

    #[test]
    fn triggers_on_the_counter() {
        let mut c = Config { trigger: true, pre: 100, edge_type: EDGE_DISABLED, ..Config::default() };
        // A0 and B0 high: t = 3 mod 4, from 100 on.
        c.mask = 3;
        c.value = 3;
        assert_eq!(c.start(), Some(103 - 100));
        // A1 (bit 2) rising: t = 4 mod 8.
        c.mask = 0;
        c.value = 0;
        c.edge_type = EDGE_RISE;
        c.edge_signal = 2;
        assert_eq!(c.start(), Some(100 - 100));
        c.edge_type = EDGE_FALL;
        assert_eq!(c.start(), Some(104 - 100));
        c.edge_type = EDGE_TOGGLE;
        c.edge_signal = 4;
        assert_eq!(c.start(), Some(112 - 100));
        // B15 rising, far away.
        c.edge_type = EDGE_RISE;
        c.edge_signal = 31;
        assert_eq!(c.start(), Some((1 << 31) - 100));
        // A0 low and rising: never.
        c.edge_signal = 0;
        c.mask = 1;
        c.value = 0;
        assert_eq!(c.start(), None);
        // An edge at 0 needs a sample before it.
        c.pre = 0;
        c.mask = 0;
        c.edge_type = EDGE_TOGGLE;
        assert_eq!(c.start(), Some(1));
        for (lo, mask, value) in [(5u64, 0x10u32, 0u32), (0x1f, 0x13, 0x02), (1000, 0xff, 0x80)] {
            let t = next_match(lo, mask, value);
            assert_eq!(t & u64::from(mask), u64::from(value));
            assert!((lo..t).all(|s| s & u64::from(mask) != u64::from(value)), "{lo} {t}");
        }
    }

    #[test]
    fn a_trigger_places_the_capture() {
        let mut dev = H4032l::new(FPGA_VERSION);
        let mut v = Violations::default();
        let mut p = configure(2048, 1024);
        p[3] = 1;
        // A1 rising, data range on.
        p[18..22].copy_from_slice(&(2u32 | (EDGE_RISE << 5) | (1 << 12)).to_le_bytes());
        let w = capture(&mut dev, &p, &mut v);
        assert!(v.is_empty(), "{v:?}");
        // 1028 is the first t >= 1024 with bits 0-2 = 100.
        assert_eq!(w[1 + 1024], 1028);
        assert_eq!(w[1], 4);
    }

    #[test]
    fn a_trigger_that_never_fires_keeps_the_status_at_1() {
        let mut dev = H4032l::new(FPGA_VERSION);
        let mut v = Violations::default();
        let mut p = configure(2048, 0);
        p[3] = 1;
        p[18..22].copy_from_slice(&(EDGE_RISE << 5 | (1 << 12)).to_le_bytes());
        p[38..42].copy_from_slice(&1u32.to_le_bytes());
        dev.control(reset_setup(), &RESET_DATA, &mut v);
        dev.out(EP_CMD, &p, &mut v);
        p[82..84].copy_from_slice(&CMD_STATUS.to_le_bytes());
        for _ in 0..10 {
            dev.out(EP_CMD, &p, &mut v);
            assert_eq!(words(&data(dev.in_transfer(EP_DATA, 512, &mut v)))[2], 1);
            dev.in_transfer(EP_DATA, 512, &mut v);
        }
        p[82..84].copy_from_slice(&CMD_GET.to_le_bytes());
        dev.out(EP_CMD, &p, &mut v);
        assert_eq!(v.drain().len(), 1, "get before ready");
    }

    #[test]
    fn flags_what_the_driver_never_sends() {
        let mut dev = H4032l::new(0);
        let mut v = Violations::default();
        // A configure without the reset.
        dev.out(EP_CMD, &configure(2048, 0), &mut v);
        assert_eq!(v.drain().len(), 1, "no reset");
        let arm = |dev: &mut H4032l, v: &mut Violations| {
            dev.control(reset_setup(), &RESET_DATA, v);
        };
        let mut bad = Vec::new();
        let mut p = configure(2048, 0);
        p[2] = 30;
        bad.push(("rate code 30", p));
        let mut p = configure(2048, 0);
        p[2] = 0x24;
        bad.push(("external clock on FPGA 0", p));
        let mut p = configure(2048, 0);
        p[3] = 2;
        bad.push(("trigger 2", p));
        let mut p = configure(2048, 0);
        p[8] = 1;
        bad.push(("reserved", p));
        bad.push(("2560 samples ok, 1000 not", configure(1000, 0)));
        bad.push(("pre above samples", configure(2048, 2560)));
        let mut p = configure(2048, 0);
        p[3] = 1;
        p[18..22].copy_from_slice(&(EDGE_DISABLED << 5 | 1 << 12 | 1 << 13).to_le_bytes());
        bad.push(("time range on", p));
        let mut p = configure(2048, 0);
        p[3] = 1;
        p[18..22].copy_from_slice(&(EDGE_DISABLED << 5).to_le_bytes());
        bad.push(("data range off", p));
        let mut p = configure(2048, 0);
        p[3] = 1;
        p[18..22].copy_from_slice(&(EDGE_DISABLED << 5 | 1 << 12).to_le_bytes());
        p[22] = 1;
        bad.push(("data range min", p));
        let mut p = configure(2048, 0);
        p[3] = 1;
        p[18..22].copy_from_slice(&(EDGE_DISABLED << 5 | 1 << 12).to_le_bytes());
        p[26] = 3;
        p[38] = 1;
        bad.push(("value outside the mask", p));
        let mut p = configure(2048, 0);
        p[60] = 1;
        bad.push(("trigger 2 data", p));
        let mut p = configure(2048, 0);
        p[0] = 0x7e;
        bad.push(("magic", p));
        let mut p = configure(2048, 0);
        p[82] = 0x11;
        bad.push(("command", p));
        bad.push(("length", configure(2048, 0)[..80].to_vec()));
        for (what, p) in bad {
            arm(&mut dev, &mut v);
            dev.out(EP_CMD, &p, &mut v);
            assert_eq!(v.drain().len(), 1, "{what}");
        }
        // The reset request's bytes and other requests.
        dev.control(reset_setup(), &[0; 10], &mut v);
        assert_eq!(v.drain().len(), 1, "reset data");
        let s = Setup { request_type: 0xc0, request: 0xb2, value: 0, index: 0, length: 2 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "unknown request");
        // Other endpoints and sizes.
        assert_eq!(dev.out(1, &packet(CMD_STATUS), &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "OUT 1");
        assert_eq!(dev.in_transfer(2, 512, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "IN 2");
        dev.out(EP_CMD, &packet(CMD_STATUS), &mut v);
        dev.in_transfer(EP_DATA, 64, &mut v);
        assert_eq!(v.drain().len(), 1, "status read of 64");
        // A good configure of a rate code with a rate passes.
        arm(&mut dev, &mut v);
        let mut p = configure(2560 - 512, 0);
        p[2] = 35;
        dev.out(EP_CMD, &p, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    /// Up to the stream's first read of a capture of 2048 samples.
    fn up_to_the_stream(dev: &mut H4032l, v: &mut Violations) {
        dev.control(reset_setup(), &RESET_DATA, v);
        dev.out(EP_CMD, &configure(2048, 0), v);
        let mut st = configure(2048, 0);
        st[82..84].copy_from_slice(&CMD_STATUS.to_le_bytes());
        for _ in 0..2 {
            dev.out(EP_CMD, &st, v);
            dev.in_transfer(EP_DATA, 512, v);
            dev.in_transfer(EP_DATA, 512, v);
        }
        st[82..84].copy_from_slice(&CMD_GET.to_le_bytes());
        dev.out(EP_CMD, &st, v);
        assert_eq!(words(&data(dev.in_transfer(EP_DATA, 512, v)))[0], START_MAGIC);
    }

    #[test]
    fn hold_holds_and_a_partial_comes_once_on_a_cancel() {
        let mut v = Violations::default();
        let mut dev = H4032l::new(FPGA_VERSION);
        dev.faults.parse("hold", "1").unwrap();
        dev.faults.parse("partial", "512").unwrap();
        // Nothing to give before the hold.
        assert_eq!(dev.in_unlinked(EP_DATA, 2048), None);
        up_to_the_stream(&mut dev, &mut v);
        assert_eq!(dev.in_unlinked(EP_DATA, 2048), None);
        // The stream's first read holds the start word and samples 0 to 126.
        assert_eq!(words(&data(dev.in_transfer(EP_DATA, 2048, &mut v)))[0], 127);
        assert_eq!(dev.in_transfer(EP_DATA, 2048, &mut v), Reply::Nak);
        let d = dev.in_unlinked(EP_DATA, 2048).unwrap();
        assert_eq!((d.len(), words(&d)[0]), (512, 639));
        assert_eq!(dev.in_unlinked(EP_DATA, 2048), None, "once");
        assert_eq!(dev.in_transfer(EP_DATA, 2048, &mut v), Reply::Nak, "without release");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn release_lets_the_stream_go_on_where_the_partial_ended() {
        let mut v = Violations::default();
        let mut dev = H4032l::new(FPGA_VERSION);
        for (k, val) in [("hold", "0"), ("partial", "512"), ("release", "1")] {
            dev.faults.parse(k, val).unwrap();
        }
        up_to_the_stream(&mut dev, &mut v);
        assert_eq!(dev.in_transfer(EP_DATA, 2048, &mut v), Reply::Nak);
        assert_eq!(words(&dev.in_unlinked(EP_DATA, 2048).unwrap())[0], 127);
        assert_eq!(words(&data(dev.in_transfer(EP_DATA, 2048, &mut v)))[0], 255);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_partial_is_whole_packets_and_needs_a_hold() {
        let o = |l: &[(&str, &str)]| -> Vec<(String, String)> {
            l.iter().map(|(k, val)| ((*k).to_owned(), (*val).to_owned())).collect()
        };
        assert!(build(&o(&[("hold", "1"), ("partial", "100")])).is_err());
        assert!(build(&o(&[("partial", "512")])).is_err());
        assert!(build(&o(&[("release", "1")])).is_err());
        assert!(build(&o(&[("hold", "1"), ("partial", "512"), ("release", "1")])).is_ok());
    }

    #[test]
    fn options_and_faults() {
        let o = |k: &str, val: &str| vec![(k.to_owned(), val.to_owned())];
        for (k, val) in [("stall", "reset@0"), ("stall", "load"), ("fpga", "x"), ("end", "0xz")] {
            assert!(build(&o(k, val)).is_err(), "{k}={val}");
        }
        assert!(build(&o("fpga", "0")).is_ok());
        assert!(build(&o("nope", "1")).is_err());

        let mut v = Violations::default();
        let mut dev = H4032l::new(FPGA_VERSION);
        dev.faults.parse("stall", "out@1,in@2,reset@1").unwrap();
        assert_eq!(dev.out(EP_CMD, &packet(CMD_STATUS), &mut v), Reply::Stall);
        assert_eq!(dev.in_transfer(EP_DATA, 512, &mut v), Reply::Nak, "not taken");
        dev.out(EP_CMD, &packet(CMD_STATUS), &mut v);
        assert!(matches!(dev.in_transfer(EP_DATA, 512, &mut v), Reply::Data(_)));
        assert_eq!(dev.in_transfer(EP_DATA, 512, &mut v), Reply::Stall);
        assert_eq!(dev.control(reset_setup(), &RESET_DATA, &mut v), Reply::Stall);
        assert!(v.is_empty(), "{v:?}");

        let mut dev = H4032l::new(FPGA_VERSION);
        dev.faults.parse("stale", "2").unwrap();
        dev.faults.parse("end", "0x12345678").unwrap();
        dev.out(EP_CMD, &packet(CMD_STATUS), &mut v);
        assert_eq!(data(dev.in_transfer(EP_DATA, 512, &mut v)), vec![0; STALE]);
        assert_eq!(data(dev.in_transfer(EP_DATA, 512, &mut v)), vec![0; STALE]);
        assert_eq!(words(&data(dev.in_transfer(EP_DATA, 512, &mut v)))[0], STATUS_MAGIC);
        let w = capture(&mut dev, &configure(2048, 0), &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(w[0], START_MAGIC);
        assert_eq!(w.len(), 2050);
        assert_eq!(*w.last().unwrap(), 0x1234_5678);
    }
}
