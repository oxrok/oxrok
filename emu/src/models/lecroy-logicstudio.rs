// Counterpart in sigrok:
//   libsigrok/src/hardware/lecroy-logicstudio/api.c (the ids, the firmware, the renumeration wait),
//   libsigrok/src/hardware/lecroy-logicstudio/protocol.c (the requests, registers, bitstream upload, trigger layout, status packets, sample memory),
//   libsigrok/src/hardware/lecroy-logicstudio/protocol.h,
//   Copyright (C) 2015 Tilman Sauerbeck <tilman@code-monkey.de>,
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Romain Tartière
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The `LeCroy` `LogicStudio` 16 (`05ff:a001`, `05ff:a002` with its
//! firmware) as libsigrok's `lecroy-logicstudio` driver sees it, built
//! on the EZ-USB shell ([`crate::shells::ezusb`]).
//!
//! Where the facts come from. Everything is from the C driver (`api.c`,
//! `protocol.c`, `protocol.h`); no `LeCroy` firmware, datasheet or
//! recording is available here, and the driver's own comments call
//! most register meanings unknown ("magic", `REG_UNK*`, "Not sure of the
//! meaning yet"). What the driver does not pin down is the model's
//! choice, marked "assumed" below.
//!
//! - **Ids** (`api.c`): vendor `0x05ff` (`LeCroy`), product `0xa001`
//!   while the FX2 lacks its firmware, `0xa002` once it runs. Strings,
//!   `bcdDevice` (0x0000) and the power are assumed; the driver reads no
//!   string. The firmware file is `lecroy-logicstudio16-fx2lp.fw`, so
//!   the chip is taken to be an FX2LP (16 KiB); its stand-in size is the
//!   model's (`tools/emu/lecroy-logicstudio.firmware`).
//! - **Before the firmware** the FX2 enumerates as `05ff:a001` (a `0xC0`
//!   EEPROM, assumed) and takes `ezusb_upload_firmware()`'s upload:
//!   `SET_CONFIGURATION` 0 (`USB_CONFIGURATION`, which `ezusb.c` sends
//!   whatever the device's state, so every recorded scan starts with
//!   it), the reset hold, the image, the release, held byte for byte
//!   against the stand-in; then it renumerates as `05ff:a002` on the
//!   same port, which `dev_open()` waits up to 3 s of wall time for.
//! - **The firmware's device**: high speed, interface 0 with interrupt
//!   IN 1 (`EP_INTR`, the 32 byte status packets, `INTR_BUF_SIZE`; a
//!   32 byte endpoint and `bInterval` 1 are assumed), bulk IN 2
//!   (`EP_BULK`, the samples) and bulk OUT 6 (`EP_BITSTREAM`), 512
//!   bytes each (assumed, the FX2's large endpoints).
//! - **Vendor requests** (`protocol.c`), all with `wIndex` 5444
//!   (0x1544) unless said:
//!
//!   | request | dir | `wValue` | data | what |
//!   |---|---|---|---|---|
//!   | `0xb1` | IN | register | 2 bytes, big endian | read a register |
//!   | `0xb1` | OUT | 0x12 | (register, value BE) triplets | write registers (FPGA) |
//!   | `0xb1` | OUT | 0, `wIndex` 0 | one triplet | write a threshold (before an upload) |
//!   | `0xb2` | OUT | 0x12 | 16 bytes | "status register" |
//!   | `0xb3` | OUT | 7 | none | start an FPGA upload |
//!   | `0xb4` | IN | 7 | 1 byte | upload verified: 1 |
//!
//!   The driver writes at most 32 registers a transfer
//!   (`write_registers_sync()`), the fetch's 12 asynchronously.
//! - **The FPGA** takes one of two bitstreams over bulk OUT 6 after
//!   `0xb3`, in 2048 byte chunks (`FPGA_FIRMWARE_CHUNK_SIZE`), exactly
//!   464196 bytes (`FPGA_FIRMWARE_SIZE`): `lecroy-logicstudio16-8.bitstream`
//!   samples 8 channels (one channel group), `-16` all 16 (two groups).
//!   The model holds the bytes against the stand-ins and answers `0xb4`
//!   with 1 once a whole one is in, 0 before (and for anything else).
//!   Registers 0x0f to 0x12, read after the upload until two reads of
//!   all four agree (`wait_for_dev_to_settle()`), answer 0 (assumed;
//!   the driver compares, never interprets them). The board keeps its
//!   FX2 firmware and FPGA design across attaches, as it keeps power.
//! - **The registers** the driver writes (`protocol.c`'s `REG_*`): the
//!   thresholds 0x22 to 0x24 (the driver's constants 0x003e, 0x00c3,
//!   0x00c2), the channel group 0x21 (1: the upper group in 8 channel
//!   mode), the trigger configuration through 0x18 (each byte as three
//!   writes `addr << 8 | byte`, with bit 15 set in the middle one, the
//!   stages at 0x00 and 0x40, 45 bytes each), the stage combination
//!   0x1b (`TRIGGER_OP_*`), and the capture registers: acquisition id
//!   0x00, rate configuration 0x02 (`samplerates[]`'s `cfg`), pre- and
//!   post-trigger counts 0x03 to 0x06, arm 0x07, fetch 0x08 and the
//!   `REG_UNK*` the driver fills with fixed or computed values. The
//!   model checks each transfer against the one the driver builds:
//!   the stop (`ARM` x, x|2), the arm (17 registers in
//!   `lls_start_acquisition()`'s order), the fetch (12 in
//!   `fetch_samples_async()`'s), the trigger transfers (15 writes of
//!   0x18), and single writes of a threshold, the channel group or the
//!   combination.
//! - **A capture**, the model's reading of the arm transfer: the
//!   counter starts at the arm (sample 0, time 0; assumed). The FPGA
//!   stores PRETRIG x d samples (d = 8 / groups, the unit of
//!   `transform_sample_count()`) before it arms the trigger, then stops
//!   POSTTRIG x d - 0x1c samples after the trigger sample, that sample
//!   included: `transform_sample_count()` adds 0x1c + d to both counts,
//!   and the model reads the 0x1c as the FPGA's pipeline delay
//!   (assumed; with it the trigger lands within a few samples of the
//!   capture ratio). Samples go round a memory of 40960 bytes
//!   (`SAMPLE_BUF_SIZE`): sample s at s mod 20480 as two bytes, D0-D7
//!   then D8-D15, with 16 channels; at s mod 40960 as one byte, the
//!   group register 0x21 chooses, with 8. The memory holds 0x00 where
//!   nothing was stored (assumed). A trigger stage holds when its levels
//!   (ones, zeroes) all hold and, if it has edges, any of its edges
//!   (rising, falling, either) happens at that sample ("they are always
//!   OR'd within a single stage", `program_trigger()`); `TRIGGER_OP_A`
//!   fires on stage A, `TRIGGER_OP_A_AND_B` when both hold at the same
//!   sample. The trigger unit sees all 16 inputs in either mode
//!   (assumed). The model decodes the stages from the 45 bytes with the
//!   inverse of `program_trigger()`'s layout (rising at 0x0a, rising or
//!   falling as the complement at 0x00, any edge at 0x05, ones or zeroes
//!   at 0x0f, ones at 0x14 and, for D15, 0x19) and flags bytes that
//!   layout does not produce.
//! - **The status packets** on interrupt IN 1, 32 bytes: the
//!   acquisition id big endian at 2, the state at 0x1f (0x09 storing the
//!   pre-trigger samples, 0x0a waiting for the trigger, 0x0b storing the
//!   post-trigger samples, 0x04 complete; `handle_intr_data()`), and in
//!   the last one the time of the latest sample at 6 and of the trigger
//!   at 0x0c, 48 bit big endian in units of 8 ns, the trigger's
//!   nanoseconds below 8 in the low bits of 0x13. Other bytes are 0
//!   (assumed). The trigger time carries the latency the driver removes
//!   (`trigger_sample_offset` of the rate, 8 more with 16 channels,
//!   0x18, one sample period): the model reports the time from which
//!   the driver's arithmetic gives back the trigger sample, the only
//!   account of that latency there is. That makes the check of the
//!   trigger's place self-referential: `trigger_ns()` is the inverse of
//!   `handle_intr_data()`, so the driver always finds the trigger where
//!   the model put it. It catches errors in the packet's layout, in the
//!   memory's wrap and in the window shown, not in the latency, which
//!   nothing here can check.
//! - **The fetch** (`fetch_samples_async()`) makes the FPGA send the
//!   whole memory, from byte 0, on bulk IN 2; the driver reads it at the
//!   positions it computes (`sample_to_byte_offset()`), so the order is
//!   the memory's. Its first register (`REG_UNK1`) is the first sample
//!   the driver will show, in units of 8 bytes; the model checks it
//!   against the times it reported.
//! - **Inputs**: input c at sample s is bit c of s (D0 toggles every
//!   sample, D15 every 32768), or all low with `still=1` (nothing
//!   connected), with which an edge trigger never fires.
//!
//! **What checks the model.** sigrok-cli cannot start a capture with
//! this driver: it runs a session only with `--samples`, `--time`,
//! `--frames` or `--continuous` (`main.c`), and the driver offers none of
//! the keys they need (`api.c` `devopts`), so each fails before the
//! acquisition. The CLI oracle cases (`tools/emu/lecroy-logicstudio.sh`)
//! therefore reach the scan, the FX2 upload, the renumeration, the open
//! and the options only. The capture side is library-verified, not
//! CLI-verified: the `lib ...` cases run `tools/libsigrok-run.c`, which
//! starts the session through libsigrok with sigrok-cli's arguments and
//! no limit (`emu_case --libsigrok`), at every rate class, both channel
//! modes, both groups, ratios 0 to 100, one and two stages, levels and
//! edges, a memory that wraps, and each fault. They log no violation,
//! the samples come out as the inputs' counter without a break and the
//! trigger at the capture ratio (10004 of 20000 at 50 % with 16
//! channels, 20000 of 40000 with 8; after the last sample at 100 %);
//! the runs that never end (no trigger, a trigger that never fires, a
//! stalled fetch, a short status packet, a halted sample endpoint, an
//! unplug while waiting or fetching) are stopped after 3 s. The unit
//! tests drive the model the same way.
//!
//! **Emulation artifacts**: each status packet answers one interrupt IN
//! transfer, the state advancing one step per transfer once armed; with
//! nothing new to tell (before an arm, after the last packet, while the
//! trigger does not fire) the endpoint NAKs, so the driver's 100 ms
//! transfer times out (the real firmware's rhythm is unknown). The
//! capture is computed at the arm, whatever the rate, and each bulk IN
//! of the fetch completes, filled to its length, per transfer the host
//! submits. A STALL fault answers the one request it hits (a real
//! STALL halts the endpoint until cleared; the driver gives up on the
//! first one, so the difference does not show).
//!
//! **Pedantic checks**: the FX2 upload as `ezusb.c` sends it; the
//! bitstream (its chunks, its size, its bytes); every request's type,
//! `wValue`, `wIndex` and length; register transfers other than the
//! driver's; the fixed values (thresholds, status bytes, the `REG_UNK*`
//! values); register writes to a blank FPGA other than the driver's stop
//! before its first upload; an upload (`0xb3`) not right after the stop
//! and the three thresholds with `wValue` 0, and those thresholds
//! anywhere else; an `ARM` or `FETCH` base other than the driver's
//! (`magic_arm_trigger` 0x00 and `magic_fetch_samples` 0 after an
//! upload's thresholds, 0x0c from an arm and 1 from a fetch, even one
//! that failed; a stop takes 0x00 as well after an arm, the base of a
//! new process's driver, which the board cannot tell from the old
//! one); a settle read before a design is loaded, or
//! out of the 0x0f-0x12 order; the trigger layout; an arm without the
//! design, the thresholds, the status write, the channel group or the
//! trigger since the upload, or with counts no capture ratio gives; a
//! group other than 0 with 16 channels; a fetch with no capture
//! complete or another start; bulk IN transfers of other lengths than
//! the driver's (17 KiB, then what is left, 16 KiB at most, rounded up
//! to whole packets as on the bus); a firmware
//! upload to the running firmware.
//!
//! **The driver's bugs** (libsigrok `0bc24877`), which the model shows
//! as the driver meets them:
//!
//! - `upload_fpga_bitstream()` logs a failed `0xb3` with
//!   `libusb_error_name(ret)`, `ret` being `SR_ERR`, not the transfer's
//!   `r` (`protocol.c:452-454`): `stall=upload` logs
//!   `LIBUSB_ERROR_IO` whatever the error.
//! - It returns from a failed verify read without
//!   `sr_resource_close()` (`protocol.c:490-492`): the bitstream's
//!   resource leaks (`stall=verify`).
//! - `recv_bulk_transfer()` never looks at the transfer's status
//!   (`protocol.c:1017-1027`): a failed or stalled sample transfer adds
//!   its 0 bytes and is resubmitted for ever (`bulk_stall=1`, an unplug
//!   during the fetch).
//! - With no trigger `lls_start_acquisition()` submits the interrupt
//!   transfer and arms nothing (`protocol.c:1119`), and
//!   `recv_intr_transfer()` resubmits only a completed transfer
//!   (`protocol.c:926-933`): the first 100 ms timeout ends the polling
//!   and the acquisition waits for ever. `configure_trigger()` ignores
//!   matches on disabled channels (`protocol.c:674`), so a trigger only
//!   on disabled channels does the same.
//! - `send_samples()` sends what is left after its 4096 sample chunks
//!   even when nothing is (`protocol.c:980-989`): with 8 channels, a
//!   stretch of a whole number of chunks ends in a logic packet of 0
//!   bytes, on which libsigrok's bits and hex outputs die (`lib 8ch
//!   upper wraps bits crash`).
//! - 1 GHz is in `lls_set_samplerate()`'s table but not in the list
//!   `config_list()` gives: `samplerate=1G` is taken, and never shown.
//! - The driver lists no scan options (`config_list()` answers
//!   `SR_CONF_SCAN_OPTIONS` with `SR_ERR_NA`), so libsigrok refuses any,
//!   `conn=` included, and a board cannot be chosen among several.
//!
//! **Faults**, all off by default: `stall=NAME[:N]` STALLs the request
//! (every one, or its Nth since the model was built; comma lists), NAME
//! one of `write`, `read`, `status`, `upload`, `verify`; `verify=N`
//! answers the first N verify reads 0 (the FPGA still configuring; 4
//! or more and the driver gives up); `settle=N` makes registers
//! 0x0f-0x12 change for the first N reads of the four (100 or more and
//! the driver gives up); `stall_bitstream=N` STALLs the Nth chunk of an
//! upload; `stale=N` sends the first N status packets after the arm
//! with the previous acquisition id; `odd_state=0xNN` sends one packet
//! in that state before the first; `intr_short=N` sends the status
//! packets N bytes long; `bulk_short=N` makes the first transfer of the
//! fetch carry N bytes; `bulk_stall=1` STALLs every bulk IN of the
//! fetch; `renum=0` is a firmware that never comes back after the FX2
//! upload (the code runs but never renumerates: the device stays on the
//! bus as `05ff:a001` and STALLs every request); `still=1` and
//! `flashed=1` (the FX2 firmware already running) are board states.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "lecroy-logicstudio";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "LeCroy LogicStudio 16 (05ff:a001, EZ-USB, renumerates as 05ff:a002, FPGA \
                         bitstream over bulk EP6, trigger-only captures); flashed=1, still=1; \
                         faults: stall=NAME[:N], verify=N, settle=N, stall_bitstream=N, stale=N, \
                         odd_state=0xNN, intr_short=N, bulk_short=N, bulk_stall=1, renum=0";

/// `LeCroy`'s vendor id (`api.c`).
pub const VID: u16 = 0x05ff;
/// The product id without the FX2 firmware.
pub const PID_BARE: u16 = 0xa001;
/// The product id with it.
pub const PID_FIRMWARE: u16 = 0xa002;
/// The FX2 firmware and its stand-in's size.
pub const FX2_FIRMWARE: (&str, usize) = ("lecroy-logicstudio16-fx2lp.fw", 6214);
/// The bitstream for 8 channels and its size (`FPGA_FIRMWARE_SIZE`).
pub const BITSTREAM_8: (&str, usize) = ("lecroy-logicstudio16-8.bitstream", 464_196);
/// The bitstream for 16 channels.
pub const BITSTREAM_16: (&str, usize) = ("lecroy-logicstudio16-16.bitstream", 464_196);
/// `FPGA_FIRMWARE_CHUNK_SIZE`.
const CHUNK: usize = 2048;
/// `SAMPLE_BUF_SIZE`: the sample memory, bytes.
pub const MEMORY: usize = 40960;
/// The interrupt packet (`INTR_BUF_SIZE`).
pub const STATUS_SIZE: usize = 32;

const CTRL_IN: u8 = 0xc0;
const CTRL_OUT: u8 = 0x40;
const REQ_REGS: u8 = 0xb1;
const REQ_STATUS: u8 = 0xb2;
const REQ_UPLOAD: u8 = 0xb3;
const REQ_VERIFY: u8 = 0xb4;
/// `wIndex` of every request but the thresholds before an upload.
const MAGIC_INDEX: u16 = 5444;

const REG_ACQUISITION_ID: u8 = 0x00;
const REG_SAMPLERATE: u8 = 0x02;
const REG_PRETRIG_LO: u8 = 0x03;
const REG_PRETRIG_HI: u8 = 0x04;
const REG_POSTTRIG_LO: u8 = 0x05;
const REG_POSTTRIG_HI: u8 = 0x06;
const REG_ARM_TRIGGER: u8 = 0x07;
const REG_FETCH_SAMPLES: u8 = 0x08;
const REG_UNK1_LO: u8 = 0x09;
const REG_UNK1_HI: u8 = 0x0a;
const REG_UNK2_LO: u8 = 0x0b;
const REG_UNK2_HI: u8 = 0x0c;
const REG_UNK3_LO: u8 = 0x0d;
const REG_UNK3_HI: u8 = 0x0e;
const REG_UNK4_LO: u8 = 0x0f;
const REG_UNK4_HI: u8 = 0x10;
const REG_UNK5_LO: u8 = 0x11;
const REG_UNK5_HI: u8 = 0x12;
const REG_UNK6_LO: u8 = 0x13;
const REG_UNK6_HI: u8 = 0x14;
const REG_UNK0_LO: u8 = 0x15;
const REG_UNK0_HI: u8 = 0x16;
const REG_TRIGGER_CFG: u8 = 0x18;
const REG_TRIGGER_COMBINE_OP: u8 = 0x1b;
const REG_SELECT_CHANNELS: u8 = 0x21;
const REG_THRESH_EXTERNAL: u8 = 0x22;
const REG_THRESH_LOWER: u8 = 0x23;
const REG_THRESH_UPPER: u8 = 0x24;

/// The thresholds and their only values (`lls_setup_acquisition()`).
const THRESHOLDS: [(u8, u16); 3] =
    [(REG_THRESH_LOWER, 0x00c3), (REG_THRESH_UPPER, 0x00c2), (REG_THRESH_EXTERNAL, 0x003e)];
/// The bytes of `0xb2` (`status_reg_value`).
const STATUS_BYTES: [u8; 16] = [1, 0, 7, 8, 9, 10, 11, 12, 13, 14, 1, 2, 3, 4, 5, 6];
/// `TRIGGER_OP_A`.
const OP_A: u16 = 0x1000;
/// `TRIGGER_OP_A_AND_B`.
const OP_A_AND_B: u16 = 0x4000;
/// `TRIGGER_CFG_SIZE`.
const STAGE_SIZE: usize = 45;
/// Where the stages start in the trigger address space.
const STAGE_BASE: [u8; 2] = [0x00, 0x40];

/// `calc_unk0()`: `(5000 + 63) | 63` and `(5000 + 63) & !63`.
const UNK0: u32 = (5000 + 63) | 63;
const UNK3: u32 = (5000 + 63) & !63;

/// `samplerates[]` of `protocol.c`: rate, `trigger_sample_offset`, `cfg`.
pub const RATES: [(u64, i64, u16); 19] = [
    (1_000_000_000, -24, 0x1f),
    (500_000_000, -6, 0x00),
    (250_000_000, -4, 0x01),
    (100_000_000, 2, 0x03),
    (50_000_000, 4, 0x04),
    (25_000_000, 8, 0x05),
    (10_000_000, 4, 0x07),
    (5_000_000, 8, 0x08),
    (2_500_000, 8, 0x09),
    (1_000_000, 8, 0x0b),
    (500_000, 8, 0x0c),
    (250_000, 8, 0x0d),
    (100_000, 8, 0x0f),
    (50_000, 8, 0x10),
    (25_000, 8, 0x11),
    (10_000, 8, 0x13),
    (5_000, 8, 0x14),
    (2_500, 8, 0x15),
    (1_000, 8, 0x17),
];

/// The status states (`handle_intr_data()`).
pub const STATE_PRE: u8 = 0x09;
/// Waiting for the trigger.
pub const STATE_ARMED: u8 = 0x0a;
/// Storing the post-trigger samples.
pub const STATE_POST: u8 = 0x0b;
/// Complete.
pub const STATE_DONE: u8 = 0x04;

/// The requests the faults name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Request {
    Write,
    Read,
    Status,
    Upload,
    Verify,
}

const REQUESTS: [(&str, Request); 5] = [
    ("write", Request::Write),
    ("read", Request::Read),
    ("status", Request::Status),
    ("upload", Request::Upload),
    ("verify", Request::Verify),
];

/// A STALL aimed at a request: every one (`nth` `None`) or the Nth.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Shot {
    request: Request,
    nth: Option<u32>,
}

fn shots(k: &str, v: &str) -> Result<Vec<Shot>> {
    let mut out = Vec::new();
    for item in v.split(',').filter(|s| !s.is_empty()) {
        let (name, nth) = match item.split_once(':') {
            Some((name, n)) => {
                let n = n
                    .parse::<u32>()
                    .ok()
                    .filter(|n| *n > 0)
                    .ok_or_else(|| Error(format!("{k}={item}: N from 1")))?;
                (name, Some(n))
            }
            None => (item, None),
        };
        let request =
            REQUESTS.iter().find(|(n, _)| *n == name).map(|(_, r)| *r).ok_or_else(|| {
                let names: Vec<&str> = REQUESTS.iter().map(|(n, _)| *n).collect();
                Error(format!("{k}={name}: one of {}", names.join(", ")))
            })?;
        out.push(Shot { request, nth });
    }
    Ok(out)
}

fn flag(k: &str, v: &str) -> Result<bool> {
    match v {
        "0" => Ok(false),
        "1" => Ok(true),
        _ => Err(Error(format!("{k}={v}: 0 or 1"))),
    }
}

fn byte(k: &str, v: &str) -> Result<u8> {
    let parsed = match v.strip_prefix("0x") {
        Some(hex) => u8::from_str_radix(hex, 16).ok(),
        None => v.parse().ok(),
    };
    parsed.ok_or_else(|| Error(format!("{k}={v}: a byte, 0 to 255 or 0x00 to 0xff")))
}

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Faults {
    stall: Vec<Shot>,
    verify: u32,
    settle: u32,
    stall_bitstream: Option<u32>,
    stale: u32,
    odd_state: Option<u8>,
    intr_short: Option<usize>,
    bulk_short: Option<usize>,
    bulk_stall: bool,
    no_renumeration: bool,
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let count = |v: &str| v.parse::<u32>().map_err(|_| Error(format!("{k}={v}: a count")));
        match k {
            "stall" => self.stall = shots(k, v)?,
            "verify" => self.verify = count(v)?,
            "settle" => self.settle = count(v)?,
            "stall_bitstream" => {
                let n = count(v)?;
                if n == 0 {
                    return Err(Error(format!("{k}={v}: N from 1")));
                }
                self.stall_bitstream = Some(n);
            }
            "stale" => self.stale = count(v)?,
            "odd_state" => self.odd_state = Some(byte(k, v)?),
            "intr_short" => {
                let n = count(v)? as usize;
                if n >= STATUS_SIZE {
                    return Err(Error(format!("{k}={v}: fewer than {STATUS_SIZE} bytes")));
                }
                self.intr_short = Some(n);
            }
            "bulk_short" => {
                let n = count(v)? as usize;
                if n == 0 || n >= MEMORY {
                    return Err(Error(format!("{k}={v}: 1 to {}", MEMORY - 1)));
                }
                self.bulk_short = Some(n);
            }
            "bulk_stall" => self.bulk_stall = flag(k, v)?,
            "renum" => self.no_renumeration = !flag(k, v)?,
            _ => return Ok(false),
        }
        Ok(true)
    }

    fn stalls(&self, request: Request, count: u32) -> bool {
        self.stall.iter().any(|s| s.request == request && s.nth.is_none_or(|n| n == count))
    }
}

/// A board from the `-o` options: `flashed=0|1`, `still=0|1` and the
/// faults (see the module's doc).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut flashed = false;
    let mut still = false;
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "flashed" => flashed = flag(k, v)?,
            "still" => still = flag(k, v)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let mut model = LogicStudio::new(flashed, still);
    model.faults = faults;
    Ok(Box::new(model))
}

/// The descriptors of the running FX2 firmware.
pub fn firmware_descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 3, [0xff, 0xff, 0xff], 0));
    body.extend_from_slice(&descriptor::endpoint(0x81, crate::model::INTERRUPT, 32, 1));
    body.extend_from_slice(&descriptor::endpoint(0x82, crate::model::BULK, 512, 0));
    body.extend_from_slice(&descriptor::endpoint(0x06, crate::model::BULK, 512, 0));
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor: VID,
        product: PID_FIRMWARE,
        device: 0x0000,
        strings: [0, 0, 0],
        configurations: 1,
    };
    Descriptors { speed: Speed::High, device: dev.bytes(), configs: vec![config], strings: vec![] }
}

/// The conditions of one trigger stage, as `struct trigger_config`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stage {
    /// Rising edges.
    pub rising: u16,
    /// Falling edges.
    pub falling: u16,
    /// Either edge.
    pub any: u16,
    /// High levels.
    pub ones: u16,
    /// Low levels.
    pub zeroes: u16,
}

impl Stage {
    /// `program_trigger()`'s 45 bytes for this stage; `last` is the
    /// final stage, with its extra bytes.
    pub fn encode(&self, last: bool) -> [u8; STAGE_SIZE] {
        let mut b = [0u8; STAGE_SIZE];
        let put = |b: &mut [u8; STAGE_SIZE], at: usize, v: u16| {
            b[at..at + 2].copy_from_slice(&v.to_le_bytes());
        };
        put(&mut b, 0x00, !(self.rising | self.falling));
        put(&mut b, 0x05, self.rising | self.falling | self.any);
        if self.ones | self.zeroes != 0 {
            b[0x09] = 0x10;
        }
        put(&mut b, 0x0a, self.rising);
        put(&mut b, 0x0f, self.ones | self.zeroes);
        b[0x13] = 0x10;
        put(&mut b, 0x14, self.ones | 0x8000);
        put(&mut b, 0x19, if self.ones == 0x01 { self.ones << 1 } else { self.ones | 0x0001 });
        if last {
            b[0x09] = 0x10;
            b[0x28..0x2c].fill(0xff);
            b[0x2c] = 0x80;
        }
        b
    }

    /// The conditions the 45 bytes hold (the inverse of [`Stage::encode`]).
    pub fn decode(b: &[u8; STAGE_SIZE]) -> Stage {
        let get = |at: usize| u16::from_le_bytes([b[at], b[at + 1]]);
        let rising = get(0x0a);
        let rf = !get(0x00);
        let level = get(0x0f);
        // D15's high level shares bit 15 of 0x14 with the marker the
        // driver always sets; 0x19 keeps it.
        let ones = (get(0x14) & 0x7fff) | (get(0x19) & 0x8000);
        Stage { rising, falling: rf & !rising, any: get(0x05) & !rf, ones, zeroes: level & !ones }
    }

    fn is_empty(&self) -> bool {
        *self == Stage::default()
    }

    /// Whether the stage holds at a sample of value `now`, after `before`.
    pub fn holds(&self, before: u16, now: u16) -> bool {
        if now & self.ones != self.ones || now & self.zeroes != 0 {
            return false;
        }
        let edges = self.rising | self.falling | self.any;
        if edges == 0 {
            return true;
        }
        let up = now & !before;
        let down = !now & before;
        (up & self.rising) | (down & self.falling) | ((up | down) & self.any) != 0
    }
}

/// What an arm transfer asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Arm {
    id: u16,
    rate: u64,
    offset: i64,
    groups: u8,
    /// `PRETRIG` and `POSTTRIG`, in units of 8 / groups samples.
    pre: u64,
    post: u64,
}

/// A capture, as the arm set it up.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Capture {
    id: u16,
    rate: u64,
    offset: i64,
    groups: u8,
    /// The trigger sample, `None` when it never fires.
    trigger: Option<u64>,
    /// The last sample stored.
    last: u64,
    /// Status packets still to send.
    states: Vec<u8>,
    /// Packets sent with the previous id (fault `stale`).
    stale: u32,
    /// The packet that ended it went out.
    done: bool,
}

impl Capture {
    /// Nanoseconds per sample, the driver's `samplerate_divider`.
    fn divider(&self) -> u64 {
        1_000_000_000 / self.rate
    }

    /// The 8 ns count of the latest sample.
    fn time_latest(&self) -> u64 {
        self.last * self.divider() / 8
    }

    /// The trigger time in ns with the latency the driver removes; see
    /// the module's doc.
    fn trigger_ns(&self) -> u64 {
        let t = self.trigger.unwrap_or(0);
        let g8 = if self.groups > 1 { 8 } else { 0 };
        let div = self.divider();
        let base = if div > 1 { t * div } else { t - 1 };
        (base as i64 - self.offset + g8 + 0x18) as u64
    }

    /// The first sample the driver will show, as `handle_intr_data()`
    /// computes it from the times reported.
    fn driver_earliest(&self) -> u64 {
        let p = 8 / u64::from(self.groups);
        let latest = self.time_latest() * 8 / self.divider();
        let aligned = if latest > p { latest - p } else { latest } & !(p - 1);
        aligned.wrapping_sub(thousands(self.groups) * 1000)
    }
}

/// `num_thousand_samples` for a number of groups.
fn thousands(groups: u8) -> u64 {
    if groups > 1 { 20 } else { 40 }
}

/// `transform_sample_count()`.
fn transform(samples: u64, groups: u8) -> u64 {
    let d = 8 / u64::from(groups);
    (samples + 0x1c + d).div_ceil(d)
}

/// The FPGA and what the FX2 knows of it.
#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)]
struct Fpga {
    /// The design loaded: the number of channel groups.
    design: Option<u8>,
    /// The bitstream being uploaded, since `0xb3`.
    upload: Option<Vec<u8>>,
    /// `0xb3` came since the FPGA came up.
    uploaded: bool,
    chunks: u32,
    short_chunk: bool,
    /// Verify reads since the upload.
    verifies: u32,
    /// Settle reads since the upload.
    settle_reads: u32,
    /// Since the upload: status written, thresholds written (bits per
    /// `THRESHOLDS` entry), the channel group.
    status: bool,
    thresholds: u8,
    group: Option<u16>,
    /// The trigger bytes, which addresses were written, the combination.
    stages: [[u8; STAGE_SIZE]; 2],
    written: [u64; 2],
    combine: Option<u16>,
    capture: Option<Capture>,
    memory: Vec<u8>,
    /// The bytes of the memory sent on bulk IN 2 since the fetch.
    fetch: Option<usize>,
    /// Bulk IN transfers since the fetch.
    fetch_transfers: u32,
    bulk_ready: bool,
    bulk_short: Option<usize>,
}

impl Fpga {
    fn blank() -> Fpga {
        Fpga {
            design: None,
            upload: None,
            uploaded: false,
            chunks: 0,
            short_chunk: false,
            verifies: 0,
            settle_reads: 0,
            status: false,
            thresholds: 0,
            group: None,
            stages: [[0; STAGE_SIZE]; 2],
            written: [0; 2],
            combine: None,
            capture: None,
            memory: vec![0; MEMORY],
            fetch: None,
            fetch_transfers: 0,
            bulk_ready: false,
            bulk_short: None,
        }
    }

    /// A design loaded: everything set since the last upload is gone.
    fn load(&mut self, groups: u8) {
        let memory = std::mem::take(&mut self.memory);
        *self = Fpga::blank();
        self.design = Some(groups);
        self.memory = memory;
    }
}

/// What the FX2 runs.
#[derive(Clone, Debug)]
enum Stage8051 {
    /// The boot loader, waiting for the firmware.
    Boot(Box<Loader>),
    /// The `LogicStudio` firmware.
    Running,
    /// A firmware that never came back (fault `renum=0`).
    Gone,
}

/// The emulated board.
pub struct LogicStudio {
    boot: Descriptors,
    firmware: Descriptors,
    stage: Stage8051,
    renumerate: bool,
    still: bool,
    fpga: Fpga,
    /// The steps of the upload's prelude seen in order: the stop, then
    /// each threshold with `wValue` 0 (`lls_setup_acquisition()`).
    prelude: usize,
    /// The driver's `magic_arm_trigger` and `magic_fetch_samples`: 0
    /// once a new design has its thresholds, 0x0c from an arm and 1 from
    /// a fetch, written or not (the driver sets them before the
    /// transfer). A failed upload leaves them as they were.
    arm_base: u16,
    fetch_base: u16,
    /// Requests seen per kind, for the faults' Nth.
    counts: [u32; 5],
    faults: Faults,
}

impl LogicStudio {
    /// A board; `flashed` starts it with the FX2 firmware running,
    /// `still` with nothing on its inputs.
    pub fn new(flashed: bool, still: bool) -> LogicStudio {
        let stage = if flashed {
            Stage8051::Running
        } else {
            Stage8051::Boot(Box::new(Loader::new(Chip::Fx2lp)))
        };
        LogicStudio {
            boot: ezusb::bare_descriptors(VID, PID_BARE, 0x0000),
            firmware: firmware_descriptors(),
            stage,
            renumerate: false,
            still,
            fpga: Fpga::blank(),
            prelude: 0,
            arm_base: 0x00,
            fetch_base: 0,
            counts: [0; 5],
            faults: Faults::default(),
        }
    }

    /// Whether the firmware runs.
    pub fn running(&self) -> bool {
        matches!(self.stage, Stage8051::Running)
    }

    /// The inputs at sample `s`.
    fn input(&self, s: u64) -> u16 {
        if self.still { 0 } else { s as u16 }
    }

    fn count(&mut self, request: Request) -> bool {
        let i = REQUESTS.iter().position(|(_, r)| *r == request).unwrap_or(0);
        self.counts[i] += 1;
        self.faults.stalls(request, self.counts[i])
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let image = ezusb::standin(FX2_FIRMWARE.0, FX2_FIRMWARE.1);
        let Stage8051::Boot(loader) = &mut self.stage else { return Reply::Stall };
        let Some((reply, started)) = loader.control(setup, data, v) else {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} to the FX2 without firmware; only its \
                 boot loader (0xa0) answers",
                setup.request_type, setup.request, setup.value
            ));
            return Reply::Stall;
        };
        if started {
            loader.check_ezusb_c(&image, v);
            if self.faults.no_renumeration {
                self.stage = Stage8051::Gone;
            } else {
                self.stage = Stage8051::Running;
                self.renumerate = true;
            }
        }
        reply
    }

    /// Checks `wValue`, `wIndex` and the length of a request.
    fn expect(setup: Setup, value: u16, index: u16, length: Option<u16>, v: &mut Violations) {
        if setup.value != value || setup.index != index {
            v.report(format!(
                "request {:#04x} with wValue {:#06x} wIndex {}; the driver sends {value:#06x} \
                 {index}",
                setup.request, setup.value, setup.index
            ));
        }
        if let Some(n) = length
            && setup.length != n
        {
            v.report(format!(
                "request {:#04x} of {} bytes; it has {n}",
                setup.request, setup.length
            ));
        }
    }

    fn firmware_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() == crate::model::VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD {
            v.report("firmware load to an FX2 whose LogicStudio firmware runs");
            return Reply::Stall;
        }
        match (setup.request_type, setup.request) {
            (CTRL_IN, REQ_REGS) => {
                if self.count(Request::Read) {
                    return Reply::Stall;
                }
                Self::expect(setup, setup.value, MAGIC_INDEX, Some(2), v);
                Reply::Data(self.read_register(setup.value, v).to_be_bytes().to_vec())
            }
            (CTRL_OUT, REQ_REGS) => {
                if self.count(Request::Write) {
                    // The driver moved its bases before it sent them.
                    match (data.len(), data.first()) {
                        (51, Some(&REG_ARM_TRIGGER)) => self.arm_base = 0x0c,
                        (36, Some(&REG_UNK1_LO)) => self.fetch_base = 1,
                        _ => {}
                    }
                    return Reply::Stall;
                }
                self.write_transfer(setup, data, v);
                Reply::ack()
            }
            (CTRL_OUT, REQ_STATUS) => {
                if self.count(Request::Status) {
                    return Reply::Stall;
                }
                Self::expect(setup, 0x12, MAGIC_INDEX, Some(16), v);
                if data != STATUS_BYTES {
                    v.report(format!(
                        "status register {data:02x?}; the driver writes {STATUS_BYTES:02x?}"
                    ));
                }
                if self.fpga.design.is_none() {
                    v.report("status register written with no FPGA design loaded");
                }
                if self.fpga.thresholds != 0b111 {
                    v.report("status register written before the thresholds of this design");
                }
                self.fpga.status = true;
                Reply::ack()
            }
            (CTRL_OUT, REQ_UPLOAD) => {
                if self.count(Request::Upload) {
                    return Reply::Stall;
                }
                Self::expect(setup, 7, MAGIC_INDEX, Some(0), v);
                if self.fpga.capture.as_ref().is_some_and(|c| !c.done) {
                    v.report("FPGA upload started during a capture");
                }
                if self.prelude != 1 + THRESHOLDS.len() {
                    v.report(
                        "FPGA upload not right after the stop and the three thresholds with \
                         wValue 0",
                    );
                }
                self.prelude = 0;
                let memory = std::mem::take(&mut self.fpga.memory);
                self.fpga = Fpga::blank();
                self.fpga.memory = memory;
                self.fpga.upload = Some(Vec::new());
                self.fpga.uploaded = true;
                Reply::ack()
            }
            (CTRL_IN, REQ_VERIFY) => {
                if self.count(Request::Verify) {
                    return Reply::Stall;
                }
                Self::expect(setup, 7, MAGIC_INDEX, Some(1), v);
                self.fpga.verifies += 1;
                if !self.fpga.uploaded {
                    v.report("upload verified with no upload since the FPGA came up");
                }
                let late = self.fpga.verifies <= self.faults.verify;
                Reply::Data(vec![u8::from(self.fpga.design.is_some() && !late)])
            }
            _ => {
                v.report(format!(
                    "control request {:#04x} {:#04x} value {:#06x}: not one of the LogicStudio's",
                    setup.request_type, setup.request, setup.value
                ));
                Reply::Stall
            }
        }
    }

    /// A register read: only 0x0f to 0x12, in that order, after an
    /// upload (`wait_for_dev_to_settle()`).
    fn read_register(&mut self, reg: u16, v: &mut Violations) -> u16 {
        if self.fpga.design.is_none() {
            v.report(format!("read of register {reg:#04x} with no FPGA design loaded"));
        }
        let at = self.fpga.settle_reads;
        let want = 0x0f + (at % 4) as u16;
        if reg != want {
            v.report(format!(
                "read of register {reg:#04x}; the driver reads 0x0f to 0x12 in order, {want:#04x} \
                 next"
            ));
        }
        self.fpga.settle_reads += 1;
        // Read k of the four: the value changes for the first `settle`
        // reads of the four, then holds (0).
        let group = at / 4;
        if group < self.faults.settle { (group + 1) as u16 } else { 0 }
    }

    /// One `0xb1` OUT: register triplets.
    fn write_transfer(&mut self, setup: Setup, data: &[u8], v: &mut Violations) {
        if data.is_empty() || !data.len().is_multiple_of(3) || data.len() > 32 * 3 {
            v.report(format!(
                "register write of {} bytes; 1 to 32 triplets of register and value",
                data.len()
            ));
            return;
        }
        let regs: Vec<(u8, u16)> =
            data.chunks(3).map(|t| (t[0], u16::from_be_bytes([t[1], t[2]]))).collect();
        let step = self.prelude;
        self.prelude = 0;
        if (setup.value, setup.index) == (0, 0) {
            // The thresholds before an upload, one each, in order.
            match regs[..] {
                [(reg, val)] if THRESHOLDS.contains(&(reg, val)) => {
                    if (1..=THRESHOLDS.len()).contains(&step) && THRESHOLDS[step - 1] == (reg, val)
                    {
                        self.prelude = step + 1;
                    } else {
                        v.report(format!(
                            "threshold {reg:#04x} with wValue 0 out of place; the driver sends \
                             {THRESHOLDS:02x?} so, in that order, between the stop and an upload"
                        ));
                    }
                }
                _ => v.report(format!(
                    "register write {regs:02x?} with wValue 0 wIndex 0; the driver sends only \
                     the thresholds {THRESHOLDS:02x?} so, one at a time"
                )),
            }
            return;
        }
        Self::expect(setup, 0x12, MAGIC_INDEX, None, v);
        match regs[0].0 {
            REG_ARM_TRIGGER if regs.len() == 2 => self.stop(&regs, v),
            REG_ARM_TRIGGER => self.arm(&regs, v),
            REG_UNK1_LO => self.fetch(&regs, v),
            REG_TRIGGER_CFG => self.trigger_cfg(&regs, v),
            _ if regs.len() == 1 => self.single(regs[0], v),
            _ => v.report(format!("register write {regs:02x?}: not one the driver sends")),
        }
    }

    /// `lls_stop_acquisition()`: `ARM` x then x|2, x the driver's
    /// `magic_arm_trigger`, or 0x00 from a new process's driver, which
    /// starts at 0 whatever the board did before.
    fn stop(&mut self, regs: &[(u8, u16)], v: &mut Violations) {
        let (_, x) = regs[0];
        let base = self.arm_base;
        if (x != base && x != 0x00) || regs[1] != (REG_ARM_TRIGGER, x | 2) {
            v.report(format!(
                "stop {regs:02x?}; the driver writes ARM x then x|2, x {base:#04x} (or 0x00 from \
                 a new process)"
            ));
        }
        self.prelude = 1;
        // Before its first upload the driver stops a blank FPGA; the
        // write has nothing to act on.
        self.fpga.capture = None;
        self.fpga.fetch = None;
    }

    /// The trigger counts of some capture ratio, for `groups`.
    fn counts_ok(pre: u64, post: u64, groups: u8) -> bool {
        let total = thousands(groups) * 1000;
        (0..=100).any(|ratio| {
            let p = total * ratio / 100;
            transform(p, groups) == pre && transform(total - p, groups) == post
        })
    }

    /// `lls_start_acquisition()`'s 17 registers.
    fn arm(&mut self, regs: &[(u8, u16)], v: &mut Violations) {
        let arm = self.check_arm(regs, v);
        if regs.len() == 17 {
            // The driver's base is 0x0c from this arm on, whatever came of it.
            self.arm_base = 0x0c;
        }
        if let Some(arm) = arm {
            self.start_capture(&arm);
        }
    }

    /// The arm transfer's checks; what it asks for when it can start.
    fn check_arm(&self, regs: &[(u8, u16)], v: &mut Violations) -> Option<Arm> {
        let names: Vec<u8> = regs.iter().map(|r| r.0).collect();
        let order = [
            REG_ARM_TRIGGER,
            REG_ARM_TRIGGER,
            REG_UNK6_LO,
            REG_UNK6_HI,
            REG_UNK0_LO,
            REG_UNK0_HI,
            REG_UNK4_LO,
            REG_UNK4_HI,
            REG_UNK5_LO,
            REG_UNK5_HI,
            REG_ACQUISITION_ID,
            REG_SAMPLERATE,
            REG_PRETRIG_LO,
            REG_PRETRIG_HI,
            REG_POSTTRIG_LO,
            REG_POSTTRIG_HI,
            REG_ARM_TRIGGER,
        ];
        if names != order {
            v.report(format!(
                "arm with registers {names:02x?}; lls_start_acquisition() writes {order:02x?}"
            ));
            return None;
        }
        let val = |i: usize| regs[i].1;
        let base = self.arm_base;
        if val(0) != base || val(1) != base | 2 || val(16) != 0x0d {
            v.report(format!(
                "arm pulse {:#06x} {:#06x} .. {:#06x}; the driver writes {base:#04x}, {:#04x}, \
                 then 0x0d",
                val(0),
                val(1),
                val(16),
                base | 2
            ));
        }
        let fixed = [val(2), val(3), val(4), val(5), val(6), val(7), val(8), val(9)];
        let want = [0, 0, UNK0 as u16, (UNK0 >> 16) as u16, 0, 0, 0, 0];
        if fixed != want {
            v.report(format!(
                "arm UNK6, UNK0, UNK4, UNK5 {fixed:04x?}; the driver sends {want:04x?}"
            ));
        }
        let Some(groups) = self.fpga.design else {
            v.report("arm with no FPGA design loaded");
            return None;
        };
        let mut missing = Vec::new();
        if self.fpga.thresholds != 0b111 {
            missing.push("the thresholds");
        }
        if !self.fpga.status {
            missing.push("the status register");
        }
        if self.fpga.group.is_none() {
            missing.push("the channel group");
        }
        if self.fpga.written != [(1 << STAGE_SIZE) - 1; 2] || self.fpga.combine.is_none() {
            missing.push("the whole trigger configuration");
        }
        for what in &missing {
            v.report(format!("arm before {what} since the upload"));
        }
        if !missing.is_empty() {
            return None;
        }
        let Some(&(rate, offset, _)) = RATES.iter().find(|r| r.2 == val(11)) else {
            v.report(format!("rate configuration {:#06x}: not one of samplerates[]", val(11)));
            return None;
        };
        let pre = u64::from(val(12)) | u64::from(val(13)) << 16;
        let post = u64::from(val(14)) | u64::from(val(15)) << 16;
        if !Self::counts_ok(pre, post, groups) {
            v.report(format!(
                "pre- and post-trigger counts {pre} and {post}: no capture ratio gives them with \
                 {groups} channel group(s)"
            ));
            return None;
        }
        Some(Arm { id: val(10), rate, offset, groups, pre, post })
    }

    /// The capture an arm starts, computed at once (see the module's doc).
    fn start_capture(&mut self, arm: &Arm) {
        let unit = 8 / u64::from(arm.groups);
        let stored_pre = arm.pre * unit;
        let stored_post = (arm.post * unit).saturating_sub(0x1c).max(1);
        let stage_a = Stage::decode(&self.fpga.stages[0]);
        let stage_b = Stage::decode(&self.fpga.stages[1]);
        let both = self.fpga.combine == Some(OP_A_AND_B);
        // The inputs repeat every 65536 samples: a trigger that has not
        // fired in one period never does.
        let first = stored_pre.max(1);
        let trigger = (first..first + 65_537).find(|&s| {
            let (before, now) = (self.input(s - 1), self.input(s));
            stage_a.holds(before, now) && (!both || stage_b.holds(before, now))
        });
        let last = trigger.map_or(0, |t| t + stored_post - 1);
        let mut states = Vec::new();
        if let Some(odd) = self.faults.odd_state {
            states.push(odd);
        }
        states.push(STATE_PRE);
        states.push(STATE_ARMED);
        if trigger.is_some() {
            states.push(STATE_POST);
            states.push(STATE_DONE);
        }
        states.reverse();
        self.fpga.fetch = None;
        self.fpga.capture = Some(Capture {
            id: arm.id,
            rate: arm.rate,
            offset: arm.offset,
            groups: arm.groups,
            trigger,
            last,
            states,
            stale: self.faults.stale,
            done: false,
        });
        if trigger.is_some() {
            self.store();
        }
    }

    /// Stores the capture's samples in the memory, as the FPGA did
    /// while it ran.
    fn store(&mut self) {
        let Some(c) = &self.fpga.capture else { return };
        let groups = usize::from(c.groups);
        let ring = (MEMORY / groups) as u64;
        let first = c.last.saturating_sub(ring - 1);
        let upper = self.fpga.group == Some(1);
        let (last, still) = (c.last, self.still);
        for s in first..=last {
            let x = if still { 0 } else { s as u16 };
            let at = (s % ring) as usize * groups;
            if groups == 2 {
                self.fpga.memory[at..at + 2].copy_from_slice(&x.to_le_bytes());
            } else {
                self.fpga.memory[at] = if upper { (x >> 8) as u8 } else { x as u8 };
            }
        }
    }

    /// `fetch_samples_async()`'s 12 registers.
    fn fetch(&mut self, regs: &[(u8, u16)], v: &mut Violations) {
        let names: Vec<u8> = regs.iter().map(|r| r.0).collect();
        let order = [
            REG_UNK1_LO,
            REG_UNK1_HI,
            REG_FETCH_SAMPLES,
            REG_FETCH_SAMPLES,
            REG_UNK1_LO,
            REG_UNK1_HI,
            REG_UNK2_LO,
            REG_UNK2_HI,
            REG_UNK3_LO,
            REG_UNK3_HI,
            REG_FETCH_SAMPLES,
            REG_FETCH_SAMPLES,
        ];
        if names != order {
            v.report(format!(
                "fetch with registers {names:02x?}; fetch_samples_async() writes {order:02x?}"
            ));
            return;
        }
        let val = |i: usize| regs[i].1;
        let m = self.fetch_base;
        self.fetch_base = 1;
        let tail = [val(4), val(5), val(6), val(7), val(8), val(9), val(10), val(11)];
        let want = [0, 0, UNK0 as u16, (UNK0 >> 16) as u16, UNK3 as u16, (UNK3 >> 16) as u16, 2, 3];
        if val(2) != m || val(3) != m | 2 || tail != want {
            v.report(format!(
                "fetch {:04x?}; the driver writes UNK1, FETCH {m}, {}, UNK1 0, UNK2 {UNK0}, UNK3 \
                 {UNK3}, FETCH 2, 3",
                regs.iter().map(|r| r.1).collect::<Vec<_>>(),
                m | 2
            ));
        }
        let Some(c) = &self.fpga.capture else {
            v.report("fetch with no capture");
            return;
        };
        if !c.done {
            v.report("fetch before the capture completed");
            return;
        }
        let groups = u64::from(c.groups);
        let start = c.driver_earliest() % (thousands(c.groups) << 10) * groups / 8;
        let got = u64::from(val(0)) | u64::from(val(1)) << 16;
        if got != start {
            v.report(format!(
                "fetch from {got} (8 byte units); the capture's first sample shown is at {start}"
            ));
        }
        self.fpga.fetch = Some(0);
        self.fpga.fetch_transfers = 0;
        self.fpga.bulk_short = self.faults.bulk_short;
    }

    /// Fifteen writes of the trigger configuration register.
    fn trigger_cfg(&mut self, regs: &[(u8, u16)], v: &mut Violations) {
        if regs.len() != 15 {
            v.report(format!(
                "trigger configuration of {} writes; upload_trigger() sends 15 (five bytes)",
                regs.len()
            ));
        }
        if self.fpga.design.is_none() {
            v.report("trigger configuration with no FPGA design loaded");
            return;
        }
        for t in regs.chunks(3) {
            let ok = t.len() == 3
                && t.iter().all(|r| r.0 == REG_TRIGGER_CFG)
                && t[1].1 == t[0].1 | 0x8000
                && t[2].1 == t[0].1
                && t[0].1 & 0x8000 == 0;
            if !ok {
                v.report(format!(
                    "trigger configuration writes {t:04x?}; the driver writes v, v|0x8000, v"
                ));
                continue;
            }
            let addr = (t[0].1 >> 8) as u8;
            let byte = t[0].1 as u8;
            let Some(stage) =
                STAGE_BASE.iter().position(|&b| addr >= b && addr < b + STAGE_SIZE as u8)
            else {
                v.report(format!("trigger configuration address {addr:#04x}: outside both stages"));
                continue;
            };
            let at = usize::from(addr - STAGE_BASE[stage]);
            self.fpga.stages[stage][at] = byte;
            self.fpga.written[stage] |= 1 << at;
        }
        self.fpga.combine = None;
    }

    /// A single register: a threshold, the channel group or the
    /// combination of the stages.
    fn single(&mut self, (reg, val): (u8, u16), v: &mut Violations) {
        if self.fpga.design.is_none() {
            v.report(format!("write of register {reg:#04x} with no FPGA design loaded"));
            return;
        }
        match reg {
            REG_THRESH_LOWER | REG_THRESH_UPPER | REG_THRESH_EXTERNAL => {
                match THRESHOLDS.iter().position(|t| *t == (reg, val)) {
                    Some(i) => {
                        let before = self.fpga.thresholds;
                        self.fpga.thresholds |= 1 << i;
                        if before != 0b111 && self.fpga.thresholds == 0b111 {
                            // A new design's thresholds: the driver's
                            // bases start again.
                            self.arm_base = 0x00;
                            self.fetch_base = 0;
                        }
                    }
                    None => v.report(format!(
                        "threshold {reg:#04x} = {val:#06x}; the driver writes {THRESHOLDS:02x?}"
                    )),
                }
            }
            REG_SELECT_CHANNELS => {
                if val > 1 || (val == 1 && self.fpga.design == Some(2)) {
                    v.report(format!(
                        "channel group {val}: 0, or 1 (the upper eight) with the 8 channel design"
                    ));
                }
                self.fpga.group = Some(val);
            }
            REG_TRIGGER_COMBINE_OP => self.combine(val, v),
            _ => v.report(format!("write of register {reg:#04x} alone: not one the driver sends")),
        }
    }

    /// The combination ends the trigger configuration: both stages are
    /// checked against `program_trigger()`'s layout.
    fn combine(&mut self, val: u16, v: &mut Violations) {
        self.fpga.combine = Some(val);
        if self.fpga.written != [(1 << STAGE_SIZE) - 1; 2] {
            v.report("trigger combination before both stages were written whole");
            return;
        }
        let mut stages = [Stage::default(); 2];
        for (i, stage) in stages.iter_mut().enumerate() {
            let bytes = self.fpga.stages[i];
            *stage = Stage::decode(&bytes);
            let want = stage.encode(i == 1);
            if bytes != want {
                v.report(format!(
                    "trigger stage {i} {bytes:02x?}: not program_trigger()'s layout, which for \
                     {stage:04x?} is {want:02x?}"
                ));
            }
        }
        match val {
            OP_A => {
                if !stages[1].is_empty() {
                    v.report("TRIGGER_OP_A with conditions in stage B");
                }
            }
            OP_A_AND_B => {
                if stages[0].is_empty() || stages[1].is_empty() {
                    v.report("TRIGGER_OP_A_AND_B with an empty stage");
                }
            }
            _ => v.report(format!(
                "trigger combination {val:#06x}; the driver sends A ({OP_A:#06x}) or A and B \
                 ({OP_A_AND_B:#06x})"
            )),
        }
    }

    /// A chunk of the bitstream on bulk OUT 6.
    fn bitstream(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let Some(bytes) = &mut self.fpga.upload else {
            v.report(format!("{} bytes on bulk OUT 6 with no FPGA upload started", data.len()));
            return Reply::Stall;
        };
        self.fpga.chunks += 1;
        if self.faults.stall_bitstream == Some(self.fpga.chunks) {
            return Reply::Stall;
        }
        if data.len() > CHUNK || data.is_empty() {
            v.report(format!("bitstream chunk of {} bytes; 1 to {CHUNK}", data.len()));
        }
        if self.fpga.short_chunk {
            v.report("bitstream chunk after a short one; only the last is short");
        }
        self.fpga.short_chunk |= data.len() < CHUNK;
        bytes.extend_from_slice(data);
        if bytes.len() < BITSTREAM_16.1 {
            return Reply::ack();
        }
        let bytes = self.fpga.upload.take().unwrap_or_default();
        let groups = if bytes == ezusb::standin(BITSTREAM_8.0, BITSTREAM_8.1) {
            1
        } else if bytes == ezusb::standin(BITSTREAM_16.0, BITSTREAM_16.1) {
            2
        } else {
            v.report(format!(
                "FPGA upload of {} bytes that are neither {} nor {} ({} bytes each)",
                bytes.len(),
                BITSTREAM_8.0,
                BITSTREAM_16.0,
                BITSTREAM_16.1
            ));
            return Reply::ack();
        };
        let verifies = self.fpga.verifies;
        self.fpga.load(groups);
        self.fpga.verifies = verifies;
        self.fpga.uploaded = true;
        Reply::ack()
    }

    /// A status packet, or NAK when there is nothing new to tell.
    fn status_packet(&mut self) -> Reply {
        let short = self.faults.intr_short;
        let Some(c) = &mut self.fpga.capture else { return Reply::Nak };
        let Some(state) = c.states.pop() else { return Reply::Nak };
        let mut p = [0u8; STATUS_SIZE];
        let id = if c.stale > 0 {
            c.stale -= 1;
            c.id.wrapping_sub(1)
        } else {
            c.id
        };
        p[2..4].copy_from_slice(&id.to_be_bytes());
        p[0x1f] = state;
        if state == STATE_DONE {
            p[6..12].copy_from_slice(&c.time_latest().to_be_bytes()[2..]);
            let ns = c.trigger_ns();
            p[0x0c..0x12].copy_from_slice(&(ns >> 3).to_be_bytes()[2..]);
            p[0x13] = (ns & 7) as u8;
        }
        if id == c.id {
            c.done |= state == STATE_DONE;
        } else {
            // A stale packet: the state goes out again with the new id.
            c.states.push(state);
        }
        Reply::Data(short.map_or(p.to_vec(), |n| p[..n].to_vec()))
    }

    fn sample_transfer(&mut self, max: usize, v: &mut Violations) -> Reply {
        let Some(at) = self.fpga.fetch else { return Reply::Nak };
        if !self.fpga.bulk_ready {
            return Reply::Nak;
        }
        // handle_fetch_samples_done() asks for 17 KiB, then
        // recv_bulk_transfer() for what is left, 16 KiB at most; the
        // model sees that rounded up to whole 512 byte packets, as the
        // device does on the bus (docs/USB.md), which matters only when
        // a short transfer left a remainder that is not one.
        let want =
            if self.fpga.fetch_transfers == 0 { 17 << 10 } else { (16 << 10).min(MEMORY - at) };
        if max != want.next_multiple_of(512) {
            v.report(format!(
                "bulk IN transfer {} of the fetch of {max} bytes, with {at} of {MEMORY} sent; the \
                 driver asks for {want} ({} on the bus)",
                self.fpga.fetch_transfers + 1,
                want.next_multiple_of(512)
            ));
        }
        self.fpga.fetch_transfers += 1;
        if self.faults.bulk_stall {
            self.fpga.bulk_ready = false;
            return Reply::Stall;
        }
        if at >= MEMORY {
            // The whole memory went out; nothing more comes.
            return Reply::Nak;
        }
        self.fpga.bulk_ready = false;
        let n = self.fpga.bulk_short.take().unwrap_or(max).min(max).min(MEMORY - at);
        self.fpga.fetch = Some(at + n);
        Reply::Data(self.fpga.memory[at..at + n].to_vec())
    }
}

impl DeviceModel for LogicStudio {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        if self.running() { &self.firmware } else { &self.boot }
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        match self.stage {
            Stage8051::Boot(_) => self.boot_control(setup, data, v),
            Stage8051::Running => self.firmware_control(setup, data, v),
            Stage8051::Gone => Reply::Stall,
        }
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != 6 || !self.running() {
            v.report(format!("OUT transfer to endpoint {ep}; the bitstream goes to 6"));
            return Reply::Stall;
        }
        self.bitstream(data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if !self.running() {
            v.report(format!("IN transfer from endpoint {ep} of the FX2 without firmware"));
            return Reply::Stall;
        }
        match ep {
            1 => {
                if max != STATUS_SIZE {
                    v.report(format!(
                        "interrupt IN of {max} bytes; the status packet has {STATUS_SIZE}"
                    ));
                }
                self.status_packet()
            }
            2 => self.sample_transfer(max, v),
            _ => {
                v.report(format!("IN transfer from endpoint {ep}"));
                Reply::Stall
            }
        }
    }

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if ep == 2 {
            self.fpga.bulk_ready = true;
        }
    }

    fn reset(&mut self) {
        // A new attach ends a capture; the FX2 firmware and the FPGA
        // design stay, as the board keeps power.
        self.fpga.capture = None;
        self.fpga.fetch = None;
        self.fpga.bulk_ready = false;
    }

    /// Firmware and bitstream chunks and sample transfers.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

    fn opts(o: &[(&str, &str)]) -> Vec<(String, String)> {
        o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn board(o: &[(&str, &str)]) -> Device {
        let mut all = vec![("flashed", "1")];
        all.extend_from_slice(o);
        let mut dev = Device::new(build(&opts(&all)).unwrap());
        dev.control(SET_CONFIG, &[], &mut Violations::default());
        dev
    }

    fn out_setup(request: u8, value: u16, index: u16, length: usize) -> Setup {
        Setup { request_type: CTRL_OUT, request, value, index, length: length as u16 }
    }

    fn write(
        dev: &mut Device,
        value: u16,
        index: u16,
        regs: &[(u8, u16)],
        v: &mut Violations,
    ) -> Reply {
        let mut data = Vec::new();
        for (r, val) in regs {
            data.push(*r);
            data.extend_from_slice(&val.to_be_bytes());
        }
        dev.control(out_setup(REQ_REGS, value, index, data.len()), &data, v)
    }

    fn sync(dev: &mut Device, regs: &[(u8, u16)], v: &mut Violations) -> Reply {
        write(dev, 0x12, MAGIC_INDEX, regs, v)
    }

    fn read(dev: &mut Device, reg: u16, v: &mut Violations) -> Reply {
        let s = Setup {
            request_type: CTRL_IN,
            request: REQ_REGS,
            value: reg,
            index: MAGIC_INDEX,
            length: 2,
        };
        dev.control(s, &[], v)
    }

    /// The stop and the thresholds with `wValue` 0, then `0xb3`.
    fn start_upload(dev: &mut Device, base: u16, v: &mut Violations) -> Reply {
        sync(dev, &[(REG_ARM_TRIGGER, base), (REG_ARM_TRIGGER, base | 2)], v);
        for t in THRESHOLDS {
            write(dev, 0, 0, &[t], v);
        }
        dev.control(out_setup(REQ_UPLOAD, 7, MAGIC_INDEX, 0), &[], v)
    }

    fn verify(dev: &mut Device, v: &mut Violations) -> Reply {
        let s = Setup {
            request_type: CTRL_IN,
            request: REQ_VERIFY,
            value: 7,
            index: MAGIC_INDEX,
            length: 1,
        };
        dev.control(s, &[], v)
    }

    /// `lls_setup_acquisition()`'s upload: the stop, the thresholds,
    /// `upload_fpga_bitstream()` and `wait_for_dev_to_settle()`.
    fn upload(dev: &mut Device, groups: u8, v: &mut Violations) {
        let (name, size) = if groups == 1 { BITSTREAM_8 } else { BITSTREAM_16 };
        assert_eq!(start_upload(dev, 0, v), Reply::ack());
        for c in ezusb::standin(name, size).chunks(CHUNK) {
            assert_eq!(dev.transfer_out(6, c, v), Reply::ack());
        }
        let s = Setup {
            request_type: CTRL_IN,
            request: REQ_VERIFY,
            value: 7,
            index: MAGIC_INDEX,
            length: 1,
        };
        assert_eq!(dev.control(s, &[], v), Reply::Data(vec![1]));
        for _ in 0..2 {
            for reg in 0x0f..=0x12 {
                assert_eq!(read(dev, reg, v), Reply::Data(vec![0, 0]));
            }
        }
    }

    /// `lls_setup_acquisition()` for a first commit: `group` is the
    /// channel group register, `stages` the trigger.
    fn commit(dev: &mut Device, groups: u8, group: u16, stages: &[Stage], v: &mut Violations) {
        upload(dev, groups, v);
        for t in THRESHOLDS {
            sync(dev, &[t], v);
        }
        sync(dev, &[(REG_SELECT_CHANNELS, group)], v);
        let op = if stages.len() > 1 { OP_A_AND_B } else { OP_A };
        let a = stages.first().copied().unwrap_or_default();
        let b = stages.get(1).copied().unwrap_or_default();
        trigger(dev, [a.encode(false), b.encode(true)], op, v);
        sync(dev, &[THRESHOLDS[0]], v);
        sync(dev, &[THRESHOLDS[1]], v);
        dev.control(out_setup(REQ_STATUS, 0x12, MAGIC_INDEX, 16), &STATUS_BYTES, v);
    }

    /// One write of the trigger configuration: `addr << 8 | byte` three
    /// times, bit 15 set in the middle one.
    fn cfg(addr: u8, byte: u8) -> [(u8, u16); 3] {
        let val = u16::from(addr) << 8 | u16::from(byte);
        [(REG_TRIGGER_CFG, val), (REG_TRIGGER_CFG, val | 0x8000), (REG_TRIGGER_CFG, val)]
    }

    /// `program_trigger()` and `upload_trigger()`: both stages' bytes,
    /// five a transfer, then the combination.
    fn trigger(dev: &mut Device, stages: [[u8; STAGE_SIZE]; 2], op: u16, v: &mut Violations) {
        for (bytes, base) in stages.iter().zip(STAGE_BASE) {
            for chunk in bytes.chunks(5).enumerate() {
                let regs: Vec<(u8, u16)> = chunk
                    .1
                    .iter()
                    .enumerate()
                    .flat_map(|(j, b)| cfg(base + (chunk.0 * 5 + j) as u8, *b))
                    .collect();
                sync(dev, &regs, v);
            }
        }
        sync(dev, &[(REG_TRIGGER_COMBINE_OP, op)], v);
    }

    /// The violations since the last call: exactly one for each of
    /// `what`, in any order.
    fn expect(v: &mut Violations, what: &[&str]) {
        let got = v.drain();
        assert_eq!(got.len(), what.len(), "{what:?}: {got:?}");
        for w in what {
            assert!(got.iter().any(|g| g.contains(w)), "{w:?} not in {got:?}");
        }
    }

    /// A board with the 16 channel design, its thresholds, group 0 and a
    /// trigger on D0 rising: ready to arm.
    fn ready() -> (Device, Violations) {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        commit(&mut dev, 2, 0, &[rising(0)], &mut v);
        assert!(v.is_empty(), "{v:?}");
        (dev, v)
    }

    /// `lls_start_acquisition()`'s registers.
    fn arm_regs(groups: u8, ratio: u64, cfg: u16, id: u16) -> Vec<(u8, u16)> {
        rearm_regs(groups, ratio, cfg, id, 0)
    }

    /// The arm with the driver's `magic_arm_trigger` `base`.
    fn rearm_regs(groups: u8, ratio: u64, cfg: u16, id: u16, base: u16) -> Vec<(u8, u16)> {
        let total = thousands(groups) * 1000;
        let pre = total * ratio / 100;
        let (p, q) = (transform(pre, groups), transform(total - pre, groups));
        vec![
            (REG_ARM_TRIGGER, base),
            (REG_ARM_TRIGGER, base | 2),
            (REG_UNK6_LO, 0),
            (REG_UNK6_HI, 0),
            (REG_UNK0_LO, UNK0 as u16),
            (REG_UNK0_HI, 0),
            (REG_UNK4_LO, 0),
            (REG_UNK4_HI, 0),
            (REG_UNK5_LO, 0),
            (REG_UNK5_HI, 0),
            (REG_ACQUISITION_ID, id),
            (REG_SAMPLERATE, cfg),
            (REG_PRETRIG_LO, p as u16),
            (REG_PRETRIG_HI, (p >> 16) as u16),
            (REG_POSTTRIG_LO, q as u16),
            (REG_POSTTRIG_HI, (q >> 16) as u16),
            (REG_ARM_TRIGGER, 0x0d),
        ]
    }

    /// What `handle_intr_data()` computes: first sample shown, trigger.
    fn driver_view(p: &[u8], groups: u8, rate: u64, offset: i64) -> (u64, u64) {
        let rb64 = |at: usize| u64::from_be_bytes(p[at..at + 8].try_into().unwrap());
        let div = 1_000_000_000 / rate;
        let latest = (rb64(6) >> 16) * 8 / div;
        let pdiv = 8 / u64::from(groups);
        let aligned = if latest > pdiv { latest - pdiv } else { latest } & !(pdiv - 1);
        let earliest = aligned - thousands(groups) * 1000;
        let mut t = (rb64(0xc) >> 16) * 8;
        t |= u64::from(u16::from_be_bytes([p[0x12], p[0x13]]) & 7);
        t = t.wrapping_add(offset as u64);
        if groups > 1 {
            t -= 8;
        }
        t -= 0x18;
        if div > 1 {
            t -= div;
            t /= div;
        }
        (earliest, t + 1)
    }

    fn fetch_regs(start: u64) -> Vec<(u8, u16)> {
        refetch_regs(start, 0)
    }

    /// The fetch with the driver's `magic_fetch_samples` `base`.
    fn refetch_regs(start: u64, base: u16) -> Vec<(u8, u16)> {
        vec![
            (REG_UNK1_LO, start as u16),
            (REG_UNK1_HI, (start >> 16) as u16),
            (REG_FETCH_SAMPLES, base),
            (REG_FETCH_SAMPLES, base | 2),
            (REG_UNK1_LO, 0),
            (REG_UNK1_HI, 0),
            (REG_UNK2_LO, UNK0 as u16),
            (REG_UNK2_HI, 0),
            (REG_UNK3_LO, UNK3 as u16),
            (REG_UNK3_HI, 0),
            (REG_FETCH_SAMPLES, 2),
            (REG_FETCH_SAMPLES, 3),
        ]
    }

    /// A whole capture as the driver runs it; the memory and the driver's
    /// view of the trigger and first sample.
    fn capture(
        dev: &mut Device,
        groups: u8,
        group: u16,
        stages: &[Stage],
        ratio: u64,
        rate_i: usize,
        v: &mut Violations,
    ) -> (Vec<u8>, u64, u64) {
        commit(dev, groups, group, stages, v);
        let (rate, offset, cfg) = RATES[rate_i];
        assert_eq!(dev.transfer_in(1, 32, v), Reply::Nak, "nothing before the arm");
        sync(dev, &arm_regs(groups, ratio, cfg, 1), v);
        let mut seen = Vec::new();
        let packet = loop {
            let Reply::Data(p) = dev.transfer_in(1, 32, v) else { panic!("no status") };
            assert_eq!(&p[2..4], &[0, 1]);
            seen.push(p[0x1f]);
            if p[0x1f] == STATE_DONE {
                break p;
            }
        };
        assert_eq!(seen, [STATE_PRE, STATE_ARMED, STATE_POST, STATE_DONE]);
        let (earliest, trigger) = driver_view(&packet, groups, rate, offset);
        let start = earliest % (thousands(groups) << 10) * u64::from(groups) / 8;
        sync(dev, &fetch_regs(start), v);
        let mut memory = Vec::new();
        for len in [17 << 10, 16 << 10, 7 << 10] {
            dev.in_submitted(2, len);
            let Reply::Data(d) = dev.transfer_in(2, len, v) else { panic!("no samples") };
            assert_eq!(d.len(), len);
            memory.extend(d);
        }
        sync(dev, &[(REG_ARM_TRIGGER, 0x0c), (REG_ARM_TRIGGER, 0x0e)], v);
        (memory, earliest, trigger)
    }

    fn rising(c: u8) -> Stage {
        Stage { rising: 1 << c, ..Stage::default() }
    }

    #[test]
    fn a_16_channel_capture_shows_the_trigger_at_its_ratio() {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        let (memory, earliest, trigger) = capture(&mut dev, 2, 0, &[rising(3)], 50, 1, &mut v);
        assert!(v.is_empty(), "{v:?}");
        // D3 rises at 8 mod 16, first after the pre-trigger samples.
        assert_eq!(trigger % 16, 8);
        let pre = trigger - earliest;
        assert!((9_990..=10_010).contains(&pre), "{pre} samples before the trigger");
        // The memory holds sample s at s mod 20480, low byte first.
        for s in [earliest, trigger, earliest + 19_999] {
            let at = (s % 20_480) as usize * 2;
            assert_eq!(u16::from_le_bytes([memory[at], memory[at + 1]]), s as u16);
        }
    }

    #[test]
    fn an_8_channel_capture_of_the_upper_group_wraps_the_memory() {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        // D14 rises at 16384 + 32768 n; after the 20040 pre-trigger
        // samples at 50 % the first is 49152, and the samples shown
        // cross the end of the 40960 sample memory.
        let (memory, earliest, trigger) = capture(&mut dev, 1, 1, &[rising(14)], 50, 9, &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(trigger, 49_152);
        assert!(earliest % 40_960 > trigger % 40_960, "the shown samples wrap");
        let at = (trigger % 40_960) as usize;
        assert_eq!(memory[at], (trigger >> 8) as u8);
    }

    #[test]
    fn two_stages_and_every_rate_give_the_trigger_back() {
        for (i, rate) in RATES.iter().enumerate() {
            let mut dev = board(&[]);
            let mut v = Violations::default();
            // D5 high and D2 rising: s = 4 mod 8 with bit 5 set.
            let a = Stage { ones: 1 << 5, ..Stage::default() };
            let b = rising(2);
            let (_, earliest, trigger) = capture(&mut dev, 2, 0, &[a, b], 25, i, &mut v);
            assert!(v.is_empty(), "{rate:?}: {v:?}");
            assert!(trigger % 8 == 4 && trigger & 32 != 0, "{rate:?}: {trigger}");
            let pre = trigger - earliest;
            assert!((4_990..=5_010).contains(&pre), "{rate:?}: {pre} samples before the trigger");
        }
    }

    #[test]
    fn stage_layout_round_trips() {
        for st in [
            Stage { rising: 0x8001, falling: 0x0100, any: 0x0010, ones: 0x8002, zeroes: 0x0004 },
            Stage { ones: 1, ..Stage::default() },
            Stage::default(),
        ] {
            for last in [false, true] {
                assert_eq!(Stage::decode(&st.encode(last)), st);
            }
        }
        let st = Stage { rising: 1, falling: 2, any: 4, ones: 8, zeroes: 16 };
        assert!(st.holds(0b0_0010, 0b0_1001), "D0 rises with D3 high and D4 low");
        assert!(!st.holds(0b0_0001, 0b0_1001), "no edge");
        assert!(!st.holds(0b0_0010, 0b1_1001), "D4 high");
        assert!(st.holds(0b0_1100, 0b0_1000), "D2 falls (either edge)");
    }

    #[test]
    fn the_trigger_that_never_fires_leaves_the_capture_waiting() {
        let mut dev = board(&[("still", "1")]);
        let mut v = Violations::default();
        commit(&mut dev, 2, 0, &[rising(0)], &mut v);
        sync(&mut dev, &arm_regs(2, 50, 0x00, 1), &mut v);
        let states: Vec<u8> = (0..2)
            .map(|_| match dev.transfer_in(1, 32, &mut v) {
                Reply::Data(p) => p[0x1f],
                r => panic!("{r:?}"),
            })
            .collect();
        assert_eq!(states, [STATE_PRE, STATE_ARMED]);
        assert_eq!(dev.transfer_in(1, 32, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_driver_never_sends() {
        let mut v = Violations::default();
        let mut dev = board(&[]);
        // Registers to a blank FPGA, other than the stop.
        sync(&mut dev, &[(REG_SELECT_CHANNELS, 0)], &mut v);
        assert_eq!(v.drain().len(), 1, "a write to a blank FPGA");
        write(&mut dev, 0, 0, &[(REG_THRESH_LOWER, 0x00c4)], &mut v);
        assert_eq!(v.drain().len(), 1, "another threshold value");
        read(&mut dev, 0x0f, &mut v);
        assert_eq!(v.drain().len(), 1, "a settle read with no design");
        // A bitstream that is not a stand-in.
        start_upload(&mut dev, 0, &mut v);
        for c in vec![0u8; BITSTREAM_16.1].chunks(CHUNK) {
            dev.transfer_out(6, c, &mut v);
        }
        assert_eq!(v.drain().len(), 1, "not a stand-in");
        let s = Setup {
            request_type: CTRL_IN,
            request: REQ_VERIFY,
            value: 7,
            index: MAGIC_INDEX,
            length: 1,
        };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Data(vec![0]));
        // Chunks of another size.
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0), (REG_ARM_TRIGGER, 2)], &mut v);
        for t in THRESHOLDS {
            write(&mut dev, 0, 0, &[t], &mut v);
        }
        dev.control(out_setup(REQ_UPLOAD, 7, 0, 0), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "wIndex");
        dev.transfer_out(6, &[0; 1000], &mut v);
        dev.transfer_out(6, &[0; 1000], &mut v);
        assert_eq!(v.drain().len(), 1, "a chunk after a short one");
        dev.transfer_out(6, &[0; 4096], &mut v);
        assert_eq!(v.drain().len(), 2, "too long, after a short one");
        // A good design, then the arm before its configuration.
        let mut dev = board(&[]);
        upload(&mut dev, 2, &mut v);
        assert!(v.is_empty(), "{v:?}");
        read(&mut dev, 0x11, &mut v);
        assert_eq!(v.drain().len(), 1, "out of order");
        sync(&mut dev, &arm_regs(2, 50, 0, 1), &mut v);
        assert_eq!(v.drain().len(), 4, "thresholds, status, group, trigger");
        sync(&mut dev, &[(REG_SELECT_CHANNELS, 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "the upper group with 16 channels");
        sync(&mut dev, &[(REG_TRIGGER_COMBINE_OP, OP_A)], &mut v);
        assert_eq!(v.drain().len(), 1, "the combination before the stages");
        sync(
            &mut dev,
            &[(REG_TRIGGER_CFG, 0x0012), (REG_TRIGGER_CFG, 0x0012), (REG_TRIGGER_CFG, 0x0012)],
            &mut v,
        );
        assert_eq!(v.drain().len(), 2, "no 0x8000 in the middle; not 15 writes");
        dev.control(out_setup(REQ_STATUS, 0x12, MAGIC_INDEX, 16), &[0; 16], &mut v);
        assert_eq!(v.drain().len(), 2, "status bytes, before the thresholds");
        sync(&mut dev, &[(REG_ARM_TRIGGER, 1), (REG_ARM_TRIGGER, 3)], &mut v);
        assert_eq!(v.drain().len(), 1, "a stop with another base");
        sync(&mut dev, &[(0x30, 1), (0x31, 2)], &mut v);
        assert_eq!(v.drain().len(), 1, "an unknown transfer");
        sync(&mut dev, &fetch_regs(0), &mut v);
        assert_eq!(v.drain().len(), 1, "a fetch with no capture");
        dev.control(
            Setup { request_type: CTRL_OUT, request: 0xb5, value: 0, index: 0, length: 0 },
            &[],
            &mut v,
        );
        assert_eq!(v.drain().len(), 1, "an unknown request");
        dev.control(
            Setup { request_type: CTRL_OUT, request: 0xa0, value: 0, index: 0, length: 0 },
            &[],
            &mut v,
        );
        assert_eq!(v.drain().len(), 1, "a firmware load to the running firmware");
        // Arms with counts no ratio gives and a rate off the table.
        let mut dev = board(&[]);
        commit(&mut dev, 2, 0, &[rising(0)], &mut v);
        assert!(v.is_empty(), "{v:?}");
        let mut regs = arm_regs(2, 50, 0, 1);
        regs[12].1 += 1;
        sync(&mut dev, &regs, &mut v);
        assert_eq!(v.drain().len(), 1, "counts");
        let mut regs = rearm_regs(2, 50, 0x02, 1, 0x0c);
        regs[1].1 = 0;
        sync(&mut dev, &regs, &mut v);
        assert_eq!(v.drain().len(), 2, "the pulse; cfg 2 is no rate");
        sync(&mut dev, &rearm_regs(2, 50, 0x00, 1, 0x0c), &mut v);
        assert!(v.is_empty(), "{v:?}");
        sync(&mut dev, &fetch_regs(0), &mut v);
        assert_eq!(v.drain().len(), 1, "a fetch before the capture completed");
        assert_eq!(
            dev.transfer_in(1, 16, &mut v),
            Reply::Data({
                let mut p = vec![0u8; 32];
                p[3] = 1;
                p[0x1f] = STATE_PRE;
                p
            })
        );
        assert_eq!(v.drain().len(), 1, "an interrupt IN of 16 bytes");
    }

    #[test]
    fn the_flashless_board_takes_the_fx2_firmware_and_renumerates() {
        let mut dev = Device::new(build(&[]).unwrap());
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().product_id(), PID_BARE);
        let load = |value: u16, length: usize| Setup {
            request_type: 0x40,
            request: 0xa0,
            value,
            index: 0,
            length: length as u16,
        };
        dev.control(load(ezusb::CPUCS, 1), &[1], &mut v);
        let image = ezusb::standin(FX2_FIRMWARE.0, FX2_FIRMWARE.1);
        for (i, c) in image.chunks(4096).enumerate() {
            dev.control(load((i * 4096) as u16, c.len()), c, &mut v);
        }
        dev.control(load(ezusb::CPUCS, 1), &[0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        assert_eq!(dev.model_descriptors().product_id(), PID_FIRMWARE);
        // A firmware that never comes back.
        let mut dev = Device::new(build(&opts(&[("renum", "0")])).unwrap());
        dev.control(load(ezusb::CPUCS, 1), &[1], &mut v);
        for (i, c) in image.chunks(4096).enumerate() {
            dev.control(load((i * 4096) as u16, c.len()), c, &mut v);
        }
        dev.control(load(ezusb::CPUCS, 1), &[0], &mut v);
        assert!(!dev.take_renumeration());
        assert_eq!(dev.model_descriptors().product_id(), PID_BARE);
    }

    #[test]
    fn faults_fail_as_asked() {
        let mut v = Violations::default();
        let mut dev = board(&[("stall", "write:2,verify")]);
        assert_eq!(
            sync(&mut dev, &[(REG_ARM_TRIGGER, 0), (REG_ARM_TRIGGER, 2)], &mut v),
            Reply::ack()
        );
        assert_eq!(write(&mut dev, 0, 0, &[THRESHOLDS[0]], &mut v), Reply::Stall);
        let s = Setup {
            request_type: CTRL_IN,
            request: REQ_VERIFY,
            value: 7,
            index: MAGIC_INDEX,
            length: 1,
        };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Stall);
        let mut dev = board(&[("verify", "3"), ("settle", "2")]);
        start_upload(&mut dev, 0, &mut v);
        for c in ezusb::standin(BITSTREAM_8.0, BITSTREAM_8.1).chunks(CHUNK) {
            dev.transfer_out(6, c, &mut v);
        }
        let answers: Vec<Reply> = (0..4).map(|_| dev.control(s, &[], &mut v)).collect();
        assert_eq!(answers[2], Reply::Data(vec![0]));
        assert_eq!(answers[3], Reply::Data(vec![1]));
        let groups: Vec<Vec<Reply>> =
            (0..4).map(|_| (0x0f..=0x12).map(|r| read(&mut dev, r, &mut v)).collect()).collect();
        assert_ne!(groups[0], groups[1]);
        assert_ne!(groups[1], groups[2]);
        assert_eq!(groups[2], groups[3]);
        assert!(v.is_empty(), "{v:?}");
        let mut dev = board(&[("stall_bitstream", "3")]);
        start_upload(&mut dev, 0, &mut v);
        let r: Vec<Reply> = (0..3).map(|_| dev.transfer_out(6, &[0; CHUNK], &mut v)).collect();
        assert_eq!(r[2], Reply::Stall);
        assert!(build(&opts(&[("stall", "nothing")])).is_err());
        assert!(build(&opts(&[("intr_short", "32")])).is_err());
        assert!(build(&opts(&[("colour", "1")])).is_err());
    }

    #[test]
    fn a_short_sample_transfer_is_followed_by_the_rest() {
        let mut dev = board(&[("bulk_short", "1000")]);
        let mut v = Violations::default();
        commit(&mut dev, 2, 0, &[rising(3)], &mut v);
        let (rate, offset, cfg) = RATES[9];
        sync(&mut dev, &arm_regs(2, 50, cfg, 1), &mut v);
        let packet = loop {
            let Reply::Data(p) = dev.transfer_in(1, 32, &mut v) else { panic!("no status") };
            if p[0x1f] == STATE_DONE {
                break p;
            }
        };
        let (earliest, _) = driver_view(&packet, 2, rate, offset);
        sync(&mut dev, &fetch_regs(earliest % 20_480 * 2 / 8), &mut v);
        let mut got = 0;
        // The driver's last transfer is 7192 bytes, which the model
        // sees as 15 packets.
        for (ask, len) in
            [(17 << 10, 1000), (16 << 10, 16 << 10), (16 << 10, 16 << 10), (7680, 7192)]
        {
            dev.in_submitted(2, ask);
            let Reply::Data(d) = dev.transfer_in(2, ask, &mut v) else { panic!("no samples") };
            assert_eq!(d.len(), len);
            got += len;
        }
        assert_eq!(got, MEMORY);
        assert!(v.is_empty(), "{v:?}");
        sync(&mut dev, &refetch_regs(earliest % 20_480 * 2 / 8, 1), &mut v);
        dev.in_submitted(2, 16 << 10);
        dev.transfer_in(2, 16 << 10, &mut v);
        assert_eq!(v.drain().len(), 1, "the first transfer of a fetch is 17 KiB");
    }

    #[test]
    fn the_upload_comes_after_the_stop_and_the_thresholds() {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        dev.control(out_setup(REQ_UPLOAD, 7, MAGIC_INDEX, 0), &[], &mut v);
        expect(&mut v, &["not right after the stop"]);
        // A threshold missing.
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0), (REG_ARM_TRIGGER, 2)], &mut v);
        write(&mut dev, 0, 0, &[THRESHOLDS[0]], &mut v);
        write(&mut dev, 0, 0, &[THRESHOLDS[1]], &mut v);
        dev.control(out_setup(REQ_UPLOAD, 7, MAGIC_INDEX, 0), &[], &mut v);
        expect(&mut v, &["not right after the stop"]);
        // Out of order, and with no stop before them.
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0), (REG_ARM_TRIGGER, 2)], &mut v);
        write(&mut dev, 0, 0, &[THRESHOLDS[1]], &mut v);
        expect(&mut v, &["out of place"]);
        write(&mut dev, 0, 0, &[THRESHOLDS[0]], &mut v);
        expect(&mut v, &["out of place"]);
        // After a configuration, with no upload to follow.
        let (mut dev, mut v) = ready();
        write(&mut dev, 0, 0, &[THRESHOLDS[0]], &mut v);
        expect(&mut v, &["out of place"]);
        // During a capture.
        sync(&mut dev, &arm_regs(2, 50, 0, 1), &mut v);
        dev.control(out_setup(REQ_UPLOAD, 7, MAGIC_INDEX, 0), &[], &mut v);
        expect(&mut v, &["during a capture", "not right after the stop"]);
    }

    #[test]
    fn the_arm_and_fetch_bases_follow_the_driver() {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        // A stop with 0x0c before any arm.
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0x0c), (REG_ARM_TRIGGER, 0x0e)], &mut v);
        expect(&mut v, &["stop"]);
        let (mut dev, mut v) = ready();
        sync(&mut dev, &rearm_regs(2, 50, 0, 1, 0x0c), &mut v);
        expect(&mut v, &["arm pulse"]);
        // From now on 0x0c; a stop may also come from a new process's 0.
        sync(&mut dev, &arm_regs(2, 50, 0, 2), &mut v);
        expect(&mut v, &["arm pulse"]);
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0), (REG_ARM_TRIGGER, 2)], &mut v);
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0x0c), (REG_ARM_TRIGGER, 0x0e)], &mut v);
        sync(&mut dev, &rearm_regs(2, 50, 0, 3, 0x0c), &mut v);
        assert!(v.is_empty(), "{v:?}");
        // A capture and its fetch with 0, then another capture, whose
        // fetch takes 1.
        let mut dev = board(&[]);
        capture(&mut dev, 2, 0, &[rising(3)], 50, 1, &mut v);
        let (rate, offset, cfg) = RATES[1];
        sync(&mut dev, &rearm_regs(2, 50, cfg, 2, 0x0c), &mut v);
        let packet = loop {
            let Reply::Data(p) = dev.transfer_in(1, 32, &mut v) else { panic!("no status") };
            if p[0x1f] == STATE_DONE {
                break p;
            }
        };
        assert!(v.is_empty(), "{v:?}");
        let (earliest, _) = driver_view(&packet, 2, rate, offset);
        let start = earliest % 20_480 * 2 / 8;
        sync(&mut dev, &refetch_regs(start, 0), &mut v);
        expect(&mut v, &["fetch ["]);
        sync(&mut dev, &refetch_regs(start, 1), &mut v);
        assert!(v.is_empty(), "{v:?}");
        // Another first sample than the one the times reported.
        sync(&mut dev, &refetch_regs(start + 1, 1), &mut v);
        expect(&mut v, &["fetch from"]);
        // Values off the driver's.
        let mut regs = refetch_regs(start, 1);
        regs[6].1 += 1;
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["fetch ["]);
        // Another order.
        let mut regs = refetch_regs(start, 1);
        regs.swap(6, 8);
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["fetch with registers"]);
    }

    #[test]
    fn a_stalled_arm_still_moves_the_drivers_base() {
        // The commit is 29 writes; the arm, the 30th, stalls.
        let mut dev = board(&[("stall", "write:30")]);
        let mut v = Violations::default();
        commit(&mut dev, 2, 0, &[rising(0)], &mut v);
        assert_eq!(sync(&mut dev, &arm_regs(2, 50, 0, 1), &mut v), Reply::Stall);
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0x0c), (REG_ARM_TRIGGER, 0x0e)], &mut v);
        assert!(v.is_empty(), "{v:?}");
        // A failed upload leaves the base as it was.
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0x0c), (REG_ARM_TRIGGER, 0x0e)], &mut v);
        for t in THRESHOLDS {
            write(&mut dev, 0, 0, &[t], &mut v);
        }
        dev.control(out_setup(REQ_UPLOAD, 7, MAGIC_INDEX, 0), &[], &mut v);
        sync(&mut dev, &[(REG_ARM_TRIGGER, 0x0c), (REG_ARM_TRIGGER, 0x0e)], &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_arm_is_the_drivers_17_registers() {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        sync(&mut dev, &arm_regs(2, 50, 0, 1), &mut v);
        expect(&mut v, &["arm with no FPGA design"]);
        let (mut dev, mut v) = ready();
        let mut regs = arm_regs(2, 50, 0, 1);
        regs.swap(12, 14);
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["arm with registers"]);
        let mut regs = rearm_regs(2, 50, 0, 1, 0x0c);
        regs[4].1 += 1;
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["arm UNK6, UNK0"]);
        let mut regs = rearm_regs(2, 50, 0, 1, 0x0c);
        regs[2].1 = 1;
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["arm UNK6, UNK0"]);
    }

    #[test]
    fn the_trigger_stages_follow_program_trigger() {
        let a = rising(0).encode(false);
        let b = rising(1).encode(true);
        let empty = Stage::default().encode(true);
        // A byte the layout never writes.
        let (mut dev, mut v) = ready();
        let mut odd = a;
        odd[STAGE_SIZE - 1] = 0xab;
        trigger(&mut dev, [odd, empty], OP_A, &mut v);
        expect(&mut v, &["not program_trigger()'s layout"]);
        trigger(&mut dev, [a, b], OP_A, &mut v);
        expect(&mut v, &["TRIGGER_OP_A with conditions in stage B"]);
        trigger(&mut dev, [a, empty], OP_A_AND_B, &mut v);
        expect(&mut v, &["TRIGGER_OP_A_AND_B with an empty stage"]);
        trigger(&mut dev, [a, empty], 0x2000, &mut v);
        expect(&mut v, &["trigger combination 0x2000"]);
        trigger(&mut dev, [a, b], OP_A_AND_B, &mut v);
        assert!(v.is_empty(), "{v:?}");
        // An address between the stages.
        let regs: Vec<(u8, u16)> = (0..5).flat_map(|i| cfg(0x30 + i, 0)).collect();
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["outside both stages"; 5]);
        // With no design.
        let mut dev = board(&[]);
        let regs: Vec<(u8, u16)> = (0..5).flat_map(|i| cfg(i, 0)).collect();
        sync(&mut dev, &regs, &mut v);
        expect(&mut v, &["trigger configuration with no FPGA design"]);
    }

    #[test]
    fn single_registers_are_the_drivers() {
        let (mut dev, mut v) = ready();
        sync(&mut dev, &[(REG_THRESH_LOWER, 0x00c4)], &mut v);
        expect(&mut v, &["threshold 0x23"]);
        sync(&mut dev, &[(0x30, 1)], &mut v);
        expect(&mut v, &["write of register 0x30 alone"]);
        sync(&mut dev, &[(REG_SELECT_CHANNELS, 2)], &mut v);
        expect(&mut v, &["channel group 2"]);
    }

    #[test]
    fn register_writes_are_whole_triplets() {
        let (mut dev, mut v) = ready();
        for data in [&[0x21u8, 0][..], &[0x21, 0, 0, 0x22]] {
            dev.control(out_setup(REQ_REGS, 0x12, MAGIC_INDEX, data.len()), data, &mut v);
            expect(&mut v, &["register write of"]);
        }
        let data = vec![0x21u8; 33 * 3];
        dev.control(out_setup(REQ_REGS, 0x12, MAGIC_INDEX, data.len()), &data, &mut v);
        expect(&mut v, &["register write of 99 bytes"]);
    }

    #[test]
    fn the_status_register_and_the_verify_have_their_place() {
        let mut dev = board(&[]);
        let mut v = Violations::default();
        dev.control(out_setup(REQ_STATUS, 0x12, MAGIC_INDEX, 16), &STATUS_BYTES, &mut v);
        expect(&mut v, &["with no FPGA design", "before the thresholds"]);
        assert_eq!(verify(&mut dev, &mut v), Reply::Data(vec![0]));
        expect(&mut v, &["verified with no upload"]);
        let (mut dev, mut v) = ready();
        dev.control(out_setup(REQ_STATUS, 0x12, MAGIC_INDEX, 15), &STATUS_BYTES[..15], &mut v);
        expect(&mut v, &["of 15 bytes; it has 16", "status register ["]);
        let s = Setup { length: 2, ..out_setup(REQ_UPLOAD, 7, MAGIC_INDEX, 0) };
        let s = Setup { request_type: CTRL_IN, request: REQ_VERIFY, ..s };
        dev.control(s, &[], &mut v);
        expect(&mut v, &["of 2 bytes; it has 1"]);
    }

    #[test]
    fn transfers_go_to_the_drivers_endpoints() {
        let mut v = Violations::default();
        let mut dev = board(&[]);
        dev.transfer_out(6, &[0; CHUNK], &mut v);
        expect(&mut v, &["with no FPGA upload started"]);
        // The model alone: the device stops transfers to endpoints the
        // descriptors lack before the model sees them.
        let mut model = build(&opts(&[("flashed", "1")])).unwrap();
        assert_eq!(model.out(2, &[0; 4], &mut v), Reply::Stall);
        expect(&mut v, &["OUT transfer to endpoint 2"]);
        assert_eq!(model.in_transfer(3, 512, &mut v), Reply::Stall);
        expect(&mut v, &["IN transfer from endpoint 3"]);
        let mut bare = build(&[]).unwrap();
        assert_eq!(bare.in_transfer(1, 32, &mut v), Reply::Stall);
        expect(&mut v, &["of the FX2 without firmware"]);
        let s = Setup { request_type: CTRL_OUT, request: REQ_REGS, value: 0, index: 0, length: 0 };
        assert_eq!(bare.control(s, &[], &mut v), Reply::Stall);
        expect(&mut v, &["only its boot loader"]);
    }

    #[test]
    fn standin_sizes_match_the_firmware_list() {
        let list = include_str!("../../../tools/emu/lecroy-logicstudio.firmware");
        let mut want: Vec<(&str, usize)> = vec![FX2_FIRMWARE, BITSTREAM_8, BITSTREAM_16];
        let mut got: Vec<(&str, usize)> = list
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .map(|l| {
                let (n, s) = l.split_once(' ').unwrap();
                (n, s.trim().parse().unwrap())
            })
            .collect();
        want.sort_unstable();
        got.sort_unstable();
        assert_eq!(got, want);
    }
}
