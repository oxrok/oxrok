// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Sysclk LWLA1034 (`2961:6689`) and LWLA1016 (`2961:6688`) as
//! libsigrok's `sysclk-lwla` driver sees them: a Cypress FX2 with its
//! firmware in EEPROM that relays 16-bit word commands to an FPGA, which
//! takes its bitstream from the host on every open.
//!
//! Every fact below is taken from the C driver (`api.c`, `protocol.c`,
//! `lwla.c`, `lwla1034.c`, `lwla1016.c` and their headers); no firmware
//! source, datasheet or recording of the devices was at hand. What the
//! driver does not pin down is marked "assumed" (the model's choice) or
//! "invented" (made-up content). How sure: the ids, the endpoints, the
//! command words, the register addresses, the bitstream names and the
//! sample formats are the driver's and certain; that the board is an FX2
//! with an FPGA is the driver's comments' reading (`lwla1034.c`, "the
//! buffer size on the FX2 USB chip"); the encoder side of the formats,
//! the status bits' timing and everything about the signal are assumed.
//!
//! **USB**: `idVendor` 0x2961, `idProduct` 0x6689 (LWLA1034) or 0x6688
//! (LWLA1016, `model=1016`), configuration 1 with interface 0
//! (`USB_CONFIG`, `USB_INTERFACE`) and bulk endpoints 2 OUT (commands),
//! 4 OUT (the bitstream) and 6 IN (replies), high speed with 512 byte
//! packets ("The configured endpoint buffer size is 512",
//! `lwla1034.c`). No strings, `bcdDevice` 0, device class 0 with a
//! vendor specific interface, 100 mA (assumed; the driver reads none of
//! it).
//!
//! **The bitstream**: one bulk OUT on endpoint 4 of a 4 byte big endian
//! length (the whole transfer's, header included) and the `.rbf` file
//! (`lwla_send_bitstream()`). The LWLA1034 has four
//! (`sysclk-lwla1034-off.rbf`, `-int.rbf`, `-extpos.rbf`,
//! `-extneg.rbf`: shut down, internal clock, external clock on its
//! rising or falling edge), the LWLA1016 two (`sysclk-lwla1016-100.rbf`
//! and `-100-ts.rbf`, plain and "timing-state", i.e. run-length coded,
//! sampling). The stand-ins are `tools/emu-firmware.sh`'s
//! ([`BITSTREAMS`] gives the sizes, `tools/emu/sysclk-lwla.firmware`
//! the same); the model checks the upload byte for byte and takes the
//! design it names. Until a bitstream the FPGA is blank and after the
//! shut-down one it is off: a command then is flagged (and a read
//! answered with zeros).
//!
//! **Commands** are bulk OUTs on endpoint 2 of little endian 16-bit
//! words, the first the command; a 32-bit value goes as two words, the
//! high half first, a 64-bit one as its low 32 bits then its high 32
//! bits (`LWLA_WORD_0` to `LWLA_WORD_3`, `LWLA_TO_UINT32`). Replies come
//! on endpoint 6 in the same order of words.
//!
//! | command | words after the id | reply |
//! |---|---|---|
//! | 1 read register | address | the 32-bit value |
//! | 2 write register | address, value | none |
//! | 3 read memory, 32 bit (LWLA1016) | address, count (32 bit each) | `count` words |
//! | 6 read memory, 36 bit (LWLA1034) | address, count (a multiple of 8) | slices of 9 words: 8 low halves, then the 8 high nibbles, the first in bits 28-31 |
//! | 7 write long registers (LWLA1034) | first, count, 64 bits each | none |
//! | 8 read long registers (LWLA1034) | first, count | 64 bits each |
//!
//! A reply is limited to 1024 bytes, the FX2's two 512 byte buffers
//! ("reading chunks larger than about 1024 bytes is unreliable",
//! `lwla1034.c`; `MAX_ACQ_RECV_LEN32`, `lwla.h`): a longer read is
//! flagged. `short_transfers=1` is the FX2 firmware's bug both C files
//! describe: the reply endpoint then sends at most 64 bytes per
//! transfer, so a longer reply comes as several transfers.
//!
//! **The LWLA1034** (`lwla1034.c`): registers 0x1074 memory control
//! (bit 0 `wr1rd0`, bit 1 `clr_idx`), 0x1078 memory fill level (read),
//! 0x107c memory read start, 0x1094 clock boost (0 or 1), and 0x10b0 to
//! 0x10bc the strobe, address, low and high words of the 64-bit long
//! registers: a strobe read latches the addressed long register into
//! low and high, a strobe write stores them into it. Long registers: 0
//! channel mask (34 bits), 1 clock divider (`100 MHz / (n + 1)`), 2
//! trigger levels, 3 trigger types (edge where set), 4 trigger enable
//! (bits 34 and 35 the external TRG input's falling and rising edge),
//! 5 the memory fill level (written: the full threshold), 7 the
//! elapsed time in ms (from the trigger: see "The clock's start" below),
//! 8 the input levels, 9 the status, 10
//! capture control (0x74 clears, 1 arms the trigger, 0 stops), 100 the
//! test word `0x1234567887654321`. Long registers 0 to 4 read 0
//! (assumed: "The first five long registers do not return useful
//! values when read"), 6 reads 0, and so does 8 (an emulation artifact:
//! the driver ignores the levels, and a counter there would keep the
//! transactions of a capture waiting for its trigger from repeating, so
//! a `--hangs` case could not cut them). Status bits (before the driver's
//! shift by one): 1 capturing, 4 triggered, 5 memory available. With
//! the clock boost on during a capture the FPGA runs at 125 MHz and its
//! millisecond counter counts 0.8 ms (the comment of
//! `handle_response()`): the model reports the time times 5/4. The
//! sample memory holds 36-bit words from address 4 (`READ_START_ADDR`):
//! a sample of 34 bits, bit 34 one more sample of it, bit 35 "length
//! follows", and then a word whose double adds to the run
//! (`read_response()`); the model's encoder (assumed) writes a run of
//! one or two samples as one word and a longer one as two. The fill
//! register (0x1078) reads the address after the last word.
//!
//! **The LWLA1016** (`lwla1016.c`): registers 0x1000 channel mask,
//! 0x1010 capture duration in ms (written 0 to clear; from the trigger:
//! see "The clock's start" below), 0x1070
//! write pointer, 0x107c memory control (bit 0 reset, bit 1 write),
//! 0x10b0 capture count (written: the full threshold in words; read:
//! the address of the last word written, data starting at 2, so the
//! driver reads addresses 2 to it), 0x10b4 test word `0x12345678` when
//! read and trigger setup (types in the high half, levels in the low)
//! when written, 0x10b8 capture control (bit 2 arms the trigger, the
//! trigger enable mask in the high half; read: bit 2 capturing, bit 5
//! triggered, bit 6 memory available), 0x10bc the clock divider. The
//! sample memory holds 32-bit words: two 16-bit samples, the first in
//! the low half, or with the "-ts" design one sample in the low half
//! and its run length less one in the high half.
//!
//! The first read of the test word after a bitstream reads 0 (assumed,
//! from "Ignore the value returned by the first read", both files).
//! Memory reads 0 where nothing was written (assumed).
//!
//! **The signal** (invented): input `i` at sample `s` is bit `i` of
//! `s / period` (`period=N`, 1 by default), a binary counter; the
//! external TRG input of the LWLA1034 is low for `ext_trg=N` samples
//! (1000 by default), then high for as many, and so on. Channels the
//! mask disables are stored as 0 (assumed). A trigger is the AND of
//! its conditions at one sample: a level is met while the input has
//! it, an edge where the input changes to its level from the sample
//! before (none at sample 0), the TRG input's edge likewise; without
//! conditions it fires at once. The memory holds the samples from the
//! trigger on (the driver configures no pre-trigger).
//!
//! **Pacing**: a real board samples in wall time and the driver polls
//! its status every 100 ms of idle USB (`lwla_start_acquisition()`).
//! The model counts its time in polls instead: each status read of a
//! running capture (long register 9 of the LWLA1034, register 0x10b8 of
//! the LWLA1016) moves the board's clock on by `tick=MS` ms (100 by
//! default), at 100 MHz divided as set, 125 MHz with the boost, or
//! `ext_clock=HZ` (1 MHz by default) on the external clock. The memory
//! is full when its words would pass the threshold the driver wrote;
//! the capture then stops on its own and the status says so.
//!
//! **The clock's start** (assumed): the C calls these counters "elapsed
//! time in ms" and "capture duration in ms" and only a struct comment
//! says "since trigger" (`duration_now`, `lwla.h`), while
//! `handle_status_response()` tests the time limit before the trigger
//! (`protocol.c:144-148`). The model counts from the trigger. If the real
//! counter runs from the arming, a trigger that never fires with
//! `--samples` would end on the derived time limit rather than hang: the
//! two `--hangs` cases of the scenario depend on this assumption.
//!
//! **The memory reset's second read** (LWLA1016): "Issue another read
//! request or the device will stall" (`lwla1016.c:281`). The model does
//! not stall, but after the first memory read following a memory reset
//! in read mode anything other than a second memory read is flagged. A
//! capture's readout owes none: the memory went through write mode.
//!
//! **Pedantic checks**: commands of the wrong length, unknown ids,
//! commands of the other model, a command while a reply is unread, a
//! reply read with fewer bytes than it has; the bitstream byte for byte
//! and its length header; commands to a blank or shut-down FPGA;
//! registers the driver does not name, writes of read-only ones and
//! reads of write-only ones, values outside what the driver writes
//! (memory and capture control values, a boost other than 0 or 1, a
//! divider beyond 100 Hz or with the boost, masks beyond the channels,
//! trigger levels or types outside the enable mask, a threshold beyond
//! the memory, non-zero writes to the status long registers); the long
//! register protocol (low or high read without a strobe read); a
//! capture armed without its configuration (long registers written,
//! capture control cleared, memory in write mode; on the LWLA1016 the
//! threshold written and the memory in write mode); the capture's
//! length read while it runs; memory read in write mode, beyond the
//! memory, in counts that are not whole slices, over 1024 bytes, or (in
//! a readout) away from the read pointer.
//!
//! **Faults**, all off by default: `stall=KIND[:N]` STALLs the command
//! (every one, or the Nth of its kind since the model was made; it has
//! no effect), `stall_reply=` STALLs its reply (the reply is dropped),
//! `short_reply=` sends the reply one 16-bit word short, with `KIND`
//! one of `read-reg`, `write-reg`, `read-mem`, `write-lregs`,
//! `read-lregs`, `bitstream` (comma lists; the reply faults only for
//! the reading ones); `test_id=0xN` makes the test word read N (a
//! bitstream that did not load right); `halt_in=1` halts endpoint 6
//! (every IN STALLs, the drain at the open included); `stale=N` leaves
//! N bytes in the reply endpoint at attach, as after a crash (the
//! open's drain is for this). A one-shot STALL is an emulation
//! artifact: a real STALL halts the endpoint until it is cleared, which
//! the driver never does.
//!
//! **Emulation artifacts**, in one place: the clock counted in status
//! polls; replies at once; the one-shot STALLs; the invented signal,
//! the external clock's and TRG input's rates; zeros where the memory
//! was not written and in long registers 0 to 4, 6 and 8.
//!
//! **Not reached by the cases** (`tests/emu/sysclk-lwla/gcov.txt`):
//!
//! - Cancelling (`dev_acquisition_stop()` with a capture running,
//!   `protocol.c:293,354,360,366,419,425`): sigrok-cli stops a session
//!   only on a failed decoder, and a failed start leaves nothing to
//!   cancel; it has no signal handler for an interrupt case.
//! - A capture with no words (`protocol.c:178-179`) on the LWLA1034: each
//!   sample there is a word at once, and the model's fill grows with the
//!   time that ends a capture, so it would take a fault, and none was
//!   invented for it. The LWLA1016 reaches it (the case "1016 capture with
//!   no words": one sample does not fill a word of two).
//! - A failing submission of the start (`protocol.c:567-568`): only an
//!   unplug causes it, and whether the submission or its completion fails
//!   depends on timing.
//! - A close during an acquisition (`api.c:296-299`): sigrok-cli closes
//!   only after the session ended.
//! - The second interface claim (`api.c:248-251`): another program's
//!   claim fails at `api.c:238` first.
//! - A partly taken OUT (`lwla.c:106-108,136-138`): a bulk OUT that does
//!   not finish ends in an error, which is checked first.
//! - sigrok-cli checks first: `api.c:327,344,404,518,531-532,594`, one
//!   trigger stage (`api.c:505-506`), re-enabling a channel (`api.c:477`).
//! - Cannot happen: argument and "BUG" checks, allocation failures,
//!   libusb's list and descriptor calls, a file that shrinks while read
//!   (`lwla.c:67-69`), a second start or commit during a capture.
//!
//! **What the model shows about the driver** (C line numbers of
//! libsigrok 0bc24877):
//!
//! - `dev_inst_new_matching()` filters by `conn=` only when
//!   `sr_usb_find()` found something (`api.c:104`): a `conn` that matches
//!   no device, or that `sr_usb_find()` refuses, selects every LWLA.
//! - `config_set()` takes any rate in the range (`api.c:409-411`) and
//!   the divider is an integer division (`lwla1034.c:390-394`,
//!   `lwla1016.c:312-315`): the board runs at 3.03 MHz for 3 MHz, at 100
//!   MHz for 60 MHz, and the LWLA1034 at 125 MHz for anything above 100
//!   MHz (`protocol.c:503`), while the limits and the output's header use
//!   the rate asked for.
//! - On the external clock neither limit is derived from the other
//!   (`protocol.c:499-519`): with `--samples` the capture has no time
//!   limit and runs until the memory fills, however few samples were
//!   asked for; with `--time` it has no sample limit and all of the
//!   memory is read.
//! - A time limit shorter than a sample makes `samples_max` 0
//!   (`protocol.c:510-511`), which `handle_read_response()` takes as done
//!   (`protocol.c:212-214`): one read, and no samples, exit 0.
//! - An error during a capture ends it with exit 0
//!   (`protocol.c:299-309`): only stderr tells, and samples not yet sent
//!   in a packet are lost (`protocol.c:216-219`).
//! - A close from `sr_exit()` goes through `std_dev_clear()` with the
//!   status still ACTIVE (`std.c:427-428`; only `sr_dev_close()` sets
//!   INACTIVE first, `device.c:779`), so `apply_fpga_config()` finds "No
//!   change" (`lwla1034.c:314-324`) and the FPGA stays on: the LWLA1034's
//!   shut-down bitstream goes out only after `--show`, `--get` and `--set`
//!   (`sr_dev_close()`), not after a capture.
//! - A claim held by another program fails at `libusb_set_configuration()`
//!   (`api.c:238`, `LIBUSB_ERROR_BUSY`), before the claim of `api.c:246`.
//! - Read only, not shown by a case: `prepare_trigger_masks()` returns
//!   early without a trigger (`api.c:501-502`), leaving the masks of an
//!   earlier session in `devc` for a later one in the same process.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{BULK, Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::ezusb::standin;
use crate::{Error, Result};
use std::collections::VecDeque;

/// The model name.
pub const NAME: &str = "sysclk-lwla";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Sysclk LWLA1034 (2961:6689) or LWLA1016 (2961:6688, model=1016), FX2 \
                         relaying word commands to an FPGA that takes its bitstream on EP4; \
                         period=N, tick=MS, ext_clock=HZ, ext_trg=N, short_transfers=1, stale=N; \
                         faults: stall=, stall_reply=, short_reply=KIND[:N], test_id=0xN, \
                         halt_in=1";

/// The vendor id (`USB_VID_SYSCLK`).
pub const VID: u16 = 0x2961;
/// The LWLA1016's product id.
pub const PID_1016: u16 = 0x6688;
/// The LWLA1034's product id.
pub const PID_1034: u16 = 0x6689;

const EP_COMMAND: u8 = 2;
const EP_CONFIG: u8 = 4;
const EP_REPLY: u8 = 6;
/// The bulk packet size at high speed.
const PACKET: usize = 512;
/// The longest reply: the FX2's two endpoint buffers.
const REPLY_MAX: usize = 1024;
/// The reply transfers with the short transfer quirk.
const QUIRK_PACKET: usize = 64;
/// The largest bitstream the driver sends (`BITSTREAM_MAX_SIZE`).
const BITSTREAM_MAX: usize = 256 * 1024;
/// Sample memory, in words, of both models (`MEMORY_DEPTH`).
const DEPTH: u32 = 256 * 1024;
/// The slowest divider: 100 MHz down to 100 Hz.
const DIV_MAX: u64 = 999_999;

const CMD_READ_REG: u16 = 1;
const CMD_WRITE_REG: u16 = 2;
const CMD_READ_MEM32: u16 = 3;
const CMD_READ_MEM36: u16 = 6;
const CMD_WRITE_LREGS: u16 = 7;
const CMD_READ_LREGS: u16 = 8;

/// The bitstream stand-ins: name and size (`tools/emu/sysclk-lwla.firmware`).
pub const BITSTREAMS: [(&str, usize); 6] = [
    ("sysclk-lwla1034-off.rbf", 3000),
    ("sysclk-lwla1034-int.rbf", 10_000),
    ("sysclk-lwla1034-extpos.rbf", 10_001),
    ("sysclk-lwla1034-extneg.rbf", 10_002),
    ("sysclk-lwla1016-100.rbf", 9000),
    ("sysclk-lwla1016-100-ts.rbf", 9001),
];

/// The FPGA designs, in the order of [`BITSTREAMS`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Design {
    Off,
    Int,
    ExtPos,
    ExtNeg,
    Plain,
    Timing,
}

const DESIGNS: [Design; 6] =
    [Design::Off, Design::Int, Design::ExtPos, Design::ExtNeg, Design::Plain, Design::Timing];

/// The two boards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Board {
    /// 16 channels, 32-bit memory words.
    Lwla1016,
    /// 34 channels, 36-bit memory words, long registers.
    Lwla1034,
}

impl Board {
    fn channels(self) -> u32 {
        match self {
            Board::Lwla1016 => 16,
            Board::Lwla1034 => 34,
        }
    }

    fn all(self) -> u64 {
        (1u64 << self.channels()) - 1
    }

    fn designs(self) -> &'static [Design] {
        match self {
            Board::Lwla1016 => &DESIGNS[4..],
            Board::Lwla1034 => &DESIGNS[..4],
        }
    }

    /// The first memory address the driver reads (`READ_START_ADDR`).
    fn read_start(self) -> u32 {
        match self {
            Board::Lwla1016 => 2,
            Board::Lwla1034 => 4,
        }
    }
}

// LWLA1034 registers.
const R34_MEM_CTRL: u16 = 0x1074;
const R34_MEM_FILL: u16 = 0x1078;
const R34_MEM_START: u16 = 0x107c;
const R34_CLK_BOOST: u16 = 0x1094;
const R34_LONG_STROBE: u16 = 0x10b0;
const R34_LONG_ADDR: u16 = 0x10b4;
const R34_LONG_LOW: u16 = 0x10b8;
const R34_LONG_HIGH: u16 = 0x10bc;
const L_MEM_FILL: usize = 5;
const L_DURATION: usize = 7;
const L_STATUS: usize = 9;
const L_CAP_CTRL: u32 = 10;
const L_TEST_ID: u32 = 100;
/// Capture control: clear the time base, flush, clear the full flag
/// and the counter (`capture_init`).
const CAP34_CLEAR: u64 = 0x74;
const CAP34_TRG_EN: u64 = 1;
/// Status bits, before the driver's shift.
const ST34_CAPTURING: u64 = 1 << 1;
const ST34_TRIGGERED: u64 = 1 << 4;
const ST34_MEM_AVAIL: u64 = 1 << 5;
const TEST_ID_1034: u64 = 0x1234_5678_8765_4321;

// LWLA1016 registers.
const R16_CHAN_MASK: u16 = 0x1000;
const R16_DURATION: u16 = 0x1010;
const R16_MEM_WR_PTR: u16 = 0x1070;
const R16_MEM_CTRL: u16 = 0x107c;
const R16_CAP_COUNT: u16 = 0x10b0;
const R16_TEST_ID: u16 = 0x10b4;
const R16_CAP_CTRL: u16 = 0x10b8;
const R16_DIV_COUNT: u16 = 0x10bc;
const MEM16_RESET: u32 = 1;
const MEM16_WRITE: u32 = 2;
const CAP16_TRG_EN: u32 = 1 << 2;
const CAP16_SAMPLE_EN: u32 = 1 << 5;
const CAP16_NOT_END: u32 = 1 << 6;
const TEST_ID_1016: u32 = 0x1234_5678;

/// What a fault aims at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    ReadReg,
    WriteReg,
    ReadMem,
    WriteLregs,
    ReadLregs,
    Bitstream,
}

const KINDS: [(&str, Kind); 6] = [
    ("read-reg", Kind::ReadReg),
    ("write-reg", Kind::WriteReg),
    ("read-mem", Kind::ReadMem),
    ("write-lregs", Kind::WriteLregs),
    ("read-lregs", Kind::ReadLregs),
    ("bitstream", Kind::Bitstream),
];

/// A fault aimed at a kind of command: every one (`nth` `None`) or the Nth.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Shot {
    kind: Kind,
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
                    .ok_or_else(|| Error::Invalid(format!("{k}={item}: N from 1")))?;
                (name, Some(n))
            }
            None => (item, None),
        };
        let kind = KINDS.iter().find(|(n, _)| *n == name).map(|(_, c)| *c).ok_or_else(|| {
            let names: Vec<&str> = KINDS.iter().map(|(n, _)| *n).collect();
            Error::Invalid(format!("{k}={name}: one of {}", names.join(", ")))
        })?;
        if k != "stall" && matches!(kind, Kind::WriteReg | Kind::WriteLregs | Kind::Bitstream) {
            return Err(Error::Invalid(format!("{k}={name}: only read-reg, read-mem and read-lregs reply")));
        }
        out.push(Shot { kind, nth });
    }
    Ok(out)
}

fn hit(list: &[Shot], kind: Kind, count: u32) -> bool {
    list.iter().any(|s| s.kind == kind && s.nth.is_none_or(|n| n == count))
}

fn number(k: &str, v: &str) -> Result<u64> {
    let parsed = match v.strip_prefix("0x") {
        Some(hex) => u64::from_str_radix(hex, 16).ok(),
        None => v.parse().ok(),
    };
    parsed.ok_or_else(|| Error::Invalid(format!("{k}={v}: a number")))
}

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    stall: Vec<Shot>,
    stall_reply: Vec<Shot>,
    short_reply: Vec<Shot>,
    test_id: Option<u64>,
    halt_in: bool,
}

/// The invented signal and the board's pace.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Signal {
    /// Samples per step of the counter.
    pub period: u64,
    /// Samples per half period of the external TRG input.
    pub ext_trg: u64,
    /// The external clock, in Hz.
    pub ext_clock: u64,
    /// Board milliseconds per status poll.
    pub tick: u64,
    /// The channels.
    pub channels: u32,
}

impl Signal {
    /// The inputs at sample `s`: a counter.
    pub fn input(&self, s: u64) -> u64 {
        (s / self.period) & ((1u64 << self.channels) - 1)
    }

    /// The external TRG input at sample `s`.
    pub fn trg(&self, s: u64) -> bool {
        (s / self.ext_trg) % 2 == 1
    }

    /// The first sample after `s` where the inputs under `mask` change.
    fn run_end(&self, s: u64, mask: u64) -> u64 {
        if mask == 0 {
            return u64::MAX;
        }
        let len = 1u64.checked_shl(mask.trailing_zeros()).and_then(|b| b.checked_mul(self.period));
        match len {
            Some(len) => (s / len + 1).saturating_mul(len),
            None => u64::MAX,
        }
    }
}

/// A trigger: levels and edges of the channels, edges of the TRG input.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Trigger {
    /// Channels with a condition.
    pub mask: u64,
    /// Their levels.
    pub values: u64,
    /// The channels whose condition is an edge.
    pub edges: u64,
    /// A rising edge on TRG.
    pub trg_rise: bool,
    /// A falling edge on TRG.
    pub trg_fall: bool,
}

impl Trigger {
    /// Whether every condition holds at sample `s`.
    pub fn holds(&self, sig: &Signal, s: u64) -> bool {
        let now = sig.input(s);
        let levels = self.mask & !self.edges;
        if (now ^ self.values) & levels != 0 {
            return false;
        }
        let edges = self.mask & self.edges;
        if edges != 0 {
            if s == 0 {
                return false;
            }
            let before = sig.input(s - 1);
            if (now ^ before) & edges != edges || (now ^ self.values) & edges != 0 {
                return false;
            }
        }
        if self.trg_rise || self.trg_fall {
            if s == 0 {
                return false;
            }
            let (b, n) = (sig.trg(s - 1), sig.trg(s));
            let (rose, fell) = (n && !b, b && !n);
            if (self.trg_rise && !rose) || (self.trg_fall && !fell) {
                return false;
            }
        }
        true
    }

    /// The first sample in `from..to` where it fires.
    pub fn search(&self, sig: &Signal, from: u64, to: u64) -> Option<u64> {
        if from >= to {
            return None;
        }
        let step = if self.trg_rise || self.trg_fall {
            sig.ext_trg
        } else if self.mask == 0 {
            return Some(from);
        } else {
            // The inputs change only at the counter's steps.
            sig.period
        };
        let mut s = from.div_ceil(step).saturating_mul(step);
        while s < to {
            if self.holds(sig, s) {
                return Some(s);
            }
            s = s.saturating_add(step);
        }
        None
    }
}

/// How the memory holds samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// LWLA1034: 34 bits, a run bit and a length-follows bit.
    Rle36,
    /// LWLA1016 "-ts": a sample and its run length less one.
    Rle16,
    /// LWLA1016: two samples per word.
    Plain16,
}

/// The longest run one LWLA1034 pair of words holds: 2 + 2 * (2^36 - 1).
const RUN36_MAX: u64 = 2 * ((1 << 36) - 1) + 2;

/// The memory words for samples `from..to` of the channels in `mask`,
/// at most `limit` of them: the words, the sample after the last one
/// stored, and whether the memory ran full before `to`.
pub fn encode(
    sig: &Signal,
    format: Format,
    mask: u64,
    from: u64,
    to: u64,
    limit: usize,
) -> (Vec<u64>, u64, bool) {
    let mut words = Vec::new();
    if format == Format::Plain16 {
        let pairs = (to.saturating_sub(from) / 2).min(limit as u64);
        for i in 0..pairs {
            let s = from + 2 * i;
            words.push((sig.input(s) & mask) | (sig.input(s + 1) & mask) << 16);
        }
        let full = (to.saturating_sub(from) / 2) > limit as u64;
        return (words, from + 2 * pairs, full);
    }
    let mut s = from;
    while s < to {
        let end = sig.run_end(s, mask).min(to);
        let value = sig.input(s) & mask;
        let mut n = end - s;
        while n > 0 {
            let (piece, need) = match format {
                Format::Rle36 => {
                    let m = n.min(RUN36_MAX);
                    (m, if m <= 2 { 1 } else { 2 })
                }
                _ => (n.min(1 << 16), 1),
            };
            if words.len() + need > limit {
                return (words, s, true);
            }
            match format {
                Format::Rle36 if piece <= 2 => words.push(value | (piece - 1) << 34),
                Format::Rle36 => {
                    words.push(value | ((piece - 1) % 2) << 34 | 1 << 35);
                    words.push((piece - 1) / 2);
                }
                _ => words.push(value | (piece - 1) << 16),
            }
            s += piece;
            n -= piece;
        }
    }
    (words, s, false)
}

/// A capture from its start to its readout.
#[derive(Clone, Debug)]
struct Capture {
    rate: u64,
    boost: bool,
    mask: u64,
    trigger: Trigger,
    format: Format,
    /// Memory words before the capture counts as full.
    limit: usize,
    /// Board time since the start, ms.
    ms: u64,
    searched: u64,
    at: Option<u64>,
    end: Option<u64>,
    full: bool,
    words: Vec<u64>,
}

impl Capture {
    fn now(&self) -> u64 {
        (u128::from(self.ms) * u128::from(self.rate) / 1000).try_into().unwrap_or(u64::MAX)
    }

    fn running(&self) -> bool {
        self.end.is_none()
    }

    /// One status poll: the clock moves on.
    fn poll(&mut self, sig: &Signal) {
        if !self.running() {
            return;
        }
        self.ms += sig.tick;
        let now = self.now();
        if self.at.is_none() {
            self.at = self.trigger.search(sig, self.searched, now);
            self.searched = now;
        }
        if let Some(t) = self.at {
            let (words, stop, full) = encode(sig, self.format, self.mask, t, now, self.limit);
            self.words = words;
            if full {
                self.end = Some(stop);
                self.full = true;
            }
        }
    }

    /// Stopped by the host.
    fn stop(&mut self, sig: &Signal) {
        if !self.running() {
            return;
        }
        let now = self.now();
        if let Some(t) = self.at {
            let (words, stop, _) = encode(sig, self.format, self.mask, t, now, self.limit);
            self.words = words;
            self.end = Some(stop);
        } else {
            self.words.clear();
            self.end = Some(now);
        }
    }

    /// Milliseconds since the trigger; with the boost, units of 0.8 ms.
    fn duration(&self) -> u64 {
        let Some(t) = self.at else { return 0 };
        let end = self.end.unwrap_or_else(|| self.now());
        let unit = if self.boost { 1250 } else { 1000 };
        let ms = u128::from(end - t) * unit / u128::from(self.rate);
        u64::try_from(ms).unwrap_or(u64::MAX)
    }
}

/// A reply transfer waiting on endpoint 6.
#[derive(Clone, Debug)]
struct Chunk {
    data: Vec<u8>,
    stall: bool,
    /// Bytes of the whole reply, for the short read check.
    total: usize,
}

/// The LWLA1034's registers.
#[derive(Clone, Debug, Default)]
struct Regs34 {
    mem_ctrl: u32,
    clk_boost: u32,
    read_ptr: Option<u32>,
    long_addr: u32,
    long_low: u32,
    long_high: u32,
    latched: bool,
    /// Long registers 0 to 10 as written.
    lregs: [u64; 11],
    configured: bool,
    cleared: bool,
}

/// The LWLA1016's registers.
#[derive(Clone, Debug, Default)]
struct Regs16 {
    chan_mask: u32,
    div: u32,
    mem_ctrl: u32,
    cap_count: Option<u32>,
    trg_sel: u32,
    read_ptr: Option<u32>,
    /// The memory was reset and has not been read or in write mode since.
    reset_reads: bool,
    /// One memory read since that reset: a second one is due.
    second_read_due: bool,
}

/// The board.
pub struct Lwla {
    board: Board,
    desc: Descriptors,
    sig: Signal,
    faults: Faults,
    short_transfers: bool,
    stale: usize,
    design: Option<Design>,
    test_reads: u32,
    pending: VecDeque<Chunk>,
    counts: [u32; 6],
    r34: Regs34,
    r16: Regs16,
    capture: Option<Capture>,
}

/// The descriptors of a board.
pub fn descriptors(board: Board) -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 3, [0xff, 0xff, 0xff], 0));
    for address in [EP_COMMAND, EP_CONFIG, 0x80 | EP_REPLY] {
        body.extend_from_slice(&descriptor::endpoint(address, BULK, PACKET as u16, 0));
    }
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: VID,
        product: if board == Board::Lwla1016 { PID_1016 } else { PID_1034 },
        device: 0x0000,
        strings: [0, 0, 0],
        configurations: 1,
    };
    Descriptors { speed: Speed::High, device: dev.bytes(), configs: vec![config], strings: vec![] }
}

/// The bitstream and the memory reads.
pub fn log_rules(_: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules::digest_above(96)
}

/// A board from the `-o` options: `model=1034|1016`, `period=N`,
/// `tick=MS`, `ext_clock=HZ`, `ext_trg=N`, `short_transfers=0|1`,
/// `stale=N` and the faults (see [`Faults`]).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut board = Board::Lwla1034;
    let mut sig = Signal { period: 1, ext_trg: 1000, ext_clock: 1_000_000, tick: 100, channels: 34 };
    let mut faults = Faults::default();
    let mut short_transfers = false;
    let mut stale = 0usize;
    for (k, v) in options {
        let positive = |v: &str| {
            number(k, v).ok().filter(|n| *n > 0).ok_or_else(|| Error::Invalid(format!("{k}={v}: from 1")))
        };
        let flag = |v: &str| match v {
            "0" => Ok(false),
            "1" => Ok(true),
            _ => Err(Error::Invalid(format!("{k}={v}: 0 or 1"))),
        };
        match k.as_str() {
            "model" => {
                board = match v.as_str() {
                    "1034" => Board::Lwla1034,
                    "1016" => Board::Lwla1016,
                    _ => return Err(Error::Invalid(format!("model={v}: 1034 or 1016"))),
                }
            }
            "period" => sig.period = positive(v)?,
            "tick" => sig.tick = positive(v)?,
            "ext_clock" => sig.ext_clock = positive(v)?,
            "ext_trg" => sig.ext_trg = positive(v)?,
            "short_transfers" => short_transfers = flag(v)?,
            "stale" => {
                stale = usize::try_from(number(k, v)?)
                    .ok()
                    .filter(|n| *n <= 4096)
                    .ok_or_else(|| Error::Invalid(format!("stale={v}: 0 to 4096 bytes")))?;
            }
            "stall" => faults.stall = shots(k, v)?,
            "stall_reply" => faults.stall_reply = shots(k, v)?,
            "short_reply" => faults.short_reply = shots(k, v)?,
            "test_id" => faults.test_id = Some(number(k, v)?),
            "halt_in" => faults.halt_in = flag(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    sig.channels = board.channels();
    let mut model = Lwla::new(board, sig);
    model.faults = faults;
    model.short_transfers = short_transfers;
    model.stale = stale;
    model.reset();
    Ok(Box::new(model))
}

/// Appends a 32-bit value as the board sends it: high half first.
fn put32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&((v >> 16) as u16).to_le_bytes());
    out.extend_from_slice(&(v as u16).to_le_bytes());
}

/// Appends a 64-bit value: its low 32 bits, then its high 32 bits.
fn put64(out: &mut Vec<u8>, v: u64) {
    put32(out, v as u32);
    put32(out, (v >> 32) as u32);
}

/// The 32-bit value of two command words.
fn word32(w: &[u16]) -> u32 {
    u32::from(w[0]) << 16 | u32::from(w[1])
}

impl Lwla {
    /// A board in its power-up state.
    pub fn new(board: Board, sig: Signal) -> Lwla {
        Lwla {
            board,
            desc: descriptors(board),
            sig,
            faults: Faults::default(),
            short_transfers: false,
            stale: 0,
            design: None,
            test_reads: 0,
            pending: VecDeque::new(),
            counts: [0; 6],
            r34: Regs34::default(),
            r16: Regs16::default(),
            capture: None,
        }
    }

    fn kind_index(kind: Kind) -> usize {
        KINDS.iter().position(|(_, k)| *k == kind).unwrap_or(0)
    }

    /// Counts a command of `kind`; whether it is to be stalled.
    fn count(&mut self, kind: Kind) -> (u32, bool) {
        let i = Self::kind_index(kind);
        self.counts[i] += 1;
        let n = self.counts[i];
        (n, hit(&self.faults.stall, kind, n))
    }

    /// Queues a reply, cut into transfers as the firmware sends it.
    fn reply(&mut self, kind: Kind, nth: u32, mut data: Vec<u8>) {
        if hit(&self.faults.short_reply, kind, nth) {
            data.truncate(data.len().saturating_sub(2));
        }
        let stall = hit(&self.faults.stall_reply, kind, nth);
        let total = data.len();
        if stall {
            self.pending.push_back(Chunk { data: Vec::new(), stall, total });
            return;
        }
        if self.short_transfers && data.len() > QUIRK_PACKET {
            for c in data.chunks(QUIRK_PACKET) {
                self.pending.push_back(Chunk { data: c.to_vec(), stall: false, total });
            }
        } else {
            self.pending.push_back(Chunk { data, stall: false, total });
        }
    }

    fn bitstream(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let (_, stall) = self.count(Kind::Bitstream);
        if stall {
            return Reply::Stall;
        }
        if data.len() < 4 {
            v.report(format!("bitstream transfer of {} bytes, no length header", data.len()));
            return Reply::ack();
        }
        let header = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;
        if header != data.len() {
            v.report(format!(
                "bitstream header says {header} bytes, the transfer has {}",
                data.len()
            ));
        }
        if data.len() - 4 > BITSTREAM_MAX {
            v.report(format!("bitstream of {} bytes, over 256 KiB", data.len() - 4));
        }
        let body = &data[4..];
        let found = self.board.designs().iter().find(|d| {
            let i = DESIGNS.iter().position(|x| x == *d).unwrap_or(0);
            let (name, size) = BITSTREAMS[i];
            body.len() == size && body == standin(name, size).as_slice()
        });
        let Some(&design) = found else {
            v.report(format!(
                "bitstream of {} bytes is none of the {:?}'s stand-ins",
                body.len(),
                self.board
            ));
            self.design = None;
            return Reply::ack();
        };
        // A new design: the FPGA starts over.
        self.design = Some(design);
        self.test_reads = 0;
        self.r34 = Regs34::default();
        self.r16 = Regs16::default();
        self.capture = None;
        Reply::ack()
    }

    /// Whether the FPGA can take commands; flags it when not.
    fn fpga_up(&self, what: &str, v: &mut Violations) -> bool {
        match self.design {
            None => {
                v.report(format!("{what} before a bitstream: the FPGA is blank"));
                false
            }
            Some(Design::Off) => {
                v.report(format!("{what} after the shut-down bitstream"));
                false
            }
            Some(_) => true,
        }
    }

    fn command(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if !data.len().is_multiple_of(2) || data.len() < 2 {
            v.report(format!("command of {} bytes; commands are 16-bit words", data.len()));
            return Reply::ack();
        }
        let w: Vec<u16> = data.chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        let (kind, want) = match w[0] {
            CMD_READ_REG => (Kind::ReadReg, Some(2)),
            CMD_WRITE_REG => (Kind::WriteReg, Some(4)),
            CMD_READ_MEM32 | CMD_READ_MEM36 => (Kind::ReadMem, Some(5)),
            CMD_WRITE_LREGS => (Kind::WriteLregs, None),
            CMD_READ_LREGS => (Kind::ReadLregs, Some(3)),
            other => {
                v.report(format!("unknown command {other}"));
                return Reply::ack();
            }
        };
        let (nth, stall) = self.count(kind);
        if stall {
            return Reply::Stall;
        }
        if let Some(n) = want
            && w.len() != n
        {
            v.report(format!("command {} of {} words; it has {n}", w[0], w.len()));
            return Reply::ack();
        }
        let other_board = match self.board {
            Board::Lwla1016 => matches!(w[0], CMD_READ_MEM36 | CMD_WRITE_LREGS | CMD_READ_LREGS),
            Board::Lwla1034 => w[0] == CMD_READ_MEM32,
        };
        if other_board {
            v.report(format!("command {} is not the {:?}'s", w[0], self.board));
            return Reply::ack();
        }
        if self.board == Board::Lwla1016 && self.r16.second_read_due && kind != Kind::ReadMem {
            v.report(format!(
                "command {} after one memory read since the memory reset; the device stalls \
                 without a second (lwla1016.c:281)",
                w[0]
            ));
            self.r16.second_read_due = false;
        }
        if !self.pending.is_empty() {
            v.report(format!("command {} while a reply is unread", w[0]));
            self.pending.clear();
        }
        let up = self.fpga_up(&format!("command {}", w[0]), v);
        let answer = match (w[0], up) {
            (CMD_READ_REG, true) => {
                let mut out = Vec::new();
                let value = match self.board {
                    Board::Lwla1034 => self.read34(w[1], v),
                    Board::Lwla1016 => self.read16(w[1], v),
                };
                put32(&mut out, value);
                Some(out)
            }
            (CMD_READ_REG, false) => Some(vec![0; 4]),
            (CMD_WRITE_REG, true) => {
                let value = word32(&w[2..4]);
                match self.board {
                    Board::Lwla1034 => self.write34(w[1], value, v),
                    Board::Lwla1016 => self.write16(w[1], value, v),
                }
                None
            }
            (CMD_READ_MEM32 | CMD_READ_MEM36, up) => {
                let (address, count) = (word32(&w[1..3]), word32(&w[3..5]));
                Some(self.read_memory(address, count, up, v))
            }
            (CMD_WRITE_LREGS, true) => {
                self.write_lregs(&w, v);
                None
            }
            (CMD_READ_LREGS, up) => Some(self.read_lregs(w[1], w[2], up, v)),
            _ => None,
        };
        if let Some(out) = answer {
            self.reply(kind, nth, out);
        }
        Reply::ack()
    }

    fn read_memory(&mut self, address: u32, count: u32, up: bool, v: &mut Violations) -> Vec<u8> {
        let wide = self.board == Board::Lwla1034;
        let bytes = if wide { count as usize / 8 * 36 } else { count as usize * 4 };
        if count == 0 {
            v.report("memory read of 0 words");
        }
        if wide && !count.is_multiple_of(8) {
            v.report(format!("36-bit memory read of {count} words, not whole slices of 8"));
        }
        if bytes > REPLY_MAX {
            v.report(format!("memory read of {bytes} bytes, over the FX2's {REPLY_MAX}"));
        }
        if u64::from(address) + u64::from(count) > u64::from(DEPTH) {
            v.report(format!("memory read of {count} words at {address}, beyond the memory"));
        }
        if self.board == Board::Lwla1016 {
            let r = &mut self.r16;
            r.second_read_due = std::mem::take(&mut r.reset_reads);
        }
        let (write_mode, read_ptr) = match self.board {
            Board::Lwla1034 => (self.r34.mem_ctrl & 1 != 0, self.r34.read_ptr),
            Board::Lwla1016 => (self.r16.mem_ctrl & MEM16_WRITE != 0, self.r16.read_ptr),
        };
        if up && write_mode {
            v.report("memory read while the memory is in write mode");
        }
        if up && self.capture.as_ref().is_some_and(Capture::running) {
            v.report("memory read while capturing");
        }
        match read_ptr {
            Some(p) if p != address => {
                v.report(format!("memory read at {address}; the read pointer is at {p}"));
            }
            None if wide && up => v.report("memory read before its start address was set"),
            _ => {}
        }
        if let Some(p) = read_ptr {
            match self.board {
                Board::Lwla1034 => self.r34.read_ptr = Some(p.wrapping_add(count)),
                Board::Lwla1016 => self.r16.read_ptr = Some(p.wrapping_add(count)),
            }
        }
        let base = self.board.read_start();
        let words = self.capture.as_ref().map(|c| c.words.as_slice()).unwrap_or_default();
        let word = |a: u32| -> u64 {
            if !up || a < base {
                return 0;
            }
            words.get((a - base) as usize).copied().unwrap_or(0)
        };
        // Flagged above when over the FX2's two buffers; a count from the
        // host is never allocated beyond them.
        let count = if wide { count.min(REPLY_MAX as u32 / 36 * 8) } else { count.min(REPLY_MAX as u32 / 4) };
        let mut out = Vec::with_capacity(REPLY_MAX);
        if wide {
            for slice in 0..count / 8 {
                let mut high = 0u32;
                for i in 0..8 {
                    let w = word(address.wrapping_add(slice * 8 + i));
                    put32(&mut out, w as u32);
                    high |= ((w >> 32) as u32 & 0xf) << (28 - 4 * i);
                }
                put32(&mut out, high);
            }
        } else {
            for i in 0..count {
                put32(&mut out, word(address.wrapping_add(i)) as u32);
            }
        }
        out
    }

    // --- LWLA1034 ---

    fn read34(&mut self, reg: u16, v: &mut Violations) -> u32 {
        let r = &mut self.r34;
        match reg {
            R34_LONG_STROBE => {
                r.latched = true;
                let value = self.long_value(self.r34.long_addr, v);
                self.r34.long_low = value as u32;
                self.r34.long_high = (value >> 32) as u32;
                0
            }
            R34_LONG_LOW | R34_LONG_HIGH => {
                if !r.latched {
                    v.report(format!(
                        "long register word {reg:#06x} read without a strobe read since the \
                         address"
                    ));
                }
                if reg == R34_LONG_LOW { r.long_low } else { r.long_high }
            }
            R34_MEM_FILL => {
                if self.capture.as_ref().is_some_and(Capture::running) {
                    v.report("memory fill level read while capturing");
                }
                self.fill34()
            }
            _ => {
                v.report(format!("read of register {reg:#06x}, which the driver does not read"));
                0
            }
        }
    }

    /// The fill level: the address after the last word.
    fn fill34(&self) -> u32 {
        let n = self.capture.as_ref().map_or(0, |c| c.words.len());
        Board::Lwla1034.read_start() + n as u32
    }

    /// A long register as read.
    fn long_value(&mut self, addr: u32, v: &mut Violations) -> u64 {
        match addr {
            L_TEST_ID => {
                self.test_reads += 1;
                if self.test_reads == 1 {
                    0
                } else {
                    self.faults.test_id.unwrap_or(TEST_ID_1034)
                }
            }
            0..=10 => self.lreg_read(addr as usize),
            _ => {
                v.report(format!("read of long register {addr}, which the driver does not read"));
                0
            }
        }
    }

    /// Long register `i` (0 to 10) as read.
    fn lreg_read(&self, i: usize) -> u64 {
        let c = self.capture.as_ref();
        match i {
            L_MEM_FILL => u64::from(self.fill34()),
            L_DURATION => c.map_or(0, Capture::duration),
            L_STATUS => c.map_or(0, |c| {
                let mut s = 0;
                if c.running() {
                    s |= ST34_CAPTURING;
                }
                if c.at.is_some() {
                    s |= ST34_TRIGGERED;
                }
                if !c.full {
                    s |= ST34_MEM_AVAIL;
                }
                s
            }),
            10 => self.r34.lregs[10],
            // "The first five long registers do not return useful values
            // when read" (lwla1034.c); 6 has no name.
            _ => 0,
        }
    }

    fn write34(&mut self, reg: u16, value: u32, v: &mut Violations) {
        let r = &mut self.r34;
        match reg {
            R34_MEM_CTRL => {
                if value != 1 && value != 2 {
                    v.report(format!("memory control {value:#x}; the driver writes 1 or 2"));
                }
                r.mem_ctrl = value;
                if value & 2 != 0 {
                    // clr_idx: the read index goes back to the start
                    // address, which the next write sets.
                    r.read_ptr = None;
                }
            }
            R34_MEM_START => {
                if value >= DEPTH {
                    v.report(format!("memory start {value}, beyond the memory"));
                }
                r.read_ptr = Some(value);
            }
            R34_CLK_BOOST => {
                if value > 1 {
                    v.report(format!("clock boost {value}; 0 or 1"));
                }
                r.clk_boost = value;
            }
            R34_LONG_ADDR => {
                r.long_addr = value;
                r.latched = false;
            }
            R34_LONG_LOW => r.long_low = value,
            R34_LONG_HIGH => r.long_high = value,
            R34_LONG_STROBE => {
                if value != 0 {
                    v.report(format!("long register strobe written {value}; the driver writes 0"));
                }
                let full = u64::from(r.long_high) << 32 | u64::from(r.long_low);
                let addr = r.long_addr;
                if addr == L_CAP_CTRL {
                    self.capture_control34(full, v);
                } else {
                    v.report(format!(
                        "long register {addr} written alone; the driver writes only capture \
                         control (10) so"
                    ));
                }
            }
            R34_MEM_FILL => v.report("write of the memory fill level, which is read only"),
            _ => v.report(format!("write of register {reg:#06x}, which the driver does not write")),
        }
    }

    fn capture_control34(&mut self, value: u64, v: &mut Violations) {
        self.r34.lregs[10] = value;
        match value {
            CAP34_CLEAR => {
                if self.capture.as_ref().is_some_and(Capture::running) {
                    v.report("capture control cleared while capturing");
                }
                self.capture = None;
                self.r34.cleared = true;
            }
            CAP34_TRG_EN => self.start34(v),
            0 => {
                let sig = self.sig;
                if let Some(c) = &mut self.capture {
                    c.stop(&sig);
                }
            }
            other => v.report(format!(
                "capture control {other:#x}; the driver writes 0x74, 1 or 0"
            )),
        }
    }

    fn start34(&mut self, v: &mut Violations) {
        let r = &self.r34;
        if !r.configured {
            v.report("capture armed before the long registers were written");
        }
        if !r.cleared {
            v.report("capture armed without clearing capture control (0x74) first");
        }
        if r.mem_ctrl & 1 == 0 {
            v.report("capture armed with the memory in read mode");
        }
        if self.capture.is_some() {
            v.report("capture armed twice");
        }
        let boost = r.clk_boost == 1;
        let div = r.lregs[1];
        let rate = match self.design {
            Some(Design::Int) if boost => 125_000_000,
            // A host's divider beyond the slowest was flagged when written.
            Some(Design::Int) => (100_000_000 / div.saturating_add(1)).max(1),
            _ => self.sig.ext_clock,
        };
        let enable = r.lregs[4];
        let trigger = Trigger {
            mask: enable & Board::Lwla1034.all(),
            values: r.lregs[2],
            edges: r.lregs[3],
            trg_rise: enable & 1 << 35 != 0,
            trg_fall: enable & 1 << 34 != 0,
        };
        // Flagged when written if beyond the memory; the board has no more.
        let limit = r.lregs[L_MEM_FILL].min(u64::from(DEPTH)).saturating_sub(4) as usize;
        let mask = r.lregs[0];
        self.r34.cleared = false;
        self.capture = Some(Capture {
            rate,
            boost,
            mask,
            trigger,
            format: Format::Rle36,
            limit,
            ms: 0,
            searched: 0,
            at: None,
            end: None,
            full: false,
            words: Vec::new(),
        });
    }

    fn write_lregs(&mut self, w: &[u16], v: &mut Violations) {
        if w.len() < 3 {
            v.report(format!("long register write of {} words", w.len()));
            return;
        }
        let (first, count) = (usize::from(w[1]), usize::from(w[2]));
        if w.len() != 3 + 4 * count {
            v.report(format!(
                "long register write of {count} registers in {} words; it takes {}",
                w.len(),
                3 + 4 * count
            ));
            return;
        }
        if count == 0 || first + count > 10 {
            v.report(format!("long register write of {first} to {}", first + count));
            return;
        }
        let all = Board::Lwla1034.all();
        for i in 0..count {
            let q = &w[3 + 4 * i..7 + 4 * i];
            let value = u64::from(word32(&q[2..4])) << 32 | u64::from(word32(&q[0..2]));
            let addr = first + i;
            let bad = match addr {
                0 | 2 | 3 => value & !all != 0,
                1 => value > DIV_MAX || value != 0 && self.r34.clk_boost == 1,
                4 => value & !(all | 3 << 34) != 0,
                L_MEM_FILL => value <= 4 || value > u64::from(DEPTH),
                _ => value != 0,
            };
            if bad {
                v.report(format!("long register {addr} written {value:#x}"));
            }
            self.r34.lregs[addr] = value;
        }
        let r = &self.r34.lregs;
        if (r[2] | r[3]) & !r[4] != 0 {
            v.report(format!(
                "trigger levels {:#x} or types {:#x} outside the enable mask {:#x}",
                r[2], r[3], r[4]
            ));
        }
        self.r34.configured = first == 0 && count == 10;
    }

    fn read_lregs(&mut self, first: u16, count: u16, up: bool, v: &mut Violations) -> Vec<u8> {
        let (first, count) = (usize::from(first), usize::from(count));
        if count == 0 || first + count > 10 {
            v.report(format!("long register read of {first} to {}", first + count));
        }
        let mut out = Vec::new();
        if up
            && (first..first + count).contains(&L_STATUS)
            && let Some(c) = &mut self.capture
        {
            c.poll(&self.sig);
        }
        for i in first..(first + count).min(11) {
            put64(&mut out, if up && i <= 10 { self.lreg_read(i) } else { 0 });
        }
        out
    }

    // --- LWLA1016 ---

    fn read16(&mut self, reg: u16, v: &mut Violations) -> u32 {
        match reg {
            R16_TEST_ID => {
                self.test_reads += 1;
                if self.test_reads == 1 {
                    0
                } else {
                    self.faults.test_id.map_or(TEST_ID_1016, |t| t as u32)
                }
            }
            R16_CAP_CTRL => {
                let sig = self.sig;
                let Some(c) = &mut self.capture else { return 0 };
                c.poll(&sig);
                let mut s = 0;
                if c.running() {
                    s |= CAP16_TRG_EN;
                }
                if c.at.is_some() {
                    s |= CAP16_SAMPLE_EN;
                }
                if !c.full {
                    s |= CAP16_NOT_END;
                }
                s
            }
            R16_MEM_WR_PTR => {
                2 + self.capture.as_ref().map_or(0, |c| c.words.len() as u32)
            }
            R16_DURATION => self.capture.as_ref().map_or(0, |c| c.duration() as u32),
            R16_CAP_COUNT => {
                if self.capture.as_ref().is_some_and(Capture::running) {
                    v.report("capture count read while capturing");
                }
                // The address of the last word written; data from 2.
                1 + self.capture.as_ref().map_or(0, |c| c.words.len() as u32)
            }
            _ => {
                v.report(format!("read of register {reg:#06x}, which the driver does not read"));
                0
            }
        }
    }

    fn write16(&mut self, reg: u16, value: u32, v: &mut Violations) {
        let r = &mut self.r16;
        match reg {
            R16_CHAN_MASK => {
                if value > 0xffff {
                    v.report(format!("channel mask {value:#x} beyond 16 channels"));
                }
                r.chan_mask = value;
            }
            R16_DURATION => {
                if value != 0 {
                    v.report(format!("duration written {value}; the driver clears it with 0"));
                }
            }
            R16_MEM_CTRL => {
                if value > 2 {
                    v.report(format!("memory control {value:#x}; the driver writes 0, 1 or 2"));
                }
                r.mem_ctrl = value;
                if value == MEM16_WRITE {
                    r.reset_reads = false;
                }
                if value & MEM16_RESET != 0 {
                    r.read_ptr = None;
                    r.reset_reads = true;
                    r.second_read_due = false;
                } else if value == 0 && self.capture.as_ref().is_some_and(|c| !c.running()) {
                    // Read mode after a capture: the readout.
                    r.read_ptr = Some(Board::Lwla1016.read_start());
                }
            }
            R16_CAP_COUNT => {
                if value == 0 || value > DEPTH {
                    v.report(format!("capture count {value}, beyond the memory"));
                }
                r.cap_count = Some(value);
            }
            R16_TEST_ID => r.trg_sel = value,
            R16_DIV_COUNT => {
                if u64::from(value) > DIV_MAX {
                    v.report(format!("clock divider {value}, below 100 Hz"));
                }
                r.div = value;
            }
            R16_CAP_CTRL => self.capture_control16(value, v),
            R16_MEM_WR_PTR => v.report("write of the write pointer, which is read only"),
            _ => v.report(format!("write of register {reg:#06x}, which the driver does not write")),
        }
    }

    fn capture_control16(&mut self, value: u32, v: &mut Violations) {
        match value {
            0 => {
                let sig = self.sig;
                if let Some(c) = &mut self.capture
                    && c.running()
                {
                    c.stop(&sig);
                }
            }
            // The FIFO flags the driver clears before a capture.
            3 | 0x10 => {
                if self.capture.as_ref().is_some_and(Capture::running) {
                    v.report(format!("capture control {value:#x} while capturing"));
                }
                self.capture = None;
            }
            _ if value & 0xffff == CAP16_TRG_EN => self.start16(value >> 16, v),
            _ => v.report(format!(
                "capture control {value:#x}; the driver writes 0, 3, 0x10 or 4 with the trigger \
                 mask"
            )),
        }
    }

    fn start16(&mut self, mask: u32, v: &mut Violations) {
        let r = &self.r16;
        let Some(limit) = r.cap_count else {
            v.report("capture armed before the capture count was written");
            return;
        };
        if r.mem_ctrl & MEM16_WRITE == 0 {
            v.report("capture armed with the memory in read mode");
        }
        if self.capture.as_ref().is_some_and(Capture::running) {
            v.report("capture armed twice");
        }
        let (edges, values) = (r.trg_sel >> 16, r.trg_sel & 0xffff);
        if (edges | values) & !mask != 0 {
            v.report(format!(
                "trigger levels {values:#x} or types {edges:#x} outside the enable mask {mask:#x}"
            ));
        }
        let format = if self.design == Some(Design::Timing) { Format::Rle16 } else { Format::Plain16 };
        self.capture = Some(Capture {
            // A host's divider beyond the slowest was flagged when written.
            rate: (100_000_000 / (u64::from(r.div) + 1)).max(1),
            boost: false,
            mask: u64::from(r.chan_mask),
            trigger: Trigger {
                mask: u64::from(mask),
                values: u64::from(values),
                edges: u64::from(edges),
                trg_rise: false,
                trg_fall: false,
            },
            format,
            limit: limit.min(DEPTH) as usize,
            ms: 0,
            searched: 0,
            at: None,
            end: None,
            full: false,
            words: Vec::new(),
        });
        self.r16.cap_count = None;
    }
}

impl DeviceModel for Lwla {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.desc
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "control request {:#04x} {:#04x}: the board takes its commands on endpoint 2",
            setup.request_type, setup.request
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        match ep {
            EP_COMMAND => self.command(data, v),
            EP_CONFIG => self.bitstream(data, v),
            _ => {
                v.report(format!("OUT transfer to endpoint {ep}"));
                Reply::Stall
            }
        }
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP_REPLY {
            v.report(format!("IN transfer from endpoint {ep}"));
            return Reply::Stall;
        }
        if self.faults.halt_in {
            return Reply::Stall;
        }
        let Some(chunk) = self.pending.pop_front() else {
            // Nothing to send: the open's drain ends on its time out.
            return Reply::Nak;
        };
        if chunk.stall {
            return Reply::Stall;
        }
        // A longer read is the rounding to whole packets; an answer
        // longer than the host's own transfer the emulator reports.
        if max < chunk.data.len() {
            v.report(format!("reply of {} bytes read with {max}", chunk.total));
        }
        Reply::Data(chunk.data)
    }

    fn reset(&mut self) {
        // The FPGA keeps its design across a bus reset; what the reply
        // endpoint holds is what a crashed session left (`stale=N`).
        self.pending.clear();
        let stale: Vec<u8> = (0..self.stale).map(|i| (i % 251) as u8).collect();
        for c in stale.chunks(PACKET) {
            self.pending.push_back(Chunk { data: c.to_vec(), stall: false, total: c.len() });
        }
    }
}

#[cfg(test)]
mod tests;
