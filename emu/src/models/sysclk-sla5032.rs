// Counterpart in sigrok:
//   libsigrok/src/hardware/sysclk-sla5032/api.c (the ids, the open),
//   libsigrok/src/hardware/sysclk-sla5032/protocol.c (the commands, the registers, the bitstream's framing, the capture sequence and the RLE records),
//   libsigrok/src/hardware/sysclk-sla5032/protocol.h,
//   Copyright (C) 2014 Daniel Elstner <daniel.kitta@gmail.com>,
//   Copyright (C) 2019 Vitaliy Vorobyov,
//   with commits by Uwe Hermann, Vitaliy Vorobyov
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Sysclk SLA5032 (`2961:66b0`) as libsigrok's `sysclk-sla5032`
//! driver sees it: a Cypress FX2LP relaying commands to a Spartan-6 FPGA
//! that holds its bitstream until the power goes.
//!
//! **Sources.** The protocol is the C driver's (`protocol.c`,
//! `protocol.h`, `api.c`), the register list in `protocol.c`'s comment
//! first of all. The descriptors and strings are the 32CH 500M mode's
//! `lsusb -v` on the sigrok wiki (`Sysclk SLA5032/Info`), and the chips
//! its device page (CY7C68013A-56LTXI, XC6SLX16, 2x 1 Gbit DDR2). I looked
//! for firmware or HDL source (sigrok-firmware, GitHub, the wiki's links)
//! and found none: the vendor ships a closed `top.bit`. That means none
//! was found, not that none exists: the vendor ISO the wiki links
//! (`sla5032_2015_1_24.iso`, with the real `top.bit`) was not examined. So
//! nothing here was checked against the device's own logic. How sure: ids, strings,
//! endpoints and packet size are a recording and certain; command bytes,
//! register numbers and the record format are the driver's and certain;
//! what is marked "assumed" below is the model's reading, "invented" is
//! made-up content.
//!
//! **USB**: high speed, `bcdDevice` 0, manufacturer `www.SysCLK.com`,
//! product `SYSCLK SLA5032-v1.0`, no serial, configuration 1 (remote
//! wakeup, 500 mA), interface 0 of class 255/0/0, bulk endpoints 0x02 and
//! 0x04 OUT, 0x86 and 0x88 IN, 512 bytes. The driver uses 4 (commands), 8
//! (replies) and 6 (samples); 2 is in the descriptors only, and a transfer
//! to it is flagged. The wiki's two other modes (the board posing as a
//! Saleae Logic16) belong to another driver and are not modelled.
//!
//! **Commands** on endpoint 4, one per transfer:
//!
//! | bytes | command | reply on endpoint 8 |
//! |---|---|---|
//! | `01` | start the bitstream upload | none |
//! | `02 LEN DATA` | a bitstream chunk, up to 250 bytes | none |
//! | `03 ADDR 04` | read register `ADDR / 4` | the value, big endian |
//! | `04 ADDR 04 VALUE` | write register `ADDR / 4`, big endian | none |
//! | `05 HI LO LEN` | read the EEPROM | `LEN` bytes |
//! | `07 03` | prepare a sample chunk | none |
//!
//! **The bitstream**: the driver cuts the `.bit` file's header (`0x69`
//! bytes), checks the sync word `AA 99 55 66` that follows and sends 256
//! bytes of `FF` and the rest of the file. The stand-in is
//! `tools/emu-firmware.sh`'s with the sync word laid over it by
//! `tools/emu/sysclk-sla5032.sh`, since the plain one is refused;
//! `fw_size=N` tells the model of another size, for the case whose upload
//! ends on a whole chunk. The upload is checked byte for byte, and chunk
//! by chunk (250 bytes but the last). The FPGA keeps its design between
//! sessions, which `configured=1` starts from: then the driver sends no
//! bitstream.
//!
//! **Keys**: after the upload the driver reads 8 bytes at EEPROM address
//! 4000 and writes them as little endian words to registers 16 and 17.
//! The comment gives their value, `F6 81 13 64 00 00 00 00`. Register 2
//! then reads with bit 0 set. An FPGA without its keys takes commands but
//! never samples (assumed from "b0: 1 - keys entered"): that is what the
//! key is for, and it makes the driver wait for ever.
//!
//! **Registers** (comment of `protocol.c`). Written: 0 trigger levels or
//! slopes, 1 edge or level per channel, 2 trigger enable mask, 3 the
//! chunk's size (`0x300000`, assumed to be 16-bit words: that is the
//! driver's buffer of 6 MiB), 4 PLL divider less 1, 5 control bits, 6 and
//! 7 post- and pre-trigger depth, `(x + 1) * 256` samples, 9 to 12 the two
//! PWM generators, 14 sampling on or off, 16 and 17 the keys. Read: 0
//! post-trigger samples so far in units of 256 (half of register 6 while
//! triggered is invented; the driver only logs it), 1 the inputs, 2 the
//! status, 5 as written. Others are flagged. Register 5 reads 0 after the
//! bitstream (assumed; the driver only ever changes single bits and the
//! whole read-back value). Register 2 reads `B6FF9C97`, `12FF9C97`,
//! `92FF9C97`, `16FF9C97` in turn without a design (the comment's values:
//! "not configured"), `A5A5A5A0` with one and `A5A5A5A1` with the keys.
//! Bit 1 says triggered and bit 3 done.
//!
//! **A capture**: register 14 is set to 1, register 5's bits go through
//! the comment's "start sample" list, and bit 2's rise starts it, at
//! sample 0. The trigger is looked for once the pre-trigger depth is
//! full (assumed), so memory holds `pre` samples before the trigger and
//! `post` from it on, and without a trigger enabled the capture takes
//! the first `pre + post` samples. The depth counts samples whatever the
//! records they take (assumed): the memory is 2 x 1 Gbit, 256 MiB, about
//! 44.7 Mi records of 6 bytes, so 64 Mi samples that all change may not
//! fit on the real board. Reading them back: register 5 is
//! written `0x08`, then `0x28`; then per chunk `07 03`, register 3 and a
//! rise of register 5's bit 4 (the comment says "b5: ->0->1 upload next
//! data chunk", the code toggles bit 4 after bit 5 was set; the model
//! follows the code, which ran on the device).
//!
//! **Samples** come on endpoint 6 as 6 byte records, a little endian
//! 32-bit value and a little endian 16-bit count less 1, up to `0x100000`
//! in a chunk. A count of `FFFF` is the end marker (its value is 0,
//! assumed), so a run takes at most `FFFE` (65535 samples) and a longer
//! one is split (assumed encoder). The transfer ends short after the
//! marker. That the marker exists, and that a chunk is full unless it is
//! the last, is what `la_prepare_data()` reads. Records are made as the
//! host reads them: nothing is allocated from a depth the host wrote.
//!
//! **The signal** (invented): input `i` at sample `s` is bit `i` of `s /
//! period` (`period=N`, 1 by default), a binary counter, wrapping at 32
//! bits. A trigger is the AND of its conditions at one sample: a level
//! while the input has it, an edge where the input changes to its level
//! from the sample before. A search that passes [`SEARCH_MAX`] candidates
//! gives up, which means the trigger never fires: a real board would go
//! on looking, and the driver's wait looks the same.
//!
//! **Pacing**: a real board samples in wall time and the driver polls
//! every 100 ms. The model counts status reads (register 2) since the
//! start: `wait=N` (1 by default) read as waiting, one as triggered, then
//! done. A trigger that never fires, or a board without keys, reads as
//! waiting for ever.
//!
//! **Pedantic checks**: commands of the wrong length, unknown ids, a
//! command with a reply unread, a reply read with fewer bytes than it
//! has; the upload (see above), with nothing else in the middle of one
//! and none once configured; registers other than those named, reads of
//! write-only ones, registers used before the design is there, values
//! the driver does not write (register 3, register 14, the PWM constants,
//! a divider and multipliers not in `pll_tbl`, depths of 0, levels or edge
//! bits outside the mask, bits of register 5 above 8, the keys unlike the
//! EEPROM's); EEPROM reads other than the keys; the order (configuration
//! written while a capture runs, a start that skipped a step of the list,
//! a read-back before the capture is done or other than `0x08`, `0x28`,
//! register 5's bit 4 moved outside a read-back, a chunk armed without
//! its command and register 3, endpoint 6 read when no chunk is armed, a
//! chunk asked for after the end marker).
//!
//! **Accepted, as C habits**: `pre + post` over 64 Mi samples at the
//! limit with a ratio of 0 or 100, which the comment forbids
//! ("`(x+1)*256 + (y+1)*256 <= 64M`") but the driver's `MAX(.., 2)`
//! sends (the case "limit highest ratio 0"); a device has no way to
//! refuse a register write. The model keeps all of it, over 64 Mi samples
//! too (assumed). Register 5 written whole by the
//! read-back, losing the PLL bits; the sampling is over. Bit 6 is set
//! twice by the start list.
//!
//! **Faults**, all off by default: `stall=KIND[:N]` STALLs the command
//! (every one, or the Nth of its kind since the model was made; it has
//! no effect), `stall_reply=` STALLs its reply, `short_reply=` sends one
//! byte too few, `hold_reply=` never answers (the driver waits 500 s),
//! with `KIND` one of `init`, `chunk`, `read-reg`, `write-reg`,
//! `read-mem`, `read-data` (comma lists; the reply faults only for
//! `read-reg` and `read-mem`); `data_stall=N` STALLs the Nth chunk's read,
//! `data_hold=N` never sends it (the driver gives up after 2 s),
//! `data_short=BYTES` ends the first chunk after so many bytes with no
//! marker, `no_marker=1` ends the data without a marker, and `overcount=1`
//! sends 32769 records of 65535 samples and the marker whatever the depth
//! (memory gone wrong: about 197 KB that claim over 2^31 samples, for the
//! driver's `int`, and no larger total is offered), and `no_zlp=1` gives a
//! chunk that ends on a packet boundary, short of the host's transfer, no
//! zero-length packet: the transfer is held and holds those bytes when the
//! host's timeout cancels it. A one-shot
//! STALL is an emulation artifact: a real STALL halts the endpoint until
//! it is cleared, which the driver never does.
//!
//! **Emulation artifacts**, in one place: the time counted in status
//! reads; replies at once; the one-shot STALLs; the invented signal and
//! the marker's value; inputs read as 0 while waiting, so that a `--hangs`
//! log repeats; R0's half depth while triggered; a transfer that ends on a
//! packet boundary needs a zero-length packet on the bus, which the
//! emulator's completed transfer stands for. Whether the firmware sends one
//! is unknown, and it matters: without a short packet or a ZLP
//! `libusb_bulk_transfer()` runs to its 2 s timeout, and `protocol.c:749`
//! checks `ret` before `xfer_len`, so the chunk is dropped with exit 0.
//! With no ZLP that would hit about 1 capture in 256, those whose records
//! end on a packet boundary. The model ends the transfer unless `no_zlp=1`
//! says otherwise (the cases "fault chunk with no zlp"; "fault chunk never
//! sent" shows the path with no data).
//!
//! **Not reached by the cases** (`tests/emu/sysclk-sla5032/gcov.txt`):
//!
//! - A claim failing (`api.c:222-225`): another program's claim fails at
//!   `libusb_set_configuration()` first (the case "interface claimed by
//!   another program").
//! - `sr_usb_open()` failing (`api.c:210`): the device would have to go
//!   between the scan and the open, which no transfer marks.
//! - A partly taken OUT (`protocol.c:180-181`): a bulk OUT that does not
//!   finish ends in an error, which is checked first.
//! - sigrok-cli checks first: `api.c:262,273,278,311,316,328,349,473,475`,
//!   the "must not happen" defaults (`api.c:293-295,334-336,490-492`),
//!   channel indexes (`api.c:354-355,411-412`), re-enabling a channel
//!   (`api.c:363`), one trigger stage (`api.c:396-397`), a trigger on a
//!   disabled channel (`api.c:405`: "Invalid channel"), the trigger's kinds
//!   (`api.c:418-419`).
//! - A rate not listed (`protocol.c:347`): the driver's own `config_set()`
//!   refuses it first (`api.c:321-322`).
//! - `config_commit()`'s failures (`api.c:445,449-450`): the open has
//!   already configured the FPGA, so its apply does nothing.
//! - Cannot happen: allocation failures (`protocol.c:132-133,541-543,
//!   564-565,741-743,892-893`; 797-801 can, see below), libusb's list and descriptor calls
//!   (`api.c:121-123,174-178`), a file that shrinks while read
//!   (`protocol.c:550-552`), null arguments (`protocol.c:600`), the state
//!   check (`protocol.c:882-883`: `devc->state` is only ever `STATE_IDLE`),
//!   and the decoder's marker test (`protocol.c:814-815`: the count before
//!   it already stopped at the marker, `protocol.c:778-780`).
//! - `src/usb.c`: the event source and the port and string helpers, which
//!   this driver's blocking transfers and scan do not use, and libusb's
//!   own failures.
//!
//! **What the model shows about the driver** (C line numbers of
//! libsigrok 0bc24877; none of these crashes or fails the exit status):
//!
//! - `dev_inst_new_matching()` filters by `conn=` only when
//!   `sr_usb_find()` found something (`api.c:116`): a `conn` that matches
//!   no device selects every SLA5032.
//! - `sla5032_send_bitstream()` ignores `la_cfg_fpga_done()`'s result
//!   (`protocol.c:646`), so a failed key read or write still reports a
//!   good open, exit 0 (the cases "fault keys ignored", which assume
//!   nothing). That the capture then never ends is the model's assumption
//!   (the "fault keys" cases, `--hangs`): register 2's `A5A5A5A0` for ever
//!   is its choice, not the C's.
//! - A trigger that never fires has no limit at all: the driver polls for
//!   ever. A reply that never comes waits `USB_REPLY_TIMEOUT_MS`, 500 s
//!   (`protocol.h:46`).
//! - An error during a capture ends it with exit 0 (`protocol.c:719-723,
//!   733-737,749-756`): only the log tells, and the chunk is lost.
//! - The depths have a minimum of 512 samples each and are whole units of
//!   256 (`protocol.c:904-910`), and the driver sends all the memory holds.
//!   The rounding goes both ways: a limit of 512 yields 1024 samples, which
//!   sigrok-cli writes whole (it counts its limit for decoders and for
//!   "Device only sent", `session.c:406-423`, but gives the output the whole
//!   packet, `session.c:464`), and a limit of 5000 yields 4608, fewer than
//!   asked for. Disabled channels are sent as well (`channel_mask` is
//!   written at `api.c:363,365` and read nowhere).
//! - T goes on the first match in memory, whatever the depth; the distance
//!   from the hardware's trigger is unknown. `protocol.c:834` scans from the
//!   first sample read back and nothing uses register 7 or register 0, so
//!   the driver never had a position to lose. That an earlier match can be
//!   in memory at all is the model's assumption about where the hardware
//!   fires: the case "trigger rising" has T at offset 16 and the model's
//!   hardware position at offset 512 (`pre()` with register 7 at 1), and
//!   those two numbers say nothing about a real board.
//! - A failed start can leave register 14 at 1 and `devc->stl` allocated
//!   (`protocol.c:890,915-945`). Read only, not shown by a case: every
//!   early `return SR_ERR` of `la_write_cmd_buf()` leaks its buffer
//!   (`protocol.c:176,181`), and `devc->stl` is freed only on the good path
//!   (`protocol.c:861-864`).
//! - Records from the device are summed into an `int` with no check
//!   (`protocol.c:782`), which then sizes the buffer (`protocol.c:795`) that
//!   the decoder fills by the records' own counts (`protocol.c:818-821`).
//!   Read only, no case: 65537 records of 65535 samples sum to 2^32 - 1, and
//!   one more of 2 samples wraps it to 1, so the driver allocates 4 bytes
//!   and writes about 17 GB. A total from 2^31 up to 2^32 goes negative, and
//!   its huge `size_t` fails `g_try_malloc()`, the memory error at
//!   `protocol.c:797-801` (the case "fault samples over an int", exit 0).
//!   A capture of 64 Mi samples never comes near; it takes bad data.
//! - The comment's "b5: ->0->1 upload next data chunk" (`protocol.c:72`)
//!   does not match the code, which toggles bit 4 (`protocol.c:435,439`).

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{BULK, Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::ezusb::standin;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "sysclk-sla5032";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Sysclk SLA5032 (2961:66b0), FX2LP relaying commands to an FPGA with RLE \
                         sample memory; period=N, wait=N, configured=1, fw_size=N; faults: \
                         stall=, stall_reply=, short_reply=, hold_reply=KIND[:N], data_stall=N, \
                         data_hold=N, data_short=BYTES, no_marker=1, overcount=1, no_zlp=1";

/// The vendor id (`USB_VID_SYSCLK`).
pub const VID: u16 = 0x2961;
/// The product id (`USB_PID_SLA5032`).
pub const PID: u16 = 0x66b0;

const EP_UNUSED: u8 = 2;
const EP_COMMAND: u8 = 4;
const EP_DATA: u8 = 6;
const EP_REPLY: u8 = 8;
const PACKET: u16 = 512;

const CMD_INIT_FW_UPLOAD: u8 = 1;
const CMD_UPLOAD_FW_CHUNK: u8 = 2;
const CMD_READ_REG: u8 = 3;
const CMD_WRITE_REG: u8 = 4;
const CMD_READ_MEM: u8 = 5;
const CMD_READ_DATA: u8 = 7;

/// The bitstream file (`BITSTREAM_NAME`).
pub const BITSTREAM: &str = "sysclk-sla5032.bit";
/// The stand-in's size (`tools/emu/sysclk-sla5032.firmware`); the upload
/// leaves a last chunk of 151 bytes.
pub const BITSTREAM_SIZE: usize = 3000;
/// What the driver cuts off the file (`BITSTREAM_HEADER_SIZE`).
const BITSTREAM_HEADER: usize = 0x69;
/// The largest file the driver sends (`BITSTREAM_MAX_SIZE`).
const BITSTREAM_MAX: usize = 512 * 1024;
/// The `FF`s before the file's body.
const BITSTREAM_PAD: usize = 0x100;
/// The sync word's bytes (`XILINX_SYNC_WORD`).
const SYNC: [u8; 4] = [0xaa, 0x99, 0x55, 0x66];
/// The driver's chunk (`FW_CHUNK_SIZE`).
const CHUNK: usize = 250;

/// Where the keys are in the EEPROM (`la_cfg_fpga_done(usb, 4000)`).
const KEY_ADDRESS: u16 = 4000;
/// The keys as stored (the comment of `protocol.c`).
const KEY: [u8; 8] = [0xf6, 0x81, 0x13, 0x64, 0, 0, 0, 0];

/// Register 2 without a design: the comment's values.
const UNCONFIGURED: [u32; 4] = [0xb6ff_9c97, 0x12ff_9c97, 0x92ff_9c97, 0x16ff_9c97];
const CONFIGURED: u32 = 0xa5a5_a5a0;
const STATUS_KEYS: u32 = 1 << 0;
const STATUS_TRIGGERED: u32 = 1 << 1;
const STATUS_DONE: u32 = 1 << 3;

const R_LEVEL: usize = 0;
const R_EDGE: usize = 1;
const R_MASK: usize = 2;
const R_CHUNK: usize = 3;
const R_DIV: usize = 4;
const R_CTRL: usize = 5;
const R_POST: usize = 6;
const R_PRE: usize = 7;
const R_PWM1_HI: usize = 9;
const R_PWM1_LO: usize = 10;
const R_PWM2_HI: usize = 11;
const R_PWM2_LO: usize = 12;
const R_SAMPLE: usize = 14;
const R_KEY0: usize = 16;
const R_KEY1: usize = 17;
const REGS: usize = 18;
/// What `sla5032_start_acquisition()` writes before sampling.
const SETUP_REGS: [usize; 11] = [
    R_LEVEL, R_EDGE, R_MASK, R_DIV, R_POST, R_PRE, R_PWM1_HI, R_PWM1_LO, R_PWM2_HI, R_PWM2_LO,
    R_SAMPLE,
];

const CTRL_MUL2: u32 = 1 << 0;
const CTRL_PULSE1: u32 = 1 << 1;
const CTRL_START: u32 = 1 << 2;
const CTRL_B3: u32 = 1 << 3;
const CTRL_NEXT: u32 = 1 << 4;
const CTRL_B5: u32 = 1 << 5;
const CTRL_B6: u32 = 1 << 6;
const CTRL_NO_MUL125: u32 = 1 << 7;
const CTRL_PULSE8: u32 = 1 << 8;
const CTRL_ALL: u32 = 0x1ff;
const READ_BACK_1: u32 = 0x08;
const READ_BACK_2: u32 = 0x28;

/// The chunk's size as the driver writes it to register 3.
const CHUNK_WORDS: u32 = 0x30_0000;
/// One chunk in bytes (`RLE_BUF_SIZE`).
const CHUNK_BYTES: usize = 0x60_0000;
const END_MARKER: u16 = 0xffff;
/// The longest run of one record.
const RUN_MAX: u64 = 0xffff;
/// The records of the fault `overcount`: 32769 * 65535 is just over 2^31.
const OVERCOUNT_RECORDS: u64 = 32769;

/// The PWM constants of `sla5032_start_acquisition()`.
const PWM: [(usize, u32); 4] = [
    (R_PWM1_HI, 20_000_000 - 1),
    (R_PWM1_LO, 200_000 - 1),
    (R_PWM2_HI, 15 - 1),
    (R_PWM2_LO, 5 - 1),
];

/// `pll_tbl`: the divider less 1, the doubler, the 1.25 multiplier.
const PLL: [(u32, bool, bool); 20] = [
    (0, true, true),
    (0, true, false),
    (0, false, true),
    (0, false, false),
    (1, false, false),
    (3, false, false),
    (7, false, false),
    (9, false, false),
    (19, false, false),
    (39, false, false),
    (99, false, false),
    (199, false, false),
    (399, false, false),
    (999, false, false),
    (1999, false, false),
    (3999, false, false),
    (9999, false, false),
    (19999, false, false),
    (39999, false, false),
    (99999, false, false),
];

/// Candidates a trigger search looks at before it gives up.
pub const SEARCH_MAX: u32 = 1 << 24;

/// What a fault aims at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Init,
    Chunk,
    ReadReg,
    WriteReg,
    ReadMem,
    ReadData,
}

const KINDS: [(&str, Kind); 6] = [
    ("init", Kind::Init),
    ("chunk", Kind::Chunk),
    ("read-reg", Kind::ReadReg),
    ("write-reg", Kind::WriteReg),
    ("read-mem", Kind::ReadMem),
    ("read-data", Kind::ReadData),
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
                    .ok_or_else(|| Error(format!("{k}={item}: N from 1")))?;
                (name, Some(n))
            }
            None => (item, None),
        };
        let kind = KINDS.iter().find(|(n, _)| *n == name).map(|(_, c)| *c).ok_or_else(|| {
            let names: Vec<&str> = KINDS.iter().map(|(n, _)| *n).collect();
            Error(format!("{k}={name}: one of {}", names.join(", ")))
        })?;
        if k != "stall" && !matches!(kind, Kind::ReadReg | Kind::ReadMem) {
            return Err(Error(format!("{k}={name}: only read-reg and read-mem reply")));
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
    parsed.ok_or_else(|| Error(format!("{k}={v}: a number")))
}

/// The fault options.
#[derive(Clone, Debug, Default)]
struct Faults {
    stall: Vec<Shot>,
    stall_reply: Vec<Shot>,
    short_reply: Vec<Shot>,
    hold_reply: Vec<Shot>,
    data_stall: Option<u32>,
    data_hold: Option<u32>,
    data_short: Option<usize>,
    no_marker: bool,
    overcount: bool,
    no_zlp: bool,
}

/// The options that make a board.
#[derive(Clone, Debug)]
struct Options {
    period: u64,
    wait: u32,
    configured: bool,
    fw_size: usize,
    faults: Faults,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            period: 1,
            wait: 1,
            configured: false,
            fw_size: BITSTREAM_SIZE,
            faults: Faults::default(),
        }
    }
}

/// The trigger as registers 0 to 2 set it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Trigger {
    /// Register 0: the level, or the level an edge goes to.
    pub level: u32,
    /// Register 1: which conditions are edges.
    pub edge: u32,
    /// Register 2: which channels take part.
    pub mask: u32,
}

impl Trigger {
    /// Whether the conditions hold at sample `s`.
    #[must_use]
    pub fn holds(&self, period: u64, s: u64) -> bool {
        let now = input(period, s);
        if (now ^ self.level) & self.mask != 0 {
            return false;
        }
        let edges = self.edge & self.mask;
        if edges == 0 {
            return true;
        }
        // Every edge must have just happened: no edge at sample 0.
        s > 0 && (now ^ input(period, s - 1)) & edges == edges
    }

    /// The first sample from `from` on where it fires.
    #[must_use]
    pub fn search(&self, period: u64, from: u64) -> Option<u64> {
        if self.mask == 0 {
            return Some(from);
        }
        // Inputs of the mask change only at multiples of this.
        let step = period.checked_shl(self.mask.trailing_zeros()).unwrap_or(u64::MAX);
        let mut s = from;
        for _ in 0..SEARCH_MAX {
            if self.holds(period, s) {
                return Some(s);
            }
            s = s.checked_add(step - s % step)?;
        }
        None
    }
}

/// Input `i` at sample `s` is bit `i` of this.
#[must_use]
pub fn input(period: u64, s: u64) -> u32 {
    (s / period) as u32
}

/// Where the board's sampling is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// No capture.
    Idle,
    /// Sampling; `fired` is where the trigger does, if it ever does.
    Running { reads: u32, fired: Option<u64> },
    /// Done: the memory holds its samples.
    Done,
    /// Register 5 was written `0x08`.
    ReadBack1,
    /// Register 5 was written `0x28`: chunks can be asked for.
    ReadBack,
}

/// A reply waiting on endpoint 8.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Pending {
    Data(Vec<u8>),
    Stall,
    Hold,
}

/// The sample memory's readout.
#[derive(Clone, Debug, Default)]
struct Readout {
    /// The next sample to encode, and the one after the last.
    next: u64,
    end: u64,
    /// Encoded bytes a transfer had no room for.
    carry: Vec<u8>,
    marker_sent: bool,
    /// The chunk's command came, and register 3.
    command: bool,
    size: bool,
    /// Bytes the chunk armed has left.
    armed: Option<usize>,
    /// Chunks armed so far.
    chunks: u32,
    /// Bytes the fault `data_short` leaves in this chunk.
    short_left: Option<usize>,
    /// The fault `no_zlp`: what a held transfer has received.
    held: Option<Vec<u8>>,
}

/// The model.
pub struct Sla5032 {
    desc: Descriptors,
    options: Options,
    /// What the driver must upload.
    image: Vec<u8>,
    /// Bytes of it taken, while an upload runs.
    upload: Option<usize>,
    configured: bool,
    keys: [bool; 2],
    /// Reads of register 2 without a design.
    blank_reads: usize,
    regs: [u32; REGS],
    written: [bool; REGS],
    /// Register 5's bits 1 and 8 went up and down since sampling was set.
    pulse1: bool,
    pulse8: bool,
    phase: Phase,
    /// The samples in memory: first and one after the last.
    window: (u64, u64),
    pending: Option<Pending>,
    readout: Readout,
    /// Commands of each kind so far.
    counts: [u32; KINDS.len()],
}

/// The descriptors, as the wiki's `lsusb -v` of the 32CH 500M mode has them.
#[must_use]
pub fn descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 4, [0xff, 0, 0], 0));
    for address in [EP_UNUSED, EP_COMMAND, 0x80 | EP_DATA, 0x80 | EP_REPLY] {
        body.extend_from_slice(&descriptor::endpoint(address, BULK, PACKET, 0));
    }
    let config = descriptor::configuration(1, 0, 0xa0, 500, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: VID,
        product: PID,
        device: 0x0000,
        strings: [1, 2, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: vec!["www.SysCLK.com".to_owned(), "SYSCLK SLA5032-v1.0".to_owned()],
    }
}

/// The bytes the driver sends for a stand-in of `size` bytes: `FF`s, then
/// the file from its sync word on, the stand-in with that word laid over.
#[must_use]
pub fn upload_image(size: usize) -> Vec<u8> {
    let mut file = standin(BITSTREAM, size);
    file[BITSTREAM_HEADER..BITSTREAM_HEADER + SYNC.len()].copy_from_slice(&SYNC);
    let mut image = vec![0xff; BITSTREAM_PAD];
    image.extend_from_slice(&file[BITSTREAM_HEADER..]);
    image
}

/// A board from the `-o` options (see the module's documentation).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut o = Options::default();
    for (k, v) in options {
        let flag = |v: &str| match v {
            "0" => Ok(false),
            "1" => Ok(true),
            _ => Err(Error(format!("{k}={v}: 0 or 1"))),
        };
        let nth = |v: &str| {
            number(k, v)
                .ok()
                .and_then(|n| u32::try_from(n).ok())
                .filter(|n| *n > 0)
                .ok_or_else(|| Error(format!("{k}={v}: from 1")))
        };
        match k.as_str() {
            "period" => {
                o.period = number(k, v)?;
                if o.period == 0 {
                    return Err(Error(format!("period={v}: from 1")));
                }
            }
            "wait" => {
                o.wait = u32::try_from(number(k, v)?)
                    .map_err(|_| Error(format!("wait={v}: a 32-bit count")))?;
            }
            "configured" => o.configured = flag(v)?,
            "fw_size" => {
                o.fw_size = usize::try_from(number(k, v)?)
                    .ok()
                    .filter(|n| (BITSTREAM_HEADER + SYNC.len()..=BITSTREAM_MAX).contains(n))
                    .ok_or_else(|| {
                        Error(format!("fw_size={v}: what the driver sends, up to 512 KiB"))
                    })?;
            }
            "stall" => o.faults.stall = shots(k, v)?,
            "stall_reply" => o.faults.stall_reply = shots(k, v)?,
            "short_reply" => o.faults.short_reply = shots(k, v)?,
            "hold_reply" => o.faults.hold_reply = shots(k, v)?,
            "data_stall" => o.faults.data_stall = Some(nth(v)?),
            "data_hold" => o.faults.data_hold = Some(nth(v)?),
            "data_short" => {
                o.faults.data_short = Some(
                    usize::try_from(number(k, v)?)
                        .ok()
                        .filter(|n| *n <= CHUNK_BYTES)
                        .ok_or_else(|| Error(format!("data_short={v}: up to a chunk")))?,
                );
            }
            "no_marker" => o.faults.no_marker = flag(v)?,
            "no_zlp" => o.faults.no_zlp = flag(v)?,
            "overcount" => o.faults.overcount = flag(v)?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(Sla5032::new(o)))
}

impl Sla5032 {
    fn new(options: Options) -> Sla5032 {
        let image = upload_image(options.fw_size);
        let configured = options.configured;
        Sla5032 {
            desc: descriptors(),
            options,
            image,
            upload: None,
            configured,
            keys: [configured; 2],
            blank_reads: 0,
            regs: [0; REGS],
            written: [false; REGS],
            pulse1: false,
            pulse8: false,
            phase: Phase::Idle,
            window: (0, 0),
            pending: None,
            readout: Readout::default(),
            counts: [0; KINDS.len()],
        }
    }

    fn keyed(&self) -> bool {
        self.keys[0] && self.keys[1]
    }

    fn count(&mut self, kind: Kind) -> u32 {
        let i = KINDS.iter().position(|(_, k)| *k == kind).unwrap_or(0);
        self.counts[i] += 1;
        self.counts[i]
    }

    /// The reply to a read command, with the faults aimed at it.
    fn reply(&mut self, kind: Kind, nth: u32, mut data: Vec<u8>) {
        self.pending = Some(if hit(&self.options.faults.stall_reply, kind, nth) {
            Pending::Stall
        } else if hit(&self.options.faults.hold_reply, kind, nth) {
            Pending::Hold
        } else {
            if hit(&self.options.faults.short_reply, kind, nth) {
                data.pop();
            }
            Pending::Data(data)
        });
    }

    fn command(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let Some(&id) = data.first() else {
            v.report("an empty command");
            return Reply::Stall;
        };
        let (kind, length) = match id {
            CMD_INIT_FW_UPLOAD => (Kind::Init, 1),
            CMD_UPLOAD_FW_CHUNK => (Kind::Chunk, 2 + data.get(1).map_or(0, |n| usize::from(*n))),
            CMD_READ_REG => (Kind::ReadReg, 3),
            CMD_WRITE_REG => (Kind::WriteReg, 7),
            CMD_READ_MEM => (Kind::ReadMem, 4),
            CMD_READ_DATA => (Kind::ReadData, 2),
            _ => {
                v.report(format!("unknown command {id:#04x}"));
                return Reply::Stall;
            }
        };
        if data.len() != length {
            v.report(format!("command {id} of {} bytes, not {length}", data.len()));
            return Reply::Stall;
        }
        let nth = self.count(kind);
        if hit(&self.options.faults.stall, kind, nth) {
            return Reply::Stall;
        }
        if self.pending.is_some() {
            v.report(format!("command {id} with a reply unread"));
            self.pending = None;
        }
        // A new start, or the status read that begins the next open, ends
        // an upload the driver gave up on when a chunk failed.
        if kind == Kind::Init || (kind == Kind::ReadReg && usize::from(data[1]) == R_MASK * 4) {
            self.upload = None;
        }
        if self.upload.is_some() && kind != Kind::Chunk {
            v.report(format!("command {id} in the middle of the upload"));
        }
        match kind {
            Kind::Init => self.init_upload(v),
            Kind::Chunk => self.upload_chunk(&data[2..], v),
            Kind::ReadReg => self.read_reg(data, nth, v),
            Kind::WriteReg => self.write_reg(data, v),
            Kind::ReadMem => self.read_mem(data, nth, v),
            Kind::ReadData => self.read_data(data[1], v),
        }
        Reply::ack()
    }

    fn init_upload(&mut self, v: &mut Violations) {
        // Without its keys register 2 says "not configured", so the driver
        // uploads again, as it may.
        if self.configured && self.keyed() {
            v.report("a bitstream upload to a configured FPGA");
        }
        self.configured = false;
        self.keys = [false; 2];
        self.upload = Some(0);
    }

    fn upload_chunk(&mut self, chunk: &[u8], v: &mut Violations) {
        let Some(at) = self.upload else {
            v.report("a bitstream chunk without the upload's start");
            return;
        };
        let left = self.image.len() - at;
        if chunk.is_empty() || (chunk.len() != CHUNK && chunk.len() != left) {
            v.report(format!(
                "a bitstream chunk of {} bytes with {left} to go, not {CHUNK}",
                chunk.len()
            ));
        }
        let end = at + chunk.len().min(left);
        if chunk.len() > left || self.image[at..end] != chunk[..end - at] {
            v.report(format!("bitstream differs at offset {at} or after, or is too long"));
        }
        if end == self.image.len() {
            self.upload = None;
            self.configured = true;
        } else {
            self.upload = Some(end);
        }
    }

    fn read_mem(&mut self, data: &[u8], nth: u32, v: &mut Violations) {
        let address = u16::from_be_bytes([data[1], data[2]]);
        let length = usize::from(data[3]);
        if address != KEY_ADDRESS || length != KEY.len() {
            v.report(format!("EEPROM read of {length} bytes at {address}: only the keys are read"));
        }
        if !self.configured {
            v.report("EEPROM read without a design");
        }
        // Outside the keys: blank (invented), and flagged above.
        let mut bytes = vec![0xff; length];
        if address == KEY_ADDRESS {
            let n = length.min(KEY.len());
            bytes[..n].copy_from_slice(&KEY[..n]);
        }
        self.reply(Kind::ReadMem, nth, bytes);
    }

    /// The register of a command's address.
    fn register(address: u8, v: &mut Violations) -> Option<usize> {
        if !address.is_multiple_of(4) || usize::from(address) / 4 >= REGS {
            v.report(format!("register address {address:#04x}"));
            return None;
        }
        Some(usize::from(address) / 4)
    }

    fn read_reg(&mut self, data: &[u8], nth: u32, v: &mut Violations) {
        if data[2] != 4 {
            v.report(format!("register read of {} bytes", data[2]));
        }
        let value = Self::register(data[1], v).map_or(0, |reg| self.reg_value(reg, v));
        self.reply(Kind::ReadReg, nth, value.to_be_bytes().to_vec());
    }

    fn reg_value(&mut self, reg: usize, v: &mut Violations) -> u32 {
        if reg == R_MASK {
            return self.status();
        }
        if !self.configured {
            v.report(format!("register {reg} read without a design"));
            return 0;
        }
        match reg {
            0 => match self.phase {
                Phase::Running { reads, fired: Some(_) }
                    if self.keyed() && reads > self.options.wait =>
                {
                    self.regs[R_POST] / 2
                }
                Phase::Running { .. } | Phase::Idle => 0,
                _ => self.regs[R_POST] + 1,
            },
            1 => match self.phase {
                Phase::Running { .. } | Phase::Idle => 0,
                _ => input(self.options.period, self.window.1.saturating_sub(1)),
            },
            R_CTRL => self.regs[R_CTRL],
            _ => {
                v.report(format!("register {reg} is write-only"));
                0
            }
        }
    }

    /// Register 2's value: a status read, which is what moves time.
    fn status(&mut self) -> u32 {
        if !self.configured {
            let value = UNCONFIGURED[self.blank_reads % UNCONFIGURED.len()];
            self.blank_reads += 1;
            return value;
        }
        let keys = if self.keyed() { STATUS_KEYS } else { 0 };
        let flags = match self.phase {
            Phase::Idle => 0,
            Phase::Running { reads, fired } => {
                let reads = reads.saturating_add(1);
                self.phase = Phase::Running { reads, fired };
                match fired {
                    Some(at) if self.keyed() && reads > self.options.wait => {
                        if reads > self.options.wait.saturating_add(1) {
                            self.finish(at);
                            STATUS_TRIGGERED | STATUS_DONE
                        } else {
                            STATUS_TRIGGERED
                        }
                    }
                    _ => 0,
                }
            }
            _ => STATUS_TRIGGERED | STATUS_DONE,
        };
        CONFIGURED | keys | flags
    }

    fn pre(&self) -> u64 {
        (u64::from(self.regs[R_PRE]) + 1) * 256
    }

    fn post(&self) -> u64 {
        (u64::from(self.regs[R_POST]) + 1) * 256
    }

    /// The capture is complete: the memory holds its window.
    fn finish(&mut self, at: u64) {
        self.window = (at - self.pre(), at + self.post());
        self.phase = Phase::Done;
    }

    fn write_reg(&mut self, data: &[u8], v: &mut Violations) {
        if data[2] != 4 {
            v.report(format!("register write of {} bytes", data[2]));
        }
        let Some(reg) = Self::register(data[1], v) else {
            return;
        };
        let value = u32::from_be_bytes([data[3], data[4], data[5], data[6]]);
        if !self.configured {
            v.report(format!("register {reg} written without a design"));
            return;
        }
        match reg {
            R_LEVEL | R_EDGE | R_DIV => self.write_setup(reg, value, v),
            R_MASK => {
                let extra = (self.regs[R_LEVEL] | self.regs[R_EDGE]) & !value;
                if extra != 0 {
                    v.report(format!("trigger bits {extra:#x} outside the mask {value:#x}"));
                }
                self.write_setup(reg, value, v);
            }
            R_POST | R_PRE => {
                if value == 0 {
                    v.report(format!("register {reg}: the depth's minimum is 1"));
                }
                self.write_setup(reg, value, v);
            }
            R_PWM1_HI | R_PWM1_LO | R_PWM2_HI | R_PWM2_LO => {
                if PWM.iter().all(|(r, want)| *r != reg || *want != value) {
                    v.report(format!("PWM register {reg} written {value}"));
                }
                self.write_setup(reg, value, v);
            }
            R_CHUNK => {
                if value != CHUNK_WORDS {
                    v.report(format!("register 3 written {value:#x}, not {CHUNK_WORDS:#x}"));
                }
                if self.phase != Phase::ReadBack {
                    v.report("register 3 written outside a read-back");
                }
                self.readout.size = true;
            }
            R_CTRL => self.write_ctrl(value, v),
            R_SAMPLE => self.write_sample(value, v),
            R_KEY0 | R_KEY1 => self.write_key(reg, value, v),
            _ => v.report(format!("register {reg} written {value:#x}")),
        }
    }

    /// A register of the capture's configuration.
    fn write_setup(&mut self, reg: usize, value: u32, v: &mut Violations) {
        if self.phase != Phase::Idle {
            v.report(format!("register {reg} written during a capture"));
        }
        self.regs[reg] = value;
        self.written[reg] = true;
    }

    fn write_key(&mut self, reg: usize, value: u32, v: &mut Violations) {
        let at = (reg - R_KEY0) * 4;
        let want = u32::from_le_bytes([KEY[at], KEY[at + 1], KEY[at + 2], KEY[at + 3]]);
        if value == want {
            self.keys[reg - R_KEY0] = true;
        } else {
            v.report(format!("key register {reg} written {value:#x}, not {want:#x}"));
        }
    }

    fn write_sample(&mut self, value: u32, v: &mut Violations) {
        match value {
            0 => {
                // The end of an upload, of a capture, or of a failed one.
                self.phase = Phase::Idle;
                self.written[R_SAMPLE] = false;
                self.regs[R_SAMPLE] = 0;
                self.readout = Readout::default();
            }
            1 => {
                if self.phase != Phase::Idle {
                    v.report("sampling set during a capture");
                }
                self.regs[R_SAMPLE] = 1;
                self.written[R_SAMPLE] = true;
                self.pulse1 = false;
                self.pulse8 = false;
            }
            _ => v.report(format!("register 14 written {value}")),
        }
    }

    fn write_ctrl(&mut self, value: u32, v: &mut Violations) {
        if value & !CTRL_ALL != 0 {
            v.report(format!("register 5 written {value:#x}, bits above 8"));
        }
        let old = self.regs[R_CTRL];
        self.regs[R_CTRL] = value;
        let fell = old & !value;
        match self.phase {
            Phase::Idle => {
                if (old ^ value) & CTRL_NEXT != 0 {
                    v.report("register 5's bit 4 moved outside a read-back");
                }
                self.pulse1 |= fell & CTRL_PULSE1 != 0;
                self.pulse8 |= fell & CTRL_PULSE8 != 0;
                if !old & value & CTRL_START != 0 {
                    self.start(v);
                }
            }
            Phase::Running { .. } => {
                v.report(format!("register 5 written {value:#x} during a capture"));
            }
            Phase::Done => {
                if value == READ_BACK_1 {
                    self.phase = Phase::ReadBack1;
                } else {
                    v.report(format!("read-back's register 5 is {value:#x}, not 0x08"));
                }
            }
            Phase::ReadBack1 => {
                if value == READ_BACK_2 {
                    self.phase = Phase::ReadBack;
                    self.readout = Readout {
                        next: self.window.0,
                        end: self.window.1,
                        short_left: self.options.faults.data_short,
                        ..Readout::default()
                    };
                } else {
                    v.report(format!("read-back's register 5 is {value:#x}, not 0x28"));
                }
            }
            Phase::ReadBack => self.write_ctrl_read_back(old, value, v),
        }
    }

    fn write_ctrl_read_back(&mut self, old: u32, value: u32, v: &mut Violations) {
        if (old ^ value) & !CTRL_NEXT != 0 {
            v.report(format!("register 5 written {value:#x} in a read-back, from {old:#x}"));
        }
        if !old & value & CTRL_NEXT != 0 {
            let r = &mut self.readout;
            if !r.command || !r.size {
                v.report("a chunk armed without its command and register 3");
            }
            if r.marker_sent {
                v.report("a chunk asked for after the end marker");
            }
            r.command = false;
            r.size = false;
            r.chunks += 1;
            r.armed = Some(CHUNK_BYTES);
        }
    }

    /// Bit 2's rise: the "start sample" list must have run.
    fn start(&mut self, v: &mut Violations) {
        for reg in SETUP_REGS {
            if !self.written[reg] {
                v.report(format!("a capture started without register {reg}"));
            }
        }
        let ctrl = self.regs[R_CTRL];
        if ctrl & (CTRL_B3 | CTRL_B5) != 0 || ctrl & CTRL_B6 == 0 {
            v.report(format!("a capture started with register 5 at {ctrl:#x}"));
        }
        if !self.pulse1 || !self.pulse8 {
            v.report("a capture started without the pulses of register 5's bits 1 and 8");
        }
        let pll = (self.regs[R_DIV], ctrl & CTRL_MUL2 != 0, ctrl & CTRL_NO_MUL125 == 0);
        if !PLL.contains(&pll) {
            v.report(format!("a sample rate not in pll_tbl: {pll:?}"));
        }
        let trigger = Trigger {
            level: self.regs[R_LEVEL],
            edge: self.regs[R_EDGE],
            mask: self.regs[R_MASK],
        };
        let fired = trigger.search(self.options.period, self.pre());
        self.phase = Phase::Running { reads: 0, fired };
    }

    fn read_data(&mut self, address: u8, v: &mut Violations) {
        if address != 3 {
            v.report(format!("chunk command with {address}, not 3"));
        }
        if self.phase != Phase::ReadBack {
            v.report("chunk command outside a read-back");
        }
        self.readout.command = true;
    }

    /// One more record, or the marker, into the carry. False when there
    /// is nothing more to send.
    fn encode(&mut self) -> bool {
        let period = self.options.period;
        let no_marker = self.options.faults.no_marker;
        let overcount = self.options.faults.overcount;
        let r = &mut self.readout;
        if overcount {
            // `next` counts records here, from the window's start of 0.
            if r.next < OVERCOUNT_RECORDS {
                r.carry.extend_from_slice(&0u32.to_le_bytes());
                r.carry.extend_from_slice(&((RUN_MAX - 1) as u16).to_le_bytes());
                r.next += 1;
                return true;
            }
            r.next = r.end.max(r.next);
        }
        if r.next < r.end {
            let value = input(period, r.next);
            let run = (period - r.next % period).min(r.end - r.next).min(RUN_MAX);
            r.carry.extend_from_slice(&value.to_le_bytes());
            r.carry.extend_from_slice(&((run - 1) as u16).to_le_bytes());
            r.next += run;
            return true;
        }
        if r.marker_sent || no_marker {
            return false;
        }
        r.carry.extend_from_slice(&0u32.to_le_bytes());
        r.carry.extend_from_slice(&END_MARKER.to_le_bytes());
        r.marker_sent = true;
        true
    }

    fn data(&mut self, max: usize, v: &mut Violations) -> Reply {
        if self.readout.held.is_some() {
            return Reply::Nak;
        }
        let Some(left) = self.readout.armed else {
            v.report("endpoint 6 read with no chunk armed");
            return Reply::Nak;
        };
        let chunk = self.readout.chunks;
        if self.options.faults.data_hold == Some(chunk) {
            return Reply::Nak;
        }
        if self.options.faults.data_stall == Some(chunk) {
            self.readout.armed = None;
            return Reply::Stall;
        }
        let mut want = max.min(left);
        if let Some(short) = self.readout.short_left {
            want = want.min(short);
        }
        // `want` is bounded by a chunk, whatever the host asks for.
        let mut out = Vec::with_capacity(want);
        loop {
            let take = (want - out.len()).min(self.readout.carry.len());
            out.extend(self.readout.carry.drain(..take));
            if out.len() == want || !self.encode() {
                break;
            }
        }
        let left = left - out.len();
        if let Some(short) = self.readout.short_left.as_mut() {
            *short -= out.len();
            if *short == 0 {
                // The fault's end of the data: nothing more follows.
                self.readout.armed = None;
                self.readout.next = self.readout.end;
                self.readout.carry.clear();
                self.readout.marker_sent = true;
                return Reply::Data(out);
            }
        }
        // A transfer that ended before the host's and the chunk's end
        // ends the chunk too; one the host cut short goes on in the next.
        self.readout.armed = if left == 0 || out.len() < want { None } else { Some(left) };
        if self.options.faults.no_zlp
            && out.len() < max
            && !out.is_empty()
            && out.len().is_multiple_of(usize::from(PACKET))
        {
            self.readout.held = Some(out);
            return Reply::Nak;
        }
        Reply::Data(out)
    }
}

impl DeviceModel for Sla5032 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.desc
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "control request {:#04x} {:#04x}: the board takes its commands on endpoint 4",
            setup.request_type, setup.request
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep == EP_COMMAND {
            return self.command(data, v);
        }
        v.report(format!("OUT transfer to endpoint {ep}"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        match ep {
            EP_DATA => self.data(max, v),
            EP_REPLY => match self.pending.take() {
                Some(Pending::Data(data)) => {
                    // A longer read is the rounding to whole packets; an
                    // answer longer than the host's own transfer the
                    // emulator reports.
                    if max < data.len() {
                        v.report(format!("reply of {} bytes read with {max}", data.len()));
                    }
                    Reply::Data(data)
                }
                Some(Pending::Stall) => Reply::Stall,
                Some(Pending::Hold) => {
                    self.pending = Some(Pending::Hold);
                    Reply::Nak
                }
                None => {
                    v.report("endpoint 8 read with no reply due");
                    Reply::Nak
                }
            },
            _ => {
                v.report(format!("IN transfer from endpoint {ep}"));
                Reply::Stall
            }
        }
    }

    /// A transfer `no_zlp` holds completes with its bytes at the cancel.
    fn in_unlinked(&mut self, ep: u8, _max: usize) -> Option<Vec<u8>> {
        if ep == EP_DATA { self.readout.held.take() } else { None }
    }

    fn reset(&mut self) {
        // The FPGA keeps its design and keys across a bus reset, and a
        // capture its memory; a reply not yet read is gone.
        self.pending = None;
    }

    /// The bitstream's chunks and the sample chunks.
    fn log_digest_above(&self) -> Option<usize> {
        Some(96)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    const RECORD: usize = 6;
    const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

    fn opts(o: &[(&str, &str)]) -> Vec<(String, String)> {
        o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    /// The host side: the driver's commands and reads.
    struct Host {
        dev: Device,
        v: Violations,
    }

    impl Host {
        fn new(o: &[(&str, &str)]) -> Host {
            let mut dev = Device::new(build(&opts(o)).unwrap());
            let mut v = Violations::default();
            dev.control(SET_CONFIG, &[], &mut v);
            Host { dev, v }
        }

        fn configured(o: &[(&str, &str)]) -> Host {
            let mut all = vec![("configured", "1")];
            all.extend_from_slice(o);
            Host::new(&all)
        }

        fn violations(&mut self) -> Vec<String> {
            self.v.drain()
        }

        fn clean(&mut self) {
            let v = self.violations();
            assert!(v.is_empty(), "violations: {v:?}");
        }

        fn flagged(&mut self, what: &str) {
            let v = self.violations();
            assert!(v.iter().any(|m| m.contains(what)), "wanted {what:?} in {v:?}");
        }

        fn send(&mut self, data: &[u8]) -> Reply {
            self.dev.transfer_out(EP_COMMAND, data, &mut self.v)
        }

        fn reply(&mut self, max: usize) -> Reply {
            self.dev.transfer_in(EP_REPLY, max, &mut self.v)
        }

        fn read(&mut self, reg: u8) -> u32 {
            assert_eq!(self.send(&[CMD_READ_REG, reg * 4, 4]), Reply::ack());
            match self.reply(512) {
                Reply::Data(d) => u32::from_be_bytes(d.try_into().unwrap()),
                other => panic!("register {reg}: {other:?}"),
            }
        }

        fn write(&mut self, reg: u8, value: u32) -> Reply {
            let b = value.to_be_bytes();
            self.send(&[CMD_WRITE_REG, reg * 4, 4, b[0], b[1], b[2], b[3]])
        }

        fn bit(&mut self, reg: u8, bit: u32, on: bool) {
            let old = self.read(reg);
            let new = if on { old | 1 << bit } else { old & !(1 << bit) };
            self.write(reg, new);
        }

        fn send_image(&mut self) {
            self.send(&[CMD_INIT_FW_UPLOAD]);
            for c in upload_image(BITSTREAM_SIZE).chunks(CHUNK) {
                let mut cmd = vec![CMD_UPLOAD_FW_CHUNK, c.len() as u8];
                cmd.extend_from_slice(c);
                self.send(&cmd);
            }
        }

        /// `sla5032_send_bitstream()`.
        fn upload(&mut self) {
            self.read(2);
            self.send_image();
            self.send(&[CMD_READ_MEM, (KEY_ADDRESS >> 8) as u8, KEY_ADDRESS as u8, 8]);
            let Reply::Data(key) = self.reply(512) else { panic!("no key") };
            self.write(16, u32::from_le_bytes(key[..4].try_into().unwrap()));
            self.write(17, u32::from_le_bytes(key[4..].try_into().unwrap()));
            self.read(2);
            self.write(14, 0);
        }

        /// `sla5032_start_acquisition()` at 1 MHz, with a trigger.
        fn setup(&mut self, pre: u32, post: u32, level: u32, edge: u32, mask: u32) {
            self.write(7, pre);
            self.write(6, post);
            self.write(0, level);
            self.write(1, edge);
            self.write(2, mask);
            self.write(4, 199);
            self.bit(5, 0, false);
            self.bit(5, 7, true);
            self.write(9, 20_000_000 - 1);
            self.write(10, 200_000 - 1);
            self.write(11, 15 - 1);
            self.write(12, 5 - 1);
            self.write(14, 1);
            for (b, on) in [
                (2, false),
                (3, false),
                (5, false),
                (6, true),
                (1, true),
                (1, false),
                (8, true),
                (8, false),
                (6, true),
                (2, true),
            ] {
                self.bit(5, b, on);
            }
        }

        fn status(&mut self) -> u32 {
            self.read(1);
            self.read(0);
            self.read(2)
        }

        fn read_back(&mut self) {
            self.write(5, READ_BACK_1);
            self.write(5, READ_BACK_2);
        }

        fn arm(&mut self) {
            self.send(&[CMD_READ_DATA, 3]);
            self.write(3, CHUNK_WORDS);
            self.bit(5, 4, false);
            self.bit(5, 4, true);
        }

        fn chunk(&mut self) -> Vec<u8> {
            self.arm();
            match self.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut self.v) {
                Reply::Data(d) => d,
                other => panic!("no chunk: {other:?}"),
            }
        }

        fn done_capture(&mut self, pre: u32, post: u32) {
            self.setup(pre, post, 0, 0, 0);
            assert_eq!(self.status() & STATUS_DONE, 0);
            assert_eq!(self.status() & (STATUS_DONE | STATUS_TRIGGERED), STATUS_TRIGGERED);
            assert_ne!(self.status() & STATUS_DONE, 0);
            self.read_back();
        }
    }

    /// Value and count less 1 of each record.
    fn records(data: &[u8]) -> Vec<(u32, u16)> {
        data.chunks(RECORD)
            .map(|r| {
                (
                    u32::from_le_bytes(r[..4].try_into().unwrap()),
                    u16::from_le_bytes(r[4..].try_into().unwrap()),
                )
            })
            .collect()
    }

    #[test]
    fn descriptors_are_the_wikis() {
        let d = descriptors();
        assert_eq!((d.vendor_id(), d.product_id(), d.bcd_device()), (0x2961, 0x66b0, 0));
        assert_eq!(d.device[7], 64);
        assert_eq!(d.strings, ["www.SysCLK.com", "SYSCLK SLA5032-v1.0"]);
        let config = &d.configs[0];
        assert_eq!(config.len(), 46);
        assert_eq!((config[7], config[8]), (0xa0, 250));
        let eps: Vec<(u8, u16)> =
            Descriptors::endpoints(config).iter().map(|e| (e.address, e.max_packet)).collect();
        assert_eq!(eps, [(0x02, 512), (0x04, 512), (0x86, 512), (0x88, 512)]);
    }

    #[test]
    fn the_standin_is_that_of_the_firmware_list_and_leaves_a_short_chunk() {
        let list = include_str!("../../../tools/emu/sysclk-sla5032.firmware");
        let line = list.lines().find(|l| l.starts_with(BITSTREAM)).unwrap();
        assert_eq!(line, format!("{BITSTREAM} {BITSTREAM_SIZE}"));
        let image = upload_image(BITSTREAM_SIZE);
        assert_eq!(image.len(), BITSTREAM_SIZE - 0x69 + 0x100);
        assert_eq!(image.len() % CHUNK, 151);
        assert_eq!(&image[0x100..0x104], &SYNC);
        assert!(image[..0x100].iter().all(|b| *b == 0xff));
    }

    #[test]
    fn an_upload_configures_and_the_keys_set_bit_0() {
        let mut h = Host::new(&[]);
        assert_eq!(h.read(2), UNCONFIGURED[0]);
        assert_eq!(h.read(2), UNCONFIGURED[1]);
        h.upload();
        h.clean();
        assert_eq!(h.read(2), 0xa5a5_a5a1);
        h.clean();
    }

    #[test]
    fn a_configured_board_starts_with_its_keys() {
        let mut h = Host::configured(&[]);
        assert_eq!(h.read(2), 0xa5a5_a5a1);
        h.clean();
    }

    #[test]
    fn a_capture_keeps_its_depths_and_ends_with_the_marker() {
        let mut h = Host::configured(&[]);
        h.done_capture(1, 1);
        let r = records(&h.chunk());
        assert_eq!(r.len(), 1025);
        assert_eq!(r[0], (0, 0));
        assert_eq!(r[1023], (1023, 0));
        assert_eq!(r[1024], (0, END_MARKER));
        h.write(14, 0);
        h.clean();
    }

    #[test]
    fn a_long_run_is_split_short_of_the_marker() {
        let mut h = Host::configured(&[("period", "200000")]);
        h.done_capture(255, 255);
        // A window of 131072 samples: 65535 + 65535 + 2 of the same value.
        let r = records(&h.chunk());
        assert_eq!(r, [(0, 0xfffe), (0, 0xfffe), (0, 1), (0, END_MARKER)]);
        h.clean();
    }

    #[test]
    fn a_full_chunk_has_no_marker_and_the_next_has_only_it() {
        let mut h = Host::configured(&[]);
        h.done_capture(2047, 2047);
        let first = h.chunk();
        assert_eq!(first.len(), CHUNK_BYTES);
        assert_eq!(records(&first).last(), Some(&(0x000f_ffff, 0)));
        assert_eq!(records(&h.chunk()), [(0, END_MARKER)]);
        h.clean();
    }

    #[test]
    fn no_marker_makes_the_next_chunk_empty() {
        let mut h = Host::configured(&[("no_marker", "1")]);
        h.done_capture(2047, 2047);
        assert_eq!(h.chunk().len(), CHUNK_BYTES);
        assert_eq!(h.chunk(), [] as [u8; 0]);
        h.clean();
    }

    #[test]
    fn a_transfer_cut_short_by_the_host_goes_on_in_the_next() {
        let mut h = Host::configured(&[]);
        h.done_capture(1, 1);
        h.arm();
        let Reply::Data(a) = h.dev.transfer_in(EP_DATA, 512, &mut h.v) else { panic!() };
        assert_eq!(a.len(), 512);
        let Reply::Data(b) = h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v) else { panic!() };
        let all: Vec<u8> = a.into_iter().chain(b).collect();
        let r = records(&all);
        assert_eq!((r.len(), r[85]), (1025, (85, 0)));
        h.clean();
    }

    #[test]
    fn nothing_is_allocated_from_a_huge_depth() {
        let mut h = Host::configured(&[("period", "1000")]);
        h.done_capture(0x00ff_ffff, 0x00ff_ffff);
        assert_eq!(h.chunk().len(), CHUNK_BYTES);
        h.clean();
    }

    #[test]
    fn a_level_trigger_sits_after_the_pre_trigger_depth() {
        let t = Trigger { level: 1 << 12, edge: 0, mask: 1 << 12 };
        assert_eq!(t.search(1, 512), Some(4096));
        assert_eq!(t.search(1, 4097), Some(4097));
    }

    #[test]
    fn edges_fire_only_where_the_input_changes() {
        let rising = Trigger { level: 8, edge: 8, mask: 8 };
        assert_eq!(rising.search(1, 512), Some(520));
        let falling = Trigger { level: 0, edge: 8, mask: 8 };
        assert_eq!(falling.search(1, 512), Some(512));
        assert_eq!(falling.search(1, 513), Some(528));
        assert!(!rising.holds(1, 0));
        // Two edges together: bit 3 rises with bit 2 falling.
        let both = Trigger { level: 8, edge: 12, mask: 12 };
        assert_eq!(both.search(1, 0), Some(8));
    }

    #[test]
    fn a_trigger_the_counter_never_meets_never_fires() {
        let mut h = Host::configured(&[]);
        // CH0 and CH1 rising together: the counter never does that.
        h.setup(1, 1, 3, 3, 3);
        for _ in 0..10 {
            assert_eq!(h.status(), 0xa5a5_a5a1);
        }
        h.clean();
    }

    #[test]
    fn the_fired_trigger_sets_the_window() {
        let mut h = Host::configured(&[]);
        // Pre 512 samples, rising CH3 first at 520 on.
        h.setup(1, 1, 8, 8, 8);
        for _ in 0..3 {
            h.status();
        }
        h.read_back();
        let r = records(&h.chunk());
        assert_eq!(r[0].0, 8);
        assert_eq!(r[1023].0, 1031);
        h.clean();
    }

    #[test]
    fn wait_sets_the_polls_before_the_trigger() {
        let mut h = Host::configured(&[("wait", "3")]);
        h.setup(1, 1, 0, 0, 0);
        for _ in 0..3 {
            assert_eq!(h.status(), 0xa5a5_a5a1);
        }
        assert_eq!(h.status(), 0xa5a5_a5a3);
        assert_eq!(h.status(), 0xa5a5_a5ab);
        // R0 now reads the post-trigger depth, R1 the last sample.
        assert_eq!(h.read(0), 2);
        assert_eq!(h.read(1), 1023);
        h.clean();
    }

    #[test]
    fn without_keys_the_board_never_samples() {
        let mut h = Host::new(&[]);
        h.read(2);
        h.send_image();
        h.setup(1, 1, 0, 0, 0);
        for _ in 0..5 {
            assert_eq!(h.status(), 0xa5a5_a5a0);
        }
        h.clean();
    }

    #[test]
    fn stall_faults_count_their_kind() {
        let mut h = Host::configured(&[("stall", "write-reg:2")]);
        assert_eq!(h.write(9, 20_000_000 - 1), Reply::ack());
        assert_eq!(h.write(10, 200_000 - 1), Reply::Stall);
        assert_eq!(h.write(10, 200_000 - 1), Reply::ack());
        h.clean();
    }

    #[test]
    fn reply_faults() {
        let mut h =
            Host::configured(&[("stall_reply", "read-reg:1"), ("short_reply", "read-reg:2")]);
        h.send(&[CMD_READ_REG, 8, 4]);
        assert_eq!(h.reply(512), Reply::Stall);
        h.send(&[CMD_READ_REG, 8, 4]);
        let Reply::Data(d) = h.reply(512) else { panic!() };
        assert_eq!(d.len(), 3);
        h.clean();
        let mut h = Host::configured(&[("hold_reply", "read-reg")]);
        h.send(&[CMD_READ_REG, 8, 4]);
        assert_eq!(h.reply(512), Reply::Nak);
        assert_eq!(h.reply(512), Reply::Nak);
        h.clean();
    }

    #[test]
    fn data_faults() {
        let mut h = Host::configured(&[("data_stall", "1")]);
        h.done_capture(1, 1);
        h.arm();
        assert_eq!(h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v), Reply::Stall);
        h.clean();
        let mut h = Host::configured(&[("data_hold", "1")]);
        h.done_capture(1, 1);
        h.arm();
        assert_eq!(h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v), Reply::Nak);
        h.clean();
        let mut h = Host::configured(&[("data_short", "7")]);
        h.done_capture(1, 1);
        assert_eq!(h.chunk().len(), 7);
        h.clean();
        let mut h = Host::configured(&[("data_short", "0")]);
        h.done_capture(1, 1);
        assert_eq!(h.chunk(), [] as [u8; 0]);
        h.clean();
    }

    #[test]
    fn no_zlp_holds_a_chunk_on_a_packet_boundary_until_it_is_cancelled() {
        // 65536 samples at 258 a run: 255 records and the marker, 1536 bytes.
        let boundary = [("period", "258"), ("no_zlp", "1")];
        let mut h = Host::configured(&boundary);
        h.done_capture(127, 127);
        h.arm();
        assert_eq!(h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v), Reply::Nak);
        assert_eq!(h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v), Reply::Nak);
        assert_eq!(h.dev.in_unlinked(EP_DATA, CHUNK_BYTES).map(|d| d.len()), Some(1536));
        assert_eq!(h.dev.in_unlinked(EP_DATA, CHUNK_BYTES), None, "once");
        h.clean();
        // Without the fault it ends as ever.
        let mut h = Host::configured(&[("period", "258")]);
        h.done_capture(127, 127);
        assert_eq!(h.chunk().len(), 1536);
        assert_eq!(h.dev.in_unlinked(EP_DATA, CHUNK_BYTES), None);
        h.clean();
        // A short packet needs no zero-length one: 1025 records.
        let mut h = Host::configured(&[("no_zlp", "1")]);
        h.done_capture(1, 1);
        assert_eq!(h.chunk().len(), 1025 * RECORD);
        h.clean();
        // Nor does a transfer the data fills.
        let mut h = Host::configured(&boundary);
        h.done_capture(127, 127);
        h.arm();
        let Reply::Data(d) = h.dev.transfer_in(EP_DATA, 1024, &mut h.v) else { panic!() };
        assert_eq!(d.len(), 1024);
        h.clean();
    }

    #[test]
    fn options_are_refused_when_wrong() {
        for o in [
            ("nonsense", "1"),
            ("period", "0"),
            ("wait", "x"),
            ("configured", "2"),
            ("fw_size", "10"),
            ("fw_size", "524289"),
            ("stall", "bad"),
            ("stall", "init:0"),
            ("stall_reply", "write-reg"),
            ("data_stall", "0"),
            ("data_short", "6291457"),
            ("overcount", "2"),
        ] {
            assert!(build(&opts(&[o])).is_err(), "{o:?}");
        }
    }

    #[test]
    fn violation_commands() {
        let mut h = Host::configured(&[]);
        h.send(&[0x77]);
        h.flagged("unknown command");
        h.send(&[CMD_READ_REG, 8]);
        h.flagged("of 2 bytes, not 3");
        h.send(&[CMD_WRITE_REG, 8, 4, 0]);
        h.flagged("of 4 bytes, not 7");
        h.send(&[CMD_READ_REG, 8, 4]);
        h.send(&[CMD_READ_REG, 8, 4]);
        h.flagged("reply unread");
        h.reply(512);
        h.reply(512);
        h.flagged("no reply due");
        h.send(&[CMD_READ_REG, 8, 4]);
        h.reply(2);
        h.flagged("read with 2");
        h.dev.transfer_out(EP_UNUSED, &[0], &mut h.v);
        h.flagged("endpoint 2");
        h.send(&[CMD_READ_REG, 8, 2]);
        h.flagged("read of 2 bytes");
        h.reply(512);
        h.send(&[CMD_WRITE_REG, 8, 2, 0, 0, 0, 0]);
        h.flagged("write of 2 bytes");
        h.send(&[CMD_READ_REG, 2, 4]);
        h.flagged("register address");
        h.reply(512);
        h.send(&[CMD_READ_REG, 18 * 4, 4]);
        h.flagged("register address");
        h.reply(512);
        h.read(4);
        h.flagged("write-only");
        h.write(8, 0);
        h.flagged("register 8 written");
    }

    #[test]
    fn violation_uploads() {
        let mut h = Host::configured(&[]);
        h.send(&[CMD_INIT_FW_UPLOAD]);
        h.flagged("configured FPGA");
        let mut h = Host::new(&[]);
        h.send(&[CMD_UPLOAD_FW_CHUNK, 1, 0xff]);
        h.flagged("without the upload");
        h.send(&[CMD_INIT_FW_UPLOAD]);
        h.send(&[CMD_UPLOAD_FW_CHUNK, 1, 0xff]);
        h.flagged("a bitstream chunk of 1 bytes");
        let mut h = Host::new(&[]);
        h.send(&[CMD_INIT_FW_UPLOAD]);
        let mut wrong = vec![CMD_UPLOAD_FW_CHUNK, CHUNK as u8];
        wrong.extend_from_slice(&[0; CHUNK]);
        h.send(&wrong);
        h.flagged("bitstream differs");
        h.send(&[CMD_READ_REG, 5 * 4, 4]);
        h.flagged("in the middle of the upload");
    }

    #[test]
    fn a_board_without_keys_may_be_uploaded_again() {
        let mut h = Host::new(&[]);
        h.read(2);
        h.send_image();
        assert_eq!(h.read(2), 0xa5a5_a5a0);
        h.upload();
        h.clean();
        assert_eq!(h.read(2), 0xa5a5_a5a1);
    }

    #[test]
    fn a_status_read_or_a_start_ends_an_abandoned_upload() {
        let chunk: Vec<u8> = [CMD_UPLOAD_FW_CHUNK, CHUNK as u8]
            .into_iter()
            .chain(upload_image(BITSTREAM_SIZE)[..CHUNK].iter().copied())
            .collect();
        // The status read of the next open, then its upload.
        let mut h = Host::new(&[]);
        h.send(&[CMD_INIT_FW_UPLOAD]);
        h.send(&chunk);
        assert_eq!(h.read(2), UNCONFIGURED[0]);
        h.clean();
        h.upload();
        h.clean();
        // A new start with no status read before it.
        let mut h = Host::new(&[]);
        h.send(&[CMD_INIT_FW_UPLOAD]);
        h.send(&chunk);
        h.send_image();
        h.clean();
        assert_eq!(h.read(2), 0xa5a5_a5a0);
        // A chunk after the status read has no upload to go to.
        let mut h = Host::new(&[]);
        h.send(&[CMD_INIT_FW_UPLOAD]);
        h.send(&chunk);
        h.read(2);
        h.send(&chunk);
        h.flagged("without the upload");
    }

    #[test]
    fn overcount_claims_over_2_31_samples_in_few_bytes() {
        let mut h = Host::configured(&[("overcount", "1")]);
        h.done_capture(1, 1);
        let r = records(&h.chunk());
        assert_eq!(r.len(), 32770);
        assert_eq!(r[0], (0, 0xfffe));
        assert_eq!(r[32769], (0, END_MARKER));
        let total: u64 = r[..32769].iter().map(|(_, c)| u64::from(*c) + 1).sum();
        assert!(total > 1 << 31 && total < 1 << 32);
        h.clean();
    }

    #[test]
    fn violation_without_a_design() {
        let mut h = Host::new(&[]);
        h.send(&[CMD_READ_REG, 5 * 4, 4]);
        h.flagged("without a design");
        h.reply(512);
        h.write(4, 1);
        h.flagged("without a design");
        h.send(&[CMD_READ_MEM, 0x0f, 0xa0, 8]);
        h.flagged("without a design");
    }

    #[test]
    fn violation_eeprom_and_keys() {
        let mut h = Host::configured(&[]);
        h.send(&[CMD_READ_MEM, 0, 0, 8]);
        h.flagged("EEPROM read");
        h.reply(512);
        h.send(&[CMD_READ_MEM, 0x0f, 0xa0, 4]);
        h.flagged("EEPROM read");
        h.reply(512);
        h.write(16, 1);
        h.flagged("key register 16");
        h.write(17, 1);
        h.flagged("key register 17");
    }

    #[test]
    fn violation_values() {
        let mut h = Host::configured(&[]);
        h.write(14, 2);
        h.flagged("register 14 written");
        h.write(9, 1);
        h.flagged("PWM register 9");
        h.write(12, 1);
        h.flagged("PWM register 12");
        h.write(6, 0);
        h.flagged("depth's minimum");
        h.write(7, 0);
        h.flagged("depth's minimum");
        h.write(0, 1);
        h.write(2, 0);
        h.flagged("outside the mask");
        h.write(5, 0x200);
        h.flagged("bits above 8");
        h.write(5, 0);
        h.write(5, 1 << 4);
        h.flagged("bit 4 moved");
    }

    #[test]
    fn violation_a_start_that_skipped_a_step() {
        let mut h = Host::configured(&[]);
        h.write(5, CTRL_START);
        h.flagged("without register 7");
        let mut h = Host::configured(&[]);
        h.setup(1, 1, 0, 0, 0);
        h.clean();
        h.write(14, 0);
        h.write(14, 1);
        h.write(5, 0);
        h.write(5, CTRL_START);
        let v = h.violations();
        assert!(v.iter().any(|m| m.contains("register 5 at")), "{v:?}");
        assert!(v.iter().any(|m| m.contains("pulses")), "{v:?}");
        // Sampling was never set.
        let mut h = Host::configured(&[]);
        h.setup(1, 1, 0, 0, 0);
        h.write(14, 0);
        h.write(5, CTRL_B6);
        h.write(5, CTRL_B6 | CTRL_START);
        h.flagged("without register 14");
    }

    #[test]
    fn violation_a_rate_not_in_the_table() {
        let mut h = Host::configured(&[]);
        h.setup(1, 1, 0, 0, 0);
        h.clean();
        h.write(14, 0);
        h.write(5, 0);
        h.write(4, 5);
        h.write(14, 1);
        for w in
            [CTRL_B6 | CTRL_PULSE1, CTRL_B6, CTRL_B6 | CTRL_PULSE8, CTRL_B6, CTRL_B6 | CTRL_START]
        {
            h.write(5, w);
        }
        h.flagged("not in pll_tbl");
    }

    #[test]
    fn violation_order_during_a_capture() {
        let mut h = Host::configured(&[]);
        h.setup(1, 1, 0, 0, 0);
        h.clean();
        h.write(4, 199);
        h.flagged("during a capture");
        h.write(14, 1);
        h.flagged("sampling set during");
        h.bit(5, 0, true);
        h.flagged("during a capture");
        h.write(3, CHUNK_WORDS);
        h.flagged("outside a read-back");
        h.send(&[CMD_READ_DATA, 3]);
        h.flagged("outside a read-back");
    }

    #[test]
    fn violation_read_back() {
        let mut h = Host::configured(&[]);
        h.setup(1, 1, 0, 0, 0);
        for _ in 0..3 {
            h.status();
        }
        h.write(5, 0x28);
        h.flagged("not 0x08");
        h.write(5, READ_BACK_1);
        h.write(5, 0x18);
        h.flagged("not 0x28");
        h.write(5, READ_BACK_2);
        h.clean();
        h.write(5, READ_BACK_2 | 1);
        h.flagged("in a read-back");
        h.write(5, READ_BACK_2);
        h.send(&[CMD_READ_DATA, 4]);
        h.flagged("not 3");
        h.write(3, 1);
        h.flagged("not 0x300000");
    }

    #[test]
    fn violation_chunks() {
        let mut h = Host::configured(&[]);
        h.done_capture(1, 1);
        // Endpoint 6 before any arm.
        assert_eq!(h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v), Reply::Nak);
        h.flagged("no chunk armed");
        // A rise without its command and register 3.
        h.bit(5, 4, true);
        h.flagged("without its command");
        let Reply::Data(_) = h.dev.transfer_in(EP_DATA, CHUNK_BYTES, &mut h.v) else { panic!() };
        // Past the marker.
        h.bit(5, 4, false);
        h.send(&[CMD_READ_DATA, 3]);
        h.write(3, CHUNK_WORDS);
        h.bit(5, 4, true);
        h.flagged("after the end marker");
    }

    #[test]
    fn stray_requests_are_flagged() {
        let mut m = Sla5032::new(Options::default());
        let mut v = Violations::default();
        let s = Setup { request_type: 0x40, request: 1, value: 0, index: 0, length: 0 };
        assert_eq!(m.control(s, &[], &mut v), Reply::Stall);
        assert!(v.drain().iter().any(|m| m.contains("control request")));
        assert_eq!(m.in_transfer(1, 512, &mut v), Reply::Stall);
        assert!(v.drain().iter().any(|m| m.contains("IN transfer")));
        assert_eq!(m.out(4, &[], &mut v), Reply::Stall);
        assert!(v.drain().iter().any(|m| m.contains("empty command")));
    }
}
