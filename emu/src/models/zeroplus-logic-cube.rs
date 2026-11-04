// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The ZEROPLUS Logic Cube LAP-C series as libsigrok's
//! `zeroplus-logic-cube` driver sees it: a vendor USB device with no
//! firmware or bitstream to upload, a register file behind control
//! requests, and a circular sample memory read over one bulk IN
//! endpoint. The driver is synchronous: `dev_acquisition_start()` polls,
//! reads and ends the capture before it returns.
//!
//! Where the facts come from. No ZEROPLUS document, firmware or recording
//! is available here; everything is from the C driver and its history:
//!
//! - Ids and boards: `zeroplus_models[]` (api.c:44-59), `pid=` picks one.
//!   The default, LAP-C(16032), is the board api.c:88-90 names. Sure.
//! - Requests (`gl_usb.c`:40-53, 55-113): vendor requests to the interface,
//!   `bRequest` 0x0c with `wValue` 0x83 (select a register, one byte), 0x85
//!   (write it) and 0x84 (read it); `bRequest` 0x04 with `wValue` 0x82
//!   (eight bytes: four 0, then the size LE32) before each bulk IN on
//!   endpoint 0x81. Interface 0, configuration 1 (api.c:24-25). Sure as
//!   what the driver sends.
//! - Registers: analyzer.c:40-109. The `START_STATUS` values:
//!   analyzer.c:394-444. The clock table: analyzer.c:298-318. The trigger
//!   registers: analyzer.c:531-545 (two bits per channel, 01 high, 10 low;
//!   the edge register's top two bits 01 rising, 10 falling, 11 either, the
//!   low five the channel). Clock source: analyzer.c:464-472.
//! - The memory and its addresses: commit 42ceb777's message, its author's
//!   findings from a real board, "still a bit of a mystery" in his words.
//!   `memory_size` samples in a circular buffer; `now_address` is where the
//!   board was about to write when it finished and where a read starts;
//!   `stop_address` is where it last wrote; `trigger_address` is the
//!   trigger's sample; `STATUS_READY` is set after `memory_size` samples or
//!   fewer and clear after more, the boundary untested by him. "The first
//!   two samples captured are always bogus": api.c:574-576.
//! - The status is 0 after power on (api.c:313-315).
//!
//! How the model behaves (its choices where the sources stop):
//!
//! - Descriptors: high speed, 512 byte bulk packets, device class 0,
//!   interface class 0xff, `bcdDevice` 0, no strings except `serial=TEXT`
//!   (`iSerialNumber` 1). None of it is known; the driver reads only the
//!   ids and the serial.
//! - The status is 0 until the first INIT, then `STATUS_READY` while idle.
//!   `warm=1` starts as an earlier session left it: initialised, the
//!   registers as the driver's power-on configure writes them. A bus reset
//!   (`dev_close()` makes one) keeps the registers, the status and the
//!   memory, and drops the selected register and a readout.
//! - GO starts a capture. The signal is a counter: write `s` of the capture
//!   (from 0, at address `s mod memory_size`) holds `s` within the board's
//!   channels, so channel `n` is bit `n`, whatever the rate, clock source
//!   or thresholds; writes 0 and 1 hold [`BOGUS`]. The trigger fires at the
//!   first `s` at or after `TRIGGERBAR_ADDRESS` (and from 1 for an edge)
//!   meeting its conditions, with none at once; the capture stops after
//!   `s + RAMSIZE_TRIGGERBAR_ADDRESS` writes. So a limit of 1 asks for no
//!   write at all, which is the driver's "empty capture" (api.c:548). The
//!   reading of those two registers is the model's, from their names and
//!   the commit's "added together indicate how many samples we want".
//!   Memory never written in the capture reads as 0.
//! - With no `-t` the board reports trigger address 0: the trigger fires at
//!   once at write 0 (commit 4c1433d1: "we trigger immediately"). A capture
//!   whose limit equals the memory then ends with the now address just
//!   before it, so api.c:556-557 and 591-592 make the trigger offset 0, and
//!   api.c:639-640 sends the trigger with no check of `devc->trigger`: a
//!   marker at sample 0 of a capture that asked for no trigger ("samples
//!   2048", "samples 16384", "samples 32768", "all channels" and those of
//!   the other boards at full memory). A smaller limit shows none, the
//!   offset falling past the samples. The driver's lines are facts; that the
//!   board reports 0 is the model's choice.
//! - A capture delivers fewer samples than its limit (1000 give 996, "Device
//!   only sent 996 samples"), and most of that is the driver's own, whatever
//!   the model says. `set_triggerbar()` (protocol.c:113-120) makes
//!   `ramsize_trigger + triggerbar` the limit - 1 with nothing before the
//!   trigger and the limit - 2 with more than 2 samples before it (limit - 2
//!   or - 3 with 1 or 2), and api.c:582-585 caps `valid_samples` at that sum.
//!   Commit b9a5614d says those offsets "match the values sent by the windows
//!   app", so the hardware may make up for them. `stop - now` (api.c:582, 597)
//!   leaves out the sample at the last address written, by commit 42ceb777's
//!   own definition of `stop_address`. api.c:577 drops the two bogus samples.
//!   Only the rest of the 996 depends on the model's reading, that the two
//!   registers count all the writes, the bogus ones included.
//! - Very small limits deliver nothing. Limits 1 and 2 stop at address 0, the
//!   empty capture of api.c:548. Limit 3 (stop 1, now moved to 2) underflows
//!   at api.c:582, and the discard that follows covers the whole readout.
//!   Limit 4 leaves `valid_samples` 0. The cases "samples 2", "samples 3" and
//!   "samples 4" hold it; the numbers follow the model's reading.
//! - When a capture exactly fills the memory, the bogus words are delivered
//!   (case "trigger fills memory output"). That is the driver's logic under
//!   its own stated facts: a now address of 0 makes api.c:563-564 clear
//!   `STATUS_READY`, so the discard with its + 2 at api.c:577 is skipped.
//! - A capture is busy for `fill` status reads (default 2) after GO, paced
//!   by the polls and never by time. A trigger that cannot fire keeps the
//!   busy bit for ever. An external clock is taken to be connected.
//! - The bulk reads hand out memory from the now address on, wrapping.
//!
//! The pedantic checks: requests other than `gl_usb.c`'s four, their index
//! and length; selecting a register the driver does not know; reading or
//! writing with none selected, or one it only reads or only writes;
//! `START_STATUS` values other than the eight; register values no path of
//! the driver writes (dividers, multipliers and clock selections outside
//! the table, `FREQUENCY_REG2` other than 2, a memory length above 8 or
//! beyond the board, a clock source above 2, a trigger or an edge on a
//! channel the board lacks, edge bits, a trigger count other than 1,
//! don't-care other than 1, any filter, delay, other "insert data",
//! compression other than none); GO without INIT since the last RESET,
//! with registers never written, or during a capture; register writes
//! during a capture or a readout; a register selected, or a `START_STATUS`
//! other than 0xe0, during a readout; address reads before the capture is
//! complete; a readout without a complete capture; `READ_RAM_STATUS`
//! outside a readout; a bulk read request outside a readout, before
//! `READ_RAM_STATUS`, with non-zero leading bytes, not in whole samples,
//! past the memory or on top of an unanswered one; a bulk IN without a
//! request or shorter than the answer; other endpoints.
//!
//! What the driver sends that the model accepts as a C habit, as a board
//! could not refuse it: a level high and low on one channel (`-t A0=0,A0=1`,
//! analyzer.c:531-536 ORs the bits; pair 11 is taken as both, which never
//! fires, case "two levels on one channel hangs"), and a mixture of two
//! rates when a write of the rate is refused (analyzer.c:333-343 gives up
//! midway and the capture starts all the same, cases "fault rate register"),
//! so the three frequency registers are checked one by one, not as a triple.
//! A bulk IN after a refused bulk read request is not flagged either: the
//! driver ignores the refusal (`gl_usb.c`:103-107), and the model NAKs it
//! until the driver's own 5 s (`TIMEOUT_MS`, `gl_usb.c`:46, 108) run out,
//! having no data to send.
//!
//! libsigrok bugs, each read line by line against the source:
//!
//! - `zp_set_samplerate()` loops on `ARRAY_SIZE(samplerates_200)`, always
//!   true (api.c:148), so a rate not in the list reads past the array until a
//!   crash or a match, and the check at api.c:152 cannot fire. What happens
//!   depends on the build's rodata. In the oracle's build 3M and 123456789
//!   crash with SIGSEGV (`--oracle-bug` cases "samplerate 3M crashes" and
//!   "samplerate 123456789 crashes"). 46 (as 10, 41, 109, 116, 2669 and
//!   3045960) meets its value there and is accepted, exit 0.
//!   `__analyzer_set_freq()` then finds it in no table and returns -1
//!   (analyzer.c:326-327), which nobody checks, so the board stays on its old
//!   rate while the driver and the output say 46 Hz (case "samplerate 46
//!   accepted", which a rebuilt oracle may turn into a crash).
//! - A failed or short bulk read only warns (api.c:605-608, the count from
//!   `gl_usb.c`:112), and the whole buffer goes out (api.c:615-648). The bytes
//!   not read repeat the previous read's or, in the first read, are whatever
//!   the uninitialised `g_malloc()` buffer holds (api.c:553; no case sends
//!   those, as they would differ between runs). Wrong samples, exit 0: cases
//!   "fault bulk stall output", "fault bulk request output", "fault short
//!   read output", "unplug mid data output".
//! - `analyzer_wait()` (analyzer.c:407-416) has no timeout and takes an
//!   error for a status. `LIBUSB_ERROR_PIPE`, -9, has the busy bit set: it
//!   polls for ever (case "fault every status read hangs").
//!   `LIBUSB_ERROR_NO_DEVICE`, -4, has it clear: after an unplug the wait
//!   ends, the status and every address read 0xfffffffc, no sample is
//!   delivered and the run exits 0 (case "unplug while polling").
//! - A second edge silently replaces the first (analyzer.c:537-545), so
//!   `-t A0=r,A1=f` triggers on A1 alone, exit 0 (case "two edges").
//! - After a failed serial read `scan()` continues without closing the
//!   handle (api.c:231-235), a leak no output shows (case "serial refused").
//!
//! Faults, all off by default. `stall=WHAT[:REG][@N],...` STALLs the
//! requests of a kind (`addr`, `wdata`, `rdata`, `bulkreq`, `bulk`), only
//! for register REG (the one selected, or for `addr` the one being
//! selected) and only the Nth that matches, counted since the model was
//! built; a stalled request has no effect. `short=N[:LEN]` answers the Nth
//! bulk IN with LEN bytes (default 1020), the rest of its request dropped;
//! LEN is not a whole number of packets, since a transfer of 2048 bytes only
//! ends early on a short packet.
//! `stall_string=I,...` STALLs a string descriptor read. `fill=N` and
//! `warm=1` are the board's own.
//!
//! Emulation artifacts: a one-shot STALL of a bulk IN (a real halt lasts
//! until cleared, and this driver never clears one), the data of a stalled
//! bulk IN being lost; the polls as the capture's time; the counter; the
//! value of the bogus words; 0 in memory never written.
//!
//! What the cases (`tools/emu/zeroplus-logic-cube.sh`) do not reach, and why
//! (`tests/emu/zeroplus-logic-cube/gcov.txt`). Dead code, called by nothing:
//! `analyzer_add_filter`, `analyzer_read_id`, `analyzer_wait_button`,
//! `analyzer_decompress`, and `gl_usb.c`:148-151 (`gl_reg_read_buf()` is only
//! called with length 0). Refused before the driver by sigrok-cli or
//! libsigrok: analyzer.c:528 (a trigger on a disabled channel), analyzer.c:
//! 546-548 and api.c:511-512 (an unknown match), api.c:375, 396-397, 436-437
//! and 471 (no device, unknown keys). Not reachable from these tables:
//! api.c:458-460 (every board is 100 or 200 MHz), protocol.c:31 (a code
//! above 8) and 38 (`clz(0)`: a limit of 1 takes the first branch).
//! `dev_acquisition_stop()` (api.c:659-669): sigrok-cli never calls it, the
//! capture being over when the start returns. Transport: api.c:224 and 241,
//! api.c:354. api.c:297 and 308-310: a failing open, or a claim failing where
//! the configuration did not. Another program's claim already fails
//! `libusb_set_configuration()` with `LIBUSB_ERROR_BUSY`, and an unplug
//! between the scan and the open depends on when libusb sees the removal
//! (tried: it ended at the configuration with `LIBUSB_ERROR_OTHER`). In
//! src/usb.c the driver calls only `sr_usb_open()` and `usb_get_port_path()`;
//! their unreached lines are transport failures.
//!
//! The boards of 2 M samples (0x700d, 0x7016) have no capture case: a full
//! readout is 4096 reads and its `.tx` golden alone 570 KB. Their memory code
//! is checked by a unit test.

use crate::descriptor::{self, DeviceDescriptor, STRING};
use crate::model::{
    BULK, Descriptors, DeviceModel, Reply, Setup, Speed, StandardFault, Violations,
};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "zeroplus-logic-cube";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "ZEROPLUS Logic Cube LAP-C series (0c12:PID, vendor USB); pid=0xPID \
     (default 0x700e, LAP-C(16032)), serial=TEXT, warm=1, fill=N; faults: \
     stall=addr|wdata|rdata|bulkreq|bulk[:REG][@N],..., short=N[:LEN], stall_string=1";

/// The vendor id of every model (api.c `zeroplus_models[]`).
pub const VID: u16 = 0x0c12;

/// One entry of `zeroplus_models[]` (api.c:44-59).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    /// USB product id.
    pub pid: u16,
    /// The model name the driver reports.
    pub name: &'static str,
    /// Channels.
    pub channels: u32,
    /// Sample depth, Ki samples per channel.
    pub depth_k: u32,
    /// Fastest rate, MHz.
    pub max_mhz: u32,
}

/// `zeroplus_models[]`, in the driver's order.
pub const PROFILES: [Profile; 13] = [
    Profile { pid: 0x7002, name: "LAP-16128U", channels: 16, depth_k: 128, max_mhz: 200 },
    Profile { pid: 0x7007, name: "LAP-16032U", channels: 16, depth_k: 32, max_mhz: 200 },
    Profile { pid: 0x7009, name: "LAP-C(16064)", channels: 16, depth_k: 64, max_mhz: 100 },
    Profile { pid: 0x700a, name: "LAP-C(16128)", channels: 16, depth_k: 128, max_mhz: 200 },
    Profile { pid: 0x700b, name: "LAP-C(32128)", channels: 32, depth_k: 128, max_mhz: 200 },
    Profile { pid: 0x700c, name: "LAP-C(321000)", channels: 32, depth_k: 1024, max_mhz: 200 },
    Profile { pid: 0x700d, name: "LAP-C(322000)", channels: 32, depth_k: 2048, max_mhz: 200 },
    Profile { pid: 0x700e, name: "LAP-C(16032)", channels: 16, depth_k: 32, max_mhz: 100 },
    Profile { pid: 0x7016, name: "LAP-C(162000)", channels: 16, depth_k: 2048, max_mhz: 200 },
    Profile { pid: 0x7025, name: "LAP-C(16128+)", channels: 16, depth_k: 128, max_mhz: 200 },
    Profile { pid: 0x7060, name: "Logian-16L", channels: 16, depth_k: 128, max_mhz: 200 },
    Profile { pid: 0x7064, name: "Logian-16L", channels: 16, depth_k: 128, max_mhz: 200 },
    Profile { pid: 0x7100, name: "AKIP-9101", channels: 16, depth_k: 256, max_mhz: 200 },
];

/// The default board: the one api.c:88-90 says the channel names were
/// taken from.
pub const DEFAULT_PID: u16 = 0x700e;

/// The bulk IN endpoint the samples come from (`gl_usb.c` `EP1_BULK_IN`).
pub const EP_DATA: u8 = 1;
/// What `short=N` answers: one packet and a short one.
pub const SHORT: usize = 1020;
/// Bulk packet size (high speed; the model's choice).
pub const PACKET: usize = 512;
/// The driver's bulk read size (api.c `PACKET_SIZE`).
pub const READ_SIZE: usize = 2048;

/// `bRequest` of the register requests (`gl_usb.c`).
pub const REQ_REG: u8 = 0x0c;
/// `bRequest` of the bulk read request (`gl_usb.c`).
pub const REQ_BULK: u8 = 0x04;
/// `wValue`: the bulk read request.
pub const REQ_READBULK: u16 = 0x82;
/// `wValue`: select a register.
pub const REQ_WRITEADDR: u16 = 0x83;
/// `wValue`: read the selected register.
pub const REQ_READDATA: u16 = 0x84;
/// `wValue`: write the selected register.
pub const REQ_WRITEDATA: u16 = 0x85;
/// Vendor, interface, host to device.
pub const CTRL_OUT: u8 = 0x41;
/// Vendor, interface, device to host.
pub const CTRL_IN: u8 = 0xc1;

// The registers (analyzer.c:40-109).
/// Start status: the control register (`unk << 6 | flags`).
pub const START_STATUS: u8 = 0x20;
/// Device status.
pub const DEV_STATUS: u8 = 0x21;
/// Frequency: divider.
pub const FREQUENCY_REG0: u8 = 0x30;
/// Frequency: multiplier.
pub const FREQUENCY_REG1: u8 = 0x31;
/// Frequency: always 2.
pub const FREQUENCY_REG2: u8 = 0x32;
/// Frequency: clock selection.
pub const FREQUENCY_REG4: u8 = 0x34;
/// Memory length code.
pub const MEMORY_LENGTH: u8 = 0x35;
/// Clock source.
pub const CLOCK_SOURCE: u8 = 0x36;
/// Trigger levels, four channels per register.
pub const TRIGGER_STATUS0: u8 = 0x40;
/// Trigger edge.
pub const TRIGGER_EDGE: u8 = 0x48;
/// Trigger count, low byte.
pub const TRIGGER_COUNT0: u8 = 0x50;
/// Trigger count, high byte.
pub const TRIGGER_COUNT1: u8 = 0x51;
/// Threshold of the first group.
pub const TRIGGER_LEVEL0: u8 = 0x55;
/// Post-trigger sample count, three bytes.
pub const RAMSIZE_TRIGGERBAR_ADDRESS0: u8 = 0x60;
/// Trigger bar address, three bytes.
pub const TRIGGERBAR_ADDRESS0: u8 = 0x63;
/// "Don't care trigger bar", always 1.
pub const DONT_CARE_TRIGGERBAR: u8 = 0x66;
/// Filter enable.
pub const FILTER_ENABLE: u8 = 0x70;
/// Filter status, eight registers.
pub const FILTER_STATUS: u8 = 0x71;
/// Delay time, two registers.
pub const ENABLE_DELAY_TIME0: u8 = 0x7a;
/// "Insert data", four registers.
pub const ENABLE_INSERT_DATA0: u8 = 0x80;
/// Compression type, two registers.
pub const COMPRESSION_TYPE0: u8 = 0x84;
/// Trigger address, three bytes.
pub const TRIGGER_ADDRESS0: u8 = 0x90;
/// Now address, three bytes.
pub const NOW_ADDRESS0: u8 = 0x96;
/// Stop address, three bytes.
pub const STOP_ADDRESS0: u8 = 0x9b;
/// Selected before the bulk reads.
pub const READ_RAM_STATUS: u8 = 0xa0;

/// Status bits (analyzer.h).
pub const STATUS_BUSY: u8 = 0x01;
/// Memory not wrapped (see the module documentation).
pub const STATUS_READY: u8 = 0x02;

/// The `START_STATUS` values the driver writes (analyzer.c:394-444).
pub const START_VALUES: [u8; 8] = [0x40, 0x41, 0x42, 0x44, 0xc0, 0xc1, 0xe0, 0xf0];

// This table is copied from analyzer.c:298-318, which carries this notice:
//
// Copyright (C) 2010 Sven Peter <sven@fail0verflow.com>
// Copyright (C) 2010 Haxx Enterprises <bushing@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.

/// `__analyzer_set_freq()`'s table (analyzer.c:298-318): selection,
/// divider, multiplier.
pub const CLOCKS: [(u8, u8, u8); 18] = [
    (0, 1, 20),
    (0, 1, 15),
    (0, 1, 10),
    (0, 2, 16),
    (0, 2, 10),
    (1, 5, 25),
    (1, 5, 10),
    (16, 5, 5),
    (17, 5, 8),
    (32, 5, 20),
    (32, 5, 10),
    (32, 5, 5),
    (33, 5, 5),
    (49, 5, 25),
    (50, 5, 10),
    (64, 5, 5),
    (64, 10, 5),
    (68, 5, 8),
];

/// The "insert data" bytes the driver writes (analyzer.c:378-384).
pub const INSERT_DATA: [u8; 4] = [0x12, 0x34, 0x56, 0x78];

/// The word the first two memory writes of a capture hold
/// (api.c:574-576, "the first two samples captured are always bogus"):
/// every channel high, the model's choice.
pub const BOGUS: u32 = 0xffff_ffff;

/// The registers `analyzer_configure()` writes (analyzer.c:446-502),
/// all of which a capture needs.
fn config_registers() -> impl Iterator<Item = u8> {
    [FREQUENCY_REG0, FREQUENCY_REG1, FREQUENCY_REG2, FREQUENCY_REG4, MEMORY_LENGTH, CLOCK_SOURCE]
        .into_iter()
        .chain(TRIGGER_STATUS0..=TRIGGER_EDGE)
        .chain([TRIGGER_COUNT0, TRIGGER_COUNT1])
        .chain(TRIGGER_LEVEL0..TRIGGER_LEVEL0 + 4)
        .chain(RAMSIZE_TRIGGERBAR_ADDRESS0..=DONT_CARE_TRIGGERBAR)
        .chain(FILTER_ENABLE..FILTER_STATUS + 8)
        .chain([ENABLE_DELAY_TIME0, ENABLE_DELAY_TIME0 + 1])
        .chain(ENABLE_INSERT_DATA0..COMPRESSION_TYPE0 + 2)
}

/// Bytes of memory of a `MEMORY_LENGTH` code (protocol.c:25-32).
pub fn memory_bytes(code: u8) -> u64 {
    if code == 0 { 8 * 1024 } else { (32 * 1024) << code }
}

/// What a `stall=` fault names.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum What {
    /// The register select request.
    Addr,
    /// The register write request.
    Wdata,
    /// The register read request.
    Rdata,
    /// The bulk read request.
    Bulkreq,
    /// A bulk IN transfer.
    Bulk,
}

/// One `stall=` item: the request, the register it is about (the one
/// selected, or for `addr` the one being selected) and the Nth.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stall {
    /// Which request.
    pub what: What,
    /// Only for this register.
    pub reg: Option<u8>,
    /// Only the Nth that matches (from 1); every one otherwise.
    pub nth: Option<u32>,
}

/// Ways the board can misbehave; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Requests that STALL.
    pub stall: Vec<Stall>,
    /// The Nth bulk IN answers this many bytes.
    pub short: Option<(u32, usize)>,
    /// String descriptor indices whose read STALLs.
    pub stall_string: Vec<u8>,
}

fn parse_u8(k: &str, v: &str) -> Result<u8> {
    let n = match v.strip_prefix("0x") {
        Some(hex) => u8::from_str_radix(hex, 16),
        None => v.parse(),
    };
    n.map_err(|_| Error(format!("{k}={v}: a byte, decimal or 0x hex")))
}

fn parse_count(k: &str, v: &str) -> Result<u32> {
    v.parse::<u32>().ok().filter(|n| *n > 0).ok_or_else(|| Error(format!("{k}={v}: N from 1")))
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        match k {
            "stall" => {
                self.stall.clear();
                for item in v.split(',').filter(|i| !i.is_empty()) {
                    let (head, nth) = match item.split_once('@') {
                        Some((h, n)) => (h, Some(parse_count(k, n)?)),
                        None => (item, None),
                    };
                    let (name, reg) = match head.split_once(':') {
                        Some((n, r)) => (n, Some(parse_u8(k, r)?)),
                        None => (head, None),
                    };
                    let what = match name {
                        "addr" => What::Addr,
                        "wdata" => What::Wdata,
                        "rdata" => What::Rdata,
                        "bulkreq" => What::Bulkreq,
                        "bulk" => What::Bulk,
                        _ => {
                            return Err(Error(format!(
                                "stall={name}: addr, wdata, rdata, bulkreq or bulk"
                            )));
                        }
                    };
                    self.stall.push(Stall { what, reg, nth });
                }
            }
            "short" => {
                let (n, len) = match v.split_once(':') {
                    Some((n, l)) => (
                        parse_count(k, n)?,
                        l.parse::<usize>()
                            .ok()
                            .filter(|l| *l < READ_SIZE && l % 4 == 0 && l % PACKET != 0)
                            .ok_or_else(|| {
                                Error(format!(
                                    "short={v}: LEN a multiple of 4 below {READ_SIZE}, not of \
                                     {PACKET}"
                                ))
                            })?,
                    ),
                    None => (parse_count(k, v)?, SHORT),
                };
                self.short = Some((n, len));
            }
            "stall_string" => {
                self.stall_string.clear();
                for i in v.split(',').filter(|i| !i.is_empty()) {
                    self.stall_string.push(parse_u8(k, i)?);
                }
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// A board from the `-o` options (see the module documentation).
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut pid = DEFAULT_PID;
    let mut serial = None;
    let mut warm = false;
    let mut fill = 2;
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "pid" => {
                let hex = v.strip_prefix("0x").unwrap_or(v);
                pid = u16::from_str_radix(hex, 16)
                    .ok()
                    .filter(|p| PROFILES.iter().any(|m| m.pid == *p))
                    .ok_or_else(|| Error(format!("pid={v}: one of zeroplus_models[] (hex)")))?;
            }
            "serial" => serial = Some(v.clone()),
            "warm" => {
                warm = match v.as_str() {
                    "0" => false,
                    "1" => true,
                    _ => return Err(Error(format!("warm={v}: 0 or 1"))),
                };
            }
            "fill" => fill = v.parse().map_err(|_| Error(format!("fill={v}: a count")))?,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let mut model = Zeroplus::new(pid, serial);
    model.fill = fill;
    model.faults = faults;
    if warm {
        model.warm();
    }
    Ok(Box::new(model))
}

/// The smallest `t >= lo` with `t & mask == value` (`value` within
/// `mask`, both within 32 bits, `t` counted on 64), as in
/// `hantek-4032l.rs`.
pub fn next_match(lo: u64, mask: u32, value: u32) -> u64 {
    let (mask, value) = (u64::from(mask), u64::from(value));
    if lo & mask == value {
        return lo;
    }
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

/// A capture as GO starts it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capture {
    /// Memory words (`MEMORY_LENGTH`).
    pub words: u64,
    /// Memory writes in all; `None` while the trigger never fires.
    pub written: Option<u64>,
    /// The write at which the trigger fired.
    pub trigger: u64,
    /// The channels the board has.
    pub mask: u32,
}

impl Capture {
    /// Whether the memory has not wrapped (`STATUS_READY`).
    pub fn ready(&self) -> bool {
        self.written.is_some_and(|w| w <= self.words)
    }

    /// `(trigger, now, stop)` addresses.
    pub fn addresses(&self) -> (u32, u32, u32) {
        let w = self.written.unwrap_or(0);
        let now = w % self.words;
        let stop = if w == 0 { 0 } else { (w - 1) % self.words };
        ((self.trigger % self.words) as u32, now as u32, stop as u32)
    }

    /// The word at memory address `a`.
    pub fn word(&self, a: u64) -> u32 {
        let w = self.written.unwrap_or(0);
        if a >= w {
            // Never written in this capture.
            return 0;
        }
        let s = a + self.words * ((w - 1 - a) / self.words);
        if s < 2 { BOGUS & self.mask } else { (s as u32) & self.mask }
    }
}

/// Where the capture logic is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Run {
    /// No capture since the last reset.
    Idle,
    /// After GO: busy for `left` more status reads, then done.
    Busy { left: u32, cap: Capture },
    /// Complete.
    Done(Capture),
}

/// The memory readout after `START_STATUS` 0xf0.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Readout {
    /// `READ_RAM_STATUS` was selected.
    armed: bool,
    /// Words handed out since, from the now address.
    at: u64,
    /// The size of the bulk read request not yet answered.
    pending: Option<usize>,
}

/// The emulated board.
pub struct Zeroplus {
    descriptors: Descriptors,
    profile: Profile,
    /// Status reads that answer busy after GO.
    pub fill: u32,
    faults: Faults,
    /// Initialised since power on (the status is not 0).
    powered: bool,
    regs: [Option<u8>; 256],
    addr: Option<u8>,
    /// INIT was written since the last RESET.
    init: bool,
    run: Run,
    read: Option<Readout>,
    /// A bulk read request was refused by a fault.
    refused: bool,
    stall_seen: Vec<u32>,
    bulk_seen: u32,
}

impl Zeroplus {
    /// A board with product id `pid` (one of [`PROFILES`]) and an
    /// optional serial number string.
    ///
    /// # Panics
    ///
    /// On a `pid` not in [`PROFILES`].
    pub fn new(pid: u16, serial: Option<String>) -> Zeroplus {
        let profile = *PROFILES.iter().find(|p| p.pid == pid).expect("a known product id");
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 1, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(0x80 | EP_DATA, BULK, PACKET as u16, 0));
        let config = descriptor::configuration(1, 0, 0x80, 500, &body);
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: VID,
            product: pid,
            device: 0x0000,
            strings: [0, 0, u8::from(serial.is_some())],
            configurations: 1,
        };
        Zeroplus {
            descriptors: Descriptors {
                speed: Speed::High,
                device: device.bytes(),
                configs: vec![config],
                strings: serial.into_iter().collect(),
            },
            profile,
            fill: 2,
            faults: Faults::default(),
            powered: false,
            regs: [None; 256],
            addr: None,
            init: false,
            run: Run::Idle,
            read: None,
            refused: false,
            stall_seen: Vec::new(),
            bulk_seen: 0,
        }
    }

    /// The state an earlier session left: initialised, the registers
    /// as the driver's power-on configure writes them.
    pub fn warm(&mut self) {
        self.powered = true;
        let mut set = |r: u8, v: u8| self.regs[usize::from(r)] = Some(v);
        for r in config_registers() {
            set(r, 0);
        }
        // 1 MHz, 8K, internal clock, count 1, 1.5 V, 2048 after the
        // trigger, analyzer.c:120-126.
        set(FREQUENCY_REG0, 5);
        set(FREQUENCY_REG1, 5);
        set(FREQUENCY_REG2, 2);
        set(FREQUENCY_REG4, 16);
        set(CLOCK_SOURCE, 1);
        set(TRIGGER_COUNT0, 1);
        for i in 0..4 {
            set(TRIGGER_LEVEL0 + i, 0x31);
        }
        set(RAMSIZE_TRIGGERBAR_ADDRESS0 + 1, 0x08);
        set(DONT_CARE_TRIGGERBAR, 1);
        for (i, b) in INSERT_DATA.iter().enumerate() {
            set(ENABLE_INSERT_DATA0 + i as u8, *b);
        }
        set(COMPRESSION_TYPE0, 1);
    }

    /// The channels the board has, as a mask.
    fn channel_mask(&self) -> u32 {
        if self.profile.channels >= 32 { u32::MAX } else { (1 << self.profile.channels) - 1 }
    }

    /// Counts a request; whether a fault STALLs it.
    fn stalls(&mut self, what: What, reg: Option<u8>) -> bool {
        if self.stall_seen.len() < self.faults.stall.len() {
            self.stall_seen.resize(self.faults.stall.len(), 0);
        }
        let mut stall = false;
        for (i, s) in self.faults.stall.iter().enumerate() {
            if s.what != what || (s.reg.is_some() && s.reg != reg) {
                continue;
            }
            self.stall_seen[i] += 1;
            if s.nth.is_none_or(|n| n == self.stall_seen[i]) {
                stall = true;
            }
        }
        stall
    }

    fn reg(&self, r: u8) -> u8 {
        self.regs[usize::from(r)].unwrap_or(0)
    }

    fn reg24(&self, r: u8) -> u64 {
        u64::from(self.reg(r)) | u64::from(self.reg(r + 1)) << 8 | u64::from(self.reg(r + 2)) << 16
    }

    /// The status register.
    fn status(&mut self) -> u8 {
        if !self.powered {
            return 0;
        }
        match self.run {
            Run::Idle => STATUS_READY,
            Run::Busy { left, cap } if left > 0 => {
                self.run = Run::Busy { left: left - 1, cap };
                STATUS_BUSY | STATUS_READY
            }
            // Waiting for a trigger that never comes, the memory long
            // wrapped.
            Run::Busy { cap, .. } if cap.written.is_none() => STATUS_BUSY,
            Run::Busy { cap, .. } | Run::Done(cap) => {
                self.run = Run::Done(cap);
                if cap.ready() { STATUS_READY } else { 0 }
            }
        }
    }

    /// The capture the registers describe, at GO.
    fn start(&mut self, v: &mut Violations) -> Capture {
        let missing: Vec<String> = config_registers()
            .filter(|r| self.regs[usize::from(*r)].is_none())
            .map(|r| format!("{r:#04x}"))
            .collect();
        if !missing.is_empty() {
            v.report(format!("GO with registers never written: {}", missing.join(" ")));
        }
        let words = memory_bytes(self.reg(MEMORY_LENGTH).min(8)) / 4;
        let pre = self.reg24(TRIGGERBAR_ADDRESS0);
        let post = self.reg24(RAMSIZE_TRIGGERBAR_ADDRESS0);
        let chmask = self.channel_mask();
        // The level conditions: per channel 01 high, 10 low, 11 both.
        let (mut mask, mut value, mut never) = (0u32, 0u32, false);
        for c in 0..32 {
            let pair = (self.reg(TRIGGER_STATUS0 + (c / 4) as u8) >> ((c % 4) * 2)) & 3;
            let bit = 1u32 << c;
            match pair {
                1 if chmask & bit == 0 => never = true,
                1 => {
                    mask |= bit;
                    value |= bit;
                }
                2 if chmask & bit != 0 => mask |= bit,
                3 => never = true,
                _ => {}
            }
        }
        let edge = self.reg(TRIGGER_EDGE);
        let mut lo = pre;
        if edge >> 6 != 0 {
            let e = u32::from(edge & 0x1f);
            let low = (1u32 << e) - 1;
            let (m, val) = match edge >> 6 {
                // From 0 to 1 at bit e: bits 0 to e are 1 followed by 0s.
                1 => (low | (1 << e), 1 << e),
                // From 1 to 0: bits 0 to e all 0.
                2 => (low | (1 << e), 0),
                // Either: the bits below e all 0.
                _ => (low, 0),
            };
            if chmask & (1 << e) == 0 || (value ^ val) & mask & m != 0 {
                never = true;
            }
            mask |= m;
            value |= val;
            lo = lo.max(1);
        }
        let trigger = if never { None } else { Some(next_match(lo, mask, value & mask)) };
        Capture {
            words,
            written: trigger.map(|t| t + post),
            trigger: trigger.unwrap_or(0),
            mask: chmask,
        }
    }

    fn start_status(&mut self, value: u8, v: &mut Violations) {
        if !START_VALUES.contains(&value) {
            v.report(format!(
                "START_STATUS {value:#04x}: not one of the driver's {START_VALUES:02x?}"
            ));
            return;
        }
        if self.read.is_some() && value != 0xe0 {
            v.report(format!("START_STATUS {value:#04x} during the memory readout"));
        }
        match value {
            0x41 | 0xc1 => {
                self.run = Run::Idle;
                self.read = None;
                self.init = false;
            }
            0x42 => {
                self.init = true;
                self.powered = true;
            }
            0x44 => {
                if matches!(self.run, Run::Busy { .. }) {
                    v.report("GO while a capture runs");
                }
                if !self.init {
                    v.report("GO without INIT since the last RESET");
                }
                let cap = self.start(v);
                self.run = Run::Busy { left: self.fill, cap };
            }
            0xf0 => {
                if !matches!(self.run, Run::Done(_)) {
                    v.report("memory readout without a complete capture");
                }
                self.read = Some(Readout::default());
            }
            0xe0 => {
                if self.read.is_none() {
                    v.report("end of the memory readout without one");
                }
                self.read = None;
            }
            // 0x40, 0xc0: the flags released.
            _ => {}
        }
    }

    /// The checks of a register write; whether it is one the board has.
    fn check_write(&self, r: u8, val: u8, v: &mut Violations) {
        let ch = self.profile.channels;
        let bad = |what: &str| format!("register {r:#04x} = {val:#04x}: {what}");
        match r {
            FREQUENCY_REG0 if ![1, 2, 5, 10].contains(&val) => {
                v.report(bad("not a divider of __analyzer_set_freq()'s table"));
            }
            FREQUENCY_REG1 if !CLOCKS.iter().any(|c| c.2 == val) => {
                v.report(bad("not a multiplier of __analyzer_set_freq()'s table"));
            }
            FREQUENCY_REG4 if !CLOCKS.iter().any(|c| c.0 == val) => {
                v.report(bad("not a clock selection of __analyzer_set_freq()'s table"));
            }
            FREQUENCY_REG2 if val != 2 => v.report(bad("the driver always writes 2")),
            MEMORY_LENGTH => {
                if val > 8 {
                    v.report(bad("no memory length code above 8 (MEMORY_SIZE_8M)"));
                } else if memory_bytes(val) / 4 > u64::from(self.profile.depth_k) * 1024 {
                    v.report(bad(&format!(
                        "more memory than the {}'s {}K samples",
                        self.profile.name, self.profile.depth_k
                    )));
                }
            }
            CLOCK_SOURCE if val > 2 => {
                v.report(bad("not 0 (external, falling), 1 (internal) or 2 (external, rising)"));
            }
            0x40..=0x47 if ch <= u32::from(r - TRIGGER_STATUS0) * 4 && val != 0 => {
                v.report(bad(&format!("a trigger on a channel the {} lacks", self.profile.name)));
            }
            TRIGGER_EDGE => {
                let (kind, c) = (val >> 6, u32::from(val & 0x1f));
                if val & 0x20 != 0 || (kind == 0 && c != 0) {
                    v.report(bad("bits the driver never sets"));
                } else if kind != 0 && c >= ch {
                    v.report(bad(&format!("an edge on a channel the {} lacks", self.profile.name)));
                }
            }
            TRIGGER_COUNT0 if val != 1 => v.report(bad("the model counts one trigger only")),
            TRIGGER_COUNT1 if val != 0 => v.report(bad("the model counts one trigger only")),
            DONT_CARE_TRIGGERBAR if val != 1 => v.report(bad("the driver always writes 1")),
            0x70..=0x78 if val != 0 => v.report(bad("the model has no input filter")),
            0x7a | 0x7b if val != 0 => v.report(bad("the driver always writes 0")),
            0x80..=0x83 if val != INSERT_DATA[usize::from(r - 0x80)] => {
                v.report(bad("not the driver's 12 34 56 78"));
            }
            COMPRESSION_TYPE0 if val != 1 => v.report(bad("only COMPRESSION_NONE is modelled")),
            0x85 if val != 0 => v.report(bad("only COMPRESSION_NONE is modelled")),
            _ => {}
        }
    }

    fn writable(r: u8) -> bool {
        r == START_STATUS || config_registers().any(|c| c == r)
    }

    fn readable(r: u8) -> bool {
        r == DEV_STATUS
            || (TRIGGER_ADDRESS0..TRIGGER_ADDRESS0 + 3).contains(&r)
            || (NOW_ADDRESS0..NOW_ADDRESS0 + 3).contains(&r)
            || (STOP_ADDRESS0..STOP_ADDRESS0 + 3).contains(&r)
    }

    fn select(&mut self, r: u8, v: &mut Violations) {
        if !(Self::writable(r) || Self::readable(r) || r == READ_RAM_STATUS) {
            v.report(format!("register {r:#04x} selected: not one the driver knows"));
        }
        if let Some(read) = &mut self.read {
            if r == READ_RAM_STATUS {
                read.armed = true;
            } else if r != START_STATUS {
                v.report(format!("register {r:#04x} selected during the memory readout"));
            }
        } else if r == READ_RAM_STATUS {
            v.report("READ_RAM_STATUS selected outside a memory readout (START_STATUS 0xf0)");
        }
        self.addr = Some(r);
    }

    fn write(&mut self, val: u8, v: &mut Violations) {
        let Some(r) = self.addr else {
            v.report(format!("register write of {val:#04x} with no register selected"));
            return;
        };
        if r == START_STATUS {
            self.start_status(val, v);
            return;
        }
        if !Self::writable(r) {
            v.report(format!("register {r:#04x} written: not one the driver writes"));
            return;
        }
        if matches!(self.run, Run::Busy { .. }) {
            v.report(format!("register {r:#04x} written while a capture runs"));
        }
        if self.read.is_some() {
            v.report(format!("register {r:#04x} written during the memory readout"));
        }
        self.check_write(r, val, v);
        self.regs[usize::from(r)] = Some(val);
    }

    fn read(&mut self, v: &mut Violations) -> u8 {
        let Some(r) = self.addr else {
            v.report("register read with no register selected");
            return 0;
        };
        if r == DEV_STATUS {
            return self.status();
        }
        if !Self::readable(r) {
            v.report(format!("register {r:#04x} read: not one the driver reads"));
            return 0;
        }
        let Run::Done(cap) = self.run else {
            v.report(format!("register {r:#04x} read without a complete capture"));
            return 0;
        };
        let (trigger, now, stop) = cap.addresses();
        let (word, base) = if r >= STOP_ADDRESS0 {
            (stop, STOP_ADDRESS0)
        } else if r >= NOW_ADDRESS0 {
            (now, NOW_ADDRESS0)
        } else {
            (trigger, TRIGGER_ADDRESS0)
        };
        (word >> (8 * (r - base))) as u8
    }

    fn bulk_request(&mut self, data: &[u8], v: &mut Violations) {
        let size = u32::from_le_bytes([data[4], data[5], data[6], data[7]]) as usize;
        if data[..4] != [0; 4] {
            v.report(format!("bulk read request {data:02x?}: its first four bytes are 0"));
        }
        let words = match self.run {
            Run::Done(cap) => cap.words,
            _ => 0,
        };
        let Some(read) = &mut self.read else {
            v.report("bulk read request outside a memory readout (START_STATUS 0xf0)");
            return;
        };
        if !read.armed {
            v.report("bulk read request before READ_RAM_STATUS was selected");
        }
        if size == 0 || !size.is_multiple_of(4) {
            v.report(format!("bulk read request of {size} bytes: whole samples of 4"));
        } else if read.at + (size / 4) as u64 > words {
            v.report(format!(
                "bulk read request of {size} bytes past the {words} words of memory ({} read)",
                read.at
            ));
        }
        if read.pending.is_some() {
            v.report("bulk read request while the previous one is unanswered");
        }
        read.pending = Some(size);
    }

    fn bulk_in(&mut self, max: usize, v: &mut Violations) -> Reply {
        let cap = match self.run {
            Run::Done(cap) => Some(cap),
            _ => None,
        };
        let pending = self.read.as_mut().and_then(|r| r.pending.take());
        let Some(size) = pending else {
            if self.refused {
                // A request a fault refused: nothing to send.
                return Reply::Nak;
            }
            v.report("bulk IN without a bulk read request");
            return Reply::Stall;
        };
        if max < size {
            v.report(format!("bulk IN of {max} bytes for a request of {size}"));
        }
        self.bulk_seen += 1;
        let read = self.read.as_mut().expect("a pending request is in a readout");
        let first = read.at;
        read.at += (size / 4) as u64;
        if self.stalls(What::Bulk, None) {
            return Reply::Stall;
        }
        let len = match self.faults.short {
            Some((n, len)) if n == self.bulk_seen => len.min(size),
            _ => size,
        }
        .min(max);
        let Some(cap) = cap else {
            return Reply::Data(vec![0; len]);
        };
        let (_, now, _) = cap.addresses();
        let mut b = Vec::with_capacity(len);
        for i in 0..(len / 4) as u64 {
            let a = (u64::from(now) + first + i) % cap.words;
            b.extend_from_slice(&cap.word(a).to_le_bytes());
        }
        Reply::Data(b)
    }
}

impl DeviceModel for Zeroplus {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let what = match (setup.request_type, setup.request, setup.value) {
            (CTRL_OUT, REQ_REG, REQ_WRITEADDR) => What::Addr,
            (CTRL_OUT, REQ_REG, REQ_WRITEDATA) => What::Wdata,
            (CTRL_IN, REQ_REG, REQ_READDATA) => What::Rdata,
            (CTRL_OUT, REQ_BULK, REQ_READBULK) => What::Bulkreq,
            _ => {
                v.report(format!(
                    "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one of gl_usb.c's",
                    setup.request_type, setup.request, setup.value, setup.index
                ));
                return Reply::Stall;
            }
        };
        let want = if what == What::Bulkreq { 8 } else { 1 };
        if setup.index != 0 || setup.length != want || (!setup.is_in() && data.len() != want.into())
        {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x}: index {} length {}, the driver sends \
                 index 0 length {want}",
                setup.request_type, setup.request, setup.value, setup.index, setup.length
            ));
            return Reply::Stall;
        }
        let reg = match what {
            What::Addr => Some(data[0]),
            What::Wdata | What::Rdata => self.addr,
            _ => None,
        };
        if self.stalls(what, reg) {
            if what == What::Bulkreq {
                self.refused = true;
            }
            return Reply::Stall;
        }
        match what {
            What::Addr => {
                self.select(data[0], v);
                Reply::ack()
            }
            What::Wdata => {
                self.write(data[0], v);
                Reply::ack()
            }
            What::Rdata => Reply::Data(vec![self.read(v)]),
            _ => {
                self.bulk_request(data, v);
                Reply::ack()
            }
        }
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the board has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP_DATA {
            v.report(format!("IN transfer from endpoint {ep}; the samples come from {EP_DATA}"));
            return Reply::Stall;
        }
        self.bulk_in(max, v)
    }

    fn reset(&mut self) {
        // A bus reset: the USB side starts over, the analyzer keeps its
        // registers, its status and its memory.
        self.addr = None;
        self.read = None;
        self.refused = false;
    }

    /// Bulk transfers of 2048 bytes.
    fn log_digest_above(&self) -> Option<usize> {
        Some(crate::txlog::HASH_ABOVE)
    }

    fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
        const GET_DESCRIPTOR: u8 = 6;
        if setup.request_type != 0x80
            || setup.request != GET_DESCRIPTOR
            || setup.value >> 8 != u16::from(STRING)
        {
            return None;
        }
        let index = (setup.value & 0xff) as u8;
        self.faults.stall_string.contains(&index).then_some(StandardFault::Stall)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup(request_type: u8, request: u8, value: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index: 0, length }
    }

    fn select(d: &mut Zeroplus, r: u8, v: &mut Violations) -> Reply {
        d.control(setup(CTRL_OUT, REQ_REG, REQ_WRITEADDR, 1), &[r], v)
    }

    fn poke(d: &mut Zeroplus, val: u8, v: &mut Violations) -> Reply {
        d.control(setup(CTRL_OUT, REQ_REG, REQ_WRITEDATA, 1), &[val], v)
    }

    fn peek(d: &mut Zeroplus, v: &mut Violations) -> Reply {
        d.control(setup(CTRL_IN, REQ_REG, REQ_READDATA, 1), &[], v)
    }

    /// `gl_reg_write()`.
    fn wr(d: &mut Zeroplus, r: u8, val: u8, v: &mut Violations) {
        select(d, r, v);
        poke(d, val, v);
    }

    /// `gl_reg_read()`.
    fn rd(d: &mut Zeroplus, r: u8, v: &mut Violations) -> u8 {
        select(d, r, v);
        match peek(d, v) {
            Reply::Data(b) => b[0],
            other => panic!("{other:?}"),
        }
    }

    fn rd24(d: &mut Zeroplus, r: u8, v: &mut Violations) -> u32 {
        let hi = u32::from(rd(d, r + 2, v));
        let mid = u32::from(rd(d, r + 1, v));
        hi << 16 | mid << 8 | u32::from(rd(d, r, v))
    }

    fn bulk_request(d: &mut Zeroplus, size: u32, v: &mut Violations) -> Reply {
        let s = size.to_le_bytes();
        let p = [0, 0, 0, 0, s[0], s[1], s[2], s[3]];
        d.control(setup(CTRL_OUT, REQ_BULK, REQ_READBULK, 8), &p, v)
    }

    /// `analyzer_configure()` with a memory code and the two counts.
    fn configure(d: &mut Zeroplus, mem: u8, pre: u32, post: u32, v: &mut Violations) {
        for s in [0x41, 0x40, 0x42, 0x40] {
            wr(d, START_STATUS, s, v);
        }
        wr(d, FREQUENCY_REG0, 5, v);
        wr(d, FREQUENCY_REG1, 5, v);
        wr(d, FREQUENCY_REG2, 2, v);
        wr(d, FREQUENCY_REG4, 16, v);
        wr(d, MEMORY_LENGTH, mem, v);
        wr(d, CLOCK_SOURCE, 1, v);
        for r in TRIGGER_STATUS0..=TRIGGER_EDGE {
            wr(d, r, 0, v);
        }
        wr(d, TRIGGER_COUNT0, 1, v);
        wr(d, TRIGGER_COUNT1, 0, v);
        for i in 0..4 {
            wr(d, TRIGGER_LEVEL0 + i, 0x31, v);
        }
        for i in 0..3 {
            wr(d, RAMSIZE_TRIGGERBAR_ADDRESS0 + i, (post >> (8 * i)) as u8, v);
            wr(d, TRIGGERBAR_ADDRESS0 + i, (pre >> (8 * i)) as u8, v);
        }
        wr(d, DONT_CARE_TRIGGERBAR, 1, v);
        for r in FILTER_ENABLE..FILTER_STATUS + 8 {
            wr(d, r, 0, v);
        }
        wr(d, ENABLE_DELAY_TIME0, 0, v);
        wr(d, ENABLE_DELAY_TIME0 + 1, 0, v);
        for (i, b) in INSERT_DATA.iter().enumerate() {
            wr(d, ENABLE_INSERT_DATA0 + i as u8, *b, v);
        }
        wr(d, COMPRESSION_TYPE0, 1, v);
        wr(d, COMPRESSION_TYPE0 + 1, 0, v);
    }

    /// `analyzer_start()`.
    fn go(d: &mut Zeroplus, v: &mut Violations) {
        for s in [0x40, 0x42, 0x40, 0x44] {
            wr(d, START_STATUS, s, v);
        }
    }

    /// `analyzer_wait_data()`: the polls until not busy, the last status.
    fn wait(d: &mut Zeroplus, v: &mut Violations) -> (u32, u8) {
        for polls in 1..100 {
            let s = rd(d, DEV_STATUS, v);
            if s & STATUS_BUSY == 0 {
                return (polls, s);
            }
        }
        panic!("still busy");
    }

    /// A clean, complete capture on a fresh board.
    fn done(mem: u8, pre: u32, post: u32) -> Zeroplus {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        let mut v = Violations::default();
        configure(&mut d, mem, pre, post, &mut v);
        go(&mut d, &mut v);
        wait(&mut d, &mut v);
        assert!(v.is_empty(), "{v:?}");
        d
    }

    /// `analyzer_read_start()`.
    fn read_start(d: &mut Zeroplus, v: &mut Violations) {
        wr(d, START_STATUS, 0xf0, v);
        select(d, READ_RAM_STATUS, v);
    }

    fn read_bulk(d: &mut Zeroplus, v: &mut Violations) -> Vec<u32> {
        bulk_request(d, READ_SIZE as u32, v);
        match d.in_transfer(EP_DATA, READ_SIZE, v) {
            Reply::Data(b) => {
                b.chunks(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
            }
            other => panic!("{other:?}"),
        }
    }

    /// What `f` does to a fresh board reports `want`.
    fn fires(want: &str, f: impl FnOnce(&mut Zeroplus, &mut Violations)) {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        fires_on(&mut d, want, f);
    }

    fn fires_on(d: &mut Zeroplus, want: &str, f: impl FnOnce(&mut Zeroplus, &mut Violations)) {
        let mut v = Violations::default();
        f(d, &mut v);
        let all = v.drain();
        assert!(all.iter().any(|m| m.contains(want)), "{want:?} not in {all:?}");
    }

    type Step = fn(&mut Zeroplus, &mut Violations);

    fn opts(list: &[(&str, &str)]) -> Vec<(String, String)> {
        list.iter().map(|(k, v)| ((*k).to_string(), (*v).to_string())).collect()
    }

    #[test]
    fn the_status_is_0_until_init() {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        let mut v = Violations::default();
        assert_eq!(rd(&mut d, DEV_STATUS, &mut v), 0);
        wr(&mut d, START_STATUS, 0x42, &mut v);
        assert_eq!(rd(&mut d, DEV_STATUS, &mut v), STATUS_READY);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_warm_board_has_status_and_captures_without_configure() {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        d.warm();
        let mut v = Violations::default();
        assert_eq!(rd(&mut d, DEV_STATUS, &mut v), STATUS_READY);
        go(&mut d, &mut v);
        wait(&mut d, &mut v);
        assert_eq!(rd24(&mut d, NOW_ADDRESS0, &mut v), 0x800 % 2048);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_capture_that_did_not_wrap_reads_from_the_now_address() {
        let mut d = done(0, 0, 999);
        let mut v = Violations::default();
        assert_eq!(rd(&mut d, DEV_STATUS, &mut v), STATUS_READY);
        assert_eq!(rd24(&mut d, STOP_ADDRESS0, &mut v), 998);
        assert_eq!(rd24(&mut d, NOW_ADDRESS0, &mut v), 999);
        assert_eq!(rd24(&mut d, TRIGGER_ADDRESS0, &mut v), 0);
        read_start(&mut d, &mut v);
        let p0 = read_bulk(&mut d, &mut v);
        // Addresses 999 to 1510: never written.
        assert!(p0.iter().all(|w| *w == 0));
        read_bulk(&mut d, &mut v);
        let p2 = read_bulk(&mut d, &mut v);
        // Addresses 2023 to 2047, then 0, 1 (bogus), 2...
        assert_eq!(p2[24], 0);
        assert_eq!(p2[25..28], [0xffff, 0xffff, 2]);
        let p3 = read_bulk(&mut d, &mut v);
        assert_eq!(p3[511], 998);
        wr(&mut d, START_STATUS, 0xe0, &mut v);
        wr(&mut d, START_STATUS, 0xc0, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_capture_past_the_memory_wraps() {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        let mut v = Violations::default();
        configure(&mut d, 0, 498, 500, &mut v);
        // B7 rising, channel 15: the first write 32768.
        wr(&mut d, TRIGGER_EDGE, 0x40 | 15, &mut v);
        go(&mut d, &mut v);
        let (polls, status) = wait(&mut d, &mut v);
        assert_eq!((polls, status), (3, 0));
        assert_eq!(rd24(&mut d, TRIGGER_ADDRESS0, &mut v), 32768 % 2048);
        assert_eq!(rd24(&mut d, NOW_ADDRESS0, &mut v), 33268 % 2048);
        assert_eq!(rd24(&mut d, STOP_ADDRESS0, &mut v), 33267 % 2048);
        read_start(&mut d, &mut v);
        // The oldest sample left: 33268 - 2048.
        assert_eq!(read_bulk(&mut d, &mut v)[0], 31220);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn ready_holds_up_to_the_memory_size() {
        let c = |written| Capture { words: 2048, written: Some(written), trigger: 0, mask: 0xffff };
        assert!(c(2048).ready());
        assert!(!c(2049).ready());
        assert_eq!(c(2048).addresses(), (0, 0, 2047));
        // The empty capture: nothing written.
        assert_eq!(c(0).addresses(), (0, 0, 0));
        assert_eq!(c(0).word(0), 0);
    }

    #[test]
    fn triggers_on_the_counter() {
        let trigger = |regs: &[(u8, u8)], pre: u32| {
            let mut d = Zeroplus::new(DEFAULT_PID, None);
            let mut v = Violations::default();
            configure(&mut d, 0, pre, 0, &mut v);
            for (r, val) in regs {
                wr(&mut d, *r, *val, &mut v);
            }
            go(&mut d, &mut v);
            assert!(v.is_empty(), "{v:?}");
            match d.run {
                Run::Busy { cap, .. } => cap.written,
                other => panic!("{other:?}"),
            }
        };
        // No condition: at the trigger bar.
        assert_eq!(trigger(&[], 100), Some(100));
        // A0 high, A1 low: 1 mod 4, from 100.
        assert_eq!(trigger(&[(TRIGGER_STATUS0, 0b1001)], 100), Some(101));
        // A1 rising: 2 mod 4; falling: 0 mod 4 from 1; either: even.
        assert_eq!(trigger(&[(TRIGGER_EDGE, 0x41)], 3), Some(6));
        assert_eq!(trigger(&[(TRIGGER_EDGE, 0x81)], 0), Some(4));
        assert_eq!(trigger(&[(TRIGGER_EDGE, 0xc1)], 1), Some(2));
        // A channel high and low: never; high with its own falling: never.
        assert_eq!(trigger(&[(TRIGGER_STATUS0, 0b11)], 0), None);
        assert_eq!(trigger(&[(TRIGGER_STATUS0, 0b01), (TRIGGER_EDGE, 0x80)], 0), None);
    }

    #[test]
    fn a_trigger_on_a_32_channel_board_uses_the_top_channels() {
        let mut d = Zeroplus::new(0x700b, None);
        let mut v = Violations::default();
        configure(&mut d, 0, 0, 0, &mut v);
        wr(&mut d, TRIGGER_EDGE, 0x40 | 31, &mut v);
        go(&mut d, &mut v);
        assert!(v.is_empty(), "{v:?}");
        let Run::Busy { cap, .. } = d.run else { panic!() };
        assert_eq!(cap.written, Some(1 << 31));
        assert_eq!(cap.word(0), (1u32 << 31) - 2048);
    }

    #[test]
    fn a_trigger_that_cannot_fire_stays_busy() {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        let mut v = Violations::default();
        configure(&mut d, 0, 0, 10, &mut v);
        wr(&mut d, TRIGGER_STATUS0, 0b11, &mut v);
        go(&mut d, &mut v);
        for _ in 0..2 {
            assert_eq!(rd(&mut d, DEV_STATUS, &mut v), STATUS_BUSY | STATUS_READY);
        }
        for _ in 0..50 {
            assert_eq!(rd(&mut d, DEV_STATUS, &mut v), STATUS_BUSY);
        }
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_bus_reset_keeps_the_analyzer() {
        let mut d = done(0, 0, 10);
        let mut v = Violations::default();
        read_start(&mut d, &mut v);
        d.reset();
        assert!(d.read.is_none() && d.addr.is_none());
        assert_eq!(rd(&mut d, DEV_STATUS, &mut v), STATUS_READY);
        assert_eq!(rd24(&mut d, NOW_ADDRESS0, &mut v), 10);
    }

    #[test]
    fn a_mix_of_rates_is_accepted() {
        // The driver gave up midway through __analyzer_set_freq().
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        let mut v = Violations::default();
        configure(&mut d, 0, 0, 10, &mut v);
        wr(&mut d, FREQUENCY_REG1, 25, &mut v);
        go(&mut d, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_unknown_requests() {
        fires("not one of gl_usb.c's", |d, v| {
            assert_eq!(d.control(setup(CTRL_OUT, 0x0d, REQ_WRITEADDR, 1), &[0], v), Reply::Stall);
        });
        fires("not one of gl_usb.c's", |d, v| {
            d.control(setup(CTRL_IN, REQ_REG, REQ_WRITEADDR, 1), &[], v);
        });
        fires("the driver sends", |d, v| {
            let s = Setup { index: 1, ..setup(CTRL_OUT, REQ_REG, REQ_WRITEADDR, 1) };
            assert_eq!(d.control(s, &[0x21], v), Reply::Stall);
        });
        fires("the driver sends", |d, v| {
            d.control(setup(CTRL_OUT, REQ_REG, REQ_WRITEDATA, 2), &[0, 0], v);
        });
        fires("the driver sends", |d, v| {
            d.control(setup(CTRL_OUT, REQ_BULK, REQ_READBULK, 4), &[0; 4], v);
        });
    }

    #[test]
    fn flags_endpoints() {
        fires("the board has none", |d, v| {
            assert_eq!(d.out(2, &[0], v), Reply::Stall);
        });
        fires("the samples come from", |d, v| {
            assert_eq!(d.in_transfer(2, 512, v), Reply::Stall);
        });
    }

    #[test]
    fn flags_registers_out_of_use() {
        fires("not one the driver knows", |d, v| {
            select(d, 0x33, v);
        });
        fires("with no register selected", |d, v| {
            poke(d, 0, v);
        });
        fires("with no register selected", |d, v| {
            peek(d, v);
        });
        fires("not one the driver writes", |d, v| {
            wr(d, DEV_STATUS, 0, v);
        });
        fires("not one the driver reads", |d, v| {
            rd(d, MEMORY_LENGTH, v);
        });
        fires("without a complete capture", |d, v| {
            rd(d, NOW_ADDRESS0, v);
        });
    }

    #[test]
    fn flags_register_values() {
        let cases: &[(u8, u8, &str)] = &[
            (FREQUENCY_REG0, 3, "not a divider"),
            (FREQUENCY_REG1, 7, "not a multiplier"),
            (FREQUENCY_REG2, 3, "always writes 2"),
            (FREQUENCY_REG4, 2, "not a clock selection"),
            (MEMORY_LENGTH, 9, "no memory length code"),
            // 128K bytes are the LAP-C(16032)'s 32K samples, 256K are not.
            (MEMORY_LENGTH, 3, "more memory than"),
            (CLOCK_SOURCE, 3, "not 0"),
            (TRIGGER_STATUS0 + 4, 1, "a trigger on a channel"),
            (TRIGGER_STATUS0 + 7, 2, "a trigger on a channel"),
            (TRIGGER_EDGE, 0x20, "bits the driver never sets"),
            (TRIGGER_EDGE, 0x01, "bits the driver never sets"),
            (TRIGGER_EDGE, 0x40 | 16, "an edge on a channel"),
            (TRIGGER_COUNT0, 2, "one trigger only"),
            (TRIGGER_COUNT1, 1, "one trigger only"),
            (DONT_CARE_TRIGGERBAR, 0, "always writes 1"),
            (FILTER_ENABLE, 1, "no input filter"),
            (FILTER_STATUS + 7, 1, "no input filter"),
            (ENABLE_DELAY_TIME0, 1, "always writes 0"),
            (ENABLE_DELAY_TIME0 + 1, 1, "always writes 0"),
            (ENABLE_INSERT_DATA0 + 3, 0, "not the driver's 12 34 56 78"),
            (COMPRESSION_TYPE0, 2, "only COMPRESSION_NONE"),
            (COMPRESSION_TYPE0 + 1, 0x80, "only COMPRESSION_NONE"),
        ];
        for (r, val, want) in cases {
            fires(want, |d, v| wr(d, *r, *val, v));
        }
        // What the same registers take.
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        let mut v = Violations::default();
        wr(&mut d, MEMORY_LENGTH, 2, &mut v);
        wr(&mut d, TRIGGER_STATUS0 + 3, 0xff, &mut v);
        wr(&mut d, TRIGGER_EDGE, 0xc0 | 15, &mut v);
        assert!(v.is_empty(), "{v:?}");
        // The 32 channel board has the upper channels, and memory.
        let mut d = Zeroplus::new(0x700d, None);
        wr(&mut d, MEMORY_LENGTH, 8, &mut v);
        wr(&mut d, TRIGGER_STATUS0 + 7, 0x40, &mut v);
        wr(&mut d, TRIGGER_EDGE, 0x40 | 31, &mut v);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_the_order_of_a_capture() {
        fires("not one of the driver's", |d, v| wr(d, START_STATUS, 0x48, v));
        fires("GO without INIT", |d, v| wr(d, START_STATUS, 0x44, v));
        fires("registers never written", |d, v| {
            wr(d, START_STATUS, 0x42, v);
            wr(d, START_STATUS, 0x44, v);
        });
        fires("GO without INIT", |d, v| {
            configure(d, 0, 0, 10, v);
            wr(d, START_STATUS, 0xc1, v);
            wr(d, START_STATUS, 0x44, v);
        });
        fires("GO while a capture runs", |d, v| {
            configure(d, 0, 0, 10, v);
            go(d, v);
            wr(d, START_STATUS, 0x44, v);
        });
        fires("written while a capture runs", |d, v| {
            configure(d, 0, 0, 10, v);
            go(d, v);
            wr(d, MEMORY_LENGTH, 0, v);
        });
        fires("readout without a complete capture", |d, v| wr(d, START_STATUS, 0xf0, v));
        fires("without one", |d, v| wr(d, START_STATUS, 0xe0, v));
    }

    #[test]
    fn flags_the_readout() {
        fires("outside a memory readout", |d, v| {
            select(d, READ_RAM_STATUS, v);
        });
        fires("outside a memory readout", |d, v| {
            bulk_request(d, 2048, v);
        });
        fires("bulk IN without a bulk read request", |d, v| {
            assert_eq!(d.in_transfer(EP_DATA, 2048, v), Reply::Stall);
        });
        let during: &[(&str, Step)] = &[
            ("selected during the memory readout", |d, v| {
                select(d, MEMORY_LENGTH, v);
            }),
            ("during the memory readout", |d, v| wr(d, START_STATUS, 0xc0, v)),
            ("written during the memory readout", |d, v| {
                d.addr = Some(MEMORY_LENGTH);
                poke(d, 0, v);
            }),
            ("first four bytes", |d, v| {
                let p = [1, 0, 0, 0, 0, 8, 0, 0];
                d.control(setup(CTRL_OUT, REQ_BULK, REQ_READBULK, 8), &p, v);
            }),
            ("whole samples", |d, v| {
                bulk_request(d, 2047, v);
            }),
            ("whole samples", |d, v| {
                bulk_request(d, 0, v);
            }),
            ("past the 2048 words", |d, v| {
                bulk_request(d, 8196, v);
            }),
            ("previous one is unanswered", |d, v| {
                bulk_request(d, 2048, v);
                bulk_request(d, 2048, v);
            }),
            ("for a request of 2048", |d, v| {
                bulk_request(d, 2048, v);
                let Reply::Data(b) = d.in_transfer(EP_DATA, 1024, v) else { panic!() };
                assert_eq!(b.len(), 1024, "the answer stays within the transfer");
            }),
        ];
        for (want, f) in during {
            let mut d = done(0, 0, 10);
            let mut v = Violations::default();
            read_start(&mut d, &mut v);
            assert!(v.is_empty(), "{v:?}");
            fires_on(&mut d, want, f);
        }
        fires("before READ_RAM_STATUS", |d, v| {
            configure(d, 0, 0, 10, v);
            go(d, v);
            wait(d, v);
            wr(d, START_STATUS, 0xf0, v);
            bulk_request(d, 2048, v);
        });
    }

    #[test]
    fn stall_picks_a_kind_a_register_and_a_count() {
        let mut d = Zeroplus::new(DEFAULT_PID, None);
        d.faults.parse("stall", "wdata:0x30@2,addr:0x21").unwrap();
        let mut v = Violations::default();
        assert_eq!(select(&mut d, DEV_STATUS, &mut v), Reply::Stall);
        select(&mut d, FREQUENCY_REG1, &mut v);
        assert_eq!(poke(&mut d, 5, &mut v), Reply::ack());
        select(&mut d, FREQUENCY_REG0, &mut v);
        assert_eq!(poke(&mut d, 5, &mut v), Reply::ack());
        assert_eq!(poke(&mut d, 1, &mut v), Reply::Stall);
        // A stalled write has no effect.
        assert_eq!(d.reg(FREQUENCY_REG0), 5);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn a_refused_bulk_request_leaves_nothing_to_send() {
        let mut d = done(0, 0, 10);
        d.faults.parse("stall", "bulkreq@1").unwrap();
        let mut v = Violations::default();
        read_start(&mut d, &mut v);
        assert_eq!(bulk_request(&mut d, 2048, &mut v), Reply::Stall);
        assert_eq!(d.in_transfer(EP_DATA, 2048, &mut v), Reply::Nak);
        assert_eq!(read_bulk(&mut d, &mut v).len(), 512);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn bulk_faults_lose_their_data() {
        let mut d = done(0, 0, 10);
        d.faults.parse("stall", "bulk@1").unwrap();
        d.faults.parse("short", "2:101").unwrap_err();
        // Whole packets would not end the transfer.
        d.faults.parse("short", "2:1024").unwrap_err();
        d.faults.parse("short", "2:0").unwrap_err();
        d.faults.parse("short", "2").unwrap();
        assert_eq!(d.faults.short, Some((2, SHORT)));
        d.faults.parse("short", "2:64").unwrap();
        let mut v = Violations::default();
        read_start(&mut d, &mut v);
        bulk_request(&mut d, 2048, &mut v);
        assert_eq!(d.in_transfer(EP_DATA, 2048, &mut v), Reply::Stall);
        assert_eq!(read_bulk(&mut d, &mut v).len(), 16);
        assert_eq!(d.read.unwrap().at, 1024);
        assert_eq!(read_bulk(&mut d, &mut v).len(), 512);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn options() {
        assert!(build(&opts(&[("pid", "0x700b"), ("fill", "0"), ("warm", "1")])).is_ok());
        assert!(build(&opts(&[("pid", "7002"), ("serial", "X"), ("short", "1")])).is_ok());
        for bad in [
            ("pid", "0x1234"),
            ("warm", "2"),
            ("fill", "x"),
            ("nope", "1"),
            ("stall", "what"),
            ("stall", "addr:0x100"),
            ("stall", "addr@0"),
            ("short", "0"),
            ("stall_string", "x"),
        ] {
            assert!(build(&opts(&[bad])).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn stalls_only_the_named_string() {
        let mut d = Zeroplus::new(DEFAULT_PID, Some("S".into()));
        d.faults.parse("stall_string", "1").unwrap();
        assert_eq!(d.fail_standard(setup(0x80, 6, 0x0301, 255)), Some(StandardFault::Stall));
        assert_eq!(d.fail_standard(setup(0x80, 6, 0x0300, 255)), None);
        assert_eq!(d.fail_standard(setup(0x80, 6, 0x0100, 18)), None);
        assert_eq!(d.descriptors().device[16], 1);
        assert_eq!(Zeroplus::new(DEFAULT_PID, None).descriptors().device[16], 0);
    }

    #[test]
    fn memory_sizes_follow_protocol_c() {
        assert_eq!(memory_bytes(0), 8 * 1024);
        assert_eq!(memory_bytes(1), 64 * 1024);
        assert_eq!(memory_bytes(8), 8 << 20);
        assert_eq!(next_match(100, 3, 1), 101);
        assert_eq!(next_match(101, 3, 1), 101);
    }
}
