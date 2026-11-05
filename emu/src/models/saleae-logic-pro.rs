// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Saleae Logic Pro 16 (`21a9:1006`) as libsigrok's
//! `saleae-logic-pro` driver sees it: a Cypress FX3 that takes its
//! firmware from the host, an FPGA that takes its bitstream through
//! scrambled bulk commands, an authentication chip on the FX3's I2C bus
//! whose answers key the scrambler, and a digital sample stream.
//!
//! Every fact below is taken from the C driver (`api.c`, `protocol.c`,
//! `protocol.h`); no firmware source, datasheet or recording of the
//! device was at hand. What the driver does not pin down is marked
//! "assumed" (the model's choice) or "invented" (made-up content).
//! How sure: the ids, the two strings and the endpoint numbers are the
//! driver's and certain; that the board is an FX3 with an FPGA and a
//! crypto chip is the driver's reading (file names `-fx3.fw`,
//! `-fpga.bitstream`, the I2C packets); everything else is assumed.
//! The driver knows only `21a9:1006` and calls it "Logic Pro 16";
//! whether a Logic Pro 8 shares the id is not in it.
//!
//! **Before the firmware** the FX3 comes up as `21a9:1006` (the driver
//! matches only these ids in both states, `api.c` `scan()`), with no
//! strings by default (assumed; `scan_firmware()` takes any device
//! whose manufacturer and product are not `Saleae` / `Logic Pro`, or
//! whose strings cannot be read, for one without firmware; libusb
//! refuses a string index 0 without asking the device), one interface
//! without endpoints (assumed). `boot_strings=MANUFACTURER/PRODUCT`
//! gives the boot stage strings, so that each `strcmp()` of
//! `scan_firmware()` can fail; the real boot stage's strings are not
//! known. Its boot loader takes vendor request `0xa0` OUT on endpoint
//! 0 with the address's low half in `wValue` and its high half in
//! `wIndex`, at most 4 KiB per request (`upload_firmware()`,
//! `FW_MAX_PART_SIZE`); a request without data starts the firmware at
//! its address (`"Started firmware at 0x%x."`), which renumerates as
//! the firmware's device. The file `saleae-logicpro16-fx3.fw` is a list
//! of parts, each a 7 byte header (size LE16, address LE32, final flag)
//! and its data, the last one empty with the flag set. The stand-in
//! ([`fx3_image`]) is SHA-256 blocks (`tools/emu-firmware.sh`'s scheme)
//! in that container, at invented addresses; `tools/emu/saleae-logic-pro.sh`
//! writes it. The model holds the upload to it request by request.
//!
//! **The firmware's device**: the same ids, manufacturer `Saleae` and
//! product `Logic Pro` (`scan_firmware()`), no serial number (assumed;
//! the driver reads none), interface 0 (`dev_open()` claims it) with
//! bulk endpoints 1 OUT and 1 IN for commands (`transact()`) and 2 IN
//! for the samples (`dev_acquisition_start()`), `bcdDevice` 0 and 100
//! mA (assumed). The real board is a USB 3 device; the emulator offers
//! high speed only, so the endpoints have 512 byte packets (an
//! emulation artifact; the driver does not depend on the packet size).
//! A vendor request `0xa0` to the running firmware is stalled
//! (assumed): the driver sends it after a failed string read in the
//! scan, taking the running firmware for the boot loader.
//!
//! **The command channel** (`transact()`): a command is one bulk OUT
//! of 2 to 1024 bytes on endpoint 1, byte 0 a flag byte (0x00, or 0x20
//! for the reseed) and byte 1 the command, scrambled with `encrypt()`;
//! every command but the reseed is answered by one bulk IN on endpoint
//! 1, scrambled, which the host undoes with `decrypt()`. The driver
//! reads the answer's exact length, but the model sees that length
//! rounded up to whole 512 byte packets (the emulator hands a bulk IN
//! over as the device sees it on the bus, `docs/USB.md`), so it flags
//! only a read shorter than the answer; a longer answer than the
//! host's transfer is a babble, which the emulator reports itself. Both XOR the bytes with the 32-bit LFSR state
//! (byte `i` with bits `8 * (i % 4)` up), except bits 0x28 of a
//! command's byte 0, which go plain (so the device can see the reseed
//! flag whatever the state), and each call steps the LFSR
//! (`iterate_lfsr()`: `(lfsr & 0x1f) + 35` shifts). The model keeps the
//! same state: it unscrambles each command with the inverse of
//! `encrypt()` and scrambles its answers with `decrypt()`'s XOR,
//! stepping once for each, so a host that sends plain bytes after the
//! key is set, steps out of turn, or skips an answer sends commands
//! the model refuses. The reseed (`20 24 4b 35 8e`) zeroes the state,
//! and 0 is a fixed point of the LFSR: until the authentication every
//! command and answer goes plain. A command before the first reseed is
//! flagged.
//!
//! The authentication (`authenticate()`, run by every
//! `saleae_logic_pro_prepare()`): the FX3 wakes the crypto chip at I2C
//! address 0xc0 (`0x89`), then the host writes it a Random, two Nonce
//! and a Sign command and reads each answer (`0x87`, `0x88`). The chip
//! is modelled from those packets: `[0x03, count, opcode, param1,
//! param2 LE, data, CRC LE]`, with the CRC-16 (polynomial 0x8005, bits
//! in from the low end, initial 0) that reproduces all three CRCs the
//! driver hard codes (a unit test checks it); its answers are `[count,
//! data, CRC]`, the wake answer `04 11 33 43` (status 0x11, the byte
//! `wake_i2c()` checks; the CRC is the same function's). The opcodes
//! 0x1b, 0x16 and 0x41 (Random, Nonce, Sign of Atmel's
//! Crypto Authentication chips, an attribution from memory the model
//! does not depend on) answer 32, 32 and 64 bytes, which is what the
//! driver reads; their content is invented (SHA-256 of a counter), as
//! no key is known. The host then keys the LFSR with the first 28
//! bytes of the last nonce, as little endian words, XOR the Sign
//! answer's CRC. The FX3 relays the chip's answers and so knows them:
//! the model switches to that key once it has sent the Sign answer
//! with status 0x02, as the host switches once it has read it. A
//! failed authentication leaves both on the old state (assumed), and
//! nothing in the driver tells whether the device refuses to capture
//! without one, so the model does not.
//!
//! | byte | command | request after the 2 header bytes | answer |
//! |---|---|---|---|
//! | `0x8b` | firmware version | none | 128 bytes, a NUL terminated text (invented) |
//! | `0x07` | read EEPROM | `33 81`, address LE16, length LE16 | the bytes |
//! | `0x81` | read registers | count 1-30, the addresses | the values |
//! | `0x80` | write registers | count 1-30, (address, value) pairs | 1 byte, 0 (assumed) |
//! | `0x7e` | bitstream init | none | 1 byte, 0 = ok |
//! | `0x7f` | bitstream data | length LE16 1-1020, the bytes | 1 byte, 0 = ok |
//! | `0x86` | temperature | none | 1 byte, signed (invented: 37) |
//! | `0x89` | wake the I2C chip | none | 1 byte, 0 = ok |
//! | `0x87` | I2C write | `c0`, length 1-128, 0, the bytes | 1 byte, 2 = ok |
//! | `0x88` | I2C read | `c0`, length 1-128, 0 | status (2 = ok), the bytes |
//! | `0x01` | start capture | none | 2 bytes, `00 00` (assumed) |
//! | `0x02` | stop capture | none | 2 bytes, `00 00` (assumed) |
//!
//! `0x8b`, `0x86`, `0x07` are only logged or kept by the driver; the
//! EEPROM (256 bytes assumed) is invented except that the driver reads
//! the serial (8 bytes at 8, unused) and the "magic" (16 bytes at 16),
//! which it writes back to register 0x17 one byte at a time.
//!
//! **The FPGA** comes up blank; its scratch register 0x7f reads 0
//! (assumed; the driver reads it to decide whether to upload, `0xaa`
//! meaning configured, so `fpga=1` starts with the FPGA configured and
//! the upload is skipped). Any other register access to a blank FPGA
//! is flagged. `0x7e` starts an upload, `0x7f` chunks follow (1020
//! bytes each, only the last shorter), and the next other command ends
//! it: the bytes must be the stand-in `saleae-logicpro16-fpga.bitstream`.
//! The registers the driver names and the values it writes, which the
//! model takes as the device's rules (the meanings are the driver's
//! comments, most with a question mark):
//!
//! - 0x00 control: 0x00 then 0x80 "reset the ADC?", 0x02, 0x04, 0x08
//!   each followed by 0x00 three "resets?", 0x01 runs the capture; no
//!   other value.
//! - 0x03, 0x04, 0x05: ADC index, value LSB, MSB; written as one
//!   triple in one command (`write_adc()`), to the ADC registers the
//!   driver writes (0x0f, 0x11, 0x12, 0x25, 0x2a, 0x2b, 0x45, 0x46,
//!   0x50, 0x55, 0x56).
//! - 0x06, 0x07 digital channel mask; 0x08, 0x09 analog channel mask,
//!   which must be 0: the driver has no analog input, so the model has
//!   no analog stream; 0x0b the digital rate as 100 MHz / value
//!   (0x64, 0x32, 0x28, 0x0a, 0x04, 0x02), 0x14 its "pre-divider" (0x80
//!   at 25 MHz, 0x40 at 50 MHz, 0xff below); 0x0e the digital "mux
//!   rate", the number of channels; 0x0a, 0x0c, 0x0d 0, 0x12 4, 0x13
//!   0, 0x15 0 or 3 (3 for a capture), the only values the driver
//!   writes to them.
//! - 0x17 takes the 16 magic bytes, in order, after their EEPROM read.
//! - 0x40 status, 0x20 after a capture (`saleae_logic_pro_stop()`
//!   wants it); 0x41-0x48 read once each at the open, 0 (invented).
//! - 0x7f scratch: reads back what was written.
//!
//! LED registers 0x0f-0x11 exist in `protocol.c` but their only writer
//! is compiled out; they, and any register not listed, are flagged.
//!
//! **A capture**: the configuration above, `0x01`, then control 0x01;
//! control 0x00 and `0x02` end it. Samples come on endpoint 2 as
//! little endian 32-bit words, each holding 32 consecutive samples of
//! one channel, the first in bit 31, the words going round the enabled
//! channels in ascending order (`saleae_logic_pro_convert_data()`).
//! The word of samples `32 * b` to `32 * b + 31` of channel `c` is
//! `b * 0x9e3779b1 + c` modulo 2^32 ([`block_word`]), so no transfer
//! repeats the one before. Every bulk IN the host has submitted is
//! filled to its length in submission order (the host's submissions
//! pace the stream); the driver submits 512 of 16 KiB before the start.
//!
//! The driver sets no sample or time limit (`api.c` lists neither), so
//! sigrok-cli captures only with `--continuous`, which it ends at the
//! first key on standard input (`anykey.c`), at once when that is
//! `/dev/null`, as the harness gives it; the driver itself stops after
//! 1 s without a USB event (`dev_acquisition_handle()`, `api.c:391`),
//! which the case `lib idle stop` reaches through `libsigrok-run`, which
//! has no any-key stop. A
//! real board streams until it is stopped, so how much a capture reads
//! would depend on when the stop lands. As an emulation artifact the
//! FPGA therefore stops sending after `transfers=N` transfers (2 by
//! default). They all complete while the host waits for the answer to
//! the write of control 0x01 (`saleae_logic_pro_start()`): the emulator
//! fills the held transfers right after that write, ahead of the
//! answer, and libusb runs their callbacks before it returns the
//! answer, which is before sigrok-cli starts watching standard input.
//!
//! **Pedantic checks**: the FX3 upload against the stand-in, part by
//! part; requests to the boot loader other than `0xa0` OUT; commands
//! before the reseed, of the wrong length, with a wrong flag byte,
//! cookie, count, I2C address or length, an unknown command (what a
//! wrongly scrambled one decodes to), a command sent before the last
//! answer was read, an answer read with fewer bytes than it has; the bitstream
//! byte for byte and its chunking; the crypto chip's packets (word
//! address, count, CRC, opcode, mode, parameters, a command before the
//! wake, a Sign without a Nonce, a read of another length than the
//! answer); register access to a blank FPGA, to registers the driver
//! does not name, writes to read-only ones; the control values, the
//! ADC triples, the magic write back; a capture started without the
//! start command, or with a configuration outside the above; bulk IN
//! transfers that are not a multiple of 512 bytes.
//!
//! **Faults**, all off by default: `stall=NAME[:N]` STALLs the command
//! (every one, or only its Nth since the model's reset; a stalled
//! command has no effect and does not step the device's LFSR),
//! `stall_reply=` STALLs its answer (the device does not step for it),
//! `short_reply=` answers one byte short, `bad_status=` answers the
//! status byte of `init-bitstream`, `send-bitstream`, `wake-i2c`,
//! `write-i2c` or `read-i2c` with 0xff, with `NAME` one of `reseed`,
//! `fw-version`, `read-eeprom`, `read-reg`, `write-reg`,
//! `init-bitstream`, `send-bitstream`, `read-temp`, `wake-i2c`,
//! `write-i2c`, `read-i2c`, `start`, `stop` (comma lists). A one-shot
//! STALL is an emulation artifact: a real STALL halts the endpoint
//! until it is cleared. The host steps its LFSR before it knows
//! whether a transfer went through (`transact()`, `protocol.c:108`), so
//! a stalled command or a short answer once the key is set leaves the
//! two states apart; the cases use those faults before the
//! authentication only. `wake=0xNN` is the chip's wake status;
//! `scratch=0xNN` is what the scratch register reads after an upload
//! (an FPGA that did not come up); `capture_status=0xNN` the status
//! register after a capture; on endpoint 2, `halt_in=N` halts the
//! endpoint after N transfers of a capture (every later transfer
//! STALLs; the driver never clears a halt), `short_in=N` makes the
//! first transfer of each capture carry only N bytes; `stall_load=N`
//! has the boot loader STALL its Nth request; `stall_string=I` and
//! `short_string=I` fail the scan's reads of string descriptor I (as
//! in the fx2lafw model).
//!
//! **Emulation artifacts**, in one place: high speed where the board
//! has USB 3; the stream that stops after `transfers=N`; every
//! submitted transfer filled at once, in order; answers on endpoint 1
//! at once (the driver waits 100 ms after each crypto command, with a
//! "TODO: Poll instead", for a chip that needs time); the one-shot
//! STALLs; the invented contents (firmware version, temperature, EEPROM,
//! registers 0x41-0x48, the crypto chip's random numbers and signature)
//! and the stand-ins' part addresses.
//!
//! **What the model shows about the driver** (C line numbers of
//! libsigrok 0bc24877):
//!
//! - `scan()` filters by `conn=` only when `sr_usb_find()` found
//!   something (`api.c:219-236`): a `conn` that matches no device, or
//!   that `sr_usb_find()` refuses, selects every Logic Pro. With a board
//!   without firmware and `conn=BUS.ADDR`, the upload moves it to
//!   another address, so the scan lists it at the new one.
//! - `saleae_logic_pro_receive_data()` converts the whole 16 KiB buffer
//!   whatever the transfer's `actual_length` (`protocol.c:906`): a short
//!   or empty transfer passes on bytes the buffer held before, or that
//!   were never written, as samples.
//! - `upload_bitstream()` reports a failed scratch read back with the
//!   init answer `rsp[0]` instead of the value read (`protocol.c:540`),
//!   and `wake_i2c()` the count byte `i2c_rsp[0]` instead of the status
//!   `i2c_rsp[1]` (`protocol.c:341`).
//! - `saleae_logic_pro_prepare()` ignores the authentication and every
//!   write and returns `SR_OK` (`protocol.c:783-791`): a capture starts
//!   after a failed authentication, unscrambled.
//! - After the upload the scan sleeps a fixed 500 ms (`api.c:216`); a
//!   board that renumerates more slowly is not found. After a failed
//!   upload the board, still in its boot stage, is listed all the same
//!   and its open fails at the reseed.
//! - Read only, not shown by a case: `transact()` never frees `req_enc`
//!   (`protocol.c:107`), one leak per command; `upload_bitstream()`
//!   returns without freeing the file when the init request fails
//!   (`protocol.c:512-514`); `dev_acquisition_stop()` neither cancels
//!   nor frees the 512 transfers (`api.c:444-458`); `dev_open()` leaves
//!   the handle open when the claim or the init fails (`api.c:284-287`,
//!   `293`).

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    Descriptors, DeviceModel, Reply, Setup, Speed, StandardFault, VENDOR, Violations,
};
use crate::shells::ezusb;
use crate::{Error, Result};
use sha2::{Digest as _, Sha256};

/// The model name.
pub const NAME: &str = "saleae-logic-pro";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Saleae Logic Pro 16 (21a9:1006, FX3 boot loader, renumerates, FPGA \
                         bitstream and crypto chip over LFSR-scrambled EP1 commands); flashed=1, \
                         fpga=1, boot_strings=M/P, transfers=N; faults: stall=, stall_reply=, \
                         short_reply=, bad_status=NAME[:N], wake=, scratch=, capture_status=, \
                         halt_in=N, short_in=N, stall_load=N, stall_string=I, short_string=I";

/// Vendor and product id, before and after the upload (`api.c`).
pub const VID: u16 = 0x21a9;
/// The product id.
pub const PID: u16 = 0x1006;
/// The FX3 firmware: its name and the size of the stand-in's payload.
pub const FX3_FIRMWARE: (&str, usize) = ("saleae-logicpro16-fx3.fw", 10_000);
/// Where the stand-in's parts go and where it starts (invented).
pub const FX3_BASE: u32 = 0x4000_3000;
/// The FPGA bitstream and its stand-in's size (`tools/emu/saleae-logic-pro.firmware`).
pub const BITSTREAM: (&str, usize) = ("saleae-logicpro16-fpga.bitstream", 10_000);
/// The bulk packet size at high speed.
const PACKET: usize = 512;
/// The EEPROM size (assumed).
const EEPROM_SIZE: usize = 256;
/// The largest boot loader part (`FW_MAX_PART_SIZE`).
const PART_MAX: usize = 4096;
/// The largest bitstream chunk (`upload_bitstream_part()`).
const CHUNK_MAX: usize = 1020;
/// The reseed command, as `reseed()` sends it.
const RESEED: [u8; 5] = [0x20, 0x24, 0x4b, 0x35, 0x8e];
/// The crypto chip's I2C address (`read_i2c()`: "Fixed address").
const I2C_ADDRESS: u8 = 0xc0;
/// The firmware version text (invented).
const FW_VERSION: &str = "oxrok-emu Logic Pro 16 1.0";
/// The temperature in degrees Celsius (invented).
const TEMPERATURE: i8 = 37;

const CMD_START: u8 = 0x01;
const CMD_STOP: u8 = 0x02;
const CMD_READ_EEPROM: u8 = 0x07;
const CMD_INIT_BITSTREAM: u8 = 0x7e;
const CMD_SEND_BITSTREAM: u8 = 0x7f;
const CMD_WRITE_REG: u8 = 0x80;
const CMD_READ_REG: u8 = 0x81;
const CMD_READ_TEMP: u8 = 0x86;
const CMD_WRITE_I2C: u8 = 0x87;
const CMD_READ_I2C: u8 = 0x88;
const CMD_WAKE_I2C: u8 = 0x89;
const CMD_READ_FW_VER: u8 = 0x8b;
/// Not a command byte: the reseed's name for the faults.
const RESEED_ID: u8 = 0x20;

/// The fault names of the commands.
const COMMANDS: [(&str, u8); 13] = [
    ("reseed", RESEED_ID),
    ("fw-version", CMD_READ_FW_VER),
    ("read-eeprom", CMD_READ_EEPROM),
    ("read-reg", CMD_READ_REG),
    ("write-reg", CMD_WRITE_REG),
    ("init-bitstream", CMD_INIT_BITSTREAM),
    ("send-bitstream", CMD_SEND_BITSTREAM),
    ("read-temp", CMD_READ_TEMP),
    ("wake-i2c", CMD_WAKE_I2C),
    ("write-i2c", CMD_WRITE_I2C),
    ("read-i2c", CMD_READ_I2C),
    ("start", CMD_START),
    ("stop", CMD_STOP),
];

/// The commands whose answer is a status byte (`bad_status=`).
const STATUS_COMMANDS: [u8; 5] =
    [CMD_INIT_BITSTREAM, CMD_SEND_BITSTREAM, CMD_WAKE_I2C, CMD_WRITE_I2C, CMD_READ_I2C];

/// Registers.
const REG_CONTROL: u8 = 0x00;
const REG_ADC_IDX: u8 = 0x03;
const REG_ADC_LSB: u8 = 0x04;
const REG_ADC_MSB: u8 = 0x05;
const REG_DIG_MASK_LO: u8 = 0x06;
const REG_DIG_MASK_HI: u8 = 0x07;
const REG_ANA_MASK_LO: u8 = 0x08;
const REG_ANA_MASK_HI: u8 = 0x09;
const REG_DIG_RATE: u8 = 0x0b;
const REG_DIG_MUX: u8 = 0x0e;
const REG_PREDIV: u8 = 0x14;
const REG_15: u8 = 0x15;
const REG_MAGIC: u8 = 0x17;
const REG_STATUS: u8 = 0x40;
const REG_SCRATCH: u8 = 0x7f;
/// Registers the driver writes.
const WRITABLE: [u8; 19] = [
    0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x12, 0x13,
    0x14, 0x15, 0x17, 0x7f,
];
/// The values the driver writes to the registers whose meaning it
/// does not give: (register, values).
const FIXED: [(u8, &[u8]); 6] =
    [(0x0a, &[0]), (0x0c, &[0]), (0x0d, &[0]), (0x12, &[4]), (0x13, &[0]), (REG_15, &[0, 3])];
/// The ADC registers the driver writes.
const ADC_REGISTERS: [u8; 11] = [0x0f, 0x11, 0x12, 0x25, 0x2a, 0x2b, 0x45, 0x46, 0x50, 0x55, 0x56];
/// The digital rates: (Hz, 100 MHz divider, pre-divider).
const RATES: [(u64, u8, u8); 6] = [
    (1_000_000, 0x64, 0xff),
    (2_000_000, 0x32, 0xff),
    (2_500_000, 0x28, 0xff),
    (10_000_000, 0x0a, 0xff),
    (25_000_000, 0x04, 0x80),
    (50_000_000, 0x02, 0x40),
];

/// `iterate_lfsr()` of `protocol.c`.
pub fn iterate_lfsr(mut lfsr: u32) -> u32 {
    let max = (lfsr & 0x1f) + 34;
    for _ in 0..=max {
        lfsr = (lfsr >> 1) | ((lfsr ^ (lfsr >> 1) ^ (lfsr >> 21) ^ (lfsr >> 31)) << 31);
    }
    lfsr
}

/// The mask for byte `i` (`encrypt()`, `decrypt()`).
fn mask(lfsr: u32, i: usize) -> u8 {
    (lfsr >> (i % 4 * 8)) as u8
}

/// `encrypt()` of `protocol.c` with the state `lfsr`, before it steps.
pub fn encrypt(lfsr: u32, plain: &[u8]) -> Vec<u8> {
    plain
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let m = mask(lfsr, i);
            if i == 0 { (v & 0x28) | ((v ^ m) & !0x28) } else { v ^ m }
        })
        .collect()
}

/// `decrypt()` of `protocol.c` with the state `lfsr`: XOR every byte.
pub fn xor(lfsr: u32, data: &[u8]) -> Vec<u8> {
    data.iter().enumerate().map(|(i, &v)| v ^ mask(lfsr, i)).collect()
}

/// The crypto chip's CRC-16: polynomial 0x8005, initial 0, each byte
/// fed from its low bit, sent low byte first. It gives the three CRCs
/// `protocol.c` hard codes.
pub fn crc16(data: &[u8]) -> [u8; 2] {
    let mut crc: u16 = 0;
    for &b in data {
        for i in 0..8 {
            let bit = u16::from(b >> i & 1);
            let top = crc >> 15;
            crc <<= 1;
            if bit != top {
                crc ^= 0x8005;
            }
        }
    }
    crc.to_le_bytes()
}

/// A crypto chip answer: count, data, CRC.
fn frame(data: &[u8]) -> Vec<u8> {
    let mut f = vec![(data.len() + 3) as u8];
    f.extend_from_slice(data);
    let crc = crc16(&f);
    f.extend_from_slice(&crc);
    f
}

/// The key `authenticate()` computes from the nonce and the Sign
/// answer's CRC.
pub fn session_key(nonce: &[u8], sign_crc: [u8; 2]) -> u32 {
    let mut lfsr = 0u32;
    for (i, &b) in nonce.iter().take(28).enumerate() {
        lfsr ^= u32::from(b) << (8 * (i % 4));
    }
    lfsr ^ u32::from(u16::from_le_bytes(sign_crc))
}

/// The FX3 firmware stand-in: parts of at most 4 KiB of the SHA-256
/// blocks of [`FX3_FIRMWARE`], from [`FX3_BASE`] up, then the empty
/// final part that starts it, each with its 7 byte header.
/// `tools/emu/saleae-logic-pro.sh` writes the same bytes.
pub fn fx3_image() -> Vec<u8> {
    let mut out = Vec::new();
    for (address, data) in fx3_parts() {
        let final_part = data.is_empty();
        out.extend_from_slice(&(data.len() as u16).to_le_bytes());
        out.extend_from_slice(&address.to_le_bytes());
        out.push(u8::from(final_part));
        out.extend_from_slice(&data);
    }
    out
}

/// The parts of [`fx3_image`]: address and data, the last one empty.
fn fx3_parts() -> Vec<(u32, Vec<u8>)> {
    let payload = ezusb::standin(FX3_FIRMWARE.0, FX3_FIRMWARE.1);
    let mut parts: Vec<(u32, Vec<u8>)> = payload
        .chunks(PART_MAX)
        .enumerate()
        .map(|(i, c)| (FX3_BASE + (i * PART_MAX) as u32, c.to_vec()))
        .collect();
    parts.push((FX3_BASE, Vec::new()));
    parts
}

/// A fault aimed at a command: every one (`nth` `None`) or the Nth.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Shot {
    command: u8,
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
        let command = COMMANDS.iter().find(|(n, _)| *n == name).map(|(_, c)| *c).ok_or_else(|| {
            let names: Vec<&str> = COMMANDS.iter().map(|(n, _)| *n).collect();
            Error::Invalid(format!("{k}={name}: one of {}", names.join(", ")))
        })?;
        if k == "bad_status" && !STATUS_COMMANDS.contains(&command) {
            return Err(Error::Invalid(format!(
                "{k}={name}: only init-bitstream, send-bitstream, wake-i2c, write-i2c and \
                 read-i2c answer a status"
            )));
        }
        out.push(Shot { command, nth });
    }
    Ok(out)
}

fn hit(list: &[Shot], command: u8, count: u32) -> bool {
    list.iter().any(|s| s.command == command && s.nth.is_none_or(|n| n == count))
}

fn byte(k: &str, v: &str) -> Result<u8> {
    let parsed = match v.strip_prefix("0x") {
        Some(hex) => u8::from_str_radix(hex, 16).ok(),
        None => v.parse().ok(),
    };
    parsed.ok_or_else(|| Error::Invalid(format!("{k}={v}: a byte, 0 to 255 or 0x00 to 0xff")))
}

fn flag(k: &str, v: &str) -> Result<bool> {
    match v {
        "0" => Ok(false),
        "1" => Ok(true),
        _ => Err(Error::Invalid(format!("{k}={v}: 0 or 1"))),
    }
}

fn string_indexes(k: &str, v: &str) -> Result<Vec<u8>> {
    let mut list = Vec::new();
    for i in v.split(',').filter(|i| !i.is_empty()) {
        list.push(
            i.parse::<u8>()
                .ok()
                .filter(|i| *i <= 3)
                .ok_or_else(|| Error::Invalid(format!("{k}={i}: a string index, 0 to 3")))?,
        );
    }
    Ok(list)
}

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    stall: Vec<Shot>,
    stall_reply: Vec<Shot>,
    short_reply: Vec<Shot>,
    bad_status: Vec<Shot>,
    wake: Option<u8>,
    scratch: Option<u8>,
    capture_status: Option<u8>,
    halt_in: Option<u32>,
    short_in: Option<usize>,
    stall_load: Option<u32>,
    stall_string: Vec<u8>,
    short_string: Vec<u8>,
}

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let count = |v: &str| v.parse::<u32>().map_err(|_| Error::Invalid(format!("{k}={v}: a count")));
        match k {
            "stall" => self.stall = shots(k, v)?,
            "stall_reply" => self.stall_reply = shots(k, v)?,
            "short_reply" => self.short_reply = shots(k, v)?,
            "bad_status" => self.bad_status = shots(k, v)?,
            "wake" => self.wake = Some(byte(k, v)?),
            "scratch" => self.scratch = Some(byte(k, v)?),
            "capture_status" => self.capture_status = Some(byte(k, v)?),
            "halt_in" => self.halt_in = Some(count(v)?),
            "short_in" => self.short_in = Some(count(v)? as usize),
            "stall_load" => {
                self.stall_load = Some(
                    count(v).ok().filter(|n| *n > 0).ok_or_else(|| Error::Invalid(format!("{k}={v}: N from 1")))?,
                );
            }
            "stall_string" => self.stall_string = string_indexes(k, v)?,
            "short_string" => self.short_string = string_indexes(k, v)?,
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// Firmware parts, bitstream chunks and sample transfers.
pub fn log_rules(_: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules::digest_above(crate::txlog::HASH_ABOVE)
}

/// A board from the `-o` options: `flashed=0|1`, `fpga=0|1`,
/// `boot_strings=MANUFACTURER/PRODUCT`, `transfers=N`, and the faults
/// (see [`Faults`]).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut flashed = false;
    let mut fpga = false;
    let mut boot_strings = None;
    let mut transfers = 2u32;
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "flashed" => flashed = flag(k, v)?,
            "fpga" => fpga = flag(k, v)?,
            "boot_strings" => {
                let (m, p) = v.split_once('/').filter(|(m, p)| !m.is_empty() && !p.is_empty()).ok_or_else(
                    || Error::Invalid(format!("boot_strings={v}: MANUFACTURER/PRODUCT, both non-empty")),
                )?;
                boot_strings = Some((m.to_owned(), p.to_owned()));
            }
            "transfers" => {
                transfers = v.parse().map_err(|_| Error::Invalid(format!("transfers={v}: a count")))?;
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let mut model = LogicPro::new(flashed, fpga, boot_strings.as_ref());
    model.transfers = transfers;
    model.faults = faults;
    Ok(Box::new(model))
}

/// The descriptors of the boot stage: no endpoints, strings when given.
pub fn boot_descriptors(strings: Option<&(String, String)>) -> Descriptors {
    let body = descriptor::interface(0, 0, 0, [0xff, 0xff, 0xff], 0);
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor: VID,
        product: PID,
        device: 0x0000,
        strings: if strings.is_some() { [1, 2, 0] } else { [0, 0, 0] },
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: strings.map(|(m, p)| vec![m.clone(), p.clone()]).unwrap_or_default(),
    }
}

/// The descriptors of the running firmware.
pub fn firmware_descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 3, [0xff, 0xff, 0xff], 0));
    for address in [0x01, 0x81, 0x82] {
        body.extend_from_slice(&descriptor::endpoint(
            address,
            crate::model::BULK,
            PACKET as u16,
            0,
        ));
    }
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
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
        strings: vec!["Saleae".into(), "Logic Pro".into()],
    }
}

/// The EEPROM, invented.
pub fn eeprom() -> Vec<u8> {
    Sha256::digest(b"saleae-logic-pro emulated EEPROM")
        .iter()
        .copied()
        .cycle()
        .take(EEPROM_SIZE)
        .collect()
}

/// The FX3 boot loader: what it was sent.
#[derive(Clone, Debug, Default)]
struct Boot {
    /// The parts written: address and data.
    parts: Vec<(u32, Vec<u8>)>,
    /// Requests seen, for `stall_load`.
    requests: u32,
}

/// The crypto chip on the FX3's I2C bus.
#[derive(Clone, Debug, Default)]
struct Chip {
    awake: bool,
    /// The answer waiting to be read, and whether it is a Sign's.
    out: Option<(Vec<u8>, bool)>,
    /// The output of the last Nonce.
    nonce: Option<Vec<u8>>,
    /// Commands run, the source of the invented answers.
    counter: u32,
}

impl Chip {
    fn invented(&mut self, what: &str, n: usize) -> Vec<u8> {
        self.counter += 1;
        let mut out = Vec::new();
        let mut i = 0;
        while out.len() < n {
            out.extend_from_slice(&Sha256::digest(
                format!("saleae-logic-pro chip {what} {} {i}", self.counter).as_bytes(),
            ));
            i += 1;
        }
        out.truncate(n);
        out
    }

    /// One packet the host writes; the answer goes to `out`.
    fn write(&mut self, p: &[u8], v: &mut Violations) {
        if !self.awake {
            v.report("I2C write to the crypto chip before it was woken (0x89)");
        }
        self.out = None;
        if p.len() < 8 || p[0] != 0x03 {
            v.report(format!(
                "crypto chip packet {p:02x?}: word address 0x03, count, opcode, two parameters \
                 and a CRC at least"
            ));
            return;
        }
        let n = p.len();
        if usize::from(p[1]) != n - 1 {
            v.report(format!("crypto chip packet with count {} for {} bytes after it", p[1], n - 1));
            return;
        }
        let crc = crc16(&p[1..n - 2]);
        if p[n - 2..] != crc {
            v.report(format!(
                "crypto chip packet CRC {:02x?}; the bytes give {crc:02x?}",
                &p[n - 2..]
            ));
            return;
        }
        let (opcode, mode, param2, data) = (p[2], p[3], [p[4], p[5]], &p[6..n - 2]);
        let answer = match opcode {
            0x1b if mode == 0 && param2 == [0, 0] && data.is_empty() => {
                (self.invented("random", 32), false)
            }
            0x16 if mode == 0 && param2 == [0, 0] && data.len() == 20 => {
                let r = self.invented("nonce", 32);
                self.nonce = Some(r.clone());
                (r, false)
            }
            0x41 if mode == 0x80 && param2 == [0, 0] && data.is_empty() => {
                if self.nonce.is_none() {
                    v.report("Sign without a Nonce before it");
                }
                (self.invented("signature", 64), true)
            }
            _ => {
                v.report(format!(
                    "crypto chip command {opcode:#04x} mode {mode:#04x} parameter {param2:02x?} \
                     with {} data bytes: not Random (0x1b), Nonce (0x16, 20 bytes) or Sign \
                     (0x41, mode 0x80) as authenticate() sends them",
                    data.len()
                ));
                return;
            }
        };
        self.out = Some((frame(&answer.0), answer.1));
    }
}

/// What the FPGA holds.
#[derive(Clone, Debug)]
struct Fpga {
    configured: bool,
    /// The bitstream being uploaded, and whether a short chunk came.
    upload: Option<(Vec<u8>, bool)>,
    regs: [u8; 256],
    adc: Vec<(u8, u16)>,
    /// Magic bytes written back since the last magic EEPROM read.
    magic: Option<Vec<u8>>,
    magic_done: bool,
    /// The start command came since the last configuration change.
    armed: bool,
    stream: Option<Stream>,
}

/// A running capture on endpoint 2.
#[derive(Clone, Debug)]
struct Stream {
    channels: Vec<u8>,
    /// The block of 32 samples the next word belongs to.
    block: u64,
    /// The index in `channels` of the next word.
    at: usize,
    /// Transfers sent.
    sent: u32,
    halted: bool,
    short: Option<usize>,
}

impl Stream {
    fn word(&mut self) -> u32 {
        let w = block_word(self.block, self.channels[self.at]);
        self.at += 1;
        if self.at == self.channels.len() {
            self.at = 0;
            self.block += 1;
        }
        w
    }
}

/// The 32 samples of block `block` (samples `32 * block` on) of
/// channel `c`, the first in bit 31: `block * 0x9e3779b1 + c`, modulo
/// 2^32, a pattern that does not repeat from one transfer to the next.
pub fn block_word(block: u64, c: u8) -> u32 {
    (block as u32).wrapping_mul(0x9e37_79b1).wrapping_add(u32::from(c))
}

/// What the FX3 runs.
#[derive(Clone, Debug)]
enum Stage {
    Boot(Boot),
    Running,
}

/// The answer waiting for its bulk IN.
#[derive(Clone, Debug)]
struct Answer {
    command: u8,
    plain: Vec<u8>,
    /// A Sign answer read with status 2: the key after it.
    key: Option<u32>,
}

/// The emulated board.
pub struct LogicPro {
    boot: Descriptors,
    firmware: Descriptors,
    stage: Stage,
    renumerate: bool,
    eeprom: Vec<u8>,
    /// The LFSR state; `None` until the first reseed.
    lfsr: Option<u32>,
    answer: Option<Answer>,
    /// The key a Sign answer being prepared brings.
    pending_key: Option<u32>,
    chip: Chip,
    fpga: Fpga,
    transfers: u32,
    /// Commands seen per command byte, for the faults' Nth.
    counts: [u32; 256],
    faults: Faults,
}

impl LogicPro {
    /// A board; `flashed` starts it with the FX3 firmware running,
    /// `fpga` with the FPGA configured.
    pub fn new(flashed: bool, fpga: bool, boot_strings: Option<&(String, String)>) -> LogicPro {
        let mut regs = [0u8; 256];
        if fpga {
            regs[usize::from(REG_SCRATCH)] = 0xaa;
        }
        LogicPro {
            boot: boot_descriptors(boot_strings),
            firmware: firmware_descriptors(),
            stage: if flashed { Stage::Running } else { Stage::Boot(Boot::default()) },
            renumerate: false,
            eeprom: eeprom(),
            lfsr: None,
            answer: None,
            pending_key: None,
            chip: Chip::default(),
            fpga: Fpga {
                configured: fpga,
                upload: None,
                regs,
                adc: Vec::new(),
                magic: None,
                magic_done: false,
                armed: false,
                stream: None,
            },
            transfers: 2,
            counts: [0; 256],
            faults: Faults::default(),
        }
    }

    fn running(&self) -> bool {
        matches!(self.stage, Stage::Running)
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let Stage::Boot(boot) = &mut self.stage else { unreachable!() };
        if setup.kind() != VENDOR || setup.request != 0xa0 || setup.is_in() {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} to the FX3 boot loader; only 0xa0 OUT is \
                 the driver's",
                setup.request_type, setup.request, setup.value
            ));
            return Reply::Stall;
        }
        boot.requests += 1;
        if self.faults.stall_load == Some(boot.requests) {
            return Reply::Stall;
        }
        let address = u32::from(setup.value) | u32::from(setup.index) << 16;
        if data.len() > PART_MAX {
            v.report(format!("boot loader write of {} bytes; at most {PART_MAX}", data.len()));
        }
        boot.parts.push((address, data.to_vec()));
        if !data.is_empty() {
            return Reply::ack();
        }
        // The empty part starts the firmware: the upload must be the
        // stand-in's parts, in order.
        let want = fx3_parts();
        if boot.parts != want {
            let got: Vec<(u32, usize)> = boot.parts.iter().map(|(a, d)| (*a, d.len())).collect();
            let wanted: Vec<(u32, usize)> = want.iter().map(|(a, d)| (*a, d.len())).collect();
            v.report(format!(
                "FX3 firmware started after the parts (address, length) {got:x?}; the stand-in \
                 {} has {wanted:x?}{}",
                FX3_FIRMWARE.0,
                if got == wanted { " (same layout, other bytes)" } else { "" }
            ));
            boot.parts.clear();
            return Reply::ack();
        }
        self.stage = Stage::Running;
        self.renumerate = true;
        Reply::ack()
    }

    /// Ends a bitstream upload: the FPGA takes the design when the
    /// bytes are the stand-in.
    fn end_upload(&mut self, v: &mut Violations) {
        let Some((bytes, _)) = self.fpga.upload.take() else { return };
        if bytes != ezusb::standin(BITSTREAM.0, BITSTREAM.1) {
            v.report(format!(
                "FPGA upload of {} bytes that are not {} ({} bytes)",
                bytes.len(),
                BITSTREAM.0,
                BITSTREAM.1
            ));
            return;
        }
        self.fpga.configured = true;
        self.fpga.regs = [0; 256];
    }

    fn read_register(&mut self, address: u8, v: &mut Violations) -> u8 {
        if !self.fpga.configured {
            if address != REG_SCRATCH {
                v.report(format!(
                    "read of FPGA register {address:#04x} with no bitstream loaded (only the \
                     scratch register 0x7f is the driver's)"
                ));
            }
            return 0;
        }
        match address {
            REG_SCRATCH => {
                self.faults.scratch.unwrap_or(self.fpga.regs[usize::from(REG_SCRATCH)])
            }
            REG_STATUS => {
                if self.fpga.stream.is_some() {
                    v.report("status register read during a capture; the driver reads it after");
                }
                self.faults.capture_status.unwrap_or(0x20)
            }
            0x41..=0x48 => 0,
            _ => {
                v.report(format!("read of FPGA register {address:#04x}, not one the driver reads"));
                0
            }
        }
    }

    /// One `write_regs()` command: its (address, value) pairs.
    fn write_registers(&mut self, pairs: &[[u8; 2]], v: &mut Violations) {
        if !self.fpga.configured {
            v.report(format!("write of FPGA registers {pairs:02x?} with no bitstream loaded"));
            return;
        }
        let mut i = 0;
        while i < pairs.len() {
            let [address, value] = pairs[i];
            if address == REG_ADC_IDX {
                let triple = pairs.get(i..i + 3).map(|t| [t[1][0], t[2][0]]);
                if triple != Some([REG_ADC_LSB, REG_ADC_MSB]) {
                    v.report(format!(
                        "ADC index written without its value LSB and MSB after it in the same \
                         command: {:02x?}",
                        &pairs[i..]
                    ));
                    i += 1;
                    continue;
                }
                let adc_value = u16::from_le_bytes([pairs[i + 1][1], pairs[i + 2][1]]);
                if !ADC_REGISTERS.contains(&value) {
                    v.report(format!(
                        "ADC register {value:#04x} = {adc_value:#06x}: not one the driver writes"
                    ));
                }
                self.fpga.adc.push((value, adc_value));
                for p in &pairs[i..i + 3] {
                    self.fpga.regs[usize::from(p[0])] = p[1];
                }
                i += 3;
                continue;
            }
            if address == REG_ADC_LSB || address == REG_ADC_MSB {
                v.report(format!("ADC value register {address:#04x} written without the index first"));
                i += 1;
                continue;
            }
            self.write_register(address, value, v);
            i += 1;
        }
    }

    fn write_register(&mut self, address: u8, value: u8, v: &mut Violations) {
        if !WRITABLE.contains(&address) {
            v.report(format!(
                "write of {value:#04x} to FPGA register {address:#04x}, not one the driver writes"
            ));
            return;
        }
        if let Some((_, values)) = FIXED.iter().find(|(r, _)| *r == address)
            && !values.contains(&value)
        {
            v.report(format!(
                "FPGA register {address:#04x} = {value:#04x}; the driver writes only {values:02x?}"
            ));
        }
        match address {
            REG_CONTROL => self.write_control(value, v),
            REG_MAGIC => {
                let Some(m) = &mut self.fpga.magic else {
                    v.report("magic register 0x17 written before the magic EEPROM read");
                    return;
                };
                m.push(value);
                let want = &self.eeprom[0x10..0x20];
                if m.len() > 16 || m[..] != want[..m.len()] {
                    v.report(format!(
                        "magic write back {m:02x?}; the EEPROM's 16 bytes at 0x10 are {want:02x?}"
                    ));
                    self.fpga.magic = None;
                } else if m.len() == 16 {
                    self.fpga.magic_done = true;
                }
            }
            _ => {}
        }
        if address != REG_CONTROL {
            if (0x06..=0x15).contains(&address) {
                self.fpga.armed = false;
            }
            self.fpga.regs[usize::from(address)] = value;
        }
    }

    fn write_control(&mut self, value: u8, v: &mut Violations) {
        if ![0x00, 0x01, 0x02, 0x04, 0x08, 0x80].contains(&value) {
            v.report(format!(
                "control register 0x00 = {value:#04x}; the driver writes 0, 0x80, 0x02, 0x04, \
                 0x08 and 0x01"
            ));
        }
        let was = self.fpga.regs[usize::from(REG_CONTROL)] & 1 != 0;
        self.fpga.regs[usize::from(REG_CONTROL)] = value;
        let run = value & 1 != 0;
        if run && !was {
            self.start_stream(v);
        } else if !run {
            self.fpga.stream = None;
        }
    }

    /// Control 0x01: the capture runs with the configuration written.
    fn start_stream(&mut self, v: &mut Violations) {
        let r = &self.fpga.regs;
        if !self.fpga.armed {
            v.report("capture run (control 0x01) without the start command (0x01) after the setup");
        }
        if !self.fpga.magic_done {
            v.report("capture run before the 16 magic bytes were written back");
        }
        let mask = u16::from_le_bytes([r[usize::from(REG_DIG_MASK_LO)], r[usize::from(REG_DIG_MASK_HI)]]);
        let analog =
            u16::from_le_bytes([r[usize::from(REG_ANA_MASK_LO)], r[usize::from(REG_ANA_MASK_HI)]]);
        if analog != 0 {
            v.report(format!(
                "analog channel mask {analog:#06x}: the driver has no analog input, the model no \
                 analog stream"
            ));
        }
        if mask == 0 {
            v.report("capture run with no digital channel");
        }
        let mux = r[usize::from(REG_DIG_MUX)];
        if u32::from(mux) != mask.count_ones() {
            v.report(format!(
                "digital mux rate {mux} for the {} channels of mask {mask:#06x}",
                mask.count_ones()
            ));
        }
        let (div, prediv) = (r[usize::from(REG_DIG_RATE)], r[usize::from(REG_PREDIV)]);
        match RATES.iter().find(|(_, d, _)| *d == div) {
            None => v.report(format!(
                "digital rate divider {div:#04x}: not one of the driver's ({:02x?})",
                RATES.iter().map(|r| r.1).collect::<Vec<_>>()
            )),
            Some((hz, _, p)) if *p != prediv => v.report(format!(
                "pre-divider {prediv:#04x} at {hz} Hz; the driver writes {p:#04x} there"
            )),
            Some(_) => {}
        }
        if r[usize::from(REG_15)] != 3 {
            v.report(format!(
                "capture run with register 0x15 = {:#04x}; the driver sets 3 first",
                r[usize::from(REG_15)]
            ));
        }
        let channels: Vec<u8> = (0..16).filter(|c| mask >> c & 1 != 0).collect();
        if channels.is_empty() {
            return;
        }
        self.fpga.stream = Some(Stream {
            channels,
            block: 0,
            at: 0,
            sent: 0,
            halted: false,
            short: self.faults.short_in,
        });
    }

    /// One command, unscrambled; the plain answer.
    fn command(&mut self, cmd: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        let c = cmd[1];
        if c != CMD_SEND_BITSTREAM {
            self.end_upload(v);
        }
        let len_is = |n: usize, v: &mut Violations| {
            if cmd.len() == n {
                return true;
            }
            v.report(format!("command {c:#04x} of {} bytes; it has {n}", cmd.len()));
            false
        };
        match c {
            CMD_READ_FW_VER => {
                len_is(2, v);
                let mut a = FW_VERSION.as_bytes().to_vec();
                a.resize(128, 0);
                Some(a)
            }
            CMD_READ_EEPROM => self.read_eeprom(cmd, v),
            CMD_READ_REG => {
                let n = usize::from(cmd.get(2).copied().unwrap_or(0));
                if !(1..=30).contains(&n) {
                    v.report(format!("register read of {n} registers; 1 to 30"));
                    return None;
                }
                if !len_is(3 + n, v) {
                    return None;
                }
                Some(cmd[3..].iter().map(|&a| self.read_register(a, v)).collect())
            }
            CMD_WRITE_REG => {
                let n = usize::from(cmd.get(2).copied().unwrap_or(0));
                if !(1..=30).contains(&n) {
                    v.report(format!("register write of {n} registers; 1 to 30"));
                    return None;
                }
                if !len_is(3 + 2 * n, v) {
                    return None;
                }
                let pairs: Vec<[u8; 2]> = cmd[3..].chunks(2).map(|p| [p[0], p[1]]).collect();
                self.write_registers(&pairs, v);
                Some(vec![0])
            }
            CMD_INIT_BITSTREAM => {
                len_is(2, v);
                self.fpga.configured = false;
                self.fpga.stream = None;
                self.fpga.upload = Some((Vec::new(), false));
                Some(vec![0])
            }
            CMD_SEND_BITSTREAM => self.bitstream_chunk(cmd, v),
            CMD_READ_TEMP => {
                len_is(2, v);
                Some(vec![TEMPERATURE as u8])
            }
            CMD_WAKE_I2C => {
                len_is(2, v);
                self.chip.awake = true;
                self.chip.out = Some((
                    frame(&[self.faults.wake.unwrap_or(0x11)]),
                    false,
                ));
                Some(vec![0])
            }
            CMD_WRITE_I2C | CMD_READ_I2C => self.i2c(cmd, v),
            CMD_START | CMD_STOP => {
                len_is(2, v);
                if c == CMD_START {
                    if self.fpga.stream.is_some() {
                        v.report("start command during a capture");
                    }
                    self.fpga.armed = true;
                } else {
                    self.fpga.armed = false;
                    if self.fpga.stream.is_some() {
                        v.report("stop command with the capture running; control 0x00 comes first");
                        self.fpga.stream = None;
                    }
                }
                Some(vec![0, 0])
            }
            other => {
                v.report(format!(
                    "command {other:#04x} (unscrambled; {} bytes): not one of the Logic Pro's, \
                     or not scrambled with the LFSR state",
                    cmd.len()
                ));
                None
            }
        }
    }

    /// `0x07`: an EEPROM read.
    fn read_eeprom(&mut self, cmd: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        if cmd.len() != 8 {
            v.report(format!("EEPROM read command of {} bytes; it has 8", cmd.len()));
            return None;
        }
        if cmd[2..4] != [0x33, 0x81] {
            v.report(format!(
                "EEPROM read with {:#04x} {:#04x}; read_eeprom() sends 0x33 0x81",
                cmd[2], cmd[3]
            ));
            return None;
        }
        let at = usize::from(u16::from_le_bytes([cmd[4], cmd[5]]));
        let n = usize::from(u16::from_le_bytes([cmd[6], cmd[7]]));
        if n == 0 || n > 128 || at + n > EEPROM_SIZE {
            v.report(format!(
                "EEPROM read of {n} bytes at {at:#x}: 1 to 128 within the {EEPROM_SIZE} bytes"
            ));
            return None;
        }
        if (at, n) == (0x10, 16) {
            self.fpga.magic = Some(Vec::new());
            self.fpga.magic_done = false;
        }
        Some(self.eeprom[at..at + n].to_vec())
    }

    /// `0x7f`: a bitstream chunk.
    fn bitstream_chunk(&mut self, cmd: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        let n = cmd.get(2..4).map_or(0, |b| usize::from(u16::from_le_bytes([b[0], b[1]])));
        if !(1..=CHUNK_MAX).contains(&n) {
            v.report(format!("bitstream chunk of {n} bytes; 1 to {CHUNK_MAX}"));
            return None;
        }
        if cmd.len() != 4 + n {
            v.report(format!("bitstream chunk of {n} bytes in a command of {}", cmd.len()));
            return None;
        }
        match &mut self.fpga.upload {
            Some((bytes, short)) => {
                if *short {
                    v.report("bitstream chunk after a short one; only the last is short");
                }
                *short |= n < CHUNK_MAX;
                bytes.extend_from_slice(&cmd[4..]);
            }
            None => v.report("bitstream data without the init (0x7e)"),
        }
        Some(vec![0])
    }

    /// `0x87` and `0x88`: the crypto chip.
    fn i2c(&mut self, cmd: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        let c = cmd[1];
        if cmd.len() < 5 || cmd[2] != I2C_ADDRESS || cmd[4] != 0 {
            v.report(format!(
                "I2C command {c:#04x} {:02x?}: address 0xc0, a length, 0",
                &cmd[2..]
            ));
            return None;
        }
        let n = usize::from(cmd[3]);
        if !(1..=128).contains(&n) {
            v.report(format!("I2C transfer of {n} bytes; 1 to 128"));
            return None;
        }
        if c == CMD_WRITE_I2C {
            if cmd.len() != 5 + n {
                v.report(format!("I2C write of {n} bytes in a command of {}", cmd.len()));
                return None;
            }
            self.chip.write(&cmd[5..], v);
            return Some(vec![2]);
        }
        if cmd.len() != 5 {
            v.report(format!("I2C read command of {} bytes; it has 5", cmd.len()));
            return None;
        }
        let Some((out, sign)) = self.chip.out.take() else {
            v.report("I2C read with no crypto chip answer due");
            return Some(vec![2; 1 + n]);
        };
        if n != out.len() {
            v.report(format!("I2C read of {n} bytes; the chip's answer has {}", out.len()));
        }
        let mut a = vec![2];
        a.extend(out.iter().copied().chain(std::iter::repeat(0xff)).take(n));
        if sign {
            let nonce = self.chip.nonce.clone().unwrap_or_default();
            let key = session_key(&nonce, [out[65], out[66]]);
            self.pending_key = Some(key);
        }
        Some(a)
    }

    fn ep1_out(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if data.len() < 2 || data.len() > 1024 {
            v.report(format!("command of {} bytes; 2 to 1024", data.len()));
            return Reply::Stall;
        }
        if let Some(a) = self.answer.take() {
            v.report(format!("command sent while the answer to {:#04x} was not read", a.command));
        }
        if data[0] & 0x20 != 0 {
            // The reseed: its bytes go plain, as the host zeroes its
            // state first.
            self.counts[usize::from(RESEED_ID)] += 1;
            if hit(&self.faults.stall, RESEED_ID, self.counts[usize::from(RESEED_ID)]) {
                return Reply::Stall;
            }
            if data != RESEED {
                v.report(format!("reseed {data:02x?}; reseed() sends {RESEED:02x?}"));
            }
            self.end_upload(v);
            self.lfsr = Some(0);
            return Reply::ack();
        }
        let Some(lfsr) = self.lfsr else {
            v.report("command before the first reseed: the scrambler state is unknown");
            return Reply::Stall;
        };
        // encrypt() undoes itself, bits 0x28 of byte 0 plain included.
        let cmd = encrypt(lfsr, data);
        let c = cmd[1];
        self.counts[usize::from(c)] += 1;
        let count = self.counts[usize::from(c)];
        if hit(&self.faults.stall, c, count) {
            return Reply::Stall;
        }
        self.lfsr = Some(iterate_lfsr(lfsr));
        if cmd[0] != 0 {
            v.report(format!(
                "command {c:#04x} with byte 0 = {:#04x} (unscrambled); transact()'s callers send 0",
                cmd[0]
            ));
        }
        let plain = self.command(&cmd, v);
        let key = self.pending_key.take();
        if let Some(mut plain) = plain {
            if STATUS_COMMANDS.contains(&c) && hit(&self.faults.bad_status, c, count) {
                plain[0] = 0xff;
            }
            let key = key.filter(|_| plain[0] == 2);
            self.answer = Some(Answer { command: c, plain, key });
        }
        Reply::ack()
    }

    fn ep1_in(&mut self, max: usize, v: &mut Violations) -> Reply {
        let Some(a) = self.answer.take() else {
            // No answer due: the FX3 has nothing to send.
            return Reply::Nak;
        };
        // A longer read is the rounding to whole packets; an answer
        // longer than the host's own transfer the emulator reports as a
        // babble.
        if max < a.plain.len() {
            v.report(format!(
                "answer to {:#04x} read with {max} bytes; it has {}",
                a.command,
                a.plain.len()
            ));
        }
        let count = self.counts[usize::from(a.command)];
        if hit(&self.faults.stall_reply, a.command, count) {
            return Reply::Stall;
        }
        let lfsr = self.lfsr.unwrap_or(0);
        let mut out = xor(lfsr, &a.plain);
        if hit(&self.faults.short_reply, a.command, count) {
            out.pop();
        }
        self.lfsr = Some(a.key.unwrap_or_else(|| iterate_lfsr(lfsr)));
        Reply::Data(out)
    }

    fn ep2_in(&mut self, max: usize, v: &mut Violations) -> Reply {
        let limit = self.transfers;
        let halt = self.faults.halt_in;
        let Some(s) = &mut self.fpga.stream else {
            return Reply::Nak;
        };
        if s.halted {
            return Reply::Stall;
        }
        if halt == Some(s.sent) {
            s.halted = true;
            return Reply::Stall;
        }
        if s.sent >= limit {
            return Reply::Nak;
        }
        if !max.is_multiple_of(PACKET) {
            v.report(format!(
                "bulk IN transfer of {max} bytes; the driver asks for multiples of {PACKET}"
            ));
        }
        s.sent += 1;
        let len = s.short.take().map_or(max, |n| n.min(max));
        let mut data = Vec::with_capacity(len + 4);
        while data.len() < len {
            data.extend_from_slice(&s.word().to_le_bytes());
        }
        data.truncate(len);
        Reply::Data(data)
    }
}

impl DeviceModel for LogicPro {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        if self.running() { &self.firmware } else { &self.boot }
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if !self.running() {
            return self.boot_control(setup, data, v);
        }
        if setup.kind() == VENDOR && setup.request == 0xa0 && !setup.is_in() {
            // scan() takes a board whose strings it cannot read for one
            // without firmware: the running firmware has no boot
            // loader (assumed).
            return Reply::Stall;
        }
        v.report(format!(
            "control request {:#04x} {:#04x} value {:#06x}: the firmware takes its commands on \
             endpoint 1",
            setup.request_type, setup.request, setup.value
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != 1 {
            v.report(format!("OUT transfer to endpoint {ep}; the commands go to 1"));
            return Reply::Stall;
        }
        self.ep1_out(data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        match ep {
            1 => self.ep1_in(max, v),
            2 => self.ep2_in(max, v),
            _ => {
                v.report(format!("IN transfer from endpoint {ep}"));
                Reply::Stall
            }
        }
    }

    fn reset(&mut self) {
        // A new attach ends a capture and drops an unread answer; the
        // firmware, the FPGA design and the scrambler state stay, as
        // the board keeps power.
        self.fpga.stream = None;
        self.answer = None;
    }

    fn take_renumeration(&mut self) -> bool {
        std::mem::take(&mut self.renumerate)
    }

    /// `stall_string` and `short_string`: `GET_DESCRIPTOR` of a string.
    fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
        const GET_DESCRIPTOR: u8 = 6;
        if setup.request_type != 0x80
            || setup.request != GET_DESCRIPTOR
            || setup.value >> 8 != u16::from(descriptor::STRING)
        {
            return None;
        }
        let index = (setup.value & 0xff) as u8;
        if self.faults.stall_string.contains(&index) {
            Some(StandardFault::Stall)
        } else if self.faults.short_string.contains(&index) {
            Some(StandardFault::Short(2))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;

    const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

    fn load(address: u32, length: u16) -> Setup {
        Setup {
            request_type: 0x40,
            request: 0xa0,
            value: address as u16,
            index: (address >> 16) as u16,
            length,
        }
    }

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

    fn nonce_packet() -> Vec<u8> {
        let mut p = vec![3, 0x1b, 0x16, 0, 0, 0];
        p.extend_from_slice(&[0; 20]);
        p.extend_from_slice(&[0x7d, 0xe0]);
        p
    }

    /// The host side of `transact()`: its LFSR state and the exchange.
    struct Host {
        lfsr: u32,
    }

    impl Host {
        fn reseed(&mut self, dev: &mut Device, v: &mut Violations) {
            self.lfsr = 0;
            assert_eq!(dev.transfer_out(1, &encrypt(0, &RESEED), v), Reply::ack());
        }

        /// One command and its answer of `rsp` bytes (1 when 0).
        fn transact(
            &mut self,
            dev: &mut Device,
            req: &[u8],
            rsp: usize,
            v: &mut Violations,
        ) -> Reply {
            let enc = encrypt(self.lfsr, req);
            self.lfsr = iterate_lfsr(self.lfsr);
            let r = dev.transfer_out(1, &enc, v);
            if r != Reply::ack() {
                return r;
            }
            match dev.transfer_in(1, rsp.max(1), v) {
                Reply::Data(d) => {
                    let plain = xor(self.lfsr, &d);
                    self.lfsr = iterate_lfsr(self.lfsr);
                    Reply::Data(plain)
                }
                other => other,
            }
        }

        fn data(&mut self, dev: &mut Device, req: &[u8], rsp: usize, v: &mut Violations) -> Vec<u8> {
            match self.transact(dev, req, rsp, v) {
                Reply::Data(d) => d,
                other => panic!("{req:02x?}: {other:?}"),
            }
        }

        fn write(&mut self, dev: &mut Device, regs: &[(u8, u8)], v: &mut Violations) {
            let mut req = vec![0, CMD_WRITE_REG, regs.len() as u8];
            for (a, b) in regs {
                req.extend_from_slice(&[*a, *b]);
            }
            self.data(dev, &req, 0, v);
        }

        fn read(&mut self, dev: &mut Device, address: u8, v: &mut Violations) -> u8 {
            self.data(dev, &[0, CMD_READ_REG, 1, address], 1, v)[0]
        }

        fn adc(&mut self, dev: &mut Device, address: u8, value: u16, v: &mut Violations) {
            let [l, h] = value.to_le_bytes();
            self.write(dev, &[(REG_ADC_IDX, address), (REG_ADC_LSB, l), (REG_ADC_MSB, h)], v);
        }

        fn eeprom(&mut self, dev: &mut Device, at: u16, n: u16, v: &mut Violations) -> Vec<u8> {
            let [al, ah] = at.to_le_bytes();
            let [nl, nh] = n.to_le_bytes();
            self.data(dev, &[0, CMD_READ_EEPROM, 0x33, 0x81, al, ah, nl, nh], usize::from(n), v)
        }

        fn bitstream(&mut self, dev: &mut Device, v: &mut Violations) {
            assert_eq!(self.data(dev, &[0, CMD_INIT_BITSTREAM], 1, v), [0]);
            for c in ezusb::standin(BITSTREAM.0, BITSTREAM.1).chunks(CHUNK_MAX) {
                let mut req = vec![0, CMD_SEND_BITSTREAM];
                req.extend_from_slice(&(c.len() as u16).to_le_bytes());
                req.extend_from_slice(c);
                assert_eq!(self.data(dev, &req, 1, v), [0]);
            }
        }

        /// `saleae_logic_pro_init()`.
        fn init(&mut self, dev: &mut Device, v: &mut Violations) {
            self.reseed(dev, v);
            let fw = self.data(dev, &[0, CMD_READ_FW_VER], 128, v);
            assert!(fw.starts_with(FW_VERSION.as_bytes()));
            self.eeprom(dev, 8, 8, v);
            if self.read(dev, REG_SCRATCH, v) != 0xaa {
                self.bitstream(dev, v);
                self.write(dev, &[(REG_SCRATCH, 0xaa)], v);
                assert_eq!(self.read(dev, REG_SCRATCH, v), 0xaa);
            }
            self.write(dev, &[(0, 0)], v);
            self.write(dev, &[(0, 0x80)], v);
            for (a, val) in [
                (0x11, 0x0444),
                (0x12, 0x0777),
                (0x25, 0),
                (0x45, 0),
                (0x2a, 0x1111),
                (0x2b, 0x1111),
                (0x46, 4),
                (0x50, 0),
                (0x55, 0x20),
                (0x56, 0),
            ] {
                self.adc(dev, a, val, v);
            }
            self.write(dev, &[(REG_15, 0)], v);
            self.adc(dev, 0x0f, 0x0100, v);
            for b in [2, 0, 4, 0, 8, 0] {
                self.write(dev, &[(0, b)], v);
            }
            for a in 0x41..=0x48 {
                self.read(dev, a, v);
            }
            for b in self.eeprom(dev, 0x10, 16, v) {
                self.write(dev, &[(REG_MAGIC, b)], v);
            }
            assert_eq!(self.data(dev, &[0, CMD_READ_TEMP], 1, v), [TEMPERATURE as u8]);
        }

        fn i2c_write(&mut self, dev: &mut Device, p: &[u8], v: &mut Violations) -> u8 {
            let mut req = vec![0, CMD_WRITE_I2C, 0xc0, p.len() as u8, 0];
            req.extend_from_slice(p);
            self.data(dev, &req, 1, v)[0]
        }

        fn i2c_read(&mut self, dev: &mut Device, n: u8, v: &mut Violations) -> Vec<u8> {
            self.data(dev, &[0, CMD_READ_I2C, 0xc0, n, 0], 1 + usize::from(n), v)
        }

        /// `authenticate()`; the key it computes.
        fn authenticate(&mut self, dev: &mut Device, v: &mut Violations) -> u32 {
            assert_eq!(self.data(dev, &[0, CMD_WAKE_I2C], 1, v), [0]);
            assert_eq!(self.i2c_read(dev, 4, v), [2, 4, 0x11, 0x33, 0x43]);
            assert_eq!(self.i2c_write(dev, &[3, 7, 0x1b, 0, 0, 0, 0x24, 0xcd], v), 2);
            self.i2c_read(dev, 35, v);
            let mut nonce = Vec::new();
            for _ in 0..2 {
                assert_eq!(self.i2c_write(dev, &nonce_packet(), v), 2);
                nonce = self.i2c_read(dev, 35, v)[2..34].to_vec();
            }
            assert_eq!(self.i2c_write(dev, &[3, 7, 0x41, 0x80, 0, 0, 0x28, 0x05], v), 2);
            let sig = self.i2c_read(dev, 67, v);
            let key = session_key(&nonce, [sig[66], sig[67]]);
            self.lfsr = key;
            key
        }

        /// `saleae_logic_pro_prepare()` and `_start()` at 10 MHz.
        fn capture(&mut self, dev: &mut Device, mask: u16, v: &mut Violations) {
            self.authenticate(dev, v);
            self.write(dev, &[(REG_15, 3)], v);
            self.write(dev, &[(3, 0x0f), (4, 0), (5, 0)], v);
            let [ml, mh] = mask.to_le_bytes();
            self.write(
                dev,
                &[
                    (0, 0),
                    (8, 0),
                    (9, 0),
                    (6, ml),
                    (7, mh),
                    (0x0a, 0),
                    (0x0b, 0x0a),
                    (0x0c, 0),
                    (0x0d, 0),
                    (0x0e, mask.count_ones() as u8),
                    (0x12, 4),
                    (0x13, 0),
                    (0x14, 0xff),
                ],
                v,
            );
            assert_eq!(self.data(dev, &[0, CMD_START], 2, v), [0, 0]);
            self.write(dev, &[(0, 1)], v);
        }

        fn stop(&mut self, dev: &mut Device, v: &mut Violations) -> u8 {
            self.write(dev, &[(0, 0)], v);
            self.data(dev, &[0, CMD_STOP], 2, v);
            self.read(dev, REG_STATUS, v)
        }
    }

    #[test]
    fn the_crc_and_the_scrambler_are_protocol_c_s() {
        // The three CRCs the driver hard codes, and the wake answer.
        assert_eq!(crc16(&[7, 0x1b, 0, 0, 0]), [0x24, 0xcd]);
        assert_eq!(crc16(&nonce_packet()[1..26]), [0x7d, 0xe0]);
        assert_eq!(crc16(&[7, 0x41, 0x80, 0, 0]), [0x28, 0x05]);
        assert_eq!(frame(&[0x11]), [4, 0x11, 0x33, 0x43]);
        // 0 is a fixed point; one step of 1 by hand: max is
        // (1 & 0x1f) + 34 = 35, so 36 shifts.
        assert_eq!(iterate_lfsr(0), 0);
        let mut l = 1u32;
        for _ in 0..36 {
            l = (l >> 1) | ((l ^ (l >> 1) ^ (l >> 21) ^ (l >> 31)) << 31);
        }
        assert_eq!(iterate_lfsr(1), l);
        // encrypt() keeps bits 0x28 of byte 0 and undoes itself.
        let plain = [0xffu8, 1, 2, 3, 4, 5];
        let enc = encrypt(0x1234_5678, &plain);
        assert_eq!(enc[0] & 0x28, 0x28);
        assert_eq!(enc[1], 1 ^ 0x56);
        assert_eq!(enc[4], 4 ^ 0x78);
        assert_eq!(encrypt(0x1234_5678, &enc), plain);
        assert_eq!(xor(7, &xor(7, &plain)), plain);
        // authenticate(): 28 bytes as little endian words, XOR the CRC.
        let mut n = [0u8; 32];
        n[0] = 1;
        n[7] = 0x80;
        n[28] = 0xff;
        assert_eq!(session_key(&n, [0x10, 0x20]), 0x8000_0001 ^ 0x2010);
    }

    #[test]
    fn the_standins_are_those_of_the_firmware_list_and_the_script() {
        let list = include_str!("../../../tools/emu/saleae-logic-pro.firmware");
        let line = list
            .lines()
            .find(|l| l.split_whitespace().next() == Some(BITSTREAM.0))
            .expect("the bitstream is in tools/emu/saleae-logic-pro.firmware");
        assert_eq!(line.split_whitespace().nth(1).unwrap().parse::<usize>().unwrap(), BITSTREAM.1);
        // The FX3 container: three parts and the empty final one.
        let image = fx3_image();
        assert_eq!(image.len(), FX3_FIRMWARE.1 + 4 * 7);
        assert_eq!(&image[..7], &[0x00, 0x10, 0x00, 0x30, 0x00, 0x40, 0]);
        assert_eq!(&image[image.len() - 7..], &[0, 0, 0x00, 0x30, 0x00, 0x40, 1]);
        let script = include_str!("../../../tools/emu/saleae-logic-pro.sh");
        assert!(script.contains("10000, 0x40003000") && script.contains(FX3_FIRMWARE.0));
    }

    #[test]
    fn uploads_renumerates_authenticates_and_streams() {
        let mut dev = Device::new(build(&[]).unwrap());
        let mut v = Violations::default();
        assert!(dev.descriptors().strings.is_empty(), "the boot stage has no strings");
        dev.control(SET_CONFIG, &[], &mut v);
        for (address, data) in fx3_parts() {
            assert_eq!(dev.control(load(address, data.len() as u16), &data, &mut v), Reply::ack());
        }
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        dev.reset();
        assert_eq!(dev.descriptors().strings, ["Saleae", "Logic Pro"]);
        dev.control(SET_CONFIG, &[], &mut v);
        let mut host = Host { lfsr: 0 };
        host.init(&mut dev, &mut v);
        assert!(v.is_empty(), "{v:?}");
        host.capture(&mut dev, 0x8001, &mut v);
        assert_ne!(host.lfsr, 0, "keyed");
        assert!(v.is_empty(), "{v:?}");
        let Reply::Data(d) = dev.transfer_in(2, 16384, &mut v) else { panic!() };
        assert_eq!(d.len(), 16384);
        let word =
            |i: usize| u32::from_le_bytes([d[4 * i], d[4 * i + 1], d[4 * i + 2], d[4 * i + 3]]);
        assert_eq!([word(0), word(1), word(2), word(3)], [0, 15, 0x9e37_79b1, 0x9e37_79c0]);
        assert!(matches!(dev.transfer_in(2, 16384, &mut v), Reply::Data(_)));
        assert_eq!(dev.transfer_in(2, 16384, &mut v), Reply::Nak, "transfers=2 by default");
        assert_eq!(host.stop(&mut dev, &mut v), 0x20);
        assert!(v.is_empty(), "{v:?}");
        // A second capture on the same open: the authentication runs
        // scrambled with the first key.
        host.capture(&mut dev, 1, &mut v);
        assert_eq!(host.stop(&mut dev, &mut v), 0x20);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_a_wrong_upload_and_requests_the_boot_loader_lacks() {
        let mut v = Violations::default();
        let mut dev = Device::new(build(&[]).unwrap());
        dev.control(SET_CONFIG, &[], &mut v);
        let parts = fx3_parts();
        dev.control(load(parts[1].0, 4096), &parts[1].1, &mut v);
        dev.control(load(FX3_BASE, 0), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "parts out of order");
        assert!(!dev.take_renumeration());
        dev.control(Setup { request_type: 0xc0, ..load(0, 4) }, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "a read through the boot loader");
        dev.control(load(0, 4097), &[0; 4097], &mut v);
        assert_eq!(v.drain().len(), 1, "a part over 4 KiB");
        let mut dev = Device::new(build(&opts(&[("stall_load", "2")])).unwrap());
        dev.control(SET_CONFIG, &[], &mut v);
        assert_eq!(dev.control(load(parts[0].0, 4096), &parts[0].1, &mut v), Reply::ack());
        assert_eq!(dev.control(load(parts[1].0, 4096), &parts[1].1, &mut v), Reply::Stall);
        assert!(v.is_empty(), "a fault is the device's: {v:?}");
        // The running firmware STALLs the boot loader's request unflagged.
        let mut dev = board(&[]);
        assert_eq!(dev.control(load(0, 1), &[0], &mut v), Reply::Stall);
        assert!(v.is_empty(), "{v:?}");
        let other = Setup { request_type: 0xc0, request: 1, value: 0, index: 0, length: 1 };
        dev.control(other, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "a vendor request to the firmware");
    }

    #[test]
    fn flags_unscrambled_malformed_and_out_of_turn_commands() {
        let mut v = Violations::default();
        let mut dev = board(&[]);
        let mut host = Host { lfsr: 0 };
        host.transact(&mut dev, &[0, CMD_READ_FW_VER], 128, &mut v);
        assert_eq!(v.drain().len(), 1, "a command before the reseed");
        dev.transfer_out(1, &[0x20, 0x24, 0x4b, 0x35, 0x8f], &mut v);
        assert_eq!(v.drain().len(), 1, "a wrong reseed");
        host.reseed(&mut dev, &mut v);
        host.eeprom(&mut dev, 8, 8, &mut v);
        assert!(v.is_empty(), "{v:?}");
        host.transact(&mut dev, &[0, CMD_READ_EEPROM, 0x33, 0x80, 8, 0, 8, 0], 8, &mut v);
        assert_eq!(v.drain().len(), 1, "wrong cookie");
        host.transact(&mut dev, &[0, CMD_READ_EEPROM, 0x33, 0x81, 0xf8, 0, 16, 0], 16, &mut v);
        assert_eq!(v.drain().len(), 1, "beyond the EEPROM");
        host.data(&mut dev, &[1, CMD_READ_TEMP], 1, &mut v);
        assert_eq!(v.drain().len(), 1, "flag byte");
        host.transact(&mut dev, &[0, 0x55], 1, &mut v);
        assert_eq!(v.drain().len(), 1, "unknown command");
        host.data(&mut dev, &[0, CMD_READ_TEMP], 512, &mut v);
        assert!(v.is_empty(), "a read rounded up to a packet: {v:?}");
        host.data(&mut dev, &[0, CMD_READ_FW_VER], 64, &mut v);
        assert_eq!(v.drain().len(), 1, "answer read with fewer bytes than it has");
        dev.transfer_out(1, &encrypt(0, &[0, CMD_READ_TEMP]), &mut v);
        dev.transfer_out(1, &encrypt(0, &[0, CMD_READ_TEMP]), &mut v);
        assert_eq!(v.drain().len(), 1, "the answer was not read");
        dev.transfer_in(1, 1, &mut v);
        // A blank FPGA.
        assert_eq!(host.read(&mut dev, REG_SCRATCH, &mut v), 0);
        assert!(v.is_empty(), "{v:?}");
        host.read(&mut dev, REG_STATUS, &mut v);
        assert_eq!(v.drain().len(), 1, "status of a blank FPGA");
        host.write(&mut dev, &[(0, 0)], &mut v);
        assert_eq!(v.drain().len(), 1, "write to a blank FPGA");
        // A bitstream that is not the stand-in, chunked wrong.
        host.data(&mut dev, &[0, CMD_SEND_BITSTREAM, 1, 0, 0], 1, &mut v);
        assert_eq!(v.drain().len(), 1, "data without init");
        host.data(&mut dev, &[0, CMD_INIT_BITSTREAM], 1, &mut v);
        host.data(&mut dev, &[0, CMD_SEND_BITSTREAM, 2, 0, 1, 2], 1, &mut v);
        host.data(&mut dev, &[0, CMD_SEND_BITSTREAM, 2, 0, 1, 2], 1, &mut v);
        assert_eq!(v.drain().len(), 1, "a chunk after a short one");
        host.transact(&mut dev, &[0, CMD_SEND_BITSTREAM, 0xfd, 3], 1, &mut v);
        assert_eq!(v.drain().len(), 1, "a chunk over 1020");
        host.read(&mut dev, REG_SCRATCH, &mut v);
        assert_eq!(v.drain().len(), 1, "not the stand-in");
        // A key the host does not use: what it sends decodes to garbage.
        let mut dev = board(&[("fpga", "1")]);
        let mut host = Host { lfsr: 0 };
        host.init(&mut dev, &mut v);
        host.authenticate(&mut dev, &mut v);
        assert!(v.is_empty(), "{v:?}");
        host.lfsr = 0;
        host.transact(&mut dev, &[0, CMD_READ_TEMP], 1, &mut v);
        assert!(!v.drain().is_empty(), "plain bytes after the key");
    }

    #[test]
    fn flags_registers_the_crypto_chip_and_captures_outside_the_driver_s_rules() {
        let mut v = Violations::default();
        let mut dev = board(&[("fpga", "1")]);
        let mut host = Host { lfsr: 0 };
        host.reseed(&mut dev, &mut v);
        host.write(&mut dev, &[(0x10, 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "an LED register");
        host.read(&mut dev, 0x06, &mut v);
        assert_eq!(v.drain().len(), 1, "a register the driver never reads");
        host.write(&mut dev, &[(0, 0x10)], &mut v);
        assert_eq!(v.drain().len(), 1, "a control value");
        host.write(&mut dev, &[(0x12, 5)], &mut v);
        assert_eq!(v.drain().len(), 1, "register 0x12 is 4");
        host.write(&mut dev, &[(REG_ADC_IDX, 0x11), (REG_ADC_MSB, 0)], &mut v);
        assert_eq!(v.drain().len(), 2, "no LSB: the index alone, then the MSB alone");
        host.adc(&mut dev, 0x99, 0, &mut v);
        assert_eq!(v.drain().len(), 1, "an ADC register the driver does not write");
        host.write(&mut dev, &[(REG_MAGIC, 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "magic before its read");
        let magic = host.eeprom(&mut dev, 0x10, 16, &mut v);
        host.write(&mut dev, &[(REG_MAGIC, magic[0] ^ 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "a wrong magic byte");
        // The crypto chip.
        host.i2c_write(&mut dev, &[3, 7, 0x1b, 0, 0, 0, 0x24, 0xcd], &mut v);
        assert_eq!(v.drain().len(), 1, "before the wake");
        host.data(&mut dev, &[0, CMD_WAKE_I2C], 1, &mut v);
        host.i2c_read(&mut dev, 3, &mut v);
        assert_eq!(v.drain().len(), 1, "the wake answer read short");
        host.i2c_write(&mut dev, &[3, 7, 0x1b, 0, 0, 0, 0x24, 0xce], &mut v);
        assert_eq!(v.drain().len(), 1, "CRC");
        host.i2c_write(&mut dev, &[3, 8, 0x1b, 0, 0, 0, 0x24, 0xcd], &mut v);
        assert_eq!(v.drain().len(), 1, "count");
        let mut p = vec![3, 7, 0x08, 0, 0, 0];
        let crc = crc16(&p[1..]);
        p.extend_from_slice(&crc);
        host.i2c_write(&mut dev, &p, &mut v);
        assert_eq!(v.drain().len(), 1, "an opcode authenticate() does not send");
        host.i2c_write(&mut dev, &[3, 7, 0x41, 0x80, 0, 0, 0x28, 0x05], &mut v);
        assert_eq!(v.drain().len(), 1, "Sign without a Nonce");
        host.transact(&mut dev, &[0, CMD_READ_I2C, 0xc2, 4, 0], 5, &mut v);
        assert_eq!(v.drain().len(), 1, "another I2C address");
        // A capture outside the rules: no start command, the analog
        // mask, a mux rate that is not the channel count, a rate off
        // the list, register 0x15 not 3, the magic not written.
        host.write(&mut dev, &[(8, 1), (6, 3), (0x0e, 1), (0x0b, 0x05), (0x14, 0xff)], &mut v);
        host.write(&mut dev, &[(0, 1)], &mut v);
        assert_eq!(v.drain().len(), 6, "{v:?}");
        // A pre-divider that does not go with the rate.
        host.write(&mut dev, &[(0, 0), (8, 0), (0x0e, 2), (0x0b, 0x02), (REG_15, 3)], &mut v);
        host.data(&mut dev, &[0, CMD_START], 2, &mut v);
        host.write(&mut dev, &[(0, 1)], &mut v);
        assert_eq!(v.drain().len(), 2, "pre-divider 0xff at 50 MHz; the magic: {v:?}");
        dev.transfer_in(2, 500, &mut v);
        assert_eq!(v.drain().len(), 1, "not a multiple of 512");
        host.read(&mut dev, REG_STATUS, &mut v);
        assert_eq!(v.drain().len(), 1, "status during the capture");
        host.data(&mut dev, &[0, CMD_STOP], 2, &mut v);
        assert_eq!(v.drain().len(), 1, "stop before control 0");
    }

    #[test]
    fn faults_answer_as_a_failing_board_and_keep_the_scramblers_in_step() {
        let mut v = Violations::default();
        let mut dev = board(&[
            ("stall", "read-reg:2"),
            ("stall_reply", "read-eeprom:1"),
            ("short_reply", "fw-version"),
            ("bad_status", "wake-i2c"),
            ("wake", "0x00"),
        ]);
        let mut host = Host { lfsr: 0 };
        host.reseed(&mut dev, &mut v);
        let Reply::Data(d) = host.transact(&mut dev, &[0, CMD_READ_FW_VER], 128, &mut v) else {
            panic!()
        };
        assert_eq!(d.len(), 127);
        let serial = [0, CMD_READ_EEPROM, 0x33, 0x81, 8, 0, 8, 0];
        assert_eq!(host.transact(&mut dev, &serial, 8, &mut v), Reply::Stall);
        assert_eq!(host.read(&mut dev, REG_SCRATCH, &mut v), 0);
        let scratch = [0, CMD_READ_REG, 1, 0x7f];
        assert_eq!(host.transact(&mut dev, &scratch, 1, &mut v), Reply::Stall);
        assert_eq!(dev.transfer_in(1, 1, &mut v), Reply::Nak, "a stalled command has no answer");
        assert_eq!(host.data(&mut dev, &[0, CMD_WAKE_I2C], 1, &mut v), [0xff]);
        let mut answer = vec![2];
        answer.extend(frame(&[0]));
        assert_eq!(host.i2c_read(&mut dev, 4, &mut v), answer, "the wake= status");
        assert!(v.is_empty(), "{v:?}");
        // The scratch register after an upload.
        let mut dev = board(&[("scratch", "0x55")]);
        let mut host = Host { lfsr: 0 };
        host.reseed(&mut dev, &mut v);
        host.bitstream(&mut dev, &mut v);
        host.write(&mut dev, &[(REG_SCRATCH, 0xaa)], &mut v);
        assert_eq!(host.read(&mut dev, REG_SCRATCH, &mut v), 0x55);
        assert!(v.is_empty(), "{v:?}");
        // A Sign answer stalled: neither side takes the key.
        let mut dev =
            board(&[("fpga", "1"), ("stall_reply", "read-i2c:5"), ("capture_status", "0x01")]);
        let mut host = Host { lfsr: 0 };
        host.init(&mut dev, &mut v);
        assert_eq!(host.data(&mut dev, &[0, CMD_WAKE_I2C], 1, &mut v), [0]);
        host.i2c_read(&mut dev, 4, &mut v);
        host.i2c_write(&mut dev, &[3, 7, 0x1b, 0, 0, 0, 0x24, 0xcd], &mut v);
        host.i2c_read(&mut dev, 35, &mut v);
        for _ in 0..2 {
            host.i2c_write(&mut dev, &nonce_packet(), &mut v);
            host.i2c_read(&mut dev, 35, &mut v);
        }
        host.i2c_write(&mut dev, &[3, 7, 0x41, 0x80, 0, 0, 0x28, 0x05], &mut v);
        let sign = [0, CMD_READ_I2C, 0xc0, 67, 0];
        assert_eq!(host.transact(&mut dev, &sign, 68, &mut v), Reply::Stall);
        assert_eq!(host.data(&mut dev, &[0, CMD_READ_TEMP], 1, &mut v), [TEMPERATURE as u8]);
        assert_eq!(host.lfsr, 0);
        assert_eq!(host.stop(&mut dev, &mut v), 0x01);
        assert!(v.is_empty(), "{v:?}");
        for bad in [
            ("stall", "nope"),
            ("stall", "start:0"),
            ("bad_status", "read-reg"),
            ("wake", "256"),
            ("halt_in", "x"),
            ("stall_load", "0"),
            ("boot_strings", "Saleae"),
            ("stall_string", "4"),
            ("flashed", "yes"),
        ] {
            assert!(build(&opts(&[bad])).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn stream_faults_are_paced_by_transfers() {
        let mut v = Violations::default();
        let mut dev =
            board(&[("fpga", "1"), ("halt_in", "1"), ("short_in", "4"), ("transfers", "9")]);
        let mut host = Host { lfsr: 0 };
        host.init(&mut dev, &mut v);
        host.capture(&mut dev, 1, &mut v);
        assert_eq!(dev.transfer_in(2, 16384, &mut v), Reply::Data(vec![0; 4]));
        assert_eq!(dev.transfer_in(2, 16384, &mut v), Reply::Stall);
        assert_eq!(dev.transfer_in(2, 16384, &mut v), Reply::Stall, "the halt stays");
        assert!(v.is_empty(), "{v:?}");
        // Faults on the strings the scan reads.
        let mut m = build(&opts(&[("stall_string", "1"), ("short_string", "2")])).unwrap();
        let get = |i: u16| Setup {
            request_type: 0x80,
            request: 6,
            value: 0x0300 | i,
            index: 0x409,
            length: 255,
        };
        assert_eq!(m.fail_standard(get(1)), Some(StandardFault::Stall));
        assert_eq!(m.fail_standard(get(2)), Some(StandardFault::Short(2)));
        assert_eq!(m.fail_standard(get(0)), None);
    }
}
