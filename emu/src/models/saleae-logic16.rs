// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The Saleae Logic16 (`21a9:1001`) as libsigrok's `saleae-logic16`
//! driver sees it, built on the EZ-USB shell ([`crate::shells::ezusb`]).
//!
//! Every fact below is taken from the C driver; no firmware source,
//! datasheet or recording of the device was available. What the driver
//! does not pin down is marked "assumed" and is the model's choice.
//!
//! **Before the firmware** the FX2 comes up from its `0xC0` EEPROM as
//! `21a9:1001` without strings (the driver matches the ids alone,
//! `api.c` `scan()`, and takes a device whose manufacturer and product
//! strings are not `Saleae LLC` / `Logic S/16` for one without
//! firmware, `check_conf_profile()`); `bcdDevice` 0x0000 is assumed.
//! The driver uploads `saleae-logic16-fx2.fw` through the boot loader
//! (`ezusb_upload_firmware()`, configuration 1), held byte for byte
//! against the stand-in, and the device renumerates. An FX2LP is
//! assumed; the stand-in fits the 8 KiB of an FX2 as well.
//!
//! **The firmware's device**: the same ids, manufacturer `Saleae LLC`
//! and product `Logic S/16` (`check_conf_profile()`), no serial number
//! (the driver reads none; assumed), high speed, interface 0 with bulk
//! endpoints 1 OUT and 1 IN for commands (`do_ep1_command()`) and 2 IN
//! for the samples (`dev_acquisition_start()`). Endpoint 1 has 64 byte
//! packets, the FX2's EP1 buffer (as in the shell's bare descriptors;
//! `do_ep1_command()` refuses more than 64 bytes); endpoint 2 has 512
//! (assumed, the FX2's large endpoints at high speed). 100 mA
//! (assumed). No vendor control request is the firmware's.
//!
//! **Commands** go to bulk endpoint 1, one per transfer of at most 64
//! bytes, scrambled with `encrypt()`; a command with an answer is
//! followed by a bulk IN on endpoint 1 (libsigrok asks for exactly the
//! answer's length; a device sees only that the host takes the packet),
//! scrambled the same way, which the host undoes with `decrypt()`. Both
//! restart their state (`0x9b`, `0x54`) for every transfer. The model
//! unscrambles each command with the inverse of `encrypt()` (which is
//! `decrypt()`) and scrambles its answers with `encrypt()`, so a host
//! that sends plain bytes, or carries the state over from one transfer
//! to the next, sends a command or an argument the model refuses:
//!
//! | byte | command | arguments | answer |
//! |---|---|---|---|
//! | `0x7d` | abort, synchronous | `0x55` | `0xaa` (`~0x55`) |
//! | `0x07` | read EEPROM | cookies `0x33 0x81`, address, length | the bytes |
//! | `0x81` | read FPGA registers | count 1, address | the value |
//! | `0x80` | write FPGA registers | count 1-31, (address, value) pairs | none |
//! | `0x7e` | FPGA upload, init | none | none |
//! | `0x7f` | FPGA upload, data | length 1-62, the bytes | none |
//! | `0x7a` | LED table | offset, length, the bytes (offset + length <= 64) | none |
//! | `0x7b` | LED mode | animate, T2 reload (2 bytes, LE), divider, repeat | none |
//! | `0x01` | start acquisition | none | none |
//! | `0x02` | abort, asynchronous | none | none |
//!
//! `0x06` (write EEPROM), `0x7c` (back to the boot loader) and `0x82`
//! (REVID) exist but the driver never sends them; the model flags them.
//!
//! **The EEPROM** (256 bytes assumed): bytes 0-7 the FX2's `0xC0`
//! header (TRM: `C0`, VID, PID, DID little endian, a config byte), the
//! rest invented (the SHA-256 of a fixed text, repeated). The driver reads
//! 8 bytes at 8, kept but unused, and 16 at 16, the FPGA primer.
//!
//! **The FPGA** comes up blank. A register read of a blank FPGA answers
//! 0 (assumed; the driver reads register 0 once, before the upload, to
//! tell an mcupro clone; any other read of a blank FPGA is flagged).
//! `0x7e` starts an upload, `0x7f` chunks follow
//! (`upload_fpga_bitstream()`: 62 bytes each, only the last shorter), and the
//! next other command ends it: the bytes must be one of the two
//! bitstream stand-ins, `saleae-logic16-fpga-18.bitstream` (1.8 V and
//! 3.3 V logic) or `-33` (5 V logic, the driver's name for it). The
//! loaded bitstream answers the register map of `protocol.c`: the old
//! one by default, the new one with `bitstream=new`, which the driver
//! tells by register 0 reading 0 and register 7 above 0x10
//! (`setup_register_mapping()`). The version register answers 0x10
//! (old) or 0x13 (new), versions the driver accepts (`prime_fpga()`);
//! which bitstream has which is assumed. `variant=mcupro` is the mcupro
//! clone, whose FPGA holds its design from power on, answers 0x40 at
//! register 0 and takes no upload; it uses the old map.
//!
//! The FPGA must be primed before it samples (the model's reading of
//! `prime_fpga()`, held to its exact sequence): with `x` the mode
//! register without its `UNKNOWN2` bit, `MODE` `x` then `x|UNKNOWN1`
//! once; then for each of the 16 EEPROM bytes from address 16,
//! `PRIMER_DATA2` the byte, `MODE` `x|UNKNOWN1|UNKNOWN2` then
//! `x|UNKNOWN1`, `PRIMER_DATA1` its `map_eeprom_data()`, and
//! `PRIMER_CONTROL` 1 then 0 (the pulse that latches the pair); then
//! `MODE` `x` again. Any other order or value, or a capture set up
//! before the sequence is complete, is flagged. The LED table must be
//! `make_heartbeat()`'s in two chunks of 32 bytes, and the LED mode
//! `configure_led()`'s (animate 1, T2 reload 6250, divider 0, repeat 1).
//!
//! The status/control register reads back the control bits written
//! (`RUNNING`, `UNKNOWN2`), `UNKNOWN1` always (the driver expects it in
//! every read, `logic16_setup_acquisition()`, `logic16_abort_acquisition()`)
//! and `OVERFLOW` after an overflow; writing `UPDATE` latches the mode
//! (bit `CLOCK`: 160 MHz, else 100 MHz), the divisor (rate = clock /
//! (divisor + 1)) and the channel selection. Command `0x01` arms the
//! capture and writing `RUNNING` starts it; `0x02` or clearing
//! `RUNNING` ends it. The rate may be at most 100 MHz, and rate times
//! channels at most 300 MHz (`MAX_SAMPLE_RATE`, `MAX_SAMPLE_RATE_X_CH`);
//! so any 100 MHz / n or 160 MHz / n with n up to 256, the driver's list
//! or not, down to 390.625 kHz.
//!
//! **Samples** come on endpoint 2 as little endian 16 bit words, each
//! holding 16 consecutive samples of one channel, the first sample in
//! bit 15, the words going round the selected channels in ascending
//! order (`convert_sample_data()`). Sample `n` of channel `c` is bit `c`
//! of `n`, so channel `c` toggles every 2^c samples.
//!
//! As in the fx2lafw model, and as an artifact of the emulation, a
//! running capture completes one bulk IN transfer of endpoint 2 per
//! transfer the host submits after the previous one completed (the
//! first right after the start), filled to the length asked; the real
//! FPGA streams as fast as it samples. Answers on endpoint 1 come at
//! once.
//!
//! **Pedantic checks**: the FX2 upload as `ezusb.c` sends it; the
//! bitstream byte for byte; every command's length, cookies, counts and
//! addresses (registers outside the loaded map, other than the ones the
//! driver reads to tell the variants: 0 and 7, and 8 and 9 of the old
//! map at a stop); an answer not read before the next command, or read
//! by a transfer shorter than the answer (the host would see a babble);
//! register access to a blank FPGA (but the first
//! read of register 0); writes to the version register; status bits the
//! host does not own; the primer; a capture without `UPDATE`, without
//! `0x01`, without channels, or beyond the rate limits; bulk IN
//! transfers that are not a multiple of 512 bytes; a firmware upload to
//! the running firmware; vendor control requests.
//!
//! **Faults**, all off by default: `stall=NAME[:N]` STALLs the command
//! (each one, or only its Nth since the model's reset), `stall_reply=`
//! STALLs its answer, `short_reply=` answers one byte short, with `NAME`
//! one of `abort-sync`, `read-eeprom`, `fpga-read`, `fpga-write`,
//! `upload-init`, `upload-data`, `led-table`, `led-mode`, `start`,
//! `abort` (comma lists; a stalled command has no effect; a STALL is
//! one-shot, an emulation artifact). A command fits one packet, which
//! the FX2 takes whole or not at all, so there is no short command;
//! `abort_reply=0xNN` answers the synchronous abort with that byte;
//! `fpga_version=0xNN` answers the version register with it;
//! `status_xor=0xNN` and `mode_xor=0xNN` flip those bits of every status
//! and mode register read; `overflow=1` reports a FIFO overflow after
//! each capture; on endpoint 2, `empty=N` completes the first N
//! transfers of each capture with no data, `stall_in=N` STALLs the N
//! after those, and `short_in=N` makes the first transfer with samples
//! carry only N bytes (an odd N is what the driver bails out on; fewer
//! than one word per channel holds no sample yet).

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::ezusb::{self, Chip, Loader};
use crate::{Error, Result};
use sha2::{Digest as _, Sha256};

/// The model name.
pub const NAME: &str = "saleae-logic16";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Saleae Logic16 (21a9:1001, EZ-USB, renumerates, FPGA bitstream over \
                         scrambled EP1 commands); flashed=1 starts with the FX2 firmware running, \
                         bitstream=old|new, variant=original|mcupro; faults: stall=, stall_reply=, \
                         short_reply=NAME[:N], abort_reply=, fpga_version=, status_xor=, \
                         mode_xor=, overflow=1, empty=N, stall_in=N, short_in=N";

/// Vendor and product id, before and after the upload (`api.c`).
pub const VID: u16 = 0x21a9;
/// The product id.
pub const PID: u16 = 0x1001;
/// The FX2 firmware and its stand-in's size (`tools/emu/saleae-logic16.firmware`).
pub const FX2_FIRMWARE: (&str, usize) = ("saleae-logic16-fx2.fw", 5340);
/// The bitstream for 1.8 V and 3.3 V logic.
pub const BITSTREAM_18: (&str, usize) = ("saleae-logic16-fpga-18.bitstream", 3001);
/// The bitstream for 5 V logic.
pub const BITSTREAM_33: (&str, usize) = ("saleae-logic16-fpga-33.bitstream", 3101);
/// The bulk packet size at high speed.
const PACKET: usize = 512;
/// The EEPROM size (assumed).
const EEPROM_SIZE: usize = 256;

const CMD_START: u8 = 0x01;
const CMD_ABORT_ASYNC: u8 = 0x02;
const CMD_WRITE_EEPROM: u8 = 0x06;
const CMD_READ_EEPROM: u8 = 0x07;
const CMD_LED_TABLE: u8 = 0x7a;
const CMD_LED_MODE: u8 = 0x7b;
const CMD_BOOTLOADER: u8 = 0x7c;
const CMD_ABORT_SYNC: u8 = 0x7d;
const CMD_UPLOAD_INIT: u8 = 0x7e;
const CMD_UPLOAD_DATA: u8 = 0x7f;
const CMD_FPGA_WRITE: u8 = 0x80;
const CMD_FPGA_READ: u8 = 0x81;
const CMD_REVID: u8 = 0x82;

/// The fault names of the commands.
const COMMANDS: [(&str, u8); 10] = [
    ("abort-sync", CMD_ABORT_SYNC),
    ("read-eeprom", CMD_READ_EEPROM),
    ("fpga-read", CMD_FPGA_READ),
    ("fpga-write", CMD_FPGA_WRITE),
    ("upload-init", CMD_UPLOAD_INIT),
    ("upload-data", CMD_UPLOAD_DATA),
    ("led-table", CMD_LED_TABLE),
    ("led-mode", CMD_LED_MODE),
    ("start", CMD_START),
    ("abort", CMD_ABORT_ASYNC),
];

/// `encrypt()` of `protocol.c`: what the host sends, and what the
/// device answers with.
pub fn encrypt(src: &[u8]) -> Vec<u8> {
    let (mut state1, mut state2) = (0x9bu8, 0x54u8);
    src.iter()
        .map(|&v| {
            let t = ((v ^ state2 ^ 0x2b).wrapping_sub(0x05) ^ 0x35).wrapping_sub(0x39);
            let t = ((t ^ state1 ^ 0x5a).wrapping_sub(0xb0) ^ 0x38).wrapping_sub(0x45);
            state2 = t;
            state1 = v;
            t
        })
        .collect()
}

/// `decrypt()` of `protocol.c`, the inverse of [`encrypt`]: what the
/// device does to a command, and the host to an answer.
pub fn decrypt(src: &[u8]) -> Vec<u8> {
    let (mut state1, mut state2) = (0x9bu8, 0x54u8);
    src.iter()
        .map(|&v| {
            let t = (((v.wrapping_add(0x45) ^ 0x38).wrapping_add(0xb0)) ^ 0x5a) ^ state1;
            let t = (((t.wrapping_add(0x39) ^ 0x35).wrapping_add(0x05)) ^ 0x2b) ^ state2;
            state1 = t;
            state2 = v;
            t
        })
        .collect()
}

/// `map_eeprom_data()` of `protocol.c`.
pub fn map_eeprom_data(v: u8) -> u8 {
    ((v ^ 0x80).wrapping_add(0x44) ^ 0xd5).wrapping_add(0x69)
}

/// `make_heartbeat()` of `protocol.c` for 64 bytes: a half sine of 8
/// steps, twice, then zeros.
pub fn heartbeat() -> [u8; 64] {
    let mut t = [0u8; 64];
    for i in 0..2 {
        for j in 0..8 {
            t[i * 8 + j] = (((j as f64) * std::f64::consts::PI / 8.0).sin() * 255.0) as u8;
        }
    }
    t
}

/// An FPGA register map with its status and mode bits (`protocol.c`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegisterMap {
    version: u8,
    status: u8,
    channel_low: u8,
    channel_high: u8,
    divisor: u8,
    led: u8,
    primer_data1: u8,
    primer_control: u8,
    mode: u8,
    primer_data2: u8,
    running: u8,
    update: u8,
    unknown1: u8,
    overflow: u8,
    unknown2: u8,
    clock: u8,
    mode_u1: u8,
    mode_u2: u8,
}

/// `fpga_register_map_old` and its bit maps.
pub const MAP_OLD: RegisterMap = RegisterMap {
    version: 0,
    status: 1,
    channel_low: 2,
    channel_high: 3,
    divisor: 4,
    led: 5,
    primer_data1: 6,
    primer_control: 7,
    mode: 10,
    primer_data2: 12,
    running: 0x01,
    update: 0x02,
    unknown1: 0x08,
    overflow: 0x20,
    unknown2: 0x40,
    clock: 0x01,
    mode_u1: 0x40,
    mode_u2: 0x80,
};

/// `fpga_register_map_new` and its bit maps.
pub const MAP_NEW: RegisterMap = RegisterMap {
    version: 7,
    status: 15,
    channel_low: 1,
    channel_high: 6,
    divisor: 11,
    led: 5,
    primer_data1: 14,
    primer_control: 2,
    mode: 4,
    primer_data2: 3,
    running: 0x20,
    update: 0x08,
    unknown1: 0x10,
    overflow: 0x01,
    unknown2: 0x04,
    clock: 0x04,
    mode_u1: 0x80,
    mode_u2: 0x01,
};

impl RegisterMap {
    fn registers(&self) -> [u8; 10] {
        [
            self.version,
            self.status,
            self.channel_low,
            self.channel_high,
            self.divisor,
            self.led,
            self.primer_data1,
            self.primer_control,
            self.mode,
            self.primer_data2,
        ]
    }
}

/// Which board.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Variant {
    /// The Saleae Logic16, blank FPGA at power on.
    Original,
    /// The mcupro clone, its FPGA configured from power on.
    Mcupro,
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
        out.push(Shot { command, nth });
    }
    Ok(out)
}

/// Ways the board can misbehave, as a real one can; none by default.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    stall: Vec<Shot>,
    stall_reply: Vec<Shot>,
    short_reply: Vec<Shot>,
    abort_reply: Option<u8>,
    fpga_version: Option<u8>,
    status_xor: u8,
    mode_xor: u8,
    overflow: bool,
    empty: u32,
    stall_in: u32,
    short_in: Option<usize>,
}

/// Whether the command has `n` bytes; reports it when not.
fn length_is(cmd: &[u8], n: usize, v: &mut Violations) -> bool {
    if cmd.len() == n {
        return true;
    }
    v.report(format!("command {:#04x} of {} bytes; it has {n}", cmd[0], cmd.len()));
    false
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

impl Faults {
    /// Takes the option `k=v` when it is a fault; false when `k` is no
    /// fault option.
    fn parse(&mut self, k: &str, v: &str) -> Result<bool> {
        let count = |v: &str| v.parse::<u32>().map_err(|_| Error::Invalid(format!("{k}={v}: a count")));
        match k {
            "stall" => self.stall = shots(k, v)?,
            "stall_reply" => self.stall_reply = shots(k, v)?,
            "short_reply" => self.short_reply = shots(k, v)?,
            "abort_reply" => self.abort_reply = Some(byte(k, v)?),
            "fpga_version" => self.fpga_version = Some(byte(k, v)?),
            "status_xor" => self.status_xor = byte(k, v)?,
            "mode_xor" => self.mode_xor = byte(k, v)?,
            "overflow" => self.overflow = flag(k, v)?,
            "empty" => self.empty = count(v)?,
            "stall_in" => self.stall_in = count(v)?,
            "short_in" => self.short_in = Some(count(v)? as usize),
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// Firmware chunks of 4 KiB and sample transfers.
pub fn log_rules(_: &[(String, String)]) -> crate::txlog::Rules {
    crate::txlog::Rules::digest_above(crate::txlog::HASH_ABOVE)
}

/// A board from the `-o` options: `flashed=0|1`, `bitstream=old|new`,
/// `variant=original|mcupro`, and the faults (see [`Faults`]).
///
/// # Errors
///
/// On an unknown option or value.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut flashed = false;
    let mut new_map = false;
    let mut variant = Variant::Original;
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.parse(k, v)? {
            continue;
        }
        match k.as_str() {
            "flashed" => flashed = flag(k, v)?,
            "bitstream" => {
                new_map = match v.as_str() {
                    "old" => false,
                    "new" => true,
                    _ => return Err(Error::Invalid(format!("bitstream={v}: old or new"))),
                }
            }
            "variant" => {
                variant = match v.as_str() {
                    "original" => Variant::Original,
                    "mcupro" => Variant::Mcupro,
                    _ => return Err(Error::Invalid(format!("variant={v}: original or mcupro"))),
                }
            }
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    if variant == Variant::Mcupro && new_map {
        return Err(Error::Invalid("bitstream=new: the mcupro clone has the old map".into()));
    }
    let mut model = Logic16::new(flashed, variant, new_map);
    model.faults = faults;
    Ok(Box::new(model))
}

/// The descriptors of the running firmware.
pub fn firmware_descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 3, [0xff, 0xff, 0xff], 0));
    for (address, size) in [(0x01, 64), (0x81, 64), (0x82, 512)] {
        body.extend_from_slice(&descriptor::endpoint(address, crate::model::BULK, size, 0));
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
        strings: vec!["Saleae LLC".into(), "Logic S/16".into()],
    }
}

/// The EEPROM: the `0xC0` header, then invented bytes.
pub fn eeprom() -> Vec<u8> {
    let mut e: Vec<u8> = Sha256::digest(b"saleae-logic16 emulated EEPROM")
        .iter()
        .copied()
        .cycle()
        .take(EEPROM_SIZE)
        .collect();
    let [vl, vh] = VID.to_le_bytes();
    let [pl, ph] = PID.to_le_bytes();
    e[..8].copy_from_slice(&[0xc0, vl, vh, pl, ph, 0, 0, 0]);
    e
}

/// What the FPGA holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Design {
    /// A bitstream the host uploaded: true for the 5 V one.
    Bitstream(bool),
    /// The mcupro clone's own.
    Mcupro,
}

/// The sampling parameters latched by `UPDATE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Config {
    rate: u64,
    channels: u16,
}

/// A running capture on endpoint 2.
#[derive(Clone, Debug)]
struct Stream {
    channels: Vec<u8>,
    /// The block of 16 samples the next word belongs to.
    block: u64,
    /// The index in `channels` of the next word.
    at: usize,
    /// Whether the next bulk IN transfer may complete.
    ready: bool,
    empty: u32,
    stall_in: u32,
    /// The length of the first transfer with samples, when short.
    short: Option<usize>,
}

impl Stream {
    fn word(&mut self) -> u16 {
        let c = u32::from(self.channels[self.at]);
        let base = self.block * 16;
        let mut w = 0u16;
        for k in 0..16u64 {
            if (base + k) >> c & 1 != 0 {
                w |= 1 << (15 - k);
            }
        }
        self.at += 1;
        if self.at == self.channels.len() {
            self.at = 0;
            self.block += 1;
        }
        w
    }
}

/// Where `prime_fpga()`'s sequence stands.
#[derive(Clone, Debug, Default)]
struct Primer {
    /// The last writes to `MODE`, `PRIMER_DATA1`, `PRIMER_DATA2`
    /// since the last pulse ended (at most 8).
    log: Vec<(u8, u8)>,
    /// Pairs latched in this sequence.
    pairs: usize,
    /// The mode register's value `x` of this sequence.
    base: u8,
    /// `PRIMER_CONTROL` is 1; the next primer write must clear it.
    pulse: bool,
    /// All 16 pairs and the mode restored.
    done: bool,
}

/// The FPGA and what the FX2 knows of it.
#[derive(Clone, Debug)]
struct Fpga {
    design: Option<Design>,
    /// The bitstream being uploaded.
    upload: Option<Vec<u8>>,
    map: RegisterMap,
    regs: [u8; 16],
    primer: Primer,
    /// The upload had a chunk shorter than 62 bytes.
    short_chunk: bool,
    /// The control bits of the status register.
    control: u8,
    overflow: bool,
    config: Option<Config>,
    /// Command `0x01` came.
    armed: bool,
    stream: Option<Stream>,
}

/// What the FX2 runs.
#[derive(Clone, Debug)]
enum Stage {
    /// The boot loader, waiting for the firmware.
    Boot(Box<Loader>),
    /// The Logic16 firmware.
    Running,
}

/// The emulated board.
pub struct Logic16 {
    variant: Variant,
    new_map: bool,
    boot: Descriptors,
    firmware: Descriptors,
    stage: Stage,
    renumerate: bool,
    eeprom: Vec<u8>,
    fpga: Fpga,
    /// The scrambled answer waiting for its bulk IN, and its command.
    reply: Option<(u8, Vec<u8>)>,
    /// The LED table as written.
    led: [u8; 64],
    /// Register 0 of the blank FPGA was read (the mcupro probe).
    probed: bool,
    /// Commands seen per command byte, for the faults' Nth.
    counts: [u32; 256],
    faults: Faults,
}

impl Logic16 {
    /// A board; `flashed` starts it with the FX2 firmware running.
    pub fn new(flashed: bool, variant: Variant, new_map: bool) -> Logic16 {
        let stage =
            if flashed { Stage::Running } else { Stage::Boot(Box::new(Loader::new(Chip::Fx2lp))) };
        let mcupro = variant == Variant::Mcupro;
        Logic16 {
            variant,
            new_map,
            boot: ezusb::bare_descriptors(VID, PID, 0x0000),
            firmware: firmware_descriptors(),
            stage,
            renumerate: false,
            eeprom: eeprom(),
            fpga: Fpga {
                design: mcupro.then_some(Design::Mcupro),
                upload: None,
                map: MAP_OLD,
                regs: [0; 16],
                primer: Primer::default(),
                short_chunk: false,
                control: 0,
                overflow: false,
                config: None,
                armed: false,
                stream: None,
            },
            reply: None,
            led: [0; 64],
            probed: false,
            counts: [0; 256],
            faults: Faults::default(),
        }
    }

    /// Whether the firmware runs.
    pub fn running(&self) -> bool {
        matches!(self.stage, Stage::Running)
    }

    fn boot_control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let image = ezusb::standin(FX2_FIRMWARE.0, FX2_FIRMWARE.1);
        let Stage::Boot(loader) = &mut self.stage else { unreachable!() };
        let Some((reply, started)) = loader.control(setup, data, v) else {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} to the FX2 without firmware; only its \
                 boot loader (0xa0) answers",
                setup.request_type, setup.request, setup.value
            ));
            return Reply::Stall;
        };
        if started {
            ezusb::check_ezusb_c(loader, &image, v);
            self.stage = Stage::Running;
            self.renumerate = true;
        }
        reply
    }

    /// Ends a bitstream upload: the FPGA takes the design when the
    /// bytes are one of the stand-ins.
    fn end_upload(&mut self, v: &mut Violations) {
        let Some(bytes) = self.fpga.upload.take() else { return };
        let five = if bytes == ezusb::standin(BITSTREAM_18.0, BITSTREAM_18.1) {
            false
        } else if bytes == ezusb::standin(BITSTREAM_33.0, BITSTREAM_33.1) {
            true
        } else {
            v.report(format!(
                "FPGA upload of {} bytes that are neither {} ({} bytes) nor {} ({} bytes)",
                bytes.len(),
                BITSTREAM_18.0,
                BITSTREAM_18.1,
                BITSTREAM_33.0,
                BITSTREAM_33.1
            ));
            return;
        };
        self.fpga.design = Some(Design::Bitstream(five));
        self.fpga.map = if self.new_map { MAP_NEW } else { MAP_OLD };
        self.fpga.regs = [0; 16];
        self.fpga.primer = Primer::default();
        self.fpga.control = 0;
        self.fpga.overflow = false;
        self.fpga.config = None;
    }

    fn stop(&mut self) {
        if self.fpga.stream.take().is_some() && self.faults.overflow {
            self.fpga.overflow = true;
        }
        self.fpga.armed = false;
    }

    fn version(&self) -> u8 {
        self.faults.fpga_version.unwrap_or(match self.fpga.design {
            Some(Design::Mcupro) => 0x40,
            _ if self.new_map => 0x13,
            _ => 0x10,
        })
    }

    fn read_register(&mut self, address: u8, v: &mut Violations) -> u8 {
        let m = self.fpga.map;
        if self.fpga.design.is_none() {
            if address != 0 || self.probed {
                v.report(format!(
                    "read of FPGA register {address} with no bitstream loaded (only the first \
                     read of register 0 is the driver's)"
                ));
            }
            self.probed = true;
            return 0;
        }
        let old = m == MAP_OLD;
        let known = m.registers().contains(&address)
            || address == 0
            || address == 7
            || (old && (address == 8 || address == 9));
        if !known {
            v.report(format!("read of FPGA register {address}, not one of the loaded map"));
        }
        if address == m.version {
            self.version()
        } else if address == m.status {
            let overflow = if self.fpga.overflow { m.overflow } else { 0 };
            (self.fpga.control | m.unknown1 | overflow) ^ self.faults.status_xor
        } else if address == m.mode {
            self.fpga.regs[usize::from(address)] ^ self.faults.mode_xor
        } else {
            self.fpga.regs.get(usize::from(address)).copied().unwrap_or(0)
        }
    }

    fn write_register(&mut self, address: u8, value: u8, v: &mut Violations) {
        let m = self.fpga.map;
        if self.fpga.design.is_none() {
            v.report(format!("write of FPGA register {address} with no bitstream loaded"));
            return;
        }
        if !m.registers().contains(&address) {
            v.report(format!("write of FPGA register {address}, not one of the loaded map"));
            return;
        }
        if address == m.version {
            v.report(format!("write of {value:#04x} to the FPGA version register {address}"));
            return;
        }
        if address == m.status {
            self.write_status(value, v);
            return;
        }
        self.fpga.regs[usize::from(address)] = value;
        if [m.mode, m.primer_data1, m.primer_data2, m.primer_control].contains(&address) {
            self.primer_write(address, value, v);
        }
    }

    /// One write of `prime_fpga()`'s sequence (see the module's doc).
    fn primer_write(&mut self, address: u8, value: u8, v: &mut Violations) {
        let m = self.fpga.map;
        let primer = &mut self.fpga.primer;
        if primer.pulse {
            primer.pulse = false;
            primer.log.clear();
            if (address, value) == (m.primer_control, 0) {
                return;
            }
            v.report(format!(
                "FPGA register {address} written while PRIMER_CONTROL is 1; the pulse ends with 0"
            ));
        }
        if primer.pairs == 16 && !primer.done {
            if (address, value) == (m.mode, primer.base) {
                primer.done = true;
            } else {
                v.report(format!(
                    "FPGA register {address} = {value:#04x} after the 16 primer pairs; the mode \
                     register goes back to {:#04x} first",
                    primer.base
                ));
                primer.pairs = 0;
            }
            return;
        }
        if address != m.primer_control {
            primer.log.push((address, value));
            if primer.log.len() > 8 {
                primer.log.remove(0);
            }
            return;
        }
        if value != 1 {
            v.report(format!("PRIMER_CONTROL {value:#04x} with no pulse to end"));
            return;
        }
        if primer.done {
            // A new sequence, as after a second upload_fpga_bitstream().
            primer.pairs = 0;
            primer.done = false;
        }
        primer.pulse = true;
        let n = primer.pairs;
        let byte = self.eeprom[16 + n];
        let (u1, u2) = (m.mode_u1, m.mode_u2);
        let take = if n == 0 { 6 } else { 4 };
        let got = &primer.log[primer.log.len().saturating_sub(take)..];
        let base = if n == 0 { got.first().map_or(0, |w| w.1) } else { primer.base };
        let mut want = Vec::new();
        if n == 0 {
            want.extend([(m.mode, base), (m.mode, base | u1)]);
        }
        want.extend([
            (m.primer_data2, byte),
            (m.mode, base | u1 | u2),
            (m.mode, base | u1),
            (m.primer_data1, map_eeprom_data(byte)),
        ]);
        if got != want || base & u2 != 0 || (n > 0 && primer.log.len() != 4) {
            v.report(format!(
                "FPGA primer pair {n}: the writes (register, value) {got:02x?} before the pulse; \
                 prime_fpga() sends {want:02x?} (EEPROM byte {} is {byte:#04x})",
                16 + n
            ));
            return;
        }
        primer.base = base;
        primer.pairs += 1;
    }

    fn write_status(&mut self, value: u8, v: &mut Violations) {
        let m = self.fpga.map;
        let owned = m.running | m.update | m.unknown2;
        if value & !owned != 0 {
            v.report(format!(
                "status/control write {value:#04x}: only {owned:#04x} are the host's bits"
            ));
        }
        if value & m.update != 0 {
            let mode = self.fpga.regs[usize::from(m.mode)];
            let clock: u64 = if mode & m.clock != 0 { 160_000_000 } else { 100_000_000 };
            let div = u64::from(self.fpga.regs[usize::from(m.divisor)]) + 1;
            let rate = clock / div;
            let channels = u16::from_le_bytes([
                self.fpga.regs[usize::from(m.channel_low)],
                self.fpga.regs[usize::from(m.channel_high)],
            ]);
            if !self.fpga.primer.done {
                v.report(format!(
                    "capture set up before the FPGA primer sequence ended ({} of 16 pairs)",
                    self.fpga.primer.pairs
                ));
            }
            if channels == 0 {
                v.report("capture set up with no channel selected");
            }
            if rate > 100_000_000 {
                v.report(format!("{rate} Hz ({clock} / {div}): the Logic16 samples at 100 MHz at most"));
            }
            if rate * u64::from(channels.count_ones()) > 300_000_000 {
                v.report(format!(
                    "{rate} Hz on {} channels: beyond the 300 MHz the Logic16 carries",
                    channels.count_ones()
                ));
            }
            self.fpga.config = Some(Config { rate, channels });
            self.fpga.overflow = false;
        }
        let was = self.fpga.control & m.running != 0;
        self.fpga.control = value & (m.running | m.unknown2);
        let now = value & m.running != 0;
        if now && !was {
            let Some(config) = self.fpga.config else {
                v.report("capture started before its parameters were latched (UPDATE)");
                return;
            };
            if !self.fpga.armed {
                v.report("capture started without the start command (0x01)");
            }
            let channels: Vec<u8> = (0..16).filter(|c| config.channels >> c & 1 != 0).collect();
            if channels.is_empty() {
                return;
            }
            let _ = config.rate;
            self.fpga.stream = Some(Stream {
                channels,
                block: 0,
                at: 0,
                ready: true,
                empty: self.faults.empty,
                stall_in: self.faults.stall_in,
                short: self.faults.short_in,
            });
        } else if !now && was {
            self.stop();
        }
    }

    /// One command, unscrambled; the plain answer, if it has one.
    fn command(&mut self, cmd: &[u8], v: &mut Violations) -> Option<Vec<u8>> {
        if cmd[0] != CMD_UPLOAD_DATA {
            self.end_upload(v);
        }
        match cmd[0] {
            CMD_ABORT_SYNC => {
                if !length_is(cmd, 2, v) {
                    return None;
                }
                if cmd[1] != 0x55 {
                    v.report(format!("synchronous abort with pattern {:#04x}; it is 0x55", cmd[1]));
                }
                self.stop();
                Some(vec![self.faults.abort_reply.unwrap_or(!cmd[1])])
            }
            CMD_READ_EEPROM => {
                if !length_is(cmd, 5, v) {
                    return None;
                }
                if cmd[1..3] != [0x33, 0x81] {
                    v.report(format!(
                        "EEPROM read with cookies {:#04x} {:#04x}; they are 0x33 0x81",
                        cmd[1], cmd[2]
                    ));
                    return None;
                }
                let (at, n) = (usize::from(cmd[3]), usize::from(cmd[4]));
                if n == 0 || n > 64 || at + n > EEPROM_SIZE {
                    v.report(format!(
                        "EEPROM read of {n} bytes at {at}: 1 to 64 within the {EEPROM_SIZE} bytes"
                    ));
                    return None;
                }
                Some(self.eeprom[at..at + n].to_vec())
            }
            CMD_FPGA_READ => {
                if !length_is(cmd, 3, v) {
                    return None;
                }
                if cmd[1] != 1 {
                    v.report(format!("FPGA read of {} registers; the driver reads 1", cmd[1]));
                    return None;
                }
                Some(vec![self.read_register(cmd[2], v)])
            }
            _ => {
                self.order(cmd, v);
                None
            }
        }
    }

    /// A command without an answer.
    fn order(&mut self, cmd: &[u8], v: &mut Violations) {
        let arg = |i: usize| usize::from(cmd.get(i).copied().unwrap_or(0));
        match cmd[0] {
            CMD_FPGA_WRITE => {
                let n = arg(1);
                if !(1..=31).contains(&n) {
                    v.report(format!("FPGA write of {n} registers; 1 to 31"));
                } else if length_is(cmd, 2 * (n + 1), v) {
                    for pair in cmd[2..].chunks(2) {
                        self.write_register(pair[0], pair[1], v);
                    }
                }
            }
            CMD_UPLOAD_INIT => {
                length_is(cmd, 1, v);
                if self.variant == Variant::Mcupro {
                    v.report("FPGA upload to the mcupro clone, whose FPGA holds its design");
                }
                self.stop();
                self.fpga.design = None;
                self.fpga.upload = Some(Vec::new());
                self.fpga.short_chunk = false;
            }
            CMD_UPLOAD_DATA => {
                let n = arg(1);
                if !(1..=62).contains(&n) {
                    v.report(format!("FPGA upload chunk of {n} bytes; 1 to 62"));
                } else if length_is(cmd, n + 2, v) {
                    match &mut self.fpga.upload {
                        Some(bytes) => bytes.extend_from_slice(&cmd[2..]),
                        None => v.report("FPGA upload data without the upload init (0x7e)"),
                    }
                    if self.fpga.short_chunk {
                        v.report("FPGA upload chunk after a short one; only the last is short");
                    }
                    self.fpga.short_chunk |= n < 62;
                }
            }
            CMD_LED_TABLE => {
                let (offset, n) = (arg(1), arg(2));
                if n == 0 || n > 32 || offset + n > 64 {
                    v.report(format!(
                        "LED table of {n} bytes at {offset}; upload_led_table() sends 1 to 32 \
                         within 64"
                    ));
                } else if length_is(cmd, n + 3, v) {
                    self.led[offset..offset + n].copy_from_slice(&cmd[3..]);
                }
            }
            CMD_LED_MODE => {
                if length_is(cmd, 6, v) && cmd[1..] != [1, 0x6a, 0x18, 0, 1] {
                    v.report(format!(
                        "LED mode {:02x?}; configure_led() sends animate 1, T2 reload 6250, \
                         divider 0, repeat 1",
                        &cmd[1..]
                    ));
                }
                if self.led != heartbeat() {
                    v.report(format!(
                        "LED mode set with the table {:02x?}; configure_led() uploads \
                         make_heartbeat()'s {:02x?}",
                        self.led,
                        heartbeat()
                    ));
                }
            }
            CMD_START => {
                length_is(cmd, 1, v);
                if self.fpga.config.is_none() {
                    v.report("start command before the capture was set up");
                }
                self.fpga.armed = true;
            }
            CMD_ABORT_ASYNC => {
                length_is(cmd, 1, v);
                self.stop();
            }
            CMD_WRITE_EEPROM | CMD_BOOTLOADER | CMD_REVID => {
                v.report(format!("command {:#04x}, which the driver never sends", cmd[0]));
            }
            other => v.report(format!(
                "command {other:#04x} (unscrambled; {} bytes): not one of the Logic16's, or not \
                 scrambled as encrypt() does",
                cmd.len()
            )),
        }
    }

    fn ep1_out(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if data.is_empty() || data.len() > 64 {
            v.report(format!("EP1 command of {} bytes; 1 to 64", data.len()));
            return Reply::Stall;
        }
        if let Some((pending, _)) = self.reply.take() {
            v.report(format!("command sent while the answer to {pending:#04x} was not read"));
        }
        let cmd = decrypt(data);
        let c = cmd[0];
        self.counts[usize::from(c)] += 1;
        let count = self.counts[usize::from(c)];
        if hit(&self.faults.stall, c, count) {
            return Reply::Stall;
        }
        if let Some(answer) = self.command(&cmd, v) {
            self.reply = Some((c, encrypt(&answer)));
        }
        Reply::ack()
    }

    fn ep1_in(&mut self, max: usize, v: &mut Violations) -> Reply {
        let Some((c, answer)) = self.reply.take() else {
            // No answer due: the FX2 has nothing to send.
            return Reply::Nak;
        };
        // The length the host asks for is not on the wire: an IN token
        // takes up to a packet, so asking for more than the answer is
        // the same to the FX2 (the emulator rounds bulk INs up to the
        // packet size). Asking for less is a babble on a real host.
        if max < answer.len() {
            v.report(format!(
                "answer to {c:#04x} read with {max} bytes; it has {}",
                answer.len()
            ));
        }
        let count = self.counts[usize::from(c)];
        if hit(&self.faults.stall_reply, c, count) {
            return Reply::Stall;
        }
        if hit(&self.faults.short_reply, c, count) {
            return Reply::Data(answer[..answer.len() - 1].to_vec());
        }
        Reply::Data(answer)
    }

    fn ep2_in(&mut self, max: usize, v: &mut Violations) -> Reply {
        let Some(s) = &mut self.fpga.stream else {
            return Reply::Nak;
        };
        if !s.ready {
            return Reply::Nak;
        }
        if !max.is_multiple_of(PACKET) {
            v.report(format!(
                "bulk IN transfer of {max} bytes; the driver asks for multiples of {PACKET}"
            ));
        }
        s.ready = false;
        if s.empty > 0 {
            s.empty -= 1;
            return Reply::Data(Vec::new());
        }
        if s.stall_in > 0 {
            s.stall_in -= 1;
            return Reply::Stall;
        }
        let len = s.short.take().map_or(max, |n| n.min(max));
        let mut data = Vec::with_capacity(len + 1);
        while data.len() < len {
            data.extend_from_slice(&s.word().to_le_bytes());
        }
        // An odd length ends mid-word; the rest of that word is lost.
        data.truncate(len);
        Reply::Data(data)
    }
}

impl DeviceModel for Logic16 {
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
        if setup.kind() == crate::model::VENDOR && setup.request == ezusb::REQ_FIRMWARE_LOAD {
            v.report("firmware load to an FX2 whose Logic16 firmware runs");
            return Reply::ack();
        }
        v.report(format!(
            "control request {:#04x} {:#04x} value {:#06x}: the Logic16 firmware takes its \
             commands on endpoint 1",
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

    fn in_submitted(&mut self, ep: u8, _max: usize) {
        if ep == 2
            && let Some(s) = &mut self.fpga.stream
        {
            s.ready = true;
        }
    }

    fn reset(&mut self) {
        // A new attach ends a capture and drops an unread answer; the
        // FX2 firmware and the FPGA design stay, as the board keeps power.
        self.fpga.stream = None;
        self.fpga.armed = false;
        self.reply = None;
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

    fn load(value: u16, length: u16) -> Setup {
        Setup { request_type: 0x40, request: 0xa0, value, index: 0, length }
    }

    fn opts(o: &[(&str, &str)]) -> Vec<(String, String)> {
        o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn flashed(o: &[(&str, &str)]) -> Device {
        let mut all = vec![("flashed", "1")];
        all.extend_from_slice(o);
        let mut dev = Device::new(build(&opts(&all)).unwrap());
        dev.control(SET_CONFIG, &[], &mut Violations::default());
        dev
    }

    /// `do_ep1_command()`: the command scrambled, and its answer read
    /// back and unscrambled.
    fn ep1(dev: &mut Device, cmd: &[u8], answer: usize, v: &mut Violations) -> Option<Vec<u8>> {
        assert_eq!(dev.transfer_out(1, &encrypt(cmd), v), Reply::ack());
        if answer == 0 {
            return None;
        }
        let Reply::Data(d) = dev.transfer_in(1, answer, v) else { panic!("no answer") };
        Some(decrypt(&d))
    }

    fn read(dev: &mut Device, address: u8, v: &mut Violations) -> u8 {
        ep1(dev, &[CMD_FPGA_READ, 1, address], 1, v).unwrap()[0]
    }

    fn write(dev: &mut Device, regs: &[(u8, u8)], v: &mut Violations) {
        let mut cmd = vec![CMD_FPGA_WRITE, regs.len() as u8];
        for (a, b) in regs {
            cmd.extend_from_slice(&[*a, *b]);
        }
        ep1(dev, &cmd, 0, v);
    }

    /// `upload_fpga_bitstream()` from the upload on, the old map.
    fn bring_up(dev: &mut Device, five: bool, v: &mut Violations) {
        let (name, size) = if five { BITSTREAM_33 } else { BITSTREAM_18 };
        ep1(dev, &[CMD_UPLOAD_INIT], 0, v);
        for c in ezusb::standin(name, size).chunks(62) {
            let mut cmd = vec![CMD_UPLOAD_DATA, c.len() as u8];
            cmd.extend_from_slice(c);
            ep1(dev, &cmd, 0, v);
        }
        prime(dev, MAP_OLD, v);
    }

    /// `prime_fpga()`.
    fn prime(dev: &mut Device, m: RegisterMap, v: &mut Violations) {
        let e = ep1(dev, &[CMD_READ_EEPROM, 0x33, 0x81, 16, 16], 16, v).unwrap();
        assert_eq!(e, eeprom()[16..32]);
        let old = read(dev, m.mode, v) & !m.mode_u2;
        let (u1, u2) = (m.mode_u1, m.mode_u2);
        for (i, b) in e.iter().enumerate() {
            let mut regs = Vec::new();
            if i == 0 {
                regs.extend([(m.mode, old), (m.mode, old | u1)]);
            }
            regs.extend([
                (m.primer_data2, *b),
                (m.mode, old | u1 | u2),
                (m.mode, old | u1),
                (m.primer_data1, map_eeprom_data(*b)),
                (m.primer_control, 1),
                (m.primer_control, 0),
            ]);
            write(dev, &regs, v);
        }
        write(dev, &[(m.mode, old)], v);
    }

    /// `configure_led()`.
    fn led(dev: &mut Device, v: &mut Violations) {
        let t = heartbeat();
        for offset in [0u8, 32] {
            let mut cmd = vec![CMD_LED_TABLE, offset, 32];
            cmd.extend_from_slice(&t[usize::from(offset)..usize::from(offset) + 32]);
            ep1(dev, &cmd, 0, v);
        }
        ep1(dev, &[CMD_LED_MODE, 1, 0x6a, 0x18, 0, 1], 0, v);
    }

    /// `logic16_setup_acquisition()` and the start, 1 MHz on `channels`.
    fn capture(dev: &mut Device, channels: u16, v: &mut Violations) {
        let m = MAP_OLD;
        assert_eq!(read(dev, m.status, v), m.unknown1);
        write(dev, &[(m.status, m.unknown2)], v);
        write(dev, &[(m.mode, 0)], v);
        write(dev, &[(m.divisor, 99)], v);
        write(dev, &[(m.channel_low, channels as u8)], v);
        write(dev, &[(m.channel_high, (channels >> 8) as u8)], v);
        write(dev, &[(m.status, m.unknown2 | m.update)], v);
        write(dev, &[(m.status, m.unknown2)], v);
        assert_eq!(read(dev, m.status, v), m.unknown2 | m.unknown1);
        dev.in_submitted(2, 512);
        ep1(dev, &[CMD_START], 0, v);
        write(dev, &[(m.status, m.unknown2 | m.running)], v);
    }

    #[test]
    fn the_scrambling_is_protocol_c_and_undoes_itself() {
        // By hand from encrypt(): 0x7d with the initial state.
        let t = ((0x7du8 ^ 0x54 ^ 0x2b).wrapping_sub(5) ^ 0x35).wrapping_sub(0x39);
        let t = ((t ^ 0x9b ^ 0x5a).wrapping_sub(0xb0) ^ 0x38).wrapping_sub(0x45);
        assert_eq!(encrypt(&[0x7d])[0], t);
        // make_heartbeat(): (uint8_t)(sin(j * G_PI / 8) * 255).
        assert_eq!(heartbeat()[..17], [0, 97, 180, 235, 255, 235, 180, 97, 0, 97, 180, 235, 255, 235, 180, 97, 0]);
        assert_eq!(heartbeat()[16..], [0; 48]);
        let plain: Vec<u8> = (0..=255).collect();
        assert_eq!(decrypt(&encrypt(&plain)), plain);
        assert_ne!(encrypt(&plain), plain);
        assert_eq!(map_eeprom_data(0), ((0x80u8.wrapping_add(0x44)) ^ 0xd5).wrapping_add(0x69));
    }

    #[test]
    fn the_standin_sizes_are_those_of_the_firmware_list() {
        let list = include_str!("../../../tools/emu/saleae-logic16.firmware");
        for (name, size) in [FX2_FIRMWARE, BITSTREAM_18, BITSTREAM_33] {
            let line = list
                .lines()
                .find(|l| l.split_whitespace().next() == Some(name))
                .unwrap_or_else(|| panic!("{name} is not in tools/emu/saleae-logic16.firmware"));
            assert_eq!(line.split_whitespace().nth(1).unwrap().parse::<usize>().unwrap(), size);
        }
    }

    #[test]
    fn uploads_renumerates_configures_and_streams() {
        let mut dev = Device::new(build(&[]).unwrap());
        let mut v = Violations::default();
        assert_eq!(dev.descriptors().strings.len(), 0, "a bare FX2 has no strings");
        assert_eq!(dev.descriptors().product_id(), PID);
        dev.control(SET_CONFIG, &[], &mut v);
        let image = ezusb::standin(FX2_FIRMWARE.0, FX2_FIRMWARE.1);
        dev.control(load(0xe600, 1), &[1], &mut v);
        for (i, c) in image.chunks(4096).enumerate() {
            dev.control(load((i * 4096) as u16, c.len() as u16), c, &mut v);
        }
        dev.control(load(0xe600, 1), &[0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert!(dev.take_renumeration());
        dev.reset();
        assert_eq!(dev.descriptors().strings, ["Saleae LLC", "Logic S/16"]);
        dev.control(SET_CONFIG, &[], &mut v);
        // logic16_init_device().
        assert_eq!(ep1(&mut dev, &[CMD_ABORT_SYNC, 0x55], 1, &mut v), Some(vec![0xaa]));
        assert_eq!(
            ep1(&mut dev, &[CMD_READ_EEPROM, 0x33, 0x81, 8, 8], 8, &mut v),
            Some(eeprom()[8..16].to_vec())
        );
        assert_eq!(read(&mut dev, 0, &mut v), 0, "a blank FPGA");
        bring_up(&mut dev, false, &mut v);
        assert_eq!(read(&mut dev, 0, &mut v), 0x10, "old bitstream version");
        led(&mut dev, &mut v);
        assert!(v.is_empty(), "{v:?}");
        // Channels 0 and 9: words alternate, 16 samples each.
        capture(&mut dev, 0x0201, &mut v);
        let Reply::Data(d) = dev.transfer_in(2, 512, &mut v) else { panic!() };
        assert_eq!(d.len(), 512);
        // Channel 0, samples 0-15: 0101..., sample 0 in bit 15.
        assert_eq!(u16::from_le_bytes([d[0], d[1]]), 0x5555);
        // Channel 9 low for samples below 512.
        assert_eq!(u16::from_le_bytes([d[2], d[3]]), 0);
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak, "one per submission");
        dev.in_submitted(2, 512);
        let Reply::Data(d) = dev.transfer_in(2, 512, &mut v) else { panic!() };
        // Block 128 (sample 2048): channel 9 is bit 9 of 2048, 0; block
        // 160 is sample 2560 = 0b101_0000_0000, high.
        assert_eq!(u16::from_le_bytes([d[2], d[3]]), 0);
        assert!(v.is_empty(), "{v:?}");
        // logic16_abort_acquisition().
        ep1(&mut dev, &[CMD_ABORT_ASYNC], 0, &mut v);
        write(&mut dev, &[(MAP_OLD.status, 0)], &mut v);
        assert_eq!(read(&mut dev, MAP_OLD.status, &mut v), MAP_OLD.unknown1);
        read(&mut dev, 8, &mut v);
        read(&mut dev, 9, &mut v);
        dev.in_submitted(2, 512);
        assert_eq!(dev.transfer_in(2, 512, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_new_bitstream_and_the_mcupro_answer_as_the_driver_tells_them() {
        let mut v = Violations::default();
        let mut dev = flashed(&[("bitstream", "new")]);
        ep1(&mut dev, &[CMD_UPLOAD_INIT], 0, &mut v);
        for c in ezusb::standin(BITSTREAM_33.0, BITSTREAM_33.1).chunks(62) {
            let mut cmd = vec![CMD_UPLOAD_DATA, c.len() as u8];
            cmd.extend_from_slice(c);
            ep1(&mut dev, &cmd, 0, &mut v);
        }
        assert_eq!(read(&mut dev, 0, &mut v), 0);
        assert_eq!(read(&mut dev, 7, &mut v), 0x13);
        prime(&mut dev, MAP_NEW, &mut v);
        assert_eq!(read(&mut dev, MAP_NEW.status, &mut v), MAP_NEW.unknown1);
        assert!(v.is_empty(), "{v:?}");
        let mut dev = flashed(&[("variant", "mcupro")]);
        assert_eq!(read(&mut dev, 0, &mut v), 0x40);
        prime(&mut dev, MAP_OLD, &mut v);
        assert!(v.is_empty(), "{v:?}");
        ep1(&mut dev, &[CMD_UPLOAD_INIT], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "upload to the mcupro");
        assert!(build(&opts(&[("variant", "mcupro"), ("bitstream", "new")])).is_err());
    }

    #[test]
    fn flags_unscrambled_and_malformed_commands() {
        let mut v = Violations::default();
        let mut dev = flashed(&[]);
        // Plain bytes: the model unscrambles them into no command.
        dev.transfer_out(1, &[CMD_ABORT_SYNC, 0x55], &mut v);
        assert_eq!(v.drain().len(), 1, "an unscrambled command");
        // The state carried over from the previous transfer.
        let both = encrypt(&[CMD_ABORT_SYNC, 0x55, CMD_ABORT_SYNC, 0x55]);
        dev.transfer_out(1, &both[2..], &mut v);
        assert!(!v.drain().is_empty(), "scrambled with a carried-over state");
        dev.reset();
        dev.control(SET_CONFIG, &[], &mut v);
        ep1(&mut dev, &[CMD_READ_EEPROM, 0x33, 0x80, 8, 8], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "wrong cookie");
        ep1(&mut dev, &[CMD_READ_EEPROM, 0x33, 0x81, 250, 8], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "beyond the EEPROM");
        ep1(&mut dev, &[CMD_ABORT_SYNC, 0x55], 0, &mut v);
        ep1(&mut dev, &[CMD_START], 0, &mut v);
        assert_eq!(v.drain().len(), 2, "answer not read; start before setup");
        ep1(&mut dev, &[CMD_ABORT_SYNC, 0x55], 0, &mut v);
        dev.transfer_in(1, 64, &mut v);
        assert!(v.is_empty(), "a whole packet asked for a 1 byte answer");
        ep1(&mut dev, &[CMD_READ_EEPROM, 0x33, 0x81, 8, 8], 0, &mut v);
        dev.transfer_in(1, 4, &mut v);
        assert_eq!(v.drain().len(), 1, "an answer longer than the transfer");
        read(&mut dev, 0, &mut v);
        assert!(v.is_empty(), "the mcupro probe");
        read(&mut dev, 0, &mut v);
        assert_eq!(v.drain().len(), 1, "register 0 of a blank FPGA a second time");
        read(&mut dev, 3, &mut v);
        assert_eq!(v.drain().len(), 1, "register 3 of a blank FPGA");
        write(&mut dev, &[(1, 0)], &mut v);
        assert_eq!(v.drain().len(), 1, "write to a blank FPGA");
        ep1(&mut dev, &[CMD_FPGA_READ, 2, 0], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "two registers");
        ep1(&mut dev, &[CMD_UPLOAD_DATA, 1, 0], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "data without init");
        ep1(&mut dev, &[CMD_UPLOAD_DATA, 3, 0], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "chunk length mismatch");
        ep1(&mut dev, &[CMD_UPLOAD_INIT], 0, &mut v);
        ep1(&mut dev, &[CMD_UPLOAD_DATA, 2, 1, 2], 0, &mut v);
        ep1(&mut dev, &[CMD_ABORT_ASYNC], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "a bitstream that is no stand-in");
        ep1(&mut dev, &[CMD_UPLOAD_INIT], 0, &mut v);
        ep1(&mut dev, &[CMD_UPLOAD_DATA, 2, 1, 2], 0, &mut v);
        ep1(&mut dev, &[CMD_UPLOAD_DATA, 2, 1, 2], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "a chunk after a short one");
        ep1(&mut dev, &[CMD_LED_TABLE, 40, 32, 0], 0, &mut v);
        assert_eq!(v.drain().len(), 2, "LED table beyond 64; the upload was no stand-in");
        ep1(&mut dev, &[CMD_LED_TABLE, 0, 33].iter().copied().chain([0; 33]).collect::<Vec<_>>(), 0, &mut v);
        assert_eq!(v.drain().len(), 1, "LED table chunk over 32");
        ep1(&mut dev, &[CMD_LED_MODE, 1, 0x6a, 0x18, 0, 2], 0, &mut v);
        assert_eq!(v.drain().len(), 2, "LED mode arguments, and no heartbeat table");
        ep1(&mut dev, &[CMD_REVID], 0, &mut v);
        assert_eq!(v.drain().len(), 1, "a command the driver never sends");
        assert_eq!(dev.control(Setup { request_type: 0xc0, ..load(0, 1) }, &[], &mut v), Reply::ack());
        assert_eq!(v.drain().len(), 1, "firmware load to the running firmware");
        dev.transfer_out(1, &[0; 65], &mut v);
        assert_eq!(v.drain().len(), 1, "a command over 64 bytes");
    }

    #[test]
    fn flags_a_wrong_primer_and_captures_the_fpga_refuses() {
        let mut v = Violations::default();
        let mut dev = flashed(&[]);
        ep1(&mut dev, &[CMD_UPLOAD_INIT], 0, &mut v);
        for c in ezusb::standin(BITSTREAM_18.0, BITSTREAM_18.1).chunks(62) {
            let mut cmd = vec![CMD_UPLOAD_DATA, c.len() as u8];
            cmd.extend_from_slice(c);
            ep1(&mut dev, &cmd, 0, &mut v);
        }
        let m = MAP_OLD;
        // A pair with the raw byte twice.
        let b = eeprom()[16];
        write(&mut dev, &[(m.primer_data2, b), (m.primer_data1, b), (m.primer_control, 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "data1 not mapped, no MODE bracket");
        write(&mut dev, &[(m.primer_control, 0)], &mut v);
        write(&mut dev, &[(m.version, 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "the version register");
        write(&mut dev, &[(9, 1)], &mut v);
        assert_eq!(v.drain().len(), 1, "a register outside the map");
        read(&mut dev, 13, &mut v);
        assert_eq!(v.drain().len(), 1, "a read outside the map");
        write(&mut dev, &[(m.divisor, 0), (m.mode, m.clock), (m.channel_low, 0xff)]
            , &mut v);
        write(&mut dev, &[(m.channel_high, 0xff), (m.status, m.update | m.overflow)], &mut v);
        // Not primed, 160 MHz, 16 channels at it, and a bit not the host's.
        assert_eq!(v.drain().len(), 4, "not primed, 160 MHz, 16 x 160 MHz, the overflow bit");
        write(&mut dev, &[(m.status, m.running)], &mut v);
        assert_eq!(v.drain().len(), 1, "running without the start command");
        dev.in_submitted(2, 512);
        dev.transfer_in(2, 500, &mut v);
        assert_eq!(v.drain().len(), 1, "not a multiple of 512");
    }

    #[test]
    fn the_primer_is_held_to_prime_fpga_s_sequence() {
        let m = MAP_OLD;
        let e = eeprom();
        let pair = |i: usize| {
            [
                (m.primer_data2, e[16 + i]),
                (m.mode, 0xc0),
                (m.mode, 0x40),
                (m.primer_data1, map_eeprom_data(e[16 + i])),
                (m.primer_control, 1),
                (m.primer_control, 0),
            ]
        };
        let fresh = |v: &mut Violations| {
            let mut dev = flashed(&[]);
            ep1(&mut dev, &[CMD_UPLOAD_INIT], 0, v);
            for c in ezusb::standin(BITSTREAM_18.0, BITSTREAM_18.1).chunks(62) {
                let mut cmd = vec![CMD_UPLOAD_DATA, c.len() as u8];
                cmd.extend_from_slice(c);
                ep1(&mut dev, &cmd, 0, v);
            }
            dev
        };
        let mut v = Violations::default();
        // The first pair without the opening MODE writes.
        let mut dev = fresh(&mut v);
        write(&mut dev, &pair(0), &mut v);
        assert_eq!(v.drain().len(), 1, "no MODE x, x|UNKNOWN1 before the first pair");
        // UNKNOWN2 not raised between the data writes.
        let mut dev = fresh(&mut v);
        let mut regs = vec![(m.mode, 0), (m.mode, 0x40)];
        regs.extend(pair(0));
        regs[3].1 = 0x40;
        write(&mut dev, &regs, &mut v);
        assert_eq!(v.drain().len(), 1, "MODE without UNKNOWN2 in the pulse");
        // PRIMER_CONTROL left at 1.
        let mut dev = fresh(&mut v);
        let mut regs = vec![(m.mode, 0), (m.mode, 0x40)];
        regs.extend(&pair(0)[..5]);
        write(&mut dev, &regs, &mut v);
        write(&mut dev, &pair(1)[..1], &mut v);
        assert_eq!(v.drain().len(), 1, "PRIMER_CONTROL not back to 0");
        // All 16 pairs, but no MODE restore before the capture's UPDATE.
        let mut dev = fresh(&mut v);
        let mut regs = vec![(m.mode, 0), (m.mode, 0x40)];
        regs.extend(pair(0));
        write(&mut dev, &regs, &mut v);
        for i in 1..16 {
            write(&mut dev, &pair(i), &mut v);
        }
        assert!(v.is_empty(), "{v:?}");
        write(&mut dev, &[(m.divisor, 99)], &mut v);
        assert_eq!(v.drain().len(), 0, "the divisor is no primer register");
        write(&mut dev, &[(m.mode, 0x40)], &mut v);
        assert_eq!(v.drain().len(), 1, "the mode not restored");
        write(&mut dev, &[(m.channel_low, 1), (m.status, m.update)], &mut v);
        assert_eq!(v.drain().len(), 1, "capture set up before the sequence ended");
    }

    #[test]
    fn a_full_bring_up_is_clean_and_faults_answer_wrong() {
        let mut v = Violations::default();
        let mut dev = flashed(&[]);
        bring_up(&mut dev, true, &mut v);
        assert!(v.is_empty(), "{v:?}");
        let mut v = Violations::default();
        let mut dev = flashed(&[
            ("stall", "fpga-read:2"),
            ("stall_reply", "read-eeprom:1"),
            ("short_reply", "abort-sync"),
            ("fpga_version", "0x22"),
        ]);
        assert_eq!(dev.transfer_out(1, &encrypt(&[CMD_ABORT_SYNC, 0x55]), &mut v), Reply::ack());
        assert_eq!(dev.transfer_in(1, 1, &mut v), Reply::Data(vec![]));
        dev.transfer_out(1, &encrypt(&[CMD_READ_EEPROM, 0x33, 0x81, 8, 8]), &mut v);
        assert_eq!(dev.transfer_in(1, 8, &mut v), Reply::Stall);
        assert_eq!(read(&mut dev, 0, &mut v), 0);
        let second = encrypt(&[CMD_FPGA_READ, 1, 0]);
        assert_eq!(dev.transfer_out(1, &second, &mut v), Reply::Stall, "the second read");
        assert_eq!(dev.transfer_in(1, 1, &mut v), Reply::Nak, "a STALLed command has no answer");
        bring_up(&mut dev, false, &mut v);
        assert_eq!(read(&mut dev, 0, &mut v), 0x22);
        assert!(v.is_empty(), "a fault is the device's, not the host's: {v:?}");
        let mut dev = flashed(&[("abort_reply", "0x55"), ("status_xor", "0x08"), ("overflow", "1")]);
        assert_eq!(ep1(&mut dev, &[CMD_ABORT_SYNC, 0x55], 1, &mut v), Some(vec![0x55]));
        bring_up(&mut dev, false, &mut v);
        assert_eq!(read(&mut dev, MAP_OLD.status, &mut v), 0);
        let mut dev = flashed(&[("overflow", "1")]);
        bring_up(&mut dev, false, &mut v);
        capture(&mut dev, 1, &mut v);
        ep1(&mut dev, &[CMD_ABORT_ASYNC], 0, &mut v);
        write(&mut dev, &[(MAP_OLD.status, 0)], &mut v);
        assert_eq!(read(&mut dev, MAP_OLD.status, &mut v), MAP_OLD.unknown1 | MAP_OLD.overflow);
        assert!(v.is_empty(), "{v:?}");
        for bad in [
            ("stall", "nope"),
            ("stall", "start:0"),
            ("abort_reply", "256"),
            ("short_in", "x"),
            ("bitstream", "x"),
            ("flashed", "yes"),
        ] {
            assert!(build(&opts(&[bad])).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn stream_faults_are_paced_by_submissions() {
        let mut v = Violations::default();
        let mut dev = flashed(&[("empty", "1"), ("stall_in", "1"), ("short_in", "1")]);
        bring_up(&mut dev, false, &mut v);
        capture(&mut dev, 1, &mut v);
        let mut got = Vec::new();
        for _ in 0..4 {
            got.push(dev.transfer_in(2, 512, &mut v));
            dev.in_submitted(2, 512);
        }
        assert_eq!(got[..2], [Reply::Data(vec![]), Reply::Stall]);
        assert_eq!(got[2], Reply::Data(vec![0x55]), "one byte of the first word");
        let Reply::Data(d) = &got[3] else { panic!() };
        assert_eq!(d.len(), 512);
        assert!(v.is_empty(), "{v:?}");
    }
}
