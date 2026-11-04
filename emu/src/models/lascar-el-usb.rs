// Counterpart in sigrok:
//   libsigrok/src/hardware/lascar-el-usb/protocol.c (the commands, the configuration block, the sample bytes),
//   libsigrok/src/hardware/lascar-el-usb/protocol.h,
//   libsigrok/src/hardware/lascar-el-usb/api.c (the log download),
//   Copyright (C) 2012 Bert Vermeulen <bert@biot.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Martin Ling, Soeren Apel, Aurelien Jacobs, Daniel Glöckner,
//   Gerhard Sittig, Frank Stettner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Lascar `EasyLog` EL-USB data loggers: a `SiLabs` C8051F321 running
//! `USBXpress`, an EEPROM holding the log, and a sensor. The host never
//! streams: it downloads a stored log.
//!
//! Sources, and how sure each fact is:
//! - **Descriptors**, from the `lsusb -vvv` of an EL-USB-2 and an EL-USB-CO
//!   on the sigrok wiki (`Lascar_Electronics_EL-USB-2/Info`,
//!   `Lascar_Electronics_EL-USB-CO/Info`), identical but for the bus:
//!   `10c4:0002`, USB 1.10, bcdDevice 1.00, EP0 64 bytes, strings "Silicon
//!   Labs", "USB API", "1", bus powered 60 mA, one vendor interface with
//!   bulk `0x02` and `0x82` of 64 bytes. These are recordings of real units;
//!   the other models are not recorded. Full speed follows from USB 1.10 and
//!   the F321.
//! - **The commands and the configuration block**, from the wiki's
//!   `Lascar_Electronics_EL-USB_protocol` and the C driver, which agree but
//!   for one bit: the wiki puts "logging start" at bit 4 of 0x21, while the
//!   C driver (`protocol.c:609`, mask 0x0100) and eltuxusb 0.3
//!   (`el_device.py:222`, `flag_bits[8]`) both use bit 0. The model follows
//!   the two programs. The wiki's prose also says the count is at 0x2e; its
//!   own table and the C say 0x1e.
//! - **The sample bytes.** Temperature and humidity pairs, 0xff after the
//!   log, from eltuxusb's `el_parse.py` (`while value != 255`, `(raw - 80) /
//!   2` Celsius, `raw - 40` Fahrenheit, `raw / 2` %RH) and the C. The CO
//!   format is the C's alone: big endian counts (`protocol.c:483`).
//! - **Block sizes.** eltuxusb's `device_nb_packets` reads N times with a
//!   4096 byte buffer and gets the memory: EL-USB-2 64 times, EL-USB-CO 127
//!   times, the 64 byte EL-USB-1 256 times. So the device ends a transfer
//!   every 512 bytes (64 on that EL-USB-1), an exact multiple of the packet
//!   size, which takes a zero length packet. With the wiki's 32 kB and 64 kB
//!   EEPROMs this gives the header's lengths, 32768 and 65024. This is
//!   inference, from a program and not a recording.
//! - **Configuration sizes.** eltuxusb's table: 64 for EL-USB-1, 128 for
//!   EL-USB-2, 256 for the others (a commented-out table beside it says 128
//!   for the 2-LCD and 2+). Ids 14 to 21 are in neither; the model takes 256
//!   and 32768, unverified.
//! - **Field values** in the default block (the name aside) come from the
//!   one real block found, eltuxusb's `backup_buffer` from an EL-USB-1: start
//!   time, interval 60 s, alarms, calibration 0.5 and -40, firmware `v2.0`,
//!   serial. What a real CO unit holds for its calibration is unknown.
//! - **The vendor requests** `0x00/0xffff`, `0x02/0x0002`, `0x02/0x0001` are
//!   the C's ("Some of these fail, but it needs doing", `protocol.c:69`).
//!   Which ones fail on a real unit is not known, so by default none does;
//!   `stall-ctl=` makes them.
//!
//! No firmware source or USB capture was found; the vendor's `EasyLog`
//! software was not examined.
//!
//! Pedantic checks: a vendor request other than the C's three; a command of
//! a length or with an argument the C does not use; a command with data
//! still queued (the C flushes first: "The F321 requires this"); a command
//! before any mode setup, a log download without a new one; a config write
//! of the wrong length or changing a field the C does not change; a read
//! shorter than its answer; a log read other than 4096 bytes.
//!
//! Not checked: that a read waits when a command comes, though the C submits
//! it first ("Keep a read request waiting in the wings", `protocol.c:112`).
//! That comment is no fact about the device, and eltuxusb, written against
//! real EL-USB-1 and 2 units, writes each command and reads afterwards
//! (`el_device.py:259-268`). It sends no flush either, so that check rests on
//! the C's word alone.
//!
//! C habits, documented and not flagged: the scan's bulk reads come with no
//! claimed interface (usbfs claims it itself; the bus shows nothing) and
//! `protocol.c:591` clears the top three bits of byte 0x21 on every write.
//!
//! Emulation artifacts: a log read is answered at once, one block per
//! transfer, with the block's end standing in for the zero length packet;
//! one-shot STALLs (`stall-in`, `stall-out`) release themselves, where a
//! real halt lasts until `CLEAR_FEATURE` (`halt-in` is that one); a write
//! changes the block in RAM only, and the harness's reset builds a fresh
//! model, so nothing persists between runs.
//!
//! Options: `model=ID` (0 to 255, default 3), `logged=N` (samples, default
//! 20), `interval=S`, `unit=N` (0x2e), `co-high=F`, `co-low=F`,
//! `logging=0|1`, `flag-bits=HEX` (byte 0x21), `config-size=N`,
//! `memory=BYTES`, `block=BYTES`, `pattern=whole|half`, `invalid=K` (each
//! Kth pair is the one the C calls invalid: raw 80, 0, or 40, 0 with
//! `unit=1`).
//!
//! Faults, off by default (N counts from 1): `cfg-silent=N`, `cfg-garbage=N`,
//! `cfg-long=N`, `cfg-short-header=N`, `cfg-short=N`, `cfg-nobody=N` for the
//! Nth configuration read; `tail=BYTES` after each configuration block;
//! `stale=BYTES` queued from the start; `stall-ctl=REQ/VALUE`; `stall-in=N`,
//! `halt-in=N` for the Nth answer on the IN endpoint, `stall-out=N` for the
//! Nth OUT transfer; `save-silent=1`, `save-answer=BYTE`; `log-garbage=1`,
//! `log-short-header=1`, `log-size=BYTES` (the header only), `log-stop=K`
//! (no blocks after the Kth, K from 0), `hold=K` and `hold-empty=K` (the Kth block's
//! read waits for the host's cancel, which takes 64 bytes or none).

use std::collections::VecDeque;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{BULK, Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "lascar-el-usb";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Lascar EL-USB data loggers (SiLabs F321, 10c4:0002); model=ID (default 3, \
     EL-USB-2), logged=N, interval=S, unit=N, co-high=F, co-low=F, logging=0|1, flag-bits=HEX, \
     config-size=N, memory=N, block=N, pattern=whole|half, invalid=K; faults: cfg-silent=N, \
     cfg-garbage=N, cfg-long=N, cfg-short-header=N, cfg-short=N, cfg-nobody=N, tail=N, stale=N, \
     stall-ctl=REQ/VALUE, stall-in=N, halt-in=N, stall-out=N, save-silent=1, save-answer=B, \
     log-garbage=1, log-short-header=1, log-size=N, log-stop=K, hold=K, hold-empty=K";

/// The endpoint number, both directions.
const EP: u8 = 2;
/// The C driver's log read (`api.c:360`).
const LOG_READ: usize = 4096;
/// The C's largest configuration block (`protocol.h:32`).
const MAX_CONFIG: usize = 256;

/// `(configuration bytes, memory bytes, block bytes)` by model id, from
/// eltuxusb's `el_devices_settings.py`, unverified for ids above 13.
fn sizes(id: u8) -> (usize, usize, usize) {
    match id {
        1 => (64, 16384, 64),
        2 => (64, 16384, 512),
        3 => (128, 32768, 512),
        8 => (256, 5120, 512),
        4..=7 | 9..=11 => (256, 65024, 512),
        _ => (256, 32768, 512),
    }
}

/// CO models: 16 bit samples. The others here are temperature and humidity
/// pairs; the C's unsupported ones never get as far as a download.
fn is_co(id: u8) -> bool {
    matches!(id, 9 | 11)
}

/// The sample pattern.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pattern {
    /// Whole degrees and percents: even raw bytes.
    Whole,
    /// Half steps every other sample: odd raw bytes.
    Half,
}

/// Options, as the `-o` values give them.
#[derive(Clone, Debug)]
struct Options {
    id: u8,
    logged: u16,
    interval: u16,
    unit: u16,
    co_high: f32,
    co_low: f32,
    logging: bool,
    flag_bits: Option<u8>,
    config_size: Option<usize>,
    memory: Option<usize>,
    block: Option<usize>,
    pattern: Pattern,
    invalid: Option<usize>,
    faults: Faults,
}

/// The fault options; none by default.
#[derive(Clone, Debug, Default)]
struct Faults {
    cfg_silent: Option<u32>,
    cfg_garbage: Option<u32>,
    cfg_long: Option<u32>,
    cfg_short_header: Option<u32>,
    cfg_short: Option<u32>,
    cfg_nobody: Option<u32>,
    tail: usize,
    stale: usize,
    stall_ctl: Vec<(u8, u16)>,
    stall_in: Option<u32>,
    halt_in: Option<u32>,
    stall_out: Option<u32>,
    save_silent: bool,
    save_answer: Option<u8>,
    log_garbage: bool,
    log_short_header: bool,
    log_size: Option<u16>,
    log_stop: Option<u32>,
    hold: Option<u32>,
    hold_empty: Option<u32>,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            id: 3,
            logged: 20,
            interval: 60,
            unit: 0,
            co_high: 0.5,
            co_low: -40.0,
            logging: false,
            flag_bits: None,
            config_size: None,
            memory: None,
            block: None,
            pattern: Pattern::Whole,
            invalid: None,
            faults: Faults::default(),
        }
    }
}

fn number<T: std::str::FromStr>(k: &str, v: &str) -> Result<T> {
    v.parse().map_err(|_| Error(format!("{k}={v}: a number")))
}

fn nth(k: &str, v: &str) -> Result<Option<u32>> {
    v.parse().ok().filter(|n| *n > 0).map(Some).ok_or_else(|| Error(format!("{k}={v}: N from 1")))
}

fn hex_u16(k: &str, v: &str) -> Result<u16> {
    u16::from_str_radix(v.trim_start_matches("0x"), 16).map_err(|_| Error(format!("{k}={v}: hex")))
}

fn hex_u8(k: &str, v: &str) -> Result<u8> {
    u8::from_str_radix(v.trim_start_matches("0x"), 16).map_err(|_| Error(format!("{k}={v}: hex")))
}

fn bounded(k: &str, v: &str, low: usize, high: usize) -> Result<usize> {
    let n: usize = number(k, v)?;
    if n < low || n > high {
        return Err(Error(format!("{k}={v}: {low} to {high}")));
    }
    Ok(n)
}

impl Options {
    fn parse(options: &[(String, String)]) -> Result<Options> {
        let mut o = Options::default();
        for (k, v) in options {
            let f = &mut o.faults;
            match k.as_str() {
                "model" => o.id = number(k, v)?,
                "logged" => o.logged = number(k, v)?,
                "interval" => o.interval = number(k, v)?,
                "unit" => o.unit = number(k, v)?,
                "co-high" => o.co_high = number(k, v)?,
                "co-low" => o.co_low = number(k, v)?,
                "logging" => o.logging = number::<u8>(k, v)? != 0,
                "flag-bits" => o.flag_bits = Some(hex_u8(k, v)?),
                "config-size" => o.config_size = Some(bounded(k, v, 0, 0xffff)?),
                // The header's length holds 16 bits.
                "memory" => o.memory = Some(bounded(k, v, 0, 0xffff)?),
                "block" => o.block = Some(bounded(k, v, 1, LOG_READ)?),
                "pattern" => {
                    o.pattern = match v.as_str() {
                        "whole" => Pattern::Whole,
                        "half" => Pattern::Half,
                        _ => return Err(Error(format!("{k}={v}: whole or half"))),
                    };
                }
                "invalid" => o.invalid = Some(bounded(k, v, 1, 0xffff)?),
                "cfg-silent" => f.cfg_silent = nth(k, v)?,
                "cfg-garbage" => f.cfg_garbage = nth(k, v)?,
                "cfg-long" => f.cfg_long = nth(k, v)?,
                "cfg-short-header" => f.cfg_short_header = nth(k, v)?,
                "cfg-short" => f.cfg_short = nth(k, v)?,
                "cfg-nobody" => f.cfg_nobody = nth(k, v)?,
                "tail" => f.tail = bounded(k, v, 0, LOG_READ)?,
                "stale" => f.stale = bounded(k, v, 0, LOG_READ)?,
                "stall-ctl" => {
                    let (r, val) = v
                        .split_once('/')
                        .ok_or_else(|| Error(format!("{k}={v}: REQ/VALUE in hex")))?;
                    f.stall_ctl.push((hex_u8(k, r)?, hex_u16(k, val)?));
                }
                "stall-in" => f.stall_in = nth(k, v)?,
                "halt-in" => f.halt_in = nth(k, v)?,
                "stall-out" => f.stall_out = nth(k, v)?,
                "save-silent" => f.save_silent = number::<u8>(k, v)? != 0,
                "save-answer" => f.save_answer = Some(hex_u8(k, v)?),
                "log-garbage" => f.log_garbage = number::<u8>(k, v)? != 0,
                "log-short-header" => f.log_short_header = number::<u8>(k, v)? != 0,
                "log-size" => f.log_size = Some(number(k, v)?),
                "log-stop" => f.log_stop = Some(number(k, v)?),
                "hold" => f.hold = nth(k, v)?,
                "hold-empty" => f.hold_empty = nth(k, v)?,
                other => return Err(Error(format!("{NAME} has no option {other}"))),
            }
        }
        Ok(o)
    }
}

/// A logger from the `-o` options, see the module documentation.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(ElUsb::new(Options::parse(options)?)))
}

/// The descriptors, from the wiki's `lsusb -vvv`.
fn descriptors() -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 2, [0, 0, 0], 0));
    body.extend_from_slice(&descriptor::endpoint(0x02, BULK, 64, 0));
    body.extend_from_slice(&descriptor::endpoint(0x82, BULK, 64, 0));
    let config = descriptor::configuration(1, 0, 0x80, 60, &body);
    let dev = DeviceDescriptor {
        usb: 0x0110,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: 0x10c4,
        product: 0x0002,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::Full,
        device: dev.bytes(),
        configs: vec![config],
        strings: vec!["Silicon Labs".into(), "USB API".into(), "1".into()],
    }
}

/// Which vendor requests of the mode setup have come, in order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Mode {
    /// None since the last reset, or an unfinished sequence.
    #[default]
    None,
    /// `0x00/0xffff`.
    First,
    /// and `0x02/0x0002`.
    Second,
    /// and `0x02/0x0001`.
    Done,
}

/// A log download under way: the blocks come after the header.
#[derive(Clone, Copy, Debug, Default)]
struct Download {
    offset: usize,
    /// Reads answered, so the block in hand is number `reads + 1`.
    reads: u32,
}

/// The logger.
#[allow(clippy::struct_excessive_bools)] // independent states of the protocol
pub struct ElUsb {
    o: Options,
    descriptors: Descriptors,
    /// The configuration block as the EEPROM holds it.
    config: Vec<u8>,
    /// The sample memory, computed here and not inside a reply.
    memory: Vec<u8>,
    /// Answers waiting for the host, one transfer each.
    queue: VecDeque<Vec<u8>>,
    download: Option<Download>,
    mode: Mode,
    /// A mode setup has completed since the last log download.
    fresh_mode: bool,
    /// Ever completed one.
    any_mode: bool,
    /// What has come of a configuration write.
    saving: Option<Vec<u8>>,
    /// The write's command was refused and the block is yet to come.
    discard_block: bool,
    config_reads: u32,
    answers: u32,
    outs: u32,
    halted: bool,
    /// The read of this block is being held for the host's cancel.
    held: bool,
}

impl ElUsb {
    fn new(o: Options) -> ElUsb {
        let (config_size, memory, _) = sizes(o.id);
        let config = make_config(&o, o.config_size.unwrap_or(config_size));
        let memory = make_memory(&o, o.memory.unwrap_or(memory));
        let mut dev = ElUsb {
            o,
            descriptors: descriptors(),
            config,
            memory,
            queue: VecDeque::new(),
            download: None,
            mode: Mode::None,
            fresh_mode: false,
            any_mode: false,
            saving: None,
            discard_block: false,
            config_reads: 0,
            answers: 0,
            outs: 0,
            halted: false,
            held: false,
        };
        dev.start_over();
        dev
    }

    fn start_over(&mut self) {
        self.queue.clear();
        let mut left = self.o.faults.stale;
        let mut i = 0u8;
        while left > 0 {
            let n = left.min(64);
            self.queue.push_back(vec![i; n]);
            left -= n;
            i = i.wrapping_add(1);
        }
        self.download = None;
        self.mode = Mode::None;
        self.fresh_mode = false;
        self.any_mode = false;
        self.saving = None;
        self.discard_block = false;
        self.config_reads = 0;
        self.answers = 0;
        self.outs = 0;
        self.halted = false;
        self.held = false;
    }

    fn block_size(&self) -> usize {
        self.o.block.unwrap_or(sizes(self.o.id).2)
    }

    /// The configuration read: the header, then the block, each a transfer.
    fn read_config(&mut self) {
        self.config_reads += 1;
        let n = Some(self.config_reads);
        let f = &self.o.faults;
        if f.cfg_silent == n {
            return;
        }
        let len = self.config.len();
        let [lo, hi] = u16::try_from(len).unwrap_or(u16::MAX).to_le_bytes();
        if f.cfg_garbage == n {
            self.queue.push_back(vec![0x07, lo, hi]);
            return;
        }
        if f.cfg_long == n {
            let [lo, hi] = u16::try_from(MAX_CONFIG + 1).unwrap_or(u16::MAX).to_le_bytes();
            self.queue.push_back(vec![0x02, lo, hi]);
            return;
        }
        if f.cfg_short_header == n {
            self.queue.push_back(vec![0x02, lo]);
            return;
        }
        self.queue.push_back(vec![0x02, lo, hi]);
        if f.cfg_nobody == n {
            return;
        }
        if f.cfg_short == n {
            self.queue.push_back(self.config[..len / 2].to_vec());
            return;
        }
        self.queue.push_back(self.config.clone());
        if f.tail > 0 {
            self.queue.push_back(vec![0x5a; f.tail]);
        }
    }

    /// The log download: the header now, the blocks when asked for.
    fn read_log(&mut self) {
        let f = &self.o.faults;
        let len = f.log_size.unwrap_or(u16::try_from(self.memory.len()).unwrap_or(u16::MAX));
        let [lo, hi] = len.to_le_bytes();
        if f.log_garbage {
            self.queue.push_back(vec![0x07, lo, hi]);
        } else if f.log_short_header {
            self.queue.push_back(vec![0x02, lo]);
        } else {
            self.queue.push_back(vec![0x02, lo, hi]);
            self.download = Some(Download::default());
        }
    }

    fn command(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if data.len() != 3 {
            v.report(format!("command of {} bytes; commands have 3", data.len()));
            return Reply::ack();
        }
        if !self.queue.is_empty() || self.download.is_some() {
            v.report("command with data still queued; the C flushes first");
        }
        if !self.any_mode {
            v.report("command before the mode setup requests");
        }
        match data {
            [0x00, 0xff, 0xff] => self.read_config(),
            [0x01, lo, hi] => {
                let len = usize::from(u16::from_le_bytes([*lo, *hi]));
                if len != self.config.len() {
                    v.report(format!(
                        "configuration write of {len} bytes; the block has {}",
                        self.config.len()
                    ));
                }
                self.saving = Some(Vec::new());
            }
            [0x03, 0xff, 0xff] => {
                if !self.fresh_mode {
                    v.report("log download without a new mode setup");
                }
                self.fresh_mode = false;
                self.read_log();
            }
            other => v.report(format!("unknown command {other:02x?}")),
        }
        Reply::ack()
    }

    /// Takes a part of the configuration write; the whole one is checked
    /// against what `protocol.c` changes: bit 8 of the flags, the top three
    /// bits of byte 0x21 cleared, and 0x18 to 0x1b zeroed on a start.
    fn save(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let Some(buf) = &mut self.saving else {
            return Reply::ack();
        };
        buf.extend_from_slice(data);
        if buf.len() < self.config.len() {
            return Reply::ack();
        }
        let new = self.saving.take().unwrap_or_default();
        if new.len() != self.config.len() {
            v.report(format!(
                "configuration of {} bytes, the block has {}",
                new.len(),
                self.config.len()
            ));
            return Reply::ack();
        }
        for (i, (old, n)) in self.config.iter().zip(&new).enumerate() {
            let allowed = match i {
                0x18..=0x1b => n == old || *n == 0,
                0x21 => n & !0x01 == old & 0x1e,
                _ => n == old,
            };
            if !allowed {
                v.report(format!("configuration byte {i:#04x} {old:#04x} written as {n:#04x}"));
            }
        }
        self.config = new;
        if !self.o.faults.save_silent {
            self.queue.push_back(vec![self.o.faults.save_answer.unwrap_or(0xff)]);
        }
        Reply::ack()
    }

    /// The next log block, up to `max` bytes.
    fn next_block(&mut self, max: usize) -> Vec<u8> {
        let block = self.block_size();
        let Some(d) = self.download.as_mut() else { return Vec::new() };
        let end = (d.offset + block.min(max)).min(self.memory.len());
        let data = self.memory[d.offset..end].to_vec();
        d.offset = end;
        d.reads += 1;
        if end >= self.memory.len() {
            self.download = None;
        }
        data
    }
}

/// The block's default contents, in the wiki's layout, values from
/// eltuxusb's `backup_buffer` where it has them.
fn make_config(o: &Options, size: usize) -> Vec<u8> {
    let mut c = vec![0u8; size.max(0x40)];
    c[0] = o.id;
    let name = b"oxrok";
    c[2..2 + name.len()].copy_from_slice(name);
    c[0x12..0x18].copy_from_slice(&[16, 8, 41, 14, 1, 10]);
    c[0x18..0x1c].copy_from_slice(&3600u32.to_le_bytes());
    c[0x1c..0x1e].copy_from_slice(&o.interval.to_le_bytes());
    c[0x1e..0x20].copy_from_slice(&o.logged.to_le_bytes());
    c[0x20] = 4;
    c[0x21] = o.flag_bits.unwrap_or(0) | u8::from(o.logging);
    c[0x22] = 140;
    c[0x23] = 80;
    c[0x24..0x28].copy_from_slice(&o.co_high.to_le_bytes());
    c[0x28..0x2c].copy_from_slice(&o.co_low.to_le_bytes());
    c[0x2e..0x30].copy_from_slice(&o.unit.to_le_bytes());
    c[0x30..0x34].copy_from_slice(b"v2.0");
    c[0x34..0x36].copy_from_slice(&[121, 243]);
    c.truncate(size);
    c
}

/// The sample memory: the log, then 0xff as an erased EEPROM reads.
fn make_memory(o: &Options, size: usize) -> Vec<u8> {
    let mut m = vec![0xff; size];
    for (i, pair) in m.as_chunks_mut::<2>().0.iter_mut().take(usize::from(o.logged)).enumerate() {
        pair.copy_from_slice(&sample(o, i));
    }
    m
}

/// The two bytes of sample `i`.
fn sample(o: &Options, i: usize) -> [u8; 2] {
    if is_co(o.id) {
        return u16::try_from((60 + 17 * i) % 0x1_0000).unwrap_or(0).to_be_bytes();
    }
    if o.invalid.is_some_and(|k| i % k == k - 1) {
        // What protocol.c:424-432 reads as 0 degrees and 0 %.
        return [if o.unit == 1 { 40 } else { 80 }, 0];
    }
    let t = 120 + 2 * (i * 3 % 11);
    let h = 80 + 2 * (i % 37);
    let odd = usize::from(o.pattern == Pattern::Half && i % 2 == 1);
    [u8::try_from(t + odd).unwrap_or(255), u8::try_from(h + odd).unwrap_or(255)]
}

impl DeviceModel for ElUsb {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let known = setup.request_type == 0x40
            && setup.index == 0
            && setup.length == 0
            && data.is_empty()
            && matches!((setup.request, setup.value), (0, 0xffff) | (2, 1 | 2));
        if !known {
            v.report(format!(
                "control {:02x} {:02x} {:04x} {:04x} {}: not one of the C's mode setup requests",
                setup.request_type, setup.request, setup.value, setup.index, setup.length
            ));
            return Reply::Stall;
        }
        self.mode = match (self.mode, setup.request, setup.value) {
            (_, 0, _) => Mode::First,
            (Mode::First, 2, 2) => Mode::Second,
            (Mode::Second, 2, 1) => Mode::Done,
            _ => Mode::None,
        };
        if self.mode == Mode::Done {
            self.any_mode = true;
            self.fresh_mode = true;
        }
        // The C ignores the outcome of each, so a refused one still counts.
        if self.o.faults.stall_ctl.contains(&(setup.request, setup.value)) {
            return Reply::Stall;
        }
        Reply::ack()
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP {
            v.report(format!("OUT on endpoint {ep}"));
            return Reply::Stall;
        }
        self.outs += 1;
        if self.o.faults.stall_out == Some(self.outs) {
            // protocol.c:250-260 sends the block whatever became of the
            // command before it: a block to throw away, not to flag.
            self.discard_block = self.saving.is_none() && data.first() == Some(&0x01);
            return Reply::Stall;
        }
        if self.discard_block {
            self.discard_block = false;
            return Reply::ack();
        }
        if self.saving.is_some() { self.save(data, v) } else { self.command(data, v) }
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP {
            v.report(format!("IN on endpoint {ep}"));
            return Reply::Stall;
        }
        if self.halted {
            return Reply::Stall;
        }
        let in_blocks = self.queue.is_empty() && self.download.is_some();
        if in_blocks {
            let d = self.download.unwrap_or_default();
            let f = &self.o.faults;
            if f.hold == Some(d.reads + 1) || f.hold_empty == Some(d.reads + 1) {
                self.held = true;
                return Reply::Nak;
            }
            if f.log_stop.is_some_and(|k| d.reads >= k) {
                return Reply::Nak;
            }
            if max != LOG_READ {
                v.report(format!("log read of {max} bytes; the driver asks for {LOG_READ}"));
            }
        } else if self.queue.is_empty() {
            return Reply::Nak;
        }
        // An answer, data or a fault.
        self.answers += 1;
        if self.o.faults.halt_in == Some(self.answers) {
            self.halted = true;
            return Reply::Stall;
        }
        if self.o.faults.stall_in == Some(self.answers) {
            return Reply::Stall;
        }
        if in_blocks {
            return Reply::Data(self.next_block(max));
        }
        let Some(front) = self.queue.front_mut() else { return Reply::Nak };
        if front.len() > max {
            v.report(format!("read of {max} bytes; the answer has {}", front.len()));
            return Reply::Data(front.drain(..max).collect());
        }
        Reply::Data(self.queue.pop_front().unwrap_or_default())
    }

    /// `hold=K` and `hold-empty=K`: what came of the block by the cancel.
    fn in_unlinked(&mut self, ep: u8, _max: usize) -> Option<Vec<u8>> {
        if ep != EP || !self.held {
            return None;
        }
        self.held = false;
        let d = self.download?;
        if self.o.faults.hold_empty == Some(d.reads + 1) {
            self.o.faults.hold_empty = None;
            return None;
        }
        self.o.faults.hold = None;
        let end = (d.offset + self.block_size().min(64)).min(self.memory.len());
        let data = self.memory[d.offset..end].to_vec();
        if let Some(d) = &mut self.download {
            d.offset = end;
        }
        Some(data)
    }

    fn log_digest_above(&self) -> Option<usize> {
        Some(MAX_CONFIG)
    }

    fn reset(&mut self) {
        self.start_over();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    fn dev(pairs: &[(&str, &str)]) -> ElUsb {
        ElUsb::new(Options::parse(&opts(pairs)).unwrap())
    }

    fn ctl(request: u8, value: u16) -> Setup {
        Setup { request_type: 0x40, request, value, index: 0, length: 0 }
    }

    /// The vendor requests, as `lascar_open()` sends them.
    fn mode(d: &mut ElUsb) {
        let mut v = Violations::default();
        for (r, val) in [(0, 0xffff), (2, 2), (2, 1)] {
            assert_eq!(d.control(ctl(r, val), &[], &mut v), Reply::ack());
        }
        assert!(v.is_empty(), "{v:?}");
    }

    fn cmd(d: &mut ElUsb, bytes: &[u8], v: &mut Violations) {
        d.out(EP, bytes, v);
    }

    fn data(r: Reply) -> Vec<u8> {
        let Reply::Data(d) = r else { panic!("{r:?}") };
        d
    }

    #[test]
    fn descriptors_are_the_wikis_lsusb() {
        let d = dev(&[]);
        let t = d.descriptors();
        assert_eq!(t.vendor_id(), 0x10c4);
        assert_eq!(t.product_id(), 0x0002);
        assert_eq!(t.bcd_device(), 0x0100);
        assert_eq!(t.device[7], 64);
        let eps = Descriptors::endpoints(&t.configs[0]);
        assert_eq!(eps.len(), 2);
        assert_eq!((eps[0].address, eps[0].max_packet), (0x02, 64));
        assert_eq!((eps[1].address, eps[1].max_packet), (0x82, 64));
        assert_eq!(t.configs[0].len(), 32, "wTotalLength");
    }

    #[test]
    fn the_config_read_gives_the_header_then_the_block() {
        for (id, size) in [("1", 64usize), ("2", 64), ("3", 128), ("9", 256), ("21", 256)] {
            let mut d = dev(&[("model", id)]);
            mode(&mut d);
            let mut v = Violations::default();
            cmd(&mut d, &[0, 0xff, 0xff], &mut v);
            let [lo, hi] = u16::try_from(size).unwrap().to_le_bytes();
            assert_eq!(data(d.in_transfer(EP, 256, &mut v)), [2, lo, hi]);
            let c = data(d.in_transfer(EP, size, &mut v));
            assert_eq!(c.len(), size);
            assert_eq!(c[0].to_string(), id);
            assert_eq!(&c[0x30..0x34], b"v2.0");
            assert!(v.is_empty(), "{v:?}");
            assert_eq!(d.in_transfer(EP, 256, &mut v), Reply::Nak, "nothing more");
        }
    }

    #[test]
    fn the_config_holds_what_the_c_reads() {
        let c = make_config(
            &Options { logged: 300, interval: 5, unit: 1, logging: true, ..Options::default() },
            128,
        );
        assert_eq!(u16::from_le_bytes([c[0x1e], c[0x1f]]), 300);
        assert_eq!(u16::from_le_bytes([c[0x1c], c[0x1d]]), 5);
        assert_eq!(u16::from_le_bytes([c[0x2e], c[0x2f]]), 1);
        assert_eq!(c[0x21] & 1, 1);
        // eltuxusb's real EL-USB-1 block holds these bytes: 0.5 and -40.
        assert_eq!(&c[0x24..0x2c], &[0, 0, 0, 63, 0, 0, 32, 194]);
    }

    #[test]
    fn the_log_comes_in_blocks_with_ff_after_the_samples() {
        let mut d = dev(&[("logged", "3")]);
        mode(&mut d);
        let mut v = Violations::default();
        cmd(&mut d, &[3, 0xff, 0xff], &mut v);
        assert_eq!(data(d.in_transfer(EP, 64, &mut v)), [2, 0x00, 0x80]);
        let mut all = Vec::new();
        for _ in 0..64 {
                let b = data(d.in_transfer(EP, LOG_READ, &mut v));
            assert_eq!(b.len(), 512);
            all.extend(b);
        }
        assert_eq!(all.len(), 32768);
        assert_eq!(&all[..6], &[120, 80, 126, 82, 132, 84]);
        assert!(all[6..].iter().all(|b| *b == 0xff));
        assert_eq!(d.in_transfer(EP, LOG_READ, &mut v), Reply::Nak, "the memory has ended");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn patterns_and_co_samples() {
        let o = Options { pattern: Pattern::Half, ..Options::default() };
        assert_eq!(sample(&o, 0), [120, 80]);
        assert_eq!(sample(&o, 1), [127, 83], "odd raw bytes");
        let o = Options { invalid: Some(2), ..Options::default() };
        assert_eq!(sample(&o, 1), [80, 0]);
        assert_eq!(sample(&o, 2), [132, 84]);
        let o = Options { invalid: Some(2), unit: 1, ..Options::default() };
        assert_eq!(sample(&o, 1), [40, 0], "0 degrees Fahrenheit is raw 40");
        let o = Options { id: 9, ..Options::default() };
        assert_eq!(sample(&o, 0), [0, 60], "big endian");
        assert_eq!(sample(&o, 20), [1, 0x90]);
        let mut d = dev(&[("model", "9")]);
        mode(&mut d);
        let mut v = Violations::default();
        cmd(&mut d, &[3, 0xff, 0xff], &mut v);
        assert_eq!(data(d.in_transfer(EP, 64, &mut v)), [2, 0x00, 0xfe], "65024");
    }

    #[test]
    fn a_save_takes_the_block_and_answers_ff() {
        let mut d = dev(&[("flag-bits", "e2")]);
        mode(&mut d);
        let mut v = Violations::default();
        let mut c = d.config.clone();
        c[0x21] = 0x03; // the top three bits gone, bit 0 set, as the C writes it
        c[0x18..0x1c].fill(0);
        cmd(&mut d, &[1, 128, 0], &mut v);
        assert_eq!(d.in_transfer(EP, 256, &mut v), Reply::Nak);
        assert_eq!(d.out(EP, &c, &mut v), Reply::ack());
        assert_eq!(data(d.in_transfer(EP, 256, &mut v)), [0xff]);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(d.config[0x21], 3);
    }

    #[test]
    fn flags_what_the_c_does_not_do() {
        let mut v = Violations::default();
        // Unknown requests.
        let mut d = dev(&[]);
        assert_eq!(d.control(ctl(2, 4), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "eltuxusb's 2/4 is not the C's");
        let s = Setup { request_type: 0xc0, request: 0, value: 0xffff, index: 0, length: 0 };
        d.control(s, &[], &mut v);
        assert_eq!(v.drain().len(), 1, "direction");
        // A command before the mode setup.
        let mut d = dev(&[]);
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        assert_eq!(v.drain().len(), 1, "no mode setup");
        // A mode setup out of order does not count.
        let mut d = dev(&[]);
        d.control(ctl(2, 1), &[], &mut v);
        d.control(ctl(2, 2), &[], &mut v);
        d.control(ctl(0, 0xffff), &[], &mut v);
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        assert_eq!(v.drain().len(), 1, "out of order");
        // Wrong lengths, arguments and commands.
        let mut d = dev(&[]);
        mode(&mut d);
        cmd(&mut d, &[0, 0xff], &mut v);
        assert_eq!(v.drain().len(), 1, "short command");
        cmd(&mut d, &[0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "argument");
        cmd(&mut d, &[7, 0xff, 0xff], &mut v);
        assert_eq!(v.drain().len(), 1, "unknown command");
        // A command with data queued.
        let mut d = dev(&[("stale", "4")]);
        mode(&mut d);
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        assert_eq!(v.drain().len(), 1, "not flushed");
        // A download with no new setup, and in the middle of another.
        let mut d = dev(&[]);
        mode(&mut d);
        cmd(&mut d, &[3, 0xff, 0xff], &mut v);
        assert!(v.is_empty(), "{v:?}");
        data(d.in_transfer(EP, 64, &mut v));
        cmd(&mut d, &[3, 0xff, 0xff], &mut v);
        assert_eq!(v.drain().len(), 2, "no setup, and the log is still queued");
        // Reads.
        let mut d = dev(&[("logged", "3")]);
        mode(&mut d);
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        data(d.in_transfer(EP, 64, &mut v));
        assert_eq!(data(d.in_transfer(EP, 64, &mut v)).len(), 64);
        assert_eq!(v.drain().len(), 1, "read shorter than the block");
        let mut d = dev(&[]);
        mode(&mut d);
        cmd(&mut d, &[3, 0xff, 0xff], &mut v);
        data(d.in_transfer(EP, 64, &mut v));
        data(d.in_transfer(EP, 512, &mut v));
        assert_eq!(v.drain().len(), 1, "log read of 512");
        assert_eq!(d.out(1, &[0], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "endpoint");
        assert_eq!(d.in_transfer(1, 64, &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "endpoint");
    }

    #[test]
    fn flags_a_save_changing_what_the_c_keeps() {
        let mut v = Violations::default();
        let mut d = dev(&[]);
        mode(&mut d);
        cmd(&mut d, &[1, 64, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "length");
        let mut d = dev(&[]);
        mode(&mut d);
        cmd(&mut d, &[1, 128, 0], &mut v);
        let mut c = d.config.clone();
        c[0x34] ^= 1; // the serial
        c[0x18] = 1; // neither kept nor zero
        c[0x21] = 0x80; // a top bit
        d.out(EP, &c, &mut v);
        assert_eq!(v.drain().len(), 3, "each field");
    }

    #[test]
    fn config_faults_count_the_reads() {
        let mut v = Violations::default();
        let read = |d: &mut ElUsb, v: &mut Violations| -> Vec<Vec<u8>> {
            let mut got = Vec::new();
            cmd(d, &[0, 0xff, 0xff], v);
            while let Reply::Data(x) = d.in_transfer(EP, 256, v) {
                got.push(x);
            }
            d.in_unlinked(EP, 256);
            got
        };
        for (fault, want) in [
            ("cfg-silent", vec![]),
            ("cfg-garbage", vec![vec![7, 128, 0]]),
            ("cfg-long", vec![vec![2, 1, 1]]),
            ("cfg-short-header", vec![vec![2, 128]]),
            ("cfg-nobody", vec![vec![2, 128, 0]]),
        ] {
            let mut d = dev(&[(fault, "2")]);
            mode(&mut d);
            assert_eq!(read(&mut d, &mut v).len(), 2, "{fault}: the first is good");
            assert_eq!(read(&mut d, &mut v), want, "{fault}");
            assert!(v.is_empty(), "{v:?}");
        }
        let mut d = dev(&[("cfg-short", "1")]);
        mode(&mut d);
        assert_eq!(read(&mut d, &mut v)[1].len(), 64);
        let mut d = dev(&[("tail", "10")]);
        mode(&mut d);
        assert_eq!(read(&mut d, &mut v)[2], [0x5a; 10]);
    }

    #[test]
    fn stalls_and_halts_count_the_answers() {
        let mut v = Violations::default();
        let mut d = dev(&[("stall-in", "1")]);
        mode(&mut d);
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        assert_eq!(d.in_transfer(EP, 64, &mut v), Reply::Stall);
        assert!(matches!(d.in_transfer(EP, 64, &mut v), Reply::Data(_)), "one shot");
        let mut d = dev(&[("halt-in", "1")]);
        mode(&mut d);
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        assert_eq!(d.in_transfer(EP, 64, &mut v), Reply::Stall);
        assert_eq!(d.in_transfer(EP, 64, &mut v), Reply::Stall, "until reset");
        d.reset();
        assert_eq!(d.in_transfer(EP, 64, &mut v), Reply::Nak);
        let mut d = dev(&[("stall-out", "1")]);
        assert_eq!(d.out(EP, &[0, 0xff, 0xff], &mut v), Reply::Stall);
        assert!(v.is_empty(), "{v:?}");
        // The C sends a save's block after a refused command.
        let mut d = dev(&[("stall-out", "1")]);
        mode(&mut d);
        assert_eq!(d.out(EP, &[1, 128, 0], &mut v), Reply::Stall);
        let c = d.config.clone();
        assert_eq!(d.out(EP, &c, &mut v), Reply::ack());
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(d.in_transfer(EP, 256, &mut v), Reply::Nak, "not saved, no answer");
        let mut d = dev(&[("stall-ctl", "2/2")]);
        assert_eq!(d.control(ctl(0, 0xffff), &[], &mut v), Reply::ack());
        assert_eq!(d.control(ctl(2, 2), &[], &mut v), Reply::Stall);
        assert_eq!(d.control(ctl(2, 1), &[], &mut v), Reply::ack());
        cmd(&mut d, &[0, 0xff, 0xff], &mut v);
        assert!(v.is_empty(), "a request refused still counts: {v:?}");
    }

    #[test]
    fn log_faults() {
        let mut v = Violations::default();
        let start = |pairs: &[(&str, &str)], v: &mut Violations| {
            let mut d = dev(pairs);
            mode(&mut d);
            cmd(&mut d, &[3, 0xff, 0xff], v);
            d
        };
        let mut d = start(&[("log-garbage", "1")], &mut v);
        assert_eq!(data(d.in_transfer(EP, 64, &mut v)), [7, 0, 0x80]);
        let mut d = start(&[("log-short-header", "1")], &mut v);
        assert_eq!(data(d.in_transfer(EP, 64, &mut v)), [2, 0]);
        let mut d = start(&[("log-size", "1024")], &mut v);
        assert_eq!(data(d.in_transfer(EP, 64, &mut v)), [2, 0, 4], "the header only");
        let mut d = start(&[("log-stop", "1")], &mut v);
        data(d.in_transfer(EP, 64, &mut v));
        data(d.in_transfer(EP, LOG_READ, &mut v));
        assert_eq!(d.in_transfer(EP, LOG_READ, &mut v), Reply::Nak, "it has stopped");
        let mut d = start(&[("hold", "1")], &mut v);
        data(d.in_transfer(EP, 64, &mut v));
        assert_eq!(d.in_transfer(EP, LOG_READ, &mut v), Reply::Nak);
        assert_eq!(d.in_unlinked(EP, LOG_READ).map(|x| x.len()), Some(64));
        assert_eq!(data(d.in_transfer(EP, LOG_READ, &mut v)).len(), 512, "the next block");
        let mut d = start(&[("hold-empty", "1")], &mut v);
        data(d.in_transfer(EP, 64, &mut v));
        assert_eq!(d.in_transfer(EP, LOG_READ, &mut v), Reply::Nak);
        assert_eq!(d.in_unlinked(EP, LOG_READ), None);
        let first = data(d.in_transfer(EP, LOG_READ, &mut v));
        assert_eq!(&first[..2], &[120, 80], "nothing lost");
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn save_faults() {
        let mut v = Violations::default();
        let save = |pairs: &[(&str, &str)], v: &mut Violations| {
            let mut d = dev(pairs);
            mode(&mut d);
            cmd(&mut d, &[1, 128, 0], v);
            let c = d.config.clone();
            d.out(EP, &c, v);
            d
        };
        let mut d = save(&[("save-silent", "1")], &mut v);
        assert_eq!(d.in_transfer(EP, 256, &mut v), Reply::Nak);
        let mut d = save(&[("save-answer", "00")], &mut v);
        assert_eq!(data(d.in_transfer(EP, 256, &mut v)), [0]);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn options_are_checked() {
        for bad in [
            ("nope", "1"),
            ("model", "300"),
            ("block", "0"),
            ("block", "5000"),
            ("memory", "70000"),
            ("config-size", "70000"),
            ("invalid", "0"),
            ("pattern", "x"),
            ("cfg-silent", "0"),
            ("tail", "5000"),
            ("stale", "5000"),
            ("stall-ctl", "2"),
        ] {
            assert!(build(&opts(&[bad])).is_err(), "{bad:?}");
        }
        assert!(build(&[]).is_ok());
    }
}
