// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Maynuo M97 or M98 series DC electronic load (M9710, M9812), a Modbus RTU
//! slave behind a USB to UART cable, on the CDC-ACM shell.
//!
//! The device, and how sure this is:
//!
//! - The frames, the functions (1, 3, 5, 16), the coil and register tables, the
//!   command values, big endian floats with the high word first, the slave
//!   address (1 to 200) and the line (8 data bits, 1 stop bit, parity and rate
//!   on the panel): the M98 user's manual, "Maynuo Electronics 2009", section
//!   4.8, read in full. The same tables are in Craig Peacock's `ATZ9712`
//!   (`ATZ9712.h`), written against an Atten ATZ9712, a rebadged M9712.
//! - That the manual's byte for a one coil read holds more than that coil: its
//!   example reads `ISTATE` and gets `0x48`, of which only the lowest bit is
//!   said to be `ISTATE`. Here a read of one coil answers the eight from there,
//!   which fits that example; that the other bits are those coils is a guess.
//! - 9600/8n1 and address 1 as the defaults: the manual's example and its
//!   communication chapter, which lists more than one set of rates.
//! - The ids in the `MODEL` register: the driver's table alone, 28 and 101, of
//!   which the driver's own comment says they are guessed.
//! - Where `SETMODE` holds the mode: the manual says only "u16". `ATZ9712`'s
//!   `main.cpp` takes the high byte and the driver the low one. `setmode=RAW`
//!   plays either; the low byte is the default as the driver's reading, not as
//!   a fact.
//! - The bridge: a real load has a DB9 and a USB or RS-232 cable. The ids here
//!   are the shell's stand-in.
//!
//! Emulation artifacts:
//!
//! - It answers at once. A real slave takes time, and nothing here checks pace.
//! - A frame ends at its length, as the function code says; RTU cuts frames by
//!   silence, for which a model has no clock.
//! - Values are taken as they come, whatever they are. The manual names no
//!   refusal of a value, only that a coil takes `0x0000` or `0xFF00`. A command
//!   out of its table is stored too, since no source says otherwise and the
//!   driver writes back whatever it read of `SETMODE`.
//! - `PC1` changes nothing here but its bit: the manual says that it locks the
//!   front panel, and no more.
//! - `U` is `source=`, and `I` is `IFIX` with the input on in CC, else 0. That
//!   is arithmetic, not a load's dynamics. No protection trips by itself:
//!   `coils=` sets its bit.
//! - `unasked=N` is no device: see below.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `id=N`, the `MODEL` register (28), and `version=N`, `EDITION` (123);
//! - `addr=N`, the slave address (1), and `uart=LINE`, the line (9600/8n1);
//! - `ifix=F`, `ufix=F`, `imax=F`, `umax=F`, the floats at power on;
//! - `setmode=RAW`, the `SETMODE` register at power on (1, CC);
//! - `source=F`, the voltage on the input (12);
//! - `coils=ADDR[,ADDR...]`, coils that are on at power on, as `0x...`;
//! - `u=F[,F...]` and `i=F[,F...]`, readings played in turn, one for each read
//!   that holds them, the last one staying;
//! - `answers=N`, frames answered per power on, then silence and the event
//!   point;
//! - `faults=N:KIND[,N:KIND...]`, what happens to the answer to the Nth frame
//!   addressed here: `silent`, `crc`, `addr` (another slave's), `short` (three
//!   bytes short), `nocrc` (all but its CRC), `first` (its first byte alone),
//!   `count` (a wrong byte count or echo), `func` (a wrong function code),
//!   `extra` (two bytes more) or `excXX` (exception XX, the manual naming 1 to
//!   4);
//! - `setters=0`, which flags any write but `PC1`, for the cases that show a
//!   tool changes nothing it was not asked to;
//! - `quiet=N`, the event point at the Nth offer of a read after `PC1` went on,
//!   for an event in a capture of a tool that sends nothing in it;
//! - `unasked=N`: right after the echo of `PC1` on, N answers to a read of `U`
//!   and `I` that nobody asked for. No slave does this, on Modbus or in any
//!   source. It is here for one reason: libsigrok's capture sends no request
//!   and reads a reply (its `protocol.c`), so this is the only way a run of
//!   that code reaches its frames, and the only oracle for what they look like.
//!   They are built from the lists of `u=` and `i=` without taking from them,
//!   so a tool that asks after all gets the same values in turn.
//!
//! The pedantic checks: a frame with a good CRC, only the functions, coils and
//! register blocks a C path uses, nothing left of a frame when a tool opens. A
//! read of four registers from `U` counts as such a path: it is what
//! `protocol.c`'s read of a reply is the reply to, and the two floats the manual
//! has there. A frame for another slave is not this model's business.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};
use std::fmt::Write;

/// The model name.
pub const NAME: &str = "maynuo-m97";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Maynuo M97 or M98 electronic load over Modbus RTU on CDC-ACM; id=, \
                         version=, addr=, uart=, ifix=, ufix=, imax=, umax=, setmode=, source=, \
                         coils=, u=, i=, answers=, faults=, setters=, quiet=, unasked=, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

const READ_COILS: u8 = 0x01;
const READ_HOLDING: u8 = 0x03;
const WRITE_COIL: u8 = 0x05;
const WRITE_MULTIPLE: u8 = 0x10;
const EXCEPTION: u8 = 0x80;
const ILLEGAL_FUNCTION: u8 = 0x01;
const ILLEGAL_ADDRESS: u8 = 0x02;
const ILLEGAL_VALUE: u8 = 0x03;
/// The longest frame Modbus has, and what a frame in the making may hold.
const LONGEST: usize = 256;

const COILS_AT: usize = 0x0500;
/// The coil table's end, `ERRCAL` being the last.
const COILS_END: usize = 0x0528;
const PC1: usize = 0x0500;
const ISTATE: usize = 0x0510;
const IOVER: usize = 0x0520;
const UOVER: usize = 0x0521;
const HEAT: usize = 0x0523;
const UNREG: usize = 0x0525;
/// The coils a C path reads, one at a time.
const COIL_READS: [usize; 5] = [ISTATE, IOVER, UOVER, HEAT, UNREG];

const SETUP_AT: usize = 0x0a00;
/// The end of the settings, `TAGSCAL` being the last.
const SETUP_END: usize = 0x0a43;
const CMD: usize = 0x0a00;
const IFIX: usize = 0x0a01;
const UFIX: usize = 0x0a03;
const IMAX: usize = 0x0a34;
const UMAX: usize = 0x0a36;
const STATUS_AT: usize = 0x0b00;
/// The end of what can only be read, `EDITION` being the last.
const STATUS_END: usize = 0x0b08;
const U: usize = 0x0b00;
const I: usize = 0x0b02;
const SETMODE: usize = 0x0b04;
const MODEL: usize = 0x0b06;
const EDITION: usize = 0x0b07;

/// The blocks a C path reads, `(address, count)`.
const READS: [(usize, usize); 9] = [
    (U, 2),
    (I, 2),
    (U, 4),
    (SETMODE, 1),
    (MODEL, 2),
    (UFIX, 2),
    (IFIX, 2),
    (UMAX, 2),
    (IMAX, 2),
];
/// The floats a C path writes.
const FLOAT_WRITES: [usize; 4] = [UFIX, IFIX, UMAX, IMAX];

const INPUT_ON: u16 = 42;
const INPUT_OFF: u16 = 43;
const MODE_CC: u16 = 1;

/// A value of a list, in turn; the last one stays.
#[derive(Clone, Debug, Default)]
struct Played {
    list: Vec<f32>,
    next: usize,
}

impl Played {
    fn take(&mut self) -> Option<f32> {
        let v = self.list.get(self.next.min(self.list.len().checked_sub(1)?)).copied();
        self.next += 1;
        v
    }
}

fn parse_float(key: &str, value: &str) -> Result<f32> {
    value.parse().map_err(|_| Error::Invalid(format!("{key}={value}: a number")))
}

fn parse_list(key: &str, value: &str) -> Result<Played> {
    let list = value.split(',').map(|v| parse_float(key, v)).collect::<Result<Vec<_>>>()?;
    Ok(Played { list, next: 0 })
}

fn parse_raw(key: &str, value: &str) -> Result<u16> {
    Ok(u16::try_from(parse_number(key, value, 0..=0xffff)?).expect("in range"))
}

fn parse_coils(value: &str) -> Result<Vec<usize>> {
    value
        .split(',')
        .map(|item| {
            item.strip_prefix("0x")
                .and_then(|hex| usize::from_str_radix(hex, 16).ok())
                .filter(|at| (COILS_AT..COILS_END).contains(at))
                .ok_or_else(|| Error::Invalid(format!("coils={value}: a coil as 0x..., in {item}")))
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Silent,
    Crc,
    Addr,
    Short,
    NoCrc,
    AddressOnly,
    Count,
    Func,
    Extra,
    Exception(u8),
}

fn parse_kind(text: &str) -> Option<Kind> {
    Some(match text {
        "silent" => Kind::Silent,
        "crc" => Kind::Crc,
        "addr" => Kind::Addr,
        "short" => Kind::Short,
        "nocrc" => Kind::NoCrc,
        "first" => Kind::AddressOnly,
        "count" => Kind::Count,
        "func" => Kind::Func,
        "extra" => Kind::Extra,
        _ => Kind::Exception(u8::from_str_radix(text.strip_prefix("exc")?, 16).ok()?),
    })
}

fn parse_faults(value: &str) -> Result<Vec<(u32, Kind)>> {
    value
        .split(',')
        .map(|item| {
            item.split_once(':')
                .and_then(|(n, k)| Some((n.parse().ok()?, parse_kind(k)?)))
                .ok_or_else(|| Error::Invalid(format!("faults={value}: N:KIND, in {item}")))
        })
        .collect()
}

#[derive(Clone, Debug)]
struct Config {
    id: u16,
    version: u16,
    addr: u8,
    ifix: f32,
    ufix: f32,
    imax: f32,
    umax: f32,
    setmode: u16,
    source: f32,
    coils: Vec<usize>,
    u: Played,
    i: Played,
    answers: Option<u32>,
    faults: Vec<(u32, Kind)>,
    setters: bool,
    quiet: Option<u32>,
    unasked: u32,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            id: 28,
            version: 123,
            addr: 1,
            ifix: 1.0,
            ufix: 10.0,
            imax: 30.0,
            umax: 150.0,
            setmode: MODE_CC,
            source: 12.0,
            coils: Vec::new(),
            u: Played::default(),
            i: Played::default(),
            answers: None,
            faults: Vec::new(),
            setters: true,
            quiet: None,
            unasked: 0,
        }
    }
}

/// A load from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Load>> {
    let mut faults = Faults::default();
    let mut uart = None;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "id" => config.id = parse_raw(k, v)?,
            "version" => config.version = parse_raw(k, v)?,
            "addr" => config.addr = u8::try_from(parse_number(k, v, 1..=200)?).expect("in range"),
            "uart" => uart = Some(Line::parse(v)?),
            "ifix" => config.ifix = parse_float(k, v)?,
            "ufix" => config.ufix = parse_float(k, v)?,
            "imax" => config.imax = parse_float(k, v)?,
            "umax" => config.umax = parse_float(k, v)?,
            "setmode" => config.setmode = parse_raw(k, v)?,
            "source" => config.source = parse_float(k, v)?,
            "coils" => config.coils = parse_coils(v)?,
            "u" => config.u = parse_list(k, v)?,
            "i" => config.i = parse_list(k, v)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "faults" => config.faults = parse_faults(v)?,
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            "quiet" => config.quiet = Some(parse_number(k, v, 1..=100_000)?),
            "unasked" => config.unasked = parse_number(k, v, 0..=64)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let line = match uart {
        Some(line) => line,
        None => Line::parse("9600/8n1")?,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Maynuo electronic load");
    Ok(Cdc::new(NAME, &id, faults, Load::new(config)).behind_uart(line))
}

/// CRC-16/MODBUS.
fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0xffff, |crc, b| {
        (0..8).fold(crc ^ u16::from(*b), |c, _| if c & 1 == 1 { c >> 1 ^ 0xa001 } else { c >> 1 })
    })
}

/// Whether `count` registers from `address` are all in one table.
fn in_a_table(address: usize, count: usize, read: bool) -> bool {
    let end = address + count;
    (SETUP_AT..=SETUP_END).contains(&address) && end <= SETUP_END
        || read && (STATUS_AT..=STATUS_END).contains(&address) && end <= STATUS_END
}

fn with_crc(address: u8, answer: &[u8]) -> Vec<u8> {
    let mut out = vec![address];
    out.extend_from_slice(answer);
    out.extend_from_slice(&crc16(&out).to_le_bytes());
    out
}

struct Load {
    config: Config,
    coils: [bool; COILS_END - COILS_AT],
    setup: [u16; SETUP_END - SETUP_AT],
    status: [u16; STATUS_END - STATUS_AT],
    u: Played,
    i: Played,
    /// What has come of a frame so far.
    pending: Vec<u8>,
    /// Frames addressed here since power on.
    frames: u32,
    /// Offers of a read since `PC1` went on, once it has.
    offers: Option<u32>,
}

impl Load {
    fn new(config: Config) -> Load {
        let mut coils = [false; COILS_END - COILS_AT];
        for at in &config.coils {
            coils[at - COILS_AT] = true;
        }
        let mut status = [0; STATUS_END - STATUS_AT];
        status[SETMODE - STATUS_AT] = config.setmode;
        status[MODEL - STATUS_AT] = config.id;
        status[EDITION - STATUS_AT] = config.version;
        let mut load = Load {
            coils,
            setup: [0; SETUP_END - SETUP_AT],
            status,
            u: config.u.clone(),
            i: config.i.clone(),
            config,
            pending: Vec::new(),
            frames: 0,
            offers: None,
        };
        let floats = [
            (IFIX, load.config.ifix),
            (UFIX, load.config.ufix),
            (IMAX, load.config.imax),
            (UMAX, load.config.umax),
        ];
        for (at, value) in floats {
            load.set_float(at, value);
        }
        load
    }

    fn setting(&self, at: usize) -> u16 {
        self.setup[at - SETUP_AT]
    }

    fn set_float(&mut self, at: usize, value: f32) {
        let bits = value.to_bits();
        self.setup[at - SETUP_AT] = (bits >> 16) as u16;
        self.setup[at - SETUP_AT + 1] = (bits & 0xffff) as u16;
    }

    fn float(&self, at: usize) -> f32 {
        f32::from_bits(u32::from(self.setting(at)) << 16 | u32::from(self.setting(at + 1)))
    }

    fn set_status_float(&mut self, at: usize, value: f32) {
        let bits = value.to_bits();
        self.status[at - STATUS_AT] = (bits >> 16) as u16;
        self.status[at - STATUS_AT + 1] = (bits & 0xffff) as u16;
    }

    /// What `I` reads with nothing played.
    fn current(&self) -> f32 {
        let cc = self.status[SETMODE - STATUS_AT] & 0xff == MODE_CC;
        if self.coils[ISTATE - COILS_AT] && cc { self.float(IFIX) } else { 0.0 }
    }

    /// `U` and `I` for a read of `address` and `count`, the lists going on.
    fn measure(&mut self, address: usize, count: usize) {
        let holds = |at: usize| address <= at && at + 2 <= address + count;
        if holds(U) {
            let value = self.u.take().unwrap_or(self.config.source);
            self.set_status_float(U, value);
        }
        if holds(I) {
            let value = self.i.take().unwrap_or_else(|| self.current());
            self.set_status_float(I, value);
        }
    }

    fn read_registers(&mut self, frame: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let address = usize::from(u16::from_be_bytes([frame[2], frame[3]]));
        let count = usize::from(u16::from_be_bytes([frame[4], frame[5]]));
        if !READS.contains(&(address, count)) {
            port.violation(format!("reading {count} registers at {address:#x}: no C path"));
        }
        if count == 0 || count > 125 || !in_a_table(address, count, true) {
            return vec![READ_HOLDING | EXCEPTION, ILLEGAL_ADDRESS];
        }
        self.measure(address, count);
        let table = if address >= STATUS_AT {
            &self.status[address - STATUS_AT..address - STATUS_AT + count]
        } else {
            &self.setup[address - SETUP_AT..address - SETUP_AT + count]
        };
        let mut out = vec![READ_HOLDING, (count * 2) as u8];
        for r in table {
            out.extend_from_slice(&r.to_be_bytes());
        }
        out
    }

    fn read_coils(&mut self, frame: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let address = usize::from(u16::from_be_bytes([frame[2], frame[3]]));
        let count = usize::from(u16::from_be_bytes([frame[4], frame[5]]));
        if count != 1 || !COIL_READS.contains(&address) {
            port.violation(format!("reading {count} coils at {address:#x}: no C path"));
        }
        if count != 1 || !(COILS_AT..COILS_END).contains(&address) {
            return vec![READ_COILS | EXCEPTION, ILLEGAL_ADDRESS];
        }
        // The byte of the manual's example: the coil, and what follows it.
        let byte = (0..8).fold(0u8, |byte, bit| {
            let on = self.coils.get(address - COILS_AT + bit).copied().unwrap_or(false);
            byte | u8::from(on) << bit
        });
        vec![READ_COILS, 1, byte]
    }

    fn write_coil(&mut self, frame: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let address = usize::from(u16::from_be_bytes([frame[2], frame[3]]));
        let value = u16::from_be_bytes([frame[4], frame[5]]);
        if address != PC1 {
            port.violation(format!("writing coil {address:#x}: no C path"));
        }
        if value != 0 && value != 0xff00 {
            port.violation(format!("coil {address:#x} set to {value:#x}: 0 or 0xff00, says the manual"));
            return vec![WRITE_COIL | EXCEPTION, ILLEGAL_VALUE];
        }
        if !(COILS_AT..COILS_END).contains(&address) {
            return vec![WRITE_COIL | EXCEPTION, ILLEGAL_ADDRESS];
        }
        self.coils[address - COILS_AT] = value != 0;
        if address == PC1 && value != 0 && self.offers.is_none() {
            self.offers = Some(0);
        }
        frame[1..6].to_vec()
    }

    fn write_registers(&mut self, frame: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let address = usize::from(u16::from_be_bytes([frame[2], frame[3]]));
        let count = usize::from(u16::from_be_bytes([frame[4], frame[5]]));
        if usize::from(frame[6]) != count * 2 {
            port.violation(format!("{count} registers in {} bytes", frame[6]));
            return vec![WRITE_MULTIPLE | EXCEPTION, ILLEGAL_VALUE];
        }
        if count == 0 || !in_a_table(address, count, false) {
            port.violation(format!("writing {count} registers at {address:#x}: off the map"));
            return vec![WRITE_MULTIPLE | EXCEPTION, ILLEGAL_ADDRESS];
        }
        if !self.config.setters {
            port.violation(format!("writing at {address:#x}: a setting no one asked for"));
        }
        let word = |i: usize| u16::from_be_bytes([frame[7 + i * 2], frame[8 + i * 2]]);
        match (address, count) {
            (CMD, 1) => self.command(word(0)),
            (at, 2) if FLOAT_WRITES.contains(&at) => {
                self.setup[at - SETUP_AT] = word(0);
                self.setup[at - SETUP_AT + 1] = word(1);
            }
            _ => port.violation(format!("writing {count} registers at {address:#x}: no C path")),
        }
        frame[1..6].to_vec()
    }

    /// `CMD`, whose lower 8 bits count, says the manual.
    fn command(&mut self, value: u16) {
        let value = value & 0xff;
        self.setup[CMD - SETUP_AT] = value;
        match value {
            INPUT_ON => self.coils[ISTATE - COILS_AT] = true,
            INPUT_OFF => self.coils[ISTATE - COILS_AT] = false,
            mode => {
                let held = self.status[SETMODE - STATUS_AT] & 0xff00;
                self.status[SETMODE - STATUS_AT] = held | mode;
            }
        }
    }

    /// The answer's bytes after the address, or an exception.
    fn answer(&mut self, frame: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        match frame[1] {
            READ_COILS => self.read_coils(frame, port),
            READ_HOLDING => self.read_registers(frame, port),
            WRITE_COIL => self.write_coil(frame, port),
            WRITE_MULTIPLE => self.write_registers(frame, port),
            other => {
                port.violation(format!("function {other:#04x}: no C path"));
                vec![other | EXCEPTION, ILLEGAL_FUNCTION]
            }
        }
    }

    /// What `unasked=` sends: not taking from the lists a tool that asks gets.
    fn unasked(&self) -> Vec<u8> {
        let mut u = self.config.u.clone();
        let mut i = self.config.i.clone();
        let mut out = Vec::new();
        for _ in 0..self.config.unasked {
            let volts = u.take().unwrap_or(self.config.source);
            let amps = i.take().unwrap_or_else(|| self.current());
            let mut answer = vec![READ_HOLDING, 8];
            answer.extend_from_slice(&volts.to_be_bytes());
            answer.extend_from_slice(&amps.to_be_bytes());
            out.extend_from_slice(&with_crc(self.config.addr, &answer));
        }
        out
    }

    /// One frame for this slave: the answer, as the faults leave it.
    fn serve(&mut self, frame: &[u8], port: &mut Port<'_>) {
        self.frames += 1;
        let number = self.frames;
        let mut answer = self.answer(frame, port);
        if self.config.answers.is_some_and(|n| number > n) {
            port.mark();
            return;
        }
        let fault = self.config.faults.iter().find(|(n, _)| *n == number).map(|(_, k)| *k);
        let mut address = self.config.addr;
        let mut cut = 0;
        let mut extra = false;
        let mut bad_crc = false;
        match fault {
            None => {}
            Some(Kind::Silent) => return,
            Some(Kind::Crc) => bad_crc = true,
            Some(Kind::Addr) => address = address.wrapping_add(1),
            Some(Kind::Short) => cut = 3,
            Some(Kind::NoCrc) => cut = 2,
            Some(Kind::AddressOnly) => {
                port.send(&[address]);
                return;
            }
            Some(Kind::Count) => answer[1] = answer[1].wrapping_add(2),
            Some(Kind::Func) => answer[0] ^= 0x01,
            Some(Kind::Extra) => extra = true,
            Some(Kind::Exception(code)) => answer = vec![frame[1] | EXCEPTION, code],
        }
        let mut out = vec![address];
        out.append(&mut answer);
        let crc = crc16(&out) ^ u16::from(bad_crc);
        out.extend_from_slice(&crc.to_le_bytes());
        if extra {
            out.extend_from_slice(&[0x55, 0xaa]);
        }
        out.truncate(out.len() - cut);
        if fault.is_none() && frame[1] == WRITE_COIL && frame[2..6] == [0x05, 0x00, 0xff, 0x00] {
            out.extend_from_slice(&self.unasked());
        }
        port.send(&out);
    }

    /// Takes the complete frames off the front of what has come.
    fn frames(&mut self, port: &mut Port<'_>) {
        while self.pending.len() >= 2 {
            let length = match self.pending[1] {
                WRITE_MULTIPLE => match self.pending.get(6) {
                    Some(bytes) => 9 + usize::from(*bytes),
                    None => return,
                },
                _ => 8,
            };
            if self.pending.len() < length {
                return;
            }
            let frame: Vec<u8> = self.pending.drain(..length).collect();
            let (body, crc) = frame.split_at(length - 2);
            if crc16(body).to_le_bytes() != crc {
                port.violation(format!("a frame with a bad CRC: {}", hex(&frame)));
                continue;
            }
            // A broadcast has no answer, and no C path sends one by itself:
            // `modbusaddr=0` meets silence.
            if frame[0] == self.config.addr {
                self.serve(&frame, port);
            }
        }
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut out, b| {
        let _ = write!(out, "{b:02x}");
        out
    })
}

impl Serial for Load {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        self.pending.extend_from_slice(data);
        self.frames(port);
        // What no frame can be: a host's bytes are its own, the bound is ours.
        if self.pending.len() > LONGEST {
            port.violation(format!("{} bytes and no frame", self.pending.len()));
            self.pending.clear();
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("a frame left unfinished: {}", hex(&self.pending)));
            self.pending.clear();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some(offers) = &mut self.offers {
            *offers += 1;
            if self.config.quiet == Some(*offers) {
                port.mark();
            }
        }
    }

    fn reset(&mut self) {
        *self = Load::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    const NOTHING: Vec<u8> = Vec::new();

    fn load_on(options: &[(&str, &str)], at: &str) -> Host<Load> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse(at).unwrap(), true, true)
    }

    fn load(options: &[(&str, &str)]) -> Host<Load> {
        load_on(options, "9600/8n1")
    }

    fn violations(h: &mut Host<Load>) -> usize {
        h.v.drain().len()
    }

    fn framed(body: &[u8]) -> Vec<u8> {
        let mut f = body.to_vec();
        f.extend_from_slice(&crc16(body).to_le_bytes());
        f
    }

    fn read_request(address: usize, count: u16) -> Vec<u8> {
        let [ah, al] = (address as u16).to_be_bytes();
        let [ch, cl] = count.to_be_bytes();
        framed(&[1, READ_HOLDING, ah, al, ch, cl])
    }

    fn coil_read(address: usize) -> Vec<u8> {
        let [ah, al] = (address as u16).to_be_bytes();
        framed(&[1, READ_COILS, ah, al, 0, 1])
    }

    fn coil_write(address: usize, on: bool) -> Vec<u8> {
        let [ah, al] = (address as u16).to_be_bytes();
        framed(&[1, WRITE_COIL, ah, al, if on { 0xff } else { 0 }, 0])
    }

    fn float_write(address: usize, value: f32) -> Vec<u8> {
        let [ah, al] = (address as u16).to_be_bytes();
        let b = value.to_be_bytes();
        framed(&[1, WRITE_MULTIPLE, ah, al, 0, 2, 4, b[0], b[1], b[2], b[3]])
    }

    fn command(value: u16) -> Vec<u8> {
        let [vh, vl] = value.to_be_bytes();
        framed(&[1, WRITE_MULTIPLE, 0x0a, 0, 0, 1, 2, vh, vl])
    }

    /// The registers of an answer, its CRC checked.
    fn registers(reply: &[u8]) -> Vec<u16> {
        let (body, crc) = reply.split_at(reply.len() - 2);
        assert_eq!(crc16(body).to_le_bytes(), crc, "the CRC of {reply:02x?}");
        assert_eq!(usize::from(body[2]), body.len() - 3);
        body[3..].chunks(2).map(|c| u16::from_be_bytes([c[0], c[1]])).collect()
    }

    fn float_of(words: &[u16]) -> f32 {
        f32::from_bits(u32::from(words[0]) << 16 | u32::from(words[1]))
    }

    fn read_float(h: &mut Host<Load>, at: usize) -> f32 {
        h.write(&read_request(at, 2));
        float_of(&registers(&h.read()))
    }

    fn read_coil(h: &mut Host<Load>, at: usize) -> u8 {
        h.write(&coil_read(at));
        h.read()[3]
    }

    #[test]
    fn the_frames_of_the_manuals_examples() {
        // Section 4.8.6: reading ISTATE, with 0x48 as the byte that comes back.
        let mut h = load(&[("coils", "0x513,0x516")]);
        h.write(&[0x01, 0x01, 0x05, 0x10, 0x00, 0x01, 0xfc, 0xc3]);
        assert_eq!(h.read(), [0x01, 0x01, 0x01, 0x48, 0x51, 0xbe]);
        // PC1 on, echoed.
        h.write(&[0x01, 0x05, 0x05, 0x00, 0xff, 0x00, 0x8c, 0xf6]);
        assert_eq!(h.read(), [0x01, 0x05, 0x05, 0x00, 0xff, 0x00, 0x8c, 0xf6]);
        assert_eq!(violations(&mut h), 0);
        // The present voltage, 0x4120002A being "10 V".
        let ten = f32::from_bits(0x4120_002a).to_string();
        let mut h = load(&[("u", ten.as_str())]);
        h.write(&[0x01, 0x03, 0x0b, 0x00, 0x00, 0x02, 0xc6, 0x2f]);
        assert_eq!(h.read(), [0x01, 0x03, 0x04, 0x41, 0x20, 0x00, 0x2a, 0x6e, 0x1a]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn it_says_its_model_and_version() {
        let mut h = load(&[("id", "101"), ("version", "17")]);
        h.write(&read_request(MODEL, 2));
        assert_eq!(registers(&h.read()), [101, 17]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_float_is_written_and_read_back_high_word_first() {
        let mut h = load(&[]);
        h.write(&float_write(UFIX, 3.3));
        assert_eq!(h.read()[..6], [1, WRITE_MULTIPLE, 0x0a, 0x03, 0, 2]);
        assert_eq!(read_float(&mut h, UFIX), 3.3);
        h.write(&read_request(UFIX, 2));
        assert_eq!(registers(&h.read())[0], (3.3f32.to_bits() >> 16) as u16);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn each_float_has_its_place() {
        let mut h = load(&[("ifix", "1.5"), ("ufix", "2.5"), ("imax", "3.5"), ("umax", "4.5")]);
        let got = [IFIX, UFIX, IMAX, UMAX].map(|at| read_float(&mut h, at));
        assert_eq!(got, [1.5, 2.5, 3.5, 4.5]);
    }

    #[test]
    fn the_command_register_runs_the_input_and_the_mode() {
        let mut h = load(&[]);
        assert_eq!(read_coil(&mut h, ISTATE) & 1, 0);
        h.write(&command(42));
        assert_eq!(h.read().len(), 8);
        assert_eq!(read_coil(&mut h, ISTATE) & 1, 1);
        h.write(&read_request(SETMODE, 1));
        assert_eq!(registers(&h.read()), [1], "the input leaves the mode");
        h.write(&command(43));
        h.read();
        assert_eq!(read_coil(&mut h, ISTATE) & 1, 0);
        h.write(&command(0x7702));
        h.read();
        h.write(&read_request(SETMODE, 1));
        assert_eq!(registers(&h.read()), [2], "lower 8 bits effective");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_current_follows_the_input_in_cc() {
        let mut h = load(&[("ifix", "2.5")]);
        assert_eq!(read_float(&mut h, I), 0.0);
        h.write(&command(42));
        h.read();
        assert_eq!(read_float(&mut h, I), 2.5);
        h.write(&command(2));
        h.read();
        assert_eq!(read_float(&mut h, I), 0.0, "not in CV");
    }

    #[test]
    fn the_poll_block_holds_both_readings() {
        let mut h = load(&[("u", "1,2"), ("i", "3")]);
        let mut seen = Vec::new();
        for _ in 0..3 {
            h.write(&read_request(U, 4));
            let r = registers(&h.read());
            seen.push((float_of(&r[..2]), float_of(&r[2..])));
        }
        assert_eq!(seen, [(1.0, 3.0), (2.0, 3.0), (2.0, 3.0)]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_read_of_one_goes_on_with_its_list_alone() {
        let mut h = load(&[("u", "1,2"), ("i", "3,4")]);
        assert_eq!(read_float(&mut h, U), 1.0);
        assert_eq!(read_float(&mut h, U), 2.0);
        assert_eq!(read_float(&mut h, I), 3.0);
        h.write(&read_request(SETMODE, 1));
        h.read();
        assert_eq!(read_float(&mut h, I), 4.0);
    }

    #[test]
    fn unasked_answers_follow_the_echo_and_leave_the_lists() {
        let mut h = load(&[("unasked", "2"), ("u", "1,2")]);
        h.write(&coil_write(PC1, true));
        let got = h.read();
        assert_eq!(got.len(), 8 + 2 * 13);
        assert_eq!(got[..8], coil_write(PC1, true)[..]);
        assert_eq!(float_of(&registers(&got[8..21])), 1.0);
        assert_eq!(float_of(&registers(&got[21..34])), 2.0);
        assert_eq!(read_float(&mut h, U), 1.0, "the lists have not moved");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn nothing_unasked_comes_of_other_writes_or_a_fault() {
        let mut h = load(&[("unasked", "2")]);
        h.write(&coil_write(PC1, false));
        assert_eq!(h.read().len(), 8);
        let mut h = load(&[("unasked", "2"), ("faults", "1:exc04")]);
        h.write(&coil_write(PC1, true));
        assert_eq!(h.read().len(), 5);
    }

    #[test]
    fn quiet_marks_after_its_offers_once_pc1_is_on() {
        let mut h = load(&[("quiet", "2")]);
        for _ in 0..5 {
            h.offer();
        }
        assert!(!h.cdc.mark(), "offers before PC1 do not count");
        h.write(&coil_write(PC1, true));
        h.read(); // The echo, then one offer with nothing queued.
        assert!(!h.cdc.mark(), "one offer");
        h.offer();
        assert!(h.cdc.mark(), "the second");
        h.offer();
        assert!(!h.cdc.mark(), "once");
        h.write(&coil_write(PC1, true));
        h.read();
        assert!(!h.cdc.mark(), "a second PC1 does not start it again");
    }

    #[test]
    fn a_line_the_uart_does_not_take_passes_nothing() {
        let mut h = load_on(&[], "19200/8n1");
        h.write(&read_request(MODEL, 2));
        assert_eq!(h.read(), NOTHING);
    }

    #[test]
    fn another_slaves_frame_gets_no_answer_and_no_flag() {
        let mut h = load(&[("addr", "5")]);
        h.write(&read_request(MODEL, 2));
        assert_eq!(h.read(), NOTHING);
        assert_eq!(violations(&mut h), 0);
        h.write(&framed(&[5, READ_HOLDING, 0x0b, 0x06, 0, 2]));
        assert_eq!(h.read()[0], 5);
    }

    #[test]
    fn a_frame_may_come_in_any_pieces() {
        let mut h = load(&[]);
        for b in &float_write(IFIX, 1.0) {
            h.write(std::slice::from_ref(b));
        }
        assert_eq!(h.read().len(), 8);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_bad_crc_is_flagged() {
        let mut h = load(&[]);
        let mut f = read_request(MODEL, 2);
        f[6] ^= 1;
        h.write(&f);
        assert_eq!(h.read(), NOTHING);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_block_no_c_path_reads_is_flagged_and_answered() {
        let mut h = load(&[]);
        h.write(&read_request(MODEL, 1));
        assert_eq!(registers(&h.read()), [28]);
        assert_eq!(violations(&mut h), 1);
        h.write(&read_request(0x0a05, 2));
        assert_eq!(registers(&h.read()), [0, 0], "PFIX is on the map");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_read_off_the_map_is_an_exception() {
        let mut h = load(&[]);
        for (at, count) in [(0x0c00, 2), (0x0b07, 2), (0x0a42, 2), (0, 2), (U, 0), (U, 126)] {
            h.write(&read_request(at, count));
            assert_eq!(h.read()[..3], [1, READ_HOLDING | EXCEPTION, ILLEGAL_ADDRESS], "{at:#x}");
            assert_eq!(violations(&mut h), 1, "{at:#x}");
        }
    }

    #[test]
    fn a_coil_read_no_c_path_makes_is_flagged() {
        let mut h = load(&[]);
        h.write(&coil_read(PC1));
        assert_eq!(h.read()[..3], [1, READ_COILS, 1], "it is on the map");
        assert_eq!(violations(&mut h), 1);
        h.write(&framed(&[1, READ_COILS, 0x05, 0x10, 0, 8]));
        assert_eq!(h.read()[1], READ_COILS | EXCEPTION);
        assert_eq!(violations(&mut h), 1);
        h.write(&coil_read(0x0600));
        assert_eq!(h.read()[..3], [1, READ_COILS | EXCEPTION, ILLEGAL_ADDRESS]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_coil_write_no_c_path_makes_is_flagged() {
        let mut h = load(&[]);
        h.write(&coil_write(ISTATE, true));
        assert_eq!(h.read().len(), 8);
        assert_eq!(violations(&mut h), 1);
        h.write(&coil_write(0x0600, true));
        assert_eq!(h.read()[..3], [1, WRITE_COIL | EXCEPTION, ILLEGAL_ADDRESS]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_coil_takes_two_values() {
        let mut h = load(&[]);
        h.write(&framed(&[1, WRITE_COIL, 0x05, 0, 0x12, 0x34]));
        assert_eq!(h.read()[..3], [1, WRITE_COIL | EXCEPTION, ILLEGAL_VALUE]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_function_no_c_path_sends_is_an_exception() {
        let mut h = load(&[]);
        h.write(&framed(&[1, 0x06, 0x0a, 0, 0, 1]));
        assert_eq!(h.read()[..3], [1, 0x86, ILLEGAL_FUNCTION]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_register_write_no_c_path_makes_is_flagged() {
        let mut h = load(&[]);
        h.write(&float_write(0x0a05, 1.0));
        assert_eq!(h.read().len(), 8, "PFIX can be written");
        assert_eq!(violations(&mut h), 1);
        h.write(&framed(&[1, WRITE_MULTIPLE, 0x0a, 0x03, 0, 1, 2, 0, 1]));
        h.read();
        assert_eq!(violations(&mut h), 1, "half a float");
    }

    #[test]
    fn a_write_off_the_map_is_an_exception() {
        let mut h = load(&[]);
        h.write(&float_write(U, 1.0));
        assert_eq!(h.read()[..3], [1, WRITE_MULTIPLE | EXCEPTION, ILLEGAL_ADDRESS], "read only");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_count_that_the_bytes_do_not_hold_is_flagged() {
        let mut h = load(&[]);
        h.write(&framed(&[1, WRITE_MULTIPLE, 0x0a, 0, 0, 100, 2, 0, 1]));
        assert_eq!(h.read()[..3], [1, WRITE_MULTIPLE | EXCEPTION, ILLEGAL_VALUE]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn setters_0_flags_a_setting_but_not_pc1() {
        let mut h = load(&[("setters", "0")]);
        h.write(&coil_write(PC1, true));
        h.read();
        assert_eq!(violations(&mut h), 0);
        h.write(&command(42));
        h.read();
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_frame_left_unfinished_is_flagged_at_an_open() {
        let mut h = load(&[]);
        h.write(&[1, READ_HOLDING]);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_stream_of_no_frame_is_bounded() {
        let mut h = load(&[]);
        h.write(&[1, WRITE_MULTIPLE, 0, 0, 0, 1, 255]);
        for _ in 0..10 {
            h.write(&[0; 64]);
        }
        h.write(&[0xff; 64]);
        assert!(violations(&mut h) >= 1);
    }

    #[test]
    fn answers_runs_out() {
        let mut h = load(&[("answers", "1")]);
        h.write(&read_request(MODEL, 2));
        assert_ne!(h.read(), NOTHING);
        h.write(&read_request(MODEL, 2));
        assert_eq!(h.read(), NOTHING);
    }

    #[test]
    fn each_fault_does_its_harm() {
        let request = read_request(MODEL, 2);
        let good = {
            let mut h = load(&[]);
            h.write(&request);
            h.read()
        };
        let reply = |kind: &str| {
            let mut h = load(&[("faults", kind)]);
            h.write(&request);
            h.read()
        };
        assert_eq!(reply("1:silent"), NOTHING);
        assert_eq!(reply("2:silent"), good, "the fault is the Nth's");
        let crc = reply("1:crc");
        assert_eq!(crc[..crc.len() - 2], good[..good.len() - 2]);
        assert_ne!(crc, good);
        assert_eq!(reply("1:addr")[0], 2);
        assert_eq!(reply("1:short"), good[..good.len() - 3]);
        assert_eq!(reply("1:nocrc"), good[..good.len() - 2]);
        assert_eq!(reply("1:first"), good[..1]);
        assert_eq!(reply("1:count")[2], good[2] + 2);
        assert_eq!(reply("1:func")[1], READ_HOLDING ^ 1);
        assert_eq!(reply("1:extra")[..good.len()], good[..]);
        assert_eq!(reply("1:extra").len(), good.len() + 2);
        assert_eq!(reply("1:exc42")[1..3], [READ_HOLDING | EXCEPTION, 0x42]);
    }

    #[test]
    fn options_are_checked() {
        for (k, v) in [
            ("id", "70000"),
            ("addr", "0"),
            ("addr", "201"),
            ("faults", "1"),
            ("faults", "x:crc"),
            ("faults", "1:what"),
            ("u", "1,x"),
            ("ifix", "x"),
            ("coils", "0x510,1296"),
            ("coils", "0x600"),
            ("uart", "fast"),
            ("unasked", "65"),
            ("nosuch", "1"),
        ] {
            assert!(function(&[(k.to_owned(), v.to_owned())]).is_err(), "{k}={v}");
        }
    }
}
