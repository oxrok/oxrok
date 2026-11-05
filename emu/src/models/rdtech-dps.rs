// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An `RDTech` (Riden) DPS, DPH or RD power supply, a Modbus RTU slave behind
//! a USB to UART bridge, on the CDC-ACM shell.
//!
//! The device, and how sure this is:
//!
//! - Modbus RTU, 8n1. 9600 by default on a DPS and 115200 on an RD, each
//!   changeable on the panel, as is the slave address, 1 by default (sigrok
//!   wiki, "`RDTech` DPS series" and "`RDTech` RD series"). The line is what the
//!   UART takes: `uart=` changes it, and nothing passes on any other.
//! - The register maps: `protocol.c`'s tables, against `rfinnie/rdserialtool`
//!   (`rdserial/dps/__init__.py`), which reads the DPS's `0x52` OVP in the
//!   voltage's scale and `0x53` OCP in the current's. The search also
//!   turned up the vendor's DPS5020 protocol V1.2 (function codes 3, 6 and
//!   16) and improwis.com's `RD60xx` notes, only through search snippets and
//!   a summary: the PDF itself could not be fetched or read here.
//! - The model numbers and each one's digits: `api.c`'s tables, the only
//!   source for most of them. Where sources part, this serves the driver's
//!   reading: a DPS5015's and DPS5020's current in hundredths (`esphome-dps`
//!   too; `rdserialtool` has thousandths on every model, tested on a DPS5005
//!   alone) and power in hundredths on the RD P models (the `riden` library has
//!   thousandths, with register 12 as a high word). So these goldens cannot
//!   settle which is right.
//! - The bridge: an RD has a CH340E (sigrok wiki). A DPS has a USB or
//!   Bluetooth module on a TTL header. The ids here are the shell's stand-in,
//!   since a real CH340's would bring the `ch341` driver and no ttyACM.
//!
//! Emulation artifacts:
//!
//! - It answers at once; a real slave takes time. Nothing here checks pace.
//! - A frame ends at its length, as the function code says. RTU cuts frames
//!   by silence, for which a model has no clock.
//! - Writes are taken as they come, whatever their value: no source says how
//!   the firmware answers a value above the model's limits or a write to
//!   `0x14`, which one note lists as read-only and `protocol.c` writes. This
//!   is a C habit a device might refuse with an exception; `faults=` plays
//!   that refusal.
//! - A change of range leaves the raw registers as they are.
//! - The operating point from `load=` is arithmetic, not a supply's dynamics.
//!   No protection trips by itself: `protect=` plays what the register says.
//!
//! Not modelled: the presets beyond the active slot (`0x23` and `0x60` up),
//! the battery, clock, calibration and option registers. They read as 0, so
//! that a read anywhere in the map answers as a real one would, and the
//! pedantic check below flags whatever no C path asks for.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `id=N`, the model number (5005); its family gives the map, a number
//!   from 60000 up being an RD;
//! - `version=N`, the firmware, as the register holds it (116);
//! - `serial=N`, an RD's serial number (12345);
//! - `addr=N`, the slave address (1);
//! - `uart=LINE`, the line, the family's by default;
//! - `uset=RAW`, `iset=RAW`, `ovp=RAW`, `ocp=RAW`, `output=0|1`, the state at
//!   power on, as the registers hold it;
//! - `load=OHMS` on the output, or none; it decides CV or CC;
//! - `uout=`, `iout=`, `power=`, `protect=`, `cc=`, `enable=` and `range=`
//!   take `RAW[,RAW...]`, register values played in turn, one for each read
//!   of the block that holds them, the last one staying; `range=` also sets
//!   the first value of the register;
//! - `answers=N`, frames answered per power on, then silence and the event
//!   point;
//! - `faults=N:KIND[,N:KIND...]`, what happens to the answer to the Nth frame
//!   addressed here: `silent`, `crc`, `addr` (another slave's), `short`
//!   (three bytes short), `nocrc` (all but its CRC), `first` (its first byte
//!   alone), `count` (a wrong byte count or echo), `func` (a
//!   wrong function code), `extra` (two bytes more) or `excXX` (exception XX);
//! - `setters=0`, which flags any write but the panel lock, for the cases
//!   that show a tool changes nothing it was not asked to.
//!
//! The pedantic checks: a frame with a good CRC, only the functions and
//! register blocks a C path uses, one register per write, nothing left of a
//! frame when a tool opens. A frame for another slave is not this model's
//! business, which `modbusaddr=` cases need.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::{Error, Result};
use std::fmt::Write;

/// The model name.
pub const NAME: &str = "rdtech-dps";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An RDTech DPS, DPH or RD power supply over Modbus RTU on CDC-ACM; id=, \
                         version=, serial=, addr=, uart=, uset=, iset=, ovp=, ocp=, output=, \
                         load=, uout=, iout=, power=, protect=, cc=, enable=, range=, answers=, \
                         faults=, setters=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

const READ_HOLDING: u8 = 0x03;
const WRITE_MULTIPLE: u8 = 0x10;
const EXCEPTION: u8 = 0x80;
const ILLEGAL_FUNCTION: u8 = 0x01;
const ILLEGAL_ADDRESS: u8 = 0x02;
const ILLEGAL_VALUE: u8 = 0x03;
/// The longest frame Modbus has, and what a frame in the making may hold.
const LONGEST: usize = 256;

/// The register map: where each thing is in a family.
#[derive(Clone, Copy, Debug)]
struct Map {
    uset: usize,
    iset: usize,
    uout: usize,
    iout: usize,
    power: usize,
    protect: usize,
    cc: usize,
    enable: usize,
    ovp: usize,
    ocp: usize,
    /// The panel lock; an RD's driver does not write its register.
    lock: Option<usize>,
    /// Further registers that mirror `uset` and `iset`.
    mirror: Option<(usize, usize)>,
    /// The register the block read of a poll starts at.
    poll: usize,
}

const DPS: Map = Map {
    uset: 0x00,
    iset: 0x01,
    uout: 0x02,
    iout: 0x03,
    power: 0x04,
    protect: 0x07,
    cc: 0x08,
    enable: 0x09,
    ovp: 0x52,
    ocp: 0x53,
    lock: Some(0x06),
    mirror: Some((0x50, 0x51)),
    poll: 0x00,
};

const RD: Map = Map {
    uset: 8,
    iset: 9,
    uout: 10,
    iout: 11,
    power: 13,
    protect: 16,
    cc: 17,
    enable: 18,
    ovp: 82,
    ocp: 83,
    lock: None,
    mirror: None,
    poll: 8,
};

const RD_RANGE: usize = 20;

/// The blocks a C path reads, `(address, count)`. The two families' probes are
/// here both: either driver may meet either device.
const READS: [(usize, usize); 7] =
    [(0x00, 10), (0x0b, 2), (0x52, 2), (0, 4), (8, 11), (8, 13), (20, 1)];

/// Volts and amperes in a register's unit, per model number, `(voltage digits,
/// current digits)` of its first range.
fn digits(id: u32) -> (u32, u32) {
    match id {
        5015 | 5020 | 60121 | 60181 | 60241 => (2, 2),
        60065 | 60125 => (3, 4),
        _ => (2, 3),
    }
}

fn pow10(n: u32) -> u64 {
    10u64.pow(n)
}

/// A value of a list, in turn; the last one stays.
#[derive(Clone, Debug, Default)]
struct Played {
    list: Vec<u16>,
    next: usize,
}

impl Played {
    fn take(&mut self) -> Option<u16> {
        let v = self.list.get(self.next.min(self.list.len().checked_sub(1)?)).copied();
        self.next += 1;
        v
    }
}

fn parse_raw(key: &str, value: &str) -> Result<u16> {
    Ok(u16::try_from(parse_number(key, value, 0..=0xffff)?).expect("in range"))
}

fn parse_list(key: &str, value: &str) -> Result<Played> {
    let list = value.split(',').map(|v| parse_raw(key, v)).collect::<Result<Vec<_>>>()?;
    Ok(Played { list, next: 0 })
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
                .ok_or_else(|| Error(format!("faults={value}: N:KIND, in {item}")))
        })
        .collect()
}

#[derive(Clone, Debug)]
struct Config {
    id: u16,
    version: u16,
    serial: u32,
    addr: u8,
    uset: Option<u16>,
    iset: Option<u16>,
    ovp: Option<u16>,
    ocp: Option<u16>,
    output: bool,
    load: Option<u32>,
    uout: Played,
    iout: Played,
    power: Played,
    protect: Played,
    cc: Played,
    enable: Played,
    range: Played,
    answers: Option<u32>,
    faults: Vec<(u32, Kind)>,
    setters: bool,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            id: 5005,
            version: 116,
            serial: 12345,
            addr: 1,
            uset: None,
            iset: None,
            ovp: None,
            ocp: None,
            output: false,
            load: None,
            uout: Played::default(),
            iout: Played::default(),
            power: Played::default(),
            protect: Played::default(),
            cc: Played::default(),
            enable: Played::default(),
            range: Played::default(),
            answers: None,
            faults: Vec::new(),
            setters: true,
        }
    }
}

impl Config {
    fn is_rd(&self) -> bool {
        self.id >= 60000
    }
}

/// A supply from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Supply>> {
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
            "serial" => {
                config.serial = v.parse().map_err(|_| Error(format!("serial={v}: a number")))?;
            }
            "addr" => config.addr = u8::try_from(parse_number(k, v, 1..=247)?).expect("in range"),
            "uart" => uart = Some(Line::parse(v)?),
            "uset" => config.uset = Some(parse_raw(k, v)?),
            "iset" => config.iset = Some(parse_raw(k, v)?),
            "ovp" => config.ovp = Some(parse_raw(k, v)?),
            "ocp" => config.ocp = Some(parse_raw(k, v)?),
            "output" => config.output = parse_number(k, v, 0..=1)? == 1,
            "load" => config.load = Some(parse_number(k, v, 1..=1_000_000)?),
            "uout" => config.uout = parse_list(k, v)?,
            "iout" => config.iout = parse_list(k, v)?,
            "power" => config.power = parse_list(k, v)?,
            "protect" => config.protect = parse_list(k, v)?,
            "cc" => config.cc = parse_list(k, v)?,
            "enable" => config.enable = parse_list(k, v)?,
            "range" => config.range = parse_list(k, v)?,
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "faults" => config.faults = parse_faults(v)?,
            "setters" => config.setters = parse_number(k, v, 0..=1)? == 1,
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    let line = match uart {
        Some(line) => line,
        None => Line::parse(if config.is_rd() { "115200/8n1" } else { "9600/8n1" })?,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "RDTech power supply");
    Ok(Cdc::new(NAME, &id, faults, Supply::new(config)).behind_uart(line))
}

/// CRC-16/MODBUS.
fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0xffff, |crc, b| {
        (0..8).fold(crc ^ u16::from(*b), |c, _| if c & 1 == 1 { c >> 1 ^ 0xa001 } else { c >> 1 })
    })
}

struct Supply {
    config: Config,
    map: Map,
    registers: [u16; 256],
    uout: Played,
    iout: Played,
    power: Played,
    protect: Played,
    cc: Played,
    enable: Played,
    range: Played,
    /// What has come of a frame so far.
    pending: Vec<u8>,
    /// Frames addressed here since power on.
    frames: u32,
}

impl Supply {
    fn new(config: Config) -> Supply {
        let rd = config.is_rd();
        let map = if rd { RD } else { DPS };
        let mut registers = [0; 256];
        if rd {
            registers[0] = config.id;
            registers[1] = u16::try_from(config.serial >> 16).expect("the high half");
            registers[2] = (config.serial & 0xffff) as u16;
            registers[3] = config.version;
            // A 6 V and 1 A setting in whichever scale, and thresholds above.
            registers[RD_RANGE] = config.range.list.first().copied().unwrap_or(0);
        } else {
            registers[0x0b] = config.id;
            registers[0x0c] = config.version;
        }
        let (vd, id) = digits(u32::from(config.id));
        let volts = |v: u64| u16::try_from(v * pow10(vd)).unwrap_or(u16::MAX);
        let amps = |a: u64| u16::try_from(a * pow10(id)).unwrap_or(u16::MAX);
        registers[map.uset] = config.uset.unwrap_or_else(|| volts(5));
        registers[map.iset] = config.iset.unwrap_or_else(|| amps(1));
        registers[map.ovp] = config.ovp.unwrap_or_else(|| volts(40));
        registers[map.ocp] = config.ocp.unwrap_or_else(|| amps(6));
        registers[map.enable] = u16::from(config.output);
        let mut supply = Supply {
            map,
            registers,
            uout: config.uout.clone(),
            iout: config.iout.clone(),
            power: config.power.clone(),
            protect: config.protect.clone(),
            cc: config.cc.clone(),
            enable: config.enable.clone(),
            range: config.range.clone(),
            config,
            pending: Vec::new(),
            frames: 0,
        };
        supply.mirror();
        supply
    }

    fn mirror(&mut self) {
        if let Some((u, i)) = self.map.mirror {
            self.registers[u] = self.registers[self.map.uset];
            self.registers[i] = self.registers[self.map.iset];
        }
    }

    /// The registers' digits now, an RD6012P's current depending on its range.
    fn scale(&self) -> (u64, u64) {
        let (vd, mut id) = digits(u32::from(self.config.id));
        if self.config.id == 60125 && self.registers[RD_RANGE] != 0 {
            id = 3;
        }
        (pow10(vd), pow10(id))
    }

    /// The output's registers from the settings and the load.
    fn operate(&mut self) {
        let map = self.map;
        let (vs, is) = self.scale();
        let uset = u64::from(self.registers[map.uset]);
        let iset = u64::from(self.registers[map.iset]);
        let (uout, iout, cc) = match (self.registers[map.enable] != 0, self.config.load) {
            (false, _) => (0, 0, false),
            (true, None) => (uset, 0, false),
            (true, Some(load)) => {
                let load = u64::from(load);
                let wanted = uset * is / vs / load;
                if wanted <= iset {
                    (uset, wanted, false)
                } else {
                    ((iset * load * vs / is).min(uset), iset, true)
                }
            }
        };
        self.registers[map.uout] = u16::try_from(uout).unwrap_or(u16::MAX);
        self.registers[map.iout] = u16::try_from(iout).unwrap_or(u16::MAX);
        let centiwatts = uout * iout * 100 / vs / is;
        self.registers[map.power] = u16::try_from(centiwatts).unwrap_or(u16::MAX);
        self.registers[map.cc] = u16::from(cc);
    }

    /// What the next poll's block holds: the settings' results, then what is
    /// played over them.
    fn poll(&mut self) {
        self.operate();
        let map = self.map;
        let played = [
            (self.uout.take(), map.uout),
            (self.iout.take(), map.iout),
            (self.power.take(), map.power),
            (self.protect.take(), map.protect),
            (self.cc.take(), map.cc),
            (self.enable.take(), map.enable),
        ];
        for (value, register) in played {
            if let Some(value) = value {
                self.registers[register] = value;
            }
        }
        if let Some(value) = self.range.take()
            && self.config.is_rd()
        {
            self.registers[RD_RANGE] = value;
        }
    }

    /// The answer's bytes after the address, or an exception.
    fn answer(&mut self, frame: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let function = frame[1];
        let address = usize::from(u16::from_be_bytes([frame[2], frame[3]]));
        let count = usize::from(u16::from_be_bytes([frame[4], frame[5]]));
        match function {
            READ_HOLDING => {
                if !READS.contains(&(address, count)) {
                    port.violation(format!("reading {count} registers at {address:#x}: no C path"));
                }
                if address + count > self.registers.len() || count == 0 || count > 125 {
                    return vec![function | EXCEPTION, ILLEGAL_ADDRESS];
                }
                if address == self.map.poll && count > 2 {
                    self.poll();
                }
                let mut out = vec![function, (count * 2) as u8];
                for r in &self.registers[address..address + count] {
                    out.extend_from_slice(&r.to_be_bytes());
                }
                out
            }
            WRITE_MULTIPLE => {
                if count != 1 {
                    port.violation(format!("writing {count} registers: no C path"));
                }
                if usize::from(frame[6]) != count * 2 {
                    port.violation(format!("{count} registers in {} bytes", frame[6]));
                    return vec![function | EXCEPTION, ILLEGAL_VALUE];
                }
                if address + count > self.registers.len() {
                    port.violation(format!("writing at {address:#x}: off the map"));
                    return vec![function | EXCEPTION, ILLEGAL_ADDRESS];
                }
                for i in 0..count {
                    let at = 7 + i * 2;
                    self.write(address + i, u16::from_be_bytes([frame[at], frame[at + 1]]), port);
                }
                frame[1..6].to_vec()
            }
            other => {
                port.violation(format!("function {other:#04x}: no C path"));
                vec![other | EXCEPTION, ILLEGAL_FUNCTION]
            }
        }
    }

    fn write(&mut self, address: usize, value: u16, port: &mut Port<'_>) {
        let map = self.map;
        let lock = map.lock == Some(address);
        let rd_range = self.config.is_rd() && address == RD_RANGE;
        let settable = [map.uset, map.iset, map.enable, map.ovp, map.ocp];
        if !lock && !rd_range && !settable.contains(&address) {
            port.violation(format!("writing {value} at {address:#x}: no C path"));
        }
        if !lock && !self.config.setters {
            port.violation(format!("writing {value} at {address:#x}: a setting no one asked for"));
        }
        if rd_range && self.config.id != 60125 {
            port.violation("writing the range of a model that has one: no C path");
        }
        self.registers[address] = value;
        self.mirror();
    }

    /// One frame for this slave: the answer, as the faults leave it.
    fn serve(&mut self, frame: &[u8], port: &mut Port<'_>) {
        self.frames += 1;
        let number = self.frames;
        let mut answer = self.answer(frame, port);
        if self.config.answers.is_some_and(|n| number > n) {
            port.mark_event_point();
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

impl Serial for Supply {
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

    fn reset(&mut self) {
        *self = Supply::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    const NOTHING: Vec<u8> = Vec::new();

    fn line(text: &str) -> Line {
        Line::parse(text).unwrap()
    }

    fn supply_on(options: &[(&str, &str)], at: &str) -> Host<Supply> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), line(at), true, true)
    }

    fn dps(options: &[(&str, &str)]) -> Host<Supply> {
        supply_on(options, "9600/8n1")
    }

    fn rd(options: &[(&str, &str)]) -> Host<Supply> {
        let mut o = vec![("id", "60125")];
        o.extend_from_slice(options);
        supply_on(&o, "115200/8n1")
    }

    fn violations(h: &mut Host<Supply>) -> usize {
        h.v.drain().len()
    }

    fn framed(body: &[u8]) -> Vec<u8> {
        let mut f = body.to_vec();
        f.extend_from_slice(&crc16(body).to_le_bytes());
        f
    }

    fn read_request(address: u16, count: u16) -> Vec<u8> {
        let [ah, al] = address.to_be_bytes();
        let [ch, cl] = count.to_be_bytes();
        framed(&[1, READ_HOLDING, ah, al, ch, cl])
    }

    fn write_request(address: u16, value: u16) -> Vec<u8> {
        let [ah, al] = address.to_be_bytes();
        let [vh, vl] = value.to_be_bytes();
        framed(&[1, WRITE_MULTIPLE, ah, al, 0, 1, 2, vh, vl])
    }

    /// The registers of an answer, its CRC checked.
    fn registers(reply: &[u8]) -> Vec<u16> {
        let (body, crc) = reply.split_at(reply.len() - 2);
        assert_eq!(crc16(body).to_le_bytes(), crc, "the CRC of {reply:02x?}");
        assert_eq!(usize::from(body[2]), body.len() - 3);
        body[3..].chunks(2).map(|c| u16::from_be_bytes([c[0], c[1]])).collect()
    }

    #[test]
    fn the_crc_is_modbus() {
        // The worked example of the Modbus over serial line guide.
        assert_eq!(crc16(&[0x01, 0x03, 0x00, 0x00, 0x00, 0x01]), 0x0a84);
    }

    #[test]
    fn a_dps_says_its_model_and_version() {
        let mut h = dps(&[("id", "5020"), ("version", "17")]);
        h.write(&read_request(0x0b, 2));
        assert_eq!(registers(&h.read()), [5020, 17]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn an_rd_says_its_model_serial_and_firmware() {
        let mut h = rd(&[("serial", "305419896"), ("version", "128")]);
        h.write(&read_request(0, 4));
        assert_eq!(registers(&h.read()), [60125, 0x1234, 0x5678, 128]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_write_is_echoed_and_mirrored() {
        let mut h = dps(&[]);
        h.write(&write_request(0x00, 1234));
        let echo = h.read();
        assert_eq!(echo[..6], [1, WRITE_MULTIPLE, 0, 0, 0, 1]);
        assert_eq!(crc16(&echo[..6]).to_le_bytes(), echo[6..]);
        assert_eq!(violations(&mut h), 0);
        h.write(&read_request(0x50, 1));
        assert_eq!(registers(&h.read()), [1234], "0x50 mirrors the active setting");
        assert_eq!(violations(&mut h), 1, "no C path reads it");
    }

    #[test]
    fn the_output_follows_the_load() {
        let mut h = dps(&[("uset", "1000"), ("iset", "500"), ("output", "1"), ("load", "10")]);
        h.write(&read_request(0, 10));
        let r = registers(&h.read());
        // 10 V across 10 ohm wants 1 A, and 0.5 A is the limit: CC, 5 V, 2.5 W.
        assert_eq!((r[2], r[3], r[4], r[8]), (500, 500, 250, 1));
        let mut h = dps(&[("uset", "1000"), ("iset", "2000"), ("output", "1"), ("load", "10")]);
        h.write(&read_request(0, 10));
        let r = registers(&h.read());
        assert_eq!((r[2], r[3], r[4], r[8]), (1000, 1000, 1000, 0), "CV: 1 A, 10 W");
    }

    #[test]
    fn played_values_go_in_turn_and_the_last_stays() {
        let mut h = dps(&[("uout", "1,2"), ("protect", "0,1,2")]);
        let mut seen = Vec::new();
        for _ in 0..4 {
            h.write(&read_request(0, 10));
            let r = registers(&h.read());
            seen.push((r[2], r[7]));
        }
        assert_eq!(seen, [(1, 0), (2, 1), (2, 2), (2, 2)]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_thresholds_read_does_not_play_a_value() {
        let mut h = dps(&[("uout", "1,2")]);
        h.write(&read_request(0x52, 2));
        h.read();
        h.write(&read_request(0, 10));
        assert_eq!(registers(&h.read())[2], 1);
    }

    #[test]
    fn the_range_of_an_rd_has_a_register_and_a_list() {
        let mut h = rd(&[("range", "1,0")]);
        h.write(&read_request(20, 1));
        assert_eq!(registers(&h.read()), [1], "the list's first at power on");
        h.write(&read_request(8, 13));
        assert_eq!(registers(&h.read())[12], 1);
        h.write(&read_request(8, 13));
        assert_eq!(registers(&h.read())[12], 0);
        h.write(&write_request(20, 1));
        h.read();
        h.write(&read_request(20, 1));
        assert_eq!(registers(&h.read()), [1]);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_scale_of_an_rd6012p_follows_its_range() {
        let mut s = Supply::new(Config { id: 60125, ..Config::default() });
        assert_eq!(s.scale(), (1000, 10000));
        s.registers[RD_RANGE] = 1;
        assert_eq!(s.scale(), (1000, 1000));
    }

    #[test]
    fn a_line_the_uart_does_not_take_passes_nothing() {
        let mut h = supply_on(&[], "19200/8n1");
        h.write(&read_request(0x0b, 2));
        assert_eq!(h.read(), NOTHING);
    }

    #[test]
    fn another_slaves_frame_gets_no_answer_and_no_flag() {
        let mut h = dps(&[("addr", "5")]);
        h.write(&read_request(0x0b, 2));
        assert_eq!(h.read(), NOTHING);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_slave_at_another_address_answers_from_it() {
        let mut h = dps(&[("addr", "5")]);
        h.write(&framed(&[5, READ_HOLDING, 0, 0x0b, 0, 2]));
        assert_eq!(h.read()[0], 5);
    }

    #[test]
    fn a_frame_may_come_in_any_pieces() {
        let mut h = dps(&[]);
        let w = write_request(0x09, 1);
        for b in &w {
            h.write(std::slice::from_ref(b));
        }
        assert_eq!(h.read().len(), 8);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_bad_crc_is_flagged() {
        let mut h = dps(&[]);
        let mut f = read_request(0x0b, 2);
        f[6] ^= 1;
        h.write(&f);
        assert_eq!(h.read(), NOTHING);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_block_no_c_path_reads_is_flagged() {
        let mut h = dps(&[]);
        h.write(&read_request(0x0b, 1));
        assert_eq!(registers(&h.read()), [5005], "it answers as a real one would");
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_read_off_the_map_is_an_exception() {
        let mut h = dps(&[]);
        h.write(&read_request(0xff, 2));
        assert_eq!(h.read()[..3], [1, READ_HOLDING | EXCEPTION, ILLEGAL_ADDRESS]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_write_off_the_map_is_an_exception() {
        let mut h = dps(&[]);
        h.write(&write_request(0x1000, 1));
        assert_eq!(h.read()[..3], [1, WRITE_MULTIPLE | EXCEPTION, ILLEGAL_ADDRESS]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_function_no_c_path_sends_is_an_exception() {
        let mut h = dps(&[]);
        h.write(&framed(&[1, 0x06, 0, 9, 0, 1]));
        assert_eq!(h.read()[..3], [1, 0x86, ILLEGAL_FUNCTION]);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_write_no_c_path_makes_is_flagged() {
        let mut h = dps(&[]);
        h.write(&write_request(0x23, 1));
        h.read();
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn writing_two_registers_is_flagged() {
        let mut h = dps(&[]);
        h.write(&framed(&[1, WRITE_MULTIPLE, 0, 0, 0, 2, 4, 0, 1, 0, 2]));
        h.read();
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_count_that_the_bytes_do_not_hold_is_flagged() {
        let mut h = dps(&[]);
        h.write(&framed(&[1, WRITE_MULTIPLE, 0, 0, 0, 100, 2, 0, 1]));
        assert_eq!(h.read()[..3], [1, WRITE_MULTIPLE | EXCEPTION, ILLEGAL_VALUE]);
        assert_eq!(violations(&mut h), 2);
    }

    #[test]
    fn the_range_of_a_model_with_one_range_is_flagged() {
        let mut h = supply_on(&[("id", "60061")], "115200/8n1");
        h.write(&write_request(20, 1));
        h.read();
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn setters_0_flags_a_setting_but_not_the_lock() {
        let mut h = dps(&[("setters", "0")]);
        h.write(&write_request(0x06, 1));
        h.read();
        assert_eq!(violations(&mut h), 0);
        h.write(&write_request(0x09, 1));
        h.read();
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_frame_left_unfinished_is_flagged_at_an_open() {
        let mut h = dps(&[]);
        h.write(&[1, READ_HOLDING]);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_stream_of_no_frame_is_bounded() {
        let mut h = dps(&[]);
        // A write that says it holds 255 bytes, and never ends.
        h.write(&[1, WRITE_MULTIPLE, 0, 0, 0, 1, 255]);
        for _ in 0..10 {
            h.write(&[0; 64]);
        }
        h.write(&[0xff; 64]);
        // Whatever it made of that, the CRC did not fit.
        assert!(violations(&mut h) >= 1);
    }

    #[test]
    fn answers_runs_out() {
        let mut h = dps(&[("answers", "1")]);
        h.write(&read_request(0x0b, 2));
        assert_ne!(h.read(), NOTHING);
        h.write(&read_request(0x0b, 2));
        assert_eq!(h.read(), NOTHING);
    }

    #[test]
    fn each_fault_does_its_harm() {
        let request = read_request(0x0b, 2);
        let good = {
            let mut h = dps(&[]);
            h.write(&request);
            h.read()
        };
        let reply = |kind: &str| {
            let mut h = dps(&[("faults", kind)]);
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
            ("addr", "248"),
            ("faults", "1"),
            ("faults", "x:crc"),
            ("faults", "1:what"),
            ("uout", "1,x"),
            ("uart", "fast"),
            ("nosuch", "1"),
        ] {
            assert!(function(&[(k.to_owned(), v.to_owned())]).is_err(), "{k}={v}");
        }
    }
}
