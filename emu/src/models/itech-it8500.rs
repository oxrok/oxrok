// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! An ITECH IT8500 series electronic load (the same as a BK Precision 8500) on
//! the CDC-ACM shell: fixed 26-byte packets, each one answered by one.
//!
//! The device, and how sure this is:
//!
//! - The USB side: the load has a TTL serial port and the vendor's adapter is
//!   a PL-2303 (BK's manual, "Remote operation"). This model is NOT that: a
//!   CDC-ACM function with test ids stands in, so the tools see a tty and the
//!   kernel's `pl2303` does not come into it. That is an emulation artifact.
//! - The line: 8n1 at 4800, 9600, 19200 or 38400 (the manual). The factory's
//!   9600 is `libsigrok`'s comment; `uart=` sets the others.
//! - The packet, the status packet and its codes, `20`, `21`, `22` to `25`,
//!   `28`, `2a` to `31`, `5f`, `6a`, `6b`, the units and the state registers:
//!   the manual of the BK Precision 8500 series, version 032218. The bar code
//!   is ASCII there, in fields this model does not look into.
//! - That an answer to a request that fails is a status packet, for reads too:
//!   the manual.
//! - `01`, the limits, and `ff` as the broadcast address: ITECH's IT8500+
//!   Programming Guide, pp. 5, 6 and 18, which lists `ff` for most commands and
//!   addresses 0 to 31, where BK's manual has up to `fe`. The guide's `01` gives
//!   the order of the fields and no units: those are what the C driver reads.
//!   That a load answers a broadcast `6a`, and with its own address, rests on
//!   the driver alone, whose author had the hardware.
//! - The ranges a load refuses: the manual gives the code, `a0`, and not when.
//!   That it refuses a setting beyond its limits is an inference.
//! - The limits at power on are a BK 8500's, 120 V, 30 A, 300 W (the sigrok
//!   wiki); the resistance range and the minimum voltage are stand-ins.
//!
//! Emulation artifacts:
//!
//! - It answers at once, where a real one takes at least the packet's 27 ms on
//!   the line at 9600, so nothing here can check a driver's pacing.
//! - It keeps a packet across pauses, however the writes cut it. How a real
//!   one resynchronises is not known.
//! - What it measures is a stand-in, [`Load::reading`]: a `source=` of a fixed
//!   voltage behind 1 ohm, in whichever mode is set. The limits do not trip.
//!   The protection flags come only from `state=`.
//!
//! Not modelled: every command the driver does not send. Each gets `b0` and a
//! violation.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `model=TEXT`, up to 5 characters, `firmware=HIGH.LOW` in hex bytes,
//!   `serial=TEXT`, up to 10, `barcode=TEXT`, up to 22, `address=N`;
//! - `max-current=N` in 0.1 mA, `max-voltage=N`, `min-voltage=N` in mV,
//!   `max-power=N` in mW, `max-resistance=N`, `min-resistance=N` in mOhm;
//! - `load=0|1`, `mode=0..3`, `cc=N`, `cv=N`, `cw=N`, `cr=N`, `ovp=N`, `ocp=N`,
//!   `source=N` in mV, the state at power on;
//! - `state=HEX[,HEX...]`, the data of a `5f`'s answers, played in turn and over
//!   again, instead of the computed one; a good checksum goes with them;
//! - `status-CC=SS`, a command `CC` that answers with status `SS` and does
//!   nothing;
//! - `silent-CC=1`, a command `CC` that gets no answer, and does what it does;
//! - `reply-CC=HEX[,HEX...]`, whole answers to `CC` played in turn and over
//!   again, of any length up to 64, for a device gone wrong;
//! - `answers=N`, packets answered per power on, then silence and the event
//!   point;
//! - `uart=LINE`.
//!
//! The pedantic checks: the preamble, the checksum, the command, that unused
//! bytes are 0 (the manual: "good programming practice", and the driver does),
//! a broadcast of anything but `6a`, a setting while not in remote mode, and
//! nothing left of a packet when a tool opens. A packet for another address gets
//! no answer, and no violation: a load on a shared bus sees those.
//!
//! Two of these are inferences, and flag what the driver does not do, not what a
//! load is known to refuse, so each still answers. The guide lists `ff` for most
//! commands, so a broadcast of another is no fault of the device's. And the guide,
//! p. 8, asks for command `20` ahead of controlling the input ("please setting
//! electronic load to PC control state"); what a load does without it, neither
//! it nor BK's manual says.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::Packets;
use crate::{Error, Result};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

/// The model name.
pub const NAME: &str = "itech-it8500";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "An ITECH IT8500 electronic load on CDC-ACM; model=, firmware=, serial=, \
                         barcode=, address=, max-current=, max-voltage=, min-voltage=, \
                         max-power=, max-resistance=, min-resistance=, load=, mode=, cc=, cv=, \
                         cw=, cr=, ovp=, ocp=, source=, state=, status-CC=, silent-CC=, \
                         reply-CC=, answers=, uart=, and the shell's stall=, chunk=, in-limit=, \
                         in-stall=, out-hold=";

/// A packet's length, both ways.
const PACKET: usize = 26;
/// The data's length within a packet.
const DATA: usize = 22;
const PREAMBLE: u8 = 0xaa;
const BROADCAST: u8 = 0xff;
const LIMITS: u8 = 0x01;
const STATUS: u8 = 0x12;
const REMOTE: u8 = 0x20;
const LOAD_ON_OFF: u8 = 0x21;
const SET_OVP: u8 = 0x22;
const GET_OVP: u8 = 0x23;
const SET_OCP: u8 = 0x24;
const GET_OCP: u8 = 0x25;
const SET_MODE: u8 = 0x28;
const SET_CC: u8 = 0x2a;
const GET_CC: u8 = 0x2b;
const SET_CV: u8 = 0x2c;
const GET_CV: u8 = 0x2d;
const SET_CW: u8 = 0x2e;
const GET_CW: u8 = 0x2f;
const SET_CR: u8 = 0x30;
const GET_CR: u8 = 0x31;
const GET_STATE: u8 = 0x5f;
const MODEL_INFO: u8 = 0x6a;
const BARCODE: u8 = 0x6b;
const OK: u8 = 0x80;
const BAD_CHECKSUM: u8 = 0x90;
const BAD_PARAMETER: u8 = 0xa0;
const UNKNOWN_COMMAND: u8 = 0xb0;
/// The operation state's bits.
const OS_REMOTE: u8 = 1 << 2;
const OS_LOAD: u8 = 1 << 3;
/// The demand state's first mode bit, constant current; the others follow.
const DS_MODE: u16 = 1 << 6;
/// Answers played by `reply-CC=` may be longer than a packet, up to this.
const LONGEST_REPLY: usize = 64;

/// A load from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn text(key: &str, value: &str, longest: usize) -> Result<Vec<u8>> {
    if value.len() > longest || !value.is_ascii() {
        return Err(Error::Invalid(format!("{key}={value}: up to {longest} ASCII characters")));
    }
    Ok(value.as_bytes().to_vec())
}

fn command_of(key: &str, hex: &str) -> Result<u8> {
    if hex.len() != 2 {
        return Err(Error::Invalid(format!("{key}: two hex digits name a command")));
    }
    u8::from_str_radix(hex, 16).map_err(|_| Error::Invalid(format!("{key}: not a command")))
}

fn byte_of(key: &str, value: &str) -> Result<u8> {
    if value.len() != 2 {
        return Err(Error::Invalid(format!("{key}={value}: two hex digits")));
    }
    u8::from_str_radix(value, 16).map_err(|_| Error::Invalid(format!("{key}={value}: hex")))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Load>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("9600/8n1")?;
    let mut config = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        if let Some(hex) = k.strip_prefix("status-") {
            config.status.insert(command_of(k, hex)?, byte_of(k, v)?);
            continue;
        }
        if let Some(hex) = k.strip_prefix("silent-") {
            if parse_number(k, v, 0..=1)? == 1 {
                config.silent.insert(command_of(k, hex)?);
            }
            continue;
        }
        if let Some(hex) = k.strip_prefix("reply-") {
            let list = Packets::new(parse_hex_list(k, v, 1, LONGEST_REPLY)?);
            config.replies.insert(command_of(k, hex)?, list);
            continue;
        }
        let u32_max = 0..=u32::MAX;
        match k.as_str() {
            "model" => config.model = text(k, v, 5)?,
            "firmware" => {
                let (high, low) = v
                    .split_once('.')
                    .ok_or_else(|| Error::Invalid(format!("{k}={v}: HIGH.LOW in hex")))?;
                config.firmware = (byte_of(k, high)?, byte_of(k, low)?);
            }
            "serial" => config.serial = text(k, v, 10)?,
            "barcode" => config.barcode = text(k, v, DATA)?,
            "address" => {
                config.address = u8::try_from(parse_number(k, v, 0..=254)?).expect("in a byte");
            }
            "max-current" => config.max_current = parse_number(k, v, u32_max)?,
            "max-voltage" => config.max_voltage = parse_number(k, v, u32_max)?,
            "min-voltage" => config.min_voltage = parse_number(k, v, u32_max)?,
            "max-power" => config.max_power = parse_number(k, v, u32_max)?,
            "max-resistance" => config.max_resistance = parse_number(k, v, u32_max)?,
            "min-resistance" => {
                config.min_resistance =
                    u16::try_from(parse_number(k, v, 0..=0xffff)?).expect("in 16 bits");
            }
            "load" => config.power_on.load = parse_number(k, v, 0..=1)? == 1,
            "mode" => {
                config.power_on.mode = u8::try_from(parse_number(k, v, 0..=3)?).expect("a mode");
            }
            "cc" => config.power_on.cc = parse_number(k, v, u32_max)?,
            "cv" => config.power_on.cv = parse_number(k, v, u32_max)?,
            "cw" => config.power_on.cw = parse_number(k, v, u32_max)?,
            "cr" => config.power_on.cr = parse_number(k, v, u32_max)?,
            "ovp" => config.power_on.ovp = parse_number(k, v, u32_max)?,
            "ocp" => config.power_on.ocp = parse_number(k, v, u32_max)?,
            "source" => config.source = parse_number(k, v, 0..=10_000_000)?,
            "state" => config.state = Some(Packets::new(parse_hex_list(k, v, 0, DATA)?)),
            "answers" => config.answers = Some(parse_number(k, v, 0..=100_000)?),
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    // Test ids: see the module's note on the real adapter.
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "ITECH IT8500 stand-in");
    Ok(Cdc::new(NAME, &id, faults, Load::new(config)).behind_uart(uart))
}

/// What a load keeps of its settings.
#[derive(Clone, Copy, Debug)]
struct Panel {
    load: bool,
    mode: u8,
    /// In 0.1 mA.
    cc: u32,
    /// In mV.
    cv: u32,
    /// In mW.
    cw: u32,
    /// In mOhm.
    cr: u32,
    /// In mV.
    ovp: u32,
    /// In 0.1 mA.
    ocp: u32,
}

#[derive(Clone, Debug)]
struct Config {
    model: Vec<u8>,
    firmware: (u8, u8),
    serial: Vec<u8>,
    barcode: Vec<u8>,
    address: u8,
    max_current: u32,
    max_voltage: u32,
    min_voltage: u32,
    max_power: u32,
    max_resistance: u32,
    min_resistance: u16,
    source: u32,
    power_on: Panel,
    state: Option<Packets>,
    status: BTreeMap<u8, u8>,
    silent: BTreeSet<u8>,
    replies: BTreeMap<u8, Packets>,
    answers: Option<u32>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            model: b"8500".to_vec(),
            firmware: (0x01, 0x23),
            serial: b"0123456789".to_vec(),
            barcode: b"BK8500012020".to_vec(),
            address: 0,
            max_current: 300_000,
            max_voltage: 120_000,
            min_voltage: 100,
            max_power: 300_000,
            max_resistance: 7_500_000,
            min_resistance: 50,
            source: 12_000,
            power_on: Panel {
                load: false,
                mode: 0,
                cc: 10_000,
                cv: 5_000,
                cw: 12_000,
                cr: 100_000,
                ovp: 120_000,
                ocp: 300_000,
            },
            state: None,
            status: BTreeMap::new(),
            silent: BTreeSet::new(),
            replies: BTreeMap::new(),
            answers: None,
        }
    }
}

struct Load {
    config: Config,
    now: Panel,
    remote: bool,
    state: Option<Packets>,
    replies: BTreeMap<u8, Packets>,
    /// What has come of a packet so far, shorter than a packet.
    pending: Vec<u8>,
    answered: u32,
}

impl Load {
    fn new(config: Config) -> Load {
        Load {
            now: config.power_on,
            remote: false,
            state: config.state.clone(),
            replies: config.replies.clone(),
            config,
            pending: Vec::new(),
            answered: 0,
        }
    }

    /// Volts in mV, amperes in 0.1 mA, watts in mW: a stand-in, a fixed source
    /// behind 1 ohm, and the current held at the load's limit.
    fn reading(&self) -> (u32, u32, u32) {
        let source = u64::from(self.config.source);
        if !self.now.load {
            return (self.config.source, 0, 0);
        }
        let (volts, amps) = match self.now.mode {
            0 => (source, u64::from(self.now.cc)),
            1 => {
                let volts = source.min(u64::from(self.now.cv));
                (volts, (source - volts) * 10)
            }
            2 => (source, u64::from(self.now.cw) * 10_000 / source.max(1)),
            _ => (source, source * 10_000 / u64::from(self.now.cr).max(1)),
        };
        let amps = amps.min(u64::from(self.config.max_current));
        let watts = volts * amps / 10_000;
        let clamp = |n: u64| u32::try_from(n).unwrap_or(u32::MAX);
        (clamp(volts), clamp(amps), clamp(watts))
    }

    fn state_data(&mut self) -> Vec<u8> {
        if let Some(list) = &mut self.state {
            return list.take();
        }
        let (volts, amps, watts) = self.reading();
        let mut data = Vec::with_capacity(DATA);
        data.extend_from_slice(&volts.to_le_bytes());
        data.extend_from_slice(&amps.to_le_bytes());
        data.extend_from_slice(&watts.to_le_bytes());
        let mut operation = 0;
        if self.remote {
            operation |= OS_REMOTE;
        }
        if self.now.load {
            operation |= OS_LOAD;
        }
        data.push(operation);
        data.extend_from_slice(&(DS_MODE << self.now.mode).to_le_bytes());
        data
    }

    fn model_info(&self) -> Vec<u8> {
        let mut data = vec![0; 17];
        data[..self.config.model.len()].copy_from_slice(&self.config.model);
        data[5] = self.config.firmware.1;
        data[6] = self.config.firmware.0;
        data[7..7 + self.config.serial.len()].copy_from_slice(&self.config.serial);
        data
    }

    fn limits(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(DATA);
        for n in [
            self.config.max_current,
            self.config.max_voltage,
            self.config.min_voltage,
            self.config.max_power,
            self.config.max_resistance,
        ] {
            data.extend_from_slice(&n.to_le_bytes());
        }
        data.extend_from_slice(&self.config.min_resistance.to_le_bytes());
        data
    }

    fn packet(&self, command: u8, data: &[u8]) -> Vec<u8> {
        let mut p = vec![0; PACKET];
        p[0] = PREAMBLE;
        p[1] = self.config.address;
        p[2] = command;
        p[3..3 + data.len()].copy_from_slice(data);
        p[PACKET - 1] = checksum(&p);
        p
    }

    fn status(&self, code: u8) -> Vec<u8> {
        self.packet(STATUS, &[code])
    }

    /// A setting of one byte: its status.
    fn set_byte(&mut self, command: u8, value: u8) -> u8 {
        match (command, value) {
            (REMOTE, 0 | 1) => self.remote = value == 1,
            (LOAD_ON_OFF, 0 | 1) => self.now.load = value == 1,
            (SET_MODE, 0..=3) => self.now.mode = value,
            _ => return BAD_PARAMETER,
        }
        OK
    }

    /// A setting of four bytes: its status. A negative value of the driver's
    /// comes as a large one, and is refused for that.
    fn set_word(&mut self, command: u8, value: u32) -> u8 {
        let (slot, range) = match command {
            SET_CC => (&mut self.now.cc, 0..=self.config.max_current),
            SET_CV => (&mut self.now.cv, self.config.min_voltage..=self.config.max_voltage),
            SET_CW => (&mut self.now.cw, 0..=self.config.max_power),
            SET_CR => (
                &mut self.now.cr,
                u32::from(self.config.min_resistance)..=self.config.max_resistance,
            ),
            SET_OVP => (&mut self.now.ovp, 0..=self.config.max_voltage),
            _ => (&mut self.now.ocp, 0..=self.config.max_current),
        };
        if !range.contains(&value) {
            return BAD_PARAMETER;
        }
        *slot = value;
        OK
    }

    /// How many data bytes `command` uses; None when it is not one of ours.
    fn data_used(command: u8) -> Option<usize> {
        Some(match command {
            LIMITS | GET_OVP | GET_OCP | GET_CC | GET_CV | GET_CW | GET_CR | GET_STATE
            | MODEL_INFO | BARCODE => 0,
            REMOTE | LOAD_ON_OFF | SET_MODE => 1,
            SET_OVP | SET_OCP | SET_CC | SET_CV | SET_CW | SET_CR => 4,
            _ => return None,
        })
    }

    /// What `command` does, and the answer it gives.
    fn act(&mut self, command: u8, data: &[u8], port: &mut Port<'_>) -> Vec<u8> {
        let word = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let got = |n: u32| n.to_le_bytes();
        match command {
            REMOTE | LOAD_ON_OFF | SET_MODE => {
                let code = self.set_byte(command, data[0]);
                self.status(code)
            }
            SET_OVP | SET_OCP | SET_CC | SET_CV | SET_CW | SET_CR => {
                let code = self.set_word(command, word);
                self.status(code)
            }
            LIMITS => self.packet(command, &self.limits()),
            GET_OVP => self.packet(command, &got(self.now.ovp)),
            GET_OCP => self.packet(command, &got(self.now.ocp)),
            GET_CC => self.packet(command, &got(self.now.cc)),
            GET_CV => self.packet(command, &got(self.now.cv)),
            GET_CW => self.packet(command, &got(self.now.cw)),
            GET_CR => self.packet(command, &got(self.now.cr)),
            GET_STATE => {
                let data = self.state_data();
                self.packet(command, &data)
            }
            MODEL_INFO => self.packet(command, &self.model_info()),
            BARCODE => self.packet(command, &self.config.barcode),
            _ => {
                port.violation(format!("{command:02x}: a command the driver does not have"));
                self.status(UNKNOWN_COMMAND)
            }
        }
    }

    fn handle(&mut self, p: &[u8], port: &mut Port<'_>) {
        if p[0] != PREAMBLE {
            port.violation(format!("{}: no preamble", show(p)));
            return;
        }
        if p[PACKET - 1] != checksum(p) {
            port.violation(format!("{}: wrong checksum", show(p)));
            let answer = self.status(BAD_CHECKSUM);
            self.answer(&answer, port);
            return;
        }
        let (address, command) = (p[1], p[2]);
        if address != self.config.address && address != BROADCAST {
            return;
        }
        if address == BROADCAST && command != MODEL_INFO {
            port.violation(format!("{}: only 6a is broadcast", show(p)));
        }
        let data = &p[3..PACKET - 1];
        if let Some(used) = Load::data_used(command) {
            if data[used..].iter().any(|b| *b != 0) {
                port.violation(format!("{}: unused bytes that are not 0", show(p)));
            }
            let a_setting = used > 0 && command != REMOTE;
            let remote_is_faulted = self.config.status.contains_key(&REMOTE)
                || self.config.replies.contains_key(&REMOTE);
            if a_setting && !self.remote && !remote_is_faulted {
                port.violation(format!("{}: a setting while not in remote mode", show(p)));
            }
        }
        let answer = if let Some(code) = self.config.status.get(&command) {
            self.status(*code)
        } else {
            let computed = self.act(command, data, port);
            match self.replies.get_mut(&command) {
                Some(list) => list.take(),
                None => computed,
            }
        };
        if !self.config.silent.contains(&command) {
            self.answer(&answer, port);
        }
    }

    fn answer(&mut self, answer: &[u8], port: &mut Port<'_>) {
        if self.config.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        port.send(answer);
    }
}

fn checksum(packet: &[u8]) -> u8 {
    packet[..PACKET - 1].iter().fold(0, |s: u8, b| s.wrapping_add(*b))
}

fn show(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

impl Serial for Load {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        // Bounded: what is left of `pending` after the loop is under a packet.
        for chunk in data.chunks(PACKET) {
            self.pending.extend_from_slice(chunk);
            while self.pending.len() >= PACKET {
                let packet: Vec<u8> = self.pending.drain(..PACKET).collect();
                self.handle(&packet, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() && !self.pending.is_empty() {
            port.violation(format!("{}: a packet left unfinished", show(&self.pending)));
            self.pending.clear();
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

    fn load(options: &[(&str, &str)]) -> Host<Load> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true)
    }

    fn cmd(address: u8, command: u8, data: &[u8]) -> Vec<u8> {
        let mut p = vec![0; PACKET];
        p[0] = PREAMBLE;
        p[1] = address;
        p[2] = command;
        p[3..3 + data.len()].copy_from_slice(data);
        p[PACKET - 1] = checksum(&p);
        p
    }

    fn ask(h: &mut Host<Load>, command: u8, data: &[u8]) -> Vec<u8> {
        h.write(&cmd(0, command, data));
        h.read()
    }

    fn word(a: &[u8], at: usize) -> u32 {
        u32::from_le_bytes([a[at], a[at + 1], a[at + 2], a[at + 3]])
    }

    fn violations(h: &mut Host<Load>) -> usize {
        h.v.drain().len()
    }

    fn remote(h: &mut Host<Load>) {
        assert_eq!(ask(h, REMOTE, &[1])[3], OK);
    }

    #[test]
    fn the_model_info_answers_a_broadcast_with_the_address() {
        let mut h = load(&[("address", "7"), ("model", "85112"), ("firmware", "02.5a")]);
        h.write(&cmd(BROADCAST, MODEL_INFO, &[]));
        let a = h.read();
        assert_eq!(a.len(), PACKET);
        assert_eq!(a[..3], [PREAMBLE, 7, MODEL_INFO]);
        assert_eq!(a[3..8], *b"85112");
        assert_eq!((a[8], a[9]), (0x5a, 0x02));
        assert_eq!(a[10..20], *b"0123456789");
        assert_eq!(a[PACKET - 1], checksum(&a));
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn another_address_gets_no_answer_and_no_violation() {
        let mut h = load(&[("address", "7")]);
        h.write(&cmd(0, GET_STATE, &[]));
        assert_eq!(h.read(), b"");
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn the_limits_are_as_the_driver_reads_them() {
        let mut h = load(&[("min-resistance", "51")]);
        let a = ask(&mut h, LIMITS, &[]);
        assert_eq!(a[2], LIMITS);
        assert_eq!(word(&a, 3), 300_000);
        assert_eq!(word(&a, 7), 120_000);
        assert_eq!(word(&a, 11), 100);
        assert_eq!(word(&a, 15), 300_000);
        assert_eq!(word(&a, 19), 7_500_000);
        assert_eq!(u16::from_le_bytes([a[23], a[24]]), 51);
    }

    #[test]
    fn the_barcode_is_text() {
        let mut h = load(&[("barcode", "ABC")]);
        let a = ask(&mut h, BARCODE, &[]);
        assert_eq!(a[3..7], *b"ABC\0");
    }

    #[test]
    fn a_state_follows_what_was_set() {
        let mut h = load(&[]);
        let a = ask(&mut h, GET_STATE, &[]);
        assert_eq!(word(&a, 3), 12_000);
        assert_eq!(word(&a, 7), 0);
        assert_eq!(word(&a, 11), 0);
        assert_eq!(a[15], 0);
        assert_eq!(u16::from_le_bytes([a[16], a[17]]), 1 << 6);
        remote(&mut h);
        assert_eq!(ask(&mut h, LOAD_ON_OFF, &[1])[3], OK);
        let a = ask(&mut h, GET_STATE, &[]);
        assert_eq!(word(&a, 7), 10_000, "1 A");
        assert_eq!(word(&a, 11), 12_000, "12 W");
        assert_eq!(a[15], OS_REMOTE | OS_LOAD);
        assert_eq!(ask(&mut h, SET_MODE, &[1])[3], OK);
        let a = ask(&mut h, GET_STATE, &[]);
        assert_eq!(word(&a, 3), 5_000, "held at the CV setting");
        assert_eq!(word(&a, 7), 70_000, "7 V over 1 ohm");
        assert_eq!(u16::from_le_bytes([a[16], a[17]]), 1 << 7);
        assert_eq!(ask(&mut h, SET_MODE, &[2])[3], OK);
        let a = ask(&mut h, GET_STATE, &[]);
        assert_eq!(word(&a, 7), 10_000, "12 W at 12 V");
        assert_eq!(u16::from_le_bytes([a[16], a[17]]), 1 << 8);
        assert_eq!(ask(&mut h, SET_MODE, &[3])[3], OK);
        let a = ask(&mut h, GET_STATE, &[]);
        assert_eq!(word(&a, 7), 1_200, "12 V on 100 ohms");
        assert_eq!(u16::from_le_bytes([a[16], a[17]]), 1 << 9);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_setting_reads_back_and_one_out_of_range_is_refused() {
        for (set, get, good, bad) in [
            (SET_CC, GET_CC, 300_000, 300_001),
            (SET_CV, GET_CV, 100, 99),
            (SET_CV, GET_CV, 120_000, 120_001),
            (SET_CW, GET_CW, 300_000, 300_001),
            (SET_CR, GET_CR, 50, 49),
            (SET_CR, GET_CR, 7_500_000, 7_500_001),
            (SET_OVP, GET_OVP, 120_000, 120_001),
            (SET_OCP, GET_OCP, 300_000, 300_001),
        ] {
            let mut h = load(&[]);
            remote(&mut h);
            let a = ask(&mut h, set, &u32::to_le_bytes(good));
            assert_eq!(a[2..4], [STATUS, OK], "{set:02x}");
            assert_eq!(word(&ask(&mut h, get, &[]), 3), good);
            let a = ask(&mut h, set, &u32::to_le_bytes(bad));
            assert_eq!(a[2..4], [STATUS, BAD_PARAMETER], "{set:02x}");
            assert_eq!(word(&ask(&mut h, get, &[]), 3), good);
            assert_eq!(violations(&mut h), 0);
        }
    }

    #[test]
    fn a_byte_setting_out_of_range_is_refused() {
        for (command, bad) in [(REMOTE, 2), (LOAD_ON_OFF, 2), (SET_MODE, 4)] {
            let mut h = load(&[]);
            remote(&mut h);
            assert_eq!(ask(&mut h, command, &[bad])[3], BAD_PARAMETER);
        }
    }

    #[test]
    fn each_check_fires() {
        let mut no_preamble = cmd(0, GET_STATE, &[]);
        no_preamble[0] = 0x55;
        let mut bad_sum = cmd(0, GET_STATE, &[]);
        bad_sum[PACKET - 1] ^= 1;
        let mut h = load(&[]);
        h.write(&no_preamble);
        assert_eq!(h.read(), b"", "without its preamble it is no packet");
        assert_eq!(violations(&mut h), 1);
        h.write(&bad_sum);
        assert_eq!(h.read()[2..4], [STATUS, BAD_CHECKSUM]);
        assert_eq!(violations(&mut h), 1);
        // Unused bytes, of a read and of settings of one and of four bytes.
        for bad in [
            cmd(0, GET_STATE, &[0, 1]),
            cmd(0, REMOTE, &[1, 1]),
            cmd(0, GET_CC, &[1]),
            cmd(BROADCAST, GET_STATE, &[]),
            cmd(0, 0x03, &[]),
        ] {
            let mut h = load(&[]);
            h.write(&bad);
            assert_eq!(violations(&mut h), 1, "{}", show(&bad));
        }
        let mut h = load(&[]);
        remote(&mut h);
        h.write(&cmd(0, SET_CC, &[1, 0, 0, 0, 1]));
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_setting_outside_remote_mode_is_flagged() {
        let mut h = load(&[]);
        assert_eq!(ask(&mut h, LOAD_ON_OFF, &[1])[3], OK);
        assert_eq!(violations(&mut h), 1);
        remote(&mut h);
        assert_eq!(ask(&mut h, LOAD_ON_OFF, &[1])[3], OK);
        assert_eq!(violations(&mut h), 0);
        assert_eq!(ask(&mut h, REMOTE, &[0])[3], OK);
        assert_eq!(ask(&mut h, GET_CC, &[])[2], GET_CC);
        assert_eq!(violations(&mut h), 0, "a read needs no remote mode");
        assert_eq!(ask(&mut h, SET_CC, &[1])[3], OK);
        assert_eq!(violations(&mut h), 1);
    }

    #[test]
    fn a_remote_mode_that_failed_excuses_the_settings_after_it() {
        let mut h = load(&[("status-20", "c0")]);
        assert_eq!(ask(&mut h, REMOTE, &[1])[3], 0xc0);
        assert_eq!(ask(&mut h, LOAD_ON_OFF, &[1])[3], OK);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn status_replaces_the_answer_and_the_action() {
        let mut h = load(&[("status-2a", "a0"), ("status-2b", "c0")]);
        remote(&mut h);
        assert_eq!(ask(&mut h, SET_CC, &[1])[2..4], [STATUS, BAD_PARAMETER]);
        assert_eq!(h.cdc.model.now.cc, 10_000);
        assert_eq!(ask(&mut h, GET_CC, &[])[2..4], [STATUS, 0xc0]);
    }

    #[test]
    fn silent_acts_with_no_answer() {
        let mut h = load(&[("silent-20", "1")]);
        assert_eq!(ask(&mut h, REMOTE, &[1]), b"");
        assert!(h.cdc.model.remote);
    }

    #[test]
    fn state_and_replies_play_in_turn() {
        let mut h = load(&[("state", "01000000,02000000"), ("reply-2b", "aabb,cc")]);
        assert_eq!(word(&ask(&mut h, GET_STATE, &[]), 3), 1);
        assert_eq!(word(&ask(&mut h, GET_STATE, &[]), 3), 2);
        assert_eq!(word(&ask(&mut h, GET_STATE, &[]), 3), 1);
        assert_eq!(ask(&mut h, GET_CC, &[]), [0xaa, 0xbb]);
        assert_eq!(ask(&mut h, GET_CC, &[]), [0xcc]);
    }

    #[test]
    fn a_packet_may_come_in_pieces_or_together() {
        let mut h = load(&[]);
        let p = cmd(0, GET_STATE, &[]);
        h.write(&p[..5]);
        assert_eq!(h.read(), b"");
        h.write(&p[5..]);
        assert_eq!(h.read().len(), PACKET);
        let two: Vec<u8> = p.iter().chain(p.iter()).copied().collect();
        h.write(&two);
        assert_eq!(h.read().len(), 2 * PACKET);
        assert_eq!(violations(&mut h), 0);
    }

    #[test]
    fn a_long_write_keeps_the_buffer_short() {
        let mut h = load(&[]);
        h.write(&[0x55; 4096]);
        assert!(h.cdc.model.pending.len() < PACKET);
        assert_eq!(violations(&mut h), 4096 / PACKET);
    }

    #[test]
    fn a_packet_left_unfinished_shows_at_the_next_open() {
        let mut h = load(&[]);
        h.write(&[PREAMBLE, 0]);
        h.set_lines(false, false);
        assert_eq!(violations(&mut h), 0);
        h.set_lines(true, true);
        assert_eq!(violations(&mut h), 1);
        assert_eq!(h.cdc.model.pending, b"");
    }

    #[test]
    fn answers_runs_out_and_marks_the_point() {
        let mut h = load(&[("answers", "1")]);
        remote(&mut h);
        assert!(!h.cdc.mark());
        h.write(&cmd(0, SET_CC, &[1]));
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
        assert_eq!(h.cdc.model.now.cc, 1, "a silent one still takes a setting");
    }

    #[test]
    fn it_listens_at_its_line_only() {
        let mut h = load(&[]);
        h.set_line(Line::parse("38400/8n1").unwrap());
        assert_eq!(ask(&mut h, GET_STATE, &[]), b"");
        let mut h = load(&[("uart", "38400/8n1")]);
        h.set_line(Line::parse("38400/8n1").unwrap());
        assert_eq!(ask(&mut h, GET_STATE, &[]).len(), PACKET);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = load(&[("answers", "2"), ("reply-2b", "aa,bb")]);
        remote(&mut h);
        assert_eq!(ask(&mut h, GET_CC, &[]), [0xaa]);
        h.cdc.model.reset();
        assert!(!h.cdc.model.remote);
        assert_eq!(ask(&mut h, GET_CC, &[]), [0xaa], "the list and the count start over");
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("model", "123456"),
            ("serial", "12345678901"),
            ("barcode", "12345678901234567890123"),
            ("firmware", "1"),
            ("firmware", "1.23"),
            ("address", "255"),
            ("min-resistance", "65536"),
            ("load", "2"),
            ("mode", "4"),
            ("source", "10000001"),
            ("state", "abc"),
            ("status-2", "80"),
            ("status-2a", "8"),
            ("status-zz", "80"),
            ("silent-2a", "2"),
            ("reply-2a", "abc"),
            ("uart", "9600"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("model", "85112"), ("firmware", "01.23"), ("chunk", "1"), ("reply-5f", "aa")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
