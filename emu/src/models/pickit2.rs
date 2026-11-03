// Counterpart in sigrok:
//   libsigrok/src/hardware/microchip-pickit2/protocol.c (the commands,
//     their layout and order),
//   libsigrok/src/hardware/microchip-pickit2/protocol.h,
//   libsigrok/src/hardware/microchip-pickit2/api.c (the rates, the
//     capture ratio presets, the interface),
//   Copyright (C) 2018 Gerhard Sittig <gerhard.sittig@gmx.net>,
//   with commits by Uwe Hermann
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! The Microchip PICkit2 programmer in its logic analyzer mode, as the
//! Pico emulator plays it (`hil/pico-hidcable`, `MODE pickit2`) with the
//! protocol libsigrok's `microchip-pickit2` driver speaks: a HID device
//! `04d8:0033` whose interface 0 has a 64 byte interrupt IN and a 64
//! byte interrupt OUT endpoint, both number 1. Every command is one
//! 64 byte packet on the OUT endpoint, the command bytes first and the
//! rest padded with `0xad`; every answer is one 64 byte packet on the
//! IN endpoint.
//!
//! | command | bytes | answer |
//! |---|---|---|
//! | SETUP | `b8 01 EN LVL EDGE REP POSL POSH DIV` | a status, once the capture is done |
//! | SETPOS | `b9 OFFSET BANK` | none |
//! | READ | `ac` | 64 bytes of the sample memory |
//!
//! SETUP arms a capture of 1024 samples at `1 MHz / (DIV + 1)`. `EN`,
//! `LVL` and `EDGE` hold one bit per channel (`0x04` pin4, `0x08`
//! pin5, `0x10` pin6): enabled, high (level one, or a rising edge),
//! edge. `REP` is 1 with a trigger and 0 without, `POS` one of seven
//! magic words for the capture ratio presets. The status comes when
//! the capture is done, after a delay here (`delay=N` offers of the IN
//! transfer, 15 by default, which is 300 ms at the server's 20 ms poll,
//! as on the Pico, and longer than the C driver's 250 ms receive
//! timeout, so its timeout path is taken); its first two bytes are a
//! little endian status word, 0 for data. Then the host reads the 512
//! bytes of sample memory: SETPOS to bank 6 or 7 at offset 0 or
//! `0x80`, then two READs of 64 bytes each, four times. The memory
//! holds two samples per byte, newest first, the low nibble the older
//! sample, with pin4 in bit 2, pin5 in bit 3 and pin6 in bit 0 of a
//! nibble.
//!
//! The samples are a counter, as on the Pico: sample `k` (in time
//! order) is `k mod 8`, so pin4 toggles every sample, pin5 every two
//! and pin6 every four. The same 1024 samples come whatever the rate
//! and the trigger position, since the model has no clock; a trigger
//! the counter can satisfy completes the capture, one it never can
//! (rising edges on pin4 and pin5 at once: pin5 only rises when pin4
//! falls) never would, so the model then answers what a user pressing
//! the device's button gets, the status `0x4004` the C driver takes
//! for a cancel.
//!
//! The pedantic checks follow what the C driver sends and its
//! comments: every packet is 64 bytes, padded with `0xad`; the
//! command is one of the three; SETUP has `01` in its second byte,
//! trigger bits only for the three channels, level and edge bits only
//! for enabled channels, `REP` 1 exactly when a channel is enabled, a
//! known position word, a divider of one of the eight rates, and comes
//! while no capture runs and none waits to be read; SETPOS comes after
//! the status of a capture that was not cancelled, with the banks and
//! offsets in the C driver's order; READ comes at most twice after
//! each SETPOS, and never while the answer to the previous READ is
//! still unread; an IN transfer asks for 64 bytes and comes only while
//! a status or a READ answer is to come (a NAKed transfer is not
//! logged, so a stray read would otherwise go unseen). The HID class
//! requests a kernel sends (`SET_IDLE`, the report descriptor) are
//! answered; reports over the control pipe (`GET_REPORT`,
//! `SET_REPORT`) are not how either driver talks to the device and are
//! flagged.
//!
//! The descriptors are those of a PICkit2 (vendor and product strings,
//! the serial number `PIC18F2550` an unnamed unit reports, endpoint 0
//! of 8 bytes, the 1 ms interrupt endpoints) with the report
//! descriptor of Microchip's generic HID firmware, 64 byte input and
//! output reports without report ids; neither driver reads more than
//! the ids.

#![allow(clippy::doc_markdown)]

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{HidInterface, HidReports};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "pickit2";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "Microchip PICkit2 logic analyzer (HID, 04d8:0033), the Pico emulator's \
     role; delay=N (IN offers before the status, default 15)";

/// A device from the `-o` options: `delay=N`, the IN transfers the
/// status waits for, 0 to 1000, default 15.
///
/// # Errors
///
/// On an unknown option or a delay out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut delay = DEFAULT_DELAY;
    for (k, v) in options {
        match k.as_str() {
            "delay" => {
                delay = v
                    .parse()
                    .ok()
                    .filter(|n| *n <= 1000)
                    .ok_or_else(|| Error(format!("delay={v}: 0 to 1000")))?;
            }
            other => return Err(Error(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(Pickit2::new(delay)))
}

/// The USB vendor id.
pub const VENDOR_ID: u16 = 0x04d8;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0x0033;
/// The packet length of both endpoints.
pub const PACKET: usize = 64;
/// The sample memory: two samples per byte.
pub const RAW_LEN: usize = 512;
/// The IN offers before the status, by default.
pub const DEFAULT_DELAY: u32 = 15;

const CMD_READ: u8 = 0xac;
const CMD_PAD: u8 = 0xad;
const CMD_SETUP: u8 = 0xb8;
const CMD_SETPOS: u8 = 0xb9;
const SEL_BANK0: u8 = 0x06;
/// Trigger bits of pin4, pin5, pin6 in the SETUP packet.
const TRIGGER_MASKS: [u8; 3] = [0x04, 0x08, 0x10];
/// The position words of the capture ratio presets 0 to 6.
const POSITIONS: [u16; 7] = [0x0000, 0x03cc, 0x000a, 0x0248, 0x07b4, 0x0b9c, 0x0f84];
/// The sample rates the driver offers, in Hz.
const RATES: [u32; 8] = [5_000, 10_000, 25_000, 50_000, 100_000, 250_000, 500_000, 1_000_000];
/// The status word the C driver takes for a cancel: the button, and
/// the transfer timeout.
pub const STATUS_CANCEL: u16 = 0x4004;
/// SETPOS arguments in the order the C driver reads the memory.
const READ_ORDER: [(u8, u8); 4] =
    [(0x00, SEL_BANK0), (0x80, SEL_BANK0), (0x00, SEL_BANK0 + 1), (0x80, SEL_BANK0 + 1)];

/// Microchip's generic HID report descriptor: vendor page, 64 byte
/// input and output reports, no report ids.
pub const REPORT_DESCRIPTOR: [u8; 29] = [
    0x06, 0x00, 0xff, // usage page, vendor defined
    0x09, 0x01, // usage 1
    0xa1, 0x01, // collection, application
    0x19, 0x01, 0x29, 0x40, // usage 1 to 64
    0x15, 0x00, 0x26, 0xff, 0x00, // logical 0 to 255
    0x75, 0x08, 0x95, 0x40, // 64 fields of 8 bits
    0x81, 0x00, // input
    0x19, 0x01, 0x29, 0x40, // usage 1 to 64
    0x91, 0x00, // output
    0xc0, // end collection
];

/// One sample nibble of the counter value `k`.
fn nibble(k: usize) -> u8 {
    let mut n = 0;
    if k & 1 != 0 {
        n |= 0x04; // pin4
    }
    if k & 2 != 0 {
        n |= 0x08; // pin5
    }
    if k & 4 != 0 {
        n |= 0x01; // pin6
    }
    n
}

/// The sample memory of a capture of the counter: newest first, the
/// Pico's `pk_fill`.
pub fn counter_memory() -> [u8; RAW_LEN] {
    let mut raw = [0u8; RAW_LEN];
    for (i, b) in raw.iter_mut().enumerate() {
        let j = RAW_LEN - 1 - i;
        *b = nibble(2 * j) | nibble(2 * j + 1) << 4;
    }
    raw
}

/// Whether the trigger of a SETUP packet fires on the counter: some
/// sample where every enabled channel has its level (and, for an edge,
/// had the other level the sample before).
pub fn counter_triggers(en: u8, lvl: u8, edge: u8) -> bool {
    if en == 0 {
        return true;
    }
    let pin = |k: usize, ch: usize| (k >> ch) & 1 == 1;
    (1..1024).any(|k| {
        (0..3).all(|ch| {
            let m = TRIGGER_MASKS[ch];
            if en & m == 0 {
                return true;
            }
            let high = lvl & m != 0;
            pin(k, ch) == high && (edge & m == 0 || pin(k - 1, ch) != high)
        })
    })
}

/// Where the capture protocol is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// No capture armed yet, or the last one read in full.
    Idle,
    /// SETUP received: the status comes after this many more IN offers.
    Armed { offers: u32, cancel: bool },
    /// The status went out; SETPOS `step` of [`READ_ORDER`] is next,
    /// `reads` READs done after the previous SETPOS.
    Reading { step: usize, reads: u8 },
    /// The status said cancel; nothing to read.
    Cancelled,
}

/// The analyzer behind the endpoints.
#[derive(Clone, Debug)]
pub struct Analyzer {
    delay: u32,
    raw: [u8; RAW_LEN],
    state: State,
    /// The SETPOS address the READs continue from: bank, offset.
    pos: (u8, u8),
    /// READs since the last SETPOS, as the firmware counts them.
    reads: usize,
    /// The packet the next IN transfer returns.
    answer: Option<[u8; PACKET]>,
    /// SETUP packets taken.
    pub captures: usize,
}

impl Analyzer {
    /// An idle analyzer whose status waits `delay` IN offers.
    pub fn new(delay: u32) -> Analyzer {
        Analyzer {
            delay,
            raw: counter_memory(),
            state: State::Idle,
            pos: (SEL_BANK0, 0),
            reads: 0,
            answer: None,
            captures: 0,
        }
    }

    /// Checks the padding behind `used` command bytes.
    fn check_padding(name: &str, packet: &[u8], used: usize, v: &mut Violations) {
        if let Some(i) = packet.iter().skip(used).position(|&b| b != CMD_PAD) {
            v.report(format!(
                "{name} byte {} is {:#04x}, the padding is {CMD_PAD:#04x}",
                used + i,
                packet[used + i]
            ));
        }
    }

    /// One packet on the OUT endpoint.
    pub fn command(&mut self, packet: &[u8], v: &mut Violations) {
        if packet.len() != PACKET {
            v.report(format!("OUT packet of {} bytes, the device takes {PACKET}", packet.len()));
        }
        let Some(&cmd) = packet.first() else {
            return;
        };
        match cmd {
            CMD_SETUP => self.setup(packet, v),
            CMD_SETPOS => self.setpos(packet, v),
            CMD_READ => self.read(packet, v),
            other => v.report(format!(
                "command {other:#04x}: the driver sends SETUP b8, SETPOS b9 and READ ac only"
            )),
        }
    }

    fn setup(&mut self, p: &[u8], v: &mut Violations) {
        Self::check_padding("SETUP", p, 9, v);
        let b = |i: usize| p.get(i).copied().unwrap_or(CMD_PAD);
        match self.state {
            State::Idle | State::Cancelled => {}
            State::Armed { .. } => v.report("SETUP while a capture runs"),
            State::Reading { step, reads } => v.report(format!(
                "SETUP before the previous capture was read: SETPOS {step} of 4, {reads} READs"
            )),
        }
        if self.answer.is_some() {
            v.report("SETUP while an answer is unread");
        }
        if b(1) != 0x01 {
            v.report(format!("SETUP byte 1 is {:#04x}, the driver sends 01", b(1)));
        }
        let (en, lvl, edge, rep) = (b(2), b(3), b(4), b(5));
        if en & !0x1c != 0 {
            v.report(format!("SETUP trigger enable {en:#04x} has bits beyond the three channels"));
        }
        if lvl & !en != 0 || edge & !en != 0 {
            v.report(format!(
                "SETUP level {lvl:#04x} or edge {edge:#04x} has bits of channels not enabled \
                 ({en:#04x})"
            ));
        }
        let want_rep = u8::from(en != 0);
        if rep != want_rep {
            v.report(format!(
                "SETUP repeat count {rep} with trigger enable {en:#04x}; the driver sends {want_rep}"
            ));
        }
        let pos = u16::from_le_bytes([b(6), b(7)]);
        if !POSITIONS.contains(&pos) {
            v.report(format!("SETUP trigger position {pos:#06x} is not one of the presets"));
        }
        let div = b(8);
        if !RATES.iter().any(|r| 1_000_000 / r - 1 == u32::from(div)) {
            v.report(format!("SETUP divider {div} gives none of the driver's rates"));
        }
        self.captures += 1;
        self.answer = None;
        self.state = State::Armed { offers: self.delay, cancel: !counter_triggers(en, lvl, edge) };
    }

    fn setpos(&mut self, p: &[u8], v: &mut Violations) {
        Self::check_padding("SETPOS", p, 3, v);
        let offset = p.get(1).copied().unwrap_or(0);
        let bank = p.get(2).copied().unwrap_or(SEL_BANK0);
        match self.state {
            State::Idle => v.report("SETPOS with no capture to read"),
            State::Armed { .. } => v.report("SETPOS before the capture's status was read"),
            State::Cancelled => v.report("SETPOS after a cancelled capture"),
            State::Reading { step, reads } => {
                if step > 0 && reads != 2 {
                    v.report(format!("SETPOS after {reads} READs; the driver reads two"));
                }
                match READ_ORDER.get(step) {
                    Some(&(o, b)) if (o, b) == (offset, bank) => {}
                    Some(&(o, b)) => v.report(format!(
                        "SETPOS offset {offset:#04x} bank {bank:#04x}; the driver's step {step} is \
                         offset {o:#04x} bank {b:#04x}"
                    )),
                    None => v.report("a fifth SETPOS; the memory is read in four"),
                }
                self.state = State::Reading { step: step + 1, reads: 0 };
            }
        }
        if bank != SEL_BANK0 && bank != SEL_BANK0 + 1 {
            v.report(format!("SETPOS bank {bank:#04x} is neither 06 nor 07"));
        }
        self.pos = (bank, offset);
        self.reads = 0;
    }

    fn read(&mut self, p: &[u8], v: &mut Violations) {
        Self::check_padding("READ", p, 1, v);
        match self.state {
            State::Reading { step, reads } if step > 0 => {
                if reads >= 2 {
                    v.report(format!("READ {} after one SETPOS; the driver reads two", reads + 1));
                }
                let reads = reads.saturating_add(1);
                self.state = if step == READ_ORDER.len() && reads == 2 {
                    State::Idle
                } else {
                    State::Reading { step, reads }
                };
            }
            State::Reading { .. } => v.report("READ before SETPOS"),
            State::Idle => v.report("READ with no capture to read"),
            State::Armed { .. } => v.report("READ before the capture's status was read"),
            State::Cancelled => v.report("READ after a cancelled capture"),
        }
        if self.answer.is_some() {
            v.report("READ while the previous answer is unread");
        }
        // The firmware's address: bank, offset, 64 bytes per READ.
        let at = usize::from(self.pos.0 & 1) * 256 + usize::from(self.pos.1) + self.reads * PACKET;
        self.reads += 1;
        let mut a = [0u8; PACKET];
        for (i, b) in a.iter_mut().enumerate() {
            *b = self.raw[(at + i) % RAW_LEN];
        }
        self.answer = Some(a);
    }

    /// The state, for a message.
    fn state_name(&self) -> &'static str {
        match self.state {
            State::Idle => "no capture",
            State::Armed { .. } => "a capture runs",
            State::Reading { reads: 0, .. } => "after SETUP's status or a SETPOS",
            State::Reading { .. } => "after a READ was answered",
            State::Cancelled => "after a cancelled capture",
        }
    }

    /// One offer of an IN transfer of up to `max` bytes.
    pub fn poll(&mut self, max: usize, v: &mut Violations) -> Reply {
        if max != PACKET {
            v.report(format!("IN transfer of {max} bytes, the device sends {PACKET}"));
        }
        if let State::Armed { offers, cancel } = self.state {
            if offers > 0 {
                self.state = State::Armed { offers: offers - 1, cancel };
                return Reply::Nak;
            }
            let mut status = [0u8; PACKET];
            if cancel {
                status[..2].copy_from_slice(&STATUS_CANCEL.to_le_bytes());
                self.state = State::Cancelled;
            } else {
                self.state = State::Reading { step: 0, reads: 0 };
            }
            return Reply::Data(status.to_vec());
        }
        if let Some(a) = self.answer.take() {
            return Reply::Data(a.to_vec());
        }
        // Neither driver reads without a status or a READ
        // answer to come; a NAKed read is not logged as a
        // transfer, so it is flagged here.
        v.report(format!(
            "IN transfer with no answer to come ({}); the driver reads after SETUP \
             and after each READ only",
            self.state_name()
        ));
        Reply::Nak
    }
}

impl HidReports for Analyzer {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        v.report(format!(
            "GET_REPORT kind {kind} id {id} of {length} bytes; the device answers on endpoint 1"
        ));
        Reply::Stall
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!(
            "SET_REPORT kind {kind} id {id} of {} bytes; commands go to endpoint 1",
            data.len()
        ));
        Reply::Stall
    }
}

/// The emulated PICkit2.
pub struct Pickit2 {
    descriptors: Descriptors,
    hid: HidInterface,
    analyzer: Analyzer,
}

impl Pickit2 {
    /// A PICkit2 whose status waits `delay` IN offers.
    pub fn new(delay: u32) -> Pickit2 {
        let hid = HidInterface::new(0, REPORT_DESCRIPTOR.to_vec());
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [3, 0, 0], 0));
        body.extend_from_slice(&hid.descriptor());
        body.extend_from_slice(&descriptor::endpoint(0x81, 3, 64, 1));
        body.extend_from_slice(&descriptor::endpoint(0x01, 3, 64, 1));
        let config = descriptor::configuration(1, 0, 0x80, 100, &body);
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 8,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x0002,
            strings: [1, 2, 3],
            configurations: 1,
        };
        Pickit2 {
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![config],
                strings: vec![
                    "Microchip Technology Inc.".to_owned(),
                    "PICkit 2 Microcontroller Programmer".to_owned(),
                    "PIC18F2550".to_owned(),
                ],
            },
            hid,
            analyzer: Analyzer::new(delay),
        }
    }
}

impl DeviceModel for Pickit2 {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if let Some(reply) = self.hid.control(&mut self.analyzer, setup, data, v) {
            return reply;
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the PICkit2 answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != 1 {
            v.report(format!("OUT transfer to endpoint {ep}; commands go to endpoint 1"));
            return Reply::Stall;
        }
        self.analyzer.command(data, v);
        Reply::ack()
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != 1 {
            v.report(format!("IN transfer from endpoint {ep}; answers come on endpoint 1"));
            return Reply::Stall;
        }
        self.analyzer.poll(max, v)
    }

    fn reset(&mut self) {
        // A reset ends whatever capture was armed or half read; the
        // counter in memory stays the same.
        self.hid.reset();
        self.analyzer = Analyzer::new(self.analyzer.delay);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(bytes: &[u8]) -> Vec<u8> {
        let mut p = vec![CMD_PAD; PACKET];
        p[..bytes.len()].copy_from_slice(bytes);
        p
    }

    /// The C driver's capture: SETUP, the status, four SETPOS with two
    /// READs each. Returns the 512 bytes read.
    fn capture(dev: &mut Pickit2, setup: &[u8], v: &mut Violations) -> (u16, Vec<u8>) {
        assert_eq!(dev.out(1, &packet(setup), v), Reply::ack());
        let status = loop {
            match dev.in_transfer(1, PACKET, v) {
                Reply::Nak => {}
                Reply::Data(d) => break u16::from_le_bytes([d[0], d[1]]),
                Reply::Stall => panic!("stall"),
            }
        };
        let mut raw = Vec::new();
        if status == STATUS_CANCEL {
            return (status, raw);
        }
        for (offset, bank) in READ_ORDER {
            assert_eq!(dev.out(1, &packet(&[CMD_SETPOS, offset, bank]), v), Reply::ack());
            for _ in 0..2 {
                dev.out(1, &packet(&[CMD_READ]), v);
                let Reply::Data(d) = dev.in_transfer(1, PACKET, v) else { panic!() };
                raw.extend(d);
            }
        }
        (status, raw)
    }

    /// The C driver's conversion, for the check.
    fn samples(raw: &[u8]) -> Vec<u8> {
        let conv = |n: u8| (n >> 2 & 1) | (n >> 3 & 1) << 1 | (n & 1) << 2;
        raw.iter().rev().flat_map(|b| [conv(b & 0x0f), conv(b >> 4)]).collect()
    }

    #[test]
    fn captures_the_counter_as_the_driver_reads_it() {
        let mut dev = Pickit2::new(3);
        let mut v = Violations::default();
        // 1 MHz, no trigger.
        let (status, raw) = capture(&mut dev, &[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(status, 0);
        assert_eq!(raw.len(), RAW_LEN);
        let s = samples(&raw);
        assert_eq!(s, (0..1024).map(|k| (k & 7) as u8).collect::<Vec<_>>());
        // A second capture: rising pin5, preset 2, 250 kHz.
        let (status, raw) =
            capture(&mut dev, &[CMD_SETUP, 1, 0x08, 0x08, 0x08, 1, 0x0a, 0, 3], &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!((status, raw.len()), (0, RAW_LEN));
        assert_eq!(dev.analyzer.captures, 2);
        // Rising pin4 and pin5 at once never comes: the button.
        let (status, raw) =
            capture(&mut dev, &[CMD_SETUP, 1, 0x0c, 0x0c, 0x0c, 1, 0, 0, 0], &mut v);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!((status, raw.len()), (STATUS_CANCEL, 0));
        // The report descriptor is served.
        let s = Setup { request_type: 0x81, request: 6, value: 0x2200, index: 0, length: 255 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Data(REPORT_DESCRIPTOR.to_vec()));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn the_status_waits_for_the_delay() {
        let mut dev = Pickit2::new(2);
        let mut v = Violations::default();
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak, "nothing armed");
        assert_eq!(v.drain().len(), 1, "an IN with nothing to come is flagged");
        dev.out(1, &packet(&[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0]), &mut v);
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak);
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak);
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Data(vec![0; PACKET]));
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn triggers_on_the_counter() {
        assert!(counter_triggers(0, 0, 0));
        assert!(counter_triggers(0x1c, 0x1c, 0), "all high");
        assert!(counter_triggers(0x1c, 0, 0), "all low");
        assert!(counter_triggers(0x04, 0x04, 0x04), "rising pin4");
        assert!(counter_triggers(0x10, 0, 0x10), "falling pin6");
        assert!(counter_triggers(0x0c, 0x08, 0x0c), "rising pin5, falling pin4");
        assert!(!counter_triggers(0x0c, 0x0c, 0x0c), "rising pin4 and pin5");
        assert!(!counter_triggers(0x0c, 0x04, 0x0c), "rising pin4, falling pin5");
    }

    #[test]
    fn flags_what_the_protocol_forbids() {
        let mut v = Violations::default();
        let mut dev = Pickit2::new(0);
        dev.out(1, &[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "short packet");
        assert_eq!(dev.in_transfer(1, 8, &mut v), Reply::Data(vec![0; PACKET]));
        assert_eq!(v.drain().len(), 1, "short IN");

        let mut dev = Pickit2::new(0);
        dev.out(1, &packet(&[0xa2]), &mut v);
        assert_eq!(v.drain().len(), 1, "CHKSTAT is not the driver's");
        dev.out(1, &packet(&[CMD_READ]), &mut v);
        assert_eq!(v.drain().len(), 1, "READ with nothing captured");
        dev.in_transfer(1, PACKET, &mut v);
        dev.out(1, &packet(&[CMD_SETPOS, 0, 6]), &mut v);
        assert_eq!(v.drain().len(), 1, "SETPOS with nothing captured");

        let setups: [(&[u8], &str); 8] = [
            (&[CMD_SETUP, 2, 0, 0, 0, 0, 0, 0, 0], "byte 1"),
            (&[CMD_SETUP, 1, 0x21, 0, 0, 1, 0, 0, 0], "enable bits"),
            (&[CMD_SETUP, 1, 0x04, 0x08, 0, 1, 0, 0, 0], "level of a disabled channel"),
            (&[CMD_SETUP, 1, 0x04, 0, 0, 0, 0, 0, 0], "repeat 0 with a trigger"),
            (&[CMD_SETUP, 1, 0, 0, 0, 1, 0, 0, 0], "repeat 1 without one"),
            (&[CMD_SETUP, 1, 0, 0, 0, 0, 0x32, 0, 0], "position"),
            (&[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 2], "divider"),
            (&[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0, 0], "padding"),
        ];
        for (setup, what) in setups {
            let mut dev = Pickit2::new(0);
            dev.out(1, &packet(setup), &mut v);
            assert_eq!(v.drain().len(), 1, "{what}");
        }

        let mut dev = Pickit2::new(1);
        let setup = packet(&[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0]);
        dev.out(1, &setup, &mut v);
        dev.out(1, &setup, &mut v);
        assert_eq!(v.drain().len(), 1, "SETUP while armed");
        dev.out(1, &packet(&[CMD_SETPOS, 0, 6]), &mut v);
        assert_eq!(v.drain().len(), 1, "SETPOS before the status");
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak);
        assert!(matches!(dev.in_transfer(1, PACKET, &mut v), Reply::Data(_)));
        dev.out(1, &packet(&[CMD_READ]), &mut v);
        assert_eq!(v.drain().len(), 1, "READ before SETPOS");
        dev.in_transfer(1, PACKET, &mut v);
        dev.out(1, &packet(&[CMD_SETPOS, 0x80, 6]), &mut v);
        assert_eq!(v.drain().len(), 1, "out of order");
        dev.out(1, &packet(&[CMD_READ]), &mut v);
        dev.out(1, &packet(&[CMD_READ]), &mut v);
        assert_eq!(v.drain().len(), 1, "READ with an answer unread");
        dev.in_transfer(1, PACKET, &mut v);
        dev.out(1, &packet(&[CMD_READ]), &mut v);
        assert_eq!(v.drain().len(), 1, "a third READ");
        dev.in_transfer(1, PACKET, &mut v);
        dev.out(1, &packet(&[CMD_SETPOS, 0, 8]), &mut v);
        assert_eq!(v.drain().len(), 3, "after three READs, out of order, bank 8");
        dev.out(1, &setup, &mut v);
        assert_eq!(v.drain().len(), 1, "SETUP before the read-out ended");

        let mut dev = Pickit2::new(0);
        dev.out(1, &packet(&[CMD_SETUP, 1, 0x0c, 0x0c, 0x0c, 1, 0, 0, 0]), &mut v);
        dev.in_transfer(1, PACKET, &mut v);
        dev.out(1, &packet(&[CMD_SETPOS, 0, 6]), &mut v);
        assert_eq!(v.drain().len(), 1, "SETPOS after a cancel");

        let feature = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 };
        assert_eq!(dev.control(feature, &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "GET_REPORT");
        let output = Setup { request_type: 0x21, request: 9, value: 0x0200, index: 0, length: 64 };
        assert_eq!(dev.control(output, &[0; 64], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "SET_REPORT");
        assert_eq!(dev.out(2, &packet(&[CMD_READ]), &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1, "endpoint 2");

        // An IN after SETPOS, after a READ was answered, after a cancel.
        let mut dev = Pickit2::new(0);
        dev.out(1, &packet(&[CMD_SETUP, 1, 0, 0, 0, 0, 0, 0, 0]), &mut v);
        assert!(matches!(dev.in_transfer(1, PACKET, &mut v), Reply::Data(_)));
        dev.out(1, &packet(&[CMD_SETPOS, 0, 6]), &mut v);
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak);
        assert_eq!(v.drain().len(), 1, "IN after SETPOS");
        dev.out(1, &packet(&[CMD_READ]), &mut v);
        assert!(matches!(dev.in_transfer(1, PACKET, &mut v), Reply::Data(_)));
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak);
        assert_eq!(v.drain().len(), 1, "a second IN for one READ");
        let mut dev = Pickit2::new(0);
        dev.out(1, &packet(&[CMD_SETUP, 1, 0x0c, 0x0c, 0x0c, 1, 0, 0, 0]), &mut v);
        dev.in_transfer(1, PACKET, &mut v);
        assert_eq!(dev.in_transfer(1, PACKET, &mut v), Reply::Nak);
        assert_eq!(v.drain().len(), 1, "IN after a cancel");
    }
}
