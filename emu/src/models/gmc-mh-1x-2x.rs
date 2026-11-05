// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Gossen Metrawatt `METRAHit` multimeter behind its infrared adapter, on the
//! CDC-ACM shell. One meter in either of its two ways of talking:
//!
//! - `mode=send`: it sends on its own and takes nothing, at 9600/6n1 unless
//!   `uart=` says otherwise (`uart=8228/6n1` is the older 1x meters' adapter).
//! - `mode=bidi` (the default): it answers a request, at 9600/8n1.
//!
//! The device, and how sure this is:
//!
//! - "Interface Protocol for `METRAHit` 22-29S/M Multimeters", Gossen-Metrawatt,
//!   2000-10-02 (the copy on mikrocontroller.net, attachment 22868), gives the
//!   send mode's blocks and their bits 5 and 4, device codes, the commands and
//!   answers of the 14 byte blocks, `aa 27H`, the complement checksum, the
//!   address byte, the error block with 0 as its second byte, the function and
//!   range tables, that 1010 is OL, 1101 a broken fuse and 1110 OPEN, and that
//!   D3 of a range byte is the sign (0 is plus). It covers the 22 to 29S; the 12
//!   to 18S are named only by their device codes.
//! - That each byte of a request goes on the wire as three, a nibble doubled for
//!   each two bits: its first page shows that as a figure, of which only the bit
//!   names could be read; what a driver written against real units sends agrees.
//! - That a meter that is off wakes on a write and does not answer that write:
//!   a guess from that driver writing three times before it asks (`asleep=1`).
//! - That a request to an address of 0 is answered by all meters: the document.
//!
//! Emulation artifacts:
//!
//! - The USB side is the shell's CDC-ACM with a test id; only the byte stream
//!   and the line settings are the adapter's. A misset line passes nothing,
//!   where a real one passes garbage.
//! - It answers at once, unless `late=` holds an answer back.
//! - A send mode meter sends a block per `period` offers of a read (about 20 ms
//!   each), counted from each open so that a tool that closes within one has
//!   caused what it read. Its rate is set in its menu, 50 ms at best.
//! - The first block since power on comes after `lead`, for a scan that has a
//!   second to find it, and the next not before another open. A real meter goes on
//!   sending, but offers go on while a tty closes, for longer than a period here,
//!   and a tool that keeps its port would see what one that closes it does not.
//! - `msgs=`, `status=`, `meas=` and `frames=` are stand-ins for what a meter
//!   measures, and for a meter or a line gone wrong.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `mode=send|bidi`, `uart=LINE`;
//! - send mode: `msgs=HEX[,HEX...]`, blocks in turn and then again; `period=N`,
//!   `lead=N` offers; `burst=N` blocks per open, or `blocks=N` per power on, then
//!   silence and the event point a period later;
//! - bidirectional: `addr=N`, its address, up to 63 to reach what no meter says;
//!   `second=N`, another meter that answers requests to all; `status=HEX,...` and
//!   `meas=HEX,...`, the nine bytes after the command of each answer, in turn;
//!   `frames=HEX|-,...`, the bytes of each answer in turn, `-` being the usual,
//!   which may be several answers or part of one; `high=HEX` set in every byte
//!   sent (the document leaves bits 7 and 6 undefined); `answers=N`, requests
//!   answered per power on, then silence and the event point; `miss=N`, the Nth
//!   request goes unanswered; `late=N:T`, the Nth answer sent after T offers;
//!   `cut=N:K`, the Nth answer short by its last K bytes; `asleep=1`, off at
//!   power on.
//!
//! The pedantic checks: a send mode meter takes nothing. A bidirectional one takes
//! whole blocks of 42 bytes each of which is 00, 0f, f0 or ff, the header `2b
//! 3f`, a good checksum, its address or 0, and only the commands a driver has for
//! it: 3 and 8 with no parameters, and 6 with 5 and 5. What it would refuse gets
//! the document's error block.

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{Packets, parse_packets};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "gmc-mh-1x-2x";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Gossen Metrawatt METRAHit on its adapter; mode=send|bidi, uart=, msgs=, \
                         period=, lead=, burst=, blocks=, addr=, second=, status=, meas=, frames=, high=, \
                         answers=, miss=, late=, cut=, asleep=, and the shell's stall=, chunk=, \
                         in-limit=, in-stall=, out-hold=";

/// A request on the wire.
const WIRE_BYTES: usize = 42;
/// A request or an answer.
const BLOCK_BYTES: usize = 14;
/// The bytes after an answer's command.
const PAYLOAD_BYTES: usize = 9;
/// The longest block `msgs=` takes: several of a meter's, sent as one.
const MSGS_MAX: usize = 1024;
/// How many answers' bytes one `frames=` entry may hold.
const FRAMES_MAX: usize = 32;

const CMD_STATUS: u8 = 3;
const CMD_MODE: u8 = 6;
const CMD_MEASURE: u8 = 8;
/// The document's error codes.
const ERR_COMMAND: u8 = 1;
const ERR_CHECKSUM: u8 = 2;
const ERR_HEADER: u8 = 4;
const ERR_PARAMETER: u8 = 5;

/// Firmware 1.2, rotary switch 3, V DC, auto 3 V, 3.0 V of battery, a 26S.
const STATUS: &str = "020103010100001e01";
/// 1.23456 V DC in the 3 V range, valid.
const MEASURE: &str = "000111060504030201";
/// A 26S/M at 1.23456 V DC, its 13 byte block.
const SEND_BLOCK: &str = "01313030313635343332313033";

/// The meter of the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

fn hex(key: &str, value: &str, min: usize, max: usize) -> Result<Vec<Vec<u8>>> {
    parse_packets(value, min, max)
        .map_err(|e| Error::Invalid(format!("{key}: {e}")))
}

/// `N:M`, for `late=` and `cut=`.
fn pair(key: &str, value: &str, second: std::ops::RangeInclusive<u32>) -> Result<(u32, u32)> {
    let (n, m) =
        value.split_once(':').ok_or_else(|| Error::Invalid(format!("{key}={value}: as in 2:40")))?;
    Ok((parse_number(key, n, 1..=100_000)?, parse_number(key, m, second)?))
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut bidi = true;
    let mut uart = None;
    let mut c = Config::default();
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "mode" => {
                bidi = match v.as_str() {
                    "send" => false,
                    "bidi" => true,
                    _ => return Err(Error::Invalid(format!("mode={v}: send or bidi"))),
                };
            }
            "uart" => uart = Some(Line::parse(v)?),
            "msgs" => c.msgs = Packets::new(hex(k, v, 1, MSGS_MAX)?),
            "period" => c.period = parse_number(k, v, 2..=1000)?,
            "lead" => c.lead = parse_number(k, v, 1..=1000)?,
            "burst" => c.burst = Some(parse_number(k, v, 0..=1000)?),
            "blocks" => c.blocks = Some(parse_number(k, v, 0..=1000)?),
            "addr" => c.addr = parse_number(k, v, 1..=63)? as u8,
            "second" => c.second = Some(parse_number(k, v, 1..=63)? as u8),
            "status" => c.status = Packets::new(hex(k, v, PAYLOAD_BYTES, PAYLOAD_BYTES)?),
            "meas" => c.meas = Packets::new(hex(k, v, PAYLOAD_BYTES, PAYLOAD_BYTES)?),
            "frames" => {
                let mut list = Vec::new();
                for f in v.split(',') {
                    list.push(if f == "-" {
                        Vec::new()
                    } else {
                        hex(k, f, 1, BLOCK_BYTES * FRAMES_MAX)?.remove(0)
                    });
                }
                c.frames = Some(Packets::new(list));
            }
            "high" => {
                let b = hex(k, v, 1, 1)?.remove(0)[0];
                if b & 0x3f != 0 {
                    return Err(Error::Invalid(format!("high={v}: bits 7 and 6 only")));
                }
                c.high = b;
            }
            "answers" => c.answers = Some(parse_number(k, v, 0..=100_000)?),
            "miss" => c.miss = Some(parse_number(k, v, 1..=100_000)?),
            "late" => c.late = Some(pair(k, v, 1..=100_000)?),
            "cut" => c.cut = Some(pair(k, v, 1..=13)?),
            "asleep" => c.awake = parse_number(k, v, 0..=1)? == 0,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let uart = match uart {
        Some(u) => u,
        None => Line::parse(if bidi { "9600/8n1" } else { "9600/6n1" })?,
    };
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "METRAHit adapter");
    let meter = Meter::new(c, bidi, uart);
    Ok(Cdc::new(NAME, &id, faults, meter).behind_uart(uart))
}

/// The block a request's or answer's checksum closes: the complement of the sum
/// of the 13 bytes before it, in six bits.
fn checksum(block: &[u8]) -> u8 {
    let sum = block[..BLOCK_BYTES - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    64u8.wrapping_sub(sum) & 0x3f
}

/// A request's 14 bytes from its 42: 00, 0f, f0 or ff for each two bits. `None`
/// for a byte that is none of them.
fn decode_wire(wire: &[u8]) -> Option<[u8; BLOCK_BYTES]> {
    let two = |b: u8| match b {
        0x00 => Some(0u8),
        0x0f => Some(1),
        0xf0 => Some(2),
        0xff => Some(3),
        _ => None,
    };
    let mut block = [0; BLOCK_BYTES];
    for (out, w) in block.iter_mut().zip(wire.as_chunks::<3>().0) {
        *out = two(w[0])? | (two(w[1])? << 2) | (two(w[2])? << 4);
    }
    Some(block)
}

#[derive(Clone, Debug)]
struct Config {
    msgs: Packets,
    period: u32,
    lead: u32,
    burst: Option<u32>,
    blocks: Option<u32>,
    addr: u8,
    second: Option<u8>,
    status: Packets,
    meas: Packets,
    frames: Option<Packets>,
    high: u8,
    answers: Option<u32>,
    miss: Option<u32>,
    late: Option<(u32, u32)>,
    cut: Option<(u32, u32)>,
    awake: bool,
}

impl Default for Config {
    fn default() -> Config {
        let list = |s: &str| Packets::new(parse_packets(s, 1, MSGS_MAX).expect("the defaults parse"));
        Config {
            msgs: list(SEND_BLOCK),
            period: 25,
            lead: 5,
            burst: None,
            blocks: None,
            addr: 1,
            second: None,
            status: list(STATUS),
            meas: list(MEASURE),
            frames: None,
            high: 0,
            answers: None,
            miss: None,
            late: None,
            cut: None,
            awake: true,
        }
    }
}

struct Meter {
    power_on: Config,
    c: Config,
    bidi: bool,
    uart: Line,
    /// What has come of a request.
    wire: Vec<u8>,
    /// Offers since the last block, and whether one went since the open.
    polls: u32,
    started: bool,
    sent: u32,
    /// Blocks since power on.
    total: u32,
    /// An open has come since the first block.
    reopened: bool,
    quiet: u32,
    answered: u32,
    /// An answer held back: the offers it still waits, and its bytes.
    held: Option<(u32, Vec<u8>)>,
}

impl Meter {
    fn new(c: Config, bidi: bool, uart: Line) -> Meter {
        Meter {
            power_on: c.clone(),
            c,
            bidi,
            uart,
            wire: Vec::new(),
            polls: 0,
            started: false,
            sent: 0,
            total: 0,
            reopened: false,
            quiet: 0,
            answered: 0,
            held: None,
        }
    }

    fn line_fits(&self, line: Line) -> bool {
        line.baud == self.uart.baud
            && line.data_bits == self.uart.data_bits
            && (line.parity == 0) == (self.uart.parity == 0)
    }

    /// An answer: address, `27 3f`, the command, nine bytes, the checksum.
    fn block(addr: u8, command: u8, payload: &[u8]) -> Vec<u8> {
        let mut b = vec![addr, 0x27, 0x3f, command];
        b.extend_from_slice(payload);
        b.push(0);
        b[BLOCK_BYTES - 1] = checksum(&b);
        b
    }

    /// The document's error block.
    fn error(&self, code: u8) -> Vec<u8> {
        let mut b = vec![self.c.addr, 0, code];
        b.resize(BLOCK_BYTES, 0);
        b[BLOCK_BYTES - 1] = checksum(&b);
        b
    }

    fn usual(&mut self, command: u8) -> Vec<u8> {
        match command {
            CMD_STATUS => Self::block(self.c.addr, command, &self.c.status.take()),
            CMD_MEASURE => Self::block(self.c.addr, command, &self.c.meas.take()),
            _ => Self::block(self.c.addr, command, &[0; PAYLOAD_BYTES]),
        }
    }

    /// What to answer a request, or a violation and the error block.
    fn check(
        &self,
        block: &[u8; BLOCK_BYTES],
        port: &mut Port<'_>,
    ) -> std::result::Result<(), Vec<u8>> {
        if checksum(block) != block[BLOCK_BYTES - 1] {
            port.violation(format!("request with a bad checksum: {block:02x?}"));
            return Err(self.error(ERR_CHECKSUM));
        }
        if block[1] != 0x2b || block[2] != 0x3f {
            port.violation(format!("request with the header {:02x} {:02x}", block[1], block[2]));
            return Err(self.error(ERR_HEADER));
        }
        let params = &block[4..BLOCK_BYTES - 1];
        match block[3] {
            CMD_STATUS | CMD_MEASURE if params.iter().any(|p| *p != 0) => {
                port.violation(format!("command {} with parameters {params:02x?}", block[3]));
                Err(self.error(ERR_PARAMETER))
            }
            CMD_MODE if params[..2] != [5, 5] || params[2..].iter().any(|p| *p != 0) => {
                port.violation(format!("command 6 with {params:02x?}; 5 and 5 is switching off"));
                Err(self.error(ERR_PARAMETER))
            }
            CMD_STATUS | CMD_MEASURE | CMD_MODE => Ok(()),
            other => {
                port.violation(format!("command {other}, which no driver path sends"));
                Err(self.error(ERR_COMMAND))
            }
        }
    }

    fn request(&mut self, wire: &[u8], port: &mut Port<'_>) {
        let Some(block) = decode_wire(wire) else {
            port.violation(format!("a request's bytes are not doubled nibbles: {wire:02x?}"));
            return;
        };
        if !self.c.awake {
            // A write wakes it, and goes unanswered.
            self.c.awake = true;
            return;
        }
        if self.c.answers.is_some_and(|n| self.answered >= n) {
            port.mark();
            return;
        }
        self.answered += 1;
        if self.c.miss == Some(self.answered) {
            return;
        }
        let to_all = block[0] == 0x03;
        if !to_all && block[0] != (((self.c.addr << 2) | 3) & 0x3f) {
            port.violation(format!("request to address byte {:#04x}", block[0]));
            return;
        }
        let answer = if let Err(error) = self.check(&block, port) {
            error
        } else {
            self.c.awake = block[3] != CMD_MODE;
            let framed = self.c.frames.as_mut().map(Packets::take).filter(|f| !f.is_empty());
            framed.unwrap_or_else(|| self.usual(block[3]))
        };
        self.answer(answer, port);
        if to_all && let Some(addr) = self.c.second {
            let payload: Vec<u8> = self.power_on.status.clone().take();
            self.answer_raw(&Self::block(addr, block[3], &payload), port);
        }
    }

    fn answer(&mut self, mut bytes: Vec<u8>, port: &mut Port<'_>) {
        if let Some((nth, k)) = self.c.cut
            && self.answered == nth
        {
            bytes.truncate(BLOCK_BYTES - k as usize);
        }
        if let Some((nth, offers)) = self.c.late
            && self.answered == nth
        {
            self.held = Some((offers, bytes));
            return;
        }
        self.answer_raw(&bytes, port);
    }

    fn answer_raw(&mut self, bytes: &[u8], port: &mut Port<'_>) {
        let out: Vec<u8> = bytes.iter().map(|b| b | self.c.high).collect();
        if let Some((_, held)) = &mut self.held {
            // A meter sends one answer at a time.
            held.extend_from_slice(&out);
        } else {
            port.send(&out);
        }
    }

    fn send_tick(&mut self, port: &mut Port<'_>) {
        if !self.line_fits(port.line()) {
            return;
        }
        if self.c.burst.is_some_and(|b| self.sent >= b)
            || self.c.blocks.is_some_and(|b| self.total >= b)
        {
            // A period after the last block: the tool has read it by then.
            self.quiet = self.quiet.saturating_add(1);
            if self.quiet == self.c.period {
                port.mark();
            }
            return;
        }
        if self.started && !self.reopened {
            return;
        }
        self.polls += 1;
        let due = if self.started { self.c.period } else { self.c.lead };
        if self.polls >= due {
            self.polls = 0;
            self.started = true;
            self.sent += 1;
            self.total += 1;
            let mask = if self.uart.data_bits == 6 { 0x3f } else { 0xff };
            let block: Vec<u8> = self.c.msgs.take().iter().map(|b| b & mask).collect();
            port.send(&block);
        }
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        if !self.bidi {
            port.violation(format!("{} bytes written to a meter in send mode", data.len()));
            return;
        }
        for byte in data {
            self.wire.push(*byte);
            if self.wire.len() == WIRE_BYTES {
                let wire = std::mem::take(&mut self.wire);
                self.request(&wire, port);
            }
        }
    }

    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        if event.opened() {
            if !self.wire.is_empty() {
                port.violation(format!("{} bytes of a request left at an open", self.wire.len()));
                self.wire.clear();
            }
            self.polls = 0;
            self.sent = 0;
            self.quiet = 0;
            self.reopened = self.started;
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if !self.bidi {
            self.send_tick(port);
            return;
        }
        if let Some((left, bytes)) = &mut self.held {
            *left -= 1;
            if *left == 0 {
                let bytes = std::mem::take(bytes);
                self.held = None;
                port.send(&bytes);
            }
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.power_on.clone(), self.bidi, self.uart);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Reply;
    use crate::shells::cdc::host::Host;

    /// A request as the document has it, on the wire.
    fn wire(block: [u8; BLOCK_BYTES]) -> Vec<u8> {
        let two = |b: u8| [0x00u8, 0x0f, 0xf0, 0xff][usize::from(b & 3)];
        block.iter().flat_map(|b| [two(*b), two(*b >> 2), two(*b >> 4)]).collect()
    }

    fn request(addr_byte: u8, command: u8, params: [u8; 9]) -> Vec<u8> {
        let mut b = [0; BLOCK_BYTES];
        b[0] = addr_byte;
        b[1] = 0x2b;
        b[2] = 0x3f;
        b[3] = command;
        b[4..13].copy_from_slice(&params);
        b[13] = checksum(&b);
        wire(b)
    }

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let cdc = function(&o).unwrap();
        let line = cdc.model.uart;
        Host::open(cdc, line, true, true)
    }

    const STATUS_TO_ALL: [u8; 9] = [0; 9];

    #[test]
    fn the_documents_example_address_and_checksum() {
        // Address 1 is 7H, by the document; an answer's bytes sum to 0 in six bits.
        let r = request(0x07, CMD_MEASURE, [0; 9]);
        let b = decode_wire(&r).unwrap();
        assert_eq!(b[0], 0x07);
        let sum = b.iter().fold(0u8, |a, x| a.wrapping_add(*x));
        assert_eq!(sum & 0x3f, 0);
    }

    #[test]
    fn it_answers_status_and_measure() {
        let mut h = meter(&[]);
        h.write(&request(0x03, CMD_STATUS, STATUS_TO_ALL));
        let a = h.read();
        assert_eq!(
            a,
            [1, 0x27, 0x3f, 3, 2, 1, 3, 1, 1, 0, 0, 0x1e, 1, checksum(&a)],
            "firmware 1.2 of a 26S"
        );
        assert!(a.iter().fold(0u8, |s, x| s.wrapping_add(*x)).trailing_zeros() >= 6);
        h.write(&request(0x07, CMD_MEASURE, [0; 9]));
        let a = h.read();
        assert_eq!(a[..13], [1, 0x27, 0x3f, 8, 0, 1, 0x11, 6, 5, 4, 3, 2, 1]);
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn a_request_may_come_cut_anywhere() {
        let mut h = meter(&[]);
        let r = request(0x03, CMD_STATUS, [0; 9]);
        h.write(&r[..10]);
        assert_eq!(h.read(), b"");
        h.write(&r[10..]);
        assert_eq!(h.read().len(), BLOCK_BYTES);
    }

    #[test]
    fn each_check_fires() {
        let mut h = meter(&[]);
        let mut bad = request(0x03, CMD_STATUS, [0; 9]);
        bad[0] = 0x55;
        h.write(&bad);
        assert_eq!(h.v.drain().len(), 1, "not doubled nibbles");
        assert_eq!(h.read(), b"");

        let mut b = decode_wire(&request(0x03, CMD_STATUS, [0; 9])).unwrap();
        b[13] ^= 1;
        h.write(&wire(b));
        assert_eq!(h.v.drain().len(), 1, "checksum");
        assert_eq!(h.read()[..3], [1, 0, ERR_CHECKSUM]);

        let mut b = decode_wire(&request(0x03, CMD_STATUS, [0; 9])).unwrap();
        b[1] = 0x2a;
        b[13] = checksum(&b);
        h.write(&wire(b));
        assert_eq!(h.v.drain().len(), 1, "header");
        assert_eq!(h.read()[..3], [1, 0, ERR_HEADER]);

        h.write(&request(0x03, 2, [0; 9]));
        assert_eq!(h.v.drain().len(), 1, "a command no path sends");
        assert_eq!(h.read()[..3], [1, 0, ERR_COMMAND]);

        h.write(&request(0x03, CMD_STATUS, [1, 0, 0, 0, 0, 0, 0, 0, 0]));
        assert_eq!(h.v.drain().len(), 1, "Idx");
        assert_eq!(h.read()[..3], [1, 0, ERR_PARAMETER]);

        h.write(&request(0x03, CMD_MODE, [5, 0, 0, 0, 0, 0, 0, 0, 0]));
        assert_eq!(h.v.drain().len(), 1, "a mode that is not off");
        assert_eq!(h.read()[..3], [1, 0, ERR_PARAMETER]);

        h.write(&request(0x0b, CMD_STATUS, [0; 9]));
        assert_eq!(h.v.drain().len(), 1, "another address");
        assert_eq!(h.read(), b"");
    }

    #[test]
    fn a_partial_request_at_an_open_is_flagged() {
        let mut h = meter(&[]);
        h.write(&[0; 3]);
        h.set_lines(true, true);
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn switching_off_answers_and_the_next_write_wakes_it() {
        let mut h = meter(&[]);
        h.write(&request(0x07, CMD_MODE, [5, 5, 0, 0, 0, 0, 0, 0, 0]));
        assert_eq!(h.read()[3], CMD_MODE);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read(), b"", "woken, not answered");
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
        assert!(h.v.is_empty(), "{:?}", h.v);
    }

    #[test]
    fn asleep_takes_a_write_to_wake() {
        let mut h = meter(&[("asleep", "1")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read(), b"");
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
    }

    #[test]
    fn answers_run_out_and_mark_the_point() {
        let mut h = meter(&[("answers", "1")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
        assert!(!h.cdc.mark());
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read(), b"");
        assert!(h.cdc.mark());
    }

    #[test]
    fn a_late_answer_waits_its_offers_and_holds_the_next() {
        let mut h = meter(&[("late", "2:3")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
        h.write(&request(0x07, CMD_MEASURE, [0; 9]));
        assert_eq!(h.offer(), Reply::Nak);
        h.write(&request(0x07, CMD_STATUS, [0; 9]));
        assert_eq!(h.offer(), Reply::Nak);
        match h.offer() {
            Reply::Data(d) => {
                assert_eq!(d.len(), 2 * BLOCK_BYTES, "the answer it held, then the next");
                assert_eq!(d[3], CMD_MEASURE);
                assert_eq!(d[BLOCK_BYTES + 3], CMD_STATUS);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_missed_request_goes_unanswered_once() {
        let mut h = meter(&[("miss", "1")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read(), b"");
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
    }

    #[test]
    fn a_cut_answer_is_short() {
        let mut h = meter(&[("cut", "1:3")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES - 3);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
    }

    #[test]
    fn frames_replace_answers_in_turn_and_a_dash_is_the_usual() {
        let o = vec![("frames".to_owned(), "012".to_owned())];
        assert!(build(&o).is_err(), "not whole bytes");
        let mut h = meter(&[("frames", "-,0127ffffffffffffffffffffffff")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read()[3], CMD_STATUS);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert_eq!(h.read()[2], 0xff);
    }

    #[test]
    fn high_bits_go_on_every_byte() {
        let mut h = meter(&[("high", "40")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        assert!(h.read().iter().all(|b| b & 0xc0 == 0x40));
        assert!(function(&[("high".to_owned(), "41".to_owned())]).is_err());
    }

    #[test]
    fn a_second_meter_answers_requests_to_all_only() {
        let mut h = meter(&[("second", "2")]);
        h.write(&request(0x03, CMD_STATUS, [0; 9]));
        let a = h.read();
        assert_eq!(a.len(), 2 * BLOCK_BYTES);
        assert_eq!((a[0], a[BLOCK_BYTES]), (1, 2));
        h.write(&request(0x07, CMD_STATUS, [0; 9]));
        assert_eq!(h.read().len(), BLOCK_BYTES);
    }

    #[test]
    fn send_mode_takes_nothing_and_sends_by_offers() {
        let mut h = meter(&[("mode", "send"), ("lead", "2"), ("period", "3"), ("msgs", "01,02")]);
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.offer(), Reply::Data(vec![1]));
        for _ in 0..10 {
            assert_eq!(h.offer(), Reply::Nak, "the scan's block, and none until an open");
        }
        // An open starts the count over, at a period, and not the list.
        h.set_lines(true, true);
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.offer(), Reply::Data(vec![2]));
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.offer(), Reply::Data(vec![1]), "then on, each period");
        h.write(b"x");
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn six_bits_on_a_six_bit_line() {
        let mut h = meter(&[("mode", "send"), ("lead", "1"), ("msgs", "ff")]);
        assert_eq!(h.offer(), Reply::Data(vec![0x3f]));
    }

    #[test]
    fn send_mode_holds_its_blocks_on_a_misset_line() {
        let mut h = meter(&[("mode", "send"), ("uart", "8228/6n1"), ("lead", "1"), ("msgs", "01,02")]);
        h.set_line(Line::parse("9600/6n1").unwrap());
        for _ in 0..5 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        h.set_line(Line::parse("8228/6n1").unwrap());
        assert_eq!(h.offer(), Reply::Data(vec![1]), "the first, not one lost to the wrong rate");
    }

    #[test]
    fn a_burst_ends_in_silence_and_the_point() {
        let mut h = meter(&[("mode", "send"), ("lead", "1"), ("period", "2"), ("burst", "1")]);
        assert!(matches!(h.offer(), Reply::Data(_)));
        assert_eq!(h.offer(), Reply::Nak);
        assert!(!h.cdc.mark());
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
        for _ in 0..5 {
            assert_eq!(h.offer(), Reply::Nak);
        }
        assert!(!h.cdc.mark(), "once");
    }

    #[test]
    fn blocks_count_across_opens() {
        let mut h = meter(&[("mode", "send"), ("lead", "1"), ("period", "2"), ("blocks", "1")]);
        assert!(matches!(h.offer(), Reply::Data(_)));
        h.set_lines(true, true);
        for _ in 0..4 {
            assert_eq!(h.offer(), Reply::Nak, "none after the open either");
        }
        assert!(h.cdc.mark());
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("mode", "x"),
            ("status", "00"),
            ("meas", "zz"),
            ("period", "1"),
            ("addr", "0"),
            ("late", "1"),
            ("cut", "1:14"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [("chunk", "1"), ("stall", "line"), ("answers", "0"), ("late", "1:5")] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
