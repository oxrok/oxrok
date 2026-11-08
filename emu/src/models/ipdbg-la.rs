// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The IPDBG logic analyzer core, on a socket: what libsigrok's `ipdbg-la` talks to
//! through the `OpenOCD` IPDBG server's or another's TCP port. TCP alone, with no USB.
//!
//! The device, and how sure this is:
//!
//! - The commands, their order and the fields: the core's source, IPDBG's
//!   `rtl/LogicAnalyser/LogicAnalyserController.vhd`, `LogicAnalyserMemory.vhd`,
//!   `LogicAnalyserTrigger.vhd`, `rtl/common/IpdbgEscaping.vhd` and the README beside
//!   them (master, read 2026-11-06). Bytes to the core are escaped: `0xEE` unescaped
//!   resets it, `0x55` makes the next byte data. Fields go most significant byte first.
//!   Sizes are two 32 bit values, least significant byte first. A dump is all of
//!   2^`ADDR_WIDTH` samples, oldest first, each least significant byte first, and the
//!   trigger's sample is number `delay`.
//! - The trigger: `LogicAnalyserTrigger.vhd`'s expression, sample by sample.
//! - What the masks and the values hold across a reset: the README says they aren't
//!   set by one, and that all must be set before the first capture.
//!
//! What is the model's and no device's:
//!
//! - The samples, a stand-in: a ramp from the arming. The first two bytes of a sample
//!   are a 16 bit count, and later bytes are other cuts of it. A real core samples a
//!   free-running design, and the host cannot know its pace: `sample_enable` is the
//!   user's signal.
//! - An answer comes at once, whole, or in the pieces `pace=` says. Time here is
//!   [`Stream::tick`], 20 ms of wall clock: what a JTAG hub and a server do to the
//!   pace isn't modelled, nor how long the core holds a byte. A real dump over JTAG
//!   comes in pieces the server's polling cuts.
//! - What a server does at a connect or a close. `OpenOCD`'s isn't read here, and a
//!   comment on the core maintainer's libsigrok branch (`danselmi/libsigrok`,
//!   `ipdbg-la-rlc`, `ipdbg_la_tcp_close`) says it serves one connection per core and
//!   rejects a second while the first is closing, which this carrier doesn't do. The
//!   state of the core stays across connections, as in the FPGA, and a dump cut by a
//!   close goes on at the next connection, until a reset.
//! - The memory of `0x00` (fire now), `0x10` (features) and `0x60` (run length width):
//!   the HDL has them, no C path sends them, and they are flagged.
//!
//! The options, none of which a device has:
//!
//! - `data-width=N` (default 16), `addr-width=N` (default 9), which is what the
//!   IPDBG Cmod S7 demo has;
//! - `sizes=HEX`: answer `0xAA` with these bytes instead, short or wrong;
//! - `id=HEX`: answer `0xBB` with these bytes instead (`69646267` is `idbg`, a core
//!   with run length compression);
//! - `mute=id|sizes`: no answer to that, and the connection stays;
//! - `hang=id|sizes|arm`: close at that command;
//! - `late=id|sizes:T`: that answer T ticks late;
//! - `first=T`: the dump starts T ticks after the arming, as a slow `sample_enable`;
//! - `pace=N`: N bytes of the dump per tick;
//! - `never=1`: the trigger doesn't fire, as a condition the design never meets;
//! - `cut=N`: the dump stops after N bytes, with the connection up;
//! - `gap=N:T`: T ticks of nothing after N bytes of the dump;
//! - `close-at=N`: after N bytes of the dump the connection closes;
//! - `extra=N`: N more bytes after a dump, which no core sends;
//! - `mark-at=N`: mark the event point when N bytes of the dump are out;
//! - `mark-arm=1`: mark it at the arming.
//!
//! The pedantic checks: anything but a command in the idle state; anything but what the
//! state wants in a configuration; a command while a dump or a capture runs, other than
//! the reset; an arming before every field was set; a delay past the memory; an escape
//! or a reset inside a field where C has none.

use crate::model::DeviceModel;
use crate::options::{parse_hex_list, parse_number};
use crate::shells::tcp::{NoUsb, Port, Stream};
use crate::txlog::{HASH_ABOVE, Rules};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "ipdbg-la";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "The IPDBG logic analyzer core on TCP; data-width=, addr-width=, sizes=, \
                         id=, mute=, hang=, late=, first=, pace=, never=, cut=, gap=, close-at=, \
                         extra=, mark-at=, mark-arm=";

const CMD_FIRE: u8 = 0x00;
const CMD_CFG_TRIGGER: u8 = 0xF0;
const CMD_CFG_LA: u8 = 0x0F;
const CMD_ARM: u8 = 0xFE;
const CMD_RESET: u8 = 0xEE;
const CMD_SIZES: u8 = 0xAA;
const CMD_ID: u8 = 0xBB;
const CMD_ESCAPE: u8 = 0x55;
const CMD_FEATURES: u8 = 0x10;
const CMD_RLC_WIDTH: u8 = 0x60;
const SELECT_CURR: u8 = 0xF1;
const SELECT_LAST: u8 = 0xF9;
const SELECT_EDGE: u8 = 0xF5;
const SET_MASK_CURR: u8 = 0xF3;
const SET_VALUE_CURR: u8 = 0xF7;
const SET_MASK_LAST: u8 = 0xFB;
const SET_VALUE_LAST: u8 = 0xFF;
const SET_EDGE_MASK: u8 = 0xF6;
const SET_DELAY: u8 = 0x1F;

/// How far past the delay the ramp is searched for a trigger: a bit past the 16 bit
/// count, so that a condition on those bits comes round.
const SEARCH: u64 = 1 << 17;
/// The most of a dump sent in one go, with no `pace=`.
const PIECE: u64 = 1 << 20;

/// An answer the options can act on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Query {
    Id,
    Sizes,
}

/// A field of the configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Field {
    MaskCurr,
    ValueCurr,
    MaskLast,
    ValueLast,
    Edge,
    Delay,
}

const FIELDS: usize = 6;

/// Where the command decoder is, as in `LogicAnalyserController.vhd`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    ConfigTrigger,
    SelectCurr,
    SelectLast,
    SelectEdge,
    ConfigLa,
    Field(Field),
}

/// Where a capture is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Capture {
    /// Nothing armed.
    None,
    /// Armed, the trigger not in this ramp.
    Waiting,
    /// Sending the dump, from sample `start` of the ramp.
    Dumping { start: u64, sent: u64, wait: u32 },
    /// All sent.
    Done,
}

#[derive(Clone, Debug, Default)]
struct Faults {
    id: Option<Vec<u8>>,
    sizes: Option<Vec<u8>>,
    mute: Option<Query>,
    hang: Option<&'static str>,
    late: Option<(Query, u32)>,
    first: u32,
    pace: u32,
    never: bool,
    cut: Option<u64>,
    gap: Option<(u64, u32)>,
    close_at: Option<u64>,
    extra: u32,
    mark_at: Option<u64>,
    mark_arm: bool,
}

/// The core.
#[derive(Debug)]
pub struct Core {
    data_width: u32,
    addr_width: u32,
    faults: Faults,
    state: State,
    escaped: bool,
    capture: Capture,
    /// What a field has taken so far, and how many bytes it wants.
    taking: Vec<u8>,
    wants: usize,
    fields: [Vec<u8>; FIELDS],
    set: [bool; FIELDS],
    /// An answer held by `late=`, and the ticks to it.
    held: Option<(Query, u32)>,
    /// The gap of `gap=`, once taken.
    gap_done: bool,
}

fn field_index(f: Field) -> usize {
    match f {
        Field::MaskCurr => 0,
        Field::ValueCurr => 1,
        Field::MaskLast => 2,
        Field::ValueLast => 3,
        Field::Edge => 4,
        Field::Delay => 5,
    }
}

impl Core {
    fn new(data_width: u32, addr_width: u32, faults: Faults) -> Core {
        Core {
            data_width,
            addr_width,
            faults,
            state: State::Idle,
            escaped: false,
            capture: Capture::None,
            taking: Vec::new(),
            wants: 0,
            fields: Default::default(),
            set: [false; FIELDS],
            held: None,
            gap_done: false,
        }
    }

    fn data_bytes(&self) -> usize {
        self.data_width.div_ceil(8) as usize
    }

    fn addr_bytes(&self) -> usize {
        self.addr_width.div_ceil(8) as usize
    }

    /// Bytes in a whole dump.
    fn dump_len(&self) -> u64 {
        (1u64 << self.addr_width.min(40)).saturating_mul(self.data_bytes() as u64)
    }

    /// Byte `b` of the ramp's sample `p`: zero in the unused bits, as the HDL's narrow
    /// path has it (the README calls them undefined).
    fn sample_byte(&self, p: u64, b: usize) -> u8 {
        let raw = match b {
            0 => p as u8,
            1 => (p >> 8) as u8,
            _ => ((p.wrapping_mul(2 * b as u64 + 1)) >> (b % 7)) as u8,
        };
        let used = self.data_width as usize - b * 8;
        if used >= 8 { raw } else { raw & ((1u8 << used) - 1) }
    }

    /// `LogicAnalyserTrigger.vhd`, for the ramp's sample `p`. Its fields are most
    /// significant byte first.
    fn fires(&self, p: u64) -> bool {
        let n = self.data_bytes();
        let last_p = p.saturating_sub(1);
        let field = |f: Field, b: usize| self.fields[field_index(f)][n - 1 - b];
        (0..n).all(|b| {
            let curr = self.sample_byte(p, b);
            let last = self.sample_byte(last_p, b);
            let last_ok = !(last ^ field(Field::ValueLast, b)) | !field(Field::MaskLast, b);
            let both = last_ok & !(curr ^ field(Field::ValueCurr, b));
            let edge_ok = !field(Field::Edge, b) | (last ^ curr);
            let used = self.data_width as usize - b * 8;
            let bits = if used >= 8 { 0xFF } else { (1u8 << used) - 1 };
            ((both | !field(Field::MaskCurr, b)) & edge_ok) | !bits == 0xFF
        })
    }

    fn delay(&self) -> u64 {
        self.fields[field_index(Field::Delay)].iter().fold(0, |v, b| (v << 8) | u64::from(*b))
    }

    fn reset_core(&mut self) {
        self.state = State::Idle;
        self.escaped = false;
        self.capture = Capture::None;
        self.held = None;
        self.gap_done = false;
    }

    fn answer(&mut self, q: Query, port: &mut Port) {
        if self.faults.mute == Some(q) {
            return;
        }
        match (q, self.faults.hang) {
            (Query::Id, Some("id")) | (Query::Sizes, Some("sizes")) => {
                port.hang_up();
                return;
            }
            _ => {}
        }
        if let Some((late, ticks)) = self.faults.late
            && late == q
        {
            self.held = Some((q, ticks));
            return;
        }
        self.say(q, port);
    }

    fn say(&mut self, q: Query, port: &mut Port) {
        match q {
            Query::Id => match &self.faults.id {
                Some(id) => port.send(id),
                None => port.send(b"IDBG"),
            },
            Query::Sizes => {
                if let Some(sizes) = &self.faults.sizes {
                    port.send(sizes);
                } else {
                    let mut a = self.data_width.to_le_bytes().to_vec();
                    a.extend_from_slice(&self.addr_width.to_le_bytes());
                    port.send(&a);
                }
            }
        }
    }

    fn arm(&mut self, port: &mut Port) {
        if self.faults.hang == Some("arm") {
            port.hang_up();
            return;
        }
        if self.set != [true; FIELDS] {
            port.violation("armed before every mask, value and the delay were set");
            return;
        }
        let delay = self.delay();
        if self.addr_width < 40 && delay >= 1u64 << self.addr_width {
            port.violation(format!(
                "a delay of {delay} in a memory of 2^{} samples",
                self.addr_width
            ));
        }
        self.gap_done = false;
        if self.faults.mark_arm {
            port.mark();
        }
        let found = if self.faults.never {
            None
        } else {
            (delay..delay + SEARCH).find(|p| self.fires(*p))
        };
        self.capture = match found {
            Some(p) => {
                Capture::Dumping { start: p - delay, sent: 0, wait: self.faults.first }
            }
            None => Capture::Waiting,
        };
        self.pump(port, false);
    }

    /// Sends what the dump may send now.
    fn pump(&mut self, port: &mut Port, ticked: bool) {
        let Capture::Dumping { start, mut sent, mut wait } = self.capture else {
            return;
        };
        if wait > 0 {
            if ticked {
                wait -= 1;
            }
            self.capture = Capture::Dumping { start, sent, wait };
            if wait > 0 || !ticked {
                return;
            }
        }
        let total = self.dump_len();
        let pace = u64::from(self.faults.pace);
        let mut budget = if pace == 0 {
            PIECE
        } else if ticked {
            pace
        } else {
            0
        };
        let n = self.data_bytes() as u64;
        // What stops the piece: a fault's point, or its end.
        while budget > 0 && sent < total {
            let mut stop = total;
            for point in [
                self.faults.cut,
                self.faults.close_at,
                self.faults.mark_at,
                self.faults.gap.filter(|_| !self.gap_done).map(|g| g.0),
            ]
            .into_iter()
            .flatten()
            {
                if point > sent {
                    stop = stop.min(point);
                }
            }
            if self.faults.cut == Some(sent) {
                self.capture = Capture::Dumping { start, sent, wait: u32::MAX };
                return;
            }
            let upto = stop.min(sent + budget);
            let piece: Vec<u8> = (sent..upto)
                .map(|i| self.sample_byte(start + i / n, (i % n) as usize))
                .collect();
            port.send(&piece);
            budget -= upto - sent;
            sent = upto;
            if self.faults.mark_at == Some(sent) {
                port.mark();
            }
            if self.faults.close_at == Some(sent) {
                port.hang_up();
                self.capture = Capture::Done;
                return;
            }
            if let Some((at, ticks)) = self.faults.gap
                && at == sent
                && !self.gap_done
            {
                self.gap_done = true;
                self.capture = Capture::Dumping { start, sent, wait: ticks };
                return;
            }
            if self.faults.cut == Some(sent) {
                self.capture = Capture::Dumping { start, sent, wait: u32::MAX };
                return;
            }
        }
        if sent >= total {
            if self.faults.extra > 0 {
                port.send(&vec![0xA5; self.faults.extra as usize]);
            }
            self.capture = Capture::Done;
        } else {
            self.capture = Capture::Dumping { start, sent, wait: 0 };
        }
    }

    fn begin_field(&mut self, f: Field) {
        self.state = State::Field(f);
        self.taking.clear();
        self.wants = if f == Field::Delay { self.addr_bytes() } else { self.data_bytes() };
        if self.wants == 0 {
            self.finish_field(f);
        }
    }

    fn finish_field(&mut self, f: Field) {
        self.fields[field_index(f)] = std::mem::take(&mut self.taking);
        self.set[field_index(f)] = true;
        self.state = State::Idle;
    }

    /// A byte past the escape layer.
    fn command(&mut self, b: u8, escaped: bool, port: &mut Port) {
        if let State::Field(f) = self.state {
            self.taking.push(b);
            if self.taking.len() == self.wants {
                self.finish_field(f);
            }
            return;
        }
        if escaped {
            port.violation(format!("escaped byte {b:02x} where a command belongs"));
            return;
        }
        let dumping = matches!(self.capture, Capture::Dumping { .. } | Capture::Waiting);
        if dumping {
            port.violation(format!("command {b:02x} during a capture: the core ignores it"));
            return;
        }
        let next = match (self.state, b) {
            (State::Idle, CMD_ID) => {
                self.answer(Query::Id, port);
                State::Idle
            }
            (State::Idle, CMD_SIZES) => {
                self.answer(Query::Sizes, port);
                State::Idle
            }
            (State::Idle, CMD_ARM) => {
                self.arm(port);
                State::Idle
            }
            (State::Idle, CMD_CFG_TRIGGER) => State::ConfigTrigger,
            (State::Idle, CMD_CFG_LA) => State::ConfigLa,
            (State::Idle, CMD_FIRE | CMD_FEATURES | CMD_RLC_WIDTH) => {
                port.violation(format!("command {b:02x}: the core has it, no C path sends it"));
                State::Idle
            }
            (State::ConfigTrigger, SELECT_CURR) => State::SelectCurr,
            (State::ConfigTrigger, SELECT_LAST) => State::SelectLast,
            (State::ConfigTrigger, SELECT_EDGE) => State::SelectEdge,
            (State::SelectCurr, SET_MASK_CURR) => return self.begin_field(Field::MaskCurr),
            (State::SelectCurr, SET_VALUE_CURR) => return self.begin_field(Field::ValueCurr),
            (State::SelectLast, SET_MASK_LAST) => return self.begin_field(Field::MaskLast),
            (State::SelectLast, SET_VALUE_LAST) => return self.begin_field(Field::ValueLast),
            (State::SelectEdge, SET_EDGE_MASK) => return self.begin_field(Field::Edge),
            (State::ConfigLa, SET_DELAY) => return self.begin_field(Field::Delay),
            (state, _) => {
                port.violation(format!("byte {b:02x} in state {state:?}: the core ignores it"));
                state
            }
        };
        self.state = next;
    }
}

impl Stream for Core {
    fn received(&mut self, data: &[u8], port: &mut Port) {
        for &b in data {
            if self.escaped {
                self.escaped = false;
                self.command(b, true, port);
            } else if b == CMD_ESCAPE {
                if !matches!(self.state, State::Field(_)) {
                    port.violation("escape outside a field");
                }
                self.escaped = true;
            } else if b == CMD_RESET {
                if let State::Field(f) = self.state {
                    port.violation(format!("reset inside field {f:?}"));
                }
                self.reset_core();
            } else {
                self.command(b, false, port);
            }
        }
    }

    fn tick(&mut self, port: &mut Port) {
        if let Some((q, ticks)) = self.held {
            if ticks <= 1 {
                self.held = None;
                self.say(q, port);
            } else {
                self.held = Some((q, ticks - 1));
            }
        }
        if let Capture::Dumping { wait: u32::MAX, .. } = self.capture {
            return;
        }
        self.pump(port, true);
    }
}

fn query(value: &str) -> Result<Query> {
    match value {
        "id" => Ok(Query::Id),
        "sizes" => Ok(Query::Sizes),
        other => Err(Error::Invalid(format!("no answer called {other}"))),
    }
}

fn parts(options: &[(String, String)]) -> Result<Core> {
    let mut data_width = 16;
    let mut addr_width = 9;
    let mut f = Faults::default();
    for (k, v) in options {
        match k.as_str() {
            "data-width" => data_width = parse_number(k, v, 0..=u32::MAX)?,
            "addr-width" => addr_width = parse_number(k, v, 0..=u32::MAX)?,
            "sizes" => f.sizes = parse_hex_list(k, v, 1, 16)?.pop(),
            "id" => f.id = parse_hex_list(k, v, 1, 16)?.pop(),
            "mute" => f.mute = Some(query(v)?),
            "hang" => {
                f.hang = Some(match v.as_str() {
                    "id" => "id",
                    "sizes" => "sizes",
                    "arm" => "arm",
                    other => return Err(Error::Invalid(format!("hang={other}"))),
                });
            }
            "late" => {
                let (q, t) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("late={v}: id|sizes:T")))?;
                f.late = Some((query(q)?, parse_number(k, t, 1..=1_000_000)?));
            }
            "first" => f.first = parse_number(k, v, 0..=1_000_000)?,
            "pace" => f.pace = parse_number(k, v, 0..=1 << 24)?,
            "never" => f.never = parse_number(k, v, 0..=1)? == 1,
            "cut" => f.cut = Some(u64::from(parse_number(k, v, 0..=u32::MAX)?)),
            "gap" => {
                let (at, t) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("gap={v}: N:T")))?;
                f.gap = Some((
                    u64::from(parse_number(k, at, 0..=u32::MAX)?),
                    parse_number(k, t, 1..=1_000_000)?,
                ));
            }
            "close-at" => f.close_at = Some(u64::from(parse_number(k, v, 0..=u32::MAX)?)),
            "extra" => f.extra = parse_number(k, v, 0..=1 << 16)?,
            "mark-at" => f.mark_at = Some(u64::from(parse_number(k, v, 0..=u32::MAX)?)),
            "mark-arm" => f.mark_arm = parse_number(k, v, 0..=1)? == 1,
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
        }
    }
    Ok(Core::new(data_width, addr_width, f))
}

/// No USB: the model is on the socket alone.
///
/// # Errors
///
/// When an option is wrong.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    parts(options)?;
    Ok(Box::new(NoUsb::new(NAME)))
}

/// The core on a socket.
///
/// # Errors
///
/// When an option is wrong.
pub fn build_tcp(options: &[(String, String)]) -> Result<Box<dyn Stream>> {
    Ok(Box::new(parts(options)?))
}

/// A dump is long: its digest in the log.
pub fn log_rules(_options: &[(String, String)]) -> Rules {
    Rules::digest_above(HASH_ABOVE)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn core(options: &[(&str, &str)]) -> Core {
        let o: Vec<(String, String)> =
            options.iter().map(|(a, b)| ((*a).to_owned(), (*b).to_owned())).collect();
        parts(&o).expect("options")
    }

    fn sent(port: &Port) -> Vec<u8> {
        port.sent().concat()
    }

    fn drive(c: &mut Core, bytes: &[u8]) -> Port {
        let mut p = Port::default();
        c.received(bytes, &mut p);
        p
    }

    /// What C sends for all-zero masks and delay `delay`, 16 data and 9 address bits.
    fn config(delay: u16) -> Vec<u8> {
        let mut v = Vec::new();
        for (a, b, c) in [
            (0xF0, 0xF1, 0xF3),
            (0xF0, 0xF1, 0xF7),
            (0xF0, 0xF9, 0xFB),
            (0xF0, 0xF9, 0xFF),
            (0xF0, 0xF5, 0xF6),
        ] {
            v.extend_from_slice(&[a, b, c, 0, 0]);
        }
        v.extend_from_slice(&[0x0F, 0x1F]);
        for byte in delay.to_be_bytes() {
            if byte == 0x55 || byte == 0xEE {
                v.push(0x55);
            }
            v.push(byte);
        }
        v
    }

    fn violations(p: &mut Port) -> usize {
        p.violations().drain().len()
    }

    #[test]
    fn id_and_sizes_answer() {
        let mut c = core(&[]);
        let mut p = drive(&mut c, &[0xEE, 0xEE, 0xBB, 0xAA]);
        assert_eq!(sent(&p), b"IDBG\x10\0\0\0\x09\0\0\0");
        assert_eq!(violations(&mut p), 0);
    }

    #[test]
    fn a_dump_is_all_of_memory_with_the_trigger_at_delay() {
        let mut c = core(&[]);
        let mut cfg = config(255);
        cfg.push(0xFE);
        let mut p = drive(&mut c, &cfg);
        assert_eq!(violations(&mut p), 0);
        let d = sent(&p);
        assert_eq!(d.len(), 1024);
        assert_eq!(&d[..4], &[0, 0, 1, 0]);
        assert_eq!(&d[1022..], &[0xFF, 1]);
    }

    #[test]
    fn a_trigger_moves_the_dump() {
        let mut c = core(&[]);
        // Rising on bit 1 of the low byte, as C sends it: value 2, mask 2, last value 0,
        // last mask 2.
        let mut b = Vec::new();
        b.extend_from_slice(&[0xF0, 0xF1, 0xF3, 0, 2]);
        b.extend_from_slice(&[0xF0, 0xF1, 0xF7, 0, 2]);
        b.extend_from_slice(&[0xF0, 0xF9, 0xFB, 0, 2]);
        b.extend_from_slice(&[0xF0, 0xF9, 0xFF, 0, 0]);
        b.extend_from_slice(&[0xF0, 0xF5, 0xF6, 0, 0]);
        b.extend_from_slice(&[0x0F, 0x1F, 0, 4]);
        b.push(0xFE);
        let mut p = drive(&mut c, &b);
        assert_eq!(violations(&mut p), 0);
        // Past the delay of 4, bit 1 next rises at 6: the dump starts at 2.
        assert_eq!(&sent(&p)[..2], &[2, 0]);
    }

    #[test]
    fn edge_alone_fires_on_any_change() {
        let mut c = core(&[]);
        let mut b = config(0);
        // The edge mask, last: bit 8, the second byte's lowest.
        let at = b.iter().rposition(|x| *x == 0xF6).expect("edge");
        b[at + 1] = 1;
        b.push(0xFE);
        let p = drive(&mut c, &b);
        assert_eq!(&sent(&p)[..2], &[0, 1]);
    }

    #[test]
    fn escapes_are_data_and_a_reset_is_no_field() {
        let mut c = core(&[]);
        let mut b = config(0);
        // 0x155: a delay with an escaped low byte.
        let n = b.len();
        b.truncate(n - 2);
        b.extend_from_slice(&[1, 0x55, 0x55, 0xFE]);
        let mut p = drive(&mut c, &b);
        assert_eq!(violations(&mut p), 0);
        assert_eq!(c.delay(), 0x155);
    }

    #[test]
    fn reset_ends_a_dump() {
        let mut c = core(&[("pace", "10")]);
        let mut b = config(0);
        b.push(0xFE);
        let mut p = drive(&mut c, &b);
        c.tick(&mut p);
        assert_eq!(sent(&p).len(), 10);
        let mut q = drive(&mut c, &[0xEE]);
        c.tick(&mut q);
        assert_eq!(sent(&q).len(), 0);
        assert_eq!(violations(&mut q), 0);
    }

    #[test]
    fn faults_each_do_what_they_say() {
        let run = |o: &[(&str, &str)]| {
            let mut c = core(o);
            let mut b = config(0);
            b.push(0xFE);
            let mut p = drive(&mut c, &b);
            for _ in 0..4 {
                c.tick(&mut p);
            }
            sent(&p).len()
        };
        assert_eq!(run(&[]), 1024);
        assert_eq!(run(&[("cut", "7")]), 7);
        assert_eq!(run(&[("close-at", "9")]), 9);
        assert_eq!(run(&[("extra", "3")]), 1027);
        assert_eq!(run(&[("never", "1")]), 0);
        assert_eq!(run(&[("first", "2")]), 1024);
        assert_eq!(run(&[("first", "9")]), 0);
        assert_eq!(run(&[("gap", "5:2")]), 1024);
        assert_eq!(run(&[("gap", "5:9")]), 5);
        assert_eq!(run(&[("pace", "3")]), 12);
    }

    #[test]
    fn answers_change_with_the_faults() {
        let mut c = core(&[("id", "69646267"), ("sizes", "01")]);
        assert_eq!(sent(&drive(&mut c, &[0xBB, 0xAA])), b"idbg\x01");
        let mut c = core(&[("mute", "id")]);
        assert_eq!(sent(&drive(&mut c, &[0xBB])), b"");
        let mut c = core(&[("late", "sizes:2")]);
        let mut p = drive(&mut c, &[0xAA]);
        assert_eq!(sent(&p), b"");
        c.tick(&mut p);
        assert_eq!(sent(&p), b"");
        c.tick(&mut p);
        assert_eq!(sent(&p).len(), 8);
    }

    // Each check once.

    #[test]
    fn arming_before_the_fields_is_flagged() {
        let mut c = core(&[]);
        assert_eq!(violations(&mut drive(&mut c, &[0xFE])), 1);
    }

    #[test]
    fn a_delay_past_the_memory_is_flagged() {
        let mut c = core(&[]);
        let mut b = config(0x200);
        b.push(0xFE);
        assert_eq!(violations(&mut drive(&mut c, &b)), 1);
    }

    #[test]
    fn a_command_during_a_capture_is_flagged() {
        let mut c = core(&[("never", "1")]);
        let mut b = config(0);
        b.push(0xFE);
        b.push(0xBB);
        let mut p = drive(&mut c, &b);
        assert_eq!(violations(&mut p), 1);
        assert_eq!(sent(&p), b"");
    }

    #[test]
    fn commands_c_lacks_are_flagged() {
        for cmd in [0x00, 0x10, 0x60, 0x77] {
            let mut c = core(&[]);
            assert_eq!(violations(&mut drive(&mut c, &[cmd])), 1, "{cmd:02x}");
        }
    }

    #[test]
    fn a_wrong_step_of_a_configuration_is_flagged() {
        for bytes in [&[0xF0, 0x77][..], &[0xF0, 0xF1, 0xFB], &[0x0F, 0xF0], &[0xF0, 0xF5, 0xF3]] {
            let mut c = core(&[]);
            assert_eq!(violations(&mut drive(&mut c, bytes)), 1, "{bytes:02x?}");
        }
    }

    #[test]
    fn an_escape_outside_a_field_is_flagged() {
        let mut c = core(&[]);
        // Twice: the escape, and the escaped byte where a command belongs.
        assert_eq!(violations(&mut drive(&mut c, &[0x55, 0x11])), 2);
    }

    #[test]
    fn a_reset_inside_a_field_is_flagged() {
        let mut c = core(&[]);
        assert_eq!(violations(&mut drive(&mut c, &[0xF0, 0xF1, 0xF3, 0, 0xEE])), 1);
    }

    #[test]
    fn options_are_checked() {
        for o in ["nope=1", "mute=x", "hang=x", "late=x", "gap=1"] {
            let (k, v) = o.split_once('=').expect("=");
            assert!(parts(&[(k.to_owned(), v.to_owned())]).is_err(), "{o}");
        }
        assert!(build(&[]).is_ok());
        assert!(build_tcp(&[]).is_ok());
    }
}
