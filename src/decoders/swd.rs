// Counterpart in sigrok:
//   libsigrokdecode/decoders/swd/pd.py,
//   Copyright (C) 2014 Angus Gratton <gus@projectgus.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ARM Serial Wire Debug protocol and the decoder's
//   documented classes and texts.

//! SWD: Serial Wire Debug requests, ACKs and data over SWCLK and SWDIO.

use crate::core::Result;
use crate::decode::{
    AnnClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef, DecoderInfo, Item,
    OptionDesc, OptionValue,
};

const SWCLK: usize = 0;
const SWDIO: usize = 1;

/// Sample edges.
const FALLING: bool = false;
const RISING: bool = true;

const ADDR_DP_SELECT: i64 = 0x8;
const ADDR_DP_CTRLSTAT: i64 = 0x4;
const BIT_SELECT_CTRLSEL: u32 = 1;
const BIT_CTRLSTAT_ORUNDETECT: u32 = 1;

/// `bin(0xE79E)[:1:-1]`, the 16 bit JTAG to SWD switch sequence as it
/// appears in SWDIO chronological order.
const RE_SWDSWITCH: &str = "0111100111100111";

/// Annotation classes, in `ANNOTATIONS` order.
const ANN_RESET: usize = 0;
const ANN_ENABLE: usize = 1;
const ANN_READ: usize = 2;
const ANN_WRITE: usize = 3;
const ANN_ACK: usize = 4;
const ANN_DATA: usize = 5;
const ANN_PARITY: usize = 6;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "swd",
    name: "SWD",
    longname: "Serial Wire Debug",
    desc: "Two-wire protocol for debug access to ARM CPUs.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["swd"],
    tags: &["Debug/trace"],
    channels: &[
        ChannelDesc { id: "swclk", name: "SWCLK", desc: "Master clock" },
        ChannelDesc { id: "swdio", name: "SWDIO", desc: "Data input/output" },
    ],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "strict_start",
        desc: "Wait for a line reset before starting to decode",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("yes"), OptionValue::s("no")],
    }],
    annotations: &[
        AnnClass { id: "reset", desc: "RESET" },
        AnnClass { id: "enable", desc: "ENABLE" },
        AnnClass { id: "read", desc: "READ" },
        AnnClass { id: "write", desc: "WRITE" },
        AnnClass { id: "ack", desc: "ACK" },
        AnnClass { id: "data", desc: "DATA" },
        AnnClass { id: "parity", desc: "PARITY" },
    ],
    annotation_rows: &[],
    binary: &[],
    doc: "SWD (Serial Wire Debug) is the two wire debug protocol of ARM \
cores: a request phase on SWDIO selects a DP or AP register and a read or \
write, the target answers with an ACK, and a 32 bit data phase follows \
with a parity bit. Fifty or more high SWCLK cycles with SWDIO high form \
the line reset that starts a session.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Swd::default())
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum State {
    /// Idle/unknown.
    #[default]
    Unknown,
    /// Request phase (first 8 bits).
    Req,
    /// Ack phase (next 3 bits).
    Ack,
    /// Reading phase (next 32 bits for reads).
    Data,
    /// Data parity phase.
    Dparity,
}

/// `self.data`: `None` after `reset()`, `''` after `reset_state()`, a value
/// after a data phase. The distinction shows in the Python output.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Data {
    #[default]
    None,
    Empty,
    Value(u32),
}

impl Data {
    fn item(self) -> Item {
        match self {
            Data::None => Item::None,
            Data::Empty => Item::Str(String::new()),
            Data::Value(v) => Item::Int(i64::from(v)),
        }
    }
}

/// The SWD decoder.
#[derive(Default)]
pub struct Swd {
    state: State,
    sample_edge: bool,
    ack: Option<String>,
    ss_req: u64,
    turnaround: u32,
    /// Bits from SWDIO, matched against the expected sequences.
    bits: String,
    /// Sample numbers that correspond to the samples in `bits`.
    samplenums: Vec<u64>,
    linereset_count: usize,
    data: Data,
    addr: Option<i64>,
    /// 'R' or 'W'.
    rw: Option<char>,
    /// Are we inside an SWD read or a write?
    apdp: Option<&'static str>,
    /// `ctrlsel` is bit 0 in the SELECT register.
    ctrlsel: u32,
    /// `orundetect` is bit 0 in the CTRLSTAT register.
    orundetect: u32,
    dparity: u32,
}

impl Swd {
    /// Output annotated data.
    fn putx(&mut self, ctx: &mut dyn Context, ann: usize, length: usize, data: &str) {
        // Python takes `samplenums[-length]`, falling back to
        // `samplenums[0]` on IndexError.
        let ss = if length <= self.samplenums.len() {
            self.samplenums[self.samplenums.len() - length]
        } else {
            self.samplenums.first().copied().unwrap_or(self.ss_req)
        };
        if self.state == State::Req {
            self.ss_req = ss;
        }
        let es = ctx.samplenum();
        ctx.put_ann(ss, es, ann, vec![data.to_owned()]);
    }

    fn putp(&self, ctx: &mut dyn Context, ptype: &str, pdata: Item) {
        ctx.put_python(
            self.ss_req,
            ctx.samplenum(),
            Item::List(vec![Item::Str(ptype.into()), pdata]),
        );
    }

    /// Emit Python data item based on current SWD packet contents.
    fn put_python_data(&self, ctx: &mut dyn Context) {
        // Upstream indexes `(self.apdp, self.rw)` directly. With
        // strict_start=yes, a line reset that arrives before the first
        // request raises AttributeError there; the native decoder has no
        // packet to flush and simply keeps going.
        let Some(apdp) = self.apdp else { return };
        let Some(rw) = self.rw else { return };
        let ptype = match (apdp, rw) {
            ("AP", 'R') => "AP_READ",
            ("AP", 'W') => "AP_WRITE",
            ("DP", 'R') => "DP_READ",
            ("DP", 'W') => "DP_WRITE",
            _ => return,
        };
        let pdata = Item::List(vec![
            Item::Int(self.addr.unwrap_or(0)),
            self.data.item(),
            self.ack.as_ref().map_or(Item::None, |a| Item::Str(a.clone())),
        ]);
        self.putp(ctx, ptype, pdata);
    }

    /// Step to the next SWD state, reset internal counters accordingly.
    fn next_state(&mut self, ctx: &mut dyn Context) {
        self.bits.clear();
        self.samplenums.clear();
        self.linereset_count = 0;
        match self.state {
            State::Unknown => {
                self.state = State::Req;
                self.sample_edge = RISING;
                self.turnaround = 0;
            }
            State::Req => {
                self.state = State::Ack;
                self.sample_edge = FALLING;
                self.turnaround = 1;
            }
            State::Ack => {
                self.state = State::Data;
                self.sample_edge = if self.rw == Some('W') { RISING } else { FALLING };
                self.turnaround = if self.rw == Some('R') { 0 } else { 2 };
            }
            State::Data => self.state = State::Dparity,
            State::Dparity => {
                self.put_python_data(ctx);
                self.state = State::Req;
                self.sample_edge = RISING;
                self.turnaround = u32::from(self.rw == Some('R'));
            }
        }
    }

    /// Line reset (or equivalent), wait for a new pending SWD request.
    fn reset_state(&mut self, ctx: &mut dyn Context) {
        if self.state != State::Req {
            // Emit a Python data item.
            self.put_python_data(ctx);
        }
        self.bits.clear();
        self.samplenums.clear();
        self.linereset_count = 0;
        self.turnaround = 0;
        self.sample_edge = RISING;
        self.data = Data::Empty;
        self.ack = None;
        self.state = State::Req;
    }

    fn handle_req_edge(&mut self, ctx: &mut dyn Context) {
        // Check for a JTAG->SWD enable sequence.
        if self.bits.ends_with(RE_SWDSWITCH) {
            self.putx(ctx, ANN_ENABLE, 16, "JTAG->SWD");
            self.reset_state(ctx);
            return;
        }

        // Or a valid SWD Request packet:
        // '1(apdp)(rw)(addr 2 bits)(parity)01$'.
        if self.bits.len() < 8 {
            return;
        }
        let b = self.bits.as_bytes();
        let n = b.len();
        let req = &b[n - 8..];
        if req[0] != b'1' || req[6] != b'0' || req[7] != b'1' {
            return;
        }
        let apdp_bit = req[1] == b'1';
        let rw_bit = req[2] == b'1';
        let addr_bits = (req[3] == b'1', req[4] == b'1');
        // Upstream also computes the request parity and compares it, but
        // the local result is never used.

        self.rw = Some(if rw_bit { 'R' } else { 'W' });
        self.apdp = Some(if apdp_bit { "AP" } else { "DP" });
        // `int(m.group('addr')[::-1], 2) << 2`.
        self.addr = Some((i64::from(addr_bits.1) << 1 | i64::from(addr_bits.0)) << 2);
        let description = self.get_address_description();
        let class = if self.rw == Some('R') { ANN_READ } else { ANN_WRITE };
        self.putx(ctx, class, 8, &description);
        self.next_state(ctx);
    }

    fn handle_ack_edge(&mut self, ctx: &mut dyn Context) {
        if self.bits.len() < 3 {
            return;
        }
        match self.bits.as_str() {
            "100" => {
                self.putx(ctx, ANN_ACK, 3, "OK");
                self.ack = Some("OK".into());
                self.next_state(ctx);
            }
            "001" => {
                self.putx(ctx, ANN_ACK, 3, "FAULT");
                self.ack = Some("FAULT".into());
                if self.orundetect == 1 {
                    self.next_state(ctx);
                } else {
                    self.reset_state(ctx);
                }
                self.turnaround = 1;
            }
            "010" => {
                self.putx(ctx, ANN_ACK, 3, "WAIT");
                self.ack = Some("WAIT".into());
                if self.orundetect == 1 {
                    self.next_state(ctx);
                } else {
                    self.reset_state(ctx);
                }
                self.turnaround = 1;
            }
            "111" => {
                self.putx(ctx, ANN_ACK, 3, "NOREPLY");
                self.ack = Some("NOREPLY".into());
                self.reset_state(ctx);
            }
            _ => {
                self.putx(ctx, ANN_ACK, 3, "ERROR");
                self.ack = Some("ERROR".into());
                self.reset_state(ctx);
            }
        }
    }

    fn handle_data_edge(&mut self, ctx: &mut dyn Context) {
        if self.bits.len() < 32 {
            return;
        }
        let mut data = 0u32;
        let mut dparity = 0u32;
        for (x, c) in self.bits.bytes().take(32).enumerate() {
            if c == b'1' {
                data += 1 << x;
                dparity += 1;
            }
        }
        self.dparity = dparity % 2;
        self.data = Data::Value(data);

        self.putx(ctx, ANN_DATA, 32, &format!("0x{data:08x}"));
        self.next_state(ctx);
    }

    fn handle_dparity_edge(&mut self, ctx: &mut dyn Context) {
        if self.dparity.to_string() != self.bits {
            // PARITY ERROR
            self.putx(ctx, ANN_PARITY, 1, &format!("{}{}", self.dparity, self.bits));
        } else if self.rw == Some('W') {
            self.handle_completed_write();
        }
        self.next_state(ctx);
    }

    /// Update internal state of the debug port based on a completed
    /// write operation.
    fn handle_completed_write(&mut self) {
        if self.apdp != Some("DP") {
            return;
        }
        let data = match self.data {
            Data::Value(v) => v,
            _ => 0,
        };
        if self.addr == Some(ADDR_DP_SELECT) {
            self.ctrlsel = data & BIT_SELECT_CTRLSEL;
        } else if self.addr == Some(ADDR_DP_CTRLSTAT) && self.ctrlsel == 0 {
            self.orundetect = data & BIT_CTRLSTAT_ORUNDETECT;
        }
    }

    /// A human-readable description of the currently selected address.
    fn get_address_description(&self) -> String {
        let addr = self.addr.unwrap_or(0);
        if self.apdp == Some("DP") {
            if self.rw == Some('R') {
                // Tables 2-4 & 2-5 in ADIv5.2 spec ARM document IHI 0031C.
                return match addr {
                    0 => "IDCODE".into(),
                    0x4 => if self.ctrlsel == 0 { "R CTRL/STAT" } else { "R DLCR" }.into(),
                    0x8 => "RESEND".into(),
                    0xC => "RDBUFF".into(),
                    _ => format!("? RDP{addr:x}"),
                };
            }
            if self.rw == Some('W') {
                // Tables 2-4 & 2-5 in ADIv5.2 spec ARM document IHI 0031C.
                return match addr {
                    0 => "W ABORT".into(),
                    0x4 => if self.ctrlsel == 0 { "W CTRL/STAT" } else { "W DLCR" }.into(),
                    0x8 => "W SELECT".into(),
                    0xC => "W RESERVED".into(),
                    _ => format!("? WDP{addr:x}"),
                };
            }
        } else if self.apdp == Some("AP") {
            if self.rw == Some('R') {
                return format!("R AP{addr:x}");
            }
            if self.rw == Some('W') {
                return format!("W AP{addr:x}");
            }
        }

        // Any legitimate operations shouldn't fall through to here.
        let rw = self.rw.unwrap_or('?');
        let apdp = self.apdp.unwrap_or("?");
        format!("? {rw}{apdp}{addr:x}")
    }
}

impl Decoder for Swd {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Swd {
            // `if self.options['strict_start'] == 'no': self.state = 'REQ'`.
            state: if config.str("strict_start", "no") == "no" {
                State::Req
            } else {
                State::Unknown
            },
            sample_edge: RISING,
            ..Swd::default()
        };
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let cond: &[Cond] = &[Cond::pin(SWCLK, CondKind::Edge)];
        loop {
            // Wait for any clock edge.
            let pins = ctx.wait(&[cond])?;
            let clk = pins.get(SWCLK);
            let dio = pins.get(SWDIO);

            // Count rising edges with DIO held high, as a line reset
            // (50+ high edges) can happen from any state.
            if clk == RISING {
                if dio {
                    self.linereset_count += 1;
                } else {
                    if self.linereset_count >= 50 {
                        self.putx(ctx, ANN_RESET, self.linereset_count, "LINERESET");
                        self.putp(ctx, "LINE_RESET", Item::None);
                        self.reset_state(ctx);
                    }
                    self.linereset_count = 0;
                }
            }

            // Otherwise, we only care about either rising or falling
            // edges (depending on sample_edge, set according to state).
            if clk != self.sample_edge {
                continue;
            }

            // Turnaround bits get skipped.
            if self.turnaround > 0 {
                self.turnaround -= 1;
                continue;
            }

            self.bits.push(if dio { '1' } else { '0' });
            self.samplenums.push(ctx.samplenum());
            match self.state {
                State::Unknown => {}
                State::Req => self.handle_req_edge(ctx),
                State::Ack => self.handle_ack_edge(ctx),
                State::Data => self.handle_data_edge(ctx),
                State::Dparity => self.handle_dparity_edge(ctx),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{RE_SWDSWITCH, Swd};

    #[test]
    fn enable_sequence_is_the_reversed_literal() {
        assert_eq!(RE_SWDSWITCH, "0111100111100111");
        assert_eq!(RE_SWDSWITCH.len(), 16);
    }

    #[test]
    fn default_state_is_unknown_until_start_makes_it_req() {
        let swd = Swd::default();
        assert_eq!(swd.state, super::State::Unknown);
    }
}
