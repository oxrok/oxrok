// Counterpart in sigrok:
//   libsigrokdecode/decoders/uart/pd.py,
//   Copyright (C) 2011-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig and others (see the libsigrokdecode
//   history).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the UART framing and the decoder's documented classes and texts.

//! UART: start bit, 5 to 9 data bits, optional parity, stop bits.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const RX: usize = 0;
const TX: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_DATA: usize = 0;
const ANN_START: usize = 2;
const ANN_PARITY_OK: usize = 4;
const ANN_PARITY_ERR: usize = 6;
// Upstream tags stop bits with class 4 (the parity-ok class) and -A
// selections rely on that, so the same class is used here.
const ANN_STOP: usize = 4;
const ANN_WARN: usize = 10;
const ANN_BITS: usize = 12;
const ANN_BREAK: usize = 14;
const ANN_PACKET: usize = 16;
const BIN_RXTX: usize = 2;

static OPTIONS: [OptionDesc; 12] = [
    OptionDesc { id: "baudrate", desc: "Baud rate", default: OptionValue::i(115_200), values: &[] },
    OptionDesc {
        id: "data_bits",
        desc: "Data bits",
        default: OptionValue::i(8),
        values: &[
            OptionValue::i(5),
            OptionValue::i(6),
            OptionValue::i(7),
            OptionValue::i(8),
            OptionValue::i(9),
        ],
    },
    OptionDesc {
        id: "parity",
        desc: "Parity",
        default: OptionValue::s("none"),
        values: &[
            OptionValue::s("none"),
            OptionValue::s("odd"),
            OptionValue::s("even"),
            OptionValue::s("zero"),
            OptionValue::s("one"),
            OptionValue::s("ignore"),
        ],
    },
    OptionDesc {
        id: "stop_bits",
        desc: "Stop bits",
        default: OptionValue::f(1.0),
        values: &[
            OptionValue::f(0.0),
            OptionValue::f(0.5),
            OptionValue::f(1.0),
            OptionValue::f(1.5),
        ],
    },
    OptionDesc {
        id: "bit_order",
        desc: "Bit order",
        default: OptionValue::s("lsb-first"),
        values: &[OptionValue::s("lsb-first"), OptionValue::s("msb-first")],
    },
    OptionDesc {
        id: "format",
        desc: "Data format",
        default: OptionValue::s("hex"),
        values: &[
            OptionValue::s("ascii"),
            OptionValue::s("dec"),
            OptionValue::s("hex"),
            OptionValue::s("oct"),
            OptionValue::s("bin"),
        ],
    },
    OptionDesc {
        id: "invert_rx",
        desc: "Invert RX",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("yes"), OptionValue::s("no")],
    },
    OptionDesc {
        id: "invert_tx",
        desc: "Invert TX",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("yes"), OptionValue::s("no")],
    },
    OptionDesc {
        id: "rx_packet_delim",
        desc: "RX packet delimiter (decimal)",
        default: OptionValue::i(-1),
        values: &[],
    },
    OptionDesc {
        id: "tx_packet_delim",
        desc: "TX packet delimiter (decimal)",
        default: OptionValue::i(-1),
        values: &[],
    },
    OptionDesc {
        id: "rx_packet_len",
        desc: "RX packet length",
        default: OptionValue::i(-1),
        values: &[],
    },
    OptionDesc {
        id: "tx_packet_len",
        desc: "TX packet length",
        default: OptionValue::i(-1),
        values: &[],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "uart",
    name: "UART",
    longname: "Universal Asynchronous Receiver/Transmitter",
    desc: "Asynchronous, serial bus.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["uart"],
    tags: &["Embedded/industrial"],
    channels: &[],
    optional_channels: &[
        ChannelDesc { id: "rx", name: "RX", desc: "UART receive line" },
        ChannelDesc { id: "tx", name: "TX", desc: "UART transmit line" },
    ],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "rx-data", desc: "RX data" },
        AnnClass { id: "tx-data", desc: "TX data" },
        AnnClass { id: "rx-start", desc: "RX start bits" },
        AnnClass { id: "tx-start", desc: "TX start bits" },
        AnnClass { id: "rx-parity-ok", desc: "RX parity OK bits" },
        AnnClass { id: "tx-parity-ok", desc: "TX parity OK bits" },
        AnnClass { id: "rx-parity-err", desc: "RX parity error bits" },
        AnnClass { id: "tx-parity-err", desc: "TX parity error bits" },
        AnnClass { id: "rx-stop", desc: "RX stop bits" },
        AnnClass { id: "tx-stop", desc: "TX stop bits" },
        AnnClass { id: "rx-warnings", desc: "RX warnings" },
        AnnClass { id: "tx-warnings", desc: "TX warnings" },
        AnnClass { id: "rx-data-bits", desc: "RX data bits" },
        AnnClass { id: "tx-data-bits", desc: "TX data bits" },
        AnnClass { id: "rx-break", desc: "RX break" },
        AnnClass { id: "tx-break", desc: "TX break" },
        AnnClass { id: "rx-packet", desc: "RX packet" },
        AnnClass { id: "tx-packet", desc: "TX packet" },
    ],
    annotation_rows: &[
        AnnRow { id: "rx-data-bits", desc: "RX bits", classes: &[12] },
        AnnRow { id: "rx-data", desc: "RX", classes: &[0, 2, 4, 6, 8] },
        AnnRow { id: "rx-warnings", desc: "RX warnings", classes: &[10] },
        AnnRow { id: "rx-break", desc: "RX break", classes: &[14] },
        AnnRow { id: "rx-packets", desc: "RX packets", classes: &[16] },
        AnnRow { id: "tx-data-bits", desc: "TX bits", classes: &[13] },
        AnnRow { id: "tx-data", desc: "TX", classes: &[1, 3, 5, 7, 9] },
        AnnRow { id: "tx-warnings", desc: "TX warnings", classes: &[11] },
        AnnRow { id: "tx-break", desc: "TX break", classes: &[15] },
        AnnRow { id: "tx-packets", desc: "TX packets", classes: &[17] },
    ],
    binary: &[
        BinClass { id: "rx", desc: "RX dump" },
        BinClass { id: "tx", desc: "TX dump" },
        BinClass { id: "rxtx", desc: "RX/TX dump" },
    ],
    doc: "UART (Universal Asynchronous Receiver Transmitter) is a simple serial \
communication protocol which allows two devices to talk to each other.\n\n\
This decoder works on all UART-like asynchronous protocols with one start \
bit (0), 5 to 9 data bits, an optional parity bit, and one or more stop \
bits (1), in this order. It can run on one line (RX or TX) or on both.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Uart::default())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    WaitStart,
    GetStart,
    GetData,
    GetParity,
    GetStop,
}

/// Per direction state.
#[derive(Clone, Debug)]
struct Line {
    state: State,
    frame_start: u64,
    frame_valid: bool,
    cur_bit: u32,
    value: u32,
    bits: Vec<(u8, u64, u64)>,
    startsample: Option<u64>,
    break_start: Option<u64>,
    idle_start: Option<u64>,
    packet: Vec<u32>,
    ss_packet: u64,
    es_packet: u64,
}

impl Default for Line {
    fn default() -> Self {
        Line {
            state: State::WaitStart,
            frame_start: 0,
            frame_valid: true,
            cur_bit: 0,
            value: 0,
            bits: Vec::new(),
            startsample: None,
            break_start: None,
            idle_start: None,
            packet: Vec::new(),
            ss_packet: 0,
            es_packet: 0,
        }
    }
}

/// The parsed options.
#[derive(Clone, Debug)]
struct Opts {
    baudrate: f64,
    data_bits: u32,
    parity: String,
    stop_bits: f64,
    msb_first: bool,
    format: String,
    invert: [bool; 2],
    packet_delim: [i64; 2],
    packet_len: [i64; 2],
}

/// The UART decoder.
#[derive(Default)]
pub struct Uart {
    opts: Option<Opts>,
    samplerate: u64,
    bit_width: f64,
    frame_len: u64,
    lines: [Line; 2],
}

impl Uart {
    fn opts(&self) -> &Opts {
        self.opts.as_ref().expect("started")
    }

    /// Sample number in the middle of bit `bitnum` of the current frame
    /// (0 = start bit).
    fn sample_point(&self, rxtx: usize, bitnum: u32) -> f64 {
        self.lines[rxtx].frame_start as f64
            + (self.bit_width - 1.0) / 2.0
            + f64::from(bitnum) * self.bit_width
    }

    fn wait_cond(&self, rxtx: usize, inv: bool, samplenum: u64) -> Cond {
        let l = &self.lines[rxtx];
        let o = self.opts();
        let bitnum = match l.state {
            State::WaitStart => {
                return Cond::Pin(rxtx, if inv { CondKind::Rising } else { CondKind::Falling });
            }
            State::GetStart => 0,
            State::GetData => 1 + l.cur_bit,
            State::GetParity => 1 + o.data_bits,
            State::GetStop => 1 + o.data_bits + u32::from(o.parity != "none"),
        };
        let want = self.sample_point(rxtx, bitnum).ceil() as u64;
        Cond::Skip(want.saturating_sub(samplenum))
    }

    fn idle_cond(&self, rxtx: usize, samplenum: u64) -> Option<Cond> {
        let start = self.lines[rxtx].idle_start?;
        let end = start + self.frame_len;
        if end < samplenum {
            return None;
        }
        Some(Cond::Skip(end - samplenum))
    }

    fn half(&self) -> (u64, u64) {
        let h = self.bit_width / 2.0;
        (h.floor() as u64, h.ceil() as u64)
    }

    fn putx(&self, ctx: &mut dyn Context, rxtx: usize, class: usize, texts: Vec<String>) {
        let (lo, hi) = self.half();
        let s = self.lines[rxtx].startsample.unwrap_or(ctx.samplenum());
        ctx.put_ann(s.saturating_sub(lo), ctx.samplenum() + hi, class, texts);
    }

    fn putg(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        let (lo, hi) = self.half();
        let s = ctx.samplenum();
        ctx.put_ann(
            s.saturating_sub(lo),
            s + hi,
            class,
            texts.iter().map(|t| (*t).to_owned()).collect(),
        );
    }

    fn format_value(&self, v: u32) -> Option<String> {
        let o = self.opts();
        let bits = o.data_bits;
        Some(match o.format.as_str() {
            "ascii" => {
                if (32..=126).contains(&v) {
                    char::from(v as u8).to_string()
                } else if bits <= 8 {
                    format!("[{v:02X}]")
                } else {
                    format!("[{v:03X}]")
                }
            }
            "dec" => v.to_string(),
            "hex" => format!("{v:0width$X}", width = bits.div_ceil(4) as usize),
            "oct" => format!("{v:0width$o}", width = bits.div_ceil(3) as usize),
            "bin" => format!("{v:0width$b}", width = bits as usize),
            _ => return None,
        })
    }

    fn parity_ok(&self, parity_bit: u8, data: u32) -> bool {
        let o = self.opts();
        let ones = data.count_ones() + u32::from(parity_bit);
        match o.parity.as_str() {
            "ignore" => true,
            "zero" => parity_bit == 0,
            "one" => parity_bit == 1,
            "odd" => !ones.is_multiple_of(2),
            "even" => ones.is_multiple_of(2),
            _ => false,
        }
    }

    fn frame_item(&self, rxtx: usize) -> Item {
        let l = &self.lines[rxtx];
        Item::List(vec![
            Item::Str("FRAME".into()),
            Item::Int(rxtx as i64),
            Item::List(vec![Item::Int(i64::from(l.value)), Item::Bool(l.frame_valid)]),
        ])
    }

    #[allow(clippy::too_many_lines)]
    fn inspect_sample(&mut self, ctx: &mut dyn Context, rxtx: usize, mut signal: bool, inv: bool) {
        if inv {
            signal = !signal;
        }
        let s = ctx.samplenum();
        let (_, hi) = self.half();
        match self.lines[rxtx].state {
            State::WaitStart => {
                let l = &mut self.lines[rxtx];
                l.frame_start = s;
                l.frame_valid = true;
                l.state = State::GetStart;
            }
            State::GetStart => {
                if signal {
                    ctx.put_python(
                        s.saturating_sub(self.half().0),
                        s + hi,
                        Item::List(vec![
                            Item::Str("INVALID STARTBIT".into()),
                            Item::Int(rxtx as i64),
                            Item::Int(1),
                        ]),
                    );
                    self.putg(ctx, rxtx + ANN_WARN, &["Frame error", "Frame err", "FE"]);
                    self.lines[rxtx].frame_valid = false;
                    let item = self.frame_item(rxtx);
                    ctx.put_python(self.lines[rxtx].frame_start, s + hi, item);
                    self.lines[rxtx].state = State::WaitStart;
                    return;
                }
                let l = &mut self.lines[rxtx];
                l.cur_bit = 0;
                l.value = 0;
                l.startsample = None;
                ctx.put_python(
                    s.saturating_sub(self.half().0),
                    s + hi,
                    Item::List(vec![
                        Item::Str("STARTBIT".into()),
                        Item::Int(rxtx as i64),
                        Item::Int(0),
                    ]),
                );
                self.putg(ctx, rxtx + ANN_START, &["Start bit", "Start", "S"]);
                self.lines[rxtx].state = State::GetData;
            }
            State::GetData => self.get_data_bit(ctx, rxtx, signal),
            State::GetParity => {
                let bit = u8::from(signal);
                let ok = self.parity_ok(bit, self.lines[rxtx].value);
                if ok {
                    ctx.put_python(
                        s.saturating_sub(self.half().0),
                        s + hi,
                        Item::List(vec![
                            Item::Str("PARITYBIT".into()),
                            Item::Int(rxtx as i64),
                            Item::Int(i64::from(bit)),
                        ]),
                    );
                    self.putg(ctx, rxtx + ANN_PARITY_OK, &["Parity bit", "Parity", "P"]);
                } else {
                    ctx.put_python(
                        s.saturating_sub(self.half().0),
                        s + hi,
                        Item::List(vec![
                            Item::Str("PARITY ERROR".into()),
                            Item::Int(rxtx as i64),
                            Item::List(vec![Item::Int(0), Item::Int(1)]),
                        ]),
                    );
                    self.putg(ctx, rxtx + ANN_PARITY_ERR, &["Parity error", "Parity err", "PE"]);
                    self.lines[rxtx].frame_valid = false;
                }
                self.lines[rxtx].state = State::GetStop;
            }
            State::GetStop => {
                if !signal {
                    ctx.put_python(
                        s.saturating_sub(self.half().0),
                        s + hi,
                        Item::List(vec![
                            Item::Str("INVALID STOPBIT".into()),
                            Item::Int(rxtx as i64),
                            Item::Int(0),
                        ]),
                    );
                    self.putg(ctx, rxtx + ANN_WARN, &["Frame error", "Frame err", "FE"]);
                    self.lines[rxtx].frame_valid = false;
                }
                ctx.put_python(
                    s.saturating_sub(self.half().0),
                    s + hi,
                    Item::List(vec![
                        Item::Str("STOPBIT".into()),
                        Item::Int(rxtx as i64),
                        Item::Int(i64::from(signal)),
                    ]),
                );
                self.putg(ctx, rxtx + ANN_STOP, &["Stop bit", "Stop", "T"]);
                let item = self.frame_item(rxtx);
                let l = &mut self.lines[rxtx];
                ctx.put_python(l.frame_start, s + hi, item);
                l.state = State::WaitStart;
                l.idle_start = Some(l.frame_start + self.frame_len);
            }
        }
    }

    fn get_data_bit(&mut self, ctx: &mut dyn Context, rxtx: usize, signal: bool) {
        let s = ctx.samplenum();
        let o = self.opts().clone();
        if self.lines[rxtx].startsample.is_none() {
            self.lines[rxtx].startsample = Some(s);
        }
        self.putg(ctx, rxtx + ANN_BITS, &[if signal { "1" } else { "0" }]);
        let halfbit = (self.bit_width / 2.0) as u64;
        let l = &mut self.lines[rxtx];
        l.bits.push((u8::from(signal), s.saturating_sub(halfbit), s + halfbit));
        l.cur_bit += 1;
        if l.cur_bit < o.data_bits {
            return;
        }
        let mut bits: Vec<u8> = l.bits.iter().map(|b| b.0).collect();
        if o.msb_first {
            bits.reverse();
        }
        l.value = bits.iter().enumerate().fold(0u32, |v, (i, &b)| v | (u32::from(b) << i));
        let value = l.value;
        let bit_items: Vec<Item> = l
            .bits
            .iter()
            .map(|&(b, ss, es)| {
                Item::List(vec![
                    Item::Int(i64::from(b)),
                    Item::Int(ss as i64),
                    Item::Int(es as i64),
                ])
            })
            .collect();
        l.bits.clear();
        let (lo, hi) = self.half();
        let ss = self.lines[rxtx].startsample.unwrap_or(s).saturating_sub(lo);
        ctx.put_python(
            ss,
            s + hi,
            Item::List(vec![
                Item::Str("DATA".into()),
                Item::Int(rxtx as i64),
                Item::List(vec![Item::Int(i64::from(value)), Item::List(bit_items)]),
            ]),
        );
        if let Some(text) = self.format_value(value) {
            self.putx(ctx, rxtx, rxtx + ANN_DATA, vec![text]);
        }
        let bw = (o.data_bits as usize).div_ceil(8);
        let bytes = value.to_be_bytes()[4 - bw..].to_vec();
        ctx.put_binary(ss, s + hi, rxtx, bytes.clone());
        ctx.put_binary(ss, s + hi, BIN_RXTX, bytes);
        self.handle_packet(ctx, rxtx);
        self.lines[rxtx].state = if o.parity == "none" { State::GetStop } else { State::GetParity };
    }

    fn handle_packet(&mut self, ctx: &mut dyn Context, rxtx: usize) {
        let o = self.opts().clone();
        let delim = o.packet_delim[rxtx];
        let plen = o.packet_len[rxtx];
        if delim == -1 && plen == -1 {
            return;
        }
        let s = ctx.samplenum();
        let (lo, hi) = self.half();
        let l = &mut self.lines[rxtx];
        if l.packet.is_empty() {
            l.ss_packet = l.startsample.unwrap_or(s).saturating_sub(lo);
        }
        l.packet.push(l.value);
        let done = (plen > 0 && l.packet.len() as i64 >= plen)
            || (delim >= 0 && i64::from(l.value) == delim);
        if !done {
            return;
        }
        l.es_packet = s + hi;
        let (ss, es) = (l.ss_packet, l.es_packet);
        let values = std::mem::take(&mut l.packet);
        let mut text = String::new();
        for v in values {
            if let Some(t) = self.format_value(v) {
                text.push_str(&t);
                if o.format != "ascii" {
                    text.push(' ');
                }
            }
        }
        if o.format != "ascii" && text.ends_with(' ') {
            text.pop();
        }
        ctx.put_ann(ss, es, rxtx + ANN_PACKET, vec![text]);
    }

    fn inspect_edge(&mut self, ctx: &mut dyn Context, rxtx: usize, mut signal: bool, inv: bool) {
        if inv {
            signal = !signal;
        }
        let s = ctx.samplenum();
        if !signal {
            self.lines[rxtx].break_start = Some(s);
            return;
        }
        let Some(start) = self.lines[rxtx].break_start else { return };
        if s - start >= self.frame_len {
            let fs = self.lines[rxtx].frame_start;
            ctx.put_python(
                fs,
                s,
                Item::List(vec![Item::Str("BREAK".into()), Item::Int(rxtx as i64), Item::Int(0)]),
            );
            ctx.put_ann(
                fs,
                s,
                rxtx + ANN_BREAK,
                ["Break condition", "Break", "Brk", "B"].iter().map(|t| (*t).to_owned()).collect(),
            );
            self.lines[rxtx].state = State::WaitStart;
        }
        self.lines[rxtx].break_start = None;
    }

    fn inspect_idle(&mut self, ctx: &mut dyn Context, rxtx: usize, mut signal: bool, inv: bool) {
        if inv {
            signal = !signal;
        }
        let s = ctx.samplenum();
        if !signal {
            self.lines[rxtx].idle_start = None;
            return;
        }
        let start = *self.lines[rxtx].idle_start.get_or_insert(s);
        // A frame sets idle_start past its own end, so it can be ahead of s;
        // the Python decoder's difference is negative there.
        if s < start || s - start < self.frame_len {
            return;
        }
        ctx.put_python(
            start,
            s,
            Item::List(vec![Item::Str("IDLE".into()), Item::Int(rxtx as i64), Item::Int(0)]),
        );
        self.lines[rxtx].idle_start = Some(s);
    }
}

impl Decoder for Uart {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.opts = Some(Opts {
            baudrate: config.float("baudrate", 115_200.0),
            data_bits: config.int("data_bits", 8) as u32,
            parity: config.str("parity", "none").to_owned(),
            stop_bits: config.float("stop_bits", 1.0),
            msb_first: config.str("bit_order", "lsb-first") == "msb-first",
            format: config.str("format", "hex").to_owned(),
            invert: [
                config.str("invert_rx", "no") == "yes",
                config.str("invert_tx", "no") == "yes",
            ],
            packet_delim: [config.int("rx_packet_delim", -1), config.int("tx_packet_delim", -1)],
            packet_len: [config.int("rx_packet_len", -1), config.int("tx_packet_len", -1)],
        });
        self.lines = [Line::default(), Line::default()];
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        if let Some(o) = &self.opts {
            self.bit_width = samplerate as f64 / o.baudrate;
        }
    }

    #[allow(clippy::too_many_lines)]
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        let has = [ctx.has_channel(RX), ctx.has_channel(TX)];
        if !has[RX] && !has[TX] {
            return Err(Error::arg("Need at least one of TX or RX pins."));
        }
        let o = self.opts().clone();
        let inv = o.invert;
        let mut frame =
            1.0 + f64::from(o.data_bits) + f64::from(u8::from(o.parity != "none")) + o.stop_bits;
        frame *= self.bit_width;
        self.frame_len = frame.ceil() as u64;
        loop {
            let samplenum = ctx.samplenum();
            let mut conds: Vec<Vec<Cond>> = Vec::new();
            let mut data_idx = [None, None];
            let mut edge_idx = [None, None];
            let mut idle_idx = [None, None];
            for rxtx in [RX, TX] {
                if !has[rxtx] {
                    continue;
                }
                data_idx[rxtx] = Some(conds.len());
                conds.push(vec![self.wait_cond(rxtx, inv[rxtx], samplenum)]);
                edge_idx[rxtx] = Some(conds.len());
                conds.push(vec![Cond::Pin(rxtx, CondKind::Edge)]);
                if let Some(c) = self.idle_cond(rxtx, samplenum) {
                    idle_idx[rxtx] = Some(conds.len());
                    conds.push(vec![c]);
                }
            }
            let refs: Vec<&[Cond]> = conds.iter().map(Vec::as_slice).collect();
            let pins = ctx.wait(&refs)?;
            let matched = ctx.matched().to_vec();
            for rxtx in [RX, TX] {
                let level = pins.get(rxtx);
                if data_idx[rxtx].is_some_and(|i| matched[i]) {
                    self.inspect_sample(ctx, rxtx, level, inv[rxtx]);
                }
                if edge_idx[rxtx].is_some_and(|i| matched[i]) {
                    self.inspect_edge(ctx, rxtx, level, inv[rxtx]);
                    self.inspect_idle(ctx, rxtx, level, inv[rxtx]);
                }
                if idle_idx[rxtx].is_some_and(|i| matched[i]) {
                    self.inspect_idle(ctx, rxtx, level, inv[rxtx]);
                }
            }
        }
    }
}
