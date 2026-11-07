// Counterpart in sigrok:
//   libsigrokdecode/decoders/swim/pd.py,
//   Copyright (C) 2018 Mike Jagdis <mjagdis@eris-associates.co.uk>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the SWIM protocol and the decoder's documented
//   classes and texts.

//! SWIM: the STM8 single wire interface, bit times from pulse widths.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const SWIM: usize = 0;

// Ann.
const ANN_BIT: usize = 0;
const ANN_ENTERSEQ: usize = 1;
const ANN_PARITY: usize = 4;
const ANN_ACK: usize = 5;
const ANN_NACK: usize = 6;

// Binary classes.
const BIN_TX: usize = 0;
const BIN_RX: usize = 1;

// "SWIM clock for the target is normally HSI/2 where HSI is 8MHz +- 5%."
const HSI: f64 = 8_000_000.0;
const HSI_MIN: f64 = HSI * 0.9;
const HSI_MAX: f64 = HSI * 1.1;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "debug",
    desc: "Debug",
    default: OptionValue::s("no"),
    values: &[OptionValue::s("yes"), OptionValue::s("no")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "swim",
    name: "SWIM",
    longname: "STM8 SWIM bus",
    desc: "STM8 Single Wire Interface Module (SWIM) protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[ChannelDesc { id: "swim", name: "SWIM", desc: "SWIM data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "enterseq", desc: "SWIM enter sequence" },
        AnnClass { id: "start-host", desc: "Start bit (host)" },
        AnnClass { id: "start-target", desc: "Start bit (target)" },
        AnnClass { id: "parity", desc: "Parity bit" },
        AnnClass { id: "ack", desc: "Acknowledgement" },
        AnnClass { id: "nack", desc: "Negative acknowledgement" },
        AnnClass { id: "byte-write", desc: "Byte write" },
        AnnClass { id: "byte-read", desc: "Byte read" },
        AnnClass { id: "cmd-unknown", desc: "Unknown SWIM command" },
        AnnClass { id: "cmd", desc: "SWIM command" },
        AnnClass { id: "bytes", desc: "Byte count" },
        AnnClass { id: "address", desc: "Address" },
        AnnClass { id: "data-write", desc: "Data write" },
        AnnClass { id: "data-read", desc: "Data read" },
        AnnClass { id: "debug", desc: "Debug" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[ANN_BIT] },
        AnnRow {
            id: "framing",
            desc: "Framing",
            classes: &[2, 3, ANN_PARITY, ANN_ACK, ANN_NACK, 7, 8],
        },
        AnnRow {
            id: "protocol",
            desc: "Protocol",
            classes: &[ANN_ENTERSEQ, 9, 10, 11, 12, 13, 14],
        },
        AnnRow { id: "debug", desc: "Debug", classes: &[15] },
    ],
    binary: &[
        BinClass { id: "tx", desc: "Dump of data written to target" },
        BinClass { id: "rx", desc: "Dump of data read from target" },
    ],
    doc: "SWIM is a single wire interface for STM8 series 8-bit \
microcontrollers that allows non-intrusive read/wite access to be performed \
on-the-fly to the memory and registers of the MCU for debug and flashing \
purposes.\n\n\
See the STMicroelectronics document UM0470 for details.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Swim::default())
}

fn texts(t: &[&str]) -> Vec<String> {
    t.iter().map(|s| (*s).to_owned()).collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ProtoState {
    Cmd,
    N,
    AtE,
    AtH,
    AtL,
    Data,
}

/// The SWIM decoder.
#[allow(clippy::struct_field_names)]
pub struct Swim {
    samplerate: u64,
    swim_clock: f64,

    eseq_edge: [(i32, Option<u64>); 2],
    eseq_pairnum: u32,
    eseq_pairstart: Option<u64>,
    eseq_reflen: f64,

    bit_edge: [(i32, Option<u64>); 2],
    bit_maxlen: i64,
    bit_reflen: i64,
    bitseq_len: usize,
    bitseq_end: u64,
    proto_state: ProtoState,
    bitseq_value: u64,
    bitseq_dir: i32,
    bitseq_start: u64,

    proto_byte_count: i64,
    proto_addr: u64,
    proto_addr_start: u64,

    debug: bool,
    sync_reflen_min: i64,
    sync_reflen_max: i64,
}

impl Default for Swim {
    fn default() -> Self {
        Swim {
            samplerate: 0,
            swim_clock: HSI_MIN / 2.0,
            eseq_edge: [(-1, None), (-1, None)],
            eseq_pairnum: 0,
            eseq_pairstart: None,
            eseq_reflen: 0.0,
            bit_edge: [(-1, None), (-1, None)],
            bit_maxlen: -1,
            bit_reflen: 0,
            bitseq_len: 0,
            bitseq_end: 0,
            proto_state: ProtoState::Cmd,
            bitseq_value: 0,
            bitseq_dir: 0,
            bitseq_start: 0,
            proto_byte_count: 0,
            proto_addr: 0,
            proto_addr_start: 0,
            debug: false,
            sync_reflen_min: 0,
            sync_reflen_max: 0,
        }
    }
}

impl Swim {
    /// The Python decoder's `reset()`.
    fn reset(&mut self) {
        self.bit_edge = [(-1, None), (-1, None)];
        self.bit_maxlen = -1;
        self.bitseq_len = 0;
        self.bitseq_end = 0;
        self.proto_state = ProtoState::Cmd;
    }

    /// The Python decoder's `adjust_timings()`.
    fn adjust_timings(&mut self) {
        // "A low-speed bit is 22 SWIM clocks long."
        self.bit_reflen = (self.samplerate as f64 * 22.0 / self.swim_clock).ceil() as i64;
    }

    /// The timing constants `start()` derives from the samplerate.
    fn init_timings(&mut self) {
        // "A synchronization frame is a low that lasts for more than 64
        // but no more than 128 SWIM clock periods based on the standard
        // SWIM clock."
        self.sync_reflen_min = (self.samplerate as f64 * 64.0 / HSI_MAX).floor() as i64;
        self.sync_reflen_max = (self.samplerate as f64 * 128.0 / (HSI_MIN / 2.0)).ceil() as i64;
        // "The SWIM entry sequence is 4 pulses at 2kHz followed by 4 at 1kHz."
        self.eseq_reflen = (self.samplerate as f64 / 2048.0).ceil();
        self.adjust_timings();
    }

    /// The Python decoder's `protocol()`.
    #[allow(clippy::too_many_lines)]
    fn protocol(&mut self, ctx: &mut dyn Context) {
        match self.proto_state {
            ProtoState::Cmd => match self.bitseq_value {
                0x00 => ctx.put_ann(
                    self.bitseq_start,
                    self.bitseq_end,
                    10,
                    texts(&["system reset", "SRST", "!"]),
                ),
                0x01 => {
                    self.proto_state = ProtoState::N;
                    ctx.put_ann(
                        self.bitseq_start,
                        self.bitseq_end,
                        10,
                        texts(&["read on-the-fly", "ROTF", "r"]),
                    );
                }
                0x02 => {
                    self.proto_state = ProtoState::N;
                    ctx.put_ann(
                        self.bitseq_start,
                        self.bitseq_end,
                        10,
                        texts(&["write on-the-fly", "WOTF", "w"]),
                    );
                }
                _ => ctx.put_ann(self.bitseq_start, self.bitseq_end, 9, texts(&["unknown", "UNK"])),
            },
            ProtoState::N => {
                // "Number of bytes."
                self.proto_byte_count = self.bitseq_value as i64;
                self.proto_state = ProtoState::AtE;
                let v = self.bitseq_value;
                ctx.put_ann(
                    self.bitseq_start,
                    self.bitseq_end,
                    11,
                    vec![
                        format!("byte count 0x{v:02x}"),
                        format!("bytes 0x{v:02x}"),
                        format!("0x{v:02x}"),
                        format!("{v:02x}"),
                        format!("{v:x}"),
                    ],
                );
            }
            ProtoState::AtE => {
                // "Address byte 1."
                self.proto_addr = self.bitseq_value;
                self.proto_addr_start = self.bitseq_start;
                self.proto_state = ProtoState::AtH;
            }
            ProtoState::AtH => {
                // "Address byte 2."
                self.proto_addr = (self.proto_addr << 8) | self.bitseq_value;
                self.proto_state = ProtoState::AtL;
            }
            ProtoState::AtL => {
                // "Address byte 3."
                self.proto_addr = (self.proto_addr << 8) | self.bitseq_value;
                self.proto_state = ProtoState::Data;
                let v = self.proto_addr;
                ctx.put_ann(
                    self.proto_addr_start,
                    self.bitseq_end,
                    12,
                    vec![
                        format!("address 0x{v:06x}"),
                        format!("addr 0x{v:06x}"),
                        format!("0x{v:06x}"),
                        format!("{v:06x}"),
                        format!("{v:x}"),
                    ],
                );
            }
            ProtoState::Data => {
                if self.proto_byte_count > 0 {
                    self.proto_byte_count -= 1;
                    if self.proto_byte_count == 0 {
                        self.proto_state = ProtoState::Cmd;
                    }
                }

                let v = self.bitseq_value & 0xff;
                ctx.put_ann(
                    self.bitseq_start,
                    self.bitseq_end,
                    (13 + self.bitseq_dir) as usize,
                    vec![format!("0x{v:02x}"), format!("{v:02x}"), format!("{v:x}")],
                );
                ctx.put_binary(
                    self.bitseq_start,
                    self.bitseq_end,
                    if self.bitseq_dir == 0 { BIN_TX } else { BIN_RX },
                    vec![v as u8],
                );
                if self.debug {
                    let count = self.proto_byte_count;
                    ctx.put_ann(
                        self.bitseq_start,
                        self.bitseq_end,
                        15,
                        vec![format!("{count} more"), format!("{count}")],
                    );
                }
            }
        }
    }

    /// The Python decoder's `bitseq()`.
    fn bitseq(&mut self, ctx: &mut dyn Context, bitstart: u64, bitend: u64, bit: i32) {
        if self.bitseq_len == 0 {
            // "Looking for start of a bit sequence (command or byte)."
            self.bit_reflen = (bitend - bitstart) as i64;
            self.bitseq_value = 0;
            self.bitseq_dir = bit;
            self.bitseq_len = 1;
            ctx.put_ann(bitstart, bitend, (2 + self.bitseq_dir) as usize, texts(&["start", "s"]));
        } else if (self.proto_state == ProtoState::Cmd && self.bitseq_len == 4)
            || (self.proto_state != ProtoState::Cmd && self.bitseq_len == 9)
        {
            // "Parity bit."
            self.bitseq_end = bitstart;
            self.bitseq_len += 1;

            ctx.put_ann(bitstart, bitend, ANN_PARITY, texts(&["parity", "par", "p"]));

            // "The start bit is not data but was used for parity
            // calculation."
            self.bitseq_value &= 0xff;
            let v = self.bitseq_value;
            ctx.put_ann(
                self.bitseq_start,
                self.bitseq_end,
                (7 + self.bitseq_dir) as usize,
                vec![format!("0x{v:02x}"), format!("{v:02x}"), format!("{v:x}")],
            );
        } else if (self.proto_state == ProtoState::Cmd && self.bitseq_len == 5)
            || (self.proto_state != ProtoState::Cmd && self.bitseq_len == 10)
        {
            // "ACK/NACK bit."
            if bit != 0 {
                ctx.put_ann(bitstart, bitend, ANN_ACK, texts(&["ack", "a"]));
            } else {
                ctx.put_ann(bitstart, bitend, ANN_NACK, texts(&["nack", "n"]));
            }

            // "We only pass data that was ack'd up the stack."
            if bit != 0 {
                self.protocol(ctx);
            }

            self.bitseq_len = 0;
        } else {
            if self.bitseq_len == 1 {
                self.bitseq_start = bitstart;
            }
            self.bitseq_value = (self.bitseq_value << 1) | (bit as u64 & 1);
            self.bitseq_len += 1;
        }
    }

    /// The Python decoder's `bit()`.
    fn bit(&mut self, ctx: &mut dyn Context, start: u64, mid: u64, end: u64) {
        let bit = i32::from(mid - start < end - mid);
        ctx.put_ann(start, end, ANN_BIT, vec![bit.to_string()]);
        self.bitseq(ctx, start, end, bit);
    }

    /// The Python decoder's `detect_synchronize_frame()`.
    fn detect_synchronize_frame(&mut self, ctx: &mut dyn Context, start: u64) {
        let len = ctx.samplenum() - start;
        if (len as i64) >= self.sync_reflen_min && (len as i64) <= self.sync_reflen_max {
            ctx.put_ann(
                start,
                ctx.samplenum(),
                ANN_ENTERSEQ,
                texts(&["synchronization frame", "synchronization", "sync", "s"]),
            );

            // "A low that lasts for more than 64 SWIM clock periods causes
            // a reset of the SWIM communication state machine."
            self.reset();

            // "The low SHOULD last 128 SWIM clocks. This is used to
            // resynchronize in order to allow for variation in the
            // frequency of the internal RC oscillator."
            self.swim_clock = 128.0 * (self.samplerate as f64 / len as f64);
            self.adjust_timings();
        }
    }

    /// The Python decoder's `eseq_potential_start()`.
    fn eseq_potential_start(&mut self, start: u64, end: u64) {
        self.eseq_pairstart = Some(start);
        self.eseq_reflen = (end - start) as f64;
        self.eseq_pairnum = 1;
    }

    /// The Python decoder's `detect_enter_sequence()`.
    fn detect_enter_sequence(&mut self, ctx: &mut dyn Context, start: u64, end: u64) {
        // "According to the spec the enter sequence is four pulses at
        // 2kHz followed by four at 1kHz. We do not check the frequency but
        // simply check the lengths of successive pulses against the first."
        if self.eseq_pairnum == 0 || (self.eseq_reflen - (end - start) as f64).abs() > 2.0 {
            self.eseq_potential_start(start, end);
        } else if self.eseq_pairnum < 4 {
            // "The next three pulses should be the same length as the first."
            self.eseq_pairnum += 1;

            if self.eseq_pairnum == 4 {
                self.eseq_reflen /= 2.0;
            }
        } else {
            // "The final four pulses should each be half the length of the
            // initial pair."
            self.eseq_pairnum += 1;
            if self.eseq_pairnum == 8 {
                // "Four matching pulses followed by four more that match
                // each other but are half the length of the first 4."
                ctx.put_ann(
                    self.eseq_pairstart.unwrap_or(start),
                    end,
                    ANN_ENTERSEQ,
                    texts(&["enter sequence", "enter seq", "enter", "ent", "e"]),
                );
                self.eseq_pairnum = 0;
            }
        }
    }
}

impl Decoder for Swim {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Swim::default();
        self.debug = config.str("debug", "no") == "yes";
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        if samplerate > 0 {
            self.init_timings();
        }
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        if self.sync_reflen_max == 0 {
            self.init_timings();
        }
        loop {
            let swim_pins;
            let mut swim: i32;
            if self.bit_maxlen >= 0 {
                swim_pins = ctx.wait(&[])?;
                swim = i32::from(swim_pins.get(SWIM));
                self.bit_maxlen -= 1;
            } else {
                swim_pins = ctx.wait(&[&[Cond::pin(SWIM, CondKind::Edge)]])?;
                swim = i32::from(swim_pins.get(SWIM));
            }

            if swim != self.eseq_edge[1].0 {
                if swim == 1
                    && let Some(start) = self.eseq_edge[1].1
                {
                    let now = ctx.samplenum();
                    self.detect_synchronize_frame(ctx, start);
                    if let Some(previous) = self.eseq_edge[0].1 {
                        self.detect_enter_sequence(ctx, previous, now);
                    }
                }
                self.eseq_edge[0] = self.eseq_edge[1];
                self.eseq_edge[1] = (swim, Some(ctx.samplenum()));
            }

            if (swim != self.bit_edge[1].0 && (swim != 1 || self.bit_edge[1].0 != -1))
                || self.bit_maxlen == 0
            {
                if self.bit_maxlen == 0 && self.bit_edge[1].0 == 1 {
                    swim = -1;
                }

                if self.bit_edge[1].0 != 0 && swim == 0 {
                    self.bit_maxlen = self.bit_reflen;
                }

                if self.bit_edge[0].0 == 0
                    && self.bit_edge[1].0 == 1
                    && ctx.samplenum() - self.bit_edge[0].1.unwrap_or(ctx.samplenum())
                        <= (self.bit_reflen + 2) as u64
                {
                    let start = self.bit_edge[0].1.unwrap_or(ctx.samplenum());
                    let mid = self.bit_edge[1].1.unwrap_or(ctx.samplenum());
                    let now = ctx.samplenum();
                    self.bit(ctx, start, mid, now);
                }

                self.bit_edge[0] = self.bit_edge[1];
                self.bit_edge[1] = (swim, Some(ctx.samplenum()));
            }
        }
    }
}
