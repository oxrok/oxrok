// Counterpart in sigrok:
//   libsigrokdecode/decoders/avr_pdi/pd.py,
//   Copyright (C) 2011-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2016 Gerhard Sittig <gerhard.sittig@gmx.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the PDI framing and instruction set and the
//   decoder's documented classes and texts.

//! AVR PDI: UART frames of PDI instructions on RESET (clock) and DATA.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};
use std::fmt::Write as _;

const RESET: usize = 0;
const DATA: usize = 1;

// Ann.
const ANN_BIT: usize = 0;
const ANN_START: usize = 1;
const ANN_DATA: usize = 2;
const ANN_PARITY_OK: usize = 3;
const ANN_PARITY_ERR: usize = 4;
const ANN_STOP_OK: usize = 5;
const ANN_STOP_ERR: usize = 6;
const ANN_BREAK: usize = 7;
const ANN_OPCODE: usize = 8;
const ANN_DATA_PROG: usize = 9;
const ANN_DATA_DEV: usize = 10;
const ANN_COMMAND: usize = 14;

/// `BIN_BYTES`.
const BIN_BYTES: usize = 0;

// PDI instruction opcodes.
const OP_LDS: u8 = 0;
const OP_LD: u8 = 1;
const OP_STS: u8 = 2;
const OP_ST: u8 = 3;
const OP_LDCS: u8 = 4;
const OP_REPEAT: u8 = 5;
const OP_STCS: u8 = 6;
const OP_KEY: u8 = 7;

/// `PDI.pointer_format_nice`.
const POINTER_FORMAT_NICE: [&str; 4] = ["*(ptr)", "*(ptr++)", "ptr", "ptr++ (rsv)"];
/// `PDI.pointer_format_terse`.
const POINTER_FORMAT_TERSE: [&str; 4] = ["*p", "*p++", "p", "(rsv)"];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "avr_pdi",
    name: "AVR PDI",
    longname: "Atmel Program and Debug Interface",
    desc: "Atmel ATxmega Program and Debug Interface (PDI) protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[
        ChannelDesc { id: "reset", name: "RESET", desc: "RESET / PDI_CLK" },
        ChannelDesc { id: "data", name: "DATA", desc: "PDI_DATA" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "uart-bit", desc: "UART bit" },
        AnnClass { id: "start-bit", desc: "Start bit" },
        AnnClass { id: "data-bit", desc: "Data bit" },
        AnnClass { id: "parity-ok", desc: "Parity OK bit" },
        AnnClass { id: "parity-err", desc: "Parity error bit" },
        AnnClass { id: "stop-ok", desc: "Stop OK bit" },
        AnnClass { id: "stop-err", desc: "Stop error bit" },
        AnnClass { id: "break", desc: "BREAK condition" },
        AnnClass { id: "opcode", desc: "Instruction opcode" },
        AnnClass { id: "data-prog", desc: "Programmer data" },
        AnnClass { id: "data-dev", desc: "Device data" },
        AnnClass { id: "pdi-break", desc: "BREAK at PDI level" },
        AnnClass { id: "enable", desc: "Enable PDI" },
        AnnClass { id: "disable", desc: "Disable PDI" },
        AnnClass { id: "cmd-data", desc: "PDI command with data" },
    ],
    annotation_rows: &[
        AnnRow { id: "uart_bits", desc: "UART bits", classes: &[ANN_BIT] },
        AnnRow {
            id: "uart_fields",
            desc: "UART fields",
            classes: &[
                ANN_START,
                ANN_DATA,
                ANN_PARITY_OK,
                ANN_PARITY_ERR,
                ANN_STOP_OK,
                ANN_STOP_ERR,
                ANN_BREAK,
            ],
        },
        AnnRow {
            id: "pdi_fields",
            desc: "PDI fields",
            classes: &[ANN_OPCODE, ANN_DATA_PROG, ANN_DATA_DEV, 11],
        },
        AnnRow { id: "pdi_cmds", desc: "PDI Cmds", classes: &[12, 13, ANN_COMMAND] },
    ],
    binary: &[BinClass { id: "bytes", desc: "PDI protocol bytes" }],
    doc: "PDI (Program and Debug Interface) is an Atmel proprietary interface \
for external programming and on-chip debugging of the device.\n\n\
See the Atmel Application Note AVR1612 \"PDI programming driver\" and the \
\"Program and Debug Interface\" section in the Xmega A manual for details.\n\n\
The protocol uses two pins: the RESET pin and one dedicated DATA pin. \
The RESET pin provides a clock, the DATA pin communicates serial frames \
with a start bit, eight data bits, an even parity bit, and two stop bits. \
Data communication is bidirectional and half duplex, the device will \
provide response data after reception of a respective request.\n\n\
Protocol frames communicate opcodes and their arguments, which provides \
random and sequential access to the device's address space. By accessing \
the registers of internal peripherals, especially the NVM controller, \
it's possible to identify the device, read from and write to several \
kinds of memory (signature rows, fuses and lock bits, internal flash and \
EEPROM, memory mapped peripherals), and to control execution of software \
on the device.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(AvrPdi::default())
}

fn texts(t: &[&str]) -> Vec<String> {
    t.iter().map(|s| (*s).to_owned()).collect()
}

/// `PDI.ctrl_reg_name`, including its fallback.
fn ctrl_reg_name(reg_num: u8) -> String {
    match reg_num {
        0 => "status".into(),
        1 => "reset".into(),
        2 => "ctrl".into(),
        _ => format!("r{reg_num}"),
    }
}

/// Python's `int(text, 0)` for the hex data texts this decoder builds.
fn int_base_zero(text: &str) -> i64 {
    if let Some(rest) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        i64::from_str_radix(rest, 16).unwrap_or(0)
    } else if text.len() > 1 && text.starts_with('0') {
        i64::from_str_radix(&text[1..], 8).unwrap_or(0)
    } else {
        text.parse().unwrap_or(0)
    }
}

/// The Python decoder's `namedtuple('Bit', 'val ss es')`.
#[derive(Clone, Copy)]
struct Bit {
    val: Option<u8>,
    ss: u64,
    es: u64,
}

/// The AVR PDI decoder.
#[derive(Default)]
pub struct AvrPdi {
    // Bit times and sampled values.
    ss_last_fall: Option<u64>,
    data_sample: Option<u8>,
    ss_curr_fall: Option<u64>,
    // UART frame accumulation.
    bits: Vec<Bit>,
    zero_count: usize,
    zero_ss: Option<u64>,
    break_ss: Option<u64>,
    // Instruction accumulation.
    insn_rep_count: i64,
    insn_opcode: Option<u8>,
    insn_wr_counts: Vec<usize>,
    insn_rd_counts: Vec<usize>,
    insn_dat_bytes: Vec<u8>,
    insn_dat_count: usize,
    insn_ss_data: Option<u64>,
    cmd_ss: Option<u64>,
    cmd_insn_parts_nice: Vec<String>,
    cmd_insn_parts_terse: Vec<String>,
}

impl AvrPdi {
    fn clear_insn(&mut self) {
        self.insn_rep_count = 0;
        self.insn_opcode = None;
        self.insn_wr_counts = Vec::new();
        self.insn_rd_counts = Vec::new();
        self.insn_dat_bytes = Vec::new();
        self.insn_dat_count = 0;
        self.insn_ss_data = None;
        self.cmd_ss = None;
        self.cmd_insn_parts_nice = Vec::new();
        self.cmd_insn_parts_terse = Vec::new();
    }

    fn put_ann_bit(&self, ctx: &mut dyn Context, bit_nr: usize, ann_idx: usize) {
        let b = self.bits[bit_nr];
        let text = b.val.map_or_else(|| "None".to_owned(), |v| v.to_string());
        ctx.put_ann(b.ss, b.es, ann_idx, vec![text]);
    }

    fn put_uart_field(&self, ctx: &mut dyn Context, bit_nr: usize, ann: usize, text: &[&str]) {
        let b = self.bits[bit_nr];
        ctx.put_ann(b.ss, b.es, ann, texts(text));
    }

    /// The Python decoder's `handle_byte()`.
    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn handle_byte(&mut self, ctx: &mut dyn Context, ss: u64, es: u64, byteval: Option<u8>) {
        // "Handle BREAK conditions, which will abort any potentially
        // currently executing instruction."
        let is_break = byteval.is_none();
        if is_break {
            self.cmd_insn_parts_nice.push("BREAK".into());
            self.cmd_insn_parts_terse.push("BRK".into());
            self.insn_rep_count = 0;
            // "Will FALLTHROUGH to 'end of instruction' below."
        }

        // "Decode instruction opcodes and argument sizes from the first
        // byte of a transaction."
        if self.insn_opcode.is_none() && !is_break {
            let byteval = byteval.unwrap();
            let opcode = (byteval & 0xe0) >> 5;
            let arg30 = byteval & 0x0f;
            let arg32 = (byteval & 0x0c) >> 2;
            let arg10 = byteval & 0x03;
            self.insn_opcode = Some(opcode);
            self.cmd_ss = Some(ss);
            let mnemonics: Vec<String>;
            if opcode == OP_LDS {
                // "LDS: load data, direct addressing. Writes an address,
                // reads a data item."
                let width_addr = usize::from(arg32) + 1;
                let width_data = usize::from(arg10) + 1;
                self.insn_wr_counts = vec![width_addr];
                self.insn_rd_counts = vec![width_data];
                mnemonics = vec![
                    format!("Insn: LDS a{width_addr}, m{width_data}"),
                    format!("LDS a{width_addr}, m{width_data}"),
                    "LDS".into(),
                ];
                self.cmd_insn_parts_nice = vec!["LDS".into()];
                self.cmd_insn_parts_terse = vec!["LDS".into()];
            } else if opcode == OP_LD {
                // "LD: load data, indirect addressing. Reads a data
                // item, with optional repeat."
                let ptr_txt = POINTER_FORMAT_NICE[usize::from(arg32)];
                let ptr_txt_terse = POINTER_FORMAT_TERSE[usize::from(arg32)];
                let width_data = usize::from(arg10) + 1;
                self.insn_wr_counts = Vec::new();
                self.insn_rd_counts = vec![width_data];
                if self.insn_rep_count != 0 {
                    self.insn_rd_counts
                        .extend(std::iter::repeat_n(width_data, self.insn_rep_count as usize));
                    self.insn_rep_count = 0;
                }
                mnemonics = vec![
                    format!("Insn: LD {ptr_txt} m{width_data}"),
                    format!("LD {ptr_txt} m{width_data}"),
                    "LD".into(),
                ];
                self.cmd_insn_parts_nice = vec!["LD".into(), ptr_txt.into()];
                self.cmd_insn_parts_terse = vec!["LD".into(), ptr_txt_terse.into()];
            } else if opcode == OP_STS {
                // "STS: store data, direct addressing. Writes an address,
                // writes a data item."
                let width_addr = usize::from(arg32) + 1;
                let width_data = usize::from(arg10) + 1;
                self.insn_wr_counts = vec![width_addr, width_data];
                self.insn_rd_counts = Vec::new();
                mnemonics = vec![
                    format!("Insn: STS a{width_addr}, i{width_data}"),
                    format!("STS a{width_addr}, i{width_data}"),
                    "STS".into(),
                ];
                self.cmd_insn_parts_nice = vec!["STS".into()];
                self.cmd_insn_parts_terse = vec!["STS".into()];
            } else if opcode == OP_ST {
                // "ST: store data, indirect addressing. Writes a data
                // item, with optional repeat."
                let ptr_txt = POINTER_FORMAT_NICE[usize::from(arg32)];
                let ptr_txt_terse = POINTER_FORMAT_TERSE[usize::from(arg32)];
                let width_data = usize::from(arg10) + 1;
                self.insn_wr_counts = vec![width_data];
                self.insn_rd_counts = Vec::new();
                if self.insn_rep_count != 0 {
                    self.insn_wr_counts
                        .extend(std::iter::repeat_n(width_data, self.insn_rep_count as usize));
                    self.insn_rep_count = 0;
                }
                mnemonics = vec![
                    format!("Insn: ST {ptr_txt} i{width_data}"),
                    format!("ST {ptr_txt} i{width_data}"),
                    "ST".into(),
                ];
                self.cmd_insn_parts_nice = vec!["ST".into(), ptr_txt.into()];
                self.cmd_insn_parts_terse = vec!["ST".into(), ptr_txt_terse.into()];
            } else if opcode == OP_LDCS {
                // "LDCS: load control/status. Loads exactly one byte."
                let reg_num = arg30;
                let reg_txt = ctrl_reg_name(reg_num);
                let reg_txt_terse = format!("{reg_num}");
                self.insn_wr_counts = Vec::new();
                self.insn_rd_counts = vec![1];
                mnemonics = vec![
                    format!("Insn: LDCS {reg_txt}, m1"),
                    format!("LDCS {reg_txt}, m1"),
                    "LDCS".into(),
                ];
                self.cmd_insn_parts_nice = vec!["LDCS".into(), reg_txt];
                self.cmd_insn_parts_terse = vec!["LDCS".into(), reg_txt_terse];
            } else if opcode == OP_STCS {
                // "STCS: store control/status. Writes exactly one byte."
                let reg_num = arg30;
                let reg_txt = ctrl_reg_name(reg_num);
                let reg_txt_terse = format!("{reg_num}");
                self.insn_wr_counts = vec![1];
                self.insn_rd_counts = Vec::new();
                mnemonics = vec![
                    format!("Insn: STCS {reg_txt}, i1"),
                    format!("STCS {reg_txt}, i1"),
                    "STCS".into(),
                ];
                self.cmd_insn_parts_nice = vec!["STCS".into(), reg_txt];
                self.cmd_insn_parts_terse = vec!["STCS".into(), reg_txt_terse];
            } else if opcode == OP_REPEAT {
                // "REPEAT: sets repeat count for the next instruction.
                // Reads repeat count from following bytes."
                let width_data = usize::from(arg10) + 1;
                self.insn_wr_counts = vec![width_data];
                self.insn_rd_counts = Vec::new();
                mnemonics = vec![
                    format!("Insn: REPEAT i{width_data}"),
                    format!("REPEAT i{width_data}"),
                    "REP".into(),
                ];
                self.cmd_insn_parts_nice = vec!["REPEAT".into()];
                self.cmd_insn_parts_terse = vec!["REP".into()];
            } else if opcode == OP_KEY {
                // OP_KEY: "set activation key (enables PDIBUS mmap access).
                // Writes a sequence of 8 bytes, fixed length."
                let width_data = 8;
                self.insn_wr_counts = vec![width_data];
                self.insn_rd_counts = Vec::new();
                mnemonics = vec![
                    format!("Insn: KEY i{width_data}"),
                    format!("KEY i{width_data}"),
                    "KEY".into(),
                ];
                self.cmd_insn_parts_nice = vec!["KEY".into()];
                self.cmd_insn_parts_terse = vec!["KEY".into()];
            } else {
                unreachable!("opcode is three bits");
            }

            // "Emit an annotation for the instruction opcode."
            ctx.put_ann(ss, es, ANN_OPCODE, mnemonics);

            // "Prepare to write/read operands/data bytes."
            self.insn_dat_bytes = Vec::new();
            if !self.insn_wr_counts.is_empty() {
                self.insn_dat_count = self.insn_wr_counts[0];
                return;
            }
            if !self.insn_rd_counts.is_empty() {
                self.insn_dat_count = self.insn_rd_counts[0];
                return;
            }
            // "FALLTHROUGH."
        }

        // "Read bytes which carry operands (addresses, immediates) or
        // data values for memory access."
        if self.insn_dat_count != 0 && !is_break {
            let byteval = byteval.unwrap();
            // "Accumulate received bytes until another multi byte data
            // item is complete."
            if self.insn_dat_bytes.is_empty() {
                self.insn_ss_data = Some(ss);
            }
            self.insn_dat_bytes.push(byteval);
            self.insn_dat_count -= 1;
            if self.insn_dat_count != 0 {
                return;
            }

            // "Determine the data item's duration and direction, 'consume'
            // its length spec (to simplify later steps)."
            let data_ss = self.insn_ss_data.unwrap_or(ss);
            let data_es = es;
            let data_ann;
            if self.insn_wr_counts.is_empty() {
                data_ann = ANN_DATA_DEV;
                self.insn_rd_counts.remove(0);
            } else {
                data_ann = ANN_DATA_PROG;
                self.insn_wr_counts.remove(0);
            }

            // "PDI communicates multi-byte data items in little endian
            // order. Get a nice textual representation of the number,
            // wide and narrow for several zoom levels."
            let mut data_txt_digits = String::new();
            for b in self.insn_dat_bytes.iter().rev() {
                let _ = write!(data_txt_digits, "{b:02x}");
            }
            let data_txt_hex = format!("0x{data_txt_digits}");
            let data_txt_prefix = format!("Data: {data_txt_hex}");
            let data_txts = vec![data_txt_prefix, data_txt_hex.clone(), data_txt_digits.clone()];
            self.insn_dat_bytes = Vec::new();

            // "Emit an annotation for the data value."
            ctx.put_ann(data_ss, data_es, data_ann, data_txts);

            // "Collect detailed information which describes the whole
            // command when combined."
            self.cmd_insn_parts_nice.push(data_txt_hex);
            self.cmd_insn_parts_terse.push(data_txt_digits);

            // "Send out write data first until exhausted, then drain
            // expected read data."
            if !self.insn_wr_counts.is_empty() {
                self.insn_dat_count = self.insn_wr_counts[0];
                return;
            }
            if !self.insn_rd_counts.is_empty() {
                self.insn_dat_count = self.insn_rd_counts[0];
                return;
            }
            // "FALLTHROUGH."
        }

        // "Postprocess the instruction after its operands were seen."
        let cmd_es = es;
        let cmd_txt_nice = self.cmd_insn_parts_nice.join(" ");
        let cmd_txt_terse = self.cmd_insn_parts_terse.join(" ");
        ctx.put_ann(
            self.cmd_ss.unwrap_or(ss),
            cmd_es,
            ANN_COMMAND,
            vec![cmd_txt_nice, cmd_txt_terse],
        );
        if self.insn_opcode == Some(OP_REPEAT) && !is_break {
            // "The last communicated data item is the repeat count for
            // the next instruction (i.e. it will execute N+1 times when
            // 'REPEAT N' is specified)."
            let count = self.cmd_insn_parts_nice.last().map_or(0, |t| int_base_zero(t));
            self.insn_rep_count = count;
        }

        // "Have the state for instruction decoding cleared, but make sure
        // to carry over REPEAT count specs between instructions."
        let save_rep_count = self.insn_rep_count;
        self.clear_insn();
        self.insn_rep_count = save_rep_count;
    }

    /// The Python decoder's `handle_bits()`.
    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn handle_bits(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        bitval: Option<u8>,
    ) -> Result<()> {
        // "The number of bits which form one UART frame. Note that the
        // decoder operates with only one stop bit."
        let frame_bitcount = 1 + 8 + 1 + 1;

        // "Detect adjacent runs of all-zero bits. This is meant to cope
        // when BREAK conditions appear at any arbitrary position, it need
        // not be 'aligned' to an UART frame."
        if bitval == Some(1) {
            self.zero_count = 0;
        } else if bitval == Some(0) {
            if self.zero_count == 0 {
                self.zero_ss = Some(ss);
            }
            self.zero_count += 1;
            if self.zero_count == frame_bitcount {
                self.break_ss = self.zero_ss;
            }
        }

        // "BREAK conditions are _at_minimum_ the length of a UART frame,
        // but can span an arbitrary number of bit times. Track the 'end
        // sample' value of the last low bit we have seen, and emit the
        // annotation only after the line went idle (high) again."
        //
        // The upstream test `if bitval == '0':` compares an int with a
        // string and never matches, so `break_es` is still None when the
        // run of low bits reaches a frame's length. `put()` then rejects
        // the None end sample (its argument format is "KKiO"), which ends
        // the Python decoder with an error here. Keep that behaviour.
        if self.break_ss.is_some() {
            return Err(Error::arg("break condition: end sample is None"));
        }

        // "Ignore high bits when waiting for START."
        if self.bits.is_empty() && bitval == Some(1) {
            return Ok(());
        }

        // "Store individual bits and their start/end sample numbers,
        // until a complete frame was received."
        self.bits.push(Bit { val: bitval, ss, es });
        if self.bits.len() < frame_bitcount {
            return Ok(());
        }

        // "Get individual fields of the UART frame."
        let mut bits_num: u64 = 0;
        for (pos, b) in self.bits.iter().enumerate() {
            bits_num |= u64::from(b.val.unwrap_or(0)) << pos;
        }
        let _ = bits_num & 0x01; // The start bit; Python does not use it.
        bits_num >>= 1;
        let data_val = (bits_num & 0xff) as u8;
        bits_num >>= 8;
        let data_text = format!("{data_val:02x}");
        let parity_bit = bits_num & 0x01;
        bits_num >>= 1;
        let stop_bit = bits_num & 0x01;

        // "Check for frame errors."
        let parity_ok = (data_val.count_ones() + parity_bit as u32).is_multiple_of(2);
        let stop_ok = stop_bit == 1;
        let valid_frame = parity_ok && stop_ok;

        // "Emit annotations."
        for idx in 0..frame_bitcount {
            self.put_ann_bit(ctx, idx, ANN_BIT);
        }
        self.put_uart_field(ctx, 0, ANN_START, &["Start bit", "Start", "S"]);
        let data = self.bits.clone();
        let start = data[1];
        let end = data[8];
        ctx.put_ann(
            start.ss,
            end.es,
            ANN_DATA,
            vec![format!("Data: {data_text}"), format!("D: {data_text}"), data_text.clone()],
        );
        self.put_uart_field(
            ctx,
            9,
            if parity_ok { ANN_PARITY_OK } else { ANN_PARITY_ERR },
            if parity_ok {
                &["Parity OK", "Par OK", "P"]
            } else {
                &["Parity error", "Par ERR", "PE"]
            },
        );
        self.put_uart_field(
            ctx,
            10,
            if stop_ok { ANN_STOP_OK } else { ANN_STOP_ERR },
            if stop_ok {
                &["Stop bit", "Stop", "T"]
            } else {
                &["Stop bit error", "Stop ERR", "TE"]
            },
        );

        // "Emit binary data stream. Have bytes interpreted at higher
        // layers."
        if valid_frame {
            let byte_ss = data[0].ss;
            let byte_es = data[data.len() - 1].es;
            ctx.put_binary(byte_ss, byte_es, BIN_BYTES, vec![data_val]);
            self.handle_byte(ctx, byte_ss, byte_es, Some(data_val));
        }

        // "Reset internal state for the next frame."
        self.bits = Vec::new();
        Ok(())
    }

    /// The Python decoder's `handle_clk_edge()`.
    #[allow(clippy::similar_names)]
    fn handle_clk_edge(
        &mut self,
        ctx: &mut dyn Context,
        clock_pin: bool,
        data_pin: bool,
    ) -> Result<()> {
        // "Sample the data line on rising clock edges. Always, for TX and
        // for RX bytes alike."
        if clock_pin {
            self.data_sample = Some(u8::from(data_pin));
            return Ok(());
        }

        // "Falling clock edges are boundaries for bit slots."
        self.ss_last_fall = self.ss_curr_fall;
        self.ss_curr_fall = Some(ctx.samplenum());
        if self.ss_last_fall.is_none() {
            return Ok(());
        }

        // "Have the past bit slot processed."
        let bit_ss = self.ss_last_fall.expect("last fall");
        let bit_es = self.ss_curr_fall.expect("current fall");
        let bit_val = self.data_sample;
        self.handle_bits(ctx, bit_ss, bit_es, bit_val)
    }
}

impl Decoder for AvrPdi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = AvrPdi::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let cond: &[Cond] = &[Cond::pin(RESET, CondKind::Edge)];
        loop {
            let pins = ctx.wait(&[cond])?;
            let clock_pin = pins.get(RESET);
            let data_pin = pins.get(DATA);
            self.handle_clk_edge(ctx, clock_pin, data_pin)?;
        }
    }
}
