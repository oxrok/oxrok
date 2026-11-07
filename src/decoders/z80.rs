// Counterpart in sigrok:
//   libsigrokdecode/decoders/z80/pd.py and tables.py,
//   Copyright (C) 2014 Daniel Elstner <daniel.kitta@gmail.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Z80 bus cycles and the decoder's instruction
//   tables and annotation texts.
//
// Subtle points of the Python original that this port keeps:
//   - Data is sampled only while a bus cycle is active, and the state
//     machine at the end of a cycle works on `pend_data`, the data of the
//     *previous* cycle. `state_IDLE` is the exception: it reads
//     `self.bus_data`, the cycle that just ended.
//   - `on_cycle_end` emits a disassembly that was prepared by that same
//     call, and only then emits the delayed data byte of the previous
//     cycle (instruction before data).
//   - `prev_cycle` is still the ended cycle while `on_cycle_end` runs, so
//     the state functions' `prev_cycle` checks see the cycle just ended.
//   - `on_cycle_trans` warns at `self.samplenum - 1` and drops the
//     pending address, data and disassembly.
//   - An unassigned bus channel is `0xFF`, so `reduce_bus` returns None
//     and the address annotation is skipped (data channels are required).

//! Z80: bus cycles and instruction disassembly.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo,
};

const ANN_ADDR: usize = 0;
const ANN_MEMRD: usize = 1;
const ANN_MEMWR: usize = 2;
const ANN_IORD: usize = 3;
const ANN_IOWR: usize = 4;
const ANN_INSTR: usize = 5;
const ANN_ROP: usize = 6;
const ANN_WOP: usize = 7;
const ANN_WARN: usize = 8;

// Channel indexes, as Python's Pin class numbers them.
const P_D0: usize = 0;
const P_D7: usize = 7;
const P_M1: usize = 8;
const P_RD: usize = 9;
const P_WR: usize = 10;
const P_MREQ: usize = 11;
const P_IORQ: usize = 12;
const P_A0: usize = 13;
const P_A15: usize = 28;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "z80",
    name: "Z80",
    longname: "Zilog Z80 CPU",
    desc: "Zilog Z80 microprocessor disassembly.",
    license: "gplv3+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Retro computing"],
    channels: &[
        ChannelDesc { id: "d0", name: "D0", desc: "Data bus line 0" },
        ChannelDesc { id: "d1", name: "D1", desc: "Data bus line 1" },
        ChannelDesc { id: "d2", name: "D2", desc: "Data bus line 2" },
        ChannelDesc { id: "d3", name: "D3", desc: "Data bus line 3" },
        ChannelDesc { id: "d4", name: "D4", desc: "Data bus line 4" },
        ChannelDesc { id: "d5", name: "D5", desc: "Data bus line 5" },
        ChannelDesc { id: "d6", name: "D6", desc: "Data bus line 6" },
        ChannelDesc { id: "d7", name: "D7", desc: "Data bus line 7" },
        ChannelDesc { id: "m1", name: "/M1", desc: "Machine cycle 1" },
        ChannelDesc { id: "rd", name: "/RD", desc: "Memory or I/O read" },
        ChannelDesc { id: "wr", name: "/WR", desc: "Memory or I/O write" },
    ],
    optional_channels: &[
        ChannelDesc { id: "mreq", name: "/MREQ", desc: "Memory request" },
        ChannelDesc { id: "iorq", name: "/IORQ", desc: "I/O request" },
        ChannelDesc { id: "a0", name: "A0", desc: "Address bus line 0" },
        ChannelDesc { id: "a1", name: "A1", desc: "Address bus line 1" },
        ChannelDesc { id: "a2", name: "A2", desc: "Address bus line 2" },
        ChannelDesc { id: "a3", name: "A3", desc: "Address bus line 3" },
        ChannelDesc { id: "a4", name: "A4", desc: "Address bus line 4" },
        ChannelDesc { id: "a5", name: "A5", desc: "Address bus line 5" },
        ChannelDesc { id: "a6", name: "A6", desc: "Address bus line 6" },
        ChannelDesc { id: "a7", name: "A7", desc: "Address bus line 7" },
        ChannelDesc { id: "a8", name: "A8", desc: "Address bus line 8" },
        ChannelDesc { id: "a9", name: "A9", desc: "Address bus line 9" },
        ChannelDesc { id: "a10", name: "A10", desc: "Address bus line 10" },
        ChannelDesc { id: "a11", name: "A11", desc: "Address bus line 11" },
        ChannelDesc { id: "a12", name: "A12", desc: "Address bus line 12" },
        ChannelDesc { id: "a13", name: "A13", desc: "Address bus line 13" },
        ChannelDesc { id: "a14", name: "A14", desc: "Address bus line 14" },
        ChannelDesc { id: "a15", name: "A15", desc: "Address bus line 15" },
    ],
    options: &[],
    annotations: &[
        AnnClass { id: "addr", desc: "Memory or I/O address" },
        AnnClass { id: "memrd", desc: "Byte read from memory" },
        AnnClass { id: "memwr", desc: "Byte written to memory" },
        AnnClass { id: "iord", desc: "Byte read from I/O port" },
        AnnClass { id: "iowr", desc: "Byte written to I/O port" },
        AnnClass { id: "instr", desc: "Z80 CPU instruction" },
        AnnClass { id: "rop", desc: "Value of input operand" },
        AnnClass { id: "wop", desc: "Value of output operand" },
        AnnClass { id: "warn", desc: "Warning message" },
    ],
    annotation_rows: &[
        AnnRow { id: "addrbus", desc: "Address bus", classes: &[0] },
        AnnRow { id: "databus", desc: "Data bus", classes: &[1, 2, 3, 4] },
        AnnRow { id: "instructions", desc: "Instructions", classes: &[5] },
        AnnRow { id: "operands", desc: "Operands", classes: &[6, 7] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[8] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Z80::default())
}

/// One instruction-table entry. `write` is signed: a negative value
/// selects big-endian operand byte order (`want_wr_be`).
#[derive(Clone, Copy, Debug)]
struct Instr {
    dis: i32,
    imm: i32,
    read: i32,
    write: i32,
    repeat: bool,
    mnemonic: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cycle {
    None,
    Memrd,
    Memwr,
    Iord,
    Iowr,
    Fetch,
    Intack,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Pre1,
    Pre2,
    PreDis,
    Opcode,
    PostDis,
    Imm1,
    Imm2,
    Rop1,
    Rop2,
    Wop1,
    Wop2,
    Restart,
}

fn reduce_bus(bus: &[u8]) -> Option<u8> {
    let mut value = 0u8;
    for (i, &pin) in bus.iter().enumerate() {
        if pin == 0xff {
            return None; // unassigned bus channels
        }
        value |= (pin & 1) << i;
    }
    Some(value)
}

fn reduce_addr(bus: &[u8]) -> Option<u16> {
    let mut value = 0u16;
    for (i, &pin) in bus.iter().enumerate() {
        if pin == 0xff {
            return None;
        }
        value |= u16::from(pin & 1) << i;
    }
    Some(value)
}

fn signed_byte(byte: u8) -> i64 {
    i64::from(byte as i8)
}

fn ann_data_cycle_map(cycle: Cycle) -> Option<usize> {
    match cycle {
        Cycle::Memrd | Cycle::Fetch => Some(ANN_MEMRD),
        Cycle::Memwr => Some(ANN_MEMWR),
        Cycle::Iord | Cycle::Intack => Some(ANN_IORD),
        Cycle::Iowr => Some(ANN_IOWR),
        Cycle::None => None,
    }
}

/// Python's `AsmFormatter`: a spec ending in H is uppercase hex with a
/// leading zero when the first digit is not a decimal digit.
fn hex_h(value: i64, width: usize) -> String {
    let s = format!("{value:0width$X}");
    match s.chars().next() {
        Some(c) if c.is_ascii_digit() => s,
        _ => format!("0{s}"),
    }
}

/// The decoder.
pub struct Z80 {
    // Bus cycle tracking.
    bus_data: Option<u8>,
    samplenum: u64,
    addr_start: Option<u64>,
    data_start: Option<u64>,
    dasm_start: Option<u64>,
    pend_addr: Option<u16>,
    pend_data: Option<u8>,
    ann_data: Option<usize>,
    ann_dasm: Option<usize>,
    prev_cycle: Cycle,
    op_state: State,
    instr_len: i64,
    // Instruction being assembled.
    want_dis: i32,
    want_imm: i32,
    want_read: i32,
    want_write: i32,
    want_wr_be: bool,
    op_repeat: bool,
    arg_dis: i64,
    arg_imm: i64,
    arg_read: i64,
    arg_write: i64,
    arg_reg: &'static str,
    mnemonic: &'static str,
    op_prefix: u32,
}

impl Default for Z80 {
    fn default() -> Self {
        Z80 {
            bus_data: None,
            samplenum: 0,
            addr_start: None,
            data_start: None,
            dasm_start: None,
            pend_addr: None,
            pend_data: None,
            ann_data: None,
            ann_dasm: None,
            prev_cycle: Cycle::None,
            op_state: State::Idle,
            instr_len: 0,
            want_dis: 0,
            want_imm: 0,
            want_read: 0,
            want_write: 0,
            want_wr_be: false,
            op_repeat: false,
            arg_dis: 0,
            arg_imm: 0,
            arg_read: 0,
            arg_write: 0,
            arg_reg: "",
            mnemonic: "",
            op_prefix: 0,
        }
    }
}

impl Z80 {
    fn put_text(&mut self, ctx: &mut dyn Context, ss: u64, ann: usize, text: String) {
        ctx.put_ann(ss, self.samplenum, ann, vec![text]);
    }

    fn put_disasm(&mut self, ctx: &mut dyn Context) {
        let text = format_mnemonic(
            self.mnemonic,
            self.arg_reg,
            self.arg_dis,
            self.arg_dis + self.instr_len,
            self.arg_imm,
            self.arg_read,
            self.arg_write,
        );
        let ss = self.dasm_start.unwrap_or(self.samplenum);
        let ann = self.ann_dasm.unwrap_or(ANN_INSTR);
        self.put_text(ctx, ss, ann, text);
        self.ann_dasm = None;
        self.dasm_start = Some(self.samplenum);
    }

    fn on_cycle_begin(&mut self, ctx: &mut dyn Context, bus_addr: Option<u16>) {
        if let Some(addr) = self.pend_addr {
            let ss = self.addr_start.unwrap_or(self.samplenum);
            self.put_text(ctx, ss, ANN_ADDR, format!("{addr:04X}"));
        }
        self.addr_start = Some(self.samplenum);
        self.pend_addr = bus_addr;
    }

    fn on_cycle_end(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.instr_len += 1;
        let mut next = self.step_state()?;
        if self.ann_dasm.is_some() {
            self.put_disasm(ctx);
        }
        if next == State::Restart {
            next = self.state_idle();
        }
        self.op_state = next;

        if let Some(ann) = self.ann_data {
            let data = self.pend_data.ok_or_else(|| Error::device("no data byte"))?;
            let ss = self.data_start.unwrap_or(self.samplenum);
            self.put_text(ctx, ss, ann, format!("{data:02X}"));
        }
        self.data_start = Some(self.samplenum);
        self.pend_data = self.bus_data;
        self.ann_data = ann_data_cycle_map(self.prev_cycle);
        Ok(())
    }

    fn on_cycle_trans(&mut self, ctx: &mut dyn Context) {
        self.put_text(
            ctx,
            self.samplenum.saturating_sub(1),
            ANN_WARN,
            "Illegal transition between control states".to_owned(),
        );
        self.pend_addr = None;
        self.ann_data = None;
        self.ann_dasm = None;
    }

    fn step_state(&mut self) -> Result<State> {
        match self.op_state {
            State::Idle => Ok(self.state_idle()),
            State::Pre1 => Ok(self.state_pre1()),
            State::Pre2 => self.state_pre2(),
            State::PreDis => self.state_predis(),
            State::Opcode => Ok(self.state_opcode()),
            State::PostDis => self.state_postdis(),
            State::Imm1 => self.state_imm1(),
            State::Imm2 => self.state_imm2(),
            State::Rop1 => self.state_rop1(),
            State::Rop2 => self.state_rop2(),
            State::Wop1 => self.state_wop1(),
            State::Wop2 => self.state_wop2(),
            State::Restart => Ok(State::Idle),
        }
    }

    fn state_idle(&mut self) -> State {
        if self.prev_cycle != Cycle::Fetch {
            return State::Idle;
        }
        self.want_dis = 0;
        self.want_imm = 0;
        self.want_read = 0;
        self.want_write = 0;
        self.want_wr_be = false;
        self.op_repeat = false;
        self.arg_dis = 0;
        self.arg_imm = 0;
        self.arg_read = 0;
        self.arg_write = 0;
        self.arg_reg = "";
        self.mnemonic = "";
        self.dasm_start = Some(self.samplenum);
        self.op_prefix = 0;
        self.instr_len = 0;
        if matches!(self.bus_data, Some(0xCB | 0xED | 0xDD | 0xFD)) {
            State::Pre1
        } else {
            State::Opcode
        }
    }

    fn state_pre1(&mut self) -> State {
        if self.prev_cycle != Cycle::Fetch {
            self.mnemonic = "Prefix not followed by fetch";
            self.ann_dasm = Some(ANN_WARN);
            return State::Restart;
        }
        self.op_prefix = u32::from(self.pend_data.unwrap_or(0));
        if self.op_prefix == 0xDD || self.op_prefix == 0xFD {
            if self.bus_data == Some(0xCB) {
                return State::Pre2;
            }
            if matches!(self.bus_data, Some(0xDD | 0xED | 0xFD)) {
                return State::Pre1;
            }
        }
        State::Opcode
    }

    fn state_pre2(&mut self) -> Result<State> {
        if self.prev_cycle != Cycle::Memrd {
            self.mnemonic = "Missing displacement";
            self.ann_dasm = Some(ANN_WARN);
            return Ok(State::Restart);
        }
        self.op_prefix = (self.op_prefix << 8)
            | u32::from(self.pend_data.ok_or_else(|| Error::device("no data"))?);
        Ok(State::PreDis)
    }

    fn state_predis(&mut self) -> Result<State> {
        if self.prev_cycle != Cycle::Memrd {
            self.mnemonic = "Missing opcode";
            self.ann_dasm = Some(ANN_WARN);
            return Ok(State::Restart);
        }
        self.arg_dis = signed_byte(self.pend_data.ok_or_else(|| Error::device("no displacement"))?);
        Ok(State::Opcode)
    }

    fn state_opcode(&mut self) -> State {
        let (table, arg_reg) = table_by_prefix(self.op_prefix);
        self.arg_reg = arg_reg;
        self.op_prefix = 0;
        let instruction = self.pend_data.and_then(table);
        let Some(instruction) = instruction else {
            self.mnemonic = "Invalid instruction";
            self.ann_dasm = Some(ANN_WARN);
            return State::Restart;
        };
        self.want_dis = instruction.dis;
        self.want_imm = instruction.imm;
        self.want_read = instruction.read;
        self.want_write = instruction.write.abs();
        self.want_wr_be = instruction.write < 0;
        self.op_repeat = instruction.repeat;
        self.mnemonic = instruction.mnemonic;
        if self.want_dis > 0 {
            return State::PostDis;
        }
        if self.want_imm > 0 {
            return State::Imm1;
        }
        self.ann_dasm = Some(ANN_INSTR);
        if self.want_read > 0 && matches!(self.prev_cycle, Cycle::Memrd | Cycle::Iord) {
            return State::Rop1;
        }
        if self.want_write > 0 && matches!(self.prev_cycle, Cycle::Memwr | Cycle::Iowr) {
            return State::Wop1;
        }
        State::Restart
    }

    fn state_postdis(&mut self) -> Result<State> {
        self.arg_dis = signed_byte(self.pend_data.ok_or_else(|| Error::device("no displacement"))?);
        if self.want_imm > 0 {
            return Ok(State::Imm1);
        }
        self.ann_dasm = Some(ANN_INSTR);
        if self.want_read > 0 && matches!(self.prev_cycle, Cycle::Memrd | Cycle::Iord) {
            return Ok(State::Rop1);
        }
        if self.want_write > 0 && matches!(self.prev_cycle, Cycle::Memwr | Cycle::Iowr) {
            return Ok(State::Wop1);
        }
        Ok(State::Restart)
    }

    fn state_imm1(&mut self) -> Result<State> {
        self.arg_imm = i64::from(self.pend_data.ok_or_else(|| Error::device("no immediate"))?);
        if self.want_imm > 1 {
            return Ok(State::Imm2);
        }
        self.ann_dasm = Some(ANN_INSTR);
        if self.want_read > 0 && matches!(self.prev_cycle, Cycle::Memrd | Cycle::Iord) {
            return Ok(State::Rop1);
        }
        if self.want_write > 0 && matches!(self.prev_cycle, Cycle::Memwr | Cycle::Iowr) {
            return Ok(State::Wop1);
        }
        Ok(State::Restart)
    }

    fn state_imm2(&mut self) -> Result<State> {
        self.arg_imm |=
            i64::from(self.pend_data.ok_or_else(|| Error::device("no immediate"))?) << 8;
        self.ann_dasm = Some(ANN_INSTR);
        if self.want_read > 0 && matches!(self.prev_cycle, Cycle::Memrd | Cycle::Iord) {
            return Ok(State::Rop1);
        }
        if self.want_write > 0 && matches!(self.prev_cycle, Cycle::Memwr | Cycle::Iowr) {
            return Ok(State::Wop1);
        }
        Ok(State::Restart)
    }

    fn state_rop1(&mut self) -> Result<State> {
        self.arg_read = i64::from(self.pend_data.ok_or_else(|| Error::device("no read data"))?);
        if self.want_read < 2 {
            self.mnemonic = "{ro:02X}";
            self.ann_dasm = Some(ANN_ROP);
        }
        if self.want_write > 0 {
            return Ok(State::Wop1);
        }
        if self.want_read > 1 {
            return Ok(State::Rop2);
        }
        if self.op_repeat && matches!(self.prev_cycle, Cycle::Memrd | Cycle::Iord) {
            return Ok(State::Rop1);
        }
        Ok(State::Restart)
    }

    fn state_rop2(&mut self) -> Result<State> {
        self.arg_read |=
            i64::from(self.pend_data.ok_or_else(|| Error::device("no read data"))?) << 8;
        self.mnemonic = "{ro:04X}";
        self.ann_dasm = Some(ANN_ROP);
        if self.want_write > 0 && matches!(self.prev_cycle, Cycle::Memwr | Cycle::Iowr) {
            return Ok(State::Wop1);
        }
        Ok(State::Restart)
    }

    fn state_wop1(&mut self) -> Result<State> {
        self.arg_write = i64::from(self.pend_data.ok_or_else(|| Error::device("no write data"))?);
        if self.want_read > 1 {
            return Ok(State::Rop2);
        }
        if self.want_write > 1 {
            return Ok(State::Wop2);
        }
        self.mnemonic = "{wo:02X}";
        self.ann_dasm = Some(ANN_WOP);
        if self.want_read > 0
            && self.op_repeat
            && matches!(self.prev_cycle, Cycle::Memrd | Cycle::Iord)
        {
            return Ok(State::Rop1);
        }
        Ok(State::Restart)
    }

    fn state_wop2(&mut self) -> Result<State> {
        let data = u32::from(self.pend_data.ok_or_else(|| Error::device("no write data"))?);
        if self.want_wr_be {
            self.arg_write = (self.arg_write << 8) | i64::from(data);
        } else {
            self.arg_write |= i64::from(data) << 8;
        }
        self.mnemonic = "{wo:04X}";
        self.ann_dasm = Some(ANN_WOP);
        Ok(State::Restart)
    }
}

impl Decoder for Z80 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            let pins = ctx.wait(&[])?;
            self.samplenum = ctx.samplenum();
            let p = &pins.0;
            let mreq = p[P_MREQ];
            let rd = p[P_RD];
            let wr = p[P_WR];
            let m1 = p[P_M1];
            let iorq = p[P_IORQ];

            let mut cycle = Cycle::None;
            if mreq != 1 {
                // default to asserted
                if rd == 0 {
                    cycle = if m1 == 0 { Cycle::Fetch } else { Cycle::Memrd };
                } else if wr == 0 {
                    cycle = Cycle::Memwr;
                }
            } else if iorq == 0 {
                // default to not asserted
                if m1 == 0 {
                    cycle = Cycle::Intack;
                } else if rd == 0 {
                    cycle = Cycle::Iord;
                } else if wr == 0 {
                    cycle = Cycle::Iowr;
                }
            }

            if cycle != Cycle::None {
                self.bus_data = reduce_bus(&p[P_D0..=P_D7]);
            }
            if cycle != self.prev_cycle {
                if self.prev_cycle == Cycle::None {
                    self.on_cycle_begin(ctx, reduce_addr(&p[P_A0..=P_A15]));
                } else if cycle == Cycle::None {
                    self.on_cycle_end(ctx)?;
                } else {
                    self.on_cycle_trans(ctx);
                }
            }
            self.prev_cycle = cycle;
        }
    }
}

/// Python's `formatter.format(self.mnemonic, r=..., d=..., j=...,
/// i=..., ro=..., wo=...)` for the placeholders the tables use.
#[allow(clippy::format_push_string)]
fn format_mnemonic(
    template: &str,
    reg: &str,
    dis: i64,
    jump: i64,
    imm: i64,
    read: i64,
    write: i64,
) -> String {
    let mut out = String::new();
    let mut rest = template;
    while let Some(start) = rest.find('{') {
        out.push_str(&rest[..start]);
        let Some(end_rel) = rest[start..].find('}') else {
            out.push_str(&rest[start..]);
            return out;
        };
        let end = start + end_rel;
        let field = &rest[start + 1..end];
        let (name, spec) = field.split_once(':').unwrap_or((field, ""));
        match (name, spec) {
            ("r", "") => out.push_str(reg),
            ("d", "+d") => out.push_str(&format!("{dis:+}")),
            ("j", "+d") => out.push_str(&format!("{jump:+}")),
            ("i", "02H") => out.push_str(&hex_h(imm, 2)),
            ("i", "04H") => out.push_str(&hex_h(imm, 4)),
            ("ro", "02X") => out.push_str(&format!("{read:02X}")),
            ("ro", "04X") => out.push_str(&format!("{read:04X}")),
            ("wo", "02X") => out.push_str(&format!("{write:02X}")),
            ("wo", "04X") => out.push_str(&format!("{write:04X}")),
            _ => {}
        }
        rest = &rest[end + 1..];
    }
    out.push_str(rest);
    out
}

#[allow(clippy::too_many_lines, clippy::match_same_arms, clippy::unnecessary_wraps)]
fn main_instr(op: u8) -> Option<Instr> {
    Some(match op {
        0x00 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NOP" },
        0x01 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "LD BC,{i:04H}h" }
        }
        0x02 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (BC),A" },
        0x03 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC BC" },
        0x04 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC B" },
        0x05 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC B" },
        0x06 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD B,{i:02H}h" }
        }
        0x07 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLCA" },
        0x08 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "EX AF,AF'" },
        0x09 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD HL,BC" },
        0x0A => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD A,(BC)" },
        0x0B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC BC" },
        0x0C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC C" },
        0x0D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC C" },
        0x0E => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD C,{i:02H}h" }
        }
        0x0F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRCA" },
        0x10 => {
            Instr { dis: 1, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DJNZ ${j:+d}" }
        }
        0x11 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "LD DE,{i:04H}h" }
        }
        0x12 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (DE),A" },
        0x13 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC DE" },
        0x14 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC D" },
        0x15 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC D" },
        0x16 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD D,{i:02H}h" }
        }
        0x17 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLA" },
        0x18 => Instr { dis: 1, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JR ${j:+d}" },
        0x19 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD HL,DE" },
        0x1A => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD A,(DE)" },
        0x1B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC DE" },
        0x1C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC E" },
        0x1D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC E" },
        0x1E => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD E,{i:02H}h" }
        }
        0x1F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRA" },
        0x20 => {
            Instr { dis: 1, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JR NZ,${j:+d}" }
        }
        0x21 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "LD HL,{i:04H}h" }
        }
        0x22 => {
            Instr { dis: 0, imm: 2, read: 0, write: 2, repeat: false, mnemonic: "LD ({i:04H}h),HL" }
        }
        0x23 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC HL" },
        0x24 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC H" },
        0x25 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC H" },
        0x26 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD H,{i:02H}h" }
        }
        0x27 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DAA" },
        0x28 => {
            Instr { dis: 1, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JR Z,${j:+d}" }
        }
        0x29 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD HL,HL" },
        0x2A => {
            Instr { dis: 0, imm: 2, read: 2, write: 0, repeat: false, mnemonic: "LD HL,({i:04H}h)" }
        }
        0x2B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC HL" },
        0x2C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC L" },
        0x2D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC L" },
        0x2E => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD L,{i:02H}h" }
        }
        0x2F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CPL" },
        0x30 => {
            Instr { dis: 1, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JR NC,${j:+d}" }
        }
        0x31 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "LD SP,{i:04H}h" }
        }
        0x32 => {
            Instr { dis: 0, imm: 2, read: 0, write: 1, repeat: false, mnemonic: "LD ({i:04H}h),A" }
        }
        0x33 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC SP" },
        0x34 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "INC (HL)" },
        0x35 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "DEC (HL)" },
        0x36 => {
            Instr { dis: 0, imm: 1, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),{i:02H}h" }
        }
        0x37 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SCF" },
        0x38 => {
            Instr { dis: 1, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JR C,${j:+d}" }
        }
        0x39 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD HL,SP" },
        0x3A => {
            Instr { dis: 0, imm: 2, read: 1, write: 0, repeat: false, mnemonic: "LD A,({i:04H}h)" }
        }
        0x3B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC SP" },
        0x3C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC A" },
        0x3D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC A" },
        0x3E => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD A,{i:02H}h" }
        }
        0x3F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CCF" },
        0x40 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,B" },
        0x41 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,C" },
        0x42 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,D" },
        0x43 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,E" },
        0x44 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,H" },
        0x45 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,L" },
        0x46 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD B,(HL)" },
        0x47 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,A" },
        0x48 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,B" },
        0x49 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,C" },
        0x4A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,D" },
        0x4B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,E" },
        0x4C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,H" },
        0x4D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,L" },
        0x4E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD C,(HL)" },
        0x4F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,A" },
        0x50 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,B" },
        0x51 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,C" },
        0x52 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,D" },
        0x53 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,E" },
        0x54 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,H" },
        0x55 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,L" },
        0x56 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD D,(HL)" },
        0x57 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,A" },
        0x58 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,B" },
        0x59 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,C" },
        0x5A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,D" },
        0x5B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,E" },
        0x5C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,H" },
        0x5D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,L" },
        0x5E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD E,(HL)" },
        0x5F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,A" },
        0x60 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,B" },
        0x61 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,C" },
        0x62 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,D" },
        0x63 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,E" },
        0x64 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,H" },
        0x65 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,L" },
        0x66 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD H,(HL)" },
        0x67 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD H,A" },
        0x68 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,B" },
        0x69 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,C" },
        0x6A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,D" },
        0x6B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,E" },
        0x6C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,H" },
        0x6D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,L" },
        0x6E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD L,(HL)" },
        0x6F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD L,A" },
        0x70 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),B" },
        0x71 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),C" },
        0x72 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),D" },
        0x73 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),E" },
        0x74 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),H" },
        0x75 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),L" },
        0x76 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "HALT" },
        0x77 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD (HL),A" },
        0x78 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,B" },
        0x79 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,C" },
        0x7A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,D" },
        0x7B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,E" },
        0x7C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,H" },
        0x7D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,L" },
        0x7E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD A,(HL)" },
        0x7F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,A" },
        0x80 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,B" },
        0x81 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,C" },
        0x82 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,D" },
        0x83 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,E" },
        0x84 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,H" },
        0x85 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,L" },
        0x86 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "ADD A,(HL)" },
        0x87 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,A" },
        0x88 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,B" },
        0x89 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,C" },
        0x8A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,D" },
        0x8B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,E" },
        0x8C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,H" },
        0x8D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,L" },
        0x8E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "ADC A,(HL)" },
        0x8F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,A" },
        0x90 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB B" },
        0x91 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB C" },
        0x92 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB D" },
        0x93 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB E" },
        0x94 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB H" },
        0x95 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB L" },
        0x96 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "SUB (HL)" },
        0x97 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB A" },
        0x98 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,B" },
        0x99 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,C" },
        0x9A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,D" },
        0x9B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,E" },
        0x9C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,H" },
        0x9D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,L" },
        0x9E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "SBC A,(HL)" },
        0x9F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,A" },
        0xA0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND B" },
        0xA1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND C" },
        0xA2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND D" },
        0xA3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND E" },
        0xA4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND H" },
        0xA5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND L" },
        0xA6 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "AND (HL)" },
        0xA7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND A" },
        0xA8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR B" },
        0xA9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR C" },
        0xAA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR D" },
        0xAB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR E" },
        0xAC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR H" },
        0xAD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR L" },
        0xAE => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "XOR (HL)" },
        0xAF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR A" },
        0xB0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR B" },
        0xB1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR C" },
        0xB2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR D" },
        0xB3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR E" },
        0xB4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR H" },
        0xB5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR L" },
        0xB6 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "OR (HL)" },
        0xB7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR A" },
        0xB8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP B" },
        0xB9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP C" },
        0xBA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP D" },
        0xBB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP E" },
        0xBC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP H" },
        0xBD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP L" },
        0xBE => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "CP (HL)" },
        0xBF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP A" },
        0xC0 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET NZ" },
        0xC1 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "POP BC" },
        0xC2 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP NZ,{i:04H}h" }
        }
        0xC3 => Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP {i:04H}h" },
        0xC4 => Instr {
            dis: 0,
            imm: 2,
            read: 0,
            write: -2,
            repeat: false,
            mnemonic: "CALL NZ,{i:04H}h",
        },
        0xC5 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "PUSH BC" },
        0xC6 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "ADD A,{i:02H}h" }
        }
        0xC7 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 00h" },
        0xC8 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET Z" },
        0xC9 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET" },
        0xCA => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP Z,{i:04H}h" }
        }
        0xCC => {
            Instr { dis: 0, imm: 2, read: 0, write: -2, repeat: false, mnemonic: "CALL Z,{i:04H}h" }
        }
        0xCD => {
            Instr { dis: 0, imm: 2, read: 0, write: -2, repeat: false, mnemonic: "CALL {i:04H}h" }
        }
        0xCE => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "ADC A,{i:02H}h" }
        }
        0xCF => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 08h" },
        0xD0 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET NC" },
        0xD1 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "POP DE" },
        0xD2 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP NC,{i:04H}h" }
        }
        0xD3 => {
            Instr { dis: 0, imm: 1, read: 0, write: 1, repeat: false, mnemonic: "OUT ({i:02H}h),A" }
        }
        0xD4 => Instr {
            dis: 0,
            imm: 2,
            read: 0,
            write: -2,
            repeat: false,
            mnemonic: "CALL NC,{i:04H}h",
        },
        0xD5 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "PUSH DE" },
        0xD6 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "SUB {i:02H}h" }
        }
        0xD7 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 10h" },
        0xD8 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET C" },
        0xD9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "EXX" },
        0xDA => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP C,{i:04H}h" }
        }
        0xDB => {
            Instr { dis: 0, imm: 1, read: 1, write: 0, repeat: false, mnemonic: "IN A,({i:02H}h)" }
        }
        0xDC => {
            Instr { dis: 0, imm: 2, read: 0, write: -2, repeat: false, mnemonic: "CALL C,{i:04H}h" }
        }
        0xDE => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "SBC A,{i:02H}h" }
        }
        0xDF => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 18h" },
        0xE0 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET PO" },
        0xE1 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "POP HL" },
        0xE2 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP PO,{i:04H}h" }
        }
        0xE3 => Instr { dis: 0, imm: 0, read: 2, write: 2, repeat: false, mnemonic: "EX (SP),HL" },
        0xE4 => Instr {
            dis: 0,
            imm: 2,
            read: 0,
            write: -2,
            repeat: false,
            mnemonic: "CALL PO,{i:04H}h",
        },
        0xE5 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "PUSH HL" },
        0xE6 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "AND {i:02H}h" }
        }
        0xE7 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 20h" },
        0xE8 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET PE" },
        0xE9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JP (HL)" },
        0xEA => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP PE,{i:04H}h" }
        }
        0xEB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "EX DE,HL" },
        0xEC => Instr {
            dis: 0,
            imm: 2,
            read: 0,
            write: -2,
            repeat: false,
            mnemonic: "CALL PE,{i:04H}h",
        },
        0xEE => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "XOR {i:02H}h" }
        }
        0xEF => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 28h" },
        0xF0 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET P" },
        0xF1 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "POP AF" },
        0xF2 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP P,{i:04H}h" }
        }
        0xF3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DI" },
        0xF4 => {
            Instr { dis: 0, imm: 2, read: 0, write: -2, repeat: false, mnemonic: "CALL P,{i:04H}h" }
        }
        0xF5 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "PUSH AF" },
        0xF6 => Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "OR {i:02H}h" },
        0xF7 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 30h" },
        0xF8 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RET M" },
        0xF9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD SP,HL" },
        0xFA => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "JP M,{i:04H}h" }
        }
        0xFB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "EI" },
        0xFC => {
            Instr { dis: 0, imm: 2, read: 0, write: -2, repeat: false, mnemonic: "CALL M,{i:04H}h" }
        }
        0xFE => Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "CP {i:02H}h" },
        0xFF => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "RST 38h" },
        _ => return None,
    })
}

#[allow(clippy::too_many_lines, clippy::match_same_arms, clippy::unnecessary_wraps)]
fn extended_instr(op: u8) -> Option<Instr> {
    Some(match op {
        0x40 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN B,(C)" },
        0x41 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),B" },
        0x42 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC HL,BC" },
        0x43 => {
            Instr { dis: 0, imm: 2, read: 0, write: 2, repeat: false, mnemonic: "LD ({i:04H}h),BC" }
        }
        0x44 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x45 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x46 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 0" },
        0x47 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD I,A" },
        0x48 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN C,(C)" },
        0x49 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),C" },
        0x4A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC HL,BC" },
        0x4B => {
            Instr { dis: 0, imm: 2, read: 2, write: 0, repeat: false, mnemonic: "LD BC,({i:04H}h)" }
        }
        0x4C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x4D => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETI" },
        0x4E => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 0/1" },
        0x4F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD R,A" },
        0x50 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN D,(C)" },
        0x51 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),D" },
        0x52 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC HL,DE" },
        0x53 => {
            Instr { dis: 0, imm: 2, read: 0, write: 2, repeat: false, mnemonic: "LD ({i:04H}h),DE" }
        }
        0x54 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x55 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x56 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 1" },
        0x57 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,I" },
        0x58 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN E,(C)" },
        0x59 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),E" },
        0x5A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC HL,DE" },
        0x5B => {
            Instr { dis: 0, imm: 2, read: 2, write: 0, repeat: false, mnemonic: "LD DE,({i:04H}h)" }
        }
        0x5C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x5D => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x5E => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 2" },
        0x5F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,R" },
        0x60 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN H,(C)" },
        0x61 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),H" },
        0x62 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC HL,HL" },
        0x63 => {
            Instr { dis: 0, imm: 2, read: 0, write: 2, repeat: false, mnemonic: "LD ({i:04H}h),HL" }
        }
        0x64 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x65 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x66 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 0" },
        0x67 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RRD" },
        0x68 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN L,(C)" },
        0x69 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),L" },
        0x6A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC HL,HL" },
        0x6B => {
            Instr { dis: 0, imm: 2, read: 2, write: 0, repeat: false, mnemonic: "LD HL,({i:04H}h)" }
        }
        0x6C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x6D => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x6E => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 0/1" },
        0x6F => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RLD" },
        0x70 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN (C)" },
        0x71 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),0" },
        0x72 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC HL,SP" },
        0x73 => {
            Instr { dis: 0, imm: 2, read: 0, write: 2, repeat: false, mnemonic: "LD ({i:04H}h),SP" }
        }
        0x74 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x75 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x76 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 1" },
        0x78 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "IN A,(C)" },
        0x79 => Instr { dis: 0, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "OUT (C),A" },
        0x7A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC HL,SP" },
        0x7B => {
            Instr { dis: 0, imm: 2, read: 2, write: 0, repeat: false, mnemonic: "LD SP,({i:04H}h)" }
        }
        0x7C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "NEG" },
        0x7D => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "RETN" },
        0x7E => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "IM 2" },
        0xA0 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "LDI" },
        0xA1 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "CPI" },
        0xA2 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "INI" },
        0xA3 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "OUTI" },
        0xA8 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "LDD" },
        0xA9 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "CPD" },
        0xAA => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "IND" },
        0xAB => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "OUTD" },
        0xB0 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: true, mnemonic: "LDIR" },
        0xB1 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: true, mnemonic: "CPIR" },
        0xB2 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: true, mnemonic: "INIR" },
        0xB3 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: true, mnemonic: "OTIR" },
        0xB8 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: true, mnemonic: "LDDR" },
        0xB9 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: true, mnemonic: "CPDR" },
        0xBA => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: true, mnemonic: "INDR" },
        0xBB => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: true, mnemonic: "OTDR" },
        _ => return None,
    })
}

#[allow(clippy::too_many_lines, clippy::match_same_arms, clippy::unnecessary_wraps)]
fn bit_instr(op: u8) -> Option<Instr> {
    Some(match op {
        0x00 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC B" },
        0x01 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC C" },
        0x02 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC D" },
        0x03 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC E" },
        0x04 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC H" },
        0x05 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC L" },
        0x06 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RLC (HL)" },
        0x07 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RLC A" },
        0x08 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC B" },
        0x09 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC C" },
        0x0A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC D" },
        0x0B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC E" },
        0x0C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC H" },
        0x0D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC L" },
        0x0E => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RRC (HL)" },
        0x0F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RRC A" },
        0x10 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL B" },
        0x11 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL C" },
        0x12 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL D" },
        0x13 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL E" },
        0x14 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL H" },
        0x15 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL L" },
        0x16 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL (HL)" },
        0x17 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RL A" },
        0x18 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR B" },
        0x19 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR C" },
        0x1A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR D" },
        0x1B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR E" },
        0x1C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR H" },
        0x1D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR L" },
        0x1E => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR (HL)" },
        0x1F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RR A" },
        0x20 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA B" },
        0x21 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA C" },
        0x22 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA D" },
        0x23 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA E" },
        0x24 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA H" },
        0x25 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA L" },
        0x26 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SLA (HL)" },
        0x27 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLA A" },
        0x28 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA B" },
        0x29 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA C" },
        0x2A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA D" },
        0x2B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA E" },
        0x2C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA H" },
        0x2D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA L" },
        0x2E => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SRA (HL)" },
        0x2F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRA A" },
        0x30 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL B" },
        0x31 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL C" },
        0x32 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL D" },
        0x33 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL E" },
        0x34 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL H" },
        0x35 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL L" },
        0x36 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SLL (HL)" },
        0x37 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SLL A" },
        0x38 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL B" },
        0x39 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL C" },
        0x3A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL D" },
        0x3B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL E" },
        0x3C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL H" },
        0x3D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL L" },
        0x3E => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SRL (HL)" },
        0x3F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SRL A" },
        0x40 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,B" },
        0x41 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,C" },
        0x42 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,D" },
        0x43 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,E" },
        0x44 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,H" },
        0x45 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,L" },
        0x46 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 0,(HL)" },
        0x47 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 0,A" },
        0x48 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,B" },
        0x49 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,C" },
        0x4A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,D" },
        0x4B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,E" },
        0x4C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,H" },
        0x4D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,L" },
        0x4E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 1,(HL)" },
        0x4F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 1,A" },
        0x50 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,B" },
        0x51 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,C" },
        0x52 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,D" },
        0x53 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,E" },
        0x54 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,H" },
        0x55 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,L" },
        0x56 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 2,(HL)" },
        0x57 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 2,A" },
        0x58 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,B" },
        0x59 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,C" },
        0x5A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,D" },
        0x5B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,E" },
        0x5C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,H" },
        0x5D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,L" },
        0x5E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 3,(HL)" },
        0x5F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 3,A" },
        0x60 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,B" },
        0x61 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,C" },
        0x62 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,D" },
        0x63 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,E" },
        0x64 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,H" },
        0x65 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,L" },
        0x66 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 4,(HL)" },
        0x67 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 4,A" },
        0x68 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,B" },
        0x69 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,C" },
        0x6A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,D" },
        0x6B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,E" },
        0x6C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,H" },
        0x6D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,L" },
        0x6E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 5,(HL)" },
        0x6F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 5,A" },
        0x70 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,B" },
        0x71 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,C" },
        0x72 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,D" },
        0x73 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,E" },
        0x74 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,H" },
        0x75 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,L" },
        0x76 => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 6,(HL)" },
        0x77 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 6,A" },
        0x78 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,B" },
        0x79 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,C" },
        0x7A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,D" },
        0x7B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,E" },
        0x7C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,H" },
        0x7D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,L" },
        0x7E => Instr { dis: 0, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "BIT 7,(HL)" },
        0x7F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "BIT 7,A" },
        0x80 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,B" },
        0x81 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,C" },
        0x82 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,D" },
        0x83 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,E" },
        0x84 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,H" },
        0x85 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,L" },
        0x86 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 0,(HL)" },
        0x87 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 0,A" },
        0x88 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,B" },
        0x89 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,C" },
        0x8A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,D" },
        0x8B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,E" },
        0x8C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,H" },
        0x8D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,L" },
        0x8E => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 1,(HL)" },
        0x8F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 1,A" },
        0x90 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,B" },
        0x91 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,C" },
        0x92 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,D" },
        0x93 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,E" },
        0x94 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,H" },
        0x95 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,L" },
        0x96 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 2,(HL)" },
        0x97 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 2,A" },
        0x98 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,B" },
        0x99 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,C" },
        0x9A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,D" },
        0x9B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,E" },
        0x9C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,H" },
        0x9D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,L" },
        0x9E => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 3,(HL)" },
        0x9F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 3,A" },
        0xA0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,B" },
        0xA1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,C" },
        0xA2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,D" },
        0xA3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,E" },
        0xA4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,H" },
        0xA5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,L" },
        0xA6 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 4,(HL)" },
        0xA7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 4,A" },
        0xA8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,B" },
        0xA9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,C" },
        0xAA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,D" },
        0xAB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,E" },
        0xAC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,H" },
        0xAD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,L" },
        0xAE => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 5,(HL)" },
        0xAF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 5,A" },
        0xB0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,B" },
        0xB1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,C" },
        0xB2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,D" },
        0xB3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,E" },
        0xB4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,H" },
        0xB5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,L" },
        0xB6 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 6,(HL)" },
        0xB7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 6,A" },
        0xB8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,B" },
        0xB9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,C" },
        0xBA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,D" },
        0xBB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,E" },
        0xBC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,H" },
        0xBD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,L" },
        0xBE => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RES 7,(HL)" },
        0xBF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "RES 7,A" },
        0xC0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,B" },
        0xC1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,C" },
        0xC2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,D" },
        0xC3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,E" },
        0xC4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,H" },
        0xC5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,L" },
        0xC6 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 0,(HL)" },
        0xC7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 0,A" },
        0xC8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,B" },
        0xC9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,C" },
        0xCA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,D" },
        0xCB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,E" },
        0xCC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,H" },
        0xCD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,L" },
        0xCE => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 1,(HL)" },
        0xCF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 1,A" },
        0xD0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,B" },
        0xD1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,C" },
        0xD2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,D" },
        0xD3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,E" },
        0xD4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,H" },
        0xD5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,L" },
        0xD6 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 2,(HL)" },
        0xD7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 2,A" },
        0xD8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,B" },
        0xD9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,C" },
        0xDA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,D" },
        0xDB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,E" },
        0xDC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,H" },
        0xDD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,L" },
        0xDE => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 3,(HL)" },
        0xDF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 3,A" },
        0xE0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,B" },
        0xE1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,C" },
        0xE2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,D" },
        0xE3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,E" },
        0xE4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,H" },
        0xE5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,L" },
        0xE6 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 4,(HL)" },
        0xE7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 4,A" },
        0xE8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,B" },
        0xE9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,C" },
        0xEA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,D" },
        0xEB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,E" },
        0xEC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,H" },
        0xED => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,L" },
        0xEE => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 5,(HL)" },
        0xEF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 5,A" },
        0xF0 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,B" },
        0xF1 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,C" },
        0xF2 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,D" },
        0xF3 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,E" },
        0xF4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,H" },
        0xF5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,L" },
        0xF6 => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 6,(HL)" },
        0xF7 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 6,A" },
        0xF8 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,B" },
        0xF9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,C" },
        0xFA => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,D" },
        0xFB => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,E" },
        0xFC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,H" },
        0xFD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,L" },
        0xFE => Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SET 7,(HL)" },
        0xFF => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SET 7,A" },
    })
}

#[allow(clippy::too_many_lines, clippy::match_same_arms, clippy::unnecessary_wraps)]
fn index_instr(op: u8) -> Option<Instr> {
    Some(match op {
        0x09 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD {r},BC" },
        0x19 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD {r},DE" },
        0x21 => {
            Instr { dis: 0, imm: 2, read: 0, write: 0, repeat: false, mnemonic: "LD {r},{i:04H}h" }
        }
        0x22 => Instr {
            dis: 0,
            imm: 2,
            read: 0,
            write: 2,
            repeat: false,
            mnemonic: "LD ({i:04H}h),{r}",
        },
        0x23 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC {r}" },
        0x24 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC {r}h" },
        0x25 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC {r}h" },
        0x26 => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,{i:02H}h" }
        }
        0x29 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD {r},{r}" },
        0x2A => Instr {
            dis: 0,
            imm: 2,
            read: 2,
            write: 0,
            repeat: false,
            mnemonic: "LD {r},({i:04H}h)",
        },
        0x2B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC {r}" },
        0x2C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "INC {r}l" },
        0x2D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "DEC {r}l" },
        0x2E => {
            Instr { dis: 0, imm: 1, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,{i:02H}h" }
        }
        0x34 => {
            Instr { dis: 1, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "INC ({r}{d:+d})" }
        }
        0x35 => {
            Instr { dis: 1, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "DEC ({r}{d:+d})" }
        }
        0x36 => Instr {
            dis: 1,
            imm: 1,
            read: 0,
            write: 1,
            repeat: false,
            mnemonic: "LD ({r}{d:+d}),{i:02H}h",
        },
        0x39 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD {r},SP" },
        0x44 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,{r}h" },
        0x45 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD B,{r}l" },
        0x46 => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD B,({r}{d:+d})" }
        }
        0x4C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,{r}h" },
        0x4D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD C,{r}l" },
        0x4E => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD C,({r}{d:+d})" }
        }
        0x54 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,{r}h" },
        0x55 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD D,{r}l" },
        0x56 => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD D,({r}{d:+d})" }
        }
        0x5C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,{r}h" },
        0x5D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD E,{r}l" },
        0x5E => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD E,({r}{d:+d})" }
        }
        0x60 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,B" },
        0x61 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,C" },
        0x62 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,D" },
        0x63 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,E" },
        0x64 => {
            Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,{r}h" }
        }
        0x65 => {
            Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,{r}l" }
        }
        0x66 => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD H,({r}{d:+d})" }
        }
        0x67 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}h,A" },
        0x68 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,B" },
        0x69 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,C" },
        0x6A => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,D" },
        0x6B => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,E" },
        0x6C => {
            Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,{r}h" }
        }
        0x6D => {
            Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,{r}l" }
        }
        0x6E => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD L,({r}{d:+d})" }
        }
        0x6F => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD {r}l,A" },
        0x70 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),B" }
        }
        0x71 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),C" }
        }
        0x72 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),D" }
        }
        0x73 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),E" }
        }
        0x74 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),H" }
        }
        0x75 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),L" }
        }
        0x77 => {
            Instr { dis: 1, imm: 0, read: 0, write: 1, repeat: false, mnemonic: "LD ({r}{d:+d}),A" }
        }
        0x7C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,{r}h" },
        0x7D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD A,{r}l" },
        0x7E => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "LD A,({r}{d:+d})" }
        }
        0x84 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,{r}h" },
        0x85 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADD A,{r}l" },
        0x86 => Instr {
            dis: 1,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "ADD A,({r}{d:+d})",
        },
        0x8C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,{r}h" },
        0x8D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "ADC A,{r}l" },
        0x8E => Instr {
            dis: 1,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "ADC A,({r}{d:+d})",
        },
        0x94 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB {r}h" },
        0x95 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SUB {r}l" },
        0x96 => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "SUB ({r}{d:+d})" }
        }
        0x9C => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,{r}h" },
        0x9D => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "SBC A,{r}l" },
        0x9E => Instr {
            dis: 1,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "SBC A,({r}{d:+d})",
        },
        0xA4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND {r}h" },
        0xA5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "AND {r}l" },
        0xA6 => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "AND ({r}{d:+d})" }
        }
        0xAC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR {r}h" },
        0xAD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "XOR {r}l" },
        0xAE => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "XOR ({r}{d:+d})" }
        }
        0xB4 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR {r}h" },
        0xB5 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "OR {r}l" },
        0xB6 => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "OR ({r}{d:+d})" }
        }
        0xBC => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP {r}h" },
        0xBD => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "CP {r}l" },
        0xBE => {
            Instr { dis: 1, imm: 0, read: 1, write: 0, repeat: false, mnemonic: "CP ({r}{d:+d})" }
        }
        0xE1 => Instr { dis: 0, imm: 0, read: 2, write: 0, repeat: false, mnemonic: "POP {r}" },
        0xE3 => Instr { dis: 0, imm: 0, read: 2, write: 2, repeat: false, mnemonic: "EX (SP),{r}" },
        0xE5 => Instr { dis: 0, imm: 0, read: 0, write: -2, repeat: false, mnemonic: "PUSH {r}" },
        0xE9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "JP ({r})" },
        0xF9 => Instr { dis: 0, imm: 0, read: 0, write: 0, repeat: false, mnemonic: "LD SP,{r}" },
        _ => return None,
    })
}

#[allow(clippy::too_many_lines, clippy::match_same_arms, clippy::unnecessary_wraps)]
fn index_bit_instr(op: u8) -> Option<Instr> {
    Some(match op {
        0x00 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),B",
        },
        0x01 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),C",
        },
        0x02 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),D",
        },
        0x03 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),E",
        },
        0x04 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),H",
        },
        0x05 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),L",
        },
        0x06 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RLC ({r}{d:+d})" }
        }
        0x07 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RLC ({r}{d:+d}),A",
        },
        0x08 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),B",
        },
        0x09 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),C",
        },
        0x0A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),D",
        },
        0x0B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),E",
        },
        0x0C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),H",
        },
        0x0D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),L",
        },
        0x0E => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RRC ({r}{d:+d})" }
        }
        0x0F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RRC ({r}{d:+d}),A",
        },
        0x10 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),B" }
        }
        0x11 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),C" }
        }
        0x12 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),D" }
        }
        0x13 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),E" }
        }
        0x14 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),H" }
        }
        0x15 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),L" }
        }
        0x16 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d})" }
        }
        0x17 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RL ({r}{d:+d}),A" }
        }
        0x18 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),B" }
        }
        0x19 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),C" }
        }
        0x1A => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),D" }
        }
        0x1B => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),E" }
        }
        0x1C => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),H" }
        }
        0x1D => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),L" }
        }
        0x1E => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d})" }
        }
        0x1F => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "RR ({r}{d:+d}),A" }
        }
        0x20 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),B",
        },
        0x21 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),C",
        },
        0x22 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),D",
        },
        0x23 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),E",
        },
        0x24 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),H",
        },
        0x25 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),L",
        },
        0x26 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SLA ({r}{d:+d})" }
        }
        0x27 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLA ({r}{d:+d}),A",
        },
        0x28 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),B",
        },
        0x29 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),C",
        },
        0x2A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),D",
        },
        0x2B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),E",
        },
        0x2C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),H",
        },
        0x2D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),L",
        },
        0x2E => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SRA ({r}{d:+d})" }
        }
        0x2F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRA ({r}{d:+d}),A",
        },
        0x30 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),B",
        },
        0x31 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),C",
        },
        0x32 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),D",
        },
        0x33 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),E",
        },
        0x34 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),H",
        },
        0x35 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),L",
        },
        0x36 => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SLL ({r}{d:+d})" }
        }
        0x37 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SLL ({r}{d:+d}),A",
        },
        0x38 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),B",
        },
        0x39 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),C",
        },
        0x3A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),D",
        },
        0x3B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),E",
        },
        0x3C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),H",
        },
        0x3D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),L",
        },
        0x3E => {
            Instr { dis: 0, imm: 0, read: 1, write: 1, repeat: false, mnemonic: "SRL ({r}{d:+d})" }
        }
        0x3F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SRL ({r}{d:+d}),A",
        },
        0x40 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x41 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x42 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x43 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x44 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x45 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x46 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x47 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 0,({r}{d:+d})",
        },
        0x48 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x49 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x4A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x4B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x4C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x4D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x4E => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x4F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 1,({r}{d:+d})",
        },
        0x50 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x51 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x52 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x53 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x54 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x55 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x56 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x57 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 2,({r}{d:+d})",
        },
        0x58 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x59 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x5A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x5B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x5C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x5D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x5E => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x5F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 3,({r}{d:+d})",
        },
        0x60 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x61 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x62 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x63 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x64 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x65 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x66 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x67 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 4,({r}{d:+d})",
        },
        0x68 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x69 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x6A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x6B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x6C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x6D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x6E => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x6F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 5,({r}{d:+d})",
        },
        0x70 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x71 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x72 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x73 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x74 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x75 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x76 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x77 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 6,({r}{d:+d})",
        },
        0x78 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x79 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x7A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x7B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x7C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x7D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x7E => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x7F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 0,
            repeat: false,
            mnemonic: "BIT 7,({r}{d:+d})",
        },
        0x80 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),B",
        },
        0x81 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),C",
        },
        0x82 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),D",
        },
        0x83 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),E",
        },
        0x84 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),H",
        },
        0x85 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),L",
        },
        0x86 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d})",
        },
        0x87 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 0,({r}{d:+d}),A",
        },
        0x88 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),B",
        },
        0x89 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),C",
        },
        0x8A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),D",
        },
        0x8B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),E",
        },
        0x8C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),H",
        },
        0x8D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),L",
        },
        0x8E => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d})",
        },
        0x8F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 1,({r}{d:+d}),A",
        },
        0x90 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),B",
        },
        0x91 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),C",
        },
        0x92 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),D",
        },
        0x93 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),E",
        },
        0x94 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),H",
        },
        0x95 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),L",
        },
        0x96 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d})",
        },
        0x97 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 2,({r}{d:+d}),A",
        },
        0x98 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),B",
        },
        0x99 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),C",
        },
        0x9A => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),D",
        },
        0x9B => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),E",
        },
        0x9C => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),H",
        },
        0x9D => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),L",
        },
        0x9E => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d})",
        },
        0x9F => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 3,({r}{d:+d}),A",
        },
        0xA0 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),B",
        },
        0xA1 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),C",
        },
        0xA2 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),D",
        },
        0xA3 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),E",
        },
        0xA4 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),H",
        },
        0xA5 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),L",
        },
        0xA6 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d})",
        },
        0xA7 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 4,({r}{d:+d}),A",
        },
        0xA8 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),B",
        },
        0xA9 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),C",
        },
        0xAA => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),D",
        },
        0xAB => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),E",
        },
        0xAC => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),H",
        },
        0xAD => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),L",
        },
        0xAE => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d})",
        },
        0xAF => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 5,({r}{d:+d}),A",
        },
        0xB0 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),B",
        },
        0xB1 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),C",
        },
        0xB2 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),D",
        },
        0xB3 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),E",
        },
        0xB4 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),H",
        },
        0xB5 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),L",
        },
        0xB6 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d})",
        },
        0xB7 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 6,({r}{d:+d}),A",
        },
        0xB8 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),B",
        },
        0xB9 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),C",
        },
        0xBA => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),D",
        },
        0xBB => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),E",
        },
        0xBC => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),H",
        },
        0xBD => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),L",
        },
        0xBE => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d})",
        },
        0xBF => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "RES 7,({r}{d:+d}),A",
        },
        0xC0 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),B",
        },
        0xC1 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),C",
        },
        0xC2 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),D",
        },
        0xC3 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),E",
        },
        0xC4 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),H",
        },
        0xC5 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),L",
        },
        0xC6 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d})",
        },
        0xC7 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 0,({r}{d:+d}),A",
        },
        0xC8 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),B",
        },
        0xC9 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),C",
        },
        0xCA => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),D",
        },
        0xCB => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),E",
        },
        0xCC => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),H",
        },
        0xCD => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),L",
        },
        0xCE => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d})",
        },
        0xCF => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 1,({r}{d:+d}),A",
        },
        0xD0 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),B",
        },
        0xD1 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),C",
        },
        0xD2 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),D",
        },
        0xD3 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),E",
        },
        0xD4 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),H",
        },
        0xD5 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),L",
        },
        0xD6 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d})",
        },
        0xD7 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 2,({r}{d:+d}),A",
        },
        0xD8 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),B",
        },
        0xD9 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),C",
        },
        0xDA => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),D",
        },
        0xDB => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),E",
        },
        0xDC => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),H",
        },
        0xDD => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),L",
        },
        0xDE => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d})",
        },
        0xDF => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 3,({r}{d:+d}),A",
        },
        0xE0 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),B",
        },
        0xE1 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),C",
        },
        0xE2 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),D",
        },
        0xE3 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),E",
        },
        0xE4 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),H",
        },
        0xE5 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),L",
        },
        0xE6 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d})",
        },
        0xE7 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 4,({r}{d:+d}),A",
        },
        0xE8 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),B",
        },
        0xE9 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),C",
        },
        0xEA => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),D",
        },
        0xEB => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),E",
        },
        0xEC => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),H",
        },
        0xED => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),L",
        },
        0xEE => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d})",
        },
        0xEF => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 5,({r}{d:+d}),A",
        },
        0xF0 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),B",
        },
        0xF1 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),C",
        },
        0xF2 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),D",
        },
        0xF3 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),E",
        },
        0xF4 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),H",
        },
        0xF5 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),L",
        },
        0xF6 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d})",
        },
        0xF7 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 6,({r}{d:+d}),A",
        },
        0xF8 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),B",
        },
        0xF9 => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),C",
        },
        0xFA => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),D",
        },
        0xFB => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),E",
        },
        0xFC => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),H",
        },
        0xFD => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),L",
        },
        0xFE => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d})",
        },
        0xFF => Instr {
            dis: 0,
            imm: 0,
            read: 1,
            write: 1,
            repeat: false,
            mnemonic: "SET 7,({r}{d:+d}),A",
        },
    })
}

/// The table selected by the accumulated prefix, and the IX/IY
/// register name for the instruction templates' `{r}` placeholder.
#[allow(clippy::match_same_arms)]
fn table_by_prefix(prefix: u32) -> (fn(u8) -> Option<Instr>, &'static str) {
    match prefix {
        0 => (main_instr, ""),
        0xED => (extended_instr, ""),
        0xCB => (bit_instr, ""),
        0xDD => (index_instr, "IX"),
        0xFD => (index_instr, "IY"),
        0xDDCB => (index_bit_instr, "IX"),
        0xFDCB => (index_bit_instr, "IY"),
        _ => (main_instr, ""),
    }
}
