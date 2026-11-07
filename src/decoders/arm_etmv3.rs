// Counterpart in sigrok:
//   libsigrokdecode/decoders/arm_etmv3/pd.py,
//   Copyright (C) 2015 Petteri Aimonen <jpa@sigrok.mail.kapsi.fi>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ETMv3 signal protocol packet formats and the
//   decoder's documented classes and texts.

//! ARM ETMv3: branch, synchronization, trigger and header packets.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};
use std::collections::HashMap;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_TRACE: usize = 0;
const ANN_BRANCH: usize = 1;
const ANN_EXCEPTION: usize = 2;
const ANN_EXECUTION: usize = 3;
#[allow(dead_code)]
const ANN_DATA: usize = 4;
const ANN_PC: usize = 5;
const ANN_INSTR_E: usize = 6;
const ANN_INSTR_N: usize = 7;
const ANN_SOURCE: usize = 8;
const ANN_LOCATION: usize = 9;
const ANN_FUNCTION: usize = 10;

// See ETMv3 Signal Protocol table 7-11: 'Encoding of Exception[8:0]'.
const EXC_NAMES: [&str; 21] = [
    "No exception",
    "IRQ1",
    "IRQ2",
    "IRQ3",
    "IRQ4",
    "IRQ5",
    "IRQ6",
    "IRQ7",
    "IRQ0",
    "UsageFault",
    "NMI",
    "SVC",
    "DebugMon",
    "MemManage",
    "PendSV",
    "SysTick",
    "Reserved",
    "Reset",
    "BusFault",
    "Reserved",
    "Reserved",
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "arm_etmv3",
    name: "ARM ETMv3",
    longname: "ARM Embedded Trace Macroblock v3",
    desc: "ARM ETM v3 instruction trace protocol.",
    license: "gplv2+",
    inputs: &["uart"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "objdump",
            desc: "objdump path",
            default: OptionValue::s("arm-none-eabi-objdump"),
            values: &[],
        },
        OptionDesc {
            id: "objdump_opts",
            desc: "objdump options",
            default: OptionValue::s("-lSC"),
            values: &[],
        },
        OptionDesc { id: "elffile", desc: ".elf path", default: OptionValue::s(""), values: &[] },
        OptionDesc {
            id: "branch_enc",
            desc: "Branch encoding",
            default: OptionValue::s("alternative"),
            values: &[OptionValue::s("alternative"), OptionValue::s("original")],
        },
    ],
    annotations: &[
        AnnClass { id: "trace", desc: "Trace info" },
        AnnClass { id: "branch", desc: "Branches" },
        AnnClass { id: "exception", desc: "Exceptions" },
        AnnClass { id: "execution", desc: "Instruction execution" },
        AnnClass { id: "data", desc: "Data access" },
        AnnClass { id: "pc", desc: "Program counter" },
        AnnClass { id: "instr_e", desc: "Executed instructions" },
        AnnClass { id: "instr_n", desc: "Not executed instructions" },
        AnnClass { id: "source", desc: "Source code" },
        AnnClass { id: "location", desc: "Current location" },
        AnnClass { id: "function", desc: "Current function" },
    ],
    annotation_rows: &[
        AnnRow { id: "trace", desc: "Trace info", classes: &[0] },
        AnnRow { id: "flow", desc: "Code flow", classes: &[1, 2, 3] },
        AnnRow { id: "data", desc: "Data access", classes: &[4] },
        AnnRow { id: "pc", desc: "Program counter", classes: &[5] },
        AnnRow { id: "instruction", desc: "Instructions", classes: &[6, 7] },
        AnnRow { id: "source", desc: "Source code", classes: &[8] },
        AnnRow { id: "location", desc: "Current location", classes: &[9] },
        AnnRow { id: "function", desc: "Current function", classes: &[10] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes packets of\n\
the ARMv7m Embedded Trace Macroblock v3.x.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Etmv3::new())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CpuState {
    Arm,
    Thumb,
    Jazelle,
}

impl CpuState {
    fn as_str(self) -> &'static str {
        match self {
            CpuState::Arm => "arm",
            CpuState::Thumb => "thumb",
            CpuState::Jazelle => "jazelle",
        }
    }
}

/// The exception information of a branch packet.
#[derive(Clone, Copy)]
struct ExcInfo {
    ns: u64,
    exc: u64,
    cancel: u64,
    altisa: u64,
    hyp: u64,
    resume: u64,
}

struct BranchInfo {
    addr: u64,
    _addrlen: usize,
    cpu_state: CpuState,
    exc_info: Option<ExcInfo>,
}

/// A source location that is extended while it does not change.
struct Loc {
    ss: u64,
    es: u64,
    loc: Option<String>,
    src: Option<String>,
}

struct Func {
    ss: u64,
    es: u64,
    func: Option<String>,
}

type Ann = (usize, Vec<String>);

enum Outcome {
    Wait,
    Emit(Ann),
}

struct Etmv3 {
    buf: Vec<u8>,
    syncbuf: Vec<u8>,
    prevsample: u64,
    startsample: u64,
    byte_len: u64,
    last_branch: u64,
    cpu_state: CpuState,
    current_pc: u64,
    current_loc: Option<Loc>,
    current_func: Option<Func>,
    next_instr_lookup: HashMap<u64, (u64, u64)>,
    file_lookup: HashMap<u64, String>,
    func_lookup: HashMap<u64, String>,
    disasm_lookup: HashMap<u64, String>,
    source_lookup: HashMap<u64, Option<String>>,
    branch_enc: String,
}

impl Etmv3 {
    fn new() -> Self {
        Etmv3 {
            buf: Vec::new(),
            syncbuf: Vec::new(),
            prevsample: 0,
            startsample: 0,
            byte_len: 0,
            last_branch: 0,
            cpu_state: CpuState::Arm,
            current_pc: 0,
            current_loc: None,
            current_func: None,
            next_instr_lookup: HashMap::new(),
            file_lookup: HashMap::new(),
            func_lookup: HashMap::new(),
            disasm_lookup: HashMap::new(),
            source_lookup: HashMap::new(),
            branch_enc: "alternative".into(),
        }
    }

    fn flush_current_loc(&mut self, ctx: &mut dyn Context) {
        if let Some(loc) = self.current_loc.take() {
            if let Some(text) = loc.loc.filter(|t| !t.is_empty()) {
                ctx.put_ann(loc.ss, loc.es, ANN_LOCATION, vec![text]);
            }
            if let Some(text) = loc.src.filter(|t| !t.is_empty()) {
                ctx.put_ann(loc.ss, loc.es, ANN_SOURCE, vec![text]);
            }
        }
    }

    fn flush_current_func(&mut self, ctx: &mut dyn Context) {
        if let Some(func) = self.current_func.take() {
            if let Some(text) = func.func.filter(|t| !t.is_empty()) {
                ctx.put_ann(func.ss, func.es, ANN_FUNCTION, vec![text]);
            }
        }
    }

    /// `instructions_executed`: advances the PC over executed and skipped
    /// instructions.
    fn instructions_executed(&mut self, ctx: &mut dyn Context, exec_status: &[bool]) {
        if exec_status.is_empty() {
            return;
        }

        let tdelta = ((self.prevsample.saturating_sub(self.startsample) as f64)
            / exec_status.len() as f64)
            .max(1.0);

        for (i, &executed) in exec_status.iter().enumerate() {
            let pc = self.current_pc;
            let default_next = if self.cpu_state == CpuState::Thumb { pc + 2 } else { pc + 4 };
            let (target_n, target_e) =
                self.next_instr_lookup.get(&pc).copied().unwrap_or((default_next, default_next));
            let ss = self.startsample + py_round(tdelta * i as f64) as u64;
            let es = self.startsample + py_round(tdelta * (i + 1) as f64) as u64;

            ctx.put_ann(
                ss,
                es,
                ANN_PC,
                vec![format!("PC 0x{pc:08x}"), format!("0x{pc:08x}"), format!("{pc:08x}")],
            );

            let new_loc = self.file_lookup.get(&pc).cloned();
            let new_src = self.source_lookup.get(&pc).cloned().flatten();
            let new_dis = self.disasm_lookup.get(&pc).cloned();
            let new_func = self.func_lookup.get(&pc).cloned();

            // Report source line only when it changes.
            if let Some(loc) = &self.current_loc {
                if new_loc != loc.loc || new_src != loc.src {
                    self.flush_current_loc(ctx);
                }
            }

            if let Some(loc) = &mut self.current_loc {
                loc.es = es;
            } else {
                self.current_loc = Some(Loc { ss, es, loc: new_loc, src: new_src });
            }

            // Report function name only when it changes.
            if let Some(func) = &self.current_func {
                if new_func != func.func {
                    self.flush_current_func(ctx);
                }
            }

            if let Some(func) = &mut self.current_func {
                func.es = es;
            } else {
                self.current_func = Some(Func { ss, es, func: new_func });
            }

            // Report instruction every time.
            if let Some(dis) = new_dis.filter(|d| !d.is_empty()) {
                let first = dis.split_whitespace().next().unwrap_or("").to_owned();
                let texts = if executed {
                    vec![format!("Executed: {dis}"), dis.clone(), first]
                } else {
                    vec![format!("Not executed: {dis}"), dis.clone(), first]
                };
                ctx.put_ann(ss, es, if executed { ANN_INSTR_E } else { ANN_INSTR_N }, texts);
            }

            self.current_pc = if executed { target_e } else { target_n };
        }
    }

    fn fallback(&self) -> Outcome {
        let ptype = packet_type(self.buf[0]);
        let bytes: Vec<String> = self.buf.iter().map(|b| format!("{b:02x}")).collect();
        Outcome::Emit((ANN_TRACE, vec![format!("Unhandled {ptype}: {}", bytes.join(" "))]))
    }

    fn handle_a_sync(&self) -> Outcome {
        if self.buf.last() == Some(&0x80) {
            Outcome::Emit((ANN_TRACE, vec!["Synchronization".into()]))
        } else {
            Outcome::Wait
        }
    }

    fn handle_exception_exit(&self) -> Outcome {
        Outcome::Emit((ANN_EXCEPTION, vec!["Exception exit".into()]))
    }

    fn handle_exception_entry(&self) -> Outcome {
        Outcome::Emit((ANN_EXCEPTION, vec!["Exception entry".into()]))
    }

    fn handle_i_sync(&mut self, ctx: &mut dyn Context) -> Result<Outcome> {
        let contextid_bytes = 0usize; // This is the default ETM config.

        if self.buf.len() < 6 {
            return Ok(Outcome::Wait); // Packet definitely not full yet.
        }

        let (cyclecount, idx) = if self.buf[0] == 0x08 {
            (None, 1 + contextid_bytes)
        } else {
            let (count, cyclen) = parse_varint(&self.buf[1..6]);
            (Some(count), 1 + cyclen + contextid_bytes)
        };

        if self.buf.len() <= idx + 4 {
            return Ok(Outcome::Wait);
        }
        let infobyte = self.buf[idx];
        let addr = parse_uint(&self.buf[idx + 1..idx + 5]);

        let reasoncode = (infobyte >> 5) & 3;
        let reason = ["Periodic", "Tracing enabled", "After overflow", "Exit from debug"]
            [usize::from(reasoncode)];
        let jazelle = (infobyte >> 4) & 1;
        let _nonsec = (infobyte >> 3) & 1;
        let _altisa = (infobyte >> 2) & 1;
        let _hypervisor = (infobyte >> 1) & 1;
        let thumb = addr & 1;
        let addr = addr & 0xFFFF_FFFE;

        if reasoncode == 0 && self.current_pc != addr {
            ctx.put_ann(
                self.startsample,
                self.prevsample,
                ANN_TRACE,
                vec![format!(
                    "WARN: Unexpected PC change 0x{:08x} -> 0x{addr:08x}",
                    self.current_pc
                )],
            );
        } else if reasoncode != 0 {
            // Reset location when the trace has been interrupted.
            self.flush_current_loc(ctx);
            self.flush_current_func(ctx);
        }

        self.last_branch = addr;
        self.current_pc = addr;

        if jazelle != 0 {
            self.cpu_state = CpuState::Jazelle;
        } else if thumb != 0 {
            self.cpu_state = CpuState::Thumb;
        } else {
            self.cpu_state = CpuState::Arm;
        }

        let cycstr = cyclecount.map_or_else(String::new, |count| format!(", cyclecount {count}"));

        if infobyte & 0x80 != 0 {
            // LSIP packet
            ctx.put_ann(
                self.startsample,
                self.prevsample,
                ANN_TRACE,
                vec!["WARN: LSIP I-Sync packet not implemented".into()],
            );
        }

        Ok(Outcome::Emit((
            ANN_TRACE,
            vec![
                format!(
                    "I-Sync: {reason}, PC 0x{addr:08x}, {} state{cycstr}",
                    self.cpu_state.as_str()
                ),
                format!("I-Sync: {reason} 0x{addr:08x}"),
            ],
        )))
    }

    fn handle_trigger(&self) -> Outcome {
        Outcome::Emit((ANN_TRACE, vec!["Trigger event".into(), "Trigger".into()]))
    }

    fn handle_p_header(&mut self, ctx: &mut dyn Context) -> Result<Outcome> {
        // Only non cycle-accurate mode supported.
        if self.buf[0] & 0x83 == 0x80 {
            let n = (self.buf[0] >> 6) & 1;
            let e = (self.buf[0] >> 2) & 15;

            let mut status = vec![true; usize::from(e)];
            status.extend(std::iter::repeat_n(false, usize::from(n)));
            self.instructions_executed(ctx, &status);

            if n != 0 {
                Ok(Outcome::Emit((
                    ANN_EXECUTION,
                    vec![
                        format!("{e} instructions executed, {n} skipped due to condition codes"),
                        format!("{e} ins exec, {n} skipped"),
                        format!("{e}E,{n}N"),
                    ],
                )))
            } else {
                Ok(Outcome::Emit((
                    ANN_EXECUTION,
                    vec![
                        format!("{e} instructions executed"),
                        format!("{e} ins exec"),
                        format!("{e}E"),
                    ],
                )))
            }
        } else if self.buf[0] & 0xF3 == 0x82 {
            let i1 = (self.buf[0] >> 3) & 1;
            let i2 = (self.buf[0] >> 2) & 1;
            self.instructions_executed(ctx, &[i1 == 0, i2 == 0]);
            let txt1 = ["executed", "skipped"];
            let txt2 = ["E", "S"];
            Ok(Outcome::Emit((
                ANN_EXECUTION,
                vec![
                    format!(
                        "Instruction 1 {}, instruction 2 {}",
                        txt1[usize::from(i1)],
                        txt1[usize::from(i2)]
                    ),
                    format!("I1 {}, I2 {}", txt2[usize::from(i1)], txt2[usize::from(i2)]),
                    format!("{},{}", txt2[usize::from(i1)], txt2[usize::from(i2)]),
                ],
            )))
        } else {
            Ok(self.fallback())
        }
    }

    fn handle_branch(&mut self, ctx: &mut dyn Context) -> Result<Outcome> {
        if self.buf.last().is_some_and(|b| b & 0x80 != 0) {
            return Ok(Outcome::Wait); // Not complete yet.
        }

        let Some(brinfo) =
            parse_branch_addr(&self.buf, self.last_branch, self.cpu_state, &self.branch_enc)?
        else {
            return Ok(Outcome::Wait); // Not complete yet.
        };

        let BranchInfo { addr, _addrlen: _, cpu_state, exc_info } = brinfo;
        self.last_branch = addr;
        self.current_pc = addr;

        let mut txt = String::new();

        if cpu_state != self.cpu_state {
            txt.push_str(&format!(", to {} state", cpu_state.as_str()));
            self.cpu_state = cpu_state;
        }

        let mut annidx = ANN_BRANCH;

        if let Some(exc) = exc_info {
            annidx = ANN_EXCEPTION;
            if exc.ns != 0 {
                txt.push_str(", to non-secure state");
            }
            if exc.exc != 0 {
                if exc.exc < exc_names_len() {
                    txt.push_str(&format!(", exception {}", exc_name(exc.exc)));
                } else {
                    txt.push_str(&format!(", exception 0x{:02x}", exc.exc));
                }
            }
            if exc.cancel != 0 {
                txt.push_str(", instr cancelled");
            }
            if exc.altisa != 0 {
                txt.push_str(", to AltISA");
            }
            if exc.hyp != 0 {
                txt.push_str(", to hypervisor");
            }
            if exc.resume != 0 {
                txt.push_str(&format!(", instr resume 0x{:02x}", exc.resume));
            }
        }

        // The branch handler only looks at the packet; ctx is unused here.
        let _ = ctx;

        Ok(Outcome::Emit((
            annidx,
            vec![format!("Branch to 0x{addr:08x}{txt}"), format!("B 0x{addr:08x}{txt}")],
        )))
    }

    /// `load_objdump`: the code lookup tables from the disassembly.
    fn load_objdump(&mut self, config: &Config) {
        let objdump = config.str("objdump", "arm-none-eabi-objdump");
        let elffile = config.str("elffile", "");
        if objdump.is_empty() || elffile.is_empty() {
            return;
        }
        let mut command = std::process::Command::new(objdump);
        command.args(config.str("objdump_opts", "-lSC").split_whitespace());
        command.arg(elffile);
        let Ok(output) = command.output() else { return };
        if !output.status.success() {
            return;
        }
        let disasm = String::from_utf8_lossy(&output.stdout);

        let mut prev_src: Option<String> = None;
        let mut prev_file = String::new();
        let mut prev_func = String::new();

        for line in disasm.split('\n') {
            if let Some((addr, raw, disas)) = parse_instruction(line) {
                let ilen = raw.chars().filter(|&c| c != ' ').count() / 2;
                self.disasm_lookup.insert(addr, disas.clone());
                self.source_lookup.insert(addr, prev_src.clone());
                self.file_lookup.insert(addr, prev_file.clone());
                self.func_lookup.insert(addr, prev_func.clone());

                // Next address in direct sequence.
                let next_n = addr + ilen as u64;

                // Next address if branch is taken.
                let next_e = branch_target(&disas).unwrap_or(next_n);

                self.next_instr_lookup.insert(addr, (next_n, next_e));
            } else if let Some(func) = parse_function(line) {
                prev_func = func;
                prev_src = None;
            } else if let Some(file) = parse_file(line) {
                prev_file = file;
                prev_src = None;
            } else {
                prev_src = Some(line.trim().to_owned());
            }
        }
    }
}

/// `len(exc_names)`: 21 named entries, then `IRQ8` through `IRQ495`.
fn exc_names_len() -> u64 {
    21 + (495 - 8 + 1)
}

/// `exc_names[exc]`.
fn exc_name(exc: u64) -> String {
    if exc < EXC_NAMES.len() as u64 {
        EXC_NAMES[exc as usize].to_owned()
    } else {
        format!("IRQ{}", i64::try_from(exc).unwrap_or(0) - 13)
    }
}

/// `get_packet_type`: the ETMv3 packet a first byte starts.
fn packet_type(byte: u8) -> &'static str {
    if byte & 0x01 == 0x01 {
        "branch"
    } else if byte == 0x00 {
        "a_sync"
    } else if byte == 0x04 {
        "cyclecount"
    } else if byte == 0x08 {
        "i_sync"
    } else if byte == 0x0C {
        "trigger"
    } else if matches!(byte & 0xF3, 0x20 | 0x40 | 0x60) {
        "ooo_data"
    } else if byte == 0x50 {
        "store_failed"
    } else if byte == 0x70 {
        "i_sync"
    } else if matches!(byte & 0xDF, 0x54 | 0x58 | 0x5C) {
        "ooo_place"
    } else if byte == 0x3C {
        "vmid"
    } else if byte & 0xD3 == 0x02 {
        "data"
    } else if byte & 0xFB == 0x42 {
        "timestamp"
    } else if byte == 0x62 {
        "data_suppressed"
    } else if byte == 0x66 {
        "ignore"
    } else if byte & 0xEF == 0x6A {
        "value_not_traced"
    } else if byte == 0x6E {
        "context_id"
    } else if byte == 0x76 {
        "exception_exit"
    } else if byte == 0x7E {
        "exception_entry"
    } else if byte & 0x81 == 0x80 {
        "p_header"
    } else {
        "unknown"
    }
}

/// `parse_varint`: a little-endian base-128 integer whose top bit
/// continues the value. Returns the value and the bytes consumed.
fn parse_varint(bytes: &[u8]) -> (u64, usize) {
    let mut v = 0u64;
    for (i, &b) in bytes.iter().enumerate() {
        v |= u64::from(b & 0x7F) << (i * 7);
        if b & 0x80 == 0 {
            return (v, i + 1);
        }
    }
    (v, bytes.len())
}

/// `parse_uint`: a little-endian integer.
fn parse_uint(bytes: &[u8]) -> u64 {
    let mut v = 0u64;
    for (i, &b) in bytes.iter().enumerate() {
        v |= u64::from(b) << (i * 8);
    }
    v
}

/// `parse_exc_info`: the exception information bytes of a branch packet.
fn parse_exc_info(bytes: &[u8]) -> Option<ExcInfo> {
    if bytes.is_empty() {
        return None;
    }

    let (mut excv, exclen) = parse_varint(bytes);
    if bytes[exclen - 1] & 0x80 != 0 {
        return None; // Exception info not complete.
    }

    if exclen == 2 && excv & (1 << 13) != 0 {
        // Exception byte 1 was skipped, fix up the decoding.
        excv = (excv & 0x7F) | ((excv & 0x3F80) << 7);
    }

    Some(ExcInfo {
        ns: excv & 1,
        exc: ((excv >> 1) & 0x0F) | ((excv >> 7) & 0x1F0),
        cancel: (excv >> 5) & 1,
        altisa: (excv >> 6) & 1,
        hyp: (excv >> 12) & 1,
        resume: (excv >> 14) & 0x0F,
    })
}

/// `parse_branch_addr`: the encoded branch address.
fn parse_branch_addr(
    bytes: &[u8],
    ref_addr: u64,
    cpu_state: CpuState,
    branch_enc: &str,
) -> Result<Option<BranchInfo>> {
    let (mut addr, addrlen) = parse_varint(bytes);

    if bytes[addrlen - 1] & 0x80 != 0 {
        return Ok(None); // Branch address not complete.
    }

    let mut addr_bits = 7 * addrlen;

    let mut have_exc_info = false;
    if branch_enc == "original" {
        if addrlen == 5 && bytes[4] & 0x40 != 0 {
            have_exc_info = true;
        }
    } else if branch_enc == "alternative" {
        addr_bits -= 1; // Top bit of address indicates exc_info.
        if addrlen >= 2 && addr & (1 << addr_bits) != 0 {
            have_exc_info = true;
            addr &= !(1 << addr_bits);
        }
    }

    let mut exc_info = None;
    if have_exc_info {
        exc_info = parse_exc_info(&bytes[addrlen..]);
        if exc_info.is_none() {
            return Ok(None); // Exception info not complete.
        }
    }

    let mut cpu_state = cpu_state;
    if addrlen == 5 {
        // Possible change in CPU state.
        if bytes[4] & 0xB8 == 0x08 {
            cpu_state = CpuState::Arm;
        } else if bytes[4] & 0xB0 == 0x10 {
            cpu_state = CpuState::Thumb;
        } else if bytes[4] & 0xA0 == 0x20 {
            cpu_state = CpuState::Jazelle;
        } else {
            return Err(Error::arg(format!("Unhandled branch byte 4: 0x{:02x}", bytes[4])));
        }
    }

    // Shift the address according to current CPU state.
    match cpu_state {
        CpuState::Arm => {
            addr = (addr & 0xFFFF_FFFE) << 1;
            addr_bits += 1;
        }
        CpuState::Thumb => {
            addr &= 0xFFFF_FFFE;
        }
        CpuState::Jazelle => {
            addr = (addr & 0x000F_FFFF_FFFE) >> 1;
            addr_bits -= 1;
        }
    }

    // If the address wasn't full, fill in with the previous address.
    if addrlen < 5 {
        let mask = if addr_bits >= 32 { 0 } else { 0xFFFF_FFFFu64 << addr_bits };
        addr |= ref_addr & mask;
    }

    Ok(Some(BranchInfo { addr, _addrlen: addrlen, cpu_state, exc_info }))
}

/// Python's `round` for a non-negative float: halves to even.
fn py_round(x: f64) -> f64 {
    let floor = x.floor();
    let diff = x - floor;
    if diff > 0.5 {
        floor + 1.0
    } else if diff < 0.5 {
        floor
    } else if (floor as i64) % 2 == 0 {
        floor
    } else {
        floor + 1.0
    }
}

/// A line of `objdump -d` output: address, raw bytes and instruction.
fn parse_instruction(line: &str) -> Option<(u64, String, String)> {
    let rest = line.trim_start_matches(char::is_whitespace);
    let colon = rest.find(':')?;
    let addr_text = &rest[..colon];
    if addr_text.is_empty() || !addr_text.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    let after = rest[colon + 1..].strip_prefix('\t')?;
    let after = after.trim_start_matches('\t');
    let run_end = after.find(|c: char| !(c.is_ascii_hexdigit() || c == ' ')).unwrap_or(after.len());
    if run_end == 0 {
        return None;
    }
    let raw = &after[..run_end];
    let after2 = after[run_end..].strip_prefix('\t')?;
    let after2 = after2.trim_start_matches('\t');
    let first = after2.chars().next()?;
    if !first.is_ascii_alphabetic() {
        return None;
    }
    let semi = after2.find(';').unwrap_or(after2.len());
    let group = &after2[..semi];
    if group.chars().count() < 2 {
        return None;
    }
    let disas = group.trim().replace('\t', " ");
    Some((u64::from_str_radix(addr_text, 16).ok()?, raw.to_owned(), disas))
}

/// `branchpat`: the target of a direct branch instruction.
fn branch_target(disas: &str) -> Option<u64> {
    let split = disas.find(char::is_whitespace)?;
    let mnemonic = &disas[..split];
    let rest = disas[split..].trim_start_matches(char::is_whitespace);
    let base =
        mnemonic.strip_suffix(".w").or_else(|| mnemonic.strip_suffix(".n")).unwrap_or(mnemonic);
    let known = matches!(base, "b" | "bl" | "cbnz" | "cbz")
        || (base.len() == 3 && base.starts_with('b'))
        || (base.len() == 4 && base.starts_with("bl"));
    if !known {
        return None;
    }
    // `(?:r[0-9]+,\s*)?`
    let rest = if let Some(after_r) = rest.strip_prefix('r') {
        let digits = after_r.chars().take_while(char::is_ascii_digit).count();
        if digits > 0 {
            let after_digits = &after_r[digits..];
            after_digits
                .strip_prefix(',')
                .map_or(rest, |r| r.trim_start_matches(char::is_whitespace))
        } else {
            rest
        }
    } else {
        rest
    };
    let end = rest.find(|c: char| !c.is_ascii_hexdigit()).unwrap_or(rest.len());
    if end == 0 {
        return None;
    }
    u64::from_str_radix(&rest[..end], 16).ok()
}

/// A `00000000 <function>:` line: the function name.
fn parse_function(line: &str) -> Option<String> {
    let hex_end = line.find(|c: char| !c.is_ascii_hexdigit()).unwrap_or(line.len());
    if hex_end == 0 {
        return None;
    }
    let rest = line[hex_end..].trim_start_matches(char::is_whitespace);
    let rest = rest.strip_prefix('<')?;
    let end = rest.find('>')?;
    if end == 0 || !rest[end + 1..].starts_with(':') {
        return None;
    }
    Some(rest[..end].to_owned())
}

/// A `path/file.c:123` line: the `file:line` part.
fn parse_file(line: &str) -> Option<String> {
    let token_end = line.find(char::is_whitespace).unwrap_or(line.len());
    let token = &line[..token_end];
    for (i, c) in token.char_indices().rev() {
        if c != '/' && c != '\\' {
            continue;
        }
        let suffix = &token[i + 1..];
        let Some(colon) = suffix.find(':') else { continue };
        let name = &suffix[..colon];
        if name.is_empty()
            || !name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-'))
        {
            continue;
        }
        let digits = &suffix[colon + 1..];
        let len = digits.chars().take_while(char::is_ascii_digit).count();
        if len > 0 {
            return Some(format!("{}:{}", name, &digits[..len]));
        }
    }
    None
}

impl Decoder for Etmv3 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Etmv3::new();
        self.branch_enc = config.str("branch_enc", "alternative").to_owned();
        self.load_objdump(config);
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("arm_etmv3 is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, pdata) = unpack_uart(item)?;

        if ptype != "DATA" {
            return Ok(());
        }
        let value = data_value(pdata)?;

        // Reset packet if there is a long pause between bytes. This helps
        // getting the initial synchronization.
        self.byte_len = es.saturating_sub(ss);
        if ss > self.prevsample && ss - self.prevsample > 16 * self.byte_len {
            self.flush_current_loc(ctx);
            self.flush_current_func(ctx);
            self.buf.clear();
        }
        self.prevsample = es;

        self.buf.push(value);

        // Store the start time of the packet.
        if self.buf.len() == 1 {
            self.startsample = ss;
        }

        // Keep a separate buffer for detection of sync packets; they
        // override everything else, so we can regain sync even if some
        // packets are corrupted.
        self.syncbuf = self.syncbuf.iter().rev().take(4).rev().copied().collect();
        self.syncbuf.push(value);
        if self.syncbuf == [0x00, 0x00, 0x00, 0x00, 0x80] {
            self.buf = self.syncbuf.clone();
            self.syncbuf.clear();
        }

        // See if it is ready to be decoded.
        let ptype = packet_type(self.buf[0]);
        let outcome = match ptype {
            "a_sync" => self.handle_a_sync(),
            "exception_exit" => self.handle_exception_exit(),
            "exception_entry" => self.handle_exception_entry(),
            "i_sync" => self.handle_i_sync(ctx)?,
            "trigger" => self.handle_trigger(),
            "p_header" => self.handle_p_header(ctx)?,
            "branch" => self.handle_branch(ctx)?,
            _ => self.fallback(),
        };

        if let Outcome::Emit((class, texts)) = outcome {
            ctx.put_ann(self.startsample, es, class, texts);
            self.buf.clear();
        }
        Ok(())
    }
}

/// Unpacks `[ptype, rxtx, pdata]`; the UART output tuple.
fn unpack_uart(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(items) = item else {
        return Err(Error::arg("cannot unpack non-sequence UART item"));
    };
    if items.len() != 3 {
        return Err(Error::arg("too many values to unpack (UART item)"));
    }
    let Item::Str(ptype) = &items[0] else {
        return Err(Error::arg("UART packet type is not a string"));
    };
    Ok((ptype, &items[2]))
}

/// `pdata[0]`: the byte value of a DATA item.
fn data_value(pdata: &Item) -> Result<u8> {
    let Item::List(items) = pdata else {
        return Err(Error::arg("UART data is not a sequence"));
    };
    match items.first() {
        Some(Item::Int(v)) => Ok(*v as u8),
        _ => Err(Error::arg("UART data has no byte value")),
    }
}
