// Counterpart in sigrok:
//   libsigrokdecode/decoders/arm_itm/pd.py,
//   Copyright (C) 2015 Petteri Aimonen <jpa@sigrok.mail.kapsi.fi>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ITM/DWT packet formats and the decoder's
//   documented classes and texts.
#![allow(
    clippy::collapsible_if,
    clippy::collapsible_match,
    clippy::too_many_lines,
    clippy::unused_self,
    clippy::verbose_bit_mask
)]

//! ARM ITM: instrumentation and DWT trace packets.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};
use std::collections::HashMap;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_TRACE: usize = 0;
const ANN_TIMESTAMP: usize = 1;
const ANN_SOFTWARE: usize = 2;
const ANN_DWT_EVENT: usize = 3;
const ANN_DWT_WATCHPOINT: usize = 4;
const ANN_DWT_EXC: usize = 5;
const ANN_DWT_PC: usize = 6;
const ANN_MODE_THREAD: usize = 7;
const ANN_MODE_IRQ: usize = 8;
const ANN_MODE_EXC: usize = 9;
const ANN_LOCATION: usize = 10;
const ANN_FUNCTION: usize = 11;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "arm_itm",
    name: "ARM ITM",
    longname: "ARM Instrumentation Trace Macroblock",
    desc: "ARM Cortex-M / ARMv7m ITM trace protocol.",
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
    ],
    annotations: &[
        AnnClass { id: "trace", desc: "Trace information" },
        AnnClass { id: "timestamp", desc: "Timestamp" },
        AnnClass { id: "software", desc: "Software message" },
        AnnClass { id: "dwt_event", desc: "DWT event" },
        AnnClass { id: "dwt_watchpoint", desc: "DWT watchpoint" },
        AnnClass { id: "dwt_exc", desc: "Exception trace" },
        AnnClass { id: "dwt_pc", desc: "Program counter" },
        AnnClass { id: "mode_thread", desc: "Current mode: thread" },
        AnnClass { id: "mode_irq", desc: "Current mode: IRQ" },
        AnnClass { id: "mode_exc", desc: "Current mode: Exception" },
        AnnClass { id: "location", desc: "Current location" },
        AnnClass { id: "function", desc: "Current function" },
    ],
    annotation_rows: &[
        AnnRow { id: "trace", desc: "Trace information", classes: &[0, 1] },
        AnnRow { id: "software", desc: "Software trace", classes: &[2] },
        AnnRow { id: "dwt_event", desc: "DWT event", classes: &[3] },
        AnnRow { id: "dwt_watchpoint", desc: "DWT watchpoint", classes: &[4] },
        AnnRow { id: "dwt_exc", desc: "Exception trace", classes: &[5] },
        AnnRow { id: "dwt_pc", desc: "Program counter", classes: &[6] },
        AnnRow { id: "mode", desc: "Current mode", classes: &[7, 8, 9] },
        AnnRow { id: "location", desc: "Current location", classes: &[10] },
        AnnRow { id: "function", desc: "Current function", classes: &[11] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' or 'arm_tpiu' PD and decodes the\n\
ARM Cortex-M processor trace data from Instrumentation Trace Macroblock.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Itm::new())
}

/// ARM exception numbers to their names, as `ARM_EXCEPTIONS`.
fn arm_exception(excnum: u32) -> Option<&'static str> {
    Some(match excnum {
        0 => "Thread",
        1 => "Reset",
        2 => "NMI",
        3 => "HardFault",
        4 => "MemManage",
        5 => "BusFault",
        6 => "UsageFault",
        11 => "SVCall",
        12 => "Debug Monitor",
        14 => "PendSV",
        15 => "SysTick",
        _ => return None,
    })
}

/// A run of joined printable software characters, by PID.
struct SwPacket {
    ss: u64,
    prevtime: u64,
    text: String,
}

/// What a packet handler decided to do.
enum Outcome {
    /// The packet is not complete yet.
    Wait,
    /// The packet was consumed without an annotation.
    Silence,
    /// Emit this annotation.
    Emit((usize, Vec<String>)),
}

struct Itm {
    buf: Vec<u8>,
    syncbuf: Vec<u8>,
    /// Insertion-ordered, as a Python dict; a `None` entry is a pushed
    /// packet that stays in its original position when it comes back.
    swpackets: Vec<(i64, Option<SwPacket>)>,
    prevsample: u64,
    startsample: u64,
    byte_len: u64,
    dwt_timestamp: i64,
    current_mode: Option<(u64, String)>,
    file_lookup: HashMap<u64, String>,
    func_lookup: HashMap<u64, String>,
}

impl Itm {
    fn new() -> Self {
        Itm {
            buf: Vec::new(),
            syncbuf: Vec::new(),
            swpackets: Vec::new(),
            prevsample: 0,
            startsample: 0,
            byte_len: 0,
            dwt_timestamp: 0,
            current_mode: None,
            file_lookup: HashMap::new(),
            func_lookup: HashMap::new(),
        }
    }

    fn sw_get_mut(&mut self, pid: i64) -> Option<&mut Option<SwPacket>> {
        self.swpackets.iter_mut().find(|(p, _)| *p == pid).map(|(_, v)| v)
    }

    fn sw_set(&mut self, pid: i64, value: Option<SwPacket>) {
        if let Some(slot) = self.sw_get_mut(pid) {
            *slot = value;
        } else {
            self.swpackets.push((pid, value));
        }
    }

    fn add_delayed_sw(&mut self, pid: i64, c: char) {
        let prevsample = self.prevsample;
        let startsample = self.startsample;
        if let Some(slot) = self.sw_get_mut(pid) {
            if let Some(packet) = slot {
                packet.prevtime = prevsample;
                packet.text.push(c);
                return;
            }
        }
        self.sw_set(
            pid,
            Some(SwPacket { ss: startsample, prevtime: prevsample, text: c.to_string() }),
        );
    }

    fn push_delayed_sw(&mut self, ctx: &mut dyn Context) {
        let prevsample = self.prevsample;
        let byte_len = self.byte_len;
        for (pid, slot) in &mut self.swpackets {
            let Some(packet) = slot else { continue };
            // Text has ended if at least 16 byte durations after the
            // previous received byte.
            let gap = i128::from(prevsample) - i128::from(packet.prevtime);
            if gap <= 0 || gap <= 16 * i128::from(byte_len) {
                continue;
            }
            ctx.put_ann(
                packet.ss,
                packet.prevtime,
                ANN_SOFTWARE,
                vec![format!("{}: \"{}\"", pid, packet.text)],
            );
            *slot = None;
        }
    }

    /// `mode_change`: reports the mode that ends at the current packet.
    fn mode_change(&mut self, ctx: &mut dyn Context, new_mode: Option<String>) {
        if let Some((start, mode)) = &self.current_mode {
            let ann_idx = if mode.starts_with("Thread") {
                ANN_MODE_THREAD
            } else if mode.starts_with("IRQ") {
                ANN_MODE_IRQ
            } else {
                ANN_MODE_EXC
            };
            ctx.put_ann(*start, self.startsample, ann_idx, vec![mode.clone()]);
        }

        self.current_mode = new_mode.map(|m| (self.startsample, m));
    }

    fn location_change(&self, ctx: &mut dyn Context, pc: u64) {
        let ss = self.startsample;
        let es = self.prevsample;

        if let Some(loc) = self.file_lookup.get(&pc) {
            ctx.put_ann(ss, es, ANN_LOCATION, vec![loc.clone()]);
        }

        if let Some(func) = self.func_lookup.get(&pc) {
            ctx.put_ann(ss, es, ANN_FUNCTION, vec![func.clone()]);
        }
    }

    fn fallback(&self) -> Outcome {
        let ptype = packet_type(self.buf[0]);
        let bytes: Vec<String> = self.buf.iter().map(|b| format!("{b:02x}")).collect();
        Outcome::Emit((ANN_TRACE, vec![format!("Unhandled {ptype}: {}", bytes.join(" "))]))
    }

    fn handle_overflow(&self) -> Outcome {
        Outcome::Emit((ANN_TRACE, vec!["Overflow".into()]))
    }

    /// `handle_hardware`: packets from the DWT block.
    fn handle_hardware(&mut self, ctx: &mut dyn Context) -> Result<Outcome> {
        let plen = [0usize, 1, 2, 4][usize::from(self.buf[0] & 0x03)];
        let pid = self.buf[0] >> 3;
        if self.buf.len() != plen + 1 {
            return Ok(Outcome::Wait); // Not complete yet.
        }

        if pid == 0 {
            let b = byte(&self.buf, 1)?;
            let mut text = String::from("DWT events:");
            if b & 0x20 != 0 {
                text.push_str(" Cyc");
            }
            if b & 0x10 != 0 {
                text.push_str(" Fold");
            }
            if b & 0x08 != 0 {
                text.push_str(" LSU");
            }
            if b & 0x04 != 0 {
                text.push_str(" Sleep");
            }
            if b & 0x02 != 0 {
                text.push_str(" Exc");
            }
            if b & 0x01 != 0 {
                text.push_str(" CPI");
            }
            return Ok(Outcome::Emit((ANN_DWT_EVENT, vec![text])));
        } else if pid == 1 {
            let b1 = byte(&self.buf, 1)?;
            let b2 = byte(&self.buf, 2)?;
            let excnum = (u32::from(b2 & 1) << 8) | u32::from(b1);
            let event = b2 >> 4;
            let excstr = arm_exception(excnum)
                .map_or_else(|| format!("IRQ {}", i64::from(excnum) - 16), str::to_owned);
            if event == 1 {
                self.mode_change(ctx, Some(excstr.clone()));
                return Ok(Outcome::Emit((
                    ANN_DWT_EXC,
                    vec![format!("Enter: {excstr}"), format!("E {excstr}")],
                )));
            } else if event == 2 {
                self.mode_change(ctx, None);
                return Ok(Outcome::Emit((
                    ANN_DWT_EXC,
                    vec![format!("Exit: {excstr}"), format!("X {excstr}")],
                )));
            } else if event == 3 {
                self.mode_change(ctx, Some(excstr.clone()));
                return Ok(Outcome::Emit((
                    ANN_DWT_EXC,
                    vec![format!("Resume: {excstr}"), format!("R {excstr}")],
                )));
            }
        } else if pid == 2 {
            let pc = u64::from(byte(&self.buf, 1)?)
                | (u64::from(byte(&self.buf, 2)?) << 8)
                | (u64::from(byte(&self.buf, 3)?) << 16)
                | (u64::from(byte(&self.buf, 4)?) << 24);
            self.location_change(ctx, pc);
            return Ok(Outcome::Emit((ANN_DWT_PC, vec![format!("PC: 0x{pc:08x}")])));
        } else if (self.buf[0] & 0xC4) == 0x84 {
            let comp = (self.buf[0] & 0x30) >> 4;
            let what = if (self.buf[0] & 0x08) == 0 { "Read" } else { "Write" };
            let data = if plen == 1 {
                format!("0x{:02x}", byte(&self.buf, 1)?)
            } else if plen == 2 {
                format!(
                    "0x{:04x}",
                    u32::from(byte(&self.buf, 1)?) | (u32::from(byte(&self.buf, 2)?) << 8)
                )
            } else {
                format!(
                    "0x{:08x}",
                    u32::from(byte(&self.buf, 1)?)
                        | (u32::from(byte(&self.buf, 2)?) << 8)
                        | (u32::from(byte(&self.buf, 3)?) << 16)
                        | (u32::from(byte(&self.buf, 4)?) << 24)
                )
            };
            let initial = what.chars().next().unwrap_or('?');
            return Ok(Outcome::Emit((
                ANN_DWT_WATCHPOINT,
                vec![
                    format!("Watchpoint {comp}: {what} data {data}"),
                    format!("WP{comp}: {initial} {data}"),
                ],
            )));
        } else if (self.buf[0] & 0xCF) == 0x47 {
            let comp = (self.buf[0] & 0x30) >> 4;
            let addr = u64::from(byte(&self.buf, 1)?)
                | (u64::from(byte(&self.buf, 2)?) << 8)
                | (u64::from(byte(&self.buf, 3)?) << 16)
                | (u64::from(byte(&self.buf, 4)?) << 24);
            self.location_change(ctx, addr);
            return Ok(Outcome::Emit((
                ANN_DWT_WATCHPOINT,
                vec![
                    format!("Watchpoint {comp}: PC 0x{addr:08x}"),
                    format!("WP{comp}: PC 0x{addr:08x}"),
                ],
            )));
        } else if (self.buf[0] & 0xCF) == 0x4E {
            let comp = (self.buf[0] & 0x30) >> 4;
            let offset = u32::from(byte(&self.buf, 1)?) | (u32::from(byte(&self.buf, 2)?) << 8);
            return Ok(Outcome::Emit((
                ANN_DWT_WATCHPOINT,
                vec![
                    format!("Watchpoint {comp}: address 0x????{offset:04x}"),
                    format!("WP{comp}: A 0x{offset:04x}"),
                ],
            )));
        }

        Ok(self.fallback())
    }

    /// `handle_software`: packets generated by software on the CPU.
    fn handle_software(&mut self, ctx: &mut dyn Context) -> Result<Outcome> {
        let plen = [0usize, 1, 2, 4][usize::from(self.buf[0] & 0x03)];
        let pid = i64::from(self.buf[0] >> 3);
        if self.buf.len() != plen + 1 {
            return Ok(Outcome::Wait); // Not complete yet.
        }

        if plen == 1 && is_printable(byte(&self.buf, 1)?) {
            self.add_delayed_sw(pid, char::from(byte(&self.buf, 1)?));
            return Ok(Outcome::Silence); // Handled but no data to output.
        }

        self.push_delayed_sw(ctx);

        if plen == 1 {
            return Ok(Outcome::Emit((
                ANN_SOFTWARE,
                vec![format!("{pid}: 0x{:02x}", byte(&self.buf, 1)?)],
            )));
        } else if plen == 2 {
            return Ok(Outcome::Emit((
                ANN_SOFTWARE,
                vec![format!("{pid}: 0x{:02x}{:02x}", byte(&self.buf, 2)?, byte(&self.buf, 1)?)],
            )));
        } else if plen == 4 {
            return Ok(Outcome::Emit((
                ANN_SOFTWARE,
                vec![format!(
                    "{pid}: 0x{:02x}{:02x}{:02x}{:02x}",
                    byte(&self.buf, 4)?,
                    byte(&self.buf, 3)?,
                    byte(&self.buf, 2)?,
                    byte(&self.buf, 1)?
                )],
            )));
        }
        Ok(Outcome::Wait)
    }

    /// `handle_timestamp`: the time of the preceding DWT event packet.
    fn handle_timestamp(&mut self) -> Result<Outcome> {
        let last = *self.buf.last().unwrap_or(&0);
        if last & 0x80 != 0 {
            return Ok(Outcome::Wait); // Not complete yet.
        }

        let (tc, ts);
        if self.buf[0] & 0x80 == 0 {
            tc = 0;
            ts = i64::from(self.buf[0] >> 4);
        } else {
            tc = (self.buf[0] & 0x30) >> 4;
            let mut t = i64::from(byte(&self.buf, 1)? & 0x7F);
            if self.buf.len() > 2 {
                t |= i64::from(byte(&self.buf, 2)? & 0x7F) << 7;
            }
            if self.buf.len() > 3 {
                t |= i64::from(byte(&self.buf, 3)? & 0x7F) << 14;
            }
            if self.buf.len() > 4 {
                t |= i64::from(byte(&self.buf, 4)? & 0x7F) << 21;
            }
            ts = t;
        }

        self.dwt_timestamp += ts;

        Ok(Outcome::Emit((
            ANN_TIMESTAMP,
            vec![format!(
                "Timestamp: {} {}",
                self.dwt_timestamp,
                match tc {
                    0 => "(exact)",
                    1 => "(timestamp delayed)",
                    2 => "(event delayed)",
                    _ => "(event and timestamp delayed)",
                }
            )],
        )))
    }

    /// `load_objdump`: file and function lookups from the disassembly.
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
        let mut prev_file = String::new();
        let mut prev_func = String::new();
        for line in disasm.split('\n') {
            if let Some((addr, _, _)) = parse_instruction(line) {
                self.file_lookup.insert(addr, prev_file.clone());
                self.func_lookup.insert(addr, prev_func.clone());
            } else if let Some(func) = parse_function(line) {
                prev_func = func;
            } else if let Some(file) = parse_file(line) {
                prev_file = file;
            }
        }
    }
}

/// `get_packet_type`: the ITM/DWT packet a first byte starts.
fn packet_type(byte: u8) -> &'static str {
    if byte & 0x7F == 0 {
        "sync"
    } else if byte == 0x70 {
        "overflow"
    } else if byte & 0x0F == 0 && byte & 0xF0 != 0 {
        "timestamp"
    } else if byte & 0x0F == 0x08 {
        "sw_extension"
    } else if byte & 0x0F == 0x0C {
        "hw_extension"
    } else if byte & 0x0F == 0x04 {
        "reserved"
    } else if byte & 0x04 == 0x00 {
        "software"
    } else {
        "hardware"
    }
}

/// `chr(byte) in string.printable`.
fn is_printable(byte: u8) -> bool {
    matches!(byte, 0x20..=0x7e | 0x09..=0x0d)
}

fn byte(buf: &[u8], index: usize) -> Result<u8> {
    buf.get(index).copied().ok_or_else(|| Error::arg("list index out of range"))
}

impl Decoder for Itm {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Itm::new();
        self.load_objdump(config);
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("arm_itm is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, pdata) = unpack_uart(item)?;

        // For now, ignore all UART packets except the actual data packets.
        if ptype != "DATA" {
            return Ok(());
        }
        let value = data_value(pdata)?;

        self.byte_len = es.saturating_sub(ss);

        // Reset packet if there is a long pause between bytes. TPIU
        // framing can introduce small pauses, but more than 1 frame
        // should reset packet.
        if ss > self.prevsample && ss - self.prevsample > 16 * self.byte_len {
            self.push_delayed_sw(ctx);
            self.buf.clear();
        }
        self.prevsample = es;

        // Build up the current packet byte by byte.
        self.buf.push(value);

        // Store the start time of the packet.
        if self.buf.len() == 1 {
            self.startsample = ss;
        }

        // Keep a separate buffer for detection of sync packets; they
        // override everything else, so we can regain sync.
        self.syncbuf = self.syncbuf.iter().rev().take(5).rev().copied().collect();
        self.syncbuf.push(value);
        if self.syncbuf == [0, 0, 0, 0, 0, 0x80] {
            self.buf = self.syncbuf.clone();
        }

        // See if it is ready to be decoded.
        let ptype = packet_type(self.buf[0]);
        let outcome = match ptype {
            "overflow" => self.handle_overflow(),
            "hardware" => self.handle_hardware(ctx)?,
            "software" => self.handle_software(ctx)?,
            "timestamp" => self.handle_timestamp()?,
            _ => self.fallback(),
        };

        match outcome {
            Outcome::Wait => {}
            Outcome::Silence => self.buf.clear(),
            Outcome::Emit((class, texts)) => {
                ctx.put_ann(self.startsample, es, class, texts);
                self.buf.clear();
            }
        }
        Ok(())
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
