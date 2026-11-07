// Counterpart in sigrok:
//   libsigrokdecode/decoders/ieee488/pd.py,
//   Copyright (C) 2016 Rudolf Reuter <reuterru@arcor.de>,
//   Copyright (C) 2017 Marcus Comstedt <marcus@mc.pp.se>,
//   Copyright (C) 2019 Gerhard Sittig <gerhard.sittig@gmx.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the IEEE-488 bus protocol and the decoder's
//   documented classes, texts and packet format.
//
// Subtle points of the Python original that this port keeps:
//   - `wait()` starts with `{DAV: 'l'}`, `{ATN: 'l'}` and, when assigned,
//     `{EOI: 'l'}` and `{IFC: 'l'}` so a capture starting with low levels
//     is inspected; the conditions then become `'e'` for every later wait.
//     All lines including the data bits are low active and are inverted
//     once per wait (`invert_pins`), but optional unassigned channels stay
//     at the host's `0xff` marker.
//   - At one sample several edges may coincide. The Python loop processes
//     asserted flags first (IFC, EOI, ATN), then DAV, then deasserted flags
//     (ATN, EOI, IFC), and this order is preserved.
//   - `has_data_n` in the Python source is `[bool(generator)]`, i.e.
//     always `[True]`, so `False in has_data_n` never fires and the eight
//     DIO lines are not really validated. Only DAV and ATN are checked.
//   - A DAV active phase latches `curr_atn`/`curr_eoi`; the byte is
//     processed when DAV goes inactive. Serial IEC input injects such a
//     DAV phase around each accumulated byte.
//   - Without ATN, `check_extra_flush()` runs before the current byte is
//     reported: when the previously accumulated byte was CR or LF and the
//     current one is not, the pending `TALKER_BYTES`/`TALKER_TEXT` run is
//     flushed first (unless `delim=none`).
//   - EOI deassertion emits the optional `EOI` annotation only when
//     `ss_eoi` is truthy (sample 0 does not count) and `latch_eoi` is set,
//     then flushes the pending text run.
//   - `bytearray` payloads are handed to stacked decoders for
//     `TALKER_BYTES`; texts are accumulated per byte, and the text run and
//     the bytes run share `ss_text`/`es_text`.

//! IEEE-488: parallel GPIB/HPIB and serial IEC bus.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue, Pins,
};

// Channel indices, in declaration order.
const PIN_DIO1: usize = 0;
const PIN_DIO8: usize = 7;
const PIN_EOI: usize = 8;
const PIN_DAV: usize = 9;
const PIN_IFC: usize = 12;
const PIN_SRQ: usize = 13;
const PIN_ATN: usize = 14;
const PIN_CLK: usize = 16;
const PIN_DATA: usize = PIN_DIO1;

// Annotation classes, as the libsigrokdecode decoder numbers them.
const ANN_RAW_BIT: usize = 0;
const ANN_RAW_BYTE: usize = 1;
const ANN_CMD: usize = 2;
const ANN_LADDR: usize = 3;
const ANN_TADDR: usize = 4;
const ANN_SADDR: usize = 5;
const ANN_DATA: usize = 6;
const ANN_EOI: usize = 7;
const ANN_TEXT: usize = 8;
const ANN_IEC_PERIPH: usize = 9;
const ANN_WARN: usize = 10;

// Binary classes.
const BIN_RAW: usize = 0;
const BIN_DATA: usize = 1;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ieee488",
    name: "IEEE-488",
    longname: "IEEE-488 GPIB/HPIB/IEC",
    desc: "IEEE-488 General Purpose Interface Bus (GPIB/HPIB or IEC).",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["ieee488"],
    tags: &["PC", "Retro computing"],
    channels: &[ChannelDesc {
        id: "dio1",
        name: "DIO1/DATA",
        desc: "Data I/O bit 1, or serial data",
    }],
    optional_channels: &[
        ChannelDesc { id: "dio2", name: "DIO2", desc: "Data I/O bit 2" },
        ChannelDesc { id: "dio3", name: "DIO3", desc: "Data I/O bit 3" },
        ChannelDesc { id: "dio4", name: "DIO4", desc: "Data I/O bit 4" },
        ChannelDesc { id: "dio5", name: "DIO5", desc: "Data I/O bit 5" },
        ChannelDesc { id: "dio6", name: "DIO6", desc: "Data I/O bit 6" },
        ChannelDesc { id: "dio7", name: "DIO7", desc: "Data I/O bit 7" },
        ChannelDesc { id: "dio8", name: "DIO8", desc: "Data I/O bit 8" },
        ChannelDesc { id: "eoi", name: "EOI", desc: "End or identify" },
        ChannelDesc { id: "dav", name: "DAV", desc: "Data valid" },
        ChannelDesc { id: "nrfd", name: "NRFD", desc: "Not ready for data" },
        ChannelDesc { id: "ndac", name: "NDAC", desc: "Not data accepted" },
        ChannelDesc { id: "ifc", name: "IFC", desc: "Interface clear" },
        ChannelDesc { id: "srq", name: "SRQ", desc: "Service request" },
        ChannelDesc { id: "atn", name: "ATN", desc: "Attention" },
        ChannelDesc { id: "ren", name: "REN", desc: "Remote enable" },
        ChannelDesc { id: "clk", name: "CLK", desc: "Serial clock" },
    ],
    options: &[
        OptionDesc {
            id: "iec_periph",
            desc: "Decode Commodore IEC bus peripherals details",
            default: OptionValue::s("no"),
            values: &[OptionValue::s("no"), OptionValue::s("yes")],
        },
        OptionDesc {
            id: "delim",
            desc: "Payload data delimiter",
            default: OptionValue::s("eol"),
            values: &[OptionValue::s("none"), OptionValue::s("eol")],
        },
    ],
    annotations: &[
        AnnClass { id: "bit", desc: "IEC bit" },
        AnnClass { id: "raw", desc: "Raw byte" },
        AnnClass { id: "cmd", desc: "Command" },
        AnnClass { id: "laddr", desc: "Listener address" },
        AnnClass { id: "taddr", desc: "Talker address" },
        AnnClass { id: "saddr", desc: "Secondary address" },
        AnnClass { id: "data", desc: "Data byte" },
        AnnClass { id: "eoi", desc: "EOI" },
        AnnClass { id: "text", desc: "Talker text" },
        AnnClass { id: "periph", desc: "IEC bus peripherals" },
        AnnClass { id: "warn", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "IEC bits", classes: &[ANN_RAW_BIT] },
        AnnRow { id: "raws", desc: "Raw bytes", classes: &[ANN_RAW_BYTE] },
        AnnRow {
            id: "gpib",
            desc: "Commands/data",
            classes: &[ANN_CMD, ANN_LADDR, ANN_TADDR, ANN_SADDR, ANN_DATA],
        },
        AnnRow { id: "eois", desc: "EOI", classes: &[ANN_EOI] },
        AnnRow { id: "texts", desc: "Talker texts", classes: &[ANN_TEXT] },
        AnnRow { id: "periphs", desc: "IEC peripherals", classes: &[ANN_IEC_PERIPH] },
        AnnRow { id: "warns", desc: "Warnings", classes: &[ANN_WARN] },
    ],
    binary: &[
        BinClass { id: "raw", desc: "Raw bytes" },
        BinClass { id: "data", desc: "Talker bytes" },
    ],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ieee488::default())
}

fn strs(t: &[&str]) -> Vec<String> {
    t.iter().map(|s| (*s).to_owned()).collect()
}

/// `_is_command()`: a command, un-listen or un-talk byte.
fn is_command(b: u32) -> (bool, Option<bool>, Option<bool>) {
    if (0x00..0x20).contains(&b) {
        return (true, None, None);
    }
    if (0x20..0x40).contains(&b) && (b & 0x1f) == 31 {
        return (true, Some(true), Some(false));
    }
    if (0x40..0x60).contains(&b) && (b & 0x1f) == 31 {
        return (true, Some(false), Some(true));
    }
    (false, None, None)
}

fn is_listen_addr(b: u32) -> Option<u32> {
    (0x20..0x40).contains(&b).then_some(b & 0x1f)
}

fn is_talk_addr(b: u32) -> Option<u32> {
    (0x40..0x60).contains(&b).then_some(b & 0x1f)
}

fn is_secondary_addr(b: u32) -> Option<u32> {
    (0x60..0x80).contains(&b).then_some(b & 0x1f)
}

fn is_msb_set(b: u32) -> Option<u32> {
    (b & 0x80 != 0).then_some(b)
}

/// `_get_raw_byte()`: decorate command bytes for stacked decoders.
fn get_raw_byte(b: u32, atn: bool) -> i64 {
    i64::from(if atn { b | 0x100 } else { b })
}

/// `_get_raw_text()`: `'/xx'` with ATN asserted, `'xx'` without.
fn get_raw_text(b: u32, atn: bool) -> Vec<String> {
    vec![format!("{}{b:02x}", if atn { "/" } else { "" })]
}

/// `_get_command_texts()`: known flag and annotation texts.
fn get_command_texts(b: u32) -> (bool, Vec<String>) {
    let known: Option<&[&str]> = match b {
        0x01 => Some(&["Go To Local", "GTL"]),
        0x04 => Some(&["Selected Device Clear", "SDC"]),
        0x05 => Some(&["Parallel Poll Configure", "PPC"]),
        0x08 => Some(&["Global Execute Trigger", "GET"]),
        0x09 => Some(&["Take Control", "TCT"]),
        0x11 => Some(&["Local Lock Out", "LLO"]),
        0x14 => Some(&["Device Clear", "DCL"]),
        0x15 => Some(&["Parallel Poll Unconfigure", "PPU"]),
        0x18 => Some(&["Serial Poll Enable", "SPE"]),
        0x19 => Some(&["Serial Poll Disable", "SPD"]),
        0x3f => Some(&["Unlisten", "UNL"]),
        0x5f => Some(&["Untalk", "UNT"]),
        _ => None,
    };
    if let Some(fmts) = known {
        return (true, strs(fmts));
    }
    let cmd_ord = char::from_u32(0x30 + b).unwrap_or('?');
    (
        false,
        vec![
            format!("Unknown command 0x{b:02x}"),
            format!("command 0x{b:02x}"),
            format!("cmd {b:02x}"),
            format!("C{cmd_ord}"),
        ],
    )
}

/// `_get_address_texts()`: listener, talker, secondary or MSB address.
fn get_address_texts(b: u32) -> Option<Vec<String>> {
    // With no address the Python function's format table is empty and it
    // returns None before `addr` would be referenced.
    let found = if let Some(a) = is_listen_addr(b) {
        Some(("L", a))
    } else if let Some(a) = is_talk_addr(b) {
        Some(("T", a))
    } else if let Some(a) = is_secondary_addr(b) {
        Some(("S", a))
    } else {
        is_msb_set(b).map(|a| ("S", a))
    };
    let (kind, addr) = found?;
    let ord = char::from_u32(0x30 + addr).unwrap_or('?');
    let word = match kind {
        "L" => "Listen",
        "T" => "Talk",
        _ => "Secondary",
    };
    Some(vec![format!("{word} {addr}"), format!("{kind} {addr}"), format!("{kind}{ord}")])
}

/// `_get_data_text()`: printable byte, control code name or `[xx]`.
fn get_data_text(b: u32) -> String {
    // The Python code's `b not in ('[', ']')` compares an int with
    // strings and is therefore always true; '[' and ']' stay literal.
    if (0x20..0x7f).contains(&b) {
        return char::from_u32(b).unwrap_or('?').to_string();
    }
    let control = match b {
        0x00 => Some("NUL"),
        0x01 => Some("SOH"),
        0x02 => Some("STX"),
        0x03 => Some("ETX"),
        0x04 => Some("EOT"),
        0x05 => Some("ENQ"),
        0x06 => Some("ACK"),
        0x07 => Some("BEL"),
        0x08 => Some("BS"),
        0x09 => Some("TAB"),
        0x0a => Some("LF"),
        0x0b => Some("VT"),
        0x0c => Some("FF"),
        0x0d => Some("CR"),
        0x0e => Some("SO"),
        0x0f => Some("SI"),
        0x10 => Some("DLE"),
        0x11 => Some("DC1"),
        0x12 => Some("DC2"),
        0x13 => Some("DC3"),
        0x14 => Some("DC4"),
        0x15 => Some("NAK"),
        0x16 => Some("SYN"),
        0x17 => Some("ETB"),
        0x18 => Some("CAN"),
        0x19 => Some("EM"),
        0x1a => Some("SUB"),
        0x1b => Some("ESC"),
        0x1c => Some("FS"),
        0x1d => Some("GS"),
        0x1e => Some("RS"),
        0x1f => Some("US"),
        _ => None,
    };
    match control {
        Some(name) => format!("[{name}]"),
        None => format!("[{b:02x}]"),
    }
}

/// `bitpack()` from `common.srdhelper`: LSB first.
fn bitpack(bits: &[u8]) -> u32 {
    bits.iter().enumerate().fold(0u32, |v, (i, &b)| v | (u32::from(b) << i))
}

/// `invert_pins()`: flip 0/1, leave an unassigned channel's marker alone.
fn invert_pins(pins: &Pins) -> Vec<u8> {
    pins.0.iter().map(|&p| if p <= 1 { 1 - p } else { p }).collect()
}

fn opt_int(v: Option<i64>) -> Item {
    v.map_or(Item::None, Item::Int)
}

fn int_list(values: &[i64]) -> Item {
    Item::List(values.iter().map(|&v| Item::Int(v)).collect())
}

/// The IEEE-488 decoder.
pub struct Ieee488 {
    iec_periph: bool,
    delim_eol: bool,
    curr_raw: Option<u32>,
    curr_atn: Option<bool>,
    curr_eoi: Option<bool>,
    latch_atn: Option<bool>,
    latch_eoi: Option<bool>,
    accu_bytes: Vec<u32>,
    accu_text: Vec<String>,
    ss_raw: Option<u64>,
    es_raw: Option<u64>,
    ss_eoi: Option<u64>,
    es_eoi: Option<u64>,
    ss_text: Option<u64>,
    es_text: Option<u64>,
    last_talker: Option<i64>,
    last_listener: Vec<i64>,
    last_iec_addr: Option<u32>,
    last_iec_sec: Option<u32>,
}

impl Default for Ieee488 {
    fn default() -> Self {
        Ieee488 {
            iec_periph: false,
            delim_eol: true,
            curr_raw: None,
            curr_atn: None,
            curr_eoi: None,
            latch_atn: None,
            latch_eoi: None,
            accu_bytes: Vec::new(),
            accu_text: Vec::new(),
            ss_raw: None,
            es_raw: None,
            ss_eoi: None,
            es_eoi: None,
            ss_text: None,
            es_text: None,
            last_talker: None,
            last_listener: Vec::new(),
            last_iec_addr: None,
            last_iec_sec: None,
        }
    }
}

impl Ieee488 {
    /// Flush accumulated payload bytes and text.
    fn flush_bytes_text_accu(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if !self.accu_bytes.is_empty()
            && let (Some(ss), Some(es)) = (self.ss_text, self.es_text)
        {
            let bytes: Vec<u8> = self
                .accu_bytes
                .iter()
                .map(|&b| u8::try_from(b).map_err(|_| Error::arg("byte out of range")))
                .collect::<Result<_>>()?;
            ctx.put_binary(ss, es, BIN_DATA, bytes.clone());
            ctx.put_python(
                ss,
                es,
                Item::List(vec![
                    Item::Str("TALKER_BYTES".into()),
                    opt_int(self.last_talker),
                    Item::Bytes(bytes),
                ]),
            );
            self.accu_bytes.clear();
        }
        if !self.accu_text.is_empty()
            && let (Some(ss), Some(es)) = (self.ss_text, self.es_text)
        {
            let text = self.accu_text.concat();
            ctx.put_ann(ss, es, ANN_TEXT, vec![text.clone()]);
            ctx.put_python(
                ss,
                es,
                Item::List(vec![
                    Item::Str("TALKER_TEXT".into()),
                    opt_int(self.last_talker),
                    Item::Str(text),
                ]),
            );
            self.accu_text.clear();
        }
        self.ss_text = None;
        self.es_text = None;
        Ok(())
    }

    /// Optionally flush a completed text line before the next data byte.
    fn check_extra_flush(&mut self, ctx: &mut dyn Context, b: u32) -> Result<()> {
        if !self.delim_eol {
            return Ok(());
        }
        if self.accu_bytes.is_empty() {
            return Ok(());
        }
        let term_chars = [10, 13];
        let is_eol = term_chars.contains(&b);
        let had_eol = self.accu_bytes.last().is_some_and(|last| term_chars.contains(last));
        if had_eol && !is_eol {
            self.flush_bytes_text_accu(ctx)?;
        }
        Ok(())
    }

    /// `handle_ifc_change()`: IFC de-selects every talker and listener.
    fn handle_ifc_change(&mut self, ctx: &mut dyn Context, ifc: bool) -> Result<()> {
        if ifc {
            self.last_talker = None;
            self.last_listener.clear();
            self.flush_bytes_text_accu(ctx)?;
        }
        Ok(())
    }

    /// `handle_eoi_change()`: track EOI for parallel and serial input.
    fn handle_eoi_change(&mut self, ctx: &mut dyn Context, eoi: bool) -> Result<()> {
        if eoi {
            self.ss_eoi = Some(ctx.samplenum());
            self.curr_eoi = Some(true);
        } else {
            let es_eoi = ctx.samplenum();
            self.es_eoi = Some(es_eoi);
            if self.ss_eoi.is_some_and(|s| s != 0) && self.latch_eoi.unwrap_or(false) {
                ctx.put_ann(self.ss_eoi.unwrap_or(es_eoi), es_eoi, ANN_EOI, vec!["EOI".into()]);
            }
            self.es_text = self.es_eoi;
            self.flush_bytes_text_accu(ctx)?;
            self.ss_eoi = None;
            self.es_eoi = None;
            self.curr_eoi = None;
        }
        Ok(())
    }

    /// `handle_atn_change()`: track ATN and flush a pending payload run.
    fn handle_atn_change(&mut self, ctx: &mut dyn Context, atn: bool) -> Result<()> {
        self.curr_atn = Some(atn);
        if atn {
            self.flush_bytes_text_accu(ctx)?;
        }
        Ok(())
    }

    /// `handle_iec_periph()`: Commodore IEC peripheral annotations.
    fn handle_iec_periph(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        addr: Option<u32>,
        sec: Option<u32>,
        data: Option<u32>,
    ) {
        if !self.iec_periph {
            return;
        }
        // Void internal state.
        if addr.is_none() && sec.is_none() && data.is_none() {
            self.last_iec_addr = None;
            self.last_iec_sec = None;
            return;
        }
        let iec_disk_range = 8..16;
        if let Some(a) = addr {
            self.last_iec_addr = Some(a);
            let name = match a {
                8 => Some("Disk 0"),
                9 => Some("Disk 1"),
                _ => None,
            };
            if let Some(name) = name {
                ctx.put_ann(ss, es, ANN_IEC_PERIPH, vec![name.to_owned()]);
            }
        }
        let addr = self.last_iec_addr;
        if let Some(sec_byte) = sec {
            // BEWARE! The secondary address is a full byte and includes
            // the 0x60 offset, to also work when the MSB was set.
            self.last_iec_sec = Some(sec_byte);
            let subcmd = sec_byte & 0xf0;
            let channel = sec_byte & 0x0f;
            let channel_ord = char::from_u32(0x30 + channel).unwrap_or('?');
            if addr.is_some_and(|a| iec_disk_range.contains(&a)) {
                let texts = match subcmd {
                    0x60 => Some(vec![
                        format!("Reopen {channel}"),
                        format!("Re {channel}"),
                        format!("R{channel_ord}"),
                    ]),
                    0xe0 => Some(vec![
                        format!("Close {channel}"),
                        format!("Cl {channel}"),
                        format!("C{channel_ord}"),
                    ]),
                    0xf0 => Some(vec![
                        format!("Open {channel}"),
                        format!("Op {channel}"),
                        format!("O{channel_ord}"),
                    ]),
                    _ => None,
                };
                if let Some(texts) = texts {
                    ctx.put_ann(ss, es, ANN_IEC_PERIPH, texts);
                }
            }
        }
        // TODO Process data depending on peripheral type and channel?
    }

    /// `handle_data_byte()`: emit one byte captured between DAV edges.
    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn handle_data_byte(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let b = self.curr_raw.unwrap_or(0);
        let atn = self.curr_atn.unwrap_or(false);
        if !atn {
            self.check_extra_flush(ctx, b)?;
        }
        let (Some(ss), Some(es)) = (self.ss_raw, self.es_raw) else {
            return Err(Error::arg("byte without a DAV phase"));
        };

        let raw_texts = get_raw_text(b, atn);
        ctx.put_ann(ss, es, ANN_RAW_BYTE, raw_texts);
        let raw_byte = u8::try_from(b).map_err(|_| Error::arg("byte out of range"))?;
        ctx.put_binary(ss, es, BIN_RAW, vec![raw_byte]);
        ctx.put_python(
            ss,
            es,
            Item::List(vec![
                Item::Str("GPIB_RAW".into()),
                Item::None,
                Item::Int(get_raw_byte(b, atn)),
            ]),
        );

        if atn {
            let mut ann_cls: Option<usize> = None;
            let mut texts: Option<Vec<String>> = None;
            let mut upd_iec = (false, None, None, None);
            let mut py_type: Option<&'static str> = None;
            let mut py_addr: Option<i64> = None;
            let mut py_peers = false;
            let (is_cmd, is_unl, is_unt) = is_command(b);
            let laddr = is_listen_addr(b);
            let taddr = is_talk_addr(b);
            let saddr = is_secondary_addr(b);
            let msb = is_msb_set(b);
            if is_cmd {
                let (known, command_texts) = get_command_texts(b);
                texts = Some(command_texts);
                if !known {
                    ctx.put_ann(
                        ss,
                        es,
                        ANN_WARN,
                        strs(&["Unknown GPIB command", "unknown", "UNK"]),
                    );
                }
                ann_cls = Some(ANN_CMD);
                py_type = Some("COMMAND");
                if is_unl == Some(true) {
                    self.last_listener.clear();
                    py_peers = true;
                }
                if is_unt == Some(true) {
                    self.last_talker = None;
                    py_peers = true;
                }
                if is_unl == Some(true) || is_unt == Some(true) {
                    upd_iec = (true, None, None, None);
                }
            } else if let Some(addr) = laddr {
                let addr = i64::from(addr);
                texts = get_address_texts(b);
                ann_cls = Some(ANN_LADDR);
                py_type = Some("LISTEN");
                py_addr = Some(addr);
                if self.last_talker == Some(addr) {
                    self.last_talker = None;
                }
                self.last_listener.push(addr);
                upd_iec = (true, Some(b & 0x1f), None, None);
                py_peers = true;
            } else if let Some(addr) = taddr {
                let addr = i64::from(addr);
                texts = get_address_texts(b);
                ann_cls = Some(ANN_TADDR);
                py_type = Some("TALK");
                py_addr = Some(addr);
                if let Some(pos) = self.last_listener.iter().position(|&a| a == addr) {
                    self.last_listener.remove(pos);
                }
                self.last_talker = Some(addr);
                upd_iec = (true, Some(b & 0x1f), None, None);
                py_peers = true;
            } else if let Some(addr) = saddr {
                texts = get_address_texts(b);
                ann_cls = Some(ANN_SADDR);
                upd_iec = (true, None, Some(b), None);
                py_type = Some("SECONDARY");
                py_addr = Some(i64::from(addr));
            } else if let Some(msb) = msb {
                // These are not really "secondary addresses", but they
                // are used by the Commodore IEC bus (floppy channels).
                texts = get_address_texts(b);
                ann_cls = Some(ANN_SADDR);
                upd_iec = (true, None, Some(b), None);
                py_type = Some("MSB_SET");
                py_addr = Some(i64::from(msb));
            }
            if let (Some(cls), Some(texts)) = (ann_cls, texts) {
                ctx.put_ann(ss, es, cls, texts);
            }
            if upd_iec.0 {
                self.handle_iec_periph(ctx, ss, es, upd_iec.1, upd_iec.2, upd_iec.3);
            }
            if let Some(py_type) = py_type {
                ctx.put_python(
                    ss,
                    es,
                    Item::List(vec![
                        Item::Str(py_type.into()),
                        opt_int(py_addr),
                        Item::Int(i64::from(b)),
                    ]),
                );
            }
            if py_peers {
                self.last_listener.sort_unstable();
                ctx.put_python(
                    ss,
                    es,
                    Item::List(vec![
                        Item::Str("TALK_LISTEN".into()),
                        opt_int(self.last_talker),
                        int_list(&self.last_listener),
                    ]),
                );
            }
            return Ok(());
        }

        self.accu_bytes.push(b);
        let text = get_data_text(b);
        if self.accu_text.is_empty() {
            self.ss_text = Some(ss);
        }
        self.accu_text.push(text.clone());
        self.es_text = Some(es);
        ctx.put_ann(ss, es, ANN_DATA, vec![text]);
        self.handle_iec_periph(ctx, ss, es, None, None, Some(b));
        ctx.put_python(
            ss,
            es,
            Item::List(vec![
                Item::Str("DATA_BYTE".into()),
                opt_int(self.last_talker),
                Item::Int(i64::from(b)),
            ]),
        );
        Ok(())
    }

    /// `handle_dav_change()`: latch on active, process on inactive.
    fn handle_dav_change(&mut self, ctx: &mut dyn Context, dav: bool, data: &[u8]) -> Result<()> {
        if dav {
            // Data availability starts when the flag goes active.
            self.ss_raw = Some(ctx.samplenum());
            self.curr_raw = Some(bitpack(data));
            self.latch_atn = self.curr_atn;
            self.latch_eoi = self.curr_eoi;
            return Ok(());
        }
        self.es_raw = Some(ctx.samplenum());
        self.handle_data_byte(ctx)?;
        self.ss_raw = None;
        self.es_raw = None;
        self.curr_raw = None;
        Ok(())
    }

    /// `inject_dav_phase()`: pretend a serial byte was a DAV phase.
    fn inject_dav_phase(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        data: &[u8],
    ) -> Result<()> {
        self.ss_raw = Some(ss);
        self.curr_raw = Some(bitpack(data));
        self.latch_atn = self.curr_atn;
        self.latch_eoi = self.curr_eoi;
        self.es_raw = Some(es);
        self.handle_data_byte(ctx)?;
        self.ss_raw = None;
        self.es_raw = None;
        self.curr_raw = None;
        Ok(())
    }

    /// Serial IEC decoding, a rephrased `decoders/iec/pd.py:decode()`.
    fn decode_serial(&mut self, ctx: &mut dyn Context) -> Result<()> {
        const STEP_WAIT_READY_TO_SEND: usize = 0;
        const STEP_WAIT_READY_FOR_DATA: usize = 1;
        const STEP_PREP_DATA_TEST_EOI: usize = 2;
        const STEP_CLOCK_DATA_BITS: usize = 3;

        let mut step = STEP_WAIT_READY_TO_SEND;
        let mut bits: Vec<u8> = Vec::new();
        let mut ss_byte = 0u64;
        let mut ss_bit = 0u64;

        loop {
            let groups: Vec<Vec<Cond>> = match step {
                STEP_WAIT_READY_TO_SEND => vec![
                    vec![Cond::pin(PIN_ATN, CondKind::Falling)],
                    vec![Cond::pin(PIN_DATA, CondKind::Low), Cond::pin(PIN_CLK, CondKind::High)],
                ],
                STEP_WAIT_READY_FOR_DATA => vec![
                    vec![Cond::pin(PIN_ATN, CondKind::Falling)],
                    vec![Cond::pin(PIN_DATA, CondKind::High), Cond::pin(PIN_CLK, CondKind::High)],
                    vec![Cond::pin(PIN_CLK, CondKind::Low)],
                ],
                STEP_PREP_DATA_TEST_EOI => vec![
                    vec![Cond::pin(PIN_ATN, CondKind::Falling)],
                    vec![Cond::pin(PIN_DATA, CondKind::Falling)],
                    vec![Cond::pin(PIN_CLK, CondKind::Low)],
                ],
                _ => vec![
                    vec![Cond::pin(PIN_ATN, CondKind::Falling)],
                    vec![Cond::pin(PIN_CLK, CondKind::Edge)],
                ],
            };
            let (pins, matched) = {
                let refs: Vec<&[Cond]> = groups.iter().map(Vec::as_slice).collect();
                let pins = ctx.wait(&refs)?;
                (pins, ctx.matched().to_vec())
            };
            let data = pins.0.get(PIN_DATA).copied().unwrap_or(0);
            let clk = pins.0.get(PIN_CLK).copied().unwrap_or(0);
            let raw_atn = pins.0.get(PIN_ATN).copied().unwrap_or(0);
            let atn = if raw_atn <= 1 { 1 - raw_atn } else { raw_atn } != 0;

            if matched.first().copied().unwrap_or(false) {
                // Falling edge on ATN, reset step.
                step = STEP_WAIT_READY_TO_SEND;
            }

            match step {
                STEP_WAIT_READY_TO_SEND => {
                    // Don't use self.matched[1] here since we might come
                    // from a step with different conds due to the code above.
                    if data == 0 && clk == 1 {
                        // Rising edge on CLK while DATA is low: Ready to send.
                        step = STEP_WAIT_READY_FOR_DATA;
                    }
                }
                STEP_WAIT_READY_FOR_DATA => {
                    if data == 1 && clk == 1 {
                        // Rising edge on DATA while CLK is high: Ready for data.
                        ss_byte = ctx.samplenum();
                        self.handle_atn_change(ctx, atn)?;
                        if self.curr_eoi.unwrap_or(false) {
                            self.handle_eoi_change(ctx, false)?;
                        }
                        bits.clear();
                        step = STEP_PREP_DATA_TEST_EOI;
                    } else if clk == 0 {
                        // CLK low again, transfer aborted.
                        step = STEP_WAIT_READY_TO_SEND;
                    }
                }
                STEP_PREP_DATA_TEST_EOI => {
                    if data == 0 && clk == 1 {
                        // DATA goes low while CLK is still high, EOI confirmed.
                        self.handle_eoi_change(ctx, true)?;
                    } else if clk == 0 {
                        step = STEP_CLOCK_DATA_BITS;
                        ss_bit = ctx.samplenum();
                    }
                }
                _ => {
                    if matched.get(1).copied().unwrap_or(false) {
                        if clk == 1 {
                            // Rising edge on CLK; latch DATA.
                            bits.push(data);
                        } else if clk == 0 {
                            // Falling edge on CLK; end of bit.
                            let es_bit = ctx.samplenum();
                            let bit = bits.last().copied().unwrap_or(0);
                            ctx.put_ann(ss_bit, es_bit, ANN_RAW_BIT, vec![bit.to_string()]);
                            ctx.put_python(
                                ss_bit,
                                es_bit,
                                Item::List(vec![
                                    Item::Str("IEC_BIT".into()),
                                    Item::None,
                                    Item::Int(i64::from(bit)),
                                ]),
                            );
                            ss_bit = ctx.samplenum();
                            if bits.len() == 8 {
                                let es_byte = ctx.samplenum();
                                self.inject_dav_phase(ctx, ss_byte, es_byte, &bits)?;
                                if self.curr_eoi.unwrap_or(false) {
                                    self.handle_eoi_change(ctx, false)?;
                                }
                                step = STEP_WAIT_READY_TO_SEND;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Parallel GPIB/HPIB decoding.
    fn decode_parallel(
        &mut self,
        ctx: &mut dyn Context,
        has_eoi: bool,
        has_ifc: bool,
    ) -> Result<()> {
        // NOTE: The Python source builds `has_data_n` as
        // `[bool(generator)]`, i.e. always `[True]`, so its
        // `False in has_data_n` check never rejects missing DIO lines.
        // Only DAV and ATN are validated (in `decode`).
        let mut waitcond: Vec<Vec<Cond>> =
            vec![vec![Cond::pin(PIN_DAV, CondKind::Low)], vec![Cond::pin(PIN_ATN, CondKind::Low)]];
        let idx_dav = 0;
        let idx_atn = 1;
        let idx_eoi = if has_eoi {
            waitcond.push(vec![Cond::pin(PIN_EOI, CondKind::Low)]);
            Some(waitcond.len() - 1)
        } else {
            None
        };
        let idx_ifc = if has_ifc {
            waitcond.push(vec![Cond::pin(PIN_IFC, CondKind::Low)]);
            Some(waitcond.len() - 1)
        } else {
            None
        };

        loop {
            let (pins, matched) = {
                let refs: Vec<&[Cond]> = waitcond.iter().map(Vec::as_slice).collect();
                let pins = ctx.wait(&refs)?;
                (pins, ctx.matched().to_vec())
            };
            let pins = invert_pins(&pins);

            // BEWARE! Order of evaluation does matter. For low samplerate
            // captures, many edges fall onto the same sample number. So
            // active edges of flags are processed early (before the data
            // bits), and inactive edges late (after data got processed).
            if idx_ifc.is_some_and(|i| matched[i]) && pins[PIN_IFC] == 1 {
                self.handle_ifc_change(ctx, true)?;
            }
            if idx_eoi.is_some_and(|i| matched[i]) && pins[PIN_EOI] == 1 {
                self.handle_eoi_change(ctx, true)?;
            }
            if matched[idx_atn] && pins[PIN_ATN] == 1 {
                self.handle_atn_change(ctx, true)?;
            }
            if matched[idx_dav] {
                let data = pins[PIN_DIO1..=PIN_DIO8].to_vec();
                self.handle_dav_change(ctx, pins[PIN_DAV] == 1, &data)?;
            }
            if matched[idx_atn] && pins[PIN_ATN] == 0 {
                self.handle_atn_change(ctx, false)?;
            }
            if idx_eoi.is_some_and(|i| matched[i]) && pins[PIN_EOI] == 0 {
                self.handle_eoi_change(ctx, false)?;
            }
            if idx_ifc.is_some_and(|i| matched[i]) && pins[PIN_IFC] == 0 {
                self.handle_ifc_change(ctx, false)?;
            }

            waitcond[idx_dav][0] = Cond::pin(PIN_DAV, CondKind::Edge);
            waitcond[idx_atn][0] = Cond::pin(PIN_ATN, CondKind::Edge);
            if let Some(i) = idx_eoi {
                waitcond[i][0] = Cond::pin(PIN_EOI, CondKind::Edge);
            }
            if let Some(i) = idx_ifc {
                waitcond[i][0] = Cond::pin(PIN_IFC, CondKind::Edge);
            }
        }
    }
}

impl Decoder for Ieee488 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Ieee488 {
            iec_periph: config.str("iec_periph", "no") == "yes",
            delim_eol: config.str("delim", "eol") != "none",
            ..Ieee488::default()
        };
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // The decoder's boilerplate declares some of the input signals as
        // optional, but only to support both serial and parallel variants.
        // The CLK signal discriminates the two. For either variant some
        // of the "optional" signals are not really optional for proper
        // operation of the decoder. Check these conditions here.
        let has_clk = ctx.has_channel(PIN_CLK);
        let has_data_1 = ctx.has_channel(PIN_DIO1);
        let has_dav = ctx.has_channel(PIN_DAV);
        let has_atn = ctx.has_channel(PIN_ATN);
        let has_eoi = ctx.has_channel(PIN_EOI);
        let _has_srq = ctx.has_channel(PIN_SRQ);
        if has_clk {
            if !has_data_1 || !has_atn {
                return Err(Error::arg("IEC bus needs at least ATN and serial CLK and DATA."));
            }
            self.decode_serial(ctx)
        } else {
            if !has_dav || !has_atn {
                return Err(Error::arg("IEEE-488 needs at least ATN and DAV and eight DIO lines."));
            }
            let has_ifc = ctx.has_channel(PIN_IFC);
            self.decode_parallel(ctx, has_eoi, has_ifc)
        }
    }
}
