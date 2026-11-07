// Counterpart in sigrok:
//   libsigrokdecode/decoders/amulet_ascii/pd.py and lists.py,
//   Copyright (C) 2019 Vesa-Pekka Palmu <vpalmu@depili.fi>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Amulet ASCII protocol and the decoder's
//   documented classes and texts.

//! Amulet ASCII: the LCD controller's command, value and string protocol.

#![allow(dead_code)]

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

// The ordered command table of `lists.py`; the annotation classes and
// their indices follow it.
static CMDS: [(u8, &str, &str); 41] = [
    (0xA0, "PAGE", "Jump to page"),
    (0xD0, "GBV", "Get byte variable"),
    (0xD1, "GWV", "Get word variable"),
    (0xD2, "GSV", "Get string variable"),
    (0xD3, "GLV", "Get label variable"),
    (0xD4, "GRPC", "Get RPC buffer"),
    (0xD5, "SBV", "Set byte variable"),
    (0xD6, "SWV", "Set word variable"),
    (0xD7, "SSV", "Set string variable"),
    (0xD8, "RPC", "Invoke RPC"),
    (0xD9, "LINE", "Draw line"),
    (0xDA, "RECT", "Draw rectangle"),
    (0xDB, "FRECT", "Draw filled rectangle"),
    (0xDC, "PIXEL", "Draw pixel"),
    (0xDD, "GBVA", "Get byte variable array"),
    (0xDE, "GWVA", "Get word variable array"),
    (0xDF, "SBVA", "Set byte variable array"),
    (0xE0, "GBVR", "Get byte variable reply"),
    (0xE1, "GWVR", "Get word variable reply"),
    (0xE2, "GSVR", "Get string variable reply"),
    (0xE3, "GLVR", "Get label variable reply"),
    (0xE4, "GRPCR", "Get RPC buffer reply"),
    (0xE5, "SBVR", "Set byte variable reply"),
    (0xE6, "SWVR", "Set word variable reply"),
    (0xE7, "SSVR", "Set string variable reply"),
    (0xE8, "RPCR", "Invoke RPC reply"),
    (0xE9, "LINER", "Draw line reply"),
    (0xEA, "RECTR", "Draw rectangle"),
    (0xEB, "FRECTR", "Draw filled rectangle reply"),
    (0xEC, "PIXELR", "Draw pixel reply"),
    (0xED, "GBVAR", "Get byte variable array reply"),
    (0xEE, "GWVAR", "Get word variable array reply"),
    (0xEF, "SBVAR", "Set byte variable array reply"),
    (0xF0, "ACK", "Acknowledgment"),
    (0xF1, "NACK", "Negative acknowledgment"),
    (0xF2, "SWVA", "Set word variable array"),
    (0xF3, "SWVAR", "Set word variable array reply"),
    (0xF4, "GCV", "Get color variable"),
    (0xF5, "GCVR", "Get color variable reply"),
    (0xF6, "SCV", "Set color variable"),
    (0xF7, "SCVR", "Set color variable reply"),
];

// Don't forget to keep this in sync with 'cmds' in lists.py.
const ANN_PAGE: usize = 0;
const ANN_GBV: usize = 1;
const ANN_GWV: usize = 2;
const ANN_GSV: usize = 3;
const ANN_GLV: usize = 4;
const ANN_GRPC: usize = 5;
const ANN_SBV: usize = 6;
const ANN_SWV: usize = 7;
const ANN_SSV: usize = 8;
const ANN_RPC: usize = 9;
const ANN_LINE: usize = 10;
const ANN_RECT: usize = 11;
const ANN_FRECT: usize = 12;
const ANN_PIXEL: usize = 13;
const ANN_GBVA: usize = 14;
const ANN_GWVA: usize = 15;
const ANN_SBVA: usize = 16;
const ANN_GBVR: usize = 17;
const ANN_GWVR: usize = 18;
const ANN_GSVR: usize = 19;
const ANN_GLVR: usize = 20;
const ANN_GRPCR: usize = 21;
const ANN_SBVR: usize = 22;
const ANN_SWVR: usize = 23;
const ANN_SSVR: usize = 24;
const ANN_RPCR: usize = 25;
const ANN_LINER: usize = 26;
const ANN_RECTR: usize = 27;
const ANN_FRECTR: usize = 28;
const ANN_PIXELR: usize = 29;
const ANN_GBVAR: usize = 30;
const ANN_GWVAR: usize = 31;
const ANN_SBVAR: usize = 32;
const ANN_ACK: usize = 33;
const ANN_NACK: usize = 34;
const ANN_SWVA: usize = 35;
const ANN_SWVAR: usize = 36;
const ANN_GCV: usize = 37;
const ANN_GCVR: usize = 38;
const ANN_SCV: usize = 39;
const ANN_SCVR: usize = 40;
const ANN_BIT: usize = 41;
const ANN_FIELD: usize = 42;
const ANN_WARN: usize = 43;

const CMDS_WITH_HIGH_BYTES: [u8; 5] = [0xA0, 0xD7, 0xE7, 0xE2, 0xE3];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "amulet_ascii",
    name: "Amulet ASCII",
    longname: "Amulet LCD ASCII",
    desc: "Amulet Technologies LCD controller ASCII protocol.",
    license: "gplv3+",
    inputs: &["uart"],
    outputs: &[],
    tags: &["Display"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc {
            id: "ms_chan",
            desc: "Master -> slave channel",
            default: OptionValue::s("RX"),
            values: &[OptionValue::s("RX"), OptionValue::s("TX")],
        },
        OptionDesc {
            id: "sm_chan",
            desc: "Slave -> master channel",
            default: OptionValue::s("TX"),
            values: &[OptionValue::s("RX"), OptionValue::s("TX")],
        },
    ],
    annotations: &[
        AnnClass { id: "page", desc: "Jump to page" },
        AnnClass { id: "gbv", desc: "Get byte variable" },
        AnnClass { id: "gwv", desc: "Get word variable" },
        AnnClass { id: "gsv", desc: "Get string variable" },
        AnnClass { id: "glv", desc: "Get label variable" },
        AnnClass { id: "grpc", desc: "Get RPC buffer" },
        AnnClass { id: "sbv", desc: "Set byte variable" },
        AnnClass { id: "swv", desc: "Set word variable" },
        AnnClass { id: "ssv", desc: "Set string variable" },
        AnnClass { id: "rpc", desc: "Invoke RPC" },
        AnnClass { id: "line", desc: "Draw line" },
        AnnClass { id: "rect", desc: "Draw rectangle" },
        AnnClass { id: "frect", desc: "Draw filled rectangle" },
        AnnClass { id: "pixel", desc: "Draw pixel" },
        AnnClass { id: "gbva", desc: "Get byte variable array" },
        AnnClass { id: "gwva", desc: "Get word variable array" },
        AnnClass { id: "sbva", desc: "Set byte variable array" },
        AnnClass { id: "gbvr", desc: "Get byte variable reply" },
        AnnClass { id: "gwvr", desc: "Get word variable reply" },
        AnnClass { id: "gsvr", desc: "Get string variable reply" },
        AnnClass { id: "glvr", desc: "Get label variable reply" },
        AnnClass { id: "grpcr", desc: "Get RPC buffer reply" },
        AnnClass { id: "sbvr", desc: "Set byte variable reply" },
        AnnClass { id: "swvr", desc: "Set word variable reply" },
        AnnClass { id: "ssvr", desc: "Set string variable reply" },
        AnnClass { id: "rpcr", desc: "Invoke RPC reply" },
        AnnClass { id: "liner", desc: "Draw line reply" },
        AnnClass { id: "rectr", desc: "Draw rectangle" },
        AnnClass { id: "frectr", desc: "Draw filled rectangle reply" },
        AnnClass { id: "pixelr", desc: "Draw pixel reply" },
        AnnClass { id: "gbvar", desc: "Get byte variable array reply" },
        AnnClass { id: "gwvar", desc: "Get word variable array reply" },
        AnnClass { id: "sbvar", desc: "Set byte variable array reply" },
        AnnClass { id: "ack", desc: "Acknowledgment" },
        AnnClass { id: "nack", desc: "Negative acknowledgment" },
        AnnClass { id: "swva", desc: "Set word variable array" },
        AnnClass { id: "swvar", desc: "Set word variable array reply" },
        AnnClass { id: "gcv", desc: "Get color variable" },
        AnnClass { id: "gcvr", desc: "Get color variable reply" },
        AnnClass { id: "scv", desc: "Set color variable" },
        AnnClass { id: "scvr", desc: "Set color variable reply" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "field", desc: "Field" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[41] },
        AnnRow { id: "fields", desc: "Fields", classes: &[42] },
        AnnRow {
            id: "commands",
            desc: "Commands",
            classes: &[
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
                23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
            ],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[43] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes the ASCII protocol\n\
for Amulet LCD display controllers.\n\
\n\
Currently the decoder treats both RX and TX the same way, decoding all\n\
message types.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Amulet::new())
}

/// The command name and description of a state byte.
fn command(code: u8) -> Option<(&'static str, &'static str)> {
    CMDS.iter().find(|(c, _, _)| *c == code).map(|(_, name, desc)| (*name, *desc))
}

/// The annotation class index of a command byte.
fn command_class(code: u8) -> Option<usize> {
    CMDS.iter().position(|(c, _, _)| *c == code)
}

struct Amulet {
    ss: u64,
    es: u64,
    ss_cmd: u64,
    es_cmd: u64,
    ss_field: u64,
    es_field: u64,
    state: Option<u8>,
    cmdstate: i64,
    addr: u32,
    value_num: i64,
    value_str: String,
    checksum: u32,
    flags: u32,
}

impl Amulet {
    fn new() -> Self {
        Amulet {
            ss: 0,
            es: 0,
            ss_cmd: 0,
            es_cmd: 0,
            ss_field: 0,
            es_field: 0,
            state: None,
            cmdstate: 0,
            addr: 0,
            value_num: 0,
            value_str: String::new(),
            checksum: 0,
            flags: 0,
        }
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    fn putf(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_field, self.es_field, class, texts);
    }

    fn putc(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_cmd, self.es_cmd, class, texts);
    }

    /// `cmd_ann_list`: the texts of a command's own annotation.
    fn cmd_ann_list(&self) -> Result<Vec<String>> {
        let code = self.state.ok_or_else(|| Error::arg("command state is not set"))?;
        let (name, desc) = command(code).ok_or_else(|| Error::arg("unknown command"))?;
        Ok(vec![
            format!("Command: {desc} ({name})"),
            format!("Command: {desc}"),
            format!("Cmd: {desc}"),
            format!("Cmd: {name}"),
            name.to_owned(),
        ])
    }

    /// `emit_cmd_byte`: the command annotation of the first byte.
    fn emit_cmd_byte(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.ss_cmd = self.ss;
        let texts = self.cmd_ann_list()?;
        self.putx(ctx, ANN_FIELD, texts);
        Ok(())
    }

    /// `emit_addr_bytes`: the two hexadecimal address nibbles.
    fn emit_addr_bytes(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 2 {
            let c = char_of(pdata)?;
            self.ss_field = self.ss;
            self.addr = hex_nibble(pdata)? << 4;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("Addr h 0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            let c = char_of(pdata)?;
            self.es_field = self.es;
            self.addr += hex_nibble(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("Addr l 0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
        }
        Ok(())
    }

    /// `emit_cmd_end`: the command's annotation, then a new command may
    /// start.
    fn emit_cmd_end(&mut self, ctx: &mut dyn Context, class: usize) -> Result<()> {
        self.es_cmd = self.es;
        let texts = self.cmd_ann_list()?;
        self.putc(ctx, class, texts);
        self.state = None;
        Ok(())
    }

    fn emit_not_implemented(&mut self, ctx: &mut dyn Context, class: usize) -> Result<()> {
        self.es_cmd = self.es;
        self.putc(ctx, ANN_WARN, vec!["Command not decoded".into(), "Not decoded".into()]);
        self.emit_cmd_end(ctx, class)
    }

    /// `handle_read`: command byte, address high nibble, address low
    /// nibble.
    fn handle_read(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 1 {
            self.emit_cmd_byte(ctx)?;
            self.addr = 0;
        } else if self.cmdstate == 2 || self.cmdstate == 3 {
            self.emit_addr_bytes(ctx, pdata)?;
        }
        self.cmdstate += 1;
        Ok(())
    }

    fn handle_set_common(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 1 {
            self.addr = 0;
        }
        self.emit_addr_bytes(ctx, pdata)
    }

    fn handle_string(&mut self, ctx: &mut dyn Context, pdata: i64, ann_class: usize) -> Result<()> {
        // TODO: unicode / string modifiers...
        self.handle_set_common(ctx, pdata)?;
        if self.cmdstate == 4 {
            self.ss_field = self.ss;
            self.value_str.clear();
        }
        if pdata == 0x00 {
            // Null terminated string ends.
            self.es_field = self.es;
            self.putx(ctx, ANN_BIT, vec!["NULL".into()]);
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Value: {}", self.value_str),
                    format!("Val: {}", self.value_str),
                    self.value_str.clone(),
                ],
            );
            self.emit_cmd_end(ctx, ann_class)?;
            return Ok(());
        }
        if self.cmdstate > 3 {
            let c = char_of(pdata)?;
            self.value_str.push(c);
            self.putx(ctx, ANN_BIT, vec![c.to_string()]);
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// One command byte, dispatched by the current state.
    fn handle(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        let code = self.state.ok_or_else(|| Error::arg("unknown command"))?;
        match code {
            // Page change 0xA0, 0x02, index_high, index_low, checksum
            0xA0 => self.handle_page(ctx, pdata),
            0xD0 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GBV)
            }
            0xD1 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GWV)
            }
            0xD2 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GSV)
            }
            0xD3 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GLV)
            }
            0xD4 => self.handle_grpc(ctx, pdata),
            0xDD => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GBVA)
            }
            0xDE => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GWVA)
            }
            0xF4 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_GCV)
            }
            0xD5 => self.handle_sbv(ctx, pdata),
            0xD6 => self.handle_swv(ctx, pdata),
            0xD7 => self.handle_string(ctx, pdata, ANN_SSV),
            0xDF => self.handle_sbva(ctx, pdata),
            0xF2 => self.handle_swva(ctx, pdata),
            0xF6 => {
                if self.cmdstate == 8 {
                    self.emit_not_implemented(ctx, ANN_SCV)?;
                }
                self.cmdstate += 1;
                Ok(())
            }
            0xD8 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_RPC)
            }
            0xD9 => Err(Error::arg("name 'decode_coords' is not defined")),
            0xDA => Err(Error::arg("name 'decode_coords' is not defined")),
            0xDB => Err(Error::arg("name 'decode_coords' is not defined")),
            0xDC => {
                self.es_cmd = self.es;
                self.putc(
                    ctx,
                    ANN_WARN,
                    vec!["Draw pixel documentation is missing.".into(), "Undocumented".into()],
                );
                self.state = None;
                Ok(())
            }
            0xE0 => Err(Error::arg("'Decoder' object has no attribute 'emit_add_bytes'")),
            0xE1 => Err(Error::arg("'Decoder' object has no attribute 'emit_add_bytes'")),
            0xE2 => self.handle_string(ctx, pdata, ANN_GSVR),
            0xE3 => self.handle_string(ctx, pdata, ANN_GLVR),
            0xE4 => Err(Error::arg("'Decoder' object has no attribute 'handle_addr'")),
            0xE5 => self.handle_sbvr(ctx, pdata),
            0xE6 => self.handle_swvr(ctx, pdata),
            0xE7 => self.handle_string(ctx, pdata, ANN_SSVR),
            0xE8 => {
                self.handle_read(ctx, pdata)?;
                self.emit_cmd_end(ctx, ANN_RPCR)
            }
            0xE9 => Err(Error::arg("name 'decode_coords' is not defined")),
            0xEA => Err(Error::arg("name 'decode_coords' is not defined")),
            0xEB => Err(Error::arg("name 'decode_coords' is not defined")),
            0xEC => {
                self.es_cmd = self.es;
                self.putc(
                    ctx,
                    ANN_WARN,
                    vec!["Draw pixel documentation is missing.".into(), "Undocumented".into()],
                );
                self.state = None;
                Ok(())
            }
            0xED => self.handle_gbvar(ctx, pdata),
            0xEE => self.handle_gwvar(ctx, pdata),
            0xEF => self.handle_sbvar(ctx, pdata),
            0xF0 => {
                let texts = self.cmd_ann_list()?;
                self.putx(ctx, ANN_ACK, texts);
                self.state = None;
                Ok(())
            }
            0xF1 => {
                let texts = self.cmd_ann_list()?;
                self.putx(ctx, ANN_NACK, texts);
                self.state = None;
                Ok(())
            }
            0xF3 => self.handle_swvar(ctx, pdata),
            0xF5 => {
                if self.cmdstate == 8 {
                    self.emit_not_implemented(ctx, ANN_SCV)?;
                }
                self.cmdstate += 1;
                Ok(())
            }
            0xF7 => {
                if self.cmdstate == 8 {
                    self.emit_not_implemented(ctx, ANN_SCV)?;
                }
                self.cmdstate += 1;
                Ok(())
            }
            _ => Err(Error::arg("unknown command")),
        }
    }

    /// Page change 0xA0, 0x02, index_high, index_low, checksum.
    fn handle_page(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 2 {
            if pdata == 0x02 {
                self.ss_field = self.ss_cmd;
                self.es_field = self.es;
                let texts = self.cmd_ann_list()?;
                self.putf(ctx, ANN_FIELD, texts);
                self.checksum = 0xA0 + 0x02;
            } else {
                self.putx(
                    ctx,
                    ANN_WARN,
                    vec!["Illegal second byte for page change".into(), "Illegal byte".into()],
                );
                self.state = None;
            }
        } else if self.cmdstate == 3 {
            self.ss_field = self.ss;
            self.checksum += pdata as u32;
            // `self.page` is never initialised upstream.
            return Err(Error::arg("'Decoder' object has no attribute 'page'"));
        } else if self.cmdstate == 4 || self.cmdstate == 5 {
            return Err(Error::arg("'Decoder' object has no attribute 'page'"));
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Get RPC buffer: command byte, two flags nibbles.
    fn handle_grpc(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 2 {
            self.ss_field = self.ss;
            self.flags = hex_nibble(pdata)? << 4;
        } else if self.cmdstate == 3 {
            self.flags += hex_nibble(pdata)?;
            self.es_field = self.es;
            self.putf(ctx, ANN_FIELD, vec![format!("RPC flag: 0x{:02X}", self.flags)]);
            self.emit_cmd_end(ctx, ANN_GRPC)?;
        }
        Ok(())
    }

    /// Set byte value: two hexadecimal character bytes.
    fn handle_sbv(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        self.handle_set_common(ctx, pdata)?;
        if self.cmdstate == 4 {
            self.ss_field = self.ss;
            self.value_str = char_of(pdata)?.to_string();
        } else if self.cmdstate == 5 {
            self.value_str.push(char_of(pdata)?);
            self.es_field = self.es;
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Value: 0x{}", self.value_str),
                    format!("Val: 0x{}", self.value_str),
                    format!("0x{}", self.value_str),
                ],
            );
            self.emit_cmd_end(ctx, ANN_SBV)?;
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Set word value: four hexadecimal nibbles.
    fn handle_swv(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        self.handle_set_common(ctx, pdata)?;
        if self.cmdstate > 3 {
            let nibble = self.cmdstate - 4;
            if nibble == 0 {
                self.ss_field = self.ss;
                self.value_num = 0;
            }
            self.value_num += i64::from(hex_nibble(pdata)?) << (12 - (4 * nibble));
            if nibble == 3 {
                self.es_field = self.es;
                self.putf(
                    ctx,
                    ANN_FIELD,
                    vec![
                        format!("Value: 0x{:04x}", self.value_num),
                        format!("Val: 0x{:04x}", self.value_num),
                        format!("0x{:04x}", self.value_num),
                    ],
                );
                self.emit_cmd_end(ctx, ANN_SWV)?;
                return Ok(());
            }
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Set byte value array. Upstream raises `NameError` for the
    /// undefined `stage` as soon as `cmdstate` is not 2 or 3.
    fn handle_sbva(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 2 {
            self.addr = hex_nibble(pdata)? << 4;
            self.ss_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            self.addr += hex_nibble(pdata)?;
            self.es_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
        } else {
            return Err(Error::arg("name 'stage' is not defined"));
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Set word value array. Upstream only advances `cmdstate` on a full
    /// word, so `cmdstate == 2` repeats the address high nibble.
    fn handle_swva(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        let nibble = (self.cmdstate - 3).rem_euclid(4);
        if self.cmdstate == 2 {
            self.addr = hex_nibble(pdata)? << 4;
            self.ss_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            self.addr += hex_nibble(pdata)?;
            self.es_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
            self.value_num = 0;
        } else {
            self.value_num += i64::from(hex_nibble(pdata)?) << (12 - (4 * nibble));
            if nibble == 0 {
                if pdata == 0x00 {
                    // Null terminated list.
                    self.emit_cmd_end(ctx, ANN_SWVA)?;
                    return Ok(());
                }
                self.ss_field = self.ss;
            }
            if nibble == 3 {
                self.es_field = self.es;
                self.putf(
                    ctx,
                    ANN_FIELD,
                    vec![
                        format!("Value 0x{:04X}", self.value_num),
                        format!("0x{:04X}", self.value_num),
                    ],
                );
                self.cmdstate += 1;
            }
        }
        Ok(())
    }

    /// Get byte variable reply. Upstream's `emit_add_bytes` does not
    /// exist, so every byte raises `AttributeError`.
    fn handle_gbvr(&mut self, _ctx: &mut dyn Context, _pdata: i64) -> Result<()> {
        Err(Error::arg("'Decoder' object has no attribute 'emit_add_bytes'"))
    }

    /// Get word variable reply, the same missing method.
    fn handle_gwvr(&mut self, _ctx: &mut dyn Context, _pdata: i64) -> Result<()> {
        Err(Error::arg("'Decoder' object has no attribute 'emit_add_bytes'"))
    }

    fn handle_sbvr(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        self.handle_set_common(ctx, pdata)?;
        if self.cmdstate == 4 {
            self.ss_field = self.ss;
            self.value_str = char_of(pdata)?.to_string();
        } else if self.cmdstate == 5 {
            self.value_str.push(char_of(pdata)?);
            self.es_field = self.es;
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Value: 0x{}", self.value_str),
                    format!("Val: 0x{}", self.value_str),
                    format!("0x{}", self.value_str),
                ],
            );
            self.emit_cmd_end(ctx, ANN_SBVR)?;
        }
        self.cmdstate += 1;
        Ok(())
    }

    fn handle_swvr(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        self.handle_set_common(ctx, pdata)?;
        if self.cmdstate == 4 {
            self.ss_field = self.ss;
            self.value_num = (pdata - 0x30) << 4;
        } else if self.cmdstate == 5 {
            self.value_num += pdata - 0x30;
            self.value_num <<= 8;
        } else if self.cmdstate == 6 {
            self.value_num += (pdata - 0x30) << 4;
        } else if self.cmdstate == 7 {
            self.value_num += pdata - 0x30;
            self.es_field = self.es;
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Value: {}", hex4_lower(self.value_num)),
                    format!("Val: {}", hex4_lower(self.value_num)),
                    hex4_lower(self.value_num),
                ],
            );
            self.emit_cmd_end(ctx, ANN_SWVR)?;
            self.state = None;
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Get byte variable array. Upstream raises `NameError` for `stage`.
    fn handle_gbvar(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 2 {
            self.addr = hex_nibble(pdata)? << 4;
            self.ss_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            self.addr += hex_nibble(pdata)?;
            self.es_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
        } else {
            return Err(Error::arg("name 'stage' is not defined"));
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Get word variable array. Upstream only advances on a full word,
    /// so `cmdstate == 2` repeats the address high nibble.
    fn handle_gwvar(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        let nibble = (self.cmdstate - 3).rem_euclid(4);
        if self.cmdstate == 2 {
            self.addr = hex_nibble(pdata)? << 4;
            self.ss_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            self.addr += hex_nibble(pdata)?;
            self.es_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
            self.value_num = 0;
        } else {
            self.value_num += i64::from(hex_nibble(pdata)?) << (12 - (4 * nibble));
            if nibble == 0 {
                if pdata == 0x00 {
                    // Null terminated list.
                    self.emit_cmd_end(ctx, ANN_GWVAR)?;
                    return Ok(());
                }
                self.ss_field = self.ss;
            }
            if nibble == 3 {
                self.es_field = self.es;
                self.putf(
                    ctx,
                    ANN_FIELD,
                    vec![
                        format!("Value 0x{:04X}", self.value_num),
                        format!("0x{:04X}", self.value_num),
                    ],
                );
                self.cmdstate += 1;
            }
        }
        Ok(())
    }

    /// Set byte variable array reply. Upstream raises `NameError`.
    fn handle_sbvar(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        if self.cmdstate == 2 {
            self.addr = hex_nibble(pdata)? << 4;
            self.ss_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            self.addr += hex_nibble(pdata)?;
            self.es_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
        } else {
            return Err(Error::arg("name 'stage' is not defined"));
        }
        self.cmdstate += 1;
        Ok(())
    }

    /// Set word variable array reply. Upstream only advances on a full
    /// word, so `cmdstate == 2` repeats the address high nibble.
    fn handle_swvar(&mut self, ctx: &mut dyn Context, pdata: i64) -> Result<()> {
        let nibble = (self.cmdstate - 3).rem_euclid(4);
        if self.cmdstate == 2 {
            self.addr = hex_nibble(pdata)? << 4;
            self.ss_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address high nibble: {c}"),
                    format!("Addr high 0x{c}"),
                    format!("0x{c}"),
                ],
            );
        } else if self.cmdstate == 3 {
            self.addr += hex_nibble(pdata)?;
            self.es_field = self.ss;
            let c = char_of(pdata)?;
            self.putx(
                ctx,
                ANN_BIT,
                vec![
                    format!("Address low nibble: {c}"),
                    format!("Addr low 0x{c}"),
                    format!("0x{c}"),
                ],
            );
            self.putf(
                ctx,
                ANN_FIELD,
                vec![
                    format!("Address: 0x{:02X}", self.addr),
                    format!("Addr: 0x{:02X}", self.addr),
                    format!("0x{:02X}", self.addr),
                ],
            );
            self.value_num = 0;
        } else {
            self.value_num += i64::from(hex_nibble(pdata)?) << (12 - (4 * nibble));
            if nibble == 0 {
                if pdata == 0x00 {
                    // Null terminated list.
                    self.emit_cmd_end(ctx, ANN_SWVAR)?;
                    return Ok(());
                }
                self.ss_field = self.ss;
            }
            if nibble == 3 {
                self.es_field = self.es;
                self.putf(
                    ctx,
                    ANN_FIELD,
                    vec![
                        format!("Value 0x{:04X}", self.value_num),
                        format!("0x{:04X}", self.value_num),
                    ],
                );
                self.cmdstate += 1;
            }
        }
        Ok(())
    }
}

/// `chr(pdata)`.
fn char_of(pdata: i64) -> Result<char> {
    let value = u32::try_from(pdata).map_err(|_| Error::arg("chr() arg not in range"))?;
    char::from_u32(value).ok_or_else(|| Error::arg("chr() arg not in range"))
}

/// `int(chr(pdata), 16)` for the single character a nibble byte holds.
fn hex_nibble(pdata: i64) -> Result<u32> {
    let c = char_of(pdata)?;
    c.to_digit(16).ok_or_else(|| Error::arg("invalid literal for int() with base 16"))
}

/// Python's `'%04x' % value`, including a negative value's sign.
fn hex4_lower(value: i64) -> String {
    if value < 0 { format!("-{:03x}", value.unsigned_abs()) } else { format!("{value:04x}") }
}

impl Decoder for Amulet {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Amulet::new();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("amulet_ascii is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, pdata) = unpack_uart(item)?;

        self.ss = ss;
        self.es = es;

        if ptype != "DATA" {
            return Ok(());
        }

        // Handle commands.
        let pdata = data_value(pdata)?;
        let abort_current = (0xD0..=0xF7).contains(&pdata)
            && !self.state.is_some_and(|state| CMDS_WITH_HIGH_BYTES.contains(&state))
            && self.state.is_some();
        if abort_current {
            self.putx(
                ctx,
                ANN_WARN,
                vec!["Command aborted by invalid byte".into(), "Abort".into()],
            );
            self.state = Some(pdata as u8);
            self.emit_cmd_byte(ctx)?;
            self.cmdstate = 1;
        }
        if self.state.is_none() {
            self.state = Some(pdata as u8);
            // `emit_cmd_byte` raises KeyError for a state that is not in
            // `cmds`, and `decode` turns that into the warning below.
            if command(self.state.unwrap_or(0)).is_none() {
                self.putx(ctx, ANN_WARN, vec![format!("Unknown command: 0x{pdata:02x}")]);
                self.state = None;
                return Ok(());
            }
            self.emit_cmd_byte(ctx)?;
            self.cmdstate = 1;
        }
        if command(self.state.unwrap_or(0)).is_none() {
            // `except KeyError:` in upstream's decode.
            self.putx(ctx, ANN_WARN, vec![format!("Unknown command: 0x{pdata:02x}")]);
            self.state = None;
            return Ok(());
        }
        self.handle(ctx, pdata)
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
fn data_value(pdata: &Item) -> Result<i64> {
    let Item::List(items) = pdata else {
        return Err(Error::arg("UART data is not a sequence"));
    };
    match items.first() {
        Some(Item::Int(v)) => Ok(*v),
        _ => Err(Error::arg("UART data has no byte value")),
    }
}
