// Counterpart in sigrok:
//   libsigrokdecode/decoders/hdcp/pd.py,
//   Copyright (C) 2018 Dave Craig <dcraig@brightsign.biz>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented classes and texts.
#![allow(clippy::assigning_clones, clippy::doc_markdown, clippy::match_same_arms)]

//! HDCP over HDMI, on top of the I2C decoder.
//!
//! Only the HDCP I2C slave address 0x3a is decoded. A write of one of
//! the register offsets in `write_items` names the transaction; RxStatus
//! and Bstatus are summarised from two bytes, read and write messages
//! from the first buffered byte. The message annotations are numbered by
//! the HDCP message id itself (`message-0x02` for AKE_Init and so on).

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "hdcp",
    name: "HDCP",
    longname: "HDCP over HDMI",
    desc: "HDCP protocol over HDMI.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &["hdcp"],
    tags: &["PC", "Security/crypto"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "message-0x00", desc: "Message 0x00" },
        AnnClass { id: "message-0x01", desc: "Message 0x01" },
        AnnClass { id: "message-0x02", desc: "Message 0x02" },
        AnnClass { id: "message-0x03", desc: "Message 0x03" },
        AnnClass { id: "message-0x04", desc: "Message 0x04" },
        AnnClass { id: "message-0x05", desc: "Message 0x05" },
        AnnClass { id: "message-0x06", desc: "Message 0x06" },
        AnnClass { id: "message-0x07", desc: "Message 0x07" },
        AnnClass { id: "message-0x08", desc: "Message 0x08" },
        AnnClass { id: "message-0x09", desc: "Message 0x09" },
        AnnClass { id: "message-0x0A", desc: "Message 0x0A" },
        AnnClass { id: "message-0x0B", desc: "Message 0x0B" },
        AnnClass { id: "message-0x0C", desc: "Message 0x0C" },
        AnnClass { id: "message-0x0D", desc: "Message 0x0D" },
        AnnClass { id: "message-0x0E", desc: "Message 0x0E" },
        AnnClass { id: "message-0x0F", desc: "Message 0x0F" },
        AnnClass { id: "message-0x10", desc: "Message 0x10" },
        AnnClass { id: "message-0x11", desc: "Message 0x11" },
        AnnClass { id: "summary", desc: "Summary" },
        AnnClass { id: "warnings", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow {
            id: "messages",
            desc: "Messages",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],
        },
        AnnRow { id: "summary", desc: "Summary", classes: &[18] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[19] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes HDCP messages.\n\
\n\
Details:\n\
https://www.digital-cp.com/sites/default/files/specifications/HDCP%20on%20HDMI%20Specification%20Rev2_2_Final1.pdf",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Hdcp::default())
}

/// `msg_ids`, keyed by HDCP message id.
fn msg_id(id: i64) -> &'static str {
    match id {
        2 => "AKE_Init",
        3 => "AKE_Send_Cert",
        4 => "AKE_No_stored_km",
        5 => "AKE_Stored_km",
        7 => "AKE_Send_H_prime",
        8 => "AKE_Send_Pairing_Info",
        9 => "LC_Init",
        10 => "LC_Send_L_prime",
        11 => "SKE_Send_Eks",
        12 => "RepeaterAuth_Send_ReceiverID_List",
        15 => "RepeaterAuth_Send_Ack",
        16 => "RepeaterAuth_Stream_Manage",
        17 => "RepeaterAuth_Stream_Ready",
        _ => "Invalid",
    }
}

/// `write_items`, the register offsets that name a transaction. The
/// ones in `BUFFER` switch to reading/writing the data buffer.
fn write_item(offset: i64) -> Option<&'static str> {
    Some(match offset {
        0x00 => "1.4 Bksv - Receiver KSV",
        0x08 => "1.4 Ri' - Link Verification",
        0x0a => "1.4 Pj' - Enhanced Link Verification",
        0x10 => "1.4 Aksv - Transmitter KSV",
        0x15 => "1.4 Ainfo - Transmitter KSV",
        0x18 => "1.4 An - Session random number",
        0x20 => "1.4 V'H0",
        0x24 => "1.4 V'H1",
        0x28 => "1.4 V'H2",
        0x2c => "1.4 V'H3",
        0x30 => "1.4 V'H4",
        0x40 => "1.4 Bcaps",
        0x41 => "1.4 Bstatus",
        0x43 => "1.4 KSV FIFO",
        0x50 => "HDCP2Version",
        0x60 => "Write_Message",
        0x70 => "RxStatus",
        0x80 => "Read_Message",
        _ => return None,
    })
}

/// Python's `str()` of a bool, as `%s` formats it.
fn py_bool(v: bool) -> &'static str {
    if v { "True" } else { "False" }
}

/// The state machine's states, as upstream's `self.state`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    WriteOffset,
    BufferData,
}

/// The HDCP decoder.
pub struct Hdcp {
    state: State,
    stack: Vec<i64>,
    msg: i64,
    ss: u64,
    es: u64,
    ss_block: u64,
    es_block: u64,
    /// The transaction type named by the register offset, `''` if none.
    r#type: String,
}

impl Default for Hdcp {
    fn default() -> Self {
        Hdcp {
            state: State::Idle,
            stack: Vec::new(),
            msg: -1,
            ss: 0,
            es: 0,
            ss_block: 0,
            es_block: 0,
            r#type: String::new(),
        }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("hdcp: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("hdcp: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("hdcp: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

impl Hdcp {
    /// As upstream's `reset()`, which runs on every new START.
    fn reset(&mut self) {
        self.state = State::Idle;
        self.stack.clear();
        self.msg = -1;
        self.ss = 0;
        self.es = 0;
        self.ss_block = 0;
        self.es_block = 0;
        self.r#type.clear();
    }

    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, class, texts);
    }

    fn finish(&mut self, ctx: &mut dyn Context) {
        if self.r#type.is_empty() {
            return;
        }
        if self.stack.is_empty() {
            let text = self.r#type.clone();
            self.putb(ctx, 18, vec![text]);
            return;
        }
        match self.r#type.as_str() {
            "RxStatus" => {
                let low = self.stack.pop().unwrap_or(0);
                let high = self.stack.pop().unwrap_or(0);
                let rxstatus = (high << 8) | low;
                let reauth_req = (rxstatus & 0x800) != 0;
                let ready = (rxstatus & 0x400) != 0;
                let length = rxstatus & 0x3ff;
                let text = format!(
                    "{}, reauth {}, ready {}, length {}",
                    self.r#type,
                    py_bool(reauth_req),
                    py_bool(ready),
                    length
                );
                self.putb(ctx, 18, vec![text]);
            }
            "1.4 Bstatus" => {
                let low = self.stack.pop().unwrap_or(0);
                let high = self.stack.pop().unwrap_or(0);
                let bstatus = (high << 8) | low;
                let device_count = bstatus & 0x7f;
                let depth = (bstatus & 0x700) >> 8;
                let hdmi_mode = (bstatus & 0x1000) != 0;
                let text = format!(
                    "{}, {} devices, depth {}, hdmi mode {}",
                    self.r#type,
                    device_count,
                    depth,
                    py_bool(hdmi_mode)
                );
                self.putb(ctx, 18, vec![text]);
            }
            "Read_Message" => {
                let msg = self.stack.remove(0);
                let class = msg as usize;
                let text = format!("{}, {}", self.r#type, msg_id(msg));
                self.putb(ctx, class, vec![text]);
            }
            "Write_Message" => {
                let msg = self.stack.remove(0);
                let class = msg as usize;
                let text = format!("{}, {}", self.r#type, msg_id(msg));
                self.putb(ctx, class, vec![text]);
            }
            "HDCP2Version" => {
                let version = self.stack.remove(0);
                let text = if (version & 0x4) != 0 { "HDCP2" } else { "NOT HDCP2" };
                self.putb(ctx, 18, vec![text.to_owned()]);
            }
            _ => {
                let text = self.r#type.clone();
                self.putb(ctx, 18, vec![text]);
            }
        }
    }
}

impl Decoder for Hdcp {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Hdcp::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("hdcp takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Collect the 'BITS' packet, then return. The next packet is
        // guaranteed to belong to these bits we just stored.
        if cmd == "BITS" {
            return Ok(());
        }

        self.ss = ss;
        self.es = es;

        match self.state {
            State::Idle => {
                // Wait for an I2C START condition.
                if cmd == "START" {
                    self.reset();
                    self.ss_block = ss;
                } else if cmd != "START REPEAT" {
                    return Ok(());
                }
                self.state = State::GetSlaveAddr;
            }
            State::GetSlaveAddr => {
                if cmd == "ADDRESS READ" {
                    self.state = State::BufferData;
                    if int_value(data)? != 0x3a {
                        self.state = State::Idle;
                    }
                } else if cmd == "ADDRESS WRITE" {
                    self.state = State::WriteOffset;
                    if int_value(data)? != 0x3a {
                        self.state = State::Idle;
                    }
                }
            }
            State::WriteOffset => {
                if cmd == "DATA WRITE" {
                    let databyte = int_value(data)?;
                    if let Some(name) = write_item(databyte) {
                        self.r#type = name.to_owned();
                    }
                    if matches!(databyte, 0x10 | 0x15 | 0x18 | 0x60) {
                        self.state = State::BufferData;
                    }
                    // If we are reading, then jump back to IDLE for a start
                    // repeat. If we are writing, then just continue onwards.
                    if self.state == State::BufferData {
                    } else if !self.r#type.is_empty() {
                        self.state = State::Idle;
                    }
                }
            }
            State::BufferData => {
                if cmd == "STOP" || cmd == "NACK" {
                    self.es_block = es;
                    self.state = State::Idle;
                    self.finish(ctx);
                } else if cmd == "DATA READ" || cmd == "DATA WRITE" {
                    // Stack up our data bytes.
                    self.stack.push(int_value(data)?);
                }
            }
        }
        Ok(())
    }
}

fn int_value(data: &Item) -> Result<i64> {
    match data {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::device("hdcp: packet has no data byte")),
    }
}
