// Counterpart in sigrok:
//   libsigrokdecode/decoders/atsha204a/pd.py,
//   Copyright (C) 2018 Michalis Pappas <mpappas@fastmail.fm>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented
//   opcodes, zones, status codes and texts.

//! Microchip ATSHA204A / ATECC508A crypto authentication protocol.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const WORD_ADDR_RESET: i64 = 0x00;
const WORD_ADDR_SLEEP: i64 = 0x01;
const WORD_ADDR_IDLE: i64 = 0x02;
const WORD_ADDR_COMMAND: i64 = 0x03;

static WORD_ADDR: [(i64, &str); 4] = [
    (WORD_ADDR_RESET, "RESET"),
    (WORD_ADDR_SLEEP, "SLEEP"),
    (WORD_ADDR_IDLE, "IDLE"),
    (WORD_ADDR_COMMAND, "COMMAND"),
];

const OPCODE_COUNTER: i64 = 0x24;
const OPCODE_DERIVE_KEY: i64 = 0x1c;
const OPCODE_DEV_REV: i64 = 0x30;
const OPCODE_ECDH: i64 = 0x43;
const OPCODE_GEN_DIG: i64 = 0x15;
const OPCODE_GEN_KEY: i64 = 0x40;
const OPCODE_HMAC: i64 = 0x11;
const OPCODE_CHECK_MAC: i64 = 0x28;
const OPCODE_LOCK: i64 = 0x17;
const OPCODE_MAC: i64 = 0x08;
const OPCODE_NONCE: i64 = 0x16;
const OPCODE_PAUSE: i64 = 0x01;
const OPCODE_PRIVWRITE: i64 = 0x46;
const OPCODE_RANDOM: i64 = 0x1b;
const OPCODE_READ: i64 = 0x02;
const OPCODE_SHA: i64 = 0x47;
const OPCODE_SIGN: i64 = 0x41;
const OPCODE_UPDATE_EXTRA: i64 = 0x20;
const OPCODE_VERIFY: i64 = 0x45;
const OPCODE_WRITE: i64 = 0x12;

/// `OPCODES`, in table order. Unknown bytes raise a `KeyError` upstream.
static OPCODES: [(i64, &str); 20] = [
    (0x01, "Pause"),
    (0x02, "Read"),
    (0x08, "MAC"),
    (0x11, "HMAC"),
    (0x12, "Write"),
    (0x15, "GenDig"),
    (0x16, "Nonce"),
    (0x17, "Lock"),
    (0x1b, "Random"),
    (0x1c, "DeriveKey"),
    (0x20, "UpdateExtra"),
    (0x24, "Counter"),
    (0x28, "CheckMac"),
    (0x30, "DevRev"),
    (0x40, "GenKey"),
    (0x41, "Sign"),
    (0x43, "ECDH"),
    (0x45, "Verify"),
    (0x46, "PrivWrite"),
    (0x47, "SHA"),
];

static ZONES: [(i64, &str); 3] = [(0x00, "CONFIG"), (0x01, "OTP"), (0x02, "DATA")];

static STATUS: [(i64, &str); 6] = [
    (0x00, "Command success"),
    (0x01, "Checkmac failure"),
    (0x03, "Parse error"),
    (0x0f, "Execution error"),
    (0x11, "Ready"),
    (0xff, "CRC / communications error"),
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    ReadRegs,
    WriteRegs,
}

/// One received or transmitted byte: sample range and value.
#[derive(Clone, Copy)]
struct Byte {
    ss: u64,
    es: u64,
    value: i64,
}

/// The atsha204a decoder.
pub struct Atsha204a {
    state: State,
    waddr: i64,
    opcode: i64,
    ss_block: u64,
    es_block: u64,
    bytes: Vec<Byte>,
}

impl Default for Atsha204a {
    fn default() -> Self {
        Atsha204a {
            state: State::Idle,
            waddr: -1,
            opcode: -1,
            ss_block: 0,
            es_block: 0,
            bytes: Vec::new(),
        }
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "atsha204a",
    name: "ATSHA204A",
    longname: "Microchip ATSHA204A",
    desc: "Microchip ATSHA204A family crypto authentication protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Security/crypto", "IC", "Memory"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "waddr", desc: "Word address" },
        AnnClass { id: "count", desc: "Count" },
        AnnClass { id: "opcode", desc: "Opcode" },
        AnnClass { id: "param1", desc: "Param1" },
        AnnClass { id: "param2", desc: "Param2" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "crc", desc: "CRC" },
        AnnClass { id: "status", desc: "Status" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "frame", desc: "Frame", classes: &[0, 1, 2, 3, 4, 5, 6] },
        AnnRow { id: "status", desc: "Status", classes: &[7] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[8] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: concat!(
        "This decoder stacks on top of the 'i2c' PD and decodes the\n",
        "Microchip ATSHA204A and ATECC508A crypto authentication protocol.\n",
        "\n",
        "The decoder might also support the following devices (untested):\n",
        " * ATSHA204\n",
        " * ATECC108\n",
        " * ATECC108A",
    ),
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Atsha204a::default())
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("atsha204a: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("atsha204a: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("atsha204a: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// `dict[key]`, which raises `KeyError` upstream.
fn lookup(table: &[(i64, &'static str)], key: i64) -> Result<&'static str> {
    table
        .iter()
        .find(|(k, _)| *k == key)
        .map(|(_, v)| *v)
        .ok_or_else(|| Error::device(format!("atsha204a: unknown key 0x{key:02X}")))
}

/// `' '.join(format(i[2], '02x') for i in s)`.
fn hexs(s: &[Byte]) -> String {
    s.iter().map(|b| format!("{:02x}", b.value)).collect::<Vec<_>>().join(" ")
}

impl Atsha204a {
    /// `putx`: annotate one byte's sample range.
    #[allow(clippy::unused_self)]
    fn putx(&self, ctx: &mut dyn Context, s: Byte, class: usize, texts: Vec<String>) {
        ctx.put_ann(s.ss, s.es, class, texts);
    }

    /// `puty`: annotate from the first byte's start to the last byte's end.
    #[allow(clippy::unused_self)]
    fn puty(&self, ctx: &mut dyn Context, s: &[Byte], class: usize, texts: Vec<String>) {
        ctx.put_ann(s[0].ss, s[1].es, class, texts);
    }

    /// `putz`: annotate an explicit sample range.
    #[allow(clippy::unused_self)]
    fn putz(&self, ctx: &mut dyn Context, ss: u64, es: u64, class: usize, texts: Vec<String>) {
        ctx.put_ann(ss, es, class, texts);
    }

    fn put_waddr(&self, ctx: &mut dyn Context, s: Byte) -> Result<()> {
        let name = lookup(&WORD_ADDR, s.value)?;
        self.putx(ctx, s, 0, vec![format!("Word addr: {name}")]);
        Ok(())
    }

    fn put_count(&self, ctx: &mut dyn Context, s: Byte) {
        self.putx(ctx, s, 1, vec![format!("Count: {}", s.value)]);
    }

    fn put_opcode(&self, ctx: &mut dyn Context, s: Byte) -> Result<()> {
        let name = lookup(&OPCODES, s.value)?;
        self.putx(ctx, s, 2, vec![format!("Opcode: {name}")]);
        Ok(())
    }

    fn put_param1(&self, ctx: &mut dyn Context, s: Byte) -> Result<()> {
        let op = self.opcode;
        if [
            OPCODE_CHECK_MAC,
            OPCODE_COUNTER,
            OPCODE_DEV_REV,
            OPCODE_ECDH,
            OPCODE_GEN_KEY,
            OPCODE_HMAC,
            OPCODE_MAC,
            OPCODE_NONCE,
            OPCODE_RANDOM,
            OPCODE_SHA,
            OPCODE_SIGN,
            OPCODE_VERIFY,
        ]
        .contains(&op)
        {
            self.putx(ctx, s, 3, vec![format!("Mode: {:02X}", s.value)]);
        } else if op == OPCODE_DERIVE_KEY {
            self.putx(ctx, s, 3, vec![format!("Random: {}", s.value)]);
        } else if op == OPCODE_PRIVWRITE {
            let yes = if s.value & 0x40 != 0 { "Yes" } else { "No" };
            self.putx(ctx, s, 3, vec![format!("Encrypted: {yes}")]);
        } else if op == OPCODE_GEN_DIG {
            let zone = lookup(&ZONES, s.value)?;
            self.putx(ctx, s, 3, vec![format!("Zone: {zone}")]);
        } else if op == OPCODE_LOCK {
            let zone = if s.value != 0 { "DATA/OTP" } else { "CONFIG" };
            let summary = if s.value & 0x80 != 0 { "Ignored" } else { "Used" };
            self.putx(ctx, s, 3, vec![format!("Zone: {zone}, Summary: {summary}")]);
        } else if op == OPCODE_PAUSE {
            self.putx(ctx, s, 3, vec![format!("Selector: {:02X}", s.value)]);
        } else if op == OPCODE_READ {
            let zone = lookup(&ZONES, s.value & 0x03)?;
            let len = if s.value & 0x90 != 0 { "32 bytes" } else { "4 bytes" };
            self.putx(ctx, s, 3, vec![format!("Zone: {zone}, Length: {len}")]);
        } else if op == OPCODE_WRITE {
            let zone = lookup(&ZONES, s.value & 0x03)?;
            let enc = if s.value & 0x40 != 0 { "Yes" } else { "No" };
            let len = if s.value & 0x90 != 0 { "32 bytes" } else { "4 bytes" };
            self.putx(ctx, s, 3, vec![format!("Zone: {zone}, Encrypted: {enc}, Length: {len}")]);
        } else {
            self.putx(ctx, s, 3, vec![format!("Param1: {:02X}", s.value)]);
        }
        Ok(())
    }

    fn put_param2(&self, ctx: &mut dyn Context, s: &[Byte]) {
        let op = self.opcode;
        // The two bytes are printed high byte first, as upstream's
        // `format(s[1][2], s[0][2])`.
        if op == OPCODE_DERIVE_KEY {
            self.puty(ctx, s, 4, vec![format!("TargetKey: {:02x} {:02x}", s[1].value, s[0].value)]);
        } else if [
            OPCODE_COUNTER,
            OPCODE_ECDH,
            OPCODE_GEN_KEY,
            OPCODE_PRIVWRITE,
            OPCODE_SIGN,
            OPCODE_VERIFY,
        ]
        .contains(&op)
        {
            self.puty(ctx, s, 4, vec![format!("KeyID: {:02x} {:02x}", s[1].value, s[0].value)]);
        } else if [OPCODE_NONCE, OPCODE_PAUSE, OPCODE_RANDOM].contains(&op) {
            self.puty(ctx, s, 4, vec![format!("Zero: {:02x} {:02x}", s[1].value, s[0].value)]);
        } else if [OPCODE_HMAC, OPCODE_MAC, OPCODE_CHECK_MAC, OPCODE_GEN_DIG].contains(&op) {
            self.puty(ctx, s, 4, vec![format!("SlotID: {:02x} {:02x}", s[1].value, s[0].value)]);
        } else if op == OPCODE_LOCK {
            self.puty(ctx, s, 4, vec![format!("Summary: {:02x} {:02x}", s[1].value, s[0].value)]);
        } else if [OPCODE_READ, OPCODE_WRITE].contains(&op) {
            self.puty(ctx, s, 4, vec![format!("Address: {:02x} {:02x}", s[1].value, s[0].value)]);
        } else if op == OPCODE_UPDATE_EXTRA {
            self.puty(ctx, s, 4, vec![format!("NewValue: {:02x}", s[0].value)]);
        } else {
            // `puty` still spans both bytes for the "-" text.
            self.puty(ctx, s, 4, vec!["-".to_owned()]);
        }
    }

    #[allow(clippy::too_many_lines)]
    fn put_data(&self, ctx: &mut dyn Context, s: &[Byte]) {
        if s.is_empty() {
            return;
        }
        let op = self.opcode;
        if op == OPCODE_CHECK_MAC {
            self.putz(ctx, s[0].ss, s[31].es, 5, vec![format!("ClientChal: {}", hexs(&s[0..32]))]);
            self.putz(
                ctx,
                s[32].ss,
                s[63].es,
                5,
                vec![format!("ClientResp: {}", hexs(&s[32..64]))],
            );
            self.putz(ctx, s[64].ss, s[76].es, 5, vec![format!("OtherData: {}", hexs(&s[64..77]))]);
        } else if op == OPCODE_DERIVE_KEY {
            self.putz(ctx, s[0].ss, s[31].es, 5, vec![format!("MAC: {}", hexs(s))]);
        } else if op == OPCODE_ECDH {
            self.putz(ctx, s[0].ss, s[31].es, 5, vec![format!("Pub X: {}", hexs(&s[0..32]))]);
            self.putz(ctx, s[32].ss, s[63].es, 5, vec![format!("Pub Y: {}", hexs(&s[32..64]))]);
        } else if op == OPCODE_GEN_DIG || op == OPCODE_GEN_KEY {
            // Upstream always ends at s[3], whatever the data length.
            self.putz(ctx, s[0].ss, s[3].es, 5, vec![format!("OtherData: {}", hexs(s))]);
        } else if op == OPCODE_MAC {
            self.putz(ctx, s[0].ss, s[31].es, 5, vec![format!("Challenge: {}", hexs(s))]);
        } else if op == OPCODE_PRIVWRITE {
            if s.len() > 36 {
                // Key + MAC. Both texts list all bytes, as upstream does.
                self.putz(ctx, s[0].ss, s[s.len() - 35].es, 5, vec![format!("Value: {}", hexs(s))]);
                self.putz(
                    ctx,
                    s[s.len() - 32].ss,
                    s[s.len() - 1].es,
                    5,
                    vec![format!("MAC: {}", hexs(s))],
                );
            } else {
                // Just value.
                self.putz(ctx, s[0].ss, s[s.len() - 1].es, 5, vec![format!("Value: {}", hexs(s))]);
            }
        } else if op == OPCODE_VERIFY {
            if s.len() >= 64 {
                // ECDSA components (always present).
                self.putz(ctx, s[0].ss, s[31].es, 5, vec![format!("ECDSA R: {}", hexs(&s[0..32]))]);
                self.putz(
                    ctx,
                    s[32].ss,
                    s[63].es,
                    5,
                    vec![format!("ECDSA S: {}", hexs(&s[32..64]))],
                );
            }
            if s.len() == 83 {
                // OtherData (follow ECDSA components in validate /
                // invalidate mode).
                self.putz(
                    ctx,
                    s[64].ss,
                    s[82].es,
                    5,
                    vec![format!("OtherData: {}", hexs(&s[64..83]))],
                );
            }
            if s.len() == 128 {
                // Public key components (follow ECDSA components in
                // external mode).
                self.putz(ctx, s[64].ss, s[95].es, 5, vec![format!("Pub X: {}", hexs(&s[64..96]))]);
                self.putz(
                    ctx,
                    s[96].ss,
                    s[127].es,
                    5,
                    vec![format!("Pub Y: {}", hexs(&s[96..128]))],
                );
            }
        } else if op == OPCODE_WRITE {
            if s.len() > 32 {
                // Value + MAC. Both texts list all bytes, as upstream does.
                self.putz(ctx, s[0].ss, s[s.len() - 31].es, 5, vec![format!("Value: {}", hexs(s))]);
                self.putz(
                    ctx,
                    s[s.len() - 32].ss,
                    s[s.len() - 1].es,
                    5,
                    vec![format!("MAC: {}", hexs(s))],
                );
            } else {
                // Just value.
                self.putz(ctx, s[0].ss, s[s.len() - 1].es, 5, vec![format!("Value: {}", hexs(s))]);
            }
        } else {
            self.putz(ctx, s[0].ss, s[s.len() - 1].es, 5, vec![format!("Data: {}", hexs(s))]);
        }
    }

    fn put_crc(&self, ctx: &mut dyn Context, s: &[Byte]) {
        self.puty(ctx, s, 6, vec![format!("CRC: {:02X} {:02X}", s[0].value, s[1].value)]);
    }

    fn put_status(&self, ctx: &mut dyn Context, ss: u64, es: u64, status: i64) -> Result<()> {
        let name = lookup(&STATUS, status)?;
        self.putz(ctx, ss, es, 7, vec![format!("Status: {name}")]);
        Ok(())
    }

    fn put_warning(&self, ctx: &mut dyn Context, ss: u64, es: u64, msg: &str) {
        self.putz(ctx, ss, es, 8, vec![format!("Warning: {msg}")]);
    }

    fn output_tx_bytes(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let b = self.bytes.clone();
        if b.is_empty() {
            // Ignore wakeup.
            return Ok(());
        }
        self.waddr = b[0].value;
        self.put_waddr(ctx, b[0])?;
        if self.waddr == WORD_ADDR_COMMAND {
            let count = b[1].value;
            self.put_count(ctx, b[1]);
            if b.len() as i64 - 1 != count {
                self.put_warning(
                    ctx,
                    b[0].ss,
                    b[b.len() - 1].es,
                    &format!("Invalid frame length: Got {}, expecting {} ", b.len() - 1, count),
                );
                return Ok(());
            }
            self.opcode = b[2].value;
            self.put_opcode(ctx, b[2])?;
            self.put_param1(ctx, b[3])?;
            self.put_param2(ctx, &[b[4], b[5]]);
            // Python's `b[6:-2]` is empty rather than an error when the
            // frame is shorter than eight bytes.
            let data_end = b.len().saturating_sub(2);
            let data = if 6 <= data_end { &b[6..data_end] } else { &[] };
            self.put_data(ctx, data);
            self.put_crc(ctx, &[b[b.len() - 2], b[b.len() - 1]]);
        }
        Ok(())
    }

    fn output_rx_bytes(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let b = self.bytes.clone();
        let count = b[0].value;
        self.put_count(ctx, b[0]);
        if self.waddr == WORD_ADDR_RESET {
            self.put_data(ctx, &[b[1]]);
            self.put_crc(ctx, &[b[2], b[3]]);
            self.put_status(ctx, b[0].ss, b[b.len() - 1].es, b[1].value)?;
        } else if self.waddr == WORD_ADDR_COMMAND {
            if count == 4 {
                // Status / Error.
                self.put_data(ctx, &[b[1]]);
                self.put_crc(ctx, &[b[2], b[3]]);
                self.put_status(ctx, b[0].ss, b[b.len() - 1].es, b[1].value)?;
            } else {
                // Python's `b[1:-2]` is empty rather than an error when
                // the frame is too short.
                let data_end = b.len().saturating_sub(2);
                let data = if 1 <= data_end { &b[1..data_end] } else { &[] };
                self.put_data(ctx, data);
                self.put_crc(ctx, &[b[b.len() - 2], b[b.len() - 1]]);
            }
        }
        Ok(())
    }

    fn handle(
        &mut self,
        ctx: &mut dyn Context,
        cmd: &str,
        databyte: Option<i64>,
        ss: u64,
        es: u64,
    ) -> Result<()> {
        match self.state {
            State::Idle => {
                // Wait for an I2C START condition.
                if cmd != "START" {
                    return Ok(());
                }
                self.state = State::GetSlaveAddr;
                self.ss_block = ss;
            }
            State::GetSlaveAddr => {
                // Wait for an address read/write operation.
                if cmd == "ADDRESS READ" {
                    self.state = State::ReadRegs;
                } else if cmd == "ADDRESS WRITE" {
                    self.state = State::WriteRegs;
                }
            }
            State::ReadRegs => {
                if cmd == "DATA READ" {
                    self.bytes.push(Byte { ss, es, value: databyte.unwrap_or(0) });
                } else if cmd == "STOP" {
                    self.es_block = es;
                    // Reset the opcode before received data, as this causes
                    // responses to be displayed incorrectly.
                    self.opcode = -1;
                    if !self.bytes.is_empty() {
                        self.output_rx_bytes(ctx)?;
                    }
                    self.waddr = -1;
                    self.bytes.clear();
                    self.state = State::Idle;
                }
            }
            State::WriteRegs => {
                if cmd == "DATA WRITE" {
                    self.bytes.push(Byte { ss, es, value: databyte.unwrap_or(0) });
                } else if cmd == "STOP" {
                    self.es_block = es;
                    self.output_tx_bytes(ctx)?;
                    self.bytes.clear();
                    self.state = State::Idle;
                }
            }
        }
        Ok(())
    }
}

impl Decoder for Atsha204a {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("atsha204a takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        let databyte = match data {
            Item::Int(v) => Some(*v),
            _ => None,
        };
        self.handle(ctx, cmd, databyte, ss, es)
    }
}
