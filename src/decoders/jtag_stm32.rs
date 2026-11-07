// Counterpart in sigrok:
//   libsigrokdecode/decoders/jtag_stm32/pd.py,
//   Copyright (C) 2012-2015 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ARM Cortex-M3 debug port and the decoder's
//   documented registers, ID codes and texts.

//! JTAG / STM32: the STM32F10xxx JTAG TAP and the ARM Cortex-M3 debug port.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// JTAG debug port data registers, keyed by IR[3:0]: name and size.
static IR: [(&str, &str, i32); 5] = [
    ("1111", "BYPASS", 1),  // Bypass register
    ("1110", "IDCODE", 32), // ID code register
    ("1010", "DPACC", 35),  // Debug port access register
    ("1011", "APACC", 35),  // Access port access register
    ("1000", "ABORT", 35),  // Abort register
];

/// Boundary scan data registers, keyed by IR[8:4].
static BS_IR: [(&str, &str, i32); 1] = [("11111", "BYPASS", 1)];

/// `cm3_idcode_ver`: the ID code version field.
static CM3_IDCODE_VER: [(i64, &str); 2] = [(0x3, "JTAG-DP"), (0x2, "SW-DP")];

/// `cm3_idcode_part`: the ID code part number field.
static CM3_IDCODE_PART: [(i64, &str); 2] = [(0xba00, "JTAG-DP"), (0xba10, "SW-DP")];

/// `jedec_id[5]`: the JEDEC manufacturer IDs.
static JEDEC_ID_5: [(i64, &str); 1] = [(0x3b, "ARM Ltd.")];

/// `ack_val`: ACK[2:0] in the DPACC/APACC registers.
static ACK_VAL: [(&str, &str); 2] = [("001", "WAIT"), ("010", "OK/FAULT")];

/// `dp_reg`: the 32 bit debug port registers (addressed via A[3:2]).
static DP_REG: [(&str, &str); 4] =
    [("00", "Reserved"), ("01", "DP CTRL/STAT"), ("10", "DP SELECT"), ("11", "DP RDBUFF")];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "jtag_stm32",
    name: "JTAG / STM32",
    longname: "Joint Test Action Group / ST STM32",
    desc: "ST STM32-specific JTAG protocol.",
    license: "gplv2+",
    inputs: &["jtag"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "item", desc: "Item" },
        AnnClass { id: "field", desc: "Field" },
        AnnClass { id: "command", desc: "Command" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "items", desc: "Items", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1] },
        AnnRow { id: "commands", desc: "Commands", classes: &[2] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[3] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'jtag' PD and decodes JTAG data specific to\n\
the STM32 microcontroller series.\n\
\n\
Details:\n\
https://en.wikipedia.org/wiki/STM32\n\
http://www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/REFERENCE_MANUAL/CD00171190.pdf (e.g. chapter 31.7: \"JTAG debug port\")",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(JtagStm32::default())
}

/// `int('0b' + s, 2)`, which fails on an empty string.
fn bin2int(bits: &str) -> Result<i64> {
    if bits.is_empty() {
        return Err(Error::device("jtag_stm32: empty bitstring"));
    }
    let mut v: i64 = 0;
    for c in bits.bytes() {
        match c {
            b'0' => v = (v << 1) | 0,
            b'1' => v = (v << 1) | 1,
            _ => return Err(Error::device("jtag_stm32: non-binary bitstring")),
        }
    }
    Ok(v)
}

/// Python's `s[start:stop]` for a step of one, with `start` and `stop`
/// allowed to be negative.
fn py_slice(s: &str, start: i64, stop: i64) -> &str {
    let len = s.len() as i64;
    let norm = |i: i64| -> usize {
        let mut i = if i < 0 { i + len } else { i };
        if i < 0 {
            i = 0;
        } else if i > len {
            i = len;
        }
        i as usize
    };
    let a = norm(start);
    let b = norm(stop);
    if a >= b { "" } else { &s[a..b] }
}

/// `'0x%x' % v`.
fn hex(v: i64) -> String {
    format!("0x{v:x}")
}

fn lookup(table: &[(i64, &'static str)], key: i64, default: &'static str) -> &'static str {
    table.iter().find(|(k, _)| *k == key).map_or(default, |(_, v)| *v)
}

/// `decode_device_id_code`: returns `(id_hex, manuf, ver, part)`.
fn decode_device_id_code(bits: &str) -> Result<(String, &'static str, &'static str, &'static str)> {
    let id = bin2int(bits)?;
    let ver = lookup(&CM3_IDCODE_VER, bin2int(py_slice(bits, -32, -28))?, "UNKNOWN");
    let part = lookup(&CM3_IDCODE_PART, bin2int(py_slice(bits, -28, -12))?, "UNKNOWN");
    let jedec_hi = bin2int(py_slice(bits, -12, -8))? + 1;
    let manuf = if jedec_hi == 5 {
        lookup(&JEDEC_ID_5, bin2int(py_slice(bits, -7, -1))?, "UNKNOWN")
    } else {
        "UNKNOWN"
    };
    Ok((hex(id), manuf, ver, part))
}

/// `data_in`: a DPACC/APACC write request on TDI.
fn data_in(instruction: &str, bits: &str) -> Result<String> {
    let data = py_slice(bits, 0, -3);
    let a = py_slice(bits, -3, -1);
    let len = bits.len() as i64;
    let rnw = py_slice(bits, -1, len);
    let data_hex = hex(bin2int(data)?);
    let r = if rnw == "1" { "Read request" } else { "Write request" };
    // `reg = dp_reg[a] if (instruction == 'DPACC') else a`.
    let reg = if instruction == "DPACC" {
        DP_REG.iter().find(|(k, _)| *k == a).map_or("", |(_, v)| *v)
    } else {
        a
    };
    Ok(format!("New transaction: DATA: {data_hex}, A: {reg}, RnW: {r}"))
}

/// `data_out`: a DPACC/APACC previous transaction on TDO.
fn data_out(bits: &str) -> Result<String> {
    let data = py_slice(bits, 0, -3);
    let ack = py_slice(bits, -3, bits.len() as i64);
    let data_hex = hex(bin2int(data)?);
    let ack_meaning = ACK_VAL.iter().find(|(k, _)| *k == ack).map_or("Reserved", |(_, v)| *v);
    Ok(format!("Previous transaction result: DATA: {data_hex}, ACK: {ack_meaning}"))
}

/// The jtag_stm32 decoder.
#[derive(Default)]
pub struct JtagStm32 {
    state: &'static str,
    ss: u64,
    es: u64,
    /// The current bitstring of the item being decoded.
    bits: String,
    /// Bit sample positions, reversed to LSB first.
    samplenums: Vec<[i64; 2]>,
}

impl JtagStm32 {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    /// `putf`: annotate bits `s` through `e`, in reversed sample order.
    fn putf(
        &self,
        ctx: &mut dyn Context,
        s: usize,
        e: usize,
        class: usize,
        texts: Vec<String>,
    ) -> Result<()> {
        let ss = self
            .samplenums
            .get(s)
            .ok_or_else(|| Error::device("jtag_stm32: field start outside bitstring"))?[0];
        let es = self
            .samplenums
            .get(e)
            .ok_or_else(|| Error::device("jtag_stm32: field end outside bitstring"))?[1];
        ctx.put_ann(ss as u64, es as u64, class, texts);
        Ok(())
    }

    fn handle_reg_bypass(&self, ctx: &mut dyn Context, bits: &str) {
        self.putx(ctx, 0, vec![format!("BYPASS: {bits}")]);
    }

    fn handle_reg_idcode(&self, ctx: &mut dyn Context, bits: &str) -> Result<()> {
        let bits = py_slice(bits, 1, bits.len() as i64);
        let (id_hex, manuf, ver, part) = decode_device_id_code(bits)?;
        let cc = hex(bin2int(py_slice(bits, -12, -8))?);
        let ic = hex(bin2int(py_slice(bits, -7, -1))?);
        self.putf(ctx, 0, 0, 1, vec!["Reserved".into(), "Res".into(), "R".into()])?;
        self.putf(
            ctx,
            8,
            11,
            0,
            vec![format!("Continuation code: {cc}"), "CC".into(), "C".into()],
        )?;
        self.putf(ctx, 1, 7, 0, vec![format!("Identity code: {ic}"), "IC".into(), "I".into()])?;
        self.putf(
            ctx,
            1,
            11,
            1,
            vec![format!("Manufacturer: {manuf}"), "Manuf".into(), "M".into()],
        )?;
        self.putf(ctx, 12, 27, 1, vec![format!("Part: {part}"), "Part".into(), "P".into()])?;
        self.putf(ctx, 28, 31, 1, vec![format!("Version: {ver}"), "Version".into(), "V".into()])?;
        self.putf(ctx, 32, 32, 1, vec!["BYPASS (BS TAP)".into(), "BS".into(), "B".into()])?;

        self.putx(ctx, 2, vec![format!("IDCODE: {id_hex} ({manuf}: {ver}/{part})")]);
        Ok(())
    }

    fn handle_reg_dpacc(&self, ctx: &mut dyn Context, cmd: &str, bits: &str) -> Result<()> {
        let bits = py_slice(bits, 1, bits.len() as i64);
        let s = if cmd == "DR TDI" { data_in("DPACC", bits)? } else { data_out(bits)? };
        self.putx(ctx, 2, vec![s]);
        Ok(())
    }

    fn handle_reg_apacc(&self, ctx: &mut dyn Context, cmd: &str, bits: &str) -> Result<()> {
        let bits = py_slice(bits, 1, bits.len() as i64);
        let s = if cmd == "DR TDI" { data_in("APACC", bits)? } else { data_out(bits)? };
        self.putx(ctx, 2, vec![s]);
        Ok(())
    }

    fn handle_reg_abort(&self, ctx: &mut dyn Context, bits: &str) -> Result<()> {
        let bits = py_slice(bits, 1, bits.len() as i64);
        // Bits[31:1]: reserved. Bit[0]: DAPABORT.
        let bit0 = py_slice(bits, 0, 1);
        let a = if bit0 == "1" { "" } else { "No " };
        let s = format!("DAPABORT = {bit0}: {a}DAP abort generated");
        self.putx(ctx, 2, vec![s]);

        // Warn if DAPABORT[31:1] contains non-zero bits.
        if py_slice(bits, 0, -1) != "0".repeat(31) {
            self.putx(ctx, 3, vec!["WARNING: DAPABORT[31:1] reserved!".into()]);
        }
        Ok(())
    }

    fn handle_reg_unknown(&self, ctx: &mut dyn Context, bits: &str) {
        let bits = py_slice(bits, 1, bits.len() as i64);
        self.putx(ctx, 2, vec![format!("Unknown instruction: {bits}")]);
    }

    fn handle_ir(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let bits = self.bits.clone();
        // The STM32F10xxx has two serially connected JTAG TAPs, the
        // boundary scan tap (5 bits) and the Cortex-M3 TAP (4 bits).
        self.state = IR
            .iter()
            .find(|(k, _, _)| *k == py_slice(&bits, 5, 9))
            .map_or("UNKNOWN", |(_, name, _)| *name);
        let bstap_ir = BS_IR
            .iter()
            .find(|(k, _, _)| *k == py_slice(&bits, 0, 5))
            .map_or("UNKNOWN", |(_, n, _)| *n);
        self.putf(ctx, 4, 8, 1, vec![format!("IR (BS TAP): {bstap_ir}")])?;
        self.putf(ctx, 0, 3, 1, vec![format!("IR (M3 TAP): {}", self.state)])?;
        self.putx(ctx, 2, vec![format!("IR: {}", self.state)]);
        Ok(())
    }

    /// The per-state data register handler.
    fn handle_state(&mut self, ctx: &mut dyn Context, cmd: &str) -> Result<()> {
        let bits = self.bits.clone();
        match self.state {
            "BYPASS" => {
                self.handle_reg_bypass(ctx, &bits);
                self.state = "IDLE";
            }
            "IDCODE" => {
                self.handle_reg_idcode(ctx, &bits)?;
                self.state = "IDLE";
            }
            "ABORT" => {
                self.handle_reg_abort(ctx, &bits)?;
                self.state = "IDLE";
            }
            "UNKNOWN" => {
                self.handle_reg_unknown(ctx, &bits);
                self.state = "IDLE";
            }
            "DPACC" => {
                self.handle_reg_dpacc(ctx, cmd, &bits)?;
                if cmd == "DR TDO" {
                    self.state = "IDLE";
                }
            }
            "APACC" => {
                self.handle_reg_apacc(ctx, cmd, &bits)?;
                if cmd == "DR TDO" {
                    self.state = "IDLE";
                }
            }
            _ => {}
        }
        Ok(())
    }
}

/// A jtag packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("jtag_stm32: expected a jtag packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("jtag_stm32: expected a two element jtag packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("jtag_stm32: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// A bit transfer is `[<bitstring>, <samplenums>]`.
fn split_bitstring(item: &Item) -> Result<(String, Vec<[i64; 2]>)> {
    let Item::List(parts) = item else {
        return Err(Error::device("jtag_stm32: expected a bit transfer"));
    };
    if parts.len() != 2 {
        return Err(Error::device("jtag_stm32: expected a two element bit transfer"));
    }
    let Item::Str(bits) = &parts[0] else {
        return Err(Error::device("jtag_stm32: bitstring is not a string"));
    };
    let Item::List(nums) = &parts[1] else {
        return Err(Error::device("jtag_stm32: sample numbers are not a list"));
    };
    let mut positions = Vec::with_capacity(nums.len());
    for n in nums {
        let Item::List(pair) = n else {
            return Err(Error::device("jtag_stm32: malformed sample number pair"));
        };
        if pair.len() != 2 {
            return Err(Error::device("jtag_stm32: malformed sample number pair"));
        }
        let (Item::Int(ss), Item::Int(es)) = (&pair[0], &pair[1]) else {
            return Err(Error::device("jtag_stm32: sample number is not an integer"));
        };
        positions.push([*ss, *es]);
    }
    Ok((bits.clone(), positions))
}

impl Decoder for JtagStm32 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.state = "IDLE";
        self.bits.clear();
        self.samplenums.clear();
        self.ss = 0;
        self.es = 0;
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("jtag_stm32 is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, val) = split_packet(item)?;
        self.ss = ss;
        self.es = es;

        if cmd != "NEW STATE" {
            // The right-most char in the 'val' bitstring is the LSB.
            let (bits, mut positions) = split_bitstring(val)?;
            positions.reverse();
            self.bits = bits;
            self.samplenums = positions;
        }

        if cmd == "IR TDI" {
            self.handle_ir(ctx)?;
        }

        // State machine.
        match self.state {
            "BYPASS" => {
                // Here we're interested in incoming bits (TDI).
                if cmd != "DR TDI" {
                    return Ok(());
                }
                self.handle_state(ctx, cmd)?;
            }
            "IDCODE" | "ABORT" | "UNKNOWN" => {
                // Here we're interested in outgoing bits (TDO).
                if cmd != "DR TDO" {
                    return Ok(());
                }
                self.handle_state(ctx, cmd)?;
            }
            "DPACC" | "APACC" => {
                // Here we're interested in incoming and outgoing bits.
                if cmd != "DR TDI" && cmd != "DR TDO" {
                    return Ok(());
                }
                self.handle_state(ctx, cmd)?;
            }
            _ => {}
        }
        Ok(())
    }
}
