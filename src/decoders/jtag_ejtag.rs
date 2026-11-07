// Counterpart in sigrok:
//   libsigrokdecode/decoders/jtag_ejtag/pd.py,
//   Copyright (C) 2018 Vladislav Ivanov <vlad.ivanov@lab-systems.ru>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the MIPS EJTAG instruction set and the decoder's
//   documented registers, control fields and texts.

//! JTAG / EJTAG: the MIPS EJTAG instruction set, registers and PrAcc.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The EJTAG instruction codes (`Instruction` in pd.py).
const INSN_IDCODE: i64 = 0x01;
const INSN_IMPCODE: i64 = 0x03;
const INSN_ADDRESS: i64 = 0x08;
const INSN_DATA: i64 = 0x09;
const INSN_CONTROL: i64 = 0x0a;
const INSN_FASTDATA: i64 = 0x0e;

/// `ControlReg.PRACC` and `ControlReg.PRNW`.
const CONTROL_PRACC: i64 = 1 << 18;
const CONTROL_PRNW: i64 = 1 << 19;

/// `ejtag_insn`: code, short name, long description. A code that is not
/// listed here is displayed as its hex value.
static EJTAG_INSN: [(i64, &str, &str); 21] = [
    (0x00, "Free", "Boundary scan"),
    (0x01, "IDCODE", "Select Device Identification (ID) register"),
    (0x02, "Free", "Boundary scan"),
    (0x03, "IMPCODE", "Select Implementation register"),
    (0x08, "ADDRESS", "Select Address register"),
    (0x09, "DATA", "Select Data register"),
    (0x0a, "CONTROL", "Select EJTAG Control register"),
    (0x0b, "ALL", "Select the Address, Data and EJTAG Control registers"),
    (0x0c, "EJTAGBOOT", "Fetch code from the debug exception vector after reset"),
    (0x0d, "NORMALBOOT", "Execute the reset handler after reset"),
    (0x0e, "FASTDATA", "Select the Data and Fastdata registers"),
    (0x0f, "Reserved", "Reserved"),
    (0x10, "TCBCONTROLA", "Select the control register TCBTraceControl"),
    (0x11, "TCBCONTROLB", "Selects trace control block register B"),
    (0x12, "TCBDATA", "Access the registers specified by TCBCONTROLB"),
    (0x13, "TCBCONTROLC", "Select trace control block register C"),
    (0x14, "PCSAMPLE", "Select the PCsample register"),
    (0x15, "TCBCONTROLD", "Select trace control block register D"),
    (0x16, "TCBCONTROLE", "Select trace control block register E"),
    (0x17, "FDC", "Select Fast Debug Channel"),
    (0x1c, "Free", "Boundary scan"),
];

/// `ejtag_reg`, indexed by `State`.
static EJTAG_REG: [&str; 9] = [
    "RESET",
    "DEVICE_ID",
    "IMPLEMENTATION",
    "DATA",
    "ADDRESS",
    "CONTROL",
    "FASTDATA",
    "PC_SAMPLE",
    "BYPASS",
];

/// The states (`State` in pd.py), numbered as the annotation classes.
///
/// PCSAMPLE and BYPASS have a register name and an annotation class, but
/// `ejtag_state_map` has no instruction that selects them; they are kept
/// so the class indices match the Python decoder exactly.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Reset = 0,
    DeviceId = 1,
    Implementation = 2,
    Data = 3,
    Address = 4,
    Control = 5,
    Fastdata = 6,
    PcSample = 7,
    Bypass = 8,
}

impl State {
    fn index(self) -> usize {
        self as usize
    }
}

/// One EJTAG control register field.
struct Field {
    /// The first (high) bit number.
    high: i64,
    /// The last (low) bit number.
    low: i64,
    /// The field's short name.
    name: &'static str,
    /// Descriptions for a read value.
    read: &'static [&'static str],
    /// Descriptions for a written value; `None` when the field has no
    /// write descriptions, which `parse_control_reg` skips.
    write: Option<&'static [&'static str]>,
}

/// `ejtag_control_reg`, in table order.
static CONTROL_FIELDS: [Field; 14] = [
    Field {
        high: 31,
        low: 31,
        name: "Rocc",
        read: &["No reset ocurred", "Reset ocurred"],
        write: Some(&["Acknowledge reset", "No effect"]),
    },
    Field {
        high: 30,
        low: 29,
        name: "Psz",
        read: &["Access: byte", "Access: halfword", "Access: word", "Access: triple"],
        write: None,
    },
    Field { high: 23, low: 23, name: "VPED", read: &["VPE disabled", "VPE enabled"], write: None },
    Field {
        high: 22,
        low: 22,
        name: "Doze",
        read: &["Processor is not in low-power mode", "Processor is in low-power mode"],
        write: None,
    },
    Field {
        high: 21,
        low: 21,
        name: "Halt",
        read: &["Internal system bus clock is running", "Internal system bus clock is stopped"],
        write: None,
    },
    Field {
        high: 20,
        low: 20,
        name: "Per Rst",
        read: &["No peripheral reset applied", "Peripheral reset applied"],
        write: Some(&["Deassert peripheral reset", "Assert peripheral reset"]),
    },
    Field {
        high: 19,
        low: 19,
        name: "PRn W",
        read: &["Read processor access", "Write processor access"],
        write: None,
    },
    Field {
        high: 18,
        low: 18,
        name: "Pr Acc",
        read: &["No pending processor access", "Pending processor access"],
        write: Some(&["Finish processor access", "Don't finish processor access"]),
    },
    Field {
        high: 16,
        low: 16,
        name: "Pr Rst",
        read: &["No processor reset applied", "Processor reset applied"],
        write: Some(&["Deassert processor reset", "Assert system reset"]),
    },
    Field {
        high: 15,
        low: 15,
        name: "Prob En",
        read: &["Probe will not serve processor accesses", "Probe will service processor accesses"],
        write: None,
    },
    Field {
        high: 14,
        low: 14,
        name: "Prob Trap",
        read: &["Default location", "DMSEG fetch"],
        write: Some(&["Set to default location", "Set to DMSEG fetch"]),
    },
    Field {
        high: 13,
        low: 13,
        name: "ISA On Debug",
        read: &["MIPS32/MIPS64 ISA", "microMIPS ISA"],
        write: Some(&["Set to MIPS32/MIPS64 ISA", "Set to microMIPS ISA"]),
    },
    Field {
        high: 12,
        low: 12,
        name: "EJTAG Brk",
        read: &["No pending debug interrupt", "Pending debug interrupt"],
        write: Some(&["No effect", "Request debug interrupt"]),
    },
    Field {
        high: 3,
        low: 3,
        name: "DM",
        read: &["Not in debug mode", "In debug mode"],
        write: None,
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "jtag_ejtag",
    name: "JTAG / EJTAG",
    longname: "Joint Test Action Group / EJTAG (MIPS)",
    desc: "MIPS EJTAG protocol.",
    license: "gplv2+",
    inputs: &["jtag"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "instruction", desc: "Instruction" },
        AnnClass { id: "reset", desc: "RESET" },
        AnnClass { id: "device_id", desc: "DEVICE_ID" },
        AnnClass { id: "implementation", desc: "IMPLEMENTATION" },
        AnnClass { id: "data", desc: "DATA" },
        AnnClass { id: "address", desc: "ADDRESS" },
        AnnClass { id: "control", desc: "CONTROL" },
        AnnClass { id: "fastdata", desc: "FASTDATA" },
        AnnClass { id: "pc_sample", desc: "PC_SAMPLE" },
        AnnClass { id: "bypass", desc: "BYPASS" },
        AnnClass { id: "control_field_in", desc: "Control field in" },
        AnnClass { id: "control_field_out", desc: "Control field out" },
        AnnClass { id: "pracc", desc: "PrAcc" },
    ],
    annotation_rows: &[
        AnnRow { id: "instructions", desc: "Instructions", classes: &[0] },
        AnnRow { id: "control_fields_in", desc: "Control fields in", classes: &[10] },
        AnnRow { id: "control_fields_out", desc: "Control fields out", classes: &[11] },
        AnnRow { id: "regs", desc: "Registers", classes: &[1, 2, 3, 4, 5, 6, 7, 8, 9] },
        AnnRow { id: "pracc", desc: "PrAcc", classes: &[12] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'jtag' PD and decodes JTAG data specific\n\
to the MIPS EJTAG protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(JtagEjtap::default())
}

/// `common.srdhelper.bin2int`: `int('0b' + s, 2)`, which fails on an
/// empty string.
fn bin2int(bits: &str) -> Result<i64> {
    if bits.is_empty() {
        return Err(Error::device("jtag_ejtag: empty bitstring"));
    }
    let mut v: i64 = 0;
    for c in bits.bytes() {
        match c {
            b'0' => v = (v << 1) | 0,
            b'1' => v = (v << 1) | 1,
            _ => return Err(Error::device("jtag_ejtag: non-binary bitstring")),
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

/// One data register's bitstring and bit sample positions.
#[derive(Clone, Default)]
struct RegData {
    ss: u64,
    es: u64,
    /// The bitstring, most significant (last shifted) bit first.
    bits: String,
    /// `[ss, es]` for each bit, same order as `bits`.
    positions: Vec<[i64; 2]>,
}

/// `LastData`: the most recent TDI and TDO data register values.
#[derive(Default)]
struct LastData {
    in_: Option<RegData>,
    out: Option<RegData>,
}

/// `PraccState`.
#[derive(Default)]
struct PraccState {
    address_in: Option<i64>,
    address_out: Option<i64>,
    data_in: Option<i64>,
    data_out: Option<i64>,
    ss: u64,
    es: u64,
}

/// The jtag_ejtag decoder.
#[derive(Default)]
pub struct JtagEjtap {
    state: State,
    pracc: PraccState,
    ss: u64,
    es: u64,
    last_data: Option<LastData>,
}

impl Default for State {
    fn default() -> Self {
        State::Reset
    }
}

impl JtagEjtap {
    fn put_at(&self, ctx: &mut dyn Context, ss: u64, es: u64, class: usize, texts: Vec<String>) {
        ctx.put_ann(ss, es, class, texts);
    }

    fn put_current(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    /// `select_reg`: the EJTAG state selected by an instruction.
    fn select_reg(&mut self, ir_value: i64) {
        self.state = match ir_value {
            INSN_IDCODE => State::DeviceId,
            INSN_IMPCODE => State::Implementation,
            INSN_DATA => State::Data,
            INSN_ADDRESS => State::Address,
            INSN_CONTROL => State::Control,
            INSN_FASTDATA => State::Fastdata,
            _ => State::Reset,
        };
    }

    /// `parse_pracc`: report a completed processor access.
    fn parse_pracc(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let last = self.last_data.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no data"))?;
        let in_ = last.in_.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no TDI data"))?;
        let out = last.out.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no TDO data"))?;
        let control_in = bin2int(&in_.bits)?;
        let control_out = bin2int(&out.bits)?;

        // Check if JTAG master acknowledges a pending PrAcc.
        if !((control_in & CONTROL_PRACC) == 0 && (control_out & CONTROL_PRACC) != 0) {
            return Ok(());
        }

        let ss = self.pracc.ss;
        let es = self.pracc.es;
        let pracc_write = (control_out & CONTROL_PRNW) != 0;

        let mut s = String::from("PrAcc: ");
        s += if pracc_write { "Store" } else { "Load/Fetch" };

        if pracc_write {
            if let Some(address) = self.pracc.address_out {
                s += &format!(", A: 0x{address:08X}");
            }
            if let Some(data) = self.pracc.data_out {
                s += &format!(", D: 0x{data:08X}");
            }
        } else {
            if let Some(address) = self.pracc.address_out {
                s += &format!(", A: 0x{address:08X}");
            }
            if let Some(data) = self.pracc.data_in {
                s += &format!(", D: 0x{data:08X}");
            }
        }

        self.pracc = PraccState::default();

        self.put_at(ctx, ss, es, 12, vec![s]);
        Ok(())
    }

    /// `parse_control_reg`: annotate one control register field per row.
    fn parse_control_reg(&self, ctx: &mut dyn Context, ann: usize) -> Result<()> {
        let reg_write = ann == 10;
        let last = self.last_data.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no data"))?;
        let reg = if reg_write {
            last.in_.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no TDI data"))?
        } else {
            last.out.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no TDO data"))?
        };

        for field in &CONTROL_FIELDS {
            let start_bit = 31 - field.low;
            let end_bit = 31 - field.high;

            let value_descriptions = if reg_write {
                match field.write {
                    Some(v) => v,
                    None => continue,
                }
            } else {
                field.read
            };

            let ss = reg
                .positions
                .get(start_bit as usize)
                .ok_or_else(|| Error::device("jtag_ejtag: missing start bit position"))?[0];
            let es = reg
                .positions
                .get(end_bit as usize)
                .ok_or_else(|| Error::device("jtag_ejtag: missing end bit position"))?[1];

            let value_str = py_slice(&reg.bits, end_bit, start_bit + 1);
            let value_index = bin2int(value_str)?;

            let short_desc = format!("{}: {}", field.name, value_str);
            let long_desc = if value_descriptions.len() as i64 > value_index {
                value_descriptions[value_index as usize]
            } else {
                "?"
            };

            self.put_at(ctx, ss as u64, es as u64, ann, vec![long_desc.to_owned(), short_desc]);
        }
        Ok(())
    }

    /// `handle_fastdata`: annotate the 32 bit data word and SPrAcc bit.
    fn handle_fastdata(
        &self,
        ctx: &mut dyn Context,
        bits: &str,
        positions: &[[i64; 2]],
        ann: usize,
    ) -> Result<()> {
        let fastdata_state =
            bits.as_bytes().get(32).copied().ok_or_else(|| {
                Error::device("jtag_ejtag: fastdata bitstring shorter than 33 bits")
            })?;
        let data = bin2int(py_slice(bits, 0, 32))?;

        let fastdata_pos = *positions
            .get(32)
            .ok_or_else(|| Error::device("jtag_ejtag: fastdata position 32 missing"))?;
        let data_pos = [
            positions
                .get(31)
                .ok_or_else(|| Error::device("jtag_ejtag: fastdata position 31 missing"))?[0],
            positions
                .first()
                .ok_or_else(|| Error::device("jtag_ejtag: fastdata position 0 missing"))?[1],
        ];

        let (ss_fastdata, es_fastdata) = (fastdata_pos[0] as u64, fastdata_pos[1] as u64);
        let (ss_data, es_data) = (data_pos[0] as u64, data_pos[1] as u64);

        let spracc_display_data = match (ann, fastdata_state) {
            (10, b'0') => vec!["0", "SPrAcc: 0", "Request completion of Fastdata access"],
            (10, b'1') => vec!["1", "SPrAcc: 1", "No effect"],
            (11, b'0') => vec!["0", "SPrAcc: 0", "Fastdata access failure"],
            (11, b'1') => vec!["1", "SPrAcc: 1", "Successful completion of Fastdata access"],
            (_, b'0' | b'1') => Vec::new(),
            _ => return Err(Error::device("jtag_ejtag: invalid SPrAcc bit")),
        };

        self.put_at(
            ctx,
            ss_fastdata,
            es_fastdata,
            ann,
            spracc_display_data.iter().map(|s| (*s).to_owned()).collect(),
        );
        self.put_at(ctx, ss_data, es_data, ann, vec![format!("0x{data:08X}")]);
        Ok(())
    }

    /// `handle_dr_tdi`.
    fn handle_dr_tdi(
        &mut self,
        ctx: &mut dyn Context,
        bits: &str,
        positions: Vec<[i64; 2]>,
    ) -> Result<()> {
        let value = bin2int(bits)?;
        self.last_data.get_or_insert_with(LastData::default).in_ =
            Some(RegData { ss: self.ss, es: self.es, bits: bits.to_owned(), positions });

        self.pracc.ss = self.ss;
        self.pracc.es = self.es;

        match self.state {
            State::Address => self.pracc.address_in = Some(value),
            State::Data => self.pracc.data_in = Some(value),
            State::Fastdata => {
                let last = self.last_data.as_ref().unwrap();
                let reg = last.in_.as_ref().unwrap();
                self.handle_fastdata(ctx, &reg.bits, &reg.positions, 10)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// `handle_dr_tdo`.
    fn handle_dr_tdo(
        &mut self,
        ctx: &mut dyn Context,
        bits: &str,
        positions: Vec<[i64; 2]>,
    ) -> Result<()> {
        let value = bin2int(bits)?;
        self.last_data.get_or_insert_with(LastData::default).out =
            Some(RegData { ss: self.ss, es: self.es, bits: bits.to_owned(), positions });

        match self.state {
            State::Address => self.pracc.address_out = Some(value),
            State::Data => self.pracc.data_out = Some(value),
            State::Fastdata => {
                let last = self.last_data.as_ref().unwrap();
                let reg = last.out.as_ref().unwrap();
                self.handle_fastdata(ctx, &reg.bits, &reg.positions, 11)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// `handle_ir_tdi`.
    fn handle_ir_tdi(&mut self, ctx: &mut dyn Context, bits: &str) -> Result<()> {
        let code = bin2int(bits)?;
        let hexval = format!("0x{code:02X}");
        if let Some((_, short, long)) = EJTAG_INSN.iter().find(|(k, _, _)| *k == code) {
            let s_short = (*short).to_owned();
            let s_long = format!("{short}: {long} ({hexval})");
            self.put_current(ctx, 0, vec![s_long, s_short]);
        } else {
            let s = format!("IR TDI ({hexval})");
            self.put_current(ctx, 0, vec![hexval, s]);
        }
        self.select_reg(code);
        Ok(())
    }

    /// `handle_new_state`.
    fn handle_new_state(&mut self, ctx: &mut dyn Context, new_state: &str) -> Result<()> {
        if new_state != "UPDATE-DR" || self.last_data.is_none() {
            return Ok(());
        }
        if self.state == State::Reset {
            return Ok(());
        }

        let reg_name = EJTAG_REG[self.state.index()];
        let ann_index = 1 + self.state.index();
        let last = self.last_data.as_ref().unwrap();
        let in_ = last.in_.as_ref().ok_or_else(|| Error::device("jtag_ejtag: no TDI data"))?;
        let (in_ss, in_es) = (in_.ss, in_.es);
        self.put_at(ctx, in_ss, in_es, ann_index, vec![reg_name.to_owned()]);

        if self.state == State::Control {
            let bit_count = in_.positions.len();
            // Check if control register data length is correct.
            if bit_count != 32 {
                self.put_at(ctx, in_ss, in_es, 1, vec!["Error: length != 32".to_owned()]);
                return Ok(());
            }
            self.parse_control_reg(ctx, 10)?;
            self.parse_control_reg(ctx, 11)?;
            self.parse_pracc(ctx)?;
        }
        Ok(())
    }

    /// One jtag item: `[code, value]`.
    fn handle_item(&mut self, ctx: &mut dyn Context, item: &Item) -> Result<()> {
        let (cmd, val) = split_packet(item)?;
        match cmd {
            "IR TDI" => {
                let (bits, _) = split_bitstring(val)?;
                self.handle_ir_tdi(ctx, &bits)
            }
            "DR TDI" => {
                let (bits, positions) = split_bitstring(val)?;
                self.handle_dr_tdi(ctx, &bits, positions)
            }
            "DR TDO" => {
                let (bits, positions) = split_bitstring(val)?;
                self.handle_dr_tdo(ctx, &bits, positions)
            }
            "NEW STATE" => {
                let Item::Str(state) = val else {
                    return Err(Error::device("jtag_ejtag: NEW STATE is not a string"));
                };
                self.handle_new_state(ctx, state)
            }
            _ => Ok(()),
        }
    }
}

/// A jtag packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("jtag_ejtag: expected a jtag packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("jtag_ejtag: expected a two element jtag packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("jtag_ejtag: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// A bit transfer is `[<bitstring>, <samplenums>]`.
fn split_bitstring(item: &Item) -> Result<(String, Vec<[i64; 2]>)> {
    let Item::List(parts) = item else {
        return Err(Error::device("jtag_ejtag: expected a bit transfer"));
    };
    if parts.len() != 2 {
        return Err(Error::device("jtag_ejtag: expected a two element bit transfer"));
    }
    let Item::Str(bits) = &parts[0] else {
        return Err(Error::device("jtag_ejtag: bitstring is not a string"));
    };
    let Item::List(nums) = &parts[1] else {
        return Err(Error::device("jtag_ejtag: sample numbers are not a list"));
    };
    let mut positions = Vec::with_capacity(nums.len());
    for n in nums {
        let Item::List(pair) = n else {
            return Err(Error::device("jtag_ejtag: malformed sample number pair"));
        };
        if pair.len() != 2 {
            return Err(Error::device("jtag_ejtag: malformed sample number pair"));
        }
        let (Item::Int(ss), Item::Int(es)) = (&pair[0], &pair[1]) else {
            return Err(Error::device("jtag_ejtag: sample number is not an integer"));
        };
        positions.push([*ss, *es]);
    }
    Ok((bits.clone(), positions))
}

impl Decoder for JtagEjtap {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.state = State::Reset;
        self.pracc = PraccState::default();
        self.last_data = None;
        self.ss = 0;
        self.es = 0;
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("jtag_ejtag is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        self.ss = ss;
        self.es = es;
        self.handle_item(ctx, item)
    }
}

#[cfg(test)]
mod tests {
    use super::{bin2int, py_slice};

    #[test]
    fn bin_strings_match_python_int() {
        assert_eq!(bin2int("101").unwrap(), 5);
        assert_eq!(bin2int("00000000000000000000000000000000").unwrap(), 0);
        assert!(bin2int("").is_err());
    }

    #[test]
    fn slicing_matches_python() {
        assert_eq!(py_slice("abcdef", 0, 3), "abc");
        assert_eq!(py_slice("abcdef", -3, -1), "de");
        assert_eq!(py_slice("abcdef", 4, 2), "");
        assert_eq!(py_slice("abcdef", 5, 100), "f");
        assert_eq!(py_slice("abcdef", -100, 2), "ab");
    }
}
