// Counterpart in sigrok:
//   libsigrokdecode/decoders/ook_vis/pd.py,
//   Copyright (C) 2018 Steve R <steversig@virginmedia.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the on-off keying visualisation's rules and
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The whole OOK packet is cached before it is displayed, so a
//     reference packet can be the current one: `refsample=1` compares
//     each packet with itself, and the first packet backfills nothing.
//   - The preamble is found by watching for a repeated or alternating
//     bit, then `preamble_end += sync_offset - 1`; a negative result is
//     a Python negative list index and a negative string slice, which
//     this port reproduces.
//   - A field containing 'E' is shown as is (or reversed when the
//     display is a `rev` one) and is not converted from binary.
//   - The byte/nibble and BCD/hex choices are substring tests on the
//     option text, as in Python, so `Byte - BCD rev` reverses the bits
//     before the BCD conversion.
#![allow(clippy::assigning_clones, clippy::doc_markdown)]

//! OOK visualisation: nibble and byte fields, BCD and a reference trace.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
// The bit class (0), ref field class (3) and ref L2 class (5) are
// declared but never emitted by the Python decoder.
const ANN_REF: usize = 1;
const ANN_FIELD: usize = 2;
const ANN_LEVEL2: usize = 4;

static OPTIONS: [OptionDesc; 4] = [
    OptionDesc {
        id: "displayas",
        desc: "Display as",
        default: OptionValue::s("Nibble - Hex"),
        values: &[
            OptionValue::s("Byte - Hex"),
            OptionValue::s("Byte - Hex rev"),
            OptionValue::s("Byte - BCD"),
            OptionValue::s("Byte - BCD rev"),
            OptionValue::s("Nibble - Hex"),
            OptionValue::s("Nibble - Hex rev"),
            OptionValue::s("Nibble - BCD"),
            OptionValue::s("Nibble - BCD rev"),
        ],
    },
    OptionDesc {
        id: "synclen",
        desc: "Sync length",
        default: OptionValue::s("4"),
        values: &[
            OptionValue::s("0"),
            OptionValue::s("1"),
            OptionValue::s("2"),
            OptionValue::s("3"),
            OptionValue::s("4"),
            OptionValue::s("5"),
            OptionValue::s("6"),
            OptionValue::s("7"),
            OptionValue::s("8"),
            OptionValue::s("9"),
            OptionValue::s("10"),
        ],
    },
    OptionDesc {
        id: "syncoffset",
        desc: "Sync offset",
        default: OptionValue::s("0"),
        values: &[
            OptionValue::s("-4"),
            OptionValue::s("-3"),
            OptionValue::s("-2"),
            OptionValue::s("-1"),
            OptionValue::s("0"),
            OptionValue::s("1"),
            OptionValue::s("2"),
            OptionValue::s("3"),
            OptionValue::s("4"),
        ],
    },
    OptionDesc {
        id: "refsample",
        desc: "Compare",
        default: OptionValue::s("off"),
        values: &[
            OptionValue::s("off"),
            OptionValue::s("show numbers"),
            OptionValue::s("1"),
            OptionValue::s("2"),
            OptionValue::s("3"),
            OptionValue::s("4"),
            OptionValue::s("5"),
            OptionValue::s("6"),
            OptionValue::s("7"),
            OptionValue::s("8"),
            OptionValue::s("9"),
            OptionValue::s("10"),
            OptionValue::s("11"),
            OptionValue::s("12"),
            OptionValue::s("13"),
            OptionValue::s("14"),
            OptionValue::s("15"),
            OptionValue::s("16"),
            OptionValue::s("17"),
            OptionValue::s("18"),
            OptionValue::s("19"),
            OptionValue::s("20"),
            OptionValue::s("21"),
            OptionValue::s("22"),
            OptionValue::s("23"),
            OptionValue::s("24"),
            OptionValue::s("25"),
            OptionValue::s("26"),
            OptionValue::s("27"),
            OptionValue::s("28"),
            OptionValue::s("29"),
            OptionValue::s("30"),
        ],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ook_vis",
    name: "OOK visualisation",
    longname: "On-off keying visualisation",
    desc: "OOK visualisation in various formats.",
    license: "gplv2+",
    inputs: &["ook"],
    outputs: &["ook"],
    tags: &["Encoding"],
    channels: &[],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "ref", desc: "Reference" },
        AnnClass { id: "field", desc: "Field" },
        AnnClass { id: "ref_field", desc: "Ref field" },
        AnnClass { id: "level2", desc: "L2" },
        AnnClass { id: "ref_level2", desc: "Ref L2" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "compare", desc: "Compare", classes: &[1] },
        AnnRow { id: "fields", desc: "Fields", classes: &[2] },
        AnnRow { id: "ref_fields", desc: "Ref fields", classes: &[3] },
        AnnRow { id: "level2", desc: "L2", classes: &[4] },
        AnnRow { id: "ref_level2", desc: "Ref L2", classes: &[5] },
    ],
    binary: &[],
    // The package's __init__.py docstring, which the Python host reads.
    doc: "This decoder stacks on top of the 'ook' PD and visualizes protocol details\n\
          in various ways.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(OokVis::default())
}

/// One decoded OOK bit: `[ss, es, bit]`.
#[derive(Clone, Copy)]
struct OokBit {
    ss: u64,
    es: u64,
    state: char,
}

/// `bcd2int` from common/srdhelper.
fn bcd2int(b: i64) -> i64 {
    (b & 0x0f) + ((b >> 4) * 10)
}

/// A Python list index: negative counts from the end.
fn py_index(i: i64, len: usize) -> Option<usize> {
    let len = len as i64;
    let j = if i < 0 { len + i } else { i };
    (0..len).contains(&j).then_some(j as usize)
}

/// Python's `s[start:start + n]`, including its negative index rules.
fn py_slice(s: &str, start: i64, n: i64) -> String {
    let len = s.len() as i64;
    let norm = |x: i64| if x < 0 { (len + x).max(0) } else { x.min(len) };
    let a = norm(start);
    let b = norm(start + n);
    if b <= a { String::new() } else { s[a as usize..b as usize].to_owned() }
}

/// Python's `s[start:]`.
fn py_slice_from(s: &str, start: i64) -> String {
    let len = s.len() as i64;
    let a = if start < 0 { (len + start).max(0) } else { start.min(len) };
    s[a as usize..].to_owned()
}

/// The ook_vis decoder.
#[derive(Default)]
pub struct OokVis {
    decoded: Vec<OokBit>,
    ookstring: String,
    ookcache: Vec<Vec<OokBit>>,
    trace_num: usize,
    decode_pos: i64,
    ss: u64,
    es: u64,
    displayas: String,
    sync_length: i64,
    sync_offset: i64,
    reference: String,
}

impl OokVis {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    /// pd.py `put_field()`.
    fn put_field(&mut self, ctx: &mut dyn Context, numbits: usize, line: usize) -> Result<()> {
        let mut param = py_slice(&self.ookstring, self.decode_pos, numbits as i64);
        if self.displayas.contains("rev") {
            param = param.chars().rev().collect();
        }
        if !param.contains('E') {
            let intval = i64::from_str_radix(&param, 2)
                .map_err(|_| Error::device("ook_vis: field is not binary"))?;
            if self.displayas.contains("Hex") {
                param = format!("{intval:x}");
            } else if self.displayas.contains("BCD") {
                param = bcd2int(intval).to_string();
            }
        }
        self.putx(ctx, line, vec![param]);
        self.decode_pos += numbits as i64;
        Ok(())
    }

    /// pd.py `display_level2()`.
    fn display_level2(&mut self, ctx: &mut dyn Context, bits: usize, line: usize) -> Result<()> {
        self.decode_pos = 0;
        if self.decoded.len() > 1 {
            let mut preamble_end = self.decoded.len() as i64 + 1;
            let char_first = self.decoded[0].state;
            let char_second = self.decoded[1].state;
            let preamble_1111 = char_first == char_second;
            let mut char_last = if preamble_1111 { char_first } else { char_second };
            for (i, bit) in self.decoded.iter().enumerate() {
                if preamble_1111 {
                    if bit.state != char_last {
                        preamble_end = i as i64;
                        break;
                    }
                    char_last = bit.state;
                } else if bit.state != char_last {
                    char_last = bit.state;
                } else {
                    preamble_end = i as i64;
                    break;
                }
            }

            if self.decoded.len() as i64 >= preamble_end {
                preamble_end += self.sync_offset - 1;
                let Some(pe) = py_index(preamble_end, self.decoded.len()) else {
                    return Err(Error::device("ook_vis: preamble end out of range"));
                };
                self.ss = self.decoded[0].ss;
                self.es = self.decoded[pe].es;
                self.putx(ctx, line, vec!["Preamble".to_owned(), "Pre".to_owned(), "P".to_owned()]);
                self.decode_pos += preamble_end;

                let sync_end = self.decode_pos + self.sync_length;
                if self.decoded.len() as i64 > sync_end {
                    let Some(se) = py_index(sync_end, self.decoded.len()) else {
                        return Err(Error::device("ook_vis: sync end out of range"));
                    };
                    self.ss = self.es;
                    self.es = self.decoded[se].es;
                    self.putx(ctx, line, vec!["Sync".to_owned(), "Syn".to_owned(), "S".to_owned()]);
                    self.decode_pos += self.sync_length + 1;
                }

                let ookstring = py_slice_from(&self.ookstring, self.decode_pos);
                let rem_nibbles = ookstring.len() / bits;
                for _ in 0..rem_nibbles {
                    let Some(a) = py_index(self.decode_pos, self.decoded.len()) else {
                        return Err(Error::device("ook_vis: field start out of range"));
                    };
                    let Some(b) = py_index(self.decode_pos + bits as i64 - 1, self.decoded.len())
                    else {
                        return Err(Error::device("ook_vis: field end out of range"));
                    };
                    self.ss = self.decoded[a].ss;
                    self.es = self.decoded[b].es;
                    self.put_field(ctx, bits, line)?;
                }
            }
        }
        Ok(())
    }

    /// pd.py `display_ref()`.
    fn display_ref(&mut self, ctx: &mut dyn Context, t_num: usize, reference: usize) {
        let mut display_len = self.ookcache[reference].len();
        if self.ookcache[t_num].len() < self.ookcache[reference].len() {
            display_len = self.ookcache[t_num].len();
        }
        for i in 0..display_len {
            self.ss = self.ookcache[t_num][i].ss;
            self.es = self.ookcache[t_num][i].es;
            let text = self.ookcache[reference][i].state.to_string();
            self.putx(ctx, ANN_REF, vec![text]);
        }
    }

    /// pd.py `display_all()`.
    fn display_all(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.decode_pos = 0;
        for bit in &self.decoded {
            self.ookstring.push(bit.state);
        }
        let bits = if self.displayas.contains("Nibble") { 4 } else { 8 };
        let rem_nibbles = self.ookstring.len() / bits;
        for _ in 0..rem_nibbles {
            let Some(a) = py_index(self.decode_pos, self.decoded.len()) else {
                return Err(Error::device("ook_vis: bit start out of range"));
            };
            let Some(b) = py_index(self.decode_pos + bits as i64 - 1, self.decoded.len()) else {
                return Err(Error::device("ook_vis: bit end out of range"));
            };
            self.ss = self.decoded[a].ss;
            self.es = self.decoded[b].es;
            self.put_field(ctx, bits, ANN_FIELD)?;
        }

        self.display_level2(ctx, bits, ANN_LEVEL2)?;

        if self.reference != "off"
            && self.reference != "show numbers"
            && let Ok(n) = self.reference.parse::<usize>()
            && self.ookcache.len() >= n
        {
            let reference = n - 1;
            self.display_ref(ctx, self.trace_num, reference);
            if self.ookcache.len() == n {
                for i in 0..reference {
                    self.display_ref(ctx, i, reference);
                }
            }
        } else if self.reference == "show numbers" {
            let Some(trace) = self.ookcache.get(self.trace_num) else {
                return Err(Error::device("ook_vis: no cached trace"));
            };
            let Some(first) = trace.first() else {
                return Err(Error::device("ook_vis: empty cached trace"));
            };
            self.ss = first.ss;
            self.es = trace[trace.len() - 1].es;
            self.putx(ctx, ANN_REF, vec![(self.trace_num + 1).to_string()]);
        }
        Ok(())
    }
}

impl Decoder for OokVis {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.displayas = config.str("displayas", "Nibble - Hex").to_owned();
        self.sync_length = config.str("synclen", "4").parse().unwrap_or(4);
        self.sync_offset = config.str("syncoffset", "0").parse().unwrap_or(0);
        self.reference = config.str("refsample", "off").to_owned();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ook_vis takes an ook decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(bits) = item else {
            return Err(Error::device("ook_vis: expected a list of OOK bits"));
        };
        let mut decoded = Vec::with_capacity(bits.len());
        for bit in bits {
            let Item::List(parts) = bit else {
                return Err(Error::device("ook_vis: OOK bit is not a list"));
            };
            if parts.len() != 3 {
                return Err(Error::device("ook_vis: OOK bit is not [ss, es, bit]"));
            }
            let (Item::Int(ss), Item::Int(es), Item::Str(state)) =
                (&parts[0], &parts[1], &parts[2])
            else {
                return Err(Error::device("ook_vis: OOK bit fields are not [int, int, str]"));
            };
            let state = state.chars().next().unwrap_or('E');
            decoded.push(OokBit { ss: *ss as u64, es: *es as u64, state });
        }

        self.decoded = decoded;
        self.ookcache.push(self.decoded.clone());
        self.display_all(ctx)?;
        self.ookstring.clear();
        self.trace_num += 1;
        self.ss = ss;
        self.es = es;
        ctx.put_python(ss, es, item.clone());
        Ok(())
    }
}
