// Counterpart in sigrok:
//   libsigrokdecode/decoders/spi/pd.py,
//   Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>,
//   Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig and others (see the libsigrokdecode
//   history).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the SPI bus protocol and the decoder's documented classes and texts.

//! SPI: words clocked on CLK with optional MISO, MOSI and chip select.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const CLK: usize = 0;
const MISO: usize = 1;
const MOSI: usize = 2;
const CS: usize = 3;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "spi",
    name: "SPI",
    longname: "Serial Peripheral Interface",
    desc: "Full-duplex, synchronous, serial bus.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["spi"],
    tags: &["Embedded/industrial"],
    channels: &[ChannelDesc { id: "clk", name: "CLK", desc: "Clock" }],
    optional_channels: &[
        ChannelDesc { id: "miso", name: "MISO", desc: "Master in, slave out" },
        ChannelDesc { id: "mosi", name: "MOSI", desc: "Master out, slave in" },
        ChannelDesc { id: "cs", name: "CS#", desc: "Chip-select" },
    ],
    options: &[
        OptionDesc {
            id: "cs_polarity",
            desc: "CS# polarity",
            default: OptionValue::s("active-low"),
            values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
        },
        OptionDesc {
            id: "cpol",
            desc: "Clock polarity",
            default: OptionValue::i(0),
            values: &[OptionValue::i(0), OptionValue::i(1)],
        },
        OptionDesc {
            id: "cpha",
            desc: "Clock phase",
            default: OptionValue::i(0),
            values: &[OptionValue::i(0), OptionValue::i(1)],
        },
        OptionDesc {
            id: "bitorder",
            desc: "Bit order",
            default: OptionValue::s("msb-first"),
            values: &[OptionValue::s("msb-first"), OptionValue::s("lsb-first")],
        },
        OptionDesc { id: "wordsize", desc: "Word size", default: OptionValue::i(8), values: &[] },
    ],
    annotations: &[
        AnnClass { id: "miso-data", desc: "MISO data" },
        AnnClass { id: "mosi-data", desc: "MOSI data" },
        AnnClass { id: "miso-bits", desc: "MISO bits" },
        AnnClass { id: "mosi-bits", desc: "MOSI bits" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
        AnnClass { id: "miso-transfer", desc: "MISO transfer" },
        AnnClass { id: "mosi-transfer", desc: "MOSI transfer" },
    ],
    annotation_rows: &[
        AnnRow { id: "miso-bits", desc: "MISO bits", classes: &[2] },
        AnnRow { id: "miso-data", desc: "MISO data", classes: &[0] },
        AnnRow { id: "miso-transfer", desc: "MISO transfer", classes: &[5] },
        AnnRow { id: "mosi-bits", desc: "MOSI bits", classes: &[3] },
        AnnRow { id: "mosi-data", desc: "MOSI data", classes: &[1] },
        AnnRow { id: "mosi-transfer", desc: "MOSI transfer", classes: &[6] },
        AnnRow { id: "other", desc: "Other", classes: &[4] },
    ],
    binary: &[BinClass { id: "miso", desc: "MISO" }, BinClass { id: "mosi", desc: "MOSI" }],
    doc: "SPI (Serial Peripheral Interface) is a synchronous serial bus: a \
master clocks words out on MOSI and in on MISO while a chip select is \
asserted. The decoder supports the four clock modes, both bit orders and \
any word size.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Spi::default())
}

use std::fmt::Write as _;

/// A word of any width, big-endian, as libsigrokdecode keeps it in a
/// Python integer: `wordsize` can exceed 64 bits.
#[derive(Clone, Default, PartialEq, Eq, Debug)]
struct WordValue(Vec<u8>);

impl WordValue {
    fn zero(bits: u32) -> Self {
        WordValue(vec![0; (bits as usize).div_ceil(8)])
    }

    fn set_bit(&mut self, bit: u32) {
        let n = self.0.len();
        self.0[n - 1 - (bit / 8) as usize] |= 1 << (bit % 8);
    }

    /// The value as Python's `'%02X' % value` prints it.
    fn hex(&self) -> String {
        let s = self.0.iter().fold(String::new(), |mut acc, b| {
            let _ = write!(acc, "{b:02X}");
            acc
        });
        let t = s.trim_start_matches('0');
        if t.len() >= 2 { t.to_owned() } else { format!("{t:0>2}") }
    }

    fn item(&self) -> Item {
        let significant: Vec<u8> = self.0.iter().copied().skip_while(|&b| b == 0).collect();
        if significant.len() <= 8 {
            let mut v = [0u8; 8];
            v[8 - significant.len()..].copy_from_slice(&significant);
            let v = u64::from_be_bytes(v);
            if let Ok(i) = i64::try_from(v) {
                return Item::Int(i);
            }
        }
        Item::BigUint(significant)
    }
}

/// A decoded word: start, end, value.
#[derive(Clone)]
struct Word {
    ss: u64,
    es: u64,
    val: WordValue,
}

/// The SPI decoder.
#[derive(Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct Spi {
    active_low: bool,
    mode: u8,
    msb_first: bool,
    wordsize: u32,
    samplerate: u64,
    have_miso: bool,
    have_mosi: bool,
    have_cs: bool,
    bitcount: u32,
    misodata: WordValue,
    mosidata: WordValue,
    /// Bits most recent first: value, ss, es.
    misobits: Vec<(u8, u64, u64)>,
    mosibits: Vec<(u8, u64, u64)>,
    misobytes: Vec<Word>,
    mosibytes: Vec<Word>,
    ss_block: u64,
    ss_transfer: Option<u64>,
    cs_was_deasserted: bool,
}

impl Spi {
    fn cs_asserted(&self, cs: bool) -> bool {
        if self.active_low { !cs } else { cs }
    }

    fn bits_item(bits: &[(u8, u64, u64)]) -> Item {
        Item::List(
            bits.iter()
                .map(|&(b, ss, es)| {
                    Item::List(vec![
                        Item::Int(i64::from(b)),
                        Item::Int(ss as i64),
                        Item::Int(es as i64),
                    ])
                })
                .collect(),
        )
    }

    fn putdata(&mut self, ctx: &mut dyn Context) {
        let mut ss = 0;
        let mut es = 0;
        if self.have_miso {
            ss = self.misobits[self.misobits.len() - 1].1;
            es = self.misobits[0].2;
            ctx.put_binary(ss, es, 0, self.misodata.0.clone());
        }
        if self.have_mosi {
            ss = self.mosibits[self.mosibits.len() - 1].1;
            es = self.mosibits[0].2;
            ctx.put_binary(ss, es, 1, self.mosidata.0.clone());
        }
        let miso_val = if self.have_miso { self.misodata.item() } else { Item::None };
        let mosi_val = if self.have_mosi { self.mosidata.item() } else { Item::None };
        let miso_bits = if self.have_miso { Self::bits_item(&self.misobits) } else { Item::None };
        let mosi_bits = if self.have_mosi { Self::bits_item(&self.mosibits) } else { Item::None };
        ctx.put_python(ss, es, Item::List(vec![Item::Str("BITS".into()), mosi_bits, miso_bits]));
        ctx.put_python(ss, es, Item::List(vec![Item::Str("DATA".into()), mosi_val, miso_val]));
        if self.have_miso {
            self.misobytes.push(Word { ss, es, val: self.misodata.clone() });
        }
        if self.have_mosi {
            self.mosibytes.push(Word { ss, es, val: self.mosidata.clone() });
        }
        if self.have_miso {
            for &(b, bss, bes) in &self.misobits {
                ctx.put_ann(bss, bes, 2, vec![b.to_string()]);
            }
        }
        if self.have_mosi {
            for &(b, bss, bes) in &self.mosibits {
                ctx.put_ann(bss, bes, 3, vec![b.to_string()]);
            }
        }
        if self.have_miso {
            ctx.put_ann(ss, es, 0, vec![self.misodata.hex()]);
        }
        if self.have_mosi {
            ctx.put_ann(ss, es, 1, vec![self.mosidata.hex()]);
        }
    }

    fn reset_word(&mut self) {
        self.misodata = WordValue::zero(self.wordsize);
        self.mosidata = WordValue::zero(self.wordsize);
        self.misobits.clear();
        self.mosibits.clear();
        self.bitcount = 0;
    }

    fn handle_bit(&mut self, ctx: &mut dyn Context, miso: bool, mosi: bool, cs: bool) {
        let s = ctx.samplenum();
        if self.bitcount == 0 {
            self.ss_block = s;
            self.cs_was_deasserted = self.have_cs && !self.cs_asserted(cs);
        }
        let ws = self.wordsize;
        let shift = if self.msb_first { ws - 1 - self.bitcount } else { self.bitcount };
        if self.have_miso && miso {
            self.misodata.set_bit(shift);
        }
        if self.have_mosi && mosi {
            self.mosidata.set_bit(shift);
        }
        let mut es = s;
        if self.bitcount > 0 {
            if self.have_miso {
                es += s - self.misobits[0].1;
            } else if self.have_mosi {
                es += s - self.mosibits[0].1;
            }
        }
        if self.have_miso {
            self.misobits.insert(0, (u8::from(miso), s, es));
        }
        if self.have_mosi {
            self.mosibits.insert(0, (u8::from(mosi), s, es));
        }
        if self.bitcount > 0 && self.have_miso {
            self.misobits[1].2 = s;
        }
        if self.bitcount > 0 && self.have_mosi {
            self.mosibits[1].2 = s;
        }
        self.bitcount += 1;
        if self.bitcount != ws {
            return;
        }
        self.putdata(ctx);
        if self.samplerate > 0 {
            // pd.py: elapsed = 1 / float(samplerate); elapsed *= delta;
            // bitrate = int(1 / elapsed * ws). The two reciprocals round
            // differently from delta / samplerate, so keep them.
            let elapsed = 1.0 / self.samplerate as f64;
            let elapsed = elapsed * (s - self.ss_block + 1) as f64;
            let bitrate = (1.0 / elapsed * f64::from(ws)) as i64;
            ctx.put_meta(self.ss_block, s, "Bitrate", Item::Int(bitrate));
        }
        if self.have_cs && self.cs_was_deasserted {
            ctx.put_ann(
                self.ss_block,
                s,
                4,
                vec!["CS# was deasserted during this data word!".into()],
            );
        }
        self.reset_word();
    }

    fn find_clk_edge(
        &mut self,
        ctx: &mut dyn Context,
        pins: (bool, bool, bool, bool),
        first: bool,
        cs_matched: bool,
    ) {
        let (clk, miso, mosi, cs) = pins;
        let s = ctx.samplenum();
        if self.have_cs && (first || cs_matched) {
            let old = if first { Item::None } else { Item::Int(i64::from(!cs)) };
            ctx.put_python(
                s,
                s,
                Item::List(vec![Item::Str("CS-CHANGE".into()), old, Item::Int(i64::from(cs))]),
            );
            if self.cs_asserted(cs) {
                self.ss_transfer = Some(s);
                self.misobytes.clear();
                self.mosibytes.clear();
            } else if let Some(ss) = self.ss_transfer {
                let join = |w: &[Word]| w.iter().map(|x| x.val.hex()).collect::<Vec<_>>().join(" ");
                if self.have_miso {
                    ctx.put_ann(ss, s, 5, vec![join(&self.misobytes)]);
                }
                if self.have_mosi {
                    ctx.put_ann(ss, s, 6, vec![join(&self.mosibytes)]);
                }
                let words = |w: &[Word]| {
                    Item::List(
                        w.iter()
                            .map(|x| {
                                Item::List(vec![
                                    Item::Int(x.ss as i64),
                                    Item::Int(x.es as i64),
                                    x.val.item(),
                                ])
                            })
                            .collect(),
                    )
                };
                ctx.put_python(
                    ss,
                    s,
                    Item::List(vec![
                        Item::Str("TRANSFER".into()),
                        words(&self.mosibytes),
                        words(&self.misobytes),
                    ]),
                );
            }
            self.reset_word();
        }
        if self.have_cs && !self.cs_asserted(cs) {
            return;
        }
        if first || !ctx.matched().first().copied().unwrap_or(false) {
            return;
        }
        // Modes 0 and 3 sample on the rising edge, 1 and 2 on the falling.
        let sample_rising = self.mode == 0 || self.mode == 3;
        if sample_rising != clk {
            return;
        }
        self.handle_bit(ctx, miso, mosi, cs);
    }
}

impl Decoder for Spi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let cpol = config.int("cpol", 0);
        let cpha = config.int("cpha", 0);
        *self = Spi {
            active_low: config.str("cs_polarity", "active-low") == "active-low",
            mode: (cpol * 2 + cpha) as u8,
            msb_first: config.str("bitorder", "msb-first") == "msb-first",
            wordsize: u32::try_from(config.int("wordsize", 8).max(1)).unwrap_or(u32::MAX),
            ..Spi::default()
        };
        self.reset_word();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if !ctx.has_channel(CLK) {
            return Err(Error::arg("Either MISO or MOSI (or both) pins required."));
        }
        self.have_miso = ctx.has_channel(MISO);
        self.have_mosi = ctx.has_channel(MOSI);
        if !self.have_miso && !self.have_mosi {
            return Err(Error::arg("Either MISO or MOSI (or both) pins required."));
        }
        self.have_cs = ctx.has_channel(CS);
        if !self.have_cs {
            ctx.put_python(
                0,
                0,
                Item::List(vec![Item::Str("CS-CHANGE".into()), Item::None, Item::None]),
            );
        }
        let clk_edge: &[Cond] = &[Cond::pin(CLK, CondKind::Edge)];
        let cs_edge: &[Cond] = &[Cond::pin(CS, CondKind::Edge)];
        let conds: Vec<&[Cond]> =
            if self.have_cs { vec![clk_edge, cs_edge] } else { vec![clk_edge] };
        let pins = ctx.wait(&[])?;
        let tuple = |p: &crate::decode::Pins| (p.get(CLK), p.get(MISO), p.get(MOSI), p.get(CS));
        self.find_clk_edge(ctx, tuple(&pins), true, false);
        loop {
            let pins = ctx.wait(&conds)?;
            let cs_matched = self.have_cs && ctx.matched().get(1).copied().unwrap_or(false);
            self.find_clk_edge(ctx, tuple(&pins), false, cs_matched);
        }
    }
}

#[cfg(test)]
mod word_tests {
    use super::WordValue;
    use crate::decode::Item;

    fn word(bits: u32, set: &[u32]) -> WordValue {
        let mut w = WordValue::zero(bits);
        for &b in set {
            w.set_bit(b);
        }
        w
    }

    #[test]
    fn hex_is_pythons_percent_02x() {
        assert_eq!(word(8, &[]).hex(), "00");
        assert_eq!(word(8, &[0]).hex(), "01");
        assert_eq!(word(16, &[0]).hex(), "01");
        assert_eq!(word(16, &[8]).hex(), "100");
        assert_eq!(word(9, &[8, 0]).hex(), "101");
        assert_eq!(word(152, &[151, 0]).hex(), format!("8{}1", "0".repeat(36)));
    }

    #[test]
    fn items_stay_ints_while_they_fit() {
        assert!(matches!(word(8, &[7]).item(), Item::Int(128)));
        assert!(matches!(word(64, &[62]).item(), Item::Int(i) if i == 1 << 62));
        assert!(
            matches!(word(64, &[63]).item(), Item::BigUint(ref b) if b == &[0x80, 0, 0, 0, 0, 0, 0, 0])
        );
        assert!(matches!(word(152, &[100]).item(), Item::BigUint(ref b) if b.len() == 13));
    }
}
