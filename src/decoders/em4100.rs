// Counterpart in sigrok:
//   libsigrokdecode/decoders/em4100/pd.py,
//   Copyright (C) 2015 Benjamin Larsson <benjamin@southpole.se>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Manchester coding, the frame layout and the
//   decoder's documented classes and texts.

//! EM4100: 100-150kHz RFID, Manchester coded 64-bit frames.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_HEADER: usize = 1;
const ANN_VERSION: usize = 2;
const ANN_DATA: usize = 3;
const ANN_ROW_OK: usize = 4;
const ANN_ROW_ERR: usize = 5;
const ANN_COL_OK: usize = 6;
const ANN_COL_ERR: usize = 7;
const ANN_STOPBIT: usize = 8;
const ANN_TAG: usize = 9;

static OPTIONS: [OptionDesc; 3] = [
    OptionDesc {
        id: "polarity",
        desc: "Polarity",
        default: OptionValue::s("active-high"),
        values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
    },
    OptionDesc {
        id: "datarate",
        desc: "Data rate",
        default: OptionValue::i(64),
        values: &[OptionValue::i(64), OptionValue::i(32), OptionValue::i(16)],
    },
    OptionDesc {
        id: "coilfreq",
        desc: "Coil frequency",
        default: OptionValue::i(125_000),
        values: &[],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "em4100",
    name: "EM4100",
    longname: "RFID EM4100",
    desc: "EM4100 100-150kHz RFID protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IC", "RFID"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "header", desc: "Header" },
        AnnClass { id: "version-customer", desc: "Version/customer" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "rowparity-ok", desc: "Row parity OK" },
        AnnClass { id: "rowparity-err", desc: "Row parity error" },
        AnnClass { id: "colparity-ok", desc: "Column parity OK" },
        AnnClass { id: "colparity-err", desc: "Column parity error" },
        AnnClass { id: "stopbit", desc: "Stop bit" },
        AnnClass { id: "tag", desc: "Tag" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2, 3, 4, 5, 6, 7, 8] },
        AnnRow { id: "tags", desc: "Tags", classes: &[9] },
    ],
    binary: &[],
    doc: "EM4100 is a biphase/manchester/PSK based 100-150kHz RFID protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Em4100::new())
}

/// The frame state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Header,
    Payload,
    Trailer,
}

/// The EM4100 decoder.
pub struct Em4100 {
    samplerate: u64,
    oldpin: bool,
    last_samplenum: u64,
    lastlast_samplenum: u64,
    last_edge: u64,
    bit_width: f64,
    halfbit_limit: f64,
    oldpp: u32,
    oldpl: u64,
    oldsamplenum: u64,
    last_bit_pos: u64,
    ss_first: u64,
    first_one: u32,
    state: State,
    data: u32,
    data_bits: usize,
    ss_data: u64,
    data_parity: u32,
    payload_cnt: u32,
    data_col_parity: [u32; 6],
    col_parity: [u32; 6],
    tag: u64,
    all_row_parity_ok: bool,
    col_parity_pos: Vec<(u64, u64)>,
    polarity: bool,
    coilfreq: i64,
    datarate: i64,
}

impl Em4100 {
    fn new() -> Self {
        Em4100 {
            samplerate: 0,
            oldpin: false,
            last_samplenum: 0,
            lastlast_samplenum: 0,
            last_edge: 0,
            bit_width: 0.0,
            halfbit_limit: 0.0,
            oldpp: 0,
            oldpl: 0,
            oldsamplenum: 0,
            last_bit_pos: 0,
            ss_first: 0,
            first_one: 0,
            state: State::Header,
            data: 0,
            data_bits: 0,
            ss_data: 0,
            data_parity: 0,
            payload_cnt: 0,
            data_col_parity: [0; 6],
            col_parity: [0; 6],
            tag: 0,
            all_row_parity_ok: true,
            col_parity_pos: Vec::new(),
            polarity: true,
            coilfreq: 125_000,
            datarate: 64,
        }
    }

    /// A bit value as the decoder's integers.
    fn b(bit: bool) -> u32 {
        u32::from(bit)
    }

    /// Python `int(x)` for the non-negative sample positions used here.
    fn trunc(x: f64) -> u64 {
        x as u64
    }

    #[allow(clippy::too_many_lines)]
    fn putbit(&mut self, ctx: &mut dyn Context, bit: bool, ss: u64, es: u64) {
        ctx.put_ann(ss, es, ANN_BIT, vec![if bit { "1" } else { "0" }.to_owned()]);
        match self.state {
            State::Header => {
                if bit {
                    if self.first_one > 0 {
                        self.first_one += 1;
                    }
                    if self.first_one == 9 {
                        ctx.put_ann(
                            self.ss_first,
                            es,
                            ANN_HEADER,
                            vec!["Header".into(), "Head".into(), "He".into(), "H".into()],
                        );
                        self.first_one = 0;
                        self.state = State::Payload;
                        return;
                    }
                    if self.first_one == 0 {
                        self.first_one = 1;
                        self.ss_first = ss;
                    }
                }
                if !bit {
                    self.first_one = 0;
                }
            }
            State::Payload => {
                self.payload_cnt += 1;
                if self.data_bits == 0 {
                    self.ss_data = ss;
                    self.data = 0;
                    self.data_parity = 0;
                }
                self.data_bits += 1;
                if self.data_bits == 5 {
                    let (s, c) = if self.payload_cnt <= 10 {
                        ("Version/customer", ANN_VERSION)
                    } else {
                        ("Data", ANN_DATA)
                    };
                    ctx.put_ann(
                        self.ss_data,
                        ss,
                        c,
                        vec![format!("{s}: {:X}", self.data), format!("{:X}", self.data)],
                    );
                    let ok = self.data_parity == Self::b(bit);
                    let s = if ok { "OK" } else { "ERROR" };
                    let c = if ok { ANN_ROW_OK } else { ANN_ROW_ERR };
                    if !ok {
                        self.all_row_parity_ok = false;
                    }
                    ctx.put_ann(
                        ss,
                        es,
                        c,
                        vec![
                            format!("Row parity: {s}"),
                            format!("RP: {s}"),
                            "RP".into(),
                            "R".into(),
                        ],
                    );
                    self.tag = (self.tag << 4) | u64::from(self.data);
                    self.data_bits = 0;
                    if self.payload_cnt == 50 {
                        self.state = State::Trailer;
                        self.payload_cnt = 0;
                    }
                }

                self.data_parity ^= Self::b(bit);
                self.data_col_parity[self.data_bits] ^= Self::b(bit);
                self.data = (self.data << 1) | Self::b(bit);
            }
            State::Trailer => {
                self.payload_cnt += 1;
                if self.data_bits == 0 {
                    self.ss_data = ss;
                    self.data = 0;
                    self.data_parity = 0;
                }
                self.data_bits += 1;
                self.col_parity[self.data_bits] = Self::b(bit);
                self.col_parity_pos.push((ss, es));

                if self.data_bits == 5 {
                    ctx.put_ann(
                        ss,
                        es,
                        ANN_STOPBIT,
                        vec!["Stop bit".into(), "SB".into(), "S".into()],
                    );

                    for i in 1..=4 {
                        let ok = self.data_col_parity[i] == self.col_parity[i];
                        let c = if ok { ANN_COL_OK } else { ANN_COL_ERR };
                        let s = if ok { "OK" } else { "ERROR" };
                        let (pss, pes) = self.col_parity_pos[i - 1];
                        ctx.put_ann(
                            pss,
                            pes,
                            c,
                            vec![
                                format!("Column parity {i}: {s}"),
                                format!("CP{i}: {s}"),
                                format!("CP{i}"),
                                "C".into(),
                            ],
                        );
                    }

                    // Emit an annotation for valid-looking tags.
                    let all_col_parity_ok = self.data_col_parity[1..5] == self.col_parity[1..5];
                    if all_col_parity_ok && self.all_row_parity_ok {
                        ctx.put_ann(
                            self.ss_first,
                            es,
                            ANN_TAG,
                            vec![format!("Tag: {:010X}", self.tag), "Tag".into(), "T".into()],
                        );
                    }

                    self.tag = 0;
                    self.data_bits = 0;

                    if self.payload_cnt == 5 {
                        self.state = State::Header;
                        self.payload_cnt = 0;
                        self.data_col_parity = [0; 6];
                        self.col_parity = [0; 6];
                        self.col_parity_pos.clear();
                        self.all_row_parity_ok = true;
                    }
                }
            }
        }
    }

    fn manchester_decode(&mut self, ctx: &mut dyn Context, pl: u64, _pp: u32, _pin: bool) {
        let bit = self.oldpin ^ self.polarity;
        let now = ctx.samplenum();
        if pl as f64 > self.halfbit_limit {
            let es = Self::trunc(now as f64 - pl as f64 / 2.0);
            if self.oldpl as f64 > self.halfbit_limit {
                let ss = Self::trunc(self.oldsamplenum as f64 - self.oldpl as f64 / 2.0);
                self.putbit(ctx, bit, ss, es);
            } else {
                let ss = self.oldsamplenum - self.oldpl;
                self.putbit(ctx, bit, ss, es);
            }
            self.last_bit_pos = Self::trunc(now as f64 - pl as f64 / 2.0);
        } else {
            let es = now;
            if self.oldpl as f64 > self.halfbit_limit {
                let ss = Self::trunc(self.oldsamplenum as f64 - self.oldpl as f64 / 2.0);
                self.putbit(ctx, bit, ss, es);
                self.last_bit_pos = now;
            } else if self.last_bit_pos <= self.oldsamplenum - self.oldpl {
                let ss = self.oldsamplenum - self.oldpl;
                self.putbit(ctx, bit, ss, es);
                self.last_bit_pos = now;
            }
        }
    }
}

impl Decoder for Em4100 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.polarity = config.str("polarity", "active-high") != "active-low";
        self.datarate = config.int("datarate", 64);
        self.coilfreq = config.int("coilfreq", 125_000);
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        // pd.py computes these on every metadata call, inside the
        // SRD_CONF_SAMPLERATE branch of the module-level metadata method.
        self.samplerate = samplerate;
        self.bit_width = (samplerate as f64 / self.coilfreq as f64) * self.datarate as f64;
        self.halfbit_limit = self.bit_width / 2.0 + self.bit_width / 4.0;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }

        // Initialize internal state from the very first sample.
        let pins = ctx.wait(&[])?;
        self.oldpin = pins.get(DATA);
        self.last_samplenum = ctx.samplenum();
        self.lastlast_samplenum = ctx.samplenum();
        self.last_edge = ctx.samplenum();
        self.oldpl = 0;
        self.oldpp = 0;
        self.oldsamplenum = 0;
        self.last_bit_pos = 0;

        let cond = [Cond::pin(DATA, CondKind::Edge)];
        loop {
            // Ignore identical samples, only process edges.
            let pins = ctx.wait(&[&cond])?;
            let pin = pins.get(DATA);
            let pl = ctx.samplenum() - self.oldsamplenum;
            let pp = u32::from(pin);
            self.manchester_decode(ctx, pl, pp, pin);
            self.oldpl = pl;
            self.oldpp = pp;
            self.oldsamplenum = ctx.samplenum();
            self.oldpin = pin;
        }
    }
}
