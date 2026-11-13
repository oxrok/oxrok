// Counterpart in sigrok:
//   libsigrokdecode/decoders/ps2/pd.py,
//   Copyright (C) 2016 Daniel Schulte <trilader@schroedingers-bit.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the PS/2 protocol and the decoder's documented
//   classes and texts.

//! PS/2: 11-bit frames clocked out on the falling clock edge.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const CLK: usize = 0;
const DATA: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_START: usize = 1;
const ANN_STOP: usize = 2;
const ANN_PARITY_OK: usize = 3;
const ANN_PARITY_ERR: usize = 4;
const ANN_WORD: usize = 6;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ps2",
    name: "PS/2",
    longname: "PS/2",
    desc: "PS/2 keyboard/mouse interface.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["PC"],
    channels: &[
        ChannelDesc { id: "clk", name: "Clock", desc: "Clock line" },
        ChannelDesc { id: "data", name: "Data", desc: "Data line" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "start-bit", desc: "Start bit" },
        AnnClass { id: "stop-bit", desc: "Stop bit" },
        AnnClass { id: "parity-ok", desc: "Parity OK bit" },
        AnnClass { id: "parity-err", desc: "Parity error bit" },
        AnnClass { id: "data-bit", desc: "Data bit" },
        AnnClass { id: "word", desc: "Word" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "fields", desc: "Fields", classes: &[1, 2, 3, 4, 5, 6] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ps2::default())
}

/// The Python decoder's `namedtuple('Bit', 'val ss es')`.
#[derive(Clone, Copy)]
struct Bit {
    val: u8,
    ss: u64,
    es: u64,
}

/// The PS/2 decoder.
#[derive(Default)]
pub struct Ps2 {
    bits: Vec<Bit>,
    bitcount: usize,
}

impl Ps2 {
    /// The Python decoder's `handle_bits(datapin)`.
    fn handle_bits(&mut self, ctx: &mut dyn Context, datapin: u8) {
        // "Ignore non start condition bits (useful during keyboard init)."
        if self.bitcount == 0 && datapin == 1 {
            return;
        }

        // "Store individual bits and their start/end samplenumbers."
        let s = ctx.samplenum();
        self.bits.push(Bit { val: datapin, ss: s, es: s });

        // "Fix up end sample numbers of the bits."
        if self.bitcount > 0 {
            self.bits[self.bitcount - 1].es = s;
        }
        // On the 12th falling edge the extra bit carries the stop bit's
        // inferred end; the arithmetic is signed in Python and is only
        // applied to the discarded bit, so it cannot affect annotations.
        if self.bitcount == 11 {
            let bitwidth = self.bits[1].es.wrapping_sub(self.bits[2].es);
            if let Some(last) = self.bits.last_mut() {
                last.es = last.es.wrapping_add(bitwidth);
            }
        }

        // "Find all 11 bits. Start + 8 data + odd parity + stop."
        if self.bitcount < 11 {
            self.bitcount += 1;
            return;
        }

        // "Extract data word."
        let mut word = 0u32;
        for i in 0..8 {
            word |= u32::from(self.bits[i + 1].val) << i;
        }

        // "Calculate parity."
        let parity_ok = (word.count_ones() + u32::from(self.bits[9].val)) % 2 == 1;

        // "Emit annotations."
        for b in &self.bits[..11] {
            ctx.put_ann(b.ss, b.es, ANN_BIT, vec![b.val.to_string()]);
        }
        let start = self.bits[0];
        ctx.put_ann(
            start.ss,
            start.es,
            ANN_START,
            vec!["Start bit".into(), "Start".into(), "S".into()],
        );
        ctx.put_ann(
            self.bits[1].ss,
            self.bits[8].es,
            ANN_WORD,
            vec![format!("Data: {word:02x}"), format!("D: {word:02x}"), format!("{word:02x}")],
        );
        let parity = self.bits[9];
        if parity_ok {
            ctx.put_ann(
                parity.ss,
                parity.es,
                ANN_PARITY_OK,
                vec!["Parity OK".into(), "Par OK".into(), "P".into()],
            );
        } else {
            ctx.put_ann(
                parity.ss,
                parity.es,
                ANN_PARITY_ERR,
                vec!["Parity error".into(), "Par err".into(), "PE".into()],
            );
        }
        let stop = self.bits[10];
        ctx.put_ann(
            stop.ss,
            stop.es,
            ANN_STOP,
            vec!["Stop bit".into(), "Stop".into(), "St".into(), "T".into()],
        );

        self.bits.clear();
        self.bitcount = 0;
    }
}

impl Decoder for Ps2 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.bits.clear();
        self.bitcount = 0;
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let group = [Cond::pin(CLK, CondKind::Falling)];
        loop {
            // "Sample data bits on falling clock edge."
            let pins = ctx.wait(&[&group])?;
            let data = u8::from(pins.get(DATA));
            self.handle_bits(ctx, data);
        }
    }
}
