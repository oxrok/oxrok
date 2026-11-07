// Counterpart in sigrok:
//   libsigrokdecode/decoders/aud/pd.py,
//   Copyright (C) 2016 fenugrec <fenugrec users.sourceforge.net>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the AUD protocol and the decoder's documented
//   classes and texts.

//! AUD: Renesas/Hitachi Advanced User Debugger, Branch Trace mode.

use crate::core::Result;
use crate::decode::{
    AnnClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef, DecoderInfo,
};

const AUDCK: usize = 0;
const NAUDSYNC: usize = 1;
// AUDATA3 .. AUDATA0, in that channel order.
const AUDATA3: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "aud",
    name: "AUD",
    longname: "Advanced User Debugger",
    desc: "Renesas/Hitachi Advanced User Debugger (AUD) protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Debug/trace"],
    channels: &[
        ChannelDesc { id: "audck", name: "AUDCK", desc: "AUD clock" },
        ChannelDesc { id: "naudsync", name: "nAUDSYNC", desc: "AUD sync" },
        ChannelDesc { id: "audata3", name: "AUDATA3", desc: "AUD data line 3" },
        ChannelDesc { id: "audata2", name: "AUDATA2", desc: "AUD data line 2" },
        ChannelDesc { id: "audata1", name: "AUDATA1", desc: "AUD data line 1" },
        ChannelDesc { id: "audata0", name: "AUDATA0", desc: "AUD data line 0" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[AnnClass { id: "dest", desc: "Destination address" }],
    annotation_rows: &[],
    binary: &[],
    doc: "This protocol decoder decodes the AUD (Advanced User Debugger) \
interface of certain Renesas / Hitachi microcontrollers, when set in Branch \
Trace mode.\n\n\
AUD has two modes, this PD currently only supports \"Branch Trace\" mode.\n\n\
Details:\n\
http://www.renesas.eu/products/mpumcu/superh/sh7050/sh7058/Documentation.jsp\n\
(\"rej09b0046 - SH7058 Hardware manual\")",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Aud::default())
}

/// The AUD decoder.
#[derive(Default)]
pub struct Aud {
    ncnt: u32,
    nmax: u32,
    addr: u128,
    lastaddr: u128,
    ss: u64,
}

impl Aud {
    /// The Python decoder's `handle_clk_edge(clk, sync, datapins)`.
    fn handle_clk_edge(&mut self, ctx: &mut dyn Context, sync: bool, data: [bool; 4]) {
        // "Reconstruct nibble."
        let mut nib = 0u32;
        for i in 0..4 {
            nib |= u32::from(data[3 - i]) << i;
        }

        // "sync == 1: annotate if finished; update cmd."
        if sync {
            if self.ncnt == self.nmax && self.nmax != 0 {
                // "Done shifting an address: annotate."
                ctx.put_ann(self.ss, ctx.samplenum(), 0, vec![format!("0x{:08X}", self.addr)]);
                self.lastaddr = self.addr;
            }

            self.ncnt = 0;
            self.addr = self.lastaddr;
            self.ss = ctx.samplenum();
            self.nmax = match nib {
                0x08 => 1,
                0x09 => 2,
                0x0a => 4,
                0x0b => 8,
                // "Undefined or idle."
                _ => 0,
            };
        } else if self.nmax > 0 {
            // "sync == 0, valid cmd: start or continue shifting in nibbles."
            // "Clear tgt nibble."
            let shift = self.ncnt * 4;
            if shift < 128 {
                self.addr &= !(0x0Fu128 << shift);
                // "Set nibble."
                self.addr |= u128::from(nib) << shift;
            }
            self.ncnt += 1;
        }
    }
}

impl Decoder for Aud {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Aud::default();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let cond: &[Cond] = &[Cond::pin(AUDCK, CondKind::Rising)];
        loop {
            let pins = ctx.wait(&[cond])?;
            let sync = pins.get(NAUDSYNC);
            let data = [
                pins.get(AUDATA3),
                pins.get(AUDATA3 + 1),
                pins.get(AUDATA3 + 2),
                pins.get(AUDATA3 + 3),
            ];
            self.handle_clk_edge(ctx, sync, data);
        }
    }
}
