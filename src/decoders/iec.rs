// Counterpart in sigrok:
//   libsigrokdecode/decoders/iec/pd.py,
//   Copyright (C) 2017 Marcus Comstedt <marcus@mc.pp.se>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the IEC bus handshake and the decoder's documented
//   classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The four `step_wait_conds` groups: a falling ATN edge resets the
//     state machine, otherwise step 0 waits for DATA low with CLK high,
//     step 1 for DATA high with CLK high (or CLK low, which aborts),
//     step 2 for DATA low with CLK high (EOI) or CLK low, and step 3
//     for any CLK edge, latching DATA on the rising and counting bits
//     on the falling edge.
//   - In step 3 the latch only happens when the second condition group
//     (`{1: 'e'}`) matched, so a sample where only the ATN reset group
//     matched still runs the step 0 branch, not a bit.
//   - An item is annotated with class 0 ("%02X"), its GPIB name with
//     class 1 and the EOI marker (a space when absent) with class 2.
//   - With ATN a command is a GPIB mnemonic, a listener/talker address
//     or a channel reopen/close/open, `chr()` of the offset byte;
//     without ATN a printable byte is shown as itself and 0x0a/0x0d as
//     LF/CR.

//! IEC: the Commodore serial IEEE-488 bus, decoded to GPIB items.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

// Channel indices, in declaration order.
const DATA: usize = 0;
const CLK: usize = 1;
const ATN: usize = 2;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_ITEMS: usize = 0;
const ANN_GPIB: usize = 1;
const ANN_EOI: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "iec",
    name: "IEC",
    longname: "Commodore bus",
    desc: "Commodore serial IEEE-488 (IEC) bus protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["gpib"],
    tags: &[],
    channels: &[
        ChannelDesc { id: "data", name: "DATA", desc: "Data I/O" },
        ChannelDesc { id: "clk", name: "CLK", desc: "Clock" },
        ChannelDesc { id: "atn", name: "ATN", desc: "Attention" },
    ],
    optional_channels: &[ChannelDesc { id: "srq", name: "SRQ", desc: "Service request" }],
    options: &[],
    annotations: &[
        AnnClass { id: "items", desc: "Items" },
        AnnClass { id: "gpib", desc: "DAT/CMD" },
        AnnClass { id: "eoi", desc: "EOI" },
    ],
    annotation_rows: &[
        AnnRow { id: "bytes", desc: "Bytes", classes: &[0] },
        AnnRow { id: "gpib", desc: "DAT/CMD", classes: &[1] },
        AnnRow { id: "eoi", desc: "EOI", classes: &[2] },
    ],
    binary: &[],
    doc: "This protocol decoder can decode the Commodore serial IEEE-488 \
          (IEC) protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Iec::new())
}

struct Iec {
    saved_atn: bool,
    saved_eoi: bool,
    ss_item: Option<u64>,
    es_item: u64,
    step: usize,
    bits: u8,
    numbits: u32,
}

impl Iec {
    fn new() -> Iec {
        Iec {
            saved_atn: false,
            saved_eoi: false,
            ss_item: None,
            es_item: 0,
            step: 0,
            bits: 0,
            numbits: 0,
        }
    }

    /// pd.py `handle_bits()`: the saved item, its GPIB name and EOI.
    fn handle_bits(&mut self, ctx: &mut dyn Context) {
        let dbyte = self.bits;
        let datn = self.saved_atn;
        let deoi = self.saved_eoi;
        self.es_item = ctx.samplenum();
        let ss = self.ss_item.unwrap_or(self.es_item);
        ctx.put_ann(ss, self.es_item, ANN_ITEMS, vec![format!("{dbyte:02X}")]);

        // Encode item byte to GPIB convention.
        let mut strgpib = " ".to_owned();
        if datn {
            // ATN, decode commands.
            // Note: Commands < 0x20 are not used on IEC bus.
            match dbyte {
                0x01 => strgpib = "GTL".into(),
                0x04 => strgpib = "SDC".into(),
                0x05 => strgpib = "PPC".into(),
                0x08 => strgpib = "GET".into(),
                0x09 => strgpib = "TCT".into(),
                0x11 => strgpib = "LLO".into(),
                0x14 => strgpib = "DCL".into(),
                0x15 => strgpib = "PPU".into(),
                0x18 => strgpib = "SPE".into(),
                0x19 => strgpib = "SPD".into(),
                0x3f => strgpib = "UNL".into(),
                0x5f => strgpib = "UNT".into(),
                _ => {}
            }
            if dbyte > 0x1f && dbyte < 0x3f {
                // Address listener.
                strgpib = format!("L{}", char::from(dbyte + 0x10));
            }
            if dbyte > 0x3f && dbyte < 0x5f {
                // Address talker.
                strgpib = format!("T{}", char::from(dbyte - 0x10));
            }
            if dbyte > 0x5f && dbyte < 0x70 {
                // Channel reopen.
                strgpib = format!("R{}", char::from(dbyte - 0x30));
            }
            if dbyte > 0xdf && dbyte < 0xf0 {
                // Channel close.
                strgpib = format!("C{}", char::from(dbyte - 0xb0));
            }
            if dbyte > 0xef {
                // Channel open.
                strgpib = format!("O{}", char::from(dbyte - 0xc0));
            }
        } else {
            if dbyte > 0x1f && dbyte < 0x7f {
                strgpib = char::from(dbyte).to_string();
            }
            if dbyte == 0x0a {
                strgpib = "LF".into();
            }
            if dbyte == 0x0d {
                strgpib = "CR".into();
            }
        }
        ctx.put_ann(ss, self.es_item, ANN_GPIB, vec![strgpib]);

        let streoi = if deoi { "EOI" } else { " " };
        ctx.put_ann(ss, self.es_item, ANN_EOI, vec![streoi.to_owned()]);
    }
}

impl Decoder for Iec {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.saved_atn = false;
        self.saved_eoi = false;
        self.ss_item = None;
        self.es_item = 0;
        self.step = 0;
        self.bits = 0;
        self.numbits = 0;
        Ok(())
    }

    #[allow(clippy::too_many_lines)]
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            let atn_fall = [Cond::pin(ATN, CondKind::Falling)];
            let matched;
            let pins;
            match self.step {
                0 => {
                    let ready = [Cond::pin(DATA, CondKind::Low), Cond::pin(CLK, CondKind::High)];
                    pins = ctx.wait(&[&atn_fall, &ready])?;
                    matched = ctx.matched().to_vec();
                }
                1 => {
                    let ready = [Cond::pin(DATA, CondKind::High), Cond::pin(CLK, CondKind::High)];
                    let abort = [Cond::pin(CLK, CondKind::Low)];
                    pins = ctx.wait(&[&atn_fall, &ready, &abort])?;
                    matched = ctx.matched().to_vec();
                }
                2 => {
                    let eoi = [Cond::pin(DATA, CondKind::Low), Cond::pin(CLK, CondKind::High)];
                    let clk_low = [Cond::pin(CLK, CondKind::Low)];
                    pins = ctx.wait(&[&atn_fall, &eoi, &clk_low])?;
                    matched = ctx.matched().to_vec();
                }
                _ => {
                    let clk_edge = [Cond::pin(CLK, CondKind::Edge)];
                    pins = ctx.wait(&[&atn_fall, &clk_edge])?;
                    matched = ctx.matched().to_vec();
                }
            }
            let data = pins.0[DATA];
            let clk = pins.0[CLK];
            let atn = pins.0[ATN];

            if matched[0] {
                // Falling edge on ATN, reset step.
                self.step = 0;
            }

            if self.step == 0 {
                // Don't use self.matched[1] here since we might come from
                // a step with different conds due to the code above.
                if data == 0 && clk == 1 {
                    // Rising edge on CLK while DATA is low: Ready to send.
                    self.step = 1;
                }
            } else if self.step == 1 {
                if data == 1 && clk == 1 {
                    // Rising edge on DATA while CLK is high: Ready for data.
                    self.ss_item = Some(ctx.samplenum());
                    self.saved_atn = atn == 0;
                    self.saved_eoi = false;
                    self.bits = 0;
                    self.numbits = 0;
                    self.step = 2;
                } else if clk == 0 {
                    // CLK low again, transfer aborted.
                    self.step = 0;
                }
            } else if self.step == 2 {
                if data == 0 && clk == 1 {
                    // DATA goes low while CLK is still high, EOI confirmed.
                    self.saved_eoi = true;
                } else if clk == 0 {
                    self.step = 3;
                }
            } else if matched[1] {
                if clk == 1 {
                    // Rising edge on CLK; latch DATA.
                    self.bits |= data << self.numbits;
                } else if clk == 0 {
                    // Falling edge on CLK; end of bit.
                    self.numbits += 1;
                    if self.numbits == 8 {
                        self.handle_bits(ctx);
                        self.step = 0;
                    }
                }
            }
        }
    }
}
