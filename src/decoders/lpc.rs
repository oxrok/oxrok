// Counterpart in sigrok:
//   libsigrokdecode/decoders/lpc/pd.py,
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the LPC bus framing and the decoder's documented
//   classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - Only rising LCLK edges are decoded, but `oldlclk` is *not* updated
//     on the edge branch: a non-clock pin change while LCLK stays high is
//     therefore seen as another rising edge. The Python code does the
//     same, so the same START/TAR/SYNC/DATA outputs appear.
//   - `self.tarcount` is used throughout; `handle_get_addr` sets the
//     misspelled `self.tar_count`, which is never read.
//   - `synccount` is never incremented, so every SYNC annotation says
//     "cycle 0".
//   - LFRAME# may stay asserted for several clocks: every START field is
//     annotated, and the state advances only on the clock where LFRAME#
//     is deasserted (`if lframe != 1: return`).

//! LPC: Low Pin Count bus cycles.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Config, Context, Decoder, DecoderDef, DecoderInfo,
};

const ANN_WARN: usize = 0;
const ANN_START: usize = 1;
const ANN_CT: usize = 2;
const ANN_ADDR: usize = 3;
const ANN_TAR1: usize = 4;
const ANN_SYNC: usize = 5;
const ANN_DATA: usize = 6;
const ANN_TAR2: usize = 7;

/// START field values.
fn start_field(lad: u8) -> &'static str {
    match lad {
        0b0000 => "Start of cycle for a target",
        0b0001 | 0b0100..=0b1100 => "Reserved",
        0b0010 => "Grant for bus master 0",
        0b0011 => "Grant for bus master 1",
        0b1101 => "Start of cycle for a Firmware Memory Read cycle",
        0b1110 => "Start of cycle for a Firmware Memory Write cycle",
        _ => "Stop/abort (end of a cycle for a target)",
    }
}

/// Cycle type / direction field values; odd values and 0b1111 are not
/// in the Python table and become "Reserved / unknown".
fn cycle_type_field(lad: u8) -> &'static str {
    match lad {
        0b0000 => "I/O read",
        0b0010 => "I/O write",
        0b0100 => "Memory read",
        0b0110 => "Memory write",
        0b1000 => "DMA read",
        0b1010 => "DMA write",
        0b1100 | 0b1110 => "Reserved / not allowed",
        _ => "Reserved / unknown",
    }
}

/// SYNC field values.
fn sync_field(lad: u8) -> &'static str {
    match lad {
        0b0000 => "Ready",
        0b0101 => "Short wait",
        0b0110 => "Long wait",
        0b1001 => "Ready more (DMA only)",
        0b1010 => "Error",
        _ => "Reserved",
    }
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "lpc",
    name: "LPC",
    longname: "Low Pin Count",
    desc: "Protocol for low-bandwidth devices on PC mainboards.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["PC"],
    channels: &[
        ChannelDesc { id: "lframe", name: "LFRAME#", desc: "Frame" },
        ChannelDesc { id: "lclk", name: "LCLK", desc: "Clock" },
        ChannelDesc { id: "lad0", name: "LAD[0]", desc: "Addr/control/data 0" },
        ChannelDesc { id: "lad1", name: "LAD[1]", desc: "Addr/control/data 1" },
        ChannelDesc { id: "lad2", name: "LAD[2]", desc: "Addr/control/data 2" },
        ChannelDesc { id: "lad3", name: "LAD[3]", desc: "Addr/control/data 3" },
    ],
    optional_channels: &[
        ChannelDesc { id: "lreset", name: "LRESET#", desc: "Reset" },
        ChannelDesc { id: "ldrq", name: "LDRQ#", desc: "Encoded DMA / bus master request" },
        ChannelDesc { id: "serirq", name: "SERIRQ", desc: "Serialized IRQ" },
        ChannelDesc { id: "clkrun", name: "CLKRUN#", desc: "Clock run" },
        ChannelDesc { id: "lpme", name: "LPME#", desc: "LPC power management event" },
        ChannelDesc { id: "lpcpd", name: "LPCPD#", desc: "Power down" },
        ChannelDesc { id: "lsmi", name: "LSMI#", desc: "System Management Interrupt" },
    ],
    options: &[],
    annotations: &[
        AnnClass { id: "warnings", desc: "Warnings" },
        AnnClass { id: "start", desc: "Start" },
        AnnClass { id: "cycle-type", desc: "Cycle-type/direction" },
        AnnClass { id: "addr", desc: "Address" },
        AnnClass { id: "tar1", desc: "Turn-around cycle 1" },
        AnnClass { id: "sync", desc: "Sync" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "tar2", desc: "Turn-around cycle 2" },
    ],
    annotation_rows: &[
        AnnRow { id: "data", desc: "Data", classes: &[1, 2, 3, 4, 5, 6, 7] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[0] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Lpc::default())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    GetStart,
    GetCtDr,
    GetAddr,
    GetTar,
    GetSync,
    GetData,
    GetTar2,
}

/// The LPC decoder.
pub struct Lpc {
    state: State,
    /// `-1`, 0 or 1, as the Python decoder starts at -1.
    oldlclk: i8,
    /// Last START LAD value; the Python decoder only ever sets it to -1.
    lad: i32,
    addr: u32,
    cur_nibble: i32,
    cycle_type: &'static str,
    databyte: u8,
    tarcount: i32,
    synccount: i32,
    oldpins: Option<Vec<u8>>,
    ss_block: Option<u64>,
    es_block: Option<u64>,
    cycle_count: i32,
    sync_val: String,
}

impl Default for Lpc {
    fn default() -> Self {
        Lpc {
            state: State::Idle,
            oldlclk: -1,
            lad: -1,
            addr: 0,
            cur_nibble: 0,
            cycle_type: "",
            databyte: 0,
            tarcount: 0,
            synccount: 0,
            oldpins: None,
            ss_block: None,
            es_block: None,
            cycle_count: 0,
            sync_val: String::new(),
        }
    }
}

impl Lpc {
    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        let ss = self.ss_block.unwrap_or_else(|| ctx.samplenum());
        let es = self.es_block.unwrap_or_else(|| ctx.samplenum());
        ctx.put_ann(ss, es, class, texts);
    }

    fn handle_get_start(&mut self, ctx: &mut dyn Context, lad: u8, lframe: bool) {
        // "The last value of LAD[3:0] before LFRAME# gets de-asserted is
        // what the peripherals must use", but every START field is
        // annotated anyway.
        self.es_block = Some(ctx.samplenum());
        self.putb(
            ctx,
            ANN_START,
            vec![start_field(lad).to_owned(), "START".to_owned(), "St".to_owned(), "S".to_owned()],
        );
        self.ss_block = Some(ctx.samplenum());

        if self.lad != -1 && self.lad != i32::from(lad) {
            self.putb(ctx, ANN_WARN, vec!["LAD[3:0] changed while LFRAME# was asserted".into()]);
        }

        if lframe {
            self.state = State::GetCtDr;
        }
    }

    fn handle_get_ct_dr(&mut self, ctx: &mut dyn Context, lad: u8, lad_bits: &str) {
        self.cycle_type = cycle_type_field(lad);
        if self.cycle_type.contains("Reserved") {
            self.putb(ctx, ANN_WARN, vec![format!("Invalid cycle type ({lad_bits})")]);
        }

        self.es_block = Some(ctx.samplenum());
        self.putb(ctx, ANN_CT, vec![format!("Cycle type: {}", self.cycle_type)]);
        self.ss_block = Some(ctx.samplenum());

        self.state = State::GetAddr;
        self.addr = 0;
        self.cur_nibble = 0;
    }

    fn handle_get_addr(&mut self, ctx: &mut dyn Context, lad: u8) -> Result<()> {
        // I/O cycles: 4 ADDR clocks. Memory cycles: 8 ADDR clocks.
        // DMA cycles: no ADDR clocks at all.
        let addr_nibbles: i32 = match self.cycle_type {
            "I/O read" | "I/O write" => 4,
            "Memory read" | "Memory write" => 8,
            _ => 0,
        };

        // Addresses are driven MSN-first.
        let offset = ((addr_nibbles - 1) - self.cur_nibble) * 4;
        // Python evaluates `lad << offset`; a negative offset raises
        // "negative shift count".
        if offset < 0 {
            return Err(Error::device("negative shift count"));
        }
        self.addr |= u32::from(lad) << offset;

        if self.cur_nibble < addr_nibbles - 1 {
            self.cur_nibble += 1;
            return Ok(());
        }

        self.es_block = Some(ctx.samplenum());
        let s = format!("Address: 0x{:0width$x}", self.addr, width = addr_nibbles as usize);
        self.putb(ctx, ANN_ADDR, vec![s]);
        self.ss_block = Some(ctx.samplenum());

        self.state = State::GetTar;
        // The Python original sets `self.tar_count` here, a misspelling
        // that is never read; the live counter is `self.tarcount`.
        Ok(())
    }

    fn handle_get_tar(&mut self, ctx: &mut dyn Context, lad_bits: &str) {
        self.es_block = Some(ctx.samplenum());
        self.putb(ctx, ANN_TAR1, vec![format!("TAR, cycle {}: {}", self.tarcount, lad_bits)]);
        self.ss_block = Some(ctx.samplenum());

        if lad_bits != "1111" {
            self.putb(
                ctx,
                ANN_WARN,
                vec![format!("TAR, cycle {}: {} (expected 1111)", self.tarcount, lad_bits)],
            );
        }

        if self.tarcount != 1 {
            self.tarcount += 1;
            return;
        }

        self.tarcount = 0;
        self.state = State::GetSync;
    }

    fn handle_get_sync(&mut self, ctx: &mut dyn Context, lad: u8, lad_bits: &str) {
        lad_bits.clone_into(&mut self.sync_val);
        self.cycle_type = sync_field(lad);

        if self.cycle_type.contains("Reserved") {
            self.putb(
                ctx,
                ANN_WARN,
                vec![format!("SYNC, cycle {}: {} (reserved value)", self.synccount, self.sync_val)],
            );
        }

        self.es_block = Some(ctx.samplenum());
        self.putb(
            ctx,
            ANN_SYNC,
            vec![format!("SYNC, cycle {}: {}", self.synccount, self.sync_val)],
        );
        self.ss_block = Some(ctx.samplenum());

        self.cycle_count = 0;
        self.state = State::GetData;
    }

    fn handle_get_data(&mut self, ctx: &mut dyn Context, lad: u8) -> Result<()> {
        // Data is driven LSN-first.
        match self.cycle_count {
            0 => self.databyte = lad,
            1 => self.databyte |= lad << 4,
            n => return Err(Error::device(format!("Invalid cycle_count: {n}"))),
        }

        if self.cycle_count != 1 {
            self.cycle_count += 1;
            return Ok(());
        }

        self.es_block = Some(ctx.samplenum());
        self.putb(ctx, ANN_DATA, vec![format!("DATA: 0x{:02x}", self.databyte)]);
        self.ss_block = Some(ctx.samplenum());

        self.cycle_count = 0;
        self.state = State::GetTar2;
        Ok(())
    }

    fn handle_get_tar2(&mut self, ctx: &mut dyn Context, lad_bits: &str) {
        self.es_block = Some(ctx.samplenum());
        self.putb(ctx, ANN_TAR2, vec![format!("TAR, cycle {}: {}", self.tarcount, lad_bits)]);
        self.ss_block = Some(ctx.samplenum());

        if lad_bits != "1111" {
            self.putb(
                ctx,
                ANN_WARN,
                vec![format!(
                    "Warning: TAR, cycle {}: {} (expected 1111)",
                    self.tarcount, lad_bits
                )],
            );
        }

        if self.tarcount != 1 {
            self.tarcount += 1;
            return;
        }

        self.tarcount = 0;
        self.state = State::Idle;
    }
}

impl Decoder for Lpc {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            let pins = ctx.wait(&[])?;
            let sn = ctx.samplenum();

            // If none of the pins changed, there's nothing to do.
            if self.oldpins.as_deref() == Some(pins.0.as_slice()) {
                continue;
            }
            self.oldpins = Some(pins.0.clone());

            let lframe = pins.get(0);
            let lclk = pins.get(1);
            let lad0 = pins.get(2);
            let lad1 = pins.get(3);
            let lad2 = pins.get(4);
            let lad3 = pins.get(5);

            // Only look at the signals upon rising LCLK edges. Note the
            // Python code does not update `oldlclk` on this branch.
            if !(self.oldlclk == 0 && lclk) {
                self.oldlclk = i8::from(lclk);
                continue;
            }

            let mut lad = 0u8;
            let mut lad_bits = String::new();
            if self.state != State::Idle {
                lad = (u8::from(lad3) << 3)
                    | (u8::from(lad2) << 2)
                    | (u8::from(lad1) << 1)
                    | u8::from(lad0);
                // Python: '{:04b}'.format(lad).
                lad_bits = format!("{lad:04b}");
            }

            match self.state {
                State::Idle => {
                    // A valid LPC cycle starts with LFRAME# being
                    // asserted (low).
                    if lframe {
                        continue;
                    }
                    self.ss_block = Some(sn);
                    self.state = State::GetStart;
                    self.lad = -1;
                }
                State::GetStart => self.handle_get_start(ctx, lad, lframe),
                State::GetCtDr => self.handle_get_ct_dr(ctx, lad, &lad_bits),
                State::GetAddr => self.handle_get_addr(ctx, lad)?,
                State::GetTar => self.handle_get_tar(ctx, &lad_bits),
                State::GetSync => self.handle_get_sync(ctx, lad, &lad_bits),
                State::GetData => self.handle_get_data(ctx, lad)?,
                State::GetTar2 => self.handle_get_tar2(ctx, &lad_bits),
            }
        }
    }
}
