// Counterpart in sigrok:
//   libsigrokdecode/decoders/mcs48/pd.py,
//   Copyright (C) 2018 fenugrec <fenugrec@users.sourceforge.net>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the MCS-48 external memory bus and the decoder's
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The address is latched on the falling ALE edge as A8..A11 (and
//     the optional A12) followed by the eight data pins as A0..A7:
//     `addr <<= len(data)` with the data bits shifted in below.
//   - Data is sampled on the rising PSEN edge. Before the first ALE
//     pulse `started` is 0 and PSEN edges emit nothing.
//   - A12 is only used when the optional channel is assigned
//     (`has_channel(OFF_BANK_BOT)`), and then becomes the highest
//     address bit.

//! MCS-48: Intel 8039/8048 external program memory accesses.

use crate::core::Result;
use crate::decode::{
    AnnClass, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

// Channel indices, in declaration order.
const ALE: usize = 0;
const PSEN: usize = 1;
const DATA_BOT: usize = 2;
const DATA_TOP: usize = 10;
const ADDR_BOT: usize = 10;
const ADDR_TOP: usize = 14;
const BANK_BOT: usize = 14;
const BANK_TOP: usize = 15;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_ROMDATA: usize = 0;
const BIN_ROMDATA: usize = 0;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "mcs48",
    name: "MCS-48",
    longname: "Intel MCS-48",
    desc: "Intel MCS-48 external memory access protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Retro computing"],
    channels: &[
        ChannelDesc { id: "ale", name: "ALE", desc: "Address latch enable" },
        ChannelDesc { id: "psen", name: "/PSEN", desc: "Program store enable" },
        ChannelDesc { id: "d0", name: "D0", desc: "CPU data line 0" },
        ChannelDesc { id: "d1", name: "D1", desc: "CPU data line 1" },
        ChannelDesc { id: "d2", name: "D2", desc: "CPU data line 2" },
        ChannelDesc { id: "d3", name: "D3", desc: "CPU data line 3" },
        ChannelDesc { id: "d4", name: "D4", desc: "CPU data line 4" },
        ChannelDesc { id: "d5", name: "D5", desc: "CPU data line 5" },
        ChannelDesc { id: "d6", name: "D6", desc: "CPU data line 6" },
        ChannelDesc { id: "d7", name: "D7", desc: "CPU data line 7" },
        ChannelDesc { id: "a8", name: "A8", desc: "CPU address line 8" },
        ChannelDesc { id: "a9", name: "A9", desc: "CPU address line 9" },
        ChannelDesc { id: "a10", name: "A10", desc: "CPU address line 10" },
        ChannelDesc { id: "a11", name: "A11", desc: "CPU address line 11" },
    ],
    optional_channels: &[ChannelDesc { id: "a12", name: "A12", desc: "CPU address line 12" }],
    options: &[],
    annotations: &[AnnClass { id: "romdata", desc: "Address:Data" }],
    annotation_rows: &[],
    binary: &[BinClass { id: "romdata", desc: "AAAA:DD" }],
    doc: "This protocol decoder de-multiplexes Intel MCS-48 (8039, 8048, etc.) \
          external program memory accesses.\n\
          \n\
          This requires 14 channels: 8 for D0-D7 (data and lower 8 bits of \
          address), 4 for A8-A11 (output on port P2), ALE and PSEN.\n\
          \n\
          An optional A12 is supported, which may be an arbitrary I/O pin driven \
          by software (use case is dumping ROM of an HP 3478A).",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Mcs48::new())
}

struct Mcs48 {
    addr: u32,
    addr_s: u64,
    data: u32,
    data_s: u64,
    /// Flag to make sure we get an ALE pulse first.
    started: bool,
}

impl Mcs48 {
    fn new() -> Mcs48 {
        Mcs48 { addr: 0, addr_s: 0, data: 0, data_s: 0, started: false }
    }

    /// pd.py `newaddr()`: falling ALE, reconstruct the address.
    fn newaddr(&mut self, addr_bits: &[u8], data_bits: &[u8], samplenum: u64) {
        self.started = true;
        let mut addr: u32 =
            addr_bits.iter().enumerate().map(|(i, &bit)| u32::from(bit) << i).sum::<u32>();
        addr <<= data_bits.len();
        addr |= data_bits.iter().enumerate().map(|(i, &bit)| u32::from(bit) << i).sum::<u32>();
        self.addr = addr;
        self.addr_s = samplenum;
    }

    /// pd.py `newdata()`: an edge on PSEN, get the data.
    fn newdata(&mut self, data_bits: &[u8], samplenum: u64, ctx: &mut dyn Context) {
        let data: u32 = data_bits.iter().enumerate().map(|(i, &bit)| u32::from(bit) << i).sum();
        self.data = data;
        self.data_s = samplenum;
        if self.started {
            let anntext = format!("{:04X}:{:02X}", self.addr, self.data);
            ctx.put_ann(self.addr_s, self.data_s, ANN_ROMDATA, vec![anntext]);
            let mut bindata = (self.addr as u16).to_be_bytes().to_vec();
            bindata.push(self.data as u8);
            ctx.put_binary(self.addr_s, self.data_s, BIN_ROMDATA, bindata);
        }
    }
}

impl Decoder for Mcs48 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.addr = 0;
        self.addr_s = 0;
        self.data = 0;
        self.data_s = 0;
        self.started = false;
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Address bits above A11 are optional, and are considered to be
        // A12+.
        let has_bank = ctx.has_channel(BANK_BOT);
        let bank_pin_count = usize::from(has_bank);
        loop {
            let ale_fall = [Cond::pin(ALE, CondKind::Falling)];
            let psen_rise = [Cond::pin(PSEN, CondKind::Rising)];
            let pins = ctx.wait(&[&ale_fall, &psen_rise])?;
            let matched = ctx.matched().to_vec();
            let samplenum = ctx.samplenum();
            let data = pins.0[DATA_BOT..DATA_TOP].to_vec();
            let mut addr = pins.0[ADDR_BOT..ADDR_TOP].to_vec();
            let bank = &pins.0[BANK_BOT..BANK_TOP];
            if has_bank {
                addr.extend_from_slice(&bank[..bank_pin_count]);
            }
            if matched[0] {
                self.newaddr(&addr, &data, samplenum);
            }
            if matched[1] {
                self.newdata(&data, samplenum, ctx);
            }
        }
    }
}
