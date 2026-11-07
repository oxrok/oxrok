// Counterpart in sigrok:
//   libsigrokdecode/decoders/tdm_audio/pd.py,
//   Copyright (C) 2019 Ben Dooks <ben.dooks@codethink.co.uk>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the TDM audio bus and the decoder's documented
//   classes and texts.

//! TDM audio: shifts bits in on the configured clock edge, one channel
//! per sample, and restarts at each frame-sync pulse.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const CLOCK: usize = 0;
const FRAME: usize = 1;
const DATA: usize = 2;

const MAX_CHANNELS: i64 = 8;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "tdm_audio",
    name: "TDM audio",
    longname: "Time division multiplex audio",
    desc: "TDM multi-channel audio protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Audio"],
    channels: &[
        ChannelDesc { id: "clock", name: "Bitclk", desc: "Data bit clock" },
        ChannelDesc { id: "frame", name: "Framesync", desc: "Frame sync" },
        ChannelDesc { id: "data", name: "Data", desc: "Serial data" },
    ],
    optional_channels: &[],
    options: &[
        OptionDesc { id: "bps", desc: "Bits per sample", default: OptionValue::i(16), values: &[] },
        OptionDesc {
            id: "channels",
            desc: "Channels per frame",
            default: OptionValue::i(MAX_CHANNELS),
            values: &[],
        },
        OptionDesc {
            id: "edge",
            desc: "Clock edge to sample on",
            default: OptionValue::s("rising"),
            values: &[OptionValue::s("rising"), OptionValue::s("falling")],
        },
    ],
    annotations: &[
        AnnClass { id: "ch0", desc: "Ch0" },
        AnnClass { id: "ch1", desc: "Ch1" },
        AnnClass { id: "ch2", desc: "Ch2" },
        AnnClass { id: "ch3", desc: "Ch3" },
        AnnClass { id: "ch4", desc: "Ch4" },
        AnnClass { id: "ch5", desc: "Ch5" },
        AnnClass { id: "ch6", desc: "Ch6" },
        AnnClass { id: "ch7", desc: "Ch7" },
    ],
    annotation_rows: &[
        AnnRow { id: "ch0", desc: "Ch0", classes: &[0] },
        AnnRow { id: "ch1", desc: "Ch1", classes: &[1] },
        AnnRow { id: "ch2", desc: "Ch2", classes: &[2] },
        AnnRow { id: "ch3", desc: "Ch3", classes: &[3] },
        AnnRow { id: "ch4", desc: "Ch4", classes: &[4] },
        AnnRow { id: "ch5", desc: "Ch5", classes: &[5] },
        AnnRow { id: "ch6", desc: "Ch6", classes: &[6] },
        AnnRow { id: "ch7", desc: "Ch7", classes: &[7] },
    ],
    binary: &[],
    doc: "TDM Audio is an audio serial bus for moving audio data between devices\n\
(usually on the same board) which can carry one or more channels of data.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(TdmAudio::default())
}

struct TdmAudio {
    samplerate: u64,
    channels: i64,
    channel: i64,
    bitdepth: i64,
    bitcount: i64,
    samplecount: i64,
    lastsync: i64,
    lastframe: bool,
    data: u128,
    ss_block: Option<u64>,
    edge: CondKind,
}

impl Default for TdmAudio {
    fn default() -> Self {
        TdmAudio {
            samplerate: 0,
            channels: MAX_CHANNELS,
            channel: 0,
            bitdepth: 16,
            bitcount: 0,
            samplecount: 0,
            lastsync: 0,
            lastframe: false,
            data: 0,
            ss_block: None,
            edge: CondKind::Rising,
        }
    }
}

impl Decoder for TdmAudio {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.samplerate = 0;
        self.channels = MAX_CHANNELS;
        self.channel = 0;
        self.bitdepth = 16;
        self.bitcount = 0;
        self.samplecount = 0;
        self.lastsync = 0;
        self.lastframe = false;
        self.data = 0;
        self.ss_block = None;

        self.bitdepth = config.int("bps", 16);
        // The Python `start()` reads `bps` and `edge` only; the declared
        // `channels` option is never used, so `self.channels` stays at
        // MAX_CHANNELS whatever the option says.
        self.edge = if config.str("edge", "rising") == "falling" {
            CondKind::Falling
        } else {
            CondKind::Rising
        };
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            // "Wait for edge of clock (sample on rising/falling edge)."
            let pins = ctx.wait(&[&[Cond::pin(CLOCK, self.edge)]])?;
            let frame = pins.get(FRAME);
            let data = pins.get(DATA);

            self.data = (self.data << 1) | u128::from(data);
            self.bitcount += 1;

            if let Some(ss_block) = self.ss_block
                && self.bitcount >= self.bitdepth
            {
                self.bitcount = 0;
                self.channel += 1;

                let c1 = format!("Channel {}", self.channel);
                let c2 = format!("C{}", self.channel);
                let c3 = format!("{}", self.channel);
                let v = if self.bitdepth <= 8 {
                    format!("{:02x}", self.data)
                } else if self.bitdepth <= 16 {
                    format!("{:04x}", self.data)
                } else {
                    format!("{:08x}", self.data)
                };

                let ch = if self.channel < self.channels { self.channel } else { 0 };

                ctx.put_ann(
                    ss_block,
                    ctx.samplenum(),
                    ch as usize,
                    vec![format!("{c1}: {v}"), format!("{c2}: {v}"), format!("{c3}: {v}")],
                );
                self.data = 0;
                self.ss_block = Some(ctx.samplenum());
                self.samplecount += 1;
            }

            // "Check for new frame.
            // Note, frame may be a single clock, or active for the first
            // sample in the frame."
            if frame != self.lastframe && frame {
                self.channel = 0;
                self.bitcount = 0;
                self.data = 0;
                if self.ss_block.is_none() {
                    self.ss_block = Some(0);
                }
            }

            self.lastframe = frame;
        }
    }
}
