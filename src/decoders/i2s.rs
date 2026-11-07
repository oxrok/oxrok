// Counterpart in sigrok:
//   libsigrokdecode/decoders/i2s/pd.py,
//   Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2S bus and the decoder's documented classes
//   and texts.

//! I²S: shifts bits in on the bit clock and reports a sample each time
//! the word-select line flips, as an annotation, a Python item and a
//! little-endian WAV sample.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item,
};

const SCK: usize = 0;
const WS: usize = 1;
const SD: usize = 2;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_LEFT: usize = 0;
const ANN_RIGHT: usize = 1;
const ANN_WARNINGS: usize = 2;

// Binary class index.
const BIN_WAV: usize = 0;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "i2s",
    name: "I²S",
    longname: "Integrated Interchip Sound",
    desc: "Serial bus for connecting digital audio devices.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["i2s"],
    tags: &["Audio", "PC"],
    channels: &[
        ChannelDesc { id: "sck", name: "SCK", desc: "Bit clock line" },
        ChannelDesc { id: "ws", name: "WS", desc: "Word select line" },
        ChannelDesc { id: "sd", name: "SD", desc: "Serial data line" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "left", desc: "Left channel" },
        AnnClass { id: "right", desc: "Right channel" },
        AnnClass { id: "warnings", desc: "Warnings" },
    ],
    annotation_rows: &[],
    binary: &[BinClass { id: "wav", desc: "WAV file" }],
    doc: "I²S (Integrated Interchip Sound) is a serial bus for connecting digital\n\
audio devices (usually on the same device/board).\n\
\n\
Details:\n\
http://www.nxp.com/acrobat_download/various/I2SBUS.pdf\n\
http://en.wikipedia.org/wiki/I2s",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(I2s::default())
}

/// "Chunk descriptor", "Fmt subchunk" and "Data subchunk" of the fixed
/// 44-byte WAV header the Python decoder emits before the first sample.
fn wav_header() -> Vec<u8> {
    let mut h = Vec::with_capacity(44);
    h.extend_from_slice(b"RIFF");
    h.extend_from_slice(b"\x24\x80\x00\x00"); // Chunk size (2084)
    h.extend_from_slice(b"WAVE");
    // Fmt subchunk
    h.extend_from_slice(b"fmt ");
    h.extend_from_slice(b"\x10\x00\x00\x00"); // Subchunk size (16 bytes)
    h.extend_from_slice(b"\x01\x00"); // Audio format (0x0001 == PCM)
    h.extend_from_slice(b"\x02\x00"); // Number of channels (2)
    h.extend_from_slice(b"\x80\x3e\x00\x00"); // Samplerate (16000)
    h.extend_from_slice(b"\x00\xfa\x00\x00"); // Byterate (64000)
    h.extend_from_slice(b"\x04\x00"); // Blockalign (4)
    h.extend_from_slice(b"\x20\x00"); // Bits per sample (32)
    // Data subchunk
    h.extend_from_slice(b"data");
    h.extend_from_slice(b"\xff\xff\xff\xff"); // Subchunk size (4G bytes) TODO
    h
}

/// `struct.pack('<I', self.data)`: four little-endian bytes, an error
/// above 32 bits as the Python `struct` error would end the decoder.
fn wav_sample(data: u128) -> Result<Vec<u8>> {
    if data > u128::from(u32::MAX) {
        return Err(Error::arg("sample does not fit in a WAV word"));
    }
    Ok((data as u32).to_le_bytes().to_vec())
}

/// A Python `int` item, using the wide form above `i64`.
fn int_item(value: u128) -> Item {
    if value <= i64::MAX as u128 {
        Item::Int(value as i64)
    } else {
        let bytes = value.to_be_bytes();
        let first = bytes.iter().position(|&b| b != 0).unwrap_or(bytes.len());
        Item::BigUint(bytes[first..].to_vec())
    }
}

#[derive(Default)]
struct I2s {
    samplerate: u64,
    oldws: bool,
    bitcount: i64,
    data: u128,
    samplesreceived: i64,
    first_sample: Option<u64>,
    ss_block: Option<u64>,
    wordlength: i64,
    wrote_wav_header: bool,
}

impl Decoder for I2s {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.samplerate = 0;
        self.oldws = true;
        self.bitcount = 0;
        self.data = 0;
        self.samplesreceived = 0;
        self.first_sample = None;
        self.ss_block = None;
        self.wordlength = -1;
        self.wrote_wav_header = false;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            // "Wait for a rising edge on the SCK pin."
            let pins = ctx.wait(&[&[Cond::pin(SCK, CondKind::Rising)]])?;
            let ws = pins.get(WS);
            let sd = pins.get(SD);

            self.data = (self.data << 1) | u128::from(sd);
            self.bitcount += 1;

            // "This was not the LSB unless WS has flipped."
            if ws == self.oldws {
                continue;
            }

            // "Only submit the sample, if we received the beginning of it."
            if let Some(ss_block) = self.ss_block {
                if !self.wrote_wav_header {
                    ctx.put_binary(0, 0, BIN_WAV, wav_header());
                    self.wrote_wav_header = true;
                }

                self.samplesreceived += 1;

                ctx.wait(&[&[Cond::pin(SCK, CondKind::Falling)]])?;

                let idx = if self.oldws { ANN_RIGHT } else { ANN_LEFT };
                let (c1, c2, c3) = if self.oldws {
                    ("Right channel", "Right", "R")
                } else {
                    ("Left channel", "Left", "L")
                };
                let v = format!("{:08x}", self.data);
                ctx.put_python(
                    ss_block,
                    ctx.samplenum(),
                    Item::List(vec![
                        Item::Str("DATA".into()),
                        Item::List(vec![Item::Str(c3.into()), int_item(self.data)]),
                    ]),
                );
                ctx.put_ann(
                    ss_block,
                    ctx.samplenum(),
                    idx,
                    vec![
                        format!("{c1}: {v}"),
                        format!("{c2}: {v}"),
                        format!("{c3}: {v}"),
                        c3.into(),
                    ],
                );
                ctx.put_binary(ss_block, ctx.samplenum(), BIN_WAV, wav_sample(self.data)?);

                // "Check that the data word was the correct length."
                if self.wordlength != -1 && self.wordlength != self.bitcount {
                    ctx.put_ann(
                        ss_block,
                        ctx.samplenum(),
                        ANN_WARNINGS,
                        vec![format!(
                            "Received {}-bit word, expected {}-bit word",
                            self.bitcount, self.wordlength
                        )],
                    );
                }

                self.wordlength = self.bitcount;
            } else {
                ctx.wait(&[&[Cond::pin(SCK, CondKind::Falling)]])?;
            }

            // "Reset decoder state."
            self.data = 0;
            self.bitcount = 0;
            let samplenum = ctx.samplenum();
            self.ss_block = Some(samplenum);

            // "Save the first sample position."
            if self.first_sample.is_none() {
                self.first_sample = Some(samplenum);
            }

            self.oldws = ws;
        }
    }
}
