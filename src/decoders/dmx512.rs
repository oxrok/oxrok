// Counterpart in sigrok:
//   libsigrokdecode/decoders/dmx512/pd.py,
//   Copyright (C) 2016 Fabian J. Stumpf <sigrok@fabianstumpf.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the DMX512 line timing and the decoder's
//   documented classes and texts.
#![allow(clippy::bool_to_int_with_if, clippy::too_many_lines)]

//! DMX512: break, mark after break, start code, channels, interframe and
//! interpacket times.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const DMX: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_BREAK: usize = 1;
const ANN_MAB: usize = 2;
const ANN_STARTBIT: usize = 3;
const ANN_STOPBITS: usize = 4;
const ANN_STARTCODE: usize = 5;
const ANN_CHANNEL: usize = 6;
const ANN_INTERFRAME: usize = 7;
const ANN_INTERPACKET: usize = 8;
const ANN_DATA: usize = 9;
const ANN_ERROR: usize = 10;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "dmx512",
    name: "DMX512",
    longname: "Digital MultipleX 512",
    desc: "Digital MultipleX 512 (DMX512) lighting protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Embedded/industrial", "Lighting"],
    channels: &[ChannelDesc { id: "dmx", name: "DMX data", desc: "Any DMX data line" }],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "break", desc: "Break" },
        AnnClass { id: "mab", desc: "Mark after break" },
        AnnClass { id: "startbit", desc: "Start bit" },
        AnnClass { id: "stopbits", desc: "Stop bit" },
        AnnClass { id: "startcode", desc: "Start code" },
        AnnClass { id: "channel", desc: "Channel" },
        AnnClass { id: "interframe", desc: "Interframe" },
        AnnClass { id: "interpacket", desc: "Interpacket" },
        AnnClass { id: "data", desc: "Data" },
        AnnClass { id: "error", desc: "Error" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 3, 4] },
        AnnRow { id: "data", desc: "Data", classes: &[9] },
        AnnRow { id: "name", desc: "Logical", classes: &[1, 2, 5, 6, 7, 8] },
        AnnRow { id: "errors", desc: "Errors", classes: &[10] },
    ],
    binary: &[],
    doc: "DMX512 (Digital MultipleX 512) is a protocol based on RS485, used to control\n\
professional lighting fixtures.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Dmx512::default())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    FindBreak,
    MarkMab,
    ReadByte,
    MarkIft,
}

/// The DMX512 decoder.
pub struct Dmx512 {
    samplerate: u64,
    sample_usec: f64,
    skip_per_bit: i64,
    run_start: i64,
    run_bit: i64,
    state: State,
    bit_break: i64,
    channel: i64,
    bit: i64,
    aggreg: i64,
    byte: i64,
    next_sample: i64,
}

impl Default for Dmx512 {
    fn default() -> Self {
        Dmx512 {
            samplerate: 0,
            sample_usec: 0.0,
            skip_per_bit: 0,
            run_start: -1,
            run_bit: 0,
            state: State::FindBreak,
            bit_break: 0,
            channel: 0,
            bit: 0,
            aggreg: 0,
            byte: 0,
            next_sample: 0,
        }
    }
}

impl Dmx512 {
    fn putr(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.run_start.max(0) as u64, ctx.samplenum(), class, texts);
    }
}

impl Decoder for Dmx512 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        self.sample_usec = 1.0 / samplerate as f64 * 1_000_000.0;
        self.skip_per_bit = (4.0 / self.sample_usec) as i64;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        loop {
            // Seek for an interval with no state change with a length
            // between 88 and 1000000 us (BREAK).
            if self.state == State::FindBreak {
                let kind = if self.run_bit == 0 { CondKind::High } else { CondKind::Low };
                let pins = ctx.wait(&[&[Cond::Pin(DMX, kind)]])?;
                let samplenum = ctx.samplenum() as i64;
                let runlen = (samplenum - self.run_start) as f64 * self.sample_usec;
                if runlen > 88.0 && runlen < 1_000_000.0 {
                    self.putr(ctx, ANN_BREAK, vec!["Break".into()]);
                    self.bit_break = self.run_bit;
                    self.state = State::MarkMab;
                    self.channel = 0;
                } else if runlen >= 1_000_000.0 {
                    // Error condition.
                    self.putr(ctx, ANN_ERROR, vec!["Invalid break length".into()]);
                }
                self.run_bit = i64::from(pins.get(DMX));
                self.run_start = samplenum;
            // Directly following the BREAK is the MARK AFTER BREAK.
            } else if self.state == State::MarkMab {
                let kind = if self.run_bit == 0 { CondKind::High } else { CondKind::Low };
                let pins = ctx.wait(&[&[Cond::Pin(DMX, kind)]])?;
                self.putr(ctx, ANN_MAB, vec!["MAB".into()]);
                self.state = State::ReadByte;
                self.channel = 0;
                self.bit = 0;
                self.aggreg = i64::from(pins.get(DMX));
                self.run_start = ctx.samplenum() as i64;
            // Mark and read a single transmitted byte
            // (start bit, 8 data bits, 2 stop bits).
            } else if self.state == State::ReadByte {
                let pins = ctx.wait(&[])?;
                let samplenum = ctx.samplenum() as i64;
                self.next_sample = self.run_start + (self.bit + 1) * self.skip_per_bit;
                self.aggreg += i64::from(pins.get(DMX));
                if samplenum != self.next_sample {
                    continue;
                }
                let bit_value = if (self.aggreg as f64 / self.skip_per_bit as f64).round_ties_even()
                    as i64
                    == self.bit_break
                {
                    0
                } else {
                    1
                };

                if self.bit == 0 {
                    self.byte = 0;
                    self.putr(ctx, ANN_STARTBIT, vec!["Start bit".into()]);
                    if bit_value != 0 {
                        // (Possibly) invalid start bit, mark but don't fail.
                        ctx.put_ann(
                            ctx.samplenum(),
                            ctx.samplenum(),
                            ANN_ERROR,
                            vec!["Invalid start bit".into()],
                        );
                    }
                } else if self.bit >= 9 {
                    ctx.put_ann(
                        (samplenum - self.skip_per_bit).max(0) as u64,
                        samplenum as u64,
                        ANN_STOPBITS,
                        vec!["Stop bit".into()],
                    );
                    if bit_value != 1 {
                        // Invalid stop bit, mark.
                        ctx.put_ann(
                            ctx.samplenum(),
                            ctx.samplenum(),
                            ANN_ERROR,
                            vec!["Invalid stop bit".into()],
                        );
                        if self.bit == 10 {
                            // On invalid 2nd stop bit, search for new break.
                            self.run_bit = i64::from(pins.get(DMX));
                            self.state = State::FindBreak;
                        }
                    }
                } else {
                    // Label and process one bit.
                    ctx.put_ann(
                        (samplenum - self.skip_per_bit).max(0) as u64,
                        samplenum as u64,
                        ANN_BIT,
                        vec![bit_value.to_string()],
                    );
                    self.byte |= bit_value << (self.bit - 1);
                }

                // Label a complete byte.
                if self.bit == 10 {
                    let (class, text) = if self.channel == 0 {
                        (ANN_STARTCODE, "Start code".to_owned())
                    } else {
                        (ANN_CHANNEL, format!("Channel {}", self.channel))
                    };
                    ctx.put_ann(
                        self.run_start.max(0) as u64,
                        self.next_sample.max(0) as u64,
                        class,
                        vec![text],
                    );
                    ctx.put_ann(
                        (self.run_start + self.skip_per_bit).max(0) as u64,
                        (self.next_sample - 2 * self.skip_per_bit).max(0) as u64,
                        ANN_DATA,
                        vec![format!("{} / {:#x}", self.byte, self.byte)],
                    );
                    // Continue by scanning the IFT.
                    self.channel += 1;
                    self.run_start = samplenum;
                    self.run_bit = i64::from(pins.get(DMX));
                    self.state = State::MarkIft;
                }

                self.aggreg = i64::from(pins.get(DMX));
                self.bit += 1;
            // Mark the INTERFRAME-TIME between bytes / INTERPACKET-TIME
            // between packets.
            } else if self.state == State::MarkIft {
                let kind = if self.run_bit == 0 { CondKind::High } else { CondKind::Low };
                let pins = ctx.wait(&[&[Cond::Pin(DMX, kind)]])?;
                if self.channel > 512 {
                    self.putr(ctx, ANN_INTERPACKET, vec!["Interpacket".into()]);
                    self.state = State::FindBreak;
                    self.run_bit = i64::from(pins.get(DMX));
                    self.run_start = ctx.samplenum() as i64;
                } else {
                    self.putr(ctx, ANN_INTERFRAME, vec!["Interframe".into()]);
                    self.state = State::ReadByte;
                    self.bit = 0;
                    self.run_start = ctx.samplenum() as i64;
                }
            }
        }
    }
}
