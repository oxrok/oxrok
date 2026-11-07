// Counterpart in sigrok:
//   libsigrokdecode/decoders/spdif/pd.py,
//   Copyright (C) 2014 Guenther Wenninger <robin@bitschubbser.org>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the S/PDIF physical layer and the decoder's
//   documented classes and texts.

//! S/PDIF: recovers the bit clock from the pulse widths, finds the
//! preambles, and decodes each 28-bit subframe.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BITRATE: usize = 0;
const ANN_PREAMBLE: usize = 1;
const ANN_BITS: usize = 2;
const ANN_AUX: usize = 3;
const ANN_SAMPLES: usize = 4;
const ANN_VALIDITY: usize = 5;
const ANN_SUBCODE: usize = 6;
const ANN_CHAN_STAT: usize = 7;
const ANN_PARITY: usize = 8;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "spdif",
    name: "S/PDIF",
    longname: "Sony/Philips Digital Interface Format",
    desc: "Serial bus for connecting digital audio devices.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Audio", "PC"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "bitrate", desc: "Bitrate / baudrate" },
        AnnClass { id: "preamble", desc: "Preamble" },
        AnnClass { id: "bits", desc: "Bits" },
        AnnClass { id: "aux", desc: "Auxillary-audio-databits" },
        AnnClass { id: "samples", desc: "Audio Samples" },
        AnnClass { id: "validity", desc: "Data Valid" },
        AnnClass { id: "subcode", desc: "Subcode data" },
        AnnClass { id: "chan_stat", desc: "Channnel Status" },
        AnnClass { id: "parity", desc: "Parity Bit" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[2] },
        AnnRow { id: "info", desc: "Info", classes: &[0, 1, 3, 5, 6, 7, 8] },
        AnnRow { id: "samples", desc: "Samples", classes: &[4] },
    ],
    binary: &[],
    doc: "S/PDIF (Sony/Philips Digital Interface Format) is a serial bus for\n\
transmitting audio data.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Spdif::default())
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum State {
    #[default]
    GetFirstPulseWidth,
    GetSecondPulseWidth,
    GetThirdPulseWidth,
    DecodeStream,
    DecodePreamble,
}

/// One decoded pulse: its type (0, 1 or 2), and its start and end.
#[derive(Clone, Copy)]
struct Pulse {
    kind: i64,
    ss: u64,
    es: u64,
}

#[derive(Default)]
struct Spdif {
    samplerate: u64,
    state: State,
    ss_edge: u64,
    samplenum_prev_edge: u64,
    pulse_width: u64,
    clocks: Vec<f64>,
    range1: f64,
    range2: f64,
    preamble_state: u8,
    preamble: Vec<i64>,
    seen_preamble: bool,
    last_preamble: u64,
    first_one: bool,
    subframe: Vec<Pulse>,
    bitcount: i64,
}

impl Spdif {
    fn get_pulse_type(&self) -> i64 {
        if self.range1 == 0.0 || self.range2 == 0.0 {
            return -1;
        }
        let width = self.pulse_width as f64;
        if width >= self.range2 { 2 } else { 1 - i64::from(width >= self.range1) }
    }

    /// "Annotation for a pulse, over the pulse under inspection."
    fn puty(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_edge, ctx.samplenum(), class, texts);
    }

    fn find_first_pulse_width(&mut self) {
        if self.pulse_width != 0 {
            self.clocks.push(self.pulse_width as f64);
            self.state = State::GetSecondPulseWidth;
        }
    }

    fn find_second_pulse_width(&mut self) {
        let first = self.clocks[0];
        let width = self.pulse_width as f64;
        if width > first * 1.3 || width < first * 0.7 {
            self.clocks.push(width);
            self.state = State::GetThirdPulseWidth;
        }
    }

    // Keep the Python decoder's `(a + b) / 2` arithmetic, not
    // `f64::midpoint`: the recovered ranges decide every pulse type.
    #[allow(clippy::manual_midpoint)]
    fn find_third_pulse_width(&mut self, ctx: &mut dyn Context) {
        let width = self.pulse_width as f64;
        let differs = |clock: f64| width > clock * 1.3 || width < clock * 0.7;
        if !(differs(self.clocks[0]) && differs(self.clocks[1])) {
            return;
        }

        self.clocks.push(width);
        self.clocks.sort_by(|a, b| a.partial_cmp(b).expect("finite pulse widths"));
        self.range1 = (self.clocks[0] + self.clocks[1]) / 2.0;
        self.range2 = (self.clocks[1] + self.clocks[2]) / 2.0;
        let spdif_bitrate = (self.samplerate as f64 / (self.clocks[2] / 1.5)) as i64;
        self.ss_edge = 0;

        self.puty(
            ctx,
            ANN_BITRATE,
            vec![format!(
                "Signal Bitrate: {spdif_bitrate} Mbit/s (=> {} kHz)",
                (spdif_bitrate as f64 / (2.0 * 32.0)) as i64
            )],
        );

        // The Python decoder computes clock_period_nsec here but never
        // uses it.
        self.last_preamble = ctx.samplenum();

        // "We are done recovering the clock, now let's decode the data stream."
        self.state = State::DecodeStream;
    }

    fn decode_stream(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let pulse = self.get_pulse_type();

        if !self.seen_preamble {
            // "This is probably the start of a preamble, decode it."
            if pulse == 2 {
                self.preamble.push(self.get_pulse_type());
                self.state = State::DecodePreamble;
                self.ss_edge = ctx.samplenum() - self.pulse_width - 1;
            }
            return Ok(());
        }

        // "We've seen a preamble."
        if pulse == 1 && self.first_one {
            self.first_one = false;
            self.subframe.push(Pulse {
                kind: pulse,
                ss: ctx.samplenum() - self.pulse_width - 1,
                es: ctx.samplenum(),
            });
        } else if pulse == 1 && !self.first_one {
            // The one bit is two short pulses: extend the previous pulse
            // to this edge and report one bit over both.
            if let Some(last) = self.subframe.last_mut() {
                last.es = ctx.samplenum();
            }
            let ss = self.subframe.last().map_or(0, |p| p.ss);
            ctx.put_ann(ss, ctx.samplenum(), ANN_BITS, vec!["1".into()]);
            self.bitcount += 1;
            self.first_one = true;
        } else {
            self.subframe.push(Pulse {
                kind: pulse,
                ss: ctx.samplenum() - self.pulse_width - 1,
                es: ctx.samplenum(),
            });
            ctx.put_ann(
                ctx.samplenum() - self.pulse_width - 1,
                ctx.samplenum(),
                ANN_BITS,
                vec!["0".into()],
            );
            self.bitcount += 1;
        }

        if self.bitcount == 28 {
            self.report_subframe(ctx)?;
        }

        Ok(())
    }

    fn report_subframe(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.subframe.len() < 28 {
            return Err(Error::arg("incomplete S/PDIF subframe"));
        }
        let aux_audio_data = &self.subframe[0..4];
        let mut sam = String::new();
        let mut sam_rot = String::new();
        for a in aux_audio_data {
            sam.push_str(&a.kind.to_string());
            sam_rot.insert_str(0, &a.kind.to_string());
        }
        let sample = &self.subframe[4..24];
        for s in sample {
            sam.push_str(&s.kind.to_string());
            sam_rot.insert_str(0, &s.kind.to_string());
        }
        let validity = &self.subframe[24..25];
        let subcode_data = &self.subframe[25..26];
        let channel_status = &self.subframe[26..27];
        let parity = &self.subframe[27..28];

        let value = bin_str_to_int(&sam)?;
        let value_rot = bin_str_to_int(&sam_rot)?;

        ctx.put_ann(
            aux_audio_data[0].ss,
            aux_audio_data[3].es,
            ANN_AUX,
            vec![format!("Aux 0x{value:x}"), format!("0x{value:x}")],
        );
        ctx.put_ann(
            sample[0].ss,
            sample[19].es,
            ANN_AUX,
            vec![format!("Sample 0x{value:x}"), format!("0x{value:x}")],
        );
        ctx.put_ann(
            aux_audio_data[0].ss,
            sample[19].es,
            ANN_SAMPLES,
            vec![format!("Audio 0x{value_rot:x}"), format!("0x{value_rot:x}")],
        );
        if validity[0].kind == 0 {
            ctx.put_ann(validity[0].ss, validity[0].es, ANN_VALIDITY, vec!["V".into()]);
        } else {
            ctx.put_ann(validity[0].ss, validity[0].es, ANN_VALIDITY, vec!["E".into()]);
        }
        ctx.put_ann(
            subcode_data[0].ss,
            subcode_data[0].es,
            ANN_SUBCODE,
            vec![format!("S: {}", subcode_data[0].kind)],
        );
        ctx.put_ann(
            channel_status[0].ss,
            channel_status[0].es,
            ANN_CHAN_STAT,
            vec![format!("C: {}", channel_status[0].kind)],
        );
        ctx.put_ann(parity[0].ss, parity[0].es, ANN_PARITY, vec![format!("P: {}", parity[0].kind)]);

        self.subframe.clear();
        self.seen_preamble = false;
        self.bitcount = 0;
        Ok(())
    }

    fn decode_preamble(&mut self, ctx: &mut dyn Context) {
        if self.preamble_state == 0 {
            self.preamble.push(self.get_pulse_type());
            self.preamble_state = 1;
        } else if self.preamble_state == 1 {
            self.preamble.push(self.get_pulse_type());
            self.preamble_state = 2;
        } else {
            self.preamble.push(self.get_pulse_type());
            self.preamble_state = 0;
            self.state = State::DecodeStream;
            let texts: &[&str] = if self.preamble == [2, 0, 1, 0] {
                &["Preamble W", "W"]
            } else if self.preamble == [2, 2, 1, 1] {
                &["Preamble M", "M"]
            } else if self.preamble == [2, 1, 1, 2] {
                &["Preamble B", "B"]
            } else {
                &["Unknown Preamble", "Unknown Prea.", "U"]
            };
            self.puty(ctx, ANN_PREAMBLE, texts.iter().map(|s| (*s).to_owned()).collect());
            self.preamble.clear();
            self.seen_preamble = true;
            self.bitcount = 0;
            self.first_one = true;
        }

        self.last_preamble = ctx.samplenum();
    }
}

/// `int(s, 2)` for a string of pulse types; Python raises `ValueError` on
/// anything but binary digits, which ends the decoder, too.
fn bin_str_to_int(s: &str) -> Result<i64> {
    i64::from_str_radix(s, 2).map_err(|_| Error::arg("invalid binary subframe"))
}

impl Decoder for Spdif {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.samplerate = 0;
        self.state = State::GetFirstPulseWidth;
        self.ss_edge = 0;
        self.samplenum_prev_edge = 0;
        self.pulse_width = 0;
        self.clocks.clear();
        self.range1 = 0.0;
        self.range2 = 0.0;
        self.preamble_state = 0;
        self.preamble.clear();
        self.seen_preamble = false;
        self.last_preamble = 0;
        self.first_one = true;
        self.subframe.clear();
        self.bitcount = 0;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        // "Throw away first detected edge as it might be mangled data."
        ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;

        loop {
            // "Wait for any edge (rising or falling)."
            ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
            self.pulse_width = ctx.samplenum() - self.samplenum_prev_edge - 1;
            self.samplenum_prev_edge = ctx.samplenum();

            match self.state {
                State::GetFirstPulseWidth => self.find_first_pulse_width(),
                State::GetSecondPulseWidth => self.find_second_pulse_width(),
                State::GetThirdPulseWidth => self.find_third_pulse_width(ctx),
                State::DecodeStream => self.decode_stream(ctx)?,
                State::DecodePreamble => self.decode_preamble(ctx),
            }
        }
    }
}
