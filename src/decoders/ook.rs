// Counterpart in sigrok:
//   libsigrokdecode/decoders/ook/pd.py,
//   Copyright (C) 2018 Steve R <steversig@virginmedia.com>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the on-off keying decoder's rules and documented
//   classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - A timeout is `{'skip': 5 * self.sample_first}`: no edge for five
//     preamble periods. On the timeout sample the pin level is inverted
//     (`pinstate = int(not pinstate)`) before it becomes a state, and
//     `invert` is applied after that.
//   - The preamble filter keeps `preamlen` pulses; a pulse five times
//     longer or shorter than the previous one clears the buffer and
//     emits the 'R' reset annotation. `sample_high`/`sample_low` come
//     from the first two pulses and are refined from the third on when
//     a pulse differs by more than 25% from the one two back.
//   - Manchester is decoded twice at once (a 1111 preamble with twice
//     the sample width and a 1010 preamble with the measured one); the
//     chosen row and stream is the 1010 one when it has fewer errors,
//     except that an explicit preamble option picks its own stream.
//   - `preamlen` is the string '0'; with it the filter never reaches
//     `len(preamble) == 0` and the decoder never enters decoding. The
//     same comparison is kept.
//   - Python `round()` is round-half-to-even, which decides the relative
//     pulse length of the differential Manchester decoder.
//   - The error texts are "Probably not Diff Manchester encoded" and
//     "Not Manchester encoded or wrong preamble"; either is emitted
//     when the error count reaches max_errors (5). The timeout marker
//     'T' is annotation class 0.

//! OOK: on-off keying preamble recovery, NRZ, Manchester and differential
//! Manchester decoding.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_FRAME: usize = 0;
const ANN_INFO: usize = 1;
const ANN_MAN1111: usize = 2;
const ANN_MAN1010: usize = 3;
const ANN_DIFFMAN: usize = 4;
const ANN_NRZ: usize = 5;
const BIN_PULSE_LENGTHS: usize = 0;

static OPTIONS: [OptionDesc; 5] = [
    OptionDesc {
        id: "invert",
        desc: "Invert data",
        default: OptionValue::s("no"),
        values: &[OptionValue::s("no"), OptionValue::s("yes")],
    },
    OptionDesc {
        id: "decodeas",
        desc: "Decode type",
        default: OptionValue::s("Manchester"),
        values: &[
            OptionValue::s("NRZ"),
            OptionValue::s("Manchester"),
            OptionValue::s("Diff Manchester"),
        ],
    },
    OptionDesc {
        id: "preamble",
        desc: "Preamble",
        default: OptionValue::s("auto"),
        values: &[OptionValue::s("auto"), OptionValue::s("1010"), OptionValue::s("1111")],
    },
    OptionDesc {
        id: "preamlen",
        desc: "Filter length",
        default: OptionValue::s("7"),
        values: &[
            OptionValue::s("0"),
            OptionValue::s("3"),
            OptionValue::s("4"),
            OptionValue::s("5"),
            OptionValue::s("6"),
            OptionValue::s("7"),
            OptionValue::s("8"),
            OptionValue::s("9"),
            OptionValue::s("10"),
        ],
    },
    OptionDesc {
        id: "diffmanvar",
        desc: "Transition at start",
        default: OptionValue::s("1"),
        values: &[OptionValue::s("1"), OptionValue::s("0")],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ook",
    name: "OOK",
    longname: "On-off keying",
    desc: "On-off keying protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["ook"],
    tags: &["Encoding"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "frame", desc: "Frame" },
        AnnClass { id: "info", desc: "Info" },
        AnnClass { id: "1111", desc: "1111" },
        AnnClass { id: "1010", desc: "1010" },
        AnnClass { id: "diffman", desc: "Diff Man" },
        AnnClass { id: "nrz", desc: "NRZ" },
    ],
    annotation_rows: &[
        AnnRow { id: "frame", desc: "Framing", classes: &[0] },
        AnnRow { id: "info", desc: "Info", classes: &[1] },
        AnnRow { id: "man1111", desc: "Man 1111", classes: &[2] },
        AnnRow { id: "man1010", desc: "Man 1010", classes: &[3] },
        AnnRow { id: "diffman", desc: "Diff Man", classes: &[4] },
        AnnRow { id: "nrz", desc: "NRZ", classes: &[5] },
    ],
    binary: &[BinClass { id: "pulse-lengths", desc: "Pulse lengths" }],
    doc: "OOK decodes On-off keying based remote control protocols.\n\
          \n\
          It is aimed at 433MHz but should also work with other common RC \
          frequencies. The input can be captured directly from a transmitter \
          (before the modulation stage) or demodulated by an RF receiver.\n\
          \n\
          Over the air captured traces will be a lot noisier and will probably \
          need the area of interest to be zoomed onto, then selected with the \
          \"Cursors\" and the \"Save Selected Range As\" feature to be used to \
          extract it from the noise.\n\
          \n\
          There is a limited amount of pre-filtering and garbage removal built \
          into the decoder which can sometimes extract signals directly from a \
          larger over the air trace. It depends heavily on your environment.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ook::new())
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Decoding,
    WaitingForPreamble,
    DecodeTimeout,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DecodeAs {
    Nrz,
    Manchester,
    DiffManchester,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Preamble {
    Auto,
    Man1010,
    Man1111,
}

#[derive(Clone, Copy)]
struct PreamblePulse {
    ss: u64,
    samples: u64,
    state: char,
}

#[derive(Clone, Copy)]
struct Decoded {
    ss: i64,
    es: i64,
    state: char,
}

/// Python's `round()`, which rounds exact halves to even.
#[allow(clippy::float_cmp)]
fn py_round(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let t = x.trunc();
    if (x - t).abs() == 0.5 { if t % 2.0 == 0.0 { t } else { t + x.signum() } } else { x.round() }
}

/// Python truthiness of `self.sample_first` / `self.samplenumber_last`:
/// 0 counts as unset.
fn truthy(v: Option<u64>) -> bool {
    v.is_some_and(|n| n != 0)
}

struct Ook {
    samplerate: u64,
    ss: i64,
    es: i64,
    ss_1111: i64,
    ss_1010: i64,
    samplenumber_last: Option<u64>,
    sample_first: Option<u64>,
    sample_high: i64,
    sample_low: i64,
    edge_count: usize,
    word_first: Option<u64>,
    state: State,
    lstate: Option<char>,
    lstate_1010: Option<char>,
    insync: bool,
    man_errors: usize,
    man_errors_1010: usize,
    preamble: Vec<PreamblePulse>,
    half_time: i64,
    half_time_1010: i64,
    pulse_lengths: Vec<u64>,
    decoded: Vec<Decoded>,
    decoded_1010: Vec<Decoded>,
    diff_man_trans: char,
    diff_man_len: i64,
    max_errors: usize,
    invert: bool,
    decodeas: DecodeAs,
    preamble_val: Preamble,
    preamble_len: u32,
    diffmanvar: char,
}

impl Ook {
    fn new() -> Ook {
        Ook {
            samplerate: 0,
            ss: -1,
            es: -1,
            ss_1111: -1,
            ss_1010: -1,
            samplenumber_last: None,
            sample_first: None,
            sample_high: 0,
            sample_low: 0,
            edge_count: 0,
            word_first: None,
            state: State::Idle,
            lstate: None,
            lstate_1010: None,
            insync: false,
            man_errors: 0,
            man_errors_1010: 0,
            preamble: Vec::new(),
            half_time: -1,
            half_time_1010: 0,
            pulse_lengths: Vec::new(),
            decoded: Vec::new(),
            decoded_1010: Vec::new(),
            diff_man_trans: '0',
            diff_man_len: 1,
            max_errors: 5,
            invert: false,
            decodeas: DecodeAs::Manchester,
            preamble_val: Preamble::Auto,
            preamble_len: 7,
            diffmanvar: '1',
        }
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss.max(0) as u64, self.es.max(0) as u64, class, texts);
    }

    fn putx1(&self, ctx: &mut dyn Context, class: usize, text: &str) {
        self.putx(ctx, class, vec![text.to_owned()]);
    }

    fn putp(&self, ctx: &mut dyn Context, decoded: &[Decoded]) {
        let items: Vec<Item> = decoded
            .iter()
            .map(|d| {
                Item::List(vec![Item::Int(d.ss), Item::Int(d.es), Item::Str(d.state.to_string())])
            })
            .collect();
        ctx.put_python(self.ss.max(0) as u64, self.es.max(0) as u64, Item::List(items));
    }

    /// pd.py `dump_pulse_lengths()`: the ascii pulse lengths in µs.
    fn dump_pulse_lengths(&self, ctx: &mut dyn Context) {
        if self.samplerate == 0 {
            return;
        }
        // Fix final pulse length.
        let mut lengths = self.pulse_lengths.clone();
        if let Some(last) = lengths.last_mut() {
            *last = self.sample_first.unwrap_or(0);
        }
        let s = format!(
            "Pulses(us)={}\n",
            lengths
                .iter()
                .map(|&x| ((x * 1_000_000) / self.samplerate).to_string())
                .collect::<Vec<_>>()
                .join(",")
        );
        let end = ctx.samplenum();
        ctx.put_binary(end.saturating_sub(10), end, BIN_PULSE_LENGTHS, s.into_bytes());
    }

    /// pd.py `decode_nrz()`.
    fn decode_nrz(
        &mut self,
        ctx: &mut dyn Context,
        start: u64,
        samples: u64,
        state: char,
    ) -> Result<()> {
        self.pulse_lengths.push(samples);
        // Use different high and low widths to compensate skewed waveforms.
        let dsamples = if state == '1' { self.sample_high } else { self.sample_low };
        if dsamples <= 0 {
            self.edge_count += 1;
            return Ok(());
        }
        self.ss = start as i64;
        self.es = start as i64 + samples as i64;
        let mut samples = samples as i64;
        while samples as f64 > dsamples as f64 * 0.5 {
            if samples as f64 >= dsamples as f64 * 1.5 {
                // More than one bit.
                self.es = self.ss + dsamples;
                self.putx1(ctx, ANN_NRZ, &state.to_string());
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state });
                self.edge_count += 1;
            } else if samples as f64 >= dsamples as f64 * 0.5
                && (samples as f64) < dsamples as f64 * 1.5
            {
                // Last bit.
                self.putx1(ctx, ANN_NRZ, &state.to_string());
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state });
                self.edge_count += 1;
            } else {
                self.edge_count += 1;
            }
            samples -= dsamples;
            self.ss += dsamples;
            self.es += dsamples;

            // Ensure 2nd row doesn't go past end of 1st row.
            if self.es > ctx.samplenum() as i64 {
                self.es = ctx.samplenum() as i64;
            }

            if self.state == State::DecodeTimeout {
                // Five bits - reset.
                if let (Some(first), Some(last)) = (self.decoded.first(), self.decoded.last()) {
                    self.ss = first.ss;
                    self.es = last.es;
                }
                self.dump_pulse_lengths(ctx);
                let decoded = self.decoded.clone();
                self.putp(ctx, &decoded);
                self.decode_timeout(ctx)?;
                break;
            }
        }
        Ok(())
    }

    /// pd.py `lock_onto_preamble()`, filters and recovers the clock.
    fn lock_onto_preamble(
        &mut self,
        ctx: &mut dyn Context,
        _samples: u64,
        state: char,
    ) -> Result<()> {
        self.edge_count += 1;
        let l2s: u64 = 5; // Max ratio of long to short pulses.

        // Filter incoming pulses to remove random noise.
        if self.state == State::DecodeTimeout {
            self.preamble.clear();
            self.edge_count = 0;
            self.word_first = Some(ctx.samplenum());
            self.sample_first =
                Some(ctx.samplenum() - self.samplenumber_last.unwrap_or(ctx.samplenum()));
            self.state = State::WaitingForPreamble;
            self.man_errors = 0;
        }

        let pre_detect = self.preamble_len as usize; // Number of valid pulses.
        let pre_samples = ctx.samplenum() - self.samplenumber_last.unwrap_or(ctx.samplenum());
        let last_snum = self.samplenumber_last.unwrap_or(ctx.samplenum());
        if let Some(last) = self.preamble.last() {
            if pre_samples * l2s < last.samples || last.samples * l2s < pre_samples {
                // Garbage in.
                let s = ctx.samplenum();
                ctx.put_ann(s, s, ANN_FRAME, vec!["R".into()]); // Display resets.
                self.preamble.clear(); // Clear buffer.
                self.preamble.push(PreamblePulse { ss: last_snum, samples: pre_samples, state });
                self.edge_count = 0;
                self.samplenumber_last = Some(ctx.samplenum());
                self.word_first = Some(ctx.samplenum());
            } else {
                self.preamble.push(PreamblePulse { ss: last_snum, samples: pre_samples, state });
            }
        } else {
            self.preamble.push(PreamblePulse { ss: last_snum, samples: pre_samples, state });
        }

        if self.preamble.len() == pre_detect {
            // Have a valid series of pulses.
            let pre = self.preamble.clone();
            if pre[0].state == '1' {
                self.sample_high = pre[0].samples as i64; // Allows skewed pulses.
                self.sample_low = pre[1].samples as i64;
            } else {
                self.sample_high = pre[1].samples as i64;
                self.sample_low = pre[0].samples as i64;
            }

            self.edge_count = 0;

            for i in 0..pre.len() {
                if i > 1 {
                    // Adjust ref width.
                    if (pre[i].samples as f64) > pre[i - 2].samples as f64 * 1.25
                        || (pre[i].samples as f64) * 1.25 < pre[i - 2].samples as f64
                    {
                        if pre[i].state == '1' {
                            self.sample_high = pre[i].samples as i64;
                        } else {
                            self.sample_low = pre[i].samples as i64;
                        }
                    }
                }

                // Display start of preamble.
                match self.decodeas {
                    DecodeAs::Nrz => {
                        self.decode_nrz(ctx, pre[i].ss, pre[i].samples, pre[i].state)?;
                    }
                    DecodeAs::Manchester => {
                        self.decode_manchester(ctx, pre[i].ss, pre[i].samples, pre[i].state)?;
                    }
                    DecodeAs::DiffManchester => {
                        self.es = pre[i].ss as i64 + pre[i].samples as i64;
                        self.decode_diff_manchester(ctx, pre[i].ss, pre[i].samples, pre[i].state)?;
                    }
                }

                // Used to timeout signal.
                self.sample_first = Some(i64::midpoint(self.sample_high, self.sample_low) as u64);
            }
            self.insync = true;
            self.state = State::Decoding;
        }
        self.lstate = Some(state);
        self.lstate_1010 = Some(state);
        Ok(())
    }

    /// pd.py `decode_diff_manchester()`.
    fn decode_diff_manchester(
        &mut self,
        ctx: &mut dyn Context,
        start: u64,
        samples: u64,
        state: char,
    ) -> Result<()> {
        self.pulse_lengths.push(samples);

        // Use different high and low widths to compensate skewed waveforms.
        let dsamples = if state == '1' { self.sample_high } else { self.sample_low };

        self.es = start as i64 + samples as i64;
        let p_length =
            if dsamples > 0 { py_round(samples as f64 / dsamples as f64) as i64 } else { 1 }; // Find relative pulse length.

        if self.edge_count == 0 {
            self.diff_man_trans = '1'; // Very first pulse must be a transition.
            self.diff_man_len = 1; // Must also be a half pulse.
            self.ss = start as i64;
        } else if self.edge_count % 2 == 1 {
            // Time to make a decision.
            if self.diffmanvar == '0' {
                // Transition at self.ss is a zero.
                self.diff_man_trans = if self.diff_man_trans == '1' { '0' } else { '1' };
            }
            if self.diff_man_len == 1 && p_length == 1 {
                self.putx1(ctx, ANN_DIFFMAN, &self.diff_man_trans.to_string());
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state: self.diff_man_trans });
                self.diff_man_trans = '1';
            } else if self.diff_man_len == 1 && p_length == 2 {
                self.es -= (samples / 2) as i64;
                self.putx1(ctx, ANN_DIFFMAN, &self.diff_man_trans.to_string());
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state: self.diff_man_trans });
                self.diff_man_trans = '0';
                self.edge_count += 1; // Add a virt edge to keep in sync with clk.
            } else if self.diff_man_len == 2 && p_length == 1 {
                self.putx1(ctx, ANN_DIFFMAN, &self.diff_man_trans.to_string());
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state: self.diff_man_trans });
                self.diff_man_trans = '1';
            } else if self.diff_man_len == 2 && p_length == 2 {
                // Double illegal E E.
                self.es -= samples as i64;
                self.putx1(ctx, ANN_DIFFMAN, "E");
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state: 'E' });
                self.ss = self.es;
                self.es += samples as i64;
                self.putx1(ctx, ANN_DIFFMAN, "E");
                self.decoded.push(Decoded { ss: self.ss, es: self.es, state: 'E' });
                self.diff_man_trans = '1';
            } else if self.diff_man_len == 1 && p_length > 4 {
                if self.state == State::DecodeTimeout {
                    self.es = self.ss + 2 * self.sample_first.unwrap_or(0) as i64;
                    // Write error.
                    self.putx1(ctx, ANN_DIFFMAN, &self.diff_man_trans.to_string());
                    self.decoded.push(Decoded {
                        ss: self.ss,
                        es: self.es,
                        state: self.diff_man_trans,
                    });
                    if let (Some(first), Some(last)) = (self.decoded.first(), self.decoded.last()) {
                        self.ss = first.ss;
                        self.es = last.es;
                    }
                    self.dump_pulse_lengths(ctx);
                    if self.man_errors < self.max_errors {
                        let decoded = self.decoded.clone();
                        self.putp(ctx, &decoded);
                    } else {
                        let error_message = "Probably not Diff Manchester encoded";
                        self.ss = self.word_first.unwrap_or(0) as i64;
                        self.putx1(ctx, ANN_INFO, error_message);
                    }
                    self.decode_timeout(ctx)?;
                }
                self.diff_man_trans = '1';
            }
            self.ss = self.es;
        }
        self.diff_man_len = p_length; // Save the previous length.
        self.edge_count += 1;
        Ok(())
    }

    /// pd.py `decode_manchester_sim()`, one of the two parallel decoders.
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    fn decode_manchester_sim(
        &self,
        start: u64,
        samples: u64,
        state: char,
        dsamples: i64,
        half_time: i64,
        lstate: Option<char>,
        ss: i64,
        pream: &str,
    ) -> (i64, Option<char>, i64, Option<Decoded>, usize) {
        let mut ook_bit: Option<Decoded> = None;
        let mut errors = 0;
        let mut half_time = half_time;
        let mut lstate = lstate;
        let mut ss = ss;
        if self.edge_count == 0 {
            half_time += 1;
        }
        let samples_f = samples as f64;
        let dsamples_f = dsamples as f64;
        if samples_f > 0.75 * dsamples_f && samples_f <= 1.5 * dsamples_f {
            // Long p.
            half_time += 2;
            let mut es = if half_time.rem_euclid(2) == 0 {
                start as i64
            } else {
                start as i64 + (samples / 2) as i64
            };
            if ss == start as i64 {
                lstate = Some('E');
                es = start as i64 + samples as i64;
            }
            if !(self.edge_count == 0 && pream == "1010") {
                // Skip first p.
                ook_bit = Some(Decoded { ss, es, state: lstate.unwrap_or('E') });
            }
            lstate = Some(state);
            ss = es;
        } else if samples_f > 0.25 * dsamples_f && samples_f <= 0.75 * dsamples_f {
            // Short p.
            half_time += 1;
            if half_time.rem_euclid(2) == 0 {
                // Transition.
                let es = start as i64 + samples as i64;
                ook_bit = Some(Decoded { ss, es, state: lstate.unwrap_or('E') });
                lstate = Some(state);
                ss = es;
            } else {
                // 1st half.
                ss = start as i64;
                lstate = Some(state);
            }
        } else {
            // Too long or too short - error.
            errors = 1;
            let es;
            if self.state == State::DecodeTimeout {
                // Assume final half bit buried in timeout pulse.
                es = ss + self.sample_first.unwrap_or(0) as i64;
            } else {
                // Error condition.
                lstate = Some('E');
                es = ss + samples as i64;
            }
            ook_bit = Some(Decoded { ss, es, state: lstate.unwrap_or('E') });
            ss = es;
        }

        (half_time, lstate, ss, ook_bit, errors)
    }

    /// pd.py `decode_manchester()`.
    fn decode_manchester(
        &mut self,
        ctx: &mut dyn Context,
        start: u64,
        samples: u64,
        state: char,
    ) -> Result<()> {
        self.pulse_lengths.push(samples);

        // Use different high and low widths to compensate skewed waveforms.
        let dsamples = if state == '1' { self.sample_high } else { self.sample_low };

        let mut ook_bit: Option<Decoded> = None;
        if self.preamble_val != Preamble::Man1010 {
            // 1111 preamble is half clock T.
            let (half_time, lstate, ss, ob, errors) = self.decode_manchester_sim(
                start,
                samples,
                state,
                dsamples * 2,
                self.half_time,
                self.lstate,
                self.ss_1111,
                "1111",
            );
            self.half_time = half_time;
            self.lstate = lstate;
            self.ss_1111 = ss;
            self.man_errors += errors;
            if let Some(bit) = ob {
                self.decoded.push(bit);
            }
            ook_bit = ob;
        }

        if self.preamble_val != Preamble::Man1111 {
            // 1010 preamble is clock T.
            let (half_time, lstate, ss, ob, errors) = self.decode_manchester_sim(
                start,
                samples,
                state,
                dsamples,
                self.half_time_1010,
                self.lstate_1010,
                self.ss_1010,
                "1010",
            );
            self.half_time_1010 = half_time;
            self.lstate_1010 = lstate;
            self.ss_1010 = ss;
            self.man_errors_1010 += errors;
            if let Some(bit) = ob {
                self.decoded_1010.push(bit);
            }
            ook_bit = ob;
        }

        self.edge_count += 1;

        // Stream display and save ook_bit.
        if let Some(bit) = ook_bit {
            self.ss = bit.ss;
            self.es = bit.es;
            if self.preamble_val == Preamble::Man1111 {
                self.putx1(ctx, ANN_MAN1111, &bit.state.to_string());
            }
            if self.preamble_val == Preamble::Man1010 {
                self.putx1(ctx, ANN_MAN1010, &bit.state.to_string());
            }
        }

        if self.state == State::DecodeTimeout {
            // End of packet.
            self.dump_pulse_lengths(ctx);

            let (decoded, man_errors, d_row);
            // If 1010 preamble has less errors use it.
            if self.preamble_val == Preamble::Man1010
                || (self.man_errors_1010 < self.max_errors
                    && self.man_errors_1010 < self.man_errors
                    && !self.decoded_1010.is_empty())
            {
                decoded = self.decoded_1010.clone();
                man_errors = self.man_errors_1010;
                d_row = ANN_MAN1010;
            } else {
                decoded = self.decoded.clone();
                man_errors = self.man_errors;
                d_row = ANN_MAN1111;
            }

            if self.preamble_val == Preamble::Auto {
                // Display OOK packet.
                for bit in &decoded {
                    self.ss = bit.ss;
                    self.es = bit.es;
                    self.putx1(ctx, d_row, &bit.state.to_string());
                }
            }

            if man_errors < self.max_errors && !decoded.is_empty() {
                self.ss = decoded[0].ss;
                self.es = decoded[decoded.len() - 1].es;
                self.putp(ctx, &decoded);
            } else {
                let error_message = "Not Manchester encoded or wrong preamble";
                self.ss = self.word_first.unwrap_or(0) as i64;
                self.putx1(ctx, ANN_INFO, error_message);
            }

            let es = self.es.max(0) as u64;
            // Mark timeout.
            ctx.put_ann(es, es, ANN_FRAME, vec!["T".into()]);
            self.decode_timeout(ctx)?;
        }

        Ok(())
    }

    /// pd.py `decode_timeout()`: get rid of the long pulse and reset.
    fn decode_timeout(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.samplenumber_last = None;
        self.edge_count = 0;
        self.man_errors = 0; // Clear the bit error counters.
        self.man_errors_1010 = 0;
        self.state = State::Idle;
        ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?; // Get rid of long pulse.
        self.samplenumber_last = Some(ctx.samplenum());
        self.word_first = Some(ctx.samplenum());
        self.insync = false; // Preamble in sync flag
        self.preamble.clear(); // Preamble buffer
        self.half_time = -1; // Half time for man 1111
        self.half_time_1010 = 0; // Half time for man 1010
        self.decoded.clear();
        self.decoded_1010.clear();
        self.pulse_lengths.clear();
        Ok(())
    }
}

impl Decoder for Ook {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.invert = config.str("invert", "no") == "yes";
        self.decodeas = match config.str("decodeas", "Manchester") {
            "NRZ" => DecodeAs::Nrz,
            "Diff Manchester" => DecodeAs::DiffManchester,
            _ => DecodeAs::Manchester,
        };
        self.preamble_val = match config.str("preamble", "auto") {
            "1010" => Preamble::Man1010,
            "1111" => Preamble::Man1111,
            _ => Preamble::Auto,
        };
        self.preamble_len = config.str("preamlen", "7").parse().unwrap_or(7);
        self.diffmanvar = config.str("diffmanvar", "1").chars().next().unwrap_or('1');
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    #[allow(clippy::too_many_lines)]
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            let pins;
            if self.edge_count == 0 {
                // Waiting for a signal.
                pins = ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)]])?;
                self.state = State::Decoding;
            } else {
                let skip = 5 * self.sample_first.unwrap_or(0);
                pins = ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)], &[Cond::Skip(skip)]])?;
                // No edges for 5 p's.
                if ctx.matched()[1] && !ctx.matched()[0] {
                    self.state = State::DecodeTimeout;
                }
            }

            if !truthy(self.samplenumber_last) {
                // Set counters to start of signal.
                self.samplenumber_last = Some(ctx.samplenum());
                self.word_first = Some(ctx.samplenum());
                continue;
            }
            let samples = ctx.samplenum() - self.samplenumber_last.unwrap_or(ctx.samplenum());
            if !truthy(self.sample_first) {
                // Get number of samples for first pulse.
                self.sample_first = Some(samples);
            }

            let mut pinstate = pins.0[DATA] == 1;
            if self.state == State::DecodeTimeout {
                // No edge so flip the state.
                pinstate = !pinstate;
            }
            if self.invert {
                // Invert signal.
                pinstate = !pinstate;
            }
            let state = if pinstate { '0' } else { '1' };

            // No preamble filtering or checking and no skew correction.
            if self.preamble_len == 0 {
                self.sample_high = self.sample_first.unwrap_or(0) as i64;
                self.sample_low = self.sample_first.unwrap_or(0) as i64;
                self.insync = false;
            }

            if self.insync {
                match self.decodeas {
                    DecodeAs::Nrz => {
                        self.decode_nrz(ctx, self.samplenumber_last.unwrap_or(0), samples, state)?;
                    }
                    DecodeAs::Manchester => self.decode_manchester(
                        ctx,
                        self.samplenumber_last.unwrap_or(0),
                        samples,
                        state,
                    )?,
                    DecodeAs::DiffManchester => self.decode_diff_manchester(
                        ctx,
                        self.samplenumber_last.unwrap_or(0),
                        samples,
                        state,
                    )?,
                }
            } else {
                self.lock_onto_preamble(ctx, samples, state)?;
            }

            self.samplenumber_last = Some(ctx.samplenum());
        }
    }
}
