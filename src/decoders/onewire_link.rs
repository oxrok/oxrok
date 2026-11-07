// Counterpart in sigrok:
//   libsigrokdecode/decoders/onewire_link/pd.py,
//   Copyright (C) 2017 Kevin Redon <kingkevin@cuvoodoo.info>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the 1-Wire link layer and the decoder's documented
//   classes and texts.

//! 1-Wire link layer: reset/presence pulses and read/write time slots.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const OWR: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_WARN: usize = 1;
const ANN_RESET: usize = 2;
const ANN_PRESENCE: usize = 3;
const ANN_OVERDRIVE: usize = 4;

// Timing values in us for the signal at regular and overdrive speed,
// with the regular value first, as pd.py indexes them by its
// `self.overdrive` bool.
const RSTL_MIN: [f64; 2] = [480.0, 48.0];
const RSTL_MAX: [f64; 2] = [960.0, 80.0];
const RSTH_MIN: [f64; 2] = [480.0, 48.0];
const PDH_MIN: [f64; 2] = [15.0, 2.0];
const PDH_MAX: [f64; 2] = [60.0, 6.0];
const PDL_MIN: [f64; 2] = [60.0, 8.0];
const PDL_MAX: [f64; 2] = [240.0, 24.0];
const SLOT_MIN: [f64; 2] = [60.0, 6.0];
const SLOT_MAX: [f64; 2] = [120.0, 16.0];
const REC_MIN: [f64; 2] = [1.0, 1.0];
const LOWR_MIN: [f64; 2] = [1.0, 1.0];
const LOWR_MAX: [f64; 2] = [15.0, 2.0];

// The timing texts use Python's str() of a float, so they keep ".0".
const RSTL_MAX_TEXT: [&str; 2] = ["RST > 960.0", "RST > 80.0"];
const PDH_MIN_TEXT: [&str; 2] = ["PDH < 15.0", "PDH < 2.0"];
const PDL_MIN_TEXT: [&str; 2] = ["PDL < 60.0", "PDL < 8.0"];
const PDL_MAX_TEXT: [&str; 2] = ["PDL > 240.0", "PDL > 24.0"];
const SLOT_MIN_TEXT: [&str; 2] = ["SLOT < 60.0", "SLOT < 6.0"];
const RSTH_MIN_TEXT: [&str; 2] = ["RTSH < 480.0", "RTSH < 48.0"];

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "overdrive",
    desc: "Start in overdrive speed",
    default: OptionValue::s("no"),
    values: &[OptionValue::s("yes"), OptionValue::s("no")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "onewire_link",
    name: "1-Wire link layer",
    longname: "1-Wire serial communication bus (link layer)",
    desc: "Bidirectional, half-duplex, asynchronous serial bus.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["onewire_link"],
    tags: &["Embedded/industrial"],
    channels: &[ChannelDesc { id: "owr", name: "OWR", desc: "1-Wire signal line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "warnings", desc: "Warnings" },
        AnnClass { id: "reset", desc: "Reset" },
        AnnClass { id: "presence", desc: "Presence" },
        AnnClass { id: "overdrive", desc: "Overdrive speed notifications" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 2, 3] },
        AnnRow { id: "info", desc: "Info", classes: &[4] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[1] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(OnewireLink::default())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Initial,
    Idle,
    Low,
    PresenceDetectHigh,
    PresenceDetectLow,
    Slot,
    PresenceDetect,
}

fn ann_texts(texts: &[&str]) -> Vec<String> {
    texts.iter().map(|t| (*t).to_owned()).collect()
}

/// The 1-Wire link layer decoder.
pub struct OnewireLink {
    samplerate: u64,
    state: State,
    bit: i64,
    bit_count: i64,
    command: u64,
    overdrive: bool,
    fall: u64,
    rise: u64,
}

impl Default for OnewireLink {
    fn default() -> Self {
        OnewireLink {
            samplerate: 0,
            state: State::Initial,
            bit: 0,
            bit_count: -1,
            command: 0,
            overdrive: false,
            fall: 0,
            rise: 0,
        }
    }
}

impl OnewireLink {
    fn od(&self) -> usize {
        usize::from(self.overdrive)
    }

    fn putm(ctx: &mut dyn Context, texts: &[&str]) {
        ctx.put_ann(0, 0, ANN_WARN, ann_texts(texts));
    }

    fn putpfs(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.fall, ctx.samplenum(), item);
    }

    fn putfs(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        ctx.put_ann(self.fall, ctx.samplenum(), class, ann_texts(texts));
    }

    fn putfr(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        ctx.put_ann(self.fall, self.rise, class, ann_texts(texts));
    }

    fn putprs(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.rise, ctx.samplenum(), item);
    }

    fn putrs(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        ctx.put_ann(self.rise, ctx.samplenum(), class, ann_texts(texts));
    }

    fn checks(&self, ctx: &mut dyn Context) {
        // Same thresholds and texts as pd.py's checks().
        if self.overdrive {
            if self.samplerate < 2_000_000 {
                Self::putm(
                    ctx,
                    &[
                        "Sampling rate is too low. Must be above 2MHz for proper overdrive mode decoding.",
                    ],
                );
            } else if self.samplerate < 5_000_000 {
                Self::putm(
                    ctx,
                    &[
                        "Sampling rate is suggested to be above 5MHz for proper overdrive mode decoding.",
                    ],
                );
            }
        } else if self.samplerate < 400_000 {
            Self::putm(
                ctx,
                &[
                    "Sampling rate is too low. Must be above 400kHz for proper normal mode decoding.",
                ],
            );
        } else if self.samplerate < 1_000_000 {
            Self::putm(
                ctx,
                &["Sampling rate is suggested to be above 1MHz for proper normal mode decoding."],
            );
        }
    }

    /// The us value as the packet's sample count, truncated as Python's `int()`.
    fn samples(&self, us: f64) -> i64 {
        ((us / 1_000_000.0) * self.samplerate as f64) as i64
    }

    /// Wait for a falling edge and/or the timeout `us` after `start`.
    fn wait_falling_timeout(&self, ctx: &mut dyn Context, start: u64, us: f64) -> Result<()> {
        let cnt = self.samples(us);
        let samples_to_skip = (start as i64 + cnt - ctx.samplenum() as i64).max(0);
        ctx.wait(&[&[Cond::Pin(OWR, CondKind::Falling)], &[Cond::Skip(samples_to_skip as u64)]])?;
        Ok(())
    }

    /// us between two sample numbers, as pd.py computes it.
    fn elapsed(&self, from: u64, to: u64) -> f64 {
        ((to as f64 - from as f64) / self.samplerate as f64) * 1_000_000.0
    }
}

impl Decoder for OnewireLink {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.overdrive = config.str("overdrive", "no") == "yes";
        self.state = State::Initial;
        self.bit = 0;
        self.bit_count = -1;
        self.command = 0;
        self.fall = 0;
        self.rise = 0;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    #[allow(clippy::too_many_lines)]
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }
        self.checks(ctx);
        loop {
            // State machine.
            match self.state {
                State::Initial => {
                    // Wait until we reach the idle high state.
                    ctx.wait(&[&[Cond::Pin(OWR, CondKind::High)]])?;
                    self.rise = ctx.samplenum();
                    self.state = State::Idle;
                }
                State::Idle => {
                    // Wait for falling edge.
                    ctx.wait(&[&[Cond::Pin(OWR, CondKind::Falling)]])?;
                    self.fall = ctx.samplenum();
                    // Get time since last rising edge.
                    let time = self.elapsed(self.rise, self.fall);
                    if self.rise > 0 && time < REC_MIN[self.od()] {
                        // pd.py concatenates the first two literals with no
                        // separator, so the first text has no space.
                        self.putfr(
                            ctx,
                            ANN_WARN,
                            &["Recovery time not long enoughRecovery too short", "REC < 1.0"],
                        );
                    }
                    // A reset pulse or slot can start on a falling edge.
                    self.state = State::Low;
                }
                State::Low => {
                    // Reset pulse or slot: wait for rising edge.
                    ctx.wait(&[&[Cond::Pin(OWR, CondKind::Rising)]])?;
                    self.rise = ctx.samplenum();
                    // Detect reset or slot based on timing.
                    let time = self.elapsed(self.fall, self.rise);
                    if time >= RSTL_MIN[0] {
                        // Normal reset pulse.
                        if time > RSTL_MAX[0] {
                            self.putfr(
                                ctx,
                                ANN_WARN,
                                &[
                                    "Too long reset pulse might mask interrupt signalling by other devices",
                                    "Reset pulse too long",
                                    RSTL_MAX_TEXT[0],
                                ],
                            );
                        }
                        // Regular reset pulse clears overdrive speed.
                        if self.overdrive {
                            self.putfr(
                                ctx,
                                ANN_OVERDRIVE,
                                &["Exiting overdrive mode", "Overdrive off"],
                            );
                        }
                        self.overdrive = false;
                        self.putfr(ctx, ANN_RESET, &["Reset", "Rst", "R"]);
                        self.state = State::PresenceDetectHigh;
                    } else if self.overdrive
                        && time >= RSTL_MIN[self.od()]
                        && time < RSTL_MAX[self.od()]
                    {
                        // Overdrive reset pulse.
                        self.putfr(ctx, ANN_RESET, &["Reset", "Rst", "R"]);
                        self.state = State::PresenceDetectHigh;
                    } else if time < SLOT_MAX[self.od()] {
                        // Read/write time slot.
                        if time < LOWR_MIN[self.od()] {
                            self.putfr(
                                ctx,
                                ANN_WARN,
                                &["Low signal not long enough", "Low too short", "LOW < 1.0"],
                            );
                        }
                        if time < LOWR_MAX[self.od()] {
                            self.bit = 1; // Short pulse is a 1 bit.
                        } else {
                            self.bit = 0; // Long pulse is a 0 bit.
                        }
                        // Wait for end of slot.
                        self.state = State::Slot;
                    } else {
                        // Timing outside of known states.
                        self.putfr(ctx, ANN_WARN, &["Erroneous signal", "Error", "Err", "E"]);
                        self.state = State::Idle;
                    }
                }
                State::PresenceDetectHigh => {
                    // Wait for a falling edge and/or presence detect signal.
                    self.wait_falling_timeout(ctx, self.rise, PDH_MAX[self.od()])?;

                    // Calculate time since rising edge.
                    let time = self.elapsed(self.rise, ctx.samplenum());

                    if ctx.matched()[0] && !ctx.matched()[1] {
                        // Presence detected.
                        if time < PDH_MIN[self.od()] {
                            self.putrs(
                                ctx,
                                ANN_WARN,
                                &[
                                    "Presence detect signal is too early",
                                    "Presence detect too early",
                                    PDH_MIN_TEXT[self.od()],
                                ],
                            );
                        }
                        self.fall = ctx.samplenum();
                        self.state = State::PresenceDetectLow;
                    } else {
                        // No presence detected.
                        self.putrs(
                            ctx,
                            ANN_PRESENCE,
                            &["Presence: false", "Presence", "Pres", "P"],
                        );
                        self.putprs(
                            ctx,
                            Item::List(vec![Item::Str("RESET/PRESENCE".into()), Item::Bool(false)]),
                        );
                        self.state = State::Idle;
                    }
                }
                State::PresenceDetectLow => {
                    // Slave presence signalled: wait for end of presence signal.
                    ctx.wait(&[&[Cond::Pin(OWR, CondKind::Rising)]])?;
                    // Calculate time since start of presence signal.
                    let time = self.elapsed(self.fall, ctx.samplenum());
                    if time < PDL_MIN[self.od()] {
                        self.putfs(
                            ctx,
                            ANN_WARN,
                            &[
                                "Presence detect signal is too short",
                                "Presence detect too short",
                                PDL_MIN_TEXT[self.od()],
                            ],
                        );
                    } else if time > PDL_MAX[self.od()] {
                        self.putfs(
                            ctx,
                            ANN_WARN,
                            &[
                                "Presence detect signal is too long",
                                "Presence detect too long",
                                PDL_MAX_TEXT[self.od()],
                            ],
                        );
                    }
                    if time > RSTH_MIN[self.od()] {
                        self.rise = ctx.samplenum();
                    }
                    // Wait for end of presence detect.
                    self.state = State::PresenceDetect;
                }
                State::Slot | State::PresenceDetect => {}
            }

            // End states (for additional checks).
            if self.state == State::Slot {
                // Wait for a falling edge and/or end of timeslot.
                self.wait_falling_timeout(ctx, self.fall, SLOT_MIN[self.od()])?;

                if ctx.matched()[0] && !ctx.matched()[1] {
                    // Low detected before end of slot.
                    self.putfs(
                        ctx,
                        ANN_WARN,
                        &["Time slot not long enough", "Slot too short", SLOT_MIN_TEXT[self.od()]],
                    );
                    // Don't output invalid bit.
                    self.fall = ctx.samplenum();
                    self.state = State::Low;
                } else {
                    // End of time slot: output bit.
                    let texts = vec![format!("Bit: {}", self.bit), self.bit.to_string()];
                    ctx.put_ann(self.fall, ctx.samplenum(), ANN_BIT, texts);
                    self.putpfs(
                        ctx,
                        Item::List(vec![Item::Str("BIT".into()), Item::Int(self.bit)]),
                    );
                    // Save command bits.
                    if self.bit_count >= 0 {
                        self.command += (self.bit as u64) << self.bit_count;
                        self.bit_count += 1;
                    }
                    // Check for overdrive ROM command.
                    if self.bit_count >= 8 {
                        if self.command == 0x3c || self.command == 0x69 {
                            self.overdrive = true;
                            let s = ctx.samplenum();
                            ctx.put_ann(
                                s,
                                s,
                                ANN_OVERDRIVE,
                                ann_texts(&["Entering overdrive mode", "Overdrive on"]),
                            );
                        }
                        self.bit_count = -1;
                    }
                    self.state = State::Idle;
                }
            }

            if self.state == State::PresenceDetect {
                // Wait for a falling edge and/or end of presence detect.
                self.wait_falling_timeout(ctx, self.rise, RSTH_MIN[self.od()])?;

                if ctx.matched()[0] && !ctx.matched()[1] {
                    // Low detected before end of presence detect.
                    self.putfs(
                        ctx,
                        ANN_WARN,
                        &[
                            "Presence detect not long enough",
                            "Presence detect too short",
                            RSTH_MIN_TEXT[self.od()],
                        ],
                    );
                    // Inform about presence detected.
                    self.putrs(
                        ctx,
                        ANN_PRESENCE,
                        &["Slave presence detected", "Slave present", "Present", "P"],
                    );
                    self.putprs(
                        ctx,
                        Item::List(vec![Item::Str("RESET/PRESENCE".into()), Item::Bool(true)]),
                    );
                    self.fall = ctx.samplenum();
                    self.state = State::Low;
                } else {
                    // End of time slot: inform about presence detected.
                    self.putrs(ctx, ANN_PRESENCE, &["Presence: true", "Presence", "Pres", "P"]);
                    self.putprs(
                        ctx,
                        Item::List(vec![Item::Str("RESET/PRESENCE".into()), Item::Bool(true)]),
                    );
                    self.rise = ctx.samplenum();
                    // Start counting the first 8 bits to get the ROM command.
                    self.bit_count = 0;
                    self.command = 0;
                    self.state = State::Idle;
                }
            }
        }
    }
}
