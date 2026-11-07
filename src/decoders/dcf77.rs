// Counterpart in sigrok:
//   libsigrokdecode/decoders/dcf77/pd.py,
//   Copyright (C) 2012-2016 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the DCF77 time code and the decoder's documented
//   classes and texts.

//! DCF77: the European longwave time signal, decoded from the length of
//! each second's high period.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_START_OF_MINUTE: usize = 0;
const ANN_SPECIAL_BITS: usize = 1;
const ANN_CALL_BIT: usize = 2;
const ANN_SUMMER_TIME: usize = 3;
const ANN_CEST: usize = 4;
const ANN_CET: usize = 5;
const ANN_LEAP_SECOND: usize = 6;
const ANN_START_OF_TIME: usize = 7;
const ANN_MINUTE: usize = 8;
const ANN_MINUTE_PARITY: usize = 9;
const ANN_HOUR: usize = 10;
const ANN_HOUR_PARITY: usize = 11;
const ANN_DAY: usize = 12;
const ANN_DAY_OF_WEEK: usize = 13;
const ANN_MONTH: usize = 14;
const ANN_YEAR: usize = 15;
const ANN_DATE_PARITY: usize = 16;
const ANN_RAW_BITS: usize = 17;
const ANN_UNKNOWN_BITS: usize = 18;
const ANN_WARNINGS: usize = 19;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "dcf77",
    name: "DCF77",
    longname: "DCF77 time protocol",
    desc: "European longwave time signal (77.5kHz carrier signal).",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Clock/timing"],
    channels: &[ChannelDesc { id: "data", name: "DATA", desc: "DATA line" }],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "start-of-minute", desc: "Start of minute" },
        AnnClass { id: "special-bits", desc: "Special bits (civil warnings, weather forecast)" },
        AnnClass { id: "call-bit", desc: "Call bit" },
        AnnClass { id: "summer-time", desc: "Summer time announcement" },
        AnnClass { id: "cest", desc: "CEST bit" },
        AnnClass { id: "cet", desc: "CET bit" },
        AnnClass { id: "leap-second", desc: "Leap second bit" },
        AnnClass { id: "start-of-time", desc: "Start of encoded time" },
        AnnClass { id: "minute", desc: "Minute" },
        AnnClass { id: "minute-parity", desc: "Minute parity bit" },
        AnnClass { id: "hour", desc: "Hour" },
        AnnClass { id: "hour-parity", desc: "Hour parity bit" },
        AnnClass { id: "day", desc: "Day of month" },
        AnnClass { id: "day-of-week", desc: "Day of week" },
        AnnClass { id: "month", desc: "Month" },
        AnnClass { id: "year", desc: "Year" },
        AnnClass { id: "date-parity", desc: "Date parity bit" },
        AnnClass { id: "raw-bits", desc: "Raw bits" },
        AnnClass { id: "unknown-bits", desc: "Unknown bits" },
        AnnClass { id: "warnings", desc: "Human-readable warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[17, 18] },
        AnnRow {
            id: "fields",
            desc: "Fields",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
        },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[19] },
    ],
    binary: &[],
    doc: "This PD decodes the DCF77 protocol (a European long-wave time signal that\n\
uses a 77.5kHz carrier frequency).\n\
\n\
Details:\n\
http://en.wikipedia.org/wiki/DCF77",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Dcf77::default())
}

/// `bcd2int()` from the Python decoder: the low nibble plus ten times
/// the high nibble.
fn bcd2int(b: i64) -> i64 {
    (b & 0x0f) + ((b >> 4) * 10)
}

#[derive(Default)]
struct Dcf77 {
    samplerate: u64,
    wait_rising: bool,
    ss_bit: u64,
    ss_bit_old: u64,
    es_bit: u64,
    ss_block: u64,
    datebits: Vec<i64>,
    bitcount: i64,
    bitnumber_is_known: bool,
    tmp: i64,
}

impl Dcf77 {
    /// "Annotation for a single DCF77 bit."
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_bit, self.es_bit, class, texts);
    }

    /// "Annotation for a multi-bit DCF77 field."
    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, ctx.samplenum(), class, texts);
    }

    #[allow(clippy::too_many_lines)]
    fn handle_dcf77_bit(&mut self, ctx: &mut dyn Context, bit: i64) {
        let c = self.bitcount;

        // "Use 'Unknown DCF77 bit x: val' if we're not sure yet which of
        // the 0..58 bits it is (because we haven't seen a 'new minute'
        // marker yet). Otherwise, use 'DCF77 bit x: val'."
        let (s, ann) = if self.bitnumber_is_known {
            ("B", ANN_RAW_BITS)
        } else {
            ("Unknown b", ANN_UNKNOWN_BITS)
        };
        self.putx(ctx, ann, vec![format!("{s}it {c}: {bit}"), format!("{bit}")]);

        // "If we're not sure yet which of the 0..58 DCF77 bits we have,
        // return. We don't want to decode bogus data."
        if !self.bitnumber_is_known {
            return;
        }

        // "Collect bits 36-58, we'll need them for a parity check later."
        if (36..=58).contains(&c) {
            self.datebits.push(bit);
        }

        if c == 0 {
            // "Start of minute: DCF bit 0."
            if bit == 0 {
                self.putx(
                    ctx,
                    ANN_START_OF_MINUTE,
                    vec![
                        "Start of minute (always 0)".into(),
                        "Start of minute".into(),
                        "SoM".into(),
                    ],
                );
            } else {
                self.putx(
                    ctx,
                    ANN_WARNINGS,
                    vec!["Start of minute != 0".into(), "SoM != 0".into()],
                );
            }
        } else if (1..=14).contains(&c) {
            // "Special bits (civil warnings, weather forecast): DCF77 bits 1-14."
            if c == 1 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 1);
            }
            if c == 14 {
                let s = format!("{:014b}", self.tmp);
                self.putb(
                    ctx,
                    ANN_SPECIAL_BITS,
                    vec![format!("Special bits: {s}"), format!("SB: {s}")],
                );
            }
        } else if c == 15 {
            let s = if bit == 1 { "" } else { "not " };
            self.putx(ctx, ANN_CALL_BIT, vec![format!("Call bit: {s}set"), format!("CB: {s}set")]);
        } else if c == 16 {
            let s = if bit == 1 { "" } else { "not " };
            let x = if bit == 1 { "yes" } else { "no" };
            self.putx(
                ctx,
                ANN_SUMMER_TIME,
                vec![
                    format!("Summer time announcement: {s}active"),
                    format!("Summer time: {s}active"),
                    format!("Summer time: {x}"),
                    format!("ST: {x}"),
                ],
            );
        } else if c == 17 {
            let s = if bit == 1 { "" } else { "not " };
            let x = if bit == 1 { "yes" } else { "no" };
            self.putx(ctx, ANN_CEST, vec![format!("CEST: {s}in effect"), format!("CEST: {x}")]);
        } else if c == 18 {
            let s = if bit == 1 { "" } else { "not " };
            let x = if bit == 1 { "yes" } else { "no" };
            self.putx(ctx, ANN_CET, vec![format!("CET: {s}in effect"), format!("CET: {x}")]);
        } else if c == 19 {
            let s = if bit == 1 { "" } else { "not " };
            let x = if bit == 1 { "yes" } else { "no" };
            self.putx(
                ctx,
                ANN_LEAP_SECOND,
                vec![
                    format!("Leap second announcement: {s}active"),
                    format!("Leap second: {s}active"),
                    format!("Leap second: {x}"),
                    format!("LS: {x}"),
                ],
            );
        } else if c == 20 {
            // "Start of encoded time: DCF bit 20."
            if bit == 1 {
                self.putx(
                    ctx,
                    ANN_START_OF_TIME,
                    vec![
                        "Start of encoded time (always 1)".into(),
                        "Start of encoded time".into(),
                        "SoeT".into(),
                    ],
                );
            } else {
                self.putx(
                    ctx,
                    ANN_WARNINGS,
                    vec!["Start of encoded time != 1".into(), "SoeT != 1".into()],
                );
            }
        } else if (21..=27).contains(&c) {
            // "Minutes (0-59): DCF77 bits 21-27 (BCD format)."
            if c == 21 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 21);
            }
            if c == 27 {
                let m = bcd2int(self.tmp);
                self.putb(ctx, ANN_MINUTE, vec![format!("Minutes: {m}"), format!("Min: {m}")]);
            }
        } else if c == 28 {
            // "Even parity over minute bits (21-28): DCF77 bit 28."
            self.tmp |= bit << (c - 21);
            let parity = self.tmp.count_ones();
            let s = if parity.is_multiple_of(2) { "OK" } else { "INVALID!" };
            self.putx(
                ctx,
                ANN_MINUTE_PARITY,
                vec![format!("Minute parity: {s}"), format!("Min parity: {s}")],
            );
        } else if (29..=34).contains(&c) {
            // "Hours (0-23): DCF77 bits 29-34 (BCD format)."
            if c == 29 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 29);
            }
            if c == 34 {
                self.putb(ctx, ANN_HOUR, vec![format!("Hours: {}", bcd2int(self.tmp))]);
            }
        } else if c == 35 {
            // "Even parity over hour bits (29-35): DCF77 bit 35."
            self.tmp |= bit << (c - 29);
            let parity = self.tmp.count_ones();
            let s = if parity.is_multiple_of(2) { "OK" } else { "INVALID!" };
            self.putx(ctx, ANN_HOUR_PARITY, vec![format!("Hour parity: {s}")]);
        } else if (36..=41).contains(&c) {
            // "Day of month (1-31): DCF77 bits 36-41 (BCD format)."
            if c == 36 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 36);
            }
            if c == 41 {
                self.putb(ctx, ANN_DAY, vec![format!("Day: {}", bcd2int(self.tmp))]);
            }
        } else if (42..=44).contains(&c) {
            // "Day of week (1-7): DCF77 bits 42-44 (BCD format).
            // A value of 1 means Monday, 7 means Sunday."
            if c == 42 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 42);
            }
            if c == 44 {
                let d = bcd2int(self.tmp);
                // Python's calendar.day_name[d - 1], with Monday first;
                // a negative index counts from Sunday, which is how a
                // value of 0 reads as Sunday.
                let names =
                    ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"];
                let index = d - 1;
                if (-7..=6).contains(&index) {
                    let dn = names[(index + 7) as usize % 7];
                    self.putb(
                        ctx,
                        ANN_DAY_OF_WEEK,
                        vec![format!("Day of week: {d} ({dn})"), format!("DoW: {d} ({dn})")],
                    );
                } else {
                    self.putb(
                        ctx,
                        ANN_WARNINGS,
                        vec![format!("Day of week: {d} (invalid)"), format!("DoW: {d} (inv)")],
                    );
                }
            }
        } else if (45..=49).contains(&c) {
            // "Month (1-12): DCF77 bits 45-49 (BCD format)."
            if c == 45 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 45);
            }
            if c == 49 {
                let m = bcd2int(self.tmp);
                // Python's calendar.month_name[m] has '' at index 0 and
                // raises IndexError above 12.
                let names = [
                    "",
                    "January",
                    "February",
                    "March",
                    "April",
                    "May",
                    "June",
                    "July",
                    "August",
                    "September",
                    "October",
                    "November",
                    "December",
                ];
                if (0..names.len() as i64).contains(&m) {
                    let mn = names[m as usize];
                    self.putb(
                        ctx,
                        ANN_MONTH,
                        vec![format!("Month: {m} ({mn})"), format!("Mon: {m} ({mn})")],
                    );
                } else {
                    self.putb(
                        ctx,
                        ANN_WARNINGS,
                        vec![format!("Month: {m} (invalid)"), format!("Mon: {m} (inv)")],
                    );
                }
            }
        } else if (50..=57).contains(&c) {
            // "Year (0-99): DCF77 bits 50-57 (BCD format)."
            if c == 50 {
                self.tmp = bit;
                self.ss_block = self.ss_bit;
            } else {
                self.tmp |= bit << (c - 50);
            }
            if c == 57 {
                self.putb(ctx, ANN_YEAR, vec![format!("Year: {}", bcd2int(self.tmp))]);
            }
        } else if c == 58 {
            // "Even parity over date bits (36-58): DCF77 bit 58." The
            // parity bit itself is in datebits: it is appended above.
            let parity = self.datebits.iter().filter(|&&b| b == 1).count();
            let s = if parity.is_multiple_of(2) { "OK" } else { "INVALID!" };
            self.putx(ctx, ANN_DATE_PARITY, vec![format!("Date parity: {s}"), format!("DP: {s}")]);
            self.datebits.clear();
        } else {
            self.putx(
                ctx,
                ANN_WARNINGS,
                vec![
                    format!("Invalid DCF77 bit: {c}"),
                    format!("Invalid bit: {c}"),
                    format!("Inv: {c}"),
                ],
            );
        }
    }
}

impl Decoder for Dcf77 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.samplerate = 0;
        self.wait_rising = true;
        self.ss_bit = 0;
        self.ss_bit_old = 0;
        self.es_bit = 0;
        self.ss_block = 0;
        self.datebits.clear();
        self.bitcount = 0;
        self.bitnumber_is_known = false;
        self.tmp = 0;
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        loop {
            if self.wait_rising {
                // "Wait until the next rising edge occurs."
                ctx.wait(&[&[Cond::pin(DATA, CondKind::Rising)]])?;

                // "Save the sample number where the DCF77 bit begins."
                self.ss_bit = ctx.samplenum();

                // "Calculate the length (in ms) between two rising edges."
                let len_edges = self.ss_bit - self.ss_bit_old;
                let len_edges_ms = (len_edges as f64 / self.samplerate as f64 * 1000.0) as i64;

                // "Thus, if we see an edge with a 2000ms distance to the
                // last one, this edge marks the beginning of a new minute
                // (and DCF77 bit 0 of that minute)."
                if (1600..=2400).contains(&len_edges_ms) {
                    self.bitcount = 0;
                    self.ss_bit_old = self.ss_bit;
                    self.bitnumber_is_known = true;
                }

                self.ss_bit_old = self.ss_bit;
                self.wait_rising = false;
            } else {
                // "Wait until the next falling edge occurs."
                ctx.wait(&[&[Cond::pin(DATA, CondKind::Falling)]])?;

                // "Save the sample number where the DCF77 bit ends."
                self.es_bit = ctx.samplenum();

                // "Calculate the length (in ms) of the current high period."
                let len_high = self.es_bit - self.ss_bit;
                let len_high_ms = (len_high as f64 / self.samplerate as f64 * 1000.0) as i64;

                // "If the high signal was 100ms long, that encodes a 0 bit.
                // If it was 200ms long, that encodes a 1 bit."
                let bit = if (40..=160).contains(&len_high_ms) {
                    0
                } else if (161..=260).contains(&len_high_ms) {
                    1
                } else {
                    -1
                };

                if bit == 0 || bit == 1 {
                    self.handle_dcf77_bit(ctx, bit);
                    self.bitcount += 1;
                } else {
                    self.putx(
                        ctx,
                        ANN_WARNINGS,
                        vec!["Invalid bit timing".into(), "Inv timing".into(), "Inv".into()],
                    );
                }

                self.wait_rising = true;
            }
        }
    }
}
