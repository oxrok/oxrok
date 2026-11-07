// Counterpart in sigrok:
//   libsigrokdecode/decoders/morse/pd.py,
//   Copyright (C) 2017 Christoph Rackwitz <christoph.rackwitz@rwth-aachen.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the ITU-R M.1677-1 code table and the decoder's
//   documented classes and texts.

//! Morse code: symbol timing, letters, words, with the time unit
//! adapted from the signals.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const DATA: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_TIME: usize = 0;
const ANN_UNITS: usize = 1;
const ANN_LETTER: usize = 3;
const ANN_WORD: usize = 4;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "timeunit",
    desc: "Time unit (guess)",
    default: OptionValue::f(0.1),
    values: &[],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "morse",
    name: "Morse",
    longname: "Morse code",
    desc: "Demodulated morse code protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Encoding"],
    channels: &[ChannelDesc { id: "data", name: "Data", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "time", desc: "Time" },
        AnnClass { id: "units", desc: "Units" },
        AnnClass { id: "symbol", desc: "Symbol" },
        AnnClass { id: "letter", desc: "Letter" },
        AnnClass { id: "word", desc: "Word" },
    ],
    annotation_rows: &[
        AnnRow { id: "time", desc: "Time", classes: &[0] },
        AnnRow { id: "units", desc: "Units", classes: &[1] },
        AnnRow { id: "symbol", desc: "Symbol", classes: &[2] },
        AnnRow { id: "letter", desc: "Letter", classes: &[3] },
        AnnRow { id: "word", desc: "Word", classes: &[4] },
    ],
    binary: &[],
    doc: "Morse code is a method of transmitting text information as a series of\n\
          on-off tones.\n\
          \n\
          Details:\n\
          https://en.wikipedia.org/wiki/Morse_code",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Morse::new())
}

/// Recommendation ITU-R M.1677-1 (10/2009), International Morse code.
///
/// Keys are the dit/dah pattern of pd.py's `encode_ditdah()`, which is
/// exactly how the decoder looks a sequence up after decoding the units.
static ALPHABET: [(&str, &str); 55] = [
    // 1.1.1 Letters
    (".-", "a"),
    ("-...", "b"),
    ("-.-.", "c"),
    ("-..", "d"),
    (".", "e"),
    ("..-..", "é"), // "accented"
    ("..-.", "f"),
    ("--.", "g"),
    ("....", "h"),
    ("..", "i"),
    (".---", "j"),
    ("-.-", "k"),
    (".-..", "l"),
    ("--", "m"),
    ("-.", "n"),
    ("---", "o"),
    (".--.", "p"),
    ("--.-", "q"),
    (".-.", "r"),
    ("...", "s"),
    ("-", "t"),
    ("..-", "u"),
    ("...-", "v"),
    (".--", "w"),
    ("-..-", "x"),
    ("-.--", "y"),
    ("--..", "z"),
    // 1.1.2 Figures
    (".----", "1"),
    ("..---", "2"),
    ("...--", "3"),
    ("....-", "4"),
    (".....", "5"),
    ("-....", "6"),
    ("--...", "7"),
    ("---..", "8"),
    ("----.", "9"),
    ("-----", "0"),
    // 1.1.3 Punctuation marks and miscellaneous signs
    (".-.-.-", "."),         // Full stop (period)
    ("--..--", ","),         // Comma
    ("---...", ":"),         // Colon or division sign
    ("..--..", "?"),         // Question mark
    (".----.", "’"),         // Apostrophe
    ("-....-", "-"),         // Hyphen or dash or subtraction sign
    ("-..-.", "/"),          // Fraction bar or division sign
    ("-.--.", "("),          // Left-hand bracket (parenthesis)
    ("-.--.-", ")"),         // Right-hand bracket (parenthesis)
    (".-..-.", "“ ”"),       // Inverted commas (quotation marks)
    ("-...-", "="),          // Double hyphen
    ("...-.", "UNDERSTOOD"), // Understood
    ("........", "ERROR"),   // Error (eight dots)
    (".-.-.", "+"),          // Cross or addition sign
    (".-...", "WAIT"),       // Wait
    ("...-.-", "EOW"),       // End of work
    ("-.-.-", "START"),      // Starting signal (to precede every transmission)
    (".--.-.", "@"),         // Commercial at
];

/// Python's `'{:.{p}g}'`: `p` significant digits, no trailing zeros.
fn format_g(x: f64, p: usize) -> String {
    if x == 0.0 {
        return "0".to_owned();
    }
    if !x.is_finite() {
        return format!("{x}");
    }
    // The exponent comes from the correctly rounded mantissa, as Python's
    // g format decides fixed vs scientific after rounding.
    let sci = format!("{:.*e}", p - 1, x);
    let (mant, exp) = sci.split_once('e').expect("scientific float format");
    let exp: i32 = exp.parse().expect("exponent");
    if exp < -4 || exp >= p as i32 {
        let mut mant = mant.to_owned();
        if mant.contains('.') {
            while mant.ends_with('0') {
                mant.pop();
            }
            if mant.ends_with('.') {
                mant.pop();
            }
        }
        let sign = if exp < 0 { '-' } else { '+' };
        return format!("{mant}e{sign}{:02}", exp.abs());
    }
    let decimals = (p as i32 - 1 - exp).max(0) as usize;
    let mut fixed = format!("{x:.decimals$}");
    if fixed.contains('.') {
        while fixed.ends_with('0') {
            fixed.pop();
        }
        if fixed.ends_with('.') {
            fixed.pop();
        }
    }
    fixed
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

struct Morse {
    samplerate: f64,
    timeunit: f64,
}

impl Morse {
    fn new() -> Morse {
        Morse { samplerate: 0.0, timeunit: 0.1 }
    }
}

impl Decoder for Morse {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.timeunit = config.float("timeunit", 0.1);
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate as f64;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Strictly speaking there is no point in running this decoder
        // when the sample rate is unknown or zero, but pd.py falls back
        // to a rate of 1.
        let samplerate = if self.samplerate == 0.0 { 1.0 } else { self.samplerate };
        let mut timeunit = self.timeunit;

        // decode_symbols(): the first actual edge starts a unit.
        ctx.wait(&[&[Cond::pin(DATA, CondKind::Rising)]])?;
        let mut prevtime = ctx.samplenum();

        // decode_morse()/decode(): the current letter and word.
        let mut sequence = String::new();
        let mut letter_s0: Option<u64> = None;
        let mut letter_s1 = 0u64;
        let mut word = String::new();
        let mut word_s0: Option<u64> = None;
        let mut word_s1 = 0u64;

        loop {
            // pd.py: int(5 * self.samplerate * timeunit)
            let skip = (5.0 * samplerate * timeunit) as u64;
            let pins = ctx.wait(&[&[Cond::pin(DATA, CondKind::Edge)], &[Cond::Skip(skip)]])?;
            let curtime = ctx.samplenum();
            let timeout = ctx.matched()[1];
            let val = pins.get(DATA);
            let pval = i64::from(!val);

            if timeout {
                // decode_symbols() yields None; decode_morse() flushes
                // the letter and passes the flush through to the word.
                flush_letter(
                    ctx,
                    &mut sequence,
                    &mut letter_s0,
                    letter_s1,
                    &mut word,
                    &mut word_s0,
                    &mut word_s1,
                );
                flush_word(ctx, &mut word, &mut word_s0, word_s1);
                continue;
            }

            let dt = (curtime - prevtime) as f64 / samplerate;
            let units = dt / timeunit;
            let iunits = py_round(units).max(1.0) as i64;
            let error = (units - iunits as f64).abs();

            // Decode symbols, annotate symbols, emit symbols.
            ctx.put_ann(prevtime, curtime, ANN_TIME, vec![format_g(dt, 3)]);

            // pd.py's `symbols` table: (level, time units).
            let known = matches!((pval, iunits), (1, 1 | 3) | (0, 1 | 3 | 7));
            if known {
                ctx.put_ann(
                    prevtime,
                    curtime,
                    ANN_UNITS,
                    vec![format!("{:.1}*{}", units, format_g(timeunit, 3))],
                );
                if pval == 1 {
                    if letter_s0.is_none() {
                        letter_s0 = Some(prevtime);
                    }
                    letter_s1 = curtime;
                    sequence.push(if iunits == 3 { '-' } else { '.' });
                } else if iunits >= 3 {
                    // Generate "flush" for end of letter, end of word.
                    flush_letter(
                        ctx,
                        &mut sequence,
                        &mut letter_s0,
                        letter_s1,
                        &mut word,
                        &mut word_s0,
                        &mut word_s1,
                    );
                }
            } else {
                ctx.put_ann(
                    prevtime,
                    curtime,
                    ANN_UNITS,
                    vec![format!("!! {:.1}*{} !!", units, format_g(timeunit, 3))],
                );
            }

            prevtime = curtime;

            // Adapt the time unit, weighted by how close the symbol was.
            let thisunit = dt / iunits as f64;
            timeunit += (thisunit - timeunit) * 0.2 * (1.0 - 2.0 * error).max(0.0);
        }
    }
}

/// Group the marks of a letter, annotate it and append it to the word.
#[allow(clippy::too_many_arguments)]
fn flush_letter(
    ctx: &mut dyn Context,
    sequence: &mut String,
    letter_s0: &mut Option<u64>,
    letter_s1: u64,
    word: &mut String,
    word_s0: &mut Option<u64>,
    word_s1: &mut u64,
) {
    if sequence.is_empty() {
        return;
    }
    let letter = ALPHABET
        .iter()
        .find(|(pattern, _)| pattern == sequence)
        .map_or_else(|| sequence.clone(), |&(_, letter)| letter.to_owned());
    let s0 = letter_s0.unwrap_or(0);
    ctx.put_ann(s0, letter_s1, ANN_LETTER, vec![letter.clone()]);
    if word_s0.is_none() {
        *word_s0 = Some(s0);
    }
    *word_s1 = letter_s1;
    word.push_str(&letter);
    sequence.clear();
    *letter_s0 = None;
}

/// Flush the word collected so far.
fn flush_word(ctx: &mut dyn Context, word: &mut String, word_s0: &mut Option<u64>, word_s1: u64) {
    if word.is_empty() {
        return;
    }
    let s0 = word_s0.unwrap_or(0);
    ctx.put_ann(s0, word_s1, ANN_WORD, vec![word.clone()]);
    word.clear();
    *word_s0 = None;
}
