//! Any logic samples through any native decoder, with its default
//! options, on a [`Stack`] as the command line runs it.
//!
//! The input: two bytes choose the decoder among `oxrok::decoders::ALL`,
//! one the sample rate and whether the optional channels are assigned,
//! and the rest are the samples. A decoder that takes another's output
//! runs on top of the decoders that produce it, down to one that takes
//! logic.
//!
//! What passes: a decoder that ends with an error is fine, as a Python
//! decoder that raises is for libsigrokdecode. A panic in a decoder's
//! thread is the finding; libFuzzer's panic hook aborts on it.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxrok::decode::{Config, Decoder, DecoderDef, Stack, parse_spec};
use oxrok::decoders::ALL;

/// Sample rates a capture has, and 0 for one that names none.
const RATES: [u64; 8] =
    [0, 1_000, 20_000, 100_000, 1_000_000, 12_000_000, 100_000_000, 1_000_000_000];

/// The decoder whose output `def` takes: the one named as the input when
/// there is one (`i2c`, not `i2cfilter`), else the first that produces it.
fn below(def: &DecoderDef) -> Option<&'static DecoderDef> {
    let feeds = |d: &&&DecoderDef| def.info.inputs.iter().any(|i| d.info.outputs.contains(i));
    ALL.iter()
        .filter(feeds)
        .find(|d| def.info.inputs.contains(&d.info.id))
        .or_else(|| ALL.iter().find(feeds))
        .copied()
}

/// `top` and what it stacks on, bottom first; `None` when no chain of
/// decoders leads down to logic.
fn chain_of(top: &'static DecoderDef) -> Option<Vec<&'static DecoderDef>> {
    let mut chain = vec![top];
    let mut def = top;
    while !def.info.inputs.contains(&"logic") {
        def = below(def)?;
        if chain.len() > 8 || chain.iter().any(|d| d.info.id == def.info.id) {
            return None;
        }
        chain.push(def);
    }
    chain.reverse();
    Some(chain)
}

fuzz_target!(|data: &[u8]| {
    let [a, b, flags, samples @ ..] = data else { return };
    let top = ALL[usize::from(u16::from_le_bytes([*a, *b])) % ALL.len()];
    let samplerate = RATES[usize::from(flags & 7)];
    let Some(chain) = chain_of(top) else { return };

    // The bottom decoder's channels on the logic channels of the same
    // index, the optional ones too or not: `-P id` with no mapping, over
    // a capture of that many channels.
    let bottom = chain[0].info;
    let mut count = bottom.channels.len();
    if flags & 8 != 0 {
        count += bottom.optional_channels.len();
    }
    let names: Vec<String> = (0..count).map(|i| format!("D{i}")).collect();
    let unit_size = count.div_ceil(8).max(1);

    let mut built: Vec<(String, Box<dyn Decoder>, Config)> = Vec::new();
    for (i, def) in chain.iter().enumerate() {
        let names: &[String] = if i == 0 { &names } else { &[] };
        let Ok(config) = parse_spec(def.info, &[], names) else { return };
        built.push((format!("{}-1", def.info.id), (def.new)(), config));
    }

    let mut stack = Stack::new();
    if stack.add_chain(built, unit_size, samplerate).is_err() {
        return;
    }
    let outputs = stack.outputs();
    // In two parts, so that a pattern across a chunk boundary is met.
    let whole = samples.len() - samples.len() % unit_size;
    let cut = (whole / 2) - (whole / 2) % unit_size;
    stack.feed(&samples[..cut]);
    stack.feed(&samples[cut..whole]);
    stack.end();
    // Until every decoder thread is gone.
    for _ in outputs.into_iter().flatten() {}
    stack.join();
});
