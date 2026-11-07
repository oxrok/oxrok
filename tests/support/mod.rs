//! Shared helpers for decoder parity tests.
//!
//! Each integration test that uses this module must declare
//! `mod support;` and be gated on the `python` or `rustpython` feature
//! where it needs the reference decoder.

use oxrok::core::{Key, Packet, Value};
use oxrok::decode::{Config, DecoderDef, Item, Output, Stack, parse_spec};

/// A logic capture loaded from an oxrok input module.
pub struct Capture {
    /// Channel names, in the file's bit order.
    pub channels: Vec<String>,
    /// Bytes per sample.
    pub unit_size: usize,
    /// Samples per second, or 0 when the file has no sample rate.
    pub samplerate: u64,
    /// Packed logic samples.
    pub data: Vec<u8>,
}

/// Loads a VCD file into packed logic samples with its channel names.
pub fn load_vcd(path: &str) -> Capture {
    let module = oxrok::formats::input::find("vcd").expect("vcd input module");
    let mut input = (module.create)(&Default::default()).expect("create vcd input");
    let bytes = std::fs::read(path).expect("read VCD");
    let mut packets = Vec::new();
    input.feed(&bytes, &mut packets).expect("parse VCD");
    input.end(&mut packets).expect("finish VCD");
    let channels =
        input.channels().expect("VCD channels").iter().map(|c| c.name.clone()).collect::<Vec<_>>();
    let mut unit_size = 0;
    let mut samplerate = 0;
    let mut data = Vec::new();
    for packet in packets {
        match packet {
            Packet::Logic(logic) => {
                unit_size = logic.unit_size;
                data.extend_from_slice(&logic.data);
            }
            Packet::Meta(settings) => {
                for (key, value) in settings {
                    if key == Key::Samplerate
                        && let Value::U64(rate) = value
                    {
                        samplerate = rate;
                    }
                }
            }
            _ => {}
        }
    }
    Capture { channels, unit_size, samplerate, data }
}

/// Loads a sigrok session file into packed logic samples.
pub fn load_sr(path: &str) -> Capture {
    let mut reader = oxrok::formats::sr::SessionReader::open(std::path::Path::new(path))
        .expect("open session file");
    let channels = reader.channels().iter().map(|c| c.name.clone()).collect::<Vec<_>>();
    let unit_size = reader.unit_size();
    let samplerate = reader.metadata().samplerate.unwrap_or(0);
    let mut data = Vec::new();
    reader
        .read(&mut |packet| {
            if let Packet::Logic(logic) = packet {
                data.extend_from_slice(&logic.data);
            }
        })
        .expect("read session file");
    Capture { channels, unit_size, samplerate, data }
}

/// Loads a `.vcd` or `.sr` capture.
pub fn load(path: &str) -> Capture {
    if path.ends_with(".sr") { load_sr(path) } else { load_vcd(path) }
}

/// The native decoder with `id`.
pub fn native(id: &str) -> &'static DecoderDef {
    oxrok::decoders::ALL
        .iter()
        .copied()
        .find(|d| d.info.id == id)
        .unwrap_or_else(|| panic!("no native decoder '{id}'"))
}

/// The Python decoder with `id`, regardless of a native decoder with the
/// same id.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn python(id: &str) -> &'static DecoderDef {
    oxrok::pydecode::defs()
        .into_iter()
        .find(|d| d.info.id == id)
        .unwrap_or_else(|| panic!("no Python decoder '{id}'"))
}

/// Parses `pairs` against `info`, as `-P dec:key=value` does.
pub fn config(
    info: &oxrok::decode::DecoderInfo,
    pairs: &[(&str, &str)],
    channels: &[String],
) -> Config {
    let pairs: Vec<(String, String)> =
        pairs.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
    parse_spec(info, &pairs, channels).expect("decoder configuration")
}

/// Runs `def` over `capture` and returns every output in order, with
/// Python objects reduced to plain values so hosts can be compared.
pub fn run(def: &'static DecoderDef, config: &Config, capture: &Capture) -> Vec<Output> {
    let mut stack = Stack::new();
    stack
        .add("test-1".to_owned(), (def.new)(), config, capture.unit_size, capture.samplerate)
        .expect("start decoder");
    let rx = stack.outputs().expect("output receiver");
    stack.feed(&capture.data);
    stack.end();
    let outputs: Vec<Output> = rx.into_iter().map(|o| plain_output(&o)).collect();
    stack.join();
    outputs
}

/// A Python `Item::Opaque` as a value-only [`Item`], recursively.
fn plain(item: &Item) -> Item {
    match item {
        Item::List(items) => Item::List(items.iter().map(plain).collect()),
        Item::Opaque(o) => {
            #[cfg(feature = "python")]
            if let Some(p) = o.downcast_ref::<oxrok::pydecode::PyItem>() {
                return plain(&p.to_item());
            }
            #[cfg(feature = "rustpython")]
            if let Some(p) = o.downcast_ref::<oxrok::pydecode::RpItem>() {
                return plain(&p.to_item());
            }
            item.clone()
        }
        _ => item.clone(),
    }
}

fn plain_output(output: &Output) -> Output {
    match output {
        Output::Python { instance, ss, es, item } => {
            Output::Python { instance: instance.clone(), ss: *ss, es: *es, item: plain(item) }
        }
        Output::Meta { instance, ss, es, name, item } => Output::Meta {
            instance: instance.clone(),
            ss: *ss,
            es: *es,
            name: name.clone(),
            item: plain(item),
        },
        _ => output.clone(),
    }
}

/// Runs the native and Python decoders with the same configuration and
/// the same capture, and returns both output sequences.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn parity(
    native_id: &str,
    python_id: &str,
    pairs: &[(&str, &str)],
    capture: &Capture,
) -> (Vec<Output>, Vec<Output>) {
    let native_def = native(native_id);
    let python_def = python(python_id);
    let cfg = config(native_def.info, pairs, &capture.channels);
    (run(native_def, &cfg, capture), run(python_def, &cfg, capture))
}

/// Asserts that the native and Python decoders emit the same outputs.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn assert_parity(native_id: &str, pairs: &[(&str, &str)], capture: &Capture) {
    let (native, python) = parity(native_id, native_id, pairs, capture);
    assert_eq!(
        native.len(),
        python.len(),
        "{native_id}: output count differs\nnative: {native:#?}\npython: {python:#?}"
    );
    for (i, (n, p)) in native.iter().zip(&python).enumerate() {
        assert_eq!(n, p, "{native_id}: output {i} differs\nnative: {n:#?}\npython: {p:#?}");
    }
}
