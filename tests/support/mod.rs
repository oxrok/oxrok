//! Shared helpers for decoder parity tests.
//!
//! Each integration test that uses this module must declare
//! `mod support;` and be gated on the `python` or `rustpython` feature
//! where it needs the reference decoder.

#![allow(dead_code)]

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

/// True when the Python decoders directory has decoders in it.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn python_available() -> bool {
    !oxrok::pydecode::defs().is_empty()
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

/// A case from sigrok-test's `test.conf`.
#[derive(Clone, Debug)]
pub struct CorpusCase {
    /// The test name.
    pub name: String,
    /// `channel id=number` pairs of the decoder under test.
    pub channels: Vec<(String, usize)>,
    /// `option id=value` pairs of the decoder under test.
    pub options: Vec<(String, String)>,
    /// The input path, relative to the sigrok-dumps root.
    pub input: Option<String>,
    /// The `stack` line, bottom first, when the test uses a chain.
    pub stack: Vec<String>,
}

/// One decoder of a corpus chain.
#[derive(Clone, Debug)]
pub struct CorpusDecoder {
    /// The decoder id.
    pub id: String,
    /// `channel id=number` pairs.
    pub channels: Vec<(String, usize)>,
    /// `option id=value` pairs.
    pub options: Vec<(String, String)>,
}

/// A corpus case with every decoder of the chain, not only the target.
#[derive(Clone, Debug)]
pub struct ChainCase {
    /// The test name.
    pub name: String,
    /// The decoders, in `test.conf` order.
    pub decoders: Vec<CorpusDecoder>,
    /// The input path, relative to the sigrok-dumps root.
    pub input: Option<String>,
    /// The `stack` line, bottom first, when the test uses a chain.
    pub stack: Vec<String>,
}

/// The sigrok-test checkout, from `SIGROK_TEST_DIR` or `~/work/sigrok-test`.
pub fn corpus_test_dir() -> Option<std::path::PathBuf> {
    if let Some(p) = std::env::var_os("SIGROK_TEST_DIR") {
        let p = std::path::PathBuf::from(p);
        return p.is_dir().then_some(p);
    }
    let home = std::env::var_os("HOME")?;
    let p = std::path::PathBuf::from(home).join("work/sigrok-test");
    p.is_dir().then_some(p)
}

/// The sigrok-dumps checkout, from `SIGROK_DUMPS_DIR` or `~/work/sigrok-dumps`.
pub fn corpus_dumps_dir() -> Option<std::path::PathBuf> {
    if let Some(p) = std::env::var_os("SIGROK_DUMPS_DIR") {
        let p = std::path::PathBuf::from(p);
        return p.is_dir().then_some(p);
    }
    let home = std::env::var_os("HOME")?;
    let p = std::path::PathBuf::from(home).join("work/sigrok-dumps");
    p.is_dir().then_some(p)
}

fn parse_conf(decoder: &str) -> Vec<ChainCase> {
    let Some(dir) = corpus_test_dir() else { return Vec::new() };
    let conf = dir.join("decoder/test").join(decoder).join("test.conf");
    let Ok(text) = std::fs::read_to_string(conf) else { return Vec::new() };
    let mut cases = Vec::new();
    let mut current: Option<ChainCase> = None;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(name) = line.strip_prefix("test ") {
            if let Some(case) = current.take() {
                cases.push(case);
            }
            current = Some(ChainCase {
                name: name.to_owned(),
                decoders: Vec::new(),
                input: None,
                stack: Vec::new(),
            });
            continue;
        }
        let Some(case) = current.as_mut() else { continue };
        let mut words = line.split_whitespace();
        match words.next() {
            Some("protocol-decoder") => {
                let id = words.next().unwrap_or_default().to_owned();
                let mut next = CorpusDecoder { id, channels: Vec::new(), options: Vec::new() };
                parse_pairs(line, "channel", &mut next.channels);
                parse_options(line, &mut next.options);
                case.decoders.push(next);
            }
            Some("stack") => {
                case.stack = words.map(str::to_owned).collect();
            }
            Some("input") => {
                case.input = words.next().map(str::to_owned);
            }
            _ => {}
        }
    }
    if let Some(case) = current {
        cases.push(case);
    }
    cases
}

/// The cases in sigrok-test for `decoder`, in file order, with the
/// target decoder's channel and option pairs.
pub fn corpus_cases(decoder: &str) -> Vec<CorpusCase> {
    parse_conf(decoder)
        .into_iter()
        .map(|case| {
            let target = case.decoders.iter().find(|d| d.id == decoder);
            CorpusCase {
                name: case.name,
                channels: target.map(|d| d.channels.clone()).unwrap_or_default(),
                options: target.map(|d| d.options.clone()).unwrap_or_default(),
                input: case.input,
                stack: case.stack,
            }
        })
        .collect()
}

/// The cases in sigrok-test for `decoder`, with every decoder of each
/// chain.
pub fn corpus_chain_cases(decoder: &str) -> Vec<ChainCase> {
    parse_conf(decoder)
}

fn parse_pairs(line: &str, kind: &str, out: &mut Vec<(String, usize)>) {
    let words: Vec<&str> = line.split_whitespace().collect();
    for pair in words.windows(2) {
        if pair[0] == kind
            && let Some((k, v)) = pair[1].split_once('=')
            && let Ok(n) = v.parse()
        {
            out.push((k.to_owned(), n));
        }
    }
}

fn parse_options(line: &str, out: &mut Vec<(String, String)>) {
    let words: Vec<&str> = line.split_whitespace().collect();
    for pair in words.windows(2) {
        if pair[0] == "option"
            && let Some((k, v)) = pair[1].split_once('=')
        {
            out.push((k.to_owned(), v.to_owned()));
        }
    }
}

/// A `Config` for a corpus case, with numeric channel assignments.
pub fn corpus_config(info: &oxrok::decode::DecoderInfo, case: &CorpusCase) -> Config {
    corpus_config_parts(info, &case.channels, &case.options)
}

fn corpus_config_parts(
    info: &oxrok::decode::DecoderInfo,
    channel_pairs: &[(String, usize)],
    option_pairs: &[(String, String)],
) -> Config {
    let mut options: std::collections::HashMap<String, oxrok::decode::OptionValue> =
        info.options.iter().map(|o| (o.id.to_owned(), o.default.clone())).collect();
    for (id, value) in option_pairs {
        if let Some(o) = info.options.iter().find(|o| o.id == id) {
            options.insert(id.clone(), o.parse(value).expect("corpus option"));
        }
    }
    let count = info.channels.len() + info.optional_channels.len();
    let mut channels = vec![None; count];
    for (id, index) in channel_pairs {
        if let Some(pos) = info
            .channels
            .iter()
            .chain(info.optional_channels.iter())
            .position(|c| c.id.eq_ignore_ascii_case(id))
        {
            channels[pos] = Some(*index);
        }
    }
    Config { options, channels }
}

/// Runs a decoder chain, bottom first, over `capture`.
pub fn run_chain(parts: &[(&'static DecoderDef, Config)], capture: &Capture) -> Vec<Output> {
    let mut stack = Stack::new();
    let mut seen: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
    let chain: Vec<(String, Box<dyn oxrok::decode::Decoder>, Config)> = parts
        .iter()
        .map(|(def, cfg)| {
            let n = seen.entry(def.info.id).or_insert(0);
            *n += 1;
            (format!("{}-{}", def.info.id, n), (def.new)(), cfg.clone())
        })
        .collect();
    stack.add_chain(chain, capture.unit_size, capture.samplerate).expect("start decoder chain");
    let rx = stack.outputs().expect("output receiver");
    stack.feed(&capture.data);
    stack.end();
    let outputs: Vec<Output> = rx.into_iter().map(|o| plain_output(&o)).collect();
    stack.join();
    outputs
}

/// The native definitions and configs of a corpus chain, or `None` when
/// a lower decoder is not native yet.
fn chain_configs(
    native_id: &str,
    target: &'static DecoderDef,
    case: &ChainCase,
) -> Option<Vec<(&'static DecoderDef, Config)>> {
    let ids: Vec<String> =
        if case.stack.is_empty() { vec![native_id.to_owned()] } else { case.stack.clone() };
    let mut out = Vec::new();
    for id in ids {
        let def = if id == native_id {
            target
        } else {
            oxrok::decoders::ALL.iter().copied().find(|d| d.info.id == id)?
        };
        let record = case.decoders.iter().find(|d| d.id == id)?;
        let cfg = corpus_config_parts(def.info, &record.channels, &record.options);
        out.push((def, cfg));
    }
    Some(out)
}

/// Runs every case in sigrok-test for `native_id`, and returns
/// `(ran, skipped)`. A case is skipped when its input is missing or a
/// lower decoder of its chain is not native yet. The corpus is optional,
/// so a missing checkout skips everything.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn assert_corpus_parity(native_id: &str) -> (usize, usize) {
    assert_corpus_parity_def(native(native_id), native_id)
}

/// Like [`assert_corpus_parity`], with the native definition at hand so a
/// decoder being ported need not be in `ALL` yet.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn assert_corpus_parity_def(def: &'static DecoderDef, native_id: &str) -> (usize, usize) {
    let cases = corpus_chain_cases(native_id);
    let Some(dumps) = corpus_dumps_dir() else { return (0, cases.len()) };
    let mut ran = 0;
    let mut skipped = 0;
    for case in cases {
        let Some(relative) = &case.input else {
            skipped += 1;
            continue;
        };
        let path = dumps.join(relative);
        if !path.is_file() {
            skipped += 1;
            continue;
        }
        let Some(native_parts) = chain_configs(native_id, def, &case) else {
            skipped += 1;
            continue;
        };
        let mut python_parts = Vec::new();
        let mut available = true;
        for (native_def, cfg) in &native_parts {
            let Some(py) = python_opt(native_def.info.id) else {
                available = false;
                break;
            };
            python_parts.push((py, cfg.clone()));
        }
        if !available {
            skipped += 1;
            continue;
        }
        let capture = load(path.to_str().expect("utf-8 corpus path"));
        let native = run_chain(&native_parts, &capture);
        let python = run_chain(&python_parts, &capture);
        assert_eq!(
            native.len(),
            python.len(),
            "{}: output count differs\nnative: {native:#?}\npython: {python:#?}",
            case.name
        );
        for (i, (n, p)) in native.iter().zip(&python).enumerate() {
            assert_eq!(
                comparable(n),
                comparable(p),
                "{}: output {i} differs\nnative: {n:#?}\npython: {p:#?}",
                case.name
            );
        }
        ran += 1;
    }
    (ran, skipped)
}

#[cfg(any(feature = "python", feature = "rustpython"))]
fn python_opt(id: &str) -> Option<&'static DecoderDef> {
    oxrok::pydecode::defs().into_iter().find(|d| d.info.id == id)
}

#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn parity(
    native_id: &str,
    python_id: &str,
    pairs: &[(&str, &str)],
    capture: &Capture,
) -> (Vec<Output>, Vec<Output>) {
    parity_def(native(native_id), python_id, pairs, capture)
}

/// Like [`parity`], with the native definition at hand.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn parity_def(
    native_def: &'static DecoderDef,
    python_id: &str,
    pairs: &[(&str, &str)],
    capture: &Capture,
) -> (Vec<Output>, Vec<Output>) {
    let python_def = python(python_id);
    let cfg = config(native_def.info, pairs, &capture.channels);
    (run(native_def, &cfg, capture), run(python_def, &cfg, capture))
}

/// Outputs as they can be compared across hosts: an `End` error keeps
/// only whether it happened. The Python host reports an exception class
/// and traceback, which is not the native error text and not part of the
/// decoder wire format.
fn comparable(output: &Output) -> Output {
    match output {
        Output::End { instance, error } => Output::End {
            instance: instance.clone(),
            error: error.as_ref().map(|_| "<error>".to_owned()),
        },
        _ => output.clone(),
    }
}

/// Asserts that the native and Python decoders emit the same outputs.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn assert_parity(native_id: &str, pairs: &[(&str, &str)], capture: &Capture) {
    assert_parity_def(native(native_id), native_id, pairs, capture);
}

/// Like [`assert_parity`], with the native definition at hand so a decoder
/// being ported need not be in `ALL` yet.
#[cfg(any(feature = "python", feature = "rustpython"))]
pub fn assert_parity_def(
    native_def: &'static DecoderDef,
    python_id: &str,
    pairs: &[(&str, &str)],
    capture: &Capture,
) {
    let (native, python) = parity_def(native_def, python_id, pairs, capture);
    let native_id = native_def.info.id;
    assert_eq!(
        native.len(),
        python.len(),
        "{native_id}: output count differs\nnative: {native:#?}\npython: {python:#?}"
    );
    for (i, (n, p)) in native.iter().zip(&python).enumerate() {
        assert_eq!(
            comparable(n),
            comparable(p),
            "{native_id}: output {i} differs\nnative: {n:#?}\npython: {p:#?}"
        );
    }
}
