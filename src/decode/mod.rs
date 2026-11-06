// Counterpart in sigrok:
//   libsigrokdecode (the decoder API, instance handling and the sample
//   matching of wait()).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the documented decoder API and the observed behaviour.

//! The protocol decoder runtime.
//!
//! A [`Decoder`] describes itself with a [`DecoderInfo`] and runs its
//! [`Decoder::decode`] loop against a [`Context`], which hands out
//! samples through `wait` and takes annotations, binary data and items
//! for stacked decoders through `put_*`. The API mirrors
//! libsigrokdecode's Python API, so Python decoders run on the same
//! runtime through a thin host and native decoders read like them.
//!
//! A [`Stack`] runs instances on their own threads, fed with logic
//! packets, and delivers their output in order.

#![forbid(unsafe_code)]

pub mod info;
pub mod stream;

pub use info::{AnnClass, AnnRow, BinClass, ChannelDesc, DecoderInfo, OptionDesc, OptionValue};
pub use stream::{Cond, CondKind, Pins, Stream, StreamWriter};

use crate::core::{Error, Result};
use std::collections::HashMap;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::JoinHandle;

/// Output kinds a decoder registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutputKind {
    /// Annotations: a class index and texts, longest first.
    Annotation,
    /// Items for a stacked decoder.
    Python,
    /// Binary data of a binary class.
    Binary,
    /// Metadata.
    Meta,
}

/// An item passed from one decoder to the one stacked on it.
#[derive(Clone)]
pub enum Item {
    /// A string, e.g. a state name.
    Str(String),
    /// An integer.
    Int(i64),
    /// A non-negative integer too wide for `Int`, as big-endian bytes
    /// without leading zeros; Python integers have no width limit and
    /// an SPI word can be 152 bits.
    BigUint(Vec<u8>),
    /// A float.
    Float(f64),
    /// A list of items.
    List(Vec<Item>),
    /// Raw bytes.
    Bytes(Vec<u8>),
    /// A boolean.
    Bool(bool),
    /// Nothing.
    None,
    /// A host specific object passed between decoders of the same host
    /// unchanged, e.g. a Python object between two Python decoders.
    Opaque(std::sync::Arc<dyn std::any::Any + Send + Sync>),
}

impl PartialEq for Item {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Item::Str(a), Item::Str(b)) => a == b,
            (Item::Int(a), Item::Int(b)) => a == b,
            (Item::Float(a), Item::Float(b)) => a == b,
            (Item::List(a), Item::List(b)) => a == b,
            (Item::Bytes(a), Item::Bytes(b)) => a == b,
            (Item::Bool(a), Item::Bool(b)) => a == b,
            (Item::None, Item::None) => true,
            (Item::Opaque(a), Item::Opaque(b)) => std::sync::Arc::ptr_eq(a, b),
            _ => false,
        }
    }
}

/// What a decoder emitted.
#[derive(Clone, Debug, PartialEq)]
pub enum Output {
    /// An annotation over `ss..es`.
    Annotation {
        /// Instance id, e.g. `uart-1`.
        instance: String,
        /// Start sample.
        ss: u64,
        /// End sample.
        es: u64,
        /// Index into `DecoderInfo::annotations`.
        class: usize,
        /// Texts, longest first.
        texts: Vec<String>,
    },
    /// Binary output.
    Binary {
        /// Instance id.
        instance: String,
        /// Start sample.
        ss: u64,
        /// End sample.
        es: u64,
        /// Index into `DecoderInfo::binary`.
        class: usize,
        /// The bytes.
        data: Vec<u8>,
    },
    /// An item for stacked decoders.
    Python {
        /// Instance id.
        instance: String,
        /// Start sample.
        ss: u64,
        /// End sample.
        es: u64,
        /// The item.
        item: Item,
    },
    /// Metadata.
    Meta {
        /// Instance id.
        instance: String,
        /// Start sample.
        ss: u64,
        /// End sample.
        es: u64,
        /// The registered name, e.g. `Bitrate`.
        name: String,
        /// The value.
        item: Item,
    },
    /// The instance's decode loop ended, with an error message when it
    /// failed.
    End {
        /// Instance id.
        instance: String,
        /// The error, if any.
        error: Option<String>,
    },
}

/// What a decoder sees while decoding. Contexts move with their
/// decoder's thread, so they are `Send`.
pub trait Context: Send {
    /// Waits for one of the condition groups to match (each group is a
    /// conjunction). Returns the pin levels at that sample, or
    /// [`Error::Eof`] at the end of the data. With no groups, advances
    /// one sample.
    fn wait(&mut self, conds: &[&[Cond]]) -> Result<Pins>;

    /// The current sample number.
    fn samplenum(&self) -> u64;

    /// Which groups matched at the last `wait`.
    fn matched(&self) -> &[bool];

    /// True when the decoder channel is assigned.
    fn has_channel(&self, channel: usize) -> bool;

    /// Emits an annotation.
    fn put_ann(&mut self, ss: u64, es: u64, class: usize, texts: Vec<String>);

    /// Emits binary data.
    fn put_binary(&mut self, ss: u64, es: u64, class: usize, data: Vec<u8>);

    /// Emits an item for stacked decoders.
    fn put_python(&mut self, ss: u64, es: u64, item: Item);

    /// Emits metadata under its registered `name`.
    fn put_meta(&mut self, ss: u64, es: u64, name: &str, item: Item);
}

impl std::fmt::Debug for Item {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self}")
    }
}

impl std::fmt::Display for Item {
    /// As sigrok-cli prints metadata values: numbers bare, strings in
    /// single quotes, lists in brackets.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Item::Str(s) => write!(f, "'{s}'"),
            Item::Int(v) => write!(f, "{v}"),
            Item::BigUint(b) => write!(f, "{}", big_decimal(b)),
            Item::Float(v) => write!(f, "{v}"),
            Item::Bool(b) => write!(f, "{b}"),
            Item::None => write!(f, "None"),
            Item::Bytes(b) => write!(f, "{b:?}"),
            Item::Opaque(_) => write!(f, "<object>"),
            Item::List(items) => {
                write!(f, "[")?;
                for (i, it) in items.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{it}")?;
                }
                write!(f, "]")
            }
        }
    }
}

/// A big-endian unsigned integer in decimal, as Python prints it.
fn big_decimal(bytes: &[u8]) -> String {
    let mut n: Vec<u8> = bytes.iter().copied().skip_while(|&b| b == 0).collect();
    if n.is_empty() {
        return "0".into();
    }
    let mut digits = Vec::new();
    while !n.is_empty() {
        let mut rem = 0u32;
        for b in &mut n {
            let cur = (rem << 8) | u32::from(*b);
            *b = (cur / 10) as u8;
            rem = cur % 10;
        }
        digits.push(b'0' + rem as u8);
        let lead = n.iter().take_while(|&&b| b == 0).count();
        n.drain(..lead);
    }
    digits.reverse();
    String::from_utf8(digits).unwrap_or_default()
}

/// The configuration of one instance.
#[derive(Clone, Debug, Default)]
pub struct Config {
    /// Option values by id, parsed to the option's type.
    pub options: HashMap<String, OptionValue>,
    /// Decoder channel (required then optional, in declaration order)
    /// to logic channel index, when assigned.
    pub channels: Vec<Option<usize>>,
}

impl Config {
    /// An option as integer, or its default.
    pub fn int(&self, id: &str, default: i64) -> i64 {
        match self.options.get(id) {
            Some(OptionValue::Int(v)) => *v,
            Some(OptionValue::Float(v)) => *v as i64,
            _ => default,
        }
    }

    /// An option as float, or its default.
    pub fn float(&self, id: &str, default: f64) -> f64 {
        match self.options.get(id) {
            Some(OptionValue::Float(v)) => *v,
            Some(OptionValue::Int(v)) => *v as f64,
            _ => default,
        }
    }

    /// An option as string, or its default.
    pub fn str<'a>(&'a self, id: &str, default: &'a str) -> &'a str {
        match self.options.get(id) {
            Some(OptionValue::Str(v)) => v,
            _ => default,
        }
    }
}

/// A protocol decoder.
pub trait Decoder: Send {
    /// What this decoder is.
    fn info(&self) -> &'static DecoderInfo;

    /// Prepares for a run with `config`. Called once before `decode`.
    fn start(&mut self, config: &Config) -> Result<()>;

    /// The sample rate of the logic input, before `decode`.
    fn metadata(&mut self, samplerate: u64) {
        let _ = samplerate;
    }

    /// The decode loop of a logic-input decoder: runs until `wait`
    /// returns [`Error::Eof`].
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()>;

    /// One item from the decoder below, for stacked decoders.
    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let _ = (ss, es, item, ctx);
        Err(Error::not_supported("this decoder takes logic input"))
    }
}

/// How to create a decoder.
pub struct DecoderDef {
    /// Its description.
    pub info: &'static DecoderInfo,
    /// Creates a fresh instance.
    pub new: &'static (dyn Fn() -> Box<dyn Decoder> + Send + Sync),
}

/// The decoders a program knows about.
#[derive(Default)]
pub struct Registry {
    defs: Vec<&'static DecoderDef>,
}

impl Registry {
    /// An empty registry.
    pub fn new() -> Self {
        Registry::default()
    }

    /// Adds decoders. Kept sorted by id.
    pub fn register(&mut self, defs: &[&'static DecoderDef]) {
        self.defs.extend_from_slice(defs);
        self.defs.sort_by_key(|d| d.info.id);
        self.defs.dedup_by_key(|d| d.info.id);
    }

    /// All decoders, sorted by id.
    pub fn decoders(&self) -> impl Iterator<Item = &'static DecoderDef> + '_ {
        self.defs.iter().copied()
    }

    /// The decoder with `id`.
    pub fn find(&self, id: &str) -> Option<&'static DecoderDef> {
        self.defs.iter().copied().find(|d| d.info.id == id)
    }
}

/// Parses `key=value` options against a decoder's option table; keys
/// that name channels assign them by logic channel name.
///
/// As in libsigrokdecode, decoder channels left unassigned default to
/// the logic channel of the same index when there is one, and a name
/// that matches no channel is a warning, not an error.
pub fn parse_spec(
    info: &DecoderInfo,
    pairs: &[(String, String)],
    channel_names: &[String],
) -> Result<Config> {
    let count = info.channels.len() + info.optional_channels.len();
    let mut config = Config {
        options: info.options.iter().map(|o| (o.id.to_owned(), o.default.clone())).collect(),
        channels: (0..count).map(|i| (i < channel_names.len()).then_some(i)).collect(),
    };
    let mut assigned = vec![false; count];
    for (key, value) in pairs {
        if let Some(pos) = info
            .channels
            .iter()
            .chain(info.optional_channels.iter())
            .position(|c| c.id.eq_ignore_ascii_case(key))
        {
            // sigrok-cli drops a name that matches no channel before it
            // builds the mapping, so it neither assigns the channel nor
            // cancels the defaults: with no valid name at all, every
            // channel keeps its default.
            match channel_names.iter().position(|n| !n.is_empty() && n == value) {
                Some(index) => {
                    assigned[pos] = true;
                    config.channels[pos] = Some(index);
                }
                None => log::warn!("No channel with name \"{value}\" found."),
            }
            continue;
        }
        let opt = info.options.iter().find(|o| o.id == key).ok_or_else(|| {
            Error::Arg(format!("Unknown option or channel '{key}' for decoder '{}'.", info.id))
        })?;
        let parsed = opt.parse(value)?;
        if !opt.values.is_empty() && !opt.values.contains(&parsed) {
            return Err(Error::Arg(format!(
                "Invalid value '{value}' for option '{key}' of decoder '{}'.",
                info.id
            )));
        }
        config.options.insert(key.clone(), parsed);
    }
    // A user mapping replaces the default for every channel.
    if assigned.iter().any(|&a| a) {
        for (i, a) in assigned.iter().enumerate() {
            if !a {
                config.channels[i] = None;
            }
        }
    }
    for (i, c) in info.channels.iter().enumerate() {
        if config.channels[i].is_none() {
            return Err(Error::Arg(format!(
                "Decoder '{}' needs channel '{}' assigned.",
                info.id, c.id
            )));
        }
    }
    Ok(config)
}

/// One running chain: the bottom decoder's thread and its stream.
struct Running {
    handle: Option<JoinHandle<()>>,
    writer: StreamWriter,
    /// Ids of the whole chain, bottom first.
    chain: Vec<String>,
}

/// A decoder stacked on the one below: it gets that decoder's Python
/// output items, synchronously, as libsigrokdecode does, so the
/// annotations of a chain come out in the same order. An item the
/// decoder fails on is logged and the next one is delivered anyway,
/// which is also what libsigrokdecode does.
struct Link {
    decoder: Box<dyn Decoder>,
    ctx: ThreadContext,
}

/// A set of decoder chains fed from one logic stream.
pub struct Stack {
    running: Vec<Running>,
    /// Dropped by `end`, so the receiver sees the end once every
    /// decoder thread is gone, even one that died.
    tx: Option<Sender<Output>>,
    rx: Option<Receiver<Output>>,
    unit_size: usize,
}

impl Default for Stack {
    fn default() -> Self {
        Self::new()
    }
}

impl Stack {
    /// An empty stack.
    pub fn new() -> Self {
        let (tx, rx) = channel();
        Stack { running: Vec::new(), tx: Some(tx), rx: Some(rx), unit_size: 0 }
    }

    /// Adds an instance and starts its thread. `unit_size` is the bytes
    /// per sample of the logic input, `samplerate` its rate.
    pub fn add(
        &mut self,
        instance: String,
        decoder: Box<dyn Decoder>,
        config: &Config,
        unit_size: usize,
        samplerate: u64,
    ) -> Result<()> {
        self.add_chain(vec![(instance, decoder, config.clone())], unit_size, samplerate)
    }

    /// Adds a chain: the first decoder reads the logic input, each next
    /// one the Python output of the one before it.
    pub fn add_chain(
        &mut self,
        chain: Vec<(String, Box<dyn Decoder>, Config)>,
        unit_size: usize,
        samplerate: u64,
    ) -> Result<()> {
        self.unit_size = unit_size;
        let mut iter = chain.into_iter();
        let Some((instance, mut decoder, config)) = iter.next() else {
            return Err(Error::arg("empty decoder chain"));
        };
        decoder.start(&config)?;
        if samplerate > 0 {
            decoder.metadata(samplerate);
        }
        let mut uppers = Vec::new();
        let mut ids = vec![instance.clone()];
        for (id, mut dec, cfg) in iter {
            dec.start(&cfg)?;
            if samplerate > 0 {
                dec.metadata(samplerate);
            }
            ids.push(id.clone());
            uppers.push((id, dec, cfg));
        }
        let (stream, writer) = Stream::new(unit_size, config.channels.clone());
        let tx = self.tx.clone().ok_or_else(|| Error::device("decoder stack already ended"))?;
        let name = instance.clone();
        let handle = std::thread::Builder::new()
            .name(format!("decoder {instance}"))
            // Python decoders run on this thread, and an interpreter
            // needs more than the default 2 MiB in a debug build.
            .stack_size(8 << 20)
            .spawn(move || {
                // Sends End for the chain when the thread ends, also by
                // a panic.
                let guard = EndGuard { tx: tx.clone(), ids: ids_for_guard(&name, &uppers) };
                // Build the chain top down so each link owns the next.
                let mut upper: Option<Box<Link>> = None;
                for (id, dec, cfg) in uppers.into_iter().rev() {
                    let (s, _) = Stream::new(unit_size, cfg.channels.clone());
                    let ctx = ThreadContext {
                        stream: s,
                        tx: tx.clone(),
                        instance: id,
                        matched: Vec::new(),
                        upper: upper.take(),
                        bottom: false,
                    };
                    upper = Some(Box::new(Link { decoder: dec, ctx }));
                }
                let mut ctx = ThreadContext {
                    stream,
                    tx: tx.clone(),
                    instance: name.clone(),
                    matched: Vec::new(),
                    upper,
                    bottom: true,
                };
                let r = decoder.decode(&mut ctx);
                let error = match r {
                    Ok(()) | Err(Error::Eof) => None,
                    Err(e) => Some(e.to_string()),
                };
                drop(ctx);
                guard.finish(error.as_deref());
            })
            .map_err(Error::Io)?;
        self.running.push(Running { handle: Some(handle), writer, chain: ids });
        Ok(())
    }

    /// The instances' ids, in order.
    pub fn instances(&self) -> Vec<String> {
        self.running.iter().flat_map(|r| r.chain.iter().cloned()).collect()
    }

    /// Feeds logic samples to every instance.
    pub fn feed(&mut self, data: &[u8]) {
        if !data.len().is_multiple_of(self.unit_size.max(1)) {
            log::warn!("logic data is not a whole number of samples");
        }
        for r in &self.running {
            r.writer.push(data);
        }
    }

    /// Ends the input; the decoders finish and send `Output::End`.
    pub fn end(&mut self) {
        for r in &self.running {
            r.writer.end();
        }
        self.tx = None;
    }

    /// Takes the output receiver. Outputs arrive in emission order per
    /// instance.
    pub fn outputs(&mut self) -> Option<Receiver<Output>> {
        self.rx.take()
    }

    /// Waits for every instance to finish.
    pub fn join(&mut self) {
        for r in &mut self.running {
            if let Some(h) = r.handle.take() {
                let _ = h.join();
            }
        }
    }
}

impl Drop for Stack {
    fn drop(&mut self) {
        self.end();
        self.join();
    }
}

/// The chain's instance ids, bottom first.
fn ids_for_guard(bottom: &str, uppers: &[(String, Box<dyn Decoder>, Config)]) -> Vec<String> {
    let mut ids = vec![bottom.to_owned()];
    ids.extend(uppers.iter().map(|u| u.0.clone()));
    ids
}

/// Sends `End` for every instance of a chain exactly once.
struct EndGuard {
    tx: Sender<Output>,
    ids: Vec<String>,
}

impl EndGuard {
    fn finish(mut self, error: Option<&str>) {
        let ids = std::mem::take(&mut self.ids);
        for (i, id) in ids.into_iter().enumerate() {
            let e = if i == 0 { error.map(str::to_owned) } else { None };
            let _ = self.tx.send(Output::End { instance: id, error: e });
        }
    }
}

impl Drop for EndGuard {
    fn drop(&mut self) {
        let error = std::thread::panicking().then(|| "decoder thread panicked".to_owned());
        for id in std::mem::take(&mut self.ids) {
            let _ = self.tx.send(Output::End { instance: id, error: error.clone() });
        }
    }
}

struct ThreadContext {
    stream: Stream,
    tx: Sender<Output>,
    instance: String,
    matched: Vec<bool>,
    upper: Option<Box<Link>>,
    /// False for a stacked decoder, which has no sample stream.
    bottom: bool,
}

impl Context for ThreadContext {
    fn wait(&mut self, conds: &[&[Cond]]) -> Result<Pins> {
        if !self.bottom {
            return Err(Error::not_supported("a stacked decoder has no sample stream"));
        }
        let (pins, matched) = self.stream.wait(conds)?;
        self.matched = matched;
        Ok(pins)
    }

    fn samplenum(&self) -> u64 {
        self.stream.samplenum()
    }

    fn matched(&self) -> &[bool] {
        &self.matched
    }

    fn has_channel(&self, channel: usize) -> bool {
        self.stream.has_channel(channel)
    }

    fn put_ann(&mut self, ss: u64, es: u64, class: usize, texts: Vec<String>) {
        let _ = self.tx.send(Output::Annotation {
            instance: self.instance.clone(),
            ss,
            es,
            class,
            texts,
        });
    }

    fn put_binary(&mut self, ss: u64, es: u64, class: usize, data: Vec<u8>) {
        let _ =
            self.tx.send(Output::Binary { instance: self.instance.clone(), ss, es, class, data });
    }

    fn put_python(&mut self, ss: u64, es: u64, item: Item) {
        if let Some(link) = &mut self.upper
            && let Err(e) = link.decoder.decode_item(ss, es, &item, &mut link.ctx)
        {
            log::warn!("{}: {e}", link.ctx.instance);
        }
        let _ = self.tx.send(Output::Python { instance: self.instance.clone(), ss, es, item });
    }

    fn put_meta(&mut self, ss: u64, es: u64, name: &str, item: Item) {
        let _ = self.tx.send(Output::Meta {
            instance: self.instance.clone(),
            ss,
            es,
            name: name.to_owned(),
            item,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::parse_spec;
    use crate::decoders::i2c::INFO;

    fn names(n: &[&str]) -> Vec<String> {
        n.iter().map(|s| (*s).to_string()).collect()
    }

    fn pairs(p: &[(&str, &str)]) -> Vec<(String, String)> {
        p.iter().map(|(k, v)| ((*k).to_string(), (*v).to_string())).collect()
    }

    #[test]
    fn wide_integers_print_in_decimal() {
        use super::Item;
        assert_eq!(Item::BigUint(vec![]).to_string(), "0");
        assert_eq!(Item::BigUint(vec![0x01, 0x00]).to_string(), "256");
        // 2^64 and 2^152 - 1, the largest 152 bit SPI word.
        let mut two64 = vec![1u8];
        two64.extend([0u8; 8]);
        assert_eq!(Item::BigUint(two64).to_string(), "18446744073709551616");
        assert_eq!(
            Item::BigUint(vec![0xff; 19]).to_string(),
            "5708990770823839524233143877797980545530986495"
        );
    }

    #[test]
    fn unknown_channel_names_keep_the_defaults_as_in_sigrok_cli() {
        let ch = names(&["SCL", "SDA"]);
        let cfg = parse_spec(&INFO, &pairs(&[("scl", "0"), ("sda", "1")]), &ch).unwrap();
        assert_eq!(cfg.channels, vec![Some(0), Some(1)]);
    }

    #[test]
    fn one_valid_name_cancels_the_other_defaults() {
        let ch = names(&["SCL", "SDA"]);
        let err = parse_spec(&INFO, &pairs(&[("scl", "SDA"), ("sda", "nope")]), &ch).unwrap_err();
        assert!(err.to_string().contains("needs channel 'sda'"), "{err}");
        let cfg = parse_spec(&INFO, &pairs(&[("scl", "SDA"), ("sda", "SCL")]), &ch).unwrap();
        assert_eq!(cfg.channels, vec![Some(1), Some(0)]);
    }
}
