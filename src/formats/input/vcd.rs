// Counterpart in sigrok:
//   libsigrok/src/input/vcd.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `vcd`: reads a Value Change Dump into logic and analog samples.
//!
//! The sample rate is the inverse of the `$timescale`, so a file with a
//! 1 us timescale becomes a 1 MHz capture; `downsample` divides that.
//! `wire`, `reg` and `logic` signals become logic channels, one per bit
//! for vectors; `real` and `integer` signals become analog channels.

use super::{Input, InputModule, InputOption, InputOptionKind};
use crate::core::util::parse_period;
use crate::core::{
    AnalogData, Channel, ChannelKind, Error, Key, Mq, MqFlags, Packet, Result, Unit,
};

const CHUNK_BYTES: usize = 4 * 1024 * 1024;
/// Scope names of files we and libsigrok write; they are not prefixed to
/// channel names, so the names round trip.
const OWN_SCOPES: [&str; 2] = ["oxrok", "libsigrok"];

pub(super) static MODULE: InputModule = InputModule {
    id: "vcd",
    name: "VCD",
    description: "Value Change Dump",
    extensions: &["vcd"],
    options: &[
        InputOption {
            id: "numchannels",
            name: "Max number of channels",
            description: "The maximum number of channels to create for VCD input signals (0: all)",
            kind: InputOptionKind::I32,
            default: "0",
        },
        InputOption {
            id: "samplerate_overwrite",
            name: "Overwrite samplerate",
            description: "Use this sample rate instead of the one from the timescale",
            kind: InputOptionKind::U64,
            default: "0",
        },
        InputOption {
            id: "downsample",
            name: "Downsampling factor",
            description: "Divide the file's sample rate by this factor",
            kind: InputOptionKind::U64,
            default: "1",
        },
        InputOption {
            id: "skip",
            name: "Skip",
            description: "Start at this timestamp; by default samples start at the first timestamp in the file, 0 starts at timestamp 0",
            kind: InputOptionKind::U64,
            default: "",
        },
        InputOption {
            id: "compress",
            name: "Compress idle periods",
            description: "Compress idle periods longer than this many timescale ticks",
            kind: InputOptionKind::U64,
            default: "0",
        },
    ],
    create: |options| {
        let maxchannels = super::option_u64(options, "numchannels", 0)? as usize;
        let samplerate = super::option_u64(options, "samplerate_overwrite", 0)?;
        let downsample = super::option_u64(options, "downsample", 1)?.max(1);
        let compress = super::option_u64(options, "compress", 0)? / downsample;
        let skip = match options.get("skip").filter(|v| !v.is_empty()) {
            None => None,
            Some(v) => Some(crate::core::util::parse_sizestring(v)? / downsample),
        };
        Ok(Box::new(Vcd::new(VcdOptions { maxchannels, samplerate, downsample, compress, skip })))
    },
    detect: |head, _name| {
        let text = String::from_utf8_lossy(head);
        let t = text.trim_start_matches('\u{feff}').trim_start();
        if t.starts_with('$') && t.contains("$end") { 90 } else { 0 }
    },
};

struct VcdOptions {
    maxchannels: usize,
    samplerate: u64,
    downsample: u64,
    compress: u64,
    skip: Option<u64>,
}

/// One `$var` of the file.
struct Signal {
    ident: String,
    name: String,
    size: usize,
    kind: ChannelKind,
    /// First logic bit position, or the analog channel position.
    first: usize,
    digits: i32,
}

#[allow(clippy::struct_excessive_bools)]
struct Vcd {
    opts: VcdOptions,
    buf: Vec<u8>,
    header_done: bool,
    samplerate: u64,
    scope: Vec<String>,
    signals: Vec<Signal>,
    ignored: Vec<String>,
    logic_count: usize,
    analog_count: usize,
    channels: Vec<Channel>,
    unit_size: usize,
    current_logic: Vec<u8>,
    current_analog: Vec<f32>,
    started: bool,
    prev_ts: u64,
    have_origin: bool,
    data_after_ts: bool,
    skip_until_end: bool,
    ignore_end: bool,
    warned_xz: bool,
}

impl Vcd {
    fn new(opts: VcdOptions) -> Self {
        let samplerate = opts.samplerate;
        Vcd {
            opts,
            buf: Vec::new(),
            header_done: false,
            samplerate,
            scope: Vec::new(),
            signals: Vec::new(),
            ignored: Vec::new(),
            logic_count: 0,
            analog_count: 0,
            channels: Vec::new(),
            unit_size: 0,
            current_logic: Vec::new(),
            current_analog: Vec::new(),
            started: false,
            prev_ts: 0,
            have_origin: false,
            data_after_ts: false,
            skip_until_end: false,
            ignore_end: false,
            warned_xz: false,
        }
    }

    /// Parses header sections until `$enddefinitions`. Returns how many
    /// bytes were consumed, or `None` when the header is incomplete.
    fn parse_header(&mut self) -> Result<Option<usize>> {
        // Words are cut in the bytes as they came, so that what is consumed
        // counts those: a byte that is no UTF-8 is three in the lossy text.
        let bytes = self.buf.clone();
        let mut i = 0;
        let mut section: Option<(String, Vec<String>)> = None;
        loop {
            while i < bytes.len() && bytes[i].is_ascii_whitespace() {
                i += 1;
            }
            if i >= bytes.len() {
                return Ok(None);
            }
            let start = i;
            while i < bytes.len() && !bytes[i].is_ascii_whitespace() {
                i += 1;
            }
            let word = String::from_utf8_lossy(&bytes[start..i]);
            let word = word.trim_start_matches('\u{feff}');
            match (&mut section, word) {
                (Some((name, _)), "$end") => {
                    let done = name == "enddefinitions";
                    let (name, contents) = section.take().unwrap_or_default();
                    self.header_section(&name, &contents)?;
                    if done {
                        self.finish_header();
                        return Ok(Some(i));
                    }
                }
                (Some((_, contents)), w) => contents.push(w.to_owned()),
                (None, w) if w.starts_with('$') && w.len() > 1 => {
                    section = Some((w[1..].to_owned(), Vec::new()));
                }
                (None, _) => {}
            }
        }
    }

    fn header_section(&mut self, name: &str, contents: &[String]) -> Result<()> {
        match name {
            "timescale" => {
                if self.opts.samplerate == 0 {
                    let (p, q) = parse_period(&contents.join(" "))
                        .ok_or_else(|| Error::arg("Parsing $timescale failed."))?;
                    // A period of 0 names no sample rate. libsigrok divides
                    // by it: 0 Hz and this warning where that does not trap
                    // (aarch64), a crash where it does.
                    let rate = q.checked_div(p).unwrap_or(0);
                    if q.checked_rem(p) != Some(0) {
                        log::warn!("Inexact rounding of samplerate, {q} / {p} to {rate} Hz.");
                    }
                    self.samplerate = rate;
                }
            }
            "scope" => {
                if let Some(scope_name) = contents.get(1)
                    && !OWN_SCOPES.contains(&scope_name.as_str())
                {
                    self.scope.push(scope_name.clone());
                }
            }
            "upscope" => {
                self.scope.pop();
            }
            "var" => {
                let words: Vec<&str> = contents.iter().map(String::as_str).collect();
                self.parse_var(&words)?;
            }
            _ => {}
        }
        Ok(())
    }

    fn parse_var(&mut self, words: &[&str]) -> Result<()> {
        if words.len() < 4 {
            return Err(Error::arg("$var section should have 4 or 5 items"));
        }
        let (kind_text, size_text, ident, reference) = (words[0], words[1], words[2], words[3]);
        let range = words.get(4).copied().unwrap_or("");
        let kind = match kind_text {
            "reg" | "wire" | "logic" => ChannelKind::Logic,
            "real" | "integer" => ChannelKind::Analog,
            "string" => {
                self.ignored.push(ident.to_owned());
                return Ok(());
            }
            other => return Err(Error::Arg(format!("Unsupported signal type: '{other}'"))),
        };
        let mut size: usize = size_text.parse().unwrap_or(0);
        if kind == ChannelKind::Analog {
            size = 1;
        }
        if size == 0 {
            return Err(Error::Arg(format!("Unsupported signal size: '{size_text}'")));
        }
        if self.opts.maxchannels != 0
            && (self.logic_count + self.analog_count).saturating_add(size) > self.opts.maxchannels
        {
            log::warn!(
                "Skipping '{reference}{range}', exceeds requested channel count {}.",
                self.opts.maxchannels
            );
            self.ignored.push(ident.to_owned());
            return Ok(());
        }
        let mut name = String::new();
        for s in &self.scope {
            name.push_str(s);
            name.push('.');
        }
        name.push_str(reference);
        name.push_str(range);
        let first = match kind {
            ChannelKind::Logic => {
                let f = self.logic_count;
                self.logic_count += size;
                f
            }
            ChannelKind::Analog => {
                let f = self.analog_count;
                self.analog_count += 1;
                f
            }
        };
        let digits = if kind_text == "real" { 2 } else { 0 };
        self.signals.push(Signal { ident: ident.to_owned(), name, size, kind, first, digits });
        Ok(())
    }

    fn finish_header(&mut self) {
        let mut channels = Vec::new();
        for sig in self.signals.iter().filter(|s| s.kind == ChannelKind::Logic) {
            for bit in 0..sig.size {
                channels.push(Channel::new(
                    sig.first + bit,
                    ChannelKind::Logic,
                    bit_name(sig, bit),
                ));
            }
        }
        for sig in self.signals.iter().filter(|s| s.kind == ChannelKind::Analog) {
            channels.push(Channel::new(
                self.logic_count + sig.first,
                ChannelKind::Analog,
                sig.name.clone(),
            ));
        }
        self.channels = channels;
        self.unit_size = self.logic_count.div_ceil(8);
        self.current_logic = vec![0; self.unit_size];
        self.current_analog = vec![0.0; self.analog_count];
        self.header_done = true;
    }

    fn start(&mut self, out: &mut Vec<Packet>) {
        if !self.started {
            self.started = true;
            out.push(Packet::header());
            let rate = self.samplerate / self.opts.downsample;
            if rate != 0 {
                out.push(Packet::meta(Key::Samplerate, rate));
            }
        }
    }

    /// Emits `count` samples of the current state.
    fn add_samples(&mut self, count: u64, out: &mut Vec<Packet>) {
        if count == 0 {
            return;
        }
        if let Some(per_chunk) = CHUNK_BYTES.checked_div(self.unit_size) {
            let per_chunk = per_chunk.max(1) as u64;
            let mut left = count;
            while left > 0 {
                let n = left.min(per_chunk) as usize;
                let data = self.current_logic.repeat(n);
                out.push(Packet::logic(self.unit_size, data));
                left -= n as u64;
            }
        }
        for sig in self.signals.iter().filter(|s| s.kind == ChannelKind::Analog) {
            let value = self.current_analog[sig.first];
            let per_chunk = (CHUNK_BYTES / 4) as u64;
            let mut left = count;
            while left > 0 {
                let n = left.min(per_chunk) as usize;
                out.push(Packet::Analog(AnalogData {
                    channels: vec![self.logic_count + sig.first],
                    samples: vec![value; n],
                    mq: Mq::Voltage,
                    flags: MqFlags::DC,
                    unit: Unit::Volt,
                    digits: sig.digits,
                }));
                left -= n as u64;
            }
        }
    }

    fn timestamp(&mut self, mut ts: u64, out: &mut Vec<Packet>) {
        ts /= self.opts.downsample;
        if !self.have_origin {
            self.have_origin = true;
            if let Some(skip) = self.opts.skip {
                self.prev_ts = skip;
            } else {
                self.prev_ts = ts;
                return;
            }
        }
        if let Some(skip) = self.opts.skip
            && skip != 0
            && ts < skip
        {
            self.prev_ts = skip;
            return;
        }
        if ts == self.prev_ts {
            return;
        }
        if ts < self.prev_ts {
            log::warn!("Invalid timestamp: {ts} (leap backwards).");
            return;
        }
        let mut count = ts - self.prev_ts;
        if self.opts.compress != 0 && count > self.opts.compress {
            count = self.opts.compress;
        }
        self.add_samples(count, out);
        self.prev_ts = ts;
        self.data_after_ts = false;
    }

    fn bit_value(&mut self, c: char) -> Option<bool> {
        match c {
            '0' | 'l' | 'L' => Some(false),
            '1' | 'h' | 'H' => Some(true),
            'x' | 'X' | 'z' | 'Z' | 'u' | 'U' | '-' => {
                if !self.warned_xz {
                    self.warned_xz = true;
                    log::warn!("Undefined bit values (x, z) are read as 0.");
                }
                Some(false)
            }
            _ => None,
        }
    }

    fn set_bits(&mut self, ident: &str, bits: &[bool]) {
        let mut found = false;
        for si in 0..self.signals.len() {
            if self.signals[si].ident != ident {
                continue;
            }
            found = true;
            match self.signals[si].kind {
                ChannelKind::Analog => {
                    let mut v: u64 = 0;
                    for (i, b) in bits.iter().enumerate().take(64) {
                        if *b {
                            v |= 1 << i;
                        }
                    }
                    self.current_analog[self.signals[si].first] = v as f32;
                }
                ChannelKind::Logic => {
                    let (first, size) = (self.signals[si].first, self.signals[si].size);
                    for bit in 0..size {
                        let value = bits.get(bit).copied().unwrap_or(false);
                        let pos = first + bit;
                        if value {
                            self.current_logic[pos / 8] |= 1 << (pos % 8);
                        } else {
                            self.current_logic[pos / 8] &= !(1 << (pos % 8));
                        }
                    }
                }
            }
        }
        if !found && !self.ignored.iter().any(|i| i == ident) {
            log::warn!("VCD signal not found for ID '{ident}'.");
        }
    }

    fn set_real(&mut self, ident: &str, value: f32) {
        let mut found = false;
        for sig in self.signals.iter().filter(|s| s.kind == ChannelKind::Analog) {
            if sig.ident == ident {
                self.current_analog[sig.first] = value;
                found = true;
            }
        }
        if !found && !self.ignored.iter().any(|i| i == ident) {
            log::warn!("VCD signal not found for ID '{ident}'.");
        }
    }

    /// Processes complete whitespace separated tokens of the body.
    fn process_tokens(&mut self, text: &str, out: &mut Vec<Packet>) -> Result<()> {
        let mut words = text.split_whitespace().peekable();
        while let Some(word) = words.next() {
            if self.skip_until_end {
                if word == "$end" {
                    self.skip_until_end = false;
                }
                continue;
            }
            if self.ignore_end && word == "$end" {
                self.ignore_end = false;
                continue;
            }
            let first = word.chars().next().unwrap_or(' ');
            if first == '$' && word.len() > 1 {
                if matches!(word, "$dumpvars" | "$dumpon" | "$dumpoff") {
                    self.ignore_end = true;
                } else {
                    self.skip_until_end = true;
                }
                continue;
            }
            if first == '#' {
                let ts: u64 = word[1..]
                    .parse()
                    .map_err(|_| Error::Arg(format!("Invalid timestamp: {word}.")))?;
                self.timestamp(ts, out);
                continue;
            }
            self.data_after_ts = true;
            match first {
                'r' | 'R' => {
                    let ident =
                        words.next().ok_or_else(|| Error::arg("Unexpected real format."))?;
                    let value: f64 = word[1..].parse().map_err(|_| {
                        Error::Arg(format!("Cannot convert value: {}.", &word[1..]))
                    })?;
                    self.set_real(ident, value as f32);
                }
                'b' | 'B' => {
                    let ident =
                        words.next().ok_or_else(|| Error::arg("Unexpected vector format."))?;
                    // The last character is bit 0.
                    let mut bits = Vec::new();
                    for c in word[1..].chars().rev() {
                        let b = self.bit_value(c).ok_or_else(|| {
                            Error::Arg(format!("Unexpected vector format: {word}"))
                        })?;
                        bits.push(b);
                    }
                    self.set_bits(ident, &bits);
                }
                's' | 'S' => {
                    let ident = words
                        .next()
                        .ok_or_else(|| Error::arg("String value without identifier."))?;
                    if !self.ignored.iter().any(|i| i == ident) {
                        log::warn!("String value for identifier '{ident}' ignored.");
                    }
                }
                _ => {
                    let Some(b) = self.bit_value(first) else {
                        return Err(Error::Arg(format!("Unknown token '{word}'.")));
                    };
                    let ident = if word.len() > 1 {
                        word[1..].to_owned()
                    } else {
                        words.next().ok_or_else(|| Error::arg("Identifier missing."))?.to_owned()
                    };
                    self.set_bits(&ident, &[b]);
                }
            }
        }
        Ok(())
    }

    fn process_buffer(&mut self, is_eof: bool, out: &mut Vec<Packet>) -> Result<()> {
        if !self.header_done {
            match self.parse_header()? {
                Some(consumed) => {
                    self.buf.drain(..consumed);
                }
                None => return Ok(()),
            }
        }
        self.start(out);
        // Process whole lines; keep a partial last line for the next feed.
        let cut = if is_eof {
            self.buf.len()
        } else {
            self.buf.iter().rposition(|b| *b == b'\n').map_or(0, |p| p + 1)
        };
        if cut == 0 {
            return Ok(());
        }
        let text = String::from_utf8_lossy(&self.buf[..cut]).into_owned();
        self.buf.drain(..cut);
        self.process_tokens(&text, out)
    }
}

/// The channel name of bit `bit` of a signal: the name itself for a
/// single bit, `base[lower + bit]` for a vector declared with a matching
/// range, else `name.bit`.
fn bit_name(sig: &Signal, bit: usize) -> String {
    if sig.size <= 1 {
        return sig.name.clone();
    }
    if let Some(open) = sig.name.rfind('[')
        && let Some(inner) = sig.name[open + 1..].strip_suffix(']')
        && let Some((hi, lo)) = inner.split_once(':')
        && let (Ok(a), Ok(b)) = (hi.trim().parse::<usize>(), lo.trim().parse::<usize>())
    {
        let (lower, upper) = if a < b { (a, b) } else { (b, a) };
        if upper + 1 - lower == sig.size {
            return format!("{}[{}]", &sig.name[..open], lower + bit);
        }
    }
    format!("{}.{bit}", sig.name)
}

impl Input for Vcd {
    fn feed(&mut self, data: &[u8], out: &mut Vec<Packet>) -> Result<()> {
        self.buf.extend_from_slice(data);
        self.process_buffer(false, out)
    }

    fn end(&mut self, out: &mut Vec<Packet>) -> Result<()> {
        self.buf.push(b'\n');
        self.process_buffer(true, out)?;
        if !self.header_done {
            return Err(Error::arg("No $enddefinitions found: not a VCD file."));
        }
        if self.data_after_ts {
            self.add_samples(1, out);
        }
        out.push(Packet::End);
        Ok(())
    }

    fn channels(&self) -> Option<&[Channel]> {
        self.header_done.then_some(&self.channels)
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) {
        if let Some(c) = self.channels.get_mut(pos) {
            c.enabled = enabled;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(text: &str, options: &[(&str, &str)]) -> (Vec<Channel>, Vec<Packet>) {
        let opts: crate::formats::input::Options =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let mut input = MODULE.instantiate(&opts).unwrap();
        let mut out = Vec::new();
        input.feed(text.as_bytes(), &mut out).unwrap();
        input.end(&mut out).unwrap();
        (input.channels().unwrap().to_vec(), out)
    }

    fn logic(packets: &[Packet]) -> Vec<u8> {
        packets
            .iter()
            .filter_map(|p| if let Packet::Logic(l) = p { Some(l.data.clone()) } else { None })
            .flatten()
            .collect()
    }

    const FILE: &str = "$timescale 1 us $end\n$scope module libsigrok $end\n$var wire 1 ! D0 $end\n$var wire 1 \" D1 $end\n$upscope $end\n$enddefinitions $end\n#0 0! 0\"\n#5 1!\n#10 0! 1\"\n#15 0\"\n#20\n";

    #[test]
    fn reads_scalars_with_timescale() {
        let (channels, packets) = run(FILE, &[]);
        let names: Vec<&str> = channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["D0", "D1"]);
        assert_eq!(packets[1], Packet::meta(Key::Samplerate, 1_000_000u64));
        let data = logic(&packets);
        assert_eq!(data.len(), 20);
        assert_eq!(&data[..6], &[0, 0, 0, 0, 0, 1]);
        assert_eq!(&data[10..12], &[2, 2]);
        assert_eq!(&data[15..], &[0; 5]);
    }

    #[test]
    fn downsample_skip_and_compress() {
        let (_, packets) = run(FILE, &[("downsample", "5")]);
        assert_eq!(packets[1], Packet::meta(Key::Samplerate, 200_000u64));
        assert_eq!(logic(&packets), [0, 1, 2, 0]);
        let late = FILE.replace("#0 0! 0\"", "#3 0! 0\"");
        let (_, packets) = run(&late, &[("skip", "0")]);
        assert_eq!(logic(&packets).len(), 20);
        let (_, packets) = run(&late, &[]);
        assert_eq!(logic(&packets).len(), 17);
        let (_, packets) = run(FILE, &[("compress", "2")]);
        assert_eq!(logic(&packets).len(), 8);
    }

    /// The header's length is counted in the file's bytes. Counted in its
    /// lossy text, each of these bytes was three: the first data lines went
    /// with the header, or the count ran past the buffer and panicked.
    #[test]
    fn a_header_with_bytes_that_are_no_utf8_ends_where_it_ends() {
        let mut file = b"$comment \xff\xfe\xff\xfe\xff\xfe\xff\xfe $end\n".to_vec();
        file.extend_from_slice(FILE.as_bytes());
        let mut input = MODULE.instantiate(&crate::formats::input::Options::new()).unwrap();
        let mut out = Vec::new();
        input.feed(&file, &mut out).unwrap();
        input.end(&mut out).unwrap();
        assert_eq!(logic(&out), logic(&run(FILE, &[]).1));

        // Short enough for the count to pass the end.
        let mut input = MODULE.instantiate(&crate::formats::input::Options::new()).unwrap();
        let short = b"$comment \xff\xff\xff $end $enddefinitions $end\n#0\n";
        input.feed(short, &mut out).unwrap();
        input.end(&mut out).unwrap();
    }

    /// A `$timescale` of 0 is no sample rate, not a division by zero.
    #[test]
    fn a_timescale_of_zero_names_no_samplerate() {
        let text =
            "$timescale 0 us $end\n$var wire 1 ! D0 $end\n$enddefinitions $end\n#0 0!\n#5 1!\n#9\n";
        let (_, packets) = run(text, &[]);
        assert!(!packets.iter().any(|p| matches!(p, Packet::Meta(_))), "{packets:?}");
        assert_eq!(logic(&packets), [0, 0, 0, 0, 0, 1, 1, 1, 1]);
    }

    /// A signal of a width that does not fit a count with the channels
    /// before it is over `numchannels` as any other, and is skipped.
    #[test]
    fn a_signal_wider_than_a_count_is_over_numchannels() {
        let text = "$timescale 1 us $end\n$var wire 1 ! D0 $end\n$var wire 18446744073709551615 \" wide $end\n$enddefinitions $end\n#0 0!\n#5 1!\n#9\n";
        let (channels, packets) = run(text, &[("numchannels", "2")]);
        assert_eq!(channels.len(), 1);
        assert_eq!(logic(&packets), [0, 0, 0, 0, 0, 1, 1, 1, 1]);
    }

    #[test]
    fn vectors_reals_and_scopes() {
        let text = "$timescale 10 ns $end\n$scope module top $end\n$var wire 1 a clk $end\n$var wire 4 b data [3:0] $end\n$var real 64 c v $end\n$var wire 2 d q $end\n$upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n0a\nb0101 b\nr1.5 c\nb11 d\n$end\n#1\n1a\nbx010 b\nr-2.25 c\n#2\n";
        let (channels, packets) = run(text, &[]);
        let names: Vec<&str> = channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "top.clk",
                "top.data[0]",
                "top.data[1]",
                "top.data[2]",
                "top.data[3]",
                "top.q.0",
                "top.q.1",
                "top.v"
            ]
        );
        assert_eq!(packets[1], Packet::meta(Key::Samplerate, 100_000_000u64));
        let data = logic(&packets);
        // Sample 0: clk 0, data 0101 (bits 1..4), q 11 (bits 5, 6).
        assert_eq!(data, [0b0110_1010, 0b0110_0101]);
        let analog: Vec<f32> = packets
            .iter()
            .filter_map(|p| if let Packet::Analog(a) = p { Some(a.samples.clone()) } else { None })
            .flatten()
            .collect();
        assert_eq!(analog, [1.5, -2.25]);
    }
}
