// Counterpart in sigrok:
//   sigrok-cli/input.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `-i FILE`: session files and input modules.

use crate::cli::args::Args;
use crate::cli::parse;
use crate::cli::pipeline::Pipeline;
use oxrok::core::{Channel, ChannelKind, Error, Key, Packet, Result};
use oxrok::formats::input::{self, Input, Options};
use oxrok::formats::sr::{SessionReader, is_session_file};
use std::io::Read;
use std::path::Path;

const CHUNK: usize = 4 * 1024 * 1024;

/// Where the packets of a file come from.
enum Source {
    Session(SessionReader<std::fs::File>),
    Module { input: Box<dyn Input>, reader: Box<dyn Read>, title: String },
}

fn open_source(args: &Args) -> Result<Source> {
    let file = args.input_file.as_deref().unwrap_or("-");
    let is_stdin = file == "-";
    if !is_stdin && args.input_format.is_none() && is_session_file(Path::new(file)) {
        return Ok(Source::Session(SessionReader::open(Path::new(file))?));
    }
    let mut reader: Box<dyn Read> = if is_stdin {
        Box::new(std::io::stdin())
    } else {
        Box::new(
            std::fs::File::open(file)
                .map_err(|e| Error::Arg(format!("Failed to load {file}: {e}.")))?,
        )
    };
    let (module, options) = if let Some(spec) = &args.input_format {
        let parsed = parse::generic_args(spec, true);
        let id = parsed.scope.unwrap_or_default();
        let m = input::find(&id)
            .ok_or_else(|| Error::Arg(format!("Error: unknown input module '{id}'.")))?;
        let options: Options =
            parsed.items.into_iter().map(|(k, v)| (k, v.unwrap_or_default())).collect();
        (m, options)
    } else {
        // Detect from the first bytes, then replay them.
        let mut head = vec![0u8; 4096];
        let n = reader.read(&mut head)?;
        head.truncate(n);
        let m = input::detect(&head, if is_stdin { None } else { Some(file) })
            .ok_or_else(|| Error::arg("Error: no input module found for this file."))?;
        reader = Box::new(std::io::Cursor::new(head).chain(reader));
        (m, Options::new())
    };
    let input = module.instantiate(&options)?;
    Ok(Source::Module { input, reader, title: module.name.to_owned() })
}

/// What the reader hands to the consumer.
enum Event<'a> {
    /// The channel list (with `-C` applied) and a title, once, before the
    /// first packet.
    Channels(&'a [Channel], &'a str, Option<u64>),
    /// A packet.
    Packet(&'a Packet),
}

/// Reads the file and runs every event through `handler`. A false return
/// stops the output but drains the file.
fn stream(args: &Args, handler: &mut dyn FnMut(Event) -> bool) -> Result<()> {
    match open_source(args)? {
        Source::Session(mut reader) => {
            if let Some(spec) = &args.channels {
                select(reader.channels_mut(), spec)?;
            }
            let rate = reader.metadata().samplerate;
            handler(Event::Channels(reader.channels(), "Session file", rate));
            let mut stop = false;
            reader.read(&mut |p| {
                if !stop && !handler(Event::Packet(&p)) {
                    stop = true;
                }
            })?;
            crate::cli::strict_check(args)
        }
        Source::Module { mut input, mut reader, title } => {
            let mut packets = Vec::new();
            let mut announced = false;
            let mut buf = vec![0u8; CHUNK];
            let mut stop = false;
            loop {
                let n = reader.read(&mut buf)?;
                if n == 0 {
                    input.end(&mut packets)?;
                } else {
                    input.feed(&buf[..n], &mut packets)?;
                }
                if !announced && let Some(channels) = input.channels() {
                    let mut chans = channels.to_vec();
                    if let Some(spec) = &args.channels {
                        select(&mut chans, spec)?;
                        for (pos, c) in chans.iter().enumerate() {
                            input.enable_channel(pos, c.enabled);
                        }
                    }
                    handler(Event::Channels(&chans, &title, None));
                    announced = true;
                }
                for p in packets.drain(..) {
                    if !stop && !handler(Event::Packet(&p)) {
                        stop = true;
                    }
                }
                // Stop at the first chunk that warned instead of
                // draining the file.
                crate::cli::strict_check(args)?;
                if n == 0 {
                    break;
                }
            }
            if !announced {
                return Err(Error::arg("File import failed: no channels found."));
            }
            Ok(())
        }
    }
}

/// Applies `-C` to a channel list: names or numeric ranges.
fn select(channels: &mut [Channel], spec: &str) -> Result<()> {
    let names: Vec<String> = channels.iter().map(|c| c.name.clone()).collect();
    let mut keep = vec![false; channels.len()];
    for token in spec.split(',') {
        if let Some((lo, hi)) = token.split_once('-')
            && let (Ok(b), Ok(e)) = (lo.parse::<usize>(), hi.parse::<usize>())
        {
            for n in b..=e {
                let pos = names
                    .iter()
                    .position(|x| *x == n.to_string())
                    .ok_or_else(|| Error::Arg(format!("unknown channel '{n}'.")))?;
                keep[pos] = true;
            }
            continue;
        }
        let (name, rename) = token.split_once('=').map_or((token, None), |(n, r)| (n, Some(r)));
        let pos = names
            .iter()
            .position(|x| x == name)
            .ok_or_else(|| Error::Arg(format!("unknown channel '{name}'.")))?;
        keep[pos] = true;
        if let Some(r) = rename {
            r.clone_into(&mut channels[pos].name);
        }
    }
    for (c, k) in channels.iter_mut().zip(keep) {
        c.enabled = k;
    }
    Ok(())
}

/// `-i FILE`: runs the file through the output.
pub fn run(args: &Args) -> Result<()> {
    let mut pipeline: Option<Pipeline> = None;
    let mut pending_samplerate: Option<u64> = None;
    let mut channels: Vec<Channel> = Vec::new();
    let mut title = String::new();
    let mut failed = None;
    stream(args, &mut |event| {
        let packet = match event {
            Event::Channels(chans, t, rate) => {
                channels = chans.to_vec();
                t.clone_into(&mut title);
                pending_samplerate = rate;
                return true;
            }
            Event::Packet(p) => p,
        };
        if let Packet::Meta(items) = packet
            && let Some((_, v)) = items.iter().find(|(k, _)| *k == Key::Samplerate)
        {
            pending_samplerate = v.as_u64();
        }
        if pipeline.is_none() {
            if !matches!(packet, Packet::Header { .. }) {
                return true;
            }
            match Pipeline::new(args, &channels, pending_samplerate, &title) {
                Ok(p) => pipeline = Some(p),
                Err(e) => {
                    failed = Some(e);
                    return false;
                }
            }
        }
        pipeline.as_mut().is_some_and(|p| p.handle(packet))
    })?;
    if let Some(e) = failed {
        return Err(e);
    }
    match pipeline.as_ref().and_then(Pipeline::failure) {
        Some(why) => Err(Error::Arg(why.to_owned())),
        None => Ok(()),
    }
}

/// `-i FILE --show`: the file's properties.
pub fn show_props(args: &Args) -> Result<()> {
    let mut channels: Vec<Channel> = Vec::new();
    let mut samplerate = 0u64;
    let mut unitsize = 0usize;
    let mut logic_samples = 0u64;
    let mut analog_samples = 0u64;
    let mut first_analog: Option<usize> = None;
    let mut frames = 0u64;
    let mut triggers = 0u64;
    stream(args, &mut |event| {
        let packet = match event {
            Event::Channels(chans, _, rate) => {
                channels = chans.to_vec();
                samplerate = rate.unwrap_or(0);
                first_analog = chans
                    .iter()
                    .find(|c| c.enabled && c.kind == ChannelKind::Analog)
                    .map(|c| c.index);
                return true;
            }
            Event::Packet(p) => p,
        };
        match packet {
            Packet::Meta(items) => {
                if let Some((_, v)) = items.iter().find(|(k, _)| *k == Key::Samplerate) {
                    samplerate = v.as_u64().unwrap_or(0);
                }
            }
            Packet::Logic(l) => {
                unitsize = l.unit_size;
                logic_samples += l.len() as u64;
            }
            Packet::Analog(a) => {
                if a.channels.first() == first_analog.as_ref() {
                    analog_samples += a.len() as u64;
                }
            }
            Packet::FrameEnd => frames += 1,
            Packet::Trigger => triggers += 1,
            _ => {}
        }
        true
    })?;
    if samplerate != 0 {
        println!("Samplerate: {samplerate}");
    }
    if !channels.is_empty() {
        println!("Channels: {}", channels.len());
        for c in &channels {
            let kind = if c.kind == ChannelKind::Analog { "analog" } else { "logic" };
            println!("- {}: {kind}", c.name);
        }
    }
    if unitsize != 0 {
        println!("Logic unitsize: {unitsize}");
    }
    if logic_samples != 0 {
        println!("Logic sample count: {logic_samples}");
    }
    if analog_samples != 0 {
        println!("Analog sample count: {analog_samples}");
    }
    if frames != 0 {
        println!("Frame count: {frames}");
    }
    if triggers != 0 {
        println!("Trigger count: {triggers}");
    }
    Ok(())
}
