// Counterpart in sigrok:
//   sigrok-cli/decode.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Gerhard Sittig, Uwe Hermann and others (see the
//   sigrok-cli history).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Protocol decoding in the command line: `-P`, `-A`, `-B` and `-M`.

use crate::cli::args::Args;
use crate::cli::parse;
use oxrok::core::{Channel, Error, Packet, Result};
use oxrok::decode::{DecoderInfo, Output, Registry, Stack};
use oxrok::setup::decoders::{self as stacks, StackPlan, Visible};
use std::fmt::Write as _;
use std::io::Write;
use std::sync::mpsc::Receiver;

/// Every decoder this binary ships.
pub fn registry() -> Registry {
    oxrok::decoder_registry()
}

/// The decoders of one run and what to print.
#[allow(clippy::struct_excessive_bools)]
pub struct Decoding {
    /// The parsed stacks until the first logic packet starts them.
    plan: Option<StackPlan>,
    /// The instances, kept after the plan was spent.
    instances: Vec<(String, &'static DecoderInfo)>,
    stack: Option<Stack>,
    outputs: Option<Receiver<Output>>,
    ann_visible: Visible,
    binary: Option<(&'static str, usize)>,
    meta_visible: Vec<&'static str>,
    show_snum: bool,
    show_quotes: bool,
    show_class: bool,
    show_abbrev: bool,
    samplerate: u64,
    ended: usize,
}

impl Decoding {
    /// Parses the decoder flags; `None` without `-P`.
    pub fn from_args(
        args: &Args,
        channels: &[Channel],
        samplerate: Option<u64>,
    ) -> Result<Option<Self>> {
        if args.decoders.is_empty() {
            return Ok(None);
        }
        let plan = stacks::parse_stacks(&registry(), &args.decoders, channels)?;
        let all_by_default = args.decoder_binary.is_none() && args.decoder_meta.is_empty();
        let ann_visible = stacks::visible_annotations(
            &plan.instances,
            &args.decoder_annotations,
            all_by_default,
        )?;
        let binary = stacks::binary_class(&plan.instances, args.decoder_binary.as_deref())?;
        let meta_visible = stacks::meta_decoders(&plan.instances, &args.decoder_meta)?;
        let instances = plan.instances.iter().map(|i| (i.id.clone(), i.info)).collect();
        let verbose = args.loglevel > 2;
        Ok(Some(Decoding {
            plan: Some(plan),
            instances,
            stack: None,
            outputs: None,
            ann_visible,
            binary,
            meta_visible,
            show_snum: args.decoder_samplenum || verbose,
            show_quotes: verbose,
            show_class: args.loglevel > 3 || args.decoder_ann_class,
            show_abbrev: args.loglevel > 3,
            samplerate: samplerate.unwrap_or(0),
            ended: 0,
        }))
    }

    fn start(&mut self, unit_size: usize) -> Result<()> {
        let Some(plan) = self.plan.take() else { return Ok(()) };
        let mut stack = plan.build(unit_size, self.samplerate)?;
        self.outputs = stack.outputs();
        self.stack = Some(stack);
        Ok(())
    }

    /// Feeds a packet; prints what the decoders produced so far.
    pub fn handle(&mut self, packet: &Packet, out: &mut dyn Write) -> Result<()> {
        match packet {
            Packet::Meta(items) => {
                // File inputs report the sample rate after the header.
                if self.stack.is_none()
                    && let Some((_, v)) =
                        items.iter().find(|(k, _)| *k == oxrok::core::Key::Samplerate)
                    && let Some(rate) = v.as_u64()
                {
                    self.samplerate = rate;
                }
                Ok(())
            }
            Packet::Logic(l) => {
                if self.stack.is_none() {
                    self.start(l.unit_size)?;
                }
                if let Some(s) = &mut self.stack {
                    s.feed(&l.data);
                }
                self.drain(out, false)
            }
            Packet::End => {
                if self.stack.is_none() {
                    self.start(1)?;
                }
                if let Some(s) = &mut self.stack {
                    s.end();
                }
                self.drain(out, true)?;
                if let Some(s) = &mut self.stack {
                    s.join();
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// Prints available output; with `all`, waits until every instance
    /// has ended.
    fn drain(&mut self, out: &mut dyn Write, all: bool) -> Result<()> {
        let Some(rx) = &self.outputs else { return Ok(()) };
        loop {
            // `recv` ends when every decoder thread is gone, so a thread
            // that died without its End marker cannot hang the drain.
            let next = if all { rx.recv().ok() } else { rx.try_recv().ok() };
            let Some(o) = next else { return Ok(()) };
            self.print(&o, out)?;
            if matches!(o, Output::End { .. }) {
                self.ended += 1;
                if all && self.ended >= self.instances.len() {
                    return Ok(());
                }
            }
        }
    }

    fn info_of(&self, instance: &str) -> Option<&'static DecoderInfo> {
        self.instances.iter().find(|(id, _)| id == instance).map(|(_, info)| *info)
    }

    fn print(&self, o: &Output, out: &mut dyn Write) -> Result<()> {
        match o {
            Output::Annotation { instance, ss, es, class, texts } => {
                let Some(info) = self.info_of(instance) else { return Ok(()) };
                let Some(visible) = self.ann_visible.get(info.id) else { return Ok(()) };
                if let Some(list) = visible
                    && !list.contains(class)
                {
                    return Ok(());
                }
                let mut line = String::new();
                if self.show_snum {
                    let _ = write!(line, "{ss}-{es} ");
                }
                let _ = write!(line, "{instance}: ");
                if self.show_class {
                    let desc = info.annotations.get(*class).map_or("?", |a| a.id);
                    let _ = write!(line, "{desc}: ");
                }
                let q = if self.show_quotes { "\"" } else { "" };
                let shown: &[String] =
                    if self.show_abbrev { texts } else { &texts[..texts.len().min(1)] };
                for (i, t) in shown.iter().enumerate() {
                    if i > 0 {
                        line.push(' ');
                    }
                    let _ = write!(line, "{q}{t}{q}");
                }
                line.push('\n');
                out.write_all(line.as_bytes())?;
                out.flush()?;
            }
            Output::Binary { instance, class, data, .. } => {
                if let Some((dec, idx)) = self.binary
                    && self.info_of(instance).is_some_and(|i| i.id == dec)
                    && *class == idx
                {
                    out.write_all(data)?;
                    out.flush()?;
                }
            }
            Output::Meta { instance, ss, es, name, item } => {
                if self.info_of(instance).is_some_and(|i| self.meta_visible.contains(&i.id)) {
                    let mut line = String::new();
                    if self.show_snum {
                        let _ = write!(line, "{ss}-{es} ");
                    }
                    let _ = writeln!(line, "{instance}: {name}: {item}");
                    out.write_all(line.as_bytes())?;
                }
            }
            Output::Python { .. } => {}
            Output::End { instance, error } => {
                if let Some(e) = error {
                    log::error!("{instance}: {e}");
                }
            }
        }
        Ok(())
    }
}

/// `-P decoder --show`.
pub fn decoder_detail(args: &Args) -> Result<()> {
    let spec = args.decoders.first().map(String::as_str).unwrap_or_default();
    let id = parse::generic_args(spec.split(',').next().unwrap_or_default(), true)
        .scope
        .unwrap_or_default();
    let registry = registry();
    let def = registry
        .find(&id)
        .ok_or_else(|| Error::Arg(format!("Protocol decoder '{id}' not found.")))?;
    let d = def.info;
    println!(
        "ID: {}\nName: {}\nLong name: {}\nDescription: {}\nLicense: {}",
        d.id, d.name, d.longname, d.desc, d.license
    );
    println!("Possible decoder input IDs:");
    for i in d.inputs {
        println!("- {i}");
    }
    println!("Possible decoder output IDs:");
    for o in d.outputs {
        println!("- {o}");
    }
    println!("Decoder tags:");
    for t in d.tags {
        println!("- {t}");
    }
    println!("Annotation classes:");
    for a in d.annotations {
        println!("- {}: {}", a.id, a.desc);
    }
    println!("Annotation rows:");
    for r in d.annotation_rows {
        let classes: Vec<&str> =
            r.classes.iter().filter_map(|&c| d.annotations.get(c).map(|a| a.id)).collect();
        println!("- {} ({}): {}", r.id, r.desc, classes.join(", "));
    }
    println!("Binary classes:");
    for b in d.binary {
        println!("- {}: {}", b.id, b.desc);
    }
    println!("Required channels:");
    if d.channels.is_empty() {
        println!("None.");
    }
    for c in d.channels {
        println!("- {} ({}): {}", c.id, c.name, c.desc);
    }
    println!("Optional channels:");
    if d.optional_channels.is_empty() {
        println!("None.");
    }
    for c in d.optional_channels {
        println!("- {} ({}): {}", c.id, c.name, c.desc);
    }
    println!("Options:");
    for o in d.options {
        if o.values.is_empty() {
            println!("- {}: {} (default {})", o.id, o.desc, o.default.show());
        } else {
            let values: Vec<String> =
                o.values.iter().map(oxrok::decode::OptionValue::show).collect();
            println!(
                "- {}: {} ({}, default {})",
                o.id,
                o.desc,
                values.join(", "),
                o.default.show()
            );
        }
    }
    println!("Documentation:\n{}", d.doc);
    Ok(())
}
