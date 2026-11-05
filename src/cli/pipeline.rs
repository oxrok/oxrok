// Counterpart in sigrok:
//   sigrok-cli/session.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The output side shared by captures and file inputs: builds the output
//! module, feeds it packets, writes the text.

use crate::cli::args::Args;
use crate::cli::decode::Decoding;
use crate::cli::parse;
use oxrok::core::{Channel, Error, Packet, Result};
use oxrok::formats::output::{self, Options, Output, OutputContext};
use std::io::{ErrorKind, Write};

const DEFAULT_OUTPUT_NOFILE: &str = "bits:width=64";
const DEFAULT_OUTPUT_FILE: &str = "srzip";

/// A running output pipeline.
pub struct Pipeline {
    /// The output module; none when only decoders were asked for.
    main: Option<Box<dyn Output>>,
    analog: Option<Box<dyn Output>>,
    decoding: Option<Decoding>,
    sink: Option<Box<dyn Write>>,
    buf: Vec<u8>,
    wait_trigger: bool,
    triggered: bool,
    /// Why the output can no longer be written, once it cannot.
    failure: Option<String>,
    /// Logic and analog samples received.
    pub received_logic: u64,
    pub received_analog: u64,
}

/// A write to a reader that has gone: no fault of the output's.
fn reader_gone(e: &Error) -> bool {
    matches!(e, Error::Io(io) if io.kind() == ErrorKind::BrokenPipe)
}

impl Pipeline {
    /// Builds the output named by `-O` (or the default) for `channels`.
    pub fn new(
        args: &Args,
        channels: &[Channel],
        samplerate: Option<u64>,
        title: &str,
    ) -> Result<Self> {
        let decoding = Decoding::from_args(args, channels, samplerate)?;
        // With decoders and no output asked for, only annotations print.
        if decoding.is_some() && args.output_format.is_none() && args.output_file.is_none() {
            return Ok(Pipeline {
                main: None,
                analog: None,
                decoding,
                sink: Some(Box::new(std::io::stdout())),
                buf: Vec::new(),
                wait_trigger: args.wait_trigger,
                triggered: false,
                failure: None,
                received_logic: 0,
                received_analog: 0,
            });
        }
        let spec = args.output_format.clone().unwrap_or_else(|| {
            if args.output_file.is_some() { DEFAULT_OUTPUT_FILE } else { DEFAULT_OUTPUT_NOFILE }
                .to_owned()
        });
        let parsed = parse::generic_args(&spec, true);
        let id = parsed.scope.ok_or_else(|| Error::arg("Invalid output format."))?;
        let module = output::find(&id)
            .ok_or_else(|| Error::Arg(format!("Unknown output module '{id}'.")))?;
        let options: Options =
            parsed.items.into_iter().map(|(k, v)| (k, v.unwrap_or_default())).collect();
        let ctx = OutputContext {
            channels,
            samplerate,
            title: title.to_owned(),
            filename: args.output_file.as_ref().map(|p| p.display().to_string()),
        };
        let main = module.instantiate(&ctx, &options)?;
        let analog = if module.analog { None } else { Some(output::analog_fallback(&ctx)) };
        let sink: Option<Box<dyn Write>> = if main.writes_file() {
            None
        } else {
            Some(match &args.output_file {
                Some(path) => {
                    Box::new(std::io::BufWriter::new(std::fs::File::create(path).map_err(|e| {
                        Error::Arg(format!("Cannot write to output file '{}': {e}", path.display()))
                    })?))
                }
                None => Box::new(std::io::stdout()),
            })
        };
        Ok(Pipeline {
            main: Some(main),
            analog,
            decoding,
            sink,
            buf: Vec::new(),
            wait_trigger: args.wait_trigger,
            triggered: false,
            failure: None,
            received_logic: 0,
            received_analog: 0,
        })
    }

    /// Handles one packet. False when the output can no longer be written,
    /// or its reader has gone, which only stops the run: `oxrok ... | head`.
    pub fn handle(&mut self, packet: &Packet) -> bool {
        match packet {
            // For the run's result, not for the outputs.
            Packet::Failed(_) => return true,
            Packet::Trigger => self.triggered = true,
            Packet::Logic(l) => self.received_logic += l.len() as u64,
            Packet::Analog(a) => self.received_analog += a.len() as u64,
            _ => {}
        }
        if self.wait_trigger && !self.triggered && matches!(packet, Packet::Logic(_)) {
            return true;
        }
        if let Some(d) = &mut self.decoding {
            let mut stdout = std::io::stdout();
            if let Err(e) = d.handle(packet, &mut stdout) {
                if !reader_gone(&e) {
                    log::error!("decoding: {e}");
                    self.failure.get_or_insert_with(|| format!("decoding: {e}"));
                }
                return false;
            }
        }
        self.buf.clear();
        let Some(main) = &mut self.main else { return true };
        match main.receive(packet, &mut self.buf) {
            Ok(()) => {}
            Err(e) => {
                if !reader_gone(&e) {
                    log::error!("output: {e}");
                    self.failure.get_or_insert_with(|| format!("output: {e}"));
                }
                return false;
            }
        }
        if self.buf.is_empty()
            && let Some(a) = &mut self.analog
        {
            let _ = a.receive(packet, &mut self.buf);
        }
        if !self.buf.is_empty()
            && let Some(sink) = &mut self.sink
            && let Err(e) = sink.write_all(&self.buf).and_then(|()| sink.flush())
        {
            if e.kind() != ErrorKind::BrokenPipe {
                self.failure.get_or_insert_with(|| format!("output: {e}"));
            }
            return false;
        }
        true
    }

    /// The reason the output could no longer be written, if so.
    pub fn failure(&self) -> Option<&str> {
        self.failure.as_deref()
    }

    /// Warns when fewer samples than asked arrived.
    pub fn check_limit(&self, limit: u64) {
        if limit == 0 {
            return;
        }
        if self.received_logic > 0 && self.received_logic < limit {
            log::warn!("Device only sent {} samples.", self.received_logic);
        } else if self.received_analog > 0 && self.received_analog < limit {
            log::warn!("Device only sent {} samples.", self.received_analog);
        }
    }
}
