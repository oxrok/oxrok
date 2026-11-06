// Counterpart in sigrok:
//   sigrok-cli/main.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `oxrok`: a sigrok-cli compatible command line for oxrok.
//!
//! The flags and the output are the ones sigrok-cli users know, so
//! existing scripts keep working. Where sigrok-cli had rough edges (one
//! trigger stage, rounded option listings) this tool does better while
//! staying a superset.

#![forbid(unsafe_code)]

mod args;
mod decode;
mod input;
mod logger;
mod parse;
mod pipeline;
mod run;
mod show;

use clap::Parser;
use oxrok::core::Registry;
use std::process::ExitCode;

/// Every driver this binary ships.
fn registry() -> Registry {
    let mut r = Registry::new();
    oxrok::register_all(&mut r);
    r
}

/// With `--strict`, a run that logged warnings fails.
pub fn strict_check(args: &args::Args) -> oxrok::core::Result<()> {
    let n = logger::warnings();
    if args.strict && n > 0 {
        let s = if n == 1 { "" } else { "s" };
        return Err(oxrok::core::Error::Arg(format!(
            "{n} warning{s} treated as error{s} (--strict)."
        )));
    }
    Ok(())
}

pub fn main() -> ExitCode {
    let args = args::Args::parse();
    logger::init(args.loglevel);
    let registry = registry();
    let result = if args.version {
        show::version(&registry);
        Ok(())
    } else if args.list_supported {
        show::supported(&registry);
        Ok(())
    } else if args.input_file.is_some() && args.show {
        input::show_props(&args)
    } else if args.input_format.is_some() && args.show {
        show::input_detail(&args)
    } else if args.output_format.is_some() && args.show {
        show::output_detail(&args)
    } else if !args.decoders.is_empty() && args.show {
        decode::decoder_detail(&args)
    } else if args.scan {
        show::device_list(&registry, &args)
    } else if args.show {
        show::device_detail(&registry, &args)
    } else if args.input_file.is_some() {
        input::run(&args)
    } else if !args.gets.is_empty() {
        run::get_options(&registry, &args)
    } else if args.set {
        run::set_options(&registry, &args)
    } else if args.samples.is_some()
        || args.time.is_some()
        || args.frames.is_some()
        || args.continuous
        || run::starts_without_limit(&registry, &args)
    {
        run::session(&registry, &args)
    } else {
        show::help();
        Ok(())
    };
    // A failed capture is still reported last, and `--strict` has its say first.
    let result = match result {
        Err(failed @ oxrok::core::Error::CaptureFailed(_)) => {
            if let Err(oxrok::core::Error::Arg(msg)) = strict_check(&args) {
                eprintln!("{msg}");
            }
            Err(failed)
        }
        other => other.and_then(|()| strict_check(&args)),
    };
    // A replay from OXROK_USB_REPLAY or OXROK_HID_REPLAY that was not
    // played to its end is a run that went otherwise than recorded.
    let result = result.and_then(|()| oxrok::core::sessions::finish_replays());
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(oxrok::core::Error::Arg(msg)) => {
            eprintln!("{msg}");
            ExitCode::FAILURE
        }
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
