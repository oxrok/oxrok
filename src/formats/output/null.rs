// Counterpart in sigrok:
//   libsigrok/src/output/null.c,
//   Copyright (C) 2018 Uwe Hermann <uwe@hermann-uwe.de>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `null`: discards everything.

use super::{Output, OutputModule};
use crate::core::{Packet, Result};

pub(super) static MODULE: OutputModule = OutputModule {
    id: "null",
    name: "Null output",
    description: "Null output (discards all data)",
    options: &[],
    extensions: &[],
    // As in libsigrok, analog packets still go to the front end's analog
    // output: sigrok-cli prints them, and so does oxrok.
    analog: false,
    create: |_, _| Ok(Box::new(Null)),
};

struct Null;

impl Output for Null {
    fn receive(&mut self, _packet: &Packet, _out: &mut Vec<u8>) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::formats::output::{self, OutputContext};

    #[test]
    fn discards_everything() {
        let module = output::find("null").unwrap();
        let mut o = module.instantiate(&OutputContext::default(), &output::Options::new()).unwrap();
        let mut out = Vec::new();
        for p in [Packet::header(), Packet::logic(1, vec![0xff, 0x00]), Packet::End] {
            o.receive(&p, &mut out).unwrap();
        }
        assert_eq!(out, b"");
    }
}
