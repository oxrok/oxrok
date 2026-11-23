//! Any bytes as an instrument's answer, arriving in reads of any size:
//! [`Scpi::get_block`]'s IEEE 488.2 block header and length, and the
//! text answers ([`Scpi::get_string`], [`Scpi::get_floatv`], `*IDN?`),
//! return a value or an error and do not panic.
//!
//! The first byte chooses the call and the size of a read, the rest is
//! the answer.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxrok::core::Result;
use oxrok::scpi::{Scpi, ScpiTransport, parse_bool, parse_hw_id, unquote_string};
use std::time::Duration;

/// An instrument that answers every question with the same bytes, `step`
/// at a read, and then is silent.
struct Canned {
    answer: Vec<u8>,
    pos: usize,
    step: usize,
}

impl ScpiTransport for Canned {
    fn resource(&self) -> &str {
        "fuzz"
    }
    fn send(&mut self, _data: &[u8]) -> Result<()> {
        Ok(())
    }
    fn read_begin(&mut self) -> Result<()> {
        Ok(())
    }
    fn read_data(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize> {
        let n = self.step.min(buf.len()).min(self.answer.len() - self.pos);
        buf[..n].copy_from_slice(&self.answer[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
    fn read_complete(&self) -> bool {
        self.pos == self.answer.len()
    }
}

fuzz_target!(|data: &[u8]| {
    let [how, answer @ ..] = data else { return };
    let step = match how >> 2 & 3 {
        0 => 1,
        1 => 7,
        2 => 64,
        _ => usize::MAX,
    };
    let mut scpi = Scpi::new(Box::new(Canned { answer: answer.to_vec(), pos: 0, step }));
    // Silence ends an answer at once: nothing here waits on a clock.
    scpi.read_timeout = Duration::ZERO;
    match how & 3 {
        0 => {
            let _ = scpi.get_block(Some(":WAV:DATA?"));
        }
        1 => {
            let _ = scpi.get_floatv(":MEAS?");
        }
        2 => {
            let _ = scpi.hw_id();
        }
        _ => {
            if let Ok(text) = scpi.get_string("?") {
                let _ = parse_hw_id(&text);
                let _ = parse_bool(&text);
                let _ = unquote_string(&text);
            }
        }
    }
});
