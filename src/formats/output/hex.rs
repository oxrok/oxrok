// Counterpart in sigrok:
//   libsigrok/src/output/hex.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `hex`: one line per channel, eight samples per hex byte, first sample
//! in the most significant bit.

use super::{Output, OutputContext, OutputModule, OutputOption, header, meta_samplerate};
use crate::core::{Packet, Result};
use std::fmt::Write as _;

const DEFAULT_WIDTH: u32 = 192;

pub(super) static MODULE: OutputModule = OutputModule {
    id: "hex",
    name: "Hexadecimal",
    description: "Hexadecimal digits logic data",
    options: &[OutputOption {
        id: "width",
        name: "Width",
        description: "Number of samples per line",
        default: "192",
    }],
    extensions: &["txt"],
    analog: false,
    create: |ctx, options| {
        let width = super::option_u32(options, "width", DEFAULT_WIDTH)? as usize;
        Ok(Box::new(Hex::new(ctx, width)))
    },
};

struct Hex {
    width: usize,
    count: usize,
    trigger: Option<usize>,
    channels: Vec<(usize, String)>,
    lines: Vec<String>,
    bytes: Vec<u8>,
    header: Option<String>,
    total_channels: usize,
}

impl Hex {
    fn new(ctx: &OutputContext, width: usize) -> Self {
        let channels = ctx.enabled_logic();
        Hex {
            // Width 0: the C count never equals it, so no line ends.
            width: if width == 0 { usize::MAX } else { width },
            count: 0,
            trigger: None,
            lines: channels.iter().map(|(_, n)| format!("{n}:")).collect(),
            bytes: vec![0; channels.len()],
            header: Some(header(ctx, channels.len(), ctx.samplerate)),
            total_channels: ctx.channels.len(),
            channels,
        }
    }

    /// Writes the pending lines; the trigger marker only goes out with
    /// a full line, as in sigrok.
    fn flush(&mut self, out: &mut Vec<u8>, with_trigger: bool) {
        for (j, line) in self.lines.iter_mut().enumerate() {
            out.extend_from_slice(line.as_bytes());
            out.push(b'\n');
            if with_trigger
                && j + 1 == self.channels.len()
                && let Some(t) = self.trigger.take()
            {
                let offset = t / 4 + t / 8;
                out.extend_from_slice(format!("T:{:offset$}^ {t}\n", "").as_bytes());
            }
            line.clear();
            line.push_str(&self.channels[j].1);
            line.push(':');
        }
    }
}

impl Output for Hex {
    fn receive(&mut self, packet: &Packet, out: &mut Vec<u8>) -> Result<()> {
        match packet {
            Packet::Meta(_) => {
                if let (Some(rate), Some(h)) = (meta_samplerate(packet), &mut self.header) {
                    *h = format!(
                        "oxrok {}\nAcquisition with {}/{} channels at {}\n",
                        crate::core::VERSION,
                        self.channels.len(),
                        self.total_channels,
                        crate::core::util::samplerate_string(rate)
                    );
                }
            }
            Packet::Trigger => self.trigger = Some(self.count),
            Packet::Logic(logic) => {
                if let Some(h) = self.header.take() {
                    out.extend_from_slice(h.as_bytes());
                }
                for s in 0..logic.len() {
                    self.count += 1;
                    let pos = self.count % 8;
                    for (j, (index, _)) in self.channels.iter().enumerate() {
                        self.bytes[j] <<= 1;
                        if logic.bit(s, *index) {
                            self.bytes[j] |= 1;
                        }
                        if pos == 0 {
                            let _ = write!(self.lines[j], "{:02x} ", self.bytes[j]);
                            self.bytes[j] = 0;
                        }
                    }
                    if self.count == self.width {
                        self.flush(out, true);
                        self.count = 0;
                    }
                }
            }
            Packet::End if self.count != 0 => {
                let rem = self.count % 8;
                if rem != 0 {
                    for j in 0..self.channels.len() {
                        let _ = write!(self.lines[j], "{:02x} ", self.bytes[j] << (8 - rem));
                        self.bytes[j] = 0;
                    }
                }
                self.flush(out, false);
                self.count = 0;
            }
            _ => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Channel;

    #[test]
    fn packs_msb_first_and_pads_the_last_byte() {
        let chans = [Channel::logic(0)];
        let ctx = OutputContext { channels: &chans, samplerate: None, ..Default::default() };
        let mut o = Hex::new(&ctx, 192);
        let mut out = Vec::new();
        // 20 samples: 1000 1111 0000 1111 1000
        let bits = [1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0];
        o.receive(&Packet::logic(1, bits.to_vec()), &mut out).unwrap();
        o.receive(&Packet::End, &mut out).unwrap();
        let text = String::from_utf8(out).unwrap();
        assert!(text.ends_with("D0:8f 0f 80 \n"), "{text}");
        assert!(text.starts_with(&format!(
            "oxrok {}\nAcquisition with 1/1 channels\n",
            crate::core::VERSION
        )));
    }
}
