// Counterpart in sigrok:
//   libsigrok/src/output/ascii.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `ascii`: one line per channel drawn with characters for low, high,
//! falling and rising samples.

use super::{Output, OutputContext, OutputModule, OutputOption, header, meta_samplerate};
use crate::core::{Packet, Result};

const DEFAULT_WIDTH: u32 = 74;
const DEFAULT_CHARSET: &str = ".\"\\/";

pub(super) static MODULE: OutputModule = OutputModule {
    id: "ascii",
    name: "ASCII",
    description: "ASCII art logic data",
    options: &[
        OutputOption {
            id: "width",
            name: "Width",
            description: "Number of samples per line",
            default: "74",
        },
        OutputOption {
            id: "charset",
            name: "Charset",
            description: "Characters for 0/1 bits (and fall/rise edges)",
            default: DEFAULT_CHARSET,
        },
    ],
    extensions: &["txt"],
    analog: false,
    create: |ctx, options| {
        let width = super::option_u32(options, "width", DEFAULT_WIDTH)? as usize;
        let charset = options.get("charset").map_or(DEFAULT_CHARSET, String::as_str);
        Ok(Box::new(Ascii::new(ctx, width, charset)))
    },
};

struct Ascii {
    width: usize,
    count: usize,
    trigger: Option<usize>,
    channels: Vec<(usize, String)>,
    names: Vec<String>,
    name_width: usize,
    lines: Vec<String>,
    prev: Vec<bool>,
    charset: Vec<char>,
    edges: bool,
    header: Option<String>,
    total_channels: usize,
}

impl Ascii {
    fn new(ctx: &OutputContext, width: usize, charset: &str) -> Self {
        let mut chars: Vec<char> = charset.chars().collect();
        if chars.len() < 2 {
            chars = DEFAULT_CHARSET.chars().collect();
        }
        let channels = ctx.enabled_logic();
        let name_width = channels.iter().map(|(_, n)| n.len()).max().unwrap_or(0);
        let names: Vec<String> =
            channels.iter().map(|(_, n)| format!("{n:>name_width$}")).collect();
        Ascii {
            // Width 0: the C count never equals it, so no line ends.
            width: if width == 0 { usize::MAX } else { width },
            count: 0,
            trigger: None,
            lines: names.iter().map(|n| format!("{n}:")).collect(),
            prev: vec![false; channels.len()],
            edges: chars.len() >= 4,
            charset: chars,
            header: Some(header(ctx, channels.len(), ctx.samplerate)),
            total_channels: ctx.channels.len(),
            name_width,
            names,
            channels,
        }
    }

    fn flush(&mut self, out: &mut Vec<u8>) {
        for (j, line) in self.lines.iter_mut().enumerate() {
            out.extend_from_slice(line.as_bytes());
            out.push(b'\n');
            if j + 1 == self.channels.len()
                && let Some(t) = self.trigger.take()
            {
                let w = self.name_width;
                out.extend_from_slice(
                    format!("{:>w$}:{:>pad$} {t}\n", "T", "^", pad = t + 1).as_bytes(),
                );
            }
            line.clear();
            line.push_str(&self.names[j]);
            line.push(':');
        }
    }
}

impl Output for Ascii {
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
                    for (j, (index, _)) in self.channels.iter().enumerate() {
                        let cur = logic.bit(s, *index);
                        let mut idx = usize::from(cur);
                        if self.edges && self.count > 1 && cur != self.prev[j] {
                            idx += 2;
                        }
                        self.lines[j].push(self.charset[idx]);
                        self.prev[j] = cur;
                    }
                    if self.count == self.width {
                        self.flush(out);
                        self.count = 0;
                    }
                }
            }
            Packet::End if self.count != 0 => {
                self.flush(out);
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
    fn draws_edges_and_aligns_names() {
        let chans = [Channel::logic(0), Channel::new(10, crate::core::ChannelKind::Logic, "D10")];
        let ctx = OutputContext { channels: &chans, samplerate: None, ..Default::default() };
        let mut o = Ascii::new(&ctx, 74, DEFAULT_CHARSET);
        let mut out = Vec::new();
        // D0: 0 1 1 0; D10 (bit 10): 1 1 0 0.
        let data = [0x00, 0x04, 0x01, 0x04, 0x01, 0x00, 0x00, 0x00];
        o.receive(&Packet::logic(2, data.to_vec()), &mut out).unwrap();
        o.receive(&Packet::End, &mut out).unwrap();
        let text = String::from_utf8(out).unwrap();
        let lines: Vec<&str> = text.lines().skip(2).collect();
        assert_eq!(lines, [" D0:./\"\\", "D10:\"\"\\."]);
    }
}
