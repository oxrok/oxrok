// Counterpart in sigrok:
//   libsigrok/src/output/bits.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `bits`: one line per channel, `0` and `1` per sample, a space every
//! eight samples.

use super::{Output, OutputContext, OutputModule, OutputOption, header, meta_samplerate};
use crate::core::{Packet, Result};

const DEFAULT_WIDTH: u32 = 64;

pub(super) static MODULE: OutputModule = OutputModule {
    id: "bits",
    name: "Bits",
    description: "0/1 digits logic data",
    options: &[OutputOption {
        id: "width",
        name: "Width",
        description: "Number of samples per line",
        default: "64",
    }],
    extensions: &["txt"],
    analog: false,
    create: |ctx, options| {
        let width = super::option_u32(options, "width", DEFAULT_WIDTH)? as usize;
        Ok(Box::new(Bits::new(ctx, width)))
    },
};

struct Bits {
    width: usize,
    count: usize,
    trigger: Option<usize>,
    samplerate: Option<u64>,
    channels: Vec<(usize, String)>,
    lines: Vec<String>,
    header: Option<String>,
    total_channels: usize,
}

impl Bits {
    fn new(ctx: &OutputContext, width: usize) -> Self {
        let channels = ctx.enabled_logic();
        let lines = channels.iter().map(|(_, n)| format!("{n}:")).collect();
        Bits {
            // Width 0: the C count never equals it, so no line ends.
            width: if width == 0 { usize::MAX } else { width },
            count: 0,
            trigger: None,
            samplerate: ctx.samplerate,
            total_channels: ctx.channels.len(),
            header: Some(header(ctx, channels.len(), ctx.samplerate)),
            channels,
            lines,
        }
    }

    /// Writes the pending lines. The trigger marker goes out only with
    /// a full line, as in sigrok, whose final flush at the end of the
    /// capture drops it.
    fn flush(&mut self, out: &mut Vec<u8>, with_trigger: bool) {
        for (j, line) in self.lines.iter_mut().enumerate() {
            out.extend_from_slice(line.as_bytes());
            out.push(b'\n');
            if with_trigger
                && j + 1 == self.channels.len()
                && let Some(t) = self.trigger.take()
            {
                let offset = t + t / 8;
                out.extend_from_slice(format!("T:{:offset$}^ {t}\n", "").as_bytes());
            }
            line.clear();
            line.push_str(&self.channels[j].1);
            line.push(':');
        }
    }
}

impl Output for Bits {
    fn receive(&mut self, packet: &Packet, out: &mut Vec<u8>) -> Result<()> {
        match packet {
            Packet::Meta(_) => {
                if let Some(rate) = meta_samplerate(packet) {
                    self.samplerate = Some(rate);
                    if let Some(h) = &mut self.header {
                        *h = header_with_total(self.channels.len(), self.total_channels, rate);
                    }
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
                        let c = if logic.bit(s, *index) { b'1' } else { b'0' };
                        self.lines[j].push(c as char);
                        if self.count != self.width && self.count.is_multiple_of(8) {
                            self.lines[j].push(' ');
                        }
                    }
                    if self.count == self.width {
                        self.flush(out, true);
                        self.count = 0;
                    }
                }
            }
            Packet::End if self.count != 0 => {
                self.flush(out, false);
                self.count = 0;
            }
            _ => {}
        }
        Ok(())
    }
}

fn header_with_total(enabled: usize, total: usize, rate: u64) -> String {
    let mut s =
        format!("oxrok {}\nAcquisition with {enabled}/{total} channels", crate::core::VERSION);
    if rate != 0 {
        s.push_str(" at ");
        s.push_str(&crate::core::util::samplerate_string(rate));
    }
    s.push('\n');
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Channel, ChannelKind};

    fn run(width: usize, unit_size: usize, data: &[u8], channels: &[Channel]) -> String {
        let ctx = OutputContext { channels, samplerate: Some(1000), ..Default::default() };
        let mut o = Bits::new(&ctx, width);
        let mut out = Vec::new();
        o.receive(&Packet::logic(unit_size, data.to_vec()), &mut out).unwrap();
        o.receive(&Packet::End, &mut out).unwrap();
        String::from_utf8(out).unwrap()
    }

    #[test]
    fn groups_of_eight_and_partial_lines() {
        let chans = [Channel::logic(0), Channel::logic(1)];
        // 20 samples: D0 toggles, D1 is always high.
        let data: Vec<u8> = (0..20u8).map(|i| 0b10 | (i & 1)).collect();
        let text = run(64, 1, &data, &chans);
        assert_eq!(
            text,
            format!(
                "oxrok {}\nAcquisition with 2/2 channels at 1 kHz\nD0:01010101 01010101 0101\nD1:11111111 11111111 1111\n",
                crate::core::VERSION
            )
        );
    }

    #[test]
    fn width_zero_never_wraps() {
        let channels = [Channel::logic(0)];
        let text = run(0, 1, &[0xff; 20], &channels);
        assert_eq!(text.lines().count(), 3);
        assert!(text.ends_with("D0:11111111 11111111 1111\n"));
    }

    #[test]
    fn exact_width_has_no_trailing_space() {
        let chans = [Channel::logic(0)];
        let text = run(8, 1, &[1; 16], &chans);
        let lines: Vec<&str> = text.lines().skip(2).collect();
        assert_eq!(lines, ["D0:11111111", "D0:11111111"]);
    }

    #[test]
    fn disabled_and_analog_channels_are_skipped() {
        let mut d1 = Channel::logic(1);
        d1.enabled = false;
        let chans = [Channel::logic(0), d1, Channel::new(2, ChannelKind::Analog, "A0")];
        let text = run(64, 1, &[0b11, 0b01], &chans);
        assert!(text.contains("Acquisition with 1/3 channels"));
        assert!(text.ends_with("D0:11\n"));
    }
}
