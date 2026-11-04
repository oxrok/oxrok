// Counterpart in sigrok:
//   libsigrok/src/output/csv.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `csv`: comma separated values, one row per sample.
//!
//! The options and the layout follow libsigrok's csv output: an optional
//! comment header, an optional label row (`units`, `channel` or `off`),
//! an optional time column in the unit that fits the sample rate, and an
//! optional trigger column.

use super::{Options, Output, OutputContext, OutputModule, OutputOption, meta_samplerate};
use crate::core::analog::unit_string;
use crate::core::util::{fmt_g, samplerate_string};
use crate::core::{ChannelKind, Packet, Result};
use std::fmt::Write as _;

pub(super) static MODULE: OutputModule = OutputModule {
    id: "csv",
    name: "CSV",
    description: "Comma-separated values",
    options: &[
        OutputOption {
            id: "value",
            name: "Value separator",
            description: "Character to print between values",
            default: ",",
        },
        OutputOption {
            id: "record",
            name: "Record separator",
            description: "String to print between records",
            default: "\n",
        },
        OutputOption {
            id: "frame",
            name: "Frame separator",
            description: "String to print between frames",
            default: "\n",
        },
        OutputOption {
            id: "comment",
            name: "Comment start string",
            description: "String used at start of comment lines",
            default: ";",
        },
        OutputOption {
            id: "header",
            name: "Output header",
            description: "Output header comment with capture metadata",
            default: "true",
        },
        OutputOption {
            id: "label",
            name: "Label values",
            description: "Type of column labels: units, channel or off",
            default: "units",
        },
        OutputOption {
            id: "time",
            name: "Time column",
            description: "Output sample time as column 1",
            default: "false",
        },
        OutputOption {
            id: "trigger",
            name: "Trigger column",
            description: "Output trigger indicator as last column",
            default: "false",
        },
        OutputOption {
            id: "dedup",
            name: "Dedup rows",
            description: "Skip rows equal to the previous one (needs time)",
            default: "false",
        },
    ],
    extensions: &["csv"],
    analog: true,
    create: |ctx, options| Ok(Box::new(Csv::new(ctx, options)?)),
};

const XLABELS: [&str; 7] = [
    "samples",
    "milliseconds",
    "microseconds",
    "nanoseconds",
    "picoseconds",
    "femtoseconds",
    "attoseconds",
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Label {
    Units,
    Channel,
    Off,
}

struct Column {
    index: usize,
    kind: ChannelKind,
    name: String,
    /// The unit text once an analog packet told us.
    unit: Option<String>,
}

#[allow(clippy::struct_excessive_bools)]
struct Csv {
    value: String,
    record: String,
    /// Written at each frame begin, before the rows still pending.
    frame: String,
    comment: String,
    header: bool,
    label: Label,
    time: bool,
    trigger: bool,
    dedup: bool,
    columns: Vec<Column>,
    total_channels: usize,
    title: String,
    samplerate: u64,
    scale: u64,
    xlabel: &'static str,
    header_done: bool,
    label_done: bool,
    triggered: bool,
    out_samples: u64,
    /// Pending logic rows: one Vec<bool> per sample, in column order.
    logic: Vec<Vec<bool>>,
    /// Pending analog values per analog column.
    analog: Vec<Vec<f32>>,
    /// Which columns delivered data since the last dump.
    seen_logic: bool,
    seen_analog: usize,
    previous: Option<(Vec<bool>, Vec<f32>)>,
}

impl Csv {
    fn new(ctx: &OutputContext, options: &Options) -> Result<Self> {
        let get = |id: &str, default: &str| {
            options.get(id).cloned().unwrap_or_else(|| default.to_owned())
        };
        let flag = |id: &str, default: bool| super::option_bool(options, id, default);
        let label = match get("label", "units").as_str() {
            "channel" => Label::Channel,
            "off" => Label::Off,
            _ => Label::Units,
        };
        let time = flag("time", false)?;
        let columns: Vec<Column> = ctx
            .channels
            .iter()
            .filter(|c| c.enabled)
            .map(|c| Column { index: c.index, kind: c.kind, name: c.name.clone(), unit: None })
            .collect();
        let analog_count = columns.iter().filter(|c| c.kind == ChannelKind::Analog).count();
        let mut csv = Csv {
            value: get("value", ","),
            record: get("record", "\n"),
            frame: get("frame", "\n"),
            comment: get("comment", ";"),
            header: flag("header", true)?,
            label,
            time,
            trigger: flag("trigger", false)?,
            dedup: flag("dedup", false)? && time,
            columns,
            total_channels: ctx.channels.len(),
            title: ctx.title.clone(),
            samplerate: 0,
            scale: 1,
            xlabel: XLABELS[0],
            header_done: false,
            label_done: false,
            triggered: false,
            out_samples: 0,
            logic: Vec::new(),
            analog: vec![Vec::new(); analog_count],
            seen_logic: false,
            seen_analog: 0,
            previous: None,
        };
        csv.set_samplerate(ctx.samplerate.unwrap_or(0));
        Ok(csv)
    }

    fn set_samplerate(&mut self, rate: u64) {
        self.samplerate = rate;
        let mut i = 0;
        self.scale = 1;
        while self.scale < rate && i + 1 < XLABELS.len() {
            i += 1;
            self.scale *= 1000;
        }
        self.xlabel = XLABELS[i];
    }

    fn write_header(&mut self, out: &mut Vec<u8>) {
        if !self.header || self.header_done {
            return;
        }
        self.header_done = true;
        let mut s = String::new();
        let date = chrono::Local::now().format("%a %b %e %H:%M:%S %Y").to_string();
        let _ = writeln!(s, "{} CSV generated by oxrok {}", self.comment, crate::core::VERSION);
        let _ = writeln!(s, "{} from {} on {date}", self.comment, self.title);
        let _ = write!(
            s,
            "{} Channels ({}/{}):",
            self.comment,
            self.columns.len(),
            self.total_channels
        );
        let names: Vec<&str> = self.columns.iter().map(|c| c.name.as_str()).collect();
        if !names.is_empty() {
            s.push(' ');
            s.push_str(&names.join(", "));
        }
        s.push('\n');
        if self.samplerate != 0 {
            let _ =
                writeln!(s, "{} Samplerate: {}", self.comment, samplerate_string(self.samplerate));
        }
        out.extend_from_slice(s.as_bytes());
    }

    fn write_labels(&mut self, out: &mut Vec<u8>) {
        if self.label == Label::Off || self.label_done {
            return;
        }
        self.label_done = true;
        let mut fields: Vec<String> = Vec::new();
        if self.time {
            fields.push(if self.label == Label::Channel {
                "Time".to_owned()
            } else {
                self.xlabel.to_owned()
            });
        }
        for c in &self.columns {
            fields.push(match (self.label, c.kind) {
                (Label::Channel, _) => c.name.clone(),
                (_, ChannelKind::Logic) => "logic".to_owned(),
                (_, ChannelKind::Analog) => c.unit.clone().unwrap_or_default(),
            });
        }
        if self.trigger {
            fields.push("Trigger".to_owned());
        }
        out.extend_from_slice(fields.join(&self.value).as_bytes());
        out.extend_from_slice(self.record.as_bytes());
    }

    fn analog_column(&self, index: usize) -> Option<usize> {
        self.columns.iter().filter(|c| c.kind == ChannelKind::Analog).position(|c| c.index == index)
    }

    fn dump(&mut self, out: &mut Vec<u8>) {
        let n_logic = self.logic.len();
        let n_analog = self.analog.iter().map(Vec::len).max().unwrap_or(0);
        let rows = n_logic.max(n_analog);
        if rows == 0 {
            self.reset_pending();
            return;
        }
        self.write_labels(out);
        let analog_cols: Vec<usize> = (0..self.columns.len())
            .filter(|i| self.columns[*i].kind == ChannelKind::Analog)
            .collect();
        let mut text = String::new();
        for i in 0..rows {
            let logic_row: Vec<bool> = self.logic.get(i).cloned().unwrap_or_default();
            let analog_row: Vec<f32> =
                self.analog.iter().map(|a| a.get(i).copied().unwrap_or(0.0)).collect();
            self.out_samples += 1;
            if self.dedup {
                let same = self
                    .previous
                    .as_ref()
                    .is_some_and(|(l, a)| *l == logic_row && *a == analog_row);
                let last = i + 1 == rows;
                if same && i > 0 && !last {
                    continue;
                }
                self.previous = Some((logic_row.clone(), analog_row.clone()));
            }
            let mut fields: Vec<String> = Vec::new();
            if self.time {
                if self.samplerate == 0 {
                    fields.push("0".to_owned());
                } else {
                    // The first row is at time 0 (libsigrok 0.6; 0.5
                    // started one period late).
                    let t =
                        (self.out_samples - 1) as f64 / self.samplerate as f64 * self.scale as f64;
                    fields.push(format!("{}", t as u64));
                }
            }
            let mut li = 0;
            for (ci, c) in self.columns.iter().enumerate() {
                match c.kind {
                    ChannelKind::Logic => {
                        fields.push(
                            if logic_row.get(li).copied().unwrap_or(false) { "1" } else { "0" }
                                .to_owned(),
                        );
                        li += 1;
                    }
                    ChannelKind::Analog => {
                        let ai = analog_cols.iter().position(|x| *x == ci).unwrap_or(0);
                        fields.push(fmt_g(f64::from(analog_row[ai]), 6));
                    }
                }
            }
            if self.trigger {
                fields.push(if self.triggered { "1" } else { "0" }.to_owned());
                self.triggered = false;
            }
            text.push_str(&fields.join(&self.value));
            text.push_str(&self.record);
        }
        out.extend_from_slice(text.as_bytes());
        self.reset_pending();
    }

    fn reset_pending(&mut self) {
        self.logic.clear();
        for a in &mut self.analog {
            a.clear();
        }
        self.seen_logic = false;
        self.seen_analog = 0;
    }

    fn all_seen(&self) -> bool {
        let has_logic = self.columns.iter().any(|c| c.kind == ChannelKind::Logic);
        let analog_count = self.analog.len();
        (!has_logic || self.seen_logic) && self.seen_analog >= analog_count
    }
}

impl Output for Csv {
    fn receive(&mut self, packet: &Packet, out: &mut Vec<u8>) -> Result<()> {
        match packet {
            Packet::Header { .. } => self.write_header(out),
            Packet::Meta(_) => {
                if let Some(rate) = meta_samplerate(packet) {
                    self.set_samplerate(rate);
                    if !self.header_done {
                        self.write_header(out);
                    }
                }
            }
            Packet::Trigger => self.triggered = true,
            Packet::Logic(logic) => {
                self.write_header(out);
                let indices: Vec<usize> = self
                    .columns
                    .iter()
                    .filter(|c| c.kind == ChannelKind::Logic)
                    .map(|c| c.index)
                    .collect();
                for s in 0..logic.len() {
                    self.logic.push(indices.iter().map(|i| logic.bit(s, *i)).collect());
                }
                self.seen_logic = true;
            }
            Packet::Analog(a) => {
                self.write_header(out);
                let n = a.channels.len();
                for (c, index) in a.channels.iter().enumerate() {
                    let Some(col) = self.analog_column(*index) else { continue };
                    if let Some(column) =
                        self.columns.iter_mut().filter(|c| c.kind == ChannelKind::Analog).nth(col)
                    {
                        column.unit = Some(unit_string(a.unit, a.flags));
                    }
                    for s in 0..a.len() {
                        self.analog[col].push(a.samples[s * n + c]);
                    }
                    self.seen_analog += 1;
                }
            }
            // csv.c's receive(): SR_DF_FRAME_BEGIN outputs the frame
            // separator (header or not, the first frame too), then falls
            // through to SR_DF_END, which dumps the rows still pending.
            Packet::FrameBegin | Packet::End => {
                if matches!(packet, Packet::FrameBegin) {
                    out.extend_from_slice(self.frame.as_bytes());
                }
                if self.seen_logic || self.seen_analog > 0 {
                    self.dump(out);
                }
                return Ok(());
            }
            Packet::FrameEnd => {}
        }
        if self.all_seen() {
            self.dump(out);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Channel;

    fn ctx(chans: &[Channel]) -> OutputContext<'_> {
        OutputContext { channels: chans, samplerate: Some(200_000), ..Default::default() }
    }

    #[test]
    fn a_flag_is_on_off_or_refused() {
        let new = |value: &str| {
            let chans = [Channel::logic(0)];
            let opts = Options::from([("header".to_owned(), value.to_owned())]);
            Csv::new(&ctx(&chans), &opts).map(|c| c.header)
        };
        assert!(new("yes").unwrap());
        assert!(new("on").unwrap());
        assert!(!new("no").unwrap());
        assert!(!new("0").unwrap());
        // No value: the default, as sigrok-cli for `-O csv:header`.
        assert!(new("").unwrap());
        for bad in ["maybe", "1x", "enable"] {
            let e = new(bad).err().unwrap().to_string();
            assert!(e.contains(&format!("Unable to convert '{bad}' to boolean!")), "{e}");
        }
    }

    #[test]
    fn logic_rows_with_labels_time_and_trigger() {
        let chans = [Channel::logic(0), Channel::logic(1)];
        let mut opts = Options::new();
        opts.insert("header".into(), "false".into());
        opts.insert("time".into(), "true".into());
        opts.insert("trigger".into(), "true".into());
        let mut c = Csv::new(&ctx(&chans), &opts).unwrap();
        let mut out = Vec::new();
        c.receive(&Packet::Trigger, &mut out).unwrap();
        c.receive(&Packet::logic(1, vec![0b01, 0b10, 0b11]), &mut out).unwrap();
        c.receive(&Packet::End, &mut out).unwrap();
        assert_eq!(
            String::from_utf8(out).unwrap(),
            "microseconds,logic,logic,Trigger\n0,1,0,1\n5,0,1,0\n10,1,1,0\n"
        );
    }

    #[test]
    fn every_frame_begin_writes_the_frame_separator() {
        let chans = [Channel::logic(0)];
        let mut opts = Options::new();
        opts.insert("header".into(), "false".into());
        opts.insert("label".into(), "off".into());
        let run = |opts: &Options| {
            let mut c = Csv::new(&ctx(&chans), opts).unwrap();
            let mut out = Vec::new();
            for p in [
                Packet::header(),
                Packet::FrameBegin,
                Packet::logic(1, vec![1, 0]),
                Packet::FrameEnd,
                Packet::FrameBegin,
                Packet::logic(1, vec![1]),
                Packet::FrameEnd,
                Packet::End,
            ] {
                c.receive(&p, &mut out).unwrap();
            }
            String::from_utf8(out).unwrap()
        };
        // The first frame too, and nothing at a frame's end.
        assert_eq!(run(&opts), "\n1\n0\n\n1\n");
        opts.insert("frame".into(), "--\n".into());
        assert_eq!(run(&opts), "--\n1\n0\n--\n1\n");
        // Rows still waiting for an analog column come after it.
        let chans = [Channel::logic(0), Channel::new(1, ChannelKind::Analog, "A0")];
        let mut c = Csv::new(&ctx(&chans), &opts).unwrap();
        let mut out = Vec::new();
        c.receive(&Packet::logic(1, vec![1]), &mut out).unwrap();
        c.receive(&Packet::FrameBegin, &mut out).unwrap();
        assert_eq!(String::from_utf8(out).unwrap(), "--\n1,0\n");
    }

    #[test]
    fn analog_uses_units_and_g_format() {
        let chans = [Channel::new(0, ChannelKind::Analog, "A0")];
        let mut opts = Options::new();
        opts.insert("header".into(), "false".into());
        let mut c = Csv::new(&ctx(&chans), &opts).unwrap();
        let mut out = Vec::new();
        c.receive(
            &Packet::Analog(crate::core::AnalogData {
                channels: vec![0],
                samples: vec![-10.0, 3.090_17],
                mq: crate::core::Mq::Voltage,
                flags: crate::core::MqFlags::DC,
                unit: crate::core::Unit::Volt,
                digits: 4,
            }),
            &mut out,
        )
        .unwrap();
        c.receive(&Packet::End, &mut out).unwrap();
        assert_eq!(String::from_utf8(out).unwrap(), "V DC\n-10\n3.09017\n");
    }
}
