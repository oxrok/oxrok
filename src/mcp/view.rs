// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! A window of a capture drawn for an agent: as text lanes with a merged
//! list of edges, for a model that reads characters, and as a PNG of
//! logic lanes, analog traces, a time axis, the trigger and decoded
//! frames, for one that sees pictures. Both are windows: a span of the
//! capture bounded in columns or pixels, never the whole data.
//!
//! The PNG is written here: a bitmap, a 5x7 font and the three chunks
//! the format needs, with zlib from `flate2`. No drawing library, so the
//! server stays a small binary.

use super::capture::Capture;
use crate::core::{Channel, ChannelKind};
use crate::decode::frames::Frame;
use std::fmt::Write as _;
use std::io::Write as _;

/// A logic lane as text: `‾` high, `_` low, `|` where it changes inside
/// a column; an analog lane as a sparkline of its mean per column.
pub fn text_lane(c: &Capture, ch: &Channel, from: u64, to: u64, columns: usize) -> String {
    let columns = columns.max(1);
    let width = (to.saturating_sub(from)) as f64 / columns as f64;
    let bounds = |col: usize| -> (u64, u64) {
        let s0 = from + (col as f64 * width) as u64;
        let s1 = if col + 1 == columns { to } else { from + ((col + 1) as f64 * width) as u64 };
        (s0, s1.max(s0 + 1).min(to))
    };
    let mut out = String::with_capacity(columns * 3);
    if ch.kind == ChannelKind::Logic {
        for col in 0..columns {
            let (s0, s1) = bounds(col);
            if s0 >= to {
                out.push(' ');
                continue;
            }
            let first = c.level(ch.index, s0);
            let changes = c.levels(ch.index, s0, s1).any(|l| l != first);
            out.push(if changes {
                '|'
            } else if first {
                '‾'
            } else {
                '_'
            });
        }
    } else {
        const BARS: [char; 8] = ['▁', '▂', '▃', '▄', '▅', '▆', '▇', '█'];
        let all = c.analog_values(ch.index, from, to);
        let (lo, hi) =
            all.iter().fold((f32::MAX, f32::MIN), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
        let span = if hi > lo { hi - lo } else { 1.0 };
        for col in 0..columns {
            let (s0, s1) = bounds(col);
            let i0 = (s0 - from) as usize;
            let i1 = ((s1 - from) as usize).min(all.len());
            if i0 >= i1 {
                out.push(' ');
                continue;
            }
            let mean = all[i0..i1].iter().sum::<f32>() / (i1 - i0) as f32;
            let k = (((mean - lo) / span) * 7.0).round().clamp(0.0, 7.0) as usize;
            out.push(BARS[k]);
        }
    }
    out
}

/// The edges of the logic channels in the window, merged in sample
/// order: `(sample, channel name, level after)`, at most `max` of them
/// and whether there were more.
pub fn events<'a>(
    c: &Capture,
    channels: &[&'a Channel],
    from: u64,
    to: u64,
    max: usize,
) -> (Vec<(u64, &'a str, bool)>, bool) {
    let mut all: Vec<(u64, &str, bool)> = Vec::new();
    for ch in channels.iter().filter(|ch| ch.kind == ChannelKind::Logic) {
        for (s, l) in c.transitions(ch.index, from, to) {
            all.push((s, ch.name.as_str(), l));
        }
    }
    all.sort_by_key(|(s, _, _)| *s);
    let more = all.len() > max;
    all.truncate(max);
    (all, more)
}

/// Decoded frames to draw under the signals: one lane per entry.
pub struct FrameLane<'a> {
    /// The lane's label, `i2c-1 addr-data`.
    pub label: String,
    /// The frames, in sample order, with the text to show.
    pub frames: Vec<(&'a Frame, String)>,
}

/// What a picture of a window shows.
pub struct Picture<'a> {
    /// The capture.
    pub capture: &'a Capture,
    /// The channels, in lane order.
    pub channels: Vec<&'a Channel>,
    /// Decoded frames, below the channels.
    pub frames: Vec<FrameLane<'a>>,
    /// The window's first sample.
    pub from: u64,
    /// One past the window's last sample.
    pub to: u64,
    /// Pixels across.
    pub width: usize,
}

const LEFT: usize = 112;
const TOP: usize = 22;
const LOGIC_H: usize = 28;
const ANALOG_H: usize = 96;
const FRAMES_H: usize = 26;
const BOTTOM: usize = 6;

const BG: [u8; 3] = [255, 255, 255];
const GRID: [u8; 3] = [226, 230, 234];
const TEXT: [u8; 3] = [36, 41, 47];
const LOGIC: [u8; 3] = [31, 111, 235];
const ANALOG: [u8; 3] = [45, 164, 78];
const TRIGGER: [u8; 3] = [207, 34, 46];
const FRAME_FILL: [u8; 3] = [255, 241, 184];
const FRAME_EDGE: [u8; 3] = [154, 103, 0];

impl Picture<'_> {
    /// The PNG.
    pub fn png(&self) -> Vec<u8> {
        let c = self.capture;
        let width = self.width.clamp(200, 4096);
        let lanes_h: usize = self
            .channels
            .iter()
            .map(|ch| if ch.kind == ChannelKind::Logic { LOGIC_H } else { ANALOG_H })
            .sum::<usize>()
            + self.frames.len() * FRAMES_H;
        let height = TOP + lanes_h + BOTTOM;
        let mut b = Bitmap::new(width, height, BG);
        let plot_w = width.saturating_sub(LEFT + 8).max(1);
        let span = self.to.saturating_sub(self.from).max(1);
        let x_of = |sample: u64| -> usize {
            LEFT + ((sample.saturating_sub(self.from)) as f64 / span as f64 * plot_w as f64)
                as usize
        };
        // The time axis: ticks at a round step, labels at the top.
        let (step, _) = nice_step(span, 8);
        let first_tick = self.from.div_ceil(step) * step;
        let mut tick = first_tick;
        while tick <= self.to {
            let x = x_of(tick).min(LEFT + plot_w);
            b.vline(x, TOP - 4, height - BOTTOM, GRID);
            let label = c.time(tick).map_or_else(|| tick.to_string(), seconds_text);
            b.text(x.saturating_sub(label.len() * 6 / 2), 4, &label, TEXT, 1);
            tick += step;
        }
        b.hline(LEFT, LEFT + plot_w, TOP - 1, GRID);
        let unit_label = if c.samplerate.is_some() { "time" } else { "samples" };
        b.text(4, 4, unit_label, TEXT, 1);
        // The lanes.
        let mut y = TOP;
        for ch in &self.channels {
            if ch.kind == ChannelKind::Logic {
                self.logic_lane(&mut b, ch, y, plot_w);
                y += LOGIC_H;
            } else {
                self.analog_lane(&mut b, ch, y, plot_w);
                y += ANALOG_H;
            }
            b.hline(0, width, y - 1, GRID);
        }
        for lane in &self.frames {
            self.frame_lane(&mut b, lane, y, plot_w, &x_of);
            y += FRAMES_H;
            b.hline(0, width, y - 1, GRID);
        }
        // The trigger.
        if let Some(t) = c.trigger_at
            && t >= self.from
            && t < self.to
        {
            let x = x_of(t);
            b.vline(x, TOP, height - BOTTOM, TRIGGER);
            b.text(x + 3, TOP + 2, "T", TRIGGER, 1);
        }
        png(width, height, &b.pixels)
    }

    fn logic_lane(&self, b: &mut Bitmap, ch: &Channel, y: usize, plot_w: usize) {
        let c = self.capture;
        b.text(4, y + 9, &fit(&ch.name, 17), TEXT, 1);
        let hi_y = y + 6;
        let lo_y = y + LOGIC_H - 8;
        let span = self.to.saturating_sub(self.from).max(1);
        let mut prev: Option<bool> = None;
        for px in 0..plot_w {
            let s0 = self.from + (px as u64 * span) / plot_w as u64;
            let s1 =
                (self.from + ((px as u64 + 1) * span) / plot_w as u64).max(s0 + 1).min(self.to);
            if s0 >= self.to {
                break;
            }
            let first = c.level(ch.index, s0);
            let mixed = c.levels(ch.index, s0, s1).any(|l| l != first);
            let x = LEFT + px;
            if mixed {
                b.vline(x, hi_y, lo_y + 1, LOGIC);
                prev = Some(c.level(ch.index, s1 - 1));
                continue;
            }
            let yy = if first { hi_y } else { lo_y };
            if prev.is_some_and(|p| p != first) {
                b.vline(x, hi_y, lo_y + 1, LOGIC);
            }
            b.set(x, yy, LOGIC);
            b.set(x, yy + 1, LOGIC);
            prev = Some(first);
        }
    }

    #[allow(clippy::many_single_char_names)]
    fn analog_lane(&self, b: &mut Bitmap, ch: &Channel, y: usize, plot_w: usize) {
        let c = self.capture;
        let values = c.analog_values(ch.index, self.from, self.to);
        let (_, unit) = c.analog_kind(ch.index).unwrap_or(("", ""));
        b.text(4, y + 4, &fit(&ch.name, 17), TEXT, 1);
        if values.is_empty() {
            b.text(LEFT + 4, y + ANALOG_H / 2, "no samples", GRID, 1);
            return;
        }
        let (lo, hi) =
            values.iter().fold((f32::MAX, f32::MIN), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
        let (lo, hi) = if hi > lo { (lo, hi) } else { (lo - 1.0, hi + 1.0) };
        let top = y + 14;
        let bottom = y + ANALOG_H - 10;
        let y_of = |v: f32| -> usize {
            let f = ((hi - v) / (hi - lo)).clamp(0.0, 1.0);
            top + (f * (bottom - top) as f32) as usize
        };
        b.text(4, top + 4, &fit(&format!("{} {unit}", short(hi)), 17), ANALOG, 1);
        b.text(4, bottom - 6, &fit(&format!("{} {unit}", short(lo)), 17), ANALOG, 1);
        if lo < 0.0 && hi > 0.0 {
            b.hline(LEFT, LEFT + plot_w, y_of(0.0), GRID);
        }
        let n = values.len();
        let mut prev_mean: Option<usize> = None;
        for px in 0..plot_w {
            let i0 = px * n / plot_w;
            let i1 = ((px + 1) * n / plot_w).max(i0 + 1).min(n);
            if i0 >= n {
                break;
            }
            let slice = &values[i0..i1];
            let (mn, mx) =
                slice.iter().fold((f32::MAX, f32::MIN), |(a, z), v| (a.min(*v), z.max(*v)));
            let mean = slice.iter().sum::<f32>() / slice.len() as f32;
            let x = LEFT + px;
            b.vline(x, y_of(mx), y_of(mn) + 1, ANALOG);
            let ym = y_of(mean);
            if let Some(p) = prev_mean {
                b.vline(x, p.min(ym), p.max(ym) + 1, ANALOG);
            }
            prev_mean = Some(ym);
        }
    }

    fn frame_lane(
        &self,
        b: &mut Bitmap,
        lane: &FrameLane<'_>,
        y: usize,
        plot_w: usize,
        x_of: &dyn Fn(u64) -> usize,
    ) {
        b.text(4, y + 9, &fit(&lane.label, 17), TEXT, 1);
        let right = LEFT + plot_w;
        for (f, text) in &lane.frames {
            if f.end < self.from || f.start >= self.to {
                continue;
            }
            let x0 = x_of(f.start.max(self.from)).min(right);
            let x1 = x_of(f.end.min(self.to)).max(x0 + 1).min(right);
            b.rect(x0, y + 4, x1, y + FRAMES_H - 5, FRAME_FILL, FRAME_EDGE);
            let room = (x1 - x0).saturating_sub(4) / 6;
            if room >= 1 {
                b.text(x0 + 2, y + 9, &fit(text, room), TEXT, 1);
            }
        }
    }
}

/// `text` cut to `chars` characters.
fn fit(text: &str, chars: usize) -> String {
    text.chars().take(chars).collect()
}

/// A value with a few significant digits.
fn short(v: f32) -> String {
    let a = v.abs();
    if a >= 100.0 {
        format!("{v:.0}")
    } else if a >= 10.0 {
        format!("{v:.1}")
    } else if a >= 1.0 {
        format!("{v:.2}")
    } else {
        format!("{v:.3}")
    }
}

/// Seconds as `12.5 ms`, `3 µs`, `1.2 s`.
pub fn seconds_text(t: f64) -> String {
    let (v, unit) = if t == 0.0 {
        (0.0, "s")
    } else if t.abs() >= 1.0 {
        (t, "s")
    } else if t.abs() >= 1e-3 {
        (t * 1e3, "ms")
    } else if t.abs() >= 1e-6 {
        (t * 1e6, "us")
    } else {
        (t * 1e9, "ns")
    };
    let mut s = format!("{v:.3}");
    while s.contains('.') && (s.ends_with('0') || s.ends_with('.')) {
        s.pop();
    }
    format!("{s} {unit}")
}

/// A round step in samples giving about `ticks` ticks over `span`.
fn nice_step(span: u64, ticks: u64) -> (u64, u64) {
    let raw = (span / ticks.max(1)).max(1);
    let mut mag = 1u64;
    while mag.saturating_mul(10) <= raw {
        mag = mag.saturating_mul(10);
    }
    let step =
        [1u64, 2, 5, 10].iter().map(|m| m.saturating_mul(mag)).find(|s| *s >= raw).unwrap_or(raw);
    (step, span / step)
}

/// RGB, one byte per channel.
struct Bitmap {
    width: usize,
    height: usize,
    pixels: Vec<u8>,
}

impl Bitmap {
    fn new(width: usize, height: usize, bg: [u8; 3]) -> Self {
        let mut pixels = Vec::with_capacity(width * height * 3);
        for _ in 0..width * height {
            pixels.extend_from_slice(&bg);
        }
        Self { width, height, pixels }
    }

    fn set(&mut self, x: usize, y: usize, rgb: [u8; 3]) {
        if x < self.width && y < self.height {
            let i = (y * self.width + x) * 3;
            self.pixels[i..i + 3].copy_from_slice(&rgb);
        }
    }

    fn hline(&mut self, x0: usize, x1: usize, y: usize, rgb: [u8; 3]) {
        for x in x0..x1.min(self.width) {
            self.set(x, y, rgb);
        }
    }

    fn vline(&mut self, x: usize, y0: usize, y1: usize, rgb: [u8; 3]) {
        for y in y0..y1.min(self.height) {
            self.set(x, y, rgb);
        }
    }

    fn rect(&mut self, x0: usize, y0: usize, x1: usize, y1: usize, fill: [u8; 3], edge: [u8; 3]) {
        for y in y0..y1.min(self.height) {
            for x in x0..x1.min(self.width) {
                let on_edge = y == y0 || y + 1 == y1 || x == x0 || x + 1 == x1;
                self.set(x, y, if on_edge { edge } else { fill });
            }
        }
    }

    /// Draws `text` with the 5x7 font at `scale`, 6 pixels per character.
    fn text(&mut self, x: usize, y: usize, text: &str, rgb: [u8; 3], scale: usize) {
        let mut cx = x;
        for ch in text.chars() {
            let glyph = glyph(ch);
            for (row, bits) in glyph.iter().enumerate() {
                for col in 0..5 {
                    if bits & (0b1_0000 >> col) != 0 {
                        for dy in 0..scale {
                            for dx in 0..scale {
                                self.set(cx + col * scale + dx, y + row * scale + dy, rgb);
                            }
                        }
                    }
                }
            }
            cx += 6 * scale;
        }
    }
}

/// A 5x7 glyph, one byte per row, the low five bits the columns.
fn glyph(ch: char) -> [u8; 7] {
    let rows: [&str; 7] = match ch {
        '0' => ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
        '1' => ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
        '2' => ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
        '3' => ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
        '4' => ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
        '5' => ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
        '6' => ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
        '7' => ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
        '8' => ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
        '9' => ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
        'A' => ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
        'B' => ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
        'C' => ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
        'D' => ["11100", "10010", "10001", "10001", "10001", "10010", "11100"],
        'E' => ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
        'F' => ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
        'G' => ["01110", "10001", "10000", "10111", "10001", "10001", "01111"],
        'H' => ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
        'I' => ["01110", "00100", "00100", "00100", "00100", "00100", "01110"],
        'J' => ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
        'K' => ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
        'L' => ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
        'M' => ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
        'N' => ["10001", "10001", "11001", "10101", "10011", "10001", "10001"],
        'O' => ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
        'P' => ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
        'Q' => ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
        'R' => ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
        'S' => ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
        'T' => ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
        'U' => ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
        'V' => ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
        'W' => ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
        'X' => ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
        'Y' => ["10001", "10001", "10001", "01010", "00100", "00100", "00100"],
        'Z' => ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
        'a' => ["00000", "00000", "01110", "00001", "01111", "10001", "01111"],
        'b' => ["10000", "10000", "10110", "11001", "10001", "10001", "11110"],
        'c' => ["00000", "00000", "01110", "10000", "10000", "10001", "01110"],
        'd' => ["00001", "00001", "01101", "10011", "10001", "10001", "01111"],
        'e' => ["00000", "00000", "01110", "10001", "11111", "10000", "01110"],
        'f' => ["00110", "01001", "01000", "11100", "01000", "01000", "01000"],
        'g' => ["00000", "01111", "10001", "10001", "01111", "00001", "01110"],
        'h' => ["10000", "10000", "10110", "11001", "10001", "10001", "10001"],
        'i' => ["00100", "00000", "01100", "00100", "00100", "00100", "01110"],
        'j' => ["00010", "00000", "00110", "00010", "00010", "10010", "01100"],
        'k' => ["10000", "10000", "10010", "10100", "11000", "10100", "10010"],
        'l' => ["01100", "00100", "00100", "00100", "00100", "00100", "01110"],
        'm' => ["00000", "00000", "11010", "10101", "10101", "10001", "10001"],
        'n' => ["00000", "00000", "10110", "11001", "10001", "10001", "10001"],
        'o' => ["00000", "00000", "01110", "10001", "10001", "10001", "01110"],
        'p' => ["00000", "00000", "11110", "10001", "11110", "10000", "10000"],
        'q' => ["00000", "00000", "01101", "10011", "01111", "00001", "00001"],
        'r' => ["00000", "00000", "10110", "11001", "10000", "10000", "10000"],
        's' => ["00000", "00000", "01110", "10000", "01110", "00001", "11110"],
        't' => ["01000", "01000", "11100", "01000", "01000", "01001", "00110"],
        'u' => ["00000", "00000", "10001", "10001", "10001", "10011", "01101"],
        'v' => ["00000", "00000", "10001", "10001", "10001", "01010", "00100"],
        'w' => ["00000", "00000", "10001", "10001", "10101", "10101", "01010"],
        'x' => ["00000", "00000", "10001", "01010", "00100", "01010", "10001"],
        'y' => ["00000", "00000", "10001", "10001", "01111", "00001", "01110"],
        'z' => ["00000", "00000", "11111", "00010", "00100", "01000", "11111"],
        '.' => ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
        ',' => ["00000", "00000", "00000", "00000", "01100", "00100", "01000"],
        ':' => ["00000", "01100", "01100", "00000", "01100", "01100", "00000"],
        ';' => ["00000", "01100", "01100", "00000", "01100", "00100", "01000"],
        '-' => ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
        '_' => ["00000", "00000", "00000", "00000", "00000", "00000", "11111"],
        '=' => ["00000", "00000", "11111", "00000", "11111", "00000", "00000"],
        '+' => ["00000", "00100", "00100", "11111", "00100", "00100", "00000"],
        '/' => ["00000", "00001", "00010", "00100", "01000", "10000", "00000"],
        '\\' => ["00000", "10000", "01000", "00100", "00010", "00001", "00000"],
        '(' => ["00010", "00100", "01000", "01000", "01000", "00100", "00010"],
        ')' => ["01000", "00100", "00010", "00010", "00010", "00100", "01000"],
        '[' => ["01110", "01000", "01000", "01000", "01000", "01000", "01110"],
        ']' => ["01110", "00010", "00010", "00010", "00010", "00010", "01110"],
        '<' => ["00010", "00100", "01000", "10000", "01000", "00100", "00010"],
        '>' => ["01000", "00100", "00010", "00001", "00010", "00100", "01000"],
        '#' => ["01010", "01010", "11111", "01010", "11111", "01010", "01010"],
        '%' => ["11000", "11001", "00010", "00100", "01000", "10011", "00011"],
        '\'' => ["01100", "00100", "01000", "00000", "00000", "00000", "00000"],
        '"' => ["01010", "01010", "01010", "00000", "00000", "00000", "00000"],
        '!' => ["00100", "00100", "00100", "00100", "00100", "00000", "00100"],
        '?' => ["01110", "10001", "00001", "00010", "00100", "00000", "00100"],
        '*' => ["00000", "00100", "10101", "01110", "10101", "00100", "00000"],
        'µ' => ["00000", "00000", "10001", "10001", "10011", "11101", "10000"],
        'Ω' => ["01110", "10001", "10001", "10001", "01010", "01010", "11011"],
        ' ' => ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
        _ => ["11111", "10001", "10001", "10001", "10001", "10001", "11111"],
    };
    let mut out = [0u8; 7];
    for (i, row) in rows.iter().enumerate() {
        out[i] = u8::from_str_radix(row, 2).unwrap_or(0);
    }
    out
}

/// A PNG of 8-bit RGB pixels, row-major.
pub fn png(width: usize, height: usize, rgb: &[u8]) -> Vec<u8> {
    let mut raw = Vec::with_capacity(height * (width * 3 + 1));
    for row in rgb.chunks(width * 3).take(height) {
        raw.push(0); // filter: none
        raw.extend_from_slice(row);
    }
    let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::fast());
    let _ = z.write_all(&raw);
    let idat = z.finish().unwrap_or_default();
    let mut out = Vec::with_capacity(idat.len() + 64);
    out.extend_from_slice(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]);
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(height as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]); // 8 bits, RGB, deflate, no filter set, no interlace
    chunk(&mut out, *b"IHDR", &ihdr);
    chunk(&mut out, *b"IDAT", &idat);
    chunk(&mut out, *b"IEND", &[]);
    out
}

fn chunk(out: &mut Vec<u8>, kind: [u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(&kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in bytes {
        crc ^= u32::from(b);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Base64 of `bytes`, the standard alphabet with padding.
pub fn base64(bytes: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let n = chunk.len();
        let v = (u32::from(chunk[0]) << 16)
            | (u32::from(*chunk.get(1).unwrap_or(&0)) << 8)
            | u32::from(*chunk.get(2).unwrap_or(&0));
        out.push(T[(v >> 18) as usize & 63] as char);
        out.push(T[(v >> 12) as usize & 63] as char);
        out.push(if n > 1 { T[(v >> 6) as usize & 63] as char } else { '=' });
        out.push(if n > 2 { T[v as usize & 63] as char } else { '=' });
    }
    out
}

/// The text form of a window, for the content block: the lanes with a
/// scale line and the names aligned.
pub fn text_block(lanes: &[(String, String)], from: u64, to: u64, columns: usize) -> String {
    let name_w = lanes.iter().map(|(n, _)| n.chars().count()).max().unwrap_or(0).max(6);
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:>name_w$} {from}{}{to}",
        "sample",
        " ".repeat(columns.saturating_sub(from.to_string().len() + to.to_string().len()))
    );
    for (name, lane) in lanes {
        let _ = writeln!(out, "{name:>name_w$} {lane}");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Packet;
    use std::io::Read as _;

    fn bits(pattern: &str) -> Capture {
        let mut c =
            Capture::new("test", vec![Channel::new(0, ChannelKind::Logic, "D0")], Some(1000));
        let data: Vec<u8> = pattern.bytes().map(|b| u8::from(b == b'1')).collect();
        c.push(&Packet::logic(1, data));
        c
    }

    #[test]
    fn text_lanes_show_levels_and_edges() {
        let c = bits("0011110000111100");
        let ch = &c.channels[0];
        assert_eq!(text_lane(&c, ch, 0, 16, 16), "__‾‾‾‾____‾‾‾‾__");
        // Two samples per column: a column with an edge inside is `|`.
        assert_eq!(text_lane(&c, ch, 0, 16, 8), "_‾‾__‾‾_");
        // Three samples per column from sample 1: edges fall inside most.
        assert_eq!(text_lane(&c, ch, 1, 16, 5), "||_‾|");
        let (ev, more) = events(&c, &[ch], 0, 16, 3);
        assert_eq!(ev, [(2, "D0", true), (6, "D0", false), (10, "D0", true)]);
        assert!(more);
        assert_eq!(seconds_text(0.0125), "12.5 ms");
        assert_eq!(seconds_text(3e-6), "3 us");
        assert_eq!(seconds_text(1.0), "1 s");
        assert_eq!(nice_step(1000, 8), (200, 5));
        assert_eq!(nice_step(16, 8), (2, 8));
    }

    #[test]
    fn the_png_is_well_formed() {
        let c = bits("0011110000111100");
        let frame = Frame {
            start: 2,
            end: 6,
            class: 0,
            row: 0,
            texts: vec!["hi".to_owned()].into(),
            value: None,
            error: false,
        };
        let pic = Picture {
            capture: &c,
            channels: vec![&c.channels[0]],
            frames: vec![FrameLane { label: "x".into(), frames: vec![(&frame, "hi".to_owned())] }],
            from: 0,
            to: 16,
            width: 300,
        };
        let png = pic.png();
        assert_eq!(&png[..8], &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]);
        // IHDR: width 300, height = top + logic lane + frame lane + bottom.
        assert_eq!(&png[12..16], b"IHDR");
        let width = u32::from_be_bytes(png[16..20].try_into().unwrap());
        let height = u32::from_be_bytes(png[20..24].try_into().unwrap());
        assert_eq!(width, 300);
        assert_eq!(height as usize, TOP + LOGIC_H + FRAMES_H + BOTTOM);
        // The IDAT inflates to one filter byte plus the row per line.
        let idat_len = u32::from_be_bytes(png[33..37].try_into().unwrap()) as usize;
        assert_eq!(&png[37..41], b"IDAT");
        let mut raw = Vec::new();
        flate2::read::ZlibDecoder::new(&png[41..41 + idat_len]).read_to_end(&mut raw).unwrap();
        assert_eq!(raw.len(), height as usize * (width as usize * 3 + 1));
        assert_eq!(crc32(b"IEND"), 0xAE42_6082);
        assert_eq!(base64(b"hi!"), "aGkh");
        assert_eq!(base64(b"hi"), "aGk=");
    }
}
