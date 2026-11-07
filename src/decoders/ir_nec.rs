// Counterpart in sigrok:
//   libsigrokdecode/decoders/ir_nec/pd.py,
//   Copyright (C) 2014 Gump Yang <gump.yang@gmail.com>,
//   and decoders/ir_nec/lists.py, Copyright (C) 2014 Uwe Hermann,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the NEC infrared protocol and the decoder's
//   documented classes and texts.

//! IR NEC: leader and repeat codes, 8-bit address/command plus inverses.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const IR: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BIT: usize = 0;
const ANN_AGC: usize = 1;
const ANN_LONG_PAUSE: usize = 2;
const ANN_SHORT_PAUSE: usize = 3;
const ANN_STOP: usize = 4;
const ANN_LEADER: usize = 5;
const ANN_ADDR: usize = 6;
const ANN_ADDR_INV: usize = 7;
const ANN_CMD: usize = 8;
const ANN_CMD_INV: usize = 9;
const ANN_REPEAT: usize = 10;
const ANN_REMOTE: usize = 11;
const ANN_WARN: usize = 12;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc {
        id: "polarity",
        desc: "Polarity",
        default: OptionValue::s("active-low"),
        values: &[OptionValue::s("active-low"), OptionValue::s("active-high")],
    },
    OptionDesc {
        id: "cd_freq",
        desc: "Carrier Frequency",
        default: OptionValue::i(0),
        values: &[],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ir_nec",
    name: "IR NEC",
    longname: "IR NEC",
    desc: "NEC infrared remote control protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["IR"],
    channels: &[ChannelDesc { id: "ir", name: "IR", desc: "Data line" }],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "agc-pulse", desc: "AGC pulse" },
        AnnClass { id: "longpause", desc: "Long pause" },
        AnnClass { id: "shortpause", desc: "Short pause" },
        AnnClass { id: "stop-bit", desc: "Stop bit" },
        AnnClass { id: "leader-code", desc: "Leader code" },
        AnnClass { id: "addr", desc: "Address" },
        AnnClass { id: "addr-inv", desc: "Address#" },
        AnnClass { id: "cmd", desc: "Command" },
        AnnClass { id: "cmd-inv", desc: "Command#" },
        AnnClass { id: "repeat-code", desc: "Repeat code" },
        AnnClass { id: "remote", desc: "Remote" },
        AnnClass { id: "warnings", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3, 4] },
        AnnRow { id: "fields", desc: "Fields", classes: &[5, 6, 7, 8, 9, 10] },
        AnnRow { id: "remote", desc: "Remote", classes: &[11] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[12] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(IrNec::default())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Address,
    AddressInv,
    Command,
    CommandInv,
    Stop,
}

// Remote names from ir_nec/lists.py. Items that are not listed are
// reserved/unknown.
const DIGITS: [(&str, &str); 10] = [
    ("0", "0"),
    ("1", "1"),
    ("2", "2"),
    ("3", "3"),
    ("4", "4"),
    ("5", "5"),
    ("6", "6"),
    ("7", "7"),
    ("8", "8"),
    ("9", "9"),
];

const COMMANDS: [(i64, &str, &str); 8] = [
    (11, "-/--", "-/--"),
    (16, "Mute", "M"),
    (18, "Standby", "StBy"),
    (26, "Volume up", "Vol+"),
    (27, "Program up", "P+"),
    (30, "Volume down", "Vol-"),
    (31, "Program down", "P-"),
    (68, "AV", "AV"),
];

fn ann_texts(texts: &[&str]) -> Vec<String> {
    texts.iter().map(|t| (*t).to_owned()).collect()
}

/// The IR NEC decoder.
pub struct IrNec {
    samplerate: u64,
    state: State,
    ss_bit: i64,
    ss_start: i64,
    ss_other_edge: i64,
    ss_remote: i64,
    data: i64,
    count: i64,
    active: i64,
    addr: Option<i64>,
    cmd: Option<i64>,
    tolerance: f64,
    lc: i64,
    rc: i64,
    dazero: i64,
    daone: i64,
    stop: i64,
    active_low: bool,
    cd_freq: i64,
    /// Mirrors the `samplenum` attribute pd.py keeps on the instance; it
    /// can move ahead of the stream in the repeat-code path.
    samplenum: i64,
}

impl Default for IrNec {
    fn default() -> Self {
        IrNec {
            samplerate: 0,
            state: State::Idle,
            ss_bit: 0,
            ss_start: 0,
            ss_other_edge: 0,
            ss_remote: 0,
            data: 0,
            count: 0,
            active: 0,
            addr: None,
            cmd: None,
            tolerance: 0.0,
            lc: 0,
            rc: 0,
            dazero: 0,
            daone: 0,
            stop: 0,
            active_low: true,
            cd_freq: 0,
            samplenum: 0,
        }
    }
}

impl IrNec {
    fn title(&self) -> &'static str {
        match self.state {
            State::Idle => "Idle",
            State::Address => "Address",
            State::AddressInv => "Address#",
            State::Command => "Command",
            State::CommandInv => "Command#",
            State::Stop => "Stop",
        }
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        ctx.put_ann(self.ss_start as u64, self.samplenum as u64, class, ann_texts(texts));
    }

    fn putd(&self, ctx: &mut dyn Context, data: i64) {
        let name = self.title();
        let (class, short, letter) = match self.state {
            State::AddressInv => (ANN_ADDR_INV, "ADDR#", "A#"),
            State::Command => (ANN_CMD, "CMD", "C"),
            State::CommandInv => (ANN_CMD_INV, "CMD#", "C#"),
            State::Address | State::Idle | State::Stop => (ANN_ADDR, "ADDR", "A"),
        };
        let texts = vec![
            format!("{name}: 0x{data:02X}"),
            format!("{short}: 0x{data:02X}"),
            format!("{letter}: 0x{data:02X}"),
            letter.to_owned(),
        ];
        ctx.put_ann(self.ss_start as u64, self.samplenum as u64, class, texts);
    }

    fn putstop(&self, ctx: &mut dyn Context, ss: i64) {
        ctx.put_ann(
            ss as u64,
            (ss + self.stop) as u64,
            ANN_STOP,
            ann_texts(&["Stop bit", "Stop", "St", "S"]),
        );
    }

    fn putpause(&self, ctx: &mut dyn Context, long: bool) {
        ctx.put_ann(
            self.ss_start as u64,
            self.ss_other_edge as u64,
            ANN_AGC,
            ann_texts(&["AGC pulse", "AGC", "A"]),
        );
        let (idx, texts): (usize, [&str; 4]) = if long {
            (ANN_LONG_PAUSE, ["Long pause", "L-pause", "LP", "P"])
        } else {
            (ANN_SHORT_PAUSE, ["Short pause", "S-pause", "SP", "P"])
        };
        ctx.put_ann(self.ss_other_edge as u64, self.samplenum as u64, idx, ann_texts(&texts));
    }

    fn putremote(&self, ctx: &mut dyn Context) {
        let dev = if self.addr == Some(0x40) { "Matsui TV" } else { "Unknown device" };
        let (btn0, btn1) = self.button();
        // pd.py spans ss_remote..ss_bit + stop, not up to samplenum.
        ctx.put_ann(
            self.ss_remote as u64,
            (self.ss_bit + self.stop) as u64,
            ANN_REMOTE,
            vec![format!("{dev}: {btn0}"), format!("{dev}: {btn1}"), btn1.to_owned()],
        );
    }

    fn button(&self) -> (&'static str, &'static str) {
        if self.addr != Some(0x40) {
            return ("Unknown", "Unk");
        }
        let Some(cmd) = self.cmd else { return ("Unknown", "Unk") };
        if (0..=9).contains(&cmd) {
            return DIGITS[cmd as usize];
        }
        for &(c, a, b) in &COMMANDS {
            if c == cmd {
                return (a, b);
            }
        }
        ("Unknown", "Unk")
    }

    fn compare_with_tolerance(&self, measured: i64, base: i64) -> bool {
        let m = measured as f64;
        let b = base as f64;
        m >= b * (1.0 - self.tolerance) && m <= b * (1.0 + self.tolerance)
    }

    fn handle_bit(&mut self, ctx: &mut dyn Context, tick: i64) {
        let ret = if self.compare_with_tolerance(tick, self.dazero) {
            Some(0i64)
        } else if self.compare_with_tolerance(tick, self.daone) {
            Some(1i64)
        } else {
            None
        };
        if let Some(ret) = ret {
            ctx.put_ann(self.ss_bit as u64, self.samplenum as u64, ANN_BIT, vec![ret.to_string()]);
            self.data |= ret << self.count; // LSB-first
            self.count += 1;
        }
        self.ss_bit = self.samplenum;
    }

    fn data_ok(&mut self, ctx: &mut dyn Context) -> bool {
        let ret = (self.data >> 8) & (self.data & 0xff);
        let name = self.title();
        if self.count == 8 {
            if self.state == State::Address {
                self.addr = Some(self.data);
            }
            if self.state == State::Command {
                self.cmd = Some(self.data);
            }
            self.putd(ctx, self.data);
            self.ss_start = self.samplenum;
            return true;
        }
        if ret == 0 {
            self.putd(ctx, self.data >> 8);
        } else {
            ctx.put_ann(
                self.ss_start as u64,
                self.samplenum as u64,
                ANN_WARN,
                vec![format!("{name} error: 0x{:04X}", self.data)],
            );
        }
        self.data = 0;
        self.count = 0;
        self.ss_bit = self.samplenum;
        self.ss_start = self.samplenum;
        ret == 0
    }
}

impl Decoder for IrNec {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.active_low = config.str("polarity", "active-low") == "active-low";
        self.cd_freq = config.int("cd_freq", 0);
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        self.tolerance = 0.05; // +/-5%
        self.lc = (samplerate as f64 * 0.0135) as i64 - 1; // 13.5ms
        self.rc = (samplerate as f64 * 0.01125) as i64 - 1; // 11.25ms
        self.dazero = (samplerate as f64 * 0.001_125) as i64 - 1; // 1.125ms
        self.daone = (samplerate as f64 * 0.00225) as i64 - 1; // 2.25ms
        self.stop = (samplerate as f64 * 0.000_652) as i64 - 1; // 0.652ms
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        let cd_freq = self.cd_freq;
        let cd_count: Option<u64> = if cd_freq != 0 {
            let count = (self.samplerate as f64 / cd_freq as f64) as i64 + 1;
            if count == 0 { None } else { Some(count.max(0) as u64) }
        } else {
            None
        };
        let mut prev_ir: Option<i64> = None;

        self.active = i64::from(!self.active_low);

        loop {
            // Detect changes in the presence of an active input signal.
            let cur_ir = if let Some(cd_count) = cd_count {
                let pins =
                    ctx.wait(&[&[Cond::Pin(IR, CondKind::Edge)], &[Cond::Skip(cd_count)]])?;
                let mut cur_ir = i64::from(pins.0.get(IR).copied().unwrap_or(0xff));
                if ctx.matched()[0] {
                    cur_ir = self.active;
                }
                self.samplenum = ctx.samplenum() as i64;
                if Some(cur_ir) == prev_ir {
                    continue;
                }
                prev_ir = Some(cur_ir);
                cur_ir
            } else {
                let pins = ctx.wait(&[&[Cond::Pin(IR, CondKind::Edge)]])?;
                self.samplenum = ctx.samplenum() as i64;
                i64::from(pins.0.get(IR).copied().unwrap_or(0xff))
            };

            if cur_ir != self.active {
                // Save the non-active edge, then wait for the next edge.
                self.ss_other_edge = self.samplenum;
                continue;
            }

            let b = self.samplenum - self.ss_bit;

            // State machine.
            match self.state {
                State::Idle => {
                    if self.compare_with_tolerance(b, self.lc) {
                        self.putpause(ctx, true);
                        self.putx(ctx, ANN_LEADER, &["Leader code", "Leader", "LC", "L"]);
                        self.ss_remote = self.ss_start;
                        self.data = 0;
                        self.count = 0;
                        self.state = State::Address;
                    } else if self.compare_with_tolerance(b, self.rc) {
                        self.putpause(ctx, false);
                        self.putstop(ctx, self.samplenum);
                        self.samplenum += self.stop;
                        self.putx(ctx, ANN_REPEAT, &["Repeat code", "Repeat", "RC", "R"]);
                        self.data = 0;
                        self.count = 0;
                    }
                    self.ss_bit = self.samplenum;
                    self.ss_start = self.samplenum;
                }
                State::Address => {
                    self.handle_bit(ctx, b);
                    if self.count == 8 {
                        self.state =
                            if self.data_ok(ctx) { State::AddressInv } else { State::Idle };
                    }
                }
                State::AddressInv => {
                    self.handle_bit(ctx, b);
                    if self.count == 16 {
                        self.state = if self.data_ok(ctx) { State::Command } else { State::Idle };
                    }
                }
                State::Command => {
                    self.handle_bit(ctx, b);
                    if self.count == 8 {
                        self.state =
                            if self.data_ok(ctx) { State::CommandInv } else { State::Idle };
                    }
                }
                State::CommandInv => {
                    self.handle_bit(ctx, b);
                    if self.count == 16 {
                        self.state = if self.data_ok(ctx) { State::Stop } else { State::Idle };
                    }
                }
                State::Stop => {
                    self.putstop(ctx, self.ss_bit);
                    self.putremote(ctx);
                    self.ss_bit = self.samplenum;
                    self.ss_start = self.samplenum;
                    self.state = State::Idle;
                }
            }
        }
    }
}
