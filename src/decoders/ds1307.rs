// Counterpart in sigrok:
//   libsigrokdecode/decoders/ds1307/pd.py,
//   Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2013 Matt Ranostay <mranostay@gmail.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented
//   registers, bits and texts.

//! Dallas DS1307 realtime clock: seconds to years, control and RAM.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// The DS1307's fixed 7 bit I2C slave address (unshifted).
const DS1307_I2C_ADDRESS: i64 = 0x68;

static DAYS_OF_WEEK: [&str; 7] =
    ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];

/// `rates` in pd.py: SQWE output frequency, keyed by the RS bits.
fn rate(bits: i64) -> &'static str {
    match bits & 0x03 {
        0b00 => "1Hz",
        0b01 => "4096Hz",
        0b10 => "8192Hz",
        _ => "32768Hz",
    }
}

/// `bcd2int` from common/srdhelper.
fn bcd2int(b: i64) -> i64 {
    (b & 0x0f) + ((b >> 4) * 10)
}

/// Python's ``days_of_week[self.days - 1]``, including its negative
/// index wraparound when no day was seen yet (`days` stays -1).
fn day_name(days: i64) -> &'static str {
    let i = (days - 1).rem_euclid(7);
    DAYS_OF_WEEK[i as usize]
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "ds1307",
    name: "DS1307",
    longname: "Dallas DS1307",
    desc: "Dallas DS1307 realtime clock module protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Clock/timing", "IC"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        // `regs_and_bits()`: one register class per `regs` entry.
        AnnClass { id: "reg-seconds", desc: "Seconds register" },
        AnnClass { id: "reg-minutes", desc: "Minutes register" },
        AnnClass { id: "reg-hours", desc: "Hours register" },
        AnnClass { id: "reg-day", desc: "Day register" },
        AnnClass { id: "reg-date", desc: "Date register" },
        AnnClass { id: "reg-month", desc: "Month register" },
        AnnClass { id: "reg-year", desc: "Year register" },
        AnnClass { id: "reg-control", desc: "Control register" },
        AnnClass { id: "reg-ram", desc: "RAM register" },
        // `regs_and_bits()`: one bit class per `bits` entry. The ids
        // replace both '/' and ' ' with '-'.
        AnnClass { id: "bit-clock-halt", desc: "Clock halt bit" },
        AnnClass { id: "bit-seconds", desc: "Seconds bit" },
        AnnClass { id: "bit-reserved", desc: "Reserved bit" },
        AnnClass { id: "bit-minutes", desc: "Minutes bit" },
        AnnClass { id: "bit-12-24-hours", desc: "12/24 hours bit" },
        AnnClass { id: "bit-am-pm", desc: "AM/PM bit" },
        AnnClass { id: "bit-hours", desc: "Hours bit" },
        AnnClass { id: "bit-day", desc: "Day bit" },
        AnnClass { id: "bit-date", desc: "Date bit" },
        AnnClass { id: "bit-month", desc: "Month bit" },
        AnnClass { id: "bit-year", desc: "Year bit" },
        AnnClass { id: "bit-out", desc: "OUT bit" },
        AnnClass { id: "bit-sqwe", desc: "SQWE bit" },
        AnnClass { id: "bit-rs", desc: "RS bit" },
        AnnClass { id: "bit-ram", desc: "RAM bit" },
        AnnClass { id: "read-datetime", desc: "Read date/time" },
        AnnClass { id: "write-datetime", desc: "Write date/time" },
        AnnClass { id: "reg-read", desc: "Register read" },
        AnnClass { id: "reg-write", desc: "Register write" },
        AnnClass { id: "warnings", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow {
            id: "bits",
            desc: "Bits",
            classes: &[9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23],
        },
        AnnRow { id: "regs", desc: "Registers", classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8] },
        AnnRow { id: "date-time", desc: "Date/time", classes: &[24, 25, 26, 27] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[28] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the Dallas DS1307\n\
real-time clock (RTC) specific registers and commands.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Ds1307::default())
}

/// One I2C bit: sample range. Index 0 is the LSB, as in the BITS
/// packet the i2c decoder sends.
#[derive(Clone, Copy)]
struct Bit {
    ss: u64,
    es: u64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    GetSlaveAddr,
    GetRegAddr,
    WriteRtcRegs,
    ReadRtcRegs,
    ReadRtcRegs2,
}

/// The ds1307 decoder.
pub struct Ds1307 {
    state: State,
    hours: i64,
    minutes: i64,
    seconds: i64,
    days: i64,
    date: i64,
    months: i64,
    years: i64,
    bits: Vec<Bit>,
    ss: u64,
    es: u64,
    ss_block: u64,
    reg: i64,
}

impl Default for Ds1307 {
    fn default() -> Self {
        Ds1307 {
            state: State::Idle,
            hours: -1,
            minutes: -1,
            seconds: -1,
            days: -1,
            date: -1,
            months: -1,
            years: -1,
            bits: Vec::new(),
            ss: 0,
            es: 0,
            ss_block: 0,
            reg: 0,
        }
    }
}

/// An I2C packet is `[<ptype>, <pdata>]`.
fn split_packet(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::device("ds1307: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("ds1307: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("ds1307: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// The BITS packet payload is a list of `[bit, ss, es]`, index 0 the LSB.
fn parse_bits(data: &Item) -> Result<Vec<Bit>> {
    let Item::List(items) = data else {
        return Err(Error::device("ds1307: BITS payload is not a list"));
    };
    let mut bits = Vec::with_capacity(items.len());
    for item in items {
        let Item::List(parts) = item else {
            return Err(Error::device("ds1307: BITS entry is not a list"));
        };
        if parts.len() != 3 {
            return Err(Error::device("ds1307: BITS entry is not [bit, ss, es]"));
        }
        let (Item::Int(_), Item::Int(ss), Item::Int(es)) = (&parts[0], &parts[1], &parts[2]) else {
            return Err(Error::device("ds1307: BITS entry is not [bit, ss, es]"));
        };
        bits.push(Bit { ss: *ss as u64, es: *es as u64 });
    }
    Ok(bits)
}

impl Ds1307 {
    /// `putd`: annotate the samples spanned by two bits.
    fn putd(
        &self,
        ctx: &mut dyn Context,
        bit1: usize,
        bit2: usize,
        class: usize,
        texts: Vec<String>,
    ) {
        ctx.put_ann(self.bits[bit1].ss, self.bits[bit2].es, class, texts);
    }

    /// `putr`: the reserved bit annotation.
    fn putr(&self, ctx: &mut dyn Context, bit: usize) {
        ctx.put_ann(
            self.bits[bit].ss,
            self.bits[bit].es,
            11,
            vec![
                "Reserved bit".to_owned(),
                "Reserved".to_owned(),
                "Rsvd".to_owned(),
                "R".to_owned(),
            ],
        );
    }

    fn handle_reg_0x00(&mut self, ctx: &mut dyn Context, b: i64) {
        // Seconds (0-59) / Clock halt bit.
        self.putd(ctx, 7, 0, 0, vec!["Seconds".to_owned(), "Sec".to_owned(), "S".to_owned()]);
        let ch = i64::from(b & (1 << 7) != 0);
        self.putd(
            ctx,
            7,
            7,
            9,
            vec![
                format!("Clock halt: {ch}"),
                format!("Clk hlt: {ch}"),
                format!("CH: {ch}"),
                "CH".to_owned(),
            ],
        );
        let s = bcd2int(b & 0x7f);
        self.seconds = s;
        self.putd(
            ctx,
            6,
            0,
            10,
            vec![format!("Second: {s}"), format!("Sec: {s}"), format!("S: {s}"), "S".to_owned()],
        );
    }

    fn handle_reg_0x01(&mut self, ctx: &mut dyn Context, b: i64) {
        // Minutes (0-59).
        self.putd(ctx, 7, 0, 1, vec!["Minutes".to_owned(), "Min".to_owned(), "M".to_owned()]);
        self.putr(ctx, 7);
        let m = bcd2int(b & 0x7f);
        self.minutes = m;
        self.putd(
            ctx,
            6,
            0,
            12,
            vec![format!("Minute: {m}"), format!("Min: {m}"), format!("M: {m}"), "M".to_owned()],
        );
    }

    fn handle_reg_0x02(&mut self, ctx: &mut dyn Context, b: i64) {
        // Hours (1-12+AM/PM or 0-23).
        self.putd(ctx, 7, 0, 2, vec!["Hours".to_owned(), "H".to_owned()]);
        self.putr(ctx, 7);
        let ampm_mode = b & (1 << 6) != 0;
        if ampm_mode {
            self.putd(
                ctx,
                6,
                6,
                13,
                vec!["12-hour mode".to_owned(), "12h mode".to_owned(), "12h".to_owned()],
            );
            let a = if b & (1 << 5) != 0 { "PM" } else { "AM" };
            self.putd(ctx, 5, 5, 14, vec![a.to_owned(), a[..1].to_owned()]);
            let h = bcd2int(b & 0x1f);
            self.hours = h;
            self.putd(ctx, 4, 0, 15, vec![format!("Hour: {h}"), format!("H: {h}"), "H".to_owned()]);
        } else {
            self.putd(
                ctx,
                6,
                6,
                13,
                vec!["24-hour mode".to_owned(), "24h mode".to_owned(), "24h".to_owned()],
            );
            let h = bcd2int(b & 0x3f);
            self.hours = h;
            self.putd(ctx, 5, 0, 15, vec![format!("Hour: {h}"), format!("H: {h}"), "H".to_owned()]);
        }
    }

    fn handle_reg_0x03(&mut self, ctx: &mut dyn Context, b: i64) {
        // Day / day of week (1-7).
        self.putd(ctx, 7, 0, 3, vec!["Day of week".to_owned(), "Day".to_owned(), "D".to_owned()]);
        for i in [7, 6, 5, 4, 3] {
            self.putr(ctx, i);
        }
        let w = bcd2int(b & 0x07);
        self.days = w;
        let ws = day_name(w);
        self.putd(
            ctx,
            2,
            0,
            16,
            vec![format!("Weekday: {ws}"), format!("WD: {ws}"), "WD".to_owned(), "W".to_owned()],
        );
    }

    fn handle_reg_0x04(&mut self, ctx: &mut dyn Context, b: i64) {
        // Date (1-31).
        self.putd(ctx, 7, 0, 4, vec!["Date".to_owned(), "D".to_owned()]);
        for i in [7, 6] {
            self.putr(ctx, i);
        }
        let d = bcd2int(b & 0x3f);
        self.date = d;
        self.putd(ctx, 5, 0, 17, vec![format!("Date: {d}"), format!("D: {d}"), "D".to_owned()]);
    }

    fn handle_reg_0x05(&mut self, ctx: &mut dyn Context, b: i64) {
        // Month (1-12).
        self.putd(ctx, 7, 0, 5, vec!["Month".to_owned(), "Mon".to_owned(), "M".to_owned()]);
        for i in [7, 6, 5] {
            self.putr(ctx, i);
        }
        let m = bcd2int(b & 0x1f);
        self.months = m;
        self.putd(
            ctx,
            4,
            0,
            18,
            vec![format!("Month: {m}"), format!("Mon: {m}"), format!("M: {m}"), "M".to_owned()],
        );
    }

    fn handle_reg_0x06(&mut self, ctx: &mut dyn Context, b: i64) {
        // Year (0-99).
        self.putd(ctx, 7, 0, 6, vec!["Year".to_owned(), "Y".to_owned()]);
        let y = bcd2int(b & 0xff);
        self.years = y + 2000;
        // `y` is printed before the 2000 is added, as upstream does.
        self.putd(ctx, 7, 0, 19, vec![format!("Year: {y}"), format!("Y: {y}"), "Y".to_owned()]);
    }

    fn handle_reg_0x07(&mut self, ctx: &mut dyn Context, b: i64) {
        // Control Register.
        self.putd(ctx, 7, 0, 7, vec!["Control".to_owned(), "Ctrl".to_owned(), "C".to_owned()]);
        for i in [6, 5, 3, 2] {
            self.putr(ctx, i);
        }
        let o = i64::from(b & (1 << 7) != 0);
        let s = i64::from(b & (1 << 4) != 0);
        let s2 = if b & (1 << 4) != 0 { "en" } else { "dis" };
        let r = rate(b);
        self.putd(
            ctx,
            7,
            7,
            20,
            vec![
                format!("Output control: {o}"),
                format!("OUT: {o}"),
                format!("O: {o}"),
                "O".to_owned(),
            ],
        );
        self.putd(
            ctx,
            4,
            4,
            21,
            vec![
                format!("Square wave output: {s2}abled"),
                format!("SQWE: {s2}abled"),
                format!("SQWE: {s}"),
                format!("S: {s}"),
                "S".to_owned(),
            ],
        );
        self.putd(
            ctx,
            1,
            0,
            22,
            vec![
                format!("Square wave output rate: {r}"),
                format!("Square wave rate: {r}"),
                format!("SQW rate: {r}"),
                format!("Rate: {r}"),
                format!("RS: {s}"),
                "RS".to_owned(),
                "R".to_owned(),
            ],
        );
    }

    fn handle_reg_0x3f(&self, ctx: &mut dyn Context, b: i64) {
        // RAM (bytes 0x08-0x3f).
        self.putd(ctx, 7, 0, 8, vec!["RAM".to_owned(), "R".to_owned()]);
        self.putd(ctx, 7, 0, 23, vec![format!("SRAM: 0x{b:02X}"), format!("0x{b:02X}")]);
    }

    fn output_datetime(&self, ctx: &mut dyn Context, class: usize, rw: &str) {
        // TODO upstream: Handle read/write of only parts of these items.
        let d = format!(
            "{}, {:02}.{:02}.{:4} {:02}:{:02}:{:02}",
            day_name(self.days),
            self.date,
            self.months,
            self.years,
            self.hours,
            self.minutes,
            self.seconds
        );
        ctx.put_ann(self.ss_block, self.es, class, vec![format!("{rw} date/time: {d}")]);
    }

    fn handle_reg(&mut self, ctx: &mut dyn Context, b: i64) {
        let r = if self.reg < 8 { self.reg } else { 0x3f };
        match r {
            0x00 => self.handle_reg_0x00(ctx, b),
            0x01 => self.handle_reg_0x01(ctx, b),
            0x02 => self.handle_reg_0x02(ctx, b),
            0x03 => self.handle_reg_0x03(ctx, b),
            0x04 => self.handle_reg_0x04(ctx, b),
            0x05 => self.handle_reg_0x05(ctx, b),
            0x06 => self.handle_reg_0x06(ctx, b),
            0x07 => self.handle_reg_0x07(ctx, b),
            _ => self.handle_reg_0x3f(ctx, b),
        }
        // Honor address auto-increment feature of the DS1307. When the
        // address reaches 0x3f, it will wrap around to address 0.
        self.reg += 1;
        if self.reg > 0x3f {
            self.reg = 0;
        }
    }

    fn is_correct_chip(&self, ctx: &mut dyn Context, addr: i64) -> bool {
        if addr == DS1307_I2C_ADDRESS {
            return true;
        }
        ctx.put_ann(
            self.ss_block,
            self.es,
            28,
            vec![format!("Ignoring non-DS1307 data (slave 0x{addr:02X})")],
        );
        false
    }

    fn handle(&mut self, ctx: &mut dyn Context, cmd: &str, databyte: Option<i64>) {
        // START/STOP and ACK/NACK packets carry no data byte; Python's
        // `databyte` is None for those and only data/address branches
        // read it.
        let b = databyte.unwrap_or(0);
        match self.state {
            State::Idle => {
                // Wait for an I2C START condition.
                if cmd != "START" {
                    return;
                }
                self.state = State::GetSlaveAddr;
                self.ss_block = self.ss;
            }
            State::GetSlaveAddr => {
                // Wait for an address write operation.
                if cmd != "ADDRESS WRITE" {
                    return;
                }
                if !self.is_correct_chip(ctx, b) {
                    self.state = State::Idle;
                    return;
                }
                self.state = State::GetRegAddr;
            }
            State::GetRegAddr => {
                // Wait for a data write (master selects the slave register).
                if cmd != "DATA WRITE" {
                    return;
                }
                self.reg = b;
                self.state = State::WriteRtcRegs;
            }
            State::WriteRtcRegs => {
                // If we see a Repeated Start here, it's an RTC read.
                if cmd == "START REPEAT" {
                    self.state = State::ReadRtcRegs;
                    return;
                }
                // Otherwise: Get data bytes until a STOP condition occurs.
                if cmd == "DATA WRITE" {
                    self.handle_reg(ctx, b);
                } else if cmd == "STOP" {
                    self.output_datetime(ctx, 25, "Written");
                    self.state = State::Idle;
                }
            }
            State::ReadRtcRegs => {
                // Wait for an address read operation.
                if cmd != "ADDRESS READ" {
                    return;
                }
                if !self.is_correct_chip(ctx, b) {
                    self.state = State::Idle;
                    return;
                }
                self.state = State::ReadRtcRegs2;
            }
            State::ReadRtcRegs2 => {
                if cmd == "DATA READ" {
                    self.handle_reg(ctx, b);
                } else if cmd == "STOP" {
                    self.output_datetime(ctx, 24, "Read");
                    self.state = State::Idle;
                }
            }
        }
    }
}

impl Decoder for Ds1307 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("ds1307 takes an i2c decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (cmd, data) = split_packet(item)?;

        // Collect the 'BITS' packet, then return. The next packet is
        // guaranteed to belong to these bits we just stored.
        if cmd == "BITS" {
            self.bits = parse_bits(data)?;
            return Ok(());
        }

        // Store the start/end samples of this I2C packet.
        self.ss = ss;
        self.es = es;

        let databyte = match data {
            Item::Int(v) => Some(*v),
            _ => None,
        };
        self.handle(ctx, cmd, databyte);
        Ok(())
    }
}
