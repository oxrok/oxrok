// Counterpart in sigrok:
//   libsigrokdecode/decoders/rtc8564/pd.py,
//   Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the I2C bus protocol and the decoder's documented
//   registers, bits and texts.

//! Epson RTC-8564 JE/NB: control, date/time and alarm registers.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

/// `bcd2int` from common/srdhelper.
fn bcd2int(b: i64) -> i64 {
    (b & 0x0f) + ((b >> 4) * 10)
}

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "rtc8564",
    name: "RTC-8564",
    longname: "Epson RTC-8564 JE/NB",
    desc: "Realtime clock module protocol.",
    license: "gplv2+",
    inputs: &["i2c"],
    outputs: &[],
    tags: &["Clock/timing"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        // `reg_list()`: register 0x00 to 0x08, one class each.
        AnnClass { id: "reg-0x00", desc: "Register 0x00" },
        AnnClass { id: "reg-0x01", desc: "Register 0x01" },
        AnnClass { id: "reg-0x02", desc: "Register 0x02" },
        AnnClass { id: "reg-0x03", desc: "Register 0x03" },
        AnnClass { id: "reg-0x04", desc: "Register 0x04" },
        AnnClass { id: "reg-0x05", desc: "Register 0x05" },
        AnnClass { id: "reg-0x06", desc: "Register 0x06" },
        AnnClass { id: "reg-0x07", desc: "Register 0x07" },
        AnnClass { id: "reg-0x08", desc: "Register 0x08" },
        AnnClass { id: "read", desc: "Read date/time" },
        AnnClass { id: "write", desc: "Write date/time" },
        AnnClass { id: "bit-reserved", desc: "Reserved bit" },
        AnnClass { id: "bit-vl", desc: "VL bit" },
        AnnClass { id: "bit-century", desc: "Century bit" },
        AnnClass { id: "reg-read", desc: "Register read" },
        AnnClass { id: "reg-write", desc: "Register write" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13] },
        AnnRow { id: "regs", desc: "Register access", classes: &[14, 15] },
        AnnRow { id: "date-time", desc: "Date/time", classes: &[9, 10] },
    ],
    binary: &[],
    // The doc is the package's __init__.py docstring, which the Python
    // host reads from the imported module.
    doc: "This decoder stacks on top of the 'i2c' PD and decodes the Epson\n\
RTC-8564 JE/NB real-time clock (RTC) protocol.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Rtc8564::default())
}

/// One I2C bit: value and sample range. Index 0 is the LSB, as in the
/// BITS packet the i2c decoder sends.
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

/// The rtc8564 decoder.
pub struct Rtc8564 {
    state: State,
    hours: i64,
    minutes: i64,
    seconds: i64,
    days: i64,
    weekdays: i64,
    months: i64,
    years: i64,
    bits: Vec<Bit>,
    ss: u64,
    es: u64,
    ss_block: u64,
    reg: i64,
}

impl Default for Rtc8564 {
    fn default() -> Self {
        Rtc8564 {
            state: State::Idle,
            hours: -1,
            minutes: -1,
            seconds: -1,
            days: -1,
            weekdays: -1,
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
        return Err(Error::device("rtc8564: expected an I2C packet"));
    };
    if parts.len() != 2 {
        return Err(Error::device("rtc8564: expected a two element I2C packet"));
    }
    let Item::Str(cmd) = &parts[0] else {
        return Err(Error::device("rtc8564: packet type is not a string"));
    };
    Ok((cmd.as_str(), &parts[1]))
}

/// The BITS packet payload is a list of `[bit, ss, es]`, index 0 the LSB.
fn parse_bits(data: &Item) -> Result<Vec<Bit>> {
    let Item::List(items) = data else {
        return Err(Error::device("rtc8564: BITS payload is not a list"));
    };
    let mut bits = Vec::with_capacity(items.len());
    for item in items {
        let Item::List(parts) = item else {
            return Err(Error::device("rtc8564: BITS entry is not a list"));
        };
        if parts.len() != 3 {
            return Err(Error::device("rtc8564: BITS entry is not [bit, ss, es]"));
        }
        let (Item::Int(_), Item::Int(ss), Item::Int(es)) = (&parts[0], &parts[1], &parts[2]) else {
            return Err(Error::device("rtc8564: BITS entry is not [bit, ss, es]"));
        };
        bits.push(Bit { ss: *ss as u64, es: *es as u64 });
    }
    Ok(bits)
}

impl Rtc8564 {
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

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

    #[allow(clippy::unused_self)]
    fn handle_reg_0x00(&mut self, _ctx: &mut dyn Context, _b: i64) {
        // Control register 1.
    }

    #[allow(clippy::format_push_string)]
    fn handle_reg_0x01(&mut self, ctx: &mut dyn Context, b: i64) {
        // Control register 2.
        let ti_tp = i64::from(b & (1 << 4) != 0);
        let af = i64::from(b & (1 << 3) != 0);
        let tf = i64::from(b & (1 << 2) != 0);
        let aie = i64::from(b & (1 << 1) != 0);
        let tie = i64::from(b & (1 << 0) != 0);

        let mut ann = String::new();
        let s = if ti_tp != 0 { "repeated" } else { "single-shot" };
        // The "interrupt events\n" split is one Python string literal
        // continued with a backslash, so there is no space at the break.
        ann.push_str(&format!(
            "TI/TP = {ti_tp}: {s} operation upon fixed-cycle timer interrupt events\n"
        ));
        let s = if af != 0 { "" } else { "no " };
        ann.push_str(&format!("AF = {af}: {s}alarm interrupt detected\n"));
        let s = if tf != 0 { "" } else { "no " };
        ann.push_str(&format!("TF = {tf}: {s}fixed-cycle timer interrupt detected\n"));
        let s = if aie != 0 { "enabled" } else { "prohibited" };
        ann.push_str(&format!("AIE = {aie}: INT# pin output {s} when an alarm interrupt occurs\n"));
        let s = if tie != 0 { "enabled" } else { "prohibited" };
        ann.push_str(&format!(
            "TIE = {tie}: INT# pin output {s} when a fixed-cycle interrupt event occurs\n"
        ));

        self.putx(ctx, 1, vec![ann]);
    }

    fn handle_reg_0x02(&mut self, ctx: &mut dyn Context, b: i64) {
        // Seconds / Voltage-low bit.
        let vl = i64::from(b & (1 << 7) != 0);
        self.putd(
            ctx,
            7,
            7,
            12,
            vec![
                format!("Voltage low: {vl}"),
                format!("Volt. low: {vl}"),
                format!("VL: {vl}"),
                "VL".to_owned(),
            ],
        );
        let s = bcd2int(b & 0x7f);
        self.seconds = s;
        self.putd(
            ctx,
            6,
            0,
            2,
            vec![format!("Second: {s}"), format!("Sec: {s}"), format!("S: {s}"), "S".to_owned()],
        );
    }

    fn handle_reg_0x03(&mut self, ctx: &mut dyn Context, b: i64) {
        // Minutes.
        self.putr(ctx, 7);
        let m = bcd2int(b & 0x7f);
        self.minutes = m;
        self.putd(
            ctx,
            6,
            0,
            3,
            vec![format!("Minute: {m}"), format!("Min: {m}"), format!("M: {m}"), "M".to_owned()],
        );
    }

    fn handle_reg_0x04(&mut self, ctx: &mut dyn Context, b: i64) {
        // Hours.
        self.putr(ctx, 7);
        self.putr(ctx, 6);
        let h = bcd2int(b & 0x3f);
        self.hours = h;
        self.putd(ctx, 5, 0, 4, vec![format!("Hour: {h}"), format!("H: {h}"), "H".to_owned()]);
    }

    fn handle_reg_0x05(&mut self, ctx: &mut dyn Context, b: i64) {
        // Days.
        self.putr(ctx, 7);
        self.putr(ctx, 6);
        let d = bcd2int(b & 0x3f);
        self.days = d;
        self.putd(ctx, 5, 0, 5, vec![format!("Day: {d}"), format!("D: {d}"), "D".to_owned()]);
    }

    fn handle_reg_0x06(&mut self, ctx: &mut dyn Context, b: i64) {
        // Weekdays.
        for i in [7, 6, 5, 4, 3] {
            self.putr(ctx, i);
        }
        let w = bcd2int(b & 0x07);
        self.weekdays = w;
        self.putd(
            ctx,
            2,
            0,
            6,
            vec![format!("Weekday: {w}"), format!("WD: {w}"), "WD".to_owned(), "W".to_owned()],
        );
    }

    fn handle_reg_0x07(&mut self, ctx: &mut dyn Context, b: i64) {
        // Months / century bit.
        let c = i64::from(b & (1 << 7) != 0);
        self.putd(
            ctx,
            7,
            7,
            13,
            vec![
                format!("Century bit: {c}"),
                format!("Century: {c}"),
                format!("Cent: {c}"),
                format!("C: {c}"),
                "C".to_owned(),
            ],
        );
        self.putr(ctx, 6);
        self.putr(ctx, 5);
        let m = bcd2int(b & 0x1f);
        self.months = m;
        self.putd(
            ctx,
            4,
            0,
            7,
            vec![format!("Month: {m}"), format!("Mon: {m}"), format!("M: {m}"), "M".to_owned()],
        );
    }

    fn handle_reg_0x08(&mut self, ctx: &mut dyn Context, b: i64) {
        // Years.
        let y = bcd2int(b & 0xff);
        self.years = y;
        // Upstream uses `putx` here, so the annotation spans the whole
        // byte rather than a bit range.
        self.putx(ctx, 8, vec![format!("Year: {y}"), format!("Y: {y}"), "Y".to_owned()]);
    }

    fn handle_reg(&mut self, ctx: &mut dyn Context, b: i64) {
        match self.reg {
            0x00 => self.handle_reg_0x00(ctx, b),
            0x01 => self.handle_reg_0x01(ctx, b),
            0x02 => self.handle_reg_0x02(ctx, b),
            0x03 => self.handle_reg_0x03(ctx, b),
            0x04 => self.handle_reg_0x04(ctx, b),
            0x05 => self.handle_reg_0x05(ctx, b),
            0x06 => self.handle_reg_0x06(ctx, b),
            0x07 => self.handle_reg_0x07(ctx, b),
            0x08 => self.handle_reg_0x08(ctx, b),
            // 0x09..0x0f (alarm, CLKOUT, timer) are no-ops upstream.
            _ => {}
        }
    }

    fn put_datetime(&self, ctx: &mut dyn Context, es: u64, class: usize, rw: &str) {
        let d = format!(
            "{:02}.{:02}.{:02} {:02}:{:02}:{:02}",
            self.days, self.months, self.years, self.hours, self.minutes, self.seconds
        );
        ctx.put_ann(
            self.ss_block,
            es,
            class,
            vec![
                format!("{rw} date/time: {d}"),
                format!("{rw}: {d}"),
                format!("{}: {d}", &rw[..1]),
            ],
        );
    }

    fn handle(&mut self, ctx: &mut dyn Context, cmd: &str, databyte: Option<i64>, es: u64) {
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
                // TODO upstream: only handle packets to the RTC slave
                // (0xa2/0xa3).
                if cmd != "ADDRESS WRITE" {
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
                // If we see a Repeated Start here, it's probably an RTC read.
                if cmd == "START REPEAT" {
                    self.state = State::ReadRtcRegs;
                    return;
                }
                // Otherwise: Get data bytes until a STOP condition occurs.
                if cmd == "DATA WRITE" {
                    let s = format!("{:02X}: {:02X}", self.reg, b);
                    self.putx(
                        ctx,
                        15,
                        vec![
                            format!("Write register {s}"),
                            format!("Write reg {s}"),
                            format!("WR {s}"),
                            "WR".to_owned(),
                            "W".to_owned(),
                        ],
                    );
                    self.handle_reg(ctx, b);
                    self.reg += 1;
                    // TODO upstream: Check for NACK!
                } else if cmd == "STOP" {
                    // TODO upstream: Handle read/write of only parts of
                    // these items.
                    self.put_datetime(ctx, es, 9, "Write");
                    self.state = State::Idle;
                }
            }
            State::ReadRtcRegs => {
                // Wait for an address read operation.
                // TODO upstream: only handle packets to the RTC slave
                // (0xa2/0xa3).
                if cmd == "ADDRESS READ" {
                    self.state = State::ReadRtcRegs2;
                }
            }
            State::ReadRtcRegs2 => {
                if cmd == "DATA READ" {
                    let s = format!("{:02X}: {:02X}", self.reg, b);
                    self.putx(
                        ctx,
                        15,
                        vec![
                            format!("Read register {s}"),
                            format!("Read reg {s}"),
                            format!("RR {s}"),
                            "RR".to_owned(),
                            "R".to_owned(),
                        ],
                    );
                    self.handle_reg(ctx, b);
                    self.reg += 1;
                    // TODO upstream: Check for NACK!
                } else if cmd == "STOP" {
                    self.put_datetime(ctx, es, 10, "Read");
                    self.state = State::Idle;
                }
            }
        }
    }
}

impl Decoder for Rtc8564 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("rtc8564 takes an i2c decoder as input"))
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
        self.handle(ctx, cmd, databyte, es);
        Ok(())
    }
}
