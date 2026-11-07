// Counterpart in sigrok:
//   libsigrokdecode/decoders/cc1101/pd.py and lists.py,
//   Copyright (C) 2019 Marco Geisler <m-sigrok@mageis.de>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the CC1101 SPI command/register map and the
//   decoder's documented classes and texts.
#![allow(clippy::semicolon_if_nothing_returned, clippy::unused_self)]

//! CC1101: Texas Instruments low-power sub-1GHz RF transceiver.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

// pd.py's annotation class indices. Note that its annotation tuple has
// only seven entries while these constants run to seven; class 7 is
// emitted for warnings and has no description, exactly as in pd.py.
const ANN_STROBE: usize = 0;
const ANN_SINGLE_READ: usize = 1;
const ANN_SINGLE_WRITE: usize = 2;
const ANN_BURST_READ: usize = 3;
const ANN_BURST_WRITE: usize = 4;
const ANN_STATUS_READ: usize = 5;
const ANN_STATUS: usize = 6;
const ANN_WARN: usize = 7;

/// pd.py's `regs` dict.
const REGS: &[(u64, &str)] = &[
    (0x00, "IOCFG2"),
    (0x01, "IOCFG1"),
    (0x02, "IOCFG0"),
    (0x03, "FIFOTHR"),
    (0x04, "SYNC1"),
    (0x05, "SYNC0"),
    (0x06, "PKTLEN"),
    (0x07, "PKTCTRL1"),
    (0x08, "PKTCTRL0"),
    (0x09, "ADDR"),
    (0x0A, "CHANNR"),
    (0x0B, "FSCTRL1"),
    (0x0C, "FSCTRL0"),
    (0x0D, "FREQ2"),
    (0x0E, "FREQ1"),
    (0x0F, "FREQ0"),
    (0x10, "MDMCFG4"),
    (0x11, "MDMCFG3"),
    (0x12, "MDMCFG2"),
    (0x13, "MDMCFG1"),
    (0x14, "MDMCFG0"),
    (0x15, "DEVIATN"),
    (0x16, "MCSM2"),
    (0x17, "MCSM1"),
    (0x18, "MCSM0"),
    (0x19, "FOCCFG"),
    (0x1A, "BSCFG"),
    (0x1B, "AGCTRL2"),
    (0x1C, "AGCTRL1"),
    (0x1D, "AGCTRL0"),
    (0x1E, "WOREVT1"),
    (0x1F, "WOREVT0"),
    (0x20, "WORCTRL"),
    (0x21, "FREND1"),
    (0x22, "FREND0"),
    (0x23, "FSCAL3"),
    (0x24, "FSCAL2"),
    (0x25, "FSCAL1"),
    (0x26, "FSCAL0"),
    (0x27, "RCCTRL1"),
    (0x28, "RCCTRL0"),
    (0x29, "FSTEST"),
    (0x2A, "PTEST"),
    (0x2B, "AGCTEST"),
    (0x2C, "TEST2"),
    (0x2D, "TEST1"),
    (0x2E, "TEST0"),
    (0x30, "PARTNUM"),
    (0x31, "VERSION"),
    (0x32, "FREQEST"),
    (0x33, "LQI"),
    (0x34, "RSSI"),
    (0x35, "MARCSTATE"),
    (0x36, "WORTIME1"),
    (0x37, "WORTIME0"),
    (0x38, "PKTSTATUS"),
    (0x39, "VCO_VC_DAC"),
    (0x3A, "TXBYTES"),
    (0x3B, "RXBYTES"),
    (0x3C, "RCCTRL1_STATUS"),
    (0x3D, "RCCTRL0_STATUS"),
    (0x3E, "PATABLE"),
    (0x3F, "FIFO"),
];

/// pd.py's `strobes` dict; the empty name for 0x37 is spelling.
const STROBES: &[(u64, &str)] = &[
    (0x30, "SRES"),
    (0x31, "SFSTXON"),
    (0x32, "SXOFF"),
    (0x33, "SCAL"),
    (0x34, "SRX"),
    (0x35, "STX"),
    (0x36, "SIDLE"),
    (0x37, ""),
    (0x38, "SWOR"),
    (0x39, "SPWD"),
    (0x3A, "SFRX"),
    (0x3B, "SFTX"),
    (0x3C, "SWORRST"),
    (0x3D, "SNOP"),
];

/// pd.py's `status_reg_states` dict, indexed by the 3-bit STATE field.
const STATUS_REG_STATES: [&str; 8] =
    ["IDLE", "RX", "TX", "FSTXON", "CALIBRATE", "SETTLING", "RXFIFO_OVERFLOW", "TXFIFO_OVERFLOW"];

/// The decoder description. The annotations are those of pd.py, whose
/// class indexes and description tuple are one apart for the last two.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "cc1101",
    name: "CC1101",
    longname: "Texas Instruments CC1101",
    desc: "Low-power sub-1GHz RF transceiver chip.",
    license: "gplv2+",
    inputs: &["spi"],
    outputs: &[],
    tags: &["IC", "Wireless/RF"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "strobe", desc: "Command strobe" },
        AnnClass { id: "single_read", desc: "Single register read" },
        AnnClass { id: "single_write", desc: "Single register write" },
        AnnClass { id: "burst_read", desc: "Burst register read" },
        AnnClass { id: "burst_write", desc: "Burst register write" },
        AnnClass { id: "status", desc: "Status register" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "cmd", desc: "Commands", classes: &[ANN_STROBE] },
        AnnRow {
            id: "data",
            desc: "Data",
            classes: &[
                ANN_SINGLE_READ,
                ANN_SINGLE_WRITE,
                ANN_BURST_READ,
                ANN_BURST_WRITE,
                ANN_STATUS_READ,
            ],
        },
        AnnRow { id: "status", desc: "Status register", classes: &[ANN_STATUS] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[ANN_WARN] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'spi' PD and decodes the protocol spoken\nby the Texas Instruments low-power sub-1GHz RF transceiver chips.\n\nDetails:\nhttp://www.ti.com/lit/ds/symlink/cc1101.pdf",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Cc1101::default())
}

/// A position, pd.py's `Pos` namedtuple.
#[derive(Clone, Copy)]
struct Pos {
    ss: u64,
    es: u64,
}

/// A non-negative Python integer from an SPI DATA item.
fn value(item: &Item) -> Result<u64> {
    match item {
        Item::Int(v) if *v >= 0 => Ok(*v as u64),
        Item::BigUint(bytes) => {
            let mut v = 0u64;
            for &b in bytes {
                v = v << 8 | u64::from(b);
            }
            Ok(v)
        }
        _ => Err(Error::arg(format!("cc1101: not an SPI word: {item}"))),
    }
}

/// pd.py's command names.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Cmd {
    Write,
    BurstWrite,
    Read,
    BurstRead,
    Strobe,
    StatusRead,
}

impl Cmd {
    fn name(self) -> &'static str {
        match self {
            Cmd::Write => "Write",
            Cmd::BurstWrite => "Burst write",
            Cmd::Read => "Read",
            Cmd::BurstRead => "Burst read",
            Cmd::Strobe => "Strobe",
            Cmd::StatusRead => "Status read",
        }
    }

    /// pd.py's RX/TX FIFO test, verbatim: `('Single read', 'Status
    /// read', 'Burst read')`. A plain `Read` is not in it, so its status
    /// byte reports the TX FIFO.
    fn is_read(self) -> bool {
        matches!(self, Cmd::BurstRead | Cmd::StatusRead)
    }
}

/// One `Data` namedtuple: the mosi and miso bytes of a data byte.
#[derive(Clone, Copy)]
struct Data {
    mosi: u64,
    miso: u64,
}

/// The CC1101 decoder.
#[derive(Default)]
pub struct Cc1101 {
    // pd.py's reset(): next() plus the two flags.
    first: bool,
    cmd: Option<Cmd>,
    dat: u64,
    min: usize,
    max: usize,
    mb: Vec<Data>,
    ss_mb: i64,
    es_mb: i64,
    requirements_met: bool,
    cs_was_released: bool,
}

impl Cc1101 {
    fn next(&mut self) {
        self.first = true;
        self.cmd = None;
        self.dat = 0;
        self.min = 0;
        self.max = 0;
        self.mb.clear();
        self.ss_mb = -1;
        self.es_mb = -1;
    }

    fn reset(&mut self) {
        *self = Cc1101 { requirements_met: true, ..Cc1101::default() };
        self.next();
    }

    fn warn(&self, ctx: &mut dyn Context, pos: Pos, msg: &str) {
        ctx.put_ann(pos.ss, pos.es, ANN_WARN, vec![msg.to_owned()]);
    }

    fn putp(&self, ctx: &mut dyn Context, pos: Pos, ann: usize, msg: String) {
        ctx.put_ann(pos.ss, pos.es, ann, vec![msg]);
    }

    fn putp2(&self, ctx: &mut dyn Context, pos: Pos, ann: usize, msg1: String, msg2: String) {
        ctx.put_ann(pos.ss, pos.es, ann, vec![msg1, msg2]);
    }

    fn mosi_bytes(&self) -> Vec<u64> {
        self.mb.iter().map(|b| b.mosi).collect()
    }

    fn miso_bytes(&self) -> Vec<u64> {
        self.mb.iter().map(|b| b.miso).collect()
    }

    /// pd.py: `format_command`.
    fn format_command(&self) -> String {
        let Some(cmd) = self.cmd else {
            return String::new();
        };
        match cmd {
            Cmd::Read | Cmd::BurstRead | Cmd::Write | Cmd::BurstWrite | Cmd::StatusRead => {
                cmd.name().to_owned()
            }
            Cmd::Strobe => {
                let reg = STROBES
                    .iter()
                    .find(|(addr, _)| *addr == self.dat)
                    .map_or("unknown strobe", |(_, name)| *name);
                format!("{} {}", cmd.name(), reg)
            }
        }
    }

    /// pd.py: `parse_command`. The two `warn` calls in its unreachable
    /// fall-through branches are not reproduced.
    fn parse_command(&self, b: u64) -> Option<(Cmd, u64, usize, usize)> {
        let addr = b & 0x3F;
        if addr < 0x30 || addr == 0x3E || addr == 0x3F {
            match b & 0xC0 {
                0x00 => Some((Cmd::Write, addr, 1, 1)),
                0x40 => Some((Cmd::BurstWrite, addr, 1, 99999)),
                0x80 => Some((Cmd::Read, addr, 1, 1)),
                _ => Some((Cmd::BurstRead, addr, 1, 99999)),
            }
        } else if b & 0x40 == 0 {
            Some((Cmd::Strobe, addr, 0, 0))
        } else if b & 0xC0 == 0xC0 {
            Some((Cmd::StatusRead, addr, 1, 99999))
        } else {
            None
        }
    }

    /// pd.py: `decode_command`.
    fn decode_command(&mut self, ctx: &mut dyn Context, pos: Pos, b: u64) {
        let Some((cmd, dat, min, max)) = self.parse_command(b) else {
            self.warn(ctx, pos, "unknown command");
            return;
        };
        self.cmd = Some(cmd);
        self.dat = dat;
        self.min = min;
        self.max = max;
        if cmd == Cmd::Strobe {
            let text = self.format_command();
            self.putp(ctx, pos, ANN_STROBE, text);
        } else {
            // Don't output anything now, the command is merged with the
            // data bytes following it.
            self.ss_mb = pos.ss as i64;
        }
    }

    /// pd.py: `decode_reg`, for an integer register id.
    fn decode_reg(&self, ctx: &mut dyn Context, pos: Pos, ann: usize, regid: u64, data: &[u64]) {
        let Some((_, reg_name)) = REGS.iter().find(|(addr, _)| *addr == regid) else {
            self.warn(ctx, pos, "unknown register");
            return;
        };
        let name = format!("{reg_name} ({regid:02X})");
        // pd.py's else branch formats `self.cmd` directly; an integer
        // register is only decoded from finish_command, where it is set.
        let label = match self.cmd {
            Some(_) => format!("{}: {name}", self.format_command()),
            None => format!("Reg (None) {name}"),
        };
        self.decode_mb_data(ctx, pos, ann, data, &label);
    }

    /// pd.py: `decode_reg`, for the `'STATUS'` string id.
    fn decode_status(&self, ctx: &mut dyn Context, pos: Pos, ann: usize, data: &[u64]) {
        self.decode_status_reg(ctx, pos, ann, data, "Status");
    }

    /// pd.py: `decode_status_reg`.
    fn decode_status_reg(
        &self,
        ctx: &mut dyn Context,
        pos: Pos,
        ann: usize,
        data: &[u64],
        label: &str,
    ) {
        let status = data[0];
        let longtext_chiprdy =
            if status & 0b1000_0000 == 0b1000_0000 { "CHIP_RDYn is high! " } else { "" };
        let state = ((status & 0x70) >> 4) as usize;
        let longtext_state = format!("STATE is {}, ", STATUS_REG_STATES[state]);
        let fifo_bytes = status & 0x0F;
        let longtext_fifo = if self.cmd.is_some_and(Cmd::is_read) {
            format!("{fifo_bytes} bytes available in RX FIFO")
        } else {
            format!("{fifo_bytes} bytes free in TX FIFO")
        };
        let text = format!("{label} = {status:02X}");
        let longtext = format!("{text}; {longtext_chiprdy}{longtext_state}{longtext_fifo}");
        self.putp2(ctx, pos, ann, longtext, text);
    }

    /// pd.py: `decode_mb_data`.
    fn decode_mb_data(
        &self,
        ctx: &mut dyn Context,
        pos: Pos,
        ann: usize,
        data: &[u64],
        label: &str,
    ) {
        let data = data.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
        let text = format!("{label} = {data}");
        self.putp(ctx, pos, ann, text);
    }

    /// pd.py: `finish_command`.
    fn finish_command(&self, ctx: &mut dyn Context, pos: Pos) {
        let Some(cmd) = self.cmd else {
            self.warn(ctx, pos, "unhandled command");
            return;
        };
        match cmd {
            Cmd::Write => self.decode_reg(ctx, pos, ANN_SINGLE_WRITE, self.dat, &self.mosi_bytes()),
            Cmd::BurstWrite => {
                self.decode_reg(ctx, pos, ANN_BURST_WRITE, self.dat, &self.mosi_bytes())
            }
            Cmd::Read => self.decode_reg(ctx, pos, ANN_SINGLE_READ, self.dat, &self.miso_bytes()),
            Cmd::BurstRead => {
                self.decode_reg(ctx, pos, ANN_BURST_READ, self.dat, &self.miso_bytes())
            }
            Cmd::Strobe => self.decode_reg(ctx, pos, ANN_STROBE, self.dat, &self.mosi_bytes()),
            Cmd::StatusRead => {
                self.decode_reg(ctx, pos, ANN_STATUS_READ, self.dat, &self.miso_bytes())
            }
        }
    }
}

/// Splits an item from the SPI decoder into `(ptype, data1, data2)`.
fn split_item(item: &Item) -> Result<(&str, &Item, &Item)> {
    let Item::List(parts) = item else {
        return Err(Error::arg("cc1101: item is not a list"));
    };
    let [ptype, data1, data2] = parts.as_slice() else {
        return Err(Error::arg("cc1101: item is not a three element packet"));
    };
    let Item::Str(ptype) = ptype else {
        return Err(Error::arg("cc1101: packet type is not a string"));
    };
    Ok((ptype.as_str(), data1, data2))
}

impl Decoder for Cc1101 {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let _ = config;
        self.reset();
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let _ = ctx;
        Err(Error::not_supported("cc1101 is a stacked decoder on 'spi'"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        if !self.requirements_met {
            return Ok(());
        }

        let (ptype, data1, data2) = split_item(item)?;

        if ptype == "CS-CHANGE" {
            if matches!(data1, Item::None) {
                if matches!(data2, Item::None) {
                    self.requirements_met = false;
                    return Err(Error::arg("CS# pin required."));
                } else if matches!(data2, Item::Int(1)) {
                    self.cs_was_released = true;
                }
            }

            if matches!(data1, Item::Int(0)) && matches!(data2, Item::Int(1)) {
                // Rising edge, the complete command is transmitted,
                // process the bytes that were sent after the command byte.
                if self.cmd.is_some() {
                    // Check if we got the minimum number of data bytes
                    // after the command byte.
                    if self.mb.len() < self.min {
                        self.warn(ctx, Pos { ss, es: ss }, "missing data bytes");
                    } else if !self.mb.is_empty() {
                        self.finish_command(
                            ctx,
                            Pos { ss: self.ss_mb as u64, es: self.es_mb as u64 },
                        );
                    }
                }
                self.next();
                self.cs_was_released = true;
            }
        } else if ptype == "DATA" && self.cs_was_released {
            if matches!(data1, Item::None) || matches!(data2, Item::None) {
                // pd.py raises before touching the packet when either
                // pin is missing.
                self.requirements_met = false;
                return Err(Error::arg("Both MISO and MOSI pins required."));
            }
            let mosi = value(data1)?;
            let miso = value(data2)?;
            let pos = Pos { ss, es };

            if self.first {
                self.first = false;
                // First MOSI byte is always the command.
                self.decode_command(ctx, pos, mosi);
                // First MISO byte is always the status register.
                self.decode_status(ctx, pos, ANN_STATUS, &[miso]);
            } else if self.cmd.is_none() || self.mb.len() >= self.max {
                self.warn(ctx, pos, "excess byte");
            } else {
                // Collect the bytes after the command byte.
                if self.ss_mb == -1 {
                    self.ss_mb = ss as i64;
                }
                self.es_mb = es as i64;
                self.mb.push(Data { mosi, miso });
            }
        }
        Ok(())
    }
}
