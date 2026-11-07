// Counterpart in sigrok:
//   libsigrokdecode/decoders/qi/pd.py,
//   Copyright (C) 2015 Josef Gajdusek <atx@atx.name>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the demodulated pulse coding and the decoder's
//   documented classes and texts.

//! Qi: demodulated receiver-to-charger packets.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};
use std::collections::VecDeque;

const QI: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_BITS: usize = 0;
const ANN_BYTES_ERRORS: usize = 1;
const ANN_BYTES_START: usize = 2;
const ANN_BYTES_INFO: usize = 3;
const ANN_BYTES_DATA: usize = 4;
const ANN_PACKETS_DATA: usize = 5;
const ANN_PACKETS_CHECKSUM: usize = 6;

/// pd.py's `end_codes` tuple.
const END_CODES: [&str; 9] = [
    "Unknown",
    "Charge Complete",
    "Internal Fault",
    "Over Temperature",
    "Over Voltage",
    "Over Current",
    "Battery Failure",
    "Reconfigure",
    "No Response",
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "qi",
    name: "Qi",
    longname: "Qi charger protocol",
    desc: "Protocol used by Qi receiver.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Embedded/industrial", "Wireless/RF"],
    channels: &[ChannelDesc { id: "qi", name: "Qi", desc: "Demodulated Qi data line" }],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "bits", desc: "Bits" },
        AnnClass { id: "bytes-errors", desc: "Bit errors" },
        AnnClass { id: "bytes-start", desc: "Start bits" },
        AnnClass { id: "bytes-info", desc: "Info bits" },
        AnnClass { id: "bytes-data", desc: "Data bytes" },
        AnnClass { id: "packets-data", desc: "Packet data" },
        AnnClass { id: "packets-checksum-ok", desc: "Packet checksum" },
        AnnClass { id: "packets-checksum-err", desc: "Packet checksum" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0] },
        AnnRow { id: "bytes", desc: "Bytes", classes: &[1, 2, 3, 4] },
        AnnRow { id: "packets", desc: "Packets", classes: &[5, 6, 7] },
    ],
    binary: &[],
    doc: "This decoder decodes demodulated data streams used by the Qi standard\n\
          for communication from the receiver to the charging station.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Qi::new())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Data,
}

/// The Qi decoder.
pub struct Qi {
    samplerate: u64,
    bit_width: f64,
    state: State,
    lastbit: u64,
    bytestart: u64,
    deq: VecDeque<i64>,
    bits: Vec<u8>,
    bitsi: Vec<u64>,
    bytesi: Vec<u64>,
    packet: Vec<u8>,
}

impl Qi {
    fn new() -> Self {
        Qi {
            samplerate: 0,
            bit_width: 0.0,
            state: State::Idle,
            lastbit: 0,
            bytestart: 0,
            deq: VecDeque::with_capacity(2),
            bits: Vec::new(),
            bitsi: Vec::new(),
            bytesi: Vec::new(),
            packet: Vec::new(),
        }
    }

    fn reset_variables(&mut self) {
        self.state = State::Idle;
        self.lastbit = 0;
        self.bytestart = 0;
        self.deq.clear();
        self.bits.clear();
        self.bitsi = vec![0];
        self.bytesi.clear();
        self.packet.clear();
    }

    fn packet_len(byte: u8) -> usize {
        let b = i64::from(byte);
        if (0x00..=0x1f).contains(&b) {
            (1 + b / 32) as usize
        } else if (0x20..=0x7f).contains(&b) {
            (2 + (b - 32) / 16) as usize
        } else if (0x80..=0xdf).contains(&b) {
            (8 + (b - 128) / 8) as usize
        } else {
            (20 + (b - 224) / 4) as usize
        }
    }

    fn in_tolerance(&self, l: f64) -> bool {
        (0.75 * self.bit_width) < l && l < (1.25 * self.bit_width)
    }

    fn putp(&self, ctx: &mut dyn Context, texts: Vec<String>) {
        let ss = self.bytesi[0];
        let es = *self.bytesi.last().expect("bytesi");
        ctx.put_ann(ss, es, ANN_PACKETS_DATA, texts);
    }

    #[allow(clippy::too_many_lines)]
    fn process_packet(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let p = &self.packet;
        match p[0] {
            0x01 => self.putp(
                ctx,
                vec![format!("Signal Strength: {}", p[1]), format!("SS: {}", p[1]), "SS".into()],
            ),
            0x02 => {
                let reason = if usize::from(p[1]) < END_CODES.len() {
                    END_CODES[p[1] as usize]
                } else {
                    "Reserved"
                };
                self.putp(
                    ctx,
                    vec![
                        format!("End Power Transfer: {reason}"),
                        format!("EPT: {reason}"),
                        "EPT".into(),
                    ],
                );
            }
            0x03 => {
                let val = if p[1] < 128 { i64::from(p[1]) } else { i64::from(p[1] & 0x7f) - 128 };
                self.putp(
                    ctx,
                    vec![format!("Control Error: {val}"), format!("CE: {val}"), "CE".into()],
                );
            }
            0x04 => self.putp(
                ctx,
                vec![format!("Received Power: {}", p[1]), format!("RP: {}", p[1]), "RP".into()],
            ),
            0x05 => self.putp(
                ctx,
                vec![format!("Charge Status: {}", p[1]), format!("CS: {}", p[1]), "CS".into()],
            ),
            0x06 => {
                // pd.py calls `self.putp([...], 'PCH')`, which is a
                // TypeError (putp takes one argument besides self); the
                // decoder stops there.
                return Err(Error::device("putp() takes 2 positional arguments but 3 were given"));
            }
            0x51 => {
                let powerclass = (p[1] & 0xc0) >> 7;
                let maxpower = p[1] & 0x3f;
                let prop = (p[3] & 0x80) >> 7;
                let count = p[3] & 0x07;
                let winsize = (p[4] & 0xf8) >> 3;
                let winoff = p[4] & 0x07;
                self.putp(
                    ctx,
                    vec![
                        format!(
                            "Configuration: Power Class = {powerclass}, Maximum Power = {maxpower}, Prop = {prop},Count = {count}, Window Size = {winsize}, Window Offset = {winoff}"
                        ),
                        format!(
                            "C: PC = {powerclass} MP = {maxpower} P = {prop} C = {count} WS = {winsize} WO = {winoff}"
                        ),
                        "Configuration".into(),
                        "C".into(),
                    ],
                );
            }
            0x71 => {
                let version = format!("{}.{}", (p[1] & 0xf0) >> 4, p[1] & 0x0f);
                let mancode = format!("{:02x}{:02x}", p[2], p[3]);
                let devid = format!("{:02x}{:02x}{:02x}{:02x}", p[4] & !0x80, p[5], p[6], p[7]);
                self.putp(
                    ctx,
                    vec![
                        format!(
                            "Identification: Version = {version}, Manufacturer = {mancode}, Device = {devid}"
                        ),
                        format!("ID: {version} {mancode} {devid}"),
                        "ID".into(),
                    ],
                );
            }
            0x81 => {
                let edevid = p[1..p.len() - 1].iter().fold(String::new(), |mut s, b| {
                    use std::fmt::Write;
                    let _ = write!(s, "{b:02x}");
                    s
                });
                self.putp(
                    ctx,
                    vec![
                        format!("Extended Identification: {edevid}"),
                        format!("EI: {edevid}"),
                        "EI".into(),
                    ],
                );
            }
            0x18 | 0x19 | 0x28 | 0x29 | 0x38 | 0x48 | 0x58 | 0x68 | 0x78 | 0x85 | 0xa4 | 0xc4
            | 0xe2 => self.putp(ctx, vec!["Proprietary".into(), "P".into()]),
            _ => self.putp(ctx, vec!["Unknown".into(), "?".into()]),
        }
        let mut checksum = 0u8;
        for &b in &self.packet[..self.packet.len() - 1] {
            checksum ^= b;
        }
        let es = ctx.samplenum();
        let ss = *self.bytesi.last().expect("bytesi");
        if checksum == self.packet[self.packet.len() - 1] {
            ctx.put_ann(ss, es, ANN_PACKETS_CHECKSUM, vec!["Checksum OK".into(), "OK".into()]);
        } else {
            ctx.put_ann(ss, es, ANN_PACKETS_CHECKSUM, vec!["Checksum error".into(), "ERR".into()]);
        }
        Ok(())
    }

    fn process_byte(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let start = if self.bits[0] == 0 {
            (ANN_BYTES_START, vec!["Start bit".to_owned(), "Start".to_owned(), "S".to_owned()])
        } else {
            (
                ANN_BYTES_ERRORS,
                vec!["Start error".to_owned(), "Start err".to_owned(), "SE".to_owned()],
            )
        };
        ctx.put_ann(self.bytestart, self.bitsi[0], start.0, start.1);
        let databits = self.bits[1..9].to_vec();
        let data = Self::bits_to_uint(&databits);
        let mut parity = 1u32;
        for &v in &databits {
            parity = (parity + u32::from(v)) % 2;
        }
        ctx.put_ann(self.bitsi[0], self.bitsi[8], ANN_BYTES_DATA, vec![format!("{data:02x}")]);
        if u32::from(self.bits[9]) == parity {
            ctx.put_ann(
                self.bitsi[8],
                self.bitsi[9],
                ANN_BYTES_INFO,
                vec!["Parity bit".into(), "Parity".into(), "P".into()],
            );
        } else {
            ctx.put_ann(
                self.bitsi[8],
                self.bitsi[9],
                ANN_BYTES_ERRORS,
                vec!["Parity error".into(), "Parity err".into(), "PE".into()],
            );
        }
        if self.bits[10] == 1 {
            ctx.put_ann(
                self.bitsi[9],
                self.bitsi[10],
                ANN_BYTES_INFO,
                vec!["Stop bit".into(), "Stop".into(), "S".into()],
            );
        } else {
            ctx.put_ann(
                self.bitsi[9],
                self.bitsi[10],
                ANN_BYTES_ERRORS,
                vec!["Stop error".into(), "Stop err".into(), "SE".into()],
            );
        }

        self.bytesi.push(self.bytestart);
        self.packet.push(data);
        if Self::packet_len(self.packet[0]) + 2 == self.packet.len() {
            self.process_packet(ctx)?;
            self.bytesi.clear();
            self.packet.clear();
        }
        Ok(())
    }

    fn add_bit(&mut self, ctx: &mut dyn Context, bit: u8) -> Result<()> {
        self.bits.push(bit);
        self.bitsi.push(ctx.samplenum());

        if self.state == State::Idle
            && self.bits.len() >= 5
            && self.bits[self.bits.len() - 5..] == [1, 1, 1, 1, 0]
        {
            self.state = State::Data;
            self.bytestart = self.bitsi[self.bitsi.len() - 2];
            self.bits = vec![0];
            self.bitsi = vec![ctx.samplenum()];
            self.packet.clear();
        } else if self.state == State::Data && self.bits.len() == 11 {
            self.process_byte(ctx)?;
            self.bytestart = ctx.samplenum();
            self.bits.clear();
            self.bitsi.clear();
        }
        if self.state != State::Idle {
            ctx.put_ann(self.lastbit, ctx.samplenum(), ANN_BITS, vec![bit.to_string()]);
        }
        self.lastbit = ctx.samplenum();
        Ok(())
    }

    fn handle_transition(&mut self, ctx: &mut dyn Context, l: u64, htl: bool) -> Result<()> {
        self.deq.push_back(l as i64);
        while self.deq.len() > 2 {
            self.deq.pop_front();
        }
        let n = self.deq.len();
        if n >= 2
            && (self.in_tolerance((self.deq[n - 1] + self.deq[n - 2]) as f64)
                || htl
                    && self.in_tolerance(l as f64 * 2.0)
                    && (self.deq[n - 2] as f64) > 1.25 * self.bit_width)
        {
            self.add_bit(ctx, 1)?;
            self.deq.clear();
        } else if self.in_tolerance(l as f64) {
            self.add_bit(ctx, 0)?;
            self.deq.clear();
        } else if (l as f64) > 1.25 * self.bit_width {
            self.state = State::Idle;
            self.bytesi.clear();
            self.packet.clear();
            self.bits.clear();
            self.bitsi.clear();
        }
        Ok(())
    }

    /// pd.py: LSB first, `reduce(lambda i, v: (i >> 1) | (v << (len(bits) - 1)), bits, 0)`.
    fn bits_to_uint(bits: &[u8]) -> u8 {
        let n = bits.len();
        let mut acc: u32 = 0;
        for &v in bits {
            acc = (acc >> 1) | (u32::from(v) << (n - 1));
        }
        acc as u8
    }
}

impl Decoder for Qi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        self.reset_variables();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        self.bit_width = samplerate as f64 / 2e3;
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }

        let pins = ctx.wait(&[])?;
        let sn = ctx.samplenum();
        self.handle_transition(ctx, sn, !pins.get(QI))?;
        loop {
            let prev = ctx.samplenum();
            let pins = ctx.wait(&[&[Cond::pin(QI, CondKind::Edge)]])?;
            let sn = ctx.samplenum();
            self.handle_transition(ctx, sn - prev, !pins.get(QI))?;
        }
    }
}
