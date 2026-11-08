// Counterpart in sigrok:
//   libsigrokdecode/decoders/usb_packet/pd.py,
//   Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>,
//   Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the USB 2.0 packet layer and the decoder's
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The packet is rebuilt from the signalling bits as a bit string;
//     fields are slices of that string, and the SYNC/PID/etc annotations
//     span the bits of the field, while the whole packet summary spans
//     SOP to EOP.
//   - The SYNC check is against the literal '00000001', and the PID is
//     looked up by its 8 bit string, so an unknown PID is 'UNKNOWN' and
//     lands in annotation class 28.
//   - `bitstr_to_num()` reverses the bit string before converting, and
//     both CRCs are bit reversed after the shift register.
//   - A DATA packet's DATABYTES item is emitted even when there are no
//     data bytes, and a packet shorter than 8/16/32 bits only emits the
//     "Invalid packet" annotation for the whole packet.
//   - The packet item is a list `['PACKET', [category, name, fields]]`,
//     the last field list being built as the fields are decoded.

//! USB (low-speed/full-speed) packet layer over the signalling decoder.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "signalling",
    desc: "Signalling",
    default: OptionValue::s("full-speed"),
    values: &[OptionValue::s("full-speed"), OptionValue::s("low-speed")],
}];

/// Annotation classes, in the order of pd.py's `annotations`.
static ANNOTATIONS: [AnnClass; 29] = [
    AnnClass { id: "sync-ok", desc: "SYNC" },
    AnnClass { id: "sync-err", desc: "SYNC (error)" },
    AnnClass { id: "pid", desc: "PID" },
    AnnClass { id: "framenum", desc: "FRAMENUM" },
    AnnClass { id: "addr", desc: "ADDR" },
    AnnClass { id: "ep", desc: "EP" },
    AnnClass { id: "crc5-ok", desc: "CRC5" },
    AnnClass { id: "crc5-err", desc: "CRC5 (error)" },
    AnnClass { id: "data", desc: "DATA" },
    AnnClass { id: "crc16-ok", desc: "CRC16" },
    AnnClass { id: "crc16-err", desc: "CRC16 (error)" },
    AnnClass { id: "packet-out", desc: "Packet: OUT" },
    AnnClass { id: "packet-in", desc: "Packet: IN" },
    AnnClass { id: "packet-sof", desc: "Packet: SOF" },
    AnnClass { id: "packet-setup", desc: "Packet: SETUP" },
    AnnClass { id: "packet-data0", desc: "Packet: DATA0" },
    AnnClass { id: "packet-data1", desc: "Packet: DATA1" },
    AnnClass { id: "packet-data2", desc: "Packet: DATA2" },
    AnnClass { id: "packet-mdata", desc: "Packet: MDATA" },
    AnnClass { id: "packet-ack", desc: "Packet: ACK" },
    AnnClass { id: "packet-nak", desc: "Packet: NAK" },
    AnnClass { id: "packet-stall", desc: "Packet: STALL" },
    AnnClass { id: "packet-nyet", desc: "Packet: NYET" },
    AnnClass { id: "packet-pre", desc: "Packet: PRE" },
    AnnClass { id: "packet-err", desc: "Packet: ERR" },
    AnnClass { id: "packet-split", desc: "Packet: SPLIT" },
    AnnClass { id: "packet-ping", desc: "Packet: PING" },
    AnnClass { id: "packet-reserved", desc: "Packet: Reserved" },
    AnnClass { id: "packet-invalid", desc: "Packet: Invalid" },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "usb_packet",
    name: "USB packet",
    longname: "Universal Serial Bus (LS/FS) packet",
    desc: "USB (low-speed and full-speed) packet protocol.",
    license: "gplv2+",
    inputs: &["usb_signalling"],
    outputs: &["usb_packet"],
    tags: &["PC"],
    channels: &[],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &ANNOTATIONS,
    annotation_rows: &[
        AnnRow {
            id: "fields",
            desc: "Packet fields",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        },
        AnnRow {
            id: "packet",
            desc: "Packets",
            classes: &[11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28],
        },
    ],
    binary: &[],
    // The package's __init__.py docstring, which the Python host reads.
    doc: "This decoder stacks on top of the 'usb_signalling' PD and decodes the USB\n\
(low-speed and full-speed) packet protocol.\n\
\n\
Protocol layer (USB spec, chapter 8):\n\
\n\
Bit/byte ordering: Bits are sent onto the bus LSB-first. Multibyte fields\n\
are transmitted in little-endian order (i.e., LSB to MSB).\n\
\n\
SYNC field: All packets begin with a SYNC field (8 bits).\n\
\n\
Packet field format: Packets start with an SOP (Start Of Packet) delimiter\n\
that is part of the SYNC field, and end with an EOP (End Of Packet).\n\
\n\
PID: A PID (packet identifier) follows the SYNC field of every packet. A PID\n\
consists of a 4-bit packet type field, and a 4 bit check field.\n\
The check field is the one's complement of the packet type field.\n\
\n\
Details:\n\
https://en.wikipedia.org/wiki/USB\n\
http://www.usb.org/developers/docs/",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(UsbPacket::default())
}

/// `pids` in pd.py, by bit string.
fn pid_name(pid: &str) -> &'static str {
    match pid {
        "10000111" => "OUT",
        "10010110" => "IN",
        "10100101" => "SOF",
        "10110100" => "SETUP",
        "11000011" => "DATA0",
        "11010010" => "DATA1",
        "11100001" => "DATA2",
        "11110000" => "MDATA",
        "01001011" => "ACK",
        "01011010" => "NAK",
        "01111000" => "STALL",
        "01101001" => "NYET",
        "00111100" => "PRE",
        "00011110" => "SPLIT",
        "00101101" => "PING",
        "00001111" => "Reserved",
        _ => "UNKNOWN",
    }
}

/// pd.py `get_category()`.
fn category(pidname: &str) -> &'static str {
    match pidname {
        "OUT" | "IN" | "SOF" | "SETUP" => "TOKEN",
        "DATA0" | "DATA1" | "DATA2" | "MDATA" => "DATA",
        "ACK" | "NAK" | "STALL" | "NYET" => "HANDSHAKE",
        _ => "SPECIAL",
    }
}

/// pd.py `ann_index()`.
fn ann_index(pidname: &str) -> usize {
    let names = [
        "OUT", "IN", "SOF", "SETUP", "DATA0", "DATA1", "DATA2", "MDATA", "ACK", "NAK", "STALL",
        "NYET", "PRE", "ERR", "SPLIT", "PING", "Reserved",
    ];
    names.iter().position(|n| *n == pidname).map_or(28, |i| i + 11)
}

/// pd.py `bitstr_to_num()`: the string is LSB first.
fn bitstr_to_num(bitstr: &str) -> i64 {
    let mut value = 0i64;
    for (i, c) in bitstr.chars().enumerate() {
        if c == '1' {
            value |= 1 << i;
        }
    }
    value
}

/// pd.py `reverse_number()`.
fn reverse_number(num: u32, count: u32) -> u32 {
    let mut out = 0u32;
    for i in 0..count {
        if num >> i & 1 == 1 {
            out |= 1 << (count - 1 - i);
        }
    }
    out
}

/// pd.py `calc_crc5()`.
fn calc_crc5(bitstr: &str) -> i64 {
    let poly5 = 0x25u32;
    let mut crc5 = 0x1fu32;
    for c in bitstr.chars() {
        crc5 <<= 1;
        let bit = u32::from(c == '1');
        if bit != (crc5 >> 5) {
            crc5 ^= poly5;
        }
        crc5 &= 0x1f;
    }
    crc5 ^= 0x1f;
    i64::from(reverse_number(crc5, 5))
}

/// pd.py `calc_crc16()`.
fn calc_crc16(bitstr: &str) -> i64 {
    let poly16 = 0x18005u32;
    let mut crc16 = 0xffffu32;
    for c in bitstr.chars() {
        crc16 <<= 1;
        let bit = u32::from(c == '1');
        if bit != (crc16 >> 16) {
            crc16 ^= poly16;
        }
        crc16 &= 0xffff;
    }
    crc16 ^= 0xffff;
    i64::from(reverse_number(crc16, 16))
}

/// One signalling bit.
#[derive(Clone, Copy)]
struct Bit {
    value: char,
    ss: u64,
    es: u64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    WaitForSop,
    GetBit,
}

/// The usb_packet decoder.
#[derive(Default)]
pub struct UsbPacket {
    bits: Vec<Bit>,
    packet: Vec<Item>,
    packet_summary: String,
    ss: u64,
    es: u64,
    ss_packet: u64,
    es_packet: u64,
    state: State,
}

impl Default for State {
    fn default() -> Self {
        State::WaitForSop
    }
}

fn as_text(item: &Item) -> Result<&str> {
    match item {
        Item::Str(s) => Ok(s.as_str()),
        _ => Err(Error::device("usb_packet: expected a string")),
    }
}

impl UsbPacket {
    fn putpb(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.ss, self.es, item);
    }

    fn putb(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss, self.es, class, texts);
    }

    fn putp(&self, ctx: &mut dyn Context, class: usize, text: String) {
        ctx.put_ann(self.ss_packet, self.es_packet, class, vec![text]);
    }

    fn putpp(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.ss_packet, self.es_packet, item);
    }

    fn handle_packet(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let packet: String = self.bits.iter().map(|b| b.value).collect();

        if packet.len() < 8 {
            self.putp(ctx, 28, "Invalid packet (shorter than 8 bits)".to_owned());
            return Ok(());
        }

        // Bits[0:7]: SYNC.
        let sync = &packet[..8];
        self.ss = self.bits[0].ss;
        self.es = self.bits[7].es;
        if sync != "00000001" {
            self.putpb(
                ctx,
                Item::List(vec![Item::Str("SYNC ERROR".into()), Item::Str(sync.into())]),
            );
            self.putb(
                ctx,
                1,
                vec![
                    format!("SYNC ERROR: {sync}"),
                    format!("SYNC ERR: {sync}"),
                    "SYNC ERR".to_owned(),
                    "SE".to_owned(),
                    "S".to_owned(),
                ],
            );
        } else {
            self.putpb(ctx, Item::List(vec![Item::Str("SYNC".into()), Item::Str(sync.into())]));
            self.putb(ctx, 0, vec![format!("SYNC: {sync}"), "SYNC".to_owned(), "S".to_owned()]);
        }
        self.packet.push(Item::Str(sync.into()));

        if packet.len() < 16 {
            self.putp(ctx, 28, "Invalid packet (shorter than 16 bits)".to_owned());
            return Ok(());
        }

        // Bits[8:15]: PID.
        let pid = &packet[8..16];
        let pidname = pid_name(pid);
        self.ss = self.bits[8].ss;
        self.es = self.bits[15].es;
        self.putpb(ctx, Item::List(vec![Item::Str("PID".into()), Item::Str(pidname.into())]));
        self.putb(
            ctx,
            2,
            vec![
                format!("PID: {pidname}"),
                pidname.to_owned(),
                pidname.chars().next().unwrap_or('U').to_string(),
            ],
        );
        self.packet.push(Item::Str(pid.into()));
        self.packet_summary.push_str(pidname);

        if matches!(pidname, "OUT" | "IN" | "SOF" | "SETUP" | "PING") {
            if packet.len() < 32 {
                self.putp(ctx, 28, "Invalid packet (shorter than 32 bits)".to_owned());
                return Ok(());
            }

            if pidname == "SOF" {
                // Bits[16:26]: Framenum.
                let framenum = bitstr_to_num(&packet[16..27]);
                self.ss = self.bits[16].ss;
                self.es = self.bits[26].es;
                self.putpb(
                    ctx,
                    Item::List(vec![Item::Str("FRAMENUM".into()), Item::Int(framenum)]),
                );
                self.putb(
                    ctx,
                    3,
                    vec![
                        format!("Frame: {framenum}"),
                        "Frame".to_owned(),
                        "Fr".to_owned(),
                        "F".to_owned(),
                    ],
                );
                self.packet.push(Item::Int(framenum));
                self.packet_summary.push_str(&format!(" {framenum}"));
            } else {
                // Bits[16:22]: Addr.
                let addr = bitstr_to_num(&packet[16..23]);
                self.ss = self.bits[16].ss;
                self.es = self.bits[22].es;
                self.putpb(ctx, Item::List(vec![Item::Str("ADDR".into()), Item::Int(addr)]));
                self.putb(
                    ctx,
                    4,
                    vec![
                        format!("Address: {addr}"),
                        format!("Addr: {addr}"),
                        "Addr".to_owned(),
                        "A".to_owned(),
                    ],
                );
                self.packet.push(Item::Int(addr));
                self.packet_summary.push_str(&format!(" ADDR {addr}"));

                // Bits[23:26]: EP.
                let ep = bitstr_to_num(&packet[23..27]);
                self.ss = self.bits[23].ss;
                self.es = self.bits[26].es;
                self.putpb(ctx, Item::List(vec![Item::Str("EP".into()), Item::Int(ep)]));
                self.putb(
                    ctx,
                    5,
                    vec![
                        format!("Endpoint: {ep}"),
                        format!("EP: {ep}"),
                        "EP".to_owned(),
                        "E".to_owned(),
                    ],
                );
                self.packet.push(Item::Int(ep));
                self.packet_summary.push_str(&format!(" EP {ep}"));
            }

            // Bits[27:31]: CRC5.
            let crc5 = bitstr_to_num(&packet[27..32]);
            let crc5_calc = calc_crc5(&packet[16..27]);
            self.ss = self.bits[27].ss;
            self.es = self.bits[31].es;
            if crc5 == crc5_calc {
                self.putpb(ctx, Item::List(vec![Item::Str("CRC5".into()), Item::Int(crc5)]));
                self.putb(
                    ctx,
                    6,
                    vec![format!("CRC5: 0x{crc5:02X}"), "CRC5".to_owned(), "C".to_owned()],
                );
            } else {
                self.putpb(ctx, Item::List(vec![Item::Str("CRC5 ERROR".into()), Item::Int(crc5)]));
                self.putb(
                    ctx,
                    7,
                    vec![
                        format!("CRC5 ERROR: 0x{crc5:02X}"),
                        "CRC5 ERR".to_owned(),
                        "CE".to_owned(),
                        "C".to_owned(),
                    ],
                );
            }
            self.packet.push(Item::Int(crc5));
        } else if matches!(pidname, "DATA0" | "DATA1" | "DATA2" | "MDATA") {
            // Bits[16:packetlen-16]: Data. Python's `packet[16:-16]`
            // is empty, not an error, when the packet is shorter than
            // 32 bits.
            let data_end = packet.len().saturating_sub(16);
            let data = if data_end >= 16 { &packet[16..data_end] } else { "" };
            let mut databytes = Vec::new();
            self.packet_summary.push_str(" [");
            let mut i = 0;
            while i < data.len() {
                let db = bitstr_to_num(&data[i..i + 8]);
                self.ss = self.bits[16 + i].ss;
                self.es = self.bits[23 + i].es;
                self.putpb(ctx, Item::List(vec![Item::Str("DATABYTE".into()), Item::Int(db)]));
                self.putb(
                    ctx,
                    8,
                    vec![
                        format!("Databyte: {db:02X}"),
                        format!("Data: {db:02X}"),
                        format!("DB: {db:02X}"),
                        format!("{db:02X}"),
                    ],
                );
                databytes.push(Item::Int(db));
                self.packet_summary.push_str(&format!(" {db:02X}"));
                i += 8;
            }
            self.packet_summary.push_str(" ]");

            // Convenience Python output (no annotation) for all bytes.
            let Some(first_bit) = self.bits.get(16) else {
                return Err(Error::device("usb_packet: short DATA packet"));
            };
            self.ss = first_bit.ss;
            self.es = self.bits[self.bits.len() - 16].es;
            self.putpb(
                ctx,
                Item::List(vec![Item::Str("DATABYTES".into()), Item::List(databytes.clone())]),
            );
            self.packet.push(Item::List(databytes));

            // Bits[packetlen-16:packetlen]: CRC16. Python's
            // `packet[-16:]` and `packet[16:-16]` slices.
            let crc16_start = packet.len().saturating_sub(16);
            let crc16 = bitstr_to_num(&packet[crc16_start..]);
            let crc16_calc = calc_crc16(data);
            self.ss = self.bits[self.bits.len() - 16].ss;
            self.es = self.bits[self.bits.len() - 1].es;
            if crc16 == crc16_calc {
                self.putpb(ctx, Item::List(vec![Item::Str("CRC16".into()), Item::Int(crc16)]));
                self.putb(
                    ctx,
                    9,
                    vec![format!("CRC16: 0x{crc16:04X}"), "CRC16".to_owned(), "C".to_owned()],
                );
            } else {
                self.putpb(
                    ctx,
                    Item::List(vec![Item::Str("CRC16 ERROR".into()), Item::Int(crc16)]),
                );
                self.putb(
                    ctx,
                    10,
                    vec![
                        format!("CRC16 ERROR: 0x{crc16:04X}"),
                        "CRC16 ERR".to_owned(),
                        "CE".to_owned(),
                        "C".to_owned(),
                    ],
                );
            }
            self.packet.push(Item::Int(crc16));
        } else {
            // ACK/NAK/STALL/NYET/ERR, PRE, SPLIT and Reserved only have
            // the fields decoded above (or none).
        }

        // Output a (summary of) the whole packet.
        let packet_item = Item::List(vec![
            Item::Str("PACKET".into()),
            Item::List(vec![
                Item::Str(category(pidname).into()),
                Item::Str(pidname.into()),
                Item::List(self.packet.clone()),
            ]),
        ]);
        self.putpp(ctx, packet_item);
        self.putp(ctx, ann_index(pidname), self.packet_summary.clone());

        self.packet.clear();
        self.packet_summary.clear();
        Ok(())
    }
}

impl Decoder for UsbPacket {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("usb_packet takes a usb_signalling decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Item::List(parts) = item else {
            return Err(Error::device("usb_packet: expected a two element packet"));
        };
        if parts.len() != 2 {
            return Err(Error::device("usb_packet: expected a two element packet"));
        }
        let ptype = as_text(&parts[0])?;

        if self.state == State::WaitForSop {
            if ptype != "SOP" {
                return Ok(());
            }
            self.ss_packet = ss;
            self.state = State::GetBit;
            return Ok(());
        }

        match ptype {
            "BIT" => {
                let value = as_text(&parts[1])?;
                self.bits.push(Bit { value: value.chars().next().unwrap_or('0'), ss, es });
            }
            "EOP" | "ERR" => {
                self.es_packet = es;
                self.handle_packet(ctx)?;
                self.packet.clear();
                self.packet_summary.clear();
                self.bits.clear();
                self.state = State::WaitForSop;
            }
            _ => {}
        }
        Ok(())
    }
}
