// Counterpart in sigrok:
//   libsigrokdecode/decoders/microwire/pd.py,
//   Copyright (C) 2017 Kevin Redon <kingkevin@cuvoodoo.info>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the Microwire bus protocol and the decoder's
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - After the rising CS edge every change is saved together with the
//     `matched` groups of the wait that produced it, because the
//     decoder cannot tell a status check from bit communication before
//     the packet has ended.
//   - If SK is high on CS start it is taken as low for the clock edge
//     choice, and a warning is emitted.
//   - A status check is assumed when the first rising SK edge has SI
//     low (or there is no rising SK edge at all). SO low means busy,
//     SO high ready; the SO edge spans are annotated as Busy.
//   - Otherwise SI is sampled on rising SK edges and SO on falling
//     edges; the first bit is the start bit and the rest are the SI/SO
//     bits of one Python packet.
//   - The Python output starts with the sample where the first change
//     was saved, not with the CS edge itself.

//! Microwire: 3-wire half-duplex synchronous serial bus.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item,
};

// Channel indices, in declaration order.
const CS: usize = 0;
const SK: usize = 1;
const SI: usize = 2;
const SO: usize = 3;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_START_BIT: usize = 0;
const ANN_SI_BIT: usize = 1;
const ANN_SO_BIT: usize = 2;
const ANN_STATUS_READY: usize = 3;
const ANN_STATUS_BUSY: usize = 4;
const ANN_WARNING: usize = 5;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "microwire",
    name: "Microwire",
    longname: "Microwire",
    desc: "3-wire, half-duplex, synchronous serial bus.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["microwire"],
    tags: &["Embedded/industrial"],
    channels: &[
        ChannelDesc { id: "cs", name: "CS", desc: "Chip select" },
        ChannelDesc { id: "sk", name: "SK", desc: "Clock" },
        ChannelDesc { id: "si", name: "SI", desc: "Slave in" },
        ChannelDesc { id: "so", name: "SO", desc: "Slave out" },
    ],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "start-bit", desc: "Start bit" },
        AnnClass { id: "si-bit", desc: "SI bit" },
        AnnClass { id: "so-bit", desc: "SO bit" },
        AnnClass { id: "status-check-ready", desc: "Status check ready" },
        AnnClass { id: "status-check-busy", desc: "Status check busy" },
        AnnClass { id: "warning", desc: "Warning" },
    ],
    annotation_rows: &[
        AnnRow { id: "si-bits", desc: "SI bits", classes: &[0, 1] },
        AnnRow { id: "so-bits", desc: "SO bits", classes: &[2] },
        AnnRow { id: "status", desc: "Status", classes: &[3, 4] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[5] },
    ],
    binary: &[],
    doc: "Microwire is a 3-wire half-duplex synchronous serial communication \
          protocol.\n\
          \n\
          Originally from National Semiconductor, it is often used in EEPROM \
          chips with device specific commands on top of the bit stream.\n\
          \n\
          Channels:\n\
          \n\
           - CS: chip select, active high\n\
           - SK: clock line, for the synchronous communication (idle low)\n\
           - SI: slave data input, output by the master and parsed by the \
          selected slave on rising edge of clock line (idle low)\n\
           - SO: slave data output, output by the selected slave and changed on \
          rising edge of clock line, or goes from low to high when ready during \
          status check (idle high impedance)\n\
          \n\
          The channel names might vary from chip to chip but the underlying \
          function is the same.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Microwire::new())
}

/// pd.py `Packet`: the levels and matched groups at one change.
#[allow(clippy::struct_excessive_bools)]
struct Packet {
    samplenum: u64,
    matched: Vec<bool>,
    cs: bool,
    sk: bool,
    si: bool,
    so: bool,
}

struct Microwire;

impl Microwire {
    fn new() -> Microwire {
        Microwire
    }
}

impl Decoder for Microwire {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        loop {
            // Wait for slave to be selected on rising CS.
            let pins = ctx.wait(&[&[Cond::pin(CS, CondKind::Rising)]])?;
            let mut cs = pins.0[CS] == 1;
            let mut sk = pins.0[SK] == 1;
            let mut si = pins.0[SI] == 1;
            let mut so = pins.0[SO] == 1;
            if sk {
                ctx.put_ann(
                    ctx.samplenum(),
                    ctx.samplenum(),
                    ANN_WARNING,
                    vec![
                        "Clock should be low on start".into(),
                        "Clock high on start".into(),
                        "Clock high".into(),
                        "SK high".into(),
                    ],
                );
                // Enforce correct state for correct clock handling.
                sk = false;
            }
            let mut packet: Vec<Packet> = Vec::new();
            while cs {
                // Save change.
                packet.push(Packet {
                    samplenum: ctx.samplenum(),
                    matched: ctx.matched().to_vec(),
                    cs,
                    sk,
                    si,
                    so,
                });
                let edge = if sk { CondKind::Falling } else { CondKind::Rising };
                let cs_low = [Cond::pin(CS, CondKind::Low)];
                let sk_change = [Cond::pin(SK, edge)];
                let so_change = [Cond::pin(SO, CondKind::Edge)];
                let pins = ctx.wait(&[&cs_low, &sk_change, &so_change])?;
                cs = pins.0[CS] == 1;
                sk = pins.0[SK] == 1;
                si = pins.0[SI] == 1;
                so = pins.0[SO] == 1;
            }
            // Save last change.
            packet.push(Packet {
                samplenum: ctx.samplenum(),
                matched: ctx.matched().to_vec(),
                cs,
                sk,
                si,
                so,
            });

            // Figure out if this is a status check.
            // Either there is no clock or no start bit (on first rising edge).
            let mut status_check = true;
            for change in &packet {
                // Get first clock rising edge.
                if change.matched.len() > 1 && change.matched[1] && change.sk {
                    if change.si {
                        status_check = false;
                    }
                    break;
                }
            }

            if status_check {
                // The packet is for a status check.
                // SO low = busy, SO high = ready.
                let mut start_samplenum = packet[0].samplenum;
                let mut bit_so = packet[0].so;
                // Check for SO edges.
                for change in &packet {
                    if change.matched.len() > 2 && change.matched[2] {
                        if !bit_so && change.so {
                            // Rising edge Busy -> Ready.
                            ctx.put_ann(
                                start_samplenum,
                                change.samplenum,
                                ANN_STATUS_BUSY,
                                vec!["Busy".into(), "B".into()],
                            );
                        }
                        start_samplenum = change.samplenum;
                        bit_so = change.so;
                    }
                }
                // Put last state.
                let last = packet[packet.len() - 1].samplenum;
                if bit_so {
                    ctx.put_ann(
                        start_samplenum,
                        last,
                        ANN_STATUS_READY,
                        vec!["Ready".into(), "R".into()],
                    );
                } else {
                    ctx.put_ann(
                        start_samplenum,
                        last,
                        ANN_STATUS_BUSY,
                        vec!["Busy".into(), "B".into()],
                    );
                }
            } else {
                // Bit communication.
                // Since the slave samples SI on clock rising edge we do the
                // same. Because the slave changes SO on clock rising edge we
                // sample on the falling edge.
                let mut bit_start = 0u64; // Rising clock sample of bit start.
                let mut bit_si = false; // SI value at rising clock edge.
                let mut bit_so = false; // SO value at falling clock edge.
                let mut start_bit = true; // Start bit incoming (first bit).
                let mut pydata: Vec<Item> = Vec::new();
                for change in &packet {
                    if change.matched.len() > 1 && change.matched[1] {
                        // Clock edge.
                        if change.sk {
                            // Rising clock edge.
                            if bit_start > 0 {
                                // Bit completed.
                                if start_bit {
                                    if bit_si {
                                        ctx.put_ann(
                                            bit_start,
                                            change.samplenum,
                                            ANN_START_BIT,
                                            vec!["Start bit".into(), "S".into()],
                                        );
                                    } else {
                                        // Start bit missing.
                                        ctx.put_ann(
                                            bit_start,
                                            change.samplenum,
                                            ANN_WARNING,
                                            vec![
                                                "Start bit not high".into(),
                                                "Start bit low".into(),
                                            ],
                                        );
                                    }
                                    start_bit = false;
                                } else {
                                    let si_text = u8::from(bit_si);
                                    let so_text = u8::from(bit_so);
                                    ctx.put_ann(
                                        bit_start,
                                        change.samplenum,
                                        ANN_SI_BIT,
                                        vec![
                                            format!("SI bit: {si_text}"),
                                            format!("SI: {si_text}"),
                                            si_text.to_string(),
                                        ],
                                    );
                                    ctx.put_ann(
                                        bit_start,
                                        change.samplenum,
                                        ANN_SO_BIT,
                                        vec![
                                            format!("SO bit: {so_text}"),
                                            format!("SO: {so_text}"),
                                            so_text.to_string(),
                                        ],
                                    );
                                    pydata.push(Item::List(vec![
                                        Item::Int(bit_start as i64),
                                        Item::Int(change.samplenum as i64),
                                        Item::Int(i64::from(si_text)),
                                        Item::Int(i64::from(so_text)),
                                    ]));
                                }
                            }
                            bit_start = change.samplenum;
                            bit_si = change.si;
                        } else {
                            // Falling clock edge.
                            bit_so = change.so;
                        }
                    } else if change.matched[0] && !change.cs && !change.sk {
                        // End of packet.
                        let si_text = u8::from(bit_si);
                        let so_text = u8::from(bit_so);
                        ctx.put_ann(
                            bit_start,
                            change.samplenum,
                            ANN_SI_BIT,
                            vec![
                                format!("SI bit: {si_text}"),
                                format!("SI: {si_text}"),
                                si_text.to_string(),
                            ],
                        );
                        ctx.put_ann(
                            bit_start,
                            change.samplenum,
                            ANN_SO_BIT,
                            vec![
                                format!("SO bit: {so_text}"),
                                format!("SO: {so_text}"),
                                so_text.to_string(),
                            ],
                        );
                        pydata.push(Item::List(vec![
                            Item::Int(bit_start as i64),
                            Item::Int(change.samplenum as i64),
                            Item::Int(i64::from(si_text)),
                            Item::Int(i64::from(so_text)),
                        ]));
                    }
                }
                ctx.put_python(
                    packet[0].samplenum,
                    packet[packet.len() - 1].samplenum,
                    Item::List(pydata),
                );
            }
        }
    }
}
