// Counterpart in sigrok:
//   libsigrokdecode/decoders/gpib/pd.py,
//   Copyright (C) 2016 Rudolf Reuter <reuterru@arcor.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the GPIB bus protocol and the decoder's documented
//   classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - The data lines are low active and inverted as a whole:
//     `item = item ^ 0xff`. ATN and EOI are active low as well and are
//     checked with `== 0` before the data byte is translated.
//   - `waitcond` starts as `{9: 'l'}` (DAV low) and becomes `{9: 'f'}`
//     after the first match, so a capture which starts with DAV low is
//     inspected from its first sample. The optional `sample_total`
//     option adds a `skip` group recomputed every iteration as
//     `lsn - self.samplenum - 1`.
//   - The first item is only saved (`first`); output starts with the
//     second item, spanning from the previous DAV event to this one.
//     Only every 16th item resets `itemcount`/`items`; `items` is not
//     used for output.
//   - With ATN active the byte is a command or an address: listener
//     addresses `> 0x1f and < 0x3f` are shown as `chr(dbyte + 0x10)`
//     and talker addresses `> 0x3f and < 0x5f` as `chr(dbyte - 0x10)`,
//     prefixed with `L` and `T`. Without ATN, LF and CR get their names
//     instead of the raw character.

//! GPIB: IEEE-488 data bytes, commands and addresses at DAV edges.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

// Channel indices, in declaration order.
const EOI: usize = 8;
const DAV: usize = 9;
const ATN: usize = 14;

// Annotation classes, as the libsigrokdecode decoder numbers them.
const ANN_ITEMS: usize = 0;
const ANN_GPIB: usize = 1;
const ANN_EOI: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "gpib",
    name: "GPIB",
    longname: "General Purpose Interface Bus",
    desc: "IEEE-488 GPIB / HPIB protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["gpib"],
    tags: &[],
    channels: &[
        ChannelDesc { id: "dio1", name: "DIO1", desc: "Data I/O bit 1" },
        ChannelDesc { id: "dio2", name: "DIO2", desc: "Data I/O bit 2" },
        ChannelDesc { id: "dio3", name: "DIO3", desc: "Data I/O bit 3" },
        ChannelDesc { id: "dio4", name: "DIO4", desc: "Data I/O bit 4" },
        ChannelDesc { id: "dio5", name: "DIO5", desc: "Data I/O bit 5" },
        ChannelDesc { id: "dio6", name: "DIO6", desc: "Data I/O bit 6" },
        ChannelDesc { id: "dio7", name: "DIO7", desc: "Data I/O bit 7" },
        ChannelDesc { id: "dio8", name: "DIO8", desc: "Data I/O bit 8" },
        ChannelDesc { id: "eoi", name: "EOI", desc: "End or identify" },
        ChannelDesc { id: "dav", name: "DAV", desc: "Data valid" },
        ChannelDesc { id: "nrfd", name: "NRFD", desc: "Not ready for data" },
        ChannelDesc { id: "ndac", name: "NDAC", desc: "Not data accepted" },
        ChannelDesc { id: "ifc", name: "IFC", desc: "Interface clear" },
        ChannelDesc { id: "srq", name: "SRQ", desc: "Service request" },
        ChannelDesc { id: "atn", name: "ATN", desc: "Attention" },
        ChannelDesc { id: "ren", name: "REN", desc: "Remote enable" },
    ],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "sample_total",
        desc: "Total number of samples",
        default: OptionValue::i(0),
        values: &[],
    }],
    annotations: &[
        AnnClass { id: "items", desc: "Items" },
        AnnClass { id: "gpib", desc: "DAT/CMD" },
        AnnClass { id: "eoi", desc: "EOI" },
    ],
    annotation_rows: &[
        AnnRow { id: "bytes", desc: "Bytes", classes: &[ANN_ITEMS] },
        AnnRow { id: "gpib", desc: "DAT/CMD", classes: &[ANN_GPIB] },
        AnnRow { id: "eoi", desc: "EOI", classes: &[ANN_EOI] },
    ],
    binary: &[],
    doc: "",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Gpib::default())
}

/// The GPIB decoder.
pub struct Gpib {
    /// The `sample_total` option; 0 disables the extra wait group.
    sample_total: i64,
    items: Vec<u32>,
    itemcount: u32,
    saved_item: u32,
    saved_atn: bool,
    saved_eoi: bool,
    ss_item: u64,
    first: bool,
}

impl Default for Gpib {
    fn default() -> Self {
        Gpib {
            sample_total: 0,
            items: Vec::new(),
            itemcount: 0,
            saved_item: 0,
            saved_atn: false,
            saved_eoi: false,
            ss_item: 0,
            first: true,
        }
    }
}

impl Gpib {
    /// The GPIB text for a saved byte, exactly as the Python decoder's
    /// nested `if` chain produces it.
    fn gpib_text(dbyte: u32, atn: bool) -> String {
        if atn {
            // ATN, decode commands.
            let command = match dbyte {
                0x01 => Some("GTL"),
                0x04 => Some("SDC"),
                0x05 => Some("PPC"),
                0x08 => Some("GET"),
                0x09 => Some("TCT"),
                0x11 => Some("LLO"),
                0x14 => Some("DCL"),
                0x15 => Some("PPU"),
                0x18 => Some("SPE"),
                0x19 => Some("SPD"),
                0x3f => Some("UNL"),
                0x5f => Some("UNT"),
                _ => None,
            };
            if let Some(command) = command {
                return command.to_owned();
            }
            if dbyte > 0x1f && dbyte < 0x3f {
                // Address Listener.
                return format!("L{}", char::from_u32(dbyte + 0x10).unwrap_or('?'));
            }
            if dbyte > 0x3f && dbyte < 0x5f {
                // Address Talker.
                return format!("T{}", char::from_u32(dbyte - 0x10).unwrap_or('?'));
            }
            " ".to_owned()
        } else {
            if dbyte == 0x0a {
                return "LF".to_owned();
            }
            if dbyte == 0x0d {
                return "CR".to_owned();
            }
            if dbyte > 0x1f && dbyte < 0x7f {
                return char::from_u32(dbyte).unwrap_or('?').to_string();
            }
            " ".to_owned()
        }
    }

    fn handle_bits(&mut self, ctx: &mut dyn Context, datapins: &[u8]) {
        let samplenum = ctx.samplenum();

        // Get the bits for this item.
        let mut item = 0u32;
        for i in 0..8 {
            item |= u32::from(datapins.get(i).copied().unwrap_or(0)) << i;
        }
        item ^= 0xff; // Invert data byte.

        self.items.push(item);
        self.itemcount += 1;

        let item2 = datapins.get(ATN).copied().unwrap_or(1) == 0;
        let item3 = datapins.get(EOI).copied().unwrap_or(1) == 0;

        if self.first {
            // Save the start sample and item for later (no output yet).
            self.ss_item = samplenum;
            self.first = false;
            self.saved_item = item;
            self.saved_atn = item2;
            self.saved_eoi = item3;
        } else {
            // Output the saved item.
            let dbyte = self.saved_item;
            let datn = self.saved_atn;
            let deoi = self.saved_eoi;
            let es_item = samplenum;

            let ss = self.ss_item;
            ctx.put_ann(ss, es_item, ANN_ITEMS, vec![format!("{dbyte:02X}")]);
            ctx.put_ann(ss, es_item, ANN_GPIB, vec![Self::gpib_text(dbyte, datn)]);
            let eoi = if deoi { "EOI" } else { " " };
            ctx.put_ann(ss, es_item, ANN_EOI, vec![eoi.to_owned()]);

            self.ss_item = samplenum;
            self.saved_item = item;
            self.saved_atn = item2;
            self.saved_eoi = item3;
        }

        if self.itemcount < 16 {
            return;
        }

        self.itemcount = 0;
        self.items.clear();
    }
}

impl Decoder for Gpib {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Gpib { sample_total: config.int("sample_total", 0), ..Gpib::default() };
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let lsn = self.sample_total;
        let mut first = true;
        loop {
            let mut groups: Vec<Vec<Cond>> = Vec::with_capacity(2);
            // Inspect samples at the falling edge of DAV, but also start
            // when the capture happens to begin with DAV low.
            groups
                .push(vec![Cond::pin(DAV, if first { CondKind::Low } else { CondKind::Falling })]);
            if lsn != 0 {
                let skip = (lsn - ctx.samplenum() as i64 - 1).max(0) as u64;
                groups.push(vec![Cond::Skip(skip)]);
            }
            let pins = {
                let refs: Vec<&[Cond]> = groups.iter().map(Vec::as_slice).collect();
                ctx.wait(&refs)?
            };
            self.handle_bits(ctx, &pins.0);
            first = false;
        }
    }
}
