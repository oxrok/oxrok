// Counterpart in sigrok:
//   libsigrokdecode/decoders/arm_tpiu/pd.py,
//   Copyright (C) 2015 Petteri Aimonen <jpa@sigrok.mail.kapsi.fi>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the TPIU framing and the decoder's documented
//   classes and texts.

//! ARM TPIU: splits a TPIU formatted trace into its data streams.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

const ANN_STREAM: usize = 0;
const ANN_DATA: usize = 1;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "arm_tpiu",
    name: "ARM TPIU",
    longname: "ARM Trace Port Interface Unit",
    desc: "Filter TPIU formatted trace data into separate streams.",
    license: "gplv2+",
    inputs: &["uart"],
    outputs: &["uart"],
    tags: &["Debug/trace"],
    channels: &[],
    optional_channels: &[],
    options: &[
        OptionDesc { id: "stream", desc: "Stream index", default: OptionValue::i(1), values: &[] },
        OptionDesc {
            id: "sync_offset",
            desc: "Initial sync offset",
            default: OptionValue::i(0),
            values: &[],
        },
    ],
    annotations: &[
        AnnClass { id: "stream", desc: "Current stream" },
        AnnClass { id: "data", desc: "Stream data" },
    ],
    annotation_rows: &[
        AnnRow { id: "stream", desc: "Current stream", classes: &[0] },
        AnnRow { id: "data", desc: "Stream data", classes: &[1] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes the frame format\n\
of ARMv7m Trace Port Interface Unit.\n\
\n\
It filters the data coming from various trace sources (such as ARMv7m ITM\n\
and ETM blocks) into separate streams that can be further decoded by other PDs.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Tpiu::new())
}

/// One byte with its sample range.
#[derive(Clone, Copy)]
struct Byte {
    ss: u64,
    es: u64,
    data: u8,
}

struct Tpiu {
    buf: Vec<Byte>,
    syncbuf: Vec<u8>,
    prevsample: u64,
    byte_len: u64,
    stream: u32,
    ss_stream: u64,
    bytenum: u64,
    stream_opt: u32,
    sync_offset: u64,
}

impl Tpiu {
    fn new() -> Self {
        Tpiu {
            buf: Vec::new(),
            syncbuf: Vec::new(),
            prevsample: 0,
            byte_len: 0,
            stream: 0,
            ss_stream: 0,
            bytenum: 0,
            stream_opt: 1,
            sync_offset: 0,
        }
    }

    /// `stream_changed`: annotates a stream that just ended.
    fn stream_changed(&mut self, ctx: &mut dyn Context, ss: u64, stream: u32) {
        if self.stream != stream {
            if self.stream != 0 {
                ctx.put_ann(
                    self.ss_stream,
                    ss,
                    ANN_STREAM,
                    vec![format!("Stream {}", self.stream), format!("S{}", self.stream)],
                );
            }
            self.stream = stream;
            self.ss_stream = ss;
        }
    }

    /// `emit_byte`: data of the selected stream, as an annotation and as
    /// the UART-shaped `['DATA', 0, (byte, [])]` item.
    fn emit_byte(&self, ctx: &mut dyn Context, ss: u64, es: u64, byte: u8) {
        if self.stream == self.stream_opt {
            ctx.put_ann(ss, es, ANN_DATA, vec![format!("0x{byte:02x}")]);
            ctx.put_python(
                ss,
                es,
                Item::List(vec![
                    Item::Str("DATA".into()),
                    Item::Int(0),
                    Item::List(vec![Item::Int(i64::from(byte)), Item::List(Vec::new())]),
                ]),
            );
        }
    }

    /// `process_frame`: byte 15 carries the low bits of bytes 0, 2, ... 14.
    fn process_frame(&mut self, ctx: &mut dyn Context, buf: &[Byte]) {
        let lowbits = buf[15].data;

        for i in (0..15).step_by(2) {
            // Odd bytes can be stream ID or data.
            let mut delayed_stream_change = None;
            let lowbit = (lowbits >> (i / 2)) & 0x01;
            if buf[i].data & 0x01 != 0 {
                if lowbit != 0 {
                    delayed_stream_change = Some(u32::from(buf[i].data >> 1));
                } else {
                    self.stream_changed(ctx, buf[i].ss, u32::from(buf[i].data >> 1));
                }
            } else {
                let byte = buf[i].data | lowbit;
                self.emit_byte(ctx, buf[i].ss, buf[i].es, byte);
            }

            // Even bytes are always data.
            if i < 14 {
                self.emit_byte(ctx, buf[i + 1].ss, buf[i + 1].es, buf[i + 1].data);
            }

            // The stream change can be delayed to occur after the data byte.
            if let Some(stream) = delayed_stream_change {
                self.stream_changed(ctx, buf[i + 1].es, stream);
            }
        }
    }
}

impl Decoder for Tpiu {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Tpiu {
            stream_opt: config.int("stream", 1) as u32,
            sync_offset: config.int("sync_offset", 0).max(0) as u64,
            ..Tpiu::new()
        };
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("arm_tpiu is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let (ptype, pdata) = unpack_uart(item)?;
        if ptype != "DATA" {
            return Ok(());
        }
        let byte = data_value(pdata)?;

        // Reset packet if there is a long pause between bytes.
        self.byte_len = es.saturating_sub(ss);
        if ss > self.prevsample && ss - self.prevsample > self.byte_len {
            self.buf.clear();
        }
        self.prevsample = es;

        self.buf.push(Byte { ss, es, data: byte });
        self.bytenum += 1;

        // Allow skipping N first bytes of the data.
        if self.bytenum < self.sync_offset {
            self.buf.clear();
            return Ok(());
        }

        // Keep a separate buffer for detection of sync packets; they
        // override everything else, so we can regain sync.
        self.syncbuf = self.syncbuf.iter().rev().take(3).rev().copied().collect();
        self.syncbuf.push(byte);
        if self.syncbuf == [0xFF, 0xFF, 0xFF, 0x7F] {
            self.buf.clear();
            self.syncbuf.clear();
            return Ok(());
        }

        if self.buf.len() == 16 {
            let buf = std::mem::take(&mut self.buf);
            self.process_frame(ctx, &buf);
        }
        Ok(())
    }
}

/// Unpacks `[ptype, rxtx, pdata]`; the UART output tuple.
fn unpack_uart(item: &Item) -> Result<(&str, &Item)> {
    let Item::List(items) = item else {
        return Err(Error::arg("cannot unpack non-sequence UART item"));
    };
    if items.len() != 3 {
        return Err(Error::arg("too many values to unpack (UART item)"));
    }
    let Item::Str(ptype) = &items[0] else {
        return Err(Error::arg("UART packet type is not a string"));
    };
    Ok((ptype, &items[2]))
}

/// `pdata[0]`: the byte value of a DATA item.
fn data_value(pdata: &Item) -> Result<u8> {
    let Item::List(items) = pdata else {
        return Err(Error::arg("UART data is not a sequence"));
    };
    match items.first() {
        Some(Item::Int(v)) => Ok(*v as u8),
        Some(Item::BigUint(bytes)) => Ok(*bytes.last().unwrap_or(&0)),
        _ => Err(Error::arg("UART data has no byte value")),
    }
}
