// Counterpart in sigrok:
//   libsigrokdecode/decoders/lin/pd.py,
//   Copyright (C) 2018 Stephan Thiele <stephan.thiele@mailbox.org>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the LIN framing and the decoder's documented
//   classes and texts.

//! LIN: break, sync, protected identifier, data bytes and checksum.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item, OptionDesc,
    OptionValue,
};

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_DATA: usize = 0;
const ANN_CONTROL: usize = 1;
const ANN_ERROR: usize = 2;
const ANN_INLINE_ERROR: usize = 3;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "lin",
    name: "LIN",
    longname: "Local Interconnect Network",
    desc: "Local Interconnect Network (LIN) protocol.",
    license: "gplv2+",
    inputs: &["uart"],
    outputs: &[],
    tags: &["Automotive"],
    channels: &[],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "version",
        desc: "Protocol version",
        default: OptionValue::i(2),
        values: &[OptionValue::i(1), OptionValue::i(2)],
    }],
    annotations: &[
        AnnClass { id: "data", desc: "LIN data" },
        AnnClass { id: "control", desc: "Protocol info" },
        AnnClass { id: "error", desc: "Error descriptions" },
        AnnClass { id: "inline_error", desc: "Protocol violations and errors" },
    ],
    annotation_rows: &[
        AnnRow { id: "data", desc: "Data", classes: &[0, 1, 3] },
        AnnRow { id: "error", desc: "Error", classes: &[2] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes the LIN\n\
(Local Interconnect Network) protocol.\n\
\n\
LIN is layered on top of the UART (async serial) protocol, with 8n1 settings.\n\
Bytes are sent LSB-first.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Lin::new())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    WaitForBreak,
    Sync,
    Pid,
    Data,
    Checksum,
    Error,
}

impl State {
    /// `LinFsm._transition_allowed`: `Error` may always be entered.
    fn allowed(self, target: State) -> bool {
        if target == State::Error {
            return true;
        }
        matches!(
            (self, target),
            (State::WaitForBreak, State::Sync)
                | (State::Sync, State::Pid)
                | (State::Pid, State::Data)
                | (State::Data, State::Data)
                | (State::Data, State::Checksum)
                | (State::Checksum, State::WaitForBreak)
                | (State::Error, State::Sync)
        )
    }
}

/// A `(ss, es, value)` triple.
type Byte = (u64, u64, u8);

/// The LIN decoder.
pub struct Lin {
    state: State,
    uart_idle_count: u32,
    lin_header: Vec<Byte>,
    lin_rsp: Vec<Byte>,
    lin_version: i64,
    ss_block: u64,
    es_block: u64,
}

impl Lin {
    fn new() -> Self {
        Lin {
            state: State::WaitForBreak,
            uart_idle_count: 0,
            lin_header: Vec::new(),
            lin_rsp: Vec::new(),
            lin_version: 2,
            ss_block: 0,
            es_block: 0,
        }
    }

    fn reset_fsm(&mut self) {
        self.state = State::WaitForBreak;
        self.uart_idle_count = 0;
    }

    fn transit(&mut self, target: State) -> bool {
        if !self.state.allowed(target) {
            return false;
        }
        self.state = target;
        true
    }

    /// `putx`: the stored packet start and end.
    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        ctx.put_ann(
            self.ss_block,
            self.es_block,
            class,
            texts.iter().map(|t| (*t).to_owned()).collect(),
        );
    }

    /// Feeds one UART item to the state machine, as upstream's `decode`.
    fn handle_item(&mut self, ctx: &mut dyn Context, item: &Item) -> Result<()> {
        let (ptype, pdata) = unpack(item)?;
        // Ignore all UART packets except the actual data packets or BREAK.
        if ptype == "IDLE" {
            self.handle_uart_idle(ctx)?;
        }
        if ptype == "BREAK" {
            self.handle_break(ctx, int_value(pdata)?)?;
        }
        if ptype != "DATA" {
            return Ok(());
        }
        // We're only interested in the byte value (not individual bits).
        let value = data_value(pdata)?;
        match self.state {
            State::WaitForBreak => self.handle_wait_for_break(ctx, value),
            State::Sync => {
                // `self.fsm.transit(LinFsm.State.Pid)`
                self.transit(State::Pid);
                self.lin_header.push((self.ss_block, self.es_block, value as u8));
                Ok(())
            }
            State::Pid => {
                self.transit(State::Data);
                self.lin_header.push((self.ss_block, self.es_block, value as u8));
                Ok(())
            }
            State::Data => {
                self.lin_rsp.push((self.ss_block, self.es_block, value as u8));
                Ok(())
            }
            // `handler(pdata)` with `handle_checksum()`: a stray data byte
            // in this state is a TypeError upstream.
            State::Checksum => {
                Err(Error::arg("handle_checksum() takes 1 positional argument but 2 were given"))
            }
            State::Error => self.handle_error(ctx),
        }
    }

    fn handle_uart_idle(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if !matches!(self.state, State::WaitForBreak | State::Error) {
            self.uart_idle_count += 1;
            if self.uart_idle_count == 2 {
                self.transit(State::Checksum);
                self.handle_checksum(ctx)?;
                self.reset_fsm();
            }
        }
        Ok(())
    }

    fn handle_wait_for_break(&mut self, ctx: &mut dyn Context, value: i64) -> Result<()> {
        self.wipe_break_null_byte(ctx, value)?;
        Ok(())
    }

    fn handle_break(&mut self, ctx: &mut dyn Context, value: i64) -> Result<()> {
        if !matches!(self.state, State::WaitForBreak | State::Error)
            && self.wipe_break_null_byte(ctx, value)?
        {
            self.transit(State::Checksum);
            self.handle_checksum(ctx)?;
        }

        self.reset_fsm();
        self.transit(State::Sync);

        self.putx(ctx, ANN_CONTROL, &["Break condition", "Break", "Brk", "B"]);
        Ok(())
    }

    /// Upon a break condition a null byte is received which must be ignored.
    fn wipe_break_null_byte(&mut self, ctx: &mut dyn Context, value: i64) -> Result<bool> {
        let mut value = value;
        if !matches!(self.state, State::WaitForBreak | State::Error) {
            if !self.lin_rsp.is_empty() {
                value = i64::from(self.lin_rsp.pop().expect("not empty").2);
            } else if self.lin_header.pop().is_none() {
                // `self.lin_header.pop()` on an empty list.
                return Err(Error::arg("pop from empty list"));
            }
        }

        if value != 0 {
            self.transit(State::Error);
            self.handle_error(ctx)?;
            return Ok(false);
        }

        Ok(true)
    }

    fn handle_checksum(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let sync = if self.lin_header.is_empty() { None } else { Some(self.lin_header.remove(0)) };
        // `sync[0]` on None is a TypeError upstream.
        let Some(sync) = sync else {
            return Err(Error::arg("'NoneType' object is not subscriptable"));
        };

        ctx.put_ann(sync.0, sync.1, ANN_DATA, vec!["Sync".into(), "S".into()]);

        if sync.2 != 0x55 {
            ctx.put_ann(
                sync.0,
                sync.1,
                ANN_ERROR,
                vec!["Sync is not 0x55".into(), "Not 0x55".into(), "!= 0x55".into()],
            );
        }

        let pid = if self.lin_header.is_empty() { None } else { Some(self.lin_header.remove(0)) };
        let checksum = self.lin_rsp.pop();

        if let Some(pid) = pid {
            let id = pid.2 & 0x3f;
            let parity = pid.2 >> 6;

            let expected_parity = Self::calc_parity(pid.2);
            let parity_valid = parity == expected_parity;

            if !parity_valid {
                ctx.put_ann(pid.0, pid.1, ANN_ERROR, vec![format!("P != {expected_parity}")]);
            }

            let ann_class = if parity_valid { ANN_DATA } else { ANN_INLINE_ERROR };
            ctx.put_ann(
                pid.0,
                pid.1,
                ann_class,
                vec![
                    format!(
                        "ID: {id:02X} Parity: {parity} ({})",
                        if parity_valid { "ok" } else { "bad" }
                    ),
                    format!("ID: 0x{id:02X}"),
                    format!("I: {id}"),
                ],
            );
        }

        if !self.lin_rsp.is_empty() {
            let Some(pid) = pid else {
                return Err(Error::arg("'NoneType' object is not subscriptable"));
            };
            let Some(checksum) = checksum else {
                return Err(Error::arg("'NoneType' object is not subscriptable"));
            };
            let checksum_valid = self.checksum_is_valid(pid.2, checksum.2);

            for b in &self.lin_rsp {
                ctx.put_ann(
                    b.0,
                    b.1,
                    ANN_DATA,
                    vec![format!("Data: 0x{:02X}", b.2), format!("D: 0x{:02X}", b.2)],
                );
            }

            let ann_class = if checksum_valid { ANN_DATA } else { ANN_INLINE_ERROR };
            ctx.put_ann(
                checksum.0,
                checksum.1,
                ann_class,
                vec![
                    format!("Checksum: 0x{:02X}", checksum.2),
                    "Checksum".into(),
                    "Chk".into(),
                    "C".into(),
                ],
            );

            if !checksum_valid {
                ctx.put_ann(checksum.0, checksum.1, ANN_ERROR, vec!["Checksum invalid".into()]);
            }
        }
        // else: No response.

        self.lin_header.clear();
        self.lin_rsp.clear();
        Ok(())
    }

    fn handle_error(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.putx(ctx, ANN_INLINE_ERROR, &["Error", "Err", "E"]);
        Ok(())
    }

    /// `checksum_is_valid`: classic and enhanced (version 2) checksums.
    fn checksum_is_valid(&self, pid: u8, checksum: u8) -> bool {
        let mut checksum = i64::from(checksum);
        if self.lin_version == 2 {
            let id = pid & 0x3f;
            if id != 60 && id != 61 {
                checksum += i64::from(pid);
            }
        }

        for d in &self.lin_rsp {
            checksum += i64::from(d.2);
        }

        let carry_bits = checksum / 256;
        checksum += carry_bits;

        checksum & 0xff == 0xff
    }

    /// `calc_parity`: `p0 = i0 ^ i1 ^ i2 ^ i4`, `p1 = not (i1 ^ i3 ^ i4 ^ i5)`.
    fn calc_parity(pid: u8) -> u8 {
        let id = |i: u8| (pid & 0x3f) >> i & 1;
        let p0 = id(0) ^ id(1) ^ id(2) ^ id(4);
        let p1 = !(id(1) ^ id(3) ^ id(4) ^ id(5)) & 1;
        p0 | (p1 << 1)
    }
}

/// Unpacks `[ptype, rxtx, pdata]`; the UART output tuple.
fn unpack(item: &Item) -> Result<(&str, &Item)> {
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
fn data_value(pdata: &Item) -> Result<i64> {
    let Item::List(items) = pdata else {
        return Err(Error::arg("UART data is not a sequence"));
    };
    match items.first() {
        Some(Item::Int(v)) => Ok(*v),
        _ => Err(Error::arg("UART data has no byte value")),
    }
}

/// A scalar `pdata`, as BREAK and IDLE carry zero.
fn int_value(pdata: &Item) -> Result<i64> {
    match pdata {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::arg("UART data is not an integer")),
    }
}

impl Decoder for Lin {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        *self = Lin { lin_version: config.int("version", 2), ..Lin::new() };
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("lin is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        self.ss_block = ss;
        self.es_block = es;
        self.handle_item(ctx, item)
    }
}
