// Counterpart in sigrok:
//   libsigrokdecode/decoders/jtag/pd.py,
//   Copyright (C) 2012-2015 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the IEEE 1149.1 state machine and the decoder's
//   documented classes and texts.

//! JTAG: the IEEE 1149.1 TAP state machine, TDI/TDO bits and bitstrings.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item,
};

const TDI: usize = 0;
const TDO: usize = 1;
const TCK: usize = 2;
const TMS: usize = 3;

/// The TAP states in `jtag_states` order, which is also the annotation
/// class order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    TestLogicReset,
    RunTestIdle,
    SelectDrScan,
    CaptureDr,
    UpdateDr,
    PauseDr,
    ShiftDr,
    Exit1Dr,
    Exit2Dr,
    SelectIrScan,
    CaptureIr,
    UpdateIr,
    PauseIr,
    ShiftIr,
    Exit1Ir,
    Exit2Ir,
}

impl Default for State {
    /// The Python decoder's `reset()` starts in RUN-TEST/IDLE.
    fn default() -> Self {
        State::RunTestIdle
    }
}

impl State {
    /// `jtag_states.index(self.state)`.
    fn index(self) -> usize {
        self as usize
    }

    /// The state's name as the Python decoder spells it.
    fn name(self) -> &'static str {
        NAMES[self.index()]
    }

    /// `self.state.startswith('SHIFT-') or self.state.startswith('EXIT1-')`.
    fn shift_or_exit1(self) -> bool {
        matches!(self, State::ShiftDr | State::Exit1Dr | State::ShiftIr | State::Exit1Ir)
    }

    /// `self.state.startswith('UPDATE-')`.
    fn update(self) -> bool {
        matches!(self, State::UpdateDr | State::UpdateIr)
    }

    /// `self.state[-2:]`, the `DR` or `IR` of the tree.
    fn tree(self) -> &'static str {
        match self {
            State::TestLogicReset
            | State::RunTestIdle
            | State::SelectDrScan
            | State::SelectIrScan => "",
            State::CaptureDr
            | State::UpdateDr
            | State::PauseDr
            | State::ShiftDr
            | State::Exit1Dr
            | State::Exit2Dr => "DR",
            State::CaptureIr
            | State::UpdateIr
            | State::PauseIr
            | State::ShiftIr
            | State::Exit1Ir
            | State::Exit2Ir => "IR",
        }
    }

    /// The state `advance_state_machine` moves to on TMS `tms`.
    #[allow(clippy::too_many_lines, clippy::match_same_arms)]
    fn advance(self, tms: bool) -> State {
        use State::{
            CaptureDr, CaptureIr, Exit1Dr, Exit1Ir, Exit2Dr, Exit2Ir, PauseDr, PauseIr,
            RunTestIdle, SelectDrScan, SelectIrScan, ShiftDr, ShiftIr, TestLogicReset, UpdateDr,
            UpdateIr,
        };
        match self {
            // Intro "tree".
            TestLogicReset => {
                if tms {
                    TestLogicReset
                } else {
                    RunTestIdle
                }
            }
            RunTestIdle => {
                if tms {
                    SelectDrScan
                } else {
                    RunTestIdle
                }
            }
            // DR "tree".
            SelectDrScan => {
                if tms {
                    SelectIrScan
                } else {
                    CaptureDr
                }
            }
            CaptureDr => {
                if tms {
                    Exit1Dr
                } else {
                    ShiftDr
                }
            }
            ShiftDr => {
                if tms {
                    Exit1Dr
                } else {
                    ShiftDr
                }
            }
            Exit1Dr => {
                if tms {
                    UpdateDr
                } else {
                    PauseDr
                }
            }
            PauseDr => {
                if tms {
                    Exit2Dr
                } else {
                    PauseDr
                }
            }
            Exit2Dr => {
                if tms {
                    UpdateDr
                } else {
                    ShiftDr
                }
            }
            UpdateDr => {
                if tms {
                    SelectDrScan
                } else {
                    RunTestIdle
                }
            }
            // IR "tree".
            SelectIrScan => {
                if tms {
                    TestLogicReset
                } else {
                    CaptureIr
                }
            }
            CaptureIr => {
                if tms {
                    Exit1Ir
                } else {
                    ShiftIr
                }
            }
            ShiftIr => {
                if tms {
                    Exit1Ir
                } else {
                    ShiftIr
                }
            }
            Exit1Ir => {
                if tms {
                    UpdateIr
                } else {
                    PauseIr
                }
            }
            PauseIr => {
                if tms {
                    Exit2Ir
                } else {
                    PauseIr
                }
            }
            Exit2Ir => {
                if tms {
                    UpdateIr
                } else {
                    ShiftIr
                }
            }
            UpdateIr => {
                if tms {
                    SelectDrScan
                } else {
                    RunTestIdle
                }
            }
        }
    }
}

const NAMES: [&str; 16] = [
    "TEST-LOGIC-RESET",
    "RUN-TEST/IDLE",
    "SELECT-DR-SCAN",
    "CAPTURE-DR",
    "UPDATE-DR",
    "PAUSE-DR",
    "SHIFT-DR",
    "EXIT1-DR",
    "EXIT2-DR",
    "SELECT-IR-SCAN",
    "CAPTURE-IR",
    "UPDATE-IR",
    "PAUSE-IR",
    "SHIFT-IR",
    "EXIT1-IR",
    "EXIT2-IR",
];

const ANN_BIT_TDI: usize = 16;
const ANN_BIT_TDO: usize = 17;
const ANN_BITSTRING_TDI: usize = 18;
const ANN_BITSTRING_TDO: usize = 19;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "jtag",
    name: "JTAG",
    longname: "Joint Test Action Group (IEEE 1149.1)",
    desc: "Protocol for testing, debugging, and flashing ICs.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["jtag"],
    tags: &["Debug/trace"],
    channels: &[
        ChannelDesc { id: "tdi", name: "TDI", desc: "Test data input" },
        ChannelDesc { id: "tdo", name: "TDO", desc: "Test data output" },
        ChannelDesc { id: "tck", name: "TCK", desc: "Test clock" },
        ChannelDesc { id: "tms", name: "TMS", desc: "Test mode select" },
    ],
    optional_channels: &[
        ChannelDesc { id: "trst", name: "TRST#", desc: "Test reset" },
        ChannelDesc { id: "srst", name: "SRST#", desc: "System reset" },
        ChannelDesc { id: "rtck", name: "RTCK", desc: "Return clock signal" },
    ],
    options: &[],
    annotations: &[
        AnnClass { id: "test-logic-reset", desc: "TEST-LOGIC-RESET" },
        AnnClass { id: "run-test/idle", desc: "RUN-TEST/IDLE" },
        AnnClass { id: "select-dr-scan", desc: "SELECT-DR-SCAN" },
        AnnClass { id: "capture-dr", desc: "CAPTURE-DR" },
        AnnClass { id: "update-dr", desc: "UPDATE-DR" },
        AnnClass { id: "pause-dr", desc: "PAUSE-DR" },
        AnnClass { id: "shift-dr", desc: "SHIFT-DR" },
        AnnClass { id: "exit1-dr", desc: "EXIT1-DR" },
        AnnClass { id: "exit2-dr", desc: "EXIT2-DR" },
        AnnClass { id: "select-ir-scan", desc: "SELECT-IR-SCAN" },
        AnnClass { id: "capture-ir", desc: "CAPTURE-IR" },
        AnnClass { id: "update-ir", desc: "UPDATE-IR" },
        AnnClass { id: "pause-ir", desc: "PAUSE-IR" },
        AnnClass { id: "shift-ir", desc: "SHIFT-IR" },
        AnnClass { id: "exit1-ir", desc: "EXIT1-IR" },
        AnnClass { id: "exit2-ir", desc: "EXIT2-IR" },
        AnnClass { id: "bit-tdi", desc: "Bit (TDI)" },
        AnnClass { id: "bit-tdo", desc: "Bit (TDO)" },
        AnnClass { id: "bitstring-tdi", desc: "Bitstring (TDI)" },
        AnnClass { id: "bitstring-tdo", desc: "Bitstring (TDO)" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits-tdi", desc: "Bits (TDI)", classes: &[16] },
        AnnRow { id: "bits-tdo", desc: "Bits (TDO)", classes: &[17] },
        AnnRow { id: "bitstrings-tdi", desc: "Bitstring (TDI)", classes: &[18] },
        AnnRow { id: "bitstrings-tdo", desc: "Bitstring (TDO)", classes: &[19] },
        AnnRow {
            id: "states",
            desc: "States",
            classes: &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
        },
    ],
    binary: &[],
    doc: "JTAG (Joint Test Action Group, IEEE 1149.1) is a debug and test \
interface with a four or five wire TAP: TCK clocks the state machine \
driven by TMS and shifts data through TDI and TDO. The decoder follows \
the state machine, annotates each state and collects the bits clocked \
into and out of the instruction and data registers.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Jtag::default())
}

/// The JTAG decoder.
#[derive(Default)]
pub struct Jtag {
    state: State,
    oldstate: Option<State>,
    /// Bits most recent first.
    bits_tdi: Vec<u8>,
    bits_tdo: Vec<u8>,
    /// Sample numbers of the bits, most recent first: `[ss, es]`.
    samplenums_tdi: Vec<[i64; 2]>,
    samplenums_tdo: Vec<[i64; 2]>,
    ss_item: u64,
    es_item: u64,
    ss_bitstring: u64,
    es_bitstring: u64,
    first: bool,
    first_bit: bool,
}

/// `'0x%x' % int('0b0' + b, 2)`: lowercase hex without leading zeros.
fn bin_to_hex(bits: &str) -> String {
    let trimmed = bits.trim_start_matches('0');
    if trimmed.is_empty() {
        return "0".into();
    }
    let mut out = String::with_capacity(trimmed.len().div_ceil(4));
    let head = trimmed.len() % 4;
    let mut idx = 0;
    if head > 0 {
        out.push(hex_digit(&trimmed[..head]));
        idx = head;
    }
    while idx < trimmed.len() {
        out.push(hex_digit(&trimmed[idx..idx + 4]));
        idx += 4;
    }
    out
}

fn hex_digit(bits: &str) -> char {
    let v = bits.bytes().fold(0u8, |v, c| (v << 1) | u8::from(c == b'1'));
    char::from_digit(u32::from(v), 16).unwrap_or('0')
}

fn bits_text(bits: &[u8]) -> String {
    bits.iter().map(|&b| if b != 0 { '1' } else { '0' }).collect()
}

fn samplenums_item(nums: &[[i64; 2]]) -> Item {
    Item::List(
        nums.iter().map(|&[ss, es]| Item::List(vec![Item::Int(ss), Item::Int(es)])).collect(),
    )
}

impl Jtag {
    fn putx(&self, ctx: &mut dyn Context, class: usize, text: &str) {
        ctx.put_ann(self.ss_item, self.es_item, class, vec![text.to_owned()]);
    }

    fn putp(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.ss_item, self.es_item, item);
    }

    fn putx_bs(&self, ctx: &mut dyn Context, class: usize, text: &str) {
        ctx.put_ann(self.ss_bitstring, self.es_bitstring, class, vec![text.to_owned()]);
    }

    fn putp_bs(&self, ctx: &mut dyn Context, item: Item) {
        ctx.put_python(self.ss_bitstring, self.es_bitstring, item);
    }

    /// Emits one bitstring class and its Python item, then clears the
    /// collected bits, as the Python `UPDATE-*` block does twice.
    fn put_bitstring(
        &mut self,
        ctx: &mut dyn Context,
        tree: &str,
        class: usize,
        bits: &mut Vec<u8>,
        nums: &mut Vec<[i64; 2]>,
        signal: &str,
    ) {
        // `self.bits_tdi[1:]` drops the most recent bit; the lists are
        // never empty by the time an UPDATE state is reached.
        let rest = bits.get(1..).unwrap_or(&[]);
        let b = bits_text(rest);
        let h = bin_to_hex(&b);
        let len = rest.len();
        let t = format!("{tree} {signal}");
        let s = format!("{t}: {b} (0x{h}), {len} bits");
        self.putx_bs(ctx, class, &s);
        let rest_nums = nums.get(1..).unwrap_or(&[]);
        let item = Item::List(vec![
            Item::Str(t),
            Item::List(vec![Item::Str(b), samplenums_item(rest_nums)]),
        ]);
        self.putp_bs(ctx, item);
        bits.clear();
        nums.clear();
    }

    fn handle_rising_tck_edge(&mut self, ctx: &mut dyn Context, tdi: bool, tdo: bool, tms: bool) {
        let samplenum = ctx.samplenum();

        // Rising TCK edges always advance the state machine.
        self.oldstate = Some(self.state);
        self.state = self.state.advance(tms);
        let oldstate = self.oldstate.unwrap_or(self.state);

        if self.first {
            // Save the start sample and item for later (no output yet).
            self.ss_item = samplenum;
            self.first = false;
        } else {
            // Output the saved item (from the last CLK edge to the current).
            self.es_item = samplenum;
            // Output the old state (from last rising TCK edge to current one).
            self.putx(ctx, oldstate.index(), oldstate.name());
            self.putp(
                ctx,
                Item::List(vec![
                    Item::Str("NEW STATE".into()),
                    Item::Str(self.state.name().into()),
                ]),
            );
        }

        // Upon SHIFT-*/EXIT1-* collect the current TDI/TDO values.
        if oldstate.shift_or_exit1() {
            if self.first_bit {
                self.ss_bitstring = samplenum;
                self.first_bit = false;
            } else {
                self.putx(ctx, ANN_BIT_TDI, &self.bits_tdi[0].to_string());
                self.putx(ctx, ANN_BIT_TDO, &self.bits_tdo[0].to_string());
                // Use the current sample as ES of the previous bit.
                self.samplenums_tdi[0][1] = samplenum as i64;
                self.samplenums_tdo[0][1] = samplenum as i64;
            }

            self.bits_tdi.insert(0, u8::from(tdi));
            self.bits_tdo.insert(0, u8::from(tdo));

            // Use the current sample as SS of the current bit.
            self.samplenums_tdi.insert(0, [samplenum as i64, -1]);
            self.samplenums_tdo.insert(0, [samplenum as i64, -1]);
        }

        // Output all TDI/TDO bits if we just switched to UPDATE-*.
        if self.state.update() {
            self.es_bitstring = samplenum;
            let tree = self.state.tree();
            let mut bits = std::mem::take(&mut self.bits_tdi);
            let mut nums = std::mem::take(&mut self.samplenums_tdi);
            self.put_bitstring(ctx, tree, ANN_BITSTRING_TDI, &mut bits, &mut nums, "TDI");
            let mut bits = std::mem::take(&mut self.bits_tdo);
            let mut nums = std::mem::take(&mut self.samplenums_tdo);
            self.put_bitstring(ctx, tree, ANN_BITSTRING_TDO, &mut bits, &mut nums, "TDO");
            self.first_bit = true;
            self.ss_bitstring = samplenum;
        }

        self.ss_item = samplenum;
    }
}

impl Decoder for Jtag {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Jtag {
            // self.state = 'TEST-LOGIC-RESET' is commented out upstream.
            state: State::RunTestIdle,
            first: true,
            first_bit: true,
            ..Jtag::default()
        };
        Ok(())
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let cond: &[Cond] = &[Cond::pin(TCK, CondKind::Rising)];
        loop {
            let pins = ctx.wait(&[cond])?;
            self.handle_rising_tck_edge(ctx, pins.get(TDI), pins.get(TDO), pins.get(TMS));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::bin_to_hex;

    #[test]
    fn hex_matches_python_int_of_a_binary_string() {
        assert_eq!(bin_to_hex(""), "0");
        assert_eq!(bin_to_hex("000000000"), "0");
        assert_eq!(bin_to_hex("111111110"), "1fe");
        assert_eq!(bin_to_hex("0001"), "1");
        assert_eq!(bin_to_hex("10000"), "10");
        assert_eq!(bin_to_hex("10101010"), "aa");
    }
}
