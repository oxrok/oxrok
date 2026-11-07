// Counterpart in sigrok:
//   libsigrokdecode/decoders/usb_signalling/pd.py,
//   Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>,
//   Copyright (C) 2012-2013 Uwe Hermann <uwe@hermann-uwe.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the USB low-/full-speed signalling and the
//   decoder's documented classes and texts.

//! USB signalling: J/K symbols, NRZI bits and bit stuffing, SOP/EOP.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};

const DP: usize = 0;
const DM: usize = 1;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "usb_signalling",
    name: "USB signalling",
    longname: "Universal Serial Bus (LS/FS) signalling",
    desc: "USB (low-speed/full-speed) signalling protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &["usb_signalling"],
    tags: &["PC"],
    channels: &[
        ChannelDesc { id: "dp", name: "D+", desc: "USB D+ signal" },
        ChannelDesc { id: "dm", name: "D-", desc: "USB D- signal" },
    ],
    optional_channels: &[],
    options: &[OptionDesc {
        id: "signalling",
        desc: "Signalling",
        default: OptionValue::s("automatic"),
        values: &[
            OptionValue::s("automatic"),
            OptionValue::s("full-speed"),
            OptionValue::s("low-speed"),
        ],
    }],
    annotations: &[
        AnnClass { id: "sym-j", desc: "J symbol" },
        AnnClass { id: "sym-k", desc: "K symbol" },
        AnnClass { id: "sym-se0", desc: "SE0 symbol" },
        AnnClass { id: "sym-se1", desc: "SE1 symbol" },
        AnnClass { id: "sop", desc: "Start of packet (SOP)" },
        AnnClass { id: "eop", desc: "End of packet (EOP)" },
        AnnClass { id: "bit", desc: "Bit" },
        AnnClass { id: "stuffbit", desc: "Stuff bit" },
        AnnClass { id: "error", desc: "Error" },
        AnnClass { id: "keep-alive", desc: "Low-speed keep-alive" },
        AnnClass { id: "reset", desc: "Reset" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[4, 5, 6, 7, 8, 9, 10] },
        AnnRow { id: "symbols", desc: "Symbols", classes: &[0, 1, 2, 3] },
    ],
    binary: &[],
    doc: "This PD decodes the USB (low-speed and full-speed) signalling protocol.\n\
\n\
Electrical/signalling layer (USB spec, chapter 7):\n\
\n\
USB signalling consists of two signal lines, both driven at 3.3V\n\
logic levels. The signals are DP (D+) and DM (D-), and normally operate in\n\
differential mode.\n\
\n\
Low-speed: The state where DP=1,DM=0 is K, the state DP=0,DM=1 is J.\n\
Full-speed: The state where DP=1,DM=0 is J, the state DP=0,DM=1 is K.\n\
\n\
A state SE0 is defined where DP=DM=0. This common mode signal is used to\n\
signal a reset or end of packet (EOP). A state SE1 is defined where DP=DM=1.\n\
\n\
Data transmitted on the USB is encoded with NRZI. A transition from J to K\n\
or vice-versa indicates a logic 0, while no transition indicates a logic 1.\n\
If 6 ones are transmitted consecutively, a zero is inserted to force a\n\
transition. This is known as bit stuffing.\n\
\n\
Data is transferred at a rate of 1.5Mbit/s (low-speed) / 12Mbit/s (full-speed).\n\
\n\
The SE0 transmitted to signal an end-of-packet is two bit intervals long\n\
(low-speed: 1.25uS - 1.50uS, full-speed: 160ns - 175ns).\n\
\n\
Details:\n\
https://en.wikipedia.org/wiki/USB\n\
http://www.usb.org/developers/docs/",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(UsbSignalling::default())
}

/// A decoded line state. `FsJ` and `LsJ` only exist while the signalling
/// is still `automatic`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Sym {
    J,
    K,
    Se0,
    Se1,
    FsJ,
    LsJ,
}

impl Sym {
    /// `symbols[signalling][(dp, dm)]` of the Python decoder.
    fn of(signalling: &str, dp: bool, dm: bool) -> Sym {
        match (dp, dm) {
            (false, false) => Sym::Se0,
            (true, true) => Sym::Se1,
            // Low-speed J and K are inverted compared to full-speed; the
            // low-speed PREamble segment keeps the full-speed polarity.
            (true, false) => match signalling {
                "low-speed" => Sym::K,
                "automatic" => Sym::FsJ,
                _ => Sym::J,
            },
            (false, true) => match signalling {
                "low-speed" => Sym::J,
                "automatic" => Sym::LsJ,
                _ => Sym::K,
            },
        }
    }

    fn name(self) -> &'static str {
        match self {
            Sym::J => "J",
            Sym::K => "K",
            Sym::Se0 => "SE0",
            Sym::Se1 => "SE1",
            Sym::FsJ => "FS_J",
            Sym::LsJ => "LS_J",
        }
    }

    /// `sym_annotation[sym]` of the Python decoder.
    fn annotation(self) -> Option<(usize, &'static [&'static str])> {
        match self {
            Sym::J => Some((0, &["J"])),
            Sym::K => Some((1, &["K"])),
            Sym::Se0 => Some((2, &["SE0", "0"])),
            Sym::Se1 => Some((3, &["SE1", "1"])),
            Sym::FsJ | Sym::LsJ => None,
        }
    }
}

/// `bitrates[signalling]`: `None` for `automatic`.
fn bitrate(signalling: &str) -> Option<f64> {
    match signalling {
        "low-speed" | "low-speed-rp" => Some(1_500_000.0),
        "full-speed" => Some(12_000_000.0),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    WaitIdle,
    GetBit,
    GetEop,
}

/// The USB signalling decoder.
pub struct UsbSignalling {
    configured: String,
    signalling: String,
    samplerate: u64,
    bitrate: Option<f64>,
    bitwidth: Option<f64>,
    samplepos: Option<f64>,
    samplenum_target: Option<u64>,
    samplenum_edge: Option<u64>,
    samplenum_lastedge: u64,
    edgepins: Option<(bool, bool)>,
    oldsym: Sym,
    ss_block: Option<u64>,
    consecutive_ones: u32,
    bits: String,
    state: State,
}

impl Default for UsbSignalling {
    fn default() -> Self {
        UsbSignalling {
            configured: "automatic".into(),
            signalling: "automatic".into(),
            samplerate: 0,
            bitrate: None,
            bitwidth: None,
            samplepos: None,
            samplenum_target: None,
            samplenum_edge: None,
            samplenum_lastedge: 0,
            edgepins: None,
            oldsym: Sym::J,
            ss_block: None,
            consecutive_ones: 0,
            bits: String::new(),
            state: State::Idle,
        }
    }
}

impl UsbSignalling {
    fn update_bitrate(&mut self) {
        self.bitrate = bitrate(&self.signalling);
        self.bitwidth = self.bitrate.map(|b| self.samplerate as f64 / b);
    }

    fn edge(&self) -> u64 {
        self.samplenum_edge.expect("an edge sample")
    }

    fn set_new_target_samplenum(&mut self) {
        let bw = self.bitwidth.expect("a bit width");
        let pos = self.samplepos.expect("a sample position") + bw;
        self.samplepos = Some(pos);
        self.samplenum_target = Some(pos as u64);
        self.samplenum_lastedge = self.edge();
        self.samplenum_edge = Some((pos - (bw / 2.0)) as u64);
    }

    fn wait_for_sop(&mut self, ctx: &mut dyn Context, sym: Sym) {
        // Wait for a Start of Packet (SOP), i.e. a J->K symbol change.
        if sym != Sym::K || self.oldsym != Sym::J {
            return;
        }
        self.consecutive_ones = 0;
        self.bits.clear();
        self.update_bitrate();
        let bw = self.bitwidth.expect("a bit width");
        let s = ctx.samplenum() as f64;
        self.samplepos = Some(s - (bw / 2.0) + 0.5);
        self.set_new_target_samplenum();
        let e = self.edge();
        ctx.put_python(e, e, Item::List(vec![Item::Str("SOP".into()), Item::None]));
        ctx.put_ann(e, e, 4, vec!["SOP".into(), "S".into()]);
        self.state = State::GetBit;
    }

    fn put_bit_range(&self, ctx: &mut dyn Context, class: usize, texts: &[&'static str]) {
        let (s, e) = (self.samplenum_lastedge, self.edge());
        ctx.put_ann(s, e, class, texts.iter().map(|t| (*t).to_owned()).collect());
    }

    /// `putpm(item)`: from the packet start to the current edge.
    fn put_sym_packet(&self, ctx: &mut dyn Context, item: Item) {
        let ss = self.ss_block.expect("a packet start");
        ctx.put_python(ss, self.edge(), item);
    }

    fn put_sym_block(&self, ctx: &mut dyn Context, class: usize, texts: &[&'static str]) {
        let ss = self.ss_block.expect("a packet start");
        ctx.put_ann(ss, self.edge(), class, texts.iter().map(|t| (*t).to_owned()).collect());
    }

    fn put_packet(&self, ctx: &mut dyn Context, item: Item) {
        let (s, e) = (self.samplenum_lastedge, self.edge());
        ctx.put_python(s, e, item);
    }

    fn handle_bit(&mut self, ctx: &mut dyn Context, b: char) {
        if self.consecutive_ones == 6 {
            if b == '0' {
                // Stuff bit.
                self.put_packet(ctx, Item::List(vec![Item::Str("STUFF BIT".into()), Item::None]));
                self.put_bit_range(ctx, 7, &["Stuff bit: 0", "SB: 0", "0"]);
                self.consecutive_ones = 0;
            } else {
                self.put_packet(ctx, Item::List(vec![Item::Str("ERR".into()), Item::None]));
                self.put_bit_range(ctx, 8, &["Bit stuff error", "BS ERR", "B"]);
                self.state = State::Idle;
            }
        } else {
            // Normal bit (not a stuff bit).
            self.put_packet(
                ctx,
                Item::List(vec![Item::Str("BIT".into()), Item::Str(b.to_string())]),
            );
            self.put_bit_range(ctx, 6, &[if b == '1' { "1" } else { "0" }]);
            if b == '1' {
                self.consecutive_ones += 1;
            } else {
                self.consecutive_ones = 0;
            }
        }
    }

    fn get_eop(&mut self, ctx: &mut dyn Context, sym: Sym) {
        // EOP: SE0 for >= 1 bittime (usually 2 bittimes), then J.
        self.set_new_target_samplenum();
        self.putsymptom(ctx, sym);
        self.oldsym = sym;
        if sym == Sym::Se0 {
            // pass
        } else if sym == Sym::J {
            // Got an EOP.
            self.put_sym_packet(ctx, Item::List(vec![Item::Str("EOP".into()), Item::None]));
            self.put_sym_block(ctx, 5, &["EOP", "E"]);
            self.state = State::WaitIdle;
        } else {
            self.put_sym_packet(ctx, Item::List(vec![Item::Str("ERR".into()), Item::None]));
            self.put_sym_block(ctx, 8, &["EOP Error", "EErr", "E"]);
            self.state = State::Idle;
        }
    }

    /// The `putpb(['SYM', sym])` plus `putb(sym_annotation[sym])` pair.
    fn putsymptom(&self, ctx: &mut dyn Context, sym: Sym) {
        let (s, e) = (self.samplenum_lastedge, self.edge());
        ctx.put_python(
            s,
            e,
            Item::List(vec![Item::Str("SYM".into()), Item::Str(sym.name().into())]),
        );
        if let Some((class, texts)) = sym.annotation() {
            ctx.put_ann(s, e, class, texts.iter().map(|t| (*t).to_owned()).collect());
        }
    }

    fn get_bit(&mut self, ctx: &mut dyn Context, sym: Sym) {
        self.set_new_target_samplenum();
        let b = if self.oldsym == sym { '1' } else { '0' };
        self.oldsym = sym;
        if sym == Sym::Se0 {
            // Start of an EOP. Change state, save edge.
            self.state = State::GetEop;
            self.ss_block = Some(self.samplenum_lastedge);
        } else {
            self.handle_bit(ctx, b);
        }
        self.putsymptom(ctx, sym);
        if self.bits.len() <= 16 {
            self.bits.push(b);
        }
        if self.bits.len() == 16 && self.bits == "0000000100111100" {
            // Sync and low-speed PREamble seen.
            let e = self.edge();
            ctx.put_python(e, e, Item::List(vec![Item::Str("EOP".into()), Item::None]));
            self.state = State::Idle;
            self.signalling = "low-speed-rp".into();
            self.update_bitrate();
            self.oldsym = Sym::J;
        }
        if b == '0' {
            let (dp, dm) = self.edgepins.expect("edge pin levels");
            let edgesym = Sym::of(&self.signalling, dp, dm);
            if edgesym != Sym::Se0 && edgesym != Sym::Se1 {
                let bw = self.bitwidth.expect("a bit width");
                let mut pos = self.samplepos.expect("a sample position");
                // The Python decoder assigns bitwidth first and then uses
                // the already updated value for samplepos.
                let new_bw = if edgesym == sym { bw - (0.001 * bw) } else { bw + (0.001 * bw) };
                self.bitwidth = Some(new_bw);
                if edgesym == sym {
                    pos -= 0.01 * new_bw;
                } else {
                    pos += 0.01 * new_bw;
                }
                self.samplepos = Some(pos);
            }
        }
    }

    fn handle_idle(&mut self, ctx: &mut dyn Context, sym: Sym) {
        self.samplenum_edge = Some(ctx.samplenum());
        let se0_length = (ctx.samplenum() as i64 - self.samplenum_lastedge as i64) as f64
            / self.samplerate as f64;
        if se0_length > 2.5e-6 {
            // 2.5us
            self.put_packet(ctx, Item::List(vec![Item::Str("RESET".into()), Item::None]));
            self.put_bit_range(ctx, 10, &["Reset", "Res", "R"]);
            self.signalling = self.configured.clone();
        } else if se0_length > 1.2e-6 && self.signalling == "low-speed" {
            self.put_packet(ctx, Item::List(vec![Item::Str("KEEP ALIVE".into()), Item::None]));
            self.put_bit_range(ctx, 9, &["Keep-alive", "KA", "A"]);
        }

        if sym == Sym::FsJ {
            self.signalling = "full-speed".into();
            self.update_bitrate();
        } else if sym == Sym::LsJ {
            self.signalling = "low-speed".into();
            self.update_bitrate();
        }
        self.oldsym = Sym::J;
        self.state = State::Idle;
    }

    fn levels(pins: &crate::decode::Pins) -> (bool, bool) {
        (pins.get(DP), pins.get(DM))
    }
}

impl Decoder for UsbSignalling {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        config.str("signalling", "automatic").clone_into(&mut self.configured);
        self.signalling = self.configured.clone();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        self.signalling = self.configured.clone();
        if self.signalling != "automatic" {
            self.update_bitrate();
        }
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        // Seed internal state from the very first sample.
        let pins = ctx.wait(&[])?;
        let (dp, dm) = Self::levels(&pins);
        let sym = Sym::of(&self.configured, dp, dm);
        self.handle_idle(ctx, sym);

        loop {
            // State machine.
            match self.state {
                State::Idle => {
                    // Wait for any edge on either DP and/or DM.
                    let pins = ctx.wait(&[
                        &[Cond::pin(DP, CondKind::Edge)],
                        &[Cond::pin(DM, CondKind::Edge)],
                    ])?;
                    let (dp, dm) = Self::levels(&pins);
                    let sym = Sym::of(&self.signalling, dp, dm);
                    if sym == Sym::Se0 {
                        self.samplenum_lastedge = ctx.samplenum();
                        self.state = State::WaitIdle;
                    } else {
                        self.wait_for_sop(ctx, sym);
                    }
                    self.edgepins = Some((dp, dm));
                }
                State::GetBit | State::GetEop => {
                    // Wait until we're in the middle of the desired bit.
                    let s = ctx.samplenum() as i64;
                    let edge = self.edge() as i64;
                    let skip = u64::try_from(edge - s).unwrap_or(0);
                    let pins = ctx.wait(&[&[Cond::Skip(skip)]])?;
                    self.edgepins = Some(Self::levels(&pins));
                    let s = ctx.samplenum() as i64;
                    let target = self.samplenum_target.expect("a target sample") as i64;
                    let skip = u64::try_from(target - s).unwrap_or(0);
                    let pins = ctx.wait(&[&[Cond::Skip(skip)]])?;
                    let (dp, dm) = Self::levels(&pins);
                    let sym = Sym::of(&self.signalling, dp, dm);
                    if self.state == State::GetBit {
                        self.get_bit(ctx, sym);
                    } else {
                        self.get_eop(ctx, sym);
                    }
                }
                State::WaitIdle => {
                    // Skip "all-low" input. Wait for high level on either DP or DM.
                    let mut pins = ctx.wait(&[])?;
                    let mut p = Self::levels(&pins);
                    while !p.0 && !p.1 {
                        pins = ctx.wait(&[
                            &[Cond::pin(DP, CondKind::High)],
                            &[Cond::pin(DM, CondKind::High)],
                        ])?;
                        p = Self::levels(&pins);
                    }
                    if ctx.samplenum() - self.samplenum_lastedge > 1 {
                        let sym = Sym::of(&self.configured, p.0, p.1);
                        self.handle_idle(ctx, sym);
                    } else {
                        let sym = Sym::of(&self.signalling, p.0, p.1);
                        self.wait_for_sop(ctx, sym);
                    }
                    self.edgepins = Some(p);
                }
            }
        }
    }
}
