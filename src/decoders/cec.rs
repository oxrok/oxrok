// Counterpart in sigrok:
//   libsigrokdecode/decoders/cec/pd.py,
//   Copyright (C) 2018 Jorge Solla Rubiales <jorgesolla@gmail.com>,
//   and decoders/cec/protocoldata.py, same copyright,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the HDMI-CEC bit timing and the decoder's
//   documented classes and texts.
#![allow(clippy::format_push_string, clippy::too_many_lines)]

//! HDMI-CEC: start/bits/bytes/frames with EOM, ACK/NACK and the
//! protocol header/opcode names.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo,
};

const CEC: usize = 0;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_ST: usize = 0;
const ANN_EOM0: usize = 1;
const ANN_EOM1: usize = 2;
const ANN_NACK: usize = 3;
const ANN_ACK: usize = 4;
const ANN_BITS: usize = 5;
const ANN_BYTES: usize = 6;
const ANN_FRAMES: usize = 7;
const ANN_SECTIONS: usize = 8;
const ANN_WARNINGS: usize = 9;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "cec",
    name: "CEC",
    longname: "HDMI-CEC",
    desc: "HDMI Consumer Electronics Control (CEC) protocol.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Display", "PC"],
    channels: &[ChannelDesc { id: "cec", name: "CEC", desc: "CEC bus data" }],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "st", desc: "Start" },
        AnnClass { id: "eom-0", desc: "End of message" },
        AnnClass { id: "eom-1", desc: "Message continued" },
        AnnClass { id: "nack", desc: "ACK not set" },
        AnnClass { id: "ack", desc: "ACK set" },
        AnnClass { id: "bits", desc: "Bits" },
        AnnClass { id: "bytes", desc: "Bytes" },
        AnnClass { id: "frames", desc: "Frames" },
        AnnClass { id: "sections", desc: "Sections" },
        AnnClass { id: "warnings", desc: "Warnings" },
    ],
    annotation_rows: &[
        AnnRow { id: "bits", desc: "Bits", classes: &[0, 1, 2, 3, 4, 5] },
        AnnRow { id: "bytes", desc: "Bytes", classes: &[6] },
        AnnRow { id: "frames", desc: "Frames", classes: &[7] },
        AnnRow { id: "sections", desc: "Sections", classes: &[8] },
        AnnRow { id: "warnings", desc: "Warnings", classes: &[9] },
    ],
    binary: &[],
    doc: "The Consumer Electronics Control (CEC) protocol allows users to command and\n\
control devices connected through HDMI.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Cec::default())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pulse {
    Invalid,
    Start,
    Zero,
    One,
}

// Pulse times in milliseconds.
const START_LOW: (f64, f64) = (3.5, 3.9);
const START_TOTAL: (f64, f64) = (4.3, 4.7);
const ZERO_LOW: (f64, f64) = (1.3, 1.7);
const ZERO_TOTAL: (f64, f64) = (2.05, 2.75);
const ONE_LOW: (f64, f64) = (0.4, 0.8);
const ONE_TOTAL: (f64, f64) = (2.05, 2.75);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stat {
    WaitStart,
    GetBits,
    WaitEom,
    WaitAck,
}

const LOGICAL_ADDRESSES: [&str; 15] = [
    "TV",
    "Recording_1",
    "Recording_2",
    "Tuner_1",
    "Playback_1",
    "AudioSystem",
    "Tuner2",
    "Tuner3",
    "Playback_2",
    "Recording_3",
    "Tuner_4",
    "Playback_3",
    "Backup_1",
    "Backup_2",
    "FreeUse",
];

// List taken from LibCEC.
const OPCODES: [(i64, &str); 62] = [
    (0x82, "ACTIVE_SOURCE"),
    (0x04, "IMAGE_VIEW_ON"),
    (0x0D, "TEXT_VIEW_ON"),
    (0x9D, "INACTIVE_SOURCE"),
    (0x85, "REQUEST_ACTIVE_SOURCE"),
    (0x80, "ROUTING_CHANGE"),
    (0x81, "ROUTING_INFORMATION"),
    (0x86, "SET_STREAM_PATH"),
    (0x36, "STANDBY"),
    (0x0B, "RECORD_OFF"),
    (0x09, "RECORD_ON"),
    (0x0A, "RECORD_STATUS"),
    (0x0F, "RECORD_TV_SCREEN"),
    (0x33, "CLEAR_ANALOGUE_TIMER"),
    (0x99, "CLEAR_DIGITAL_TIMER"),
    (0xA1, "CLEAR_EXTERNAL_TIMER"),
    (0x34, "SET_ANALOGUE_TIMER"),
    (0x97, "SET_DIGITAL_TIMER"),
    (0xA2, "SET_EXTERNAL_TIMER"),
    (0x67, "SET_TIMER_PROGRAM_TITLE"),
    (0x43, "TIMER_CLEARED_STATUS"),
    (0x35, "TIMER_STATUS"),
    (0x9E, "CEC_VERSION"),
    (0x9F, "GET_CEC_VERSION"),
    (0x83, "GIVE_PHYSICAL_ADDRESS"),
    (0x91, "GET_MENU_LANGUAGE"),
    (0x84, "REPORT_PHYSICAL_ADDRESS"),
    (0x32, "SET_MENU_LANGUAGE"),
    (0x42, "DECK_CONTROL"),
    (0x1B, "DECK_STATUS"),
    (0x1A, "GIVE_DECK_STATUS"),
    (0x41, "PLAY"),
    (0x08, "GIVE_TUNER_DEVICE_STATUS"),
    (0x92, "SELECT_ANALOGUE_SERVICE"),
    (0x93, "SELECT_DIGITAL_SERVICE"),
    (0x07, "TUNER_DEVICE_STATUS"),
    (0x06, "TUNER_STEP_DECREMENT"),
    (0x05, "TUNER_STEP_INCREMENT"),
    (0x87, "DEVICE_VENDOR_ID"),
    (0x8C, "GIVE_DEVICE_VENDOR_ID"),
    (0x89, "VENDOR_COMMAND"),
    (0xA0, "VENDOR_COMMAND_WITH_ID"),
    (0x8A, "VENDOR_REMOTE_BUTTON_DOWN"),
    (0x8B, "VENDOR_REMOTE_BUTTON_UP"),
    (0x64, "SET_OSD_STRING"),
    (0x46, "GIVE_OSD_NAME"),
    (0x47, "SET_OSD_NAME"),
    (0x8D, "MENU_REQUEST"),
    (0x8E, "MENU_STATUS"),
    (0x44, "USER_CONTROL_PRESSED"),
    (0x45, "USER_CONTROL_RELEASE"),
    (0x8F, "GIVE_DEVICE_POWER_STATUS"),
    (0x90, "REPORT_POWER_STATUS"),
    (0x00, "FEATURE_ABORT"),
    (0xFF, "ABORT"),
    (0x71, "GIVE_AUDIO_STATUS"),
    (0x7D, "GIVE_SYSTEM_AUDIO_MODE_STATUS"),
    (0x7A, "REPORT_AUDIO_STATUS"),
    (0x72, "SET_SYSTEM_AUDIO_MODE"),
    (0x70, "SYSTEM_AUDIO_MODE_REQUEST"),
    (0x7E, "SYSTEM_AUDIO_MODE_STATUS"),
    (0x9A, "SET_AUDIO_RATE"),
];

fn resolve_logical_address(id: i64, is_initiator: bool) -> &'static str {
    if !(0..=0x0F).contains(&id) {
        return "Invalid";
    }
    // Special handling of 0x0F.
    if id == 0x0F {
        return if is_initiator { "Unregistered" } else { "Broadcast" };
    }
    LOGICAL_ADDRESSES[id as usize]
}

fn decode_header(header: i64) -> (&'static str, &'static str) {
    let src = (header & 0xF0) >> 4;
    let dst = header & 0x0F;
    (resolve_logical_address(src, true), resolve_logical_address(dst, false))
}

fn opcode(value: i64) -> &'static str {
    OPCODES.iter().find(|&&(v, _)| v == value).map_or("Invalid", |&(_, name)| name)
}

/// The HDMI-CEC decoder.
pub struct Cec {
    samplerate: u64,
    max_ack_len_samples: i64,
    stat: Stat,
    fall_start: i64,
    fall_end: i64,
    rise: i64,
    eom: Option<i64>,
    bit_count: i64,
    byte_count: i64,
    byte: i64,
    byte_start: i64,
    frame_start: i64,
    frame_end: i64,
    is_nack: i64,
    cmd_bytes: Vec<(i64, i64, i64)>,
}

impl Default for Cec {
    fn default() -> Self {
        Cec {
            samplerate: 0,
            max_ack_len_samples: 0,
            stat: Stat::WaitStart,
            fall_start: 0,
            fall_end: 0,
            rise: 0,
            eom: None,
            bit_count: 0,
            byte_count: 0,
            byte: 0,
            byte_start: 0,
            frame_start: 0,
            frame_end: 0,
            is_nack: 0,
            cmd_bytes: Vec::new(),
        }
    }
}

impl Cec {
    fn precalculate(&mut self) {
        // Restrict max length of ACK/NACK labels to 2 BIT pulses.
        let bit_time = ZERO_TOTAL.0 * 2.0;
        self.max_ack_len_samples =
            (bit_time / 1000.0 * self.samplerate as f64).round_ties_even() as i64;
    }

    fn reset_frame_vars(&mut self) {
        self.eom = None;
        self.bit_count = 0;
        self.byte_count = 0;
        self.byte = 0;
        self.byte_start = 0;
        self.frame_start = 0;
        self.frame_end = 0;
        self.is_nack = 0;
        self.cmd_bytes.clear();
    }

    fn handle_frame(&mut self, ctx: &mut dyn Context, is_nack: bool) {
        let mut string = String::new();
        for (i, b) in self.cmd_bytes.iter().enumerate() {
            string.push_str(&format!("{:02x}", b.2));
            if i != self.cmd_bytes.len() - 1 {
                string.push(':');
            }
        }

        ctx.put_ann(
            self.frame_start.max(0) as u64,
            self.frame_end.max(0) as u64,
            ANN_FRAMES,
            vec![string],
        );

        let mut i = 0;
        let mut operands = 0;
        let mut string = String::new();
        while i < self.cmd_bytes.len() {
            if i == 0 {
                // Parse header.
                let (src, dst) = decode_header(self.cmd_bytes[i].2);
                string = format!("HDR: {src}, {dst}");
            } else if i == 1 {
                // Parse opcode.
                string.push_str(" | OPC: ");
                string.push_str(opcode(self.cmd_bytes[i].2));
            } else {
                // Parse operands.
                if operands == 0 {
                    string.push_str(" | OPS: ");
                }
                operands += 1;
                string.push_str(&format!("0x{:02x}", self.cmd_bytes[i].2));
                if i != self.cmd_bytes.len() - 1 {
                    string.push_str(", ");
                }
            }
            i += 1;
        }

        // Header only commands are PINGS.
        if i == 1 {
            string.push_str(if self.eom.unwrap_or(0) != 0 {
                " | OPC: PING"
            } else {
                " | OPC: NONE. Aborted cmd"
            });
        }

        // Add extra information (ack of the command from the destination).
        string.push_str(if is_nack { " | R: NACK" } else { " | R: ACK" });

        ctx.put_ann(
            self.frame_start.max(0) as u64,
            self.frame_end.max(0) as u64,
            ANN_SECTIONS,
            vec![string],
        );
    }

    fn process(&mut self, ctx: &mut dyn Context) {
        let zero_time = (self.rise - self.fall_start) as f64 / self.samplerate as f64 * 1000.0;
        let total_time = (self.fall_end - self.fall_start) as f64 / self.samplerate as f64 * 1000.0;
        let mut pulse = Pulse::Invalid;

        // VALIDATION: Identify pulse based on length of the low period.
        if zero_time >= START_LOW.0 && zero_time <= START_LOW.1 {
            pulse = Pulse::Start;
        } else if zero_time >= ZERO_LOW.0 && zero_time <= ZERO_LOW.1 {
            pulse = Pulse::Zero;
        } else if zero_time >= ONE_LOW.0 && zero_time <= ONE_LOW.1 {
            pulse = Pulse::One;
        }

        // VALIDATION: Invalid pulse.
        if pulse == Pulse::Invalid {
            self.stat = Stat::WaitStart;
            ctx.put_ann(
                self.fall_start.max(0) as u64,
                self.fall_end.max(0) as u64,
                ANN_WARNINGS,
                vec!["Invalid pulse: Wrong timing".into()],
            );
            return;
        }

        // VALIDATION: If waiting for start, discard everything else.
        if self.stat == Stat::WaitStart && pulse != Pulse::Start {
            ctx.put_ann(
                self.fall_start.max(0) as u64,
                self.fall_end.max(0) as u64,
                ANN_WARNINGS,
                vec!["Expected START: BIT found".into()],
            );
            return;
        }

        // VALIDATION: If waiting for ACK or EOM, only BIT pulses (0/1)
        // are expected.
        if (self.stat == Stat::WaitAck || self.stat == Stat::WaitEom) && pulse == Pulse::Start {
            ctx.put_ann(
                self.fall_start.max(0) as u64,
                self.fall_end.max(0) as u64,
                ANN_WARNINGS,
                vec!["Expected BIT: START received)".into()],
            );
            self.stat = Stat::WaitStart;
        }

        // VALIDATION: ACK bit pulse remains high till the next frame (if
        // any): Validate only min time of the low period.
        if self.stat == Stat::WaitAck && pulse != Pulse::Start {
            let min = match pulse {
                Pulse::Start => START_TOTAL.0,
                Pulse::Zero => ZERO_TOTAL.0,
                Pulse::One => ONE_TOTAL.0,
                Pulse::Invalid => 0.0,
            };
            if total_time < min {
                ctx.put_ann(
                    self.fall_start.max(0) as u64,
                    self.fall_end.max(0) as u64,
                    ANN_WARNINGS,
                    vec!["ACK pulse below minimun time".into()],
                );
                self.stat = Stat::WaitStart;
                return;
            }
        }

        // VALIDATION / PING FRAME DETECTION: Initiator doesn't sets the
        // EOM = 1 but stops sending when ack doesn't arrive.
        if self.stat == Stat::GetBits && pulse == Pulse::Start {
            // Make sure we received a complete byte to consider it a
            // valid ping.
            if self.bit_count == 0 {
                let is_nack = self.is_nack != 0;
                self.handle_frame(ctx, is_nack);
            } else {
                ctx.put_ann(
                    self.frame_start.max(0) as u64,
                    ctx.samplenum(),
                    ANN_WARNINGS,
                    vec!["ERROR: Incomplete byte received".into()],
                );
            }

            // Set wait start so we receive next frame.
            self.stat = Stat::WaitStart;
        }

        // VALIDATION: Check timing of the BIT (0/1) pulse in any other
        // case (not waiting for ACK).
        if self.stat != Stat::WaitAck && pulse != Pulse::Start {
            let total = match pulse {
                Pulse::Start => START_TOTAL,
                Pulse::Zero => ZERO_TOTAL,
                Pulse::One => ONE_TOTAL,
                Pulse::Invalid => (0.0, 0.0),
            };
            if total_time < total.0 || total_time > total.1 {
                ctx.put_ann(
                    self.fall_start.max(0) as u64,
                    self.fall_end.max(0) as u64,
                    ANN_WARNINGS,
                    vec!["Bit pulse exceeds total pulse timespan".into()],
                );
                self.stat = Stat::WaitStart;
                return;
            }
        }

        let mut bit = 0i64;
        match pulse {
            Pulse::Zero => bit = 0,
            Pulse::One => bit = 1,
            _ => {}
        }

        // STATE: WAIT START.
        if self.stat == Stat::WaitStart {
            self.stat = Stat::GetBits;
            self.reset_frame_vars();
            ctx.put_ann(
                self.fall_start.max(0) as u64,
                self.fall_end.max(0) as u64,
                ANN_ST,
                vec!["ST".into()],
            );
        // STATE: GET BITS.
        } else if self.stat == Stat::GetBits {
            // Reset stats on first bit.
            if self.bit_count == 0 {
                self.byte_start = self.fall_start;
                self.byte = 0;

                // If 1st byte of the datagram save its sample num.
                if self.cmd_bytes.is_empty() {
                    self.frame_start = self.fall_start;
                }
            }

            self.byte += bit << (7 - self.bit_count);
            self.bit_count += 1;
            ctx.put_ann(
                self.fall_start.max(0) as u64,
                self.fall_end.max(0) as u64,
                ANN_BITS,
                vec![bit.to_string()],
            );

            if self.bit_count == 8 {
                self.bit_count = 0;
                self.byte_count += 1;
                self.stat = Stat::WaitEom;
                ctx.put_ann(
                    self.byte_start.max(0) as u64,
                    ctx.samplenum(),
                    ANN_BYTES,
                    vec![format!("0x{:02x}", self.byte)],
                );
                self.cmd_bytes.push((self.byte_start, ctx.samplenum() as i64, self.byte));
            }
        // STATE: WAIT EOM.
        } else if self.stat == Stat::WaitEom {
            self.eom = Some(bit);
            self.frame_end = self.fall_end;

            let a =
                if self.eom.unwrap_or(0) != 0 { (ANN_EOM1, "EOM=Y") } else { (ANN_EOM0, "EOM=N") };
            ctx.put_ann(
                self.fall_start.max(0) as u64,
                self.fall_end.max(0) as u64,
                a.0,
                vec![a.1.into()],
            );

            self.stat = Stat::WaitAck;
        // STATE: WAIT ACK.
        } else if self.stat == Stat::WaitAck {
            // If a frame with broadcast destination is being sent, the
            // ACK is inverted: a 0 is considered a NACK, therefore we
            // invert the value of the bit here, so we match the real
            // meaning of it.
            if !self.cmd_bytes.is_empty() && (self.cmd_bytes[0].2 & 0x0F) == 0x0F {
                bit = !bit & 0x01;
            }

            let ann_end = if (self.fall_end - self.fall_start) > self.max_ack_len_samples {
                self.fall_start + self.max_ack_len_samples
            } else {
                self.fall_end
            };

            if bit != 0 {
                // Any NACK detected in the frame is enough to consider
                // the whole frame NACK'd.
                self.is_nack = 1;
                ctx.put_ann(
                    self.fall_start.max(0) as u64,
                    ann_end.max(0) as u64,
                    ANN_NACK,
                    vec!["NACK".into()],
                );
            } else {
                ctx.put_ann(
                    self.fall_start.max(0) as u64,
                    ann_end.max(0) as u64,
                    ANN_ACK,
                    vec!["ACK".into()],
                );
            }

            // After ACK bit, wait for new datagram or continue reading
            // current one based on EOM value.
            if self.eom.unwrap_or(0) != 0 || self.is_nack != 0 {
                self.stat = Stat::WaitStart;
                let is_nack = self.is_nack != 0;
                self.handle_frame(ctx, is_nack);
            } else {
                self.stat = Stat::GetBits;
            }
        }
    }
}

impl Decoder for Cec {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        self.precalculate();
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::arg("Cannot decode without samplerate."));
        }

        // Wait for first falling edge.
        ctx.wait(&[&[Cond::Pin(CEC, CondKind::Falling)]])?;
        self.fall_end = ctx.samplenum() as i64;

        loop {
            ctx.wait(&[&[Cond::Pin(CEC, CondKind::Rising)]])?;
            self.rise = ctx.samplenum() as i64;

            if self.stat == Stat::WaitAck {
                let skip = self.max_ack_len_samples.max(0) as u64;
                let refs: &[&[Cond]] = &[&[Cond::Pin(CEC, CondKind::Falling)], &[Cond::Skip(skip)]];
                ctx.wait(refs)?;
            } else {
                ctx.wait(&[&[Cond::Pin(CEC, CondKind::Falling)]])?;
            }

            self.fall_start = self.fall_end;
            self.fall_end = ctx.samplenum() as i64;
            self.process(ctx);

            // If there was a timeout while waiting for ACK: RESYNC.
            // Note: This is an expected situation as no new falling edge
            // will happen until next frame is transmitted.
            let m = ctx.matched();
            if m.len() == 2 && !m[0] && m[1] {
                ctx.wait(&[&[Cond::Pin(CEC, CondKind::Falling)]])?;
                self.fall_end = ctx.samplenum() as i64;
            }
        }
    }
}
