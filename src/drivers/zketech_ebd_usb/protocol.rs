// Counterpart in sigrok:
//   libsigrok/src/hardware/zketech-ebd-usb/protocol.c,
//   libsigrok/src/hardware/zketech-ebd-usb/protocol.h,
//   Copyright (C) 2018 Sven Bursch-Osewold <sb_git@bursch.com>,
//   Copyright (C) 2019 King Kévin <kingkevin@cuvoodoo.info>,
//   with commits by Uwe Hermann, Florian Ragwitz, Frank Stettner, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The EBD-USB's frames.
//!
//! A frame starts with `fa` and ends with `f8`. From the host it has ten bytes:
//! the type, the current limit in mA, the cutoff in 10 mV, two bytes of a time
//! limit the driver leaves at 0, and the XOR of all but the ends. From the device
//! a measurement has nineteen: the type (`00` or `64` without load, `0a` or `6e`
//! with), the current in 0.1 mA, the volts of VBUS, D+ and D- in mV, and in a
//! `0a` the limit and the cutoff as the host sends them, then the XOR at 17. Two
//! byte values are in base 240, so neither end can stand for a value.
//!
//! The commands: `05` starts the measurements and `06` stops them, `01` starts
//! the load, `02` toggles it and `07` changes its values.

use crate::core::Result;

/// `MSG_MAX_LEN`, a measurement's length and the most [`read_message`] holds.
pub const MSG_MAX_LEN: usize = 19;
/// `MSG_FRAME_BEGIN`.
pub const FRAME_BEGIN: u8 = 0xfa;
/// `MSG_FRAME_END`.
pub const FRAME_END: u8 = 0xf8;
/// A command's length.
pub const COMMAND_LEN: usize = 10;
/// Reads a message may take, those that time out too.
pub const MAX_TURNS: usize = 200;
/// The base of a two byte value.
const BASE: i32 = 240;

/// The divisor of a current limit in a command or a measurement.
pub const LIMIT_DIVISOR: f32 = 1000.0;
/// The divisor of a threshold in a command or a measurement.
pub const THRESHOLD_DIVISOR: f32 = 100.0;
/// The divisor of a measured current.
const CURRENT_DIVISOR: f32 = 10000.0;
/// The divisor of a measured voltage.
const VOLTAGE_DIVISOR: f32 = 1000.0;

/// `decode_value`. The sum is in double precision, as `240.0` makes it in C.
#[must_use]
pub fn decode_value(hi: u8, lo: u8, divisor: f32) -> f32 {
    ((f64::from(hi) * 240.0 + f64::from(lo)) / f64::from(divisor)) as f32
}

/// `encode_value`: a product in single precision, truncated. What the setter
/// takes never goes out of range, and a byte is what C's assignment makes of a
/// value that does.
#[must_use]
pub fn encode_value(value: f32, divisor: f32) -> [u8; 2] {
    let value = (value * divisor) as i32;
    [(value / BASE) as u8, (value % BASE) as u8]
}

/// A command with its checksum.
fn command(kind: u8, limit: [u8; 2], threshold: [u8; 2]) -> [u8; COMMAND_LEN] {
    let mut f = [0; COMMAND_LEN];
    f[0] = FRAME_BEGIN;
    f[1] = kind;
    f[2..4].copy_from_slice(&limit);
    f[4..6].copy_from_slice(&threshold);
    f[8] = f[1..8].iter().fold(0, |x, b| x ^ b);
    f[9] = FRAME_END;
    f
}

/// `ebd_init`'s command.
#[must_use]
pub fn init() -> [u8; COMMAND_LEN] {
    command(0x05, [0; 2], [0; 2])
}

/// `ebd_stop`'s command.
#[must_use]
pub fn stop() -> [u8; COMMAND_LEN] {
    command(0x06, [0; 2], [0; 2])
}

/// `ebd_loadtoggle`'s command.
#[must_use]
pub fn toggle() -> [u8; COMMAND_LEN] {
    command(0x02, [0; 2], [0; 2])
}

/// `ebd_loadstart`'s command.
#[must_use]
pub fn load_start(limit: f32, threshold: f32) -> [u8; COMMAND_LEN] {
    command(0x01, encode_value(limit, LIMIT_DIVISOR), encode_value(threshold, THRESHOLD_DIVISOR))
}

/// `send_cfg`'s command.
#[must_use]
pub fn config(limit: f32, threshold: f32) -> [u8; COMMAND_LEN] {
    command(0x07, encode_value(limit, LIMIT_DIVISOR), encode_value(threshold, THRESHOLD_DIVISOR))
}

/// `ebd_current_is0`.
#[must_use]
pub fn current_is_0(limit: f32) -> bool {
    limit < 0.001
}

/// What [`read_message`] came back with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Message {
    /// A frame of more than two bytes, ending in `f8`, of this length. Its first
    /// byte is `fa` unless a frame too long came before.
    Frame([u8; MSG_MAX_LEN], usize),
    /// C's 0: the reads ran out.
    Nothing,
    /// C's -1: a frame with no body.
    Bad,
    /// The stop came first, which C has no way of seeing here.
    Stopped,
}

/// `ebd_read_message`, with `read` for its one-byte `serial_read_blocking`: it
/// says how many bytes came, the turn counting either way.
///
/// C looks at `buf[-1]` as a frame end when it has nothing yet, on the way into
/// the second loop (`protocol.c:207`) and after a frame too long; here nothing
/// is no end. After a frame too long C goes on with what comes as a frame's body,
/// `buf[0]` being still `fa` and so not looked for, and so does this: it decides
/// which frame comes out and what shows.
///
/// # Errors
///
/// What `read` gives, after C's message.
pub fn read_message(
    read: &mut dyn FnMut(&mut [u8]) -> Result<usize>,
    stopped: &dyn Fn() -> bool,
) -> Result<Message> {
    let mut buf = [0; MSG_MAX_LEN];
    let mut complete = false;
    let mut turn = 0;
    let mut len = 0;
    let mut one = |into: &mut u8| -> Result<bool> {
        let mut byte = [0];
        let got = read(&mut byte).inspect_err(|e| log::error!("Error {e} reading byte."))?;
        if got == 1 {
            *into = byte[0];
        }
        Ok(got == 1)
    };
    while !complete && turn < MAX_TURNS {
        len = 0;
        while buf[0] != FRAME_BEGIN && turn < MAX_TURNS {
            if stopped() {
                return Ok(Message::Stopped);
            }
            if one(&mut buf[0])? {
                if buf[0] == FRAME_BEGIN {
                    log::debug!("Message header received: {:02x}", buf[0]);
                    len += 1;
                } else {
                    log::warn!("Not frame begin byte {:02x} received", buf[0]);
                }
            }
            turn += 1;
        }
        while (len == 0 || buf[len - 1] != FRAME_END) && len < MSG_MAX_LEN && turn < MAX_TURNS {
            if stopped() {
                return Ok(Message::Stopped);
            }
            if one(&mut buf[len])? {
                if buf[len] == FRAME_BEGIN {
                    log::warn!("Frame begin before end received");
                    len = 1;
                } else {
                    log::debug!("Message data received: {:02x}", buf[len]);
                    len += 1;
                }
            }
            turn += 1;
        }
        if turn >= MAX_TURNS {
            log::warn!("Invalid data and timeout");
        } else if len > 0 && buf[len - 1] == FRAME_END {
            complete = true;
            log::debug!("Message end received");
        } else {
            log::warn!("Frame end not received");
        }
    }
    Ok(if complete && len > 2 {
        Message::Frame(buf, len)
    } else if turn >= MAX_TURNS {
        Message::Nothing
    } else {
        Message::Bad
    })
}

/// A measurement's values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measurement {
    /// VBUS's volts.
    pub voltage: f32,
    /// VBUS's amperes.
    pub current: f32,
    /// D+'s volts.
    pub voltage_dp: f32,
    /// D-'s volts.
    pub voltage_dm: f32,
    /// Whether the device says the load is on.
    pub load_on: bool,
    /// The limit and the threshold the device says it holds, in a `0a` alone. The
    /// others have other things there.
    pub reported: Option<(f32, f32)>,
}

/// What a frame turned out to be: `ebd_receive_data`'s checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Parsed {
    /// A measurement.
    Measurement(Measurement),
    /// Not 19 bytes, or another type.
    NotMeasurement,
    /// The XOR does not come to 0.
    BadChecksum,
}

/// Looks at what [`read_message`] returned, the frame being `frame[..len]`.
#[must_use]
pub fn parse(frame: &[u8; MSG_MAX_LEN], len: usize) -> Parsed {
    if len != MSG_MAX_LEN || !matches!(frame[1], 0x00 | 0x0a | 0x64 | 0x6e) {
        return Parsed::NotMeasurement;
    }
    if frame[1..len - 1].iter().fold(0, |x, b| x ^ b) != 0 {
        return Parsed::BadChecksum;
    }
    let reported = (frame[1] == 0x0a).then(|| {
        (
            decode_value(frame[10], frame[11], LIMIT_DIVISOR),
            decode_value(frame[12], frame[13], THRESHOLD_DIVISOR),
        )
    });
    Parsed::Measurement(Measurement {
        current: decode_value(frame[2], frame[3], CURRENT_DIVISOR),
        voltage: decode_value(frame[4], frame[5], VOLTAGE_DIVISOR),
        voltage_dp: decode_value(frame[6], frame[7], VOLTAGE_DIVISOR),
        voltage_dm: decode_value(frame[8], frame[9], VOLTAGE_DIVISOR),
        load_on: matches!(frame[1], 0x0a | 0x6e),
        reported,
    })
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use crate::core::Error;
    use std::cell::Cell;
    use std::collections::VecDeque;

    fn hex(text: &str) -> Vec<u8> {
        (0..text.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&text[i..i + 2], 16).unwrap())
            .collect()
    }

    /// The wiki's example of a measurement without load.
    pub(in super::super) fn without_load() -> Vec<u8> {
        hex("fa00000014c507a907ad0064000a00001aa1f8")
    }

    /// A measurement with the type, the limit in mA and the current in 0.1 mA.
    pub(in super::super) fn measurement(kind: u8, limit: i32, threshold: i32) -> Vec<u8> {
        let mut f = without_load();
        let current = limit * 10;
        f[1] = kind;
        f[2] = u8::try_from(current / 240).unwrap();
        f[3] = u8::try_from(current % 240).unwrap();
        f[10] = u8::try_from(limit / 240).unwrap();
        f[11] = u8::try_from(limit % 240).unwrap();
        f[12] = u8::try_from(threshold / 240).unwrap();
        f[13] = u8::try_from(threshold % 240).unwrap();
        f[17] = f[1..17].iter().fold(0, |x, b| x ^ b);
        f
    }

    /// Runs [`read_message`] over the script: `Some` a byte, `None` a timeout.
    fn run(script: &[Option<u8>]) -> (Message, usize) {
        let mut script: VecDeque<Option<u8>> = script.iter().copied().collect();
        let mut reads = 0;
        let message = read_message(
            &mut |buf| {
                reads += 1;
                Ok(match script.pop_front().flatten() {
                    Some(b) => {
                        buf[0] = b;
                        1
                    }
                    None => 0,
                })
            },
            &|| false,
        )
        .unwrap();
        (message, reads)
    }

    fn bytes(frame: &[u8]) -> Vec<Option<u8>> {
        frame.iter().map(|b| Some(*b)).collect()
    }

    fn frame_of(message: Message) -> Vec<u8> {
        match message {
            Message::Frame(buf, len) => buf[..len].to_vec(),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn the_commands_are_the_wikis() {
        assert_eq!(init().to_vec(), hex("fa0500000000000005f8"));
        assert_eq!(stop().to_vec(), hex("fa0600000000000006f8"));
        assert_eq!(toggle().to_vec(), hex("fa0200000000000002f8"));
        assert_eq!(load_start(0.1, 0.1).to_vec(), hex("fa010064000a00006ff8"));
        assert_eq!(config(0.1, 0.1).to_vec(), hex("fa070064000a000069f8"));
    }

    #[test]
    fn values_are_in_base_240() {
        assert_eq!(encode_value(4.0, LIMIT_DIVISOR), [16, 160]);
        assert_eq!(encode_value(21.0, THRESHOLD_DIVISOR), [8, 180]);
        assert_eq!(encode_value(0.24, LIMIT_DIVISOR), [1, 0]);
        assert_eq!(encode_value(-0.0, LIMIT_DIVISOR), [0, 0]);
        assert_eq!(decode_value(0x01, 0x0a, LIMIT_DIVISOR).to_bits(), 0.25f32.to_bits());
    }

    /// A checksum goes out as the plain XOR, as C has it, `f8` or not.
    #[test]
    fn a_commands_checksum_can_reach_the_top() {
        assert_eq!(load_start(3.824, 0.16)[8], 0xfe);
    }

    #[test]
    fn encoding_truncates_a_value_off_the_grid() {
        assert_eq!(encode_value(0.0015, LIMIT_DIVISOR), [0, 1]);
        assert_eq!(encode_value(0.0019, LIMIT_DIVISOR), [0, 1]);
    }

    /// Every step in range keeps its value through a command and a report, or
    /// a load would be sent its values again for each frame.
    #[test]
    fn steps_survive_the_round_trip() {
        for ma in 0..=4000u16 {
            let value = (f64::from(ma) * 0.001) as f32;
            let [hi, lo] = encode_value(value, LIMIT_DIVISOR);
            let back = decode_value(hi, lo, LIMIT_DIVISOR);
            let off = (i32::from(hi) * BASE + i32::from(lo)) - i32::from(ma);
            assert!(off == 0 || off == -1, "{ma}: {off}");
            if off == 0 {
                assert_eq!(back.to_bits(), value.to_bits(), "{ma}");
            }
        }
    }

    #[test]
    fn a_good_frame_comes_whole() {
        let (message, reads) = run(&bytes(&without_load()));
        assert_eq!(frame_of(message), without_load());
        assert_eq!(reads, MSG_MAX_LEN);
    }

    #[test]
    fn timeouts_between_bytes_are_waited_out() {
        let mut script = vec![None; 10];
        for b in without_load() {
            script.push(Some(b));
            script.push(None);
        }
        assert_eq!(frame_of(run(&script).0), without_load());
    }

    #[test]
    fn silence_is_200_turns() {
        assert_eq!(run(&[]), (Message::Nothing, 200));
    }

    #[test]
    fn silence_in_a_frame_counts_the_same_turns() {
        let (message, reads) = run(&bytes(&[0xfa, 0x00]));
        assert_eq!((message, reads), (Message::Nothing, 200));
    }

    #[test]
    fn an_end_on_the_last_turn_is_no_frame() {
        let mut script = vec![None; 200 - 3];
        script.extend(bytes(&[0xfa, 0x00, 0xf8]));
        assert_eq!(run(&script), (Message::Nothing, 200));
        let mut script = vec![None; 200 - 5];
        script.extend(bytes(&[0xfa, 0x00, 0x00, 0xf8]));
        assert_eq!(frame_of(run(&script).0), [0xfa, 0x00, 0x00, 0xf8]);
    }

    #[test]
    fn junk_ahead_of_a_begin_is_passed_over() {
        let mut script = bytes(&[0x55, 0xf8, 0x00]);
        script.extend(bytes(&without_load()));
        assert_eq!(frame_of(run(&script).0), without_load());
    }

    #[test]
    fn a_begin_starts_the_frame_over() {
        let mut script = bytes(&[0xfa, 0x01, 0x02]);
        script.extend(bytes(&without_load()));
        assert_eq!(frame_of(run(&script).0), without_load());
    }

    #[test]
    fn a_begin_and_an_end_alone_are_bad() {
        assert_eq!(run(&bytes(&[0xfa, 0xf8])).0, Message::Bad);
    }

    /// `fa`, a checksum of `fa` and the end: one more frame with no body.
    #[test]
    fn a_checksum_of_fa_starts_over_and_makes_a_bad_frame() {
        let mut f = without_load();
        f[17] = 0xfa;
        assert_eq!(run(&bytes(&f)), (Message::Bad, 19));
    }

    /// The `f8` that follows is the next message's junk.
    #[test]
    fn a_checksum_of_f8_ends_the_frame_early() {
        let mut f = without_load();
        f[17] = 0xf8;
        assert_eq!(frame_of(run(&bytes(&f)).0), f[..18]);
    }

    /// Past 19 bytes C reads on with no begin, so its next frame's first byte
    /// is whatever came.
    #[test]
    fn past_19_bytes_what_comes_is_a_frame_without_a_begin() {
        let mut script = bytes(&without_load()[..18]);
        script.push(Some(0x11));
        script.extend(bytes(&[0x22, 0x33, 0xf8]));
        assert_eq!(frame_of(run(&script).0), [0x22, 0x33, 0xf8]);
    }

    #[test]
    fn past_19_bytes_an_end_alone_is_bad() {
        let mut script = bytes(&without_load()[..18]);
        script.push(Some(0x11));
        script.push(Some(0xf8));
        assert_eq!(run(&script).0, Message::Bad);
    }

    #[test]
    fn past_19_bytes_a_begin_is_a_begin() {
        let mut script = bytes(&without_load()[..18]);
        script.push(Some(0x11));
        script.extend(bytes(&without_load()));
        assert_eq!(frame_of(run(&script).0), without_load());
    }

    #[test]
    fn a_long_run_ends_within_its_turns() {
        let script = vec![Some(0x11); 1000];
        assert_eq!(run(&script), (Message::Nothing, 200));
        let mut script = vec![Some(0xfa)];
        script.extend(vec![Some(0x11); 1000]);
        assert_eq!(run(&script), (Message::Nothing, 200));
    }

    #[test]
    fn a_read_error_ends_it() {
        let got = read_message(&mut |_| Err(Error::device("gone")), &|| false);
        assert!(got.is_err());
    }

    #[test]
    fn a_stop_ends_it_at_the_next_read_and_leaves_the_rest() {
        let reads = Cell::new(0);
        let message = read_message(
            &mut |_| {
                reads.set(reads.get() + 1);
                Ok(0)
            },
            &|| reads.get() >= 3,
        );
        assert_eq!(message.unwrap(), Message::Stopped);
        assert_eq!(reads.get(), 3);
        let reads = Cell::new(0);
        let message = read_message(
            &mut |buf| {
                reads.set(reads.get() + 1);
                buf[0] = 0xfa;
                Ok(1)
            },
            &|| reads.get() >= 3,
        );
        assert_eq!(message.unwrap(), Message::Stopped);
    }

    #[test]
    fn a_measurement_without_load_is_the_wikis() {
        let f: [u8; MSG_MAX_LEN] = without_load().try_into().unwrap();
        let Parsed::Measurement(m) = parse(&f, MSG_MAX_LEN) else { panic!() };
        assert_eq!(m.voltage.to_bits(), 4.997f32.to_bits());
        assert_eq!(m.current.to_bits(), 0f32.to_bits());
        assert_eq!(m.voltage_dp.to_bits(), 1.849f32.to_bits());
        assert_eq!(m.voltage_dm.to_bits(), 1.853f32.to_bits());
        assert!(!m.load_on);
        assert_eq!(m.reported, None);
    }

    #[test]
    fn a_measurement_with_load_reports_its_values() {
        let f: [u8; MSG_MAX_LEN] = measurement(0x0a, 1500, 300).try_into().unwrap();
        let Parsed::Measurement(m) = parse(&f, MSG_MAX_LEN) else { panic!() };
        assert!(m.load_on);
        assert_eq!(m.current.to_bits(), 1.5f32.to_bits());
        assert_eq!(m.reported, Some((1.5, 3.0)));
    }

    /// The wiki's frames: what sits at 10 to 15 of a `64` is no limit.
    #[test]
    fn device_frames_report_nothing() {
        let f: [u8; MSG_MAX_LEN] =
            hex("fa6e085214670795079c0150096f10a01ad3f8").try_into().unwrap();
        let Parsed::Measurement(m) = parse(&f, MSG_MAX_LEN) else { panic!() };
        assert!(m.load_on);
        assert_eq!(m.reported, None);
        assert_eq!(m.current.to_bits(), 0.2002f32.to_bits());
        let f: [u8; MSG_MAX_LEN] =
            hex("fa6400000000013b00e70150096f10a01a24f8").try_into().unwrap();
        let Parsed::Measurement(m) = parse(&f, MSG_MAX_LEN) else { panic!() };
        assert!(!m.load_on);
    }

    #[test]
    fn other_frames_are_told_apart() {
        let mut f: [u8; MSG_MAX_LEN] = without_load().try_into().unwrap();
        assert_eq!(parse(&f, 18), Parsed::NotMeasurement);
        f[1] = 0x01;
        assert_eq!(parse(&f, MSG_MAX_LEN), Parsed::NotMeasurement);
        f[1] = 0x00;
        f[17] ^= 1;
        assert_eq!(parse(&f, MSG_MAX_LEN), Parsed::BadChecksum);
    }

    #[test]
    fn the_least_current_is_a_mill_ampere() {
        assert!(current_is_0(0.0));
        assert!(current_is_0(-0.0));
        assert!(current_is_0(0.000_999));
        assert!(!current_is_0(0.001));
    }
}
