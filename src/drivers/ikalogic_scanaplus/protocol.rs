// Counterpart in sigrok:
//   libsigrok/src/hardware/ikalogic-scanaplus/protocol.c,
//   libsigrok/src/hardware/ikalogic-scanaplus/protocol.h,
//   Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Bert Vermeulen, Daniel Elstner, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The protocol of the IKALOGIC `ScanaPLUS`, `protocol.c`: the FPGA's
//! register writes, the device id from the EEPROM, and the decoding of
//! the compressed sample stream.
//!
//! The `ScanaPLUS` is an FT232H in synchronous FIFO mode in front of an
//! FPGA that samples nine channels at a fixed 100 MHz. The host writes
//! (register, value) pairs, register bit 7 set:
//!
//! - `0x88` the channel configuration (`0x40` normal; `0x50`, `0x48`,
//!   `0x58` make channels 5/6, 7/8 or both differential; `0x41` is what
//!   the init writes first);
//! - `0x89`, `0x8a` the logic thresholds of channels 1-4 and 5-9
//!   ([`THRESHOLD_3_3V`] and the others: half the system voltage);
//! - `0x8d` a sequence the init writes: `01 05 01 02`, then 57 times
//!   `06 02`;
//! - `0x8c`, `0x8e`, `0x8f` the device id: the three bytes in EEPROM
//!   words 16 and 17, bit 7 of each cleared.
//!
//! An acquisition is the init ([`init_writes`]) and the start
//! ([`start_writes`]: thresholds, configuration, the id cleared and
//! sent again), each write its own bulk transfer; then the FPGA streams
//! pairs of bytes: a count of samples in bits 1 to 7 and channel 9 in
//! bit 0, then channels 1 to 8. A sample goes out as two bytes, channels
//! 1 to 8 then channel 9.
//!
//! The reads are `ftdi_read_data` of 64 KiB, which libftdi makes of
//! transfers of 16 KiB on Linux. The FPGA sends dummy zeros for a while
//! after a start, so the first 64 KiB of the stream are dropped. Where
//! the C driver differs (see `docs/DEVICES.md`), this one does the sane
//! thing:
//!
//! - it drops the first 64 KiB of data, counted in bytes, where the C
//!   driver drops the first read whatever its length (`protocol.c`
//!   345-350: `compressed_bytes_ignored += COMPRESSED_BUF_SIZE`), so a
//!   short first read made it decode the dummy samples that follow;
//! - it keeps the odd byte of a read of odd length for the next one,
//!   where the C driver decodes the whole buffer in pairs (the TODO at
//!   `protocol.c` 352) and so takes a stale byte past the end, and
//!   splits every pair after it;
//! - with both limits it ends on whichever is reached first, where the
//!   C driver tests the sample limit first (`protocol.c` 358-367) and
//!   so ends a read that passes both on the sample limit even when the
//!   time limit is the smaller;
//! - the time limit in samples saturates rather than wrapping
//!   (`(SR_MHZ(100) / 1000) * limit_msec`, `protocol.c` 356).

use crate::core::{Packet, PacketSink};
use crate::ftdi::libftdi::{Context, FtdiError};
use std::sync::atomic::{AtomicBool, Ordering};

/// The one sample rate.
pub const SAMPLERATE: u64 = 100_000_000;
/// `COMPRESSED_BUF_SIZE`: what one read asks for, and how much of the
/// stream is dropped at the start.
pub const COMPRESSED_BUF_SIZE: usize = 64 * 1024;

/// Threshold of a 1.2 V system: 0.6 V.
pub const THRESHOLD_1_2V: u8 = 0x2e;
/// Threshold of a 1.5 V system: 0.75 V.
pub const THRESHOLD_1_5V: u8 = 0x39;
/// Threshold of a 1.8 V system: 0.9 V.
pub const THRESHOLD_1_8V: u8 = 0x45;
/// Threshold of a 2.8 V system: 1.4 V.
pub const THRESHOLD_2_8V: u8 = 0x6c;
/// Threshold of a 3.3 V system: 1.65 V, what an acquisition sets.
pub const THRESHOLD_3_3V: u8 = 0x7f;

/// The device id the FPGA is sent: EEPROM word 16's low and high byte
/// and word 17's low byte, bit 7 of each cleared
/// (`scanaplus_get_device_id()`).
pub fn device_id(word16: u16, word17: u16) -> [u8; 3] {
    let [a, b] = word16.to_le_bytes();
    [a & 0x7f, b & 0x7f, word17.to_le_bytes()[0] & 0x7f]
}

/// The three writes that set the device id.
fn id_writes(id: [u8; 3]) -> [Vec<u8>; 3] {
    [vec![0x8c, id[0]], vec![0x8e, id[1]], vec![0x8f, id[2]]]
}

/// The writes of `scanaplus_init()`, in order, one bulk transfer each.
pub fn init_writes(id: [u8; 3]) -> Vec<Vec<u8>> {
    let mut w = vec![
        vec![0x88, 0x41],
        vec![0x89, 0x64, 0x8a, 0x64],
        vec![0x88, 0x41],
        vec![0x88, 0x40],
        vec![0x8d, 0x01, 0x8d, 0x05, 0x8d, 0x01, 0x8d, 0x02],
    ];
    for _ in 0..57 {
        w.push(vec![0x8d, 0x06]);
        w.push(vec![0x8d, 0x02]);
    }
    w.extend(id_writes(id));
    w.push(vec![0x88, 0x40]);
    w
}

/// The writes of `scanaplus_start_acquisition()`: both thresholds for
/// 3.3 V, the normal channel configuration, the id cleared and sent
/// again.
pub fn start_writes(id: [u8; 3]) -> Vec<Vec<u8>> {
    let mut w = vec![vec![0x89, THRESHOLD_3_3V, 0x8a, THRESHOLD_3_3V], vec![0x88, 0x40]];
    w.extend(id_writes([0; 3]));
    w.extend(id_writes(id));
    w
}

/// `scanaplus_write()`: one write, logged as the C driver logs it.
///
/// # Errors
///
/// libftdi's, logged.
pub fn write(ctx: &mut Context, data: &[u8]) -> std::result::Result<(), FtdiError> {
    use std::fmt::Write as _;
    let mut s = format!("Writing {} bytes: ", data.len());
    for b in data {
        let _ = write!(s, "0x{b:02x} ");
    }
    log::trace!("{s}");
    ctx.write_data(data).map(|_| ()).inspect_err(|e| {
        log::error!("Failed to write FTDI data ({}): {}.", e.code, e.message);
    })
}

/// The stream decoder: drops the first [`COMPRESSED_BUF_SIZE`] bytes,
/// then turns (count and channel 9, channels 1 to 8) pairs into samples
/// of two bytes.
#[derive(Debug, Default)]
pub struct Decoder {
    /// Stream bytes dropped so far.
    ignored: usize,
    /// The first byte of a pair whose second is in the next read.
    odd: Option<u8>,
}

impl Decoder {
    /// A decoder at the start of an acquisition.
    pub fn new() -> Self {
        Self::default()
    }

    /// Decodes the bytes of one read, appending its samples to `out`.
    pub fn feed(&mut self, mut data: &[u8], out: &mut Vec<u8>) {
        if self.ignored < COMPRESSED_BUF_SIZE {
            let skip = (COMPRESSED_BUF_SIZE - self.ignored).min(data.len());
            log::trace!("Ignoring {skip} bytes of the first 64kB of data.");
            self.ignored += skip;
            data = &data[skip..];
        }
        if let (Some(first), Some(&second)) = (self.odd, data.first()) {
            self.odd = None;
            push_pair(first, second, out);
            data = &data[1..];
        }
        let (pairs, rest) = data.as_chunks::<2>();
        for &[first, second] in pairs {
            push_pair(first, second, out);
        }
        if let Some(&b) = rest.first() {
            self.odd = Some(b);
        }
    }
}

fn push_pair(first: u8, second: u8, out: &mut Vec<u8>) {
    let low = first & 1;
    for _ in 0..first >> 1 {
        out.push(second);
        out.push(low);
    }
}

/// Why an acquisition ends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Limit {
    /// The sample limit.
    Samples,
    /// The time limit.
    Time,
}

/// The limits in samples: the sample limit and the time limit at 100
/// MHz (0 for none), the smaller one first when both are set.
pub fn limits(limit_samples: u64, limit_msec: u64) -> Option<(u64, Limit)> {
    let time =
        (limit_msec > 0).then(|| ((SAMPLERATE / 1000).saturating_mul(limit_msec), Limit::Time));
    let samples = (limit_samples > 0).then_some((limit_samples, Limit::Samples));
    match (samples, time) {
        (Some(s), Some(t)) => Some(if t.0 < s.0 { t } else { s }),
        (s, t) => s.or(t),
    }
}

/// How many of `available` new samples go out after `sent`, and the
/// limit that is reached with them, if any.
pub fn to_send(sent: u64, available: u64, limit: Option<(u64, Limit)>) -> (u64, Option<Limit>) {
    match limit {
        Some((n, why)) if sent + available >= n => (n - sent, Some(why)),
        _ => (available, None),
    }
}

/// The acquisition loop on its own thread, after the start's writes:
/// the header, reads until a limit, a failed read or `stop`, the end.
pub fn run(
    ctx: &mut Context,
    limit: Option<(u64, Limit)>,
    sink: &PacketSink,
    stop: &AtomicBool,
) -> std::result::Result<(), FtdiError> {
    sink.send(Packet::header());
    let r = receive(ctx, limit, sink, stop);
    sink.send(Packet::End);
    r
}

fn receive(
    ctx: &mut Context,
    limit: Option<(u64, Limit)>,
    sink: &PacketSink,
    stop: &AtomicBool,
) -> std::result::Result<(), FtdiError> {
    let mut buf = vec![0u8; COMPRESSED_BUF_SIZE];
    let mut decoder = Decoder::new();
    let mut samples = Vec::new();
    let mut sent = 0u64;
    while !stop.load(Ordering::SeqCst) {
        let n = ctx.read_data(&mut buf).inspect_err(|e| {
            log::error!("Failed to read FTDI data ({}): {}.", e.code, e.message);
        })?;
        if n == 0 {
            log::trace!("Received 0 bytes, nothing to do.");
            continue;
        }
        samples.clear();
        decoder.feed(&buf[..n], &mut samples);
        let (count, reached) = to_send(sent, samples.len() as u64 / 2, limit);
        if count > 0 {
            log::trace!("Sending {count} samples.");
            let bytes = usize::try_from(count * 2).expect("at most one read's samples");
            if !sink.send(Packet::logic(2, samples[..bytes].to_vec())) {
                return Ok(());
            }
            sent += count;
        }
        match reached {
            Some(Limit::Samples) => {
                log::info!("Requested number of samples reached.");
                return Ok(());
            }
            Some(Limit::Time) => {
                log::info!("Requested time limit reached.");
                return Ok(());
            }
            None => {}
        }
    }
    Ok(())
}

/// Sends the writes in order, stopping at the first that fails.
///
/// # Errors
///
/// The failing write's.
pub fn write_all(ctx: &mut Context, writes: &[Vec<u8>]) -> std::result::Result<(), FtdiError> {
    for w in writes {
        write(ctx, w)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn device_ids_lose_bit_7() {
        assert_eq!(device_id(0xa31c, 0x0057), [0x1c, 0x23, 0x57]);
        assert_eq!(device_id(0xd5ff, 0xffaa), [0x7f, 0x55, 0x2a]);
    }

    #[test]
    fn writes_as_the_c_driver_sends_them() {
        let id = [0x1c, 0x23, 0x57];
        let init = init_writes(id);
        assert_eq!(init.len(), 5 + 114 + 3 + 1);
        assert_eq!(init[4], [0x8d, 0x01, 0x8d, 0x05, 0x8d, 0x01, 0x8d, 0x02]);
        assert_eq!(init[5], [0x8d, 0x06]);
        assert_eq!(init[118], [0x8d, 0x02]);
        assert_eq!(
            &init[119..],
            [vec![0x8c, 0x1c], vec![0x8e, 0x23], vec![0x8f, 0x57], vec![0x88, 0x40]]
        );
        let start = start_writes(id);
        assert_eq!(start[0], [0x89, 0x7f, 0x8a, 0x7f]);
        assert_eq!(start[1], [0x88, 0x40]);
        assert_eq!(start[2..5], [vec![0x8c, 0], vec![0x8e, 0], vec![0x8f, 0]]);
        assert_eq!(start[5..], id_writes(id));
    }

    #[test]
    fn decodes_after_the_first_64_kib() {
        let mut d = Decoder::new();
        let mut out = Vec::new();
        // A short first read is dropped, and the rest of the 64 KiB.
        d.feed(&[0xff; 100], &mut out);
        assert_eq!(out, Vec::<u8>::new());
        let mut read = vec![0u8; COMPRESSED_BUF_SIZE - 100];
        // Three samples of 0xa5 with channel 9 high, one of 0x01.
        read.extend([0x07, 0xa5, 0x02, 0x01]);
        d.feed(&read, &mut out);
        assert_eq!(out, [0xa5, 1, 0xa5, 1, 0xa5, 1, 0x01, 0]);
        // A count of 0 gives nothing; 127 the most.
        out.clear();
        d.feed(&[0x00, 0x33, 0xfe, 0x44], &mut out);
        assert_eq!(out.len(), 254);
        assert!(out.chunks(2).all(|s| s == [0x44, 0]));
    }

    #[test]
    fn carries_an_odd_byte_over() {
        let mut d = Decoder { ignored: COMPRESSED_BUF_SIZE, odd: None };
        let mut out = Vec::new();
        d.feed(&[0x02, 0x11, 0x05], &mut out);
        assert_eq!(out, [0x11, 0]);
        out.clear();
        d.feed(&[0x22, 0x04, 0x33], &mut out);
        assert_eq!(out, [0x22, 1, 0x22, 1, 0x33, 0, 0x33, 0]);
        // An odd byte at the end of the dropped part.
        let mut d = Decoder::new();
        out.clear();
        d.feed(&vec![0; COMPRESSED_BUF_SIZE - 1], &mut out);
        d.feed(&[0, 0x02, 0x99, 0x02], &mut out);
        assert_eq!(out, [0x99, 0]);
        assert_eq!(d.odd, Some(0x02));
    }

    #[test]
    fn the_first_limit_reached_ends() {
        assert_eq!(limits(0, 0), None);
        assert_eq!(limits(64, 0), Some((64, Limit::Samples)));
        assert_eq!(limits(0, 1), Some((100_000, Limit::Time)));
        // The C driver would end on 300000 samples.
        assert_eq!(limits(300_000, 1), Some((100_000, Limit::Time)));
        assert_eq!(limits(64, 1), Some((64, Limit::Samples)));
        assert_eq!(limits(100_000, 1), Some((100_000, Limit::Samples)));
        assert_eq!(limits(0, u64::MAX), Some((u64::MAX, Limit::Time)));
        let l = limits(100, 0);
        assert_eq!(to_send(0, 40, l), (40, None));
        assert_eq!(to_send(40, 80, l), (60, Some(Limit::Samples)));
        assert_eq!(to_send(0, 1000, None), (1000, None));
    }
}
