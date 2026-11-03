// Counterpart in sigrok:
//   libsigrok/src/hardware/ftdi-la/protocol.c,
//   libsigrok/src/hardware/ftdi-la/protocol.h,
//   Copyright (C) 2015 Sergey Alirzaev <zl29ah@gmail.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The acquisition of `ftdi-la`, `protocol.c`: the sample rate as a baud
//! rate, and the reads.
//!
//! An FTDI chip in asynchronous bit-bang mode with every pin an input
//! sends the host one byte per tick of its baud rate generator, the
//! levels of the eight data pins of port A. The driver asks libftdi for
//! a baud rate of `samplerate / 20` on the FT2232H and FT4232H and
//! `samplerate / 30` on the FT232R and FT232H
//! (`ftdi_chip_desc.samplerate_div`), which libftdi multiplies by 4 in
//! bit-bang mode.
//!
//! The reads are `ftdi_read_data` of 16 KiB, every one of which is a
//! logic packet of what it brought, until the sample limit, which the
//! last packet is cut to.

use crate::core::{Packet, PacketSink, Result};
use crate::ftdi::libftdi::Context;
use std::sync::atomic::{AtomicBool, Ordering};

/// `DATA_BUF_SIZE`: what one read asks libftdi for.
pub const DATA_BUF_SIZE: usize = 16 * 1024;

/// The lowest sample rate `config_set()` takes.
pub const MIN_SAMPLERATE: u64 = 3600;

/// The baud rate `ftdi_la_set_samplerate()` asks libftdi for.
pub fn baud_rate(samplerate: u64, samplerate_div: u64) -> u64 {
    samplerate / samplerate_div
}

/// How many of `n` bytes just read go out: all of them, or with a
/// sample limit what is left of it (`ftdi_la_receive_data()`). The
/// second value says whether the limit is reached.
pub fn to_send(sent: u64, n: u64, limit: u64) -> (u64, bool) {
    if limit > 0 && sent + n >= limit { (limit - sent, true) } else { (n, false) }
}

/// The acquisition loop on its own thread: reads until the sample
/// limit, a failed read or `stop`. The header goes first, the end
/// last, whatever happens.
pub fn run(ctx: &mut Context, limit: u64, sink: &PacketSink, stop: &AtomicBool) {
    sink.send(Packet::header());
    if let Err(e) = receive(ctx, limit, sink, stop) {
        log::debug!("acquisition ended: {e}");
    }
    sink.send(Packet::End);
}

fn receive(ctx: &mut Context, limit: u64, sink: &PacketSink, stop: &AtomicBool) -> Result<()> {
    let mut buf = vec![0u8; DATA_BUF_SIZE];
    let mut sent = 0u64;
    while !stop.load(Ordering::SeqCst) {
        let n = match ctx.read_data(&mut buf) {
            Ok(n) => n,
            Err(e) => {
                log::error!("Failed to read FTDI data {e}.");
                return Err(e.into());
            }
        };
        if n == 0 {
            log::trace!("Received 0 bytes, nothing to do.");
            continue;
        }
        let (count, done) = to_send(sent, n as u64, limit);
        log::trace!("Sending {count} samples.");
        if !sink.send(Packet::logic(1, buf[..count as usize].to_vec())) {
            return Ok(());
        }
        sent += count;
        if done {
            log::info!("Requested number of samples reached.");
            return Ok(());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rates_and_limits() {
        assert_eq!(baud_rate(1_000_000, 20), 50_000);
        assert_eq!(baud_rate(1_000_000, 30), 33_333);
        assert_eq!(to_send(0, 16384, 100), (100, true));
        assert_eq!(to_send(16384, 16384, 0), (16384, false));
        assert_eq!(to_send(16384, 16384, 32768), (16384, true));
        assert_eq!(to_send(16384, 16384, 40000), (16384, false));
    }
}
