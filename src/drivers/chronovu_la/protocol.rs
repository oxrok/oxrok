// Counterpart in sigrok:
//   libsigrok/src/hardware/chronovu-la/protocol.c,
//   libsigrok/src/hardware/chronovu-la/protocol.h,
//   Copyright (C) 2011-2014 Uwe Hermann <uwe@hermann-uwe.de>,
//   with commits by Gerhard Sittig, Bert Vermeulen, Daniel Elstner
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The protocol of the `ChronoVu` LA8 and LA16, `protocol.c` and
//! `protocol.h`: an FTDI chip in FIFO mode (driven through
//! [`crate::ftdi::libftdi`]) in front of the analyzer's logic and 8 MiB
//! of SDRAM.
//!
//! - The command, one write: on the LA8 four bytes, divcount, 0xff,
//!   trigger pattern, trigger mask; on the LA16 eight, divcount, 0xff,
//!   then pattern, mask and edge mask as 16 bit values, channels 8 to
//!   15 first. divcount is 0 to 0xfe: the sample period is
//!   `(divcount + 1) * 10 ns` on the LA8 and `* 5 ns` on the LA16.
//! - The device samples, waits for the trigger and fills its memory;
//!   until it is full the chip answers status-only packets, reads of 0
//!   bytes. Then the whole memory comes out, 2048 blocks of 4096 bytes,
//!   in eight sections of 1 MiB that interleave the samples
//!   ([`demangle_index`]); on the LA16 the two bytes of a sample come
//!   high byte first.
//! - Eight bytes of 0x01 reset the sequencer, after a failure, before
//!   the FTDI chip is purged, reset and closed.
//!
//! What the port does differently from the C driver, each a libsigrok
//! bug (`docs/DEVICES.md`):
//!
//! - The trigger is searched in whole samples after the LA16 byte swap,
//!   with the edge conditions too ([`find_trigger`]). The C driver
//!   compares single bytes before the swap with the 16 bit mask and an
//!   8 bit expected value (`protocol.c:425-445`), so channel 0 on the
//!   LA16 matches a byte into the sample, channels 8 to 15 match the
//!   first byte whatever the data, and an edge-only trigger, whose
//!   state mask is 0, gives no trigger at all (`protocol.c:436`).
//! - A trigger on the last sample of a block keeps the samples after
//!   it: the C driver sends the post-trigger packet only when the
//!   trigger is before the block's last byte (`protocol.c:500`), and
//!   drops that sample.
//! - A failed write is logged and the reset carries on. In C,
//!   `cv_write()` resets the sequencer on a failed write, whose own
//!   write fails again and recurses until the stack overflows
//!   (`protocol.c:137-145`, `198`).
//! - The wait for the trigger and the full memory is sized from the
//!   rate and counted in reads ([`deadline_polls`]). The C deadline adds
//!   the capture time in seconds to a microsecond clock
//!   (`api.c:491-492`): 10 s whatever the rate, shorter than a full
//!   memory takes below about 833 kHz on the LA8. The purge before a
//!   reset is counted in reads too ([`DRAIN_READS`]) rather than 20 s.
//! - The sample limit (or time limit) is honoured: the device always
//!   fills and sends its whole memory, and the port reads all of it
//!   (the samples are spread over the whole readout) but sends no more
//!   than asked. The C driver stores `limit_samples` and `limit_msec`
//!   and never reads them (`api.c:337-342`).
//! - "only N/M bytes written" names the written count first; the C
//!   message swaps them (`protocol.c:142-143`).

use crate::core::{Packet, PacketSink, Trigger, TriggerMatch};
use crate::ftdi::libftdi::{Context, FtdiError};
use std::ops::Range;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

/// `SDRAM_SIZE`: the sample memory, in bytes.
pub const SDRAM_SIZE: usize = 8 * 1024 * 1024;
/// `BS`: one block of the readout, what one read asks for.
pub const BS: usize = 4096;
/// `NUM_BLOCKS`: the blocks of the readout.
pub const NUM_BLOCKS: usize = SDRAM_SIZE / BS;
/// One de-mangling section.
const SECTION: usize = 1024 * 1024;
/// The sequencer reset (`close_usb_reset_sequencer`).
pub const RESET_SEQUENCER: [u8; 8] = [0x01; 8];
/// libftdi's `SIO_RTS_CTS_HS`, the flow control `dev_open` enables.
pub const SIO_RTS_CTS_HS: u16 = 0x0100;
/// How long the C driver waits for the trigger beyond the capture
/// time (`api.c:492`, meant as 10 s).
pub const TRIGGER_WAIT_US: u64 = 10_000_000;
/// How long a status-only read takes on the device: the FTDI chip
/// answers one when its latency timer runs out, 16 ms after a reset
/// (the driver never sets it).
pub const LATENCY_US: u64 = 16_000;
/// The reads of the purge before a sequencer reset: enough for the
/// whole memory and what a read left, then one that brings nothing.
pub const DRAIN_READS: usize = NUM_BLOCKS + 2;

/// The two models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    /// 8 channels, 100 MHz, state triggers.
    La8,
    /// 16 channels, 200 MHz, state and edge triggers.
    La16,
}

/// `struct cv_profile`.
#[derive(Debug, PartialEq, Eq)]
pub struct Profile {
    /// Which one.
    pub model: Model,
    /// The model name.
    pub name: &'static str,
    /// The product string the scan and the open look for.
    pub iproduct: &'static str,
    /// The channels, named "0" up.
    pub num_channels: usize,
    /// The highest rate, divcount 0.
    pub max_samplerate: u64,
    /// The trigger matches (`num_trigger_matches` of the C list).
    pub trigger_matches: &'static [TriggerMatch],
}

const MATCHES: [TriggerMatch; 4] =
    [TriggerMatch::Zero, TriggerMatch::One, TriggerMatch::Rising, TriggerMatch::Falling];

/// `cv_profiles`.
pub static PROFILES: [Profile; 2] = [
    Profile {
        model: Model::La8,
        name: "LA8",
        iproduct: "ChronoVu LA8",
        num_channels: 8,
        max_samplerate: 100_000_000,
        trigger_matches: MATCHES.split_at(2).0,
    },
    Profile {
        model: Model::La16,
        name: "LA16",
        iproduct: "ChronoVu LA16",
        num_channels: 16,
        max_samplerate: 200_000_000,
        trigger_matches: &MATCHES,
    },
];

impl Profile {
    /// The profile of a product string.
    pub fn of_product(product: &str) -> Option<&'static Profile> {
        PROFILES.iter().find(|p| p.iproduct == product)
    }

    /// Bytes per sample.
    pub fn unit_size(&self) -> usize {
        self.num_channels / 8
    }

    /// The samples the memory holds (`MAX_NUM_SAMPLES`, halved on the
    /// LA16).
    pub fn max_samples(&self) -> u64 {
        (SDRAM_SIZE / self.unit_size()) as u64
    }

    /// `cv_fill_samplerates_if_needed`: the highest rate divided by 255
    /// down to 1, slowest first.
    pub fn samplerates(&self) -> Vec<u64> {
        (1..=255).rev().map(|d| self.max_samplerate / d).collect()
    }

    /// `cv_samplerate_to_divcount`: `None` for a rate not in the list.
    pub fn divcount(&self, samplerate: u64) -> Option<u8> {
        if samplerate == 0 || !self.samplerates().contains(&samplerate) {
            return None;
        }
        u8::try_from(self.max_samplerate / samplerate - 1).ok()
    }

    /// How many reads to wait for the memory to fill: the time a full
    /// memory takes at `divcount`, plus the C driver's 10 s for the
    /// trigger, in status-only reads of [`LATENCY_US`] each. Counted in
    /// reads rather than wall time, as the device paces them.
    pub fn deadline_polls(&self, divcount: u8) -> u64 {
        let capture_us =
            self.max_samples() * (u64::from(divcount) + 1) * 1_000_000 / self.max_samplerate;
        (TRIGGER_WAIT_US + capture_us).div_ceil(LATENCY_US)
    }
}

/// The trigger the device is given (`trigger_pattern`, `trigger_mask`,
/// `trigger_edgemask`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TriggerMasks {
    /// The levels asked for; for an edge 1 is rising.
    pub pattern: u16,
    /// Channels that must be at the pattern's level.
    pub mask: u16,
    /// Channels that must see an edge (LA16).
    pub edges: u16,
}

/// `cv_convert_trigger`: the masks of a one stage trigger; conditions
/// on disabled channels are ignored. `Err` is the message the C driver
/// logs. An `e` (either edge) or analog condition, which the C driver
/// silently drops, is refused.
///
/// # Errors
///
/// More than one stage, an edge on the LA8, a match the device lacks.
pub fn convert_trigger(
    prof: &Profile,
    trigger: Option<&Trigger>,
    enabled: &[bool],
) -> std::result::Result<TriggerMasks, &'static str> {
    let mut t = TriggerMasks::default();
    let Some(trigger) = trigger else { return Ok(t) };
    if trigger.stages.len() > 1 {
        return Err("This device only supports 1 trigger stage.");
    }
    for c in trigger.stages.iter().flat_map(|s| &s.conditions) {
        if !enabled.get(c.channel).copied().unwrap_or(false) {
            continue;
        }
        if !prof.trigger_matches.contains(&c.matches) {
            return Err(if c.matches.is_edge() && prof.model == Model::La8 {
                "This model supports only simple triggers."
            } else {
                "Trigger match not supported by this device."
            });
        }
        let bit = 1u16 << c.channel;
        match c.matches {
            TriggerMatch::One => {
                t.pattern |= bit;
                t.mask |= bit;
            }
            TriggerMatch::Zero => t.mask |= bit,
            TriggerMatch::Rising => {
                t.pattern |= bit;
                t.edges |= bit;
            }
            _ => t.edges |= bit,
        }
    }
    Ok(t)
}

/// The command that starts a capture (`dev_acquisition_start`).
pub fn command(model: Model, divcount: u8, t: TriggerMasks) -> Vec<u8> {
    match model {
        // Only channels 0 to 7 exist.
        Model::La8 => vec![divcount, 0xff, t.pattern as u8, t.mask as u8],
        Model::La16 => {
            let [p1, p0] = t.pattern.to_be_bytes();
            let [m1, m0] = t.mask.to_be_bytes();
            let [e1, e0] = t.edges.to_be_bytes();
            vec![divcount, 0xff, p1, p0, m1, m0, e1, e0]
        }
    }
}

/// `cv_read_block`'s de-mangling: where byte `offset` of the readout
/// goes in the memory image, before the LA16 byte swap.
pub fn demangle_index(model: Model, divcount: u8, offset: usize) -> usize {
    let m = offset / SECTION;
    let within = offset % SECTION;
    let p = offset & 1;
    match model {
        Model::La8 => 2 * m + (within / 2) * 16 + if divcount == 0 { p } else { 1 - p },
        Model::La16 => 4 * m + (within / 4) * 32 + (offset & 2) + (1 - p),
    }
}

/// The LA16's byte swap (`cv_send_block_to_session_bus`): high byte
/// first from the device, little endian samples after.
pub fn swap_pairs(data: &mut [u8]) {
    for pair in data.as_chunks_mut::<2>().0 {
        pair.swap(0, 1);
    }
}

fn sample(data: &[u8], unit: usize, i: usize) -> u16 {
    if unit == 1 { u16::from(data[i]) } else { u16::from_le_bytes([data[2 * i], data[2 * i + 1]]) }
}

/// The first sample that meets the trigger, in whole samples of
/// `unit` bytes: every masked channel at its level and, for edge
/// channels, at the edge's new level after a change from the sample
/// before. The device stores from the sample that met its trigger (or
/// some before it), so on sample 0, which has none before it, an edge
/// channel at its new level counts as the edge. `None` without a
/// trigger or a match.
pub fn find_trigger(data: &[u8], unit: usize, t: TriggerMasks) -> Option<usize> {
    if t.mask | t.edges == 0 {
        return None;
    }
    let want = t.mask | t.edges;
    let mut prev: Option<u16> = None;
    for i in 0..data.len() / unit {
        let cur = sample(data, unit, i);
        if (cur ^ t.pattern) & want == 0 && prev.is_none_or(|p| (p ^ cur) & t.edges == t.edges) {
            return Some(i);
        }
        prev = Some(cur);
    }
    None
}

/// What to send, in order: logic packets of sample ranges (at most one
/// block each, as the C driver sends them) and the trigger.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Chunk {
    /// The samples in this range.
    Logic(Range<usize>),
    /// The trigger, before the sample that met it.
    Trigger,
}

/// The packets for `total` samples of `unit` bytes, a trigger at
/// sample `trigger`, and no more than `limit` samples (0: all).
pub fn chunks(total: usize, unit: usize, trigger: Option<usize>, limit: u64) -> Vec<Chunk> {
    let end = match usize::try_from(limit) {
        Ok(l) if l > 0 => l.min(total),
        _ => total,
    };
    let per_block = BS / unit;
    let mut out = Vec::new();
    let mut start = 0;
    while start < end {
        let stop = (start + per_block).min(end);
        match trigger {
            Some(t) if (start..stop).contains(&t) => {
                if t > start {
                    out.push(Chunk::Logic(start..t));
                }
                out.push(Chunk::Trigger);
                out.push(Chunk::Logic(t..stop));
            }
            _ => out.push(Chunk::Logic(start..stop)),
        }
        start = stop;
    }
    out
}

/// The sample limit a capture honours: the smaller of `limit_samples`
/// and what `limit_msec` gives at `samplerate`, each 0 for none.
pub fn sample_limit(limit_samples: u64, limit_msec: u64, samplerate: u64) -> u64 {
    let by_time = if limit_msec > 0 { (limit_msec * samplerate / 1000).max(1) } else { 0 };
    match (limit_samples, by_time) {
        (0, t) => t,
        (s, 0) => s,
        (s, t) => s.min(t),
    }
}

/// The FTDI chip, with what `ftdi_get_error_string` would give: the
/// message of the last libftdi error, "all fine" before any.
pub struct Ftdi {
    /// The chip.
    pub ctx: Context,
    /// libftdi's `error_str`.
    pub last_error: &'static str,
}

impl Ftdi {
    /// A chip just opened.
    pub fn new(ctx: Context) -> Self {
        Ftdi { ctx, last_error: "all fine" }
    }

    /// Keeps the message of `e` as libftdi does.
    pub fn note(&mut self, e: FtdiError) -> FtdiError {
        self.last_error = e.message;
        e
    }

    /// `cv_write` without its sequencer reset: the bytes written, or
    /// `None` after logging the failure.
    pub fn write(&mut self, buf: &[u8]) -> Option<usize> {
        match self.ctx.write_data(buf) {
            Ok(n) if n == buf.len() => Some(n),
            Ok(n) => {
                log::error!("Failed to write data, only {n}/{} bytes written.", buf.len());
                Some(n)
            }
            Err(e) => {
                let e = self.note(e);
                log::error!("Failed to write data ({}): {}.", e.code, e.message);
                None
            }
        }
    }

    /// `cv_read`: the bytes read, or libftdi's negative code after
    /// logging it.
    pub fn read(&mut self, buf: &mut [u8]) -> i64 {
        match self.ctx.read_data(buf) {
            Ok(n) => n as i64,
            Err(e) => {
                let e = self.note(e);
                log::error!("Failed to read data ({}): {}.", e.code, e.message);
                i64::from(e.code)
            }
        }
    }

    /// `ftdi_usb_close`, logged as `dev_close` and
    /// `close_usb_reset_sequencer` log it.
    pub fn close(self) {
        let last = self.last_error;
        if let Err(e) = self.ctx.close() {
            log::debug!("release port A: {e}");
            log::error!("Failed to close FTDI device (-1): {last}.");
        }
    }
}

/// `close_usb_reset_sequencer`: the sequencer reset, 100 ms, then the
/// chip purged, reset and closed, each failure logged and passed over.
/// A failed write of the reset does not start another reset.
pub fn close_usb_reset_sequencer(mut ftdi: Ftdi) {
    log::debug!("Resetting sequencer logic.");
    let _ = ftdi.write(&RESET_SEQUENCER);
    std::thread::sleep(Duration::from_millis(100));
    log::debug!("Purging buffers, resetting+closing FTDI device.");
    if let Err(e) = ftdi.ctx.tcioflush() {
        let e = ftdi.note(e);
        log::error!("Failed to purge FTDI buffers ({}): {}.", e.code, e.message);
    }
    if let Err(e) = ftdi.ctx.reset() {
        let e = ftdi.note(e);
        log::error!("Failed to reset FTDI device ({}): {}.", e.code, e.message);
    }
    ftdi.close();
}

/// `reset_device`: reads until the chip has nothing left (at most
/// [`DRAIN_READS`]), then the sequencer reset and the close.
pub fn reset_device(mut ftdi: Ftdi) {
    log::debug!("Resetting the device.");
    let mut buf = vec![0u8; BS];
    for _ in 0..DRAIN_READS {
        if ftdi.read(&mut buf) <= 0 {
            break;
        }
    }
    close_usb_reset_sequencer(ftdi);
    log::debug!("Device reset finished.");
}

/// One capture's settings.
#[derive(Clone, Copy, Debug)]
pub struct Capture {
    /// The model.
    pub prof: &'static Profile,
    /// The clock divider minus 1.
    pub divcount: u8,
    /// The trigger sent.
    pub trigger: TriggerMasks,
    /// The samples to send, 0 for all.
    pub limit: u64,
}

/// The acquisition after the command, on its own thread: the header,
/// the readout, the packets, the end. Gives the chip back, or `None`
/// once it was reset and closed (a failed read, a stop).
pub fn run(mut ftdi: Ftdi, cap: Capture, sink: &PacketSink, stop: &AtomicBool) -> Option<Ftdi> {
    sink.send(Packet::header());
    let mut image = vec![0u8; SDRAM_SIZE];
    let mut buf = vec![0u8; BS];
    let deadline = cap.prof.deadline_polls(cap.divcount);
    for block in 0..NUM_BLOCKS {
        if stop.load(Ordering::SeqCst) {
            log::debug!("Stopped in block {block}.");
            close_usb_reset_sequencer(ftdi);
            sink.send(Packet::End);
            return None;
        }
        log::trace!("Reading block {block}.");
        let mut got = ftdi.read(&mut buf);
        if got == 0 && block == 0 {
            // Until the memory is full: status-only reads.
            let mut polls = 0;
            while got == 0 && polls < deadline && !stop.load(Ordering::SeqCst) {
                log::trace!("Reading block 0 (again).");
                got = ftdi.read(&mut buf);
                polls += 1;
            }
        }
        if got != BS as i64 {
            log::error!("Trigger timed out. Bytes read: {got}.");
            reset_device(ftdi);
            log::error!("Failed to read data block: -1.");
            sink.send(Packet::End);
            return None;
        }
        let offset = block * BS;
        for (i, &b) in buf.iter().enumerate() {
            image[demangle_index(cap.prof.model, cap.divcount, offset + i)] = b;
        }
    }
    log::debug!("Sampling finished, sending data to session bus now.");
    let unit = cap.prof.unit_size();
    if unit == 2 {
        swap_pairs(&mut image);
    }
    let trigger = find_trigger(&image, unit, cap.trigger);
    if let Some(t) = trigger {
        log::debug!("Trigger at sample {t}.");
    }
    for chunk in chunks(image.len() / unit, unit, trigger, cap.limit) {
        let packet = match chunk {
            Chunk::Logic(r) => Packet::logic(unit, image[r.start * unit..r.end * unit].to_vec()),
            Chunk::Trigger => Packet::Trigger,
        };
        if !sink.send(packet) {
            break;
        }
    }
    sink.send(Packet::End);
    Some(ftdi)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn la8() -> &'static Profile {
        &PROFILES[0]
    }

    fn la16() -> &'static Profile {
        &PROFILES[1]
    }

    #[test]
    fn rates_and_divcounts() {
        let r = la8().samplerates();
        assert_eq!(r.len(), 255);
        assert_eq!((r[0], r[254]), (392_156, 100_000_000));
        assert_eq!(la16().samplerates()[0], 784_313);
        assert_eq!(la8().divcount(100_000_000), Some(0));
        assert_eq!(la8().divcount(1_000_000), Some(99));
        assert_eq!(la8().divcount(392_156), Some(254));
        assert_eq!(la8().divcount(3_000_000), None);
        assert_eq!(la8().divcount(0), None);
        assert_eq!(la16().divcount(200_000_000), Some(0));
        assert_eq!(la16().divcount(784_313), Some(254));
        assert_eq!(la8().max_samples(), 8_388_608);
        assert_eq!(la16().max_samples(), 4_194_304);
    }

    #[test]
    fn deadlines_follow_the_rate() {
        // 10 s and 84 ms at 100 MHz, in 16 ms reads.
        assert_eq!(la8().deadline_polls(0), 631);
        // The slowest LA8 rate: 21.4 s of capture on top.
        assert_eq!(la8().deadline_polls(254), (10_000_000 + 21_390_950u64).div_ceil(16_000));
        assert_eq!(la16().deadline_polls(0), (10_000_000 + 20_971u64).div_ceil(16_000));
    }

    fn trigger(stages: &[&[(usize, TriggerMatch)]]) -> Trigger {
        let mut t = Trigger::new();
        for (s, conds) in stages.iter().enumerate() {
            for &(c, m) in *conds {
                t.add(s, c, m);
            }
        }
        t
    }

    #[test]
    fn triggers_convert_as_cv_convert_trigger() {
        use TriggerMatch::{Edge, Falling, One, Rising, Zero};
        let all = [true; 16];
        assert_eq!(convert_trigger(la8(), None, &all), Ok(TriggerMasks::default()));
        let t = trigger(&[&[(0, Zero), (1, One)]]);
        assert_eq!(
            convert_trigger(la8(), Some(&t), &all),
            Ok(TriggerMasks { pattern: 2, mask: 3, edges: 0 })
        );
        // A disabled channel's condition is ignored.
        let mut some = [false; 16];
        some[1] = true;
        assert_eq!(
            convert_trigger(la8(), Some(&t), &some),
            Ok(TriggerMasks { pattern: 2, mask: 2, edges: 0 })
        );
        let t = trigger(&[&[(1, Rising), (0, Zero), (9, Falling)]]);
        assert_eq!(
            convert_trigger(la16(), Some(&t), &all),
            Ok(TriggerMasks { pattern: 2, mask: 1, edges: 0x202 })
        );
        assert_eq!(
            convert_trigger(la8(), Some(&t), &all),
            Err("This model supports only simple triggers.")
        );
        let t = trigger(&[&[(0, Edge)]]);
        assert!(convert_trigger(la16(), Some(&t), &all).is_err());
        let t = trigger(&[&[(0, One)], &[(1, One)]]);
        assert_eq!(
            convert_trigger(la16(), Some(&t), &all),
            Err("This device only supports 1 trigger stage.")
        );
    }

    #[test]
    fn commands_as_dev_acquisition_start_builds_them() {
        let t = TriggerMasks { pattern: 0x0201, mask: 0x0001, edges: 0x0200 };
        assert_eq!(command(Model::La8, 99, t), [99, 0xff, 0x01, 0x01]);
        assert_eq!(command(Model::La16, 0, t), [0, 0xff, 0x02, 0x01, 0x00, 0x01, 0x02, 0x00]);
    }

    /// The readout of `samples` (a function of the sample number) as the
    /// device sends it: the inverse of the de-mangling.
    fn readout(model: Model, divcount: u8, f: impl Fn(usize) -> u16) -> Vec<u8> {
        let mut image = vec![0u8; SDRAM_SIZE];
        match model {
            Model::La8 => {
                for (n, b) in image.iter_mut().enumerate() {
                    *b = f(n) as u8;
                }
            }
            Model::La16 => {
                for n in 0..SDRAM_SIZE / 2 {
                    // High byte first before the swap.
                    image[2 * n..2 * n + 2].copy_from_slice(&f(n).to_be_bytes());
                }
            }
        }
        (0..SDRAM_SIZE).map(|o| image[demangle_index(model, divcount, o)]).collect()
    }

    #[test]
    fn demangling_is_a_permutation_in_the_c_order() {
        // Section m holds samples 16k + 2m and 16k + 2m + 1 (LA8).
        assert_eq!(demangle_index(Model::La8, 0, 0), 0);
        assert_eq!(demangle_index(Model::La8, 0, 1), 1);
        assert_eq!(demangle_index(Model::La8, 1, 0), 1);
        assert_eq!(demangle_index(Model::La8, 1, 2), 17);
        assert_eq!(demangle_index(Model::La8, 0, SECTION), 2);
        assert_eq!(demangle_index(Model::La16, 0, 0), 1);
        assert_eq!(demangle_index(Model::La16, 0, 3), 2);
        assert_eq!(demangle_index(Model::La16, 0, 4), 33);
        assert_eq!(demangle_index(Model::La16, 0, SECTION), 5);
        for (model, divcount) in [(Model::La8, 0), (Model::La8, 7), (Model::La16, 0)] {
            let mut seen = vec![false; SDRAM_SIZE];
            for o in 0..SDRAM_SIZE {
                let i = demangle_index(model, divcount, o);
                assert!(!seen[i], "{model:?} {divcount}: {o} lands on {i} twice");
                seen[i] = true;
            }
        }
        // Round trip, with the LA16 swap.
        let f = |n: usize| (n ^ n >> 16) as u16;
        let raw = readout(Model::La16, 3, f);
        let mut image = vec![0u8; SDRAM_SIZE];
        for (o, &b) in raw.iter().enumerate() {
            image[demangle_index(Model::La16, 3, o)] = b;
        }
        swap_pairs(&mut image);
        for n in [0, 1, 2, 70_000, SDRAM_SIZE / 2 - 1] {
            assert_eq!(sample(&image, 2, n), f(n), "sample {n}");
        }
    }

    #[test]
    fn triggers_are_found_in_whole_samples() {
        // LA16, samples 0x0000, 0x0001, 0x0002, 0x0103 ...
        let data: Vec<u8> =
            [0u16, 1, 2, 0x0103, 0x0200].iter().flat_map(|s| s.to_le_bytes()).collect();
        let t = |pattern, mask, edges| TriggerMasks { pattern, mask, edges };
        // Channel 0 high: sample 1, not byte 1.
        assert_eq!(find_trigger(&data, 2, t(1, 1, 0)), Some(1));
        // Channel 9 high: sample 4, not the first byte.
        assert_eq!(find_trigger(&data, 2, t(0x200, 0x200, 0)), Some(4));
        // Channel 8 high with channel 1 rising: 0x0002 -> 0x0103 keeps
        // channel 1 high, no edge; none.
        assert_eq!(find_trigger(&data, 2, t(0x102, 0x100, 2)), None);
        // Channel 1 rising alone: sample 2.
        assert_eq!(find_trigger(&data, 2, t(2, 0, 2)), Some(2));
        // Channel 0 falling: low on sample 0 already, so sample 0; on
        // samples high first, 0x0001 -> 0x0002.
        assert_eq!(find_trigger(&data, 2, t(0, 0, 1)), Some(0));
        assert_eq!(find_trigger(&data[2..], 2, t(0, 0, 1)), Some(1));
        // An edge channel already at its new level on sample 0.
        assert_eq!(find_trigger(&data, 2, t(0, 0, 4)), Some(0));
        assert_eq!(find_trigger(&data, 2, TriggerMasks::default()), None);
        // LA8 bytes.
        assert_eq!(find_trigger(&[0, 0x80, 0x81], 1, t(0x81, 0x81, 0)), Some(2));
    }

    #[test]
    fn chunks_split_at_the_trigger_and_stop_at_the_limit() {
        use Chunk::{Logic, Trigger};
        let n = SDRAM_SIZE;
        assert_eq!(chunks(n, 1, None, 100), [Logic(0..100)]);
        assert_eq!(chunks(n, 1, Some(0), 100), [Trigger, Logic(0..100)]);
        assert_eq!(chunks(n, 1, Some(100), 100), [Logic(0..100)]);
        assert_eq!(chunks(n, 1, Some(50), 100), [Logic(0..50), Trigger, Logic(50..100)]);
        let all = chunks(n / 2, 2, None, 0);
        assert_eq!(all.len(), NUM_BLOCKS);
        assert_eq!(all[1], Logic(2048..4096));
        // A trigger on a block's last sample keeps it.
        assert_eq!(
            chunks(n, 1, Some(4095), 5000),
            [Logic(0..4095), Trigger, Logic(4095..4096), Logic(4096..5000)]
        );
        assert_eq!(chunks(n, 1, Some(4096), 5000), [Logic(0..4096), Trigger, Logic(4096..5000)]);
    }

    #[test]
    fn limits() {
        assert_eq!(sample_limit(0, 0, 1_000_000), 0);
        assert_eq!(sample_limit(100, 0, 1_000_000), 100);
        assert_eq!(sample_limit(0, 5, 392_156), 1960);
        assert_eq!(sample_limit(1000, 5, 392_156), 1000);
    }
}
