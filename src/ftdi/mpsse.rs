// Counterpart in sigrok:
//   none. openFPGALoader (src/ftdiJtagMPSSE.cpp, src/ftdipp_mpsse.cpp, src/jtag.cpp),
//   Apache-2.0, by Gwenhael Goavec-Merou, was the reference for the order of
//   the commands, which is kept: the adapters this is used with are not all FTDI's
//   chips, and what a firmware that stands in for one takes is what that tool sends.
// This file is written for oxrok from FTDI's AN_108.

//! JTAG on an FTDI port in MPSSE mode, for one device on the chain.
//!
//! What a driver needs to load an FPGA: the TAP reset and left in
//! Run-Test/Idle, an instruction shifted in, a data register shifted in or
//! out, and clocks in Run-Test/Idle. Every call leaves the TAP in
//! Run-Test/Idle.
//!
//! The pins are FTDI's for JTAG: TCK on bit 0 of the port, TDI on 1, TDO
//! on 2, TMS on 3. Data changes on the falling edge of TCK and is read on
//! the rising one. Bits go least significant first, as JTAG has them.
//!
//! A command is never split over two USB writes, and a command that reads
//! is followed by the read: a firmware that only stands in for the chip
//! (the debugger of a Sipeed board, a BL616) has less of a buffer than the
//! chip.

use super::{BitMode, Ftdi, Port};
use crate::core::{Error, Result};
use crate::usb::Transport;
use std::time::Duration;

/// MPSSE opcodes (`AN_108`).
mod op {
    /// Bytes out on TDI, least significant bit first, on the falling edge.
    pub const BYTES_OUT: u8 = 0x19;
    /// Bits out on TDI.
    pub const BITS_OUT: u8 = 0x1b;
    /// Bytes out on TDI and in from TDO.
    pub const BYTES_IO: u8 = 0x39;
    /// Bits out on TDI and in from TDO.
    pub const BITS_IO: u8 = 0x3b;
    /// Bits out on TMS, with bit 7 of the data held on TDI.
    pub const TMS: u8 = 0x4b;
    /// Bits out on TMS, and TDO read.
    pub const TMS_IN: u8 = 0x6b;
    pub const SET_LOW: u8 = 0x80;
    pub const SET_HIGH: u8 = 0x82;
    pub const DIVISOR: u8 = 0x86;
    pub const NO_DIVIDE_BY_5: u8 = 0x8a;
}

/// The most data bytes one command carries here: it and its three bytes
/// of header are one USB write of 4096.
const CHUNK: usize = 4096 - 3;
/// TMS bits per command, as openFPGALoader sends them.
const TMS_BITS: u32 = 6;
/// Silence this long in an answer is a failure.
const ANSWER: Duration = Duration::from_secs(2);

/// JTAG through one port of an FTDI chip, or of what stands in for one.
pub struct Jtag {
    ftdi: Ftdi,
    hz: u32,
}

impl Jtag {
    /// Claims `port` of the device behind `transport`, puts it in MPSSE
    /// mode at about `hz` (never above), resets the TAP and leaves it in
    /// Run-Test/Idle.
    pub fn open(transport: Box<dyn Transport>, port: Port, hz: u32) -> Result<Jtag> {
        let mut ftdi = Ftdi::open(transport, port)?;
        ftdi.read_timeout = Duration::from_millis(100);
        ftdi.reset()?;
        ftdi.set_bitmode(0, BitMode::Reset)?;
        ftdi.purge_read()?;
        ftdi.purge_write()?;
        ftdi.set_latency_timer(5)?;
        // TCK, TDI and TMS out, TDO in; the other pins as the cable
        // openFPGALoader calls ft2232 has them.
        ftdi.set_bitmode(0xfb, BitMode::Mpsse)?;
        let mut jtag = Jtag { ftdi, hz: 0 };
        jtag.drain()?;
        jtag.set_clock(hz)?;
        jtag.drain()?;
        jtag.ftdi.write(&[op::SET_LOW, 0x08, 0x0b, op::SET_HIGH, 0x08, 0x0b])?;
        jtag.reset_tap()?;
        Ok(jtag)
    }

    /// The TCK rate, in Hz.
    pub fn hz(&self) -> u32 {
        self.hz
    }

    /// Gives the port back to its serial function and the transport to the
    /// caller.
    pub fn close(mut self) -> Result<Box<dyn Transport>> {
        self.ftdi.set_bitmode(0, BitMode::Reset)?;
        Ok(self.ftdi.into_transport())
    }

    /// Drops what the chip still has for the host.
    fn drain(&mut self) -> Result<()> {
        let mut buf = [0; 64];
        while self.ftdi.read(&mut buf)? > 0 {}
        Ok(())
    }

    /// The divisor for the nearest rate at or under `hz`: the chips with a
    /// 60 MHz clock are told not to divide it by five, the others have 12.
    fn set_clock(&mut self, hz: u32) -> Result<()> {
        let base: u32 = if self.ftdi.chip().high_speed() { 60_000_000 } else { 12_000_000 };
        let hz = hz.clamp(1, base / 2);
        let mut divisor = (base / hz).saturating_sub(1) / 2;
        if base / ((1 + divisor) * 2) > hz {
            divisor += 1;
        }
        let divisor = u16::try_from(divisor).map_err(|_| Error::arg("JTAG clock too slow"))?;
        self.hz = base / ((1 + u32::from(divisor)) * 2);
        let d = divisor.to_le_bytes();
        if self.ftdi.chip().high_speed() {
            self.ftdi.write(&[op::NO_DIVIDE_BY_5, op::DIVISOR, d[0], d[1]])
        } else {
            self.ftdi.write(&[op::DIVISOR, d[0], d[1]])
        }
    }

    /// Clocks `count` bits of `bits` out on TMS, the first in bit 0, with
    /// TDI low.
    fn tms(&mut self, mut bits: u32, mut count: u32) -> Result<()> {
        let mut out = Vec::new();
        while count > 0 {
            let n = count.min(TMS_BITS);
            let now = (bits & ((1 << n) - 1)) as u8;
            // The bit after the last is the level TMS is left at.
            let last = (now >> (n - 1)) & 1;
            out.extend_from_slice(&[op::TMS, (n - 1) as u8, now | (last << n)]);
            bits >>= n;
            count -= n;
        }
        self.ftdi.write(&out)
    }

    /// Test-Logic-Reset from any state, then Run-Test/Idle.
    pub fn reset_tap(&mut self) -> Result<()> {
        self.tms(0b0_111111, 7)
    }

    /// `count` clocks in Run-Test/Idle.
    pub fn idle(&mut self, count: u32) -> Result<()> {
        let mut out = Vec::new();
        let mut left = count;
        while left > 0 {
            let n = left.min(TMS_BITS);
            out.extend_from_slice(&[op::TMS, (n - 1) as u8, 0]);
            left -= n;
            if out.len() >= CHUNK {
                self.ftdi.write(&out)?;
                out.clear();
            }
        }
        self.ftdi.write(&out)
    }

    /// Shifts `bits` bits of `data` (the first in bit 0 of the first byte)
    /// through the register the TAP is at Shift of, the last of them with
    /// TMS high, and gives what came out when `read`.
    fn shift(&mut self, data: &[u8], bits: usize, read: bool) -> Result<Vec<u8>> {
        if bits == 0 || data.len() * 8 < bits {
            return Err(Error::arg("JTAG shift length"));
        }
        // All but the last bit as whole bytes and then bits.
        let bytes = (bits - 1) / 8;
        let rest = (bits - 1) % 8;
        let mut got = Vec::with_capacity(bits.div_ceil(8));
        for chunk in data[..bytes].chunks(CHUNK) {
            let n = (chunk.len() - 1).to_le_bytes();
            let mut out = vec![if read { op::BYTES_IO } else { op::BYTES_OUT }, n[0], n[1]];
            out.extend_from_slice(chunk);
            self.ftdi.write(&out)?;
            if read {
                let at = got.len();
                got.resize(at + chunk.len(), 0);
                self.ftdi.read_exact(&mut got[at..], ANSWER)?;
            }
        }
        let tail = data[bytes];
        let mut out = Vec::new();
        if rest > 0 {
            out.extend_from_slice(&[
                if read { op::BITS_IO } else { op::BITS_OUT },
                (rest - 1) as u8,
                tail,
            ]);
        }
        // The last bit goes out on TDI (bit 7 of the data) with TMS high:
        // Exit1.
        let last = (tail >> rest) & 1;
        out.extend_from_slice(&[if read { op::TMS_IN } else { op::TMS }, 0, (last << 7) | 1]);
        self.ftdi.write(&out)?;
        if read {
            // Bits come in at the top of a byte and move down.
            let mut byte = 0;
            if rest > 0 {
                let mut b = [0];
                self.ftdi.read_exact(&mut b, ANSWER)?;
                byte = b[0] >> (8 - rest);
            }
            let mut b = [0];
            self.ftdi.read_exact(&mut b, ANSWER)?;
            got.push(byte | ((b[0] >> 7) << rest));
        }
        Ok(got)
    }

    /// Shifts the `bits` low bits of `ir` in as the instruction.
    pub fn shift_ir(&mut self, ir: u32, bits: usize) -> Result<()> {
        // Select-DR, Select-IR, Capture-IR, Shift-IR.
        self.tms(0b0011, 4)?;
        self.shift(&ir.to_le_bytes(), bits, false)?;
        // Update-IR, Run-Test/Idle.
        self.tms(0b01, 2)
    }

    /// Shifts `bits` bits of `data` into the data register, the first in
    /// bit 0 of the first byte.
    pub fn write_dr(&mut self, data: &[u8], bits: usize) -> Result<()> {
        // Select-DR, Capture-DR, Shift-DR.
        self.tms(0b001, 3)?;
        self.shift(data, bits, false)?;
        self.tms(0b01, 2)
    }

    /// Shifts 32 bits out of the data register, ones going in.
    pub fn read_dr32(&mut self) -> Result<u32> {
        self.tms(0b001, 3)?;
        let got = self.shift(&[0xff; 4], 32, true)?;
        self.tms(0b01, 2)?;
        let got: [u8; 4] = got.try_into().map_err(|_| Error::device("short JTAG answer"))?;
        Ok(u32::from_le_bytes(got))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::{BulkStream, ControlRequest, InterfaceDesc, UsbDeviceInfo};
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum State {
        Reset,
        Idle,
        SelectDr,
        CaptureDr,
        ShiftDr,
        Exit1Dr,
        PauseDr,
        Exit2Dr,
        UpdateDr,
        SelectIr,
        CaptureIr,
        ShiftIr,
        Exit1Ir,
        PauseIr,
        Exit2Ir,
        UpdateIr,
    }

    impl State {
        fn next(self, tms: bool) -> State {
            use State::*;
            match (self, tms) {
                (Reset | SelectIr, true) => Reset,
                (Reset | Idle | UpdateDr | UpdateIr, false) => Idle,
                (Idle | UpdateDr | UpdateIr, true) => SelectDr,
                (SelectDr, false) => CaptureDr,
                (SelectDr, true) => SelectIr,
                (CaptureDr | ShiftDr | Exit2Dr, false) => ShiftDr,
                (CaptureDr | ShiftDr, true) => Exit1Dr,
                (Exit1Dr | PauseDr, false) => PauseDr,
                (Exit1Dr | Exit2Dr, true) => UpdateDr,
                (PauseDr, true) => Exit2Dr,
                (SelectIr, false) => CaptureIr,
                (CaptureIr | ShiftIr | Exit2Ir, false) => ShiftIr,
                (CaptureIr | ShiftIr, true) => Exit1Ir,
                (Exit1Ir | PauseIr, false) => PauseIr,
                (Exit1Ir | Exit2Ir, true) => UpdateIr,
                (PauseIr, true) => Exit2Ir,
            }
        }
    }

    /// A TAP with an eight bit instruction register, a 32 bit register
    /// that reads `idcode` behind instruction 0x11, and behind every other
    /// instruction a register that keeps what is shifted into it.
    struct Tap {
        state: State,
        ir: u8,
        idcode: u32,
        shift: u64,
        /// The data registers written, each with the instruction it was
        /// behind.
        written: Vec<(u8, Vec<bool>)>,
        bits: Vec<bool>,
        idle_clocks: u32,
    }

    impl Tap {
        /// One TCK with these levels; what TDO was.
        fn clock(&mut self, tms: bool, tdi: bool) -> bool {
            let tdo = self.shift & 1 != 0;
            match self.state {
                State::CaptureIr => self.shift = 1,
                State::CaptureDr => {
                    self.shift = u64::from(self.idcode);
                    self.bits.clear();
                }
                State::ShiftIr => self.shift = (self.shift >> 1) | (u64::from(tdi) << 7),
                State::ShiftDr => {
                    self.shift = (self.shift >> 1) | (u64::from(tdi) << 31);
                    self.bits.push(tdi);
                }
                State::Idle => self.idle_clocks += 1,
                _ => {}
            }
            self.state = self.state.next(tms);
            match self.state {
                State::UpdateIr => self.ir = self.shift as u8,
                State::UpdateDr if self.ir != 0x11 => {
                    self.written.push((self.ir, std::mem::take(&mut self.bits)));
                }
                _ => {}
            }
            tdo
        }
    }

    #[derive(Default)]
    struct Seen {
        /// Every bulk write.
        writes: Vec<Vec<u8>>,
        bit_modes: Vec<u16>,
    }

    /// An FT2232 in MPSSE mode with a [`Tap`] on its pins.
    struct Adapter {
        info: UsbDeviceInfo,
        tap: Arc<Mutex<Tap>>,
        seen: Arc<Mutex<Seen>>,
        answer: VecDeque<u8>,
    }

    impl Adapter {
        fn run(&mut self, data: &[u8]) {
            let mut tap = self.tap.lock().unwrap();
            let mut at = 0;
            while at < data.len() {
                let op = data[at];
                let read = op & 0x20 != 0;
                match op {
                    0x80 | 0x82 | 0x86 => at += 3,
                    0x8a => at += 1,
                    0x4b | 0x6b => {
                        let (n, bits) = (data[at + 1] + 1, data[at + 2]);
                        let mut got = 0u8;
                        for i in 0..n {
                            let tdo = tap.clock(bits >> i & 1 != 0, bits & 0x80 != 0);
                            got = (got >> 1) | (u8::from(tdo) << 7);
                        }
                        if read {
                            self.answer.push_back(got);
                        }
                        at += 3;
                    }
                    0x19 | 0x39 => {
                        let n = usize::from(u16::from_le_bytes([data[at + 1], data[at + 2]])) + 1;
                        for byte in &data[at + 3..at + 3 + n] {
                            let mut got = 0u8;
                            for i in 0..8 {
                                got |= u8::from(tap.clock(false, byte >> i & 1 != 0)) << i;
                            }
                            if read {
                                self.answer.push_back(got);
                            }
                        }
                        at += 3 + n;
                    }
                    0x1b | 0x3b => {
                        let (n, byte) = (data[at + 1] + 1, data[at + 2]);
                        let mut got = 0u8;
                        for i in 0..n {
                            let tdo = tap.clock(false, byte >> i & 1 != 0);
                            got = (got >> 1) | (u8::from(tdo) << 7);
                        }
                        if read {
                            self.answer.push_back(got);
                        }
                        at += 3;
                    }
                    other => panic!("MPSSE command {other:#04x}"),
                }
            }
            assert_eq!(at, data.len(), "a command split over two writes");
        }
    }

    impl Transport for Adapter {
        fn info(&self) -> &UsbDeviceInfo {
            &self.info
        }
        fn set_configuration(&mut self, _configuration: u8) -> Result<()> {
            Ok(())
        }
        fn claim_interface(&mut self, _interface: u8) -> Result<()> {
            Ok(())
        }
        fn release_interface(&mut self, _interface: u8) -> Result<()> {
            Ok(())
        }
        fn reset(&mut self) -> Result<()> {
            Ok(())
        }
        fn max_packet_size(&mut self, _endpoint: u8) -> Result<usize> {
            Ok(64)
        }
        fn interfaces(&mut self) -> Result<Vec<InterfaceDesc>> {
            Ok(Vec::new())
        }
        fn control_out(&mut self, r: ControlRequest, _d: &[u8], _t: Duration) -> Result<()> {
            if r.request == 0x0b {
                self.seen.lock().unwrap().bit_modes.push(r.value);
            }
            Ok(())
        }
        fn control_in(&mut self, _r: ControlRequest, l: usize, _t: Duration) -> Result<Vec<u8>> {
            Ok(vec![0; l])
        }
        fn bulk_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
            assert!(data.len() <= 4096);
            self.seen.lock().unwrap().writes.push(data.to_vec());
            self.run(data);
            Ok(data.len())
        }
        fn bulk_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            // Two status bytes start every packet.
            let mut raw = vec![0x32, 0x60];
            let n = self.answer.len().min(62);
            raw.extend(self.answer.drain(..n));
            Ok(raw)
        }
        fn interrupt_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
            Ok(data.len())
        }
        fn interrupt_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            Err(Error::Timeout)
        }
        fn bulk_in_stream(&mut self, _e: u8, _s: usize, _d: usize) -> Result<Box<dyn BulkStream>> {
            Err(Error::not_supported("stream"))
        }
    }

    fn open(device_version: u16, idcode: u32) -> (Jtag, Arc<Mutex<Tap>>, Arc<Mutex<Seen>>) {
        let tap = Arc::new(Mutex::new(Tap {
            // Wherever the last user left it.
            state: State::PauseIr,
            ir: 0,
            idcode,
            shift: 0,
            written: Vec::new(),
            bits: Vec::new(),
            idle_clocks: 0,
        }));
        let seen = Arc::new(Mutex::new(Seen::default()));
        let adapter = Adapter {
            info: UsbDeviceInfo { device_version, ..UsbDeviceInfo::default() },
            tap: Arc::clone(&tap),
            seen: Arc::clone(&seen),
            answer: VecDeque::new(),
        };
        let jtag = Jtag::open(Box::new(adapter), Port::A, 6_000_000).unwrap();
        (jtag, tap, seen)
    }

    fn bits_of(data: &[u8], n: usize) -> Vec<bool> {
        (0..n).map(|i| data[i / 8] >> (i % 8) & 1 != 0).collect()
    }

    #[test]
    fn the_open_sets_the_mode_and_the_clock_and_leaves_the_tap_idle() {
        let (jtag, tap, seen) = open(0x0500, 0);
        assert_eq!(jtag.hz(), 6_000_000);
        assert_eq!(tap.lock().unwrap().state, State::Idle);
        {
            let seen = seen.lock().unwrap();
            // Reset, then MPSSE with TCK, TDI and TMS out.
            assert_eq!(seen.bit_modes, [0x0000, 0x02fb]);
            // 12 MHz over two: no divisor, and no command the chip lacks.
            assert_eq!(seen.writes[0], [0x86, 0, 0]);
            assert_eq!(seen.writes[1], [0x80, 0x08, 0x0b, 0x82, 0x08, 0x0b]);
        }
        jtag.close().unwrap();
        assert_eq!(seen.lock().unwrap().bit_modes.last(), Some(&0));

        // A chip with the 60 MHz clock: not divided by five, then by ten.
        let (jtag, _tap, seen) = open(0x0700, 0);
        assert_eq!(jtag.hz(), 6_000_000);
        assert_eq!(seen.lock().unwrap().writes[0], [0x8a, 0x86, 4, 0]);
    }

    #[test]
    fn an_instruction_and_a_register_read() {
        let (mut jtag, tap, _seen) = open(0x0500, 0x8765_4321);
        jtag.shift_ir(0x11, 8).unwrap();
        {
            let tap = tap.lock().unwrap();
            assert_eq!((tap.ir, tap.state), (0x11, State::Idle));
        }
        assert_eq!(jtag.read_dr32().unwrap(), 0x8765_4321);
        assert_eq!(tap.lock().unwrap().state, State::Idle);
        jtag.shift_ir(0xa5, 8).unwrap();
        assert_eq!(tap.lock().unwrap().ir, 0xa5);
    }

    #[test]
    fn a_register_is_written_first_bit_first_at_any_length() {
        let (mut jtag, tap, seen) = open(0x0500, 0);
        jtag.shift_ir(0x17, 8).unwrap();
        // 1 bit, bits short of a byte, whole bytes, and more than one
        // write holds.
        let long: Vec<u8> = (0..9000u32).map(|i| (i * 7 + i / 256) as u8).collect();
        for (data, n) in
            [(&[0x01][..], 1), (&[0xa5, 0x3c, 0x01], 17), (&[0x5a, 0xc3], 16), (&long, 72_000)]
        {
            jtag.write_dr(data, n).unwrap();
            let (ir, got) = tap.lock().unwrap().written.pop().unwrap();
            assert_eq!(ir, 0x17);
            assert!(got == bits_of(data, n), "{n} bits");
            assert_eq!(tap.lock().unwrap().state, State::Idle);
        }
        assert!(seen.lock().unwrap().writes.iter().all(|w| w.len() <= 4096));
        assert!(jtag.write_dr(&[0], 9).is_err());
    }

    #[test]
    fn idle_clocks_stay_in_idle() {
        let (mut jtag, tap, _seen) = open(0x0500, 0);
        let before = tap.lock().unwrap().idle_clocks;
        jtag.idle(13).unwrap();
        jtag.idle(6).unwrap();
        let tap = tap.lock().unwrap();
        assert_eq!((tap.idle_clocks - before, tap.state), (19, State::Idle));
    }
}
