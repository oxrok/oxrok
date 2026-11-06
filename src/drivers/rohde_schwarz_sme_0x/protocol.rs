// Counterpart in sigrok:
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/api.c,
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/protocol.c,
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/protocol.h,
//   Copyright (C) 2016 Vlad Ivanov <vlad.ivanov@lab-systems.ru>,
//   with commits by Uwe Hermann, Gerhard Sittig, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The commands of the Rohde&Schwarz SME series, in SCPI, as the maker's operating
//! manual (edition 14, 1038.6002.02) has them.
//!
//! `\n` alone switches RS-232 to remote (a CR or an LF does, section 3.3), at a
//! scan. `*IDN?` says who it is, as `Rohde&Schwarz, SME03,00000001, 1.03`.
//! `FREQ %.1fHz` and `POW %.1fdBm` set and have no answer, so a value the device
//! does not take goes unnoticed. `FREQ?` and `POW?` read, with no unit (section
//! 3.5.4: `FREQuency? Response: 1E6`).
//!
//! The ranges below are the manual's with no offset, and C's. `FREQ` and `POW` are
//! the output plus `FREQ:OFFS` (-50 to +50 GHz) and `POW:OFFS` (-100 to +100 dB),
//! which the driver neither sets nor reads, so a set valid under an offset is
//! refused here, and nothing goes out in C either.
//!
//! A reply does not say which query it answers. After a query went unanswered, or
//! got what cannot be its answer, the next query on a serial port first reads
//! until the line has been silent for [`QUIET`], and drops that, as a late reply
//! would shift every reading after it. What is left:
//!
//! - A reply that begins later than that, or in the moment between the last look
//!   and the purge, which is microseconds to a byte's millisecond at 9600.
//! - A `POW?` that gets a frequency fails by its size, but nothing tells a `FREQ?`
//!   that gets a level, nor one reply of a kind from another: a `FREQ?` more than
//!   [`QUIET`] late, a set, and a `FREQ?` give the old frequency once, as in C.
//! - A line that never falls silent holds a query for [`SETTLE_MAX`], and the
//!   query then goes out, and `Scpi::get_data` reads it for as long as bytes come
//!   with no newline. That is shared code, as in C.
//!
//! The settling is for a serial port. Over USBTMC a read is a request to the
//! device, and over TCP there is no purge, so there C's behaviour stands. An SME
//! has neither: it has the IEC bus and RS-232, and those resources reach it only
//! through what a user puts in between, which no case here runs.

use crate::core::{Error, Result};
use crate::scpi::{HwInfo, Scpi};
use std::time::{Duration, Instant};

/// `manufacturer` in `api.c`, compared exactly.
pub const MANUFACTURER: &str = "Rohde&Schwarz";

/// What `api.c` knows of a model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    /// As `*IDN?` spells it, compared exactly.
    pub name: &'static str,
    /// Highest frequency in Hz.
    pub freq_max: f64,
    /// Lowest frequency in Hz.
    pub freq_min: f64,
    /// Highest level in dBm.
    pub power_max: f64,
    /// Lowest level in dBm.
    pub power_min: f64,
}

const fn model(name: &'static str, freq_max: f64) -> Model {
    Model { name, freq_max, freq_min: 5e3, power_max: 16.0, power_min: -144.0 }
}

/// `device_models`, but for the SME06: C has 1.5 GHz, a copy of the SME02's, and
/// the manual has 6 GHz (`[:SOURce]:FREQuency[:CW|:FIXed]`, 5 kHz to 1.5 GHz and
/// SME03E/03/06 to 2.2/3/6 GHz; "SME06: to 6 GHz").
pub const MODELS: [Model; 5] = [
    model("SME02", 1.5e9),
    model("SME03E", 2.2e9),
    model("SME03A", 3e9),
    model("SME03", 3e9),
    model("SME06", 6e9),
];

/// How long the line must stay silent before a query follows one that got no
/// good answer. Anything answering within this is read and dropped.
pub const QUIET: Duration = Duration::from_secs(1);
/// However long ago that was, the line gets this long to show a reply on its way,
/// as a purge would cut one and leave its tail for an answer. Past the time 9600
/// baud takes for a byte, and more.
const LOOK: Duration = Duration::from_millis(20);
/// The most a line that never falls silent can hold the next query. It holds it
/// only that far: see the module's list for what such a line does then.
const SETTLE_MAX: Duration = Duration::from_secs(2);
/// Beyond this no reply is a level, under any `:UNIT:POWer` the manual has, the
/// widest being about -274 to +223: -144 to +16 dBm, an offset of 100 dB, then
/// dBW at the bottom and dBuV at the top.
const LEVEL_LIMIT: f64 = 1e3;

/// The model `id` names, when this driver has it and the maker is the one.
pub fn find_model(id: &HwInfo) -> Option<&'static Model> {
    if id.manufacturer != MANUFACTURER {
        return None;
    }
    MODELS.iter().find(|m| m.name == id.model)
}

/// A reply as a number: `sr_atod_ascii`, but for what a C string makes of a NUL,
/// and but for hexadecimal, which it takes and the manual's replies never are.
/// Trailing space is no number in it either. An empty reply is 0 there and nothing
/// here: nothing was said. So is what is not finite, which no generator is set to.
pub fn parse_reading(reply: &str) -> Option<f64> {
    let text = reply.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    if text.is_empty() {
        return None;
    }
    text.parse().ok().filter(|v: &f64| v.is_finite())
}

/// `FREQ %.1fHz`; Rust and `printf` round the exact value alike.
pub fn frequency_command(hz: f64) -> String {
    format!("FREQ {hz:.1}Hz")
}

/// `POW %.1fdBm`.
pub fn level_command(dbm: f64) -> String {
    format!("POW {dbm:.1}dBm")
}

/// Whether `value` may go out in `min..=max`. A NaN passes C's two comparisons
/// and goes out as `nan`.
fn check(what: &str, value: f64, min: f64, max: f64) -> Result<()> {
    if value.is_nan() || value < min || value > max {
        return Err(Error::Arg(format!("{what} {value} is outside {min} to {max}")));
    }
    Ok(())
}

/// The instrument while open, and whether a reply may still be on its way.
pub struct Link {
    /// The transport, while open.
    pub scpi: Option<Scpi>,
    /// The model the scan found.
    pub model: Model,
    /// Since when a query has had no good answer.
    out_of_step: Option<Instant>,
}

impl Link {
    /// A link, not yet open.
    pub fn new(model: Model) -> Link {
        Link { scpi: None, model, out_of_step: None }
    }

    /// A new transport. What the last one had on its way is gone with it.
    pub fn attach(&mut self, scpi: Scpi) {
        self.scpi = Some(scpi);
        self.out_of_step = None;
    }

    fn scpi(&mut self) -> Result<&mut Scpi> {
        self.scpi.as_mut().ok_or_else(|| Error::device("The device is not open."))
    }

    /// Reads and drops what comes until the line has been silent for [`QUIET`]
    /// since `since` or the last byte, and for [`LOOK`] at least, up to
    /// [`SETTLE_MAX`]. The last read comes after the time is up, so a byte that
    /// came in the last pause counts.
    fn settle(scpi: &mut Scpi, since: Instant) -> Result<()> {
        let began = Instant::now();
        let bound = began + SETTLE_MAX;
        let mut deadline = (since + QUIET).max(began + LOOK);
        loop {
            let left = deadline.min(bound).saturating_duration_since(Instant::now());
            let asked = left.max(Duration::from_millis(1));
            let Some(got) = scpi.discard(asked)? else {
                return Ok(());
            };
            let now = Instant::now();
            if got > 0 {
                deadline = now + QUIET;
                if now >= bound {
                    return Ok(());
                }
            } else if left.is_zero() {
                return Ok(());
            } else if now.duration_since(began) < SETTLE_MAX {
                // A port may return at once with nothing.
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }

    /// The answer to `command`, which `fits` says can be one.
    fn read(&mut self, command: &str, fits: impl Fn(f64) -> bool) -> Result<f64> {
        let pending = self.out_of_step.take();
        let scpi = self.scpi.as_mut().ok_or_else(|| Error::device("The device is not open."))?;
        if let Some(since) = pending {
            log::debug!("Waiting for the line, after a query with no good answer.");
            if let Err(e) = Self::settle(scpi, since) {
                self.out_of_step = pending;
                return Err(e);
            }
        }
        scpi.purge()?;
        let reply = match scpi.get_string(command) {
            Ok(reply) => reply,
            Err(e) => {
                self.out_of_step = Some(Instant::now());
                return Err(e);
            }
        };
        if let Some(value) = parse_reading(&reply).filter(|v| fits(*v)) {
            return Ok(value);
        }
        self.out_of_step = Some(Instant::now());
        Err(Error::device(format!("'{reply}' is no answer to {command}")))
    }

    /// `FREQ?`, in Hz. Any number: with an offset a real one can be small or
    /// negative, as `FREQ` is the output plus the offset.
    pub fn get_frequency(&mut self) -> Result<f64> {
        self.read("FREQ?", |_| true)
    }

    /// `POW?`, in dBm.
    pub fn get_level(&mut self) -> Result<f64> {
        self.read("POW?", |v| v.abs() <= LEVEL_LIMIT)
    }

    /// `FREQ`, within the model's range.
    pub fn set_frequency(&mut self, hz: f64) -> Result<()> {
        check("Frequency", hz, self.model.freq_min, self.model.freq_max)?;
        self.scpi()?.send(&frequency_command(hz))
    }

    /// `POW`, within the model's range.
    pub fn set_level(&mut self, dbm: f64) -> Result<()> {
        check("Level", dbm, self.model.power_min, self.model.power_max)?;
        self.scpi()?.send(&level_command(dbm))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::scpi::serial::Serial as ScpiSerial;
    use crate::serial::{Params, Port};
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// What the fake holds: bytes to give, and what was written.
    #[derive(Default)]
    pub struct Line {
        /// Bytes that have come and a read can take.
        pub arrived: VecDeque<u8>,
        /// Each entry joins `arrived` after that many more reads.
        pub coming: Vec<(usize, Vec<u8>)>,
        /// Replies by the command that asks for them, in order.
        pub answers: VecDeque<Option<Vec<u8>>>,
        /// All that was written.
        pub written: Vec<u8>,
        /// Reads so far.
        pub reads: usize,
        /// Bytes that a purge dropped.
        pub purged: usize,
        /// A byte for every read, until this time.
        pub noise: Option<Instant>,
        /// Writes left to fail.
        pub failing_writes: usize,
        /// Reads left to fail.
        pub failing_reads: usize,
        /// How long the next read that gives nothing takes, past its timeout.
        pub stall: Option<Duration>,
    }

    /// A port driven by counts, with no clock, that releases one byte per read
    /// as a slow line does.
    #[derive(Clone, Default)]
    pub struct Fake(pub Arc<Mutex<Line>>);

    impl Fake {
        /// A fake that answers the queries from `answers` in order: `None`, no
        /// answer.
        pub fn with(answers: &[Option<&[u8]>]) -> Fake {
            let fake = Fake::default();
            fake.line().answers = answers.iter().map(|a| a.map(<[u8]>::to_vec)).collect();
            fake
        }

        /// The lock, which a test's panic must not poison for the next look.
        pub fn line(&self) -> std::sync::MutexGuard<'_, Line> {
            self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
        }

        /// What was written, as text.
        pub fn written(&self) -> String {
            String::from_utf8_lossy(&self.line().written).into_owned()
        }

        /// A transport on this port.
        pub fn scpi(&self) -> Scpi {
            Scpi::new(Box::new(ScpiSerial::from_port(Box::new(self.clone()))))
        }
    }

    impl Port for Fake {
        fn name(&self) -> &'static str {
            "fake"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            let mut line = self.line();
            line.reads += 1;
            if line.failing_reads > 0 {
                line.failing_reads -= 1;
                return Err(Error::device("read failed"));
            }
            let mut due = Vec::new();
            line.coming.retain_mut(|(reads, bytes)| {
                if *reads == 0 {
                    due.append(bytes);
                    return false;
                }
                *reads -= 1;
                true
            });
            line.arrived.extend(due);
            if line.noise.is_some_and(|end| Instant::now() < end) {
                drop(line);
                std::thread::sleep(Duration::from_micros(200));
                buf[0] = b'x';
                return Ok(1);
            }
            if let Some(byte) = line.arrived.pop_front() {
                buf[0] = byte;
                return Ok(1);
            }
            let stall = line.stall.take();
            drop(line);
            // No clock in what the test holds: a pause so that real waits pass.
            std::thread::sleep(stall.unwrap_or(timeout.min(Duration::from_millis(5))));
            Ok(0)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            let mut line = self.line();
            if line.failing_writes > 0 {
                line.failing_writes -= 1;
                return Err(Error::device("write failed"));
            }
            line.written.extend_from_slice(data);
            if data.ends_with(b"?\n")
                && let Some(answer) = line.answers.pop_front()
                && let Some(answer) = answer
            {
                // A device answers in order: behind what is still coming.
                match line.coming.last_mut() {
                    Some((_, bytes)) => bytes.extend(answer),
                    None => line.arrived.extend(answer),
                }
            }
            Ok(())
        }
        fn purge(&mut self) -> Result<()> {
            let mut line = self.line();
            line.purged += line.arrived.len();
            line.arrived.clear();
            Ok(())
        }
    }

    fn link(fake: &Fake) -> Link {
        let mut link = Link::new(MODELS[3]);
        link.attach(fake.scpi());
        link
    }

    /// A read's timeouts cut to what a test can wait for.
    fn quick(link: &mut Link) {
        link.scpi.as_mut().unwrap().read_timeout = Duration::from_millis(50);
    }

    #[test]
    fn commands_as_printf_has_them() {
        assert_eq!(frequency_command(1e6), "FREQ 1000000.0Hz");
        assert_eq!(frequency_command(5000.25), "FREQ 5000.2Hz");
        assert_eq!(frequency_command(5000.75), "FREQ 5000.8Hz");
        assert_eq!(frequency_command(123_456_789.0), "FREQ 123456789.0Hz");
        assert_eq!(level_command(-10.25), "POW -10.2dBm");
        assert_eq!(level_command(-0.0), "POW -0.0dBm");
        assert_eq!(level_command(-0.04), "POW -0.0dBm");
        assert_eq!(level_command(16.0), "POW 16.0dBm");
    }

    #[test]
    fn readings_parse_as_c_does_and_not_where_nothing_was_said() {
        for (text, want) in [
            ("100000000", 1e8),
            ("1.00E9", 1e9),
            ("+5e3", 5e3),
            (" 12.5", 12.5),
            ("\r-30", -30.0),
            (".5", 0.5),
            ("5.", 5.0),
        ] {
            assert_eq!(parse_reading(text), Some(want), "{text}");
        }
        for text in ["", "  ", "1 ", "1x", "x", "nan", "inf", "-inf", "1e999", "1,2"] {
            assert_eq!(parse_reading(text), None, "{text}");
        }
    }

    #[test]
    fn the_table() {
        let tops: Vec<(&str, f64)> = MODELS.iter().map(|m| (m.name, m.freq_max)).collect();
        assert_eq!(
            tops,
            [
                ("SME02", 1_500_000_000.0),
                ("SME03E", 2_200_000_000.0),
                ("SME03A", 3_000_000_000.0),
                ("SME03", 3_000_000_000.0),
                ("SME06", 6_000_000_000.0),
            ]
        );
        for m in MODELS {
            assert_eq!(m.freq_min, 5000.0);
            assert_eq!(m.power_max, 16.0);
            assert_eq!(m.power_min, -144.0);
        }
    }

    #[test]
    fn who_it_is() {
        let id = |maker: &str, model: &str| HwInfo {
            manufacturer: maker.to_owned(),
            model: model.to_owned(),
            ..HwInfo::default()
        };
        assert_eq!(find_model(&id("Rohde&Schwarz", "SME03")).unwrap().name, "SME03");
        assert_eq!(find_model(&id("Rohde&Schwarz", "SME03A")).unwrap().name, "SME03A");
        assert!(find_model(&id("ROHDE&SCHWARZ", "SME03")).is_none());
        assert!(find_model(&id("Rohde&Schwarz", "sme03")).is_none());
        assert!(find_model(&id("Rohde&Schwarz", "SME03B")).is_none());
    }

    #[test]
    fn settings_within_the_range_go_out_and_others_do_not() {
        let fake = Fake::default();
        let mut link = link(&fake);
        link.set_frequency(5e3).unwrap();
        link.set_frequency(3e9).unwrap();
        link.set_level(16.0).unwrap();
        link.set_level(-144.0).unwrap();
        assert_eq!(
            fake.written(),
            "FREQ 5000.0Hz\nFREQ 3000000000.0Hz\nPOW 16.0dBm\nPOW -144.0dBm\n"
        );
        fake.line().written.clear();
        for hz in [4999.9, 3e9 + 1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(link.set_frequency(hz).is_err(), "{hz}");
        }
        for dbm in [16.1, -144.1, f64::NAN, f64::INFINITY] {
            assert!(link.set_level(dbm).is_err(), "{dbm}");
        }
        assert_eq!(fake.written(), "");
    }

    #[test]
    fn a_closed_link_says_so() {
        let mut link = Link::new(MODELS[0]);
        assert!(link.get_frequency().is_err());
        assert!(link.set_level(0.0).is_err());
    }

    #[test]
    fn queries_and_readings() {
        let fake = Fake::with(&[Some(b"100000000\n"), Some(b"-30\r\n")]);
        let mut link = link(&fake);
        assert_eq!(link.get_frequency().unwrap(), 1e8);
        assert_eq!(link.get_level().unwrap(), -30.0);
        assert_eq!(fake.written(), "FREQ?\nPOW?\n");
    }

    #[test]
    fn what_cannot_be_the_answer_fails() {
        for reply in [&b"\n"[..], b"abc\n", b"nan\n", b"inf\n", b"1 \n"] {
            let fake = Fake::with(&[Some(reply)]);
            let mut l = link(&fake);
            assert!(l.get_frequency().is_err(), "{}", reply.escape_ascii());
            assert!(l.out_of_step.is_some());
        }
        for reply in [&b"100000000\n"[..], b"1001\n", b"-1001\n", b"\n", b"abc\n"] {
            let fake = Fake::with(&[Some(reply)]);
            let mut l = link(&fake);
            assert!(l.get_level().is_err(), "{}", reply.escape_ascii());
        }
        // The other side of those bounds.
        let fake = Fake::with(&[Some(b"1000\n"), Some(b"-1000\n")]);
        let mut l = link(&fake);
        assert_eq!(l.get_level().unwrap(), 1000.0);
        assert_eq!(l.get_level().unwrap(), -1000.0);
    }

    /// With an offset a frequency can be anything: the output plus the offset.
    #[test]
    fn a_frequency_reading_has_no_floor() {
        let fake = Fake::with(&[Some(b"1\n"), Some(b"0\n"), Some(b"-4.9E10\n"), Some(b"-3E1\n")]);
        let mut l = link(&fake);
        assert_eq!(l.get_frequency().unwrap(), 1.0);
        assert_eq!(l.get_frequency().unwrap(), 0.0);
        assert_eq!(l.get_frequency().unwrap(), -4.9e10);
        // So this is how a level one query behind reads, which nothing can tell.
        assert_eq!(l.get_frequency().unwrap(), -30.0);
    }

    /// A reply late by more than the wait: it comes byte by byte while the next
    /// query's wait for the line runs, and must not be taken for that answer.
    #[test]
    fn a_late_reply_does_not_shift_what_follows() {
        let fake = Fake::with(&[None, Some(b"-30\n"), Some(b"100000000\n")]);
        let mut l = link(&fake);
        quick(&mut l);
        assert!(matches!(l.get_frequency(), Err(Error::Timeout)));
        fake.line().coming.push((1, b"100000000\n".to_vec()));
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert_eq!(l.get_frequency().unwrap(), 1e8);
        assert!(l.out_of_step.is_none());
        assert_eq!(fake.written(), "FREQ?\nPOW?\nFREQ?\n");
    }

    /// One that has come in full by the time of the next query: the purge has
    /// it even with no wait left.
    #[test]
    fn a_queued_reply_is_purged() {
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = link(&fake);
        fake.line().arrived.extend(b"100000000\n");
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert_eq!(fake.line().purged, 10);
    }

    /// A recording of the session must not keep the purge from the port.
    #[test]
    fn a_recorded_session_purges_too() {
        use crate::scpi::record::Recorder;
        let fake = Fake::with(&[Some(b"-30\n")]);
        let serial = ScpiSerial::from_port(Box::new(fake.clone()));
        let mut l = Link::new(MODELS[3]);
        l.attach(Scpi::new(Box::new(Recorder::new(Box::new(serial)))));
        fake.line().arrived.extend(b"100000000\n");
        assert_eq!(l.get_level().unwrap(), -30.0);
    }

    /// Past the wait, the form is left: it takes the late reply for the answer's
    /// place, fails, and the wait that follows has the true answer dropped.
    #[test]
    fn a_reply_later_than_the_wait_is_told_by_its_form() {
        let fake = Fake::with(&[Some(b"100000000\n"), Some(b"-30\n")]);
        let mut l = link(&fake);
        assert!(l.get_level().is_err());
        assert!(l.out_of_step.is_some());
        assert_eq!(l.get_level().unwrap(), -30.0);
    }

    /// A reply in flight when the query comes long after the expiry: a purge
    /// straight away would drop its head and leave `00000\n` for an answer, which
    /// is a number, and a level. So there is always a look, and a byte seen in it
    /// starts the whole wait again.
    #[test]
    fn a_reply_in_flight_long_after_the_expiry_is_not_cut() {
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = link(&fake);
        fake.line().arrived.extend(b"1000");
        fake.line().coming.push((6, b"00000\n".to_vec()));
        l.out_of_step = Some(Instant::now().checked_sub(QUIET * 3).unwrap());
        let began = Instant::now();
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert!(began.elapsed() >= QUIET);
        // The look read it: a purge had nothing left to cut.
        assert_eq!(fake.line().purged, 0);
    }

    /// And nothing in flight costs only the look: fewer reads than a wait's.
    #[test]
    fn long_after_the_expiry_the_look_is_short_and_has_a_read() {
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = link(&fake);
        l.out_of_step = Some(Instant::now().checked_sub(QUIET * 3).unwrap());
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert!(fake.line().reads > 4);
    }

    /// A line that does not fall silent holds the wait for a bound, however long
    /// it goes on after that.
    #[test]
    fn a_noisy_line_holds_the_wait_for_a_bound() {
        let fake = Fake::default();
        let mut scpi = fake.scpi();
        fake.line().noise = Some(Instant::now() + Duration::from_secs(60));
        let began = Instant::now();
        Link::settle(&mut scpi, began).unwrap();
        assert!(began.elapsed() >= Duration::from_millis(2000), "{:?}", began.elapsed());
        assert!(began.elapsed() < Duration::from_secs(30), "{:?}", began.elapsed());
        assert!(fake.line().reads > 1);
    }

    /// A silent line ends the wait at the quiet time, counted from `since`.
    #[test]
    fn the_wait_is_quiet_after_since() {
        assert_eq!(QUIET, Duration::from_millis(1000));
        assert_eq!(LOOK, Duration::from_millis(20));
        let fake = Fake::default();
        let mut scpi = fake.scpi();
        let since = Instant::now();
        Link::settle(&mut scpi, since).unwrap();
        assert!(since.elapsed() >= QUIET);
        // Long gone: only the look.
        let began = Instant::now();
        Link::settle(&mut scpi, began.checked_sub(QUIET * 3).unwrap()).unwrap();
        assert!(began.elapsed() >= LOOK);
    }

    /// One more byte starts the quiet again, from that byte.
    #[test]
    fn the_wait_is_quiet_after_the_last_byte() {
        let fake = Fake::default();
        let mut scpi = fake.scpi();
        // On the second read: a slow first one may use up all of the look, and
        // the read after that still comes.
        fake.line().coming.push((1, b"x".to_vec()));
        let since = Instant::now().checked_sub(QUIET * 3).unwrap();
        let began = Instant::now();
        Link::settle(&mut scpi, since).unwrap();
        assert!(began.elapsed() >= QUIET, "{:?}", began.elapsed());
    }

    /// A read that stalls past the time, with a byte right behind it: the time
    /// being up is no reason not to look once more before the purge.
    #[test]
    fn a_stall_past_the_time_gets_one_more_read() {
        let fake = Fake::default();
        let mut scpi = fake.scpi();
        fake.line().stall = Some(LOOK * 2);
        fake.line().coming.push((1, b"x".to_vec()));
        let since = Instant::now().checked_sub(QUIET * 3).unwrap();
        let began = Instant::now();
        Link::settle(&mut scpi, since).unwrap();
        assert_eq!(fake.line().arrived.len(), 0);
        assert!(began.elapsed() >= QUIET, "{:?}", began.elapsed());
    }

    #[test]
    fn a_read_error_in_the_wait_fails_the_query_and_sends_nothing() {
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = link(&fake);
        l.out_of_step = Some(Instant::now());
        fake.line().failing_reads = 1;
        assert!(l.get_level().is_err());
        assert_eq!(fake.written(), "");
        assert!(l.out_of_step.is_some());
    }

    #[test]
    fn with_no_expiry_nothing_waits() {
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = link(&fake);
        l.get_level().unwrap();
        // One read per byte, and none before them.
        assert_eq!(fake.line().reads, 4);
        assert!(l.out_of_step.is_none());
    }

    /// A transport with no [`Scpi::discard`], whose reads may be requests, does no
    /// reading before its query.
    #[test]
    fn a_transport_that_cannot_look_does_not_settle() {
        struct NoLook(Fake, bool);
        impl crate::scpi::ScpiTransport for NoLook {
            fn resource(&self) -> &'static str {
                "no-look"
            }
            fn send(&mut self, data: &[u8]) -> Result<()> {
                self.0.write(data)
            }
            fn read_begin(&mut self) -> Result<()> {
                self.1 = false;
                Ok(())
            }
            fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
                assert!(!self.0.written().is_empty(), "a read before the query");
                let n = self.0.read(buf, timeout)?;
                self.1 = n > 0 && buf[n - 1] == b'\n';
                Ok(n)
            }
            fn read_complete(&self) -> bool {
                self.1
            }
        }
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = Link::new(MODELS[3]);
        l.attach(Scpi::new(Box::new(NoLook(fake.clone(), false))));
        l.out_of_step = Some(Instant::now());
        let began = Instant::now();
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert!(began.elapsed() < Duration::from_secs(30));
        assert_eq!(fake.line().reads, 4);
    }

    /// A session with a late reply replays: what the wait drops is not recorded.
    #[test]
    fn a_recording_with_a_late_reply_replays() {
        use crate::scpi::record::{Recorder, Replay};
        let file = std::env::temp_dir().join(format!("oxrok-sme-late-{}.txt", std::process::id()));
        let fake = Fake::with(&[None, Some(b"-30\n")]);
        {
            let serial = ScpiSerial::from_port(Box::new(fake.clone()));
            let recorder = Recorder::to_file(Box::new(serial), file.display().to_string());
            let mut l = Link::new(MODELS[3]);
            l.attach(Scpi::new(Box::new(recorder)));
            quick(&mut l);
            assert!(l.get_frequency().is_err());
            fake.line().coming.push((1, b"100000000\n".to_vec()));
            assert_eq!(l.get_level().unwrap(), -30.0);
        }
        let text = std::fs::read_to_string(&file).unwrap();
        std::fs::remove_file(&file).unwrap();
        let mut l = Link::new(MODELS[3]);
        l.attach(Scpi::new(Box::new(Replay::from_text(&text).unwrap())));
        quick(&mut l);
        assert!(l.get_frequency().is_err(), "{text}");
        assert_eq!(l.get_level().unwrap(), -30.0, "{text}");
    }

    /// A setting right after an expiry goes out at once and leaves the wait for
    /// the next query.
    #[test]
    fn a_setting_does_not_clear_the_wait() {
        let fake = Fake::with(&[None, Some(b"-30\n")]);
        let mut l = link(&fake);
        quick(&mut l);
        assert!(l.get_level().is_err());
        l.set_level(1.0).unwrap();
        assert_eq!(fake.written(), "POW?\nPOW 1.0dBm\n");
        assert!(l.out_of_step.is_some());
        fake.line().coming.push((1, b"7\n".to_vec()));
        assert_eq!(l.get_level().unwrap(), -30.0);
    }

    /// C drops this too, and says OK (`api.c:203`).
    #[test]
    fn a_failed_write_is_an_error() {
        let fake = Fake::default();
        let mut l = link(&fake);
        fake.line().failing_writes = 2;
        assert!(l.set_frequency(1e6).is_err());
        assert!(l.set_level(0.0).is_err());
        l.set_level(0.0).unwrap();
    }

    /// A new transport has nothing on its way.
    #[test]
    fn an_open_forgets_the_old_transport() {
        let fake = Fake::with(&[None]);
        let mut l = link(&fake);
        quick(&mut l);
        assert!(l.get_level().is_err());
        let second = Fake::with(&[Some(b"-30\n")]);
        l.attach(second.scpi());
        assert_eq!(l.get_level().unwrap(), -30.0);
        // One read per byte, and none before them.
        assert_eq!(second.line().reads, 4);
    }
}
