// Counterpart in sigrok:
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/api.c,
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/protocol.c,
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/protocol.h,
//   Copyright (C) 2016 Vlad Ivanov <vlad.ivanov@lab-systems.ru>,
//   with commits by Uwe Hermann, Gerhard Sittig, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The commands of the Rohde&Schwarz SME series, in SCPI.
//!
//! `\n` alone asks for remote control, at a scan. `*IDN?` says who it is.
//! `FREQ %.1fHz` and `POW %.1fdBm` set and have no answer, so a value the device
//! does not take goes unnoticed. `FREQ?` and `POW?` read.
//!
//! A reply does not say which query it answers. After a query went unanswered,
//! or got what cannot be its answer, the next query waits for the line to fall
//! silent and drops what came, as the late reply would shift every reading after
//! it. A reply later than that is told by its form: a frequency and a level
//! cannot be taken for each other, as no model goes below 5 kHz or near 1000 dBm.
//! `SETTLE_MAX` is what a line that does not fall silent can hold a query up.

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
/// the maker's brochure has 5 kHz to 6 GHz. A top too high costs a device error
/// nobody sees, and one too low refuses what the instrument does.
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
/// The most a line that never falls silent can hold the next query.
const SETTLE_MAX: Duration = Duration::from_secs(2);
/// Below this no reply is a frequency: every model starts at 5 kHz.
const FREQUENCY_FLOOR: f64 = 1e3;
/// Beyond this no reply is a level: the range is -144 to 16 dBm.
const LEVEL_LIMIT: f64 = 1e3;

/// The model `id` names, when this driver has it and the maker is the one.
pub fn find_model(id: &HwInfo) -> Option<&'static Model> {
    if id.manufacturer != MANUFACTURER {
        return None;
    }
    MODELS.iter().find(|m| m.name == id.model)
}

/// A reply as a number: `sr_atod_ascii`, but for what a C string makes of a NUL
/// and `strtod`'s hexadecimal, no generator's habit. Trailing space is no number
/// in it either. An empty reply is 0 there and nothing here: nothing was said.
/// So is what is not finite, which no generator is set to.
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

    /// Waits for the line to fall silent, for [`QUIET`] after `since` or the
    /// last byte, and drops what comes.
    fn settle(scpi: &mut Scpi, since: Instant) -> Result<()> {
        let began = Instant::now();
        let mut last = since;
        let mut buf = [0u8; 256];
        let saved = scpi.read_timeout;
        scpi.read_begin()?;
        let outcome = loop {
            let wait = QUIET
                .saturating_sub(last.elapsed())
                .min(SETTLE_MAX.saturating_sub(began.elapsed()));
            if wait.is_zero() {
                break Ok(());
            }
            scpi.read_timeout = wait;
            match scpi.read_data(&mut buf) {
                Ok(0) | Err(Error::Timeout) => {
                    // A port may return at once with nothing.
                    std::thread::sleep(Duration::from_millis(1));
                }
                Ok(_) => last = Instant::now(),
                Err(e) => break Err(e),
            }
        };
        scpi.read_timeout = saved;
        outcome
    }

    /// The answer to `command`, which `fits` says can be one.
    fn read(&mut self, command: &str, fits: impl Fn(f64) -> bool) -> Result<f64> {
        let pending = self.out_of_step.take();
        let scpi = self.scpi()?;
        if let Some(since) = pending {
            log::debug!("Waiting for the line, after a query with no good answer.");
            Self::settle(scpi, since)?;
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

    /// `FREQ?`, in Hz.
    pub fn get_frequency(&mut self) -> Result<f64> {
        self.read("FREQ?", |v| v >= FREQUENCY_FLOOR)
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
        /// A byte for every read, until a purge or this time, which is there for a
        /// mutant to fail instead of reading for ever.
        pub noise: Option<Instant>,
        /// Writes left to fail.
        pub failing_writes: usize,
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
            drop(line);
            // No clock in what the test holds: a pause so that real waits pass.
            std::thread::sleep(timeout.min(Duration::from_millis(5)));
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
            line.noise = None;
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
        for reply in [&b"-30\n"[..], b"999\n", b"\n", b"abc\n", b"nan\n"] {
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
        let fake = Fake::with(&[Some(b"1000\n"), Some(b"1000\n"), Some(b"-1000\n")]);
        let mut l = link(&fake);
        assert_eq!(l.get_frequency().unwrap(), 1000.0);
        assert_eq!(l.get_level().unwrap(), 1000.0);
        assert_eq!(l.get_level().unwrap(), -1000.0);
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

    /// A line that does not fall silent holds a query for a bound, and it does
    /// go out then.
    #[test]
    fn a_noisy_line_holds_a_query_for_a_bound() {
        let fake = Fake::with(&[None]);
        let mut l = link(&fake);
        quick(&mut l);
        assert!(l.get_level().is_err());
        fake.line().noise = Some(Instant::now() + Duration::from_secs(30));
        fake.line().answers.push_back(Some(b"-30\n".to_vec()));
        let began = Instant::now();
        // The purge ends the noise here, so the answer is clean.
        assert_eq!(l.get_level().unwrap(), -30.0);
        let took = began.elapsed();
        assert!(took >= SETTLE_MAX, "{took:?}");
        assert!(took < SETTLE_MAX * 15, "{took:?}");
    }

    #[test]
    fn the_wait_is_quiet_after_the_last_byte() {
        assert_eq!(QUIET, Duration::from_millis(1000));
        assert_eq!(SETTLE_MAX, Duration::from_millis(2000));
        let fake = Fake::with(&[None, Some(b"-30\n")]);
        let mut l = link(&fake);
        quick(&mut l);
        assert!(l.get_level().is_err());
        let began = Instant::now();
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert!(began.elapsed() >= QUIET);
    }

    #[test]
    fn with_no_expiry_nothing_waits() {
        let fake = Fake::with(&[Some(b"-30\n")]);
        let mut l = link(&fake);
        let began = Instant::now();
        l.get_level().unwrap();
        assert!(began.elapsed() < QUIET);
        assert!(l.out_of_step.is_none());
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
        l.attach(Fake::with(&[Some(b"-30\n")]).scpi());
        let began = Instant::now();
        assert_eq!(l.get_level().unwrap(), -30.0);
        assert!(began.elapsed() < QUIET);
    }

    /// The timeout the wait borrows is given back.
    #[test]
    fn the_read_timeout_survives_a_wait() {
        let fake = Fake::with(&[None, Some(b"-30\n")]);
        let mut l = link(&fake);
        quick(&mut l);
        assert!(l.get_level().is_err());
        l.get_level().unwrap();
        assert_eq!(l.scpi.as_ref().unwrap().read_timeout, Duration::from_millis(50));
    }
}
