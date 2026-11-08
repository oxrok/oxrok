// Counterpart in sigrok:
//   libsigrok/src/hardware/rigol-dg/api.c,
//   libsigrok/src/hardware/rigol-dg/protocol.c,
//   libsigrok/src/hardware/rigol-dg/protocol.h,
//   Copyright (C) 2020 Timo Kokkonen <tjko@iki.fi>,
//   with commits by Ralf, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The wire of a Rigol DG: command tables, what comes back, and what a capture
//! does when it comes back wrong.
//!
//! Every query goes after an `*OPC?`, whose result is dropped, as in C. A DG1000Z, by
//! its Programming Guide, says all of a channel's state in `SOURn:APPL?`, a quoted
//! string of five parts with `DEF` for what a waveform lacks. An MSO5000's table has
//! `SOURn:FUNC?` and one query per value. The counter's `COUN:MEAS?` is five numbers
//! of ten digits: frequency, period, duty cycle, and the widths of a high and a low
//! pulse, all 0 when the counter is off. Nothing on this wire has a check, so a
//! changed digit in a number is a good reading: [`tests`]' sweep of a frame counts
//! what that lets through.

use crate::core::{Error, Result};
use crate::scpi::{self, CmdArg, HwInfo, Scpi};
use std::time::{Duration, Instant};

/// What `psg_commands` has of the two tables. A command a table lacks sends nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmd {
    /// Back to local.
    SetupLocal,
    /// No table has it, as C's is commented out, so `sr_scpi_cmd` selects nothing.
    SelectChannel,
    /// Whether the output is on.
    GetEnabled,
    /// Output on.
    SetEnable,
    /// Output off.
    SetDisable,
    /// A channel's whole state.
    GetSource,
    /// Sets a waveform.
    SetSource,
    /// A channel's waveform alone.
    GetSourceNoParam,
    /// Frequency.
    SetFrequency,
    /// Frequency.
    GetFrequency,
    /// Amplitude.
    SetAmplitude,
    /// Amplitude.
    GetAmplitude,
    /// Offset.
    GetOffset,
    /// Offset.
    SetOffset,
    /// Phase.
    GetPhase,
    /// Phase.
    SetPhase,
    /// A pulse's duty cycle.
    GetDutyPulse,
    /// A pulse's duty cycle.
    SetDutyPulse,
    /// A square's duty cycle.
    GetDutySquare,
    /// A square's duty cycle.
    SetDutySquare,
    /// Whether the counter runs.
    CounterGetEnabled,
    /// Counter on.
    CounterSetEnable,
    /// Counter off.
    CounterSetDisable,
    /// A counter's reading.
    CounterMeasure,
}

/// `cmdset_dg1000z`, without what no code reads, and with `FUNC` to set a waveform where
/// C sends `APPL:x`: with no values that takes 1 kHz, 5 Vpp, 0 V and 0 degrees (the
/// DG1000Z guide, p.1-5 and its `APPL` entries), on an output that may be on, where
/// `FUNC` keeps what is valid for the new waveform (p.2-97, 2-109). The DG800's and
/// DG900's guides were not to be had: they take it too, on C's one table.
const CMDSET_DG1000Z: &[(Cmd, &str)] = &[
    (Cmd::SetupLocal, "SYST:KLOC:STATE OFF"),
    (Cmd::GetEnabled, "OUTP%s:STATE?"),
    (Cmd::SetEnable, "OUTP%s:STATE ON"),
    (Cmd::SetDisable, "OUTP%s:STATE OFF"),
    (Cmd::GetSource, "SOUR%s:APPL?"),
    (Cmd::SetSource, "SOUR%s:FUNC %s"),
    (Cmd::GetFrequency, "SOUR%s:FREQ?"),
    (Cmd::SetFrequency, "SOUR%s:FREQ %f"),
    (Cmd::GetAmplitude, "SOUR%s:VOLT?"),
    (Cmd::SetAmplitude, "SOUR%s:VOLT %f"),
    (Cmd::GetOffset, "SOUR%s:VOLT:OFFS?"),
    (Cmd::SetOffset, "SOUR%s:VOLT:OFFS %f"),
    (Cmd::GetPhase, "SOUR%s:PHAS?"),
    (Cmd::SetPhase, "SOUR%s:PHAS %f"),
    (Cmd::GetDutyPulse, "SOUR%s:FUNC:PULS:DCYC?"),
    (Cmd::SetDutyPulse, "SOUR%s:FUNC:PULS:DCYC %f"),
    (Cmd::GetDutySquare, "SOUR%s:FUNC:SQU:DCYC?"),
    (Cmd::SetDutySquare, "SOUR%s:FUNC:SQU:DCYC %f"),
    (Cmd::CounterGetEnabled, "COUN:STAT?"),
    (Cmd::CounterSetEnable, "COUN:STAT ON"),
    (Cmd::CounterSetDisable, "COUN:STAT OFF"),
    (Cmd::CounterMeasure, "COUN:MEAS?"),
];

/// `cmdset_mso5000`.
const CMDSET_MSO5000: &[(Cmd, &str)] = &[
    (Cmd::GetEnabled, "SOUR%s:OUTPUT:STATE?"),
    (Cmd::SetEnable, "SOUR%s:OUTPUT:STATE ON"),
    (Cmd::SetDisable, "SOUR%s:OUTPUT:STATE OFF"),
    (Cmd::SetSource, "SOUR%s:FUNC %s"),
    (Cmd::GetSourceNoParam, "SOUR%s:FUNC?"),
    (Cmd::GetFrequency, "SOUR%s:FREQ?"),
    (Cmd::SetFrequency, "SOUR%s:FREQ %f"),
    (Cmd::GetAmplitude, "SOUR%s:VOLT?"),
    (Cmd::SetAmplitude, "SOUR%s:VOLT %f"),
    (Cmd::GetOffset, "SOUR%s:VOLT:OFFS?"),
    (Cmd::SetOffset, "SOUR%s:VOLT:OFFS %f"),
    (Cmd::GetPhase, "SOUR%s:PHAS?"),
    (Cmd::SetPhase, "SOUR%s:PHAS %f"),
    (Cmd::GetDutyPulse, "SOUR%s:PULS:DCYC?"),
    (Cmd::SetDutyPulse, "SOUR%s:PULS:DCYC %f"),
];

/// A waveform takes a frequency.
pub const WFO_FREQUENCY: u8 = 1;
/// A waveform takes an amplitude.
pub const WFO_AMPLITUDE: u8 = 2;
/// A waveform takes an offset.
pub const WFO_OFFSET: u8 = 4;
/// A waveform takes a phase.
pub const WFO_PHASE: u8 = 8;
/// A waveform takes a duty cycle.
pub const WFO_DUTY_CYCLE: u8 = 16;
const DEFAULT: u8 = WFO_FREQUENCY | WFO_AMPLITUDE | WFO_OFFSET | WFO_PHASE;

/// `struct waveform_spec`, where `waveform_type` is needed only for a duty cycle's
/// command.
#[derive(Debug, PartialEq)]
pub struct WaveformSpec {
    /// What the table's commands say.
    pub scpi_name: &'static str,
    /// What `pattern` says.
    pub user_name: &'static str,
    /// The least frequency.
    pub freq_min: f64,
    /// The most frequency.
    pub freq_max: f64,
    /// The frequency's step.
    pub freq_step: f64,
    /// What it takes: `WFO_*`.
    pub opts: u8,
}

const fn wf(
    scpi_name: &'static str,
    user_name: &'static str,
    freq_min: f64,
    freq_max: f64,
    freq_step: f64,
    opts: u8,
) -> WaveformSpec {
    WaveformSpec { scpi_name, user_name, freq_min, freq_max, freq_step, opts }
}

/// The seven waveforms of a DG, by the tops that make its table in `api.c`.
const fn dg(
    sine: f64,
    square: f64,
    ramp: f64,
    pulse: f64,
    arb: f64,
    noise: f64,
) -> [WaveformSpec; 7] {
    const LOW: f64 = 1.0e-6;
    [
        wf("SIN", "Sine", LOW, sine, LOW, DEFAULT),
        wf("SQU", "Square", LOW, square, LOW, DEFAULT | WFO_DUTY_CYCLE),
        wf("RAMP", "Ramp", LOW, ramp, LOW, DEFAULT),
        wf("PULSE", "Pulse", LOW, pulse, LOW, DEFAULT | WFO_DUTY_CYCLE),
        wf("USER", "Arb", LOW, arb, LOW, DEFAULT),
        wf("NOISE", "Noise", noise, noise, 0.0, WFO_AMPLITUDE | WFO_OFFSET),
        wf("DC", "DC", 0.0, 0.0, 0.0, WFO_OFFSET),
    ]
}

static DG810: [WaveformSpec; 7] = dg(10.0e6, 5.0e6, 0.2e6, 5.0e6, 5.0e6, 100.0e6);
static DG820: [WaveformSpec; 7] = dg(25.0e6, 10.0e6, 0.5e6, 10.0e6, 10.0e6, 100.0e6);
static DG830: [WaveformSpec; 7] = dg(35.0e6, 10.0e6, 1.0e6, 10.0e6, 10.0e6, 100.0e6);
static DG952: [WaveformSpec; 7] = dg(50.0e6, 15.0e6, 1.5e6, 15.0e6, 15.0e6, 100.0e6);
static DG972: [WaveformSpec; 7] = dg(70.0e6, 20.0e6, 1.5e6, 20.0e6, 20.0e6, 100.0e6);
static DG992: [WaveformSpec; 7] = dg(100.0e6, 25.0e6, 2.0e6, 25.0e6, 25.0e6, 100.0e6);
static DG1022Z: [WaveformSpec; 7] = dg(25.0e6, 25.0e6, 0.5e6, 15.0e6, 10.0e6, 25.0e6);
static DG1032Z: [WaveformSpec; 7] = dg(30.0e6, 25.0e6, 0.5e6, 15.0e6, 10.0e6, 30.0e6);
static DG1062Z: [WaveformSpec; 7] = dg(60.0e6, 25.0e6, 1.0e6, 25.0e6, 20.0e6, 60.0e6);

const MSO_LOW: f64 = 100.0e-3;
static MSO5000: [WaveformSpec; 14] = [
    wf("SIN", "Sine", MSO_LOW, 25.0e6, MSO_LOW, DEFAULT),
    wf("SQU", "Square", MSO_LOW, 15.0e6, MSO_LOW, DEFAULT),
    wf("RAMP", "Ramp", MSO_LOW, 100.0e3, MSO_LOW, DEFAULT),
    wf("PULS", "Pulse", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT | WFO_DUTY_CYCLE),
    wf("USER", "Arb", MSO_LOW, 10.0e6, MSO_LOW, DEFAULT),
    wf("NOIS", "Noise", 60.0e6, 60.0e6, 0.0, WFO_AMPLITUDE | WFO_OFFSET),
    wf("DC", "DC", 0.0, 0.0, 0.0, WFO_OFFSET),
    wf("SINC", "Sinc", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
    wf("EXPR", "ExpRise", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
    wf("EXPF", "ExpFall", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
    wf("ECG", "ECG", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
    wf("GAUS", "Gauss", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
    wf("LOR", "Lorentz", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
    wf("HAV", "Haversin", MSO_LOW, 1.0e6, MSO_LOW, DEFAULT),
];

/// `dg1000z_phase_min_max_step`.
const DG_PHASE: [f64; 3] = [0.0, 360.0, 0.001];
/// `mso5000_phase_min_max_step`.
const MSO_PHASE: [f64; 3] = [0.0, 360.0, 0.1];

/// `struct device_spec`, with its `channel_spec`s, as a model's are all alike.
#[derive(Debug)]
pub struct DeviceSpec {
    /// The model, as `*IDN?` says it, in any case.
    pub model: &'static str,
    /// Its generator's channels.
    pub channels: usize,
    /// Each channel's waveforms.
    pub waveforms: &'static [WaveformSpec],
    /// Each channel's phase: least, most, step.
    pub phase: [f64; 3],
    /// Its commands.
    pub cmdset: &'static [(Cmd, &'static str)],
    /// A frequency counter, and with it its four channels and the limit keys.
    pub has_counter: bool,
}

const fn dgm(
    model: &'static str,
    channels: usize,
    waveforms: &'static [WaveformSpec],
) -> DeviceSpec {
    DeviceSpec {
        model,
        channels,
        waveforms,
        phase: DG_PHASE,
        cmdset: CMDSET_DG1000Z,
        has_counter: true,
    }
}

const fn mso(model: &'static str) -> DeviceSpec {
    DeviceSpec {
        model,
        channels: 2,
        waveforms: &MSO5000,
        phase: MSO_PHASE,
        cmdset: CMDSET_MSO5000,
        has_counter: false,
    }
}

/// `device_models`.
pub static MODELS: [DeviceSpec; 18] = [
    dgm("DG811", 1, &DG810),
    dgm("DG812", 2, &DG810),
    dgm("DG821", 1, &DG820),
    dgm("DG822", 2, &DG820),
    dgm("DG831", 1, &DG830),
    dgm("DG832", 2, &DG830),
    dgm("DG952", 2, &DG952),
    dgm("DG972", 2, &DG972),
    dgm("DG992", 2, &DG992),
    dgm("DG1022Z", 2, &DG1022Z),
    dgm("DG1032Z", 2, &DG1032Z),
    dgm("DG1062Z", 2, &DG1062Z),
    mso("MSO5354"),
    mso("MSO5204"),
    mso("MSO5104"),
    mso("MSO5102"),
    mso("MSO5074"),
    mso("MSO5072"),
];

/// The table's entry for what `*IDN?` said: maker and model, in any case.
pub fn find_model(id: &HwInfo) -> Option<&'static DeviceSpec> {
    if !id.manufacturer.eq_ignore_ascii_case("Rigol Technologies") {
        return None;
    }
    MODELS.iter().find(|m| m.model.eq_ignore_ascii_case(&id.model))
}

impl DeviceSpec {
    /// `check_device_quirks`: the DG800 and DG900's counter wants a second each way.
    pub fn counter_needs_delay(&self) -> bool {
        let start = self.model.get(..3).unwrap_or("");
        start.eq_ignore_ascii_case("DG8") || start.eq_ignore_ascii_case("DG9")
    }

    /// A DG800 or DG900 with a channel 2, which by C's commit can't run with the
    /// counter: its check is dead in C, see [`super`].
    pub fn counter_conflicts_with_channel_2(&self) -> bool {
        self.counter_needs_delay() && self.channels > 1
    }

    /// `rigol_dg_string_to_waveform`, with the longest name that fits: C takes the
    /// first, so an MSO5000's `SINC` is its `SIN`, which reads as `Sine`.
    pub fn waveform(&self, text: &str) -> Option<&'static WaveformSpec> {
        let fits = |name: &str| {
            text.get(..name.len()).is_some_and(|start| start.eq_ignore_ascii_case(name))
        };
        self.waveforms
            .iter()
            .filter_map(|w| {
                [w.scpi_name, w.user_name]
                    .iter()
                    .filter(|name| fits(name))
                    .map(|name| name.len())
                    .max()
                    .map(|length| (length, w))
            })
            .fold(None, |best: Option<(usize, &'static WaveformSpec)>, (length, w)| match best {
                Some((longest, _)) if longest >= length => best,
                _ => Some((length, w)),
            })
            .map(|(_, w)| w)
    }

    /// The waveform `pattern=` names: whole, in any case.
    pub fn waveform_named(&self, name: &str) -> Option<&'static WaveformSpec> {
        self.waveforms.iter().find(|w| w.user_name.eq_ignore_ascii_case(name))
    }
}

/// A number as the instrument writes one: a sign, digits and a point, an exponent.
/// `g_ascii_strtod` takes anything, gives 0 of what it can't read and stops where it
/// must, so that `1.5xyz` is 1.5 and `abc` is a reading of 0; and hexadecimal, `inf`
/// and `nan`, which the guide's numbers never are.
pub fn parse_number(text: &str) -> Option<f64> {
    // Rust's own form: no hexadecimal, no trailing text. It has `inf` and `nan`.
    text.trim().parse::<f64>().ok().filter(|v| v.is_finite())
}

/// `g_strstrip` and `sr_scpi_unquote_string`.
fn clean(answer: &str) -> String {
    scpi::unquote_string(answer.trim())
}

/// What a channel says of itself. A value it did not give is `None`: `get` says so,
/// where C keeps what it had, 0 at first.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChannelState {
    /// Its waveform.
    pub waveform: &'static WaveformSpec,
    /// Hz; 0 for `DEF`.
    pub frequency: Option<f64>,
    /// Vpp; 0 for `DEF`.
    pub amplitude: Option<f64>,
    /// V; 0 for `DEF`.
    pub offset: Option<f64>,
    /// Degrees; 0 for `DEF`.
    pub phase: Option<f64>,
}

/// One part of `SOURn:APPL?`: `DEF` is the guide's word for what a waveform lacks, and
/// comes to 0, as C has it.
fn apply_field(field: Option<&str>) -> Option<f64> {
    let field = field?;
    if field.trim().eq_ignore_ascii_case("DEF") { Some(0.0) } else { parse_number(field) }
}

/// `SOURn:APPL?`'s answer, for a DG. What this fails is no state at all.
pub fn parse_apply(spec: &DeviceSpec, answer: &str) -> Result<ChannelState> {
    let text = clean(answer);
    let mut parts = text.split(',');
    let first = parts.next().unwrap_or("");
    if first.is_empty() {
        return Err(Error::device(format!("No usable answer about a channel: '{answer}'")));
    }
    let Some(waveform) = spec.waveform(first) else {
        log::warn!("Unknown waveform: {first}");
        return Err(Error::device(format!("Unknown waveform: {first}")));
    };
    Ok(ChannelState {
        waveform,
        frequency: apply_field(parts.next()),
        amplitude: apply_field(parts.next()),
        offset: apply_field(parts.next()),
        phase: apply_field(parts.next()),
    })
}

/// One `COUN:MEAS?`.
pub type Measure = [f64; 5];

/// `COUN:MEAS?`'s answer: five numbers, and what follows them goes unread, as in C.
/// `None` for fewer, where C sends nothing either, and for what isn't a number, where
/// C sends 0 or the digits it found.
pub fn parse_measure(answer: &str) -> Option<Measure> {
    let mut parts = answer.trim().split(',');
    let mut out = [0.0; 5];
    for value in &mut out {
        *value = parse_number(parts.next()?)?;
    }
    Some(out)
}

/// Why a value outside `range` isn't sent: the instrument would set the nearest limit
/// in its place and say nothing (the DG1000Z guide, p.2-97, 2-155, 2-182, 2-185).
pub fn out_of_range(key: &str, value: f64, range: [f64; 2], unit: &str) -> Option<String> {
    if value >= range[0] && value <= range[1] {
        return None;
    }
    Some(format!(
        "{key} {value} {unit} is outside {} to {}, and the instrument would set the nearest \
         limit instead: not sent",
        range[0], range[1]
    ))
}

/// The warning where the limits are not to be had, as nothing then stands between a
/// value and the instrument's clamp.
pub fn limits_unknown(key: &str) -> String {
    format!(
        "The instrument's limits for {key} are not known here: a value outside them is set to \
         the nearest limit."
    )
}

/// The pulse duty cycle the guide gives (p.2-106); a short period narrows it further.
pub const PULSE_DUTY: [f64; 2] = [0.001, 99.999];

/// How long `*OPC?` may go unanswered once one has gone unanswered: the instrument
/// isn't answering, and a close should not take [`Scpi::get_opc`]'s 30 s more than once.
const DOUBTFUL_OPC: Duration = Duration::from_secs(2);

/// One instrument's connection.
pub struct Link {
    /// Closed, between an open and a close.
    pub scpi: Scpi,
    /// What it is.
    pub spec: &'static DeviceSpec,
    /// [`Link::open`] happened.
    pub is_open: bool,
    /// `opc_silence` as it came, for when the instrument answers again.
    opc_silence: Duration,
    /// `devc->counter_enabled`: the counter ran already at a capture's start.
    pub counter_was_running: bool,
}

impl Link {
    /// A closed connection to `spec`.
    pub fn new(scpi: Scpi, spec: &'static DeviceSpec) -> Link {
        let opc_silence = scpi.opc_silence;
        Link { scpi, spec, is_open: false, opc_silence, counter_was_running: false }
    }

    /// `sr_scpi_get_opc`, whose result C drops everywhere: what follows finds out.
    pub fn opc(&mut self) {
        self.opc_while(|| true);
    }

    /// [`Link::opc`] that stops when `go_on` says.
    pub fn opc_while(&mut self, go_on: impl Fn() -> bool) {
        match self.scpi.get_opc_while(go_on) {
            Ok(()) => self.scpi.opc_silence = self.opc_silence,
            Err(e) => {
                log::debug!("*OPC?: {e}.");
                self.scpi.opc_silence = self.opc_silence.min(DOUBTFUL_OPC);
            }
        }
    }

    /// An answer came to something else: `*OPC?` may be asked for long again.
    pub fn heard(&mut self) {
        self.scpi.opc_silence = self.opc_silence;
    }

    /// The instrument didn't answer: what `*OPC?` it gets next may not wait long.
    pub fn doubtful(&mut self) {
        self.scpi.opc_silence = self.opc_silence.min(DOUBTFUL_OPC);
    }

    /// Whether this instrument's table has `command`.
    pub fn has(&self, command: Cmd) -> bool {
        scpi::cmd_get(self.spec.cmdset, &command).is_some_and(|text| !text.is_empty())
    }

    /// `PSG_CMD_SETUP_LOCAL`, with its `*OPC?`, if the table has it.
    pub fn setup_local(&mut self) {
        if !self.has(Cmd::SetupLocal) {
            return;
        }
        self.opc();
        if let Err(e) = self.scpi.cmd(self.spec.cmdset, None, &Cmd::SetupLocal, &[]) {
            log::debug!("Back to local: {e}.");
        }
    }

    /// `sr_scpi_cmd_resp` on a channel.
    pub fn ask(&mut self, command: Cmd, group: &str) -> Result<String> {
        let channel = Some((Cmd::SelectChannel, group));
        self.scpi.cmd_resp(self.spec.cmdset, channel, &command, &[CmdArg::Str(group)])
    }

    /// What `command`, a query, says of its limits: `X? MIN` and `X? MAX`, each after an
    /// `*OPC?`. `None` where either is not a number or doesn't come.
    pub fn limits(&mut self, command: Cmd, group: &str) -> Option<[f64; 2]> {
        let template = scpi::cmd_get(self.spec.cmdset, &command)?;
        let query = scpi::render(template, &[CmdArg::Str(group)]);
        let mut out = [0.0; 2];
        for (slot, which) in out.iter_mut().zip(["MIN", "MAX"]) {
            self.opc();
            match self.scpi.get_string(&format!("{query} {which}")) {
                Ok(answer) => *slot = parse_number(&clean(&answer))?,
                Err(e) => {
                    log::debug!("{query} {which}: {e}.");
                    return None;
                }
            }
        }
        Some(out)
    }

    /// `sr_scpi_cmd` on a channel, with what the command takes after its name.
    pub fn tell(&mut self, command: Cmd, group: &str, value: Option<CmdArg<'_>>) -> Result<()> {
        let channel = Some((Cmd::SelectChannel, group));
        let args = [CmdArg::Str(group), value.unwrap_or(CmdArg::Str(""))];
        let given = if value.is_some() { 2 } else { 1 };
        self.scpi.cmd(self.spec.cmdset, channel, &command, &args[..given])
    }

    /// `rigol_dg_get_channel_state`.
    pub fn channel_state(&mut self, group: &str) -> Result<ChannelState> {
        if self.has(Cmd::GetSourceNoParam) {
            self.opc();
            let answer = self.ask(Cmd::GetSourceNoParam, group)?;
            let text = clean(&answer);
            let Some(waveform) = self.spec.waveform(&text) else {
                log::warn!("Unknown waveform: {text}");
                return Err(Error::device(format!("Unknown waveform: {text}")));
            };
            return Ok(ChannelState {
                waveform,
                frequency: self.double_param(Cmd::GetFrequency, group),
                amplitude: self.double_param(Cmd::GetAmplitude, group),
                offset: self.double_param(Cmd::GetOffset, group),
                phase: self.double_param(Cmd::GetPhase, group),
            });
        }
        self.opc();
        let answer = self.ask(Cmd::GetSource, group)?;
        parse_apply(self.spec, &answer)
    }

    /// `rigol_dg_get_double_param`, whose failure C ignores on the way out of its
    /// caller, if a `g_variant_unref(NULL)` doesn't end sigrok-cli first.
    fn double_param(&mut self, command: Cmd, group: &str) -> Option<f64> {
        self.opc();
        match self.ask(command, group) {
            Ok(answer) => parse_number(&clean(&answer)),
            Err(e) => {
                log::debug!("{command:?}: {e}.");
                None
            }
        }
    }

    /// `COUN:MEAS?` and the `*OPC?` before it.
    pub fn measure(&mut self, go_on: &impl Fn() -> bool) -> Round {
        self.opc_while(go_on);
        if !go_on() {
            return Round::Stopped;
        }
        match self.scpi.cmd_resp(self.spec.cmdset, None, &Cmd::CounterMeasure, &[]) {
            Ok(answer) => {
                self.heard();
                if let Some(measure) = parse_measure(&answer) {
                    return Round::Reading(measure);
                }
                // It may be another query's, late: get back in step.
                let quiet = self.scpi.read_timeout;
                if let Err(e) = self.scpi.discard(quiet) {
                    log::debug!("Looking at the line: {e}.");
                }
                Round::Junk(answer)
            }
            Err(Error::Timeout) => {
                log::info!("Error getting measurement from counter: {}.", Error::Timeout);
                self.doubtful();
                Round::Silent
            }
            Err(e) => {
                log::info!("Error getting measurement from counter: {e}.");
                Round::Failed(e)
            }
        }
    }
}

/// What one asking of the counter came to.
#[derive(Debug)]
pub enum Round {
    /// A reading.
    Reading(Measure),
    /// An answer that makes none.
    Junk(String),
    /// No answer.
    Silent,
    /// A stop came first.
    Stopped,
    /// The line failed.
    Failed(Error),
}

/// How long a counter may give no answer before its first reading. The guide's gate
/// times go up to 10.73 s and past it: it doesn't say a query waits for a gate, and C
/// gives up at the first read's 1 s, so this is a margin over the slowest gate named.
pub const SILENT_FIRST: Duration = Duration::from_secs(30);
/// How long it may give no answer after a reading, where C ends at the first.
pub const SILENT_AFTER: Duration = Duration::from_secs(600);
/// How long it may answer what makes no reading before its first.
pub const JUNK_FIRST: Duration = Duration::from_secs(10);
/// How long it may answer what makes no reading after one.
pub const JUNK_AFTER: Duration = Duration::from_secs(60);
/// The most time a capture is excused for not having run.
pub const CREDIT_CAP: Duration = Duration::from_secs(60);

/// How long past a time limit a round may wait on a silent `*OPC?`.
pub const TIME_GRACE: Duration = Duration::from_secs(5);

/// What a capture does next.
#[derive(Debug, PartialEq)]
pub enum Next {
    /// Asks again.
    GoOn,
    /// A limit is met.
    Done,
    /// It failed, and why.
    Fail(String),
}

/// The limits, as they are now: a limit set on a running capture counts at once.
#[derive(Clone, Copy, Debug, Default)]
pub struct Limits {
    /// `limit_samples`; 0 for none.
    pub samples: u64,
    /// `limit_msec`; 0 for none.
    pub msec: u64,
}

/// A capture's rule, in its times and with what it says: what follows each round.
#[derive(Debug)]
pub struct Patience {
    started: Instant,
    /// When the counter last made a reading, or the capture's start.
    heard: Instant,
    samples: u64,
    /// When what makes no reading began to come, with nothing silent since.
    junk_since: Option<Instant>,
    /// Time the thread didn't run, which isn't the counter's.
    credit: Duration,
    /// Said that it's silent, and hasn't said it's back.
    said_silent: bool,
    /// Said that it isn't making sense, and hasn't said it's back.
    said_junk: bool,
    bounds: Bounds,
}

/// [`Patience`]'s bounds, which a test sets short.
#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    /// [`SILENT_FIRST`].
    pub silent_first: Duration,
    /// [`SILENT_AFTER`].
    pub silent_after: Duration,
    /// [`JUNK_FIRST`].
    pub junk_first: Duration,
    /// [`JUNK_AFTER`].
    pub junk_after: Duration,
}

impl Default for Bounds {
    fn default() -> Bounds {
        Bounds {
            silent_first: SILENT_FIRST,
            silent_after: SILENT_AFTER,
            junk_first: JUNK_FIRST,
            junk_after: JUNK_AFTER,
        }
    }
}

/// The shortest of an answer, in a message.
fn shown(text: &str) -> String {
    text.chars().take(70).collect()
}

impl Patience {
    /// A capture starting at `now`.
    pub fn new(now: Instant, bounds: Bounds) -> Patience {
        Patience {
            started: now,
            heard: now,
            samples: 0,
            junk_since: None,
            credit: Duration::ZERO,
            said_silent: false,
            said_junk: false,
            bounds,
        }
    }

    /// The readings so far.
    pub fn samples(&self) -> u64 {
        self.samples
    }

    fn time_up(&self, now: Instant, limits: Limits) -> bool {
        limits.msec > 0
            && now.saturating_duration_since(self.started) >= Duration::from_millis(limits.msec)
    }

    /// Whether a time limit ran out some while ago, for a round that isn't getting
    /// anywhere: the first reading may come after a limit shorter than the poll, as a
    /// limit is looked at after a reading, so a round has [`TIME_GRACE`] past it.
    pub fn out_of_time(&self, now: Instant, limits: Limits) -> bool {
        limits.msec > 0
            && now.saturating_duration_since(self.started)
                >= Duration::from_millis(limits.msec) + TIME_GRACE
    }

    /// The thread lost `stall` that wasn't the counter's.
    pub fn excuse(&mut self, stall: Duration) {
        self.credit = (self.credit + stall).min(CREDIT_CAP);
    }

    /// A reading went to the queue, at `now`. With it, a word if there was a warning
    /// to take back.
    pub fn reading(&mut self, now: Instant, limits: Limits) -> (Next, Option<&'static str>) {
        self.samples += 1;
        self.heard = now;
        self.credit = Duration::ZERO;
        self.junk_since = None;
        let back = (self.said_silent || self.said_junk).then_some("The counter answers again.");
        self.said_silent = false;
        self.said_junk = false;
        let done = limits.samples > 0 && self.samples >= limits.samples;
        (if done || self.time_up(now, limits) { Next::Done } else { Next::GoOn }, back)
    }

    fn since(&self, then: Instant, asked: Instant) -> Duration {
        asked.saturating_duration_since(then).saturating_sub(self.credit)
    }

    /// No answer to a question asked at `asked`, which is what counts: a stall after
    /// that is no silence of the counter's. Warns once, and a time limit with nothing
    /// captured ends well with a word on it.
    pub fn silent(
        &mut self,
        asked: Instant,
        now: Instant,
        limits: Limits,
    ) -> (Next, Option<String>) {
        self.junk_since = None;
        let bound =
            if self.samples == 0 { self.bounds.silent_first } else { self.bounds.silent_after };
        if self.time_up(now, limits) {
            return (Next::Done, self.empty_time(now));
        }
        if self.since(self.heard, asked) >= bound {
            return (
                Next::Fail(format!("no answer from the counter in {} s", bound.as_secs_f32())),
                None,
            );
        }
        let say =
            (!self.said_silent).then(|| "No answer from the counter: asking again.".to_owned());
        self.said_silent = true;
        (Next::GoOn, say)
    }

    /// An answer that makes no reading, asked at `asked`. Bytes that make nothing are a
    /// quicker failure than silence. The time runs from the first of them, or from what
    /// was last heard, whichever is later: silence in between has its own bound.
    pub fn junk(
        &mut self,
        answer: &str,
        asked: Instant,
        now: Instant,
        limits: Limits,
    ) -> (Next, Option<String>) {
        let since = *self.junk_since.get_or_insert(asked);
        if self.time_up(now, limits) {
            return (Next::Done, self.empty_time(now));
        }
        let bound = if self.samples == 0 { self.bounds.junk_first } else { self.bounds.junk_after };
        if self.since(since, asked) >= bound {
            return (
                Next::Fail(format!(
                    "no usable answer from the counter in {} s: '{}'",
                    bound.as_secs_f32(),
                    shown(answer)
                )),
                None,
            );
        }
        let say = (!self.said_junk)
            .then(|| format!("No usable answer from the counter: '{}'.", shown(answer)));
        self.said_junk = true;
        (Next::GoOn, say)
    }

    fn empty_time(&self, now: Instant) -> Option<String> {
        (self.samples == 0).then(|| {
            format!(
                "Nothing from the counter in {:.1} s of a time limit: nothing was captured.",
                now.saturating_duration_since(self.started).as_secs_f32()
            )
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    fn spec(model: &str) -> &'static DeviceSpec {
        MODELS.iter().find(|m| m.model == model).unwrap()
    }

    fn id(manufacturer: &str, model: &str) -> HwInfo {
        HwInfo {
            manufacturer: manufacturer.to_owned(),
            model: model.to_owned(),
            ..HwInfo::default()
        }
    }

    #[test]
    fn a_model_is_found_in_any_case_and_only_of_this_maker() {
        assert_eq!(find_model(&id("RIGOL TECHNOLOGIES", "dg1062z")).unwrap().model, "DG1062Z");
        assert!(find_model(&id("Rigol", "DG1062Z")).is_none());
        assert!(find_model(&id("Rigol Technologies", "DG1062")).is_none());
    }

    #[test]
    fn the_quirks_go_by_series() {
        assert!(spec("DG811").counter_needs_delay());
        assert!(spec("DG992").counter_needs_delay());
        assert!(!spec("DG1022Z").counter_needs_delay());
        assert!(!spec("MSO5074").counter_needs_delay());
        assert!(!spec("DG811").counter_conflicts_with_channel_2());
        assert!(spec("DG812").counter_conflicts_with_channel_2());
        assert!(!spec("DG1062Z").counter_conflicts_with_channel_2());
    }

    #[test]
    fn a_table_is_as_c_has_it() {
        let dg = spec("DG1032Z").waveforms;
        assert_eq!(dg[0].freq_max, 30.0e6);
        assert_eq!(dg[1].freq_max, 25.0e6);
        assert_eq!(dg[2].freq_max, 0.5e6);
        assert_eq!(dg[3].freq_max, 15.0e6);
        assert_eq!(dg[4].freq_max, 10.0e6);
        assert_eq!(dg[5].freq_min, 30.0e6);
        assert_eq!(dg[5].opts, WFO_AMPLITUDE | WFO_OFFSET);
        assert_eq!(dg[6].opts, WFO_OFFSET);
        assert_eq!(spec("DG811").waveforms[5].freq_max, 100.0e6);
        assert_eq!(spec("DG811").channels, 1);
        assert_eq!(spec("DG992").waveforms[0].freq_max, 100.0e6);
        assert_eq!(spec("MSO5354").waveforms[1].opts, DEFAULT, "a square has no duty there");
        assert_eq!(spec("MSO5354").phase[2], 0.1);
        assert!(!spec("MSO5354").has_counter);
    }

    #[test]
    fn a_waveform_takes_the_longest_name_that_fits() {
        let mso = spec("MSO5074");
        assert_eq!(mso.waveform("SINC").unwrap().user_name, "Sinc");
        assert_eq!(mso.waveform("sinc").unwrap().user_name, "Sinc");
        assert_eq!(mso.waveform("SIN").unwrap().user_name, "Sine");
        assert_eq!(mso.waveform("SINE").unwrap().user_name, "Sine");
        assert_eq!(mso.waveform("Sine").unwrap().user_name, "Sine");
        assert_eq!(mso.waveform("EXPF").unwrap().user_name, "ExpFall");
        assert!(mso.waveform("FOO").is_none());
        let dg = spec("DG1062Z");
        assert_eq!(dg.waveform("PULSE").unwrap().user_name, "Pulse");
        assert!(dg.waveform("PULS").is_none(), "a DG's is PULSE");
        assert_eq!(dg.waveform("Arb").unwrap().scpi_name, "USER");
        assert!(dg.waveform_named("SIN").is_none(), "pattern= says the name");
        assert_eq!(dg.waveform_named("pULSE").unwrap().scpi_name, "PULSE");
    }

    #[test]
    fn a_number_is_what_the_guide_writes() {
        for (text, value) in [
            ("1.000000E+03", 1000.0),
            ("-2.5e-1", -0.25),
            ("+3", 3.0),
            (" 4.5 ", 4.5),
            (".5", 0.5),
            ("5.", 5.0),
        ] {
            assert_eq!(parse_number(text), Some(value), "{text}");
        }
        for text in [
            "", "abc", "1.5xyz", "0x10", "inf", "nan", "-", ".", "1e", "1e+", "1.2.3", "1e5e5",
            "1e999", "1 2", "--1", "DEF",
        ] {
            assert_eq!(parse_number(text), None, "{text}");
        }
    }

    #[test]
    fn apply_gives_what_it_says() {
        let dg = spec("DG1062Z");
        let state =
            parse_apply(dg, "\"SQU,1.000000E+03,2.000000E+00,3.000000E+00,4.000000E+00\"\n")
                .unwrap();
        assert_eq!(state.waveform.user_name, "Square");
        assert_eq!(state.frequency, Some(1000.0));
        assert_eq!(state.amplitude, Some(2.0));
        assert_eq!(state.offset, Some(3.0));
        assert_eq!(state.phase, Some(4.0));
        let state = parse_apply(dg, "\"DC,DEF,def,1.000000E+00,DEF\"").unwrap();
        assert_eq!(state.frequency, Some(0.0), "DEF, the guide's word, comes to 0 as in C");
        assert_eq!(state.amplitude, Some(0.0));
        assert_eq!(state.offset, Some(1.0));
    }

    #[test]
    fn apply_leaves_out_what_it_cannot_read() {
        let dg = spec("DG1062Z");
        let state = parse_apply(dg, "SIN,1000").unwrap();
        assert_eq!(state.frequency, Some(1000.0));
        assert_eq!(state.amplitude, None);
        assert_eq!(state.phase, None);
        let state = parse_apply(dg, "\"SIN,abc,1.5xyz,,4\"").unwrap();
        assert_eq!(state.frequency, None);
        assert_eq!(state.amplitude, None);
        assert_eq!(state.offset, None);
        assert_eq!(state.phase, Some(4.0));
        // Words C's check would pin only as a crash or a line.
        let e = parse_apply(dg, "\"\"").unwrap_err().to_string();
        assert!(e.contains("No usable answer about a channel"), "{e}");
        let e = parse_apply(dg, "FOO,1,2,3,4").unwrap_err().to_string();
        assert!(e.contains("Unknown waveform: FOO"), "{e}");
    }

    #[test]
    fn a_measure_is_five_numbers() {
        let m = parse_measure(
            "2.000000000E+03,5.000000000E-04,4.760800000E+01,2.380415000E-04, 2.619585000E-04\n",
        );
        assert_eq!(m, Some([2000.0, 5e-4, 47.608, 2.380_415e-4, 2.619_585e-4]));
        assert_eq!(parse_measure("1,2,3,4,5,6"), Some([1.0, 2.0, 3.0, 4.0, 5.0]));
        for text in ["", "1,2,3,4", "a,b,c,d,e", "1,2,3,4,x", "1,2,3,,5", "1"] {
            assert_eq!(parse_measure(text), None, "{text}");
        }
    }

    /// What damage to `frame` comes to: refused, a wrong value, the same value.
    #[allow(clippy::float_cmp)]
    fn damage(frame: &str, cut: bool) -> (usize, usize, usize) {
        let good = parse_measure(frame).unwrap();
        let (mut refused, mut wrong, mut same) = (0, 0, 0);
        let mut judge = |bytes: &[u8]| match parse_measure(&String::from_utf8_lossy(bytes)) {
            None => refused += 1,
            Some(m) if m == good => same += 1,
            Some(_) => wrong += 1,
        };
        for at in 0..frame.len() {
            if cut {
                let mut bytes = frame.as_bytes().to_vec();
                bytes.remove(at);
                judge(&bytes);
            } else {
                for bit in 0..8 {
                    let mut bytes = frame.as_bytes().to_vec();
                    bytes[at] ^= 1 << bit;
                    judge(&bytes);
                }
            }
        }
        (refused, wrong, same)
    }

    /// What a damaged frame comes to, by the guide's frame, one of 0, and one at the top
    /// of the range. No check on this wire, so a digit stays a digit: what can't be
    /// caught here is left to USBTMC's and TCP's own checks below it.
    #[test]
    fn damage_to_a_frame_counted() {
        let guide =
            "2.000000000E+03,5.000000000E-04,4.760800000E+01,2.380415000E-04,2.619585000E-04";
        let off = "0.000000000E+00,0.000000000E+00,0.000000000E+00,0.000000000E+00,0.000000000E+00";
        let top = "9.999999999E+07,1.000000000E-08,9.999900000E+01,9.999900000E-09,1.000000000E-13";
        assert_eq!(damage(guide, false), (407, 220, 5), "flips: refused, wrong, the same");
        assert_eq!(damage(guide, true), (9, 34, 36), "lost bytes: refused, wrong, the same");
        assert_eq!(damage(off, false), (372, 200, 60), "flips of a frame of 0");
        assert_eq!(damage(off, true), (9, 0, 70), "lost bytes of a frame of 0");
        assert_eq!(damage(top, false), (428, 199, 5), "flips at the top");
        assert_eq!(damage(top, true), (9, 36, 34), "lost bytes at the top");
    }

    #[test]
    fn out_of_range_says_why() {
        assert_eq!(out_of_range("output_frequency", 5.0, [0.0, 10.0], "Hz"), None);
        assert_eq!(out_of_range("output_frequency", 10.0, [0.0, 10.0], "Hz"), None);
        assert_eq!(out_of_range("output_frequency", 0.0, [0.0, 10.0], "Hz"), None);
        assert_eq!(
            out_of_range("phase", 400.0, [0.0, 360.0], "degrees").unwrap(),
            "phase 400 degrees is outside 0 to 360, and the instrument would set the nearest \
             limit instead: not sent"
        );
        assert!(out_of_range("phase", -0.5, [0.0, 360.0], "degrees").is_some());
        assert_eq!(
            limits_unknown("amplitude"),
            "The instrument's limits for amplitude are not known here: a value outside them is \
             set to the nearest limit."
        );
        assert_eq!(PULSE_DUTY, [0.001, 99.999]);
    }

    const NONE: Limits = Limits { samples: 0, msec: 0 };
    const S: fn(u64) -> Duration = Duration::from_secs;

    #[test]
    fn silence_before_a_reading_fails_at_its_bound() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        let (next, say) = p.silent(t0 + S(1), t0 + S(2), NONE);
        assert_eq!(next, Next::GoOn);
        assert_eq!(say.as_deref(), Some("No answer from the counter: asking again."));
        assert_eq!(p.silent(t0 + S(29), t0 + S(30), NONE), (Next::GoOn, None), "said once");
        let (next, _) = p.silent(t0 + S(30), t0 + S(31), NONE);
        assert_eq!(next, Next::Fail("no answer from the counter in 30 s".to_owned()));
    }

    #[test]
    fn silence_after_a_reading_has_ten_minutes_and_a_word_when_back() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        assert_eq!(p.reading(t0 + S(1), NONE), (Next::GoOn, None));
        let (next, say) = p.silent(t0 + S(2), t0 + S(3), NONE);
        assert_eq!(next, Next::GoOn);
        assert!(say.is_some());
        assert_eq!(p.silent(t0 + S(600), t0 + S(600), NONE).0, Next::GoOn);
        let (next, back) = p.reading(t0 + S(600), NONE);
        assert_eq!(next, Next::GoOn);
        assert_eq!(back, Some("The counter answers again."));
        assert_eq!(p.reading(t0 + S(601), NONE).1, None, "said once");
        // The bound runs from the last reading: it spans many reads, and no real gap.
        assert_eq!(p.silent(t0 + S(1200), t0 + S(1200), NONE).0, Next::GoOn);
        let (next, say) = p.silent(t0 + S(1201), t0 + S(1201), NONE);
        assert_eq!(next, Next::Fail("no answer from the counter in 600 s".to_owned()));
        assert_eq!(say, None);
    }

    #[test]
    fn the_time_ahead_of_the_read_is_what_counts() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        // Asked in time, and a stall made the read come back late.
        assert_eq!(p.silent(t0 + S(29), t0 + S(100), NONE).0, Next::GoOn);
    }

    #[test]
    fn a_reading_ends_what_was_excused() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        p.excuse(S(10));
        p.reading(t0, NONE);
        assert!(matches!(p.silent(t0 + S(600), t0 + S(600), NONE).0, Next::Fail(_)));
    }

    #[test]
    fn a_stall_is_excused_up_to_a_cap() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        p.excuse(S(10));
        assert_eq!(p.silent(t0 + S(35), t0 + S(35), NONE).0, Next::GoOn);
        assert!(matches!(p.silent(t0 + S(40), t0 + S(40), NONE).0, Next::Fail(_)));
        let mut p = Patience::new(t0, Bounds::default());
        for _ in 0..100 {
            p.excuse(S(10));
        }
        assert_eq!(p.silent(t0 + S(89), t0 + S(89), NONE).0, Next::GoOn);
        assert!(matches!(p.silent(t0 + S(90), t0 + S(90), NONE).0, Next::Fail(_)), "the cap");
    }

    #[test]
    fn junk_is_a_quicker_failure() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        let (next, say) = p.junk("1,2", t0, t0, NONE);
        assert_eq!(next, Next::GoOn);
        assert_eq!(say.as_deref(), Some("No usable answer from the counter: '1,2'."));
        assert_eq!(p.junk("1,2", t0 + S(9), t0 + S(9), NONE), (Next::GoOn, None));
        let (next, _) = p.junk("1,2", t0 + S(10), t0 + S(10), NONE);
        assert_eq!(next, Next::Fail("no usable answer from the counter in 10 s: '1,2'".to_owned()));
        // After a reading it has longer, from the first of them, not from the reading.
        let mut p = Patience::new(t0, Bounds::default());
        p.reading(t0 + S(1), NONE);
        assert_eq!(p.junk("x", t0 + S(100), t0 + S(100), NONE).0, Next::GoOn);
        assert_eq!(p.junk("x", t0 + S(159), t0 + S(159), NONE).0, Next::GoOn);
        assert!(matches!(p.junk("x", t0 + S(160), t0 + S(160), NONE).0, Next::Fail(_)));
    }

    #[test]
    fn a_reading_ends_a_run_of_junk() {
        let t0 = Instant::now();
        let mut p = Patience::new(t0, Bounds::default());
        p.junk("x", t0, t0, NONE);
        let (_, back) = p.reading(t0 + S(9), NONE);
        assert_eq!(back, Some("The counter answers again."));
        assert_eq!(p.junk("x", t0 + S(70), t0 + S(70), NONE).0, Next::GoOn);
        assert!(p.junk("x", t0 + S(129), t0 + S(129), NONE).1.is_none());
        assert_eq!(p.junk("x", t0 + S(129), t0 + S(129), NONE).0, Next::GoOn);
    }

    #[test]
    fn a_limit_is_met_by_readings_and_by_time() {
        let t0 = Instant::now();
        let two = Limits { samples: 2, msec: 0 };
        let mut p = Patience::new(t0, Bounds::default());
        assert_eq!(p.reading(t0, two).0, Next::GoOn);
        assert_eq!(p.reading(t0, two).0, Next::Done);
        assert_eq!(p.samples(), 2);
        let time = Limits { samples: 0, msec: 500 };
        let mut p = Patience::new(t0, Bounds::default());
        assert_eq!(p.reading(t0 + Duration::from_millis(499), time).0, Next::GoOn);
        assert_eq!(p.reading(t0 + Duration::from_millis(500), time).0, Next::Done);
        assert!(!p.out_of_time(t0 + Duration::from_millis(5499), time));
        assert!(p.out_of_time(t0 + Duration::from_millis(5500), time));
        assert!(!p.out_of_time(t0 + S(9999), NONE));
    }

    #[test]
    fn a_time_limit_with_nothing_captured_ends_well_with_a_word() {
        let t0 = Instant::now();
        let time = Limits { samples: 0, msec: 500 };
        let mut p = Patience::new(t0, Bounds::default());
        let (next, say) = p.silent(t0, t0 + S(1), time);
        assert_eq!(next, Next::Done);
        assert_eq!(
            say.as_deref(),
            Some("Nothing from the counter in 1.0 s of a time limit: nothing was captured.")
        );
        let mut p = Patience::new(t0, Bounds::default());
        assert_eq!(p.junk("x", t0, t0 + S(1), time).0, Next::Done);
        let mut p = Patience::new(t0, Bounds::default());
        p.reading(t0, NONE);
        assert_eq!(p.silent(t0, t0 + S(1), time), (Next::Done, None), "something was");
    }

    #[test]
    fn the_bounds_are_these() {
        assert_eq!(SILENT_FIRST, S(30));
        assert_eq!(SILENT_AFTER, S(600));
        assert_eq!(JUNK_FIRST, S(10));
        assert_eq!(JUNK_AFTER, S(60));
        assert_eq!(CREDIT_CAP, S(60));
        assert_eq!(DOUBTFUL_OPC, S(2));
    }
}
