// Counterpart in sigrok:
//   libsigrokdecode/decoders/stepper_motor/pd.py,
//   Copyright (C) 2015 Petteri Aimonen <jpa@sigrok.mail.kapsi.fi>,
//   GPL-2.0-or-later.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the step/dir behaviour and the decoder's documented
//   classes and texts.

//! Stepper motor: absolute position and movement speed from step/dir.

use crate::core::Result;
use crate::decode::{
    AnnClass, AnnRow, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, OptionDesc, OptionValue,
};

const STEP: usize = 0;
const DIR: usize = 1;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_SPEED: usize = 0;
const ANN_POSITION: usize = 1;

static OPTIONS: [OptionDesc; 2] = [
    OptionDesc {
        id: "unit",
        desc: "Unit",
        default: OptionValue::s("steps"),
        values: &[OptionValue::s("steps"), OptionValue::s("mm")],
    },
    OptionDesc {
        id: "steps_per_mm",
        desc: "Steps per mm",
        default: OptionValue::f(100.0),
        values: &[],
    },
];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "stepper_motor",
    name: "Stepper motor",
    longname: "Stepper motor position / speed",
    desc: "Absolute position and movement speed from step/dir.",
    license: "gplv2+",
    inputs: &["logic"],
    outputs: &[],
    tags: &["Embedded/industrial"],
    channels: &[
        ChannelDesc { id: "step", name: "Step", desc: "Step pulse" },
        ChannelDesc { id: "dir", name: "Direction", desc: "Direction select" },
    ],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "speed", desc: "Speed" },
        AnnClass { id: "position", desc: "Position" },
    ],
    annotation_rows: &[
        AnnRow { id: "speed", desc: "Speed", classes: &[0] },
        AnnRow { id: "position", desc: "Position", classes: &[1] },
    ],
    binary: &[],
    doc: "This PD decodes the stepper motor controller signals (step / dir) and\n\
          shows the step speed and absolute position of the stepper motor.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(StepperMotor::new())
}

/// The stepper motor decoder.
pub struct StepperMotor {
    /// Set from metadata; `None` when the input has no sample rate.
    samplerate: Option<u64>,
    ss_prev_step: Option<u64>,
    pos: i64,
    /// `mm` when the unit option is `mm`, `steps` otherwise.
    mm: bool,
    scale: f64,
    /// `%0.0f` for steps, `%0.2f` for mm.
    unit: &'static str,
}

impl StepperMotor {
    fn new() -> Self {
        StepperMotor {
            samplerate: None,
            ss_prev_step: None,
            pos: 0,
            mm: false,
            scale: 1.0,
            unit: "steps",
        }
    }

    /// pd.py's `self.format % value`: `%0.0f` for steps, `%0.2f` for mm.
    fn format(&self, value: f64) -> String {
        if self.mm { format!("{value:.2}") } else { format!("{value:.0}") }
    }

    fn step(&mut self, ctx: &mut dyn Context, ss: u64, direction: bool) {
        if let Some(prev) = self.ss_prev_step {
            if let Some(samplerate) = self.samplerate {
                // delta = ss - self.ss_prev_step
                // speed = self.samplerate / delta / self.scale
                let delta = (ss - prev) as f64;
                let speed = samplerate as f64 / delta / self.scale;
                let speed_txt = self.format(speed);
                ctx.put_ann(
                    prev,
                    ss,
                    ANN_SPEED,
                    vec![format!("{speed_txt} {}/s", self.unit), speed_txt],
                );
            }
            // pos_txt = self.format % (self.pos / self.scale)
            let pos_txt = self.format(self.pos as f64 / self.scale);
            ctx.put_ann(prev, ss, ANN_POSITION, vec![format!("{pos_txt} {}", self.unit), pos_txt]);
        }

        // self.pos += (1 if direction else -1)
        self.pos += if direction { 1 } else { -1 };
        self.ss_prev_step = Some(ss);
    }
}

impl Decoder for StepperMotor {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        if config.str("unit", "steps") == "steps" {
            self.scale = 1.0;
            self.mm = false;
            self.unit = "steps";
        } else {
            self.scale = config.float("steps_per_mm", 100.0);
            self.mm = true;
            self.unit = "mm";
        }
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = Some(samplerate);
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let cond = [Cond::pin(STEP, CondKind::Rising)];
        loop {
            // step, direction = self.wait({0: 'r'})
            let pins = ctx.wait(&[&cond])?;
            let ss = ctx.samplenum();
            let direction = pins.get(DIR);
            self.step(ctx, ss, direction);
        }
    }
}
