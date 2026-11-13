// Counterpart in sigrok:
//   libsigrokdecode/decoders/midi/pd.py and lists.py,
//   Copyright (C) 2013-2016 Uwe Hermann <uwe@hermann-uwe.de>,
//   Copyright (C) 2016 Chris Dreher <chrisdreher@hotmail.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the MIDI protocol and the decoder's documented
//   classes and texts.
#![allow(
    clippy::assigning_clones,
    clippy::collapsible_if,
    clippy::doc_markdown,
    clippy::format_push_string,
    clippy::if_not_else,
    clippy::large_types_passed_by_value,
    clippy::map_unwrap_or,
    clippy::similar_names,
    clippy::unnecessary_wraps
)]

//! MIDI: status bytes, channel messages, SysEx, system common and realtime.

use crate::core::{Error, Result};
use crate::decode::{AnnClass, AnnRow, Config, Context, Decoder, DecoderDef, DecoderInfo, Item};

const ANN_TEXT: usize = 0;
const ANN_SYSREAL: usize = 1;
const ANN_ERROR: usize = 2;

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "midi",
    name: "MIDI",
    longname: "Musical Instrument Digital Interface",
    desc: "Musical Instrument Digital Interface (MIDI) protocol.",
    license: "gplv2+",
    inputs: &["uart"],
    outputs: &[],
    tags: &["Audio", "PC"],
    channels: &[],
    optional_channels: &[],
    options: &[],
    annotations: &[
        AnnClass { id: "text-verbose", desc: "Human-readable text (verbose)" },
        AnnClass { id: "text-sysreal-verbose", desc: "Human-readable SysReal text (verbose)" },
        AnnClass { id: "text-error", desc: "Human-readable Error text" },
    ],
    annotation_rows: &[
        AnnRow { id: "normal", desc: "Normal", classes: &[0, 2] },
        AnnRow { id: "sys-real", desc: "SysReal", classes: &[1] },
    ],
    binary: &[],
    doc: "This decoder stacks on top of the 'uart' PD and decodes the MIDI\n\
(Musical Instrument Digital Interface) protocol.\n\
\n\
MIDI is layered on top of the UART (async serial) protocol, with a fixed\n\
baud rate of 31250 baud (+/- 1%) and 8n1 settings. Bytes are sent LSB-first.",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(Midi::default())
}

// status_bytes
static STATUS_BYTES: &[(u8, [&str; 3])] = &[
    (0x80, ["note off", "note off", "N off"]),
    (0x90, ["note on", "note on", "N on"]),
    (0xa0, ["polyphonic key pressure / aftertouch", "key pressure", "KP"]),
    (0xb0, ["control change", "ctrl chg", "CC"]),
    (0xc0, ["program change", "prgm chg", "PC"]),
    (0xd0, ["channel pressure / aftertouch", "channel pressure", "CP"]),
    (0xe0, ["pitch bend change", "pitch bend", "PB"]),
    (0xf0, ["system exclusive", "SysEx", "SE"]),
    (0xf1, ["MIDI time code quarter frame", "MIDI time code", "MIDI time"]),
    (0xf2, ["song position pointer", "song position", "song pos"]),
    (0xf3, ["song select", "song select", "song sel"]),
    (0xf4, ["undefined 0xf4", "undef 0xf4", "undef"]),
    (0xf5, ["undefined 0xf5", "undef 0xf5", "undef"]),
    (0xf6, ["tune request", "tune request", "tune req"]),
    (0xf7, ["end of system exclusive (EOX)", "end of SysEx", "EOX"]),
    (0xf8, ["timing clock", "timing clock", "clock"]),
    (0xf9, ["undefined 0xf9", "undef 0xf9", "undef"]),
    (0xfa, ["start", "start", "s"]),
    (0xfb, ["continue", "continue", "cont"]),
    (0xfc, ["stop", "stop", "st"]),
    (0xfd, ["undefined 0xfd", "undef 0xfd", "undef"]),
    (0xfe, ["active sensing", "active sensing", "sensing"]),
    (0xff, ["system reset", "reset", "rst"]),
];

// chromatic_notes
static CHROMATIC_NOTES: [&str; 128] = [
    "C-1", "C#-1", "D-1", "D#-1", "E-1", "F-1", "F#-1", "G-1", "G#-1", "A-1", "A#-1", "B-1", "C0",
    "C#0", "D0", "D#0", "E0", "F0", "F#0", "G0", "G#0", "A0", "A#0", "B0", "C1", "C#1", "D1",
    "D#1", "E1", "F1", "F#1", "G1", "G#1", "A1", "A#1", "B1", "C2", "C#2", "D2", "D#2", "E2", "F2",
    "F#2", "G2", "G#2", "A2", "A#2", "B2", "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3",
    "G#3", "A3", "A#3", "B3", "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4",
    "A#4", "B4", "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5", "C6",
    "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6", "C7", "C#7", "D7",
    "D#7", "E7", "F7", "F#7", "G7", "G#7", "A7", "A#7", "B7", "C8", "C#8", "D8", "D#8", "E8", "F8",
    "F#8", "G8", "G#8", "A8", "A#8", "B8", "C9", "C#9", "D9", "D#9", "E9", "F9", "F#9", "G9",
];

// percussion_notes
static PERCUSSION_NOTES: &[(u8, &str)] = &[
    (35, "Acoustic Bass Drum"),
    (36, "Bass Drum 1"),
    (37, "Side Stick"),
    (38, "Acoustic Snare"),
    (39, "Hand Clap"),
    (40, "Electric Snare"),
    (41, "Low Floor Tom"),
    (42, "Closed Hi Hat"),
    (43, "High Floor Tom"),
    (44, "Pedal Hi-Hat"),
    (45, "Low Tom"),
    (46, "Open Hi-Hat"),
    (47, "Low-Mid Tom"),
    (48, "Hi Mid Tom"),
    (49, "Crash Cymbal 1"),
    (50, "High Tom"),
    (51, "Ride Cymbal 1"),
    (52, "Chinese Cymbal"),
    (53, "Ride Bell"),
    (54, "Tambourine"),
    (55, "Splash Cymbal"),
    (56, "Cowbell"),
    (57, "Crash Cymbal 2"),
    (58, "Vibraslap"),
    (59, "Ride Cymbal 2"),
    (60, "Hi Bongo"),
    (61, "Low Bongo"),
    (62, "Mute Hi Conga"),
    (63, "Open Hi Conga"),
    (64, "Low Conga"),
    (65, "High Timbale"),
    (66, "Low Timbale"),
    (67, "High Agogo"),
    (68, "Low Agogo"),
    (69, "Cabasa"),
    (70, "Maracas"),
    (71, "Short Whistle"),
    (72, "Long Whistle"),
    (73, "Short Guiro"),
    (74, "Long Guiro"),
    (75, "Claves"),
    (76, "Hi Wood Block"),
    (77, "Low Wood Block"),
    (78, "Mute Cuica"),
    (79, "Open Cuica"),
    (80, "Mute Triangle"),
    (81, "Open Triangle"),
];

// gm_instruments
static GM_INSTRUMENTS: [&str; 128] = [
    "Acoustic Grand Piano",
    "Bright Acoustic Piano",
    "Electric Grand Piano",
    "Honky-tonk Piano",
    "Electric Piano 1",
    "Electric Piano 2",
    "Harpsichord",
    "Clavi",
    "Celesta",
    "Glockenspiel",
    "Music Box",
    "Vibraphone",
    "Marimba",
    "Xylophone",
    "Tubular Bells",
    "Dulcimer",
    "Drawbar Organ",
    "Percussive Organ",
    "Rock Organ",
    "Church Organ",
    "Reed Organ",
    "Accordion",
    "Harmonica",
    "Tango Accordion",
    "Acoustic Guitar (nylon)",
    "Acoustic Guitar (steel)",
    "Electric Guitar (jazz)",
    "Electric Guitar (clean)",
    "Electric Guitar (muted)",
    "Overdriven Guitar",
    "Distortion Guitar",
    "Guitar harmonics",
    "Acoustic Bass",
    "Electric Bass (finger)",
    "Electric Bass (pick)",
    "Fretless Bass",
    "Slap Bass 1",
    "Slap Bass 2",
    "Synth Bass 1",
    "Synth Bass 2",
    "Violin",
    "Viola",
    "Cello",
    "Contrabass",
    "Tremolo Strings",
    "Pizzicato Strings",
    "Orchestral Harp",
    "Timpani",
    "String Ensemble 1",
    "String Ensemble 2",
    "SynthStrings 1",
    "SynthStrings 2",
    "Choir Aahs",
    "Voice Oohs",
    "Synth Voice",
    "Orchestra Hit",
    "Trumpet",
    "Trombone",
    "Tuba",
    "Muted Trumpet",
    "French Horn",
    "Brass Section",
    "SynthBrass 1",
    "SynthBrass 2",
    "Soprano Sax",
    "Alto Sax",
    "Tenor Sax",
    "Baritone Sax",
    "Oboe",
    "English Horn",
    "Bassoon",
    "Clarinet",
    "Piccolo",
    "Flute",
    "Recorder",
    "Pan Flute",
    "Blown Bottle",
    "Shakuhachi",
    "Whistle",
    "Ocarina",
    "Lead 1 (square)",
    "Lead 2 (sawtooth)",
    "Lead 3 (calliope)",
    "Lead 4 (chiff)",
    "Lead 5 (charang)",
    "Lead 6 (voice)",
    "Lead 7 (fifths)",
    "Lead 8 (bass + lead)",
    "Pad 1 (new age)",
    "Pad 2 (warm)",
    "Pad 3 (polysynth)",
    "Pad 4 (choir)",
    "Pad 5 (bowed)",
    "Pad 6 (metallic)",
    "Pad 7 (halo)",
    "Pad 8 (sweep)",
    "FX 1 (rain)",
    "FX 2 (soundtrack)",
    "FX 3 (crystal)",
    "FX 4 (atmosphere)",
    "FX 5 (brightness)",
    "FX 6 (goblins)",
    "FX 7 (echoes)",
    "FX 8 (sci-fi)",
    "Sitar",
    "Banjo",
    "Shamisen",
    "Koto",
    "Kalimba",
    "Bag pipe",
    "Fiddle",
    "Shanai",
    "Tinkle Bell",
    "Agogo",
    "Steel Drums",
    "Woodblock",
    "Taiko Drum",
    "Melodic Tom",
    "Synth Drum",
    "Reverse Cymbal",
    "Guitar Fret Noise",
    "Breath Noise",
    "Seashore",
    "Bird Tweet",
    "Telephone Ring",
    "Helicopter",
    "Applause",
    "Gunshot",
];

// drum_kit
static DRUM_KITS: &[(u8, &str)] = &[
    (1, "GM Standard Kit"),
    (9, "GS Room Kit"),
    (17, "GS Power Kit"),
    (25, "GS Power Kit"),
    (26, "GS TR-808 Kit"),
    (33, "GS Jazz Kit"),
    (41, "GS Brush Kit"),
    (49, "GS Orchestra Kit"),
    (57, "GS Sound FX Kit"),
    (128, "GS CM-64/CM-32 Kit"),
];

// control_functions
static CONTROL_FUNCTIONS: &[(u8, [&str; 3])] = &[
    (0x00, ["bank select MSB", "bank MSB", "bank-M"]),
    (0x01, ["modulation wheel/lever MSB", "modulation MSB", "mod-M"]),
    (0x02, ["breath controller MSB", "breath MSB", "breath-M"]),
    (0x04, ["foot controller MSB", "foot MSB", "foot-M"]),
    (0x05, ["portamento time MSB", "portamento MSB", "porta-M"]),
    (0x06, ["data entry MSB", "data entry MSB", "data-M"]),
    (0x07, ["channel volume MSB (formerly main volume)", "channel volume MSB", "ch vol-M"]),
    (0x08, ["balance MSB", "bal MSB", "bal-M"]),
    (0x0a, ["pan MSB", "pan MSB", "pan-M"]),
    (0x0b, ["expression controller MSB", "expression MSB", "expr-M"]),
    (0x0c, ["effect control 1 MSB", "effect 1 MSB", "eff-1-M"]),
    (0x0d, ["effect control 2 MSB", "effect 2 MSB", "eff-2-M"]),
    (0x10, ["general purpose controller 1 MSB", "GP ctrl 1 MSB", "GPC-1-M"]),
    (0x11, ["general purpose controller 2 MSB", "GP ctrl 2 MSB", "GPC-2-M"]),
    (0x12, ["general purpose controller 3 MSB", "GP ctrl 3 MSB", "GPC-3-M"]),
    (0x13, ["general purpose controller 4 MSB", "GP ctrl 4 MSB", "GPC-4-M"]),
    (0x20, ["bank select LSB", "bank LSB", "bank-L"]),
    (0x21, ["modulation wheel/lever LSB", "modulation LSB", "mod-L"]),
    (0x22, ["breath controller LSB", "breath LSB", "breath-L"]),
    (0x24, ["foot controller LSB", "foot LSB", "foot-L"]),
    (0x25, ["portamento time LSB", "portamento LSB", "porta-L"]),
    (0x26, ["data entry LSB", "data entry LSB", "data-L"]),
    (0x27, ["channel volume LSB (formerly main volume)", "channel volume LSB", "ch vol-L"]),
    (0x28, ["balance LSB", "bal LSB", "bal-L"]),
    (0x2a, ["pan LSB", "pan LSB", "pan-L"]),
    (0x2b, ["expression controller LSB", "expression LSB", "expr-L"]),
    (0x2c, ["effect control 1 LSB", "effect 1 LSB", "eff-1-L"]),
    (0x2d, ["effect control 2 LSB", "effect 2 LSB", "eff-2-L"]),
    (0x30, ["general purpose controller 1 LSB", "GP ctrl 1 LSB", "GPC-1-L"]),
    (0x31, ["general purpose controller 2 LSB", "GP ctrl 2 LSB", "GPC-2-L"]),
    (0x32, ["general purpose controller 3 LSB", "GP ctrl 3 LSB", "GPC-3-L"]),
    (0x33, ["general purpose controller 4 LSB", "GP ctrl 4 LSB", "GPC-4-L"]),
    (0x40, ["damper pedal (sustain)", "sustain", "sust"]),
    (0x41, ["portamento on/off", "porta on/off", "porta on/off"]),
    (0x42, ["sostenuto", "sostenuto", "sostenuto"]),
    (0x43, ["soft pedal", "soft pedal", "soft pedal"]),
    (0x44, ["legato footswitch", "legato switch", "legato"]),
    (0x45, ["hold 2", "hold 2", "hold 2"]),
    (0x46, ["sound controller 1 (default: sound variation)", "sound ctrl 1", "snd ctrl 1"]),
    (
        0x47,
        ["sound controller 2 (default: timbre / harmonic intensity)", "sound ctrl 2", "snd ctrl 2"],
    ),
    (0x48, ["sound controller 3 (default: release time)", "sound ctrl 3", "snd ctrl 3"]),
    (0x49, ["sound controller 4 (default: attack time)", "sound ctrl 4", "snd ctrl 4"]),
    (0x4a, ["sound controller 5 (default: brightness)", "sound ctrl 5", "snd ctrl 5"]),
    (0x4b, ["sound controller 6 (GM2 default: decay time)", "sound ctrl 6", "snd ctrl 6"]),
    (0x4c, ["sound controller 7 (GM2 default: vibrato rate)", "sound ctrl 7", "snd ctrl 7"]),
    (0x4d, ["sound controller 8 (GM2 default: vibrato depth)", "sound ctrl 8", "snd ctrl 8"]),
    (0x4e, ["sound controller 9 (GM2 default: vibrato delay)", "sound ctrl 9", "snd ctrl 9"]),
    (0x4f, ["sound controller 10", "sound ctrl 10", "snd ctrl 10"]),
    (0x50, ["general purpose controller 5", "GP controller 5", "GPC-5"]),
    (0x51, ["general purpose controller 6", "GP controller 6", "GPC-6"]),
    (0x52, ["general purpose controller 7", "GP controller 7", "GPC-7"]),
    (0x53, ["general purpose controller 8", "GP controller 8", "GPC-8"]),
    (0x54, ["portamento control", "portamento ctrl", "porta ctrl"]),
    (0x5b, ["effects 1 depth (formerly external effects depth)", "effects 1 depth", "eff 1 depth"]),
    (0x5c, ["effects 2 depth (formerly tremolo depth)", "effects 2 depth", "eff 2 depth"]),
    (0x5d, ["effects 3 depth (formerly chorus depth)", "effects 3 depth", "eff 3 depth"]),
    (0x5e, ["effects 4 depth (formerly celeste/detune depth)", "effects 4 depth", "eff 4 depth"]),
    (0x5f, ["effects 5 depth (formerly phaser depth)", "effects 5 depth", "eff 5 depth"]),
    (0x60, ["data increment", "data inc", "data++"]),
    (0x61, ["data decrement", "data dec", "data--"]),
    (0x62, ["Non-Registered Parameter Number LSB", "NRPN LSB", "NRPN-L"]),
    (0x63, ["Non-Registered Parameter Number MSB", "NRPN MSB", "NRPN-M"]),
    (0x64, ["Registered Parameter Number LSB", "RPN LSB", "RPN-L"]),
    (0x65, ["Registered Parameter Number MSB", "RPN MSB", "RPN-M"]),
    (0x78, ["all sound off", "all snd off", "snd off"]),
    (0x79, ["reset all controllers", "reset all ctrls", "reset ctrls"]),
    (0x7a, ["local control", "local ctrl", "local ctrl"]),
    (0x7b, ["all notes off", "notes off", "notes off"]),
    (0x7c, ["omni mode off", "omni off", "omni off"]),
    (0x7d, ["omni mode on", "omni on", "omni on"]),
    (0x7e, ["mono mode on", "mono on", "mono"]),
    (0x7f, ["poly mode on", "poly on", "poly"]),
];

// quarter_frame_type
static QUARTER_FRAME_TYPE: [&[&str; 2]; 8] = [
    &["frame count LS nibble", "frame LSN"],
    &["frame count MS nibble", "frame MSN"],
    &["seconds LS nibble", "sec LSN"],
    &["seconds MS nibble", "sec MSN"],
    &["minutes LS nibble", "min LSN"],
    &["minutes MS nibble", "min MSN"],
    &["hours LS nibble", "hrs LSN"],
    &["hours MS nibble and SMPTE type", "hrs MSN"],
];

// smpte_type
static SMPTE_TYPE: [&str; 4] = ["24 fps", "25 fps", "30 fps (drop-frame)", "30 fps (non-drop)"];

// sysex_manufacturer_ids
static SYSEX_IDS: &[(&[u8], &str)] = &[
    (&[0x01], "Sequential"),
    (&[0x02], "IDP"),
    (&[0x03], "Voyetra/Octave-Plateau"),
    (&[0x04], "Moog"),
    (&[0x05], "Passport Designs"),
    (&[0x06], "Lexicon"),
    (&[0x07], "Kurzweil"),
    (&[0x08], "Fender"),
    (&[0x09], "Gulbransen"),
    (&[0x0a], "AKG Acoustics"),
    (&[0x0b], "Voyce Music"),
    (&[0x0c], "Waveframe Corp"),
    (&[0x0d], "ADA Signal Processors"),
    (&[0x0e], "Garfield Electronics"),
    (&[0x0f], "Ensoniq"),
    (&[0x10], "Oberheim"),
    (&[0x11], "Apple Computer"),
    (&[0x12], "Grey Matter Response"),
    (&[0x13], "Digidesign"),
    (&[0x14], "Palm Tree Instruments"),
    (&[0x15], "JLCooper Electronics"),
    (&[0x16], "Lowrey"),
    (&[0x17], "Adams-Smith"),
    (&[0x18], "Emu Systems"),
    (&[0x19], "Harmony Systems"),
    (&[0x1a], "ART"),
    (&[0x1b], "Baldwin"),
    (&[0x1c], "Eventide"),
    (&[0x1d], "Inventronics"),
    (&[0x1f], "Clarity"),
    (&[0x00, 0x00, 0x01], "Time Warner Interactive"),
    (&[0x00, 0x00, 0x07], "Digital Music Corp."),
    (&[0x00, 0x00, 0x08], "IOTA Systems"),
    (&[0x00, 0x00, 0x09], "New England Digital"),
    (&[0x00, 0x00, 0x0a], "Artisyn"),
    (&[0x00, 0x00, 0x0b], "IVL Technologies"),
    (&[0x00, 0x00, 0x0c], "Southern Music Systems"),
    (&[0x00, 0x00, 0x0d], "Lake Butler Sound Company"),
    (&[0x00, 0x00, 0x0e], "Alesis"),
    (&[0x00, 0x00, 0x10], "DOD Electronics"),
    (&[0x00, 0x00, 0x11], "Studer-Editech"),
    (&[0x00, 0x00, 0x14], "Perfect Fretworks"),
    (&[0x00, 0x00, 0x15], "KAT"),
    (&[0x00, 0x00, 0x16], "Opcode"),
    (&[0x00, 0x00, 0x17], "Rane Corp."),
    (&[0x00, 0x00, 0x18], "Anadi Inc."),
    (&[0x00, 0x00, 0x19], "KMX"),
    (&[0x00, 0x00, 0x1a], "Allen & Heath Brenell"),
    (&[0x00, 0x00, 0x1b], "Peavy Electronics"),
    (&[0x00, 0x00, 0x1c], "360 Systems"),
    (&[0x00, 0x00, 0x1d], "Spectrum Design and Development"),
    (&[0x00, 0x00, 0x1e], "Marquis Music"),
    (&[0x00, 0x00, 0x1f], "Zeta Systems"),
    (&[0x00, 0x00, 0x20], "Axxes"),
    (&[0x00, 0x00, 0x21], "Orban"),
    (&[0x00, 0x00, 0x24], "KTI"),
    (&[0x00, 0x00, 0x25], "Breakaway Technologies"),
    (&[0x00, 0x00, 0x26], "CAE"),
    (&[0x00, 0x00, 0x29], "Rocktron Corp."),
    (&[0x00, 0x00, 0x2a], "PianoDisc"),
    (&[0x00, 0x00, 0x2b], "Cannon Research Group"),
    (&[0x00, 0x00, 0x2d], "Rogers Instrument Corp."),
    (&[0x00, 0x00, 0x2e], "Blue Sky Logic"),
    (&[0x00, 0x00, 0x2f], "Encore Electronics"),
    (&[0x00, 0x00, 0x30], "Uptown"),
    (&[0x00, 0x00, 0x31], "Voce"),
    (&[0x00, 0x00, 0x32], "CTI Audio, Inc. (Music. Intel Dev.)"),
    (&[0x00, 0x00, 0x33], "S&S Research"),
    (&[0x00, 0x00, 0x34], "Broderbund Software, Inc."),
    (&[0x00, 0x00, 0x35], "Allen Organ Co."),
    (&[0x00, 0x00, 0x37], "Music Quest"),
    (&[0x00, 0x00, 0x38], "APHEX"),
    (&[0x00, 0x00, 0x39], "Gallien Krueger"),
    (&[0x00, 0x00, 0x3a], "IBM"),
    (&[0x00, 0x00, 0x3c], "Hotz Instruments Technologies"),
    (&[0x00, 0x00, 0x3d], "ETA Lighting"),
    (&[0x00, 0x00, 0x3e], "NSI Corporation"),
    (&[0x00, 0x00, 0x3f], "Ad Lib, Inc."),
    (&[0x00, 0x00, 0x40], "Richmond Sound Design"),
    (&[0x00, 0x00, 0x41], "Microsoft"),
    (&[0x00, 0x00, 0x42], "The Software Toolworks"),
    (&[0x00, 0x00, 0x43], "Niche/RJMG"),
    (&[0x00, 0x00, 0x44], "Intone"),
    (&[0x00, 0x00, 0x47], "GT Electronics / Groove Tubes"),
    (&[0x00, 0x00, 0x49], "Timeline Vista"),
    (&[0x00, 0x00, 0x4a], "Mesa Boogie"),
    (&[0x00, 0x00, 0x4c], "Sequoia Development"),
    (&[0x00, 0x00, 0x4d], "Studio Electronics"),
    (&[0x00, 0x00, 0x4e], "Euphonix"),
    (&[0x00, 0x00, 0x4f], "InterMIDI, Inc."),
    (&[0x00, 0x00, 0x50], "MIDI Solutions"),
    (&[0x00, 0x00, 0x51], "3DO Company"),
    (&[0x00, 0x00, 0x52], "Lightwave Research"),
    (&[0x00, 0x00, 0x53], "Micro-W"),
    (&[0x00, 0x00, 0x54], "Spectral Synthesis"),
    (&[0x00, 0x00, 0x55], "Lone Wolf"),
    (&[0x00, 0x00, 0x56], "Studio Technologies"),
    (&[0x00, 0x00, 0x57], "Peterson EMP"),
    (&[0x00, 0x00, 0x58], "Atari"),
    (&[0x00, 0x00, 0x59], "Marion Systems"),
    (&[0x00, 0x00, 0x5a], "Design Event"),
    (&[0x00, 0x00, 0x5b], "Winjammer Software"),
    (&[0x00, 0x00, 0x5c], "AT&T Bell Labs"),
    (&[0x00, 0x00, 0x5e], "Symetrix"),
    (&[0x00, 0x00, 0x5f], "MIDI the World"),
    (&[0x00, 0x00, 0x60], "Desper Products"),
    (&[0x00, 0x00, 0x61], "Micros'N MIDI"),
    (&[0x00, 0x00, 0x62], "Accordians Intl"),
    (&[0x00, 0x00, 0x63], "EuPhonics"),
    (&[0x00, 0x00, 0x64], "Musonix"),
    (&[0x00, 0x00, 0x65], "Turtle Beach Systems"),
    (&[0x00, 0x00, 0x66], "Mackie Designs"),
    (&[0x00, 0x00, 0x67], "Compuserve"),
    (&[0x00, 0x00, 0x68], "BES Technologies"),
    (&[0x00, 0x00, 0x69], "QRS Music Rolls"),
    (&[0x00, 0x00, 0x6a], "P G Music"),
    (&[0x00, 0x00, 0x6b], "Sierra Semiconductor"),
    (&[0x00, 0x00, 0x6c], "EpiGraf Audio Visual"),
    (&[0x00, 0x00, 0x6d], "Electronics Deiversified"),
    (&[0x00, 0x00, 0x6e], "Tune 1000"),
    (&[0x00, 0x00, 0x6f], "Advanced Micro Devices"),
    (&[0x00, 0x00, 0x70], "Mediamation"),
    (&[0x00, 0x00, 0x71], "Sabine Music"),
    (&[0x00, 0x00, 0x72], "Woog Labs"),
    (&[0x00, 0x00, 0x73], "Micropolis"),
    (&[0x00, 0x00, 0x74], "Ta Horng Musical Inst."),
    (&[0x00, 0x00, 0x75], "eTek (formerly Forte)"),
    (&[0x00, 0x00, 0x76], "Electrovoice"),
    (&[0x00, 0x00, 0x77], "Midisoft"),
    (&[0x00, 0x00, 0x78], "Q-Sound Labs"),
    (&[0x00, 0x00, 0x79], "Westrex"),
    (&[0x00, 0x00, 0x7a], "NVidia"),
    (&[0x00, 0x00, 0x7b], "ESS Technology"),
    (&[0x00, 0x00, 0x7c], "MediaTrix Peripherals"),
    (&[0x00, 0x00, 0x7d], "Brooktree"),
    (&[0x00, 0x00, 0x7e], "Otari"),
    (&[0x00, 0x00, 0x7f], "Key Electronics"),
    (&[0x00, 0x01, 0x01], "Crystalake Multimedia"),
    (&[0x00, 0x01, 0x02], "Crystal Semiconductor"),
    (&[0x00, 0x01, 0x03], "Rockwell Semiconductor"),
    (&[0x20], "Passac"),
    (&[0x21], "SIEL"),
    (&[0x22], "Synthaxe"),
    (&[0x24], "Hohner"),
    (&[0x25], "Twister"),
    (&[0x26], "Solton"),
    (&[0x27], "Jellinghaus MS"),
    (&[0x28], "Southworth Music Systems"),
    (&[0x29], "PPG"),
    (&[0x2a], "JEN"),
    (&[0x2b], "SSL Limited"),
    (&[0x2c], "Audio Veritrieb"),
    (&[0x2f], "Elka"),
    (&[0x30], "Dynacord"),
    (&[0x31], "Viscount"),
    (&[0x33], "Clavia Digital Instruments"),
    (&[0x34], "Audio Architecture"),
    (&[0x35], "GeneralMusic Corp."),
    (&[0x39], "Soundcraft Electronics"),
    (&[0x3b], "Wersi"),
    (&[0x3c], "Avab Elektronik Ab"),
    (&[0x3d], "Digigram"),
    (&[0x3e], "Waldorf Electronics"),
    (&[0x3f], "Quasimidi"),
    (&[0x00, 0x20, 0x00], "Dream"),
    (&[0x00, 0x20, 0x01], "Strand Lighting"),
    (&[0x00, 0x20, 0x02], "Amek Systems"),
    (&[0x00, 0x20, 0x04], "Böhm Electronic"),
    (&[0x00, 0x20, 0x06], "Trident Audio"),
    (&[0x00, 0x20, 0x07], "Real World Studio"),
    (&[0x00, 0x20, 0x09], "Yes Technology"),
    (&[0x00, 0x20, 0x0a], "Audiomatica"),
    (&[0x00, 0x20, 0x0b], "Bontempi/Farfisa"),
    (&[0x00, 0x20, 0x0c], "F.B.T. Elettronica"),
    (&[0x00, 0x20, 0x0d], "MidiTemp"),
    (&[0x00, 0x20, 0x0e], "LA Audio (Larking Audio)"),
    (&[0x00, 0x20, 0x0f], "Zero 88 Lighting Limited"),
    (&[0x00, 0x20, 0x10], "Micon Audio Electronics GmbH"),
    (&[0x00, 0x20, 0x11], "Forefront Technology"),
    (&[0x00, 0x20, 0x13], "Kenton Electronics"),
    (&[0x00, 0x20, 0x15], "ADB"),
    (&[0x00, 0x20, 0x16], "Marshall Products"),
    (&[0x00, 0x20, 0x17], "DDA"),
    (&[0x00, 0x20, 0x18], "BSS"),
    (&[0x00, 0x20, 0x19], "MA Lighting Technology"),
    (&[0x00, 0x20, 0x1a], "Fatar"),
    (&[0x00, 0x20, 0x1b], "QSC Audio"),
    (&[0x00, 0x20, 0x1c], "Artisan Classic Organ"),
    (&[0x00, 0x20, 0x1d], "Orla Spa"),
    (&[0x00, 0x20, 0x1e], "Pinnacle Audio"),
    (&[0x00, 0x20, 0x1f], "TC Electronics"),
    (&[0x00, 0x20, 0x20], "Doepfer Musikelektronik"),
    (&[0x00, 0x20, 0x21], "Creative Technology Pte"),
    (&[0x00, 0x20, 0x22], "Minami/Seiyddo"),
    (&[0x00, 0x20, 0x23], "Goldstar"),
    (&[0x00, 0x20, 0x24], "Midisoft s.a.s di M. Cima"),
    (&[0x00, 0x20, 0x25], "Samick"),
    (&[0x00, 0x20, 0x26], "Penny and Giles"),
    (&[0x00, 0x20, 0x27], "Acorn Computer"),
    (&[0x00, 0x20, 0x28], "LSC Electronics"),
    (&[0x00, 0x20, 0x29], "Novation EMS"),
    (&[0x00, 0x20, 0x2a], "Samkyung Mechatronics"),
    (&[0x00, 0x20, 0x2b], "Medeli Electronics"),
    (&[0x00, 0x20, 0x2c], "Charlie Lab"),
    (&[0x00, 0x20, 0x2d], "Blue Chip Music Tech"),
    (&[0x00, 0x20, 0x2e], "BEE OH Corp"),
    (&[0x40], "Kawai"),
    (&[0x41], "Roland"),
    (&[0x42], "Korg"),
    (&[0x43], "Yamaha"),
    (&[0x44], "Casio"),
    (&[0x46], "Kamiya Studio"),
    (&[0x47], "Akai"),
    (&[0x48], "Japan Victor"),
    (&[0x49], "Mesosha"),
    (&[0x4a], "Hoshino Gakki"),
    (&[0x4b], "Fujitsu Elect"),
    (&[0x4c], "Sony"),
    (&[0x4d], "Nisshin Onpa"),
    (&[0x4e], "TEAC"),
    (&[0x50], "Matsushita Electric"),
    (&[0x51], "Fostex"),
    (&[0x52], "Zoom"),
    (&[0x53], "Midori Electronics"),
    (&[0x54], "Matsushita Communication Industrial"),
    (&[0x55], "Suzuki Musical Inst. Mfg."),
    (&[0x7d], "Non-Commercial"),
    (&[0x7e], "Universal Non-Realtime"),
    (&[0x7f], "Universal Realtime"),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    HandleChannelMsg,
    HandleSysexMsg,
    HandleSyscommonMsg,
    HandleSysrealtimeMsg,
    BufferGarbageMsg,
}

/// The MIDI decoder.
#[derive(Default)]
pub struct Midi {
    state: Option<State>,
    status_byte: i64,
    explicit_status_byte: bool,
    cmd: Vec<i64>,
    ss: u64,
    es: u64,
    ss_block: u64,
    es_block: u64,
}

impl Midi {
    fn state(&self) -> State {
        self.state.unwrap_or(State::Idle)
    }

    fn putx(&self, ctx: &mut dyn Context, class: usize, texts: Vec<String>) {
        ctx.put_ann(self.ss_block, self.es_block, class, texts);
    }

    fn putx_texts(&self, ctx: &mut dyn Context, class: usize, texts: &[&str]) {
        self.putx(ctx, class, texts.iter().map(|t| (*t).to_owned()).collect());
    }

    fn get_note_name(channel: i64, note: i64) -> String {
        if channel != 10 {
            note_table(CHROMATIC_NOTES, note)
        } else {
            format!("assuming {}", percussion_name(note))
        }
    }

    fn check_for_garbage_flush(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        if is_flushed {
            if self.explicit_status_byte {
                self.cmd.insert(0, self.status_byte);
            }
            self.handle_garbage_msg(ctx, None)?;
        }
        Ok(())
    }

    fn soft_clear_status_byte(&mut self) {
        self.explicit_status_byte = false;
    }

    fn hard_clear_status_byte(&mut self) {
        self.status_byte = 0;
        self.explicit_status_byte = false;
    }

    fn set_status_byte(&mut self, newbyte: i64) {
        self.status_byte = newbyte;
        self.explicit_status_byte = true;
    }

    fn handle_channel_msg_0x80(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Note off: 8n kk vv
        if self.cmd.len() < 2 {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let note = self.cmd[0];
        let velocity = self.cmd[1];
        let note_name = Self::get_note_name(chan, note);
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!(
                    "Channel {chan}: {} (note = {note} '{}', velocity = {velocity})",
                    s[0], note_name
                ),
                format!("ch {chan}: {} {note}, velocity = {velocity}", s[1]),
                format!("{chan}: {} {note}, vel {velocity}", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_0x90(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Note on: 9n kk vv; velocity 0 actually means "note off".
        if self.cmd.len() < 2 {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let note = self.cmd[0];
        let velocity = self.cmd[1];
        let s = if velocity == 0 { status_names(0x80) } else { status_names(msg) };
        let note_name = Self::get_note_name(chan, note);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!(
                    "Channel {chan}: {} (note = {note} '{}', velocity = {velocity})",
                    s[0], note_name
                ),
                format!("ch {chan}: {} {note}, velocity = {velocity}", s[1]),
                format!("{chan}: {} {note}, vel {velocity}", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_0xa0(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Polyphonic key pressure / aftertouch: An kk vv
        if self.cmd.len() < 2 {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let note = self.cmd[0];
        let pressure = self.cmd[1];
        let note_name = Self::get_note_name(chan, note);
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} of {pressure} for note = {note} '{}'", s[0], note_name),
                format!("ch {chan}: {} {pressure} for note {note}", s[1]),
                format!("{chan}: {} {pressure}, N {note}", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_controller_0x44(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Legato footswitch: Bn 44 vv
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let vv = self.cmd[1];
        let t = if vv <= 0x3f { ("normal", "no") } else { ("legato", "yes") };
        let s = status_names(msg);
        let cf = control_function(0x44);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} '{}' = {}", s[0], cf[0], t.0),
                format!("ch {chan}: {} '{}' = {}", s[1], cf[1], t.0),
                format!("{chan}: {} '{}' = {}", s[2], cf[2], t.1),
            ],
        );
        Ok(())
    }

    fn handle_controller_0x54(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Portamento control (PTC): Bn 54 kk
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let kk = self.cmd[1];
        let kk_name = Self::get_note_name(chan, kk);
        let s = status_names(msg);
        let cf = control_function(0x54);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} '{}' (source note = {kk} / {kk_name})", s[0], cf[0]),
                format!("ch {chan}: {} '{}' (source note = {kk})", s[1], cf[1]),
                format!("{chan}: {} '{}' (src N {kk})", s[2], cf[2]),
            ],
        );
        Ok(())
    }

    fn handle_controller_generic(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let (fn_, param) = (self.cmd[0], self.cmd[1]);
        let (n0, n1, n2) = match control_function_opt(fn_) {
            Some(cf) => (cf[0].to_owned(), cf[1].to_owned(), cf[2].to_owned()),
            None => (
                format!("undefined 0x{fn_:02x}"),
                format!("undef 0x{fn_:02x}"),
                format!("0x{fn_:02x}"),
            ),
        };
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} '{n0}' (param = 0x{param:02x})", s[0]),
                format!("ch {chan}: {} '{n1}' (param = 0x{param:02x})", s[1]),
                format!("{chan}: {} '{n2}' is 0x{param:02x}", s[2]),
            ],
        );
        Ok(())
    }

    fn handle_channel_mode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        // Channel Mode: Bn mm vv
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let (mm, vv) = (self.cmd[0], self.cmd[1]);
        let mode_fn = control_function_opt(mm).unwrap_or(["undefined", "undef", "undef"]);
        // Decode the value based on the mode number.
        let (v0, v1) = if mm == 122 {
            if vv == 0 {
                ("off".to_owned(), "off".to_owned())
            } else if vv == 127 {
                ("on".to_owned(), "on".to_owned())
            } else {
                (format!("(non-standard param value of 0x{vv:02x})"), format!("0x{vv:02x}"))
            }
        } else if mm == 126 {
            if vv != 0 {
                (format!("({vv} channels)"), format!("({vv} ch)"))
            } else {
                ("(channels 'basic' through 16)".to_owned(), "(ch 'basic' thru 16)".to_owned())
            }
        } else if vv != 0 {
            (format!("(non-standard param value of 0x{vv:02x})"), format!("0x{vv:02x}"))
        } else {
            (String::new(), String::new())
        };
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} '{}' {v0}", s[0], mode_fn[0]),
                format!("ch {chan}: {} '{}' {v1}", s[1], mode_fn[1]),
                format!("{chan}: {} '{}' {v1}", s[2], mode_fn[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_0xb0(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Control change (or channel mode messages): Bn cc vv
        if self.cmd.len() < 2 {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        if (0x78..=0x7f).contains(&self.cmd[0]) {
            return self.handle_channel_mode(ctx);
        }
        match self.cmd[0] {
            0x44 => self.handle_controller_0x44(ctx)?,
            0x54 => self.handle_controller_0x54(ctx)?,
            _ => self.handle_controller_generic(ctx)?,
        }
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_0xc0(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Program change: Cn pp
        if self.cmd.is_empty() {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let pp = self.cmd[0] + 1;
        let (change_type, name) = if chan != 10 {
            ("instrument", instrument_name(pp))
        } else {
            ("drum kit", drum_kit_name(pp))
        };
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} to {change_type} {pp} (assuming {name})", s[0]),
                format!("ch {chan}: {} to {change_type} {pp}", s[1]),
                format!("{chan}: {} {pp}", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_0xd0(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Channel pressure / aftertouch: Dn vv
        if self.cmd.is_empty() {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let vv = self.cmd[0];
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} {vv}", s[0]),
                format!("ch {chan}: {} {vv}", s[1]),
                format!("{chan}: {} {vv}", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_0xe0(&mut self, ctx: &mut dyn Context, is_flushed: bool) -> Result<()> {
        // Pitch bend change: En ll mm
        if self.cmd.len() < 2 {
            self.check_for_garbage_flush(ctx, is_flushed)?;
            return Ok(());
        }
        self.es_block = self.es;
        let msg = self.status_byte & 0xf0;
        let chan = (self.status_byte & 0x0f) + 1;
        let (ll, mm) = (self.cmd[0], self.cmd[1]);
        let decimal = (mm << 7) + ll;
        let s = status_names(msg);
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("Channel {chan}: {} 0x{ll:02x} 0x{mm:02x} ({decimal})", s[0]),
                format!("ch {chan}: {} 0x{ll:02x} 0x{mm:02x} ({decimal})", s[1]),
                format!("{chan}: {} ({decimal})", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg_generic(
        &mut self,
        ctx: &mut dyn Context,
        _is_flushed: bool,
    ) -> Result<()> {
        let msg_type = self.status_byte & 0xf0;
        self.es_block = self.es;
        self.putx(ctx, ANN_ERROR, vec![format!("Unknown channel message type: 0x{msg_type:02x}")]);
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.soft_clear_status_byte();
        Ok(())
    }

    fn handle_channel_msg(&mut self, ctx: &mut dyn Context, newbyte: Option<i64>) -> Result<()> {
        if let Some(b) = newbyte {
            if b >= 0x80 {
                self.set_status_byte(b);
            } else {
                self.cmd.push(b);
            }
        }
        let msg_type = self.status_byte & 0xf0;
        let is_flushed = newbyte.is_none();
        match msg_type {
            0x80 => self.handle_channel_msg_0x80(ctx, is_flushed),
            0x90 => self.handle_channel_msg_0x90(ctx, is_flushed),
            0xa0 => self.handle_channel_msg_0xa0(ctx, is_flushed),
            0xb0 => self.handle_channel_msg_0xb0(ctx, is_flushed),
            0xc0 => self.handle_channel_msg_0xc0(ctx, is_flushed),
            0xd0 => self.handle_channel_msg_0xd0(ctx, is_flushed),
            0xe0 => self.handle_channel_msg_0xe0(ctx, is_flushed),
            _ => self.handle_channel_msg_generic(ctx, is_flushed),
        }
    }

    fn handle_sysex_msg(&mut self, ctx: &mut dyn Context, newbyte: Option<i64>) -> Result<()> {
        // SysEx message: 1 status byte, 1-3 manuf. bytes, x data bytes, EOX.
        self.hard_clear_status_byte();
        if let Some(b) = newbyte {
            if b != 0xf7 {
                self.cmd.push(b);
                return Ok(());
            }
        }
        self.es_block = self.es;
        // Note: Unlike other methods, this code pops bytes out of self.cmd.
        if self.cmd.is_empty() {
            return Err(Error::arg("pop from empty list"));
        }
        let msg = self.cmd.remove(0);
        let s = status_names(msg);
        if self.cmd.is_empty() {
            self.putx(
                ctx,
                ANN_ERROR,
                vec![
                    format!("{}: truncated manufacturer code (<1 bytes)", s[0]),
                    format!("{}: truncated manufacturer (<1 bytes)", s[1]),
                    format!("{}: trunc. manu.", s[2]),
                ],
            );
            self.cmd.clear();
            self.state = Some(State::Idle);
            return Ok(());
        }
        // Extract the manufacturer name (or SysEx realtime or non-realtime).
        let m1 = self.cmd.remove(0);
        let manu: Vec<i64> = if m1 == 0 {
            // If byte == 0, then 2 more manufacturer bytes follow.
            if self.cmd.len() < 2 {
                self.putx(
                    ctx,
                    ANN_ERROR,
                    vec![
                        format!("{}: truncated manufacturer code (<3 bytes)", s[0]),
                        format!("{}: truncated manufacturer (<3 bytes)", s[1]),
                        format!("{}: trunc. manu.", s[2]),
                    ],
                );
                self.cmd.clear();
                self.state = Some(State::Idle);
                return Ok(());
            }
            let b2 = self.cmd.remove(0);
            let b3 = self.cmd.remove(0);
            vec![m1, b2, b3]
        } else {
            vec![m1]
        };
        let (manu_name, manu_short) = sysex_manufacturer_name(&manu);
        // Extract the payload, display in 1 of 2 formats.
        let mut payload0 = String::new();
        let mut payload1 = String::new();
        while !self.cmd.is_empty() {
            let byte = self.cmd.remove(0);
            payload0.push_str(&format!("0x{byte:02x} "));
            payload1.push_str(&format!("{byte:02x} "));
        }
        if payload0.is_empty() {
            payload0 = "<empty>".to_owned();
            payload1 = "<>".to_owned();
        }
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!("{}: for '{manu_name}' with payload {payload0}", s[0]),
                format!("{}: '{manu_short}', payload {payload1}", s[1]),
                format!("{}: '{manu_short}', payload {payload1}", s[2]),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        Ok(())
    }

    fn handle_syscommon_quarter_frame(
        &mut self,
        ctx: &mut dyn Context,
        newbyte: Option<i64>,
    ) -> Result<()> {
        // MIDI time code quarter frame: F1 nd
        if self.cmd.len() < 2 {
            if newbyte.is_none() {
                self.handle_garbage_msg(ctx, None)?;
            }
            return Ok(());
        }
        let msg = self.cmd[0];
        let nn = (self.cmd[1] & 0x70) >> 4;
        let dd = self.cmd[1] & 0x0f;
        let group = ["System Common", "SysCom", "SC"];
        let s = status_names(msg);
        let qf = QUARTER_FRAME_TYPE[nn as usize];
        self.es_block = self.es;
        if nn != 7 {
            // If message type does not contain SMPTE type.
            self.putx(
                ctx,
                ANN_TEXT,
                vec![
                    format!("{}: {} of {}, value 0x{dd:01x}", group[0], s[0], qf[0]),
                    format!("{}: {} of {}, value 0x{dd:01x}", group[1], s[1], qf[1]),
                    format!("{}: {} of {}, value 0x{dd:01x}", group[2], s[2], qf[1]),
                ],
            );
            self.cmd.clear();
            self.state = Some(State::Idle);
            return Ok(());
        }
        let tt = (dd & 0x6) >> 1;
        self.putx(
            ctx,
            ANN_TEXT,
            vec![
                format!(
                    "{}: {} of {}, value 0x{dd:01x} for {}",
                    group[0], s[0], qf[0], SMPTE_TYPE[tt as usize]
                ),
                format!(
                    "{}: {} of {}, value 0x{dd:01x} for {}",
                    group[1], s[1], qf[1], SMPTE_TYPE[tt as usize]
                ),
                format!(
                    "{}: {} of {}, value 0x{dd:01x} for {}",
                    group[2], s[2], qf[1], SMPTE_TYPE[tt as usize]
                ),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        Ok(())
    }

    fn handle_syscommon_msg(&mut self, ctx: &mut dyn Context, newbyte: Option<i64>) -> Result<()> {
        // System common messages.
        self.hard_clear_status_byte();
        if let Some(b) = newbyte {
            self.cmd.push(b);
        }
        if self.cmd.is_empty() {
            return Err(Error::arg("list index out of range"));
        }
        let msg = self.cmd[0];
        let group = ["System Common", "SysCom", "SC"];
        let s = status_names(msg);
        if msg == 0xf1 {
            // MIDI time code quarter frame.
            return self.handle_syscommon_quarter_frame(ctx, newbyte);
        } else if msg == 0xf2 {
            // Song position pointer: F2 ll mm
            if self.cmd.len() < 3 {
                if newbyte.is_none() {
                    self.handle_garbage_msg(ctx, None)?;
                }
                return Ok(());
            }
            let (ll, mm) = (self.cmd[1], self.cmd[2]);
            let decimal = (mm << 7) + ll;
            self.es_block = self.es;
            self.putx(
                ctx,
                ANN_TEXT,
                vec![
                    format!("{}: {} 0x{ll:02x} 0x{mm:02x} ({decimal})", group[0], s[0]),
                    format!("{}: {} 0x{ll:02x} 0x{mm:02x} ({decimal})", group[1], s[1]),
                    format!("{}: {} ({decimal})", group[2], s[2]),
                ],
            );
        } else if msg == 0xf3 {
            // Song select: F3 ss
            if self.cmd.len() < 2 {
                if newbyte.is_none() {
                    self.handle_garbage_msg(ctx, None)?;
                }
                return Ok(());
            }
            let ss = self.cmd[1];
            self.es_block = self.es;
            self.putx(
                ctx,
                ANN_TEXT,
                vec![
                    format!("{}: {} number {ss}", group[0], s[0]),
                    format!("{}: {} number {ss}", group[1], s[1]),
                    format!("{}: {} # {ss}", group[2], s[2]),
                ],
            );
        } else if msg == 0xf4 || msg == 0xf5 || msg == 0xf6 {
            // Undefined 0xf4, Undefined 0xf5, and Tune Request.
            self.es_block = self.es;
            self.putx(
                ctx,
                ANN_TEXT,
                vec![
                    format!("{}: {}", group[0], s[0]),
                    format!("{}: {}", group[1], s[1]),
                    format!("{}: {}", group[2], s[2]),
                ],
            );
        }
        self.cmd.clear();
        self.state = Some(State::Idle);
        Ok(())
    }

    fn handle_sysrealtime_msg(&mut self, ctx: &mut dyn Context, newbyte: i64) {
        // System realtime messages may interrupt other messages.
        let old_ss_block = self.ss_block;
        let old_es_block = self.es_block;
        self.ss_block = self.ss;
        self.es_block = self.es;
        let group = ["System Realtime", "SysReal", "SR"];
        let s = status_names(newbyte);
        self.putx_texts(
            ctx,
            ANN_SYSREAL,
            &[
                &format!("{}: {}", group[0], s[0]),
                &format!("{}: {}", group[1], s[1]),
                &format!("{}: {}", group[2], s[2]),
            ],
        );
        self.ss_block = old_ss_block;
        self.es_block = old_es_block;
        // Deliberately not resetting self.cmd or self.state.
    }

    fn handle_garbage_msg(&mut self, ctx: &mut dyn Context, newbyte: Option<i64>) -> Result<()> {
        // Handle messages that are either not handled or are corrupt.
        self.es_block = self.es;
        if let Some(b) = newbyte {
            self.cmd.push(b);
            return Ok(());
        }
        let mut payload = "<empty>".to_owned();
        let max_bytes = 16; // Put a limit on the length on the hex dump.
        for (index, byte) in self.cmd.iter().enumerate() {
            if index == max_bytes {
                payload.push_str(" ...");
                break;
            }
            if index == 0 {
                payload = format!("0x{byte:02x}");
            } else {
                payload.push_str(&format!(" 0x{byte:02x}"));
            }
        }
        self.putx(
            ctx,
            ANN_ERROR,
            vec![
                format!("UNHANDLED DATA: {payload}"),
                "UNHANDLED".to_owned(),
                "???".to_owned(),
                "?".to_owned(),
            ],
        );
        self.cmd.clear();
        self.state = Some(State::Idle);
        self.hard_clear_status_byte();
        Ok(())
    }

    fn handle_state(
        &mut self,
        ctx: &mut dyn Context,
        state: State,
        newbyte: Option<i64>,
    ) -> Result<()> {
        match state {
            State::HandleChannelMsg => self.handle_channel_msg(ctx, newbyte),
            State::HandleSysexMsg => self.handle_sysex_msg(ctx, newbyte),
            State::HandleSyscommonMsg => self.handle_syscommon_msg(ctx, newbyte),
            State::HandleSysrealtimeMsg => {
                self.handle_sysrealtime_msg(ctx, newbyte.unwrap_or(0));
                Ok(())
            }
            State::BufferGarbageMsg => self.handle_garbage_msg(ctx, newbyte),
            State::Idle => Ok(()),
        }
    }

    fn get_next_state(&self, newbyte: i64) -> State {
        if (0x80..=0xef).contains(&newbyte) {
            return State::HandleChannelMsg;
        }
        if newbyte == 0xf0 {
            return State::HandleSysexMsg;
        }
        if (0xf1..=0xf6).contains(&newbyte) {
            return State::HandleSyscommonMsg;
        }
        if (0xf8..=0xff).contains(&newbyte) {
            return State::HandleSysrealtimeMsg;
        }
        // Passing 0xf7 is an error; messages don't start with 0xf7.
        if newbyte == 0xf7 {
            return State::BufferGarbageMsg;
        }
        // Next, base the state off of self.status_byte.
        if self.status_byte < 0x80 {
            return State::BufferGarbageMsg;
        }
        self.get_next_state(self.status_byte)
    }

    fn decode_byte(&mut self, ctx: &mut dyn Context, ss: u64, es: u64, pdata: i64) -> Result<()> {
        let state;

        // State machine.
        if pdata >= 0x80 && pdata != 0xf7 {
            state = self.get_next_state(pdata);
            let current = self.state();
            if state != State::HandleSysrealtimeMsg && current != State::Idle {
                // Flush the previous data since a new message is starting.
                self.handle_state(ctx, current, None)?;
            }
            // Cache ss and es -after- flushing previous data.
            self.ss = ss;
            self.es = es;
            // This is a status byte, remember the start sample.
            if state != State::HandleSysrealtimeMsg {
                self.ss_block = ss;
            }
        } else if self.state() == State::Idle || self.state() == State::BufferGarbageMsg {
            // Deal with "running status" or that we're buffering garbage.
            self.ss = ss;
            self.es = es;
            if self.state() == State::Idle {
                self.ss_block = ss;
            }
            state = self.get_next_state(pdata);
        } else {
            self.ss = ss;
            self.es = es;
            state = self.state();
        }

        // Yes, this is intentionally _not_ an 'elif' here.
        if state != State::HandleSysrealtimeMsg {
            self.state = Some(state);
        }
        if state == State::BufferGarbageMsg {
            self.status_byte = 0;
        }
        self.handle_state(ctx, state, Some(pdata))
    }
}

/// The status byte's three names, longest first.
fn status_names(byte: i64) -> [&'static str; 3] {
    STATUS_BYTES
        .iter()
        .find(|(k, _)| i64::from(*k) == byte)
        .map(|(_, v)| *v)
        .unwrap_or(["", "", ""])
}

fn control_function(byte: i64) -> [&'static str; 3] {
    control_function_opt(byte).unwrap_or(["undefined", "undef", "undef"])
}

fn control_function_opt(byte: i64) -> Option<[&'static str; 3]> {
    CONTROL_FUNCTIONS.iter().find(|(k, _)| i64::from(*k) == byte).map(|(_, v)| *v)
}

fn note_table(table: [&'static str; 128], note: i64) -> String {
    table.get(note as usize).copied().unwrap_or("undefined").to_owned()
}

fn percussion_name(note: i64) -> &'static str {
    PERCUSSION_NOTES
        .iter()
        .find(|(k, _)| i64::from(*k) == note)
        .map(|(_, v)| *v)
        .unwrap_or("undefined")
}

fn instrument_name(pp: i64) -> &'static str {
    if (1..=128).contains(&pp) { GM_INSTRUMENTS[(pp - 1) as usize] } else { "undefined" }
}

fn drum_kit_name(pp: i64) -> &'static str {
    DRUM_KITS.iter().find(|(k, _)| i64::from(*k) == pp).map(|(_, v)| *v).unwrap_or("undefined")
}

fn sysex_manufacturer_name(manu: &[i64]) -> (String, String) {
    let key: Vec<u8> = manu.iter().map(|&b| b as u8).collect();
    if let Some((_, name)) = SYSEX_IDS.iter().find(|(k, _)| *k == key.as_slice()) {
        return ((*name).to_owned(), (*name).to_owned());
    }
    if manu.len() == 3 {
        (
            format!("undefined (0x{:02x} 0x{:02x} 0x{:02x})", manu[0], manu[1], manu[2]),
            "undefined".to_owned(),
        )
    } else {
        (format!("undefined (0x{:02x})", manu[0]), "undefined".to_owned())
    }
}

/// Unpacks `[ptype, rxtx, pdata]` and the value of a DATA item.
fn data_byte(item: &Item) -> Result<Option<i64>> {
    let Item::List(items) = item else {
        return Err(Error::arg("cannot unpack non-sequence UART item"));
    };
    if items.len() != 3 {
        return Err(Error::arg("too many values to unpack (UART item)"));
    }
    let Item::Str(ptype) = &items[0] else {
        return Err(Error::arg("UART packet type is not a string"));
    };
    // For now, ignore all UART packets except the actual data packets.
    if ptype != "DATA" {
        return Ok(None);
    }
    // We're only interested in the byte value (not individual bits).
    match &items[2] {
        Item::List(pdata) => match pdata.first() {
            Some(Item::Int(v)) => Ok(Some(*v)),
            _ => Err(Error::arg("UART data has no byte value")),
        },
        _ => Err(Error::arg("UART data is not a sequence")),
    }
}

impl Decoder for Midi {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, _config: &Config) -> Result<()> {
        *self = Midi::default();
        Ok(())
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("midi is a stacked decoder"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        let Some(value) = data_byte(item)? else { return Ok(()) };
        self.decode_byte(ctx, ss, es, value)
    }
}
