// Counterpart in sigrok:
//   sigrok-cli/options.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Command line flags, sigrok-cli compatible.

use clap::{Parser, Subcommand};
use std::path::PathBuf;

/// A sigrok compatible logic analyzer command line tool.
#[derive(Parser, Debug)]
#[allow(clippy::struct_excessive_bools, clippy::doc_markdown)]
#[command(
    name = "oxrok",
    disable_version_flag = true,
    disable_help_subcommand = true,
    args_conflicts_with_subcommands = true,
    subcommand_negates_reqs = true,
    about,
    long_about = None
)]
pub struct Args {
    /// `oxrok mcp`: the other things this binary does.
    #[command(subcommand)]
    pub command: Option<Command>,

    /// Show version and support list
    #[arg(short = 'V', long)]
    pub version: bool,

    /// List supported devices/modules/decoders
    #[arg(short = 'L', long = "list-supported")]
    pub list_supported: bool,

    /// List the firmware and bitstream files the drivers need, as JSON
    #[arg(long = "list-firmware", hide = true)]
    pub list_firmware: bool,

    /// Set loglevel (0-5, default 2)
    #[arg(short = 'l', long, default_value_t = 2, value_name = "LEVEL")]
    pub loglevel: u8,

    /// Treat warnings as errors: stop and exit with failure
    #[arg(long)]
    pub strict: bool,

    /// The driver to use, with scan options: driver[:key=value]...
    #[arg(short = 'd', long, value_name = "DRIVER")]
    pub driver: Option<String>,

    /// Device option: [channel_group=GROUP:]key=value[:key=value]...
    #[arg(short = 'c', long = "config", value_name = "CONFIG")]
    pub configs: Vec<String>,

    /// Load input from file (- for stdin)
    #[arg(short = 'i', long = "input-file", value_name = "FILE")]
    pub input_file: Option<String>,

    /// Input format: module[:option=value]...
    #[arg(short = 'I', long = "input-format", value_name = "FORMAT")]
    pub input_format: Option<String>,

    /// Save output to file
    #[arg(short = 'o', long = "output-file", value_name = "FILE")]
    pub output_file: Option<PathBuf>,

    /// Output format: module[:option=value]...
    #[arg(short = 'O', long = "output-format", value_name = "FORMAT")]
    pub output_format: Option<String>,

    /// Channels to use: names or ranges, comma separated, name=newname to rename
    #[arg(short = 'C', long, value_name = "CHANNELS")]
    pub channels: Option<String>,

    /// Channel group to show or configure
    #[arg(short = 'g', long = "channel-group", value_name = "GROUP")]
    pub channel_group: Option<String>,

    /// Trigger: channel=matches[,channel=matches]... with ; between stages
    #[arg(short = 't', long, value_name = "TRIGGERS")]
    pub triggers: Option<String>,

    /// Wait for trigger before saving data
    #[arg(short = 'w', long = "wait-trigger")]
    pub wait_trigger: bool,

    /// Scan for devices
    #[arg(long)]
    pub scan: bool,

    /// Show device details
    #[arg(long)]
    pub show: bool,

    /// How long to sample (ms, or with a unit: 2s, 500ms)
    #[arg(long, value_name = "TIME")]
    pub time: Option<String>,

    /// Number of samples to acquire (with size suffix: 1k, 2m)
    #[arg(long, value_name = "SAMPLES")]
    pub samples: Option<String>,

    /// Number of frames to acquire
    #[arg(long, value_name = "FRAMES")]
    pub frames: Option<String>,

    /// Sample continuously until stopped
    #[arg(long)]
    pub continuous: bool,

    /// Protocol decoders to run: decoder[:option=value][:channel=NAME]...
    #[arg(short = 'P', long = "protocol-decoders", value_name = "DECODERS")]
    pub decoders: Vec<String>,

    /// Annotations to show: decoder[=class[:class]...][,decoder...]
    #[arg(short = 'A', long = "protocol-decoder-annotations", value_name = "ANN")]
    pub decoder_annotations: Vec<String>,

    /// Binary output to show: decoder=class
    #[arg(short = 'B', long = "protocol-decoder-binary", value_name = "BIN")]
    pub decoder_binary: Option<String>,

    /// Decoder metadata to show: decoder[,decoder...]
    #[arg(short = 'M', long = "protocol-decoder-meta", value_name = "META")]
    pub decoder_meta: Vec<String>,

    /// Show sample numbers with annotations
    #[arg(long = "protocol-decoder-samplenum")]
    pub decoder_samplenum: bool,

    /// Show the annotation class with annotations
    #[arg(long = "protocol-decoder-ann-class")]
    pub decoder_ann_class: bool,

    /// Get device option only: [channel_group=GROUP:]key
    #[arg(long = "get", value_name = "KEY")]
    pub gets: Vec<String>,

    /// Set device options only
    #[arg(long)]
    pub set: bool,
}

/// The subcommands. Every sigrok-cli flag keeps its meaning without one.
#[derive(Subcommand, Debug)]
pub enum Command {
    /// Serve the Model Context Protocol over stdio, for LLM agents
    Mcp(McpArgs),
}

/// `oxrok mcp`.
#[derive(clap::Args, Debug)]
#[allow(clippy::struct_excessive_bools)]
pub struct McpArgs {
    /// Set loglevel (0-5, default 2); logs go to stderr
    #[arg(short = 'l', long, default_value_t = 2, value_name = "LEVEL")]
    pub loglevel: u8,

    /// Serve only the tools that change nothing
    #[arg(long)]
    pub read_only: bool,

    /// Drop the tools that open or drive hardware
    #[arg(long)]
    pub no_hardware: bool,

    /// Drop the `cli` pass-through
    #[arg(long)]
    pub no_cli: bool,

    /// Serve only these tools, comma separated
    #[arg(long, value_name = "NAMES", value_delimiter = ',')]
    pub tools: Vec<String>,

    /// How long the `cli` tool waits for a command (seconds, or with a unit: 30s, 2m)
    #[arg(long, value_name = "TIME", default_value = "60s")]
    pub timeout: String,

    /// Print the tools the server would offer and exit
    #[arg(long)]
    pub list_tools: bool,

    /// With --list-tools: print the tools/list result as JSON
    #[arg(long, requires = "list_tools")]
    pub json: bool,

    /// With --list-tools: print the tool reference as Markdown
    #[arg(long, requires = "list_tools", conflicts_with = "json")]
    pub markdown: bool,

    /// Send this file's text to the model as the instructions
    #[arg(long, value_name = "FILE")]
    pub instructions: Option<PathBuf>,
}
