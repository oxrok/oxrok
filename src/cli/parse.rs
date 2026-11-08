// Counterpart in sigrok:
//   sigrok-cli/parsers.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The sigrok-cli argument syntaxes, over [`oxrok::setup`]: the parsers
//! live in the library, shared with the MCP server; what is here takes
//! the command line's [`Args`] and hands them the pieces.

use crate::cli::args::Args;
use oxrok::core::{Device, Registry, Result};
pub use oxrok::setup::{
    ConfigPlan, driver_spec, generic_args, open_device, option_args, parse_triggers, resolve_key,
    select_channels,
};

/// Scans for devices: the `-d` driver, or every driver.
pub fn device_scan(registry: &Registry, args: &Args) -> Result<Vec<Box<dyn Device>>> {
    oxrok::setup::scan(registry, args.driver.as_deref())
}

/// The scope of a `-c`, a `--get` or a `--show`: its own `channel_group=`,
/// else `-g`; see [`oxrok::setup::resolve_scope`].
pub fn resolve_scope(
    device: &dyn Device,
    own: Option<&str>,
    args: &Args,
) -> Result<Option<String>> {
    oxrok::setup::resolve_scope(device, own, args.channel_group.as_deref())
}

/// Resolves and checks every `-c` option; see [`oxrok::setup::plan_configs`].
pub fn plan_configs(device: &dyn Device, args: &Args) -> Result<ConfigPlan> {
    oxrok::setup::plan_configs(device, &args.configs, args.channel_group.as_deref())
}

/// Applies every `-c` option to the device; see [`oxrok::setup::apply_configs`].
pub fn apply_configs(device: &mut dyn Device, args: &Args) -> Result<()> {
    oxrok::setup::apply_configs(device, &args.configs, args.channel_group.as_deref())
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser as _;
    use oxrok::core::Error;

    /// `-g` reaches the library's planner as written, and a `-c` reaches
    /// the device.
    #[test]
    fn the_flags_reach_the_planner() {
        let mut r = Registry::new();
        r.register(Box::new(oxrok::drivers::demo::DemoDriver));
        let args = Args::try_parse_from(["oxrok", "-d", "demo", "--set", "-g", "R1"]).unwrap();
        let mut dev = device_scan(&r, &args).unwrap().remove(0);
        let e = resolve_scope(dev.as_ref(), None, &args).unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m == "Invalid channel group 'R1'"), "{e}");
        let args = Args::try_parse_from(["oxrok", "--set", "-c", "samplerate=1m"]).unwrap();
        apply_configs(dev.as_mut(), &args).unwrap();
        assert_eq!(dev.get(&oxrok::core::Key::Samplerate, None).unwrap().as_u64(), Some(1_000_000));
    }
}
