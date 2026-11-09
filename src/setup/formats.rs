// Counterpart in sigrok:
//   sigrok-cli/session.c, input.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `-I` and `-O`: a module and its options from `module:key=value...`.

use super::generic_args;
use crate::core::{Error, Result};
use crate::formats::input::{self, InputModule};
use crate::formats::output::{self, OutputModule};
use std::collections::HashMap;

/// The output module and options of an `-O` spec.
pub fn output_spec(spec: &str) -> Result<(&'static OutputModule, HashMap<String, String>)> {
    let mut parsed = generic_args(spec, true);
    let id = parsed.scope.take().ok_or_else(|| Error::arg("Invalid output format."))?;
    let module =
        output::find(&id).ok_or_else(|| Error::Arg(format!("Unknown output module '{id}'.")))?;
    Ok((module, parsed.text_options()))
}

/// The input module and options of an `-I` spec.
pub fn input_spec(spec: &str) -> Result<(&'static InputModule, HashMap<String, String>)> {
    let mut parsed = generic_args(spec, true);
    let id = parsed.scope.take().unwrap_or_default();
    let module = input::find(&id)
        .ok_or_else(|| Error::Arg(format!("Error: unknown input module '{id}'.")))?;
    Ok((module, parsed.text_options()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_module_specs() {
        let Ok((m, o)) = output_spec("bits:width=64") else { panic!("bits") };
        assert_eq!(m.id, "bits");
        assert_eq!(o.get("width").map(String::as_str), Some("64"));
        let Err(e) = output_spec("nope") else { panic!("an error") };
        assert!(matches!(&e, Error::Arg(m) if m == "Unknown output module 'nope'."), "{e}");
        let Err(e) = output_spec("") else { panic!("an error") };
        assert!(matches!(&e, Error::Arg(m) if m == "Invalid output format."), "{e}");
        let Ok((m, o)) = input_spec("vcd:downsample=2") else { panic!("vcd") };
        assert_eq!(m.id, "vcd");
        assert_eq!(o.get("downsample").map(String::as_str), Some("2"));
        let Err(e) = input_spec("") else { panic!("an error") };
        assert!(matches!(&e, Error::Arg(m) if m == "Error: unknown input module ''."), "{e}");
    }
}
