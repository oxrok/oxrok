// Counterpart in sigrok:
//   sigrok-cli/parsers.c, input.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! `-C`: which channels, under which names.

use crate::core::{Channel, Device, Error, Result};

/// Applies `-C` to a device: enables the listed channels, disables the
/// rest, renames `name=newname` entries. Accepts names and numeric ranges
/// (`0-3`).
pub fn select_channels(device: &mut dyn Device, spec: &str) -> Result<()> {
    let names: Vec<String> = device.channels().iter().map(|c| c.name.clone()).collect();
    let find = |name: &str| names.iter().position(|n| n == name);
    let mut selected: Vec<(usize, Option<String>)> = Vec::new();
    for token in spec.split(',') {
        if token.is_empty() {
            return Err(Error::arg("Invalid empty channel."));
        }
        if let Some((lo, hi)) = token.split_once('-') {
            let (b, e): (usize, usize) = match (lo.parse(), hi.parse()) {
                (Ok(b), Ok(e)) if b < e => (b, e),
                _ => return Err(Error::Arg(format!("Invalid channel range '{token}'."))),
            };
            for n in b..=e {
                let pos = find(&n.to_string())
                    .ok_or_else(|| Error::Arg(format!("unknown channel '{n}'.")))?;
                selected.push((pos, None));
            }
        } else {
            let (name, rename) = match token.split_once('=') {
                Some((n, r)) => (n, Some(r.to_owned())),
                None => (token, None),
            };
            let pos = find(name).ok_or_else(|| Error::Arg(format!("unknown channel '{name}'.")))?;
            selected.push((pos, rename));
        }
    }
    for pos in 0..names.len() {
        device.enable_channel(pos, selected.iter().any(|(p, _)| *p == pos))?;
    }
    for (pos, rename) in selected {
        if let Some(name) = rename {
            device.rename_channel(pos, &name)?;
        }
    }
    Ok(())
}

/// Applies `-C` to a channel list, a file's: names or numeric ranges,
/// `name=newname` renames.
pub fn select_channel_list(channels: &mut [Channel], spec: &str) -> Result<()> {
    let names: Vec<String> = channels.iter().map(|c| c.name.clone()).collect();
    let mut keep = vec![false; channels.len()];
    for token in spec.split(',') {
        if let Some((lo, hi)) = token.split_once('-')
            && let (Ok(b), Ok(e)) = (lo.parse::<usize>(), hi.parse::<usize>())
        {
            for n in b..=e {
                let pos = names
                    .iter()
                    .position(|x| *x == n.to_string())
                    .ok_or_else(|| Error::Arg(format!("unknown channel '{n}'.")))?;
                keep[pos] = true;
            }
            continue;
        }
        let (name, rename) = token.split_once('=').map_or((token, None), |(n, r)| (n, Some(r)));
        let pos = names
            .iter()
            .position(|x| x == name)
            .ok_or_else(|| Error::Arg(format!("unknown channel '{name}'.")))?;
        keep[pos] = true;
        if let Some(r) = rename {
            r.clone_into(&mut channels[pos].name);
        }
    }
    for (c, k) in channels.iter_mut().zip(keep) {
        c.enabled = k;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Registry;

    fn demo(logic: i32) -> Box<dyn Device> {
        let mut r = Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        let (d, o) = super::super::driver_spec(
            &r,
            &format!("demo:logic_channels={logic}:analog_channels=1"),
        )
        .unwrap();
        d.scan(&o).unwrap().remove(0)
    }

    #[test]
    fn selects_and_renames_channels() {
        let mut dev = demo(4);
        select_channels(dev.as_mut(), "D0,D2=clk,A0").unwrap();
        let enabled: Vec<&str> =
            dev.channels().iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
        assert_eq!(enabled, ["D0", "clk", "A0"]);
        assert!(select_channels(dev.as_mut(), "D9").is_err());
    }

    #[test]
    fn selects_in_a_list() {
        let dev = demo(4);
        let mut list = dev.channels().to_vec();
        select_channel_list(&mut list, "D1=sda,D3").unwrap();
        let enabled: Vec<&str> =
            list.iter().filter(|c| c.enabled).map(|c| c.name.as_str()).collect();
        assert_eq!(enabled, ["sda", "D3"]);
        assert!(select_channel_list(&mut list, "D9").is_err());
    }
}
