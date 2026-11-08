// Counterpart in sigrok:
//   sigrok-cli/decode.c,
//   Copyright (C) 2013 Bert Vermeulen <bert@biot.com>,
//   with commits by Gerhard Sittig, Uwe Hermann and others (see the
//   sigrok-cli history).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! Decoder stacks from `-P`, and what `-A`, `-B` and `-M` select.

use super::generic_args;
use crate::core::{Channel, ChannelKind, Error, Result};
use crate::decode::{Config, Decoder, DecoderInfo, Registry, Stack};
use std::collections::HashMap;

/// One decoder instance of a run: `i2c-1`, `eeprom24xx-1`.
pub struct Instance {
    /// The instance id, the decoder id with a count.
    pub id: String,
    /// The decoder.
    pub info: &'static DecoderInfo,
}

/// An instance waiting for the first logic packet: id, configuration
/// and constructor.
type Pending = (String, Config, &'static (dyn Fn() -> Box<dyn Decoder> + Send + Sync));

/// The decoder stacks of a run, parsed and checked, not yet started: a
/// [`Stack`] needs the unit size of the first logic packet.
pub struct StackPlan {
    /// Every instance, in the order given.
    pub instances: Vec<Instance>,
    /// The chains, bottom first.
    chains: Vec<Vec<Pending>>,
}

impl std::fmt::Debug for StackPlan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list().entries(self.instances.iter().map(|i| &i.id)).finish()
    }
}

/// Which annotation classes of a decoder are shown: `None` for all.
pub type Visible = HashMap<&'static str, Option<Vec<usize>>>;

/// The logic channels' names, each at its bit in a sample: a channel's place in
/// the device's list says nothing, as a driver may list analog channels first. A
/// bit no channel has is an empty name, which no value matches. The length is at
/// least the number of channels, which a decoder's default mapping counts: a
/// default on a bit no channel has reads 0, as under sigrok-cli.
pub fn logic_bit_names(channels: &[Channel]) -> Vec<String> {
    let logic = || channels.iter().filter(|c| c.kind == ChannelKind::Logic);
    let bits = logic().map(|c| c.index + 1).max().unwrap_or(0);
    let mut names = vec![String::new(); bits.max(channels.len())];
    for c in logic() {
        names[c.index].clone_from(&c.name);
    }
    names
}

/// A decoder's channel takes a logic channel. sigrok-cli hands it the bit of the
/// analog channel's index, which may be another channel's or none.
fn refuse_analog(
    info: &DecoderInfo,
    pairs: &[(String, String)],
    channels: &[Channel],
) -> Result<()> {
    for (key, value) in pairs {
        let is_channel = info
            .channels
            .iter()
            .chain(info.optional_channels.iter())
            .any(|c| c.id.eq_ignore_ascii_case(key));
        let named = |kind| channels.iter().any(|c| c.kind == kind && c.name == *value);
        if is_channel && named(ChannelKind::Analog) && !named(ChannelKind::Logic) {
            return Err(Error::Arg(format!(
                "Channel '{value}' is analog: decoder '{}' needs a logic channel for '{key}'.",
                info.id
            )));
        }
    }
    Ok(())
}

/// Parses the `-P` specs (`i2c:scl=D0:sda=D1,eeprom24xx`, one chain each)
/// over `channels`.
pub fn parse_stacks(
    registry: &Registry,
    specs: &[String],
    channels: &[Channel],
) -> Result<StackPlan> {
    let names = logic_bit_names(channels);
    let mut instances = Vec::new();
    let mut chains = Vec::new();
    let mut counts: HashMap<&str, usize> = HashMap::new();
    for spec in specs {
        let mut chain: Vec<Pending> = Vec::new();
        let mut below: Option<&'static DecoderInfo> = None;
        for part in spec.split(',') {
            let parsed = generic_args(part, true);
            let id = parsed.scope.ok_or_else(|| Error::arg("Invalid decoder specification."))?;
            let def = registry
                .find(&id)
                .ok_or_else(|| Error::Arg(format!("Protocol decoder '{id}' not found.")))?;
            if let Some(b) = below
                && !def.info.inputs.iter().any(|i| b.outputs.contains(i))
            {
                // libsigrokdecode runs such a stack anyway; the upper
                // decoder then gets items it cannot use.
                log::warn!(
                    "Protocol decoder '{}' does not take the output of '{}'.",
                    def.info.id,
                    b.id
                );
            }
            let pairs: Vec<(String, String)> =
                parsed.items.into_iter().map(|(k, v)| (k, v.unwrap_or_default())).collect();
            let channel_names: &[String] = if below.is_none() { &names } else { &[] };
            if below.is_none() {
                refuse_analog(def.info, &pairs, channels)?;
            }
            let config = crate::decode::parse_spec(def.info, &pairs, channel_names)?;
            let n = counts.entry(def.info.id).or_insert(0);
            *n += 1;
            let inst_id = format!("{}-{n}", def.info.id);
            instances.push(Instance { id: inst_id.clone(), info: def.info });
            chain.push((inst_id, config, def.new));
            below = Some(def.info);
        }
        chains.push(chain);
    }
    Ok(StackPlan { instances, chains })
}

impl StackPlan {
    /// The decoder of an instance id.
    pub fn info_of(&self, instance: &str) -> Option<&'static DecoderInfo> {
        self.instances.iter().find(|i| i.id == instance).map(|i| i.info)
    }

    /// Starts every chain on a [`Stack`] for samples of `unit_size` bytes
    /// at `samplerate` (0 when unknown).
    pub fn build(self, unit_size: usize, samplerate: u64) -> Result<Stack> {
        let mut stack = Stack::new();
        for chain in self.chains {
            let built: Vec<(String, Box<dyn Decoder>, Config)> =
                chain.into_iter().map(|(id, config, new)| (id, new(), config)).collect();
            stack.add_chain(built, unit_size, samplerate)?;
        }
        Ok(stack)
    }
}

/// The decoder among `instances` called `dec`, in sigrok-cli's words when
/// there is none.
fn instance_info(instances: &[Instance], dec: &str) -> Result<&'static DecoderInfo> {
    instances
        .iter()
        .find(|i| i.info.id == dec)
        .map(|i| i.info)
        .ok_or_else(|| Error::Arg(format!("Protocol decoder '{dec}' not found.")))
}

/// What `-A` shows: `decoder[=class[:class]...][,decoder...]`, each spec
/// one `-A`. With `all_by_default`, every instance is visible when no
/// `-A` was given (sigrok-cli: when neither `-A`, `-B` nor `-M` was).
pub fn visible_annotations(
    instances: &[Instance],
    specs: &[String],
    all_by_default: bool,
) -> Result<Visible> {
    let mut visible: Visible = HashMap::new();
    if specs.is_empty() && all_by_default {
        for i in instances {
            visible.insert(i.info.id, None);
        }
    }
    for spec in specs {
        for item in spec.split(',') {
            let (dec, classes) = item.split_once('=').map_or((item, None), |(d, c)| (d, Some(c)));
            let info = instance_info(instances, dec)?;
            let Some(classes) = classes else {
                visible.insert(info.id, None);
                continue;
            };
            let mut list = Vec::new();
            for c in classes.split(':') {
                let found = info.ann_classes(c);
                if found.is_empty() {
                    return Err(Error::Arg(format!(
                        "Annotation class '{c}' not found for decoder '{dec}'."
                    )));
                }
                list.extend(found);
            }
            visible
                .entry(info.id)
                .and_modify(|v| {
                    if let Some(v) = v {
                        v.extend(list.iter().copied());
                    }
                })
                .or_insert(Some(list));
        }
    }
    Ok(visible)
}

/// What `-B decoder=class` selects: the decoder id and the binary class
/// index.
pub fn binary_class(
    instances: &[Instance],
    spec: Option<&str>,
) -> Result<Option<(&'static str, usize)>> {
    let Some(spec) = spec else { return Ok(None) };
    let (dec, class) =
        spec.split_once('=').ok_or_else(|| Error::arg("Binary output needs decoder=class."))?;
    let info = instance_info(instances, dec)?;
    let idx = info.binary.iter().position(|b| b.id == class).ok_or_else(|| {
        Error::Arg(format!("Binary class '{class}' not found for decoder '{dec}'."))
    })?;
    Ok(Some((info.id, idx)))
}

/// What `-M decoder[,decoder...]` selects.
pub fn meta_decoders(instances: &[Instance], specs: &[String]) -> Result<Vec<&'static str>> {
    let mut list = Vec::new();
    for spec in specs {
        for dec in spec.split(',') {
            list.push(instance_info(instances, dec)?.id);
        }
    }
    Ok(list)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn registry() -> Registry {
        let mut r = Registry::new();
        r.register(&crate::decoders::ALL);
        r
    }

    fn channels(logic: &[&str], analog: &[&str]) -> Vec<Channel> {
        let mut list: Vec<Channel> = analog
            .iter()
            .enumerate()
            .map(|(i, n)| Channel::new(i, ChannelKind::Analog, *n))
            .collect();
        list.extend(logic.iter().enumerate().map(|(i, n)| Channel::new(i, ChannelKind::Logic, *n)));
        list
    }

    fn strings(items: &[&str]) -> Vec<String> {
        items.iter().map(|s| (*s).to_owned()).collect()
    }

    #[test]
    fn bit_names_follow_the_index_not_the_list() {
        let names = logic_bit_names(&channels(&["D0", "D1"], &["A0"]));
        assert_eq!(names, ["D0", "D1", ""]);
    }

    #[test]
    fn a_name_sits_at_its_logic_bit_whatever_the_list_s_order() {
        let mixed = vec![
            Channel::new(4, ChannelKind::Analog, "DSO"),
            Channel::new(0, ChannelKind::Logic, "0"),
            Channel::new(1, ChannelKind::Logic, "1"),
            Channel::new(4, ChannelKind::Logic, "4"),
        ];
        assert_eq!(logic_bit_names(&mixed), ["0", "1", "", "", "4"]);
    }

    #[test]
    fn the_names_are_at_least_as_many_as_the_channels() {
        let channels =
            [Channel::new(0, ChannelKind::Logic, "D0"), Channel::new(1, ChannelKind::Analog, "A0")];
        assert_eq!(logic_bit_names(&channels), ["D0", ""]);
    }

    #[test]
    fn parses_chains_and_numbers_instances() {
        let chans = channels(&["D0", "D1", "D2", "D3"], &[]);
        let plan = parse_stacks(
            &registry(),
            &strings(&["i2c:scl=D0:sda=D1,eeprom24xx", "i2c:scl=D2:sda=D3"]),
            &chans,
        )
        .unwrap();
        let ids: Vec<&str> = plan.instances.iter().map(|i| i.id.as_str()).collect();
        assert_eq!(ids, ["i2c-1", "eeprom24xx-1", "i2c-2"]);
        assert_eq!(plan.info_of("eeprom24xx-1").unwrap().id, "eeprom24xx");
        let stack = plan.build(1, 1_000_000).unwrap();
        assert_eq!(stack.instances().len(), 3);
        let e = parse_stacks(&registry(), &strings(&["nope"]), &chans).unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m == "Protocol decoder 'nope' not found."), "{e}");
    }

    #[test]
    fn an_analog_channel_is_refused() {
        let chans = channels(&["D0", "D1"], &["A0"]);
        let e = parse_stacks(&registry(), &strings(&["i2c:scl=A0:sda=D1"]), &chans).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "Channel 'A0' is analog: decoder 'i2c' needs a logic channel for 'scl'."),
            "{e}"
        );
    }

    #[test]
    fn resolves_what_a_b_and_m_select() {
        let chans = channels(&["D0", "D1"], &[]);
        let plan =
            parse_stacks(&registry(), &strings(&["i2c:scl=D0:sda=D1,eeprom24xx"]), &chans).unwrap();
        let all = visible_annotations(&plan.instances, &[], true).unwrap();
        assert_eq!(all.len(), 2);
        assert!(all["i2c"].is_none());
        assert!(visible_annotations(&plan.instances, &[], false).unwrap().is_empty());
        let some =
            visible_annotations(&plan.instances, &strings(&["i2c=address-read:data-read"]), true)
                .unwrap();
        assert_eq!(some.len(), 1);
        assert_eq!(some["i2c"].as_ref().unwrap().len(), 2);
        let e = visible_annotations(&plan.instances, &strings(&["i2c=nope"]), true).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "Annotation class 'nope' not found for decoder 'i2c'."),
            "{e}"
        );
        let e = visible_annotations(&plan.instances, &strings(&["spi"]), true).unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m == "Protocol decoder 'spi' not found."), "{e}");
        assert_eq!(binary_class(&plan.instances, None).unwrap(), None);
        let e = binary_class(&plan.instances, Some("i2c")).unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m == "Binary output needs decoder=class."), "{e}");
        let e = binary_class(&plan.instances, Some("i2c=nope")).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "Binary class 'nope' not found for decoder 'i2c'."),
            "{e}"
        );
        assert_eq!(
            meta_decoders(&plan.instances, &strings(&["i2c,eeprom24xx"])).unwrap(),
            ["i2c", "eeprom24xx"]
        );
        assert!(meta_decoders(&plan.instances, &strings(&["uart"])).is_err());
    }
}
