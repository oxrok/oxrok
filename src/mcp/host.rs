// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The host: what the tools work on.
//!
//! For now the driver and decoder registries and the limits; the open
//! devices and the capture store come with the device tools.

use crate::core::Registry;
use crate::decode;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Bounds the tools keep.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Limits {
    /// How long the `cli` tool waits for a command before killing it.
    pub cli_timeout: Duration,
    /// How much of each of a command's stdout and stderr the `cli` tool
    /// keeps.
    pub cli_max_bytes: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self { cli_timeout: Duration::from_secs(60), cli_max_bytes: 1 << 20 }
    }
}

/// What the tools work on.
pub struct Host {
    registry: Registry,
    decoders: decode::Registry,
    limits: Limits,
    cli_program: Option<PathBuf>,
}

impl Default for Host {
    fn default() -> Self {
        Self::new()
    }
}

impl Host {
    /// Every driver and decoder this build has, default limits, and the
    /// running executable as the `cli` program.
    pub fn new() -> Self {
        let mut registry = Registry::new();
        crate::register_all(&mut registry);
        Self::with(registry, crate::decoder_registry())
    }

    /// A host over the given registries.
    pub fn with(registry: Registry, decoders: decode::Registry) -> Self {
        Self {
            registry,
            decoders,
            limits: Limits::default(),
            cli_program: std::env::current_exe().ok(),
        }
    }

    /// Sets the limits.
    #[must_use]
    pub fn with_limits(mut self, limits: Limits) -> Self {
        self.limits = limits;
        self
    }

    /// Sets the program the `cli` tool runs, for a host inside another
    /// executable.
    #[must_use]
    pub fn with_cli_program(mut self, program: PathBuf) -> Self {
        self.cli_program = Some(program);
        self
    }

    /// The drivers.
    pub fn registry(&self) -> &Registry {
        &self.registry
    }

    /// The decoders.
    pub fn decoders(&self) -> &decode::Registry {
        &self.decoders
    }

    /// The limits.
    pub fn limits(&self) -> &Limits {
        &self.limits
    }

    /// The program the `cli` tool runs, when one is known.
    pub fn cli_program(&self) -> Option<&Path> {
        self.cli_program.as_deref()
    }
}
