// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `oxrok mcp`: the MCP server over stdio.

use crate::cli::args::McpArgs;
use oxrok::core::util::parse_sizestring;
use oxrok::core::{Error, Result};
use oxrok::mcp::{Catalog, Host, Limits, Server, ServerInfo, Tag, tools};
use std::sync::Arc;
use std::time::Duration;

/// Builds the catalog the flags ask for and serves it, or lists it.
pub fn run(args: &McpArgs) -> Result<()> {
    let timeout = parse_time(&args.timeout)?;
    let max_memory = parse_sizestring(&args.max_memory)
        .map_err(|_| Error::Arg(format!("Invalid size '{}'.", args.max_memory)))?;
    let mut limits = Limits {
        cli_timeout: timeout,
        capture_timeout: timeout,
        max_memory: usize::try_from(max_memory).unwrap_or(usize::MAX),
        ..Limits::default()
    };
    if !args.drivers.is_empty() {
        limits.drivers = Some(args.drivers.clone());
    }
    if !args.allow_paths.is_empty() {
        limits.allow_paths.clone_from(&args.allow_paths);
    }
    let host = Arc::new(Host::new().with_limits(limits));
    let catalog = select(tools::catalog(&host), args)?;
    if args.list_tools {
        if args.json {
            println!("{}", serde_json::to_string_pretty(&catalog.to_json()).unwrap_or_default());
        } else if args.markdown {
            print!("{}", catalog.markdown());
        } else {
            for tool in catalog.iter() {
                let d = tool.desc();
                let tags: Vec<&str> = d.tags.iter().map(|t| t.name()).collect();
                println!("{:<18} {:<28} [{}]", d.name, d.title, tags.join(", "));
            }
        }
        return Ok(());
    }
    let mut info = ServerInfo::oxrok();
    if let Some(path) = &args.instructions {
        info.instructions = Some(std::fs::read_to_string(path).map_err(|e| {
            Error::Arg(format!("Cannot read instructions from {}: {e}", path.display()))
        })?);
    }
    let server = Server::new(info, catalog);
    // Ctrl-C closes what is open before the process goes, as EOF does.
    let on_exit = host.clone();
    let _ = ctrlc::set_handler(move || {
        on_exit.close_all();
        std::process::exit(130);
    });
    log::info!(
        "oxrok mcp serving {} tools on stdio.",
        server.catalog().read().map_or(0, |c| c.len())
    );
    server.serve_stdio()?;
    host.close_all();
    Ok(())
}

/// Narrows `catalog` as the flags say: `--no-cli`, `--no-hardware`,
/// `--read-only`, then `--tools`.
fn select(mut catalog: Catalog, args: &McpArgs) -> Result<Catalog> {
    if args.no_cli {
        catalog = catalog.without(&["cli"]);
    }
    if args.no_hardware {
        catalog = catalog.without_tag(Tag::Hardware);
    }
    if args.read_only {
        catalog = catalog.read_only();
    }
    if !args.tools.is_empty() {
        let names: Vec<&str> = args.tools.iter().map(String::as_str).collect();
        catalog = catalog.only(&names).map_err(|e| Error::Arg(e.to_string()))?;
    }
    Ok(catalog)
}

/// `30`, `30s`, `500ms`, `2m` as a duration.
fn parse_time(text: &str) -> Result<Duration> {
    let bad = || Error::Arg(format!("Invalid time '{text}'."));
    let (number, scale) = if let Some(n) = text.strip_suffix("ms") {
        (n, 0.001)
    } else if let Some(n) = text.strip_suffix('s') {
        (n, 1.0)
    } else if let Some(n) = text.strip_suffix('m') {
        (n, 60.0)
    } else {
        (text, 1.0)
    };
    let value: f64 = number.trim().parse().map_err(|_| bad())?;
    if !value.is_finite() || value < 0.0 {
        return Err(bad());
    }
    Ok(Duration::from_secs_f64(value * scale))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_times() {
        assert_eq!(parse_time("30").unwrap(), Duration::from_secs(30));
        assert_eq!(parse_time("30s").unwrap(), Duration::from_secs(30));
        assert_eq!(parse_time("500ms").unwrap(), Duration::from_millis(500));
        assert_eq!(parse_time("2m").unwrap(), Duration::from_secs(120));
        assert_eq!(parse_time("1.5s").unwrap(), Duration::from_millis(1500));
        assert!(parse_time("soon").is_err());
        assert!(parse_time("-1").is_err());
    }
}
