// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The library's tools, and the catalog of all of them.

pub mod catalog;
pub mod cli;

use super::Host;
use std::sync::Arc;

/// Every tool the library has, over `host`: the reference tools and the
/// `cli` pass-through. A server narrows it from here.
pub fn catalog(host: &Arc<Host>) -> super::Catalog {
    let mut tools = catalog::tools(host);
    tools.push(cli::tool(host));
    super::Catalog::from_tools(tools).expect("the library's tool names are distinct")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::tool::Call;
    use serde_json::Value;

    /// Every tool's example parses as its arguments: the schema a client
    /// sees and the parser that reads a call agree at least on that.
    #[test]
    fn every_example_parses() {
        let host = Arc::new(Host::new());
        for tool in catalog(&host).iter() {
            let d = tool.desc();
            assert!(d.input_schema["type"] == "object", "{}: {}", d.name, d.input_schema);
            assert!(!d.description.is_empty() && !d.title.is_empty(), "{}", d.name);
            if let Err(e) = tool.check_args(&d.example) {
                panic!("{}: example {} does not parse: {e}", d.name, d.example);
            }
        }
    }

    /// The reference tools run without hardware and give JSON.
    #[test]
    fn reference_tools_answer() {
        let host = Arc::new(Host::new());
        let catalog = catalog(&host);
        let mut cx = Call::detached();
        for name in ["version", "list_drivers", "list_decoders", "list_formats", "firmware_status"]
        {
            let tool = catalog.get(name).unwrap();
            let r = tool.call(tool.desc().example.clone(), &mut cx).unwrap();
            assert!(matches!(r.structured, Some(Value::Object(_))), "{name}");
        }
    }
}
