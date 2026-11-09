// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! A catalog: the tools one server offers, and the ways to narrow it.
//!
//! `--read-only`, `--no-hardware`, `--tools` and a UI's checklist all go
//! through [`Catalog::select`], so there is one mechanism and it is
//! testable.

use super::tool::{Call, Tag, Tool, ToolDesc, ToolError, ToolResult};
use serde_json::Value;
use std::fmt::Write as _;
use std::sync::Arc;

/// A catalog refused a name.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CatalogError {
    /// `only` named a tool the catalog does not have.
    #[error("Unknown tool '{0}'.")]
    UnknownTool(String),
    /// Two tools with one name.
    #[error("Duplicate tool '{0}'.")]
    DuplicateTool(String),
}

/// The tools one server offers, in the order `tools/list` gives them.
#[derive(Clone, Default)]
pub struct Catalog {
    tools: Vec<Arc<dyn Tool>>,
}

impl std::fmt::Debug for Catalog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list().entries(self.names()).finish()
    }
}

impl Catalog {
    /// No tools.
    pub fn new() -> Self {
        Self::default()
    }

    /// A catalog of `tools`, refusing a duplicate name.
    pub fn from_tools(tools: Vec<Arc<dyn Tool>>) -> Result<Self, CatalogError> {
        let mut c = Self::new();
        for t in tools {
            c.push(t)?;
        }
        Ok(c)
    }

    /// Adds a tool, refusing a name already taken.
    pub fn push(&mut self, tool: Arc<dyn Tool>) -> Result<(), CatalogError> {
        let name = &tool.desc().name;
        if self.get(name).is_some() {
            return Err(CatalogError::DuplicateTool(name.clone()));
        }
        self.tools.push(tool);
        Ok(())
    }

    /// Keeps the tools `keep` says yes to.
    #[must_use]
    pub fn select(mut self, keep: impl Fn(&ToolDesc) -> bool) -> Self {
        self.tools.retain(|t| keep(t.desc()));
        self
    }

    /// Drops the tools named; a name not in the catalog is no error.
    #[must_use]
    pub fn without(self, names: &[&str]) -> Self {
        self.select(|d| !names.contains(&d.name.as_str()))
    }

    /// Keeps only the tools named, every one of which must exist.
    pub fn only(self, names: &[&str]) -> Result<Self, CatalogError> {
        for n in names {
            if self.get(n).is_none() {
                return Err(CatalogError::UnknownTool((*n).to_owned()));
            }
        }
        Ok(self.select(|d| names.contains(&d.name.as_str())))
    }

    /// Both catalogs' tools, refusing a name in both.
    pub fn merge(mut self, other: Catalog) -> Result<Self, CatalogError> {
        for t in other.tools {
            self.push(t)?;
        }
        Ok(self)
    }

    /// Every tool renamed with `prefix` in front, for a server whose own
    /// tools would otherwise collide with the library's.
    #[must_use]
    pub fn prefixed(self, prefix: &str) -> Self {
        let tools = self
            .tools
            .into_iter()
            .map(|inner| {
                let mut desc = inner.desc().clone();
                desc.name = format!("{prefix}{}", desc.name);
                Arc::new(Prefixed { inner, desc }) as Arc<dyn Tool>
            })
            .collect();
        Self { tools }
    }

    /// Keeps the tools that change nothing.
    #[must_use]
    pub fn read_only(self) -> Self {
        self.select(|d| d.annotations.read_only)
    }

    /// Drops the tools carrying `tag`.
    #[must_use]
    pub fn without_tag(self, tag: Tag) -> Self {
        self.select(|d| !d.has_tag(tag))
    }

    /// The tool called `name`.
    pub fn get(&self, name: &str) -> Option<&Arc<dyn Tool>> {
        self.tools.iter().find(|t| t.desc().name == name)
    }

    /// The tools, in order.
    pub fn iter(&self) -> impl Iterator<Item = &Arc<dyn Tool>> {
        self.tools.iter()
    }

    /// The names, in order.
    pub fn names(&self) -> Vec<&str> {
        self.tools.iter().map(|t| t.desc().name.as_str()).collect()
    }

    /// How many tools.
    pub fn len(&self) -> usize {
        self.tools.len()
    }

    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.tools.is_empty()
    }

    /// The `tools/list` result.
    pub fn to_json(&self) -> Value {
        let tools: Vec<Value> = self.tools.iter().map(|t| t.desc().to_json()).collect();
        serde_json::json!({"tools": tools})
    }

    /// The tool reference as Markdown, one section per tool, for the
    /// docs. `oxrok mcp --list-tools --markdown` prints it and a test
    /// keeps `docs/MCP.md` equal to it.
    pub fn markdown(&self) -> String {
        let mut out = String::new();
        for t in &self.tools {
            let d = t.desc();
            let _ = writeln!(out, "### `{}`\n", d.name);
            let tags: Vec<&str> = d.tags.iter().map(|t| t.name()).collect();
            let effect = if d.annotations.read_only {
                "Read-only."
            } else if d.annotations.destructive {
                "Acts on hardware or runs commands."
            } else {
                "Changes the server's state."
            };
            let _ = writeln!(out, "{}. Tags: {}. {}\n", d.title, tags.join(", "), effect);
            let _ = writeln!(out, "{}\n", d.description);
            let props = d.input_schema.get("properties").and_then(Value::as_object);
            match props {
                Some(props) if !props.is_empty() => {
                    out.push_str("Arguments:\n\n");
                    let required: Vec<&str> = d
                        .input_schema
                        .get("required")
                        .and_then(Value::as_array)
                        .map(|a| a.iter().filter_map(Value::as_str).collect())
                        .unwrap_or_default();
                    for (name, schema) in props {
                        let mut kind = schema_type(schema);
                        if required.contains(&name.as_str()) {
                            kind.push_str(", required");
                        }
                        let desc = schema
                            .get("description")
                            .and_then(Value::as_str)
                            .unwrap_or("")
                            .split_whitespace()
                            .collect::<Vec<_>>()
                            .join(" ");
                        let _ = writeln!(out, "- `{name}` ({kind}): {desc}");
                    }
                    out.push('\n');
                }
                _ => out.push_str("No arguments.\n\n"),
            }
            let _ = writeln!(out, "Example: `{}`\n", d.example);
        }
        out
    }
}

/// A property's type as the docs name it.
fn schema_type(schema: &Value) -> String {
    let base = match schema.get("type") {
        Some(Value::String(s)) => s.clone(),
        Some(Value::Array(a)) => {
            a.iter().filter_map(Value::as_str).find(|s| *s != "null").unwrap_or("any").to_owned()
        }
        _ if schema.get("$ref").is_some() => "object".to_owned(),
        _ if schema.get("enum").is_some() => "string".to_owned(),
        _ => "any".to_owned(),
    };
    if base == "array"
        && let Some(items) = schema.get("items")
    {
        return format!("array of {}", schema_type(items));
    }
    if let Some(values) = schema.get("enum").and_then(Value::as_array) {
        let names: Vec<String> = values.iter().map(ToString::to_string).collect();
        return format!("{base}: {}", names.join(", "));
    }
    base
}

struct Prefixed {
    inner: Arc<dyn Tool>,
    desc: ToolDesc,
}

impl Tool for Prefixed {
    fn desc(&self) -> &ToolDesc {
        &self.desc
    }

    fn call(&self, args: Value, cx: &mut Call) -> Result<ToolResult, ToolError> {
        self.inner.call(args, cx)
    }

    fn check_args(&self, args: &Value) -> Result<(), String> {
        self.inner.check_args(args)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::tool::{Annotations, ToolSpec, function};
    use schemars::JsonSchema;
    use serde::Deserialize;
    use serde_json::json;

    #[derive(Deserialize, JsonSchema)]
    struct NoArgs {}

    fn tool(name: &'static str, annotations: Annotations, tags: &'static [Tag]) -> Arc<dyn Tool> {
        function::<NoArgs, _>(
            ToolSpec {
                name,
                title: name,
                description: "A test tool.",
                example: json!({}),
                annotations,
                tags,
            },
            move |_, _| Ok(ToolResult::text(name)),
        )
    }

    fn catalog() -> Catalog {
        Catalog::from_tools(vec![
            tool("list", Annotations::READ_ONLY, &[Tag::Catalog]),
            tool("open", Annotations::HARDWARE, &[Tag::Device, Tag::Hardware]),
            tool("drop", Annotations::MUTATING, &[Tag::Capture]),
        ])
        .unwrap()
    }

    #[test]
    fn narrows_in_every_way() {
        assert_eq!(catalog().names(), ["list", "open", "drop"]);
        assert_eq!(catalog().read_only().names(), ["list"]);
        assert_eq!(catalog().without_tag(Tag::Hardware).names(), ["list", "drop"]);
        assert_eq!(catalog().without(&["drop", "nope"]).names(), ["list", "open"]);
        assert_eq!(catalog().only(&["drop", "list"]).unwrap().names(), ["list", "drop"]);
        assert_eq!(
            catalog().only(&["list", "nope"]).unwrap_err(),
            CatalogError::UnknownTool("nope".into())
        );
        assert_eq!(catalog().select(|d| d.has_tag(Tag::Capture)).names(), ["drop"]);
    }

    #[test]
    fn merges_and_prefixes() {
        let ui = Catalog::from_tools(vec![tool("view", Annotations::READ_ONLY, &[])]).unwrap();
        let both = catalog().merge(ui).unwrap();
        assert_eq!(both.names(), ["list", "open", "drop", "view"]);
        let dup = Catalog::from_tools(vec![tool("open", Annotations::READ_ONLY, &[])]).unwrap();
        assert_eq!(catalog().merge(dup).unwrap_err(), CatalogError::DuplicateTool("open".into()));
        let p = catalog().prefixed("oxrok_");
        assert_eq!(p.names(), ["oxrok_list", "oxrok_open", "oxrok_drop"]);
        let r = p.get("oxrok_list").unwrap().call(json!({}), &mut Call::detached()).unwrap();
        assert_eq!(r, ToolResult::text("list"));
        assert!(p.get("oxrok_list").unwrap().check_args(&json!({})).is_ok());
    }

    #[test]
    fn renders_the_docs() {
        let md = catalog().read_only().markdown();
        assert!(
            md.starts_with("### `list`\n\nlist. Tags: catalog. Read-only.\n\nA test tool.\n\n")
        );
        assert!(md.contains("No arguments.\n\nExample: `{}`\n"), "{md}");
        assert_eq!(schema_type(&json!({"type": ["string", "null"]})), "string");
        assert_eq!(
            schema_type(&json!({"type": "array", "items": {"type": "integer"}})),
            "array of integer"
        );
        assert_eq!(
            schema_type(&json!({"type": "string", "enum": ["a", "b"]})),
            "string: \"a\", \"b\""
        );
    }
}
