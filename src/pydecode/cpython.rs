// Counterpart in sigrok:
//   libsigrokdecode (srd.c, decoder.c, instance.c, type_decoder.c: the
//   Python side of the decoder API).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the documented decoder API and the observed behaviour.

//! The CPython backend, through pyo3: the system's Python runs the
//! decoders. A `sigrokdecode` module written in Rust provides what the
//! decoders import: the `Decoder` base class with `wait`, `put`,
//! `register` and `has_channel`, and the output constants. `decode`
//! runs on the instance's thread with the interpreter attached, and
//! `wait` releases the interpreter while it blocks for samples.

#![allow(clippy::doc_markdown)]

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};
use pyo3::exceptions::PyEOFError;
use pyo3::prelude::*;
use pyo3::types::{
    PyByteArray, PyByteArrayMethods, PyBytes, PyDict, PyInt, PyList, PyString, PyTuple,
};
use std::sync::{Arc, OnceLock};

use super::{CONF_SAMPLERATE, leak};

const OUTPUT_ANN: u8 = 0;
const OUTPUT_PYTHON: u8 = 1;
const OUTPUT_BINARY: u8 = 2;
const OUTPUT_META: u8 = 3;

/// A Python object travelling between two Python decoders as
/// [`Item::Opaque`].
pub struct PyItem(pub Py<PyAny>);

impl PyItem {
    /// The object's value as an [`Item`], with lists and tuples
    /// converted recursively. Objects with no value form stay
    /// [`Item::Opaque`]. For comparing what two hosts emit.
    pub fn to_item(&self) -> Item {
        Python::attach(|py| plain_item(self.0.bind(py)))
    }
}

/// Like [`py_to_item`], but lists and tuples become [`Item::List`].
fn plain_item(obj: &Bound<'_, PyAny>) -> Item {
    if let Ok(l) = obj.cast::<PyList>() {
        return Item::List(l.iter().map(|v| plain_item(&v)).collect());
    }
    if let Ok(t) = obj.cast::<PyTuple>() {
        return Item::List(t.iter().map(|v| plain_item(&v)).collect());
    }
    py_to_item(obj)
}

fn init_interpreter(dir: &std::path::Path) -> Result<()> {
    static ONCE: OnceLock<()> = OnceLock::new();
    ONCE.get_or_init(|| {
        pyo3::append_to_inittab!(sigrokdecode);
    });
    Python::attach(|py| -> PyResult<()> {
        let sys = py.import("sys")?;
        let path = sys.getattr("path")?;
        let dir = dir.to_string_lossy().into_owned();
        if !path.contains(&dir)? {
            path.call_method1("insert", (0, dir))?;
        }
        Ok(())
    })
    .map_err(|e| Error::device(format!("python: {e}")))
}

pub(super) fn scan(dir: &std::path::Path, names: Vec<String>) -> Result<Vec<&'static DecoderDef>> {
    init_interpreter(dir)?;
    let mut defs = Vec::new();
    Python::attach(|py| {
        for name in names {
            match load_info(py, &name) {
                Ok(info) => {
                    let info: &'static DecoderInfo = Box::leak(Box::new(info));
                    let module = leak(name.clone());
                    let factory: &'static (dyn Fn() -> Box<dyn Decoder> + Send + Sync) =
                        Box::leak(Box::new(move || {
                            Box::new(PyDecoder::new(info, module)) as Box<dyn Decoder>
                        }));
                    defs.push(&*Box::leak(Box::new(DecoderDef { info, new: factory })));
                }
                Err(e) => log::debug!("python decoder {name}: {e}"),
            }
        }
    });
    Ok(defs)
}

/// A class attribute the decoder defined itself; the base class's
/// descriptors do not count.
fn class_attr<'py>(class: &Bound<'py, PyAny>, name: &str) -> Option<Bound<'py, PyAny>> {
    let v = class.getattr(name).ok()?;
    let tname = v.get_type().name().ok()?.to_string();
    if tname == "getset_descriptor" || tname == "method_descriptor" || v.is_none() {
        return None;
    }
    Some(v)
}

fn attr_str(class: &Bound<'_, PyAny>, name: &str) -> &'static str {
    class_attr(class, name).and_then(|v| v.extract::<String>().ok()).map_or("", leak)
}

fn attr_strs(class: &Bound<'_, PyAny>, name: &str) -> &'static [&'static str] {
    let v: Vec<&'static str> = class_attr(class, name)
        .and_then(|v| v.extract::<Vec<String>>().ok())
        .unwrap_or_default()
        .into_iter()
        .map(leak)
        .collect();
    Box::leak(v.into_boxed_slice())
}

fn channel_descs(class: &Bound<'_, PyAny>, name: &str) -> PyResult<&'static [ChannelDesc]> {
    let mut out = Vec::new();
    if let Some(list) = class_attr(class, name) {
        for d in list.try_iter()? {
            let d = d?;
            let get = |k: &str| d.get_item(k).and_then(|v| v.extract::<String>()).map(leak);
            out.push(ChannelDesc {
                id: get("id")?,
                name: get("name")?,
                desc: get("desc").unwrap_or(""),
            });
        }
    }
    Ok(Box::leak(out.into_boxed_slice()))
}

fn option_value(v: &Bound<'_, PyAny>) -> PyResult<OptionValue> {
    if v.is_instance_of::<pyo3::types::PyBool>() {
        return Ok(OptionValue::Int(i64::from(v.extract::<bool>()?)));
    }
    if let Ok(i) = v.extract::<i64>() {
        return Ok(OptionValue::Int(i));
    }
    if let Ok(f) = v.extract::<f64>() {
        return Ok(OptionValue::Float(f));
    }
    Ok(OptionValue::Str(std::borrow::Cow::Owned(v.str()?.to_string())))
}

fn load_info(py: Python<'_>, module: &str) -> PyResult<DecoderInfo> {
    let m = py.import(module)?;
    let class = m.getattr("Decoder")?;
    let api: i64 = class.getattr("api_version")?.extract()?;
    if api != 3 {
        return Err(pyo3::exceptions::PyValueError::new_err(format!("api_version {api} is not 3")));
    }
    let mut options = Vec::new();
    if let Some(list) = class_attr(&class, "options") {
        for o in list.try_iter()? {
            let o = o?;
            let default = o.get_item("default").map(|v| option_value(&v))??;
            let mut values = Vec::new();
            if let Ok(vs) = o.get_item("values") {
                for v in vs.try_iter()? {
                    values.push(option_value(&v?)?);
                }
            }
            options.push(OptionDesc {
                id: leak(o.get_item("id")?.extract()?),
                desc: leak(o.get_item("desc")?.extract()?),
                default,
                values: Box::leak(values.into_boxed_slice()),
            });
        }
    }
    let mut annotations = Vec::new();
    if let Some(list) = class_attr(&class, "annotations") {
        for a in list.try_iter()? {
            let (id, desc): (String, String) = a?.extract()?;
            annotations.push(AnnClass { id: leak(id), desc: leak(desc) });
        }
    }
    let mut rows = Vec::new();
    if let Some(list) = class_attr(&class, "annotation_rows") {
        for r in list.try_iter()? {
            let (id, desc, classes): (String, String, Vec<usize>) = r?.extract()?;
            rows.push(AnnRow {
                id: leak(id),
                desc: leak(desc),
                classes: Box::leak(classes.into_boxed_slice()),
            });
        }
    }
    let mut binary = Vec::new();
    if let Some(list) = class_attr(&class, "binary") {
        for b in list.try_iter()? {
            let (id, desc): (String, String) = b?.extract()?;
            binary.push(BinClass { id: leak(id), desc: leak(desc) });
        }
    }
    let doc =
        m.getattr("__doc__").ok().and_then(|d| d.extract::<String>().ok()).unwrap_or_default();
    Ok(DecoderInfo {
        id: attr_str(&class, "id"),
        name: attr_str(&class, "name"),
        longname: attr_str(&class, "longname"),
        desc: attr_str(&class, "desc"),
        license: attr_str(&class, "license"),
        inputs: attr_strs(&class, "inputs"),
        outputs: attr_strs(&class, "outputs"),
        tags: attr_strs(&class, "tags"),
        channels: channel_descs(&class, "channels")?,
        optional_channels: channel_descs(&class, "optional_channels")?,
        options: Box::leak(options.into_boxed_slice()),
        annotations: Box::leak(annotations.into_boxed_slice()),
        annotation_rows: Box::leak(rows.into_boxed_slice()),
        binary: Box::leak(binary.into_boxed_slice()),
        doc: leak(doc.trim().to_owned()),
    })
}

/// A pointer to the context of the running `decode` call. The Python
/// methods reach the context through it; it is set only while the
/// call runs, on the thread that runs it.
type CtxPtr = *mut (dyn Context + 'static);

/// The pointer in a form the interpreter's object model accepts.
struct CtxCell(CtxPtr);

// SAFETY: the pointer is only dereferenced on the thread running the
// decode call that set it, and never outlives that call.
unsafe impl Send for CtxCell {}
unsafe impl Sync for CtxCell {}

/// The `sigrokdecode.Decoder` base class.
#[pyclass(subclass, name = "Decoder", module = "sigrokdecode")]
struct SrdDecoder {
    /// Registered outputs: kind and meta name.
    outputs: Vec<(u8, Option<String>)>,
    channels: Vec<bool>,
    ctx: Option<CtxCell>,
}

impl SrdDecoder {
    fn ctx(&mut self) -> PyResult<&mut dyn Context> {
        let cell = self
            .ctx
            .as_ref()
            .ok_or_else(|| pyo3::exceptions::PyRuntimeError::new_err("not decoding"))?;
        let p: CtxPtr = cell.0;
        // SAFETY: `ctx` is set by `PyDecoder` around the call into
        // Python on this thread, from a reference that outlives the
        // call, and cleared afterwards.
        Ok(unsafe { &mut *p })
    }
}

fn parse_conds(conds: Option<&Bound<'_, PyAny>>) -> PyResult<Vec<Vec<Cond>>> {
    let Some(obj) = conds else { return Ok(Vec::new()) };
    if obj.is_none() {
        return Ok(Vec::new());
    }
    let dicts: Vec<Bound<'_, PyDict>> = if let Ok(d) = obj.cast::<PyDict>() {
        vec![d.clone()]
    } else {
        obj.try_iter()?
            .map(|d| d.and_then(|d| d.cast_into::<PyDict>().map_err(Into::into)))
            .collect::<PyResult<_>>()?
    };
    let mut out = Vec::new();
    for d in dicts {
        let mut g = Vec::new();
        for (k, v) in d.iter() {
            if let Ok(s) = k.extract::<String>() {
                if s == "skip" {
                    let n: i64 = v.extract()?;
                    g.push(Cond::Skip(n.max(0) as u64));
                }
                continue;
            }
            let ch: usize = k.extract()?;
            let kind = match v.extract::<String>()?.as_str() {
                "l" | "low" => CondKind::Low,
                "h" | "high" => CondKind::High,
                "r" | "rising" => CondKind::Rising,
                "f" | "falling" => CondKind::Falling,
                "e" | "edge" => CondKind::Edge,
                "s" | "stable" => CondKind::Stable,
                other => {
                    return Err(pyo3::exceptions::PyValueError::new_err(format!(
                        "unknown condition '{other}'"
                    )));
                }
            };
            g.push(Cond::Pin(ch, kind));
        }
        out.push(g);
    }
    Ok(out)
}

/// Converts a Python object to an [`Item`], keeping the object itself
/// too when it is not a plain value, so a Python decoder above gets it
/// unchanged.
fn py_to_item(obj: &Bound<'_, PyAny>) -> Item {
    if obj.is_none() {
        return Item::None;
    }
    if obj.is_instance_of::<pyo3::types::PyBool>() {
        return Item::Bool(obj.extract().unwrap_or(false));
    }
    if let Ok(i) = obj.extract::<i64>() {
        return Item::Int(i);
    }
    // A wider int: keep every bit instead of letting it become a float.
    if obj.is_instance_of::<PyInt>() {
        let bytes = obj
            .call_method0("bit_length")
            .and_then(|n| n.extract::<usize>())
            .and_then(|n| obj.call_method1("to_bytes", (n.div_ceil(8), "big")));
        if let Ok(Ok(b)) = bytes.as_ref().map(|b| b.cast::<PyBytes>()) {
            return Item::BigUint(b.as_bytes().to_vec());
        }
    }
    if let Ok(f) = obj.extract::<f64>() {
        return Item::Float(f);
    }
    if let Ok(s) = obj.cast::<PyString>() {
        return Item::Str(s.to_string_lossy().into_owned());
    }
    if let Ok(b) = obj.cast::<PyByteArray>() {
        return Item::Bytes(b.to_vec());
    }
    if let Ok(b) = obj.cast::<PyBytes>() {
        return Item::Bytes(b.as_bytes().to_vec());
    }
    // Snapshot lists and tuples: Python decoders keep emitting the same
    // mutable object (e.g. ieee488's `last_listener`) and later mutate it,
    // so holding a reference would make the observed value depend on when
    // the consumer looks at it. A stacked decoder gets an equal list back.
    if let Ok(l) = obj.cast::<PyList>() {
        return Item::List(l.iter().map(|v| py_to_item(&v)).collect());
    }
    if let Ok(t) = obj.cast::<PyTuple>() {
        return Item::List(t.iter().map(|v| py_to_item(&v)).collect());
    }
    Item::Opaque(Arc::new(PyItem(obj.clone().unbind())))
}

fn item_to_py<'py>(py: Python<'py>, item: &Item) -> PyResult<Bound<'py, PyAny>> {
    Ok(match item {
        Item::Str(s) => PyString::new(py, s).into_any(),
        Item::Int(i) => i.into_pyobject(py)?.into_any(),
        Item::BigUint(b) => {
            py.get_type::<PyInt>().call_method1("from_bytes", (PyBytes::new(py, b), "big"))?
        }
        Item::Float(f) => f.into_pyobject(py)?.into_any(),
        Item::Bool(b) => pyo3::types::PyBool::new(py, *b).to_owned().into_any(),
        Item::None => py.None().into_bound(py),
        Item::Bytes(b) => PyBytes::new(py, b).into_any(),
        Item::List(items) => {
            let l = PyList::empty(py);
            for it in items {
                l.append(item_to_py(py, it)?)?;
            }
            l.into_any()
        }
        Item::Opaque(o) => match o.downcast_ref::<PyItem>() {
            Some(p) => p.0.bind(py).clone(),
            None => py.None().into_bound(py),
        },
    })
}

#[pymethods]
impl SrdDecoder {
    #[new]
    fn new() -> Self {
        SrdDecoder { outputs: Vec::new(), channels: Vec::new(), ctx: None }
    }

    #[pyo3(signature = (output_type, proto_id=None, meta=None))]
    fn register(
        &mut self,
        output_type: u8,
        proto_id: Option<&Bound<'_, PyAny>>,
        meta: Option<&Bound<'_, PyAny>>,
    ) -> usize {
        let _ = proto_id;
        let name = meta.and_then(|m| m.get_item(1).ok()).and_then(|n| n.extract::<String>().ok());
        self.outputs.push((output_type, name));
        self.outputs.len() - 1
    }

    fn has_channel(&self, channel: usize) -> bool {
        self.channels.get(channel).copied().unwrap_or(false)
    }

    fn put(&mut self, ss: i64, es: i64, output_id: usize, data: &Bound<'_, PyAny>) -> PyResult<()> {
        let (kind, meta_name) = self
            .outputs
            .get(output_id)
            .cloned()
            .ok_or_else(|| pyo3::exceptions::PyValueError::new_err("unknown output id"))?;
        let (ss, es) = (ss.max(0) as u64, es.max(0) as u64);
        let item = py_to_item(data);
        let ctx = self.ctx()?;
        match kind {
            OUTPUT_ANN => {
                let class: usize = data.get_item(0)?.extract()?;
                let texts: Vec<String> = data.get_item(1)?.extract()?;
                ctx.put_ann(ss, es, class, texts);
            }
            OUTPUT_BINARY => {
                let class: usize = data.get_item(0)?.extract()?;
                let bytes: Vec<u8> = data.get_item(1)?.extract()?;
                ctx.put_binary(ss, es, class, bytes);
            }
            OUTPUT_META => ctx.put_meta(ss, es, meta_name.as_deref().unwrap_or(""), item),
            _ => ctx.put_python(ss, es, item),
        }
        Ok(())
    }

    /// `samplenum` and `matched` are set on the instance, as
    /// libsigrokdecode does, so a decoder's class attributes of the same
    /// names are not in the way.
    #[pyo3(signature = (conds=None))]
    fn wait<'py>(
        slf: &Bound<'py, Self>,
        conds: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyTuple>> {
        let py = slf.py();
        let groups = parse_conds(conds)?;
        let refs: Vec<&[Cond]> = groups.iter().map(Vec::as_slice).collect();
        let (pins, samplenum, matched) = {
            let mut me = slf.borrow_mut();
            let ctx = me.ctx()?;
            let r = py.detach(|| ctx.wait(&refs));
            let pins = match r {
                Ok(p) => p,
                Err(Error::Eof) => return Err(PyEOFError::new_err("end of data")),
                Err(e) => return Err(pyo3::exceptions::PyRuntimeError::new_err(e.to_string())),
            };
            (pins, ctx.samplenum(), ctx.matched().to_vec())
        };
        slf.setattr("samplenum", samplenum)?;
        slf.setattr("matched", PyTuple::new(py, matched)?)?;
        PyTuple::new(py, pins.0)
    }
}

#[pymodule]
fn sigrokdecode(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SrdDecoder>()?;
    m.add("OUTPUT_ANN", OUTPUT_ANN)?;
    m.add("OUTPUT_PYTHON", OUTPUT_PYTHON)?;
    m.add("OUTPUT_BINARY", OUTPUT_BINARY)?;
    m.add("OUTPUT_META", OUTPUT_META)?;
    m.add("SRD_CONF_SAMPLERATE", CONF_SAMPLERATE)?;
    Ok(())
}

/// A Python decoder instance.
pub struct PyDecoder {
    info: &'static DecoderInfo,
    module: &'static str,
    obj: Option<Py<PyAny>>,
    /// The sample rate from `metadata`, kept for `start` because
    /// libsigrokdecode delivers it before `start` while the stack calls
    /// `start` first.
    samplerate: u64,
}

impl PyDecoder {
    fn new(info: &'static DecoderInfo, module: &'static str) -> Self {
        PyDecoder { info, module, obj: None, samplerate: 0 }
    }

    fn py_err(e: &PyErr, py: Python<'_>) -> Error {
        let tb = e.traceback(py).and_then(|t| t.format().ok()).unwrap_or_default();
        Error::Device(format!("{e}\n{tb}").trim_end().to_owned())
    }

    /// Runs `f` with the context reachable from the Python methods.
    #[allow(clippy::transmute_ptr_to_ptr)]
    fn with_ctx<T>(
        &self,
        ctx: &mut dyn Context,
        f: impl FnOnce(Python<'_>, &Bound<'_, PyAny>) -> PyResult<T>,
    ) -> Result<T> {
        let obj = self.obj.as_ref().ok_or_else(|| Error::device("decoder not started"))?;
        Python::attach(|py| {
            let obj = obj.bind(py);
            let base = obj.cast::<SrdDecoder>().map_err(|e| Error::device(e.to_string()))?;
            // SAFETY: the pointer is used only inside `f`, on this
            // thread, while `ctx` is borrowed; see `SrdDecoder::ctx`.
            let ptr: CtxPtr =
                unsafe { std::mem::transmute::<*mut (dyn Context + '_), CtxPtr>(ctx) };
            base.borrow_mut().ctx = Some(CtxCell(ptr));
            let r = f(py, obj);
            base.borrow_mut().ctx = None;
            match r {
                Ok(v) => Ok(v),
                Err(e) if e.is_instance_of::<PyEOFError>(py) => Err(Error::Eof),
                Err(e) => Err(Self::py_err(&e, py)),
            }
        })
    }
}

impl Decoder for PyDecoder {
    fn info(&self) -> &'static DecoderInfo {
        self.info
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        Python::attach(|py| -> PyResult<()> {
            let class = py.import(self.module)?.getattr("Decoder")?;
            let obj = class.call0()?;
            let options = PyDict::new(py);
            for o in self.info.options {
                let v = config.options.get(o.id).unwrap_or(&o.default);
                let value: Bound<'_, PyAny> = match v {
                    OptionValue::Int(i) => i.into_pyobject(py)?.into_any(),
                    OptionValue::Float(f) => f.into_pyobject(py)?.into_any(),
                    OptionValue::Str(s) => PyString::new(py, s).into_any(),
                };
                options.set_item(o.id, value)?;
            }
            obj.setattr("options", options)?;
            obj.setattr("samplenum", 0)?;
            obj.setattr("matched", PyTuple::empty(py))?;
            {
                let base = obj.cast::<SrdDecoder>()?;
                base.borrow_mut().channels = config.channels.iter().map(Option::is_some).collect();
            }
            // libsigrokdecode calls metadata() before start(), and some
            // decoders (swim, for one) read the samplerate there.
            if self.samplerate > 0 && obj.hasattr("metadata").unwrap_or(false) {
                obj.call_method1("metadata", (CONF_SAMPLERATE, self.samplerate))?;
            }
            obj.call_method0("start")?;
            self.obj = Some(obj.unbind());
            Ok(())
        })
        .map_err(|e| Python::attach(|py| Self::py_err(&e, py)))
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        if let Some(obj) = &self.obj {
            Python::attach(|py| {
                let obj = obj.bind(py);
                if obj.hasattr("metadata").unwrap_or(false)
                    && let Err(e) = obj.call_method1("metadata", (CONF_SAMPLERATE, samplerate))
                {
                    log::warn!("{}: metadata: {e}", self.info.id);
                }
            });
        }
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.with_ctx(ctx, |_, obj| obj.call_method0("decode").map(|_| ()))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        self.with_ctx(ctx, |py, obj| {
            let data = item_to_py(py, item)?;
            obj.call_method1("decode", (ss, es, data)).map(|_| ())
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{Item, item_to_py, py_to_item};
    use pyo3::prelude::*;

    #[test]
    fn wide_integers_cross_into_python_and_back_intact() {
        Python::attach(|py| {
            // 2^152 - 1, the largest 152 bit SPI word.
            let item = Item::BigUint(vec![0xff; 19]);
            let obj = item_to_py(py, &item).unwrap();
            assert_eq!(
                obj.str().unwrap().to_string(),
                "5708990770823839524233143877797980545530986495"
            );
            assert!(matches!(py_to_item(&obj), Item::BigUint(ref b) if b == &vec![0xff; 19]));
            let small = 42i64.into_pyobject(py).unwrap().into_any();
            assert!(matches!(py_to_item(&small), Item::Int(42)));
        });
    }
}
