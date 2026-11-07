// Counterpart in sigrok:
//   libsigrokdecode (srd.c, decoder.c, instance.c, type_decoder.c: the
//   Python side of the decoder API).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the documented decoder API and the observed behaviour.

//! The RustPython backend: an interpreter built into the binary runs the
//! decoders, with its standard library frozen in. `sigrokdecode` is the
//! Python module in `sigrokdecode.py`; the native `_oxrok` module under
//! it reaches the running decoder's [`Context`].
//!
//! RustPython has no global interpreter lock: every thread that runs
//! Python gets its own virtual machine on the shared interpreter, and
//! `wait` detaches the thread while it blocks for samples.

use super::{CONF_SAMPLERATE, leak};
use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, ChannelDesc, Cond, CondKind, Config, Context, Decoder, DecoderDef,
    DecoderInfo, Item, OptionDesc, OptionValue,
};
use malachite_bigint::{BigInt, Sign};
use rustpython_vm::builtins::{
    PyBaseExceptionRef, PyByteArray, PyBytes, PyFloat, PyInt, PyList, PyStr, PyTuple,
};
use rustpython_vm::vm::thread::ThreadedVirtualMachine;
use rustpython_vm::{AsObject, Interpreter, PyObjectRef, PyResult, VirtualMachine, pymodule};
use std::cell::RefCell;
use std::path::Path;
use std::sync::{Arc, Mutex, OnceLock};

const SHIM: &str = include_str!("sigrokdecode.py");

/// Installs the shim as `sigrokdecode` and puts the decoders directory
/// first on `sys.path`. Older decoders have invalid escapes in plain
/// strings; the compiler's warnings about them are not for the user.
/// Nothing is written next to the decoders.
const BOOT: &str = "\
import sys
import warnings
sys.dont_write_bytecode = True
warnings.simplefilter('ignore', SyntaxWarning)
_m = type(sys)('sigrokdecode')
exec(compile(_src, 'sigrokdecode.py', 'exec'), _m.__dict__)
sys.modules['sigrokdecode'] = _m
if _dir not in sys.path:
    sys.path.insert(0, _dir)
";

/// A Python object travelling between two Python decoders as
/// [`Item::Opaque`].
pub struct RpItem(pub PyObjectRef);

impl RpItem {
    /// The object's value as an [`Item`], with lists and tuples
    /// converted recursively. Objects with no value form stay
    /// [`Item::Opaque`]. For comparing what two hosts emit.
    pub fn to_item(&self) -> Item {
        with_vm(|vm| plain_item(&self.0, vm))
    }
}

/// Like [`py_to_item`], but lists and tuples become [`Item::List`].
fn plain_item(obj: &PyObjectRef, vm: &VirtualMachine) -> Item {
    if let Some(l) = obj.downcast_ref::<PyList>() {
        return Item::List(l.borrow_vec().iter().map(|o| plain_item(o, vm)).collect());
    }
    if let Some(t) = obj.downcast_ref::<PyTuple>() {
        return Item::List(t.as_slice().iter().map(|o| plain_item(o, vm)).collect());
    }
    py_to_item(obj, vm)
}

/// The virtual machine new threads are made from.
static ROOT: OnceLock<std::result::Result<Mutex<ThreadedVirtualMachine>, String>> = OnceLock::new();

thread_local! {
    /// This thread's virtual machine, made from [`ROOT`] on first use.
    static VM: RefCell<Option<ThreadedVirtualMachine>> = const { RefCell::new(None) };
    /// The contexts of the `decode` calls running on this thread,
    /// innermost last: a stacked decoder runs inside the `put` of the
    /// one below it.
    static CTX: RefCell<Vec<CtxPtr>> = const { RefCell::new(Vec::new()) };
}

/// An interpreter with the frozen standard library and `_oxrok`.
fn interpreter() -> Interpreter {
    let builder = Interpreter::builder(rustpython_vm::Settings::default());
    let mut defs = rustpython_stdlib::stdlib_module_defs(&builder.ctx);
    defs.push(oxrok_native::module_def(&builder.ctx));
    builder
        .add_native_modules(&defs)
        .add_frozen_modules(rustpython_pylib::FROZEN_STDLIB)
        .init_hook(|vm| {
            if let Some(state) = rustpython_vm::common::rc::PyRc::get_mut(&mut vm.state) {
                state.config.paths.stdlib_dir = Some(rustpython_pylib::LIB_PATH.to_owned());
            }
        })
        .build()
}

fn root(dir: &Path) -> Result<&'static Mutex<ThreadedVirtualMachine>> {
    ROOT.get_or_init(|| {
        let interp = interpreter();
        let dir = dir.to_string_lossy().into_owned();
        interp
            .enter(|vm| {
                let scope = vm.new_scope_with_builtins();
                let r = scope
                    .globals
                    .set_item("_src", vm.ctx.new_str(SHIM).into(), vm)
                    .and_then(|()| scope.globals.set_item("_dir", vm.ctx.new_str(dir).into(), vm))
                    .and_then(|()| vm.run_string(scope, BOOT, "<oxrok>"));
                r.map(drop).map_err(|e| exc_text(vm, &e))
            })
            .map_err(|e: String| format!("rustpython: {e}"))?;
        let vm = interp.new_thread();
        // The interpreter lives as long as the process; the threads'
        // machines share its state.
        std::mem::forget(interp);
        Ok(Mutex::new(vm))
    })
    .as_ref()
    .map_err(|e| Error::device(e.clone()))
}

/// Runs `f` on this thread's virtual machine.
fn with_vm<R>(f: impl FnOnce(&VirtualMachine) -> R) -> R {
    VM.with(|cell| {
        if cell.borrow().is_none() {
            let root = ROOT
                .get()
                .and_then(|r| r.as_ref().ok())
                .expect("the interpreter is made by scan before any decoder runs");
            let vm = root.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
            *cell.borrow_mut() = Some(vm.run(VirtualMachine::new_thread));
        }
        let guard = cell.borrow();
        let vm = guard.as_ref().expect("set above");
        vm.run(f)
    })
}

/// A Python exception as text: the traceback and the message.
fn exc_text(vm: &VirtualMachine, e: &PyBaseExceptionRef) -> String {
    let mut s = String::new();
    if vm.write_exception(&mut s, e).is_err() {
        "python exception".clone_into(&mut s);
    }
    s.trim_end().to_owned()
}

/// A plain value from `_oxrok_info`.
enum Val {
    None,
    Int(i64),
    Float(f64),
    Str(String),
    List(Vec<Val>),
}

impl Val {
    fn from_py(obj: &PyObjectRef, vm: &VirtualMachine) -> PyResult<Val> {
        if vm.is_none(obj) {
            return Ok(Val::None);
        }
        if let Some(i) = obj.downcast_ref::<PyInt>() {
            return Ok(Val::Int(i.try_to_primitive::<i64>(vm)?));
        }
        if let Some(f) = obj.downcast_ref::<PyFloat>() {
            return Ok(Val::Float(f.to_f64()));
        }
        if let Some(s) = obj.downcast_ref::<PyStr>() {
            return Ok(Val::Str(s.to_string()));
        }
        let items: Vec<PyObjectRef> = if let Some(l) = obj.downcast_ref::<PyList>() {
            l.borrow_vec().to_vec()
        } else if let Some(t) = obj.downcast_ref::<PyTuple>() {
            t.as_slice().to_vec()
        } else {
            return Err(vm.new_type_error(format!(
                "unexpected {} in decoder attributes",
                obj.class().name()
            )));
        };
        items.iter().map(|o| Val::from_py(o, vm)).collect::<PyResult<_>>().map(Val::List)
    }

    fn str(&self) -> std::result::Result<&'static str, String> {
        match self {
            Val::Str(s) => Ok(leak(s.clone())),
            _ => Err("expected a string".to_owned()),
        }
    }

    fn list(&self) -> std::result::Result<&[Val], String> {
        match self {
            Val::List(l) => Ok(l),
            _ => Err("expected a list".to_owned()),
        }
    }

    fn strs(&self) -> std::result::Result<&'static [&'static str], String> {
        let v = self.list()?.iter().map(Val::str).collect::<std::result::Result<Vec<_>, _>>()?;
        Ok(Box::leak(v.into_boxed_slice()))
    }

    fn option(&self) -> std::result::Result<OptionValue, String> {
        match self {
            Val::Int(i) => Ok(OptionValue::Int(*i)),
            Val::Float(f) => Ok(OptionValue::Float(*f)),
            Val::Str(s) => Ok(OptionValue::Str(std::borrow::Cow::Owned(s.clone()))),
            _ => Err("bad option value".to_owned()),
        }
    }

    fn index(&self) -> std::result::Result<usize, String> {
        match self {
            Val::Int(i) => usize::try_from(*i).map_err(|e| e.to_string()),
            _ => Err("expected an integer".to_owned()),
        }
    }
}

/// The fields of `_oxrok_info`, as [`DecoderInfo`].
fn info_from(v: &Val) -> std::result::Result<DecoderInfo, String> {
    let f = v.list()?;
    if f.len() != 15 {
        return Err("bad decoder description".to_owned());
    }
    let channels = |v: &Val| -> std::result::Result<&'static [ChannelDesc], String> {
        let mut out = Vec::new();
        for c in v.list()? {
            let c = c.list()?;
            out.push(ChannelDesc { id: c[0].str()?, name: c[1].str()?, desc: c[2].str()? });
        }
        Ok(Box::leak(out.into_boxed_slice()))
    };
    let mut options = Vec::new();
    for o in f[10].list()? {
        let o = o.list()?;
        let values =
            o[3].list()?.iter().map(Val::option).collect::<std::result::Result<Vec<_>, _>>()?;
        options.push(OptionDesc {
            id: o[0].str()?,
            desc: o[1].str()?,
            default: o[2].option()?,
            values: Box::leak(values.into_boxed_slice()),
        });
    }
    let mut annotations = Vec::new();
    for a in f[11].list()? {
        let a = a.list()?;
        annotations.push(AnnClass { id: a[0].str()?, desc: a[1].str()? });
    }
    let mut rows = Vec::new();
    for r in f[12].list()? {
        let r = r.list()?;
        let classes =
            r[2].list()?.iter().map(Val::index).collect::<std::result::Result<Vec<_>, _>>()?;
        rows.push(AnnRow {
            id: r[0].str()?,
            desc: r[1].str()?,
            classes: Box::leak(classes.into_boxed_slice()),
        });
    }
    let mut binary = Vec::new();
    for b in f[13].list()? {
        let b = b.list()?;
        binary.push(BinClass { id: b[0].str()?, desc: b[1].str()? });
    }
    Ok(DecoderInfo {
        id: f[0].str()?,
        name: f[1].str()?,
        longname: f[2].str()?,
        desc: f[3].str()?,
        license: f[4].str()?,
        inputs: f[5].strs()?,
        outputs: f[6].strs()?,
        tags: f[7].strs()?,
        channels: channels(&f[8])?,
        optional_channels: channels(&f[9])?,
        options: Box::leak(options.into_boxed_slice()),
        annotations: Box::leak(annotations.into_boxed_slice()),
        annotation_rows: Box::leak(rows.into_boxed_slice()),
        binary: Box::leak(binary.into_boxed_slice()),
        doc: f[14].str()?,
    })
}

fn load_info(vm: &VirtualMachine, module: &str) -> std::result::Result<DecoderInfo, String> {
    let r = (|| -> PyResult<Val> {
        let shim = vm.import("sigrokdecode", 0)?;
        let f = shim.get_attr("_oxrok_info", vm)?;
        let v = f.call((vm.ctx.new_str(module),), vm)?;
        Val::from_py(&v, vm)
    })();
    match r {
        Ok(v) => info_from(&v),
        Err(e) => Err(exc_text(vm, &e)),
    }
}

pub(super) fn scan(dir: &Path, names: Vec<String>) -> Result<Vec<&'static DecoderDef>> {
    root(dir)?;
    let mut defs = Vec::new();
    with_vm(|vm| {
        for name in names {
            match load_info(vm, &name) {
                Ok(info) => {
                    let info: &'static DecoderInfo = Box::leak(Box::new(info));
                    let module = leak(name.clone());
                    let factory: &'static (dyn Fn() -> Box<dyn Decoder> + Send + Sync) =
                        Box::leak(Box::new(move || {
                            Box::new(RpDecoder { info, module, obj: None }) as Box<dyn Decoder>
                        }));
                    defs.push(&*Box::leak(Box::new(DecoderDef { info, new: factory })));
                }
                Err(e) => log::debug!("python decoder {name}: {e}"),
            }
        }
    });
    Ok(defs)
}

/// A pointer to the context of a running `decode` call, valid while the
/// call runs, on the thread that runs it.
type CtxPtr = *mut (dyn Context + 'static);

/// Runs `f` with the innermost running context of this thread.
fn with_ctx<R>(vm: &VirtualMachine, f: impl FnOnce(&mut dyn Context) -> R) -> PyResult<R> {
    let p = CTX
        .with(|c| c.borrow().last().copied())
        .ok_or_else(|| vm.new_runtime_error("not decoding"))?;
    // SAFETY: the pointer was pushed by `RpDecoder::run` on this thread
    // from a reference that outlives the call into Python, and is popped
    // when that call returns. Nested calls push their own context, so
    // the innermost one is the decoder now running.
    Ok(f(unsafe { &mut *p }))
}

/// Converts a Python object to an [`Item`], keeping the object itself
/// when it is not a plain value, so a Python decoder above gets it
/// unchanged.
fn py_to_item(obj: &PyObjectRef, vm: &VirtualMachine) -> Item {
    if vm.is_none(obj) {
        return Item::None;
    }
    if obj.class().is(vm.ctx.types.bool_type) {
        return Item::Bool(obj.is(&vm.ctx.true_value));
    }
    if let Some(i) = obj.downcast_ref::<PyInt>() {
        let big = i.as_bigint();
        if let Ok(v) = i64::try_from(big) {
            return Item::Int(v);
        }
        if let (Sign::Plus, bytes) = big.to_bytes_be() {
            return Item::BigUint(bytes);
        }
    }
    if let Some(f) = obj.downcast_ref::<PyFloat>() {
        return Item::Float(f.to_f64());
    }
    if let Some(s) = obj.downcast_ref::<PyStr>() {
        return Item::Str(s.to_string());
    }
    if let Some(b) = obj.downcast_ref::<PyByteArray>() {
        return Item::Bytes(b.borrow_buf().to_vec());
    }
    if let Some(b) = obj.downcast_ref::<PyBytes>() {
        return Item::Bytes(b.as_bytes().to_vec());
    }
    // Snapshot lists and tuples: Python decoders keep emitting the same
    // mutable object (e.g. ieee488's `last_listener`) and later mutate it,
    // so holding a reference would make the observed value depend on when
    // the consumer looks at it. A stacked decoder gets an equal list back.
    if let Some(l) = obj.downcast_ref::<PyList>() {
        return Item::List(l.borrow_vec().iter().map(|o| py_to_item(o, vm)).collect());
    }
    if let Some(t) = obj.downcast_ref::<PyTuple>() {
        return Item::List(t.as_slice().iter().map(|o| py_to_item(o, vm)).collect());
    }
    Item::Opaque(Arc::new(RpItem(obj.clone())))
}

fn item_to_py(item: &Item, vm: &VirtualMachine) -> PyObjectRef {
    match item {
        Item::Str(s) => vm.ctx.new_str(s.as_str()).into(),
        Item::Int(i) => vm.ctx.new_int(*i).into(),
        Item::BigUint(b) => vm.ctx.new_int(BigInt::from_bytes_be(Sign::Plus, b)).into(),
        Item::Float(f) => vm.ctx.new_float(*f).into(),
        Item::Bool(b) => vm.ctx.new_bool(*b).into(),
        Item::None => vm.ctx.none(),
        Item::Bytes(b) => vm.ctx.new_bytes(b.clone()).into(),
        Item::List(items) => {
            vm.ctx.new_list(items.iter().map(|i| item_to_py(i, vm)).collect()).into()
        }
        Item::Opaque(o) => match o.downcast_ref::<RpItem>() {
            Some(p) => p.0.clone(),
            None => vm.ctx.none(),
        },
    }
}

// The macro passes the arguments by value.
#[allow(clippy::needless_pass_by_value)]
#[pymodule(name = "_oxrok")]
mod oxrok_native {
    use super::{Cond, CondKind, Error, py_to_item, with_ctx};
    use rustpython_vm::builtins::PyBytesRef;
    use rustpython_vm::{PyObjectRef, PyResult, VirtualMachine};

    fn sample(n: i64) -> u64 {
        n.max(0) as u64
    }

    #[pyfunction]
    fn put_ann(
        ss: i64,
        es: i64,
        class: usize,
        texts: Vec<String>,
        vm: &VirtualMachine,
    ) -> PyResult<()> {
        with_ctx(vm, |ctx| ctx.put_ann(sample(ss), sample(es), class, texts))
    }

    #[pyfunction]
    fn put_binary(
        ss: i64,
        es: i64,
        class: usize,
        data: PyBytesRef,
        vm: &VirtualMachine,
    ) -> PyResult<()> {
        let data = data.as_bytes().to_vec();
        with_ctx(vm, |ctx| ctx.put_binary(sample(ss), sample(es), class, data))
    }

    #[pyfunction]
    fn put_meta(
        ss: i64,
        es: i64,
        name: String,
        data: PyObjectRef,
        vm: &VirtualMachine,
    ) -> PyResult<()> {
        let item = py_to_item(&data, vm);
        with_ctx(vm, |ctx| ctx.put_meta(sample(ss), sample(es), &name, item))
    }

    #[pyfunction]
    fn put_python(ss: i64, es: i64, data: PyObjectRef, vm: &VirtualMachine) -> PyResult<()> {
        let item = py_to_item(&data, vm);
        with_ctx(vm, |ctx| ctx.put_python(sample(ss), sample(es), item))
    }

    /// Waits for the condition groups, each a flat list of channel and
    /// kind pairs, channel -1 for a skip count. Returns the pins, the
    /// sample number and the matched flags.
    #[pyfunction]
    fn wait(spec: Vec<Vec<i64>>, vm: &VirtualMachine) -> PyResult<PyObjectRef> {
        let mut groups = Vec::with_capacity(spec.len());
        for terms in spec {
            let mut g = Vec::with_capacity(terms.len() / 2);
            for &[ch, kind] in terms.as_chunks::<2>().0 {
                if ch < 0 {
                    g.push(Cond::Skip(sample(kind)));
                    continue;
                }
                let kind = match kind {
                    0 => CondKind::Low,
                    1 => CondKind::High,
                    2 => CondKind::Rising,
                    3 => CondKind::Falling,
                    4 => CondKind::Edge,
                    _ => CondKind::Stable,
                };
                g.push(Cond::Pin(ch as usize, kind));
            }
            groups.push(g);
        }
        let refs: Vec<&[Cond]> = groups.iter().map(Vec::as_slice).collect();
        let r = with_ctx(vm, |ctx| {
            vm.allow_threads(|| ctx.wait(&refs))
                .map(|pins| (pins, ctx.samplenum(), ctx.matched().to_vec()))
        })?;
        let (pins, samplenum, matched) = match r {
            Ok(v) => v,
            Err(Error::Eof) => return Err(vm.new_eof_error("end of data")),
            Err(e) => return Err(vm.new_runtime_error(e.to_string())),
        };
        let pins = vm.ctx.new_tuple(pins.0.iter().map(|p| vm.ctx.new_int(*p).into()).collect());
        let matched =
            vm.ctx.new_tuple(matched.iter().map(|m| vm.ctx.new_bool(*m).into()).collect());
        Ok(vm
            .ctx
            .new_tuple(vec![pins.into(), vm.ctx.new_int(samplenum).into(), matched.into()])
            .into())
    }
}

/// A Python decoder instance.
pub struct RpDecoder {
    info: &'static DecoderInfo,
    module: &'static str,
    obj: Option<PyObjectRef>,
}

impl RpDecoder {
    /// Calls into the instance with `ctx` reachable from `_oxrok`.
    #[allow(clippy::transmute_ptr_to_ptr)]
    fn run(
        &self,
        ctx: &mut dyn Context,
        f: impl FnOnce(&VirtualMachine, &PyObjectRef) -> PyResult<()>,
    ) -> Result<()> {
        let obj = self.obj.as_ref().ok_or_else(|| Error::device("decoder not started"))?;
        // SAFETY: the pointer is used only while `f` runs, on this
        // thread, while `ctx` is borrowed; see `with_ctx`.
        let ptr: CtxPtr = unsafe { std::mem::transmute::<*mut (dyn Context + '_), CtxPtr>(ctx) };
        CTX.with(|c| c.borrow_mut().push(ptr));
        let r = with_vm(|vm| match f(vm, obj) {
            Ok(()) => Ok(()),
            Err(e) if e.fast_isinstance(vm.ctx.exceptions.eof_error) => Err(Error::Eof),
            Err(e) => Err(Error::Device(exc_text(vm, &e))),
        });
        CTX.with(|c| c.borrow_mut().pop());
        r
    }
}

impl Decoder for RpDecoder {
    fn info(&self) -> &'static DecoderInfo {
        self.info
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        let obj = with_vm(|vm| -> std::result::Result<PyObjectRef, String> {
            let r = (|| -> PyResult<PyObjectRef> {
                let class = vm.import(self.module, 0)?.get_attr("Decoder", vm)?;
                let obj = class.call((), vm)?;
                let options = vm.ctx.new_dict();
                for o in self.info.options {
                    let v = config.options.get(o.id).unwrap_or(&o.default);
                    let value: PyObjectRef = match v {
                        OptionValue::Int(i) => vm.ctx.new_int(*i).into(),
                        OptionValue::Float(f) => vm.ctx.new_float(*f).into(),
                        OptionValue::Str(s) => vm.ctx.new_str(s.as_ref()).into(),
                    };
                    options.set_item(o.id, value, vm)?;
                }
                obj.set_attr("options", options, vm)?;
                obj.set_attr("samplenum", vm.ctx.new_int(0), vm)?;
                obj.set_attr("matched", vm.ctx.empty_tuple.clone(), vm)?;
                let channels: Vec<PyObjectRef> =
                    config.channels.iter().map(|c| vm.ctx.new_bool(c.is_some()).into()).collect();
                obj.set_attr("_oxrok_channels", vm.ctx.new_list(channels), vm)?;
                vm.call_method(&obj, "start", ())?;
                Ok(obj)
            })();
            r.map_err(|e| exc_text(vm, &e))
        })
        .map_err(Error::Device)?;
        self.obj = Some(obj);
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        let Some(obj) = &self.obj else { return };
        with_vm(|vm| {
            if obj.get_attr("metadata", vm).is_err() {
                return;
            }
            let args = (vm.ctx.new_int(CONF_SAMPLERATE), vm.ctx.new_int(samplerate));
            if let Err(e) = vm.call_method(obj, "metadata", args) {
                log::warn!("{}: metadata: {}", self.info.id, exc_text(vm, &e));
            }
        });
    }

    fn decode(&mut self, ctx: &mut dyn Context) -> Result<()> {
        self.run(ctx, |vm, obj| vm.call_method(obj, "decode", ()).map(drop))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        self.run(ctx, |vm, obj| {
            let data = item_to_py(item, vm);
            let args = (vm.ctx.new_int(ss), vm.ctx.new_int(es), data);
            vm.call_method(obj, "decode", args).map(drop)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{Item, interpreter, item_to_py, py_to_item};

    #[test]
    fn wide_integers_cross_into_python_and_back_intact() {
        // The interpreter's start needs more stack than a test thread has
        // in a debug build.
        let t = std::thread::Builder::new().stack_size(16 << 20).spawn(|| {
            interpreter().enter(|vm| {
                // 2^152 - 1, the largest 152 bit SPI word.
                let item = Item::BigUint(vec![0xff; 19]);
                let obj = item_to_py(&item, vm);
                assert_eq!(
                    obj.str(vm).unwrap().to_string(),
                    "5708990770823839524233143877797980545530986495"
                );
                let back = py_to_item(&obj, vm);
                assert!(matches!(back, Item::BigUint(ref b) if b == &vec![0xff; 19]));
                let small = item_to_py(&Item::Int(42), vm);
                assert!(matches!(py_to_item(&small, vm), Item::Int(42)));
                let t = item_to_py(&Item::Bool(true), vm);
                assert!(matches!(py_to_item(&t, vm), Item::Bool(true)));
            });
        });
        t.unwrap().join().unwrap();
    }
}
