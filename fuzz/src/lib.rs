//! What the fuzz targets share.

use oxrok::formats::input::{self, Options};

/// Feeds `data` to the input module `id` with `options` (its defaults
/// for the rest), in chunks of a size the data chooses, then ends the
/// file. An error is fine; the module must not panic.
///
/// # Panics
///
/// When there is no input module `id`.
pub fn feed_input(id: &str, options: &[(&str, &str)], data: &[u8]) {
    let module = input::find(id).expect("an input module of this id");
    let options: Options = options.iter().map(|&(k, v)| (k.to_owned(), v.to_owned())).collect();
    let Ok(mut running) = module.instantiate(&options) else { return };
    let _ = (module.detect)(&data[..data.len().min(4096)], None);
    // 1 to 251 bytes at a time: a file read in pieces is cut anywhere,
    // inside a token or a sample.
    let hash = data.iter().fold(data.len(), |a, &b| a.wrapping_mul(31).wrapping_add(b.into()));
    let mut out = Vec::new();
    for chunk in data.chunks(1 + hash % 251) {
        if running.feed(chunk, &mut out).is_err() {
            return;
        }
        out.clear();
    }
    let _ = running.end(&mut out);
    let _ = running.channels();
}
