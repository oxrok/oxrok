//! A session file of any bytes: opening and reading it returns packets
//! or an error, and does not panic.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxrok::formats::sr::SessionReader;
use std::io::Cursor;

fuzz_target!(|data: &[u8]| {
    let Ok(mut reader) = SessionReader::new(Cursor::new(data)) else { return };
    let _ = reader.unit_size();
    let _ = reader.read(&mut |_| {});
});
