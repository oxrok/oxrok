//! A VCD file of any bytes: `-I vcd` returns packets or an error, and
//! does not panic.
//!
//! With `compress` and `numchannels` set. A VCD file says how long
//! nothing happens and how wide a signal is in a few digits, and the
//! module holds the samples of one piece of the file in memory: without
//! the two, the first thing libFuzzer finds is a far timestamp (`#1e10`
//! is that many samples), an out of memory report that hides the rest.
//! That a file can ask for that much is known, and is not what this
//! target is for.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxrok_fuzz::feed_input;

fuzz_target!(|data: &[u8]| {
    feed_input("vcd", &[("compress", "4096"), ("numchannels", "64")], data);
});
