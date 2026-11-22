//! A VCD file of any bytes: `-I vcd` returns packets or an error, and
//! does not panic.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxrok_fuzz::feed_input;

fuzz_target!(|data: &[u8]| feed_input("vcd", data));
