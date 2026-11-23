//! Any bytes as what a meter sends: the packet framing and parsing of
//! every serial multimeter protocol (`serial-dmm`'s model table, the HID
//! cable twins included) and of the LCR meter chips.
//!
//! The first byte chooses the protocol, the rest is the line. The bytes
//! are stepped over as the drivers do it: one at a time until a packet
//! frames, which is then parsed.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxrok::dmm::{Protocol, Validity};
use oxrok::drivers::serial_dmm::models::{MODELS, hid_models};
use oxrok::lcr::Chip;

const CHIPS: [Chip; 2] = [Chip::Es51919, Chip::Vc4080];

/// `DmmDevice`'s receive loop over a whole buffer, with no port.
fn dmm(p: &Protocol, buf: &[u8]) {
    let _ = p.find_packet(buf);
    let mut pos = 0;
    while pos < buf.len() && buf.len() - pos >= p.packet_size {
        match p.frame_at(&buf[pos..]) {
            Validity::Need => break,
            Validity::Invalid => pos += 1,
            Validity::Valid(len) => {
                // A framing function that claims nothing, or more than
                // there is, would hang or overrun the driver's loop.
                assert!(len > 0 && len <= buf.len() - pos, "a frame of {len} bytes at {pos}");
                let _ = (p.parse)(&buf[pos..pos + len]);
                pos += len;
            }
        }
    }
}

/// `serial-lcr`'s: a packet is `packet_size` bytes that are valid.
fn lcr(chip: Chip, buf: &[u8]) {
    for pkt in buf.windows(chip.packet_size()) {
        if chip.valid(pkt) {
            let _ = chip.settings(pkt);
            for channel in 0..chip.channel_count() {
                let _ = chip.reading(pkt, channel);
            }
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let [which, line @ ..] = data else { return };
    let hid = hid_models();
    let n = usize::from(*which) % (MODELS.len() + hid.len() + CHIPS.len());
    if let Some(model) = MODELS.iter().chain(hid).nth(n) {
        dmm(&model.protocol, line);
    } else {
        lcr(CHIPS[n - MODELS.len() - hid.len()], line);
    }
});
