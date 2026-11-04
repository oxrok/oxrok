// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Protocol decoding through the binary, compared with annotation
//! output captured from sigrok-cli 0.8.0-git with libsigrokdecode on
//! 2026-10-22. `uart-hi.vcd` is a synthetic 8N1 capture at 115200 baud,
//! 1 MHz, carrying "Hi!\n", 0x00 and 0xff.

use std::process::Command;

fn oxrok(args: &[&str]) -> (String, String, bool) {
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok")).args(args).output().expect("run oxrok");
    (
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
        out.status.success(),
    )
}

const VCD: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/uart-hi.vcd");
const I2C: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/i2c-write-read.vcd");
const SPI: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/spi-mode0.vcd");
const UART_ESP32: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/uart-esp32.sr");
const SPI_ESP32: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/spi-esp32.sr");
const I2C_ESP32: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/i2c-esp32.sr");

#[test]
fn uart_annotations_match_libsigrokdecode() {
    let (out, _, ok) = oxrok(&["-l", "4", "-i", VCD, "-P", "uart:baudrate=115200:rx=top.D0"]);
    assert!(ok);
    assert_eq!(out, include_str!("fixtures/uart-hi.ann"));
    // Default verbosity: only the longest text, no sample numbers.
    let (out, _, _) =
        oxrok(&["-i", VCD, "-P", "uart:baudrate=115200:rx=top.D0", "-A", "uart=rx-data"]);
    assert_eq!(out, "uart-1: 48\nuart-1: 69\nuart-1: 21\nuart-1: 0A\nuart-1: 00\nuart-1: FF\n");
    let (out, _, _) = oxrok(&["-i", VCD, "-P", "uart:format=ascii", "-A", "uart=rx-data"]);
    assert_eq!(out, include_str!("fixtures/uart-hi-ascii.ann"));
    let (out, _, _) = oxrok(&[
        "-i",
        VCD,
        "-P",
        "uart:rx=top.D0",
        "-A",
        "uart=rx-data",
        "--protocol-decoder-samplenum",
    ]);
    assert!(out.starts_with("29-99 uart-1: 48\n"), "{out}");
    let (out, _, _) =
        oxrok(&["-i", VCD, "-P", "uart", "-A", "uart=rx-start", "--protocol-decoder-ann-class"]);
    assert_eq!(out.lines().next(), Some("uart-1: rx-start: Start bit"));
}

/// A frame sets the decoder's idle start past its own end. Reaching an edge
/// before that sample must not underflow (a panic in a debug build).
#[test]
fn uart_idle_start_ahead_of_the_sample() {
    let dir = tempfile_dir();
    let cap = dir.join("cap.vcd");
    let cap = cap.to_str().unwrap();
    let (_, err, ok) = oxrok(&[
        "-d",
        "demo:logic_channels=2",
        "-g",
        "Logic",
        "-c",
        "pattern=walking-one",
        "--samples",
        "200",
        "-O",
        "vcd",
        "-o",
        cap,
    ]);
    assert!(ok, "{err}");
    let (_, err, ok) =
        oxrok(&["-i", cap, "-P", "uart:baudrate=115200:rx=D0", "-A", "uart=rx-data"]);
    assert!(ok, "{err}");
    assert!(!err.contains("panicked"), "{err}");
    std::fs::remove_dir_all(dir).unwrap();
}

fn tempfile_dir() -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("oxrok-decode-cli-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn uart_binary_output() {
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-i", VCD, "-P", "uart", "-B", "uart=rx"])
        .output()
        .unwrap();
    assert_eq!(out.stdout, b"Hi!\n\x00\xff");
}

#[test]
fn decoder_listing_and_errors() {
    let (out, _, _) = oxrok(&["-L"]);
    // Python decoders may sit between the native ones in the listing.
    assert!(out.contains(
        "Supported protocol decoders:
"
    ));
    assert!(out.contains(
        "  i2c                  Inter-Integrated Circuit
"
    ));
    assert!(out.contains(
        "  spi                  Serial Peripheral Interface
"
    ));
    assert!(out.contains(
        "  uart                 Universal Asynchronous Receiver/Transmitter
"
    ));
    let (out, _, ok) = oxrok(&["-P", "uart", "--show"]);
    assert!(ok);
    assert!(out.starts_with("ID: uart\nName: UART\n"));
    assert!(
        out.contains("- rx-data (RX): rx-data, rx-start, rx-parity-ok, rx-parity-err, rx-stop\n")
    );
    assert!(out.contains("- stop_bits: Stop bits (0.0, 0.5, 1.0, 1.5, default 1.0)\n"));
    let (_, err, ok) = oxrok(&["-i", VCD, "-P", "nosuch"]);
    assert!(!ok);
    assert!(err.contains("Protocol decoder 'nosuch' not found."), "{err}");
    let (_, err, ok) = oxrok(&["-i", VCD, "-P", "uart:parity=maybe"]);
    assert!(!ok);
    assert!(err.contains("Invalid value 'maybe'"), "{err}");
    let (_, err, ok) = oxrok(&["-i", VCD, "-P", "uart", "-A", "uart=nosuch"]);
    assert!(!ok);
    assert!(err.contains("Annotation class 'nosuch' not found"), "{err}");
}

#[test]
fn i2c_annotations_match_libsigrokdecode() {
    // Channels default to scl = first, sda = second, as in libsigrokdecode.
    let (out, _, ok) = oxrok(&["-l", "4", "-i", I2C, "-P", "i2c"]);
    assert!(ok);
    assert_eq!(out, include_str!("fixtures/i2c-write-read.ann"));
    let (out, _, _) =
        oxrok(&["-i", I2C, "-P", "i2c", "-A", "i2c=address-write:data-write:data-read"]);
    assert_eq!(
        out,
        "i2c-1: Write\ni2c-1: Address write: 51\ni2c-1: Data write: 55\ni2c-1: Data write: AA\ni2c-1: Data read: 0F\n"
    );
    let (out, _, _) = oxrok(&["-i", I2C, "-P", "i2c", "-M", "i2c"]);
    assert_eq!(out, "i2c-1: Bitrate: 88028\ni2c-1: Bitrate: 87628\n");
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-i", I2C, "-P", "i2c", "-B", "i2c=data-write"])
        .output()
        .unwrap();
    assert_eq!(out.stdout, [0x55, 0xaa]);
}

#[test]
fn spi_annotations_match_libsigrokdecode() {
    let (out, _, ok) = oxrok(&["-l", "4", "-i", SPI, "-P", "spi"]);
    assert!(ok);
    assert_eq!(out, include_str!("fixtures/spi-mode0.ann"));
    let (out, _, _) = oxrok(&["-i", SPI, "-P", "spi", "-A", "spi=mosi-transfer:miso-transfer"]);
    assert_eq!(out, "spi-1: 3C FF\nspi-1: A5 01\nspi-1: 7E\nspi-1: 80\n");
    let (out, _, _) =
        oxrok(&["-i", SPI, "-P", "spi:bitorder=lsb-first:wordsize=4", "-A", "spi=mosi-data"]);
    assert_eq!(out.lines().next(), Some("spi-1: 05"));
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-i", SPI, "-P", "spi", "-B", "spi=miso"])
        .output()
        .unwrap();
    assert_eq!(out.stdout, [0x3c, 0xff, 0x7e]);
}

/// Captures taken with the `LogicPort` from the ESP32 generator on the
/// bench, 2026-10-22: its `PONG` reply at 921600 baud, an I2C write and
/// read to address 0x50 played by its sequencer, and an SPI transfer of
/// `A5 01 3C` with `FF` on MISO. The goldens are sigrok-cli's output.
#[test]
fn real_captures_match_libsigrokdecode() {
    let (out, _, ok) = oxrok(&["-l", "4", "-i", UART_ESP32, "-P", "uart:baudrate=921600:rx=D21"]);
    assert!(ok);
    assert_eq!(out, include_str!("fixtures/uart-esp32.ann"));
    let (out, _, _) = oxrok(&[
        "-i",
        UART_ESP32,
        "-P",
        "uart:baudrate=921600:rx=D21:format=ascii",
        "-A",
        "uart=rx-data",
    ]);
    let text: String = out.lines().map(|l| l.trim_start_matches("uart-1: ")).collect();
    assert_eq!(text, "PONG[0D][0A]");
    let (out, _, ok) = oxrok(&["-l", "4", "-i", I2C_ESP32, "-P", "i2c:scl=D11:sda=D12"]);
    assert!(ok);
    assert_eq!(out, include_str!("fixtures/i2c-esp32.ann"));
    let (out, _, _) = oxrok(&[
        "-i",
        I2C_ESP32,
        "-P",
        "i2c:scl=D11:sda=D12",
        "-A",
        "i2c=address-write:address-read:data-write:data-read",
    ]);
    let text = out.lines().map(|l| l.trim_start_matches("i2c-1: ")).collect::<Vec<_>>().join(", ");
    assert_eq!(
        text,
        "Write, Address write: 50, Data write: 0A, Data write: 55, Data write: AA, \
         Read, Address read: 50, Data read: 3C"
    );
    let (out, _, ok) =
        oxrok(&["-l", "4", "-i", SPI_ESP32, "-P", "spi:clk=D28:miso=D26:mosi=D5:cs=D25"]);
    assert!(ok);
    assert_eq!(out, include_str!("fixtures/spi-esp32.ann"));
    let (out, _, _) = oxrok(&[
        "-i",
        SPI_ESP32,
        "-P",
        "spi:clk=D28:miso=D26:mosi=D5:cs=D25",
        "-A",
        "spi=mosi-transfer",
    ]);
    assert_eq!(out, "spi-1: A5 01 3C\n");
}

/// The Python host, with the python feature and a libsigrokdecode
/// installation. Without the decoders the tests pass vacuously and say
/// so.
#[cfg(any(feature = "python", feature = "rustpython"))]
mod python {
    use super::*;

    fn available() -> bool {
        let (out, _, _) = oxrok(&["-L"]);
        if out.contains("\n  timing ") {
            return true;
        }
        eprintln!("no libsigrokdecode decoders found, skipping");
        false
    }

    #[test]
    fn timing_decoder_runs_unchanged() {
        if !available() {
            return;
        }
        let (out, _, ok) = oxrok(&["-l", "4", "-i", VCD, "-P", "timing"]);
        assert!(ok);
        assert_eq!(out, include_str!("fixtures/uart-hi-timing.ann"));
    }

    #[test]
    fn native_to_python_chain_keeps_order() {
        if !available() {
            return;
        }
        let (out, _, ok) = oxrok(&["-i", I2C, "-P", "i2c,eeprom24xx"]);
        assert!(ok);
        let lines: Vec<&str> = out.lines().collect();
        assert_eq!(lines[0], "i2c-1: Start");
        assert_eq!(lines[1], "eeprom24xx-1: Control code bits: 1010");
        assert!(lines.iter().any(|l| l.starts_with("eeprom24xx-1: Data byte")), "{out}");
        // A decoder exception on one item does not stop the chain.
        let (out, err, ok) = oxrok(&["-i", I2C, "-P", "i2c,rtc8564"]);
        assert!(ok);
        assert!(err.contains("handle_reg_0x55"), "{err}");
        assert!(out.contains("rtc8564-1: Write date/time"), "{out}");
    }

    #[test]
    fn python_decoder_show_and_options() {
        if !available() {
            return;
        }
        let (out, _, ok) = oxrok(&["-P", "eeprom24xx", "--show"]);
        assert!(ok);
        assert!(out.starts_with("ID: eeprom24xx\n"));
        assert!(out.contains("- chip: Chip ("), "{out}");
        let (_, err, ok) = oxrok(&["-i", I2C, "-P", "i2c,eeprom24xx:chip=nosuch"]);
        assert!(!ok);
        assert!(err.contains("Invalid value 'nosuch'"), "{err}");
    }
}
