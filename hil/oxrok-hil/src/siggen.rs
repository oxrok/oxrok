// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Client for the `hil/esp32-siggen` firmware: a line protocol at 921600
//! baud. Mask bits with the default map: 0 = D0 (GPIO25), 1 = D1
//! (GPIO26), 2 = D2 (GPIO27), 3 = CLK1 (GPIO14).

use std::io::{BufRead, BufReader, Write};
use std::time::{Duration, Instant};

/// The signal generator.
pub struct SigGen {
    port: BufReader<Box<dyn serialport::SerialPort>>,
}

impl SigGen {
    /// Opens the ESP32 on `path`, or the first USB serial port found
    /// (`SIGGEN_PORT` overrides).
    pub fn open(path: Option<&str>) -> Result<SigGen, String> {
        let path = match path.map(str::to_owned).or_else(|| std::env::var("SIGGEN_PORT").ok()) {
            Some(p) => p,
            None => find_port().ok_or("ESP32 serial port not found, set SIGGEN_PORT")?,
        };
        let mut port = serialport::new(&path, 921_600)
            .timeout(Duration::from_secs(2))
            .open()
            .map_err(|e| format!("{path}: {e}"))?;
        // Do not hold the chip in reset or boot mode.
        let _ = port.write_data_terminal_ready(false);
        let _ = port.write_request_to_send(false);
        // Opening may still reset the ESP32; wait for it to boot.
        std::thread::sleep(Duration::from_millis(1500));
        let _ = port.clear(serialport::ClearBuffer::Input);
        let mut g = SigGen { port: BufReader::new(port) };
        let pong = g.cmd("PING", Duration::from_secs(5))?;
        if pong != "PONG" {
            return Err(format!("siggen not responding on {path}: {pong:?}"));
        }
        Ok(g)
    }

    /// Sends one command line and returns the reply line.
    pub fn cmd(&mut self, line: &str, timeout: Duration) -> Result<String, String> {
        let port = self.port.get_mut();
        port.write_all(format!("{line}\n").as_bytes()).map_err(|e| e.to_string())?;
        let start = Instant::now();
        while start.elapsed() < timeout {
            let mut reply = String::new();
            match self.port.read_line(&mut reply) {
                Ok(0) => {}
                Ok(_) => {
                    let reply = reply.trim();
                    if !reply.is_empty() && !reply.starts_with("SIGGEN") {
                        return Ok(reply.to_owned());
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e.to_string()),
            }
        }
        Err(format!("timeout waiting for the reply to {line}"))
    }

    /// Sets the output levels.
    pub fn set(&mut self, mask: u32) -> Result<(), String> {
        self.cmd(&format!("SET {mask}"), Duration::from_secs(5)).map(|_| ())
    }

    /// A continuous square wave on CLK1, 0 stops it.
    pub fn clk(&mut self, hz: u32) -> Result<(), String> {
        self.cmd(&format!("CLK {hz} 50"), Duration::from_secs(5)).map(|_| ())
    }

    /// Plays `steps` (mask, nanoseconds) `repeat` times.
    pub fn seq(&mut self, steps: &[(u32, u32)], repeat: u32) -> Result<(), String> {
        use std::fmt::Write as _;
        let mut line = format!("SEQ {repeat}");
        for (m, ns) in steps {
            let _ = write!(line, " {m}:{ns}");
        }
        self.cmd(&line, Duration::from_mins(1)).map(|_| ())
    }

    /// Maps mask bit i to GPIO `gpios[i]`; empty restores the default.
    pub fn map(&mut self, gpios: &[u32]) -> Result<(), String> {
        use std::fmt::Write as _;
        let mut line = String::from("MAP");
        for g in gpios {
            let _ = write!(line, " {g}");
        }
        self.cmd(&line, Duration::from_secs(5)).map(|_| ())
    }
}

fn find_port() -> Option<String> {
    let dir = "/dev";
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned()))
        .filter(|n| {
            (n.starts_with("cu.")
                && ["usbserial", "SLAB", "wchusb", "usbmodem"].iter().any(|k| n.contains(k)))
                || n.starts_with("ttyUSB")
                || n.starts_with("ttyACM")
        })
        .collect();
    // USB to UART bridges first: a CDC port (usbmodem, ttyACM) is more
    // likely another board, such as the HID cable emulator.
    let rank = |n: &str| usize::from(n.contains("usbmodem") || n.starts_with("ttyACM"));
    names.retain(|n| !n.contains("HIDCABLE"));
    names.sort_by(|a, b| rank(a).cmp(&rank(b)).then_with(|| a.cmp(b)));
    names.first().map(|n| format!("{dir}/{n}"))
}
