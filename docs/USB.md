# USB transport

Drivers talk to USB through the `Transport` trait of the `usb` module:
control and bulk transfers with timeouts, interface claims, resets and
the packet size of an endpoint. Two backends implement it, selected by
cargo feature. The drivers never see which one is in use.

| Feature | Crate | Platforms | Notes |
|---|---|---|---|
| `nusb` (default) | nusb 0.2 | Linux, macOS, Windows, Android | pure Rust, no system library |
| `libusb` | rusb 0.9 | the above plus the BSDs, Solaris, Haiku | builds libusb from source |

With both compiled in, `OXROK_USB_BACKEND=libusb` or `=nusb` picks one
at run time; nusb is the default.

`cargo run --example usb_list` prints what the backend sees.

## Verified

2026-10-22, macOS 26 on an Intronix LogicPort LA1034 (FT245BL,
`0403:dc48`): both backends open the device, claim interface 0, run the
FTDI vendor requests (reset, latency timer, baud rate, bit-bang mode,
pin readback, modem status, EEPROM read) and read the FIFO. No kernel
driver claimed the device; Apple ships no FTDI driver since Monterey
and the LogicPort's product id is not one a third party FTDI driver
matches by default.

## Streams

`Transport::bulk_in_stream` opens a queue of IN transfers for devices
that stream faster than one blocking read after another can drain
them: the nusb backend keeps `depth` transfers submitted in the kernel,
the libusb backend runs a thread reading back to back. A recording
logs every completed transfer as a `bulk_in` of the transfer size, and
a replay answers a stream from those steps.

## Record and replay

`record::Recorder` wraps a transport and logs every call with its
result. `record::Replay` answers from such a log and fails on the first
call that differs from the recording: a different request, different
data, or a call past the end. Every driver ships at least one recorded
session so its logic runs in CI without hardware. A `claim` or
`release` floats among the control transfers around it, up to the next
bulk or interrupt step, so a recording stays valid when a driver moves
its claim past a control transfer; `err disconnected` records a device
that went away (`Error::Disconnected`).

A fixture is a text file, one call per line, hand editable:

```text
# oxrok usb fixture 1
device vid=0403 pid=dc48 version=0400 bus=0 addr=1 product="LogicPort Logic Analyzer"
claim 0 -> ok
mps 81 -> 64
ctl_out 40 09 0004 0000 - -> ok
ctl_in 40 0a 0000 0000 1 -> 04
bulk_out 02 00{4096} -> 4096
bulk_in 81 4096 -> 3160aabb
bulk_in 81 4096 -> err timeout
```

- `device`: what enumeration reported; strings are quoted with `\"`
  and `\\` escapes.
- `config N`, `claim N`, `release N`, `reset`, `mps EP`: result `ok`, or
  the packet size for `mps`.
- `ctl_out TYPE REQ VALUE INDEX DATA`, `ctl_in TYPE REQ VALUE INDEX LEN`:
  the setup fields in hex, `TYPE` without the direction bit. Results
  are `ok` and the data read.
- `bulk_out EP DATA`, `bulk_in EP LEN`: results are the byte count and
  the data read.
- Data is hex pairs; `-` is empty and `xx{N}` repeats a byte, so a zero
  flood stays one short token. Data sent to the device that is longer
  than 1 KiB is recorded as `@LEN:SHA256` and compared by digest, so a
  1 MB bitstream is one line.
- Errors are `err KIND ["message"]` with kind `timeout`, `busy`,
  `device`, `io` or `unsupported`.

Timeouts are not recorded; a replay answers at once. Polling loops run
on time, so a replay may poll more or fewer times than the recording:
a read identical to the one just answered is answered again, and
leftover identical reads in the recording are skipped.

`OXROK_USB_RECORD=FILE` makes `usb::open` record every session and
write it to `FILE` when the device is closed; a later session of the
same process (a scan that opened the device, then the acquisition) is
appended after it, each starting with its own `# oxrok usb fixture 1`
line. This works with any program that uses the drivers:

```sh
OXROK_USB_RECORD=capture.txt oxrok -d intronix-logicport -c samplerate=10M --samples 64 -O bits
```

`OXROK_USB_REPLAY=FILE` is the other way round: the USB listing shows
only the device of the file's first session (no hardware is asked),
and each open of it is served the file's next session as a `Replay`.
A session not played to its end, or one no open asked for, makes the
command line tool exit with an error. So a recording runs again on any
system with the same command line:

```sh
OXROK_USB_REPLAY=capture.txt oxrok -d intronix-logicport -c samplerate=10M --samples 64 -O bits
```

The `device` line also takes `strings=M,P,S`, the string descriptor
indexes, recorded when the driver asked for them (the OS answers from
its cache, so they are no step) and answered from there on replay;
fx2lafw's scan needs them to read the strings.

`OXROK_HID_RECORD=FILE` and `OXROK_HID_REPLAY=FILE` do the same for HID
devices (`hid::record`): `# oxrok hid fixture 1`, a `device vid= pid=
manufacturer= product= serial= path=` line, then `read LEN -> DATA`
(`-` for a read that timed out), `write DATA -> ok`,
`get_feature ID LEN -> DATA` and `set_feature DATA -> ok`, errors as in
USB fixtures. The `path` is the node the device had (`/dev/hidrawN`);
a replayed listing reports it, and a `conn=` that names it matches it
as text, so the fixture also runs on macOS and Windows. With either
replay variable set, both listings show only replayed devices. A serial
over HID cable records its HID reports here, which covers the chip
adapter too (`OXROK_SERIAL_RECORD` keeps the decoded bytes, but there
is no serial replay variable). Timed out reads depend on the pace of
the run, so on replay they float: one recorded where the replay makes
another call is skipped, and a read where the recording has no read
next (or nothing more) answers a timeout. Data reads, writes and
feature reports must come as recorded. As on a real device, extra
reads that come back empty are not caught: a driver that polls more
often than recorded passes.

Replay fixtures of drivers with an emulator model are recorded from
the model, not written by hand: `emu_case --record` (see "Scenarios")
keeps oxrok's sessions and stdout as `tests/fixtures/DRIVER/emu-SLUG.txt`
and `.out`, and `tests/emu_replay.rs` runs the oxrok binary on every
such file with the recorded arguments and the replay variable set.

To record, wrap the transport before handing it to the driver:

```rust
let (recorder, handle) = Recorder::with_handle(oxrok::usb::open(&info)?);
let ftdi = Ftdi::open(Box::new(recorder), Port::A)?;
// ... drive it ...
std::fs::write("session.txt", handle.fixture().to_text())?;
```


## FTDI

The `ftdi` module speaks the FT232/FT245 vendor protocol over a transport:
reset and purge, latency timer, baud rate (which also clocks the
bit-bang modes), line settings, modem control and status, the bit modes,
pin readback and EEPROM reads. Bulk reads strip the two status bytes
that begin every USB packet. The EEPROM write request is not
implemented and will not be: a wrong write bricks the device.

The baud rate encoding follows FTDI's application note AN_120: a 3 MHz
clock (12 MHz on the H chips above 1.2 MBaud), divisors 1 and 1.5 with
their own codes, then 14 integer bits and a 3 bit code for the eighths.

## Emulation over USB/IP

On Linux a device can be a program: the `vhci-hcd` module turns a
USB/IP server into a USB device that libusb, nusb, hidraw and the
kernel's class drivers see as real. `oxrok-emu`, the `emu` crate of
the workspace (`cargo build -p oxrok-emu`), is that server, on the
[`vusb`](https://crates.io/crates/vusb) crate, which holds the
contract, the bus, the shells, the USB/IP server, the attach path and
the control socket; [its documentation](https://docs.rs/vusb) and its
repository's `docs/design.md` and `docs/control.md` describe them. `emu/`
holds what is oxrok's own: the device models, the rules each gives the
transaction log (the log itself is vusb's `observer::fixture::Log`, and
`emu/src/txlog.rs` wires it in), `shells/hidcable.rs` and the
checks on a firmware upload in `shells/ezusb.rs`, which know sigrok's
fixtures, and the `oxrok-emu` commands. It re-exports vusb's paths, so a
model says `crate::model` or `crate::shells::cdc` whichever crate holds
them. It is how a driver is verified against sigrok-cli without the
hardware (`docs/ROADMAP.md`, "Emulation plan"). `oxrok-emu` also builds
on Windows; attaching a device is Linux only.

- A device is a `DeviceModel` (`vusb::model`): its descriptors,
  `control(setup, data)`, `out(ep, data)`, `in_transfer(ep, max)`
  answering data, NAK or STALL, and `reset`. Plain Rust with no I/O,
  so a model runs in a unit test, on this server and, later, on a
  Pico. `Device` (`vusb::device`) wraps it with the standard requests
  every device answers (descriptors, configuration, status, halts) and
  the checks on every transfer.
- Transport shells, in `vusb::shells`, carry what every device of a
  kind shares, and a model fills in what its own protocol means.
  `shells::hid` is the first (`HidInterface::with_version` sets a
  `bcdHID` other than the default 1.11): the HID class descriptor, the
  report descriptor fetch, `GET_REPORT`, `SET_REPORT`, idle and protocol, with
  the model answering the reports through `HidReports`.
  `emu/src/shells/hidcable.rs` is what the serial over HID cable models
  share: the Pico emulator's descriptors, the packets they play, a stream
  paced by polls of the held interrupt transfer rather than wall time.
- `shells::ftdi` is the FTDI shell. Per chip (AM, BM, FT2232C, R,
  FT2232H, FT4232H, FT232H, FT-X): the bcdDevice libftdi and ftdi_sio
  take the type from (0x0200 for a BM without a serial, as on the
  chip), speed, ports with bulk IN 0x81+2n and OUT 0x02+2n of 64 or 512
  bytes, and the bit modes of each port; an AM has no latency timer,
  bit modes or pins. Vendor requests take the port in the low byte of
  wIndex (1 for A; a single-port chip also takes 0, which the kernel
  sends). SET_BAUD_RATE is decoded per AN_120 and checked per chip
  (`libftdi_divisor` is a port of libftdi's); libftdi 1.5 puts the port
  in wIndex on the H chips only, and on the FT2232C the chip reads
  libftdi's bare divisor bits as the kernel's port form, which is
  documented, not flagged. EEPROM reads come from an image with its
  checksum; libftdi reads 128 words, and a 64-word EEPROM wraps. Every
  packet of an IN transfer starts with the two status bytes, and an
  idle chip answers them alone. A model implements `FtdiPorts`: notice
  of changes (`Event`), host writes, the bytes for the host
  (`Fill::Data`, `Idle`, or `Wait` to NAK) and `READ_PINS`. Time in
  the shell is counted in the host's transfers, never the clock:
  `Countdown` ("busy for N polls after an event") is what a model uses
  for anything its driver polls. Every FTDI model takes the shell's
  fault options, all off by default: `stall=REQUEST[/WVALUE]` STALLs a
  vendor request (named, `reset`, `baud`, `bitmode`, `latency`,
  `read-eeprom`, ..., or a number; repeatable; a STALLed request has
  no effect), `in-stall=N` halts each port's bulk IN after N completed
  transfers until a bus reset, and `idle=N` sends only the two status
  bytes for the first N bulk IN transfers after each bit mode change,
  which libftdi turns into reads of 0 bytes.
  What the first FTDI model taught: the kernel's ftdi_sio binds every
  port at attach, reads and sets the latency timer of each, and reads
  the EEPROM of R, FT232H and FT-X chips for its CBUS GPIOs; libftdi
  detaches it from the interface it claims and does not reattach it.
- `shells::ezusb` is the FX2 without firmware: its built-in
  descriptors, the boot loader's vendor request `0xa0` (RAM `0x0000`
  up to 8 or 16 KiB and `0xe000` to `0xe1ff`, CPUCS at `0xe600`), and
  `check_ezusb_c` (`emu/src/shells/ezusb.rs`), which holds an upload
  against libsigrok's `ezusb.c` byte for byte. When the 8051 is released
  the model asks to renumerate (`DeviceModel::take_renumeration`). The log gets
  `disconnect` and `connect VID:PID` at once in the running section,
  and what the host sends until the detach follows there and is
  compared. The main loop then detaches the device, imports it again
  and re-attaches it on the same port, since a driver finds a
  renumerated device by its port path. From the detach on the log is
  in a `renumeration` section; the run's section resumes once the
  kernel has configured the device, then `vhci::open_device` runs and
  a `RENUMERATED port=... usb=...` line on standard output reports the
  new nodes. So the
  kernel's enumeration never lands in a tool's transactions. libusb
  can open a renumerated device before its node is world accessible;
  the C driver logs `LIBUSB_ERROR_ACCESS` and tries again, so only its
  stderr changes.
- A `reset` whose fresh model has other descriptors than the attached
  device (it renumerated, or the options changed) re-attaches the same
  way, from a `reset` section. `vusb-ctl options KEY=VALUE...` sets
  the options of the resets that follow (`options` alone restores the
  command line's), which is how one scenario covers several boards or
  chips. vusb-ctl's words read a value like `1e3` as a number and an
  object holds a key once, so `emu_cmd` sends them as `--args` JSON,
  `{"pairs":[["KEY","VALUE"],...]}`.
- A model sees each IN submission (`DeviceModel::in_submitted`), and
  transfers on one endpoint complete in order. A streaming model can
  thus complete one transfer per submission, which real devices do
  not do, so that the number of transfers a capture completes does not
  depend on timing. A model can ask for data over a size to be logged
  as `@LEN:SHA256`, the fixtures' digest form (a `log_rules` function
  in its module; fx2lafw does, for its firmware
  chunks and transfers of up to 240 KiB); by default every byte is
  logged. What fx2lafw taught: a driver that stops after a transfer
  must not have resubmitted it first, or one more completes; libsigrok
  counts any failed bulk IN transfer, not only an empty one, toward
  `MAX_EMPTY_TRANSFERS`; a model's faults are paced like its data (one
  per submission), so a run's transactions stay the driver's own; and
  a claim held by another program can be staged without a transfer by
  a usbfs helper that claims through `/sys/bus/usb/devices/$EMU_BUSID`
  between cases whose descriptors do not change.
- Mid-run events. `vusb-ctl arm event=unplug when=WHEN` or `arm
  event=mark name=interrupt when=WHEN` arms an unplug or an interrupt
  for the run that follows (a `reset` disarms it): `WHEN`
  is `in=EP:N` (the Nth IN transfer on endpoint EP completed; a STALL
  counts, a NAK does not; endpoint 0 counts control reads),
  `out=EP:N`, or `model`, the point the model marks through
  `DeviceModel::mark`. When it fires the log gets an
  `unplug` or `interrupt` line right after the transfer that fired it,
  and `EVENT unplug|interrupt` is printed. An unplug drops whatever the
  host submits from then on, detaches the port and keeps the device
  off the bus until `replug` or the next `reset`, which attach it again
  on the same port when free, its enumeration in a `replug` section,
  and print `REPLUGGED port=...` with the new nodes. An interrupt is
  the harness's to deliver; how many transfers the tool makes before
  the signal lands depends on timing, so an interrupt case waits on
  NAKed polls, not on data.
- Faults that make a model misbehave as firmware can (STALL a request,
  a short or empty packet, a silent meter, an unanswered request) are
  model options, off by default, so the goldens of the plain cases do
  not change. A model's one-shot STALL (`stall-in=N` and the like) is
  an emulation artifact: a real STALL halts the endpoint until
  CLEAR_FEATURE. What the emulator cannot show: a STALLed interrupt IN
  never reaches a hidraw reader, since usbhid clears the halt and
  resubmits, and a read error on hidraw is only an unplug, which cannot
  be timed against libsigrok's reads, so `serial_hid*.c` read errors
  stay unreached.
- A model can name the bytes of an OUT transfer the host leaves
  undefined (`Rules::undefined`, from its `log_rules`): the log prints them as
  `xx`, and in a digest they count as 0 and their ranges follow
  (`@LEN:SHA256/xx=A-B`). The DSLogic driver sends an uninitialised
  struct field that would otherwise make the log differ between runs.
- A model can fail a standard request that `Device` answers itself
  (`DeviceModel::fail_standard`): it STALLs it, with no effect, or
  cuts the answer short. The kernel's enumeration goes through the
  same hook, so such a fault is one whose options leave the
  descriptors unchanged (no re-attach). fx2lafw's `stall_string=I` and
  `short_string=I` fail `scan()`'s string reads this way.
- What the FTDI family taught (batch 3, 2026-11-03): libftdi reads in
  16 KiB transfers on Linux and ends a read early only on a transfer
  of the two status bytes alone, so a model shows a short read as a
  short transfer followed by an empty one; drivers that end captures
  on wall time (asix-sigma checks a time limit every 10 ms) need cases
  where the limit has surely run out at the first poll, or none at
  all; drivers that always read the whole memory (chronovu-la,
  asix-sigma) make large `.tx` goldens; and `options` values cannot
  contain spaces (a `product=` with a space hung the session).
- What the EZ-USB family taught (batch 2, 2026-11-03): a driver that
  submits its whole batch of transfers after the start would see every
  one complete under a plain one-per-submission rule, so a streaming
  model completes one per submission made after the previous
  completion; a renumerated device's scan line carries its new
  address, so scan cases filter it; the C drivers' `LIBUSB_ERROR_ACCESS`
  retry after a renumeration shows in stderr at random, so a stderr
  filter matches the line it wants, not line 1; a driver that retries
  capture errors for ever (hantek-dso, hantek-6xxx after its flush)
  have their unplug, empty-transfer and never-firing-trigger cases as
  `--hangs` cases; and fault cases run on a board that starts
  with its firmware (`flashed=1` or the like), since after an upload
  the drivers retry a failing open for 3 s of wall time.
- USBTMC comes next.
- Models live in `emu/src/models`, one file `NAME.rs` or one directory
  `NAME/mod.rs` each, and nothing else names them: `emu/build.rs` finds
  them and generates the module declarations, the table
  `oxrok-emu list` prints and the dispatch `serve NAME` uses. A model
  module exports `pub const NAME: &str` (its file name; a `-` in it
  becomes `_` in the module identifier), `pub const ABOUT: &str` (one
  line: the device and its options) and
  `pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>>`,
  which parses the `-o KEY=VALUE` options and rejects unknown ones. A
  model whose data is not all logged byte by byte also exports
  `pub fn log_rules(options: &[(String, String)]) -> crate::txlog::Rules`, at
  the start of a line, which is how `build.rs` finds it. A unit test checks that each `NAME` is its file name and that each
  model builds with no options.
- `sudo oxrok-emu serve MODEL --attach --open-devices --log FILE`
  listens on TCP (`--listen`, port 0 picks a free one), imports the
  device over its own loopback connection, and writes
  `port sockfd devid speed` to `/sys/devices/platform/vhci_hcd.0/attach`,
  as `usbip attach` would; no usbip tool is involved, but root is.
  Once the kernel has configured the device it prints
  `READY port=N busid=B usb=BUS.ADDR usbfs=/dev/bus/usb/BBB/DDD nodes=/dev/...`:
  `usb` is the `bus.address` pair libusb drivers take in `conn=`,
  `usbfs` the raw node libusb and libftdi open, `nodes` those of the
  kernel's class drivers (`/dev/hidrawN`, `/dev/ttyACMN`, `/dev/usbtmcN`).
  `--open-devices` makes all of them world accessible so the tools
  under test run as a user. udev applies its own mode to a new node
  some time after the kernel creates it, so the server waits for
  `udevadm settle`, changes the modes, and checks them again after a
  second settle; `vhci::open_device` does all of it and is what a
  device that reappears under a new address (EZ-USB renumeration)
  calls again. With `--control PATH`, `vusb-ctl -s PATH` drives the run
  (`cargo install vusb-cli --version 0.2.1 --locked`; vusb's
  `docs/control.md` has the commands), and exits 0 once the server has
  taken the command and 1 if it refused: `section name=NAME` marks the
  log, `reset` puts a fresh model behind the attached device (its
  initial state, for a second run; the kernel keeps seeing the device
  it enumerated), `quit` ends the run, as do a termination signal and,
  with `--while-alive PID`, the end of that process (the harness passes
  its own), and
  `sync` does nothing but answer, after every input taken before it, a
  renumeration a run asked for included. Whatever `READY`, `RENUMERATED`,
  `REPLUGGED` or `EVENT` line that input caused is on standard output
  before the answer. `tools/diffcli.sh` finds vusb-ctl as `VUSB_CTL`,
  `target/debug/vusb-ctl` or on `PATH`. The exit status is 2 when
  a violation was logged. Without `--attach` it is a plain USB/IP
  server a `usbip` client can import `1-1` from. A device that never
  configures is detached before the server exits with an error.
- `sudo oxrok-emu cleanup` detaches the ports a run left attached when
  its server hung: those whose `devid` in the controller's `status` is
  an oxrok-emu one (bus `0x6f78`, "ox", in the upper half), or with
  `--all` every attached port. A server that dies outright needs
  nothing, as the controller frees the port when the connection
  closes. Each server announces a device number (the lower half) no
  attached oxrok-emu port has, taken from its process id, and detaches
  its port on the way out only while the port still shows its own
  `devid`. `detach` takes a bare port number, and `free_port` hands
  the lowest free one to the next server, so without this a server
  whose device a `cleanup` had removed would, when it ended, remove
  the next server's device from the same port. Such a server says that
  its device was detached by someone else and exits with status 1.
  A running session's port carries the same `devid`, so `cleanup`
  takes the session lock (`--lock PATH`, default `/tmp/oxrok-emu.lock`)
  and refuses while another session holds it. A cleanup inside a
  session (`diffcli.sh`'s, or a script run under `flock`) passes
  `--locked`; `--force` detaches regardless. An ad hoc cleanup run
  outside the lock used to kill sessions midway.
- The transaction log is vusb's `observer::fixture::Log` (vusb's
  `docs/log.md`), the fixture dialect above; `emu/src/txlog.rs` gives it
  the rules of the model each time the model is built, and in verbose
  mode says on standard error what it writes as comments. One line per
  completed transfer: `ctl_in 21 01 0300 0000 8 -> 4142434445000000`,
  `ctl_out 21 09 0300 0000 ff02000000000000 -> ok`, `int_in`,
  `bulk_out`, `-> err stall`. A transfer the model NAKs is held, as a
  host controller holds a URB, and is not logged until it completes,
  so a log does not depend on how often the host polls (the kernel
  keeps one interrupt IN URB pending while a hidraw node is open, and
  unlinks it on close). `# section NAME` lines separate the phases of
  a run, `! violation: ...` lines record what the host did outside the
  protocol, and with `-v` the holds, unlinks and connections appear as
  `#` comments.
- Pedantic checks come in two layers: `Device` flags transfers to
  endpoints the configuration lacks or before `SET_CONFIGURATION`,
  unknown configurations, interfaces, features and descriptors; a
  model flags what its protocol forbids (for the relay board: report
  lengths other than 8, report ids, unknown commands, relay numbers
  beyond the board). A clean run under sigrok-cli validates the
  model, a clean run under oxrok validates the port.

### Scenarios

`tools/diffcli.sh --emu` runs, after the software cases, one session
per scenario file `tools/emu/MODEL.sh`. The file is sourced; it sets
`EMU_MODEL` (the `oxrok-emu` model name), optionally `EMU_OPTIONS`
(`-o relays=2`), `EMU_DRIVER` (the sigrok driver ids its cases use,
the first being the default) and `EMU_GCOV` (the libsigrok driver
directories `--gcov` measures, default `EMU_DRIVER`), and defines a
function `emu_cases` of `emu_case NAME [flags] -- ARGS...` lines.
`EMU_FULL` lists the drivers oxrok has ported: a case of one of them
that `oxrok -L` does not list fails, rather than quietly turning into
an oracle case, and an `oxrok -L` that fails or lists no driver fails
the run. A case name neither starts nor ends with white space and holds
no tab or newline, since the emulator trims what it acknowledges.
`--emu-model NAME` (repeatable, or a comma list) runs only those
models and no other cases; `-k` still selects cases by name.

A session takes an exclusive `flock` on `/tmp/oxrok-emu.lock`
(`EMU_LOCK`; waiting up to `EMU_LOCK_WAIT`, 1800 s), so several runs on
one machine take turns at the controller. Under the lock it runs
`oxrok-emu cleanup`, starts the server, waits for `READY`, runs the
cases, detaches and checks that no violation was logged outside the
cases (during the attach or the detach, or logged after a case read its
section: the session counts every violation in the log against those
the cases reported). If the server dies, is stopped (process state
`T`) or does not answer within 15 s, the remaining cases fail at once,
and the session ends it with a TERM, then a KILL (with a CONT, so that
a stopped sudo goes too), fails and releases the lock. An interrupt
(INT, TERM, HUP) stops the whole run: the tool running is terminated,
the device detached and the lock released. The lock is the session's,
not the server's (sudo closes inherited descriptors): a server left
behind by a session that was killed outright keeps running until its
end of input closes, but the `devid` check above keeps it away from
the next session's device. The values of the latest READY, REPLUGGED or
RENUMERATED line are exported to the cases as `EMU_PORT`, `EMU_BUSID`,
`EMU_USB`, `EMU_USBFS`, `EMU_NODES` (comma separated) and the first node
of each kind as `EMU_HIDRAW`, `EMU_TTY` and `EMU_USBTMC`; a case that
selects the device by path uses them (`conn=$EMU_HIDRAW`), never a fixed
`/dev/hidraw0`. After each tool's run the harness sends `sync`, so a
renumeration that run caused is reported before the outputs are compared
and before the next case line is read. A case line expands them before
its own reset, and a reset that re-attaches the device (it renumerated
in the case before, or the options changed its descriptors) moves it
once more: `conn=$EMU_USB` holds only on a board whose descriptors stay.
`emu_reset` on the line before moves such a board first, so that the
case line reads where it went. That is good for one run: a full case's
two tools see different addresses on a board the first tool renumerates.
A case that wants a stale address keeps the old value itself.

Each case runs every tool against a fresh model (a `reset`) in its own
log section. Its output first has the node numbers and the device
address made generic (`/dev/hidrawN`, `/dev/bus/usb/BBB/DDD`,
`conn=BUS.ADDR`), since they change from one attach to the next, then
is prepared as `case_` prepares it (header, `--filter`, `--names`), so
a `--filter` sees the text the goldens hold, and the version line goes
even under `--keep-header`. `--filter` and `--names` apply to the
output only, never to the transactions. Every tool must print
something (`--no-output` asks for nothing instead, for a `--set`),
full case or oracle case. What else is checked depends on oxrok:

- When `oxrok -L` lists the case's driver, the case is full: both tools
  run (each stopped after `EMU_CASE_TIMEOUT`, 60 s, so a tool stuck on
  the model does not hold the lock), must exit 0 (non-zero with
  `--expect-fail`), give the same
  output and cause the same transactions (the log lines of the
  section, `#` comments dropped), with no violation under either.
- Otherwise it is an oracle case, the state of a model written before
  its driver is ported: sigrok-cli alone, which must exit as expected
  and log no violation.
- `--oracle-bug WHY` marks a full case sigrok-cli cannot run because of
  a libsigrok bug: sigrok-cli must fail (its stderr is the golden
  `SLUG.oracle`), oxrok alone is held to the `.out` and `.tx` goldens,
  and a sigrok-cli that starts succeeding fails the case ("no longer
  fails (WHY fixed?)"), so an upstream fix shows. In an `.oracle`
  golden a repeated stderr line counts once, since a driver looping on
  an error repeats it as often as time allows. From a sigrok-cli the
  timeout stopped, a last line without its newline, or a bare `sr: `,
  is dropped first, as it was cut off in mid-print. The
  `LIBUSB_ERROR_ACCESS` line of a retried open of a renumerated device
  is dropped from any `.oracle`, as it comes at random.
- `--oracle-differs WHY` marks a full case where sigrok-cli exits 0 but
  is wrong because of a libsigrok bug. sigrok-cli must exit 0 and its
  output or transactions must differ from oxrok's; its stdout, prepared
  as the case says (never its stderr), is the golden `SLUG.oracle`, and
  oxrok alone is held to the `.out` and `.tx` goldens. A sigrok-cli
  that comes to match oxrok fails the case ("oracle fixed?"), so an
  upstream fix shows. `--expect-fail` and `--stderr` then apply to
  oxrok alone, for an oxrok that refuses what sigrok-cli runs. It does
  not combine with `--oracle-bug`, `--hangs` or `--record`. A blessed
  full run removes `.oracle` goldens no case wrote, and a case that
  becomes ordinary drops its `.oracle` when blessed. Bytes libsigrok
  reads past a buffer are masked by the case's `--filter`.
- `tools/diffcli.sh --no-oracle` (implies `--emu`) runs the emulator
  tier without sigrok-cli, as CI does. A full case runs oxrok alone
  against the committed `.out` and `.tx` goldens, the ones sigrok-cli
  wrote; for an `--oracle-bug` or `--oracle-differs` case that is
  oxrok's pinned `.out`/`.tx`, and its `.oracle` is not checked. An
  oracle-only case (oxrok has no driver for it) is skipped and counted,
  and a model with no full case gets no session. Events, `--record`
  fixtures and the violation checks work as in a normal run. The
  software cases, `--hw`, `--gcov` and `EMU_BLESS=1` need sigrok-cli, so
  `--no-oracle` refuses them. The tally reads `P passed, F failed: N
  ran, S skipped`. A case behind a helper that holds the device is
  counted too: the helpers do nothing in the dry count. A driver that
  `EMU_FULL` lists but `oxrok -L` lacks fails its cases, as in a normal
  run, and a run in which no case ran fails. Before each of oxrok's runs
  the harness unbinds, through sysfs as root, any kernel driver still
  bound to a vendor-specific interface of the emulated device (ftdi_sio
  on the FTDI models), as libftdi's detach in the sigrok-cli run that
  would have gone first leaves it; class interfaces (HID, CDC, USBTMC)
  keep their drivers, and each unbind is logged as a section of its own
  in the session log, outside the cases' transactions.
- A scenario that sets `EMU_TX_FOLD=1` folds its transaction log before
  it is compared or cut at a hang. Each run of two or more consecutive
  lines that differ only in their one `@LEN:SHA256` payload digest (same
  direction, endpoint, request and lengths) becomes a single line,
  `@LEN:fold=COUNT:SHA256`, where COUNT is the number of lines folded
  and SHA256 is the digest of their text, each line with its newline.
  Violation lines (`!`) and comments are never folded, and violations
  are counted before folding. Any change to a folded payload, or to the
  number of transfers, changes the golden. chronovu-la and asix-sigma
  use it for their memory readouts. Leave it off for a model whose
  `--hangs` loop repeats a digested transfer: folding would put the
  timing-dependent repeat count into the golden before the hang cut
  could remove it.
- A bulk IN is handed to the model, and logged, at the URB's length
  rounded up to whole packets, which is what the device sees on the bus
  (nusb can submit nothing shorter, libusb can). The URB's own length
  still decides the outcome on the host: an answer longer than it
  completes with -EOVERFLOW (a babble), as the Linux host controller
  reports it, and the log says so.
- `--libsigrok` marks a library-verified case, not a CLI-verified one,
  for a driver sigrok-cli cannot drive: `tools/libsigrok-run.c` runs in
  sigrok-cli's place and alone, whatever oxrok has. It takes
  sigrok-cli's `-d`, `-c`, `-C`, `-t`, `-O`, `-l`, `--samples`,
  `--time`, `--frames` and `--scan`, calls `sr_session_start()` itself
  with or without a limit, and writes through libsigrok's output
  modules. `--stop-after N` counts samples itself, cuts the output at N
  and asks for the stop from the main loop, as a client's button would;
  a driver that ends the capture in the callback that sends its samples
  has finished by then, and `--stop-after 0` stops at the header, before
  any sample. The stop of `--stop-after` runs at `G_PRIORITY_HIGH`, ahead of the USB
  source's events: at `g_idle_add()`'s priority a completion due in the
  same iteration goes first, and how far the driver gets before the stop
  is a race. `--stop-after-ms N` asks for the stop N ms after the session
  has started, from a timeout of the main loop, for a stop between a
  driver's transfers. Like `--stop-after`, it goes after the `--`.
  `--stop-after`'s stop runs from a main-loop callback, but libusb can
  complete further transfers in one event pass before it runs.
  `--stop-in-callback` asks for the stop from the datafeed callback
  instead, for a driver whose stop does no USB (ikalogic-scanalogic2).
  `--continuous` is taken and means the session runs until the driver
  ends it, with no any-key stop, so that a full case can give oxrok the
  same arguments. diffcli.sh builds the runner with `cc` against the
  oracle's prefix (`LIBSIGROK_PREFIX` overrides it). That needs a C
  compiler and glib's headers (glib2-devel, `pkg-config glib-2.0`), not
  the -devel packages behind libsigrok.pc, and the build fails if the
  runner does not load the `libsigrok.so` sigrok-cli loads, so `--gcov`
  counts its runs alike. Without them a `--libsigrok` case fails, naming
  what is missing. The case is held to its own goldens like any oracle
  case, takes `--hangs`, `--timeout`, `--expect-fail` and events, not
  `--oracle-bug`, `--oracle-differs` or `--record`, and `--no-oracle`
  skips it. Such a case says what libsigrok does, not what sigrok-cli
  does.
- `--libsigrok --oxrok` is a full case with the runner in sigrok-cli's
  place, for a driver sigrok-cli cannot drive once oxrok has it: same
  arguments, oxrok held to the same goldens for output and transactions,
  under `--no-oracle` too, and `--oracle-bug`, `--oracle-differs` and
  `--record` work as for sigrok-cli.
- `--tx-filter SED` is a `sed -E` script over each tool's transaction log
  before it is compared, for an order the C driver's timing decides; the
  case says what it hides.
- `--no-oracle-only` marks a case that runs only under `--no-oracle`, for
  what sigrok-cli cannot run first (a flag it lacks, an address its
  upload would move). Its goldens are written by hand, as a bless does
  not run it, and its slug counts in every mode, so that a bless keeps
  them.
- The tools run with stdin from `/dev/null`, whatever the harness was
  launched from: sigrok-cli's `--continuous` stops on any key, and at
  EOF it stops at once, so a driver's own stop is reached only through
  `libsigrok-run`. A case with `--interrupt` gets a FIFO that stays
  quiet instead, or a `--continuous` capture would end before the signal.
- `--oracle-hangs` with `--oracle-bug` marks a full case where
  sigrok-cli must be stopped by the timeout (status 124 or 137), so a
  silent exit 1 fails. `--oracle-differs` does not require sigrok-cli to
  print something: its `.oracle` golden pins an empty output, and the
  two tools must still differ. An `--oracle-hangs` run keeps its gcov
  counts, so turning a `--hangs` case into one loses no coverage.
- A model can say what a held IN transfer holds when the host cancels it
  (`DeviceModel::in_unlinked`). The URB then completes with -ECONNRESET
  and those bytes, as an HCD does, and libusb reports a timed out
  transfer with data. This is how a device slower than a driver's
  transfer timeout, or a last transfer with no short packet, is shown. A
  case on it must end after few rounds: C's timeouts and resubmissions
  interleave with the unlinks, so the transfer that gets the data
  varies; only the logged bytes are stable. What a model says a held
  transfer holds goes to the oldest held URB of its endpoint alone, since
  a queue fills in order: nusb cancels the newest first, libusb's
  timeouts the oldest.
- `--digest` ends the compared output with the SHA-256 of the whole
  output before `--filter`, so a filter that thins a long output (the
  hantek-dso frames keep every 64th line) still compares every byte.
- `--record` records oxrok's run of a full case with
  `OXROK_USB_RECORD` and `OXROK_HID_RECORD` set (the USB recording is
  kept when both wrote), as `tests/fixtures/DRIVER/emu-SLUG.txt`
  (`DRIVER` the case's driver id with `-` made `_`), and its stdout as
  `emu-SLUG.out`, node names made generic and the version line
  dropped. The fixture gets the case name as `# emu MODEL: NAME` and
  the arguments as one `# arg: ...` line each, so arguments with
  spaces survive; node names are made generic as in the output, the
  bus and address are dropped from a USB `device` line, and timed out
  reads are dropped from a HID recording (their number depends on the
  run). Like the goldens, `EMU_BLESS=1` writes them and otherwise the
  new recording must equal the kept one byte for byte, so a recording
  that is not deterministic fails the case. Only a full case that
  succeeds, without an event or `--stderr`, can be recorded. Fixtures
  no case records any more are not removed: delete them by hand.
- `--unplug WHEN` and `--interrupt WHEN` arm the event for each tool's
  run, which must fire it. After an unplug the harness replugs and
  takes over the new nodes and address, so a case that names the
  device by address or node uses the attach before the unplug. An
  interrupt sends SIGINT to the tool, which runs under `env
  --default-signal=INT` (a background job starts with SIGINT ignored).
  `--timeout S` stops each tool after S seconds; a sigrok-cli under
  `--oracle-bug` stopped so fails as the case wants. sigrok-cli has no
  SIGINT handler (it dies, where oxrok ends the capture), so interrupt
  cases are `--oracle-bug` cases, and a run killed by a signal writes
  no gcov counts.
- `--hangs` marks an oracle case where the C driver never ends (it
  retries for ever, or waits for a transfer it never submitted). The
  case passes when `--timeout` stops sigrok-cli (status 124, or 137
  after timeout's KILL) and fails if sigrok-cli ends on its own. A
  full case cannot take it: a port must end there, so such a case is
  an `--oracle-bug` one. Its transactions are cut where the log
  becomes periodic up to its end: the first full period is kept, then
  a `~ the last P lines repeat to the end (3 or more times)` line. The
  period is at most `EMU_HANG_PERIOD`, 512, lines and must be seen at
  least three times; the cut that keeps the fewest lines wins. Three
  periods are all the cut checks, so the line does not claim that the
  loop ran until the timeout. A log that simply stops is kept whole.
  The compared output, usually stderr, goes through `uniq`, by design:
  a looping driver repeats its message as often as the time allows, so
  runs of a repeated line count once, as in an `.oracle` golden.
  sigrok-cli runs with a preload that turns timeout's SIGTERM into
  `exit()`, so its gcov counts are written; an `exit()` that deadlocks
  is ended by an alarm 2 s later and writes none. A hang case compares
  `--stderr` or uses `--no-output`.
- Either way the output and the transactions are compared with the
  goldens `tests/emu/MODEL/SLUG.out` and `SLUG.tx` (`SLUG` is the case
  name in lower case, runs of other characters made `-`), the
  sigrok-cli run's as `EMU_BLESS=1` wrote them, so a later change in
  either tool shows. An oracle case without goldens fails; a full case
  without them passes on the comparison of the two tools. With
  `EMU_BLESS=1` every case writes its goldens instead of comparing
  them (`(golden written)` after its `PASS`), and a full run of a model
  removes the goldens no case wrote any more.

`--gcov` (which implies `--emu`) zeroes the coverage counters of each
model's libsigrok drivers before its session (`tools/emu-gcov.sh DRIVER zero`)
and writes `tests/emu/MODEL/gcov.txt` after it
(`tools/emu-gcov.sh DRIVER report`): per source file the share of
executable lines the cases reached, the total, and the lines never
reached, as ranges per function, a function never called said so.
`EMU_GCOV_SHARED` names the libsigrok sources outside `src/hardware`
the driver goes through (`src/usb.c src/ezusb.c`, `src/serial.c
src/serial_hid.c src/serial_hid_cp2110.c`); `tools/emu-gcov.sh --shared
zero|report SRC...` zeroes them with the driver and reports them after
it in the same form. Every driver goes through them, so their figure
is what that model's cases reached, not what the driver can.
With `-k` the report is printed, not kept, since a subset of the cases
says nothing about the whole. A failing `emu-gcov.sh` (a directory in
`EMU_GCOV` that does not exist: driver ids and directory names differ,
`uni-t-ut61e` lives in `uni-t-dmm`) fails the session and leaves
`gcov.txt` as it was; a report that finds no executable line of the
driver is a failure too. The oracle is the libsigrok built with
`--coverage` (`docs/DEVICES.md`, "Oracle"); libtool's PIC objects in
`.libs` are the ones `libsigrok.so` is linked from, so their `.gcda`
files hold the counts. Other drivers' runs at the same time only add
zero counts to this driver's files.

Drivers that upload firmware get stand-ins: `tools/emu/MODEL.firmware`
lists `NAME SIZE` lines (`#` comments), and `tools/emu-firmware.sh`,
which `--emu` runs first, writes each file into
`~/work/sigrok/firmware-standin` (`EMU_FIRMWARE_DIR`) as
SHA-256(`NAME:0`), SHA-256(`NAME:1`), ... cut to its size (no two 32
byte blocks alike, so a repeated or reordered chunk shows), rewriting
only files whose
bytes changed. The directory is exported as `SIGROK_FIRMWARE_DIR`,
which libsigrok's `resource.c` searches first and oxrok searches right
after its own `OXROK_FIRMWARE_DIR` (`src/core/firmware.rs`), which the
harness unsets, so both tools upload the same bytes, and the model checks the size and treats
the content as opaque. `EMULOG=DIR` keeps each session's log as
`DIR/MODEL.log`.

What the first model taught: the kernel's HID name joins manufacturer
and product (`www.dcttech.com USBRelay2`), while hidapi reports the
USB device's own product string and names the connection
`/dev/hidrawN`; oxrok's HID layer now does the same on Linux. The hub
asks a full speed device for its device qualifier three times and
takes the STALL; `vhci_hcd: vhci_device speed not set` in `dmesg`
during the attach is harmless. A USB/IP server must not close its end
while attached, so the process keeps the socket it handed to the
kernel until it detaches.

- `--bound DRV` binds DRV to the device's vendor interfaces before each
  tool's run, in a section of its own, as on a fresh plug in. A kernel
  without DRV runs the case unbound, with a note.
- The `.oracle` of `--oracle-bug` masks the address in the core's
  "device N.M acquisition." line.
- `--oracle-env VAR=VALUE` sets a variable in sigrok-cli's environment
  alone, to make a crash that depends on the heap come every time.
- Which signal killed sigrok-cli is not compared, by decision of
  2026-11-05: a case needs only that it failed, and its stderr. To pin
  signals again, revert the commit found by
  `git log -S'oracle died: signal' -- tools/diffcli.sh`, whose message
  says how, and bless the `.oracle` files added since.

### Writing a serial model

`vusb::shells::cdc` makes a CDC-ACM function that the kernel's
`cdc_acm` binds and the tools reach through `/dev/ttyACMn`. A model is a
byte stream and never sees a URB:

```rust
struct Meter;
impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        if data.contains(&b'?') { port.send(b"reply\r"); }
    }
}
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut faults = Faults::default();
    for (k, v) in options {
        if faults.option(k, v)? { continue; }
        return Err(Error::Invalid(format!("{NAME} has no option {k}")));
    }
    let id = Identity::new(0x1209, 0x0001, "Maker", "Name");
    Ok(Box::new(Cdc::new(NAME, &id, faults, Meter)))
}
```

1. `Serial::received` gets what the host wrote. How it was cut into
   transfers means nothing, so collect frames yourself, in a buffer with
   a bound. `Serial::changed` gets `LineEvent::{Coding, Lines, Break}`.
   Each open sends `SET_CONTROL_LINE_STATE` 3, which arrives as `Lines {
   dtr: true, rts: true }` whether or not the lines changed;
   `LineEvent::opened()` says so. DTR rising is not an open: with
   `HUPCL` off a close leaves the lines up. `Serial::tick` is one offer
   of a pending read with nothing queued, about every 20 ms and more
   when the host is busy, and it stands still while the tty is closed.
   `Serial::reset` goes back to power on.
2. `Port` has `send`, `line`, `dtr`, `rts`, `violation`,
   `mark` and `set_modem_lines`.
3. An instrument behind a UART calls
   `.behind_uart(Line::parse("1200/7n2")?)`: while the rate, data bits
   or presence of parity differ, nothing passes either way. A native USB
   port omits it and the settings do nothing.
4. Pace by the host. Answer in `received`, and the reply goes out in the
   same server round as the request. A meter that streams uses
   `hidcable::Stream`, restarted on `opened()`, with a period long
   enough for the tool to close first.
5. Unit tests: `shells::cdc::host::Host::open(cdc, line, dtr, rts)`,
   then `write`, `read`, `offer`, `set_line`, `set_lines`.

The shell's options (all models take them): `stall=line|lines|break`,
`chunk=N`, `in-limit=N` (holds after N bytes and marks the event point),
`in-stall=N`, `out-hold=N`. Endpoints: 1 notifications, 2 data, so
`--unplug in=2:N` and `--interrupt out=2:N` count data transfers.

The scenario sets `EMU_CDC=1` and names the node `conn=$EMU_TTY`, with
`EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"`. Under `EMU_CDC=1`:
- A session with no ttyACM fails. A case whose tty moved after its line
  was read fails, so re-read `conn=` after anything that re-attaches
  (see `serial_dmm_conns`).
- Before each tool the tty goes back to 9600/8n1 in a section of its
  own, and the model is replaced, because the kernel keeps termios and
  the last line coding across closes.
- The `.tx` is normalised: consecutive bulk transfers of a direction
  become one line (`bulk_in 82 LEN -> DATA`, a digest above 1 KiB), and
  a run of `SET_LINE_CODING` and `SET_CONTROL_LINE_STATE` keeps the last
  coding, first, if it changes the line, then the line states that
  change the lines.
- libsigrok swaps odd and even (`serial.c:652-655` against
  `serial_libsp.c:253-258`), so in sigrok-cli's `.tx` alone bParityType
  01 and 02 are swapped back. A driver with `o` or `e` runs full. A
  fixed libsigrok fails the comparison, which is how that would show.

Consecutive means with nothing between: a device that streams while the
host writes interleaves by timing, so avoid it or filter it.

On a request/response driver with no `rts` setting, keeping the probed
port and reopening it look identical in a `.tx`, since an open repeats
the state the lines are in. So a clean `.tx` there does not tell you
which one a driver does.

What the first model taught:
- While the tty is open the kernel holds 16 bulk INs of two packets (128
  bytes at full speed) and one interrupt IN. One `write()` is one bulk
  OUT, up to 20 packets.
- sigrok-cli leaves DTR and RTS up at close (libserialport clears
  `HUPCL`), and each open raises both, so RTS pulses even with `rts=0`.
- serial-dmm closes the port after the scan, and its streaming-meter
  scans took two packets to find one. `ut32x` and the Pico keep the
  probed port, so check each driver's C.
- A driver that asks without a `req_timeout_ms` asks again every 50 ms
  while it waits: a silent device's write count is wall-clock, so filter
  it (`--tx-filter 's/^bulk_out 02 (44)+ -> [0-9]+$/.../'`), and keep
  answers immediate.
- Events go at a quiet point (`--unplug model`, `--interrupt model`,
  `--tx-filter '/^interrupt$/q'`). Bytes just delivered are lost to an
  unplug's hangup, or not, as the pace has it.
- After an unplug, a read returns EOF or POLLHUP. A C driver that only
  reads never notices (`--oracle-bug --oracle-hangs`).
- A device that NAKs bulk OUT blocks C's next write for good, and the
  kernel holds any tool's close for `closing_wait`, 30 s. The pilot pins
  this once ("metex stops reading"; `emu_tty_closing_wait` makes it
  cheap). Do not repeat that case unless a driver's write-error path is
  its own: each copy costs at least 20 s.
- The node number is the lowest free one and comes back the same after a
  replug, once the tool has exited.
- Faults on the line: a stalled `SET_LINE_CODING` is ignored by the
  kernel, a stalled `SET_CONTROL_LINE_STATE` fails the tool's ioctl, and
  a stalled read is recovered by `CLEAR_FEATURE` without any tool seeing
  it.
- `collect_nodes` waits for a device's first node only.
- Packages: `kernel-modules-core` on Fedora, `linux-modules` or the
  extras on Ubuntu. A distribution's brltty or gpsd may claim a tty;
  only ModemManager has been looked at.

### Writing a device model

Do no slow work inside a reply: the host's timeout is running, and a
debug build under load can exceed a 100 ms one. Compute stand-ins when
the model is built.

Every file below is the model's own, so models written in parallel,
each in its own worktree, merge without conflicts. For a model `NAME`
of a sigrok driver `DRIVER`:

1. Read the C driver (`api.c`, `protocol.c`) and, where there is one,
   the datasheet or the firmware source, for what the device answers
   and what the driver may send. Pick the shell (`vusb::shells`,
   `emu/src/shells`) the device builds on; a shell that does not exist yet is a separate
   piece of work, not part of a model.
2. Write `emu/src/models/NAME.rs`, or for a model with submodules
   `emu/src/models/NAME/mod.rs` with the submodules beside it in
   `NAME/` (the generated `#[path]` declaration would make rustc look
   for the submodules of a `NAME.rs` in `models/`, so `build.rs`
   refuses `NAME.rs` next to a directory `NAME`; `tests` is a fine
   name, files starting with `.` are skipped):
   the descriptors as the real device has them (ids, strings, speed,
   endpoints the driver uses), the protocol state machine, a sample
   generator that starts over on `reset` so both tools see the same
   samples, `NAME`, `ABOUT` and `build`. Start the file with the
   "Counterpart in sigrok" header naming the C files it follows, and
   add `tools/attribution.d/NAME.py` with its `MAP` entry (the format
   is in `tools/attribution.d/usbrelay.py`); generate the header with
   `tools/attribution.py --only emu/src/models/NAME.rs`, which touches
   no other file. Unit tests drive the
   model the way the driver does and check one violation of each kind.
3. Make the pedantic checks bite: report through `Violations` every
   request the driver is not supposed to send, every length, index,
   register value or sequence number outside what the C driver's
   comments and the datasheet allow, and the order where it matters
   (a capture before its configuration, a read before its write). A
   model that accepts anything validates nothing.
4. Write `tools/emu/NAME.sh` with `EMU_MODEL`, `EMU_OPTIONS`,
   `EMU_DRIVER` and the `emu_cases` (scan, show, each option's get and
   set, captures at a few rates and sizes, triggers, the selection by
   `conn=`, and the error answers the driver handles), and
   `tools/emu/NAME.firmware` when the driver uploads files.
5. Run it alone, without the software cases:
   `cargo build --workspace && . ~/work/sigrok/env.sh && sh tools/diffcli.sh --emu-model NAME`.
   Every case must pass and log no violation under sigrok-cli; a
   violation there is a fault of the model (or a rule the C driver
   does not keep, which then goes into the model's comments) until
   shown otherwise. `EMULOG=/tmp/emu` keeps the log, and
   `flock /tmp/oxrok-emu.lock sudo target/debug/oxrok-emu serve NAME --attach --open-devices -v`
   serves the model by hand. Take the lock, as that line does, whenever
   other agents may be running sessions: a device served beside a
   session's device makes the session's cases see two, and a session's
   `cleanup` would detach the hand-served one. A script that cleans up
   inside its `flock` passes `cleanup --locked`.
6. Bless the goldens, `EMU_BLESS=1 sh tools/diffcli.sh --emu-model NAME`,
   read them (they are the reference from now on), and run again
   without `EMU_BLESS` to see them pass. Commit them.
7. Measure: `sh tools/diffcli.sh --emu-model NAME --gcov` writes
   `tests/emu/NAME/gcov.txt`. Read the unreached ranges against the C
   source: add cases for the paths a device can take, and leave those
   it cannot (allocation failures, transport errors, model variants
   nobody has) as the "not emulated" list. The roadmap's criterion 4
   stops there.
8. In `docs/DEVICES.md`, in the driver's entry (or a new one): that it
   is emulated, the model's name and options, the cases, the date, the
   gcov total with the "not emulated" paths named, and anything the
   model taught about the driver. The README table says "emulator"
   only once oxrok's port passes the same cases as full cases.
9. Before handing over: `cargo fmt --all --check`,
   `cargo clippy --workspace --all-targets` with no warning,
   `cargo test --workspace`, and `sh tools/diffcli.sh --emu-model NAME`.

## Troubleshooting

- `Segmentation fault timeout -k 5 ...` appears twice in every
  lecroy-logicstudio run, from the pinned `bits.c:158` crash cases. It
  is not tied to the case printed next to it.
- Linux: without a udev rule the device is root only. Packaging will
  ship rules (`docs/ROADMAP.md`); until then `sudo` or a rule with `MODE="0666"` for the vendor
  and product id. A kernel driver (`ftdi_sio` for FTDI devices with a
  known product id) is detached when the interface is claimed, with
  both backends.
- macOS: a third party FTDI VCP driver that matches the device would
  block the claim with a busy error. `kextstat | grep -i ftdi` shows
  whether one is loaded. After a firmware upload and renumeration the
  system can hand out a device whose configuration is active but whose
  interface cannot be claimed ("interface not found"); the nusb backend
  then sets the configuration again and reopens the device, which is
  what libusb does on every open.
- Windows: nusb needs the WinUSB driver bound to the device, as libusb
  does; use Zadig once. The control transfers go through the claimed
  interface, so drivers claim before any request (the FTDI layer does).
- The `bus.address` in `conn=` comes from the OS: on Linux the real
  numbers, on macOS the ones libusb derives from the location id (the
  same sigrok shows), on Windows always `0.0`.
