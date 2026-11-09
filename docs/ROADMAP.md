# Roadmap

Status: 2026-11-09. `v0.2.0` is the current stable release, on GitHub and
crates.io. All 87 of libsigrok's `src/hardware` drivers are
ported. Native decoder porting is
complete: all 111 decoders of libsigrokdecode 0.5.3 are native.

The work that remains is transports, packaging, the Pico backend, and
the emulator, CLI and hardware-verification gaps listed here. Completed
phases and per-batch history have been removed; the evidence per driver
is in [DEVICES.md](DEVICES.md) and the decoder checklist in
[NATIVE_DECODERS.md](NATIVE_DECODERS.md).

## Transports

No USB logic-analyzer driver needs code: every one is ported, has a replay
fixture, and needs hardware runs.

### Bluetooth

`mooshimeter-dmm` runs over `src/bt` (an L2CAP ATT socket) and the BLE
shell (`emu/src/shells/ble.rs`, a controller on `hci_vhci`). Open:

- `atorch`'s BLE units (libsigrok's `serial_bt.c`: `bt/ble122`,
  `bt/cc254x`, `bt/notify`) can go over `src/bt` and the BLE shell. They
  need a `Port` on a notify/write characteristic pair, CCCD indications in
  the shell (`ble122` writes 0x0003), and an ATT MTU above 23 with
  fragmentation if a unit asks for one. RFCOMM (`bt/rfcomm`) is still open:
  the shell has no BR/EDR.
- The BLE model is skipped by name where the kernel has no `hci_vhci`
  (presumably GitHub's hosted runners), so CI gives it no coverage; a
  self-hosted runner with `hci_vhci` and no bluetoothd would run it.

### GPIB

`hp-3478a` runs over `src/gpib` (`conn=gpib/MINOR/PAD`) on the kernel's
`gpib` subsystem; only `ni_usb_b` is modelled. Open:

- `libgpib/NAME` resources, read from `/etc/gpib.conf` as libgpib's
  `ibfind` does.
- `agilent_82357a` and `lpvo_usb_gpib` models.
- Other SCPI drivers over `conn=gpib/...`: the transport is shared, but no
  case covers one.
- A CI runner without `ni_usb_gpib` skips the model by name with a warning
  and runs none of its cases.
- Waiting on SRQ (`IBWAIT`) instead of the 100 ms serial poll.
- usbtester4 has minor 0 left configured by an early `gpib-config` with no
  marker: take it offline by hand (or reboot).
- Prologix adapters over serial remain an oxrok extra.

## Upstream pull requests

Pull requests to sigrok's libsigrok, sigrok-util, sigrok-firmware-fx2lafw
and sigrok-firmware that never landed upstream, triaged for oxrok on
2026-11-09 and tracked one item per unit of work on the
[Upstream intake](https://github.com/orgs/oxrok/projects/1) board
(issues labelled `upstream` here). The rules, the intake and progress
skills and `track.py` are in
[oxrok/management](https://github.com/oxrok/management). The workstreams:

- `bug-check`: upstream fixes for bugs oxrok may share (USB bus 0 on
  macOS, macOS HID paths, RD6006 multipliers, Kingst stream stalls,
  center-3xx recovery, crafted VCD input, demo, R&S RTM/RTB).
- `tooling`: firmware extractors in oxrok-util.
- `firmware-build`: firmware-fx2lafw built on Linux, macOS and Windows
  in CI, WCID descriptors, and the open Kingst FX2 firmware.
- `model`, `driver-rework`, `new-driver`, `input-format`: models for
  ported drivers (Rigol DHO, Siglent HD, Korad, ...), larger driver
  changes (OLS, FTDI-LA, DSLogic), about 30 drivers oxrok lacks, and two
  input formats.

## Packaging and release

- Linux udev rules. The README and [USB.md](USB.md) both say a device is
  root-only without one, and no rules ship yet.
- A Homebrew formula or tap.
- An AUR `PKGBUILD`.
- A stable release. Every published version so far is a pre-release, so
  `cargo install oxrok` needs an explicit `--version`.

Static binaries, the release archives, `make dist`, notices and
`cargo publish` already work.

## MCP server

An MCP server so an agent can scan, capture, measure and decode through
oxrok: `oxrok mcp` over stdio ([MCP.md](MCP.md) documents it), and an
extended server in oxrok-ui whose tools the user picks.
[PLAN.mcp.md](../PLAN.mcp.md) has the design, the phases and the open
decisions; the CLI side (phases 1 to 5) is in.

- `driver_info` does not yet report a driver's verification status
  (hardware, stand-in, emulated, replay) from `docs/drivers.toml`: the
  docs are not in the published crate, so the table would have to be
  generated into the source at build time or kept in `tools/site-data`.
- A driver whose acquisition ends on its limit is only stopped (its
  thread joined) by the MCP run thread; the CLI runs one capture per
  process and never needed it. `Session::run` could stop the devices
  itself when every `End` came, so every caller gets an idle device.
- A `guess_signals` tool: rank a capture's logic channels as clock,
  data, select or idle from the features `measure` reports (regular
  period, pulse widths that are multiples of another channel's period,
  edges only while another channel is low), try the common two and
  three wire decoders on the candidates, and score each by frames
  decoded and the warnings row. An agent does this by hand today over
  `measure`, `capture_logic` and `capture_decode`.
- Phase 6: the UI server over a local socket, with a per-tool checklist.

## Pico backend

Replace the C++ roles in `hil/pico-hidcable` with Rust `DeviceModel`s on
a Pico, so the same model runs on the host over USB/IP and on the bench
device. One model (for example the relay board or a HID cable) running
both ways with the bench cases passing on both is the exit criterion.
The models are plain Rust with no I/O for this reason; see
[USB.md](USB.md).

## Emulator and CI gaps

### Models and coverage

- String-descriptor failures at the scan: the `fail_standard` hook
  exists (fx2lafw uses it), but the earlier FTDI and EZ-USB models still
  need their own option and cases.
- A persistent endpoint halt that stays until `CLEAR_FEATURE`, not a
  one-shot STALL, with `--timeout` and `--oracle-bug` cases.
- The BM52x `data_source` option (recordings stored in the meter) is in
  libsigrok and absent from oxrok, so `bu86x`'s `meter=bm52x` has no
  `--show` case.
- Variable-length and `after_open` meters (BM85x) and the multi-display
  meters (EEVblog 121GW, MS2115B, Metex 4-display) wait for CDC-ACM
  coverage; they hold most of `serial-dmm`'s unreached lines.
- `asix-sigma`'s inline bulk OUT payloads are not covered by
  `EMU_TX_FOLD`.
- The harness records USB and HID only. Add `--record` for serial, so
  serial drivers get replay fixtures.
- A CP210x shell (`10c4:ea60`, vendor class) and the FTDI shell for
  serial models. Manson, GPD and Atten currently run on a CDC-ACM
  stand-in, so `cp210x`, `ftdi_sio` and `/dev/ttyUSBn` are not
  exercised.
- `--oracle-bug` accepts any non-zero exit; dslogic's two trigger-fault
  cases still have an empty `.oracle` and pin nothing.
- `tools/attribution.py --check` would rewrite the headers of
  `emu/src/models/hantek-dso.rs`, `emu/src/models/hantek-6xxx.rs` and
  `src/drivers/fx2lafw/mod.rs`. The credits belong in
  `tools/attribution.d`; firmware-derived facts are credited by hand.
- Device firmware sources may settle what models assume (the
  Pipistrello's overturned three assumptions). DreamSourceLab's FPGA
  sources and the sysclk-sla5032 vendor ISO are unexamined.
- Literal short USB timeouts in C drivers, the kind that made Kecheng's
  scan miss a present device: `hantek-dso` at 40 ms and `lascar-el-usb`
  at 50 ms (control transfers, which `EMU_ORACLE_USB_FLOOR_MS` does not
  cover).
- Other models call `ezusb::standin` during transfers: `saleae-logic16`,
  `saleae-logic-pro`, `dreamsourcelab-dslogic` and `kingst-la2016`; the
  same stop race may be open in `hantek-4032l` and `sysclk-lwla`.
- `saleae-logic-pro`'s `usb.c` gcov varies from run to run (76–79%).
- Three unexplained rare failures: the emulator's `usbtmc` unplug case,
  `hantek-dso`'s `interrupt continuous` transaction log, and a
  `cem-dt-885x` session lost after an interrupt's close.

### Harness and CI

- `uss720` keeps a detached cable's parport number (one per emulator
  session on kernel 7.2.x), so after 8 sessions on a tester libieee1284
  finds no ppdev and the `hung-chang-dso-2100` oracle stops; the harness
  fails such a session and says so. Unloading `parport` doesn't help (the
  new `parport0` collides in sysfs); only a reboot does. Testers need a
  reboot between batches, or the harness a way to reset the numbering.
- Pulling a `uss720` cable while the port is claimed crashed usbtester3's
  kernel, and usbtester3 did not come back from its reboot: no unplug case
  for `hung-chang-dso-2100` until that is understood.

- No record/replay path for a driver that runs a program
  (`asix-omega-rtm-cli`): its logic runs in CI only through the emulator's
  stand-in and unit tests.

- sysfs carrier, synchronous stores: `oxrok-emu --sysfs` sees a write only
  after the tool closes the file, so a store the model refuses never fails at
  the tool, and a read right after can still see the refused value. A
  same-length rewrite is one write(2) a reader could catch mid-copy. A FUSE
  tree, or fanotify permission events, would make a store's errno, -ENODEV
  reads of a removed device and sysfs's 4096-byte attribute size reachable.
  Reads already wait for the carrier (fanotify permission events, as root),
  so a read's value depends only on the reads and writes before it.

- The emulator CI job skips when the hosted kernel has no `vhci-hcd`,
  and branch protection counts a skipped required job as satisfied. A
  self-hosted Linux runner, or a setup that guarantees the module, would
  close this.
- The libusb backend has no regular emulator run; the gates use nusb.
  Wanted: a regular `OXROK_USB_BACKEND=libusb` run, comparing output
  only.
- Several emulator sessions at once are not isolated: libusb lists every
  device and drivers match by VID:PID, and cleanup, port choice and gcov
  counts are global. Test one tool per namespace whose `/dev/bus/usb`
  holds only its own node.
- The harness follows the address through RENUMERATED, but a case line
  expands `$EMU_USB` before its own reset, so a full case's two tools
  can see different addresses on a board that renumerates. `emu_case`
  also resets and re-attaches before a claim helper's hold, so scenarios
  order around it.
- `tools/diffcli.sh`'s `to_cmp` ignores `sed` and `mv` failures, so a
  full disk can fail a case falsely or pass one; `emu_hex` loses a value
  with a leading `-` (`printf --` fixes it); a `libgcov profiling error:`
  line or a "dumped core" line fails a stderr comparison and should be
  filtered.
- Test fakes not yet on `WriteLog`: the `Seen` aliases in
  `gwinstek_gpd`, `gwinstek_psp` and `motech_lps_30x`; the `written`,
  `writes`, `log` or `commands` fields in `devantech_eth008`,
  `icstation_usbrelay`, `link_mso19`, `pce_322a`, `uni_t_ut181a`,
  `uni_t_ut32x`, `serial_lcr`, `mastech_ms6514`, `zketech_ebd_usb`,
  `ikalogic_scanalogic2`, `manson_hcs_3xxx/protocol.rs`,
  `juntek_jds6600`, `rohde_schwarz_sme_0x`, `pipistrello_ols`,
  `ols/protocol.rs` and `modbus`. A panic in a fake can poison a lock
  and abort the test binary; a worker that spins after a stop without
  writing hangs `close()`.
- Three scenarios carry copies of the lock preload (`gwinstek-gpd`,
  `tondaj-sl-814`, and a larger one in `devantech-eth008`): make one
  shared file and helper.
- `fn shots/hit` is duplicated in five models.
- A vusb race, seen once in about 11,100 attaches: the kernel can send
  `CMD_UNLINK` ahead of its `CMD_SUBMIT`, and the late `RET_SUBMIT`
  makes `vhci_hcd` drop the connection. The fix is in vusb: remember an
  unlink above the highest submitted seqnum and drop that submit.
- `serve --attach`'s "sudo modprobe vhci-hcd" hint on Windows is
  Linux-specific.
- CI's Python decoder slices are floored by one test:
  `decoder_uart_rest::amulet_ascii_regression` runs over four minutes
  under CPython. Shorten its input or find why it is slow, then retune
  the slice counts in `ci.yml`. Those slices also re-run the whole
  `oxrok` suite, most of which the `test` job already runs; a nextest
  `-E` filter to the decoder tests and the Python-gated code would
  halve them.
- `tools/ci/changes.py` balances emulator shards by golden case count,
  a rough proxy (a 12-minute and a 22-minute shard had 1339 and 1789
  cases). `diffcli.sh` printing each model's elapsed time would allow
  real weights. It also treats all of `src/serial`, `src/usb`,
  `src/cli` and `src/formats` as core; per-file rules (`tcpraw.rs` is
  used by the TCP drivers only) would narrow the full runs that most
  driver ports still trigger.

### CLI and format correctness

- The CLI closes a device only on its happy path, and `ctrlc` is built
  without `termination`: a driver that writes at close (`rdtech-dps`'s
  panel lock) needs its own `Drop`. Close on every exit.
- `-O csv` mixes units in a column for a driver that sends one packet
  per reading (`gwinstek-gpd`), and holds pending rows in memory: all 13
  `demo` channels at 200 kHz reach 171 MB in 30 s and 624 MB in 100 s
  on a debug build. Dump past a row bound, avoid `Vec<bool>` per row,
  and cap `demo`'s catch-up batch. `-O vcd` is unmeasured.
- `-C D-` reads as a range, so a channel named `D-` cannot be selected
  alone; try the exact name first.
- `-c` validates parsing, scope and access before a driver's ranges, so
  `voltage_target=5:current_limit=99` sends the voltage before the
  refusal.
- `src/cli/pipeline.rs`'s `check_limit` sums readings across channels,
  so a capture cut short with at least half its samples gets no warning.
- `-O analog:digits=spec` shows the same digits as `all`; `AnalogData`
  holds one digit count. `-O analog --show` also words `digits`
  differently from sigrok-cli. `-O ols` is absent, and `0x10`/`3.3V`
  are refused where sigrok-cli's `strtof` takes them.
- Several drivers narrow settings to `f32` where only sigrok-cli parses
  with `strtof` (`manson-hcs-3xxx`, `conrad-digi-35-cpu`, and more
  among the serial drivers). The narrowing belongs in the CLI's option
  parsing, not in the drivers.
- `--show -g GROUP` behaves differently: sigrok-cli stops after the
  device line when `sr_dev_options` fails for the group, oxrok prints
  the groups and an empty option list; `--get` with a group fails with
  different wording.
- A demo capture with a trigger on an analog channel
  (`-d demo:logic_channels=2:analog_channels=1 -C D0,A0 -t A0=r`)
  hangs oxrok. Mixed VCD has a double space after the timestamp and,
  on demo, lacks sigrok-cli's `#5 0!` and `#20 1!` lines.
- `--samples 16` on `demo:logic_channels=8` gives different `bits`
  under the two tools, with or without an analog channel, so `-P i2c`
  differs there too.

### Driver behaviour

- Analog samples are `f32` in the core (`AnalogData.samples`): `rigol-dg`'s
  counter gives 10 digits and keeps about 7.
- A core output maps a logic channel to its bit by its listing index, so
  `yokogawa-dlm` needs `Device::channels_listed_as_they_are()`; a separate
  channel-to-bit map would let a driver keep C's index-32 channels.
- A VXI-11 transport, the Yokogawa DLM's Ethernet path.
- `siglent-sds`: an E-series history mode as pull request 247 has it
  (`HSMD`, `FRAM`).

- `hp-3457a`'s look at the line before each question, which catches an extra
  answer, has no counterpart on USBTMC.

- `ipdbg-la`: run-length compression (core id `idbg`, commands 0x10 and 0x60),
  as on the core maintainer's libsigrok branch; check it against a real core
  behind OpenOCD, including OpenOCD's one connection per core.
- SIGINT doesn't end a run whose stdout is blocked: the `ctrlc` handler only
  sets a flag (`src/cli/run.rs`) while the main thread waits in the write.
  `oxrok --continuous -O binary | (sleep 8; ...)` then SIGINT: still running
  after 10 s, and the driver's `close` never goes out.
- csv invents a value for an enabled channel that sends nothing: `fluke-45`
  with P2 enabled and silent, `--samples 3 -O csv:header=false:time=false`,
  prints `1.0001,0`, a 0 nobody measured. Leave the cell empty, with a case.
- `-O analog:digits=spec` is accepted and ignored: `src/formats/output/analog.rs`
  never reads it. sigrok-cli prints `0.1 A` where oxrok prints `0.12 V`.
- A VICP transport (LeCroy's network interface, TCP port 1861, 8-byte headers,
  per the X-Stream Remote Control Manual WM-RCM-E Rev C), for `lecroy-xstream`.

- `Scpi::get_block` (`src/scpi/mod.rs`) returns `Ok` with a cut block when
  a read ends in LF on a serial transport, or on a short `tcp-raw` read:
  its `read_complete()` check ends the block early. A fake port serving
  `#210abc\n` then `defghi` gives `Ok("abc\n")`. No driver calls it yet;
  fix it before the first one does.
- `Serial::read_data` (`src/scpi/serial.rs`) drops a CR from a read that
  ends in LF CR, so `rigol-ds` on a serial port loses a sample byte when
  a pair 0x0A 0x0D ends a read, and the block comes up one short. C's
  `scpi_serial_read_data` has the same lines (a0ade2f9). Read binary
  blocks without the strip, and add a `rigol-ds-serial` case that places
  those bytes.

- `Session`'s `End` is sent before the link goes back in its slot, so
  `start()` right after `End` can get `Busy` (`rdtech-dps`'s
  `a_second_capture_runs_after_the_first` failed once under load, `lecroy-logicstudio`,
  `saleae-logic16`, `kingst-la2016`, `hantek-6xxx`, `sysclk-lwla`);
  `sysclk-sla5032` has the fixed order.
- About 20 drivers block in `sink.send` while close joins them; a full
  queue nobody reads hangs the close (`atten-pps3xxx` did under load).
  `PacketSink::send_while` is the fix; whether the rest get it one by
  one or through a shared change is undecided.
- The rusb stream thread exits on any non-timeout error, keeps one read
  in flight, and is not joined on drop, so one transient error loses
  the rest of a capture. `nusb_backend`'s `bulk_out` takes `Cancelled`
  with some data sent as success, where C fails on a timed out write;
  check it against a device that times out.
- `src/scpi`: `Scpi::get_data` has no size bound, so a device streaming
  with no newline holds a `--get` or `--scan` until it stops.
  `rigol_ds/mod.rs` passes `None` when it reopens, so a user's
  `serialcomm` is lost and it opens at 115200, where C keeps it.
- `devantech-eth008` forces 9600/8n1 and refuses `serialcomm`; over TCP
  with `OXROK_SERIAL_RECORD`, a reconnect starts a new `Recorder` on the
  same file and can keep only the last connection. It and
  `rohde-schwarz-sme-0x` could move to the TCP carrier.
- `korad-kaxxxxp` has the old silence bound on `--time` and
  `--continuous`, holds its lock across a send, and ignores `-C`.
- `p-ols` has the `Busy` on a second capture that `ols` fixed by
  reaping once the port is back.
- Silence after a sample: ports merged earlier fail at their bound after
  a sample (`kern-scale`, `serial-lcr`, `cem-dt-885x`, and others), where
  the newer reviews (`center-3xx`, `teleinfo`, `tondaj-sl-814`) warn once
  and go on to a 10-minute limit. Sweep them to the same rule.
  `manson` checks its bound between bytes of a reply, so a stall over
  its 5 s bound mid-reply could end a capture falsely.
- `Native::drain` has run only on Linux; on macOS or Windows a failing
  `bytes_to_write` would fail a healthy `conrad-digi-35-cpu` set after
  its frame went out.
- Testo: more units from Testo's table (°F, CO2, dew point) once a
  recording shows a 435 sends them; decimal exponent for `digits`.
- Serial `Native` holds a second tty descriptor for HUPCL and INPCK, so
  each oxrok open sends the line state twice (the `.tx` normalisation
  hides it). Kept rather than relaxing `forbid(unsafe_code)`.

### Flakes and test gaps

- Nine unit tests fail with every sleep at six times, a harsher test
  than CI's macOS runner: `gwinstek-gpd` (2), `atten-pps3xxx` (3),
  `rdtech-dps` (2), `rdtech-um` (1), `manson-hcs-3xxx` (1).
- `csv_matches_sigrok_cli` fails about 2 of 30 here with an extra row at
  5 µs, likely `dedup` flushing at a demo packet boundary. Under
  injected stalls: `motech_lps_30x::a_status_that_is_no_number_keeps_its_sample_only_with_no_expiry`
  about 1 in 3 at 150 ms; at 400 ms two `fluke_dmm`, `gwinstek`
  `the_second_half_of_a_line_end` and `korad` `a_long_gap`.
- `tcp_raw_cli::nothing_listening_is_no_device` failed once in a
  `norma-dmm` gate run and passed on rerun; a sibling's listener may
  have taken the freed port.
- Scenario filters hide oxrok's own text: the `bkprecision-1856d`
  review found `A` and `O` patterns hiding `: invalid value for ...` and
  `: device error: ...` after "Failed to open device"; motech, rdtech
  and others use the same patterns. Sweep them, and pin the real
  wordings with `--oracle-differs` or `--no-oracle-only` cases.

## Open verification

- `mooshimeter-dmm` on a real meter: the per-reading message order, the
  DEPTH/RATE period and the buffers' full scale are the model's reading of
  the vendor's code; radio timing, pairing and more than one link are not
  modelled.

- `hung-chang-dso-2100` on a scope and a real cable: the line decoding, the
  waveform, the interleaved timing and the post-trigger fill time are the
  model's reading of C, and sigrok-cli prints no sample to compare with.

- `hp-3478a` on a real meter and adapter: the reading rate, the 4.5 and 3.5
  digit readings, `F7`'s range code, and a real adapter's timeouts (the
  kernel rounds them up to the adapter's next step).

- `asix-omega-rtm-cli` with a real OMEGA and ASIX's `omegartmcli`, natively
  and under wine: the program's real timing, its stdin behaviour under wine,
  and a release build's throughput at 200 MHz.

- `baylibre-acme` on an ACME cape, and on current kernels: the fixed GPIO
  numbers 474-511 and the legacy `/sys/class/gpio` don't exist there; the
  GPIO character device would be needed.

- `scpi-dmm` on hardware: a 34401A or 34465A for the wait on one USBTMC
  request at slow integration times, and a GW Instek or OWON for dialects
  written from the driver's tables alone.

- `hameg-hmo` on any real HMO or RTB, above all the RTB/RTM/RTA/RTH
  `:LOGn`/`:DIGn` commands and `:PODn:DATA?`: the RTB2000 manual has
  `DIGital<m>:DISPlay` for a channel where C sends `:LOGn:STAT`; fix the model
  and the port once a real RTB answers.

- `scpi-pps`: the answer forms and the HP 66xxB sweep queries against a real
  supply or a recording.
- `lecroy-xstream`: whether a real X-Stream sends `ALL,`, what its firmware
  does with a second USB request, and keepalive ending a dead TCP peer.

- `rigol-dg` on a real DG1000Z: the answers to `X? MIN|MAX` for amplitude,
  offset and square duty. On a DG800/DG900: whether `FUNC` and the MIN/MAX
  queries behave as on the DG1000Z, and channel 2 with the counter on. The
  MSO5000 generator's syntax.
- `siglent-sds` and `siglent-sdl10x0`: a recording from a real unit. The
  SDL's USB id, and what its USBTMC does with INITIATE_CLEAR (and whether it
  needs the bulk OUT halt cleared after, which `Transport` has no call for).
  The SDS E series' second frame, and multi-frame `ARM` on the others.
- `yokogawa-dlm`: the DL9000 series' commands, USB id and channel 4/logic
  sharing need a manual or a recording.

- `hp-3457a` and `hp-59306a` behind a TCP bridge with real HP-IB: untried.
  For the 3457A, whether a measurement holds up `ID?`, and what `PRESET`
  does to the terminals and the function.

- `beaglelogic`: emulated over its TCP server only. `/dev/beaglelogic` is not
  ported (ioctls, `mmap`, `poll`; needs unsafe code or a crate, and the kernel
  module or a CUSE fake to test on). No run on a board: the busy window after
  a capture, the 100 MHz divisor rule and the server's stream end are read
  from source.

Hardware checks not yet done. For the emulated USB drivers, a capture
next to sigrok-cli's, or a recording (`OXROK_USB_RECORD=FILE`), is the
most useful report; `tools/diffcli.sh --hw` runs the recorded cases when
the device is attached, and `oxrok-hil` runs the LogicPort bench.

- The ported USB logic analyzers other than `fx2lafw`, the LogicPort and
  `raspberrypi-pico` have met only the emulator. `asix-sigma`'s last
  event and the Pipistrello's marker table and 200 MHz trigger are
  written down in DEVICES.md and not run.
- Every other emulated USB driver (the USB scopes and the serial, HID
  and SCPI meters, supplies and loads) has met only the model;
  `tools/diffcli.sh --hw` is the path when a unit is available.
- `fx2lafw` on the Saleae clone again: the upload moved into the scan
  and the open order changed (version and REVID before the claim); a
  cold start after a power cycle on macOS covers renumeration. The nusb
  claim before the first control transfer has not run on Windows.
  Triggers and timing against the signal generator need a wire from the
  ESP32.
- `rigol-ds` on the other series (DS2000, DS1000Z, DS4000, MSO5000, the
  Agilent DSO1000): the DS1102E is verified; each other series needs an
  `OXROK_SCPI_RECORD` recording and a sigrok-cli comparison.
- CP2110: the host emulator only. A real cable would settle which parity
  codes the chip follows and whether it honours the purge as an output
  report. The BU-86X, CH9325 and Victor paths have met the Pico and the
  host emulator.
- A real PICkit2 and a real relay board: both drivers have only met the
  Pico emulator.
- Real multimeters: the serial meters have met ESP32 replays of UT61E
  and UT60A streams, the HID ones the emulator, no meter yet.
- `raspberrypi-pico`: digital, analog and mixed captures are verified
  against sigrok-cli, but a real signal on the inputs (a wire from the
  ESP32 to GPIO 2) and the larger digital-only firmware variants are
  not.
- LogicPort: the clock qualifier input (which input gates the clock),
  the 2.5 ns `before`/`after` sample point (needs a phase-locked source
  with adjustable delay), the duration correction at 100 MHz and
  500 MHz DDR, inversion with the clock edge, counter source 1
  (`D0=1*2` on the pulse train), and the frequency counter above 1 MHz
  (up to the vendor's 300 MHz).

## Emulation plan

Most drivers cannot be run against their hardware here. A driver is
ported against a model of the device and checked with sigrok-cli as the
oracle through Linux's USB/IP `vhci-hcd`; [USB.md](USB.md) describes the
server and the harness. A driver is emulator-verified when:

1. sigrok-cli and oxrok produce identical output for the scenario list
   in `diffcli.sh`, after the header line.
2. The model's pedantic mode logged no violation under either tool. A
   clean run under sigrok-cli validates the model; one under oxrok
   validates the port.
3. The transaction logs of the two runs are identical after the fixture
   layer's normalisation.
4. The gcov report of the C driver, from a libsigrok built with
   `--coverage`, is attached to the driver's entry in
   [DEVICES.md](DEVICES.md), with the unreached paths named as "not
   emulated" on purpose.
5. A replay fixture is recorded from the emulator run, so `cargo test`
   needs neither the model nor root.
6. `diffcli.sh --hw` runs the same scenarios against the real device
   with one command, so a user who has it can promote the driver to
   hardware-verified.

What this cannot prove: a model built from the same C source can share
its misreading; quirks the C driver relies on without saying so cannot
be found; and timing, lost samples at speed, short packet framing and
real analog front ends need hardware. Device firmware sources outrank a
model's guesses where they exist.
