# Roadmap

Status: 2026-11-22. `v0.2.0` is the stable release and `v0.3.0-alpha.1`
the newest pre-release, both on GitHub and crates.io. All 87 of
libsigrok's `src/hardware` drivers are ported. Native decoder porting is
complete: all 111 decoders of libsigrokdecode 0.5.3 are native.

The work that remains is the core's shutdown, transports, packaging,
the Pico backend, and the emulator, CLI and hardware-verification gaps
listed here. [Next](#next) ranks what comes first and
[0.3.0 stable blockers](#030-stable-blockers) says what the next stable
release waits for; everything after them is the backlog by area, in no
order. Completed phases and per-batch history have been removed; the
evidence per driver is in [DEVICES.md](DEVICES.md) and the decoder
checklist in [NATIVE_DECODERS.md](NATIVE_DECODERS.md).

## Next

In this order. Each item stands for entries further down, which have
the detail, and leaves this list when they are gone.

1. A shutdown that cannot hang: everything under [Shutdown](#shutdown).
   Done when a stop and a close return in bounded time for every driver
   on a queue nobody reads, with a test over the registry that keeps a
   new driver from bringing the hang back, and when the CLI closes its
   device on every exit.
2. The software cases of `diffcli.sh` checked by default again (the
   last entry of [Harness and CI](#harness-and-ci)): the demo driver,
   the formats and the decoders on the command line have had no check
   outside `--oracle` since the goldens replaced the oracle. Done when
   each has a golden or a `cargo test` that covers it, and CI runs them.
3. An emulator tier that gates a merge ([Harness and CI](#harness-and-ci):
   the vusb `CMD_UNLINK` race, then the `vhci-hcd` skip). Done when an
   emulator run that was selected and failed, or did not happen, blocks
   the pull request, and the race no longer makes that a coin toss.
4. A release that cannot go out half done
   ([Packaging and release](#packaging-and-release), its last entry),
   and with it a stable 0.3.0.
5. A way in for hardware reports (the head of
   [Open verification](#open-verification)): 80 of the 90 entries of
   `docs/drivers.toml` have met only the emulator, 6 a real device. Done
   when a user with a device can file a report that has what a
   maintainer needs, and it is labelled and found.

## 0.3.0 stable blockers

A proposal: what a stable 0.3.0 should not ship without. Everything
else on this page can follow it.

- Items 1 and 4 of [Next](#next): a capture whose close can hang, and a
  release flow that publishes before it has built, are not stable.
- `oxla`'s emulator cases have run on Linux and its goldens are read
  ([Models and coverage](#models-and-coverage)): 0.3.0 is the release
  that brings the driver.
- Item 2 of [Next](#next), at least for the demo driver and the
  outputs: the part of the command line every user meets.
- `Serial::read_data` drops a CR from a binary block
  ([Driver behaviour](#driver-behaviour)): `rigol-ds` on a serial port
  loses a sample byte today.
- A decision on the breaking changes under [Structure](#structure) (the
  `Device` trait's channel defaults, the typed failure kind): in before
  0.3.0 is stable, or kept for 0.4.

## Core and library

### Shutdown

A stop and a close have to return, whoever reads the queue and however
the process ends. These are one piece of work:

- About 35 drivers block in `sink.send` or `sink.end` while close joins
  them; a full queue nobody reads hangs the close. `Session` always reads
  its queue, so this bites a direct `Device` user and the drivers' own
  unit tests (one hung a CI runner for an hour). The bounded `send_end`
  of `gmc-mh-1x-2x`, `appa-55ii`, `beaglelogic`, `atorch` and
  `atten-pps3xxx` fits the rest; one by one or through a shared
  `PacketSink` change is undecided.
- `Session`'s `End` is sent before the link goes back in its slot, so
  `start()` right after `End` can get `Busy` (`lecroy-logicstudio`,
  `saleae-logic16`, `kingst-la2016`, `hantek-6xxx`, `sysclk-lwla`). The
  fixed order of `sysclk-sla5032`, or a busy flag cleared before the End
  as in `rdtech-dps`, fits each of them.
- A driver whose acquisition ends on its limit is only stopped (its
  thread joined) by the MCP server's run thread; the CLI runs one
  capture per process and never needed it. `Session::run` could stop
  the devices itself when every `End` came, so every caller gets an
  idle device; a driver whose `stop` writes to the device would then
  write once more, which its goldens pin.
- The CLI closes a device only on its happy path, and `ctrlc` is built
  without `termination`: a driver that writes at close (`rdtech-dps`'s
  panel lock) needs its own `Drop`. Close on every exit.
- SIGINT doesn't end a run whose stdout is blocked: the `ctrlc` handler
  only sets a flag (`src/cli/run.rs`) while the main thread waits in the
  write. `oxrok --continuous -O binary | (sleep 8; ...)` then SIGINT:
  still running after 10 s, and the driver's `close` never goes out.
- Issue #82: on macOS nextest hung for 67 minutes in
  `atten_pps3xxx::tests::a_capture_with_something_to_say_notes_it`. That
  driver bounds its `End`, so it is another wait: the stop flag against
  the fake port's read is the suspect.

### Structure

How the library is put together. None of it is a bug a user meets; the
first entry comes before the others.

- Rust module paths are part of the goldens: 69 files under `tests/emu`
  hold log targets, as in `oxrok: err: oxrok::drivers::bkprecision_1856d:
  unable to send gate time command`, so a `log::` call that moves to
  another module changes stderr a golden pins. Give the drivers an
  explicit `target:` equal to the driver id, which is also nearer
  libsigrok's `LOG_PREFIX`, and re-record those goldens once, before
  any entry below moves code.
- The acquisition thread is copied per driver: `struct Worker { stop,
  handle }` in 79 driver files, `fn send_end` in 35, `END_AFTER_STOP`
  33 times (500 ms; 1 s in `atorch`, `pce_322a` and `zketech_ebd_usb`),
  a `Limits` of atomics 20 times, and 26 drivers still call
  `is_finished()` in `start`, which `DRIVER_PORTING.md` warns against.
  A `core::acquire` with a `Worker` (the port out of its slot, the
  `Busy` check, the named thread, the port back, the bounded `End`) and
  an `SwLimits` after libsigrok's `sr_sw_limits` would hold it once.
  Each driver checks its limits and gives its port back where the C
  driver does, so one shape will not fit all: family by family, the
  serial meters first, with the emulator corpus as the gate.
- A poisoned lock has no policy: 395 `lock().expect(..)` in the drivers
  against about 9 that recover. A worker that panics is reported by
  `Session::run` and the MCP server forgets its handle, but a later
  `get`, `set` or `close` panics in turn, and six drivers' `Drop` call
  a `close` that takes the lock (`rdtech_dps`, `maynuo_m97`,
  `itech_it8500`, `link_mso19`, `cem_dt_885x`, `siglent_sdl10x0`),
  which aborts the process when it happens during an unwind. Recover in
  `close`, `Drop` and `stop`; in `get`, `set` and `start` a poisoned
  lock is an `Err`, not a recovery, since a supply's link may be half
  updated. One type (a `core::sync::Shared`) rather than 395 call
  sites, and a test that a device whose worker panicked still closes.
- `Device::enable_channel` and `rename_channel` are required and
  written 88 times, about 80 of them the same three lines with two
  wordings of one error (`No channel {pos}.` and `no such channel`);
  `fn no_group` is in 12 drivers and the `conn`/`serialcomm` scan loop
  in every serial one. Defaults over a `channels_mut`, an
  `Error::no_channel` and `unknown_key`, and a `SerialScan::parse`
  would take 800 to 1,000 lines out. The drivers that react to an
  enable keep their override; one missed loses its side effect with
  only an emulator case that toggles channels to notice. A new required
  method breaks the trait.
- A failed capture loses its kind: `PacketSink::end` sends
  `Packet::Failed(e.to_string())`, `Session::run` returns
  `Error::CaptureFailed(String)` and the MCP server keeps the text, so
  a consumer cannot tell a disconnect from a timeout. "device is not
  open" is built as a string 112 times and `invalid value for KEY` 89.
  Wanted: typed `NotOpen` and `InvalidValue` variants and a
  `FailureKind` (io, disconnected, timeout, firmware, device, other)
  carried by `Failed` and `CaptureFailed`, with `Display` unchanged so
  that no golden moves. It breaks every match on the two, and helps
  only as far as drivers return typed errors: many build
  `Error::device(format!(..))` for a disconnect.
- Every build of the library compiles 191,000 lines of drivers and
  54,000 of decoders with their dependencies (`aes` for one driver,
  `serialport`, `async-hid`): the features are `cli`,
  `firmware-download`, `mcp`, `nusb`, `libusb`, `python` and
  `rustpython` only. Group features (`drivers`, `decoders`,
  `transport-serial`, `transport-hid`), all in `default`, would let a
  library user leave them out. It is more than a few `cfg` lines
  (`mcp`, `setup` and the driver support modules name the drivers), it
  saves CI nothing, and each feature is one more combination to keep
  building: when a user asks for it.

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
  (presumably GitHub's hosted runners), so CI gives it no coverage; the
  self-hosted runner under [Harness and CI](#harness-and-ci), with
  `hci_vhci` and no bluetoothd, would run it.

### GPIB

`hp-3478a` runs over `src/gpib` (`conn=gpib/MINOR/PAD`) on the kernel's
`gpib` subsystem; only `ni_usb_b` is modelled. Open:

- `libgpib/NAME` resources, read from `/etc/gpib.conf` as libgpib's
  `ibfind` does.
- `agilent_82357a` and `lpvo_usb_gpib` models.
- Other SCPI drivers over `conn=gpib/...`: the transport is shared, but no
  case covers one.
- A CI runner without `ni_usb_gpib` skips the model by name with a warning
  and runs none of its cases; the self-hosted runner under
  [Harness and CI](#harness-and-ci) would run them.
- Waiting on SRQ (`IBWAIT`) instead of the 100 ms serial poll.
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

- EPEL 10 has no `libsigrokdecode`, so the COPR package for RHEL and
  CentOS Stream 10 runs without the Python decoders unless the user
  finds them elsewhere: package `libsigrokdecode` for EPEL, or ship the
  bundled decoders in the RPM there.
- A Homebrew formula or tap.
- An AUR `PKGBUILD`.

- Releases are cut by hand, from memory: bump, tag, publish the GitHub
  release, `cargo publish`. `release.yml` runs on `release: published`,
  so a release is public before its assets exist (54 minutes for
  0.3.0-alpha.1) and a failed target leaves it half filled. Nothing
  compares the tag with `Cargo.toml`'s version, which names the
  archives, or asks for a `CHANGELOG.md` section. Wanted: a written
  procedure, a tag, version and changelog check before anything is
  built, a draft that is published only when every asset is on it, and
  crates.io published from the same workflow through trusted publishing
  (OIDC, no stored token), with a decision on what happens when that
  publish fails after the binaries are out: a crates.io publish cannot
  be undone.

Static binaries, the release archives, `make dist`, notices and
`cargo publish` already work.

## Firmware fetching

`oxrok firmware` and the `oxrok-firmware-fetch` crate (`firmware/` in
oxrok-util) fetch, import and list vendor firmware; every downloadable
package is checked end to end by its `cargo test --test network --
--ignored`, which reproduces sigrok-util's outputs byte for byte.

- LeCroy LogicStudio: the extractors (`LS16Lib.dll`, and the FX2LP
  firmware from `LogicStudio16Load.spt`) are checked on synthetic input
  only. The installer is behind a Teledyne LeCroy account; once one is
  approved, import the 1.2.5.0 zip, check the outputs against the sigrok
  wiki's MD5s, and see whether the zip opens without an external
  unpacker.
- Sysclk SLA5032: `top.bit` from the vendor ISO
  (`sla5032_2015_1_24.iso`) has not been tried; the package has no known
  hash yet. Sysclk LWLA1034 from the CD installer and LWLA1016 from an
  installed copy: the offsets are sigrok-util's, never run on the real
  files (no copy found online).
- Saleae Logic 1.1.x: the path that rebuilds the FX2 firmware from x86
  code is not ported; 1.2.10 is pinned instead. 1.2.18 has no firmware.
- The LWLA1016 downloads come from a user's Google Drive folder (the
  blog.danman.eu post); find a sturdier source, or SysCLK's permission
  for fixed bitstreams in oxrok-firmware (sigrok bug 760).
- The MCP server reports each file's package but cannot fetch; add a
  `firmware_fetch` tool behind `--read-only`, and the UI's prompt.
- Drivers' `FirmwareDependency::source` texts still name sigrok-util;
  point them at the packages, or derive them from the registry.
- `fx2lafw-instrustar-isds205b.fw` and `fx2lafw-usb-c-grok.fw` are in no
  fx2lafw release; fetch them once one ships, or build them in CI
  (`firmware-build` above).

## MCP server

An MCP server so an agent can scan, capture, measure and decode through
oxrok: `oxrok mcp` over stdio ([MCP.md](MCP.md) documents it), and an
extended server in oxrok-ui whose tools the user picks.
[plans/mcp-ui.md](plans/mcp-ui.md) has the plan for the UI's server, its
later steps and the open decisions.

- `driver_info` does not yet report a driver's verification status
  (hardware, stand-in, emulated, replay) from `docs/drivers.toml`: the
  docs are not in the published crate, so the table would have to be
  generated into the source at build time or kept in `tools/site-data`.
- A `guess_signals` tool: rank a capture's logic channels as clock,
  data, select or idle from the features `measure` reports (regular
  period, pulse widths that are multiples of another channel's period,
  edges only while another channel is low), try the common two and
  three wire decoders on the candidates, and score each by frames
  decoded and the warnings row. An agent does this by hand today over
  `measure`, `capture_logic` and `capture_decode`.
- The UI server over a local socket, with a per-tool checklist. Its
  first piece is here: the data tools read the `Host`'s own
  `CaptureStore`, and need a narrow trait (or a store a front end can
  back) before they can serve the UI's sessions.

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
- `oxla`'s cases have never run: the model (`emu/src/models/oxla.rs`) and
  the scenario (`tools/emu/oxla.sh`, 138 cases, all `--no-oracle-only`)
  were written on macOS, where the harness does not run. Until
  `tests/emu/oxla` exists the scenario skips itself by name, so that the
  emulator tier does not fail for goldens there are none of. Run
  `EMU_BLESS=missing sh tools/diffcli.sh --emu-model oxla` on Linux, read
  each golden and fix the cases that guessed wrong (the stderr texts, the
  cases with `--timeout`), then name the model in `docs/drivers.toml`
  (`emu = ["oxla"]`) and say so in `docs/DEVICES.md` and the README. Its
  board's bridge shows as an FTDI `0403:6010`, so it too is on a CDC-ACM
  stand-in until there is an FTDI shell for serial models.
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

- The trigger benchmark (`examples/trigger_bench.rs`) runs by hand
  only: shared runners are too noisy for a throughput floor. A quiet
  runner, or a ratio against a baseline loop measured in the same run,
  would let CI hold the software triggers above fx2lafw's 24 MS/s.

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
  and a skipped required job counts as satisfied, which is why the
  emulator jobs are not required checks at all (oxrok/management,
  `docs/HARDENING.md`, "Known gaps"): an emulator regression can merge.
  A setup that guarantees the module, or failing instead of skipping on
  a push to `main`, would close the skip. One aggregate job
  (`if: always()`, needing every other job, red when a selected
  emulator run failed or did not happen) as the only required check
  would make the tier gate, and let the Python slices go back to a
  job-level `if`. The required list is oxrok/management's
  `config.toml`, so it changes there and in `ci.yml` together, and not
  before the vusb race below is fixed: it would block about one pull
  request in three that selects a serial model.
- A self-hosted Linux runner would run what the hosted ones cannot: the
  BLE model (`hci_vhci`), `hp-3478a` (`ni_usb_gpib`), a `vhci-hcd` that
  is always there, and the libusb run below. On a public repository it
  may run only what is on `main` (a schedule or a manual run, never a
  pull request), and has to fit oxrok/management's hardening rules.
- The libusb backend has no regular emulator run; the gates use nusb.
  Wanted: a regular `OXROK_USB_BACKEND=libusb` run, comparing output
  only.
- Several emulator sessions at once are not isolated: libusb lists every
  device and drivers match by VID:PID, and cleanup, port choice and gcov
  counts are global. Test one tool per namespace whose `/dev/bus/usb`
  holds only its own node. CI's emulator time waits for this too: a
  shard runs its 1,600 cases one after another under one host-wide lock
  (`EMU_LOCK`). Three sessions at once on a 4-vCPU runner need a lock
  per port, a firmware stand-in directory and an `oxrok-emu cleanup
  --locked` that are not host-wide, and a serial tail for the models
  with global state (`hci_vhci`, the parport, sysfs). The cases are
  sensitive to timing under one session already.
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
  writing hangs `close()`. The drivers hold 64 hand-written
  `impl Port` fakes and share none: a `ScriptedPort` (expect these
  bytes, reply, stay silent for a time, hang up) with the write log
  built in would replace them, these 18 first.
- Three scenarios carry copies of the lock preload (`gwinstek-gpd`,
  `tondaj-sl-814`, and a larger one in `devantech-eth008`): make one
  shared file and helper.
- `fn shots/hit` is duplicated in five models.
- A vusb race is the main cause of a red `main`: the kernel can send
  `CMD_UNLINK` ahead of its `CMD_SUBMIT`, and the late `RET_SUBMIT`
  makes `vhci_hcd` drop the connection (`cannot find a urb of seqnum`
  in dmesg). The device is gone, the harness says `no tty for 1-1`, and
  every later case of the session fails with `could not put the tty
  back`: 5 to 112 cases, in 10 of 18 red runs of `main` within a week
  (issues #89 and #105), most often first in an unplug case of
  `gwinstek-psp`, `cem-dt-885x` or `mastech-ms6514`. It was put at once
  in about 11,100 attaches; on the hosted runners it is far more
  frequent. The fix is in vusb (CI pins vusb-cli 0.2.1): remember an
  unlink above the highest submitted seqnum and drop that submit. Only
  if it stays after that: `diffcli.sh` starts the session again and
  runs the case once more, counted and shown as a retry, and only when
  the kernel's line is there, since a device that leaves in an unplug
  case can be oxrok's or the model's doing.
- `serve --attach`'s "sudo modprobe vhci-hcd" hint on Windows is
  Linux-specific.
- CI's Python decoder slices are floored by one test:
  `decoder_uart_rest::amulet_ascii_regression` runs over four minutes
  under CPython. Shorten its input or find why it is slow, then retune
  the slice counts in `ci.yml`. Those slices also re-run the whole
  `oxrok` suite, most of which the `test` job already runs; a nextest
  `-E` filter to the decoder tests and the Python-gated code would
  halve them. With the filter in, one job per feature would do, which
  frees three of the 20 job slots on every pull request; the slice
  names are required checks, kept in oxrok/management's `config.toml`,
  so the ruleset changes in the same step. The RustPython slices now
  really decode, and how long they take in CI is not known: 15 to 30
  minutes each is the guess, against a job limit of 45. Under
  RustPython the amulet test took 508 s on a loaded Mac, so it has a
  30-minute limit of its own in `.config/nextest.toml`, which a shorter
  capture would make unnecessary.
- `tools/ci/changes.py` balances emulator shards by golden case count,
  a rough proxy (a 12-minute and a 22-minute shard had 1339 and 1789
  cases). `diffcli.sh` printing each model's elapsed time would allow
  real weights. A submodule of `src/core`, `src/serial`, `src/usb`,
  `src/cli` or `src/formats` selects only the drivers that name it
  when it can tell (`core/softtrig.rs`); the rest still select
  everything, mostly because `cli` or a sibling uses them.
  `serial/tcpraw.rs` is one: `serial::open` hands it any `tcp-raw/`
  port, so it could select the TCP drivers plus the models served
  with `EMU_TCP`, if the replay tests cover the other drivers' use.
- Nothing checks the goldens against libsigrok again: `EMU_BLESS=1`
  without `--oracle` re-records them from oxrok, CI runs no oracle, and
  the oracle's build (libsigrok 0bc24877 with the four patches of
  `tools/oracle-patches`, a pinned libserialport, a patched
  libsigrokdecode and sigrok-cli f44dd913, on Fedora) is prose in
  `DEVICES.md`. Wanted: that build as a container file, and a scheduled
  workflow, sharded as the emulator job is, that runs
  `diffcli.sh --oracle` and says when a golden no longer matches. It
  could run `--gcov` as well, which is the refresh the next entry asks
  for. Expect noise at first: sigrok-cli's timing on shared runners,
  `hp-3478a`'s libgpib, the parport numbers.
- The gcov figures of the 48 models that lost their library-only cases
  (atten-pps3xxx, kecheng-kc-330b, lecroy-logicstudio, sysclk-sla5032,
  greatfet, uni-t-ut181a and others) still count those cases: their
  `tests/emu/MODEL/gcov.txt` and the percentages in `docs/DEVICES.md`
  (labelled there as from the last `--oracle --gcov` run, which still
  had them) predate the deletion. The next `--oracle --gcov` run of each
  model lowers them; refresh them together, drop the label, and say
  which C paths no case reaches any more (stops from a client, the
  no-limit crashes, the waits that never end).
- Two advisories are ignored in `deny.toml`. chrono is flagged
  unmaintained (RUSTSEC-2026-0543) and is a direct dependency
  (`chrono = "0.4.45"`, for the dates the csv and vcd outputs write):
  move to jiff or time and delete the line. paste (RUSTSEC-2024-0436,
  unmaintained) comes through RustPython 0.6.0 and malachite-bigint:
  look again at the next RustPython bump.
- The `test` job on the three systems passes all 286 native against
  Python parity tests by returning early, since no decoders are
  installed there. Say so in the job, or commit reference outputs for
  the 112 decoder captures, so the native decoders are checked without
  Python on every OS. Where decoders are required
  (`OXROK_REQUIRE_DECODERS=1`, the Python jobs), fourteen tests in 12
  files still return early when the one Python decoder they name is
  missing from a tree that has others: Ubuntu's `libsigrokdecode4` may
  lack some the native side has.
- Both Python features compare against the distribution's
  `libsigrokdecode4`, not the 0.5.3 tree `make decoders` pins and a
  release ships: have the Python jobs use the pinned tree.
- The vendor-firmware guard (`tools/ci/firmware.py`) goes by file name
  only. Still open: a size limit on non-text files under `tests/` and
  `tools/emu`; the same check on the release's source tarball; a
  denylist of hashes, which would have to be built (oxrok-util's
  extractors know the files); a line in AGENTS.md that names the
  check. `.gitignore` has the same extensions for the whole tree, so a
  rightful `.hex` or `.bit` needs `git add -f` and then fails the
  guard: the guard would need a list of exceptions.
- The toolchain is pinned in `rust-toolchain.toml` and moved by
  Dependabot, so each Rust release is a pull request whose new clippy
  lints are fixed by hand. `release.yml` installs the toolchain the
  same way and is not run by a pull request: run it by hand
  (`workflow_dispatch`) after such a bump, and once before the next
  tag, since `rustup toolchain install` with no name needs rustup 1.28
  and its arm and Intel macOS runners have not been through it.
- The daily advisories workflow, the `rust-toolchain` Dependabot entry
  and any change to Dependabot's grouping are not in the org's baseline
  (oxrok/management, `docs/HARDENING.md`: "weekly, grouped"): record
  them there or apply them to the other repositories. Dependabot
  security updates are still off in the repository's settings. A
  scheduled run that fails only sends a mail: decide whether a failed
  advisories run opens an issue (management's ci-doctor may be the
  place).
- `tests/emu` is 21,989 of the 23,160 tracked files, with no
  `.gitattributes` rule and no check that runs without the emulator.
  Mark it `linguist-generated=true` (not `-diff`: goldens are there to
  be read) and `-text`, as `tests/fixtures` is. A static check in the
  `fmt and clippy` job, over the cases of each scenario as the
  harness's dry run lists them (`EMU_DRY`): a golden no case has, a
  case with no golden (run 44313467058 failed on 138 of `oxla`'s), an
  `--oracle-bug` case with an empty `.oracle`. Not an empty `.out`:
  1,854 are empty by right.
- `tests/` is 48 files, each a binary linked against the whole library,
  and `--all-features` builds both pyo3 and RustPython in every test
  job. On Windows the nextest step is 9 minutes of build and link
  against 4 of tests; on macOS the tests are the larger part (507 s
  against Linux's 257 s). Measure with `cargo build --timings`, then
  merge the `*_replay.rs` and `*_cli.rs` groups into a few binaries.
  The `decoder_*.rs` files keep their names: `tools/ci/changes.py` and
  the nextest filter of the Python slices go by them. A split into
  crates would change the log targets the goldens pin
  ([Structure](#structure)).
- The emulator cases run on Linux only. A third carrier beside USB/IP
  and replay, a `usb::Transport` and a `serial::Port` that call the
  `DeviceModel` directly, would run them in `cargo test` on every OS.
  The `.tx` goldens record what the kernel carrier did (how URBs split,
  unbinds, renumeration), so it needs a normalisation or goldens of its
  own; the 30 s cases are oxrok's real timeouts and stay; and it goes
  around nusb, usbfs and the tty layer, which the tier is there to
  test. The small step first: one nextest test per fixture in
  `tests/emu_replay.rs`, in place of one test that starts 65 processes
  at once.
- A failed emulator shard keeps its models' transaction logs, a dmesg
  tail and the times (`emu-failed-N`, 7 days), not the `.out` and `.tx`
  each failed case produced; that needs an option of the harness at its
  twenty or so comparison sites. A failure counted before a model's
  session starts is not put down to that model, so its log is not kept
  when the session then passes. The harness step has 40 of the job's 45
  minutes: on a cold cache a hang can meet the job's limit first, and
  then nothing is kept.
- An apt outage of more than about a minute still fails the Python and
  emulator jobs (`tools/ci/apt.sh` tries three times; the outage of
  2026-11-11 lasted 42 minutes). Caching the two `.deb` files of the
  Python jobs is what is left, with a cache that goes stale as its
  price. The `vhci-hcd` probe still turns a failed apt into a skipped
  tier.
- Only `main` saves the Rust cache, so a pull request that changes
  `Cargo.lock` (every Dependabot one) builds the changed dependencies
  on each push: watch whether that costs more than the cache quota it
  saves.
- The bench runs by hand: no workflow has a schedule for it,
  `oxrok-hil` prints PASS, FAIL or UNVERIFIED to stdout and knows only
  the LogicPort, and nothing builds `hil/esp32-siggen` (PlatformIO
  C++; `hil/pico-hidcable` is to go, see [Pico backend](#pico-backend)).
  Wanted: a hosted job that builds the ESP32 firmware when `hil/`
  changes; a JSON or JUnit report from `oxrok-hil`, with the device's
  serial number and firmware, so the evidence in `docs/drivers.toml`
  gets a date; and on the self-hosted runner above a scheduled
  `diffcli.sh --hw`, which needs the oracle's build there, with a
  device that is not attached reported as absent, not as failed. The
  bench is not ready to run unattended ([Test machines](#test-machines)),
  and the Pico is reflashed between roles.
- The repository's settings are not what oxrok/management's
  `docs/HARDENING.md` says, and nothing enforces AGENTS.md's "no direct
  push, by anyone": the one live ruleset has no pull request rule and
  an organization admin bypass (edited by hand on 2026-11-14), there is
  no review ruleset, and rebase merges and auto-merge are on. Apply the
  rulesets again (`tools/rulesets.py apply`, there) or write the change
  down, and check for drift on a schedule.
- A merge queue for `main`. A push to `main` runs everything (199
  runner-minutes, 48 times since 2026-11-08), which is the only full
  run of a pull request whose jobs were selected by path. A queue
  (`merge_group` in `ci.yml`, a push to `changes.py`) tests the merged
  commit before it lands. It allows one merge method, where batch pull
  requests use `merge` and others squash, costs a 25-minute run per
  entry, and throws a batch out on a flake: after the aggregate check,
  not before.
- The pull request template's boxes are not checked by anything. Of
  the last 31 merges that touched `src/`, 24 did not touch
  `CHANGELOG.md`, so a check that fails would fail most of them: a
  warning from the `changes` job is the most that fits.
  `tools/attribution.py --check` in CI needs full clones of libsigrok
  and sigrok-cli, which no job has.
- `docs/DEVICES.md` (4,135 lines, 91 sections in porting order, no
  index), `docs/drivers.toml`, the README's table and
  `tests/emu/*/gcov.txt` hold the same case counts and gcov figures by
  hand: two commits rewrote 189 and 92 lines of `DEVICES.md` for one
  change to the harness. Wanted: an index by driver id, and each
  section's status line generated from `drivers.toml` and `tests/emu`
  and checked as `MCP.md`'s tool list is. `tools/site-data`'s check of
  the libsigrok names runs nowhere in this repository.
- The software cases in `diffcli.sh` (`case_`, about 70 lines: the demo
  driver, input and output modules, the `.sr`/`.vcd` round trips, the
  decoders, 9 of them only with the python feature) compare oxrok with
  sigrok-cli live and have no goldens, so they now run only under
  `--oracle` and nothing checks them by default. Give them goldens (a
  `tests/cli/` directory compared like the emulator's), or confirm that
  `cargo test` covers each, then run them in the default mode and in CI.

### Test machines

Notes about the testers, kept here until they have a place in
oxrok/management.

- usbtester4 has GPIB minor 0 left configured by an early `gpib-config`
  with no marker: take it offline by hand (or reboot).
- `uss720` keeps a detached cable's parport number (one per emulator
  session on kernel 7.2.x), so after 8 sessions on a tester libieee1284
  finds no ppdev and the `hung-chang-dso-2100` oracle stops; the harness
  fails such a session and says so. Unloading `parport` doesn't help (the
  new `parport0` collides in sysfs); only a reboot does. Testers need a
  reboot between batches, or the harness a way to reset the numbering.
- Pulling a `uss720` cable while the port is claimed crashed usbtester3's
  kernel, and usbtester3 did not come back from its reboot: no unplug case
  for `hung-chang-dso-2100` until that is understood.

### Flakes and test gaps

- The flakes CI has met are issues, not entries here: #77 (`greatfet`
  "unplug at start" passes on rerun), #88 (`lecroy-logicstudio` "lib
  bitstream missing" prints an extra firmware line), #89 and #105 (a
  lost tty, the vusb race under [Harness and CI](#harness-and-ci)) and
  #82 (under [Shutdown](#shutdown)). oxrok/management's `flakes.toml`
  knows only #77 and #105. One place should hold them all, with the
  others pointing at it.

- The `test` and Python decoder jobs retry a failed test once and name
  each test that passed on its second try (FLAKY, in the job summary
  and in the `junit-*` artifacts). Nothing collects them: feed the
  JUnit files to oxrok/management's ci-doctor and `flakes.toml`, and
  add the tests CI has seen flake that this list lacks
  (`rigol_dg/tests.rs:1195` and `cem_dt_885x/tests.rs:318`, both on
  macOS). A retry is not a fix: each is a race to remove.
- Nine unit tests fail with every sleep at six times, a harsher test
  than CI's macOS runner: `gwinstek-gpd` (2), `atten-pps3xxx` (3),
  `rdtech-dps` (2), `rdtech-um` (1), `manson-hcs-3xxx` (1).
- `csv_matches_sigrok_cli` fails about 2 of 30 here with an extra row at
  5 µs, likely `dedup` flushing at a demo packet boundary. Under
  injected stalls: `motech_lps_30x::a_status_that_is_no_number_keeps_its_sample_only_with_no_expiry`
  about 1 in 3 at 150 ms; at 400 ms two `fluke_dmm`, `gwinstek`
  `the_second_half_of_a_line_end` and `korad` `a_long_gap`.
- The unit tests wait on the wall clock: 262 tests of a second or more
  are 666 s of the 981 s of test time on Linux, and the suite runs
  twice as long on macOS. A clock that a fake port advances
  (`Port::now` and `sleep`, or a `core::clock`) would test a 2 s
  silence bound in microseconds, for the serial drivers only (the USB
  ones are on `Transport`). It changes a public trait, and a test of a
  stop against a running worker has to stay on real time: those races
  are what such a test is for. `core::softtrig`'s
  `pulse_widths_with_or_and_and` takes 16 to 23 s on every OS and waits
  for nothing: find out why.
- `tcp_raw_cli::nothing_listening_is_no_device` failed once in a
  `norma-dmm` gate run and passed on rerun; a sibling's listener may
  have taken the freed port.
- Scenario filters hide oxrok's own text: the `bkprecision-1856d`
  review found `A` and `O` patterns hiding `: invalid value for ...` and
  `: device error: ...` after "Failed to open device"; motech, rdtech
  and others use the same patterns. Sweep them, and pin the real
  wordings with `--oracle-differs` or `--no-oracle-only` cases.

## CLI and format correctness

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
- `-O analog:digits=spec` is accepted and ignored:
  `src/formats/output/analog.rs` never reads it and `AnalogData` holds
  one digit count, so it shows the same digits as `all`. sigrok-cli
  prints `0.1 A` where oxrok prints `0.12 V`. `-O analog --show` also
  words `digits` differently from sigrok-cli. `-O ols` is absent, and
  `0x10`/`3.3V` are refused where sigrok-cli's `strtof` takes them.
- `-O csv` invents a value for an enabled channel that sends nothing:
  `fluke-45` with P2 enabled and silent, `--samples 3
  -O csv:header=false:time=false`, prints `1.0001,0`, a 0 nobody
  measured. Leave the cell empty, with a case.
- Several drivers narrow settings to `f32` where only sigrok-cli parses
  with `strtof` (`manson-hcs-3xxx`, `conrad-digi-35-cpu`, and more
  among the serial drivers). The narrowing belongs in the CLI's option
  parsing, not in the drivers.
- `docs/CLI.md` is one section of a thousand lines of differences;
  subsections by area (triggers, outputs, inputs, options, per driver)
  would make it a page to look things up in. No test says that a flag
  of `src/cli/args.rs` is in it or in `MCP.md`.
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

## Driver behaviour

- `demo`'s software trigger sends every sample of the triggering call
  before the trigger, on top of the pre-trigger buffer, where libsigrok
  caps the lot at the capture ratio's share (`pre_trigger_append`,
  `soft-trigger.c:83-107`): with `--samples 12` and the default ratio of
  20 it shows 4 samples before the trigger, libsigrok 2. Capping it
  changes the outputs `demo_cli` holds.
- Analog samples are `f32` in the core (`AnalogData.samples`): `rigol-dg`'s
  counter gives 10 digits and keeps about 7.
- A core output maps a logic channel to its bit by its listing index, so
  `yokogawa-dlm` needs `Device::channels_listed_as_they_are()`; a separate
  channel-to-bit map would let a driver keep C's index-32 channels.
- A VXI-11 transport, the Yokogawa DLM's Ethernet path.
- `oxla-tn20k` loads the FPGA and does not write the board's flash, so
  the gateware is gone at power off and loaded again at the next open.
  Writing it takes the SPI flash behind the GW2A's JTAG (instruction
  `16`, openFPGALoader's `Gowin::spi_put`) and a command that asks for
  it: nothing should replace what a board starts with unasked.
- `oxla-tn20k` asks a board what it runs on its UART, nine bytes that a
  board with another design gets at every scan. The FPGA's user code,
  read over JTAG, would tell a release's bitstream from anything else
  without a byte to the design; the gateware would have to set it.
- A front end can list the boards without a byte sent to any
  (`drivers::oxla::tn20k::boards`) and load one (`Board::load`); the UI
  does neither yet; `Board::variants` gives what there is to offer, from
  the descriptions installed. A device that still needs its load says so
  (`OxlaDevice::needs_load`), which the `Device` trait has no word for.
- `oxla` leaves parts of the device unused. Its hold-off (`PRE_WORDS`)
  counts words of memory, not samples, so `Trigger::holdoff` is refused:
  taking it rounded up to a word (2 or 4 samples raw, a run under RLE)
  needs a word on whether a hold-off may be longer than asked. Its event
  count on a two term trigger (the third A then B, the fifth A or B) has
  no place in `-t`, where a count belongs to a stage: a count on the
  combination would be new syntax, or a driver key (`Key::Custom`) beside
  `-t`. B then A is A then B with the stages swapped, and needs nothing.
- `oxla` fails a capture whose memory read loses a byte. The request
  could be made again: a byte sent during a readout ends it, so eight
  NOPs, a purge and the same `RMEM` would do, with a bound on the tries.
  An overflowed capture sends no samples though the device says the data
  up to there is good: what to send without a trigger position to cut
  around is undecided.
- `oxla`'s ADC bitstream (`la8-adc`) is eight logic channels here. The
  device says the channels are one analog value of eight bits; an analog
  channel in volts needs the ADC's range, which is the module's on the
  header and nothing reports: a device option would have to give it.
- `oxla` reads what a bitstream says of itself (the registers from `90`,
  from release 0.2.0 of the gateware) and uses its clocks and its lowest
  divider. The rest is read and kept, and not acted on: the channels that
  are analog, the trigger's terms, its count's width and its modes, the
  width of a run's count. A bitstream with three terms, or another count,
  is still driven as the document has it.
- `siglent-sds`: an E-series history mode as pull request 247 has it
  (`HSMD`, `FRAM`).
- The trigger hold-off (`Trigger::holdoff`) beyond `SoftTrigger`:
  - The `-t` grammar and the MCP `capture` tool have no word for it, so
    only a front end that builds a `Trigger` can ask for one.
  - `saleae-logic16`, `beaglelogic` and `sysclk-sla5032` run libsigrok's
    `SoftTriggerLogic`, which is armed from the first sample; `fx2lafw`
    switches to the general matcher for a hold-off, and they could too.
  - The hardware triggers: which devices fill their pre-trigger memory
    before they arm is not recorded per driver, and needs the bench.
- Pulse width triggers beyond `SoftTrigger`:
  - The LogicPort in hardware. Its trigger level has a counter with a low
    and a high bound (`count_low`, `count_high`, `counter_mode` in
    `logicport/protocol.rs`), so the FPGA may time a pulse; which counter
    mode does, and where the trigger point lands, needs the bench.
    `FEATURES` has `pulse: false` until then.
  - `beaglelogic` and `saleae_logic16` run `SoftTriggerLogic` only.
    `fx2lafw`'s split (libsigrok's matcher when the trigger fits it, the
    general one otherwise) would give them pulse widths, counts, ranges
    and or/and.
  - A width on a level stage (`D0=1<50us`, `D0-D2=3..5<10`): a pattern or
    a value held for less than a time, then left. `Trigger::check`
    refuses `Shorter` and `Between` without an edge today; the matcher's
    pulse state would serve.
  - `TriggerMatch::Edge` with a width (a pulse of either polarity).
  - oxrok-ui still scans the recorded data for its Positive and Negative
    Pulse triggers (`src/engine/trigger.rs`). They map to `Falling` or
    `Rising` plus `Between(min, max)` in samples, linked channels to
    levels, so the UI can read `trigger_features().pulse` and arm the
    device trigger on streaming devices.

- `hp-3457a`'s look at the line before each question, which catches an extra
  answer, has no counterpart on USBTMC.

- `ipdbg-la`: run-length compression (core id `idbg`, commands 0x10 and 0x60),
  as on the core maintainer's libsigrok branch; check it against a real core
  behind OpenOCD, including OpenOCD's one connection per core.
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

## Open verification

`docs/drivers.toml` has 90 entries: 6 have met a real device
(`intronix-logicport`, `fx2lafw`, `rigol-ds`, `raspberrypi-pico`, `oxla`,
`oxla-tn20k`), 3 a stand-in, 1 is software, and 80 have met only the
emulator. A report from a device's owner is the only thing that moves
those numbers, and the way in is thin:

- The hardware report issue form asks for a `hardware` label the
  repository does not have, so a report arrives unlabelled; no issue,
  open or closed, is one. The label is a repository setting (the owner,
  or oxrok/management). Issue #116, a user asking for a device with no
  form that fits, shows the demand.
- A report command (`oxrok report -d DRIVER`, say) that runs the scan,
  `--show` and a short capture with the recorders on
  (`OXROK_USB_RECORD` and the others) and writes one archive to attach,
  so the form asks for a file and not for commands typed by hand. A
  recording holds serial numbers and network addresses: the command has
  to say so, or strip them.
- A "wanted" list by device: the entries of `docs/drivers.toml` without
  a `hardware` status and their `limits`, generated into the site and
  `DEVICES.md`, so an owner finds their device by name. The entries
  below are prose a search does not reach.

What each driver still needs from a device:

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
  ESP32, including the counts, durations, value ranges and or/and that
  fx2lafw now runs through `SoftTrigger`.
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
- `oxla`: captures of the device's counter pattern and of idle inputs are
  verified against the gateware's Python tool on four bitstreams, but not
  a real signal on the inputs (the path of two samples per clock above
  all, which the counter enters after), a 16 channel bitstream with RLE
  (`la16-rle` had no build on 2026-11-14), the ADC variant, a capture
  stopped before its first sample under RLE, a second open after a close,
  and Linux, where the board is `/dev/ttyUSB1` behind `ftdi_sio`.
- `oxla-tn20k` is one board: the debugger's ids and the board's name are
  in the driver, and only the variants come from the descriptions beside
  the bitstreams. A description also says how its board shows on USB and
  how it is loaded (`target`), which is what a driver for every described
  board would go by; that wants a second board to be written against.
- `oxla-tn20k`: loads were run on one board on macOS. Not run: Linux,
  where the claim of the debugger's first port detaches `ftdi_sio` from
  it and the UART is found among `/dev/ttyUSB*` by the serial number;
  Windows; a board whose flash holds nothing valid (the FPGA then reports
  a CRC error, and the sequence that clears it is openFPGALoader's,
  unrun here); two boards at once; a debugger with other firmware than
  this board's (serial number 2025030317). There is no recording of a
  load to replay: one is a megabyte of bitstream.
- LogicPort: the clock qualifier input (which input gates the clock),
  the 2.5 ns `before`/`after` sample point (needs a phase-locked source
  with adjustable delay), the duration correction at 100 MHz and
  500 MHz DDR, inversion with the clock edge, counter source 1
  (`D0=1*2` on the pulse train), and the frequency counter above 1 MHz
  (up to the vendor's 300 MHz).
