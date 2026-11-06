# Roadmap

| Phase | Deliverable | Exit criterion |
|---|---|---|
| 0 | workspace, CI, core types, demo driver, CLI with sigrok-cli compatible outputs | done when `cargo nextest run` is green on three platforms and the demo outputs match sigrok-cli byte for byte |
| 1 | `.sr` read and write, VCD, CSV, binary | round trips; sigrok-dumps files load; PulseView opens our files |
| 2 | USB transport (nusb, optional libusb), record and replay, FTDI | done 2026-10-22: FT245 requests, bit-bang entry and FIFO reads work on macOS with both backends; a recorded session replays in CI. Linux is checked with the first driver |
| 3 | Intronix LogicPort driver, bench crate, replay fixture | done 2026-10-22: 35 bench checks pass, `bits` and `csv` match libsigrok byte for byte, two recordings replay in CI, `docs/DRIVER_PORTING.md` |
| 4 | EZ-USB upload, software trigger, fx2lafw driver | done 2026-10-22 on a Saleae Logic clone: firmware upload and renumeration, 60 s at 24 MS/s without a missing byte (1.6 s CPU, 25 MB), `--show` identical to sigrok-cli, continuous mode stopped by Ctrl-C, a recording replays in CI |
| 5 | CLI parity pass and the differential script | done 2026-10-22: `tools/diffcli.sh` runs 29 software and 8 hardware cases against sigrok-cli 0.8.0-git, all clean; module listings compared by option name |
| 6 | decoder runtime, native uart, i2c, spi, Python decoder host, conformance fixtures | done 2026-10-22: runtime, native uart, i2c, spi, stacking and the PyO3 host (all 112 upstream decoders load, checked ones identical to sigrok-cli); UART, I2C and SPI fixtures from the bench |
| 6a | LogicPort extras: trigger counts, durations, value ranges, or/and combines, clock qualifier, sample point, inversion, frequency counter | done 2026-10-22: `-t` grammar in core, options on the device, 60 bench checks pass; the verification below is still open |
| 6b | serial transport and the serial multimeter family | done 2026-10-23: serial transport with record and replay, 20 chipset parsers, all 69 meters; UT61E and UT60A streams from the ESP32 identical to sigrok-cli; HID cables (BU-86X, CH9325, CP2110, Victor) identical to sigrok-cli on the Pico emulator, except the CP2110 |
| 6c | SCPI transport (USBTMC, TCP, serial) and the first scope driver (`rigol-ds`) | done 2026-10-23: SCPI with record and replay, `rigol-ds` with all 61 models; byte identical to sigrok-cli on a real DS1102E and on the Pico's USBTMC emulation of it |
| 6e | `raspberrypi-pico` (sigrok-pico firmware over USB serial) | done 2026-10-23: byte identical to sigrok-cli on a Pico, two recorded replay fixtures |
| 6f | HID devices: `uni-t-dmm` twins, `uni-t-ut32x`, `dcttech-usbrelay`, `microchip-pickit2` | done 2026-10-23 against the Pico emulator; a real PICkit2 is the open check |
| 6d | the remaining USB logic analyzers, one per recording | each ships a replay fixture; bench runs where the device is at hand |
| 7 | packaging: static binaries, Homebrew, AUR, udev rules | a clean machine installs and captures without any sigrok package |
| 7b | release binaries with the Python decoders | done 2026-10-23: the `rustpython` feature and `make dist`; the `Release binaries` workflow builds static Linux (musl), macOS and Windows archives with the 0.5.3 decoders, by hand and on each release |
| 7a | one crate, GitHub repository, CI on three platforms | done 2026-10-23: the workspace folded into the `oxrok` package, pushed to github.com/mangelajo/oxrok, CI green on Linux, macOS and Windows |
| 8 | release 0.1 | tags, changelog, guides, `cargo publish` |
| 9 | USB/IP device server in Rust (`emu`): the protocol, a `DeviceModel` trait, attach and detach through `vhci-hcd`, pedantic checks and a transaction log | done 2026-11-02 on Fedora 44: `oxrok-emu` attaches the relay board; `tools/diffcli.sh --emu` runs 11 cases with identical output, identical transactions and zero violations under both tools, and found the first Linux-only HID bug of the port. Open: running it on CI (`modprobe vhci-hcd` on the hosted runner, else a self-hosted Linux runner for this tier) |
| 9a | transport shells: vendor bulk and interrupt, FTDI, EZ-USB renumeration, HID, CDC-ACM with modem lines, USBTMC | the six Pico roles, the LogicPort and fx2lafw re-run on the host; `tools/diffcli.sh --emu` needs no hardware. 2026-11-03: HID (relay, PICkit2, the BU-86X, CH9325 and Victor cables, all full cases) and FTDI shells done. CDC-ACM shell done 2026-11-04 |
| 9b | oracle instrumentation: libsigrok built with `--coverage`, a per driver gcov report of the lines the scenarios reach, transaction log diff in `diffcli.sh` | each emulated driver has a coverage figure and a list of C paths not emulated. Harness done 2026-11-02: models found by `emu/build.rs`, one scenario file per model with goldens (oracle-only cases before a port exists), a host-wide session lock, firmware stand-ins, `tools/emu-gcov.sh` and `diffcli.sh --gcov` (`docs/USB.md`). 2026-11-03: the shared sources (`usb.c`, `ezusb.c`, `serial*.c`) reported too, fault options in the shells and models, unplug and interrupt events; drivers at 60% to 86% |
| 9c | FTDI family: `chronovu-la`, `ftdi-la`, `asix-sigma`, `ikalogic-scanaplus`, `pipistrello-ols` | emulator-verified, per the criteria below. Started 2026-11-03: the FTDI shell (now vusb's) with fault options, and `ftdi-la` as oracle cases (44, gcov 86.4%), ported in batch 4 (46 cases, all full) over `src/ftdi/libftdi.rs`, whose divisor matches the libftdi 1.5 binary on every chip type. Batch 3 modelled the rest; batch 5 (2026-11-03) ported them over `src/ftdi/libftdi.rs`: `chronovu-la` (52 cases, gcov 87.0%), `asix-sigma` (110, 89.4%), `ikalogic-scanaplus` (53, 93.8%), `pipistrello-ols` (107, 97.0%, its model checked against the author's firmware source). All of 9c is ported and emulator-verified; none on hardware, and the chips are inferred from the C drivers |
| 9d | EZ-USB family: `dreamsourcelab-dslogic`, `hantek-6xxx`, `hantek-dso`, `kingst-la2016`, `lecroy-logicstudio`, `saleae-logic16` | emulator-verified. 2026-11-03: the EZ-USB shell with renumeration, and `fx2lafw` emulator-verified (78 cases, gcov 91.2%), the USBee AX analog input ported. Batch 2: `saleae-logic16`, `dreamsourcelab-dslogic`, `kingst-la2016`, `hantek-6xxx`, `hantek-dso` modelled as oracle cases (80 to 138 cases each, gcov 90% to 96%). Ported: `saleae-logic16` (87 cases, batch 4), `kingst-la2016` (151) and `dreamsourcelab-dslogic` (94, batch 5); Batch 4: `lecroy-logicstudio` modelled (83 oracle cases, 56 of them library-verified through `tools/libsigrok-run.c`, as sigrok-cli cannot capture with it; gcov 95.9%). Batch 6 (2026-11-04) ported the rest: `lecroy-logicstudio` (94 cases, gcov 95.9%, its captures full against `tools/libsigrok-run.c`), `hantek-dso` (148, 96.2%), `hantek-6xxx` (102, 91.0%). All of 9d is ported and emulator-verified; none on hardware |
| 9e | vendor USB: `zeroplus-logic-cube`, `greatfet`, `hantek-4032l`, `lascar-el-usb`, `testo`, `kecheng-kc-330b`, `ikalogic-scanalogic2`, `sysclk-lwla`, `sysclk-sla5032`, `saleae-logic-pro`, `uni-t-dmm` (raw USB path) | emulator-verified. Batch 4 (2026-11-03): `saleae-logic-pro` (102 oracle cases, gcov 97.0%) and `hantek-4032l` (121, 90.7%) modelled. Batch 5: `zeroplus-logic-cube` (121, 87.6%) and `sysclk-lwla` (130, 90.4%) modelled. Batch 6 (2026-11-04) ported `hantek-4032l` (135 cases, gcov 91.4%), `sysclk-lwla` (141, 91.1%), `zeroplus-logic-cube` (134, 87.6%) and `saleae-logic-pro` (107, 97.1%), none on hardware, and modelled `sysclk-sla5032` (142 oracle cases, 91.0%) and `ikalogic-scanalogic2` (134, 88.6%), ready for their ports. Batch 7 (2026-11-04) ported the rest: `uni-t-dmm` raw USB (78 cases, gcov 97.4%), `testo` (70, 91.3%), `sysclk-sla5032` (165, 91.5%), `ikalogic-scanalogic2` (143, 88.6%), `kecheng-kc-330b` (129, 79.3%), `lascar-el-usb` (152, 93.9%), `greatfet` (131, 84.8%). All of 9e is ported and emulator-verified; none on hardware |
| 9f | serial instruments as CDC-ACM models, modbus included | emulator-verified; the ESP32 `EMIT` cases move to the host. Shell, tty harness and pilot done 2026-11-04: `serial-dmm` (29 cases, gcov 71.3%) and the first new driver on it, `korad-kaxxxxp` (130, 88.6%), neither on hardware. Batch 8 (2026-11-05): `ols` (213 cases, gcov 94.7%), `rdtech-dps` with `src/modbus` (226, 89.2%), `rdtech-um` (90, 93.7%), `rdtech-tc` (73, 93.8%), `manson-hcs-3xxx` (171, 91.9%), `fluke-dmm` (159, 98.6%), `atten-pps3xxx` (133, 90.5%), `gwinstek-gpd` (141, 95.6%), none on hardware. Batch 9 (2026-11-05): `maynuo-m97` (210 cases, gcov 95.8%), `motech-lps-301` (149, 79.3%), `juntek-jds6600` (229, 89.2%), `zketech-ebd-usb` (109, 86.6%), `agilent-dmm` (383, 97.4%), `gwinstek-psp` (154, 92.5%), `itech-it8500` (223, 93.6%), `arachnid-labs-re-load-pro` (131, 91.8%), none on hardware. Batch 10: `icstation-usbrelay` (43, 83.0%), `rohde-schwarz-sme-0x` (76, 94.9%), `appa-55ii` (72, 96.6%), `mastech-ms6514` (72, 94.9%), `pce-322a` (95, 88.1%), not on hardware; 51 of the 87 drivers ported, 36 to go. The emulated cases replace the `--hw` EMIT cases' parser and stream checks, not the real UART, adapter driver or handshake lines: keep at least one EMIT case |
| 9g | SCPI instruments as TCP and USBTMC models | emulator-verified; `rigol-ds` series beyond the DS1102E included |
| 9h | the odd transports: GPIB shim, parallel port shim, sysfs root, vendor CLI stub, BLE on the ESP32 | each driver either emulator-verified or listed as hardware-only with the reason |
| 9i | Pico backend for the same models, replacing the C firmware in `hil/pico-hidcable` | one model runs on the host over USB/IP and on the Pico; the bench cases pass on both |

## Emulation plan

Porting a driver without the hardware means building the device from
the C driver's view of it, checking that the real C driver is satisfied
by that device, and then checking that oxrok drives it identically.
What that proves is equivalence with libsigrok on the paths the device
model exercises; what it cannot prove is behaviour the C driver relies
on without stating. The method is only as good as the model is strict,
so the rules below are the plan.

### Device models on the host

On Linux the device is a program, not a board: the `vhci-hcd` module
turns a USB/IP server into a USB device that libusb, nusb, hidraw and
the kernel's cdc-acm and usbtmc drivers see as real. So:

- `emu` is a crate in the workspace with a USB/IP server written here
  (the protocol is `OP_REQ_IMPORT` plus `CMD_SUBMIT`, `RET_SUBMIT` and
  `CMD_UNLINK`; the existing `usbip` and `usbip-device` crates are
  alpha and lack the hooks below). The harness attaches and detaches
  through `/sys/devices/platform/vhci_hcd.0`, so no `usbip` tool is
  needed, only root on the test machine.
- A device is a `DeviceModel`: descriptors, `control(setup, data)`,
  `out(ep, bytes)`, `in(ep, max)` returning data, NAK or STALL, and
  `reset`. The model is plain Rust with no I/O, so it can also run on
  a Pico later (phase 9i) and in-process for unit tests.
- Transport shells carry the parts every device of a kind shares:
  the FTDI vendor requests, latency timer and the two status bytes per
  packet; EZ-USB firmware upload and renumeration (the model asks the
  server to disconnect and the harness re-attaches it with its new
  ids); HID report framing; CDC-ACM with `SET_CONTROL_LINE_STATE`, so
  serial drivers get a real `/dev/ttyACM` with RTS and DTR (a pty
  would fail the handshake line ioctls); USBTMC. A driver's model is
  a state machine and a sample generator on top of one shell.
- Serial instruments are CDC-ACM models; SCPI over the network is a
  TCP server, no USB/IP needed. Sample generators reset on every
  session start, so both tools see the same samples and nothing has to
  be filtered out of the comparison.
- USB/IP is Linux only. The Pico remains the bench device on macOS and
  the check that real USB framing (short packets, ZLPs, timing) is
  handled; it does not have to run every model.

### What counts as emulator-verified

A driver is emulator-verified when all of these hold, and the README
table says "emulator" rather than "hardware" for it:

1. sigrok-cli and oxrok produce identical output for the scenario list
   in `diffcli.sh`, after the header line.
2. The model's pedantic mode logged no violation under either tool.
   Pedantic mode checks what the host sends against the protocol as the
   C driver's comments and the datasheet state it: transfer sizes,
   request order, legal register values, sequence numbers. A clean run
   under sigrok-cli validates the model; a clean run under oxrok
   validates the port. This closes the main gap of the method, which is
   that the C driver ignores most of what it does not need and would
   accept a wrong model.
3. The transaction logs of the two runs are identical after the
   normalisation the fixture layer already defines (poll counts on
   reads without sequence numbers). Identical bytes on the wire is a
   much tighter claim than identical `bits` output.
4. The gcov report of the C driver, from a libsigrok built with
   `--coverage` and run through the scenarios, is attached to the
   driver's entry in `docs/DEVICES.md`, with the unreached paths
   named. Scenarios are added until what remains is listed as "not
   emulated" on purpose (hardware error paths, model variants nobody
   has). The figure says when to stop writing scenarios.
5. Replay fixtures recorded from the emulator run, so `cargo test`
   needs neither the model nor root: one or two `--record` cases per
   driver (`tests/fixtures/DRIVER/emu-*.txt` with oxrok's stdout beside
   it), replayed by `tests/emu_replay.rs` through the oxrok binary on
   every platform, and stable from one emulator run to the next (a run
   without `EMU_BLESS` fails on a recording that differs). Done
   2026-11-03 for `fx2lafw`, `microchip-pickit2`, `dcttech-usbrelay`,
   `brymen-bm86x`, `uni-t-ut61e-ser` (CH9325 and CP2110 cables) and
   `victor-dmm`.
6. `diffcli.sh --hw` runs the same scenarios against the real device
   with one command, so a user who has it can promote the driver to
   hardware-verified and report back.

Firmware the driver uploads is opaque to the model; where the vendor's
file cannot be redistributed, the scenario uses a stand-in of the
right name and size, and both tools upload the same bytes.

### Driver inventory by shell

libsigrok at the oracle commit has 87 driver directories; with
`intronix-logicport`, which only oxrok has, that is 88. By the shell
their model needs (`demo` and the seven ported drivers included):

- FTDI (6): `intronix-logicport`, `chronovu-la`, `ftdi-la`,
  `asix-sigma`, `ikalogic-scanaplus`, `pipistrello-ols`.
- EZ-USB renumeration (7): `fx2lafw`, `dreamsourcelab-dslogic`,
  `hantek-6xxx`, `hantek-dso`, `kingst-la2016`, `lecroy-logicstudio`,
  `saleae-logic16`.
- Vendor bulk and interrupt (12): `zeroplus-logic-cube`, `greatfet`,
  `hantek-4032l`, `lascar-el-usb`, `testo`, `kecheng-kc-330b`,
  `ikalogic-scanalogic2`, `sysclk-lwla`, `sysclk-sla5032`,
  `saleae-logic-pro`, `uni-t-dmm`, `microchip-pickit2`. No libsigrok
  driver uses isochronous transfers, which USB/IP handles badly.
- HID (1 driver plus the cables): `dcttech-usbrelay`; the BU-86X,
  CH9325, CP2110 and Victor cables under `serial-dmm`.
- CDC-ACM (39): `agilent-dmm`, `appa-55ii`, `arachnid-labs-re-load-pro`,
  `atorch`, `atten-pps3xxx`, `bkprecision-1856d`, `cem-dt-885x`,
  `center-3xx`, `colead-slm`, `conrad-digi-35-cpu`, `devantech-eth008`,
  `fluke-dmm`, `gmc-mh-1x-2x`, `gwinstek-gpd`, `gwinstek-psp`,
  `icstation-usbrelay`, `itech-it8500`, `juntek-jds6600`, `kern-scale`,
  `korad-kaxxxxp`, `link-mso19`, `manson-hcs-3xxx`, `mastech-ms6514`,
  `mic-985xx`, `motech-lps-30x`, `norma-dmm`, `openbench-logic-sniffer`,
  `pce-322a`, `raspberrypi-pico`, `rdtech-tc`, `rdtech-um`,
  `rohde-schwarz-sme-0x`, `serial-dmm`, `serial-lcr`, `teleinfo`,
  `tondaj-sl-814`, `uni-t-ut181a`, `uni-t-ut32x`, `zketech-ebd-usb`;
  plus `maynuo-m97` and `rdtech-dps` over modbus.
- SCPI over TCP or USBTMC (16): `beaglelogic`, `fluke-45`,
  `gwinstek-gds-800`, `hameg-hmo`, `hp-3457a`, `hp-3478a`, `hp-59306a`,
  `ipdbg-la`, `lecroy-xstream`, `rigol-dg`, `rigol-ds`, `scpi-dmm`,
  `scpi-pps`, `siglent-sdl10x0`, `siglent-sds`, `yokogawa-dlm`.
- Odd transports (5): `hung-chang-dso-2100` (parallel port),
  `mooshimeter-dmm` (BLE), `baylibre-acme` (sysfs and iio),
  `asix-omega-rtm-cli` (spawns the vendor's program), and GPIB as a
  transport under the SCPI drivers.

Order within a phase: by how many users the device has (DSLogic,
Kingst, Hantek 6022, the Saleae clones and ZeroPlus first among the
analyzers; Rigol, Siglent and Hantek among the scopes; Korad and RDTech
among the supplies).

### The transports USB/IP does not cover

- GPIB and the parallel port have no in-tree virtual host the way USB
  has `vhci-hcd`, but the kernel drives USB adapters for both, so the
  same USB/IP server can present the adapter and the instrument
  behind it. Three ways, in order of preference:
  1. A model of a USB adapter with an in-tree driver. Parallel:
     the Belkin USS-720 (`uss720`, mainline since 2.4), vendor
     requests for the SPP registers and bulk for ECP; the driver
     registers a real `struct parport`, so `/dev/parport0` and the
     `/proc/sys/dev/parport` entries libieee1284 enumerates both
     exist. GPIB: linux-gpib is in `drivers/staging/gpib` since 6.13
     with `lpvo_usb_gpib` (an ASCII command protocol, the simplest),
     `agilent_82357a` and `ni_usb_gpib`; a model of one gives a real
     `/dev/gpib0`. Both tools run unpatched, and oxrok's own ppdev and
     linux-gpib ioctl backends get tested, not a stand-in. Needs the
     modules on the CI kernel (`uss720` is in Ubuntu's modules-extra;
     the staging GPIB drivers need a 6.13 kernel that builds them),
     which is the first thing to check. Checked 2026-11-02 on Fedora 44,
     kernel 7.2.14: `vhci-hcd` is in `kernel-modules-extra`, `uss720`
     and `lpvo_usb_gpib` (with `agilent_82357a` and `ni_usb_gpib`) in
     `kernel-modules`; all three load.
  2. CUSE: a userspace program that is `/dev/gpib0` or `/dev/parport0`
     and answers the ioctls (`IBRD`, `IBWRT`, `IBRSP`, `IBONL`, ...;
     `PPRDATA`, `PPWDATA`, `PPWCONTROL`, `PPRSTATUS`, `PPCLAIM`, ...).
     In-tree and no kernel code, but `fuser` has no CUSE support, so
     it is a direct implementation against `/dev/cuse`, and CUSE cannot
     create the `/proc/sys/dev/parport` entries; whether libieee1284's
     fallback probing in `detect.c` finds `/dev/parport0` without them
     is unverified.
  3. An `LD_PRELOAD` shim of `libgpib` (six symbols: `ibfind`, `ibrd`,
     `ibwrt`, `ibrsp`, `ibloc`, `ibonl`) or `libieee1284` (twelve
     calls) forwarding sigrok-cli to the model. Cheapest, but it only
     covers the oracle; oxrok's backend is then tested only against
     the in-process model.
  Not on the list: an out-of-tree `parport_virtual` or `gpib_virtual`
  kernel module (works, but is built against the CI kernel on every
  run, against a GPIB API still in staging, and a bug panics the
  runner), and eBPF, which can observe and fail syscalls but cannot
  create a device or fill an ioctl buffer.
  In any case every GPIB instrument's driver logic is testable over
  `tcp-raw` with the same SCPI model, since GPIB sits under the SCPI
  layer; only the transport needs one of the above, and `hp-3478a` is
  the only instrument with no other port. Prologix adapters over
  serial are an oxrok extra libsigrok does not have.
- Bluetooth LE. `mooshimeter-dmm` talks GATT through BlueZ. Virtual
  controllers (`hci_vhci`, `btvirt`) plus a GATT server are possible
  but heavy for one driver; the ESP32 on the bench as a BLE peripheral
  that serves the Mooshimeter's characteristics is the practical path.
- sysfs. `baylibre-acme` reads hard-coded `/sys` paths; a fake tree
  bind-mounted in a user namespace (`unshare -Urm`) is enough.
- A vendor program. `asix-omega-rtm-cli` runs the vendor's CLI and
  reads its stream; a stand-in executable on `PATH` is the model.

### Open items of the emulator tier

Left over from the first batch and the coverage round (2026-11-03),
each with where it belongs:

- Failed string descriptor reads at the scan for the batch 2 and 3
  models (chronovu-la `api.c` 192-204 and the like): the
  `fail_standard` hook exists now (fx2lafw uses it); each model needs
  its own option and cases.
- A persistent endpoint halt: the models' `stall-in=N` STALLs once, as
  firmware that re-arms its endpoint; a real STALL halts until
  CLEAR_FEATURE, which no C driver sends. A variant would show what
  both tools do then (likely log for ever, so `--timeout` and
  `--oracle-bug`).
- The BM52x `data_source` option (recordings stored in the meter):
  libsigrok has it, oxrok does not, so the `bu86x` model's `meter=bm52x`
  has no `--show` case (6b).
- Meters whose packets are of variable length or need `after_open`
  (BM85x) and the multi display ones (EEVblog 121GW, MS2115B, Metex 4
  display) wait for the CDC-ACM shell (9f); they hold most of
  serial-dmm's unreached lines.
- The emulator runs on the `vusb` crate, and the harness drives it
  through `vusb-ctl`. The goldens keep the fixture dialect, written by
  vusb's `observer::fixture::Log`; `emu/src/txlog.rs` gives it each
  model's rules.
- The emulator tier has a CI job (`.github/workflows/ci.yml`,
  `emulator`): it loads vhci-hcd (installing
  `linux-modules-extra-$(uname -r)` if needed) and runs
  `tools/diffcli.sh --no-oracle`, which holds oxrok to the goldens of
  every model that has oxrok's driver, and skips with "skipped: no
  vhci-hcd" when the runner's kernel cannot load the module. Its
  `--no-oracle` step succeeded on GitHub's runners (run of `e86ce84`). A
  skipped job counts as satisfied for branch protection.
- The harness follows the address through RENUMERATED, but a case line
  expands `$EMU_USB` before its own reset: `emu_reset` covers a case
  with one tool, and a full case's two tools still see different
  addresses on a board the first renumerates. And `emu_case` starts with a
  reset that re-attaches a board whose descriptors changed, so a claim
  helper's hold taken before it is on the device that left; the
  scenarios order their cases around it.
- Several emulator sessions at once: one vhci controller with 8 ports
  is enough, but libusb lists every device and most drivers match by
  VID:PID, so sessions would see each other's devices; cleanup, port
  choice and the gcov counts are global too. A test of running each
  tool in a namespace whose `/dev/bus/usb` holds only its own node
  (bubblewrap, no sysfs) would show whether libusb then sees only its
  own device.
- A LogicPort model on the FTDI shell (row 9a), once the sigrok
  drivers are converted: oxrok alone has the driver, so its cases are
  held to goldens taken from the bench's recordings rather than to
  sigrok-cli. It brings the LogicPort into CI without the hardware.
- Large transaction goldens: done for chronovu-la (3.1 MB to 55 KB)
  and asix-sigma (2.1 MB to 0.98 MB) with `EMU_TX_FOLD`. What remains
  in asix-sigma is inline bulk OUT payloads, which folding does not
  cover.
- The libusb backend has no regular emulator run: the gates use nusb,
  which is how a stream reading ahead lost samples unseen (dslogic). In
  batch 6, lecroy-logicstudio passed 83/83 on it and hantek-6xxx was
  run, differing only by string reads. String reads at listing, which
  only libusb makes, make other models fail on transactions. Its timed
  stream still reads packet by packet, so its `.tx` differs and a fast
  stream may overflow. Wanted: one `OXROK_USB_BACKEND=libusb` run per
  batch, comparing output only.
- `--oracle-bug` accepts any non-zero exit, and `--oracle-hangs` now
  pins a real hang (pipistrello-ols); dslogic's two trigger-fault cases
  still have an empty `.oracle`, which pins nothing.
- Two rare flakes, explained. `lib fault still=1 hangs` and the like
  (lecroy-logicstudio): the model hashed its bitstream stand-ins inside
  the reply to the last chunk, past the C driver's 100 ms under load;
  they are now made when the model is built. `fault halt after one`
  (saleae-logic-pro): `protocol.c:909`'s resubmit lands inside the
  start's OUT or IN wait; a submit-order shim over 40 runs matched the
  log order.
- Other models call `ezusb::standin` during transfers: saleae-logic16 and
  saleae-logic-pro (little exposed by the C's timeouts or the file's
  size), dreamsourcelab-dslogic and kingst-la2016 (not checked). The same
  stop race may be open in hantek-4032l's `lib stop while held` and
  sysclk-lwla's `lib stop during the readout`; no failure was seen.
- `nusb_backend`'s `bulk_out` takes `Cancelled` with some data sent as
  success, where the C fails on a timed out write. Check it against a
  device that times out.
- Device firmware sources may settle what models assume: the
  Pipistrello's overturned three of them. Look for the others'
  (DreamSourceLab's FPGA sources, for one).
- Hardware checks written down in DEVICES.md and not yet run:
  asix-sigma's last event, the Pipistrello's marker table and 200 MHz
  trigger.
- `--show -g GROUP` (fx2lafw's `Logic` and `A0`): sigrok-cli stops
  after the device line when `sr_dev_options` fails for the group,
  oxrok prints the groups and an empty option list
  (`src/cli/show.rs`); and `--get` with a group fails in both tools
  with other wording.
- `tools/attribution.py --check` would rewrite the headers of
  `emu/src/models/hantek-dso.rs` and `src/drivers/fx2lafw/mod.rs`
  (a hand-added "hantekdso project" credit is not in the generator's
  sources); the credit belongs in `tools/attribution.d`. Add
  `emu/src/models/hantek-6xxx.rs` to this list; `--check` would rewrite
  it too.
- `tools/attribution.py` credits files only: facts taken from a
  firmware source tree (the fx2lafw and hantek-6xxx models use
  sigrok-firmware-fx2lafw) are credited by hand in the model's header.
- A failed capture exits 0 on every port. Decided 2026-11-04: it will exit
  non-zero and say what happened on stderr, on all ports, through an error
  path alongside `Packet::End`.
- `Session`'s `End` is sent before the link goes back in its slot, so
  `start()` right after `End` can get Busy (lecroy-logicstudio,
  saleae-logic16, kingst-la2016, hantek-6xxx, sysclk-lwla);
  sysclk-sla5032's order is the fixed form: the link goes back first, and
  it asks whether the thread still holds it.
- The rusb stream thread exits on any non-timeout error, so one
  transient error loses the rest of a capture; it is not joined on drop;
  it keeps one read in flight.
- `fn shots/hit` is duplicated in 5 models: a candidate for a shared
  helper.
- Testo: more units from Testo's table (°F, CO2, dew point) once a
  recording shows a 435 sends them. Decimal exponent for `digits`,
  likewise.
- Serial: `Native` holds a second tty descriptor for HUPCL and INPCK, so
  each oxrok open sends the line state twice (the `.tx` normalisation
  hides it). Kept, rather than relaxing `forbid(unsafe_code)` in
  `src/serial`.
- `--record` for serial: the harness records USB and HID only, so serial
  drivers have no replay fixture.
- `-O analog --show` words `digits` differently from sigrok-cli.
- Goldens hold "timeout: the monitored command dumped core", which
  differs on a host with a plain core_pattern and limit 0: normalise it
  in the harness.
- From batch 8: `korad-kaxxxxp` has the old silence bound on `--time` and
  `--continuous`, holds its lock across a send, and ignores `-C`.
- `-O csv` makes up 0 for channels absent from a packet and appends per
  channel, so a driver sending one packet per reading (`gwinstek-gpd`)
  mixes units in a column. Decide a format.
- `-C D-` reads as a range, in both tools, so a channel named `D-`
  cannot be selected alone: try the exact name first.
- `-c` checks parsing, scope and access before sending, not a driver's
  ranges, so `voltage_target=5:current_limit=99` sends the voltage
  before the refusal.
- The CLI closes a device only on its happy path, and `ctrlc` is built
  without `termination`: a device that writes at close (`rdtech-dps`'s
  panel lock) needs its own `Drop`. Close on every exit.
- `tools/diffcli.sh`'s `to_cmp` ignores `sed` and `mv` failures: a full
  disk failed a case falsely, and could pass one.
- A CP210x shell (`10c4:ea60`, vendor class) and using the FTDI shell
  for serial models: Manson, GPD and Atten run on a CDC-ACM stand-in, so
  `cp210x`, `ftdi_sio` and `/dev/ttyUSBn` are not exercised.
- `p-ols` has the `Busy` on a second capture that `ols` fixed by reaping
  once the port is back.
- `-O ols` is absent, and `0x10` and `3.3V` as values are refused where
  sigrok-cli's `strtof` takes them.
- About 20 drivers have a capture thread that blocks in `sink.send` while
  close joins it: a full queue nobody reads hangs the close, as
  `atten-pps3xxx` did under load. `PacketSink::send_while` is the fix;
  `rdtech-dps` uses it, and whether the rest get it one by one or through
  a shared change is undecided.
- 9 unit tests fail with every sleep at six times, a harsher test than
  CI's macOS runner, where they pass today: `gwinstek-gpd` (2),
  `atten-pps3xxx` (3), `rdtech-dps` (2), `rdtech-um` (1), `manson-hcs-3xxx`
  (1). `ols` had 3 of them, two of which did fail on macOS. Widen these if
  macOS starts to flake on them.
- Several drivers narrow settings to `f32` because sigrok-cli parses
  values with `strtof` (`session.c:570`), where C's driver keeps a
  double, so a library client gets a different request:
  `manson-hcs-3xxx`, and more in batch 9. The narrowing belongs in the
  CLI's option parsing.
- `-O analog:digits=spec` shows the same digits as `all`: `AnalogData`
  holds one digit count, where libsigrok has a separate spec figure.
- `tools/diffcli.sh`'s `emu_hex` loses a value with a leading `-`, as
  `printf` takes it for a flag. No scenario has one today; `printf --`
  fixes it.
- A race in vusb's USB/IP server, seen once in about 11,100 attaches: the
  kernel can send `CMD_UNLINK` ahead of its `CMD_SUBMIT`, vusb answers the
  unknown unlink with status 0, and the late `RET_SUBMIT` then makes
  `vhci_hcd` drop the connection ("cannot find a urb of seqnum"), so
  `oxrok-emu` exits 1. vusb's half is proven with a raw client; the kernel's
  reordering is inferred from its source and that one log line. The fix is
  in vusb: remember an unlink above the highest seqnum submitted, and drop
  that submit with no reply.
- `src/cli/pipeline.rs`'s `check_limit` sums readings across channels, as
  `session.c:441-444` does, so "Device only sent N samples" counts readings
  and a capture cut short with at least half its samples gets no warning.
- A `libgcov profiling error:` line on the oracle's stderr fails a stderr
  comparison on any model, as a `.gcda` truncated by a full disk did. The
  harness could filter that line.
- Literal short USB timeouts in C drivers, the kind that made Kecheng's scan
  miss a present device: `hantek-dso` at 40 ms and `lascar-el-usb` at 50 ms
  (control transfers, which `EMU_ORACLE_USB_FLOOR_MS` does not cover) are
  the next if they start to flake.
- `src/scpi`: `Scpi::get_data` has no size bound, as in C, so a device
  streaming with no newline holds a `--get` or `--scan` until it stops.
  `rigol_ds/mod.rs:632` passes `None` when it reopens, so a user's
  `serialcomm` is lost after a close and it opens at 115200, where C
  keeps it.
- A demo capture with a trigger on an analog channel (`-d
  demo:logic_channels=2:analog_channels=1 -C D0,A0 -t A0=r`) hangs
  oxrok, where sigrok-cli captures. Mixed VCD has a double space after
  the timestamp, and on demo lacks sigrok-cli's `#5 0!` and `#20 1!`
  lines.
- `atorch` and other drivers with BLE units: `bt/` conns and all BLE
  forms (`bt/notify`, `ble122`, `cc254x`) have no path in oxrok, so a
  `*-BLE` unit can't be reached. RFCOMM goes through `/dev/rfcommN`.
- `serve --attach`'s "sudo modprobe vhci-hcd" hint on Windows is
  Linux-specific.
- saleae-logic-pro's `usb.c` gcov varies from run to run (76-79%).
- The vendor ISO for sysclk-sla5032 (`sla5032_2015_1_24.iso`) has not
  been examined; it may settle what that model assumes.

### Risks

- The hosted CI kernel may not ship `vhci-hcd`; `linux-modules-extra`
  for the running kernel usually has it. If not, this tier runs on a
  self-hosted Linux runner and the hosted runners keep the replay
  tier, which needs nothing.
- A model written from the same C source as the port can share its
  misreading. sigrok-cli passing against the model is the independent
  check, and pedantic mode is what makes that check bite; a model that
  accepts anything proves nothing.
- Hardware quirks the C driver depends on without saying so cannot be
  found this way. The LogicPort bench found three. Nothing replaces a
  user with the device, which is why every driver keeps a one-command
  `--hw` path.
- The oracle version matters: the model emulates libsigrok's view of
  the device at the pinned commit, recorded in each fixture.

## Open verification

Hardware checks not yet done, with what each needs:

- Clock qualifier input: which input gates the clock (CLK2, or the other
  clock input, or a data channel). Wire ESP32 GPIO15 to CLK2, run
  `oxrok-hil -k qualifier --clk2`; it reports which of off, active-high
  and active-low complete or stall with CLK2 low and high. Then fix the
  option's description in `docs/DEVICES.md` and the driver.
- Sample point timing: the `setup-hold` section only shows that images
  0 to 4 capture correctly. Whether `before` and `after` move the sample
  by 2.5 ns needs a source phase locked to the clock with an adjustable
  delay, which the ESP32 does not provide.
- Duration delay at other rates: the one sample correction was measured
  at 1 MHz; check 100 MHz and 500 MHz DDR (ticks are two samples there)
  with the `duration` section at those rates.
- Inversion and the clock edge: `invert=CLK1` with state mode should
  sample on the opposite edge; check with `sample_point=before` that
  the image follows (`StateBeforeClk1Fall` for a rising edge selection).
- Counter source 1 (pattern entered) is used for level only counts and
  passed with `D0=1` entered twice in the software trigger model; the
  hardware check (`D0=1*2` on the pulse train) is not in the bench yet.
- Frequency counter above 1 MHz, up to the 300 MHz the vendor states:
  needs a faster source than the ESP32's LEDC output.
- HID cables: the BU-86X, CH9325 and Victor paths are verified with
  the Pico emulator and the host emulator, the CP2110 with the host
  emulator only (see `docs/DEVICES.md`). A real CP2110 would settle
  which parity codes the chip follows (libsigrok sends AN434 rev 0.4's,
  pyserial the other order) and whether it honours the purge as an
  output report.
- fx2lafw on the Saleae clone again: the upload moved into the scan
  and the open order changed (version and REVID before the claim) on
  2026-11-03, checked on the emulator only; a cold start after a power
  cycle on macOS covers the renumeration path. The nusb backend's claim
  before the first control transfer on Windows has not run on Windows.
- `rigol-ds` on the other series (DS2000, DS1000Z, DS4000, MSO5000, the
  Agilent DSO1000): the DS1102E is verified; each other series needs
  a recording (`OXROK_SCPI_RECORD`) and a sigrok-cli comparison.
- A real PICkit2 and a real relay board: both drivers have only met
  the Pico emulator.
- Real multimeters: the serial meters have met ESP32 replays of UT61E
  and UT60A streams and the HID ones the emulator, no meter yet.
- fx2lafw triggers and timing against the signal generator: needs a
  wire from the ESP32 to the analyzer.
