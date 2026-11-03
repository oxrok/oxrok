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
| 9a | transport shells: vendor bulk and interrupt, FTDI, EZ-USB renumeration, HID, CDC-ACM with modem lines, USBTMC | the six Pico roles, the LogicPort and fx2lafw re-run on the host; `tools/diffcli.sh --emu` needs no hardware. 2026-11-03: HID (relay, PICkit2, the BU-86X, CH9325 and Victor cables, all full cases) and FTDI shells done |
| 9b | oracle instrumentation: libsigrok built with `--coverage`, a per driver gcov report of the lines the scenarios reach, transaction log diff in `diffcli.sh` | each emulated driver has a coverage figure and a list of C paths not emulated. Harness done 2026-11-02: models found by `emu/build.rs`, one scenario file per model with goldens (oracle-only cases before a port exists), a host-wide session lock, firmware stand-ins, `tools/emu-gcov.sh` and `diffcli.sh --gcov` (`docs/USB.md`); the relay driver's figure is 78.1% |
| 9c | FTDI family: `chronovu-la`, `ftdi-la`, `asix-sigma`, `ikalogic-scanaplus`, `pipistrello-ols` | emulator-verified, per the criteria below. Started 2026-11-03: the FTDI shell (`emu/src/shells/ftdi.rs`) and `ftdi-la` as oracle cases (36, gcov 79.0%) |
| 9d | EZ-USB family: `dreamsourcelab-dslogic`, `hantek-6xxx`, `hantek-dso`, `kingst-la2016`, `lecroy-logicstudio`, `saleae-logic16` | emulator-verified |
| 9e | vendor USB: `zeroplus-logic-cube`, `greatfet`, `hantek-4032l`, `lascar-el-usb`, `testo`, `kecheng-kc-330b`, `ikalogic-scanalogic2`, `sysclk-lwla`, `sysclk-sla5032`, `saleae-logic-pro`, `uni-t-dmm` (raw USB path) | emulator-verified |
| 9f | serial instruments as CDC-ACM models, modbus included | emulator-verified; the ESP32 `EMIT` cases move to the host |
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
   needs neither the model nor root.
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
  the Pico emulator (see `docs/DEVICES.md`); the CP2110 framing and
  feature reports are tested only against the C code.
- `rigol-ds` on the other series (DS2000, DS1000Z, DS4000, MSO5000, the
  Agilent DSO1000): the DS1102E is verified; each other series needs
  a recording (`OXROK_SCPI_RECORD`) and a sigrok-cli comparison.
- A real PICkit2 and a real relay board: both drivers have only met
  the Pico emulator.
- Real multimeters: the serial meters have met ESP32 replays of UT61E
  and UT60A streams and the HID ones the emulator, no meter yet.
- fx2lafw triggers and timing against the signal generator: needs a
  wire from the ESP32 to the analyzer.
