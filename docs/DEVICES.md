# Devices

## Intronix LogicPort LA1034

Driver `intronix-logicport`. 34 channels (D0-D31, CLK1, CLK2), 2048
sample entries, 1 kHz to 500 MHz timing mode, state mode on CLK1 or
CLK2, run length compression up to 200 MHz, two hardware trigger
levels, an input threshold of -6 to 6 V in 50 mV steps. An FT245BL
FIFO bridge (`0403:dc48`) in front of an SRAM based Cyclone FPGA that
the host configures on first use; the protocol and register map are in
the driver's `protocol.rs`.

Firmware: the vendor's `LogicPort.ccf`, unmodified, installed as
`intronix-logicport.ccf` in one of the firmware directories (first hit
wins):

- `$OXROK_FIRMWARE_DIR` (a colon separated list), `$SIGROK_FIRMWARE_DIR`
  (one directory, as libsigrok takes it), `$SIGROK_FIRMWARE_PATH` (a
  colon separated list)
- `oxrok-firmware`, then `sigrok-firmware`, in the user's data directory
  (`$XDG_DATA_HOME`, else `~/.local/share`)
- the same two in each system data directory (`$XDG_DATA_DIRS`, else
  `/usr/local/share`, `/opt/homebrew/share` and `/usr/share`)

This follows libsigrok's order (`resource.c`) with oxrok's names first
at each step, so a variable set for sigrok-cli reaches oxrok too. Three
differences remain. libsigrok also searches its compile-time
`FIRMWARE_DIR` (`$prefix/share/sigrok-firmware`) between the user's
directory and the system's; oxrok has no install prefix, so a sigrok
installed under another prefix needs `SIGROK_FIRMWARE_PATH`. oxrok's
default system list adds `/opt/homebrew/share`, where Homebrew puts
`sigrok-firmware`. And a `SIGROK_FIRMWARE_DIR` that is set but empty
is skipped by oxrok, where libsigrok keeps it as a lookup relative to
the current directory.

`sigrok-fwextract-intronix-logicport` from sigrok-util gets the file
from the vendor's Windows installer. Per image `.rbf` files written by
that tool (`intronix-logicport-<image>.rbf`) are accepted as a fallback.

Permissions: on Linux the device is root only without a udev rule; until
oxrok ships rules (phase 7), a rule with `MODE="0666"` for `0403:dc48`
does. macOS and Windows need nothing beyond `docs/USB.md`.

Options beyond libsigrok's, set with `-c`:

- `clock_qualifier=off|active-high|active-low`: in state mode, sample
  only while the qualifier input is high or low.
- `sample_point=on|before|after`: in state mode, sample on the clock
  edge, 2.5 ns before it or 2.5 ns after it (the FPGA image changes).
- `invert=D0,CLK1` or a hex mask, `none` to clear: inputs the device
  reads inverted, for the trigger and the stored data.
- `frequency_channel=CLK1`: the input of the frequency counter.
- `frequency`: read only, `--get frequency` measures the frequency on
  that input to 10 Hz, up to 300 MHz. A `--show` also measures it, which
  configures the FPGA on a cold device.

Triggers take the forms in `docs/CLI.md`: occurrence counts (plain or
consecutive), durations, value ranges on a channel group, and two stages
in sequence, or, or and.

Bench: `oxrok-hil` runs the hardware checks (levels, threshold, timebase
up to 500 MHz, triggers, counts, durations, value ranges, combine modes,
sample points, inversion, the frequency counter, compression, frames,
all byte lanes) against a LogicPort wired to the ESP32 signal generator
in `hil/esp32-siggen`. `oxrok-hil -k trigger` runs a
subset, `LOGICPORT_CONN` picks a unit, `SIGGEN_PORT` the serial port.
`oxrok-hil --clk HZ`, `--set MASK`, `--cmd LINE` and `--delay MS` only
drive the generator, for captures by hand. Opening the serial port
resets the ESP32, so the commands of one experiment go in one
invocation. Checks of features not yet seen working print UNVERIFIED
rather than FAIL; `--strict` counts them as failures. The `qualifier`
section needs a wire from GPIO15 to CLK2 and `--clk2`; it is the one
section still unverified, and it also tells which input the qualifier
is.

Verified 2026-10-22 on two units: all 60 bench checks pass, the `bits`
and `csv` outputs match the libsigrok driver through sigrok-cli byte for
byte for the same captures, and a cold start (unconfigured FPGA after a
power cycle) configures and captures in about 4 s: the open sees no
answer, loads the image by bit-bang, resyncs once while the FPGA comes
up, and proceeds.

## Serial multimeters

Handheld meters with a serial cable, one driver id per meter as in
sigrok (`uni-t-ut61e-ser`, `voltcraft-vc820-ser`, `metex-m3640d`, ...;
`oxrok -L` lists them). The byte stream does not say which meter is
talking, so the driver id picks the chipset protocol and the cable's
serial parameters, `conn=` names the port and `serialcomm=` overrides
the parameters:

```
oxrok -d uni-t-ut61e-ser:conn=/dev/ttyUSB0 --samples 10
oxrok -d metex-m3640d:conn=/dev/ttyUSB0:serialcomm=1200/7n2/rts=0/dtr=1 --continuous
```

The scan listens for up to three seconds for a packet the chipset
parser accepts. Readings stream as analog packets, one per display, on
channels `P1` (and `P2` to `P4` for the four display Metex meters);
`--samples` counts packets, `--time` limits the run, `--continuous`
runs until stopped.

Chipsets ported: FS9721, FS9922, DTM0660, Metex 14 byte (single and
four display), VC-96, ASYC-II, M2110, UT71x, ES519xx (11 and 14 byte,
2400 and 19200 baud variants), VC-870, RadioShack 22-812, Brymen BM25x,
BM52x, BM82x, BM85x, BM86x, Digitech QM1578, EEVblog 121GW, Mastech
MS2115B and MS8250D, Meterman 38XR: all 69 meters of libsigrok's
serial-dmm driver. Multi display meters put each display on its own
channel (`P1`/`P2`, or `main`/`sub`/`bar` where sigrok names them so).
Not ported: the recordings a BM52x stores in memory.

The BM85x is the one meter with variable length packets: its protocol
frames the stream itself (`Protocol::frame`), and the driver pulses RTS
after opening the port as the meter requires. Nothing on this list past
the FS9721 and ES519xx families has met a real meter; see below.

### HID cables

Some cables are USB HID devices rather than serial ports: the Brymen
BU-86X infrared adapter (`brymen-bm52x`, `brymen-bm82x`,
`brymen-bm86x`), the Victor 70C and 86C cable (`victor-dmm`), and the
WCH CH9325 and Silicon Labs CP2110 UART bridges that other meters'
cables use. The `hid` module talks to them through `async-hid` (IOHIDManager
on macOS, hidraw on Linux, Win32 HID on Windows, no C code), and
the `serial` module wraps each chip's report framing as a port, so the
meter drivers see a byte stream as with a serial cable. The port name
follows sigrok:

```
oxrok -d brymen-bm86x --samples 5                       # finds the adapter on its own
oxrok -d brymen-bm86x:conn=hid/bu86x/sn=1234 --samples 5
oxrok -d uni-t-ut61e-ser:conn=hid/ch9325 --samples 5    # a serial meter on a HID bridge
```

`hid` takes any cable with a known USB id, `hid/CHIP` one of `bu86x`,
`ch9325`, `cp2110`, `victor`, `sn=` picks one by serial number and
`raw=PATH` names the node (`hid/ch9325/raw=/dev/hidraw3`; without a
chip, the chip comes from the node's USB id). `usb=` and `iokit=`, the
other path forms sigrok prints, are parsed but match no device yet:
oxrok's HID backend names devices by node only. On
Linux the user needs read and write access to the `/dev/hidraw*` node
(a udev rule for the cable's USB id). Meters that come with a HID cable
scan without `conn=`; an unplugged default cable is simply not found.

Verified 2026-10-23 with a Raspberry Pi Pico running the cable emulator
in `hil/pico-hidcable`: it enumerates as the
BU-86X, the CH9325 or the Victor cable (`MODE bu86x|ch9325|victor` on
its USB serial port, `PACKET hex`, `PERIOD ms`, `SAVE`; `cable.sh`
sends the commands) and plays a meter packet. `brymen-bm86x` on the
BU-86X, `uni-t-ut61e-ser:conn=hid/ch9325` and `victor-dmm` give the same
readings through oxrok and sigrok-cli; `tools/diffcli.sh --hw` runs
the three cases. The CP2110 has no emulation yet. sigrok-cli needed two
small fixes in libsigrok's `serial_hid.c` to work on this macOS at all
(the `DevSrvsID:` paths of current hidapi, and a double conversion of
found paths); they are applied in the local build only.

Emulated (2026-11-03): the three cables are also `oxrok-emu` models,
`bu86x`, `ch9325` and `victor`, after the Pico firmware and libsigrok's
`serial_hid_bu86x.c`, `serial_hid_ch9325.c` and `serial_hid_victor.c`,
sharing `emu/src/shells/hidcable.rs`:

- `bu86x`: 0820:0001, 8 byte reports. The BM86x answers the output
  report `00 86 66` with its 24 byte packet in three input reports.
  `packet=HEX[,HEX...]` gives the packets, played in turn; `drop=N`
  leaves every Nth request unanswered.
- `ch9325`: 1a86:e008, up to 7 UART bytes per report behind
  `0xf0|count`. The line parameters come as a 5 byte feature report
  (rate LE, 0, 0, bits - 5), checked against `baud=` (19200) and
  `bits=` (7). One packet per `period=N` polls of the interrupt
  transfer (50, about a second), restarted by each line setting.
- `victor`: 1244:d237, FS9922 packets scrambled into 14 byte reports,
  one per `period=N` polls (50).

`tools/emu/{bu86x,ch9325,victor}.sh` hold 27 cases with no violation
under either tool. 19 are full: scan, `--show`, sample limits 1 to 4, a
channel selection, csv, the chip taken from the node's USB id, a lost
BM86x request and a misaligned UT61E packet. 8 are oxrok only
(`--oracle-bug`): the names users type (`hid`, `hid/CHIP`, `sn=`, the
driver's default `conn`), which the unpatched libsigrok 0bc24877 cannot
open, since it converts a found cable's path twice
(`raw=hid/CHIP/raw=...`) and hands that to `hid_open_path`; only
`hid[/CHIP]/raw=PATH` works there. gcov of serial-dmm: 68.9% (bu86x),
61.4% (ch9325), 59.0% (victor). Not emulated: `limit_time`, the BM52x
hooks, transport errors. The models taught that sigrok holds the node
about 100 ms before it sets the line parameters (`try_open_path` opens
it once just to check), and that oxrok kept the probed HID port open
into the capture, skipping the second parameter report and reading
stale packets; it now reopens the cable, and flushes a kept UART, as
sigrok's `serial_open` does.

### UNI-T meters on the UT-D04 cable, and the UT32x

libsigrok's `uni-t-dmm` driver covers 27 meters whose UT-D04 cable is a
CH9325 HID bridge (`uni-t-ut61e`, `voltcraft-vc820`, `tenma-72-7730`,
`uni-t-ut372`, ...). oxrok registers the same ids as twins of the
serial entries over `hid/ch9325`, so they scan without `conn=` and also
take sigrok's `conn=1a86.e008` form. The UT372 tachometer parser came
with them. The UNI-T UT32x thermometer (`uni-t-ut32x`) has its own
driver: channels `T1`, `T2`, `T1-T2`, `data_source` `Live` or `Memory`,
`conn=` required since the cable cannot be told from any other UT-D04.
Both were exercised with the Pico emulator in CH9325 mode (a UT61E
stream for the meters, a 23.5 C packet for the UT32x); sigrok-cli has
no reference run for them on macOS, where libusb cannot claim a HID.

## USB relay boards and the PICkit2

`dcttech-usbrelay` switches the dcttech relay boards (`R1` to `Rn`
channel groups, `enabled` per group or for all at once):

```
oxrok -d dcttech-usbrelay --show
oxrok -d dcttech-usbrelay -g R2 --set -c enabled=on
oxrok -d dcttech-usbrelay:conn=sn=ABCDE -g R1 --get enabled
```

`microchip-pickit2` uses a PICkit2 as a three channel logic analyzer
(pins 4 to 6, 1024 samples, 5 kHz to 1 MHz, a one stage trigger on
levels and edges, `captureratio` presets 0 to 6 for the trigger
position):

```
oxrok -d microchip-pickit2 -c samplerate=1M --samples 1024 -O bits
oxrok -d microchip-pickit2 -c samplerate=250k:captureratio=2 -t pin5=r --samples 1024 -O csv
```

libsigrok reaches the PICkit2 with raw USB interrupt transfers on
interface 0. On Linux oxrok does the same: the kernel keeps usbhid off
the PICkit2 (it is on the HID ignore list), so there is no hidraw node
to use. macOS does not let a program claim a HID interface and Windows
binds its HID driver, so there oxrok speaks HID reports instead. On
Linux `conn` is the device's bus.address, as under libsigrok, and
`conn=04d8.0033` or `conn=BUS.ADDR` selects the device. Both drivers
were verified on macOS against the Pico emulator (`MODE usbrelay`,
`MODE pickit2`, which plays a counter on the three pins; firmware in
`hil/pico-hidcable`). A real PICkit2 has not been tried yet.

The PICkit2 is also a host model of the USB/IP emulator: `sudo
oxrok-emu serve pickit2 --attach --open-devices` puts a 04d8:0033
PICkit2 on the bus. It plays the Pico firmware's counter (pin4 toggles
every sample, pin5 every two, pin6 every four), and its status comes
15 IN offers (300 ms) after SETUP; `-o delay=N` changes that. A trigger
the counter can never meet (rising edges on pin4 and pin5 together) is
answered with the cancel status 0x4004 that a press of the button
gives. Emulated status, 2026-11-03: the 41 cases of
`tools/emu/pickit2.sh` (scan, show, selection by ids and by address,
probe names, get and set, captures at all eight rates, 1 to 5000
samples, two channels, triggers on levels and edges, a cancelled
capture, the `captureratio` presets 0 to 6, no trigger, and an ignored
ratio) give identical output and identical transactions under
sigrok-cli and oxrok, with zero pedantic violations under either; the
model also flags IN reads with no answer to come. The goldens are in
`tests/emu/pickit2`. The gcov report of the C driver over those cases
(`tests/emu/pickit2/gcov.txt`) is 83.2% of the executable lines (317 of
381; `api.c` 79.5%, `protocol.c` 86.9%). Not reached, and not emulated
on purpose: the kernel driver detach and reattach in `dev_open` and
`dev_close` (no driver is bound to a PICkit2 on Linux), USB transfer
errors and short transfers, stopping a capture by software while it
waits for the trigger, the `SR_ERR_ARG` and `SR_ERR_NA` defaults
sigrok-cli never asks for, multi-stage triggers, and the error returns
of the read-out. The model showed three oxrok bugs, all fixed: the
driver found no PICkit2 on Linux; nusb's claim reattached kernel
drivers on release, so usbhid read the serial number again; and
`--get` did not print values the way sigrok-cli's `g_variant_print`
does. Like the C driver, oxrok ignores a failed send and reads a short
packet again; a hard read error still ends oxrok's capture, where
libsigrok would poll a vanished device forever.

The relay board is also the first host model of the USB/IP emulator
(`docs/USB.md`): `sudo oxrok-emu serve usbrelay --attach
--open-devices` puts a `USBRelay2` (`-o relays=N` for another count)
on the bus, with the serial number `ABCDE` in its report. Emulated
status, 2026-11-02: the 11 cases of `tools/diffcli.sh --emu` (scan,
show, get, the four switch commands, selection by serial, by path and
by ids) give identical output and identical transactions under
sigrok-cli and oxrok, with zero pedantic violations under either. On
Linux `conn` is the hidraw node, as under libsigrok, and
`conn=/dev/hidrawN`, `conn=VID.PID` and `conn=sn=SERIAL` all select a
board. The cases are `tools/emu/usbrelay.sh`, their goldens
`tests/emu/usbrelay`. The gcov report of the C driver over those cases
(`tests/emu/usbrelay/gcov.txt`, 2026-11-02) is 78.1% of the executable
lines (211 of 270; `api.c` 77.5%, `protocol.c` 80.8%). Not reached, and
not emulated on purpose: the error returns when hidapi cannot open,
read or write the device or reports a short or failed transfer, HID
devices whose strings are missing or name another vendor or product
(the emulator is the only one on the bus), a non-printable serial
number, a `conn=sn=` that matches no board, the
`SR_ERR_NA` defaults of `config_get`, `config_set` and `config_list`
for keys sigrok-cli never asks, `dev_open` on a device already open,
and the hex dumps at log level 5. Of the emulator-verified criteria of
the roadmap, a replay fixture is still open.

## Raspberry Pi Pico (sigrok-pico firmware)

A Pico running the sigrok-pico firmware is a 21 channel logic analyzer
with 3 analog inputs over its USB serial port, driver `raspberrypi-pico`.
`conn=` is required; the Pico shows up as a CDC port (`/dev/cu.usbmodem*`
on macOS, `/dev/ttyACM*` on Linux):

```
oxrok -d raspberrypi-pico:conn=/dev/ttyACM0 -c samplerate=1M -C D2,D3,D4 --samples 100 -O bits
oxrok -d raspberrypi-pico:conn=/dev/ttyACM0 -c samplerate=100k -C A0,D2 --samples 50 -O csv
oxrok -d raspberrypi-pico:conn=/dev/ttyACM0 -c samplerate=1M -C D2,D3 -t D2=r --samples 1000 -O bits
```

Digital channels are `D2` to `D22` (the board's GPIO numbers) and must
be enabled from D2 up without gaps; analog `A0` to `A2` are GPIO 26 to
28 scaled to volts. The firmware takes any sample rate and the driver
rounds it to a common divisor when analog and digital run together, as
libsigrok does. Triggers run on the host (one stage, digital channels,
with `captureratio` of pre-trigger samples); the device streams until
told to stop. Rates the firmware cannot do abort the capture, the same
way in both tools (one analog channel at 1 MHz, for example).

Verified 2026-10-23 on a Pico with `pico_baseline.uf2`: `--show`, bits,
hex, ascii, csv with and without trigger marks, analog, mixed and
triggered captures are byte identical between oxrok and sigrok-cli;
two recorded sessions are the replay tests. Not yet exercised: a real
signal on the inputs (a wire from the ESP32 to GPIO 2 would do) and the
larger digital-only firmware variants.

## Rigol and Agilent oscilloscopes

The `rigol-ds` driver covers the same 61 models as libsigrok's: Rigol
VS5000, DS1000 (DS1052E, DS1102E, DS1152E and the D variants with the
logic pod), DS2000 and DS2000A/MSO2000A, DS1000Z and MSO1000Z, DS4000,
MSO5000, and the Agilent DSO1000A/B and MSO7034A. It speaks SCPI over
the `scpi` module, which carries USBTMC (the USB port on the scope), raw TCP,
Rigol's length prefixed TCP and serial ports:

```
oxrok -d rigol-ds --show                           # any USBTMC scope
oxrok -d rigol-ds:conn=usbtmc/1.5 --frames 1 -O analog
oxrok -d rigol-ds:conn=tcp-rigol/192.168.1.20/5555 -c timebase=1ms -C CH1 --frames 2 -O csv
```

Options follow sigrok: `timebase`, `triggersource`, `triggerslope`,
`triggerlevel`, `horiz_triggerpos`, `data_source` (`Live`, `Memory`,
`Segmented` as the series allows) and `limit_frames`; per analog channel
group (`-g CH1`) `vdiv`, `coupling` and `probe_factor`. Every setting is
written to the scope at once and the current values are read back when
the device opens. A capture downloads each enabled channel once per
frame, scaled to volts, between `FRAME-BEGIN` and `FRAME-END`; the
sample rate is the frame length over the screen width for `Live`
and `:WAV:XINC?` for the memory sources.

One difference from libsigrok on purpose: `horiz_triggerpos` is
converted to seconds with the number of horizontal divisions, where
libsigrok multiplies by the number of timebases in its table.

Verification: the Pico emulator has a `usbtmc` mode that is a Rigol
DS1102E on a real USBTMC interface (the DS1000 command set, a sine on
CH1 and a ramp on CH2, 600 point live frames). Against it, `--show` at
device and channel group level, single and dual channel frames, two
frame runs, and runs with `timebase`, `triggersource`, `triggerslope`,
`vdiv`, `coupling` and `probe_factor` set are byte identical between
oxrok and sigrok-cli; `tools/diffcli.sh --hw` runs those cases when
the emulator is in that mode. The memory data source fails on the
DS1000 series in both tools (libsigrok cannot read the sample rate
there). Verified 2026-10-23 on a real DS1102E as well (firmware 00.02.06.00.01):
`--show` at both levels and single, dual channel, two frame and
settings runs are byte identical with sigrok-cli; its recorded session
is a replay test next to the hand written one. The real scope answers
without line endings, writes block headers with eight length digits
and reports an `ALTERNATION` slope outside edge trigger mode, where
both tools decline the `triggerslope` query. Still open: the other
series, for which `OXROK_SCPI_RECORD=FILE` captures a fixture from any
run.

## FTDI logic analyzers (ftdi-la)

oxrok has no `ftdi-la` driver yet; the emulator checks libsigrok's
driver alone, so its cases are oracle cases that become full ones once
the driver is ported.

Emulated (2026-11-03): model `ftdi-la` (`emu/src/models/ftdi-la.rs`,
on the FTDI shell `emu/src/shells/ftdi.rs`), an FTDI chip with port A
in asynchronous bit-bang mode, all pins inputs. Sample n is n modulo
256, and the count restarts at every SET_BITMODE and at every purge of
the chip-to-host buffer. Options: `chip=` `ft2232h` (default),
`ft4232h`, `ft232h`, `ft232r`, `tumpa` (0403:8a98, found only with
`conn=`), `ft245bm` (the driver takes it for an FT232R) or `ft230x`
(0403:6015, which the driver rejects); `manufacturer=`, `product=` and
`serial=` replace a string, or remove it when empty, as an unprogrammed
EEPROM does. `tools/emu/ftdi-la.sh` runs 15 oracle cases on the
FT2232H: scan, show, get and set samplerate, get conn, captures at
3.6 kHz, 1 MHz and 10 MHz, a `--time` capture, `-C`, `conn=`, and the
error answers (samplerate below 3600, a samplerate beyond libftdi's 5 %
tolerance, triggers). No violation under sigrok-cli. The other chips
and the missing strings were served by hand, also with no violation;
they become cases once a scenario can switch options. gcov: 69.3 %
(178 of 257). Not emulated: failures of libftdi calls (string reads,
`find_all`, open, purge, bit mode, reads), `config_get` conn (the
driver never sets `sdi->conn`, so `--get conn` always fails with
"invalid argument") and `LIMIT_MSEC` (not advertised; sigrok-cli turns
`--time` into a sample count).

What the model taught about the driver: `dev_open` goes reset, 9600 Bd
(libftdi's open), purge both buffers, bit mode reset, bit-bang with
mask 0, and acquisition sets bit-bang again. The samplerate is sent as
a baud rate of samplerate / 20 (H chips) or / 30 (FT232R, FT232H),
which libftdi multiplies by 4 in bit-bang mode; with no samplerate set
the 9600 Bd of the open stays. Every read asks libftdi for 16 KiB in
4 KiB bulk transfers, so even a 100 sample capture reads about 20 KB.
The FT230X is listed by libftdi's scan but has no entry in the
driver's table.

## fx2lafw devices

Saleae Logic and its clones, the CWAV USBee models, the bare Cypress FX2
board, Braintechnology USB-LPS and the sigrok FX2 LA boards. The driver
is `fx2lafw`; 8 channels at up to 24 MS/s, 16 channels at up to
12 MS/s on the models that have them, triggers on the host.

Firmware: the sigrok `fx2lafw-*.fw` files (the `sigrok-firmware-fx2lafw`
package, or `/opt/homebrew/share/sigrok-firmware` from Homebrew's
sigrok). The driver uploads the file for the device's USB id when the
device does not report the firmware in its strings, then waits up to
3 s for it to renumerate.

Analog channels (USBee AX) are not supported.

Verified 2026-10-22 on a Saleae Logic clone (`0925:3881`, firmware 1.4):
the cold path after a power cycle (upload, renumeration in about 2 s,
reopen and claim), 60 s at 24 MS/s delivering every byte, and `--show`
identical to sigrok-cli. Trigger and timing checks against the
signal generator need a wire from the ESP32 to the analyzer; they are
not run yet.

## Oracle

The sigrok build that `tools/diffcli.sh` and the emulation work of
phase 9 compare against, built from source under `~/work/sigrok` with
its own prefix (`~/work/sigrok/install`; `. ~/work/sigrok/env.sh` puts
it on `PATH` and `LD_LIBRARY_PATH`), so no distribution package is
involved. Set up 2026-11-02 on Fedora 44:

| Project | Commit | Notes |
|---|---|---|
| libserialport | `21b3dfe5f68c205be4086469335fd2fc2ce11ed2` | master, 2024-09-24 |
| libsigrok | `0bc2487778e660f4d3116729b6f4aee2b1996bb0` | master, 2025-11-20, `0.6.0-git-0bc24877`; built with `CFLAGS="-O0 -g --coverage" LDFLAGS="--coverage"` for the phase 9b gcov reports, `--disable-bindings`; 86 of the 87 drivers enabled, `hp-3478a` off for want of libgpib (linux-gpib is not packaged for Fedora) |
| libsigrokdecode | `1577c4354b4caba6fcb62d5919a1d4a196ae4512` | the `libsigrokdecode-0.5.3` tag, the decoders oxrok follows (`docs/DECODERS.md`), with two one-line local patches to build against Python 3.14: `python3-embed` added to the module list in `configure.ac`, and the `PyEval_InitThreads()` call removed (a no-op since 3.7, gone in 3.13). master (`71f45144`, 2024-10-01) renamed the uart and spi annotation classes after 0.5.3, and 17 decoder cases differ against it; its library also refuses to load the 0.5.3 decoders |
| sigrok-cli | `f44dd91347e7ac797cefc23162b9fcf0b7329f1f` | master, 2024-08-26, `0.8.0-git-f44dd91` |

The roadmap's inventory named libsigrok `e6907e1e`, and the
attribution headers `20baa77f`; neither commit exists on sigrok.org
or the GitHub mirror, whose tips are both `0bc24877`, so they belong
to the fork that carries the LogicPort driver. `0bc24877` is the
upstream base used here until that checkout is available.

Build dependencies, in Fedora's names: `autoconf automake libtool
pkgconf-pkg-config glib2-devel libzip-devel libusb1-devel libftdi-devel
hidapi-devel bluez-libs-devel libieee1284-devel nettle-devel
check-devel python3-devel python3-gobject` (nettle for `rdtech-tc`).
Each project: `./autogen.sh && ./configure --prefix=$HOME/work/sigrok/install`
with `PKG_CONFIG_PATH=$HOME/work/sigrok/install/lib/pkgconfig`, in the
order of the table. The 46 software cases of `tools/diffcli.sh` pass
against this build.
