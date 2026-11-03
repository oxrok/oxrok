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
the three cases. sigrok-cli needed two
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
`hid[/CHIP]/raw=PATH` works there. Fault options, off by default:
`burst=N` caps how much a meter sends (ch9325: packets per open;
victor: in all), so a scan that finds nothing sends the same reports
whatever the time, and `burst=0` is a silent meter; bu86x takes
`lose=N[,N]` (requests unanswered), `stall=N[,N]` (the request's
SET_REPORT STALLed, so `hid_write` fails) and `meter=bm52x` (a BM52x
behind the cable). Their cases cover a scan that gives up, a scan that
warns it dropped a lot of data, a scan whose 128 byte buffer fills
with noise, two answers lost in a row, and a stalled request, which
the BM86x ignores and the BM52x does not. gcov of serial-dmm: 76.5%
(bu86x), 62.2% (ch9325), 59.8% (victor); `gcov.txt` also reports the
shared `serial.c`, `serial_hid.c` and the cable's file. Not emulated:
`limit_time`, transport errors (a STALLed interrupt IN never reaches
hidraw, usbhid clears it), and the buffer-full drop of
`handle_new_data`, which no meter can reach: its 256 bytes always hold
less than a packet once searched. The models taught that sigrok holds the node
about 100 ms before it sets the line parameters (`try_open_path` opens
it once just to check), and that oxrok kept the probed HID port open
into the capture, skipping the second parameter report and reading
stale packets; it now reopens the cable, and flushes a kept UART, as
sigrok's `serial_open` does. The fault cases found two more: oxrok's
scan dropped half its buffer and searched on for the full three
seconds, where `serial_stream_detect` gives up once 128 bytes hold no
packet, so oxrok found meters libsigrok does not; and a failed request
write failed oxrok's scan, where libsigrok's BM86x ignores the write
and the other meters end the scan with no device, or end the
acquisition only when the request goes from the timeout branch.

The CP2110 (`hid/cp2110`, 10c4:ea80) has no Pico emulation; the
`oxrok-emu` model `cp2110` (2026-11-03) is its first independent check.
It plays a CP2110 after `serial_hid_cp2110.c` and AN434 rev 0.4 with a
UNI-T UT61E on its UART: while the UART is enabled the meter sends one
14 byte packet per `period=N` polls (25, about half a second), in two
parts three polls apart, eight readings then ten packets of noise.
Another baud rate gives garbled bytes, 8 data bits put the parity bit
in bit 7, a parity mismatch sets the error bit, more than 480 bytes in
the FIFO the overrun bit, and Get Receive Data answers from the FIFO.
It checks report ids and lengths, baud 300 to 1000000, parity 0 to 4,
flow 0 or 1, data bits code 0 to 3, stop bits 0 or 1, and no data
before the UART is enabled. `tools/emu/cp2110.sh` holds 17 cases for
`uni-t-ut61e-ser`: 14 full (scan, show, 1, 2, 4 and 9 samples, csv,
analog, `raw=` without a chip, 7o1, 7e1, 7o2, baud rates out of range)
and 3 `--oracle-bug` search forms, then, with `burst=N` packets per
UART enable, 7n1, 8n1, 9600 baud and a silent meter, deterministic
whatever the scan's three seconds hold. gcov: serial-dmm 62.9% (158 of
251); the shared sources (`serial.c`, `serial_hid.c`,
`serial_hid_cp2110.c`) 56.7%. Not emulated: transport errors,
`write_bytes` (the UT61E takes no data), `drain` (serial-dmm never
calls it), the codes `serialcomm` cannot express, and the flow
branches.

What the model showed in libsigrok 0bc24877: `cp2110_set_params` reads
`serialcomm`'s flow (1 RTS/CTS, 2 XON/XOFF) with libserialport's
numbering (1 XON/XOFF, 2 RTS/CTS), so it refuses `flow=1` and turns
RTS/CTS on for `flow=2`; it sends parity with AN434 rev 0.4's codes (1
even, 2 odd) but data bits as 0 to 3 where rev 0.4 has 0x05 to 0x08
(pyserial sends 1 odd, 2 even; which a real chip follows is not
verified); and the analog output of an overload (`inf`) rests on an
undefined float to int conversion, so the goldens are x86-64's (INT_MIN)
and aarch64 would print another prefix. In oxrok it found: the purge
missing on open, a burst of reports that is one read in C, the sample
limit checked per packet instead of per read, a receive buffer that
dropped a good packet after noise (now `handle_new_data`'s rules,
`DMM_BUFSIZE` 256), a request sent where C defers it by the meter's
delay, and a panic on an overload reading.

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
(`tests/emu/pickit2/gcov.txt`) is 84.0% of the executable lines (320 of
381). Fault options, off by default: `button=0` (no cancel), and
`stall-in=N`, `short-in=N`, `stall-out=N` for the Nth transfer; the
fault cases add a stalled and a short status and read, a stalled
SETUP, an interrupt while waiting for a trigger and an unplug during
the read-out (the last three oxrok only: sigrok-cli has no SIGINT
handler, and libsigrok reads an unplugged PICkit2 for ever). Not
reached, and not emulated on purpose: the kernel driver detach and
reattach in `dev_open` and `dev_close` (no driver is bound to a
PICkit2 on Linux), a short send (a 64 byte interrupt packet is taken
whole or STALLed), stopping a capture by software while it waits for
the trigger (sigrok-cli never asks), the `SR_ERR_ARG` and `SR_ERR_NA` defaults
sigrok-cli never asks for, multi-stage triggers, and the error returns
of the read-out. The model showed three oxrok bugs, all fixed: the
driver found no PICkit2 on Linux; nusb's claim reattached kernel
drivers on release, so usbhid read the serial number again; and
`--get` did not print values the way sigrok-cli's `g_variant_print`
does. Like the C driver, oxrok ignores a failed send, reads again after
a failed read (ending the capture only on a disconnect), and reads a short
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
(`tests/emu/usbrelay/gcov.txt`, 2026-11-03) is 82.2% of the executable
lines (222 of 270). Options `serial=TEXT|0xHEX`, `stall-get=N`,
`short-get=N` and `stall-set=N` give the cases of a non-printable
serial number, a `conn=sn=` that matches no board, a stalled or short
report at the scan, a stalled state read and a stalled switch. They
found two oxrok differences, now fixed: open failed when the state read
failed, which libsigrok ignores, and a failed switch reported a hidraw
message instead of "input/output error". Not reached, and not
emulated on purpose: hidapi failing to open the device, HID devices
whose strings are missing or name another vendor or product (the
emulator is the only one on the bus), the
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

Ported (2026-11-03): `src/drivers/ftdi_la` (api.c, protocol.c) over
`src/ftdi/libftdi.rs`, an FTDI port that sends what libftdi 1.5 sends:
`ftdi_usb_open_dev` (claim port A, reset, 9600 Bd), `ftdi_tcioflush`
(-1 and -2), `ftdi_set_bitmode`, `ftdi_set_baudrate` (libftdi's divisor,
the AM's `ftdi_to_clkbits_AM` included, x4 in bit-bang, refused beyond
5 %) and `ftdi_read_data` (4 KiB bulk transfers, the rest of the last
one kept for the next read), with libftdi's error codes and messages.
The FT2232H, FT4232H, FT232H, FT232R (and a BM at 0403:6001) and the
TIAO adapter (0403:8a98, only with `conn=`) sample port A's eight pins
in asynchronous bit-bang mode, 3.6 kHz up to about 10 MHz; no trigger.

Where oxrok differs from libsigrok:
- `conn` is readable and the device line carries it
  (`ftdi-la:conn=BUS.ADDR`). The C driver advertises conn but never sets
  the `sdi->conn` that `config_get()` reads (api.c:390-394,
  scan_device() api.c:195-202): sigrok-cli prints no `conn=`, an empty
  `conn:` under `--show`, and `--get conn` fails with "invalid
  argument".
- The samplerate is kept in 64 bits. The C driver keeps it in a
  `uint32_t` (protocol.h:44), so `samplerate=4295967296` silently
  samples at 1 MHz; oxrok asks libftdi's divisor for it, which refuses.
- A samplerate set during an acquisition is refused; the C driver
  sends it between two reads.
- After closing, oxrok gives port A back to the kernel: `ftdi_sio` binds
  again and probes it (`GET_LATENCY_TIMER`, `SET_LATENCY_TIMER` 16 ms),
  so the serial port comes back. libftdi detaches `ftdi_sio` and leaves
  it detached. The next open by either tool detaches it again.

Emulated (2026-11-03): model `ftdi-la` (`emu/src/models/ftdi-la.rs`, on
the FTDI shell `emu/src/shells/ftdi.rs`), an FTDI chip with port A in
asynchronous bit-bang mode, all pins inputs. Sample n is n modulo 256,
and the count restarts at every SET_BITMODE and at every purge of the
chip-to-host buffer. Options: `chip=` `ft2232h` (default), `ft4232h`,
`ft232h`, `ft232r`, `tumpa`, `ft245bm` or `ft230x`; `manufacturer=`,
`product=` and `serial=` replace a string, or remove it when empty.
`tools/emu/ftdi-la.sh` runs 46 cases, all full (both tools): 16 on the
FT2232H (scan, show, the get and set cases, captures, the error answers,
among them a samplerate beyond 32 bits), the variants through the
emulator's `options` command, and 9 fault cases (idle reads, a halted
bulk IN mid capture and at the first read, and each request of
`dev_open` STALLed: libftdi's reset, both purges, the bit mode reset,
bit-bang). The 13 scan and show cases are `--oracle-differs` and
`--get conn` is `--oracle-bug` (the conn bug above); the samplerate
beyond 32 bits is `--oracle-differs`. "ftdi-la 1 MHz" and "ft232r 1 MHz"
are recorded to `tests/fixtures/ftdi_la/` and replayed by
`tests/emu_replay.rs`. No case shows the `ftdi_sio` difference:
sigrok-cli runs first in each, and leaves port A free for oxrok.

What the model taught about the driver: `dev_open` goes reset, 9600 Bd
(libftdi's open), purge both buffers, bit mode reset, bit-bang with mask
0, and the acquisition sets bit-bang again, ignoring its result
(api.c:461). The samplerate is sent as a baud rate of samplerate / 20
(FT2232H, FT4232H) or / 30 (FT232R, FT232H), which libftdi multiplies by
4 in bit-bang mode; with no samplerate set the 9600 Bd of the open
stays. Every read asks libftdi for 16 KiB in 4 KiB bulk transfers, so
even a 100 sample capture reads about 20 KB. The FT230X is listed by
libftdi's scan but has no entry in the driver's table.

## fx2lafw devices

Saleae Logic and its clones, the CWAV USBee models, the bare Cypress FX2
board, Braintechnology USB-LPS and the sigrok FX2 LA boards. The driver
is `fx2lafw`; 8 channels at up to 24 MS/s, 16 channels at up to
12 MS/s on the models that have them, triggers on the host.

Firmware: the sigrok `fx2lafw-*.fw` files (the `sigrok-firmware-fx2lafw`
package, or `/opt/homebrew/share/sigrok-firmware` from Homebrew's
sigrok). As libsigrok's driver, the scan uploads the file for the
device's USB id when the device does not report the firmware in its
strings (the scan line then has no `conn=`, which stays unknown until
the device is back), and open waits 300 ms and retries until 3 s for
it to renumerate, finding it again by its port path. Open sets
configuration 1 if it is not active (macOS needs it right after
renumeration), reads the firmware version and REVID, and then claims.

The USBee AX's analog input is supported: channel A0, in its own
channel group, read as volts, (code - 128) / 12.8, so -10 V to
+9.92 V. With A0 enabled the samples are 16 bit (the logic byte, then
the ADC's, clocked by the firmware's CTL2 pulses), so the rate is
limited to 12 MHz. A trigger cannot be set on A0.

Verified 2026-10-22 on a Saleae Logic clone (`0925:3881`, firmware 1.4):
the cold path after a power cycle (upload, renumeration in about 2 s,
reopen and claim), 60 s at 24 MS/s delivering every byte, and `--show`
identical to sigrok-cli. Trigger and timing checks against the
signal generator need a wire from the ESP32 to the analyzer; they are
not run yet. That check ran the open path of the time (upload on open);
the scan-time upload and the open order of 2026-11-03 are checked on
the emulator only and want a second run on the clone.

Emulated (2026-11-03): model `fx2lafw` of `oxrok-emu`, options
`profile=cypress-fx2|saleae-logic|sigrok-fx2-8ch|usbee-ax` and
`flashed=0|1`.
Without `flashed=1` it is an FX2 without firmware that takes the
upload through its boot loader and renumerates; it checks the upload
byte for byte against the stand-in files (`tools/emu/fx2lafw.firmware`)
in 4096 byte chunks between the CPUCS hold and release. With
`flashed=1` it starts with the firmware running (bcdDevice 0x0001, as
the real firmware reports). It checks the vendor requests, start
flags, sample delays and rates against `protocol.c` and streams a
counter from endpoint 2. Unlike the real device, which fills its
queued transfers as fast as it samples, the model completes one
transfer per transfer the host submits after the previous one
completed, so a run's transactions do not depend on timing; that is
an emulation artifact. Faults, off by default: `version=MAJOR.MINOR`,
`stall=version,start,revid` (STALL those vendor requests), `empty=N`
and `stall_in=N` (the first bulk IN transfers of each acquisition
complete empty, then STALL, one per submission). The model's facts are
checked against the firmware source (sigrok-firmware-fx2lafw):
bcdDevice 0x0001 (`.dw` is big endian), strings `sigrok`, `fx2lafw`
and the board name ("sigrok FX2 8ch" for 1d50:608c), the vendor
commands' answer lengths, and a STALL for any other request. The 49
cases of `tools/emu/fx2lafw.sh` (scan,
show, conn by ids, probe names, 20 kHz to 48 MHz on 8 channels, 50 kHz
to 12 MHz on 16, up to six transfers, triggers with capture ratio, in
later transfers and with frames, frames across transfers, a time
limit, the refused 16 channel 24 MHz, and the already-flashed start of
each board) give identical output and transactions with no violation
under either tool. One is oxrok only (`--oracle-bug`): a frame without
trigger that ends exactly at the end of a transfer, where libsigrok's
`receive_transfer()` hands its NULL soft trigger to
`soft_trigger_logic_check()` and crashes (`protocol.c:484`); oxrok
begins the next frame with the next transfer. Nine fault cases on the
flashed Saleae Logic: each vendor request STALLed, 64 and 65 empty
transfers, stalled bulk IN transfers, and the interface claimed by
another program (a usbfs helper in the scenario holds it). A second is
oxrok only: a firmware of another major version, which
`fx2lafw_dev_open()` reports but opens anyway (its `ret` still holds
the REVID read's `SR_OK`). Six more full cases: `stall_string=I` and
`short_string=I` fail the scan's string descriptor reads (the model's
`fail_standard` hook; a short answer is the 2 byte header, which libusb
reads as an I/O error), and an unplug mid capture ends both tools'
acquisition after 3 transfers. The `usbee-ax` profile (08a9:0014,
string 3 "CWAV USBee AX", from the firmware's `hw/cwav-usbeeax`)
streams, with CTL2 and 16-bit samples, the counter's low byte and an
ADC ramp that reads n/12.8 V at sample n (each 256-sample block one
step higher, so a misplaced transfer shows). It flags CTL2 without
16-bit samples, CTL2 with a low delay byte of 0 (the firmware's
`gpif-acquisition.c` says it never is), and 16-bit samples without
CTL2. Twenty-three AX cases: scan with upload and flashed, show, probe
names, conn by ids, A0 alone in analog and csv, logic plus A0, 20 kHz
over three transfers, 12 MHz, frames, a logic-only trigger, the
refused 16, 24 and 48 MHz, an unknown A1 and a channel-group get, all
full cases with identical output and transactions, one recorded as a
replay fixture; and four that pin oxrok where libsigrok is wrong. Two
`--oracle-differs` cases cover libsigrok's soft trigger with A0 (a
trigger with A0 enabled, and one on A0, which oxrok refuses), a third
mixed logic and A0 csv, and an `--oracle-bug` case a `probe_names`
spec starting with `-`, which crashes sigrok-cli (`api.c:353-356`
names 8 channels from a list `sr_parse_probe_names()` does not pad).
78 cases in all. gcov: 91.2% of the driver (`api.c` 88.6%,
`protocol.c` 93.1%), `mso_send_data_proc` and the analog setup
included; `usb.c` and `ezusb.c` 63.3%. Not emulated: a `libusb_open`
that fails at the scan, profiles matched by USB strings, upload
failures, a device that never renumerates, claim failures other than
a busy interface, transfer submit and resubmit failures, and
allocation failures.

What the model changed in oxrok's port, which now follows `api.c` and
`protocol.c`: the upload moved into the scan; open finds the device by
port path and fails on a failed REVID read; transfers count one byte
per sample even at 16 bits (16 bit captures used twice the size);
frames continue in the stream instead of restarting the device; a
failed bulk IN transfer counts as an empty one rather than ending the
capture; a refused start command fails the session's start; open
reads the version and REVID before it judges the version and claims
the interface after both, on every OS (on Windows the nusb backend
claims for the first control transfer); the
soft trigger is a line by line port of `soft_trigger_logic`; 48 MHz is
accepted on 8 channels; and nusb's bulk stream resubmits a transfer
when the next is asked for, as a libusb callback does, so no extra
transfer completes after a capture stops (the rusb backend's reader
thread still reads ahead). One C bug is not followed: libsigrok's soft
trigger takes its sample width from all logic channels of the device,
so on a 16 channel board with only D0 to D7 enabled it reads the 1 byte
stream as 2 byte samples and the data starts at half the trigger
offset; oxrok uses the stream's width, and the cases avoid that
configuration.

## Saleae Logic16

`saleae-logic16` (21a9:1001): 16 channels, 100 MHz / n or 160 MHz / n
(n up to 256, the driver lists 500 kHz to 100 MHz), at most 300 MHz
across the enabled channels. It needs `saleae-logic16-fx2.fw` and an
FPGA bitstream, `saleae-logic16-fpga-18.bitstream` for 1.8 V and 3.3 V
logic or `-33` for 5 V (`voltage_threshold`), which sigrok-util
extracts from the vendor's application.

Ported 2026-11-03 (`src/drivers/saleae_logic16`), emulator-verified:
the 87 cases of `tools/emu/saleae-logic16.sh` run under both tools,
and 86 match in output and USB transactions with no violation. The
87th, `-C 0 -c samplerate=1k`, is an `--oracle-bug` case: oxrok refuses
the rate where libsigrok divides by zero. Two mcupro cases are recorded
and replayed in `cargo test`; the clone takes no bitstream, so the
replay needs no firmware file. A failed start sends the abort
commands, as libsigrok's session stops a device whose start failed.
The driver reads its EP1 answers with 1, 8 and 16 byte bulk INs, which
nusb cannot submit (only whole packets): the emulator rounds bulk INs
to the packet size, which is what the device sees, and still reports
an answer longer than the URB as a babble. `captureratio` above 100 is
refused in the driver, where libsigrok's `sr_config_set()` refuses it
generically (`hwdriver.c:757`).

Emulated (2026-11-03): model `saleae-logic16`,
options `flashed=0|1`, `bitstream=old|new` (the two FPGA register maps
of `protocol.c`) and `variant=original|mcupro` (the clone whose FPGA
holds its design). The FX2 takes its firmware through the boot loader
and renumerates as "Saleae LLC" / "Logic S/16"; commands go to bulk
EP1 (64 byte packets) scrambled by `encrypt()`, which the model undoes
and applies to its answers, so a host that sends plain bytes or keeps
the scrambler's state across transfers is refused. The bitstream comes
in 62 byte chunks after `0x7e` and is checked byte for byte against
the stand-in; the FPGA is then primed with the EEPROM's 16 bytes from
address 16 and their `map_eeprom_data()`, checked as `prime_fpga()`'s
whole write sequence (MODE brackets, the PRIMER_CONTROL pulse, the
final MODE). Samples are 16-bit words of 16 samples of one channel,
first sample in bit 15, round the selected channels. Faults, off by
default: each command or its answer STALLed or its answer short
(`stall=`, `stall_reply=`, `short_reply=`, `NAME[:N]`), a wrong abort
answer, another FPGA version, flipped status or mode bits, a FIFO
overflow, and empty, stalled, short or odd-length sample transfers.
The 85 cases of `tools/emu/saleae-logic16.sh` cover scan, show,
selection, the driver's rates and two off its list on both base
clocks, the 5 V re-upload, triggers, a time limit, refused rates and
thresholds, the flashed board with the new bitstream and the mcupro
clone, a claimed interface, 44 faults (the version, status, mode and
overflow cases show the driver's warnings) and an unplug mid capture
and at three points of the FX2 upload, with no violation under
sigrok-cli. gcov: 91.6% (`api.c` 85.8%, `protocol.c` 96.4%); `usb.c`
and `ezusb.c` 67.6%. Many device facts (bcdDevice 0, no serial, the
EEPROM beyond the bytes the driver reads, FPGA versions 0x10 and 0x13)
come from the C driver alone; no firmware or recording was at hand.

What the model taught: the driver divides 0 by 0 at `api.c:544`
(`500 * bytes_per_ms() / get_buffer_size()`, both 0 under 8000 bytes
per second) from `get_timeout()` before it checks the rate, so
`-C 0 -c samplerate=1k` kills sigrok-cli with SIGFPE. And libsigrok's
`sr_parse_sizestring()` (`strutil.c:1213-1244`) skips the suffix after
a fraction, so `samplerate=12.5M` is 12 Hz (and then hits the crash)
and a bare `12.5` reads past the string's end; `12500k` works.

## DreamSourceLab DSLogic

`dreamsourcelab-dslogic`: the DSLogic, Pro, Plus and Basic and the
DSCope (2a0e:0001, 0003, 0020, 0021, 0002). oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model
`dreamsourcelab-dslogic`, options `profile=plus|basic|pro|dslogic|dscope`
(the Plus by default) and `flashed=0|1`. The FX2 takes its firmware,
renumerates as "DreamSourceLab" / "USB-based Instrument", and every
open loads the FPGA bitstream over bulk OUT 2 after vendor request
0xb3 and writes the threshold register; both uploads are checked byte
for byte against the stand-ins. The model decodes the 360 byte setting
packet (`struct fpga_config`) field by field: syncs and headers, mode
bits, a divider of a listed rate, half and quarter mode only at
divider 1, RLE exactly where the driver's rule sets it, a trigger
position in 64 sample steps within the memory, the trigger stages as
`set_trigger()` writes them (stage 0 at its defaults without a
trigger, no value bit under a mask, nothing on a disabled channel).
It answers the 512 byte trigger packet and streams a counter in 64
sample atoms per channel on bulk IN 6. The global trigger word the
driver leaves uninitialised without a trigger is logged as `xxxx`.
Faults, off by default: version, each vendor request, the FPGA and
setting writes STALLed or short, the trigger packet STALLed, short or
never sent, empty, stalled or short sample transfers, each step of the
FX2 upload. The 80 cases of `tools/emu/dreamsourcelab-dslogic.sh`
(10 kHz to 400 MHz, 1 to 16 channels, five trigger kinds, stream mode,
external clock, threshold, RLE, every profile, the flashed start, 22
faults with an unplug and a claimed interface) log no violation under
sigrok-cli. gcov: 90.2% (`api.c` 87.0%, `protocol.c` 91.9%); `usb.c`
and `ezusb.c` 72.0%. Open questions without DSView's FPGA source:
whether the FPGA reads that trigger word without a trigger (if so, the
uninitialised bytes change captures), and how a buffered capture ends
(`count = limit_samples / 16` truncates, so `--samples 65` might come
up short).

What the model taught about the C driver: the trigger word goes out
uninitialised; `clock_edge` always reads back "rising"; the first
DSLogic loads its 3.3 V bitstream but reports the 1.4-3.6 V range; the
90 % trigger-position bound overflows 32 bits on the 256 Mibit boards
(about 10 %, found by reading); a STALLed or short trigger packet is
freed without a branch for it, so the capture never ends and a later
abort cancels freed memory; after a failed FX2 upload the driver opens
the bare FX2 and asks it for its version; handles leak in `scan()`,
`dslogic_dev_open()` (again on each retry) and
`ezusb_upload_firmware()`.

## Kingst LA2016 family

`kingst-la2016`: the LA2016, LA1016 (and their A1 boards), LA1010,
LA5016 and LA5032. oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model `kingst-la2016`
(`emu/src/models/kingst-la2016/`), options
`model=la2016a1|la2016|la1016|la1016a1|la1010a1|la5016a1|la5032a0`,
`mcu=1` (firmware running) and `fpga=1` (bitstream loaded). The bare
FX2 takes `kingst-la-01a2.fw` (the name is built from the PID) and
renumerates; the EEPROM holds the model's magic in a primary and a
backup copy with their complements, the bitstream goes up in 4096
byte chunks with zero padding, and a loaded FPGA is reused unless a
PWM output was left running or the run state is unexpected. Captures
advance by 50 ms of samples per poll of the run state (the driver's
poll interval, counted in polls), in the run-length encoded memory the
driver downloads; streaming, the PWM outputs, thresholds and triggers
follow `protocol.c`. Faults, off by default: any request STALLed
(`stall=WHAT[@N]`), FPGA init answers, run states, PWM left on, EEPROM
contents, extra or short data, a firmware that never runs. The 138
cases of `tools/emu/kingst-la2016.sh` (all seven models, 3051 Hz to
500 MHz including rates off the list, time limits with and without a
pending trigger, triggers with capture ratios 0 to 100, streaming, PWM,
groups, a claimed interface, every vendor request failing, an unplug
mid download) log no violation under sigrok-cli. gcov: 91.2%
(`api.c` 88.7%, `protocol.c` 92.7%); `usb.c` and `ezusb.c` 69.8%.

What the model taught: when `la2016_receive_data()` fails with the
bulk transfers in flight (a refused download or stream start), it
ends without cancelling them and `dev_close()` frees them, and libusb
aborts (three cases expect it); `run_state()` returns a failed read as
0xfff5, whose idle bit makes the driver download a capture that has
not finished; at the lowest rate the driver accepts (3051 Hz on the
LA2016) the divider is truncated to 16 bits and the device samples at
12.5 MHz; the driver passes on every sample of the transfer in which
the limit falls; and sigrok-cli parses float options in single
precision (20000001 Hz is 2e7).

## Hantek 6022BE/BL and relatives (hantek-6xxx)

`hantek-6xxx`: the Hantek 6022BE and 6022BL, Voltcraft DSO-2020,
Sainsmart DDS120, YiXingDianZi MDSO and Instrustar ISDS205B, two
channel USB scopes on an FX2LP with sigrok's own firmware
(sigrok-firmware-fx2lafw). oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model `hantek-6xxx`, options
`profile=6022be|6022be-04b5|dso2020|dds120|6022bl|6022bl-04b5|mdso|isds205b`
and `flashed=0|1`. It follows the firmware source (0.1.7-10-g0f2d324:
`scope.inc`, `hw/*/fw.c`, `dscr_scope.inc`): the scope renumerates as
1d50:608e with bcdDevice telling the boards apart, vendor commands
0xe0 to 0xe6 each stop sampling first, values a board does not know
are ignored and other requests STALL, and the stream interleaves one
byte of CH1 and one of CH2 on bulk IN 6. The model holds the driver to
its own tables and checks that the rate, both gains and (on the DDS120
and ISDS205B) the coupling are sent before every start. Inputs are
invented: a 1 kHz square wave on CH1, a 10 kHz triangle on CH2; the
gains are the driver's own V/div table. Faults, off by default: each
configuration request and each upload step STALLed, empty or stalled
transfers, short packets, another board's firmware. The 86 cases of
`tools/emu/hantek-6xxx.sh` (all eight profiles flashed or not, every
rate from 100 kHz to 48 MHz, every range, coupling, single channels,
get and set, faults, a claimed interface, an unplug) log no violation
under sigrok-cli, plus two `--hangs` cases. gcov: 91.0%;
`ezusb.c` 82.6%, `usb.c` 63.8%. Not emulated: the end of a capture by
a plain `--time` limit (wall time), a continuous capture.

What the model taught: with CH1 disabled, CH2's samples come out
labelled CH1; a bulk transfer that completes empty or fails after the
flush is neither resubmitted nor ends the capture, so it hangs for
ever (two `--hangs` cases: an empty and a stalled transfer after the
flush); `coupling=DC` on a DC-only board
reads back AC; `num_vdiv` cannot be read; a rate off the list is sent
as 100 kHz while the output is labelled with it; configuration errors
are logged and the capture goes on with the old settings; three
ISDS205B ranges (1.28 V, 57.6 mV, 17.6 mV) cannot be set from
sigrok-cli; and the open after an upload ignores bcdDevice, so a scope
with another board's firmware opens.

## Hantek DSO-2090 family (hantek-dso)

`hantek-dso`: the Hantek DSO-2090, 2150, 2250, 5200 and 5200A. oxrok
has no port yet.

Emulated (2026-11-03), oracle cases only: model `hantek-dso`, options
`profile=dso-2090|dso-2150|dso-2250|dso-5200|dso-5200a`, `flashed=1`,
`speed=full`, `ext=1`, `fill=N`, `wrap=N`. The FX2 takes
`hantek-dso-*.fw` and renumerates from 04b4 to 04b5; the model follows
`protocol.c`'s bulk commands behind the 'begin' request, the capture
state polls, triggers, timebases, ranges, coupling and filters, the
EEPROM calibration, and a Gray-coded trigger position in the state
answer. Faults, off by default: each request STALLed
(`stall=WHAT[@N]`), capture states (`state=V@N`), endpoint layouts the
driver refuses. The 132 cases of `tools/emu/hantek-dso.sh` (frames with
every trigger source, slope, ratio, timebase branch, range, coupling
and filter, full speed, the DSO-2250's own commands and each of CH2's
range groups, the upload and every step of the init and capture loop
failing) log no violation under sigrok-cli; frame outputs keep every
64th line plus a digest of the whole output (`--digest`). Five `--hangs` cases cover the paths that never end
(the driver retries capture errors for ever): an empty data transfer
(fault `empty=N`), an unplug mid frame and while polling, an external
trigger that never fires, and the DSO-5200's 9-bit frames ("Not yet
supported." at every poll). gcov: 96.2% (`ezusb.c` 82.6%, `usb.c`
67.7%). Not emulated: a second device. Known flake source: the driver submits a
frame's transfers at once with a 40 ms timeout each, and a timeout
hangs sigrok-cli; no run has hit it.

What the model taught: `--get num_hdiv` and `num_vdiv` always fail
(`config_get()` fills the value, then returns `SR_ERR_NA`); a frame
whose trigger offset is 0 sends an empty analog packet, on which
`output/analog.c` frees its buffer twice and sigrok-cli aborts;
capture errors are retried for ever; probably, the DSO-2250's voltage
header bit 0x08 overlaps CH2's range bit, so CH2 at 5 V, 500 mV and
50 mV reads as 1 V, 100 mV and 10 mV; and the capture-state answer's
length is never checked.

## ChronoVu LA8 / LA16 (chronovu-la)

oxrok has no `chronovu-la` driver yet; the emulator checks libsigrok's
driver alone, so its cases are oracle cases.

Emulated (2026-11-03): model `chronovu-la`
(`emu/src/models/chronovu-la.rs`, on the FTDI shell), an FTDI FIFO chip
in front of the analyzer's logic and 8 MiB of memory. The command block
(LA8: divcount, 0xff, pattern, mask; LA16: divcount, 0xff, then pattern,
mask and edge mask high byte first) starts a capture; status-only reads
follow until the memory is full (`fill=N` polls, 2 by default, counted
in polls rather than time); then the whole 8 MiB comes out in the order
the driver de-mangles (1 MiB sections; LA8 byte pairs swapped except at
divcount 0); eight 0x01 bytes reset the sequencer. By default the first
stored sample is the one that met the trigger (an assumption: the driver
gets no trigger position); `pretrigger=N` keeps N samples before it. The
inputs carry t ^ t>>8 ^ t>>16 (LA8) or t ^ t>>16 (LA16). Options:
`model=la8|la16`, `pid=` (0x8867 by default, 0x6001 the "old" id),
`chip=bm|r` (the chip is not known), `product=`, `serial=`, `fill=`,
`pretrigger=`; faults `no-trigger=1`, `gap=N` and the FTDI shell's
`stall=`, `in-stall=`, `idle=`. `tools/emu/chronovu-la.sh` runs 42
oracle cases: scan, show, get and set, captures at 100 MHz, 1 MHz, the
slowest rate, with -C and conn=, with 100 pre-trigger samples, conn by
ids and two refused conn strings, the error answers (invalid samplerate,
too many samples, an edge trigger on the LA8); LA16 captures with state,
edge-only, mixed and high-channel triggers; the old id, an FT232R, no
serial, unknown and missing product strings; FIFO gaps in blocks 0 and
2, a halted bulk IN, each request of the open STALLed, unplugs in and
after the readout, and a trigger that never comes (a --hangs case
stopped after 3 s). The driver ignores the sample limit and always reads
and sends the whole memory, so captures use --digest. No violation under
sigrok-cli. gcov: 87.0 % (368 of 423). Reached but uncounted: cv_write's
error branch, which runs in the unplug case before sigrok-cli dies of
SIGSEGV. Not emulated: allocation failures, failed string descriptor
reads, NULL and default branches sigrok-cli never reaches, trigger
stages and disabled-channel triggers (sigrok-cli refuses them), a short
write (impossible for one packet), purge or reset failures after a
successful write, and a trigger in the last sample of a block.

What the model taught about the driver: the LA16 trigger search compares
single bytes (before the byte swap) with a 16 bit mask and an 8 bit
expected value, so channel 0 high gives a 1 byte packet of unitsize 2;
hex and bits output then crash (length - unitsize wraps) and csv shows
the rest of the block a byte off, while triggers on channels 8 to 15
always match the first byte and an edge-only trigger sends no trigger at
all. Any write failure recurses for ever between cv_write and
close_usb_reset_sequencer (an unplug mid capture overflows the stack).
The capture deadline adds the capture time in seconds to a microsecond
clock, so it is 10 s whatever the rate: a real LA8 times out at divcount
119 and up (below about 833 kHz), where a full memory takes longer; the
sum is also computed in float, which loses the clock's precision after
long uptimes. Every capture completes in the emulator only because the
memory fills in polls. The error message at protocol.c:142-143 prints
its two counts in the wrong order. A port should place the trigger at
the right sample of 16 bit data, report edge triggers, stop on a failed
write, size its deadline from the rate, and honour the sample limit on
the host side.

## ASIX SIGMA and SIGMA2 (asix-sigma)

`asix-sigma`: the ASIX SIGMA and SIGMA2; the OMEGA is found by the scan
but not opened. oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model `asix-sigma`
(`emu/src/models/asix-sigma/`), an FT245BM in front of a Xilinx FPGA
without flash. The driver configures it in async bit-bang mode on every
acquisition (suicide sequence, PROG pulse, INIT_B poll, bitstream as
CCLK/DIN pairs). The model unscrambles the stand-in netlists and checks
every bit clocked in. The FPGA then answers the FIFO command protocol:
register file, trigger LUT, run-length-compressed sample memory read row
by row; a forced stop stores one last cluster. Options:
`variant=sigma|sigma2|omega`, `serial=`, `product=`, `step=N`; faults
`init-busy=N`, `config=fail`, `id=N`, `scratch=stuck|bit0`,
`stop-polls=N` and the FTDI shell's `stall=`, `in-stall=`, `idle=`. 104
oracle cases (scan, show, get/set, captures at 50/25/30/100/200 MHz and
external clock, a full-memory wrap, LUT and edge triggers at 200 kHz to
200 MHz, run-length compression, variants, faults, 19 unplugs). gcov
89.4% (`api.c` 93.7%, `protocol.c` 88.0%). Not emulated: allocation,
session and limit failures, a missing firmware file, the OMEGA's open, a
second acquisition in one session, a stop during capture, and the
trigger functions the driver never builds.

What it taught:
- The driver ends captures on wall time: a time limit checked every 10
  ms. Deterministic cases need rates >=25 MHz with small limits, or
  `captureratio=100`, which removes the limit.
- With a trigger the sample limit is ignored and the whole memory region
  is output.
- A trigger in the first 28 events gets no marker.
- The last stored event is never decoded: the decremented stop position
  is used as a count. A wrapped capture decodes nothing of the last row
  it reads.
- Channel 16 as the external clock overflows the 4-bit pin field into
  `CLKSEL_RISING`.
- Unplugs and read faults end a capture with "USB data read/write
  failed" and exit 0.

## IKALOGIC ScanaPLUS (ikalogic-scanaplus)

`ikalogic-scanaplus`: oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model `ikalogic-scanaplus`
(emu/src/models/ikalogic-scanaplus.rs, FTDI shell): an FT232H
`0403:6014` with the product string `SCANAPLUS` in sync FIFO mode, and
the device id in EEPROM words 16-17. Behind it is an FPGA that takes
(register, value) pairs and streams run length compressed pairs: a count
and channel 9, then channels 1 to 8. The FPGA's register meanings and
its start point are assumed. Options: `devid=XXXXXX`, `product=`,
latency flushes `short=N:M[,N:M...]` (transfer N of the stream carries M
bytes, 0 the status bytes alone), and the shell's `stall=`, `in-stall=`,
`idle=`. 51 oracle cases (2026-11-03): settings, captures with sample
and time limits, id and product variants, short and odd reads, each open
request STALLed, read errors, and unplugs before every write with its
own error return. gcov: driver 93.8 % (288/307). Not emulated:
allocation failures, libftdi calls that send nothing over USB
(set_interface, set_chunksize), the default branches that check_key
keeps sigrok-cli from reaching, the NULL guards, and a partial bulk
write.

What the model found:
- libftdi reads in 16 KiB transfers on Linux, and ends a read early only
  on a transfer of the status bytes alone.
- With both limits set, the sample limit wins even when the time limit
  is smaller (protocol.c 358-367; cases "time 1 ms, 300000 samples" and
  "time 1 ms, 700000 samples"). A port should end on whichever limit is
  reached first.
- The first read is dropped as if it were 64 KiB whatever its length
  (protocol.c 345-350; case "short first read").
- A read of an odd length is decoded with a stale byte past its end, and
  every later pair is split (the TODO at protocol.c 352; case "odd
  read"). A port should carry the odd byte over to the next read.
- A failed start makes session.c stop the failing device too (session.c
  831-835), which logs "Cannot remove non-existing event source".

## Pipistrello OLS (p-ols)

`p-ols`: the Pipistrello board with the Open Logic Sniffer bitstream.
oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model `pipistrello-ols`
(`emu/src/models/pipistrello-ols.rs`): an FT2232H (`0403:6010`, product
string "Pipistrello LX45") whose port B carries SUMP to an Open Logic
Sniffer FPGA. The parser is strict: one command per write, little-endian
arguments, divider of 24 bits and flags of 16, and demux flags must
replicate the lower channel-group bits. Counts are taken as an OLS takes
them: an oversized read count reads the whole memory, and a delay beyond
the read count captures that long. The model answers "1ALS" and a
metadata block of keys the driver parses. It has four trigger stages
with level, start and edge masks, RLE on words with the count in the top
bit of the top enabled group, demux packing two 16-channel samples per
word, and disabled channel groups packed out of each word. Captures are
sent newest first. Capture time counts in port B reads that get no data,
each one 16 ms latency period of samples.

Options: `product=`, `manufacturer=`, `serial=`, `id=sla|ols|none|HEX`,
`metadata=none|HEX`, `name=`, `fpga-version=`, `channels=`, `memory=`,
`max-rate=`, `protocol=`, `ext-clock=HZ`, and the FTDI shell's `stall=`,
`in-stall=`, `idle=`.

100 oracle cases: scan, show, get and set, captures from 10 Hz to 200
MHz, channel groups, triggers of one to three stages, RLE, demux, the
flags, the refusals, metadata variants, ID and product faults, each
request of the open STALLed, read halts, 15 unplugs, and three `--hangs`
cases. gcov 97.0% (api.c 97.2%, protocol.c 96.7%). Not emulated:
allocation failures, `ftdi_set_interface` failing, partial writes
(libusb never makes them), paths sigrok-cli never takes (no device
instance, unknown keys, a limit below 4, triggers on disabled channels),
and a capture without a sample limit (sigrok-cli always sets one).

What the model found:
- A sample limit that is not a multiple of 4 (8 in demux) crashes
  sigrok-cli, because `rle_count` wraps in the non-RLE overrun path
  (protocol.c 557-563, 597-600).
- With two or more stages and `captureratio=0`, the driver sends
  `num_stages` samples from before its sample buffer (`trigger_at =
  -num_stages`, protocol.c 620 tests only -1).
- With two or more stages and a capture ratio above 0, the trigger
  marker is `num_stages - 1` samples early.
- Metadata without a memory size makes the read and delay counts
  0xffffffff, and sigrok-cli dies on the empty logic packet.
- These never end: a 4-stage trigger (no start bit), all 32 channels at
  200 MHz (each demux word taken for one sample), `captureratio=100`
  (delay count 0xffffffff).
A port should compute the counts without underflow; refuse more stages
than leave room for its start stage, or put the start bit on the last
user stage as openbench-logic-sniffer does (its protocol.c 567), with
the marker placed to match; test for any negative trigger position;
refuse a board that reports no memory; and stop on the device's own
sample count.

## LeCroy LogicStudio 16 (lecroy-logicstudio)

`lecroy-logicstudio`: oxrok has no port yet.

Emulated (2026-11-03), oracle cases only: model `lecroy-logicstudio`
(emu/src/models/lecroy-logicstudio.rs, EZ-USB shell): an FX2LP that
enumerates as `05ff:a001`, takes `lecroy-logicstudio16-fx2lp.fw` in the
scan and renumerates as `05ff:a002`. Behind it is an FPGA that is loaded
at the first acquisition with one of two 464196 byte bitstreams (8 or 16
channels, over bulk OUT 6), is programmed with big endian register
triplets (vendor request 0xb1), reports its state in 32 byte packets on
interrupt IN 1, and sends its 40960 byte sample memory on bulk IN 2 (17
KiB, then 16 KiB at most). Everything comes from the C driver; the
register meanings, the trigger semantics, the 0x1c pipeline delay and
the trigger latency are assumed. Options: `flashed=1`, `still=1`, and
the faults `stall=NAME[:N]`, `verify=N`, `settle=N`,
`stall_bitstream=N`, `stale=N`, `odd_state=0xNN`, `intr_short=N`,
`bulk_short=N`, `bulk_stall=1`, `renum=0`. 27 oracle cases (2026-11-03):
scan with and without the FX2 firmware, show, the options, every way of
asking for a capture, an interface claimed by another program, a
firmware that never renumerates, and unplugs during the FX2 upload.
gcov: api.c 84.2 % (154/183), protocol.c 2.1 % (10/479). sigrok-cli
cannot start a capture with this driver: it runs a session only with
`--samples`, `--time`, `--frames` or `--continuous`, and the driver
offers none of their keys (api.c 44-48), so each fails before the
acquisition. The capture side (bitstream, trigger, status packets,
fetch) is held by the model's unit tests. It was checked once through
libsigrok directly, with a small C program not kept in the tree: 45
runs, no violations, protocol.c 94.6 %.

What the model found:
- sigrok-cli has no way to capture with this driver (see above); only a
  libsigrok client that calls `sr_session_start()` itself can.
- `conn=` cannot select the board: the driver lists no scan options, so
  `sr_driver_scan()` returns no device (hwdriver.c 512-515, 583-586;
  case "conn refused").
- 1 GHz is accepted (protocol.c's rate table) but not listed (api.c's);
  case "1G off the list".
- Without a trigger the driver never arms the board, and its interrupt
  transfer is not resubmitted after a timeout (protocol.c 926-933,
  1119), so the session never ends. A trigger only on disabled channels
  does the same.
- A failed 0xb3 is logged with `libusb_error_name(ret)` instead of `r`
  (protocol.c 452-454), so a STALL reads "LIBUSB_ERROR_IO".
- `recv_bulk_transfer()` never checks the transfer status (protocol.c
  1017-1027): a STALLed or unplugged sample endpoint is resubmitted
  forever.

## Saleae Logic Pro 16 (saleae-logic-pro)

`saleae-logic-pro` (21a9:1006, the only id the driver knows; it calls
the device "Logic Pro 16"): 16 digital channels at 1, 2, 2.5, 10, 25 or
50 MHz. The driver has no analog input and no sample or time limit, so
sigrok-cli captures only with `--continuous`. It needs
`saleae-logicpro16-fx3.fw` (Cypress FX3 firmware: a list of parts, each
with a 7-byte header) and `saleae-logicpro16-fpga.bitstream`. oxrok has
no port yet.

Emulated (2026-11-03), oracle cases only: model `saleae-logic-pro`,
options `flashed=0|1`, `fpga=0|1`, `boot_strings=MANUFACTURER/PRODUCT`
and `transfers=N`. The FX3 boot loader takes the parts through vendor
request 0xa0 with a 32-bit address in wValue/wIndex, an empty part
starting the firmware; the model checks them against a stand-in
container written by `tools/emu/saleae-logic-pro.sh`, then renumerates
as "Saleae" / "Logic Pro". Commands go to bulk EP1 and answers come back
on EP1, both XORed with an LFSR that the reseed zeroes (0 is a fixed
point, so the open runs in plain bytes); each capture's authentication
then keys the LFSR from a crypto chip on the FX3's I2C bus (wake,
Random, two Nonces, Sign), whose packets carry a CRC-16 that reproduces
the driver's three hard-coded CRCs; the key is the last nonce XOR the
Sign answer's CRC, and the model keeps the same state, so any byte
scrambled out of turn is refused. The bitstream comes in 1020-byte
chunks, checked byte for byte; the registers are held to the driver's
values (the ADC index/LSB/MSB triples, the 16 EEPROM magic bytes written
back to 0x17, the rate divider and pre-divider, the mux count). Samples
are 32-bit LE words of 32 samples of one channel, first sample in bit
31. A real board streams until stopped; as an emulation artifact the
model stops after `transfers=N` 16 KiB transfers. Faults, off by
default: any command or its answer STALLed or short, bad status bytes,
the chip's wake status, an FPGA that does not come up, a capture error,
a halted or short sample endpoint, a boot loader refusing a part,
unreadable strings. The 101 cases of `tools/emu/saleae-logic-pro.sh` log
no violation under sigrok-cli: every rate, 1 to 16 channels, selection
by the board's real address (read from sysfs, as the session's address
goes stale after an upload), the unflashed upload, every failing request
of the open, each authentication step, broken firmware files and a
missing bitstream, a claimed interface, unplugs during the upload,
prepare and mid capture. gcov: 97.0% (`api.c` 93.9%, `protocol.c`
98.4%); `usb.c` 79.9%. Not emulated: argument checks, libusb open
failures, sigrok-cli-unreachable config branches, and the driver's own 1
s idle stop (`api.c:391`; unreached only because the harness gives
sigrok-cli /dev/null on stdin, so its 'any key' stop fires at once).
Device facts beyond the ids, strings, endpoints and the driver's packets
are assumed: the boot stage's lack of strings, the firmware version,
temperature, EEPROM, the chip's random values and signature, the
stand-in addresses, high speed rather than USB 3.

What the model taught: a `conn=` that matches nothing, or that libsigrok
refuses, selects every Logic Pro (`api.c:219-236`), and after the upload
an unflashed board's address changes, so `conn=BUS.ADDR` always falls
into this; `receive_data()` converts all 16 KiB whatever the transfer's
length (`protocol.c:906`), so short transfers come out as stale or
never-written samples; two error messages print the wrong byte
(`protocol.c:340-341`, `539-540`); `prepare()` ignores a failed
authentication; `transact()` leaks every scrambled request
(`protocol.c:107`), and the stop leaves the 512 transfers submitted.

## Hantek 4032L (hantek-4032l)

`hantek-4032l`: the Hantek 4032L, 32 channels in groups A and B. oxrok
has no port yet.

Emulated (2026-11-03), oracle cases only: model `hantek-4032l`
(`emu/src/models/hantek-4032l.rs`, a plain vendor device with no shell),
options `fpga=VERSION` (default 0x4303; 0 is the driver's old variant,
with one data transfer at a time and no external clock) and `fill=N`
(status answers of 1 before the capture is done, default 1). The model
follows `protocol.c`: the 0xb3 reset request with its ten bytes, the
84-byte command packet on bulk OUT 2 (configure, status, get; the
struct's pack(2) bitfield layout checked against gcc), the 20-byte
status packet with the FPGA version on bulk IN 6, and the sample stream
between its start and end words, read as 512 bytes and then 2048-byte
transfers. The signal is a counter (channel n toggles every 2^n
samples). Triggers combine levels with one edge, the capture starts
`pre_trigger_size` samples before the trigger point, and a condition the
counter never meets keeps the status at 1. Pacing is counted in status
commands, not time. Faults, off by default: `stall=reset|out|in[@N]`,
`stale=N` (leftover packets before each answer), `end=HEX` (a garbled
end word). The 121 cases of `tools/emu/hantek-4032l.sh` log no violation
under sigrok-cli: every rate code, sizes up to 256k samples, thresholds
(including the -9 V that sends PWM 4096), external clock sources and
edges, every trigger kind and position (a capture ratio of 100
included), every get and refused value, FPGA version 0, a claimed
interface, the open's and the capture's STALLs, stale packets, an unplug
while polling and mid data. Three `--hangs` cases (2 s each) cover the
paths that never end: a refused configure, a refused get, and a trigger
that never fires. gcov: 90.7% (`usb.c` 68.1%). Not emulated: submissions
that fail (their timing against an unplug is not deterministic),
transfers after an abort (only `dev_acquisition_stop()` causes them, and
sigrok-cli never calls it), a second device, and refusals sigrok-cli
makes before the driver.

What the model taught: the driver ignores failed transfers (`sr_dbg`
only), so a refused configure or get keeps sigrok-cli running for ever,
and a failed status read is judged on the previous answer;
`limit_samples` of 2^32 and more loses its upper 32 bits (api.c:415); a
threshold of -8.2 V or below sends PWM 4096, one more than the 12-bit
maximum the code's comment gives; a capture ratio of 100 makes the
pre-trigger count equal to the sample count, so the trigger falls just
past the capture and sigrok-cli shows no marker; the poll transfer that
read the stream's start is leaked. The end-word check (protocol.c:194)
would read 4 bytes past the buffer if the last data transfer were full
of samples, as with a board that sends the start word in a packet of its
own (not the model's layout).

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
