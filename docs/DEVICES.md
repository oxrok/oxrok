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

Emulated on the CDC-ACM shell: model `serial-dmm`
(`meter=metex|es51922|fs9721`; `packet=`, `noise=`, `flood=`, `period=`,
`burst=`, `answers=`, and the shell's faults), 29 cases on
`metex-m3640d`, `uni-t-ut61e-ser` and `uni-t-ut60a-ser`, 2026-11-04.
gcov 71.3% of the driver. Not emulated: the HID and variable-length
meters. libsigrok swaps odd and even parity, which the harness undoes in
its `.tx`; see USB.md.

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
serial entries over `hid/ch9325`, so they scan without `conn=`. The UT372 tachometer parser came
with them. They were exercised with the Pico emulator in CH9325 mode (a
UT61E stream); sigrok-cli has no reference run for them on macOS, where
libusb cannot claim a HID.

libsigrok's `uni-t-dmm` is ported as `src/drivers/uni_t_dmm`, 27 ids.
`conn=VID.PID` and `conn=BUS.ADDRESS` use raw USB as the C does: on the
first poll usbhid is detached, interface 0 is claimed, the line is set
with `baud 00 00 03` (8 data bits, so a 7o1 meter's parity arrives in
bit 7 and is masked), then one 8 byte interrupt read on 0x82 per poll.
No `conn=` and `conn=hid...` keep the HID route through the `serial_dmm`
twin, and so do ids of another chip (a CP2110's `10c4.ea80`), which the
C would send a CH9325 report.

On Linux usbhid stays detached after a raw capture, as with sigrok-cli
(logged at info level). Until a replug, no `conn=` and `conn=hid` find
no hidraw node and say "No devices found". Raw USB needs write access to
the usbfs node, not to hidraw (untested). A failed claim logs `Failed to
claim interface 0` and `HID chip init failed`, and the capture fails
(exit 1), where sigrok-cli exits 0.

Off Linux, `conn=VID.PID` of a known cable the HID layer sees goes to
the twin, since nusb cannot claim it (busy on macOS, unsupported on
Windows with HidUsb). On Windows a cable bound to WinUSB through Zadig
is no HID device and works raw only (untested); there a failed claim
names `conn=hid/ch9325`. `serialcomm=` on raw USB finds no device, as in
the C.

Emulated by model `uni-t-dmm`, with descriptors from the sigrok wiki's
lsusb: options packet, baud, parity, chunk, idle, burst, stall-report,
stall-in, short-in, hold. 78 cases (75 full, 1 --oracle-bug, 1
--oracle-differs, 1 library), gcov 97.4%, unreached: a failed detach and
a short control write. Not on hardware. The HE2325U cable and the
bcdDevice 12.00 variant are not modelled, and full speed is an
assumption from the CH9326's datasheet. The model found that oxrok's
fs9922 overload ignored the C's early return.

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
vusb's FTDI shell), an FTDI chip with port A in
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
DSCope (2a0e:0001, 0003, 0020, 0021, 0002).

Ported 2026-11-03 (`src/drivers/dreamsourcelab_dslogic`),
emulator-verified: the 94 cases of `tools/emu/dreamsourcelab-dslogic.sh`
run under both tools; 83 match in output and USB transactions, with no
violation under oxrok. Seven are `--oracle-differs`: `clock_edge` reads
back as set (`api.c:444` always answers rising); the trigger position's
90 % bound does not wrap (`protocol.c:374-376`, about 10 % on the 256
Mibit boards); a threshold outside 0 to 5 V is refused
(`protocol.c:519`); the first DSLogic reports the 0.7-1.4 V range of the
3.3 V bitstream it loads, where libsigrok reports 1.4-3.6 V
(`api.c:355-366`, two cases; whether that board has the threshold
register the open writes is unknown); a trigger of two stages (`0=rf`)
is refused, where libsigrok writes every stage to stage 0
(`protocol.c:386-421`); and a transfer cut inside an atom has 0 for the
bits it lacks, where libsigrok reads the old buffer
(`protocol.c:732-748`). Four are `--oracle-bug`: a STALLed or short
trigger packet ends the capture with an error and a stop command, where
libsigrok never ends (`protocol.c:1010-1026`); an interrupt while
waiting for a trigger sends the stop command (sigrok-cli has no SIGINT
handler); and a trigger on the last sample sends no empty packet, where
libsigrok's packet of length 0 kills sigrok-cli (`protocol.c:861-862`,
`output/hex.c:163`). The global trigger word is sent as 0 without a
trigger. Sample transfers have the C driver's timeout, so a capture that
stops inside a transfer keeps that part and gives up as libsigrok does.
The sample count saturates at 32 bits where C truncates, and a short
version or REVID answer is refused where C accepts it. Two flashed cases
are recorded and replayed in `cargo test`, with the firmware stand-ins.
Not verified on hardware.

Emulated (2026-11-03): model
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
FX2 upload. The first 80 cases of `tools/emu/dreamsourcelab-dslogic.sh`
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
LA5016 and LA5032.

Ported 2026-11-03: `src/drivers/kingst_la2016/`. The 151 cases of
`tools/emu/kingst-la2016.sh`: 129 full, 9 `--oracle-bug`, 13
`--oracle-differs`, no violation under either tool; replay fixtures with
the bitstream loaded. Not verified on hardware. Bulk IN transfers keep
the C driver's 500 ms timeout and hand over what they hold, so a slow
stream's samples arrive as they come; `slow-in=1` is a device that never
fills a transfer. The port refuses a rate whose divider does not fit 16
bits (the C driver truncates it: 3051 Hz on the LA2016 samples at 12.5
MHz, 1525 Hz on the 100 MHz models at 2.7 MHz, and 12207 Hz on the 800
MHz models truncates to 0 and dies of SIGFPE). It ends a capture on a
failed run state poll. It stops at the sample limit, where the C driver
passes on the whole transfer the limit falls in. It ends cleanly when a
download or stream fails to start with its transfers queued. It gives up
after 64 failed transfers in a row, saying the data is incomplete, where
the C driver retries for ever. Ctrl-C during a capture halts it and
downloads what it holds; during a stream or a download it ends there.
Firmware and bitstreams: `kingst-la-01a2.fw` and
`kingst-MODEL-fpga.bitstream`, as sigrok. With the libusb backend, reads
go one packet at a time, so a fast stream may overflow.

Emulated (2026-11-03): model `kingst-la2016`
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
Sainsmart DDS120, YiXingDianZi MDSO and Instrustar ISDS205B, two channel
USB scopes on an FX2LP with sigrok's own firmware
(sigrok-firmware-fx2lafw): `fx2lafw-hantek-6022be.fw`,
`fx2lafw-hantek-6022bl.fw`, `fx2lafw-sainsmart-dds120.fw`,
`fx2lafw-yixingdianzi-mdso.fw`, `fx2lafw-instrustar-isds205b.fw`.

Ported (2026-11-04), `src/drivers/hantek_6xxx`. The 102 cases of
`tools/emu/hantek-6xxx.sh`: 80 full, 12 `--oracle-bug`, 10
`--oracle-differs`, no violation under either tool; replay fixtures of
flashed boards (a replay cannot follow the id change of an upload). Not
verified on hardware. Like the C driver it keeps one transfer in flight,
throws away the first 1024 bytes, and sizes transfers from what the
limits leave. What a transfer holds when its 4 s timeout runs out is
data, as a transfer can outlast it (1 MiB at 100 kHz). The port cuts the
output at the sample limit, with the same transfers, where the C driver
passes on all a transfer holds (`--samples 8` gives 256 samples). A
rate, range or coupling the scope refuses fails the start, where the C
driver logs it and labels the samples with the setting it asked for. It
labels CH2 as CH2 with CH1 disabled; takes an empty or failed transfer
again and gives up after 64 in a row, where the C driver waits for ever;
ends after 4 s of silence or on an unplug; does not hang on a limit past
2^31 bytes; reports 100 kHz for a rate off the list, which is what the C
driver sends for it, and sizes transfers for 100 kHz where the C driver
sizes them for the rate asked for; reads back `coupling` as set; answers
`num_vdiv`. A firmware reporting another board's `bcdDevice` after an
upload opens, with a warning. Ctrl-C ends the capture within the
transfer in flight, up to 4 s. At 8 MHz and up with no limit, the
session's queue of 64 packets of 24 MiB could hold about 1.5 GiB behind
a slow output. New model option: `slow=N` holds bulk IN transfers after
the first N until the host cancels them.

Emulated (2026-11-03): model `hantek-6xxx`, options
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
transfers, short packets, another board's firmware. The cases cover all eight profiles flashed or not, every rate from 100
kHz to 48 MHz, every range, coupling, single channels, get and set,
faults, a claimed interface and an unplug. gcov: 91.0%; `ezusb.c` 82.6%,
`usb.c` 68.6%. Not emulated: the end of a capture by a plain `--time`
limit (wall time), a continuous capture.

What the model taught: with CH1 disabled, CH2's samples come out
labelled CH1; a bulk transfer that completes empty or fails after the
flush is neither resubmitted nor ends the capture, so it hangs for
ever; `coupling=DC` on a DC-only board
reads back AC; `num_vdiv` cannot be read; a rate off the list is sent
as 100 kHz while the output is labelled with it; configuration errors
are logged and the capture goes on with the old settings; three
ISDS205B ranges (1.28 V, 57.6 mV, 17.6 mV) cannot be set from
sigrok-cli; and the open after an upload ignores bcdDevice, so a scope
with another board's firmware opens.

## Hantek DSO-2090 family (hantek-dso)

`hantek-dso`: the Hantek DSO-2090, 2150, 2250, 5200 and 5200A.

Ported (2026-11-04): `src/drivers/hantek_dso`, emulator-verified, no
hardware. The 148 cases of `tools/emu/hantek-dso.sh` are 136 full, 11
`--oracle-bug` and one `--libsigrok`, with no violation; gcov 96.2% as
before. The port sends the C driver's transactions, reading a frame one
packet at a time. It answers `num_hdiv` and `num_vdiv` and sends no
empty analog packet. It reads again after an empty or timed out data
transfer: on the model, which keeps the bytes for the next transfer, the
frame completes; on a device where an empty packet means no more, it
ends the frame with an error after 64 such reads (about 2.56 s if they
time out). A capture ends on a disconnect, after 64 failures in a row of
one kind, a DSO-5200's 9 bit frames included (those are still not read),
and after 2112 unknown capture states in a row, enough to drain the
largest frame a stopped run left unread. It waits with no bound in state
0 (a trigger that never fires), state 1 (filling) and state 127 (the
device's timeout, which C says does not matter), until a stop; 64 states
127 in a row give one warning. The DSO-2250's voltage header stays as
the C driver sends it: that CH2 reads five times too large at 5 V, 500
mV and 50 mV rests on the model alone, so whether it is a bug is open.
The firmware is not available here (stand-ins); no real scope has run
this.

Emulated (2026-11-03): model `hantek-dso`, options
`profile=dso-2090|dso-2150|dso-2250|dso-5200|dso-5200a`, `flashed=1`,
`speed=full`, `ext=1`, `fill=N`, `wrap=N`. The FX2 takes
`hantek-dso-*.fw` and renumerates from 04b4 to 04b5; the model follows
`protocol.c`'s bulk commands behind the 'begin' request, the capture
state polls, triggers, timebases, ranges, coupling and filters, the
EEPROM calibration, and a Gray-coded trigger position in the state
answer. Faults, off by default: each request STALLed
(`stall=WHAT[@N]`), capture states (`state=V@N`), endpoint layouts the
driver refuses. The cases cover frames with every trigger source, slope, ratio, timebase
branch, range, coupling and filter, full speed, the DSO-2250's own
commands and each of CH2's range groups, the upload and every step of
the init and capture loop failing; frame outputs keep every 64th line
plus a digest of the whole output (`--digest`). They also cover the
paths on which the C driver never ends (it retries capture errors for
ever): an empty data transfer (fault `empty=N`), an unplug mid frame and
while polling, an external trigger that never fires, and the DSO-5200's
9-bit frames ("Not yet supported." at every poll). gcov: 96.2%
(`ezusb.c` 82.6%, `usb.c` 67.7%). Not emulated: a second device. Known
flake source: the C driver submits a frame's transfers at once with a 40
ms timeout each, and a timeout hangs sigrok-cli; no run has hit it.

What the model taught: `--get num_hdiv` and `num_vdiv` always fail
(`config_get()` fills the value, then returns `SR_ERR_NA`); a frame
whose trigger offset is 0 sends an empty analog packet, on which
`output/analog.c` frees its buffer twice and sigrok-cli aborts;
capture errors are retried for ever; on the model alone, so whether
it is a bug is open, the DSO-2250's voltage header bit 0x08 overlaps
CH2's range bit, so CH2 at 5 V, 500 mV and 50 mV reads five times too
large; and the capture-state answer's length is never checked.

## ChronoVu LA8 / LA16 (chronovu-la)

`chronovu-la`: the ChronoVu LA8 (8 channels, 100 MHz) and LA16 (16
channels, 200 MHz), ported in `src/drivers/chronovu_la` over
`src/ftdi/libftdi.rs`, emulator-verified, not checked on hardware. The
device always fills and sends its whole 8 MiB, so a capture takes the
full readout whatever the limit; the port reads all of it, since the
samples are spread over it, and sends only what the limit asks. It
places the LA16 trigger at the sample that met it, reports edge
triggers, resets once after a failed write, and waits for the trigger
for 10 s plus the capture time, counted in status-only reads paced at
least 16 ms apart. The count is exact and the wait is at least that
long; the remaining assumption is that a poll takes no more than 16 ms,
since a longer latency timer, which the driver never sets, stretches the
wait in proportion (255 ms makes a no-trigger run about 2.7 min at 100
MHz and 8.3 min at the slowest LA8 rate). An edge channel already at its
new level on sample 0 counts as the trigger, since the device stores
from its trigger; with samples kept before the trigger this can mark
sample 0 ahead of the real edge. A stop resets the sequencer and closes
the chip. `tools/emu/chronovu-la.sh` runs 52 cases: 32 full, 16
`--oracle-differs`, 4 `--oracle-bug`. Two replay fixtures cover the
read-error path, since a full readout would be about 17 MB of fixture.

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
`stall=`, `in-stall=`, `idle=`. `tools/emu/chronovu-la.sh` runs, among
others: scan, show, get and set, captures at 100 MHz, 1 MHz, the
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
but skipped, as in libsigrok. Ported in `src/drivers/asix_sigma/`, over
`src/ftdi/libftdi.rs`. Needs the netlists `asix-sigma-50.fw`, `-100.fw`
and `-200.fw` from sigrok-firmware. The FPGA is configured when a
capture needs another netlist than the one running, about 4 s; as in
libsigrok this is remembered across a close. Emulator-verified (model
`asix-sigma`, 110 cases: 107 full, 3 `--oracle-differs`), not on
hardware. An event is one sample up to 50 MHz, 2 samples at 100 MHz and
4 at 200 MHz.

Where the port differs on purpose:
- All of a wrapped memory's last row is decoded.
- A trigger in the first 28 events gets its marker.
- A forced stop that never completes ends after 1000 mode reads, or 100
  once a stop was asked for (libsigrok polls for ever).

Where it follows libsigrok on purpose:
- Without a wrap the last row's count is the low bits of the decremented
  stop position. That drops the last event if the register points after
  it (the comment at protocol.c:371-377), and is what stays out of an
  unwritten cluster if the register is late (protocol.c:1433-1435, model
  option `stop-mid=N`). More than that count could repeat the last
  sample about 65000 times, against a cost of one event. A hardware
  recording could settle it: capture a counter with `--time` and check
  whether the last sample continues the count.
- Channel 16 as the external clock writes 16 into the 4 bit pin field
  (api.c:578, protocol.h:137), which sets `CLKSEL_RISING`. With a rising
  edge this is what was asked if the hardware takes the field modulo 16;
  oxrok warns.
- A trigger lifts the sample limit, so the whole memory region is
  output, and captures end on wall time at low rates.

Emulated (2026-11-03): model `asix-sigma`
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
- The last row's count is one event short of the decremented stop
  position; whether that loses an event depends on what the register
  means, which only hardware can tell (see above). A wrapped capture
  decodes nothing of the last row it reads, a bug of
  protocol.c:1418-1421: that row is whole.
- Channel 16 as the external clock overflows the 4-bit pin field into
  `CLKSEL_RISING`.
- Unplugs and read faults fail a capture with "USB data read/write
  failed" (exit 1, where sigrok-cli exits 0).

## IKALOGIC ScanaPLUS (ikalogic-scanaplus)

`ikalogic-scanaplus`: the IKALOGIC ScanaPLUS, ported to oxrok
(src/drivers/ikalogic_scanaplus over src/ftdi/libftdi.rs), checked with
the emulator only, never on hardware. 53 cases, all running both tools
with identical transactions: 49 full and 4 `--oracle-differs`. Unlike
the C driver, oxrok drops the first 64 KiB counted in bytes, carries the
odd byte of a read over to the next, and ends on whichever limit is
reached first; a limit set during a capture and already passed ends it
rather than wrapping. After an unplug, the C driver logs "Failed to
close FTDI device (-1): " with libftdi's stale error string where oxrok
logs nothing, because nusb's release cannot fail (the line stays in
oxrok's code for the libusb backend, unexercised by any case). Reads use
libftdi's Linux 16 KiB transfers on every platform.

Emulated (2026-11-03): model `ikalogic-scanaplus`
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

Ported (2026-11-03): `src/drivers/pipistrello_ols` (api.c, protocol.c)
over `src/ftdi/libftdi.rs`, port B. `tools/emu/pipistrello-ols.sh` runs
107 cases: 93 full, 7 `--oracle-bug`, 7 `--oracle-differs`. "trigger
rising" and "rle one group" are recorded to `tests/fixtures/p_ols/`. The
model takes facts from the driver author's firmware source,
github.com/magnuskarlsson/Pipistrello_ols_verilog at 2423ef3 (hand read,
not simulated, not known to be the bitstream on boards).

Where oxrok differs from libsigrok:
- A limit that is not a multiple of 4 (8 in demux) keeps the newest
  samples instead of crashing.
- With RLE or such a limit the marker follows the board's word count; a
  marker before the first sample is no marker. With no RLE and a
  multiple-of-4 limit it is at the C's position.
- Four stages work, with the start bit on the last stage and the marker
  on its match.
- `captureratio=100` sends a delay of one unit.
- Channels 16-31 above 100 MHz are refused, as is a board reporting no
  memory.
- A board slower than 100 MHz starts at its maximum, or the rate below
  it the clock divides to, with FLAG_FILTER set as at any rate up to 100
  MHz; the C reads 0 Hz, sends divider 0 and no filter flag.
- After closing, oxrok gives port B back to the kernel if it detached
  `ftdi_sio`, which then probes the port.
- Known gap: a device that left without a transfer seeing it closes
  without an error, where libftdi returns -1.

libsigrok findings from the firmware source, expected bench results (no
bench yet):
- Marker position after the last user match: 100 MHz: +3 (n=1), +2
  (n=2), +1 (n=3). 50 MHz: +2, +1, 0. 33 MHz or less: +1, 0, -1. No rate
  correction is applied without a bench.
- In demux mode stage.v ORs the matches of the word's halves, and the
  driver writes masks to the low half only, so every trigger at 200 MHz
  fires at once. Both tools send this unchanged; replicating the masks
  into the upper half is the likely fix.
- rle_enc.v masks the top channel of both demux samples, so the C's
  clear at protocol.c:490, 508 is a wrong index with no effect.
- 0xffffffff as the delay count is 2^28 words (`fwd` has 26 bits,
  controller.v), about 2.7 s at 100 MHz, so "never ends" holds only at
  low rates.

Emulated (2026-11-03): model `pipistrello-ols`
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

`lecroy-logicstudio`: ported to oxrok (src/drivers/lecroy_logicstudio),
checked with the emulator only, never on a board. A capture takes -O, -o
or -t with no limit, and needs a trigger. 94 cases: 26 full against
sigrok-cli, 46 full against `tools/libsigrok-run.c` (`emu_case
--libsigrok --oxrok`), 11 `--oracle-bug`, 11 library-only. "lib recorded
16ch" and "lib recorded 8ch upper" are recorded to
tests/fixtures/lecroy_logicstudio. Where oxrok differs from libsigrok: a
capture with no trigger is refused before the output is opened; a failed
transfer, a short status packet, a failed fetch write or a silent sample
endpoint (64 empty transfers) fails the capture, exit status 1; a status timeout polls again; no logic packet of 0 bytes; a
refused 0xb3 names the real error; where the C's subtractions would
wrap, the earliest sample is 0 and the trigger sample 1; `conn=` is
refused with "Unknown scan option 'conn'.". The whole model passes on
the libusb backend as well (83 cases under `--no-oracle`).

Emulated (2026-11-03): model `lecroy-logicstudio`
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
`bulk_short=N`, `bulk_stall=1`, `intr_silent=1`, `renum=0`. The
sigrok-cli cases cover the scan with and without the FX2 firmware, show,
the options, every way of asking for a capture, an interface claimed by
another program, a firmware that never renumerates and unplugs during
the FX2 upload. Captures run against `tools/libsigrok-run.c`, since
sigrok-cli cannot start a capture with this driver (it runs a session
only with `--samples`, `--time`, `--frames` or `--continuous`, and the
driver offers none of their keys, api.c 44-48): the rate classes, 8 and
16 channels, capture ratios, trigger stages, a wrapping memory, a stop
while waiting for the trigger, the faults and the bitstream file. gcov:
api.c 92.9 % (170/183), protocol.c 97.1 % (465/479). Not reached:
OS-level failures, a NULL sdi, match or transfer data, unknown keys,
more than 32 registers, another 05ff product or a second board, a status
packet with no trigger programmed, a failed libusb submit, and a match
on a disabled channel (sigrok-cli's trigger syntax refuses it).

What the model found:
- sigrok-cli has no way to capture with this driver; only a libsigrok
  client that calls `sr_session_start()` itself can
  (`tools/libsigrok-run.c`).
- `send_samples()` sends a logic packet of 0 bytes for any stretch of a
  whole number of its 4096 sample chunks with 8 channels (protocol.c
  980-989), and for an empty stretch with 16 (957-960, from 1062 when
  the trigger is where the read starts). libsigrok's outputs do not take
  it: bits, hex and ols loop to `length - unitsize`, which wraps (bits.c
  158, hex.c 163, ols.c 121), and die of SIGSEGV; csv dies of SIGFPE.
  Case "lib 8ch upper wraps bits crash"; the ascii output takes the same
  capture.
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
with a 7-byte header) and `saleae-logicpro16-fpga.bitstream`.

Ported (2026-11-04): `src/drivers/saleae_logic_pro`, on model
`saleae-logic-pro`, options as before. The 107 cases of
`tools/emu/saleae-logic-pro.sh`: 99 full (one, `lib idle stop`, against
`libsigrok-run`), 7 `--oracle-differs`, 1 `--oracle-bug` (Ctrl-C); 2
more run only under `--no-oracle` (a board named by address, followed
through its upload; `--strict` on a failed authentication). gcov: 97.1%
(`api.c` 94.3%, `protocol.c` 98.4%); `usb.c` 76-79%, varying from run to
run, perhaps with the unplug cases' timing. Not emulated: argument
checks, libusb open failures, sigrok-cli-unreachable config branches.
The driver's 1 s idle stop (`api.c:391`) is reached only through
`libsigrok-run`, because sigrok-cli's any-key stop fires at once on the
harness's `/dev/null`.

A capture ends on Ctrl-C, when the device leaves, on the first failed
transfer, or after 1 s without a transfer; the last three warn. A failed
authentication warns and the capture goes on as in the C driver, since
nothing shows whether a real board refuses to capture without one.

Firmware: `saleae-logicpro16-fx3.fw` and
`saleae-logicpro16-fpga.bitstream` come from
`sigrok-fwextract-saleae-logic16`, run on Logic 1.2.10 for Linux (per
the sigrok wiki).

libusb backend: one synchronous read at a time with a 100 ms read
timeout. One channel at 1 MHz takes 131 ms per transfer; unmeasured at
50 MHz. Outputs matched there at review time (105 cases).

Not verified on hardware. Device facts beyond the C driver's are the
model's assumptions. No public HDL or FX3 source is known.
`eisbaw/saleae_logic_reversed` documents the protocol from static
analysis of Saleae's v2.4.40 software (tested on a Logic Pro 8), newer
firmware than the 1.2.10 files: it gives `07 00` as the answer to 0x01
and 0x02, which no tool here checks. Which FPGA the board has is
unresolved.

Emulated (2026-11-03): model `saleae-logic-pro`,
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
unreadable strings. The cases cover every rate, 1 to 16 channels, selection by the board's
real address (read from sysfs, as the session's address goes stale after
an upload), the unflashed upload, every failing request of the open,
each authentication step, broken firmware files and a missing bitstream,
a claimed interface, unplugs during the upload, prepare and mid capture.
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
(`protocol.c:340-341`, `539-540`); `transact()` leaks every scrambled request
(`protocol.c:107`), and the stop leaves the 512 transfers submitted.

## Hantek 4032L (hantek-4032l)

`hantek-4032l`: the Hantek 4032L, 32 channels in groups A and B, no
firmware to upload. Ported 2026-11-04 from `api.c` and `protocol.c`; the
device sees the same transactions as under sigrok-cli. Emulator only,
not verified on hardware. The 135 cases of `tools/emu/hantek-4032l.sh`:
127 full, 5 `--oracle-bug`, 1 `--oracle-differs`, 2 library; gcov 91.4%.
The model's new options `hold=N`, `partial=BYTES` and `release=1` show a
stream that stops, a transfer that times out with data, and a slow
board. Differences: a refused configure or get ends the capture with an
error, where libsigrok polls for ever; 64 failed transfers in a row end
it, and from the get on so do 64 timeouts; a status command gets 10
reads and is then sent again; a sample count of 4 Gi and more is refused
(api.c:415 loses its upper half); the end word is only checked if it
arrived (protocol.c:194); trigger 1 is rebuilt at each start, where
api.c:521-523 clears only the enable flags. Kept unbounded, as in
libsigrok: the wait for status 2, including silent reads, so a trigger
that never fires waits until stopped; and the packets dropped before the
start word, so a board that never sends one also waits until stopped.
Kept from the C driver because nothing disproves them: a capture ratio
of 100 gives no trigger marker, a threshold of -8.2 V or below sends PWM
4096, the trigger marker comes at the pre-trigger position even without
a trigger. Replay fixtures: `tests/fixtures/hantek_4032l/`.

Emulated (2026-11-03): model `hantek-4032l`
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
end word). The cases cover every rate code, sizes up to 256k samples, thresholds
(including the -9 V that sends PWM 4096), external clock sources and
edges, every trigger kind and position (a capture ratio of 100
included), every get and refused value, FPGA version 0, a claimed
interface, the open's and the capture's STALLs, stale packets, an unplug
while polling and mid data, and the paths on which the C driver never
ends: a refused configure, a refused get, and a trigger that never
fires. `usb.c` gcov: 68.1%. Not emulated: submissions that fail (their
timing against an unplug is not deterministic), transfers after an abort
(only `dev_acquisition_stop()` causes them, and sigrok-cli never calls
it), a second device, and refusals sigrok-cli makes before the driver.

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

## ZEROPLUS Logic Cube (zeroplus-logic-cube)

`zeroplus-logic-cube`: ported (`src/drivers/zeroplus_logic_cube`), all
13 boards of the C's table. 134 cases: 111 full, 7 `--oracle-bug`, 15
`--oracle-differs`, 1 on libsigrok alone; gcov unchanged, 87.6%. Scan,
open and capture send what the C sends. `conn=` selects a board,
accepted but not listed, as libsigrok refuses it. A rate not listed is
refused. A failed status or address read is not used as a value, and the
wait is bounded (16 failed polls, about 1.6 s) and can be stopped. A
short or failed bulk read ends the capture with the samples of the reads
that completed. A second edge, or a channel asked high and low, is sent
as in C with a warning. Kept as in C until a recording says otherwise:
the address arithmetic (a limit of 1000 delivers 996 samples), limits 1
to 4 delivering nothing, the wrapped post-trigger count at limits 1 and
2 with ratio 100, the bogus words at full memory, no marker at
captureratio=100, and the marker at sample 0 of a capture with no
trigger at full memory (commit 4c1433d1: "we trigger immediately").
Nothing here has run on a real board.

Emulated (2026-11-03): model `zeroplus-logic-cube`
(`emu/src/models/zeroplus-logic-cube.rs`, default LAP-C(16032),
0c12:700e; `pid=` any of the driver's 13 boards). gcov
87.6% (550 of 628); src/usb.c 10.0%, the driver calling only
`sr_usb_open()` and `usb_get_port_path()`. Options: `pid=`,
`serial=TEXT`, `warm=1`, `fill=N`; faults
`stall=addr|wdata|rdata|bulkreq|bulk[:REG][@N]`, `short=N[:LEN]`
(default 1020 bytes; never whole packets), `stall_string=I`. Not
emulated: dead code (`analyzer_add_filter`, `analyzer_read_id`,
`analyzer_wait_button`, `analyzer_decompress`), what sigrok-cli refuses
first (a trigger on a disabled channel, unknown matches and keys),
`dev_acquisition_stop()` (never called), transport failures, and a claim
that fails where the configuration did not. The boards of 2 M samples
have no capture case, as one `.tx` golden would be 570 KB. No ZEROPLUS
document or recording is available: the requests and registers are the
driver's, the memory semantics come from commit 42ceb777's message, and
what the two trigger registers mean, the trigger address 0 of an
untriggered capture, the descriptors and the counter signal are the
model's choice.

What the model taught:
- The driver lists no scan option, so libsigrok refuses any `conn=` and
  a board cannot be selected.
- A rate not in the list reads past the array (api.c:148) until a crash
  or a match, depending on the build's rodata. In the oracle's build 3M
  crashes, while 46 is accepted with exit 0: `__analyzer_set_freq()`
  returns -1 unchecked (analyzer.c:326-327), so the board stays on its
  old rate while the output says 46 Hz.
- A failed or short bulk read delivers the previous read's bytes as
  samples, with exit 0 (in the first read, those of an uninitialised
  buffer).
- `analyzer_wait()` hangs on `LIBUSB_ERROR_PIPE` but carries on after an
  unplug, since -4 has the busy bit clear, ending with no samples and
  exit 0.
- A second edge silently replaces the first.
- A capture delivers fewer samples than its limit, mostly by the
  driver's own doing: protocol.c:113-120 makes `ramsize_trigger +
  triggerbar` the limit - 1 or - 2 (- 2 or - 3 with 1 or 2 samples
  before the trigger), which commit b9a5614d says matches the Windows
  app, so the hardware may make up for it. api.c:582-585 caps at that,
  `stop - now` leaves out the last address written (42ceb777's
  definition), and api.c:577 drops the two bogus samples. Only the rest
  of 1000 giving 996 depends on the model's reading.
- Limits 1 and 2 give an empty capture, limit 3 underflows at api.c:582,
  and limits 3 and 4 deliver nothing.
- A capture that exactly fills the memory delivers the bogus words, by
  the driver's own logic (api.c:563-564 clears `STATUS_READY`, so the
  discard at api.c:577 is skipped).
- A capture with no `-t` at full memory carries a trigger marker at
  sample 0 (api.c:556-557, 591-592, 639-640 have no `devc->trigger`
  check; commit 4c1433d1: "we trigger immediately").
- A limit of 0 is refused by libsigrok and sigrok-cli runs no capture
  without a limit, so `set_triggerbar()`'s `0 - 1` post-trigger count is
  never reached.

## Sysclk LWLA1034 and LWLA1016 (sysclk-lwla)

`sysclk-lwla`: the LWLA1034 (2961:6689, 34 channels, up to 125 MHz) and
LWLA1016 (2961:6688, 16 channels, up to 100 MHz). The driver needs
`sysclk-lwla1034-{off,int,extpos,extneg}.rbf` or
`sysclk-lwla1016-{100,100-ts}.rbf`.

Ported (2026-11-04) in `src/drivers/sysclk_lwla`, with the 141 cases of
`tools/emu/sysclk-lwla.sh`: 126 full, 4 `--oracle-differs`, 3
`--oracle-bug` (interrupts: sigrok-cli has no SIGINT handler), 8
`--libsigrok` for what sigrok-cli cannot show (a stop, a root hub, the
wait for a trigger). gcov 91.1%. Not on hardware.

The port keeps the C's integer divider and the rate asked for, with a
warning when they differ. `close()` always sends the shut-down
bitstream, as `sr_dev_close()` does for a libsigrok client
(`device.c:779`). Only the CLI's capture path skips it, through
`Device::close_after_capture()`, for wire parity with sigrok-cli, whose
`sr_exit()` closes the device still active (`std.c:427-428`).

The port does not copy:

- A `conn=` selecting nothing listing every board (`api.c:104`).
- A stop that writes only the first register of each sequence
  (`protocol.c:341`). This is a regression from `be64f90b` (2015-11-22):
  before it the five writes all went out with no cancel check, and
  `protocol.c:292-293` and `354` still submit all five on a cancel. The
  `lib stop` cases show it on the wire. On the LWLA1034 only the long
  register's address goes out, with no strobe, which would leave the
  capture running if the strobe is what latches a long register
  (`lwla1034.c:72` reads so; no device has confirmed it). On the
  LWLA1016 the first write is `REG_CAP_CTRL=0`, which does stop it, and
  only `DIV_COUNT=0` is skipped.
- Samples lost when a capture ends early: on a reply of the wrong size
  (`protocol.c:216-219`), on a failed transfer (`protocol.c:325-330`,
  then `299-309`) and on a stop (`protocol.c:237`).
- Trigger masks kept from an earlier session (`api.c:501-502`).

A capture waiting for a trigger that never fires polls until it is
stopped, in both (the model counts from the trigger; not on hardware).

Emulated (2026-11-03): model `sysclk-lwla`
(`emu/src/models/sysclk-lwla/`, a plain vendor device with no shell; the
FX2's firmware is not uploaded), options `model=1034|1016`, `period=N`,
`tick=MS`, `ext_clock=HZ`, `ext_trg=N`, `short_transfers=1`, `stale=N`.
Each open sends a bitstream as one bulk OUT on EP4 behind a 4-byte
big-endian length, checked byte for byte. Commands are 16-bit LE words
on EP2, replies come on EP6, and 32-bit values go high half first. The
LWLA1034 has short registers, 64-bit long registers behind a strobe, and
36-bit memory read as slices of 9 words with run-length coded samples;
the LWLA1016 has 32-bit memory, plain or run-length coded by its "-ts"
design. The model's clock moves on per status poll, not wall time. The
signal is an invented binary counter; a trigger is the AND of levels and
edges, with the TRG input's edge on the LWLA1034. Registers, values and
order are held to the driver's, and on the LWLA1016 a second memory read
is required after the first one following a memory reset
(`lwla1016.c:281`). Faults, off by default: any command or its reply
STALLed, a reply one word short, a wrong test word, a halted reply
endpoint, stale reply bytes. The cases cover both boards, rates, limits, every trigger kind, the
external clock and TRG input, the 64-byte quirk, every step of the open
with its three tries, the capture's requests, a capture with no words
(LWLA1016), broken bitstream files, a claimed interface, unplugs while
polling and in the quirk checks, and two `--libsigrok --hangs` cases (2
s) for a trigger that never fires. `usb.c` gcov: 74.2%. Not emulated: a
close during an acquisition, a capture with no words on the LWLA1034 (it
would take a fault), a failing submission of the start, and refusals
sigrok-cli makes before the driver.
Device facts beyond the ids, endpoints, commands,
registers and formats are assumed: no strings, the encoder's choices,
the status timing, the first test read returning 0, the capture count
reading the last address written, and the time counters running from the
trigger. The C says only "elapsed time" and "capture duration" and tests
the time limit before the trigger (`protocol.c:144-148`); if the real
counter runs from the arming, the two `--hangs` cases would end on the
derived time limit instead.

What the model taught: a `conn=` that matches nothing or is refused
selects every LWLA (`api.c:104`). Any rate in range is taken and the
divider is an integer division, so 3 MHz runs at 3.03 MHz and 60 MHz at
100 MHz, and on the LWLA1034 anything above 100 MHz runs at 125 MHz,
while the limits and the output header use the rate asked for
(`api.c:409-411`, `lwla1034.c:390-394`, `lwla1016.c:312-315`,
`protocol.c:503`). On the external clock neither limit is derived from
the other (`protocol.c:499-519`): `--samples` runs until the memory
fills, `--time` reads all of it. A time limit shorter than a sample
makes `samples_max` 0: one read, no samples, exit 0
(`protocol.c:510-511`, `212-214`). An error during a capture fails it
(exit 1, where the C exits 0), and samples not yet sent in a packet are lost
(`protocol.c:216-219`, `299-309`). A close from `sr_exit()` goes through
`std_dev_clear()` with the status still ACTIVE (`std.c:427-428`; only
`sr_dev_close()` sets INACTIVE first, `device.c:779`), so the FPGA stays
on after a capture and the shut-down bitstream goes out only after
`--show`, `--get` and `--set`. A claim held by another program fails at
`libusb_set_configuration()` with BUSY (`api.c:238`), before the
driver's own claim. Read only: `prepare_trigger_masks()` returns early
without a trigger (`api.c:501-502`), keeping an earlier session's masks.

## Sysclk SLA5032 (sysclk-sla5032)

`sysclk-sla5032`: ported, emulator-verified, not checked on hardware.
165 cases: 131
full, 8 `--oracle-bug`, 15 `--oracle-differs`, 11 library; gcov 91.5%
(api.c 85.7%, protocol.c 94.7%), usb.c 35.4%. The driver rounds depths
both ways: a limit of 512 gets 1024 samples, 5000 gets 4608. sigrok-cli
counts its limit only for decoders and "Device only sent", and writes
whole packets to its output (session.c:464), so it prints all 1024;
oxrok cuts to the limit on the host, with the same transactions, and a
shortfall is reported by both. Where oxrok differs: a `conn=` selecting
nothing lists nothing (api.c:116); records are summed in 64 bits and
expanded in blocks, so what they claim allocates nothing
(protocol.c:782, 795); a chunk ended by the 2 s timeout is kept where
the C drops it (protocol.c:749), which matters only if the firmware
sends no ZLP, unknown; a failed key step (protocol.c:646) and a failure
that ends a capture keep the C's good open but warn, the second also
failing the capture (exit 1, where the C exits 0), and so does a chunk that
is empty or neither full nor marked
(protocol.c:760-765, 851); a stop ends a wait for a reply. The bitstream
is the vendor's, as `sysclk-sla5032.bit`. A stop while waiting writes
nothing to the device, as in the C, so sampling stays on in register 14.
A stop that ends a wait for a reply leaves any late reply on the FX2, so
the next session's reads would be one behind until a reset. `limit
highest` compares transactions only: there is no sample digest at 64 Mi
samples. Replay fixtures record a board with its design already in
place.

The model `sysclk-sla5032` (2026-11-04) is an FX2LP with
a Spartan-6, bitstream upload and EEPROM keys, and RLE sample memory. Options: `period=`, `wait=`, `configured=1`, `fw_size=` and the faults
`stall=`, `stall_reply=`, `short_reply=`, `hold_reply=`, `data_stall=`,
`data_hold=`, `data_short=`, `no_marker=1`, `overcount=1`, `no_zlp=1`. Not
emulated: allocation and libusb list failures other than
protocol.c:797-801, a partly taken OUT, an open failing between scan and
open, what sigrok-cli or the driver's config_set checks first;
protocol.c:814-815 and 882-883 are dead code. Descriptors come from the
sigrok wiki's lsusb. No firmware or HDL source was found, which does not
mean none exists: the vendor ISO the wiki links
(`sla5032_2015_1_24.iso`, with the real `top.bit`) was not examined. So
the trigger's position in memory, register 5's initial value, "no keys,
no sampling", a depth independent of RLE (256 MiB is about 44.7 Mi
records) and a capture over 64 Mi samples kept whole are assumptions,
and the signal and register 0's value while triggered are invented.
Whether the firmware sends a ZLP is unknown: without one, a capture
whose records end on a packet boundary (about 1 in 256) would hit the 2
s timeout, and protocol.c:749 drops the chunk with exit 0. What the
driver shows: it ignores a failed key step (protocol.c:646), has no time
limit on a trigger that never fires, ends a capture with exit 0 on
errors. T goes on the first match in memory, whatever the
depth; the distance from the hardware's trigger is unknown. It sums
device records into an `int` unchecked (protocol.c:782): a total from
2^31 up to 2^32 goes negative and fails the allocation, and one that
wraps to a small number under-allocates and overruns the heap (read from
the C, no case).

## IKALOGIC Scanalogic-2 (ikalogic-scanalogic2)

`ikalogic-scanalogic2`: ported (2026-11-04), never on a board. All cases
full or pinned: 143 (123 full, 3 `--oracle-bug`, 9 `--oracle-differs`, 8
library, among them the stops and the no-limit crash, which only
libsigrok-run can reach). gcov as before. Control transfers on a claimed
interface as in C; the scan and the open detach usbhid, which stays
detached until the device is plugged in again, as libusb leaves it. Not
through hidraw, which needs usbhid bound. On Windows the default nusb
backend cannot claim a device bound to the HID driver, while libusb has
a HID backend and may work (both from their source, not tried; nor was
macOS). A status wait stays 1.5 s of wall time, and a stop cuts it
short. A two-stage trigger is refused (the C drops the error and
captures untriggered, api.c:342). A sample packet that lacks bytes the
capture uses ends it (the C uses old buffer contents,
protocol.c:301-307); a shorter report that holds them all is fine. A
status of fewer than two bytes counts as no valid status. The last match
of a multi-channel trigger is used, with a warning. A stale info report
is listed, with a warning. The serial is unsigned. A device that does
not get ready after a capture or a stop ends the capture after 100
status reads, where the C goes on for ever: a few seconds at low speed,
not a time the device is known to need, and a device still busy stays
un-idled until the next scan or open resets it. Replay: `emu-bits`,
`emu-ratio-1-rounding`.

The model is `ikalogic-scanalogic2`
(`emu/src/models/ikalogic-scanalogic2.rs`), a low speed HID device,
`20a0:4123`, whose every command and answer is a 128 byte feature report
on endpoint 0. No firmware source was found; descriptors come from the
sigrok wiki's lsusb of a real device, the protocol from its Protocol
section (by the driver's author) and the C driver. Options: `serial=`,
`fw=`, `wait-trigger=N|never`, `sampling=N`, `digest=1`; faults
`stall-get=`, `stall-set=`, `short-get=`, `stale-sample=`,
`stale-info=1`, `stale-reset=`, `busy-reset=`, `ignore-reset=N|never`, `short-len=N`. gcov 88.6% of the driver, src/usb.c 64.2%. Not emulated:
allocation failures, a kernel driver returning or refusing to detach,
the device vanishing between scan and open or between a callback and its
resubmit (races only), a stop during the 1.5 s status poll
(libsigrok-run cannot time one), and `sl2_set_after_trigger_delay`,
which nothing calls.

What the model taught: `CMD_SAMPLE` has no channel mask, so the model
sends all four channels (an inference from the driver), and data packets
begin `05 CHANNEL`, so a capture ends through a RESET only when a packet
is left over for the read after the driver's extra read; otherwise it
reads `05 63` and goes straight to IDLE. Without a limit a library
client's capture asks for 0 bytes, and if the device then says data
ready, as the model chooses to, the driver divides by zero
(protocol.c:317); a device that never said so would make it poll
forever; sigrok-cli cannot reach this, and the case pins only the `.tx`,
not the status. A short async read goes unnoticed and repeats old
samples; a two-stage trigger logs an error and captures untriggered;
only the last match of a multi-channel trigger is used; a serial over
2^31 prints negative; `samplerate=2.5M` parses as 2
(`sr_parse_sizestring`), so 2.5 MHz is written 2500000.

## Testo 435/635/735 (testo)

Testo 435/635/735 (`testo`): emulated, model `testo` (FT232R shell,
`128d:0003`, descriptors from the wiki's lsusb of a 435-4). The protocol
comes from the C and, independently, from Testo's own description
(`hhtcde.chm` in its `toolbox.exe`, pages hhx35_e, hhx35r_e, hhx35ee_x,
hhtcd7b_e, HHTCD4_e, exto_e), which agree on the request, 115200 8N1,
the layout and units 1, 3, 5, 24; the CRC's algorithm is the C's alone.
No capture is known, so the model's Size, Unused and Decimal exponent
bytes are invented zeros. Ported, 70 cases: 54 full, 13 --oracle-bug, 2
--oracle-differs, 1 library; gcov 91.3%. **This oracle has one line
restored: `sdi->driver = di;`, removed by b258c09f
(tools/oracle-patches/libsigrok-testo-sdi-driver.patch). Stock libsigrok
finds no Testo: libsigrok 0.5.1, 0.5.2 and unreleased lack the line,
0.5.0 has it. This rests on device.c:730, api.c and the commit, not on a
run.** The C's capture path has met no device in those versions, so its
behaviour is weak evidence. A device whose answer holds a unit other
than 1 (°C), 3 (%rF), 5 (m/s) or 24 (hPa) is not found, with only a
debug line, as in the C (protocol.c:134-136). Testo's table lists 173
units across all its instruments, so that a 435 set to °F or with a CO2
or dew point reading sends one is an inference. The scan opens every USB
device to read its strings, as the C does. A kernel's ftdi_sio is
detached once and stays detached until the device is replugged, as in
the C. Not verified on hardware.

## Kecheng KC-330B (kecheng-kc-330b)

Kecheng KC-330B (`kecheng-kc-330b`), sound level meter, 1041:8101,
vendor bulk. Emulated and ported: model `kecheng-kc-330b` (options
model=, inactive=, stored=, log-interval=, log-time=, log-freq=,
log-source=, id-length=, digest=, and faults stall-out=, silent=,
stall-in=, short=, garbage=, each CC[@N]). 129 cases, gcov 79.3%.
Descriptors and protocol are from the sigrok wiki and the C; no firmware
or recording, not verified on hardware. The model needs a quiet host:
the oracle's 5 to 40 ms timeouts miss under load, and oxrok's are 100 to
250 ms.

libsigrok cannot read the log. sigrok-cli dies on the meta packet's
tuple (api.c:367), and past that api.c:405 refuses every log start for
any client, on a request that went out, since 85ed4ab3 (2013). It also
cannot change the interval or run `--show`. A client that survives the
critical, PulseView for one, would reach api.c:405, and the C's stop
could then wipe an unread log there (session.c:831, api.c:429, against
api.c:433; inferred from source, not run).

oxrok reads the whole log with `-c data_source=Memory --continuous` or
`--samples N`, and never erases a log the user only asked to read. By
the C's comment (api.c:430, the only source) a configure clears the log,
so oxrok sends none on a refused start, a cut, a stop, or a failed read
or output. A live capture configures, in both tools, so it clears the
log. So does a read to the end with `-c sample_interval=...` or a
weighting set: that setting is applied after the read, and by the C's
comment the log goes with it.

Data source byte: the wiki's "0=memory, 1=real time" looks observed, and
the C writes its own enum (bc7be4a9). The memory path sends no configure
before 05 and 07, so the byte affects only a later configure. A live
configure sends 0 in both tools for parity with the C's working live
path; if the wiki is right, that byte says "memory", unproven.
`set_date_time` (protocol.c:232) is unused by the C (api.c:141), and
oxrok has no counterpart. Command 06 is not read.

## Lascar EL-USB data loggers (lascar-el-usb)

lascar-el-usb: Lascar EasyLog EL-USB data loggers (SiLabs F321,
10c4:0002), a stored log downloaded, not a stream. Needs conn=.
Temperature and humidity models 3, 12, 13, 16 (channels Temp, Hum) and
CO models 9, 11 (channel CO); the 15 others of the C table are
identified and refused, as in C. Emulated and ported 2026-11-04, none on
hardware: model lascar-el-usb, 152 cases (140 full, 4 --oracle-bug, 5
--oracle-differs, 3 library), gcov 93.9%. Not emulated: allocation
failures, libusb_open failing, a claim failing only after the scan, a
submit failing right after one that worked. The model's descriptors are
the wiki's lsusb recordings. Its block and memory sizes are inferred
from eltuxusb's read counts and tables: 0.3 for most, 128 bytes of
configuration for id 12 from ab91106, 65024 bytes of memory for id 14
from branch el-usb1-pro (f7aa7f7), and ids 15 to 21 are in none of them
and unverified. The driver takes the lengths from the device's headers.
What it found in libsigrok: api.c:312 reads 256 bytes into resp[4], and
sigrok-cli dies with enough queued; protocol.c:426, 430 lose half
degrees and percents; api.c:174 ignores the sample limit; protocol.c:558
never ends on a stopped or halted device; protocol.c:155 closes a handle
still in use.
Kept from C: CO's /1e6 at 0 digits. libsigrok.h:293-298 defines
SR_UNIT_CONCENTRATION as normalised 0 to 1, so the division is
deliberate, while src/analog.c:70 labels it "ppm". The analog output
shows 0 for all realistic readings (under 500,000 ppm, if s*high+low is
ppm, which is inferred); use `-O csv` and multiply by 1e6.
Also kept from C: `datalog=on` sets bit 0 of 0x21 and zeroes 0x18 to
0x1b (protocol.c:630-637), and leaves the sample count, the start time
and the downloaded bit as they were, unlike eltuxusb's start, which
writes 0x21 = 00000011, a count of 0 and a new start time. Download
first; the effect on stored data is unverified on hardware.

## GreatFET One (greatfet)

greatfet (Great Scott Gadgets GreatFET One, 1d50:60e6). Emulated by
model `greatfet` (options `in-limit=N`, `wide=1`, `level=HHLL`; faults
`stall=`, `stall-read=`, `serial=`, `version=`, `configure-len=`,
`in-stall=`, `stall_string=`, `short_string=`), built from
greatscottgadgets/greatfet da49f6b, libgreat 699d267 and the C driver.
131 cases, 2026-11-04: 104 full, 6 `--oracle-bug`, 11
`--oracle-differs`, 10 library. gcov 84.8% (api.c 86.6%, protocol.c
84.1%); not emulated: a failing open, a device without string 3,
timed-out transfers (timeout 0), allocation failures, unknown transfer
status. No real board has been tried.
What the model taught:
- After a failed transfer, the C's stop runs inside the transfer
  callback, where libusb answers BUSY, so it never reaches the board. On
  a limit or a stop by hand it runs outside the callback and goes out,
  as every full capture's `.tx` shows under both tools.
- A failed transfer loses what the feed queue holds, as the capture's
  end runs at the first transfer freed.
- With `probe_names=-...` of 8 names or fewer, packed bytes pass as
  samples.
- A limit overshoots to a whole byte of sample memory at 1, 2 and 4
  bits.
- A failed submission after a start leaves the board sampling.
- The C sends a rate of 2^32 or more truncated to the firmware's 32 bit
  field, so 2^32 runs at 204 MHz.
- By the firmware source, more than 8 channels on one bus, or a rate
  over 204 MHz, is refused without telling the host, and no samples
  come: `slices_in_use` stays 0, so nothing shifts (read from the
  source, not seen on a board).
- Rates below about 49.8 kHz are answered as asked but likely run faster
  (10 kHz at 50.8 kHz, 1 kHz at 61.9 kHz). This is inferred: the
  firmware writes the divider unmasked, and NXP's register description
  gives the counter 12 bits.
What oxrok does about these: it sends the stop after a failed transfer,
keeps the samples, unpacks, cuts to the sample, refuses rates over 32
bits, and warns on the two doubtful rate ranges.

## Korad KAxxxxP power supplies (korad-kaxxxxp)

Korad KAxxxxP family (`korad-kaxxxxp`): ported and emulated on the
CDC-ACM shell. Model `korad-kaxxxxp` (`idn=HEX|none`, `vmax=`, `imax=`,
`vset=`, `iset=`, `output=`, `ocp=`, `ovp=`, `beep=`, `load=`, `vout=`,
`iout=`, `status=`, `sixth=`, `answers=`, `setters=`, `uart=`, and the
shell's faults), 130 cases over all 14 table entries, 2026-11-05: 111
full, 15 `--oracle-differs`, 4 `--oracle-bug`. gcov 88.6% of the driver.
Not emulated: BEEP, SAV, RCL and TRACK (no C path sends them, so the
model flags them), firmware silent on `STATUS?` with the output off, the
OEM's binary protocol.

USB ids from reports, not a device: `0416:5011` "Nuvoton USB Virtual
COM", native USB, on a KA3005P and in a FreeBSD report of a Tenma
72-2635.

A scan, `--get`, `--show` or capture sends only queries, and a close
nothing. A `-c` with any bad option or limit sends no setting.

The status bits are disputed. `protocol.c` has bit 5 as OCP and bit 7 as
OVP; RND's document has bit 5 as Lock and bit 7 unused; the sigrok wiki
has 0x20 as either protection and trusts only 0x01, 0x20 and 0x40;
Nicoretti's ka3005p decodes neither; tspi.at has bit 4 as OCP. oxrok
keeps the C's decoding. So `ocp_enabled` and `ovp_enabled` readback is
not reliable: set them explicitly and check the panel. `enabled` and
CV/CC are the bits the sources agree on.

Some firmware answers `*IDN?` once per power cycle (tspi.at), so a
second run finds no device, in either tool; `force_detect=` gets round
it.

A silent device is not 0 V: a reading with no reply fails `--get` and
gives no sample, and a capture ends after 5 in a row. A silent `STATUS?`
still reads as all bits clear, so `--get enabled` prints `false`: do not
take that as "the output is off" from a device that may not be
answering.

A garbage reply keeps the last value, as in C. A reply arriving after
its timeout can be read once as the next request's answer: nothing in
the protocol tells them apart.

A capture that loses the device, or ends for silence with its limit not
met, fails (exit 1).

No serial replay fixture, as the harness records USB and HID only. Not
checked on hardware.

## RDTech TC66C (rdtech-tc)

`rdtech-tc` (TC66C): emulated and ported, CDC-ACM model `rdtech-tc`, 73
cases (63 full, 5 `--oracle-bug`, 5 `--oracle-differs`), gcov 93.8%.
Options: `name=`, `fw=`, `sn=`, `v= i= dp= dm= e0= e1=`,
`flip=REPLY:BYTE`, `cut=REPLY:LEN`, `extra=REPLY:N`, `again=REPLY`,
`late=REPLY:OFFERS`, `answers=N`, plus the shell's. `conn=` is required.
Replies are AES-256 ECB under a key shared by all meters. It is a
protocol fact from the wiki, TC66C.py and protocol.c. Not emulated:
failed writes in the scan and between polls, read errors. No USB ids are
known for the real meter. Bluetooth is not available. `-C D-` cannot be
selected, in either tool.

## RDTech UM series (rdtech-um)

RDTech UM24C, UM25C, UM34C (`rdtech-um`). Energy meters that report over
Bluetooth SPP; use `conn=/dev/rfcommN`, as `bt/rfcomm/MAC` is not
supported. Default 115200/8n1, which an RFCOMM tty ignores. Channels V,
I, D+, D-, T, E, one frame per poll at about 100 ms; use `--frames` or
`--time`. `-C D-` is refused, as in sigrok-cli (the dash reads as a
range). Emulator-verified on a CDC-ACM stand-in (the real meters have no
USB data side); not verified on hardware. The model's layout comes from
the sigrok wiki and the C driver. 90 cases: 72 full, 7 `--oracle-bug`, 7
`--oracle-differs`, 4 library; gcov 93.7%.

## RDTech DPS and RD power supplies (rdtech-dps, rdtech-rd)

rdtech-dps, rdtech-rd: model `rdtech-dps` on the CDC-ACM shell, a Modbus
RTU slave behind a UART; 226 cases, 137 full; gcov 89.2%; not on
hardware. libsigrok leaves a one-range model's multipliers at 0.0, so
all but the RD6012P read inf or -nan and write 0; oxrok uses the scale
in `api.c`'s table. That table is unproven in places: `api.c` and
esphome-dps have the DPS5015's and DPS5020's current in hundredths,
rdserialtool in thousandths on every model (tested on a DPS5005 only),
and the DPS5020 V1.2 PDF was not read. Power is register 13 in
hundredths, as in C, where the `riden` library has thousandths on the P
models and register 12 as a high word. The DPS8005's 5 A is `api.c`'s;
the sigrok wiki says 5.1 A. The modbus layer is `src/modbus`.

An open of a DPS locks its panel, and a close or a drop unlocks it. An
unplug, SIGTERM, SIGHUP, a SIGINT before the handler is set, or a crash
leaves it locked; the sigrok wiki says to hold the dial for 3 s to
recover the panel.

## Atten PPS3203T-3S (atten-pps3203)

Ported and emulated on the CDC-ACM shell. Model `atten-pps3xxx`
(`vset=`, `iset=`, `output=`, `ocp=`, `mode=`, `load=`, `replies=`,
`answers=`, `setters=`, `uart=`, and the shell's faults), 133 cases,
2026-11-05: 86 full, 20 `--oracle-differs`, 14 `--oracle-bug`, 13
library. gcov 90.5% of the driver. The real device is an FT232R; the
model is a CDC-ACM stand-in, as ftdi_sio would take the real ids. Not
checked on hardware.

A frame sets all setpoints, outputs, protection and mode at once, and
setpoints cannot be read back. So settings queue and go out as one frame
at the close or the next poll, and an output never comes on ahead of its
setpoints. What no option named goes out as 0 V, 0 A and off.

Polls are such frames too: **a capture zeroes and turns off whatever its
command line did not set**. This is as in C with stdin held open;
sigrok-cli under an EOF on stdin stops before its first poll and leaves
the supply alone, and oxrok does not read stdin, so it sends that poll.
A warning says what the frame changes from what the device reported
(outputs going off, protection, mode) and gives the setpoints it writes.

As in C, only a change from what this run set makes a frame. `-c
enabled=off` alone on an output that is on sends nothing and the output
stays on, and oxrok says so on stderr at the close. Likewise
`ocp_enabled=off` and `channel_config=Independent`. Sending any setpoint
with it sends a frame, which sets everything.

That the `aa aa` frame of a scan and a capture's start changes nothing
is an inference. `api.c:116` says only that the vendor software scans
so; biot's trace on EEVblog shows that software sending it ahead of its
`aa 20` frames, with a sum of `54` where C sends 0; pps-tools calls byte
1 an unused address and the sigrok wiki has it fixed at `20`, so neither
knows of an `aa` command. pps-tools polls with set frames as well, but
programs from a user's file that gives each channel's values.

`--get` and `--show` give the scan's answer for `voltage`, `current`,
`enabled`, `ocp_enabled` and `channel_config`. `voltage_target` and
`current_limit` give what this run set, 0 if nothing. `--samples`,
`--time` and `--frames` are refused, as in C: a capture is
`--continuous` and ends on Ctrl-C, or fails after 2 s without an answer
(exit 1). An answer without its header, or with readings over twice a
channel's range, gives no reading and leaves the state alone.

## Manson HCS-3xxx power supplies (manson-hcs-3xxx)

Ported and emulated on the CDC-ACM shell, not run on hardware. Model
`manson-hcs-3xxx` (`model=`, `gmod=`, `vmax=`, `imax=`, `vset=`,
`iset=`, `output=`, `load=`, `gmod-raw=`, `gmax-raw=`, `getd-raw=`,
`ack-raw=`, `answers=`, `gap=`, `setters=`). 171 cases: 134 full, 17
`--oracle-bug`, 20 `--oracle-differs`; gcov 91.9%. A real one is a
CP2102 at 9600/8n1 on /dev/ttyUSBn; the model is CDC-ACM behind a UART
at that line. `voltage_target` and `current_limit` read 0 until set, as
nothing asks the device, so a library client that gets `current_limit`
to put it back writes `CURR000`. `enabled` is read from the voltage, so
a short circuit reads as off. A `GMAX` that makes no number or is not
finite is a limit of 0; a finite one above the rating is not caught. A
reading must be eight digits and a mode, by the wiki's format and no
recording: firmware that sends another form gives warnings and no
samples. libsigrok waits for ever on a silent device, overruns its
buffer on over 50 bytes with no OK, and joins a fragment of a reply to
the next, each reporting made-up values.

## GW Instek GPD series (gwinstek-gpd)

GW Instek GPD-2303S and GPD-3303S (`gwinstek-gpd`): ported and emulated
on the CDC-ACM shell, model `gwinstek-gpd`, 141 cases: 107 full, 17
`--oracle-differs`, 15 `--oracle-bug`, 2 oxrok-only. gcov 95.6% of the
driver. Every case but the four named `stock` runs sigrok-cli with
locking disabled. From libserialport 0.1.2 (2024-09-10), stock
sigrok-cli can only `--scan` this driver: `api.c:255` returns with the
port open, and `dev_open`'s second open is refused by the exclusive open
(`serialport.c:589`, `:599`, commit ffbfc5c). It was seen with
libserialport-unreleased-104-g21b3dfe. On 0.1.1 the driver works, so the
shim compares a path real users reach there. The scenario builds a
preload that makes `flock` and `ioctl(TIOCEXCL)` no-ops, applied only to
this model's sigrok-cli through a wrapper; the C driver is unmodified.
The stock cases pin the failure with its stderr. oxrok closes the port
after the scan and opens it again. Csv is a known limit: one packet per
reading gives four rows a round, a current under the volts' label and
made-up zeros, and the driver warns. Never on hardware; the vendor
manual could not be read; line ends and reply shape are from PyGPD3303S.

## Fluke meters (fluke-dmm)

Fluke 87, 89, 187, 189, 199B, 287, 289 (`fluke-dmm`): ported and
emulated on the CDC-ACM shell. Model `fluke-dmm` (`model=`,
`uart=LINE|native`, `id=HEX|none`, `qm=HEX`, `data=HEX`, `answers=`, and
the shell's faults), 159 cases, 2026-11-05: 148 full, 5
`--oracle-differs`, 5 `--oracle-bug`, 1 library. gcov 98.6% of the
driver. Not emulated: a write failing at the first QM or a `QM N`
(`api.c:220-221`, `protocol.c:73`; an unplug there is a race), and the
real IR cable, which is a USB serial adapter, not CDC-ACM.

A scan tries 115200, 9600, 1200 Bd, three IDs each, so a 199B takes
about a second and a half unless given `serialcomm=1200/8n1`. Sources
disagree on the 8x and 18x text: a copy of Fluke's specification shows
`+47.66 KOhms` and `-121.43 VDC`, which the C driver reads as 47.66 Ω
and as no sample, while its list of units has what the driver expects.
No recording was found: check a reading against the display. A 199B
reads as fast as the meter answers, each answer asking for the next,
plus a QM each second.

A meter that gives no sample is waited for without a bound, unlike the
other serial ports: `protocol.c:127` has the retry there to recover from
temporary disconnects. So `--samples` on a silent meter, or on one that
answers and never has a sample (`BLANK`, a 199B reading exactly 0, which
is refused as in C with "Invalid float"), ends only at a time limit,
Ctrl-C or a port that is gone: use `--time`. A meter that stops taking
bytes costs each write 1 ms, as in C, and is logged. Only a port that is
gone ends a capture.

## Open Bench Logic Sniffer (ols)

`ols`: the Open Bench Logic Sniffer and SUMP compatibles on a serial
port. Ported (src/drivers/ols), sharing `p-ols`' commands, metadata
reader, channel groups and sample decoder.
`tools/emu/openbench-logic-sniffer.sh` runs 213 cases: 184 full, 10
`--oracle-bug`, 15 `--oracle-differs`, 4 library; gcov 94.7%. Model
`openbench-logic-sniffer` on the CDC-ACM shell (`04d8:fc92`), options
id=, meta=, name=, fpga=, channels=, rate=, protocol=, memory=, slow=,
sends=, uart=. Facts from jawi/ols, gillham/logic_analyzer, pysump and a
Pipistrello fork's Verilog (hand read); the board's own 3.07 source not
read, no bench.

Where oxrok differs from libsigrok:
- A board without a channel count captures (C's sample size of 0 hangs
  its output).
- A capture with no sample sends no packet (C crashes).
- No marker beyond or before the samples (C crashes, or sends bytes from
  before its buffer).
- Without a memory key the read count comes from the limit, up to
  0xffff, with a warning beyond it (C always sends 0xffff).
- Where C's delay is 0 and goes out as 0xffff, one unit goes out, with
  the marker from that delay, on the oldest word if the whole read
  follows the match. Any other delay is C's.
- Without RLE a limit that cuts the read doesn't move the marker. Under
  RLE the marker is C's.
- A drain gives up after 1 s.
- A board below 200 kHz starts at the highest rate the clock gives at or
  below its maximum (C reads 0 Hz, divider 0).

Decisions with no evidence: the 1 s drain bound, counting from the limit
without a memory key, one unit for C's delay of 0, and a slow board's
starting rate.

Unverified, from the fork's source only:
- Under RLE the delay counts stored words, so C's marker sample is not
  the match when runs or a limit are involved. oxrok keeps C's.
- The driver leaves unused stages cleared, which raises the level at the
  first word, so a multi-stage trigger waits only for its later stages.

## Maynuo M97 and M98 electronic loads (maynuo-m97)

maynuo-m97: model `maynuo-m97` on the CDC-ACM shell, a Modbus RTU slave
behind a UART; 210 cases, 172 full; gcov 95.8%; not on hardware. A scan
finds the two models of `api.c`'s table (M9710, id 28, and M9812, id
101), whose ids its comment calls guessed. Settings and state are in
channel group `1`: `-g 1`.

libsigrok's commit 524ec154 (2025, "Synchronize read and write
operations") deleted `maynuo_m97_capture_start`, which since ffb580cf
(2015) sent four registers from `U` at the start and after every read.
At the pinned commit a capture sends no request and reads the reply to
it (`api.c:376-383`, `protocol.c:227`), so it has no samples:
`--samples` never ends, and `--time` ends with nothing. oxrok sends the
deleted frame. No recording of it is known, and no run of the pinned
commit shows it. Differences from the older C: the first request goes 10
ms after the start, where C sent it at once; later ones go 10 ms after a
read ends, where C sent one right after a read; and the next request's
purge drops a late reply, where C's close waited for the last one.

C's frames have an oracle only through the model's `unasked=`, replies
no device sends. oxrok's request purges them, so those cases compare C's
unasked frames with the frames of oxrok's polls.

An open sets `PC1`, which locks the front panel, and a close or a drop
clears it. An unplug, SIGTERM, SIGHUP or a crash leaves it set.
sigrok-cli leaves it set on an error, so 49 failing cases are full with
PC1 off filtered from the `.tx`, and six `way out` cases pin the off.

The channels have indexes 0 and 1, where C gives both 0 (`api.c:150`,
`153`); no output shows it.

## Motech LPS-300 series (motech-lps-301)

Motech LPS-301 (`motech-lps-301`; libsigrok registers only that driver
from `motech-lps-30x`, with the model fixed, so any `LPS-` answer shows
as an LPS-301 with one channel): ported and emulated on the CDC-ACM
shell, model `motech-lps-30x` (`model=`, `version=`, `status=`,
`vout1=`, `iout1=`, `set=` answer lists; `uart=`, `eol=`, `blank=`,
`output=`, `v1=`, `i1=`, `answers=`, `late=`, `setters=`, and the
shell's faults), 149 cases, 2026-11-05: 116 full, 20 `--oracle-bug`, 13
`--oracle-differs`. gcov 79.3% of the driver. Not emulated: other
models' paths, which `scan_lps301` cannot reach (channels 2 and 3,
`TRACK`, `VDD`, model detection), library-only guards, a set after a
capture (`api.c:553`), and a failing write (`api.c:134`, `:167`,
`:200`). The request timeout (`protocol.c:201-203`) is dead code in C.
Commands, 2400 8n1 and status bits are from the series' manual, chapter
5; the answer form is from the C driver; line ends, reading widths,
`VOUT1` reading 0 with the output off, and how firmware 1.17 fails
`VERSION` are guesses. Never on hardware.

C waits for as long as a device stays silent: `calc_timeout_ms` gives 0
once 250 ms are up, and 0 is no timeout to `serial_read_blocking`, so a
wrong `conn=` or rate holds the scan; a late reply then fails after five
tries. In a capture `protocol.c:199` subtracts a real time (`:239`) from
a monotonic one, so a request never expires. oxrok fails an answer at
250 ms. In a capture it expires the request and sends nothing more until
the line is free: a line saying `OK`, or 400 ms of silence, at most 1 s.
No line says which request it answers, so an answer later than that
shows as a status that is no number, which makes no sample and waits
again, and no round makes a sample until a good status has come after
the expiry. A device printing readings as bare integers would not show
the shift; the manual's examples have decimals. With only a sample limit
it gives up after 2 s with no sample. A reading that is no number gives
no sample for that round, where C sends 0; a status that is no number
with no expiry before it keeps C's sample. A bare `OK` or a 48 byte line
does not hold the capture. Settings work after a capture, where C never
clears `acq_running`, and they and a second capture wait out an answer
the last capture left on its way. When a limit ends a capture in the
pass that read an `OK`, one more `VOUT1` goes out, as in C.

## Juntek JDS6600 signal generator (juntek-jds6600)

Juntek JDS6600 DDS signal generator (also Joy-IT), two channels, CH340
at 115200/8n1, `conn=` required. Emulator-verified on the CDC-ACM shell
(`emu/src/models/juntek-jds6600.rs`), no hardware. Groups CH1 and CH2
have enabled, pattern (17 built in, arb-01 to arb-60), output_frequency,
amplitude, offset, output_duty_cycle; device-wide are conn, enabled (set
only) and phase. It acquires nothing: sigrok-cli and oxrok refuse a
capture for want of a limit key, and a session (libsigrok's and
`Session::run`) refuses to start one because the channels start
disabled. Only a direct `Device::start` gets a header and an end.
libsigrok bugs the model found: every request after the first waits for
ever on a silent device or after an unplug (protocol.c:442-450); a
frequency reply of fewer than two words crashes the scan
(protocol.c:602-603, 608-609); NaN goes out as `nan` and an infinite
phase as `-nan`. As a precaution oxrok sends `-0` as `0`; nothing shows
the firmware rejects it. Kept from C but doubtful, with no recording to
settle either: the frequency's scale code divides (protocol.c:617-618),
where on1arf/jds6600_python multiplies by the same table and its notes
fit only that, so `100,3` reads as 1000 Hz here and 0.001 Hz there
(writes always send scale 0); and a negative phase goes out with its
sign, though protocol.c:811 says 0..360 and the Python sends -90 as
35100. The model's ranges are the C driver's limits, not the device's.
229 cases: 205 full, 10 --oracle-bug, 13 --oracle-differs, 1 library.
gcov 89.2%.

## ZKETECH EBD-USB (zketech-ebd-usb)

A 35 W electronic load, 0 to 4 A and 0 to 21 V, behind a PL2303. `conn=`
is required and `serialcomm=` defaults to 9600/8e1. Emulated by the
`zketech-ebd-usb` model on the CDC-ACM shell (test ids, as `pl2303`
would take the real ones), options `vbus=`, `dp=`, `dm=`, `limit=`,
`cutoff=`, `load=`, `sag=`, `device=`, `raw-checksum=`, `period=`,
`frames=`, `replies=`, `deaf=`, `setters=`, `uart=`. 109 cases: 88 full,
8 `--oracle-bug`, 7 `--oracle-differs`, 6 library. gcov 86.6%; not
reached: the setters past "later", as no client sets during a capture,
the end of `ebd_loadstart`, read errors, and argument checks. Never run
on hardware. Frames are from the sigrok wiki, the line from mdijkens'
program written against a real unit. Not known: the stream's rate, what
`06` does to a load, and the checksum when the XOR reaches `f0`. The
model sends it modulo `f0` and takes either from a host.

What the model taught: `send_cmd` returns `SR_ERR` for every command
that goes out (`protocol.c:58`), which leaves the end of `ebd_loadstart`
dead and makes a setter fail after sending. A checksum of `f8` ends a
frame early and one of `fa` restarts it, if a device sends the plain
XOR. If it takes it modulo `f0`, such frames fail C's check: 783 of the
20994 millivolt readings then fail all through the model's ripple, so
`--samples` never ends there, in C or in oxrok, until the capture is
stopped. After an unplug `serial_timeout` falls back to 1 s
(`serial.c:1159`), so C's 200 turns take some 200 s before it ends by
itself; oxrok ends at once.

The load is reached only in a capture. `current_limit` and
`uvc_threshold` read back what was set, and `--set` alone sends nothing.
A capture with no limit toggles off a load that is on. A stop toggles by
what the last frame said, so a capture ending on the frame that started
the load leaves it on, and one with nothing set, ending on the frame
that toggled it off, toggles it back on. Silence sends nothing. oxrok
keeps those wires and warns of each.

## Agilent and Keysight U12xx meters (agilent-dmm)

U1231A-U1233A, U1241A/B, U1242A/B, U1241C, U1242C, U1251A/B-U1253A/B,
U1271A-U1273AX, U1281A, U1282A (`agilent-dmm`): ported and emulated on
the CDC-ACM shell. Model `agilent-dmm` (`model=`, `uart=LINE|native`,
`id=HEX|none`, `conf=`, `conf2=`, `stat=`, `fetc=`, `fetc2=`, `fetc3=`,
`log=HEX|none`, `answers=`, and the shell's faults), 383 cases,
2026-11-05: 374 full, 5 `--oracle-bug`, 1 `--oracle-differs`, 3 library.
gcov 97.4% of the driver. Not emulated: the scan's write going short
(`api.c:135-136`; a unit test has it), and the real IR or Bluetooth
cable, which is a USB serial adapter or an rfcomm port, not CDC-ACM.
Facts come from the C driver, Keysight's U1281A/U1282A guide and the
sigrok wiki's page on the series; no recording of a meter was found.

The line is 9600/8n1 unless `serialcomm=` says otherwise (some U127x run
at 19200). A capture polls `CONF?` and `STAT?` once a second and `FETC?`
per enabled channel at `samplerate` (1 to 20 Hz, default 5). `--samples
N` counts channel samples, so a three display meter ends at 2 after P2.
On the U124xC and U128x, `data_source=Log-Hand`, `Log-Trig`, `Log-Auto`
or `Log-Export` reads the log by index until `*E`; only P1 carries it,
and other models refuse it as in C.

Turning the dial through OFF for over 10 s ends a sample-limited capture
with an error, as does any meter silent that long, or a log entry that
never reads. That is this port's choice: C goes on asking and picks up
when the meter answers. With `--time`, or no limit, oxrok waits too. A
meter that answers with no sample (square wave position, a display with
no quantity) is waited for. So is one answering `*E` to all it is asked:
each `*E` asks again at once, paced only by the meter and unbounded
under `--samples`, in both tools.

Kept from C without proof: a U128x reads peak hold as `4` in `STAT?`'s
fifth letter, where the guide says `1`; on a U128x with P2 enabled, the
square wave position still gets `FETC?`, because P2's `CONF?` answer
clears the mode; a meter that answers `CONF?` but not `FETC?` gets no
further `CONF?`; a `FETC?` answered late is taken for the channel asked
next, so values shift to the wrong channel (`P2: 1.00300 kHz`, `P1:
3.00200 kV`).

## GW Instek PSP series (gwinstek-psp)

GW Instek PSP-603, PSP-405 and PSP-2010 (`gwinstek-psp`): ported and
emulated on the CDC-ACM shell. Model `gwinstek-psp` (`model=`, `uart=`,
`urpsp=`, `v=`, `i=`, `u=`, `output=`, `otp=`, `load=`, `limit-mode=`,
`replies=`, `answers=`, `babble=`, `setters=`, and the shell's faults),
154 cases: 128 full, 18 `--oracle-differs`, 7 `--oracle-bug`, 1
`--no-oracle-only`. gcov 92.5% of the driver. Never on hardware.
Commands and frame are from Texio's manual and pypsp; URPSP1 rests only
on protocol.c's comment, and the frame at 10 A is a guess. The model
answers at once, so a real unit's timing is tested only with a
byte-paced fake in the unit tests.

## ICStation USB relay boards (icstation-usbrelay)

**One command per power cycle, and a second can switch relays.**
`--show`, `--get`, `--set` and a capture each put the board in command
mode at their open, even when they then fail (a bad group, a capture
with no limit). Only a power cycle leaves it; a `--scan` alone doesn't
enter it. The next run's identification request is then a relay byte. A
run meant to switch a relay off can switch relays on: all four on an
ICSE012A, R1 to R4, R6 and R8 on an ICSE014A. It then exits 1 after a
second, warning that relays may have switched. The request is both the
probe and the harm, so it can't be held back. libsigrok does the same,
without the warning.

`icstation-usbrelay` switches the ICSE012A (4 relays), ICSE013A (2) and
ICSE014A (8), through a USB serial port (a PL2303, `067b:2303`, per
sigrok's wiki). `conn=` is required; `serialcomm=` defaults to 9600/8n1.
Each relay is a group `R1` to `Rn` with `enabled`; without a group
`enabled` switches them all.

    oxrok -d icstation-usbrelay:conn=/dev/ttyUSB0 -g R2 --set -c enabled=on

The relays can't be read: a get reads what this process last set, from
all off. Two sources say all are on at power up; none measures it.

**ICSE013A polarity is unsettled.** oxrok keeps libsigrok's wire, 0 for
on. The programs written for that board (`brightlight`, `icserelay`,
`piswitcher`) and the `delendanet` post have 1 for on. If they are
right, `-g R1 --set -c enabled=on` sends `02`, turning R2 on and R1 off
while `--get` says R1 is on, and `enabled=off` for all sends `03`,
turning both on. On the ICSE012A and ICSE014A the sources mostly have 0
for on; `icsectl` has 1 on all three (it also swaps two ids) and
`LalunePierr0t/usbrelay` says both. No recording settles it.

**A reopen sends no second start**, unlike libsigrok, on outside
evidence and with no recording. A library client that closes and reopens
one device object keeps its relays: libsigrok's reopen, by that
evidence, sets R1 off and R2 to R4 on behind its cache on an ICSE012A.
The cost: the board has only USB power, so a replug between a close and
a reopen is a power cycle. oxrok then sends no start, a set returns Ok,
the cache says on, and nothing switched.

The model is `emu/src/models/icstation-usbrelay.rs`, a CDC-ACM stand-in
with test ids; cases `tools/emu/icstation-usbrelay.sh`, goldens
`tests/emu/icstation-usbrelay` (43 cases: 36 full, 2 `--oracle-bug`, 3
`--oracle-differs`, 1 `--libsigrok`, 1 `--no-oracle-only`). gcov of the
C driver over them is 83.0%; the rest is argument checks, defaults that
libsigrok refuses first, and write failures, which an unplug reaches
only by a race and a fake `Port` covers. No factory firmware source was
found. Not run on a real board.

## ITECH IT8500 series, BK Precision 8500 series (itech-it8500)

Electronic load on its TTL serial port. Emulator only (`itech-it8500`
model on CDC-ACM, 223 cases, 2026-11-05, gcov 93.6%), never on hardware.
`conn=` is required. Without `serialcomm=` a scan tries 9600, 38400,
19200 and 4800, and the line caps the sample rate at 10 to 60 Hz.
Options other than the limits and sample rate are in channel group `1`:
`-g 1` or `channel_group=1`. An open puts the load in remote mode and a
close in local mode, on error paths too. Sources: BK Precision's 8500
manual (version 032218) and ITECH's IT8500+ Programming Guide; `01`'s
units and that a load answers a broadcast `6a` rest on the C driver
alone. No packet says which command it answers, so an answer later than
its 100 ms can pass for the next command's: oxrok waits for a quiet line
after a timeout, which narrows this and does not remove it, and C has it
too. Not emulated: allocation failures, and paths `check_key()` refuses
first.

## Arachnid Labs Re:load Pro (arachnid-labs-re-load-pro)

Arachnid Labs Re:load Pro electronic load: ported and emulated on the
CDC-ACM shell, not on hardware. Model `arachnid-labs-re-load-pro`
(`version=TEXT|none`, `banner=`, `stale=`, `set=`, `uvlo=`, `on=`,
`volts=`, `amps=`, `replies=`, `answers=`, `lines=`, `count=`,
`period=`, `fields=2|4`, `gap=`, `setters=`), 131 cases (90 full, 15
`--oracle-bug`, 23 `--oracle-differs`, 1 library, 2 `--no-oracle-only`),
2026-11-06, gcov 91.8%. Not emulated: a port gone between the open and
the scan's first write, a get during a capture (no CLI or runner path
asks; fake-port tests only), branches check_key stops first, and oxrok's
own failed writes after an answer (no event falls there; unit tests
only).

Built from the firmware's source and history (arachnidlabs/reload-pro).
`conn=` is required, 115200/8n1 by default. C's float check on firmware
>= 1.10 refuses no real firmware: `version` came with v1.6, and 1.6 to
1.9 compare above 1.10. `uvlo` came with v1.9, `on` and `off` with
v1.10, and two more numbers on `read` after v1.11's release. The real
device is an FT230X, 0403:6015, which ftdi_sio binds by id, so the model
has stand-in ids. Keys other than the limits are on channel group `1`
(`-g 1`). A capture sends `monitor 200`, a sample is a `read` line, and
`monitor 0` goes at every close. `overtemp`, `undervolt`, `set` and
`uvlo` lines in a capture are META. What arrives ahead of a request or a
capture is read first, keeping only `overtemp` and `undervolt`. A reply
later than the 1 s wait stays in the input: the next request reads it
first if it has come, and otherwise, on a device lagging by more than
the wait, may take it for its own.

libsigrok: an `err` line is a success because its check reads the
pointer's bytes (protocol.c:59); silence or an error reads as 0
(protocol.c:180, 216); a cut line or the first line of any kind is taken
as the answer (protocol.c:51); a read with no answer or too few numbers
kills sigrok-cli (protocol.c:254, 336); a failed current_limit, uvc or
uvc_threshold query returns OK with no value, which kills it too, now
and then with a GLib error on stderr, so two of those cases run on oxrok
alone (api.c:230, 246, 253); a time limit, a stopped stream and an
unplug all hang (protocol.c:425); a line in two pieces is a wrong
reading (protocol.c:393). oxrok does the sane thing for each. A capture
with only a sample limit ends after 5 s with no reading.

## Rohde&Schwarz SME signal generators (rohde-schwarz-sme-0x)

SME02, SME03, SME03A, SME03E and SME06, SCPI over RS-232 (`conn=PORT`,
optional `serialcomm=`). `tcp-raw` and `usbtmc` resources go through
`src/scpi` untested, and C's GPIB is not ported. Emulator-verified on
the CDC-ACM shell (`emu/src/models/rohde-schwarz-sme-0x.rs`), 76 cases
(55 full, 20 `--oracle-differs`, 1 library), gcov 94.9%, no hardware.
Options: output_frequency and amplitude, get and set. A scan sends a
bare LF, which switches RS-232 to remote, and `*IDN?`, and closes; an
open and a close send nothing. With no `serialcomm=` the port keeps its
speed, frame and flow control, as in C, but on Windows and on a HID
cable, where it is 9600/8n1. The instrument has 8n1 fixed, 1200 to
115200 baud at its panel, 9600 at reset, and prefers RTS/CTS to
XON/XOFF. It acquires nothing: a capture is refused for want of a limit
key, and a session refuses to start with no channels.

Facts are from the maker's operating manual, edition 14 (1038.6002.02):
the IDN form, replies with no unit (`1E6`), the line, the reset state of
100 MHz and -30 dBm, the ranges. The model's stand-ins are LF as a
reply's end and the exponent form for a level. `FREQ?` and `POW?`
include the offsets, which neither tool sets, so a set range holds only
with offset 0, in C as well.

libsigrok bugs the model found: a failed read prints a double nobody
set, with exit 0 (api.c:170, 176-177, 180-181); a value out of range
does nothing and says nothing (protocol.c:71-72, 85-86, api.c:203, 207);
a NaN goes out as `FREQ nanHz`; an empty reply reads 0; the table has
the SME06 at 1.5 GHz (api.c:60-61), a copy of the SME02's, where the
manual has 6 GHz. By reading only: the stop takes the SCPI instance for
a serial port (api.c:237, std.c:366), and no session gets there.

Limits: after a query with no good answer the next waits, on a serial
port only, for 1 s of silence, up to 2 s, and purges. A reply that
begins later still can be taken for an answer, unless it is a frequency
for a `POW?`: `FREQ?` has no check, as with an offset any number is
real. `Scpi::get_data` reads for as long as bytes come with no newline,
so a device streaming that way hangs a query, as in C (shared code).

## APPA 55II (appa-55ii)

A two-probe logging thermometer (and the RS 55II clone) on a serial port
at 9600/8n1. It only sends. Emulated by `appa-55ii` on the CDC-ACM
shell, with options reading=, packet=, period=, burst=, mute=, records=,
after=, gap=, piece=, claim=, bad=, sync=, drop=: 72 cases, 55 full, 6
--oracle-bug, 10 --oracle-differs, 1 library, gcov 96.6%. Not emulated:
the `default:` arms for unknown keys and the read error, which C's check
of revents never reaches. Not on hardware. Protocol from the sigrok wiki
and the driver; no firmware source found.

`data_source=Memory` waits for a log a user starts on the device, ended
by its end frame, a limit or a stop. Differences from C:
- A time limit ends a capture on a silent device, and on time on a
  stream, where C waits for ever or overruns to the next frame.
- A hangup ends a capture with an error, where C waits for ever.
- A Live capture under a sample limit alone ends with an error after 10
  s with no byte, as the meter streams in menus and hold too. This is a
  choice: C waits. Memory mode, a time limit and no limit keep waiting.
- A wrong sum or bytes that start no frame, with records due, drop the
  records left with a warning, where C shifts every later reading (0.3
  degrees for 20.3).
- A piece gone without a trace still reads shifted under both tools, as
  nothing shows it; oxrok says at the end that the log ended N records
  short. A count above what comes reads the padding as a record, -0.1
  degrees, and gets the same warning.
- Log bytes with no metadata or no record due are dropped, where C spins
  for ever; oxrok warns once when there was no metadata.
- A live frame too short is no sample, with a warning, where C reads at
  fixed places and gets a value from the next frame's bytes (-32768).
- A metadata frame under 2 bytes keeps the count and warns, where C
  reads the sum and the next byte.
- A later capture on an open port drops what waited. A first one reads
  it, as C does: a client that starts long after its open gets up to
  about 54 s of old frames as new.

Kept from C: a size over 32 ends a capture, now with a warning; a limit
is checked after a whole read, so a limit of 2 in a log gets 3 records.
These log paths end normally with exit 0.

## MASTECH MS6514 (mastech-ms6514)

Dual-channel thermometer behind a CP2102, 9600/8n1, about 2 frames a
second once PC-Link is on. `conn=` is required. Options:
`data_source=Live|Memory`, `limit_samples`, `limit_time`. Not checked on
hardware.

**Stock libsigrok finds no device through a scan, in any build since the
driver's first commit.** `api.c:85` asks `serial_stream_detect` for a
packet of 36 bytes with a 36 byte buffer, and `serial.c:833` wants two
packets' room, so every scan ends at `:839`. The check is from 2012
(`2546b05c`, moved by `766456be`; `maxlen < packet_size * 2` at
`23669c3d^`), and the driver had the same 36 and 36 from its pull
request (`13b72fa9`) on. **So every "full" case here is against a
patched C:** the unmodified driver with that one argument changed to 18
in memory (`tools/emu/mastech-ms6514-scan.c`, through wrappers for this
model's tools only). `gcov.txt` doesn't say so, and its figures are that
patched C's. Two stock cases pin the failure.

Emulated: model `mastech-ms6514` on the CDC-ACM shell (options `frame=`,
`stored=`, `period=`, `first=`, `burst=`, `bunch=`, `still=`), 72 cases
(49 full against the patched C, 5 `--oracle-bug`, 11 `--oracle-differs`,
5 library, 2 oxrok alone), 2026-11-06. gcov 94.9% of the driver. Not
emulated: `protocol.c:223-224` (a read of 0 with exactly `G_IO_IN`),
`:240-241` (dead: the parse always leaves under 18 bytes), and the
default branches of `config_get`, `config_set` and `config_list`.

What it taught:
- A live frame that crosses the `a1` request ends a dump before it
  starts, with exit 0. At 2 Hz a frame is on the line at least 3.75% of
  the time. C says nothing, so it looks like an empty memory. oxrok
  keeps the behaviour and warns.
- `-C` in C selects by display, not channel. oxrok goes by the channel a
  reading names.
- A time limit in C is checked only when bytes arrive.
- A sample limit alone waits on a silent unit in both tools.
  `MS6514GUI.pde:465` says "BUG IN DEVICE!! doesn't send serial in this
  mode".
- **Known limit of `-O csv`:** a frame names two of three channels,
  often the same one twice, and csv takes readings by count. With all
  channels on, three frames gave four rows, with a T1 paired to the
  previous frame's T2 and made-up zeros. With T1 and T1 MAX, the label
  is `°C MAX,` and each reading is a row. oxrok warns, and `-O analog`
  is the output to use. `-C T1,T2` gives one row per frame only on
  frames showing T1 and T2 alone. C's csv prints an unset label there,
  which can crash, and it differs from run to run even with `perturb`.

Sources: sigrok's wiki pages, `tc2100` and `MS6514GUI`. No firmware
found. No source says what follows a dump or how fast it comes.

## PCE PCE-322A sound level meter (pce-322a)

Serial, 115200/8n1 (the wiki and libsigrok; the older PCE manual says
9600, which is unresolved): `-d pce-322a:conn=/dev/ttyUSB0`. Any port
that opens is taken for a meter, as in libsigrok. Emulator-verified (95
cases: 60 full, 12 full via library, 5 library-only, 9 `--oracle-bug`, 9
`--oracle-differs`; gcov 88.1%), not on hardware. `data_source=Live`
reads the stream; `Memory` reads the recordings by blocks and ends by
itself, each recording starting with a reading of -1. A full memory of
32,700 values takes about 44 s, and a small `--samples` still reads it
all and cuts the output. The meter reports its settings only in its
readings, so before a capture oxrok assumes A, F and 30-130. The
settings are toggles: on a meter set otherwise they land on the wrong
side, and the first live reading warns of what it contradicts. oxrok
counts a toggle as done, where C counts only from readings. Neither is
right on every meter: from the assumed start oxrok ends where asked and
C may not, and elsewhere the reverse. A memory capture takes the
weighting from each recording, which says nothing of the meter. `aa f2`
is taken to step upwards, by the wiki's order; the 2022 manual lists
50-100, 80-130, 30-80, automatic, which would put each step elsewhere if
it is the key's cycle. A status ending in 0x10, which the wiki allows
with no meaning, is read with a warning, by decision; C waits for ever.
oxrok ends a limited live capture or a memory read after 5 s of silence
(the display updates at 2 Hz; no source gives the stream's rate). A port
that is gone ends a capture; C's had not ended at 90 s, with no read
failing and `serial_timeout` never called. The block count C logs is one
block less than it reads, and which is right is unknown. History:
libsigrok ae87e02f added the memory read, and a4be2b32 fixed
`send_command` with a user's `Tested-By`.

## Devantech relay boards (devantech-eth008)

ETH002, ETH008, ETH484 and ETH8020, plus the ETH1610, WIFI002, WIFI008,
WIFI484 and WIFI8020 on the word of libsigrok's table alone
(`devantech-eth008`): ported and emulated on the CDC-ACM shell, model
`devantech-eth008`, 117 cases: 108 full, 5 `--oracle-bug`, 3
`--oracle-differs`, 1 library. gcov 88.2% of the driver. Never on
hardware. These are full under a preload, on a tty; TCP was compared by
hand only. The boards are reached over TCP, `conn=tcp-raw/HOST/17494`,
which the emulator can't carry, and no board with this command set on a
serial port is known, so the model is the same bytes on a tty with a
test id. The 108 compare requests, parsing, tables and messages on a
path no stock C reaches: `api.c:105` opens with no flags (since the
driver's first commit, 8712c783, 2023-09-24), which is read only, and
the first write fails with EBADF, on any libserialport. Every case but
the three named `stock` runs sigrok-cli under a preload that makes that
mode read/write, disables the exclusive open that would refuse
`dev_open`'s second open, and puts a 250 ms floor under the 20 ms read;
the C driver is unmodified. The silent and short cases show C failing
with `data is invalid`, where on TCP its `recv` (`tcp.c:362`) hangs: a
silent scan was still running at 40 s. oxrok's side of TCP is held by
`tests/tcp_raw_cli.rs` and unit tests on sockets. After a request with
no whole answer, oxrok connects again on TCP, as a new connection can't
receive an old answer (C's own remedy, `protocol.c:61`), and waits for
300 ms of quiet on a tty. A byte too many is dropped by a purge before
each request, where it shifts every later answer in C. `-g AIn --show`
dies on a GLib critical in C (`api.c:279` gives a float key a uint32).
Sources: the vendor's ETH008-B, ETH484-B and ETH8020-B documents and its
Python module. No firmware source found.

## UNI-T UT32x thermometers (uni-t-ut32x)

libsigrok's `uni-t-ut32x` is ported as `src/drivers/uni_t_ut32x`:
channels `T1`, `T2`, `T1-T2`, `data_source` `Live` or `Memory`, `conn=`
required since the cable cannot be told from any other UT-D04. A scan
opens the port and closes it, and a port that does not open is no
device. Differences from C:
- A channel that is off gets no packet, though its records still count.
  With the meter on T1 and `-C T2` there is no output and exit 0.
- A time limit waits 2 s past its end for the record C would stop on.
- A sample limit alone stops after 5 s with no record. This is a choice
  on an inference: no source says a unit never pauses, so a gap over 5 s
  (a menu, say) ends a `--samples` capture with a warning and exit 0,
  where C waits.
- A failed read ends the capture. A gone tty polls readable and reads 0,
  which C takes as no data (`protocol.c:238-239`), and it spins on a CPU
  for ever.
- A marker split over the receive buffer's end costs one record. In C
  (`protocol.c:231-234`) one stray byte loses every record for good.
- The first bad report on the cable ends the capture, a choice in the
  shared `HidPort`. By reading, C goes on after a lone one with nothing
  queued. When its error reaches the driver, C keeps `SR_ERR` in a
  `size_t` (`protocol.c:236-241`) and sigrok-cli can segfault; a `00`
  header per record does so in some runs. By reading, an `ff` header
  makes `ch9325_read_bytes` over-read its 8 byte buffer.
- 19 bytes with CR LF at the end and another inside are always cut at
  the inner marker. C does that when reading them byte by byte, and
  takes them as a record when it reads them in one go.
- `conn=1a86.e008`, a form C has retired, is an oxrok extension. It
  means `hid/ch9325`, so the chip and not those ids, and of two cables
  the first the search finds.

Emulated by models `uni-t-ut32x` (CDC-ACM; options live, stored, slots,
period, burst, uart, and the shell's) and `uni-t-ut32x-hid` (CH9325
descriptors from the wiki's UT325 lsusb, bcdDevice 12.00; also baud,
bits, chunk, idle, stall-report, stall-out, bad-report). 84 cases (67
full, 13 --oracle-bug, 2 --oracle-differs, 2 library). gcov 94.5% on the
serial model and 87.7% on the cable. Unreached: option defaults
sigrok-cli cannot ask, spew, a loop line a 19 byte buffer cannot reach,
null guards, and the chip's bad report header, whose crashing runs write
no counts. Not on hardware. The record layout is the wiki's. How the
memory dump is paced and ends, the line rate, and the update rate
(sources say 6 Hz, two or three a second, some 60 a second) are not
known. C leaves 6 bytes of its stack in each output report on the cable
(`serial_hid_ch9325.c:130,149-150`); oxrok sends zeros.

## UNI-T UT181A (uni-t-ut181a)

UNI-T UT181A multimeter (`uni-t-ut181a`): ported and emulated on the
CDC-ACM shell, model `uni-t-ut181a` (`readings=`, `saved=`, `rec=`,
`recs=`, `first=`, `frames=`, `period=`, `per=`, `over=`, `have=`,
`fail=`, `pad=`, `off-reading=`, `uart=`, `setters=`, and the shell's
faults), 195 cases, 2026-11-06: 168 full, 10 `--oracle-bug`, 8
`--oracle-differs`, 9 library. gcov 90.1% of the driver. Behind its real
cable as model `uni-t-ut181a-cp2110` (the `cp2110` model's chip with
this meter on its UART), 6 full cases; gcov of `serial_hid_cp2110.c`
48.9%. BLE is not emulated. `conn=` is required, as nothing identifies
the meter. Frames, packets, modes and commands are from `antage/ut181a`
(commit 21c5b07, Protocol.md and its Rust code) and the C driver; OK
ahead of the first reading, the chunk size and ER for what is not there
are guesses, so "ER ends a download" rests on the model's guess too. The
checksum is C's, the sum of the bytes of the length and the payload:
`antage` sums 2 and the bytes of the payload's length, which differs
when that length's low byte is 0xfe or 0xff (a chunk of 28 or 85
samples), and no recording settles it. Never on hardware.

The oracle's `-O0` build fails on anything behind a unit: `consume_str`
writes past its 8 byte buffer (`protocol.c:1720`) onto the low byte of
the count of bytes left, so aux values are lost and a meter with any
recording is no device. Two `-O2` compiles have other bytes there, by
their debug info and not by a run. The model's `pad=` gets C's parser
past it; pad-less cases pin the failure, with oxrok held to frames as a
meter sends them. A field cut behind the first unit cannot be shown in
C: it fails at that first field whatever is cut. `cp2110_write_bytes`
sends uninitialised stack behind a report's bytes
(`serial_hid_cp2110.c:246,263-265`).

oxrok waits 2 s for the first reading where C gives up at 100 ms, and
500 ms for a reply. A range or mode set waits for its reply code, and
`ER` or nothing fails it; C crashes at the mode (`api.c:412`) and has no
wait at the range. `range=auto` sends a range of 0, where C sends a mode
of 0 (`protocol.c:684`). A mode set reads back as auto until the next
reading. An `ER` to a count or an info ends that wait at once. Each
chunk asks for the recording selected, where C asks for the first after
the first chunk (`protocol.c:2682`). A download ends on `ER`, an empty
chunk or 5 s with nothing read, where C waits or asks for ever; such a
capture exits 0 with its message on stderr, like every failed capture
for now. A time limit is checked on each pass, not only at a packet, and
a failed read ends a capture. A live capture with only a sample or frame
limit ends after 10 s of silence. A capture takes only its own kind of
packet, where C feeds any and a stray one can end it, and a record info
of another name refuses the start, as its unit scales every sample; so
does a recording renamed since the list. After a wait that expired or a
capture that may have left an answer, nothing goes out until that answer
has come or 500 ms have passed, and the line has been quiet for 100 ms,
at most 3 s more; a reply code carries no id, so an answer later than
that can still answer the next request. Kept as in C: only a frame's
first value counts as a sample and a limit drops the rest of that frame,
a failed frame stays open until the next closes it, and in a live
capture what comes behind garbage waits for the next bytes.

## Atorch meters and loads (atorch)

Atorch DC meters and loads (`DC Meter`, the DL24 family) and USB meters
(`USB Meter`, the J7-C, UD18 and others), which report unasked about
once a second. `conn=` names a tty: on Bluetooth SPP an RFCOMM tty,
`/dev/rfcommN`, bound by the user; libsigrok's table has DL24M and UC96
`_SPP` units on channel 2, which `rfcomm bind` must be told. `bt/...`
names are refused, so `/channel=N` is lost, and `*_BLE` units (`ac6328`,
and C's `ble122`, `nrf51`, `cc254x`, `dialog`, `notify` types) have no
tty and no path in oxrok. Default 9600/8n1, which an RFCOMM tty ignores.
Channels V, I, C, E, T, plus D-, D+ on a USB meter; one frame per
report; use `--frames` or `--time`. An AC meter (type 01) is refused, as
in C. Nothing is ever written to a unit. A scan waits up to 10 s.
Emulator-verified on a CDC-ACM stand-in, model `atorch` (`meter=`,
`packet=`, `noise=`, `period=`, `scan=`, `burst=`, and the shell's
faults), whose default reports are recordings quoted by
esphome-atorch-dl24; not verified on hardware. No firmware source was
found. NiceLabs/atorch-console's `docs/UD18-SPP.pcap`, a capture of a
real UD18 over SPP, has 91 reports with good sums, 0.987 to 1.039 s
apart, 1 C count = 1 mAh and 1 E count = 10 mWh. That program's USB mAh
(×10) and DC energy differ from C's tables, and C's are kept. Its USB
temperature is signed where C's is not, so a reading below 0 would show
about 65535 °C, as in C. 70 cases, 2026-11-06: 58 full, 5
`--oracle-differs`, 4 `--oracle-bug`, 3 library; gcov 95.8%. The sum at
a message's end is not checked, as in C: esphome-atorch-dl24's README
says the J7-C, DL24M and DT24PW fail it, and the DL24M log in its issue
59 is cited for last bytes that look random.

## Link Instruments MSO-19 (link-mso19)

Emulated: model `link-mso19` on CDC-ACM, 3195:f190, options `vid= pid=
manufacturer= product= serial= pre= wait= post= status-at= mute-at=
late-at= mark-at= dump= extra= pause= raw=`. 162 cases on 2026-11-06:
134 full, 7 `--oracle-bug`, 19 `--oracle-differs`, 2 library. gcov 92.0%
of the driver; not emulated: write and open failures, null and group
checks the CLI makes first, the unused forced capture and the dead
default of the first serial field. Not checked on a unit.

The calibration comes from the USB serial number, so a device with none
is refused, and `conn=` must be a USB port with those ids; no scan opens
anything. A capture is always one dump of 1024 samples. C and oxrok read
19 digits of the serial number; mso19fcgi uses 11 more at 200 MSa/s, so
200 MHz readings use the other rates' calibration. C's dividers are kept
where mso19fcgi's differ (0x0362 is 1 MHz here, 2 MHz there, whose own
comments agree with C). mso19fcgi takes 3070 bytes as a dump, so a unit
sending a little short is possible: oxrok then loses that capture and
says how many bytes came.

What the model taught: C hangs when the status read at the start already
says READY; C has truncated the trigger level to whole volts over the
probe factor since 932a3452; a probe factor of 65536 kills C with SIGFPE
and 65537 wraps to 1; a silent device holds C for ever (oxrok ends after
5 s); C's csv shifts logic columns by one beside an analog channel
(`csv.c:500`). The dump's two high bits and the strings are guesses: on
C's XON/XOFF line a dump holding those bytes comes up short.

macOS and Windows are from reading serialport 4.10.1, unrun: macOS lists
callout ports alone here, and the serial number may not reach us there;
on Windows a unit with no serial gives an instance id, which is refused,
and the strings are friendly names.

## Norma DM9x0 and Siemens B102x (norma-dmm, siemens-b102x)

Norma DM910 to DM950 and Siemens B1024 to B1028, with the RS-232
interface board, at 4800/8n1 with RTS/CTS. `-d
norma-dmm:conn=/dev/ttyUSB0` or `-d siemens-b102x:conn=...`; the two
differ only in vendor and model names. Emulated: 95 cases on 2026-11-06:
81 full, 12 `--oracle-differs`, 2 `--oracle-bug`. gcov 93.5% of the
driver. Not checked on a unit, so the digits' meanings, the line end and
the reply latency are unverified.

What the model taught:
- C's `xgittoint` gives a to f as 0 to 5 (`protocol.c:67-74`), so modes
  0xa and 0xb never run, though the wiki's table has a diode as
  `DC,DIODE` and A≃ as `AC,DC,RMS`, which only they give; oxrok gives
  each digit its value. An unknown value type or mode logs C's error and
  still sends, as gain or unitless. That choice has no C run behind it,
  since C's digit conversion stops at 9.
- C shifts a flag pair's first digit by 8 from its first patch on
  (`protocol.c:282`, `:300`, `:316`), so continuity is always 0 and
  every reading AUTO; oxrok shifts by 4. No recording settles that a
  pair is a byte. If it is not, the damage is flags, a warning and
  continuity's 0 or 1, never a numeric value.
- C keeps part of a line past its request's expiry and cuts what follows
  at 20 digits (`protocol.c:97`), which made 24.03 with no unit of 24.91
  V DC. oxrok drops the part and refuses a 21st hex digit; another tail
  is cut as in C.
- C stops reading for good at 255 bytes with no LF (`protocol.c:393`);
  oxrok takes one bad line.
- C asks at every line and every 2 s, so at a steady 2.5 s latency 24
  replies are due after a minute. oxrok counts requests: a line asks
  with none out, and an expiry asks with a wait of 2, 4, then 8 s. A
  silent meter is asked less often than in C, and a lost reply costs 4
  s. A meter slower than 8 s still gains a request per wait.
- Replies left over from a capture are no sample of the next one on an
  open device.
- A silent `--samples` warns once after 30 s and goes on, as the
  interface can take 10 s to charge. Replies ended by CR alone warn
  once.

Sources: the sigrok wiki's page has the only real lines; it writes
replies ending in CR and C reads CR LF, which stays open (`eol=` has
each). The Wayback Machine has set.nl's product page, with "NM 950 with
interface, A 1834 06512", the `1834 065` of `IDN?`, and no protocol. No
manual, firmware or recording was found.

## LCR meters (serial-lcr)

`deree-de5000`, `mastech-ms5308`, `peaktech-2170`, `uni-t-ut612`
(Cyrustek ES51919, 9600/8n1/rts=1/dtr=1, streams 17 byte packets) and
`peaktech-2165`, `voltcraft-4080` (1200/8n1, answers `N` with 39
characters). `conn=` is required. A scan listens 3 s for a packet, then
1.5 s for the settings. Output frequency and circuit model cannot be
set: they are read, and sent as Meta before a frame when they change. A
capture takes `--frames` or `--time`; `--samples` is refused, as in C.
An ES51919 takes no commands and streams all the time, so at a frame
limit alone 10 s of no bytes ends the capture with an error. A VC4080
that does not answer may be in setup mode: oxrok warns once and goes on
asking.

Emulated on the CDC-ACM shell: model `serial-lcr`, 124 cases on
2026-11-06 (118 full, 5 `--oracle-bug`, 1 library), gcov 87.4% of the
driver. Not checked on a unit. Facts come from the C's reverse
engineered layouts, sigrok's wiki and Conrad's protocol description
(`web.archive.org`, the C's link is dead), whose range and Rs tables all
match. Conrad says 7E1: a 7E1 meter and an 8n1 host share a 10-bit
frame, so bit 7 as parity is physically right. The C's comment says its
FT232R gave nothing at 7e1, and the model repeats that as an artefact.
`4x1md/de5000_lcr_py` reads a DE-5000 with DTR=1, RTS=0 on a UT61E's IR
cable. Not modelled: the meter's other commands (`A`, `B`, `S`,
`[BXXXXXX]`).

Unverified: a 7e1 host on a real cable, a real ES51919's packet rate and
whether one stays silent only when off or unplugged (from the wiki's
"continually outputs"), the VC4080 warning's wording against a real
setup mode, macOS and Windows.

## Oracle

The oracle is libsigrok 0bc24877 with one patch,
`tools/oracle-patches/libsigrok-testo-sdi-driver.patch`. It restores the
`sdi->driver = di;` that `b258c09f` (2017) removed from testo's scan:
without it, `testo_probe_channels()` calls `sr_dev_open()` on an instance
with no driver, which fails (`device.c:730`), so stock libsigrok finds no
Testo and no capture is reachable. Apply it when building the oracle.

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
