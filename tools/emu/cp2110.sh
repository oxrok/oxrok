# The CP2110 HID-to-UART cable with a UNI-T UT61E behind it
# (emu/src/models/cp2110.rs), four packets per purge of the RX FIFO.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
#
# Every case names the cable by its node, conn=hid/cp2110/raw=PATH:
# libsigrok's search for a cable (conn=hid, hid/cp2110, hid/cp2110/sn=)
# hands hidapi-hidraw its own port name ("hid/cp2110/raw=/dev/hidrawN")
# as the path to open and fails, so sigrok-cli only finds a CP2110 by
# path (see the model's notes).
EMU_MODEL=cp2110
EMU_OPTIONS="-o packets=4"
EMU_DRIVER=uni-t-ut61e-ser
# Coverage of the serial-dmm driver. serial_hid_cp2110.c is outside
# src/hardware, which tools/emu-gcov.sh does not cover: delete its
# src/.libs/serial_hid_cp2110.gcda before the session and run
# 'gcov -t -o src/.libs src/serial_hid_cp2110.c' in libsigrok after it.
EMU_GCOV=serial-dmm
# oxrok has the driver and the cable: its cases must run full.
EMU_FULL=uni-t-ut61e-ser

emu_cases() {
    D=uni-t-ut61e-ser
    C=conn=hid/cp2110/raw=$EMU_HIDRAW
    emu_case "cp2110 scan" --keep-header -- -d $D:$C --scan
    emu_case "cp2110 show" --keep-header -- -d $D:$C --show
    emu_case "cp2110 one sample" --keep-header -- -d $D:$C --samples 1
    emu_case "cp2110 two samples" --keep-header -- -d $D:$C --samples 2
    emu_case "cp2110 four samples" --keep-header -- -d $D:$C --samples 4
    # A burst of four packets is one read: all four are out before
    # serial-dmm checks a limit, so one, two and four samples print the
    # same four readings. A time limit ends the run with the same.
    emu_case "cp2110 time limit" --keep-header -- -d $D:$C --time 1000
    # The comment lines name the tool and the time.
    emu_case "cp2110 csv" --filter "/^;/d" -- -d $D:$C --samples 4 -O csv
    emu_case "cp2110 analog" --keep-header -- -d $D:$C --samples 3 -O analog
    emu_case "cp2110 raw without chip" --keep-header -- -d $D:conn=hid/raw=$EMU_HIDRAW --samples 1
    # The meter's own line settings, spelled out.
    emu_case "cp2110 serialcomm" --keep-header -- -d $D:$C:serialcomm=19200/7o1 --samples 2
    # The other line settings the CP2110 takes: even parity (the chip
    # flags parity errors, the 7 data bits still arrive), two stop bits.
    # No flow= case: libsigrok's cp2110_set_params reads serialcomm's
    # flow (1 RTS/CTS, 2 XON/XOFF) as libserialport's enum (1 XON/XOFF,
    # 2 RTS/CTS), so it refuses flow=1 and turns RTS/CTS on for flow=2;
    # oxrok does what serialcomm says.
    emu_case "cp2110 even parity" --keep-header -- -d $D:$C:serialcomm=19200/7e1 --samples 1
    emu_case "cp2110 two stop bits" --keep-header -- -d $D:$C:serialcomm=19200/7o2 --samples 1
    # What the chip cannot do, refused before any report is sent: the
    # scan finds nothing.
    emu_case "cp2110 baud rate too low" --keep-header --no-output -- -d $D:$C:serialcomm=200/7o1 --scan
    emu_case "cp2110 baud rate too high" --keep-header --no-output -- -d $D:$C:serialcomm=2000000/7o1 --scan
    # 8 data bits: the meter's parity bit lands in bit 7 and the packets
    # fail the check, so the scan finds no meter.
    emu_case "cp2110 eight data bits" --keep-header --no-output -- -d $D:$C:serialcomm=19200/8n1 --scan
    # Another baud rate: the cable receives nothing.
    emu_case "cp2110 wrong baud rate" --keep-header --no-output -- -d $D:$C:serialcomm=9600/7o1 --scan
}
