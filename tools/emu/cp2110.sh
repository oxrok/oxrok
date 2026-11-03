# The CP2110 HID-to-UART cable with a UNI-T UT61E behind it
# (emu/src/models/cp2110.rs). Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model". Once the host enables the UART
# the meter sends a packet every 25 polls (the default period), each in
# two parts 3 polls apart: eight readings, then ten packets of noise
# ("?" bytes), and again.
#
# Every case that sigrok-cli runs names the node: the oracle's libsigrok
# (0bc24877) cannot open a HID cable it has to search for (see
# tools/emu/bu86x.sh).
EMU_MODEL=cp2110
EMU_OPTIONS=""
EMU_DRIVER=uni-t-ut61e-ser
EMU_GCOV_SHARED="src/serial.c src/serial_hid.c src/serial_hid_cp2110.c"
# Coverage of the serial-dmm driver, then of the shared serial sources
# above (serial_hid_cp2110.c among them) in gcov.txt's shared section.
EMU_GCOV=serial-dmm
# oxrok has the driver and the cable: its cases must run full.
EMU_FULL=uni-t-ut61e-ser

emu_cases() {
    SEARCH_BUG="libsigrok 0bc24877 converts a searched path twice (serial_hid.c ser_hid_chip_search)"
    D=uni-t-ut61e-ser
    C=conn=hid/cp2110/raw=$EMU_HIDRAW
    emu_case "cp2110 scan" --keep-header -- -d $D:$C --scan
    emu_case "cp2110 show" --keep-header -- -d $D:$C --show
    emu_case "cp2110 one sample" -- -d $D:$C --samples 1
    emu_case "cp2110 two samples" -- -d $D:$C --samples 2
    emu_case "cp2110 four samples" -- -d $D:$C --samples 4
    # Past the noise: the driver searches 140 bytes for the next packet.
    emu_case "cp2110 nine samples" -- -d $D:$C --samples 9
    emu_case "cp2110 csv" --record -- -d $D:$C --samples 3 -O csv:header=false
    emu_case "cp2110 analog" -- -d $D:$C --samples 7 -O analog
    # The chip from the node's USB id.
    emu_case "cp2110 raw without chip" -- -d $D:conn=hid/raw=$EMU_HIDRAW --samples 1
    # The meter's own line settings, spelled out.
    emu_case "cp2110 serialcomm" -- -d $D:$C:serialcomm=19200/7o1 --samples 2
    # The other line settings the CP2110 takes: even parity (the chip
    # flags parity errors, the 7 data bits still arrive), two stop bits.
    # No flow= case: libsigrok's cp2110_set_params reads serialcomm's
    # flow (1 RTS/CTS, 2 XON/XOFF) as libserialport's enum (1 XON/XOFF,
    # 2 RTS/CTS), so it refuses flow=1 and turns RTS/CTS on for flow=2;
    # oxrok does what serialcomm says.
    emu_case "cp2110 even parity" -- -d $D:$C:serialcomm=19200/7e1 --samples 1
    emu_case "cp2110 two stop bits" -- -d $D:$C:serialcomm=19200/7o2 --samples 1
    # No parity: the meter's parity bit is taken for the stop bit, the
    # 7 data bits arrive.
    emu_case "cp2110 no parity" -- -d $D:$C:serialcomm=19200/7n1 --samples 2
    # 8 data bits (the meter's parity bit lands in bit 7) and another
    # baud rate (garbled bytes): the scan searches them for 3 s of wall
    # time and finds nothing. Two packets per open (burst=2), so that
    # the number the stream sends in those 3 s does not depend on time.
    cp2110_options burst=2
    emu_case "cp2110 8n1" --keep-header --no-output -- -d $D:$C:serialcomm=19200/8n1 --scan
    emu_case "cp2110 9600 baud" --keep-header --no-output -- -d $D:$C:serialcomm=9600/7o1 --scan
    # A meter that never sends.
    cp2110_options burst=0
    emu_case "cp2110 never sends" --keep-header --no-output -- -d $D:$C --scan
    cp2110_options
    # What the chip cannot do, refused before any report is sent: the
    # scan finds nothing.
    emu_case "cp2110 baud rate too low" --keep-header --no-output -- -d $D:$C:serialcomm=200/7o1 --scan
    emu_case "cp2110 baud rate too high" --keep-header --no-output -- -d $D:$C:serialcomm=2000000/7o1 --scan
    # The forms users type, which sigrok-cli cannot open (see above):
    # oxrok alone, against the goldens.
    emu_case "cp2110 search chip" --oracle-bug "$SEARCH_BUG" -- -d $D:conn=hid/cp2110 --samples 1
    emu_case "cp2110 search sn" --oracle-bug "$SEARCH_BUG" -- -d $D:conn=hid/cp2110/sn=0001A5F8 --samples 1
    emu_case "cp2110 search any" --oracle-bug "$SEARCH_BUG" -- -d $D:conn=hid --samples 1
}

# The model's options for the cases that follow (see
# tools/emu/ftdi-la.sh); none restores EMU_OPTIONS.
cp2110_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "cp2110: the emulator refused '$*'"
}
