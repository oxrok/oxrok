# A UNI-T UT32x thermometer on its CH9325 cable (emu/src/models/uni-t-ut32x-hid.rs),
# the route most units take. The driver's parsing and limits are in
# tools/emu/uni-t-ut32x.sh; these are what the cable changes. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a device model".
#
# A case names the node: the oracle's libsigrok cannot open a cable it has to search
# for (see tools/emu/ch9325.sh).
EMU_MODEL=uni-t-ut32x-hid
EMU_DRIVER=uni-t-ut32x
EMU_FULL=$EMU_DRIVER
EMU_GCOV=uni-t-ut32x
EMU_GCOV_SHARED="src/serial.c src/serial_hid.c src/serial_hid_ch9325.c"

emu_conns() {
    H=uni-t-ut32x:conn=hid/ch9325/raw=$EMU_HIDRAW
}

ut32x_hid_rec() {
    emu_hex "$1$2$3${4}01230$5"'001\r\n'
}

emu_cases() {
    emu_conns
    F='s/^oxrok: [a-z]+: oxrok::drivers::uni_t_ut32x(::protocol)?: /sr: uni-t-ut32x: /'
    SEARCH_BUG="libsigrok 0bc24877 converts a searched path twice (serial_hid.c ser_hid_chip_search)"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture and sends the stop"
    UNTIL_PACKET="protocol.c:154-156 checks the limits only behind a packet, so a device that says nothing never ends a capture; oxrok ends it"
    UNPLUG_BUG="the read errors end hidapi_source_cb's loop (serial_hid.c:451-458) and nothing tells ut32x_handle_events, so it goes on for ever; oxrok gets an error and ends"
    # ch9325_write_bytes (serial_hid_ch9325.c:130,149-150) sends 8 bytes and sets
    # only the count and the data: the rest is C's stack, oxrok's zeros. A unit test
    # pins those.
    TAIL='s/^(int_out 02 01[0-9a-f]{2})[0-9a-f]{12} -> /\1 and C'"'"'s stack -> /'
    NOHID="the VID.PID form is a tty's name to libsigrok here, and no tty has it; oxrok takes it for the cable"
    T1=$(ut32x_hid_rec 2 :235 1 00 0)
    T2=$(ut32x_hid_rec 2 :241 1 00 1)

    emu_case "scan" --keep-header -- -d $H --scan
    emu_case "show" --keep-header -- -d $H --show
    emu_case "no chip named" --keep-header -- -d uni-t-ut32x:conn=hid/raw=$EMU_HIDRAW --scan

    # The forms users type, which sigrok-cli cannot open: oxrok alone.
    emu_options period=2 burst=1
    emu_case "search chip" --oracle-bug "$SEARCH_BUG" --tx-filter "$TAIL" -- -d uni-t-ut32x:conn=hid/ch9325 --samples 1
    emu_case "search bare" --oracle-bug "$SEARCH_BUG" --tx-filter "$TAIL" -- -d uni-t-ut32x:conn=hid --samples 1
    emu_case "by usb ids" --oracle-bug "$NOHID" --tx-filter "$TAIL" -- -d uni-t-ut32x:conn=1a86.e008 --samples 1

    emu_options period=2 burst=3
    emu_case "three samples" --tx-filter "$TAIL" -- -d $H --samples 3
    emu_options period=2 burst=2
    emu_case "two samples recorded" --record --tx-filter "$TAIL" -- -d $H --samples 2
    emu_options period=2 burst=3 chunk=1
    emu_case "one byte a report" --tx-filter "$TAIL" -- -d $H --samples 3
    emu_options period=2 burst=3 chunk=7
    emu_case "seven bytes a report" --tx-filter "$TAIL" -- -d $H --samples 3
    emu_options period=2 burst=3 idle=2
    emu_case "idle reports" --tx-filter "$TAIL" -- -d $H --samples 3
    emu_options period=1 slots=5
    emu_case "memory" --tx-filter "$TAIL" -- -d $H -c data_source=Memory --samples 5

    # The rate falls back to 2400 on the chip; 9600 is deaf to a unit at 2400.
    emu_options period=2 burst=2 baud=9600
    emu_case "another line, as the device is set" --tx-filter "$TAIL" -- -d $H:serialcomm=9600/8n1 --samples 2
    emu_options period=2 burst=2
    emu_case "a rate the chip has not" --tx-filter "$TAIL" -- -d $H:serialcomm=1200/8n1 --samples 2
    emu_case "time, the line wrong" --oracle-bug "$UNTIL_PACKET" --oracle-hangs --timeout 5 --no-output --tx-filter "$TAIL" -- -d $H:serialcomm=9600/8n1 --time 50

    # The noise that needs a look through the cable's chunks too.
    emu_options period=2 burst=3 live=$(emu_hex 'xxxxx'),$T1,$T2
    emu_case "a record cut by noise is lost" --tx-filter "$TAIL" -- -d $H --samples 1

    # Faults. A chip that takes no setting: the open fails, and that is no device.
    emu_options stall-report=1
    emu_case "the setting stalled" --keep-header --no-output -- -d $H --scan
    # A stalled stop: it logs and the capture still ends.
    emu_options period=2 burst=2 stall-out=2
    emu_case "the stop stalled" --tx-filter "$TAIL" -- -d $H --samples 2

    # A long period: the stop lands ahead of the second record.
    emu_options period=10 burst=2
    emu_case "lib stop" --libsigrok --tx-filter "$TAIL" -- -d $H --continuous --stop-after 1
    emu_options period=2 burst=2
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter "$TAIL" -- -d $H --samples 5
    # A long period: the event point is a period behind the last record, and a hangup
    # loses bytes the tool has not read yet. At 2 offers, 1 run of 15 lost one under load.
    emu_options period=10 burst=2
    emu_case "unplug" --oracle-bug "$UNPLUG_BUG" --oracle-hangs --unplug model --timeout 6 --tx-filter "$TAIL" -- -d $H --samples 5
    emu_options
}
