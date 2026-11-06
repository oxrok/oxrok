# A Hameg scope on a serial port (emu/src/models/hameg-hmo-serial.rs), on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model". The
# driver's own paths are hameg-hmo.sh's: this holds what a tty changes. An HMO1002 by
# default: the scan and the open take 30 queries, so a first request is the 31st.
EMU_MODEL=hameg-hmo-serial
EMU_CDC=1
EMU_DRIVER=hameg-hmo
EMU_FULL=$EMU_DRIVER
EMU_GCOV=hameg-hmo
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    # What C can't show. Its analog packets are empty (see ANALOG_EMPTY), so a case about
    # the wire, or about logic, takes oxrok's analog lines out, and they have their own
    # cases, with AO.
    AO='1~64!d'
    A='/^CH[1-4]: /d
/^(libsigrok|oxrok) [0-9]/d'
    D=hameg-hmo:conn=$EMU_TTY
    NULLKEY='/^    \(null\): $/d'
    ANALOG_EMPTY="protocol.c:1521-1527 sets analog.data and num_samples and then calls sr_analog_init, whose memset clears them (analog.c), so since 8c381a3 every analog packet is empty and sigrok-cli prints no sample; oxrok sends what the scope sent. So wire cases here use -O bits"
    NEVER_ENDS="hmo_receive_data returns TRUE on a failed sr_scpi_get_block (protocol.c:1513) and its source stays, with no bound, so a device that gives no block holds the capture for ever; oxrok's capture fails"
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header --filter "$NULLKEY" -- -d $D --show
    emu_case "serialcomm" --keep-header -- -d $D:serialcomm=115200/8n1 --scan
    emu_case "set slope" --no-output -- -d $D --config triggerslope=NEG --set
    emu_case "frame" --digest --filter "$AO" --oracle-differs "$ANALOG_EMPTY" -- -d $D --frames 1 -O analog
    emu_case "frame wire" --no-output --filter "$A" -- -d $D --frames 1 -O bits
    emu_case "pod" --filter "$A" -- -d $D -C D0 --frames 1 -O bits
    emu_options block-lf=0
    emu_case "no line end after a block" --no-output --filter "$A" -- -d $D --frames 1 -O bits
    emu_options cut-at=1
    emu_case "block cut" --filter "$A" -- -d $D -C D0 --frames 1 -O bits
    emu_options late-at=31:10
    emu_case "block late" --filter "$A" -- -d $D -C D0 --frames 1 -O bits
    emu_options silent-at=3
    emu_case "silent in the open" --expect-fail --filter "$NULLKEY" -- -d $D --show
    emu_options opc-late=20
    emu_case "opc says 0 a while" --no-output -- -d $D --config triggerslope=NEG --set
    # 30 s, as on USBTMC, for what a tty does with it.
    emu_options mute-at=32
    emu_case "no block" --no-output --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 40 --oxrok-fails "has sent no block" -- -d $D -C CH1 --frames 1 -O bits
    emu_options
}
