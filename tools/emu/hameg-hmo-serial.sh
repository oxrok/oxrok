# A Hameg scope on a serial port (emu/src/models/hameg-hmo-serial.rs), on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model". The
# driver's own paths are hameg-hmo.sh's: this holds what a tty changes. An HMO1002 by
# default: the scan and the open take 30 queries, so a first request is the 31st.
EMU_MODEL=hameg-hmo-serial
EMU_CDC=1
EMU_DRIVER=hameg-hmo
EMU_FULL=
EMU_GCOV=hameg-hmo
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    A='1~64!d'
    D=hameg-hmo:conn=$EMU_TTY
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "serialcomm" --keep-header -- -d $D:serialcomm=115200/8n1 --scan
    emu_case "set slope" --no-output -- -d $D --config triggerslope=NEG --set
    emu_case "frame" --digest --filter "$A" -- -d $D --frames 1 -O analog
    emu_case "pod" --digest --filter "$A" -- -d $D -C D0 --frames 1 -O bits
    emu_options block-lf=0
    emu_case "no line end after a block" --digest --filter "$A" -- -d $D --frames 1 -O analog
    emu_options cut-at=1
    emu_case "block cut" --digest --filter "$A" -- -d $D -C CH1 --frames 1 -O analog
    emu_options late-at=31:10
    emu_case "block late" --digest --filter "$A" -- -d $D -C CH1 --frames 1 -O analog
    emu_options silent-at=3
    emu_case "silent in the open" --expect-fail -- -d $D --show
    emu_options opc-late=20
    emu_case "opc says 0 a while" --no-output -- -d $D --config triggerslope=NEG --set
    emu_options
}
