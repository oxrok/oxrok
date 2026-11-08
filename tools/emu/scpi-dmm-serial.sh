# A SCPI meter on a serial port (emu/src/models/scpi-dmm-serial.rs), on the CDC-ACM
# shell, as an OWON or a GW Instek is reached. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a SCPI model". The driver's own paths are scpi-dmm.sh's: this
# holds what a tty changes.
EMU_MODEL=scpi-dmm-serial
EMU_CDC=1
EMU_DRIVER=scpi-dmm
EMU_FULL=$EMU_DRIVER
EMU_GCOV=scpi-dmm
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    D=scpi-dmm:conn=$EMU_TTY
    # oxrok's close after --show says SYST:LOC, where sigrok-cli's sends nothing
    # (scpi-dmm.sh, LOCAL_BUG): that line goes from both tools' logs.
    LOCAL='/^bulk_out 02 535953543a4c4f430a -> 9$/d'
    LOCAL_BUG="api.c:453 returns at sdi->status <= SR_ST_INACTIVE, which sr_dev_close has just set (device.c:779), so the close after --set sends no SYST:LOC; oxrok sends it"
    # An XDM2041 with no *OPC?: the probe waits it out, and it is never asked again.
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header --tx-filter "$LOCAL" -- -d $D --show
    emu_case "capture" -- -d $D --samples 2 -O analog
    emu_case "serialcomm" --keep-header -- -d $D:serialcomm=115200/8n1 --scan
    emu_case "serialcomm in a capture" -- -d $D:serialcomm=9600/8n1 --samples 1 -O analog
    emu_case "set a function" --no-output --oracle-differs "$LOCAL_BUG" -- -d $D -c measured_quantity=voltage/ac --set
    emu_options opc=1
    emu_case "scan with opc" --keep-header -- -d $D --scan
    emu_case "capture with opc" -- -d $D --samples 2 -O analog
    emu_options model=GDM8251A func=RES readings=1.20000E+08
    emu_case "a gw" -- -d $D --samples 1 -O analog
    emu_options model=GDM8255A eol=crlf
    emu_case "a gw with cr lf" -- -d $D --samples 1 -O analog
    # What a line can do: pieces, a reading slower than a read, one that doesn't come.
    emu_options opc=1 chunk=3
    emu_case "answers in pieces" -- -d $D --samples 2 -O analog
    emu_options model=34401A reading-pace=25
    emu_case "real rate" -- -d $D --samples 2 -O analog
    emu_case "real rate, a time limit shorter than the pace" -- -d $D --time 100 -O analog
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $D --scan
    # An XDM2041's 5th query is the first MEAS1?, with no *OPC? in a capture.
    emu_options silent-at=5
    emu_case "silent reading and a time limit" --no-output -- -d $D --time 200 -O analog
    FIRST_TIMEOUT="protocol.c:395-397 ends the capture at the first read's timeout, exit 0, where the meter may only be slow; oxrok reads on for its answer"
    emu_options late-at=5:80
    emu_case "late reading" --oracle-differs "$FIRST_TIMEOUT" -- -d $D --samples 1 -O analog
}
