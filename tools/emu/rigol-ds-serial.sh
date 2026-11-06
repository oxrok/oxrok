# A scope on a serial port (emu/src/models/rigol-ds-serial.rs), on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model". The
# template for a SCPI driver on a tty. The driver's own paths are rigol-ds.sh's: this
# holds what a tty changes.
EMU_MODEL=rigol-ds-serial
EMU_CDC=1
EMU_DRIVER=rigol-ds
EMU_FULL=$EMU_DRIVER
EMU_GCOV=rigol-ds
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    A='1~64!d'
    NO_WAIT="scpi_serial_read_data doesn't block, so right after :WAV:DATA? rigol_ds_read_header reads nothing and returns 0, still reading (protocol.c), and the next callback asks for the block again, so the block lands where *OPC? expects its answer; sigrok-cli ends with no sample. oxrok's read waits for the header"
    emu_case "scan" --keep-header -- -d rigol-ds:conn=$EMU_TTY --scan
    emu_case "show" --keep-header -- -d rigol-ds:conn=$EMU_TTY --show
    emu_case "set level" --no-output -- -d rigol-ds:conn=$EMU_TTY --config triggerlevel=1.5 --set
    emu_case "frame" --digest --filter "$A" --oracle-differs "$NO_WAIT" -- -d rigol-ds:conn=$EMU_TTY -C CH1 --frames 1 -O analog
    emu_case "serialcomm" --keep-header -- -d rigol-ds:conn=$EMU_TTY:serialcomm=115200/8n1 --scan
}
