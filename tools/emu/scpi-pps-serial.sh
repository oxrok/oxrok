# A bench supply on a serial port (emu/src/models/scpi-pps-serial.rs), on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
# The driver's own paths are scpi-pps.sh's: this holds what a tty changes.
EMU_MODEL=scpi-pps-serial
EMU_CDC=1
EMU_DRIVER="scpi-pps hpib-pps"
EMU_GCOV=scpi-pps
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    S=scpi-pps:conn=$EMU_TTY
    emu_case "scan" --keep-header -- -d $S --scan
    emu_case "serialcomm" --keep-header -- -d $S:serialcomm=115200/8n1 --scan
    emu_case "show" --keep-header -- -d $S --show
    emu_case "get voltage target" -- -d $S -g 2 --get voltage_target
    emu_case "set voltage" --no-output -- -d $S -g 2 --config voltage_target=3.3 --set
    emu_case "capture" -- -d $S -C V1,I2 --samples 2
    emu_options model=6632A on=1
    emu_case "hpib" --keep-header -- -d hpib-pps:conn=$EMU_TTY -C V1 --samples 2
}
