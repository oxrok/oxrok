# A bench supply on a serial port (emu/src/models/scpi-pps-serial.rs), on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
# The driver's own paths are scpi-pps.sh's: this holds what a tty changes.
EMU_MODEL=scpi-pps-serial
EMU_CDC=1
EMU_DRIVER="scpi-pps hpib-pps"
EMU_FULL=$EMU_DRIVER
EMU_GCOV=scpi-pps
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    S=scpi-pps:conn=$EMU_TTY
    LOST="a read on a tty times out after 1 s and the driver asks again (protocol.c:97-98), and this model drops an answer nobody read when the next command comes, so C loses that reading and goes on in step, here with I1's first value later than the round it stands in; oxrok keeps one question out and reads the late answer for it"
    NEVER_ENDS="scpi_pps_receive_data returns on a failed read ahead of its limit check (protocol.c:97-98) and its source stays, so on a silent instrument it asks for ever, and not even a time limit ends it; oxrok ends at the limit with a warning"
    emu_case "scan" --keep-header -- -d $S --scan
    emu_case "serialcomm" --keep-header -- -d $S:serialcomm=115200/8n1 --scan
    emu_case "show" --keep-header -- -d $S --show
    emu_case "get voltage target" -- -d $S -g 2 --get voltage_target
    emu_case "set voltage" --no-output -- -d $S -g 2 --config voltage_target=3.3 --set
    emu_case "capture" -- -d $S -C V1,I2 --samples 2
    emu_options model=6632A on=1
    emu_case "hpib" --keep-header -- -d hpib-pps:conn=$EMU_TTY -C V1 --samples 2
    emu_options on=1 mute-at=3
    emu_case "get silent" --no-output --expect-fail -- -d $S -g 1 --get voltage_target
    emu_case "a capture hears nothing" --stderr --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 9 -- -d $S --time 500ms
    emu_options on=1 late-at=4:75
    emu_case "a late answer" --oracle-differs "$LOST" -- -d $S -C V1,I1 --samples 2
}
