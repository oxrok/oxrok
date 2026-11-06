# A Fluke 45 multimeter (emu/src/models/fluke-45.rs) on the CDC-ACM shell. Sourced
# by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=fluke-45
EMU_CDC=1
EMU_DRIVER=fluke-45
EMU_GCOV=fluke-45
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_conns() {
    K=fluke-45:conn=$EMU_TTY
}

emu_cases() {
    emu_conns
    # Where the prompts fall against the commands is the one thing that differs in
    # every capture: C sends its next command on the answer's line end and finds the
    # prompt ahead of the next answer, and at its stop reads one more; oxrok waits for
    # the prompt, as the manual says. Taken out of each direction's bytes here, with
    # the lengths that change with them: "prompt order" pins the order.
    P='s/^bulk_in 82 [0-9]+ -> /bulk_in 82 -> /
s/3d3e0d0a//g
/^bulk_in 82 -> $/d'

    emu_options
    emu_case "scan" --keep-header --tx-filter "$P" -- -d $K --scan
    emu_case "serialcomm" --keep-header --tx-filter "$P" -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "no such node" --keep-header --no-output -- -d fluke-45:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d fluke-45 --scan
    emu_case "show" --keep-header --tx-filter "$P" -- -d $K --show
    emu_case "samples 1" --tx-filter "$P" -- -d $K --samples 1
    emu_case "samples 3 analog" --tx-filter "$P" -- -d $K --samples 3 -O analog
    emu_options func2=FREQ
    emu_case "two displays" --tx-filter "$P" -- -d $K --samples 2
}
