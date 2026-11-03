# The dcttech relay board (emu/src/models/usbrelay.rs), two relays.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=usbrelay
EMU_OPTIONS="-o relays=2"
EMU_DRIVER=dcttech-usbrelay
EMU_GCOV_SHARED="src/serial.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=dcttech-usbrelay

# The model's options for the cases that follow (every case's reset
# builds the model with them); none restores EMU_OPTIONS.
usbrelay_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "usbrelay: the emulator refused '$*'"
}

emu_cases() {
    R=dcttech-usbrelay
    emu_case "relay scan" --keep-header -- -d $R --scan
    emu_case "relay show" --keep-header -- -d $R --show
    emu_case "relay show R1" --keep-header -- -d $R -g R1 --show
    emu_case "relay get R1" --keep-header -- -d $R -g R1 --get enabled
    emu_case "relay set R2 on" --keep-header --no-output -- -d $R -g R2 --set -c enabled=on
    emu_case "relay set all on" --keep-header --no-output -- -d $R --set -c enabled=on
    emu_case "relay set all off" --keep-header --no-output -- -d $R --set -c enabled=off
    emu_case "relay set R1 off" --keep-header --no-output -- -d $R -g R1 --set -c enabled=off
    emu_case "relay by serial" --keep-header -- -d $R:conn=sn=ABCDE -g R2 --get enabled
    # The node the emulator got, as the READY line reported it.
    emu_case "relay by path" --keep-header -- -d $R:conn=$EMU_HIDRAW -g R1 --get enabled
    emu_case "relay by ids" --keep-header -- -d $R:conn=16c0.05df -g R1 --get enabled
    # A serial number no board has: nothing found.
    emu_case "relay by other serial" --keep-header --no-output -- -d $R:conn=sn=ZZZZZ --scan
    # The driver's error paths, through the model's faults
    # (emu/src/models/usbrelay.rs). A serial with a control character:
    # the board is skipped.
    usbrelay_options relays=2 serial=0x4142014445
    emu_case "relay fault non-printable serial" --keep-header --no-output -- -d $R --scan
    # The scan's GET_REPORT STALLed, or answered short: no board.
    usbrelay_options relays=2 stall-get=1
    emu_case "relay fault scan report stalled" --keep-header --no-output -- -d $R --scan
    usbrelay_options relays=2 short-get=1
    emu_case "relay fault scan report short" --keep-header --no-output -- -d $R --scan
    # The second GET_REPORT, the state read before a get, STALLed.
    usbrelay_options relays=2 stall-get=2
    emu_case "relay fault state report stalled" --keep-header -- -d $R -g R1 --get enabled
    # The SET_REPORT that switches a relay STALLed.
    usbrelay_options relays=2 stall-set=1
    emu_case "relay fault switch stalled" --keep-header --expect-fail --stderr -- -d $R -g R2 --set -c enabled=on
    usbrelay_options
}
