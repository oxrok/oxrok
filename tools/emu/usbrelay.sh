# The dcttech relay board (emu/src/models/usbrelay.rs), two relays.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=usbrelay
EMU_OPTIONS="-o relays=2"
EMU_DRIVER=dcttech-usbrelay
EMU_GCOV_SHARED="src/serial.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=dcttech-usbrelay

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
}
