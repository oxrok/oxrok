# The Microchip PICkit2 logic analyzer (emu/src/models/pickit2.rs): the
# counter pattern (pin4 toggles every sample, pin5 every two, pin6
# every four), the status 300 ms after SETUP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=pickit2
EMU_OPTIONS=""
EMU_DRIVER=microchip-pickit2
EMU_GCOV_SHARED="src/usb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=microchip-pickit2

emu_cases() {
    P=microchip-pickit2
    # --show prints the device's bus.address, which changes with every
    # attach; the conn= the scan line holds is made generic already.
    A='s/^(    conn: )[0-9]+\.[0-9]+/\1BUS.ADDR/'
    emu_case "pickit2 scan" --keep-header -- -d $P --scan
    emu_case "pickit2 show" --keep-header --filter "$A" -- -d $P --show
    emu_case "pickit2 by ids" --keep-header -- -d $P:conn=04d8.0033 --scan
    emu_case "pickit2 by address" --keep-header -- -d $P:conn=$EMU_USB --scan
    emu_case "pickit2 by other ids" --expect-fail --stderr -- -d $P:conn=04d8.0034 --show
    emu_case "pickit2 probe names" --keep-header -- -d $P:probe_names=a,b,c --scan
    emu_case "pickit2 get samplerate" -- -d $P --get samplerate
    emu_case "pickit2 get captureratio" -- -d $P --get captureratio
    emu_case "pickit2 get limit_samples" -- -d $P --get limit_samples
    emu_case "pickit2 get conn" --filter "s/^'[0-9]+\\.[0-9]+'\$/'BUS.ADDR'/" -- -d $P --get conn
    emu_case "pickit2 set samplerate" --no-output -- -d $P --set -c samplerate=250k
    # Every rate the driver offers, 64 samples each.
    for r in 5k 10k 25k 50k 100k 250k 500k 1m; do
        emu_case "pickit2 capture $r" -- -d $P -c samplerate=$r --samples 64 -O bits
    done
    emu_case "pickit2 capture all 1024" -- -d $P --samples 1024 -O hex
    emu_case "pickit2 capture more than memory" -- -d $P --samples 5000 -O hex
    emu_case "pickit2 capture 1 sample" -- -d $P --samples 1 -O bits
    emu_case "pickit2 capture two channels" -- -d $P -C pin4,pin6 --samples 32 -O bits
    emu_case "pickit2 capture probe names" -- -d $P:probe_names=a,b,c --samples 32 -O bits
    # oxrok's message says why after "invalid argument"; cut there.
    emu_case "pickit2 capture bad rate" --expect-fail --stderr --filter 's/(invalid argument).*/\1./' -- -d $P -c samplerate=2m --samples 8 -O bits
    # Triggers on levels and edges, one channel and several.
    emu_case "pickit2 trigger pin4 low" -- -d $P --samples 32 -t pin4=0 -O bits
    emu_case "pickit2 trigger pin5 high" -- -d $P --samples 32 -t pin5=1 -O bits
    emu_case "pickit2 trigger pin4 rising" -- -d $P -c samplerate=500k --samples 32 -t pin4=r -O bits
    emu_case "pickit2 trigger pin6 falling" -- -d $P -c samplerate=100k --samples 32 -t pin6=f -O bits
    emu_case "pickit2 trigger levels" -- -d $P --samples 32 -t pin4=1,pin5=0,pin6=1 -O bits
    emu_case "pickit2 trigger edge and level" -- -d $P --samples 32 -t pin5=r,pin6=0 -O bits
    # Rising edges on pin4 and pin5 at once never come on the counter:
    # the model answers the cancel a press of the button gives, and
    # the capture ends with no samples.
    emu_case "pickit2 trigger never, cancelled" --no-output -- -d $P --samples 32 -t pin4=r,pin5=r -O bits
    # The capture ratio presets, with a trigger (without one the driver
    # sends position 0).
    for c in 0 1 2 3 4 5 6; do
        emu_case "pickit2 captureratio $c" -- -d $P -c captureratio=$c --samples 16 -t pin4=1 -O bits
    done
    emu_case "pickit2 captureratio no trigger" -- -d $P -c captureratio=4 --samples 16 -O bits
    # A ratio outside the presets is ignored, as in libsigrok.
    emu_case "pickit2 captureratio 50" -- -d $P -c captureratio=50 --samples 16 -t pin4=1 -O bits
}
