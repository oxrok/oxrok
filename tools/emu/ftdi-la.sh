# An FT2232H in asynchronous bit-bang mode (emu/src/models/ftdi-la.rs),
# port A sampling the count 0, 1, 2, ... that restarts at every bit mode
# change. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# device model". The other chips (chip=ft232r, ft232h, ft4232h, tumpa,
# ft245bm, ft230x) need a session of their own: one session serves one
# set of options.
EMU_MODEL=ftdi-la
EMU_OPTIONS="-o chip=ft2232h"
EMU_DRIVER=ftdi-la
# oxrok has no ftdi-la driver yet: these are oracle cases.
EMU_FULL=""

emu_cases() {
    D=ftdi-la
    # A long capture keeps its first 16 and its last output lines; the
    # transactions hold every sample.
    LONG='17,$ {$!d}'
    emu_case "ftdi-la scan" --keep-header -- -d $D --scan
    emu_case "ftdi-la show" --keep-header -- -d $D --show
    emu_case "ftdi-la get samplerate" -- -d $D --get samplerate
    # The driver never sets sdi->conn: SR_ERR_ARG.
    emu_case "ftdi-la get conn" --stderr --expect-fail -- -d $D --get conn
    emu_case "ftdi-la set samplerate" --no-output -- -d $D --set -c samplerate=2m
    # No samplerate set: the 9600 Bd of libftdi's open stays.
    emu_case "ftdi-la 100 samples" -- -d $D --samples 100 -O hex
    emu_case "ftdi-la 3.6 kHz" -- -d $D -c samplerate=3600 --samples 64 -O hex
    emu_case "ftdi-la 1 MHz" -- -d $D -c samplerate=1m --samples 1000 -O hex
    emu_case "ftdi-la 10 MHz" --filter "$LONG" -- -d $D -c samplerate=10m --samples 17000 -O hex
    # No SR_CONF_LIMIT_MSEC: sigrok-cli turns the time into samples.
    emu_case "ftdi-la time" --filter "$LONG" -- -d $D -c samplerate=100k --time 30 -O hex
    emu_case "ftdi-la channels" -- -d $D --samples 16 -C ADBUS0,ADBUS3
    emu_case "ftdi-la by conn" -- -d $D:conn=$EMU_USB --samples 16 -C ADBUS7
    # The error answers: below the driver's 3600 Hz, beyond what
    # libftdi's divisor reaches within 5 %, and no trigger support.
    emu_case "ftdi-la samplerate too low" --stderr --expect-fail -- -d $D -c samplerate=1k --samples 10
    emu_case "ftdi-la samplerate too high" --stderr --expect-fail -- -d $D -c samplerate=100m --samples 10
    emu_case "ftdi-la trigger" --stderr --expect-fail -- -d $D --samples 10 --triggers ADBUS0=1
}
