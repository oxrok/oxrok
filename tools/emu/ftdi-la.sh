# An FT2232H in asynchronous bit-bang mode (emu/src/models/ftdi-la.rs),
# port A sampling the count 0, 1, 2, ... that restarts at every bit mode
# change. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# device model". The other chips follow, through the emulator's
# options command.
EMU_MODEL=ftdi-la
EMU_OPTIONS="-o chip=ft2232h"
EMU_DRIVER=ftdi-la
EMU_GCOV_SHARED="src/usb.c"
# oxrok has no ftdi-la driver yet: these are oracle cases.
EMU_FULL=""

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change. None restores EMU_OPTIONS.
ftdi_la_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "ftdi-la: the emulator refused '$*'"
}

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

    # The variants. EMU_USB is the first attach's address: a device
    # whose descriptors changed is attached again, so conn= takes ids.
    # FT232R: full speed, 64 byte packets, samplerate / 30 on the 3 MHz
    # clock, the RS232 pin names.
    ftdi_la_options chip=ft232r
    emu_case "ft232r scan" --keep-header -- -d $D --scan
    emu_case "ft232r 1 MHz" -- -d $D -c samplerate=1m --samples 300 -O hex
    emu_case "ft232r 10 MHz" -- -d $D -c samplerate=10m --samples 1000 -O hex
    # FT232H: high speed, but samplerate / 30 like the R.
    ftdi_la_options chip=ft232h
    emu_case "ft232h scan" --keep-header -- -d $D --scan
    emu_case "ft232h 2 MHz" -- -d $D -c samplerate=2m --samples 300 -O hex
    ftdi_la_options chip=ft4232h
    emu_case "ft4232h scan" --keep-header -- -d $D --scan
    emu_case "ft4232h 1 MHz" -- -d $D -c samplerate=1m --samples 100 -O hex
    # The TIAO adapter: not among the ids libftdi's scan looks for.
    ftdi_la_options chip=tumpa
    emu_case "tumpa without conn" --stderr --expect-fail -- -d $D --samples 10
    emu_case "tumpa by ids" --keep-header -- -d $D:conn=0403.8a98 --scan
    emu_case "tumpa capture" -- -d $D:conn=0403.8a98 -c samplerate=100k --samples 100 -O hex
    # A BM: the driver's table has 0403:6001 as an FT232R.
    ftdi_la_options chip=ft245bm
    emu_case "ft245bm scan" --keep-header -- -d $D --scan
    emu_case "ft245bm capture" -- -d $D -c samplerate=500k --samples 100 -O hex
    # An FT230X: libftdi lists it, the driver has no entry for it.
    ftdi_la_options chip=ft230x
    emu_case "ft230x" --stderr --expect-fail -- -d $D --samples 10
    emu_case "ft230x by ids" --stderr --expect-fail -- -d $D:conn=0403.6015 --samples 10
    # The strings an unprogrammed EEPROM lacks: no manufacturer is
    # "Generic", no product the chip's name, no serial number none.
    ftdi_la_options manufacturer= product=
    emu_case "no strings scan" --keep-header -- -d $D --scan
    emu_case "no strings capture" -- -d $D --samples 50 -O hex
    ftdi_la_options chip=ft232r product=
    emu_case "ft232r no product" --keep-header -- -d $D --scan
    ftdi_la_options chip=ft232h serial=
    emu_case "ft232h no serial" --keep-header -- -d $D --scan
    ftdi_la_options chip=ft232h product=
    emu_case "ft232h no product" --keep-header -- -d $D --scan
    ftdi_la_options chip=ft4232h product=
    emu_case "ft4232h no product" --keep-header -- -d $D --scan
    ftdi_la_options chip=tumpa product=
    emu_case "tumpa no product" --keep-header -- -d $D:conn=0403.8a98 --scan

    # The FTDI shell's faults (emu/src/shells/ftdi.rs), on an FT2232H.
    # A chip with nothing sampled yet answers its two status bytes
    # alone, which libftdi strips: reads of 0 bytes, then the samples.
    ftdi_la_options idle=3
    emu_case "idle reads" -- -d $D -c samplerate=1m --samples 100 -O hex
    # Port A's bulk IN endpoint halts after so many transfers: the read
    # fails and the acquisition stops with what it had.
    ftdi_la_options in-stall=6
    emu_case "read error" --filter "$LONG" -- -d $D -c samplerate=1m --samples 100000 -O hex
    emu_case "read error message" --stderr -- -d $D -c samplerate=1m --samples 100000 -O hex
    ftdi_la_options in-stall=0
    emu_case "first read error" --stderr -- -d $D -c samplerate=1m --samples 100 -O hex
    # The requests of the driver's open, each STALLed in turn: libftdi's
    # own reset in ftdi_usb_open (-6), the purge, the reset of the bit
    # mode, bit-bang.
    ftdi_la_options stall=reset/0
    emu_case "open reset stalls" --stderr --expect-fail -- -d $D --samples 10
    ftdi_la_options stall=reset/1
    emu_case "purge stalls" --stderr --expect-fail -- -d $D --samples 10
    ftdi_la_options stall=bitmode/0
    emu_case "bit mode reset stalls" --stderr --expect-fail -- -d $D --samples 10
    ftdi_la_options stall=bitmode/0x0100
    emu_case "bit-bang stalls" --stderr --expect-fail -- -d $D --samples 10
    ftdi_la_options
}
