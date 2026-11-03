# An FT2232H in asynchronous bit-bang mode (emu/src/models/ftdi-la.rs),
# port A sampling the count 0, 1, 2, ... that restarts at every bit mode
# change. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# device model". The other chips follow, through the emulator's
# options command.
EMU_MODEL=ftdi-la
EMU_OPTIONS="-o chip=ft2232h"
EMU_DRIVER=ftdi-la
EMU_GCOV_SHARED="src/usb.c"
# oxrok has the driver (src/drivers/ftdi_la): every case runs both tools.
# sigrok-cli runs first in each, and libftdi detaches the kernel's
# ftdi_sio from port A without attaching it again, so oxrok always
# finds the port free. Run alone on a fresh attach, oxrok detaches
# ftdi_sio, and gives the port back when it closes: ftdi_sio binds
# again and probes port A (ctl_in 40 0a 0000 0001, its latency timer,
# and ctl_out 40 09 0010 0001, 16 ms), which sigrok-cli never causes.
# No case shows it: the harness has no way to run oxrok first.
EMU_FULL=ftdi-la

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
    # The driver advertises conn (api.c:39) and config_get() reads
    # sdi->conn for it (api.c:390-394), which scan_device() never sets
    # (api.c:195-202): --get conn fails, and the device line, which
    # sigrok-cli writes with the conn config_get() gives (show.c:313),
    # lacks it. oxrok answers the device's bus.address.
    CONN_BUG="api.c:390-394 reads sdi->conn, which scan_device() (api.c:195-202) never sets"
    # oxrok's device line and --show then print the bus.address. On the
    # first attach it is EMU_USB, which the harness's own conn=EMU_USB
    # rule makes generic and the rules below pin for --show and --get;
    # a variant whose descriptors changed is attached again at another
    # address, which C makes generic.
    U=$(echo "$EMU_USB" | sed 's/[.]/\\./g')
    A="s/^(    conn: )$U( |\$)/\\1BUS.ADDR\\2/"
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    # oxrok's log lines and error texts as sigrok-cli words them: its
    # log prefix, and the sr_strerror() text for its errors, whose
    # cause the driver's own log line (compared as it is) gives.
    E='s/^oxrok: err: oxrok::drivers::ftdi_la(::protocol)?: /sr: ftdi-la: /
s/^oxrok: warn: oxrok::cli::pipeline: //
s/^(Failed to open device): .*/\1./
s/^(Failed to set device option .samplerate.): invalid argument: (invalid samplerate)\.$/\1: \2./
s/^(Failed to set device option .samplerate.): device error: .*/\1: generic\/unspecified error./'
    emu_case "ftdi-la scan" --keep-header --oracle-differs "$CONN_BUG" -- -d $D --scan
    emu_case "ftdi-la show" --keep-header --oracle-differs "$CONN_BUG" --filter "$A" -- -d $D --show
    emu_case "ftdi-la get samplerate" -- -d $D --get samplerate
    # The driver never sets sdi->conn: SR_ERR_ARG.
    emu_case "ftdi-la get conn" --oracle-bug "$CONN_BUG" --filter "s/^'$U'\$/'BUS.ADDR'/" -- -d $D --get conn
    emu_case "ftdi-la set samplerate" --no-output -- -d $D --set -c samplerate=2m
    # No samplerate set: the 9600 Bd of libftdi's open stays.
    emu_case "ftdi-la 100 samples" -- -d $D --samples 100 -O hex
    emu_case "ftdi-la 3.6 kHz" -- -d $D -c samplerate=3600 --samples 64 -O hex
    emu_case "ftdi-la 1 MHz" --record -- -d $D -c samplerate=1m --samples 1000 -O hex
    emu_case "ftdi-la 10 MHz" --filter "$LONG" -- -d $D -c samplerate=10m --samples 17000 -O hex
    # No SR_CONF_LIMIT_MSEC: sigrok-cli turns the time into samples.
    emu_case "ftdi-la time" --filter "$LONG" -- -d $D -c samplerate=100k --time 30 -O hex
    emu_case "ftdi-la channels" -- -d $D --samples 16 -C ADBUS0,ADBUS3
    emu_case "ftdi-la by conn" -- -d $D:conn=$EMU_USB --samples 16 -C ADBUS7
    # The error answers: below the driver's 3600 Hz, beyond what
    # libftdi's divisor reaches within 5 %, and no trigger support.
    emu_case "ftdi-la samplerate too low" --stderr --expect-fail --filter "$E" -- -d $D -c samplerate=1k --samples 10
    emu_case "ftdi-la samplerate too high" --stderr --expect-fail --filter "$E" -- -d $D -c samplerate=100m --samples 10
    # dev_context.cur_samplerate is a uint32_t (protocol.h:44), which
    # config_set() (api.c:426) gives the 64 bit value: 2^32 + 1 MHz is
    # sampled at 1 MHz. oxrok keeps 64 bits, and libftdi refuses the
    # baud rate.
    emu_case "ftdi-la samplerate beyond 32 bits" --oracle-differs "protocol.h:44 keeps the samplerate in a uint32_t, which api.c:426 truncates" --stderr --expect-fail --filter "$E" -- -d $D -c samplerate=4295967296 --samples 100 -O hex
    emu_case "ftdi-la trigger" --stderr --expect-fail -- -d $D --samples 10 --triggers ADBUS0=1

    # The variants. EMU_USB is the first attach's address: a device
    # whose descriptors changed is attached again, so conn= takes ids.
    # FT232R: full speed, 64 byte packets, samplerate / 30 on the 3 MHz
    # clock, the RS232 pin names.
    ftdi_la_options chip=ft232r
    emu_case "ft232r scan" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    emu_case "ft232r 1 MHz" --record -- -d $D -c samplerate=1m --samples 300 -O hex
    emu_case "ft232r 10 MHz" -- -d $D -c samplerate=10m --samples 1000 -O hex
    # FT232H: high speed, but samplerate / 30 like the R.
    ftdi_la_options chip=ft232h
    emu_case "ft232h scan" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    emu_case "ft232h 2 MHz" -- -d $D -c samplerate=2m --samples 300 -O hex
    ftdi_la_options chip=ft4232h
    emu_case "ft4232h scan" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    emu_case "ft4232h 1 MHz" -- -d $D -c samplerate=1m --samples 100 -O hex
    # The TIAO adapter: not among the ids libftdi's scan looks for.
    ftdi_la_options chip=tumpa
    emu_case "tumpa without conn" --stderr --expect-fail -- -d $D --samples 10
    emu_case "tumpa by ids" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D:conn=0403.8a98 --scan
    emu_case "tumpa capture" -- -d $D:conn=0403.8a98 -c samplerate=100k --samples 100 -O hex
    # A BM: the driver's table has 0403:6001 as an FT232R.
    ftdi_la_options chip=ft245bm
    emu_case "ft245bm scan" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    emu_case "ft245bm capture" -- -d $D -c samplerate=500k --samples 100 -O hex
    # An FT230X: libftdi lists it, the driver has no entry for it.
    ftdi_la_options chip=ft230x
    emu_case "ft230x" --stderr --expect-fail -- -d $D --samples 10
    emu_case "ft230x by ids" --stderr --expect-fail -- -d $D:conn=0403.6015 --samples 10
    # The strings an unprogrammed EEPROM lacks: no manufacturer is
    # "Generic", no product the chip's name, no serial number none.
    ftdi_la_options manufacturer= product=
    emu_case "no strings scan" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    emu_case "no strings capture" -- -d $D --samples 50 -O hex
    ftdi_la_options chip=ft232r product=
    emu_case "ft232r no product" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    ftdi_la_options chip=ft232h serial=
    emu_case "ft232h no serial" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    ftdi_la_options chip=ft232h product=
    emu_case "ft232h no product" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    ftdi_la_options chip=ft4232h product=
    emu_case "ft4232h no product" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D --scan
    ftdi_la_options chip=tumpa product=
    emu_case "tumpa no product" --keep-header --oracle-differs "$CONN_BUG" --filter "$C" -- -d $D:conn=0403.8a98 --scan

    # The FTDI shell's faults (emu/src/shells/ftdi.rs), on an FT2232H.
    # A chip with nothing sampled yet answers its two status bytes
    # alone, which libftdi strips: reads of 0 bytes, then the samples.
    ftdi_la_options idle=3
    emu_case "idle reads" -- -d $D -c samplerate=1m --samples 100 -O hex
    # Port A's bulk IN endpoint halts after so many transfers: the read
    # fails and the acquisition stops with what it had.
    ftdi_la_options in-stall=6
    emu_case "read error" --filter "$LONG" -- -d $D -c samplerate=1m --samples 100000 -O hex
    emu_case "read error message" --stderr --filter "$E" -- -d $D -c samplerate=1m --samples 100000 -O hex
    ftdi_la_options in-stall=0
    emu_case "first read error" --stderr --filter "$E" -- -d $D -c samplerate=1m --samples 100 -O hex
    # The requests of the driver's open, each STALLed in turn: libftdi's
    # own reset in ftdi_usb_open (-6), the purge, the reset of the bit
    # mode, bit-bang.
    ftdi_la_options stall=reset/0
    emu_case "open reset stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    ftdi_la_options stall=reset/1
    emu_case "purge stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    # The second purge of ftdi_tcioflush(), the chip's buffer to the
    # host: -2.
    ftdi_la_options stall=reset/2
    emu_case "purge rx stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    ftdi_la_options stall=bitmode/0
    emu_case "bit mode reset stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    ftdi_la_options stall=bitmode/0x0100
    emu_case "bit-bang stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    ftdi_la_options
}
