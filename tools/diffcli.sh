#!/bin/sh
# Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
# Differential check: runs the same command lines through sigrok-cli and
# oxrok and diffs the output after the first header line.
#
#   tools/diffcli.sh [-k substring] [--hw] [--emu] [--emu-model NAME[,NAME]]... [--gcov]
#                    [--no-oracle]
#
# SIGROK_CLI and OXROK name the binaries (defaults: sigrok-cli on PATH
# and target/debug/oxrok). -k runs only the cases whose name contains
# the substring. --hw adds the hardware cases, which need the
# LogicPort and the fx2lafw device attached, and the ESP32 generator for
# the LogicPort cases: HIL names the bench binary that drives it
# (default target/debug/oxrok-hil), which sets idle inputs and a 1 MHz
# clock on CLK1 for the trigger case.
#
# --emu adds the emulated devices (docs/USB.md, "Emulation over USB/IP"):
# for each scenario file tools/emu/MODEL.sh the USB/IP server EMU
# (default target/debug/oxrok-emu) is started under sudo, attaches the
# model through vhci-hcd, and the file's emu_case lines run against it.
# A case is full when oxrok has the case's driver (both tools, outputs
# and transactions compared) and oracle-only when it does not
# (sigrok-cli alone, checked against the goldens in tests/emu/MODEL);
# a driver the scenario lists in EMU_FULL must be oxrok's. Either way
# no pedantic violation may be logged, every tool must print something
# (unless --no-output), and existing goldens are compared. A full case
# where libsigrok is wrong is oxrok's alone against the goldens, with
# sigrok-cli kept in SLUG.oracle: emu_case --oracle-bug when sigrok-cli
# fails (its stderr kept), --oracle-differs when it exits 0 with other
# output or transactions (its stdout kept); either fails once
# sigrok-cli is fixed. EMU_BLESS=1 writes the goldens instead. --emu-model
# runs only the named models (and no other cases); --gcov
# zeroes the coverage counters of the model's libsigrok driver before
# its session and writes tests/emu/MODEL/gcov.txt after it. Sessions
# hold an exclusive lock on EMU_LOCK (default /tmp/oxrok-emu.lock), so
# concurrent runs on one machine take turns; EMULOG=DIR keeps the
# transaction logs as DIR/MODEL.log (default: a temporary directory).
# Firmware stand-ins (tools/emu-firmware.sh) go to EMU_FIRMWARE_DIR,
# exported to both tools as SIGROK_FIRMWARE_DIR. Linux only, needs
# passwordless sudo and `modprobe vhci-hcd`.
#
# --no-oracle (implies --emu) runs the emulated devices without
# sigrok-cli, as CI does: a full case runs oxrok alone against the
# goldens sigrok-cli wrote (.out and .tx, for --oracle-bug and
# --oracle-differs cases too, their .oracle left alone), an
# oracle-only case is skipped and counted, and a model none of whose
# cases is full gets no session at all. A case of a driver EMU_FULL
# lists that oxrok -L lacks fails as in a normal run, and so does a run
# where no case ran. Session-only helpers in a scenario (the *_hold
# claims) return at once while the cases are counted. Before each of
# oxrok's runs, the kernel drivers bound to the device's vendor-specific
# interfaces are unbound (emu_unbind), as libftdi's detach in
# sigrok-cli's run would have left them (in a normal run too, before
# oxrok in an --unplug case, whose replug binds them again). Events,
# --record and the
# violation checks are as in a normal run; the software cases, --hw,
# --gcov and EMU_BLESS=1 need sigrok-cli and are refused.
#
# emu_case --libsigrok makes a case library-verified rather than
# CLI-verified: tools/libsigrok-run.c, built here with cc against the
# oracle's libsigrok (the prefix sigrok-cli runs from, LIBSIGROK_PREFIX to
# override, and glib's headers; the libsigrok.so it loads must be the
# one sigrok-cli loads, so that --gcov counts its runs alike), takes
# sigrok-cli's place, for drivers sigrok-cli cannot drive (no limit it
# can set). It is an oracle case, held to goldens of its own, and
# --no-oracle skips it. With --oxrok it is a full case with the runner in
# sigrok-cli's place: oxrok is held to the same goldens, under --no-oracle
# too, and --oracle-bug, --oracle-differs and --record work as for
# sigrok-cli.
#
# EMU_USB, EMU_USBFS and the node variables follow the device: each
# READY, REPLUGGED and RENUMERATED line replaces them (emu_track), and
# after each tool's run a `sync` makes sure a renumeration it caused
# was reported. A scenario expands them when the case line is read, and
# the case's reset comes after that: a reset that re-attaches the device
# (it renumerated in the case before, or the options changed its
# descriptors) moves it once more, so conn=$EMU_USB holds only on a board
# whose descriptors stay, or after an emu_reset of its own on the line
# before, for a case with one tool.
#
# A scenario that sets EMU_CDC=1 is a model on the CDC-ACM shell, which
# the tools reach through /dev/ttyACMn, EMU_TTY: a session that gets no
# such node fails. ModemManager, when it runs, is told to leave the
# emulated device alone (emu_cdc_quiet). Before each tool's run the tty
# is put back at 9600/8n1 and the model replaced (emu_tty_restore): the
# kernel keeps termios and the line coding it last sent across closes, so
# otherwise what a tool sends would depend on the tool before it. Its
# .tx is normalised (emu_tx_tty): the consecutive bulk transfers of a
# direction are one line, as their cut is the kernel's and the pace's;
# and of a run of consecutive SET_LINE_CODING and SET_CONTROL_LINE_STATE
# what stays is the last line coding, first, when it changes the line,
# then the control line states that change the lines, in their order.
# So a transient frame on the way to the last (the crate serialport's
# cfmakeraw, under oxrok's Native), a repeat of the state the lines are
# in (each open() raises them, and oxrok's Native opens twice), and the
# order of a setting and an RTS change, which no UART can tell, do not
# show; a DTR pulse does. libsigrok swaps odd and even (serial.c:652-655
# passes libserialport's SP_PARITY_* to serial_libsp.c:253-258, which reads
# 1 as even and 2 as odd), so in sigrok-cli's .tx alone bParityType 01 and
# 02 are swapped back; oxrok sends what serialcomm says. A fixed libsigrok
# fails the comparison, which is how that would show. A case whose tty
# moves under the conn= the scenario read (a re-attach that got another
# minor) fails rather than running against nothing.
#
# A scenario that sets EMU_TX_FOLD=1 folds each run of two or more
# consecutive transactions of the same shape (the same line once the
# one @LEN:SHA256 payload digest in it is set aside: direction,
# endpoint, request, lengths) into one line whose digest is
# @LEN:fold=COUNT:SHA256, COUNT the lines folded and SHA256 the digest
# of their text, each line with its newline. It applies after the
# violation count, before the --hangs cut and the comparisons.

set -u
SIGROK_CLI=${SIGROK_CLI:-sigrok-cli}
OXROK=${OXROK:-$(dirname "$0")/../target/debug/oxrok}
HIL=${HIL:-$(dirname "$0")/../target/debug/oxrok-hil}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
# An interrupt stops the run: the emulator (when one runs) is detached
# and the lock released by the exit, rather than every remaining case
# failing against a server the signal took down.
EMUPID=""
TOOLPID=""
on_signal() {
    trap - INT TERM HUP
    echo "interrupted by SIG$1" >&2
    # timeout runs the tool in a process group of its own, which a
    # Ctrl-C does not reach; it passes a TERM on.
    if [ -n "$TOOLPID" ]; then kill -TERM "$TOOLPID" 2>/dev/null; wait "$TOOLPID" 2>/dev/null; fi
    if [ -n "$EMUPID" ]; then emu_stop >/dev/null 2>&1; fi
    exit 130
}
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM
trap 'on_signal HUP' HUP
filter=""
hw=0
emu=0
emu_models=""
gcov=0
no_oracle=0
while [ $# -gt 0 ]; do
    case "$1" in
        -k) filter=$2; shift ;;
        --hw) hw=1 ;;
        --emu) emu=1 ;;
        --emu-model) emu=1; emu_models="$emu_models $(echo "$2" | tr ',' ' ')"; shift ;;
        --gcov) emu=1; gcov=1 ;;
        --no-oracle) emu=1; no_oracle=1 ;;
        *) echo "unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done
# --emu-model runs that model's cases alone.
software=1
[ -n "$emu_models" ] && software=0
if [ $no_oracle = 1 ]; then
    [ $hw = 0 ] || { echo "--no-oracle runs no sigrok-cli, which --hw needs" >&2; exit 2; }
    [ $gcov = 0 ] || { echo "--no-oracle runs no sigrok-cli, which --gcov needs" >&2; exit 2; }
    [ "${EMU_BLESS:-0}" != 1 ] || { echo "--no-oracle runs no sigrok-cli, which EMU_BLESS=1 needs" >&2; exit 2; }
    software=0
fi
pass=0
fail=0
skipped=0

# Drops the lines that name the tool: the version line and the META
# lines sigrok-cli prints for file inputs.
# Frame markers go too: libsigrok's fx2lafw wraps even a single capture
# in a frame and re-arms before stopping, oxrok sends frames only for
# limit_frames other than 1.
strip_header() {
    grep -v -E '^(META |FRAME-)' | sed -E '1{/^(libsigrok|sigrok-cli|oxrok) /d;}' \
        | sed -E 's/^\$(date|version) .*/$\1 X $end/; s/^\$scope module (libsigrok|oxrok) /$scope module X /'
}

# Turns tool T's output into $TMP/T.cmp, as the case's flags say:
# the stream (which), the header (strip), the --filter script, --names,
# and with --digest a last line holding the SHA-256 of the output as it
# was before the filter, so a filter that thins a long output loses
# nothing.
to_cmp() {
    if [ "$which" = stderr ]; then
        # The core's refusals (oxrok::core::config_set) under libsigrok's
        # name; drivers' filters read the one form.
        sed -E 's/^oxrok: err: oxrok::core::hwdriver: /sr: hwdriver: /' "$TMP/$1.err" >"$TMP/$1.cmp"
    elif [ $strip = 1 ]; then
        strip_header <"$TMP/$1.out" >"$TMP/$1.cmp"
    else
        cp "$TMP/$1.out" "$TMP/$1.cmp"
    fi
    sum=""
    [ "${digest:-0}" = 1 ] && sum=$(sha256sum <"$TMP/$1.cmp")
    if [ -n "$filter_expr" ]; then
        sed -E "$filter_expr" "$TMP/$1.cmp" >"$TMP/$1.f"; mv "$TMP/$1.f" "$TMP/$1.cmp"
    fi
    [ -z "$sum" ] || echo "$sum" >>"$TMP/$1.cmp"
    if [ $names = 1 ]; then
        if [ "$1" = a ]; then
            sed -E 's/^([^:]*):.*/\1/' "$TMP/a.cmp" | grep -v -E '^  (gnuplot|scale)$' >"$TMP/a.n"
        else
            sed -E 's/^([^:]*):.*/\1/' "$TMP/b.cmp" >"$TMP/b.n"
        fi
        mv "$TMP/$1.n" "$TMP/$1.cmp"
    fi
}

# case NAME [--keep-header] [--stderr] [--names] [--filter SED] -- ARGS...
#   --names compares only the words before a colon, for listings whose
#   descriptions are worded differently; --filter applies a sed script
#   to both outputs first.
case_() {
    name=$1; shift
    strip=1; which=stdout; names=0; filter_expr=""
    while [ "$1" != "--" ]; do
        case "$1" in
            --keep-header) strip=0 ;;
            --stderr) which=stderr ;;
            --names) names=1 ;;
            --filter) filter_expr=$2; shift ;;
        esac
        shift
    done
    shift
    [ $software = 1 ] || return
    case "$name" in *"$filter"*) ;; *) return ;; esac
    "$SIGROK_CLI" "$@" >"$TMP/a.out" 2>"$TMP/a.err"
    "$OXROK" "$@" >"$TMP/b.out" 2>"$TMP/b.err"
    to_cmp a; to_cmp b
    if cmp -s "$TMP/a.cmp" "$TMP/b.cmp"; then
        pass=$((pass + 1))
        echo "PASS $name"
    else
        fail=$((fail + 1))
        echo "FAIL $name: $*"
        diff "$TMP/a.cmp" "$TMP/b.cmp" | head -${DIFF_LINES:-6}
    fi
}

# Files written by both tools, compared with a reader.
D4="demo:logic_channels=4:analog_channels=0"
D8="demo:logic_channels=8:analog_channels=0"
DA="demo:logic_channels=4:analog_channels=2"

case_ "demo bits walking-one" -- -d $D4 -g Logic -c pattern=walking-one --samples 20 -O bits
case_ "demo bits incremental 8ch" -- -d $D8 -g Logic -c pattern=incremental --samples 300 -O bits
case_ "demo hex graycode" -- -d $D8 -g Logic -c pattern=graycode --samples 100 -O hex
case_ "demo ascii" -- -d $D4 -g Logic -c pattern=walking-zero --samples 50 -O ascii
case_ "demo analog sine" -- -d demo:logic_channels=0:analog_channels=1 -g A0 -c pattern=sine --samples 20 -O analog
case_ "demo analog square amplitude" -- -d demo:logic_channels=0:analog_channels=1 -g A0 -c pattern=square:amplitude=5 --samples 10 -O analog
case_ "demo null" -- -d $D4 --samples 10 -O null
case_ "demo null with analog" -- -d demo:logic_channels=2:analog_channels=1 --samples 4 -O null
case_ "null show" --keep-header -- -O null --show
case_ "demo csv" -- -d $D4 -g Logic -c pattern=walking-one --samples 10 -O csv:header=false:time=true
case_ "demo csv dedup" -- -d $D4 -g Logic -c pattern=walking-one --samples 10 -O csv:header=false:dedup=true:label=channel
case_ "demo vcd" -- -d $D4 -g Logic -c pattern=walking-one --samples 10 -O vcd
case_ "demo binary" -- -d $D8 -g Logic -c pattern=incremental --samples 16 -O binary
case_ "demo time limit" -- -d $D4 -g Logic -c pattern=walking-one --time 1ms -O bits
case_ "demo analog with logic falls back" -- -d $DA -c samplerate=100k -g Logic -c pattern=walking-one --samples 5 -O bits
case_ "demo show" --keep-header -- -d $DA --show
case_ "demo get samplerate" --keep-header -- -d demo --get samplerate
case_ "demo set bad option" --stderr -- -d demo -c nosuch=1 --samples 1
case_ "demo scan option without a value" --stderr -- -d demo:logic_channels --scan
case_ "demo option without a value" --stderr -- -d demo -c samplerate= --set
case_ "demo option with an empty value" --stderr -- -d demo -c pattern= --set
# The core's refusals, hwdriver.c:746-765; the last line without oxrok's reason.
R0='s/(invalid argument).*$/\1./'
case_ "demo samples 0" --stderr -- -d demo --samples 0
case_ "demo samples 2^64-1, read as 0" --stderr -- -d demo --samples 18446744073709551615
case_ "demo limit_samples 0" --stderr --filter "$R0" -- -d demo -c limit_samples=0 --samples 5
case_ "demo samplerate 0" --stderr --filter "$R0" -- -d demo -c samplerate=0 --set
case_ "demo limit_time 0" --stderr --filter "$R0" -- -d demo -c limit_time=0 --set
case_ "demo captureratio 101" --stderr --filter "$R0" -- -d demo -c captureratio=101 --set
# Disabled channels: sigrok's demo masks by the count of enabled ones,
# oxrok's by position (docs/CLI.md), so the selection stays contiguous.
case_ "demo channels selection" -- -d $D8 -g Logic -c pattern=incremental -C D0,D1,D2 --samples 24 -O bits
case_ "demo rename channel" -- -d $D4 -g Logic -c pattern=walking-one -C D0=clk,D1 --samples 8 -O bits
# Triggers: sigrok's demo ignores them, oxrok's honours them, so no case.
case_ "input module show vcd" --keep-header --names -- -I vcd --show
case_ "input module show binary" --keep-header --names -- -I binary --show
# csv: the gnuplot script options are not available in oxrok.
case_ "output module show csv" --keep-header --names -- -O csv --show
case_ "output module show vcd" --keep-header --names -- -O vcd --show
case_ "output module show bits" --keep-header --names -- -O bits --show

# Files round trips: each tool writes, the other reads.
if [ $software = 1 ]; then
    "$SIGROK_CLI" -d $D8 -g Logic -c pattern=incremental --samples 100 -o "$TMP/sr.sr" 2>/dev/null
    "$OXROK" -d $D8 -g Logic -c pattern=incremental --samples 100 -o "$TMP/ox.sr" 2>/dev/null
fi
case_ "sr written by sigrok-cli" -- -i "$TMP/sr.sr" -O bits
case_ "sr written by oxrok" -- -i "$TMP/ox.sr" -O bits
case_ "sr to csv" -- -i "$TMP/sr.sr" -O csv:header=false
if [ $software = 1 ]; then
    "$SIGROK_CLI" -d $D4 -g Logic -c pattern=walking-one --samples 40 -o "$TMP/sr.vcd" -O vcd 2>/dev/null
    "$OXROK" -d $D4 -g Logic -c pattern=walking-one --samples 40 -o "$TMP/ox.vcd" -O vcd 2>/dev/null
fi
case_ "vcd written by sigrok-cli" -- -i "$TMP/sr.vcd" -O bits
# A VCD written by oxrok names its scope "oxrok", which libsigrok's
# reader prefixes to the channel names (docs/CLI.md); only the reverse
# direction diffs clean.
case_ "vcd downsample" -- -i "$TMP/sr.vcd" -I vcd:downsample=4 -O bits
if [ $software = 1 ]; then
    "$SIGROK_CLI" -d $D8 -g Logic -c pattern=incremental --samples 64 -o "$TMP/raw.bin" -O binary 2>/dev/null
fi
case_ "binary input" -- -i "$TMP/raw.bin" -I binary:numchannels=8 -O hex
case_ "file show" --keep-header -- -i "$TMP/sr.sr" --show

# Protocol decoders on a synthetic UART capture.
UART="$(dirname "$0")/../tests/fixtures/uart-hi.vcd"
case_ "uart all annotations verbose" --keep-header -- -l 4 -i "$UART" -P uart:baudrate=115200
case_ "uart rx-data" --keep-header -- -i "$UART" -P uart -A uart=rx-data
case_ "uart ascii start and data" --keep-header -- -i "$UART" -P uart:format=ascii -A uart=rx-data:rx-start
case_ "uart samplenum" --keep-header -- -i "$UART" -P uart -A uart=rx-data --protocol-decoder-samplenum
case_ "uart binary" --keep-header -- -i "$UART" -P uart -B uart=rx
I2C="$(dirname "$0")/../tests/fixtures/i2c-write-read.vcd"
SPI="$(dirname "$0")/../tests/fixtures/spi-mode0.vcd"
case_ "i2c all annotations verbose" --keep-header -- -l 4 -i "$I2C" -P i2c
case_ "i2c unshifted addresses" --keep-header -- -i "$I2C" -P i2c:address_format=unshifted -A i2c=address-write:address-read
case_ "i2c meta" --keep-header -- -i "$I2C" -P i2c -M i2c
case_ "i2c binary" --keep-header -- -i "$I2C" -P i2c -B i2c=data-write
case_ "spi all annotations verbose" --keep-header -- -l 4 -i "$SPI" -P spi
case_ "spi lsb first nibbles" --keep-header -- -i "$SPI" -P spi:bitorder=lsb-first:wordsize=4 -A spi=mosi-data
case_ "spi transfers" --keep-header -- -i "$SPI" -P spi -A spi=mosi-transfer:miso-transfer
case_ "spi binary" --keep-header -- -i "$SPI" -P spi -B spi=miso
UARTSR="$(dirname "$0")/../tests/fixtures/uart-esp32.sr"
SPISR="$(dirname "$0")/../tests/fixtures/spi-esp32.sr"
I2CSR="$(dirname "$0")/../tests/fixtures/i2c-esp32.sr"
case_ "uart real capture" --keep-header -- -l 4 -i "$UARTSR" -P uart:baudrate=921600:rx=D21
case_ "i2c real capture" --keep-header -- -l 4 -i "$I2CSR" -P i2c:scl=D11:sda=D12
case_ "spi real capture" --keep-header -- -l 4 -i "$SPISR" -P spi:clk=D28:miso=D26:mosi=D5:cs=D25
# The documentation text is oxrok's own, and this libsigrokdecode build
# predates decoder tags.
case_ "decoder show uart" --keep-header --names --filter '/^Documentation:/,$d; /^Decoder tags:/,/^Annotation classes:/{/^- /d;/^None\.$/d;}' -- -P uart --show

# Python decoders, when oxrok was built with the python feature.
if "$OXROK" -L 2>/dev/null | grep -q '^  timing '; then
    case_ "python timing" --keep-header -- -l 4 -i "$UART" -P timing
    case_ "python timing options" --keep-header -- -i "$UART" -P timing:edge=rising:avg_period=3
    case_ "python chain uart midi" --keep-header -- -l 4 -i "$UART" -P uart,midi
    case_ "python chain i2c eeprom24xx" --keep-header -- -i "$I2C" -P i2c,eeprom24xx
    case_ "python chain i2c rtc8564" --keep-header -- -i "$I2C" -P i2c,rtc8564
    case_ "python chain spi sdcard" --keep-header -- -l 4 -i "$SPI" -P spi,sdcard_spi
    case_ "python parallel" --keep-header -- -l 4 -i "$SPI" -P parallel:clk=top.clk:d0=top.mosi
    case_ "python decoder show eeprom24xx" --keep-header --names --filter '/^Documentation:/,$d; /^Decoder tags:/,/^Annotation classes:/{/^- /d;/^None\.$/d;}' -- -P eeprom24xx --show
    case_ "decoder listing" --keep-header --filter '1,/^Supported protocol decoders:/d' --names -- -L
fi

if [ $hw = 1 ]; then
    LP="intronix-logicport"
    gen() { [ -x "$HIL" ] && "$HIL" "$@" > /dev/null; }
    gen --clk 0 --set 0
    # The options beyond libsigrok's are filtered out, and the threshold
    # list: the C driver accumulates the 0.05 V step, so its values round
    # differently at one decimal and never equal the current value.
    case_ "logicport show" --keep-header --filter '/^    clock_qualifier:/d; /^    sample_point:/d; /^    invert:/d; /^    frequency_channel:/d; /^    frequency:/d; /^    voltage_threshold:/d' -- -d $LP --show
    case_ "logicport bits 1M" -- -d $LP -c samplerate=1M:voltage_threshold=1.5-1.5 -C D0,D1,CLK1 --samples 64 -O bits
    gen --clk 1000000
    case_ "logicport csv 10M trigger" -- -d $LP -c samplerate=10M:captureratio=25 -C D0,CLK1 --samples 200 -t CLK1=r -O csv:trigger=true:header=false
    gen --clk 0
    case_ "logicport rle" -- -d $LP -c samplerate=1M:rle=on -C CLK1 --samples 500 -O bits
    case_ "logicport 500M" -- -d $LP -c samplerate=500M -C CLK1 --samples 100 -O hex
    # Serial multimeters: the ESP32 plays the meter on its own serial
    # port (EMIT keeps emitting across the reset that opening the port
    # causes), at its fixed 921600 baud.
    DMMPORT=${DMMPORT:-/dev/cu.usbserial-0001}
    gen --cmd "EMIT 200 60 3131323334353b3030303a300d0a"
    case_ "serial dmm ut61e" -- -d uni-t-ut61e-ser:conn=$DMMPORT:serialcomm=921600/8n1 --samples 3
    gen --cmd "EMIT 200 60 1720354d5b617f8297a0b0c0d4e0"
    case_ "serial dmm fs9721" -- -d uni-t-ut60a-ser:conn=$DMMPORT:serialcomm=921600/8n1 --samples 2
    gen --cmd "EMIT 0 0"
    # HID cables: the Pico emulator (hil/pico-hidcable) switches cable
    # type on its serial port and reboots,
    # so each mode change waits for the new device. Skipped when the
    # emulator's port is absent.
    CABLE=${CABLE:-$(dirname "$0")/../hil/pico-hidcable/cable.sh}
    if [ -x "$CABLE" ] && ls /dev/cu.usbmodemHIDCABLE* >/dev/null 2>&1; then
        cable() { "$CABLE" "$@" > /dev/null 2>&1; }
        mode() { cable "MODE $1"; sleep 6; }
        mode bu86x
        cable "PACKET 000001a0dbf8e47c01007cbebf0004008686868600000000" "PERIOD 0" "SAVE"
        case_ "hid dmm bm86x" -- -d brymen-bm86x --samples 2
        mode ch9325
        cable "PACKET 3131323334353b3030303a300d0a" "PERIOD 200" "SAVE"
        case_ "hid dmm ut61e on ch9325" -- -d uni-t-ut61e-ser:conn=hid/ch9325 --samples 2
        mode victor
        cable "PACKET 2b31323334203130000080000d0a" "PERIOD 300" "SAVE"
        case_ "hid dmm victor" -- -d victor-dmm --samples 2
        mode bu86x
    fi
    # A Pico running the sigrok-pico firmware, found by asking every CDC
    # port that is not the HID cable emulator. Inputs float, so the
    # trigger is a level that fires at once.
    PICO=${PICO_PORT:-}
    if [ -z "$PICO" ]; then
        for p in /dev/cu.usbmodem* /dev/ttyACM*; do
            case "$p" in *HIDCABLE*|*\**) continue ;; esac
            if "$OXROK" -d raspberrypi-pico:conn=$p --show >/dev/null 2>&1; then PICO=$p; break; fi
        done
    fi
    if [ -n "$PICO" ]; then
        RP="raspberrypi-pico:conn=$PICO"
        case_ "pico show" --keep-header -- -d $RP --show
        case_ "pico bits d4" -- -d $RP -c samplerate=1M -C D2,D3,D4 --samples 12 -O bits
        case_ "pico hex 5 channels" -- -d $RP -c samplerate=1M -C D2,D3,D4,D5,D6 --samples 20 -O hex
        # The analog inputs float, so their values differ between runs:
        # only the shape of the mixed output is compared.
        case_ "pico mixed csv" --filter 's/[0-9]*\.[0-9][0-9]*/V/g' -- -d $RP -c samplerate=100k -C A0,A1,D2,D3 --samples 3 -O csv:header=false
        case_ "pico trigger csv" -- -d $RP -c samplerate=1M -C D2,D3 -t D2=0 --samples 40 -O csv:trigger=true:header=false
    fi
    # A Rigol scope, or the Pico emulating a DS1102E on USBTMC.
    if "$OXROK" -d rigol-ds --scan 2>/dev/null | grep -q rigol-ds; then
        case_ "rigol show" --keep-header -- -d rigol-ds --show
        case_ "rigol show CH1" --keep-header -- -d rigol-ds -g CH1 --show
        case_ "rigol frame" -- -d rigol-ds -C CH1 --frames 1 -O analog
        case_ "rigol two frames two channels" -- -d rigol-ds -C CH1,CH2 --frames 2 -O analog
        # sigrok-cli applies every -c to its one -g, so device and group
        # settings are separate cases.
        case_ "rigol device settings" -- -d rigol-ds -c timebase=500us:triggersource=CH2:triggerslope=f --frames 1 -C CH1 -O analog
        case_ "rigol group settings" -- -d rigol-ds -g CH1 -c vdiv=500mV:coupling=AC:probe_factor=1 --frames 1 -C CH1 -O analog
    fi
    case_ "fx2lafw show" --keep-header -- -d fx2lafw --show
    case_ "fx2lafw bits 1M" -- -d fx2lafw -c samplerate=1M --samples 64 -O bits
    case_ "fx2lafw hex 24M" -- -d fx2lafw -c samplerate=24M --samples 1024 -O hex
fi

if [ $emu = 1 ]; then
    TOOLS=$(dirname "$0")
    EMU=${EMU:-$TOOLS/../target/debug/oxrok-emu}
    EMU_LOCK=${EMU_LOCK:-/tmp/oxrok-emu.lock}
    EMU_LOCK_WAIT=${EMU_LOCK_WAIT:-1800}
    EMU_GOLDEN=${EMU_GOLDEN:-$(cd "$TOOLS/.." && pwd)/tests/emu}
    EMU_FIXTURES=${EMU_FIXTURES:-$(cd "$TOOLS/.." && pwd)/tests/fixtures}
    EMU_BLESS=${EMU_BLESS:-0}
    EMU_CASE_TIMEOUT=${EMU_CASE_TIMEOUT:-60}
    EMULOG=${EMULOG:-$TMP}
    mkdir -p "$EMULOG"
    emu_fail() { fail=$((fail + 1)); echo "FAIL $*"; }
    EMU_DRY=0
    # The drivers oxrok has: a case of one of them runs under both tools.
    # An oxrok that cannot list them would turn every case into an
    # oracle case, so that fails the run.
    if ! "$OXROK" -L >"$TMP/drivers" 2>"$TMP/drivers.err"; then
        emu_fail "$OXROK -L: $(head -1 "$TMP/drivers.err")"
    fi
    OXROK_DRIVERS=" $(sed -n '/^Supported hardware drivers:/,/^$/s/^  \([^ ]*\) .*/\1/p' "$TMP/drivers" | tr '\n' ' ') "
    [ "$OXROK_DRIVERS" != "  " ] || emu_fail "$OXROK -L lists no hardware driver; every case would be oracle-only"
    EMU_FIRMWARE_DIR=${EMU_FIRMWARE_DIR:-$HOME/work/sigrok/firmware-standin}
    # oxrok searches its own variable first; the stand-ins must be what
    # both tools upload.
    unset OXROK_FIRMWARE_DIR
    if sh "$TOOLS/emu-firmware.sh" "$EMU_FIRMWARE_DIR" >"$TMP/fw.out" 2>&1; then
        export SIGROK_FIRMWARE_DIR="$EMU_FIRMWARE_DIR"
    else
        emu_fail "firmware stand-ins"; cat "$TMP/fw.out"
    fi
    # A --hangs case stops sigrok-cli with timeout's SIGTERM, of which
    # it would die without writing its gcov counts. This preload makes
    # the signal an exit(), whose handlers write them. An exit() from a
    # signal handler can deadlock (the signal may land inside a lock an
    # exit handler takes; it does now and then in hantek-6xxx), so an
    # alarm ends such a run 2 s later, without counts but still as a
    # timed out run (status 124). Without a C compiler the case still
    # runs; it only counts nothing.
    EMU_TERM_EXIT=""
    printf '%s\n' '#include <signal.h>' '#include <stdlib.h>' '#include <unistd.h>' \
        'static void on_term(int s) { (void)s; alarm(2); exit(1); }' \
        '__attribute__((constructor)) static void init(void) { signal(SIGTERM, on_term); }' \
        >"$TMP/term-exit.c"
    if cc -shared -fPIC -o "$TMP/term-exit.so" "$TMP/term-exit.c" 2>"$TMP/term-exit.err"; then
        EMU_TERM_EXIT=$TMP/term-exit.so
    else
        echo "note: no SIGTERM preload for --hangs cases (cc failed), so they write no gcov counts"
    fi

    # The state letter of a process (/proc/PID/stat), empty when gone.
    emu_pstate() {
        st=""
        [ -r "/proc/$1/stat" ] && read -r _ _ st _ <"/proc/$1/stat" 2>/dev/null
        echo "$st"
    }
    # The sudo process in front of the emulator and the emulator itself.
    emu_pids() {
        [ -n "${EMUPID:-}" ] || return 0
        echo "$EMUPID"; pgrep -P "$EMUPID" 2>/dev/null || true
    }
    # Whether the emulator still runs; a zombie does not count.
    emu_alive() {
        [ -n "${EMUPID:-}" ] || return 1
        st=$(emu_pstate "$EMUPID")
        [ -n "$st" ] && [ "$st" != Z ]
    }
    # Whether it, or sudo, is stopped (SIGSTOP, a debugger): it cannot
    # answer until someone continues it.
    emu_stopped() {
        for p in $(emu_pids); do
            case $(emu_pstate "$p") in T | t) return 0 ;; esac
        done
        return 1
    }
    # Sends signal $1 to sudo and the emulator, a stopped one continued.
    emu_signal() {
        pids=$(emu_pids | tr '\n' ' ')
        [ -n "$pids" ] || return 0
        sudo -n kill -"$1" $pids 2>/dev/null
        [ "$1" = KILL ] || sudo -n kill -CONT $pids 2>/dev/null
        return 0
    }
    # emu_gone TICKS: waits up to TICKS times 50 ms for the emulator to end.
    emu_gone() {
        i=0
        while emu_alive && [ $i -lt "$1" ]; do sleep 0.05; i=$((i + 1)); done
        ! emu_alive
    }
    # emu_wait SKIP GREP-ARGS...: waits until a line after the first
    # SKIP lines of the server's output matches; fails when the server
    # exits, is stopped, or after 15 s. A server that is stopped or did
    # not answer counts as dead from then on (EMU_DEAD), so the cases
    # after it fail at once instead of waiting 15 s each.
    emu_wait() {
        skip=$1; shift
        i=0
        while :; do
            if tail -n +$((skip + 1)) "$TMP/emu.out" 2>/dev/null | grep -q "$@"; then return 0; fi
            if ! emu_alive; then
                # It may have answered just before exiting.
                tail -n +$((skip + 1)) "$TMP/emu.out" 2>/dev/null | grep -q "$@" && return 0
                echo "emulator exited:" >&2; sed 's/^/  /' "$TMP/emu.err" >&2
                return 1
            fi
            if emu_stopped; then
                EMU_DEAD=1
                echo "emulator: stopped (process state T); not waiting for it" >&2
                return 1
            fi
            i=$((i + 1))
            if [ $i -gt 300 ]; then
                EMU_DEAD=1
                echo "emulator: no answer after 15 s" >&2; sed 's/^/  /' "$TMP/emu.err" >&2
                return 1
            fi
            sleep 0.05
        done
    }
    # Sends a command and waits for its own acknowledgement.
    emu_cmd() {
        # Counting the cases of a --no-oracle model without a session.
        [ "$EMU_DRY" = 0 ] || return 0
        emu_alive || { echo "emulator: not running for '$1'" >&2; return 1; }
        [ "$EMU_DEAD" = 0 ] || { echo "emulator: not answering, so no '$1'" >&2; return 1; }
        lines=$(wc -l <"$TMP/emu.out")
        printf '%s\n' "$1" >&7 || return 1
        emu_wait "$lines" -F -x "OK $1" || return 1
        emu_track
    }
    # Takes over the nodes and address of each attach the server
    # reported since the last call (READY, REPLUGGED, and RENUMERATED
    # after a renumeration or a reset that re-attached the device), in
    # order, so that EMU_USB and the rest name the device as it is now
    # and the outputs name every attach of the session generically. The
    # server prints such a line before the OK of any command it takes
    # in after the attach.
    emu_track() {
        total=$(wc -l <"$TMP/emu.out")
        [ "$total" -gt "$EMU_SEEN" ] || return 0
        tail -n +$((EMU_SEEN + 1)) "$TMP/emu.out" | head -n $((total - EMU_SEEN)) \
            | grep -E '^(READY|REPLUGGED|RENUMERATED) ' >"$TMP/emu.attach"
        EMU_SEEN=$total
        while IFS= read -r l; do emu_fields "$l"; done <"$TMP/emu.attach"
    }
    emu_start() {
        [ "$EMU_CDC" != 1 ] || emu_cdc_quiet
        : >"$TMP/emu.out"; : >"$TMP/emu.err"
        EMU_DEAD=0
        rm -f "$TMP/emu.ctl"; mkfifo "$TMP/emu.ctl"
        sudo -n "$EMU" serve "$EMU_MODEL" $EMU_OPTIONS --listen 127.0.0.1:0 --attach \
            --open-devices --log "$EMULOG/$EMU_MODEL.log" \
            <"$TMP/emu.ctl" >"$TMP/emu.out" 2>"$TMP/emu.err" 7>&- 8<&- &
        EMUPID=$!
        # Read-write, so neither end blocks opening it; closing it is
        # the end of input that detaches the device.
        exec 7<>"$TMP/emu.ctl"
        emu_wait 0 '^READY ' || return 1
        : >"$TMP/emu.sed"
        EMU_SEEN=0
        emu_track
        if [ "$EMU_CDC" = 1 ]; then
            case "$EMU_TTY" in
                /dev/ttyACM*) ;;
                *)
                    echo "$EMU_MODEL: the device got no /dev/ttyACMn (nodes: $EMU_NODES); is the kernel's cdc_acm there? (modprobe cdc_acm)" >&2
                    return 1 ;;
            esac
        fi
    }
    # emu_fields LINE: the EMU_* values from a READY, REPLUGGED or
    # RENUMERATED line, and the rules that make its node names and address generic
    # in the outputs, added to those of the attaches before.
    emu_fields() {
        EMU_PORT=""; EMU_BUSID=""; EMU_USB=""; EMU_USBFS=""; EMU_NODES=""
        for kv in $1; do
            case "$kv" in
                port=*) EMU_PORT=${kv#port=} ;;
                busid=*) EMU_BUSID=${kv#busid=} ;;
                usb=*) EMU_USB=${kv#usb=} ;;
                usbfs=*) EMU_USBFS=${kv#usbfs=} ;;
                nodes=*) EMU_NODES=${kv#nodes=} ;;
            esac
        done
        first_node() { echo "$EMU_NODES" | tr ',' '\n' | grep -m1 "^$1" || true; }
        EMU_HIDRAW=$(first_node /dev/hidraw)
        EMU_TTY=$(first_node /dev/tty)
        EMU_USBTMC=$(first_node /dev/usbtmc)
        export EMU_PORT EMU_BUSID EMU_USB EMU_USBFS EMU_NODES EMU_HIDRAW EMU_TTY EMU_USBTMC
        # The node numbers and the device address change from one attach
        # to the next; the outputs name them generically.
        for n in $(echo "$EMU_NODES" | tr ',' ' ') $EMU_USBFS; do
            gen=$(echo "$n" | sed -E 's#^/dev/bus/usb/[0-9]+/[0-9]+$#/dev/bus/usb/BBB/DDD#; s#[0-9]+$#N#')
            printf 's#%s([^0-9]|$)#%s\\1#g\n' "$(echo "$n" | sed 's/[.]/\\./g')" "$gen" >>"$TMP/emu.sed"
        done
        if [ -n "$EMU_USB" ]; then
            printf 's#conn=%s([^0-9]|$)#conn=BUS.ADDR\\1#g\n' "$(echo "$EMU_USB" | sed 's/[.]/\\./g')" >>"$TMP/emu.sed"
        fi
    }
    # Detaches and ends the emulator, whatever state it is in: a
    # server that does not end after the detach is terminated, then
    # killed (a stopped one too, which a TERM alone would leave
    # waiting), so the session never waits on it for ever. The detach
    # traffic gets a section of its own.
    emu_stop() {
        if [ "${EMU_DEAD:-0}" = 0 ]; then
            emu_cmd "section detach" >/dev/null 2>&1 && emu_cmd detach >/dev/null 2>&1
        fi
        exec 7>&-
        if [ "$EMU_DEAD" = 1 ] && emu_alive; then
            echo "emulator: not answering; terminating it" >&2
        fi
        if [ "$EMU_DEAD" = 1 ] || ! emu_gone 200; then
            [ "$EMU_DEAD" = 1 ] || echo "emulator: still running 10 s after detach; terminating it" >&2
            emu_signal TERM
            if ! emu_gone 60; then
                echo "emulator: still running 3 s after SIGTERM; killing it" >&2
                emu_signal KILL
                emu_gone 60
            fi
        fi
        if emu_alive; then
            echo "emulator: pid $EMUPID survived SIGKILL; leaving it" >&2
            emu_status=1
        else
            wait "$EMUPID" 2>/dev/null
            emu_status=$?
        fi
        [ "$EMU_DEAD" = 0 ] || [ $emu_status != 0 ] || emu_status=1
        EMUPID=""
        [ "$EMU_CDC" != 1 ] || emu_cdc_unquiet
        sudo -n "$EMU" cleanup --locked >&2
        return $emu_status
    }
    # The transaction lines of a section of the log, comments dropped.
    # The name goes through the environment: awk -v would turn its
    # backslashes into escapes.
    # --no-oracle: oxrok runs where sigrok-cli would have run before it,
    # and some of libsigrok's libraries detach a kernel driver from the
    # interface and leave it detached (libftdi: ftdi_sio). Each kernel
    # driver still bound to a vendor-specific interface (class ff) of
    # the device, whichever driver and interface, is unbound through
    # sysfs, as root, so that oxrok meets the device as in a normal run.
    # A class interface (HID, CDC, USBTMC) keeps its driver: both tools
    # reach it through the driver's node (hidraw, tty, usbtmc), which
    # the unbind would remove. A claim a process holds (usbfs, the
    # *_hold helpers of the scenarios) is left alone. (A future model
    # that both tools use through the tty of a vendor serial chip,
    # ftdi_sio or cp210x, would need its driver kept: none does yet.)
    # Each unbind is a section of the
    # session log of its own, so that whatever traffic it causes is in
    # no case's transactions.
    emu_unbind() {
        [ -n "$EMU_BUSID" ] || return 0
        for intf in /sys/bus/usb/devices/"$EMU_BUSID":*; do
            [ -e "$intf/driver" ] || continue
            drv=$(basename "$(readlink "$intf/driver")")
            [ "$drv" != usbfs ] || continue
            [ "$(cat "$intf/bInterfaceClass" 2>/dev/null)" = ff ] || continue
            intf=$(basename "$intf")
            emu_cmd "section unbind $drv from $intf before $1" >/dev/null || return 1
            printf '%s' "$intf" | sudo -n tee "/sys/bus/usb/drivers/$drv/unbind" >/dev/null || return 1
        done
    }
    # EMU_CDC=1, before attaching: a ModemManager whose filter lets a
    # ttyACM through opens it and sends AT commands, which would land in
    # the case's transactions. Version 1.24's default filter forbids the
    # emulated device whatever its interface's protocol. For a version or
    # a setting that does not, the rule is udev's own way to keep it off,
    # in /run, for the emulated devices only.
    # DEVPATH: udev's SUBSYSTEMS and KERNELS must match one ancestor, and
    # vhci_hcd.0 is a platform device. emu_stop removes the rule.
    EMU_MM_RULE=/run/udev/rules.d/90-oxrok-emu-mm.rules
    emu_cdc_quiet() {
        systemctl is-active -q ModemManager 2>/dev/null || return 0
        sudo -n mkdir -p /run/udev/rules.d
        printf '%s\n' 'DEVPATH=="*/vhci_hcd.0/*", ENV{ID_MM_DEVICE_IGNORE}="1"' \
            | sudo -n tee "$EMU_MM_RULE" >/dev/null
        sudo -n udevadm control --reload
    }
    emu_cdc_unquiet() {
        [ -e "$EMU_MM_RULE" ] || return 0
        sudo -n rm -f "$EMU_MM_RULE"
        sudo -n udevadm control --reload
    }
    # emu_tty_closing_wait CENTISECONDS, in a scenario, before a case whose
    # device stops reading: how long the kernel holds a tool's close for
    # bytes still in flight (30 s by default), through TIOCSSERIAL as root.
    # It lasts until the next attach, so a scenario puts 3000 back after.
    # serial_struct's closing_wait is at offset 40 on Linux.
    emu_tty_closing_wait() {
        [ "$EMU_DRY" = 0 ] || return 0
        emu_cmd "section tty closing_wait $1" >/dev/null || return 1
        sudo -n python3 -I -c '
import fcntl, os, struct, sys
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
info = bytearray(128)
fcntl.ioctl(fd, 0x541E, info)
struct.pack_into("H", info, 40, int(sys.argv[2]))
fcntl.ioctl(fd, 0x541F, info)
' "$EMU_TTY" "$1" || emu_fail "$EMU_MODEL: could not set closing_wait"
    }
    # EMU_CDC=1, before each tool's run: the tty back at the state a new
    # attach has, then a fresh model that knows nothing of the opening
    # and closing that took. In a section of its own, as an unbind is.
    emu_tty_restore() {
        emu_cmd "section tty restore before $1" >/dev/null || return 1
        stty -F "$EMU_TTY" 9600 cs8 -parenb -cstopb -crtscts -ixon -ixoff clocal hupcl raw -echo \
            || return 1
        emu_cmd reset >/dev/null
    }
    # EMU_CDC=1: the transactions as a tty's, see the head of this file.
    # Joined data longer than 1 KiB is its digest, as the fixtures have it.
    emu_tx_tty() {
        python3 -I -c '
import hashlib, re, sys
OUT = re.compile(r"bulk_out ([0-9a-f]{2}) ([0-9a-f]+|-) -> [0-9]+$")
IN = re.compile(r"bulk_in ([0-9a-f]{2}) [0-9]+ -> ([0-9a-f]+|-)$")
SETUP = re.compile(r"ctl_out 21 (20|22) ")
key, data, setup = None, "", []
line = "80250000000008 -> ok"
lines = "0000"
stalled = ""
def flush_data():
    global key, data
    if key and data:
        shown = data
        size = len(shown) // 2
        if size > 1024:
            shown = "@%d:%s" % (size, hashlib.sha256(bytes.fromhex(data)).hexdigest())
        if key[0] == "out":
            print("bulk_out %s %s -> %d" % (key[1], shown, size))
        else:
            print("bulk_in %s %d -> %s" % (key[1], size, shown))
    key, data = None, ""
def flush_setup():
    global setup, line, lines, stalled
    coding = [l for l in setup if l.split()[2] == "20"]
    if coding:
        last = coding[-1].split(" ", 5)[5]
        if last != line:
            print(coding[-1])
        line = last
    for l in setup:
        if l.split()[2] != "22":
            continue
        value = l.split()[3]
        if "stall" in l:
            if l != stalled:
                print(l)
            stalled = l
        elif value != lines:
            print(l)
            lines = value
    setup = []
def swap_parity(text):
    w = text.split(" ")
    if w[2] == "20" and len(w[5]) == 14 and w[5][10:12] in ("01", "02"):
        w[5] = w[5][:10] + ("02" if w[5][10:12] == "01" else "01") + w[5][12:]
    return " ".join(w)
for text in sys.stdin:
    text = text.rstrip("\n")
    m = OUT.match(text)
    k = ("out", m.group(1)) if m else None
    if not m:
        m = IN.match(text)
        k = ("in", m.group(1)) if m else None
    if m:
        flush_setup()
        if k != key:
            flush_data()
        key = k
        if m.group(2) != "-":
            data += m.group(2)
        continue
    flush_data()
    if SETUP.match(text):
        setup.append(swap_parity(text) if sys.argv[1] == "a" else text)
        continue
    flush_setup()
    print(text)
flush_data()
flush_setup()
' "$1"
    }
    # emu_case --bound DRV: binds DRV to the device's vendor interfaces, so that
    # each tool starts as on a fresh plug in, not after the other's detach. Its
    # probe is a section of its own. A kernel without DRV runs the case unbound,
    # with a note, as the transactions then show nothing else.
    emu_bind() {
        [ -n "$EMU_BUSID" ] || return 0
        if [ ! -d "/sys/bus/usb/drivers/$2" ]; then
            echo "note: $1: no $2 in this kernel, the case runs unbound"
            return 0
        fi
        for intf in /sys/bus/usb/devices/"$EMU_BUSID":*; do
            [ "$(cat "$intf/bInterfaceClass" 2>/dev/null)" = ff ] || continue
            [ ! -e "$intf/driver" ] || continue
            intf=$(basename "$intf")
            emu_cmd "section bind $2 to $intf before $1" >/dev/null || return 1
            printf '%s' "$intf" | sudo -n tee "/sys/bus/usb/drivers/$2/bind" >/dev/null || return 1
        done
    }
    emu_tx() {
        EMU_SECTION="# section $1" awk '$0 == ENVIRON["EMU_SECTION"] { on = 1; next }
            /^# section / { on = 0 } on && !/^#/' "$EMULOG/$EMU_MODEL.log"
    }
    # The transactions of a --hangs case, made deterministic: a driver
    # that never ends may repeat its last steps for as long as the
    # timeout lets it, so where the log is periodic up to its end (a
    # period of at most EMU_HANG_PERIOD lines, default 512, seen at
    # least three times), it is cut after the first full period and a
    # `~` line says so. Three periods are all the cut checks: they do
    # not tell a loop that runs until the timeout from one that stops
    # after a few rounds, and the line claims no more than that. Of the periods, the one that keeps the fewest lines wins (an
    # outer loop over an inner one). A log without such a tail, a driver
    # that waits without a transfer, is kept whole.
    emu_hang_cut() {
        awk -v pmax="${EMU_HANG_PERIOD:-512}" '
            { l[NR] = $0 }
            END {
                n = NR; best = n; bp = 0
                for (p = 1; p <= pmax && 3 * p <= n && p < best; p++) {
                    # l[s..n] repeats with period p from s = j - p + 1.
                    j = n
                    while (j > p && l[j] == l[j - p]) j--
                    s = j - p + 1
                    if (n - s + 1 >= 3 * p && j < best) { best = j; bp = p }
                }
                for (i = 1; i <= best; i++) print l[i]
                if (bp) printf "~ the last %d lines repeat to the end (3 or more times)\n", bp
            }'
    }
    # EMU_TX_FOLD=1: runs of transactions of the same shape folded into
    # one line each (see the head of this file).
    emu_tx_fold() {
        python3 -I -c '
import hashlib, re, sys
digest = re.compile(r"@([0-9]+):([0-9a-f]{64})")
key, run = None, []
def flush():
    if len(run) > 1:
        h = hashlib.sha256("".join(l + "\n" for l in run).encode()).hexdigest()
        print(key.replace("\0", "fold=%d:%s" % (len(run), h)))
    else:
        for l in run:
            print(l)
for line in sys.stdin:
    line = line.rstrip("\n")
    m = list(digest.finditer(line))
    k = None
    if len(m) == 1 and line[:1] not in "!#~":
        k = line[:m[0].start(2)] + "\0" + line[m[0].end(2):]
    if k is not None and k == key:
        run.append(line)
        continue
    flush()
    key, run = k, [line]
flush()
'
    }
    emu_slug() {
        echo "$1" | tr 'A-Z' 'a-z' | sed -E 's/[^a-z0-9]+/-/g; s/^-//; s/-$//'
    }

    # A reset of its own before a case line that names the address: a
    # board the case's reset would re-attach (it renumerated in the case
    # before, or the options changed its descriptors) moves now, and
    # EMU_USB, read when the case line runs, follows. The case's reset
    # then changes nothing. Good for one run: a full case's second tool
    # meets a board the first one moved.
    emu_reset() { [ "$EMU_DRY" = 1 ] || emu_cmd reset >/dev/null; }

    # Puts a device an armed unplug took off the bus back on it, and
    # takes over the new attach's nodes and address.
    emu_replug() {
        emu_cmd replug >/dev/null
    }
    # emu_interrupt SKIP PID: once the emulator prints EVENT interrupt
    # after line SKIP of its output, sends SIGINT to the children of PID
    # (timeout's tool); gives up when PID has ended.
    emu_interrupt() {
        while kill -0 "$2" 2>/dev/null; do
            if tail -n +$(($1 + 1)) "$TMP/emu.out" | grep -q -x 'EVENT interrupt'; then
                pkill -INT -P "$2"
                return 0
            fi
            sleep 0.02
        done
    }

    # Builds tools/libsigrok-run.c once per run against the oracle's
    # libsigrok: the prefix sigrok-cli runs from (LIBSIGROK_PREFIX
    # overrides it), glib's headers, and the libsigrok.so sigrok-cli
    # loads, which the check after the build holds it to (the
    # --coverage build then counts the runner's runs in the same .gcda
    # files). Sets EMU_LIBSIGROK_RUN, or EMU_LIBSIGROK_ERR on failure.
    EMU_LIBSIGROK_RUN=""
    EMU_LIBSIGROK_ERR=""
    emu_libsigrok_run() {
        [ -z "$EMU_LIBSIGROK_RUN" ] || return 0
        [ -z "$EMU_LIBSIGROK_ERR" ] || return 1
        cli=$(command -v "$SIGROK_CLI") || { EMU_LIBSIGROK_ERR="no $SIGROK_CLI on PATH"; return 1; }
        prefix=${LIBSIGROK_PREFIX:-$(dirname "$(dirname "$cli")")}
        libdir=$prefix/lib
        [ -e "$libdir/libsigrok.so" ] || libdir=$prefix/lib64
        [ -e "$libdir/libsigrok.so" ] || { EMU_LIBSIGROK_ERR="no libsigrok.so under $prefix"; return 1; }
        # Not libsigrok.pc: its Requires.private ask for the -devel
        # package of every library libsigrok links, of which the runner
        # uses none. libsigrok.h needs glib's headers alone.
        if ! flags=$(pkg-config --cflags --libs glib-2.0 2>"$TMP/lsr.err"); then
            EMU_LIBSIGROK_ERR="pkg-config glib-2.0 (the glib headers, glib2-devel): $(head -1 "$TMP/lsr.err")"; return 1
        fi
        # shellcheck disable=SC2086
        if ! cc -std=gnu11 -O1 -o "$TMP/libsigrok-run" "$TOOLS/libsigrok-run.c" -I"$prefix/include" $flags \
                -L"$libdir" -lsigrok -Wl,-rpath,"$libdir" -Wl,-rpath-link,"$libdir" 2>"$TMP/lsr.err"; then
            EMU_LIBSIGROK_ERR="cc failed: $(head -3 "$TMP/lsr.err" | tr '\n' ' ')"; return 1
        fi
        a=$(ldd "$cli" | sed -n 's/^[[:space:]]*libsigrok\.so[^ ]* => \([^ ]*\).*/\1/p')
        b=$(ldd "$TMP/libsigrok-run" | sed -n 's/^[[:space:]]*libsigrok\.so[^ ]* => \([^ ]*\).*/\1/p')
        if [ -z "$a" ] || [ "$(readlink -f "$a")" != "$(readlink -f "$b")" ]; then
            EMU_LIBSIGROK_ERR="the runner loads libsigrok '$b', sigrok-cli '$a'"; return 1
        fi
        EMU_LIBSIGROK_RUN=$TMP/libsigrok-run
    }
    # The name of tool T (a or b) of the case running, for messages.
    emu_tname() {
        if [ "$1" = b ]; then echo oxrok
        elif [ "$libsig" = 1 ]; then echo libsigrok-run
        else echo sigrok-cli; fi
    }

    # emu_case NAME [--keep-header] [--stderr] [--names] [--filter SED]
    #          [--digest] [--expect-fail] [--no-output] [--driver ID]
    #          [--oracle-bug WHY [--oracle-hangs]] [--oracle-differs WHY] [--unplug WHEN]
    #          [--interrupt WHEN] [--timeout S] [--hangs] [--record]
    #          [--libsigrok [--oxrok]] [--tx-filter SED] [--no-oracle-only] [--bound DRV] -- ARGS...
    #   The flags of case_, plus: --digest, the compared output ends
    #   with the SHA-256 of the whole output before --filter (a filter
    #   that keeps every 64th line of a frame still compares every
    #   byte); --expect-fail, the tools must exit
    #   with a failure; --no-output, the compared output must be empty
    #   (a --set prints nothing) rather than not empty; --driver, the
    #   driver the case uses when the model has several in EMU_DRIVER
    #   (default: the first); --oracle-bug, a full case that sigrok-cli
    #   cannot run because of the libsigrok bug WHY: sigrok-cli must
    #   fail (so a fix upstream shows up as a failing case, to be made
    #   an ordinary one), its stderr is kept in the golden SLUG.oracle,
    #   and oxrok alone is held to the .out and .tx goldens (runs of a
    #   repeated stderr line count once there, as a driver that loops
    #   on an error repeats its message as often as the time lets it).
    #   --oracle-differs, a full case where sigrok-cli exits 0 but is
    #   wrong because of the libsigrok bug WHY: sigrok-cli must exit 0
    #   and its output or transactions must differ from oxrok's; its
    #   stdout, prepared as the case says (never its stderr), is kept in
    #   the golden SLUG.oracle, oxrok is held to the .out and .tx
    #   goldens, and a sigrok-cli that matches oxrok fails the case
    #   ("oracle fixed?"). --expect-fail and --stderr then apply to
    #   oxrok alone (an oxrok that refuses what sigrok-cli runs).
    #   --unplug and --interrupt arm the emulator's event at WHEN
    #   (in=EP:N, out=EP:N or model; docs/USB.md) for each tool's run,
    #   which must fire it: the device leaves the bus and comes back
    #   after the run (a case that names the device by address or node
    #   gets the attach before the unplug), or the tool gets SIGINT.
    #   --timeout stops each tool after S seconds rather than
    #   EMU_CASE_TIMEOUT; a sigrok-cli under --oracle-bug that has to be
    #   stopped so fails as the case wants; with --oracle-hangs it must
    #   be stopped so, not fail any other way. --hangs, an oracle case
    #   where the C driver never ends: it passes when sigrok-cli is
    #   stopped by the timeout (and fails when it ends on its own), its
    #   transactions cut where they start repeating (emu_hang_cut), runs
    #   of a repeated output line counted once; sigrok-cli's stdout is
    #   lost with it, so such a case compares --stderr or --no-output.
    #   --no-oracle-only, a case that runs only under --no-oracle, for
    #   what sigrok-cli cannot run first (a flag it lacks, an address its
    #   upload would move); its goldens are written by hand, as a bless
    #   does not run it, and its slug counts in every mode, so that a bless
    #   keeps them.
    #   --tx-filter, a sed -E script over each tool's transaction log
    #   before it is compared, for an order the C driver's timing decides;
    #   the case says what it hides. --libsigrok, a library-verified
    #   oracle case: libsigrok-run
    #   (tools/libsigrok-run.c, sigrok-cli's syntax, no limit needed,
    #   --stop-after N its own sample count) runs in sigrok-cli's place
    #   and alone, whatever oxrok has, held to the case's goldens like
    #   any oracle case; it takes --hangs, --timeout, --expect-fail,
    #   events and --gcov, not --oracle-bug, --oracle-differs or
    #   --record, and --no-oracle skips it. --oxrok, with --libsigrok:
    #   a full case whose oracle is libsigrok-run, not sigrok-cli, for a
    #   driver sigrok-cli cannot drive once oxrok has it: same arguments,
    #   same goldens, and every flag of a full case.
    #   --record records oxrok's
    #   run of a full case with OXROK_USB_RECORD and OXROK_HID_RECORD
    #   (the USB one kept when both wrote) as the replay fixture
    #   tests/fixtures/DRIVER/emu-SLUG.txt, its stdout as emu-SLUG.out
    #   (DRIVER with - made _), which tests/emu_replay.rs replays through
    #   oxrok with OXROK_USB_REPLAY or OXROK_HID_REPLAY; like the goldens,
    #   EMU_BLESS=1 writes them and otherwise the recording must match
    #   them, so a recording that is not deterministic fails. The name
    #   neither starts nor ends with white space and holds no tab or
    #   newline: the emulator trims the
    #   command line it acknowledges and logs as the section name.
    emu_case() {
        name=$1; shift
        strip=1; which=stdout; names=0; filter_expr=""; expect=0; no_output=0; bug=""; digest=0
        event=""; case_timeout=$EMU_CASE_TIMEOUT; record=0; hangs=0; differs=""; libsig=0; oracle_hangs=0; libsig_full=0; tx_filter=""; no_oracle_only=0; bound=""
        driver=${EMU_DRIVER%% *}
        while [ $# -gt 0 ] && [ "$1" != "--" ]; do
            case "$1" in
                --keep-header) strip=0 ;;
                --stderr) which=stderr ;;
                --names) names=1 ;;
                --filter) filter_expr=$2; shift ;;
                --digest) digest=1 ;;
                --expect-fail) expect=1 ;;
                --no-output) no_output=1 ;;
                --driver) driver=$2; shift ;;
                --oracle-bug) bug=$2; shift ;;
                --oracle-hangs) oracle_hangs=1 ;;
                --oracle-differs) differs=$2; shift ;;
                --unplug) event="unplug $2"; shift ;;
                --interrupt) event="interrupt $2"; shift ;;
                --timeout) case_timeout=$2; shift ;;
                --record) record=1 ;;
                --hangs) hangs=1 ;;
                --libsigrok) libsig=1 ;;
                --oxrok) libsig_full=1 ;;
                --tx-filter) tx_filter=$2; shift ;;
                --no-oracle-only) no_oracle_only=1 ;;
                --bound) bound=$2; shift ;;
                *) emu_fail "$name: emu_case has no flag $1"; return ;;
            esac
            shift
        done
        [ $# -gt 0 ] && shift
        nargs=$#
        case "$name" in
            "" | [[:space:]]* | *[[:space:]] | *"$(printf '\t')"* | *"
"*)
                emu_fail "'$name': a case name is not empty, holds no tab or newline, and neither starts nor ends with white space"
                return ;;
        esac
        case "$name" in *"$filter"*) ;; *) return ;; esac
        slug=$(emu_slug "$name")
        case " $emu_slugs " in
            *" $slug "*) emu_fail "$name: another case of $EMU_MODEL has the file name $slug"; return ;;
        esac
        emu_slugs="$emu_slugs $slug"
        [ $no_oracle_only = 0 ] || [ $no_oracle = 1 ] || return
        case "$OXROK_DRIVERS" in *" $driver "*) mode=full; tools="a b" ;; *) mode=oracle; tools=a ;; esac
        # A library case is the runner's alone, whatever oxrok has,
        # unless it says --oxrok.
        label=$mode
        if [ $libsig = 1 ]; then
            if [ $libsig_full = 1 ]; then label="$mode, library"; else mode=oracle; tools=a; label=library; fi
        elif [ $libsig_full = 1 ]; then
            emu_fail "$name: --oxrok goes with --libsigrok"; return
        fi
        if [ "$EMU_DRY" = 1 ]; then
            # A case of a driver EMU_FULL lists counts as full even when
            # oxrok -L lacks it: the model then gets its session, where
            # the case fails as in a normal run instead of being skipped.
            case "$mode:$libsig: $EMU_FULL " in full:* | *:0:*" $driver "*) emu_dry_full=$((emu_dry_full + 1)) ;; *) emu_dry_oracle=$((emu_dry_oracle + 1)) ;; esac
            return
        fi
        why=""
        # A driver the scenario says oxrok has must not quietly fall
        # back to an oracle case.
        if [ $mode = oracle ] && { [ $libsig = 0 ] || [ $libsig_full = 1 ]; }; then
            case " $EMU_FULL " in *" $driver "*) why="; oxrok -L lacks $driver, which EMU_FULL lists" ;; esac
        fi
        # --no-oracle: oxrok alone, and no case without it.
        if [ $no_oracle = 1 ]; then
            if [ $mode = oracle ]; then
                if [ -n "$why" ]; then emu_fail "$name (oracle):${why#;} [$*]"; else skipped=$((skipped + 1)); fi
                return
            fi
            tools=b
        fi
        if [ $libsig = 1 ]; then
            if [ $libsig_full = 0 ] && { [ -n "$bug" ] || [ -n "$differs" ] || [ $record = 1 ]; }; then
                emu_fail "$name (library): --libsigrok without --oxrok takes neither --oracle-bug, --oracle-differs nor --record"; return
            fi
            # Only when it will run: --no-oracle has no sigrok to build on.
            if [ "$tools" != b ] && ! emu_libsigrok_run; then
                emu_fail "$name (library): no libsigrok runner: $EMU_LIBSIGROK_ERR [$*]"; return
            fi
        fi
        # oxrok's port must end where the C driver hangs: such a full
        # case is an --oracle-bug one.
        if [ $hangs = 1 ] && { [ $mode = full ] || [ -n "$bug" ]; }; then
            why="$why; --hangs takes an oracle case (a full case whose sigrok-cli hangs is an --oracle-bug one)"
        fi
        if [ -n "$differs" ]; then
            [ $mode = full ] || why="$why; --oracle-differs needs oxrok's $driver"
            if [ -n "$bug" ] || [ $hangs = 1 ] || [ $record = 1 ]; then
                why="$why; --oracle-differs takes neither --oracle-bug, --hangs nor --record"
            fi
        fi
        tty0=$EMU_TTY
        for t in $tools; do
            if [ $t = b ]; then tool=$OXROK; elif [ $libsig = 1 ]; then tool=$EMU_LIBSIGROK_RUN; else tool=$SIGROK_CLI; fi
            if ! emu_cmd reset >/dev/null; then
                emu_fail "$name: the emulator did not answer"; return
            fi
            # After the reset: a reset that applies new options
            # renumerates the device, and its drivers bind again.
            # So does the replug after sigrok-cli's run in an --unplug
            # case: libftdi's detach would have left them off for oxrok,
            # whose USB layer gives back on release what it detached
            # (ftdi_sio's probe would land among oxrok's transactions).
            replugged=0
            case "$t: $event" in "b: unplug"*) replugged=1 ;; esac
            if [ -z "$bound" ] && { [ $no_oracle = 1 ] || [ $replugged = 1 ]; } && ! emu_unbind "$name"; then
                emu_fail "$name: could not unbind the kernel drivers as sigrok-cli's run would have"; return
            fi
            if [ -n "$bound" ] && ! emu_bind "$name" "$bound"; then
                emu_fail "$name: could not bind $bound"; return
            fi
            if [ "$EMU_CDC" = 1 ] && ! emu_tty_restore "$t $name"; then
                emu_fail "$name: could not put the tty back"; return
            fi
            if [ "$EMU_CDC" = 1 ] && [ "$EMU_TTY" != "$tty0" ]; then
                case " $* " in
                    *"$tty0"[:\ ]*)
                        emu_fail "$name: the tty moved from $tty0 to $EMU_TTY after the scenario read it"; return ;;
                esac
            fi
            if ! emu_cmd "section $t $name" >/dev/null; then
                emu_fail "$name: the emulator did not answer"; return
            fi
            if [ -n "$event" ] && ! emu_cmd "arm $event" >/dev/null; then
                emu_fail "$name: the emulator refused 'arm $event'"; return
            fi
            evskip=$(wc -l <"$TMP/emu.out")
            # A tool that hangs on the model must not hold the lock for
            # ever; 124 is timeout's status. It runs in the background so
            # that a signal's trap runs at once, not when the tool ends.
            # A command run in the background starts with SIGINT ignored;
            # an interrupt case gives the tool the default that a
            # terminal's Ctrl-C meets.
            case "$event" in
                interrupt*) set -- env --default-signal=INT "$tool" "$@" ;;
                *) set -- "$tool" "$@" ;;
            esac
            if { [ $hangs = 1 ] || [ $oracle_hangs = 1 ]; } && [ $t = a ] && [ -n "$EMU_TERM_EXIT" ]; then
                set -- env LD_PRELOAD="$EMU_TERM_EXIT" "$@"
            fi
            if [ $record = 1 ] && [ $t = b ]; then
                rm -f "$TMP/rec.usb" "$TMP/rec.hid"
                set -- env OXROK_USB_RECORD="$TMP/rec.usb" OXROK_HID_RECORD="$TMP/rec.hid" "$@"
            fi
            # stdin is /dev/null, whatever the harness was launched with,
            # as sigrok-cli stops a capture at any key or at EOF. A case
            # that interrupts it gets a FIFO that stays quiet instead, or
            # a --continuous capture would be over before the signal.
            stdin=/dev/null
            case $event in
                interrupt*)
                    stdin=$TMP/quiet-stdin
                    [ -p "$stdin" ] || mkfifo "$stdin"
                    ;;
            esac
            timeout -k 5 "$case_timeout" "$@" <>"$stdin" >"$TMP/$t.out" 2>"$TMP/$t.err" 7>&- 8<&- &
            TOOLPID=$!
            shift $(($# - nargs))
            WATCHPID=""
            case "$event" in
                interrupt*) emu_interrupt "$evskip" "$TOOLPID" 7>&- 8<&- & WATCHPID=$! ;;
            esac
            wait "$TOOLPID"
            st=$?
            [ $t = b ] || a_st=$st
            TOOLPID=""
            if [ -n "$WATCHPID" ]; then kill "$WATCHPID" 2>/dev/null; wait "$WATCHPID" 2>/dev/null; fi
            if [ $st = 124 ] && { [ -z "$bug" ] || [ $t = b ]; } && [ $hangs = 0 ]; then
                why="$why; $(basename "$tool") timed out after $case_timeout s"
            fi
            if [ -n "$bug" ] && [ $t = a ]; then
                [ $st = 0 ] && why="$why; sigrok-cli no longer fails ($bug fixed?): make it an ordinary case"
                if [ $oracle_hangs = 1 ]; then
                    case $st in
                        124 | 137) ;;
                        *) why="$why; sigrok-cli exited $st before the timeout, where --oracle-hangs says it never ends" ;;
                    esac
                fi
            elif [ -n "$differs" ] && [ $t = a ]; then
                [ $st = 0 ] || why="$why; sigrok-cli exited $st, where --oracle-differs says it exits 0"
            elif [ $hangs = 1 ]; then
                # 137: timeout's KILL, 5 s after a TERM that did not end it.
                case $st in
                    124 | 137) ;;
                    *) why="$why; $(basename "$tool") exited $st before the timeout, where --hangs says it never ends" ;;
                esac
            else
                if [ $expect = 0 ] && [ $st != 0 ]; then why="$why; $(basename "$tool") exited $st"; fi
                if [ $expect = 1 ] && [ $st = 0 ]; then why="$why; $(basename "$tool") did not fail"; fi
            fi
            # Once the run's status is checked (emu_cmd sets st): a
            # renumeration the run caused is reported by the OK, so the
            # outputs name its address generically and the next case
            # line reads it in EMU_USB.
            if ! emu_cmd sync >/dev/null; then
                emu_fail "$name: the emulator did not answer"; return
            fi
            case "$event" in
                unplug*)
                    if ! emu_replug; then emu_fail "$name: the emulator did not replug the device"; return; fi ;;
            esac
            if [ -n "$event" ] && ! tail -n +$((evskip + 1)) "$TMP/emu.out" | grep -q -x "EVENT ${event%% *}"; then
                why="$why; the armed ${event%% *} never fired under $(basename "$tool")"
            fi
        done
        for t in $tools; do
            tname=$(emu_tname $t)
            emu_tx "$t $name" >"$TMP/$t.tx"
            v=$(grep -c '^!' "$TMP/$t.tx")
            emu_case_v=$((emu_case_v + v))
            if [ "$EMU_CDC" = 1 ]; then
                emu_tx_tty "$t" <"$TMP/$t.tx" >"$TMP/$t.n" && mv "$TMP/$t.n" "$TMP/$t.tx"
            fi
            if [ -n "$tx_filter" ]; then
                sed -E "$tx_filter" "$TMP/$t.tx" >"$TMP/$t.n" && mv "$TMP/$t.n" "$TMP/$t.tx"
            fi
            # The fold runs before the --hangs cut, so the cut sees folded
            # lines: a loop that repeats a digested transfer of one shape
            # folds into a single line whose count is however many rounds
            # ran before the timeout, and the period the cut looks for is
            # gone. Do not set EMU_TX_FOLD=1 on a model whose --hangs
            # cases loop over such a transfer.
            if [ "$EMU_TX_FOLD" = 1 ]; then
                emu_tx_fold <"$TMP/$t.tx" >"$TMP/$t.n" && mv "$TMP/$t.n" "$TMP/$t.tx"
            fi
            if [ $hangs = 1 ]; then
                emu_hang_cut <"$TMP/$t.tx" >"$TMP/$t.n" && mv "$TMP/$t.n" "$TMP/$t.tx"
            fi
            # The attach's node names and address made generic first, so
            # that --filter and --names see the text the goldens hold.
            for f in out err; do
                sed -E -f "$TMP/emu.sed" "$TMP/$t.$f" >"$TMP/$t.n" && mv "$TMP/$t.n" "$TMP/$t.$f"
            done
            # Under --oracle-differs sigrok-cli exits 0: its stdout is
            # what it got wrong, whatever --stderr says for oxrok.
            if [ -n "$differs" ] && [ $t = a ]; then
                which_case=$which; which=stdout; to_cmp $t; which=$which_case
            else
                to_cmp $t
            fi
            # The version line goes even with --keep-header, so goldens
            # do not change with the tools' versions.
            sed -E '1{/^(libsigrok|sigrok-cli|oxrok) /d;}' "$TMP/$t.cmp" >"$TMP/$t.n"
            mv "$TMP/$t.n" "$TMP/$t.cmp"
            # A hung driver repeats its message as often as time allows.
            if [ $hangs = 1 ]; then uniq "$TMP/$t.cmp" >"$TMP/$t.n" && mv "$TMP/$t.n" "$TMP/$t.cmp"; fi
            if [ "$v" != 0 ]; then
                why="$why; violations under $tname: $(grep -m1 '^!' "$TMP/$t.tx")"
            fi
            # Full or oracle, an empty output is a case that never got to
            # the device, unless --no-output asks for it.
            # (Under --oracle-differs an empty sigrok-cli output is what
            # the .oracle golden holds, and oxrok's must differ from it.)
            if { [ -n "$bug" ] || [ -n "$differs" ]; } && [ $t = a ]; then continue; fi
            if [ $no_output = 0 ] && ! [ -s "$TMP/$t.cmp" ]; then why="$why; no output from $tname"; fi
            if [ $no_output = 1 ] && [ -s "$TMP/$t.cmp" ]; then why="$why; output from $tname where none was expected"; fi
        done
        if [ $no_oracle = 1 ]; then
            [ $record = 1 ] && [ -z "$bug" ] && emu_record "$@"
            emu_no_oracle "$@"
            return
        fi
        if [ -n "$bug" ]; then
            emu_oracle_bug "$@"
            return
        fi
        if [ -n "$differs" ]; then
            emu_oracle_differs "$@"
            return
        fi
        [ $record = 1 ] && emu_record "$@"
        if [ $mode = full ]; then
            cmp -s "$TMP/a.cmp" "$TMP/b.cmp" || why="$why; outputs differ"
            cmp -s "$TMP/a.tx" "$TMP/b.tx" || why="$why; transactions differ"
        fi
        golden=$EMU_GOLDEN/$EMU_MODEL/$slug
        blessed=""
        if [ "$EMU_BLESS" = 1 ]; then
            mkdir -p "$EMU_GOLDEN/$EMU_MODEL"
            cp "$TMP/a.cmp" "$golden.out"; cp "$TMP/a.tx" "$golden.tx"; rm -f "$golden.oracle"
            blessed=" (golden written)"
        elif [ -f "$golden.out" ] && [ -f "$golden.tx" ]; then
            for t in $tools; do
                cmp -s "$golden.out" "$TMP/$t.cmp" || why="$why; $(emu_tname $t) output differs from $slug.out"
                cmp -s "$golden.tx" "$TMP/$t.tx" || why="$why; $(emu_tname $t) transactions differ from $slug.tx"
            done
        elif [ $mode = oracle ]; then
            why="$why; no golden $golden.out/.tx (EMU_BLESS=1 writes it)"
        fi
        if [ -z "$why" ]; then
            pass=$((pass + 1))
            echo "PASS $name ($label, $(wc -l <"$TMP/a.tx" | tr -d ' ') transactions)$blessed"
        else
            emu_fail "$name ($label):${why#;} [$*]"
            if [ $mode = full ] && ! cmp -s "$TMP/a.cmp" "$TMP/b.cmp"; then
                diff "$TMP/a.cmp" "$TMP/b.cmp" | head -${DIFF_LINES:-6}
            elif [ $mode = full ] && ! cmp -s "$TMP/a.tx" "$TMP/b.tx"; then
                diff "$TMP/a.tx" "$TMP/b.tx" | head -${DIFF_LINES:-6}
            elif [ -f "$golden.out" ] && ! cmp -s "$golden.out" "$TMP/a.cmp"; then
                diff "$golden.out" "$TMP/a.cmp" | head -${DIFF_LINES:-6}
            elif [ -f "$golden.tx" ] && ! cmp -s "$golden.tx" "$TMP/a.tx"; then
                diff "$golden.tx" "$TMP/a.tx" | head -${DIFF_LINES:-6}
            fi
        fi
    }

    # emu_case --record: oxrok's recording of the case (each session it
    # opened, in order), the attach's node names made generic, the bus
    # and address dropped from a USB device line and the timed out reads
    # from a HID recording (their number depends on the pace of the run;
    # a HID replay answers a timeout wherever the recording has no read
    # next), with the case name and one `# arg:` line per argument; and
    # oxrok's stdout, node names made generic and the version line
    # dropped (tests/emu_replay.rs drops it from the replay's alike).
    # Written under EMU_BLESS=1, else compared.
    emu_record() {
        if [ $mode != full ] || [ -n "$event" ] || [ $which != stdout ] || [ $expect = 1 ]; then
            why="$why; --record takes a full case that succeeds, without events or --stderr"
            return
        fi
        rec=""
        for k in usb hid; do
            if [ -s "$TMP/rec.$k" ]; then rec=$TMP/rec.$k; break; fi
        done
        if [ -z "$rec" ]; then why="$why; oxrok recorded no USB or HID session"; return; fi
        {
            sed -n 1p "$rec"
            echo "# emu $EMU_MODEL: $name"
            for a in "$@"; do printf '# arg: %s\n' "$a"; done | sed -E -f "$TMP/emu.sed"
            sed -E -f "$TMP/emu.sed" "$rec" | sed -E '1d; /^device /s/ bus=[0-9]+ addr=[0-9]+//' |
                if [ "$rec" = "$TMP/rec.hid" ]; then sed -E '/^read [0-9]+ -> -$/d'; else cat; fi
        } >"$TMP/rec.txt"
        sed -E '1{/^oxrok /d;}' "$TMP/b.out" >"$TMP/rec.out"
        fix=$EMU_FIXTURES/$(echo "$driver" | tr - _)/emu-$slug
        if [ "$EMU_BLESS" = 1 ]; then
            mkdir -p "$(dirname "$fix")"
            cp "$TMP/rec.txt" "$fix.txt"; cp "$TMP/rec.out" "$fix.out"
        elif [ -f "$fix.txt" ] && [ -f "$fix.out" ]; then
            if ! cmp -s "$fix.txt" "$TMP/rec.txt"; then
                why="$why; the recording differs from $(basename "$fix").txt"
                diff "$fix.txt" "$TMP/rec.txt" | head -${DIFF_LINES:-6}
            fi
            cmp -s "$fix.out" "$TMP/rec.out" || why="$why; oxrok output differs from $(basename "$fix").out"
        else
            why="$why; no fixture $fix.txt/.out (EMU_BLESS=1 records it)"
        fi
    }

    # The end of a full emu_case under --no-oracle, --oracle-bug and
    # --oracle-differs ones too: oxrok against the .out and .tx goldens.
    emu_no_oracle() {
        golden=$EMU_GOLDEN/$EMU_MODEL/$slug
        if [ -f "$golden.out" ] && [ -f "$golden.tx" ]; then
            cmp -s "$golden.out" "$TMP/b.cmp" || why="$why; oxrok output differs from $slug.out"
            cmp -s "$golden.tx" "$TMP/b.tx" || why="$why; oxrok transactions differ from $slug.tx"
        else
            why="$why; no golden $golden.out/.tx to hold oxrok to"
        fi
        if [ -z "$why" ]; then
            pass=$((pass + 1))
            echo "PASS $name (full, oxrok only: no oracle; $(wc -l <"$TMP/b.tx" | tr -d ' ') transactions)"
        else
            emu_fail "$name (full, no oracle):${why#;} [$*]"
            if [ -f "$golden.out" ] && ! cmp -s "$golden.out" "$TMP/b.cmp"; then
                diff "$golden.out" "$TMP/b.cmp" | head -${DIFF_LINES:-6}
            elif [ -f "$golden.tx" ] && ! cmp -s "$golden.tx" "$TMP/b.tx"; then
                diff "$golden.tx" "$TMP/b.tx" | head -${DIFF_LINES:-6}
            fi
        fi
    }

    # The end of an emu_case --oracle-bug: oxrok against the .out and .tx
    # goldens, sigrok-cli's failure against the .oracle golden.
    emu_oracle_bug() {
        [ $mode = full ] || why="$why; --oracle-bug needs oxrok's $driver"
        # A sigrok-cli the timeout stopped may have been in the middle
        # of a line (libsigrok writes its "sr: " first): a last line
        # without its newline, or that prefix alone, is not part of what
        # it failed with. The libusb line of a retried open of a
        # renumerated device comes at random (docs/USB.md).
        case $a_st in
            124 | 137)
                [ -z "$(tail -c 1 "$TMP/a.err")" ] || sed -i '$d' "$TMP/a.err"
                sed -i -E '${/^sr: *$/d;}' "$TMP/a.err" ;;
        esac
        sed -i '/LIBUSB_ERROR_ACCESS/d' "$TMP/a.err"
        # A crash: whether timeout(1) says "dumped core" depends on the
        # host's core pattern and limit. libsigrok may have written its
        # "sr: " first. The exit status has shown it failed.
        sed -i -E '/timeout: the monitored command dumped core$/d; ${/^sr: *$/d;}' "$TMP/a.err"
        # The core's line on a refused start, for a driver whose connection
        # id is the address that changes with each attach (greatfet).
        sed -i -E 's/( device )[0-9]+\.[0-9]+( acquisition\.)$/\1BUS.ADDR\2/' "$TMP/a.err"
        golden=$EMU_GOLDEN/$EMU_MODEL/$slug
        blessed=""
        if [ "$EMU_BLESS" = 1 ]; then
            mkdir -p "$EMU_GOLDEN/$EMU_MODEL"
            cp "$TMP/b.cmp" "$golden.out"; cp "$TMP/b.tx" "$golden.tx"; uniq "$TMP/a.err" >"$golden.oracle"
            blessed=" (golden written)"
        elif [ -f "$golden.out" ] && [ -f "$golden.tx" ] && [ -f "$golden.oracle" ]; then
            cmp -s "$golden.out" "$TMP/b.cmp" || why="$why; oxrok output differs from $slug.out"
            cmp -s "$golden.tx" "$TMP/b.tx" || why="$why; oxrok transactions differ from $slug.tx"
            uniq "$TMP/a.err" >"$TMP/a.uniq"
            cmp -s "$golden.oracle" "$TMP/a.uniq" || why="$why; sigrok-cli fails otherwise than $slug.oracle"
        else
            why="$why; no golden $golden.out/.tx/.oracle (EMU_BLESS=1 writes it)"
        fi
        if [ -z "$why" ]; then
            pass=$((pass + 1))
            echo "PASS $name (oxrok only, sigrok-cli fails: $bug; $(wc -l <"$TMP/b.tx" | tr -d ' ') transactions)$blessed"
        else
            emu_fail "$name (oracle bug):${why#;} [$*]"
            if [ -f "$golden.oracle" ] && ! cmp -s "$golden.oracle" "$TMP/a.uniq"; then
                diff "$golden.oracle" "$TMP/a.uniq" | head -${DIFF_LINES:-6}
            elif [ -f "$golden.out" ] && ! cmp -s "$golden.out" "$TMP/b.cmp"; then
                diff "$golden.out" "$TMP/b.cmp" | head -${DIFF_LINES:-6}
            fi
        fi
    }

    # The end of an emu_case --oracle-differs: oxrok against the .out and
    # .tx goldens, sigrok-cli's output against the .oracle golden, and
    # the two tools must still differ.
    emu_oracle_differs() {
        if cmp -s "$TMP/a.cmp" "$TMP/b.cmp" && cmp -s "$TMP/a.tx" "$TMP/b.tx"; then
            why="$why; sigrok-cli now matches oxrok (oracle fixed? $differs): make it an ordinary case"
        fi
        golden=$EMU_GOLDEN/$EMU_MODEL/$slug
        blessed=""
        if [ "$EMU_BLESS" = 1 ]; then
            mkdir -p "$EMU_GOLDEN/$EMU_MODEL"
            cp "$TMP/b.cmp" "$golden.out"; cp "$TMP/b.tx" "$golden.tx"; cp "$TMP/a.cmp" "$golden.oracle"
            blessed=" (golden written)"
        elif [ -f "$golden.out" ] && [ -f "$golden.tx" ] && [ -f "$golden.oracle" ]; then
            cmp -s "$golden.out" "$TMP/b.cmp" || why="$why; oxrok output differs from $slug.out"
            cmp -s "$golden.tx" "$TMP/b.tx" || why="$why; oxrok transactions differ from $slug.tx"
            cmp -s "$golden.oracle" "$TMP/a.cmp" || why="$why; sigrok-cli output differs from $slug.oracle"
        else
            why="$why; no golden $golden.out/.tx/.oracle (EMU_BLESS=1 writes it)"
        fi
        if [ -z "$why" ]; then
            pass=$((pass + 1))
            echo "PASS $name (oxrok pinned, sigrok-cli differs: $differs; $(wc -l <"$TMP/b.tx" | tr -d ' ') transactions)$blessed"
        else
            emu_fail "$name (oracle differs):${why#;} [$*]"
            if [ -f "$golden.oracle" ] && ! cmp -s "$golden.oracle" "$TMP/a.cmp"; then
                diff "$golden.oracle" "$TMP/a.cmp" | head -${DIFF_LINES:-6}
            elif [ -f "$golden.out" ] && ! cmp -s "$golden.out" "$TMP/b.cmp"; then
                diff "$golden.out" "$TMP/b.cmp" | head -${DIFF_LINES:-6}
            fi
        fi
    }

    # One model: lock, clean up, attach, cases, detach, unlock.
    emu_session() {
        [ -e "$EMU_LOCK" ] || (umask 0; : >>"$EMU_LOCK")
        if ! exec 8<"$EMU_LOCK"; then emu_fail "$EMU_MODEL: cannot open $EMU_LOCK"; return; fi
        if ! flock -n 8; then
            echo "waiting for $EMU_LOCK (another emulator session)"
            if ! flock -w "$EMU_LOCK_WAIT" 8; then
                exec 8<&-
                emu_fail "$EMU_MODEL: $EMU_LOCK still held after $EMU_LOCK_WAIT s"; return
            fi
        fi
        # A run that crashed may have left its device attached.
        sudo -n "$EMU" cleanup --locked || echo "emulator cleanup failed; carrying on with a free port"
        if [ $gcov = 1 ]; then
            emu_gcov zero || emu_fail "$EMU_MODEL: emu-gcov.sh zero failed"
        fi
        emu_slugs=""
        emu_case_v=0
        if emu_start; then
            emu_cases
            emu_stop
            st=$?
            [ $st = 0 ] || [ $st = 2 ] || emu_fail "$EMU_MODEL session: the emulator exited with $st"
            # Violations no case reported: during the attach or the
            # detach, or logged after a case had read its section.
            total=$(grep -c '^!' "$EMULOG/$EMU_MODEL.log")
            n=$((total - emu_case_v))
            if [ "$n" != 0 ]; then
                emu_fail "$EMU_MODEL session: $n violations outside what the cases checked"
                awk '/^# section / { s = substr($0, 11) } /^!/ { print "  [" s "] " $0 }' "$EMULOG/$EMU_MODEL.log" | head -5
            fi
        else
            emu_fail "$EMU_MODEL: the emulator did not start"
            emu_stop >/dev/null 2>&1
        fi
        if [ $gcov = 1 ]; then
            if emu_gcov report >"$TMP/gcov.txt"; then
                if [ -z "$filter" ]; then
                    mkdir -p "$EMU_GOLDEN/$EMU_MODEL"
                    cp "$TMP/gcov.txt" "$EMU_GOLDEN/$EMU_MODEL/gcov.txt"
                    echo "gcov: $EMU_GOLDEN/$EMU_MODEL/gcov.txt: $(grep '^  total' "$TMP/gcov.txt" | tr -s ' ' | tr '\n' ';')"
                else
                    # A subset of the cases says nothing about the whole.
                    cat "$TMP/gcov.txt"
                fi
            else
                emu_fail "$EMU_MODEL: emu-gcov.sh report failed; gcov.txt left as it was"
            fi
        fi
        exec 8<&-
        # Goldens no case wrote, after a full run of the model.
        if [ -z "$filter" ] && [ -d "$EMU_GOLDEN/$EMU_MODEL" ]; then
            for g in "$EMU_GOLDEN/$EMU_MODEL"/*.out; do
                [ -f "$g" ] || continue
                slug=$(basename "$g" .out)
                case " $emu_slugs " in *" $slug "*) continue ;; esac
                if [ "$EMU_BLESS" = 1 ]; then
                    rm -f "$g" "${g%.out}.tx" "${g%.out}.oracle"; echo "removed golden $slug of $EMU_MODEL: no case has it"
                else
                    echo "note: golden $slug of $EMU_MODEL belongs to no case (EMU_BLESS=1 removes it)"
                fi
            done
        fi
    }

    # emu_gcov zero|report: tools/emu-gcov.sh on each of the model's
    # libsigrok driver directories, then on the shared sources it names
    # (EMU_GCOV_SHARED); fails when one of them fails.
    emu_gcov() {
        gst=0
        for d in ${EMU_GCOV:-$EMU_DRIVER}; do sh "$TOOLS/emu-gcov.sh" "$d" "$1" || gst=1; done
        if [ -n "$EMU_GCOV_SHARED" ]; then
            # shellcheck disable=SC2086
            sh "$TOOLS/emu-gcov.sh" --shared "$1" $EMU_GCOV_SHARED || gst=1
        fi
        return $gst
    }

    found=""
    for f in "$TOOLS"/emu/*.sh; do
        [ -f "$f" ] || continue
        EMU_MODEL=""; EMU_OPTIONS=""; EMU_DRIVER=""; EMU_GCOV=""; EMU_GCOV_SHARED=""; EMU_FULL=""; EMU_TX_FOLD=""; EMU_CDC=""
        emu_cases() { emu_fail "$f: no emu_cases function"; }
        . "$f"
        [ -n "$EMU_MODEL" ] || { emu_fail "$f: no EMU_MODEL"; continue; }
        if [ -n "$emu_models" ]; then
            case " $emu_models " in *" $EMU_MODEL "*) ;; *) continue ;; esac
        fi
        found="$found $EMU_MODEL"
        # --no-oracle: a model without a full case is not attached; its
        # cases are counted by running the scenario with emu_case and
        # emu_cmd doing nothing else (in a subshell, no device named).
        if [ $no_oracle = 1 ]; then
            dry=$(
                set +u
                EMU_DRY=1; emu_dry_full=0; emu_dry_oracle=0; emu_slugs=""
                EMU_PORT=""; EMU_BUSID=""; EMU_USB=""; EMU_USBFS=""; EMU_NODES=""
                EMU_HIDRAW=""; EMU_TTY=""; EMU_USBTMC=""
                emu_cases >/dev/null 2>&1 7>&- 8<&-
                echo "$emu_dry_full $emu_dry_oracle"
            )
            case "$dry" in
                [0-9]*" "[0-9]*) ;;
                *) emu_fail "$EMU_MODEL: counting its cases failed ('$dry'); running its session"; dry="1 0" ;;
            esac
            if [ "${dry%% *}" = 0 ]; then
                skipped=$((skipped + ${dry#* }))
                echo "SKIP $EMU_MODEL: ${dry#* } oracle-only cases (oxrok has no ${EMU_DRIVER%% *} driver)"
                continue
            fi
        fi
        emu_session
    done
    for m in $emu_models; do
        case " $found " in *" $m "*) ;; *) emu_fail "--emu-model $m: no tools/emu/*.sh sets EMU_MODEL=$m" ;; esac
    done
fi

if [ $no_oracle = 1 ]; then
    # Nothing compared is no evidence: a run that skipped every case fails.
    [ $((pass + fail)) != 0 ] || emu_fail "--no-oracle: no case ran ($skipped skipped)"
    echo "$pass passed, $fail failed: $((pass + fail)) ran, $skipped skipped (oracle-only, --no-oracle)"
else
    echo "$pass passed, $fail failed"
fi
[ $fail = 0 ]
