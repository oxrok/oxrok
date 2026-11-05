# A Devantech relay board's commands (emu/src/models/devantech-eth008.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model". The real boards are reached over TCP, which no shell carries:
# libsigrok's serial layer takes either transport, and this is the serial one.
EMU_MODEL=devantech-eth008
EMU_CDC=1
EMU_DRIVER=devantech-eth008
EMU_FULL=$EMU_DRIVER
EMU_GCOV=devantech-eth008
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# Three things for sigrok-cli only, from a preload that a wrapper sets on its exec,
# for the length of emu_cases. The C driver is unmodified.
#
# - The scan's open. api.c:105 opens with no flags, serial_libsp.c:67-71 gives
#   sp_open a mode of 0, which opens read only, and the first write fails with
#   EBADF (the "stock" cases show it). So on a tty this driver never finds a board,
#   and the comment at protocol.c:93-94 doesn't hold. serial_tcpraw.c ignores the
#   flags, and over TCP, the boards' own transport, every path works. The cases
#   below compare those paths: ETH_RUN makes that mode 0 read and write.
# - Locks. api.c:191 returns from the probe with the port open, std_serial_dev_open
#   opens it again, and libserialport's exclusive open would refuse that (as for
#   gwinstek-gpd). Over TCP it is one more connection. ETH_RUN makes flock() and the
#   one ioctl TIOCEXCL no-ops.
# - A floor on the read. READ_TIMEOUT_MS is 20 (protocol.c:135), less than the host
#   can hold up a transfer for, and such a timeout on a scan shows only as nothing
#   found. So no sp_blocking_read waits under 250 ms. No golden depends on the
#   length: a silent board's cases end the same, later. Over TCP this timeout has
#   no effect at all.
eth_oracle() {
    ETH_STOCK_CLI=$SIGROK_CLI
    [ "$no_oracle" = 1 ] && return
    printf '%s\n' '#define _GNU_SOURCE' '#include <dlfcn.h>' '#include <stdarg.h>' \
        '#include <stddef.h>' '#include <stdlib.h>' '#include <sys/file.h>' '#include <sys/ioctl.h>' \
        '#ifdef ETH_RUN_ALWAYS' \
        '__attribute__((constructor)) static void eth_run(void) { setenv("ETH_RUN", "1", 1); }' \
        '#endif' \
        'int flock(int fd, int op) {' \
        '    static int (*real)(int, int);' \
        '    if (getenv("ETH_RUN")) return 0;' \
        '    if (!real) real = (int (*)(int, int))dlsym(RTLD_NEXT, "flock");' \
        '    return real(fd, op);' \
        '}' \
        'int ioctl(int fd, unsigned long req, ...) {' \
        '    static int (*real)(int, unsigned long, void *);' \
        '    va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);' \
        '    if (req == TIOCEXCL && getenv("ETH_RUN")) return 0;' \
        '    if (!real) real = (int (*)(int, unsigned long, void *))dlsym(RTLD_NEXT, "ioctl");' \
        '    return real(fd, req, arg);' \
        '}' \
        'int sp_open(void *port, int mode) {' \
        '    static int (*real)(void *, int);' \
        '    if (!real) real = (int (*)(void *, int))dlsym(RTLD_NEXT, "sp_open");' \
        '    return real(port, mode == 0 && getenv("ETH_RUN") ? 3 : mode);' \
        '}' \
        'int sp_blocking_read(void *port, void *buf, size_t count, unsigned int timeout_ms) {' \
        '    static int (*real)(void *, void *, size_t, unsigned int);' \
        '    if (!real) real = (int (*)(void *, void *, size_t, unsigned int))dlsym(RTLD_NEXT, "sp_blocking_read");' \
        '    return real(port, buf, count, timeout_ms != 0 && timeout_ms < 250 ? 250 : timeout_ms);' \
        '}' >"$TMP/eth-oracle.c"
    cc -shared -fPIC -o "$TMP/eth-oracle.so" "$TMP/eth-oracle.c" -ldl 2>"$TMP/eth-oracle.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    cc -shared -fPIC -DETH_RUN_ALWAYS -o "$TMP/eth-run.so" "$TMP/eth-oracle.c" -ldl 2>"$TMP/eth-oracle.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    printf '%s\n' '#!/bin/sh' \
        "LD_PRELOAD=$TMP/eth-run.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$SIGROK_CLI") \"\$@\"" \
        >"$TMP/eth-sigrok-cli"
    printf '%s\n' '#!/bin/sh' \
        "LD_PRELOAD=$TMP/eth-oracle.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$SIGROK_CLI") \"\$@\"" \
        >"$TMP/eth-stock-cli"
    chmod +x "$TMP/eth-sigrok-cli" "$TMP/eth-stock-cli"
    SIGROK_CLI=$TMP/eth-sigrok-cli
}

emu_cases() {
    eth_oracle
    emu_conns
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: err: oxrok::drivers::devantech_eth008(::protocol)?: /sr: devantech-eth008: /'
    SHOW_BUG="api.c:279 gives SR_CONF_VOLTAGE a uint32 where hwdriver.c:141 has it SR_T_FLOAT, which show.c:750 reads with g_variant_get_double: a GLib critical, on which sigrok-cli's logger exits (main.c:46-47); oxrok shows the number"
    STOCK_BUG="api.c:105 opens with no flags, which serial_libsp.c:67-71 gives sp_open as a mode of 0, so read only, and the scan's first write fails with EBADF: on a tty stock sigrok-cli finds no board, with any libserialport; oxrok opens for both"
    SHIFT_BUG="recv_response reads what it asks for (protocol.c:191) and nothing clears the rest, so a byte too many is the head of the next answer: here the serial number reads one byte along, and so does every later answer of a session; oxrok drops the input queue before a request"
    REFUSED_BUG="sigrok-cli sets -c at session.c:804 before the limit it cannot set (:866), so every relay goes on for a capture it then refuses; oxrok holds enabled=on until the start (docs/CLI.md) and sends nothing"
    LOOSE_BUG="sigrok-cli takes what is no boolean as off (session.c:566, sr_parse_boolstring gives false) and turns every relay off; oxrok refuses it and sends nothing"
    SERIALCOMM_BUG="hwdriver.c's check_options logs the unlisted option and scan goes on with nothing, so sigrok-cli exits 0 finding nothing; oxrok refuses it by its name and exits 1"

    # Each line of api.c's models[]: its name, and that a scan reads the module
    # info and the serial number, and sets nothing.
    for id in 18 19 20 21 22 24 26 28 52; do
        emu_options setters=0 board=$id
        emu_case "scan board $id" --keep-header -- -d $K --scan
    done
    emu_options setters=0 hw=0 fw=255 mac=ffeeddccbbaa
    emu_case "scan version and serial corners" --keep-header -- -d $K --scan
    emu_options setters=0 board=99
    emu_case "scan unknown board" --stderr --filter "$L" -- -d $K --scan
    emu_options setters=0
    emu_case "no conn" --keep-header --no-output -- -d devantech-eth008 --scan
    emu_case "no such node" --keep-header --no-output -- -d devantech-eth008:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm is no scan option" --oracle-differs "$SERIALCOMM_BUG" --expect-fail --stderr -- -d $K:serialcomm=9600/8n1 --scan

    # A probe with a wrong answer. Each is one request with no second try.
    for bad in module=~ module=1302 serial=~ serial=0004a3 serial=0004a31234; do
        emu_options setters=0 "$bad"
        emu_case "scan $bad" --keep-header --no-output -- -d $K --scan
    done
    # A byte too many stays for the next read.
    emu_options setters=0 module=130203ff
    emu_case "scan module info too long" --oracle-differs "$SHIFT_BUG" --keep-header -- -d $K --scan
    emu_options setters=0 serial=0004a3123456ff
    emu_case "scan serial number too long" --keep-header -- -d $K --scan

    # --show: the groups each board has, and what a group lists.
    for id in 18 19 20 21 52; do
        emu_options setters=0 board=$id
        emu_case "show board $id" --keep-header -- -d $K --show
    done
    emu_options setters=0 board=20 do=0x0001 di=0x0002 ai=513
    emu_case "show DO" --keep-header -- -d $K -g DO1 --show
    emu_case "show DI" --keep-header -- -d $K -g DI2 --show
    emu_case "show Vsupply" --keep-header -- -d $K -g Vsupply --show
    emu_case "show AI" --oracle-bug "$SHOW_BUG" --keep-header -- -d $K -g AI1 --show
    emu_case "show no such group" --keep-header --stderr --expect-fail -- -d $K -g DO5 --show
    emu_options setters=0 board=20 outputs=~
    emu_case "show DO with no answer" --keep-header -- -d $K -g DO1 --show

    # --get: each kind of group, at the corners of each image.
    emu_options setters=0
    emu_case "get conn" -- -d $K --get conn
    emu_options setters=0 do=0x81
    for io in DO1 DO2 DO8; do emu_case "get $io of an ETH008" -- -d $K -g $io --get enabled; done
    emu_options setters=0 board=20 do=0x8001 di=0x0081
    for io in DO1 DO9 DO16 DI1 DI2 DI8; do emu_case "get $io of an ETH484" -- -d $K -g $io --get enabled; done
    emu_options setters=0 board=21 do=0x080100 di=0xffffff80
    for io in DO1 DO9 DO20 DI1 DI8; do emu_case "get $io of an ETH8020" -- -d $K -g $io --get enabled; done
    emu_options setters=0 board=52 do=0x0200 di=0x8000
    for io in DO10 DI1 DI16; do emu_case "get $io of an ETH1610" -- -d $K -g $io --get enabled; done
    # Bits with no channel, and a relay the board doesn't have.
    emu_options setters=0 board=18 do=0xfe
    emu_case "get outputs beyond the mask" -- -d $K -g DO1 --get enabled
    emu_options setters=0 board=20 ai=0,513,1023,65535
    for io in AI1 AI2 AI3 AI4; do emu_case "get $io" -- -d $K -g $io --get voltage; done
    emu_options setters=0 board=21 ai=0,0,0,0,0,0,0,4095
    emu_case "get AI8 of an ETH8020" -- -d $K -g AI8 --get voltage
    emu_options setters=0 board=52 ai=0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4095
    emu_case "get AI16 of an ETH1610" -- -d $K -g AI16 --get voltage
    for tenths in 0 125 123 255; do
        emu_options setters=0 volts=$tenths
        emu_case "get supply $tenths" -- -d $K -g Vsupply --get voltage
    done

    # What a group doesn't list, which sigrok-cli's check turns down.
    emu_options setters=0 board=20
    emu_case "get enabled with no group" --stderr --expect-fail -- -d $K --get enabled
    emu_case "get voltage with no group" --stderr --expect-fail -- -d $K --get voltage
    emu_case "get voltage of DO" --stderr --expect-fail -- -d $K -g DO1 --get voltage
    emu_case "get enabled of AI" --stderr --expect-fail -- -d $K -g AI1 --get enabled
    emu_case "get enabled of Vsupply" --stderr --expect-fail -- -d $K -g Vsupply --get enabled
    emu_case "get conn of DO" --stderr --expect-fail -- -d $K -g DO1 --get conn
    emu_case "get no such group" --stderr --expect-fail -- -d $K -g DO5 --get enabled

    # A get whose answer fails. Getting a relay reads the inputs too, and the two
    # fail the same way.
    emu_options setters=0 outputs=~
    emu_case "get DO silent" --stderr --expect-fail -- -d $K -g DO1 --get enabled
    emu_options setters=0 board=20 outputs=00
    emu_case "get DO short" --stderr --expect-fail -- -d $K -g DO1 --get enabled
    emu_options setters=0 board=20 inputs=~
    emu_case "get DO inputs silent" --stderr --expect-fail -- -d $K -g DO1 --get enabled
    emu_case "get DI inputs silent" --stderr --expect-fail -- -d $K -g DI1 --get enabled
    emu_options setters=0 board=20 outputs=~
    emu_case "get DI outputs silent" --stderr --expect-fail -- -d $K -g DI1 --get enabled
    emu_options setters=0 board=21 inputs=000000
    emu_case "get DI short" --stderr --expect-fail -- -d $K -g DI1 --get enabled
    emu_options setters=0 board=20 analog=~
    emu_case "get AI silent" --stderr --expect-fail -- -d $K -g AI1 --get voltage
    emu_options setters=0 board=20 analog=01
    emu_case "get AI short" --stderr --expect-fail -- -d $K -g AI1 --get voltage
    emu_options setters=0 supply=~
    emu_case "get supply silent" --stderr --expect-fail -- -d $K -g Vsupply --get voltage

    # --set: all of them at each width, one by its number.
    for id in 18 19 20 21 52; do
        emu_options board=$id
        emu_case "set all on board $id" --no-output -- -d $K -c enabled=on --set
    done
    emu_options board=21
    emu_case "set all off" --no-output -- -d $K -c enabled=off --set
    emu_options board=20
    emu_case "set DO1 on" --no-output -- -d $K -g DO1 -c enabled=on --set
    emu_case "set DO4 off" --no-output -- -d $K -g DO4 -c enabled=off --set
    emu_case "set DO9 on" --no-output -- -d $K -g DO9 -c enabled=on --set
    emu_case "set DO16 off" --no-output -- -d $K -g DO16 -c enabled=off --set
    emu_options board=21
    emu_case "set DO20 on" --no-output -- -d $K -g DO20 -c enabled=on --set

    # A set that fails, for one relay and for all.
    for scope in all DO1; do
        where=
        [ "$scope" = all ] || where="-g $scope"
        emu_options set=01
        emu_case "set $scope refused" --stderr --expect-fail -- -d $K $where -c enabled=on --set
        emu_options set=~
        emu_case "set $scope silent" --stderr --expect-fail -- -d $K $where -c enabled=on --set
        emu_options set=ff
        emu_case "set $scope other answer" --stderr --expect-fail -- -d $K $where -c enabled=on --set
    done
    emu_options set=0001
    emu_case "set answer too long" --no-output -- -d $K -c enabled=on --set

    # What can't be set.
    emu_options board=20
    emu_case "set DI" --stderr --expect-fail -- -d $K -g DI1 -c enabled=on --set
    emu_case "set AI" --stderr --expect-fail -- -d $K -g AI1 -c voltage=1 --set
    emu_case "set Vsupply" --stderr --expect-fail -- -d $K -g Vsupply -c voltage=1 --set
    emu_case "set voltage with no group" --stderr --expect-fail -- -d $K -c voltage=1 --set
    emu_case "set no such key" --stderr --expect-fail -- -d $K -c nonesuch=1 --set
    emu_case "set not a boolean" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr -- -d $K -c enabled=perhaps --set
    emu_case "set no such group" --stderr --expect-fail -- -d $K -g DO5 -c enabled=on --set
    emu_case "set conn" --stderr --expect-fail -- -d $K -c conn=x --set
    emu_case "set with nothing" --stderr --expect-fail -- -d $K --set
    emu_case "set with no value" --no-output -- -d $K -c enabled --set
    emu_case "set in a refused capture" --oracle-bug "$REFUSED_BUG" --expect-fail --stderr -- -d $K -c enabled=on --samples 1

    # C's hexdumps of each request and answer, at the level that prints them.
    emu_options setters=0
    emu_case "get at spew level" -- -l 5 -d $K -g DO1 --get enabled

    # There is no capture, and nothing to limit one by.
    emu_options setters=0
    emu_case "samples" --stderr --expect-fail -- -d $K --samples 1
    emu_case "time" --stderr --expect-fail -- -d $K --time 10
    emu_case "frames" --stderr --expect-fail -- -d $K --frames 1
    emu_case "continuous" --stderr --expect-fail -- -d $K --continuous
    # What a session makes of a device with no channels. The runner finds its
    # libsigrok through the real sigrok-cli, and has the preload through the env.
    SIGROK_CLI=$ETH_STOCK_CLI
    emu_case "lib capture has no enabled channels" --libsigrok --oracle-env "LD_PRELOAD=$TMP/eth-run.so" --expect-fail --stderr -- -d $K --continuous
    [ "$no_oracle" = 1 ] || SIGROK_CLI=$TMP/eth-sigrok-cli

    # The device leaves in a get's answer.
    emu_options setters=0 answers=2
    emu_case "unplug in a get" --expect-fail --no-output --unplug model --timeout 20 --tx-filter '/^unplug$/q' -- -d $K -g DO1 --get enabled
    emu_case "unplug in a set" --expect-fail --no-output --unplug model --timeout 20 --tx-filter '/^unplug$/q' -- -d $K -g DO1 -c enabled=on --set

    # As the oracle really is, with the exclusive open.
    SIGROK_CLI=$TMP/eth-stock-cli
    [ "$no_oracle" = 1 ] && SIGROK_CLI=$ETH_STOCK_CLI
    emu_options setters=0
    emu_case "stock show" --oracle-bug "$STOCK_BUG" --keep-header -- -d $K --show
    emu_case "stock get" --oracle-bug "$STOCK_BUG" -- -d $K -g DO1 --get enabled
    emu_options
    emu_case "stock set" --oracle-bug "$STOCK_BUG" --no-output -- -d $K -c enabled=on --set
    SIGROK_CLI=$ETH_STOCK_CLI
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=devantech-eth008:conn=$EMU_TTY
}
