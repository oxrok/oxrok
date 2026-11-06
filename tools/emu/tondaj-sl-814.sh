# A Tondaj SL-814 sound level meter (emu/src/models/tondaj-sl-814.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model".
EMU_MODEL=tondaj-sl-814
EMU_CDC=1
EMU_DRIVER=tondaj-sl-814
EMU_GCOV=tondaj-sl-814
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# api.c's scan returns with the port open (no serial_close before :96), and
# dev_open opens it again, which libserialport's exclusive open refuses
# (serialport.c:589 flock, :599 TIOCEXCL, first released in libserialport 0.1.2,
# 2024-09-10): from that version, which this oracle builds, stock sigrok-cli runs
# nothing but --scan with this driver. On 0.1.1 it works, and users there reach
# the paths the cases compare. So every case below but the "stock" ones runs
# sigrok-cli with locking disabled: a preload makes flock() and the one ioctl
# TIOCEXCL no-ops, and the C driver itself is unmodified. It is set on the exec of
# this model's sigrok-cli only, by a wrapper that SIGROK_CLI names for the length
# of emu_cases, and neither oxrok nor the harness runs under it. It changes the
# lock and nothing of the timing.
tondaj_openable() {
    TONDAJ_STOCK_CLI=$SIGROK_CLI
    [ "$no_oracle" = 1 ] && return
    printf '%s\n' '#define _GNU_SOURCE' '#include <dlfcn.h>' '#include <stdarg.h>' \
        '#include <sys/ioctl.h>' \
        'int flock(int fd, int op) { (void)fd; (void)op; return 0; }' \
        'int ioctl(int fd, unsigned long req, ...) {' \
        '    static int (*real)(int, unsigned long, void *);' \
        '    va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);' \
        '    if (req == TIOCEXCL) return 0;' \
        '    if (!real) real = (int (*)(int, unsigned long, void *))dlsym(RTLD_NEXT, "ioctl");' \
        '    return real(fd, req, arg);' \
        '}' >"$TMP/tondaj-openable.c"
    cc -shared -fPIC -o "$TMP/tondaj-openable.so" "$TMP/tondaj-openable.c" -ldl 2>"$TMP/tondaj-openable.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    printf '%s\n' '#!/bin/sh' \
        "LD_PRELOAD=$TMP/tondaj-openable.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$SIGROK_CLI") \"\$@\"" \
        >"$TMP/tondaj-sigrok-cli"
    chmod +x "$TMP/tondaj-sigrok-cli"
    SIGROK_CLI=$TMP/tondaj-sigrok-cli
}

emu_cases() {
    tondaj_openable
    emu_conns
    # The first request, for a .tx cut there.
    Q1='/^bulk_out 02 30000d/q'
    # A reply's frame with the sequence for 0.
    OK=1234010d
    # oxrok's log prefix, on its side only, and its own wording for a 0.
    L='s/^oxrok: (err|warn): oxrok::drivers::tondaj_sl_814(::protocol)?: /sr: tondaj-sl-814: /'
    A='s/^(Failed to set device option .limit_samples.: invalid argument): it cannot be 0\.$/\1./'

    # The scan says nothing on the wire and takes no answer: a port that opens is
    # a meter.
    emu_options
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=9600/8e1 --scan
    emu_case "no such node" --keep-header --no-output -- -d tondaj-sl-814:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d tondaj-sl-814 --scan
    emu_options reply=
    emu_case "scan of a silent meter" --keep-header -- -d $K --scan
    emu_options

    emu_case "show" --keep-header -- -d $K --show
    for key in limit_samples limit_time conn continuous samplerate; do
        emu_case "get $key" --expect-fail --stderr -- -d $K --get $key
    done
    emu_case "set limit_samples" --no-output -- -d $K -c limit_samples=2 --set
    emu_case "set limit_time" --no-output -- -d $K -c limit_time=100 --set
    emu_case "set limit_samples 0" --stderr --filter "$A" --expect-fail -- -d $K -c limit_samples=0 --set
    emu_case "set continuous" --stderr --filter "$L" --expect-fail -- -d $K -c continuous=1 --set
    emu_case "set samplerate" --stderr --filter "$L" --expect-fail -- -d $K -c samplerate=1 --set
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses them
    # first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan

    # A capture: init at the source's first 500 ms, its reply at once, a request
    # 500 ms on, and each next one 500 ms after the last reply.
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2 analog" -- -d $K --samples 2 -O analog
    emu_case "samples csv" -- -d $K --samples 2 -O csv:header=false:time=false
    emu_case "config limit_samples" -- -d $K -c limit_samples=2 --samples 1
    emu_case "channel P1" -- -d $K -C P1 --samples 1
    # Past the first limit check, each tool has only the init on the wire.
    emu_case "time 1 ms" --no-output -- -d $K --time 1ms
    emu_case "time up to the first request" --no-output --tx-filter "$Q1" -- -d $K --time 700ms
    # The line reaches the meter: the first two tell the rate and the parity.
    emu_options uart=2400/8e1
    emu_case "serialcomm reaches the line" -- -d $K:serialcomm=2400/8e1 --samples 1

    # The wiki's readings, then each bit: weight, speed, the level that is dropped
    # and the unknown bit, and the two ends of the value.
    emu_options data=09af,89cb,8a6c,8299,9285,a393,b3f2,c000,47ff,7fff,ffff
    emu_case "readings" -- -d $K --samples 11 -O analog

    # In pieces, and with the reply cut across the reads.
    emu_options chunk=1
    emu_case "one byte per transfer" -- -d $K --samples 2
    emu_options cut=2:5
    emu_case "reply in two pieces" -- -d $K --samples 2
    emu_options cut=1:5
    emu_case "init reply in two pieces" -- -d $K --samples 1
    # A slow meter: both wait for as long as it takes.
    emu_options late=1:100
    emu_case "a reply late" -- -d $K --samples 2

    # Replies gone wrong: said, and asked again 500 ms on.
    emu_options init=060d
    emu_case "wrong init reply" --stderr -- -d $K --samples 1
    emu_case "wrong init reply, its sample" -- -d $K --samples 1
    emu_options "reply=1234020d"
    emu_case "wrong sequence" --stderr -- -d $K --samples 1
    emu_case "wrong sequence, its sample" -- -d $K --samples 1
    emu_options "reply=1234010a"
    emu_case "wrong end" --stderr -- -d $K --samples 1
    emu_options stall=lines
    emu_case "control lines stalled" -- -d $K --samples 1
    emu_options in-stall=1
    emu_case "a stalled read" -- -d $K --scan

    # An unplug with nothing out: the next write is what finds it.
    emu_options quiet-mark=2:5
    emu_case "unplug between requests" --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 5

    # Lost in a wait with no bound: C blocks, or polls for ever.
    emu_options init=
    emu_case "silent at init" --hangs --no-output --timeout 3 -- -d $K --samples 1
    emu_options init=05
    emu_case "half an init reply" --hangs --no-output --timeout 3 -- -d $K --samples 1
    emu_options reply=
    emu_case "silent at the request" --hangs --no-output --timeout 3 -- -d $K --samples 1
    emu_case "silent at the request, a time limit" --hangs --no-output --timeout 3 -- -d $K --time 1500ms
    emu_options reply=1234
    emu_case "half a reply" --hangs --no-output --timeout 3 -- -d $K --samples 1
    # One byte too many and every later reply is cut in the same wrong place.
    emu_options "reply=aa$OK"
    emu_case "a stray byte ahead of a reply" --hangs --no-output --timeout 4 --tx-filter 's/(30000d)+/\1/' -- -d $K --samples 1
    emu_case "a stray byte, a time limit" --no-output --tx-filter "$Q1" -- -d $K --time 3s
    emu_options "reply=${OK}aa"
    emu_case "a stray byte behind a reply" --hangs --timeout 4 --tx-filter 's/(30000d)+/\1/' -- -d $K --samples 2
    # A port gone with a request out.
    emu_options answers=2
    emu_case "unplug with a request out" --hangs --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    emu_options answers=0
    emu_case "unplug at the init reply" --hangs --no-output --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 5

    # What stock sigrok-cli does here: the scan's port is still open.
    SIGROK_CLI=$TONDAJ_STOCK_CLI
    emu_options
    emu_case "stock scan" --keep-header -- -d $K --scan
    emu_case "stock show" --expect-fail --stderr --filter 's/ \(.*\)$//' -- -d $K --show
    emu_case "stock samples" --expect-fail --stderr --filter 's/ \(.*\)$//' -- -d $K --samples 1
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=tondaj-sl-814:conn=$EMU_TTY
}
