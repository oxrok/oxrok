# A Tondaj SL-814 sound level meter (emu/src/models/tondaj-sl-814.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model".
EMU_MODEL=tondaj-sl-814
EMU_CDC=1
EMU_DRIVER=tondaj-sl-814
EMU_FULL=$EMU_DRIVER
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
# lock and nothing of the timing. On stock sigrok-cli only the scans and the stock
# cases hold: each other label says it is under the preload.
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
    # What each command looks like, for a .tx cut there.
    Q0='/^bulk_out 02 10040d/q'
    Q1='/^bulk_out 02 30000d/q'
    # A reply's frame with the sequence for 0.
    OK=1234010d
    # oxrok's log prefix, on its side only, and its own wording for a 0.
    L='s/^oxrok: (err|warn): oxrok::drivers::tondaj_sl_814(::protocol)?: /sr: tondaj-sl-814: /'
    A='s/^(Failed to set device option .limit_samples.: invalid argument): it cannot be 0\.$/\1./'
    # What a failed write says, past its words: C's return value, oxrok's OS error,
    # and libserialport's line, which oxrok's port does not log.
    W="$L
/^sr: serial-libsp: /d
/^oxrok: warn: oxrok::cli::pipeline: Device only sent /d
s/^(sr: tondaj-sl-814: Error sending request command): .*\$/\1./"
    LOCK_BUG="api.c:85 opens the port and :96 returns with it open, so dev_open's second open is refused by the exclusive open of libserialport 0.1.2 and later (serialport.c:589, :599): every path but --scan fails on stock sigrok-cli there, and works on 0.1.1; oxrok closes the port after the scan and opens it again"
    P="under the lock preload, C unmodified:"
    INIT_BUG="$P protocol.c:148 reads the init reply with serial_read_blocking and a timeout of 0, which is none (serial.c:481), in the session's callback: nothing ends that wait, not even a limit; oxrok on a sample limit alone fails after 5 s"
    REPLY_BUG="$P protocol.c:176-187 reads a reply and returns ahead of the limit's check at :207 for as long as it has not all come, and no request goes again: a capture never ends; oxrok on a sample limit fails after 5 s with no sample yet, and after one asks again"
    TIME_BUG="$P protocol.c:187 returns ahead of the limit's check at :207, so a time limit does not end a wait for a reply; oxrok asks again after 5 s, ends at the limit and warns"
    STRAY_BUG="$P protocol.c:193-198 takes four bytes and asks again with the fifth unread, and :172 clears only the count, so every later reply is read cut in the same wrong place and fails :193: no reading ever again; oxrok reads away what comes with no command out, and goes on"
    STRAY_TIME_BUG="$STRAY_BUG; a time limit reaches :207 through the send, so C ends with exit 0 and no sample"
    SIGINT_BUG="$P sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # What the third request of a stray byte's capture gets, for a .tx cut there.
    Q3='/^bulk_in 82 4 -> 09af010d/q'

    # The scan says nothing on the wire and takes no answer: a port that opens is
    # a meter.
    emu_options
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=9600/8e1 --scan
    emu_case "no such node" --keep-header --no-output -- -d tondaj-sl-814:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d tondaj-sl-814 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses them
    # first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
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

    # A capture: init at the source's first 500 ms, its reply at once, a request
    # 500 ms on, and each next one 500 ms after the last reply.
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2 analog" -- -d $K --samples 2 -O analog
    emu_case "samples csv" -- -d $K --samples 2 -O csv:header=false:time=false
    emu_case "config limit_samples" -- -d $K -c limit_samples=2 --samples 1
    emu_case "channel P1" -- -d $K -C P1 --samples 1
    emu_case "no such channel" --stderr --filter 's/^Failed to set channels: invalid argument: //' --expect-fail -- -d $K -C P2 --samples 1
    # A limit ahead of the source's first pass: its command still goes, in both.
    emu_case "time 1 ms" --no-output --tx-filter "$Q0" -- -d $K --time 1ms
    # Time decides how many: kept up to the first.
    emu_case "time" --filter '2,$d' --tx-filter "$Q1" -- -d $K --time 2s
    # The line reaches the meter, which hears nothing at another rate.
    emu_options uart=2400/8e1
    emu_case "serialcomm reaches the line" -- -d $K:serialcomm=2400/8e1 --samples 1

    # The wiki's readings and the ends of the value, with all four weightings, the
    # level that is dropped and the unknown bit; the rest is in the unit tests.
    emu_options data=09af,47ff,9285,a393,ffff
    emu_case "readings" -- -d $K --samples 5 -O analog

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
    emu_case "wrong init reply" --stderr --filter "$L" -- -d $K --samples 1
    emu_case "wrong init reply, its sample" -- -d $K --samples 1
    emu_options reply=1234020d
    emu_case "wrong sequence" --stderr --filter "$L" -- -d $K --samples 1
    emu_case "wrong sequence, its sample" -- -d $K --samples 1
    emu_options reply=1234010a
    emu_case "wrong end" --stderr --filter "$L" -- -d $K --samples 1
    emu_options stall=lines
    emu_case "control lines stalled" -- -d $K --samples 1

    # Each wait C never leaves, and the words oxrok leaves it with.
    emu_options init=
    emu_case "silent at init" --oxrok-fails "no answer to init for 5 s" --oracle-bug "$INIT_BUG" --oracle-hangs --no-output --timeout 7 -- -d $K --samples 1
    emu_options init=05
    emu_case "half an init reply" --oxrok-fails "no usable answer for 5 s" --oracle-bug "$INIT_BUG" --oracle-hangs --no-output --timeout 7 -- -d $K --samples 1
    emu_options reply=
    emu_case "silent at the request" --oxrok-fails "no answer to a request for 5 s" --oracle-bug "$REPLY_BUG" --oracle-hangs --no-output --timeout 10 -- -d $K --samples 1
    emu_case "silent, a time limit" --oracle-bug "$TIME_BUG" --oracle-hangs --stderr --filter "$L" --timeout 5 -- -d $K --time 2500ms
    # Past the first wait: asked again with a number of its own, told once, and a
    # time limit past the bound ends as a short one does. The clock decides whether
    # a stalled tool got to ask again by then: the next case holds that it does.
    emu_options answers=2
    emu_case "dead, a long time limit" --oracle-bug "$TIME_BUG" --oracle-hangs --stderr --filter "$L" --timeout 11 --tx-filter 's/^bulk_out 02 30000d(30010d)? -> [36]$/bulk_out 02 30000d/' -- -d $K --time 9s
    # A lost reply: C never ends, and oxrok asks again with a warning and goes on.
    emu_options "reply=$OK,"
    emu_case "a reply lost" --oracle-bug "$REPLY_BUG" --oracle-hangs --timeout 10 -- -d $K --samples 2
    emu_case "a reply lost, told" --oracle-bug "$REPLY_BUG" --oracle-hangs --stderr --filter "$L" --timeout 10 -- -d $K --samples 2
    emu_options reply=1234
    emu_case "half a reply" --oxrok-fails "no usable answer for 5 s" --oracle-bug "$REPLY_BUG" --oracle-hangs --no-output --timeout 10 -- -d $K --samples 1
    emu_case "half a reply, a time limit" --oracle-bug "$TIME_BUG" --oracle-hangs --stderr --filter "$L" --timeout 5 -- -d $K --time 2500ms

    # One byte too many and every later reply is cut in the same wrong place in C.
    # Each list plays once, as a second stray byte can put C back in step.
    emu_options "reply=aa$OK"
    emu_case "a stray byte ahead of a reply" --oracle-bug "$STRAY_BUG" --oracle-hangs --timeout 5 -- -d $K --samples 2
    emu_case "a stray byte ahead of a reply, said" --oracle-bug "$STRAY_BUG" --oracle-hangs --stderr --filter "$L" --timeout 5 -- -d $K --samples 1
    # Time decides how many oxrok has: kept up to the first, and C has none.
    emu_case "a stray byte, a time limit" --oracle-differs "$STRAY_TIME_BUG" --filter '2,$d' --tx-filter "$Q3" -- -d $K --time 3s
    emu_options "reply=${OK}aa"
    emu_case "a stray byte behind a reply" --oracle-bug "$STRAY_BUG" --oracle-hangs --timeout 5 -- -d $K --samples 2

    # An unplug with nothing out: the next write is what finds it.
    emu_options quiet-mark=2:5
    emu_case "unplug between requests" --oxrok-fails "error" --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    emu_case "unplug between requests, said" --oxrok-fails "error" --stderr --filter "$W" --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    # What that filter takes from oxrok's side, pinned: the OS error's text alone is
    # masked here, which goes with the system.
    emu_case "unplug between requests, oxrok's words" --no-oracle-only --oxrok-fails "error" --stderr --filter "$L
s/^(sr: tondaj-sl-814: Error sending request command: ).*\$/\1OS error./" --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    # With a request out nothing tells a gone tty from a silent meter.
    emu_options answers=2
    emu_case "unplug with a request out" --oxrok-fails "no answer to a request" --oracle-bug "$REPLY_BUG" --oracle-hangs --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    emu_options answers=0
    emu_case "unplug at the init reply" --oxrok-fails "no answer to init" --oracle-bug "$INIT_BUG" --oracle-hangs --no-output --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    # A quiet point: the request goes unanswered, the signal lands in its wait.
    emu_options answers=2
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 8 --tx-filter '/^interrupt$/q' -- -d $K --samples 5

    # What stock sigrok-cli does here: the scan's port is still open.
    SIGROK_CLI=$TONDAJ_STOCK_CLI
    emu_options
    emu_case "stock scan" --keep-header -- -d $K --scan
    emu_case "stock show" --oracle-bug "$LOCK_BUG" --keep-header -- -d $K --show
    emu_case "stock samples" --oracle-bug "$LOCK_BUG" -- -d $K --samples 1
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=tondaj-sl-814:conn=$EMU_TTY
}
