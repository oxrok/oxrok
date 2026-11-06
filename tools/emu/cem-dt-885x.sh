# A CEM DT-8852 (emu/src/models/cem-dt-885x.rs) on the CDC-ACM shell. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
#
# wait_for_token reads with serial_read_nonblocking and takes "no byte yet" for
# a device that left (protocol.c:425-427). At 9600 baud no byte is there yet
# nearly always, so every get and set of the meter's state fails on stock
# sigrok-cli, ahead of its toggles. To compare those paths the oracle runs under a
# preload that waits up to a second in sp_nonblocking_read when there is nothing,
# as power_off's comment ("reset the port back to blocking") says the driver was
# written for. It is set on the exec of this model's sigrok-cli only.
#
# The "stock" cases at the end pin the failure, with the read the driver really
# has, but for the scan's: that is the port opened read only, whose 25 ms of wall
# clock (api.c:27) a host can outlast, so a case would say "No devices found" at
# random. It waits there too.
EMU_MODEL=cem-dt-885x
EMU_CDC=1
EMU_DRIVER=cem-dt-885x
EMU_FULL=$EMU_DRIVER
EMU_GCOV=cem-dt-885x
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

cem_blocking() {
    CEM_STOCK_CLI=$SIGROK_CLI
    [ "$no_oracle" = 1 ] && return
    # $1 the file, $2 when a read that found nothing waits.
    cem_preload() {
        printf '%s\n' '#define _GNU_SOURCE' '#include <dlfcn.h>' '#include <stddef.h>' \
            'static int read_only;' \
            'int sp_open(void *port, int flags) {' \
            '    static int (*open_it)(void *, int);' \
            '    if (!open_it) *(void **)&open_it = dlsym(RTLD_NEXT, "sp_open");' \
            '    read_only = flags == 1;' \
            '    return open_it(port, flags);' \
            '}' \
            'int sp_nonblocking_read(void *port, void *buf, size_t count) {' \
            '    static int (*next)(void *, void *, size_t);' \
            '    static int (*wait)(void *, void *, size_t, unsigned int);' \
            '    if (!next) {' \
            '        *(void **)&next = dlsym(RTLD_NEXT, "sp_nonblocking_read");' \
            '        *(void **)&wait = dlsym(RTLD_NEXT, "sp_blocking_read");' \
            '    }' \
            '    int got = next(port, buf, count);' \
            "    return got == 0 && ($2) ? wait(port, buf, count, 1000) : got;" \
            '}' >"$TMP/$1.c"
        cc -shared -fPIC -o "$TMP/$1.so" "$TMP/$1.c" -ldl 2>"$TMP/$1.err" \
            || emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"
    }
    # $1 the script's name, $2 the preload, $3 what follows the arguments.
    cem_wrapper() {
        printf '%s\n' '#!/bin/sh' \
            "LD_PRELOAD=$TMP/$2.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$CEM_STOCK_CLI") \"\$@\" $3" \
            >"$TMP/$1"
        chmod +x "$TMP/$1"
    }
    cem_preload cem-blocking 1
    cem_preload cem-scan read_only
    cem_wrapper cem-sigrok-cli cem-blocking ""
    cem_wrapper cem-scan-cli cem-scan ""
    # glibc's words for a heap it finds broken depend on where it looks first.
    cem_wrapper cem-quiet-cli cem-blocking "2>/dev/null"
    CEM_BLOCKING_CLI=$TMP/cem-sigrok-cli
    SIGROK_CLI=$CEM_BLOCKING_CLI
}

emu_cases() {
    cem_blocking
    emu_conns
    # How much the meter says while a tool listens depends on the time, and an
    # oxrok open's second line state lands among it: what comes in goes. The levels
    # show in the output. A command's token comes ahead of the next command, so the
    # commands never join and each stays on its line.
    OUT='/^bulk_in 82 /d'
    # A log: a tool asks once per byte until it has read the bb, and how many
    # those are depends on what the meter said first. The first stays, and the
    # hold space remembers it.
    ASK='/^bulk_in 82 /d
/^bulk_out 02 (ac)+ -> [0-9]+$/{
x
/^seen$/{
x
d
}
s/.*/seen/
x
s/.*/bulk_out 02 ac -> asked/
}'
    # Power off: whether a byte was on its way at a flush is up to the time, and
    # such a byte says 33 again. How many a lost one takes is in the unit tests.
    POWER='/^bulk_in 82 /d
/^bulk_out 02 (33)+ -> [0-9]+$/{
x
/^seen$/{
x
d
}
s/.*/seen/
x
s/.*/bulk_out 02 33 -> told/
}'
    # Frequency's toggle: C's 10 ms are short enough that a slow pass sends one more.
    # Its count in oxrok is in the unit tests.
    FREQ='/^bulk_in 82 /d
/^bulk_out 02 (99)+ -> [0-9]+$/{
x
/^seen$/{
x
d
}
s/.*/seen/
x
s/.*/bulk_out 02 99 -> told/
}'
    # The driver's SR_ERR and SR_ERR_ARG in each tool's form, and oxrok's log
    # prefix on its side, with no line of a tool's own dropped.
    F='s/^oxrok: [a-z]+: oxrok::drivers::cem_dt_885x(::protocol)?: /sr: cem-dt-885x: /
s/^(Failed to get .[a-z_]+.: )device error: generic\/unspecified error$/\1generic\/unspecified error/
s/^(Failed to set device option .[a-z_]+.: )device error: generic\/unspecified error\.$/\1generic\/unspecified error./
s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    NO_BYTE_BUG="protocol.c:425-427 takes a read that found no byte yet for a meter that left, and at 9600 baud that is nearly every read: a get or a set of the meter's state fails on a meter that is talking; oxrok waits for the byte. The other cases here hold what C's wire would be, under a preload that makes the read wait"
    NO_BYTE_SHOW="As NO_BYTE_BUG: a get finds no byte (protocol.c:425-427), and show.c prints each list with no (current), which oxrok has"
    NO_END_BUG="wait_for_token with a timeout of 0 goes on for as long as bytes come (protocol.c:424-440): with the preload's read a meter that does not say it holds it for ever; oxrok gives up after 2 s"
    DEAF_BUG="cem_dt_885x_toggle goes on for ever on a meter that does not hear it (protocol.c:459-467); oxrok gives up after 5 s"
    POWER_BUG="cem_dt_885x_power_off goes on for ever on a meter that goes on talking (protocol.c:819-828); oxrok gives up after 5 s"
    SILENT_BUG="receive_data never hears of a silent meter, so a --samples capture never ends; oxrok ends it after 3 s with no byte"
    UNPLUG_BUG="after an unplug a --samples capture hangs in C, by reading protocol.c:392 as nothing arriving with revents just G_IO_IN; oxrok's read fails and ends it"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    STALE_RANGE_BUG="protocol.c:806 does not store the range it set, so a get after the set answers the old one (:729); oxrok answers the new one"
    OVERSHOOT_BUG="send_data sends a whole packet of up to 50 levels (protocol.c:369-371) and looks at the limit after it (:210); session.c's limit cuts only the decoders' input, so stdout has them all; oxrok holds the limit"
    OVERFLOW_BUG="a record's head with no known token is never reset (protocol.c:338-341), so :330 stores each later byte on past buf[100] and over last_spl, hold_last_sent and the heap after devc; oxrok stores no more than the head"

    # The scan: the first a5.
    # No chunk= on this stream: with every read completing all the time, a close
    # hits the kernel's unlink race (the model's PERIOD) and the session is lost.
    # The unit tests' fake gives a byte per read.
    emu_options setters=0
    emu_case "scan" --keep-header --tx-filter "$OUT" -- -d $X --scan
    emu_case "show" --keep-header --tx-filter "$OUT" -- -d $X --show
    emu_case "no such node" --keep-header --no-output -- -d cem-dt-885x:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d cem-dt-885x --scan
    emu_options setters=0 blocks=0
    emu_case "scan silent" --keep-header --no-output -- -d $X --scan
    emu_options setters=0 hold-blocks=0
    emu_case "scan in hold has no a5" --keep-header --no-output --tx-filter "$OUT" -- -d $X --scan
    emu_options setters=0 uart=2400/8n1
    emu_case "scan another rate" --keep-header --no-output -- -d $X --scan

    # A get waits for the token. All of the state, each way.
    emu_options setters=0
    for key in spl_weight_freq spl_weight_time spl_meas_range datalog hold_max hold_min power_off data_source limit_samples; do
        emu_case "get $key" --tx-filter "$OUT" -- -d $X --get $key
    done
    emu_case "get continuous unlisted" --tx-filter "$OUT" --stderr --filter "$F" --expect-fail -- -d $X --get continuous
    emu_case "get conn unlisted" --tx-filter "$OUT" --stderr --filter "$F" --expect-fail -- -d $X --get conn
    emu_options setters=0 freq=C time=S recording=1
    for key in spl_weight_freq spl_weight_time datalog; do
        emu_case "get $key other" --tx-filter "$OUT" -- -d $X --get $key
    done
    for r in 30-80 50-100 80-130; do
        emu_options setters=0 range=$r
        emu_case "get range $r" --tx-filter "$OUT" -- -d $X --get spl_meas_range
    done
    for m in max min; do
        emu_options setters=0 hold-mode=$m
        emu_case "get hold_max with $m" --tx-filter "$OUT" -- -d $X --get hold_max
        emu_case "get hold_min with $m" --tx-filter "$OUT" -- -d $X --get hold_min
    done
    emu_options setters=0 freq=C time=S range=80-130 recording=1 hold-mode=min
    emu_case "show the other state" --keep-header --tx-filter "$OUT" -- -d $X --show
    emu_options setters=0
    emu_case "two gets" --tx-filter "$OUT" -- -d $X --get spl_weight_freq --get spl_meas_range

    # A get when the token does not come. datalog and a range's set have a time.
    emu_options setters=0 time-only=1
    emu_case "get datalog does not come" --tx-filter "$OUT" --stderr --filter "$F" --expect-fail -- -d $X --get datalog
    emu_case "set range does not come" --tx-filter "$OUT" --stderr --filter "$F" --expect-fail -- -d $X -c spl_meas_range=30-80 --set
    emu_options setters=0 time-only=1 blocks=2
    for key in spl_weight_freq spl_weight_time spl_meas_range datalog hold_max; do
        emu_case "get $key a meter gone quiet" --tx-filter "$OUT" --stderr --filter "$F" --expect-fail -- -d $X --get $key
    done
    for set in spl_weight_freq=C spl_weight_time=S spl_meas_range=30-80 datalog=on hold_max=on; do
        emu_case "set $set a meter gone quiet" --tx-filter "$OUT" --stderr --filter "$F" --expect-fail -- -d $X -c $set --set
    done
    emu_case "show a meter gone quiet" --keep-header --tx-filter "$OUT" -- -d $X --show
    emu_options setters=0 time-only=1
    emu_case "get freq that never comes" --oracle-bug "$NO_END_BUG" --oracle-hangs --timeout 4 --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X --get spl_weight_freq
    # In hold nothing but ff comes, and the first of them sets the flag.
    emu_options setters=0 time-only=1 hold-blocks=2
    emu_case "get freq in hold" --oracle-bug "$NO_END_BUG" --oracle-hangs --timeout 4 --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X --get spl_weight_freq

    # A setting: nothing to do, one toggle, and toggles lost. A block that goes out
    # as a command does brings the old state after it, and C takes that as the
    # command lost, sends it again and undoes it: real, and by the clock, and oxrok's
    # flush leaves no room for it. So these meters say what
    # is asked for in two blocks and then only answer: short blocks (range, hold,
    # frequency) by default, and with startup=2 the second is long (time, recording).
    Q="blocks=2"
    QL="blocks=2 startup=2"
    QH="blocks=2 startup=1"
    emu_options $Q
    emu_case "set freq C" --no-output --tx-filter "$FREQ" -- -d $X -c spl_weight_freq=C --set
    emu_options $QL
    emu_case "set time S" --no-output --tx-filter "$OUT" -- -d $X -c spl_weight_time=S --set
    emu_case "set datalog on" --no-output --tx-filter "$OUT" -- -d $X -c datalog=on --set
    emu_options $QH
    emu_case "set hold_max on" --no-output --tx-filter "$OUT" -- -d $X -c hold_max=on --set
    emu_case "set hold_min on" --no-output --tx-filter "$OUT" -- -d $X -c hold_min=on --set
    # Only the range's announcements count, and it needs the stream to time out on.
    # C waits 110 ms for the first, so no long block leads: startup=3.
    emu_options startup=3
    for r in 30-80 50-100 80-130; do
        emu_case "set range $r" --no-output --tx-filter "$OUT" -- -d $X -c spl_meas_range=$r --set
    done
    emu_options setters=0
    emu_case "set freq A has nothing to do" --no-output --tx-filter "$OUT" -- -d $X -c spl_weight_freq=A --set
    emu_case "set time F has nothing to do" --no-output --tx-filter "$OUT" -- -d $X -c spl_weight_time=F --set
    emu_case "set datalog off has nothing to do" --no-output --tx-filter "$OUT" -- -d $X -c datalog=off --set
    emu_case "set hold_max off has nothing to do" --no-output --tx-filter "$OUT" -- -d $X -c hold_max=off --set
    emu_case "set hold_min off has nothing to do" --no-output --tx-filter "$OUT" -- -d $X -c hold_min=off --set
    emu_options setters=0 startup=3
    emu_case "set range has nothing to do" --no-output --tx-filter "$OUT" -- -d $X -c spl_meas_range=30-130 --set
    emu_options setters=0
    emu_case "set power_off off" --no-output --tx-filter "$OUT" -- -d $X -c power_off=off --set
    emu_case "set data_source Live" --no-output --tx-filter "$OUT" -- -d $X -c data_source=Live --set
    emu_case "set data_source Memory" --no-output --tx-filter "$OUT" -- -d $X -c data_source=Memory --set
    emu_case "set limit_samples" --no-output --tx-filter "$OUT" -- -d $X -c limit_samples=5 --set
    emu_case "set data_source then get" --tx-filter "$OUT" -- -d $X -c data_source=Memory --get data_source
    emu_case "set limit then get" --tx-filter "$OUT" -- -d $X -c limit_samples=7 --get limit_samples
    emu_options deaf=2 $QH
    emu_case "set hold_max lost twice" --no-output --tx-filter "$OUT" -- -d $X -c hold_max=on --set
    emu_options deaf=2 startup=3
    emu_case "set range lost twice" --no-output --tx-filter "$OUT" -- -d $X -c spl_meas_range=30-80 --set
    emu_options deaf=1000000
    emu_case "set freq on a deaf meter" --oracle-bug "$DEAF_BUG" --oracle-hangs --timeout 7 --stderr --filter "$F" --expect-fail --tx-filter '/^bulk_in 82 /d
/^bulk_out 02 99 -> 1$/d' -- -d $X -c spl_weight_freq=C --set
    emu_options hold-mode=max $QH
    emu_case "set hold_max off from max" --no-output --tx-filter "$OUT" -- -d $X -c hold_max=off --set
    emu_case "set hold_min on from max" --no-output --tx-filter "$OUT" -- -d $X -c hold_min=on --set
    emu_options hold-mode=min $QH
    emu_case "set hold_max on from min" --no-output --tx-filter "$OUT" -- -d $X -c hold_max=on --set
    # The token of a toggle never comes, as the meter says nothing more.
    emu_options mute=1 $Q
    emu_case "set freq no answer" --stderr --filter "$F" --expect-fail --tx-filter "$FREQ" -- -d $X -c spl_weight_freq=C --set
    emu_options blocks=3 startup=3
    emu_case "set two" --no-output --tx-filter "$FREQ" -- -d $X -c spl_weight_freq=C:spl_weight_time=S --set
    emu_options startup=3
    emu_case "set range then get it" --oracle-differs "$STALE_RANGE_BUG" --tx-filter "$OUT" -- -d $X -c spl_meas_range=30-80 --get spl_meas_range
    # Refused.
    emu_options setters=0
    emu_case "set freq no such" --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X -c spl_weight_freq=Z --set
    emu_case "set time no such" --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X -c spl_weight_time=Z --set
    emu_case "set range no such" --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X -c spl_meas_range=10-20 --set
    emu_case "set data_source no such" --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X -c data_source=Cloud --set
    emu_case "set no such key" --stderr --filter "$F" --expect-fail --tx-filter "$OUT" -- -d $X -c voltage=1 --set

    # power_off: 33, 25 ms, and a look for a byte; then the port goes round.
    emu_options
    emu_case "power off" --no-output --tx-filter "$POWER" -- -d $X -c power_off=on --set
    emu_options deaf=2
    emu_case "power off lost twice" --no-output --tx-filter "$POWER" -- -d $X -c power_off=on --set
    emu_options deaf=1000000
    emu_case "power off on a deaf meter" --oracle-bug "$POWER_BUG" --oracle-hangs --timeout 7 --stderr --filter "$F" --expect-fail --tx-filter "$POWER" -- -d $X -c power_off=on --set

    # Live levels. Which ones show depends on the display token after each.
    emu_options setters=0
    emu_case "one level" --tx-filter "$OUT" -- -d $X --samples 1
    emu_case "five levels" --tx-filter "$OUT" -- -d $X --samples 5
    emu_case "five levels as csv" --tx-filter "$OUT" -- -d $X --samples 5 -O csv:header=false
    emu_options setters=0 bar=0
    emu_case "only readouts" --tx-filter "$OUT" -- -d $X --samples 3
    emu_options setters=0 freq=C time=S range=80-130 recording=1 battery=low store=full threshold=over
    emu_case "the other flags" --tx-filter "$OUT" -- -d $X --samples 3
    emu_options setters=0 threshold=under
    emu_case "under range" --tx-filter "$OUT" -- -d $X --samples 1
    emu_options setters=0 hold-mode=max bar=1
    emu_case "max hold drops the bar graph" --tx-filter "$OUT" -- -d $X --samples 3
    emu_options setters=0 hold-mode=min bar=1
    emu_case "min hold drops the bar graph" --tx-filter "$OUT" -- -d $X --samples 3
    # Not under setters=0: a tty's first echo sends 99 back.
    emu_options levels=9999,0000,0abc,f0f9
    emu_case "levels that are no BCD" --tx-filter "$OUT" -- -d $X --samples 4

    # Noise. A token no one knows takes all up to the next a5 or bb, and that one's
    # command with it: the level after it is lost.
    emu_options setters=0 extra=a5e1
    emu_case "unknown token" --tx-filter "$OUT" -- -d $X --samples 2
    emu_options setters=0 extra=a5e1010203
    emu_case "unknown token with data" --tx-filter "$OUT" -- -d $X --samples 2
    emu_options setters=0 extra=a5a5
    emu_case "a5 as a token" --tx-filter "$OUT" -- -d $X --samples 2
    emu_options setters=0 extra=0102030405
    emu_case "bytes between packets" --tx-filter "$OUT" -- -d $X --samples 2
    # Past the buffer: protocol.c:289 stores a byte past it before :290 looks. Only
    # a level's low digits could show that, and the output has one decimal.
    emu_options setters=0 extra=a5e1$(printf '07%.0s' $(seq 1 101))
    emu_case "unknown token over the buffer" --tx-filter "$OUT" -- -d $X --samples 2
    emu_options setters=0 extra=a506
    emu_case "time cut by the next packet" --tx-filter "$OUT" -- -d $X --samples 2
    # ff cuts any packet.
    emu_options setters=0 extra=a50dff
    emu_case "ff in a packet" --tx-filter "$OUT" -- -d $X --samples 2
    # A log no one asked for: it ends the capture.
    emu_options setters=0 extra=bb0064dd
    emu_case "a log unasked" --no-output --tx-filter "$OUT" -- -d $X --samples 2

    # Hold: the last level again every 50 ms for as long as ff comes.
    emu_options setters=0 hold-after=1
    emu_case "hold repeats the level" --tx-filter "$OUT" -- -d $X --samples 3
    emu_options setters=0 hold-blocks=2
    emu_case "hold with no level yet" --tx-filter "$OUT" -- -d $X --samples 2
    emu_options setters=0 hold-after=1 hold-for=1
    emu_case "hold ends at once" --tx-filter "$OUT" -- -d $X --samples 2

    # The log. Asked for once per byte, until its bb is read. A meter says levels
    # from the open, and how many come ahead of the bb depends on how long a tool
    # takes to start, so these meters have none.
    emu_options no-level=1
    emu_case "log" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 100
    emu_case "log as csv" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 100 -O csv:header=false
    emu_options no-level=1 log=empty
    emu_case "log empty" --tx-filter "$ASK" --no-output -- -d $X -c data_source=Memory --samples 100
    emu_options no-level=1 log=c59:1
    emu_case "log of one level" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 100
    # A packet of 50 levels at a time.
    emu_options no-level=1 log=a02:120
    emu_case "log over the packet size" --tx-filter "$ASK" --digest --filter '$=' -- -d $X -c data_source=Memory --samples 500
    emu_case "log limit between packets" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 50 -O csv:header=false
    emu_case "log limit in a packet" --oracle-differs "$OVERSHOOT_BUG" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 3
    # The stop inside the last record, on the byte of its end marker.
    emu_options no-level=1 log=a02:2
    emu_case "log limit at the end" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 2
    # Levels with no record ahead of them keep the live flags; whether a long
    # block has said F by then is up to the time.
    emu_options no-level=1 dump=bb0064ac0655dd
    emu_case "log levels with no record" --tx-filter "$ASK" --filter 's/ F$//' -- -d $X -c data_source=Memory --samples 100
    # No stray byte: nothing to cut.
    emu_options no-level=1 dump=bb0070aa26110512345601ac06550661dd
    emu_case "log with no stray byte" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 100
    emu_options setters=0
    emu_case "capture live after a set to Live" --tx-filter "$OUT" -- -d $X -c data_source=Live --samples 1
    # A record token no one knows, its levels after it with the flags it left.
    emu_options no-level=1 dump=bb0070112611051234561aac0655dd
    emu_case "log unknown record token" --tx-filter "$ASK" --filter 's/ F$//' -- -d $X -c data_source=Memory --samples 100
    # One that goes on: protocol.c:330 stores each byte, and the head never ends.
    JUNK=$(printf '12%.0s' $(seq 1 4000))
    emu_options no-level=1 dump=bb0064${JUNK}dd
    [ "$no_oracle" = 1 ] || SIGROK_CLI=$TMP/cem-quiet-cli
    emu_case "log unknown record token, long" --oracle-bug "$OVERFLOW_BUG" --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 --no-output --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 100
    [ "$no_oracle" = 1 ] || SIGROK_CLI=$CEM_BLOCKING_CLI

    # What ends a capture or does not.
    emu_options setters=0 time-only=1 blocks=2
    emu_case "silence" --oracle-bug "$SILENT_BUG" --oracle-hangs --timeout 6 --no-output --tx-filter "$OUT" -- -d $X --samples 1
    # Its log has no end and no more bytes.
    emu_options time-only=1 blocks=2 dump=bb0064
    emu_case "silence ends a log" --oracle-bug "$SILENT_BUG" --oracle-hangs --timeout 6 --no-output --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 1
    emu_options setters=0 mark=20
    emu_case "unplug in a capture" --oracle-bug "$UNPLUG_BUG" --oracle-hangs --unplug model --timeout 6 --filter '1!d;s/.*/levels/' --tx-filter "$OUT" -- -d $X --samples 100000
    # The model goes on sending after the point, so only a stop ends it. oxrok
    # takes SIGINT only once its capture is under way, so the point comes late.
    emu_options setters=0 mark=20
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --filter '1!d;s/.*/levels/' --tx-filter "$OUT" -- -d $X --continuous

    # The library runner finds libsigrok by sigrok-cli's path, so the real one; the
    # scan's read waits through its environment.
    SIGROK_CLI=$CEM_STOCK_CLI
    SCAN=LD_PRELOAD=$TMP/cem-scan.so
    emu_options no-level=1
    emu_case "log with no limit" --libsigrok --oxrok --oracle-env "$SCAN" --tx-filter "$ASK" -- -d $X -c data_source=Memory --continuous
    emu_options setters=0
    emu_case "lib stop" --libsigrok --oracle-env "$SCAN" --tx-filter "$OUT" -- -d $X --continuous --stop-after 2

    # The read the driver really has: no byte yet is an error.
    [ "$no_oracle" = 1 ] || SIGROK_CLI=$TMP/cem-scan-cli
    emu_case "stock levels" --tx-filter "$OUT" -- -d $X --samples 3
    # A block that beat C's first read could say what it asks for: nothing comes
    # for some 100 ms after the open, and these tools have read by then.
    emu_options setters=0 first-block=8
    for key in spl_weight_freq spl_weight_time spl_meas_range datalog hold_max hold_min; do
        emu_case "stock get $key" --oracle-bug "$NO_BYTE_BUG" --tx-filter "$OUT" -- -d $X --get $key
    done
    emu_case "stock get what needs no meter" --tx-filter "$OUT" -- -d $X --get power_off --get data_source --get limit_samples
    emu_case "stock show has no state" --oracle-differs "$NO_BYTE_SHOW" --keep-header --tx-filter "$OUT" -- -d $X --show
    emu_options first-block=8
    emu_case "stock set fails" --oracle-bug "$NO_BYTE_BUG" --no-output --tx-filter "$OUT" -- -d $X -c spl_weight_freq=C --set
    emu_options
    emu_case "stock power off" --no-output --tx-filter "$POWER" -- -d $X -c power_off=on --set
    emu_options no-level=1
    emu_case "stock log" --tx-filter "$ASK" -- -d $X -c data_source=Memory --samples 100
    SIGROK_CLI=$CEM_STOCK_CLI
}

emu_conns() {
    X=cem-dt-885x:conn=$EMU_TTY
}
