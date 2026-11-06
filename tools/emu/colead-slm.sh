# A Colead SL-5868P sound level meter (emu/src/models/colead-slm.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model".
EMU_MODEL=colead-slm
EMU_CDC=1
EMU_DRIVER=colead-slm
EMU_FULL=$EMU_DRIVER
EMU_GCOV=colead-slm
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# A record, with its sum: configuration, five digits, status.
slm() {
    printf '0804%02x%02x%02x%02x%02x%02x%02x%02x' "$1" "$2" "$3" "$4" "$5" "$6" "$7" \
        $(((0x08 + 0x04 + $1 + $2 + $3 + $4 + $5 + $6 + $7) & 0xff))
}

# slm_sweep NAME SAMPLES RECORD...: a capture of these records, one for each
# announcement, with silence after the last. The last must be a sample, or a
# tool waits. SLM_FLAGS and SLM_ARGS are the case's flags and its tool's.
slm_sweep() {
    _name=$1
    _samples=$2
    shift 2
    _list=
    for _p in "$@"; do
        _list=$_list${_list:+,}$_p
    done
    emu_options period=5 burst=$# packet=$_list
    # shellcheck disable=SC2086
    emu_case "$_name" $SLM_FLAGS ${SLM_WHY:+--oracle-differs "$SLM_WHY"} --filter "$SLM_FILTER" -- -d $C $SLM_ARGS --samples $_samples
}

emu_cases() {
    emu_conns
    # The driver's own lines at any level, without the time.
    D='s/^oxrok: [a-z]+: oxrok::drivers::colead_slm[a-z_:]*: /sr: colead-slm: /; s/^sr: \[[^]]*\] /sr: /; /^sr: colead-slm: /!d; s/^sr: //'
    NO_END="protocol.c:177-180 looks at the limits behind a level alone, so a meter that sends none keeps a capture open for ever; oxrok ends a time limit on time"
    NO_BOUND="protocol.c:177-180 looks at the limits behind a level alone, so a sample limit on a meter that sends none is open for ever; oxrok ends it after 15 s, with an error"
    HUP_BUG="protocol.c:199-201 takes a hung up port for a timeout, as revents is not exactly G_IO_IN, and so never ends; oxrok ends the capture with an error"
    SLM_WHY=
    # What oxrok adds to the error sigrok-cli names, exactly those words.
    A='s/^(Failed to set device option .limit_samples.: invalid argument): it cannot be 0\.$/\1./'
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    CHAR_DIGIT="protocol.c:70-73 reads the digit as char, signed on this platform, so 0xff is -1 and the level is -1024.7; where char is unsigned it is passed over, as oxrok does. No meter sends one"
    CHAR_SUM="protocol.c:63 compares a masked sum with a char, signed on this platform, so a sum of 0x80 or more never matches and the record is refused; where char is unsigned it matches, as in oxrok. No meter's sum comes to that"
    SLM_FLAGS=
    SLM_ARGS=
    SLM_FILTER=
    GOOD=$(slm 0x10 10 10 7 5 3 1)

    # A scan touches nothing: the first .tx is empty, wrong node or not.
    emu_options burst=0
    emu_case "scan" --keep-header -- -d $C --scan
    emu_case "no conn" --keep-header --no-output -- -d colead-slm --scan
    emu_case "no such node finds a device" --keep-header -- -d colead-slm:conn=/dev/ttyACM97 --scan
    emu_case "show" --keep-header -- -d $C --show
    emu_case "scan with serialcomm" --keep-header -- -d $C:serialcomm=9600/7e1 --scan
    emu_case "show no such node" --keep-header --expect-fail -- -d colead-slm:conn=/dev/ttyACM97 --show
    emu_case "capture no such node" --keep-header --no-output --expect-fail -- -d colead-slm:conn=/dev/ttyACM97 --samples 1
    emu_case "capture no such serialcomm" --keep-header --no-output --expect-fail --tx-filter '/^ctl_out/d' -- -d $C:serialcomm=fast --samples 1

    # The keys: there is no get at all.
    emu_case "get limit_samples" --expect-fail --stderr -- -d $C --get limit_samples
    emu_case "get limit_time" --expect-fail --stderr -- -d $C --get limit_time
    emu_case "get conn" --expect-fail --stderr -- -d $C --get conn
    emu_case "set limit_samples" --no-output -- -d $C -c limit_samples=3 --set
    emu_case "set limit_time" --no-output -- -d $C -c limit_time=300 --set
    emu_case "frames has no key" --expect-fail --stderr -- -d $C --frames 1
    emu_case "samples 0 refused" --expect-fail --stderr --filter "$A" -- -d $C -c limit_samples=0 --set

    # Captures. One announcement each, then silence, so that no count here
    # depends on how long a tool listens.
    emu_options period=5 burst=1
    emu_case "one sample" -- -d $C --samples 1
    emu_options period=5 burst=3
    emu_case "three samples" -- -d $C --samples 3
    emu_case "three samples csv" -- -d $C --samples 3 -O csv:header=false:time=false
    emu_options period=5 burst=2
    emu_case "samples from limit_samples" -- -d $C -c limit_samples=2 --time 10s
    emu_case "samples before a long time limit" -- -d $C --samples 2 --time 10s
    # C looks at the time only behind a sample, so the first ends this.
    emu_options period=5 burst=1
    emu_case "time ends behind the first sample" -- -d $C --time 1ms
    emu_options period=5 burst=2 uart=9600/8n1
    emu_case "serialcomm as the device is set" -- -d $C:serialcomm=9600/8n1 --samples 2
    # With no burst, about the wiki's rate. The limit comes well before the
    # next announcement.
    emu_options period=50
    emu_case "free running" -- -d $C --samples 2
    # Byte by byte, a reply comes in ten reads. The offers go fast there.
    emu_options period=400 burst=2 chunk=1
    emu_case "one byte per transfer" -- -d $C --samples 2

    # Every configuration of the two tables, and the max hold of each.
    _all=
    for _high in 0x10 0x20; do
        for _low in 0 1 2 3 4 5 6 7 8 9 10 11 12 13; do
            _all="$_all $(slm $((_high + _low)) 10 10 9 4 0 1)"
        done
    done
    # shellcheck disable=SC2086
    slm_sweep "every configuration" 28 $_all

    # Values: blanks, the most, zero, every digit.
    slm_sweep "values" 7 "$(slm 0x10 0 0 0 0 0 1)" "$(slm 0x10 1 2 3 4 5 1)" "$(slm 0x10 6 7 8 9 0 1)" \
        "$(slm 0x10 9 9 9 9 9 1)" "$(slm 0x10 10 10 10 10 5 1)" "$(slm 0x10 1 10 2 10 3 1)" "$GOOD"
    # Digits all ignored make a level of 0.0.
    slm_sweep "a record of ignored digits is a level of nothing" 1 "$(slm 0x10 10 10 10 10 10 1)"
    # Anything above 9 is passed over, not only 0x0a.
    slm_sweep "bytes above nine are passed over" 2 "$(slm 0x10 11 12 7 15 5 1)" "$(slm 0x10 16 32 7 5 3 1)"
    # C reads its buffer as char, signed here. A digit with the top bit goes in
    # as a negative one, and a sum with the top bit never matches.
    SLM_WHY=$CHAR_DIGIT
    slm_sweep "a digit with the top bit" 1 "$(slm 0x10 255 255 7 5 3 1)"
    SLM_WHY=
    # C needs the second announcement and oxrok doesn't, so it comes behind
    # oxrok's record or not, as the time to its close has it.
    emu_options period=5 burst=2 "packet=$(slm 0x10 96 10 7 5 3 1),$(slm 0x10 10 10 8 0 0 1)"
    emu_case "a sum with the top bit" --oracle-differs "$CHAR_SUM" --tx-filter 's/^bulk_in 82 11 -> (.*)10$/bulk_in 82 10 -> \1/' -- -d $C --samples 1

    # What is dropped does not count, and the state goes back to waiting.
    SLM_FLAGS="--stderr"
    SLM_ARGS="-l 4"
    SLM_FILTER=$D
    slm_sweep "log first byte" 1 "$(printf '0904100a0a07050301%02x' 65)" "$GOOD"
    slm_sweep "log second byte" 1 "$(printf '0805100a0a07050301%02x' 65)" "$GOOD"
    slm_sweep "log status 0" 1 "$(slm 0x10 10 10 7 5 3 0)" "$GOOD"
    slm_sweep "log status 2" 1 "$(slm 0x10 10 10 7 5 3 2)" "$GOOD"
    slm_sweep "log bad sum" 1 "0804100a0a0705030141" "$GOOD"
    slm_sweep "log high nibble 0" 1 "$(slm 0x00 10 10 7 5 3 1)" "$GOOD"
    slm_sweep "log high nibble 3" 1 "$(slm 0x30 10 10 7 5 3 1)" "$GOOD"
    slm_sweep "log low nibble e" 1 "$(slm 0x1e 10 10 7 5 3 1)" "$GOOD"
    slm_sweep "log low nibble f" 1 "$(slm 0x2f 10 10 7 5 3 1)" "$GOOD"
    # The wiki's markers around a stored dump. Their byte 2 has no high nibble,
    # so all three are dropped, if their status is 1 as here.
    slm_sweep "log the read key's markers" 1 "$(slm 0x09 10 10 10 10 10 1)" "$(slm 0x08 10 10 10 10 10 1)" "$(slm 0x07 10 10 10 10 10 1)" "$GOOD"
    SLM_ARGS="-l 5"
    SLM_FILTER="$D"'
/^colead-slm: received packet:/!d'
    slm_sweep "spew shows the record" 2 "$GOOD" "$(slm 0x21 1 2 3 4 5 1)"
    SLM_FLAGS=
    SLM_ARGS=
    SLM_FILTER=

    # Only a read of the one byte 0x10 is an announcement.
    emu_options period=5 burst=2 announce=1010,10
    emu_case "two announcements in one read are none" -- -d $C --samples 1
    emu_options period=5 burst=2 announce=ff,10
    emu_case "another byte is no announcement" -- -d $C --samples 1
    emu_options period=5 burst=2 announce=4010,10
    emu_case "an announcement behind a tail is none" -- -d $C --samples 1

    # A reply that does not fit: C fills its ten bytes with whatever comes, then
    # finds a bad record, and is back in step. So a missing reply costs ten
    # announcements.
    emu_options period=5 burst=12 "packet=-,$GOOD"
    emu_case "no reply takes ten announcements" -- -d $C --samples 1
    emu_options period=5 burst=7 "packet=0804100a0a,$GOOD"
    emu_case "a short reply takes five" -- -d $C --samples 1
    emu_options period=5 burst=2 "packet=${GOOD}ff,$GOOD"
    emu_case "a byte too many is dropped" -- -d $C --samples 2
    # That byte alone in a read is an announcement, as nothing tells it from
    # one, and the record it brings is a good one.
    emu_options period=5 burst=1 "packet=${GOOD}10,$(slm 0x10 10 10 8 0 0 1)"
    emu_case "a byte too many that reads as an announcement" -- -d $C --samples 2
    # An announcement lands ahead of a late reply, and the head of the record is
    # then one too far along. The reply's last byte is dropped on its own.
    emu_options period=20 burst=3 late=1:30
    emu_case "a late reply after an announcement is lost" -- -d $C --samples 1

    # What C has no end for: its limits are looked at behind a level alone
    # (protocol.c:177-180), and a read of the hung up port is a timeout
    # (protocol.c:199-201). 15 s is the bound's own time, so this case takes it,
    # and C the timeout's 18 s.
    emu_options burst=0
    emu_case "silent with samples" --oracle-bug "$NO_BOUND" --oracle-hangs --oxrok-fails "no bytes from the meter for 15 s" --timeout 18 --no-output -- -d $C --samples 1
    emu_case "silent with time" --oracle-bug "$NO_END" --oracle-hangs --timeout 2 --no-output -- -d $C --time 300ms
    emu_options period=5 burst=1 packet=-
    emu_case "no reply with time" --oracle-bug "$NO_END" --oracle-hangs --timeout 2 --no-output -- -d $C --time 300ms
    emu_options period=5 burst=2
    emu_case "wrong rate with time" --oracle-bug "$NO_END" --oracle-hangs --timeout 2 --no-output -- -d $C:serialcomm=9600/8n1 --time 300ms
    # No read comes back empty while these flow, so only the grace ends it. A free
    # stream, with its count of bytes taken out.
    emu_options period=5 announce=ff
    emu_case "time on a flood of no level" --oracle-bug "$NO_END" --oracle-hangs --timeout 3 --no-output --tx-filter 's/^bulk_in 82 [0-9]+ -> (ff)*$/bulk_in 82, some ff/' -- -d $C --time 300ms
    # The mark is a period after the last, and 1 s outlasts a stall that would
    # leave the record unread at the hangup.
    emu_options period=50 burst=1
    emu_case "unplug in a capture" --oxrok-fails "the port gone" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 8 -- -d $C --samples 2
    emu_case "interrupt with samples" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $C --samples 2
    emu_case "interrupt continuous" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $C --continuous

    # The port's own requests failing: both tools run on. With no line coding
    # the tty stays at 9600.
    emu_options period=50 burst=1 stall=line uart=9600/8n1
    emu_case "line coding stalled" -- -d $C --samples 1
    emu_options burst=0 stall=lines
    emu_case "lines stalled" --keep-header -- -d $C --show
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    C=colead-slm:conn=$EMU_TTY
}
