# A MASTECH MS6514 (emu/src/models/mastech-ms6514.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=mastech-ms6514
EMU_CDC=1
EMU_DRIVER=mastech-ms6514
EMU_FULL=$EMU_DRIVER
EMU_GCOV=mastech-ms6514
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# api.c:85 asks serial_stream_detect for a packet of two frames with a buffer of
# two frames, and serial.c:833 wants a buffer of two packets: every scan fails, so
# stock sigrok-cli finds no device and nothing behind the scan has ever run. The
# check is from 2012 (2546b05c; maxlen < packet_size * 2 at 23669c3d^), and the
# driver has had the same 36 and 36 from its pull request on (13b72fa9, 23669c3d,
# 2019). So the oracle carries a one-line patch of api.c:85, which asks for one
# frame (tools/oracle-patches/libsigrok-mastech-ms6514-scan.patch).

# A frame: main, aux (16 bit hex), then the bytes 10, 11, 12.
ms_f() {
    printf '6514000000%s%s01%s%s%s0c22380d0a' "$1" "$2" "$3" "$4" "$5"
}
# A stored frame: its index (16 bit hex), main, aux.
ms_s() {
    printf '651401%s%s%s310108080c22380d0a' "$1" "$2" "$3"
}

emu_cases() {
    emu_conns
    T1=00eb
    T2=00f1
    OK=$(ms_f $T1 $T2 01 08 08)
    NOISE19=$(printf '3f%.0s' $(seq 1 19))
    NOISE18=$(printf '3f%.0s' $(seq 1 18))
    BAD_END=6514$(printf '00%.0s' $(seq 1 16))
    SHORT_HEAD=65140000000000
    S0=$(ms_s 0000 00c8 00d2)
    S1=$(ms_s 0001 00c9 00d3)
    # A case's length of a stream isn't a fact, so a limit comes with a burst of
    # as many frames, and a time limit folds what came.
    N='/^bulk_in /d'
    CSV_LABEL="csv.c:187's g_malloc leaves the labels unset and :361 sets one only for a channel that came, so T1-T2, which no frame of this function names, is printed from what the heap holds at :437; oxrok's is empty, and its rows mix frames (a known limit, it warns)"
    CSV_COUNT="csv.c:191 counts every channel where :181 keeps the enabled ones, and :713 dumps at that count, so with T1-T2 off a row comes at every third reading, with one frame's T2 and the next frame's T1; oxrok writes each frame's row on frames showing T1 and T2 alone"
    PARSE_FIRST="sigrok-cli opens the port before it reads the parameters and exits with it open, so the open's lines and what the unit sent are in its .tx; oxrok refuses them first and opens nothing"
    NO_STORED="C ends a dump at a first live frame as here, with no word (protocol.c:156-160), so an empty memory and a live frame that crossed the request look alike; oxrok warns"
    SLOTS="protocol.c:163-164 takes the first two channels' enabled for the main and aux displays, whatever they show (:175-177), so -C gives another channel's reading; oxrok goes by the channel the reading names"
    LATE_LIMIT="protocol.c:215 and :244 look at the clock only after bytes came, so a time limit that ends before the first frame sends that frame; oxrok ends at the limit"
    TIME_HANG="protocol.c:215 returns at once on a read's timeout, ahead of the limit's check at :244, so a time limit never ends a capture on a silent unit; oxrok ends it"
    HUP="a port that hung up gives revents beyond G_IO_IN, and protocol.c:215 takes that as nothing to do, for ever (70 s tried); oxrok's failed read ends the capture"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    ANYKEY="sigrok-cli stops a --continuous at stdin's end (anykey.c), which here is at once; oxrok has no such stop, so the time limit ends it"
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'

    # The scan: the first frame ends it, and it closes the port.
    emu_options burst=1
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "show" --keep-header -- -d $K --show
    emu_case "no conn" --keep-header --no-output -- -d mastech-ms6514 --scan
    emu_case "no such node" --keep-header --no-output -- -d mastech-ms6514:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm as the default" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    # A line the unit is not on: nothing gets through.
    emu_case "serialcomm wrong baud" --keep-header --no-output -- -d $K:serialcomm=1200/8n1 --scan
    emu_case "serialcomm no such thing" --oracle-differs "$PARSE_FIRST" --stderr -- -d $K:serialcomm=fast --scan
    emu_options burst=0
    emu_case "scan silent" --keep-header --no-output -- -d $K --scan
    emu_options frame=3f3f3f$OK burst=1
    emu_case "scan noise then a frame" --keep-header -- -d $K --scan
    # Two frames' room: 18 bytes of noise leave room for the frame, 19 don't.
    emu_options frame=$NOISE18$OK burst=1
    emu_case "scan 18 bytes of noise" --keep-header -- -d $K --scan
    emu_options frame=$NOISE19$OK burst=1
    emu_case "scan 19 bytes of noise" --keep-header --no-output -- -d $K --scan
    emu_options frame=$BAD_END burst=1
    emu_case "scan a frame with no end" --keep-header --no-output -- -d $K --scan
    emu_options chunk=1 burst=1
    emu_case "scan one byte per transfer" --keep-header -- -d $K --scan
    emu_options stall=line burst=1
    emu_case "scan line coding stalled" --keep-header -- -d $K --scan
    emu_options stall=lines burst=1
    emu_case "scan line state stalled" --keep-header --no-output --tx-filter "$N" -- -d $K --scan
    emu_options in-stall=1
    emu_case "scan a stalled read" --keep-header --tx-filter '/^(bulk_in|ctl_out 02 01) /d' -- -d $K --scan

    # The options, which reach no device.
    emu_options burst=1
    emu_case "get data_source" -- -d $K --get data_source
    emu_case "get limit_samples" -- -d $K --get limit_samples
    emu_case "get limit_time" -- -d $K --get limit_time
    emu_case "get conn" --stderr --expect-fail -- -d $K --get conn
    emu_case "set data_source then get" -- -d $K -c data_source=Memory --get data_source
    emu_case "set limit then get" -- -d $K -c limit_samples=7 --get limit_samples
    emu_case "set data_source nothing" --stderr --expect-fail --filter "$A" -- -d $K -c data_source=Nothing --set
    emu_case "set data_source in lower case" --stderr --expect-fail --filter "$A" -- -d $K -c data_source=memory --set
    emu_case "set something else" --stderr --expect-fail -- -d $K -c samplerate=1000 --set

    # Captures: the readings, as each frame's bits have them.
    emu_options burst=1
    emu_case "samples 1" -- -d $K --samples 1
    emu_options burst=3
    emu_case "samples 3" -- -d $K --samples 3 -O analog
    emu_case "samples 3 csv" --oracle-differs "$CSV_COUNT" -- -d $K -C T1,T2 --samples 3 -O csv:header=false:time=false
    # A known limit of oxrok's csv with this driver, not a behaviour to keep: rows
    # by count mix frames here, and the driver warns. C crashes.
    emu_case "csv known limit" --oracle-bug "$CSV_LABEL" --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 -- -d $K --samples 3 -O csv:header=false:time=false
    emu_case "csv known limit warns" --oracle-differs "$CSV_COUNT" --stderr -- -d $K -C T1,T2 --samples 3 -O csv:header=false:time=false
    emu_options frame=$(ms_f $T1 $T2 01 08 09) burst=2
    # A frame names T1 twice and T2 never: each reading is a row of T1, under MAX's
    # label, with 0 for T2. oxrok alone, by hand: T2's label in libsigrok's csv is
    # unset (csv.c:187, :361) and comes from the heap at :437, a blank, "(null)" or
    # a crash from run to run, even with perturb, so no comparison is possible.
    emu_case "csv known limit, T1 and T1 MAX" --no-oracle-only -- -d $K -C T1,T2 --samples 2 -O csv:header=false:time=false
    emu_options burst=3
    # All 16 pairs of functions, T1 and T2 named as each pair has them.
    F=""
    for ms_a in 0 1 2 3; do
        for ms_m in 0 1 2 3; do
            F="$F${F:+,}$(ms_f $T1 $T2 01 0$ms_m 0$ms_a)"
        done
    done
    emu_options frame=$F burst=16
    emu_case "all functions" -- -d $K --samples 16
    # Units, hold, record, the sign, tenths, overload, the top value, and bits no
    # source names.
    F=$(ms_f 04d2 0929 00 00 00)
    for ms_x in \
        "$T1 $T2 01 08 08" "$T1 $T2 02 08 08" "$T1 $T2 03 08 08" \
        "$T1 $T2 41 08 08" "$T1 $T2 21 08 08" "$T1 $T2 fd 08 08" \
        "$T1 $T2 01 88 08" "$T1 $T2 01 08 88" "$T1 $T2 01 80 80" \
        "$T1 $T2 01 00 00" "0000 0000 01 88 88" "0000 0000 01 80 80" \
        "$T1 $T2 01 48 08" "$T1 $T2 01 08 48" "$T1 $T2 01 c8 c8" "$T1 $T2 01 40 40" \
        "ffff ffff 01 08 08" "ffff ffff 01 00 00" "$T1 $T2 01 34 34"; do
        # shellcheck disable=SC2086
        F="$F,$(ms_f $ms_x)"
    done
    emu_options frame=$F burst=20
    emu_case "all flags" -- -d $K --samples 20
    # tc2100's published frame, from a sibling unit: -14.1 C and an open input.
    emu_options frame=6514000000008d090c018188400002050d0a burst=1
    emu_case "sibling frame" -- -d $K --samples 1
    # A stored frame in a live capture counts as any other.
    emu_options frame=$S0 burst=1
    emu_case "stored frame in a live capture" -- -d $K --samples 1

    # Which channels: T1 and T2 name the display slots in C, main and aux.
    emu_options burst=1
    emu_case "channel T1" -- -d $K -C T1 --samples 1
    emu_case "channel T2" -- -d $K -C T2 --samples 1
    # Each tool reads T1-T2 as a range of numbered channels (parsers.c:81-91) and
    # refuses it, in its own words: no full case can hold both.
    emu_case "lib channel T1-T2 alone" --libsigrok --stderr --expect-fail --tx-filter "$N" -- -d $K -C T1-T2 --samples 1
    emu_case "channel T1-T2 alone" --no-oracle-only --stderr --expect-fail --tx-filter "$N" -- -d $K -C T1-T2 --samples 1
    emu_options frame=$(ms_f $T1 $T2 01 01 00) burst=1
    emu_case "channel T1 with T2 on main" --oracle-differs "$SLOTS" -- -d $K -C T1 --samples 1
    emu_case "channel T2 with T2 on main" --oracle-differs "$SLOTS" -- -d $K -C T2 --samples 1
    # T1 and T1 MAX: C gives T2 the aux display, a reading named T1.
    emu_options frame=$(ms_f $T1 $T2 01 00 01) burst=1
    emu_case "channel T2 with T1 and T1 MAX" --oracle-differs "$SLOTS" --no-output -- -d $K -C T2 --samples 1
    emu_options frame=$(ms_f $T1 $T2 01 02 00) burst=1
    emu_case "channel T1 with T1-T2 on main" --oracle-differs "$SLOTS" -- -d $K -C T1 --samples 1

    # What a read holds: the limit comes after a whole read, of 54 bytes at most.
    emu_options bunch=3 burst=3
    emu_case "samples 1 with 3 frames in a read" -- -d $K --samples 1
    emu_options bunch=4 burst=4
    emu_case "samples 1 with 4 frames in a read" -- -d $K --samples 1
    emu_case "samples 4 with 4 frames in a read" -- -d $K --samples 4
    emu_options chunk=1 burst=2
    emu_case "samples one byte per transfer" -- -d $K --samples 2

    # Out of step on the stream.
    emu_options frame=3f3f3f$OK burst=2
    emu_case "noise before each frame" -- -d $K --samples 2
    emu_options frame=$OK,$BAD_END,$OK burst=3
    emu_case "a frame with no end is skipped" -- -d $K --samples 2
    # A head with no end skips 18 bytes, into the frame after it.
    emu_options frame=$OK,$SHORT_HEAD,$OK,$OK burst=4
    emu_case "a false head takes the next frame with it" -- -d $K --samples 2

    # A time limit: how many frames came is the clock's.
    emu_options period=4 first=0
    emu_case "time" --filter '2,$d' --tx-filter "$N" --timeout 10 -- -d $K --time 300ms
    # C looks at the clock when bytes come, and only then.
    emu_options burst=1
    emu_case "time, then silence" --oracle-bug "$TIME_HANG" --oracle-hangs --timeout 5 -- -d $K --time 1s
    emu_case "time 1 ms" --oracle-differs "$LATE_LIMIT" --tx-filter "$N" --no-output --timeout 10 -- -d $K --time 1ms
    # sigrok-cli's --continuous ends at stdin's end, whatever the device does.
    emu_options period=4
    emu_case "continuous and a time limit" --oracle-differs "$ANYKEY" --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --continuous --time 2s

    # A unit that went quiet, or left. With a sample limit alone both wait.
    emu_options burst=1
    emu_case "lib samples, then silence" --libsigrok --hangs --timeout 3 -- -d $K --samples 2
    # A period long enough that the tool has read the burst when the event lands.
    emu_options burst=2 first=0 period=15
    emu_case "interrupt a silent unit" --oracle-bug "$SIGINT_BUG" --timeout 10 --interrupt model -- -d $K --samples 100
    emu_case "unplug in a capture" --oxrok-fails "the port gone" --oracle-bug "$HUP" --oracle-hangs --timeout 5 --unplug model -- -d $K --samples 100

    # The stored values: a1 goes out after the header, the live frame that follows
    # the dump ends the capture.
    emu_options still=1 burst=1
    emu_case "memory" -- -d $K -c data_source=Memory --samples 100
    emu_case "memory stderr" --stderr --filter 's/^oxrok: warn: oxrok::cli::pipeline: //' -- -d $K -c data_source=Memory --samples 100
    emu_case "lib memory, no limit" --libsigrok -- -d $K -c data_source=Memory --continuous
    emu_options still=1 burst=1 chunk=1
    emu_case "memory one byte per transfer" -- -d $K -c data_source=Memory --samples 100
    emu_options still=1 burst=1 stored=none
    emu_case "memory empty" --no-output -- -d $K -c data_source=Memory --samples 100
    emu_case "memory empty warns" --oracle-differs "$NO_STORED" --stderr -- -d $K -c data_source=Memory --samples 100
    emu_options still=1 burst=1 bunch=4
    emu_case "memory and the live frame in one read" -- -d $K -c data_source=Memory --samples 100
    emu_options still=2 burst=1 stored=$S0,$S1
    emu_case "memory limit within the dump" -- -d $K -c data_source=Memory --samples 2
    emu_options still=2 burst=1
    emu_case "lib memory, silence after the dump" --libsigrok --hangs --timeout 3 -- -d $K -c data_source=Memory --samples 100
    # What a stop in the parse leaves behind in one read: the parse goes on.
    emu_options still=2 burst=1 stored=$S0,$OK,$OK bunch=3
    emu_case "memory two live frames in one read" -- -d $K -c data_source=Memory --samples 100
    emu_options still=2 burst=1 stored=$S0,$OK,$S1 bunch=3
    emu_case "memory a stored frame after the live one" -- -d $K -c data_source=Memory --samples 100
    emu_case "lib memory, a stored frame after the live one" --libsigrok -- -d $K -c data_source=Memory --continuous
    emu_options still=2 burst=1 stored=$S0,$OK bunch=2
    emu_case "memory limit and the live frame in one read" -- -d $K -c data_source=Memory --samples 1
}

emu_conns() {
    K=mastech-ms6514:conn=$EMU_TTY
}
