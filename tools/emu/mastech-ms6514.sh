# A MASTECH MS6514 (emu/src/models/mastech-ms6514.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=mastech-ms6514
EMU_CDC=1
EMU_DRIVER=mastech-ms6514
EMU_GCOV=mastech-ms6514
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# api.c:85 asks serial_stream_detect for a packet of two frames with a buffer of
# two frames, and serial.c:837 wants a buffer of two packets: every scan fails, so
# stock sigrok-cli finds no device and nothing behind the scan has ever run. It
# has been so since the driver came (23669c3d, 2019; serial.c had the check
# then). So all cases but the "stock" ones at the end run the C driver with that
# one argument changed to a frame in memory, by tools/emu/mastech-ms6514-scan.c,
# which is set on the exec of this model's tools only, through wrappers that
# SIGROK_CLI and EMU_LIBSIGROK_RUN name for the length of emu_cases. Neither oxrok
# nor the harness runs under it.
ms6514_scan_fixed() {
    MS_STOCK_CLI=$SIGROK_CLI
    MS_STOCK_RUN=$EMU_LIBSIGROK_RUN
    [ "$no_oracle" = 1 ] && return
    cc -shared -fPIC -o "$TMP/ms6514-scan.so" "$TOOLS/emu/mastech-ms6514-scan.c" -ldl 2>"$TMP/ms6514-scan.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    # The runner's own check compares what sigrok-cli and it load, so it is
    # built ahead of the wrapper.
    emu_libsigrok_run || { emu_fail "$EMU_MODEL: $EMU_LIBSIGROK_ERR"; return; }
    for w in cli:"$(command -v "$SIGROK_CLI")" run:"$EMU_LIBSIGROK_RUN"; do
        printf '%s\n' '#!/bin/sh' \
            "LD_PRELOAD=$TMP/ms6514-scan.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec ${w#*:} \"\$@\"" \
            >"$TMP/ms6514-${w%%:*}"
        chmod +x "$TMP/ms6514-${w%%:*}"
    done
    SIGROK_CLI=$TMP/ms6514-cli
    EMU_LIBSIGROK_RUN=$TMP/ms6514-run
}

# A frame: main, aux (16 bit hex), then the bytes 10, 11, 12.
ms_f() {
    printf '6514000000%s%s01%s%s%s0c22380d0a' "$1" "$2" "$3" "$4" "$5"
}
# A stored frame: its index (16 bit hex), main, aux.
ms_s() {
    printf '651401%s%s%s310108080c22380d0a' "$1" "$2" "$3"
}

emu_cases() {
    ms6514_scan_fixed
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
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses
    # them first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
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
    emu_case "samples 3 csv" -- -d $K -C T1,T2 --samples 3 -O csv:header=false:time=false
    # csv.c:187's g_malloc leaves the labels unset, and :362 sets one only for a
    # channel that came: T1-T2, which no frame of this function names, prints
    # what the heap holds at :445.
    emu_case "csv with a channel no frame names" --stderr --expect-fail --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 -- -d $K --samples 3 -O csv:header=false:time=false
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
    # sigrok-cli reads T1-T2 as a range of numbered channels (parsers.c:84).
    emu_case "channel T1-T2 alone" --stderr --expect-fail -- -d $K -C T1-T2 --samples 1
    emu_options frame=$(ms_f $T1 $T2 01 01 00) burst=1
    emu_case "channel T1 with T2 on main" -- -d $K -C T1 --samples 1
    emu_case "channel T2 with T2 on main" -- -d $K -C T2 --samples 1
    emu_options frame=$(ms_f $T1 $T2 01 02 00) burst=1
    emu_case "channel T1 with T1-T2 on main" -- -d $K -C T1 --samples 1

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
    emu_options period=4
    emu_case "time" --filter '2,$d' --tx-filter "$N" --timeout 10 -- -d $K --time 300ms
    # C looks at the clock when bytes come, and only then.
    emu_options burst=1
    emu_case "lib time, then silence" --libsigrok --hangs --timeout 4 -- -d $K --time 1s
    emu_case "time 1 ms" --tx-filter "$N" --timeout 10 -- -d $K --time 1ms

    # sigrok-cli's --continuous ends at stdin's end, whatever the device does.
    emu_options burst=1
    emu_case "continuous" --no-output --tx-filter "$N" -- -d $K --continuous

    # A unit that went quiet, or left.
    emu_options burst=1
    emu_case "lib samples, then silence" --libsigrok --hangs --timeout 3 -- -d $K --samples 2
    emu_options burst=2
    emu_case "unplug in a capture" --hangs --timeout 5 --unplug model -- -d $K --samples 100

    # The stored values: a1 goes out after the header, the live frame that follows
    # the dump ends the capture.
    emu_options still=1 burst=1
    emu_case "memory" -- -d $K -c data_source=Memory --samples 100
    emu_case "memory csv" -- -d $K -C T1,T2 -c data_source=Memory --samples 100 -O csv:header=false:time=false
    emu_case "lib memory, no limit" --libsigrok -- -d $K -c data_source=Memory --continuous
    emu_options still=1 burst=1 chunk=1
    emu_case "memory one byte per transfer" -- -d $K -c data_source=Memory --samples 100
    emu_options still=1 burst=1 stored=none
    emu_case "memory empty" --no-output -- -d $K -c data_source=Memory --samples 100
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

    # Stock libsigrok: no scan finds it.
    SIGROK_CLI=$MS_STOCK_CLI
    EMU_LIBSIGROK_RUN=$MS_STOCK_RUN
    emu_options burst=1
    emu_case "stock scan" --stderr -- -d $K --scan
    emu_case "stock capture" --stderr --expect-fail -- -d $K --samples 1
}

emu_conns() {
    K=mastech-ms6514:conn=$EMU_TTY
}
