# A Center 309 / Voltcraft K204 thermometer (emu/src/models/center-3xx.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model". C registers two drivers from one directory, center-309 and
# voltcraft-k204, which differ in their vendor and model strings.
EMU_MODEL=center-3xx
EMU_CDC=1
EMU_DRIVER="center-309 voltcraft-k204"
EMU_FULL="center-309 voltcraft-k204"
EMU_GCOV=center-3xx
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"
# gcov's unreached lines of the driver: protocol.c:146-147 (packet_parse always
# returns SR_OK), :188-189 (a read of nothing on G_IO_IN alone: a gone tty polls with
# HUP too, so it takes the other branch), :224 and :227 (no sdi or no devc: the
# session always has them).

emu_cases() {
    emu_conns
    # sr_strerror's words against core::config_set's reason.
    A='s/^(Failed to set device option .limit_samples.: invalid argument): it cannot be 0\.$/\1./'
    # oxrok's log prefix, on its side only.
    P='s/^oxrok: (err|warn|info): oxrok::drivers::center_3xx: /sr: center-3xx: /'
    CHANNEL_BUG="handle_packet sends all four whatever -C said (protocol.c:158-166 has no ch->enabled), and the analog output prints them; csv honours it; oxrok sends those asked for"
    NO_RETRY="receive_data sets request_new_packet from the last read alone and sends A only when it is set (protocol.c:233, :239-242), with no expiry, so no whole packet means no more A and no end to a --samples capture; oxrok asks again, and fails after 10 s with no sample"
    STRAY_BUG="handle_new_data keeps what is left after a packet (protocol.c:207-208) and the search takes 02 and, 44 on, 03 (:48), so a stray 02 ahead of a reply whose byte 43 is 3 is a packet a byte out: T1 51.3 F, in place of 291 C; oxrok drops it ahead of a request"
    HUP_BUG="revents == G_IO_IN fails with HUP set (protocol.c:231), so a port gone never reads, its A fails at most once (:240, center_send's result dropped) and the source calls back for ever; oxrok fails the capture"
    LENGTH="handle_new_data takes any 45 bytes starting 02 and ending 03 (protocol.c:197, :48) with no look at what came with them, and says nothing; oxrok warns of a burst that is no multiple of 45"
    MODE_BUG="packet_parse reads bytes 7-14 in every mode (protocol.c:91-94) where the document has MAX, MIN and REL elsewhere, and says nothing; oxrok sends the same and warns"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # The first A goes at the source's first 500 ms; each next one 500 ms after the
    # last bytes.
    # 23.9, 25.6, -5.0 and 0.0, in Celsius.
    F=$(center_fr 80 00 00ef 0100 ffce 0000)
    # The next, so a sample's source shows.
    G=$(center_fr 80 00 0001 0002 0003 0004)
    # A request, for a .tx cut at the first. A cut there hides an overshoot of what
    # follows, so "a reply lost" and the driver's tests pin that.
    Q1='/^bulk_out 02 41/q'

    emu_options
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "scan k204" --driver voltcraft-k204 --keep-header -- -d $Z --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "no such node" --keep-header --no-output -- -d center-309:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d center-309 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses them
    # first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
    # The line coding reaches the wire: this one hears 4800 alone.
    emu_options uart=4800/8n1
    emu_case "serialcomm reaches the line" -- -d $K:serialcomm=4800/8n1 --samples 1
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    emu_options stall=line
    emu_case "line coding stalled" --keep-header -- -d $K --scan

    # --show, and what the driver has no answer to: config_get is NULL.
    emu_options
    emu_case "show" --keep-header -- -d $K --show
    emu_case "show k204" --driver voltcraft-k204 --keep-header -- -d $Z --show
    for key in limit_samples limit_time conn continuous samplerate; do
        emu_case "get $key" --expect-fail --stderr -- -d $K --get $key
    done
    emu_case "set limit_samples" --no-output -- -d $K -c limit_samples=2 --set
    emu_case "set limit_time" --no-output -- -d $K -c limit_time=100 --set
    emu_case "set limit_samples 0" --stderr --filter "$A" --expect-fail -- -d $K -c limit_samples=0 --set
    # That filter takes oxrok's own words off: here they are, oxrok alone.
    emu_case "set limit_samples 0, oxrok's words" --no-oracle-only --stderr --expect-fail -- -d $K -c limit_samples=0 --set
    emu_case "set continuous" --stderr --expect-fail -- -d $K -c continuous=1 --set
    emu_case "set samplerate" --stderr --expect-fail -- -d $K -c samplerate=1 --set

    # A capture.
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 3 analog" -- -d $K --samples 3 -O analog
    emu_case "samples k204" --driver voltcraft-k204 -- -d $Z --samples 1
    emu_case "samples csv" -- -d $K --samples 2 -O csv:header=false:time=false
    emu_case "config limit_samples" -- -d $K -c limit_samples=2 --samples 1
    # sigrok-cli stops at the end of stdin, before the first A, where oxrok goes on
    # till its signal: see "interrupt".
    emu_case "lib continuous stopped" --libsigrok -- -d $K --continuous --stop-after 8
    # What -C does: C's packets go out whatever it says, here only those asked for.
    emu_options "frames=$F"
    emu_case "channel T1" --oracle-differs "$CHANNEL_BUG" -- -d $K -C T1 --samples 1
    emu_case "channels T2 T4" --oracle-differs "$CHANNEL_BUG" -- -d $K -C T2,T4 --samples 1
    emu_case "channel T1 csv" -- -d $K -C T1 --samples 1 -O csv:header=false:time=false
    emu_case "no such channel" --stderr --filter 's/^Failed to set channels: invalid argument: //' --expect-fail -- -d $K -C T5 --samples 1
    # That filter takes oxrok's own words off: here they are, oxrok alone.
    emu_case "no such channel, oxrok's words" --no-oracle-only --stderr --expect-fail -- -d $K -C T5 --samples 1
    # Time decides the requests: kept up to the first.
    emu_options "frames=$F"
    emu_case "time" --filter '5,$d' --tx-filter "$Q1" -- -d $K --time 700ms
    # The first pass is at 500 ms, whatever the limit: it asks, and ends.
    emu_case "time shorter than the first pass" --no-output -- -d $K --time 1ms
    emu_options answers=0
    emu_case "time on a silent one" --no-output -- -d $K --time 1s

    # What a packet says: the values, in turn.
    emu_options "frames=$(center_fr 80 00 7fff 8000 ffff 0001)"
    emu_case "extremes" -- -d $K --samples 1 -O analog
    # Fahrenheit is bit 7 clear.
    emu_options "frames=$(center_fr 00 00 00ef 0100 ffce 0000)"
    emu_case "fahrenheit" -- -d $K --samples 1 -O analog
    # Byte 43, bit by bit, its high bits meaning nothing.
    L=$(center_list "80 00 00ef 0100 ffce 0001 00 00 00 00" 01 02 04 08 05 0f f0)
    emu_options "frames=$L"
    emu_case "resolution bits" -- -d $K --samples 7 -O analog
    # The status bits that say nothing to a sample.
    L=$(center_st 80 01 80 08 80 10 80 20 80 40 81 00 88 00 90 00 a0 00 c0 00)
    emu_options "frames=$L"
    emu_case "status bits" -- -d $K --samples 10
    # Over limit, by mode: a bit counts only in its mode's byte. Normal looks at
    # byte 39, MAX at 41, MIN at 42; REL's 40 and MIN/MAX's have no line.
    for mode in 80:normal 82:max 84:min 86:maxmin 90:rel 92:relmax; do
        m=${mode%:*}
        n=${mode#*:}
        OL="$(center_fr $m 00 00ef 0100 ffce 0001 05 00 00 00);$(center_fr $m 00 00ef 0100 ffce 0001 00 05 00 00);$(center_fr $m 00 00ef 0100 ffce 0001 00 00 05 00);$(center_fr $m 00 00ef 0100 ffce 0001 00 00 00 05);$(center_fr $m 00 00ef 0100 ffce 0001 f0 f0 f0 f0)"
        emu_options "frames=$OL"
        emu_case "over limit $n" -- -d $K --samples 5 -O analog
    done
    # The document's: T2 to T4 over limit, whole degrees.
    emu_options
    emu_case "the document's answer" -- -d $K --samples 1 -O analog
    emu_case "the document's answer csv" -- -d $K --samples 1 -O csv:header=false:time=false

    # How the bytes come. One read may take them all, and the limit is looked at
    # after a read: two packets for one A both go out at --samples 1.
    emu_options "frames=$F$G"
    emu_case "two packets in a read" -- -d $K --samples 1 -O analog
    emu_options chunk=1
    emu_case "samples one byte per transfer" -- -d $K --samples 2
    # A pause within a reply, shorter than the pass and then longer.
    emu_options "frames=$F" split=1:20:5
    emu_case "a reply in two parts" -- -d $K --samples 2
    emu_options "frames=$F" split=1:20:35
    emu_case "a reply with a pause past a pass" -- -d $K --samples 1
    emu_options "frames=$F" late=1:35
    emu_case "a reply later than a pass" -- -d $K --samples 1
    # Bytes ahead of a packet are passed over, and what is left after it waits.
    emu_options "frames=ffff$F"
    emu_case "junk ahead" -- -d $K --samples 1 -O analog
    emu_options "frames=${F}ffff;$G"
    emu_case "junk behind" -- -d $K --samples 2 -O analog
    emu_options "frames=00$F"
    emu_case "a zero ahead" -- -d $K --samples 1 -O analog
    # A burst that is no multiple of 45: oxrok says the sample may be wrong.
    emu_case "a zero ahead says so" --oracle-differs "$LENGTH" --stderr --filter "$P" -- -d $K --samples 1 -O analog
    # A byte inside a reply passes as a packet, and is told only by its length. Byte 43
    # is 3, so the window at 0 still ends in 3: in both tools T2 to T4 are wrong.
    MIXED=$(center_fr 80 00 0123 0456 0789 0abc 00 00 00 00 03)
    emu_options "frames=$(echo $MIXED | cut -c1-18)55$(echo $MIXED | cut -c19-90)"
    emu_case "a byte inside a reply" -- -d $K --samples 1 -O analog
    emu_case "a byte inside a reply says so" --oracle-differs "$LENGTH" --stderr --filter "$P" -- -d $K --samples 1 -O analog
    # The document has MAX's values at bytes 31 to 38: 40.0 here, and bytes 7 to 14
    # 25.0. Which one a meter displays no source settles, so both tools send 25.0, and
    # oxrok warns.
    MODE=$(center_fr 82 00 00fa 00fa 00fa 00fa | sed -E 's/^(.{62})0000000000000000/\1'"0190019001900190"'/')
    emu_options "frames=$MODE"
    emu_case "max mode reports bytes 7 to 14" -- -d $K --samples 1 -O analog
    emu_case "max mode says so" --oracle-differs "$MODE_BUG" --stderr --filter "$P" -- -d $K --samples 1 -O analog
    emu_options "frames=$(center_fr 90 00 00fa 00fa 00fa 00fa)"
    emu_case "rel mode says so" --oracle-differs "$MODE_BUG" --stderr --filter "$P" -- -d $K --samples 1 -O analog

    # No reply, or none whole. A is not sent again in C, so a --samples never ends:
    # pinned by a hang for C, and a second request 2 s on for oxrok.
    # The one that ends on the bound takes its 10 s, as nothing else shows the words of
    # a failed capture from the binary. `(41)+` folds its requests, since how many fit
    # in 10 s is the clock's: that hides an overshoot, which the tests' every 2 s pins.
    emu_options answers=0
    emu_case "silent" --oracle-bug "$NO_RETRY" --oracle-hangs --oxrok-fails "no answer from the thermometer for 10 s" --stderr --filter "$P" --timeout 14 --tx-filter 's/^bulk_out 02 (41)+ -> [0-9]+$/bulk_out 02 41 -> 1/' -- -d $K --samples 1
    emu_options "frames=~;$F"
    emu_case "a reply lost" --oracle-bug "$NO_RETRY" --oracle-hangs --timeout 5 -- -d $K --samples 1
    emu_options "frames=$(echo $F | cut -c1-40);$F"
    emu_case "part of a reply" --oracle-bug "$NO_RETRY" --oracle-hangs --timeout 5 -- -d $K --samples 1
    # 45 bytes with no end byte.
    emu_options "frames=$(echo $F | cut -c1-88)ff;$F"
    emu_case "wrong end byte" --oracle-bug "$NO_RETRY" --oracle-hangs --timeout 5 -- -d $K --samples 1
    emu_options "frames=ff$(echo $F | cut -c3-90);$F"
    emu_case "wrong start byte" --oracle-bug "$NO_RETRY" --oracle-hangs --timeout 5 -- -d $K --samples 1
    # An outage after a sample: asks go on, a warning at 10 s, and the meter's return
    # says so. It takes those 10 s, as nothing else pins the warning and its wiring.
    emu_options "frames=$F;~;~;~;~;~;~;$F"
    emu_case "an outage says so and goes on" --oracle-bug "$NO_RETRY" --oracle-hangs --stderr --filter "$P
s/(answers again, after )[0-9]+/\1N/" --timeout 16 -- -d $K --samples 2
    # One stray byte, a pass with no packet, after a good reply: no A from there on.
    emu_options "frames=${F}ee" split=1:45:5
    emu_case "a stray byte between replies" --oracle-bug "$NO_RETRY" --oracle-hangs --timeout 5 -- -d $K --samples 2
    # A reply past the expiry: whichever request it answers, the sample is the same.
    # The .tx goes up to the first A, as the second goes where C has none, and may
    # share its line: "a reply lost" pins how many go.
    emu_options late=1:150 "frames=$F"
    emu_case "a reply late" --tx-filter "s/^bulk_out 02 (41)+ -> [0-9]+\$/bulk_out 02 41 -> 1/
$Q1" -- -d $K --samples 1 -O analog
    # A stray 2 ahead of a packet whose byte 43 is 3: one starting a byte early.
    STRAY=$(center_fr 80 00 0123 0456 0789 0abc 00 00 00 00 03)
    emu_options "frames=${F}02;$STRAY"
    emu_case "a stray start byte makes a false packet" --oracle-differs "$STRAY_BUG" -- -d $K --samples 2 -O analog
    emu_options late=1:150 "frames=$F"
    emu_case "time ends a reply lost" --no-output -- -d $K --time 1s

    # A port gone: the source calls back with HUP set and never with G_IO_IN alone.
    # With A due, the write fails; with A out, there is nothing to do.
    emu_options answers=1
    emu_case "unplug with a request out" --oracle-bug "$HUP_BUG" --oracle-hangs --oxrok-fails "the port gone" --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 2
    # What it says: the OS's words are masked, and the line kept.
    emu_case "unplug says so" --oracle-bug "$HUP_BUG" --oracle-hangs --oxrok-fails "the port gone" --stderr --filter "$P
s/^(sr: center-3xx: Serial port read error: ).*/\1OS ERROR/" --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 2
    emu_options mark=1:5
    emu_case "unplug with a request due" --oracle-bug "$HUP_BUG" --oracle-hangs --oxrok-fails "the port gone" --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 2

    # A quiet point: the request goes unanswered, the signal lands in its wait.
    emu_options answers=1
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 8 --tx-filter '/^interrupt$/q' -- -d $K --continuous
    emu_options
}

# A packet: bytes 1 and 2, the four values, the four over limit bytes (00) and
# the resolution (00); bytes 3 to 6 as in Conrad's example.
center_fr() {
    echo "02$1$2""01020202$3$4$5$6""000000000000000000000000000000000000000000000000${7:-00}${8:-00}${9:-00}${10:-00}${11:-00}03"
}

# Packets differing in their resolution byte: "BYTE1 BYTE2 T1 T2 T3 T4 OL0 OL1 OL2
# OL3" and then each resolution.
center_list() {
    center_common=$1
    shift
    center_sep=""
    for center_r in "$@"; do
        printf '%s%s' "$center_sep" "$(center_fr $center_common $center_r)"
        center_sep=";"
    done
}

# Packets from pairs of bytes 1 and 2, on 23.9, 25.6, -5.0 and 0.1.
center_st() {
    center_sep=""
    while [ $# -ge 2 ]; do
        printf '%s%s' "$center_sep" "$(center_fr $1 $2 00ef 0100 ffce 0001)"
        center_sep=";"
        shift 2
    done
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=center-309:conn=$EMU_TTY
    Z=voltcraft-k204:conn=$EMU_TTY
}
