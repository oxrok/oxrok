# The APPA 55II thermometer (emu/src/models/appa-55ii.rs), which only sends, on
# the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing
# a serial model".
#
# A case's frames are its burst= where it can be, so that what a .tx holds is a
# fact and not a margin: the thermometer goes silent when the tool has what it
# needs. A log's frames are as many as it holds.
EMU_MODEL=appa-55ii
EMU_CDC=1
EMU_DRIVER=appa-55ii
EMU_FULL=$EMU_DRIVER
EMU_GCOV=appa-55ii
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # How oxrok's log line and sr_strerror's read.
    F='s/^oxrok: [a-z]+: oxrok::drivers::appa_55ii(::protocol)?: /sr: appa-55ii: /'
    SHIFT_BUG="a frame with a wrong sum (protocol.c:245-247) or a broken sync (:237-239) is passed over and the pieces left are joined (:209-213), so each later record is read from the wrong bytes: 0.3 degrees where the log holds 20.3; oxrok warns and the records left are dropped"
    SHORT_BUG="appa_55ii_live_data reads at fixed places whatever the frame's size (protocol.c:79-81, :59, called from :251), so here T1 comes from the next frame's bytes 11-12, its 00 80 for no time shown, -32768, with its flags from that frame's content 9: the value goes by what the next frame holds; oxrok warns and makes no sample"
    SPIN_BUG="with no record due, appa_55ii_log_data_parse takes nothing (protocol.c:159) and at a full log_buf s is 0, so while (size > 0) in appa_55ii_log_data never ends (:208-216); oxrok drops those bytes and goes on to the log's end"
    NO_META_BUG="$SPIN_BUG, here as the metadata's frame was lost and the count stayed 0"
    SILENT_BUG="receive_data returns at a timeout, whose revents are 0, before it checks its limits (protocol.c:282-283, :311), so a time limit never fires on a device that is silent; oxrok's does"
    HUP_BUG="receive_data takes a hung up port for a timeout (protocol.c:282, revents != G_IO_IN), so its read error at :289-291 is never reached; oxrok's read fails and ends the capture"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    ANYKEY="sigrok-cli stops a --continuous at stdin's end (anykey.c), which here is at once, before the first frame; oxrok has no such stop, so the time limit ends it"
    G=$(appa_live)
    W=$(appa_live | sed 's/..$/00/')

    # The device, and what a scan takes.
    emu_options burst=1
    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "get data source" --keep-header -- -d $A --get data_source
    emu_case "get limit samples" --keep-header -- -d $A --get limit_samples
    emu_case "get limit time" --keep-header -- -d $A --get limit_time
    emu_case "set memory then get" --keep-header -- -d $A -c data_source=Memory --get data_source
    emu_case "set live then get" --keep-header -- -d $A -c data_source=Live --get data_source
    emu_case "set limit then get" --keep-header -- -d $A -c limit_samples=7 --get limit_samples
    emu_case "set time then get" --keep-header -- -d $A -c limit_time=7 --get limit_time
    # strcmp's refusal, so sent nothing. Each tool has its own words for it, and a
    # unit test holds oxrok's.
    emu_case "set no such source" --expect-fail --no-output -- -d $A -c data_source=memory --set
    emu_case "no conn" --keep-header --no-output -- -d appa-55ii --scan
    emu_case "no such node" --keep-header --no-output -- -d appa-55ii:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $A:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset" --keep-header --no-output -- -d $A:serialcomm=19200/8n1 --scan
    # sigrok-cli opens the port before it reads the setting, oxrok does not. No
    # frame comes, as how long C's open lasts would decide if one is in the .tx.
    emu_options burst=0
    emu_case "serialcomm no such" --tx-filter '/^ctl_out/d' --keep-header --no-output -- -d $A:serialcomm=fast --scan
    emu_case "scan silent" --keep-header --no-output -- -d $A --scan

    # 50 bytes are all a scan looks at, and one try per byte from the 25th, so
    # 25 bytes ahead of a frame are the most it passes over.
    emu_options burst=1 packet=$(appa_noise 25)$G
    emu_case "scan 25 bytes of noise" --keep-header -- -d $A --scan
    emu_options burst=1 packet=$(appa_noise 26)$G
    emu_case "scan 26 bytes of noise" --keep-header --no-output -- -d $A --scan
    # Any type will do, and a frame shorter than 25 bytes once 25 have come.
    emu_options burst=1 packet=$(appa_frame 19 "")$(appa_noise 20)
    emu_case "scan a short frame" --keep-header -- -d $A --scan
    emu_options burst=1 packet=$(appa_frame 19 "")
    emu_case "scan a short frame alone" --keep-header --no-output -- -d $A --scan
    # The sum of a frame longer than the 25 bytes a try has in hand falls past
    # what came, in a buffer C leaves unset: no device, unless that byte is luck.
    emu_options burst=1 packet=$(appa_frame 14 $(appa_noise 32))
    emu_case "scan a whole piece of log" --keep-header --no-output -- -d $A --scan
    # A log's first frames do, as a try reaches them.
    emu_options after=0 records=1 in-limit=56
    emu_case "scan in a log" --keep-header -- -d $A --scan
    emu_options burst=1 packet=$W
    emu_case "scan a wrong sum" --keep-header --no-output -- -d $A --scan
    emu_options burst=1 packet=$(appa_size_over)
    emu_case "scan a size over 32" --keep-header --no-output -- -d $A --scan

    # Live readings.
    emu_options burst=2 period=2
    emu_case "samples 2" -- -d $A --samples 2
    emu_case "samples 2 of t1" -- -d $A --samples 2 -C T1
    emu_case "samples 2 of t2" -- -d $A --samples 2 -C T2
    emu_case "samples 2 as csv" --filter '/^; /d' -- -d $A --samples 2 -O csv
    emu_options burst=1
    emu_case "samples 1" -- -d $A --samples 1
    # Through every flag the driver reads, with odd values.
    emu_options burst=12 period=2 reading=234/5/-56/5/1,23/4/-5/4/1,32767/25/32767/45/1,1/60/2/20/1,-32768/5/32767/5/1,-1/4/-1/5/21,1/5/2/5/5,1/5/2/5/9,1/5/2/5/d,1/5/2/5/2d,1/5/2/5/51,0/0/0/1/f0
    emu_case "every flag" -- -d $A --samples 12

    # A time limit, and a stream that runs on: the count is the time's, so the
    # .tx leaves out what came, and the output holds the first reading. C overruns
    # to the next frame and oxrok does not, which only this count shows. A period of
    # 2 free-runs and ends the session: see the model. The burst is more than a second
    # brings, and keeps a scan to its one frame: frames of a scan's free stream that
    # come after the next open's flush change the first reading.
    emu_options period=3 burst=1000
    T='/^bulk_in /d'
    emu_case "time on a stream" --filter '3,$d' --timeout 10 --tx-filter "$T" -- -d $A --time 1s

    # What a line can do to the frames.
    emu_options burst=2 period=2 packet=$(appa_noise 7)$G
    emu_case "noise before each frame" -- -d $A --samples 2
    emu_options burst=3 period=2 packet=$G,$W,$G
    emu_case "a wrong sum is skipped" -- -d $A --samples 2
    emu_options burst=3 period=2 packet=$G,$(appa_frame 77 ""),$G
    emu_case "unknown type" --stderr --filter "$F" -- -d $A --samples 2
    emu_options burst=2 period=2 chunk=1
    emu_case "one byte per transfer" -- -d $A --samples 2
    emu_options burst=1 in-stall=1
    emu_case "a stalled read at the scan" --keep-header -- -d $A --scan
    emu_options burst=1 stall=line
    emu_case "line coding stalled" --keep-header -- -d $A --scan
    emu_options burst=1 stall=lines
    emu_case "control lines stalled" --keep-header --no-output \
        --tx-filter '/^ctl_out 21 20 /d' -- -d $A --scan

    # The log.
    M="-c data_source=Memory"
    emu_options burst=1 records=3
    emu_case "log" -- -d $A $M --samples 100
    emu_case "log of t1" -- -d $A $M -C T1 --samples 100
    emu_case "log of t2" -- -d $A $M -C T2 --samples 100
    emu_options burst=1 records=3 gap=0
    emu_case "log all at once" -- -d $A $M --samples 100
    emu_options burst=1 records=0
    emu_case "log with no records" --no-output -- -d $A $M --samples 100
    emu_options burst=1 records=9 piece=7
    emu_case "log in small pieces" -- -d $A $M --samples 100
    emu_options burst=1 records=3 piece=1 gap=0
    emu_case "log byte by byte" -- -d $A $M --samples 100
    # Time stamps go to the debug log only.
    emu_options burst=1 records=1
    emu_case "log says the time" --stderr --filter "$F
s/^sr: \[[^]]*\] /sr: /
/Timestamp/!d" -- -l 4 -d $A $M --samples 100
    # A limit comes up after a whole piece, and the records it finished all go:
    # the second piece finishes two, the first has the one. in-limit holds the rest
    # back: the scan's frame counts too, then a live frame, the start, the metadata,
    # then 37 bytes a piece.
    emu_options burst=1 records=6 in-limit=106
    emu_case "log limit 1" -- -d $A $M --samples 1
    emu_options burst=1 records=6 in-limit=143
    emu_case "log limit 2 gets 3" -- -d $A $M --samples 2
    # The metadata says more than come: the end still ends it.
    # Its last piece's padding reads as a record, -0.1 degrees, under both tools.
    emu_options burst=1 records=2 claim=9
    emu_case "log says more than it holds" -- -d $A $M --samples 100
    emu_case "log says more than it holds says so" --oracle-differs "C says nothing of a log that ends short; oxrok warns" --stderr -- -d $A $M --samples 100
    # The live frames between are nobody's in this mode, nor a log in the other.
    emu_options burst=2 after=2 records=3
    emu_case "live frames do not count in memory" -- -d $A $M --samples 100
    emu_options burst=3 after=1 records=3
    emu_case "a log does not count in live" -- -d $A --samples 3
    # Nothing starts a log: a time limit still reads the stream and ends it.
    emu_options period=3 burst=1000
    emu_case "memory but no log" --no-output --timeout 10 --tx-filter "$T" -- -d $A $M --time 1s

    # A limit and the log's end in one read: C calls its stop twice (protocol.c:227
    # and :311-312), which shows neither in its output nor on its stderr.
    emu_options burst=1 records=1 piece=20 gap=0
    emu_case "log limit and end in one read" -- -d $A $M --samples 1
    # Nothing can come of a size over 32: the buffer fills and the capture ends
    # (protocol.c:306-309). oxrok warns on stderr, which this case does not compare.
    emu_options burst=2 period=2 packet=$G,$(appa_size_over)$G$G
    emu_case "a size over 32 ends it" --timeout 4 -- -d $A --samples 5
    emu_case "a size over 32 says so" --oracle-differs "protocol.c:306-309 ends the capture and says nothing; oxrok warns" --stderr --timeout 4 -- -d $A --samples 5

    # Where C is wrong.
    emu_options burst=2 packet=$G,$(appa_frame 00 0102)$G
    emu_case "a live frame too short" --oracle-differs "$SHORT_BUG" -- -d $A --samples 2
    emu_case "a live frame too short says so" --oracle-differs "$SHORT_BUG" --stderr -- -d $A --samples 2
    emu_options burst=1 records=6 bad=3
    emu_case "log with a piece broken" --oracle-differs "$SHIFT_BUG" -- -d $A $M --samples 100
    emu_case "log with a piece broken says so" --oracle-differs "$SHIFT_BUG" --stderr -- -d $A $M --samples 100
    emu_options burst=1 records=6 sync=3
    emu_case "log with a sync broken" --oracle-differs "$SHIFT_BUG" -- -d $A $M --samples 100
    emu_case "log with a sync broken says so" --oracle-differs "$SHIFT_BUG" --stderr -- -d $A $M --samples 100
    # A piece gone without a trace: nothing shows it before the end, so both tools
    # give the same shifted readings, and oxrok says at the end that the log is short.
    emu_options burst=1 records=6 drop=3
    emu_case "log with a piece missing" -- -d $A $M --samples 100
    emu_case "log with a piece missing says so" --oracle-differs "C says nothing of a log that ends short; oxrok warns" --stderr -- -d $A $M --samples 100
    emu_options burst=1 records=5 bad=1
    emu_case "log with its metadata broken" --oracle-bug "$NO_META_BUG" --oracle-hangs --timeout 4 --stderr -- -d $A $M --samples 100
    emu_options burst=1 records=8 claim=1
    emu_case "log with more than it says" --oracle-bug "$SPIN_BUG" --oracle-hangs --timeout 4 -- -d $A $M --samples 100
    emu_options mute=1
    emu_case "time on a silent device" --oracle-bug "$SILENT_BUG" --oracle-hangs --timeout 4 --no-output -- -d $A --time 300ms
    emu_options burst=1
    emu_case "unplug" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 6 -- -d $A --samples 5

    # A log no one starts waits for the user, under both tools, and a stop ends
    # oxrok's. So does a device that falls silent in C; oxrok gives that 10 s under a
    # sample limit in Live mode (SILENCE_BOUND, by choice, in its unit tests), and
    # the signal lands well before.
    emu_options mute=1
    emu_case "lib silent with no time limit" --libsigrok --hangs --timeout 3 --no-output -- -d $A --samples 3
    emu_case "interrupt on a silent device" --oracle-bug "$SIGINT_BUG" --no-output --interrupt model --timeout 8 -- -d $A --samples 3
    emu_options burst=1
    emu_case "interrupt waiting for a log" --oracle-bug "$SIGINT_BUG" --no-output --interrupt model --timeout 8 -- -d $A $M --samples 3
    emu_case "continuous and a time limit" --oracle-differs "$ANYKEY" -- -d $A --continuous --time 1s
    emu_options
}

emu_conns() {
    A=appa-55ii:conn=$EMU_TTY
}

# A whole frame in hex: its type and its content.
appa_frame() {
    h=$(printf '5555%s%02x%s' "$1" $((${#2} / 2)) "$2")
    s=0 r=$h
    while [ -n "$r" ]; do
        s=$((s + 0x$(printf %s "$r" | cut -c1-2)))
        r=$(printf %s "$r" | cut -c3-)
    done
    printf '%s%02x' "$h" $((s & 255))
}

# A plain live frame: 23.4 and -5.6 degrees.
appa_live() {
    appa_frame 00 0101ea00050100008000c8ff0501ea0005c8ff05
}

# N bytes that start no frame.
appa_noise() {
    i=0
    while [ "$i" -lt "$1" ]; do
        printf 3f
        i=$((i + 1))
    done
}

# A frame's start with a size that no frame has, and enough behind it for the
# driver to wait for: a line's bit error. No sum can be right.
appa_size_over() {
    printf '55550021'
    appa_noise 40
}
