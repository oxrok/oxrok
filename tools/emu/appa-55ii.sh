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
EMU_GCOV=appa-55ii
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # How oxrok's log line and sr_strerror's read.
    F='s/^oxrok: [a-z]+: oxrok::drivers::appa_55ii(::protocol)?: /sr: appa-55ii: /'
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
    emu_case "set no such source" --expect-fail --stderr -- -d $A -c data_source=memory --set
    emu_case "no conn" --keep-header --no-output -- -d appa-55ii --scan
    emu_case "no such node" --keep-header --no-output -- -d appa-55ii:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $A:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset" --keep-header --no-output -- -d $A:serialcomm=19200/8n1 --scan
    emu_case "serialcomm no such" --tx-filter '/^ctl_out/d' --keep-header --no-output -- -d $A:serialcomm=fast --scan
    emu_options burst=0
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
    # .tx says only that bytes came, and the output its first line.
    emu_options period=2
    T='s/^bulk_in 82 [0-9]+ -> .*$/bulk_in 82 N -> .../'
    emu_case "time on a stream" --filter '2,$d' --timeout 10 --tx-filter "$T" -- -d $A --time 1s

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
    emu_options burst=1 records=2 claim=9
    emu_case "log says more than it holds" -- -d $A $M --samples 100
    # The live frames between are nobody's in this mode, nor a log in the other.
    emu_options burst=2 after=2 records=3
    emu_case "live frames do not count in memory" -- -d $A $M --samples 100
    emu_options burst=3 after=1 records=3
    emu_case "a log does not count in live" -- -d $A --samples 3
    # Nothing starts a log: a time limit still reads the stream and ends it.
    emu_options period=2
    emu_case "memory but no log" --no-output --timeout 10 --tx-filter "$T" -- -d $A $M --time 1s

    # What C does at the edges, to pin once seen.
    emu_options burst=1 records=1 piece=20 gap=0
    emu_case "log limit and end in one read" -- -d $A $M --samples 1
    emu_options burst=2 packet=$G,$(appa_frame 00 0102)$G
    emu_case "a live frame too short" -- -d $A --samples 3
    emu_options burst=2 period=2 packet=$G,$(appa_size_over)$G$G
    emu_case "a size over 32 ends it" --timeout 4 -- -d $A --samples 5
    emu_options burst=1 records=5 bad=1
    emu_case "log with its metadata broken" --hangs --timeout 4 --no-output -- -d $A $M --samples 100
    emu_options burst=1 records=8 claim=1
    emu_case "log with more than it says" --hangs --timeout 4 -- -d $A $M --samples 100
    emu_options burst=1 records=6 bad=3
    emu_case "log with a piece broken" -- -d $A $M --samples 100
    emu_options mute=1
    emu_case "time on a silent device" --hangs --timeout 4 --no-output -- -d $A --time 300ms
    emu_case "silent with no limit" --hangs --timeout 4 --no-output -- -d $A --samples 3
    emu_options burst=1
    emu_case "unplug" --hangs --unplug model --timeout 6 -- -d $A --samples 5
    emu_case "continuous" --no-output -- -d $A --continuous
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
