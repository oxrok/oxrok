# An Atorch meter or load (emu/src/models/atorch.rs), with the CDC-ACM shell
# standing in for its Bluetooth link. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a serial model".
EMU_MODEL=atorch
EMU_CDC=1
EMU_DRIVER=atorch
EMU_FULL=$EMU_DRIVER
EMU_GCOV=atorch
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# A DC report: VVVVVV IIIIII CCCCCC EEEEEEEE TTTT, hex.
atorch_dc() {
    printf 'ff550102%s%s%s%s00000000000000%s00000000000000000000' "$1" "$2" "$3" "$4" "$5"
}

# A USB report: VVVVVV IIIIII CCCCCC EEEEEEEE D-D- D+D+ TTTT, hex.
atorch_usb() {
    printf 'ff550103%s%s%s%s%s%s%s0000000000000000000000000000' "$1" "$2" "$3" "$4" "$5" "$6" "$7"
}

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: (err|warn): oxrok::drivers::atorch(::protocol)?: /sr: atorch: /'
    # What oxrok adds after the error sigrok-cli names.
    N='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    LAG_BUG="receive_msg reads before it looks (protocol.c:151-157), so what a read holds beyond its first message waits for the next byte, and here none comes; oxrok takes all of a read"
    STALE_BUG="a scan's read can leave a report in buf, which only a probe's start clears (protocol.c:202-203), so a report from the scan is the capture's first frame; oxrok starts each capture empty, and shows what came in the capture"
    CHANNEL_BUG="parse_report_msg sends every channel whatever -C said (protocol.c:184 has no ch->enabled), and the analog output prints them; the man page has -C as the channels to be used, and csv and other drivers honour it; oxrok sends those asked for"
    QUIET_BUG="a limit is read only after a report (protocol.c:192), so a unit gone quiet holds the capture for ever; oxrok ends 3 s past the time limit"
    HUP_BUG="after an unplug a read gives nothing, as for a unit gone quiet, and no limit is read (protocol.c:192, 238-239), so it never ends; oxrok ends on the read's error"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # Every case's rate, a real unit's: the counts in scan= and burst= hold whatever
    # the time, but a report must not land in an open, whose flush takes it, and
    # two must not come in one read. Offers come fast at an open, with the kernel's
    # 16 reads to fill: 5 was too few, and at 25 a stall of 0.45 s in an open failed
    # a case. That margin is about 0.95 s here, at about 1 s per open with a report.
    P=period=50
    # Recordings, as in the model.
    R1=ff55010200011a00003c0004d40000002000006400000000002600590d363c00000000f0
    R2=ff55010200011a0000000004d40000002000006400000000002a00590e343c000000003f
    R3=$(atorch_dc 000123 000456 000789 00000abc 0019)
    AC=ff55010100090400000e0000040000000000006401f40085002f00000a093c0000000039
    REPLY=ff55020101000040
    COMMAND=ff55110203000000000052
    USB1=$(atorch_usb 000001 000001 000001 00000001 0001 0001 0001)

    # What a scan finds, in the first report it reads. Each scan's open has one
    # report here, and later opens as a case says.
    for m in dc usb; do
        emu_options $P meter=$m scan=1 burst=0
        emu_case "scan $m" --keep-header -- -d $A --scan
        emu_case "show $m" --keep-header -- -d $A --show
        emu_options $P meter=$m scan=1 burst=1
        emu_case "$m frames 1" -- -d $A --frames 1
        emu_options $P meter=$m scan=1 burst=3
        emu_case "$m frames 3 analog" -- -d $A --frames 3 -O analog
        emu_options $P meter=$m scan=1 burst=2
        emu_case "$m frames 2 csv" -- -d $A --frames 2 -O csv:header=false:time=false
    done
    emu_options $P meter=ac scan=1 burst=0
    emu_case "scan an ac meter" --keep-header --stderr --filter "$L" -- -d $A --scan
    emu_options $P packet=ff550107$(printf %s "$R1" | cut -c9-) scan=1 burst=0
    emu_case "scan an unknown type" --keep-header --stderr --filter "$L" -- -d $A --scan
    # Noise ahead of the unknown type's report, in one read: C's message reads the
    # type from the start of its buffer (protocol.c:216), so it says 55 where the
    # report has 01. A stderr difference can't be --oracle-differs, so C's line alone
    # is put right, ahead of the rule that gives oxrok's the same prefix.
    emu_options $P packet=aabb$AC scan=1 burst=0
    emu_case "scan noise and an ac meter" --keep-header --stderr --filter "s/^sr: atorch: Unrecognized device type \(0x0055\)/sr: atorch: Unrecognized device type (0x0001)/
$L" -- -d $A --scan
    # 2 x 10 s: the probe's own deadline is this path, and nothing else reaches it.
    emu_options $P scan=0 burst=0
    emu_case "scan silent" --keep-header --stderr --filter "$L" --timeout 14 -- -d $A --scan
    # What is not a report, ahead of one.
    emu_options $P packet=$REPLY,$R1 scan=2 burst=0
    emu_case "scan a reply first" --keep-header -- -d $A --scan
    emu_options $P packet=$COMMAND,$R1 scan=2 burst=0
    emu_case "scan a command first" --keep-header -- -d $A --scan
    emu_options $P packet=aabbcc$R1 scan=1 burst=0
    emu_case "scan noise first" --keep-header -- -d $A --scan
    emu_options $P packet=ff5577$R1 scan=1 burst=0
    emu_case "scan an unknown message first" --keep-header -- -d $A --scan
    emu_options $P packet=ffff55$R1 scan=1 burst=0
    emu_case "scan half a magic first" --keep-header -- -d $A --scan
    # A report cut across reads a period apart, as a real link does: inside the
    # magic, then inside the body. chunk= cuts nothing here, as a read comes after
    # all of a report has come.
    emu_options $P packet=$(printf %s "$R1" | cut -c1-4),$(printf %s "$R1" | cut -c5-40),$(printf %s "$R1" | cut -c41-) scan=3 burst=0
    emu_case "scan a report cut across reads" --keep-header -- -d $A --scan
    emu_options $P in-stall=1 scan=1 burst=0
    emu_case "scan a stalled read" --keep-header -- -d $A --scan
    # Nothing to scan. oxrok refuses a Bluetooth name (a unit test has its words).
    emu_options $P
    emu_case "conn bluetooth" --keep-header --no-output -- -d atorch:conn=bt/rfcomm/00-11-22-33-44-55 --scan
    emu_case "no such node" --keep-header --no-output -- -d atorch:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d atorch --scan
    emu_case "empty conn" --keep-header --stderr --expect-fail -- -d atorch:conn= --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d atorch --show
    # The link takes any line; the .tx shows it was asked.
    emu_options $P scan=1 burst=0
    emu_case "serialcomm the default" --keep-header -- -d $A:serialcomm=9600/8n1 --scan
    emu_case "serialcomm another" --keep-header -- -d $A:serialcomm=115200/8n1 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses
    # them first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $A:serialcomm=fast --scan
    emu_options $P stall=line scan=1 burst=0
    emu_case "line coding stalled" --keep-header -- -d $A --scan
    emu_options $P stall=lines scan=1 burst=0
    emu_case "control lines stalled" --keep-header -- -d $A --scan

    # What a limit reads back: nothing goes to the meter.
    emu_options $P scan=1 burst=0
    emu_case "get limit_frames" -- -d $A --get limit_frames
    emu_case "get limit_time" -- -d $A --get limit_time
    emu_case "get conn" --stderr --expect-fail -- -d $A --get conn
    emu_case "get limit_samples" --stderr --expect-fail -- -d $A --get limit_samples
    emu_case "get samplerate" --stderr --expect-fail -- -d $A --get samplerate
    emu_case "set limit_frames" -- -d $A -c limit_frames=3 --get limit_frames
    emu_case "set limit_time" -- -d $A -c limit_time=500 --get limit_time
    emu_case "set limit_frames 0" -- -d $A -c limit_frames=0 --get limit_frames
    emu_case "set limit_time 0" --stderr --expect-fail --filter "$N" -- -d $A -c limit_time=0 --set
    emu_case "set limit_samples" --stderr --expect-fail -- -d $A -c limit_samples=1 --set
    emu_case "samples" --stderr --expect-fail -- -d $A --samples 1

    # A capture: a frame per report, a channel apiece, read as C does.
    emu_options $P scan=1 burst=2
    emu_case "config limit_frames" --libsigrok --oxrok -- -d $A -c limit_frames=2 --continuous -O analog
    emu_case "frames csv with time" -- -d $A --frames 2 -O csv:header=false
    emu_case "channel V" --oracle-differs "$CHANNEL_BUG" -- -d $A -C V --frames 2
    emu_case "channel E" --oracle-differs "$CHANNEL_BUG" -- -d $A -C E --frames 2
    emu_case "channels V and T" --oracle-differs "$CHANNEL_BUG" -- -d $A -C V,T --frames 2
    emu_case "channel E csv" -- -d $A -C E --frames 2 -O csv:header=false:time=false
    emu_options $P meter=usb scan=1 burst=1
    emu_case "channel D plus" --oracle-differs "$CHANNEL_BUG" -- -d $A -C D+ --frames 1
    # Both read the dash as a range and refuse it, in their own words.
    emu_case "channel D minus" --expect-fail --no-output -- -d $A -C D- --frames 1

    # The readings' corners, in single precision and scaled by a double, on each
    # table. E's top bits show the float: 2^24 + 1 has no float of its own.
    C1=$(atorch_dc 000000 000000 000000 00000000 0000)
    C2=$(atorch_dc 000001 000001 000001 00000001 0001)
    C3=$(atorch_dc 000fff 001000 00ffff 01000001 0055)
    C4=$(atorch_dc ffffff ffffff ffffff ffffffff ffff)
    emu_options $P packet=$R1,$C1,$C2,$C3,$C4 scan=1 burst=4
    emu_case "dc corners" -- -d $A --frames 4 -O analog
    C1=$(atorch_usb 000000 000000 000000 00000000 0000 0000 0000)
    C3=$(atorch_usb 001388 000bb8 01f400 01000001 0258 0190 0023)
    C4=$(atorch_usb ffffff ffffff ffffff ffffffff ffff ffff ffff)
    emu_options $P packet=$USB1,$C1,$USB1,$C3,$C4 scan=1 burst=4
    emu_case "usb corners" -- -d $A --frames 4 -O analog

    # What is not a report, in a capture.
    emu_options $P packet=$R1,$REPLY,$R2,$COMMAND,$R3 scan=1 burst=4
    emu_case "a reply and a command between" -- -d $A --frames 2
    emu_options $P packet=$R1,aabbccdd$R2,ff5500$R3 scan=1 burst=2
    emu_case "noise between" -- -d $A --frames 2
    # A false magic takes a report's length with it: no check says otherwise,
    # and rightly, as some units fail the sum (the model's header).
    emu_options $P packet=$R1,ff5501$R2,$R3 scan=1 burst=2
    emu_case "a false magic swallows a report" -- -d $A --frames 2
    # A report of another type: only the scan's type counts.
    emu_options $P packet=$R1,$USB1 scan=1 burst=1
    emu_case "a report of another type" -- -d $A --frames 1 -O analog
    # A bad sum is taken, as C has no check.
    emu_options $P packet=$R1,$(printf %s "$R2" | sed 's/..$/00/') scan=1 burst=1
    emu_case "a bad sum" -- -d $A --frames 1
    # A report cut across reads, a period apart.
    emu_options $P packet=$R1,$(printf %s "$R2" | cut -c1-30),$(printf %s "$R2" | cut -c31-) scan=1 burst=2
    emu_case "a report cut across reads" -- -d $A --frames 1
    # Two reports in one read, and the limit in the first.
    emu_options $P packet=$R1,$R2$R3 scan=1 burst=1
    emu_case "two reports in one read" -- -d $A --frames 1
    # The second report of a read has to wait for a next byte in C
    # (protocol.c:151-157), and none comes.
    emu_case "two reports in one read both" --oracle-bug "$LAG_BUG" --oracle-hangs --timeout 6 -- -d $A --frames 2
    # A scan that read more than its report leaves the rest for the capture
    # (protocol.c:202-203 clears only at the probe's start).
    emu_options $P packet=$R1$R2,$R3 scan=1 burst=1
    emu_case "a report left from the scan" --oracle-differs "$STALE_BUG" -- -d $A --frames 1
    # A limit in time: C looks only after a report (protocol.c:192).
    emu_options $P scan=1 burst=1
    emu_case "time" -- -d $A --time 1
    emu_options $P scan=1 burst=0
    emu_case "time silent" --oracle-bug "$QUIET_BUG" --oracle-hangs --stderr --filter "$L" --timeout 8 -- -d $A --time 100
    # A library's stop.
    emu_options $P scan=1 burst=2
    emu_case "stop after 2 frames" --libsigrok -- -d $A --continuous -O analog --stop-after 10
    emu_options $P scan=1 burst=0
    emu_case "stop at the header" --libsigrok --no-output -- -d $A --continuous -O analog --stop-after 0
    # The debug line, for its form.
    emu_options $P scan=1 burst=0
    emu_case "the debug line" --stderr --filter 's/^oxrok: dbg: oxrok::drivers::atorch::protocol: /sr: atorch: /
s/^sr: \[[0-9:.]+\] /sr: /
/Atorch msg/!d' -- -l 4 -d $A --scan
    # An unplug: in a scan's wait, and in a capture.
    emu_options $P scan=0 burst=0
    emu_case "unplug in the scan" --unplug model --timeout 14 --keep-header --stderr --filter "$L" -- -d $A --scan
    emu_options $P scan=1 burst=1
    emu_case "unplug in a capture" --oxrok-fails "the port gone" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 7 --tx-filter '/^unplug$/q' -- -d $A --frames 5
    # oxrok's 30 s for a frame limit alone is for unit tests: the case would hold
    # the lock for it twice, to say what they say.
    emu_case "stops sending" --libsigrok --hangs --filter '2,$d' --timeout 6 -- -d $A --frames 5
    # A quiet point: the report has gone, the signal lands in the wait for the next.
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $A --continuous -O analog
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    A=atorch:conn=$EMU_TTY
}
