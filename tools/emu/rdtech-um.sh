# An RDTech UM24C, UM25C or UM34C power meter (emu/src/models/rdtech-um.rs),
# with the CDC-ACM shell standing in for its Bluetooth link. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=rdtech-um
EMU_CDC=1
EMU_DRIVER=rdtech-um
EMU_FULL=$EMU_DRIVER
EMU_GCOV=rdtech-um
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: (err|warn): oxrok::drivers::rdtech_um: /sr: rdtech-um: /'
    # What oxrok adds after the error sigrok-cli names.
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    STUCK_BUG="an incomplete reply, or two bytes a failed sync left, hold back every later poll (protocol.c:166-167 returns while buflen is set), so nothing more ever comes; oxrok drops them and asks again"
    HUP_BUG="a hung up port is not read, as its events are not plain G_IO_IN (protocol.c:350), and the failed poll is dropped (protocol.c:365), so it logs that for ever; oxrok ends the capture"
    SILENT_BUG="a meter that does not answer is polled for ever, and the frame limit is never reached (protocol.c:365); oxrok ends the capture after 5 s, where only that limit could"
    DASH_BUG="sigrok-cli reads the dash of a channel named D- as a range and fails on 'D' (parsers.c); oxrok refuses it as a range too, in its own words"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # What a scan finds, in each model's dump.
    for m in um24c um25c um34c; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $U --scan
        emu_case "show $m" --keep-header -- -d $U --show
        emu_case "$m frames 1" -- -d $U --frames 1
        emu_case "$m frames 3 analog" -- -d $U --frames 3 -O analog
        emu_case "$m frames 2 csv" -- -d $U --frames 2 -O csv:header=false:time=false
        emu_options model=$m bad-end=1
        emu_case "scan $m bad end" --stderr --filter "$L" --keep-header -- -d $U --scan
        emu_options model=$m chunk=1
        emu_case "scan $m a byte per transfer" --keep-header -- -d $U --scan
        emu_options model=$m in-stall=1
        emu_case "$m a stalled read" -- -d $U --frames 2
    done
    emu_options model=um24c id=0d4c
    emu_case "scan um34c id without its sum" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options model=um34c id=0963
    emu_case "scan um24c id with um34c ending" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options id=1234
    emu_case "scan an unknown id" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options noise=1:abcd
    emu_case "scan noise ahead" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options noise=1:ff
    emu_case "scan a power on byte" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options short=1:129
    emu_case "scan a short answer" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options answers=0
    emu_case "scan no answer" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options defer=1
    emu_case "scan a late answer" --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options
    emu_case "no such node" --keep-header --no-output -- -d rdtech-um:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d rdtech-um --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d rdtech-um --show
    # The link takes any line.
    emu_case "serialcomm the wiki's" --keep-header -- -d $U:serialcomm=9600/8n1 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses
    # them first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $U:serialcomm=fast --scan

    # What a limit reads back: nothing goes to the meter but the scan's poll.
    emu_case "get limit_frames" -- -d $U --get limit_frames
    emu_case "get limit_time" -- -d $U --get limit_time
    emu_case "get conn" --stderr --expect-fail -- -d $U --get conn
    emu_case "get samplerate" --stderr --expect-fail -- -d $U --get samplerate
    emu_case "set limit_frames" -- -d $U -c limit_frames=3 --get limit_frames
    emu_case "set limit_time" -- -d $U -c limit_time=500 --get limit_time
    emu_case "set limit_frames 0" -- -d $U -c limit_frames=0 --get limit_frames
    emu_case "set limit_time 0" --stderr --expect-fail --filter "$A" -- -d $U -c limit_time=0 --set
    emu_case "set limit_samples" --stderr --expect-fail -- -d $U -c limit_samples=1 --set
    emu_case "samples" --stderr --expect-fail -- -d $U --samples 1

    # A capture: a frame per poll, a channel apiece.
    emu_case "frames 5" -- -d $U --frames 5
    emu_case "config limit_frames" --libsigrok --oxrok -- -d $U -c limit_frames=2 --continuous -O analog
    emu_case "channel V" -- -d $U -C V --frames 2
    emu_case "channel I" -- -d $U -C I --frames 2
    emu_case "channel D plus" -- -d $U -C D+ --frames 2
    # sigrok-cli reads the dash as a range, whose end is missing.
    emu_case "channel D minus" --oracle-bug "$DASH_BUG" --stderr --expect-fail -- -d $U -C D- --frames 2
    emu_case "channel T" -- -d $U -C T --frames 2
    emu_case "channel E" -- -d $U -C E --frames 2
    emu_case "channels V and E" -- -d $U -C V,E --frames 2
    emu_case "channel E csv" -- -d $U -C E --frames 2 -O csv:header=false:time=false
    # The readings' corners, in single precision and scaled by a double.
    emu_options volts=0,1,12,512,2000,65535 amps=0,1,999,1000,3000,65535 dplus=0,1,330,65535 dminus=65535,330,1,0 celsius=0,1,85,65535 energy=0,1,16777217,4294967295
    emu_case "um24c corners" -- -d $U --frames 12 -O analog
    emu_options model=um25c volts=0,1,12,5123,20000,65535 amps=0,1,999,10000,30000,65535 dplus=0,1,330,65535 dminus=65535,330,1,0 celsius=0,1,85,65535 energy=0,1,16777217,4294967295
    emu_case "um25c corners" -- -d $U --frames 12 -O analog
    emu_options model=um34c volts=0,1,12,512,2000,65535 amps=0,1,999,1000,3000,65535 energy=33554435
    emu_case "um34c corners" -- -d $U --frames 6 -O analog
    # A time limit: how many polls that is depends on the clock, so both
    # are kept up to the first frame's poll.
    emu_options
    emu_case "time" --filter '3,$d' \
        --tx-filter '/^bulk_in 82 130 -> /{x;s/^/x/;/^xx/{x;q};x}' -- -d $U --time 350ms
    emu_case "stop after 2 frames" --libsigrok -- -d $U --continuous -O analog --stop-after 12
    emu_case "stop at the header" --libsigrok --no-output -- -d $U --continuous -O analog --stop-after 0

    # Answers gone wrong.
    emu_options bad-end=3
    emu_case "a bad end in a capture" --stderr --filter "$L" -- -d $U --frames 5
    emu_options bad-end=2
    emu_case "a bad end at once" --stderr --filter "$L" -- -d $U --frames 5
    emu_options model=um34c bad-end=3
    emu_case "a bad sum in a capture" --stderr --filter "$L" -- -d $U --frames 5
    emu_options noise=3:abcd
    emu_case "noise ahead of a frame" --stderr --filter "$L" -- -d $U --frames 3
    emu_case "noise ahead of a frame stdout" -- -d $U --frames 3
    emu_options noise=3:ab
    emu_case "a byte of noise ahead of a frame" -- -d $U --frames 3
    # A dump's own id ahead of the dump: nothing to resynchronize on.
    emu_options noise=3:0963
    emu_case "an id ahead of a frame" --stderr --filter "$L" -- -d $U --frames 3
    # An answer that stays incomplete holds back every later poll.
    emu_options short=2:60
    emu_case "a short answer" --oracle-bug "$STUCK_BUG" --oracle-hangs --timeout 4 -- -d $U --frames 2
    emu_options short=3:60
    emu_case "a short answer time limit" -- -d $U --time 300ms
    emu_options short=2:1
    emu_case "one byte of an answer" --oracle-bug "$STUCK_BUG" --oracle-hangs --timeout 4 -- -d $U --frames 2
    # What does not sync leaves two bytes behind, which count as accumulating.
    emu_options junk=2:130
    emu_case "junk for an answer" --oracle-bug "$STUCK_BUG" --oracle-hangs --timeout 4 -- -d $U --frames 2
    emu_options junk=2:300
    # How the reads cut it decides what C says: only the first read of a reply
    # syncs, so it hangs or fails a checksum. Either way no poll follows. oxrok
    # goes on, whatever the cut (unit tests).
    emu_case "more junk than the buffer" --libsigrok --no-output -- -d $U --time 300ms
    # A late answer comes with the next: both are taken in one read.
    emu_options defer=2
    emu_case "a late answer" -- -d $U --frames 2
    emu_case "a late answer past the limit" -- -d $U --frames 1
    emu_options defer=3
    emu_case "a late answer in a capture" -- -d $U --frames 3

    # Silence: it polls on, as often as the time lets it.
    emu_options answers=1
    emu_case "silent time limit" --no-output --tx-filter 's/^bulk_out 02 (f0)+ -> [0-9]+$/bulk_out 02 f0... -> N/' -- -d $U --time 300ms
    emu_case "silent frames" --oracle-bug "$SILENT_BUG" --oracle-hangs --timeout 8 --no-output --tx-filter 's/^bulk_out 02 (f0)+ -> [0-9]+$/bulk_out 02 f0... -> N/' -- -d $U --frames 1

    # The USB side misbehaving, the shell's options.
    emu_options chunk=7
    emu_case "seven bytes per transfer" -- -d $U --frames 2
    emu_options stall=line
    emu_case "line coding stalled" --keep-header -- -d $U --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $U --scan
    # The write path is this driver's own: 1 ms, and a timeout is no error.
    emu_options out-hold=1
    emu_tty_closing_wait 100
    emu_case "stops reading" --no-output --tx-filter '/^bulk_out 02 /d' -- -d $U --time 3s
    # Nor is it an error to report, which the case above cannot see: the kernel's
    # 16 writes are full after 1.6 s, and this finds no room from then on.
    emu_case "stops reading says nothing" --stderr --no-output --tx-filter '/^bulk_out 02 /d' -- -d $U --time 3s
    emu_tty_closing_wait 3000
    # An unplug: in a scan's wait, and in a capture.
    emu_options answers=0
    emu_case "unplug in the scan" --unplug model --timeout 8 --stderr --filter "$L" --keep-header -- -d $U --scan
    emu_options answers=2
    # -l 0: its two lines repeat in turn, as often as the time lets them, which
    # the .oracle's count of repeats does not fold. The next case pins them.
    emu_case "unplug in a capture" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -l 0 -d $U --frames 5
    emu_case "unplug in a capture what C says" --libsigrok --hangs --unplug model --timeout 4 --stderr --filter '3,$d' --tx-filter '/^unplug$/q' -- -d $U --frames 5
    # A quiet point: the poll goes unanswered, the signal lands in its wait.
    emu_options answers=3
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $U --continuous -O analog
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    U=rdtech-um:conn=$EMU_TTY
}
