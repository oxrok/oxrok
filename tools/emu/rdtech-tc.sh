# An RDTech TC66C USB power meter (emu/src/models/rdtech-tc.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=rdtech-tc
EMU_CDC=1
EMU_DRIVER=rdtech-tc
EMU_FULL=$EMU_DRIVER
EMU_GCOV=rdtech-tc
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: err: oxrok::drivers::rdtech_tc(::protocol)?: /sr: rdtech-tc: /'
    HANG_BUG="C asks nothing with no answer to its last request (protocol.c:213-214) or part of one in hand (protocol.c:211-212), and waits for ever; oxrok ends the capture after 5 s with no reply"
    HUP_BUG="receive_data reads only on exactly G_IO_IN (protocol.c:339), so a hung up port is never read, and it drops poll's result (protocol.c:352) and waits for ever; oxrok ends the capture on the read's error"
    CHANNEL_BUG="handle_poll_data sends every channel whatever -C said (protocol.c:266-274 has no ch->enabled); oxrok sends those asked for"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # What a scan reads of the reply: the name, the version, the serial.
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "show" --keep-header -- -d $K --show
    emu_options sn=7
    emu_case "scan a short serial number" --keep-header -- -d $K --show
    emu_options sn=4294967295
    emu_case "scan the largest serial number" --keep-header -- -d $K --show
    emu_options name=$(emu_hex 'TC\000\000') fw=$(emu_hex '2\000\000\000')
    emu_case "scan a name and version cut at a NUL" --keep-header -- -d $K --scan
    emu_options name=$(emu_hex 'XY12') fw=$(emu_hex '9.99')
    emu_case "scan another name" --keep-header -- -d $K --scan
    emu_options
    emu_case "no such node" --keep-header --no-output -- -d rdtech-tc:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d rdtech-tc --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d rdtech-tc --show

    # The log's dump of what the reply decrypts to, at spew only.
    emu_case "spew dumps the reply" --stderr --filter 's/^oxrok: spew: oxrok::drivers::rdtech_tc::protocol: /sr: rdtech-tc: /
/rdtech-tc: (check passed|[0-9a-f]{4}  )/!d
s/^sr: \[[^]]*\] /sr: /' --keep-header -- -l 5 -d $K --scan

    # The line: native USB does not mind.
    emu_case "serialcomm 9600" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "serialcomm 115200 8n2" --keep-header -- -d $K:serialcomm=115200/8n2 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses
    # them first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan

    # The options there are, and what is not.
    for key in limit_frames limit_time; do
        emu_case "get $key" -- -d $K --get $key
    done
    emu_case "get conn" --stderr --expect-fail -- -d $K --get conn
    emu_case "get samplerate" --stderr --expect-fail -- -d $K --get samplerate
    emu_case "set limit_frames" --no-output -- -d $K -c limit_frames=3 --set
    emu_case "set then read back" -- -d $K -c limit_frames=3 --get limit_frames
    emu_case "set limit_time" -- -d $K -c limit_time=250 --get limit_time

    # What a scan's reply must be. Under ECB a bit gone wrong ruins 16 bytes:
    # those holding each magic first, each block's data after.
    for at in 0 64 128; do
        emu_options flip=1:$at
        emu_case "scan magic wrong at $at" --stderr --filter "$L" --keep-header -- -d $K --scan
    done
    for at in 20 100 176; do
        emu_options flip=1:$at
        emu_case "scan checksum wrong at $at" --stderr --filter "$L" --keep-header -- -d $K --scan
    done
    # Bad data in a block both magics of which are good: a bit in the last
    # block of each is the checksum's own.
    emu_options flip=1:48
    emu_case "scan checksum field wrong" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options answers=0
    emu_case "scan no answer" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options cut=1:100
    emu_case "scan a short answer" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options cut=1:191
    emu_case "scan one byte short" --stderr --filter "$L" --keep-header -- -d $K --scan
    # What comes after the 192 bytes is for no one, and the reopen flushes it.
    emu_options extra=1:10
    emu_case "scan with more than asked" --keep-header -- -d $K --frames 1
    emu_options again=1
    emu_case "scan answered twice" --keep-header -- -d $K --frames 1
    emu_options

    # A capture: one getva per frame, and the six channels of a frame.
    emu_case "frames 1" -- -d $K --frames 1
    emu_case "frames 2" -- -d $K --frames 2
    emu_case "frames 5" -- -d $K --frames 5
    emu_case "frames from config" -- -d $K -c limit_frames=2 --frames 2
    emu_case "frames analog" --keep-header -- -d $K --frames 2 -O analog
    emu_case "frames csv" -- -d $K --frames 3 -O csv:header=false:time=false
    emu_case "channels V and I" --oracle-differs "$CHANNEL_BUG" -- -d $K -C V,I --frames 2
    emu_case "channels E0 only" --oracle-differs "$CHANNEL_BUG" -- -d $K -C E0 --frames 2
    emu_case "channels D+ csv" -- -d $K -C D+ --frames 2 -O csv:header=false:time=false
    # sigrok-cli reads the - of a channel's name as a range.
    emu_case "channel D-" --stderr --expect-fail --filter "s/^Failed to set channels: invalid argument: Invalid channel range 'D-'.\$/Invalid channel 'D'./" -- -d $K -C D- --frames 2
    emu_case "samples is not a limit here" --stderr --expect-fail -- -d $K --samples 2
    # hwdriver.c's check_key takes a 0 for frames, and refuses it for time.
    emu_case "config limit_frames 0" -- -d $K -c limit_frames=0 --frames 1
    Z='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./
/^sr: hwdriver: Cannot set/d'
    emu_case "config limit_time 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_time=0 --frames 1

    # A time limit: how many polls that is depends on the clock, so both are
    # kept up to the first of the capture.
    emu_case "time" --filter '7,$d' --tx-filter '/^bulk_in 82 192 -> /{x;s/^/x/;/^xx/{x;q};x}' -- -d $K --time 500ms

    # The values: a float of the 32 bits, then C's scale in single precision.
    emu_options v=0,1,10000,99999,16777216,16777217,4294967295 i=0,1,100000,123456,16777217,3,4294967295 \
        dp=0,1,10,100,333,4294967295,7 dm=1,0,999,1000,12345,4294967295,7 \
        e0=0,1,999,1000,16777217,4294967295,7 e1=1,0,1001,123456789,3,4294967295,7
    emu_case "values in turn" -- -d $K --frames 6 -O csv:header=false:time=false
    emu_case "values in turn analog" -- -d $K --frames 6

    # analog.c:412 multiplies by a float power of ten: 1.5000001 kV, not 1.5.
    emu_options v=15000000
    emu_case "1500 volts" -- -d $K -C V --frames 1 -O csv:header=false:time=false
    emu_case "1500 volts analog" --oracle-differs "$CHANNEL_BUG" -- -d $K -C V --frames 1

    # Replies gone wrong in a capture: the frames before stay, the capture
    # ends with a message and sigrok-cli's exit status 0.
    emu_options flip=2:0
    emu_case "capture magic wrong" --stderr --filter "$L" -- -d $K --frames 2
    emu_case "capture magic wrong output" --no-output -- -d $K --frames 2
    emu_options flip=3:100
    emu_case "capture checksum wrong" --stderr --filter "$L" -- -d $K --frames 5
    emu_case "capture checksum wrong output" -- -d $K --frames 5
    # A reply twice: the second is a frame of its own, without a request.
    emu_options again=2
    emu_case "capture a reply twice" -- -d $K --frames 2
    # 256 bytes fit C's buffer: the rest of a reply waits for the next read.
    emu_options extra=2:192
    emu_case "capture garbage after a reply" --stderr --filter "$L" -- -d $K --frames 3
    emu_options

    # A time limit ends what would otherwise wait for ever.
    emu_options answers=1
    emu_case "silence with a time limit" --no-output -- -d $K --time 300ms
    emu_options cut=2:100
    emu_case "short reply with a time limit" --no-output -- -d $K --time 300ms
    emu_options extra=2:10
    emu_case "extra bytes with a time limit" --filter '7,$d' --tx-filter '/^bulk_in 82 202 -> /q' -- -d $K --time 300ms

    # C waits for ever for a reply to its last request (protocol.c:213-214),
    # or for the rest of it, or asks no more with a part in its buffer
    # (protocol.c:211-212).
    emu_options answers=1
    emu_case "silence no limit" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 8 --stderr --filter "$L" -- -d $K --frames 2
    emu_options cut=2:100
    emu_case "short reply no limit" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 8 --stderr --filter "$L" -- -d $K --frames 2
    emu_options extra=2:10
    emu_case "extra bytes no limit" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 8 --stderr --filter "$L" -- -d $K --frames 2

    # The USB side misbehaving, the shell's options.
    emu_options chunk=1
    emu_case "one byte per transfer" -- -d $K --frames 2
    emu_options chunk=100
    emu_case "chunks of 100" -- -d $K --frames 2
    emu_options in-stall=1
    emu_case "a stalled read" -- -d $K --frames 2
    emu_options stall=line
    emu_case "line coding stalled" --keep-header -- -d $K --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # An unplug during the scan, and during a capture.
    emu_options answers=0
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $K --scan
    emu_options answers=2
    emu_case "unplug in a capture" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 8 --stderr --filter "$L" --tx-filter '/^unplug$/q' -- -d $K --frames 5
    # A quiet point: the third request goes unanswered, the signal lands in
    # its wait.
    emu_options answers=2
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $K --continuous
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=rdtech-tc:conn=$EMU_TTY
}
