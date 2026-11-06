# A KERN scale on its serial port (emu/src/models/kern-scale.rs), on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial
# model".
EMU_MODEL=kern-scale
EMU_CDC=1
EMU_DRIVER=kern-ew-6200-2nm
EMU_FULL=$EMU_DRIVER
EMU_GCOV=kern-scale
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/scale/kern.c"

emu_conns() {
    K=kern-ew-6200-2nm:conn=$EMU_TTY
}

# A packet as text, to go through printf: write % as %%. A leading - would be an
# option to printf.
kp() {
    case $1 in
    -*) emu_hex "\\055${1#-}\r\n" ;;
    *) emu_hex "$1\r\n" ;;
    esac
}

# kern_sweep NAME SAMPLES PACKET...: a capture of these packets, sent at once, so
# no count here depends on how long a tool listens. The scan takes one of its own.
# KERN_OPTS adds options, KERN_FLAGS=stderr compares what is logged instead.
kern_sweep() {
    _name=$1
    _samples=$2
    shift 2
    _list=$KP15
    for _p in "$@"; do
        _list=$_list,$_p
    done
    emu_options period=10 scan-burst=1 burst=1 group=$# packet=$_list $KERN_OPTS
    if [ "$KERN_FLAGS" = stderr ]; then
        emu_case "$_name" --stderr --filter "$F" -- -d $K --samples $_samples
    else
        emu_case "$_name" --keep-header -- -d $K --samples $_samples
    fi
}

emu_cases() {
    emu_conns
    KP15=$(kp '+00127.20 G S')
    KP14=$(kp '+0026.90 G S')
    KERN_OPTS=
    KERN_FLAGS=
    # oxrok's log prefix, on its side only.
    F='s/^oxrok: [a-z]+: oxrok::drivers::kern_scale(::protocol)?: /sr: kern-scale: /'
    HUP_BUG="kern_scale_receive_data reads only when revents is exactly G_IO_IN (protocol.c:121), so a port that hangs up is never read and only a limit could end the capture, which a sample limit never does; oxrok ends the capture"
    SILENT_BUG="kern_scale_receive_data has no bound (protocol.c:121-129), so a scale that stops sending keeps a sample limit open for ever; oxrok ends such a capture after 10 s with no packet"
    STEP_BUG="handle_new_data takes 15 bytes for any packet (protocol.c:89-92: offset += scale->packet_size), so at 14 bytes it eats the next packet's sign and loses that packet; oxrok takes a packet's own length and reads each"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # The scan: O1, then a packet within 3 s and 128 bytes.
    emu_options period=10 scan-burst=1 burst=0
    emu_case "show" --keep-header -- -d $K --show
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "no conn" --keep-header --no-output -- -d kern-ew-6200-2nm --scan
    emu_case "no such node" --keep-header --no-output -- -d kern-ew-6200-2nm:conn=/dev/ttyACM97 --scan
    emu_case "wrong baud" --keep-header --no-output -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "other stop bits" --keep-header -- -d $K:serialcomm=1200/8n1 --scan
    # C opens the port, raising the lines, and then finds the serialcomm bad; oxrok
    # refuses it before opening. The filter hides C's SET_CONTROL_LINE_STATE and
    # nothing else, as neither tool sends more.
    emu_case "no such serialcomm" --keep-header --no-output --tx-filter '/^ctl_out/d' -- -d $K:serialcomm=fast --scan
    emu_case "get limit_samples" --expect-fail --stderr -- -d $K --get limit_samples
    emu_case "get limit_time" --expect-fail --stderr -- -d $K --get limit_time
    emu_case "get conn" --expect-fail --stderr -- -d $K --get conn
    emu_case "frames refused" --expect-fail --stderr -- -d $K --frames 1
    emu_options period=10 scan-burst=0
    emu_case "never sends" --keep-header --no-output -- -d $K --scan
    # The manual's two answers, and none. E01 leaves it silent.
    emu_options period=10 ack=e01
    emu_case "e01" --keep-header --no-output -- -d $K --scan
    emu_options period=10 scan-burst=1 burst=1 ack=none
    emu_case "no answer to o1" --keep-header -- -d $K --samples 1
    # Later than the first read, within the 3 s.
    emu_options period=10 scan-burst=1 burst=1 late=30
    emu_case "o1 late" --keep-header -- -d $K --samples 1
    # A captured unit at the other line.
    emu_options period=10 scan-burst=1 burst=1 uart=9600/8o2
    emu_case "9600/8o2 as it can be set" --keep-header -- -d $K:serialcomm=9600/8o2 --samples 1
    emu_options

    # A capture, plain. The one case of a free stream, at about 1 s a packet: the
    # filter hides how the reads fell, as lengths and bytes vary by the time. The
    # second tool's O1 can meet the stream its scan started.
    emu_options period=50
    emu_case "free stream" --keep-header --tx-filter 's/^bulk_in 82 [0-9]+ -> .*$/bulk_in 82 STREAM/' -- -d $K --samples 1
    emu_options period=10 scan-burst=1 burst=1
    emu_case "samples 1" --keep-header -- -d $K --samples 1
    emu_case "limit_samples" --keep-header -- -d $K -c limit_samples=1 --time 5s
    emu_case "csv" -- -d $K --samples 1 -O csv:header=false
    emu_case "renamed" --keep-header -- -d $K -C Mass=load --samples 1
    emu_case "time" --keep-header -- -d $K --time 800ms
    emu_case "limit_time" --keep-header -- -d $K -c limit_time=800 --samples 5
    emu_options period=10 scan-burst=1 burst=2
    emu_case "samples 2" --keep-header -- -d $K --samples 2
    emu_case "samples 0" --expect-fail --stderr -- -d $K --samples 0 --time 800ms
    # The limit is checked once all of a read is out: a read holding two packets
    # sends both.
    emu_options period=10 scan-burst=1 burst=1 group=2
    emu_case "a read of two packets" --keep-header -- -d $K --samples 1
    emu_options

    # Each side of the packet's fields, in the 15-byte format.
    kern_sweep "units" 12 $(for _u in ' G' CT OZ LB OT DW GR TL MO to ' %%' PC; do kp "+00127.20$_u S"; printf ' '; done)
    # No such unit: C leaves the unit at 0.
    kern_sweep "no such unit" 4 $(kp '+00127.20 g S') $(kp '+00127.20G  S') $(kp '+00127.20   S') $(kp '+00127.20XX S')
    # S2 and S1. E is over or under the range, whatever the digits say.
    kern_sweep "status" 4 $(kp '+00127.20 G S') $(kp '+00127.20 G U') $(kp '+00127.20 G  ') $(kp '+00127.20 G E')
    kern_sweep "result" 4 $(kp '+00127.20 GLS') $(kp '+00127.20 GGU') $(kp '+00127.20 GHS') $(kp '+00127.20 GLE')
    kern_sweep "over range as a unit has it" 3 $(kp '+00000.00 G E') $(kp '-00000.00 G E') $(kp '+00000.00CT E')
    # As captured from a real unit, which zero fills.
    kern_sweep "captured values" 6 $(kp '+000000.0 G S') $(kp '+0000000 PC S') $(kp '+0000000  %% U') $(kp '-00000.04 G S') $(kp '+002014.8CT S') $(kp '+000636.0CT S')
    # What a number can be: where the point is decides the digits.
    kern_sweep "points" 7 $(for _v in '+000000.1' '+00000.12' '+0000.123' '+1.234567' '+.1234567' '+1234567.' '+12345678'; do kp "$_v G S"; printf ' '; done)
    # The manual's leading zeros as spaces, which no capture shows. strtod takes
    # no space behind a sign: C reads 0 there, with a point's digits.
    kern_sweep "spaces for zeros" 4 $(kp '+  127.20 G S') $(kp '-  127.20 G S') $(kp '   127.20 G S') $(kp ' 00127.20 G S')
    # strtod's other words, as the check looks at neither digit.
    kern_sweep "numbers" 8 $(kp '+1.5e1    G S') $(kp '+0x1f     G S') $(kp '+inf      G S') $(kp '-inf      G S') $(kp '+nan      G S') $(kp '+1e99     G S') $(kp '+abcdefgh G S') $(kp '+12\0005.7   G S')
    # A sign the check does not take: no packet there.
    kern_sweep "signs" 2 $(kp '*00127.20 G S') $(kp '-00127.20 G S') $(kp ' 00127.20 G S')
    # S1 and S2 the check does not take, at 15 bytes and 14: no packet there.
    kern_sweep "no such result or state" 1 $(kp '+00127.20 GXS') $(kp '+00127.20 G X') $(kp '+00001.00 G S')
    KERN_OPTS=pad=00
    kern_sweep "14 bytes no such result or state" 1 $(kp '+0127.20 GXS') $(kp '+0127.20 G X') $(kp '+0001.00 G S')
    KERN_OPTS=

    # The 14-byte format, which the same check takes. A byte behind each packet
    # gives the 15 bytes that C's step of 15 takes, so that it reads each.
    KERN_OPTS=pad=00
    kern_sweep "14 bytes units" 12 $(for _u in ' G' CT OZ LB OT DW GR TL MO to ' %%' PC; do kp "+0127.20$_u S"; printf ' '; done)
    kern_sweep "14 bytes status" 4 $(kp '+0026.90 G S') $(kp '+0026.90 G U') $(kp '+0026.90 GL ') $(kp '+0000.00 G E')
    kern_sweep "14 bytes points" 3 $(kp '+.123456 G S') $(kp '-0000.04 G S') $(kp '+123456. G S')
    KERN_OPTS=
    # What a real unit sends at 14 bytes: distinct packets with nothing between.
    # C reads each second one, 0.01 0.03 0.05 here.
    emu_options period=10 scan-burst=1 burst=1 group=6 packet=$KP15,$(kp '+0000.01 G S'),$(kp '+0000.02 G S'),$(kp '+0000.03 G S'),$(kp '+0000.04 G S'),$(kp '+0000.05 G S'),$(kp '+0000.06 G S')
    emu_case "14 bytes in a row" --oracle-differs "$STEP_BUG" --keep-header -- -d $K --time 800ms
    # The same loop wants 15 bytes to look at a packet: C holds a last one of 14
    # until a byte follows.
    emu_options period=10 scan-burst=1 burst=1 packet=$KP15,$KP14
    emu_case "14 bytes alone" --oracle-differs "$STEP_BUG" --keep-header -- -d $K --time 800ms
    # A scan's search looks at 15 bytes, so one 14-byte packet alone is not enough.
    emu_options period=10 scan-burst=1 burst=0 packet=$KP14
    emu_case "scan of one 14-byte packet" --keep-header --no-output -- -d $K --scan
    emu_options period=10 scan-burst=2 burst=0 packet=$KP14
    emu_case "scan of two 14-byte packets" --keep-header -- -d $K --scan

    # Noise on the line: the answer is noise too. The search gives up after 128.
    emu_options period=10 scan-burst=1 burst=2 noise=3f3f3f
    emu_case "noise before each packet" --keep-header -- -d $K --samples 2
    emu_options period=10 scan-burst=1 ack=none noise=$(printf '3f%.0s' $(seq 1 113))
    emu_case "noise up to the search" --keep-header -- -d $K --scan
    emu_options period=10 scan-burst=1 ack=none noise=$(printf '3f%.0s' $(seq 1 114))
    emu_case "noise past the search" --keep-header --no-output -- -d $K --scan
    # A cut packet as one capture has it, and the whole one after it.
    emu_options period=10 scan-burst=1 burst=1 noise=2b303030302ef0
    emu_case "a packet cut as captured" --keep-header -- -d $K --samples 1
    # A packet with its head lost, then a whole one.
    emu_options period=10 scan-burst=1 burst=1 group=2 packet=$KP15,$(kp '0127.20 G S'),$(kp '+00127.20 G S')
    emu_case "a head lost" --keep-header -- -d $K --samples 1
    emu_options period=10 ack=none scan-burst=1 packet=$(printf '3f%.0s' $(seq 1 15))
    emu_case "no packet" --keep-header --no-output -- -d $K --scan

    # The USB side.
    emu_options period=10 chunk=1 scan-burst=1 burst=1
    emu_case "one byte per transfer" --keep-header -- -d $K --samples 1
    # The kernel takes the stall for no error, and the line stays at 9600 on a
    # model that has no such line.
    emu_options stall=line uart=native period=10 scan-burst=1 burst=0
    emu_case "line coding stalled" --keep-header -- -d $K --scan
    emu_options stall=lines period=10 scan-burst=1 burst=0
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    emu_options in-stall=1 period=10 scan-burst=1 burst=0
    emu_case "a stalled read" --keep-header -- -d $K --scan
    emu_options

    # An unplug with the tty open. The mark is a period after the packet, and 1 s
    # outlasts a stall that would leave the packet unread at the hangup.
    emu_options period=50 scan-burst=1 burst=1
    emu_case "unplug" --oxrok-fails "the port gone" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --stderr --filter "$F" --timeout 8 -- -d $K --samples 5
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $K --samples 5
    # Silence at a sample limit: for ever in C. This case costs oxrok's 10 s: a seam
    # tests the times, this the message and the end.
    emu_options period=10 scan-burst=1 burst=1
    emu_case "lib silent at a sample limit" --libsigrok --hangs --timeout 3 -- -d $K --samples 5
    emu_case "silent ends at the bound" --oxrok-fails "no usable packet from the scale" --oracle-bug "$SILENT_BUG" --oracle-hangs --stderr --filter "$F" --timeout 13 -- -d $K --samples 5
    emu_options
}
