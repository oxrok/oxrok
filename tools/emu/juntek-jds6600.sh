# A Juntek JDS6600 signal generator (emu/src/models/juntek-jds6600.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=juntek-jds6600
EMU_CDC=1
EMU_DRIVER=juntek-jds6600
EMU_FULL=$EMU_DRIVER
EMU_GCOV=juntek-jds6600
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # What oxrok says for what sigrok-cli calls "not applicable" (api.c turns
    # every failure into SR_ERR_NA). On oxrok's wording alone, so a change in
    # either shows.
    E='s/^(Failed to get [^ ]+: )device error: .*$/\1not applicable/
s/^(Failed to set device option [^ ]+: )device error: .*$/\1not applicable./
s/^(Failed to set device option [^ ]+: )invalid argument: no such pattern\.$/\1not applicable./'
    CRASH_BUG="a frequency of fewer than two words gets NULL to a parser: protocol.c:602-603 strtof(NULL), 608-609 strtoul(NULL); oxrok finds no device"
    HANG_BUG="no timeout_ms after the first request (protocol.c:442-450), so it reads on for ever; oxrok ends after 1.2 s"
    NAN_BUG="a NaN passes every limit (protocol.c:750-753, 770-773, 792-795, 813, 1369-1372) and goes out as 'nan' or '-nan', from an infinite phase too; oxrok refuses it"
    ZERO_BUG="-0.0 passes the limits and %.0f sends it as -0 (protocol.c:752, 792, 813-816); oxrok sends 0"
    ORDER_BUG="sigrok-cli sets -c's options in hash table order (session.c:700-701), here the output before its amplitude; oxrok sets it last"
    LOOSE_BUG="sigrok-cli takes what is no value (session.c's opt_to_gvar, strtod) and sends 0 V; oxrok refuses it and sends nothing"
    SCAN_OPT_BUG="the core drops a scan option the driver does not list, so it finds nothing and exits 0; oxrok refuses it"
    PARTIAL_BUG="it switches the outputs, then fails on the limit (session.c:865); oxrok holds enabled=on until the rest was accepted"

    # A scan: type and serial number, enables, per channel waveform, frequency,
    # amplitude, offset, duty cycle, then the phase. 14 requests, no write.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $J --scan
    emu_case "show" --keep-header -- -d $J --show
    emu_case "show CH1" --keep-header -- -d $J -g CH1 --show
    emu_case "show CH2" --keep-header -- -d $J -g CH2 --show
    emu_case "show ch1 in lower case" --keep-header -- -d $J -g ch1 --show
    emu_case "show no such group" --keep-header --stderr --expect-fail -- -d $J -g CH3 --show
    emu_options setters=0 r00=15
    emu_case "show CH1 of a 15 MHz model" --keep-header -- -d $J -g CH1 --show
    emu_options setters=0 r00=60x
    emu_case "type with trailing text" --keep-header -- -d $J -g CH1 --show
    for jt in 0 abc "" 4294967296 99999999999999999999; do
        emu_options setters=0 r00=$jt
        emu_case "type '$jt' is no device" --keep-header --no-output -- -d $J --scan
    done
    emu_options setters=0 r00=4294967295
    emu_case "type at the top of 32 bits" --keep-header -- -d $J -g CH1 --show
    emu_options setters=0 r01=
    emu_case "empty serial number" --keep-header -- -d $J --scan
    emu_options setters=0 r20=0,1 r21=16 r22=160
    emu_case "show CH1 another state" --keep-header -- -d $J -g CH1 --show
    emu_case "show CH2 another state" --keep-header -- -d $J -g CH2 --show

    emu_options setters=0
    emu_case "no such node" --keep-header --no-output -- -d juntek-jds6600:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d juntek-jds6600 --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d juntek-jds6600 --show
    # The line: scanopts lists conn alone, so serialcomm never reaches the driver.
    emu_case "serialcomm as the default" --oracle-differs "$SCAN_OPT_BUG" --expect-fail --keep-header --stderr -- -d $J:serialcomm=115200/8n1 --scan
    emu_case "serialcomm on another line" --oracle-differs "$SCAN_OPT_BUG" --expect-fail --keep-header --stderr -- -d $J:serialcomm=9600/8n1 --scan

    # What --get asks for, in both channels.
    emu_options setters=0
    emu_case "get conn" -- -d $J --get conn
    emu_case "get phase" -- -d $J --get phase
    emu_case "get enabled device wide" --stderr --expect-fail -- -d $J --get enabled
    for jg in CH1 CH2; do
        for jkey in enabled pattern output_frequency amplitude offset output_duty_cycle; do
            emu_case "get $jg $jkey" -- -d $J -g $jg --get $jkey
        done
        emu_case "get $jg phase" --stderr --expect-fail -- -d $J -g $jg --get phase
    done

    # How values read: the parsers' scales, caps and single precision.
    emu_options setters=0 r20=0,0
    emu_case "get enabled both off" -- -d $J -g CH2 --get enabled
    emu_options setters=0 r20=2,0
    emu_case "get enabled any number is on" -- -d $J -g CH1 --get enabled
    for jf in 100,0 100,1 100,2 100,3 100,4 1234567891,0 1,0 12.5,0 ; do
        emu_options setters=0 r23=$jf
        emu_case "get frequency $jf" -- -d $J -g CH1 --get output_frequency
    done
    for ja in 0 1 1234 20000 12.5 -500; do
        emu_options setters=0 r25=$ja
        emu_case "get amplitude $(jm $ja)" -- -d $J -g CH1 --get amplitude
    done
    for jo in 0 1 1023 1000 1999 2000 9999; do
        emu_options setters=0 r27=$jo
        emu_case "get offset $jo" -- -d $J -g CH1 --get offset
    done
    for jd in 0 1 333 1000 1500; do
        emu_options setters=0 r29=$jd
        emu_case "get duty $jd" -- -d $J -g CH1 --get output_duty_cycle
    done
    for jp in 0 1 900 3599 -900; do
        emu_options setters=0 r31=$jp
        emu_case "get phase $(jm $jp)" -- -d $J --get phase
    done
    for jw in 0 16 101 160; do
        emu_options setters=0 r21=$jw
        emu_case "get waveform $jw" -- -d $J -g CH1 --get pattern
    done
    emu_options setters=0 r21=abc
    emu_case "get waveform that is no number" -- -d $J -g CH1 --get pattern
    emu_options setters=0 r21=3x
    emu_case "get waveform with trailing text" -- -d $J -g CH1 --get pattern

    emu_options setters=0 r25=nan
    emu_case "get amplitude nan" -- -d $J -g CH1 --get amplitude
    emu_options setters=0 r25=inf
    emu_case "get amplitude inf" -- -d $J -g CH1 --get amplitude

    # A value that ends in a period of its own, after the frame's.
    emu_options setters=0 r20=0,1.
    emu_case "two periods on the enables" -- -d $J -g CH1 --get enabled
    emu_options setters=0 r23=100,0.
    emu_case "two periods on a frequency" -- -d $J -g CH1 --get output_frequency

    # Replies a scan cannot use: the rest of its requests still go.
    emu_options setters=0 r21=99999999999999999999
    emu_case "scan waveform too long" --keep-header --no-output -- -d $J --scan
    for jv in 17 100 161; do
        emu_options setters=0 r21=$jv
        emu_case "scan waveform $jv" --keep-header --no-output -- -d $J --scan
    done
    emu_options setters=0 r20=1
    emu_case "scan one enable flag" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r20=1,x
    emu_case "scan enable no number" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r23=100,5
    emu_case "scan frequency scale out of range" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r23=x,0
    emu_case "scan frequency mantissa no number" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r23=100,x
    emu_case "scan frequency scale no number" --keep-header -- -d $J --scan
    emu_options setters=0 r23=100,99999999999999999999
    emu_case "scan frequency scale too long" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r25=x
    emu_case "scan amplitude no number" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r27=x
    emu_case "scan offset no number" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r29=x
    emu_case "scan duty no number" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r31=x
    emu_case "scan phase no number" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r25=1e99
    emu_case "scan amplitude out of range" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 r25=
    emu_case "amplitude empty reads 0" -- -d $J -g CH1 --get amplitude

    # A reply's frame, on the first request, with a timeout of its own.
    jn=0
    for jr in ':x\r\n' 'r00=60.\r\n' ':w00=60.\r\n' ':a00=60.\r\n' ':b00=60.\r\n' ':r60.\r\n' \
        ':r99999999999999999999999=1.\r\n' '\r\n' ':\r\n' ' :r00=60.\r\n' ':r00=60'; do
        jn=$((jn + 1))
        emu_options setters=0 raw00=$(emu_hex "$jr")
        emu_case "first reply refused $jn" --keep-header --no-output -- -d $J --scan
    done
    jn=0
    for jr in ':r01=60.\r\n' ':r00=60\n' ':r00=60. \r\n' ':r0=60.\r\n' ':r=60.\r\n' ':r00 =60.\r\n'; do
        jn=$((jn + 1))
        emu_options setters=0 raw00=$(emu_hex "$jr")
        emu_case "first reply passes $jn" --keep-header -- -d $J --scan
    done
    # A NUL hides what follows it in that read from the look for the line feed.
    emu_options setters=0 raw00=$(emu_hex ':r00=60\000x\r\n')
    emu_case "first reply NUL before the line feed" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 raw00=$(emu_hex ':r00=60.\r\n\000')
    emu_case "first reply NUL behind the line feed" --keep-header -- -d $J --scan
    emu_options setters=0 raw00=none
    emu_case "scan silence" --keep-header --no-output -- -d $J --scan
    # Later replies: a wrong index is refused, as index 0 alone is not checked.
    emu_options setters=0 raw01=$(emu_hex ':r02=1.\r\n')
    emu_case "serial number wrong index" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 raw20=$(emu_hex ':r20=1,1.\r\n:r21=0.\r\n')
    emu_case "two lines in one reply" --keep-header -- -d $J -g CH1 --show

    # C has no time limit here (timeout_ms 0): only the buffer's end stops it.
    emu_options setters=0 after=1 flood=17000
    emu_case "serial number with no end of line" --keep-header --no-output -- -d $J --scan

    # A frequency of one word or none: a parser handed NULL.
    for jf in 100 ""; do
        emu_options setters=0 r23=$jf
        emu_case "scan frequency words '$jf'" --oracle-bug "$CRASH_BUG" --keep-header --no-output -- -d $J --scan
    done

    # A device that stops answering: no timeout but the first's.
    for jr in raw01 raw20 raw21 raw23 raw25 raw27 raw29 raw31; do
        emu_options setters=0 $jr=none
        emu_case "scan silence at $jr" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 5 --keep-header --no-output -- -d $J --scan
    done
    emu_options setters=0 answers=14
    emu_case "get silence" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 5 --expect-fail --stderr -- -d $J --get phase
    emu_options answers=14
    emu_case "set silence" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 5 --expect-fail --stderr -- -d $J -c phase=1 --set

    # Bad replies to a get that follows a good scan.
    for jk in "CH1 enabled 20" "CH1 pattern 21" "CH2 pattern 22" "CH1 output_frequency 23" "CH2 output_frequency 24" \
        "CH1 amplitude 25" "CH2 amplitude 26" "CH1 offset 27" "CH2 offset 28" "CH1 output_duty_cycle 29" "CH2 output_duty_cycle 30"; do
        set -- $jk
        emu_options setters=0 after=14 raw$3=$(emu_hex ':x\r\n')
        emu_case "get frame error $1 $2" --stderr --expect-fail --filter "$E" -- -d $J -g $1 --get $2
    done
    for jk in "CH1 enabled 20 x" "CH1 pattern 21 999" "CH1 output_frequency 23 x" "CH1 amplitude 25 x" "CH1 offset 27 x" "CH1 output_duty_cycle 29 x"; do
        set -- $jk
        emu_options setters=0 after=14 raw$3=$(emu_hex ":r$3=$4.\r\n")
        emu_case "get value error $1 $2" --stderr --expect-fail --filter "$E" -- -d $J -g $1 --get $2
    done
    emu_options setters=0 after=14 raw31=$(emu_hex ':r31=x.\r\n')
    emu_case "get value error phase" --stderr --expect-fail --filter "$E" -- -d $J --get phase
    emu_options setters=0 after=14 raw31=$(emu_hex ':x\r\n')
    emu_case "get frame error phase" --stderr --expect-fail --filter "$E" -- -d $J --get phase
    emu_options setters=0 after=14 raw20=$(emu_hex ':x\r\n')
    emu_case "show CH1 frame error on one" --keep-header -- -d $J -g CH1 --show

    # Settings, each on its own line.
    emu_options
    emu_case "set enabled off all" --no-output -- -d $J -c enabled=off --set
    emu_case "set enabled on all" --no-output -- -d $J -c enabled=on --set
    emu_case "set CH1 enabled off" --no-output -- -d $J -g CH1 -c enabled=off --set
    emu_case "set CH2 enabled off" --no-output -- -d $J -g CH2 -c enabled=off --set
    emu_case "set phase" --no-output -- -d $J -c phase=90 --set
    for jp in 359.9 360 450.5 -90 -360 0.05 0.15 -0 1e30; do
        case $jp in
            -0 | -360) emu_case "set phase $(jm $jp)" --oracle-differs "$ZERO_BUG" --no-output -- -d $J -c phase=$jp --set ;;
            *) emu_case "set phase $(jm $jp)" --no-output -- -d $J -c phase=$jp --set ;;
        esac
    done
    emu_case "set phase inf" --oracle-differs "$NAN_BUG" --expect-fail --stderr -- -d $J -c phase=inf --set
    emu_case "set phase nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr -- -d $J -c phase=nan --set
    for jw in sine lorenz arb-01 arb-60; do
        emu_case "set CH1 pattern $jw" --no-output -- -d $J -g CH1 -c pattern=$jw --set
        emu_case "set CH2 pattern $jw" --no-output -- -d $J -g CH2 -c pattern=$jw --set
    done
    emu_case "set pattern unknown" --stderr --expect-fail --filter "$E" -- -d $J -g CH1 -c pattern=arb-61 --set
    emu_case "set pattern device wide" --stderr --expect-fail -- -d $J -c pattern=sine --set
    for jf in 1000 0.01 0.001 0 -5 60000000 60000001 1234.56 0.125 inf; do
        emu_case "set CH1 frequency $(jm $jf)" --no-output -- -d $J -g CH1 -c output_frequency=$jf --set
    done
    emu_case "set CH1 frequency nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr -- -d $J -g CH1 -c output_frequency=nan --set
    emu_case "set CH2 frequency" --no-output -- -d $J -g CH2 -c output_frequency=1000 --set
    for ja in 5 0 20 21 -1 1.2345 0.0005 0.0015 inf; do
        emu_case "set CH1 amplitude $(jm $ja)" --no-output -- -d $J -g CH1 -c amplitude=$ja --set
    done
    emu_case "set CH1 amplitude minus 0" --oracle-differs "$ZERO_BUG" --no-output -- -d $J -g CH1 -c amplitude=-0 --set
    emu_case "set CH1 amplitude nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr -- -d $J -g CH1 -c amplitude=nan --set
    emu_case "set CH2 amplitude" --no-output -- -d $J -g CH2 -c amplitude=5 --set
    for jo in 0 1 -1 9.99 -9.99 10 -10 0.005 -0 inf; do
        emu_case "set CH1 offset $(jm $jo)" --no-output -- -d $J -g CH1 -c offset=$jo --set
    done
    emu_case "set CH1 offset nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr -- -d $J -g CH1 -c offset=nan --set
    emu_case "set CH2 offset" --no-output -- -d $J -g CH2 -c offset=1 --set
    for jd in 0.5 0 1 2 -1 0.0005 0.0015 0.3333 inf; do
        emu_case "set CH1 duty $(jm $jd)" --no-output -- -d $J -g CH1 -c output_duty_cycle=$jd --set
    done
    emu_case "set CH1 duty minus 0" --oracle-differs "$ZERO_BUG" --no-output -- -d $J -g CH1 -c output_duty_cycle=-0 --set
    emu_case "set CH1 duty nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr -- -d $J -g CH1 -c output_duty_cycle=nan --set
    emu_case "set CH2 duty" --no-output -- -d $J -g CH2 -c output_duty_cycle=0.25 --set
    emu_case "set then read back" -- -d $J -g CH1 -c amplitude=3.3 --get amplitude
    emu_case "set enable then read back" -- -d $J -g CH1 -c enabled=off --get enabled
    emu_case "set several" --no-output -- -d $J -g CH1 -c output_frequency=1000:amplitude=2:pattern=sine:offset=1:output_duty_cycle=0.5 --set
    emu_case "set each channel" --no-output -- -d $J -c channel_group=CH1:amplitude=1 -c channel_group=CH2:amplitude=2 --set
    emu_case "set the output with the rest" --oracle-differs "$ORDER_BUG" --no-output -- -d $J -g CH1 -c amplitude=1:enabled=on --set
    emu_case "set not a number" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr -- -d $J -g CH1 -c amplitude=abc --set
    emu_case "set read only conn" --stderr --expect-fail -- -d $J -c conn=x --set
    emu_case "set amplitude device wide" --stderr --expect-fail -- -d $J -c amplitude=1 --set
    emu_case "set phase on a channel" --stderr --expect-fail -- -d $J -g CH1 -c phase=1 --set
    # The channel's enable is the two as last seen, the other's as the scan read it.
    emu_options r20=0,1
    emu_case "set CH1 keeps CH2 as read" --no-output -- -d $J -g CH1 -c enabled=on --set

    # A write the device does not take.
    for jk in "enabled=on" "phase=1"; do
        emu_options after=14 ok=$(emu_hex ':err\r\n')
        emu_case "set answered badly $jk" --stderr --expect-fail --filter "$E" -- -d $J -c $jk --set
    done
    for jk in enabled=on pattern=sine output_frequency=1 amplitude=1 offset=1 output_duty_cycle=0.5; do
        emu_options after=14 ok=$(emu_hex ':err\r\n')
        emu_case "set CH1 answered badly $jk" --stderr --expect-fail --filter "$E" -- -d $J -g CH1 -c $jk --set
    done
    emu_options after=14 ok=$(emu_hex 'ok\r\n')
    emu_case "set answered with no colon" --stderr --expect-fail --filter "$E" -- -d $J -c phase=1 --set
    emu_options after=14 ok=$(emu_hex ':ok.\r\n')
    emu_case "set answered with a period" --no-output -- -d $J -c phase=1 --set

    # A capture: nothing to acquire, and no limit key to set.
    emu_options setters=0
    emu_case "samples" --keep-header --stderr --expect-fail -- -d $J --samples 1
    emu_case "time" --keep-header --stderr --expect-fail -- -d $J --time 100
    emu_case "continuous" --keep-header --stderr --expect-fail -- -d $J --continuous
    # Past sigrok-cli's refusals: the channels start disabled (api.c:109-110), so
    # the session refuses to start at all.
    emu_case "lib capture has no enabled channels" --libsigrok --expect-fail --stderr -- -d $J --continuous
    emu_options
    emu_case "capture with a setting" --oracle-bug "$PARTIAL_BUG" --keep-header --stderr --expect-fail -- -d $J -c enabled=on --samples 1

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 chunk=1
    emu_case "one byte per transfer" --keep-header -- -d $J -g CH1 --show
    emu_options setters=0 in-stall=14
    emu_case "a stalled read" --keep-header -- -d $J --scan
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header --no-output -- -d $J --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $J --scan
    # An unplug: a read that finds nothing goes on and on in C.
    emu_options setters=0 answers=1
    emu_case "unplug in the scan" --oracle-bug "$HANG_BUG" --oracle-hangs --unplug model --timeout 5 --keep-header --no-output -- -d $J --scan
    emu_options setters=0 answers=14
    emu_case "unplug in a get" --oracle-bug "$HANG_BUG" --oracle-hangs --unplug model --timeout 5 --expect-fail --no-output -- -d $J --get phase
    emu_options
}

# A sign in words, as a case's file name would lose it.
jm() {
    case $1 in
        -*) echo "minus ${1#-}" ;;
        *) echo "$1" ;;
    esac
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    J=juntek-jds6600:conn=$EMU_TTY
}
