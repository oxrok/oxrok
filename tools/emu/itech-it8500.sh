# An ITECH IT8500 electronic load (emu/src/models/itech-it8500.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model".
EMU_MODEL=itech-it8500
EMU_CDC=1
EMU_DRIVER=itech-it8500
EMU_FULL=$EMU_DRIVER
EMU_GCOV=itech-it8500
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# it_le N BYTES: N as BYTES little-endian bytes of hex.
it_le() {
    it_n=$1 it_c=$2
    while [ "$it_c" -gt 0 ]; do
        printf '%02x' $((it_n & 255))
        it_n=$((it_n >> 8)) it_c=$((it_c - 1))
    done
}
# it_state V I P OP DEMAND: a 5f's data, in mV, 0.1 mA, mW and the registers.
it_state() {
    printf '%s%s%s%s%s' "$(it_le "$1" 4)" "$(it_le "$2" 4)" "$(it_le "$3" 4)" "$(it_le "$4" 1)" "$(it_le "$5" 2)"
}
# it_frame ADDRESS COMMAND HEX: a whole packet, padded, with its checksum.
it_frame() {
    it_h="aa$(it_le "$1" 1)$2$3"
    while [ ${#it_h} -lt 50 ]; do it_h=${it_h}00; done
    it_r=$it_h it_s=0
    while [ -n "$it_r" ]; do
        it_b=${it_r%"${it_r#??}"} it_r=${it_r#??}
        it_s=$((it_s + 0x$it_b))
    done
    printf '%s%s' "$it_h" "$(it_le "$it_s" 1)"
}

emu_cases() {
    emu_conns
    G="channel_group=1"
    # The driver's own lines at -l 4, no other module's.
    N='s/^oxrok: [a-z]+: oxrok::drivers::itech_it8500[a-z_:]*: /sr: itech-it8500: /'
    D="$N; "'s/^sr: \[[^]]*\] /sr: /; /^sr: itech-it8500: /!d; s/^sr: //'
    # What oxrok adds to the error sigrok-cli names.
    F="$N; "'s/: unknown option: uvc_threshold\.$/: not applicable./; s/^(Failed to [a-z ]+ .[a-z_]+.: )device error: generic/\1generic/; s/^(Failed to set device option .[a-z_]+.: )invalid argument: invalid samplerate\.$/\1invalid samplerate./; s/^(Failed to set device option .[a-z_]+.: invalid argument): [^.]*\.$/\1./'
    NO_CLOSE_BUG="sigrok-cli's logger exits on a critical (main.c:46-47), before its sr_dev_close (main.c:130 then 201, session.c:708), so an error leaves the load in remote mode with its panel locked; oxrok sends local mode"
    NO_INT_BUG="api.c:503 converts a double no int can hold, which is undefined, 0x80000000 on x86, and sends it; oxrok refuses it and sends nothing"
    SUCCESS_BUG="protocol.c:139-144 takes a status of success for any command, and callers read its data: 0x80 as a number, 0.128 V, or a model named so; oxrok takes it for no data"
    SHIFT_BUG="itech_it8500_send_cmd reads 26 bytes with nothing dropped first (protocol.c:112), so a byte too many shifts every answer until 26 of them have come round; oxrok drops the queue before each command"
    HANG_BUG="itech_it8500_receive_data returns before its limit check when a poll fails (protocol.c:385-386), so a silent load holds a capture for ever, a time limit too; oxrok checks the time, and ends after 5 s where only a sample limit could"
    ORDER_BUG="sigrok-cli sets each -c in turn (session.c:700-707), so a load goes on ahead of what follows; oxrok sends enabled=on last"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # 12.345 V, 1.2345 A, 15.24 W, load on, constant current, then the rest.
    S0=$(it_state 12345 12345 15240 8 64)
    S1=$(it_state 11000 20000 22000 8 64)
    R0=$(it_frame 0 5f "$S0")
    R1=$(it_frame 0 5f "$S1")

    # Scans. A scan asks 6a, 6b, 01, 5f, and sets nothing.
    emu_options
    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "show group" --keep-header -- -d $A -g 1 --show
    emu_case "show no such group" --oracle-bug "$NO_CLOSE_BUG" --keep-header --stderr --expect-fail -- -d $A -g 2 --show
    emu_case "no such node" --keep-header --no-output -- -d itech-it8500:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d itech-it8500 --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d itech-it8500 --show
    emu_case "scan log" --stderr --filter "$D" -- -l 4 -d $A --scan
    emu_options model=85112 firmware=02.5a serial=ABCDEFGHIJ
    emu_case "scan names in full" --keep-header -- -d $A --scan
    emu_options model= serial=
    emu_case "scan empty names" --keep-header -- -d $A --scan
    emu_options address=7
    emu_case "scan an address" --keep-header -- -d $A --scan
    emu_case "get at an address" -- -d $A -g 1 --get voltage
    emu_case "set at an address" --no-output -- -d $A -c $G:enabled=on --set
    emu_options max-current=1200000 max-voltage=800000 min-voltage=0 max-power=1200000 max-resistance=4000000 min-resistance=65535
    emu_case "show other limits" --keep-header -- -d $A -g 1 --show
    emu_case "scan log other limits" --stderr --filter "$D" -- -l 4 -d $A --scan
    emu_options chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $A --scan

    # What a scan does with each answer gone wrong.
    emu_options answers=0
    emu_case "scan no answer" --keep-header --no-output -- -d $A --scan
    emu_options silent-6b=1
    emu_case "scan no barcode" --keep-header -- -d $A --scan
    emu_options status-6b=c0
    emu_case "scan barcode refused" --keep-header -- -d $A --scan
    emu_options barcode=ABCDEFGHIJKLMNOPQRSTUV
    # api.c:217-219 copies it, terminated, and logs the packet's own data, with
    # no end: what comes after the 22 bytes is heap, so the line is replaced.
    emu_case "scan log long barcode" --stderr --filter "$D; /Barcode/!d; /Barcode: ABCDEFGHIJKLMNOPQRSTUV/c 22 bytes of barcode" -- -l 4 -d $A --scan
    emu_options silent-01=1
    emu_case "scan no limits" --keep-header --no-output -- -d $A --scan
    emu_options status-01=c0
    emu_case "scan limits refused" --keep-header --no-output -- -d $A --scan
    emu_options silent-5f=1
    emu_case "scan no state" --keep-header --no-output -- -d $A --scan
    emu_case "scan no state says so" --stderr --keep-header --filter "$N" -- -d $A --scan
    emu_options status-5f=c0
    emu_case "scan state refused" --keep-header --no-output -- -d $A --scan
    emu_options status-6a=b0
    emu_case "scan refused" --keep-header --no-output -- -d $A --scan
    emu_options status-6a=80
    emu_case "scan a success for data" --oracle-differs "$SUCCESS_BUG" --keep-header --no-output -- -d $A --scan
    emu_options reply-6a=aa00
    emu_case "scan a short answer" --keep-header --no-output -- -d $A --scan
    emu_options reply-6a=$(it_frame 0 6b "")
    emu_case "scan another command" --keep-header --no-output -- -d $A --scan
    GOOD=$(it_frame 0 6a "$(emu_hex 8500)")
    emu_options reply-6a=${GOOD%??}00
    emu_case "scan wrong checksum" --keep-header --no-output -- -d $A --scan
    emu_options reply-6a=55${GOOD#??}
    emu_case "scan no preamble" --keep-header --no-output -- -d $A --scan
    emu_options reply-6a=$(it_frame 9 6a "$(emu_hex 8500)")
    emu_case "scan names an address it does not answer" --keep-header --no-output -- -d $A --scan

    # The line: four rates to try, and the sample rates each one allows.
    emu_options uart=38400/8n1
    emu_case "scan at 38400" --keep-header -- -d $A --scan
    emu_case "samplerate 60 at 38400" -- -d $A -c samplerate=60 --get samplerate
    emu_case "samplerate 61 at 38400" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c samplerate=61 --get samplerate
    emu_case "show at 38400" --keep-header -- -d $A --show
    emu_options uart=19200/8n1
    emu_case "scan at 19200" --keep-header -- -d $A --scan
    emu_case "samplerate 30 at 19200" -- -d $A -c samplerate=30 --get samplerate
    emu_case "samplerate 31 at 19200" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c samplerate=31 --get samplerate
    emu_options uart=4800/8n1
    emu_case "scan at 4800" --keep-header -- -d $A --scan
    emu_case "samplerate 10 at 4800" -- -d $A -c samplerate=10 --get samplerate
    emu_case "samplerate 11 at 4800" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c samplerate=11 --get samplerate
    emu_options uart=2400/8n1
    emu_case "scan at none it tries" --keep-header --no-output -- -d $A --scan
    emu_case "serialcomm at 2400" --keep-header -- -d $A:serialcomm=2400/8n1 --scan
    emu_case "samplerate 10 at 2400" -- -d $A:serialcomm=2400/8n1 -c samplerate=10 --get samplerate
    emu_options uart=115200/8n1
    emu_case "samplerate 60 at 115200" -- -d $A:serialcomm=115200/8n1 -c samplerate=60 --get samplerate
    emu_options
    emu_case "serialcomm as the device" --keep-header -- -d $A:serialcomm=9600/8n1 --scan
    emu_case "serialcomm another rate" --keep-header --no-output -- -d $A:serialcomm=19200/8n1 --scan
    emu_case "serialcomm with parity" --keep-header --no-output -- -d $A:serialcomm=9600/8e1 --scan
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $A:serialcomm=fast --scan
    emu_case "samplerate 15 at 9600" -- -d $A -c samplerate=15 --get samplerate
    emu_case "samplerate 16 at 9600" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c samplerate=16 --get samplerate
    emu_case "samplerate off the list" -- -d $A -c samplerate=7 --get samplerate
    emu_case "samplerate 0" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c samplerate=0 --get samplerate

    # Gets.
    emu_options
    for key in samplerate limit_samples limit_time; do
        emu_case "get $key" -- -d $A --get $key
    done
    emu_case "set limit_samples read back" -- -d $A -c limit_samples=5 --get limit_samples
    emu_case "set limit_time read back" -- -d $A -c limit_time=5 --get limit_time
    for key in enabled regulation voltage voltage_target current current_limit power power_target resistance_target ovp_enabled ovp_active ovp_threshold ocp_enabled ocp_active ocp_threshold uvc uvc_active uvc_threshold otp otp_active; do
        emu_case "get $key" -- -d $A -g 1 --get $key
    done
    emu_case "get voltage without a group" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A --get voltage
    emu_case "get samplerate of a group" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get samplerate
    emu_case "get in a group that is not" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 2 --get voltage
    emu_options load=1 mode=1
    emu_case "get regulation CV" -- -d $A -g 1 --get regulation
    emu_case "get enabled on" -- -d $A -g 1 --get enabled
    emu_case "get current in CV" -- -d $A -g 1 --get current
    emu_options load=1 mode=2
    emu_case "get regulation CW" -- -d $A -g 1 --get regulation
    emu_case "get power in CW" -- -d $A -g 1 --get power
    emu_options load=1 mode=3
    emu_case "get regulation CR" -- -d $A -g 1 --get regulation
    emu_case "get current in CR" -- -d $A -g 1 --get current
    # Protection flags, and no mode flag at all, which reads as CC.
    emu_options state=$(it_state 1 2 3 0 2)
    emu_case "get ovp active set" -- -d $A -g 1 --get ovp_active
    emu_case "get ocp active is not" -- -d $A -g 1 --get ocp_active
    emu_case "get regulation of no flag" -- -d $A -g 1 --get regulation
    emu_options state=$(it_state 1 2 3 0 4)
    emu_case "get ocp active set" -- -d $A -g 1 --get ocp_active
    emu_options state=$(it_state 1 2 3 0 16)
    emu_case "get otp active set" -- -d $A -g 1 --get otp_active
    emu_options state=$(it_state 4294967295 4294967295 4294967295 255 65535)
    emu_case "get voltage the largest" -- -d $A -g 1 --get voltage
    emu_case "get current the largest" -- -d $A -g 1 --get current
    emu_case "get power the largest" -- -d $A -g 1 --get power
    emu_options cv=4294967295 cc=4294967295 cw=2147483648 cr=4294966296 ovp=2147483647 ocp=2147483648
    emu_case "get voltage_target negative" -- -d $A -g 1 --get voltage_target
    emu_case "get current_limit negative" -- -d $A -g 1 --get current_limit
    emu_case "get power_target lowest" -- -d $A -g 1 --get power_target
    emu_case "get resistance_target negative" -- -d $A -g 1 --get resistance_target
    emu_case "get ovp_threshold highest" -- -d $A -g 1 --get ovp_threshold
    emu_case "get ocp_threshold lowest" -- -d $A -g 1 --get ocp_threshold
    # A change since the scan. A get has no session to tell: C logs "session was
    # NULL" for each (protocol.c:202-234, session.c:1142-1145), oxrok nothing, and
    # a unit test pins that it stays the baseline of a capture's meta packets.
    emu_options state=$(it_state 1 2 3 0 64),$(it_state 1 2 3 8 66)
    emu_case "get after a change" -- -d $A -g 1 --get voltage
    # Five answers: the scan's four and the remote mode.
    emu_options answers=5
    emu_case "get voltage no answer" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get voltage
    emu_case "get voltage_target no answer" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get voltage_target
    for key in enabled regulation current power ovp_active ocp_active otp_active; do
        emu_case "get $key no answer" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get $key
    done
    emu_options status-2d=a0
    emu_case "get voltage_target refused" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get voltage_target
    for key in voltage_target current_limit power_target resistance_target ovp_threshold ocp_threshold; do
        emu_options reply-23=aa reply-25=aa reply-2b=aa reply-2d=aa reply-2f=aa reply-31=aa
        emu_case "get $key a short answer" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get $key
    done
    emu_options status-2d=80
    emu_case "get voltage_target a success for data" --oracle-differs "$SUCCESS_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get voltage_target
    emu_options reply-2d=$(it_frame 0 2b "$(it_le 1000 4)")
    emu_case "get voltage_target another command" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -g 1 --get voltage_target

    # The remote mode, at each open and close, whose failure is only a note.
    emu_options silent-20=1
    emu_case "no answer to remote" -- -d $A -g 1 --get voltage
    emu_options status-20=c0
    emu_case "remote refused" -- -d $A -g 1 --get voltage
    emu_case "remote refused log" --stderr --filter "$D; /remote|local/!d" -- -l 4 -d $A -g 1 --get voltage

    # Sets.
    emu_options
    emu_case "set enabled on" --no-output -- -d $A -c $G:enabled=on --set
    emu_case "set enabled off" --no-output -- -d $A -c $G:enabled=off --set
    for mode in CC CV CW CR; do
        emu_case "set regulation $mode" --no-output -- -d $A -c $G:regulation=$mode --set
    done
    emu_case "set regulation with more behind" --no-output -- -d $A -c $G:regulation=CVX --set
    emu_case "set regulation no such" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:regulation=XX --set
    emu_case "set regulation lower case" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:regulation=cc --set
    emu_case "set voltage_target" --no-output -- -d $A -c $G:voltage_target=5 --set
    emu_case "set current_limit" --no-output -- -d $A -c $G:current_limit=1.5 --set
    emu_case "set power_target" --no-output -- -d $A -c $G:power_target=20 --set
    emu_case "set resistance_target" --no-output -- -d $A -c $G:resistance_target=4 --set
    emu_case "set ovp_threshold" --no-output -- -d $A -c $G:ovp_threshold=50 --set
    emu_case "set ocp_threshold" --no-output -- -d $A -c $G:ocp_threshold=10 --set
    emu_case "set voltage_target 3.3" --no-output -- -d $A -c $G:voltage_target=3.3 --set
    emu_case "set current_limit 0.29" --no-output -- -d $A -c $G:current_limit=0.29 --set
    emu_case "set voltage_target read back" -- -d $A -g 1 -c voltage_target=7.25 --get voltage_target
    emu_case "set enabled read back" -- -d $A -g 1 -c enabled=on --get enabled
    emu_case "set regulation read back" -- -d $A -g 1 -c regulation=CW --get regulation
    emu_case "set ovp_threshold read back" -- -d $A -g 1 -c ovp_threshold=50 --get ovp_threshold
    emu_case "set ocp_threshold read back" -- -d $A -g 1 -c ocp_threshold=1.25 --get ocp_threshold
    # Refused by the load, once the driver has sent them.
    emu_case "set voltage_target too high" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage_target=121 --set
    emu_case "set voltage_target negative" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage_target=-1 --set
    emu_case "set current_limit -0" --no-output -- -d $A -c $G:current_limit=-0 --set
    emu_case "set current_limit too high" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:current_limit=31 --set
    emu_case "set power_target too high" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:power_target=301 --set
    emu_case "set resistance_target too low" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:resistance_target=0.01 --set
    emu_case "set ovp_threshold too high" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:ovp_threshold=121 --set
    emu_case "set ocp_threshold too high" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:ocp_threshold=31 --set
    emu_case "set voltage_target nan" --oracle-bug "$NO_INT_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage_target=nan --set
    emu_case "set voltage_target inf" --oracle-bug "$NO_INT_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage_target=inf --set
    emu_case "set voltage_target 1e10" --oracle-bug "$NO_INT_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage_target=1e10 --set
    # Scope.
    emu_case "set uvc_threshold" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:uvc_threshold=1 --set
    emu_case "set enabled without a group" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c enabled=on --set
    emu_case "set voltage read only" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage=5 --set
    emu_case "set samplerate of a group" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:samplerate=5 --set
    emu_case "set in a group that is not" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c channel_group=2:enabled=on --set
    emu_case "set limit_samples 0" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c limit_samples=0 --set
    emu_case "set limit_time 0" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c limit_time=0 --set
    # Answers gone wrong.
    emu_options status-2c=a0
    emu_case "set refused" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:voltage_target=5 --set
    emu_options silent-21=1
    emu_case "set no answer" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:enabled=on --set
    emu_options reply-21=$(it_frame 0 22 "")
    emu_case "set another answer" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:enabled=on --set
    emu_options reply-21=$(it_frame 0 21 "")
    emu_case "set an echo for a status" --no-output -- -d $A -c $G:enabled=on --set

    # Output after its setpoints, one -c each, the order kept.
    emu_options
    emu_case "set setpoint then output" --no-output -- -d $A -c $G:current_limit=2 -c $G:enabled=on --set
    emu_case "set mode setpoint output" --no-output -- -d $A -c $G:regulation=CV -c $G:voltage_target=5 -c $G:enabled=on --set

    # oxrok holds the output back, whatever the order given.
    emu_case "set output then setpoint" --oracle-differs "$ORDER_BUG" --no-output -- -d $A -c $G:enabled=on -c $G:current_limit=2 --set
    emu_case "set output then a refused setpoint" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A -c $G:enabled=on -c $G:voltage_target=121 --set
    emu_case "capture with output then setpoint" --oracle-differs "$ORDER_BUG" -- -d $A -c $G:enabled=on -c $G:current_limit=2 --samples 1

    # Captures. No poll comes before one period, and a limit is checked after
    # each reading alone.
    emu_options load=1 cc=12345
    emu_case "samples 1" -- -d $A --samples 1
    emu_case "samples 3" -- -d $A --samples 3
    emu_case "samples 2 analog" -- -d $A --samples 2 -O analog
    emu_case "samples 2 csv" -- -d $A --samples 2 -O csv:header=false:time=false
    emu_case "samples 2 V1" -- -d $A -C V1 --samples 2
    emu_case "samples 2 I1 P1" -- -d $A -C I1,P1 --samples 2
    emu_case "samples 2 P1 csv" -- -d $A -C P1 --samples 2 -O csv:header=false:time=false
    emu_case "config limit_samples" --libsigrok --oxrok -- -d $A -c limit_samples=2 --continuous
    emu_case "time" -- -d $A -c samplerate=1 --time 500
    emu_case "config limit_time" --libsigrok --oxrok -- -d $A -c samplerate=1:limit_time=500 --continuous
    emu_case "samplerate 15" -- -d $A -c samplerate=15 --samples 2
    emu_case "frames refused" --oracle-bug "$NO_CLOSE_BUG" --stderr --expect-fail --filter "$F" -- -d $A --frames 1
    emu_options uart=38400/8n1 load=1
    emu_case "samplerate 60" -- -d $A -c samplerate=60 --samples 3
    emu_options
    emu_case "capture with a setting" -- -d $A -c $G:current_limit=2 --samples 1
    emu_case "capture with the load on" -- -d $A -c $G:current_limit=2 -c $G:enabled=on --samples 2
    # Numbers in the last digits, on every output.
    emu_options state=$S0,$S1
    emu_case "samples values" -- -d $A --samples 3
    emu_case "samples values analog" -- -d $A --samples 3 -O analog
    emu_case "samples values csv" -- -d $A --samples 3 -O csv:header=false:time=false
    # Changes of state, which go out as meta packets.
    emu_options state=$(it_state 1 2 3 0 64),$(it_state 1 2 3 8 66),$(it_state 1 2 3 8 134),$(it_state 1 2 3 0 528)
    emu_case "samples changes of state" -- -d $A --samples 4
    emu_case "samples changes of state analog" -- -d $A --samples 4 -O analog
    # A poll that fails is no sample, and the next one counts.
    emu_options reply-5f=$R0,$R1,${R0%??}00,$R1
    emu_case "a wrong checksum is no sample" -- -d $A --samples 3 -O csv:header=false:time=false
    emu_options reply-5f=$R0,$R1,55${R0#??},$R1
    emu_case "no preamble is no sample" -- -d $A --samples 3 -O csv:header=false:time=false
    emu_options reply-5f=$R0,$R1,$(it_frame 0 2b ""),$R1
    emu_case "another command is no sample" -- -d $A --samples 3 -O csv:header=false:time=false
    emu_options reply-5f=$R0,$R1,$(it_frame 0 12 a0),$R1
    emu_case "a status is no sample" -- -d $A --samples 3 -O csv:header=false:time=false
    emu_options reply-5f=$R0,$R1,aa00,$R1
    emu_case "a short answer is no sample" -- -d $A --samples 3 -O csv:header=false:time=false
    # Readings of what is not one, and no framing: a byte too many shifts all
    # that follow, until 26 of them have come round, some 8 s here.
    emu_options reply-5f=$R0,$(it_frame 0 12 80),$R1
    emu_case "a success for data" --oracle-differs "$SUCCESS_BUG" -- -d $A --samples 2 -O csv:header=false:time=false
    emu_options reply-5f=$R0,${R1}ff,$R1
    emu_case "a long answer shifts nothing" --oracle-differs "$SHIFT_BUG" --timeout 30 -- -d $A --samples 3 -O csv:header=false:time=false
    # The library, as a client's button: the first sample comes after a period.
    emu_options
    emu_case "lib two samples" --libsigrok -- -d $A --continuous --stop-after 2
    emu_case "lib stop at the header" --libsigrok --no-output -- -d $A --continuous --stop-after 0
    emu_case "lib samples 1" --libsigrok -- -d $A --samples 1
    # A load that stops answering holds it, for each limit.
    emu_options answers=5
    emu_case "lib silent with a time limit" --libsigrok --hangs --timeout 2 --no-output -- -d $A --time 100
    emu_case "lib silent with a sample limit" --libsigrok --hangs --timeout 2 --no-output -- -d $A --samples 2
    emu_case "lib stop on a silent load" --libsigrok --no-output -- -d $A --continuous --stop-after 0

    # oxrok ends them. The scan's four answers and the remote mode, then silence.
    emu_case "silent with a time limit" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 4 --no-output -- -d $A -c samplerate=1 --time 500
    emu_case "silent with a sample limit says so" --oracle-bug "$HANG_BUG" --oracle-hangs --timeout 9 --stderr --filter "$N" -- -d $A --samples 2
    # With no limit both wait. Three readings, then the signal on silence.
    emu_options answers=8
    emu_case "interrupt polls" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $A --continuous
    emu_options answers=5
    emu_case "interrupt on a silent load" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --no-output -- -d $A --continuous

    # A load that leaves.
    emu_options answers=0
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $A --scan
    emu_options answers=5
    emu_case "lib unplug in a capture" --libsigrok --hangs --unplug model --timeout 3 --no-output -- -d $A --continuous
    emu_case "unplug in a capture" --oracle-bug "$HANG_BUG" --oracle-hangs --unplug model --timeout 5 --no-output --tx-filter '/^unplug$/q' -- -d $A --samples 5

    # The USB side misbehaving, the shell's options.
    emu_options stall=line
    emu_case "line coding stalled" --keep-header -- -d $A --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $A --scan
    emu_options in-stall=4
    emu_case "a stalled read" --keep-header -- -d $A --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    A=itech-it8500:conn=$EMU_TTY
}
