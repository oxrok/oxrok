# An Arachnid Labs Re:load Pro electronic load
# (emu/src/models/arachnid-labs-re-load-pro.rs) on the CDC-ACM shell. Sourced
# by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=arachnid-labs-re-load-pro
EMU_CDC=1
EMU_DRIVER=arachnid-labs-re-load-pro
EMU_FULL=$EMU_DRIVER
EMU_GCOV=arachnid-labs-re-load-pro
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: (err|warn|info): oxrok::drivers::arachnid_labs_re_load_pro: /sr: arachnid-labs-re-load-pro: /'
    # What it adds after the error sigrok-cli names.
    A="$L
s/^(Failed to set device option .[a-z_]+.: invalid argument).*\$/\\1./"
    # A port that went. Of sigrok-cli's lines, libsigrok's own, with the OS's words
    # in it. Of oxrok's, the OS's words in its read error, for the line to stay.
    G="$L
/^sr: serial-libsp: /d
s/^oxrok: warn: oxrok::drivers::arachnid_labs_re_load_pro::protocol: (A read failed): .*\$/sr: arachnid-labs-re-load-pro: \\1./"
    # hwdriver.c's check_key refuses it ahead of the driver, and logs that.
    Z="$A
/^sr: hwdriver: Cannot set/d"
    NAN_BUG="protocol.c:78 and 130's two comparisons let a NaN through, and (int)round() sends set -2147483648, which the firmware refuses in a reply protocol.c:59 does not see; oxrok refuses it and sends nothing"
    ERR_BUG="protocol.c:59 looks for 'err ' in the bytes of the pointer bufptr (&bufptr), not in the reply, so an err line is a success; oxrok fails"
    SILENT_BUG="protocol.c:180 and 216 take a reply that did not come, or an err line, for a value: strtod of an empty buffer or of the error's text is 0; oxrok fails --get"
    ORDER_BUG="sigrok-cli sets -c's options in hash table order (session.c:700-701), here on before uvlo; oxrok sets enabled=on last"
    NOANS_BUG="the C takes a setting with no answer for a success, but the firmware answers each every time (comms.c), so the line was lost or the link is down, and enabled=off may have left the current flowing; oxrok fails"
    GONE_BUG="the C takes the silence ahead of the unplug for a success and goes on to a write that fails; oxrok has failed at the silence"
    CUT_BUG="protocol.c:51 takes what a read found in its time for a whole line, here a number cut short; oxrok fails"
    BACKLOG_BUG="protocol.c:51 takes the first line for the answer, here a reading left by monitoring, whose current it gives as the limit; oxrok reads on to the answer"
    FLAG_BUG="the C looks at overtemp and undervolt only in a capture, and only asks the flag it holds; oxrok reads what has arrived"
    NOVALUE_BUG="api.c:230, 246 and 253 return SR_OK with no value when the query failed, and sigrok-cli dies on it; protocol.c:167 and 203 also leave the mutex locked; oxrok has failed at the silence ahead of it"
    RETRY_BUG="protocol.c:54 retries once, and a CR LF ahead of the reply is two empty lines, so the reply is left unread and 0 is taken; oxrok passes over empty lines"
    SHORT_BUG="protocol.c:336 hands the token that g_strsplit did not make to g_ascii_strtod, whose assertion kills sigrok-cli; oxrok passes over the line"
    READ_BUG="protocol.c:254 reads tokens[2] past the end of what g_strsplit made of an empty reply, and sigrok-cli dies; oxrok fails --get"
    GAP_BUG="protocol.c:393 takes what a read found in 250 ms for a whole line, so a line in two pieces gives a wrong reading and an unknown packet; oxrok waits for the end of the line"
    STALE_BUG="api.c:126 reads what is first in the input after monitor 0, so a line still on its way from monitoring left on loses the device; oxrok drops the input first"
    TIME_BUG="protocol.c:425 returns ahead of the limit check at protocol.c:430 unless a byte has come, so a time limit waits for the next line; oxrok ends at the limit"
    SAMPLES_BUG="protocol.c:425 ignores everything but G_IO_IN, and nothing ends a capture whose lines stopped; oxrok gives up after 5 s with no reading"
    HUP_BUG="protocol.c:425 ignores everything but G_IO_IN, so a hung up port is no event and the session goes on for ever; oxrok ends the capture"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture and closes, with monitor 0"

    # The scan: monitor 0, 50 ms, version, one line back.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "scan with serialcomm" --keep-header -- -d $K:serialcomm=115200/8n1 --scan
    emu_case "scan misset behind the UART" --keep-header --no-output -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "scan serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
    emu_case "no such node" --keep-header --no-output -- -d arachnid-labs-re-load-pro:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d arachnid-labs-re-load-pro --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d arachnid-labs-re-load-pro --show

    # What the version check lets through: it compares as a float, so 1.9 and
    # 1.2 pass though they come before 1.10. No real firmware meets the refusal:
    # `version` came with v1.6, and 1.6 to 1.9 compare above 1.10.
    for v in 1.10 1.11 1.9 1.2 2.0 1.1 1.11x; do
        emu_options setters=0 version=$v
        emu_case "scan version $v" --keep-header -- -d $K --scan
    done
    for v in 1.0 1 0.13 abc; do
        emu_options setters=0 version=$v
        emu_case "scan version $v" --keep-header --no-output -- -d $K --scan
    done
    emu_options setters=0 version=1.0
    emu_case "scan too old says so" --keep-header --stderr --filter "$L" -- -d $K --scan -l 3
    emu_options setters=0 version=none
    emu_case "scan no answer" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 banner=$(emu_hex 'version \r\n')
    emu_case "scan version with nothing after it" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 banner=$(emu_hex 'ok\r\n')
    emu_case "scan another answer" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 banner=$(emu_hex '\r\nversion 1.11\r\n')
    emu_case "scan an empty line first" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 banner=$(emu_hex 'version 1.11')
    emu_case "scan no line end" --keep-header -- -d $K --scan
    emu_options setters=0 banner=$(emu_hex 'version 1.11 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n')
    emu_case "scan a line too long" --keep-header -- -d $K --scan

    # --show and --get: each asks the device afresh.
    emu_options setters=0 set=1500 uvlo=3300 volts=12345 amps=1499
    emu_case "show" --keep-header -- -d $K --show
    emu_case "show group" --keep-header -- -d $K -g 1 --show
    emu_case "get limit_samples" -- -d $K --get limit_samples
    emu_case "get limit_time" -- -d $K --get limit_time
    for key in regulation voltage current current_limit ovp_enabled ocp_enabled otp otp_active \
        uvc uvc_active uvc_threshold; do
        emu_case "get $key" -- -d $K -g 1 --get $key
    done
    emu_case "get a group's key with no group" --stderr --expect-fail -- -d $K --get voltage
    emu_case "get enabled cannot" --stderr --expect-fail -- -d $K -g 1 --get enabled
    emu_case "get another group" --stderr --expect-fail -- -d $K -g 2 --get voltage
    emu_options setters=0 uvlo=0
    emu_case "get uvc off" -- -d $K -g 1 --get uvc
    emu_case "get uvc_threshold off" -- -d $K -g 1 --get uvc_threshold
    emu_options setters=0 version=1.8
    emu_case "show on a firmware with no uvlo" --oracle-differs "$SILENT_BUG" --keep-header -- -d $K -g 1 --show

    # A setting answers with its value.
    emu_options
    emu_case "set enabled on" --no-output -- -d $K -g 1 -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -g 1 -c enabled=off --set
    emu_case "set current_limit" --no-output -- -d $K -g 1 -c current_limit=1.5 --set
    emu_case "set current_limit rounds in single precision" --no-output -- -d $K -g 1 -c current_limit=2.0005 --set
    emu_case "set current_limit 0" --no-output -- -d $K -g 1 -c current_limit=0 --set
    emu_case "set current_limit max" --no-output -- -d $K -g 1 -c current_limit=6 --set
    emu_case "set current_limit too high" --stderr --expect-fail --filter "$A" -- -d $K -g 1 -c current_limit=6.001 --set
    emu_case "set current_limit negative" --stderr --expect-fail --filter "$A" -- -d $K -g 1 -c current_limit=-1 --set
    emu_case "set current_limit nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr --filter "$A" -- -d $K -g 1 -c current_limit=nan --set
    emu_case "set uvc_threshold" --no-output -- -d $K -g 1 -c uvc_threshold=3.3 --set
    emu_case "set uvc_threshold 0" --no-output -- -d $K -g 1 -c uvc_threshold=0 --set
    emu_case "set uvc_threshold max" --no-output -- -d $K -g 1 -c uvc_threshold=60 --set
    emu_case "set uvc_threshold too high" --stderr --expect-fail --filter "$A" -- -d $K -g 1 -c uvc_threshold=60.001 --set
    emu_case "set uvc_threshold nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr --filter "$A" -- -d $K -g 1 -c uvc_threshold=nan --set
    emu_case "set then get" -- -d $K -g 1 -c current_limit=2.5 --get current_limit
    emu_case "set a read-only key" --stderr --expect-fail -- -d $K -g 1 -c voltage=1 --set
    emu_case "set a group's key with no group" --stderr --expect-fail -- -d $K -c current_limit=1 --set
    emu_case "set several" --oracle-differs "$ORDER_BUG" --no-output -- -d $K -g 1 -c current_limit=1.5:uvc_threshold=3.3:enabled=on --set
    # In the order sigrok-cli's hash table has them, so that the tools agree.
    emu_case "set two" --no-output -- -d $K -g 1 -c current_limit=1.5:uvc_threshold=3.3 --set
    emu_options version=1.8
    emu_case "set uvc_threshold on a firmware with no uvlo" --oracle-differs "$ERR_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 -c uvc_threshold=3.3 --set
    emu_options version=1.9
    emu_case "set enabled refused" --oracle-differs "$ERR_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 -c enabled=on --set
    emu_options answers=0
    emu_case "set enabled no answer" --oracle-differs "$NOANS_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 -c enabled=on --set
    emu_case "set current_limit no answer" --oracle-differs "$NOANS_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 -c current_limit=1 --set
    emu_options replies=$(emu_hex 'set 10')
    emu_case "set current_limit cut short" --oracle-differs "$CUT_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 -c current_limit=1 --set
    emu_options

    # Answers gone wrong, to what asks for a value.
    emu_options setters=0 answers=0
    emu_case "get current_limit no answer" --oracle-differs "$SILENT_BUG" --expect-fail --stderr -- -d $K -g 1 --get current_limit
    emu_case "get uvc_threshold no answer" --oracle-differs "$SILENT_BUG" --expect-fail --stderr -- -d $K -g 1 --get uvc_threshold
    emu_case "get uvc no answer" --oracle-differs "$SILENT_BUG" --expect-fail --stderr -- -d $K -g 1 --get uvc
    emu_options setters=0 replies=$(emu_hex '\nset 1500\r\n')
    emu_case "get the last answer's LF first" -- -d $K -g 1 --get current_limit
    emu_options setters=0 replies=$(emu_hex '\r\nset 1500\r\n')
    emu_case "get an empty line first" --oracle-differs "$RETRY_BUG" -- -d $K -g 1 --get current_limit
    emu_options setters=0 replies=$(emu_hex "err Unknown command 'set'\r\n")
    emu_case "get current_limit err" --oracle-differs "$SILENT_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 --get current_limit
    emu_options setters=0 replies=$(emu_hex "err Unknown command 'read'\r\n")
    emu_case "get voltage err" --oracle-differs "$SILENT_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 --get voltage
    emu_options setters=0 fields=4 volts=12345 amps=1499
    emu_case "get voltage four numbers" -- -d $K -g 1 --get voltage
    emu_case "get current four numbers" -- -d $K -g 1 --get current
    emu_options setters=0 replies=$(emu_hex 'set 15')
    emu_case "get current_limit cut short" --oracle-differs "$CUT_BUG" --expect-fail --stderr --filter "$L" -- -d $K -g 1 --get current_limit
    # Monitoring left on by a capture: lines ahead of the answer.
    B=$(i=0; while [ $i -lt 24 ]; do printf 'read 1 2\\r\\n'; i=$((i + 1)); done)
    emu_options setters=0 replies=$(emu_hex "${B}set 1500\r\n")
    emu_case "get with readings ahead of the answer" --oracle-differs "$BACKLOG_BUG" -- -d $K -g 1 --get current_limit
    emu_options setters=0 replies=$(emu_hex 'read x y\r\n')
    emu_case "get voltage garbage" -- -d $K -g 1 --get voltage

    # A capture: monitor 200, lines, monitor 0 at the close. Each case's count
    # holds what comes after the last line it reads.
    emu_options setters=0 set=1500 volts=12345 amps=1499 count=1
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 1 csv" -- -d $K --samples 1 -O csv:header=false:time=false
    emu_case "samples 1 analog" --keep-header -- -d $K --samples 1 -O analog
    emu_options setters=0 fields=4 volts=12345 amps=1499 count=1
    emu_case "samples 1 four numbers" -- -d $K --samples 1
    emu_options setters=0 set=1500 volts=12345 amps=1499 count=3
    emu_case "samples 3" -- -d $K --samples 3
    emu_case "channel V only" -- -d $K -C V --samples 3
    emu_case "channel I only" -- -d $K -C I --samples 3
    emu_case "channel V only csv" -- -d $K -C V --samples 3 -O csv:header=false:time=false
    emu_case "set then get limit_samples" -- -d $K -c limit_samples=3 --get limit_samples
    emu_case "config limit_samples 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_samples=0 --samples 1
    emu_case "config limit_time 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_time=0 --samples 1
    emu_options set=1500 volts=12345 count=1
    emu_case "capture after a set" -- -d $K -g 1 -c current_limit=0.5 --samples 1
    emu_case "capture with enabled off" -- -d $K -g 1 -c enabled=off --samples 1
    # The readings a float holds, from small to the largest in the field.
    emu_options setters=0 count=4 lines=$(emu_hex 'read 1 1 0 0\r\n'),$(emu_hex 'read 3 3 0 0\r\n'),$(emu_hex 'read 6000 60000 123 456\r\n'),$(emu_hex 'read 0 0\r\n')
    emu_case "readings in turn" -- -d $K --samples 4
    # A line of the load's own, come ahead of what reads next: the only word that
    # it turned itself off. It stays in the input behind a setting's answer.
    emu_options volts=12345 count=1 replies=$(emu_hex 'set 500\r\novertemp\r\n')
    emu_case "overtemp ahead of a capture" --keep-header -- -d $K -g 1 -c current_limit=0.5 --samples 1 -O analog
    emu_case "overtemp ahead of a get" --oracle-differs "$FLAG_BUG" -- -d $K -g 1 -c current_limit=0.5 --get otp_active
    emu_options replies=$(emu_hex 'set 500\r\nundervolt\r\n')
    emu_case "undervolt ahead of a get" --oracle-differs "$FLAG_BUG" -- -d $K -g 1 -c current_limit=0.5 --get uvc_active
    # A time limit: how many lines that is depends on the clock, so the output
    # is kept up to the first sample, and the lines' length not at all.
    emu_options setters=0 period=5
    emu_case "time" --filter '3,$d' --tx-filter 's/^bulk_in 82 [0-9]+ -> 72.*$/bulk_in lines/
/^bulk_out 02 6d6f6e69746f7220300d0a /q' -- -d $K --time 1000ms
    emu_options setters=0 count=1 period=50
    emu_case "time before the first line" --oracle-differs "$TIME_BUG" --no-output -- -d $K --time 100ms
    emu_options setters=0 count=0
    emu_case "time on a silent device" --oracle-bug "$TIME_BUG" --oracle-hangs --no-output --timeout 3 -- -d $K --time 100ms
    emu_options setters=0 count=1
    emu_case "samples 2 of a silent device" --oracle-bug "$SAMPLES_BUG" --oracle-hangs --stderr --filter "$L" --timeout 8 -- -d $K --samples 2

    # What monitoring may send besides readings: all of them as META lines.
    emu_options setters=0 count=6 lines=$(emu_hex 'overtemp\r\n'),$(emu_hex 'undervolt\r\n'),$(emu_hex 'set 1234\r\n'),$(emu_hex 'uvlo 3300\r\n'),$(emu_hex 'uvlo 0\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "lines of every kind" --keep-header -- -d $K --samples 1 -O analog
    emu_case "lines of every kind default" --keep-header -- -d $K --samples 1
    emu_options setters=0 count=3 lines=$(emu_hex 'err Unknown command x\r\n'),$(emu_hex 'nonsense\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "an error line and an unknown one" --stderr --filter "$L" -- -d $K --samples 1 -l 2
    emu_options setters=0 count=2 lines=$(emu_hex '\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "an empty line" -- -d $K --samples 1
    emu_options setters=0 count=2 lines=$(emu_hex 'read 5\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "a short reading" --oracle-bug "$SHORT_BUG" -- -d $K --samples 1
    emu_options setters=0 count=2 lines=$(emu_hex 'read 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789 1 1 1\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "a line too long" --oracle-bug "$SHORT_BUG" -- -d $K --samples 1
    # A line that stops for longer than a read's 250 ms.
    emu_options setters=0 count=2 gap=1:10:40 lines=$(emu_hex 'read 1000 12000 0 0\r\n')
    emu_case "a line with a gap" --oracle-differs "$GAP_BUG" -- -d $K --samples 1

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 chunk=1 count=2
    emu_case "one byte per transfer" -- -d $K --samples 2
    emu_options setters=0 in-stall=1
    emu_case "a stalled read" --keep-header -- -d $K --scan
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # A stale line in the input stops the scan.
    emu_options setters=0 stale=$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "scan a stale line" --oracle-differs "$STALE_BUG" --keep-header -- -d $K --scan
    # An unplug between the scan's two writes, in its 50 ms.
    emu_options setters=0
    emu_case "unplug in the scan" --unplug out=2:1 --timeout 30 --keep-header --no-output -- -d $K --scan
    # An unplug while a setting waits for its answer, and what comes next.
    emu_options answers=0
    # Each C path that writes to a port gone comes here, past a setting it takes
    # for done. oxrok fails at that setting, so its own failed writes are only in
    # the unit tests: no event falls between an answer and the next write.
    emu_case "unplug before the capture" --oracle-bug "$GONE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c current_limit=1 --samples 1
    emu_case "unplug before a set current_limit" --oracle-bug "$GONE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c uvc_threshold=1 -c current_limit=1 --set
    emu_case "unplug before a set enabled" --oracle-bug "$GONE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c current_limit=1 -c enabled=on --set
    emu_case "unplug before a set uvc_threshold" --oracle-bug "$GONE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c current_limit=1 -c uvc_threshold=1 --set
    emu_case "unplug before a get voltage" --oracle-bug "$GONE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c current_limit=1 --get voltage
    emu_case "unplug before a get current" --oracle-bug "$GONE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c current_limit=1 --get current
    # api.c:230 and 253 hand sigrok-cli no value, and it prints what its stack held.
    # It dies, and now and then in GLib, with a stderr no golden can hold: seen on
    # each of these two, 1 run in over 100 under load. So they run on oxrok alone.
    # `uvc` (api.c:246) is the same path, and has not shown it in 55 runs.
    emu_case "unplug before a get current_limit" --no-oracle-only --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c enabled=on --get current_limit
    emu_case "unplug before a get uvc" --oracle-bug "$NOVALUE_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c enabled=on --get uvc
    emu_case "unplug before a get uvc_threshold" --no-oracle-only --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c enabled=on --get uvc_threshold
    # The C's close then fails its write too, and it says nothing of either.
    emu_case "unplug before the close" --oracle-differs "$NOANS_BUG" --unplug model --timeout 15 --expect-fail --stderr --filter "$G" -- -d $K -g 1 -c enabled=on --set
    # No answer to a read: C reads past what g_strsplit gave it.
    emu_options setters=0 answers=0
    emu_case "get voltage no answer" --oracle-bug "$READ_BUG" --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 --expect-fail --stderr -- -d $K -g 1 --get voltage
    emu_case "get current no answer" --oracle-bug "$READ_BUG" --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 --expect-fail --stderr -- -d $K -g 1 --get current
    # The end of a capture from outside.
    emu_options setters=0 count=1
    emu_case "unplug in a capture" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 6 -- -d $K --samples 3
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $K --continuous
    # A library client's stop: monitoring goes on until the close.
    emu_case "lib stop" --libsigrok -- -d $K --continuous --stop-after 1
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=arachnid-labs-re-load-pro:conn=$EMU_TTY
}
