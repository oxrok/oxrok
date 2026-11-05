# An Arachnid Labs Re:load Pro electronic load
# (emu/src/models/arachnid-labs-re-load-pro.rs) on the CDC-ACM shell. Sourced
# by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=arachnid-labs-re-load-pro
EMU_CDC=1
EMU_DRIVER=arachnid-labs-re-load-pro
EMU_GCOV=arachnid-labs-re-load-pro
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
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
    # 1.2 pass though they come before 1.10.
    for v in 1.10 1.11 1.9 1.2 2.0 1.1 1.11x; do
        emu_options setters=0 version=$v
        emu_case "scan version $v" --keep-header -- -d $K --scan
    done
    for v in 1.0 1 0.13 abc; do
        emu_options setters=0 version=$v
        emu_case "scan version $v" --keep-header --no-output -- -d $K --scan
    done
    emu_options setters=0 version=1.0
    emu_case "scan too old says so" --keep-header --stderr -- -d $K --scan -l 3
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
    emu_case "show on a firmware with no uvlo" --keep-header -- -d $K -g 1 --show

    # A setting answers with its value.
    emu_options
    emu_case "set enabled on" --no-output -- -d $K -g 1 -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -g 1 -c enabled=off --set
    emu_case "set current_limit" --no-output -- -d $K -g 1 -c current_limit=1.5 --set
    emu_case "set current_limit rounds in single precision" --no-output -- -d $K -g 1 -c current_limit=1.0005 --set
    emu_case "set current_limit 0" --no-output -- -d $K -g 1 -c current_limit=0 --set
    emu_case "set current_limit max" --no-output -- -d $K -g 1 -c current_limit=6 --set
    emu_case "set current_limit too high" --stderr --expect-fail -- -d $K -g 1 -c current_limit=6.001 --set
    emu_case "set current_limit negative" --stderr --expect-fail -- -d $K -g 1 -c current_limit=-1 --set
    emu_case "set current_limit nan" --no-output -- -d $K -g 1 -c current_limit=nan --set
    emu_case "set uvc_threshold" --no-output -- -d $K -g 1 -c uvc_threshold=3.3 --set
    emu_case "set uvc_threshold 0" --no-output -- -d $K -g 1 -c uvc_threshold=0 --set
    emu_case "set uvc_threshold max" --no-output -- -d $K -g 1 -c uvc_threshold=60 --set
    emu_case "set uvc_threshold too high" --stderr --expect-fail -- -d $K -g 1 -c uvc_threshold=60.001 --set
    emu_case "set uvc_threshold nan" --no-output -- -d $K -g 1 -c uvc_threshold=nan --set
    emu_case "set then get" -- -d $K -g 1 -c current_limit=2.5 --get current_limit
    emu_case "set a read-only key" --stderr --expect-fail -- -d $K -g 1 -c voltage=1 --set
    emu_case "set a group's key with no group" --stderr --expect-fail -- -d $K -c current_limit=1 --set
    emu_case "set several" --no-output -- -d $K -g 1 -c current_limit=1.5:uvc_threshold=3.3:enabled=on --set
    emu_options version=1.8
    emu_case "set uvc_threshold on a firmware with no uvlo" --no-output -- -d $K -g 1 -c uvc_threshold=3.3 --set
    emu_options replies=$(emu_hex "err Unknown command 'on'\r\n")
    emu_case "set enabled refused" --no-output -- -d $K -g 1 -c enabled=on --set
    emu_options answers=0
    emu_case "set enabled no answer" --no-output -- -d $K -g 1 -c enabled=on --set
    emu_case "set current_limit no answer" --no-output -- -d $K -g 1 -c current_limit=1 --set
    emu_options

    # Answers gone wrong, to what asks for a value.
    emu_options setters=0 answers=0
    emu_case "get current_limit no answer" -- -d $K -g 1 --get current_limit
    emu_case "get uvc_threshold no answer" -- -d $K -g 1 --get uvc_threshold
    emu_case "get uvc no answer" -- -d $K -g 1 --get uvc
    emu_options setters=0 replies=$(emu_hex '\r\nset 1500\r\n')
    emu_case "get an empty line first" -- -d $K -g 1 --get current_limit
    emu_options setters=0 replies=$(emu_hex "err Unknown command 'set'\r\n")
    emu_case "get current_limit err" -- -d $K -g 1 --get current_limit
    emu_options setters=0 replies=$(emu_hex "err Unknown command 'read'\r\n")
    emu_case "get voltage err" -- -d $K -g 1 --get voltage
    emu_options setters=0 replies=$(emu_hex 'read 1000 12000\r\n')
    emu_case "get voltage old reply" -- -d $K -g 1 --get voltage
    emu_case "get current old reply" -- -d $K -g 1 --get current
    emu_options setters=0 replies=$(emu_hex 'read x y\r\n')
    emu_case "get voltage garbage" -- -d $K -g 1 --get voltage

    # A capture: monitor 200, lines, monitor 0 at the close. Each case's count
    # holds what comes after the last line it reads.
    emu_options setters=0 set=1500 volts=12345 amps=1499 count=1
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 1 csv" -- -d $K --samples 1 -O csv:header=false:time=false
    emu_case "samples 1 analog" --keep-header -- -d $K --samples 1 -O analog
    emu_case "samples 1 digits spec" -- -d $K --samples 1 -O analog:digits=spec
    emu_options setters=0 set=1500 volts=12345 amps=1499 count=3
    emu_case "samples 3" -- -d $K --samples 3
    emu_case "channel V only" -- -d $K -C V --samples 3
    emu_case "channel I only" -- -d $K -C I --samples 3
    emu_case "channel V only csv" -- -d $K -C V --samples 3 -O csv:header=false:time=false
    emu_case "config limit_samples" -- -d $K -c limit_samples=3
    emu_case "config limit_samples 0" --stderr --expect-fail -- -d $K -c limit_samples=0 --samples 1
    emu_case "config limit_time 0" --stderr --expect-fail -- -d $K -c limit_time=0 --samples 1
    emu_options set=1500 volts=12345 count=1
    emu_case "capture after a set" -- -d $K -g 1 -c current_limit=0.5 --samples 1
    emu_case "capture with enabled off" -- -d $K -g 1 -c enabled=off --samples 1
    # The readings a float holds, from small to the largest in the field.
    emu_options setters=0 count=4 lines=$(emu_hex 'read 1 1 0 0\r\n'),$(emu_hex 'read 3 3 0 0\r\n'),$(emu_hex 'read 6000 60000 123 456\r\n'),$(emu_hex 'read 0 0\r\n')
    emu_case "readings in turn" -- -d $K --samples 4
    # A time limit is looked at only when a line has come.
    emu_options setters=0 count=1 period=50
    emu_case "time" -- -d $K --time 100ms

    # What monitoring may send besides readings: all of them as META lines.
    emu_options setters=0 count=6 lines=$(emu_hex 'overtemp\r\n'),$(emu_hex 'undervolt\r\n'),$(emu_hex 'set 1234\r\n'),$(emu_hex 'uvlo 3300\r\n'),$(emu_hex 'uvlo 0\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "lines of every kind" --keep-header -- -d $K --samples 1 -O analog
    emu_case "lines of every kind default" --keep-header -- -d $K --samples 1
    emu_options setters=0 count=3 lines=$(emu_hex 'err Unknown command x\r\n'),$(emu_hex 'nonsense\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "an error line and an unknown one" --stderr -- -d $K --samples 1 -l 2
    emu_options setters=0 count=2 lines=$(emu_hex '\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "an empty line" -- -d $K --samples 1
    emu_options setters=0 count=2 lines=$(emu_hex 'read 5\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "a short reading" --stderr --expect-fail -- -d $K --samples 1
    emu_options setters=0 count=2 lines=$(emu_hex 'read 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789 1 1 1\r\n'),$(emu_hex 'read 1 2 3 4\r\n')
    emu_case "a line too long" --stderr --expect-fail -- -d $K --samples 1
    # A line that stops for longer than a read's 250 ms.
    emu_options setters=0 count=2 gap=1:10:40 lines=$(emu_hex 'read 1000 12000 0 0\r\n')
    emu_case "a line with a gap" -- -d $K --samples 1

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
    emu_case "scan a stale line" --keep-header --no-output -- -d $K --scan
    # An unplug between the scan's two writes, in its 50 ms.
    emu_options setters=0
    emu_case "unplug in the scan" --unplug out=2:1 --timeout 30 --keep-header --no-output -- -d $K --scan
    # An unplug while a setting waits for its answer, and what comes next.
    emu_options answers=0
    emu_case "unplug before the capture" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c current_limit=1 --samples 1
    emu_case "unplug before a set current_limit" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c enabled=on -c current_limit=1 --set
    emu_case "unplug before a set enabled" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c current_limit=1 -c enabled=on --set
    emu_case "unplug before a set uvc_threshold" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c current_limit=1 -c uvc_threshold=1 --set
    emu_case "unplug before a get voltage" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c current_limit=1 --get voltage
    emu_case "unplug before a get current" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c current_limit=1 --get current
    emu_case "unplug before a get current_limit" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c enabled=on --get current_limit
    emu_case "unplug before a get uvc" --unplug model --timeout 15 --expect-fail --stderr -- -d $K -g 1 -c enabled=on --get uvc
    # The close's write then fails too.
    emu_case "unplug before the close" --unplug model --timeout 15 --no-output -- -d $K -g 1 -c enabled=on --set
    # No answer to a read: C reads past what g_strsplit gave it.
    emu_options setters=0 answers=0
    emu_case "get voltage no answer" --stderr --expect-fail --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 -- -d $K -g 1 --get voltage
    # Lines stop and no limit comes: C waits for ever, and a time limit is not
    # looked at unless a line comes.
    emu_options setters=0 count=1
    emu_case "lib samples 2 of a silent device" --libsigrok --hangs --timeout 3 -- -d $K --samples 2
    emu_options setters=0 count=0
    emu_case "lib time on a silent device" --libsigrok --hangs --no-output --timeout 3 -- -d $K --time 100ms
    emu_options setters=0 count=1
    emu_case "lib stop" --libsigrok -- -d $K --continuous --stop-after 1
    emu_case "lib unplug" --libsigrok --hangs --unplug model --timeout 3 -- -d $K --continuous
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=arachnid-labs-re-load-pro:conn=$EMU_TTY
}
